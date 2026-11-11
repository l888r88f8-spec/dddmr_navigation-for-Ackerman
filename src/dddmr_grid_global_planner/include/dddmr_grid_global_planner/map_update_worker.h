// Copyright 2026 DDDMobileRobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DDDMR_GRID_GLOBAL_PLANNER__MAP_UPDATE_WORKER_H_
#define DDDMR_GRID_GLOBAL_PLANNER__MAP_UPDATE_WORKER_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "dddmr_grid_global_planner/map_snapshot.h"
#include "dddmr_grid_global_planner/metrics.h"
#include "dddmr_grid_global_planner/multiresolution_map.h"
#include "dddmr_grid_global_planner/snapshot_store.h"
#include "dddmr_grid_global_planner/source_observation.h"
#include "dddmr_grid_global_planner/terrain_grid_builder.h"

namespace dddmr_grid_global_planner
{

// One queued map-update event kind. Terrain edits are whole-content
// replacements, source edits carry frozen observations or removals, and expire
// requests are evaluated against the worker's monotonic clock.
enum class MapUpdateRequestKind : std::uint8_t
{
  kTerrainReplacement,
  kSourceObservation,
  kSourceRemoval,
  kExpireSources,
};

struct MapUpdateWorkerOptions
{
  TerrainGridBuilderOptions builder{};
  CoarseTerrainMapOptions coarse{};
  ObstacleDistanceOptions distance{};

  // Maximum age of a dynamic source contribution before ExpireSources removes
  // it. A missing receipt stamp is treated as expired immediately.
  double source_ttl_s{2.0};

  // Maximum age of a static source contribution. A value <= 0 means the static
  // contribution is immortal and is removed only by an explicit removal. This
  // is deliberately separate from the dynamic TTL so a latched static map does
  // not disappear after the dynamic expiry period.
  double static_source_ttl_s{-1.0};

  // Hard bound on queued update events. Complete replacements may coalesce
  // older pending events first, but an exact delta never replaces another
  // event. A submission that would exceed this bound is rejected with
  // kQueueFull before any sequence state is advanced.
  std::size_t max_pending_batches{4};

  // Hard bound on distinct source identifiers in the monotonic sequence
  // highwater registry. The registry survives TTL expiry and explicit removal,
  // so a late packet for a removed source cannot resurrect occupancy. A new
  // source id is rejected with kRejectedCapacity when the registry is full.
  // Zero disables the bound.
  std::size_t max_sources{1024};

  // Hard total record budget across accepted frozen records and pending events.
  // Zero disables the total bound. Exceeding it rejects with kRejectedCapacity
  // before any sequence/queue mutation.
  std::size_t max_records_total{1'000'000};

  // Hard per-source record budget for frozen records. Zero disables the bound.
  std::size_t max_records_per_source{100'000};

  // Publish batching interval. A positive value coalesces dirty source updates
  // into at most one obstacle snapshot per period. Zero disables batching and
  // publishes after each processed event. max_publish_hz is used only when
  // publish_period_s is non-positive.
  double publish_period_s{0.0};
  double max_publish_hz{0.0};

  // Vertical association envelope for an obstacle slab that does not overlap a
  // terrain surface but lies above it. These are explicit observed/configured
  // bounds, not a synthetic vehicle height. A non-finite or non-positive maximum
  // disables non-overlap association, which keeps missing physical-envelope
  // information failing closed (unknown) instead of inventing 2 m ability.
  double association_max_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  double association_min_clearance_m{0.0};

  // Number of historic snapshots retained by the SnapshotStore supplied to the
  // constructor. The store owns the actual bound.
  std::size_t retained_snapshots{4};

  // When true, an occupied/cleared record that cannot be associated with any
  // terrain node contributes nothing instead of being guessed onto a deck.
  bool reject_unknown_sources{true};
};

enum class MapUpdateStatus : std::uint8_t
{
  kAccepted,
  kRejectedInvalid,
  kRejectedStale,
  kRejectedDuplicate,
  // A hard capacity bound (source highwater, pending queue, or record budget)
  // rejected the submission without accepting or dropping any update.
  kRejectedCapacity,
  // The pending queue is at its hard bound; the caller must resync/resubmit.
  kQueueFull,
  // Older pending events for the same source (or the same expiry/terrain kind)
  // were superseded by this accepted event.
  kQueueFullCoalesced,
  kStopped,
};

struct MapUpdateResult
{
  MapUpdateStatus status{MapUpdateStatus::kRejectedInvalid};
  std::string detail;
};

// Single-writer asynchronous map updater.
//
// Thread ownership: Start() creates exactly one owned std::thread and Stop()
// cooperatively joins it; no work is ever detached. Submit* methods only freeze
// and enqueue an event, so ROS callbacks never build terrain or a distance
// field. The worker is the only writer of the immutable snapshot chain.
//
// Shutdown: Stop() sets a stop flag, wakes the queue, and joins the owner
// thread. The thread drains accepted queued events before exiting so an
// accepted update is not silently lost. Submit* after Stop() returns
// kStopped. The destructor calls Stop().
//
// Backpressure: the queue is hard-bounded by max_pending_batches, the source
// identifier registry by max_sources, and frozen records by max_records_total /
// max_records_per_source. A newer complete replacement for the same source_id
// supersedes older pending events for that source; a newer terrain replacement
// supersedes an older pending one. An exact delta is never dropped. If adding an
// event would exceed the hard queue bound the submission returns kQueueFull
// without advancing that source's sequence highwater; the caller must resync and
// resubmit. If a store publication is backpressured (TryPublish false), the
// worker coalesces the newest complete snapshot and retries after the store's
// reclaimer catches up. published_version and the publish callback are only
// advanced after the store actually accepted the snapshot. The publish callback
// runs only after SnapshotStore::TryPublish has returned and released the store
// handoff mutex, and after the queue lock is released, so a slow subscriber can
// neither block producers/search readers nor extend a CommitLatest() critical
// section; it must still return promptly.
class MapUpdateWorker
{
public:
  using PublishCallback =
    std::function<void (std::shared_ptr<const MapSnapshot>)>;
  // Optional per-event/per-publication telemetry observer. It is invoked by the
  // worker thread only after the queue lock and SnapshotStore handoff mutex are
  // released. The observer must return promptly and must not call back into the
  // worker or store.
  using UpdateMetricsCallback = std::function<void (const MapUpdateMetrics &)>;

  // store may be null when only on_publish is used. Both sinks are optional;
  // diagnostics still track published_version() after the store accepts a
  // snapshot. The metrics observer is optional and defaults to no observer, so
  // existing constructors remain source compatible.
  MapUpdateWorker(
    MapUpdateWorkerOptions options, SnapshotStore * store,
    PublishCallback on_publish = {},
    UpdateMetricsCallback on_metrics = {});

  // Stops and joins the owned thread when still running.
  ~MapUpdateWorker();

  MapUpdateWorker(const MapUpdateWorker &) = delete;
  MapUpdateWorker & operator=(const MapUpdateWorker &) = delete;

  // Starts the single owned thread. Idempotent: returns true when the worker is
  // already running. After Stop() a later Start() creates a fresh thread.
  bool Start();

  // Cooperative stop + join. Idempotent and safe from any thread.
  void Stop();

  // Enqueues a frozen terrain replacement. Invalid/incomplete input is rejected
  // synchronously; a content-identical replacement is accepted and later
  // skipped by the worker's content-hash comparison.
  MapUpdateResult SubmitTerrain(TerrainPointObservation observation);

  // Enqueues a frozen source observation. Duplicate and older sequences are
  // rejected synchronously and deterministically. A newer complete replacement
  // coalesces all older pending events for that source_id.
  MapUpdateResult SubmitSource(SourceObservationSnapshot observation);

  // Enqueues removal of one source and all of its pending events.
  MapUpdateResult SubmitSourceRemoval(std::string source_id);

  // Enqueues a TTL sweep at the supplied monotonic time. Older pending sweeps
  // are coalesced into this one.
  MapUpdateResult ExpireSources(std::int64_t now_steady_ns);

  bool running() const noexcept;

  // Diagnostics only; never used by search hot paths.
  std::uint64_t published_version() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__MAP_UPDATE_WORKER_H_
