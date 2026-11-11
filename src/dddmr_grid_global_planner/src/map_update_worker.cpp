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

#include "dddmr_grid_global_planner/map_update_worker.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/distance_field.h"
#include "dddmr_grid_global_planner/obstacle_tile.h"
#include "dddmr_grid_global_planner/source_observation.h"

namespace dddmr_grid_global_planner
{
namespace
{

// Default tile size used by the worker's COW layer builders. Tile size is a
// representation parameter, not a vehicle capability.
constexpr double kDefaultTileSizeM = 3.2;

// Vertical tolerance used when deciding which deck an obstacle slab touches.
// This absorbs small plane-fit offsets without merging distinct decks.
constexpr double kDeckOverlapToleranceM = 0.30;

// Worker poll interval while a store publication is backpressured or while a
// publish-rate deadline is pending. This is a retry cadence, not a guarantee.
constexpr std::chrono::milliseconds kWorkerPollInterval{2};

// Bounded shutdown retry window for an accepted but not yet publishable
// snapshot. The store is expected to be stopped only after the worker joins.
constexpr std::chrono::milliseconds kShutdownPublishRetryWindow{2000};

// Returns the current monotonic time in nanoseconds. The worker uses one clock
// domain for receipt/TTL comparisons; sensor stamps remain diagnostic only.
std::int64_t NowSteadyNs() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

// Exact frozen record identity used by exact-delta add/remove. The input record
// already carries explicit geometry, so exact double comparison is deterministic
// and does not silently merge nearby obstacles.
struct RecordKey
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double min_z{0.0};
  double max_z{0.0};

  bool operator<(const RecordKey & other) const noexcept
  {
    if (x != other.x) {
      return x < other.x;
    }
    if (y != other.y) {
      return y < other.y;
    }
    if (z != other.z) {
      return z < other.z;
    }
    if (min_z != other.min_z) {
      return min_z < other.min_z;
    }
    return max_z < other.max_z;
  }
};

// Builds the exact lookup key for one frozen record.
RecordKey MakeRecordKey(const SourceOccupancyRecord & record) noexcept
{
  return RecordKey{record.point.x, record.point.y, record.point.z, record.min_z,
    record.max_z};
}

// Vertical evidence captured from one observation. It is stored with the record
// so a later terrain revision can recompute headroom against a new deck height
// without retaining the whole source snapshot.
struct RecordEvidence
{
  bool explicit_headroom_known{false};
  double explicit_headroom_m{std::numeric_limits<double>::quiet_NaN()};
  bool observed_bottom_known{false};
  double observed_bottom_z{std::numeric_limits<double>::quiet_NaN()};
  bool observed_top_known{false};
  double observed_top_z{std::numeric_limits<double>::quiet_NaN()};
};

// One inclusive vertical interval used for per-node extent aggregation.
struct VerticalInterval
{
  double min_z{0.0};
  double max_z{0.0};
};

// How a frozen record relates to one terrain surface.
enum class AssociationKind : std::uint8_t
{
  // The record slab overlaps the surface. For an occupied record this is
  // road-level occupancy.
  kSurface = 0,
  // The record slab lies above the surface within the configured envelope. For
  // an occupied record this is ceiling/headroom evidence, not road occupancy.
  kOverhead = 1,
};

// One record-to-node association with its vertical relation.
struct NodeAssociation
{
  TerrainNodeId id{kInvalidTerrainNodeId};
  AssociationKind kind{AssociationKind::kSurface};
};

// One contribution record owned by one source. `nodes` is the current terrain
// association and is rebuilt when the terrain revision changes.
struct SourceRecord
{
  SourceOccupancyRecord geometry{};
  bool occupied{false};
  std::vector<NodeAssociation> nodes;
  RecordEvidence evidence{};
};

// Per-node evidence owned by one source. Counts and per-record maps let
// overlapping records and other sources be recomputed without clearing a node
// to free space or dropping vertical metadata.
struct SourceNodeEvidence
{
  std::uint32_t occupied_count{0};
  std::uint32_t clear_count{0};
  std::map<RecordKey, double> occupied_headrooms;
  std::map<RecordKey, double> clear_headrooms;
  std::map<RecordKey, VerticalInterval> occupied_intervals;
  std::map<RecordKey, VerticalInterval> observed_intervals;
  std::int64_t observation_stamp_ns{0};
  std::int64_t received_steady_ns{0};
};

// One source's complete contribution state. Records are the authority; `nodes`
// is a derived index used to update only affected terrain nodes.
struct SourceState
{
  SourceLayer layer{SourceLayer::kStatic};
  std::uint64_t last_sequence{0};
  bool have_sequence{false};
  std::int64_t observation_stamp_ns{0};
  std::int64_t received_steady_ns{0};
  std::map<RecordKey, SourceRecord> records;
  // Accepted records that could not be associated with any current terrain
  // surface. They survive a later terrain revision and are retried on
  // reassociation instead of being silently lost. Bounded by max_records_*.
  std::map<RecordKey, SourceRecord> pending_records;
  std::unordered_map<TerrainNodeId, SourceNodeEvidence> nodes;
};

// Aggregated evidence for one layer at one terrain node.
struct LayerAggregate
{
  bool occupied{false};
  bool clear{false};
  bool clearance_known{false};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
  bool occupied_extent_known{false};
  double occupied_min_z{std::numeric_limits<double>::quiet_NaN()};
  double occupied_max_z{std::numeric_limits<double>::quiet_NaN()};
  bool observed_extent_known{false};
  double observed_min_z{std::numeric_limits<double>::quiet_NaN()};
  double observed_max_z{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t observation_stamp_ns{0};
  std::int64_t received_steady_ns{0};
};

// Aggregated static/dynamic evidence for one terrain node.
struct NodeAggregate
{
  LayerAggregate static_layer{};
  LayerAggregate dynamic_layer{};
};

// One queued event. optional members avoid default-constructing immutable
// observations that require their full argument list.
struct Request
{
  MapUpdateRequestKind kind{MapUpdateRequestKind::kExpireSources};
  std::optional<TerrainPointObservation> terrain;
  std::optional<SourceObservationSnapshot> source;
  std::string source_id;
  std::uint64_t source_sequence{0};
  std::int64_t now_steady_ns{0};
  // Monotonic instant at which this event was accepted into the pending queue.
  std::int64_t enqueued_steady_ns{0};
};

// Returns true when one layer aggregate carries an explicit state.
bool HasEvidence(const LayerAggregate & layer) noexcept
{
  return layer.occupied || layer.clear;
}

// Returns true when a layer aggregate should publish a clearance value.
bool HasKnownClearance(const LayerAggregate & layer) noexcept
{
  return layer.clearance_known && std::isfinite(layer.clearance_m);
}

// Converts a finite non-negative seconds value to saturated nanoseconds.
std::int64_t SecondsToNanoseconds(double seconds)
{
  const long double wide = static_cast<long double>(seconds) * 1.0e9L;
  if (wide >= static_cast<long double>(
      std::numeric_limits<std::int64_t>::max()))
  {
    return std::numeric_limits<std::int64_t>::max();
  }
  if (wide <= 0.0L) {
    return 0;
  }
  return static_cast<std::int64_t>(wide);
}

}  // namespace

// Private implementation. All queue and worker-only state lives here so the
// public header stays free of internal record types.
struct MapUpdateWorker::Impl
{
  Impl(
    MapUpdateWorkerOptions worker_options, SnapshotStore * snapshot_store,
    PublishCallback publish_callback,
    UpdateMetricsCallback metrics_callback)
  : options(std::move(worker_options)),
    store(snapshot_store),
    on_publish(std::move(publish_callback)),
    on_metrics(std::move(metrics_callback)),
    terrain_builder(options.builder)
  {
    // A positive publish_period_s is authoritative; otherwise an explicit
    // max_publish_hz supplies the period. Zero or invalid values disable
    // batching so existing per-event callers keep their behavior.
    double period_s = options.publish_period_s;
    if (!(period_s > 0.0) || !std::isfinite(period_s)) {
      period_s = 0.0;
      if (options.max_publish_hz > 0.0 &&
        std::isfinite(options.max_publish_hz))
      {
        period_s = 1.0 / options.max_publish_hz;
      }
    }
    publish_period_ns_ = SecondsToNanoseconds(period_s);
  }

  // Starts one owned thread. Called with lifecycle_mutex held.
  bool Start()
  {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    if (running.load(std::memory_order_acquire)) {
      return true;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      stop = false;
    }
    thread = std::thread([this]() {Run();});
    running.store(true, std::memory_order_release);
    return true;
  }

  // Stops and joins the owned thread. Pending accepted events are drained and
  // a pending publication is retried for a bounded window before exit.
  void Stop()
  {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    if (!running.load(std::memory_order_acquire)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      stop = true;
    }
    queue_cv.notify_all();
    if (thread.joinable()) {
      thread.join();
    }
    running.store(false, std::memory_order_release);
  }

  // Main worker loop. The queue lock is held only while waiting for or popping
  // one event. Dirty updates may batch until the publish-rate deadline.
  void Run()
  {
    while (true) {
      Request request;
      bool have_request = false;
      bool stopping = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        if (pending.empty()) {
          if (stop) {
            stopping = true;
          } else if (!dirty_nodes.empty() ||
            pending_publish_ != nullptr)
          {
            // A backpressured publication or a scheduled rate-limited flush
            // needs a periodic retry wakeup.
            queue_cv.wait_for(
              lock, kWorkerPollInterval, [this]() {
                return stop || !pending.empty();
              });
          } else {
            queue_cv.wait(
              lock, [this]() {
                return stop || !pending.empty();
              });
          }
        }
        if (!stopping && !pending.empty()) {
          request = std::move(pending.front());
          pending.pop_front();
          have_request = true;
        }
      }
      if (have_request) {
        MapUpdateMetrics event_metrics;
        event_metrics.started_steady_ns = NowSteadyNs();
        event_metrics.enqueued_steady_ns = request.enqueued_steady_ns;
        event_metrics.source_id = request.source_id;
        event_metrics.source_sequence = request.source_sequence;
        switch (request.kind) {
          case MapUpdateRequestKind::kTerrainReplacement:
            event_metrics.kind =
              MapUpdateMetrics::Kind::kTerrainReplacement;
            break;
          case MapUpdateRequestKind::kSourceObservation:
            event_metrics.kind =
              MapUpdateMetrics::Kind::kSourceObservation;
            break;
          case MapUpdateRequestKind::kSourceRemoval:
            event_metrics.kind = MapUpdateMetrics::Kind::kSourceRemoval;
            break;
          case MapUpdateRequestKind::kExpireSources:
            event_metrics.kind = MapUpdateMetrics::Kind::kExpireSources;
            break;
        }
        try {
          Process(std::move(request), &event_metrics);
        } catch (const std::exception & exception) {
          event_metrics.detail = exception.what();
        } catch (...) {
          // A malformed event must not kill the single writer. The previous
          // immutable snapshot remains published and later events still run.
          event_metrics.detail = "unknown worker exception";
        }
        event_metrics.completed_steady_ns = NowSteadyNs();
        event_metrics.queue_wait_ms =
          SteadyDurationMs(
          event_metrics.enqueued_steady_ns,
          event_metrics.started_steady_ns);
        event_metrics.total_ms =
          SteadyDurationMs(
          event_metrics.started_steady_ns,
          event_metrics.completed_steady_ns);
        event_metrics.resources = ReadProcessResourceSnapshot();
        EmitMetrics(event_metrics);
      }
      MaybePublish();
      if (stopping) {
        FlushForShutdown();
        break;
      }
    }
  }

  // Dispatches one queued event and records its actual process phases into the
  // caller-owned local metric. The metric is emitted only after this returns.
  void Process(Request request, MapUpdateMetrics * metrics)
  {
    switch (request.kind) {
      case MapUpdateRequestKind::kTerrainReplacement:
        if (request.terrain.has_value()) {
          ProcessTerrain(*request.terrain, metrics);
        }
        break;
      case MapUpdateRequestKind::kSourceObservation:
        if (request.source.has_value()) {
          ProcessSource(*request.source, metrics);
        }
        break;
      case MapUpdateRequestKind::kSourceRemoval:
        ProcessSourceRemoval(request.source_id, metrics);
        break;
      case MapUpdateRequestKind::kExpireSources:
        ProcessExpire(request.now_steady_ns, metrics);
        break;
    }
  }

  // Delivers one immutable local metric object to the optional observer. The
  // observer runs on the worker thread only after queue/store locks are
  // released; a throwing observer is isolated from the map writer.
  void EmitMetrics(const MapUpdateMetrics & metrics)
  {
    if (!on_metrics) {
      return;
    }
    try {
      on_metrics(metrics);
    } catch (...) {
      // Telemetry must never break map publication or ingestion.
    }
  }

  // Builds a fresh terrain baseline only when the content hash changed. Old
  // source records are re-associated to the new node IDs by fine cell, and any
  // observations deferred before the first terrain (or for a matching future
  // revision) are replayed before the baseline is published.
  void ProcessTerrain(
    const TerrainPointObservation & observation,
    MapUpdateMetrics * metrics)
  {
    const auto terrain_start = std::chrono::steady_clock::now();
    const TerrainBuildResult result =
      terrain_builder.Build(observation, NowSteadyNs());
    if (metrics != nullptr) {
      metrics->terrain = result.metrics;
      metrics->input_received_steady_ns = result.metrics.received_steady_ns;
      metrics->contribution_ms = DurationToMs(
        std::chrono::steady_clock::now() - terrain_start);
    }
    if (!result.ok()) {
      if (metrics != nullptr) {
        metrics->detail = result.detail;
      }
      return;
    }
    if (terrain != nullptr &&
      result.snapshot->content_hash() == terrain->content_hash())
    {
      latest_terrain_revision.store(
        result.snapshot->revision(),
        std::memory_order_release);
      if (metrics != nullptr) {
        metrics->detail = "content unchanged";
      }
      return;
    }
    terrain = result.snapshot;
    const auto coarse_start = std::chrono::steady_clock::now();
    coarse = OverlayCoarseTerrain(nullptr, terrain, {}, options.coarse);
    if (metrics != nullptr) {
      metrics->coarse_ms = DurationToMs(
        std::chrono::steady_clock::now() - coarse_start);
    }
    latest_terrain_revision.store(
      terrain->revision(),
      std::memory_order_release);
    ReassociateAllSources();
    ReplayDeferredObservations();

    const auto layers_start = std::chrono::steady_clock::now();
    const std::uint64_t next_version = version + 1;
    ObstacleTileLayer new_static;
    ObstacleTileLayer new_dynamic;
    BuildFullLayers(next_version, &new_static, &new_dynamic);
    if (metrics != nullptr) {
      metrics->cow_ms = DurationToMs(
        std::chrono::steady_clock::now() - layers_start);
    }
    static_layer = std::move(new_static);
    dynamic_layer = std::move(new_dynamic);
    version = next_version;
    dirty_nodes.clear();

    // Empty changed_cells is deliberate: a new terrain pointer makes the
    // previous field stale, so UpdateIncremental performs a full rebuild.
    const auto distance_start = std::chrono::steady_clock::now();
    std::shared_ptr<const ObstacleDistanceField> new_field =
      ObstacleDistanceField::UpdateIncremental(
      field, terrain, ObstacleLayer(static_layer),
      ObstacleLayer(dynamic_layer), {}, options.distance);
    if (metrics != nullptr) {
      metrics->distance_ms = DurationToMs(
        std::chrono::steady_clock::now() - distance_start);
    }
    if (new_field == nullptr || !new_field->ready()) {
      return;
    }
    const std::shared_ptr<const MapSnapshot> snapshot =
      MapSnapshot::CreateWithPinnedField(
      terrain, coarse, ObstacleLayer(static_layer),
      ObstacleLayer(dynamic_layer), version, new_field);
    field = std::move(new_field);
    pending_publish_ = snapshot;
    if (metrics != nullptr) {
      metrics->version = version;
      metrics->snapshot_ms = DurationToMs(
        std::chrono::steady_clock::now() - terrain_start);
    }
    static_cast<void>(TryFlushPending(metrics));
  }

  // Applies one frozen source observation. Sequence ordering is also enforced by
  // SubmitSource; this check protects replayed/coalesced processing. An
  // observation whose terrain is not available yet is preserved as pending
  // state instead of being silently lost after Submit already accepted it.
  void ProcessSource(
    const SourceObservationSnapshot & observation,
    MapUpdateMetrics * metrics)
  {
    const auto source_start = std::chrono::steady_clock::now();
    auto finish = [&]() {
        if (metrics != nullptr) {
          metrics->contribution_ms = DurationToMs(
            std::chrono::steady_clock::now() - source_start);
        }
      };
    if (metrics != nullptr) {
      metrics->input_received_steady_ns = observation.received_steady_ns();
    }
    if (terrain == nullptr) {
      DeferObservation(observation);
      finish();
      return;
    }
    if (observation.terrain_revision() != 0 &&
      observation.terrain_revision() != terrain->revision())
    {
      // The observation was produced for another terrain revision. Keep it until
      // a matching revision arrives; never guess node IDs.
      DeferObservation(observation);
      finish();
      return;
    }
    static_cast<void>(ApplySourceObservation(observation, metrics));
    finish();
  }

  // Applies a frozen observation against the current terrain. Returns true when
  // at least one node aggregate changed.
  bool ApplySourceObservation(
    const SourceObservationSnapshot & observation,
    MapUpdateMetrics * metrics = nullptr)
  {
    if (terrain == nullptr) {
      return false;
    }
    auto source_iterator = sources.find(observation.source_id());
    if (source_iterator == sources.end()) {
      SourceState fresh;
      fresh.layer = observation.layer();
      source_iterator = sources.emplace(
        observation.source_id(),
        std::move(fresh))
        .first;
    }
    SourceState & state = source_iterator->second;
    if (state.have_sequence &&
      observation.source_sequence() <= state.last_sequence)
    {
      return false;
    }
    if (observation.kind() == SourceUpdateKind::kExactDelta &&
      state.have_sequence && state.layer != observation.layer())
    {
      return false;
    }

    // Advance source-level stamps before adding records so node evidence carries
    // the newest observation time.
    state.observation_stamp_ns =
      std::max(state.observation_stamp_ns, observation.sensor_stamp_ns());
    state.received_steady_ns =
      std::max(state.received_steady_ns, observation.received_steady_ns());

    std::unordered_set<TerrainNodeId> changed;
    if (observation.kind() == SourceUpdateKind::kCompleteReplacement) {
      // Remove the old contribution exactly. The new union is then added, so an
      // overlap is not double-counted and a vanished record is not left behind.
      std::vector<RecordKey> old_keys;
      old_keys.reserve(state.records.size());
      for (const auto & entry : state.records) {
        old_keys.push_back(entry.first);
      }
      for (const RecordKey & key : old_keys) {
        RemoveSourceRecord(&state, observation.source_id(), key, &changed);
      }
      // A complete replacement supersedes any older unassociated records too.
      state.pending_records.clear();
      // The layer may change only through a complete replacement. Set it after
      // the old records are gone so transient recomputation cannot mix layers.
      state.layer = observation.layer();
      for (const SourceOccupancyRecord & record : observation.occupied()) {
        AddSourceRecord(&state, observation, record, true, &changed, nullptr);
      }
      for (const SourceOccupancyRecord & record : observation.cleared()) {
        AddSourceRecord(&state, observation, record, false, &changed, nullptr);
      }
    } else {
      // Exact delta: listed cleared records remove matching evidence; a clear
      // that does not match an existing record is still an explicit clear.
      for (const SourceOccupancyRecord & record : observation.cleared()) {
        const RecordKey key = MakeRecordKey(record);
        if (state.records.find(key) != state.records.end()) {
          RemoveSourceRecord(&state, observation.source_id(), key, &changed);
        } else {
          AddSourceRecord(&state, observation, record, false, &changed, nullptr);
        }
      }
      for (const SourceOccupancyRecord & record : observation.occupied()) {
        AddSourceRecord(&state, observation, record, true, &changed, nullptr);
      }
    }

    // Every valid known volume contributes explicit clear coverage on the
    // surfaces it actually covers. A point-only observation never rasterizes a
    // bounding box as free.
    RasterizeKnownVolumes(&state, observation, &changed, metrics);

    state.last_sequence = observation.source_sequence();
    state.have_sequence = true;
    if (!changed.empty()) {
      dirty_nodes.insert(changed.begin(), changed.end());
      return true;
    }
    return false;
  }

  // Preserves an accepted observation until a matching terrain revision exists.
  // The sequence highwater and record budget already account for it.
  void DeferObservation(const SourceObservationSnapshot & observation)
  {
    std::vector<SourceObservationSnapshot> & deferred =
      deferred_sources[observation.source_id()];
    if (observation.kind() == SourceUpdateKind::kCompleteReplacement) {
      deferred.clear();
    }
    deferred.push_back(observation);
  }

  // Replays deferred observations whose revision is unknown or matches the
  // current terrain. Non-matching observations stay pending for a later
  // revision instead of being applied to the wrong node IDs.
  void ReplayDeferredObservations()
  {
    if (terrain == nullptr) {
      return;
    }
    for (auto iterator = deferred_sources.begin();
      iterator != deferred_sources.end(); )
    {
      std::vector<SourceObservationSnapshot> remaining;
      for (SourceObservationSnapshot & deferred : iterator->second) {
        if (deferred.terrain_revision() == 0 ||
          deferred.terrain_revision() == terrain->revision())
        {
          static_cast<void>(ApplySourceObservation(deferred));
        } else {
          remaining.push_back(std::move(deferred));
        }
      }
      if (remaining.empty()) {
        iterator = deferred_sources.erase(iterator);
      } else {
        iterator->second = std::move(remaining);
        ++iterator;
      }
    }
  }

  // Removes one source and recomputes only the nodes it touched.
  void ProcessSourceRemoval(
    const std::string & source_id,
    MapUpdateMetrics * metrics)
  {
    const auto removal_start = std::chrono::steady_clock::now();
    std::unordered_set<TerrainNodeId> changed;
    RemoveSourceInternal(source_id, &changed);
    if (!changed.empty()) {
      dirty_nodes.insert(changed.begin(), changed.end());
    }
    if (metrics != nullptr) {
      metrics->contribution_ms = DurationToMs(
        std::chrono::steady_clock::now() - removal_start);
    }
  }

  // Expires dynamic sources and any finite-TTL static sources. Static sources
  // are immortal by default. Expiry removes the contribution (UNKNOWN), never
  // converts missing evidence into clear, and never clears a static layer.
  void ProcessExpire(
    std::int64_t now_steady_ns,
    MapUpdateMetrics * metrics)
  {
    const auto expire_start = std::chrono::steady_clock::now();
    std::vector<std::string> expired;
    for (const auto & entry : sources) {
      const SourceState & state = entry.second;
      double ttl_s = 0.0;
      if (!TtlSecondsForLayer(state.layer, &ttl_s)) {
        continue;
      }
      if (IsExpired(state.received_steady_ns, now_steady_ns, ttl_s)) {
        expired.push_back(entry.first);
      }
    }
    std::unordered_set<TerrainNodeId> changed;
    for (const std::string & source_id : expired) {
      RemoveSourceInternal(source_id, &changed);
    }

    // Deferred observations obey the same per-layer TTL so a contribution
    // waiting for a terrain cannot live forever.
    for (auto iterator = deferred_sources.begin();
      iterator != deferred_sources.end(); )
    {
      std::vector<SourceObservationSnapshot> remaining;
      for (SourceObservationSnapshot & deferred : iterator->second) {
        double ttl_s = 0.0;
        if (!TtlSecondsForLayer(deferred.layer(), &ttl_s)) {
          remaining.push_back(std::move(deferred));
          continue;
        }
        if (IsExpired(deferred.received_steady_ns(), now_steady_ns, ttl_s)) {
          continue;
        }
        remaining.push_back(std::move(deferred));
      }
      if (remaining.empty()) {
        iterator = deferred_sources.erase(iterator);
      } else {
        iterator->second = std::move(remaining);
        ++iterator;
      }
    }

    if (!changed.empty()) {
      dirty_nodes.insert(changed.begin(), changed.end());
    }
    if (metrics != nullptr) {
      metrics->contribution_ms = DurationToMs(
        std::chrono::steady_clock::now() - expire_start);
      metrics->dirty_nodes = dirty_nodes.size();
    }
  }

  // Returns false when the layer is immortal (<= 0 TTL) or the TTL is invalid.
  bool TtlSecondsForLayer(SourceLayer layer, double * ttl_s) const
  {
    if (ttl_s == nullptr) {
      return false;
    }
    double value = layer == SourceLayer::kStatic ?
      options.static_source_ttl_s :
      options.source_ttl_s;
    if (!std::isfinite(value)) {
      value = layer == SourceLayer::kStatic ? -1.0 : 0.0;
    }
    if (value <= 0.0) {
      // A static source with a non-positive TTL is explicitly immortal. A
      // dynamic source with a non-positive TTL expires on the next sweep after
      // its receipt, which keeps the documented "0 disables waiting" behavior.
      if (layer == SourceLayer::kStatic) {
        return false;
      }
      value = 0.0;
    }
    *ttl_s = value;
    return true;
  }

  // Returns true when a contribution is older than the supplied TTL.
  bool IsExpired(
    std::int64_t received_ns, std::int64_t now_ns,
    double ttl_s) const
  {
    if (received_ns <= 0) {
      return true;
    }
    const std::int64_t ttl_ns = SecondsToNanoseconds(ttl_s);
    return now_ns > received_ns && now_ns - received_ns > ttl_ns;
  }

  // Removes every record of one source and unregisters its node evidence. The
  // sequence highwater is deliberately retained so a late packet stays stale.
  void RemoveSourceInternal(
    const std::string & source_id,
    std::unordered_set<TerrainNodeId> * changed)
  {
    deferred_sources.erase(source_id);
    const auto source_iterator = sources.find(source_id);
    if (source_iterator == sources.end()) {
      ReleaseSubmittedRecordBudgetIfIdle(source_id);
      return;
    }
    std::vector<RecordKey> keys;
    keys.reserve(source_iterator->second.records.size());
    for (const auto & entry : source_iterator->second.records) {
      keys.push_back(entry.first);
    }
    for (const RecordKey & key : keys) {
      RemoveSourceRecord(&source_iterator->second, source_id, key, changed);
    }
    sources.erase(source_iterator);
    ReleaseSubmittedRecordBudgetIfIdle(source_id);
  }

  // Adds one occupied or cleared record and updates every associated node.
  // `override_evidence` is used by volume rasterization, where the evidence is
  // derived from the covering volume rather than the record point alone. A
  // record that cannot be associated is retained in pending_records so a later
  // terrain revision can associate it instead of losing accepted evidence.
  void AddSourceRecord(
    SourceState * state,
    const SourceObservationSnapshot & observation,
    const SourceOccupancyRecord & geometry, bool occupied,
    std::unordered_set<TerrainNodeId> * changed,
    const RecordEvidence * override_evidence)
  {
    const RecordKey key = MakeRecordKey(geometry);
    if (state->records.find(key) != state->records.end()) {
      // Overlapping add replaces the previous record with the same geometry.
      RemoveSourceRecord(state, observation.source_id(), key, changed);
    }
    state->pending_records.erase(key);
    SourceRecord record;
    record.geometry = geometry;
    record.occupied = occupied;
    record.evidence = override_evidence != nullptr ?
      *override_evidence :
      BuildRecordEvidence(observation, geometry);
    record.nodes = AssociateRecord(geometry);
    if (record.nodes.empty()) {
      // No surface could be associated yet. Keep the frozen record for a future
      // terrain revision; reject_unknown_sources must not guess a deck.
      AddPendingRecord(state, key, std::move(record));
      return;
    }
    InstallAssociatedRecord(
      state, observation.source_id(), key,
      std::move(record), changed);
  }

  // Retains one unassociated frozen record within the configured bounds.
  void AddPendingRecord(
    SourceState * state, const RecordKey & key,
    SourceRecord record)
  {
    state->pending_records.erase(key);
    if (options.max_records_per_source > 0 &&
      state->records.size() + state->pending_records.size() >=
      options.max_records_per_source)
    {
      return;
    }
    if (options.max_records_total > 0 &&
      worker_record_count_ + state->pending_records.size() >=
      options.max_records_total)
    {
      return;
    }
    state->pending_records.emplace(key, std::move(record));
  }

  // Installs one already-associated record into a source's active map and
  // updates every associated node's evidence.
  void InstallAssociatedRecord(
    SourceState * state, const std::string & source_id,
    const RecordKey & key, SourceRecord record,
    std::unordered_set<TerrainNodeId> * changed)
  {
    if (state->records.find(key) != state->records.end()) {
      RemoveSourceRecord(state, source_id, key, changed);
    }
    const double interval_min =
      std::min(record.geometry.min_z, record.geometry.max_z);
    const double interval_max =
      std::max(record.geometry.min_z, record.geometry.max_z);
    for (const NodeAssociation & association : record.nodes) {
      SourceNodeEvidence & evidence = state->nodes[association.id];
      if (record.occupied &&
        association.kind == AssociationKind::kSurface)
      {
        ++evidence.occupied_count;
        evidence.occupied_intervals[key] =
          VerticalInterval{interval_min, interval_max};
        double headroom_m = std::numeric_limits<double>::quiet_NaN();
        if (ClearanceForNode(record, association.id, &headroom_m)) {
          evidence.occupied_headrooms[key] = headroom_m;
        }
      } else if (record.occupied) {
        // Overhead/ceiling evidence: the road stays clear, but the known slab
        // interval and its clearance are retained for body-envelope checks.
        ++evidence.clear_count;
        evidence.occupied_intervals[key] =
          VerticalInterval{interval_min, interval_max};
        double headroom_m = std::numeric_limits<double>::quiet_NaN();
        if (ClearanceForNode(record, association.id, &headroom_m)) {
          evidence.clear_headrooms[key] = headroom_m;
        }
      } else {
        ++evidence.clear_count;
        if (record.evidence.observed_bottom_known &&
          record.evidence.observed_top_known)
        {
          evidence.observed_intervals[key] = VerticalInterval{
            record.evidence.observed_bottom_z,
            record.evidence.observed_top_z};
        }
        double headroom_m = std::numeric_limits<double>::quiet_NaN();
        if (ClearanceForNode(record, association.id, &headroom_m)) {
          evidence.clear_headrooms[key] = headroom_m;
        }
      }
      evidence.observation_stamp_ns =
        std::max(
        evidence.observation_stamp_ns,
        state->observation_stamp_ns);
      evidence.received_steady_ns =
        std::max(evidence.received_steady_ns, state->received_steady_ns);
      node_sources[association.id].insert(source_id);
      RecomputeNode(association.id, changed);
    }
    state->records.emplace(key, std::move(record));
    ++worker_record_count_;
  }

  // Retries every pending unassociated record against the current terrain.
  void RetryPendingRecords(
    SourceState * state, const std::string & source_id,
    std::unordered_set<TerrainNodeId> * changed)
  {
    for (auto iterator = state->pending_records.begin();
      iterator != state->pending_records.end(); )
    {
      SourceRecord record = std::move(iterator->second);
      record.nodes = AssociateRecord(record.geometry);
      if (record.nodes.empty()) {
        iterator->second = std::move(record);
        ++iterator;
        continue;
      }
      const RecordKey key = iterator->first;
      iterator = state->pending_records.erase(iterator);
      InstallAssociatedRecord(
        state, source_id, key, std::move(record),
        changed);
    }
  }

  // Rasterizes every valid known volume into explicit clear coverage on the
  // terrain surfaces whose deck Z lies inside the observed free vertical extent.
  // This never infers free space from a bounding box of occupied points. An
  // occupied slab belonging to the same source that overlaps the volume is
  // never cleared through: the cell stays occupied/unknown instead of gaining
  // contradictory free evidence from a coarse free-volume record.
  void RasterizeKnownVolumes(
    SourceState * state, const SourceObservationSnapshot & observation,
    std::unordered_set<TerrainNodeId> * changed, MapUpdateMetrics * metrics)
  {
    if (terrain == nullptr) {
      return;
    }
    const std::size_t volume_cap =
      options.max_records_per_source > 0 ?
      options.max_records_per_source :
      1'000'000U;
    // Aggregate every occupied slab of this source by fine cell before any
    // clear rasterization. Pending (unassociated) occupied records are included
    // because they are still observed obstacles even when the current terrain
    // has no matching deck.
    using CellKey = std::pair<std::int64_t, std::int64_t>;
    std::map<CellKey, std::vector<VerticalInterval>> occupied_slabs;
    auto collect_occupied = [&](const SourceRecord & record) {
        if (!record.occupied) {
          return;
        }
        const auto cell = FineCellOf(
          record.geometry.point, terrain->origin_x(),
          terrain->origin_y(), terrain->resolution());
        if (!cell.has_value()) {
          return;
        }
        occupied_slabs[CellKey{cell->x, cell->y}].push_back(
          VerticalInterval{std::min(
              record.geometry.min_z,
              record.geometry.max_z),
            std::max(
              record.geometry.min_z,
              record.geometry.max_z)});
      };
    for (const auto & entry : state->records) {
      collect_occupied(entry.second);
    }
    for (const auto & entry : state->pending_records) {
      collect_occupied(entry.second);
    }
    // Bound the total traversal for this observation, including empty cells.
    // Resetting this budget per volume would permit a large list of empty
    // volumes to multiply the configured work limit without adding records.
    std::size_t visited = 0;
    bool budget_exhausted = false;
    for (const ObservedVolume & volume : observation.known_volumes()) {
      if (budget_exhausted) {
        break;
      }
      if (!volume.known || !volume.free_space ||
        !volume.min.IsFinite() || !volume.max.IsFinite())
      {
        continue;
      }
      const auto minimum_cell =
        FineCellOf(
        volume.min, terrain->origin_x(), terrain->origin_y(),
        terrain->resolution());
      const auto maximum_cell =
        FineCellOf(
        volume.max, terrain->origin_x(), terrain->origin_y(),
        terrain->resolution());
      if (!minimum_cell.has_value() || !maximum_cell.has_value()) {
        continue;
      }
      const std::int64_t min_x = std::min(minimum_cell->x, maximum_cell->x);
      const std::int64_t max_x = std::max(minimum_cell->x, maximum_cell->x);
      const std::int64_t min_y = std::min(minimum_cell->y, maximum_cell->y);
      const std::int64_t max_y = std::max(minimum_cell->y, maximum_cell->y);
      const double volume_min_z = std::min(volume.min.z, volume.max.z);
      const double volume_max_z = std::max(volume.min.z, volume.max.z);
      for (std::int64_t x = min_x; x <= max_x && !budget_exhausted; ++x) {
        for (std::int64_t y = min_y; y <= max_y; ++y) {
          if (visited >= volume_cap ||
            (options.max_records_total > 0 &&
            worker_record_count_ >= options.max_records_total))
          {
            budget_exhausted = true;
            break;
          }
          ++visited;
          const std::vector<TerrainNodeId> * node_ids =
            terrain->FindNodesInCell(FineCellIndex{x, y});
          if (node_ids == nullptr) {
            continue;
          }
          for (const TerrainNodeId node_id : *node_ids) {
            const TerrainNode * node = terrain->FindNode(node_id);
            if (node == nullptr) {
              continue;
            }
            const Point3 center = FineCellCenter(
              node->cell, terrain->origin_x(), terrain->origin_y(),
              terrain->resolution());
            if (center.x < volume.min.x - 1.0e-9 ||
              center.x > volume.max.x + 1.0e-9 ||
              center.y < volume.min.y - 1.0e-9 ||
              center.y > volume.max.y + 1.0e-9)
            {
              continue;
            }
            const double deck_z = node->representative.z;
            if (!std::isfinite(deck_z) || deck_z < volume_min_z - 1.0e-9 ||
              deck_z > volume_max_z + 1.0e-9)
            {
              continue;
            }
            // Do not add clear evidence through a slab this same source
            // observed occupied. Unknown/elevated slabs therefore remain
            // prohibited candidates instead of being erased by a coarse free
            // volume whose XY footprint covers them.
            const auto occupied_iterator =
              occupied_slabs.find(CellKey{node->cell.x, node->cell.y});
            if (occupied_iterator != occupied_slabs.end()) {
              bool occupied_overlap = false;
              for (const VerticalInterval & slab :
                occupied_iterator->second)
              {
                if (slab.max_z >= volume_min_z - 1.0e-9 &&
                  slab.min_z <= volume_max_z + 1.0e-9)
                {
                  occupied_overlap = true;
                  break;
                }
              }
              if (occupied_overlap) {
                continue;
              }
            }
            SourceOccupancyRecord clear_record;
            clear_record.point = Point3{center.x, center.y, volume_min_z};
            clear_record.min_z = volume_min_z;
            clear_record.max_z = volume_max_z;
            RecordEvidence evidence;
            evidence.observed_bottom_known = true;
            evidence.observed_bottom_z = volume_min_z;
            evidence.observed_top_known = true;
            evidence.observed_top_z = volume_max_z;
            if (volume.headroom_known &&
              std::isfinite(volume.headroom_m))
            {
              evidence.explicit_headroom_known = true;
              evidence.explicit_headroom_m = volume.headroom_m;
            }
            AddSourceRecord(
              state, observation, clear_record, false, changed,
              &evidence);
            if (options.max_records_per_source > 0 &&
              state->records.size() >= options.max_records_per_source)
            {
              budget_exhausted = true;
              break;
            }
          }
        }
      }
    }
    if (budget_exhausted && metrics != nullptr) {
      if (!metrics->detail.empty()) {
        metrics->detail += "; ";
      }
      metrics->detail +=
        "free-volume traversal budget exhausted; uncovered cells remain "
        "unknown";
    }
  }

  // Removes one record and recomputes surviving contributions at each node.
  void RemoveSourceRecord(
    SourceState * state, const std::string & source_id,
    const RecordKey & key,
    std::unordered_set<TerrainNodeId> * changed)
  {
    const auto record_iterator = state->records.find(key);
    if (record_iterator == state->records.end()) {
      return;
    }
    const SourceRecord & record = record_iterator->second;
    for (const NodeAssociation & association : record.nodes) {
      const auto node_iterator = state->nodes.find(association.id);
      if (node_iterator != state->nodes.end()) {
        if (record.occupied &&
          association.kind == AssociationKind::kSurface)
        {
          if (node_iterator->second.occupied_count > 0) {
            --node_iterator->second.occupied_count;
          }
          node_iterator->second.occupied_headrooms.erase(key);
          node_iterator->second.observed_intervals.erase(key);
        } else if (record.occupied) {
          if (node_iterator->second.clear_count > 0) {
            --node_iterator->second.clear_count;
          }
          node_iterator->second.clear_headrooms.erase(key);
        } else if (node_iterator->second.clear_count > 0) {
          --node_iterator->second.clear_count;
          node_iterator->second.clear_headrooms.erase(key);
          node_iterator->second.observed_intervals.erase(key);
        }
        if (record.occupied) {
          node_iterator->second.occupied_intervals.erase(key);
        }
        if (node_iterator->second.occupied_count == 0 &&
          node_iterator->second.clear_count == 0)
        {
          state->nodes.erase(node_iterator);
          const auto source_set = node_sources.find(association.id);
          if (source_set != node_sources.end()) {
            source_set->second.erase(source_id);
            if (source_set->second.empty()) {
              node_sources.erase(source_set);
              node_aggregates.erase(association.id);
            }
          }
        }
        RecomputeNode(association.id, changed);
      }
    }
    state->records.erase(record_iterator);
    if (worker_record_count_ > 0) {
      --worker_record_count_;
    }
  }

  // Recomputes one node aggregate from all surviving source contributions.
  void RecomputeNode(
    TerrainNodeId node_id,
    std::unordered_set<TerrainNodeId> * changed)
  {
    if (changed != nullptr) {
      changed->insert(node_id);
    }
    const auto sources_at_node = node_sources.find(node_id);
    if (sources_at_node == node_sources.end() ||
      sources_at_node->second.empty())
    {
      node_aggregates.erase(node_id);
      return;
    }
    NodeAggregate aggregate;
    bool any_evidence = false;
    for (const std::string & source_id : sources_at_node->second) {
      const auto source_iterator = sources.find(source_id);
      if (source_iterator == sources.end()) {
        continue;
      }
      const auto node_iterator =
        source_iterator->second.nodes.find(node_id);
      if (node_iterator == source_iterator->second.nodes.end()) {
        continue;
      }
      const SourceNodeEvidence & evidence = node_iterator->second;
      if (evidence.occupied_count == 0 && evidence.clear_count == 0) {
        continue;
      }
      LayerAggregate & layer =
        source_iterator->second.layer == SourceLayer::kStatic ?
        aggregate.static_layer :
        aggregate.dynamic_layer;
      if (evidence.occupied_count > 0) {
        layer.occupied = true;
      }
      if (evidence.clear_count > 0) {
        layer.clear = true;
      }
      // Surface occupancy clearance wins when the node is occupied; otherwise
      // clear-volume headroom is used. Missing evidence never implies infinity.
      if (evidence.occupied_count > 0 &&
        !evidence.occupied_headrooms.empty())
      {
        MergeMinimumHeadroom(evidence.occupied_headrooms, &layer);
      } else if (evidence.occupied_count == 0 &&
        !evidence.clear_headrooms.empty())
      {
        MergeMinimumHeadroom(evidence.clear_headrooms, &layer);
      }
      for (const auto & interval : evidence.occupied_intervals) {
        MergeInterval(
          interval.second, &layer.occupied_extent_known,
          &layer.occupied_min_z, &layer.occupied_max_z);
      }
      for (const auto & interval : evidence.observed_intervals) {
        MergeInterval(
          interval.second, &layer.observed_extent_known,
          &layer.observed_min_z, &layer.observed_max_z);
      }
      layer.observation_stamp_ns =
        std::max(
        layer.observation_stamp_ns,
        evidence.observation_stamp_ns);
      layer.received_steady_ns =
        std::max(layer.received_steady_ns, evidence.received_steady_ns);
      any_evidence = true;
    }
    if (!any_evidence) {
      node_aggregates.erase(node_id);
      return;
    }
    node_aggregates[node_id] = aggregate;
  }

  // Merges the smallest known headroom into a layer aggregate.
  static void MergeMinimumHeadroom(
    const std::map<RecordKey, double> & values,
    LayerAggregate * layer)
  {
    if (layer == nullptr) {
      return;
    }
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto & headroom : values) {
      minimum = std::min(minimum, headroom.second);
    }
    if (std::isfinite(minimum) &&
      (!layer->clearance_known || minimum < layer->clearance_m))
    {
      layer->clearance_m = minimum;
      layer->clearance_known = true;
    }
  }

  // Merges one vertical interval into an aggregate union.
  static void MergeInterval(
    const VerticalInterval & interval, bool * known,
    double * min_z, double * max_z)
  {
    if (!std::isfinite(interval.min_z) ||
      !std::isfinite(interval.max_z))
    {
      return;
    }
    if (!*known) {
      *known = true;
      *min_z = interval.min_z;
      *max_z = interval.max_z;
      return;
    }
    *min_z = std::min(*min_z, interval.min_z);
    *max_z = std::max(*max_z, interval.max_z);
  }

  // Rebuilds all source->node associations after a terrain revision change. This
  // is the only full source scan and is allowed for a fresh terrain baseline.
  void ReassociateAllSources()
  {
    node_sources.clear();
    node_aggregates.clear();
    for (auto & source_entry : sources) {
      SourceState & state = source_entry.second;
      state.nodes.clear();
      for (auto & record_entry : state.records) {
        SourceRecord & record = record_entry.second;
        record.nodes = AssociateRecord(record.geometry);
        const double interval_min =
          std::min(record.geometry.min_z, record.geometry.max_z);
        const double interval_max =
          std::max(record.geometry.min_z, record.geometry.max_z);
        for (const NodeAssociation & association : record.nodes) {
          SourceNodeEvidence & evidence = state.nodes[association.id];
          if (record.occupied &&
            association.kind == AssociationKind::kSurface)
          {
            ++evidence.occupied_count;
            evidence.occupied_intervals[record_entry.first] =
              VerticalInterval{interval_min, interval_max};
            double headroom_m = std::numeric_limits<double>::quiet_NaN();
            if (ClearanceForNode(record, association.id, &headroom_m)) {
              evidence.occupied_headrooms[record_entry.first] = headroom_m;
            }
          } else if (record.occupied) {
            ++evidence.clear_count;
            evidence.occupied_intervals[record_entry.first] =
              VerticalInterval{interval_min, interval_max};
            double headroom_m = std::numeric_limits<double>::quiet_NaN();
            if (ClearanceForNode(record, association.id, &headroom_m)) {
              evidence.clear_headrooms[record_entry.first] = headroom_m;
            }
          } else {
            ++evidence.clear_count;
            if (record.evidence.observed_bottom_known &&
              record.evidence.observed_top_known)
            {
              evidence.observed_intervals[record_entry.first] = VerticalInterval{
                record.evidence.observed_bottom_z,
                record.evidence.observed_top_z};
            }
            double headroom_m = std::numeric_limits<double>::quiet_NaN();
            if (ClearanceForNode(record, association.id, &headroom_m)) {
              evidence.clear_headrooms[record_entry.first] = headroom_m;
            }
          }
          evidence.observation_stamp_ns =
            std::max(
            evidence.observation_stamp_ns,
            state.observation_stamp_ns);
          evidence.received_steady_ns =
            std::max(
            evidence.received_steady_ns,
            state.received_steady_ns);
          node_sources[association.id].insert(source_entry.first);
        }
      }
    }
    // Retry unassociated raw records against the new terrain before the full
    // layer baseline is built. Records that still do not match stay pending.
    for (auto & source_entry : sources) {
      RetryPendingRecords(&source_entry.second, source_entry.first, nullptr);
    }
    for (const auto & entry : node_sources) {
      RecomputeNode(entry.first, nullptr);
    }
  }

  // Builds complete static/dynamic tiled layers from the aggregate table. Used
  // only by a terrain replacement, where a fresh baseline is permitted.
  void BuildFullLayers(
    std::uint64_t layer_version,
    ObstacleTileLayer * static_out,
    ObstacleTileLayer * dynamic_out) const
  {
    std::unordered_map<TerrainNodeId, ObstacleAttributes> static_attributes;
    std::unordered_map<TerrainNodeId, ObstacleAttributes> dynamic_attributes;
    std::int64_t static_stamp = 0;
    std::int64_t static_received = 0;
    std::int64_t dynamic_stamp = 0;
    std::int64_t dynamic_received = 0;
    for (const auto & entry : node_aggregates) {
      if (HasEvidence(entry.second.static_layer)) {
        static_attributes.emplace(
          entry.first, MakeAttributes(entry.second.static_layer, true));
        static_stamp = std::max(
          static_stamp,
          entry.second.static_layer.observation_stamp_ns);
        static_received =
          std::max(
          static_received,
          entry.second.static_layer.received_steady_ns);
      }
      if (HasEvidence(entry.second.dynamic_layer)) {
        dynamic_attributes.emplace(
          entry.first, MakeAttributes(entry.second.dynamic_layer, false));
        dynamic_stamp = std::max(
          dynamic_stamp, entry.second.dynamic_layer.observation_stamp_ns);
        dynamic_received = std::max(
          dynamic_received, entry.second.dynamic_layer.received_steady_ns);
      }
    }
    const ObstacleTileOptions tile_options = TileOptions();
    *static_out = ObstacleTileLayer::FromFlat(
      *terrain, static_attributes, tile_options, static_stamp, static_received,
      layer_version, LayerObserved(SourceLayer::kStatic));
    *dynamic_out = ObstacleTileLayer::FromFlat(
      *terrain, dynamic_attributes, tile_options, dynamic_stamp,
      dynamic_received, layer_version,
      LayerObserved(SourceLayer::kDynamic));
  }

  // Publishes the current aggregate state using COW tiled layers and one
  // incremental field update. A failed field build or a backpressured store
  // keeps the snapshot staged for a retry.
  void BuildAndPublishObstacles(MapUpdateMetrics * metrics = nullptr)
  {
    if (terrain == nullptr || dirty_nodes.empty()) {
      return;
    }
    const auto build_start = std::chrono::steady_clock::now();
    const std::uint64_t next_version = version + 1;
    std::map<ObstacleTileKey, std::size_t> changed_tiles;
    for (const TerrainNodeId node_id : dirty_nodes) {
      const TerrainNode * node = terrain->FindNode(node_id);
      if (node != nullptr) {
        changed_tiles[static_layer.TileKeyForCell(node->cell)] = 0U;
      }
    }
    // Copy volume is an estimate from the actual base payload attribute counts
    // for the touched tiles, not a synthetic per-cell constant.
    std::size_t copy_bytes_estimated = 0U;
    for (const auto & entry : changed_tiles) {
      const auto static_tile = static_layer.FindTile(entry.first);
      const auto dynamic_tile = dynamic_layer.FindTile(entry.first);
      if (static_tile) {
        copy_bytes_estimated +=
          static_tile->attributes.size() * sizeof(ObstacleAttributes);
      }
      if (dynamic_tile) {
        copy_bytes_estimated +=
          dynamic_tile->attributes.size() * sizeof(ObstacleAttributes);
      }
    }
    if (metrics != nullptr) {
      metrics->dirty_nodes = dirty_nodes.size();
      metrics->updated_tiles = changed_tiles.size();
      metrics->copy_bytes_estimated = copy_bytes_estimated;
    }
    ObstacleTileBuilder static_builder(static_layer, TileOptions());
    ObstacleTileBuilder dynamic_builder(dynamic_layer, TileOptions());
    std::vector<FineCellIndex> changed_cells;
    changed_cells.reserve(dirty_nodes.size());
    std::int64_t static_stamp = static_layer.observation_stamp_ns();
    std::int64_t static_received = static_layer.received_steady_ns();
    std::int64_t dynamic_stamp = dynamic_layer.observation_stamp_ns();
    std::int64_t dynamic_received = dynamic_layer.received_steady_ns();
    for (const TerrainNodeId node_id : dirty_nodes) {
      const TerrainNode * node = terrain->FindNode(node_id);
      if (node == nullptr) {
        continue;
      }
      changed_cells.push_back(node->cell);
      const auto aggregate_iterator = node_aggregates.find(node_id);
      if (aggregate_iterator == node_aggregates.end()) {
        static_builder.Erase(node->cell, node_id);
        dynamic_builder.Erase(node->cell, node_id);
        continue;
      }
      const NodeAggregate & aggregate = aggregate_iterator->second;
      if (HasEvidence(aggregate.static_layer)) {
        static_builder.Set(
          node->cell, node_id,
          MakeAttributes(aggregate.static_layer, true));
        static_stamp = std::max(
          static_stamp, aggregate.static_layer.observation_stamp_ns);
        static_received = std::max(
          static_received, aggregate.static_layer.received_steady_ns);
      } else {
        static_builder.Erase(node->cell, node_id);
      }
      if (HasEvidence(aggregate.dynamic_layer)) {
        dynamic_builder.Set(
          node->cell, node_id,
          MakeAttributes(aggregate.dynamic_layer, false));
        dynamic_stamp = std::max(
          dynamic_stamp, aggregate.dynamic_layer.observation_stamp_ns);
        dynamic_received = std::max(
          dynamic_received, aggregate.dynamic_layer.received_steady_ns);
      } else {
        dynamic_builder.Erase(node->cell, node_id);
      }
    }
    std::sort(changed_cells.begin(), changed_cells.end());
    changed_cells.erase(
      std::unique(changed_cells.begin(), changed_cells.end()),
      changed_cells.end());

    ObstacleTileLayer new_static = static_builder.Build(
      static_stamp, static_received, next_version,
      LayerObserved(SourceLayer::kStatic));
    ObstacleTileLayer new_dynamic = dynamic_builder.Build(
      dynamic_stamp, dynamic_received, next_version,
      LayerObserved(SourceLayer::kDynamic));
    const auto cow_end = std::chrono::steady_clock::now();
    if (metrics != nullptr) {
      metrics->cow_ms = DurationToMs(cow_end - build_start);
      metrics->dirty_cells = changed_cells.size();
    }
    const auto coarse_start = std::chrono::steady_clock::now();
    const std::shared_ptr<const CoarseTerrainMap> new_coarse =
      OverlayCoarseTerrain(coarse, terrain, {}, options.coarse);
    if (metrics != nullptr) {
      metrics->coarse_ms = DurationToMs(
        std::chrono::steady_clock::now() - coarse_start);
    }
    const auto distance_start = std::chrono::steady_clock::now();
    std::shared_ptr<const ObstacleDistanceField> new_field =
      ObstacleDistanceField::UpdateIncremental(
      field, terrain, ObstacleLayer(new_static),
      ObstacleLayer(new_dynamic), changed_cells, options.distance);
    if (metrics != nullptr) {
      metrics->distance_ms = DurationToMs(
        std::chrono::steady_clock::now() - distance_start);
    }
    if (new_field == nullptr || !new_field->ready()) {
      return;
    }
    const auto snapshot_start = std::chrono::steady_clock::now();
    const std::shared_ptr<const MapSnapshot> snapshot =
      MapSnapshot::CreateWithPinnedField(
      terrain, new_coarse, ObstacleLayer(new_static),
      ObstacleLayer(new_dynamic), next_version, new_field);
    if (metrics != nullptr) {
      metrics->snapshot_ms = DurationToMs(
        std::chrono::steady_clock::now() - snapshot_start);
    }
    // Commit the worker state only after a complete snapshot exists. The stage
    // pointer always refers to the newest complete state; a backpressured older
    // stage is coalesced by replacing it.
    static_layer = std::move(new_static);
    dynamic_layer = std::move(new_dynamic);
    coarse = new_coarse;
    field = std::move(new_field);
    version = next_version;
    dirty_nodes.clear();
    pending_publish_ = snapshot;
    if (metrics != nullptr) {
      metrics->version = next_version;
    }
    static_cast<void>(TryFlushPending(metrics));
  }

  // Flushes the staged snapshot. Returns false when the store is backpressured;
  // the callback and published_version are never touched before acceptance.
  // When a telemetry sink is supplied, the actual TryPublish/callback interval
  // and the accepted/backpressured outcome are recorded.
  bool TryFlushPending(MapUpdateMetrics * metrics = nullptr)
  {
    if (pending_publish_ == nullptr) {
      return true;
    }
    const auto publish_start = std::chrono::steady_clock::now();
    if (metrics != nullptr) {
      metrics->publish_attempted = true;
    }
    const bool accepted = PublishAccepted(pending_publish_);
    if (metrics != nullptr) {
      metrics->publish_ms += DurationToMs(
        std::chrono::steady_clock::now() - publish_start);
      if (accepted) {
        metrics->publish_succeeded = true;
        metrics->backpressured = false;
        metrics->published_steady_ns = NowSteadyNs();
        metrics->version = pending_publish_->obstacle_version();
        if (store != nullptr) {
          // Diagnostics only, sampled after TryPublish released its handoff
          // mutex. These accessors take the store lock briefly.
          metrics->live_versions = store->historical_count() + 1U;
          metrics->retired_versions = store->retired_count();
          metrics->reclaimed_versions_total = store->reclaimed_versions();
          metrics->reclaim_ms = store->last_reclaim_ms();
          metrics->version_counts_available = true;
        }
      } else {
        metrics->backpressured = true;
      }
    }
    if (!accepted) {
      return false;
    }
    pending_publish_.reset();
    if (publish_period_ns_ > 0) {
      next_publish_deadline_ns_ = NowSteadyNs() + publish_period_ns_;
    }
    return true;
  }

  // Publishes to the store and optional callback. SnapshotStore::TryPublish
  // holds its handoff mutex only for the store mutation; on_publish is invoked
  // after that call returns, so worker subscribers never run while the store
  // handoff mutex is held.
  bool PublishAccepted(const std::shared_ptr<const MapSnapshot> & snapshot)
  {
    if (snapshot == nullptr) {
      return true;
    }
    if (store != nullptr) {
      if (!store->TryPublish(snapshot)) {
        return false;
      }
    }
    if (on_publish) {
      try {
        on_publish(snapshot);
      } catch (...) {
        // A subscriber failure must not break the map writer.
      }
    }
    published_version.store(
      snapshot->obstacle_version(),
      std::memory_order_release);
    return true;
  }

  // Flushes a staged publication and, when the rate period has elapsed, builds
  // at most one snapshot for all accumulated dirty nodes.
  void MaybePublish()
  {
    MapUpdateMetrics publish_metrics;
    publish_metrics.kind = MapUpdateMetrics::Kind::kPublish;
    publish_metrics.started_steady_ns = NowSteadyNs();
    if (pending_publish_ != nullptr) {
      if (!TryFlushPending(&publish_metrics)) {
        publish_metrics.completed_steady_ns = NowSteadyNs();
        publish_metrics.total_ms = SteadyDurationMs(
          publish_metrics.started_steady_ns,
          publish_metrics.completed_steady_ns);
        publish_metrics.resources = ReadProcessResourceSnapshot();
        EmitMetrics(publish_metrics);
        return;
      }
    }
    if (terrain == nullptr || dirty_nodes.empty()) {
      if (publish_metrics.publish_attempted) {
        publish_metrics.completed_steady_ns = NowSteadyNs();
        publish_metrics.total_ms = SteadyDurationMs(
          publish_metrics.started_steady_ns,
          publish_metrics.completed_steady_ns);
        publish_metrics.resources = ReadProcessResourceSnapshot();
        EmitMetrics(publish_metrics);
      }
      return;
    }
    if (publish_period_ns_ > 0 &&
      NowSteadyNs() < next_publish_deadline_ns_)
    {
      if (publish_metrics.publish_attempted) {
        publish_metrics.completed_steady_ns = NowSteadyNs();
        publish_metrics.total_ms = SteadyDurationMs(
          publish_metrics.started_steady_ns,
          publish_metrics.completed_steady_ns);
        publish_metrics.resources = ReadProcessResourceSnapshot();
        EmitMetrics(publish_metrics);
      }
      return;
    }
    BuildAndPublishObstacles(&publish_metrics);
    publish_metrics.completed_steady_ns = NowSteadyNs();
    publish_metrics.total_ms = SteadyDurationMs(
      publish_metrics.started_steady_ns,
      publish_metrics.completed_steady_ns);
    publish_metrics.resources = ReadProcessResourceSnapshot();
    if (publish_metrics.publish_attempted) {
      EmitMetrics(publish_metrics);
    }
  }

  // Shutdown flush ignores the rate period and retries a backpressured store for
  // a bounded window so accepted state is published when the store can drain.
  void FlushForShutdown()
  {
    const auto deadline =
      std::chrono::steady_clock::now() + kShutdownPublishRetryWindow;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pending_publish_ == nullptr && !dirty_nodes.empty()) {
        BuildAndPublishObstacles();
      } else if (pending_publish_ != nullptr) {
        static_cast<void>(TryFlushPending());
      }
      if (pending_publish_ == nullptr && dirty_nodes.empty()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Converts one source record into its terrain-node associations. Only decks in
  // the current fine cell are considered; unobserved cells stay unknown. A slab
  // that lies above a surface becomes overhead evidence only when the explicit
  // association envelope allows it; otherwise it is not guessed onto a deck.
  std::vector<NodeAssociation> AssociateRecord(
    const SourceOccupancyRecord & record) const
  {
    std::vector<NodeAssociation> result;
    if (terrain == nullptr || !record.point.IsFinite() ||
      !std::isfinite(record.min_z) || !std::isfinite(record.max_z))
    {
      return result;
    }
    const auto cell = FineCellOf(
      record.point, terrain->origin_x(),
      terrain->origin_y(), terrain->resolution());
    if (!cell.has_value()) {
      return result;
    }
    const std::vector<TerrainNodeId> * node_ids =
      terrain->FindNodesInCell(*cell);
    if (node_ids == nullptr) {
      return result;
    }
    const double observed_min = std::min(record.min_z, record.max_z);
    const double observed_max = std::max(record.min_z, record.max_z);
    std::vector<NodeAssociation> overlap;
    TerrainNodeId best_below = kInvalidTerrainNodeId;
    double best_below_top = -std::numeric_limits<double>::infinity();
    const bool overhead_enabled =
      std::isfinite(options.association_max_clearance_m) &&
      options.association_max_clearance_m > 0.0;
    for (const TerrainNodeId node_id : *node_ids) {
      const TerrainNode * node = terrain->FindNode(node_id);
      if (node == nullptr) {
        continue;
      }
      const double deck_min = std::isfinite(node->support.z_min) ?
        node->support.z_min :
        node->representative.z;
      const double deck_max = std::isfinite(node->support.z_max) ?
        node->support.z_max :
        node->representative.z;
      if (observed_max >= deck_min - kDeckOverlapToleranceM &&
        observed_min <= deck_max + kDeckOverlapToleranceM)
      {
        overlap.push_back(NodeAssociation{node_id, AssociationKind::kSurface});
        continue;
      }
      if (!overhead_enabled) {
        continue;
      }
      // A surface below the slab is a ceiling candidate only when the slab
      // underside is inside the configured clearance envelope.
      const double gap = observed_min - deck_max;
      if (gap < options.association_min_clearance_m ||
        gap < -kDeckOverlapToleranceM ||
        gap > options.association_max_clearance_m)
      {
        continue;
      }
      if (deck_max > best_below_top) {
        best_below = node_id;
        best_below_top = deck_max;
      }
    }
    if (!overlap.empty()) {
      return overlap;
    }
    if (best_below != kInvalidTerrainNodeId) {
      result.push_back(
        NodeAssociation{best_below, AssociationKind::kOverhead});
    }
    return result;
  }

  // Captures vertical evidence from the source snapshot once per record.
  RecordEvidence BuildRecordEvidence(
    const SourceObservationSnapshot & observation,
    const SourceOccupancyRecord & record) const
  {
    RecordEvidence evidence;
    const double record_min = std::min(record.min_z, record.max_z);
    const double record_max = std::max(record.min_z, record.max_z);
    bool explicit_known = false;
    double explicit_value = std::numeric_limits<double>::infinity();
    if (observation.headroom().headroom_known &&
      std::isfinite(observation.headroom().headroom_m))
    {
      explicit_known = true;
      explicit_value = observation.headroom().headroom_m;
    }
    bool found_bottom = false;
    double bottom = std::numeric_limits<double>::infinity();
    bool found_top = false;
    double top = -std::numeric_limits<double>::infinity();
    for (const ObservedVolume & volume : observation.known_volumes()) {
      if (!volume.known || !volume.free_space ||
        !volume.min.IsFinite() || !volume.max.IsFinite())
      {
        continue;
      }
      if (record.point.x < volume.min.x ||
        record.point.x > volume.max.x ||
        record.point.y < volume.min.y ||
        record.point.y > volume.max.y)
      {
        continue;
      }
      const double volume_min = std::min(volume.min.z, volume.max.z);
      const double volume_max = std::max(volume.min.z, volume.max.z);
      if (record_max < volume_min || record_min > volume_max) {
        continue;
      }
      if (volume_min < bottom) {
        bottom = volume_min;
        found_bottom = true;
      }
      if (volume_max > top) {
        top = volume_max;
        found_top = true;
      }
      if (volume.headroom_known &&
        std::isfinite(volume.headroom_m) &&
        volume.headroom_m < explicit_value)
      {
        explicit_known = true;
        explicit_value = volume.headroom_m;
      }
    }
    if (explicit_known) {
      evidence.explicit_headroom_known = true;
      evidence.explicit_headroom_m = explicit_value;
    }
    if (found_bottom) {
      evidence.observed_bottom_known = true;
      evidence.observed_bottom_z = bottom;
    } else {
      // The slab underside is itself explicit evidence. This is what lets an
      // overhead bridge slab expose a known ceiling clearance even when the
      // producer supplied no separate free-volume record.
      evidence.observed_bottom_known = true;
      evidence.observed_bottom_z = record_min;
    }
    if (found_top) {
      evidence.observed_top_known = true;
      evidence.observed_top_z = top;
    } else {
      evidence.observed_top_known = true;
      evidence.observed_top_z = record_max;
    }
    return evidence;
  }

  // Computes explicit clearance for one record/node pair. Occupied records use
  // the slab underside; clear records use the observed top of free space.
  // Missing evidence keeps clearance_known false rather than assuming free
  // vertical space.
  bool ClearanceForNode(
    const SourceRecord & record, TerrainNodeId node_id,
    double * clearance_m) const
  {
    if (terrain == nullptr || clearance_m == nullptr) {
      return false;
    }
    const TerrainNode * node = terrain->FindNode(node_id);
    if (node == nullptr) {
      return false;
    }
    const double deck_z = node->representative.z;
    double deck_top = deck_z;
    if (std::isfinite(node->support.z_max)) {
      deck_top = std::max(deck_top, node->support.z_max);
    }
    double value = std::numeric_limits<double>::infinity();
    bool known = false;
    if (record.evidence.explicit_headroom_known) {
      value = record.evidence.explicit_headroom_m;
      known = true;
    }
    if (record.occupied) {
      if (record.evidence.observed_bottom_known &&
        record.evidence.observed_bottom_z >=
        deck_top - kDeckOverlapToleranceM)
      {
        const double candidate =
          std::max(0.0, record.evidence.observed_bottom_z - deck_top);
        value = std::min(value, candidate);
        known = true;
      }
    } else if (record.evidence.observed_top_known &&
      record.evidence.observed_top_z >=
      deck_z - kDeckOverlapToleranceM)
    {
      const double candidate =
        std::max(0.0, record.evidence.observed_top_z - deck_z);
      value = std::min(value, candidate);
      known = true;
    }
    if (!known || !std::isfinite(value)) {
      return false;
    }
    *clearance_m = value;
    return true;
  }

  // Returns true when any source currently contributes to this layer.
  bool LayerObserved(SourceLayer layer) const
  {
    for (const auto & entry : sources) {
      if (entry.second.layer == layer) {
        return true;
      }
    }
    return false;
  }

  // Builds one layer's attribute record from its aggregate.
  ObstacleAttributes MakeAttributes(
    const LayerAggregate & layer,
    bool is_static) const
  {
    ObstacleAttributes attributes;
    const ObstacleState state =
      layer.occupied ? ObstacleState::kOccupied : ObstacleState::kClear;
    if (is_static) {
      attributes.static_state = state;
    } else {
      attributes.dynamic_state = state;
    }
    attributes.clearance_known = HasKnownClearance(layer);
    attributes.clearance_above_m =
      attributes.clearance_known ?
      layer.clearance_m :
      std::numeric_limits<double>::quiet_NaN();
    attributes.occupied_vertical_extent_known = layer.occupied_extent_known;
    attributes.occupied_min_z = layer.occupied_extent_known ?
      layer.occupied_min_z :
      std::numeric_limits<double>::quiet_NaN();
    attributes.occupied_max_z = layer.occupied_extent_known ?
      layer.occupied_max_z :
      std::numeric_limits<double>::quiet_NaN();
    attributes.observed_vertical_extent_known = layer.observed_extent_known;
    attributes.observed_min_z = layer.observed_extent_known ?
      layer.observed_min_z :
      std::numeric_limits<double>::quiet_NaN();
    attributes.observed_max_z = layer.observed_extent_known ?
      layer.observed_max_z :
      std::numeric_limits<double>::quiet_NaN();
    attributes.observation_stamp_ns = layer.observation_stamp_ns;
    attributes.received_steady_ns = layer.received_steady_ns;
    return attributes;
  }

  // Fixed tile policy for this worker.
  ObstacleTileOptions TileOptions() const
  {
    ObstacleTileOptions tile_options;
    tile_options.tile_size_m = kDefaultTileSizeM;
    tile_options.reject_unknown = options.reject_unknown_sources;
    return tile_options;
  }

  // Releases the conservative submitted-record budget for a source only when no
  // newer source event is still pending for it.
  void ReleaseSubmittedRecordBudgetIfIdle(const std::string & source_id)
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    for (const Request & request : pending) {
      if (request.kind == MapUpdateRequestKind::kSourceObservation &&
        request.source.has_value() &&
        request.source->source_id() == source_id)
      {
        return;
      }
    }
    const auto iterator = submitted_record_counts.find(source_id);
    if (iterator == submitted_record_counts.end()) {
      return;
    }
    if (submitted_records_total >= iterator->second) {
      submitted_records_total -= iterator->second;
    } else {
      submitted_records_total = 0;
    }
    submitted_record_counts.erase(iterator);
  }

  // Updates the monotonic per-source sequence highwater and its LRU ordering.
  // The registry is never erased, so a late packet after expiry/removal stays
  // stale instead of resurrecting a contribution.
  void TouchHighwater(const std::string & source_id, std::uint64_t sequence)
  {
    const auto iterator = highwater.find(source_id);
    if (iterator == highwater.end()) {
      highwater_lru.push_back(source_id);
      highwater.emplace(
        source_id,
        HighwaterEntry{sequence, std::prev(highwater_lru.end())});
      return;
    }
    iterator->second.sequence = sequence;
    highwater_lru.splice(
      highwater_lru.end(), highwater_lru,
      iterator->second.lru);
  }

  MapUpdateResult SubmitTerrain(TerrainPointObservation observation)
  {
    const std::int64_t now = NowSteadyNs();
    const ObservationValidation validation = observation.Validate(
      options.builder.observation_validation, now);
    if (!validation.ok) {
      return MapUpdateResult{MapUpdateStatus::kRejectedInvalid,
        validation.detail};
    }
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (stop) {
      return MapUpdateResult{MapUpdateStatus::kStopped,
        "worker is stopped"};
    }
    std::size_t coalesced = 0;
    for (const Request & request : pending) {
      if (request.kind == MapUpdateRequestKind::kTerrainReplacement) {
        ++coalesced;
      }
    }
    const std::size_t projected_size =
      pending.size() - coalesced + 1U;
    if (projected_size > options.max_pending_batches) {
      return MapUpdateResult{MapUpdateStatus::kQueueFull,
        "terrain queue is at its hard bound"};
    }
    for (auto iterator = pending.begin(); iterator != pending.end(); ) {
      if (iterator->kind == MapUpdateRequestKind::kTerrainReplacement) {
        iterator = pending.erase(iterator);
      } else {
        ++iterator;
      }
    }
    Request request;
    request.kind = MapUpdateRequestKind::kTerrainReplacement;
    request.terrain = std::move(observation);
    request.now_steady_ns = now;
    request.enqueued_steady_ns = NowSteadyNs();
    pending.push_back(std::move(request));
    queue_cv.notify_one();
    return MapUpdateResult{MapUpdateStatus::kAccepted, ""};
  }

  MapUpdateResult SubmitSource(SourceObservationSnapshot observation)
  {
    std::string detail;
    if (!observation.IsUsable(&detail)) {
      return MapUpdateResult{MapUpdateStatus::kRejectedInvalid, detail};
    }
    const std::int64_t now = NowSteadyNs();
    if (observation.received_steady_ns() >
      now + options.builder.observation_validation.future_tolerance_ns)
    {
      return MapUpdateResult{MapUpdateStatus::kRejectedInvalid,
        "source receipt stamp is in the future"};
    }
    const std::uint64_t current_terrain_revision =
      latest_terrain_revision.load(std::memory_order_acquire);
    if (observation.terrain_revision() != 0 &&
      current_terrain_revision != 0 &&
      observation.terrain_revision() != current_terrain_revision)
    {
      return MapUpdateResult{
        MapUpdateStatus::kRejectedStale,
        "source observation was produced for a different terrain revision"};
    }

    const std::string source_id = observation.source_id();
    const std::uint64_t sequence = observation.source_sequence();
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (stop) {
      return MapUpdateResult{MapUpdateStatus::kStopped,
        "worker is stopped"};
    }
    const auto sequence_iterator = highwater.find(source_id);
    if (sequence_iterator != highwater.end()) {
      if (sequence == sequence_iterator->second.sequence) {
        return MapUpdateResult{MapUpdateStatus::kRejectedDuplicate,
          "duplicate source sequence"};
      }
      if (sequence < sequence_iterator->second.sequence) {
        return MapUpdateResult{MapUpdateStatus::kRejectedStale,
          "source sequence is older than the last one"};
      }
    } else if (options.max_sources > 0 &&
      highwater.size() >= options.max_sources)
    {
      return MapUpdateResult{
        MapUpdateStatus::kRejectedCapacity,
        "source sequence highwater registry is full"};
    }

    const std::size_t new_records = RecordCountOf(observation);
    const std::size_t old_records = LookupSubmittedRecordCount(source_id);
    std::size_t projected_source_records = 0;
    std::size_t projected_total_records = submitted_records_total;
    if (observation.kind() == SourceUpdateKind::kCompleteReplacement) {
      projected_source_records = new_records;
      projected_total_records =
        submitted_records_total >= old_records ?
        submitted_records_total - old_records :
        0U;
      if (!AddWouldOverflow(projected_total_records, new_records)) {
        projected_total_records += new_records;
      } else {
        projected_total_records = std::numeric_limits<std::size_t>::max();
      }
    } else {
      projected_source_records = old_records;
      if (!AddWouldOverflow(projected_source_records, new_records)) {
        projected_source_records += new_records;
      } else {
        projected_source_records = std::numeric_limits<std::size_t>::max();
      }
      projected_total_records =
        AddWouldOverflow(submitted_records_total, new_records) ?
        std::numeric_limits<std::size_t>::max() :
        submitted_records_total + new_records;
    }
    if (options.max_records_per_source > 0 &&
      projected_source_records > options.max_records_per_source)
    {
      return MapUpdateResult{MapUpdateStatus::kRejectedCapacity,
        "per-source record budget would be exceeded"};
    }
    if (options.max_records_total > 0 &&
      projected_total_records > options.max_records_total)
    {
      return MapUpdateResult{MapUpdateStatus::kRejectedCapacity,
        "total record budget would be exceeded"};
    }

    std::size_t coalesced = 0;
    if (observation.kind() == SourceUpdateKind::kCompleteReplacement) {
      // A newer complete replacement is the whole final state for this source;
      // dropping older pending complete/delta events preserves final semantics
      // without ever dropping an unsuperseded delta.
      for (const Request & request : pending) {
        if (request.kind == MapUpdateRequestKind::kSourceObservation &&
          request.source.has_value() &&
          request.source->source_id() == source_id)
        {
          ++coalesced;
        }
      }
    }
    const std::size_t projected_size =
      pending.size() - coalesced + 1U;
    if (projected_size > options.max_pending_batches) {
      return MapUpdateResult{MapUpdateStatus::kQueueFull,
        "source queue is at its hard bound"};
    }

    // All checks passed; mutate deterministically.
    if (coalesced > 0) {
      for (auto iterator = pending.begin(); iterator != pending.end(); ) {
        if (iterator->kind == MapUpdateRequestKind::kSourceObservation &&
          iterator->source.has_value() &&
          iterator->source->source_id() == source_id)
        {
          iterator = pending.erase(iterator);
        } else {
          ++iterator;
        }
      }
    }
    Request request;
    request.kind = MapUpdateRequestKind::kSourceObservation;
    request.source = std::move(observation);
    request.source_id = source_id;
    request.source_sequence = sequence;
    request.enqueued_steady_ns = NowSteadyNs();
    pending.push_back(std::move(request));
    submitted_record_counts[source_id] = projected_source_records;
    submitted_records_total = projected_total_records;
    TouchHighwater(source_id, sequence);
    queue_cv.notify_one();
    if (coalesced > 0) {
      return MapUpdateResult{MapUpdateStatus::kQueueFullCoalesced,
        "superseded older pending source events"};
    }
    return MapUpdateResult{MapUpdateStatus::kAccepted, ""};
  }

  MapUpdateResult SubmitSourceRemoval(std::string source_id)
  {
    if (source_id.empty()) {
      return MapUpdateResult{MapUpdateStatus::kRejectedInvalid,
        "source_id is empty"};
    }
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (stop) {
      return MapUpdateResult{MapUpdateStatus::kStopped,
        "worker is stopped"};
    }
    std::size_t coalesced = 0;
    for (const Request & request : pending) {
      if (request.kind == MapUpdateRequestKind::kSourceObservation &&
        request.source.has_value() &&
        request.source->source_id() == source_id)
      {
        ++coalesced;
      }
    }
    const std::size_t projected_size =
      pending.size() - coalesced + 1U;
    if (projected_size > options.max_pending_batches) {
      return MapUpdateResult{MapUpdateStatus::kQueueFull,
        "source removal queue is at its hard bound"};
    }
    for (auto iterator = pending.begin(); iterator != pending.end(); ) {
      if (iterator->kind == MapUpdateRequestKind::kSourceObservation &&
        iterator->source.has_value() &&
        iterator->source->source_id() == source_id)
      {
        iterator = pending.erase(iterator);
      } else {
        ++iterator;
      }
    }
    // The highwater is intentionally not erased. A later stale packet for this
    // removed source must be rejected, not resurrect occupancy.
    Request request;
    request.kind = MapUpdateRequestKind::kSourceRemoval;
    request.source_id = std::move(source_id);
    request.enqueued_steady_ns = NowSteadyNs();
    pending.push_back(std::move(request));
    queue_cv.notify_one();
    return MapUpdateResult{MapUpdateStatus::kAccepted, ""};
  }

  MapUpdateResult ExpireSources(std::int64_t now_steady_ns)
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (stop) {
      return MapUpdateResult{MapUpdateStatus::kStopped,
        "worker is stopped"};
    }
    std::size_t coalesced = 0;
    for (const Request & request : pending) {
      if (request.kind == MapUpdateRequestKind::kExpireSources) {
        ++coalesced;
      }
    }
    const std::size_t projected_size =
      pending.size() - coalesced + 1U;
    if (projected_size > options.max_pending_batches) {
      return MapUpdateResult{MapUpdateStatus::kQueueFull,
        "expiry queue is at its hard bound"};
    }
    for (auto iterator = pending.begin(); iterator != pending.end(); ) {
      if (iterator->kind == MapUpdateRequestKind::kExpireSources) {
        iterator = pending.erase(iterator);
      } else {
        ++iterator;
      }
    }
    Request request;
    request.kind = MapUpdateRequestKind::kExpireSources;
    request.now_steady_ns = now_steady_ns;
    request.enqueued_steady_ns = NowSteadyNs();
    pending.push_back(std::move(request));
    queue_cv.notify_one();
    return MapUpdateResult{MapUpdateStatus::kAccepted, ""};
  }

  // Counts the frozen records and volumes that a submission charges against the
  // record budget. A volume is charged as one record here; rasterized coverage
  // is separately capped by max_records_per_source/max_records_total.
  static std::size_t RecordCountOf(
    const SourceObservationSnapshot & observation)
  {
    std::size_t total = observation.occupied().size();
    if (!AddWouldOverflow(total, observation.cleared().size())) {
      total += observation.cleared().size();
    } else {
      return std::numeric_limits<std::size_t>::max();
    }
    if (!AddWouldOverflow(total, observation.known_volumes().size())) {
      total += observation.known_volumes().size();
    } else {
      return std::numeric_limits<std::size_t>::max();
    }
    return total;
  }

  // Lookup helper that never mutates the submitted record table.
  std::size_t LookupSubmittedRecordCount(const std::string & source_id) const
  {
    const auto iterator = submitted_record_counts.find(source_id);
    return iterator == submitted_record_counts.end() ? 0U : iterator->second;
  }

  // Returns true when lhs + rhs would overflow std::size_t.
  static bool AddWouldOverflow(std::size_t lhs, std::size_t rhs) noexcept
  {
    return lhs > std::numeric_limits<std::size_t>::max() - rhs;
  }

  struct HighwaterEntry
  {
    std::uint64_t sequence{0};
    std::list<std::string>::iterator lru;
  };

  MapUpdateWorkerOptions options;
  SnapshotStore * store{nullptr};
  PublishCallback on_publish;
  UpdateMetricsCallback on_metrics;

  // Serializes Start/Stop so two callers cannot join one thread twice.
  std::mutex lifecycle_mutex;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::list<Request> pending;
  bool stop{false};
  std::atomic<bool> running{false};
  std::thread thread;

  // Monotonic per-source highwater and conservative accepted-record budget.
  // Guarded by queue_mutex; used only at Submit time and by worker release.
  std::unordered_map<std::string, HighwaterEntry> highwater;
  std::list<std::string> highwater_lru;
  std::unordered_map<std::string, std::size_t> submitted_record_counts;
  std::size_t submitted_records_total{0};

  // Worker-owned pipeline state.
  TerrainGridBuilder terrain_builder;
  std::shared_ptr<const TerrainSnapshot> terrain;
  std::shared_ptr<const CoarseTerrainMap> coarse;
  ObstacleTileLayer static_layer;
  ObstacleTileLayer dynamic_layer;
  std::shared_ptr<const ObstacleDistanceField> field;
  std::uint64_t version{0};
  std::atomic<std::uint64_t> published_version{0};
  std::atomic<std::uint64_t> latest_terrain_revision{0};
  std::unordered_map<std::string, SourceState> sources;
  std::unordered_map<TerrainNodeId, std::unordered_set<std::string>>
  node_sources;
  std::unordered_map<TerrainNodeId, NodeAggregate> node_aggregates;
  std::unordered_set<TerrainNodeId> dirty_nodes;
  std::size_t worker_record_count_{0};

  // Accepted observations that arrived before any terrain (revision 0) or for a
  // terrain revision that is not current yet. Bounded by the submit-time source
  // and record budgets; replayed before a matching baseline is published.
  std::map<std::string, std::vector<SourceObservationSnapshot>>
  deferred_sources;

  // Newest complete snapshot that the store has not accepted yet. A newer build
  // replaces it so a backpressured publication is coalesced, never queued
  // without bound.
  std::shared_ptr<const MapSnapshot> pending_publish_;
  std::int64_t publish_period_ns_{0};
  std::int64_t next_publish_deadline_ns_{0};
};

MapUpdateWorker::MapUpdateWorker(
  MapUpdateWorkerOptions options,
  SnapshotStore * store,
  PublishCallback on_publish,
  UpdateMetricsCallback on_metrics)
: impl_(std::make_unique<Impl>(
      std::move(options), store,
      std::move(on_publish),
      std::move(on_metrics))) {}

MapUpdateWorker::~MapUpdateWorker() {Stop();}

bool MapUpdateWorker::Start() {return impl_->Start();}

void MapUpdateWorker::Stop()
{
  if (impl_ != nullptr) {
    impl_->Stop();
  }
}

MapUpdateResult MapUpdateWorker::SubmitTerrain(
  TerrainPointObservation observation)
{
  return impl_->SubmitTerrain(std::move(observation));
}

MapUpdateResult MapUpdateWorker::SubmitSource(
  SourceObservationSnapshot observation)
{
  return impl_->SubmitSource(std::move(observation));
}

MapUpdateResult MapUpdateWorker::SubmitSourceRemoval(std::string source_id)
{
  return impl_->SubmitSourceRemoval(std::move(source_id));
}

MapUpdateResult MapUpdateWorker::ExpireSources(std::int64_t now_steady_ns)
{
  return impl_->ExpireSources(now_steady_ns);
}

bool MapUpdateWorker::running() const noexcept
{
  return impl_ != nullptr &&
         impl_->running.load(std::memory_order_acquire);
}

std::uint64_t MapUpdateWorker::published_version() const noexcept
{
  return impl_ == nullptr ?
         0 :
         impl_->published_version.load(std::memory_order_acquire);
}

}  // namespace dddmr_grid_global_planner
