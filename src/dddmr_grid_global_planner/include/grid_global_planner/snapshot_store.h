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

#ifndef GRID_GLOBAL_PLANNER__SNAPSHOT_STORE_H_
#define GRID_GLOBAL_PLANNER__SNAPSHOT_STORE_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "grid_global_planner/map_snapshot.h"

namespace grid_global_planner
{

// Result of SnapshotStore::CommitLatest().
enum class SnapshotCommitStatus : std::uint8_t
{
  // The predicate accepted the current latest snapshot and the commit callback
  // completed successfully while publication was excluded.
  kCommitted,
  // The store is stopping or already stopped.
  kStopped,
  // No snapshot has been published yet.
  kNoSnapshot,
  // The caller's cheap relevant-content predicate rejected the latest snapshot.
  kPredicateRejected,
  // The predicate accepted, but the commit callback declined (for example a
  // canceled job or a stale lifecycle epoch) before publishing anything.
  kCallbackRejected,
};

// Handoff between the single map-update writer and planning readers.
//
// Ownership: Publish/TryPublish take shared ownership of an immutable
// MapSnapshot. The latest pointer, every retained historical entry, and every
// retired entry are shared_ptr, so a snapshot stays alive while a search or a
// callback still holds it.
//
// Retirement: publication never destroys an old snapshot on the publisher or
// reader thread. Evicted historical entries are moved to a bounded retired
// queue owned by a dedicated reclaimer thread. The reclaimer releases a queued
// reference only after `use_count() == 1` (only the queue still holds it), and
// it always releases that reference outside the handoff mutex. A pinned retired
// reference is therefore never popped or dropped by normal operation.
//
// Backpressure: `max_retired_versions` bounds how many retired references may
// be alive at once. If a publication would need to retire one more reference
// while the bound is reached, TryPublish returns false and leaves the store
// unchanged. The update worker treats that as backpressure, coalesces its
// pending snapshot, and retries after the reclaimer has caught up.
//
// Locking: the mutex protects the latest pointer, the bounded historical deque,
// and the retired queue. It is never held by a search hot loop; AcquireLatest
// copies one shared_ptr once per job and returns immediately. Destruction of a
// released snapshot happens outside the mutex.
//
// Final acceptance: CommitLatest() is the only way to serialize a result commit
// with publication. It holds the handoff mutex, runs a cheap predicate on the
// actual latest snapshot, and then runs the commit callback before releasing
// the mutex. A TryPublish() racing this call therefore either publishes before
// the predicate or waits until the callback returns, so a known-invalid path
// cannot be attached to a terminal result while a relevant newer map is already
// visible. The callback must perform only the short in-memory acceptance
// transition (job identity/phase), never an action SDK succeed()/abort() or
// private path DDS publication; those run after CommitLatest() returns and the
// handoff mutex is released. The lock order is store handoff mutex -> caller
// job/commit mutex. No caller may acquire the handoff mutex while holding its
// commit mutex.
//
// Reentrancy: predicate and callback run with the handoff mutex held. They must
// not call any SnapshotStore method (including AcquireLatest/TryPublish/Stop);
// doing so would self-deadlock. They must also stay short: callback work blocks
// all map publication. Worker publish subscribers are different: the worker
// invokes them only after TryPublish() has returned and released the mutex.
//
// Shutdown: call Stop() only after planning jobs and the map-update worker have
// stopped. Stop() stops and joins the reclaimer, then releases any remaining
// retired references. It is idempotent. WaitForRetire() and the diagnostics
// accessors are intended for tests and shutdown sequencing, never hot paths.
class SnapshotStore
{
public:
  // retained_versions is the number of historical entries kept in addition to
  // the latest pointer. max_retired_versions bounds the live retired queue; a
  // publication fails rather than exceeding it.
  explicit SnapshotStore(
    std::size_t retained_versions = 4,
    std::size_t max_retired_versions = 1024);

  // Stops the reclaimer. The store must not be used for publication afterwards.
  ~SnapshotStore();

  SnapshotStore(const SnapshotStore &) = delete;
  SnapshotStore & operator=(const SnapshotStore &) = delete;

  // Attempts to publish a new latest snapshot. Returns false, without mutating
  // the store, when the retired queue bound would be exceeded or after Stop().
  // A null snapshot is rejected. Re-publishing the exact current pointer is a
  // successful no-op, which keeps idempotent writers from consuming history.
  bool TryPublish(std::shared_ptr<const MapSnapshot> snapshot);

  // Compatibility wrapper around TryPublish. A rejected publication is ignored;
  // callers that need backpressure must use TryPublish and retry.
  void Publish(std::shared_ptr<const MapSnapshot> snapshot);

  // Returns the latest published snapshot once per call, or nullptr before the
  // first publish. The returned shared_ptr keeps the snapshot alive.
  std::shared_ptr<const MapSnapshot> AcquireLatest() const;

  // Atomically accepts the actual latest snapshot for a final result commit.
  //
  // The handoff mutex is held for the whole call. `predicate` must be a cheap
  // test over the current latest snapshot (for example relevant corridor tile
  // identities), and `on_commit` must perform only the short in-memory terminal
  // acceptance (job identity/epoch/phase), not diagnostics, rendering, action
  // SDK calls, or private path publication. Neither callback may call back into
  // this store. If the predicate rejects, `on_commit` is not called and the
  // store is unchanged. If `on_commit` returns false, it must not have
  // published anything and the store is unchanged. Both callbacks are invoked
  // with the current latest snapshot by const reference; its lifetime is
  // guaranteed by the handoff mutex for the duration of the call.
  //
  // Linearization: the plan is valid at the instant this call accepts the
  // immutable snapshot. A map update after that point may become visible during
  // later DDS delivery; no freshness-at-client-receipt claim is made and the map
  // is not frozen until the SDK reply.
  SnapshotCommitStatus CommitLatest(
    const std::function<bool(const MapSnapshot &)> & predicate,
    const std::function<bool(const MapSnapshot &)> & on_commit);

  // Returns the configured historical-version bound (latest is extra).
  std::size_t retained_versions() const;

  // Returns the configured live-retired-reference bound.
  std::size_t retired_versions_bound() const;

  // Diagnostics: number of historical entries currently retained in addition
  // to latest.
  std::size_t historical_count() const;

  // Diagnostics: number of retired references the reclaimer has not released
  // yet. A nonzero value may include references still pinned by readers.
  std::size_t retired_count() const;

  // Diagnostics: cumulative retired snapshot references the reclaimer has
  // released and the duration of the most recent release batch. These are
  // sampled by the update worker's telemetry and are never used by search.
  std::uint64_t reclaimed_versions() const noexcept;
  double last_reclaim_ms() const noexcept;

  // Waits until the retired queue is empty. Returns false on timeout. Useful in
  // tests and in shutdown sequencing after readers have released their pins.
  bool WaitForRetire(std::chrono::milliseconds timeout);

  // Stops and joins the reclaimer, then releases remaining retired references.
  // Idempotent. Publications after this call return false.
  void Stop();

private:
  // Reclaimer loop. Pops only entries whose shared_ptr use_count is exactly one
  // and destroys them outside the handoff mutex.
  void ReclaimerLoop();

  // Serializes Stop so two callers cannot join the same thread twice.
  std::mutex stop_mutex_;

  mutable std::mutex mutex_;
  std::condition_variable reclaimer_cv_;
  std::condition_variable retire_cv_;
  std::shared_ptr<const MapSnapshot> latest_;
  std::deque<std::shared_ptr<const MapSnapshot>> retained_;
  std::deque<std::shared_ptr<const MapSnapshot>> retired_;
  std::size_t retained_versions_{4};
  std::size_t max_retired_versions_{1024};
  bool stopping_{false};
  std::thread reclaimer_;
  std::atomic<std::uint64_t> reclaimed_versions_{0};
  std::atomic<double> last_reclaim_ms_{0.0};
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__SNAPSHOT_STORE_H_
