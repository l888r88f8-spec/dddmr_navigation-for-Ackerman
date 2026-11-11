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

#include "dddmr_grid_global_planner/snapshot_store.h"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace dddmr_grid_global_planner
{

SnapshotStore::SnapshotStore(
  std::size_t retained_versions,
  std::size_t max_retired_versions)
: retained_versions_(retained_versions),
  max_retired_versions_(max_retired_versions),
  reclaimer_(&SnapshotStore::ReclaimerLoop, this) {}

SnapshotStore::~SnapshotStore() {Stop();}

bool SnapshotStore::TryPublish(
  std::shared_ptr<const MapSnapshot> snapshot)
{
  if (snapshot == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) {
    return false;
  }
  if (latest_ != nullptr && latest_.get() == snapshot.get()) {
    // Publishing the same immutable pointer cannot evict anything and is a
    // successful idempotent no-op.
    return true;
  }
  // One publish evicts at most one historical entry because the invariant
  // retained_.size() <= retained_versions_ is maintained here.
  const bool will_retire =
    latest_ != nullptr &&
    (retained_versions_ == 0 || retained_.size() >= retained_versions_);
  if (will_retire && retired_.size() >= max_retired_versions_) {
    return false;
  }
  if (latest_ != nullptr) {
    // Historical versions are kept in addition to the latest pointer. The old
    // latest becomes the newest historical entry; overflow moves to retirement.
    retained_.push_back(latest_);
    while (retained_.size() > retained_versions_) {
      retired_.push_back(std::move(retained_.front()));
      retained_.pop_front();
    }
  }
  latest_ = std::move(snapshot);
  reclaimer_cv_.notify_one();
  return true;
}

void SnapshotStore::Publish(std::shared_ptr<const MapSnapshot> snapshot)
{
  static_cast<void>(TryPublish(std::move(snapshot)));
}

std::shared_ptr<const MapSnapshot> SnapshotStore::AcquireLatest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

SnapshotCommitStatus SnapshotStore::CommitLatest(
  const std::function<bool(const MapSnapshot &)> & predicate,
  const std::function<bool(const MapSnapshot &)> & on_commit)
{
  if (!predicate || !on_commit) {
    return SnapshotCommitStatus::kCallbackRejected;
  }
  // Holding the handoff mutex excludes TryPublish for the whole predicate and
  // callback. A relevant map publication therefore either happens before this
  // acquisition or after the terminal result, never between the check and the
  // commit. The callbacks run with the mutex held and must not re-enter it.
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) {
    return SnapshotCommitStatus::kStopped;
  }
  if (latest_ == nullptr) {
    return SnapshotCommitStatus::kNoSnapshot;
  }
  const MapSnapshot & current = *latest_;
  if (!predicate(current)) {
    return SnapshotCommitStatus::kPredicateRejected;
  }
  if (!on_commit(current)) {
    return SnapshotCommitStatus::kCallbackRejected;
  }
  return SnapshotCommitStatus::kCommitted;
}

std::size_t SnapshotStore::retained_versions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return retained_versions_;
}

std::size_t SnapshotStore::retired_versions_bound() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_retired_versions_;
}

std::size_t SnapshotStore::historical_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return retained_.size();
}

std::size_t SnapshotStore::retired_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return retired_.size();
}

std::uint64_t SnapshotStore::reclaimed_versions() const noexcept
{
  return reclaimed_versions_.load(std::memory_order_relaxed);
}

double SnapshotStore::last_reclaim_ms() const noexcept
{
  return last_reclaim_ms_.load(std::memory_order_relaxed);
}

bool SnapshotStore::WaitForRetire(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return retire_cv_.wait_for(
    lock, timeout, [this]() {return retired_.empty();});
}

void SnapshotStore::Stop()
{
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  reclaimer_cv_.notify_all();
  if (reclaimer_.joinable()) {
    reclaimer_.join();
  }
  // No publication can be in flight after the reclaimer join because Stopping
  // rejects every TryPublish. Release whatever the reclaimer could not reclaim
  // (for example a snapshot still pinned by an exiting reader) outside the
  // mutex. This is the only path that may destroy a pinned retired reference,
  // and it runs only during explicit shutdown.
  std::deque<std::shared_ptr<const MapSnapshot>> drained;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drained.swap(retired_);
  }
  drained.clear();
  retire_cv_.notify_all();
}

void SnapshotStore::ReclaimerLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (stopping_) {
      return;
    }
    if (retired_.empty()) {
      reclaimer_cv_.wait(
        lock, [this]() {
          return stopping_ || !retired_.empty();
        });
      continue;
    }
    std::deque<std::shared_ptr<const MapSnapshot>> released;
    for (auto iterator = retired_.begin(); iterator != retired_.end(); ) {
      // use_count == 1 means the retired queue is the only owner. A pinned
      // entry stays in place so the reader, not the reclaimer, cannot race with
      // destruction.
      if (iterator->use_count() == 1) {
        released.push_back(std::move(*iterator));
        iterator = retired_.erase(iterator);
      } else {
        ++iterator;
      }
    }
    if (!released.empty()) {
      // Destroy outside the handoff mutex. A heavy snapshot destructor or a
      // late reader notification must not stall publication. The elapsed time
      // and count are diagnostics for the update worker's telemetry.
      const std::size_t released_count = released.size();
      lock.unlock();
      const auto reclaim_start = std::chrono::steady_clock::now();
      released.clear();
      const double reclaim_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() -
        reclaim_start)
        .count();
      reclaimed_versions_.fetch_add(
        static_cast<std::uint64_t>(released_count),
        std::memory_order_relaxed);
      last_reclaim_ms_.store(reclaim_ms, std::memory_order_relaxed);
      lock.lock();
      retire_cv_.notify_all();
      continue;
    }
    // A pinned retired reference is not dropped. Poll briefly so a reader that
    // releases later is observed without a busy spin.
    reclaimer_cv_.wait_for(lock, std::chrono::milliseconds(2));
  }
}

}  // namespace dddmr_grid_global_planner
