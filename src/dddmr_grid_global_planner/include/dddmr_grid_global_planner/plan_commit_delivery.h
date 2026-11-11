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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__PLAN_COMMIT_DELIVERY_H_
#define DDDMR_GRID_GLOBAL_PLANNER__PLAN_COMMIT_DELIVERY_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "dddmr_grid_global_planner/latest_validation.h"
#include "dddmr_grid_global_planner/snapshot_store.h"

namespace dddmr_grid_global_planner
{

// Identity of one accepted action job. All three fields are compared together:
// a goal from an older deactivation generation can never mutate, cancel, or
// publish for the current job even if it reuses a goal sequence number.
struct PlanJobIdentity
{
  std::string goal_uuid;
  std::uint64_t goal_seq{0};
  std::uint64_t lifecycle_epoch{0};

  bool empty() const noexcept {return goal_uuid.empty();}
  bool operator==(const PlanJobIdentity & other) const noexcept
  {
    return goal_seq == other.goal_seq &&
           lifecycle_epoch == other.lifecycle_epoch &&
           goal_uuid == other.goal_uuid;
  }
  bool operator!=(const PlanJobIdentity & other) const noexcept
  {
    return !(*this == other);
  }
};

// Explicit delivery phase. Admission stays busy (the caller must not accept a
// parallel goal) until kTerminal. kDeliveryFailed is a bounded fail-closed
// reservation: the snapshot commit linearized but SDK terminality is unknown,
// so the goal handle and the reservation must not be released.
enum class PlanJobPhase : std::uint8_t
{
  kIdle = 0,
  kPlanning,
  kCommitted,
  kDelivering,
  kTerminal,
  kDeliveryFailed,
};

const char * PlanJobPhaseToString(PlanJobPhase phase) noexcept;

// Cancel arbitration result for one action cancel request. A cancel request
// binds to the exact goal UUID; it can never cancel whatever job happens to be
// current after a deactivation or a newer pending goal.
enum class CancelDecision : std::uint8_t
{
  // The exact current job is still precommit: set its cancel flag and ACCEPT.
  kAcceptPreCommit = 0,
  // The exact current job already linearized its snapshot commit. REJECT so
  // Humble's set_canceling cannot race the terminal succeed() transition.
  kRejectPostCommit,
  // No current job has this UUID (old/deactivation goal). REJECT without
  // touching the current job or its cancel token.
  kRejectStaleIdentity,
};

const char * CancelDecisionToString(CancelDecision decision) noexcept;

// Identity-aware release decision after the planning thread returns.
enum class ReleaseDecision : std::uint8_t
{
  kReleased = 0,
  // Terminality is unproven (or delivery is still in flight); keep the
  // admission reservation and the opaque goal-handle owner busy.
  kRetainedDeliveryFailed,
  // The job no longer owns delivery state (deactivated and superseded in the
  // planning phase). The caller may clear only its own scheduling slot, never a
  // newer pending one.
  kSuperseded,
};

const char * ReleaseDecisionToString(ReleaseDecision decision) noexcept;

enum class CommitDeliveryOutcome : std::uint8_t
{
  kTerminalSuccess = 0,
  // The success transport threw (any exception type), but an explicit
  // cleanup/query proved the SDK is already terminal. The goal may already have
  // SUCCEEDED and the transport may have sent a partial/full result before
  // throwing; only delivery confirmation is missing. Callers must report a
  // delivery error with known terminality, never a success and never a
  // fabricated abort/cancel.
  kDeliveryErrorKnownTerminal,
  // A deactivation/new generation superseded this job before DDS delivery.
  kSkippedStaleIdentity,
  // The success transport threw (any exception type) and terminality could not
  // be established. Keep the reservation and report an explicit
  // desynchronized-delivery error; do not claim whether a result was sent.
  kTransportUnresolved,
  // The atomic snapshot acceptance predicate or commit hook rejected the job.
  kCommitRejected,
  // The store was stopped or had no snapshot; nothing was delivered.
  kStoreUnavailable,
};

const char * CommitDeliveryOutcomeToString(
  CommitDeliveryOutcome outcome) noexcept;

// Transport and lifecycle hooks for one commit/delivery attempt.
//
// `accept_commit` runs under the SnapshotStore handoff mutex (store -> caller
// job lock) and must perform only a short in-memory check/transition. It
// receives a closure that records the committed phase under the helper state
// mutex. The hook MUST call that closure while it still holds its own job guard,
// after its epoch/cancel/identity checks, and return the closure's result. No
// SDK state query, action call, publication, store re-entry, or blocking is
// permitted here. This is what removes the old check-to-mark gap: the phase
// becomes kCommitted before the job guard is released.
//
// The other hooks run after CommitAndDeliver() has released the store handoff
// mutex and every caller job lock:
//   - terminal_success: action SDK succeed()/result delivery; may throw.
//   - terminal_failure: abort/cancel cleanup and/or terminal-state query after a
//     success-transport throw. It returns true only when it positively proved
//     the SDK goal is no longer active. A no-op, an exception, or an
//     unverifiable query returns false, so the reservation stays fail-closed.
//   - publish_path: private path DDS publication of the already-validated path;
//     may throw and never invalidates the action result.
struct CommitDeliveryHooks
{
  std::function<bool(const std::function<bool()> & mark_committed)>
  accept_commit;
  std::function<void()> terminal_success;
  std::function<bool()> terminal_failure;
  std::function<void()> publish_path;
};

struct CommitDeliveryResult
{
  CommitDeliveryOutcome outcome{CommitDeliveryOutcome::kCommitRejected};
  std::string detail;
  bool path_published{false};
};

// Result of invalidating the current delivery generation.
struct DeactivateResult
{
  // Set only when an uncommitted (planning) or already-terminal job was
  // superseded. A committed/delivering/failed job is retained instead.
  std::optional<PlanJobIdentity> superseded;
  // Opaque goal-handle owner handed back for destruction OUTSIDE every
  // store/job/state lock. Empty when the failed/committed owner is retained.
  std::shared_ptr<void> released_owner;
};

// ROS-independent production protocol that linearizes a validated plan at the
// immutable snapshot commit and then delivers the action result and private path
// outside every store/job lock.
//
// Ownership: the helper retains the opaque goal-handle owner for exactly as long
// as it owns the current identity. Humble's ServerGoalHandle destructor
// auto-cancels when the last reference is released, so dropping the owner while
// terminality is unproven would silently change the goal state. The owner is
// never captured in a stored lambda; it is moved out under the state mutex and
// destroyed by the caller after the lock is released.
//
// Deactivation: planning-phase deactivation supersedes the uncommitted job. A
// committed/delivering/failed job is retained (identity, phase, owner) so late
// cancel/release cannot untrack it; only private path publication is forbidden.
// This keeps admission busy until a terminal state is proven and lets an empty
// path clear suppress a late publication without losing actual ownership.
//
// Locking contract:
//   - `mutex_` protects only the phase/identity/owner bookkeeping and is held for
//     short, non-I/O transitions.
//   - `path_mutex_` serializes private path publication (including the empty
//     deactivation clear) across the actual DDS publish. It is only ever taken
//     without holding `mutex_`; inside it the state check takes `mutex_`
//     briefly, so the order is path -> state.
//   - CommitAndDeliver() takes store handoff -> caller job (inside accept hook)
//     -> `mutex_` for the in-memory commit, releases everything, and only then
//     runs the SDK/path hooks. No job/store lock is held across I/O.
//
// Validity: the plan is valid at the atomic commit linearization point. A map
// update after that point may be visible during DDS delivery; freshness at
// client receipt is not claimed and the map is never frozen until the SDK
// reply.
class PlanCommitDelivery
{
public:
  PlanCommitDelivery() = default;
  ~PlanCommitDelivery();

  PlanCommitDelivery(const PlanCommitDelivery &) = delete;
  PlanCommitDelivery & operator=(const PlanCommitDelivery &) = delete;

  // Registers the admitted job as kPlanning. Rejects a second live job so the
  // caller keeps at most one executing and bounded pending admission.
  //
  // `owner` is an opaque keepalive for the real action goal handle. The helper
  // retains it exactly while it owns the current identity so the SDK handle
  // cannot be destroyed (and implicitly auto-cancel) during an unresolved
  // delivery. It is never stored in a lambda. The final implementation must
  // hand it back for destruction outside every store/job/state lock.
  bool Admit(
    const PlanJobIdentity & identity,
    std::shared_ptr<void> owner = {});

  // Runs the atomic snapshot acceptance and, on success, the terminal delivery.
  // The returned detail is suitable for action diagnostics.
  CommitDeliveryResult CommitAndDeliver(
    SnapshotStore & store, const LatestCommitProof & proof,
    const PlanJobIdentity & identity, const CommitDeliveryHooks & hooks);

  // Resolves an action cancel request by exact goal UUID. The caller must hold
  // its job/scheduling lock while applying kAcceptPreCommit to the job's cancel
  // token.
  CancelDecision OnCancelRequested(const std::string & goal_uuid) const;

  // Invalidates the current generation. Planning-phase jobs are superseded;
  // committed/delivering/failed jobs retain their identity/phase/owner and only
  // lose private-path publication permission. The caller must publish the empty
  // path through PublishEmptyPath() after releasing its job lock so a blocked
  // old publication cannot land after the clear.
  DeactivateResult Deactivate();

  // Serializes an empty-path clear with every private path publication. An old
  // committed result that arrives later cannot publish after this clear.
  void PublishEmptyPath(const std::function<void()> & publish_empty);

  // Identity-aware release after the planning thread returns. If the helper
  // actually drops the opaque owner, it is moved into `released_owner` so the
  // caller destroys it after it has released its job lock. When no out
  // parameter is supplied the owner is destroyed after the helper state lock is
  // released.
  ReleaseDecision ReleaseActive(
    const PlanJobIdentity & identity,
    std::shared_ptr<void> * released_owner = nullptr);

  // Shutdown/restart escape hatch: drops all state and returns the opaque owner.
  // The caller MUST disable/reset the action server first, then destroy the
  // returned owner outside every lock, so Humble's weak server callback cannot
  // publish from the implicit goal-handle destructor during shutdown.
  std::shared_ptr<void> TakeOwnerForShutdown();

  PlanJobPhase phase() const;
  std::optional<PlanJobIdentity> current_identity() const;
  bool IsCurrent(const PlanJobIdentity & identity) const;

private:
  bool MarkCommitted(const PlanJobIdentity & identity);
  void RollBackCommitted(const PlanJobIdentity & identity);
  bool AttemptTerminalFailure(const CommitDeliveryHooks & hooks);
  // Shared terminal-evidence path for every transport exception type. Only an
  // explicit positive terminal proof moves the phase to kTerminal; otherwise
  // the reservation and the opaque owner are retained fail-closed.
  CommitDeliveryResult ResolveTransportFailure(
    const PlanJobIdentity & identity, const CommitDeliveryHooks & hooks,
    std::string reason);
  CommitDeliveryResult Deliver(
    const PlanJobIdentity & identity,
    const CommitDeliveryHooks & hooks);
  bool PublishPathIfCurrent(
    const PlanJobIdentity & identity,
    const std::function<void()> & publish_path,
    std::string * detail);

  mutable std::mutex mutex_;
  // Serializes path DDS publication and the empty clear. Never held with
  // `mutex_` in the opposite order.
  std::mutex path_mutex_;
  std::optional<PlanJobIdentity> current_;
  PlanJobPhase phase_{PlanJobPhase::kIdle};
  // Opaque keepalive for the real SDK goal handle while this helper owns the
  // current identity. Dropped only through the owner-aware release paths above.
  std::shared_ptr<void> owner_;
  // False after deactivation until a new job is admitted. A committed delivery
  // that was in flight at deactivation must not republish its private path.
  bool path_publish_allowed_{true};
};

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__PLAN_COMMIT_DELIVERY_H_
