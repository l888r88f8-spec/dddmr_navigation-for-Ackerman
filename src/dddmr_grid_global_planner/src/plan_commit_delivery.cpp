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

#include "grid_global_planner/plan_commit_delivery.h"

#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace grid_global_planner
{

const char * PlanJobPhaseToString(PlanJobPhase phase) noexcept
{
  switch (phase) {
    case PlanJobPhase::kIdle:
      return "idle";
    case PlanJobPhase::kPlanning:
      return "planning";
    case PlanJobPhase::kCommitted:
      return "committed";
    case PlanJobPhase::kDelivering:
      return "delivering";
    case PlanJobPhase::kTerminal:
      return "terminal";
    case PlanJobPhase::kDeliveryFailed:
      return "delivery_failed";
  }
  return "unknown";
}

const char * CancelDecisionToString(CancelDecision decision) noexcept
{
  switch (decision) {
    case CancelDecision::kAcceptPreCommit:
      return "accept_precommit";
    case CancelDecision::kRejectPostCommit:
      return "reject_postcommit";
    case CancelDecision::kRejectStaleIdentity:
      return "reject_stale_identity";
  }
  return "unknown";
}

const char * ReleaseDecisionToString(ReleaseDecision decision) noexcept
{
  switch (decision) {
    case ReleaseDecision::kReleased:
      return "released";
    case ReleaseDecision::kRetainedDeliveryFailed:
      return "retained_delivery_failed";
    case ReleaseDecision::kSuperseded:
      return "superseded";
  }
  return "unknown";
}

const char * CommitDeliveryOutcomeToString(
  CommitDeliveryOutcome outcome) noexcept
{
  switch (outcome) {
    case CommitDeliveryOutcome::kTerminalSuccess:
      return "terminal_success";
    case CommitDeliveryOutcome::kDeliveryErrorKnownTerminal:
      return "delivery_error_known_terminal";
    case CommitDeliveryOutcome::kSkippedStaleIdentity:
      return "skipped_stale_identity";
    case CommitDeliveryOutcome::kTransportUnresolved:
      return "transport_unresolved";
    case CommitDeliveryOutcome::kCommitRejected:
      return "commit_rejected";
    case CommitDeliveryOutcome::kStoreUnavailable:
      return "store_unavailable";
  }
  return "unknown";
}

PlanCommitDelivery::~PlanCommitDelivery()
{
  std::shared_ptr<void> owner;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    owner = std::move(owner_);
    current_.reset();
    phase_ = PlanJobPhase::kIdle;
    path_publish_allowed_ = true;
  }
  // The owner is intentionally destroyed after the state lock is released. The
  // node must have reset its action server before this destructor so an implicit
  // ServerGoalHandle destructor cannot publish through a live weak server.
}

bool PlanCommitDelivery::Admit(
  const PlanJobIdentity & identity,
  std::shared_ptr<void> owner)
{
  bool admitted = false;
  std::shared_ptr<void> rejected_owner;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_.has_value()) {
      // Defer destruction of the caller-supplied owner until after the lock.
      rejected_owner = std::move(owner);
    } else {
      current_ = identity;
      phase_ = PlanJobPhase::kPlanning;
      path_publish_allowed_ = true;
      owner_ = std::move(owner);
      admitted = true;
    }
  }
  return admitted;
}

CommitDeliveryResult PlanCommitDelivery::CommitAndDeliver(
  SnapshotStore & store, const LatestCommitProof & proof,
  const PlanJobIdentity & identity, const CommitDeliveryHooks & hooks)
{
  CommitDeliveryResult result;
  if (!hooks.accept_commit || !hooks.terminal_success) {
    result.outcome = CommitDeliveryOutcome::kCommitRejected;
    result.detail = "commit/delivery hooks are incomplete";
    return result;
  }

  std::string predicate_detail;
  std::string accept_detail;
  bool marked_committed = false;
  const SnapshotCommitStatus status = store.CommitLatest(
    [&](const MapSnapshot & latest) {
      if (LatestCommitProofMatches(proof, latest)) {
        return true;
      }
      predicate_detail =
      proof.kind == LatestCommitProofKind::kWholeSnapshot ?
      "validated whole snapshot changed at commit" :
      "relevant corridor changed at commit";
      return false;
    },
    [&](const MapSnapshot &) {
      // Short in-memory acceptance only. The caller's hook holds its job guard
      // for the epoch/cancel/identity check AND the committed-phase mark; this
      // helper records the exact committed identity under the same acceptance
      // callback. No SDK query/DDS call is made here.
      auto mark_committed = [&]() {
        const bool marked = MarkCommitted(identity);
        if (marked) {
          marked_committed = true;
        }
        return marked;
      };
      if (!hooks.accept_commit(mark_committed)) {
        if (marked_committed) {
          // A contract-violating hook marked the phase and then rejected.
          // The store did not accept the commit, so undo the in-memory mark.
          RollBackCommitted(identity);
          accept_detail =
          "commit acceptance marked then rejected the terminal state";
        } else {
          accept_detail =
          "commit acceptance hook rejected the terminal state";
        }
        return false;
      }
      if (!marked_committed) {
        accept_detail =
        "commit acceptance did not linearize the committed phase";
        return false;
      }
      return true;
    });

  switch (status) {
    case SnapshotCommitStatus::kCommitted:
      // The store handoff mutex is released here. Delivery runs outside the
      // store and every caller job lock.
      return Deliver(identity, hooks);
    case SnapshotCommitStatus::kStopped:
      result.outcome = CommitDeliveryOutcome::kStoreUnavailable;
      result.detail = "snapshot store is stopped";
      return result;
    case SnapshotCommitStatus::kNoSnapshot:
      result.outcome = CommitDeliveryOutcome::kStoreUnavailable;
      result.detail = "latest snapshot is not usable at commit";
      return result;
    case SnapshotCommitStatus::kPredicateRejected:
      result.outcome = CommitDeliveryOutcome::kCommitRejected;
      result.detail = predicate_detail.empty() ?
        "latest snapshot rejected the commit proof" :
        predicate_detail;
      return result;
    case SnapshotCommitStatus::kCallbackRejected:
      result.outcome = CommitDeliveryOutcome::kCommitRejected;
      result.detail = accept_detail.empty() ?
        "commit callback rejected the terminal transition" :
        accept_detail;
      return result;
  }
  result.outcome = CommitDeliveryOutcome::kCommitRejected;
  result.detail = "final commit failed";
  return result;
}

CancelDecision PlanCommitDelivery::OnCancelRequested(
  const std::string & goal_uuid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_.has_value() || current_->goal_uuid != goal_uuid) {
    return CancelDecision::kRejectStaleIdentity;
  }
  if (phase_ == PlanJobPhase::kPlanning) {
    return CancelDecision::kAcceptPreCommit;
  }
  return CancelDecision::kRejectPostCommit;
}

DeactivateResult PlanCommitDelivery::Deactivate()
{
  DeactivateResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value()) {
      return result;
    }
    if (phase_ == PlanJobPhase::kPlanning ||
      phase_ == PlanJobPhase::kTerminal)
    {
      // An uncommitted job may be superseded; a terminal job is already safe to
      // forget. Hand the opaque owner back for destruction outside every lock.
      result.superseded = std::move(current_);
      current_.reset();
      phase_ = PlanJobPhase::kIdle;
      path_publish_allowed_ = true;
      result.released_owner = std::move(owner_);
    } else {
      // Committed/delivering/failed ownership is retained. Deactivation only
      // revokes private-path publication so a late DDS path cannot resurrect
      // after the empty clear.
      path_publish_allowed_ = false;
    }
  }
  return result;
}

void PlanCommitDelivery::PublishEmptyPath(
  const std::function<void()> & publish_empty)
{
  if (!publish_empty) {
    return;
  }
  // The path lock is held across the actual DDS clear. A committed result that
  // is still blocked in delivery can therefore either publish before this clear
  // or be suppressed by the generation check, never resurrect afterwards.
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  publish_empty();
}

ReleaseDecision PlanCommitDelivery::ReleaseActive(
  const PlanJobIdentity & identity, std::shared_ptr<void> * released_owner)
{
  std::shared_ptr<void> dropped_owner;
  ReleaseDecision decision = ReleaseDecision::kSuperseded;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value() || *current_ != identity) {
      decision = ReleaseDecision::kSuperseded;
    } else if (phase_ == PlanJobPhase::kDeliveryFailed ||
      phase_ == PlanJobPhase::kCommitted ||
      phase_ == PlanJobPhase::kDelivering)
    {
      // Terminality is unproven or delivery is still in flight. Keep the
      // identity and the real goal-handle owner so admission remains busy and
      // a later release attempt can still observe the reservation.
      return ReleaseDecision::kRetainedDeliveryFailed;
    } else {
      dropped_owner = std::move(owner_);
      current_.reset();
      phase_ = PlanJobPhase::kIdle;
      path_publish_allowed_ = true;
      decision = ReleaseDecision::kReleased;
    }
  }
  if (released_owner != nullptr && dropped_owner) {
    *released_owner = std::move(dropped_owner);
  }
  // When no out parameter is supplied, dropped_owner is destroyed here after
  // the state lock has been released.
  return decision;
}

std::shared_ptr<void> PlanCommitDelivery::TakeOwnerForShutdown()
{
  std::shared_ptr<void> owner;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    owner = std::move(owner_);
    current_.reset();
    phase_ = PlanJobPhase::kIdle;
    path_publish_allowed_ = true;
  }
  // The caller destroys the returned owner after resetting the action server
  // and outside every lock.
  return owner;
}

PlanJobPhase PlanCommitDelivery::phase() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return phase_;
}

std::optional<PlanJobIdentity> PlanCommitDelivery::current_identity() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_;
}

bool PlanCommitDelivery::IsCurrent(const PlanJobIdentity & identity) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_.has_value() && *current_ == identity;
}

bool PlanCommitDelivery::MarkCommitted(const PlanJobIdentity & identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_.has_value() || *current_ != identity ||
    phase_ != PlanJobPhase::kPlanning)
  {
    return false;
  }
  phase_ = PlanJobPhase::kCommitted;
  return true;
}

void PlanCommitDelivery::RollBackCommitted(const PlanJobIdentity & identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_.has_value() && *current_ == identity &&
    phase_ == PlanJobPhase::kCommitted)
  {
    phase_ = PlanJobPhase::kPlanning;
  }
}

bool PlanCommitDelivery::AttemptTerminalFailure(
  const CommitDeliveryHooks & hooks)
{
  if (!hooks.terminal_failure) {
    return false;
  }
  try {
    // Only an explicit positive evidence result counts. A no-op cleanup that
    // returns normally, a false result, or any query/abort exception keeps the
    // failed reservation.
    return hooks.terminal_failure();
  } catch (...) {
    return false;
  }
}

CommitDeliveryResult PlanCommitDelivery::ResolveTransportFailure(
  const PlanJobIdentity & identity, const CommitDeliveryHooks & hooks,
  std::string reason)
{
  CommitDeliveryResult result;
  const bool terminal_confirmed = AttemptTerminalFailure(hooks);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_.has_value() && *current_ == identity) {
      phase_ = terminal_confirmed ? PlanJobPhase::kTerminal :
        PlanJobPhase::kDeliveryFailed;
    }
  }
  if (terminal_confirmed) {
    result.outcome = CommitDeliveryOutcome::kDeliveryErrorKnownTerminal;
    result.detail =
      "action success delivery threw; delivery not confirmed (SDK raised, "
      "terminality known): " +
      reason;
  } else {
    result.outcome = CommitDeliveryOutcome::kTransportUnresolved;
    result.detail =
      "action success delivery threw; delivery not confirmed and "
      "terminality is unproven; reservation retained (SDK raised): " +
      reason;
  }
  return result;
}

CommitDeliveryResult PlanCommitDelivery::Deliver(
  const PlanJobIdentity & identity, const CommitDeliveryHooks & hooks)
{
  CommitDeliveryResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value() || *current_ != identity ||
      phase_ != PlanJobPhase::kCommitted)
    {
      result.outcome = CommitDeliveryOutcome::kSkippedStaleIdentity;
      result.detail = "job was superseded before DDS delivery";
      return result;
    }
    phase_ = PlanJobPhase::kDelivering;
  }

  // No store, job, or coordinator lock is held across the action SDK call.
  // Every exception type takes the same explicit terminal-evidence path; a
  // non-std::exception must not escape and release the owner/reservation in the
  // node's outer finish path.
  try {
    hooks.terminal_success();
  } catch (const std::exception & exception) {
    return ResolveTransportFailure(identity, hooks, exception.what());
  } catch (...) {
    return ResolveTransportFailure(identity, hooks, "non-standard exception");
  }

  std::string publish_detail;
  const bool published =
    PublishPathIfCurrent(identity, hooks.publish_path, &publish_detail);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_.has_value() && *current_ == identity) {
      phase_ = PlanJobPhase::kTerminal;
    }
  }
  result.outcome = CommitDeliveryOutcome::kTerminalSuccess;
  result.path_published = published;
  result.detail = std::move(publish_detail);
  return result;
}

bool PlanCommitDelivery::PublishPathIfCurrent(
  const PlanJobIdentity & identity,
  const std::function<void()> & publish_path, std::string * detail)
{
  if (!publish_path) {
    if (detail != nullptr) {
      *detail = "no private path publisher configured";
    }
    return false;
  }
  // Path -> state lock order. The path lock is never acquired while the state
  // lock is held, and no job/store lock is held here.
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value() || *current_ != identity ||
      !path_publish_allowed_ ||
      (phase_ != PlanJobPhase::kDelivering &&
      phase_ != PlanJobPhase::kTerminal))
    {
      if (detail != nullptr) {
        *detail = "private path suppressed for a superseded job generation";
      }
      return false;
    }
  }
  try {
    publish_path();
  } catch (const std::exception & exception) {
    if (detail != nullptr) {
      *detail = "private path publish failed: " + std::string(exception.what());
    }
    return false;
  } catch (...) {
    if (detail != nullptr) {
      *detail = "private path publish failed";
    }
    return false;
  }
  return true;
}

}  // namespace grid_global_planner
