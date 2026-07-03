#include "p2p_move_base/navigation_fault_policy.h"

#include <algorithm>
#include <cmath>

namespace p2p_move_base
{

NavigationFaultPolicy::NavigationFaultPolicy()
: NavigationFaultPolicy(Config())
{
}

NavigationFaultPolicy::NavigationFaultPolicy(const Config & config)
: config_(config),
  last_state_(dddmr_sys_core::PlannerState::TRAJECTORY_FOUND),
  has_last_state_(false),
  consecutive_cycles_(0),
  first_seen_sec_(0.0)
{
}

void NavigationFaultPolicy::setConfig(const Config & config)
{
  config_ = config;
}

const NavigationFaultPolicy::Config & NavigationFaultPolicy::config() const
{
  return config_;
}

void NavigationFaultPolicy::reset()
{
  has_last_state_ = false;
  consecutive_cycles_ = 0;
  first_seen_sec_ = 0.0;
  last_state_ = dddmr_sys_core::PlannerState::TRAJECTORY_FOUND;
}

void NavigationFaultPolicy::updateStreak(dddmr_sys_core::PlannerState state, double now_sec)
{
  if(!std::isfinite(now_sec)){
    now_sec = 0.0;
  }

  if(!has_last_state_ || state != last_state_){
    last_state_ = state;
    has_last_state_ = true;
    consecutive_cycles_ = 1;
    first_seen_sec_ = now_sec;
    return;
  }

  ++consecutive_cycles_;
}

double NavigationFaultPolicy::currentDuration(double now_sec) const
{
  if(!std::isfinite(now_sec) || !has_last_state_){
    return 0.0;
  }
  return std::max(0.0, now_sec - first_seen_sec_);
}

bool NavigationFaultPolicy::replanThrottleSatisfied(double now_sec, double last_replan_sec) const
{
  if(config_.min_replan_interval_sec <= 0.0){
    return true;
  }
  if(!std::isfinite(now_sec) || !std::isfinite(last_replan_sec)){
    return false;
  }
  return now_sec - last_replan_sec >= config_.min_replan_interval_sec;
}

FaultDecision NavigationFaultPolicy::update(
  dddmr_sys_core::PlannerState state,
  bool allow_blocked_wait,
  double now_sec,
  double last_replan_sec,
  double controller_failure_elapsed_sec)
{
  if(state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
    reset();
    return FaultDecision{};
  }

  updateStreak(state, now_sec);
  const double duration = currentDuration(now_sec);

  FaultDecision decision;
  decision.consecutive_cycles = consecutive_cycles_;
  decision.duration_sec = duration;

  if(state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
    decision.action = FaultAction::TemporaryStop;
    decision.reason = NavigationReason::PerceptionStale;
    decision.message = "perception_stale: pause without replanning";
    return decision;
  }

  if(state == dddmr_sys_core::PlannerState::TF_FAIL){
    decision.action = FaultAction::TemporaryStop;
    decision.reason = NavigationReason::TfStale;
    decision.message = "tf_stale: pause without replanning";
    return decision;
  }

  if(state == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
    if(controller_failure_elapsed_sec > config_.control_failure_patience_sec){
      decision.action = FaultAction::RecoveryOrRouteRefresh;
      decision.reason = NavigationReason::TrackingControllerStalled;
      decision.message = "tracking_controller_stalled: controller stalled";
      return decision;
    }
    decision.action = FaultAction::TemporaryStop;
    decision.reason = NavigationReason::LocalObstacleDetected;
    decision.message = "controller_temporarily_failed: keep current route";
    return decision;
  }

  if(state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
    decision.action = allow_blocked_wait ? FaultAction::WaitForClearance : FaultAction::TemporaryStop;
    decision.reason = NavigationReason::LocalObstacleDetected;
    decision.message = "path_blocked_wait: wait for clearance";
    return decision;
  }

  if(state == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
    const bool confirmed =
      consecutive_cycles_ >= config_.collision_confirm_cycles &&
      duration >= config_.blocked_confirm_time_sec;
    if(!confirmed){
      decision.action = FaultAction::WaitForClearance;
      decision.reason = NavigationReason::LocalObstacleDetected;
      decision.message = "path_blocked_unconfirmed: wait before global replan";
      return decision;
    }
    if(!replanThrottleSatisfied(now_sec, last_replan_sec)){
      decision.action = FaultAction::WaitForClearance;
      decision.reason = NavigationReason::ReplanThrottled;
      decision.message = "path_blocked_confirmed: replan throttled";
      return decision;
    }
    decision.action = FaultAction::RouteRefresh;
    decision.reason = NavigationReason::StaticBlockageConfirmed;
    decision.allows_global_replan = true;
    decision.message = "route_blocked: static blockage confirmed";
    return decision;
  }

  if(state == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
    const bool confirmed =
      consecutive_cycles_ >= config_.route_deviation_confirm_cycles &&
      duration >= config_.route_deviation_confirm_time_sec;
    if(!confirmed){
      decision.action = FaultAction::TemporaryStop;
      decision.reason = NavigationReason::RouteDeviationSuspected;
      decision.message = "route_deviation_suspected: wait for confirmation";
      return decision;
    }
    if(!replanThrottleSatisfied(now_sec, last_replan_sec)){
      decision.action = FaultAction::TemporaryStop;
      decision.reason = NavigationReason::ReplanThrottled;
      decision.message = "route_deviation_confirmed: replan throttled";
      return decision;
    }
    decision.action = FaultAction::RouteRefresh;
    decision.reason = NavigationReason::RouteDeviationConfirmed;
    decision.allows_global_replan = true;
    decision.message = "route_deviation: local reference prune failed";
    return decision;
  }

  decision.action = FaultAction::TemporaryStop;
  decision.reason = NavigationReason::UnhandledPlannerState;
  decision.message = "unhandled_planner_state: stop safely";
  return decision;
}

const char * toString(FaultAction action)
{
  switch(action){
    case FaultAction::None:
      return "none";
    case FaultAction::TemporaryStop:
      return "temporary_stop";
    case FaultAction::WaitForClearance:
      return "wait_for_clearance";
    case FaultAction::RouteRefresh:
      return "route_refresh";
    case FaultAction::RecoveryOrRouteRefresh:
      return "recovery_or_route_refresh";
    case FaultAction::FatalAbort:
      return "fatal_abort";
  }
  return "unknown";
}

const char * toString(NavigationReason reason)
{
  switch(reason){
    case NavigationReason::None:
      return "none";
    case NavigationReason::LocalObstacleDetected:
      return "local_obstacle_detected";
    case NavigationReason::StaticBlockageConfirmed:
      return "static_blockage_confirmed";
    case NavigationReason::ReplanThrottled:
      return "replan_throttled";
    case NavigationReason::RouteDeviationSuspected:
      return "route_deviation_suspected";
    case NavigationReason::RouteDeviationConfirmed:
      return "route_deviation_confirmed";
    case NavigationReason::TrackingControllerStalled:
      return "tracking_controller_stalled";
    case NavigationReason::PerceptionStale:
      return "perception_stale";
    case NavigationReason::TfStale:
      return "tf_stale";
    case NavigationReason::UnhandledPlannerState:
      return "unhandled_planner_state";
  }
  return "unknown";
}

}  // namespace p2p_move_base
