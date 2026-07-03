#ifndef P2P_MOVE_BASE_NAVIGATION_FAULT_POLICY_H_
#define P2P_MOVE_BASE_NAVIGATION_FAULT_POLICY_H_

#include <cstddef>
#include <string>

#include "dddmr_sys_core/dddmr_enum_states.h"

namespace p2p_move_base
{

enum class FaultAction
{
  None,
  TemporaryStop,
  WaitForClearance,
  RouteRefresh,
  RecoveryOrRouteRefresh,
  FatalAbort
};

enum class NavigationReason
{
  None,
  LocalObstacleDetected,
  StaticBlockageConfirmed,
  ReplanThrottled,
  RouteDeviationSuspected,
  RouteDeviationConfirmed,
  TrackingControllerStalled,
  PerceptionStale,
  TfStale,
  UnhandledPlannerState
};

struct FaultDecision
{
  FaultAction action = FaultAction::None;
  NavigationReason reason = NavigationReason::None;
  bool allows_global_replan = false;
  std::size_t consecutive_cycles = 0;
  double duration_sec = 0.0;
  std::string message;
};

class NavigationFaultPolicy
{
public:
  struct Config
  {
    std::size_t collision_confirm_cycles = 5;
    double blocked_confirm_time_sec = 1.0;
    double min_replan_interval_sec = 3.0;
    std::size_t route_deviation_confirm_cycles = 10;
    double route_deviation_confirm_time_sec = 1.0;
    double control_failure_patience_sec = 3.0;
  };

  NavigationFaultPolicy();
  explicit NavigationFaultPolicy(const Config & config);

  void setConfig(const Config & config);
  const Config & config() const;
  void reset();

  FaultDecision update(
    dddmr_sys_core::PlannerState state,
    bool allow_blocked_wait,
    double now_sec,
    double last_replan_sec,
    double controller_failure_elapsed_sec = 0.0);

private:
  Config config_;
  dddmr_sys_core::PlannerState last_state_;
  bool has_last_state_;
  std::size_t consecutive_cycles_;
  double first_seen_sec_;

  void updateStreak(dddmr_sys_core::PlannerState state, double now_sec);
  double currentDuration(double now_sec) const;
  bool replanThrottleSatisfied(double now_sec, double last_replan_sec) const;
};

const char * toString(FaultAction action);
const char * toString(NavigationReason reason);

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_NAVIGATION_FAULT_POLICY_H_
