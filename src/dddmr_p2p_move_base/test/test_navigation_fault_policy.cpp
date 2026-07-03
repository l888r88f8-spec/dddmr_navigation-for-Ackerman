#include <gtest/gtest.h>

#include "p2p_move_base/navigation_fault_policy.h"

using dddmr_sys_core::PlannerState;
using p2p_move_base::FaultAction;
using p2p_move_base::NavigationFaultPolicy;
using p2p_move_base::NavigationReason;

namespace
{
NavigationFaultPolicy::Config testConfig()
{
  NavigationFaultPolicy::Config config;
  config.collision_confirm_cycles = 3;
  config.blocked_confirm_time_sec = 1.0;
  config.min_replan_interval_sec = 2.0;
  config.route_deviation_confirm_cycles = 2;
  config.route_deviation_confirm_time_sec = 0.5;
  config.control_failure_patience_sec = 3.0;
  return config;
}
}  // namespace

TEST(NavigationFaultPolicyTest, SingleCollisionWaitsInsteadOfReplanning)
{
  NavigationFaultPolicy policy(testConfig());
  const auto decision = policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.0, 0.0);

  EXPECT_EQ(decision.action, FaultAction::WaitForClearance);
  EXPECT_EQ(decision.reason, NavigationReason::LocalObstacleDetected);
  EXPECT_FALSE(decision.allows_global_replan);
}

TEST(NavigationFaultPolicyTest, ConfirmedCollisionCanReplanAfterThrottleWindow)
{
  NavigationFaultPolicy policy(testConfig());

  EXPECT_EQ(policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.0, 0.0).action,
    FaultAction::WaitForClearance);
  EXPECT_EQ(policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.4, 0.0).action,
    FaultAction::WaitForClearance);
  const auto decision = policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 11.1, 8.0);

  EXPECT_EQ(decision.action, FaultAction::RouteRefresh);
  EXPECT_EQ(decision.reason, NavigationReason::StaticBlockageConfirmed);
  EXPECT_TRUE(decision.allows_global_replan);
}

TEST(NavigationFaultPolicyTest, ReplanThrottleKeepsConfirmedCollisionWaiting)
{
  NavigationFaultPolicy policy(testConfig());

  policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.0, 0.0);
  policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.5, 0.0);
  const auto decision = policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 11.2, 10.0);

  EXPECT_EQ(decision.action, FaultAction::WaitForClearance);
  EXPECT_EQ(decision.reason, NavigationReason::ReplanThrottled);
  EXPECT_FALSE(decision.allows_global_replan);
}

TEST(NavigationFaultPolicyTest, RouteDeviationRequiresConsecutiveConfirmation)
{
  NavigationFaultPolicy policy(testConfig());

  EXPECT_EQ(policy.update(PlannerState::PRUNE_PLAN_FAIL, true, 20.0, 0.0).action,
    FaultAction::TemporaryStop);
  const auto decision = policy.update(PlannerState::PRUNE_PLAN_FAIL, true, 20.6, 0.0);

  EXPECT_EQ(decision.action, FaultAction::RouteRefresh);
  EXPECT_EQ(decision.reason, NavigationReason::RouteDeviationConfirmed);
  EXPECT_TRUE(decision.allows_global_replan);
}

TEST(NavigationFaultPolicyTest, AllTrajectoriesFailEscalatesAfterControllerPatience)
{
  NavigationFaultPolicy policy(testConfig());

  EXPECT_EQ(policy.update(PlannerState::ALL_TRAJECTORIES_FAIL, true, 30.0, 0.0, 2.9).action,
    FaultAction::TemporaryStop);
  const auto decision = policy.update(PlannerState::ALL_TRAJECTORIES_FAIL, true, 30.1, 0.0, 3.1);

  EXPECT_EQ(decision.action, FaultAction::RecoveryOrRouteRefresh);
  EXPECT_EQ(decision.reason, NavigationReason::TrackingControllerStalled);
}

TEST(NavigationFaultPolicyTest, SuccessResetsFaultStreak)
{
  NavigationFaultPolicy policy(testConfig());

  policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.0, 0.0);
  policy.update(PlannerState::TRAJECTORY_FOUND, true, 10.1, 0.0);
  const auto decision = policy.update(PlannerState::PATH_BLOCKED_REPLANNING, true, 10.2, 0.0);

  EXPECT_EQ(decision.action, FaultAction::WaitForClearance);
  EXPECT_FALSE(decision.allows_global_replan);
}
