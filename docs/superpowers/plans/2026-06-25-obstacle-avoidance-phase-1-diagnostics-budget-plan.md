# Obstacle Avoidance Phase 1 Diagnostics And Budget Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first safe increment of the ground-robot obstacle-avoidance refactor: structured replan debounce, explicit reason codes, runtime budget monitoring, and diagnostics without replacing the local planner yet.

**Architecture:** This phase adds small pure-C++ policy components that can be unit-tested outside ROS callbacks, then wires them into `P2PMoveBase::executeCycle`. It keeps the existing RPP/local planner behavior as the motion source, but prevents single-cycle obstacle/replan triggers from immediately requesting global planning and adds Orin NX budget observability.

**Tech Stack:** ROS2 Humble, ament_cmake, C++17, gtest via `ament_cmake_gtest`, existing `p2p_move_base`, `local_planner`, and `dddmr_sys_core`.

---

## Scope

This plan implements Phase 1 only. It does not implement `LocalObstacleLayer`, Ackermann State Lattice, dynamic obstacle tracking, or two-stage global planning. Those become later plans after this phase gives reliable diagnostics and replan control.

The Phase 1 deliverable is intentionally small:

```text
planner state from local_planner
  -> NavigationFaultPolicy
  -> temporary stop / blocked wait / route refresh / recovery decision
  -> RuntimeBudgetMonitor diagnostics
```

## Files

- Modify: `src/dddmr_p2p_move_base/CMakeLists.txt`
  - Add test infrastructure.
  - Add new helper libraries to `p2p_mov2_base`.
- Modify: `src/dddmr_p2p_move_base/package.xml`
  - Add `ament_cmake_gtest` test dependency.
- Create: `src/dddmr_p2p_move_base/include/p2p_move_base/navigation_fault_policy.h`
  - Pure C++ policy for debounce and reason-code decisions.
- Create: `src/dddmr_p2p_move_base/src/navigation_fault_policy.cpp`
  - Implementation of the policy.
- Create: `src/dddmr_p2p_move_base/test/test_navigation_fault_policy.cpp`
  - Unit tests for collision debounce, blocked wait, replan throttle, route deviation handling, and recovery escalation.
- Create: `src/dddmr_p2p_move_base/include/p2p_move_base/runtime_budget_monitor.h`
  - Pure C++ runtime budget tracker for local cycle timing and resource counters.
- Create: `src/dddmr_p2p_move_base/src/runtime_budget_monitor.cpp`
  - Implementation of budget status and degradation level calculation.
- Create: `src/dddmr_p2p_move_base/test/test_runtime_budget_monitor.cpp`
  - Unit tests for budget OK, soft overrun, persistent overrun, and reset behavior.
- Modify: `src/dddmr_p2p_move_base/include/p2p_move_base/p2p_move_base.h`
  - Add policy and budget monitor members.
- Modify: `src/dddmr_p2p_move_base/src/p2p_move_base.cpp`
  - Declare/read parameters.
  - Route planner faults through `NavigationFaultPolicy`.
  - Record cycle runtime and log budget diagnostics.
- Modify: `src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml`
  - Add conservative Phase 1 parameters.

## Roadmap After Phase 1

Later plans should be executed in this order:

1. Phase 2: Extract `LocalObstacleLayer` snapshot model with ROI crop and point budget.
2. Phase 3: Add Ackermann State Lattice local planner behind a runtime switch.
3. Phase 4: Split `NavigationSupervisor` out of `P2PMoveBase`.
4. Phase 5: Add fast/refined global planning backend selection.
5. Phase 6: Add dynamic obstacle tracking and prediction.

## Task 1: Add Unit Test Infrastructure

**Files:**
- Modify: `src/dddmr_p2p_move_base/CMakeLists.txt`
- Modify: `src/dddmr_p2p_move_base/package.xml`

- [ ] **Step 1: Add gtest test dependency to package.xml**

Add this line in `package.xml` near the existing test dependencies:

```xml
  <test_depend>ament_cmake_gtest</test_depend>
```

- [ ] **Step 2: Add gtest support to CMakeLists.txt**

Add this near the top after the existing `find_package(ament_cmake REQUIRED)`:

```cmake
find_package(ament_cmake_gtest REQUIRED)
```

Add these source files to `p2p_mov2_base` only after Tasks 2 and 4 create them:

```cmake
add_library(p2p_navigation_fault_policy SHARED
  src/navigation_fault_policy.cpp
)
ament_target_dependencies(p2p_navigation_fault_policy
  dddmr_sys_core
)

add_library(p2p_runtime_budget_monitor SHARED
  src/runtime_budget_monitor.cpp
)
ament_target_dependencies(p2p_runtime_budget_monitor
  rclcpp
)
```

Update `target_link_libraries(p2p_mov2_base ...)` to:

```cmake
target_link_libraries(p2p_mov2_base
  p2p_fsm
  p2p_global_plan_manager
  p2p_navigation_fault_policy
  p2p_runtime_budget_monitor
)
```

Add both helper libraries to install targets:

```cmake
install(TARGETS
  p2p_fsm
  p2p_mov2_base
  p2p_global_plan_manager
  p2p_navigation_fault_policy
  p2p_runtime_budget_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

Add this before `ament_package()`:

```cmake
if(BUILD_TESTING)
  ament_add_gtest(test_navigation_fault_policy
    test/test_navigation_fault_policy.cpp
  )
  target_link_libraries(test_navigation_fault_policy
    p2p_navigation_fault_policy
  )
  ament_target_dependencies(test_navigation_fault_policy
    dddmr_sys_core
  )

  ament_add_gtest(test_runtime_budget_monitor
    test/test_runtime_budget_monitor.cpp
  )
  target_link_libraries(test_runtime_budget_monitor
    p2p_runtime_budget_monitor
  )
  ament_target_dependencies(test_runtime_budget_monitor
    rclcpp
  )
endif()
```

- [ ] **Step 3: Run configure/build expecting failure before source files exist**

Run:

```bash
colcon build --packages-select p2p_move_base --cmake-args -DBUILD_TESTING=ON
```

Expected: FAIL because `src/navigation_fault_policy.cpp` and `src/runtime_budget_monitor.cpp` do not exist yet.

- [ ] **Step 4: Commit after helper source files and tests are added in later tasks**

Do not commit Task 1 alone if the package does not configure. Commit test infrastructure together with Task 2 and Task 4 helper files.

## Task 2: Add NavigationFaultPolicy Tests

**Files:**
- Create: `src/dddmr_p2p_move_base/test/test_navigation_fault_policy.cpp`

- [ ] **Step 1: Write failing tests**

Create `test/test_navigation_fault_policy.cpp`:

```cpp
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
```

- [ ] **Step 2: Run test expecting missing header failure**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args -R test_navigation_fault_policy --output-on-failure
```

Expected: FAIL with a compile error mentioning `p2p_move_base/navigation_fault_policy.h` is missing.

## Task 3: Implement NavigationFaultPolicy

**Files:**
- Create: `src/dddmr_p2p_move_base/include/p2p_move_base/navigation_fault_policy.h`
- Create: `src/dddmr_p2p_move_base/src/navigation_fault_policy.cpp`

- [ ] **Step 1: Create the header**

Create `include/p2p_move_base/navigation_fault_policy.h`:

```cpp
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

  explicit NavigationFaultPolicy(const Config & config = Config());

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
```

- [ ] **Step 2: Create the implementation**

Create `src/navigation_fault_policy.cpp`:

```cpp
#include "p2p_move_base/navigation_fault_policy.h"

#include <algorithm>
#include <cmath>

namespace p2p_move_base
{

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
    case FaultAction::None: return "none";
    case FaultAction::TemporaryStop: return "temporary_stop";
    case FaultAction::WaitForClearance: return "wait_for_clearance";
    case FaultAction::RouteRefresh: return "route_refresh";
    case FaultAction::RecoveryOrRouteRefresh: return "recovery_or_route_refresh";
    case FaultAction::FatalAbort: return "fatal_abort";
  }
  return "unknown";
}

const char * toString(NavigationReason reason)
{
  switch(reason){
    case NavigationReason::None: return "none";
    case NavigationReason::LocalObstacleDetected: return "local_obstacle_detected";
    case NavigationReason::StaticBlockageConfirmed: return "static_blockage_confirmed";
    case NavigationReason::ReplanThrottled: return "replan_throttled";
    case NavigationReason::RouteDeviationSuspected: return "route_deviation_suspected";
    case NavigationReason::RouteDeviationConfirmed: return "route_deviation_confirmed";
    case NavigationReason::TrackingControllerStalled: return "tracking_controller_stalled";
    case NavigationReason::PerceptionStale: return "perception_stale";
    case NavigationReason::TfStale: return "tf_stale";
    case NavigationReason::UnhandledPlannerState: return "unhandled_planner_state";
  }
  return "unknown";
}

}  // namespace p2p_move_base
```

- [ ] **Step 3: Run tests**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args -R test_navigation_fault_policy --output-on-failure
```

Expected: PASS for `test_navigation_fault_policy`.

- [ ] **Step 4: Commit**

Run:

```bash
git add src/dddmr_p2p_move_base/package.xml \
  src/dddmr_p2p_move_base/CMakeLists.txt \
  src/dddmr_p2p_move_base/include/p2p_move_base/navigation_fault_policy.h \
  src/dddmr_p2p_move_base/src/navigation_fault_policy.cpp \
  src/dddmr_p2p_move_base/test/test_navigation_fault_policy.cpp
git commit -m "feat: add navigation fault debounce policy"
```

## Task 4: Add RuntimeBudgetMonitor Tests

**Files:**
- Create: `src/dddmr_p2p_move_base/test/test_runtime_budget_monitor.cpp`

- [ ] **Step 1: Write failing tests**

Create `test/test_runtime_budget_monitor.cpp`:

```cpp
#include <gtest/gtest.h>

#include "p2p_move_base/runtime_budget_monitor.h"

using p2p_move_base::BudgetLevel;
using p2p_move_base::RuntimeBudgetMonitor;
using p2p_move_base::RuntimeBudgetSample;

namespace
{
RuntimeBudgetMonitor::Config testConfig()
{
  RuntimeBudgetMonitor::Config config;
  config.max_local_planning_time_ms = 50.0;
  config.max_obstacle_update_time_ms = 50.0;
  config.max_end_to_end_local_cycle_ms = 80.0;
  config.max_candidate_trajectories = 80;
  config.max_dynamic_obstacles = 48;
  config.max_points_per_snapshot = 120000;
  config.max_costmap_cells = 120000;
  config.max_navigation_memory_mb = 1200.0;
  config.warn_after_consecutive_overruns = 2;
  config.stop_after_consecutive_overruns = 4;
  return config;
}
}  // namespace

TEST(RuntimeBudgetMonitorTest, AcceptsSampleWithinBudget)
{
  RuntimeBudgetMonitor monitor(testConfig());
  RuntimeBudgetSample sample;
  sample.local_planning_time_ms = 20.0;
  sample.obstacle_update_time_ms = 15.0;
  sample.local_cycle_time_ms = 40.0;
  sample.candidate_trajectories = 30;
  sample.dynamic_obstacles = 3;
  sample.snapshot_points = 10000;
  sample.costmap_cells = 8000;
  sample.process_rss_mb = 400.0;

  const auto status = monitor.update(sample);

  EXPECT_EQ(status.level, BudgetLevel::Ok);
  EXPECT_FALSE(status.budget_exceeded);
  EXPECT_EQ(status.consecutive_overruns, 0u);
}

TEST(RuntimeBudgetMonitorTest, ReportsSoftOverrunBeforePersistentThreshold)
{
  RuntimeBudgetMonitor monitor(testConfig());
  RuntimeBudgetSample sample;
  sample.local_cycle_time_ms = 90.0;

  const auto status = monitor.update(sample);

  EXPECT_EQ(status.level, BudgetLevel::SoftOverrun);
  EXPECT_TRUE(status.budget_exceeded);
  EXPECT_EQ(status.consecutive_overruns, 1u);
}

TEST(RuntimeBudgetMonitorTest, PersistentOverrunRequestsDegradation)
{
  RuntimeBudgetMonitor monitor(testConfig());
  RuntimeBudgetSample sample;
  sample.local_cycle_time_ms = 90.0;

  monitor.update(sample);
  const auto status = monitor.update(sample);

  EXPECT_EQ(status.level, BudgetLevel::Degrade);
  EXPECT_TRUE(status.budget_exceeded);
  EXPECT_EQ(status.consecutive_overruns, 2u);
}

TEST(RuntimeBudgetMonitorTest, RepeatedOverrunRequestsSafeStop)
{
  RuntimeBudgetMonitor monitor(testConfig());
  RuntimeBudgetSample sample;
  sample.local_cycle_time_ms = 90.0;

  monitor.update(sample);
  monitor.update(sample);
  monitor.update(sample);
  const auto status = monitor.update(sample);

  EXPECT_EQ(status.level, BudgetLevel::SafetyStop);
  EXPECT_TRUE(status.budget_exceeded);
  EXPECT_EQ(status.consecutive_overruns, 4u);
}

TEST(RuntimeBudgetMonitorTest, NormalSampleResetsOverrunStreak)
{
  RuntimeBudgetMonitor monitor(testConfig());
  RuntimeBudgetSample over_budget;
  over_budget.local_cycle_time_ms = 90.0;
  RuntimeBudgetSample normal;
  normal.local_cycle_time_ms = 20.0;

  monitor.update(over_budget);
  const auto status = monitor.update(normal);

  EXPECT_EQ(status.level, BudgetLevel::Ok);
  EXPECT_FALSE(status.budget_exceeded);
  EXPECT_EQ(status.consecutive_overruns, 0u);
}
```

- [ ] **Step 2: Run test expecting missing header failure**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args -R test_runtime_budget_monitor --output-on-failure
```

Expected: FAIL with a compile error mentioning `p2p_move_base/runtime_budget_monitor.h` is missing.

## Task 5: Implement RuntimeBudgetMonitor

**Files:**
- Create: `src/dddmr_p2p_move_base/include/p2p_move_base/runtime_budget_monitor.h`
- Create: `src/dddmr_p2p_move_base/src/runtime_budget_monitor.cpp`

- [ ] **Step 1: Create the header**

Create `include/p2p_move_base/runtime_budget_monitor.h`:

```cpp
#ifndef P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_
#define P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_

#include <cstddef>
#include <string>

namespace p2p_move_base
{

enum class BudgetLevel
{
  Ok,
  SoftOverrun,
  Degrade,
  SafetyStop
};

struct RuntimeBudgetSample
{
  double obstacle_update_time_ms = 0.0;
  double local_planning_time_ms = 0.0;
  double local_cycle_time_ms = 0.0;
  std::size_t snapshot_points = 0;
  std::size_t costmap_cells = 0;
  std::size_t candidate_trajectories = 0;
  std::size_t dynamic_obstacles = 0;
  double process_rss_mb = 0.0;
};

struct RuntimeBudgetStatus
{
  BudgetLevel level = BudgetLevel::Ok;
  bool budget_exceeded = false;
  std::size_t consecutive_overruns = 0;
  std::string reason;
};

class RuntimeBudgetMonitor
{
public:
  struct Config
  {
    double max_local_planning_time_ms = 50.0;
    double max_obstacle_update_time_ms = 50.0;
    double max_end_to_end_local_cycle_ms = 80.0;
    std::size_t max_candidate_trajectories = 80;
    std::size_t max_dynamic_obstacles = 48;
    std::size_t max_points_per_snapshot = 120000;
    std::size_t max_costmap_cells = 120000;
    double max_navigation_memory_mb = 1200.0;
    std::size_t warn_after_consecutive_overruns = 2;
    std::size_t stop_after_consecutive_overruns = 10;
  };

  explicit RuntimeBudgetMonitor(const Config & config = Config());

  void setConfig(const Config & config);
  const Config & config() const;
  void reset();

  RuntimeBudgetStatus update(const RuntimeBudgetSample & sample);
  RuntimeBudgetStatus lastStatus() const;

private:
  Config config_;
  RuntimeBudgetStatus last_status_;
  std::size_t consecutive_overruns_;

  bool exceedsBudget(const RuntimeBudgetSample & sample, std::string * reason) const;
};

const char * toString(BudgetLevel level);

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_
```

- [ ] **Step 2: Create the implementation**

Create `src/runtime_budget_monitor.cpp`:

```cpp
#include "p2p_move_base/runtime_budget_monitor.h"

#include <cmath>
#include <sstream>

namespace p2p_move_base
{

RuntimeBudgetMonitor::RuntimeBudgetMonitor(const Config & config)
: config_(config), consecutive_overruns_(0)
{
}

void RuntimeBudgetMonitor::setConfig(const Config & config)
{
  config_ = config;
}

const RuntimeBudgetMonitor::Config & RuntimeBudgetMonitor::config() const
{
  return config_;
}

void RuntimeBudgetMonitor::reset()
{
  consecutive_overruns_ = 0;
  last_status_ = RuntimeBudgetStatus{};
}

bool RuntimeBudgetMonitor::exceedsBudget(
  const RuntimeBudgetSample & sample,
  std::string * reason) const
{
  std::ostringstream out;

  if(std::isfinite(sample.local_planning_time_ms) &&
     sample.local_planning_time_ms > config_.max_local_planning_time_ms){
    out << "local_planning_time_ms=" << sample.local_planning_time_ms;
  }
  else if(std::isfinite(sample.obstacle_update_time_ms) &&
          sample.obstacle_update_time_ms > config_.max_obstacle_update_time_ms){
    out << "obstacle_update_time_ms=" << sample.obstacle_update_time_ms;
  }
  else if(std::isfinite(sample.local_cycle_time_ms) &&
          sample.local_cycle_time_ms > config_.max_end_to_end_local_cycle_ms){
    out << "local_cycle_time_ms=" << sample.local_cycle_time_ms;
  }
  else if(sample.candidate_trajectories > config_.max_candidate_trajectories){
    out << "candidate_trajectories=" << sample.candidate_trajectories;
  }
  else if(sample.dynamic_obstacles > config_.max_dynamic_obstacles){
    out << "dynamic_obstacles=" << sample.dynamic_obstacles;
  }
  else if(sample.snapshot_points > config_.max_points_per_snapshot){
    out << "snapshot_points=" << sample.snapshot_points;
  }
  else if(sample.costmap_cells > config_.max_costmap_cells){
    out << "costmap_cells=" << sample.costmap_cells;
  }
  else if(std::isfinite(sample.process_rss_mb) &&
          sample.process_rss_mb > config_.max_navigation_memory_mb){
    out << "process_rss_mb=" << sample.process_rss_mb;
  }
  else{
    return false;
  }

  if(reason != nullptr){
    *reason = out.str();
  }
  return true;
}

RuntimeBudgetStatus RuntimeBudgetMonitor::update(const RuntimeBudgetSample & sample)
{
  std::string reason;
  const bool exceeded = exceedsBudget(sample, &reason);

  if(!exceeded){
    consecutive_overruns_ = 0;
    last_status_ = RuntimeBudgetStatus{};
    return last_status_;
  }

  ++consecutive_overruns_;

  last_status_.budget_exceeded = true;
  last_status_.consecutive_overruns = consecutive_overruns_;
  last_status_.reason = reason;

  if(consecutive_overruns_ >= config_.stop_after_consecutive_overruns){
    last_status_.level = BudgetLevel::SafetyStop;
  }
  else if(consecutive_overruns_ >= config_.warn_after_consecutive_overruns){
    last_status_.level = BudgetLevel::Degrade;
  }
  else{
    last_status_.level = BudgetLevel::SoftOverrun;
  }

  return last_status_;
}

RuntimeBudgetStatus RuntimeBudgetMonitor::lastStatus() const
{
  return last_status_;
}

const char * toString(BudgetLevel level)
{
  switch(level){
    case BudgetLevel::Ok: return "ok";
    case BudgetLevel::SoftOverrun: return "soft_overrun";
    case BudgetLevel::Degrade: return "degrade";
    case BudgetLevel::SafetyStop: return "safety_stop";
  }
  return "unknown";
}

}  // namespace p2p_move_base
```

- [ ] **Step 3: Run tests**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args -R test_runtime_budget_monitor --output-on-failure
```

Expected: PASS for `test_runtime_budget_monitor`.

- [ ] **Step 4: Run all p2p_move_base tests**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args --output-on-failure
```

Expected: PASS for both `test_navigation_fault_policy` and `test_runtime_budget_monitor`.

- [ ] **Step 5: Commit**

Run:

```bash
git add src/dddmr_p2p_move_base/include/p2p_move_base/runtime_budget_monitor.h \
  src/dddmr_p2p_move_base/src/runtime_budget_monitor.cpp \
  src/dddmr_p2p_move_base/test/test_runtime_budget_monitor.cpp
git commit -m "feat: add runtime budget monitor"
```

## Task 6: Wire Policy And Budget Monitor Into P2PMoveBase

**Files:**
- Modify: `src/dddmr_p2p_move_base/include/p2p_move_base/p2p_move_base.h`
- Modify: `src/dddmr_p2p_move_base/src/p2p_move_base.cpp`

- [ ] **Step 1: Include new headers**

Add these includes to `include/p2p_move_base/p2p_move_base.h`:

```cpp
#include "p2p_move_base/navigation_fault_policy.h"
#include "p2p_move_base/runtime_budget_monitor.h"
```

- [ ] **Step 2: Add members to P2PMoveBase**

Add these private members near `recovery_action_name_`:

```cpp
    p2p_move_base::NavigationFaultPolicy navigation_fault_policy_;
    p2p_move_base::RuntimeBudgetMonitor runtime_budget_monitor_;
    double min_replan_interval_sec_;
    bool runtime_budget_safety_stop_enabled_;
```

- [ ] **Step 3: Declare and read parameters in constructor**

In `P2PMoveBase::P2PMoveBase`, after existing parameter reads, add:

```cpp
  this->declare_parameter("collision_confirm_cycles", rclcpp::ParameterValue(5));
  this->declare_parameter("blocked_confirm_time", rclcpp::ParameterValue(1.0));
  this->declare_parameter("min_replan_interval", rclcpp::ParameterValue(3.0));
  this->declare_parameter("route_deviation_confirm_cycles", rclcpp::ParameterValue(10));
  this->declare_parameter("route_deviation_confirm_time", rclcpp::ParameterValue(1.0));
  this->declare_parameter("runtime_budget_safety_stop_enabled", rclcpp::ParameterValue(false));
  this->declare_parameter("max_local_planning_time_ms", rclcpp::ParameterValue(50.0));
  this->declare_parameter("max_end_to_end_local_cycle_ms", rclcpp::ParameterValue(80.0));
  this->declare_parameter("budget_warn_after_consecutive_overruns", rclcpp::ParameterValue(2));
  this->declare_parameter("budget_stop_after_consecutive_overruns", rclcpp::ParameterValue(10));

  NavigationFaultPolicy::Config fault_config;
  fault_config.collision_confirm_cycles =
    static_cast<std::size_t>(std::max(1, this->get_parameter("collision_confirm_cycles").as_int()));
  fault_config.blocked_confirm_time_sec = this->get_parameter("blocked_confirm_time").as_double();
  fault_config.min_replan_interval_sec = this->get_parameter("min_replan_interval").as_double();
  fault_config.route_deviation_confirm_cycles =
    static_cast<std::size_t>(std::max(1, this->get_parameter("route_deviation_confirm_cycles").as_int()));
  fault_config.route_deviation_confirm_time_sec =
    this->get_parameter("route_deviation_confirm_time").as_double();
  fault_config.control_failure_patience_sec = FSM_->controller_patience_;
  navigation_fault_policy_.setConfig(fault_config);
  min_replan_interval_sec_ = fault_config.min_replan_interval_sec;

  RuntimeBudgetMonitor::Config budget_config;
  budget_config.max_local_planning_time_ms =
    this->get_parameter("max_local_planning_time_ms").as_double();
  budget_config.max_end_to_end_local_cycle_ms =
    this->get_parameter("max_end_to_end_local_cycle_ms").as_double();
  budget_config.warn_after_consecutive_overruns =
    static_cast<std::size_t>(std::max(1, this->get_parameter("budget_warn_after_consecutive_overruns").as_int()));
  budget_config.stop_after_consecutive_overruns =
    static_cast<std::size_t>(std::max(1, this->get_parameter("budget_stop_after_consecutive_overruns").as_int()));
  runtime_budget_monitor_.setConfig(budget_config);
  runtime_budget_safety_stop_enabled_ =
    this->get_parameter("runtime_budget_safety_stop_enabled").as_bool();
```

If the constructor initializes `FSM_` after parameter declarations, move only `fault_config.control_failure_patience_sec = FSM_->controller_patience_;` after `FSM_` is created. Do not dereference `FSM_` before initialization.

- [ ] **Step 4: Replace direct route refresh for tracking faults**

In `executeCycle`, replace the current `PRUNE_PLAN_FAIL`, `ALL_TRAJECTORIES_FAIL`, `PATH_BLOCKED_REPLANNING`, and `PATH_BLOCKED_WAIT` branches in `handle_tracking_control_fault` with:

```cpp
      const double now_sec = clock_->now().seconds();
      const double last_replan_sec = FSM_->last_valid_plan_.seconds();
      const double controller_failure_elapsed =
        (clock_->now() - FSM_->last_valid_control_).seconds();
      const auto decision = navigation_fault_policy_.update(
        planner_state,
        allow_blocked_wait,
        now_sec,
        last_replan_sec,
        controller_failure_elapsed);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "%s fault policy: action=%s, reason=%s, cycles=%zu, duration=%.2f, message=%s",
        phase_label.c_str(),
        p2p_move_base::toString(decision.action),
        p2p_move_base::toString(decision.reason),
        decision.consecutive_cycles,
        decision.duration_sec,
        decision.message.c_str());

      if(decision.action == FaultAction::TemporaryStop){
        publishZeroVelocity();
        return false;
      }
      if(decision.action == FaultAction::WaitForClearance){
        return wait_for_clearance(decision.message);
      }
      if(decision.action == FaultAction::RouteRefresh){
        return route_refresh(decision.message);
      }
      if(decision.action == FaultAction::RecoveryOrRouteRefresh){
        return start_recovery_or_route_refresh(decision.message);
      }
      if(decision.action == FaultAction::FatalAbort){
        return abort_goal_with_reason(decision.message);
      }
```

Keep the existing `PERCEPTION_MALFUNCTION` and `TF_FAIL` early branches before policy handling.

- [ ] **Step 5: Record cycle budget**

At the beginning of `executeCycle`, add:

```cpp
  const auto cycle_started_at = clock_->now();
```

Before each normal `return false` at the end of a control cycle, record budget with a helper lambda:

```cpp
  const auto record_cycle_budget =
    [&]() {
      RuntimeBudgetSample sample;
      sample.local_cycle_time_ms = (clock_->now() - cycle_started_at).seconds() * 1000.0;
      const auto status = runtime_budget_monitor_.update(sample);
      if(status.budget_exceeded){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "runtime budget: level=%s, consecutive_overruns=%zu, reason=%s",
          p2p_move_base::toString(status.level),
          status.consecutive_overruns,
          status.reason.c_str());
      }
      if(runtime_budget_safety_stop_enabled_ && status.level == BudgetLevel::SafetyStop){
        publishZeroVelocity();
      }
    };
```

Call `record_cycle_budget();` before returns in major branches where `executeCycle` continues. Do not call it after aborting/succeeding a goal.

- [ ] **Step 6: Build**

Run:

```bash
colcon build --packages-select p2p_move_base
```

Expected: Build succeeds.

- [ ] **Step 7: Run tests**

Run:

```bash
colcon test --packages-select p2p_move_base --ctest-args --output-on-failure
```

Expected: Unit tests pass.

- [ ] **Step 8: Commit**

Run:

```bash
git add src/dddmr_p2p_move_base/include/p2p_move_base/p2p_move_base.h \
  src/dddmr_p2p_move_base/src/p2p_move_base.cpp
git commit -m "feat: debounce navigation replans in p2p move base"
```

## Task 7: Add Conservative Phase 1 Parameters

**Files:**
- Modify: `src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml`

- [ ] **Step 1: Add parameters under `p2p_move_base.ros__parameters`**

Add these parameters after `blocked_wait_patience`:

```yaml
    collision_confirm_cycles: 5                                 # 连续碰撞确认周期，避免单帧点云误触发全局重规划
    blocked_confirm_time: 1.0                                   # 阻塞持续确认时间（s）
    min_replan_interval: 3.0                                    # 两次全局重规划之间的最小间隔（s）
    route_deviation_confirm_cycles: 10                          # 路径偏离连续确认周期
    route_deviation_confirm_time: 1.0                           # 路径偏离持续确认时间（s）
    runtime_budget_safety_stop_enabled: false                   # 初期只告警不因预算超限停车
    max_local_planning_time_ms: 50.0                            # Orin NX 局部规划预算（ms）
    max_end_to_end_local_cycle_ms: 80.0                         # Orin NX 单周期端到端预算（ms）
    budget_warn_after_consecutive_overruns: 2                   # 连续超预算告警阈值
    budget_stop_after_consecutive_overruns: 10                  # 连续超预算安全停车阈值
```

Change:

```yaml
    blocked_wait_patience: 0.5
```

to:

```yaml
    blocked_wait_patience: 2.0
```

- [ ] **Step 2: Validate YAML syntax**

Run:

```bash
python3 -c "import yaml; yaml.safe_load(open('/home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman/src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml')); print('yaml ok')"
```

Expected:

```text
yaml ok
```

- [ ] **Step 3: Build**

Run:

```bash
colcon build --packages-select p2p_move_base
```

Expected: Build succeeds.

- [ ] **Step 4: Commit**

Run:

```bash
git add src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml
git commit -m "config: add obstacle debounce and budget parameters"
```

## Task 8: Manual ROS Verification

**Files:**
- No code changes.

- [ ] **Step 1: Build the workspace package**

Run:

```bash
colcon build --packages-select p2p_move_base
```

Expected: Build succeeds.

- [ ] **Step 2: Source install setup**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/navigation_ws/install/setup.bash
```

Expected: No output or no error.

- [ ] **Step 3: Launch localization navigation without RViz for log inspection**

Run:

```bash
ros2 launch p2p_move_base p2p_move_base_localization.launch.py use_sim:=false
```

Expected:

```text
p2p_move_base_node starts
global_planner_node starts
No immediate fatal error
```

- [ ] **Step 4: Inspect logs for policy output when local planner faults**

When a local planner fault occurs, expected log form:

```text
route tracking fault policy: action=wait_for_clearance, reason=local_obstacle_detected, cycles=1, duration=0.00, message=path_blocked_unconfirmed: wait before global replan
```

On a confirmed blockage after debounce:

```text
route tracking fault policy: action=route_refresh, reason=static_blockage_confirmed, cycles=5, duration=1.00, message=route_blocked: static blockage confirmed
```

- [ ] **Step 5: Confirm global replan is not triggered by a single collision cycle**

Observe that a single `critic_reject_collision` does not immediately produce:

```text
navigation phase: route_tracking -> route_request (route_blocked: route blocked, refresh route)
```

Expected: First collision enters waiting/temporary stop. Global replan only after confirm cycles and confirm time.

- [ ] **Step 6: Commit verification note if docs are updated**

If you add a manual test note, commit it separately:

```bash
git add docs/superpowers/plans/2026-06-25-obstacle-avoidance-phase-1-diagnostics-budget-plan.md
git commit -m "docs: record phase 1 manual verification"
```

## Self-Review Checklist

- [ ] Spec coverage: This plan covers Phase 1 requirements from the design document: replan debounce, structured reasons, budget monitoring, Orin NX observability, and safe failure behavior.
- [ ] Explicit gap: This plan intentionally does not implement `LocalObstacleLayer`, State Lattice, dynamic obstacle tracking, or two-stage planner backend selection.
- [ ] Placeholder scan: Run the repository placeholder scanner against this plan; there should be no matches.
- [ ] Type consistency: `FaultAction`, `NavigationReason`, `FaultDecision`, `RuntimeBudgetMonitor`, and `RuntimeBudgetSample` names match between tests, headers, and implementation snippets.
- [ ] Test coverage: Policy tests cover single collision debounce, confirmed replan, throttle, route deviation debounce, control failure escalation, and reset.
- [ ] Resource coverage: Budget tests cover OK, soft overrun, degradation, safety stop, and reset.
