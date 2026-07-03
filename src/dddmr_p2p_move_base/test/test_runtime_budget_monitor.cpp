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
