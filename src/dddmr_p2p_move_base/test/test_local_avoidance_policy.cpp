#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "p2p_move_base/local_avoidance_policy.h"

using p2p_move_base::AvoidanceDirection;
using p2p_move_base::LocalAvoidancePolicy;
using p2p_move_base::LocalAvoidanceState;
using p2p_move_base::LocalObstacleSnapshot;
using p2p_move_base::LocalPoint;

namespace
{
LocalAvoidancePolicy::Config testConfig()
{
  LocalAvoidancePolicy::Config config;
  config.enabled = true;
  config.forward_check_distance = 4.0;
  config.corridor_half_width = 0.35;
  config.slowdown_distance = 3.0;
  config.stop_distance = 0.75;
  config.min_speed_scale = 0.25;
  config.avoidance_start_distance = 2.0;
  config.side_probe_width = 1.2;
  config.side_switch_margin = 0.5;
  config.direction_hold_time_sec = 1.0;
  config.max_angular_bias = 0.20;
  config.avoidance_speed_scale = 0.45;
  config.takeover_enabled = true;
  config.takeover_timeout_sec = 4.0;
  config.takeover_linear_speed = 0.12;
  return config;
}

std::shared_ptr<const LocalObstacleSnapshot> snapshot(
  const std::vector<LocalPoint> & points,
  bool stale = false,
  bool over_budget = false)
{
  auto snapshot = std::make_shared<LocalObstacleSnapshot>();
  snapshot->frame_id = "base_link";
  snapshot->obstacle_points = points;
  snapshot->filtered_points = points.size();
  snapshot->perception_stale = stale;
  snapshot->over_budget = over_budget;
  return snapshot;
}
}  // namespace

TEST(LocalAvoidancePolicyTest, PassesThroughWhenDisabledOrSnapshotInvalid)
{
  LocalAvoidancePolicy policy(testConfig());
  auto command = p2p_move_base::LocalAvoidanceCommand();
  command.linear_x = 0.4;
  command.angular_z = 0.1;

  LocalAvoidancePolicy::Config config = testConfig();
  config.enabled = false;
  policy.setConfig(config);
  auto decision = policy.update(snapshot({{1.0, 0.0, 0.0}}), command, 10.0);
  EXPECT_EQ(decision.state, LocalAvoidanceState::Clear);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.4);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.1);

  config.enabled = true;
  policy.setConfig(config);
  decision = policy.update(snapshot({{1.0, 0.0, 0.0}}, true), command, 10.1);
  EXPECT_EQ(decision.state, LocalAvoidanceState::Clear);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.4);
}

TEST(LocalAvoidancePolicyTest, SlowsDownForForwardObstacle)
{
  LocalAvoidancePolicy policy(testConfig());
  p2p_move_base::LocalAvoidanceCommand command;
  command.linear_x = 0.6;
  command.angular_z = 0.0;

  const auto decision = policy.update(snapshot({{2.5, 0.0, 0.0}}), command, 10.0);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Slowdown);
  EXPECT_LT(decision.speed_scale, 1.0);
  EXPECT_GE(decision.speed_scale, testConfig().min_speed_scale);
  EXPECT_LT(decision.command.linear_x, command.linear_x);
  EXPECT_EQ(decision.direction, AvoidanceDirection::None);
}

TEST(LocalAvoidancePolicyTest, StopsForObstacleInsideStopDistance)
{
  LocalAvoidancePolicy policy(testConfig());
  p2p_move_base::LocalAvoidanceCommand command;
  command.linear_x = 0.5;
  command.angular_z = 0.1;

  const auto decision = policy.update(snapshot({{0.5, 0.0, 0.0}}), command, 10.0);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Stop);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}

TEST(LocalAvoidancePolicyTest, SelectsClearerSideForLocalAvoidance)
{
  LocalAvoidancePolicy policy(testConfig());
  p2p_move_base::LocalAvoidanceCommand command;
  command.linear_x = 0.5;
  command.angular_z = 0.0;

  const auto decision = policy.update(
    snapshot({
      {1.2, 0.0, 0.0},
      {1.0, -0.8, 0.0},
      {1.5, -0.9, 0.0},
    }),
    command,
    10.0);

  EXPECT_EQ(decision.state, LocalAvoidanceState::LocalAvoidance);
  EXPECT_EQ(decision.direction, AvoidanceDirection::Left);
  EXPECT_GT(decision.command.angular_z, 0.0);
  EXPECT_LT(decision.command.linear_x, command.linear_x);
}

TEST(LocalAvoidancePolicyTest, HoldsAvoidanceDirectionToPreventLeftRightOscillation)
{
  LocalAvoidancePolicy policy(testConfig());
  p2p_move_base::LocalAvoidanceCommand command;
  command.linear_x = 0.5;
  command.angular_z = 0.0;

  auto decision = policy.update(
    snapshot({
      {1.2, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    command,
    10.0);
  ASSERT_EQ(decision.direction, AvoidanceDirection::Left);

  decision = policy.update(
    snapshot({
      {1.2, 0.0, 0.0},
      {1.0, 0.8, 0.0},
    }),
    command,
    10.5);
  EXPECT_EQ(decision.direction, AvoidanceDirection::Left);
  EXPECT_GT(decision.command.angular_z, 0.0);

  decision = policy.update(
    snapshot({
      {1.2, 0.0, 0.0},
      {1.0, 0.8, 0.0},
    }),
    command,
    11.2);
  EXPECT_EQ(decision.direction, AvoidanceDirection::Right);
  EXPECT_LT(decision.command.angular_z, 0.0);
}

TEST(LocalAvoidancePolicyTest, StartsTakeoverWithUsefulSpeedWhenPlannerReportsBlocked)
{
  LocalAvoidancePolicy policy(testConfig());

  auto decision = policy.update(
    snapshot({
      {1.8, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.0,
    true);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Takeover);
  EXPECT_EQ(decision.direction, AvoidanceDirection::Left);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, testConfig().takeover_linear_speed);
  EXPECT_GT(decision.command.angular_z, 0.0);
}

TEST(LocalAvoidancePolicyTest, StopsInsteadOfTakingOverInsideStopDistance)
{
  LocalAvoidancePolicy policy(testConfig());

  const auto decision = policy.update(
    snapshot({{0.5, 0.0, 0.0}}),
    p2p_move_base::LocalAvoidanceCommand(),
    10.0,
    true);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Stop);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}

TEST(LocalAvoidancePolicyTest, RefusesTakeoverAfterTimeout)
{
  LocalAvoidancePolicy policy(testConfig());

  auto decision = policy.update(
    snapshot({
      {1.8, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.0,
    true);
  ASSERT_EQ(decision.state, LocalAvoidanceState::Takeover);

  decision = policy.update(
    snapshot({
      {1.7, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    14.2,
    true);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Stop);
  EXPECT_EQ(decision.reason, "local_takeover_timeout");
}

TEST(LocalAvoidancePolicyTest, ContinuesTakeoverWhilePlannerStillBlockedAfterForwardCorridorClears)
{
  LocalAvoidancePolicy policy(testConfig());

  auto decision = policy.update(
    snapshot({
      {1.8, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.0,
    true);
  ASSERT_EQ(decision.state, LocalAvoidanceState::Takeover);
  ASSERT_EQ(decision.direction, AvoidanceDirection::Left);

  decision = policy.update(
    snapshot({
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.5,
    true);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Takeover);
  EXPECT_EQ(decision.direction, AvoidanceDirection::Left);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, testConfig().takeover_linear_speed);
  EXPECT_GT(decision.command.angular_z, 0.0);
  EXPECT_EQ(decision.reason, "planner_blocked_takeover_continues_after_forward_clear");
}

TEST(LocalAvoidancePolicyTest, ClearsAfterForwardCorridorClearsWhenPlannerRecovered)
{
  LocalAvoidancePolicy policy(testConfig());

  auto decision = policy.update(
    snapshot({
      {1.8, 0.0, 0.0},
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.0,
    true);
  ASSERT_EQ(decision.state, LocalAvoidanceState::Takeover);

  decision = policy.update(
    snapshot({
      {1.0, -0.8, 0.0},
    }),
    p2p_move_base::LocalAvoidanceCommand(),
    10.5,
    false);

  EXPECT_EQ(decision.state, LocalAvoidanceState::Clear);
  EXPECT_EQ(decision.direction, AvoidanceDirection::None);
  EXPECT_EQ(decision.reason, "forward_corridor_clear");
}
