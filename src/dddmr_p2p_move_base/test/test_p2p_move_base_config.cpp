#include <fstream>
#include <cmath>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void expectInOrder(const std::string & config, const std::vector<std::string> & markers)
{
  std::string::size_type previous = 0;
  for(const auto & marker : markers){
    const auto pos = config.find(marker, previous);
    EXPECT_NE(pos, std::string::npos) << "missing marker: " << marker;
    if(pos == std::string::npos){
      return;
    }
    EXPECT_GE(pos, previous) << "marker out of order: " << marker;
    previous = pos;
  }
}

std::string findBlock(const std::string & config, const std::string & start_marker)
{
  const auto start = config.find(start_marker);
  if(start == std::string::npos){
    return {};
  }
  const auto end = config.find("\n\n", start + start_marker.size());
  return config.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

double extractDouble(const std::string & block, const std::string & key)
{
  const std::regex pattern(key + R"(:\s*(-?[0-9]+(?:\.[0-9]+)?))");
  std::smatch match;
  EXPECT_TRUE(std::regex_search(block, match, pattern)) << "missing numeric key: " << key;
  if(match.empty()){
    return 0.0;
  }
  return std::stod(match[1].str());
}
}  // namespace

TEST(P2PMoveBaseConfigTest, LocalizationConfigDoesNotExposeLocalAvoidanceControl)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");

  ASSERT_FALSE(config.empty());
  EXPECT_EQ(config.find("\n    local_avoidance:"), std::string::npos);
  EXPECT_NE(config.find("\n    local_obstacle_layer:"), std::string::npos);
  EXPECT_NE(config.find("collision_confirm_cycles"), std::string::npos);
  EXPECT_NE(config.find("blocked_confirm_time"), std::string::npos);
}

TEST(P2PMoveBaseConfigTest, LocalizationConfigContainsMppiBackendParameters)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");

  ASSERT_FALSE(config.empty());
  EXPECT_NE(config.find("controller_backend: \"mppi\""), std::string::npos);
  EXPECT_NE(config.find("\n    mppi:"), std::string::npos);
  EXPECT_EQ(config.find("mppi:\n      critic_trajectory_generator_name"), std::string::npos);
  EXPECT_NE(config.find("critics: [\"ConstraintCritic\", \"ObstaclesCritic\", \"GoalCritic\", \"GoalAngleCritic\", \"PathAlignCritic\", \"PathFollowCritic\", \"PathAngleCritic\", \"PreferForwardCritic\"]"), std::string::npos);
  EXPECT_NE(config.find("batch_size: 96"), std::string::npos);
  EXPECT_NE(config.find("gamma: 0.015"), std::string::npos);
  EXPECT_NE(config.find("vx_std: 0.2"), std::string::npos);
  EXPECT_NE(config.find("wz_std: 0.08"), std::string::npos);
  EXPECT_NE(config.find("regenerate_noises: false"), std::string::npos);
  EXPECT_NE(config.find("structured_candidate_count: 8"), std::string::npos);
  EXPECT_NE(config.find("inflation_radius: 2.2"), std::string::npos);
  EXPECT_NE(config.find("use_perception_blocked_preemption: false"), std::string::npos);
  EXPECT_EQ(config.find("trajectory_generator: mppi"), std::string::npos);
}

TEST(P2PMoveBaseConfigTest, MppiTemperatureConcentratesAroundBestAvoidanceSamples)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");
  const std::string mppi = findBlock(config, "\n    mppi:");

  ASSERT_FALSE(mppi.empty());
  const double temperature = extractDouble(mppi, "temperature");

  EXPECT_LE(temperature, 0.12)
    << "MPPI temperature is too high and averages away left/right avoidance commands";
}

TEST(P2PMoveBaseConfigTest, MppiHorizonSupportsAckermannObstacleBypass)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");
  const std::string mppi = findBlock(config, "\n    mppi:");

  ASSERT_FALSE(mppi.empty());
  const double horizon = extractDouble(mppi, "prediction_horizon_sec");
  const double vx_max = extractDouble(mppi, "vx_max");
  const double wz_max = extractDouble(mppi, "wz_max");
  const double wheelbase = extractDouble(mppi, "wheelbase");
  const double max_steer = extractDouble(mppi, "max_steer");
  const double steering_wz = vx_max * std::tan(max_steer) / wheelbase;
  const double max_wz = std::min(wz_max, steering_wz);
  ASSERT_GT(max_wz, 1e-6);

  const double turning_radius = vx_max / max_wz;
  const double max_heading_change = max_wz * horizon;
  const double max_lateral_reach =
    turning_radius * (1.0 - std::cos(std::min(max_heading_change, 1.57079632679)));

  EXPECT_GE(max_lateral_reach, 0.8)
    << "MPPI horizon is too short for Ackermann obstacle bypass";
}

TEST(P2PMoveBaseConfigTest, MppiHasEnoughInitialAngularAuthorityForObstacleAvoidance)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");
  const std::string mppi = findBlock(config, "\n    mppi:");

  ASSERT_FALSE(mppi.empty());
  const double model_dt = extractDouble(mppi, "model_dt");
  const double vx_max = extractDouble(mppi, "vx_max");
  const double wz_max = extractDouble(mppi, "wz_max");
  const double az_max = extractDouble(mppi, "az_max");
  const double wheelbase = extractDouble(mppi, "wheelbase");
  const double max_steer = extractDouble(mppi, "max_steer");
  const double steering_wz = vx_max * std::tan(max_steer) / wheelbase;
  const double first_command_wz = std::min({wz_max, steering_wz, az_max * model_dt});

  EXPECT_GE(first_command_wz, 0.2)
    << "MPPI first command cannot turn enough to start obstacle avoidance";
}

TEST(P2PMoveBaseConfigTest, MppiHasEnoughInitialLinearAuthorityForTracking)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");
  const std::string mppi = findBlock(config, "\n    mppi:");

  ASSERT_FALSE(mppi.empty());
  const double model_dt = extractDouble(mppi, "model_dt");
  const double vx_max = extractDouble(mppi, "vx_max");
  const double ax_max = extractDouble(mppi, "ax_max");
  const double first_command_vx = std::min(vx_max, ax_max * model_dt);

  EXPECT_GE(first_command_vx, 0.15)
    << "MPPI first command is effectively pinned to vx_min from rest";
}

TEST(P2PMoveBaseConfigTest, MppiObstacleScoringStaysWithinRuntimeBudget)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");
  const std::string mppi = findBlock(config, "\n    mppi:");
  const std::string obstacles = findBlock(config, "\n      ObstaclesCritic:");

  ASSERT_FALSE(mppi.empty());
  ASSERT_FALSE(obstacles.empty());
  const double horizon = extractDouble(mppi, "prediction_horizon_sec");
  const double model_dt = extractDouble(mppi, "model_dt");
  const double batch_size = extractDouble(mppi, "batch_size");
  const double trajectory_point_step = extractDouble(obstacles, "trajectory_point_step");
  ASSERT_GT(model_dt, 1e-6);
  ASSERT_GE(trajectory_point_step, 1.0);

  const double time_steps = std::ceil(horizon / model_dt);
  const double obstacle_queries_per_cycle =
    batch_size * std::ceil(time_steps / trajectory_point_step);

  EXPECT_LE(obstacle_queries_per_cycle, 2000.0)
    << "MPPI obstacle KDTree queries per cycle exceed the local runtime budget";
}

TEST(P2PMoveBaseConfigTest, PlannerParametersAreGroupedByPlannerAndBackend)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");

  ASSERT_FALSE(config.empty());
  expectInOrder(config, {
    "# 全局规划：路径请求与后端选择",
    "# 全局规划器公共参数",
    "# 全局规划器后端选择",
    "# 全局规划器后端参数: graph_a_star",
    "# 全局规划器后端参数: hybrid_a_star",
    "# 全局规划感知输入"
  });
  expectInOrder(config, {
    "# 局部规划：控制后端选择与公共约束",
    "# 局部规划器公共参数",
    "# 局部规划器后端选择",
    "# 局部规划器公共路径/目标参数",
    "# 局部控制后端参数: regulated_pure_pursuit",
    "# 局部控制后端参数: mppi",
    "# 局部控制后端参数: legacy_rollout trajectory_generators",
    "# 局部规划感知输入",
    "# 局部轨迹评分 critics"
  });
}
