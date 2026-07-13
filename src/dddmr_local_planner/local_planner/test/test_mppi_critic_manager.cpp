#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <numeric>
#include <random>
#include <unordered_set>
#include <vector>

#include "local_planner/mppi_critic_manager.h"
#include "local_planner/mppi_obstacle_geometry.h"

using local_planner::computeObstacleClearanceForCuboid;
using local_planner::computeAdaptiveObstacleClearanceForCuboid;
using local_planner::computeMppiCuboidCollisionQueryRadius;
using local_planner::collectMppiObstacleCollisionCandidates;
using local_planner::estimateMppiObstacleScoringWork;
using local_planner::evaluateMppiObstacleCollisionForCuboid;
using local_planner::formatMppiObstacleQueryMetrics;
using local_planner::annotateMppiCandidateMetadata;
using local_planner::classifyMppiBypassSide;
using local_planner::hasObstacleCollisionForCuboid;
using local_planner::isMppiTrajectoryCollisionFree;
using local_planner::makeMppiObstacleSnapshot;
using local_planner::MppiBypassSide;
using local_planner::MppiBypassSideClassifier;
using local_planner::MppiCandidateScore;
using local_planner::MppiCriticConfig;
using local_planner::MppiCriticData;
using local_planner::MppiCriticManager;
using local_planner::MppiCriticResult;
using local_planner::MppiPathPoint;
using local_planner::MppiScoredTrajectory;
using local_planner::MppiTrajectoryPoint;

namespace
{
constexpr double kTestPi = 3.14159265358979323846;

pcl::PointCloud<pcl::PointXYZ> makeTestCuboid(
  const double center_x,
  const double center_y,
  const double center_z,
  const double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const auto transform = [&](const double x, const double y, const double z) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(center_x + cos_yaw * x - sin_yaw * y);
      point.y = static_cast<float>(center_y + sin_yaw * x + cos_yaw * y);
      point.z = static_cast<float>(center_z + z);
      return point;
    };
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(transform(-0.38, 0.32, -0.32));  // blb
  cuboid.push_back(transform(-0.38, -0.32, -0.32));  // brb
  cuboid.push_back(transform(-0.38, 0.32, 0.55));  // blt
  cuboid.push_back(transform(0.50, 0.32, -0.32));  // flb
  cuboid.push_back(transform(-0.38, -0.32, 0.55));  // brt
  cuboid.push_back(transform(0.50, -0.32, 0.55));  // frt
  cuboid.push_back(transform(0.50, 0.32, 0.55));  // flt
  cuboid.push_back(transform(0.50, -0.32, -0.32));  // frb
  return cuboid;
}

MppiScoredTrajectory makeTrajectory(
  const double vx,
  const std::vector<MppiTrajectoryPoint> & points)
{
  MppiScoredTrajectory trajectory;
  trajectory.controls.resize(points.size());
  for(auto & control : trajectory.controls){
    control.vx = vx;
    control.wz = 0.0;
  }
  trajectory.points = points;
  for(auto & point : trajectory.points){
    point.collision_evaluated = true;
  }
  return trajectory;
}

MppiCriticData makeData(const std::vector<MppiScoredTrajectory> & trajectories)
{
  MppiCriticData data;
  data.trajectories = trajectories;
  data.path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0},
    MppiPathPoint{3.0, 0.0, 0.0}
  };
  data.local_path_length = 3.0;
  data.vx_min = 0.0;
  data.vx_max = 0.8;
  data.min_turning_radius = 1.9;
  data.model_dt = 0.1;
  return data;
}
}  // namespace

TEST(MppiCriticManagerTest, AnnotatesAlignedScoresWithPhysicalSideAndCollisionMetadata)
{
  MppiCriticData data;
  data.path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}};
  data.trajectories = {
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.5, 0.3, 0.0},
      MppiTrajectoryPoint{1.5, 0.4, 0.0}}),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.5, -0.3, 0.0, true},
      MppiTrajectoryPoint{1.5, -0.4, 0.0}}),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.5, 0.3, 0.0},
      MppiTrajectoryPoint{1.5, 0.4, 0.0}})};
  MppiCriticResult result;
  result.scores = {
    MppiCandidateScore{true, 1.0},
    MppiCandidateScore{true, 2.0},
    MppiCandidateScore{false, -1.0}};

  ASSERT_TRUE(annotateMppiCandidateMetadata(data, &result));
  EXPECT_EQ(result.scores[0].bypass_side, MppiBypassSide::Left);
  EXPECT_TRUE(result.scores[0].collision_free);
  EXPECT_EQ(result.scores[1].bypass_side, MppiBypassSide::Right);
  EXPECT_FALSE(result.scores[1].collision_free);
  EXPECT_EQ(result.scores[2].bypass_side, MppiBypassSide::Neutral);
  EXPECT_FALSE(result.scores[2].collision_free);
  EXPECT_FALSE(result.scores[2].valid);
  EXPECT_DOUBLE_EQ(result.scores[2].cost, -1.0);
}

TEST(MppiCriticManagerTest, MetadataAnnotationRejectsMismatchedResultWithoutOutOfBoundsAccess)
{
  const auto data = makeData({
    makeTrajectory(0.2, {MppiTrajectoryPoint{0.5, 0.3, 0.0}}),
    makeTrajectory(0.2, {MppiTrajectoryPoint{0.5, -0.3, 0.0}})});
  MppiCriticResult result;
  result.scores = {MppiCandidateScore{true, 1.0}};

  EXPECT_FALSE(annotateMppiCandidateMetadata(data, &result));
  ASSERT_EQ(result.scores.size(), 1u);
  EXPECT_FALSE(result.scores.front().valid);
}

TEST(MppiCriticManagerTest, MetadataAnnotationTreatsUnevaluatedPoseAsNotCollisionFree)
{
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0},
    MppiTrajectoryPoint{1.0, 0.4, 0.0},
    MppiTrajectoryPoint{1.5, 0.3, 0.0}});
  trajectory.points[1].collision_evaluated = false;
  auto data = makeData({trajectory});
  MppiCriticResult result;
  result.scores = {MppiCandidateScore{true, 1.0}};

  ASSERT_TRUE(annotateMppiCandidateMetadata(data, &result));
  EXPECT_EQ(result.scores.front().bypass_side, MppiBypassSide::Left);
  EXPECT_FALSE(result.scores.front().collision_free);
}

TEST(MppiCriticManagerTest, CandidateScoreDefaultsToNeutralCollisionFreeMetadata)
{
  const MppiCandidateScore score{true, 1.0};

  EXPECT_EQ(score.bypass_side, MppiBypassSide::Neutral);
  EXPECT_TRUE(score.collision_free);
}

TEST(MppiCriticManagerTest, ClassifiesTrajectoryAboveStraightPathAsLeft)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.2, 0.0},
    MppiTrajectoryPoint{1.5, 0.4, 0.0}
  });

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Left);
}

TEST(MppiCriticManagerTest, ClassifiesTrajectoryBelowStraightPathAsRight)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, -0.2, 0.0},
    MppiTrajectoryPoint{1.5, -0.4, 0.0}
  });

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Right);
}

TEST(MppiCriticManagerTest, ClassifiesTrajectoryWithinDeadbandAsNeutral)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.05, 0.0},
    MppiTrajectoryPoint{1.5, 0.10, 0.0}
  });

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Neutral);
}

TEST(MppiCriticManagerTest, ClassifiesGeometryIndependentlyOfTerminalYawRate)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0},
    MppiTrajectoryPoint{1.5, 0.4, 0.0}
  });
  trajectory.controls.back().wz = -0.8;

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Left);
}

TEST(MppiCriticManagerTest, ClassifierRejectsTrajectoryWithEmptyControls)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0}
  });
  trajectory.controls.clear();

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Neutral);
}

TEST(MppiCriticManagerTest, ClassifierRejectsTrajectoryWithFewerControlsThanPoints)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0},
    MppiTrajectoryPoint{1.5, 0.4, 0.0}
  });
  trajectory.controls.resize(1);

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Neutral);
}

TEST(MppiCriticManagerTest, ClassifierRejectsTrajectoryWithMoreControlsThanPoints)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0}
  });
  trajectory.controls.resize(2);

  EXPECT_EQ(classifyMppiBypassSide(trajectory, path), MppiBypassSide::Neutral);
}

TEST(MppiCriticManagerTest, ClassifierUsesPathDirectionForRotatedAndReversedSegments)
{
  const std::vector<MppiPathPoint> upward_path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{0.0, 2.0, 0.0}
  };
  const std::vector<MppiPathPoint> downward_path = {
    MppiPathPoint{0.0, 2.0, 0.0},
    MppiPathPoint{0.0, 0.0, 0.0}
  };
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{-0.5, 1.0, 0.0}
  });

  EXPECT_EQ(classifyMppiBypassSide(trajectory, upward_path), MppiBypassSide::Left);
  EXPECT_EQ(classifyMppiBypassSide(trajectory, downward_path), MppiBypassSide::Right);
}

TEST(MppiCriticManagerTest, ClassifierUsesStrictSymmetricDeadband)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  const auto positive_boundary = makeTrajectory(0.2, {
    MppiTrajectoryPoint{1.0, 0.10, 0.0}
  });
  const auto negative_boundary = makeTrajectory(0.2, {
    MppiTrajectoryPoint{1.0, -0.10, 0.0}
  });
  const auto just_left = makeTrajectory(0.2, {
    MppiTrajectoryPoint{1.0, 0.1001, 0.0}
  });
  const auto just_right = makeTrajectory(0.2, {
    MppiTrajectoryPoint{1.0, -0.1001, 0.0}
  });

  EXPECT_EQ(classifyMppiBypassSide(positive_boundary, path), MppiBypassSide::Neutral);
  EXPECT_EQ(classifyMppiBypassSide(negative_boundary, path), MppiBypassSide::Neutral);
  EXPECT_EQ(classifyMppiBypassSide(just_left, path), MppiBypassSide::Left);
  EXPECT_EQ(classifyMppiBypassSide(just_right, path), MppiBypassSide::Right);
}

TEST(MppiCriticManagerTest, ReusableClassifierMatchesCompatibilityWrapper)
{
  const std::vector<MppiPathPoint> path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0},
    MppiPathPoint{2.0, 0.0, 0.0}
  };
  const std::vector<MppiScoredTrajectory> trajectories = {
    makeTrajectory(0.2, {MppiTrajectoryPoint{1.0, 0.3, 0.0}}),
    makeTrajectory(0.2, {MppiTrajectoryPoint{1.0, -0.3, 0.0}}),
    makeTrajectory(0.2, {MppiTrajectoryPoint{1.0, 0.05, 0.0}})
  };
  const MppiBypassSideClassifier classifier(path);

  for(const auto & trajectory : trajectories){
    EXPECT_EQ(classifier.classify(trajectory), classifyMppiBypassSide(trajectory, path));
  }
}

TEST(MppiCriticManagerTest, ClassifierRejectsDegenerateAndNonfiniteInputs)
{
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.5, 0.3, 0.0}
  });
  const std::vector<MppiPathPoint> degenerate_path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{0.0, 0.0, 0.0}
  };
  const std::vector<MppiPathPoint> valid_path = {
    MppiPathPoint{0.0, 0.0, 0.0},
    MppiPathPoint{1.0, 0.0, 0.0}
  };
  auto nonfinite_trajectory = trajectory;
  nonfinite_trajectory.points.front().y = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    classifyMppiBypassSide(trajectory, degenerate_path), MppiBypassSide::Neutral);
  EXPECT_EQ(
    classifyMppiBypassSide(MppiScoredTrajectory{}, valid_path), MppiBypassSide::Neutral);
  EXPECT_EQ(
    classifyMppiBypassSide(nonfinite_trajectory, valid_path), MppiBypassSide::Neutral);
}

TEST(MppiCriticManagerTest, ObstaclesCriticAddsCollisionCostWithoutRejectingWholeBatch)
{
  MppiCriticConfig config;
  config.critics = {"ObstaclesCritic"};
  config.obstacles.collision_cost = 1000.0;
  config.obstacles.critical_weight = 10.0;
  config.obstacles.repulsion_weight = 1.0;
  config.obstacles.collision_margin_distance = 0.3;
  config.obstacles.inflation_radius = 1.0;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 1.2},
      MppiTrajectoryPoint{1.0, 0.0, 0.0, false, 1.2}
    }),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 0.5},
      MppiTrajectoryPoint{1.0, 0.0, 0.0, true, 0.0}
    })
  });

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 2u);
  EXPECT_FALSE(result.fail_flag);
  EXPECT_TRUE(result.scores[0].valid);
  EXPECT_TRUE(result.scores[1].valid);
  EXPECT_LT(result.scores[0].cost, result.scores[1].cost);
  EXPECT_GE(result.scores[1].cost, config.obstacles.collision_cost);
}

TEST(MppiCriticManagerTest, ObstaclesCriticFailsWhenEveryTrajectoryCollides)
{
  MppiCriticConfig config;
  config.critics = {"ObstaclesCritic"};

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {MppiTrajectoryPoint{0.0, 0.0, 0.0, true, 0.0}}),
    makeTrajectory(0.2, {MppiTrajectoryPoint{0.0, 1.0, 0.0, true, 0.0}})
  });

  const auto result = manager.score(data);

  EXPECT_TRUE(result.fail_flag);
}

TEST(MppiCriticManagerTest, ObstaclesCriticNormalizesByEvaluatedClearancePoints)
{
  MppiCriticConfig config;
  config.critics = {"ObstaclesCritic"};
  config.obstacles.critical_weight = 0.0;
  config.obstacles.repulsion_weight = 1.0;
  config.obstacles.inflation_radius = 1.5;

  MppiCriticManager manager;
  manager.setConfig(config);

  std::vector<MppiTrajectoryPoint> points(75);
  for(auto & point : points){
    point.obstacle_clearance = std::numeric_limits<double>::infinity();
  }
  for(std::size_t i = 0; i < 20; ++i){
    points[i * 4].obstacle_clearance = 0.5;
  }

  const auto result = manager.score(makeData({makeTrajectory(0.2, points)}));

  ASSERT_EQ(result.scores.size(), 1u);
  EXPECT_NEAR(result.scores.front().cost, 1.0, 1e-9);
}

TEST(MppiCriticManagerTest, PathFollowCriticScoresTerminalDistanceToForwardPathPoint)
{
  MppiCriticConfig config;
  config.critics = {"PathFollowCritic"};
  config.path_follow.cost_weight = 5.0;
  config.path_follow.offset_from_furthest = 1;
  config.path_follow.threshold_to_consider = 0.5;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{2.0, 0.0, 0.0, false, 2.0}
    }),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{0.5, 2.0, 0.0, false, 2.0}
    })
  });

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 2u);
  EXPECT_LT(result.scores[0].cost, result.scores[1].cost);
}

TEST(MppiCriticManagerTest, PathFollowCriticSkipsOccupiedTargetPoint)
{
  MppiCriticConfig config;
  config.critics = {"PathFollowCritic"};
  config.path_follow.cost_weight = 5.0;
  config.path_follow.offset_from_furthest = 1;
  config.path_follow.threshold_to_consider = 0.5;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0}
    })
  });
  data.path[1].obstacle_free = false;

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 1u);
  EXPECT_NEAR(result.scores.front().cost, 10.0, 1e-9);
}

TEST(MppiCriticManagerTest, PathFollowCriticSkipsScoringWhenNoFreeTargetRemains)
{
  MppiCriticConfig config;
  config.critics = {"PathFollowCritic"};
  config.path_follow.cost_weight = 5.0;
  config.path_follow.offset_from_furthest = 1;
  config.path_follow.threshold_to_consider = 0.5;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0}
    })
  });
  for(std::size_t i = 1; i < data.path.size(); ++i){
    data.path[i].obstacle_free = false;
  }

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 1u);
  EXPECT_DOUBLE_EQ(result.scores.front().cost, 0.0);
}

TEST(MppiCriticManagerTest, PathAlignCriticSkipsWhenPathOccupancyIsTooHigh)
{
  MppiCriticConfig config;
  config.critics = {"PathAlignCritic"};
  config.path_align.cost_weight = 10.0;
  config.path_align.threshold_to_consider = 0.5;
  config.path_align.offset_from_furthest = 0;
  config.path_align.trajectory_point_step = 1;
  config.path_align.max_path_occupancy_ratio = 0.1;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{1.0, 0.0, 0.0, false, 2.0}
    }),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 1.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{1.0, 1.0, 0.0, false, 2.0}
    })
  });
  data.path_obstacle_occupancy_ratio = 0.5;
  data.blocked_path_points = 3;

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 2u);
  EXPECT_DOUBLE_EQ(result.scores[0].cost, 0.0);
  EXPECT_DOUBLE_EQ(result.scores[1].cost, 0.0);
}

TEST(MppiCriticManagerTest, PathAlignCriticStaysActiveForFewerThanThreeBlockedPoints)
{
  MppiCriticConfig config;
  config.critics = {"PathAlignCritic"};
  config.path_align.cost_weight = 10.0;
  config.path_align.threshold_to_consider = 0.5;
  config.path_align.offset_from_furthest = 0;
  config.path_align.trajectory_point_step = 1;
  config.path_align.max_path_occupancy_ratio = 0.1;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{1.0, 0.0, 0.0, false, 2.0}
    }),
    makeTrajectory(0.2, {
      MppiTrajectoryPoint{0.0, 1.0, 0.0, false, 2.0},
      MppiTrajectoryPoint{1.0, 1.0, 0.0, false, 2.0}
    })
  });
  data.path_obstacle_occupancy_ratio = 0.5;
  data.blocked_path_points = 2;

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 2u);
  EXPECT_DOUBLE_EQ(result.scores[0].cost, 0.0);
  EXPECT_GT(result.scores[1].cost, 0.0);
}

TEST(MppiCriticManagerTest, PreferForwardCriticPenalizesReverseControls)
{
  MppiCriticConfig config;
  config.critics = {"PreferForwardCritic"};
  config.prefer_forward.cost_weight = 5.0;
  config.prefer_forward.threshold_to_consider = 0.5;

  MppiCriticManager manager;
  manager.setConfig(config);

  auto data = makeData({
    makeTrajectory(0.2, {MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0}}),
    makeTrajectory(-0.2, {MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 2.0}})
  });

  const auto result = manager.score(data);

  ASSERT_EQ(result.scores.size(), 2u);
  EXPECT_LT(result.scores[0].cost, result.scores[1].cost);
}

TEST(MppiCriticManagerTest, FinalTrajectoryValidatorRejectsAnyCollision)
{
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 1.0},
    MppiTrajectoryPoint{1.0, 0.0, 0.0, true, 0.0},
    MppiTrajectoryPoint{2.0, 0.0, 0.0, false, 1.0}
  });

  EXPECT_FALSE(isMppiTrajectoryCollisionFree(trajectory));
}

TEST(MppiCriticManagerTest, FinalTrajectoryValidatorRejectsUnevaluatedIntermediatePose)
{
  auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 1.0},
    MppiTrajectoryPoint{1.0, 0.0, 0.0, false, 1.0},
    MppiTrajectoryPoint{2.0, 0.0, 0.0, false, 1.0}
  });
  trajectory.points[1].collision_evaluated = false;

  EXPECT_FALSE(isMppiTrajectoryCollisionFree(trajectory));
}

TEST(MppiCriticManagerTest, FinalTrajectoryValidatorAcceptsCollisionFreeTrajectory)
{
  const auto trajectory = makeTrajectory(0.2, {
    MppiTrajectoryPoint{0.0, 0.0, 0.0, false, 1.0},
    MppiTrajectoryPoint{1.0, 0.0, 0.0, false, 0.5}
  });

  EXPECT_TRUE(isMppiTrajectoryCollisionFree(trajectory));
}

TEST(MppiCriticManagerTest, FinalTrajectoryValidatorRejectsEmptyTrajectory)
{
  EXPECT_FALSE(isMppiTrajectoryCollisionFree(MppiScoredTrajectory{}));
}

TEST(MppiObstacleGeometryTest, IgnoresStaleObstacleIndices)
{
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, 1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, 1.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, -1.0F, -1.0F});

  pcl::PointCloud<pcl::PointXYZI> obstacles;
  pcl::PointXYZI obstacle;
  obstacle.x = 3.0F;
  obstacle.y = 0.0F;
  obstacle.z = 0.0F;
  obstacle.intensity = 0.0F;
  obstacles.push_back(obstacle);

  const auto result = computeObstacleClearanceForCuboid(cuboid, obstacles, {5, -1, 0});

  EXPECT_FALSE(result.in_collision);
  EXPECT_EQ(result.ignored_indices, 2u);
  EXPECT_GT(result.clearance, 0.0);
}

TEST(MppiObstacleGeometryTest, TightCollisionQueryRadiusBoundsEveryCuboidVertex)
{
  const pcl::PointXYZ center{1.0F, 2.0F, 3.0F};
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(pcl::PointXYZ{2.0F, 2.0F, 3.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, 4.0F, 3.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, 2.0F, 0.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, 2.0F, 7.0F});

  const double radius = computeMppiCuboidCollisionQueryRadius(cuboid, center);

  EXPECT_GT(radius, 4.0);
  EXPECT_LT(radius, 4.0 + 1e-5);
}

TEST(MppiObstacleGeometryTest, CollisionOnlyQueryUsesExactCuboidFootprint)
{
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, 1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, 1.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, -1.0F, -1.0F});
  pcl::PointCloud<pcl::PointXYZI> obstacles;
  pcl::PointXYZI inside;
  inside.x = 0.0F;
  inside.y = 0.0F;
  inside.z = 0.0F;
  obstacles.push_back(inside);
  pcl::PointXYZI outside;
  outside.x = 2.0F;
  outside.y = 0.0F;
  outside.z = 0.0F;
  obstacles.push_back(outside);

  EXPECT_TRUE(hasObstacleCollisionForCuboid(cuboid, obstacles, {0}));
  EXPECT_FALSE(hasObstacleCollisionForCuboid(cuboid, obstacles, {1}));
}

TEST(MppiObstacleGeometryTest, CollisionKeepsZeroClearanceWhenOutsidePointIsVisitedFirst)
{
  const auto cuboid = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  pcl::PointCloud<pcl::PointXYZI> obstacles;
  pcl::PointXYZI outside;
  outside.x = 2.0F;
  outside.y = 0.0F;
  outside.z = 0.0F;
  obstacles.push_back(outside);
  pcl::PointXYZI inside;
  inside.x = 0.0F;
  inside.y = 0.0F;
  inside.z = 0.0F;
  obstacles.push_back(inside);

  const auto result = computeObstacleClearanceForCuboid(cuboid, obstacles, {0, 1});

  EXPECT_TRUE(result.in_collision);
  EXPECT_DOUBLE_EQ(result.clearance, 0.0);
}

TEST(MppiObstacleGeometryTest, DirectScanMatchesIndexedScanAndIgnoresNonFinitePoints)
{
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, 1.0F, -1.0F});
  cuboid.push_back(pcl::PointXYZ{-1.0F, -1.0F, 1.0F});
  cuboid.push_back(pcl::PointXYZ{1.0F, -1.0F, -1.0F});

  pcl::PointCloud<pcl::PointXYZI> obstacles;
  pcl::PointXYZI non_finite;
  non_finite.x = std::numeric_limits<float>::quiet_NaN();
  non_finite.y = 0.0F;
  non_finite.z = 0.0F;
  obstacles.push_back(non_finite);
  pcl::PointXYZI outside;
  outside.x = 2.0F;
  outside.y = 0.0F;
  outside.z = 0.0F;
  obstacles.push_back(outside);

  EXPECT_EQ(
    hasObstacleCollisionForCuboid(cuboid, obstacles),
    hasObstacleCollisionForCuboid(cuboid, obstacles, {0, 1}));

  pcl::PointXYZI inside;
  inside.x = 0.0F;
  inside.y = 0.0F;
  inside.z = 0.0F;
  obstacles.push_back(inside);
  EXPECT_EQ(
    hasObstacleCollisionForCuboid(cuboid, obstacles),
    hasObstacleCollisionForCuboid(cuboid, obstacles, {0, 1, 2}));
}

TEST(MppiObstacleGeometryTest, WorkEstimateIsIndependentOfObstacleCount)
{
  for(const std::size_t obstacle_count : {129u, 1024u, 30000u}){
    const auto work = estimateMppiObstacleScoringWork(96, 75, 4, obstacle_count);
    EXPECT_EQ(work.clearance_index_queries, 1920u);
    EXPECT_EQ(work.collision_index_queries, 7200u);
    EXPECT_EQ(work.collision_pose_checks, 7200u);
  }
}

TEST(MppiObstacleGeometryTest, WorkEstimateMatchesEmptyCloudFastPath)
{
  const auto work = estimateMppiObstacleScoringWork(96, 75, 4, 0);
  EXPECT_EQ(work.totalIndexQueries(), 0u);
  EXPECT_EQ(work.collision_index_queries, 0u);
  EXPECT_EQ(work.collision_pose_checks, 7200u);
  EXPECT_FALSE(work.overflowed);
}

TEST(MppiObstacleGeometryTest, WorkEstimateSaturatesInsteadOfWrapping)
{
  const auto maximum = std::numeric_limits<std::size_t>::max();
  const auto work = estimateMppiObstacleScoringWork(maximum, maximum, 4, 64);
  EXPECT_TRUE(work.overflowed);
  EXPECT_EQ(work.collision_pose_checks, maximum);
  EXPECT_EQ(work.collision_index_queries, maximum);
  EXPECT_EQ(work.clearance_index_queries, maximum);
}

TEST(MppiObstacleGeometryTest, TotalIndexQueriesSaturatesAddition)
{
  local_planner::MppiObstacleScoringWork work;
  work.clearance_index_queries = std::numeric_limits<std::size_t>::max() - 2;
  work.collision_index_queries = 10;
  EXPECT_EQ(work.totalIndexQueries(), std::numeric_limits<std::size_t>::max());
}

TEST(MppiObstacleGeometryTest, QueryMetricLogFieldsRemainAligned)
{
  EXPECT_EQ(
    formatMppiObstacleQueryMetrics(11, 22, 33, 44),
    "collision_nodes=11, collision_candidates=22, clearance_nodes=33, "
    "clearance_candidates=44");
}

TEST(MppiObstacleGeometryTest, SpatialIndexCollisionMatchesFullScanOracleAcrossCloudSizes)
{
  std::mt19937 generator(17);
  std::uniform_real_distribution<float> xy(-5.0F, 5.0F);
  std::uniform_real_distribution<float> z(-0.3F, 1.0F);
  std::uniform_real_distribution<double> pose_xy(-2.0, 2.0);
  std::uniform_real_distribution<double> yaw(-kTestPi, kTestPi);

  for(const std::size_t obstacle_count : {129u, 1024u, 30000u}){
    auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    obstacles->reserve(obstacle_count);
    for(std::size_t index = 0; index < obstacle_count; ++index){
      pcl::PointXYZI point;
      point.x = xy(generator);
      point.y = xy(generator);
      point.z = z(generator);
      obstacles->push_back(point);
    }
    const auto snapshot = makeMppiObstacleSnapshot(obstacles);
    for(std::size_t sample = 0; sample < 12; ++sample){
      const auto cuboid = makeTestCuboid(pose_xy(generator), pose_xy(generator), 0.0, yaw(generator));
      std::vector<int> candidate_ids;
      ASSERT_TRUE(collectMppiObstacleCollisionCandidates(cuboid, snapshot, &candidate_ids));
      const std::unordered_set<int> unique_ids(candidate_ids.begin(), candidate_ids.end());
      EXPECT_EQ(unique_ids.size(), candidate_ids.size());
      EXPECT_EQ(
        hasObstacleCollisionForCuboid(cuboid, snapshot, &candidate_ids),
        hasObstacleCollisionForCuboid(cuboid, *snapshot.cloud));
    }
  }
}

TEST(MppiObstacleGeometryTest, SpatialIndexCoversRotatedCuboidAndFormerCellBoundaries)
{
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for(const float coordinate : {-1.0F, -0.5F, 0.0F, 0.5F, 1.0F}){
    pcl::PointXYZI point;
    point.x = coordinate;
    point.y = 0.0F;
    point.z = 0.0F;
    obstacles->push_back(point);
  }
  const auto snapshot = makeMppiObstacleSnapshot(obstacles);

  for(const double yaw : {0.0, kTestPi / 4.0, kTestPi / 2.0, -kTestPi / 3.0}){
    const auto cuboid = makeTestCuboid(0.5, 0.0, 0.0, yaw);
    std::vector<int> candidate_ids;
    ASSERT_TRUE(collectMppiObstacleCollisionCandidates(cuboid, snapshot, &candidate_ids));
    EXPECT_EQ(
      hasObstacleCollisionForCuboid(cuboid, snapshot, &candidate_ids),
      hasObstacleCollisionForCuboid(cuboid, *snapshot.cloud));
  }
}

TEST(MppiObstacleGeometryTest, SpatialIndexIncludesObstacleAtCombinedFarCorner)
{
  const double yaw = kTestPi / 4.0;
  const auto cuboid = makeTestCuboid(0.5, -0.5, 0.0, yaw);
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI far_corner;
  far_corner.x = cuboid.points[5].x;
  far_corner.y = cuboid.points[5].y;
  far_corner.z = cuboid.points[5].z;
  obstacles->push_back(far_corner);
  const auto snapshot = makeMppiObstacleSnapshot(obstacles);
  std::vector<int> candidate_ids;

  ASSERT_TRUE(collectMppiObstacleCollisionCandidates(cuboid, snapshot, &candidate_ids));
  ASSERT_EQ(candidate_ids.size(), 1u);
  EXPECT_TRUE(hasObstacleCollisionForCuboid(cuboid, snapshot, &candidate_ids));
}

TEST(MppiObstacleGeometryTest, RotatedTolerancePointOutsideWorldPaddingStillCollides)
{
  constexpr double yaw = kTestPi / 4.0;
  constexpr double local_tolerance_offset = 0.9e-6;
  const auto cuboid = makeTestCuboid(0.0, 0.0, 0.0, yaw);
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI tolerance_point;
  const double local_x = 0.50 + local_tolerance_offset;
  const double local_y = -0.32 - local_tolerance_offset;
  tolerance_point.x = static_cast<float>(
    std::cos(yaw) * local_x - std::sin(yaw) * local_y);
  tolerance_point.y = static_cast<float>(
    std::sin(yaw) * local_x + std::cos(yaw) * local_y);
  tolerance_point.z = 0.0F;
  obstacles->push_back(tolerance_point);
  const auto snapshot = makeMppiObstacleSnapshot(obstacles);
  std::vector<int> ids;

  ASSERT_TRUE(hasObstacleCollisionForCuboid(cuboid, *snapshot.cloud));
  const auto indexed = evaluateMppiObstacleCollisionForCuboid(cuboid, snapshot, &ids);
  EXPECT_TRUE(indexed.evaluated);
  EXPECT_TRUE(indexed.in_collision);
  EXPECT_GT(indexed.node_checks, 0u);
  EXPECT_GT(indexed.point_checks, 0u);
}

TEST(MppiObstacleGeometryTest, SpatialIndexHandlesExtremeFiniteRanges)
{
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI extreme;
  extreme.x = std::numeric_limits<float>::max();
  extreme.y = 0.0F;
  extreme.z = 0.0F;
  obstacles->push_back(extreme);
  const auto snapshot = makeMppiObstacleSnapshot(obstacles);
  const auto cuboid = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  std::vector<int> candidate_ids;

  EXPECT_TRUE(collectMppiObstacleCollisionCandidates(cuboid, snapshot, &candidate_ids));
  EXPECT_TRUE(candidate_ids.empty());
  EXPECT_EQ(
    hasObstacleCollisionForCuboid(cuboid, snapshot, &candidate_ids),
    hasObstacleCollisionForCuboid(cuboid, *snapshot.cloud));
}

TEST(MppiObstacleGeometryTest, MissingSpatialIndexFallsBackToExactFullScan)
{
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for(const float x : {-2.0F, 0.0F, 2.0F}){
    pcl::PointXYZI point;
    point.x = x;
    point.y = 0.0F;
    point.z = 0.0F;
    obstacles->push_back(point);
  }
  auto snapshot = makeMppiObstacleSnapshot(obstacles);
  snapshot.obstacle_index.reset();
  const auto cuboid = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  std::vector<int> ids;
  std::vector<float> squared_distances;
  pcl::PointXYZ center;
  center.x = 0.0F;
  center.y = 0.0F;
  center.z = 0.0F;

  const auto collision = evaluateMppiObstacleCollisionForCuboid(cuboid, snapshot, &ids);
  const auto clearance = computeAdaptiveObstacleClearanceForCuboid(
    cuboid, snapshot, center, 2.2, &ids, &squared_distances);
  const auto exact = computeObstacleClearanceForCuboid(cuboid, *snapshot.cloud, {0, 1, 2});
  EXPECT_TRUE(collision.evaluated);
  EXPECT_EQ(collision.in_collision, exact.in_collision);
  EXPECT_EQ(collision.point_checks, 2u);
  EXPECT_EQ(clearance.in_collision, exact.in_collision);
  EXPECT_DOUBLE_EQ(clearance.clearance, exact.clearance);
  EXPECT_EQ(clearance.point_checks, 2u);
}

TEST(MppiObstacleGeometryTest, MalformedCuboidsRemainUnevaluatedAndFailValidation)
{
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  obstacle.x = 0.0F;
  obstacle.y = 0.0F;
  obstacle.z = 0.0F;
  obstacles->push_back(obstacle);
  const auto snapshot = makeMppiObstacleSnapshot(obstacles);
  std::vector<int> scratch;

  pcl::PointCloud<pcl::PointXYZ> non_finite = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  non_finite.points[0].x = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ> degenerate = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  degenerate.points[3] = degenerate.points[0];
  pcl::PointCloud<pcl::PointXYZ> too_short;

  for(const auto * cuboid : {&non_finite, &degenerate, &too_short}){
    const auto collision = evaluateMppiObstacleCollisionForCuboid(*cuboid, snapshot, &scratch);
    EXPECT_FALSE(collision.evaluated);
    MppiScoredTrajectory trajectory = makeTrajectory(0.2, {MppiTrajectoryPoint{0.0, 0.0, 0.0}});
    trajectory.points[0].collision_evaluated = collision.evaluated;
    trajectory.points[0].in_collision = collision.in_collision;
    EXPECT_FALSE(isMppiTrajectoryCollisionFree(trajectory));
  }
}

TEST(MppiObstacleGeometryTest, AdaptiveClearanceMatchesFullScanWithinInflationDomain)
{
  constexpr double inflation_radius = 2.2;
  std::mt19937 generator(29);
  std::uniform_real_distribution<float> xy(-5.0F, 5.0F);
  std::uniform_real_distribution<float> z(-0.3F, 1.0F);
  std::uniform_real_distribution<double> pose_xy(-1.0, 1.0);
  std::uniform_real_distribution<double> yaw(-kTestPi, kTestPi);

  for(const std::size_t obstacle_count : {129u, 1024u, 30000u}){
    auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    obstacles->reserve(obstacle_count);
    for(std::size_t index = 0; index < obstacle_count; ++index){
      pcl::PointXYZI point;
      point.x = xy(generator);
      point.y = xy(generator);
      point.z = z(generator);
      obstacles->push_back(point);
    }
    const auto snapshot = makeMppiObstacleSnapshot(obstacles);
    for(std::size_t sample = 0; sample < 8; ++sample){
      const double center_x = pose_xy(generator);
      const double center_y = pose_xy(generator);
      const auto cuboid = makeTestCuboid(center_x, center_y, 0.0, yaw(generator));
      pcl::PointXYZ center;
      center.x = static_cast<float>(center_x);
      center.y = static_cast<float>(center_y);
      center.z = 0.0F;
      std::vector<int> ids;
      std::vector<float> squared_distances;
      const auto adaptive = computeAdaptiveObstacleClearanceForCuboid(
        cuboid, snapshot, center, inflation_radius, &ids, &squared_distances);
      std::vector<int> all_ids(snapshot.cloud->points.size());
      std::iota(all_ids.begin(), all_ids.end(), 0);
      const auto exact = computeObstacleClearanceForCuboid(cuboid, *snapshot.cloud, all_ids);

      if(exact.clearance < inflation_radius){
        EXPECT_NEAR(adaptive.clearance, exact.clearance, 1e-5)
          << "obstacles=" << obstacle_count << ", sample=" << sample
          << ", adaptive_ids=" << ids.size();
      }
      else{
        EXPECT_GE(adaptive.clearance, inflation_radius);
      }
    }
  }
}

TEST(MppiObstacleGeometryTest, DensePathStyleQueryMatchesFullScanOracle)
{
  constexpr double collision_margin = 0.10;
  auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  obstacles->reserve(30000);
  std::mt19937 generator(41);
  std::uniform_real_distribution<float> xy(-5.0F, 5.0F);
  std::uniform_real_distribution<float> z(-0.3F, 1.0F);
  for(std::size_t index = 0; index < 29999; ++index){
    pcl::PointXYZI point;
    point.x = xy(generator);
    point.y = xy(generator);
    point.z = z(generator);
    if(std::fabs(point.x) < 1.0F && std::fabs(point.y) < 1.0F){
      point.x = 3.0F;
    }
    obstacles->push_back(point);
  }
  pcl::PointXYZI margin_obstacle;
  margin_obstacle.x = 0.55F;
  margin_obstacle.y = 0.0F;
  margin_obstacle.z = 0.0F;
  obstacles->push_back(margin_obstacle);

  const auto snapshot = makeMppiObstacleSnapshot(obstacles);
  const auto cuboid = makeTestCuboid(0.0, 0.0, 0.0, 0.0);
  pcl::PointXYZ center;
  center.x = 0.0F;
  center.y = 0.0F;
  center.z = 0.0F;
  std::vector<int> ids;
  std::vector<float> squared_distances;
  const auto collision = evaluateMppiObstacleCollisionForCuboid(cuboid, snapshot, &ids);
  const auto clearance = computeAdaptiveObstacleClearanceForCuboid(
    cuboid, snapshot, center, collision_margin, &ids, &squared_distances);

  std::vector<int> all_ids(snapshot.cloud->points.size());
  std::iota(all_ids.begin(), all_ids.end(), 0);
  const auto exact = computeObstacleClearanceForCuboid(cuboid, *snapshot.cloud, all_ids);
  ASSERT_TRUE(collision.evaluated);
  EXPECT_EQ(collision.in_collision, exact.in_collision);
  EXPECT_NEAR(clearance.clearance, exact.clearance, 1e-5);
  EXPECT_FALSE(
    collision.evaluated && !collision.in_collision &&
    clearance.clearance >= collision_margin);
}

TEST(MppiObstacleGeometryTest, RepresentativeScoringGeometryFitsFifteenHertzBudget)
{
  constexpr std::size_t batch_size = 96;
  constexpr std::size_t trajectory_point_count = 75;
  constexpr std::size_t trajectory_point_step = 4;
  constexpr double inflation_radius = 2.2;
  struct Scenario
  {
    std::size_t obstacle_count;
    bool dense_wall;
    bool dense_bucket;
  };

  for(const auto scenario : {
      Scenario{129u, false, false},
      Scenario{1024u, true, false},
      Scenario{30000u, false, false},
      Scenario{30000u, false, true}})
  {
    auto obstacles = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    obstacles->reserve(scenario.obstacle_count);
    std::mt19937 generator(static_cast<unsigned int>(scenario.obstacle_count));
    std::uniform_real_distribution<float> xy(-5.0F, 5.0F);
    std::uniform_real_distribution<float> z(-0.3F, 1.0F);
    for(std::size_t index = 0; index < scenario.obstacle_count; ++index){
      pcl::PointXYZI obstacle;
      if(scenario.dense_bucket){
        obstacle.x = 0.45F + 0.04F * static_cast<float>(index) /
          static_cast<float>(scenario.obstacle_count);
        obstacle.y = 0.49F;
        obstacle.z = 0.49F;
      }
      else if(scenario.dense_wall){
        obstacle.x = 1.20F;
        obstacle.y = -4.0F + 8.0F * static_cast<float>(index % 64) / 63.0F;
        obstacle.z = -0.3F + 1.3F * static_cast<float>(index / 64) / 15.0F;
      }
      else{
        obstacle.x = xy(generator);
        obstacle.y = xy(generator);
        obstacle.z = z(generator);
      }
      obstacles->push_back(obstacle);
    }

    std::size_t evaluated_poses = 0;
    std::size_t collision_count = 0;
    std::size_t clearance_calls = 0;
    std::size_t collision_point_checks = 0;
    std::size_t clearance_point_checks = 0;
    double clearance_sum = 0.0;
    bool every_pose_evaluated = true;
    const auto start = std::chrono::steady_clock::now();
    const auto snapshot = makeMppiObstacleSnapshot(obstacles);
    std::vector<int> ids;
    std::vector<float> squared_distances;
    const auto evaluate_pose = [&](const double x, const double y, const double yaw,
                                   const bool evaluate_clearance) {
      const auto cuboid = makeTestCuboid(x, y, 0.0, yaw);
      const auto collision = evaluateMppiObstacleCollisionForCuboid(cuboid, snapshot, &ids);
      every_pose_evaluated = every_pose_evaluated && collision.evaluated;
      collision_count += collision.in_collision ? 1u : 0u;
      collision_point_checks += collision.point_checks;
      if(evaluate_clearance){
        if(collision.in_collision){
          clearance_sum += 0.0;
        }
        else{
          pcl::PointXYZ center;
          center.x = static_cast<float>(x);
          center.y = static_cast<float>(y);
          center.z = 0.0F;
          const auto clearance = computeAdaptiveObstacleClearanceForCuboid(
            cuboid,
            snapshot,
            center,
            inflation_radius,
            &ids,
            &squared_distances);
          ++clearance_calls;
          clearance_point_checks += clearance.point_checks;
          if(std::isfinite(clearance.clearance)){
            clearance_sum += clearance.clearance;
          }
        }
      }
      ++evaluated_poses;
    };

    for(std::size_t trajectory = 0; trajectory < batch_size; ++trajectory){
      const double lateral_offset =
        scenario.dense_bucket ? 0.0 :
        (static_cast<double>(trajectory % 12) - 5.5) * 0.04;
      const double yaw =
        scenario.dense_bucket ? 0.0 :
        (static_cast<double>(trajectory / 12) - 3.5) * 0.012;
      for(std::size_t point = 0; point < trajectory_point_count; ++point){
        evaluate_pose(
          scenario.dense_bucket ? 0.0 : static_cast<double>(point) * 0.01,
          lateral_offset,
          yaw,
          point % trajectory_point_step == 0 || point + 1 == trajectory_point_count);
      }
    }
    for(std::size_t point = 0; point < trajectory_point_count; ++point){
      evaluate_pose(
        scenario.dense_bucket ? 0.0 : static_cast<double>(point) * 0.01,
        0.0,
        0.0,
        true);
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start);
    std::cout << "MPPI geometry benchmark: obstacles=" << scenario.obstacle_count
              << ", dense_wall=" << scenario.dense_wall
              << ", dense_bucket=" << scenario.dense_bucket
              << ", elapsed_ms=" << elapsed.count() << std::endl;

    EXPECT_EQ(evaluated_poses, 7275u) << "obstacles=" << scenario.obstacle_count;
    EXPECT_TRUE(every_pose_evaluated) << "obstacles=" << scenario.obstacle_count;
    EXPECT_TRUE(std::isfinite(clearance_sum));
    EXPECT_LE(collision_count, evaluated_poses);
    if(scenario.dense_bucket){
      EXPECT_EQ(collision_count, 0u);
      EXPECT_EQ(clearance_calls, 1995u);
      EXPECT_LT(collision_point_checks, evaluated_poses * 32u);
      EXPECT_LT(clearance_point_checks, clearance_calls * 64u);
    }
    EXPECT_LT(elapsed.count(), 45.0)
      << "Full MPPI obstacle cycle exceeded 45 ms, obstacles="
      << scenario.obstacle_count << ", dense_wall=" << scenario.dense_wall;
  }
}

TEST(MppiObstacleGeometryTest, SnapshotFiltersNonFinitePointsBeforeBuildingKdTree)
{
  auto source = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI valid;
  valid.x = 1.0F;
  valid.y = 0.0F;
  valid.z = 0.0F;
  source->push_back(valid);
  pcl::PointXYZI nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = 0.0F;
  nan_point.z = 0.0F;
  source->push_back(nan_point);
  pcl::PointXYZI infinite_point;
  infinite_point.x = 0.0F;
  infinite_point.y = std::numeric_limits<float>::infinity();
  infinite_point.z = 0.0F;
  source->push_back(infinite_point);

  const auto snapshot = makeMppiObstacleSnapshot(source);

  ASSERT_TRUE(snapshot.cloud);
  ASSERT_TRUE(snapshot.kdtree);
  ASSERT_EQ(snapshot.cloud->points.size(), 1u);
  pcl::PointXYZI query;
  query.x = 1.0F;
  query.y = 0.0F;
  query.z = 0.0F;
  std::vector<int> ids;
  std::vector<float> distances;
  EXPECT_EQ(snapshot.kdtree->radiusSearch(query, 0.1, ids, distances), 1);
}

TEST(MppiObstacleGeometryTest, SnapshotTurnsAllNonFiniteInputIntoEmptyFastPath)
{
  auto source = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = 0.0F;
  nan_point.z = 0.0F;
  source->push_back(nan_point);
  pcl::PointXYZI infinite_point;
  infinite_point.x = 0.0F;
  infinite_point.y = 0.0F;
  infinite_point.z = -std::numeric_limits<float>::infinity();
  source->push_back(infinite_point);

  const auto snapshot = makeMppiObstacleSnapshot(source);

  ASSERT_TRUE(snapshot.cloud);
  ASSERT_TRUE(snapshot.kdtree);
  EXPECT_TRUE(snapshot.cloud->points.empty());
  const auto work = estimateMppiObstacleScoringWork(96, 75, 4, snapshot.cloud->points.size());
  EXPECT_EQ(work.totalIndexQueries(), 0u);
}

TEST(MppiObstacleGeometryTest, SnapshotOwnsCloudUsedByKdTree)
{
  auto source = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointXYZI obstacle;
  obstacle.x = 1.0F;
  obstacle.y = 0.0F;
  obstacle.z = 0.0F;
  obstacle.intensity = 0.0F;
  source->push_back(obstacle);

  const auto snapshot = makeMppiObstacleSnapshot(source);
  source->clear();

  ASSERT_TRUE(snapshot.cloud);
  ASSERT_TRUE(snapshot.kdtree);
  ASSERT_EQ(snapshot.cloud->points.size(), 1u);

  pcl::PointXYZI query;
  query.x = 1.0F;
  query.y = 0.0F;
  query.z = 0.0F;
  query.intensity = 0.0F;
  std::vector<int> ids;
  std::vector<float> distances;

  EXPECT_EQ(snapshot.kdtree->radiusSearch(query, 0.1, ids, distances), 1);
}
