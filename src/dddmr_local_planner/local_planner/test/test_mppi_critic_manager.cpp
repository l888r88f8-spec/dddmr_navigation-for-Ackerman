#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "local_planner/mppi_critic_manager.h"
#include "local_planner/mppi_obstacle_geometry.h"

using local_planner::computeObstacleClearanceForCuboid;
using local_planner::annotateMppiCandidateMetadata;
using local_planner::classifyMppiBypassSide;
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
      MppiTrajectoryPoint{0.5, 0.02, 0.0},
      MppiTrajectoryPoint{1.5, 0.03, 0.0}})};
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
  EXPECT_TRUE(result.scores[2].collision_free);
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
