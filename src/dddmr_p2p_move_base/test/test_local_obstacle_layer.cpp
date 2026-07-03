#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "p2p_move_base/local_obstacle_layer.h"

using p2p_move_base::CollisionQuery;
using p2p_move_base::LocalObstacleLayer;
using p2p_move_base::LocalObstacleLayerConfig;
using p2p_move_base::LocalPoint;
using p2p_move_base::Pose2D;

namespace
{
LocalObstacleLayerConfig testConfig()
{
  LocalObstacleLayerConfig config;
  config.roi_forward = 4.0;
  config.roi_backward = 1.0;
  config.roi_left = 2.0;
  config.roi_right = 2.0;
  config.min_z = -0.1;
  config.max_z = 1.0;
  config.voxel_leaf_size = 0.5;
  config.max_snapshot_points = 4;
  config.stale_timeout_sec = 0.3;
  config.footprint_front = 0.5;
  config.footprint_back = 0.4;
  config.footprint_left = 0.3;
  config.footprint_right = 0.3;
  return config;
}
}  // namespace

TEST(LocalObstacleLayerTest, FiltersPointsOutsideRoiAndHeightLimits)
{
  LocalObstacleLayer layer(testConfig());
  const std::vector<LocalPoint> points = {
    {1.0, 0.0, 0.0},
    {4.5, 0.0, 0.0},
    {-1.5, 0.0, 0.0},
    {1.0, 2.5, 0.0},
    {1.0, -2.5, 0.0},
    {1.0, 0.0, -0.2},
    {1.0, 0.0, 1.2},
  };

  const auto snapshot = layer.buildSnapshot(points, 10.0, 10.1, "base_link");

  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->raw_points, points.size());
  EXPECT_EQ(snapshot->filtered_points, 1u);
  ASSERT_EQ(snapshot->obstacle_points.size(), 1u);
  EXPECT_DOUBLE_EQ(snapshot->obstacle_points.front().x, 1.0);
  EXPECT_DOUBLE_EQ(snapshot->obstacle_points.front().y, 0.0);
  EXPECT_FALSE(snapshot->perception_stale);
}

TEST(LocalObstacleLayerTest, VoxelDownsamplesAndCapsSnapshotPoints)
{
  LocalObstacleLayerConfig config = testConfig();
  config.max_snapshot_points = 2;
  LocalObstacleLayer layer(config);
  const std::vector<LocalPoint> points = {
    {0.10, 0.10, 0.0},
    {0.20, 0.20, 0.0},
    {1.10, 0.10, 0.0},
    {2.10, 0.10, 0.0},
  };

  const auto snapshot = layer.buildSnapshot(points, 10.0, 10.1, "base_link");

  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->filtered_points, 2u);
  EXPECT_EQ(snapshot->obstacle_points.size(), 2u);
  EXPECT_TRUE(snapshot->over_budget);
}

TEST(LocalObstacleLayerTest, MarksSnapshotStale)
{
  LocalObstacleLayer layer(testConfig());

  const auto snapshot = layer.buildSnapshot({{1.0, 0.0, 0.0}}, 10.0, 10.5, "base_link");

  ASSERT_NE(snapshot, nullptr);
  EXPECT_TRUE(snapshot->perception_stale);
}

TEST(CollisionQueryTest, DetectsFootprintCollisionAndDistance)
{
  LocalObstacleLayerConfig config = testConfig();
  LocalObstacleLayer layer(config);
  const auto snapshot = layer.buildSnapshot(
    {{0.2, 0.0, 0.0}, {2.0, 0.0, 0.0}},
    10.0,
    10.0,
    "base_link");
  CollisionQuery query(snapshot, config);

  EXPECT_TRUE(query.footprintCollides({0.0, 0.0, 0.0}));
  EXPECT_FALSE(query.footprintCollides({1.5, 1.5, 0.0}));
  EXPECT_NEAR(query.minDistanceToObstacle({1.0, 0.0, 0.0}), 0.8, 1e-6);
}

TEST(CollisionQueryTest, DetectsTrajectoryCollision)
{
  LocalObstacleLayerConfig config = testConfig();
  LocalObstacleLayer layer(config);
  const auto snapshot = layer.buildSnapshot({{1.0, 0.0, 0.0}}, 10.0, 10.0, "base_link");
  CollisionQuery query(snapshot, config);

  EXPECT_TRUE(query.trajectoryCollides({{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}}));
  EXPECT_FALSE(query.trajectoryCollides({{0.0, 1.0, 0.0}, {0.8, 1.0, 0.0}}));
}

TEST(CollisionQueryTest, EmptySnapshotHasNoCollision)
{
  LocalObstacleLayerConfig config = testConfig();
  CollisionQuery query(std::shared_ptr<const p2p_move_base::LocalObstacleSnapshot>(), config);

  EXPECT_FALSE(query.footprintCollides({0.0, 0.0, 0.0}));
  EXPECT_FALSE(query.trajectoryCollides({{0.0, 0.0, 0.0}}));
  EXPECT_TRUE(std::isinf(query.minDistanceToObstacle({0.0, 0.0, 0.0})));
}
