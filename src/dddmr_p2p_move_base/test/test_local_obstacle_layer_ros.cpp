#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include "p2p_move_base/local_obstacle_layer.h"
#include "p2p_move_base/local_obstacle_layer_ros.h"

namespace
{
sensor_msgs::msg::PointCloud2 makeXyzCloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 3;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  *x = 1.0F;
  *y = 0.0F;
  *z = 0.1F;
  ++x;
  ++y;
  ++z;
  *x = 2.0F;
  *y = -0.5F;
  *z = 0.2F;
  ++x;
  ++y;
  ++z;
  *x = std::numeric_limits<float>::quiet_NaN();
  *y = 0.0F;
  *z = 0.0F;
  return cloud;
}
}  // namespace

TEST(LocalObstacleLayerRosTest, ExtractsFiniteXyzPointsFromPointCloud2)
{
  const auto points = p2p_move_base::pointCloud2ToLocalPoints(makeXyzCloud());

  ASSERT_EQ(points.size(), 2u);
  EXPECT_DOUBLE_EQ(points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(points[0].y, 0.0);
  EXPECT_NEAR(points[0].z, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(points[1].x, 2.0);
  EXPECT_DOUBLE_EQ(points[1].y, -0.5);
  EXPECT_NEAR(points[1].z, 0.2, 1e-6);
}

TEST(LocalObstacleLayerRosTest, RejectsCloudMissingRequiredFields)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 8;
  cloud.row_step = 8;
  cloud.data.resize(8);
  cloud.fields.resize(2);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  EXPECT_TRUE(p2p_move_base::pointCloud2ToLocalPoints(cloud).empty());
}

TEST(LocalObstacleLayerRosTest, BuildsStatusMessageFromSnapshot)
{
  p2p_move_base::LocalObstacleSnapshot snapshot;
  snapshot.frame_id = "base_link";
  snapshot.raw_points = 10;
  snapshot.filtered_points = 3;
  snapshot.perception_stale = true;
  snapshot.over_budget = false;
  snapshot.update_time_ms = 1.25;

  const auto status = p2p_move_base::snapshotToStatusMessage(snapshot);

  EXPECT_NE(status.data.find("\"frame_id\":\"base_link\""), std::string::npos);
  EXPECT_NE(status.data.find("\"raw_points\":10"), std::string::npos);
  EXPECT_NE(status.data.find("\"filtered_points\":3"), std::string::npos);
  EXPECT_NE(status.data.find("\"perception_stale\":true"), std::string::npos);
  EXPECT_NE(status.data.find("\"over_budget\":false"), std::string::npos);
}

TEST(LocalObstacleLayerRosTest, ConvertsSnapshotToDebugCloud)
{
  p2p_move_base::LocalObstacleSnapshot snapshot;
  snapshot.obstacle_points = {{1.0, 0.0, 0.1}, {2.0, -0.5, 0.2}};
  std_msgs::msg::Header header;
  header.frame_id = "base_link";

  const auto cloud = p2p_move_base::snapshotToPointCloud2(snapshot, header);

  EXPECT_EQ(cloud.header.frame_id, "base_link");
  EXPECT_EQ(cloud.width, 2u);
  EXPECT_EQ(cloud.height, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  EXPECT_FLOAT_EQ(*x, 1.0F);
  EXPECT_FLOAT_EQ(*y, 0.0F);
  EXPECT_FLOAT_EQ(*z, 0.1F);
}
