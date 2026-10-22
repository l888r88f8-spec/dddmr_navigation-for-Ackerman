#include <gtest/gtest.h>

#include <perception_3d/cloud_preprocessor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud Sorted(Cloud cloud)
{
  std::sort(cloud.begin(), cloud.end(), [](const auto & a, const auto & b) {
    if (a.x != b.x) {return a.x < b.x;}
    if (a.y != b.y) {return a.y < b.y;}
    return a.z < b.z;
  });
  return cloud;
}
}  // namespace

TEST(CloudPreprocessor, CropsAtSixMetersAndHeightInBodyFrame)
{
  Cloud input;
  for (const auto & point : std::vector<pcl::PointXYZ>{
      {6.0f, 0.0f, 0.5f}, {-6.0f, 0.0f, 0.5f},
      {0.0f, 6.0f, 0.5f}, {0.0f, -6.0f, 0.5f},
      {6.01f, 0.0f, 0.5f}, {0.0f, -6.01f, 0.5f},
      {0.0f, 0.0f, -0.13f}, {0.0f, 0.0f, 2.01f}})
  {
    input.push_back(point);
  }
  const auto result = perception_3d::PreprocessCloud(
    input, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), {});
  ASSERT_EQ(result.size(), 4u);
  for (const auto & point : result) {
    EXPECT_LE(std::abs(point.x), 6.0f);
    EXPECT_LE(std::abs(point.y), 6.0f);
  }
}

TEST(CloudPreprocessor, MatchesLegacyBodyCropAndVoxelBeforeGlobalTransform)
{
  Cloud input;
  input.push_back({0.01f, 0.01f, 0.5f});
  input.push_back({0.02f, 0.02f, 0.5f});
  input.push_back({1.0f, 2.0f, 0.5f});
  input.push_back({8.0f, 0.0f, 0.5f});
  Eigen::Affine3d sensor_to_body = Eigen::Affine3d::Identity();
  sensor_to_body.translation() << 0.3, 0.1, 0.2;
  sensor_to_body.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  Eigen::Affine3d body_to_global = Eigen::Affine3d::Identity();
  body_to_global.translation() << 10.0, -20.0, 1.0;
  body_to_global.rotate(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));

  auto legacy = std::make_shared<Cloud>();
  pcl::transformPointCloud(input, *legacy, sensor_to_body);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(legacy);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-6.0, 6.0);
  pass.filter(*legacy);
  pass.setFilterFieldName("y");
  pass.filter(*legacy);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.12, 2.0);
  pass.filter(*legacy);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(legacy);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*legacy);
  pcl::transformPointCloud(*legacy, *legacy, body_to_global);
  const auto expected = Sorted(*legacy);
  const auto actual = Sorted(perception_3d::PreprocessCloud(
      input, sensor_to_body, body_to_global, {}));
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i].x, expected[i].x, 1e-5);
    EXPECT_NEAR(actual[i].y, expected[i].y, 1e-5);
    EXPECT_NEAR(actual[i].z, expected[i].z, 1e-5);
  }
}

TEST(CloudPreprocessor, RejectsNonfiniteEvenWhenInputIncorrectlyClaimsDense)
{
  Cloud input;
  input.push_back({1.0f, 0.0f, 0.5f});
  input.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.5f});
  input.push_back({0.0f, std::numeric_limits<float>::infinity(), 0.5f});
  input.is_dense = true;
  const auto result = perception_3d::PreprocessCloud(
    input, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), {});
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.is_dense);
}

TEST(CloudPreprocessor, EmptyObservationRetainsAcquisitionStampAndGlobalFrame)
{
  std_msgs::msg::Header header;
  header.frame_id = "hesai_lidar";
  header.stamp.sec = 123;
  header.stamp.nanosec = 987654321;
  const auto cloud = perception_3d::PreprocessCloud(
    {}, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), {});
  const auto message = perception_3d::MakePreprocessedCloudMessage(cloud, header, "map");
  EXPECT_EQ(message.width * message.height, 0u);
  EXPECT_EQ(message.header.stamp, header.stamp);
  EXPECT_EQ(message.header.frame_id, "map");
}

TEST(CloudPreprocessor, FreshnessUsesAcquisitionTimeAndRejectsZeroAndFuture)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  EXPECT_TRUE(perception_3d::IsCloudStampFresh(
      stamp, rclcpp::Time(100, 200000000, RCL_ROS_TIME), 0.5));
  EXPECT_FALSE(perception_3d::IsCloudStampFresh(
      stamp, rclcpp::Time(101, 0, RCL_ROS_TIME), 0.5));
  EXPECT_FALSE(perception_3d::IsCloudStampFresh(
      stamp, rclcpp::Time(99, 0, RCL_ROS_TIME), 0.5));
  EXPECT_FALSE(perception_3d::IsCloudStampFresh(
      builtin_interfaces::msg::Time(), rclcpp::Time(0, 0, RCL_ROS_TIME), 0.5));
}
