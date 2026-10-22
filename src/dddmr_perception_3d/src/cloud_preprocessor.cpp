#include <perception_3d/cloud_preprocessor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <stdexcept>

namespace perception_3d
{
pcl::PointCloud<pcl::PointXYZ> PreprocessCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  const Eigen::Affine3d & sensor_to_body, const Eigen::Affine3d & body_to_global,
  const CloudPreprocessorConfig & config)
{
  if (!std::isfinite(config.window) || config.window <= 0.0 ||
    !std::isfinite(config.voxel_size) || config.voxel_size <= 0.0 ||
    !std::isfinite(config.min_height) || !std::isfinite(config.max_height) ||
    config.min_height > config.max_height ||
    !sensor_to_body.matrix().allFinite() || !body_to_global.matrix().allFinite())
  {
    throw std::invalid_argument("Invalid cloud preprocessing bounds or transform");
  }
  auto body = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  body->reserve(input.size());
  // Do not rely on an upstream is_dense flag to detect NaN/Inf.
  for (const auto & point : input) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      body->push_back(point);
    }
  }
  body->is_dense = true;
  if (body->empty()) {return *body;}
  pcl::transformPointCloud(*body, *body, sensor_to_body);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(body);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-config.window, config.window);
  pass.filter(*body);
  pass.setFilterFieldName("y");
  pass.filter(*body);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(config.min_height, config.max_height);
  pass.filter(*body);
  if (body->empty()) {return *body;}
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(body);
  const auto leaf = static_cast<float>(config.voxel_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*body);
  pcl::PointCloud<pcl::PointXYZ> output;
  pcl::transformPointCloud(*body, output, body_to_global);
  output.is_dense = true;
  return output;
}

sensor_msgs::msg::PointCloud2 MakePreprocessedCloudMessage(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std_msgs::msg::Header & header, const std::string & global_frame)
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  message.header.frame_id = global_frame;
  return message;
}

bool IsCloudStampFresh(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now, double max_age)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
    (stamp.sec == 0 && stamp.nanosec == 0) || !std::isfinite(max_age) || max_age <= 0.0)
  {
    return false;
  }
  const auto age = (now - rclcpp::Time(stamp, now.get_clock_type())).seconds();
  return age >= 0.0 && age <= max_age;
}
}  // namespace perception_3d
