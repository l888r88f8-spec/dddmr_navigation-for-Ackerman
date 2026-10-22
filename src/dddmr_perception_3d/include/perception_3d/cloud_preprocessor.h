#ifndef PERCEPTION_3D_CLOUD_PREPROCESSOR_H_
#define PERCEPTION_3D_CLOUD_PREPROCESSOR_H_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace perception_3d
{

struct CloudPreprocessorConfig
{
  double window = 6.0;
  double min_height = -0.12;
  double max_height = 2.0;
  double voxel_size = 0.1;
};

// Crop and voxelize in body coordinates, then publish stable world coordinates.
pcl::PointCloud<pcl::PointXYZ> PreprocessCloud(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  const Eigen::Affine3d & sensor_to_body,
  const Eigen::Affine3d & body_to_global,
  const CloudPreprocessorConfig & config);

sensor_msgs::msg::PointCloud2 MakePreprocessedCloudMessage(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const std_msgs::msg::Header & acquisition_header,
  const std::string & global_frame);

bool IsCloudStampFresh(
  const builtin_interfaces::msg::Time & stamp,
  const rclcpp::Time & now, double max_age);

}  // namespace perception_3d
#endif
