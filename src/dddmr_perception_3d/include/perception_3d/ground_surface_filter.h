#ifndef PERCEPTION_3D_GROUND_SURFACE_FILTER_H_
#define PERCEPTION_3D_GROUND_SURFACE_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace perception_3d
{

struct GroundSurfaceFilterConfig {
  bool merge_ground_layers = false;
  double merge_ground_xy_resolution = 0.1;
  double merge_ground_z_tolerance = 0.12;
  bool force_single_ground_surface = false;
  double single_surface_max_slope_deg = 20.0;
  double single_surface_z_margin = 0.05;
  double single_surface_max_xy_shift = 0.05;
  std::string single_surface_select_policy = "largest_connected";
};

pcl::PointCloud<pcl::PointXYZI>::Ptr FilterGroundSurface(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  const GroundSurfaceFilterConfig& config);

}  // namespace perception_3d

#endif  // PERCEPTION_3D_GROUND_SURFACE_FILTER_H_
