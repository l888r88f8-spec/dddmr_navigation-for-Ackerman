#ifndef PERCEPTION_3D_GROUND_SURFACE_FILTER_H_
#define PERCEPTION_3D_GROUND_SURFACE_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perception_3d
{

struct GroundSurfaceFilterConfig {
  bool merge_ground_layers = false;
  double merge_ground_xy_resolution = 0.1;
  double merge_ground_z_tolerance = 0.12;
};

pcl::PointCloud<pcl::PointXYZI>::Ptr FilterGroundSurface(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  const GroundSurfaceFilterConfig& config);

}  // namespace perception_3d

#endif  // PERCEPTION_3D_GROUND_SURFACE_FILTER_H_
