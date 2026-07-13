#ifndef LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_
#define LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace local_planner
{

struct MppiObstacleClearanceResult
{
  bool in_collision = false;
  double clearance = std::numeric_limits<double>::infinity();
  std::size_t ignored_indices = 0;
};

struct MppiObstacleSnapshot
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree;
};

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles);

MppiObstacleClearanceResult computeObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices);

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_
