#ifndef LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_
#define LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace local_planner
{

class MppiObstacleSpatialIndex;

struct MppiObstacleClearanceResult
{
  bool in_collision = false;
  double clearance = std::numeric_limits<double>::infinity();
  std::size_t ignored_indices = 0;
  std::size_t node_checks = 0;
  std::size_t point_checks = 0;
};

struct MppiObstacleCollisionResult
{
  bool in_collision = false;
  bool evaluated = false;
  std::size_t node_checks = 0;
  std::size_t point_checks = 0;
};

struct MppiObstacleSnapshot
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree;
  std::shared_ptr<const MppiObstacleSpatialIndex> obstacle_index;
};

struct MppiObstacleScoringWork
{
  std::size_t clearance_index_queries = 0;
  std::size_t collision_index_queries = 0;
  std::size_t collision_pose_checks = 0;
  bool overflowed = false;

  std::size_t totalIndexQueries() const
  {
    if(clearance_index_queries >
       std::numeric_limits<std::size_t>::max() - collision_index_queries)
    {
      return std::numeric_limits<std::size_t>::max();
    }
    return clearance_index_queries + collision_index_queries;
  }
};

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles);

MppiObstacleClearanceResult computeObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices);

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices);

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles);

bool collectMppiObstacleCollisionCandidates(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices);

MppiObstacleCollisionResult evaluateMppiObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices_scratch);

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices_scratch);

MppiObstacleClearanceResult computeAdaptiveObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  const pcl::PointXYZ & pose_center,
  double inflation_radius,
  std::vector<int> * obstacle_indices_scratch,
  std::vector<float> * squared_distances_scratch);

MppiObstacleScoringWork estimateMppiObstacleScoringWork(
  std::size_t batch_size,
  std::size_t trajectory_point_count,
  std::size_t trajectory_point_step,
  std::size_t obstacle_count);

double computeMppiCuboidCollisionQueryRadius(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointXYZ & pose_center);

std::string formatMppiObstacleQueryMetrics(
  std::size_t collision_node_checks,
  std::size_t collision_point_checks,
  std::size_t clearance_node_checks,
  std::size_t clearance_point_checks);

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_
