#ifndef LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_
#define LOCAL_PLANNER_MPPI_OBSTACLE_GEOMETRY_H_

#include <cstddef>
#include <cstdint>
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

// Lossless, POD, flat export of the immutable BVH built by
// makeMppiObstacleSnapshot(). Leaf point ranges index point_indices; the cloud
// itself is snapshot.cloud (already filtered to finite points). The export
// copies the arrays so callers may keep it independently of the snapshot.
struct MppiObstacleBvhNode
{
  double minimum[3] = {0.0, 0.0, 0.0};
  double maximum[3] = {0.0, 0.0, 0.0};
  std::uint32_t begin = 0;
  std::uint32_t end = 0;
  std::int32_t left = -1;
  std::int32_t right = -1;

  bool isLeaf() const {return left < 0;}
};

struct MppiObstacleBvhExport
{
  std::vector<MppiObstacleBvhNode> nodes;
  std::vector<std::int32_t> point_indices;
};

bool exportMppiObstacleBvh(
  const MppiObstacleSnapshot & snapshot,
  MppiObstacleBvhExport * output,
  std::string * error = nullptr);

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

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles,
  bool build_compatibility_kdtree);

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

// True when any obstacle point lies within `radius` of `centre`. Uses the
// compatibility kd-tree only when it has an input cloud; snapshots built with
// build_compatibility_kdtree=false fall back to a linear scan. The snapshot
// always owns a kdtree object, so checking the pointer alone is not enough.
bool mppiObstacleSnapshotHasPointWithin(
  const MppiObstacleSnapshot & snapshot,
  const pcl::PointXYZI & centre,
  double radius);

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
