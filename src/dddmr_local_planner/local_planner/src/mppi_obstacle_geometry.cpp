#include "local_planner/mppi_obstacle_geometry.h"

#include <algorithm>
#include <cmath>

namespace local_planner
{

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles)
{
  MppiObstacleSnapshot snapshot;
  snapshot.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  snapshot.kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

  if(obstacles){
    *snapshot.cloud = *obstacles;
  }
  if(!snapshot.cloud->points.empty()){
    snapshot.kdtree->setInputCloud(snapshot.cloud);
  }

  return snapshot;
}

namespace
{

std::size_t saturatingMultiply(
  const std::size_t lhs,
  const std::size_t rhs,
  bool & overflowed)
{
  if(lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs){
    overflowed = true;
    return std::numeric_limits<std::size_t>::max();
  }
  return lhs * rhs;
}

MppiObstacleClearanceResult evaluateObstacleCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> * obstacle_indices,
  const bool calculate_clearance)
{
  MppiObstacleClearanceResult result;
  double minimum_squared_clearance = std::numeric_limits<double>::infinity();
  if(cuboid.points.size() < 4 || obstacles.points.empty() ||
     (obstacle_indices != nullptr && obstacle_indices->empty()))
  {
    return result;
  }

  pcl::PointXYZ cuboid_center;
  cuboid_center.x = 0.0;
  cuboid_center.y = 0.0;
  cuboid_center.z = 0.0;
  for(const auto & point : cuboid.points){
    cuboid_center.x += point.x;
    cuboid_center.y += point.y;
    cuboid_center.z += point.z;
  }
  cuboid_center.x /= cuboid.points.size();
  cuboid_center.y /= cuboid.points.size();
  cuboid_center.z /= cuboid.points.size();

  pcl::PointXYZ dx;
  dx.x = cuboid.points[3].x - cuboid.points[0].x;
  dx.y = cuboid.points[3].y - cuboid.points[0].y;
  dx.z = cuboid.points[3].z - cuboid.points[0].z;
  pcl::PointXYZ dy;
  dy.x = cuboid.points[1].x - cuboid.points[0].x;
  dy.y = cuboid.points[1].y - cuboid.points[0].y;
  dy.z = cuboid.points[1].z - cuboid.points[0].z;
  pcl::PointXYZ dz;
  dz.x = cuboid.points[2].x - cuboid.points[0].x;
  dz.y = cuboid.points[2].y - cuboid.points[0].y;
  dz.z = cuboid.points[2].z - cuboid.points[0].z;

  const double half_x = std::sqrt(dx.x * dx.x + dx.y * dx.y + dx.z * dx.z) * 0.5;
  const double half_y = std::sqrt(dy.x * dy.x + dy.y * dy.y + dy.z * dy.z) * 0.5;
  const double half_z = std::sqrt(dz.x * dz.x + dz.y * dz.y + dz.z * dz.z) * 0.5;
  if(half_x <= 1e-6 || half_y <= 1e-6 || half_z <= 1e-6){
    return result;
  }

  dx.x /= (2.0 * half_x);
  dx.y /= (2.0 * half_x);
  dx.z /= (2.0 * half_x);
  dy.x /= (2.0 * half_y);
  dy.y /= (2.0 * half_y);
  dy.z /= (2.0 * half_y);
  dz.x /= (2.0 * half_z);
  dz.y /= (2.0 * half_z);
  dz.z /= (2.0 * half_z);

  const auto evaluate_obstacle = [&](const pcl::PointXYZI & obstacle) {
    if(!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
       !std::isfinite(obstacle.z))
    {
      return false;
    }

    pcl::PointXYZ delta;
    delta.x = obstacle.x - cuboid_center.x;
    delta.y = obstacle.y - cuboid_center.y;
    delta.z = obstacle.z - cuboid_center.z;

    const double x_value = std::fabs(delta.x * dx.x + delta.y * dx.y + delta.z * dx.z);
    const double y_value = std::fabs(delta.x * dy.x + delta.y * dy.y + delta.z * dy.z);
    const double z_value = std::fabs(delta.x * dz.x + delta.y * dz.y + delta.z * dz.z);

    if(x_value <= half_x && y_value <= half_y && z_value <= half_z){
      result.in_collision = true;
      result.clearance = 0.0;
      return true;
    }

    if(calculate_clearance){
      const double clearance_x = std::max(0.0, x_value - half_x);
      const double clearance_y = std::max(0.0, y_value - half_y);
      const double clearance_z = std::max(0.0, z_value - half_z);
      minimum_squared_clearance = std::min(
        minimum_squared_clearance,
        clearance_x * clearance_x +
        clearance_y * clearance_y +
        clearance_z * clearance_z);
    }
    return false;
  };

  if(obstacle_indices == nullptr){
    for(const auto & obstacle : obstacles.points){
      if(evaluate_obstacle(obstacle)){
        break;
      }
    }
    if(calculate_clearance && std::isfinite(minimum_squared_clearance)){
      result.clearance = std::sqrt(minimum_squared_clearance);
    }
    return result;
  }

  for(const auto id : *obstacle_indices){
    if(id < 0 || static_cast<std::size_t>(id) >= obstacles.points.size()){
      ++result.ignored_indices;
      continue;
    }
    if(evaluate_obstacle(obstacles.points[static_cast<std::size_t>(id)])){
      break;
    }
  }

  if(calculate_clearance && std::isfinite(minimum_squared_clearance)){
    result.clearance = std::sqrt(minimum_squared_clearance);
  }

  return result;
}

}  // namespace

MppiObstacleClearanceResult computeObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices)
{
  return evaluateObstacleCuboid(cuboid, obstacles, &obstacle_indices, true);
}

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices)
{
  return evaluateObstacleCuboid(cuboid, obstacles, &obstacle_indices, false).in_collision;
}

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles)
{
  return evaluateObstacleCuboid(cuboid, obstacles, nullptr, false).in_collision;
}

bool shouldDirectlyScanMppiObstacles(const std::size_t obstacle_count)
{
  // The workload model budgets one radiusSearch (tree traversal, result collection and
  // indexed OBB scan) as at most 128 direct OBB point checks. The target-configuration
  // latency test below this helper verifies that this crossover fits one 15 Hz period.
  constexpr std::size_t kKdTreeQueryEquivalentPointChecks = 128;
  return obstacle_count <= kKdTreeQueryEquivalentPointChecks;
}

MppiObstacleScoringWork estimateMppiObstacleScoringWork(
  const std::size_t batch_size,
  const std::size_t trajectory_point_count,
  const std::size_t trajectory_point_step,
  const std::size_t obstacle_count)
{
  MppiObstacleScoringWork work;
  if(batch_size == 0 || trajectory_point_count == 0){
    return work;
  }

  work.collision_pose_checks = saturatingMultiply(
    batch_size, trajectory_point_count, work.overflowed);
  if(obstacle_count == 0){
    return work;
  }

  const std::size_t step = std::max<std::size_t>(1, trajectory_point_step);
  std::size_t clearance_points = (trajectory_point_count - 1) / step + 1;
  if((trajectory_point_count - 1) % step != 0){
    ++clearance_points;
  }
  const std::size_t collision_only_points = trajectory_point_count - clearance_points;
  work.clearance_kdtree_queries = saturatingMultiply(
    batch_size, clearance_points, work.overflowed);
  if(shouldDirectlyScanMppiObstacles(obstacle_count)){
    const std::size_t collision_only_pose_checks = saturatingMultiply(
      batch_size, collision_only_points, work.overflowed);
    work.direct_collision_point_checks = saturatingMultiply(
      collision_only_pose_checks, obstacle_count, work.overflowed);
  }
  else{
    work.collision_kdtree_queries = saturatingMultiply(
      batch_size, collision_only_points, work.overflowed);
  }
  return work;
}

double computeMppiCuboidCollisionQueryRadius(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointXYZ & pose_center)
{
  double maximum_squared_distance = 0.0;
  for(const auto & vertex : cuboid.points){
    const double dx = static_cast<double>(vertex.x) - pose_center.x;
    const double dy = static_cast<double>(vertex.y) - pose_center.y;
    const double dz = static_cast<double>(vertex.z) - pose_center.z;
    const double squared_distance = dx * dx + dy * dy + dz * dz;
    if(!std::isfinite(squared_distance)){
      return std::numeric_limits<double>::infinity();
    }
    maximum_squared_distance = std::max(maximum_squared_distance, squared_distance);
  }
  return std::sqrt(maximum_squared_distance) + 1e-6;
}

}  // namespace local_planner
