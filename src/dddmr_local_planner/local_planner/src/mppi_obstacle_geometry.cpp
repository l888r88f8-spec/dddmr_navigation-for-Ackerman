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

MppiObstacleClearanceResult evaluateObstacleCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices,
  const bool calculate_clearance)
{
  MppiObstacleClearanceResult result;
  if(cuboid.points.size() < 4 || obstacles.points.empty() || obstacle_indices.empty()){
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

  for(const auto id : obstacle_indices){
    if(id < 0 || static_cast<std::size_t>(id) >= obstacles.points.size()){
      ++result.ignored_indices;
      continue;
    }

    const auto & obstacle = obstacles.points[static_cast<std::size_t>(id)];
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
      return result;
    }

    if(calculate_clearance){
      const double clearance_x = std::max(0.0, x_value - half_x);
      const double clearance_y = std::max(0.0, y_value - half_y);
      const double clearance_z = std::max(0.0, z_value - half_z);
      result.clearance = std::min(
        result.clearance,
        std::sqrt(
          clearance_x * clearance_x +
          clearance_y * clearance_y +
          clearance_z * clearance_z));
    }
  }

  return result;
}

}  // namespace

MppiObstacleClearanceResult computeObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices)
{
  return evaluateObstacleCuboid(cuboid, obstacles, obstacle_indices, true);
}

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> & obstacle_indices)
{
  return evaluateObstacleCuboid(cuboid, obstacles, obstacle_indices, false).in_collision;
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
