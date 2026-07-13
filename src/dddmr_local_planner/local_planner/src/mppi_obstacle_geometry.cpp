#include "local_planner/mppi_obstacle_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace local_planner
{

namespace
{
constexpr double kMppiObstacleCollisionCellSize = 0.5;
constexpr std::size_t kMaximumCollisionCellsPerCuboid = 4096;

struct MppiObstacleCellKey
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const MppiObstacleCellKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct MppiObstacleCellKeyHash
{
  std::size_t operator()(const MppiObstacleCellKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

bool coordinateToCell(const double coordinate, std::int64_t * cell)
{
  if(cell == nullptr || !std::isfinite(coordinate)){
    return false;
  }
  const long double scaled = std::floor(
    static_cast<long double>(coordinate) /
    static_cast<long double>(kMppiObstacleCollisionCellSize));
  if(!std::isfinite(scaled) ||
     scaled < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
     scaled > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    return false;
  }
  *cell = static_cast<std::int64_t>(scaled);
  return true;
}

bool cellSpan(
  const std::int64_t minimum,
  const std::int64_t maximum,
  std::size_t * span)
{
  if(span == nullptr || maximum < minimum){
    return false;
  }
  const long double difference =
    static_cast<long double>(maximum) - static_cast<long double>(minimum) + 1.0L;
  if(difference <= 0.0L ||
     difference > static_cast<long double>(kMaximumCollisionCellsPerCuboid))
  {
    return false;
  }
  *span = static_cast<std::size_t>(difference);
  return true;
}
}  // namespace

class MppiObstacleSpatialHash
{
public:
  bool valid = true;
  std::unordered_map<MppiObstacleCellKey, std::vector<int>, MppiObstacleCellKeyHash> cells;
};

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles)
{
  MppiObstacleSnapshot snapshot;
  snapshot.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  snapshot.kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  auto collision_hash = std::make_shared<MppiObstacleSpatialHash>();

  if(obstacles){
    snapshot.cloud->reserve(obstacles->points.size());
    for(const auto & obstacle : obstacles->points){
      if(std::isfinite(obstacle.x) && std::isfinite(obstacle.y) &&
         std::isfinite(obstacle.z))
      {
        snapshot.cloud->push_back(obstacle);
      }
    }
    snapshot.cloud->is_dense = true;
  }
  collision_hash->cells.reserve(snapshot.cloud->points.size());
  for(std::size_t index = 0; index < snapshot.cloud->points.size(); ++index){
    const auto & obstacle = snapshot.cloud->points[index];
    MppiObstacleCellKey key;
    if(!coordinateToCell(obstacle.x, &key.x) ||
       !coordinateToCell(obstacle.y, &key.y) ||
       !coordinateToCell(obstacle.z, &key.z) ||
       index > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
      collision_hash->valid = false;
      collision_hash->cells.clear();
      break;
    }
    collision_hash->cells[key].push_back(static_cast<int>(index));
  }
  snapshot.collision_hash = collision_hash;
  if(!snapshot.cloud->points.empty()){
    snapshot.kdtree->setInputCloud(snapshot.cloud);
  }

  return snapshot;
}

namespace
{

struct MppiCuboidGeometry
{
  double center[3] = {0.0, 0.0, 0.0};
  double axis_x[3] = {0.0, 0.0, 0.0};
  double axis_y[3] = {0.0, 0.0, 0.0};
  double axis_z[3] = {0.0, 0.0, 0.0};
  double half_x = 0.0;
  double half_y = 0.0;
  double half_z = 0.0;
};

bool makeMppiCuboidGeometry(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  MppiCuboidGeometry * geometry)
{
  if(geometry == nullptr || cuboid.points.size() < 4){
    return false;
  }

  MppiCuboidGeometry parsed;
  for(const auto & point : cuboid.points){
    if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)){
      return false;
    }
    parsed.center[0] += static_cast<double>(point.x);
    parsed.center[1] += static_cast<double>(point.y);
    parsed.center[2] += static_cast<double>(point.z);
  }
  const double inverse_size = 1.0 / static_cast<double>(cuboid.points.size());
  for(std::size_t axis = 0; axis < 3; ++axis){
    parsed.center[axis] *= inverse_size;
    if(!std::isfinite(parsed.center[axis])){
      return false;
    }
  }

  const auto fill_axis = [&](const std::size_t endpoint, double * axis, double * half) {
    axis[0] = static_cast<double>(cuboid.points[endpoint].x) - cuboid.points[0].x;
    axis[1] = static_cast<double>(cuboid.points[endpoint].y) - cuboid.points[0].y;
    axis[2] = static_cast<double>(cuboid.points[endpoint].z) - cuboid.points[0].z;
    const double length = std::hypot(axis[0], axis[1], axis[2]);
    if(!std::isfinite(length) || length <= 2e-6){
      return false;
    }
    *half = length * 0.5;
    axis[0] /= length;
    axis[1] /= length;
    axis[2] /= length;
    return true;
  };
  if(!fill_axis(3, parsed.axis_x, &parsed.half_x) ||
     !fill_axis(1, parsed.axis_y, &parsed.half_y) ||
     !fill_axis(2, parsed.axis_z, &parsed.half_z))
  {
    return false;
  }

  *geometry = parsed;
  return true;
}

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

MppiObstacleClearanceResult evaluateObstacleCuboidWithGeometry(
  const MppiCuboidGeometry & geometry,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> * obstacle_indices,
  const bool calculate_clearance)
{
  MppiObstacleClearanceResult result;
  double minimum_squared_clearance = std::numeric_limits<double>::infinity();
  if(obstacles.points.empty() || (obstacle_indices != nullptr && obstacle_indices->empty()))
  {
    return result;
  }

  const auto evaluate_obstacle = [&](const pcl::PointXYZI & obstacle) {
    if(!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
       !std::isfinite(obstacle.z))
    {
      return false;
    }

    const double delta_x = static_cast<double>(obstacle.x) - geometry.center[0];
    const double delta_y = static_cast<double>(obstacle.y) - geometry.center[1];
    const double delta_z = static_cast<double>(obstacle.z) - geometry.center[2];

    const auto projected_distance = [&](const double * axis) {
      return std::fabs(delta_x * axis[0] + delta_y * axis[1] + delta_z * axis[2]);
    };
    const double x_value = projected_distance(geometry.axis_x);
    const double y_value = projected_distance(geometry.axis_y);
    const double z_value = projected_distance(geometry.axis_z);

    constexpr double kCuboidBoundaryTolerance = 1e-6;
    if(x_value <= geometry.half_x + kCuboidBoundaryTolerance &&
       y_value <= geometry.half_y + kCuboidBoundaryTolerance &&
       z_value <= geometry.half_z + kCuboidBoundaryTolerance)
    {
      result.in_collision = true;
      result.clearance = 0.0;
      return true;
    }

    if(calculate_clearance){
      const double clearance_x = std::max(0.0, x_value - geometry.half_x);
      const double clearance_y = std::max(0.0, y_value - geometry.half_y);
      const double clearance_z = std::max(0.0, z_value - geometry.half_z);
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
    if(calculate_clearance && !result.in_collision && std::isfinite(minimum_squared_clearance)){
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

  if(calculate_clearance && !result.in_collision && std::isfinite(minimum_squared_clearance)){
    result.clearance = std::sqrt(minimum_squared_clearance);
  }

  return result;
}

MppiObstacleClearanceResult evaluateObstacleCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const pcl::PointCloud<pcl::PointXYZI> & obstacles,
  const std::vector<int> * obstacle_indices,
  const bool calculate_clearance)
{
  MppiCuboidGeometry geometry;
  if(!makeMppiCuboidGeometry(cuboid, &geometry)){
    return MppiObstacleClearanceResult{};
  }
  return evaluateObstacleCuboidWithGeometry(
    geometry, obstacles, obstacle_indices, calculate_clearance);
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

bool collectMppiObstacleCollisionCandidates(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices)
{
  if(obstacle_indices == nullptr){
    return false;
  }
  obstacle_indices->clear();
  MppiCuboidGeometry geometry;
  if(!makeMppiCuboidGeometry(cuboid, &geometry)){
    return false;
  }
  if(!snapshot.cloud || snapshot.cloud->points.empty()){
    return true;
  }
  if(!snapshot.collision_hash || !snapshot.collision_hash->valid){
    return false;
  }

  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for(const auto & vertex : cuboid.points){
    if(!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z)){
      return false;
    }
    minimum_x = std::min(minimum_x, static_cast<double>(vertex.x));
    minimum_y = std::min(minimum_y, static_cast<double>(vertex.y));
    minimum_z = std::min(minimum_z, static_cast<double>(vertex.z));
    maximum_x = std::max(maximum_x, static_cast<double>(vertex.x));
    maximum_y = std::max(maximum_y, static_cast<double>(vertex.y));
    maximum_z = std::max(maximum_z, static_cast<double>(vertex.z));
  }
  constexpr double kCellBoundaryPadding = 1e-6;
  MppiObstacleCellKey minimum;
  MppiObstacleCellKey maximum;
  if(!coordinateToCell(minimum_x - kCellBoundaryPadding, &minimum.x) ||
     !coordinateToCell(minimum_y - kCellBoundaryPadding, &minimum.y) ||
     !coordinateToCell(minimum_z - kCellBoundaryPadding, &minimum.z) ||
     !coordinateToCell(maximum_x + kCellBoundaryPadding, &maximum.x) ||
     !coordinateToCell(maximum_y + kCellBoundaryPadding, &maximum.y) ||
     !coordinateToCell(maximum_z + kCellBoundaryPadding, &maximum.z))
  {
    return false;
  }

  std::size_t span_x = 0;
  std::size_t span_y = 0;
  std::size_t span_z = 0;
  if(!cellSpan(minimum.x, maximum.x, &span_x) ||
     !cellSpan(minimum.y, maximum.y, &span_y) ||
     !cellSpan(minimum.z, maximum.z, &span_z) ||
     span_x > kMaximumCollisionCellsPerCuboid / span_y ||
     span_x * span_y > kMaximumCollisionCellsPerCuboid / span_z)
  {
    return false;
  }

  for(std::int64_t x = minimum.x;; ++x){
    for(std::int64_t y = minimum.y;; ++y){
      for(std::int64_t z = minimum.z;; ++z){
        const auto cell = snapshot.collision_hash->cells.find(MppiObstacleCellKey{x, y, z});
        if(cell != snapshot.collision_hash->cells.end()){
          obstacle_indices->insert(
            obstacle_indices->end(), cell->second.begin(), cell->second.end());
        }
        if(z == maximum.z){break;}
      }
      if(y == maximum.y){break;}
    }
    if(x == maximum.x){break;}
  }
  return true;
}

bool hasObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices_scratch)
{
  return evaluateMppiObstacleCollisionForCuboid(
    cuboid, snapshot, obstacle_indices_scratch).in_collision;
}

MppiObstacleCollisionResult evaluateMppiObstacleCollisionForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  std::vector<int> * obstacle_indices_scratch)
{
  MppiObstacleCollisionResult result;
  MppiCuboidGeometry geometry;
  if(!makeMppiCuboidGeometry(cuboid, &geometry) || obstacle_indices_scratch == nullptr){
    return result;
  }

  result.evaluated = true;
  if(!snapshot.cloud || snapshot.cloud->points.empty()){
    obstacle_indices_scratch->clear();
    return result;
  }
  if(!collectMppiObstacleCollisionCandidates(cuboid, snapshot, obstacle_indices_scratch)){
    result.in_collision = hasObstacleCollisionForCuboid(cuboid, *snapshot.cloud);
    return result;
  }
  result.in_collision = hasObstacleCollisionForCuboid(
    cuboid, *snapshot.cloud, *obstacle_indices_scratch);
  return result;
}

MppiObstacleClearanceResult computeAdaptiveObstacleClearanceForCuboid(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid,
  const MppiObstacleSnapshot & snapshot,
  const pcl::PointXYZ & pose_center,
  const double inflation_radius,
  std::vector<int> * obstacle_indices_scratch,
  std::vector<float> * squared_distances_scratch)
{
  MppiObstacleClearanceResult result;
  MppiCuboidGeometry geometry;
  if(!makeMppiCuboidGeometry(cuboid, &geometry) ||
     !snapshot.cloud || snapshot.cloud->points.empty())
  {
    return result;
  }
  if(!snapshot.kdtree || obstacle_indices_scratch == nullptr ||
     squared_distances_scratch == nullptr || !std::isfinite(pose_center.x) ||
     !std::isfinite(pose_center.y) || !std::isfinite(pose_center.z) ||
     !std::isfinite(inflation_radius) || inflation_radius < 0.0)
  {
    return result;
  }

  pcl::PointXYZI query;
  query.x = pose_center.x;
  query.y = pose_center.y;
  query.z = pose_center.z;
  query.intensity = 0.0F;
  obstacle_indices_scratch->resize(1);
  squared_distances_scratch->resize(1);
  if(snapshot.kdtree->nearestKSearch(
      query, 1, *obstacle_indices_scratch, *squared_distances_scratch) <= 0)
  {
    obstacle_indices_scratch->clear();
    squared_distances_scratch->clear();
    return result;
  }
  const auto seed = evaluateObstacleCuboidWithGeometry(
    geometry, *snapshot.cloud, obstacle_indices_scratch, true);
  const double seed_clearance = std::isfinite(seed.clearance) ?
    std::max(0.0, seed.clearance) : inflation_radius;
  const double cuboid_radius = computeMppiCuboidCollisionQueryRadius(cuboid, pose_center);
  const double query_radius =
    cuboid_radius + std::min(seed_clearance, inflation_radius) + 1e-6;
  if(!std::isfinite(query_radius) || query_radius <= 0.0){
    obstacle_indices_scratch->clear();
    squared_distances_scratch->clear();
    return result;
  }

  obstacle_indices_scratch->clear();
  squared_distances_scratch->clear();
  snapshot.kdtree->radiusSearch(
    query, query_radius, *obstacle_indices_scratch, *squared_distances_scratch);
  return evaluateObstacleCuboidWithGeometry(
    geometry, *snapshot.cloud, obstacle_indices_scratch, true);
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
  work.collision_hash_queries = work.collision_pose_checks;

  const std::size_t step = std::max<std::size_t>(1, trajectory_point_step);
  std::size_t clearance_points = (trajectory_point_count - 1) / step + 1;
  if((trajectory_point_count - 1) % step != 0){
    ++clearance_points;
  }
  work.clearance_nearest_kdtree_queries = saturatingMultiply(
    batch_size, clearance_points, work.overflowed);
  work.clearance_radius_kdtree_queries = work.clearance_nearest_kdtree_queries;
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
