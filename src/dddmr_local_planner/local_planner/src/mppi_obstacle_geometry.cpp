#include "local_planner/mppi_obstacle_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <sstream>

namespace local_planner
{

class MppiObstacleSpatialIndex
{
public:
  struct Node
  {
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    std::size_t begin = 0;
    std::size_t end = 0;
    int left = -1;
    int right = -1;

    bool isLeaf() const {return left < 0;}
  };

  std::vector<Node> nodes;
  std::vector<int> point_indices;
};

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles,
  const bool build_compatibility_kdtree)
{
  MppiObstacleSnapshot snapshot;
  snapshot.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  snapshot.kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  auto obstacle_index = std::make_shared<MppiObstacleSpatialIndex>();

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
  const std::size_t maximum_indexed_points =
    static_cast<std::size_t>(std::numeric_limits<int>::max()) / 2u;
  if(snapshot.cloud->points.size() <= maximum_indexed_points)
  {
    obstacle_index->point_indices.resize(snapshot.cloud->points.size());
    std::iota(
      obstacle_index->point_indices.begin(), obstacle_index->point_indices.end(), 0);
    obstacle_index->nodes.reserve(snapshot.cloud->points.size() * 2);
    constexpr std::size_t kMaximumLeafPoints = 24;
    const auto coordinate = [&](const int point_index, const std::size_t axis) {
      const auto & point = snapshot.cloud->points[static_cast<std::size_t>(point_index)];
      if(axis == 0){return static_cast<double>(point.x);}
      if(axis == 1){return static_cast<double>(point.y);}
      return static_cast<double>(point.z);
    };
    std::function<int(std::size_t, std::size_t)> build_node =
      [&](const std::size_t begin, const std::size_t end) {
        MppiObstacleSpatialIndex::Node node;
        node.begin = begin;
        node.end = end;
        node.minimum.fill(std::numeric_limits<double>::infinity());
        node.maximum.fill(-std::numeric_limits<double>::infinity());
        for(std::size_t offset = begin; offset < end; ++offset){
          const int point_index = obstacle_index->point_indices[offset];
          for(std::size_t axis = 0; axis < 3; ++axis){
            const double value = coordinate(point_index, axis);
            node.minimum[axis] = std::min(node.minimum[axis], value);
            node.maximum[axis] = std::max(node.maximum[axis], value);
          }
        }
        const int node_index = static_cast<int>(obstacle_index->nodes.size());
        obstacle_index->nodes.push_back(node);
        if(end - begin <= kMaximumLeafPoints){
          return node_index;
        }

        std::size_t split_axis = 0;
        for(std::size_t axis = 1; axis < 3; ++axis){
          if(node.maximum[axis] - node.minimum[axis] >
             node.maximum[split_axis] - node.minimum[split_axis])
          {
            split_axis = axis;
          }
        }
        const std::size_t middle = begin + (end - begin) / 2;
        std::nth_element(
          obstacle_index->point_indices.begin() + static_cast<std::ptrdiff_t>(begin),
          obstacle_index->point_indices.begin() + static_cast<std::ptrdiff_t>(middle),
          obstacle_index->point_indices.begin() + static_cast<std::ptrdiff_t>(end),
          [&](const int lhs, const int rhs) {
            return coordinate(lhs, split_axis) < coordinate(rhs, split_axis);
          });
        const int left = build_node(begin, middle);
        const int right = build_node(middle, end);
        obstacle_index->nodes[static_cast<std::size_t>(node_index)].left = left;
        obstacle_index->nodes[static_cast<std::size_t>(node_index)].right = right;
        return node_index;
      };
    if(!obstacle_index->point_indices.empty()){
      build_node(0, obstacle_index->point_indices.size());
    }
  }
  snapshot.obstacle_index = obstacle_index;
  if(build_compatibility_kdtree && !snapshot.cloud->points.empty()){
    snapshot.kdtree->setInputCloud(snapshot.cloud);
  }

  return snapshot;
}

MppiObstacleSnapshot makeMppiObstacleSnapshot(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & obstacles)
{
  return makeMppiObstacleSnapshot(obstacles, true);
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

constexpr double kMppiCuboidBoundaryTolerance = 1e-6;

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
  const auto dot = [](const double * lhs, const double * rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
  };
  constexpr double kOrthogonalityTolerance = 1e-4;
  if(std::fabs(dot(parsed.axis_x, parsed.axis_y)) > kOrthogonalityTolerance ||
     std::fabs(dot(parsed.axis_x, parsed.axis_z)) > kOrthogonalityTolerance ||
     std::fabs(dot(parsed.axis_y, parsed.axis_z)) > kOrthogonalityTolerance)
  {
    return false;
  }

  *geometry = parsed;
  return true;
}

double obstacleClearanceToCuboid(
  const MppiCuboidGeometry & geometry,
  const pcl::PointXYZI & obstacle,
  bool * in_collision)
{
  const double delta_x = static_cast<double>(obstacle.x) - geometry.center[0];
  const double delta_y = static_cast<double>(obstacle.y) - geometry.center[1];
  const double delta_z = static_cast<double>(obstacle.z) - geometry.center[2];
  const auto projected_distance = [&](const double * axis) {
    return std::fabs(delta_x * axis[0] + delta_y * axis[1] + delta_z * axis[2]);
  };
  const double x_value = projected_distance(geometry.axis_x);
  const double y_value = projected_distance(geometry.axis_y);
  const double z_value = projected_distance(geometry.axis_z);
  const bool collision =
    x_value <= geometry.half_x + kMppiCuboidBoundaryTolerance &&
    y_value <= geometry.half_y + kMppiCuboidBoundaryTolerance &&
    z_value <= geometry.half_z + kMppiCuboidBoundaryTolerance;
  if(in_collision != nullptr){
    *in_collision = collision;
  }
  if(collision){
    return 0.0;
  }
  const double clearance_x = std::max(0.0, x_value - geometry.half_x);
  const double clearance_y = std::max(0.0, y_value - geometry.half_y);
  const double clearance_z = std::max(0.0, z_value - geometry.half_z);
  return std::hypot(clearance_x, clearance_y, clearance_z);
}

std::array<double, 3> obstacleNodeAxisGaps(
  const MppiCuboidGeometry & geometry,
  const MppiObstacleSpatialIndex::Node & node)
{
  // The AABB projection interval is exact on each orthonormal OBB axis. Every
  // contained point is at least each interval gap away on that axis, so the L2
  // norm of all three gaps is a conservative point-to-OBB distance lower bound.
  const double node_center[3] = {
    (node.minimum[0] + node.maximum[0]) * 0.5,
    (node.minimum[1] + node.maximum[1]) * 0.5,
    (node.minimum[2] + node.maximum[2]) * 0.5};
  const double node_half[3] = {
    (node.maximum[0] - node.minimum[0]) * 0.5,
    (node.maximum[1] - node.minimum[1]) * 0.5,
    (node.maximum[2] - node.minimum[2]) * 0.5};
  const double * axes[3] = {geometry.axis_x, geometry.axis_y, geometry.axis_z};
  const double cuboid_halves[3] = {
    geometry.half_x, geometry.half_y, geometry.half_z};
  double gaps[3] = {0.0, 0.0, 0.0};
  for(std::size_t axis_index = 0; axis_index < 3; ++axis_index){
    const double * axis = axes[axis_index];
    const double node_projection =
      node_center[0] * axis[0] + node_center[1] * axis[1] + node_center[2] * axis[2];
    const double node_projection_radius =
      node_half[0] * std::fabs(axis[0]) +
      node_half[1] * std::fabs(axis[1]) +
      node_half[2] * std::fabs(axis[2]);
    const double cuboid_projection =
      geometry.center[0] * axis[0] +
      geometry.center[1] * axis[1] +
      geometry.center[2] * axis[2];
    const double raw_gap =
      std::fabs(node_projection - cuboid_projection) -
      node_projection_radius - cuboid_halves[axis_index];
    const double rounding_margin =
      64.0 * std::numeric_limits<double>::epsilon() *
      (std::fabs(node_projection) + std::fabs(cuboid_projection) +
      node_projection_radius + cuboid_halves[axis_index] + 1.0);
    gaps[axis_index] = std::max(0.0, raw_gap - rounding_margin);
  }
  return {gaps[0], gaps[1], gaps[2]};
}

double obstacleNodeLowerBound(
  const MppiCuboidGeometry & geometry,
  const MppiObstacleSpatialIndex::Node & node)
{
  const auto gaps = obstacleNodeAxisGaps(geometry, node);
  return std::hypot(gaps[0], gaps[1], gaps[2]);
}

double obstacleNodeCollisionLowerBound(
  const MppiCuboidGeometry & geometry,
  const MppiObstacleSpatialIndex::Node & node)
{
  const auto gaps = obstacleNodeAxisGaps(geometry, node);
  // Exact collision expands every OBB interval independently by the boundary
  // tolerance, so collision pruning must use the L-infinity gap rather than L2.
  return std::max(gaps[0], std::max(gaps[1], gaps[2]));
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
  if(obstacles.points.empty() || (obstacle_indices != nullptr && obstacle_indices->empty()))
  {
    return result;
  }

  const auto evaluate_obstacle = [&](const pcl::PointXYZI & obstacle) {
    ++result.point_checks;
    if(!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
       !std::isfinite(obstacle.z))
    {
      return false;
    }

    bool collision = false;
    const double clearance = obstacleClearanceToCuboid(geometry, obstacle, &collision);
    if(collision){
      result.in_collision = true;
      result.clearance = 0.0;
      return true;
    }

    if(calculate_clearance){
      result.clearance = std::min(result.clearance, clearance);
    }
    return false;
  };

  if(obstacle_indices == nullptr){
    for(const auto & obstacle : obstacles.points){
      if(evaluate_obstacle(obstacle)){
        break;
      }
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
  if(!snapshot.obstacle_index || snapshot.obstacle_index->nodes.empty()){
    return false;
  }
  std::vector<int> stack{0};
  while(!stack.empty()){
    const int node_index = stack.back();
    stack.pop_back();
    const auto & node =
      snapshot.obstacle_index->nodes[static_cast<std::size_t>(node_index)];
    if(obstacleNodeCollisionLowerBound(geometry, node) >
       kMppiCuboidBoundaryTolerance)
    {
      continue;
    }
    if(node.isLeaf()){
      obstacle_indices->insert(
        obstacle_indices->end(),
        snapshot.obstacle_index->point_indices.begin() +
        static_cast<std::ptrdiff_t>(node.begin),
        snapshot.obstacle_index->point_indices.begin() +
        static_cast<std::ptrdiff_t>(node.end));
    }
    else{
      stack.push_back(node.left);
      stack.push_back(node.right);
    }
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
  if(!makeMppiCuboidGeometry(cuboid, &geometry)){
    return result;
  }
  if(obstacle_indices_scratch != nullptr){
    obstacle_indices_scratch->clear();
  }

  result.evaluated = true;
  if(!snapshot.cloud || snapshot.cloud->points.empty()){
    return result;
  }
  if(!snapshot.obstacle_index || snapshot.obstacle_index->nodes.empty()){
    const auto fallback = evaluateObstacleCuboidWithGeometry(
      geometry, *snapshot.cloud, nullptr, false);
    result.in_collision = fallback.in_collision;
    result.point_checks = fallback.point_checks;
    return result;
  }

  std::vector<int> stack{0};
  while(!stack.empty()){
    const int node_index = stack.back();
    stack.pop_back();
    const auto & node =
      snapshot.obstacle_index->nodes[static_cast<std::size_t>(node_index)];
    ++result.node_checks;
    if(obstacleNodeCollisionLowerBound(geometry, node) >
       kMppiCuboidBoundaryTolerance)
    {
      continue;
    }
    if(!node.isLeaf()){
      stack.push_back(node.left);
      stack.push_back(node.right);
      continue;
    }
    for(std::size_t offset = node.begin; offset < node.end; ++offset){
      const int point_index = snapshot.obstacle_index->point_indices[offset];
      ++result.point_checks;
      bool in_collision = false;
      (void)obstacleClearanceToCuboid(
        geometry,
        snapshot.cloud->points[static_cast<std::size_t>(point_index)],
        &in_collision);
      if(in_collision){
        result.in_collision = true;
        return result;
      }
    }
  }
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
  if(!std::isfinite(pose_center.x) ||
     !std::isfinite(pose_center.y) || !std::isfinite(pose_center.z) ||
     !std::isfinite(inflation_radius) || inflation_radius < 0.0)
  {
    return result;
  }
  if(obstacle_indices_scratch != nullptr){
    obstacle_indices_scratch->clear();
  }
  if(squared_distances_scratch != nullptr){
    squared_distances_scratch->clear();
  }
  if(!snapshot.obstacle_index || snapshot.obstacle_index->nodes.empty()){
    return evaluateObstacleCuboidWithGeometry(
      geometry, *snapshot.cloud, nullptr, true);
  }

  struct NodeVisit
  {
    int index = -1;
    double lower_bound = 0.0;
  };
  result.clearance = inflation_radius > 0.0 ?
    inflation_radius : kMppiCuboidBoundaryTolerance;
  std::vector<NodeVisit> stack;
  stack.reserve(64);
  stack.push_back(NodeVisit{
    0,
    obstacleNodeLowerBound(geometry, snapshot.obstacle_index->nodes.front())});
  while(!stack.empty()){
    const NodeVisit visit = stack.back();
    stack.pop_back();
    ++result.node_checks;
    const double prune_tolerance =
      1e-12 * std::max(1.0, std::fabs(result.clearance));
    if(visit.lower_bound >= result.clearance - prune_tolerance){
      continue;
    }
    const auto & node =
      snapshot.obstacle_index->nodes[static_cast<std::size_t>(visit.index)];
    if(node.isLeaf()){
      for(std::size_t offset = node.begin; offset < node.end; ++offset){
        const int point_index = snapshot.obstacle_index->point_indices[offset];
        ++result.point_checks;
        bool in_collision = false;
        const double clearance = obstacleClearanceToCuboid(
          geometry,
          snapshot.cloud->points[static_cast<std::size_t>(point_index)],
          &in_collision);
        if(in_collision){
          result.in_collision = true;
          result.clearance = 0.0;
          return result;
        }
        result.clearance = std::min(result.clearance, clearance);
      }
      continue;
    }
    const auto & left = snapshot.obstacle_index->nodes[static_cast<std::size_t>(node.left)];
    const auto & right = snapshot.obstacle_index->nodes[static_cast<std::size_t>(node.right)];
    NodeVisit first{node.left, obstacleNodeLowerBound(geometry, left)};
    NodeVisit second{node.right, obstacleNodeLowerBound(geometry, right)};
    if(second.lower_bound < first.lower_bound){
      std::swap(first, second);
    }
    if(second.lower_bound < result.clearance){
      stack.push_back(second);
    }
    if(first.lower_bound < result.clearance){
      stack.push_back(first);
    }
  }
  if(!std::isfinite(result.clearance)){
    result.clearance = inflation_radius;
  }
  if(result.clearance < 0.0){
    result.clearance = 0.0;
  }
  return result;
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
  work.collision_index_queries = work.collision_pose_checks;

  const std::size_t step = std::max<std::size_t>(1, trajectory_point_step);
  std::size_t clearance_points = (trajectory_point_count - 1) / step + 1;
  if((trajectory_point_count - 1) % step != 0){
    ++clearance_points;
  }
  work.clearance_index_queries = saturatingMultiply(
    batch_size, clearance_points, work.overflowed);
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

std::string formatMppiObstacleQueryMetrics(
  const std::size_t collision_node_checks,
  const std::size_t collision_point_checks,
  const std::size_t clearance_node_checks,
  const std::size_t clearance_point_checks)
{
  std::ostringstream stream;
  stream << "collision_nodes=" << collision_node_checks
         << ", collision_candidates=" << collision_point_checks
         << ", clearance_nodes=" << clearance_node_checks
         << ", clearance_candidates=" << clearance_point_checks;
  return stream.str();
}

}  // namespace local_planner
