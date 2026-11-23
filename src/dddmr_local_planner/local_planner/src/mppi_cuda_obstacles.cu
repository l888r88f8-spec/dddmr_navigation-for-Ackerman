#include "local_planner/mppi_cuda_obstacles.h"

#include <cuda_runtime.h>

#include "local_planner/mppi_cuda_launch.h"
#include <math_constants.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace local_planner
{
namespace
{

constexpr int kMaxLocalVertices = 64;
constexpr int kTraversalStack = 128;
constexpr double kCuboidBoundaryTolerance = 1e-6;
constexpr double kOrthogonalityTolerance = 1e-4;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::size_t kMaximumPlanarIndex =
  static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max());

struct GpuPoint
{
  float x;
  float y;
  float z;
};

struct GpuBvhNode
{
  double minimum[3];
  double maximum[3];
  std::uint32_t begin;
  std::uint32_t end;
  std::int32_t left;
  std::int32_t right;
};

struct GpuCuboid
{
  double center[3];
  double axis[3][3];
  double half[3];
  int valid;
};

struct GpuConfig
{
  double model_dt;
  double inflation_radius;
  double evaluation_distance;
  double evaluation_step;
  std::size_t trajectory_point_step;
  std::size_t evaluation_max_points;
  int check_every_obstacle_point;
};

struct GpuPose
{
  double x;
  double y;
  double z;
  double yaw;
};

using GpuTrajectoryPoint = MppiCudaTrajectoryPoint;

// Pose hand-off between the split planar rollout phases. The pose phase stores
// the exact double result of the midpoint Ackermann integration, so the query
// phase evaluates the identical geometry the single-kernel rollout did.
struct GpuPlanarPose
{
  double x;
  double y;
  double yaw;
};

bool checkCuda(const cudaError_t status, const char * operation, std::string & error)
{
  if(status == cudaSuccess){
    return true;
  }
  std::ostringstream message;
  message << "CUDA " << operation << " failed: " << cudaGetErrorString(status) <<
    " (" << static_cast<int>(status) << ")";
  error = message.str();
  // A failed CUDA call is often sticky; consume it so the next independent
  // operation is not reported with this call's stale error.
  cudaGetLastError();
  return false;
}

template<typename T>
bool ensureCapacity(
  T * & pointer,
  std::size_t & capacity,
  const std::size_t needed,
  std::string & error)
{
  if(needed <= capacity){
    return true;
  }
  if(pointer != nullptr){
    if(!checkCuda(cudaFree(pointer), "cudaFree", error)){
      pointer = nullptr;
      capacity = 0;
      return false;
    }
    pointer = nullptr;
    capacity = 0;
  }
  if(needed == 0){
    return true;
  }
  void * allocated = nullptr;
  if(!checkCuda(cudaMalloc(&allocated, needed * sizeof(T)), "cudaMalloc", error)){
    return false;
  }
  pointer = static_cast<T *>(allocated);
  capacity = needed;
  return true;
}

// O(1) structural checks only, mirroring the host query's
// planarGridStructureValid. Prefix-sum monotonicity is trusted from
// buildMppiPlanarObstacleGrid; re-scanning it here would be O(C). Returns true
// and fills detail when the grid must be rejected.
bool planarGridUploadRejected(
  const MppiPlanarObstacleGrid & grid, std::string & detail)
{
  if(!grid.valid){
    detail = "grid is not marked valid";
    return true;
  }
  if(!std::isfinite(grid.cell_size) || grid.cell_size <= 0.0){
    detail = "cell_size must be finite and positive";
    return true;
  }
  if(!std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y)){
    detail = "grid origin must be finite";
    return true;
  }
  if(grid.cells_x > kMaximumPlanarIndex ||
     grid.cells_y > kMaximumPlanarIndex)
  {
    detail = "grid cell count exceeds the 32-bit index range";
    return true;
  }
  if(grid.cells_y != 0 &&
     grid.cells_x > std::numeric_limits<std::size_t>::max() / grid.cells_y)
  {
    detail = "grid cell count overflows the addressable index range";
    return true;
  }
  const std::size_t cell_count = grid.cells_x * grid.cells_y;
  if(cell_count == std::numeric_limits<std::size_t>::max()){
    detail = "grid cell count exceeds the addressable index range";
    return true;
  }
  if(grid.point_count > kMaximumPlanarIndex ||
     grid.point_count > std::numeric_limits<std::size_t>::max() / 2)
  {
    detail = "grid point count exceeds the 32-bit index range";
    return true;
  }
  if(grid.cell_offsets.size() != cell_count + 1){
    detail = "cell_offsets size does not match the cell count";
    return true;
  }
  if(grid.points_xy.size() != grid.point_count * 2){
    detail = "points_xy size does not match the point count";
    return true;
  }
  if(grid.cell_offsets.front() != 0){
    detail = "cell_offsets[0] is not zero";
    return true;
  }
  if(grid.cell_offsets.back() != grid.point_count){
    detail = "the cell_offsets prefix sum does not end at point_count";
    return true;
  }
  return false;
}

__device__ double deviceNormalizeAngle(double angle)
{
  const double result = fmod(angle + kPi, 2.0 * kPi);
  if(result <= 0.0){
    return result + kPi;
  }
  return result - kPi;
}

__device__ bool parseCuboidAxes(
  const GpuPoint * transformed_prefix,
  GpuCuboid & cuboid)
{
  const std::size_t endpoints[3] = {3, 1, 2};
  for(std::size_t axis = 0; axis < 3; ++axis){
    const GpuPoint & endpoint = transformed_prefix[endpoints[axis]];
    const double dx =
      static_cast<double>(endpoint.x) - transformed_prefix[0].x;
    const double dy =
      static_cast<double>(endpoint.y) - transformed_prefix[0].y;
    const double dz =
      static_cast<double>(endpoint.z) - transformed_prefix[0].z;
    const double length = hypot(hypot(dx, dy), dz);
    if(!isfinite(length) || length <= 2e-6){
      return false;
    }
    cuboid.half[axis] = length * 0.5;
    cuboid.axis[axis][0] = dx / length;
    cuboid.axis[axis][1] = dy / length;
    cuboid.axis[axis][2] = dz / length;
  }
  for(std::size_t first = 0; first < 3; ++first){
    for(std::size_t second = first + 1; second < 3; ++second){
      const double dot =
        cuboid.axis[first][0] * cuboid.axis[second][0] +
        cuboid.axis[first][1] * cuboid.axis[second][1] +
        cuboid.axis[first][2] * cuboid.axis[second][2];
      if(!isfinite(dot) || fabs(dot) > kOrthogonalityTolerance){
        return false;
      }
    }
  }
  cuboid.valid = 1;
  return true;
}

__device__ bool transformAndBuildCuboid(
  const GpuPoint * local,
  const std::size_t count,
  const GpuPose & pose,
  GpuCuboid & cuboid)
{
  cuboid.valid = 0;
  if(count < 4 || count > static_cast<std::size_t>(kMaxLocalVertices)){
    return false;
  }
  const double cosine = cos(pose.yaw);
  const double sine = sin(pose.yaw);
  // Exactly local_planner.cpp::transformRobotCuboid: 3-D rotation in double,
  // translation, then PCL float storage before the OBB parser promotes it.
  // The OBB parser only reads vertices 0..3, so stream the remaining
  // transformed vertices directly into the center accumulator.
  GpuPoint transformed_prefix[4];
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  for(std::size_t i = 0; i < count; ++i){
    const double rotated_x =
      cosine * static_cast<double>(local[i].x) -
      sine * static_cast<double>(local[i].y);
    const double rotated_y =
      sine * static_cast<double>(local[i].x) +
      cosine * static_cast<double>(local[i].y);
    const float transformed_x = static_cast<float>(pose.x + rotated_x);
    const float transformed_y = static_cast<float>(pose.y + rotated_y);
    const float transformed_z = static_cast<float>(pose.z +
      static_cast<double>(local[i].z));
    if(!isfinite(transformed_x) || !isfinite(transformed_y) ||
       !isfinite(transformed_z))
    {
      return false;
    }
    if(i < 4){
      transformed_prefix[i] = GpuPoint{
        transformed_x, transformed_y, transformed_z};
    }
    center_x += static_cast<double>(transformed_x);
    center_y += static_cast<double>(transformed_y);
    center_z += static_cast<double>(transformed_z);
  }
  const double inverse_count = 1.0 / static_cast<double>(count);
  cuboid.center[0] = center_x * inverse_count;
  cuboid.center[1] = center_y * inverse_count;
  cuboid.center[2] = center_z * inverse_count;
  for(std::size_t axis = 0; axis < 3; ++axis){
    if(!isfinite(cuboid.center[axis])){
      return false;
    }
  }
  return parseCuboidAxes(transformed_prefix, cuboid);
}

__device__ bool cuboidCollision(
  const GpuCuboid & cuboid,
  const GpuPoint & point,
  double & clearance)
{
  const double delta[3] = {
    static_cast<double>(point.x) - cuboid.center[0],
    static_cast<double>(point.y) - cuboid.center[1],
    static_cast<double>(point.z) - cuboid.center[2]};
  double projected[3];
  for(std::size_t axis = 0; axis < 3; ++axis){
    projected[axis] = fabs(
      delta[0] * cuboid.axis[axis][0] +
      delta[1] * cuboid.axis[axis][1] +
      delta[2] * cuboid.axis[axis][2]);
  }
  const bool collision =
    projected[0] <= cuboid.half[0] + kCuboidBoundaryTolerance &&
    projected[1] <= cuboid.half[1] + kCuboidBoundaryTolerance &&
    projected[2] <= cuboid.half[2] + kCuboidBoundaryTolerance;
  if(collision){
    clearance = 0.0;
    return true;
  }
  const double dx = fmax(0.0, projected[0] - cuboid.half[0]);
  const double dy = fmax(0.0, projected[1] - cuboid.half[1]);
  const double dz = fmax(0.0, projected[2] - cuboid.half[2]);
  clearance = hypot(hypot(dx, dy), dz);
  return false;
}

__device__ void nodeAxisGaps(
  const GpuCuboid & cuboid,
  const GpuBvhNode & node,
  double gaps[3])
{
  const double node_center[3] = {
    (node.minimum[0] + node.maximum[0]) * 0.5,
    (node.minimum[1] + node.maximum[1]) * 0.5,
    (node.minimum[2] + node.maximum[2]) * 0.5};
  const double node_half[3] = {
    (node.maximum[0] - node.minimum[0]) * 0.5,
    (node.maximum[1] - node.minimum[1]) * 0.5,
    (node.maximum[2] - node.minimum[2]) * 0.5};
  for(std::size_t axis = 0; axis < 3; ++axis){
    const double node_projection =
      node_center[0] * cuboid.axis[axis][0] +
      node_center[1] * cuboid.axis[axis][1] +
      node_center[2] * cuboid.axis[axis][2];
    const double node_projection_radius =
      node_half[0] * fabs(cuboid.axis[axis][0]) +
      node_half[1] * fabs(cuboid.axis[axis][1]) +
      node_half[2] * fabs(cuboid.axis[axis][2]);
    const double cuboid_projection =
      cuboid.center[0] * cuboid.axis[axis][0] +
      cuboid.center[1] * cuboid.axis[axis][1] +
      cuboid.center[2] * cuboid.axis[axis][2];
    const double raw_gap =
      fabs(node_projection - cuboid_projection) -
      node_projection_radius - cuboid.half[axis];
    const double rounding_margin =
      64.0 * 2.2204460492503130808472633361816e-16 *
      (fabs(node_projection) + fabs(cuboid_projection) +
      node_projection_radius + cuboid.half[axis] + 1.0);
    gaps[axis] = fmax(0.0, raw_gap - rounding_margin);
  }
}

__device__ double collisionLowerBound(
  const GpuCuboid & cuboid,
  const GpuBvhNode & node)
{
  double gaps[3];
  nodeAxisGaps(cuboid, node, gaps);
  return fmax(gaps[0], fmax(gaps[1], gaps[2]));
}

__device__ double clearanceLowerBound(
  const GpuCuboid & cuboid,
  const GpuBvhNode & node)
{
  double gaps[3];
  nodeAxisGaps(cuboid, node, gaps);
  return hypot(hypot(gaps[0], gaps[1]), gaps[2]);
}

__device__ bool collisionQuery(
  const GpuCuboid & cuboid,
  const GpuBvhNode * nodes,
  const std::size_t node_count,
  const std::int32_t * point_indices,
  const std::size_t point_index_count,
  const GpuPoint * points,
  const std::size_t point_count,
  unsigned long long & node_checks,
  unsigned long long & point_checks,
  unsigned int * overflow)
{
  if(node_count == 0 || point_count == 0){
    return false;
  }
  int stack[kTraversalStack];
  int top = 0;
  stack[top++] = 0;
  while(top > 0){
    const int index = stack[--top];
    if(index < 0 || static_cast<std::size_t>(index) >= node_count){
      atomicExch(overflow, 1u);
      return false;
    }
    const GpuBvhNode & node = nodes[index];
    ++node_checks;
    if(collisionLowerBound(cuboid, node) > kCuboidBoundaryTolerance){
      continue;
    }
    if(node.left < 0){
      for(std::uint32_t i = node.begin; i < node.end; ++i){
        if(i >= point_index_count){
          atomicExch(overflow, 1u);
          return false;
        }
        const std::int32_t point_index = point_indices[i];
        if(point_index < 0 ||
           static_cast<std::size_t>(point_index) >= point_count)
        {
          atomicExch(overflow, 1u);
          return false;
        }
        ++point_checks;
        double clearance = 0.0;
        if(cuboidCollision(cuboid, points[point_index], clearance)){
          return true;
        }
      }
      continue;
    }
    if(node.right < 0 || top + 2 > kTraversalStack){
      atomicExch(overflow, 1u);
      return false;
    }
    stack[top++] = node.right;
    stack[top++] = node.left;
  }
  return false;
}

__device__ bool clearanceQuery(
  const GpuCuboid & cuboid,
  const GpuBvhNode * nodes,
  const std::size_t node_count,
  const std::int32_t * point_indices,
  const std::size_t point_index_count,
  const GpuPoint * points,
  const std::size_t point_count,
  const double inflation_radius,
  double & result_clearance,
  unsigned long long & node_checks,
  unsigned long long & point_checks,
  unsigned int * overflow)
{
  if(node_count == 0 || point_count == 0){
    result_clearance = CUDART_INF;
    return false;
  }
  result_clearance = inflation_radius > 0.0 ?
    inflation_radius : kCuboidBoundaryTolerance;
  int stack[kTraversalStack];
  int top = 0;
  stack[top++] = 0;
  while(top > 0){
    const int index = stack[--top];
    if(index < 0 || static_cast<std::size_t>(index) >= node_count){
      atomicExch(overflow, 1u);
      return false;
    }
    const GpuBvhNode & node = nodes[index];
    ++node_checks;
    const double prune_tolerance =
      1e-12 * fmax(1.0, fabs(result_clearance));
    if(clearanceLowerBound(cuboid, node) >=
       result_clearance - prune_tolerance)
    {
      continue;
    }
    if(node.left < 0){
      for(std::uint32_t i = node.begin; i < node.end; ++i){
        if(i >= point_index_count){
          atomicExch(overflow, 1u);
          return false;
        }
        const std::int32_t point_index = point_indices[i];
        if(point_index < 0 ||
           static_cast<std::size_t>(point_index) >= point_count)
        {
          atomicExch(overflow, 1u);
          return false;
        }
        ++point_checks;
        double clearance = 0.0;
        if(cuboidCollision(cuboid, points[point_index], clearance)){
          result_clearance = 0.0;
          return true;
        }
        result_clearance = fmin(result_clearance, clearance);
      }
      continue;
    }
    if(node.right < 0 || top + 2 > kTraversalStack){
      atomicExch(overflow, 1u);
      return false;
    }
    const GpuBvhNode & left = nodes[node.left];
    const GpuBvhNode & right = nodes[node.right];
    double left_bound = clearanceLowerBound(cuboid, left);
    double right_bound = clearanceLowerBound(cuboid, right);
    int first = node.left;
    int second = node.right;
    if(right_bound < left_bound){
      const double temporary = left_bound;
      left_bound = right_bound;
      right_bound = temporary;
      first = node.right;
      second = node.left;
    }
    // The CPU reference pushes only children below the current bound; the
    // pop-time check makes this equivalent while keeping stack depth bounded.
    if(right_bound < result_clearance){
      stack[top++] = second;
    }
    if(left_bound < result_clearance){
      stack[top++] = first;
    }
  }
  if(!isfinite(result_clearance)){
    result_clearance = inflation_radius;
  }
  if(result_clearance < 0.0){
    result_clearance = 0.0;
  }
  return false;
}

// Device mirror of queryMppiPlanarObstacleGrid. The collision pass scans cells
// in ascending y then x order and stops at the first point inside the oriented
// rectangle; the clearance pass repeats the same order while keeping the
// minimum point distance within the radius when the distance field is
// unavailable at that pose. A pass that does not run leaves its counters
// untouched, and early collision returns clearance 0.
__device__ void planarQuery(
  const MppiPlanarObstacleGridView & view,
  const MppiPlanarBodyRect & body,
  const double center_x,
  const double center_y,
  const MppiPlanarBodyTrig & trig,
  const double inflation_radius,
  bool & evaluated,
  bool & in_collision,
  double & clearance,
  unsigned long long & collision_cell_checks,
  unsigned long long & collision_point_checks,
  unsigned long long & clearance_cell_checks,
  unsigned long long & clearance_point_checks,
  unsigned int * overflow)
{
  // Exactly the host query's validity contract: an invalid body, non-finite
  // pose or non-finite precomputed trig leaves the pose UNKNOWN, never free.
  // The device receives cos/sin instead of yaw, so non-finite trig is the
  // equivalent check for a non-finite yaw (a NaN/infinite yaw produces NaN
  // trig under both libms).
  evaluated =
    view.valid && body.valid &&
    isfinite(body.x_min) && isfinite(body.x_max) &&
    isfinite(body.y_min) && isfinite(body.y_max) &&
    body.x_max >= body.x_min && body.y_max >= body.y_min &&
    isfinite(center_x) && isfinite(center_y) &&
    isfinite(trig.cos_yaw) && isfinite(trig.sin_yaw);
  in_collision = false;
  clearance = CUDART_INF;
  if(!evaluated || view.cells_x == 0 || view.cells_y == 0){
    return;
  }

  // The collision AABB carries the local classification tolerance because
  // inside() accepts points just outside an edge.
  const MppiPlanarAabb collision_aabb = mppiPlanarBodyAabb(
    body, center_x, center_y, trig, kMppiPlanarBoundaryTolerance, 0.0);
  MppiPlanarCellRange collision_range;
  if(mppiPlanarCellRangeForAabb(
      view, collision_aabb.minimum_x, collision_aabb.maximum_x,
      collision_aabb.minimum_y, collision_aabb.maximum_y,
      &collision_range))
  {
    for(std::size_t y = collision_range.y_begin;
      y <= collision_range.y_end; ++y)
    {
      for(std::size_t x = collision_range.x_begin;
        x <= collision_range.x_end; ++x)
      {
        ++collision_cell_checks;
        const std::size_t cell = y * view.cells_x + x;
        const std::uint32_t begin = __ldg(view.cell_offsets + cell);
        const std::uint32_t end = __ldg(view.cell_offsets + cell + 1);
        if(begin > end || static_cast<std::size_t>(end) > view.point_count){
          atomicExch(overflow, 1u);
          return;
        }
        for(std::uint32_t index = begin; index < end; ++index){
          ++collision_point_checks;
          // cudaMalloc ownership makes the XY array 16-byte aligned, so the
          // aligned vector load reads the same two doubles without changing
          // the geometric expression.
          const double2 point_xy =
            __ldg(reinterpret_cast<const double2 *>(view.points_xy) + index);
          if(mppiPlanarPointToBody(
              body, center_x, center_y, trig, point_xy.x,
              point_xy.y).inside())
          {
            in_collision = true;
            clearance = 0.0;
            return;
          }
        }
      }
    }
  }

  // Clearance is an optional refinement for a finite positive radius only.
  if(!isfinite(inflation_radius) || inflation_radius <= 0.0){
    return;
  }
  if(mppiPlanarFieldClearance(
      view.distance_field, body, center_x, center_y, trig,
      inflation_radius, &clearance))
  {
    return;
  }
  const MppiPlanarAabb clearance_aabb = mppiPlanarBodyAabb(
    body, center_x, center_y, trig, kMppiPlanarBoundaryTolerance,
    inflation_radius);
  MppiPlanarCellRange clearance_range;
  if(!mppiPlanarCellRangeForAabb(
      view, clearance_aabb.minimum_x, clearance_aabb.maximum_x,
      clearance_aabb.minimum_y, clearance_aabb.maximum_y,
      &clearance_range))
  {
    return;
  }
  double search_limit = fmax(inflation_radius, kMppiPlanarBoundaryTolerance);
  for(std::size_t y = clearance_range.y_begin;
    y <= clearance_range.y_end; ++y)
  {
    for(std::size_t x = clearance_range.x_begin;
      x <= clearance_range.x_end; ++x)
    {
      ++clearance_cell_checks;
      const std::size_t cell = y * view.cells_x + x;
      const std::uint32_t begin = __ldg(view.cell_offsets + cell);
      const std::uint32_t end = __ldg(view.cell_offsets + cell + 1);
      if(begin > end || static_cast<std::size_t>(end) > view.point_count){
        atomicExch(overflow, 1u);
        return;
      }
      for(std::uint32_t index = begin; index < end; ++index){
        ++clearance_point_checks;
        const double2 point_xy =
          __ldg(reinterpret_cast<const double2 *>(view.points_xy) + index);
        const auto distance = mppiPlanarPointToBody(
          body, center_x, center_y, trig, point_xy.x,
          point_xy.y);
        if(distance.dx > search_limit || distance.dy > search_limit){
          continue;
        }
        const double point_clearance = distance.clearance();
        if(point_clearance <= inflation_radius && point_clearance < clearance){
          clearance = point_clearance;
          search_limit = fmax(clearance, kMppiPlanarBoundaryTolerance);
        }
      }
    }
  }
}

// The tilted-plane rollout is split into three phases so independent poses run
// in parallel. The pose phase owns the integration, the query phase owns one
// pose's collision + sampled clearance scan (same shared MPPI_PLANAR_HD
// primitives, same ascending cell/point order, same early stop), and the probe
// phase owns the terminal constant-curvature probe. The point scans match
// the CPU reference; positions and exact clearances pass
// through device vs host libm, so they agree to ulp-level differences (the
// differential tests hold x/y/yaw to 1e-12 and clearance to 1e-9, with exact
// counters).
//
// Read-only load of one pose entry through the read-only data cache. The
// struct is three consecutive doubles at 8-byte alignment, so scalar __ldg
// loads read exactly the bits a plain dereference would.
__device__ GpuPlanarPose loadPlanarPose(
  const GpuPlanarPose * poses, const std::size_t index)
{
  const double * values = reinterpret_cast<const double *>(poses) + 3 * index;
  return GpuPlanarPose{
    __ldg(values), __ldg(values + 1), __ldg(values + 2)};
}

// Phase 1: one thread per candidate. Rollout starts at the plane origin (the
// current base_link pose by construction) and uses the same midpoint Ackermann
// step as the CPU planar scorer. Only poses are produced here; no obstacle
// queries run.
__global__ void planarPosesKernel(
  const GpuConfig config,
  const MppiControl * controls,
  const std::size_t * control_starts,
  const std::size_t * control_counts,
  const std::size_t candidate_count,
  const std::size_t pose_stride,
  GpuPlanarPose * poses)
{
  const std::size_t candidate =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(candidate >= candidate_count){
    return;
  }
  const std::size_t control_start = control_starts[candidate];
  const std::size_t control_count = control_counts[candidate];
  const double dt = fmax(0.02, config.model_dt);
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  GpuPlanarPose * output = poses + candidate * pose_stride;
  for(std::size_t index = 0; index < control_count; ++index){
    const MppiControl control = controls[control_start + index];
    x += control.vx * cos(yaw + control.wz * dt * 0.5) * dt;
    y += control.vx * sin(yaw + control.wz * dt * 0.5) * dt;
    yaw = deviceNormalizeAngle(yaw + control.wz * dt);
    output[index] = GpuPlanarPose{x, y, yaw};
  }
}

// Phase 2: one thread per (candidate, step) pair. Reads the pose, runs the
// collision scan (every point) and the sampled clearance scan (index % step
// == 0 or the last point) through the exact planarQuery mirror, and writes the
// whole GpuTrajectoryPoint including all four counters. The per-point
// in_collision flag left here is what the probe phase ORs back into the
// time-channel collision decision.
__global__ void planarQueryPosesKernel(
  const GpuConfig config,
  const MppiPlanarBodyRect body,
  const MppiPlanarObstacleGridView view,
  const std::size_t * control_counts,
  const std::size_t candidate_count,
  const std::size_t pose_stride,
  const std::size_t pose_count,
  const GpuPlanarPose * poses,
  GpuTrajectoryPoint * trajectory_points,
  const std::size_t * point_starts,
  unsigned int * overflow)
{
  const std::size_t thread =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(thread >= pose_count){
    return;
  }
  // pose_stride is the per-candidate row length, so thread maps one-to-one to
  // poses[candidate][index]; rows shorter than the stride are skipped.
  const std::size_t candidate = thread / pose_stride;
  const std::size_t index = thread - candidate * pose_stride;
  if(candidate >= candidate_count || index >= control_counts[candidate]){
    return;
  }
  const std::size_t clearance_step = config.check_every_obstacle_point ? 1 :
    (config.trajectory_point_step > 0 ? config.trajectory_point_step : 1);
  const bool evaluate_clearance =
    index % clearance_step == 0 || index + 1 == control_counts[candidate];
  const GpuPlanarPose pose = loadPlanarPose(poses, thread);
  const MppiPlanarBodyTrig trig = mppiPlanarBodyTrig(pose.yaw);
  bool evaluated = false;
  bool in_collision = false;
  double obstacle_clearance = CUDART_INF;
  unsigned long long collision_cell_checks = 0;
  unsigned long long collision_point_checks = 0;
  unsigned long long clearance_cell_checks = 0;
  unsigned long long clearance_point_checks = 0;
  planarQuery(
    view, body, pose.x, pose.y, trig,
    evaluate_clearance ? config.inflation_radius : -1.0,
    evaluated, in_collision, obstacle_clearance,
    collision_cell_checks, collision_point_checks,
    clearance_cell_checks, clearance_point_checks, overflow);
  GpuTrajectoryPoint & output =
    trajectory_points[point_starts[candidate] + index];
  output.x = pose.x;
  output.y = pose.y;
  output.yaw = pose.yaw;
  output.obstacle_clearance =
    evaluate_clearance ? obstacle_clearance : CUDART_INF;
  output.in_collision = in_collision ? 1u : 0u;
  output.collision_evaluated = evaluated ? 1u : 0u;
  output.collision_node_checks = collision_cell_checks;
  output.collision_point_checks = collision_point_checks;
  output.clearance_node_checks = clearance_cell_checks;
  output.clearance_point_checks = clearance_point_checks;
}

// Phase 3: one thread per candidate. Recomputes time_channel_collided by
// OR-ing the step flags written by phase 2, then, only when the time rollout
// never collided and the probe is enabled, integrates and queries the terminal
// constant-curvature probe exactly as the single-kernel rollout did. The probe
// stays collision-only and stops at the first contact.
__global__ void planarProbeKernel(
  const GpuConfig config,
  const MppiPlanarBodyRect body,
  const MppiPlanarObstacleGridView view,
  const MppiControl * controls,
  const std::size_t * control_starts,
  const std::size_t * control_counts,
  const std::size_t candidate_count,
  const std::size_t pose_stride,
  const GpuPlanarPose * poses,
  const GpuTrajectoryPoint * trajectory_points,
  const std::size_t * point_starts,
  GpuTrajectoryPoint * probe_points,
  const std::size_t * probe_starts,
  const std::size_t probe_capacity,
  std::size_t * probe_counts,
  double * probe_first_block_m,
  unsigned int * overflow)
{
  const std::size_t candidate =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(candidate >= candidate_count){
    return;
  }
  probe_counts[candidate] = 0;
  probe_first_block_m[candidate] = -1.0;
  // The single-kernel rollout zeroed the probe outputs and bailed out as soon
  // as any traversal reported malformed grid state; the batch then fails and
  // the host discards the output. Mirror that bail-out here.
  if(*overflow != 0u){
    return;
  }
  const std::size_t control_start = control_starts[candidate];
  const std::size_t control_count = control_counts[candidate];
  if(control_count == 0 ||
     !(config.evaluation_distance > 1e-3) ||
     !(config.evaluation_step > 1e-3) || view.point_count == 0 ||
     probe_capacity == 0)
  {
    return;
  }
  bool time_channel_collided = false;
  const std::size_t output_start = point_starts[candidate];
  for(std::size_t index = 0; index < control_count; ++index){
    if(trajectory_points[output_start + index].in_collision != 0u){
      time_channel_collided = true;
      break;
    }
  }
  if(time_channel_collided){
    return;
  }
  const GpuPlanarPose terminal_pose = loadPlanarPose(
    poses, candidate * pose_stride + control_count - 1);
  double probe_x = terminal_pose.x;
  double probe_y = terminal_pose.y;
  double probe_yaw = terminal_pose.yaw;
  const MppiControl terminal = controls[control_start + control_count - 1];
  const double direction = terminal.vx >= 0.0 ? 1.0 : -1.0;
  const double curvature =
    fabs(terminal.vx) > 1e-6 ? terminal.wz / terminal.vx : 0.0;
  const double step = fmax(0.02, config.evaluation_step);
  const std::size_t limit = config.evaluation_max_points > 0 ?
    config.evaluation_max_points : 1;
  double arc = 0.0;
  std::size_t probe_index = probe_starts[candidate];
  std::size_t produced = 0;
  while(arc < config.evaluation_distance && produced < limit &&
    produced < probe_capacity)
  {
    const double next_arc = fmin(arc + step, config.evaluation_distance);
    const double ds_signed = direction * (next_arc - arc);
    arc = next_arc;
    const double dtheta = curvature * ds_signed;
    if(fabs(curvature) > 1e-9){
      probe_x += (sin(probe_yaw + dtheta) - sin(probe_yaw)) / curvature;
      probe_y -= (cos(probe_yaw + dtheta) - cos(probe_yaw)) / curvature;
    }
    else{
      probe_x += ds_signed * cos(probe_yaw);
      probe_y += ds_signed * sin(probe_yaw);
    }
    probe_yaw = deviceNormalizeAngle(probe_yaw + dtheta);

    const MppiPlanarBodyTrig trig = mppiPlanarBodyTrig(probe_yaw);
    bool evaluated = false;
    bool in_collision = false;
    double unused_clearance = CUDART_INF;
    unsigned long long collision_cell_checks = 0;
    unsigned long long collision_point_checks = 0;
    unsigned long long clearance_cell_checks = 0;
    unsigned long long clearance_point_checks = 0;
    planarQuery(
      view, body, probe_x, probe_y, trig, -1.0,
      evaluated, in_collision, unused_clearance,
      collision_cell_checks, collision_point_checks,
      clearance_cell_checks, clearance_point_checks, overflow);
    GpuTrajectoryPoint & output = probe_points[probe_index++];
    output.x = probe_x;
    output.y = probe_y;
    output.yaw = probe_yaw;
    output.obstacle_clearance = CUDART_INF;
    output.in_collision = in_collision ? 1u : 0u;
    output.collision_evaluated = evaluated ? 1u : 0u;
    output.collision_node_checks = collision_cell_checks;
    output.collision_point_checks = collision_point_checks;
    output.clearance_node_checks = 0;
    output.clearance_point_checks = 0;
    ++produced;
    probe_counts[candidate] = produced;
    if(in_collision){
      probe_first_block_m[candidate] = arc;
      break;
    }
    if(*overflow != 0u){
      return;
    }
  }
}

__global__ void rolloutKernel(
  const GpuConfig config,
  const GpuPose initial_pose,
  const GpuPoint * local_vertices,
  const std::size_t local_vertex_count,
  const GpuBvhNode * nodes,
  const std::size_t node_count,
  const std::int32_t * point_indices,
  const std::size_t point_index_count,
  const GpuPoint * points,
  const std::size_t point_count,
  const MppiControl * controls,
  const std::size_t * control_starts,
  const std::size_t * control_counts,
  const std::size_t candidate_count,
  GpuTrajectoryPoint * trajectory_points,
  const std::size_t * point_starts,
  GpuTrajectoryPoint * probe_points,
  const std::size_t * probe_starts,
  const std::size_t probe_capacity,
  std::size_t * probe_counts,
  double * probe_first_block_m,
  unsigned int * overflow)
{
  const std::size_t candidate =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(candidate >= candidate_count){
    return;
  }
  const std::size_t control_start = control_starts[candidate];
  const std::size_t control_count = control_counts[candidate];
  double x = initial_pose.x;
  double y = initial_pose.y;
  const double z = initial_pose.z;
  double yaw = initial_pose.yaw;
  const double dt = fmax(0.02, config.model_dt);
  const std::size_t clearance_step = config.check_every_obstacle_point ? 1 :
    (config.trajectory_point_step > 0 ? config.trajectory_point_step : 1);
  bool time_channel_collided = false;
  std::size_t output_index = point_starts[candidate];
  for(std::size_t index = 0; index < control_count; ++index){
    const MppiControl control = controls[control_start + index];
    x += control.vx * cos(yaw + control.wz * dt * 0.5) * dt;
    y += control.vx * sin(yaw + control.wz * dt * 0.5) * dt;
    yaw = deviceNormalizeAngle(yaw + control.wz * dt);

    GpuCuboid cuboid;
    const bool geometry_valid = transformAndBuildCuboid(
      local_vertices, local_vertex_count, GpuPose{x, y, z, yaw}, cuboid);
    unsigned long long collision_node_checks = 0;
    unsigned long long collision_point_checks = 0;
    unsigned long long clearance_node_checks = 0;
    unsigned long long clearance_point_checks = 0;
    bool in_collision = false;
    const bool evaluated = geometry_valid;
    if(geometry_valid){
      in_collision = collisionQuery(
        cuboid, nodes, node_count, point_indices, point_index_count, points,
        point_count, collision_node_checks, collision_point_checks, overflow);
    }
    if(in_collision){
      time_channel_collided = true;
    }
    double obstacle_clearance = CUDART_INF;
    const bool evaluate_clearance =
      index % clearance_step == 0 || index + 1 == control_count;
    if(evaluate_clearance){
      if(in_collision){
        obstacle_clearance = 0.0;
      }
      else if(geometry_valid && point_count > 0 && node_count > 0){
        clearanceQuery(
          cuboid, nodes, node_count, point_indices, point_index_count,
          points, point_count, config.inflation_radius, obstacle_clearance,
          clearance_node_checks, clearance_point_checks, overflow);
      }
    }
    GpuTrajectoryPoint & output = trajectory_points[output_index++];
    output.x = x;
    output.y = y;
    output.yaw = yaw;
    output.obstacle_clearance = obstacle_clearance;
    output.in_collision = in_collision ? 1u : 0u;
    output.collision_evaluated = evaluated ? 1u : 0u;
    output.collision_node_checks = collision_node_checks;
    output.collision_point_checks = collision_point_checks;
    output.clearance_node_checks = clearance_node_checks;
    output.clearance_point_checks = clearance_point_checks;
    if(*overflow != 0u){
      probe_counts[candidate] = 0;
      probe_first_block_m[candidate] = -1.0;
      return;
    }
  }

  probe_counts[candidate] = 0;
  probe_first_block_m[candidate] = -1.0;
  if(control_count == 0 || time_channel_collided ||
     !(config.evaluation_distance > 1e-3) ||
     !(config.evaluation_step > 1e-3) || point_count == 0 ||
     probe_capacity == 0)
  {
    return;
  }
  double probe_x = x;
  double probe_y = y;
  double probe_yaw = yaw;
  const MppiControl terminal = controls[control_start + control_count - 1];
  const double direction = terminal.vx >= 0.0 ? 1.0 : -1.0;
  const double curvature =
    fabs(terminal.vx) > 1e-6 ? terminal.wz / terminal.vx : 0.0;
  const double step = fmax(0.02, config.evaluation_step);
  const std::size_t limit = config.evaluation_max_points > 0 ?
    config.evaluation_max_points : 1;
  double arc = 0.0;
  std::size_t probe_index = probe_starts[candidate];
  std::size_t produced = 0;
  while(arc < config.evaluation_distance && produced < limit &&
    produced < probe_capacity)
  {
    const double next_arc = fmin(arc + step, config.evaluation_distance);
    const double ds_signed = direction * (next_arc - arc);
    arc = next_arc;
    const double dtheta = curvature * ds_signed;
    if(fabs(curvature) > 1e-9){
      probe_x += (sin(probe_yaw + dtheta) - sin(probe_yaw)) / curvature;
      probe_y -= (cos(probe_yaw + dtheta) - cos(probe_yaw)) / curvature;
    }
    else{
      probe_x += ds_signed * cos(probe_yaw);
      probe_y += ds_signed * sin(probe_yaw);
    }
    probe_yaw = deviceNormalizeAngle(probe_yaw + dtheta);

    GpuCuboid cuboid;
    const bool geometry_valid = transformAndBuildCuboid(
      local_vertices, local_vertex_count,
      GpuPose{probe_x, probe_y, z, probe_yaw}, cuboid);
    unsigned long long node_checks = 0;
    unsigned long long point_checks = 0;
    bool in_collision = false;
    if(geometry_valid){
      in_collision = collisionQuery(
        cuboid, nodes, node_count, point_indices, point_index_count, points,
        point_count, node_checks, point_checks, overflow);
    }
    GpuTrajectoryPoint & output = probe_points[probe_index++];
    output.x = probe_x;
    output.y = probe_y;
    output.yaw = probe_yaw;
    output.obstacle_clearance = CUDART_INF;
    output.in_collision = in_collision ? 1u : 0u;
    output.collision_evaluated = geometry_valid ? 1u : 0u;
    output.collision_node_checks = node_checks;
    output.collision_point_checks = point_checks;
    output.clearance_node_checks = 0;
    output.clearance_point_checks = 0;
    ++produced;
    probe_counts[candidate] = produced;
    if(in_collision){
      probe_first_block_m[candidate] = arc;
      break;
    }
    if(*overflow != 0u){
      return;
    }
  }
}

MppiTrajectoryPoint toTrajectoryPoint(const GpuTrajectoryPoint & point)
{
  MppiTrajectoryPoint output;
  output.x = point.x;
  output.y = point.y;
  output.yaw = point.yaw;
  output.in_collision = point.in_collision != 0u;
  output.obstacle_clearance = point.obstacle_clearance;
  output.collision_evaluated = point.collision_evaluated != 0u;
  output.collision_node_checks =
    static_cast<std::size_t>(point.collision_node_checks);
  output.collision_point_checks =
    static_cast<std::size_t>(point.collision_point_checks);
  output.clearance_node_checks =
    static_cast<std::size_t>(point.clearance_node_checks);
  output.clearance_point_checks =
    static_cast<std::size_t>(point.clearance_point_checks);
  return output;
}

class MppiCudaObstaclesImpl final : public MppiCudaObstacles
{
public:
  MppiCudaObstaclesImpl()
  {
    int device_count = 0;
    if(cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0){
      cudaGetLastError();
      return;
    }
    // Match mppi_cuda_critics.cu: pin the runtime to device 0 once, before
    // querying its properties or allocating, so available() and every later
    // upload/rollout operate on the same device.
    if(cudaSetDevice(0) != cudaSuccess){
      cudaGetLastError();
      return;
    }
    cudaDeviceProp properties{};
    if(cudaGetDeviceProperties(&properties, 0) != cudaSuccess){
      cudaGetLastError();
      return;
    }
    sm_count_ = std::max(1, properties.multiProcessorCount);
    device_name_ = properties.name;
  }

  ~MppiCudaObstaclesImpl() override
  {
    release();
  }

  bool available(std::string * reason) const override
  {
    int device_count = 0;
    const cudaError_t status = cudaGetDeviceCount(&device_count);
    if(status != cudaSuccess || device_count <= 0){
      if(reason != nullptr){
        *reason = status != cudaSuccess ?
          std::string("cudaGetDeviceCount failed: ") +
          cudaGetErrorString(status) : "no CUDA device is present";
      }
      return false;
    }
    cudaDeviceProp properties{};
    const cudaError_t property_status = cudaGetDeviceProperties(&properties, 0);
    if(property_status != cudaSuccess){
      if(reason != nullptr){
        *reason = std::string("cudaGetDeviceProperties failed: ") +
          cudaGetErrorString(property_status);
      }
      return false;
    }
    if(reason != nullptr){
      std::ostringstream message;
      message << properties.name << " (compute capability " <<
        properties.major << "." << properties.minor << ", " <<
        properties.multiProcessorCount << " SMs)";
      *reason = message.str();
    }
    return true;
  }

  bool upload(
    const MppiObstacleSnapshot & snapshot,
    std::string & error) override
  {
    error.clear();
    uploaded_ = false;
    if(!available(&error)){
      error = "CUDA obstacle backend unavailable: " + error;
      return false;
    }
    if(!snapshot.cloud){
      error = "obstacle snapshot has no cloud";
      return false;
    }
    MppiObstacleBvhExport view;
    if(!exportMppiObstacleBvh(snapshot, &view, &error)){
      error = "cannot export obstacle BVH: " + error;
      return false;
    }
    if(snapshot.cloud->points.size() != view.point_indices.size()){
      error = "obstacle BVH point_indices are not a permutation of the finite cloud";
      return false;
    }
    if(!snapshot.cloud->points.empty() && view.nodes.empty()){
      error = "obstacle point cloud is non-empty but its BVH is empty";
      return false;
    }
    for(const auto & point : snapshot.cloud->points){
      if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
         !std::isfinite(point.z))
      {
        error = "obstacle snapshot cloud contains a non-finite point";
        return false;
      }
    }
    for(const auto & node : view.nodes){
      if(node.begin > node.end || node.end > view.point_indices.size()){
        error = "obstacle BVH node has an invalid point range";
        return false;
      }
      if(node.left < 0){
        if(node.right != -1){
          error = "obstacle BVH leaf has an unexpected right child";
          return false;
        }
      }
      else if(node.right < 0 ||
        static_cast<std::size_t>(node.left) >= view.nodes.size() ||
        static_cast<std::size_t>(node.right) >= view.nodes.size())
      {
        error = "obstacle BVH node has an invalid child index";
        return false;
      }
    }
    for(const std::int32_t index : view.point_indices){
      if(index < 0 ||
         static_cast<std::size_t>(index) >= snapshot.cloud->points.size())
      {
        error = "obstacle BVH point index is out of range";
        return false;
      }
    }
    if(!view.nodes.empty()){
      std::vector<std::pair<int, int>> stack{{0, 0}};
      int maximum_depth = 0;
      while(!stack.empty()){
        const auto [node_index, depth] = stack.back();
        stack.pop_back();
        maximum_depth = std::max(maximum_depth, depth);
        if(depth >= kTraversalStack - 1){
          error = "obstacle BVH depth exceeds the explicit device stack capacity";
          return false;
        }
        const auto & node = view.nodes[static_cast<std::size_t>(node_index)];
        if(node.left >= 0){
          stack.emplace_back(node.left, depth + 1);
          stack.emplace_back(node.right, depth + 1);
        }
      }
    }

    const std::size_t point_count = snapshot.cloud->points.size();
    const std::size_t node_count = view.nodes.size();
    const std::size_t index_count = view.point_indices.size();
    if(!ensureCapacity(points_, point_capacity_, point_count, error) ||
       !ensureCapacity(nodes_, node_capacity_, node_count, error) ||
       !ensureCapacity(point_indices_, point_index_capacity_, index_count, error))
    {
      return false;
    }
    std::vector<GpuPoint> points(point_count);
    for(std::size_t i = 0; i < point_count; ++i){
      const auto & source = snapshot.cloud->points[i];
      points[i] = GpuPoint{source.x, source.y, source.z};
    }
    std::vector<GpuBvhNode> nodes(node_count);
    for(std::size_t i = 0; i < node_count; ++i){
      const auto & source = view.nodes[i];
      GpuBvhNode & node = nodes[i];
      for(std::size_t axis = 0; axis < 3; ++axis){
        node.minimum[axis] = source.minimum[axis];
        node.maximum[axis] = source.maximum[axis];
      }
      node.begin = source.begin;
      node.end = source.end;
      node.left = source.left;
      node.right = source.right;
    }
    if(point_count > 0 && !checkCuda(cudaMemcpy(points_, points.data(),
        point_count * sizeof(GpuPoint), cudaMemcpyHostToDevice),
        "copy obstacle points", error))
    {
      return false;
    }
    if(node_count > 0 && !checkCuda(cudaMemcpy(nodes_, nodes.data(),
        node_count * sizeof(GpuBvhNode), cudaMemcpyHostToDevice),
        "copy obstacle BVH nodes", error))
    {
      return false;
    }
    if(index_count > 0 && !checkCuda(cudaMemcpy(point_indices_,
        view.point_indices.data(), index_count * sizeof(std::int32_t),
        cudaMemcpyHostToDevice), "copy obstacle BVH point indices", error))
    {
      return false;
    }
    point_count_ = point_count;
    node_count_ = node_count;
    point_index_count_ = index_count;
    uploaded_ = true;
    return true;
  }

  bool rolloutBatch(
    const MppiCudaObstaclePose & initial_pose,
    const pcl::PointCloud<pcl::PointXYZ> & local_cuboid_vertices,
    const std::vector<MppiCandidate> & candidates,
    const MppiCudaObstacleConfig & config,
    std::vector<MppiScoredTrajectory> & output,
    std::string & error) override
  {
    error.clear();
    output.clear();
    if(!available(&error)){
      error = "CUDA obstacle backend unavailable: " + error;
      return false;
    }
    if(!uploaded_){
      error = "rolloutBatch called before a successful scene upload";
      return false;
    }
    if(local_cuboid_vertices.points.size() < 4 ||
       local_cuboid_vertices.points.size() > kMaxLocalVertices)
    {
      error = "local cuboid must contain between 4 and 64 vertices";
      return false;
    }
    if(!std::isfinite(initial_pose.x) || !std::isfinite(initial_pose.y) ||
       !std::isfinite(initial_pose.z) || !std::isfinite(initial_pose.yaw))
    {
      error = "initial pose contains a non-finite value";
      return false;
    }
    if(!std::isfinite(config.model_dt) ||
       !std::isfinite(config.inflation_radius) ||
       config.inflation_radius < 0.0 ||
       !std::isfinite(config.evaluation_distance) ||
       !std::isfinite(config.evaluation_step))
    {
      error = "obstacle rollout config contains an invalid scalar";
      return false;
    }
    if(candidates.empty()){
      return true;
    }

    std::vector<GpuPoint> local(local_cuboid_vertices.points.size());
    for(std::size_t i = 0; i < local.size(); ++i){
      const auto & source = local_cuboid_vertices.points[i];
      if(!std::isfinite(source.x) || !std::isfinite(source.y) ||
         !std::isfinite(source.z))
      {
        error = "local cuboid contains a non-finite vertex";
        return false;
      }
      local[i] = GpuPoint{source.x, source.y, source.z};
    }

    std::vector<MppiControl> flat_controls;
    std::vector<std::size_t> control_starts(candidates.size());
    std::vector<std::size_t> control_counts(candidates.size());
    std::vector<std::size_t> point_starts(candidates.size());
    std::size_t total_points = 0;
    for(std::size_t i = 0; i < candidates.size(); ++i){
      control_starts[i] = flat_controls.size();
      control_counts[i] = candidates[i].controls.size();
      point_starts[i] = total_points;
      if(control_counts[i] > std::numeric_limits<std::size_t>::max() -
         total_points)
      {
        error = "trajectory point count overflow";
        return false;
      }
      for(const auto & control : candidates[i].controls){
        if(!std::isfinite(control.vx) || !std::isfinite(control.wz)){
          error = "candidate control contains a non-finite value";
          return false;
        }
        flat_controls.push_back(control);
      }
      total_points += control_counts[i];
    }

    const bool probe_enabled =
      point_count_ > 0 && config.evaluation_distance > 1e-3 &&
      config.evaluation_step > 1e-3;
    std::size_t per_candidate_probe = 0;
    if(probe_enabled){
      const double effective_step = std::max(0.02, config.evaluation_step);
      const double raw_count =
        std::ceil(config.evaluation_distance / effective_step);
      if(!std::isfinite(raw_count) || raw_count > 10000000.0){
        error = "distance probe would require an unsafe number of points";
        return false;
      }
      const std::size_t arc_count = std::max<std::size_t>(
        1, static_cast<std::size_t>(raw_count));
      const std::size_t limit = config.evaluation_max_points > 0 ?
        config.evaluation_max_points : 1;
      // The +1 slot covers one extra floating-point iteration of the arc loop:
      // the accumulated arc can land a rounding step below evaluation_distance
      // after ceil(distance / step) samples, so the loop may produce one more
      // point. Without this slot a future simplification of the bound check
      // could truncate probe points.
      per_candidate_probe = std::min(arc_count + 1, limit);
    }
    std::vector<std::size_t> probe_starts(candidates.size());
    std::size_t total_probes = 0;
    for(std::size_t i = 0; i < candidates.size(); ++i){
      probe_starts[i] = total_probes;
      if(control_counts[i] == 0 || !probe_enabled){
        continue;
      }
      if(per_candidate_probe > std::numeric_limits<std::size_t>::max() -
         total_probes)
      {
        error = "probe point count overflow";
        return false;
      }
      total_probes += per_candidate_probe;
    }

    if(!ensureCapacity(local_vertices_, local_vertex_capacity_, local.size(), error) ||
       !ensureCapacity(controls_, control_capacity_, flat_controls.size(), error) ||
       !ensureCapacity(control_starts_, control_start_capacity_, control_starts.size(), error) ||
       !ensureCapacity(control_counts_, control_count_capacity_, control_counts.size(), error) ||
       !ensureCapacity(trajectory_points_, trajectory_capacity_, total_points, error) ||
       !ensureCapacity(point_starts_, point_start_capacity_, point_starts.size(), error) ||
       !ensureCapacity(probe_points_, probe_capacity_, total_probes, error) ||
       !ensureCapacity(probe_starts_, probe_start_capacity_, probe_starts.size(), error) ||
       !ensureCapacity(probe_counts_, probe_count_capacity_, candidates.size(), error) ||
       !ensureCapacity(probe_first_block_, probe_first_capacity_, candidates.size(), error) ||
       !ensureCapacity(overflow_, overflow_capacity_, 1, error))
    {
      return false;
    }
    if(!checkCuda(cudaMemcpy(local_vertices_, local.data(),
        local.size() * sizeof(GpuPoint), cudaMemcpyHostToDevice),
        "copy local cuboid vertices", error) ||
       (!flat_controls.empty() && !checkCuda(cudaMemcpy(controls_,
        flat_controls.data(), flat_controls.size() * sizeof(MppiControl),
        cudaMemcpyHostToDevice), "copy candidate controls", error)) ||
       !checkCuda(cudaMemcpy(control_starts_, control_starts.data(),
        control_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy candidate control starts", error) ||
       !checkCuda(cudaMemcpy(control_counts_, control_counts.data(),
        control_counts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy candidate control counts", error) ||
       !checkCuda(cudaMemcpy(point_starts_, point_starts.data(),
        point_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy trajectory point starts", error) ||
       !checkCuda(cudaMemcpy(probe_starts_, probe_starts.data(),
        probe_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy probe starts", error))
    {
      return false;
    }
    if(!checkCuda(cudaMemset(overflow_, 0, sizeof(unsigned int)),
        "clear BVH overflow flag", error) ||
       !checkCuda(cudaMemset(probe_counts_, 0,
        candidates.size() * sizeof(std::size_t)), "clear probe counts", error))
    {
      return false;
    }

    GpuConfig gpu_config;
    gpu_config.model_dt = config.model_dt;
    gpu_config.inflation_radius = config.inflation_radius;
    gpu_config.evaluation_distance = config.evaluation_distance;
    gpu_config.evaluation_step = config.evaluation_step;
    gpu_config.trajectory_point_step = config.trajectory_point_step;
    gpu_config.evaluation_max_points = config.evaluation_max_points;
    gpu_config.check_every_obstacle_point =
      config.check_every_obstacle_point ? 1 : 0;
    const GpuPose gpu_pose{
      initial_pose.x, initial_pose.y, initial_pose.z, initial_pose.yaw};
    const MppiCudaLaunchGeometry launch =
      mppiCudaSelectLaunch(candidates.size(), sm_count_);
    rolloutKernel<<<launch.grid, launch.block_dim>>>(
      gpu_config, gpu_pose, local_vertices_, local.size(), nodes_, node_count_,
      point_indices_, point_index_count_, points_, point_count_,
      controls_, control_starts_, control_counts_, candidates.size(),
      trajectory_points_, point_starts_, probe_points_, probe_starts_,
      per_candidate_probe, probe_counts_, probe_first_block_, overflow_);
    if(!checkCuda(cudaGetLastError(), "mppi obstacle rollout kernel launch", error) ||
       !checkCuda(cudaDeviceSynchronize(), "mppi obstacle rollout kernel", error))
    {
      return false;
    }
    unsigned int overflow = 0;
    if(!checkCuda(cudaMemcpy(&overflow, overflow_, sizeof(unsigned int),
        cudaMemcpyDeviceToHost), "copy BVH overflow flag", error))
    {
      return false;
    }
    if(overflow != 0u){
      error = "obstacle BVH traversal overflowed or observed a malformed node";
      return false;
    }

    std::vector<GpuTrajectoryPoint> host_points(total_points);
    std::vector<GpuTrajectoryPoint> host_probes(total_probes);
    std::vector<std::size_t> host_probe_counts(candidates.size());
    std::vector<double> host_first_block(candidates.size());
    if(total_points > 0 && !checkCuda(cudaMemcpy(host_points.data(),
        trajectory_points_, total_points * sizeof(GpuTrajectoryPoint),
        cudaMemcpyDeviceToHost), "copy trajectory points", error))
    {
      return false;
    }
    if(total_probes > 0 && !checkCuda(cudaMemcpy(host_probes.data(),
        probe_points_, total_probes * sizeof(GpuTrajectoryPoint),
        cudaMemcpyDeviceToHost), "copy probe points", error))
    {
      return false;
    }
    if(!checkCuda(cudaMemcpy(host_probe_counts.data(), probe_counts_,
        candidates.size() * sizeof(std::size_t), cudaMemcpyDeviceToHost),
        "copy probe counts", error) ||
       !checkCuda(cudaMemcpy(host_first_block.data(), probe_first_block_,
        candidates.size() * sizeof(double), cudaMemcpyDeviceToHost),
        "copy probe first block distances", error))
    {
      return false;
    }

    output.resize(candidates.size());
    for(std::size_t i = 0; i < candidates.size(); ++i){
      output[i].controls = candidates[i].controls;
      output[i].probe_first_block_m = host_first_block[i];
      output[i].points.reserve(control_counts[i]);
      for(std::size_t j = 0; j < control_counts[i]; ++j){
        output[i].points.push_back(toTrajectoryPoint(
          host_points[point_starts[i] + j]));
      }
      output[i].probe_points.reserve(host_probe_counts[i]);
      for(std::size_t j = 0; j < host_probe_counts[i]; ++j){
        output[i].probe_points.push_back(toTrajectoryPoint(
          host_probes[probe_starts[i] + j]));
      }
    }
    return true;
  }

  bool uploadPlanar(
    const MppiPlanarObstacleGrid & grid,
    std::string & error) override
  {
    error.clear();
    planar_uploaded_ = false;
    planar_distance_field_ = {};
    // A rejected upload must not leave a stale scene size in the diagnostics;
    // 0 (and 0 ms) means the last call completed no device upload, while -1 is
    // reserved for "never called".
    planar_stats_.upload_ms = 0.0;
    planar_stats_.grid_points = 0;
    planar_stats_.grid_cells = 0;
    if(!available(&error)){
      error = "CUDA obstacle backend unavailable: " + error;
      return false;
    }
    std::string grid_detail;
    if(planarGridUploadRejected(grid, grid_detail)){
      error = "planar obstacle grid upload rejected: " + grid_detail;
      return false;
    }
    if(!ensureCapacity(
        planar_offsets_, planar_offset_capacity_,
        grid.cell_offsets.size(), error) ||
       !ensureCapacity(
        planar_points_xy_, planar_points_xy_capacity_,
        grid.points_xy.size(), error))
    {
      return false;
    }

    const auto upload_start = std::chrono::steady_clock::now();
    if(!grid.cell_offsets.empty() && !checkCuda(cudaMemcpy(
        planar_offsets_, grid.cell_offsets.data(),
        grid.cell_offsets.size() * sizeof(std::uint32_t),
        cudaMemcpyHostToDevice), "copy planar obstacle cell offsets", error))
    {
      return false;
    }
    if(!grid.points_xy.empty() && !checkCuda(cudaMemcpy(
        planar_points_xy_, grid.points_xy.data(),
        grid.points_xy.size() * sizeof(double), cudaMemcpyHostToDevice),
        "copy planar obstacle points", error))
    {
      return false;
    }
    const auto & field = grid.distance_field;
    if(!field.distances.empty()){
      if(field.width == 0 || field.distances.size() % field.width != 0 ||
         field.distances.size() / field.width != field.height ||
         !std::isfinite(field.origin_x) || !std::isfinite(field.origin_y) ||
         !std::isfinite(field.resolution) || field.resolution <= 0.0)
      {
        error = "invalid planar obstacle distance field";
        return false;
      }
      if(!ensureCapacity(
          planar_distance_values_, planar_distance_capacity_, field.distances.size(), error) ||
         !checkCuda(cudaMemcpy(
          planar_distance_values_, field.distances.data(),
          field.distances.size() * sizeof(float), cudaMemcpyHostToDevice),
          "copy planar obstacle distance field", error))
      {
        return false;
      }
      planar_distance_field_ = {
        planar_distance_values_, field.width, field.height, field.origin_x,
        field.origin_y, field.resolution};
    }
    planar_stats_.upload_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - upload_start).count();
    planar_stats_.grid_points = grid.point_count;
    planar_stats_.grid_cells = grid.cells_x * grid.cells_y;

    planar_cells_x_ = grid.cells_x;
    planar_cells_y_ = grid.cells_y;
    planar_point_count_ = grid.point_count;
    planar_cell_size_ = grid.cell_size;
    planar_origin_x_ = grid.origin_x;
    planar_origin_y_ = grid.origin_y;
    planar_uploaded_ = true;
    return true;
  }

  bool rolloutPlanarBatch(
    const MppiPlanarBodyRect & body,
    const std::vector<MppiCandidate> & candidates,
    const MppiCudaObstacleConfig & config,
    std::vector<MppiScoredTrajectory> & output,
    std::string & error,
    MppiCudaTrajectoryBatchView * device_batch) override
  {
    error.clear();
    output.clear();
    if(device_batch != nullptr){
      *device_batch = {};
    }
    const bool has_candidates = !candidates.empty();
    // Metadata-only (empty candidate) batches are used by the directed-retreat
    // path and must not wipe the last real batch timing. A non-empty call that
    // completes no device work reports 0; -1 is reserved for "never called".
    if(has_candidates){
      planar_stats_.rollout_ms = 0.0;
      planar_stats_.pose_kernel_ms = -1.0;
      planar_stats_.query_kernel_ms = -1.0;
      planar_stats_.probe_kernel_ms = -1.0;
      planar_stats_.readback_ms = -1.0;
    }
    if(!available(&error)){
      error = "CUDA obstacle backend unavailable: " + error;
      return false;
    }
    if(!planar_uploaded_){
      error = "rolloutPlanarBatch called before a successful planar scene upload";
      return false;
    }
    if(!body.valid || !std::isfinite(body.x_min) ||
       !std::isfinite(body.x_max) || !std::isfinite(body.y_min) ||
       !std::isfinite(body.y_max) || body.x_max < body.x_min ||
       body.y_max < body.y_min)
    {
      error = "planar body rectangle is invalid";
      return false;
    }
    // inflation_radius deliberately accepts any value, including negative,
    // NaN and infinity: the CPU contract uses every non-finite or
    // non-positive radius as collision-only, and the device query applies the
    // same rule. Only the scalars with no CPU fallback semantics are
    // validated here.
    if(!std::isfinite(config.model_dt) ||
       !std::isfinite(config.evaluation_distance) ||
       !std::isfinite(config.evaluation_step))
    {
      error = "planar obstacle rollout config contains an invalid scalar";
      return false;
    }
    if(!has_candidates){
      return true;
    }

    std::vector<MppiControl> flat_controls;
    std::vector<std::size_t> control_starts(candidates.size());
    std::vector<std::size_t> control_counts(candidates.size());
    std::vector<std::size_t> point_starts(candidates.size());
    std::size_t total_points = 0;
    std::size_t pose_stride = 0;
    for(std::size_t i = 0; i < candidates.size(); ++i){
      control_starts[i] = flat_controls.size();
      control_counts[i] = candidates[i].controls.size();
      point_starts[i] = total_points;
      if(control_counts[i] > std::numeric_limits<std::size_t>::max() -
         total_points)
      {
        error = "planar trajectory point count overflow";
        return false;
      }
      for(const auto & control : candidates[i].controls){
        if(!std::isfinite(control.vx) || !std::isfinite(control.wz)){
          error = "planar candidate control contains a non-finite value";
          return false;
        }
        flat_controls.push_back(control);
      }
      total_points += control_counts[i];
      pose_stride = std::max(pose_stride, control_counts[i]);
    }
    // The pose phase hands one row of pose_stride poses per candidate to the
    // query phase; a batch with no controls needs neither phase.
    std::size_t pose_count = 0;
    if(pose_stride != 0){
      if(candidates.size() > std::numeric_limits<std::size_t>::max() /
         pose_stride)
      {
        error = "planar pose buffer count overflow";
        return false;
      }
      pose_count = candidates.size() * pose_stride;
    }

    const bool probe_enabled =
      planar_point_count_ > 0 && config.evaluation_distance > 1e-3 &&
      config.evaluation_step > 1e-3;
    std::size_t per_candidate_probe = 0;
    if(probe_enabled){
      const double effective_step = std::max(0.02, config.evaluation_step);
      const double raw_count =
        std::ceil(config.evaluation_distance / effective_step);
      if(!std::isfinite(raw_count) || raw_count > 10000000.0){
        error = "planar distance probe would require an unsafe number of points";
        return false;
      }
      const std::size_t arc_count = std::max<std::size_t>(
        1, static_cast<std::size_t>(raw_count));
      const std::size_t limit = config.evaluation_max_points > 0 ?
        config.evaluation_max_points : 1;
      // The +1 slot covers one extra floating-point iteration of the arc loop:
      // the accumulated arc can land a rounding step below evaluation_distance
      // after ceil(distance / step) samples, so the loop may produce one more
      // point. Without this slot a future simplification of the bound check
      // could truncate probe points.
      per_candidate_probe = std::min(arc_count + 1, limit);
    }
    std::vector<std::size_t> probe_starts;
    std::size_t total_probes = 0;
    if(probe_enabled){
      probe_starts.resize(candidates.size());
      for(std::size_t i = 0; i < candidates.size(); ++i){
        probe_starts[i] = total_probes;
        if(control_counts[i] == 0){
          continue;
        }
        if(per_candidate_probe > std::numeric_limits<std::size_t>::max() -
           total_probes)
        {
          error = "planar probe point count overflow";
          return false;
        }
        total_probes += per_candidate_probe;
      }
    }
    const bool run_probes = total_probes > 0;

    if(!ensureCapacity(controls_, control_capacity_, flat_controls.size(), error) ||
       !ensureCapacity(control_starts_, control_start_capacity_, control_starts.size(), error) ||
       !ensureCapacity(control_counts_, control_count_capacity_, control_counts.size(), error) ||
       !ensureCapacity(planar_poses_, planar_pose_capacity_, pose_count, error) ||
       !ensureCapacity(trajectory_points_, trajectory_capacity_, total_points, error) ||
       !ensureCapacity(point_starts_, point_start_capacity_, point_starts.size(), error) ||
       !ensureCapacity(overflow_, overflow_capacity_, 1, error))
    {
      return false;
    }
    if(run_probes &&
       (!ensureCapacity(probe_points_, probe_capacity_, total_probes, error) ||
       !ensureCapacity(probe_starts_, probe_start_capacity_, probe_starts.size(), error) ||
       !ensureCapacity(probe_counts_, probe_count_capacity_, candidates.size(), error) ||
       !ensureCapacity(probe_first_block_, probe_first_capacity_, candidates.size(), error)))
    {
      return false;
    }
    if((!flat_controls.empty() && !checkCuda(cudaMemcpy(controls_,
        flat_controls.data(), flat_controls.size() * sizeof(MppiControl),
        cudaMemcpyHostToDevice), "copy planar candidate controls", error)) ||
       !checkCuda(cudaMemcpy(control_starts_, control_starts.data(),
        control_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy planar candidate control starts", error) ||
       !checkCuda(cudaMemcpy(control_counts_, control_counts.data(),
        control_counts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy planar candidate control counts", error) ||
       !checkCuda(cudaMemcpy(point_starts_, point_starts.data(),
        point_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy planar trajectory point starts", error) ||
       (run_probes && !checkCuda(cudaMemcpy(probe_starts_, probe_starts.data(),
        probe_starts.size() * sizeof(std::size_t), cudaMemcpyHostToDevice),
        "copy planar probe starts", error)))
    {
      return false;
    }
    // The probe kernel writes probe_counts_ for every candidate before any
    // early return, so no separate device clear is needed here.
    if(!checkCuda(cudaMemset(overflow_, 0, sizeof(unsigned int)),
        "clear planar overflow flag", error))
    {
      return false;
    }

    GpuConfig gpu_config;
    gpu_config.model_dt = config.model_dt;
    gpu_config.inflation_radius = config.inflation_radius;
    gpu_config.evaluation_distance = config.evaluation_distance;
    gpu_config.evaluation_step = config.evaluation_step;
    gpu_config.trajectory_point_step = config.trajectory_point_step;
    gpu_config.evaluation_max_points = config.evaluation_max_points;
    gpu_config.check_every_obstacle_point =
      config.check_every_obstacle_point ? 1 : 0;
    MppiPlanarObstacleGridView planar_view;
    planar_view.cell_offsets = planar_offsets_;
    planar_view.points_xy = planar_points_xy_;
    planar_view.cells_x = planar_cells_x_;
    planar_view.cells_y = planar_cells_y_;
    planar_view.point_count = planar_point_count_;
    planar_view.cell_size = planar_cell_size_;
    planar_view.origin_x = planar_origin_x_;
    planar_view.origin_y = planar_origin_y_;
    planar_view.distance_field = planar_distance_field_;
    // uploadPlanar() rejects an invalid grid, so any uploaded scene (including
    // a valid empty grid) is marked valid on the device view.
    planar_view.valid = true;
    const MppiCudaLaunchGeometry launch =
      mppiCudaSelectLaunch(candidates.size(), sm_count_);
    bool measure_kernels = pose_count > 0 && ensurePlanarTimingEvents();
    const auto record_stage = [&](const PlanarTimingEvent stage) {
      if(measure_kernels && cudaEventRecord(planar_timing_events_[stage]) != cudaSuccess){
        cudaGetLastError();
        measure_kernels = false;
      }
    };
    const auto rollout_start = std::chrono::steady_clock::now();
    // Phase split: poses (one thread per candidate), then obstacle queries
    // (one thread per candidate/step pair, the dominant clearance work), then
    // the terminal probe (one thread per candidate). The up-to-three launches
    // run back-to-back on the same stream, so phase N+1 observes phase N's
    // writes; a batch whose candidates all have zero controls has no poses and
    // skips the first two launches.
    if(pose_count > 0){
      record_stage(kRolloutStart);
      planarPosesKernel<<<launch.grid, launch.block_dim>>>(
        gpu_config, controls_, control_starts_, control_counts_,
        candidates.size(), pose_stride, planar_poses_);
      if(!checkCuda(cudaGetLastError(),
          "mppi planar pose kernel launch", error))
      {
        return false;
      }
      record_stage(kPosesDone);
      const MppiCudaLaunchGeometry query_launch =
        mppiCudaSelectLaunch(pose_count, sm_count_);
      planarQueryPosesKernel<<<query_launch.grid, query_launch.block_dim>>>(
        gpu_config, body, planar_view, control_counts_, candidates.size(),
        pose_stride, pose_count, planar_poses_, trajectory_points_,
        point_starts_, overflow_);
      if(!checkCuda(cudaGetLastError(),
          "mppi planar pose query kernel launch", error))
      {
        return false;
      }
      record_stage(kQueriesDone);
    }
    if(run_probes){
      planarProbeKernel<<<launch.grid, launch.block_dim>>>(
        gpu_config, body, planar_view, controls_, control_starts_,
        control_counts_, candidates.size(), pose_stride, planar_poses_,
        trajectory_points_, point_starts_, probe_points_, probe_starts_,
        per_candidate_probe, probe_counts_, probe_first_block_, overflow_);
      if(!checkCuda(cudaGetLastError(),
          "mppi planar probe kernel launch", error))
      {
        return false;
      }
      record_stage(kProbesDone);
    }
    if(!checkCuda(cudaDeviceSynchronize(),
        "mppi planar obstacle rollout kernels", error))
    {
      return false;
    }
    const auto readback_start = std::chrono::steady_clock::now();
    unsigned int overflow = 0;
    if(!checkCuda(cudaMemcpy(&overflow, overflow_, sizeof(unsigned int),
        cudaMemcpyDeviceToHost), "copy planar overflow flag", error))
    {
      return false;
    }
    if(overflow != 0u){
      error = "planar obstacle grid traversal observed a malformed cell range";
      return false;
    }

    std::vector<GpuTrajectoryPoint> host_points(total_points);
    std::vector<GpuTrajectoryPoint> host_probes(total_probes);
    std::vector<std::size_t> host_probe_counts(run_probes ? candidates.size() : 0);
    std::vector<double> host_first_block(run_probes ? candidates.size() : 0);
    if(total_points > 0 && !checkCuda(cudaMemcpy(host_points.data(),
        trajectory_points_, total_points * sizeof(GpuTrajectoryPoint),
        cudaMemcpyDeviceToHost), "copy planar trajectory points", error))
    {
      return false;
    }
    if(total_probes > 0 && !checkCuda(cudaMemcpy(host_probes.data(),
        probe_points_, total_probes * sizeof(GpuTrajectoryPoint),
        cudaMemcpyDeviceToHost), "copy planar probe points", error))
    {
      return false;
    }
    if(run_probes && (!checkCuda(cudaMemcpy(host_probe_counts.data(), probe_counts_,
        candidates.size() * sizeof(std::size_t), cudaMemcpyDeviceToHost),
        "copy planar probe counts", error) ||
       !checkCuda(cudaMemcpy(host_first_block.data(), probe_first_block_,
        candidates.size() * sizeof(double), cudaMemcpyDeviceToHost),
        "copy planar probe first block distances", error)))
    {
      return false;
    }
    const auto readback_end = std::chrono::steady_clock::now();
    planar_stats_.rollout_ms = std::chrono::duration<double, std::milli>(
      readback_end - rollout_start).count();
    planar_stats_.readback_ms = std::chrono::duration<double, std::milli>(
      readback_end - readback_start).count();
    if(pose_count == 0){
      planar_stats_.pose_kernel_ms = 0.0;
      planar_stats_.query_kernel_ms = 0.0;
    }
    if(!run_probes){
      planar_stats_.probe_kernel_ms = 0.0;
    }
    if(measure_kernels){
      // The existing synchronization above already completed these events.
      planar_stats_.pose_kernel_ms = planarElapsedMs(kRolloutStart, kPosesDone);
      planar_stats_.query_kernel_ms = planarElapsedMs(kPosesDone, kQueriesDone);
      if(run_probes){
        planar_stats_.probe_kernel_ms = planarElapsedMs(kQueriesDone, kProbesDone);
      }
    }

    output.resize(candidates.size());
    for(std::size_t i = 0; i < candidates.size(); ++i){
      output[i].controls = candidates[i].controls;
      output[i].points.reserve(control_counts[i]);
      for(std::size_t j = 0; j < control_counts[i]; ++j){
        output[i].points.push_back(toTrajectoryPoint(
          host_points[point_starts[i] + j]));
      }
      if(run_probes){
        output[i].probe_first_block_m = host_first_block[i];
        output[i].probe_points.reserve(host_probe_counts[i]);
        for(std::size_t j = 0; j < host_probe_counts[i]; ++j){
          output[i].probe_points.push_back(toTrajectoryPoint(
            host_probes[probe_starts[i] + j]));
        }
      }
    }
    if(device_batch != nullptr){
      *device_batch = MppiCudaTrajectoryBatchView{
        trajectory_points_, probe_points_, controls_, candidates.size(),
        total_points, flat_controls.size(), total_probes, per_candidate_probe};
    }
    return true;
  }

  MppiCudaPlanarStats lastPlanarStats() const override
  {
    return planar_stats_;
  }

private:
  enum PlanarTimingEvent : std::size_t
  {
    kRolloutStart,
    kPosesDone,
    kQueriesDone,
    kProbesDone,
    kPlanarTimingEventCount
  };

  bool ensurePlanarTimingEvents()
  {
    if(!planar_timing_attempted_){
      planar_timing_attempted_ = true;
      for(auto & event : planar_timing_events_){
        cudaEvent_t handle = nullptr;
        if(cudaEventCreate(&handle) != cudaSuccess){
          cudaGetLastError();
          break;
        }
        event = handle;
      }
    }
    return planar_timing_events_.back() != nullptr;
  }

  double planarElapsedMs(const PlanarTimingEvent start, const PlanarTimingEvent end) const
  {
    float milliseconds = 0.0f;
    if(cudaEventElapsedTime(
        &milliseconds, planar_timing_events_[start], planar_timing_events_[end]) != cudaSuccess)
    {
      cudaGetLastError();
      return -1.0;
    }
    return static_cast<double>(milliseconds);
  }

  void release()
  {
    for(auto event : planar_timing_events_){
      if(event != nullptr){
        cudaEventDestroy(event);
      }
    }
    cudaFree(points_);
    cudaFree(nodes_);
    cudaFree(point_indices_);
    cudaFree(local_vertices_);
    cudaFree(controls_);
    cudaFree(control_starts_);
    cudaFree(control_counts_);
    cudaFree(planar_poses_);
    cudaFree(trajectory_points_);
    cudaFree(point_starts_);
    cudaFree(probe_points_);
    cudaFree(probe_starts_);
    cudaFree(probe_counts_);
    cudaFree(probe_first_block_);
    cudaFree(overflow_);
    cudaFree(planar_offsets_);
    cudaFree(planar_points_xy_);
    cudaFree(planar_distance_values_);
  }

  bool uploaded_ = false;
  int sm_count_ = 1;
  std::string device_name_;
  std::size_t point_count_ = 0;
  std::size_t node_count_ = 0;
  std::size_t point_index_count_ = 0;
  GpuPoint * points_ = nullptr;
  std::size_t point_capacity_ = 0;
  GpuBvhNode * nodes_ = nullptr;
  std::size_t node_capacity_ = 0;
  std::int32_t * point_indices_ = nullptr;
  std::size_t point_index_capacity_ = 0;
  GpuPoint * local_vertices_ = nullptr;
  std::size_t local_vertex_capacity_ = 0;
  MppiControl * controls_ = nullptr;
  std::size_t control_capacity_ = 0;
  std::size_t * control_starts_ = nullptr;
  std::size_t control_start_capacity_ = 0;
  std::size_t * control_counts_ = nullptr;
  std::size_t control_count_capacity_ = 0;
  // Per-candidate rows of pose_stride integrated poses for the split planar
  // rollout (phase 1 producer, phases 2 and 3 consumers).
  GpuPlanarPose * planar_poses_ = nullptr;
  std::size_t planar_pose_capacity_ = 0;
  GpuTrajectoryPoint * trajectory_points_ = nullptr;
  std::size_t trajectory_capacity_ = 0;
  std::size_t * point_starts_ = nullptr;
  std::size_t point_start_capacity_ = 0;
  GpuTrajectoryPoint * probe_points_ = nullptr;
  std::size_t probe_capacity_ = 0;
  std::size_t * probe_starts_ = nullptr;
  std::size_t probe_start_capacity_ = 0;
  std::size_t * probe_counts_ = nullptr;
  std::size_t probe_count_capacity_ = 0;
  double * probe_first_block_ = nullptr;
  std::size_t probe_first_capacity_ = 0;
  unsigned int * overflow_ = nullptr;
  std::size_t overflow_capacity_ = 0;
  // Tilted-plane 2-D scene. The grid is uploaded once per composed scene and
  // reused for every candidate batch; scalars mirror the host grid verbatim.
  bool planar_uploaded_ = false;
  std::uint32_t * planar_offsets_ = nullptr;
  std::size_t planar_offset_capacity_ = 0;
  double * planar_points_xy_ = nullptr;
  std::size_t planar_points_xy_capacity_ = 0;
  float * planar_distance_values_ = nullptr;
  std::size_t planar_distance_capacity_ = 0;
  MppiPlanarDistanceFieldView planar_distance_field_;
  std::size_t planar_cells_x_ = 0;
  std::size_t planar_cells_y_ = 0;
  std::size_t planar_point_count_ = 0;
  double planar_cell_size_ = 0.0;
  double planar_origin_x_ = 0.0;
  double planar_origin_y_ = 0.0;
  MppiCudaPlanarStats planar_stats_;
  bool planar_timing_attempted_ = false;
  std::array<cudaEvent_t, kPlanarTimingEventCount> planar_timing_events_{};
};

}  // namespace

std::unique_ptr<MppiCudaObstacles> makeMppiCudaObstacles()
{
  return std::make_unique<MppiCudaObstaclesImpl>();
}

}  // namespace local_planner
