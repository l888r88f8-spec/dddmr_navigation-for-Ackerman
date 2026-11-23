#include "local_planner/mppi_cuda_critics.h"

#include <cuda_runtime.h>

#include "local_planner/mppi_cuda_launch.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace local_planner
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

struct GpuPoint
{
  double x;
  double y;
  double yaw;
  double obstacle_clearance;
  unsigned int in_collision;
};

struct GpuPointView
{
  const GpuPoint * packed;
  const MppiCudaTrajectoryPoint * rollout;

  __device__ GpuPoint operator[](const std::size_t index) const
  {
    if(rollout != nullptr) {
      const auto & point = rollout[index];
      return GpuPoint{
        point.x, point.y, point.yaw, point.obstacle_clearance,
        point.in_collision};
    }
    return packed[index];
  }
};

using GpuControl = MppiControl;

struct GpuPathPoint
{
  double x;
  double y;
  double yaw;
  unsigned int obstacle_free;
};

struct GpuTrajectory
{
  unsigned long long point_offset;
  unsigned int point_count;
  unsigned long long probe_offset;
  unsigned int probe_count;
  unsigned long long control_offset;
  unsigned int control_count;
  unsigned int padding;
};

struct GpuParams
{
  int critic_count;
  unsigned int trajectory_count;
  unsigned int path_count;

  double robot_x;
  double robot_y;
  double robot_yaw;
  double effective_path_length;
  double vx_min;
  double vx_max;
  double min_turning_radius;
  double model_dt;
  double path_obstacle_occupancy_ratio;
  unsigned long long blocked_path_points;
  int near_goal;

  int constraint_enabled;
  int constraint_power;
  double constraint_weight;

  int obstacles_enabled;
  int obstacles_power;
  double obstacles_critical_weight;
  double obstacles_repulsion_weight;
  double obstacles_collision_cost;
  double obstacles_collision_margin_distance;
  double obstacles_inflation_radius;

  int goal_enabled;
  int goal_power;
  double goal_weight;
  double goal_threshold_to_consider;

  int goal_angle_enabled;
  int goal_angle_power;
  double goal_angle_weight;
  double goal_angle_threshold_to_consider;
  int goal_angle_symmetric_yaw_tolerance;

  int path_align_enabled;
  int path_align_power;
  double path_align_weight;
  double path_align_threshold_to_consider;
  double path_align_max_path_occupancy_ratio;
  unsigned long long path_align_offset_from_furthest;
  unsigned long long path_align_trajectory_point_step;
  int path_align_use_path_orientations;

  int path_follow_enabled;
  int path_follow_power;
  double path_follow_weight;
  double path_follow_threshold_to_consider;
  unsigned long long path_follow_offset_from_furthest;

  int path_angle_enabled;
  int path_angle_power;
  double path_angle_weight;
  double path_angle_threshold_to_consider;
  double path_angle_max_angle_to_furthest;
  unsigned long long path_angle_offset_from_furthest;
  int path_angle_mode;

  int prefer_forward_enabled;
  int prefer_forward_power;
  double prefer_forward_weight;
  double prefer_forward_threshold_to_consider;
};

struct GpuOutput
{
  double cost;
  double term[8];
  int fail;
};

struct GpuDiagnostics
{
  int path_align_applied;
  int path_align_skipped_blocked;
  int path_follow_applied;
  int path_follow_skipped_blocked;
  unsigned long long path_follow_target_index;
  double path_follow_target_distance;
};

enum CriticId : int
{
  kConstraint = 0,
  kObstacles = 1,
  kGoal = 2,
  kGoalAngle = 3,
  kPathAlign = 4,
  kPathFollow = 5,
  kPathAngle = 6,
  kPreferForward = 7,
  kUnknownCritic = -1
};

int criticId(const std::string & name)
{
  if(name == "ConstraintCritic") {
    return kConstraint;
  }
  if(name == "ObstaclesCritic") {
    return kObstacles;
  }
  if(name == "GoalCritic") {
    return kGoal;
  }
  if(name == "GoalAngleCritic") {
    return kGoalAngle;
  }
  if(name == "PathAlignCritic") {
    return kPathAlign;
  }
  if(name == "PathFollowCritic") {
    return kPathFollow;
  }
  if(name == "PathAngleCritic") {
    return kPathAngle;
  }
  if(name == "PreferForwardCritic") {
    return kPreferForward;
  }
  return kUnknownCritic;
}

double pathLength(const std::vector<MppiPathPoint> & path)
{
  double length = 0.0;
  for(std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

__device__ __forceinline__ double deviceMax(const double a, const double b)
{
  return a > b ? a : b;
}

__device__ __forceinline__ double deviceMin(const double a, const double b)
{
  return a < b ? a : b;
}

__device__ __forceinline__ double deviceAbs(const double value)
{
  return value < 0.0 ? -value : value;
}

__device__ __forceinline__ double deviceSqrt(const double value)
{
  return sqrt(value);
}

__device__ __forceinline__ double deviceAtan2(const double y, const double x)
{
  return atan2(y, x);
}

__device__ __forceinline__ double deviceSin(const double value)
{
  return sin(value);
}

__device__ __forceinline__ double deviceCos(const double value)
{
  return cos(value);
}

__device__ __forceinline__ double deviceHypot(const double x, const double y)
{
  return sqrt(x * x + y * y);
}

__device__ __forceinline__ double devicePow(const double value, const double power)
{
  return pow(value, power);
}

__device__ __forceinline__ double shortestAngularDistance(const double from, const double to)
{
  return deviceAtan2(deviceSin(to - from), deviceCos(to - from));
}

__device__ __forceinline__ double distance2d(
  const double x0, const double y0, const double x1, const double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return deviceSqrt(dx * dx + dy * dy);
}

__device__ __forceinline__ double poweredCost(
  const double value, const double weight, const int power)
{
  const double weighted = deviceMax(0.0, value) * weight;
  if(power <= 1) {
    return weighted;
  }
  return devicePow(weighted, static_cast<double>(power));
}

__device__ __forceinline__ void addPoweredCost(
  const double value, const double weight, const int power,
  double * cost, double * term, const int term_index)
{
  const double powered = poweredCost(value, weight, power);
  *cost += powered;
  term[term_index] += powered;
}

__device__ unsigned long long nearestPathIndex(
  const GpuPathPoint * path,
  const unsigned long long path_count,
  const double x,
  const double y,
  const unsigned long long start_index)
{
  if(path_count == 0) {
    return 0;
  }
  unsigned long long nearest = start_index < path_count ? start_index : path_count - 1;
  double best = __longlong_as_double(0x7ff0000000000000ULL);
  double best_squared = best;
  for(unsigned long long i = nearest; i < path_count; ++i) {
    const double dx = path[i].x - x;
    const double dy = path[i].y - y;
    const double squared = dx * dx + dy * dy;
    // sqrt is monotone: a squared distance at least as large as the current
    // winner cannot improve its rounded distance. Still compare the actual
    // square roots for a smaller value, preserving the original first-index
    // tie behavior when distinct squared distances round to the same root.
    if(squared < best_squared) {
      const double d = deviceSqrt(squared);
      if(d < best) {
        best = d;
        best_squared = squared;
        nearest = i;
      }
    }
  }
  return nearest;
}

__device__ void scoreConstraint(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuControl * controls,
  double * cost,
  double * term)
{
  if(!p.constraint_enabled) {
    return;
  }
  double raw = 0.0;
  for(unsigned int i = 0; i < trajectory.control_count; ++i) {
    const GpuControl control = controls[trajectory.control_offset + i];
    raw += deviceMax(0.0, control.vx - p.vx_max);
    raw += deviceMax(0.0, p.vx_min - control.vx);
    const double min_turning_radius = deviceMax(0.0, p.min_turning_radius);
    if(min_turning_radius > 0.0 && deviceAbs(control.wz) > 1e-6) {
      raw += deviceMax(
        0.0,
        min_turning_radius - deviceAbs(control.vx / control.wz));
    }
  }
  addPoweredCost(
    raw * deviceMax(1e-3, p.model_dt),
    p.constraint_weight,
    p.constraint_power,
    cost,
    term,
    kConstraint);
}

__device__ void scoreObstacles(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPointView probes,
  const unsigned int all_trajectories_collide,
  double * cost,
  double * term,
  int * fail)
{
  if(!p.obstacles_enabled) {
    return;
  }
  bool trajectory_collides = false;
  double raw_cost = 0.0;
  double repulsive_cost = 0.0;
  unsigned long long evaluated_clearance_points = 0;
  for(unsigned int i = 0; i < trajectory.point_count; ++i) {
    const GpuPoint point = points[trajectory.point_offset + i];
    if(point.in_collision != 0u) {
      trajectory_collides = true;
      break;
    }
    const double clearance = point.obstacle_clearance;
    if(!isfinite(clearance)) {
      continue;
    }
    ++evaluated_clearance_points;
    if(clearance < p.obstacles_collision_margin_distance) {
      raw_cost += p.obstacles_collision_margin_distance - clearance;
    }
    if(!p.near_goal && clearance < p.obstacles_inflation_radius) {
      repulsive_cost += p.obstacles_inflation_radius - clearance;
    }
  }
  if(!trajectory_collides) {
    for(unsigned int i = 0; i < trajectory.probe_count; ++i) {
      if(probes[trajectory.probe_offset + i].in_collision != 0u) {
        trajectory_collides = true;
        break;
      }
    }
  }

  const double normalizer = static_cast<double>(
    evaluated_clearance_points > 0ull ? evaluated_clearance_points : 1ull);
  const double total =
    p.obstacles_critical_weight *
    (trajectory_collides ? p.obstacles_collision_cost : raw_cost) +
    p.obstacles_repulsion_weight * repulsive_cost / normalizer;
  addPoweredCost(total, 1.0, p.obstacles_power, cost, term, kObstacles);
  if(all_trajectories_collide != 0u) {
    *fail = 1;
  }
}

__device__ void scoreGoal(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPathPoint * path,
  double * cost,
  double * term)
{
  if(!p.goal_enabled || p.path_count == 0) {
    return;
  }
  if(p.effective_path_length > p.goal_threshold_to_consider) {
    return;
  }
  const GpuPathPoint goal = path[p.path_count - 1];
  double raw = 0.0;
  for(unsigned int i = 0; i < trajectory.point_count; ++i) {
    const GpuPoint point = points[trajectory.point_offset + i];
    raw += distance2d(point.x, point.y, goal.x, goal.y);
  }
  raw /= static_cast<double>(trajectory.point_count > 0u ? trajectory.point_count : 1u);
  addPoweredCost(raw, p.goal_weight, p.goal_power, cost, term, kGoal);
}

__device__ void scoreGoalAngle(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPathPoint * path,
  double * cost,
  double * term)
{
  if(!p.goal_angle_enabled || p.path_count == 0) {
    return;
  }
  if(p.effective_path_length > p.goal_angle_threshold_to_consider) {
    return;
  }
  const GpuPathPoint goal = path[p.path_count - 1];
  double raw = 0.0;
  for(unsigned int i = 0; i < trajectory.point_count; ++i) {
    const GpuPoint point = points[trajectory.point_offset + i];
    double angular_distance =
      deviceAbs(shortestAngularDistance(point.yaw, goal.yaw));
    if(p.goal_angle_symmetric_yaw_tolerance != 0) {
      angular_distance = deviceMin(
        angular_distance,
        deviceAbs(shortestAngularDistance(point.yaw, goal.yaw + kPi)));
    }
    raw += angular_distance;
  }
  raw /= static_cast<double>(trajectory.point_count > 0u ? trajectory.point_count : 1u);
  addPoweredCost(
    raw, p.goal_angle_weight, p.goal_angle_power, cost, term, kGoalAngle);
}

__device__ void scorePathAlign(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPathPoint * path,
  const unsigned int furthest,
  const bool diagnostics_thread,
  GpuDiagnostics * diagnostics,
  double * cost,
  double * term)
{
  if(!p.path_align_enabled || p.path_count < 2u) {
    return;
  }
  if(p.effective_path_length < p.path_align_threshold_to_consider) {
    return;
  }
  if(p.blocked_path_points >= 3ull &&
    p.path_obstacle_occupancy_ratio > p.path_align_max_path_occupancy_ratio)
  {
    if(diagnostics_thread) {
      diagnostics->path_align_skipped_blocked = 1;
    }
    return;
  }
  if(static_cast<unsigned long long>(furthest) < p.path_align_offset_from_furthest) {
    return;
  }

  if(diagnostics_thread) {
    diagnostics->path_align_applied = 1;
  }
  const unsigned long long step =
    p.path_align_trajectory_point_step > 0ull ?
    p.path_align_trajectory_point_step : 1ull;
  double raw = 0.0;
  unsigned long long count = 0ull;
  unsigned long long path_search_from = 0ull;
  for(unsigned long long point_index = 0;
    point_index < trajectory.point_count; point_index += step)
  {
    const GpuPoint point = points[trajectory.point_offset + point_index];
    path_search_from = nearestPathIndex(
      path, p.path_count, point.x, point.y, path_search_from);
    const GpuPathPoint path_point = path[path_search_from];
    raw += distance2d(point.x, point.y, path_point.x, path_point.y);
    if(p.path_align_use_path_orientations != 0) {
      raw += deviceAbs(shortestAngularDistance(point.yaw, path_point.yaw));
    }
    ++count;
  }
  raw /= static_cast<double>(count > 0ull ? count : 1ull);
  addPoweredCost(
    raw, p.path_align_weight, p.path_align_power, cost, term, kPathAlign);
}

__device__ void scorePathFollow(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPathPoint * path,
  const unsigned int furthest,
  const bool diagnostics_thread,
  GpuDiagnostics * diagnostics,
  double * cost,
  double * term)
{
  if(!p.path_follow_enabled || p.path_count == 0u) {
    return;
  }
  if(p.effective_path_length < p.path_follow_threshold_to_consider) {
    return;
  }

  const unsigned long long path_count = p.path_count;
  unsigned long long target_index = static_cast<unsigned long long>(furthest) +
    p.path_follow_offset_from_furthest;
  if(target_index > path_count - 1ull) {
    target_index = path_count - 1ull;
  }
  while(target_index < path_count && path[target_index].obstacle_free == 0u) {
    ++target_index;
  }

  GpuPathPoint target;
  if(target_index < path_count) {
    target = path[target_index];
  } else {
    const unsigned long long robot_index = nearestPathIndex(
      path, path_count, p.robot_x, p.robot_y, 0ull);
    double free_route_ahead = 0.0;
    unsigned long long free_index = robot_index;
    while(path[free_index].obstacle_free != 0u &&
      free_index + 1ull < path_count &&
      path[free_index + 1ull].obstacle_free != 0u)
    {
      free_route_ahead += distance2d(
        path[free_index].x, path[free_index].y,
        path[free_index + 1ull].x, path[free_index + 1ull].y);
      ++free_index;
    }
    if(path[robot_index].obstacle_free == 0u ||
      free_route_ahead < p.path_follow_threshold_to_consider)
    {
      if(diagnostics_thread) {
        diagnostics->path_follow_skipped_blocked = 1;
      }
      return;
    }

    target = path[path_count - 1ull];
    double tangent_x = 0.0;
    double tangent_y = 0.0;
    if(path_count >= 2ull) {
      const GpuPathPoint before = path[path_count - 2ull];
      tangent_x = path[path_count - 1ull].x - before.x;
      tangent_y = path[path_count - 1ull].y - before.y;
    }
    const double tangent_length = deviceHypot(tangent_x, tangent_y);
    const double spacing = tangent_length > 1e-6 ? tangent_length : 0.1;
    if(tangent_length > 1e-6) {
      tangent_x /= tangent_length;
      tangent_y /= tangent_length;
    } else {
      tangent_x = deviceCos(path[path_count - 1ull].yaw);
      tangent_y = deviceSin(path[path_count - 1ull].yaw);
    }
    const unsigned long long extension_steps =
      p.path_follow_offset_from_furthest > 0ull ?
      p.path_follow_offset_from_furthest : 1ull;
    const double extension = static_cast<double>(extension_steps) * spacing;
    target.x += tangent_x * extension;
    target.y += tangent_y * extension;
  }

  if(trajectory.point_count > 0u) {
    const GpuPoint last = points[
      trajectory.point_offset + trajectory.point_count - 1u];
    addPoweredCost(
      distance2d(last.x, last.y, target.x, target.y),
      p.path_follow_weight,
      p.path_follow_power,
      cost,
      term,
      kPathFollow);
  }
  if(diagnostics_thread) {
    diagnostics->path_follow_applied = 1;
    diagnostics->path_follow_target_index = target_index;
    diagnostics->path_follow_target_distance =
      distance2d(p.robot_x, p.robot_y, target.x, target.y);
  }
}

__device__ void scorePathAngle(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuPointView points,
  const GpuPathPoint * path,
  const unsigned int furthest,
  double * cost,
  double * term)
{
  if(!p.path_angle_enabled || p.path_count == 0u) {
    return;
  }
  if(p.effective_path_length < p.path_angle_threshold_to_consider) {
    return;
  }

  unsigned long long target_index = static_cast<unsigned long long>(furthest) +
    p.path_angle_offset_from_furthest;
  if(target_index > static_cast<unsigned long long>(p.path_count) - 1ull) {
    target_index = static_cast<unsigned long long>(p.path_count) - 1ull;
  }
  const GpuPathPoint target = path[target_index];
  const double robot_to_target = deviceAtan2(
    target.y - p.robot_y, target.x - p.robot_x);
  if(deviceAbs(shortestAngularDistance(p.robot_yaw, robot_to_target)) <
    p.path_angle_max_angle_to_furthest)
  {
    return;
  }

  if(trajectory.point_count == 0u) {
    return;
  }
  const GpuPoint last = points[trajectory.point_offset + trajectory.point_count - 1u];
  const double terminal_to_target = deviceAtan2(
    target.y - last.y, target.x - last.x);
  // The reference manager currently applies the same terminal-heading formula
  // for every configured PathAngleCritic mode (0, 1, 2, ...).  Keep that exact
  // behaviour here rather than inventing a mode-dependent cost.
  (void)p.path_angle_mode;
  addPoweredCost(
    deviceAbs(shortestAngularDistance(last.yaw, terminal_to_target)),
    p.path_angle_weight,
    p.path_angle_power,
    cost,
    term,
    kPathAngle);
}

__device__ void scorePreferForward(
  const GpuParams & p,
  const GpuTrajectory & trajectory,
  const GpuControl * controls,
  double * cost,
  double * term)
{
  if(!p.prefer_forward_enabled) {
    return;
  }
  if(p.effective_path_length < p.prefer_forward_threshold_to_consider) {
    return;
  }
  double reverse_motion = 0.0;
  for(unsigned int i = 0; i < trajectory.control_count; ++i) {
    const GpuControl control = controls[trajectory.control_offset + i];
    reverse_motion += deviceMax(0.0, -control.vx) * deviceMax(1e-3, p.model_dt);
  }
  addPoweredCost(
    reverse_motion,
    p.prefer_forward_weight,
    p.prefer_forward_power,
    cost,
    term,
    kPreferForward);
}

__global__ void prepareCriticBatchKernel(
  const GpuParams params,
  const GpuPathPoint * path,
  const GpuTrajectory * trajectories,
  const GpuPointView points,
  const GpuPointView probes,
  unsigned int * furthest,
  unsigned int * all_trajectories_collide)
{
  const unsigned int candidate = blockIdx.x * blockDim.x + threadIdx.x;
  if(candidate >= params.trajectory_count) {
    return;
  }
  const GpuTrajectory trajectory = trajectories[candidate];
  unsigned long long search_from = 0ull;
  unsigned int candidate_furthest = 0u;
  for(unsigned int i = 0; i < trajectory.point_count; ++i) {
    const GpuPoint point = points[trajectory.point_offset + i];
    search_from = nearestPathIndex(
      path, params.path_count, point.x, point.y, search_from);
    if(search_from > candidate_furthest) {
      candidate_furthest = static_cast<unsigned int>(search_from);
    }
  }
  atomicMax(furthest, candidate_furthest);

  bool collides = false;
  for(unsigned int i = 0; i < trajectory.point_count; ++i) {
    if(points[trajectory.point_offset + i].in_collision != 0u) {
      collides = true;
      break;
    }
  }
  if(!collides) {
    for(unsigned int i = 0; i < trajectory.probe_count; ++i) {
      if(probes[trajectory.probe_offset + i].in_collision != 0u) {
        collides = true;
        break;
      }
    }
  }
  atomicAnd(all_trajectories_collide, collides ? 1u : 0u);
}

__global__ void scoreCriticBatchKernel(
  const GpuParams params,
  const GpuPathPoint * path,
  const GpuTrajectory * trajectories,
  const GpuPointView points,
  const GpuPointView probes,
  const GpuControl * controls,
  const int * critic_ids,
  const unsigned int * furthest,
  const unsigned int * all_trajectories_collide,
  GpuOutput * outputs,
  GpuDiagnostics * diagnostics)
{
  const unsigned int candidate = blockIdx.x * blockDim.x + threadIdx.x;
  if(candidate >= params.trajectory_count) {
    return;
  }
  const bool diagnostics_thread = blockIdx.x == 0u && threadIdx.x == 0u;
  const GpuTrajectory trajectory = trajectories[candidate];
  double cost = 0.0;
  double term[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int fail = 0;
  const unsigned int global_furthest = *furthest;
  const unsigned int all_collide = *all_trajectories_collide;

  for(int i = 0; i < params.critic_count; ++i) {
    if(fail != 0) {
      break;
    }
    switch(critic_ids[i]) {
      case kConstraint:
        scoreConstraint(params, trajectory, controls, &cost, term);
        break;
      case kObstacles:
        scoreObstacles(
          params, trajectory, points, probes, all_collide,
          &cost, term, &fail);
        break;
      case kGoal:
        scoreGoal(params, trajectory, points, path, &cost, term);
        break;
      case kGoalAngle:
        scoreGoalAngle(params, trajectory, points, path, &cost, term);
        break;
      case kPathAlign:
        scorePathAlign(
          params, trajectory, points, path, global_furthest,
          diagnostics_thread, diagnostics, &cost, term);
        break;
      case kPathFollow:
        scorePathFollow(
          params, trajectory, points, path, global_furthest,
          diagnostics_thread, diagnostics, &cost, term);
        break;
      case kPathAngle:
        scorePathAngle(
          params, trajectory, points, path, global_furthest, &cost, term);
        break;
      case kPreferForward:
        scorePreferForward(params, trajectory, controls, &cost, term);
        break;
      default:
        // Disabled and unknown critic names are no-ops in MppiCriticManager.
        break;
    }
  }

  GpuOutput output;
  output.cost = cost;
  for(int i = 0; i < 8; ++i) {
    output.term[i] = term[i];
  }
  output.fail = fail;
  outputs[candidate] = output;
}

// Shared path diagnostics are a property of the path and critic order, not of
// any single candidate. MppiCriticManager evaluates them even when the sample
// batch is empty: with no trajectories the furthest reached index is zero, yet
// PathFollow still resolves its target and reports applied/index/distance, and
// PathAlign applies whenever its furthest-offset gate admits index zero.
// Reusing the scoring helpers with one synthetic empty trajectory preserves the
// reference gating and ordering. The synthetic batch never collides, so the
// obstacle fail gate is cleared rather than read from the empty prepare pass,
// which would otherwise latch all_collide to true and skip later critics.
__global__ void scoreSharedDiagnosticsKernel(
  const GpuParams params,
  const GpuPathPoint * path,
  const GpuPointView points,
  const GpuPointView probes,
  const GpuControl * controls,
  const int * critic_ids,
  const unsigned int * furthest,
  GpuDiagnostics * diagnostics)
{
  if(blockIdx.x != 0u || threadIdx.x != 0u) {
    return;
  }
  const GpuTrajectory empty_trajectory{};
  const unsigned int global_furthest = *furthest;
  const unsigned int no_candidate_collision = 0u;
  double cost = 0.0;
  double term[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int ignored_fail = 0;

  for(int i = 0; i < params.critic_count; ++i) {
    switch(critic_ids[i]) {
      case kConstraint:
        scoreConstraint(params, empty_trajectory, controls, &cost, term);
        break;
      case kObstacles:
        scoreObstacles(
          params, empty_trajectory, points, probes,
          no_candidate_collision, &cost, term, &ignored_fail);
        break;
      case kGoal:
        scoreGoal(params, empty_trajectory, points, path, &cost, term);
        break;
      case kGoalAngle:
        scoreGoalAngle(params, empty_trajectory, points, path, &cost, term);
        break;
      case kPathAlign:
        scorePathAlign(
          params, empty_trajectory, points, path, global_furthest,
          true, diagnostics, &cost, term);
        break;
      case kPathFollow:
        scorePathFollow(
          params, empty_trajectory, points, path, global_furthest,
          true, diagnostics, &cost, term);
        break;
      case kPathAngle:
        scorePathAngle(
          params, empty_trajectory, points, path, global_furthest, &cost, term);
        break;
      case kPreferForward:
        scorePreferForward(params, empty_trajectory, controls, &cost, term);
        break;
      default:
        break;
    }
  }
}

template<typename T>
class DeviceBuffer
{
public:
  DeviceBuffer() = default;
  ~DeviceBuffer()
  {
    release();
  }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;

  bool reserve(const std::size_t count, const char * name, std::string & error)
  {
    if(count <= capacity_) {
      return true;
    }
    release();
    if(count == 0) {
      return true;
    }
    void * allocation = nullptr;
    const cudaError_t status = cudaMalloc(&allocation, count * sizeof(T));
    if(status != cudaSuccess) {
      error = std::string("CUDA allocation failed for ") + name + ": " +
        cudaGetErrorString(status);
      return false;
    }
    pointer_ = static_cast<T *>(allocation);
    capacity_ = count;
    return true;
  }

  T * get()
  {
    return pointer_;
  }

  const T * get() const
  {
    return pointer_;
  }

  std::size_t capacity() const
  {
    return capacity_;
  }

private:
  void release()
  {
    if(pointer_ != nullptr) {
      cudaFree(pointer_);
      pointer_ = nullptr;
      capacity_ = 0;
    }
  }

  T * pointer_ = nullptr;
  std::size_t capacity_ = 0;
};

template<typename T>
bool uploadToDevice(
  DeviceBuffer<T> & buffer,
  const std::vector<T> & host,
  const char * name,
  std::string & error)
{
  if(!buffer.reserve(host.size(), name, error)) {
    return false;
  }
  if(host.empty()) {
    return true;
  }
  const cudaError_t status = cudaMemcpy(
    buffer.get(), host.data(), host.size() * sizeof(T), cudaMemcpyHostToDevice);
  if(status != cudaSuccess) {
    error = std::string("CUDA upload failed for ") + name + ": " +
      cudaGetErrorString(status);
    return false;
  }
  return true;
}

}  // namespace

struct MppiCudaCritics::Impl
{
  bool available = false;
  std::string reason;
  int sm_count = 1;

  DeviceBuffer<GpuPathPoint> device_path;
  DeviceBuffer<GpuTrajectory> device_trajectories;
  DeviceBuffer<GpuPoint> device_points;
  DeviceBuffer<GpuPoint> device_probes;
  DeviceBuffer<GpuControl> device_controls;
  DeviceBuffer<int> device_critic_ids;
  DeviceBuffer<GpuParams> device_params;
  DeviceBuffer<GpuOutput> device_outputs;
  DeviceBuffer<GpuDiagnostics> device_diagnostics;
  DeviceBuffer<unsigned int> device_furthest;
  DeviceBuffer<unsigned int> device_all_collide;

  std::vector<GpuPathPoint> host_path;
  std::vector<GpuTrajectory> host_trajectories;
  std::vector<GpuPoint> host_points;
  std::vector<GpuPoint> host_probes;
  std::vector<GpuControl> host_controls;
  std::vector<int> host_critic_ids;
  std::vector<GpuOutput> host_outputs;
  std::vector<GpuDiagnostics> host_diagnostics;

  Impl()
  {
    int device_count = 0;
    cudaError_t status = cudaGetDeviceCount(&device_count);
    if(status != cudaSuccess) {
      reason = std::string("CUDA device query failed: ") +
        cudaGetErrorString(status);
      return;
    }
    if(device_count <= 0) {
      reason = "no CUDA device is visible";
      return;
    }
    status = cudaSetDevice(0);
    if(status != cudaSuccess) {
      reason = std::string("cudaSetDevice failed: ") + cudaGetErrorString(status);
      return;
    }
    cudaDeviceProp properties{};
    status = cudaGetDeviceProperties(&properties, 0);
    if(status != cudaSuccess) {
      reason = std::string("cudaGetDeviceProperties failed: ") +
        cudaGetErrorString(status);
      return;
    }
    available = true;
    sm_count = std::max(1, properties.multiProcessorCount);
    reason = std::string(properties.name) + " (" +
      std::to_string(sm_count) + " SMs)";
  }
};

MppiCudaCritics::MppiCudaCritics()
: impl_(std::make_unique<Impl>())
{
}

MppiCudaCritics::~MppiCudaCritics() = default;

MppiCudaCritics::MppiCudaCritics(MppiCudaCritics &&) noexcept = default;

MppiCudaCritics & MppiCudaCritics::operator=(MppiCudaCritics &&) noexcept = default;

bool MppiCudaCritics::available(std::string * reason) const
{
  if(!impl_) {
    if(reason != nullptr) {
      *reason = "CUDA critic implementation is not initialized";
    }
    return false;
  }
  if(!impl_->available && reason != nullptr) {
    *reason = impl_->reason;
  }
  return impl_->available;
}

bool MppiCudaCritics::score(
  const MppiCriticConfig & config,
  const MppiCriticData & data,
  MppiCriticResult & result,
  std::string & error)
{
  return scoreImpl(config, data, nullptr, result, error);
}

bool MppiCudaCritics::scoreDevice(
  const MppiCriticConfig & config,
  const MppiCriticData & data,
  const MppiCudaTrajectoryBatchView & device_batch,
  MppiCriticResult & result,
  std::string & error)
{
  return scoreImpl(config, data, &device_batch, result, error);
}

bool MppiCudaCritics::scoreImpl(
  const MppiCriticConfig & config,
  const MppiCriticData & data,
  const MppiCudaTrajectoryBatchView * device_batch,
  MppiCriticResult & result,
  std::string & error)
{
  error.clear();
  result = MppiCriticResult{};
  if(!impl_) {
    error = "CUDA critic implementation is not initialized";
    return false;
  }
  Impl & impl = *impl_;
  if(!impl.available) {
    error = "MPPI CUDA critics are unavailable: " + impl.reason;
    return false;
  }
  if(data.trajectories.size() > std::numeric_limits<unsigned int>::max() ||
    data.path.size() > std::numeric_limits<unsigned int>::max() ||
    config.critics.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    error = "MPPI CUDA critic input exceeds the supported 32-bit batch dimensions";
    return false;
  }

  GpuParams params{};
  params.critic_count = static_cast<int>(config.critics.size());
  params.trajectory_count = static_cast<unsigned int>(data.trajectories.size());
  params.path_count = static_cast<unsigned int>(data.path.size());
  params.robot_x = data.robot_x;
  params.robot_y = data.robot_y;
  params.robot_yaw = data.robot_yaw;
  const double effective_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  params.effective_path_length = effective_path_length;
  params.vx_min = data.vx_min;
  params.vx_max = data.vx_max;
  params.min_turning_radius = data.min_turning_radius;
  params.model_dt = data.model_dt;
  params.path_obstacle_occupancy_ratio = data.path_obstacle_occupancy_ratio;
  params.blocked_path_points = data.blocked_path_points;
  params.near_goal =
    effective_path_length < config.obstacles.near_goal_distance ? 1 : 0;

  params.constraint_enabled = config.constraint.enabled ? 1 : 0;
  params.constraint_power = config.constraint.cost_power;
  params.constraint_weight = config.constraint.cost_weight;

  params.obstacles_enabled = config.obstacles.enabled ? 1 : 0;
  params.obstacles_power = config.obstacles.cost_power;
  params.obstacles_critical_weight = config.obstacles.critical_weight;
  params.obstacles_repulsion_weight = config.obstacles.repulsion_weight;
  params.obstacles_collision_cost = config.obstacles.collision_cost;
  params.obstacles_collision_margin_distance =
    config.obstacles.collision_margin_distance;
  params.obstacles_inflation_radius = config.obstacles.inflation_radius;

  params.goal_enabled = config.goal.enabled ? 1 : 0;
  params.goal_power = config.goal.cost_power;
  params.goal_weight = config.goal.cost_weight;
  params.goal_threshold_to_consider = config.goal.threshold_to_consider;

  params.goal_angle_enabled = config.goal_angle.enabled ? 1 : 0;
  params.goal_angle_power = config.goal_angle.cost_power;
  params.goal_angle_weight = config.goal_angle.cost_weight;
  params.goal_angle_threshold_to_consider =
    config.goal_angle.threshold_to_consider;
  params.goal_angle_symmetric_yaw_tolerance =
    config.goal_angle.symmetric_yaw_tolerance ? 1 : 0;

  params.path_align_enabled = config.path_align.enabled ? 1 : 0;
  params.path_align_power = config.path_align.cost_power;
  params.path_align_weight = config.path_align.cost_weight;
  params.path_align_threshold_to_consider =
    config.path_align.threshold_to_consider;
  params.path_align_max_path_occupancy_ratio =
    config.path_align.max_path_occupancy_ratio;
  params.path_align_offset_from_furthest =
    static_cast<unsigned long long>(config.path_align.offset_from_furthest);
  params.path_align_trajectory_point_step =
    static_cast<unsigned long long>(config.path_align.trajectory_point_step);
  params.path_align_use_path_orientations =
    config.path_align.use_path_orientations ? 1 : 0;

  params.path_follow_enabled = config.path_follow.enabled ? 1 : 0;
  params.path_follow_power = config.path_follow.cost_power;
  params.path_follow_weight = config.path_follow.cost_weight;
  params.path_follow_threshold_to_consider =
    config.path_follow.threshold_to_consider;
  params.path_follow_offset_from_furthest =
    static_cast<unsigned long long>(config.path_follow.offset_from_furthest);

  params.path_angle_enabled = config.path_angle.enabled ? 1 : 0;
  params.path_angle_power = config.path_angle.cost_power;
  params.path_angle_weight = config.path_angle.cost_weight;
  params.path_angle_threshold_to_consider =
    config.path_angle.threshold_to_consider;
  params.path_angle_max_angle_to_furthest =
    config.path_angle.max_angle_to_furthest;
  params.path_angle_offset_from_furthest =
    static_cast<unsigned long long>(config.path_angle.offset_from_furthest);
  params.path_angle_mode = config.path_angle.mode;

  params.prefer_forward_enabled = config.prefer_forward.enabled ? 1 : 0;
  params.prefer_forward_power = config.prefer_forward.cost_power;
  params.prefer_forward_weight = config.prefer_forward.cost_weight;
  params.prefer_forward_threshold_to_consider =
    config.prefer_forward.threshold_to_consider;

  impl.host_path.clear();
  impl.host_trajectories.clear();
  impl.host_points.clear();
  impl.host_probes.clear();
  impl.host_controls.clear();
  impl.host_critic_ids.clear();

  std::size_t total_points = 0;
  std::size_t total_probes = 0;
  std::size_t total_controls = 0;
  for(const auto & trajectory : data.trajectories) {
    total_points += trajectory.points.size();
    total_probes += trajectory.probe_points.size();
    total_controls += trajectory.controls.size();
  }
  if(total_points > std::numeric_limits<unsigned int>::max() ||
    total_probes > std::numeric_limits<unsigned int>::max() ||
    total_controls > std::numeric_limits<unsigned int>::max())
  {
    error = "MPPI CUDA critic input exceeds the supported 32-bit point offsets";
    return false;
  }
  if(device_batch != nullptr &&
    (device_batch->candidate_count != data.trajectories.size() ||
    device_batch->point_count != total_points ||
    device_batch->control_count != total_controls ||
    (total_points != 0 && device_batch->points == nullptr) ||
    (total_controls != 0 && device_batch->controls == nullptr) ||
    (total_probes != 0 && device_batch->probes == nullptr)))
  {
    error = "MPPI CUDA critic rollout batch does not match the host metadata";
    return false;
  }

  impl.host_path.reserve(data.path.size());
  for(const auto & point : data.path) {
    impl.host_path.push_back(GpuPathPoint{
      point.x, point.y, point.yaw, point.obstacle_free ? 1u : 0u});
  }
  impl.host_trajectories.reserve(data.trajectories.size());
  if(device_batch == nullptr) {
    impl.host_points.reserve(total_points);
    impl.host_probes.reserve(total_probes);
    impl.host_controls.reserve(total_controls);
  }
  std::size_t point_offset = 0;
  std::size_t control_offset = 0;
  std::size_t probe_offset = 0;
  for(const auto & trajectory : data.trajectories) {
    GpuTrajectory packed{};
    packed.point_offset = point_offset;
    packed.point_count = static_cast<unsigned int>(trajectory.points.size());
    packed.probe_offset = probe_offset;
    packed.probe_count = static_cast<unsigned int>(trajectory.probe_points.size());
    packed.control_offset = control_offset;
    packed.control_count = static_cast<unsigned int>(trajectory.controls.size());
    point_offset += packed.point_count;
    control_offset += packed.control_count;
    if(device_batch != nullptr) {
      const std::size_t slice_size = packed.control_count == 0 ? 0 : device_batch->probe_stride;
      if(packed.point_count != packed.control_count || packed.probe_count > slice_size ||
        slice_size > device_batch->probe_capacity - probe_offset)
      {
        error = "MPPI CUDA critic rollout contains an invalid candidate slice";
        return false;
      }
      probe_offset += slice_size;
    } else {
      for(const auto & point : trajectory.points) {
        impl.host_points.push_back(GpuPoint{
          point.x, point.y, point.yaw, point.obstacle_clearance,
          point.in_collision ? 1u : 0u});
      }
      for(const auto & point : trajectory.probe_points) {
        impl.host_probes.push_back(GpuPoint{
          point.x, point.y, point.yaw, point.obstacle_clearance,
          point.in_collision ? 1u : 0u});
      }
      impl.host_controls.insert(
        impl.host_controls.end(), trajectory.controls.begin(), trajectory.controls.end());
      probe_offset += packed.probe_count;
    }
    impl.host_trajectories.push_back(packed);
  }
  if(device_batch != nullptr && probe_offset != device_batch->probe_capacity) {
    error = "MPPI CUDA critic rollout probe slices do not cover the batch";
    return false;
  }
  impl.host_critic_ids.reserve(config.critics.size());
  for(const auto & name : config.critics) {
    impl.host_critic_ids.push_back(criticId(name));
  }

  if(!uploadToDevice(
      impl.device_path, impl.host_path, "path points", error) ||
    !uploadToDevice(
      impl.device_trajectories, impl.host_trajectories, "trajectories", error) ||
    (device_batch == nullptr &&
    (!uploadToDevice(impl.device_points, impl.host_points, "trajectory points", error) ||
    !uploadToDevice(impl.device_probes, impl.host_probes, "probe points", error) ||
    !uploadToDevice(impl.device_controls, impl.host_controls, "controls", error))) ||
    !uploadToDevice(
      impl.device_critic_ids, impl.host_critic_ids, "critic order", error) ||
    !impl.device_params.reserve(1, "params", error) ||
    !impl.device_furthest.reserve(1, "furthest index", error) ||
    !impl.device_all_collide.reserve(1, "collision flag", error) ||
    !impl.device_diagnostics.reserve(1, "diagnostics", error) ||
    !impl.device_outputs.reserve(data.trajectories.size(), "outputs", error))
  {
    return false;
  }

  const GpuPointView points = device_batch == nullptr ?
    GpuPointView{impl.device_points.get(), nullptr} : GpuPointView{nullptr, device_batch->points};
  const GpuPointView probes = device_batch == nullptr ?
    GpuPointView{impl.device_probes.get(), nullptr} : GpuPointView{nullptr, device_batch->probes};
  const GpuControl * controls = device_batch == nullptr ?
    impl.device_controls.get() : device_batch->controls;

  cudaError_t status = cudaMemcpy(
    impl.device_params.get(), &params, sizeof(params), cudaMemcpyHostToDevice);
  if(status != cudaSuccess) {
    error = std::string("CUDA upload failed for params: ") +
      cudaGetErrorString(status);
    return false;
  }
  unsigned int zero_furthest = 0u;
  unsigned int all_collide = 1u;
  status = cudaMemcpy(
    impl.device_furthest.get(), &zero_furthest, sizeof(zero_furthest),
    cudaMemcpyHostToDevice);
  if(status == cudaSuccess) {
    status = cudaMemcpy(
      impl.device_all_collide.get(), &all_collide, sizeof(all_collide),
      cudaMemcpyHostToDevice);
  }
  if(status == cudaSuccess) {
    status = cudaMemset(
      impl.device_diagnostics.get(), 0, sizeof(GpuDiagnostics));
  }
  if(status != cudaSuccess) {
    error = std::string("CUDA initialization failed: ") + cudaGetErrorString(status);
    return false;
  }

  const unsigned int candidate_count = params.trajectory_count;
  const MppiCudaLaunchGeometry launch =
    mppiCudaSelectLaunch(candidate_count, impl_->sm_count);
  const unsigned int block_size = launch.block_dim;
  if(candidate_count > 0u) {
    const unsigned int block_count = launch.grid;
    prepareCriticBatchKernel<<<block_count, block_size>>>(
      params,
      impl.device_path.get(),
      impl.device_trajectories.get(),
      points,
      probes,
      impl.device_furthest.get(),
      impl.device_all_collide.get());
    status = cudaGetLastError();
    if(status != cudaSuccess) {
      error = std::string("CUDA prepare kernel launch failed: ") +
        cudaGetErrorString(status);
      return false;
    }
    scoreCriticBatchKernel<<<block_count, block_size>>>(
      params,
      impl.device_path.get(),
      impl.device_trajectories.get(),
      points,
      probes,
      controls,
      impl.device_critic_ids.get(),
      impl.device_furthest.get(),
      impl.device_all_collide.get(),
      impl.device_outputs.get(),
      impl.device_diagnostics.get());
    status = cudaGetLastError();
    if(status != cudaSuccess) {
      error = std::string("CUDA critic kernel launch failed: ") +
        cudaGetErrorString(status);
      return false;
    }
  } else {
    // No candidate thread exists to publish the shared path diagnostics, so
    // evaluate them on a one-thread metadata pass with the same critic order.
    scoreSharedDiagnosticsKernel<<<1, 1>>>(
      params,
      impl.device_path.get(),
      points,
      probes,
      controls,
      impl.device_critic_ids.get(),
      impl.device_furthest.get(),
      impl.device_diagnostics.get());
    status = cudaGetLastError();
    if(status != cudaSuccess) {
      error = std::string("CUDA shared diagnostics kernel launch failed: ") +
        cudaGetErrorString(status);
      return false;
    }
  }
  status = cudaDeviceSynchronize();
  if(status != cudaSuccess) {
    error = std::string("CUDA critic execution failed: ") +
      cudaGetErrorString(status);
    return false;
  }

  impl.host_outputs.resize(data.trajectories.size());
  impl.host_diagnostics.resize(1);
  if(!impl.host_outputs.empty()) {
    status = cudaMemcpy(
      impl.host_outputs.data(), impl.device_outputs.get(),
      impl.host_outputs.size() * sizeof(GpuOutput), cudaMemcpyDeviceToHost);
    if(status != cudaSuccess) {
      error = std::string("CUDA output download failed: ") +
        cudaGetErrorString(status);
      return false;
    }
  }
  status = cudaMemcpy(
    impl.host_diagnostics.data(), impl.device_diagnostics.get(),
    sizeof(GpuDiagnostics), cudaMemcpyDeviceToHost);
  if(status != cudaSuccess) {
    error = std::string("CUDA diagnostics download failed: ") +
      cudaGetErrorString(status);
    return false;
  }

  result.scores.assign(
    data.trajectories.size(), MppiCandidateScore{true, 0.0});
  result.term_costs.assign(data.trajectories.size(), MppiCriticTermCosts{});
  for(std::size_t i = 0; i < impl.host_outputs.size(); ++i) {
    const GpuOutput & output = impl.host_outputs[i];
    result.scores[i].cost = output.cost;
    result.term_costs[i].constraint = output.term[kConstraint];
    result.term_costs[i].obstacles = output.term[kObstacles];
    result.term_costs[i].goal = output.term[kGoal];
    result.term_costs[i].goal_angle = output.term[kGoalAngle];
    result.term_costs[i].path_align = output.term[kPathAlign];
    result.term_costs[i].path_follow = output.term[kPathFollow];
    result.term_costs[i].path_angle = output.term[kPathAngle];
    result.term_costs[i].prefer_forward = output.term[kPreferForward];
    result.fail_flag = result.fail_flag || output.fail != 0;
  }
  const GpuDiagnostics & diagnostics = impl.host_diagnostics.front();
  result.path_align_applied = diagnostics.path_align_applied != 0;
  result.path_align_skipped_blocked =
    diagnostics.path_align_skipped_blocked != 0;
  result.path_follow_applied = diagnostics.path_follow_applied != 0;
  result.path_follow_skipped_blocked =
    diagnostics.path_follow_skipped_blocked != 0;
  result.path_follow_target_index = static_cast<std::size_t>(
    diagnostics.path_follow_target_index);
  result.path_follow_target_distance =
    diagnostics.path_follow_target_distance;
  return true;
}

std::unique_ptr<MppiCudaCritics> makeMppiCudaCritics()
{
  return std::make_unique<MppiCudaCritics>();
}

}  // namespace local_planner
