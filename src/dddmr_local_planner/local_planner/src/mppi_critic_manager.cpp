#include "local_planner/mppi_critic_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner
{

bool isMppiTrajectoryCollisionFree(const MppiScoredTrajectory & trajectory)
{
  return !trajectory.points.empty() &&
    trajectory.points.size() == trajectory.controls.size() &&
    std::none_of(
    trajectory.points.begin(),
    trajectory.points.end(),
    [](const MppiTrajectoryPoint & point) {return point.in_collision;});
}

MppiBypassSideClassifier::MppiBypassSideClassifier(const std::vector<MppiPathPoint> & path)
{
  if(path.size() < 2){
    return;
  }

  for(const auto & path_point : path){
    if(!std::isfinite(path_point.x) || !std::isfinite(path_point.y)){
      return;
    }
  }

  segments_.reserve(path.size() - 1);
  for(std::size_t i = 1; i < path.size(); ++i){
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length = std::hypot(dx, dy);
    if(!std::isfinite(length)){
      segments_.clear();
      return;
    }
    if(length == 0.0){
      continue;
    }
    segments_.push_back(PathSegment{
      path[i - 1].x,
      path[i - 1].y,
      dx,
      dy,
      1.0 / length});
  }
  path_valid_ = !segments_.empty();
}

MppiBypassSide MppiBypassSideClassifier::classify(
  const MppiScoredTrajectory & trajectory) const
{
  if(!path_valid_ || trajectory.points.empty() ||
    trajectory.points.size() != trajectory.controls.size())
  {
    return MppiBypassSide::Neutral;
  }

  double peak_offset = 0.0;
  double terminal_offset = 0.0;
  for(const auto & trajectory_point : trajectory.points){
    if(!std::isfinite(trajectory_point.x) || !std::isfinite(trajectory_point.y)){
      return MppiBypassSide::Neutral;
    }

    double closest_distance_squared = std::numeric_limits<double>::infinity();
    double closest_signed_offset = 0.0;
    for(const auto & segment : segments_){
      const double projection = std::clamp(
        ((trajectory_point.x - segment.start_x) * segment.dx +
        (trajectory_point.y - segment.start_y) * segment.dy) *
        segment.inverse_length * segment.inverse_length,
        0.0,
        1.0);
      const double projected_x = segment.start_x + projection * segment.dx;
      const double projected_y = segment.start_y + projection * segment.dy;
      const double offset_x = trajectory_point.x - projected_x;
      const double offset_y = trajectory_point.y - projected_y;
      const double distance_squared = offset_x * offset_x + offset_y * offset_y;
      const double signed_offset =
        (segment.dx * offset_y - segment.dy * offset_x) * segment.inverse_length;

      if(!std::isfinite(distance_squared) || !std::isfinite(signed_offset)){
        return MppiBypassSide::Neutral;
      }
      if(distance_squared < closest_distance_squared){
        closest_distance_squared = distance_squared;
        closest_signed_offset = signed_offset;
      }
    }

    if(!std::isfinite(closest_signed_offset)){
      return MppiBypassSide::Neutral;
    }
    if(std::fabs(closest_signed_offset) > std::fabs(peak_offset)){
      peak_offset = closest_signed_offset;
    }
    terminal_offset = closest_signed_offset;
  }

  const double metric = 0.7 * peak_offset + 0.3 * terminal_offset;
  if(!std::isfinite(metric)){
    return MppiBypassSide::Neutral;
  }
  if(metric > 0.10){
    return MppiBypassSide::Left;
  }
  if(metric < -0.10){
    return MppiBypassSide::Right;
  }
  return MppiBypassSide::Neutral;
}

MppiBypassSide classifyMppiBypassSide(
  const MppiScoredTrajectory & trajectory,
  const std::vector<MppiPathPoint> & path)
{
  return MppiBypassSideClassifier(path).classify(trajectory);
}

bool annotateMppiCandidateMetadata(
  const MppiCriticData & data,
  MppiCriticResult * const result)
{
  if(result == nullptr){
    return false;
  }
  if(result->scores.size() != data.trajectories.size()){
    for(auto & score : result->scores){
      score.valid = false;
    }
    return false;
  }

  const MppiBypassSideClassifier classifier(data.path);
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    result->scores[i].bypass_side = classifier.classify(data.trajectories[i]);
    result->scores[i].collision_free =
      isMppiTrajectoryCollisionFree(data.trajectories[i]);
  }
  return true;
}

MppiConstraintCriticConfig::MppiConstraintCriticConfig()
{
  cost_weight = 4.0;
}

MppiGoalCriticConfig::MppiGoalCriticConfig()
{
  cost_weight = 5.0;
}

MppiGoalAngleCriticConfig::MppiGoalAngleCriticConfig()
{
  cost_weight = 3.0;
}

MppiPathAlignCriticConfig::MppiPathAlignCriticConfig()
{
  cost_weight = 10.0;
}

MppiPathFollowCriticConfig::MppiPathFollowCriticConfig()
{
  cost_weight = 5.0;
}

MppiPathAngleCriticConfig::MppiPathAngleCriticConfig()
{
  cost_weight = 2.2;
}

MppiPreferForwardCriticConfig::MppiPreferForwardCriticConfig()
{
  cost_weight = 5.0;
}

void MppiCriticManager::setConfig(const MppiCriticConfig & config)
{
  config_ = config;
}

const MppiCriticConfig & MppiCriticManager::config() const
{
  return config_;
}

MppiCriticResult MppiCriticManager::score(const MppiCriticData & data) const
{
  MppiCriticResult result;
  result.scores.assign(data.trajectories.size(), MppiCandidateScore{true, 0.0});

  for(const auto & critic : config_.critics){
    if(result.fail_flag){
      break;
    }
    if(critic == "ConstraintCritic"){
      scoreConstraintCritic(data, &result);
    }
    else if(critic == "ObstaclesCritic"){
      scoreObstaclesCritic(data, &result);
    }
    else if(critic == "GoalCritic"){
      scoreGoalCritic(data, &result);
    }
    else if(critic == "GoalAngleCritic"){
      scoreGoalAngleCritic(data, &result);
    }
    else if(critic == "PathAlignCritic"){
      scorePathAlignCritic(data, &result);
    }
    else if(critic == "PathFollowCritic"){
      scorePathFollowCritic(data, &result);
    }
    else if(critic == "PathAngleCritic"){
      scorePathAngleCritic(data, &result);
    }
    else if(critic == "PreferForwardCritic"){
      scorePreferForwardCritic(data, &result);
    }
  }

  return result;
}

double MppiCriticManager::poweredCost(
  const double value,
  const double weight,
  const int power) const
{
  const double weighted = std::max(0.0, value) * weight;
  if(power <= 1){
    return weighted;
  }
  return std::pow(weighted, static_cast<double>(power));
}

double MppiCriticManager::distance(
  const double x0,
  const double y0,
  const double x1,
  const double y1) const
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

double MppiCriticManager::shortestAngularDistance(const double from, const double to) const
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double MppiCriticManager::pathLength(const std::vector<MppiPathPoint> & path) const
{
  double length = 0.0;
  for(std::size_t i = 1; i < path.size(); ++i){
    length += distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
  }
  return length;
}

std::size_t MppiCriticManager::nearestPathIndex(
  const std::vector<MppiPathPoint> & path,
  const double x,
  const double y,
  const std::size_t start_index) const
{
  if(path.empty()){
    return 0;
  }

  std::size_t nearest = std::min(start_index, path.size() - 1);
  double best = std::numeric_limits<double>::infinity();
  for(std::size_t i = nearest; i < path.size(); ++i){
    const double d = distance(x, y, path[i].x, path[i].y);
    if(d < best){
      best = d;
      nearest = i;
    }
  }
  return nearest;
}

std::size_t MppiCriticManager::furthestReachedPathIndex(const MppiCriticData & data) const
{
  std::size_t furthest = 0;
  for(const auto & trajectory : data.trajectories){
    std::size_t search_from = 0;
    for(const auto & point : trajectory.points){
      search_from = nearestPathIndex(data.path, point.x, point.y, search_from);
      furthest = std::max(furthest, search_from);
    }
  }
  return furthest;
}

void MppiCriticManager::scoreConstraintCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.constraint.enabled || result == nullptr){
    return;
  }

  const double min_turning_radius = std::max(0.0, data.min_turning_radius);
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    double cost = 0.0;
    for(const auto & control : data.trajectories[i].controls){
      cost += std::max(0.0, control.vx - data.vx_max);
      cost += std::max(0.0, data.vx_min - control.vx);
      if(min_turning_radius > 0.0 && std::fabs(control.wz) > 1e-6){
        cost += std::max(0.0, min_turning_radius - std::fabs(control.vx / control.wz));
      }
    }
    result->scores[i].cost += poweredCost(
      cost * std::max(1e-3, data.model_dt),
      config_.constraint.cost_weight,
      config_.constraint.cost_power);
  }
}

void MppiCriticManager::scoreObstaclesCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.obstacles.enabled || result == nullptr || data.trajectories.empty()){
    return;
  }

  const bool near_goal =
    (data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path)) <
    config_.obstacles.near_goal_distance;
  bool all_trajectories_collide = true;

  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    const auto & trajectory = data.trajectories[i];
    bool trajectory_collides = false;
    double raw_cost = 0.0;
    double repulsive_cost = 0.0;
    std::size_t evaluated_clearance_points = 0;

    for(const auto & point : trajectory.points){
      if(point.in_collision){
        trajectory_collides = true;
        break;
      }
      if(!std::isfinite(point.obstacle_clearance)){
        continue;
      }
      ++evaluated_clearance_points;
      if(point.obstacle_clearance < config_.obstacles.collision_margin_distance){
        raw_cost += config_.obstacles.collision_margin_distance - point.obstacle_clearance;
      }
      if(!near_goal && point.obstacle_clearance < config_.obstacles.inflation_radius){
        repulsive_cost += config_.obstacles.inflation_radius - point.obstacle_clearance;
      }
    }

    if(!trajectory_collides){
      all_trajectories_collide = false;
    }

    const double normalizer = std::max<std::size_t>(1, evaluated_clearance_points);
    const double total =
      config_.obstacles.critical_weight *
        (trajectory_collides ? config_.obstacles.collision_cost : raw_cost) +
      config_.obstacles.repulsion_weight * repulsive_cost / normalizer;
    result->scores[i].cost += poweredCost(total, 1.0, config_.obstacles.cost_power);
  }

  result->fail_flag = all_trajectories_collide;
}

void MppiCriticManager::scoreGoalCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.goal.enabled || result == nullptr || data.path.empty()){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length > config_.goal.threshold_to_consider){
    return;
  }

  const auto & goal = data.path.back();
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    double cost = 0.0;
    for(const auto & point : data.trajectories[i].points){
      cost += distance(point.x, point.y, goal.x, goal.y);
    }
    cost /= std::max<std::size_t>(1, data.trajectories[i].points.size());
    result->scores[i].cost += poweredCost(
      cost,
      config_.goal.cost_weight,
      config_.goal.cost_power);
  }
}

void MppiCriticManager::scoreGoalAngleCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.goal_angle.enabled || result == nullptr || data.path.empty()){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length > config_.goal_angle.threshold_to_consider){
    return;
  }

  const auto & goal = data.path.back();
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    double cost = 0.0;
    for(const auto & point : data.trajectories[i].points){
      double angular_distance = std::fabs(shortestAngularDistance(point.yaw, goal.yaw));
      if(config_.goal_angle.symmetric_yaw_tolerance){
        angular_distance = std::min(
          angular_distance,
          std::fabs(shortestAngularDistance(point.yaw, goal.yaw + M_PI)));
      }
      cost += angular_distance;
    }
    cost /= std::max<std::size_t>(1, data.trajectories[i].points.size());
    result->scores[i].cost += poweredCost(
      cost,
      config_.goal_angle.cost_weight,
      config_.goal_angle.cost_power);
  }
}

void MppiCriticManager::scorePathAlignCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.path_align.enabled || result == nullptr || data.path.size() < 2){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length < config_.path_align.threshold_to_consider){
    return;
  }
  if(data.blocked_path_points >= 3 &&
     data.path_obstacle_occupancy_ratio > config_.path_align.max_path_occupancy_ratio)
  {
    return;
  }

  const std::size_t furthest = furthestReachedPathIndex(data);
  if(furthest < config_.path_align.offset_from_furthest){
    return;
  }

  const std::size_t step = std::max<std::size_t>(1, config_.path_align.trajectory_point_step);
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    const auto & trajectory = data.trajectories[i];
    double cost = 0.0;
    std::size_t count = 0;
    std::size_t path_search_from = 0;
    for(std::size_t p = 0; p < trajectory.points.size(); p += step){
      const auto & point = trajectory.points[p];
      path_search_from = nearestPathIndex(data.path, point.x, point.y, path_search_from);
      const auto & path_point = data.path[path_search_from];
      cost += distance(point.x, point.y, path_point.x, path_point.y);
      if(config_.path_align.use_path_orientations){
        cost += std::fabs(shortestAngularDistance(point.yaw, path_point.yaw));
      }
      ++count;
    }
    cost /= std::max<std::size_t>(1, count);
    result->scores[i].cost += poweredCost(
      cost,
      config_.path_align.cost_weight,
      config_.path_align.cost_power);
  }
}

void MppiCriticManager::scorePathFollowCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.path_follow.enabled || result == nullptr || data.path.empty()){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length < config_.path_follow.threshold_to_consider){
    return;
  }

  std::size_t target_index = std::min(
    furthestReachedPathIndex(data) + config_.path_follow.offset_from_furthest,
    data.path.size() - 1);
  while(target_index < data.path.size() && !data.path[target_index].obstacle_free){
    ++target_index;
  }
  if(target_index >= data.path.size()){
    return;
  }
  const auto & target = data.path[target_index];
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    if(data.trajectories[i].points.empty()){
      continue;
    }
    const auto & last = data.trajectories[i].points.back();
    result->scores[i].cost += poweredCost(
      distance(last.x, last.y, target.x, target.y),
      config_.path_follow.cost_weight,
      config_.path_follow.cost_power);
  }
}

void MppiCriticManager::scorePathAngleCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.path_angle.enabled || result == nullptr || data.path.empty()){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length < config_.path_angle.threshold_to_consider){
    return;
  }

  const std::size_t target_index = std::min(
    furthestReachedPathIndex(data) + config_.path_angle.offset_from_furthest,
    data.path.size() - 1);
  const auto & target = data.path[target_index];
  const double robot_to_target =
    std::atan2(target.y - data.robot_y, target.x - data.robot_x);
  if(std::fabs(shortestAngularDistance(data.robot_yaw, robot_to_target)) <
     config_.path_angle.max_angle_to_furthest)
  {
    return;
  }

  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    if(data.trajectories[i].points.empty()){
      continue;
    }
    const auto & last = data.trajectories[i].points.back();
    const double terminal_to_target = std::atan2(target.y - last.y, target.x - last.x);
    result->scores[i].cost += poweredCost(
      std::fabs(shortestAngularDistance(last.yaw, terminal_to_target)),
      config_.path_angle.cost_weight,
      config_.path_angle.cost_power);
  }
}

void MppiCriticManager::scorePreferForwardCritic(
  const MppiCriticData & data,
  MppiCriticResult * result) const
{
  if(!config_.prefer_forward.enabled || result == nullptr){
    return;
  }
  const double local_path_length =
    data.local_path_length > 0.0 ? data.local_path_length : pathLength(data.path);
  if(local_path_length < config_.prefer_forward.threshold_to_consider){
    return;
  }

  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    double reverse_motion = 0.0;
    for(const auto & control : data.trajectories[i].controls){
      reverse_motion += std::max(0.0, -control.vx) * std::max(1e-3, data.model_dt);
    }
    result->scores[i].cost += poweredCost(
      reverse_motion,
      config_.prefer_forward.cost_weight,
      config_.prefer_forward.cost_power);
  }
}

}  // namespace local_planner
