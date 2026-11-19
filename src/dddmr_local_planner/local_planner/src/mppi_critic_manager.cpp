#include "local_planner/mppi_critic_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner
{

bool isMppiTrajectoryCollisionFree(
  const MppiScoredTrajectory & trajectory,
  const bool include_probe)
{
  const bool time_channel_clear =
    !trajectory.points.empty() &&
    trajectory.points.size() == trajectory.controls.size() &&
    std::all_of(
    trajectory.points.begin(),
    trajectory.points.end(),
    [](const MppiTrajectoryPoint & point) {
      return point.collision_evaluated && !point.in_collision;
    });
  if(!time_channel_clear){
    return false;
  }
  if(!include_probe){
    // Callers that validate the command actually sent must not re-use the
    // hypothetical arc: it is a selection heuristic, and treating it as a
    // certificate escalates every corridor to a global replan.
    return true;
  }
  // The distance probe is the only evidence beyond the time rollout. A probe
  // contact must veto the candidate here as well, because this predicate is the
  // hard filter for side selection, the committed-bypass guard and the reverse
  // fallback -- not just the soft obstacle cost.
  return std::all_of(
    trajectory.probe_points.begin(),
    trajectory.probe_points.end(),
    [](const MppiTrajectoryPoint & point) {
      return point.collision_evaluated && !point.in_collision;
    });
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

MppiPathProjection MppiBypassSideClassifier::projectPoint(
  const double x,
  const double y) const
{
  if(!path_valid_ || !std::isfinite(x) || !std::isfinite(y)){
    return MppiPathProjection{};
  }

  double closest_distance_squared = std::numeric_limits<double>::infinity();
  double closest_signed_offset = 0.0;
  for(const auto & segment : segments_){
    const double projection = std::clamp(
      ((x - segment.start_x) * segment.dx +
      (y - segment.start_y) * segment.dy) *
      segment.inverse_length * segment.inverse_length,
      0.0,
      1.0);
    const double projected_x = segment.start_x + projection * segment.dx;
    const double projected_y = segment.start_y + projection * segment.dy;
    const double offset_x = x - projected_x;
    const double offset_y = y - projected_y;
    const double distance_squared = offset_x * offset_x + offset_y * offset_y;
    const double signed_offset =
      (segment.dx * offset_y - segment.dy * offset_x) * segment.inverse_length;

    if(!std::isfinite(distance_squared) || !std::isfinite(signed_offset)){
      return MppiPathProjection{};
    }
    if(distance_squared < closest_distance_squared){
      closest_distance_squared = distance_squared;
      closest_signed_offset = signed_offset;
    }
  }

  return MppiPathProjection{closest_signed_offset, true};
}

MppiPathProjection projectPointOntoMppiPath(
  const std::vector<MppiPathPoint> & path,
  const double x,
  const double y)
{
  return MppiBypassSideClassifier(path).projectPoint(x, y);
}

namespace
{

// Longest contiguous run of free lateral probe cells. Each sample represents
// a vehicle-centre position, so a run of k samples gives the centre roughly k
// steps of lateral freedom. The start index is the innermost sample of that
// run, which is where the usable band begins relative to the route.
struct FreeRun
{
  std::size_t length = 0;
  std::size_t start = 0;
};

FreeRun longestFreeRun(const std::vector<bool> & blocked)
{
  FreeRun longest;
  std::size_t run = 0;
  for(std::size_t i = 0; i < blocked.size(); ++i){
    if(blocked[i]){
      run = 0;
      continue;
    }
    ++run;
    if(run > longest.length){
      longest.length = run;
      longest.start = i + 1 - run;
    }
  }
  return longest;
}

}  // namespace

MppiLateralOccupancyProfile measureMppiLateralOccupancyProfile(
  const std::vector<MppiPathPoint> & path,
  const MppiBlockedProbe & probe_blocked,
  const MppiLateralProfileConfig & config)
{
  MppiLateralOccupancyProfile profile;
  if(!probe_blocked || path.empty() || config.max_lateral_offset <= 0.0 ||
    config.lateral_step <= 0.0 || config.max_blocked_points == 0)
  {
    return profile;
  }

  std::vector<std::size_t> blocked_indices;
  for(std::size_t i = 0; i < path.size(); ++i){
    if(!path[i].obstacle_free){
      blocked_indices.push_back(i);
    }
  }
  if(blocked_indices.empty()){
    return profile;
  }

  const std::size_t stride = std::max<std::size_t>(
    1, (blocked_indices.size() + config.max_blocked_points - 1) /
    config.max_blocked_points);
  const int samples_per_side = std::max(
    0, static_cast<int>(std::floor(
      config.max_lateral_offset / config.lateral_step + 1e-9)));

  double offset_sum = 0.0;
  std::size_t offset_count = 0;
  for(std::size_t k = 0; k < blocked_indices.size(); k += stride){
    const auto & anchor = path[blocked_indices[k]];
    if(!std::isfinite(anchor.x) || !std::isfinite(anchor.y) ||
      !std::isfinite(anchor.yaw))
    {
      continue;
    }
    // Left normal of the route direction.
    const double normal_x = -std::sin(anchor.yaw);
    const double normal_y = std::cos(anchor.yaw);
    // Tangential direction, used to sweep the probe band over the footprint
    // that extends ahead of and behind the anchor.
    const double tangent_x = std::cos(anchor.yaw);
    const double tangent_y = std::sin(anchor.yaw);
    const int longitudinal_samples =
      (config.max_longitudinal_offset > 0.0 && config.longitudinal_step > 0.0) ?
      static_cast<int>(std::lround(
        config.max_longitudinal_offset / config.longitudinal_step)) : 0;
    std::vector<bool> left_blocked(
      static_cast<std::size_t>(samples_per_side), false);
    std::vector<bool> right_blocked(
      static_cast<std::size_t>(samples_per_side), false);
    for(double offset = -config.max_lateral_offset;
      offset <= config.max_lateral_offset + 1e-9;
      offset += config.lateral_step)
    {
      if(std::fabs(offset) < config.lateral_step * 0.5){
        continue;
      }
      MppiPathPoint probe = anchor;
      probe.x = anchor.x + normal_x * offset;
      probe.y = anchor.y + normal_y * offset;
      bool blocked = probe_blocked(probe);
      for(int step = 1; !blocked && step <= longitudinal_samples; ++step){
        for(int direction = -1; direction <= 1 && !blocked; direction += 2){
          MppiPathPoint along = probe;
          along.x = probe.x +
            tangent_x * static_cast<double>(direction * step) * config.longitudinal_step;
          along.y = probe.y +
            tangent_y * static_cast<double>(direction * step) * config.longitudinal_step;
          blocked = probe_blocked(along);
        }
      }
      if(!blocked){
        continue;
      }
      offset_sum += offset;
      ++offset_count;
      const int side_index = static_cast<int>(std::lround(
        std::fabs(offset) / config.lateral_step)) - 1;
      if(side_index >= 0 && side_index < samples_per_side){
        if(offset > 0.0){
          left_blocked[static_cast<std::size_t>(side_index)] = true;
        }
        else{
          right_blocked[static_cast<std::size_t>(side_index)] = true;
        }
      }
      if(offset > 0.0){
        profile.left_clearance = std::min(profile.left_clearance, offset);
      }
      else{
        profile.right_clearance = std::min(profile.right_clearance, -offset);
      }
    }
    // The free span on the pass side is what the vehicle can actually use;
    // take the narrowest anchor so a pinch point anywhere cannot be averaged
    // away by wider sections of the blocked run. Keep the band edge from the
    // same anchor so the detour cost measures the room this scan actually saw.
    const FreeRun left_run = longestFreeRun(left_blocked);
    const FreeRun right_run = longestFreeRun(right_blocked);
    const double left_span =
      static_cast<double>(left_run.length) * config.lateral_step;
    if(left_span < profile.left_free_span){
      profile.left_free_span = left_span;
      profile.left_band_offset = left_run.length == 0 ?
        std::numeric_limits<double>::quiet_NaN() :
        static_cast<double>(left_run.start + 1) * config.lateral_step;
    }
    const double right_span =
      static_cast<double>(right_run.length) * config.lateral_step;
    if(right_span < profile.right_free_span){
      profile.right_free_span = right_span;
      profile.right_band_offset = right_run.length == 0 ?
        std::numeric_limits<double>::quiet_NaN() :
        -static_cast<double>(right_run.start + 1) * config.lateral_step;
    }
  }

  if(offset_count == 0){
    return profile;
  }
  profile.valid = true;
  profile.obstacle_offset = offset_sum / static_cast<double>(offset_count);
  return profile;
}

MppiBypassSide selectMppiGeometricBypassSide(
  const MppiLateralOccupancyProfile & profile,
  const double min_decisive_offset,
  const double required_width)
{
  if(!profile.valid || !std::isfinite(profile.obstacle_offset)){
    return MppiBypassSide::Neutral;
  }
  // An obstacle centred on the route blocks both sides; a tiny asymmetry is
  // measurement noise, not a reason to commit to a side.
  if(std::fabs(profile.obstacle_offset) < std::max(0.0, min_decisive_offset)){
    return MppiBypassSide::Neutral;
  }
  const bool obstacle_on_left = profile.obstacle_offset > 0.0;
  const MppiBypassSide side =
    obstacle_on_left ? MppiBypassSide::Right : MppiBypassSide::Left;
  // The pass side needs a contiguous free lateral band, not merely distance
  // from the centreline to the obstacle influence: an obstacle whose footprint
  // crosses the centreline can still leave a wide free band farther out.
  const double pass_free_span =
    obstacle_on_left ? profile.right_free_span : profile.left_free_span;
  // Infinity means nothing was observed in the sampled band on that side.
  if(std::isfinite(pass_free_span) && pass_free_span < required_width){
    return MppiBypassSide::Neutral;
  }
  return side;
}

double mppiBypassBandDetour(
  const MppiLateralOccupancyProfile & profile,
  const MppiBypassSide side,
  const double vehicle_offset)
{
  if(!profile.valid || !std::isfinite(vehicle_offset)){
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Bands are signed: the left band runs outward from its inner edge, the
  // right band inward (towards more negative offsets).
  double low = 0.0;
  double high = 0.0;
  if(side == MppiBypassSide::Left){
    if(!std::isfinite(profile.left_band_offset) ||
      !std::isfinite(profile.left_free_span))
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    low = profile.left_band_offset;
    high = low + profile.left_free_span;
  }
  else if(side == MppiBypassSide::Right){
    if(!std::isfinite(profile.right_band_offset) ||
      !std::isfinite(profile.right_free_span))
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    high = profile.right_band_offset;
    low = high - profile.right_free_span;
  }
  else{
    return std::numeric_limits<double>::quiet_NaN();
  }
  if(vehicle_offset < low){
    return low - vehicle_offset;
  }
  if(vehicle_offset > high){
    return vehicle_offset - high;
  }
  return 0.0;
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
    const auto projection = projectPoint(trajectory_point.x, trajectory_point.y);
    if(!projection.valid){
      return MppiBypassSide::Neutral;
    }
    const double closest_signed_offset = projection.signed_offset;
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
      score.bypass_side = MppiBypassSide::Neutral;
      score.collision_free = false;
      score.signed_lateral_progress = std::numeric_limits<double>::quiet_NaN();
    }
    return false;
  }

  const MppiBypassSideClassifier classifier(data.path);
  const auto start = classifier.projectPoint(data.robot_x, data.robot_y);
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    auto & score = result->scores[i];
    score.signed_lateral_progress = std::numeric_limits<double>::quiet_NaN();
    if(!score.valid){
      score.bypass_side = MppiBypassSide::Neutral;
      score.collision_free = false;
      continue;
    }
    score.bypass_side = classifier.classify(data.trajectories[i]);
    score.collision_free = isMppiTrajectoryCollisionFree(data.trajectories[i]);
    const auto & points = data.trajectories[i].points;
    if(start.valid && score.collision_free &&
      std::isfinite(data.model_dt) && data.model_dt > 0.0)
    {
      // Rollout point zero is already one model step into the future. Use the
      // actual robot pose as the origin, not the first predicted position.
      const double horizon_steps = std::min(
        static_cast<double>(points.size()), std::max(1.0, std::ceil(1.0 / data.model_dt)));
      const auto index = static_cast<std::size_t>(horizon_steps) - 1;
      const auto future = classifier.projectPoint(points[index].x, points[index].y);
      if(future.valid){
        score.signed_lateral_progress = future.signed_offset - start.signed_offset;
      }
    }
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
  result.term_costs.assign(data.trajectories.size(), MppiCriticTermCosts{});

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
    if(i < result->term_costs.size()){
      result->term_costs[i].constraint += poweredCost(
        cost * std::max(1e-3, data.model_dt),
        config_.constraint.cost_weight,
        config_.constraint.cost_power);
    }
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

    for(const auto & probe : trajectory.probe_points){
      // Distance-parameterized continuation: collision-only evidence, so a
      // candidate that would reach the obstacle beyond the time rollout is
      // still vetoed instead of scoring as free.
      if(probe.in_collision){
        trajectory_collides = true;
        break;
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
    if(i < result->term_costs.size()){
      result->term_costs[i].obstacles +=
        poweredCost(total, 1.0, config_.obstacles.cost_power);
    }
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
    if(i < result->term_costs.size()){
      result->term_costs[i].goal += poweredCost(
        cost, config_.goal.cost_weight, config_.goal.cost_power);
    }
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
    if(i < result->term_costs.size()){
      result->term_costs[i].goal_angle += poweredCost(
        cost, config_.goal_angle.cost_weight, config_.goal_angle.cost_power);
    }
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
    result->path_align_skipped_blocked = true;
    return;
  }

  const std::size_t furthest = furthestReachedPathIndex(data);
  if(furthest < config_.path_align.offset_from_furthest){
    return;
  }

  const std::size_t step = std::max<std::size_t>(1, config_.path_align.trajectory_point_step);
  result->path_align_applied = true;
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
    if(i < result->term_costs.size()){
      result->term_costs[i].path_align += poweredCost(
        cost, config_.path_align.cost_weight, config_.path_align.cost_power);
    }
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
  MppiPathPoint target;
  if(target_index < data.path.size()){
    target = data.path[target_index];
  }
  else{
    // The scan ran off the far end of the window because the blocked region
    // reaches the end of the local path. Returning here silently removes the
    // only critic that rewards forward progress: PathAlign is already gated off
    // by the blocked-path occupancy check. The averaged obstacle repulsion is
    // then the only motion-dependent term left, and it is lowest for the
    // trajectory that travels least, so a near-stationary sample becomes
    // strictly cheapest and the vehicle creeps at vx_min until the geometry
    // closes and reverse recovery has to unwedge it. Keep a progress target by
    // extrapolating the route past the window end along its final heading.
    // No target is invented when the route directly ahead of the vehicle is
    // blocked as well: a fully closed corridor has nothing to progress toward.
    const std::size_t robot_index =
      nearestPathIndex(data.path, data.robot_x, data.robot_y, 0);
    double free_route_ahead = 0.0;
    std::size_t free_index = robot_index;
    while(data.path[free_index].obstacle_free &&
      free_index + 1 < data.path.size() && data.path[free_index + 1].obstacle_free)
    {
      free_route_ahead += distance(
        data.path[free_index].x, data.path[free_index].y,
        data.path[free_index + 1].x, data.path[free_index + 1].y);
      ++free_index;
    }
    if(!data.path[robot_index].obstacle_free ||
      free_route_ahead < config_.path_follow.threshold_to_consider)
    {
      // The route ahead of the vehicle is closed, so there is no progress
      // target to extrapolate. Recorded so a creep can be told apart from a
      // cost balance that deliberately chose to decelerate.
      result->path_follow_skipped_blocked = true;
      return;
    }

    target = data.path.back();
    double tangent_x = 0.0;
    double tangent_y = 0.0;
    if(data.path.size() >= 2){
      const auto & before = data.path[data.path.size() - 2];
      tangent_x = data.path.back().x - before.x;
      tangent_y = data.path.back().y - before.y;
    }
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    const double spacing = tangent_length > 1e-6 ? tangent_length : 0.1;
    if(tangent_length > 1e-6){
      tangent_x /= tangent_length;
      tangent_y /= tangent_length;
    }
    else{
      tangent_x = std::cos(data.path.back().yaw);
      tangent_y = std::sin(data.path.back().yaw);
    }
    const double extension =
      static_cast<double>(std::max<std::size_t>(
        1, config_.path_follow.offset_from_furthest)) * spacing;
    target.x += tangent_x * extension;
    target.y += tangent_y * extension;
  }
  for(std::size_t i = 0; i < data.trajectories.size(); ++i){
    if(data.trajectories[i].points.empty()){
      continue;
    }
    const auto & last = data.trajectories[i].points.back();
    result->scores[i].cost += poweredCost(
      distance(last.x, last.y, target.x, target.y),
      config_.path_follow.cost_weight,
      config_.path_follow.cost_power);
    if(i < result->term_costs.size()){
      result->term_costs[i].path_follow += poweredCost(
        distance(last.x, last.y, target.x, target.y),
        config_.path_follow.cost_weight,
        config_.path_follow.cost_power);
    }
  }
  result->path_follow_applied = true;
  result->path_follow_target_index = target_index;
  result->path_follow_target_distance =
    distance(data.robot_x, data.robot_y, target.x, target.y);
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
    if(i < result->term_costs.size()){
      result->term_costs[i].path_angle += poweredCost(
        std::fabs(shortestAngularDistance(last.yaw, terminal_to_target)),
        config_.path_angle.cost_weight,
        config_.path_angle.cost_power);
    }
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
    if(i < result->term_costs.size()){
      result->term_costs[i].prefer_forward += poweredCost(
        reverse_motion,
        config_.prefer_forward.cost_weight,
        config_.prefer_forward.cost_power);
    }
  }
}

}  // namespace local_planner
