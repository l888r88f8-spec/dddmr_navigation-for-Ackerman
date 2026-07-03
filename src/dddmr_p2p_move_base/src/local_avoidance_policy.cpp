#include "p2p_move_base/local_avoidance_policy.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace p2p_move_base
{
namespace
{
double positiveOrInfinity(const double value)
{
  return value < 0.0 ? std::numeric_limits<double>::infinity() : value;
}

double clamp(const double value, const double min_value, const double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double directionSign(const AvoidanceDirection direction)
{
  if (direction == AvoidanceDirection::Left) {
    return 1.0;
  }
  if (direction == AvoidanceDirection::Right) {
    return -1.0;
  }
  return 0.0;
}
}  // namespace

LocalAvoidancePolicy::LocalAvoidancePolicy() = default;

LocalAvoidancePolicy::LocalAvoidancePolicy(const Config & config)
: config_(config)
{
}

void LocalAvoidancePolicy::setConfig(const Config & config)
{
  config_ = config;
}

const LocalAvoidancePolicy::Config & LocalAvoidancePolicy::config() const
{
  return config_;
}

void LocalAvoidancePolicy::reset()
{
  latched_direction_ = AvoidanceDirection::None;
  direction_hold_until_sec_ = 0.0;
  resetTakeover();
}

LocalAvoidanceDecision LocalAvoidancePolicy::update(
  std::shared_ptr<const LocalObstacleSnapshot> snapshot,
  const LocalAvoidanceCommand & nominal_command,
  const double now_sec)
{
  return update(snapshot, nominal_command, now_sec, false);
}

LocalAvoidanceDecision LocalAvoidancePolicy::update(
  std::shared_ptr<const LocalObstacleSnapshot> snapshot,
  const LocalAvoidanceCommand & nominal_command,
  const double now_sec,
  const bool planner_blocked)
{
  LocalAvoidanceDecision decision;
  decision.command = nominal_command;

  if (!config_.enabled || !snapshot || snapshot->perception_stale || snapshot->over_budget) {
    reset();
    decision.reason = "local_avoidance_inactive";
    return decision;
  }
  decision.snapshot_usable = true;

  const auto summary = summarizeObstacles(*snapshot);
  decision.min_forward_distance = summary.min_forward_distance;
  decision.left_clearance = summary.left_clearance;
  decision.right_clearance = summary.right_clearance;

  if (!summary.has_forward_obstacle) {
    if (planner_blocked && config_.takeover_enabled && takeover_active_ &&
      latched_direction_ != AvoidanceDirection::None)
    {
      if (config_.takeover_timeout_sec > 0.0 &&
        now_sec - takeover_started_sec_ > config_.takeover_timeout_sec)
      {
        decision.state = LocalAvoidanceState::Stop;
        decision.direction = latched_direction_;
        decision.speed_scale = 0.0;
        decision.command.linear_x = 0.0;
        decision.command.angular_z = 0.0;
        decision.reason = "local_takeover_timeout";
        return decision;
      }

      decision.state = LocalAvoidanceState::Takeover;
      decision.direction = latched_direction_;
      decision.speed_scale = 1.0;
      decision.command.linear_x = std::max(0.0, config_.takeover_linear_speed);
      decision.command.angular_z = directionSign(latched_direction_) * config_.max_angular_bias;
      decision.reason = "planner_blocked_takeover_continues_after_forward_clear";
      return decision;
    }

    reset();
    decision.reason = "forward_corridor_clear";
    return decision;
  }

  if (summary.min_forward_distance <= config_.stop_distance) {
    reset();
    decision.state = LocalAvoidanceState::Stop;
    decision.direction = AvoidanceDirection::None;
    decision.speed_scale = 0.0;
    decision.command.linear_x = 0.0;
    decision.command.angular_z = 0.0;
    decision.reason = "obstacle_inside_stop_distance";
    return decision;
  }

  const bool should_try_avoidance =
    (summary.min_forward_distance <= config_.avoidance_start_distance || planner_blocked) &&
    std::abs(nominal_command.linear_x) > 1e-4;
  if (should_try_avoidance || (planner_blocked && config_.takeover_enabled)) {
    const auto direction = chooseDirection(summary, now_sec);
    if (direction != AvoidanceDirection::None) {
      if (planner_blocked && config_.takeover_enabled) {
        if (!takeover_active_) {
          takeover_active_ = true;
          takeover_started_sec_ = now_sec;
        }
        if (config_.takeover_timeout_sec > 0.0 &&
          now_sec - takeover_started_sec_ > config_.takeover_timeout_sec)
        {
          decision.state = LocalAvoidanceState::Stop;
          decision.direction = direction;
          decision.speed_scale = 0.0;
          decision.command.linear_x = 0.0;
          decision.command.angular_z = 0.0;
          decision.reason = "local_takeover_timeout";
          return decision;
        }

        decision.state = LocalAvoidanceState::Takeover;
        decision.direction = direction;
        decision.speed_scale = 1.0;
        decision.command.linear_x = std::max(0.0, config_.takeover_linear_speed);
        decision.command.angular_z = directionSign(direction) * config_.max_angular_bias;
        decision.reason = "planner_blocked_local_takeover";
        return decision;
      }

      resetTakeover();
      decision.state = LocalAvoidanceState::LocalAvoidance;
      decision.direction = direction;
      decision.speed_scale = clamp(config_.avoidance_speed_scale, 0.0, 1.0);
      decision.command.linear_x = nominal_command.linear_x * decision.speed_scale;
      decision.command.angular_z =
        nominal_command.angular_z + directionSign(direction) * config_.max_angular_bias;
      decision.reason = "local_side_avoidance";
      return decision;
    }
  }

  decision.state = LocalAvoidanceState::Slowdown;
  resetTakeover();
  decision.speed_scale = computeSpeedScale(summary.min_forward_distance);
  decision.command.linear_x = nominal_command.linear_x * decision.speed_scale;
  decision.reason = "forward_obstacle_slowdown";
  return decision;
}

void LocalAvoidancePolicy::resetTakeover()
{
  takeover_active_ = false;
  takeover_started_sec_ = 0.0;
}

LocalAvoidancePolicy::ObstacleSummary LocalAvoidancePolicy::summarizeObstacles(
  const LocalObstacleSnapshot & snapshot) const
{
  ObstacleSummary summary;
  double min_forward = std::numeric_limits<double>::infinity();
  double min_left = std::numeric_limits<double>::infinity();
  double min_right = std::numeric_limits<double>::infinity();

  for (const auto & point : snapshot.obstacle_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    if (point.x < 0.0 || point.x > config_.forward_check_distance) {
      continue;
    }

    const double distance = std::hypot(point.x, point.y);
    if (std::abs(point.y) <= config_.corridor_half_width) {
      summary.has_forward_obstacle = true;
      min_forward = std::min(min_forward, distance);
    } else if (point.y > config_.corridor_half_width && point.y <= config_.side_probe_width) {
      min_left = std::min(min_left, distance);
    } else if (point.y < -config_.corridor_half_width && point.y >= -config_.side_probe_width) {
      min_right = std::min(min_right, distance);
    }
  }

  summary.min_forward_distance =
    std::isfinite(min_forward) ? min_forward : -1.0;
  summary.left_clearance =
    std::isfinite(min_left) ? min_left : config_.forward_check_distance;
  summary.right_clearance =
    std::isfinite(min_right) ? min_right : config_.forward_check_distance;
  return summary;
}

AvoidanceDirection LocalAvoidancePolicy::chooseDirection(
  const ObstacleSummary & summary,
  const double now_sec)
{
  const double left_clearance = positiveOrInfinity(summary.left_clearance);
  const double right_clearance = positiveOrInfinity(summary.right_clearance);
  AvoidanceDirection candidate = AvoidanceDirection::None;

  if (left_clearance > right_clearance + config_.side_switch_margin) {
    candidate = AvoidanceDirection::Left;
  } else if (right_clearance > left_clearance + config_.side_switch_margin) {
    candidate = AvoidanceDirection::Right;
  } else if (latched_direction_ != AvoidanceDirection::None) {
    candidate = latched_direction_;
  } else {
    candidate = left_clearance >= right_clearance ? AvoidanceDirection::Left : AvoidanceDirection::Right;
  }

  if (latched_direction_ != AvoidanceDirection::None &&
    candidate != latched_direction_ &&
    now_sec < direction_hold_until_sec_)
  {
    return latched_direction_;
  }

  if (candidate != AvoidanceDirection::None && candidate != latched_direction_) {
    latched_direction_ = candidate;
    direction_hold_until_sec_ = now_sec + std::max(0.0, config_.direction_hold_time_sec);
  }
  return candidate;
}

double LocalAvoidancePolicy::computeSpeedScale(const double min_forward_distance) const
{
  if (min_forward_distance < 0.0) {
    return 1.0;
  }
  if (min_forward_distance <= config_.stop_distance) {
    return 0.0;
  }
  if (min_forward_distance >= config_.slowdown_distance) {
    return 1.0;
  }

  const double denominator = std::max(1e-6, config_.slowdown_distance - config_.stop_distance);
  const double ratio = (min_forward_distance - config_.stop_distance) / denominator;
  return clamp(
    config_.min_speed_scale + ratio * (1.0 - config_.min_speed_scale),
    config_.min_speed_scale,
    1.0);
}

const char * toString(const LocalAvoidanceState state)
{
  switch (state) {
    case LocalAvoidanceState::Clear:
      return "clear";
    case LocalAvoidanceState::Slowdown:
      return "slowdown";
    case LocalAvoidanceState::LocalAvoidance:
      return "local_avoidance";
    case LocalAvoidanceState::Takeover:
      return "takeover";
    case LocalAvoidanceState::Stop:
      return "stop";
  }
  return "unknown";
}

const char * toString(const AvoidanceDirection direction)
{
  switch (direction) {
    case AvoidanceDirection::None:
      return "none";
    case AvoidanceDirection::Left:
      return "left";
    case AvoidanceDirection::Right:
      return "right";
  }
  return "unknown";
}

}  // namespace p2p_move_base
