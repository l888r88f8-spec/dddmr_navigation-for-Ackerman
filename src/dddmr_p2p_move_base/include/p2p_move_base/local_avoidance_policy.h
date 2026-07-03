#ifndef P2P_MOVE_BASE_LOCAL_AVOIDANCE_POLICY_H_
#define P2P_MOVE_BASE_LOCAL_AVOIDANCE_POLICY_H_

#include <memory>
#include <string>

#include "p2p_move_base/local_obstacle_layer.h"

namespace p2p_move_base
{

enum class LocalAvoidanceState
{
  Clear,
  Slowdown,
  LocalAvoidance,
  Takeover,
  Stop
};

enum class AvoidanceDirection
{
  None,
  Left,
  Right
};

struct LocalAvoidanceCommand
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

struct LocalAvoidanceDecision
{
  LocalAvoidanceState state = LocalAvoidanceState::Clear;
  AvoidanceDirection direction = AvoidanceDirection::None;
  LocalAvoidanceCommand command;
  double speed_scale = 1.0;
  double min_forward_distance = -1.0;
  double left_clearance = -1.0;
  double right_clearance = -1.0;
  bool snapshot_usable = false;
  std::string reason = "clear";
};

class LocalAvoidancePolicy
{
public:
  struct Config
  {
    bool enabled = false;
    double forward_check_distance = 4.0;
    double corridor_half_width = 0.35;
    double slowdown_distance = 3.0;
    double stop_distance = 0.75;
    double min_speed_scale = 0.25;
    double avoidance_start_distance = 2.0;
    double side_probe_width = 1.2;
    double side_switch_margin = 0.5;
    double direction_hold_time_sec = 1.0;
    double max_angular_bias = 0.20;
    double avoidance_speed_scale = 0.45;
    bool takeover_enabled = true;
    double takeover_timeout_sec = 4.0;
    double takeover_linear_speed = 0.12;
  };

  LocalAvoidancePolicy();
  explicit LocalAvoidancePolicy(const Config & config);

  void setConfig(const Config & config);
  const Config & config() const;
  void reset();

  LocalAvoidanceDecision update(
    std::shared_ptr<const LocalObstacleSnapshot> snapshot,
    const LocalAvoidanceCommand & nominal_command,
    double now_sec);
  LocalAvoidanceDecision update(
    std::shared_ptr<const LocalObstacleSnapshot> snapshot,
    const LocalAvoidanceCommand & nominal_command,
    double now_sec,
    bool planner_blocked);

private:
  struct ObstacleSummary
  {
    bool has_forward_obstacle = false;
    double min_forward_distance = -1.0;
    double left_clearance = -1.0;
    double right_clearance = -1.0;
  };

  ObstacleSummary summarizeObstacles(const LocalObstacleSnapshot & snapshot) const;
  AvoidanceDirection chooseDirection(const ObstacleSummary & summary, double now_sec);
  double computeSpeedScale(double min_forward_distance) const;
  void resetTakeover();

  Config config_;
  AvoidanceDirection latched_direction_ = AvoidanceDirection::None;
  double direction_hold_until_sec_ = 0.0;
  bool takeover_active_ = false;
  double takeover_started_sec_ = 0.0;
};

const char * toString(LocalAvoidanceState state);
const char * toString(AvoidanceDirection direction);

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_LOCAL_AVOIDANCE_POLICY_H_
