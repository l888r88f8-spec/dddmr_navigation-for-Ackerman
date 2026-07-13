#ifndef LOCAL_PLANNER_MPPI_CRITIC_MANAGER_H_
#define LOCAL_PLANNER_MPPI_CRITIC_MANAGER_H_

#include <string>
#include <vector>

#include "local_planner/mppi_controller.h"

namespace local_planner
{

struct MppiTrajectoryPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool in_collision = false;
  double obstacle_clearance = 0.0;
};

struct MppiPathPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool obstacle_free = true;
};

struct MppiScoredTrajectory
{
  std::vector<MppiControl> controls;
  std::vector<MppiTrajectoryPoint> points;
};

bool isMppiTrajectoryCollisionFree(const MppiScoredTrajectory & trajectory);

struct MppiCriticTermConfig
{
  bool enabled = true;
  int cost_power = 1;
  double cost_weight = 1.0;
};

struct MppiConstraintCriticConfig : public MppiCriticTermConfig
{
  MppiConstraintCriticConfig();
};

struct MppiObstaclesCriticConfig
{
  bool enabled = true;
  int cost_power = 1;
  double critical_weight = 20.0;
  double repulsion_weight = 1.5;
  double collision_cost = 100000.0;
  double collision_margin_distance = 0.10;
  double near_goal_distance = 0.5;
  double inflation_radius = 0.9;
  std::size_t trajectory_point_step = 3;
};

struct MppiGoalCriticConfig : public MppiCriticTermConfig
{
  MppiGoalCriticConfig();
  double threshold_to_consider = 1.4;
};

struct MppiGoalAngleCriticConfig : public MppiCriticTermConfig
{
  MppiGoalAngleCriticConfig();
  double threshold_to_consider = 0.5;
  bool symmetric_yaw_tolerance = false;
};

struct MppiPathAlignCriticConfig : public MppiCriticTermConfig
{
  MppiPathAlignCriticConfig();
  double threshold_to_consider = 0.5;
  double max_path_occupancy_ratio = 0.07;
  std::size_t offset_from_furthest = 20;
  std::size_t trajectory_point_step = 4;
  bool use_path_orientations = false;
};

struct MppiPathFollowCriticConfig : public MppiCriticTermConfig
{
  MppiPathFollowCriticConfig();
  double threshold_to_consider = 1.4;
  std::size_t offset_from_furthest = 6;
};

struct MppiPathAngleCriticConfig : public MppiCriticTermConfig
{
  MppiPathAngleCriticConfig();
  double threshold_to_consider = 0.5;
  double max_angle_to_furthest = 0.785398;
  std::size_t offset_from_furthest = 4;
  int mode = 0;
};

struct MppiPreferForwardCriticConfig : public MppiCriticTermConfig
{
  MppiPreferForwardCriticConfig();
  double threshold_to_consider = 0.5;
};

struct MppiCriticConfig
{
  std::vector<std::string> critics = {
    "ConstraintCritic",
    "ObstaclesCritic",
    "GoalCritic",
    "GoalAngleCritic",
    "PathAlignCritic",
    "PathFollowCritic",
    "PathAngleCritic",
    "PreferForwardCritic"};

  MppiConstraintCriticConfig constraint;
  MppiObstaclesCriticConfig obstacles;
  MppiGoalCriticConfig goal;
  MppiGoalAngleCriticConfig goal_angle;
  MppiPathAlignCriticConfig path_align;
  MppiPathFollowCriticConfig path_follow;
  MppiPathAngleCriticConfig path_angle;
  MppiPreferForwardCriticConfig prefer_forward;
};

struct MppiCriticData
{
  std::vector<MppiScoredTrajectory> trajectories;
  std::vector<MppiPathPoint> path;
  double local_path_length = 0.0;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  double vx_min = 0.0;
  double vx_max = 0.0;
  double min_turning_radius = 0.0;
  double model_dt = 0.1;
  double path_obstacle_occupancy_ratio = 0.0;
  std::size_t blocked_path_points = 0;
};

struct MppiCriticResult
{
  bool fail_flag = false;
  std::vector<MppiCandidateScore> scores;
};

class MppiCriticManager
{
public:
  void setConfig(const MppiCriticConfig & config);
  const MppiCriticConfig & config() const;

  MppiCriticResult score(const MppiCriticData & data) const;

private:
  MppiCriticConfig config_;

  double poweredCost(double value, double weight, int power) const;
  double distance(double x0, double y0, double x1, double y1) const;
  double shortestAngularDistance(double from, double to) const;
  double pathLength(const std::vector<MppiPathPoint> & path) const;
  std::size_t nearestPathIndex(
    const std::vector<MppiPathPoint> & path,
    double x,
    double y,
    std::size_t start_index = 0) const;
  std::size_t furthestReachedPathIndex(const MppiCriticData & data) const;

  void scoreConstraintCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scoreObstaclesCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scoreGoalCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scoreGoalAngleCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scorePathAlignCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scorePathFollowCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scorePathAngleCritic(const MppiCriticData & data, MppiCriticResult * result) const;
  void scorePreferForwardCritic(const MppiCriticData & data, MppiCriticResult * result) const;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CRITIC_MANAGER_H_
