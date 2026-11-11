#ifndef LOCAL_PLANNER_MPPI_CRITIC_MANAGER_H_
#define LOCAL_PLANNER_MPPI_CRITIC_MANAGER_H_

#include <functional>
#include <limits>
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
  bool collision_evaluated = false;
  std::size_t collision_node_checks = 0;
  std::size_t collision_point_checks = 0;
  std::size_t clearance_node_checks = 0;
  std::size_t clearance_point_checks = 0;
};

struct MppiPathPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool obstacle_free = true;
  // Optional support height used by lateral occupancy probes. Path points that
  // never query a 3-D map leave it at zero.
  double z = 0.0;
};

struct MppiPathProjection
{
  double signed_offset = 0.0;
  bool valid = false;
};

struct MppiScoredTrajectory
{
  std::vector<MppiControl> controls;
  std::vector<MppiTrajectoryPoint> points;
};

class MppiBypassSideClassifier
{
public:
  explicit MppiBypassSideClassifier(const std::vector<MppiPathPoint> & path);

  MppiPathProjection projectPoint(double x, double y) const;
  MppiBypassSide classify(const MppiScoredTrajectory & trajectory) const;

private:
  struct PathSegment
  {
    double start_x = 0.0;
    double start_y = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double inverse_length = 0.0;
  };

  bool path_valid_ = false;
  std::vector<PathSegment> segments_;
};

MppiPathProjection projectPointOntoMppiPath(
  const std::vector<MppiPathPoint> & path,
  double x,
  double y);

// Lateral occupancy measured around the blocked section of the local path.
// Positive offsets are to the path's left, matching projectPointOntoMppiPath.
struct MppiLateralOccupancyProfile
{
  bool valid = false;
  // Mean signed offset of every blocked lateral probe. Its sign is the side
  // the obstacle actually sits on; a value near zero means the obstacle
  // straddles the route and neither side is geometrically preferred.
  double obstacle_offset = 0.0;
  // Distance from the route centreline to the nearest blocked probe on each
  // side. Infinity means no blocked probe was found within the sampled band.
  double left_clearance = std::numeric_limits<double>::infinity();
  double right_clearance = std::numeric_limits<double>::infinity();
  // Widest contiguous free lateral band on each side, in metres, taking the
  // most restrictive sampled route point. This is the room the vehicle centre
  // actually has beyond a partial obstruction: an obstacle whose influence
  // reaches across the centreline can still leave a wide free band farther
  // out on the opposite side. The bypass gate uses this span rather than the
  // nearest blocked probe, which would reject that free band.
  double left_free_span = std::numeric_limits<double>::infinity();
  double right_free_span = std::numeric_limits<double>::infinity();
  // Signed offset (metres, left positive) of the inner edge of the same free
  // run that produced left/right_free_span. Together with the span it gives
  // the lateral interval the vehicle can occupy on that side, which is what
  // the bypass detour cost measures the vehicle's distance to. NaN means no
  // free run was found on that side.
  double left_band_offset = std::numeric_limits<double>::quiet_NaN();
  double right_band_offset = std::numeric_limits<double>::quiet_NaN();
};

struct MppiLateralProfileConfig
{
  double max_lateral_offset = 1.5;
  double lateral_step = 0.15;
  // At most this many blocked route points are probed, spread across the run.
  std::size_t max_blocked_points = 3;
  // A lateral probe sitting exactly at the blocked anchor's longitudinal
  // position cannot see an obstacle that only intrudes into the front (or
  // rear) of the vehicle footprint. When positive, every lateral offset is
  // also probed this far along the route direction (0 disables the extra
  // probes). Each lateral offset still contributes at most one vote.
  double max_longitudinal_offset = 0.0;
  double longitudinal_step = 0.35;
};

using MppiBlockedProbe = std::function<bool(const MppiPathPoint &)>;

MppiLateralOccupancyProfile measureMppiLateralOccupancyProfile(
  const std::vector<MppiPathPoint> & path,
  const MppiBlockedProbe & probe_blocked,
  const MppiLateralProfileConfig & config = MppiLateralProfileConfig{});

// Mirrors the Autoware avoidance rule: pass on the side opposite the obstacle
// and only when that side has at least required_width of contiguous free
// lateral room. A neutral result means "no geometric preference", which keeps
// the previous cost-based selection behaviour intact.
MppiBypassSide selectMppiGeometricBypassSide(
  const MppiLateralOccupancyProfile & profile,
  double min_decisive_offset,
  double required_width);

// Lateral distance from the vehicle's signed route offset to the free band on
// the given side, in metres. Zero when the vehicle already sits inside the
// band. NaN when the profile has no usable band on that side, which lets the
// caller treat the detour cost as unavailable rather than free.
double mppiBypassBandDetour(
  const MppiLateralOccupancyProfile & profile,
  MppiBypassSide side,
  double vehicle_offset);

bool isMppiTrajectoryCollisionFree(const MppiScoredTrajectory & trajectory);
MppiBypassSide classifyMppiBypassSide(
  const MppiScoredTrajectory & trajectory,
  const std::vector<MppiPathPoint> & path);

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
  double robot_path_offset = 0.0;
  bool robot_path_offset_valid = false;
  double vx_min = 0.0;
  double vx_max = 0.0;
  double min_turning_radius = 0.0;
  double model_dt = 0.1;
  double path_obstacle_occupancy_ratio = 0.0;
  std::size_t blocked_path_points = 0;
  std::size_t obstacle_collision_node_checks = 0;
  std::size_t obstacle_collision_point_checks = 0;
  std::size_t obstacle_clearance_node_checks = 0;
  std::size_t obstacle_clearance_point_checks = 0;
};

struct MppiCriticResult
{
  bool fail_flag = false;
  std::vector<MppiCandidateScore> scores;
};

bool annotateMppiCandidateMetadata(
  const MppiCriticData & data,
  MppiCriticResult * result);

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
