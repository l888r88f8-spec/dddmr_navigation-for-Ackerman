#ifndef GLOBAL_PLANNER__FORWARD_HYBRID_A_STAR_H_
#define GLOBAL_PLANNER__FORWARD_HYBRID_A_STAR_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <perception_3d/perception_3d_ros.h>

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace global_planner
{

class ForwardHybridAStar
{
public:
  using CancelRequestedCallback = std::function<bool()>;
  struct ProjectionDiagnostics
  {
    bool success = false;
    double query_x = std::numeric_limits<double>::quiet_NaN();
    double query_y = std::numeric_limits<double>::quiet_NaN();
    double query_z = std::numeric_limits<double>::quiet_NaN();
    std::size_t ground_index = 0;
    double ground_x = std::numeric_limits<double>::quiet_NaN();
    double ground_y = std::numeric_limits<double>::quiet_NaN();
    double ground_z = std::numeric_limits<double>::quiet_NaN();
    double projection_distance = std::numeric_limits<double>::quiet_NaN();
    std::string fallback = "none";
    double fallback_z_before = std::numeric_limits<double>::quiet_NaN();
    double fallback_z_after = std::numeric_limits<double>::quiet_NaN();
    double fallback_nearest_distance = std::numeric_limits<double>::quiet_NaN();
  };

  struct SearchDiagnostics
  {
    bool success = false;
    bool canceled = false;
    std::size_t expansions = 0;
    double planning_time_sec = 0.0;
    std::size_t path_pose_count = 0;
  };
  using ProgressCallback = std::function<void(const SearchDiagnostics &)>;

  // This planner is a forward continuous-lattice search with projection-based
  // collision/ground validation. It is not a strictly graph-constrained Hybrid A*.
  struct Config
  {
    double wheelbase = 0.549185;
    double max_steer = 0.69;
    int steer_sample_count = 5;
    int heading_bin_count = 72;
    double primitive_length = 0.6;
    double primitive_step = 0.05;
    double projection_search_radius = 0.6;
    double goal_position_tolerance = 0.4;
    double goal_heading_tolerance = 0.35;
    bool use_goal_heading = false;
    double steering_penalty = 0.2;
    double steering_change_penalty = 0.3;
    double heading_change_penalty = 0.4;
    double obstacle_penalty_weight = 1.0;
    double edge_weight_penalty_weight = 1.0;
    double edge_weight_safe_threshold = 1.0;
    double edge_weight_soft_cap = 8.0;
    double edge_weight_hard_reject_threshold = 0.0;
    double heuristic_heading_weight = 0.2;
    double turn_side_hysteresis_penalty = 0.8;
    double rearward_check_distance = 1.5;
    double rearward_allowance = 0.05;
    double rearward_excursion_penalty = 4.0;
    double rearward_hard_reject_distance = 0.20;
    double strict_forward_check_distance = 1.5;
    double min_initial_forward_projection = 0.05;
    double max_projected_pitch = 0.55;
    double max_projected_vertical_jump = 0.25;
    bool allow_sample_nearest_fallback = false;
  };

  explicit ForwardHybridAStar(
    std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground,
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground,
    const rclcpp::Logger & logger);

  void SetConfig(const Config & config);
  void SetGlobalFrame(const std::string & global_frame);

  bool MakePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path * ros_path,
    bool force_position_only_goal = false,
    bool force_use_goal_heading = false,
    int preferred_initial_turn_sign = 0,
    const CancelRequestedCallback & cancel_requested = CancelRequestedCallback(),
    bool * was_canceled = nullptr,
    const std::string & debug_label = "",
    SearchDiagnostics * search_diagnostics = nullptr,
    ProjectionDiagnostics * start_projection = nullptr,
    ProjectionDiagnostics * goal_projection = nullptr,
    const ProgressCallback & progress_callback = ProgressCallback());

private:
  struct NodeRecord
  {
    bool opened = false;
    bool closed = false;
    double g = std::numeric_limits<double>::infinity();
    double h = 0.0;
    double f = std::numeric_limits<double>::infinity();
    std::size_t parent_state_id = std::numeric_limits<std::size_t>::max();
    int primitive_index = -1;
    int initial_turn_sign = 0;
    std::size_t ground_index = 0;
    int heading_bin = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
  };

  struct OpenEntry
  {
    double f = 0.0;
    std::size_t state_id = 0;
  };

  struct OpenEntryGreater
  {
    bool operator()(const OpenEntry & lhs, const OpenEntry & rhs) const
    {
      return lhs.f > rhs.f;
    }
  };

  struct SamplePose
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
  };

  struct PrimitiveResult
  {
    double end_x = 0.0;
    double end_y = 0.0;
    double end_z = 0.0;
    double end_yaw = 0.0;
    std::size_t end_ground_index = 0;
    int end_heading_bin = 0;
    double path_length = 0.0;
    double heading_change = 0.0;
    double obstacle_penalty = 0.0;
    double edge_penalty = 0.0;
    double rearward_penalty = 0.0;
  };

  static constexpr std::size_t kInvalidStateId = std::numeric_limits<std::size_t>::max();

  double NormalizeAngle(double angle) const;
  int QuantizeYaw(double yaw) const;
  double DequantizeYaw(int heading_bin) const;
  bool ExtractYaw(const geometry_msgs::msg::PoseStamped & pose, double * yaw) const;

  bool ComputeStateId(std::size_t ground_index, int heading_bin, std::size_t * state_id) const;

  bool PoseToGroundIndex(
    double x,
    double y,
    double z_hint,
    std::size_t * ground_index,
    double * projected_z,
    std::size_t previous_ground_index = std::numeric_limits<std::size_t>::max(),
    bool allow_nearest_fallback = true,
    ProjectionDiagnostics * diagnostics = nullptr) const;

  bool ValidateSample(
    double x,
    double y,
    double yaw,
    double z_hint,
    std::size_t * ground_index,
    double * projected_z,
    double * dgraph_value,
    std::size_t previous_ground_index,
    bool allow_nearest_fallback) const;

  bool IsGroundIndexInWeightMap(std::size_t ground_index) const;

  bool IsProjectedGroundTransitionValid(
    std::size_t from_ground_index,
    std::size_t to_ground_index) const;

  double ComputeHeuristic(
    double x,
    double y,
    double yaw,
    const geometry_msgs::msg::PoseStamped & goal,
    double goal_yaw,
    bool has_goal_yaw,
    bool use_goal_heading) const;

  bool IsGoalReached(
    const NodeRecord & node,
    const geometry_msgs::msg::PoseStamped & goal,
    double goal_yaw,
    bool has_goal_yaw,
    bool use_goal_heading) const;

  double GetPrimitiveSteerByIndex(int primitive_index) const;
  int GetPrimitiveTurnSign(int primitive_index) const;
  double ComputeTransitionCost(
    const NodeRecord & parent,
    int primitive_index,
    const PrimitiveResult & primitive_result,
    int preferred_initial_turn_sign) const;

  bool RolloutPrimitive(
    const NodeRecord & parent,
    int primitive_index,
    double planning_start_x,
    double planning_start_y,
    double planning_start_yaw,
    PrimitiveResult * primitive_result,
    std::vector<SamplePose> * sampled_poses) const;

  void AppendPose(
    double x,
    double y,
    double z,
    double yaw,
    nav_msgs::msg::Path * ros_path) const;

  bool ReconstructPath(
    std::size_t goal_state_id,
    const geometry_msgs::msg::PoseStamped & goal,
    double goal_yaw,
    bool has_goal_yaw,
    bool use_goal_heading,
    nav_msgs::msg::Path * ros_path) const;

  const NodeRecord * FindNode(std::size_t state_id) const;
  NodeRecord * FindNode(std::size_t state_id);

  std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_;

  rclcpp::Logger logger_;
  std::string global_frame_;
  Config config_;
  std::size_t weight_map_size_ = 0;
  std::vector<double> primitive_steers_;
  std::unordered_map<std::size_t, NodeRecord> nodes_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__FORWARD_HYBRID_A_STAR_H_
