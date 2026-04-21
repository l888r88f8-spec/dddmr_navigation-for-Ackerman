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

  // This planner is a forward continuous-lattice search with projection-based
  // collision/ground validation. It is not a strictly graph-constrained Hybrid A*.
  struct Config
  {
    double wheelbase = 0.549185;
    double max_steer = 0.69;
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
    double heuristic_heading_weight = 0.2;
    double turn_side_hysteresis_penalty = 0.8;
    double rearward_check_distance = 1.5;
    double rearward_allowance = 0.05;
    double rearward_excursion_penalty = 4.0;
    double rearward_hard_reject_distance = 0.20;
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
    bool * was_canceled = nullptr);

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
    double * projected_z) const;

  bool ValidateSample(
    double x,
    double y,
    double yaw,
    double z_hint,
    std::size_t * ground_index,
    double * projected_z,
    double * dgraph_value) const;

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
  std::vector<double> primitive_steers_;
  std::unordered_map<std::size_t, NodeRecord> nodes_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__FORWARD_HYBRID_A_STAR_H_
