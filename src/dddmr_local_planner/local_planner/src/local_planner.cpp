/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <local_planner/local_planner.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner {
namespace
{
std::string normalizeControllerBackend(const std::string & backend)
{
  if(backend == "regulated_pure_pursuit" || backend == "rpp"){
    return "regulated_pure_pursuit";
  }
  if(backend == "legacy_rollout" || backend == "legacy"){
    return "legacy_rollout";
  }
  return backend;
}
}

Local_Planner::Local_Planner(const std::string& name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  got_odom_ = false;
  last_valid_prune_plan_ = clock_->now();
  cached_local_pruned_path_stamp_ = clock_->now();
  route_version_ = 0;
  goal_seq_ = 0;
  route_source_label_ = "planner_result";
  local_route_progress_index_ = 0;
  last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
  consecutive_prune_failure_cycles_ = 0;
  last_prune_used_cache_ = false;
  cached_local_pruned_path_valid_ = false;
  last_heading_reference_valid_ = false;
  heading_reference_stale_cycles_ = 0;
  consecutive_heading_reference_failure_cycles_ = 0;
  controller_backend_ = "legacy_rollout";
  rpp_critic_trajectory_generator_name_ = "ackermann_simple";
  rpp_nominal_linear_speed_ = 1.0;
  rpp_min_linear_speed_ = 0.15;
  rpp_alignment_linear_speed_ = 0.25;
  rpp_goal_slowdown_distance_ = 1.5;
  rpp_min_lookahead_distance_ = 1.0;
  rpp_max_lookahead_distance_ = 3.0;
  rpp_lookahead_time_ = 1.5;
  rpp_alignment_lookahead_distance_ = 1.0;
  rpp_max_lateral_accel_ = 1.5;
  rpp_prediction_horizon_sec_ = 1.5;
  rpp_prediction_step_sec_ = 0.1;
  rpp_wheelbase_ = 0.55;
  rpp_max_steer_ = 0.69;
  rpp_max_angular_velocity_ = 1.2;
  route_start_front_reach_distance_ = 1.2;
  route_start_front_projection_threshold_ = 0.05;
}

void Local_Planner::initial(
      const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
      const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
      const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators){

  declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_topic", odom_topic_);
  RCLCPP_DEBUG(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());

  declare_parameter("odom_topic_qos", rclcpp::ParameterValue("reliable"));
  this->get_parameter("odom_topic_qos", odom_topic_qos_);
  RCLCPP_DEBUG(this->get_logger(), "odom_topic_qos: %s", odom_topic_qos_.c_str());

  declare_parameter("compute_best_trajectory_in_odomCb", rclcpp::ParameterValue(false));
  this->get_parameter("compute_best_trajectory_in_odomCb", compute_best_trajectory_in_odomCb_);
  RCLCPP_DEBUG(this->get_logger(), "compute_best_trajectory_in_odomCb: %d", compute_best_trajectory_in_odomCb_);

  declare_parameter(
    "local_reference.forward_distance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_forward_reference_distance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "local_reference.forward_distance",
    configured_forward_reference_distance);
  declare_parameter("forward_prune", rclcpp::ParameterValue(1.0));
  this->get_parameter("forward_prune", forward_prune_);
  if(std::isfinite(configured_forward_reference_distance)){
    forward_prune_ = configured_forward_reference_distance;
    RCLCPP_DEBUG(this->get_logger(), "local_reference.forward_distance: %.2f", forward_prune_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "forward_prune (legacy local reference alias): %.2f",
      forward_prune_);
  }

  declare_parameter(
    "local_reference.backward_distance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_backward_reference_distance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "local_reference.backward_distance",
    configured_backward_reference_distance);
  declare_parameter("backward_prune", rclcpp::ParameterValue(0.5));
  this->get_parameter("backward_prune", backward_prune_);
  if(std::isfinite(configured_backward_reference_distance)){
    backward_prune_ = configured_backward_reference_distance;
    RCLCPP_DEBUG(this->get_logger(), "local_reference.backward_distance: %.2f", backward_prune_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "backward_prune (legacy local reference alias): %.2f",
      backward_prune_);
  }

  declare_parameter("heading_tracking_distance", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_tracking_distance", heading_tracking_distance_);
  RCLCPP_DEBUG(this->get_logger(), "heading_tracking_distance: %.2f", heading_tracking_distance_);

  declare_parameter(
    "route_start_alignment.heading_tolerance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_route_start_alignment_tolerance =
    std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "route_start_alignment.heading_tolerance",
    configured_route_start_alignment_tolerance);
  declare_parameter("heading_align_angle", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_align_angle", heading_align_angle_);
  if(std::isfinite(configured_route_start_alignment_tolerance)){
    heading_align_angle_ = configured_route_start_alignment_tolerance;
    RCLCPP_DEBUG(
      this->get_logger(),
      "route_start_alignment.heading_tolerance: %.2f",
      heading_align_angle_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "heading_align_angle (legacy route start alignment alias): %.2f",
      heading_align_angle_);
  }

  declare_parameter(
    "route_start_alignment.front_reach_distance",
    rclcpp::ParameterValue(1.2));
  this->get_parameter(
    "route_start_alignment.front_reach_distance",
    route_start_front_reach_distance_);
  RCLCPP_DEBUG(
    this->get_logger(),
    "route_start_alignment.front_reach_distance: %.2f",
    route_start_front_reach_distance_);

  declare_parameter(
    "route_start_alignment.min_front_projection",
    rclcpp::ParameterValue(0.05));
  this->get_parameter(
    "route_start_alignment.min_front_projection",
    route_start_front_projection_threshold_);
  RCLCPP_DEBUG(
    this->get_logger(),
    "route_start_alignment.min_front_projection: %.2f",
    route_start_front_projection_threshold_);

  declare_parameter("causal_prune_search_window", rclcpp::ParameterValue(40));
  this->get_parameter("causal_prune_search_window", causal_prune_search_window_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_search_window: %zu", causal_prune_search_window_);

  declare_parameter("causal_prune_max_index_jump", rclcpp::ParameterValue(15));
  this->get_parameter("causal_prune_max_index_jump", causal_prune_max_index_jump_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_index_jump: %zu", causal_prune_max_index_jump_);

  declare_parameter("causal_prune_max_arc_jump", rclcpp::ParameterValue(2.0));
  this->get_parameter("causal_prune_max_arc_jump", causal_prune_max_arc_jump_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_arc_jump: %.2f", causal_prune_max_arc_jump_);

  declare_parameter("causal_prune_max_lateral_distance", rclcpp::ParameterValue(1.0));
  this->get_parameter("causal_prune_max_lateral_distance", causal_prune_max_lateral_distance_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_lateral_distance: %.2f", causal_prune_max_lateral_distance_);

  declare_parameter("causal_prune_max_heading_error", rclcpp::ParameterValue(1.75));
  this->get_parameter("causal_prune_max_heading_error", causal_prune_max_heading_error_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_heading_error: %.2f", causal_prune_max_heading_error_);

  declare_parameter("min_heading_reference_length", rclcpp::ParameterValue(0.3));
  this->get_parameter("min_heading_reference_length", min_heading_reference_length_);
  RCLCPP_DEBUG(this->get_logger(), "min_heading_reference_length: %.2f", min_heading_reference_length_);

  declare_parameter("max_heading_reference_stale_cycles", rclcpp::ParameterValue(10));
  this->get_parameter("max_heading_reference_stale_cycles", max_heading_reference_stale_cycles_);
  RCLCPP_DEBUG(this->get_logger(), "max_heading_reference_stale_cycles: %zu", max_heading_reference_stale_cycles_);

  declare_parameter("max_prune_failure_cycles", rclcpp::ParameterValue(10));
  this->get_parameter("max_prune_failure_cycles", max_prune_failure_cycles_);
  RCLCPP_DEBUG(this->get_logger(), "max_prune_failure_cycles: %zu", max_prune_failure_cycles_);

  declare_parameter("cached_pruned_path_timeout_sec", rclcpp::ParameterValue(0.8));
  this->get_parameter("cached_pruned_path_timeout_sec", cached_pruned_path_timeout_sec_);
  RCLCPP_DEBUG(this->get_logger(), "cached_pruned_path_timeout_sec: %.2f", cached_pruned_path_timeout_sec_);

  declare_parameter("prune_plane_timeout", rclcpp::ParameterValue(3.0));
  this->get_parameter("prune_plane_timeout", prune_plane_timeout_);
  RCLCPP_DEBUG(this->get_logger(), "prune_plane_timeout: %.2f", prune_plane_timeout_);

  declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_DEBUG(this->get_logger(), "xy_goal_tolerance: %.2f", xy_goal_tolerance_);

  declare_parameter(
    "goal_alignment.heading_tolerance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_goal_alignment_tolerance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "goal_alignment.heading_tolerance",
    configured_goal_alignment_tolerance);
  declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  if(std::isfinite(configured_goal_alignment_tolerance)){
    yaw_goal_tolerance_ = configured_goal_alignment_tolerance;
    RCLCPP_DEBUG(
      this->get_logger(),
      "goal_alignment.heading_tolerance: %.2f",
      yaw_goal_tolerance_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "yaw_goal_tolerance (legacy goal alignment alias): %.2f",
      yaw_goal_tolerance_);
  }

  declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_DEBUG(this->get_logger(), "controller_frequency: %.2f", controller_frequency_);

  declare_parameter("controller_backend", rclcpp::ParameterValue("legacy_rollout"));
  this->get_parameter("controller_backend", controller_backend_);
  controller_backend_ = normalizeControllerBackend(controller_backend_);
  if(controller_backend_ != "legacy_rollout" &&
     controller_backend_ != "regulated_pure_pursuit")
  {
    RCLCPP_WARN(
      this->get_logger(),
      "unknown controller_backend '%s', fallback to legacy_rollout",
      controller_backend_.c_str());
    controller_backend_ = "legacy_rollout";
  }
  RCLCPP_DEBUG(this->get_logger(), "controller_backend: %s", controller_backend_.c_str());

  declare_parameter("rpp.critic_trajectory_generator_name", rclcpp::ParameterValue("ackermann_simple"));
  this->get_parameter("rpp.critic_trajectory_generator_name", rpp_critic_trajectory_generator_name_);
  declare_parameter("rpp.nominal_linear_speed", rclcpp::ParameterValue(1.0));
  this->get_parameter("rpp.nominal_linear_speed", rpp_nominal_linear_speed_);
  declare_parameter("rpp.min_linear_speed", rclcpp::ParameterValue(0.15));
  this->get_parameter("rpp.min_linear_speed", rpp_min_linear_speed_);
  declare_parameter("rpp.alignment_linear_speed", rclcpp::ParameterValue(0.25));
  this->get_parameter("rpp.alignment_linear_speed", rpp_alignment_linear_speed_);
  declare_parameter("rpp.goal_slowdown_distance", rclcpp::ParameterValue(1.5));
  this->get_parameter("rpp.goal_slowdown_distance", rpp_goal_slowdown_distance_);
  declare_parameter("rpp.min_lookahead_distance", rclcpp::ParameterValue(1.0));
  this->get_parameter("rpp.min_lookahead_distance", rpp_min_lookahead_distance_);
  declare_parameter("rpp.max_lookahead_distance", rclcpp::ParameterValue(3.0));
  this->get_parameter("rpp.max_lookahead_distance", rpp_max_lookahead_distance_);
  declare_parameter("rpp.lookahead_time", rclcpp::ParameterValue(1.5));
  this->get_parameter("rpp.lookahead_time", rpp_lookahead_time_);
  declare_parameter("rpp.alignment_lookahead_distance", rclcpp::ParameterValue(1.0));
  this->get_parameter("rpp.alignment_lookahead_distance", rpp_alignment_lookahead_distance_);
  declare_parameter("rpp.max_lateral_accel", rclcpp::ParameterValue(1.5));
  this->get_parameter("rpp.max_lateral_accel", rpp_max_lateral_accel_);
  declare_parameter("rpp.prediction_horizon_sec", rclcpp::ParameterValue(1.5));
  this->get_parameter("rpp.prediction_horizon_sec", rpp_prediction_horizon_sec_);
  declare_parameter("rpp.prediction_step_sec", rclcpp::ParameterValue(0.1));
  this->get_parameter("rpp.prediction_step_sec", rpp_prediction_step_sec_);
  declare_parameter("rpp.wheelbase", rclcpp::ParameterValue(0.55));
  this->get_parameter("rpp.wheelbase", rpp_wheelbase_);
  declare_parameter("rpp.max_steer", rclcpp::ParameterValue(0.69));
  this->get_parameter("rpp.max_steer", rpp_max_steer_);
  declare_parameter("rpp.max_angular_velocity", rclcpp::ParameterValue(1.2));
  this->get_parameter("rpp.max_angular_velocity", rpp_max_angular_velocity_);
  if(controller_backend_ == "regulated_pure_pursuit"){
    RCLCPP_DEBUG(this->get_logger(), "rpp.critic_trajectory_generator_name: %s", rpp_critic_trajectory_generator_name_.c_str());
    RCLCPP_DEBUG(this->get_logger(), "rpp.nominal_linear_speed: %.2f", rpp_nominal_linear_speed_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.min_linear_speed: %.2f", rpp_min_linear_speed_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.alignment_linear_speed: %.2f", rpp_alignment_linear_speed_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.goal_slowdown_distance: %.2f", rpp_goal_slowdown_distance_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.min_lookahead_distance: %.2f", rpp_min_lookahead_distance_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.max_lookahead_distance: %.2f", rpp_max_lookahead_distance_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.lookahead_time: %.2f", rpp_lookahead_time_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.alignment_lookahead_distance: %.2f", rpp_alignment_lookahead_distance_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.max_lateral_accel: %.2f", rpp_max_lateral_accel_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.prediction_horizon_sec: %.2f", rpp_prediction_horizon_sec_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.prediction_step_sec: %.2f", rpp_prediction_step_sec_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.wheelbase: %.2f", rpp_wheelbase_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.max_steer: %.2f", rpp_max_steer_);
    RCLCPP_DEBUG(this->get_logger(), "rpp.max_angular_velocity: %.2f", rpp_max_angular_velocity_);
  }

  declare_parameter("debug_publish.robot_cuboid", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.robot_cuboid", debug_publish_robot_cuboid_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.robot_cuboid: %d", debug_publish_robot_cuboid_);

  declare_parameter("debug_publish.aggregated_pc", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.aggregated_pc", debug_publish_aggregated_pc_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.aggregated_pc: %d", debug_publish_aggregated_pc_);

  declare_parameter("debug_publish.prune_plan", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.prune_plan", debug_publish_prune_plan_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.prune_plan: %d", debug_publish_prune_plan_);

  declare_parameter("debug_publish.accepted_trajectory", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.accepted_trajectory", debug_publish_accepted_trajectory_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.accepted_trajectory: %d", debug_publish_accepted_trajectory_);

  declare_parameter("debug_publish.best_trajectory", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.best_trajectory", debug_publish_best_trajectory_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.best_trajectory: %d", debug_publish_best_trajectory_);

  declare_parameter("debug_publish.all_trajectories", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.all_trajectories", debug_publish_all_trajectories_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.all_trajectories: %d", debug_publish_all_trajectories_);


  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;

  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  parseCuboid(); //after robot_frame is got
  
  const auto debug_path_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  pub_robot_cuboid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_cuboid", 1);  
  pub_aggregate_observation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregated_pc", 1);  
  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>("prune_plan", 1);
  pub_route_sent_to_local_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/route_sent_to_local", debug_path_qos);
  pub_local_pruned_path_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/local_pruned_path", debug_path_qos);
  pub_best_trajectory_path_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/best_trajectory", debug_path_qos);
  pub_accepted_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("accepted_trajectory", 1);
  pub_best_trajectory_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>("best_trajectory", 2);
  pub_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("trajectory", 2);
  //pub_pc_normal_ = pnh_.advertise<visualization_msgs::MarkerArray>("normal_marker", 2, true);
  //pub_trajectory_cuboids_ = pnh_.advertise<sensor_msgs::PointCloud2>("trajectory_cuboids", 2, true);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  if(odom_topic_qos_=="reliable" || odom_topic_qos_=="Reliable"){
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  else{
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  
  //@Initial pcl ptr
  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
}

Local_Planner::~Local_Planner(){

  perception_3d_ros_.reset();
  mpc_critics_ros_.reset();
  trajectory_generators_ros_.reset();
  trajectories_.reset();
  tf2Buffer_.reset();

}

nav_msgs::msg::Path Local_Planner::buildPathFromPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(poses.size());
  for(const auto & pose : poses){
    geometry_msgs::msg::PoseStamped pose_stamped = pose;
    pose_stamped.header = path.header;
    path.poses.push_back(pose_stamped);
  }
  return path;
}

nav_msgs::msg::Path Local_Planner::buildPathFromTrajectory(
  const base_trajectory::Trajectory & traj) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(traj.getPointsSize());
  for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = traj.getPoint(i).pose;
    path.poses.push_back(pose_stamped);
  }
  return path;
}

void Local_Planner::publishDebugPath(
  const nav_msgs::msg::Path & path,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
  const std::string & stage_label,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label,
  bool throttle_log) const
{
  if(!publisher){
    return;
  }

  nav_msgs::msg::Path output = path;
  output.header.frame_id = global_frame_;
  output.header.stamp = clock_->now();
  for(auto & pose : output.poses){
    pose.header = output.header;
  }
  publisher->publish(output);

  if(throttle_log){
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "%s published, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, poses=%zu",
      stage_label.c_str(),
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_,
      output.poses.size());
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger().get_child(name_),
      "%s published, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, poses=%zu",
      stage_label.c_str(),
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_,
      output.poses.size());
  }
}

dddmr_sys_core::PlannerState Local_Planner::prepareControllerContext(
  ControllerContext * context)
{
  if(context == nullptr){
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  auto * stacked_perception = perception_3d_ros_ ? perception_3d_ros_->getStackedPerception() : nullptr;
  auto perception_shared_data = perception_3d_ros_ ? perception_3d_ros_->getSharedDataPtr() : nullptr;

  if(!got_odom_){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Odom is not received.");
    return dddmr_sys_core::TF_FAIL;
  }

  if(!stacked_perception || !perception_shared_data){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D shared data is not ready.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }

  if(!stacked_perception->isSensorOK()){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D is not ok.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }

  control_loop_time_ = clock_->now();
  bool have_usable_prune_plan = false;

  {
    std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(stacked_perception->getMutex()));
    have_usable_prune_plan = prunePlan(forward_prune_, backward_prune_);
    perception_shared_data->pcl_prune_plan_ = pcl_prune_plan_;
    context->aggregate_observation = perception_shared_data->aggregate_observation_;
    context->aggregate_observation_kdtree = perception_shared_data->aggregate_observation_kdtree_;
    context->current_allowed_max_linear_speed = perception_shared_data->current_allowed_max_linear_speed_;
    context->opinions = stacked_perception->getOpinions();
  }

  if(!context->aggregate_observation){
    context->aggregate_observation = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  if(debug_publish_aggregated_pc_ && pub_aggregate_observation_->get_subscription_count() > 0){
    sensor_msgs::msg::PointCloud2 ros2_aggregate_onservation;
    pcl::toROSMsg(*context->aggregate_observation, ros2_aggregate_onservation);
    pub_aggregate_observation_->publish(ros2_aggregate_onservation);
  }

  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "TF out of date in local planner, the local planner wont go further.");
    return dddmr_sys_core::TF_FAIL;
  }

  if(!have_usable_prune_plan){
    const bool deviation_timeout =
      (clock_->now() - last_valid_prune_plan_).seconds() >= prune_plane_timeout_;
    const bool deviation_distance =
      last_robot_to_route_distance_ > causal_prune_max_lateral_distance_;
    const bool deviation_confirmed =
      consecutive_prune_failure_cycles_ > max_prune_failure_cycles_ &&
      deviation_timeout &&
      deviation_distance;

    if(deviation_confirmed){
      RCLCPP_FATAL(
        this->get_logger().get_child(name_),
        "deviation confirmed after consecutive prune failures, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        consecutive_prune_failure_cycles_,
        last_robot_to_route_distance_);
      RCLCPP_FATAL(
        this->get_logger().get_child(name_),
        "deviation from route reference exceeded tolerance, controller returns PRUNE_PLAN_FAIL.");
      return dddmr_sys_core::PRUNE_PLAN_FAIL;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "transient prune failure, not treating as deviation yet, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      consecutive_prune_failure_cycles_,
      last_robot_to_route_distance_);
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::evaluatePerceptionOpinions(
  const std::vector<perception_3d::PerceptionOpinion> & opinions) const
{
  for(auto opinion_it = opinions.begin(); opinion_it != opinions.end(); ++opinion_it){
    if((*opinion_it) == perception_3d::PATH_BLOCKED_WAIT){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 5000,
        "Found the prune plan is blocked, go to wait state.");
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    if((*opinion_it) == perception_3d::PATH_BLOCKED_REPLANNING){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 5000,
        "Found the prune plan is blocked, go to replanning.");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }
  }
  return dddmr_sys_core::TRAJECTORY_FOUND;
}

void Local_Planner::updateCriticSharedData(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & aggregate_observation,
  const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr & aggregate_observation_kdtree)
{
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = aggregate_observation;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_ = aggregate_observation_kdtree;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  mpc_critics_ros_->updateSharedData();
}

bool Local_Planner::selectLookaheadPose(
  double lookahead_distance,
  geometry_msgs::msg::PoseStamped * lookahead_pose) const
{
  if(lookahead_pose == nullptr || prune_plan_.poses.empty()){
    return false;
  }

  const double robot_x = trans_gbl2b_.transform.translation.x;
  const double robot_y = trans_gbl2b_.transform.translation.y;
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();

  for(std::size_t i = 0; i < prune_plan_.poses.size(); ++i){
    const double dx = prune_plan_.poses[i].pose.position.x - robot_x;
    const double dy = prune_plan_.poses[i].pose.position.y - robot_y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    if(distance < closest_distance){
      closest_distance = distance;
      closest_index = i;
    }
  }

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped previous_pose = prune_plan_.poses[closest_index];
  for(std::size_t i = closest_index; i < prune_plan_.poses.size(); ++i){
    if(i > closest_index){
      accumulated_distance += getDistanceBTWPoseStamp(previous_pose, prune_plan_.poses[i]);
      previous_pose = prune_plan_.poses[i];
    }

    if(accumulated_distance + 1e-6 < lookahead_distance && i + 1 < prune_plan_.poses.size()){
      continue;
    }

    const double robot_yaw = getRobotYaw();
    const double dx = prune_plan_.poses[i].pose.position.x - robot_x;
    const double dy = prune_plan_.poses[i].pose.position.y - robot_y;
    const double x_local =
      std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    if(x_local > 0.05 || i + 1 == prune_plan_.poses.size()){
      *lookahead_pose = prune_plan_.poses[i];
      return true;
    }
  }

  *lookahead_pose = prune_plan_.poses.back();
  return true;
}

bool Local_Planner::buildGoalAlignmentReference(tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr || global_plan_.empty()){
    return false;
  }

  if(buildHeadingReferenceFromPoseOrientation(global_plan_.back(), reference_pose)){
    return true;
  }

  if(global_plan_.size() < 2){
    return false;
  }

  const std::size_t start_index = global_plan_.size() >= 2 ? global_plan_.size() - 2 : 0;
  return buildHeadingReferenceFromPlan(
    global_plan_,
    start_index,
    std::max(0.05, min_heading_reference_length_),
    reference_pose);
}

double Local_Planner::getDistanceToGoal() const
{
  if(global_plan_.empty()){
    return std::numeric_limits<double>::infinity();
  }

  const auto & goal_pose = global_plan_.back();
  const double dx = trans_gbl2b_.transform.translation.x - goal_pose.pose.position.x;
  const double dy = trans_gbl2b_.transform.translation.y - goal_pose.pose.position.y;
  const double dz = trans_gbl2b_.transform.translation.z - goal_pose.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Local_Planner::getMaxAckermannCurvature() const
{
  return std::tan(rpp_max_steer_) / std::max(rpp_wheelbase_, 1e-6);
}

pcl::PointCloud<pcl::PointXYZ> Local_Planner::transformRobotCuboid(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.reserve(robot_cuboid_vertices_.size());

  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for(const auto & vertex : robot_cuboid_vertices_){
    pcl::PointXYZ transformed_point;
    transformed_point.x =
      pose.pose.position.x + cos_yaw * vertex.x - sin_yaw * vertex.y;
    transformed_point.y =
      pose.pose.position.y + sin_yaw * vertex.x + cos_yaw * vertex.y;
    transformed_point.z = pose.pose.position.z + vertex.z;
    cuboid.push_back(transformed_point);
  }

  return cuboid;
}

base_trajectory::cuboid_min_max_t Local_Planner::computeCuboidMinMax(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid) const
{
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
  max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

  for(const auto & point : cuboid.points){
    min_pt.x = std::min(min_pt.x, point.x);
    min_pt.y = std::min(min_pt.y, point.y);
    min_pt.z = std::min(min_pt.z, point.z);
    max_pt.x = std::max(max_pt.x, point.x);
    max_pt.y = std::max(max_pt.y, point.y);
    max_pt.z = std::max(max_pt.z, point.z);
  }

  return {min_pt, max_pt};
}

base_trajectory::Trajectory Local_Planner::buildPredictedTrajectory(
  double linear_velocity,
  double angular_velocity) const
{
  const double time_step = std::max(0.02, rpp_prediction_step_sec_);
  const double horizon = std::max(time_step, rpp_prediction_horizon_sec_);
  const unsigned int steps =
    std::max<unsigned int>(2, static_cast<unsigned int>(std::ceil(horizon / time_step)) + 1);

  base_trajectory::Trajectory trajectory(
    linear_velocity,
    0.0,
    angular_velocity,
    time_step,
    steps);
  trajectory.cost_ = 0.0;

  double x = trans_gbl2b_.transform.translation.x;
  double y = trans_gbl2b_.transform.translation.y;
  double z = trans_gbl2b_.transform.translation.z;
  double yaw = getRobotYaw();

  for(unsigned int i = 0; i < steps; ++i){
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = clock_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);

    auto cuboid = transformRobotCuboid(pose);
    trajectory.addPoint(pose, cuboid, computeCuboidMinMax(cuboid));

    if(i + 1 >= steps){
      break;
    }

    x += linear_velocity * std::cos(yaw + angular_velocity * time_step * 0.5) * time_step;
    y += linear_velocity * std::sin(yaw + angular_velocity * time_step * 0.5) * time_step;
    yaw += angular_velocity * time_step;
  }

  return trajectory;
}

void Local_Planner::resetLocalRouteTrackingState()
{
  local_route_progress_index_ = 0;
  last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
  consecutive_prune_failure_cycles_ = 0;
  last_prune_used_cache_ = false;
  cached_local_pruned_path_valid_ = false;
  cached_local_pruned_path_.poses.clear();
  cached_local_pruned_path_.header.frame_id = global_frame_;
  cached_local_pruned_path_.header.stamp = clock_->now();
  cached_local_pruned_path_stamp_ = clock_->now();
  cached_pcl_prune_plan_.clear();
  prune_plan_.poses.clear();
  prune_plan_.header.frame_id = global_frame_;
  prune_plan_.header.stamp = clock_->now();
  pcl_prune_plan_.clear();
  global_plan_arc_lengths_.clear();
  last_heading_reference_valid_ = false;
  heading_reference_stale_cycles_ = 0;
  consecutive_heading_reference_failure_cycles_ = 0;
  last_valid_prune_plan_ = clock_->now();
}

void Local_Planner::rebuildGlobalPlanArcLengths()
{
  global_plan_arc_lengths_.clear();
  global_plan_arc_lengths_.reserve(global_plan_.size());
  double accumulated_distance = 0.0;
  for(std::size_t i = 0; i < global_plan_.size(); ++i){
    if(i > 0){
      accumulated_distance += getDistanceBTWPoseStamp(global_plan_[i - 1], global_plan_[i]);
    }
    global_plan_arc_lengths_.push_back(accumulated_distance);
  }
}

double Local_Planner::getGlobalPlanArcDistance(std::size_t start_index, std::size_t end_index) const
{
  if(global_plan_arc_lengths_.empty()){
    return 0.0;
  }
  start_index = std::min(start_index, global_plan_arc_lengths_.size() - 1);
  end_index = std::min(end_index, global_plan_arc_lengths_.size() - 1);
  const double start_arc = global_plan_arc_lengths_[start_index];
  const double end_arc = global_plan_arc_lengths_[end_index];
  return std::fabs(end_arc - start_arc);
}

double Local_Planner::getRobotYaw() const
{
  tf2::Quaternion q;
  tf2::fromMsg(trans_gbl2b_.transform.rotation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double Local_Planner::getRouteSegmentLateralDistance(std::size_t index) const
{
  if(global_plan_.empty()){
    return std::numeric_limits<double>::infinity();
  }

  index = std::min(index, global_plan_.size() - 1);
  const double robot_x = trans_gbl2b_.transform.translation.x;
  const double robot_y = trans_gbl2b_.transform.translation.y;

  const auto distance_to_point =
    [&](const geometry_msgs::msg::PoseStamped & pose) {
      const double dx = robot_x - pose.pose.position.x;
      const double dy = robot_y - pose.pose.position.y;
      return std::sqrt(dx * dx + dy * dy);
    };

  const auto distance_to_segment =
    [&](const geometry_msgs::msg::PoseStamped & start_pose,
        const geometry_msgs::msg::PoseStamped & end_pose) {
      const double start_x = start_pose.pose.position.x;
      const double start_y = start_pose.pose.position.y;
      const double end_x = end_pose.pose.position.x;
      const double end_y = end_pose.pose.position.y;
      const double seg_x = end_x - start_x;
      const double seg_y = end_y - start_y;
      const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
      if(seg_len_sq < 1e-6){
        return distance_to_point(start_pose);
      }
      const double projection =
        ((robot_x - start_x) * seg_x + (robot_y - start_y) * seg_y) / seg_len_sq;
      const double clamped_projection = std::max(0.0, std::min(1.0, projection));
      const double closest_x = start_x + clamped_projection * seg_x;
      const double closest_y = start_y + clamped_projection * seg_y;
      const double dx = robot_x - closest_x;
      const double dy = robot_y - closest_y;
      return std::sqrt(dx * dx + dy * dy);
    };

  double best_distance = distance_to_point(global_plan_[index]);
  if(index > 0){
    best_distance = std::min(best_distance, distance_to_segment(global_plan_[index - 1], global_plan_[index]));
  }
  if(index + 1 < global_plan_.size()){
    best_distance = std::min(best_distance, distance_to_segment(global_plan_[index], global_plan_[index + 1]));
  }
  return best_distance;
}

double Local_Planner::getRouteHeadingError(std::size_t index) const
{
  if(global_plan_.size() < 2){
    return 0.0;
  }

  index = std::min(index, global_plan_.size() - 1);
  std::size_t first_index = index;
  std::size_t last_index = index;
  if(index + 1 < global_plan_.size()){
    last_index = index + 1;
  }
  else if(index > 0){
    first_index = index - 1;
  }

  const double dx =
    global_plan_[last_index].pose.position.x - global_plan_[first_index].pose.position.x;
  const double dy =
    global_plan_[last_index].pose.position.y - global_plan_[first_index].pose.position.y;
  if(std::hypot(dx, dy) < 1e-6){
    return 0.0;
  }

  const double route_yaw = std::atan2(dy, dx);
  return std::fabs(angles::shortest_angular_distance(getRobotYaw(), route_yaw));
}

bool Local_Planner::selectCausalPruneAnchor(
  std::size_t * anchor_index,
  double * robot_to_route_distance)
{
  if(anchor_index == nullptr || robot_to_route_distance == nullptr || global_plan_.empty()){
    return false;
  }

  const std::size_t current_index = std::min(local_route_progress_index_, global_plan_.size() - 1);
  const std::size_t search_window =
    std::max<std::size_t>(1, causal_prune_search_window_);
  const std::size_t search_end =
    std::min(current_index + search_window, global_plan_.size() - 1);

  bool have_candidate = false;
  std::size_t best_index = current_index;
  double best_lateral_distance = std::numeric_limits<double>::infinity();

  for(std::size_t idx = current_index; idx <= search_end; ++idx){
    const std::size_t index_jump = idx - current_index;
    const double arc_jump = getGlobalPlanArcDistance(current_index, idx);
    const double lateral_distance = getRouteSegmentLateralDistance(idx);
    const double heading_error = getRouteHeadingError(idx);

    // Local causal prune must follow route order. It must not jump to a later
    // segment that is only Euclidean-close.
    const bool heading_ok =
      heading_error <= causal_prune_max_heading_error_ || index_jump <= 1;
    const bool accept_candidate =
      index_jump <= causal_prune_max_index_jump_ &&
      arc_jump <= causal_prune_max_arc_jump_ &&
      lateral_distance <= causal_prune_max_lateral_distance_ &&
      heading_ok;

    if(!accept_candidate){
      continue;
    }

    if(!have_candidate || lateral_distance + 1e-6 < best_lateral_distance ||
       (std::fabs(lateral_distance - best_lateral_distance) <= 1e-6 && idx < best_index)){
      have_candidate = true;
      best_index = idx;
      best_lateral_distance = lateral_distance;
    }
  }

  if(have_candidate){
    *anchor_index = best_index;
    *robot_to_route_distance = best_lateral_distance;
    return true;
  }

  const double fallback_distance = getRouteSegmentLateralDistance(current_index);
  if(fallback_distance <= causal_prune_max_lateral_distance_ * 1.5){
    *anchor_index = current_index;
    *robot_to_route_distance = fallback_distance;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "causal prune rejected jump, keeping current progress index, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_);
    return true;
  }

  const std::size_t relaxed_step_end = std::min(
    current_index + std::min<std::size_t>(causal_prune_max_index_jump_, 3),
    global_plan_.size() - 1);
  for(std::size_t idx = current_index + 1; idx <= relaxed_step_end; ++idx){
    const double lateral_distance = getRouteSegmentLateralDistance(idx);
    if(lateral_distance <= causal_prune_max_lateral_distance_ * 1.25){
      *anchor_index = idx;
      *robot_to_route_distance = lateral_distance;
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "causal prune accepted only a small progress step, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, next_index=%zu",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        idx);
      return true;
    }
  }

  *robot_to_route_distance = fallback_distance;
  return false;
}

bool Local_Planner::buildPrunedPlanAroundIndex(
  std::size_t anchor_index,
  double forward_distance,
  double backward_distance,
  nav_msgs::msg::Path * prune_plan,
  pcl::PointCloud<pcl::PointXYZI> * pcl_prune_plan) const
{
  if(prune_plan == nullptr || pcl_prune_plan == nullptr || global_plan_.empty()){
    return false;
  }

  anchor_index = std::min(anchor_index, global_plan_.size() - 1);
  prune_plan->poses.clear();
  pcl_prune_plan->clear();
  prune_plan->header.frame_id = global_frame_;
  prune_plan->header.stamp = clock_->now();

  geometry_msgs::msg::PoseStamped last_pose = global_plan_[anchor_index];
  for(std::size_t offset = 0; offset <= anchor_index; ++offset){
    const std::size_t i = anchor_index - offset;
    prune_plan->poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x;
    pt.y = global_plan_[i].pose.position.y;
    pt.z = global_plan_[i].pose.position.z;
    pt.intensity = -1.0;
    pcl_prune_plan->points.push_back(pt);
    if(i < anchor_index){
      backward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(backward_distance < 0.0 || i == 0){
      break;
    }
  }

  std::reverse(prune_plan->poses.begin(), prune_plan->poses.end());

  last_pose = global_plan_[anchor_index];
  for(std::size_t i = anchor_index; i < global_plan_.size(); ++i){
    prune_plan->poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x;
    pt.y = global_plan_[i].pose.position.y;
    pt.z = global_plan_[i].pose.position.z;
    pt.intensity = (i == 0) ? 0.0 : 1.0;
    pcl_prune_plan->points.push_back(pt);
    if(i > anchor_index){
      forward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(forward_distance < 0.0){
      break;
    }
  }

  for(auto & pose : prune_plan->poses){
    pose.header = prune_plan->header;
  }

  return !prune_plan->poses.empty();
}

bool Local_Planner::tryReuseCachedPrunedPath()
{
  if(!cached_local_pruned_path_valid_){
    return false;
  }

  const double cached_age = (clock_->now() - cached_local_pruned_path_stamp_).seconds();
  if(cached_age > cached_pruned_path_timeout_sec_){
    return false;
  }

  if(consecutive_prune_failure_cycles_ > max_prune_failure_cycles_){
    return false;
  }

  prune_plan_ = cached_local_pruned_path_;
  prune_plan_.header.frame_id = global_frame_;
  prune_plan_.header.stamp = clock_->now();
  for(auto & pose : prune_plan_.poses){
    pose.header = prune_plan_.header;
  }
  pcl_prune_plan_ = cached_pcl_prune_plan_;
  last_prune_used_cache_ = true;
  last_valid_prune_plan_ = clock_->now();
  publishDebugPath(
    prune_plan_,
    pub_local_pruned_path_,
    "local_reference_path",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);
  RCLCPP_WARN_THROTTLE(
    this->get_logger().get_child(name_), *clock_, 2000,
    "reuse cached local pruned path due to transient prune failure, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu",
    route_version_,
    goal_seq_,
    route_source_label_.c_str(),
    local_route_progress_index_,
    consecutive_prune_failure_cycles_);
  return true;
}

bool Local_Planner::buildHeadingReferenceFromPoses(
  const geometry_msgs::msg::PoseStamped & first_pose,
  const geometry_msgs::msg::PoseStamped & last_pose,
  tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr){
    return false;
  }

  const double vx = last_pose.pose.position.x - first_pose.pose.position.x;
  const double vy = last_pose.pose.position.y - first_pose.pose.position.y;
  const double vz = last_pose.pose.position.z - first_pose.pose.position.z;
  const double distance = std::sqrt(vx * vx + vy * vy + vz * vz);
  if(distance < 1e-6){
    return false;
  }

  tf2::Quaternion q;
  if(std::fabs(vz) > 1e-6){
    tf2::Vector3 axis_vector(vx / distance, vy / distance, vz / distance);
    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    if(right_vector.length2() < 1e-6){
      return false;
    }
    right_vector.normalize();
    tf2::Quaternion q_pre(right_vector, -1.0 * std::acos(axis_vector.dot(up_vector)));
    q_pre.normalize();
    q = q_pre;
  }
  else{
    tf2::Quaternion q_pre;
    q_pre.setRPY(0.0, 0.0, std::atan2(vy, vx));
    q = q_pre;
  }

  reference_pose->setRotation(q);
  reference_pose->setOrigin(tf2::Vector3(
    first_pose.pose.position.x,
    first_pose.pose.position.y,
    first_pose.pose.position.z));
  return true;
}

bool Local_Planner::buildHeadingReferenceFromPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  std::size_t start_index,
  double min_reference_length,
  tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr || plan.size() < 2){
    return false;
  }

  start_index = std::min(start_index, plan.size() - 1);
  double accumulated_distance = 0.0;
  for(std::size_t end_index = start_index + 1; end_index < plan.size(); ++end_index){
    accumulated_distance += getDistanceBTWPoseStamp(plan[end_index - 1], plan[end_index]);
    if(accumulated_distance + 1e-6 < min_reference_length){
      continue;
    }
    if(buildHeadingReferenceFromPoses(plan[start_index], plan[end_index], reference_pose)){
      return true;
    }
  }

  return false;
}

bool Local_Planner::buildHeadingReferenceFromPoseOrientation(
  const geometry_msgs::msg::PoseStamped & pose,
  tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr){
    return false;
  }

  const auto & q_msg = pose.pose.orientation;
  const double quat_norm =
    q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
  if(quat_norm < 1e-6){
    return false;
  }

  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  q.normalize();
  reference_pose->setRotation(q);
  reference_pose->setOrigin(tf2::Vector3(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z));
  return true;
}

void Local_Planner::cacheHeadingReference(const tf2::Transform & reference_pose)
{
  last_heading_reference_pose_ = reference_pose;
  last_heading_reference_valid_ = true;
  heading_reference_stale_cycles_ = 0;
  consecutive_heading_reference_failure_cycles_ = 0;
}

double Local_Planner::updateHeadingDeviation(const tf2::Transform & reference_pose)
{
  const double yaw = getShortestAngleFromPose2RobotHeading(reference_pose);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  return yaw;
}

void Local_Planner::parseCuboid(){
  marker_edge_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();;
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3; marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9; marker_edge_.color.g = 1; marker_edge_.color.b = 0; marker_edge_.color.a = 0.8;
  //@ parse cuboid, currently the cuboid in local planner is just for visualization
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {"cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt", "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;

  for(auto it=cuboid_vertex_queue.begin(); it!=cuboid_vertex_queue.end();it++){
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param= this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {"cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt", "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
                                                      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb", "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for(auto it=cuboid_vertex_connect.begin(); it!=cuboid_vertex_connect.end();it++){
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }

  robot_cuboid_vertices_.clear();
  const std::vector<std::string> collision_model_order = {
    "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.flb",
    "cuboid.brt", "cuboid.frt", "cuboid.flt", "cuboid.frb"};
  robot_cuboid_vertices_.reserve(collision_model_order.size());
  for(const auto & name : collision_model_order){
    const auto & p = cuboid_vertex_parameter_map[name];
    pcl::PointXYZ pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    robot_cuboid_vertices_.push_back(pt);
  }
}

void Local_Planner::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_ = *msg;
  updateGlobalPose();
  if(compute_best_trajectory_in_odomCb_){
    if(controller_backend_ == "regulated_pure_pursuit"){
      geometry_msgs::msg::Twist cmd_vel;
      computeRppControlCommand("rpp_preview", &cmd_vel);
    }
    else{
      base_trajectory::Trajectory best_traj;
      computeVelocityCommand("differential_drive_simple", best_traj);
    }
  }
  got_odom_ = true;
}


double Local_Planner::getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose){

  //@Transform trans_gbl2b_ to tf2; Get baselink to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2b;
  tf2::fromMsg(trans_gbl2b_, tf2_trans_gbl2b);
  auto tf2_trans_gbl2b_inverse = tf2_trans_gbl2b.inverse();
  //@Get baselink to last pose
  tf2::Transform tf2_baselink2prunelastpose;
  tf2_baselink2prunelastpose.mult(tf2_trans_gbl2b_inverse, m_pose);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_baselink2prunelastpose.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //@Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  
  return yaw;

}

bool Local_Planner::isInitialHeadingAligned(){
  prunePlan(heading_tracking_distance_, 0.0);
  return isInitialHeadingAlignedOnCurrentPrunePlan();
}

bool Local_Planner::isInitialHeadingAlignedOnCurrentPrunePlan(){
  tf2::Transform heading_reference_pose;
  bool have_heading_reference = false;

  if(buildHeadingReferenceFromPlan(prune_plan_.poses, 0, heading_tracking_distance_, &heading_reference_pose)){
    cacheHeadingReference(heading_reference_pose);
    have_heading_reference = true;
  }
  else if(buildHeadingReferenceFromPlan(
            global_plan_,
            local_route_progress_index_,
            min_heading_reference_length_,
            &heading_reference_pose)){
    cacheHeadingReference(heading_reference_pose);
    have_heading_reference = true;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "heading check degraded to route tangent, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_);
  }
  else if(last_heading_reference_valid_ &&
          heading_reference_stale_cycles_ < max_heading_reference_stale_cycles_){
    heading_reference_pose = last_heading_reference_pose_;
    have_heading_reference = true;
    ++heading_reference_stale_cycles_;
    consecutive_heading_reference_failure_cycles_ = 0;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "heading check reused previous valid heading reference, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, stale_cycles=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      heading_reference_stale_cycles_);
  }
  else if(!prune_plan_.poses.empty() &&
          buildHeadingReferenceFromPlan(
            prune_plan_.poses, 0, min_heading_reference_length_, &heading_reference_pose)){
    cacheHeadingReference(heading_reference_pose);
    have_heading_reference = true;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "heading check degraded to short local prune tangent, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_);
  }
  else if(!prune_plan_.poses.empty() &&
          buildHeadingReferenceFromPoseOrientation(prune_plan_.poses.back(), &heading_reference_pose)){
    cacheHeadingReference(heading_reference_pose);
    have_heading_reference = true;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "heading check degraded to local path tail orientation, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_);
  }

  if(!have_heading_reference){
    ++consecutive_heading_reference_failure_cycles_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "heading check failed after fallback, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      consecutive_heading_reference_failure_cycles_);
    return false;
  }

  const double yaw = updateHeadingDeviation(heading_reference_pose);
  RCLCPP_DEBUG(
    this->get_logger().get_child(name_),
    "Heading difference from the prune plan starting at %.2f is %.2f",
    heading_tracking_distance_,
    yaw);

  return std::fabs(yaw) < heading_align_angle_;
}

bool Local_Planner::isRouteStartFrontReachableOnCurrentPrunePlan(std::string * detail) const
{
  if(detail != nullptr){
    detail->clear();
  }

  if(prune_plan_.poses.size() < 2){
    if(detail != nullptr){
      *detail = "route_start_front_check_failed: local_reference_path_too_short";
    }
    return false;
  }

  const double lookahead_distance = std::max(0.10, route_start_front_reach_distance_);
  geometry_msgs::msg::PoseStamped lookahead_pose;
  if(!selectLookaheadPose(lookahead_distance, &lookahead_pose)){
    if(detail != nullptr){
      *detail = "route_start_front_check_failed: lookahead_pose_unavailable";
    }
    return false;
  }

  const double robot_x = trans_gbl2b_.transform.translation.x;
  const double robot_y = trans_gbl2b_.transform.translation.y;
  const double robot_yaw = getRobotYaw();

  const auto project_to_robot_x =
    [robot_x, robot_y, robot_yaw](const geometry_msgs::msg::PoseStamped & pose) {
      const double dx = pose.pose.position.x - robot_x;
      const double dy = pose.pose.position.y - robot_y;
      return std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    };

  const double lookahead_x_local = project_to_robot_x(lookahead_pose);
  if(!std::isfinite(lookahead_x_local) ||
     lookahead_x_local < route_start_front_projection_threshold_)
  {
    if(detail != nullptr){
      *detail = "route_start_front_check_failed: lookahead_not_in_front";
    }
    return false;
  }

  const double reach_window_sq = lookahead_distance * lookahead_distance;
  bool has_front_reachable_pose = false;
  for(const auto & pose : prune_plan_.poses){
    const double dx = pose.pose.position.x - robot_x;
    const double dy = pose.pose.position.y - robot_y;
    const double distance_sq = dx * dx + dy * dy;
    if(distance_sq > reach_window_sq){
      continue;
    }
    if(project_to_robot_x(pose) >= route_start_front_projection_threshold_){
      has_front_reachable_pose = true;
      break;
    }
  }

  if(!has_front_reachable_pose){
    if(detail != nullptr){
      *detail = "route_start_front_check_failed: no_front_reachable_pose_in_start_window";
    }
    return false;
  }

  return true;
}

dddmr_sys_core::RouteStartupStatus Local_Planner::evaluateRouteStartupStatus(std::string * detail)
{
  if(detail != nullptr){
    detail->clear();
  }

  if(global_plan_.size() < 2){
    if(detail != nullptr){
      *detail = "route_start_alignment_failed: global_route_unavailable";
    }
    return dddmr_sys_core::RouteStartupStatus::kRouteUnavailable;
  }

  const double startup_check_distance =
    std::max(heading_tracking_distance_, route_start_front_reach_distance_);
  if(!prunePlan(startup_check_distance, 0.0)){
    if(detail != nullptr){
      *detail = "route_start_alignment_failed: local_reference_unavailable";
    }
    return dddmr_sys_core::RouteStartupStatus::kRouteUnavailable;
  }

  std::string front_detail;
  if(!isRouteStartFrontReachableOnCurrentPrunePlan(&front_detail)){
    if(detail != nullptr){
      *detail = front_detail;
    }
    return dddmr_sys_core::RouteStartupStatus::kFrontUnreachable;
  }

  if(!isInitialHeadingAlignedOnCurrentPrunePlan()){
    if(detail != nullptr){
      *detail = "route_start_alignment_failed: heading_tolerance_unsatisfied";
    }
    return dddmr_sys_core::RouteStartupStatus::kAlignmentUnsatisfied;
  }

  return dddmr_sys_core::RouteStartupStatus::kAligned;
}

bool Local_Planner::isGoalHeadingAligned(){

  if(global_plan_.empty()){
    return false;
  }

  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();

  geometry_msgs::msg::TransformStamped final_pose_ts;
  final_pose_ts.header = final_pose.header;
  final_pose_ts.transform.translation.x = final_pose.pose.position.x;
  final_pose_ts.transform.translation.y = final_pose.pose.position.y;
  final_pose_ts.transform.translation.z = final_pose.pose.position.z;
  final_pose_ts.transform.rotation.x = final_pose.pose.orientation.x;
  final_pose_ts.transform.rotation.y = final_pose.pose.orientation.y;
  final_pose_ts.transform.rotation.z = final_pose.pose.orientation.z;
  final_pose_ts.transform.rotation.w = final_pose.pose.orientation.w;
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2goal;
  tf2::fromMsg(final_pose_ts, tf2_trans_gbl2goal);  

  //@Update the value to critics that allow the robot to turn by shortest angle
  double yaw = getShortestAngleFromPose2RobotHeading(tf2_trans_gbl2goal);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Heading difference to goal is %.2f", yaw);

  if(fabs(yaw) < yaw_goal_tolerance_)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalReached(){
  if(global_plan_.empty()){
    return false;
  }
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();
  double dx = trans_gbl2b_.transform.translation.x - final_pose.pose.position.x;
  double dy = trans_gbl2b_.transform.translation.y - final_pose.pose.position.y;
  double dz = trans_gbl2b_.transform.translation.z - final_pose.pose.position.z;
  double distance = sqrt(dx*dx + dy*dy + dz*dz);
  if(xy_goal_tolerance_>distance)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalPositionReached()
{
  return isGoalReached();
}

bool Local_Planner::isRouteStartAligned()
{
  return evaluateRouteStartupStatus(nullptr) == dddmr_sys_core::RouteStartupStatus::kAligned;
}

bool Local_Planner::isGoalHeadingSatisfied()
{
  return isGoalHeadingAligned();
}

void Local_Planner::setRoute(
  const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label)
{
  if(
    !global_plan_.empty() &&
    route_version_ == route_version &&
    goal_seq_ == goal_seq &&
    route_source_label_ == source_label)
  {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 5000,
      "reuse delivered route in route controller, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_);
    return;
  }

  publishDebugPath(
    buildPathFromPlan(orig_global_plan),
    pub_route_sent_to_local_,
    "controller_route_reference",
    route_version,
    goal_seq,
    source_label,
    false);

  if(orig_global_plan.size()<3){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "route has fewer than 3 poses, route_version=%zu, goal_seq=%zu, source=%s",
      route_version,
      goal_seq,
      source_label.c_str());
    return;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;
  route_version_ = route_version;
  goal_seq_ = goal_seq;
  route_source_label_ = source_label;
  resetLocalRouteTrackingState();

  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto gbl_it = global_plan_.begin(); gbl_it!=global_plan_.end();gbl_it++){
    pcl::PointXYZ pt;
    pt.x = (*gbl_it).pose.position.x;
    pt.y = (*gbl_it).pose.position.y;
    pt.z = (*gbl_it).pose.position.z;
    pcl_global_plan_->push_back(pt);
  }

  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree_global_plan_->setInputCloud (pcl_global_plan_);
  rebuildGlobalPlanArcLengths();
  RCLCPP_DEBUG(
    this->get_logger().get_child(name_),
    "receive new route reference, route_version=%zu, goal_seq=%zu, source=%s, poses=%zu, local_route_progress_index=%zu",
    route_version_,
    goal_seq_,
    route_source_label_.c_str(),
    global_plan_.size(),
    local_route_progress_index_);
}

void Local_Planner::setPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label)
{
  setRoute(orig_global_plan, route_version, goal_seq, source_label);
}

double Local_Planner::getDistanceBTWPoseStamp(
  const geometry_msgs::msg::PoseStamped& a,
  const geometry_msgs::msg::PoseStamped& b) const
{

  double dx = a.pose.position.x-b.pose.position.x;
  double dy = a.pose.position.y-b.pose.position.y;
  double dz = a.pose.position.z-b.pose.position.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

void Local_Planner::updateGlobalPose(){
  try
  {
    trans_gbl2b_ = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "%s: %s", name_.c_str(),e.what());
  }
  robot_cuboid_.markers.clear();
  marker_edge_.header.stamp = trans_gbl2b_.header.stamp;
  robot_cuboid_.markers.push_back(marker_edge_);
  if(debug_publish_robot_cuboid_ && pub_robot_cuboid_->get_subscription_count() > 0){
    pub_robot_cuboid_->publish(robot_cuboid_);
  }
}

geometry_msgs::msg::TransformStamped Local_Planner::getGlobalPose(){
  return trans_gbl2b_;
}

bool Local_Planner::prunePlan(double forward_distance, double backward_distance){

  prune_plan_.poses.clear();
  pcl_prune_plan_.clear();
  prune_plan_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan_.header.stamp = clock_->now();
  last_prune_used_cache_ = false;

  if(global_plan_.size() < 3){
    ++consecutive_prune_failure_cycles_;
    last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
    if(tryReuseCachedPrunedPath()){
      return true;
    }
    publishDebugPath(
      prune_plan_,
      pub_local_pruned_path_,
      "local_reference_path",
      route_version_,
      goal_seq_,
      route_source_label_,
      true);
    return false;
  }

  std::size_t anchor_index = local_route_progress_index_;
  double robot_to_route_distance = std::numeric_limits<double>::infinity();
  const bool have_anchor = selectCausalPruneAnchor(&anchor_index, &robot_to_route_distance);
  last_robot_to_route_distance_ = robot_to_route_distance;

  if(have_anchor &&
     buildPrunedPlanAroundIndex(anchor_index, forward_distance, backward_distance, &prune_plan_, &pcl_prune_plan_)){
    local_route_progress_index_ = std::max(local_route_progress_index_, anchor_index);
    consecutive_prune_failure_cycles_ = 0;
    cached_local_pruned_path_valid_ = true;
    cached_local_pruned_path_ = prune_plan_;
    cached_local_pruned_path_stamp_ = clock_->now();
    cached_pcl_prune_plan_ = pcl_prune_plan_;
    last_valid_prune_plan_ = clock_->now();
  }
  else{
    ++consecutive_prune_failure_cycles_;
    if(tryReuseCachedPrunedPath()){
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "transient prune failure, not treating as deviation yet, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      consecutive_prune_failure_cycles_,
      last_robot_to_route_distance_);
    publishDebugPath(
      prune_plan_,
      pub_local_pruned_path_,
      "local_reference_path",
      route_version_,
      goal_seq_,
      route_source_label_,
      true);
    return false;
  }

  if(debug_publish_prune_plan_ && pub_prune_plan_->get_subscription_count() > 0){
    pub_prune_plan_->publish(prune_plan_);
  }
  publishDebugPath(
    prune_plan_,
    pub_local_pruned_path_,
    "local_reference_path",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);
  return true;
}

void Local_Planner::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

  //@ in case we have collision
  best_traj.cost_ = -1;

  double minimum_cost = 9999999;
  const bool publish_accepted_trajectory =
    debug_publish_accepted_trajectory_ &&
    pub_accepted_trajectory_pose_array_->get_subscription_count() > 0;
  const bool publish_best_trajectory =
    debug_publish_best_trajectory_ &&
    pub_best_trajectory_pose_->get_subscription_count() > 0;
  geometry_msgs::msg::PoseArray accepted_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  for(auto traj_it=trajectories_->begin();traj_it!=trajectories_->end();traj_it++){

    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));

    if((*traj_it).cost_>=0){
      constexpr double kCostTieEpsilon = 1e-6;
      bool strictly_better = (*traj_it).cost_ + kCostTieEpsilon < minimum_cost;
      bool cost_tie = std::fabs((*traj_it).cost_ - minimum_cost) <= kCostTieEpsilon;
      bool straighter_tie_break =
        best_traj.cost_ >= 0 &&
        std::fabs((*traj_it).thetav_) + kCostTieEpsilon < std::fabs(best_traj.thetav_);
      bool faster_tie_break =
        best_traj.cost_ >= 0 &&
        std::fabs(std::fabs((*traj_it).thetav_) - std::fabs(best_traj.thetav_)) <= kCostTieEpsilon &&
        (*traj_it).xv_ > best_traj.xv_ + kCostTieEpsilon;

      // Trajectory samples are iterated from negative yaw rate to positive yaw
      // rate. Using <= here makes equal-cost ties drift toward the last sample,
      // which biases the robot to the left. Prefer straighter, then faster
      // trajectories when the critic score is effectively tied.
      if(strictly_better || (cost_tie && (straighter_tie_break || faster_tie_break))){
        best_traj = (*traj_it);
        minimum_cost = (*traj_it).cost_;
      }
    }

    if(publish_accepted_trajectory && (*traj_it).cost_>=0){
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr, cuboids_pcl);
    }

  }

  if(best_traj.cost_ < 0 && !trajectories_->empty()){
    size_t rejected_collision = 0;
    size_t rejected_pure_pursuit = 0;
    size_t rejected_toward_global_plan = 0;
    size_t rejected_other = 0;

    for(const auto& traj : *trajectories_){
      if(traj.cost_ == -1.0){
        rejected_collision++;
      }
      else if(traj.cost_ == -4.0){
        rejected_pure_pursuit++;
      }
      else if(traj.cost_ == -12.0){
        rejected_toward_global_plan++;
      }
      else if(traj.cost_ < 0.0){
        rejected_other++;
      }
    }

    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 2000,
      "Trajectory rejection summary: collision=%zu, pure_pursuit=%zu, toward_global_plan=%zu, other=%zu, total=%zu",
      rejected_collision, rejected_pure_pursuit, rejected_toward_global_plan, rejected_other, trajectories_->size());
  }

  if(publish_accepted_trajectory){
    accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    accepted_pose_arr.header.stamp = clock_->now();
    pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);
  }

  if(publish_best_trajectory){
    geometry_msgs::msg::PoseArray best_pose_arr;
    trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
    best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  nav_msgs::msg::Path best_trajectory_path;
  if(best_traj.cost_ >= 0){
    best_trajectory_path = buildPathFromTrajectory(best_traj);
  }
  else{
    best_trajectory_path.header.frame_id = global_frame_;
    best_trajectory_path.header.stamp = clock_->now();
  }
  publishDebugPath(
    best_trajectory_path,
    pub_best_trajectory_path_,
    "best_control_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

}

dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){
  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return context_state;
  }

  //Do not create a function to set the parameters unless a nice structure is found
  //Below assignment of variables is useful when migrate to ROS2
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ 
                  = context.current_allowed_max_linear_speed;

  trajectory_generators_ros_->initializeTheories_wi_Shared_data();

  const bool publish_all_trajectories =
    debug_publish_all_trajectories_ &&
    pub_trajectory_pose_array_->get_subscription_count() > 0;
  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  #endif

  //@ We queue all trajectories in trajectories_, then score them one by one in getBestTrajectory()
  while(trajectory_generators_ros_->hasMoreTrajectories(traj_gen_name)){
    base_trajectory::Trajectory a_traj;
    if(trajectory_generators_ros_->nextTrajectory(traj_gen_name, a_traj)){
      //@ collected all trajectories here, for later scoring
      trajectories_->push_back(a_traj);
      if(publish_all_trajectories){
        trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
      }
    }

  }

  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  RCLCPP_DEBUG(this->get_logger(), "Trajectory generation time: %.9f", t_diff);
  #endif

  if(publish_all_trajectories){
    pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    pose_arr.header.stamp = clock_->now();
    pub_trajectory_pose_array_->publish(pose_arr);
  }

  
  //cuboids_pcl.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  //pub_cuboids_.publish(cuboids_pcl);
  

  //@Update data for critics
  {
    std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(
      *(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
    updateCriticSharedData(
      context.aggregate_observation,
      context.aggregate_observation_kdtree);
    getBestTrajectory(traj_gen_name, best_traj);
  }

  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full control cycle time: %.9f", t_diff.seconds());

  // if(t_diff.seconds() > 1./controller_frequency_){
  //   RCLCPP_WARN(this->get_logger().get_child(name_), "Local planner control time exceed expect time: %.2f but is %.2f", 1./controller_frequency_, t_diff.seconds());
  // }
  
  const auto opinion_state = evaluatePerceptionOpinions(context.opinions);
  if(opinion_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return opinion_state;
  }

  if(best_traj.cost_<0){
    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "All trajectories are rejected by critics.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }
  else{
    return dddmr_sys_core::TRAJECTORY_FOUND;
  }
  
  //@ Reset kd tree/observations because it is shared_ptr and copied from perception_ros
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
}

dddmr_sys_core::PlannerState Local_Planner::computeRppControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return context_state;
  }

  const bool goal_alignment_mode = isGoalReached();
  const double distance_to_goal = getDistanceToGoal();
  double linear_speed_cap = rpp_nominal_linear_speed_;
  if(context.current_allowed_max_linear_speed > 0.0){
    linear_speed_cap = std::min(linear_speed_cap, context.current_allowed_max_linear_speed);
  }

  if(goal_alignment_mode){
    linear_speed_cap = std::min(linear_speed_cap, rpp_alignment_linear_speed_);
  }
  else if(distance_to_goal < rpp_goal_slowdown_distance_ && rpp_goal_slowdown_distance_ > 1e-3){
    const double ratio = std::max(0.0, std::min(1.0, distance_to_goal / rpp_goal_slowdown_distance_));
    const double slowdown_floor = std::min(rpp_min_linear_speed_, linear_speed_cap);
    linear_speed_cap = slowdown_floor + (linear_speed_cap - slowdown_floor) * ratio;
  }

  if(linear_speed_cap <= 1e-3){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "RPP speed cap collapsed to zero, route_version=%zu, goal_seq=%zu, source=%s",
      route_version_,
      goal_seq_,
      route_source_label_.c_str());
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  const double max_curvature = getMaxAckermannCurvature();
  double curvature = 0.0;
  if(goal_alignment_mode){
    tf2::Transform reference_pose;
    if(!buildGoalAlignmentReference(&reference_pose)){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP goal alignment reference is unavailable, route_version=%zu, goal_seq=%zu, source=%s",
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    const double heading_error = getShortestAngleFromPose2RobotHeading(reference_pose);
    mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = heading_error;
    curvature =
      2.0 * std::sin(heading_error) / std::max(0.05, rpp_alignment_lookahead_distance_);
  }
  else{
    const double current_speed = std::fabs(robot_state_.twist.twist.linear.x);
    double lookahead_distance = std::max(
      rpp_min_lookahead_distance_,
      current_speed * rpp_lookahead_time_);
    lookahead_distance = std::min(lookahead_distance, rpp_max_lookahead_distance_);

    geometry_msgs::msg::PoseStamped lookahead_pose;
    if(!selectLookaheadPose(lookahead_distance, &lookahead_pose)){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP lookahead target is unavailable, route_version=%zu, goal_seq=%zu, source=%s",
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    const double robot_yaw = getRobotYaw();
    const double dx = lookahead_pose.pose.position.x - trans_gbl2b_.transform.translation.x;
    const double dy = lookahead_pose.pose.position.y - trans_gbl2b_.transform.translation.y;
    const double x_local = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double y_local = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double distance_sq = x_local * x_local + y_local * y_local;

    if(distance_sq < 1e-6 || x_local <= 0.05){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP lookahead target is not reachable in front of the robot, controller=%s, route_version=%zu, goal_seq=%zu, source=%s",
        controller_name.c_str(),
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    curvature = (2.0 * y_local) / distance_sq;
  }

  curvature = std::max(-max_curvature, std::min(max_curvature, curvature));

  double linear_velocity = linear_speed_cap;
  if(std::fabs(curvature) > 1e-6 && rpp_max_lateral_accel_ > 0.0){
    linear_velocity = std::min(
      linear_velocity,
      std::sqrt(rpp_max_lateral_accel_ / std::fabs(curvature)));
  }
  if(std::fabs(curvature) > 1e-6 && rpp_max_angular_velocity_ > 0.0){
    linear_velocity = std::min(
      linear_velocity,
      rpp_max_angular_velocity_ / std::fabs(curvature));
  }

  const double min_linear_speed =
    std::min(goal_alignment_mode ? rpp_alignment_linear_speed_ : rpp_min_linear_speed_, linear_speed_cap);
  linear_velocity = std::max(min_linear_speed, std::min(linear_velocity, linear_speed_cap));
  const double angular_velocity =
    std::max(-rpp_max_angular_velocity_,
      std::min(rpp_max_angular_velocity_, linear_velocity * curvature));

  auto predicted_traj = buildPredictedTrajectory(linear_velocity, angular_velocity);
  {
    std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(
      *(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
    updateCriticSharedData(
      context.aggregate_observation,
      context.aggregate_observation_kdtree);
    mpc_critics_ros_->scoreTrajectory(rpp_critic_trajectory_generator_name_, predicted_traj);
  }

  publishDebugPath(
    buildPathFromTrajectory(predicted_traj),
    pub_best_trajectory_path_,
    "best_control_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

  if(debug_publish_best_trajectory_ && pub_best_trajectory_pose_->get_subscription_count() > 0){
    geometry_msgs::msg::PoseArray best_pose_arr;
    pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
    trajectory2posearray_cuboids(predicted_traj, best_pose_arr, cuboids_pcl);
    best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  const auto opinion_state = evaluatePerceptionOpinions(context.opinions);
  if(opinion_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return opinion_state;
  }

  if(predicted_traj.cost_ < 0.0){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "RPP control trajectory was rejected by critics, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, cost=%.2f",
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      predicted_traj.cost_);
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  if(cmd_vel != nullptr){
    cmd_vel->linear.x = linear_velocity;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = angular_velocity;
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::computeControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  if(cmd_vel != nullptr){
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = 0.0;
  }

  if(controller_backend_ == "regulated_pure_pursuit"){
    return computeRppControlCommand(controller_name, cmd_vel);
  }

  base_trajectory::Trajectory best_traj;
  const auto planner_state = computeVelocityCommand(controller_name, best_traj);
  if(planner_state == dddmr_sys_core::TRAJECTORY_FOUND && cmd_vel != nullptr){
    cmd_vel->linear.x = best_traj.xv_;
    cmd_vel->linear.y = best_traj.yv_;
    cmd_vel->angular.z = best_traj.thetav_;
  }
  return planner_state;
}

void Local_Planner::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPointsSize();i++){
      auto p = a_traj.getPoint(i);
      pose_arr.poses.push_back(p.pose);
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);       
  }

}

/*
void Local_Planner::cbMCL_ground_normal(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  ground_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromROSMsg(*msg, *ground_with_normals_);
  if(perception_3d_ros_->getGlobalUtils()->getGblFrame().compare(msg->header.frame_id) != 0)
    ROS_ERROR("%s: the global frame is not consistent with topics and perception setting.", name_.c_str());
  global_frame_ = msg->header.frame_id;
  normal2quaternion();
}

void Local_Planner::normal2quaternion(){

  visualization_msgs::MarkerArray markerArray;
  for(size_t i=0;i<ground_with_normals_->points.size();i++){

    tf2::Vector3 axis_vector(ground_with_normals_->points[i].normal_x, ground_with_normals_->points[i].normal_y, ground_with_normals_->points[i].normal_z);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    //@Create arrow
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = ground_with_normals_->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = ground_with_normals_->points[i].x;
    marker.pose.position.y = ground_with_normals_->points[i].y;
    marker.pose.position.z = ground_with_normals_->points[i].z;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3; //scale.x is the arrow length,
    marker.scale.y = 0.05; //scale.y is the arrow width 
    marker.scale.z = 0.1; //scale.z is the arrow height. 

    double angle = atan2(ground_with_normals_->points[i].normal_z, 
                  sqrt(ground_with_normals_->points[i].normal_x*ground_with_normals_->points[i].normal_x+ ground_with_normals_->points[i].normal_y*ground_with_normals_->points[i].normal_y) ) * 180 / 3.1415926535;

    if(fabs(angle)<=10){
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;      
    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f; 
    }

    marker.color.a = 0.6f;   
    markerArray.markers.push_back(marker); 
  }
  pub_pc_normal_.publish(markerArray);
}
*/

}// end of name space
