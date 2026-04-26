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
#include <global_planner/global_planner.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;

namespace global_planner
{

GlobalPlanner::GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
  debug_goal_seq_ = 0;
  debug_route_version_ = 0;
  debug_request_id_ = 0;
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received request to cancel planner goal; computation will stop at the next cancel check.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool GlobalPlanner::clearCurrentHandleIfMatches(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  std::lock_guard<std::mutex> lock(current_handle_mutex_);
  if(current_handle_ != goal_handle){
    return false;
  }
  current_handle_.reset();
  return true;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  {
    std::lock_guard<std::mutex> lock(current_handle_mutex_);
    current_handle_.reset();
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){
  
  static_ground_size_ = 0;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);

  declare_parameter("graph_a_star.turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("graph_a_star.turning_weight", turning_weight_);
  RCLCPP_DEBUG(this->get_logger(), "graph_a_star.turning_weight: %.2f", turning_weight_);

  declare_parameter("enable_detail_log", rclcpp::ParameterValue(false));
  this->get_parameter("enable_detail_log", enable_detail_log_);
  RCLCPP_DEBUG(this->get_logger(), "enable_detail_log: %d", enable_detail_log_);    

  declare_parameter("graph_a_star.a_star_expanding_radius", rclcpp::ParameterValue(0.5));
  this->get_parameter("graph_a_star.a_star_expanding_radius", a_star_expanding_radius_);
  RCLCPP_DEBUG(
    this->get_logger(),
    "graph_a_star.a_star_expanding_radius: %.2f",
    a_star_expanding_radius_);

  declare_parameter("raw_route_backend", rclcpp::ParameterValue(std::string("graph_a_star")));
  this->get_parameter("raw_route_backend", raw_route_backend_);
  std::transform(
    raw_route_backend_.begin(),
    raw_route_backend_.end(),
    raw_route_backend_.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  if(raw_route_backend_ != "graph_a_star" && raw_route_backend_ != "hybrid_a_star"){
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid raw_route_backend '%s', fallback to 'graph_a_star'.",
      raw_route_backend_.c_str());
    raw_route_backend_ = "graph_a_star";
  }
  RCLCPP_DEBUG(this->get_logger(), "raw_route_backend: %s", raw_route_backend_.c_str());

  // Keep graph A* as topology backbone (bridge/ramp traversability) and only
  // smooth the sampled raw route with bounded lateral displacement.
  declare_parameter("graph_a_star.graph_path_smoothing.enable", rclcpp::ParameterValue(true));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.enable",
    graph_path_smoothing_enable_);
  declare_parameter("graph_a_star.graph_path_smoothing.iterations", rclcpp::ParameterValue(6));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.iterations",
    graph_path_smoothing_iterations_);
  graph_path_smoothing_iterations_ = std::max(graph_path_smoothing_iterations_, 0);
  declare_parameter("graph_a_star.graph_path_smoothing.weight", rclcpp::ParameterValue(0.35));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.weight",
    graph_path_smoothing_weight_);
  graph_path_smoothing_weight_ = std::clamp(graph_path_smoothing_weight_, 0.0, 1.0);
  declare_parameter("graph_a_star.graph_path_smoothing.max_shift", rclcpp::ParameterValue(0.30));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.max_shift",
    graph_path_smoothing_max_shift_);
  graph_path_smoothing_max_shift_ = std::max(graph_path_smoothing_max_shift_, 0.0);
  declare_parameter(
    "graph_a_star.graph_path_smoothing.lock_start_distance",
    rclcpp::ParameterValue(1.2));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.lock_start_distance",
    graph_path_smoothing_lock_start_distance_);
  graph_path_smoothing_lock_start_distance_ =
    std::max(graph_path_smoothing_lock_start_distance_, 0.0);
  declare_parameter(
    "graph_a_star.graph_path_smoothing.lock_goal_distance",
    rclcpp::ParameterValue(1.2));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.lock_goal_distance",
    graph_path_smoothing_lock_goal_distance_);
  graph_path_smoothing_lock_goal_distance_ =
    std::max(graph_path_smoothing_lock_goal_distance_, 0.0);
  declare_parameter(
    "graph_a_star.graph_path_smoothing.ground_search_radius",
    rclcpp::ParameterValue(0.8));
  this->get_parameter(
    "graph_a_star.graph_path_smoothing.ground_search_radius",
    graph_path_smoothing_ground_search_radius_);
  graph_path_smoothing_ground_search_radius_ =
    std::max(graph_path_smoothing_ground_search_radius_, 0.1);
  RCLCPP_DEBUG(
    this->get_logger(),
    "graph_path_smoothing: enable=%d, iterations=%d, weight=%.2f, max_shift=%.2f, lock_start=%.2f, lock_goal=%.2f, ground_search_radius=%.2f",
    graph_path_smoothing_enable_ ? 1 : 0,
    graph_path_smoothing_iterations_,
    graph_path_smoothing_weight_,
    graph_path_smoothing_max_shift_,
    graph_path_smoothing_lock_start_distance_,
    graph_path_smoothing_lock_goal_distance_,
    graph_path_smoothing_ground_search_radius_);

  declare_parameter("hybrid_astar.wheelbase", rclcpp::ParameterValue(0.549185));
  this->get_parameter("hybrid_astar.wheelbase", forward_hybrid_astar_config_.wheelbase);
  declare_parameter("hybrid_astar.max_steer", rclcpp::ParameterValue(0.69));
  this->get_parameter("hybrid_astar.max_steer", forward_hybrid_astar_config_.max_steer);
  declare_parameter("hybrid_astar.steer_sample_count", rclcpp::ParameterValue(5));
  this->get_parameter(
    "hybrid_astar.steer_sample_count",
    forward_hybrid_astar_config_.steer_sample_count);
  declare_parameter("hybrid_astar.heading_bin_count", rclcpp::ParameterValue(72));
  this->get_parameter("hybrid_astar.heading_bin_count", forward_hybrid_astar_config_.heading_bin_count);
  declare_parameter("hybrid_astar.primitive_length", rclcpp::ParameterValue(0.6));
  this->get_parameter("hybrid_astar.primitive_length", forward_hybrid_astar_config_.primitive_length);
  declare_parameter("hybrid_astar.primitive_step", rclcpp::ParameterValue(0.05));
  this->get_parameter("hybrid_astar.primitive_step", forward_hybrid_astar_config_.primitive_step);
  declare_parameter("hybrid_astar.projection_search_radius", rclcpp::ParameterValue(0.6));
  this->get_parameter("hybrid_astar.projection_search_radius", forward_hybrid_astar_config_.projection_search_radius);
  declare_parameter("hybrid_astar.goal_position_tolerance", rclcpp::ParameterValue(0.4));
  this->get_parameter("hybrid_astar.goal_position_tolerance", forward_hybrid_astar_config_.goal_position_tolerance);
  declare_parameter("hybrid_astar.goal_heading_tolerance", rclcpp::ParameterValue(0.35));
  this->get_parameter("hybrid_astar.goal_heading_tolerance", forward_hybrid_astar_config_.goal_heading_tolerance);
  declare_parameter("hybrid_astar.use_goal_heading", rclcpp::ParameterValue(false));
  this->get_parameter("hybrid_astar.use_goal_heading", forward_hybrid_astar_config_.use_goal_heading);
  declare_parameter("hybrid_astar.steering_penalty", rclcpp::ParameterValue(0.2));
  this->get_parameter("hybrid_astar.steering_penalty", forward_hybrid_astar_config_.steering_penalty);
  declare_parameter("hybrid_astar.steering_change_penalty", rclcpp::ParameterValue(0.3));
  this->get_parameter("hybrid_astar.steering_change_penalty", forward_hybrid_astar_config_.steering_change_penalty);
  declare_parameter("hybrid_astar.heading_change_penalty", rclcpp::ParameterValue(0.4));
  this->get_parameter("hybrid_astar.heading_change_penalty", forward_hybrid_astar_config_.heading_change_penalty);
  declare_parameter("hybrid_astar.obstacle_penalty_weight", rclcpp::ParameterValue(1.0));
  this->get_parameter("hybrid_astar.obstacle_penalty_weight", forward_hybrid_astar_config_.obstacle_penalty_weight);
  declare_parameter("hybrid_astar.edge_weight_penalty_weight", rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "hybrid_astar.edge_weight_penalty_weight",
    forward_hybrid_astar_config_.edge_weight_penalty_weight);
  declare_parameter("hybrid_astar.edge_weight_safe_threshold", rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "hybrid_astar.edge_weight_safe_threshold",
    forward_hybrid_astar_config_.edge_weight_safe_threshold);
  declare_parameter("hybrid_astar.edge_weight_soft_cap", rclcpp::ParameterValue(8.0));
  this->get_parameter(
    "hybrid_astar.edge_weight_soft_cap",
    forward_hybrid_astar_config_.edge_weight_soft_cap);
  declare_parameter(
    "hybrid_astar.edge_weight_hard_reject_threshold",
    rclcpp::ParameterValue(0.0));
  this->get_parameter(
    "hybrid_astar.edge_weight_hard_reject_threshold",
    forward_hybrid_astar_config_.edge_weight_hard_reject_threshold);
  declare_parameter("hybrid_astar.heuristic_heading_weight", rclcpp::ParameterValue(0.2));
  this->get_parameter("hybrid_astar.heuristic_heading_weight", forward_hybrid_astar_config_.heuristic_heading_weight);
  declare_parameter("hybrid_astar.turn_side_hysteresis_penalty", rclcpp::ParameterValue(0.8));
  this->get_parameter("hybrid_astar.turn_side_hysteresis_penalty", forward_hybrid_astar_config_.turn_side_hysteresis_penalty);
  declare_parameter("hybrid_astar.rearward_check_distance", rclcpp::ParameterValue(1.5));
  this->get_parameter("hybrid_astar.rearward_check_distance", forward_hybrid_astar_config_.rearward_check_distance);
  declare_parameter("hybrid_astar.rearward_allowance", rclcpp::ParameterValue(0.05));
  this->get_parameter("hybrid_astar.rearward_allowance", forward_hybrid_astar_config_.rearward_allowance);
  declare_parameter("hybrid_astar.rearward_excursion_penalty", rclcpp::ParameterValue(4.0));
  this->get_parameter("hybrid_astar.rearward_excursion_penalty", forward_hybrid_astar_config_.rearward_excursion_penalty);
  declare_parameter("hybrid_astar.rearward_hard_reject_distance", rclcpp::ParameterValue(0.20));
  this->get_parameter(
    "hybrid_astar.rearward_hard_reject_distance",
    forward_hybrid_astar_config_.rearward_hard_reject_distance);
  declare_parameter("hybrid_astar.strict_forward_check_distance", rclcpp::ParameterValue(1.5));
  this->get_parameter(
    "hybrid_astar.strict_forward_check_distance",
    forward_hybrid_astar_config_.strict_forward_check_distance);
  declare_parameter("hybrid_astar.min_initial_forward_projection", rclcpp::ParameterValue(0.05));
  this->get_parameter(
    "hybrid_astar.min_initial_forward_projection",
    forward_hybrid_astar_config_.min_initial_forward_projection);
  declare_parameter("hybrid_astar.max_projected_pitch", rclcpp::ParameterValue(0.55));
  this->get_parameter(
    "hybrid_astar.max_projected_pitch",
    forward_hybrid_astar_config_.max_projected_pitch);
  declare_parameter("hybrid_astar.max_projected_vertical_jump", rclcpp::ParameterValue(0.25));
  this->get_parameter(
    "hybrid_astar.max_projected_vertical_jump",
    forward_hybrid_astar_config_.max_projected_vertical_jump);
  declare_parameter(
    "hybrid_astar.allow_sample_nearest_fallback",
    rclcpp::ParameterValue(false));
  this->get_parameter(
    "hybrid_astar.allow_sample_nearest_fallback",
    forward_hybrid_astar_config_.allow_sample_nearest_fallback);

  RCLCPP_DEBUG(
    this->get_logger(),
    "hybrid_astar: wheelbase=%.6f max_steer=%.3f steer_samples=%d heading_bins=%d primitive_len=%.2f primitive_step=%.2f",
    forward_hybrid_astar_config_.wheelbase,
    forward_hybrid_astar_config_.max_steer,
    forward_hybrid_astar_config_.steer_sample_count,
    forward_hybrid_astar_config_.heading_bin_count,
    forward_hybrid_astar_config_.primitive_length,
    forward_hybrid_astar_config_.primitive_step);
  RCLCPP_DEBUG(
    this->get_logger(),
    "hybrid_astar: goal_tol=(%.2f, %.2f) use_goal_heading=%d proj_radius=%.2f",
    forward_hybrid_astar_config_.goal_position_tolerance,
    forward_hybrid_astar_config_.goal_heading_tolerance,
    forward_hybrid_astar_config_.use_goal_heading,
    forward_hybrid_astar_config_.projection_search_radius);
  RCLCPP_DEBUG(
    this->get_logger(),
    "hybrid_astar: strict_forward_check_distance=%.2f min_initial_forward_projection=%.2f max_projected_pitch=%.2f max_projected_vertical_jump=%.2f allow_sample_nearest_fallback=%d edge_weight_penalty_weight=%.2f edge_weight_safe_threshold=%.2f edge_weight_soft_cap=%.2f edge_weight_hard_reject_threshold=%.2f",
    forward_hybrid_astar_config_.strict_forward_check_distance,
    forward_hybrid_astar_config_.min_initial_forward_projection,
    forward_hybrid_astar_config_.max_projected_pitch,
    forward_hybrid_astar_config_.max_projected_vertical_jump,
    forward_hybrid_astar_config_.allow_sample_nearest_fallback ? 1 : 0,
    forward_hybrid_astar_config_.edge_weight_penalty_weight,
    forward_hybrid_astar_config_.edge_weight_safe_threshold,
    forward_hybrid_astar_config_.edge_weight_soft_cap,
    forward_hybrid_astar_config_.edge_weight_hard_reject_threshold);

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_DEBUG(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    


  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::checkPerception3DThread, this), action_server_group_);
  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&GlobalPlanner::cbClickedPoint, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_raw_route_path_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/raw_route_path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_frozen_route_path_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/frozen_route_path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_route_request_stage_ = this->create_publisher<std_msgs::msg::String>(
    "/debug/planner_route_request_stage",
    rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

GlobalPlanner::~GlobalPlanner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  a_star_planner_pre_graph_.reset();
  forward_hybrid_astar_planner_.reset();
  action_server_global_planner_.reset();
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  if(static_ground_size_!=perception_3d_ros_->getSharedDataPtr()->static_ground_size_){
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    *pcl_ground_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_ground_);
    global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    *kdtree_ground_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_ground_);
    *kdtree_map_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_map_);
    *pcl_map_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_map_);
    static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_; //@ node weight
    RCLCPP_DEBUG(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d.");
    getStaticGraphFromPerception3D();
    static_ground_size_ = perception_3d_ros_->getSharedDataPtr()->static_ground_size_;
  }

}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = global_frame_;
  start.header.stamp = clock_->now();
  goal.header.frame_id = global_frame_;
  goal.header.stamp = clock_->now();

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;
  goal.pose.orientation.w = 1.0;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
    start.pose.orientation = transformStamped.transform.rotation;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
    return;
  }
  
  const std::size_t goal_seq = ++debug_goal_seq_;
  nav_msgs::msg::Path raw_route;
  nav_msgs::msg::Path frozen_route;
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!buildFrozenRouteLocked(
         start,
         goal,
         &raw_route,
         &frozen_route))
    {
      frozen_route = raw_route;
    }
  }

  if(raw_route.poses.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found for clicked goal.");
    return;
  }

  const std::size_t route_version = ++debug_route_version_;
  publishRawRouteDebugPath(raw_route, goal_seq, route_version, "clicked_goal_raw_route");
  publishFrozenRouteDebugPath(
    frozen_route,
    goal_seq,
    route_version,
    "clicked_goal_frozen_route");
  pub_path_->publish(frozen_route);
  RCLCPP_DEBUG(this->get_logger(), "Frozen route found for clicked goal.");

}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(auto it=1;it<path_id.size()-1;it++){

    geometry_msgs::msg::PoseStamped next_pst;
    next_pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    next_pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    next_pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    double vx,vy,vz;
    vx = next_pst.pose.position.x - current_pst.pose.position.x;
    vy = next_pst.pose.position.y - current_pst.pose.position.y;
    vz = next_pst.pose.position.z - current_pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    current_pst.pose.orientation.x = q.getX();
    current_pst.pose.orientation.y = q.getY();
    current_pst.pose.orientation.z = q.getZ();
    current_pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    for(double step=0.05;step<0.99;step+=0.05){
      pcl::PointXYZI pst_inter_polate_pc;
      pst_inter_polate_pc.x = current_pst.pose.position.x + vx*step;
      pst_inter_polate_pc.y = current_pst.pose.position.y + vy*step;
      pst_inter_polate_pc.z = current_pst.pose.position.z + vz*step;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree_map_->radiusSearch(pst_inter_polate_pc, perception_3d_ros_->getGlobalUtils()->getInscribedRadius(), pointIdxRadiusSearch, pointRadiusSquaredDistance)>1){
        //@ no line of sight, keep this pose
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      if(kdtree_ground_->radiusSearch(pst_inter_polate_pc, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<2){
        //@ not on the ground
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      double dx = vx*step;
      double dy = vy*step;
      double dr = sqrt(dx*dx+dy*dy);
      double dz = fabs(vz*step);
      float vertical_angle = std::asin(dz / dr);
      if(dr>0.5 && vertical_angle>0.349){
        //@ z jump
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
      if(dr>20.0){
        //@ longer than 10 meter
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
    }
  }
  smoothed_path_id.push_back(path_id[path_id.size()-1]);
}

void GlobalPlanner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1){
      next_pst.pose.position.x = pcl_ground_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pcl_ground_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pcl_ground_->points[path_id[it+1]].z;
    }


    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;

    if(vz!=0){
      double unit = sqrt(vx*vx + vy*vy + vz*vz);
      
      tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      right_vector.normalized();
      tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
      q.normalize();
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }
    else{
      //@ handle with 2D
      double yaw = atan2(vy, vx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }

    //RCLCPP_DEBUG(this->get_logger(), "%.2f, %.2f, %.2f,%.2f, %.2f, %.2f, %.2f", vx, vy, vz, q.getX(), q.getY(), q.getZ(), q.getW());
    //@Interpolation to make global plan smoother and better resolution for local planner
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    if(it<path_id.size()-1){
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      for(double step=0.05;step<0.99;step+=0.05){

        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      ros_path.poses.push_back(pst);
    }
    
  }
}

void GlobalPlanner::refreshPathOrientations(nav_msgs::msg::Path * path) const
{
  if(path == nullptr || path->poses.empty()){
    return;
  }

  if(path->poses.size() == 1){
    return;
  }

  for(std::size_t i = 0; i < path->poses.size(); ++i){
    const std::size_t prev_idx = (i > 0) ? i - 1 : i;
    const std::size_t next_idx = (i + 1 < path->poses.size()) ? i + 1 : i;
    if(prev_idx == next_idx){
      continue;
    }

    const double vx =
      path->poses[next_idx].pose.position.x - path->poses[prev_idx].pose.position.x;
    const double vy =
      path->poses[next_idx].pose.position.y - path->poses[prev_idx].pose.position.y;
    const double vz =
      path->poses[next_idx].pose.position.z - path->poses[prev_idx].pose.position.z;
    const double tangent_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if(tangent_norm < 1e-6){
      continue;
    }

    tf2::Quaternion q;
    if(std::fabs(vz) > 1e-6){
      tf2::Vector3 axis_vector(vx / tangent_norm, vy / tangent_norm, vz / tangent_norm);
      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      const double right_norm = right_vector.length();
      if(right_norm > 1e-6){
        right_vector /= right_norm;
        const double axis_dot =
          std::clamp(static_cast<double>(axis_vector.dot(up_vector)), -1.0, 1.0);
        q = tf2::Quaternion(right_vector, -1.0 * std::acos(axis_dot));
      }
      else{
        q.setRPY(0.0, 0.0, std::atan2(vy, vx));
      }
    }
    else{
      q.setRPY(0.0, 0.0, std::atan2(vy, vx));
    }
    q.normalize();
    path->poses[i].pose.orientation.x = q.getX();
    path->poses[i].pose.orientation.y = q.getY();
    path->poses[i].pose.orientation.z = q.getZ();
    path->poses[i].pose.orientation.w = q.getW();
  }
}

void GlobalPlanner::smoothGraphPathForAckermann(nav_msgs::msg::Path * path) const
{
  if(path == nullptr || path->poses.size() < 5){
    return;
  }
  if(!graph_path_smoothing_enable_ ||
     graph_path_smoothing_iterations_ <= 0 ||
     graph_path_smoothing_weight_ <= 0.0 ||
     graph_path_smoothing_max_shift_ <= 0.0)
  {
    return;
  }

  std::vector<double> arc_lengths(path->poses.size(), 0.0);
  for(std::size_t i = 1; i < path->poses.size(); ++i){
    const double dx =
      path->poses[i].pose.position.x - path->poses[i - 1].pose.position.x;
    const double dy =
      path->poses[i].pose.position.y - path->poses[i - 1].pose.position.y;
    const double dz =
      path->poses[i].pose.position.z - path->poses[i - 1].pose.position.z;
    arc_lengths[i] = arc_lengths[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  const double total_length = arc_lengths.back();
  if(!std::isfinite(total_length) || total_length < 0.5){
    return;
  }

  std::vector<double> reference_x(path->poses.size(), 0.0);
  std::vector<double> reference_y(path->poses.size(), 0.0);
  for(std::size_t i = 0; i < path->poses.size(); ++i){
    reference_x[i] = path->poses[i].pose.position.x;
    reference_y[i] = path->poses[i].pose.position.y;
  }

  double inscribed_radius = 0.3;
  if(perception_3d_ros_ && perception_3d_ros_->getGlobalUtils()){
    inscribed_radius = std::max(
      0.05,
      static_cast<double>(perception_3d_ros_->getGlobalUtils()->getInscribedRadius()));
  }

  std::size_t total_adjusted_points = 0;
  for(int iteration = 0; iteration < graph_path_smoothing_iterations_; ++iteration){
    const auto previous = path->poses;
    bool iteration_changed = false;

    for(std::size_t i = 1; i + 1 < path->poses.size(); ++i){
      if(arc_lengths[i] <= graph_path_smoothing_lock_start_distance_){
        continue;
      }
      if((total_length - arc_lengths[i]) <= graph_path_smoothing_lock_goal_distance_){
        continue;
      }

      const double prev_x = previous[i - 1].pose.position.x;
      const double prev_y = previous[i - 1].pose.position.y;
      const double next_x = previous[i + 1].pose.position.x;
      const double next_y = previous[i + 1].pose.position.y;
      const double curr_x = previous[i].pose.position.x;
      const double curr_y = previous[i].pose.position.y;

      double target_x =
        curr_x + graph_path_smoothing_weight_ * (0.5 * (prev_x + next_x) - curr_x);
      double target_y =
        curr_y + graph_path_smoothing_weight_ * (0.5 * (prev_y + next_y) - curr_y);

      const double reference_dx = target_x - reference_x[i];
      const double reference_dy = target_y - reference_y[i];
      const double reference_shift = std::hypot(reference_dx, reference_dy);
      if(reference_shift > graph_path_smoothing_max_shift_ && reference_shift > 1e-6){
        const double scale = graph_path_smoothing_max_shift_ / reference_shift;
        target_x = reference_x[i] + reference_dx * scale;
        target_y = reference_y[i] + reference_dy * scale;
      }

      const double candidate_step = std::hypot(target_x - curr_x, target_y - curr_y);
      if(candidate_step < 1e-5){
        continue;
      }

      pcl::PointXYZI candidate;
      candidate.x = target_x;
      candidate.y = target_y;
      candidate.z = previous[i].pose.position.z;

      if(kdtree_ground_){
        std::vector<int> ground_indices;
        std::vector<float> ground_squared_distances;
        if(kdtree_ground_->radiusSearch(
             candidate,
             graph_path_smoothing_ground_search_radius_,
             ground_indices,
             ground_squared_distances) < 1)
        {
          continue;
        }
      }

      if(kdtree_map_){
        std::vector<int> map_indices;
        std::vector<float> map_squared_distances;
        if(kdtree_map_->radiusSearch(
             candidate,
             inscribed_radius,
             map_indices,
             map_squared_distances) > 1)
        {
          continue;
        }
      }

      path->poses[i].pose.position.x = target_x;
      path->poses[i].pose.position.y = target_y;
      iteration_changed = true;
      ++total_adjusted_points;
    }

    if(!iteration_changed){
      break;
    }
  }

  path->poses.front().pose.position.x = reference_x.front();
  path->poses.front().pose.position.y = reference_y.front();
  path->poses.back().pose.position.x = reference_x.back();
  path->poses.back().pose.position.y = reference_y.back();

  refreshPathOrientations(path);

  RCLCPP_DEBUG(
    this->get_logger(),
    "graph_path_smoothing applied, poses=%zu, adjusted_points=%zu, iterations=%d",
    path->poses.size(),
    total_adjusted_points,
    graph_path_smoothing_iterations_);
}

unsigned int GlobalPlanner::getClosestIndexFromRadiusSearch(
  const std::vector<int>& point_indices,
  const std::vector<float>& point_squared_distances) const
{
  if(point_indices.empty() || point_squared_distances.empty()){
    return 0;
  }

  std::size_t closest_idx = 0;
  for(std::size_t i = 1; i < point_indices.size() && i < point_squared_distances.size(); ++i){
    if(point_squared_distances[i] < point_squared_distances[closest_idx]){
      closest_idx = i;
    }
  }
  return static_cast<unsigned int>(point_indices[closest_idx]);
}

bool GlobalPlanner::getStartGoalID(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  unsigned int& start_id,
  unsigned int& goal_id,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection){

  const double search_radius = 1.0;
  const double fallback_vertical_radius = 0.5;
  const double fallback_max_nearest_distance = 3.0;
  const auto init_projection =
    [](ForwardHybridAStar::ProjectionDiagnostics * projection,
       const geometry_msgs::msg::PoseStamped & pose) {
      if(projection == nullptr){
        return;
      }
      *projection = ForwardHybridAStar::ProjectionDiagnostics();
      projection->query_x = pose.pose.position.x;
      projection->query_y = pose.pose.position.y;
      projection->query_z = pose.pose.position.z;
      projection->fallback = "none";
    };
  const auto finalize_projection =
    [this](
      ForwardHybridAStar::ProjectionDiagnostics * projection,
      unsigned int ground_id,
      const std::vector<float> & squared_distances)
    {
      if(projection == nullptr){
        return;
      }
      projection->success = true;
      projection->ground_index = ground_id;
      projection->ground_x = pcl_ground_->points[ground_id].x;
      projection->ground_y = pcl_ground_->points[ground_id].y;
      projection->ground_z = pcl_ground_->points[ground_id].z;
      projection->projection_distance = std::sqrt(
        std::pow(projection->ground_x - projection->query_x, 2.0) +
        std::pow(projection->ground_y - projection->query_y, 2.0) +
        std::pow(projection->ground_z - projection->query_z, 2.0));
      if(!squared_distances.empty()){
        const std::size_t nearest_idx =
          static_cast<std::size_t>(std::min_element(
            squared_distances.begin(),
            squared_distances.end()) - squared_distances.begin());
        projection->fallback_nearest_distance =
          std::sqrt(static_cast<double>(squared_distances[nearest_idx]));
      }
    };

  init_projection(start_projection, start);
  init_projection(goal_projection, goal);

  //@Get goal ID
  std::vector<int> pointIdxRadiusSearch_goal;
  std::vector<float> pointRadiusSquaredDistance_goal;
  pcl::PointXYZI pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;
  std::string goal_fallback = "none";
  double goal_fallback_z_before = goal.pose.position.z;
  double goal_fallback_z_after = goal.pose.position.z;
  double goal_fallback_nearest_distance = std::numeric_limits<double>::quiet_NaN();

  //@Compute nearest pc as goal
  //@TODO: add an edge between goal and nearest pc
  
  if(kdtree_ground_->radiusSearch (pcl_goal, search_radius, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal)<1){
    RCLCPP_WARN(this->get_logger(), "Goal is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    for(double z=goal.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_goal.clear();
      pointRadiusSquaredDistance_goal.clear();
      pcl_goal.z = z;
      if(kdtree_ground_->radiusSearch(pcl_goal, fallback_vertical_radius, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal,0)>0)
      {
        fallback_search = true;
        goal_fallback = "vertical_search";
        goal_fallback_z_after = z;
        break;
      }
    }

    if(!fallback_search){
      pointIdxRadiusSearch_goal.clear();
      pointRadiusSquaredDistance_goal.clear();
      if(kdtree_ground_->nearestKSearch(pcl_goal, 1, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal) > 0){
        double nearest_distance = sqrt(pointRadiusSquaredDistance_goal[0]);
        if(nearest_distance <= fallback_max_nearest_distance){
          fallback_search = true;
          goal_fallback = "nearest_fallback";
          goal_fallback_nearest_distance = nearest_distance;
          RCLCPP_WARN(this->get_logger(), "Falling back to nearest goal point on the ground at %.2f m.", nearest_distance);
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Nearest goal point on the ground is too far away: %.2f m.", nearest_distance);
        }
      }
    }

    if(!fallback_search){
      return false;
    }
  }
  
  goal_id = getClosestIndexFromRadiusSearch(pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal);
  if(goal_projection != nullptr){
    goal_projection->fallback = goal_fallback;
    goal_projection->fallback_z_before = goal_fallback_z_before;
    goal_projection->fallback_z_after =
      goal_fallback == "nearest_fallback" ? pcl_ground_->points[goal_id].z : goal_fallback_z_after;
    goal_projection->fallback_nearest_distance = goal_fallback_nearest_distance;
  }
  finalize_projection(goal_projection, goal_id, pointRadiusSquaredDistance_goal);

  if(enable_detail_log_){
    RCLCPP_DEBUG(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }
  else{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 5000, "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }

  //--------------------------------------------------------------------------------------
  //@Get start ID
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZI pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;
  std::string start_fallback = "none";
  double start_fallback_z_before = start.pose.position.z;
  double start_fallback_z_after = start.pose.position.z;
  double start_fallback_nearest_distance = std::numeric_limits<double>::quiet_NaN();

  if(kdtree_ground_->radiusSearch (pcl_start, search_radius, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start)<1){
    RCLCPP_WARN(this->get_logger(), "Start is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    for(double z=start.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_start.clear();
      pointRadiusSquaredDistance_start.clear();
      pcl_start.z = z;
      if(kdtree_ground_->radiusSearch(pcl_start, fallback_vertical_radius, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start,0)>0)
      {
        fallback_search = true;
        start_fallback = "vertical_search";
        start_fallback_z_after = z;
        break;
      }
    }

    if(!fallback_search){
      pointIdxRadiusSearch_start.clear();
      pointRadiusSquaredDistance_start.clear();
      if(kdtree_ground_->nearestKSearch(pcl_start, 1, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start) > 0){
        double nearest_distance = sqrt(pointRadiusSquaredDistance_start[0]);
        if(nearest_distance <= fallback_max_nearest_distance){
          fallback_search = true;
          start_fallback = "nearest_fallback";
          start_fallback_nearest_distance = nearest_distance;
          RCLCPP_WARN(this->get_logger(), "Falling back to nearest start point on the ground at %.2f m.", nearest_distance);
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Nearest start point on the ground is too far away: %.2f m.", nearest_distance);
        }
      }
    }

    if(!fallback_search){
      return false;
    }
  }
  
  start_id = getClosestIndexFromRadiusSearch(pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start);
  if(start_projection != nullptr){
    start_projection->fallback = start_fallback;
    start_projection->fallback_z_before = start_fallback_z_before;
    start_projection->fallback_z_after =
      start_fallback == "nearest_fallback" ? pcl_ground_->points[start_id].z : start_fallback_z_after;
    start_projection->fallback_nearest_distance = start_fallback_nearest_distance;
  }
  finalize_projection(start_projection, start_id, pointRadiusSquaredDistance_start);

  if(enable_detail_log_){
    RCLCPP_DEBUG(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);
  }
  else{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 5000, "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);

  }

  return true;

}

void GlobalPlanner::publishRawRouteDebugPath(
  const nav_msgs::msg::Path & path,
  std::size_t goal_seq,
  std::size_t route_version,
  const std::string & source_label)
{
  nav_msgs::msg::Path debug_path = path;
  debug_path.header.frame_id = global_frame_;
  debug_path.header.stamp = clock_->now();
  for(auto & pose : debug_path.poses){
    pose.header = debug_path.header;
  }
  pub_raw_route_path_->publish(debug_path);
  RCLCPP_DEBUG(
    this->get_logger(),
    "raw_route_path published, route_version=%zu, goal_seq=%zu, source=%s, poses=%zu",
    route_version,
    goal_seq,
    source_label.c_str(),
    debug_path.poses.size());
}

void GlobalPlanner::publishFrozenRouteDebugPath(
  const nav_msgs::msg::Path & path,
  std::size_t goal_seq,
  std::size_t route_version,
  const std::string & source_label)
{
  nav_msgs::msg::Path debug_path = path;
  debug_path.header.frame_id = global_frame_;
  debug_path.header.stamp = clock_->now();
  for(auto & pose : debug_path.poses){
    pose.header = debug_path.header;
  }
  pub_frozen_route_path_->publish(debug_path);
  RCLCPP_DEBUG(
    this->get_logger(),
    "frozen_route_path published, route_version=%zu, goal_seq=%zu, source=%s, poses=%zu",
    route_version,
    goal_seq,
    source_label.c_str(),
    debug_path.poses.size());
}

void GlobalPlanner::publishRouteRequestStageDebug(
  std::size_t request_id,
  std::size_t goal_seq,
  const std::string & stage,
  double stage_elapsed_sec,
  std::size_t expansions,
  const std::string & result_class)
{
  if(!pub_route_request_stage_){
    return;
  }

  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "request_id=" << request_id
         << ";goal_seq=" << goal_seq
         << ";stage=" << stage
         << ";stage_elapsed_sec=" << stage_elapsed_sec
         << ";expansions=" << expansions
         << ";result_class=" << result_class;
  message.data = stream.str();
  pub_route_request_stage_->publish(message);
}

bool GlobalPlanner::buildFrozenRouteLocked(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path * raw_route,
  nav_msgs::msg::Path * frozen_route,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  std::size_t request_id,
  std::size_t goal_seq,
  std::string * request_result_class)
{
  if(raw_route == nullptr || frozen_route == nullptr){
    return false;
  }

  if(was_canceled != nullptr){
    *was_canceled = false;
  }
  if(request_result_class != nullptr){
    *request_result_class = "failed_raw_route";
  }

  const auto elapsed_seconds =
    [](const std::chrono::steady_clock::time_point & started_at) {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
    };
  const auto publish_stage =
    [this, request_id, goal_seq, &elapsed_seconds](
      const std::string & stage,
      const std::chrono::steady_clock::time_point & stage_started_at,
      std::size_t expansions,
      const std::string & result_class)
    {
      if(request_id == 0){
        return;
      }
      publishRouteRequestStageDebug(
        request_id,
        goal_seq,
        stage,
        elapsed_seconds(stage_started_at),
        expansions,
        result_class);
    };

  const auto projecting_started_at = std::chrono::steady_clock::now();
  publish_stage("projecting", projecting_started_at, 0, "in_progress");

  ForwardHybridAStar::SearchDiagnostics raw_route_search_diagnostics;
  ForwardHybridAStar::ProjectionDiagnostics start_projection;
  ForwardHybridAStar::ProjectionDiagnostics goal_projection;
  bool raw_route_stage_started = false;
  auto raw_route_stage_started_at = std::chrono::steady_clock::now();
  const auto raw_route_progress_callback =
    [&publish_stage, &raw_route_stage_started, &raw_route_stage_started_at](
      const ForwardHybridAStar::SearchDiagnostics & diagnostics)
    {
      if(!raw_route_stage_started){
        raw_route_stage_started = true;
        raw_route_stage_started_at = std::chrono::steady_clock::now();
      }
      publish_stage(
        "raw_route",
        raw_route_stage_started_at,
        diagnostics.expansions,
        "in_progress");
    };
  std::ostringstream raw_route_debug_label;
  raw_route_debug_label
    << "raw_route goal_seq=" << goal_seq
    << ", request_id=" << request_id
    << ", stage=raw_route";

  *raw_route = makeROSPlanLocked(
    start,
    goal,
    false,
    false,
    0,
    cancel_requested,
    was_canceled,
    raw_route_debug_label.str(),
    &raw_route_search_diagnostics,
    &start_projection,
    &goal_projection,
    raw_route_progress_callback);
  if(!raw_route_stage_started && start_projection.success && goal_projection.success){
    raw_route_stage_started = true;
    raw_route_stage_started_at = std::chrono::steady_clock::now();
    publish_stage(
      "raw_route",
      raw_route_stage_started_at,
      raw_route_search_diagnostics.expansions,
      "in_progress");
  }

  if(was_canceled != nullptr && *was_canceled){
    if(request_result_class != nullptr){
      *request_result_class = "planner_canceled";
    }
    frozen_route->poses.clear();
    return false;
  }

  const bool projection_failed = !start_projection.success || !goal_projection.success;
  if(raw_route->poses.empty()){
    const std::string raw_route_result_class =
      projection_failed ? "failed_projection" : "failed_raw_route";
    if(request_result_class != nullptr){
      *request_result_class = raw_route_result_class;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "raw route stage finished, goal_seq=%zu, request_id=%zu, stage=raw_route, backend=%s, success=0, result_class=%s, expansions=%zu, planning_time=%.3f, path_poses=%zu, start_ground_idx=%zu, start_ground_z=%.2f, start_projection_distance=%.3f, start_fallback=%s, goal_ground_idx=%zu, goal_ground_z=%.2f, goal_projection_distance=%.3f, goal_fallback=%s",
      goal_seq,
      request_id,
      raw_route_backend_.c_str(),
      raw_route_result_class.c_str(),
      raw_route_search_diagnostics.expansions,
      raw_route_search_diagnostics.planning_time_sec,
      raw_route_search_diagnostics.path_pose_count,
      start_projection.ground_index,
      start_projection.ground_z,
      start_projection.projection_distance,
      start_projection.fallback.c_str(),
      goal_projection.ground_index,
      goal_projection.ground_z,
      goal_projection.projection_distance,
      goal_projection.fallback.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "raw route stage finished, goal_seq=%zu, request_id=%zu, stage=raw_route, backend=%s, success=1, result_class=succeeded, expansions=%zu, planning_time=%.3f, path_poses=%zu, start_ground_idx=%zu, start_ground_z=%.2f, start_projection_distance=%.3f, start_fallback=%s, goal_ground_idx=%zu, goal_ground_z=%.2f, goal_projection_distance=%.3f, goal_fallback=%s",
      goal_seq,
      request_id,
      raw_route_backend_.c_str(),
      raw_route_search_diagnostics.expansions,
      raw_route_search_diagnostics.planning_time_sec,
      raw_route_search_diagnostics.path_pose_count,
      start_projection.ground_index,
      start_projection.ground_z,
      start_projection.projection_distance,
      start_projection.fallback.c_str(),
      goal_projection.ground_index,
      goal_projection.ground_z,
      goal_projection.projection_distance,
      goal_projection.fallback.c_str());
  }

  *frozen_route = *raw_route;
  if(raw_route->poses.empty()){
    return false;
  }

  if(cancel_requested && cancel_requested()){
    if(was_canceled != nullptr){
      *was_canceled = true;
    }
    if(request_result_class != nullptr){
      *request_result_class = "planner_canceled";
    }
    frozen_route->poses.clear();
    return false;
  }

  const auto composing_started_at = std::chrono::steady_clock::now();
  publish_stage("composing_frozen_route", composing_started_at, 0, "in_progress");
  if(request_result_class != nullptr){
    *request_result_class = "succeeded";
  }
  return true;
}

void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  //@get goal and start
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  const auto cancel_requested =
    [goal_handle]() {
      return goal_handle->is_canceling();
    };

  const auto finish_canceled =
    [this, &goal_handle, &result](const std::string & reason) {
      RCLCPP_WARN(this->get_logger(), "%s", reason.c_str());
      goal_handle->canceled(result);
      clearCurrentHandleIfMatches(goal_handle);
    };

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 1000, "Received the request before static layer is ready");
    goal_handle->abort(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 1000, "Deactivate thread");
    goal_handle->succeed(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  const std::size_t goal_seq = ++debug_goal_seq_;
  const std::size_t request_id = ++debug_request_id_;
  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);

  nav_msgs::msg::Path raw_route;
  nav_msgs::msg::Path frozen_route;
  bool planner_canceled = false;
  std::string request_result_class = "failed_raw_route";

  if(cancel_requested()){
    finish_canceled("planner canceled before route computation started");
    return;
  }
  {
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    if(!buildFrozenRouteLocked(
         start,
         goal->goal,
         &raw_route,
         &frozen_route,
         cancel_requested,
         &planner_canceled,
         request_id,
         goal_seq,
         &request_result_class))
    {
      frozen_route = raw_route;
    }
  }

  if(planner_canceled || cancel_requested()){
    publishRouteRequestStageDebug(
      request_id,
      goal_seq,
      "finished",
      0.0,
      0,
      "planner_canceled");
    finish_canceled(
      "planner canceled while computing route; client-side wait ended before planner finished");
    return;
  }

  if(raw_route.poses.empty()){
    publishRouteRequestStageDebug(
      request_id,
      goal_seq,
      "finished",
      0.0,
      0,
      request_result_class);
    publishRawRouteDebugPath(raw_route, goal_seq, debug_route_version_, "get_plan_failed");
    publishFrozenRouteDebugPath(frozen_route, goal_seq, debug_route_version_, "get_plan_failed");
    result->path = frozen_route;
    goal_handle->abort(result);
  }
  else{
    const std::size_t route_version = ++debug_route_version_;
    publishRawRouteDebugPath(raw_route, goal_seq, route_version, "get_plan_raw_route");
    RCLCPP_DEBUG(
      this->get_logger(),
      "raw route generated, goal_seq=%zu, route_version=%zu, poses=%zu",
      goal_seq,
      route_version,
      raw_route.poses.size());
    publishFrozenRouteDebugPath(
      frozen_route,
      goal_seq,
      route_version,
      "get_plan_frozen_route");
    RCLCPP_DEBUG(
      this->get_logger(),
      "frozen route composed, goal_seq=%zu, route_version=%zu, frozen_route_poses=%zu",
      goal_seq,
      route_version,
      frozen_route.poses.size());
    pub_path_->publish(frozen_route);
    RCLCPP_DEBUG(
      this->get_logger(),
      "frozen route published for goal_seq=%zu, route_version=%zu",
      goal_seq,
      route_version);
    publishRouteRequestStageDebug(
      request_id,
      goal_seq,
      "finished",
      0.0,
      0,
      request_result_class);
    result->path = frozen_route;
    goal_handle->succeed(result);
  }
  clearCurrentHandleIfMatches(goal_handle);
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  bool force_position_only_goal,
  bool force_use_goal_heading,
  int preferred_initial_turn_sign,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label,
  ForwardHybridAStar::SearchDiagnostics * search_diagnostics,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection,
  const ForwardHybridAStar::ProgressCallback & progress_callback)
{
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  return makeROSPlanLocked(
    start,
    goal,
    force_position_only_goal,
    force_use_goal_heading,
    preferred_initial_turn_sign,
    cancel_requested,
    was_canceled,
    debug_label,
    search_diagnostics,
    start_projection,
    goal_projection,
    progress_callback);
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlanLocked(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  bool force_position_only_goal,
  bool force_use_goal_heading,
  int preferred_initial_turn_sign,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label,
  ForwardHybridAStar::SearchDiagnostics * search_diagnostics,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection,
  const ForwardHybridAStar::ProgressCallback & progress_callback){

  unsigned int start_id = 0;
  unsigned int goal_id = 0;
  bool has_start_goal_id = false;
  std::vector<unsigned int> path;
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();

  if(was_canceled != nullptr){
    *was_canceled = false;
  }

  const auto mark_canceled =
    [&ros_path, was_canceled]() -> nav_msgs::msg::Path {
      if(was_canceled != nullptr){
        *was_canceled = true;
      }
      ros_path.poses.clear();
      return ros_path;
    };

  if(cancel_requested && cancel_requested()){
    return mark_canceled();
  }

  const bool raw_route_uses_forward_hybrid_astar =
    raw_route_backend_ == "hybrid_a_star";
  const auto elapsed_seconds =
    [](const std::chrono::steady_clock::time_point & started_at) {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
    };

  if(raw_route_uses_forward_hybrid_astar && forward_hybrid_astar_planner_){
    nav_msgs::msg::Path hybrid_path;
    bool hybrid_planning_canceled = false;
    if(forward_hybrid_astar_planner_->MakePlan(
        start, goal, &hybrid_path,
        force_position_only_goal,
        force_use_goal_heading,
        preferred_initial_turn_sign,
        cancel_requested,
        &hybrid_planning_canceled,
        debug_label,
        search_diagnostics,
        start_projection,
        goal_projection,
        progress_callback))
    {
      hybrid_path.header.frame_id = global_frame_;
      hybrid_path.header.stamp = clock_->now();
      return hybrid_path;
    }
    if(hybrid_planning_canceled){
      return mark_canceled();
    }
    if(cancel_requested && cancel_requested()){
      return mark_canceled();
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "hybrid_a_star raw_route failed; fallback to graph_a_star is disabled.");
    ros_path.poses.clear();
    return ros_path;
  }
  else if(raw_route_uses_forward_hybrid_astar){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "raw_route_backend=hybrid_a_star requested but backend is unavailable; fallback to graph_a_star is disabled.");
    if(search_diagnostics != nullptr){
      search_diagnostics->success = false;
      search_diagnostics->canceled = false;
      search_diagnostics->expansions = 0;
      search_diagnostics->planning_time_sec = 0.0;
      search_diagnostics->path_pose_count = 0;
    }
    ros_path.poses.clear();
    return ros_path;
  }

  if(cancel_requested && cancel_requested()){
    return mark_canceled();
  }

  const auto graph_planning_started_at = std::chrono::steady_clock::now();
  has_start_goal_id =
    getStartGoalID(start, goal, start_id, goal_id, start_projection, goal_projection);
  if(has_start_goal_id && progress_callback){
    ForwardHybridAStar::SearchDiagnostics progress;
    progress.success = false;
    progress.canceled = false;
    progress.expansions = 0;
    progress.planning_time_sec = 0.0;
    progress.path_pose_count = 0;
    progress_callback(progress);
  }
  if(cancel_requested && cancel_requested()){
    if(search_diagnostics != nullptr){
      search_diagnostics->success = false;
      search_diagnostics->canceled = true;
      search_diagnostics->expansions = 0;
      search_diagnostics->planning_time_sec = elapsed_seconds(graph_planning_started_at);
      search_diagnostics->path_pose_count = 0;
    }
    return mark_canceled();
  }
  if(has_start_goal_id){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);  
  }
  if(search_diagnostics != nullptr){
    search_diagnostics->success = !path.empty();
    search_diagnostics->canceled = false;
    search_diagnostics->expansions = 0;
    search_diagnostics->planning_time_sec = elapsed_seconds(graph_planning_started_at);
    search_diagnostics->path_pose_count = 0;
  }

  if(path.empty()){
    if(enable_detail_log_ && has_start_goal_id)
      RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
    else if(enable_detail_log_)
      RCLCPP_WARN(this->get_logger(), "No path found because start/goal projection failed.");
    else if(has_start_goal_id)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found because start/goal projection failed.");
    return ros_path;
  }
  else{
    if(enable_detail_log_)
      RCLCPP_DEBUG(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 5000, "Path found from: %u to %u", start_id, goal_id);
    getROSPath(path, ros_path);
    if(search_diagnostics != nullptr){
      search_diagnostics->path_pose_count = ros_path.poses.size();
    }

    // Legacy graph A* can return a first segment that starts from projected
    // ground nodes slightly behind the current robot pose. Trim this near-start
    // rearward prefix and anchor the path with the current start pose so the
    // local planner does not receive an immediate backward hook.
    double start_yaw = 0.0;
    bool has_start_yaw = false;
    {
      const auto & q_msg = start.pose.orientation;
      const double norm =
        q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
      if (std::isfinite(norm) && norm > 1e-9) {
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        q.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, start_yaw);
        has_start_yaw = std::isfinite(start_yaw);
      }
    }

    if (has_start_yaw && !ros_path.poses.empty()) {
      constexpr double kRearTrimDistance = 1.5;
      constexpr double kRearTrimAllowance = 0.05;
      std::size_t trim_count = 0;
      for (std::size_t i = 0; i < ros_path.poses.size(); ++i) {
        const double dx = ros_path.poses[i].pose.position.x - start.pose.position.x;
        const double dy = ros_path.poses[i].pose.position.y - start.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance) || distance > kRearTrimDistance) {
          break;
        }
        const double x_local = std::cos(start_yaw) * dx + std::sin(start_yaw) * dy;
        if (x_local < -kRearTrimAllowance) {
          trim_count = i + 1;
          continue;
        }
        break;
      }

      if (trim_count > 0 && trim_count < ros_path.poses.size()) {
        ros_path.poses.erase(ros_path.poses.begin(), ros_path.poses.begin() + trim_count);
      } else if (trim_count >= ros_path.poses.size()) {
        ros_path.poses.clear();
      }
    }

    if (ros_path.poses.empty()) {
      geometry_msgs::msg::PoseStamped start_pose = start;
      start_pose.header = ros_path.header;
      ros_path.poses.push_back(start_pose);
    } else {
      const double first_dx = ros_path.poses.front().pose.position.x - start.pose.position.x;
      const double first_dy = ros_path.poses.front().pose.position.y - start.pose.position.y;
      if (std::hypot(first_dx, first_dy) > 0.05) {
        geometry_msgs::msg::PoseStamped start_pose = start;
        start_pose.header = ros_path.header;
        ros_path.poses.insert(ros_path.poses.begin(), start_pose);
      }
    }

    ros_path.poses.push_back(goal);
    smoothGraphPathForAckermann(&ros_path);
    if(search_diagnostics != nullptr){
      search_diagnostics->planning_time_sec = elapsed_seconds(graph_planning_started_at);
      search_diagnostics->path_pose_count = ros_path.poses.size();
    }
    return ros_path;
  }
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight
  
  /*
  //static graph has been remove 9 Mar 2025
  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();
  pubStaticGraph();
  
  RCLCPP_DEBUG(this->get_logger(), "Static graph is generated with size: %lu", static_graph_.getSize());
  */

  const bool raw_route_uses_graph_astar = raw_route_backend_ == "graph_a_star";

  if(!has_initialized_){
    has_initialized_ = true;
    if(raw_route_uses_graph_astar){
      if(a_star_expanding_radius_ >= perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2){
        RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
      }
      if(!use_pre_graph_){
        a_star_planner_ = std::make_shared<A_Star_on_Graph>(pcl_ground_, perception_3d_ros_, a_star_expanding_radius_);
        a_star_planner_->setupTurningWeight(turning_weight_);
      }
      else{
        a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(pcl_ground_, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
        a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
      }
    }

    forward_hybrid_astar_planner_ = std::make_shared<ForwardHybridAStar>(
      perception_3d_ros_, pcl_ground_, kdtree_ground_, this->get_logger());
    forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
    forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
  }
  else{
    if(raw_route_uses_graph_astar){
      if(!use_pre_graph_ && a_star_planner_){
        a_star_planner_->updateGraph(pcl_ground_);
      }
      else if(use_pre_graph_ && a_star_planner_pre_graph_){
        a_star_planner_pre_graph_->updateGraph(pcl_ground_, static_graph_);
      }
    }
    if(forward_hybrid_astar_planner_){
      forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
      forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
    }
  }

  pubWeight();

  RCLCPP_INFO(this->get_logger(), "Publish weighted ground point cloud.");
  graph_ready_ = true;
}


void GlobalPlanner::pubWeight(){

  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);
  
  unsigned long node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeightSize();
  for(auto it=0; it<node_weight_size; it++){

    pcl::PointXYZI ipt;

    ipt.x = pcl_ground_->points[it].x;
    ipt.y = pcl_ground_->points[it].y;
    ipt.z = pcl_ground_->points[it].z;
    ipt.intensity = static_graph_.getNodeWeight(it);    
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void GlobalPlanner::pubStaticGraph(){
 
  //@edge visualization 
  //@This is just for visulization, therefore reduce edges to let rviz less lag
  
  std::set<std::pair<unsigned int, unsigned int>> duplicate_check;
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = global_frame_;
  markerEdge.header.stamp = clock_->now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "edges";
  markerEdge.id = 3;
  markerEdge.scale.x = 0.03;
  markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
  markerEdge.color.a = 0.2;

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();

  int cnt = 0;
  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){
    geometry_msgs::msg::Point p;
    p.x = pcl_ground_->points[(*it).first].x;
    p.y = pcl_ground_->points[(*it).first].y;
    p.z = pcl_ground_->points[(*it).first].z;
    for(auto it_set = (*it).second.begin();it_set != (*it).second.end();it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = (*it).first;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = (*it).first;
      if( !duplicate_check.insert(edge_marker).second )
      {   
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {   
        continue;
      }
      markerEdge.points.push_back(p);
      p.x = pcl_ground_->points[(*it_set).first].x;
      p.y = pcl_ground_->points[(*it_set).first].y;
      p.z = pcl_ground_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

}
