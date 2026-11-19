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
namespace
{

constexpr double kHybridRefineSegmentLength = 8.0;
constexpr double kHybridRefineMinSegmentLength = 1.0;
constexpr double kPathDuplicateDistance = 0.05;

double ElapsedMs(const std::chrono::steady_clock::time_point & started_at)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started_at).count();
}

// Diagnostic-only accumulation for one request's Hybrid-refine segments.
// Per-thread because the route build is serialized and MakePlan() is called on
// this thread; no new lock is introduced on the planning hot path.
struct HybridRefineTiming
{
  std::size_t segments = 0;
  std::size_t segments_succeeded = 0;
  std::size_t dgraph_calls = 0;
  double dgraph_query_ms = 0.0;
  double dgraph_max_ms = 0.0;
  std::size_t dgraph_over_1ms = 0;
  std::size_t expansions = 0;
  double segment_search_ms = 0.0;
  double projection_ms = 0.0;
  std::size_t batch_lock_calls = 0;
  double batch_lock_wait_ms = 0.0;
  double batch_lock_max_ms = 0.0;

  void reset()
  {
    segments = 0;
    segments_succeeded = 0;
    dgraph_calls = 0;
    dgraph_query_ms = 0.0;
    dgraph_max_ms = 0.0;
    dgraph_over_1ms = 0;
    expansions = 0;
    segment_search_ms = 0.0;
    projection_ms = 0.0;
    batch_lock_calls = 0;
    batch_lock_wait_ms = 0.0;
    batch_lock_max_ms = 0.0;
  }

  void add(const ForwardHybridAStar::SearchDiagnostics & diagnostics, double segment_ms, bool succeeded)
  {
    ++segments;
    if (succeeded) {
      ++segments_succeeded;
    }
    dgraph_calls += diagnostics.dgraph_calls;
    dgraph_query_ms += diagnostics.dgraph_query_ms;
    if (diagnostics.dgraph_max_ms > dgraph_max_ms) {
      dgraph_max_ms = diagnostics.dgraph_max_ms;
    }
    dgraph_over_1ms += diagnostics.dgraph_over_1ms;
    expansions += diagnostics.expansions;
    segment_search_ms += segment_ms;
    projection_ms += diagnostics.projection_ms;
    batch_lock_calls += diagnostics.batch_lock_calls;
    batch_lock_wait_ms += diagnostics.batch_lock_wait_ms;
    if (diagnostics.batch_lock_max_ms > batch_lock_max_ms) {
      batch_lock_max_ms = diagnostics.batch_lock_max_ms;
    }
  }
};

thread_local HybridRefineTiming g_hybrid_refine_timing;

double Distance2D(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  return std::hypot(dx, dy);
}

std::size_t IndexAtDistanceFromIndex(
  const nav_msgs::msg::Path & path,
  std::size_t start_index,
  double distance)
{
  if(path.poses.empty()){
    return 0;
  }

  const std::size_t last_index = path.poses.size() - 1;
  if(start_index >= last_index){
    return last_index;
  }

  double accumulated = 0.0;
  for(std::size_t i = start_index + 1; i < path.poses.size(); ++i){
    accumulated += Distance2D(path.poses[i - 1], path.poses[i]);
    if(accumulated >= distance){
      return i;
    }
  }
  return last_index;
}

void AppendPath(
  const nav_msgs::msg::Path & source,
  nav_msgs::msg::Path * destination,
  std::size_t begin,
  std::size_t end)
{
  if(destination == nullptr || source.poses.empty() || begin >= source.poses.size()){
    return;
  }
  end = std::min(end, source.poses.size() - 1);
  if(begin > end){
    return;
  }

  for(std::size_t i = begin; i <= end; ++i){
    if(!destination->poses.empty() &&
      Distance2D(destination->poses.back(), source.poses[i]) < kPathDuplicateDistance)
    {
      continue;
    }
    destination->poses.push_back(source.poses[i]);
  }
}

}  // namespace

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
  
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  static_scene_generation_ = 0;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();

  //@ One-time producer mode: original-planner cost only. Evidence export and
  //@ the point-export/local pipeline are left off exactly as before.
  perception_3d::SceneProducerConfig scene_producer_config;
  scene_producer_config.enabled = true;
  scene_producer_config.evidence = false;
  scene_producer_config.legacy_cost = true;
  scene_producer_config.static_source_name = "map";
  perception_3d_ros_->configureSceneProducer(scene_producer_config);

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

  declare_parameter("hybrid_astar.wheelbase", rclcpp::ParameterValue(0.550));
  this->get_parameter("hybrid_astar.wheelbase", forward_hybrid_astar_config_.wheelbase);
  declare_parameter("hybrid_astar.max_steer", rclcpp::ParameterValue(0.3270520925102273));
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
    "hybrid_astar: goal_tol=(%.2f, %.2f) proj_radius=%.2f",
    forward_hybrid_astar_config_.goal_position_tolerance,
    forward_hybrid_astar_config_.goal_heading_tolerance,
    forward_hybrid_astar_config_.projection_search_radius);
  RCLCPP_DEBUG(
    this->get_logger(),
    "hybrid_astar: max_projected_pitch=%.2f max_projected_vertical_jump=%.2f allow_sample_nearest_fallback=%d edge_weight_penalty_weight=%.2f edge_weight_safe_threshold=%.2f edge_weight_soft_cap=%.2f edge_weight_hard_reject_threshold=%.2f",
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
}

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_){
    return;
  }

  //@ Static-only capture. This never evaluates the per-ground fused dGraph or
  //@ freezes lethal points, so the 500 ms idle timer stays cheap. A full cost
  //@ export happens only on an actual route request.
  perception_3d::SceneSnapshotRequest request;
  request.include_evidence = false;
  request.include_legacy_cost = false;
  request.include_legacy_static = true;
  const auto static_snapshot = perception_3d_ros_->captureSceneSnapshot(
    request, this->now().nanoseconds(), 0, 0, false);

  if(!static_snapshot || !static_snapshot->complete() ||
    !static_snapshot->hasLegacyStatic() || !static_snapshot->sourceMap().known)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Waiting for one complete immutable static scene");
    return;
  }

  const perception_3d::SourceMapIdentity & new_identity =
    static_snapshot->sourceMap();
  const std::shared_ptr<const perception_3d::LegacyStaticView> & new_view =
    static_snapshot->legacyStatic();

  {
    std::lock_guard<std::mutex> cache_lock(scene_cache_mutex_);
    if(current_static_view_ == new_view &&
      current_static_identity_.SameContent(new_identity))
    {
      return;
    }
  }

  //@ Static changes are serialized with route builds through the planner's own
  //@ workspace lock. No perception lock is taken directly here.
  std::unique_lock<std::mutex> workspace_lock(protect_kdtree_ground_);
  std::uint64_t previous_revision = 0;
  {
    std::lock_guard<std::mutex> cache_lock(scene_cache_mutex_);
    previous_revision = current_static_identity_.revision;
    current_static_view_ = new_view;
    current_static_identity_ = new_identity;
    ++static_scene_generation_;
  }

  //@ Drop every backend's pinned route view. The next request re-pins the new
  //@ static identity before projection/search, so no old index binding is
  //@ retained as a cached route.
  if(a_star_planner_){
    a_star_planner_->clearSceneSnapshot();
  }
  if(a_star_planner_pre_graph_){
    a_star_planner_pre_graph_->clearSceneSnapshot();
  }
  if(forward_hybrid_astar_planner_){
    forward_hybrid_astar_planner_->ClearSceneSnapshot();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Pinned static scene changed: revision %llu -> %llu, content_hash=%llu, ground=%zu, map=%zu, generation=%llu; cached route views invalidated",
    static_cast<unsigned long long>(previous_revision),
    static_cast<unsigned long long>(new_identity.revision),
    static_cast<unsigned long long>(new_identity.content_hash),
    new_identity.ground_point_count,
    new_identity.map_point_count,
    static_cast<unsigned long long>(static_scene_generation_));

  getStaticGraphFromPerception3D();
}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
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

  ScenePipelineContext scene;
  std::string scene_reason;
  if(!acquirePinnedScene(&scene, &scene_reason)){
    RCLCPP_WARN(
      this->get_logger(),
      "Clicked goal route request aborted without a complete pinned scene: %s",
      scene_reason.c_str());
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
         scene,
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

void GlobalPlanner::getROSPath(
  std::vector<unsigned int>& path_id,
  nav_msgs::msg::Path& ros_path,
  const ScenePipelineContext & scene)
{
  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();

  if(!scene.valid() || !scene.staticView() || !scene.staticView()->groundPoints()){
    return;
  }
  const pcl::PointCloud<pcl::PointXYZI> & ground_points =
    *scene.staticView()->groundPoints();

  for(auto it=0;it<path_id.size();it++){
    if(path_id[it] >= ground_points.points.size()){
      continue;
    }
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = ground_points.points[path_id[it]].x;
    pst.pose.position.y = ground_points.points[path_id[it]].y;
    pst.pose.position.z = ground_points.points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1 && path_id[it+1] < ground_points.points.size()){
      next_pst.pose.position.x = ground_points.points[path_id[it+1]].x;
      next_pst.pose.position.y = ground_points.points[path_id[it+1]].y;
      next_pst.pose.position.z = ground_points.points[path_id[it+1]].z;
    }

    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;

    if(vz!=0){
      double unit = sqrt(vx*vx + vy*vy + vz*vz);
      if(unit > 1e-9){
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
  RefreshPathOrientations(path);
}

void GlobalPlanner::smoothGraphPathForAckermann(
  nav_msgs::msg::Path * path,
  const ScenePipelineContext & scene) const
{
  if(path == nullptr){
    return;
  }
  const std::size_t adjusted_points = SmoothPathAgainstScene(
    path,
    scene,
    graph_path_smoothing_enable_,
    graph_path_smoothing_iterations_,
    graph_path_smoothing_weight_,
    graph_path_smoothing_max_shift_,
    graph_path_smoothing_lock_start_distance_,
    graph_path_smoothing_lock_goal_distance_,
    graph_path_smoothing_ground_search_radius_);
  if(adjusted_points > 0){
    RCLCPP_DEBUG(
      this->get_logger(),
      "graph_path_smoothing applied, poses=%zu, adjusted_points=%zu, iterations=%d",
      path->poses.size(),
      adjusted_points,
      graph_path_smoothing_iterations_);
  }
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
  const ScenePipelineContext & scene,
  unsigned int& start_id,
  unsigned int& goal_id,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection){

  const double search_radius = 1.0;
  const double fallback_vertical_radius = 0.5;
  const double fallback_max_nearest_distance = 3.0;

  if(!scene.valid() || !scene.staticView() ||
    !scene.staticView()->groundPoints() || !scene.staticView()->groundKdtree())
  {
    return false;
  }
  const pcl::PointCloud<pcl::PointXYZI> & ground_points =
    *scene.staticView()->groundPoints();
  const auto ground_tree = scene.staticView()->groundKdtree();

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
    [&ground_points](
      ForwardHybridAStar::ProjectionDiagnostics * projection,
      unsigned int ground_id,
      const std::vector<float> & squared_distances)
    {
      if(projection == nullptr || ground_id >= ground_points.points.size()){
        return;
      }
      projection->success = true;
      projection->ground_index = ground_id;
      projection->ground_x = ground_points.points[ground_id].x;
      projection->ground_y = ground_points.points[ground_id].y;
      projection->ground_z = ground_points.points[ground_id].z;
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

  if(!scene.ProjectOnGround(
      pcl_goal, search_radius, &pointIdxRadiusSearch_goal,
      &pointRadiusSquaredDistance_goal))
  {
    RCLCPP_WARN(this->get_logger(), "Goal is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    for(double z=goal.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_goal.clear();
      pointRadiusSquaredDistance_goal.clear();
      pcl_goal.z = z;
      if(scene.ProjectOnGround(
          pcl_goal, fallback_vertical_radius, &pointIdxRadiusSearch_goal,
          &pointRadiusSquaredDistance_goal))
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
      pcl_goal.z = goal.pose.position.z;  // restore query z after vertical scan
      if(ground_tree->nearestKSearch(
          pcl_goal, 1, pointIdxRadiusSearch_goal,
          pointRadiusSquaredDistance_goal) > 0)
      {
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
  if(goal_id >= ground_points.points.size()){
    return false;
  }
  if(goal_projection != nullptr){
    goal_projection->fallback = goal_fallback;
    goal_projection->fallback_z_before = goal_fallback_z_before;
    goal_projection->fallback_z_after =
      goal_fallback == "nearest_fallback" ? ground_points.points[goal_id].z : goal_fallback_z_after;
    goal_projection->fallback_nearest_distance = goal_fallback_nearest_distance;
  }
  finalize_projection(goal_projection, goal_id, pointRadiusSquaredDistance_goal);

  if(enable_detail_log_){
    RCLCPP_DEBUG(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      ground_points.points[goal_id].x, ground_points.points[goal_id].y, ground_points.points[goal_id].z);
  }
  else{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 5000, "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      ground_points.points[goal_id].x, ground_points.points[goal_id].y, ground_points.points[goal_id].z);
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

  if(!scene.ProjectOnGround(
      pcl_start, search_radius, &pointIdxRadiusSearch_start,
      &pointRadiusSquaredDistance_start))
  {
    RCLCPP_WARN(this->get_logger(), "Start is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    for(double z=start.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_start.clear();
      pointRadiusSquaredDistance_start.clear();
      pcl_start.z = z;
      if(scene.ProjectOnGround(
          pcl_start, fallback_vertical_radius, &pointIdxRadiusSearch_start,
          &pointRadiusSquaredDistance_start))
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
      pcl_start.z = start.pose.position.z;  // restore query z after vertical scan
      if(ground_tree->nearestKSearch(
          pcl_start, 1, pointIdxRadiusSearch_start,
          pointRadiusSquaredDistance_start) > 0)
      {
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
  if(start_id >= ground_points.points.size()){
    return false;
  }
  if(start_projection != nullptr){
    start_projection->fallback = start_fallback;
    start_projection->fallback_z_before = start_fallback_z_before;
    start_projection->fallback_z_after =
      start_fallback == "nearest_fallback" ? ground_points.points[start_id].z : start_fallback_z_after;
    start_projection->fallback_nearest_distance = start_fallback_nearest_distance;
  }
  finalize_projection(start_projection, start_id, pointRadiusSquaredDistance_start);

  if(enable_detail_log_){
    RCLCPP_DEBUG(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      ground_points.points[start_id].x, ground_points.points[start_id].y, ground_points.points[start_id].z);
  }
  else{
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 5000, "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      ground_points.points[start_id].x, ground_points.points[start_id].y, ground_points.points[start_id].z);

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

bool GlobalPlanner::acquirePinnedScene(
  ScenePipelineContext * scene,
  std::string * rejection_reason)
{
  if(scene == nullptr){
    if(rejection_reason != nullptr){
      *rejection_reason = "scene context pointer is null";
    }
    return false;
  }
  scene->Reset();
  if(!perception_3d_ros_){
    if(rejection_reason != nullptr){
      *rejection_reason = "perception node is unavailable";
    }
    return false;
  }

  perception_3d::SceneSnapshotRequest request;
  request.include_evidence = false;
  request.include_legacy_cost = true;
  request.include_legacy_static = true;
  const auto snapshot = perception_3d_ros_->captureSceneSnapshot(
    request, this->now().nanoseconds(), 0, 0, false);

  LegacySearchParameters parameters;
  if(auto utils = perception_3d_ros_->getGlobalUtils()){
    parameters.inscribed_radius = utils->getInscribedRadius();
    parameters.inflation_descending_rate = utils->getInflationDescendingRate();
    parameters.max_obstacle_distance = utils->getMaxObstacleDistance();
  }

  std::string reason;
  if(!scene->Bind(snapshot, parameters, &reason)){
    if(rejection_reason != nullptr){
      *rejection_reason = reason;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "scene_not_ready: route request aborted without live fallback: %s",
      reason.c_str());
    return false;
  }

  //@ Root sensor status (not the cached cost-view metadata) drives the warning.
  //@ The original stale policy is retained: costs are not silently cleared.
  if(!snapshot->sensorsCurrent()){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "scene_stale_sensors: root sensor status is stale; retained pinned costs are used under the original stale policy");
  }
  RCLCPP_DEBUG(
    this->get_logger(),
    "scene_pinned: static_revision=%llu content_hash=%llu observation_token=%llu ground=%zu map=%zu sensors_current=%d",
    static_cast<unsigned long long>(snapshot->sourceMap().revision),
    static_cast<unsigned long long>(snapshot->sourceMap().content_hash),
    static_cast<unsigned long long>(snapshot->observationRevision()),
    snapshot->sourceMap().ground_point_count,
    snapshot->sourceMap().map_point_count,
    snapshot->sensorsCurrent() ? 1 : 0);
  return true;
}

bool GlobalPlanner::acquireLatestPinnedScene(
  const ScenePipelineContext & pinned,
  ScenePipelineContext * latest,
  bool * static_map_changed,
  std::string * rejection_reason)
{
  if(latest == nullptr){
    return false;
  }
  latest->Reset();
  if(static_map_changed != nullptr){
    *static_map_changed = false;
  }
  if(!perception_3d_ros_){
    if(rejection_reason != nullptr){
      *rejection_reason = "perception node is unavailable";
    }
    return false;
  }

  perception_3d::SceneSnapshotRequest request;
  request.include_evidence = false;
  request.include_legacy_cost = true;
  request.include_legacy_static = true;
  const auto snapshot = perception_3d_ros_->captureSceneSnapshot(
    request, this->now().nanoseconds(), 0, 0, false);
  if(!snapshot){
    if(rejection_reason != nullptr){
      *rejection_reason = "latest scene capture returned null";
    }
    return false;
  }

  //@ Compare the known static identity even when the envelope is incomplete.
  //@ A changed map invalidates the index binding of the route we just built.
  if(pinned.valid() && pinned.snapshot() && snapshot->sourceMap().known &&
    !snapshot->sourceMap().SameContent(pinned.snapshot()->sourceMap()))
  {
    if(static_map_changed != nullptr){
      *static_map_changed = true;
    }
    if(rejection_reason != nullptr){
      *rejection_reason = "latest static source-map identity changed";
    }
  }

  std::string reason;
  if(!latest->Bind(snapshot, pinned.parameters(), &reason)){
    //@ Preserve the stronger static-map-changed reason when it was already
    //@ detected; otherwise propagate the structural Bind refusal.
    if(rejection_reason != nullptr &&
      (!static_map_changed || !*static_map_changed))
    {
      *rejection_reason = reason;
    }
    return false;
  }
  return true;
}

GlobalPlanner::LatestRouteStatus GlobalPlanner::validateRouteAgainstLatest(
  const ScenePipelineContext & pinned,
  const nav_msgs::msg::Path & route,
  ScenePipelineContext * latest,
  std::string * result_class,
  std::string * detail)
{
  if(result_class != nullptr){
    *result_class = "scene_latest_rejected";
  }
  if(detail != nullptr){
    detail->clear();
  }

  ScenePipelineContext latest_scene;
  bool static_map_changed = false;
  std::string reason;
  if(!acquireLatestPinnedScene(
      pinned, &latest_scene, &static_map_changed, &reason))
  {
    if(detail != nullptr){
      *detail = reason;
    }
    if(static_map_changed){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_static_map_changed: latest capture is not usable for the old route index binding (%s); bounded replan required",
        reason.c_str());
      if(result_class != nullptr){
        *result_class = "scene_static_map_changed";
      }
      if(latest != nullptr){
        *latest = latest_scene;
      }
      return LatestRouteStatus::kReplan;
    }
    //@ A failed capture/Bind is structurally unusable, not the original
    //@ complete-but-stale sensor policy. Fail closed instead of authorizing the
    //@ old route. Stale complete snapshots still Bind successfully and remain
    //@ report-only below.
    RCLCPP_WARN(
      this->get_logger(),
      "scene_latest_rejected: no structurally complete latest scene (%s); refusing to authorize the pinned route",
      reason.c_str());
    return LatestRouteStatus::kRejected;
  }

  const SceneRelation relation = RelatePinnedToLatest(pinned, latest_scene);
  if(!relation.static_map_same){
    RCLCPP_WARN(
      this->get_logger(),
      "scene_static_map_changed: %s; bounded replan required",
      relation.detail.c_str());
    if(result_class != nullptr){
      *result_class = "scene_static_map_changed";
    }
    if(latest != nullptr){
      *latest = latest_scene;
    }
    return LatestRouteStatus::kReplan;
  }

  const double sample_step = std::max(
    0.05, 0.5 * pinned.parameters().inscribed_radius);
  const ScenePathValidation validation = ValidatePathAgainstLatest(
    pinned, latest_scene, route, sample_step);
  if(detail != nullptr){
    *detail = validation.reason;
  }

  switch(validation.result){
    case ScenePathValidation::Result::kAccepted:
      if(validation.observation_same){
        RCLCPP_DEBUG(
          this->get_logger(),
          "scene_latest_ok: %s", validation.reason.c_str());
      }
      else{
        RCLCPP_INFO(
          this->get_logger(),
          "scene_path_unaffected: %s", validation.reason.c_str());
      }
      if(result_class != nullptr){
        *result_class = "succeeded";
      }
      return LatestRouteStatus::kAccepted;
    case ScenePathValidation::Result::kDynamicConflictReplan:
      RCLCPP_WARN(
        this->get_logger(),
        "scene_path_changed: %s; bounded replan required",
        validation.reason.c_str());
      if(result_class != nullptr){
        *result_class = "scene_path_changed";
      }
      if(latest != nullptr){
        *latest = latest_scene;
      }
      return LatestRouteStatus::kReplan;
    case ScenePathValidation::Result::kStaticMismatchReject:
      RCLCPP_WARN(
        this->get_logger(),
        "scene_static_map_changed: %s; bounded replan required",
        validation.reason.c_str());
      if(result_class != nullptr){
        *result_class = "scene_static_map_changed";
      }
      if(latest != nullptr){
        *latest = latest_scene;
      }
      return LatestRouteStatus::kReplan;
    case ScenePathValidation::Result::kNotReady:
    default:
      RCLCPP_WARN(
        this->get_logger(),
        "scene_latest_rejected: latest scene could not validate the route (%s); refusing to authorize the old route",
        validation.reason.c_str());
      if(result_class != nullptr){
        *result_class = "scene_latest_rejected";
      }
      return LatestRouteStatus::kRejected;
  }
}

bool GlobalPlanner::buildFrozenRouteLocked(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const ScenePipelineContext & scene,
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

  const auto total_started_at = std::chrono::steady_clock::now();
  g_hybrid_refine_timing.reset();

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
  const auto raw_route_started_at = std::chrono::steady_clock::now();
  bool raw_route_stage_started = false;
  auto raw_route_stage_started_at = raw_route_started_at;
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

  if(raw_route_backend_ == "hybrid_a_star"){
    *raw_route = makeGraphRouteLocked(
      start,
      goal,
      scene,
      cancel_requested,
      was_canceled,
      &raw_route_search_diagnostics,
      &start_projection,
      &goal_projection);
  }
  else{
    *raw_route = makeROSPlanLocked(
      start,
      goal,
      scene,
      false,
      false,
      cancel_requested,
      was_canceled,
      raw_route_debug_label.str(),
      &raw_route_search_diagnostics,
      &start_projection,
      &goal_projection,
      raw_route_progress_callback);
  }
  if(!raw_route_stage_started && start_projection.success && goal_projection.success){
    raw_route_stage_started = true;
    publish_stage(
      "raw_route",
      raw_route_stage_started_at,
      raw_route_search_diagnostics.expansions,
      "in_progress");
  }

  const double raw_route_ms = ElapsedMs(raw_route_started_at);
  const auto log_request_timing =
    [this, total_started_at, goal_seq, request_id, &elapsed_seconds, &raw_route_search_diagnostics](
      const char * result_class,
      double raw_route_ms_value,
      double hybrid_refine_ms_value)
    {
      const auto & refine = g_hybrid_refine_timing;
      RCLCPP_INFO(
        this->get_logger(),
        "global_planner request timing: goal_seq=%zu, request_id=%zu, result=%s, "
        "raw_route_ms=%.3f, hybrid_refine_ms=%.3f, total_ms=%.3f, "
        "raw_projection_ms=%.3f, raw_aggregate_lethal_ms=%.3f, raw_graph_search_ms=%.3f, "
        "raw_dgraph_calls=%zu, raw_dgraph_query_ms=%.3f, raw_dgraph_max_ms=%.3f, "
        "raw_dgraph_over_1ms=%zu, raw_planning_time_sec=%.3f, raw_expansions_placeholder=%zu, "
        "raw_batch_lock_calls=%zu, raw_batch_lock_wait_ms=%.3f, raw_batch_lock_max_ms=%.3f, "
        "hybrid_segments=%zu, hybrid_segments_succeeded=%zu, hybrid_dgraph_calls=%zu, "
        "hybrid_dgraph_query_ms=%.3f, hybrid_dgraph_max_ms=%.3f, hybrid_dgraph_over_1ms=%zu, "
        "hybrid_segment_search_ms=%.3f, hybrid_expansions=%zu, hybrid_projection_ms=%.3f, "
        "hybrid_batch_lock_calls=%zu, hybrid_batch_lock_wait_ms=%.3f, hybrid_batch_lock_max_ms=%.3f",
        goal_seq,
        request_id,
        result_class,
        raw_route_ms_value,
        hybrid_refine_ms_value,
        elapsed_seconds(total_started_at) * 1000.0,
        raw_route_search_diagnostics.projection_ms,
        raw_route_search_diagnostics.aggregate_lethal_ms,
        raw_route_search_diagnostics.graph_search_ms,
        raw_route_search_diagnostics.dgraph_calls,
        raw_route_search_diagnostics.dgraph_query_ms,
        raw_route_search_diagnostics.dgraph_max_ms,
        raw_route_search_diagnostics.dgraph_over_1ms,
        raw_route_search_diagnostics.planning_time_sec,
        raw_route_search_diagnostics.expansions,
        raw_route_search_diagnostics.batch_lock_calls,
        raw_route_search_diagnostics.batch_lock_wait_ms,
        raw_route_search_diagnostics.batch_lock_max_ms,
        refine.segments,
        refine.segments_succeeded,
        refine.dgraph_calls,
        refine.dgraph_query_ms,
        refine.dgraph_max_ms,
        refine.dgraph_over_1ms,
        refine.segment_search_ms,
        refine.expansions,
        refine.projection_ms,
        refine.batch_lock_calls,
        refine.batch_lock_wait_ms,
        refine.batch_lock_max_ms);
    };

  if(was_canceled != nullptr && *was_canceled){
    if(request_result_class != nullptr){
      *request_result_class = "planner_canceled";
    }
    if(enable_detail_log_){
      RCLCPP_WARN(
        this->get_logger(),
        "global_planner total timing: backend=%s, result=planner_canceled, total_time=%.3f ms",
        raw_route_backend_.c_str(),
        elapsed_seconds(total_started_at) * 1000.0);
    }
    log_request_timing("planner_canceled", raw_route_ms, 0.0);
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
      raw_route_backend_ == "hybrid_a_star" ? "graph_a_star" : raw_route_backend_.c_str(),
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
      raw_route_backend_ == "hybrid_a_star" ? "graph_a_star" : raw_route_backend_.c_str(),
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
    if(enable_detail_log_){
      const std::string result_class =
        request_result_class ? *request_result_class : "failed_raw_route";
      RCLCPP_WARN(
        this->get_logger(),
        "global_planner total timing: backend=%s, result=%s, total_time=%.3f ms",
        raw_route_backend_.c_str(),
        result_class.c_str(),
        elapsed_seconds(total_started_at) * 1000.0);
    }
    log_request_timing(
      request_result_class ? request_result_class->c_str() : "failed_raw_route",
      raw_route_ms,
      0.0);
    return false;
  }

  if(cancel_requested && cancel_requested()){
    if(was_canceled != nullptr){
      *was_canceled = true;
    }
    if(request_result_class != nullptr){
      *request_result_class = "planner_canceled";
    }
    if(enable_detail_log_){
      RCLCPP_WARN(
        this->get_logger(),
        "global_planner total timing: backend=%s, result=planner_canceled, total_time=%.3f ms",
        raw_route_backend_.c_str(),
        elapsed_seconds(total_started_at) * 1000.0);
    }
    log_request_timing("planner_canceled", raw_route_ms, 0.0);
    frozen_route->poses.clear();
    return false;
  }

  const auto composing_started_at = std::chrono::steady_clock::now();
  publish_stage("composing_frozen_route", composing_started_at, 0, "in_progress");
  double hybrid_refine_ms = 0.0;
  if(raw_route_backend_ == "hybrid_a_star"){
    bool refine_canceled = false;
    std::ostringstream refine_debug_label;
    refine_debug_label
      << "hybrid_refine goal_seq=" << goal_seq
      << ", request_id=" << request_id
      << ", stage=hybrid_refine";
    const auto hybrid_refine_started_at = std::chrono::steady_clock::now();
    *frozen_route = makeHybridRefinedRouteLocked(
      start,
      goal,
      *raw_route,
      scene,
      cancel_requested,
      &refine_canceled,
      refine_debug_label.str());
    hybrid_refine_ms = ElapsedMs(hybrid_refine_started_at);
    if(refine_canceled){
      if(was_canceled != nullptr){
        *was_canceled = true;
      }
      if(request_result_class != nullptr){
        *request_result_class = "planner_canceled";
      }
      if(enable_detail_log_){
        RCLCPP_WARN(
          this->get_logger(),
          "global_planner total timing: backend=%s, result=hybrid_refine_canceled, total_time=%.3f ms",
          raw_route_backend_.c_str(),
          elapsed_seconds(total_started_at) * 1000.0);
      }
      log_request_timing("hybrid_refine_canceled", raw_route_ms, hybrid_refine_ms);
      frozen_route->poses.clear();
      return false;
    }
  }
  if(request_result_class != nullptr){
    *request_result_class = "succeeded";
  }
  log_request_timing("succeeded", raw_route_ms, hybrid_refine_ms);
  if(enable_detail_log_){
    RCLCPP_INFO(
      this->get_logger(),
      "global_planner total timing: backend=%s, result=succeeded, total_time=%.3f ms, raw_route_poses=%zu, frozen_route_poses=%zu",
      raw_route_backend_.c_str(),
      elapsed_seconds(total_started_at) * 1000.0,
      raw_route->poses.size(),
      frozen_route->poses.size());
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

  ScenePipelineContext scene;
  std::string scene_reason;
  if(!acquirePinnedScene(&scene, &scene_reason)){
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Received the request before one complete static/cost scene is ready: %s",
      scene_reason.c_str());
    goal_handle->abort(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  nav_msgs::msg::Path raw_route;
  nav_msgs::msg::Path frozen_route;
  bool planner_canceled = false;
  bool route_validated = false;
  std::string request_result_class = "failed_raw_route";

  if(cancel_requested()){
    finish_canceled("planner canceled before route computation started");
    return;
  }
  const auto route_request_started_at = std::chrono::steady_clock::now();

  //@ Same bounded policy as the public makeROSPlan(): one pinned scene per
  //@ attempt, accept only a complete latest scene, and replan only for a
  //@ changed static map or an affected dynamic route. A malformed latest scene
  //@ is a refusal, never a stale-policy acceptance.
  constexpr int kMaxLatestSceneReplans = 2;
  for(int attempt = 0; attempt <= kMaxLatestSceneReplans; ++attempt){
    raw_route.poses.clear();
    frozen_route.poses.clear();
    planner_canceled = false;
    request_result_class = "failed_raw_route";
    {
      std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
      if(!buildFrozenRouteLocked(
           start,
           goal->goal,
           scene,
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
      break;
    }
    if(raw_route.poses.empty()){
      break;
    }

    ScenePipelineContext latest;
    std::string result_class;
    std::string detail;
    const LatestRouteStatus status = validateRouteAgainstLatest(
      scene, frozen_route.poses.empty() ? raw_route : frozen_route,
      &latest, &result_class, &detail);
    if(status == LatestRouteStatus::kAccepted){
      route_validated = true;
      break;
    }
    if(status == LatestRouteStatus::kRejected){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_latest_rejected: refusing the action route instead of accepting stale data, result_class=%s, detail=%s",
        result_class.c_str(),
        detail.c_str());
      request_result_class = result_class;
      break;
    }
    if(attempt >= kMaxLatestSceneReplans){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_replan_budget_exhausted: result_class=%s, detail=%s",
        result_class.c_str(),
        detail.c_str());
      request_result_class = result_class;
      break;
    }

    if(latest.valid()){
      scene = latest;
    }
    else{
      std::string reason;
      if(!acquirePinnedScene(&scene, &reason)){
        request_result_class = "scene_latest_rejected";
        break;
      }
    }
    RCLCPP_WARN(
      this->get_logger(),
      "scene_replan: attempt=%d, result_class=%s, detail=%s",
      attempt + 1,
      result_class.c_str(),
      detail.c_str());
  }
  const double route_request_finished_sec =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - route_request_started_at).count();

  if(planner_canceled || cancel_requested()){
    publishRouteRequestStageDebug(
      request_id,
      goal_seq,
      "finished",
      route_request_finished_sec,
      0,
      "planner_canceled");
    finish_canceled(
      "planner canceled while computing route; client-side wait ended before planner finished");
    return;
  }

  if(!route_validated || raw_route.poses.empty()){
    publishRouteRequestStageDebug(
      request_id,
      goal_seq,
      "finished",
      route_request_finished_sec,
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
      route_request_finished_sec,
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
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label,
  ForwardHybridAStar::SearchDiagnostics * search_diagnostics,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection,
  const ForwardHybridAStar::ProgressCallback & progress_callback)
{
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);

  const auto failed_path = [this]() {
      nav_msgs::msg::Path path;
      path.header.frame_id = global_frame_;
      path.header.stamp = clock_->now();
      return path;
    };

  ScenePipelineContext scene;
  std::string scene_reason;
  if(!acquirePinnedScene(&scene, &scene_reason)){
    return failed_path();
  }

  //@ Bounded work only: no new tunable parameter list. The first search uses
  //@ one pinned scene; a changed map or an affected dynamic route may replan
  //@ at most twice against the latest pinned scene.
  constexpr int kMaxLatestSceneReplans = 2;
  for(int attempt = 0; attempt <= kMaxLatestSceneReplans; ++attempt){
    nav_msgs::msg::Path path = makeROSPlanLocked(
      start,
      goal,
      scene,
      force_position_only_goal,
      force_use_goal_heading,
      cancel_requested,
      was_canceled,
      debug_label,
      search_diagnostics,
      start_projection,
      goal_projection,
      progress_callback);
    if(path.poses.empty()){
      return path;
    }

    ScenePipelineContext latest;
    std::string result_class;
    std::string detail;
    const LatestRouteStatus status = validateRouteAgainstLatest(
      scene, path, &latest, &result_class, &detail);
    if(status == LatestRouteStatus::kAccepted){
      return path;
    }
    if(status == LatestRouteStatus::kRejected){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_latest_rejected: refusing the route instead of accepting stale data, result_class=%s, detail=%s",
        result_class.c_str(),
        detail.c_str());
      if(search_diagnostics != nullptr){
        search_diagnostics->success = false;
        search_diagnostics->path_pose_count = 0;
      }
      return failed_path();
    }
    if(attempt >= kMaxLatestSceneReplans){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_replan_budget_exhausted: result_class=%s, detail=%s",
        result_class.c_str(),
        detail.c_str());
      if(search_diagnostics != nullptr){
        search_diagnostics->success = false;
        search_diagnostics->path_pose_count = 0;
      }
      return failed_path();
    }

    if(latest.valid()){
      scene = latest;
    }
    else{
      std::string reason;
      if(!acquirePinnedScene(&scene, &reason)){
        return failed_path();
      }
    }
    RCLCPP_WARN(
      this->get_logger(),
      "scene_replan: attempt=%d, result_class=%s, detail=%s",
      attempt + 1,
      result_class.c_str(),
      detail.c_str());
  }
  return failed_path();
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlanLocked(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  const ScenePipelineContext & scene,
  bool force_position_only_goal,
  bool force_use_goal_heading,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label,
  ForwardHybridAStar::SearchDiagnostics * search_diagnostics,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection,
  const ForwardHybridAStar::ProgressCallback & progress_callback,
  const std::string & route_backend_override){

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
  if(!scene.valid()){
    RCLCPP_WARN(
      this->get_logger(),
      "scene_not_ready: graph/hybrid planning refused without a pinned scene");
    ros_path.poses.clear();
    return ros_path;
  }

  const std::string & route_backend =
    route_backend_override.empty() ? raw_route_backend_ : route_backend_override;
  const bool raw_route_uses_forward_hybrid_astar = route_backend == "hybrid_a_star";
  const auto elapsed_seconds =
    [](const std::chrono::steady_clock::time_point & started_at) {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
    };

  if(raw_route_uses_forward_hybrid_astar && forward_hybrid_astar_planner_){
    std::string scene_reason;
    if(!forward_hybrid_astar_planner_->SetSceneSnapshot(scene, &scene_reason)){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_not_ready: hybrid planner refused the pinned scene: %s",
        scene_reason.c_str());
      ros_path.poses.clear();
      return ros_path;
    }
    nav_msgs::msg::Path hybrid_path;
    bool hybrid_planning_canceled = false;
    if(forward_hybrid_astar_planner_->MakePlan(
        start, goal, &hybrid_path,
        force_position_only_goal,
        force_use_goal_heading,
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

  {
    std::string scene_reason;
    const bool scene_pinned = use_pre_graph_ ?
      (a_star_planner_pre_graph_ != nullptr &&
      a_star_planner_pre_graph_->setSceneSnapshot(
        scene.snapshot(), scene.parameters(), &scene_reason)) :
      (a_star_planner_ != nullptr &&
      a_star_planner_->setSceneSnapshot(
        scene.snapshot(), scene.parameters(), &scene_reason));
    if(!scene_pinned){
      RCLCPP_WARN(
        this->get_logger(),
        "scene_not_ready: %s A* refused the pinned scene: %s; live fallback is disabled",
        use_pre_graph_ ? "pre-graph" : "graph",
        scene_reason.c_str());
      ros_path.poses.clear();
      return ros_path;
    }
  }

  const auto graph_planning_started_at = std::chrono::steady_clock::now();
  const auto endpoint_projection_started_at = std::chrono::steady_clock::now();
  has_start_goal_id =
    getStartGoalID(
      start, goal, scene, start_id, goal_id, start_projection, goal_projection);
  const double endpoint_projection_ms = ElapsedMs(endpoint_projection_started_at);
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
      search_diagnostics->projection_ms = endpoint_projection_ms;
    }
    return mark_canceled();
  }
  GraphPlanDiagnostics graph_diagnostics;
  const auto graph_search_started_at = std::chrono::steady_clock::now();
  if(has_start_goal_id){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path, &graph_diagnostics);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);  
  }
  const double graph_search_ms = ElapsedMs(graph_search_started_at);
  if(search_diagnostics != nullptr){
    search_diagnostics->success = !path.empty();
    search_diagnostics->canceled = false;
    search_diagnostics->expansions = 0;
    search_diagnostics->planning_time_sec = elapsed_seconds(graph_planning_started_at);
    search_diagnostics->path_pose_count = 0;
    search_diagnostics->projection_ms = endpoint_projection_ms;
    search_diagnostics->aggregate_lethal_ms = graph_diagnostics.aggregate_lethal_ms;
    search_diagnostics->graph_search_ms = graph_search_ms;
    search_diagnostics->dgraph_calls = graph_diagnostics.dgraph_calls;
    search_diagnostics->dgraph_query_ms = graph_diagnostics.dgraph_query_ms;
    search_diagnostics->dgraph_max_ms = graph_diagnostics.dgraph_max_ms;
    search_diagnostics->dgraph_over_1ms = graph_diagnostics.dgraph_over_1ms;
    search_diagnostics->batch_lock_calls = graph_diagnostics.batch_lock_calls;
    search_diagnostics->batch_lock_wait_ms = graph_diagnostics.batch_lock_wait_ms;
    search_diagnostics->batch_lock_max_ms = graph_diagnostics.batch_lock_max_ms;
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
    getROSPath(path, ros_path, scene);
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
    smoothGraphPathForAckermann(&ros_path, scene);
    if(search_diagnostics != nullptr){
      search_diagnostics->planning_time_sec = elapsed_seconds(graph_planning_started_at);
      search_diagnostics->path_pose_count = ros_path.poses.size();
    }
    return ros_path;
  }
}

nav_msgs::msg::Path GlobalPlanner::makeGraphRouteLocked(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const ScenePipelineContext & scene,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  ForwardHybridAStar::SearchDiagnostics * search_diagnostics,
  ForwardHybridAStar::ProjectionDiagnostics * start_projection,
  ForwardHybridAStar::ProjectionDiagnostics * goal_projection)
{
  return makeROSPlanLocked(
    start,
    goal,
    scene,
    false,
    false,
    cancel_requested,
    was_canceled,
    "graph_global_route",
    search_diagnostics,
    start_projection,
    goal_projection,
    ForwardHybridAStar::ProgressCallback(),
    "graph_a_star");
}

nav_msgs::msg::Path GlobalPlanner::makeHybridRefinedRouteLocked(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & graph_route,
  const ScenePipelineContext & scene,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label)
{
  nav_msgs::msg::Path refined_route;
  refined_route.header.frame_id = global_frame_;
  refined_route.header.stamp = clock_->now();
  g_hybrid_refine_timing.reset();

  if(was_canceled != nullptr){
    *was_canceled = false;
  }
  if(graph_route.poses.empty() || !forward_hybrid_astar_planner_){
    return graph_route;
  }

  //@ Re-pin the exact route scene before every refine request. A failed pin is
  //@ a hard refusal: keep the already-valid graph route and never let the
  //@ hybrid backend fall back to live SharedData.
  std::string scene_reason;
  if(!forward_hybrid_astar_planner_->SetSceneSnapshot(scene, &scene_reason)){
    RCLCPP_WARN(
      this->get_logger(),
      "scene_not_ready: hybrid refine refused the pinned scene: %s",
      scene_reason.c_str());
    return graph_route;
  }

  const auto mark_canceled = [&]() {
    if(was_canceled != nullptr){
      *was_canceled = true;
    }
    refined_route.poses.clear();
  };

  if(cancel_requested && cancel_requested()){
    mark_canceled();
    return refined_route;
  }

  std::size_t segment_start_index = 0;
  geometry_msgs::msg::PoseStamped segment_start = start;
  segment_start.header = graph_route.header;
  const std::size_t last_index = graph_route.poses.size() - 1;

  while(segment_start_index < last_index){
    if(cancel_requested && cancel_requested()){
      mark_canceled();
      return refined_route;
    }

    std::size_t segment_goal_index = IndexAtDistanceFromIndex(
      graph_route,
      segment_start_index,
      kHybridRefineSegmentLength);
    if(segment_goal_index <= segment_start_index){
      segment_goal_index = last_index;
    }

    geometry_msgs::msg::PoseStamped segment_goal = graph_route.poses[segment_goal_index];
    segment_goal.header = graph_route.header;
    const bool is_final_segment = segment_goal_index == last_index;
    if(is_final_segment){
      segment_goal.pose.position = goal.pose.position;
      segment_goal.pose.orientation = goal.pose.orientation;
    }

    bool use_graph_segment = true;
    if(Distance2D(segment_start, segment_goal) >= kHybridRefineMinSegmentLength){
      nav_msgs::msg::Path hybrid_segment;
      bool segment_canceled = false;
      std::ostringstream segment_label;
      segment_label
        << debug_label
        << " hybrid_refine_segment start_index=" << segment_start_index
        << ", goal_index=" << segment_goal_index;
      ForwardHybridAStar::SearchDiagnostics segment_diagnostics;
      const auto segment_started_at = std::chrono::steady_clock::now();
      const bool segment_plan_ok = forward_hybrid_astar_planner_->MakePlan(
          segment_start,
          segment_goal,
          &hybrid_segment,
          false,
          true,
          cancel_requested,
          &segment_canceled,
          segment_label.str(),
          &segment_diagnostics);
      const double segment_ms = ElapsedMs(segment_started_at);
      g_hybrid_refine_timing.add(
        segment_diagnostics,
        segment_ms,
        segment_plan_ok && !hybrid_segment.poses.empty());
      if(segment_plan_ok && !hybrid_segment.poses.empty())
      {
        hybrid_segment.header.frame_id = global_frame_;
        hybrid_segment.header.stamp = clock_->now();
        if(is_final_segment){
          hybrid_segment.poses.back().pose.orientation = goal.pose.orientation;
        }
        AppendPath(hybrid_segment, &refined_route, 0, hybrid_segment.poses.size() - 1);
        segment_start = hybrid_segment.poses.back();
        segment_start.header = graph_route.header;
        use_graph_segment = false;
      }
      else if(segment_canceled){
        mark_canceled();
        return refined_route;
      }
    }

    if(use_graph_segment){
      AppendPath(graph_route, &refined_route, segment_start_index, segment_goal_index);
      segment_start = segment_goal;
    }

    segment_start_index = segment_goal_index;
  }

  if(refined_route.poses.empty()){
    return graph_route;
  }
  refreshPathOrientations(&refined_route);
  if(!refined_route.poses.empty()){
    refined_route.poses.back().pose.orientation = goal.pose.orientation;
  }
  return refined_route;
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  const bool needs_graph_astar =
    raw_route_backend_ == "graph_a_star" || raw_route_backend_ == "hybrid_a_star";

  if(!has_initialized_){
    has_initialized_ = true;
    if(needs_graph_astar){
      if(perception_3d_ros_ && perception_3d_ros_->getGlobalUtils() &&
        a_star_expanding_radius_ >=
        perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2)
      {
        RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
      }
      //@ Snapshot-only construction: no live ground copy or FLANN tree is
      //@ built here. Every request pins the immutable view with
      //@ setSceneSnapshot() before projection/search. The pre-graph
      //@ StaticGraph argument is only a legacy placeholder.
      if(!use_pre_graph_){
        a_star_planner_ = std::make_shared<A_Star_on_Graph>(
          nullptr, perception_3d_ros_, a_star_expanding_radius_);
        a_star_planner_->setupTurningWeight(turning_weight_);
      }
      else{
        a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(
          nullptr, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
        a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
      }
    }

    forward_hybrid_astar_planner_ = std::make_shared<ForwardHybridAStar>(
      perception_3d_ros_, nullptr, nullptr, this->get_logger());
    forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
    forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
  }
  else{
    if(a_star_planner_){
      a_star_planner_->setupTurningWeight(turning_weight_);
    }
    if(a_star_planner_pre_graph_){
      a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
    }
    if(forward_hybrid_astar_planner_){
      forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
      forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
    }
  }

  pubWeight();
  pubStaticGraph();

  RCLCPP_INFO(this->get_logger(), "Publish weighted ground point cloud.");
  graph_ready_ = true;
}


void GlobalPlanner::pubWeight(){

  std::shared_ptr<const perception_3d::LegacyStaticView> static_view;
  {
    std::lock_guard<std::mutex> cache_lock(scene_cache_mutex_);
    static_view = current_static_view_;
  }
  if(!static_view || !static_view->groundPoints()){
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);

  const auto & ground_points = *static_view->groundPoints();
  const perception_3d::LegacyStaticGraph & graph = static_view->graph();
  const size_t publish_size =
    std::min(graph.nodeWeightCount(), ground_points.points.size());
  for(size_t it=0; it<publish_size; it++){

    pcl::PointXYZI ipt;

    ipt.x = ground_points.points[it].x;
    ipt.y = ground_points.points[it].y;
    ipt.z = ground_points.points[it].z;
    ipt.intensity = graph.getNodeWeight(it);
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void GlobalPlanner::pubStaticGraph(){

  std::shared_ptr<const perception_3d::LegacyStaticView> static_view;
  {
    std::lock_guard<std::mutex> cache_lock(scene_cache_mutex_);
    static_view = current_static_view_;
  }
  if(!static_view || !static_view->groundPoints()){
    return;
  }

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

  const auto & ground_points = *static_view->groundPoints();
  const perception_3d::LegacyStaticGraph & graph = static_view->graph();
  const unsigned int node_count =
    static_cast<unsigned int>(graph.nodeCount());

  int cnt = 0;
  for(unsigned int node = 0; node < node_count; ++node){
    if(node >= ground_points.points.size()){
      break;
    }
    geometry_msgs::msg::Point p;
    p.x = ground_points.points[node].x;
    p.y = ground_points.points[node].y;
    p.z = ground_points.points[node].z;
    const edges_t & edges = graph.getEdge(node);
    for(auto it_set = edges.begin(); it_set != edges.end(); it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = node;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = node;
      if( !duplicate_check.insert(edge_marker).second )
      {
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {
        continue;
      }
      markerEdge.points.push_back(p);
      if((*it_set).first >= ground_points.points.size()){
        continue;
      }
      p.x = ground_points.points[(*it_set).first].x;
      p.y = ground_points.points[(*it_set).first].y;
      p.z = ground_points.points[(*it_set).first].z;
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}


}
