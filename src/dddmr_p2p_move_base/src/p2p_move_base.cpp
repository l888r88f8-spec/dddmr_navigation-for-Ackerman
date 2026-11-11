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
#include <p2p_move_base/p2p_move_base.h>
#include <p2p_move_base/parameter_utils.h>
#include <p2p_move_base/publish_feedback_if_active.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

#include <p2p_move_base/local_obstacle_layer_ros.h>
#include <tf2/LinearMath/Transform.h>

namespace p2p_move_base
{
namespace
{
void transformLocalPoints(
  std::vector<LocalPoint> * points,
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  if (points == nullptr) {
    return;
  }

  const auto & rotation_msg = transform_msg.transform.rotation;
  const auto & translation_msg = transform_msg.transform.translation;
  const tf2::Quaternion rotation(
    rotation_msg.x,
    rotation_msg.y,
    rotation_msg.z,
    rotation_msg.w);
  const tf2::Transform transform(
    rotation,
    tf2::Vector3(translation_msg.x, translation_msg.y, translation_msg.z));

  for (auto & point : *points) {
    const tf2::Vector3 transformed = transform * tf2::Vector3(point.x, point.y, point.z);
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
  }
}
}  // namespace

P2PMoveBase::P2PMoveBase(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  is_recoverying_ = false;
  is_recoverying_succeed_ = false;
  terminal_goal_reason_ = "goal failed";
  current_route_result_class_ = "unknown";
  min_replan_interval_sec_ = 3.0;
  runtime_budget_safety_stop_enabled_ = false;
  local_obstacle_layer_enabled_ = false;
  local_obstacle_debug_cloud_enabled_ = false;
  local_obstacle_base_frame_ = "base_link";
}

rclcpp_action::GoalResponse P2PMoveBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal)
{
  (void)uuid;
  if(stop_requested_.load()){
    RCLCPP_WARN(this->get_logger(), "Rejecting navigation goal because execution is stopping.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse P2PMoveBase::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void P2PMoveBase::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> previous_handle;
  {
    std::lock_guard<std::mutex> lock(current_handle_mutex_);
    previous_handle = current_handle_;
    current_handle_ = goal_handle;
  }

  if(previous_handle && previous_handle != goal_handle){
    RCLCPP_DEBUG(this->get_logger(), "An older goal is active, cancelling current one.");
    abortGoalBestEffort(previous_handle, "preempted by newer goal");
  }

  if(stop_requested_.load()){
    abortGoalBestEffort(goal_handle, "navigation is stopping");
    publishZeroVelocityBestEffort();
    clearCurrentHandleIf(goal_handle);
    return;
  }

  ExecutionWorker::Task task;
  task.run = [this, goal_handle]() {
      executeCb(goal_handle);
      clearCurrentHandleIf(goal_handle);
    };
  task.on_replaced = [this, goal_handle]() {
      abortGoalBestEffort(goal_handle, "goal replaced by newer goal");
      clearCurrentHandleIf(goal_handle);
    };
  task.on_failed = [this, goal_handle](std::exception_ptr error) {
      handleExecutionException(goal_handle, error);
    };

  if(!execution_worker_.submit(std::move(task))){
    RCLCPP_WARN(
      this->get_logger(),
      "Rejecting navigation goal because the execution worker is stopping.");
    abortGoalBestEffort(goal_handle, "execution worker stopped");
    publishZeroVelocityBestEffort();
    clearCurrentHandleIf(goal_handle);
  }
}

void P2PMoveBase::initial(
  const std::shared_ptr<dddmr_sys_core::RouteTrackingController>& route_controller,
  const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& route_manager){
  
  route_controller_ = route_controller;
  route_manager_ = route_manager;

  FSM_ = std::make_shared<p2p_move_base::FSM>(this->get_node_logging_interface(), this->get_node_parameters_interface());
  oscillation_watchdog_.setPatience(FSM_->oscillation_patience_);
  blocked_wait_watchdog_.setPatience(FSM_->blocked_wait_patience_);
  
  if(FSM_->use_twist_stamped_){
    stamped_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 1);
  }
  else{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }
  

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
  
  recovery_behaviors_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recovery_behaviors_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::RecoveryBehaviors>(
      this,
      "recovery_behaviors", recovery_behaviors_client_group_);

  this->declare_parameter("tracking_controller_name", rclcpp::ParameterValue(""));
  this->get_parameter("tracking_controller_name", tracking_controller_name_);
  this->declare_parameter("drive_trajectory_generator_name", rclcpp::ParameterValue("differential_drive_simple"));
  std::string legacy_tracking_controller_name;
  this->get_parameter("drive_trajectory_generator_name", legacy_tracking_controller_name);
  if(tracking_controller_name_.empty()){
    tracking_controller_name_ = legacy_tracking_controller_name;
    RCLCPP_DEBUG(
      this->get_logger(),
      "drive_trajectory_generator_name (legacy tracking controller alias): %s",
      tracking_controller_name_.c_str());
  }
  else{
    RCLCPP_DEBUG(this->get_logger(), "tracking_controller_name: %s", tracking_controller_name_.c_str());
  }

  this->declare_parameter("alignment_controller_name", rclcpp::ParameterValue(""));
  this->get_parameter("alignment_controller_name", alignment_controller_name_);
  this->declare_parameter("rotate_trajectory_generator_name", rclcpp::ParameterValue("differential_drive_rotate_shortest_angle"));
  std::string legacy_alignment_controller_name;
  this->get_parameter("rotate_trajectory_generator_name", legacy_alignment_controller_name);
  if(alignment_controller_name_.empty()){
    alignment_controller_name_ = legacy_alignment_controller_name;
    RCLCPP_DEBUG(
      this->get_logger(),
      "rotate_trajectory_generator_name (legacy alignment controller alias): %s",
      alignment_controller_name_.c_str());
  }
  else{
    RCLCPP_DEBUG(this->get_logger(), "alignment_controller_name: %s", alignment_controller_name_.c_str());
  }

  this->declare_parameter("recovery_action_name", rclcpp::ParameterValue(""));
  this->get_parameter("recovery_action_name", recovery_action_name_);
  this->declare_parameter("recovery_behavior_name", rclcpp::ParameterValue(""));
  std::string legacy_recovery_action_name;
  this->get_parameter("recovery_behavior_name", legacy_recovery_action_name);
  if(recovery_action_name_.empty()){
    recovery_action_name_ = legacy_recovery_action_name;
  }
  if(recovery_action_name_.empty()){
    RCLCPP_WARN(this->get_logger(), "recovery_action_name is empty, explicit recovery action is disabled.");
  }
  else if(recovery_action_name_ == legacy_recovery_action_name){
    RCLCPP_DEBUG(
      this->get_logger(),
      "recovery_behavior_name (legacy recovery action alias): %s",
      recovery_action_name_.c_str());
  }
  else{
    RCLCPP_DEBUG(this->get_logger(), "recovery_action_name: %s", recovery_action_name_.c_str());
  }

  this->declare_parameter("collision_confirm_cycles", rclcpp::ParameterValue(5));
  this->declare_parameter("blocked_confirm_time", rclcpp::ParameterValue(1.0));
  this->declare_parameter("min_replan_interval", rclcpp::ParameterValue(3.0));
  this->declare_parameter("route_deviation_confirm_cycles", rclcpp::ParameterValue(10));
  this->declare_parameter("route_deviation_confirm_time", rclcpp::ParameterValue(1.0));
  this->declare_parameter("runtime_budget_safety_stop_enabled", rclcpp::ParameterValue(false));
  this->declare_parameter("max_local_planning_time_ms", rclcpp::ParameterValue(50.0));
  this->declare_parameter("max_end_to_end_local_cycle_ms", rclcpp::ParameterValue(80.0));
  this->declare_parameter("budget_warn_after_consecutive_overruns", rclcpp::ParameterValue(2));
  this->declare_parameter("budget_stop_after_consecutive_overruns", rclcpp::ParameterValue(10));

  NavigationFaultPolicy::Config fault_config;
  fault_config.collision_confirm_cycles = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("collision_confirm_cycles").as_int()));
  fault_config.blocked_confirm_time_sec = this->get_parameter("blocked_confirm_time").as_double();
  fault_config.min_replan_interval_sec = this->get_parameter("min_replan_interval").as_double();
  fault_config.route_deviation_confirm_cycles = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("route_deviation_confirm_cycles").as_int()));
  fault_config.route_deviation_confirm_time_sec =
    this->get_parameter("route_deviation_confirm_time").as_double();
  fault_config.control_failure_patience_sec = FSM_->controller_patience_;
  navigation_fault_policy_.setConfig(fault_config);
  min_replan_interval_sec_ = fault_config.min_replan_interval_sec;

  RuntimeBudgetMonitor::Config budget_config;
  budget_config.max_local_planning_time_ms =
    this->get_parameter("max_local_planning_time_ms").as_double();
  budget_config.max_end_to_end_local_cycle_ms =
    this->get_parameter("max_end_to_end_local_cycle_ms").as_double();
  budget_config.warn_after_consecutive_overruns = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("budget_warn_after_consecutive_overruns").as_int()));
  budget_config.stop_after_consecutive_overruns = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("budget_stop_after_consecutive_overruns").as_int()));
  runtime_budget_monitor_.setConfig(budget_config);
  runtime_budget_safety_stop_enabled_ =
    this->get_parameter("runtime_budget_safety_stop_enabled").as_bool();

  initializeLocalObstacleLayer();

  //@Create action server
  action_server_p2p_move_base_ = rclcpp_action::create_server<dddmr_sys_core::action::PToPMoveBase>(
    this,
    "/p2p_move_base",
    std::bind(&P2PMoveBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&P2PMoveBase::handle_cancel, this, std::placeholders::_1),
    std::bind(&P2PMoveBase::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);

  RCLCPP_DEBUG(this->get_logger(), "\033[1;32m---->\033[0m P2P move base launched.");

}

void P2PMoveBase::initializeLocalObstacleLayer()
{
  this->declare_parameter("local_obstacle_layer.enabled", rclcpp::ParameterValue(false));
  const std::string input_cloud_topic = getStringParameter(
    *this, "local_obstacle_layer.input_cloud_topic", "input_cloud_topic",
    "lio_sam/cloud_nonground");
  this->declare_parameter("local_obstacle_layer.status_topic", rclcpp::ParameterValue("local_obstacle_layer/status"));
  this->declare_parameter("local_obstacle_layer.debug_cloud_topic", rclcpp::ParameterValue("local_obstacle_layer/cloud"));
  this->declare_parameter("local_obstacle_layer.roi_forward", rclcpp::ParameterValue(6.0));
  this->declare_parameter("local_obstacle_layer.roi_backward", rclcpp::ParameterValue(1.0));
  this->declare_parameter("local_obstacle_layer.roi_left", rclcpp::ParameterValue(3.0));
  this->declare_parameter("local_obstacle_layer.roi_right", rclcpp::ParameterValue(3.0));
  this->declare_parameter("local_obstacle_layer.min_z", rclcpp::ParameterValue(-0.15));
  this->declare_parameter("local_obstacle_layer.max_z", rclcpp::ParameterValue(2.0));
  this->declare_parameter("local_obstacle_layer.voxel_leaf_size", rclcpp::ParameterValue(0.08));
  this->declare_parameter("local_obstacle_layer.max_snapshot_points", rclcpp::ParameterValue(30000));
  this->declare_parameter("local_obstacle_layer.max_update_time_ms", rclcpp::ParameterValue(20.0));
  this->declare_parameter("local_obstacle_layer.stale_timeout_sec", rclcpp::ParameterValue(0.3));
  this->declare_parameter("local_obstacle_layer.publish_debug_cloud", rclcpp::ParameterValue(false));
  this->declare_parameter("local_obstacle_layer.footprint_front", rclcpp::ParameterValue(0.50));
  this->declare_parameter("local_obstacle_layer.footprint_back", rclcpp::ParameterValue(0.38));
  this->declare_parameter("local_obstacle_layer.footprint_left", rclcpp::ParameterValue(0.32));
  this->declare_parameter("local_obstacle_layer.footprint_right", rclcpp::ParameterValue(0.32));

  LocalObstacleLayerConfig config;
  config.roi_forward = this->get_parameter("local_obstacle_layer.roi_forward").as_double();
  config.roi_backward = this->get_parameter("local_obstacle_layer.roi_backward").as_double();
  config.roi_left = this->get_parameter("local_obstacle_layer.roi_left").as_double();
  config.roi_right = this->get_parameter("local_obstacle_layer.roi_right").as_double();
  config.min_z = this->get_parameter("local_obstacle_layer.min_z").as_double();
  config.max_z = this->get_parameter("local_obstacle_layer.max_z").as_double();
  config.voxel_leaf_size = this->get_parameter("local_obstacle_layer.voxel_leaf_size").as_double();
  config.max_snapshot_points = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("local_obstacle_layer.max_snapshot_points").as_int()));
  config.max_update_time_ms =
    this->get_parameter("local_obstacle_layer.max_update_time_ms").as_double();
  config.stale_timeout_sec = this->get_parameter("local_obstacle_layer.stale_timeout_sec").as_double();
  config.publish_debug_cloud =
    this->get_parameter("local_obstacle_layer.publish_debug_cloud").as_bool();
  config.footprint_front = this->get_parameter("local_obstacle_layer.footprint_front").as_double();
  config.footprint_back = this->get_parameter("local_obstacle_layer.footprint_back").as_double();
  config.footprint_left = this->get_parameter("local_obstacle_layer.footprint_left").as_double();
  config.footprint_right = this->get_parameter("local_obstacle_layer.footprint_right").as_double();
  local_obstacle_layer_.setConfig(config);

  local_obstacle_layer_enabled_ =
    this->get_parameter("local_obstacle_layer.enabled").as_bool();
  local_obstacle_debug_cloud_enabled_ = config.publish_debug_cloud;
  local_obstacle_base_frame_ = getStringParameter(
    *this, "local_obstacle_layer.base_frame", "robot_base_frame", "base_link");
  if (!local_obstacle_layer_enabled_) {
    RCLCPP_INFO(this->get_logger(), "local_obstacle_layer disabled; diagnostics subscription not created");
    return;
  }

  const std::string status_topic =
    this->get_parameter("local_obstacle_layer.status_topic").as_string();
  const std::string debug_cloud_topic =
    this->get_parameter("local_obstacle_layer.debug_cloud_topic").as_string();

  local_obstacle_layer_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = local_obstacle_layer_group_;
  local_obstacle_status_pub_ =
    this->create_publisher<std_msgs::msg::String>(status_topic, 1);
  if (local_obstacle_debug_cloud_enabled_) {
    local_obstacle_debug_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(debug_cloud_topic, 1);
  }
  local_obstacle_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&P2PMoveBase::localObstacleCloudCallback, this, std::placeholders::_1),
    subscription_options);

  RCLCPP_INFO(
    this->get_logger(),
    "local_obstacle_layer enabled: input=%s, base_frame=%s, status=%s, debug_cloud=%s",
    input_cloud_topic.c_str(),
    local_obstacle_base_frame_.c_str(),
    status_topic.c_str(),
    local_obstacle_debug_cloud_enabled_ ? debug_cloud_topic.c_str() : "disabled");
}

void P2PMoveBase::localObstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg || !local_obstacle_layer_enabled_) {
    return;
  }

  std::vector<LocalPoint> points = pointCloud2ToLocalPoints(*msg);
  const std::string source_frame =
    msg->header.frame_id.empty() ? local_obstacle_base_frame_ : msg->header.frame_id;
  if (source_frame != local_obstacle_base_frame_) {
    try {
      const auto transform = tf2Buffer_->lookupTransform(
        local_obstacle_base_frame_,
        source_frame,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
      transformLocalPoints(&points, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "local_obstacle_layer skipped cloud: transform %s -> %s unavailable: %s",
        source_frame.c_str(),
        local_obstacle_base_frame_.c_str(),
        ex.what());
      return;
    }
  }

  const double now_sec = clock_->now().seconds();
  const rclcpp::Time cloud_stamp(msg->header.stamp);
  const double stamp_sec = cloud_stamp.nanoseconds() == 0 ? now_sec : cloud_stamp.seconds();
  const auto snapshot = local_obstacle_layer_.buildSnapshot(
    points,
    stamp_sec,
    now_sec,
    local_obstacle_base_frame_);

  {
    std::lock_guard<std::mutex> lock(local_obstacle_snapshot_mutex_);
    latest_local_obstacle_snapshot_ = snapshot;
  }

  if (local_obstacle_status_pub_) {
    local_obstacle_status_pub_->publish(snapshotToStatusMessage(*snapshot));
  }

  if (local_obstacle_debug_cloud_enabled_ && local_obstacle_debug_cloud_pub_) {
    std_msgs::msg::Header debug_header = msg->header;
    debug_header.frame_id = local_obstacle_base_frame_;
    local_obstacle_debug_cloud_pub_->publish(snapshotToPointCloud2(*snapshot, debug_header));
  }

  if (snapshot->perception_stale || snapshot->over_budget) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "local_obstacle_layer status: stale=%d, over_budget=%d, raw=%zu, filtered=%zu, update_ms=%.3f",
      snapshot->perception_stale,
      snapshot->over_budget,
      snapshot->raw_points,
      snapshot->filtered_points,
      snapshot->update_time_ms);
  }
}

void P2PMoveBase::stopExecution()
{
  stop_requested_.store(true);
  // A join failure is a fatal teardown condition. Let it reach main (and the
  // destructor fail-fast) instead of returning while the worker is still alive.
  execution_worker_.stop();
}

bool P2PMoveBase::shouldStopExecution() const
{
  return stop_requested_.load() || !rclcpp::ok();
}

void P2PMoveBase::abortGoalBestEffort(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> & goal_handle,
  const std::string & reason)
{
  if(!goal_handle){
    return;
  }

  try{
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    if(goal_handle->is_canceling()){
      goal_handle->canceled(result);
      RCLCPP_DEBUG(this->get_logger(), "navigation goal canceled: %s", reason.c_str());
    }
    else if(goal_handle->is_active()){
      goal_handle->abort(result);
      RCLCPP_DEBUG(this->get_logger(), "navigation goal aborted: %s", reason.c_str());
    }
  }
  catch(const std::exception & ex){
    RCLCPP_WARN(
      this->get_logger(),
      "could not terminate navigation goal (%s): %s",
      reason.c_str(),
      ex.what());
  }
  catch(...){
    RCLCPP_WARN(
      this->get_logger(),
      "could not terminate navigation goal (%s): unknown exception",
      reason.c_str());
  }
}

void P2PMoveBase::handleExecutionException(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> & goal_handle,
  std::exception_ptr error)
{
  std::string message = "unknown exception";
  try{
    if(error){
      std::rethrow_exception(error);
    }
  }
  catch(const std::exception & ex){
    message = ex.what();
  }
  catch(...){
  }

  const bool stopping = stop_requested_.load() || !rclcpp::ok();
  try{
    if(stopping){
      RCLCPP_WARN(
        this->get_logger(),
        "navigation execution stopped with exception during shutdown: %s",
        message.c_str());
    }
    else{
      RCLCPP_ERROR(
        this->get_logger(),
        "navigation execution failed with uncaught exception: %s",
        message.c_str());
    }
  }
  catch(...){
  }

  terminal_goal_reason_ = stopping ? "goal stopped" : "goal failed";
  abortGoalBestEffort(goal_handle, stopping ? "navigation stopped" : "navigation execution failed");
  publishZeroVelocityBestEffort();

  try{
    if(route_manager_){
      route_manager_->stop(terminal_goal_reason_);
    }
  }
  catch(const std::exception & ex){
    try{
      RCLCPP_WARN(
        this->get_logger(),
        "route manager cleanup after execution failure did not complete: %s",
        ex.what());
    }
    catch(...){
    }
  }
  catch(...){
  }

  clearCurrentHandleIf(goal_handle);
}

void P2PMoveBase::publishZeroVelocityBestEffort()
{
  if(!rclcpp::ok() || !FSM_){
    return;
  }

  try{
    publishZeroVelocity();
  }
  catch(const std::exception & ex){
    try{
      RCLCPP_WARN(this->get_logger(), "could not publish zero velocity during shutdown: %s", ex.what());
    }
    catch(...){
    }
  }
  catch(...){
  }
}

void P2PMoveBase::cancelRecoveryBehaviorsBestEffort()
{
  if(!rclcpp::ok() || !recovery_behaviors_client_ptr_){
    return;
  }

  try{
    recovery_behaviors_client_ptr_->async_cancel_all_goals();
  }
  catch(const std::exception & ex){
    RCLCPP_WARN(
      this->get_logger(),
      "could not cancel recovery behaviors during shutdown: %s",
      ex.what());
  }
  catch(...){
    RCLCPP_WARN(
      this->get_logger(),
      "could not cancel recovery behaviors during shutdown: unknown exception");
  }
}

void P2PMoveBase::clearCurrentHandleIf(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> & goal_handle)
{
  std::lock_guard<std::mutex> lock(current_handle_mutex_);
  if(current_handle_ == goal_handle){
    current_handle_.reset();
  }
}

P2PMoveBase::~P2PMoveBase(){
  try{
    stopExecution();
  }
  catch(const std::exception & ex){
    try{
      RCLCPP_FATAL(
        this->get_logger(),
        "failed to join navigation execution worker during teardown: %s; terminating instead of releasing active dependencies",
        ex.what());
    }
    catch(...){
    }
    std::terminate();
  }
  catch(...){
    try{
      RCLCPP_FATAL(
        this->get_logger(),
        "failed to join navigation execution worker during teardown: unknown exception; terminating instead of releasing active dependencies");
    }
    catch(...){
    }
    std::terminate();
  }
  {
    std::lock_guard<std::mutex> lock(current_handle_mutex_);
    current_handle_.reset();
  }
  FSM_.reset();
  // TransformListener keeps a buffer reference and owns a dedicated thread.
  // Stop the listener before releasing the buffer it references.
  tfl_.reset();
  tf2Buffer_.reset();
  route_controller_.reset();
  route_manager_.reset();
}

bool P2PMoveBase::isQuaternionValid(const geometry_msgs::msg::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    RCLCPP_ERROR(this->get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

void P2PMoveBase::publishZeroVelocity(){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  if(FSM_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = FSM_->twist_frame_id_;
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void P2PMoveBase::publishVelocity(double vx, double vy, double angular_z){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = angular_z;
  if(FSM_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = FSM_->twist_frame_id_;
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void P2PMoveBase::publishVelocity(const geometry_msgs::msg::Twist & cmd_vel)
{
  publishVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
}

bool P2PMoveBase::syncRouteReferenceFromManager(const std::string & consumer_label)
{
  if(!route_manager_ || !route_controller_ || !route_manager_->hasRoute()){
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> route;
  std::size_t route_version = 0;
  std::size_t goal_seq = 0;
  std::string source_label;
  std::string route_result_class = "unknown";
  route_manager_->copyRoute(
    route,
    &route_version,
    &goal_seq,
    &source_label,
    &route_result_class);
  if(route.empty()){
    RCLCPP_WARN(
      this->get_logger(),
      "route manager reported an available route but returned an empty path (%s)",
      consumer_label.c_str());
    return false;
  }

  current_route_result_class_ = route_result_class;
  route_controller_->setRoute(route, route_version, goal_seq, source_label);
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *clock_, 2000,
    "route reference synced (%s), route_version=%zu, goal_seq=%zu, source=%s, result_class=%s, poses=%zu",
    consumer_label.c_str(),
    route_version,
    goal_seq,
    source_label.c_str(),
    route_result_class.c_str(),
    route.size());
  return true;
}

void P2PMoveBase::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
  auto move_base_goal = goal_handle->get_goal();

  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    RCLCPP_WARN(this->get_logger(),"Aborting on goal because it was sent with an invalid quaternion");
    goal_handle->abort(result);
    publishZeroVelocity();
    return;
  }

  if(shouldStopExecution()){
    abortGoalBestEffort(goal_handle, "navigation stopped before execution");
    publishZeroVelocityBestEffort();
    return;
  }

  rclcpp::Rate r(FSM_->orchestrator_frequency_);

  //@ if we dont initialize oscillation pose here, the first controlling entry will cause recovery behavior.
  //@ the rclcpp::Time initial are all done in FSM class
  const auto goal_started_at = clock_->now();
  FSM_->initialParams(route_controller_->getGlobalPose(), goal_started_at);
  oscillation_watchdog_.reset(goal_started_at.seconds());
  blocked_wait_watchdog_.reset(goal_started_at.seconds());
  FSM_->current_goal_ = move_base_goal->target_pose;
  route_manager_->setGoal(FSM_->current_goal_);
  route_manager_->resume();
  terminal_goal_reason_ = "goal failed";
  current_route_result_class_ = "unknown";

  while(!shouldStopExecution()){

    if(!goal_handle->is_active()){
      
      if(FSM_->isPhase(FSM::NavigationPhase::kRecoveryAction)){
        RCLCPP_DEBUG(this->get_logger(), "navigation is in recovery action state, cancel recovery behaviors.");
        cancelRecoveryBehaviorsBestEffort();
      }

      RCLCPP_DEBUG(this->get_logger(), "P2P move base preempted.");
      publishZeroVelocity();
      terminal_goal_reason_ = "goal preempted";
      route_manager_->stop("goal preempted");
      return;
    }

    if(goal_handle->is_canceling()){

      if(FSM_->isPhase(FSM::NavigationPhase::kRecoveryAction)){
        RCLCPP_DEBUG(this->get_logger(), "navigation is in recovery action state, cancel recovery behaviors.");
        cancelRecoveryBehaviorsBestEffort();
      }

      goal_handle->canceled(result);
      RCLCPP_DEBUG(this->get_logger(), "P2P move base cancelled.");
      publishZeroVelocity();
      terminal_goal_reason_ = "goal canceled";
      route_manager_->stop("goal canceled");
      return;
    }

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal_handle);

    // A completed goal must stop its route and leave before any feedback is
    // attempted; a newly accepted goal may already have terminated this handle.
    if(done){
      route_manager_->stop(terminal_goal_reason_);
      return;
    }

    // The ROS context or the execution token can become invalid during a cycle.
    if(shouldStopExecution()){
      break;
    }
    
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = FSM_->global_pose_;
    feedback->last_decision = FSM_->getLastDecision();
    feedback->current_decision = FSM_->getCurrentDecision();
    if(!PublishFeedbackIfActive(
        goal_handle,
        [this]() {return shouldStopExecution();},
        [&goal_handle, &feedback]() {goal_handle->publish_feedback(feedback);}))
    {
      // A preemption/cancellation/shutdown raced the guard. Continue at the
      // loop head so the existing termination cleanup owns the handle.
      if(shouldStopExecution()){
        break;
      }
      continue;
    }
    
    r.sleep();

    //if(FSM_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / FSM_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", FSM_->controller_frequency_, r.cycleTime().toSec());
  }

  if(stop_requested_.load()){
    RCLCPP_DEBUG(this->get_logger(), "navigation execution stopped by shutdown request");
  }
  else{
    RCLCPP_DEBUG(this->get_logger(), "navigation execution stopped because the ROS context is invalid");
  }
  abortGoalBestEffort(goal_handle, "navigation stopped");
  publishZeroVelocityBestEffort();
  terminal_goal_reason_ = "goal stopped";
  route_manager_->stop(terminal_goal_reason_);
}

bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){
  const auto cycle_started_at = std::chrono::steady_clock::now();
  FSM_->global_pose_ = route_controller_->getGlobalPose();

  const auto record_cycle_budget =
    [&]() {
      RuntimeBudgetSample sample;
      sample.local_cycle_time_ms =
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - cycle_started_at).count();
      const auto status = runtime_budget_monitor_.update(sample);
      if(status.budget_exceeded){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "runtime budget: level=%s, consecutive_overruns=%zu, reason=%s",
          p2p_move_base::toString(status.level),
          status.consecutive_overruns,
          status.reason.c_str());
      }
      if(runtime_budget_safety_stop_enabled_ && status.level == BudgetLevel::SafetyStop){
        publishZeroVelocity();
      }
    };

  const auto continue_cycle =
    [&]() -> bool {
      record_cycle_budget();
      return false;
    };

  const auto abort_goal_with_reason =
    [&](const std::string & reason) -> bool {
      terminal_goal_reason_ = "goal failed";
      auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
      goal_handle->abort(result);
      publishZeroVelocity();
      RCLCPP_ERROR(this->get_logger(), "%s", reason.c_str());
      return true;
    };

  const auto start_recovery_or_route_refresh =
    [&](const std::string & reason) {
      publishZeroVelocity();
      if(startRecoveryAction(reason)){
        FSM_->setPhase(FSM::NavigationPhase::kRecoveryAction, reason);
      }
      else{
        FSM_->last_valid_plan_ = clock_->now();
        FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, reason + ", fallback to route refresh");
      }
      return continue_cycle();
    };

  const auto route_refresh =
    [&](const std::string & reason) {
      publishZeroVelocity();
      FSM_->last_valid_plan_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, reason);
      return continue_cycle();
    };

  const auto start_recovery_or_abort =
    [&](const std::string & reason) -> bool {
      publishZeroVelocity();
      if(startRecoveryAction(reason)){
        FSM_->setPhase(FSM::NavigationPhase::kRecoveryAction, reason);
        return continue_cycle();
      }
      return abort_goal_with_reason(reason);
    };

  const auto wait_for_clearance =
    [&](const std::string & reason) {
      publishZeroVelocity();
      const auto wait_started_at = clock_->now();
      FSM_->waiting_time_ = wait_started_at;
      blocked_wait_watchdog_.reset(wait_started_at.seconds());
      FSM_->setPhase(FSM::NavigationPhase::kBlockedWait, reason);
      return continue_cycle();
    };

  const auto reset_oscillation_watchdog =
    [&]() {
      const auto reset_at = clock_->now();
      FSM_->oscillation_pose_ = FSM_->global_pose_;
      FSM_->last_oscillation_reset_ = reset_at;
      oscillation_watchdog_.reset(reset_at.seconds());
    };

  const auto is_data_availability_fault =
    [](dddmr_sys_core::PlannerState planner_state) {
      return planner_state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION ||
             planner_state == dddmr_sys_core::PlannerState::TF_FAIL;
    };

  const bool update_oscillation_progress =
    FSM_->isPhase(FSM::NavigationPhase::kRouteTracking) ||
    FSM_->isPhase(FSM::NavigationPhase::kGoalAlignment) ||
    FSM_->isPhase(FSM::NavigationPhase::kBlockedWait);
  if(update_oscillation_progress &&
     (FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_distance_ ||
      FSM_->getAngle(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_angle_))
  {
    reset_oscillation_watchdog();
  }

  const auto handle_tracking_control_fault =
    [&](dddmr_sys_core::PlannerState planner_state,
        const std::string & phase_label,
        bool allow_blocked_wait) -> bool {
      if(planner_state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 5000,
          "%s paused because perception data is out of date",
          phase_label.c_str());
        publishZeroVelocity();
        return continue_cycle();
      }
      if(planner_state == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 5000,
          "%s paused because TF is out of date",
          phase_label.c_str());
        publishZeroVelocity();
        return continue_cycle();
      }

      const double now_sec = clock_->now().seconds();
      const double last_replan_sec = FSM_->last_valid_plan_.seconds();
      const double controller_failure_elapsed =
        (clock_->now() - FSM_->last_valid_control_).seconds();
      const auto decision = navigation_fault_policy_.update(
        planner_state,
        allow_blocked_wait,
        now_sec,
        last_replan_sec,
        controller_failure_elapsed);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "%s fault policy: action=%s, reason=%s, cycles=%zu, duration=%.2f, message=%s",
        phase_label.c_str(),
        p2p_move_base::toString(decision.action),
        p2p_move_base::toString(decision.reason),
        decision.consecutive_cycles,
        decision.duration_sec,
        decision.message.c_str());

      if(decision.action == FaultAction::TemporaryStop){
        publishZeroVelocity();
        return continue_cycle();
      }
      if(decision.action == FaultAction::WaitForClearance){
        return wait_for_clearance(decision.message);
      }
      if(decision.action == FaultAction::RouteRefresh){
        return route_refresh(decision.message);
      }
      if(decision.action == FaultAction::RecoveryOrRouteRefresh){
        return start_recovery_or_route_refresh(decision.message);
      }
      if(decision.action == FaultAction::FatalAbort){
        return abort_goal_with_reason(decision.message);
      }
      publishZeroVelocity();
      return continue_cycle();
    };

  const auto handle_goal_alignment_control_fault =
    [&](dddmr_sys_core::PlannerState planner_state) -> bool {
      if(planner_state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "goal_alignment paused because perception data is out of date");
        publishZeroVelocity();
        return continue_cycle();
      }
      if(planner_state == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "goal_alignment paused because TF is out of date");
        publishZeroVelocity();
        return continue_cycle();
      }
      if(planner_state == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
         planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
         planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING ||
         planner_state == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL)
      {
        if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
          return start_recovery_or_abort("goal_alignment_unsatisfied: controller stalled");
        }
        publishZeroVelocity();
        return continue_cycle();
      }
      RCLCPP_FATAL(this->get_logger(), "unhandled PlannerState in goal_alignment");
      publishZeroVelocity();
      return continue_cycle();
    };

  if(FSM_->isPhase(FSM::NavigationPhase::kIdle)){
    FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, "start goal orchestration");
    return continue_cycle();
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRouteRequest)){
    route_manager_->queryThread();
    FSM_->setPhase(FSM::NavigationPhase::kRoutePending, "route request sent");
    return continue_cycle();
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRoutePending)){
    if(syncRouteReferenceFromManager("route_pending")){
      FSM_->last_valid_plan_ = clock_->now();
      FSM_->last_valid_control_ = clock_->now();
      reset_oscillation_watchdog();
      FSM_->setPhase(FSM::NavigationPhase::kRouteTracking, "route received");
      return continue_cycle();
    }

    if((clock_->now() - FSM_->last_valid_plan_).seconds() > FSM_->route_request_patience_){
      RCLCPP_WARN(
        this->get_logger(),
        "route request timed out for goal (%.2f, %.2f, %.2f)",
        FSM_->current_goal_.pose.position.x,
        FSM_->current_goal_.pose.position.y,
        FSM_->current_goal_.pose.position.z);
      route_manager_->cancelActiveRouteRequest("planner request timed out on client side");
      return start_recovery_or_abort("route_request_failed: route request timeout");
    }
    return continue_cycle();
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kGoalAlignment)){
    if(route_controller_->isGoalHeadingSatisfied()){
      terminal_goal_reason_ = "goal succeeded";
      RCLCPP_INFO(this->get_logger(), "goal succeeded");
      auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
      goal_handle->succeed(result);
      publishZeroVelocity();
      return true;
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(alignment_controller_name_, &cmd_vel);
    const auto oscillation_update = oscillation_watchdog_.update(
      is_data_availability_fault(planner_state), clock_->now().seconds());
    if(oscillation_update.resumed){
      reset_oscillation_watchdog();
      FSM_->last_valid_control_ = clock_->now();
      navigation_fault_policy_.reset();
    }
    if(oscillation_update.expired){
      const auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "oscillation timeout during goal alignment: %.2f secs for %.2f m",
        diff,
        FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
      return start_recovery_or_abort("goal_alignment_unsatisfied: oscillation timeout");
    }
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      publishVelocity(cmd_vel);
      return continue_cycle();
    }
    return handle_goal_alignment_control_fault(planner_state);
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRouteTracking)){
    if(route_controller_->isGoalPositionReached()){
      publishZeroVelocity();
      RCLCPP_DEBUG(this->get_logger(), "goal position tolerance reached, entering goal alignment");
      reset_oscillation_watchdog();
      FSM_->setPhase(FSM::NavigationPhase::kGoalAlignment, "goal position reached");
      return continue_cycle();
    }

    if(!syncRouteReferenceFromManager("route_tracking") && !route_manager_->hasRoute()){
      return route_refresh("route_invalid: frozen route unavailable during tracking");
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(tracking_controller_name_, &cmd_vel);
    const auto oscillation_update = oscillation_watchdog_.update(
      is_data_availability_fault(planner_state), clock_->now().seconds());
    if(oscillation_update.resumed){
      reset_oscillation_watchdog();
      FSM_->last_valid_control_ = clock_->now();
      navigation_fault_policy_.reset();
    }
    if(oscillation_update.expired){
      const auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "oscillation timeout during route tracking: %.2f secs for %.2f m",
        diff,
        FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
      return start_recovery_or_route_refresh("route tracking oscillation timeout");
    }
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      publishVelocity(cmd_vel);
      return continue_cycle();
    }
    return handle_tracking_control_fault(planner_state, "route tracking", true);
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRecoveryAction)){
    if(is_recoverying_){
      return continue_cycle();
    }

    if(FSM_->recovery_attempt_count_ >= FSM_->max_recovery_attempts_){
      terminal_goal_reason_ = "goal failed";
      RCLCPP_ERROR(
        this->get_logger(),
        "goal failed after %d recovery attempts",
        FSM_->recovery_attempt_count_);
      auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
      goal_handle->abort(result);
      publishZeroVelocity();
      return true;
    }

    if(is_recoverying_succeed_){
      RCLCPP_DEBUG(this->get_logger(), "recovery action completed, request a fresh route");
      FSM_->recovery_attempt_count_++;
      FSM_->last_valid_plan_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, "recovery action completed");
      return continue_cycle();
    }

    terminal_goal_reason_ = "recovery failed";
    RCLCPP_ERROR(this->get_logger(), "recovery failed");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    goal_handle->abort(result);
    publishZeroVelocity();
    return true;
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kBlockedWait)){
    if(!syncRouteReferenceFromManager("blocked_wait") && !route_manager_->hasRoute()){
      return route_refresh("route_invalid: frozen route unavailable during blocked wait");
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(tracking_controller_name_, &cmd_vel);
    const auto wait_update = blocked_wait_watchdog_.update(
      is_data_availability_fault(planner_state), clock_->now().seconds());
    if(wait_update.resumed){
      FSM_->waiting_time_ = clock_->now();
      FSM_->last_valid_control_ = clock_->now();
      navigation_fault_policy_.reset();
    }
    if(wait_update.expired){
      RCLCPP_WARN(
        this->get_logger(),
        "blocked wait exceeded %.2f seconds, request a fresh route",
        FSM_->blocked_wait_patience_);
      return route_refresh("route_blocked: blocked wait timeout");
    }

    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      reset_oscillation_watchdog();
      blocked_wait_watchdog_.reset(clock_->now().seconds());
      FSM_->setPhase(FSM::NavigationPhase::kRouteTracking, "blocked wait cleared");
      publishVelocity(cmd_vel);
      return continue_cycle();
    }
    if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
      publishZeroVelocity();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "route remains blocked, keep waiting");
      return continue_cycle();
    }
    return handle_tracking_control_fault(planner_state, "blocked wait", false);
  }

  return continue_cycle();
}

bool P2PMoveBase::startRecoveryAction(const std::string & trigger_reason){

  if(recovery_action_name_.empty()){
    RCLCPP_WARN(
      this->get_logger(),
      "skip recovery action because it is disabled (%s)",
      trigger_reason.c_str());
    is_recoverying_ = false;
    is_recoverying_succeed_ = false;
    const auto reset_at = clock_->now();
    FSM_->oscillation_pose_ = FSM_->global_pose_;
    FSM_->last_oscillation_reset_ = reset_at;
    oscillation_watchdog_.reset(reset_at.seconds());
    publishZeroVelocity();
    return false;
  }

  auto goal_msg = dddmr_sys_core::action::RecoveryBehaviors::Goal();
  goal_msg.behavior_name = recovery_action_name_;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_result_callback, this, std::placeholders::_1);
  
  is_recoverying_ = true;
  is_recoverying_succeed_ = false;
  if(!rclcpp::ok() || !recovery_behaviors_client_ptr_){
    RCLCPP_DEBUG(
      this->get_logger(),
      "skip recovery action '%s' because the ROS context is invalid",
      recovery_action_name_.c_str());
    is_recoverying_ = false;
    publishZeroVelocityBestEffort();
    return false;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "start recovery action '%s' because %s",
    recovery_action_name_.c_str(),
    trigger_reason.c_str());
  try{
    recovery_behaviors_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
  catch(...){
    is_recoverying_ = false;
    is_recoverying_succeed_ = false;
    throw;
  }
  return true;
}

void P2PMoveBase::recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "recovery action request was rejected by recovery behaviors server");
    is_recoverying_ = false;
    is_recoverying_succeed_ = false;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "recovery action accepted by recovery behaviors server, waiting for result");
  }
}

void P2PMoveBase::recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result)
{
  is_recoverying_succeed_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      is_recoverying_succeed_ = true;
      RCLCPP_DEBUG(this->get_logger(), "recovery action finished successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "recovery action was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "recovery action was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "recovery action returned unknown result code");
      break;
  }
  
  is_recoverying_ = false;
}

}//end of name space
