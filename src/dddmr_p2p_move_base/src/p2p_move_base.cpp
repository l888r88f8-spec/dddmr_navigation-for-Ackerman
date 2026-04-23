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

namespace p2p_move_base
{

P2PMoveBase::P2PMoveBase(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  is_recoverying_ = false;
  is_recoverying_succeed_ = false;
  terminal_goal_reason_ = "goal failed";
  current_route_has_entry_connector_ = true;
  current_route_result_class_ = "unknown";
}

rclcpp_action::GoalResponse P2PMoveBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal)
{
  (void)uuid;
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
  if (is_active(current_handle_)){
    RCLCPP_DEBUG(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    current_handle_->abort(result);
    return;
  }
  else{
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&P2PMoveBase::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void P2PMoveBase::initial(
  const std::shared_ptr<dddmr_sys_core::RouteTrackingController>& route_controller,
  const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& route_manager){
  
  route_controller_ = route_controller;
  route_manager_ = route_manager;

  FSM_ = std::make_shared<p2p_move_base::FSM>(this->get_node_logging_interface(), this->get_node_parameters_interface());
  
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

P2PMoveBase::~P2PMoveBase(){
  FSM_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
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
  bool has_entry_connector = true;
  std::string route_result_class = "unknown";
  route_manager_->copyRoute(
    route,
    &route_version,
    &goal_seq,
    &source_label,
    &has_entry_connector,
    &route_result_class);
  if(route.empty()){
    RCLCPP_WARN(
      this->get_logger(),
      "route manager reported an available route but returned an empty path (%s)",
      consumer_label.c_str());
    return false;
  }

  current_route_has_entry_connector_ = has_entry_connector;
  current_route_result_class_ = route_result_class;
  route_controller_->setRoute(route, route_version, goal_seq, source_label);
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *clock_, 2000,
    "route reference synced (%s), route_version=%zu, goal_seq=%zu, source=%s, result_class=%s, has_entry_connector=%d, poses=%zu",
    consumer_label.c_str(),
    route_version,
    goal_seq,
    source_label.c_str(),
    route_result_class.c_str(),
    has_entry_connector ? 1 : 0,
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

  rclcpp::Rate r(FSM_->orchestrator_frequency_);

  //@ if we dont initialize oscillation pose here, the first controlling entry will cause recovery behavior.
  //@ the rclcpp::Time initial are all done in FSM class
  FSM_->initialParams(route_controller_->getGlobalPose(), clock_->now());
  FSM_->current_goal_ = move_base_goal->target_pose;
  route_manager_->setGoal(FSM_->current_goal_);
  route_manager_->resume();
  terminal_goal_reason_ = "goal failed";
  current_route_has_entry_connector_ = true;
  current_route_result_class_ = "unknown";

  while(rclcpp::ok()){

    if(!goal_handle->is_active()){
      
      if(FSM_->isPhase(FSM::NavigationPhase::kRecoveryAction)){
        RCLCPP_DEBUG(this->get_logger(), "navigation is in recovery action state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
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
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
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
    
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = FSM_->global_pose_;
    feedback->last_decision = FSM_->getLastDecision();
    feedback->current_decision = FSM_->getCurrentDecision();
    goal_handle->publish_feedback(feedback);

    //if we're done, then we'll return from execute
    if(done){
      route_manager_->stop(terminal_goal_reason_);
      return;
    }
    
    r.sleep();

    //if(FSM_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / FSM_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", FSM_->controller_frequency_, r.cycleTime().toSec());
  }
  route_manager_->stop(terminal_goal_reason_);
}

bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){
  FSM_->global_pose_ = route_controller_->getGlobalPose();
  if(FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_distance_ ||
     FSM_->getAngle(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_angle_)
  {
    FSM_->oscillation_pose_ = FSM_->global_pose_;
    FSM_->last_oscillation_reset_ = clock_->now();
  }

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
      return false;
    };

  const auto route_refresh =
    [&](const std::string & reason) {
      publishZeroVelocity();
      FSM_->last_valid_plan_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, reason);
      return false;
    };

  const auto start_recovery_or_abort =
    [&](const std::string & reason) -> bool {
      publishZeroVelocity();
      if(startRecoveryAction(reason)){
        FSM_->setPhase(FSM::NavigationPhase::kRecoveryAction, reason);
        return false;
      }
      return abort_goal_with_reason(reason);
    };

  const auto wait_for_clearance =
    [&](const std::string & reason) {
      publishZeroVelocity();
      FSM_->waiting_time_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kBlockedWait, reason);
      return false;
    };

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
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 5000,
          "%s paused because TF is out of date",
          phase_label.c_str());
        publishZeroVelocity();
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        return route_refresh("route_deviation: local reference prune failed");
      }
      if(planner_state == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
          return start_recovery_or_route_refresh(
            "tracking_controller_stalled: controller stalled");
        }
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "%s: controller temporarily failed, keep current frozen route",
          phase_label.c_str());
        publishZeroVelocity();
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
        return route_refresh("route_blocked: route blocked, refresh route");
      }
      if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT && allow_blocked_wait){
        return wait_for_clearance("route_blocked: route blocked, waiting");
      }
      if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
        return route_refresh("route_blocked: route blocked, refresh route");
      }
      RCLCPP_FATAL(this->get_logger(), "unhandled PlannerState in %s", phase_label.c_str());
      publishZeroVelocity();
      return false;
    };

  const auto handle_startup_control_fault =
    [&](dddmr_sys_core::PlannerState planner_state) -> bool {
      if(planner_state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "route_start_alignment paused because perception data is out of date");
        publishZeroVelocity();
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "route_start_alignment paused because TF is out of date");
        publishZeroVelocity();
        return false;
      }

      const bool patience_exceeded =
        (clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_;
      if(planner_state == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        if(patience_exceeded){
          const std::string failure_type =
            current_route_has_entry_connector_ ?
            "startup_front_unreachable" :
            "startup_connector_missing";
          return start_recovery_or_abort(
            failure_type + ": local reference prune failed during startup");
        }
        publishZeroVelocity();
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
         planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING)
      {
        if(patience_exceeded){
          return start_recovery_or_abort(
            "startup_front_unreachable: startup path blocked continuously");
        }
        publishZeroVelocity();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "startup path is currently blocked, keep frozen route and wait");
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        if(patience_exceeded){
          return start_recovery_or_abort(
            "startup_alignment_unsatisfied: alignment controller stalled");
        }
        publishZeroVelocity();
        return false;
      }

      RCLCPP_FATAL(this->get_logger(), "unhandled PlannerState in route_start_alignment");
      publishZeroVelocity();
      return false;
    };

  const auto handle_goal_alignment_control_fault =
    [&](dddmr_sys_core::PlannerState planner_state) -> bool {
      if(planner_state == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "goal_alignment paused because perception data is out of date");
        publishZeroVelocity();
        return false;
      }
      if(planner_state == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 2000,
          "goal_alignment paused because TF is out of date");
        publishZeroVelocity();
        return false;
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
        return false;
      }
      RCLCPP_FATAL(this->get_logger(), "unhandled PlannerState in goal_alignment");
      publishZeroVelocity();
      return false;
    };

  if(FSM_->isPhase(FSM::NavigationPhase::kIdle)){
    FSM_->setPhase(FSM::NavigationPhase::kRouteRequest, "start goal orchestration");
    return false;
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRouteRequest)){
    route_manager_->queryThread();
    FSM_->setPhase(FSM::NavigationPhase::kRoutePending, "route request sent");
    return false;
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRoutePending)){
    if(syncRouteReferenceFromManager("route_pending")){
      FSM_->last_valid_plan_ = clock_->now();
      FSM_->last_valid_control_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kRouteStartAlignment, "route received");
      return false;
    }

    if((clock_->now() - FSM_->last_valid_plan_).seconds() > FSM_->route_request_patience_){
      RCLCPP_WARN(
        this->get_logger(),
        "route request timed out for goal (%.2f, %.2f, %.2f)",
        FSM_->current_goal_.pose.position.x,
        FSM_->current_goal_.pose.position.y,
        FSM_->current_goal_.pose.position.z);
      route_manager_->cancelActiveRouteRequest("planner request timed out on client side");
      return start_recovery_or_abort("startup_alignment_unsatisfied: route request timeout");
    }
    return false;
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRouteStartAlignment)){
    std::string startup_detail;
    const auto startup_status = route_controller_->evaluateRouteStartupStatus(&startup_detail);
    if(startup_status == dddmr_sys_core::RouteStartupStatus::kAligned){
      FSM_->setPhase(FSM::NavigationPhase::kRouteTracking, "route start alignment satisfied");
      return false;
    }

    if(startup_status == dddmr_sys_core::RouteStartupStatus::kFrontUnreachable){
      const std::string startup_failure_class =
        current_route_has_entry_connector_ ?
        "startup_front_unreachable" :
        "startup_connector_missing";
      const std::string reason = startup_detail.empty() ?
        startup_failure_class :
        startup_failure_class + ": " + startup_detail;
      return start_recovery_or_abort(reason);
    }

    if(FSM_->oscillation_patience_ > 0 &&
       (clock_->now() - FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_)
    {
      const auto diff = (clock_->now() - FSM_->last_oscillation_reset_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "oscillation timeout during route start alignment: %.2f secs for %.2f m",
        diff,
        FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
      return start_recovery_or_abort("startup_alignment_unsatisfied: oscillation timeout");
    }

    if(startup_status == dddmr_sys_core::RouteStartupStatus::kRouteUnavailable){
      if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
        const std::string reason = startup_detail.empty() ?
          "startup_alignment_unsatisfied: local reference unavailable" :
          "startup_alignment_unsatisfied: " + startup_detail;
        return start_recovery_or_abort(reason);
      }
      publishZeroVelocity();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "route_start_alignment waiting for local reference: %s",
        startup_detail.empty() ? "unknown" : startup_detail.c_str());
      return false;
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(alignment_controller_name_, &cmd_vel);
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      publishVelocity(cmd_vel);
      return false;
    }
    return handle_startup_control_fault(planner_state);
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

    if(FSM_->oscillation_patience_ > 0 &&
       (clock_->now()-FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_)
    {
      const auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "oscillation timeout during goal alignment: %.2f secs for %.2f m",
        diff,
        FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
      return start_recovery_or_abort("goal_alignment_unsatisfied: oscillation timeout");
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(alignment_controller_name_, &cmd_vel);
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      publishVelocity(cmd_vel);
      return false;
    }
    return handle_goal_alignment_control_fault(planner_state);
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRouteTracking)){
    if(route_controller_->isGoalPositionReached()){
      publishZeroVelocity();
      RCLCPP_DEBUG(this->get_logger(), "goal position tolerance reached, entering goal alignment");
      FSM_->setPhase(FSM::NavigationPhase::kGoalAlignment, "goal position reached");
      return false;
    }

    if(!syncRouteReferenceFromManager("route_tracking") && !route_manager_->hasRoute()){
      return route_refresh("route_invalid: frozen route unavailable during tracking");
    }

    if(FSM_->oscillation_patience_ > 0 &&
       (clock_->now()-FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_)
    {
      const auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
      RCLCPP_WARN(
        this->get_logger(),
        "oscillation timeout during route tracking: %.2f secs for %.2f m",
        diff,
        FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
      return start_recovery_or_route_refresh("route tracking oscillation timeout");
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(tracking_controller_name_, &cmd_vel);
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      publishVelocity(cmd_vel);
      return false;
    }
    return handle_tracking_control_fault(planner_state, "route tracking", true);
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kRecoveryAction)){
    if(is_recoverying_){
      return false;
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
      return false;
    }

    terminal_goal_reason_ = "recovery failed";
    RCLCPP_ERROR(this->get_logger(), "recovery failed");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    goal_handle->abort(result);
    publishZeroVelocity();
    return true;
  }

  if(FSM_->isPhase(FSM::NavigationPhase::kBlockedWait)){
    if((clock_->now()-FSM_->waiting_time_).seconds() >= FSM_->blocked_wait_patience_){
      RCLCPP_WARN(
        this->get_logger(),
        "blocked wait exceeded %.2f seconds, request a fresh route",
        FSM_->blocked_wait_patience_);
      return route_refresh("route_blocked: blocked wait timeout");
    }

    if(!syncRouteReferenceFromManager("blocked_wait") && !route_manager_->hasRoute()){
      return route_refresh("route_invalid: frozen route unavailable during blocked wait");
    }

    geometry_msgs::msg::Twist cmd_vel;
    const auto planner_state =
      route_controller_->computeControlCommand(tracking_controller_name_, &cmd_vel);
    if(planner_state == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
      FSM_->last_valid_control_ = clock_->now();
      FSM_->setPhase(FSM::NavigationPhase::kRouteTracking, "blocked wait cleared");
      publishVelocity(cmd_vel);
      return false;
    }
    if(planner_state == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
      publishZeroVelocity();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "route remains blocked, keep waiting");
      return false;
    }
    return handle_tracking_control_fault(planner_state, "blocked wait", false);
  }

  return false;
}

bool P2PMoveBase::startRecoveryAction(const std::string & trigger_reason){

  if(recovery_action_name_.empty()){
    RCLCPP_WARN(
      this->get_logger(),
      "skip recovery action because it is disabled (%s)",
      trigger_reason.c_str());
    is_recoverying_ = false;
    is_recoverying_succeed_ = false;
    FSM_->oscillation_pose_ = FSM_->global_pose_;
    FSM_->last_oscillation_reset_ = clock_->now();
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
  RCLCPP_WARN(
    this->get_logger(),
    "start recovery action '%s' because %s",
    recovery_action_name_.c_str(),
    trigger_reason.c_str());
  recovery_behaviors_client_ptr_->async_send_goal(goal_msg, send_goal_options);
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
