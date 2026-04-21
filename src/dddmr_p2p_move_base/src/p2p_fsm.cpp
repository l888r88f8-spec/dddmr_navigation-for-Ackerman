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
#include "p2p_move_base/p2p_fsm.h"

namespace p2p_move_base
{

FSM::FSM(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter)
{
  logger_ = m_logger;
  parameter_ = m_parameter;
  current_phase_ = NavigationPhase::kIdle;
  last_phase_ = NavigationPhase::kIdle;

  // Goal orchestration / route management related.
  parameter_->declare_parameter("route_request_patience", rclcpp::ParameterValue(-1.0));
  rclcpp::Parameter route_request_patience = parameter_->get_parameter("route_request_patience");
  parameter_->declare_parameter("planner_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter planner_patience = parameter_->get_parameter("planner_patience");
  route_request_patience_ =
    route_request_patience.as_double() >= 0.0 ?
    route_request_patience.as_double() :
    planner_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "route_request_patience: %.2f", route_request_patience_);  

  //Controlling related
  parameter_->declare_parameter("oscillation_distance", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter oscillation_distance = parameter_->get_parameter("oscillation_distance");
  oscillation_distance_ = oscillation_distance.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_distance: %.2f", oscillation_distance_);  

  parameter_->declare_parameter("oscillation_angle", rclcpp::ParameterValue(0.5));
  rclcpp::Parameter oscillation_angle = parameter_->get_parameter("oscillation_angle");
  oscillation_angle_ = oscillation_angle.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_angle: %.2f", oscillation_angle_);  

  parameter_->declare_parameter("oscillation_patience", rclcpp::ParameterValue(20.0));
  rclcpp::Parameter oscillation_patience = parameter_->get_parameter("oscillation_patience");
  oscillation_patience_ = oscillation_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_patience: %.2f", oscillation_patience_);  

  parameter_->declare_parameter("control_failure_patience", rclcpp::ParameterValue(-1.0));
  rclcpp::Parameter control_failure_patience = parameter_->get_parameter("control_failure_patience");
  parameter_->declare_parameter("controller_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter controller_patience = parameter_->get_parameter("controller_patience");
  controller_patience_ =
    control_failure_patience.as_double() >= 0.0 ?
    control_failure_patience.as_double() :
    controller_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "control_failure_patience: %.2f", controller_patience_);  

  parameter_->declare_parameter("max_recovery_attempts", rclcpp::ParameterValue(-1));
  rclcpp::Parameter max_recovery_attempts = parameter_->get_parameter("max_recovery_attempts");
  parameter_->declare_parameter("no_plan_retry_num", rclcpp::ParameterValue(10));
  rclcpp::Parameter no_plan_retry_num = parameter_->get_parameter("no_plan_retry_num");
  max_recovery_attempts_ =
    max_recovery_attempts.as_int() >= 0 ?
    max_recovery_attempts.as_int() :
    no_plan_retry_num.as_int();
  RCLCPP_INFO(logger_->get_logger(), "max_recovery_attempts: %d", max_recovery_attempts_);  

  parameter_->declare_parameter("blocked_wait_patience", rclcpp::ParameterValue(-1.0));
  rclcpp::Parameter blocked_wait_patience = parameter_->get_parameter("blocked_wait_patience");
  parameter_->declare_parameter("waiting_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter waiting_patience = parameter_->get_parameter("waiting_patience");
  blocked_wait_patience_ =
    blocked_wait_patience.as_double() >= 0.0 ?
    blocked_wait_patience.as_double() :
    waiting_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "blocked_wait_patience: %.2f", blocked_wait_patience_); 

  parameter_->declare_parameter("orchestrator_frequency", rclcpp::ParameterValue(-1.0));
  rclcpp::Parameter orchestrator_frequency = parameter_->get_parameter("orchestrator_frequency");
  parameter_->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  rclcpp::Parameter controller_frequency = parameter_->get_parameter("controller_frequency");
  orchestrator_frequency_ =
    orchestrator_frequency.as_double() > 0.0 ?
    orchestrator_frequency.as_double() :
    controller_frequency.as_double();
  RCLCPP_INFO(logger_->get_logger(), "orchestrator_frequency: %.2f", orchestrator_frequency_); 

  parameter_->declare_parameter("use_twist_stamped", rclcpp::ParameterValue(false));
  rclcpp::Parameter use_twist_stamped = parameter_->get_parameter("use_twist_stamped");
  use_twist_stamped_ = use_twist_stamped.as_bool();
  RCLCPP_INFO(logger_->get_logger(), "use_stamped_twist: %d", use_twist_stamped_); 

  parameter_->declare_parameter("twist_frame_id", rclcpp::ParameterValue("base_link"));
  rclcpp::Parameter twist_frame_id = parameter_->get_parameter("twist_frame_id");
  twist_frame_id_ = twist_frame_id.as_string();
  RCLCPP_INFO(logger_->get_logger(), "twist_frame_id: %s", twist_frame_id_.c_str()); 

}

std::string FSM::phaseName(NavigationPhase phase)
{
  switch(phase){
    case NavigationPhase::kIdle:
      return "idle";
    case NavigationPhase::kRouteRequest:
      return "route_request";
    case NavigationPhase::kRoutePending:
      return "route_pending";
    case NavigationPhase::kRouteStartAlignment:
      return "route_start_alignment";
    case NavigationPhase::kRouteTracking:
      return "route_tracking";
    case NavigationPhase::kGoalAlignment:
      return "goal_alignment";
    case NavigationPhase::kBlockedWait:
      return "blocked_wait";
    case NavigationPhase::kRecoveryAction:
      return "recovery_action";
  }
  return "unknown";
}

bool FSM::legacyDecisionToPhase(const std::string & decision, NavigationPhase * phase)
{
  if(phase == nullptr){
    return false;
  }

  if(decision == "d_initial" || decision == "idle"){
    *phase = NavigationPhase::kIdle;
    return true;
  }
  if(decision == "d_planning" || decision == "route_request"){
    *phase = NavigationPhase::kRouteRequest;
    return true;
  }
  if(decision == "d_planning_waitdone" || decision == "route_pending"){
    *phase = NavigationPhase::kRoutePending;
    return true;
  }
  if(decision == "d_align_heading" || decision == "route_start_alignment"){
    *phase = NavigationPhase::kRouteStartAlignment;
    return true;
  }
  if(decision == "d_controlling" || decision == "route_tracking"){
    *phase = NavigationPhase::kRouteTracking;
    return true;
  }
  if(decision == "d_align_goal_heading" || decision == "goal_alignment"){
    *phase = NavigationPhase::kGoalAlignment;
    return true;
  }
  if(decision == "d_waiting" || decision == "blocked_wait"){
    *phase = NavigationPhase::kBlockedWait;
    return true;
  }
  if(decision == "d_recovery_waitdone" || decision == "recovery_action"){
    *phase = NavigationPhase::kRecoveryAction;
    return true;
  }
  return false;
}

bool FSM::isPhase(NavigationPhase phase) const
{
  return current_phase_ == phase;
}

bool FSM::isCurrentDecision(std::string m_decision){
  NavigationPhase phase = NavigationPhase::kIdle;
  return legacyDecisionToPhase(m_decision, &phase) && current_phase_ == phase;
}

void FSM::initialParams(geometry_msgs::msg::TransformStamped robot_curent_pose, rclcpp::Time current_time){

  last_phase_ = NavigationPhase::kIdle;
  current_phase_ = NavigationPhase::kIdle;

  oscillation_pose_ = robot_curent_pose;
  last_oscillation_reset_ = current_time;
  last_valid_plan_ = current_time;
  last_valid_control_ = current_time;
  recovery_attempt_count_ = 0;

}


double FSM::getDistance(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b){

  double dx = a.transform.translation.x - b.transform.translation.x;
  double dy = a.transform.translation.y - b.transform.translation.y;
  double dz = a.transform.translation.z - b.transform.translation.z;

  return sqrt(dx*dx + dy*dy + dz*dz);
}

double FSM::getAngle(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b){

  tf2::Transform pose_a;
  pose_a.setRotation(tf2::Quaternion(a.transform.rotation.x, a.transform.rotation.y, a.transform.rotation.z, a.transform.rotation.w));
  pose_a.setOrigin(tf2::Vector3(a.transform.translation.x, a.transform.translation.y, a.transform.translation.z));
  
  tf2::Transform pose_b;
  pose_b.setRotation(tf2::Quaternion(b.transform.rotation.x, b.transform.rotation.y, b.transform.rotation.z, b.transform.rotation.w));
  pose_b.setOrigin(tf2::Vector3(b.transform.translation.x, b.transform.translation.y, b.transform.translation.z));
  
  auto pose_a_inverse = pose_a.inverse();

  tf2::Transform pose_a2b;
  pose_a2b.mult(pose_a_inverse, pose_b);
  /*Get RPY*/
  tf2::Matrix3x3 m(pose_a2b.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return fabs(yaw);

}

bool FSM::setPhase(NavigationPhase next_phase, const std::string & reason){
  const NavigationPhase previous_phase = current_phase_;
  last_phase_ = current_phase_;
  current_phase_ = next_phase;

  if(previous_phase != next_phase){
    if(reason.empty()){
      RCLCPP_INFO(
        logger_->get_logger(),
        "navigation phase: %s -> %s",
        phaseName(previous_phase).c_str(),
        phaseName(next_phase).c_str());
    }
    else{
      RCLCPP_INFO(
        logger_->get_logger(),
        "navigation phase: %s -> %s (%s)",
        phaseName(previous_phase).c_str(),
        phaseName(next_phase).c_str(),
        reason.c_str());
    }
  }
  return true;
}

bool FSM::setDecision(std::string m_decision){
  NavigationPhase phase = NavigationPhase::kIdle;
  if(!legacyDecisionToPhase(m_decision, &phase)){
    RCLCPP_WARN(
      logger_->get_logger(),
      "unknown legacy navigation decision: %s",
      m_decision.c_str());
    return false;
  }
  return setPhase(phase, "legacy decision alias");

}

}//end of name space
