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
#include <p2p_move_base/p2p_global_plan_manager.h>

#include <cmath>
#include <limits>
#include <sstream>

namespace p2p_move_base
{
namespace
{
bool is_failure_termination_reason(const std::string & reason)
{
  return reason == "goal failed" || reason == "recovery failed";
}

bool is_client_side_timeout_reason(const std::string & reason)
{
  return reason == "planner request timed out on client side";
}

std::string timeout_result_class_from_stage(const std::string & stage)
{
  if(stage == "entry_connector"){
    return "timed_out_during_entry_connector";
  }
  return "timed_out_during_raw_route";
}

std::string classify_aborted_planner_result(
  const P2PGlobalPlanManager::PlannerRouteRequestDiagnostics * diagnostics)
{
  if(diagnostics != nullptr){
    if(!diagnostics->result_class.empty() && diagnostics->result_class != "in_progress"){
      return diagnostics->result_class;
    }
    if(diagnostics->stage == "projecting"){
      return "failed_projection";
    }
    if(diagnostics->stage == "entry_connector"){
      return "failed_entry_connector";
    }
  }
  return "failed_raw_route";
}

bool parse_planner_route_request_diagnostics(
  const std::string & serialized,
  P2PGlobalPlanManager::PlannerRouteRequestDiagnostics * diagnostics)
{
  if(diagnostics == nullptr){
    return false;
  }

  P2PGlobalPlanManager::PlannerRouteRequestDiagnostics parsed;
  std::istringstream stream(serialized);
  std::string token;
  while(std::getline(stream, token, ';')){
    const auto separator = token.find('=');
    if(separator == std::string::npos){
      continue;
    }

    const std::string key = token.substr(0, separator);
    const std::string value = token.substr(separator + 1);
    try{
      if(key == "request_id"){
        parsed.request_id = static_cast<std::size_t>(std::stoull(value));
      }
      else if(key == "stage"){
        parsed.stage = value;
      }
      else if(key == "stage_elapsed_sec"){
        parsed.stage_elapsed_sec = std::stod(value);
      }
      else if(key == "expansions"){
        parsed.expansions = static_cast<std::size_t>(std::stoull(value));
      }
      else if(key == "result_class"){
        parsed.result_class = value;
      }
    }
    catch(const std::exception &){
      return false;
    }
  }

  if(parsed.request_id == 0 || parsed.stage.empty()){
    return false;
  }

  parsed.valid = true;
  *diagnostics = parsed;
  return true;
}
}

P2PGlobalPlanManager::P2PGlobalPlanManager(std::string name)
: Node(name),
  name_(name),
  is_planning_(false),
  got_first_goal_(false),
  freeze_route_per_goal_(true),
  route_requested_for_goal_(false),
  route_request_failed_(false),
  goal_seq_(0),
  route_version_(0),
  next_route_request_id_(0),
  active_route_request_id_(0)
{
  clock_ = this->get_clock();
}

P2PGlobalPlanManager::~P2PGlobalPlanManager(){
  tf2Buffer_.reset();
  tfl_.reset();
}

void P2PGlobalPlanManager::initial(){

  this->declare_parameter("route_action_name", rclcpp::ParameterValue(""));
  this->get_parameter("route_action_name", route_action_name_);
  this->declare_parameter("global_planner_action_name", rclcpp::ParameterValue("get_plan"));
  std::string legacy_route_action_name;
  this->get_parameter("global_planner_action_name", legacy_route_action_name);
  if(route_action_name_.empty()){
    route_action_name_ = legacy_route_action_name;
    RCLCPP_INFO(
      this->get_logger(),
      "global_planner_action_name (legacy route action alias): %s",
      route_action_name_.c_str());
  }
  else{
    RCLCPP_INFO(this->get_logger(), "route_action_name: %s", route_action_name_.c_str());
  }
  RCLCPP_INFO(
    this->get_logger(),
    "route manager uses \033[1;32m%s\033[0m action to request routes.",
    route_action_name_.c_str());

  this->declare_parameter(
    "route_request_frequency",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  this->get_parameter("route_request_frequency", route_request_frequency_);
  this->declare_parameter("global_plan_query_frequency", rclcpp::ParameterValue(5.0));
  double legacy_route_request_frequency = 5.0;
  this->get_parameter("global_plan_query_frequency", legacy_route_request_frequency);
  if(!std::isfinite(route_request_frequency_)){
    route_request_frequency_ = legacy_route_request_frequency;
    RCLCPP_INFO(
      this->get_logger(),
      "global_plan_query_frequency (legacy route request alias): %.2f",
      route_request_frequency_);
  }
  else{
    RCLCPP_INFO(this->get_logger(), "route_request_frequency: %.2f", route_request_frequency_);
  }

  this->declare_parameter("freeze_route_per_goal", rclcpp::ParameterValue(true));
  this->get_parameter("freeze_route_per_goal", freeze_route_per_goal_);
  RCLCPP_INFO(this->get_logger(), "freeze_route_per_goal: %d", freeze_route_per_goal_);

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  global_planner_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  global_planner_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::GetPlan>(
      this,
      route_action_name_, global_planner_client_group_);
  planner_route_request_diag_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/debug/planner_route_request_stage",
    rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&P2PGlobalPlanManager::planner_route_request_diag_callback, this, std::placeholders::_1));
  
  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if(route_request_frequency_>0.0){
    auto loop_time = std::chrono::milliseconds(int(1000/route_request_frequency_));
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }
  else{
    auto loop_time = std::chrono::milliseconds(1000000000);
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }

}

void P2PGlobalPlanManager::resume(){
  std::unique_lock<std::mutex> lock(access_);
  if(!freeze_route_per_goal_){
    active_route_.poses.clear();
  }
  is_planning_ = false;
  loop_timer_->reset();
  RCLCPP_INFO(this->get_logger(), "route manager resumed");
}

void P2PGlobalPlanManager::cancelActiveRouteRequest(const std::string & reason)
{
  GetPlanClientGoalHandle::SharedPtr goal_handle_to_cancel;
  std::size_t goal_seq = 0;
  std::size_t request_id = 0;
  bool has_active_request = false;
  PlannerRouteRequestDiagnostics diagnostics;
  bool has_diagnostics = false;
  std::string timeout_result_class;

  {
    std::unique_lock<std::mutex> lock(access_);
    has_active_request =
      is_planning_ || active_route_request_handle_ != nullptr || active_route_request_id_ != 0;
    if(!has_active_request){
      return;
    }

    goal_seq = goal_seq_;
    request_id = active_route_request_id_;
    goal_handle_to_cancel = active_route_request_handle_;
    active_route_request_handle_.reset();
    is_planning_ = false;
    route_requested_for_goal_ = false;
    route_request_failed_ = false;
    const auto diagnostics_it = planner_route_request_diagnostics_.find(request_id);
    if(diagnostics_it != planner_route_request_diagnostics_.end()){
      diagnostics = diagnostics_it->second;
      has_diagnostics = diagnostics.valid;
    }
    if(is_client_side_timeout_reason(reason)){
      timeout_result_class =
        timeout_result_class_from_stage(has_diagnostics ? diagnostics.stage : "raw_route");
      if(request_id != 0){
        route_request_cancel_reasons_[request_id] = reason;
        planner_route_request_diagnostics_[request_id].valid = true;
        planner_route_request_diagnostics_[request_id].request_id = request_id;
        planner_route_request_diagnostics_[request_id].stage =
          has_diagnostics ? diagnostics.stage : "unknown";
        planner_route_request_diagnostics_[request_id].stage_elapsed_sec =
          has_diagnostics ? diagnostics.stage_elapsed_sec : 0.0;
        planner_route_request_diagnostics_[request_id].expansions =
          has_diagnostics ? diagnostics.expansions : 0;
        planner_route_request_diagnostics_[request_id].result_class = timeout_result_class;
      }
    }
    if(request_id != 0){
      route_request_cancel_reasons_[request_id] = reason;
    }
  }

  if(is_client_side_timeout_reason(reason)){
    if(has_diagnostics){
      RCLCPP_WARN(
        this->get_logger(),
        "planner request timed out on client side, goal_seq=%zu, request_id=%zu, planner_stage=%s, stage_elapsed_sec=%.3f, expansions=%zu, final_result=%s; canceling planner request",
        goal_seq,
        request_id,
        diagnostics.stage.c_str(),
        diagnostics.stage_elapsed_sec,
        diagnostics.expansions,
        timeout_result_class.c_str());
    }
    else{
      RCLCPP_WARN(
        this->get_logger(),
        "planner request timed out on client side, goal_seq=%zu, request_id=%zu, planner_stage=unknown, stage_elapsed_sec=0.000, expansions=0, final_result=%s; canceling planner request",
        goal_seq,
        request_id,
        timeout_result_class.c_str());
    }
  }
  else{
    RCLCPP_INFO(
      this->get_logger(),
      "cancel planner request because %s, goal_seq=%zu, request_id=%zu",
      reason.c_str(),
      goal_seq,
      request_id);
  }

  if(goal_handle_to_cancel){
    global_planner_client_ptr_->async_cancel_goal(goal_handle_to_cancel);
  }
}

void P2PGlobalPlanManager::stop(const std::string & reason){
  cancelActiveRouteRequest(reason);

  bool should_deactivate_threading = false;
  bool should_log_release = false;
  std::size_t goal_seq = 0;
  std::size_t route_version = 0;
  {
    std::unique_lock<std::mutex> lock(access_);
    loop_timer_->cancel();
    active_route_.poses.clear();
    frozen_route_.poses.clear();
    is_planning_ = false;
    should_deactivate_threading = got_first_goal_;
    should_log_release = got_first_goal_ || route_requested_for_goal_ || route_request_failed_;
    goal_seq = goal_seq_;
    route_version = route_version_;
    got_first_goal_ = false;
    route_requested_for_goal_ = false;
    route_request_failed_ = false;
    route_source_label_.clear();
    active_route_request_handle_.reset();
    active_route_request_id_ = 0;
    route_request_cancel_reasons_.clear();
    planner_route_request_diagnostics_.clear();
  }

  if(freeze_route_per_goal_ && should_log_release){
    if(is_failure_termination_reason(reason)){
      RCLCPP_WARN(
        this->get_logger(),
        "release frozen route after %s, goal_seq=%zu, route_version=%zu",
        reason.c_str(),
        goal_seq,
        route_version);
    }
    else{
      RCLCPP_INFO(
        this->get_logger(),
        "release frozen route after %s, goal_seq=%zu, route_version=%zu",
        reason.c_str(),
        goal_seq,
        route_version);
    }
  }

  if(should_deactivate_threading){
    auto goal_msg = dddmr_sys_core::action::GetPlan::Goal();
    goal_msg.activate_threading = false;
    auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
    const auto request_goal_seq = goal_seq;
    const std::size_t request_id = 0;
    send_goal_options.goal_response_callback =
      [this, request_goal_seq, request_id](const GetPlanClientGoalHandle::SharedPtr & goal_handle)
      {
        global_planner_client_goal_response_callback(goal_handle, false, request_goal_seq, request_id);
      };
    send_goal_options.result_callback =
      [this, request_goal_seq, request_id](
        const GetPlanClientGoalHandle::WrappedResult & result)
      {
        global_planner_client_result_callback(result, false, request_goal_seq, request_id);
      };
    global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  RCLCPP_INFO(this->get_logger(), "route manager stopped");
}

void P2PGlobalPlanManager::queryThread(){
  dddmr_sys_core::action::GetPlan::Goal goal_msg;
  bool should_send_goal = false;
  std::size_t goal_seq = 0;
  std::size_t route_version = 0;
  std::size_t request_id = 0;

  {
    std::unique_lock<std::mutex> lock(access_);
    if(is_planning_ || !got_first_goal_){
      return;
    }

    if(freeze_route_per_goal_ && route_requested_for_goal_ && !frozen_route_.poses.empty()){
      goal_seq = goal_seq_;
      route_version = route_version_;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "reuse frozen route for current goal, goal_seq=%zu, route_version=%zu, poses=%zu",
        goal_seq,
        route_version,
        frozen_route_.poses.size());
      return;
    }

    goal_msg.goal = goal_;
    goal_msg.activate_threading = true;
    goal_seq = goal_seq_;
    request_id = ++next_route_request_id_;
    active_route_request_id_ = request_id;
    active_route_request_handle_.reset();
    route_request_cancel_reasons_.erase(request_id);
    planner_route_request_diagnostics_.erase(request_id);
    is_planning_ = true;
    route_requested_for_goal_ = true;
    should_send_goal = true;
  }

  if(!should_send_goal){
    return;
  }

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
  const auto request_goal_seq = goal_seq;
  
  send_goal_options.goal_response_callback =
    [this, request_goal_seq, request_id](const GetPlanClientGoalHandle::SharedPtr & goal_handle)
    {
      global_planner_client_goal_response_callback(goal_handle, true, request_goal_seq, request_id);
    };
  send_goal_options.result_callback =
    [this, request_goal_seq, request_id](
      const GetPlanClientGoalHandle::WrappedResult & result)
    {
      global_planner_client_result_callback(result, true, request_goal_seq, request_id);
    };
  
  global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void P2PGlobalPlanManager::global_planner_client_goal_response_callback(
  const GetPlanClientGoalHandle::SharedPtr & goal_handle,
  bool is_route_request,
  std::size_t request_goal_seq,
  std::size_t request_id)
{
  const auto * request_label =
    is_route_request ? "route request" : "route manager deactivation request";
  bool cancel_stale_goal = false;
  std::string cancel_reason;

  if (!goal_handle) {
    if(is_route_request){
      std::unique_lock<std::mutex> lock(access_);
      if(got_first_goal_ && request_goal_seq == goal_seq_ && request_id == active_route_request_id_){
        is_planning_ = false;
        route_requested_for_goal_ = false;
        route_request_failed_ = true;
        active_route_request_handle_.reset();
        planner_route_request_diagnostics_.erase(request_id);
      }
    }
    if(route_request_frequency_>2)
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "%s was rejected by: %s",
        request_label,
        route_action_name_.c_str());
    else
      RCLCPP_ERROR(
        this->get_logger(),
        "%s was rejected by: %s",
        request_label,
        route_action_name_.c_str());
  } else {
    if(is_route_request){
      std::unique_lock<std::mutex> lock(access_);
      const auto cancel_it = route_request_cancel_reasons_.find(request_id);
      if(cancel_it != route_request_cancel_reasons_.end()){
        cancel_stale_goal = true;
        cancel_reason = cancel_it->second;
      }
      else if(!got_first_goal_ || request_goal_seq != goal_seq_ || request_id != active_route_request_id_){
        cancel_stale_goal = true;
        cancel_reason = "request is stale on client side";
      }
      else{
        active_route_request_handle_ = goal_handle;
      }
    }

    if(cancel_stale_goal){
      RCLCPP_INFO(
        this->get_logger(),
        "planner accepted a stale route request, goal_seq=%zu, request_id=%zu; canceling because %s",
        request_goal_seq,
        request_id,
        cancel_reason.c_str());
      global_planner_client_ptr_->async_cancel_goal(goal_handle);
      return;
    }

    if(route_request_frequency_>2)
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "planner still computing %s via %s, goal_seq=%zu, request_id=%zu",
        request_label,
        route_action_name_.c_str(),
        request_goal_seq,
        request_id);
    else
      RCLCPP_INFO(
        this->get_logger(),
        "planner still computing %s via %s, goal_seq=%zu, request_id=%zu",
        request_label,
        route_action_name_.c_str(),
        request_goal_seq,
        request_id);
  }
}

void P2PGlobalPlanManager::global_planner_client_result_callback(
  const GetPlanClientGoalHandle::WrappedResult & result,
  bool is_route_request,
  std::size_t request_goal_seq,
  std::size_t request_id)
{
  if(!is_route_request){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "route manager deactivation request finished");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "route manager deactivation request was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "route manager deactivation request was canceled");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "route manager deactivation request returned unknown result code");
        break;
    }
    return;
  }

  bool freeze_current_goal = false;
  bool succeeded_with_route = false;
  std::size_t goal_seq = 0;
  std::size_t route_version = 0;
  std::size_t pose_count = 0;
  bool ignore_stale_result = false;
  std::string cancel_reason;
  PlannerRouteRequestDiagnostics diagnostics;
  bool has_diagnostics = false;

  {
    std::unique_lock<std::mutex> lock(access_);
    const auto cancel_it = route_request_cancel_reasons_.find(request_id);
    if(cancel_it != route_request_cancel_reasons_.end()){
      cancel_reason = cancel_it->second;
      route_request_cancel_reasons_.erase(cancel_it);
    }
    const auto diagnostics_it = planner_route_request_diagnostics_.find(request_id);
    if(diagnostics_it != planner_route_request_diagnostics_.end()){
      diagnostics = diagnostics_it->second;
      has_diagnostics = diagnostics.valid;
      planner_route_request_diagnostics_.erase(diagnostics_it);
    }

    if(!got_first_goal_ || request_goal_seq != goal_seq_ || request_id != active_route_request_id_){
      ignore_stale_result = true;
    }
    else{
      freeze_current_goal = freeze_route_per_goal_;
      active_route_request_handle_.reset();
      succeeded_with_route =
        result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result != nullptr &&
        !result.result->path.poses.empty();

      if(succeeded_with_route){
        active_route_ = result.result->path;
        ++route_version_;
        route_source_label_ = freeze_route_per_goal_ ? "frozen_route" : "planner_result";
        if(freeze_route_per_goal_){
          frozen_route_ = active_route_;
        }
        route_request_failed_ = false;
        pose_count = active_route_.poses.size();
      }
      else{
        active_route_.poses.clear();
        frozen_route_.poses.clear();
        route_source_label_.clear();
        route_requested_for_goal_ = false;
        route_request_failed_ = result.code == rclcpp_action::ResultCode::ABORTED;
      }

      goal_seq = goal_seq_;
      route_version = route_version_;
      is_planning_ = false;
    }
  }

  if(ignore_stale_result){
    RCLCPP_INFO(
      this->get_logger(),
      "ignore stale planner result, request_goal_seq=%zu, request_id=%zu",
      request_goal_seq,
      request_id);
    return;
  }

  std::string final_result_class =
    has_diagnostics ? diagnostics.result_class : std::string();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if(final_result_class.empty() || final_result_class == "in_progress"){
        final_result_class = "succeeded";
      }
      RCLCPP_INFO(
        this->get_logger(),
        "planner finished for goal_seq=%zu, request_id=%zu via %s, final_result=%s",
        goal_seq,
        request_id,
        route_action_name_.c_str(),
        final_result_class.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      final_result_class = classify_aborted_planner_result(
        has_diagnostics ? &diagnostics : nullptr);
      RCLCPP_ERROR(
        this->get_logger(),
        "planner failed for goal_seq=%zu, request_id=%zu via %s, final_result=%s",
        goal_seq,
        request_id,
        route_action_name_.c_str(),
        final_result_class.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      if(is_client_side_timeout_reason(cancel_reason)){
        if(final_result_class.empty() || final_result_class == "in_progress"){
          final_result_class = timeout_result_class_from_stage(
            has_diagnostics ? diagnostics.stage : "raw_route");
        }
        RCLCPP_INFO(
          this->get_logger(),
          "planner timeout cleanup completed, goal_seq=%zu, request_id=%zu, final_result=%s",
          goal_seq,
          request_id,
          final_result_class.c_str());
      }
      else if(!cancel_reason.empty()){
        final_result_class = "planner_canceled";
        RCLCPP_INFO(
          this->get_logger(),
          "planner canceled for goal_seq=%zu, request_id=%zu because %s, final_result=%s",
          goal_seq,
          request_id,
          cancel_reason.c_str(),
          final_result_class.c_str());
      }
      else{
        final_result_class = "planner_canceled";
        RCLCPP_INFO(
          this->get_logger(),
          "planner canceled for goal_seq=%zu, request_id=%zu, final_result=%s",
          goal_seq,
          request_id,
          final_result_class.c_str());
      }
      break;
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "planner returned unknown result code for goal_seq=%zu, request_id=%zu via %s",
        goal_seq,
        request_id,
        route_action_name_.c_str());
      break;
  }

  if(succeeded_with_route){
    if(freeze_current_goal){
      RCLCPP_INFO(
        this->get_logger(),
        "freeze route for current goal, goal_seq=%zu, route_version=%zu, poses=%zu",
        goal_seq,
        route_version,
        pose_count);
    }
    else{
      RCLCPP_INFO(
        this->get_logger(),
        "update route for current goal, goal_seq=%zu, route_version=%zu, poses=%zu",
        goal_seq,
        route_version,
        pose_count);
    }
  }
  else if(freeze_current_goal && result.code == rclcpp_action::ResultCode::ABORTED){
    RCLCPP_WARN(
      this->get_logger(),
      "release frozen route because planner failed, goal_seq=%zu, route_version=%zu, final_result=%s",
      goal_seq,
      route_version,
      final_result_class.c_str());
  }
}

void P2PGlobalPlanManager::planner_route_request_diag_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  PlannerRouteRequestDiagnostics diagnostics;
  if(msg == nullptr || !parse_planner_route_request_diagnostics(msg->data, &diagnostics)){
    return;
  }

  bool should_log_stage_update = false;
  std::size_t active_goal_seq = 0;
  {
    std::unique_lock<std::mutex> lock(access_);
    const auto previous_it = planner_route_request_diagnostics_.find(diagnostics.request_id);
    const bool stage_changed =
      previous_it == planner_route_request_diagnostics_.end() ||
      previous_it->second.stage != diagnostics.stage;
    const bool result_changed =
      previous_it == planner_route_request_diagnostics_.end() ||
      previous_it->second.result_class != diagnostics.result_class;
    planner_route_request_diagnostics_[diagnostics.request_id] = diagnostics;
    should_log_stage_update =
      diagnostics.request_id == active_route_request_id_ &&
      (stage_changed || result_changed || diagnostics.stage == "finished");
    active_goal_seq = goal_seq_;
  }

  if(should_log_stage_update){
    RCLCPP_INFO(
      this->get_logger(),
      "planner stage update, goal_seq=%zu, request_id=%zu, stage=%s, stage_elapsed_sec=%.3f, expansions=%zu, result_class=%s",
      active_goal_seq,
      diagnostics.request_id,
      diagnostics.stage.c_str(),
      diagnostics.stage_elapsed_sec,
      diagnostics.expansions,
      diagnostics.result_class.c_str());
  }
}

void P2PGlobalPlanManager::setGoal(const geometry_msgs::msg::PoseStamped& goal){
  std::unique_lock<std::mutex> lock(access_);
  goal_ = goal;
  got_first_goal_ = true;
  is_planning_ = false;
  route_requested_for_goal_ = false;
  route_request_failed_ = false;
  active_route_.poses.clear();
  frozen_route_.poses.clear();
  route_source_label_.clear();
  active_route_request_handle_.reset();
  active_route_request_id_ = 0;
  route_request_cancel_reasons_.clear();
  planner_route_request_diagnostics_.clear();
  ++goal_seq_;
  RCLCPP_INFO(
    this->get_logger(),
      "start goal lifecycle, goal_seq=%zu, freeze_route_per_goal=%d",
    goal_seq_,
    freeze_route_per_goal_);
}

bool P2PGlobalPlanManager::hasRoute(){
  std::unique_lock<std::mutex> lock(access_);
  const nav_msgs::msg::Path & active_path =
    (freeze_route_per_goal_ && !frozen_route_.poses.empty()) ? frozen_route_ : active_route_;
  if(!is_planning_ && !active_path.poses.empty())
    return true;
  return false;
}

void P2PGlobalPlanManager::copyRoute(
  std::vector<geometry_msgs::msg::PoseStamped>& route,
  std::size_t * route_version,
  std::size_t * goal_seq,
  std::string * source_label)
{
  std::unique_lock<std::mutex> lock(access_);
  route.clear();
  const nav_msgs::msg::Path & active_path =
    (freeze_route_per_goal_ && !frozen_route_.poses.empty()) ? frozen_route_ : active_route_;
  route.reserve(active_path.poses.size());
  for(const auto & pose : active_path.poses){
    route.push_back(pose);
  }
  if(route_version != nullptr){
    *route_version = route_version_;
  }
  if(goal_seq != nullptr){
    *goal_seq = goal_seq_;
  }
  if(source_label != nullptr){
    if(freeze_route_per_goal_ && !frozen_route_.poses.empty()){
      *source_label = "frozen_route";
    }
    else if(!route_source_label_.empty()){
      *source_label = route_source_label_;
    }
    else{
      *source_label = "planner_result";
    }
  }
}
}
