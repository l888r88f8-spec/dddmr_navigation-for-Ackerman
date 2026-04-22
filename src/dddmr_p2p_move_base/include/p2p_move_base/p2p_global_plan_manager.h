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
#ifndef P2P_GLOBAL_PLAN_MANAGER_H
#define P2P_GLOBAL_PLAN_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include "dddmr_sys_core/action/get_plan.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace p2p_move_base
{
class P2PGlobalPlanManager  : public rclcpp::Node 
{

private:
  using GetPlanClientGoalHandle = rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::GetPlan>;

public:
  struct PlannerRouteRequestDiagnostics
  {
    bool valid = false;
    std::size_t request_id = 0;
    std::string stage = "unknown";
    double stage_elapsed_sec = 0.0;
    std::size_t expansions = 0;
    std::string result_class = "unknown";
  };

private:
  
  std::string name_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex access_;

  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
  
  std::string route_action_name_;
  double route_request_frequency_;
  bool freeze_route_per_goal_;
  geometry_msgs::msg::PoseStamped goal_;
  bool is_planning_;
  bool got_first_goal_;
  bool route_requested_for_goal_;
  bool route_request_failed_;
  std::size_t goal_seq_;
  std::size_t route_version_;
  std::size_t next_route_request_id_;
  std::size_t active_route_request_id_;
  nav_msgs::msg::Path active_route_;
  nav_msgs::msg::Path frozen_route_;
  std::string route_source_label_;
  GetPlanClientGoalHandle::SharedPtr active_route_request_handle_;
  std::unordered_map<std::size_t, std::string> route_request_cancel_reasons_;
  std::unordered_map<std::size_t, PlannerRouteRequestDiagnostics> planner_route_request_diagnostics_;

  rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr global_planner_client_group_;
  
  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SharedPtr global_planner_client_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_route_request_diag_sub_;
  void global_planner_client_goal_response_callback(
    const GetPlanClientGoalHandle::SharedPtr & goal_handle,
    bool is_route_request,
    std::size_t request_goal_seq,
    std::size_t request_id);
  void global_planner_client_result_callback(
    const GetPlanClientGoalHandle::WrappedResult & result,
    bool is_route_request,
    std::size_t request_goal_seq,
    std::size_t request_id);
  void planner_route_request_diag_callback(const std_msgs::msg::String::SharedPtr msg);
  

public:

  P2PGlobalPlanManager(std::string name);
  ~P2PGlobalPlanManager();
  
  void queryThread();

  void initial();
  void setGoal(const geometry_msgs::msg::PoseStamped& goal);
  void resume();
  void cancelActiveRouteRequest(const std::string & reason);
  void stop(const std::string & reason = "goal stopped");
  bool hasRoute();
  bool hasPlan(){return hasRoute();}
  void copyRoute(
    std::vector<geometry_msgs::msg::PoseStamped>& route,
    std::size_t * route_version = nullptr,
    std::size_t * goal_seq = nullptr,
    std::string * source_label = nullptr);
  void copyPlan(
    std::vector<geometry_msgs::msg::PoseStamped>& plan,
    std::size_t * route_version = nullptr,
    std::size_t * goal_seq = nullptr,
    std::string * source_label = nullptr)
  {
    copyRoute(plan, route_version, goal_seq, source_label);
  }

};
}  // namespace p2p_move_base

#endif  // P2P_GLOBAL_PLAN_MANAGER_H
