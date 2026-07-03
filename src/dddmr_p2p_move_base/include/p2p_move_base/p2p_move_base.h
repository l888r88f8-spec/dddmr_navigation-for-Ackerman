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
/*Debug*/
#include <chrono>
#include <mutex>
#include <p2p_move_base/p2p_fsm.h>

//@in enum state, the p_to_p_move_base is included
#include <dddmr_sys_core/dddmr_enum_states.h>
#include <dddmr_sys_core/route_tracking_controller.h>

//@for call global planner action
#include "dddmr_sys_core/action/get_plan.hpp"
#include "p2p_move_base/local_obstacle_layer.h"
#include "p2p_move_base/navigation_fault_policy.h"
#include "p2p_move_base/runtime_budget_monitor.h"
#include "p2p_move_base/p2p_global_plan_manager.h"
//@for call recovery action
#include "dddmr_sys_core/action/recovery_behaviors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"


namespace p2p_move_base
{

class P2PMoveBase : public rclcpp::Node {

  public:

    P2PMoveBase(std::string name);
    ~P2PMoveBase();

    void initial(
      const std::shared_ptr<dddmr_sys_core::RouteTrackingController>& route_controller,
      const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& route_manager);
  private:

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);
    
    rclcpp_action::Server<dddmr_sys_core::action::PToPMoveBase>::SharedPtr action_server_p2p_move_base_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> current_handle_;
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::CallbackGroup::SharedPtr action_server_group_;
    rclcpp::CallbackGroup::SharedPtr recovery_behaviors_client_group_;
    rclcpp::CallbackGroup::SharedPtr local_obstacle_layer_group_;

    rclcpp::Clock::SharedPtr clock_;
    
    std::string name_;
    
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mb_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr local_obstacle_status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_obstacle_debug_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_obstacle_cloud_sub_;

    bool isQuaternionValid(const geometry_msgs::msg::Quaternion& q);

    void publishZeroVelocity();
    void publishVelocity(double vx, double vy, double angular_z);
    void publishVelocity(const geometry_msgs::msg::Twist & cmd_vel);

    std::string tracking_controller_name_;
    std::string alignment_controller_name_;
    std::shared_ptr<p2p_move_base::FSM> FSM_;
    std::shared_ptr<dddmr_sys_core::RouteTrackingController> route_controller_;
    std::shared_ptr<p2p_move_base::P2PGlobalPlanManager> route_manager_;

    void executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);

    bool executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);
    bool syncRouteReferenceFromManager(const std::string & consumer_label);
    std::string terminal_goal_reason_;
    std::string current_route_result_class_;

    bool is_active(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }

    rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr recovery_behaviors_client_ptr_;
    void recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle);
    void recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result);
    bool is_recoverying_;
    bool is_recoverying_succeed_;
    std::string recovery_action_name_;
    p2p_move_base::NavigationFaultPolicy navigation_fault_policy_;
    p2p_move_base::RuntimeBudgetMonitor runtime_budget_monitor_;
    p2p_move_base::LocalObstacleLayer local_obstacle_layer_;
    double min_replan_interval_sec_;
    bool runtime_budget_safety_stop_enabled_;
    bool local_obstacle_layer_enabled_;
    bool local_obstacle_debug_cloud_enabled_;
    std::string local_obstacle_base_frame_;
    std::mutex local_obstacle_snapshot_mutex_;
    std::shared_ptr<const p2p_move_base::LocalObstacleSnapshot> latest_local_obstacle_snapshot_;
    bool startRecoveryAction(const std::string & trigger_reason);
    void initializeLocalObstacleLayer();
    void localObstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


};



}//end of name space
