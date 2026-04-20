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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

/*global planner*/
#include <global_planner/global_planner.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*TF listener, although it is included in sensor.h*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*srv msg for make plan*/
#include "dddmr_sys_core/action/get_plan.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <string>

namespace global_planner
{

class DWA_GlobalPlanner : public rclcpp::Node {
    public:
      DWA_GlobalPlanner(const std::string& name);
      ~DWA_GlobalPlanner();

      void initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner);
   
    private:

      bool is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> handle) const
      {
        return handle != nullptr && handle->is_active();
      }

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);

      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);
      
      std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> current_handle_;
      std::mutex current_handle_mutex_;
      std::mutex plan_state_mutex_;

      rclcpp_action::Server<dddmr_sys_core::action::GetPlan>::SharedPtr action_server_global_planner_;

      rclcpp::CallbackGroup::SharedPtr action_server_group_;
      
      rclcpp::Clock::SharedPtr clock_;
      rclcpp::TimerBase::SharedPtr threading_timer_;
      
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
      
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      std::shared_ptr<global_planner::GlobalPlanner> global_planner_;

      std::string global_frame_;
      std::string robot_frame_;
      geometry_msgs::msg::PoseStamped new_goal_;
      geometry_msgs::msg::PoseStamped current_goal_;
      nav_msgs::msg::Path global_path_;
      nav_msgs::msg::Path global_dwa_path_;
      double look_ahead_distance_;
      double recompute_frequency_;
      double startup_replan_lock_distance_;
      double startup_replan_lock_offtrack_tolerance_;
      double corridor_forward_distance_;
      double reconnect_target_distance_;
      double reconnect_deviation_threshold_;
      double reconnect_heading_threshold_;
      double max_reconnect_failures_before_global_replan_;
      double preferred_turn_sign_lookahead_distance_;
      double min_force_goal_heading_distance_;
      double max_force_goal_heading_angle_;
      double connector_goal_tolerance_;
      double connector_commit_distance_;
      double connector_timeout_sec_;
      std::size_t route_version_;
      std::size_t connector_version_;
      rclcpp::Time last_successful_reconnect_time_;
      rclcpp::Time last_global_replan_time_;
      std::size_t consecutive_reconnect_failures_;
      bool active_connector_latched_;
      geometry_msgs::msg::PoseStamped active_reconnect_goal_;
      std::size_t active_route_anchor_index_;
      rclcpp::Time active_connector_start_time_;
      std::size_t active_connector_route_version_;
      nav_msgs::msg::Path active_connector_path_;
      geometry_msgs::msg::PoseStamped active_connector_start_pose_;
      bool active_connector_has_goal_heading_;
      int active_connector_preferred_turn_sign_;

      enum class PlannerMode
      {
        kRouteFollow = 0,
        kLocalReconnect = 1,
        kLockConnector = 2,
        kReplanGlobal = 3,
        kSafeStop = 4
      };

      enum class ReconnectResult
      {
        kSuccess = 0,
        kRouteProjectionFailed = 1,
        kConnectorFailed = 2,
        kCorridorEmpty = 3,
        kReusePrunedPath = 4,
        kStartupLockReusedPath = 5,
        kGlobalReplanTriggered = 6
      };

      struct CorridorSegment
      {
        nav_msgs::msg::Path path;
        std::size_t nearest_index = 0;
        std::size_t anchor_index = 0;
        bool has_anchor_yaw = false;
        double anchor_yaw = 0.0;
      };

      PlannerMode planner_mode_;
      
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_path_; 
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_path_;
      
      /*Func*/
      void makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);
      bool isNewGoal();
      void determineDWAPlan();
      bool ComputePathTangentYaw(
        const nav_msgs::msg::Path & path,
        std::size_t pivot_index,
        double * yaw) const;
      bool PrunePathPrefix(
        nav_msgs::msg::Path * path,
        const geometry_msgs::msg::PoseStamped & robot_pose,
        bool insert_robot_pose) const;
      bool ExtractCorridor(
        const nav_msgs::msg::Path & route_path,
        const geometry_msgs::msg::PoseStamped & robot_pose,
        CorridorSegment * corridor) const;
      bool IsReconnectRequired(
        const CorridorSegment & corridor,
        const geometry_msgs::msg::PoseStamped & robot_pose) const;
      bool BuildReconnectGoalFromCorridor(
        const CorridorSegment & corridor,
        geometry_msgs::msg::PoseStamped * reconnect_goal,
        bool * has_tangent_yaw,
        std::size_t * anchor_index) const;
      nav_msgs::msg::Path ComposeConnectorWithRouteTail(
        const nav_msgs::msg::Path & connector_path,
        const nav_msgs::msg::Path & route_path,
        std::size_t route_anchor_index) const;
      bool ExtractPoseYaw(
        const geometry_msgs::msg::PoseStamped & pose,
        double * yaw) const;
      std::size_t FindNearestPathIndex(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseStamped & pose) const;
      int EstimatePreferredTurnSignAhead(
        const nav_msgs::msg::Path & path,
        std::size_t start_index,
        double max_forward_distance) const;
      bool ShouldLockStartupReplan(
        const nav_msgs::msg::Path & dwa_path,
        const geometry_msgs::msg::PoseStamped & robot_pose) const;
      double ComputeDistanceToPath(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseStamped & pose) const;
      double EstimateProgressAlongPath(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseStamped & pose) const;
      const char * PlannerModeToString(PlannerMode mode) const;
      const char * ReconnectResultToString(ReconnectResult result) const;
      void SwitchPlannerMode(
        PlannerMode next_mode,
        const std::string & reason,
        ReconnectResult result);
      bool clearCurrentHandleIfMatches(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);

};

}
