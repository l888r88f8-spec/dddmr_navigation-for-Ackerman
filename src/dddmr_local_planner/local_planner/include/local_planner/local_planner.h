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

#ifndef DDDMR_LOCAL_PLANNER_H_
#define DDDMR_LOCAL_PLANNER_H_

#include <angles/angles.h>

/*For graph*/
#include <queue>
#include <set>
#include <unordered_map>
/*For perception plugin*/
#include <perception_3d/perception_3d_ros.h>

/*For critics plugin*/
#include <mpc_critics/mpc_critics_ros.h>

/*For trajectory generators plugin*/
#include <trajectory_generators/trajectory_generators_ros.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>

//@planner state
#include <dddmr_sys_core/dddmr_enum_states.h>
#include <dddmr_sys_core/route_tracking_controller.h>

#include <cstddef>
#include <string>
#include <vector>

namespace local_planner {

class Local_Planner : public rclcpp::Node, public dddmr_sys_core::RouteTrackingController {

    public:
      Local_Planner(const std::string& name);
      
      void initial(
        const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
        const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
        const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators);

      ~Local_Planner();

      void setRoute(
        const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
        std::size_t route_version = 0,
        std::size_t goal_seq = 0,
        const std::string & source_label = "planner_result") override;
      void setPlan(
        const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
        std::size_t route_version = 0,
        std::size_t goal_seq = 0,
        const std::string & source_label = "planner_result");
      dddmr_sys_core::PlannerState computeControlCommand(
        const std::string & controller_name,
        geometry_msgs::msg::Twist * cmd_vel) override;
      dddmr_sys_core::PlannerState computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);
      void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);

      //@ shared data for trajectory generator, we manage the variables by this way for future changed to ROS2
      std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> traj_shared_data_;
      
      bool isGoalPositionReached() override;
      bool isGoalReached();
      
      bool isRouteStartAligned() override;
      dddmr_sys_core::RouteStartupStatus evaluateRouteStartupStatus(
        std::string * detail) override;
      bool isInitialHeadingAligned();
      bool isInitialHeadingAlignedOnCurrentPrunePlan();
      bool isRouteStartFrontReachableOnCurrentPrunePlan(std::string * detail) const;
      bool isGoalHeadingSatisfied() override;
      bool isGoalHeadingAligned();

      void updateGlobalPose();
      geometry_msgs::msg::TransformStamped getGlobalPose() override;
      
    private: 
      
      rclcpp::Clock::SharedPtr clock_;

      rclcpp::CallbackGroup::SharedPtr cbs_group_;
      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

      //@ for percertion
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      //@ for critics
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
      //@ for trajectory generator
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;

      std::string global_frame_;
      std::string robot_frame_;
      std::string odom_topic_;
      std::string odom_topic_qos_;
      bool debug_publish_robot_cuboid_;
      bool debug_publish_aggregated_pc_;
      bool debug_publish_prune_plan_;
      bool debug_publish_accepted_trajectory_;
      bool debug_publish_best_trajectory_;
      bool debug_publish_all_trajectories_;
      
      /*For cuboid visualization*/
      visualization_msgs::msg::MarkerArray robot_cuboid_;
      visualization_msgs::msg::Marker marker_edge_;

      /*Original point cloud*/
      pcl::PointCloud<pcl::PointNormal>::Ptr ground_with_normals_;
      
      /*global plan from global planner*/
      std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_plan_; 
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_plan_;
      
      /*store global pose*/
      geometry_msgs::msg::TransformStamped trans_gbl2b_;
      
      /*Compute shortest between robot heading and given pose*/
      double getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose);

      /*Pub*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_robot_cuboid_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aggregate_observation_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_prune_plan_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_route_sent_to_local_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_pruned_path_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_best_trajectory_path_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_accepted_trajectory_pose_array_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_best_trajectory_pose_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
      //ros::Publisher pub_pc_normal_;
      //ros::Publisher pub_trajectory_cuboids_;
      

      /*Sub*/
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ros_sub_;

      /*cb*/
      void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

      void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids);
      
      void parseCuboid();
      //void normal2quaternion();
      bool prunePlan(double forward_distance, double backward_distance);
      double getDistanceBTWPoseStamp(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) const;
      void resetLocalRouteTrackingState();
      void rebuildGlobalPlanArcLengths();
      double getGlobalPlanArcDistance(std::size_t start_index, std::size_t end_index) const;
      double getRobotYaw() const;
      double getRouteSegmentLateralDistance(std::size_t index) const;
      double getRouteHeadingError(std::size_t index) const;
      bool selectCausalPruneAnchor(std::size_t * anchor_index, double * robot_to_route_distance);
      bool buildPrunedPlanAroundIndex(
        std::size_t anchor_index,
        double forward_distance,
        double backward_distance,
        nav_msgs::msg::Path * prune_plan,
        pcl::PointCloud<pcl::PointXYZI> * pcl_prune_plan) const;
      bool tryReuseCachedPrunedPath();
      bool buildHeadingReferenceFromPoses(
        const geometry_msgs::msg::PoseStamped & first_pose,
        const geometry_msgs::msg::PoseStamped & last_pose,
        tf2::Transform * reference_pose) const;
      bool buildHeadingReferenceFromPlan(
        const std::vector<geometry_msgs::msg::PoseStamped> & plan,
        std::size_t start_index,
        double min_reference_length,
        tf2::Transform * reference_pose) const;
      bool buildHeadingReferenceFromPoseOrientation(
        const geometry_msgs::msg::PoseStamped & pose,
        tf2::Transform * reference_pose) const;
      void cacheHeadingReference(const tf2::Transform & reference_pose);
      double updateHeadingDeviation(const tf2::Transform & reference_pose);

      bool compute_best_trajectory_in_odomCb_;

      /* Variables for prune. I have no idea to put these variables now.
         This should be optimize in future. The variables affect the control behavior.
         The variable should be adapt to vehicle speed!!!
      */
      double forward_prune_, backward_prune_, heading_tracking_distance_, heading_align_angle_;
      double route_start_front_reach_distance_;
      double route_start_front_projection_threshold_;
      std::size_t causal_prune_search_window_, causal_prune_max_index_jump_;
      double causal_prune_max_arc_jump_, causal_prune_max_lateral_distance_, causal_prune_max_heading_error_;
      double min_heading_reference_length_;
      std::size_t max_heading_reference_stale_cycles_;
      std::size_t max_prune_failure_cycles_;
      double cached_pruned_path_timeout_sec_;
      bool enable_prune_deviation_hard_fail_;
      bool allow_offroute_anchor_recovery_;

      /*Timer for robust system design*/
      double prune_plane_timeout_;
      rclcpp::Time last_valid_prune_plan_;
      rclcpp::Time cached_local_pruned_path_stamp_;
      bool got_odom_;
      std::size_t local_route_progress_index_;
      double last_robot_to_route_distance_;
      std::size_t consecutive_prune_failure_cycles_;
      bool last_prune_used_cache_;
      bool cached_local_pruned_path_valid_;
      nav_msgs::msg::Path cached_local_pruned_path_;
      pcl::PointCloud<pcl::PointXYZI> cached_pcl_prune_plan_;
      std::vector<double> global_plan_arc_lengths_;
      bool last_heading_reference_valid_;
      tf2::Transform last_heading_reference_pose_;
      std::size_t heading_reference_stale_cycles_;
      std::size_t consecutive_heading_reference_failure_cycles_;

      double xy_goal_tolerance_, yaw_goal_tolerance_;
      double controller_frequency_;

      rclcpp::Time control_loop_time_;
      std::size_t route_version_;
      std::size_t goal_seq_;
      std::string route_source_label_;

      nav_msgs::msg::Path buildPathFromPlan(
        const std::vector<geometry_msgs::msg::PoseStamped> & poses) const;
      nav_msgs::msg::Path buildPathFromTrajectory(
        const base_trajectory::Trajectory & traj) const;
      void publishDebugPath(
        const nav_msgs::msg::Path & path,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
        const std::string & stage_label,
        std::size_t route_version,
        std::size_t goal_seq,
        const std::string & source_label,
        bool throttle_log) const;

      struct ControllerContext
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_observation;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr aggregate_observation_kdtree;
        std::vector<perception_3d::PerceptionOpinion> opinions;
        double current_allowed_max_linear_speed = -1.0;
      };

      dddmr_sys_core::PlannerState prepareControllerContext(
        ControllerContext * context);
      dddmr_sys_core::PlannerState evaluatePerceptionOpinions(
        const std::vector<perception_3d::PerceptionOpinion> & opinions) const;
      void updateCriticSharedData(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr & aggregate_observation,
        const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr & aggregate_observation_kdtree);
      bool selectLookaheadPose(
        double lookahead_distance,
        geometry_msgs::msg::PoseStamped * lookahead_pose) const;
      bool buildGoalAlignmentReference(tf2::Transform * reference_pose) const;
      double getDistanceToGoal() const;
      double getMaxAckermannCurvature() const;
      pcl::PointCloud<pcl::PointXYZ> transformRobotCuboid(
        const geometry_msgs::msg::PoseStamped & pose) const;
      base_trajectory::cuboid_min_max_t computeCuboidMinMax(
        const pcl::PointCloud<pcl::PointXYZ> & cuboid) const;
      base_trajectory::Trajectory buildPredictedTrajectory(
        double linear_velocity,
        double angular_velocity) const;
      dddmr_sys_core::PlannerState computeRppControlCommand(
        const std::string & controller_name,
        geometry_msgs::msg::Twist * cmd_vel);

    protected:

      std::shared_ptr<tf2_ros::TransformListener> tfl_;
      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
      nav_msgs::msg::Odometry robot_state_;
      std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;
      nav_msgs::msg::Path prune_plan_;
      pcl::PointCloud<pcl::PointXYZI> pcl_prune_plan_; //@ will be copied to perception_ros, so do not use shared_ptr
      std::string name_;
      std::string controller_backend_;
      std::string rpp_critic_trajectory_generator_name_;
      std::vector<pcl::PointXYZ> robot_cuboid_vertices_;
      double rpp_nominal_linear_speed_;
      double rpp_min_linear_speed_;
      double rpp_alignment_linear_speed_;
      double rpp_goal_slowdown_distance_;
      double rpp_min_lookahead_distance_;
      double rpp_max_lookahead_distance_;
      double rpp_lookahead_time_;
      double rpp_alignment_lookahead_distance_;
      double rpp_max_lateral_accel_;
      double rpp_prediction_horizon_sec_;
      double rpp_prediction_step_sec_;
      double rpp_wheelbase_;
      double rpp_max_steer_;
      double rpp_max_angular_velocity_;
      int rpp_avoidance_angular_samples_;
      double rpp_avoidance_angular_span_ratio_;
      double rpp_avoidance_prefer_nominal_weight_;
      double rpp_avoidance_curvature_switch_weight_;
      double rpp_avoidance_turn_switch_penalty_;
      double rpp_avoidance_turn_deadband_;
      bool rpp_have_last_selected_curvature_;
      double rpp_last_selected_curvature_;

};

} // end of name space

#endif  // DDDMR_LOCAL_PLANNER_H_
