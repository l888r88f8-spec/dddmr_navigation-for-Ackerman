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

#include <chrono>
#include <string>
#include <vector>

#include <dddmr_local_map/ros_adapter.hpp>

#include <local_planner/mppi_critic_manager.h>
#include <local_planner/mppi_controller.h>
#include <local_planner/mppi_directed_retreat.h>
#include <local_planner/mppi_local_map_rollout.h>
#include <local_planner/mppi_measured_travel.h>
#include <local_planner/mppi_obstacle_geometry.h>
#include <local_planner/published_cloud_cache.h>
#include <local_planner/debug_publish.h>

#include <Eigen/Geometry>

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
#include <cstdint>
#include <memory>
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

      bool isGoalPositionReached() override;
      bool isGoalReached();
      
      bool isGoalHeadingSatisfied() override;
      bool isGoalHeadingAligned();

      bool updateGlobalPose();
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

      /*Diagnostic aggregated_pc publishing, decoupled from the control loop.
        The perception cache refreshes at its own frequency, so a dedicated
        timer publishes only freshly refreshed clouds and the planner cycle
        never pays for PointCloud2 conversion.*/
      double debug_publish_aggregated_pc_frequency_{10.0};
      lp::PublishedCloudCache aggregate_cloud_cache_;
      rclcpp::TimerBase::SharedPtr aggregate_cloud_timer_;
      rclcpp::CallbackGroup::SharedPtr aggregate_cloud_callback_group_;
      void publishAggregatedCloud();
      
      /*For cuboid visualization*/
      visualization_msgs::msg::MarkerArray robot_cuboid_;
      visualization_msgs::msg::Marker marker_edge_;

      /*global plan from global planner*/
      std::vector<geometry_msgs::msg::PoseStamped> global_plan_;

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
      // Same data as /debug/best_trajectory, split into coloured segments:
      // time rollout, distance probe, and the first probe contact.
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pub_best_trajectory_markers_;
      // The distance-probe tail alone, as a PoseArray so the web UI can draw it
      // as a second series with its own colour.
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        pub_best_trajectory_probe_pose_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_accepted_trajectory_pose_array_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_best_trajectory_pose_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
      

      /*Sub*/
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ros_sub_;

      /*cb*/
      void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

      void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj,
                                      geometry_msgs::msg::PoseArray& pose_arr);
      
      void parseCuboid();
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
      bool buildHeadingReferenceFromPoseOrientation(
        const geometry_msgs::msg::PoseStamped & pose,
        tf2::Transform * reference_pose) const;

      /* Variables for prune. I have no idea to put these variables now.
         This should be optimize in future. The variables affect the control behavior.
         The variable should be adapt to vehicle speed!!!
      */
      double forward_prune_, backward_prune_;
      std::size_t causal_prune_search_window_, causal_prune_max_index_jump_;
      double causal_prune_max_arc_jump_, causal_prune_max_lateral_distance_, causal_prune_max_heading_error_;
      std::size_t max_prune_failure_cycles_;
      double cached_pruned_path_timeout_sec_;
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
      base_trajectory::Trajectory buildPredictedTrajectoryFromMppiControls(
        const std::vector<local_planner::MppiControl> & controls) const;
      local_planner::MppiScoredTrajectory buildMppiScoredTrajectory(
        const std::vector<local_planner::MppiControl> & controls,
        const local_planner::MppiObstacleSnapshot & obstacle_snapshot,
        bool check_every_obstacle_point = false) const;
      // Distance-parameterized continuation of one terminal control from the
      // given pose. Collision-only and stopped at first contact.
      // first_block_m receives the arc length of that contact, or -1.
      std::vector<local_planner::MppiTrajectoryPoint> buildMppiDistanceProbe(
        const geometry_msgs::msg::Pose & terminal_pose,
        const local_planner::MppiControl & terminal,
        const local_planner::MppiObstacleSnapshot & obstacle_snapshot,
        double * first_block_m) const;
      local_planner::MppiCriticData buildMppiCriticData(
        const std::vector<local_planner::MppiCandidate> & candidates,
        const local_planner::MppiObstacleSnapshot & obstacle_snapshot) const;
      dddmr_sys_core::PlannerState computeRppControlCommand(
        const std::string & controller_name,
        geometry_msgs::msg::Twist * cmd_vel);
      dddmr_sys_core::PlannerState computeMppiControlCommand(
        const std::string & controller_name,
        geometry_msgs::msg::Twist * cmd_vel);

      // One immutable local-map cycle: the adapter bundle, the single
      // planner<-map transform for this cycle, and the exact starting pose
      // shared by every candidate and the final weighted sequence.
      struct MppiLocalMapCycle
      {
        std::shared_ptr<const dddmr_local_map::ros::AdapterHandle> handle;
        local_planner::MppiLocalMapContext context;
        Eigen::Isometry3d initial_pose_planner{Eigen::Isometry3d::Identity()};
        std::uint64_t generation{0};
        double measurement_stamp_sec{0.0};
        double receive_steady_sec{0.0};
      };

      struct MppiLocalMapCriticBundle
      {
        local_planner::MppiCriticData data;
        std::vector<bool> candidate_valid;
        std::vector<local_planner::MppiLocalMapRolloutStatus> candidate_status;
        std::vector<std::string> candidate_detail;
        local_planner::MppiLateralOccupancyProfile lateral_profile;
      };

      bool isMppiLocalMapBackendSelected() const;
      bool acquireMppiLocalMapCycle(
        MppiLocalMapCycle * cycle, std::string * reason);
      bool revalidateMppiLocalMapCycle(
        const MppiLocalMapCycle & cycle, std::string * reason);
      local_planner::MppiLocalMapRolloutResult buildMppiLocalMapRollout(
        const MppiLocalMapCycle & cycle,
        const std::vector<local_planner::MppiControl> & controls) const;
      MppiLocalMapCriticBundle buildMppiLocalMapCriticData(
        const std::vector<local_planner::MppiCandidate> & candidates,
        const MppiLocalMapCycle & cycle) const;
      local_planner::MppiLateralOccupancyProfile measureMppiLateralOccupancyForLocalMap(
        const std::vector<local_planner::MppiPathPoint> & path,
        const MppiLocalMapCycle & cycle) const;
      local_planner::MppiLateralOccupancyProfile measureMppiLateralOccupancyForObstacleCloud(
        const std::vector<local_planner::MppiPathPoint> & path,
        const local_planner::MppiObstacleSnapshot & obstacle_snapshot) const;
      local_planner::MppiBypassSide preferredMppiBypassSide(
        const local_planner::MppiLateralOccupancyProfile & profile) const;
      // Effective forward speed cap for the MPPI backends: the external
      // allowed speed and the linear goal-approach slowdown, whichever is
      // lower. Applied to candidate rejection and the reverse ceiling.
      double mppiSpeedCap(double allowed_max_linear_speed) const;
      base_trajectory::Trajectory buildTrajectoryFromProjectedMppiPoses(
        const std::vector<Eigen::Isometry3d> & poses,
        const rclcpp::Time & stamp) const;
      dddmr_sys_core::PlannerState computeMppiLocalMapControlCommand(
        const std::string & controller_name,
        geometry_msgs::msg::Twist * cmd_vel);
      void resetMppiLocalMapSequenceOnFailure(
        const local_planner::MppiControl & measured_speed);
      void updateMppiMeasuredTravelFromOdom(
        const nav_msgs::msg::Odometry & msg);

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
      local_planner::MppiController mppi_controller_;
      local_planner::MppiDirectedRetreat mppi_directed_retreat_;
      std::size_t mppi_retreat_route_version_{std::numeric_limits<std::size_t>::max()};
      std::size_t mppi_retreat_goal_seq_{std::numeric_limits<std::size_t>::max()};
      local_planner::MppiResult computeMppiDirectedRetreat(
        const local_planner::MppiControl & measured,
        const std::vector<local_planner::MppiPathPoint> & path,
        double runtime_speed_limit,
        const local_planner::MppiDirectedRetreat::PoseValidator & validator,
        std::uint64_t environment_id);

      // Diagnostic-only blocked-corridor trace. It never feeds a control
      // decision; it records why the controller saw (or lost) a passable side.
      // Emitted on side/stage/feasibility changes and periodically while the
      // path is blocked, so the 0.1 s transition that a 2 s debug line hides is
      // visible.
      void maybeLogMppiBlockedDiagnostics(
        const local_planner::MppiObstacleSnapshot & obstacle_snapshot,
        const local_planner::MppiResult & result,
        const local_planner::MppiControl & best_sample_command,
        const local_planner::MppiLateralOccupancyProfile & profile,
        double path_offset,
        bool path_offset_valid,
        bool optimal_sequence_collision_free,
        bool has_optimal_collision_pose,
        double optimal_collision_x,
        double optimal_collision_y,
        double optimal_collision_yaw,
        std::size_t blocked_path_points);
      std::vector<local_planner::MppiPathPoint> mppi_diagnostic_path_;
      std::string mppi_blocked_diagnostics_state_;
      std::chrono::steady_clock::time_point mppi_blocked_diagnostics_last_{};
      bool mppi_debug_blocked_diagnostics_{true};
      double mppi_debug_blocked_diagnostics_period_{0.1};

      local_planner::MppiCriticManager mppi_critic_manager_;
      local_planner::MppiControl mppi_last_command_;
      bool mppi_use_perception_blocked_preemption_;
      // Autoware-style geometric bypass side selection. When disabled the
      // controller keeps its previous cost-based side comparison.
      bool mppi_bypass_geometry_enabled_{true};
      double mppi_preferred_side_min_offset_{0.15};
      double mppi_bypass_width_margin_{0.10};
      // Distance from the goal at which the forward speed cap starts to ramp
      // down to vx_min; 0 disables the goal-approach slowdown.
      double mppi_goal_slowdown_distance_{2.5};
      local_planner::MppiLateralProfileConfig mppi_lateral_profile_config_;
      local_planner::MppiMeasuredTravelTracker mppi_measured_travel_tracker_;
      local_planner::MppiEnvironmentBackend mppi_environment_backend_{
        local_planner::MppiEnvironmentBackend::kLegacyPoints};
      std::string mppi_environment_backend_name_{"legacy_points"};
      std::string mppi_local_map_init_error_;
      std::unique_ptr<dddmr_local_map::ros::LocalMapRosAdapter>
        mppi_local_map_adapter_;
      dddmr_local_map::ros::AdapterConfig mppi_local_map_config_;
      double mppi_local_map_last_ros_stamp_sec_{-1.0};
      std::vector<pcl::PointXYZ> robot_cuboid_vertices_;
      // Half the widest lateral extent of the loaded collision cuboid. Used as
      // the minimum free width a geometric bypass side must offer.
      double robot_half_width_{0.35};
      double rpp_nominal_linear_speed_;
      double rpp_min_linear_speed_;
      double rpp_alignment_linear_speed_;
      double rpp_goal_slowdown_distance_;
      bool rpp_use_velocity_scaled_lookahead_dist_;
      double rpp_lookahead_dist_;
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

};

} // end of name space

#endif  // DDDMR_LOCAL_PLANNER_H_
