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
#include <local_planner/local_planner.h>
#include <tf2/utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <string>
#include <utility>

namespace local_planner {
namespace
{
std::string normalizeControllerBackend(const std::string & backend)
{
  if(backend == "regulated_pure_pursuit" || backend == "rpp"){
    return "regulated_pure_pursuit";
  }
  if(backend == "legacy_rollout" || backend == "legacy"){
    return "legacy_rollout";
  }
  if(backend == "mppi" || backend == "mppi_controller" || backend == "nav2_mppi"){
    return "mppi";
  }
  return backend;
}
}

Local_Planner::Local_Planner(const std::string& name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  got_odom_ = false;
  last_valid_prune_plan_ = clock_->now();
  cached_local_pruned_path_stamp_ = clock_->now();
  route_version_ = 0;
  goal_seq_ = 0;
  route_source_label_ = "planner_result";
  local_route_progress_index_ = 0;
  last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
  consecutive_prune_failure_cycles_ = 0;
  last_prune_used_cache_ = false;
  cached_local_pruned_path_valid_ = false;
  controller_backend_ = "legacy_rollout";
  rpp_critic_trajectory_generator_name_ = "ackermann_simple";
  mppi_last_command_ = MppiControl{0.05, 0.0};
  rpp_nominal_linear_speed_ = 1.0;
  rpp_min_linear_speed_ = 0.15;
  rpp_alignment_linear_speed_ = 0.25;
  rpp_goal_slowdown_distance_ = 1.5;
  rpp_use_velocity_scaled_lookahead_dist_ = false;
  rpp_lookahead_dist_ = 1.0;
  rpp_min_lookahead_distance_ = 1.0;
  rpp_max_lookahead_distance_ = 3.0;
  rpp_lookahead_time_ = 1.5;
  rpp_alignment_lookahead_distance_ = 1.0;
  rpp_max_lateral_accel_ = 1.5;
  rpp_prediction_horizon_sec_ = 1.5;
  rpp_prediction_step_sec_ = 0.1;
  rpp_wheelbase_ = 0.550;
  rpp_max_steer_ = 0.3270520925102273;
  rpp_max_angular_velocity_ = 1.2;
  mppi_use_perception_blocked_preemption_ = false;
  allow_offroute_anchor_recovery_ = false;
}

void Local_Planner::initial(
      const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
      const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
      const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators){

  declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_topic", odom_topic_);
  RCLCPP_DEBUG(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());

  declare_parameter("odom_topic_qos", rclcpp::ParameterValue("reliable"));
  this->get_parameter("odom_topic_qos", odom_topic_qos_);
  RCLCPP_DEBUG(this->get_logger(), "odom_topic_qos: %s", odom_topic_qos_.c_str());

  declare_parameter(
    "local_reference.forward_distance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_forward_reference_distance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "local_reference.forward_distance",
    configured_forward_reference_distance);
  declare_parameter("forward_prune", rclcpp::ParameterValue(1.0));
  this->get_parameter("forward_prune", forward_prune_);
  if(std::isfinite(configured_forward_reference_distance)){
    forward_prune_ = configured_forward_reference_distance;
    RCLCPP_DEBUG(this->get_logger(), "local_reference.forward_distance: %.2f", forward_prune_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "forward_prune (legacy local reference alias): %.2f",
      forward_prune_);
  }

  declare_parameter(
    "local_reference.backward_distance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_backward_reference_distance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "local_reference.backward_distance",
    configured_backward_reference_distance);
  declare_parameter("backward_prune", rclcpp::ParameterValue(0.5));
  this->get_parameter("backward_prune", backward_prune_);
  if(std::isfinite(configured_backward_reference_distance)){
    backward_prune_ = configured_backward_reference_distance;
    RCLCPP_DEBUG(this->get_logger(), "local_reference.backward_distance: %.2f", backward_prune_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "backward_prune (legacy local reference alias): %.2f",
      backward_prune_);
  }

  declare_parameter("causal_prune_search_window", rclcpp::ParameterValue(40));
  this->get_parameter("causal_prune_search_window", causal_prune_search_window_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_search_window: %zu", causal_prune_search_window_);

  declare_parameter("causal_prune_max_index_jump", rclcpp::ParameterValue(15));
  this->get_parameter("causal_prune_max_index_jump", causal_prune_max_index_jump_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_index_jump: %zu", causal_prune_max_index_jump_);

  declare_parameter("causal_prune_max_arc_jump", rclcpp::ParameterValue(2.0));
  this->get_parameter("causal_prune_max_arc_jump", causal_prune_max_arc_jump_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_arc_jump: %.2f", causal_prune_max_arc_jump_);

  declare_parameter("causal_prune_max_lateral_distance", rclcpp::ParameterValue(1.0));
  this->get_parameter("causal_prune_max_lateral_distance", causal_prune_max_lateral_distance_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_lateral_distance: %.2f", causal_prune_max_lateral_distance_);

  declare_parameter("causal_prune_max_heading_error", rclcpp::ParameterValue(1.75));
  this->get_parameter("causal_prune_max_heading_error", causal_prune_max_heading_error_);
  RCLCPP_DEBUG(this->get_logger(), "causal_prune_max_heading_error: %.2f", causal_prune_max_heading_error_);

  declare_parameter("max_prune_failure_cycles", rclcpp::ParameterValue(10));
  this->get_parameter("max_prune_failure_cycles", max_prune_failure_cycles_);
  RCLCPP_DEBUG(this->get_logger(), "max_prune_failure_cycles: %zu", max_prune_failure_cycles_);

  declare_parameter("allow_offroute_anchor_recovery", rclcpp::ParameterValue(false));
  this->get_parameter("allow_offroute_anchor_recovery", allow_offroute_anchor_recovery_);
  RCLCPP_DEBUG(
    this->get_logger(),
    "allow_offroute_anchor_recovery: %d",
    allow_offroute_anchor_recovery_ ? 1 : 0);

  declare_parameter("cached_pruned_path_timeout_sec", rclcpp::ParameterValue(0.8));
  this->get_parameter("cached_pruned_path_timeout_sec", cached_pruned_path_timeout_sec_);
  RCLCPP_DEBUG(this->get_logger(), "cached_pruned_path_timeout_sec: %.2f", cached_pruned_path_timeout_sec_);

  declare_parameter("prune_plane_timeout", rclcpp::ParameterValue(3.0));
  this->get_parameter("prune_plane_timeout", prune_plane_timeout_);
  RCLCPP_DEBUG(this->get_logger(), "prune_plane_timeout: %.2f", prune_plane_timeout_);

  declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_DEBUG(this->get_logger(), "xy_goal_tolerance: %.2f", xy_goal_tolerance_);

  declare_parameter(
    "goal_alignment.heading_tolerance",
    rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
  double configured_goal_alignment_tolerance = std::numeric_limits<double>::quiet_NaN();
  this->get_parameter(
    "goal_alignment.heading_tolerance",
    configured_goal_alignment_tolerance);
  declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  if(std::isfinite(configured_goal_alignment_tolerance)){
    yaw_goal_tolerance_ = configured_goal_alignment_tolerance;
    RCLCPP_DEBUG(
      this->get_logger(),
      "goal_alignment.heading_tolerance: %.2f",
      yaw_goal_tolerance_);
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger(),
      "yaw_goal_tolerance (legacy goal alignment alias): %.2f",
      yaw_goal_tolerance_);
  }

  declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_DEBUG(this->get_logger(), "controller_frequency: %.2f", controller_frequency_);

  declare_parameter("controller_backend", rclcpp::ParameterValue("legacy_rollout"));
  this->get_parameter("controller_backend", controller_backend_);
  controller_backend_ = normalizeControllerBackend(controller_backend_);
  if(controller_backend_ != "legacy_rollout" &&
     controller_backend_ != "regulated_pure_pursuit" &&
     controller_backend_ != "mppi")
  {
    RCLCPP_WARN(
      this->get_logger(),
      "unknown controller_backend '%s', fallback to legacy_rollout",
      controller_backend_.c_str());
    controller_backend_ = "legacy_rollout";
  }
  RCLCPP_DEBUG(this->get_logger(), "controller_backend: %s", controller_backend_.c_str());

  declare_parameter(
    mppiEnvironmentBackendParameterName(),
    rclcpp::ParameterValue("legacy_points"));
  this->get_parameter(
    mppiEnvironmentBackendParameterName(), mppi_environment_backend_name_);
  if(!parseMppiEnvironmentBackend(
      mppi_environment_backend_name_, &mppi_environment_backend_))
  {
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "invalid %s '%s'; exact accepted values are 'legacy_points' and 'local_map'",
      mppiEnvironmentBackendParameterName(),
      mppi_environment_backend_name_.c_str());
    mppi_local_map_init_error_ =
      "invalid " + std::string(mppiEnvironmentBackendParameterName()) + " '" +
      mppi_environment_backend_name_ + "'";
    mppi_environment_backend_ = MppiEnvironmentBackend::kLegacyPoints;
  }
  RCLCPP_DEBUG(
    this->get_logger(),
    "%s: %s",
    mppiEnvironmentBackendParameterName(),
    toString(mppi_environment_backend_));

  declare_parameter(
    "regulated_pure_pursuit.critic_trajectory_generator_name",
    rclcpp::ParameterValue("ackermann_simple"));
  this->get_parameter(
    "regulated_pure_pursuit.critic_trajectory_generator_name",
    rpp_critic_trajectory_generator_name_);
  declare_parameter(
    "regulated_pure_pursuit.nominal_linear_speed",
    rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "regulated_pure_pursuit.nominal_linear_speed",
    rpp_nominal_linear_speed_);
  declare_parameter(
    "regulated_pure_pursuit.min_linear_speed",
    rclcpp::ParameterValue(0.15));
  this->get_parameter(
    "regulated_pure_pursuit.min_linear_speed",
    rpp_min_linear_speed_);
  declare_parameter(
    "regulated_pure_pursuit.alignment_linear_speed",
    rclcpp::ParameterValue(0.25));
  this->get_parameter(
    "regulated_pure_pursuit.alignment_linear_speed",
    rpp_alignment_linear_speed_);
  declare_parameter(
    "regulated_pure_pursuit.goal_slowdown_distance",
    rclcpp::ParameterValue(1.5));
  this->get_parameter(
    "regulated_pure_pursuit.goal_slowdown_distance",
    rpp_goal_slowdown_distance_);
  declare_parameter(
    "regulated_pure_pursuit.use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  this->get_parameter(
    "regulated_pure_pursuit.use_velocity_scaled_lookahead_dist",
    rpp_use_velocity_scaled_lookahead_dist_);
  declare_parameter(
    "regulated_pure_pursuit.lookahead_dist",
    rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "regulated_pure_pursuit.lookahead_dist",
    rpp_lookahead_dist_);
  declare_parameter(
    "regulated_pure_pursuit.min_lookahead_distance",
    rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "regulated_pure_pursuit.min_lookahead_distance",
    rpp_min_lookahead_distance_);
  declare_parameter(
    "regulated_pure_pursuit.max_lookahead_distance",
    rclcpp::ParameterValue(3.0));
  this->get_parameter(
    "regulated_pure_pursuit.max_lookahead_distance",
    rpp_max_lookahead_distance_);
  declare_parameter(
    "regulated_pure_pursuit.lookahead_time",
    rclcpp::ParameterValue(1.5));
  this->get_parameter(
    "regulated_pure_pursuit.lookahead_time",
    rpp_lookahead_time_);
  declare_parameter(
    "regulated_pure_pursuit.alignment_lookahead_distance",
    rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "regulated_pure_pursuit.alignment_lookahead_distance",
    rpp_alignment_lookahead_distance_);
  declare_parameter(
    "regulated_pure_pursuit.max_lateral_accel",
    rclcpp::ParameterValue(1.5));
  this->get_parameter(
    "regulated_pure_pursuit.max_lateral_accel",
    rpp_max_lateral_accel_);
  declare_parameter(
    "regulated_pure_pursuit.prediction_horizon_sec",
    rclcpp::ParameterValue(1.5));
  this->get_parameter(
    "regulated_pure_pursuit.prediction_horizon_sec",
    rpp_prediction_horizon_sec_);
  declare_parameter(
    "regulated_pure_pursuit.prediction_step_sec",
    rclcpp::ParameterValue(0.1));
  this->get_parameter(
    "regulated_pure_pursuit.prediction_step_sec",
    rpp_prediction_step_sec_);
  declare_parameter(
    "regulated_pure_pursuit.wheelbase",
    rclcpp::ParameterValue(0.550));
  this->get_parameter(
    "regulated_pure_pursuit.wheelbase",
    rpp_wheelbase_);
  declare_parameter(
    "regulated_pure_pursuit.max_steer",
    rclcpp::ParameterValue(0.3270520925102273));
  this->get_parameter(
    "regulated_pure_pursuit.max_steer",
    rpp_max_steer_);
  declare_parameter(
    "regulated_pure_pursuit.max_angular_velocity",
    rclcpp::ParameterValue(1.2));
  this->get_parameter(
    "regulated_pure_pursuit.max_angular_velocity",
    rpp_max_angular_velocity_);
  if(controller_backend_ == "regulated_pure_pursuit"){
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.critic_trajectory_generator_name: %s",
      rpp_critic_trajectory_generator_name_.c_str());
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.nominal_linear_speed: %.2f",
      rpp_nominal_linear_speed_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.min_linear_speed: %.2f",
      rpp_min_linear_speed_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.alignment_linear_speed: %.2f",
      rpp_alignment_linear_speed_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.goal_slowdown_distance: %.2f",
      rpp_goal_slowdown_distance_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.use_velocity_scaled_lookahead_dist: %d",
      rpp_use_velocity_scaled_lookahead_dist_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.lookahead_dist: %.2f",
      rpp_lookahead_dist_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.min_lookahead_distance: %.2f",
      rpp_min_lookahead_distance_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.max_lookahead_distance: %.2f",
      rpp_max_lookahead_distance_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.lookahead_time: %.2f",
      rpp_lookahead_time_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.alignment_lookahead_distance: %.2f",
      rpp_alignment_lookahead_distance_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.max_lateral_accel: %.2f",
      rpp_max_lateral_accel_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.prediction_horizon_sec: %.2f",
      rpp_prediction_horizon_sec_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.prediction_step_sec: %.2f",
      rpp_prediction_step_sec_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.wheelbase: %.2f",
      rpp_wheelbase_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.max_steer: %.2f",
      rpp_max_steer_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "regulated_pure_pursuit.max_angular_velocity: %.2f",
      rpp_max_angular_velocity_);
  }

  MppiConfig mppi_config;
  MppiCriticConfig mppi_critic_config;
  double mppi_prediction_horizon_sec = 2.5;
  declare_parameter("mppi.prediction_horizon_sec", rclcpp::ParameterValue(2.5));
  declare_parameter("mppi.model_dt", rclcpp::ParameterValue(0.1));
  declare_parameter("mppi.batch_size", rclcpp::ParameterValue(384));
  declare_parameter("mppi.structured_candidate_count", rclcpp::ParameterValue(8));
  declare_parameter("mppi.structured_min_vx", rclcpp::ParameterValue(0.20));
  declare_parameter("mppi.departure_lateral_offset", rclcpp::ParameterValue(0.42));
  declare_parameter("mppi.iteration_count", rclcpp::ParameterValue(1));
  declare_parameter("mppi.temperature", rclcpp::ParameterValue(0.3));
  declare_parameter("mppi.gamma", rclcpp::ParameterValue(0.015));
  declare_parameter("mppi.vx_std", rclcpp::ParameterValue(0.18));
  declare_parameter("mppi.wz_std", rclcpp::ParameterValue(0.25));
  declare_parameter("mppi.vx_min", rclcpp::ParameterValue(0.05));
  declare_parameter("mppi.vx_max", rclcpp::ParameterValue(0.8));
  declare_parameter("mppi.reverse_recovery_enabled", rclcpp::ParameterValue(false));
  declare_parameter("mppi.reverse_max_speed", rclcpp::ParameterValue(0.15));
  declare_parameter("mppi.reverse_episode_distance", rclcpp::ParameterValue(0.5));
  declare_parameter("mppi.wz_max", rclcpp::ParameterValue(1.2));
  declare_parameter("mppi.ax_min", rclcpp::ParameterValue(-0.8));
  declare_parameter("mppi.ax_max", rclcpp::ParameterValue(0.8));
  declare_parameter("mppi.az_max", rclcpp::ParameterValue(1.5));
  declare_parameter("mppi.wheelbase", rclcpp::ParameterValue(0.550));
  declare_parameter("mppi.max_steer", rclcpp::ParameterValue(0.3270520925102273));
  declare_parameter("mppi.seed", rclcpp::ParameterValue(7));
  declare_parameter("mppi.regenerate_noises", rclcpp::ParameterValue(false));
  declare_parameter("mppi.use_perception_blocked_preemption", rclcpp::ParameterValue(false));
  declare_parameter("mppi.avoidance_geometry_enabled", rclcpp::ParameterValue(true));
  declare_parameter("mppi.preferred_side_min_offset", rclcpp::ParameterValue(0.15));
  declare_parameter("mppi.bypass_width_margin", rclcpp::ParameterValue(0.10));
  // Cost per metre of lateral travel needed to reach the usable bypass band on
  // a side. Only applied while the blocking geometry does not force a side.
  // 0 disables the detour preference and restores pure cost comparison.
  declare_parameter("mppi.bypass_detour_weight", rclcpp::ParameterValue(1.0));
  // Linear approach slowdown shared with the RPP backend: full speed at this
  // distance from the goal, vx_min at the goal. 0 disables it.
  declare_parameter("mppi.goal_slowdown_distance", rclcpp::ParameterValue(2.5));
  declare_parameter("mppi.lateral_probe_max_offset", rclcpp::ParameterValue(1.5));
  declare_parameter("mppi.lateral_probe_step", rclcpp::ParameterValue(0.15));
  // Extra probes along the route direction so an obstacle that only blocks the
  // front of the footprint is still seen by the point-cloud probe band.
  declare_parameter("mppi.lateral_probe_longitudinal_offset", rclcpp::ParameterValue(0.7));
  declare_parameter("mppi.lateral_probe_longitudinal_step", rclcpp::ParameterValue(0.35));
  declare_parameter("mppi.debug_blocked_diagnostics", rclcpp::ParameterValue(true));
  declare_parameter("mppi.debug_blocked_diagnostics_period", rclcpp::ParameterValue(0.1));
  declare_parameter("mppi.critics", rclcpp::ParameterValue(mppi_critic_config.critics));

  auto get_double_param = [&](const std::string & param_name, const double default_value) {
    declare_parameter(param_name, rclcpp::ParameterValue(default_value));
    return this->get_parameter(param_name).as_double();
  };
  auto get_int_param = [&](const std::string & param_name, const int default_value) {
    declare_parameter(param_name, rclcpp::ParameterValue(default_value));
    return static_cast<int>(this->get_parameter(param_name).as_int());
  };
  auto get_bool_param = [&](const std::string & param_name, const bool default_value) {
    declare_parameter(param_name, rclcpp::ParameterValue(default_value));
    return this->get_parameter(param_name).as_bool();
  };

  // Declared here so the value is visible even before the first optimization.
  this->declare_parameter("mppi.evaluation_distance", rclcpp::ParameterValue(6.5));
  this->declare_parameter("mppi.evaluation_step", rclcpp::ParameterValue(0.15));
  this->declare_parameter("mppi.evaluation_max_points", rclcpp::ParameterValue(64));
  mppi_config.evaluation_distance =
    std::max(0.0, this->get_parameter("mppi.evaluation_distance").as_double());
  mppi_config.evaluation_step =
    std::max(0.02, this->get_parameter("mppi.evaluation_step").as_double());
  mppi_config.evaluation_max_points = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("mppi.evaluation_max_points").as_int()));
  RCLCPP_INFO(
    this->get_logger().get_child(name_),
    "mppi distance probe: range=%.2f m step=%.2f m max_points=%zu",
    mppi_config.evaluation_distance, mppi_config.evaluation_step,
    mppi_config.evaluation_max_points);

  this->get_parameter("mppi.prediction_horizon_sec", mppi_prediction_horizon_sec);
  this->get_parameter("mppi.model_dt", mppi_config.model_dt);
  mppi_config.controller_period = 1.0 / std::max(1e-3, controller_frequency_);
  mppi_config.time_steps = static_cast<std::size_t>(
    std::max(1.0, std::ceil(mppi_prediction_horizon_sec / std::max(1e-3, mppi_config.model_dt))));
  mppi_config.batch_size = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("mppi.batch_size").as_int()));
  mppi_config.structured_candidate_count = static_cast<std::size_t>(
    std::max<int64_t>(0, this->get_parameter("mppi.structured_candidate_count").as_int()));
  mppi_config.structured_min_vx =
    this->get_parameter("mppi.structured_min_vx").as_double();
  mppi_config.departure_lateral_offset =
    this->get_parameter("mppi.departure_lateral_offset").as_double();
  mppi_config.bypass_detour_weight =
    std::max(0.0, this->get_parameter("mppi.bypass_detour_weight").as_double());
  mppi_config.iteration_count = static_cast<std::size_t>(
    std::max<int64_t>(1, this->get_parameter("mppi.iteration_count").as_int()));
  mppi_config.temperature = this->get_parameter("mppi.temperature").as_double();
  mppi_config.gamma = this->get_parameter("mppi.gamma").as_double();
  mppi_config.vx_std = this->get_parameter("mppi.vx_std").as_double();
  mppi_config.wz_std = this->get_parameter("mppi.wz_std").as_double();
  mppi_config.vx_min = this->get_parameter("mppi.vx_min").as_double();
  mppi_config.vx_max = this->get_parameter("mppi.vx_max").as_double();
  mppi_config.reverse_recovery_enabled =
    this->get_parameter("mppi.reverse_recovery_enabled").as_bool();
  mppi_config.reverse_max_speed =
    this->get_parameter("mppi.reverse_max_speed").as_double();
  mppi_config.reverse_episode_distance =
    this->get_parameter("mppi.reverse_episode_distance").as_double();
  RCLCPP_INFO(this->get_logger().get_child(name_),
    "mppi.reverse_episode_distance is deprecated and ignored; directed retreat has no accumulated distance cap");
  mppi_config.wz_max = this->get_parameter("mppi.wz_max").as_double();
  mppi_config.ax_min = this->get_parameter("mppi.ax_min").as_double();
  mppi_config.ax_max = this->get_parameter("mppi.ax_max").as_double();
  mppi_config.az_max = this->get_parameter("mppi.az_max").as_double();
  mppi_config.wheelbase = this->get_parameter("mppi.wheelbase").as_double();
  mppi_config.max_steer = this->get_parameter("mppi.max_steer").as_double();
  mppi_config.seed = static_cast<std::uint32_t>(
    std::max<int64_t>(0, this->get_parameter("mppi.seed").as_int()));
  mppi_config.regenerate_noises =
    this->get_parameter("mppi.regenerate_noises").as_bool();
  mppi_use_perception_blocked_preemption_ =
    this->get_parameter("mppi.use_perception_blocked_preemption").as_bool();
  mppi_bypass_geometry_enabled_ =
    this->get_parameter("mppi.avoidance_geometry_enabled").as_bool();
  mppi_preferred_side_min_offset_ =
    std::max(0.0, this->get_parameter("mppi.preferred_side_min_offset").as_double());
  mppi_bypass_width_margin_ =
    std::max(0.0, this->get_parameter("mppi.bypass_width_margin").as_double());
  mppi_goal_slowdown_distance_ =
    std::max(0.0, this->get_parameter("mppi.goal_slowdown_distance").as_double());
  mppi_lateral_profile_config_.max_lateral_offset =
    std::max(0.1, this->get_parameter("mppi.lateral_probe_max_offset").as_double());
  mppi_lateral_profile_config_.lateral_step =
    std::max(0.01, this->get_parameter("mppi.lateral_probe_step").as_double());
  mppi_lateral_profile_config_.max_longitudinal_offset =
    std::max(0.0, this->get_parameter("mppi.lateral_probe_longitudinal_offset").as_double());
  mppi_lateral_profile_config_.longitudinal_step =
    std::max(0.01, this->get_parameter("mppi.lateral_probe_longitudinal_step").as_double());
  mppi_debug_blocked_diagnostics_ =
    this->get_parameter("mppi.debug_blocked_diagnostics").as_bool();
  mppi_debug_blocked_diagnostics_period_ =
    std::max(0.0, this->get_parameter("mppi.debug_blocked_diagnostics_period").as_double());
  mppi_config.shift_control_sequence =
    std::fabs(mppi_config.controller_period - mppi_config.model_dt) < 1e-6;
  mppi_controller_.setConfig(mppi_config);
  mppi_last_command_ = MppiControl{std::max(0.0, mppi_config.vx_min), 0.0};

  this->get_parameter("mppi.critics", mppi_critic_config.critics);
  mppi_critic_config.constraint.cost_weight =
    get_double_param("mppi.ConstraintCritic.cost_weight", mppi_critic_config.constraint.cost_weight);
  mppi_critic_config.constraint.cost_power =
    get_int_param("mppi.ConstraintCritic.cost_power", mppi_critic_config.constraint.cost_power);

  mppi_critic_config.obstacles.cost_power =
    get_int_param("mppi.ObstaclesCritic.cost_power", mppi_critic_config.obstacles.cost_power);
  mppi_critic_config.obstacles.critical_weight =
    get_double_param("mppi.ObstaclesCritic.critical_weight", mppi_critic_config.obstacles.critical_weight);
  mppi_critic_config.obstacles.repulsion_weight =
    get_double_param("mppi.ObstaclesCritic.repulsion_weight", mppi_critic_config.obstacles.repulsion_weight);
  mppi_critic_config.obstacles.collision_cost =
    get_double_param("mppi.ObstaclesCritic.collision_cost", mppi_critic_config.obstacles.collision_cost);
  mppi_critic_config.obstacles.collision_margin_distance =
    get_double_param("mppi.ObstaclesCritic.collision_margin_distance", mppi_critic_config.obstacles.collision_margin_distance);
  mppi_critic_config.obstacles.near_goal_distance =
    get_double_param("mppi.ObstaclesCritic.near_goal_distance", mppi_critic_config.obstacles.near_goal_distance);
  mppi_critic_config.obstacles.inflation_radius =
    get_double_param("mppi.ObstaclesCritic.inflation_radius", mppi_critic_config.obstacles.inflation_radius);
  mppi_critic_config.obstacles.trajectory_point_step = static_cast<std::size_t>(
    std::max(1, get_int_param("mppi.ObstaclesCritic.trajectory_point_step", static_cast<int>(mppi_critic_config.obstacles.trajectory_point_step))));

  mppi_critic_config.goal.cost_weight =
    get_double_param("mppi.GoalCritic.cost_weight", mppi_critic_config.goal.cost_weight);
  mppi_critic_config.goal.cost_power =
    get_int_param("mppi.GoalCritic.cost_power", mppi_critic_config.goal.cost_power);
  mppi_critic_config.goal.threshold_to_consider =
    get_double_param("mppi.GoalCritic.threshold_to_consider", mppi_critic_config.goal.threshold_to_consider);

  mppi_critic_config.goal_angle.cost_weight =
    get_double_param("mppi.GoalAngleCritic.cost_weight", mppi_critic_config.goal_angle.cost_weight);
  mppi_critic_config.goal_angle.cost_power =
    get_int_param("mppi.GoalAngleCritic.cost_power", mppi_critic_config.goal_angle.cost_power);
  mppi_critic_config.goal_angle.threshold_to_consider =
    get_double_param("mppi.GoalAngleCritic.threshold_to_consider", mppi_critic_config.goal_angle.threshold_to_consider);
  mppi_critic_config.goal_angle.symmetric_yaw_tolerance =
    get_bool_param("mppi.GoalAngleCritic.symmetric_yaw_tolerance", mppi_critic_config.goal_angle.symmetric_yaw_tolerance);

  mppi_critic_config.path_align.cost_weight =
    get_double_param("mppi.PathAlignCritic.cost_weight", mppi_critic_config.path_align.cost_weight);
  mppi_critic_config.path_align.cost_power =
    get_int_param("mppi.PathAlignCritic.cost_power", mppi_critic_config.path_align.cost_power);
  mppi_critic_config.path_align.threshold_to_consider =
    get_double_param("mppi.PathAlignCritic.threshold_to_consider", mppi_critic_config.path_align.threshold_to_consider);
  mppi_critic_config.path_align.max_path_occupancy_ratio =
    get_double_param("mppi.PathAlignCritic.max_path_occupancy_ratio", mppi_critic_config.path_align.max_path_occupancy_ratio);
  mppi_critic_config.path_align.offset_from_furthest = static_cast<std::size_t>(
    std::max(0, get_int_param("mppi.PathAlignCritic.offset_from_furthest", static_cast<int>(mppi_critic_config.path_align.offset_from_furthest))));
  mppi_critic_config.path_align.trajectory_point_step = static_cast<std::size_t>(
    std::max(1, get_int_param("mppi.PathAlignCritic.trajectory_point_step", static_cast<int>(mppi_critic_config.path_align.trajectory_point_step))));
  mppi_critic_config.path_align.use_path_orientations =
    get_bool_param("mppi.PathAlignCritic.use_path_orientations", mppi_critic_config.path_align.use_path_orientations);

  mppi_critic_config.path_follow.cost_weight =
    get_double_param("mppi.PathFollowCritic.cost_weight", mppi_critic_config.path_follow.cost_weight);
  mppi_critic_config.path_follow.cost_power =
    get_int_param("mppi.PathFollowCritic.cost_power", mppi_critic_config.path_follow.cost_power);
  mppi_critic_config.path_follow.threshold_to_consider =
    get_double_param("mppi.PathFollowCritic.threshold_to_consider", mppi_critic_config.path_follow.threshold_to_consider);
  mppi_critic_config.path_follow.offset_from_furthest = static_cast<std::size_t>(
    std::max(0, get_int_param("mppi.PathFollowCritic.offset_from_furthest", static_cast<int>(mppi_critic_config.path_follow.offset_from_furthest))));

  mppi_critic_config.path_angle.cost_weight =
    get_double_param("mppi.PathAngleCritic.cost_weight", mppi_critic_config.path_angle.cost_weight);
  mppi_critic_config.path_angle.cost_power =
    get_int_param("mppi.PathAngleCritic.cost_power", mppi_critic_config.path_angle.cost_power);
  mppi_critic_config.path_angle.threshold_to_consider =
    get_double_param("mppi.PathAngleCritic.threshold_to_consider", mppi_critic_config.path_angle.threshold_to_consider);
  mppi_critic_config.path_angle.max_angle_to_furthest =
    get_double_param("mppi.PathAngleCritic.max_angle_to_furthest", mppi_critic_config.path_angle.max_angle_to_furthest);
  mppi_critic_config.path_angle.offset_from_furthest = static_cast<std::size_t>(
    std::max(0, get_int_param("mppi.PathAngleCritic.offset_from_furthest", static_cast<int>(mppi_critic_config.path_angle.offset_from_furthest))));
  mppi_critic_config.path_angle.mode =
    get_int_param("mppi.PathAngleCritic.mode", mppi_critic_config.path_angle.mode);

  mppi_critic_config.prefer_forward.cost_weight =
    get_double_param("mppi.PreferForwardCritic.cost_weight", mppi_critic_config.prefer_forward.cost_weight);
  mppi_critic_config.prefer_forward.cost_power =
    get_int_param("mppi.PreferForwardCritic.cost_power", mppi_critic_config.prefer_forward.cost_power);
  mppi_critic_config.prefer_forward.threshold_to_consider =
    get_double_param("mppi.PreferForwardCritic.threshold_to_consider", mppi_critic_config.prefer_forward.threshold_to_consider);
  mppi_critic_manager_.setConfig(mppi_critic_config);

  if(controller_backend_ == "mppi"){
    RCLCPP_DEBUG(
      this->get_logger(),
      "mppi.batch_size: %zu, time_steps: %zu, model_dt: %.3f, temperature: %.3f, gamma: %.3f",
      mppi_controller_.config().batch_size,
      mppi_controller_.config().time_steps,
      mppi_controller_.config().model_dt,
      mppi_controller_.config().temperature,
      mppi_controller_.config().gamma);
  }

  declare_parameter("debug_publish.robot_cuboid", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.robot_cuboid", debug_publish_robot_cuboid_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.robot_cuboid: %d", debug_publish_robot_cuboid_);

  declare_parameter("debug_publish.aggregated_pc", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.aggregated_pc", debug_publish_aggregated_pc_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.aggregated_pc: %d", debug_publish_aggregated_pc_);

  declare_parameter("debug_publish.prune_plan", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.prune_plan", debug_publish_prune_plan_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.prune_plan: %d", debug_publish_prune_plan_);

  declare_parameter("debug_publish.accepted_trajectory", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.accepted_trajectory", debug_publish_accepted_trajectory_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.accepted_trajectory: %d", debug_publish_accepted_trajectory_);

  declare_parameter("debug_publish.best_trajectory", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.best_trajectory", debug_publish_best_trajectory_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.best_trajectory: %d", debug_publish_best_trajectory_);

  declare_parameter("debug_publish.all_trajectories", rclcpp::ParameterValue(false));
  this->get_parameter("debug_publish.all_trajectories", debug_publish_all_trajectories_);
  RCLCPP_DEBUG(this->get_logger(), "debug_publish.all_trajectories: %d", debug_publish_all_trajectories_);

  // Independent wall-timer rate for the diagnostic aggregated_pc topic, in Hz.
  // It is deliberately not tied to the controller frequency: the perception
  // cache refreshes on its own sensor timer, so publishing faster only
  // re-serializes identical data. Values <= 0 disable the timer entirely.
  declare_parameter("debug_publish.aggregated_pc_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("debug_publish.aggregated_pc_frequency", debug_publish_aggregated_pc_frequency_);
  RCLCPP_DEBUG(
    this->get_logger(), "debug_publish.aggregated_pc_frequency: %.2f",
    debug_publish_aggregated_pc_frequency_);


  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;

  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  parseCuboid(); //after robot_frame is got

  // The local-map environment backend is strictly opt-in. Construct the
  // adapter only for controller_backend=mppi plus
  // mppi.environment_backend=local_map, and fail closed if the map or measured
  // vehicle geometry is invalid. Never fall back to the legacy point cloud.
  if(isMppiLocalMapBackendSelected() && mppi_local_map_init_error_.empty()){
    const auto load_status = dddmr_local_map::ros::loadAdapterConfig(
      *this, "local_map.", &mppi_local_map_config_);
    if(!load_status.ok){
      mppi_local_map_init_error_ =
        "invalid local_map parameters: " + load_status.error;
    }
    else{
      const auto adapter_status =
        dddmr_local_map::ros::validateAdapterConfig(mppi_local_map_config_);
      const auto geometry_status = mppi_local_map_config_.geometry.validate();
      if(!adapter_status.ok){
        mppi_local_map_init_error_ =
          "invalid local_map adapter configuration: " + adapter_status.error;
      }
      else if(!mppi_local_map_config_.geometry_status.ok ||
              !geometry_status.ok)
      {
        mppi_local_map_init_error_ =
          "local_map backend requires validated vehicle geometry: " +
          (mppi_local_map_config_.geometry_status.ok ?
            geometry_status.error : mppi_local_map_config_.geometry_status.error);
      }
      else{
        try{
          mppi_local_map_adapter_ =
            std::make_unique<dddmr_local_map::ros::LocalMapRosAdapter>(
              *this, *tf2Buffer_, mppi_local_map_config_);
        }
        catch(const std::exception & exception){
          mppi_local_map_init_error_ =
            std::string("failed to start local_map adapter: ") + exception.what();
        }
      }
    }
    if(!mppi_local_map_init_error_.empty()){
      RCLCPP_ERROR(
        this->get_logger().get_child(name_),
        "MPPI local-map backend unavailable, commands will be stopped: %s",
        mppi_local_map_init_error_.c_str());
    }
    else{
      RCLCPP_INFO(
        this->get_logger().get_child(name_),
        "MPPI local-map backend enabled: cloud=%s map_frame=%s base_frame=%s sensor_frame=%s",
        mppi_local_map_config_.cloud_topic.c_str(),
        mppi_local_map_config_.map_frame.c_str(),
        mppi_local_map_config_.base_frame.c_str(),
        mppi_local_map_config_.sensor_frame.c_str());
    }
  }
  
  const auto debug_path_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  pub_robot_cuboid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_cuboid", 1);  
  pub_aggregate_observation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregated_pc", 1);  
  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>("prune_plan", 1);
  pub_route_sent_to_local_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/route_sent_to_local", debug_path_qos);
  pub_local_pruned_path_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/local_pruned_path", debug_path_qos);
  pub_best_trajectory_path_ =
    this->create_publisher<nav_msgs::msg::Path>("/debug/best_trajectory", debug_path_qos);
  pub_best_trajectory_markers_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/debug/best_trajectory_markers", debug_path_qos);
  pub_best_trajectory_probe_pose_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/best_trajectory_probe", debug_path_qos);
  pub_accepted_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("accepted_trajectory", 1);
  pub_best_trajectory_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>("best_trajectory", 2);
  pub_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("trajectory", 2);
  // Diagnostic cloud publishing runs in its own callback group so a slow
  // conversion never sits in the control callback.
  if(debug_publish_aggregated_pc_ && debug_publish_aggregated_pc_frequency_ > 0.0){
    aggregate_cloud_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    aggregate_cloud_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / debug_publish_aggregated_pc_frequency_),
      std::bind(&Local_Planner::publishAggregatedCloud, this),
      aggregate_cloud_callback_group_);
  }


  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  if(odom_topic_qos_=="reliable" || odom_topic_qos_=="Reliable"){
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  else{
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  
  //@Initial pcl ptr
}

Local_Planner::~Local_Planner(){

  perception_3d_ros_.reset();
  mpc_critics_ros_.reset();
  trajectory_generators_ros_.reset();
  trajectories_.reset();
  mppi_local_map_adapter_.reset();
  tf2Buffer_.reset();

}

void Local_Planner::publishAggregatedCloud(){

  if(!debug_publish_aggregated_pc_ || !pub_aggregate_observation_){
    return;
  }

  if(pub_aggregate_observation_->get_subscription_count() == 0){
    return;
  }

  auto * stacked_perception =
    perception_3d_ros_ ? perception_3d_ros_->getStackedPerception() : nullptr;
  auto perception_shared_data =
    perception_3d_ros_ ? perception_3d_ros_->getSharedDataPtr() : nullptr;

  if(!stacked_perception || !perception_shared_data){
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::ConstPtr aggregate_observation;
  {
    // Only the shared_ptr copy happens under the perception lock; converting
    // and publishing keep running outside it, on this timer's thread.
    std::unique_lock<perception_3d::StackedPerception::mutex_t> lock(
      *(stacked_perception->getMutex()));
    aggregate_observation = perception_shared_data->aggregate_observation_;
  }

  // The perception cache allocates a new cloud on every refresh, so an
  // unchanged pointer means no new sensor data arrived since the last publish.
  if(!aggregate_cloud_cache_.shouldPublish(aggregate_observation)){
    return;
  }

  sensor_msgs::msg::PointCloud2 ros2_aggregate_observation;
  pcl::toROSMsg(*aggregate_observation, ros2_aggregate_observation);
  pub_aggregate_observation_->publish(ros2_aggregate_observation);
  aggregate_cloud_cache_.remember(aggregate_observation);

}

bool Local_Planner::isMppiLocalMapBackendSelected() const
{
  return controller_backend_ == "mppi" &&
         mppi_environment_backend_ == MppiEnvironmentBackend::kLocalMap &&
         mppi_local_map_init_error_.empty();
}

nav_msgs::msg::Path Local_Planner::buildPathFromPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(poses.size());
  for(const auto & pose : poses){
    geometry_msgs::msg::PoseStamped pose_stamped = pose;
    pose_stamped.header = path.header;
    path.poses.push_back(pose_stamped);
  }
  return path;
}

nav_msgs::msg::Path Local_Planner::buildPathFromTrajectory(
  const base_trajectory::Trajectory & traj) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(traj.getPointsSize());
  for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = traj.getPoint(i).pose;
    path.poses.push_back(pose_stamped);
  }
  return path;
}

void Local_Planner::publishDebugPath(
  const nav_msgs::msg::Path & path,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
  const std::string & stage_label,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label,
  bool throttle_log) const
{
  if(!publisher){
    return;
  }
  if(!lp::shouldPublishDebug(true, publisher->get_subscription_count())){
    return;
  }

  nav_msgs::msg::Path output = path;
  output.header.frame_id = global_frame_;
  output.header.stamp = clock_->now();
  for(auto & pose : output.poses){
    pose.header = output.header;
  }
  publisher->publish(output);

  if(throttle_log){
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "%s published, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, poses=%zu",
      stage_label.c_str(),
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_,
      output.poses.size());
  }
  else{
    RCLCPP_DEBUG(
      this->get_logger().get_child(name_),
      "%s published, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, poses=%zu",
      stage_label.c_str(),
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_,
      output.poses.size());
  }
}

dddmr_sys_core::PlannerState Local_Planner::prepareControllerContext(
  ControllerContext * context)
{
  if(context == nullptr){
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  auto * stacked_perception = perception_3d_ros_ ? perception_3d_ros_->getStackedPerception() : nullptr;
  auto perception_shared_data = perception_3d_ros_ ? perception_3d_ros_->getSharedDataPtr() : nullptr;

  if(!got_odom_){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "local planner trigger: state=TF_FAIL, action=stop_wait, reason=odom_not_received, route_version=%zu, goal_seq=%zu, source=%s",
      route_version_,
      goal_seq_,
      route_source_label_.c_str());
    return dddmr_sys_core::TF_FAIL;
  }

  if(!stacked_perception || !perception_shared_data){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "local planner trigger: state=PERCEPTION_MALFUNCTION, action=stop_wait, reason=perception_shared_data_unavailable, route_version=%zu, goal_seq=%zu, source=%s",
      route_version_,
      goal_seq_,
      route_source_label_.c_str());
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }

  if(!isMppiLocalMapBackendSelected() && !stacked_perception->isSensorOK()){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "local planner trigger: state=PERCEPTION_MALFUNCTION, action=stop_wait, reason=perception_sensor_not_ok, route_version=%zu, goal_seq=%zu, source=%s",
      route_version_,
      goal_seq_,
      route_source_label_.c_str());
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }
  if(isMppiLocalMapBackendSelected() && !stacked_perception->isSensorOK()){
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI local-map backend bypasses the legacy nonground sensor-health gate; map bundle health is checked per cycle");
  }

  control_loop_time_ = clock_->now();
  bool have_usable_prune_plan = false;

  {
    std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(stacked_perception->getMutex()));
    const bool aggregate_observation_stale =
      perception_shared_data->aggregate_observation_stale_;
    if(!isMppiLocalMapBackendSelected() && aggregate_observation_stale){
      RCLCPP_ERROR(
        this->get_logger().get_child(name_),
        "local planner trigger: state=PERCEPTION_MALFUNCTION, action=stop_wait, reason=perception_cache_stale, route_version=%zu, goal_seq=%zu, source=%s",
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::PERCEPTION_MALFUNCTION;
    }
    have_usable_prune_plan = prunePlan(forward_prune_, backward_prune_);
    perception_shared_data->pcl_prune_plan_ = pcl_prune_plan_;
    context->aggregate_observation = perception_shared_data->aggregate_observation_;
    context->aggregate_observation_kdtree = perception_shared_data->aggregate_observation_kdtree_;
    context->current_allowed_max_linear_speed = perception_shared_data->current_allowed_max_linear_speed_;
    context->opinions = stacked_perception->getOpinions();
  }

  if(!context->aggregate_observation){
    context->aggregate_observation = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "local planner trigger: state=TF_FAIL, action=stop_wait, reason=tf_out_of_date, route_version=%zu, goal_seq=%zu, source=%s, tf_age=%.2f",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      (clock_->now()-trans_gbl2b_.header.stamp).seconds());
    return dddmr_sys_core::TF_FAIL;
  }

  if(!have_usable_prune_plan){
    const bool deviation_timeout =
      (clock_->now() - last_valid_prune_plan_).seconds() >= prune_plane_timeout_;
    const bool deviation_distance =
      last_robot_to_route_distance_ > causal_prune_max_lateral_distance_;
    const bool deviation_confirmed =
      consecutive_prune_failure_cycles_ > max_prune_failure_cycles_ &&
      deviation_timeout &&
      deviation_distance;

    if(deviation_confirmed){
      RCLCPP_WARN(
        this->get_logger().get_child(name_),
        "local planner trigger: state=PRUNE_PLAN_FAIL, action=route_replan, reason=route_deviation_confirmed, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f, tolerance=%.2f",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        consecutive_prune_failure_cycles_,
        last_robot_to_route_distance_,
        causal_prune_max_lateral_distance_);
      return dddmr_sys_core::PRUNE_PLAN_FAIL;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "local planner trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=transient_prune_failure, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      consecutive_prune_failure_cycles_,
      last_robot_to_route_distance_);
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::evaluatePerceptionOpinions(
  const std::vector<perception_3d::PerceptionOpinion> & opinions) const
{
  for(auto opinion_it = opinions.begin(); opinion_it != opinions.end(); ++opinion_it){
    if((*opinion_it) == perception_3d::PATH_BLOCKED_WAIT){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 5000,
        "local planner trigger: state=PATH_BLOCKED_WAIT, action=blocked_wait, reason=perception_opinion_path_blocked_wait, route_version=%zu, goal_seq=%zu, source=%s",
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    if((*opinion_it) == perception_3d::PATH_BLOCKED_REPLANNING){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 5000,
        "local planner trigger: state=PATH_BLOCKED_REPLANNING, action=route_replan, reason=perception_opinion_path_blocked_replanning, route_version=%zu, goal_seq=%zu, source=%s",
        route_version_,
        goal_seq_,
        route_source_label_.c_str());
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }
  }
  return dddmr_sys_core::TRAJECTORY_FOUND;
}

void Local_Planner::updateCriticSharedData(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & aggregate_observation,
  const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr & aggregate_observation_kdtree)
{
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = aggregate_observation;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_ = aggregate_observation_kdtree;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  mpc_critics_ros_->updateSharedData();
}

bool Local_Planner::selectLookaheadPose(
  double lookahead_distance,
  geometry_msgs::msg::PoseStamped * lookahead_pose) const
{
  if(lookahead_pose == nullptr || prune_plan_.poses.empty()){
    return false;
  }

  const double robot_x = trans_gbl2b_.transform.translation.x;
  const double robot_y = trans_gbl2b_.transform.translation.y;
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();

  for(std::size_t i = 0; i < prune_plan_.poses.size(); ++i){
    const double dx = prune_plan_.poses[i].pose.position.x - robot_x;
    const double dy = prune_plan_.poses[i].pose.position.y - robot_y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    if(distance < closest_distance){
      closest_distance = distance;
      closest_index = i;
    }
  }

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped previous_pose = prune_plan_.poses[closest_index];
  for(std::size_t i = closest_index; i < prune_plan_.poses.size(); ++i){
    if(i > closest_index){
      accumulated_distance += getDistanceBTWPoseStamp(previous_pose, prune_plan_.poses[i]);
      previous_pose = prune_plan_.poses[i];
    }

    if(accumulated_distance + 1e-6 < lookahead_distance && i + 1 < prune_plan_.poses.size()){
      continue;
    }

    const double robot_yaw = getRobotYaw();
    const double dx = prune_plan_.poses[i].pose.position.x - robot_x;
    const double dy = prune_plan_.poses[i].pose.position.y - robot_y;
    const double x_local =
      std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    if(x_local > 0.05 || i + 1 == prune_plan_.poses.size()){
      *lookahead_pose = prune_plan_.poses[i];
      return true;
    }
  }

  *lookahead_pose = prune_plan_.poses.back();
  return true;
}

bool Local_Planner::buildGoalAlignmentReference(tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr || global_plan_.empty()){
    return false;
  }

  return buildHeadingReferenceFromPoseOrientation(global_plan_.back(), reference_pose);
}

double Local_Planner::getDistanceToGoal() const
{
  if(global_plan_.empty()){
    return std::numeric_limits<double>::infinity();
  }

  const auto & goal_pose = global_plan_.back();
  const double dx = trans_gbl2b_.transform.translation.x - goal_pose.pose.position.x;
  const double dy = trans_gbl2b_.transform.translation.y - goal_pose.pose.position.y;
  const double dz = trans_gbl2b_.transform.translation.z - goal_pose.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Local_Planner::getMaxAckermannCurvature() const
{
  return std::tan(rpp_max_steer_) / std::max(rpp_wheelbase_, 1e-6);
}

pcl::PointCloud<pcl::PointXYZ> Local_Planner::transformRobotCuboid(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  // The local-map backend scores the configured rigid-body AABB
  // (geometry.body_min/body_max), not the legacy visualization cuboid. Render
  // the same 8 corners in the same order so a pose shows the exact envelope
  // that terrain/model collision used. Legacy mode keeps its vertices
  // untouched.
  std::vector<pcl::PointXYZ> vertices;
  if(isMppiLocalMapBackendSelected()){
    const auto & min = mppi_local_map_config_.geometry.body_min;
    const auto & max = mppi_local_map_config_.geometry.body_max;
    const auto corner = [](double x, double y, double z){
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = z;
      return point;
    };
    // blb, brb, blt, flb, brt, frt, flt, frb (front=+x, left=+y, top=+z).
    vertices = {
      corner(min.x(), max.y(), min.z()),
      corner(min.x(), min.y(), min.z()),
      corner(min.x(), max.y(), max.z()),
      corner(max.x(), max.y(), min.z()),
      corner(min.x(), min.y(), max.z()),
      corner(max.x(), min.y(), max.z()),
      corner(max.x(), max.y(), max.z()),
      corner(max.x(), min.y(), min.z())};
  }
  else{
    vertices = robot_cuboid_vertices_;
  }

  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.reserve(vertices.size());

  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  q.normalize();
  // Full 3-D rotation: ramps and mixed roll/pitch trajectories must use the
  // same oriented rigid body as the MPPI local-map terrain checks.
  const tf2::Matrix3x3 rotation(q);

  for(const auto & vertex : vertices){
    const tf2::Vector3 rotated = rotation * tf2::Vector3(vertex.x, vertex.y, vertex.z);
    pcl::PointXYZ transformed_point;
    transformed_point.x = pose.pose.position.x + rotated.x();
    transformed_point.y = pose.pose.position.y + rotated.y();
    transformed_point.z = pose.pose.position.z + rotated.z();
    cuboid.push_back(transformed_point);
  }

  return cuboid;
}

base_trajectory::cuboid_min_max_t Local_Planner::computeCuboidMinMax(
  const pcl::PointCloud<pcl::PointXYZ> & cuboid) const
{
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
  max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

  for(const auto & point : cuboid.points){
    min_pt.x = std::min(min_pt.x, point.x);
    min_pt.y = std::min(min_pt.y, point.y);
    min_pt.z = std::min(min_pt.z, point.z);
    max_pt.x = std::max(max_pt.x, point.x);
    max_pt.y = std::max(max_pt.y, point.y);
    max_pt.z = std::max(max_pt.z, point.z);
  }

  return {min_pt, max_pt};
}

base_trajectory::Trajectory Local_Planner::buildPredictedTrajectory(
  double linear_velocity,
  double angular_velocity) const
{
  const double time_step = std::max(0.02, rpp_prediction_step_sec_);
  const double horizon = std::max(time_step, rpp_prediction_horizon_sec_);
  const unsigned int steps =
    std::max<unsigned int>(2, static_cast<unsigned int>(std::ceil(horizon / time_step)) + 1);

  base_trajectory::Trajectory trajectory(
    linear_velocity,
    0.0,
    angular_velocity,
    time_step,
    steps);
  trajectory.cost_ = 0.0;

  double x = trans_gbl2b_.transform.translation.x;
  double y = trans_gbl2b_.transform.translation.y;
  double z = trans_gbl2b_.transform.translation.z;
  double yaw = getRobotYaw();

  for(unsigned int i = 0; i < steps; ++i){
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = clock_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);

    auto cuboid = transformRobotCuboid(pose);
    trajectory.addPoint(pose, cuboid, computeCuboidMinMax(cuboid));

    if(i + 1 >= steps){
      break;
    }

    x += linear_velocity * std::cos(yaw + angular_velocity * time_step * 0.5) * time_step;
    y += linear_velocity * std::sin(yaw + angular_velocity * time_step * 0.5) * time_step;
    yaw += angular_velocity * time_step;
  }

  return trajectory;
}

base_trajectory::Trajectory Local_Planner::buildPredictedTrajectoryFromMppiControls(
  const std::vector<MppiControl> & controls) const
{
  const double time_step = std::max(0.02, mppi_controller_.config().model_dt);
  const unsigned int steps = static_cast<unsigned int>(std::max<std::size_t>(1, controls.size()));
  const double initial_vx = controls.empty() ? 0.0 : controls.front().vx;
  const double initial_wz = controls.empty() ? 0.0 : controls.front().wz;

  base_trajectory::Trajectory trajectory(
    initial_vx,
    0.0,
    initial_wz,
    time_step,
    steps);
  trajectory.cost_ = 0.0;

  double x = trans_gbl2b_.transform.translation.x;
  double y = trans_gbl2b_.transform.translation.y;
  double z = trans_gbl2b_.transform.translation.z;
  double yaw = getRobotYaw();

  for(const auto & control : controls){
    x += control.vx * std::cos(yaw + control.wz * time_step * 0.5) * time_step;
    y += control.vx * std::sin(yaw + control.wz * time_step * 0.5) * time_step;
    yaw = angles::normalize_angle(yaw + control.wz * time_step);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = clock_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);

    auto cuboid = transformRobotCuboid(pose);
    trajectory.addPoint(pose, cuboid, computeCuboidMinMax(cuboid));
  }

  return trajectory;
}

MppiScoredTrajectory Local_Planner::buildMppiScoredTrajectory(
  const std::vector<MppiControl> & controls,
  const MppiObstacleSnapshot & obstacle_snapshot,
  const bool check_every_obstacle_point) const
{
  MppiScoredTrajectory trajectory;
  trajectory.controls = controls;
  trajectory.points.reserve(controls.size());

  const double time_step = std::max(0.02, mppi_controller_.config().model_dt);
  double x = trans_gbl2b_.transform.translation.x;
  double y = trans_gbl2b_.transform.translation.y;
  double z = trans_gbl2b_.transform.translation.z;
  double yaw = getRobotYaw();
  const std::size_t obstacle_point_step = check_every_obstacle_point ? 1 :
    std::max<std::size_t>(1, mppi_critic_manager_.config().obstacles.trajectory_point_step);
  std::size_t point_index = 0;
  bool time_channel_collided = false;
  std::vector<int> obstacle_ids;
  std::vector<float> squared_distances;

  for(const auto & control : controls){
    x += control.vx * std::cos(yaw + control.wz * time_step * 0.5) * time_step;
    y += control.vx * std::sin(yaw + control.wz * time_step * 0.5) * time_step;
    yaw = angles::normalize_angle(yaw + control.wz * time_step);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = clock_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    pose.pose.orientation = tf2::toMsg(q);

    const auto cuboid = transformRobotCuboid(pose);
    const auto collision = evaluateMppiObstacleCollisionForCuboid(
      cuboid, obstacle_snapshot, &obstacle_ids);
    if(collision.in_collision){
      time_channel_collided = true;
    }
    double obstacle_clearance = std::numeric_limits<double>::infinity();
    MppiObstacleClearanceResult clearance_result;
    const bool evaluate_clearance =
      point_index % obstacle_point_step == 0 || point_index + 1 == controls.size();
    if(evaluate_clearance){
      if(collision.in_collision){
        obstacle_clearance = 0.0;
      }
      else{
        pcl::PointXYZ pose_center;
        pose_center.x = pose.pose.position.x;
        pose_center.y = pose.pose.position.y;
        pose_center.z = pose.pose.position.z;
        clearance_result = computeAdaptiveObstacleClearanceForCuboid(
          cuboid,
          obstacle_snapshot,
          pose_center,
          mppi_critic_manager_.config().obstacles.inflation_radius,
          &obstacle_ids,
          &squared_distances);
        obstacle_clearance = clearance_result.clearance;
      }
    }
    trajectory.points.push_back(
      MppiTrajectoryPoint{
        x,
        y,
        yaw,
        collision.in_collision,
        obstacle_clearance,
        collision.evaluated,
        collision.node_checks,
        collision.point_checks,
        clearance_result.node_checks,
        clearance_result.point_checks});
    ++point_index;
  }

  // Distance-parameterized collision probe: continue the terminal command as
  // an arc so a creeping candidate still reaches the obstacle its time
  // rollout never approached. Shared with the debug publisher, which appends
  // the same tail to /debug/best_trajectory.
  if(!controls.empty() && !time_channel_collided){
    geometry_msgs::msg::Pose terminal_pose;
    terminal_pose.position.x = x;
    terminal_pose.position.y = y;
    terminal_pose.position.z = z;
    tf2::Quaternion terminal_q;
    terminal_q.setRPY(0.0, 0.0, yaw);
    terminal_pose.orientation = tf2::toMsg(terminal_q);
    trajectory.probe_points = buildMppiDistanceProbe(
      terminal_pose, controls.back(), obstacle_snapshot,
      &trajectory.probe_first_block_m);
  }

  return trajectory;
}

std::vector<MppiTrajectoryPoint> Local_Planner::buildMppiDistanceProbe(
  const geometry_msgs::msg::Pose & terminal_pose,
  const MppiControl & terminal,
  const MppiObstacleSnapshot & obstacle_snapshot,
  double * first_block_m) const
{
  std::vector<MppiTrajectoryPoint> probe;
  if(first_block_m != nullptr){
    *first_block_m = -1.0;
  }
  const auto & probe_config = mppi_controller_.config();
  if(probe_config.evaluation_distance <= 1e-3 ||
    probe_config.evaluation_step <= 1e-3 ||
    !obstacle_snapshot.cloud || obstacle_snapshot.cloud->points.empty())
  {
    return probe;
  }

  double x = terminal_pose.position.x;
  double y = terminal_pose.position.y;
  const double z = terminal_pose.position.z;
  double yaw = tf2::getYaw(terminal_pose.orientation);
  // The arc geometry depends only on the curvature ratio, so the probe stays
  // meaningful at near-zero speed where the time rollout covers almost nothing.
  const double direction = terminal.vx >= 0.0 ? 1.0 : -1.0;
  const double curvature =
    std::fabs(terminal.vx) > 1e-6 ? terminal.wz / terminal.vx : 0.0;
  const double step = std::max(0.02, probe_config.evaluation_step);
  const std::size_t limit = std::max<std::size_t>(
    1, probe_config.evaluation_max_points);
  probe.reserve(std::min(
      limit, static_cast<std::size_t>(
        std::ceil(probe_config.evaluation_distance / step))));

  std::vector<int> obstacle_ids;
  double arc = 0.0;
  while(arc < probe_config.evaluation_distance && probe.size() < limit){
    const double next_arc = std::min(
      arc + step, probe_config.evaluation_distance);
    const double ds_signed = direction * (next_arc - arc);
    arc = next_arc;
    const double dtheta = curvature * ds_signed;
    if(std::fabs(curvature) > 1e-9){
      x += (std::sin(yaw + dtheta) - std::sin(yaw)) / curvature;
      y -= (std::cos(yaw + dtheta) - std::cos(yaw)) / curvature;
    }
    else{
      x += ds_signed * std::cos(yaw);
      y += ds_signed * std::sin(yaw);
    }
    yaw = angles::normalize_angle(yaw + dtheta);

    geometry_msgs::msg::PoseStamped probe_pose;
    probe_pose.header.frame_id = global_frame_;
    probe_pose.header.stamp = clock_->now();
    probe_pose.pose.position.x = x;
    probe_pose.pose.position.y = y;
    probe_pose.pose.position.z = z;
    tf2::Quaternion probe_q;
    probe_q.setRPY(0.0, 0.0, yaw);
    probe_q.normalize();
    probe_pose.pose.orientation = tf2::toMsg(probe_q);

    const auto collision = evaluateMppiObstacleCollisionForCuboid(
      transformRobotCuboid(probe_pose), obstacle_snapshot, &obstacle_ids);
    probe.push_back(MppiTrajectoryPoint{
      x,
      y,
      yaw,
      collision.in_collision,
      std::numeric_limits<double>::infinity(),
      collision.evaluated,
      collision.node_checks,
      collision.point_checks,
      0,
      0});
    if(collision.in_collision){
      if(first_block_m != nullptr){
        *first_block_m = arc;
      }
      break;
    }
  }
  return probe;
}

MppiCriticData Local_Planner::buildMppiCriticData(
  const std::vector<MppiCandidate> & candidates,
  const MppiObstacleSnapshot & obstacle_snapshot) const
{
  MppiCriticData data;
  data.trajectories.reserve(candidates.size());
  for(const auto & candidate : candidates){
    auto trajectory = buildMppiScoredTrajectory(candidate.controls, obstacle_snapshot);
    for(const auto & point : trajectory.points){
      data.obstacle_collision_node_checks += point.collision_node_checks;
      data.obstacle_collision_point_checks += point.collision_point_checks;
      data.obstacle_clearance_node_checks += point.clearance_node_checks;
      data.obstacle_clearance_point_checks += point.clearance_point_checks;
    }
    data.trajectories.push_back(std::move(trajectory));
  }

  data.path.reserve(prune_plan_.poses.size());
  data.local_path_length = 0.0;
  for(std::size_t i = 0; i < prune_plan_.poses.size(); ++i){
    const auto & pose = prune_plan_.poses[i];
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    MppiPathPoint path_point;
    path_point.x = pose.pose.position.x;
    path_point.y = pose.pose.position.y;
    path_point.yaw = yaw;
    path_point.z = pose.pose.position.z;
    data.path.push_back(path_point);
    if(i > 0){
      const auto & previous = prune_plan_.poses[i - 1];
      data.local_path_length += getDistanceBTWPoseStamp(previous, pose);
    }
  }

  if(obstacle_snapshot.cloud && !obstacle_snapshot.cloud->points.empty() &&
     !data.path.empty())
  {
    std::vector<int> ids;
    std::vector<float> squared_distances;
    for(std::size_t path_index = 0; path_index < data.path.size(); ++path_index){
      auto & point = data.path[path_index];
      geometry_msgs::msg::PoseStamped path_pose;
      path_pose.pose.position.x = point.x;
      path_pose.pose.position.y = point.y;
      path_pose.pose.position.z = prune_plan_.poses[path_index].pose.position.z;
      tf2::Quaternion path_orientation;
      path_orientation.setRPY(0.0, 0.0, point.yaw);
      path_orientation.normalize();
      path_pose.pose.orientation = tf2::toMsg(path_orientation);
      const auto path_cuboid = transformRobotCuboid(path_pose);

      const auto path_collision = evaluateMppiObstacleCollisionForCuboid(
        path_cuboid, obstacle_snapshot, &ids);
      pcl::PointXYZ path_center;
      path_center.x = point.x;
      path_center.y = point.y;
      path_center.z = path_pose.pose.position.z;
      const double collision_margin =
        mppi_critic_manager_.config().obstacles.collision_margin_distance;
      MppiObstacleClearanceResult path_clearance_result;
      double path_clearance = 0.0;
      if(!path_collision.in_collision){
        path_clearance_result = computeAdaptiveObstacleClearanceForCuboid(
          path_cuboid,
          obstacle_snapshot,
          path_center,
          collision_margin,
          &ids,
          &squared_distances);
        path_clearance = path_clearance_result.clearance;
      }
      data.obstacle_collision_node_checks += path_collision.node_checks;
      data.obstacle_collision_point_checks += path_collision.point_checks;
      data.obstacle_clearance_node_checks += path_clearance_result.node_checks;
      data.obstacle_clearance_point_checks += path_clearance_result.point_checks;
      point.obstacle_free =
        path_collision.evaluated && !path_collision.in_collision &&
        (!std::isfinite(path_clearance) || path_clearance >= collision_margin);
      if(!point.obstacle_free){
        ++data.blocked_path_points;
      }
    }
    data.path_obstacle_occupancy_ratio =
      static_cast<double>(data.blocked_path_points) / static_cast<double>(data.path.size());
  }

  data.robot_x = trans_gbl2b_.transform.translation.x;
  data.robot_y = trans_gbl2b_.transform.translation.y;
  data.robot_yaw = getRobotYaw();
  const auto robot_path_projection = projectPointOntoMppiPath(
    data.path, data.robot_x, data.robot_y);
  data.robot_path_offset = robot_path_projection.signed_offset;
  data.robot_path_offset_valid = robot_path_projection.valid;
  data.vx_min = mppi_controller_.config().vx_min;
  data.vx_max = mppi_controller_.config().vx_max;
  data.min_turning_radius =
    mppi_controller_.config().wheelbase /
    std::max(1e-6, std::tan(mppi_controller_.config().max_steer));
  data.model_dt = mppi_controller_.config().model_dt;
  return data;
}

MppiResult Local_Planner::computeMppiDirectedRetreat(
  const MppiControl & measured, const std::vector<MppiPathPoint> & path,
  const double runtime_speed_limit,
  const MppiDirectedRetreat::PoseValidator & validator,
  const std::uint64_t environment_id)
{
  if(mppi_retreat_route_version_ != route_version_ ||
     mppi_retreat_goal_seq_ != goal_seq_)
  {
    if(mppi_directed_retreat_.active()){
      mppi_directed_retreat_.invalidate();
    }
    mppi_retreat_route_version_ = route_version_;
    mppi_retreat_goal_seq_ = goal_seq_;
  }
  tf2::Quaternion orientation;
  tf2::fromMsg(trans_gbl2b_.transform.rotation, orientation);
  MppiRetreatPose pose;
  pose.x = trans_gbl2b_.transform.translation.x;
  pose.y = trans_gbl2b_.transform.translation.y;
  pose.z = trans_gbl2b_.transform.translation.z;
  tf2::Matrix3x3(orientation).getRPY(pose.roll, pose.pitch, pose.yaw);
  auto result = mppi_directed_retreat_.compute(
    pose, measured, path, mppi_controller_.config(), runtime_speed_limit, validator,
    environment_id);
  result.reverse_recovery_active = mppi_directed_retreat_.active();
  RCLCPP_WARN_THROTTLE(
    this->get_logger().get_child(name_), *clock_, 2000,
    "MPPI directed retreat: backend=%s, reason=%s, active=%d, reverse=%d, direction=%d, route_points=%zu, validation_calls=%zu, cycle_ms=%.2f, search_restarts=%zu, expansions=%zu, routes_selected=%zu, cmd_vx=%.3f, cmd_wz=%.3f%s",
    mppi_environment_backend_name_.c_str(), mppi_directed_retreat_.reason().c_str(),
    result.reverse_recovery_active ? 1 : 0, result.reverse_motion_active ? 1 : 0,
    mppi_directed_retreat_.direction(), mppi_directed_retreat_.route().size(),
    mppi_directed_retreat_.lastValidationCalls(), mppi_directed_retreat_.lastCycleMs(),
    mppi_directed_retreat_.searchRestartCount(),
    mppi_directed_retreat_.lastSearchExpansions(),
    mppi_directed_retreat_.routesSelectedCount(),
    result.command.vx, result.command.wz,
    mppi_environment_backend_ == MppiEnvironmentBackend::kLegacyPoints ?
      "; legacy validates supplied obstacle points only; unseen space is not certified free" : "");
  return result;
}

void Local_Planner::resetMppiLocalMapSequenceOnFailure(
  const MppiControl & measured_speed)
{
  // A failure only resets the MPPI warm start. The directed route and measured
  // travel telemetry survive ordinary stop/retry cycles.
  mppi_controller_.resetControlSequence(measured_speed);
  mppi_last_command_ = measured_speed;
}

void Local_Planner::updateMppiMeasuredTravelFromOdom(
  const nav_msgs::msg::Odometry & msg)
{
  local_planner::OdometryTravelSample sample;
  sample.time_sec =
    static_cast<double>(msg.header.stamp.sec) +
    1e-9 * static_cast<double>(msg.header.stamp.nanosec);
  sample.x = msg.pose.pose.position.x;
  sample.y = msg.pose.pose.position.y;
  sample.yaw = tf2::getYaw(msg.pose.pose.orientation);
  sample.frame_id = msg.header.frame_id;

  const local_planner::MeasuredTravelIncrement increment =
    mppi_measured_travel_tracker_.update(sample);

  // Keep the odometry baseline current for every backend, but only charge the
  // MPPI reverse episode while this planner is actually driving and that
  // episode is active. This prevents pre-activation or unrelated-controller
  // motion from consuming the reverse budget.
  if(controller_backend_ != "mppi" ||
     !mppi_controller_.config().reverse_recovery_enabled)
  {
    return;
  }
  if(!increment.integrated ||
     !mppi_controller_.reverseRecoveryActive())
  {
    return;
  }

  // Report the real measured displacement exactly once per accepted odometry
  // observation. Zero is reported as zero; it never gets converted into a
  // nominal control-period's worth of synthetic travel.
  mppi_controller_.noteMeasuredReverseTravel(increment.reverse_distance);
  mppi_controller_.noteMeasuredForwardTravel(increment.forward_distance);
}

MppiLocalMapRolloutResult Local_Planner::buildMppiLocalMapRollout(
  const MppiLocalMapCycle & cycle,
  const std::vector<MppiControl> & controls) const
{
  MppiLocalMapRolloutInput input;
  input.controls = controls;
  input.dt = mppi_controller_.config().model_dt;
  input.initial_pose_planner = cycle.initial_pose_planner;
  return rolloutMppiLocalMap(cycle.context, input);
}

bool Local_Planner::acquireMppiLocalMapCycle(
  MppiLocalMapCycle * cycle, std::string * reason)
{
  const auto reject = [reason](const std::string & message) {
    if(reason != nullptr){
      *reason = message;
    }
    return false;
  };
  if(cycle == nullptr){
    return reject("local map cycle output is null");
  }
  if(mppi_local_map_adapter_ == nullptr){
    return reject(mppi_local_map_init_error_.empty() ?
      "local map adapter is not initialized" : mppi_local_map_init_error_);
  }

  const rclcpp::Time now_ros = clock_->now();
  const double now_ros_sec = now_ros.seconds();
  if(mppi_local_map_last_ros_stamp_sec_ > 0.0 &&
     now_ros_sec + mppi_local_map_config_.clock_rewind_tolerance_sec <
       mppi_local_map_last_ros_stamp_sec_)
  {
    mppi_local_map_last_ros_stamp_sec_ = now_ros_sec;
    return reject("ROS clock moved backwards relative to the last MPPI cycle");
  }

  const auto steady_now = std::chrono::steady_clock::now();
  const auto handle = mppi_local_map_adapter_->handleAt(steady_now);
  if(handle == nullptr){
    return reject("local map adapter has no bundle yet");
  }

  MppiLocalMapContext context;
  context.snapshot = handle->snapshot;
  context.terrain = handle->terrain;
  // Observation freshness is relative to this control cycle, not to the
  // timestamp of an older (but still globally usable) snapshot.
  context.query_stamp = now_ros_sec;

  if(global_frame_ == mppi_local_map_config_.map_frame){
    context.planner_to_map = Eigen::Isometry3d::Identity();
  }
  else{
    try{
      // Match the robot pose acquisition time. A localization correction
      // between two "latest" lookups must not move the seed in odom.
      const auto transform = tf2Buffer_->lookupTransform(
        mppi_local_map_config_.map_frame, global_frame_,
        rclcpp::Time(trans_gbl2b_.header.stamp, clock_->get_clock_type()));
      const auto & rotation = transform.transform.rotation;
      const auto & translation = transform.transform.translation;
      Eigen::Quaterniond quaternion(
        rotation.w, rotation.x, rotation.y, rotation.z);
      if(!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9){
        return reject("planner->map TF has an invalid rotation");
      }
      quaternion.normalize();
      context.planner_to_map = Eigen::Isometry3d::Identity();
      context.planner_to_map.linear() = quaternion.toRotationMatrix();
      context.planner_to_map.translation() =
        Eigen::Vector3d(translation.x, translation.y, translation.z);
    }
    catch(const tf2::TransformException & exception){
      return reject(std::string("planner->map TF unavailable: ") +
                    exception.what());
    }
  }

  MppiLocalMapHealthInput health;
  health.context = context;
  health.handle_present = true;
  health.map_healthy = handle->mapHealthy();
  health.control_ready = handle->readyForControl();
  health.measurement_stamp_sec = handle->measurement_stamp.seconds();
  health.now_ros_sec = now_ros_sec;
  health.receive_steady_sec =
    std::chrono::duration<double>(
      handle->receive_steady.time_since_epoch()).count();
  health.now_steady_sec =
    std::chrono::duration<double>(steady_now.time_since_epoch()).count();
  health.max_age_sec = mppi_local_map_config_.max_stamp_age_sec;
  health.future_tolerance_sec = mppi_local_map_config_.future_tolerance_sec;
  health.clock_rewind_tolerance_sec =
    mppi_local_map_config_.clock_rewind_tolerance_sec;
  health.generation = handle->generation;
  const auto health_status = MppiLocalMapHealth::validate(health);
  if(!health_status.ok){
    return reject(std::string("local map health rejected: ") +
                  toString(health_status.status) + ": " + health_status.detail);
  }
  if(handle->status.code != dddmr_local_map::ros::AdapterCode::kOk){
    return reject(std::string("local map adapter status is ") +
                  dddmr_local_map::ros::toString(handle->status.code));
  }

  tf2::Quaternion base_orientation;
  tf2::fromMsg(trans_gbl2b_.transform.rotation, base_orientation);
  base_orientation.normalize();

  cycle->handle = handle;
  cycle->context = context;
  cycle->initial_pose_planner = Eigen::Isometry3d::Identity();
  cycle->initial_pose_planner.linear() =
    Eigen::Quaterniond(
      base_orientation.w(), base_orientation.x(), base_orientation.y(),
      base_orientation.z()).normalized().toRotationMatrix();
  cycle->initial_pose_planner.translation() = Eigen::Vector3d(
    trans_gbl2b_.transform.translation.x,
    trans_gbl2b_.transform.translation.y,
    trans_gbl2b_.transform.translation.z);
  cycle->generation = handle->generation;
  cycle->measurement_stamp_sec = health.measurement_stamp_sec;
  cycle->receive_steady_sec = health.receive_steady_sec;

  mppi_local_map_last_ros_stamp_sec_ = now_ros_sec;
  return true;
}

bool Local_Planner::revalidateMppiLocalMapCycle(
  const MppiLocalMapCycle & cycle, std::string * reason)
{
  const auto reject = [reason](const std::string & message) {
    if(reason != nullptr){
      *reason = message;
    }
    return false;
  };
  if(mppi_local_map_adapter_ == nullptr){
    return reject("local map adapter is not initialized");
  }

  const rclcpp::Time now_ros = clock_->now();
  const double now_ros_sec = now_ros.seconds();
  if(mppi_local_map_last_ros_stamp_sec_ > 0.0 &&
     now_ros_sec + mppi_local_map_config_.clock_rewind_tolerance_sec <
       mppi_local_map_last_ros_stamp_sec_)
  {
    mppi_local_map_last_ros_stamp_sec_ = now_ros_sec;
    return reject("ROS clock moved backwards during MPPI optimization");
  }

  const auto steady_now = std::chrono::steady_clock::now();
  const auto latest = mppi_local_map_adapter_->handleAt(steady_now);
  if(latest == nullptr){
    return reject("local map adapter lost its bundle during optimization");
  }
  if(!latest->mapHealthy() || !latest->readyForControl() ||
     latest->status.code != dddmr_local_map::ros::AdapterCode::kOk)
  {
    return reject(std::string("local map adapter status became unhealthy: ") +
                  dddmr_local_map::ros::toString(latest->status.code) +
                  (latest->status.reason.empty() ?
                    "" : ": " + latest->status.reason));
  }
  if(latest->generation != cycle.generation){
    return reject("local map generation changed during optimization");
  }

  MppiLocalMapHealthInput health;
  health.context = cycle.context;
  health.handle_present = true;
  health.map_healthy = latest->mapHealthy();
  health.control_ready = latest->readyForControl();
  health.measurement_stamp_sec = cycle.measurement_stamp_sec;
  health.now_ros_sec = now_ros_sec;
  health.receive_steady_sec = cycle.receive_steady_sec;
  health.now_steady_sec =
    std::chrono::duration<double>(steady_now.time_since_epoch()).count();
  health.max_age_sec = mppi_local_map_config_.max_stamp_age_sec;
  health.future_tolerance_sec = mppi_local_map_config_.future_tolerance_sec;
  health.clock_rewind_tolerance_sec =
    mppi_local_map_config_.clock_rewind_tolerance_sec;
  health.generation = latest->generation;
  const auto health_status =
    MppiLocalMapHealth::validate(health, true, cycle.generation);
  if(!health_status.ok){
    return reject(std::string("pinned local map bundle rejected after "
                              "optimization: ") +
                  toString(health_status.status) + ": " +
                  health_status.detail);
  }

  // A newer healthy frame may exist now. It is deliberately NOT used to
  // rescore or mix the already validated weighted sequence.
  mppi_local_map_last_ros_stamp_sec_ = now_ros_sec;
  return true;
}

Local_Planner::MppiLocalMapCriticBundle Local_Planner::buildMppiLocalMapCriticData(
  const std::vector<MppiCandidate> & candidates,
  const MppiLocalMapCycle & cycle) const
{
  MppiLocalMapCriticBundle bundle;
  bundle.data.trajectories.reserve(candidates.size());
  bundle.candidate_valid.reserve(candidates.size());
  bundle.candidate_status.reserve(candidates.size());
  bundle.candidate_detail.reserve(candidates.size());

  for(const auto & candidate : candidates){
    const auto rollout = buildMppiLocalMapRollout(cycle, candidate.controls);
    const auto validation = validateMppiLocalMapRollout(
      cycle.context, rollout, candidate.controls.size());
    const bool valid = validation.ok;
    bundle.candidate_valid.push_back(valid);
    bundle.candidate_status.push_back(rollout.status);
    bundle.candidate_detail.push_back(
      validation.ok ? std::string() : validation.detail);

    if(valid){
      bundle.data.obstacle_collision_point_checks +=
        rollout.trajectory.points.size();
      bundle.data.obstacle_clearance_point_checks +=
        rollout.trajectory.points.size();
      bundle.data.trajectories.push_back(rollout.trajectory);
    }
    else{
      MppiScoredTrajectory invalid;
      invalid.controls = candidate.controls;
      bundle.data.trajectories.push_back(std::move(invalid));
    }
  }

  // Path metadata is derived from the pinned new terrain, not from the empty
  // legacy obstacle snapshot. The support height is seeded from the previous
  // projected path pose so the query walks the terrain continuously.
  const double collision_margin =
    mppi_critic_manager_.config().obstacles.collision_margin_distance;
  bundle.data.path.reserve(prune_plan_.poses.size());
  bundle.data.local_path_length = 0.0;
  bundle.data.blocked_path_points = 0;
  double seed_z = prune_plan_.poses.empty() ?
    trans_gbl2b_.transform.translation.z : prune_plan_.poses.front().pose.position.z;
  for(std::size_t i = 0; i < prune_plan_.poses.size(); ++i){
    const auto & pose = prune_plan_.poses[i];
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    q.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    MppiPathPoint path_point;
    path_point.x = pose.pose.position.x;
    path_point.y = pose.pose.position.y;
    path_point.yaw = yaw;
    path_point.obstacle_free = false;

    dddmr_local_map::TerrainQuery query;
    query.stamp = cycle.context.query_stamp;
    query.position = dddmr_local_map::Vec3(path_point.x, path_point.y, seed_z);
    query.roll = roll;
    query.pitch = pitch;
    query.yaw = yaw;
    const auto terrain = cycle.context.terrain->evaluate(query);
    if(terrain.ok()){
      if(std::isfinite(terrain.projected_position.z())){
        seed_z = terrain.projected_position.z();
      }
      path_point.obstacle_free =
        !std::isfinite(terrain.obstacle_clearance) ||
        terrain.obstacle_clearance >= collision_margin;
    }
    path_point.z = seed_z;
    if(!path_point.obstacle_free){
      ++bundle.data.blocked_path_points;
    }

    bundle.data.path.push_back(path_point);
    if(i > 0){
      const auto & previous = prune_plan_.poses[i - 1];
      bundle.data.local_path_length += getDistanceBTWPoseStamp(previous, pose);
    }
  }
  bundle.data.path_obstacle_occupancy_ratio = bundle.data.path.empty() ?
    0.0 : static_cast<double>(bundle.data.blocked_path_points) /
            static_cast<double>(bundle.data.path.size());

  bundle.data.robot_x = trans_gbl2b_.transform.translation.x;
  bundle.data.robot_y = trans_gbl2b_.transform.translation.y;
  bundle.data.robot_yaw = getRobotYaw();
  const auto robot_path_projection = projectPointOntoMppiPath(
    bundle.data.path, bundle.data.robot_x, bundle.data.robot_y);
  bundle.data.robot_path_offset = robot_path_projection.signed_offset;
  bundle.data.robot_path_offset_valid = robot_path_projection.valid;
  bundle.data.vx_min = mppi_controller_.config().vx_min;
  bundle.data.vx_max = mppi_controller_.config().vx_max;
  bundle.data.min_turning_radius =
    mppi_controller_.config().wheelbase /
    std::max(1e-6, std::tan(mppi_controller_.config().max_steer));
  bundle.data.model_dt = mppi_controller_.config().model_dt;
  if(bundle.data.blocked_path_points >= 3){
    bundle.lateral_profile = measureMppiLateralOccupancyForLocalMap(
      bundle.data.path, cycle);
  }
  return bundle;
}

MppiLateralOccupancyProfile Local_Planner::measureMppiLateralOccupancyForLocalMap(
  const std::vector<MppiPathPoint> & path,
  const MppiLocalMapCycle & cycle) const
{
  MppiLateralOccupancyProfile profile;
  if(!mppi_bypass_geometry_enabled_ || path.empty() ||
    cycle.context.terrain == nullptr)
  {
    return profile;
  }
  const double collision_margin =
    mppi_critic_manager_.config().obstacles.collision_margin_distance;
  const double query_stamp = cycle.context.query_stamp;
  const auto probe_blocked = [&](const MppiPathPoint & probe) {
      dddmr_local_map::TerrainQuery query;
      query.stamp = query_stamp;
      query.position = dddmr_local_map::Vec3(probe.x, probe.y, probe.z);
      query.roll = 0.0;
      query.pitch = 0.0;
      query.yaw = probe.yaw;
      const auto terrain = cycle.context.terrain->evaluate(query);
      if(!terrain.ok()){
        // Outside the map is not evidence of free space.
        return true;
      }
      return std::isfinite(terrain.obstacle_clearance) &&
        terrain.obstacle_clearance < collision_margin;
    };
  // Every terrain query already tests the whole rigid body OBB, so the probe
  // at the anchor's longitudinal position covers the footprint along the route
  // as well; extra along-route terrain queries would only add cost.
  MppiLateralProfileConfig profile_config = mppi_lateral_profile_config_;
  profile_config.max_longitudinal_offset = 0.0;
  return measureMppiLateralOccupancyProfile(
    path, probe_blocked, profile_config);
}

MppiLateralOccupancyProfile Local_Planner::measureMppiLateralOccupancyForObstacleCloud(
  const std::vector<MppiPathPoint> & path,
  const MppiObstacleSnapshot & obstacle_snapshot) const
{
  MppiLateralOccupancyProfile profile;
  if(!mppi_bypass_geometry_enabled_ || path.empty() ||
    !obstacle_snapshot.cloud || obstacle_snapshot.cloud->points.empty())
  {
    return profile;
  }
  const double probe_radius = std::max(
    0.05, mppi_critic_manager_.config().obstacles.collision_margin_distance);
  const auto probe_blocked = [&](const MppiPathPoint & probe) {
      pcl::PointXYZI centre;
      centre.x = static_cast<float>(probe.x);
      centre.y = static_cast<float>(probe.y);
      centre.z = static_cast<float>(probe.z);
      centre.intensity = 0.0f;
      return mppiObstacleSnapshotHasPointWithin(
        obstacle_snapshot, centre, probe_radius);
    };
  return measureMppiLateralOccupancyProfile(
    path, probe_blocked, mppi_lateral_profile_config_);
}

MppiBypassSide Local_Planner::preferredMppiBypassSide(
  const MppiLateralOccupancyProfile & profile) const
{
  if(!mppi_bypass_geometry_enabled_){
    return MppiBypassSide::Neutral;
  }
  const double required_width = robot_half_width_ +
    mppi_critic_manager_.config().obstacles.collision_margin_distance +
    mppi_bypass_width_margin_;
  return selectMppiGeometricBypassSide(
    profile, mppi_preferred_side_min_offset_, required_width);
}

double Local_Planner::mppiSpeedCap(const double allowed_max_linear_speed) const
{
  const auto & config = mppi_controller_.config();
  double cruise_cap = config.vx_max;
  if(allowed_max_linear_speed > 0.0){
    cruise_cap = std::min(cruise_cap, allowed_max_linear_speed);
  }
  return mppiGoalApproachSpeedCap(
    getDistanceToGoal(), cruise_cap, mppi_goal_slowdown_distance_, config.vx_min);
}

void Local_Planner::maybeLogMppiBlockedDiagnostics(
  const MppiObstacleSnapshot & obstacle_snapshot,
  const MppiResult & result,
  const MppiControl & best_sample_command,
  const MppiLateralOccupancyProfile & profile,
  const double path_offset,
  const bool path_offset_valid,
  const bool optimal_sequence_collision_free,
  const bool has_optimal_collision_pose,
  const double optimal_collision_x,
  const double optimal_collision_y,
  const double optimal_collision_yaw,
  const std::size_t blocked_path_points)
{
  if(!mppi_debug_blocked_diagnostics_){
    mppi_blocked_diagnostics_state_.clear();
    return;
  }
  if(blocked_path_points < 3 || mppi_diagnostic_path_.size() < 2 ||
    !obstacle_snapshot.cloud || obstacle_snapshot.cloud->points.empty())
  {
    mppi_blocked_diagnostics_state_.clear();
    return;
  }

  // Emit when the decision state changes, and otherwise at the configured
  // period. A fixed 2 s throttle would miss the sub-second infeasibility that
  // abandons a passable side.
  const std::string state =
    std::string(toString(result.selected_bypass_side)) + "/" +
    std::string(toString(result.avoidance_stage)) + "/" +
    (result.feasible_left_candidates > 0 ? "L" : "l") +
    (result.feasible_right_candidates > 0 ? "R" : "r") +
    (result.feasible_neutral_candidates > 0 ? "N" : "n") + "/" +
    (optimal_sequence_collision_free ? "1" : "0") + "/" +
    (result.has_valid_control ? "1" : "0");
  const auto now = std::chrono::steady_clock::now();
  const bool first_trace =
    mppi_blocked_diagnostics_last_.time_since_epoch().count() == 0;
  const bool period_elapsed = first_trace ||
    now - mppi_blocked_diagnostics_last_ >=
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(mppi_debug_blocked_diagnostics_period_));
  if(state == mppi_blocked_diagnostics_state_ &&
    !period_elapsed && !first_trace)
  {
    return;
  }
  mppi_blocked_diagnostics_state_ = state;
  mppi_blocked_diagnostics_last_ = now;

  std::vector<std::size_t> blocked_indices;
  blocked_indices.reserve(mppi_diagnostic_path_.size());
  for(std::size_t i = 0; i < mppi_diagnostic_path_.size(); ++i){
    if(!mppi_diagnostic_path_[i].obstacle_free){
      blocked_indices.push_back(i);
    }
  }
  if(blocked_indices.empty()){
    return;
  }
  std::vector<std::size_t> anchors;
  anchors.push_back(blocked_indices.front());
  if(blocked_indices.size() > 2){
    anchors.push_back(blocked_indices[blocked_indices.size() / 2]);
  }
  if(blocked_indices.size() > 1){
    anchors.push_back(blocked_indices.back());
  }

  const double probe_step =
    std::max(0.01, mppi_lateral_profile_config_.lateral_step);
  const double probe_max =
    std::max(probe_step, mppi_lateral_profile_config_.max_lateral_offset);
  std::vector<int> obstacle_ids;
  std::ostringstream corridor;
  for(const std::size_t anchor_index : anchors){
    const auto & anchor = mppi_diagnostic_path_[anchor_index];
    const double normal_x = -std::sin(anchor.yaw);
    const double normal_y = std::cos(anchor.yaw);
    geometry_msgs::msg::PoseStamped probe_pose;
    probe_pose.pose.position.x = anchor.x;
    probe_pose.pose.position.y = anchor.y;
    probe_pose.pose.position.z = anchor.z;
    tf2::Quaternion probe_orientation;
    probe_orientation.setRPY(0.0, 0.0, anchor.yaw);
    probe_pose.pose.orientation = tf2::toMsg(probe_orientation);
    const auto probe_band = [&](const double lateral) {
        geometry_msgs::msg::PoseStamped pose = probe_pose;
        pose.pose.position.x += normal_x * lateral;
        pose.pose.position.y += normal_y * lateral;
        const auto collision = evaluateMppiObstacleCollisionForCuboid(
          transformRobotCuboid(pose), obstacle_snapshot, &obstacle_ids);
        if(!collision.evaluated){
          return '?';
        }
        return collision.in_collision ? 'B' : 'F';
      };
    std::string left_band;
    std::string right_band;
    for(double offset = probe_step;
      offset <= probe_max + 1e-9; offset += probe_step)
    {
      left_band.push_back(probe_band(offset));
      right_band.push_back(probe_band(-offset));
    }

    // Three nearest returns at this anchor, not just the closest one: the
    // blocking decision is taken from the nearest point, so its height and
    // lateral position are what separates a real obstacle from a stray ground
    // or vegetation return.
    struct NearestPoint
    {
      double squared = std::numeric_limits<double>::infinity();
      double lateral = 0.0;
      double along = 0.0;
      double z = 0.0;
    };
    NearestPoint nearest_points[3];
    std::size_t band_points = 0;
    double band_lateral_min = std::numeric_limits<double>::infinity();
    double band_lateral_max = -std::numeric_limits<double>::infinity();
    double band_z_min = std::numeric_limits<double>::infinity();
    double band_z_max = -std::numeric_limits<double>::infinity();
    const double cos_yaw = std::cos(anchor.yaw);
    const double sin_yaw = std::sin(anchor.yaw);
    for(const auto & point : obstacle_snapshot.cloud->points){
      if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      const double dx = point.x - anchor.x;
      const double dy = point.y - anchor.y;
      const double squared = dx * dx + dy * dy;
      if(squared > 4.0){
        // Only a 2 m neighbourhood is reported; farther points are noise for
        // a corridor-width decision.
        continue;
      }
      const double lateral = normal_x * dx + normal_y * dy;
      const double along = cos_yaw * dx + sin_yaw * dy;
      ++band_points;
      band_lateral_min = std::min(band_lateral_min, lateral);
      band_lateral_max = std::max(band_lateral_max, lateral);
      band_z_min = std::min(band_z_min, static_cast<double>(point.z));
      band_z_max = std::max(band_z_max, static_cast<double>(point.z));
      NearestPoint candidate;
      candidate.squared = squared;
      candidate.lateral = lateral;
      candidate.along = along;
      candidate.z = point.z;
      for(auto & slot : nearest_points){
        if(candidate.squared < slot.squared){
          std::swap(slot, candidate);
        }
      }
    }
    corridor << " [i=" << anchor_index <<
      " L=" << left_band << " R=" << right_band;
    corridor << " near3=[";
    bool first_nearest = true;
    for(const auto & nearest : nearest_points){
      if(!std::isfinite(nearest.squared)){
        continue;
      }
      if(!first_nearest){
        corridor << " ";
      }
      first_nearest = false;
      corridor << "(d=" << std::sqrt(nearest.squared) <<
        ",lat=" << nearest.lateral <<
        ",along=" << nearest.along <<
        ",z=" << nearest.z << ")";
    }
    corridor << "]";
    corridor << " band(n=" << band_points;
    if(band_points > 0){
      corridor << ",lat=[" << band_lateral_min << "," << band_lateral_max <<
        "],z=[" << band_z_min << "," << band_z_max << "]";
    }
    corridor << "]";
  }

  std::ostringstream optimal_collision;
  if(has_optimal_collision_pose){
    double nearest_squared = std::numeric_limits<double>::infinity();
    double nearest_z = 0.0;
    for(const auto & point : obstacle_snapshot.cloud->points){
      if(!std::isfinite(point.x) || !std::isfinite(point.y)){
        continue;
      }
      const double dx = point.x - optimal_collision_x;
      const double dy = point.y - optimal_collision_y;
      const double squared = dx * dx + dy * dy;
      if(squared < nearest_squared){
        nearest_squared = squared;
        nearest_z = point.z;
      }
    }
    optimal_collision << "(x=" << optimal_collision_x <<
      ",y=" << optimal_collision_y << ",yaw=" << optimal_collision_yaw;
    if(std::isfinite(nearest_squared)){
      optimal_collision << ",nearest=" << std::sqrt(nearest_squared) <<
        ",z=" << nearest_z;
    }
    optimal_collision << ")";
  }
  else{
    optimal_collision << "none";
  }

  RCLCPP_INFO(this->get_logger().get_child(name_),
    "MPPI blocked diagnostics: stage=%s, selected=%s, side_failures=%zu, quality_adv=%zu, feasible=L%zu/R%zu/N%zu, profile=(valid=%d,offset=%.3f), path_offset=%.3f(valid=%d), cmd=(%.3f,%.3f), best_sample=(%.3f,%.3f), optimal_valid=%d, valid_control=%d, blocked=%zu, opt_collision=%s, corridor=%s",
    toString(result.avoidance_stage),
    toString(result.selected_bypass_side),
    result.side_infeasible_cycles,
    result.side_quality_advantage_cycles,
    result.feasible_left_candidates,
    result.feasible_right_candidates,
    result.feasible_neutral_candidates,
    profile.valid ? 1 : 0,
    profile.obstacle_offset,
    path_offset,
    path_offset_valid ? 1 : 0,
    result.command.vx,
    result.command.wz,
    best_sample_command.vx,
    best_sample_command.wz,
    optimal_sequence_collision_free ? 1 : 0,
    result.has_valid_control ? 1 : 0,
    blocked_path_points,
    optimal_collision.str().c_str(),
    corridor.str().c_str());
}

base_trajectory::Trajectory Local_Planner::buildTrajectoryFromProjectedMppiPoses(
  const std::vector<Eigen::Isometry3d> & poses,
  const rclcpp::Time & stamp) const
{
  const double time_step = std::max(0.02, mppi_controller_.config().model_dt);
  const unsigned int steps =
    static_cast<unsigned int>(std::max<std::size_t>(1, poses.size()));
  base_trajectory::Trajectory trajectory(0.0, 0.0, 0.0, time_step, steps);
  trajectory.cost_ = 0.0;

  for(const auto & projected_pose : poses){
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = stamp;
    pose.pose.position.x = projected_pose.translation().x();
    pose.pose.position.y = projected_pose.translation().y();
    pose.pose.position.z = projected_pose.translation().z();
    Eigen::Quaterniond orientation(projected_pose.linear());
    orientation.normalize();
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    const auto cuboid = transformRobotCuboid(pose);
    trajectory.addPoint(pose, cuboid, computeCuboidMinMax(cuboid));
  }
  return trajectory;
}

dddmr_sys_core::PlannerState Local_Planner::computeMppiLocalMapControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  const MppiControl measured_speed{
    robot_state_.twist.twist.linear.x,
    robot_state_.twist.twist.angular.z};
  const auto stop = [&](const dddmr_sys_core::PlannerState state,
                        const std::string & reason) {
    resetMppiLocalMapSequenceOnFailure(measured_speed);
    if(cmd_vel != nullptr){
      cmd_vel->linear.x = 0.0;
      cmd_vel->linear.y = 0.0;
      cmd_vel->linear.z = 0.0;
      cmd_vel->angular.x = 0.0;
      cmd_vel->angular.y = 0.0;
      cmd_vel->angular.z = 0.0;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI local-map trigger: state=STOP, action=stop_wait, reason=%s, controller=%s, route_version=%zu, goal_seq=%zu, source=%s",
      reason.c_str(),
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str());
    return state;
  };

  if(!mppi_local_map_init_error_.empty()){
    return stop(dddmr_sys_core::ALL_TRAJECTORIES_FAIL,
                mppi_local_map_init_error_);
  }

  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    resetMppiLocalMapSequenceOnFailure(measured_speed);
    if(cmd_vel != nullptr){
      cmd_vel->linear.x = 0.0;
      cmd_vel->linear.y = 0.0;
      cmd_vel->linear.z = 0.0;
      cmd_vel->angular.x = 0.0;
      cmd_vel->angular.y = 0.0;
      cmd_vel->angular.z = 0.0;
    }
    return context_state;
  }

  // New-backend blocked handling comes from terrain/path metadata. Legacy
  // perception blocked opinions are intentionally not used here; route and
  // speed context above remain mandatory.
  const double mppi_cycle_speed_cap =
    mppiSpeedCap(context.current_allowed_max_linear_speed);
  MppiLocalMapCycle cycle;
  std::string cycle_reason;
  if(!acquireMppiLocalMapCycle(&cycle, &cycle_reason)){
    return stop(dddmr_sys_core::ALL_TRAJECTORIES_FAIL, cycle_reason);
  }

  if(context.current_allowed_max_linear_speed > 0.0 &&
     context.current_allowed_max_linear_speed < mppi_controller_.config().vx_min)
  {
    return stop(
      dddmr_sys_core::ALL_TRAJECTORIES_FAIL,
      "speed cap is below the configured MPPI minimum");
  }

  std::size_t mppi_blocked_path_points = 0;
  double mppi_robot_path_offset = 0.0;
  bool mppi_robot_path_offset_valid = false;
  MppiLateralOccupancyProfile mppi_lateral_profile;
  MppiBypassSide mppi_preferred_bypass_side = MppiBypassSide::Neutral;
  double mppi_left_detour = 0.0;
  double mppi_right_detour = 0.0;
  bool mppi_detour_valid = false;
  std::size_t mppi_valid_scores = 0;
  std::size_t mppi_invalid_terrain_candidates = 0;
  std::size_t mppi_scored_trajectories = 0;

  const auto scoring_start = std::chrono::steady_clock::now();
  auto score_function =
    [&](const std::vector<MppiCandidate> & candidates)
      -> std::vector<MppiCandidateScore> {
      auto bundle = buildMppiLocalMapCriticData(candidates, cycle);
      mppi_scored_trajectories = bundle.data.trajectories.size();
      mppi_blocked_path_points = bundle.data.blocked_path_points;
      mppi_lateral_profile = bundle.lateral_profile;
      mppi_preferred_bypass_side = preferredMppiBypassSide(mppi_lateral_profile);
      mppi_robot_path_offset = bundle.data.robot_path_offset;
      mppi_robot_path_offset_valid = bundle.data.robot_path_offset_valid;
      mppi_left_detour = mppiBypassBandDetour(
        mppi_lateral_profile, MppiBypassSide::Left, mppi_robot_path_offset);
      mppi_right_detour = mppiBypassBandDetour(
        mppi_lateral_profile, MppiBypassSide::Right, mppi_robot_path_offset);
      mppi_detour_valid = mppi_robot_path_offset_valid &&
        std::isfinite(mppi_left_detour) && std::isfinite(mppi_right_detour);
      mppi_invalid_terrain_candidates = 0;
      for(const bool valid : bundle.candidate_valid){
        if(!valid){
          ++mppi_invalid_terrain_candidates;
        }
      }

      auto critic_result = mppi_critic_manager_.score(bundle.data);
      if(critic_result.fail_flag ||
         critic_result.scores.size() != candidates.size())
      {
        return std::vector<MppiCandidateScore>(
          candidates.size(), MppiCandidateScore{false, -1.0});
      }

      if(mppi_cycle_speed_cap > 0.0){
        for(std::size_t i = 0; i < candidates.size(); ++i){
          if(!mppiSequenceRespectsSpeedCap(
              candidates[i].controls,
              measured_speed.vx,
              mppi_cycle_speed_cap,
              mppi_controller_.config().shift_control_sequence,
              mppi_controller_.config().ax_min,
              mppi_controller_.config().model_dt,
              mppi_controller_.config().controller_period,
              mppi_controller_.config().ax_max))
          {
            critic_result.scores[i] = MppiCandidateScore{false, -1.0};
          }
        }
      }

      // Terrain/support invalid rollouts are hard-rejected regardless of
      // whether the optional obstacle critic is enabled or disabled.
      for(std::size_t i = 0; i < candidates.size(); ++i){
        if(!bundle.candidate_valid[i]){
          critic_result.scores[i] = MppiCandidateScore{false, -1.0};
        }
      }
      annotateMppiCandidateMetadata(bundle.data, &critic_result);

      mppi_valid_scores = 0;
      for(const auto & score : critic_result.scores){
        if(score.valid && std::isfinite(score.cost) && score.cost >= 0.0){
          ++mppi_valid_scores;
        }
      }
      return critic_result.scores;
    };

  constexpr std::size_t max_optimization_attempts = 2;
  const MppiAvoidanceState entry_avoidance_state =
    mppi_controller_.avoidanceState();
  MppiResult result;
  MppiLocalMapRolloutResult final_rollout;
  bool final_engine_valid = false;
  std::size_t optimization_attempts = 0;
  for(; optimization_attempts < max_optimization_attempts;
      ++optimization_attempts)
  {
    result = mppi_controller_.optimizeBatch(
      measured_speed,
      score_function,
      [&]() {
        MppiDirectionalContext context{
          mppi_blocked_path_points >= 3,
          mppi_robot_path_offset,
          mppi_robot_path_offset_valid};
        context.preferred_side = mppi_preferred_bypass_side;
        context.left_detour = mppi_left_detour;
        context.right_detour = mppi_right_detour;
        context.detour_valid = mppi_detour_valid;
        return context;
      },
      mppi_cycle_speed_cap,
      [&](bool) {
        const auto metadata = buildMppiLocalMapCriticData({}, cycle);
        const MppiDirectedRetreat::PoseValidator validator =
          [&](MppiRetreatPose & pose) {
            MppiLocalMapRolloutInput input;
            input.controls = {MppiControl{0.0, 0.0}};
            input.dt = mppi_controller_.config().model_dt;
            input.initial_pose_planner = Eigen::Isometry3d::Identity();
            input.initial_pose_planner.linear() =
              (Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
            input.initial_pose_planner.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
            const auto rollout = rolloutMppiLocalMap(cycle.context, input);
            if(!validateMppiLocalMapRollout(cycle.context, rollout, 1).ok){return false;}
            // Propagate the actual terrain projection into the next search
            // primitive, including its support height and full orientation.
            const auto & projected = rollout.projected_start_pose;
            pose.x = projected.translation().x();
            pose.y = projected.translation().y();
            pose.z = projected.translation().z();
            const Eigen::Quaterniond q(projected.linear());
            tf2::Matrix3x3(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(
              pose.roll, pose.pitch, pose.yaw);
            return true;
          };
        // mapVersion is retained as an ABI-compatible argument but the core
        // ignores it: per-voxel freshness also depends on query_stamp/age, so
        // the remaining route is revalidated every update.
        return computeMppiDirectedRetreat(
          measured_speed, metadata.data.path, context.current_allowed_max_linear_speed,
          validator, cycle.context.mapVersion());      });
    if(!result.has_valid_control){
      break;
    }

    final_rollout = buildMppiLocalMapRollout(
      cycle, result.optimal_sequence);
    const auto validation = validateMppiLocalMapRollout(
      cycle.context, final_rollout, result.optimal_sequence.size());
    if(!validation.ok){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "MPPI local-map final rollout rejected, resetting sequence: %s",
        validation.detail.c_str());
      if(mppi_directed_retreat_.active()){
        // A final backend rejection also makes the retained directed route
        // unexecutable; invalidate it into braking instead of only resetting
        // the unrelated MPPI warm start.
        mppi_directed_retreat_.invalidate();
      }
      mppi_controller_.resetControlSequenceForRetry(
        measured_speed, entry_avoidance_state);
      continue;
    }

    if(mppi_cycle_speed_cap > 0.0 &&
       !mppiSequenceRespectsSpeedCap(
         result.optimal_sequence,
         measured_speed.vx,
         mppi_cycle_speed_cap,
         mppi_controller_.config().shift_control_sequence,
         mppi_controller_.config().ax_min,
         mppi_controller_.config().model_dt,
         mppi_controller_.config().controller_period,
         mppi_controller_.config().ax_max))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "MPPI local-map final weighted sequence violates the speed cap, resetting sequence");
      mppi_controller_.resetControlSequenceForRetry(
        measured_speed, entry_avoidance_state);
      continue;
    }

    // Recheck the pinned bundle age plus current adapter status/generation.
    // A newer healthy frame may be present now, but it is not rescored here.
    std::string recheck_reason;
    if(!revalidateMppiLocalMapCycle(cycle, &recheck_reason)){
      return stop(dddmr_sys_core::ALL_TRAJECTORIES_FAIL, recheck_reason);
    }
    final_engine_valid = true;
    ++optimization_attempts;
    break;
  }

  const double scoring_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - scoring_start).count();
  RCLCPP_INFO_THROTTLE(
    this->get_logger().get_child(name_), *clock_, 2000,
    "MPPI local-map debug: controller=%s, environment=local_map, map_version=%llu, generation=%llu, scoring_ms=%.2f, trajectories=%zu, valid_scores=%zu, terrain_invalid=%zu, blocked_path_points=%zu, path_offset=%.3f, preferred_side=%s, obstacle_offset=%.3f, lateral_profile_valid=%d, attempts=%zu, final_valid=%d, best_cost=%.3f, cmd_vx=%.3f, cmd_wz=%.3f, allowed_max_speed=%.3f",
    controller_name.c_str(),
    static_cast<unsigned long long>(cycle.context.mapVersion()),
    static_cast<unsigned long long>(cycle.generation),
    scoring_ms,
    mppi_scored_trajectories,
    mppi_valid_scores,
    mppi_invalid_terrain_candidates,
    mppi_blocked_path_points,
    mppi_robot_path_offset,
    toString(mppi_preferred_bypass_side),
    mppi_lateral_profile.obstacle_offset,
    mppi_lateral_profile.valid ? 1 : 0,
    optimization_attempts,
    final_engine_valid ? 1 : 0,
    result.best_cost,
    result.command.vx,
    result.command.wz,
    context.current_allowed_max_linear_speed);

  if(result.reverse_recovery_active || result.reverse_candidates_generated ||
     result.reverse_budget_exhausted)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI reverse telemetry (distance cap removed): active=%d, motion=%d, generated=%d, used=%.3f, remaining=%.3f, exhausted=%d, cmd_vx=%.3f, cmd_wz=%.3f, measured_vx=%.3f",
      result.reverse_recovery_active ? 1 : 0,
      result.reverse_motion_active ? 1 : 0,
      result.reverse_candidates_generated ? 1 : 0,
      result.reverse_distance_used,
      result.reverse_distance_remaining,
      result.reverse_budget_exhausted ? 1 : 0,
      result.command.vx,
      result.command.wz,
      measured_speed.vx);
  }

  if(!result.has_valid_control){
    if(mppi_directed_retreat_.active()){
      // Recovery still owns the cycle; stop and wait instead of asking the
      // global planner to replace a route that is still being recovered.
      return stop(
        dddmr_sys_core::PATH_BLOCKED_WAIT,
        "directed retreat braking");
    }
    return stop(
      dddmr_sys_core::PATH_BLOCKED_REPLANNING,
      result.reverse_budget_exhausted ?
      "MPPI reverse episode distance exhausted" :
      "all MPPI local-map candidates were rejected");
  }
  if(!final_engine_valid){
    return stop(dddmr_sys_core::PATH_BLOCKED_REPLANNING,
                "no collision-free final weighted sequence in the pinned map");
  }

  mppi_last_command_ = result.command;
  if(cmd_vel != nullptr){
    cmd_vel->linear.x = result.command.vx;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = result.command.wz;
  }

  // The final projected pose vector is the exact source for visualization;
  // no second map query or re-integration is performed.
  const auto best_traj = buildTrajectoryFromProjectedMppiPoses(
    final_rollout.projected_poses, clock_->now());
  publishDebugPath(
    buildPathFromTrajectory(best_traj),
    pub_best_trajectory_path_,
    "mppi_local_map_best_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

  if(debug_publish_best_trajectory_ &&
     pub_best_trajectory_pose_->get_subscription_count() > 0)
  {
    geometry_msgs::msg::PoseArray best_pose_arr;
    trajectory2posearray_cuboids(best_traj, best_pose_arr);
    best_pose_arr.header.frame_id = global_frame_;
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

void Local_Planner::resetLocalRouteTrackingState()
{
  local_route_progress_index_ = 0;
  last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
  consecutive_prune_failure_cycles_ = 0;
  last_prune_used_cache_ = false;
  cached_local_pruned_path_valid_ = false;
  cached_local_pruned_path_.poses.clear();
  cached_local_pruned_path_.header.frame_id = global_frame_;
  cached_local_pruned_path_.header.stamp = clock_->now();
  cached_local_pruned_path_stamp_ = clock_->now();
  cached_pcl_prune_plan_.clear();
  prune_plan_.poses.clear();
  prune_plan_.header.frame_id = global_frame_;
  prune_plan_.header.stamp = clock_->now();
  pcl_prune_plan_.clear();
  global_plan_arc_lengths_.clear();
  last_valid_prune_plan_ = clock_->now();
}

void Local_Planner::rebuildGlobalPlanArcLengths()
{
  global_plan_arc_lengths_.clear();
  global_plan_arc_lengths_.reserve(global_plan_.size());
  double accumulated_distance = 0.0;
  for(std::size_t i = 0; i < global_plan_.size(); ++i){
    if(i > 0){
      accumulated_distance += getDistanceBTWPoseStamp(global_plan_[i - 1], global_plan_[i]);
    }
    global_plan_arc_lengths_.push_back(accumulated_distance);
  }
}

double Local_Planner::getGlobalPlanArcDistance(std::size_t start_index, std::size_t end_index) const
{
  if(global_plan_arc_lengths_.empty()){
    return 0.0;
  }
  start_index = std::min(start_index, global_plan_arc_lengths_.size() - 1);
  end_index = std::min(end_index, global_plan_arc_lengths_.size() - 1);
  const double start_arc = global_plan_arc_lengths_[start_index];
  const double end_arc = global_plan_arc_lengths_[end_index];
  return std::fabs(end_arc - start_arc);
}

double Local_Planner::getRobotYaw() const
{
  tf2::Quaternion q;
  tf2::fromMsg(trans_gbl2b_.transform.rotation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double Local_Planner::getRouteSegmentLateralDistance(std::size_t index) const
{
  if(global_plan_.empty()){
    return std::numeric_limits<double>::infinity();
  }

  index = std::min(index, global_plan_.size() - 1);
  const double robot_x = trans_gbl2b_.transform.translation.x;
  const double robot_y = trans_gbl2b_.transform.translation.y;

  const auto distance_to_point =
    [&](const geometry_msgs::msg::PoseStamped & pose) {
      const double dx = robot_x - pose.pose.position.x;
      const double dy = robot_y - pose.pose.position.y;
      return std::sqrt(dx * dx + dy * dy);
    };

  const auto distance_to_segment =
    [&](const geometry_msgs::msg::PoseStamped & start_pose,
        const geometry_msgs::msg::PoseStamped & end_pose) {
      const double start_x = start_pose.pose.position.x;
      const double start_y = start_pose.pose.position.y;
      const double end_x = end_pose.pose.position.x;
      const double end_y = end_pose.pose.position.y;
      const double seg_x = end_x - start_x;
      const double seg_y = end_y - start_y;
      const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
      if(seg_len_sq < 1e-6){
        return distance_to_point(start_pose);
      }
      const double projection =
        ((robot_x - start_x) * seg_x + (robot_y - start_y) * seg_y) / seg_len_sq;
      const double clamped_projection = std::max(0.0, std::min(1.0, projection));
      const double closest_x = start_x + clamped_projection * seg_x;
      const double closest_y = start_y + clamped_projection * seg_y;
      const double dx = robot_x - closest_x;
      const double dy = robot_y - closest_y;
      return std::sqrt(dx * dx + dy * dy);
    };

  double best_distance = distance_to_point(global_plan_[index]);
  if(index > 0){
    best_distance = std::min(best_distance, distance_to_segment(global_plan_[index - 1], global_plan_[index]));
  }
  if(index + 1 < global_plan_.size()){
    best_distance = std::min(best_distance, distance_to_segment(global_plan_[index], global_plan_[index + 1]));
  }
  return best_distance;
}

double Local_Planner::getRouteHeadingError(std::size_t index) const
{
  if(global_plan_.size() < 2){
    return 0.0;
  }

  index = std::min(index, global_plan_.size() - 1);
  std::size_t first_index = index;
  std::size_t last_index = index;
  if(index + 1 < global_plan_.size()){
    last_index = index + 1;
  }
  else if(index > 0){
    first_index = index - 1;
  }

  const double dx =
    global_plan_[last_index].pose.position.x - global_plan_[first_index].pose.position.x;
  const double dy =
    global_plan_[last_index].pose.position.y - global_plan_[first_index].pose.position.y;
  if(std::hypot(dx, dy) < 1e-6){
    return 0.0;
  }

  const double route_yaw = std::atan2(dy, dx);
  return std::fabs(angles::shortest_angular_distance(getRobotYaw(), route_yaw));
}

bool Local_Planner::selectCausalPruneAnchor(
  std::size_t * anchor_index,
  double * robot_to_route_distance)
{
  if(anchor_index == nullptr || robot_to_route_distance == nullptr || global_plan_.empty()){
    return false;
  }

  const std::size_t current_index = std::min(local_route_progress_index_, global_plan_.size() - 1);
  const std::size_t search_window =
    std::max<std::size_t>(1, causal_prune_search_window_);
  const std::size_t search_end =
    std::min(current_index + search_window, global_plan_.size() - 1);

  bool have_candidate = false;
  std::size_t best_index = current_index;
  double best_lateral_distance = std::numeric_limits<double>::infinity();

  for(std::size_t idx = current_index; idx <= search_end; ++idx){
    const std::size_t index_jump = idx - current_index;
    const double arc_jump = getGlobalPlanArcDistance(current_index, idx);
    const double lateral_distance = getRouteSegmentLateralDistance(idx);
    const double heading_error = getRouteHeadingError(idx);

    // Local causal prune must follow route order. It must not jump to a later
    // segment that is only Euclidean-close.
    const bool heading_ok =
      heading_error <= causal_prune_max_heading_error_ || index_jump <= 1;
    const bool accept_candidate =
      index_jump <= causal_prune_max_index_jump_ &&
      arc_jump <= causal_prune_max_arc_jump_ &&
      lateral_distance <= causal_prune_max_lateral_distance_ &&
      heading_ok;

    if(!accept_candidate){
      continue;
    }

    if(!have_candidate || lateral_distance + 1e-6 < best_lateral_distance ||
       (std::fabs(lateral_distance - best_lateral_distance) <= 1e-6 && idx < best_index)){
      have_candidate = true;
      best_index = idx;
      best_lateral_distance = lateral_distance;
    }
  }

  if(have_candidate){
    *anchor_index = best_index;
    *robot_to_route_distance = best_lateral_distance;
    return true;
  }

  const double fallback_distance = getRouteSegmentLateralDistance(current_index);
  if(fallback_distance <= causal_prune_max_lateral_distance_ * 1.5){
    *anchor_index = current_index;
    *robot_to_route_distance = fallback_distance;
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "causal prune rejected jump, keeping current progress index, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_);
    return true;
  }

  const std::size_t relaxed_step_end = std::min(
    current_index + std::min<std::size_t>(causal_prune_max_index_jump_, 3),
    global_plan_.size() - 1);
  for(std::size_t idx = current_index + 1; idx <= relaxed_step_end; ++idx){
    const double lateral_distance = getRouteSegmentLateralDistance(idx);
    if(lateral_distance <= causal_prune_max_lateral_distance_ * 1.25){
      *anchor_index = idx;
      *robot_to_route_distance = lateral_distance;
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "causal prune accepted only a small progress step, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, next_index=%zu",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        idx);
      return true;
    }
  }

  if(allow_offroute_anchor_recovery_){
    bool have_relaxed_anchor = false;
    std::size_t relaxed_anchor_index = current_index;
    double relaxed_anchor_distance = std::numeric_limits<double>::infinity();
    for(std::size_t idx = current_index; idx <= search_end; ++idx){
      const std::size_t index_jump = idx - current_index;
      const double arc_jump = getGlobalPlanArcDistance(current_index, idx);
      if(index_jump > causal_prune_max_index_jump_ || arc_jump > causal_prune_max_arc_jump_){
        continue;
      }
      const double lateral_distance = getRouteSegmentLateralDistance(idx);
      if(!std::isfinite(lateral_distance)){
        continue;
      }
      if(!have_relaxed_anchor || lateral_distance < relaxed_anchor_distance){
        relaxed_anchor_distance = lateral_distance;
        relaxed_anchor_index = idx;
        have_relaxed_anchor = true;
      }
    }

    if(have_relaxed_anchor){
      *anchor_index = relaxed_anchor_index;
      *robot_to_route_distance = relaxed_anchor_distance;
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "off-route anchor recovery engaged, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, anchor_index=%zu, robot_to_route_distance=%.2f",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        relaxed_anchor_index,
        relaxed_anchor_distance);
      return true;
    }
  }

  *robot_to_route_distance = fallback_distance;
  return false;
}

bool Local_Planner::buildPrunedPlanAroundIndex(
  std::size_t anchor_index,
  double forward_distance,
  double backward_distance,
  nav_msgs::msg::Path * prune_plan,
  pcl::PointCloud<pcl::PointXYZI> * pcl_prune_plan) const
{
  if(prune_plan == nullptr || pcl_prune_plan == nullptr || global_plan_.empty()){
    return false;
  }

  anchor_index = std::min(anchor_index, global_plan_.size() - 1);
  prune_plan->poses.clear();
  pcl_prune_plan->clear();
  prune_plan->header.frame_id = global_frame_;
  prune_plan->header.stamp = clock_->now();

  geometry_msgs::msg::PoseStamped last_pose = global_plan_[anchor_index];
  for(std::size_t offset = 0; offset <= anchor_index; ++offset){
    const std::size_t i = anchor_index - offset;
    prune_plan->poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x;
    pt.y = global_plan_[i].pose.position.y;
    pt.z = global_plan_[i].pose.position.z;
    pt.intensity = -1.0;
    pcl_prune_plan->points.push_back(pt);
    if(i < anchor_index){
      backward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(backward_distance < 0.0 || i == 0){
      break;
    }
  }

  std::reverse(prune_plan->poses.begin(), prune_plan->poses.end());

  last_pose = global_plan_[anchor_index];
  for(std::size_t i = anchor_index; i < global_plan_.size(); ++i){
    prune_plan->poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x;
    pt.y = global_plan_[i].pose.position.y;
    pt.z = global_plan_[i].pose.position.z;
    pt.intensity = (i == 0) ? 0.0 : 1.0;
    pcl_prune_plan->points.push_back(pt);
    if(i > anchor_index){
      forward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(forward_distance < 0.0){
      break;
    }
  }

  for(auto & pose : prune_plan->poses){
    pose.header = prune_plan->header;
  }

  return !prune_plan->poses.empty();
}

bool Local_Planner::tryReuseCachedPrunedPath()
{
  if(!cached_local_pruned_path_valid_){
    return false;
  }

  const double cached_age = (clock_->now() - cached_local_pruned_path_stamp_).seconds();
  if(cached_age > cached_pruned_path_timeout_sec_){
    return false;
  }

  prune_plan_ = cached_local_pruned_path_;
  prune_plan_.header.frame_id = global_frame_;
  prune_plan_.header.stamp = clock_->now();
  for(auto & pose : prune_plan_.poses){
    pose.header = prune_plan_.header;
  }
  pcl_prune_plan_ = cached_pcl_prune_plan_;
  last_prune_used_cache_ = true;
  last_valid_prune_plan_ = clock_->now();
  publishDebugPath(
    prune_plan_,
    pub_local_pruned_path_,
    "local_reference_path",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);
  RCLCPP_WARN_THROTTLE(
    this->get_logger().get_child(name_), *clock_, 2000,
    "reuse cached local pruned path due to transient prune failure, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu",
    route_version_,
    goal_seq_,
    route_source_label_.c_str(),
    local_route_progress_index_,
    consecutive_prune_failure_cycles_);
  return true;
}

bool Local_Planner::buildHeadingReferenceFromPoseOrientation(
  const geometry_msgs::msg::PoseStamped & pose,
  tf2::Transform * reference_pose) const
{
  if(reference_pose == nullptr){
    return false;
  }

  const auto & q_msg = pose.pose.orientation;
  const double quat_norm =
    q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
  if(quat_norm < 1e-6){
    return false;
  }

  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  q.normalize();
  reference_pose->setRotation(q);
  reference_pose->setOrigin(tf2::Vector3(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z));
  return true;
}

void Local_Planner::parseCuboid(){
  marker_edge_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();;
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3; marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9; marker_edge_.color.g = 1; marker_edge_.color.b = 0; marker_edge_.color.a = 0.8;
  //@ parse cuboid, currently the cuboid in local planner is just for visualization
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {"cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt", "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;

  for(auto it=cuboid_vertex_queue.begin(); it!=cuboid_vertex_queue.end();it++){
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param= this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {"cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt", "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
                                                      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb", "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for(auto it=cuboid_vertex_connect.begin(); it!=cuboid_vertex_connect.end();it++){
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }

  robot_cuboid_vertices_.clear();
  const std::vector<std::string> collision_model_order = {
    "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.flb",
    "cuboid.brt", "cuboid.frt", "cuboid.flt", "cuboid.frb"};
  robot_cuboid_vertices_.reserve(collision_model_order.size());
  for(const auto & name : collision_model_order){
    const auto & p = cuboid_vertex_parameter_map[name];
    pcl::PointXYZ pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    robot_cuboid_vertices_.push_back(pt);
  }
  double half_width = 0.0;
  for(const auto & vertex : robot_cuboid_vertices_){
    if(std::isfinite(vertex.y)){
      half_width = std::max(half_width, std::fabs(static_cast<double>(vertex.y)));
    }
  }
  if(half_width > 1e-6){
    robot_half_width_ = half_width;
  }
}

void Local_Planner::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_ = *msg;
  // Account measured travel before any readiness/TF/preview early-return so
  // pauses, stop transitions, and failed readiness waits keep the real
  // episode budget. Repeated optimization retries consume no extra distance.
  updateMppiMeasuredTravelFromOdom(*msg);
  if(!updateGlobalPose()){
    got_odom_ = false;
    return;
  }
  got_odom_ = true;
}


double Local_Planner::getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose){

  //@Transform trans_gbl2b_ to tf2; Get baselink to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2b;
  tf2::fromMsg(trans_gbl2b_, tf2_trans_gbl2b);
  auto tf2_trans_gbl2b_inverse = tf2_trans_gbl2b.inverse();
  //@Get baselink to last pose
  tf2::Transform tf2_baselink2prunelastpose;
  tf2_baselink2prunelastpose.mult(tf2_trans_gbl2b_inverse, m_pose);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_baselink2prunelastpose.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //@Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  
  return yaw;

}

bool Local_Planner::isGoalHeadingAligned(){

  if(global_plan_.empty()){
    return false;
  }

  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();

  geometry_msgs::msg::TransformStamped final_pose_ts;
  final_pose_ts.header = final_pose.header;
  final_pose_ts.transform.translation.x = final_pose.pose.position.x;
  final_pose_ts.transform.translation.y = final_pose.pose.position.y;
  final_pose_ts.transform.translation.z = final_pose.pose.position.z;
  final_pose_ts.transform.rotation.x = final_pose.pose.orientation.x;
  final_pose_ts.transform.rotation.y = final_pose.pose.orientation.y;
  final_pose_ts.transform.rotation.z = final_pose.pose.orientation.z;
  final_pose_ts.transform.rotation.w = final_pose.pose.orientation.w;
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2goal;
  tf2::fromMsg(final_pose_ts, tf2_trans_gbl2goal);  

  //@Update the value to critics that allow the robot to turn by shortest angle
  double yaw = getShortestAngleFromPose2RobotHeading(tf2_trans_gbl2goal);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Heading difference to goal is %.2f", yaw);

  if(fabs(yaw) < yaw_goal_tolerance_)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalReached(){
  if(global_plan_.empty()){
    return false;
  }
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();
  double dx = trans_gbl2b_.transform.translation.x - final_pose.pose.position.x;
  double dy = trans_gbl2b_.transform.translation.y - final_pose.pose.position.y;
  double distance = sqrt(dx*dx + dy*dy);
  if(distance <= xy_goal_tolerance_)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalPositionReached()
{
  return isGoalReached();
}

bool Local_Planner::isGoalHeadingSatisfied()
{
  return isGoalHeadingAligned();
}

void Local_Planner::setRoute(
  const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label)
{
  if(
    !global_plan_.empty() &&
    route_version_ == route_version &&
    goal_seq_ == goal_seq &&
    route_source_label_ == source_label)
  {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 5000,
      "reuse delivered route in route controller, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu",
      route_version,
      goal_seq,
      source_label.c_str(),
      local_route_progress_index_);
    return;
  }

  publishDebugPath(
    buildPathFromPlan(orig_global_plan),
    pub_route_sent_to_local_,
    "controller_route_reference",
    route_version,
    goal_seq,
    source_label,
    false);

  if(orig_global_plan.size()<3){
    RCLCPP_ERROR(
      this->get_logger().get_child(name_),
      "route has fewer than 3 poses, route_version=%zu, goal_seq=%zu, source=%s",
      route_version,
      goal_seq,
      source_label.c_str());
    return;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;
  route_version_ = route_version;
  goal_seq_ = goal_seq;
  route_source_label_ = source_label;
  resetLocalRouteTrackingState();
  const MppiControl route_measured_speed{
    robot_state_.twist.twist.linear.x,
    robot_state_.twist.twist.angular.z};
  mppi_controller_.resetControlSequence(route_measured_speed);
  mppi_last_command_ = route_measured_speed;

  rebuildGlobalPlanArcLengths();
  RCLCPP_DEBUG(
    this->get_logger().get_child(name_),
    "receive new route reference, route_version=%zu, goal_seq=%zu, source=%s, poses=%zu, local_route_progress_index=%zu",
    route_version_,
    goal_seq_,
    route_source_label_.c_str(),
    global_plan_.size(),
    local_route_progress_index_);
}

void Local_Planner::setPlan(
  const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan,
  std::size_t route_version,
  std::size_t goal_seq,
  const std::string & source_label)
{
  setRoute(orig_global_plan, route_version, goal_seq, source_label);
}

double Local_Planner::getDistanceBTWPoseStamp(
  const geometry_msgs::msg::PoseStamped& a,
  const geometry_msgs::msg::PoseStamped& b) const
{

  double dx = a.pose.position.x-b.pose.position.x;
  double dy = a.pose.position.y-b.pose.position.y;
  double dz = a.pose.position.z-b.pose.position.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

bool Local_Planner::updateGlobalPose(){
  try
  {
    trans_gbl2b_ = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "%s: %s", name_.c_str(), e.what());
    return false;
  }

  if(debug_publish_robot_cuboid_ && pub_robot_cuboid_->get_subscription_count() > 0){
    robot_cuboid_.markers.clear();
    marker_edge_.header.stamp = rclcpp::Time(0);
    robot_cuboid_.markers.push_back(marker_edge_);
    pub_robot_cuboid_->publish(robot_cuboid_);
  }
  return true;
}

geometry_msgs::msg::TransformStamped Local_Planner::getGlobalPose(){
  return trans_gbl2b_;
}

bool Local_Planner::prunePlan(double forward_distance, double backward_distance){

  prune_plan_.poses.clear();
  pcl_prune_plan_.clear();
  prune_plan_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan_.header.stamp = clock_->now();
  last_prune_used_cache_ = false;

  if(global_plan_.size() < 3){
    ++consecutive_prune_failure_cycles_;
    last_robot_to_route_distance_ = std::numeric_limits<double>::infinity();
    if(tryReuseCachedPrunedPath()){
      return true;
    }
    publishDebugPath(
      prune_plan_,
      pub_local_pruned_path_,
      "local_reference_path",
      route_version_,
      goal_seq_,
      route_source_label_,
      true);
    return false;
  }

  std::size_t anchor_index = local_route_progress_index_;
  double robot_to_route_distance = std::numeric_limits<double>::infinity();
  const bool have_anchor = selectCausalPruneAnchor(&anchor_index, &robot_to_route_distance);
  last_robot_to_route_distance_ = robot_to_route_distance;
  const bool within_route_deviation_tolerance =
    robot_to_route_distance <= causal_prune_max_lateral_distance_;

  if(have_anchor &&
     within_route_deviation_tolerance &&
     buildPrunedPlanAroundIndex(anchor_index, forward_distance, backward_distance, &prune_plan_, &pcl_prune_plan_)){
    local_route_progress_index_ = std::max(local_route_progress_index_, anchor_index);
    consecutive_prune_failure_cycles_ = 0;
    cached_local_pruned_path_valid_ = true;
    cached_local_pruned_path_ = prune_plan_;
    cached_local_pruned_path_stamp_ = clock_->now();
    cached_pcl_prune_plan_ = pcl_prune_plan_;
    last_valid_prune_plan_ = clock_->now();
  }
  else{
    ++consecutive_prune_failure_cycles_;
    if(tryReuseCachedPrunedPath()){
      return true;
    }
    if(have_anchor && !within_route_deviation_tolerance){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "local route reference rejected because robot is off route, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, anchor_index=%zu, robot_to_route_distance=%.2f, tolerance=%.2f, failure_cycles=%zu",
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        local_route_progress_index_,
        anchor_index,
        last_robot_to_route_distance_,
        causal_prune_max_lateral_distance_,
        consecutive_prune_failure_cycles_);
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "transient prune failure, not treating as deviation yet, route_version=%zu, goal_seq=%zu, source=%s, local_route_progress_index=%zu, failure_cycles=%zu, robot_to_route_distance=%.2f",
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      local_route_progress_index_,
      consecutive_prune_failure_cycles_,
      last_robot_to_route_distance_);
    publishDebugPath(
      prune_plan_,
      pub_local_pruned_path_,
      "local_reference_path",
      route_version_,
      goal_seq_,
      route_source_label_,
      true);
    return false;
  }

  if(debug_publish_prune_plan_ && pub_prune_plan_->get_subscription_count() > 0){
    pub_prune_plan_->publish(prune_plan_);
  }
  publishDebugPath(
    prune_plan_,
    pub_local_pruned_path_,
    "local_reference_path",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);
  return true;
}

void Local_Planner::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

  //@ in case we have collision
  best_traj.cost_ = -1;

  double minimum_cost = 9999999;
  const bool publish_accepted_trajectory =
    debug_publish_accepted_trajectory_ &&
    pub_accepted_trajectory_pose_array_->get_subscription_count() > 0;
  const bool publish_best_trajectory =
    debug_publish_best_trajectory_ &&
    pub_best_trajectory_pose_->get_subscription_count() > 0;
  geometry_msgs::msg::PoseArray accepted_pose_arr;

  for(auto traj_it=trajectories_->begin();traj_it!=trajectories_->end();traj_it++){

    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));

    if((*traj_it).cost_>=0){
      constexpr double kCostTieEpsilon = 1e-6;
      bool strictly_better = (*traj_it).cost_ + kCostTieEpsilon < minimum_cost;
      bool cost_tie = std::fabs((*traj_it).cost_ - minimum_cost) <= kCostTieEpsilon;
      bool straighter_tie_break =
        best_traj.cost_ >= 0 &&
        std::fabs((*traj_it).thetav_) + kCostTieEpsilon < std::fabs(best_traj.thetav_);
      bool faster_tie_break =
        best_traj.cost_ >= 0 &&
        std::fabs(std::fabs((*traj_it).thetav_) - std::fabs(best_traj.thetav_)) <= kCostTieEpsilon &&
        (*traj_it).xv_ > best_traj.xv_ + kCostTieEpsilon;

      // Trajectory samples are iterated from negative yaw rate to positive yaw
      // rate. Using <= here makes equal-cost ties drift toward the last sample,
      // which biases the robot to the left. Prefer straighter, then faster
      // trajectories when the critic score is effectively tied.
      if(strictly_better || (cost_tie && (straighter_tie_break || faster_tie_break))){
        best_traj = (*traj_it);
        minimum_cost = (*traj_it).cost_;
      }
    }

    if(publish_accepted_trajectory && (*traj_it).cost_>=0){
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr);
    }

  }

  if(best_traj.cost_ < 0 && !trajectories_->empty()){
    size_t rejected_collision = 0;
    size_t rejected_pure_pursuit = 0;
    size_t rejected_toward_global_plan = 0;
    size_t rejected_other = 0;

    for(const auto& traj : *trajectories_){
      if(traj.cost_ == -1.0){
        rejected_collision++;
      }
      else if(traj.cost_ == -4.0){
        rejected_pure_pursuit++;
      }
      else if(traj.cost_ == -12.0){
        rejected_toward_global_plan++;
      }
      else if(traj.cost_ < 0.0){
        rejected_other++;
      }
    }

    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 2000,
      "Trajectory rejection summary: collision=%zu, pure_pursuit=%zu, toward_global_plan=%zu, other=%zu, total=%zu",
      rejected_collision, rejected_pure_pursuit, rejected_toward_global_plan, rejected_other, trajectories_->size());
  }

  if(publish_accepted_trajectory){
    accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    accepted_pose_arr.header.stamp = clock_->now();
    pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);
  }

  if(publish_best_trajectory){
    geometry_msgs::msg::PoseArray best_pose_arr;
    trajectory2posearray_cuboids(best_traj, best_pose_arr);
    best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  nav_msgs::msg::Path best_trajectory_path;
  if(best_traj.cost_ >= 0){
    best_trajectory_path = buildPathFromTrajectory(best_traj);
  }
  else{
    best_trajectory_path.header.frame_id = global_frame_;
    best_trajectory_path.header.stamp = clock_->now();
  }
  publishDebugPath(
    best_trajectory_path,
    pub_best_trajectory_path_,
    "best_control_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

}

dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){
  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return context_state;
  }

  //Do not create a function to set the parameters unless a nice structure is found
  //Below assignment of variables is useful when migrate to ROS2
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ 
                  = context.current_allowed_max_linear_speed;

  trajectory_generators_ros_->initializeTheories_wi_Shared_data();

  const bool publish_all_trajectories =
    debug_publish_all_trajectories_ &&
    pub_trajectory_pose_array_->get_subscription_count() > 0;
  geometry_msgs::msg::PoseArray pose_arr;

  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  #endif

  //@ We queue all trajectories in trajectories_, then score them one by one in getBestTrajectory()
  while(trajectory_generators_ros_->hasMoreTrajectories(traj_gen_name)){
    base_trajectory::Trajectory a_traj;
    if(trajectory_generators_ros_->nextTrajectory(traj_gen_name, a_traj)){
      //@ collected all trajectories here, for later scoring
      trajectories_->push_back(a_traj);
      if(publish_all_trajectories){
        trajectory2posearray_cuboids(a_traj, pose_arr);
      }
    }

  }

  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  RCLCPP_DEBUG(this->get_logger(), "Trajectory generation time: %.9f", t_diff);
  #endif

  if(publish_all_trajectories){
    pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    pose_arr.header.stamp = clock_->now();
    pub_trajectory_pose_array_->publish(pose_arr);
  }

  
  

  //@Update data for critics
  {
    std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(
      *(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
    updateCriticSharedData(
      context.aggregate_observation,
      context.aggregate_observation_kdtree);
    getBestTrajectory(traj_gen_name, best_traj);
  }

  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full control cycle time: %.9f", t_diff.seconds());

  // if(t_diff.seconds() > 1./controller_frequency_){
  //   RCLCPP_WARN(this->get_logger().get_child(name_), "Local planner control time exceed expect time: %.2f but is %.2f", 1./controller_frequency_, t_diff.seconds());
  // }
  
  const auto opinion_state = evaluatePerceptionOpinions(context.opinions);
  if(opinion_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return opinion_state;
  }

  if(best_traj.cost_<0){
    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "All trajectories are rejected by critics.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }
  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::computeRppControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return context_state;
  }

  const bool goal_alignment_mode = isGoalReached();
  const double distance_to_goal = getDistanceToGoal();
  double linear_speed_cap = rpp_nominal_linear_speed_;
  if(context.current_allowed_max_linear_speed > 0.0){
    linear_speed_cap = std::min(linear_speed_cap, context.current_allowed_max_linear_speed);
  }

  if(goal_alignment_mode){
    linear_speed_cap = std::min(linear_speed_cap, rpp_alignment_linear_speed_);
  }
  else if(distance_to_goal < rpp_goal_slowdown_distance_ && rpp_goal_slowdown_distance_ > 1e-3){
    const double ratio = std::max(0.0, std::min(1.0, distance_to_goal / rpp_goal_slowdown_distance_));
    const double slowdown_floor = std::min(rpp_min_linear_speed_, linear_speed_cap);
    linear_speed_cap = slowdown_floor + (linear_speed_cap - slowdown_floor) * ratio;
  }

  if(linear_speed_cap <= 1e-3){
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "RPP trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=speed_cap_zero, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, distance_to_goal=%.2f, allowed_max_speed=%.2f",
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      distance_to_goal,
      context.current_allowed_max_linear_speed);
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  const double max_curvature = getMaxAckermannCurvature();
  double curvature = 0.0;
  if(goal_alignment_mode){
    tf2::Transform reference_pose;
    if(!buildGoalAlignmentReference(&reference_pose)){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=goal_alignment_reference_unavailable, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, distance_to_goal=%.2f",
        controller_name.c_str(),
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        distance_to_goal);
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    const double heading_error = getShortestAngleFromPose2RobotHeading(reference_pose);
    mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = heading_error;
    curvature =
      2.0 * std::sin(heading_error) / std::max(0.05, rpp_alignment_lookahead_distance_);
  }
  else{
    double lookahead_distance = std::max(0.05, rpp_lookahead_dist_);
    if(rpp_use_velocity_scaled_lookahead_dist_){
      const double current_speed = std::fabs(robot_state_.twist.twist.linear.x);
      lookahead_distance = std::max(
        rpp_min_lookahead_distance_,
        current_speed * rpp_lookahead_time_);
      lookahead_distance = std::min(lookahead_distance, rpp_max_lookahead_distance_);
    }

    geometry_msgs::msg::PoseStamped lookahead_pose;
    if(!selectLookaheadPose(lookahead_distance, &lookahead_pose)){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=lookahead_target_unavailable, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, lookahead_distance=%.2f, velocity_scaled=%d",
        controller_name.c_str(),
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        lookahead_distance,
        rpp_use_velocity_scaled_lookahead_dist_ ? 1 : 0);
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    const double robot_yaw = getRobotYaw();
    const double dx = lookahead_pose.pose.position.x - trans_gbl2b_.transform.translation.x;
    const double dy = lookahead_pose.pose.position.y - trans_gbl2b_.transform.translation.y;
    const double x_local = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double y_local = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double distance_sq = x_local * x_local + y_local * y_local;

    if(distance_sq < 1e-6){
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "RPP trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=lookahead_target_too_close, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, lookahead_distance=%.2f, x_local=%.3f, y_local=%.3f",
        controller_name.c_str(),
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        lookahead_distance,
        x_local,
        y_local);
      return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
    }

    curvature = (2.0 * y_local) / distance_sq;
  }

  curvature = std::max(-max_curvature, std::min(max_curvature, curvature));
  const double min_linear_speed =
    std::min(goal_alignment_mode ? rpp_alignment_linear_speed_ : rpp_min_linear_speed_, linear_speed_cap);

  double linear_velocity = linear_speed_cap;
  if(std::fabs(curvature) > 1e-6 && rpp_max_lateral_accel_ > 0.0){
    linear_velocity = std::min(
      linear_velocity,
      std::sqrt(rpp_max_lateral_accel_ / std::fabs(curvature)));
  }
  if(std::fabs(curvature) > 1e-6 && rpp_max_angular_velocity_ > 0.0){
    linear_velocity = std::min(
      linear_velocity,
      rpp_max_angular_velocity_ / std::fabs(curvature));
  }

  linear_velocity = std::max(min_linear_speed, std::min(linear_velocity, linear_speed_cap));
  const double angular_velocity =
    std::max(-rpp_max_angular_velocity_,
      std::min(rpp_max_angular_velocity_, linear_velocity * curvature));
  auto predicted_traj = buildPredictedTrajectory(linear_velocity, angular_velocity);
  {
    std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(
      *(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
    updateCriticSharedData(
      context.aggregate_observation,
      context.aggregate_observation_kdtree);
    mpc_critics_ros_->scoreTrajectory(
      rpp_critic_trajectory_generator_name_,
      predicted_traj);
  }

  publishDebugPath(
    buildPathFromTrajectory(predicted_traj),
    pub_best_trajectory_path_,
    "best_control_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

  if(debug_publish_best_trajectory_ && pub_best_trajectory_pose_->get_subscription_count() > 0){
    geometry_msgs::msg::PoseArray best_pose_arr;
    trajectory2posearray_cuboids(predicted_traj, best_pose_arr);
    best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  const auto opinion_state = evaluatePerceptionOpinions(context.opinions);
  if(opinion_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return opinion_state;
  }

  if(predicted_traj.cost_ < 0.0){
    const char * reject_reason = "unknown";
    if(std::fabs(predicted_traj.cost_ + 1.0) < 1e-6){
      reject_reason = "collision";
    }
    else if(std::fabs(predicted_traj.cost_ + 4.0) < 1e-6){
      reject_reason = "pure_pursuit";
    }
    else if(std::fabs(predicted_traj.cost_ + 12.0) < 1e-6){
      reject_reason = "toward_global_plan";
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "RPP trigger: state=%s, action=%s, reason=critic_reject_%s, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, cost=%.2f, linear_velocity=%.2f, angular_velocity=%.3f, curvature=%.3f, prediction_horizon=%.2f",
      std::fabs(predicted_traj.cost_ + 1.0) < 1e-6 ? "PATH_BLOCKED_REPLANNING" : "ALL_TRAJECTORIES_FAIL",
      std::fabs(predicted_traj.cost_ + 1.0) < 1e-6 ? "route_replan" : "stop_wait",
      reject_reason,
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      predicted_traj.cost_,
      linear_velocity,
      angular_velocity,
      curvature,
      rpp_prediction_horizon_sec_);

    if(std::fabs(predicted_traj.cost_ + 1.0) < 1e-6){
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }

    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  if(cmd_vel != nullptr){
    cmd_vel->linear.x = linear_velocity;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = angular_velocity;
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::computeMppiControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  // Fail closed: an invalid environment selector or failed local-map setup
  // must never silently continue with the legacy point cloud backend.
  if(!mppi_local_map_init_error_.empty() ||
     mppi_environment_backend_ == MppiEnvironmentBackend::kLocalMap)
  {
    return computeMppiLocalMapControlCommand(controller_name, cmd_vel);
  }

  ControllerContext context;
  const auto context_state = prepareControllerContext(&context);
  if(context_state != dddmr_sys_core::TRAJECTORY_FOUND){
    return context_state;
  }

  // Forward speed cap for this cycle: the external allowed speed combined with
  // the linear goal-approach slowdown.
  const double mppi_cycle_speed_cap =
    mppiSpeedCap(context.current_allowed_max_linear_speed);

  if(mppi_use_perception_blocked_preemption_){
    const auto opinion_state = evaluatePerceptionOpinions(context.opinions);
    if(opinion_state != dddmr_sys_core::TRAJECTORY_FOUND){
      return opinion_state;
    }
  }
  else{
    for(const auto & opinion : context.opinions){
      if(opinion == perception_3d::PATH_BLOCKED_WAIT ||
         opinion == perception_3d::PATH_BLOCKED_REPLANNING)
      {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger().get_child(name_), *clock_, 2000,
          "MPPI debug: ignoring perception blocked opinion and deferring obstacle handling to MPPI critics, controller=%s, route_version=%zu, goal_seq=%zu, source=%s",
          controller_name.c_str(),
          route_version_,
          goal_seq_,
          route_source_label_.c_str());
        break;
      }
    }
  }

  if(context.current_allowed_max_linear_speed > 0.0 &&
     context.current_allowed_max_linear_speed < mppi_controller_.config().vx_min)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI trigger: state=ALL_TRAJECTORIES_FAIL, action=stop_wait, reason=speed_cap_below_minimum, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, allowed_max_speed=%.2f, vx_min=%.2f",
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      context.current_allowed_max_linear_speed,
      mppi_controller_.config().vx_min);
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }

  MppiControl measured_speed;
  measured_speed.vx = robot_state_.twist.twist.linear.x;
  measured_speed.wz = robot_state_.twist.twist.angular.z;

  const auto mppi_scoring_start = std::chrono::steady_clock::now();
  const auto obstacle_snapshot = makeMppiObstacleSnapshot(
    context.aggregate_observation, false);
  const std::size_t mppi_obstacle_points =
    obstacle_snapshot.cloud ? obstacle_snapshot.cloud->points.size() : 0u;
  std::size_t mppi_scored_trajectories = 0;
  std::size_t mppi_scored_points = 0;
  std::size_t mppi_finite_clearance_points = 0;
  std::size_t mppi_collision_trajectories = 0;
  std::size_t mppi_collision_node_checks = 0;
  std::size_t mppi_collision_candidate_checks = 0;
  std::size_t mppi_clearance_node_checks = 0;
  std::size_t mppi_clearance_candidate_checks = 0;
  std::size_t mppi_valid_scores = 0;
  bool mppi_critic_fail_flag = false;
  double mppi_min_clearance = std::numeric_limits<double>::infinity();
  double mppi_path_occupancy_ratio = 0.0;
  std::size_t mppi_blocked_path_points = 0;
  double mppi_robot_path_offset = 0.0;
  bool mppi_robot_path_offset_valid = false;
  MppiLateralOccupancyProfile mppi_lateral_profile;
  MppiBypassSide mppi_preferred_bypass_side = MppiBypassSide::Neutral;
  double mppi_left_detour = 0.0;
  double mppi_right_detour = 0.0;
  bool mppi_detour_valid = false;

  // Instrumentation for decision reach. These are measurements, not controls:
  // lookahead_m is the along-track distance the batch actually predicts,
  // end_lat/end_along bound the spread of collision-free trajectory endpoints,
  // and clear_* quantify how much of the obstacle field can influence cost.
  double mppi_lookahead_m = 0.0;
  std::size_t mppi_clear_capped_points = 0;
  std::size_t mppi_clear_evaluated_points = 0;
  double mppi_clear_sum = 0.0;
  double mppi_end_lateral_min = std::numeric_limits<double>::infinity();
  double mppi_end_lateral_max = -std::numeric_limits<double>::infinity();
  double mppi_end_along_max = -std::numeric_limits<double>::infinity();
  std::size_t mppi_struct_total = 0;
  std::size_t mppi_struct_free = 0;
  double mppi_struct_best_cost = std::numeric_limits<double>::infinity();
  double mppi_noise_best_cost = std::numeric_limits<double>::infinity();
  bool mppi_path_align_applied = false;
  bool mppi_path_align_skipped_blocked = false;
  bool mppi_path_follow_applied = false;
  bool mppi_path_follow_skipped_blocked = false;
  double mppi_path_follow_target_m = -1.0;
  std::vector<MppiCriticTermCosts> mppi_term_costs;
  std::vector<MppiCandidateScore> mppi_last_scores;
  std::size_t mppi_probe_points = 0;
  double mppi_probe_block_m = -1.0;

  MppiResult result;
  auto score_function =
    [&](const std::vector<MppiCandidate> & candidates) -> std::vector<MppiCandidateScore> {
      mppi_term_costs.clear();
      mppi_last_scores.clear();
      auto critic_data = buildMppiCriticData(
        candidates,
        obstacle_snapshot);
      auto critic_result = mppi_critic_manager_.score(critic_data);
      mppi_scored_trajectories = critic_data.trajectories.size();
      mppi_scored_points = 0;
      mppi_finite_clearance_points = 0;
      mppi_collision_trajectories = 0;
      mppi_valid_scores = 0;
      mppi_critic_fail_flag = critic_result.fail_flag;
      mppi_min_clearance = std::numeric_limits<double>::infinity();
      mppi_clear_capped_points = 0;
      mppi_clear_evaluated_points = 0;
      mppi_clear_sum = 0.0;
      mppi_end_lateral_min = std::numeric_limits<double>::infinity();
      mppi_end_lateral_max = -std::numeric_limits<double>::infinity();
      mppi_end_along_max = -std::numeric_limits<double>::infinity();
      mppi_probe_points = 0;
      mppi_probe_block_m = std::numeric_limits<double>::infinity();
      mppi_path_occupancy_ratio = critic_data.path_obstacle_occupancy_ratio;
      mppi_blocked_path_points = critic_data.blocked_path_points;
      mppi_diagnostic_path_ = critic_data.path;
      mppi_lateral_profile = critic_data.blocked_path_points >= 3 ?
        measureMppiLateralOccupancyForObstacleCloud(
        critic_data.path, obstacle_snapshot) : MppiLateralOccupancyProfile{};
      mppi_preferred_bypass_side = preferredMppiBypassSide(mppi_lateral_profile);
      mppi_robot_path_offset = critic_data.robot_path_offset;
      mppi_robot_path_offset_valid = critic_data.robot_path_offset_valid;
      mppi_left_detour = mppiBypassBandDetour(
        mppi_lateral_profile, MppiBypassSide::Left, mppi_robot_path_offset);
      mppi_right_detour = mppiBypassBandDetour(
        mppi_lateral_profile, MppiBypassSide::Right, mppi_robot_path_offset);
      mppi_detour_valid = mppi_robot_path_offset_valid &&
        std::isfinite(mppi_left_detour) && std::isfinite(mppi_right_detour);
      mppi_collision_node_checks += critic_data.obstacle_collision_node_checks;
      mppi_collision_candidate_checks += critic_data.obstacle_collision_point_checks;
      mppi_clearance_node_checks += critic_data.obstacle_clearance_node_checks;
      mppi_clearance_candidate_checks += critic_data.obstacle_clearance_point_checks;

      const double robot_x = trans_gbl2b_.transform.translation.x;
      const double robot_y = trans_gbl2b_.transform.translation.y;
      const double robot_yaw = getRobotYaw();
      const double clearance_cap =
        std::max(0.0, mppi_critic_manager_.config().obstacles.inflation_radius);
      for(const auto & trajectory : critic_data.trajectories){
        bool trajectory_collides = false;
        for(const auto & point : trajectory.points){
          ++mppi_scored_points;
          if(point.in_collision){
            trajectory_collides = true;
          }
          if(std::isfinite(point.obstacle_clearance)){
            ++mppi_finite_clearance_points;
            mppi_min_clearance = std::min(mppi_min_clearance, point.obstacle_clearance);
            ++mppi_clear_evaluated_points;
            mppi_clear_sum += point.obstacle_clearance;
            if(clearance_cap > 0.0 && point.obstacle_clearance >= clearance_cap - 1e-9){
              ++mppi_clear_capped_points;
            }
          }
        }
        if(trajectory_collides){
          ++mppi_collision_trajectories;
        }
        else if(!trajectory.points.empty()){
          const auto & end = trajectory.points.back();
          const double dx = end.x - robot_x;
          const double dy = end.y - robot_y;
          const double along = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
          const double lateral = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
          mppi_end_along_max = std::max(mppi_end_along_max, along);
          mppi_end_lateral_min = std::min(mppi_end_lateral_min, lateral);
          mppi_end_lateral_max = std::max(mppi_end_lateral_max, lateral);
        }
        mppi_probe_points += trajectory.probe_points.size();
        if(trajectory.probe_first_block_m >= 0.0){
          mppi_probe_block_m = std::min(
            mppi_probe_block_m, trajectory.probe_first_block_m);
        }
      }

      if(critic_result.fail_flag){
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{false, -1.0});
      }

      if(critic_result.scores.size() != candidates.size()){
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{false, -1.0});
      }

      if(mppi_cycle_speed_cap > 0.0){
        for(std::size_t i = 0; i < candidates.size(); ++i){
          if(!mppiSequenceRespectsSpeedCap(
              candidates[i].controls,
              measured_speed.vx,
              mppi_cycle_speed_cap,
              mppi_controller_.config().shift_control_sequence,
              mppi_controller_.config().ax_min,
              mppi_controller_.config().model_dt,
              mppi_controller_.config().controller_period,
              mppi_controller_.config().ax_max))
          {
            critic_result.scores[i] = MppiCandidateScore{false, -1.0};
          }
        }
      }

      annotateMppiCandidateMetadata(critic_data, &critic_result);

      mppi_path_align_applied = critic_result.path_align_applied;
      mppi_path_align_skipped_blocked = critic_result.path_align_skipped_blocked;
      mppi_path_follow_applied = critic_result.path_follow_applied;
      mppi_path_follow_skipped_blocked = critic_result.path_follow_skipped_blocked;
      mppi_path_follow_target_m = critic_result.path_follow_target_distance;
      mppi_term_costs = critic_result.term_costs;
      mppi_last_scores = critic_result.scores;

      // Structured-sample attribution is resolved at the print site, where the
      // controller's actual active structured count is known.

      for(const auto & score : critic_result.scores){
        if(score.valid && std::isfinite(score.cost) && score.cost >= 0.0){
          ++mppi_valid_scores;
        }
      }

      return critic_result.scores;
    };

  bool optimal_sequence_collision_free = false;
  bool has_optimal_collision_pose = false;
  double optimal_collision_x = 0.0;
  double optimal_collision_y = 0.0;
  double optimal_collision_yaw = 0.0;
  std::size_t optimization_attempts = 0;
  constexpr std::size_t max_optimization_attempts = 2;
  const MppiAvoidanceState entry_avoidance_state = mppi_controller_.avoidanceState();
  for(; optimization_attempts < max_optimization_attempts; ++optimization_attempts){
    result = mppi_controller_.optimizeBatch(
      measured_speed,
      score_function,
      [&]() {
        MppiDirectionalContext context{
          mppi_blocked_path_points >= 3,
          mppi_robot_path_offset,
          mppi_robot_path_offset_valid};
        context.preferred_side = mppi_preferred_bypass_side;
        context.left_detour = mppi_left_detour;
        context.right_detour = mppi_right_detour;
        context.detour_valid = mppi_detour_valid;
        return context;
      },
      mppi_cycle_speed_cap,
      [&](bool) {
        const auto metadata = buildMppiCriticData({}, obstacle_snapshot);
        std::vector<int> directed_retreat_obstacle_ids;
        const MppiDirectedRetreat::PoseValidator validator =
          [&](MppiRetreatPose & pose) {
            geometry_msgs::msg::PoseStamped candidate;
            candidate.pose.position.x = pose.x;
            candidate.pose.position.y = pose.y;
            candidate.pose.position.z = pose.z;
            tf2::Quaternion q;
            q.setRPY(pose.roll, pose.pitch, pose.yaw);
            candidate.pose.orientation = tf2::toMsg(q);
            const auto collision = evaluateMppiObstacleCollisionForCuboid(
              transformRobotCuboid(candidate), obstacle_snapshot,
              &directed_retreat_obstacle_ids);
            return collision.evaluated && !collision.in_collision;
          };
        // The legacy snapshot is rebuilt for every update, so it has no stable
        // environment identity; pass zero to force route revalidation.
        return computeMppiDirectedRetreat(
          measured_speed, metadata.path, context.current_allowed_max_linear_speed,
          validator, 0);
      });
    if(!result.has_valid_control){
      break;
    }

    const auto optimal_scored_trajectory = buildMppiScoredTrajectory(
      result.optimal_sequence,
      obstacle_snapshot,
      true);
    for(const auto & point : optimal_scored_trajectory.points){
      mppi_collision_node_checks += point.collision_node_checks;
      mppi_collision_candidate_checks += point.collision_point_checks;
      mppi_clearance_node_checks += point.clearance_node_checks;
      mppi_clearance_candidate_checks += point.clearance_point_checks;
    }
    optimal_sequence_collision_free =
      isMppiTrajectoryCollisionFree(optimal_scored_trajectory, false);
    if(mppi_cycle_speed_cap > 0.0 &&
       !mppiSequenceRespectsSpeedCap(
         result.optimal_sequence,
         measured_speed.vx,
         mppi_cycle_speed_cap,
         mppi_controller_.config().shift_control_sequence,
         mppi_controller_.config().ax_min,
         mppi_controller_.config().model_dt,
         mppi_controller_.config().controller_period,
         mppi_controller_.config().ax_max))
    {
      optimal_sequence_collision_free = false;
    }
    has_optimal_collision_pose = false;
    if(!optimal_sequence_collision_free){
      // Record where the averaged command actually hits, so the diagnostic
      // distinguishes a corner-cutting average from a speed-cap rejection.
      for(const auto & point : optimal_scored_trajectory.points){
        if(point.in_collision){
          has_optimal_collision_pose = true;
          optimal_collision_x = point.x;
          optimal_collision_y = point.y;
          optimal_collision_yaw = point.yaw;
          break;
        }
      }
    }
    if(optimal_sequence_collision_free){
      ++optimization_attempts;
      break;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI optimal trajectory validation failed: resetting control sequence and retrying, controller=%s, attempt=%zu/%zu",
      controller_name.c_str(),
      optimization_attempts + 1,
      max_optimization_attempts);
    mppi_controller_.resetControlSequenceForRetry(
      measured_speed, entry_avoidance_state);
  }
  const std::size_t mppi_command_index =
    mppi_controller_.config().shift_control_sequence &&
    mppi_controller_.controlSequence().size() > 1 ? 1 : 0;
  const MppiControl best_sample_command =
    result.best_sample_sequence.empty() ? MppiControl{} :
    result.best_sample_sequence[
      std::min(mppi_command_index, result.best_sample_sequence.size() - 1)];
  const double mppi_scoring_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - mppi_scoring_start).count();
  const std::string mppi_obstacle_query_metrics = formatMppiObstacleQueryMetrics(
    mppi_collision_node_checks,
    mppi_collision_candidate_checks,
    mppi_clearance_node_checks,
    mppi_clearance_candidate_checks);
  mppi_lookahead_m = std::fabs(measured_speed.vx) *
    static_cast<double>(mppi_controller_.config().time_steps) *
    mppi_controller_.config().model_dt;
  const double mppi_clear_mean = mppi_clear_evaluated_points > 0 ?
    mppi_clear_sum / static_cast<double>(mppi_clear_evaluated_points) : -1.0;
  const double mppi_end_lateral_spread =
    (std::isfinite(mppi_end_lateral_min) && std::isfinite(mppi_end_lateral_max)) ?
    (mppi_end_lateral_max - mppi_end_lateral_min) : -1.0;
  const double mppi_end_along_max_m =
    std::isfinite(mppi_end_along_max) ? mppi_end_along_max : -1.0;
  MppiCriticTermCosts mppi_best_terms;
  if(result.best_sample_index < mppi_term_costs.size()){
    mppi_best_terms = mppi_term_costs[result.best_sample_index];
  }
  // Structured samples are always the first active count of the batch. The
  // active count is 4 when a side is latched, so it must come from the MPPI
  // result rather than the configured maximum.
  mppi_struct_total = std::min(result.structured_candidates, mppi_last_scores.size());
  mppi_struct_free = 0;
  mppi_struct_best_cost = std::numeric_limits<double>::infinity();
  mppi_noise_best_cost = std::numeric_limits<double>::infinity();
  for(std::size_t i = 0; i < mppi_last_scores.size(); ++i){
    const auto & score = mppi_last_scores[i];
    const bool usable = score.valid && std::isfinite(score.cost) && score.cost >= 0.0;
    if(i < mppi_struct_total){
      if(usable && score.collision_free){
        ++mppi_struct_free;
      }
      if(usable){
        mppi_struct_best_cost = std::min(mppi_struct_best_cost, score.cost);
      }
    }
    else if(usable){
      mppi_noise_best_cost = std::min(mppi_noise_best_cost, score.cost);
    }
  }
  RCLCPP_INFO_THROTTLE(
    this->get_logger().get_child(name_), *clock_, 2000,
    "MPPI debug: controller=%s, obstacle_points=%zu, scoring_ms=%.2f, trajectories=%zu, trajectory_points=%zu, %s, finite_clearances=%zu, min_clearance=%.3f, path_occupancy=%.3f, collision_trajectories=%zu, valid_scores=%zu, critic_fail=%d, rejected=%zu, best_cost=%.3f, best_vx=%.3f, best_wz=%.3f, prefilter_wz=%.3f, cmd_vx=%.3f, cmd_wz=%.3f, effective_samples=%.2f, weight_pos=%.3f, weight_neg=%.3f, weight_neutral=%.3f, structured=%zu, structured_weight=%.3f, selected_side=%s, best_side=%s, feasible_left=%zu, feasible_right=%zu, feasible_neutral=%zu, phase_compatible=%zu, side_failures=%zu, quality_advantage=%zu, best_left_cost=%.3f, best_right_cost=%.3f, avoidance_stage=%s, path_offset=%.3f, path_offset_valid=%d, preferred_side=%s, obstacle_offset=%.3f, lateral_profile_valid=%d, departure_offset=%.3f, best_sample=%zu, best_critic_cost=%.3f, best_energy=%.3f, optimal_valid=%d, attempts=%zu, allowed_max_speed=%.3f, lookahead_m=%.2f, end_along_max_m=%.2f, end_lat_spread_m=%.3f, clear_mean=%.3f, clear_capped=%zu/%zu, align_applied=%d, align_skip_blocked=%d, follow_applied=%d, follow_skip_blocked=%d, follow_target_m=%.2f, struct_free=%zu/%zu, struct_best_cost=%.2f, noise_best_cost=%.2f, guard_applied=%d, guard_removed=%zu, best_terms=obs:%.2f,align:%.2f,follow:%.2f,angle:%.2f,constr:%.2f,prefer:%.2f,goal:%.2f,term_sum:%.2f, probe_points=%zu, probe_block_m=%.2f",
    controller_name.c_str(),
    mppi_obstacle_points,
    mppi_scoring_ms,
    mppi_scored_trajectories,
    mppi_scored_points,
    mppi_obstacle_query_metrics.c_str(),
    mppi_finite_clearance_points,
    std::isfinite(mppi_min_clearance) ? mppi_min_clearance : -1.0,
    mppi_path_occupancy_ratio,
    mppi_collision_trajectories,
    mppi_valid_scores,
    mppi_critic_fail_flag,
    result.rejected_samples,
    result.best_cost,
    best_sample_command.vx,
    best_sample_command.wz,
    result.prefilter_command.wz,
    result.command.vx,
    result.command.wz,
    result.effective_sample_size,
    result.positive_weight_mass,
    result.negative_weight_mass,
    result.neutral_weight_mass,
    result.structured_candidates,
    result.structured_weight_mass,
    toString(result.selected_bypass_side),
    toString(result.best_sample_side),
    result.feasible_left_candidates,
    result.feasible_right_candidates,
    result.feasible_neutral_candidates,
    result.phase_compatible_candidates,
    result.side_infeasible_cycles,
    result.side_quality_advantage_cycles,
    result.best_left_cost,
    result.best_right_cost,
    toString(result.avoidance_stage),
    mppi_robot_path_offset,
    mppi_robot_path_offset_valid ? 1 : 0,
    toString(mppi_preferred_bypass_side),
    mppi_lateral_profile.obstacle_offset,
    mppi_lateral_profile.valid ? 1 : 0,
    mppi_controller_.config().departure_lateral_offset,
    result.best_sample_index,
    result.best_critic_cost,
    result.best_control_energy,
    optimal_sequence_collision_free ? 1 : 0,
    optimization_attempts,
    context.current_allowed_max_linear_speed,
    mppi_lookahead_m,
    mppi_end_along_max_m,
    mppi_end_lateral_spread,
    mppi_clear_mean,
    mppi_clear_capped_points,
    mppi_clear_evaluated_points,
    mppi_path_align_applied ? 1 : 0,
    mppi_path_align_skipped_blocked ? 1 : 0,
    mppi_path_follow_applied ? 1 : 0,
    mppi_path_follow_skipped_blocked ? 1 : 0,
    mppi_path_follow_target_m,
    mppi_struct_free,
    mppi_struct_total,
    std::isfinite(mppi_struct_best_cost) ? mppi_struct_best_cost : -1.0,
    std::isfinite(mppi_noise_best_cost) ? mppi_noise_best_cost : -1.0,
    result.bypass_guard_applied ? 1 : 0,
    result.bypass_guard_removed,
    mppi_best_terms.obstacles,
    mppi_best_terms.path_align,
    mppi_best_terms.path_follow,
    mppi_best_terms.path_angle,
    mppi_best_terms.constraint,
    mppi_best_terms.prefer_forward,
    mppi_best_terms.goal + mppi_best_terms.goal_angle,
    mppi_best_terms.obstacles + mppi_best_terms.path_align +
    mppi_best_terms.path_follow + mppi_best_terms.path_angle +
    mppi_best_terms.constraint + mppi_best_terms.prefer_forward +
    mppi_best_terms.goal + mppi_best_terms.goal_angle,
    mppi_probe_points,
    std::isfinite(mppi_probe_block_m) ? mppi_probe_block_m : -1.0);

  maybeLogMppiBlockedDiagnostics(
    obstacle_snapshot, result, best_sample_command, mppi_lateral_profile,
    mppi_robot_path_offset, mppi_robot_path_offset_valid,
    optimal_sequence_collision_free, has_optimal_collision_pose,
    optimal_collision_x, optimal_collision_y, optimal_collision_yaw,
    mppi_blocked_path_points);

  if(result.reverse_recovery_active || result.reverse_candidates_generated ||
     result.reverse_budget_exhausted)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI reverse telemetry (distance cap removed): active=%d, motion=%d, generated=%d, used=%.3f, remaining=%.3f, exhausted=%d, cmd_vx=%.3f, cmd_wz=%.3f, measured_vx=%.3f",
      result.reverse_recovery_active ? 1 : 0,
      result.reverse_motion_active ? 1 : 0,
      result.reverse_candidates_generated ? 1 : 0,
      result.reverse_distance_used,
      result.reverse_distance_remaining,
      result.reverse_budget_exhausted ? 1 : 0,
      result.command.vx,
      result.command.wz,
      measured_speed.vx);
  }

  if(!result.has_valid_control){
    if(mppi_directed_retreat_.active()){
      // Directed retreat still owns the cycle (searching, invalidated, or
      // braking). A missing control is a temporary stopping condition, not
      // evidence that the global route is blocked; requesting a replan here
      // aborts a recovery that is still making progress.
      if(cmd_vel != nullptr){
        cmd_vel->linear.x = 0.0;
        cmd_vel->linear.y = 0.0;
        cmd_vel->linear.z = 0.0;
        cmd_vel->angular.x = 0.0;
        cmd_vel->angular.y = 0.0;
        cmd_vel->angular.z = 0.0;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger().get_child(name_), *clock_, 2000,
        "MPPI trigger: state=PATH_BLOCKED_WAIT, action=stop_wait, reason=directed_retreat_braking, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, retreat_reason=%s",
        controller_name.c_str(),
        route_version_,
        goal_seq_,
        route_source_label_.c_str(),
        mppi_directed_retreat_.reason().c_str());
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    const char * failure_reason =
      result.reverse_budget_exhausted ?
      "directed_retreat_unavailable" :
      (mppi_scored_trajectories == 0 ?
      "no_scored_forward_candidates" :
      (result.rejected_samples >= mppi_scored_trajectories ?
      "all_samples_rejected" :
      (result.selected_bypass_side == MppiBypassSide::Neutral ?
      "directional_selection_failed" : "directional_side_unavailable")));
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI trigger: state=PATH_BLOCKED_REPLANNING, action=route_replan, reason=%s, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, rejected=%zu, valid_scores=%zu",
      failure_reason,
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      result.rejected_samples,
      mppi_valid_scores);
    return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
  }

  if(!optimal_sequence_collision_free){
    if(mppi_directed_retreat_.active()){
      // Feed the final backend rollout rejection back into directed retreat:
      // the selected route is no longer executable, so clear it and brake.
      mppi_directed_retreat_.invalidate();
    }
    mppi_controller_.resetControlSequenceForRetry(
      measured_speed, entry_avoidance_state);
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 2000,
      "MPPI trigger: state=PATH_BLOCKED_REPLANNING, action=route_replan, reason=optimal_sequence_collision, controller=%s, route_version=%zu, goal_seq=%zu, source=%s, attempts=%zu",
      controller_name.c_str(),
      route_version_,
      goal_seq_,
      route_source_label_.c_str(),
      optimization_attempts);
    return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
  }

  mppi_last_command_ = result.command;
  if(cmd_vel != nullptr){
    cmd_vel->linear.x = result.command.vx;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = result.command.wz;
  }

  auto best_traj = buildPredictedTrajectoryFromMppiControls(result.optimal_sequence);
  publishDebugPath(
    buildPathFromTrajectory(best_traj),
    pub_best_trajectory_path_,
    "mppi_best_trajectory",
    route_version_,
    goal_seq_,
    route_source_label_,
    true);

  // nav_msgs/Path carries one colour for the whole polyline, so the two
  // segments are published separately: green is the time rollout, orange the
  // distance probe, red the first probe contact. The probe is recomputed from
  // the published endpoint so the tail stays attached to the rollout that is
  // actually drawn.
  if(debug_publish_best_trajectory_ && best_traj.getPointsSize() > 0 &&
    !result.optimal_sequence.empty())
  {
    const bool want_markers =
      pub_best_trajectory_markers_->get_subscription_count() > 0;
    const bool want_probe = pub_best_trajectory_probe_pose_ &&
      pub_best_trajectory_probe_pose_->get_subscription_count() > 0;
    if(want_markers || want_probe){
      double first_block_m = -1.0;
      const auto probe = buildMppiDistanceProbe(
        best_traj.getPoint(best_traj.getPointsSize() - 1).pose,
        result.optimal_sequence.back(), obstacle_snapshot, &first_block_m);
      const double probe_z = best_traj.getPoint(best_traj.getPointsSize() - 1)
        .pose.position.z;

      if(want_probe){
        geometry_msgs::msg::PoseArray probe_arr;
        probe_arr.header.frame_id = global_frame_;
        probe_arr.header.stamp = clock_->now();
        probe_arr.poses.reserve(probe.size());
        for(const auto & probe_point : probe){
          geometry_msgs::msg::Pose pose;
          pose.position.x = probe_point.x;
          pose.position.y = probe_point.y;
          pose.position.z = probe_z;
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, probe_point.yaw);
          pose.orientation = tf2::toMsg(q);
          probe_arr.poses.push_back(pose);
        }
        pub_best_trajectory_probe_pose_->publish(probe_arr);
      }

      if(want_markers){
        const auto stamp = clock_->now();
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker rollout_marker;
        rollout_marker.header.frame_id = global_frame_;
        rollout_marker.header.stamp = stamp;
        rollout_marker.ns = "mppi_best";
        rollout_marker.id = 0;
        rollout_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rollout_marker.action = visualization_msgs::msg::Marker::ADD;
        rollout_marker.scale.x = 0.03;
        rollout_marker.color.r = 0.10f;
        rollout_marker.color.g = 0.90f;
        rollout_marker.color.b = 0.20f;
        rollout_marker.color.a = 1.0f;
        rollout_marker.pose.orientation.w = 1.0;
        rollout_marker.points.reserve(best_traj.getPointsSize());
        for(unsigned int i = 0; i < best_traj.getPointsSize(); ++i){
          const auto & pose = best_traj.getPoint(i).pose;
          geometry_msgs::msg::Point point;
          point.x = pose.position.x;
          point.y = pose.position.y;
          point.z = pose.position.z;
          rollout_marker.points.push_back(point);
        }
        markers.markers.push_back(rollout_marker);

        visualization_msgs::msg::Marker probe_marker = rollout_marker;
        probe_marker.id = 1;
        probe_marker.color.r = 1.00f;
        probe_marker.color.g = 0.55f;
        probe_marker.color.b = 0.10f;
        // Start where the rollout ended so the two drawn segments join up.
        probe_marker.points.clear();
        probe_marker.points.push_back(rollout_marker.points.back());
        for(const auto & probe_point : probe){
          geometry_msgs::msg::Point point;
          point.x = probe_point.x;
          point.y = probe_point.y;
          point.z = probe_z;
          probe_marker.points.push_back(point);
        }
        markers.markers.push_back(probe_marker);

        if(first_block_m >= 0.0 && !probe_marker.points.empty()){
          visualization_msgs::msg::Marker block_marker = rollout_marker;
          block_marker.id = 2;
          block_marker.type = visualization_msgs::msg::Marker::SPHERE;
          block_marker.scale.x = 0.18;
          block_marker.scale.y = 0.18;
          block_marker.scale.z = 0.18;
          block_marker.color.r = 1.00f;
          block_marker.color.g = 0.10f;
          block_marker.color.b = 0.10f;
          block_marker.points.clear();
          block_marker.pose.position = probe_marker.points.back();
          markers.markers.push_back(block_marker);
        }
        pub_best_trajectory_markers_->publish(markers);
      }
    }
  }

  if(debug_publish_best_trajectory_ && pub_best_trajectory_pose_->get_subscription_count() > 0){
    geometry_msgs::msg::PoseArray best_pose_arr;
    trajectory2posearray_cuboids(best_traj, best_pose_arr);
    best_pose_arr.header.frame_id = global_frame_;
    best_pose_arr.header.stamp = clock_->now();
    pub_best_trajectory_pose_->publish(best_pose_arr);
  }

  return dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::computeControlCommand(
  const std::string & controller_name,
  geometry_msgs::msg::Twist * cmd_vel)
{
  if(cmd_vel != nullptr){
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.x = 0.0;
    cmd_vel->angular.y = 0.0;
    cmd_vel->angular.z = 0.0;
  }

  if(controller_backend_ == "regulated_pure_pursuit"){
    return computeRppControlCommand(controller_name, cmd_vel);
  }

  if(controller_backend_ == "mppi"){
    return computeMppiControlCommand(controller_name, cmd_vel);
  }

  base_trajectory::Trajectory best_traj;
  const auto planner_state = computeVelocityCommand(controller_name, best_traj);
  if(planner_state == dddmr_sys_core::TRAJECTORY_FOUND && cmd_vel != nullptr){
    cmd_vel->linear.x = best_traj.xv_;
    cmd_vel->linear.y = best_traj.yv_;
    cmd_vel->angular.z = best_traj.thetav_;
  }
  return planner_state;
}

void Local_Planner::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj,
                                      geometry_msgs::msg::PoseArray& pose_arr){

  for(unsigned int i=0;i<a_traj.getPointsSize();i++){
      auto p = a_traj.getPoint(i);
      pose_arr.poses.push_back(p.pose);
  }

}


}// end of name space
