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
#include <global_planner/dynamic_window_aware_global_planner.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace std::chrono_literals;

namespace global_planner
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool IsFinitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  return std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z);
}

double NormalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

DWA_GlobalPlanner::DWA_GlobalPlanner(const std::string & name)
: Node(name)
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse DWA_GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DWA_GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool DWA_GlobalPlanner::clearCurrentHandleIfMatches(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  std::lock_guard<std::mutex> lock(current_handle_mutex_);
  if (current_handle_ != goal_handle) {
    return false;
  }
  current_handle_.reset();
  return true;
}

void DWA_GlobalPlanner::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>>
  goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  {
    std::lock_guard<std::mutex> lock(current_handle_mutex_);
    current_handle_.reset();
    current_handle_ = goal_handle;
  }
  std::thread{
    std::bind(&DWA_GlobalPlanner::makePlan, this, std::placeholders::_1),
    goal_handle}.detach();
}

void DWA_GlobalPlanner::initial(
  const std::shared_ptr<perception_3d::Perception3D_ROS> & perception_3d,
  const std::shared_ptr<global_planner::GlobalPlanner> & global_planner)
{
  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  pcl_global_path_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  declare_parameter("look_ahead_distance", rclcpp::ParameterValue(5.0));
  this->get_parameter("look_ahead_distance", look_ahead_distance_);
  look_ahead_distance_ = std::max(look_ahead_distance_, 0.1);

  double legacy_recompute_frequency = 10.0;
  declare_parameter("recompute_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("recompute_frequency", legacy_recompute_frequency);

  declare_parameter("reconnect_frequency", rclcpp::ParameterValue(legacy_recompute_frequency));
  this->get_parameter("reconnect_frequency", recompute_frequency_);
  recompute_frequency_ = std::max(recompute_frequency_, 0.1);

  declare_parameter("startup_replan_lock_distance", rclcpp::ParameterValue(0.8));
  this->get_parameter("startup_replan_lock_distance", startup_replan_lock_distance_);
  startup_replan_lock_distance_ = std::max(startup_replan_lock_distance_, 0.0);

  declare_parameter("startup_replan_lock_offtrack_tolerance", rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "startup_replan_lock_offtrack_tolerance",
    startup_replan_lock_offtrack_tolerance_);
  startup_replan_lock_offtrack_tolerance_ =
    std::max(startup_replan_lock_offtrack_tolerance_, 0.0);

  declare_parameter("corridor_forward_distance", rclcpp::ParameterValue(8.0));
  this->get_parameter("corridor_forward_distance", corridor_forward_distance_);
  corridor_forward_distance_ = std::max(corridor_forward_distance_, 0.5);

  declare_parameter("reconnect_target_distance", rclcpp::ParameterValue(look_ahead_distance_));
  this->get_parameter("reconnect_target_distance", reconnect_target_distance_);
  reconnect_target_distance_ = std::max(reconnect_target_distance_, 0.2);

  declare_parameter("reconnect_deviation_threshold", rclcpp::ParameterValue(0.7));
  this->get_parameter("reconnect_deviation_threshold", reconnect_deviation_threshold_);
  reconnect_deviation_threshold_ = std::max(reconnect_deviation_threshold_, 0.1);

  declare_parameter("reconnect_heading_threshold", rclcpp::ParameterValue(1.3));
  this->get_parameter("reconnect_heading_threshold", reconnect_heading_threshold_);
  reconnect_heading_threshold_ = std::clamp(reconnect_heading_threshold_, 0.1, kPi);

  declare_parameter("max_reconnect_failures_before_global_replan", rclcpp::ParameterValue(3.0));
  this->get_parameter(
    "max_reconnect_failures_before_global_replan",
    max_reconnect_failures_before_global_replan_);
  max_reconnect_failures_before_global_replan_ =
    std::max(max_reconnect_failures_before_global_replan_, 1.0);

  declare_parameter("preferred_turn_sign_lookahead_distance", rclcpp::ParameterValue(1.8));
  this->get_parameter(
    "preferred_turn_sign_lookahead_distance",
    preferred_turn_sign_lookahead_distance_);
  preferred_turn_sign_lookahead_distance_ = std::max(preferred_turn_sign_lookahead_distance_, 0.5);

  declare_parameter("min_force_goal_heading_distance", rclcpp::ParameterValue(3.0));
  this->get_parameter("min_force_goal_heading_distance", min_force_goal_heading_distance_);
  min_force_goal_heading_distance_ = std::max(min_force_goal_heading_distance_, 0.0);

  declare_parameter("max_force_goal_heading_angle", rclcpp::ParameterValue(1.75));
  this->get_parameter("max_force_goal_heading_angle", max_force_goal_heading_angle_);
  max_force_goal_heading_angle_ = std::clamp(max_force_goal_heading_angle_, 0.0, kPi);

  declare_parameter("connector_goal_tolerance", rclcpp::ParameterValue(0.4));
  this->get_parameter("connector_goal_tolerance", connector_goal_tolerance_);
  connector_goal_tolerance_ = std::max(connector_goal_tolerance_, 0.05);

  declare_parameter("connector_commit_distance", rclcpp::ParameterValue(1.5));
  this->get_parameter("connector_commit_distance", connector_commit_distance_);
  connector_commit_distance_ = std::max(connector_commit_distance_, 0.1);

  declare_parameter("connector_timeout_sec", rclcpp::ParameterValue(4.0));
  this->get_parameter("connector_timeout_sec", connector_timeout_sec_);
  connector_timeout_sec_ = std::max(connector_timeout_sec_, 0.1);

  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    planner_mode_ = PlannerMode::kRouteFollow;
    route_version_ = 0;
    connector_version_ = 0;
    consecutive_reconnect_failures_ = 0;
    const rclcpp::Time now = clock_->now();
    last_successful_reconnect_time_ = now;
    last_global_replan_time_ = now;
    active_connector_latched_ = false;
    active_route_anchor_index_ = 0;
    active_connector_start_time_ = now;
    active_connector_route_version_ = 0;
    active_connector_path_.poses.clear();
    active_connector_path_.header.frame_id = global_frame_;
    active_connector_path_.header.stamp = now;
    active_connector_has_goal_heading_ = false;
    active_connector_preferred_turn_sign_ = 0;
    active_reconnect_goal_.header.frame_id = global_frame_;
    active_reconnect_goal_.header.stamp = now;
    active_reconnect_goal_.pose.orientation.w = 1.0;
    active_connector_start_pose_.header.frame_id = global_frame_;
    active_connector_start_pose_.header.stamp = now;
    active_connector_start_pose_.pose.orientation.w = 1.0;
  }

  RCLCPP_INFO(this->get_logger(), "look_ahead_distance: %.2f", look_ahead_distance_);
  RCLCPP_INFO(this->get_logger(), "reconnect_frequency: %.2f", recompute_frequency_);
  RCLCPP_INFO(this->get_logger(), "startup_replan_lock_distance: %.2f", startup_replan_lock_distance_);
  RCLCPP_INFO(
    this->get_logger(),
    "startup_replan_lock_offtrack_tolerance: %.2f",
    startup_replan_lock_offtrack_tolerance_);
  RCLCPP_INFO(
    this->get_logger(), "corridor_forward_distance: %.2f", corridor_forward_distance_);
  RCLCPP_INFO(
    this->get_logger(), "reconnect_target_distance: %.2f", reconnect_target_distance_);
  RCLCPP_INFO(
    this->get_logger(), "reconnect_deviation_threshold: %.2f", reconnect_deviation_threshold_);
  RCLCPP_INFO(
    this->get_logger(), "reconnect_heading_threshold: %.2f", reconnect_heading_threshold_);
  RCLCPP_INFO(
    this->get_logger(),
    "max_reconnect_failures_before_global_replan: %.0f",
    max_reconnect_failures_before_global_replan_);
  RCLCPP_INFO(
    this->get_logger(),
    "connector_goal_tolerance: %.2f, connector_commit_distance: %.2f, connector_timeout_sec: %.2f",
    connector_goal_tolerance_,
    connector_commit_distance_,
    connector_timeout_sec_);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("awared_global_path", 1);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto loop_time = std::chrono::milliseconds(
    static_cast<int>(1000.0 / recompute_frequency_));
  threading_timer_ = this->create_wall_timer(
    loop_time,
    std::bind(&DWA_GlobalPlanner::determineDWAPlan, this),
    action_server_group_);
  threading_timer_->cancel();

  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_dwa_plan",
    std::bind(&DWA_GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DWA_GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&DWA_GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
}

DWA_GlobalPlanner::~DWA_GlobalPlanner()
{
  action_server_global_planner_.reset();
}

bool DWA_GlobalPlanner::isNewGoal()
{
  std::lock_guard<std::mutex> lock(plan_state_mutex_);
  if (new_goal_.pose.position.x == current_goal_.pose.position.x &&
    new_goal_.pose.position.y == current_goal_.pose.position.y &&
    new_goal_.pose.position.z == current_goal_.pose.position.z &&
    new_goal_.pose.orientation.x == current_goal_.pose.orientation.x &&
    new_goal_.pose.orientation.y == current_goal_.pose.orientation.y &&
    new_goal_.pose.orientation.z == current_goal_.pose.orientation.z &&
    new_goal_.pose.orientation.w == current_goal_.pose.orientation.w)
  {
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Received new goal at: %.2f, %.2f, %.2f",
    new_goal_.pose.position.x,
    new_goal_.pose.position.y,
    new_goal_.pose.position.z);
  return true;
}

void DWA_GlobalPlanner::makePlan(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    new_goal_ = goal_handle->get_goal()->goal;
  }

  if (!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  if (!goal_handle->get_goal()->activate_threading) {
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->succeed(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  if (isNewGoal()) {
    geometry_msgs::msg::PoseStamped start;
    perception_3d_ros_->getGlobalPose(start);

    nav_msgs::msg::Path route_path = global_planner_->makeROSPlan(start, goal_handle->get_goal()->goal);
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();

    if (route_path.poses.empty()) {
      result->path = route_path;
      goal_handle->abort(result);
      SwitchPlannerMode(
        PlannerMode::kSafeStop,
        "new-goal route planning failed",
        ReconnectResult::kGlobalReplanTriggered);
      clearCurrentHandleIfMatches(goal_handle);
      return;
    }

    route_path.header.frame_id = global_frame_;
    route_path.header.stamp = clock_->now();
    PrunePathPrefix(&route_path, start, true);

    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      current_goal_ = new_goal_;
      global_path_ = route_path;
      global_dwa_path_.poses.clear();
      global_dwa_path_.header = global_path_.header;

      pcl_global_path_->points.clear();
      for (const auto & pose : global_path_.poses) {
        pcl::PointXYZ pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.z = pose.pose.position.z;
        pcl_global_path_->push_back(pt);
      }
      kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
      kdtree_global_path_->setInputCloud(pcl_global_path_);

      ++route_version_;
      connector_version_ = 0;
      consecutive_reconnect_failures_ = 0;
      const rclcpp::Time now = clock_->now();
      last_global_replan_time_ = now;
      last_successful_reconnect_time_ = now;
      planner_mode_ = PlannerMode::kRouteFollow;
      active_connector_latched_ = false;
      active_route_anchor_index_ = 0;
      active_connector_start_time_ = now;
      active_connector_route_version_ = route_version_;
      active_connector_path_.poses.clear();
      active_connector_path_.header.frame_id = global_frame_;
      active_connector_path_.header.stamp = now;
      active_connector_has_goal_heading_ = false;
      active_connector_preferred_turn_sign_ = 0;
    }

    result->path = route_path;
    goal_handle->succeed(result);
    pub_path_->publish(route_path);

    threading_timer_->reset();
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  nav_msgs::msg::Path path_to_publish;
  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    path_to_publish = global_dwa_path_.poses.empty() ? global_path_ : global_dwa_path_;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  if (!path_to_publish.poses.empty()) {
    PrunePathPrefix(&path_to_publish, start, true);
  }

  result->path = path_to_publish;
  pub_path_->publish(path_to_publish);
  goal_handle->succeed(result);
  clearCurrentHandleIfMatches(goal_handle);
}

bool DWA_GlobalPlanner::ComputePathTangentYaw(
  const nav_msgs::msg::Path & path,
  std::size_t pivot_index,
  double * yaw) const
{
  if (yaw == nullptr || path.poses.size() < 2) {
    return false;
  }
  if (pivot_index >= path.poses.size()) {
    return false;
  }

  auto compute_yaw_from_pair =
    [&](std::size_t from_index, std::size_t to_index, double * pair_yaw) -> bool
    {
      if (pair_yaw == nullptr) {
        return false;
      }
      if (from_index >= path.poses.size() || to_index >= path.poses.size()) {
        return false;
      }
      const auto & from_pose = path.poses[from_index].pose.position;
      const auto & to_pose = path.poses[to_index].pose.position;
      const double dx = to_pose.x - from_pose.x;
      const double dy = to_pose.y - from_pose.y;
      const double norm = std::hypot(dx, dy);
      if (!std::isfinite(norm) || norm < 1e-4) {
        return false;
      }
      *pair_yaw = std::atan2(dy, dx);
      return std::isfinite(*pair_yaw);
    };

  double tangent_yaw = 0.0;
  if (pivot_index + 1 < path.poses.size() &&
    compute_yaw_from_pair(pivot_index, pivot_index + 1, &tangent_yaw))
  {
    *yaw = tangent_yaw;
    return true;
  }

  if (pivot_index > 0 &&
    compute_yaw_from_pair(pivot_index - 1, pivot_index, &tangent_yaw))
  {
    *yaw = tangent_yaw;
    return true;
  }

  return false;
}

bool DWA_GlobalPlanner::PrunePathPrefix(
  nav_msgs::msg::Path * path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  bool insert_robot_pose) const
{
  if (path == nullptr || path->poses.empty() || !IsFinitePose(robot_pose)) {
    return false;
  }

  const std::size_t nearest_index = FindNearestPathIndex(*path, robot_pose);
  if (nearest_index >= path->poses.size()) {
    return false;
  }

  nav_msgs::msg::Path pruned;
  pruned.header = path->header;
  pruned.header.frame_id = global_frame_;
  pruned.header.stamp = clock_->now();

  auto append_if_non_duplicate = [&](const geometry_msgs::msg::PoseStamped & candidate)
    {
      if (!IsFinitePose(candidate)) {
        return;
      }
      if (pruned.poses.empty()) {
        pruned.poses.push_back(candidate);
        return;
      }
      const auto & last = pruned.poses.back().pose.position;
      const auto & cur = candidate.pose.position;
      if (std::hypot(last.x - cur.x, last.y - cur.y) > 0.02) {
        pruned.poses.push_back(candidate);
      }
    };

  if (insert_robot_pose) {
    geometry_msgs::msg::PoseStamped robot_anchor = robot_pose;
    robot_anchor.header = pruned.header;
    append_if_non_duplicate(robot_anchor);
  }

  for (std::size_t i = nearest_index; i < path->poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose = path->poses[i];
    pose.header = pruned.header;
    append_if_non_duplicate(pose);
  }

  if (pruned.poses.empty()) {
    return false;
  }

  *path = std::move(pruned);
  return true;
}

bool DWA_GlobalPlanner::ExtractCorridor(
  const nav_msgs::msg::Path & route_path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  CorridorSegment * corridor) const
{
  if (corridor == nullptr || route_path.poses.empty() || !IsFinitePose(robot_pose)) {
    return false;
  }

  corridor->path.poses.clear();
  corridor->path.header = route_path.header;
  corridor->path.header.frame_id = global_frame_;
  corridor->path.header.stamp = clock_->now();
  corridor->nearest_index = 0;
  corridor->anchor_index = 0;
  corridor->has_anchor_yaw = false;
  corridor->anchor_yaw = 0.0;

  const std::size_t nearest_index = FindNearestPathIndex(route_path, robot_pose);
  if (nearest_index >= route_path.poses.size()) {
    return false;
  }
  corridor->nearest_index = nearest_index;

  const double forward_distance = std::max(corridor_forward_distance_, reconnect_target_distance_);
  double accumulated_distance = 0.0;
  std::size_t index = nearest_index;

  while (index < route_path.poses.size()) {
    geometry_msgs::msg::PoseStamped pose = route_path.poses[index];
    pose.header = corridor->path.header;
    corridor->path.poses.push_back(pose);

    if (index + 1 >= route_path.poses.size() || accumulated_distance >= forward_distance) {
      break;
    }

    const auto & p0 = route_path.poses[index].pose.position;
    const auto & p1 = route_path.poses[index + 1].pose.position;
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (!std::isfinite(segment_length)) {
      break;
    }
    accumulated_distance += segment_length;
    ++index;
  }

  if (corridor->path.poses.empty()) {
    return false;
  }

  std::size_t anchor_index = nearest_index;
  double anchor_distance = 0.0;
  while (anchor_index + 1 < route_path.poses.size() && anchor_distance < reconnect_target_distance_) {
    const auto & p0 = route_path.poses[anchor_index].pose.position;
    const auto & p1 = route_path.poses[anchor_index + 1].pose.position;
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (!std::isfinite(segment_length)) {
      break;
    }
    anchor_distance += segment_length;
    ++anchor_index;
  }
  corridor->anchor_index = anchor_index;

  double anchor_yaw = 0.0;
  if (ComputePathTangentYaw(route_path, anchor_index, &anchor_yaw)) {
    corridor->has_anchor_yaw = true;
    corridor->anchor_yaw = anchor_yaw;
  }

  return true;
}

bool DWA_GlobalPlanner::IsReconnectRequired(
  const CorridorSegment & corridor,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (corridor.path.poses.empty()) {
    return true;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto & pose : corridor.path.poses) {
    const double dx = pose.pose.position.x - robot_pose.pose.position.x;
    const double dy = pose.pose.position.y - robot_pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  if (std::isfinite(min_distance) && min_distance > reconnect_deviation_threshold_) {
    return true;
  }

  double start_yaw = 0.0;
  if (!ExtractPoseYaw(robot_pose, &start_yaw)) {
    return false;
  }

  double corridor_yaw = 0.0;
  if (!ComputePathTangentYaw(corridor.path, 0, &corridor_yaw)) {
    return false;
  }

  const double heading_error = std::abs(NormalizeAngle(corridor_yaw - start_yaw));
  if (std::isfinite(heading_error) && heading_error > reconnect_heading_threshold_) {
    return true;
  }

  return false;
}

bool DWA_GlobalPlanner::BuildReconnectGoalFromCorridor(
  const CorridorSegment & corridor,
  geometry_msgs::msg::PoseStamped * reconnect_goal,
  bool * has_tangent_yaw,
  std::size_t * anchor_index) const
{
  if (reconnect_goal == nullptr || has_tangent_yaw == nullptr || anchor_index == nullptr) {
    return false;
  }
  if (corridor.path.poses.empty()) {
    return false;
  }

  std::size_t local_anchor_index = 0;
  if (corridor.anchor_index > corridor.nearest_index) {
    local_anchor_index = corridor.anchor_index - corridor.nearest_index;
  }
  if (local_anchor_index >= corridor.path.poses.size()) {
    local_anchor_index = corridor.path.poses.size() - 1;
  }

  *reconnect_goal = corridor.path.poses[local_anchor_index];
  reconnect_goal->header.frame_id = global_frame_;
  reconnect_goal->header.stamp = clock_->now();
  *anchor_index = corridor.anchor_index;

  *has_tangent_yaw = corridor.has_anchor_yaw;
  if (corridor.has_anchor_yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, corridor.anchor_yaw);
    q.normalize();
    reconnect_goal->pose.orientation.x = q.x();
    reconnect_goal->pose.orientation.y = q.y();
    reconnect_goal->pose.orientation.z = q.z();
    reconnect_goal->pose.orientation.w = q.w();
  } else {
    reconnect_goal->pose.orientation.x = 0.0;
    reconnect_goal->pose.orientation.y = 0.0;
    reconnect_goal->pose.orientation.z = 0.0;
    reconnect_goal->pose.orientation.w = 1.0;
  }

  return true;
}

nav_msgs::msg::Path DWA_GlobalPlanner::ComposeConnectorWithRouteTail(
  const nav_msgs::msg::Path & connector_path,
  const nav_msgs::msg::Path & route_path,
  std::size_t route_anchor_index) const
{
  nav_msgs::msg::Path composed;
  composed.header.frame_id = global_frame_;
  composed.header.stamp = clock_->now();

  auto append_if_non_duplicate = [&](const geometry_msgs::msg::PoseStamped & candidate)
    {
      if (!IsFinitePose(candidate)) {
        return;
      }
      if (composed.poses.empty()) {
        geometry_msgs::msg::PoseStamped first = candidate;
        first.header = composed.header;
        composed.poses.push_back(first);
        return;
      }
      const auto & last = composed.poses.back().pose.position;
      const auto & cur = candidate.pose.position;
      if (std::hypot(last.x - cur.x, last.y - cur.y) > 0.02) {
        geometry_msgs::msg::PoseStamped pose = candidate;
        pose.header = composed.header;
        composed.poses.push_back(pose);
      }
    };

  for (const auto & pose : connector_path.poses) {
    append_if_non_duplicate(pose);
  }

  if (route_path.poses.empty()) {
    return composed;
  }

  if (route_anchor_index >= route_path.poses.size()) {
    route_anchor_index = route_path.poses.size() - 1;
  }

  for (std::size_t i = route_anchor_index; i < route_path.poses.size(); ++i) {
    append_if_non_duplicate(route_path.poses[i]);
  }

  return composed;
}

bool DWA_GlobalPlanner::ExtractPoseYaw(
  const geometry_msgs::msg::PoseStamped & pose,
  double * yaw) const
{
  if (yaw == nullptr) {
    return false;
  }
  const auto & q_msg = pose.pose.orientation;
  if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
    !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w))
  {
    return false;
  }
  const double norm =
    q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
  if (norm < 1e-9) {
    return false;
  }
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw_value = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_value);
  if (!std::isfinite(yaw_value)) {
    return false;
  }
  *yaw = NormalizeAngle(yaw_value);
  return true;
}

std::size_t DWA_GlobalPlanner::FindNearestPathIndex(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (path.poses.empty()) {
    return 0;
  }

  std::size_t nearest_index = 0;
  double nearest_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - pose.pose.position.x;
    const double dy = path.poses[i].pose.position.y - pose.pose.position.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_sq) {
      nearest_sq = distance_sq;
      nearest_index = i;
    }
  }
  return nearest_index;
}

int DWA_GlobalPlanner::EstimatePreferredTurnSignAhead(
  const nav_msgs::msg::Path & path,
  std::size_t start_index,
  double max_forward_distance) const
{
  // Estimate turn-side preference from the untraversed local path segment.
  // Do not use historical curvature in the already-traversed path prefix.
  if (path.poses.size() < 3 || start_index >= path.poses.size()) {
    return 0;
  }
  max_forward_distance = std::max(max_forward_distance, 0.0);

  std::size_t end_index = start_index;
  double accumulated_distance = 0.0;
  while (end_index + 1 < path.poses.size() && accumulated_distance < max_forward_distance) {
    const auto & p0 = path.poses[end_index].pose.position;
    const auto & p1 = path.poses[end_index + 1].pose.position;
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (!std::isfinite(segment_length)) {
      break;
    }
    accumulated_distance += segment_length;
    ++end_index;
  }
  if (end_index <= start_index + 1) {
    return 0;
  }

  for (std::size_t i = start_index; i + 2 <= end_index; ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;
    const auto & p2 = path.poses[i + 2].pose.position;
    const double v1x = p1.x - p0.x;
    const double v1y = p1.y - p0.y;
    const double v2x = p2.x - p1.x;
    const double v2y = p2.y - p1.y;
    const double n1 = std::hypot(v1x, v1y);
    const double n2 = std::hypot(v2x, v2y);
    if (!std::isfinite(n1) || !std::isfinite(n2) || n1 < 1e-4 || n2 < 1e-4) {
      continue;
    }

    const double cross = v1x * v2y - v1y * v2x;
    if (!std::isfinite(cross) || std::abs(cross) < 1e-4) {
      continue;
    }
    return cross > 0.0 ? 1 : -1;
  }
  return 0;
}

bool DWA_GlobalPlanner::ShouldLockStartupReplan(
  const nav_msgs::msg::Path & dwa_path,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (startup_replan_lock_distance_ <= 1e-6 || dwa_path.poses.size() < 2) {
    return false;
  }

  std::size_t nearest_index = 0;
  double nearest_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < dwa_path.poses.size(); ++i) {
    const double dx = dwa_path.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = dwa_path.poses[i].pose.position.y - robot_pose.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < nearest_sq) {
      nearest_sq = d2;
      nearest_index = i;
    }
  }

  if (!std::isfinite(nearest_sq)) {
    return false;
  }

  if (nearest_sq >
    startup_replan_lock_offtrack_tolerance_ * startup_replan_lock_offtrack_tolerance_)
  {
    return false;
  }

  double progressed = 0.0;
  for (std::size_t i = 1; i <= nearest_index && i < dwa_path.poses.size(); ++i) {
    const auto & p0 = dwa_path.poses[i - 1].pose.position;
    const auto & p1 = dwa_path.poses[i].pose.position;
    progressed += std::hypot(p1.x - p0.x, p1.y - p0.y);
  }
  return progressed < startup_replan_lock_distance_;
}

double DWA_GlobalPlanner::ComputeDistanceToPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (path.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::size_t nearest_index = FindNearestPathIndex(path, pose);
  if (nearest_index >= path.poses.size()) {
    return std::numeric_limits<double>::infinity();
  }

  const double dx = path.poses[nearest_index].pose.position.x - pose.pose.position.x;
  const double dy = path.poses[nearest_index].pose.position.y - pose.pose.position.y;
  return std::hypot(dx, dy);
}

double DWA_GlobalPlanner::EstimateProgressAlongPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  const std::size_t nearest_index = FindNearestPathIndex(path, pose);
  if (nearest_index == 0 || nearest_index >= path.poses.size()) {
    return 0.0;
  }

  double progressed = 0.0;
  for (std::size_t i = 1; i <= nearest_index && i < path.poses.size(); ++i) {
    const auto & p0 = path.poses[i - 1].pose.position;
    const auto & p1 = path.poses[i].pose.position;
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (!std::isfinite(segment_length)) {
      continue;
    }
    progressed += segment_length;
  }
  return progressed;
}

const char * DWA_GlobalPlanner::PlannerModeToString(PlannerMode mode) const
{
  switch (mode) {
    case PlannerMode::kRouteFollow:
      return "ROUTE_FOLLOW";
    case PlannerMode::kLocalReconnect:
      return "LOCAL_RECONNECT";
    case PlannerMode::kLockConnector:
      return "LOCK_CONNECTOR";
    case PlannerMode::kReplanGlobal:
      return "REPLAN_GLOBAL";
    case PlannerMode::kSafeStop:
      return "SAFE_STOP";
    default:
      return "UNKNOWN";
  }
}

const char * DWA_GlobalPlanner::ReconnectResultToString(ReconnectResult result) const
{
  switch (result) {
    case ReconnectResult::kSuccess:
      return "SUCCESS";
    case ReconnectResult::kRouteProjectionFailed:
      return "ROUTE_PROJECTION_FAILED";
    case ReconnectResult::kConnectorFailed:
      return "CONNECTOR_FAILED";
    case ReconnectResult::kCorridorEmpty:
      return "CORRIDOR_EMPTY";
    case ReconnectResult::kReusePrunedPath:
      return "REUSE_PRUNED_PATH";
    case ReconnectResult::kStartupLockReusedPath:
      return "STARTUP_LOCK_REUSED_PATH";
    case ReconnectResult::kGlobalReplanTriggered:
      return "GLOBAL_REPLAN_TRIGGERED";
    default:
      return "UNKNOWN";
  }
}

void DWA_GlobalPlanner::SwitchPlannerMode(
  PlannerMode next_mode,
  const std::string & reason,
  ReconnectResult result)
{
  PlannerMode previous_mode;
  std::size_t route_version = 0;
  std::size_t connector_version = 0;
  std::size_t reconnect_failures = 0;

  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    previous_mode = planner_mode_;
    route_version = route_version_;
    connector_version = connector_version_;
    reconnect_failures = consecutive_reconnect_failures_;
    planner_mode_ = next_mode;
  }

  if (previous_mode != next_mode) {
    RCLCPP_INFO(
      this->get_logger(),
      "PlannerMode %s -> %s, reason=%s, result=%s, route_v=%zu, connector_v=%zu, reconnect_failures=%zu",
      PlannerModeToString(previous_mode),
      PlannerModeToString(next_mode),
      reason.c_str(),
      ReconnectResultToString(result),
      route_version,
      connector_version,
      reconnect_failures);
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "PlannerMode keep %s, reason=%s, result=%s",
      PlannerModeToString(next_mode),
      reason.c_str(),
      ReconnectResultToString(result));
  }
}

void DWA_GlobalPlanner::determineDWAPlan()
{
  nav_msgs::msg::Path route_snapshot;
  nav_msgs::msg::Path connector_snapshot;
  geometry_msgs::msg::PoseStamped goal_snapshot;
  std::size_t route_version_snapshot = 0;
  bool active_connector_latched_snapshot = false;
  geometry_msgs::msg::PoseStamped active_reconnect_goal_snapshot;
  std::size_t active_route_anchor_index_snapshot = 0;
  rclcpp::Time active_connector_start_time_snapshot(0, 0, RCL_ROS_TIME);
  std::size_t active_connector_route_version_snapshot = 0;
  nav_msgs::msg::Path active_connector_path_snapshot;
  bool active_connector_has_goal_heading_snapshot = false;
  int active_connector_preferred_turn_sign_snapshot = 0;

  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    if (global_path_.poses.empty()) {
      return;
    }
    route_snapshot = global_path_;
    connector_snapshot = global_dwa_path_;
    goal_snapshot = current_goal_;
    route_version_snapshot = route_version_;
    active_connector_latched_snapshot = active_connector_latched_;
    active_reconnect_goal_snapshot = active_reconnect_goal_;
    active_route_anchor_index_snapshot = active_route_anchor_index_;
    active_connector_start_time_snapshot = active_connector_start_time_;
    active_connector_route_version_snapshot = active_connector_route_version_;
    active_connector_path_snapshot = active_connector_path_;
    active_connector_has_goal_heading_snapshot = active_connector_has_goal_heading_;
    active_connector_preferred_turn_sign_snapshot = active_connector_preferred_turn_sign_;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  if (!IsFinitePose(start)) {
    return;
  }

  const rclcpp::Time now = clock_->now();

  if (!PrunePathPrefix(&route_snapshot, start, true)) {
    SwitchPlannerMode(
      PlannerMode::kSafeStop,
      "failed to prune route prefix",
      ReconnectResult::kRouteProjectionFailed);
    return;
  }

  if (!connector_snapshot.poses.empty()) {
    if (!PrunePathPrefix(&connector_snapshot, start, true)) {
      connector_snapshot.poses.clear();
    }
  }

  auto publish_path = [&](const nav_msgs::msg::Path & path)
    {
      nav_msgs::msg::Path output = path;
      output.header.frame_id = global_frame_;
      output.header.stamp = clock_->now();
      pub_path_->publish(output);
    };

  auto store_connector_path = [&](const nav_msgs::msg::Path & path)
    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      global_dwa_path_ = path;
    };

  auto clear_active_connector_latch = [&](const std::string & reason)
    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      const bool was_latched = active_connector_latched_;
      active_connector_latched_ = false;
      active_route_anchor_index_ = 0;
      active_connector_start_time_ = now;
      active_connector_route_version_ = route_version_;
      active_connector_path_.poses.clear();
      active_connector_path_.header.frame_id = global_frame_;
      active_connector_path_.header.stamp = now;
      active_connector_has_goal_heading_ = false;
      active_connector_preferred_turn_sign_ = 0;
      if (was_latched) {
        RCLCPP_INFO(this->get_logger(), "Release latched connector: %s", reason.c_str());
      }
    };

  auto set_reconnect_failure_count = [&](std::size_t count)
    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      consecutive_reconnect_failures_ = count;
    };

  auto get_reconnect_failure_count = [&]() -> std::size_t
    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      return consecutive_reconnect_failures_;
    };

  auto trigger_route_replan = [&](const std::string & reason) -> bool
    {
      constexpr double kMinGlobalReplanIntervalSec = 2.0;
      double elapsed_sec = 0.0;
      {
        std::lock_guard<std::mutex> lock(plan_state_mutex_);
        elapsed_sec = (now - last_global_replan_time_).seconds();
      }
      if (elapsed_sec < kMinGlobalReplanIntervalSec) {
        return false;
      }

      SwitchPlannerMode(
        PlannerMode::kReplanGlobal,
        reason,
        ReconnectResult::kGlobalReplanTriggered);

      nav_msgs::msg::Path replanned_route = global_planner_->makeROSPlan(start, goal_snapshot);
      if (replanned_route.poses.empty()) {
        SwitchPlannerMode(
          PlannerMode::kSafeStop,
          "route replan failed",
          ReconnectResult::kGlobalReplanTriggered);
        return false;
      }

      if (!PrunePathPrefix(&replanned_route, start, true)) {
        SwitchPlannerMode(
          PlannerMode::kSafeStop,
          "route replan prune failed",
          ReconnectResult::kRouteProjectionFailed);
        return false;
      }

      {
        std::lock_guard<std::mutex> lock(plan_state_mutex_);
        global_path_ = replanned_route;
        global_dwa_path_.poses.clear();
        global_dwa_path_.header = replanned_route.header;
        ++route_version_;
        connector_version_ = 0;
        consecutive_reconnect_failures_ = 0;
        last_global_replan_time_ = now;
        active_connector_latched_ = false;
        active_route_anchor_index_ = 0;
        active_connector_start_time_ = now;
        active_connector_route_version_ = route_version_;
        active_connector_path_.poses.clear();
        active_connector_path_.header.frame_id = global_frame_;
        active_connector_path_.header.stamp = now;
        active_connector_has_goal_heading_ = false;
        active_connector_preferred_turn_sign_ = 0;

        pcl_global_path_->points.clear();
        for (const auto & pose : global_path_.poses) {
          pcl::PointXYZ pt;
          pt.x = pose.pose.position.x;
          pt.y = pose.pose.position.y;
          pt.z = pose.pose.position.z;
          pcl_global_path_->push_back(pt);
        }
        kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        kdtree_global_path_->setInputCloud(pcl_global_path_);
      }

      publish_path(replanned_route);
      SwitchPlannerMode(
        PlannerMode::kRouteFollow,
        "route replan success",
        ReconnectResult::kGlobalReplanTriggered);
      return true;
    };

  const std::size_t max_reconnect_failures =
    static_cast<std::size_t>(std::ceil(max_reconnect_failures_before_global_replan_));

  if (active_connector_latched_snapshot) {
    // Latched connector avoids chasing a moving reconnect anchor while the
    // robot is executing the current reconnect maneuver.
    bool should_release_latch = false;
    std::string release_reason;

    if (active_connector_route_version_snapshot != route_version_snapshot) {
      should_release_latch = true;
      release_reason = "route version changed";
    } else if (ComputeDistanceToPath(active_connector_path_snapshot, start) >
      reconnect_deviation_threshold_ * 2.5)
    {
      should_release_latch = true;
      release_reason = "robot deviated from active connector";
    } else {
      const double distance_to_goal = std::hypot(
        active_reconnect_goal_snapshot.pose.position.x - start.pose.position.x,
        active_reconnect_goal_snapshot.pose.position.y - start.pose.position.y);
      if (std::isfinite(distance_to_goal) && distance_to_goal < connector_goal_tolerance_) {
        should_release_latch = true;
        release_reason = "reached active reconnect goal";
      }
    }

    if (!should_release_latch) {
      const double connector_progress =
        EstimateProgressAlongPath(active_connector_path_snapshot, start);
      if (std::isfinite(connector_progress) && connector_progress >= connector_commit_distance_) {
        should_release_latch = true;
        release_reason = "connector commit distance reached";
      }
    }

    if (!should_release_latch) {
      const double elapsed = (now - active_connector_start_time_snapshot).seconds();
      if (std::isfinite(elapsed) && elapsed > connector_timeout_sec_) {
        should_release_latch = true;
        release_reason = "connector latch timeout";
      }
    }

    if (should_release_latch) {
      clear_active_connector_latch(release_reason);
      active_connector_latched_snapshot = false;
      active_connector_path_snapshot.poses.clear();
      connector_snapshot = route_snapshot;
      PrunePathPrefix(&connector_snapshot, start, true);
      SwitchPlannerMode(
        PlannerMode::kRouteFollow,
        std::string("release connector latch: ") + release_reason,
        ReconnectResult::kReusePrunedPath);
    } else {
      nav_msgs::msg::Path locked_path = connector_snapshot;
      if (locked_path.poses.empty() || !PrunePathPrefix(&locked_path, start, true)) {
        nav_msgs::msg::Path retry_connector = global_planner_->makeROSPlan(
          start,
          active_reconnect_goal_snapshot,
          false,
          active_connector_has_goal_heading_snapshot,
          active_connector_preferred_turn_sign_snapshot);

        if (retry_connector.poses.empty()) {
          const std::size_t failure_count = get_reconnect_failure_count() + 1;
          set_reconnect_failure_count(failure_count);

          if (failure_count >= max_reconnect_failures &&
            trigger_route_replan("latched connector failed repeatedly"))
          {
            return;
          }

          clear_active_connector_latch("latched connector replan failed");
          nav_msgs::msg::Path fallback_path = route_snapshot;
          if (PrunePathPrefix(&fallback_path, start, true)) {
            store_connector_path(fallback_path);
            publish_path(fallback_path);
            SwitchPlannerMode(
              PlannerMode::kRouteFollow,
              "latched connector failed, fallback to route",
              ReconnectResult::kConnectorFailed);
          } else {
            SwitchPlannerMode(
              PlannerMode::kSafeStop,
              "latched connector failed and fallback route invalid",
              ReconnectResult::kConnectorFailed);
          }
          return;
        }

        nav_msgs::msg::Path composed_full =
          ComposeConnectorWithRouteTail(
          retry_connector,
          route_snapshot,
          active_route_anchor_index_snapshot);
        nav_msgs::msg::Path composed_to_publish = composed_full;
        if (composed_full.poses.empty() || !PrunePathPrefix(&composed_to_publish, start, true)) {
          clear_active_connector_latch("latched connector compose failed");
          SwitchPlannerMode(
            PlannerMode::kRouteFollow,
            "latched connector compose failed, release latch",
            ReconnectResult::kConnectorFailed);
          store_connector_path(route_snapshot);
          publish_path(route_snapshot);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(plan_state_mutex_);
          global_dwa_path_ = composed_to_publish;
          active_connector_path_ = composed_full;
          ++connector_version_;
          consecutive_reconnect_failures_ = 0;
          last_successful_reconnect_time_ = now;
        }

        publish_path(composed_to_publish);
        SwitchPlannerMode(
          PlannerMode::kLockConnector,
          "refresh connector path to latched goal",
          ReconnectResult::kSuccess);
        return;
      }

      set_reconnect_failure_count(0);
      store_connector_path(locked_path);
      publish_path(locked_path);
      SwitchPlannerMode(
        PlannerMode::kLockConnector,
        "keep latched connector",
        ReconnectResult::kSuccess);
      return;
    }
  }

  if (!connector_snapshot.poses.empty() && ShouldLockStartupReplan(connector_snapshot, start)) {
    store_connector_path(connector_snapshot);
    publish_path(connector_snapshot);
    SwitchPlannerMode(
      PlannerMode::kLockConnector,
      "startup lock active, reuse current connector",
      ReconnectResult::kStartupLockReusedPath);
    return;
  }

  // Route layer: route_snapshot carries global route semantics.
  // Corridor layer: extract a local forward window from the pruned route.
  CorridorSegment corridor;
  if (!ExtractCorridor(route_snapshot, start, &corridor)) {
    const std::size_t failure_count = get_reconnect_failure_count() + 1;
    set_reconnect_failure_count(failure_count);

    if (failure_count >= max_reconnect_failures &&
      trigger_route_replan("corridor extraction failed repeatedly"))
    {
      return;
    }

    nav_msgs::msg::Path fallback_path = !connector_snapshot.poses.empty() ? connector_snapshot : route_snapshot;
    if (PrunePathPrefix(&fallback_path, start, true)) {
      store_connector_path(fallback_path);
      publish_path(fallback_path);
      SwitchPlannerMode(
        PlannerMode::kRouteFollow,
        "corridor empty, reuse pruned path",
        ReconnectResult::kCorridorEmpty);
      return;
    }

    SwitchPlannerMode(
      PlannerMode::kSafeStop,
      "corridor empty and no reusable path",
      ReconnectResult::kCorridorEmpty);
    return;
  }

  // Connector layer: only reconnect when corridor tracking is not feasible.
  const bool reconnect_required =
    connector_snapshot.poses.empty() || IsReconnectRequired(corridor, start);

  if (!reconnect_required) {
    set_reconnect_failure_count(0);
    clear_active_connector_latch("corridor tracking became feasible");

    nav_msgs::msg::Path follow_path =
      !connector_snapshot.poses.empty() ? connector_snapshot : route_snapshot;
    if (!PrunePathPrefix(&follow_path, start, true)) {
      follow_path = route_snapshot;
    }

    store_connector_path(follow_path);
    publish_path(follow_path);
    SwitchPlannerMode(
      PlannerMode::kRouteFollow,
      "corridor tracking feasible, skip reconnect",
      ReconnectResult::kReusePrunedPath);
    return;
  }

  SwitchPlannerMode(
    PlannerMode::kLocalReconnect,
    "corridor deviation or heading mismatch",
    ReconnectResult::kSuccess);

  geometry_msgs::msg::PoseStamped reconnect_goal;
  bool has_tangent_yaw = false;
  std::size_t route_anchor_index = 0;
  if (!BuildReconnectGoalFromCorridor(
      corridor,
      &reconnect_goal,
      &has_tangent_yaw,
      &route_anchor_index))
  {
    const std::size_t failure_count = get_reconnect_failure_count() + 1;
    set_reconnect_failure_count(failure_count);

    SwitchPlannerMode(
      PlannerMode::kRouteFollow,
      "failed to build reconnect goal, reuse route",
      ReconnectResult::kReusePrunedPath);
    store_connector_path(route_snapshot);
    publish_path(route_snapshot);
    return;
  }

  int preferred_turn_sign = 0;
  const nav_msgs::msg::Path & turn_sign_path =
    !connector_snapshot.poses.empty() ? connector_snapshot : route_snapshot;
  const std::size_t turn_sign_start_index = FindNearestPathIndex(turn_sign_path, start);
  preferred_turn_sign = EstimatePreferredTurnSignAhead(
    turn_sign_path,
    turn_sign_start_index,
    preferred_turn_sign_lookahead_distance_);

  bool force_use_goal_heading = false;
  if (has_tangent_yaw) {
    double start_yaw = 0.0;
    if (ExtractPoseYaw(start, &start_yaw)) {
      const double dx = reconnect_goal.pose.position.x - start.pose.position.x;
      const double dy = reconnect_goal.pose.position.y - start.pose.position.y;
      const double start_goal_distance = std::hypot(dx, dy);
      const double heading_error = std::abs(NormalizeAngle(corridor.anchor_yaw - start_yaw));
      force_use_goal_heading =
        std::isfinite(start_goal_distance) &&
        std::isfinite(heading_error) &&
        start_goal_distance > min_force_goal_heading_distance_ &&
        heading_error < max_force_goal_heading_angle_;
    }
  }

  // ForwardHybridAStar is used as a short-horizon connector planner here.
  nav_msgs::msg::Path connector_path = global_planner_->makeROSPlan(
    start,
    reconnect_goal,
    false,
    force_use_goal_heading,
    preferred_turn_sign);

  if (connector_path.poses.empty()) {
    const std::size_t failure_count = get_reconnect_failure_count() + 1;
    set_reconnect_failure_count(failure_count);

    if (failure_count >= max_reconnect_failures &&
      trigger_route_replan("connector planning failed repeatedly"))
    {
      return;
    }

    nav_msgs::msg::Path fallback_path = !connector_snapshot.poses.empty() ? connector_snapshot : route_snapshot;
    if (PrunePathPrefix(&fallback_path, start, true)) {
      store_connector_path(fallback_path);
      publish_path(fallback_path);
      SwitchPlannerMode(
        PlannerMode::kRouteFollow,
        "connector failed, reuse pruned path",
        ReconnectResult::kConnectorFailed);
      return;
    }

    SwitchPlannerMode(
      PlannerMode::kSafeStop,
      "connector failed and no reusable path",
      ReconnectResult::kConnectorFailed);
    return;
  }

  nav_msgs::msg::Path composed_path_full =
    ComposeConnectorWithRouteTail(connector_path, route_snapshot, route_anchor_index);
  nav_msgs::msg::Path composed_path_to_publish = composed_path_full;
  if (composed_path_full.poses.empty() || !PrunePathPrefix(&composed_path_to_publish, start, true)) {
    const std::size_t failure_count = get_reconnect_failure_count() + 1;
    set_reconnect_failure_count(failure_count);

    SwitchPlannerMode(
      PlannerMode::kRouteFollow,
      "connector composition failed, reuse route",
      ReconnectResult::kReusePrunedPath);
    store_connector_path(route_snapshot);
    publish_path(route_snapshot);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    global_dwa_path_ = composed_path_to_publish;
    active_connector_latched_ = true;
    active_reconnect_goal_ = reconnect_goal;
    active_route_anchor_index_ = route_anchor_index;
    active_connector_start_time_ = now;
    active_connector_route_version_ = route_version_snapshot;
    active_connector_path_ = composed_path_full;
    active_connector_start_pose_ = start;
    active_connector_has_goal_heading_ = force_use_goal_heading;
    active_connector_preferred_turn_sign_ = preferred_turn_sign;
    ++connector_version_;
    consecutive_reconnect_failures_ = 0;
    last_successful_reconnect_time_ = now;
  }

  publish_path(composed_path_to_publish);
  SwitchPlannerMode(
    PlannerMode::kLockConnector,
    "new connector latched to fixed reconnect goal",
    ReconnectResult::kSuccess);
}

}  // namespace global_planner
