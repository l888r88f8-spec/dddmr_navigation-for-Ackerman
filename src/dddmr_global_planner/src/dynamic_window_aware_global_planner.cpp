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

using namespace std::chrono_literals;

namespace global_planner
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

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

DWA_GlobalPlanner::DWA_GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse DWA_GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
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
  if(current_handle_ != goal_handle){
    return false;
  }
  current_handle_.reset();
  return true;
}

void DWA_GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
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
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DWA_GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void DWA_GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner){

  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pcl_global_path_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  declare_parameter("look_ahead_distance", rclcpp::ParameterValue(5.0));
  this->get_parameter("look_ahead_distance", look_ahead_distance_);
  RCLCPP_INFO(this->get_logger(), "look_ahead_distance: %.2f", look_ahead_distance_);    
  declare_parameter("recompute_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("recompute_frequency", recompute_frequency_);
  RCLCPP_INFO(this->get_logger(), "recompute_frequency: %.2f", recompute_frequency_);     
  declare_parameter("startup_replan_lock_distance", rclcpp::ParameterValue(0.8));
  this->get_parameter("startup_replan_lock_distance", startup_replan_lock_distance_);
  RCLCPP_INFO(this->get_logger(), "startup_replan_lock_distance: %.2f", startup_replan_lock_distance_);
  declare_parameter("startup_replan_lock_offtrack_tolerance", rclcpp::ParameterValue(1.0));
  this->get_parameter(
    "startup_replan_lock_offtrack_tolerance",
    startup_replan_lock_offtrack_tolerance_);
  RCLCPP_INFO(
    this->get_logger(),
    "startup_replan_lock_offtrack_tolerance: %.2f",
    startup_replan_lock_offtrack_tolerance_);
  declare_parameter("preferred_turn_sign_lookahead_distance", rclcpp::ParameterValue(1.8));
  this->get_parameter(
    "preferred_turn_sign_lookahead_distance",
    preferred_turn_sign_lookahead_distance_);
  preferred_turn_sign_lookahead_distance_ = std::max(preferred_turn_sign_lookahead_distance_, 0.5);
  RCLCPP_INFO(
    this->get_logger(),
    "preferred_turn_sign_lookahead_distance: %.2f",
    preferred_turn_sign_lookahead_distance_);
  declare_parameter("min_force_goal_heading_distance", rclcpp::ParameterValue(3.0));
  this->get_parameter("min_force_goal_heading_distance", min_force_goal_heading_distance_);
  min_force_goal_heading_distance_ = std::max(min_force_goal_heading_distance_, 0.0);
  RCLCPP_INFO(
    this->get_logger(),
    "min_force_goal_heading_distance: %.2f",
    min_force_goal_heading_distance_);
  declare_parameter("max_force_goal_heading_angle", rclcpp::ParameterValue(1.75));
  this->get_parameter("max_force_goal_heading_angle", max_force_goal_heading_angle_);
  max_force_goal_heading_angle_ = std::clamp(max_force_goal_heading_angle_, 0.0, kPi);
  RCLCPP_INFO(
    this->get_logger(),
    "max_force_goal_heading_angle: %.2f",
    max_force_goal_heading_angle_);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("awared_global_path", 1);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;

  auto loop_time = std::chrono::milliseconds(int(1000/recompute_frequency_));
  threading_timer_ = this->create_wall_timer(loop_time, std::bind(&DWA_GlobalPlanner::determineDWAPlan, this), action_server_group_);
  threading_timer_->cancel();
  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_dwa_plan",
    std::bind(&DWA_GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DWA_GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&DWA_GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

DWA_GlobalPlanner::~DWA_GlobalPlanner(){
  action_server_global_planner_.reset();
}

bool DWA_GlobalPlanner::isNewGoal(){
  std::lock_guard<std::mutex> lock(plan_state_mutex_);
  if(new_goal_.pose.position.x==current_goal_.pose.position.x && 
      new_goal_.pose.position.y==current_goal_.pose.position.y && 
        new_goal_.pose.position.z==current_goal_.pose.position.z && 
          new_goal_.pose.orientation.x==current_goal_.pose.orientation.x && 
            new_goal_.pose.orientation.y==current_goal_.pose.orientation.y && 
              new_goal_.pose.orientation.z==current_goal_.pose.orientation.z && 
                new_goal_.pose.orientation.w==current_goal_.pose.orientation.w)
     {
      
      return false;
     }
  
  RCLCPP_INFO(this->get_logger(), "Received new goal at: %.2f, %.2f, %.2f", new_goal_.pose.position.x, new_goal_.pose.position.y, new_goal_.pose.position.z);
  return true;
}

void DWA_GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    new_goal_ = goal_handle->get_goal()->goal;
  }

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->succeed(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }
  
  if(isNewGoal()){
    
    geometry_msgs::msg::PoseStamped start;
    perception_3d_ros_->getGlobalPose(start);

    nav_msgs::msg::Path global_path = global_planner_->makeROSPlan(start, goal_handle->get_goal()->goal);
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    result->path = global_path;

    if(global_path.poses.empty()){
      goal_handle->abort(result);
    }
    else{
      {
        std::lock_guard<std::mutex> lock(plan_state_mutex_);
        current_goal_ = new_goal_;
        global_path_ = global_path;
        global_dwa_path_.poses.clear();
        global_dwa_path_.header = global_path.header;

        pcl_global_path_->points.clear();
        for(const auto& pose : global_path_.poses){
          pcl::PointXYZ pt;
          pt.x = pose.pose.position.x;
          pt.y = pose.pose.position.y;
          pt.z = pose.pose.position.z;
          pcl_global_path_->push_back(pt);
        }

        kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        kdtree_global_path_->setInputCloud(pcl_global_path_);
      }
      result->path = global_path;
      goal_handle->succeed(result);

      threading_timer_->reset();
    }
    pub_path_->publish(global_path);
  }
  else{
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    nav_msgs::msg::Path path_to_publish;
    {
      std::lock_guard<std::mutex> lock(plan_state_mutex_);
      if(global_dwa_path_.poses.empty()){
        path_to_publish = global_path_;
      }
      else{
        path_to_publish = global_dwa_path_;
      }
    }
    result->path = path_to_publish;
    pub_path_->publish(path_to_publish);
    goal_handle->succeed(result);
  }
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

void DWA_GlobalPlanner::determineDWAPlan(){

  nav_msgs::msg::Path global_path_snapshot;
  nav_msgs::msg::Path global_dwa_path_snapshot;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_path_snapshot(new pcl::PointCloud<pcl::PointXYZ>);
  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    if(global_path_.poses.empty() || pcl_global_path_->points.empty()){
      return;
    }
    global_path_snapshot = global_path_;
    global_dwa_path_snapshot = global_dwa_path_;
    *pcl_global_path_snapshot = *pcl_global_path_;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  if(ShouldLockStartupReplan(global_dwa_path_snapshot, start)){
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Keep current DWA global path during startup lock window.");
    return;
  }

  pcl::PointXYZ start_pt;
  start_pt.x = start.pose.position.x; start_pt.y = start.pose.position.y; start_pt.z = start.pose.position.z;
  std::vector<float> resultant_distances;
  std::vector<int> indices;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_global_path_snapshot;
  kdtree_global_path_snapshot.setInputCloud(pcl_global_path_snapshot);
  kdtree_global_path_snapshot.nearestKSearch(start_pt, 1, indices, resultant_distances);

  if(indices.empty()){
    RCLCPP_INFO(this->get_logger(), "k-NN search fail, something wrong.");
    return;
  }
  //@protect kdtree_ground
  std::unique_lock<std::recursive_mutex> lock(perception_3d_ros_->getSharedDataPtr()->ground_kdtree_cb_mutex_);
  //@check is dwa goal being blocked, if yes, shift ahead distance 1.0 m
  bool dwa_goal_clear = false;
  double look_ahead_step = 0.0;
  int dwa_pivot = 0;
  while(rclcpp::ok() && !dwa_goal_clear){

    double accumulative_distance = 0.0;
    int pivot = indices[0];
    pcl::PointXYZ last_point = pcl_global_path_snapshot->points[pivot];
    while(accumulative_distance<look_ahead_distance_+look_ahead_step && pivot<pcl_global_path_snapshot->points.size()-1){
      pcl::PointXYZ current_point = pcl_global_path_snapshot->points[pivot];
      double dx = current_point.x - last_point.x;
      double dy = current_point.y - last_point.y;
      double dz = current_point.z - last_point.z;
      accumulative_distance += sqrt(dx*dx + dy*dy + dz*dz);
      last_point = current_point;
      pivot++;
    }

    if(pivot>=pcl_global_path_snapshot->points.size()-1){
      dwa_pivot = pcl_global_path_snapshot->points.size()-1;
      if(recompute_frequency_>2.0)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "DWA goal reach the global path end at: %.2f, %.2f, %.2f", pcl_global_path_snapshot->points[dwa_pivot].x, pcl_global_path_snapshot->points[dwa_pivot].y, pcl_global_path_snapshot->points[dwa_pivot].z);
      else
        RCLCPP_INFO(this->get_logger(), "DWA goal reach the global path end at: %.2f, %.2f, %.2f", pcl_global_path_snapshot->points[dwa_pivot].x, pcl_global_path_snapshot->points[dwa_pivot].y, pcl_global_path_snapshot->points[dwa_pivot].z);
      break;
    }

    if(look_ahead_distance_+look_ahead_step>100.0){
      dwa_pivot = pcl_global_path_snapshot->points.size()-1;
      RCLCPP_INFO(this->get_logger(), "Look ahead distance reach unrealistic distance.");
      break;      
    }
    //check nothing is around dwa goal
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZI ipt;
    ipt.x = pcl_global_path_snapshot->points[pivot].x;
    ipt.y = pcl_global_path_snapshot->points[pivot].y;
    ipt.z = pcl_global_path_snapshot->points[pivot].z;
    perception_3d_ros_->getSharedDataPtr()->kdtree_ground_->radiusSearch(ipt, 0.25, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    
    double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
    if(pointIdxRadiusSearch.empty()){
      look_ahead_step+=1.0;
      dwa_goal_clear = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000,  "No ground is found for DWA goal at: %.2f, %.2f, %.2f", pcl_global_path_snapshot->points[pivot].x, pcl_global_path_snapshot->points[pivot].y, pcl_global_path_snapshot->points[pivot].z);
    }
    else{
      for(auto it=pointIdxRadiusSearch.begin();it!=pointIdxRadiusSearch.end();it++){
        //@ dGraphValue is the distance to lethal
        double dGraphValue = perception_3d_ros_->get_min_dGraphValue((*it));
        /*This is for lethal*/
        if(dGraphValue<inscribed_radius){
          look_ahead_step+=1.0;
          dwa_goal_clear = false;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "DWA goal is blocked at: %.2f, %.2f, %.2f", pcl_global_path_snapshot->points[pivot].x, pcl_global_path_snapshot->points[pivot].y, pcl_global_path_snapshot->points[pivot].z);
          break;
        }
        else{
          dwa_goal_clear = true;
        }
      }
    }
    dwa_pivot = pivot;
  }

  geometry_msgs::msg::PoseStamped dwa_goal;
  dwa_goal.header.frame_id = global_frame_;
  dwa_goal.header.stamp = clock_->now();
  dwa_goal.pose.position.x = pcl_global_path_snapshot->points[dwa_pivot].x;
  dwa_goal.pose.position.y = pcl_global_path_snapshot->points[dwa_pivot].y;
  dwa_goal.pose.position.z = pcl_global_path_snapshot->points[dwa_pivot].z;
  dwa_goal.pose.orientation.w = 1.0;

  double tangent_yaw = 0.0;
  const bool has_tangent_yaw = ComputePathTangentYaw(
    global_path_snapshot,
    static_cast<std::size_t>(dwa_pivot),
    &tangent_yaw);
  if(has_tangent_yaw){
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, tangent_yaw);
    q.normalize();
    dwa_goal.pose.orientation.x = q.x();
    dwa_goal.pose.orientation.y = q.y();
    dwa_goal.pose.orientation.z = q.z();
    dwa_goal.pose.orientation.w = q.w();
  }
  else{
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Failed to compute tangent yaw for DWA pivot, fallback to position-only heading.");
  }

  int preferred_turn_sign = 0;
  if (!global_dwa_path_snapshot.poses.empty()) {
    const std::size_t nearest_index =
      FindNearestPathIndex(global_dwa_path_snapshot, start);
    preferred_turn_sign = EstimatePreferredTurnSignAhead(
      global_dwa_path_snapshot,
      nearest_index,
      preferred_turn_sign_lookahead_distance_);
  }

  bool force_use_goal_heading = false;
  if (has_tangent_yaw) {
    double start_yaw = 0.0;
    if (ExtractPoseYaw(start, &start_yaw)) {
      const double dx = dwa_goal.pose.position.x - start.pose.position.x;
      const double dy = dwa_goal.pose.position.y - start.pose.position.y;
      const double start_goal_distance = std::hypot(dx, dy);
      const double heading_error = std::abs(NormalizeAngle(tangent_yaw - start_yaw));
      force_use_goal_heading =
        std::isfinite(start_goal_distance) &&
        std::isfinite(heading_error) &&
        start_goal_distance > min_force_goal_heading_distance_ &&
        heading_error < max_force_goal_heading_angle_;
    }
  }
  nav_msgs::msg::Path dwa_path = global_planner_->makeROSPlan(
    start, dwa_goal, false, force_use_goal_heading, preferred_turn_sign);
  dwa_path.header.frame_id = global_frame_;
  dwa_path.header.stamp = clock_->now();

  if(dwa_path.poses.empty()){
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Failed to refresh DWA path from current pose, keep the previous valid path.");
    return;
  }

  size_t safe_pivot = static_cast<size_t>(dwa_pivot);
  if(safe_pivot >= global_path_snapshot.poses.size()){
    safe_pivot = global_path_snapshot.poses.size() - 1;
  }

  for(size_t i=safe_pivot; i<global_path_snapshot.poses.size(); i++){
    dwa_path.poses.push_back(global_path_snapshot.poses[i]);
  }
  dwa_path.poses.push_back(global_path_snapshot.poses.back());
  {
    std::lock_guard<std::mutex> lock(plan_state_mutex_);
    global_dwa_path_ = dwa_path;
  }
}

}
