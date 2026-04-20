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
#include <global_planner/global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

GlobalPlanner::GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
  debug_goal_seq_ = 0;
  debug_route_version_ = 0;
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool GlobalPlanner::clearCurrentHandleIfMatches(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  std::lock_guard<std::mutex> lock(current_handle_mutex_);
  if(current_handle_ != goal_handle){
    return false;
  }
  current_handle_.reset();
  return true;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
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
  std::thread{std::bind(&GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){
  
  static_ground_size_ = 0;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  
  declare_parameter("turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("turning_weight", turning_weight_);
  RCLCPP_INFO(this->get_logger(), "turning_weight: %.2f", turning_weight_);    

  declare_parameter("enable_detail_log", rclcpp::ParameterValue(false));
  this->get_parameter("enable_detail_log", enable_detail_log_);
  RCLCPP_INFO(this->get_logger(), "enable_detail_log: %d", enable_detail_log_);    

  declare_parameter("a_star_expanding_radius", rclcpp::ParameterValue(0.5));
  this->get_parameter("a_star_expanding_radius", a_star_expanding_radius_);
  RCLCPP_INFO(this->get_logger(), "a_star_expanding_radius: %.2f", a_star_expanding_radius_);    

  declare_parameter("direct_path_distance_threshold", rclcpp::ParameterValue(3.0));
  this->get_parameter("direct_path_distance_threshold", direct_path_distance_threshold_);
  RCLCPP_INFO(this->get_logger(), "direct_path_distance_threshold: %.2f", direct_path_distance_threshold_);

  declare_parameter("use_forward_hybrid_astar", rclcpp::ParameterValue(true));
  this->get_parameter("use_forward_hybrid_astar", use_forward_hybrid_astar_);
  RCLCPP_INFO(this->get_logger(), "use_forward_hybrid_astar: %d", use_forward_hybrid_astar_);

  declare_parameter("enable_direct_path_shortcut", rclcpp::ParameterValue(false));
  this->get_parameter("enable_direct_path_shortcut", enable_direct_path_shortcut_);
  RCLCPP_INFO(this->get_logger(), "enable_direct_path_shortcut: %d", enable_direct_path_shortcut_);

  declare_parameter("hybrid_astar.wheelbase", rclcpp::ParameterValue(0.549185));
  this->get_parameter("hybrid_astar.wheelbase", forward_hybrid_astar_config_.wheelbase);
  declare_parameter("hybrid_astar.max_steer", rclcpp::ParameterValue(0.69));
  this->get_parameter("hybrid_astar.max_steer", forward_hybrid_astar_config_.max_steer);
  declare_parameter("hybrid_astar.heading_bin_count", rclcpp::ParameterValue(72));
  this->get_parameter("hybrid_astar.heading_bin_count", forward_hybrid_astar_config_.heading_bin_count);
  declare_parameter("hybrid_astar.primitive_length", rclcpp::ParameterValue(0.6));
  this->get_parameter("hybrid_astar.primitive_length", forward_hybrid_astar_config_.primitive_length);
  declare_parameter("hybrid_astar.primitive_step", rclcpp::ParameterValue(0.05));
  this->get_parameter("hybrid_astar.primitive_step", forward_hybrid_astar_config_.primitive_step);
  declare_parameter("hybrid_astar.projection_search_radius", rclcpp::ParameterValue(0.6));
  this->get_parameter("hybrid_astar.projection_search_radius", forward_hybrid_astar_config_.projection_search_radius);
  declare_parameter("hybrid_astar.goal_position_tolerance", rclcpp::ParameterValue(0.4));
  this->get_parameter("hybrid_astar.goal_position_tolerance", forward_hybrid_astar_config_.goal_position_tolerance);
  declare_parameter("hybrid_astar.goal_heading_tolerance", rclcpp::ParameterValue(0.35));
  this->get_parameter("hybrid_astar.goal_heading_tolerance", forward_hybrid_astar_config_.goal_heading_tolerance);
  declare_parameter("hybrid_astar.use_goal_heading", rclcpp::ParameterValue(false));
  this->get_parameter("hybrid_astar.use_goal_heading", forward_hybrid_astar_config_.use_goal_heading);
  declare_parameter("hybrid_astar.steering_penalty", rclcpp::ParameterValue(0.2));
  this->get_parameter("hybrid_astar.steering_penalty", forward_hybrid_astar_config_.steering_penalty);
  declare_parameter("hybrid_astar.steering_change_penalty", rclcpp::ParameterValue(0.3));
  this->get_parameter("hybrid_astar.steering_change_penalty", forward_hybrid_astar_config_.steering_change_penalty);
  declare_parameter("hybrid_astar.heading_change_penalty", rclcpp::ParameterValue(0.4));
  this->get_parameter("hybrid_astar.heading_change_penalty", forward_hybrid_astar_config_.heading_change_penalty);
  declare_parameter("hybrid_astar.obstacle_penalty_weight", rclcpp::ParameterValue(1.0));
  this->get_parameter("hybrid_astar.obstacle_penalty_weight", forward_hybrid_astar_config_.obstacle_penalty_weight);
  declare_parameter("hybrid_astar.heuristic_heading_weight", rclcpp::ParameterValue(0.2));
  this->get_parameter("hybrid_astar.heuristic_heading_weight", forward_hybrid_astar_config_.heuristic_heading_weight);
  declare_parameter("hybrid_astar.turn_side_hysteresis_penalty", rclcpp::ParameterValue(0.8));
  this->get_parameter("hybrid_astar.turn_side_hysteresis_penalty", forward_hybrid_astar_config_.turn_side_hysteresis_penalty);
  declare_parameter("hybrid_astar.rearward_check_distance", rclcpp::ParameterValue(1.5));
  this->get_parameter("hybrid_astar.rearward_check_distance", forward_hybrid_astar_config_.rearward_check_distance);
  declare_parameter("hybrid_astar.rearward_allowance", rclcpp::ParameterValue(0.05));
  this->get_parameter("hybrid_astar.rearward_allowance", forward_hybrid_astar_config_.rearward_allowance);
  declare_parameter("hybrid_astar.rearward_excursion_penalty", rclcpp::ParameterValue(4.0));
  this->get_parameter("hybrid_astar.rearward_excursion_penalty", forward_hybrid_astar_config_.rearward_excursion_penalty);
  declare_parameter("hybrid_astar.rearward_hard_reject_distance", rclcpp::ParameterValue(0.20));
  this->get_parameter(
    "hybrid_astar.rearward_hard_reject_distance",
    forward_hybrid_astar_config_.rearward_hard_reject_distance);

  RCLCPP_INFO(
    this->get_logger(),
    "hybrid_astar: wheelbase=%.6f max_steer=%.3f heading_bins=%d primitive_len=%.2f primitive_step=%.2f",
    forward_hybrid_astar_config_.wheelbase,
    forward_hybrid_astar_config_.max_steer,
    forward_hybrid_astar_config_.heading_bin_count,
    forward_hybrid_astar_config_.primitive_length,
    forward_hybrid_astar_config_.primitive_step);
  RCLCPP_INFO(
    this->get_logger(),
    "hybrid_astar: goal_tol=(%.2f, %.2f) use_goal_heading=%d proj_radius=%.2f",
    forward_hybrid_astar_config_.goal_position_tolerance,
    forward_hybrid_astar_config_.goal_heading_tolerance,
    forward_hybrid_astar_config_.use_goal_heading,
    forward_hybrid_astar_config_.projection_search_radius);

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_INFO(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    


  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::checkPerception3DThread, this), action_server_group_);
  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&GlobalPlanner::cbClickedPoint, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_raw_route_path_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/raw_route_path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

GlobalPlanner::~GlobalPlanner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  a_star_planner_pre_graph_.reset();
  forward_hybrid_astar_planner_.reset();
  action_server_global_planner_.reset();
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  if(static_ground_size_!=perception_3d_ros_->getSharedDataPtr()->static_ground_size_){
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    *pcl_ground_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_ground_);
    global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    *kdtree_ground_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_ground_);
    *kdtree_map_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_map_);
    *pcl_map_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_map_);
    static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_; //@ node weight
    RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d.");
    getStaticGraphFromPerception3D();
    static_ground_size_ = perception_3d_ros_->getSharedDataPtr()->static_ground_size_;
  }

}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = global_frame_;
  start.header.stamp = clock_->now();
  goal.header.frame_id = global_frame_;
  goal.header.stamp = clock_->now();

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;
  goal.pose.orientation.w = 1.0;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
    start.pose.orientation = transformStamped.transform.rotation;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
    return;
  }
  
  auto ros_path = makeROSPlan(start, goal, true, false, 0);
  if(ros_path.poses.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found for clicked goal.");
    return;
  }

  pub_path_->publish(ros_path);
  RCLCPP_INFO(this->get_logger(), "Path found for clicked goal.");

}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(auto it=1;it<path_id.size()-1;it++){

    geometry_msgs::msg::PoseStamped next_pst;
    next_pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    next_pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    next_pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    double vx,vy,vz;
    vx = next_pst.pose.position.x - current_pst.pose.position.x;
    vy = next_pst.pose.position.y - current_pst.pose.position.y;
    vz = next_pst.pose.position.z - current_pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    current_pst.pose.orientation.x = q.getX();
    current_pst.pose.orientation.y = q.getY();
    current_pst.pose.orientation.z = q.getZ();
    current_pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    for(double step=0.05;step<0.99;step+=0.05){
      pcl::PointXYZI pst_inter_polate_pc;
      pst_inter_polate_pc.x = current_pst.pose.position.x + vx*step;
      pst_inter_polate_pc.y = current_pst.pose.position.y + vy*step;
      pst_inter_polate_pc.z = current_pst.pose.position.z + vz*step;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree_map_->radiusSearch(pst_inter_polate_pc, perception_3d_ros_->getGlobalUtils()->getInscribedRadius(), pointIdxRadiusSearch, pointRadiusSquaredDistance)>1){
        //@ no line of sight, keep this pose
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      if(kdtree_ground_->radiusSearch(pst_inter_polate_pc, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<2){
        //@ not on the ground
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      double dx = vx*step;
      double dy = vy*step;
      double dr = sqrt(dx*dx+dy*dy);
      double dz = fabs(vz*step);
      float vertical_angle = std::asin(dz / dr);
      if(dr>0.5 && vertical_angle>0.349){
        //@ z jump
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
      if(dr>20.0){
        //@ longer than 10 meter
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
    }
  }
  smoothed_path_id.push_back(path_id[path_id.size()-1]);
}

void GlobalPlanner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1){
      next_pst.pose.position.x = pcl_ground_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pcl_ground_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pcl_ground_->points[path_id[it+1]].z;
    }


    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;

    if(vz!=0){
      double unit = sqrt(vx*vx + vy*vy + vz*vz);
      
      tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      right_vector.normalized();
      tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
      q.normalize();
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }
    else{
      //@ handle with 2D
      double yaw = atan2(vy, vx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }

    //RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f,%.2f, %.2f, %.2f, %.2f", vx, vy, vz, q.getX(), q.getY(), q.getZ(), q.getW());
    //@Interpolation to make global plan smoother and better resolution for local planner
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    if(it<path_id.size()-1){
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      for(double step=0.05;step<0.99;step+=0.05){

        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      ros_path.poses.push_back(pst);
    }
    
  }
}

unsigned int GlobalPlanner::getClosestIndexFromRadiusSearch(
  const std::vector<int>& point_indices,
  const std::vector<float>& point_squared_distances) const
{
  if(point_indices.empty() || point_squared_distances.empty()){
    return 0;
  }

  std::size_t closest_idx = 0;
  for(std::size_t i = 1; i < point_indices.size() && i < point_squared_distances.size(); ++i){
    if(point_squared_distances[i] < point_squared_distances[closest_idx]){
      closest_idx = i;
    }
  }
  return static_cast<unsigned int>(point_indices[closest_idx]);
}

bool GlobalPlanner::buildStraightLinePlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  nav_msgs::msg::Path& ros_path)
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double dz = goal.pose.position.z - start.pose.position.z;
  const double distance = sqrt(dx * dx + dy * dy + dz * dz);

  if(distance > direct_path_distance_threshold_){
    return false;
  }

  if(distance < 1e-3){
    geometry_msgs::msg::PoseStamped pose = goal;
    pose.header = ros_path.header;
    ros_path.poses.push_back(pose);
    return true;
  }

  const double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
  const double sample_step = 0.05;
  const double ground_search_radius = 0.5;

  for(double travelled = 0.0; travelled <= distance + sample_step; travelled += sample_step){
    const double ratio = std::min(1.0, travelled / distance);
    pcl::PointXYZI sample_pt;
    sample_pt.x = start.pose.position.x + dx * ratio;
    sample_pt.y = start.pose.position.y + dy * ratio;
    sample_pt.z = start.pose.position.z + dz * ratio;

    std::vector<int> ground_indices;
    std::vector<float> ground_squared_distances;
    if(kdtree_ground_->radiusSearch(sample_pt, ground_search_radius, ground_indices, ground_squared_distances) < 1){
      return false;
    }

    const unsigned int nearest_ground_index =
      getClosestIndexFromRadiusSearch(ground_indices, ground_squared_distances);
    if(perception_3d_ros_->get_min_dGraphValue(nearest_ground_index) < inscribed_radius){
      return false;
    }

    std::vector<int> map_indices;
    std::vector<float> map_squared_distances;
    if(kdtree_map_->radiusSearch(sample_pt, inscribed_radius, map_indices, map_squared_distances) > 1){
      return false;
    }
  }

  ros_path.poses.clear();

  geometry_msgs::msg::PoseStamped pose_start = start;
  pose_start.header = ros_path.header;
  ros_path.poses.push_back(pose_start);

  geometry_msgs::msg::PoseStamped last_pose = pose_start;
  for(double travelled = sample_step; travelled < distance; travelled += sample_step){
    const double ratio = travelled / distance;
    geometry_msgs::msg::PoseStamped pose = goal;
    pose.header = ros_path.header;
    pose.pose.position.x = start.pose.position.x + dx * ratio;
    pose.pose.position.y = start.pose.position.y + dy * ratio;
    pose.pose.position.z = start.pose.position.z + dz * ratio;

    const double ddx = pose.pose.position.x - last_pose.pose.position.x;
    const double ddy = pose.pose.position.y - last_pose.pose.position.y;
    const double ddz = pose.pose.position.z - last_pose.pose.position.z;
    if(sqrt(ddx * ddx + ddy * ddy + ddz * ddz) > 0.1){
      ros_path.poses.push_back(pose);
      last_pose = pose;
    }
  }

  geometry_msgs::msg::PoseStamped pose_goal = goal;
  pose_goal.header = ros_path.header;
  ros_path.poses.push_back(pose_goal);

  for(std::size_t i = 0; i < ros_path.poses.size(); ++i){
    const std::size_t next_idx = (i + 1 < ros_path.poses.size()) ? i + 1 : i;
    const double vx = ros_path.poses[next_idx].pose.position.x - ros_path.poses[i].pose.position.x;
    const double vy = ros_path.poses[next_idx].pose.position.y - ros_path.poses[i].pose.position.y;
    const double vz = ros_path.poses[next_idx].pose.position.z - ros_path.poses[i].pose.position.z;

    if(fabs(vz) > 1e-6){
      const double unit = sqrt(vx * vx + vy * vy + vz * vz);
      if(unit > 1e-6){
        tf2::Vector3 axis_vector(vx / unit, vy / unit, vz / unit);
        tf2::Vector3 up_vector(1.0, 0.0, 0.0);
        tf2::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf2::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
        q.normalize();
        ros_path.poses[i].pose.orientation.x = q.getX();
        ros_path.poses[i].pose.orientation.y = q.getY();
        ros_path.poses[i].pose.orientation.z = q.getZ();
        ros_path.poses[i].pose.orientation.w = q.getW();
      }
    }
    else{
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, atan2(vy, vx));
      ros_path.poses[i].pose.orientation.x = q.getX();
      ros_path.poses[i].pose.orientation.y = q.getY();
      ros_path.poses[i].pose.orientation.z = q.getZ();
      ros_path.poses[i].pose.orientation.w = q.getW();
    }
  }

  if(enable_detail_log_){
    RCLCPP_INFO(this->get_logger(),
      "Use straight-line plan for nearby goal, distance: %.2f m, poses: %zu",
      distance, ros_path.poses.size());
  }

  return true;
}

bool GlobalPlanner::getStartGoalID(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                                    unsigned int& start_id, unsigned int& goal_id){

  const double search_radius = 1.0;
  const double fallback_vertical_radius = 0.5;
  const double fallback_max_nearest_distance = 3.0;

  //@Get goal ID
  std::vector<int> pointIdxRadiusSearch_goal;
  std::vector<float> pointRadiusSquaredDistance_goal;
  pcl::PointXYZI pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;

  //@Compute nearest pc as goal
  //@TODO: add an edge between goal and nearest pc
  
  if(kdtree_ground_->radiusSearch (pcl_goal, search_radius, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal)<1){
    RCLCPP_WARN(this->get_logger(), "Goal is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    for(double z=goal.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_goal.clear();
      pointRadiusSquaredDistance_goal.clear();
      pcl_goal.z = z;
      if(kdtree_ground_->radiusSearch(pcl_goal, fallback_vertical_radius, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal,0)>0)
      {
        fallback_search = true;
        break;
      }
    }

    if(!fallback_search){
      pointIdxRadiusSearch_goal.clear();
      pointRadiusSquaredDistance_goal.clear();
      if(kdtree_ground_->nearestKSearch(pcl_goal, 1, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal) > 0){
        double nearest_distance = sqrt(pointRadiusSquaredDistance_goal[0]);
        if(nearest_distance <= fallback_max_nearest_distance){
          fallback_search = true;
          RCLCPP_WARN(this->get_logger(), "Falling back to nearest goal point on the ground at %.2f m.", nearest_distance);
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Nearest goal point on the ground is too far away: %.2f m.", nearest_distance);
        }
      }
    }

    if(!fallback_search){
      return false;
    }
  }
  
  goal_id = getClosestIndexFromRadiusSearch(pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal);

  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id, 
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }

  //--------------------------------------------------------------------------------------
  //@Get start ID
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZI pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;

  if(kdtree_ground_->radiusSearch (pcl_start, search_radius, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start)<1){
    RCLCPP_WARN(this->get_logger(), "Start is not found within %.2f m.", search_radius);
    bool fallback_search = false;

    for(double z=start.pose.position.z; z>-10;z-=0.1){
      pointIdxRadiusSearch_start.clear();
      pointRadiusSquaredDistance_start.clear();
      pcl_start.z = z;
      if(kdtree_ground_->radiusSearch(pcl_start, fallback_vertical_radius, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start,0)>0)
      {
        fallback_search = true;
        break;
      }
    }

    if(!fallback_search){
      pointIdxRadiusSearch_start.clear();
      pointRadiusSquaredDistance_start.clear();
      if(kdtree_ground_->nearestKSearch(pcl_start, 1, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start) > 0){
        double nearest_distance = sqrt(pointRadiusSquaredDistance_start[0]);
        if(nearest_distance <= fallback_max_nearest_distance){
          fallback_search = true;
          RCLCPP_WARN(this->get_logger(), "Falling back to nearest start point on the ground at %.2f m.", nearest_distance);
        }
        else{
          RCLCPP_WARN(this->get_logger(), "Nearest start point on the ground is too far away: %.2f m.", nearest_distance);
        }
      }
    }

    if(!fallback_search){
      return false;
    }
  }
  
  start_id = getClosestIndexFromRadiusSearch(pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start);

  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id, 
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);

  }

  return true;

}

void GlobalPlanner::publishRawRouteDebugPath(
  const nav_msgs::msg::Path & path,
  std::size_t goal_seq,
  std::size_t route_version,
  const std::string & source_label)
{
  nav_msgs::msg::Path debug_path = path;
  debug_path.header.frame_id = global_frame_;
  debug_path.header.stamp = clock_->now();
  for(auto & pose : debug_path.poses){
    pose.header = debug_path.header;
  }
  pub_raw_route_path_->publish(debug_path);
  RCLCPP_INFO(
    this->get_logger(),
    "raw_route_path published, route_version=%zu, goal_seq=%zu, source=%s, poses=%zu",
    route_version,
    goal_seq,
    source_label.c_str(),
    debug_path.poses.size());
}

void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  //@get goal and start
  const auto goal = goal_handle->get_goal();

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received the request before static layer is ready");
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Deactivate thread");
    goal_handle->succeed(result);
    clearCurrentHandleIfMatches(goal_handle);
    return;
  }

  const std::size_t goal_seq = ++debug_goal_seq_;
  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);

  auto ros_path = makeROSPlan(start, goal->goal, false, false, 0);

  if(ros_path.poses.empty()){
    publishRawRouteDebugPath(ros_path, goal_seq, debug_route_version_, "get_plan_failed");
    global_plan_result_->path = ros_path;
    goal_handle->abort(global_plan_result_);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    const std::size_t route_version = ++debug_route_version_;
    publishRawRouteDebugPath(ros_path, goal_seq, route_version, "get_plan");
    pub_path_->publish(ros_path);
    global_plan_result_->path = ros_path;
    goal_handle->succeed(global_plan_result_);
  }
  clearCurrentHandleIfMatches(goal_handle);
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  bool force_position_only_goal,
  bool force_use_goal_heading,
  int preferred_initial_turn_sign){
  
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  unsigned int start_id = 0;
  unsigned int goal_id = 0;
  bool has_start_goal_id = false;
  std::vector<unsigned int> path;
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();

  const bool allow_direct_shortcut =
    enable_direct_path_shortcut_ && !use_forward_hybrid_astar_;
  if(allow_direct_shortcut && buildStraightLinePlan(start, goal, ros_path)){
    return ros_path;
  }

  if(use_forward_hybrid_astar_ && forward_hybrid_astar_planner_){
    nav_msgs::msg::Path hybrid_path;
    if(forward_hybrid_astar_planner_->MakePlan(
        start, goal, &hybrid_path,
        force_position_only_goal,
        force_use_goal_heading,
        preferred_initial_turn_sign))
    {
      hybrid_path.header.frame_id = global_frame_;
      hybrid_path.header.stamp = clock_->now();
      return hybrid_path;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Forward Hybrid A* failed, fallback to legacy A*.");
  }

  has_start_goal_id = getStartGoalID(start, goal, start_id, goal_id);
  if(has_start_goal_id){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);  
  }

  if(path.empty()){
    if(enable_detail_log_ && has_start_goal_id)
      RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
    else if(enable_detail_log_)
      RCLCPP_WARN(this->get_logger(), "No path found because start/goal projection failed.");
    else if(has_start_goal_id)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found because start/goal projection failed.");
    return ros_path;
  }
  else{
    if(enable_detail_log_)
      RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Path found from: %u to %u", start_id, goal_id);
    getROSPath(path, ros_path);

    // Legacy graph A* can return a first segment that starts from projected
    // ground nodes slightly behind the current robot pose. Trim this near-start
    // rearward prefix and anchor the path with the current start pose so the
    // local planner does not receive an immediate backward hook.
    double start_yaw = 0.0;
    bool has_start_yaw = false;
    {
      const auto & q_msg = start.pose.orientation;
      const double norm =
        q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
      if (std::isfinite(norm) && norm > 1e-9) {
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        q.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, start_yaw);
        has_start_yaw = std::isfinite(start_yaw);
      }
    }

    if (has_start_yaw && !ros_path.poses.empty()) {
      constexpr double kRearTrimDistance = 1.5;
      constexpr double kRearTrimAllowance = 0.05;
      std::size_t trim_count = 0;
      for (std::size_t i = 0; i < ros_path.poses.size(); ++i) {
        const double dx = ros_path.poses[i].pose.position.x - start.pose.position.x;
        const double dy = ros_path.poses[i].pose.position.y - start.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance) || distance > kRearTrimDistance) {
          break;
        }
        const double x_local = std::cos(start_yaw) * dx + std::sin(start_yaw) * dy;
        if (x_local < -kRearTrimAllowance) {
          trim_count = i + 1;
          continue;
        }
        break;
      }

      if (trim_count > 0 && trim_count < ros_path.poses.size()) {
        ros_path.poses.erase(ros_path.poses.begin(), ros_path.poses.begin() + trim_count);
      } else if (trim_count >= ros_path.poses.size()) {
        ros_path.poses.clear();
      }
    }

    if (ros_path.poses.empty()) {
      geometry_msgs::msg::PoseStamped start_pose = start;
      start_pose.header = ros_path.header;
      ros_path.poses.push_back(start_pose);
    } else {
      const double first_dx = ros_path.poses.front().pose.position.x - start.pose.position.x;
      const double first_dy = ros_path.poses.front().pose.position.y - start.pose.position.y;
      if (std::hypot(first_dx, first_dy) > 0.05) {
        geometry_msgs::msg::PoseStamped start_pose = start;
        start_pose.header = ros_path.header;
        ros_path.poses.insert(ros_path.poses.begin(), start_pose);
      }
    }

    ros_path.poses.push_back(goal);
    return ros_path;
  }
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight
  
  /*
  //static graph has been remove 9 Mar 2025
  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();
  pubStaticGraph();
  
  RCLCPP_INFO(this->get_logger(), "Static graph is generated with size: %lu", static_graph_.getSize());
  */

  if(!has_initialized_){
    has_initialized_ = true;
    if(a_star_expanding_radius_ >= perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2){
      RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
    }
    if(!use_pre_graph_){
      a_star_planner_ = std::make_shared<A_Star_on_Graph>(pcl_ground_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_planner_->setupTurningWeight(turning_weight_);
    }
    else{
      a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(pcl_ground_, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
    }

    forward_hybrid_astar_planner_ = std::make_shared<ForwardHybridAStar>(
      perception_3d_ros_, pcl_ground_, kdtree_ground_, this->get_logger());
    forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
    forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
  }
  else{
    if(!use_pre_graph_){
      a_star_planner_->updateGraph(pcl_ground_);
    }
    else{
      a_star_planner_pre_graph_->updateGraph(pcl_ground_, static_graph_);
    }
    if(forward_hybrid_astar_planner_){
      forward_hybrid_astar_planner_->SetConfig(forward_hybrid_astar_config_);
      forward_hybrid_astar_planner_->SetGlobalFrame(global_frame_);
    }
  }

  pubWeight();

  RCLCPP_INFO(this->get_logger(), "Publish weighted ground point cloud.");
  graph_ready_ = true;
}


void GlobalPlanner::pubWeight(){

  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);
  
  unsigned long node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeightSize();
  for(auto it=0; it<node_weight_size; it++){

    pcl::PointXYZI ipt;

    ipt.x = pcl_ground_->points[it].x;
    ipt.y = pcl_ground_->points[it].y;
    ipt.z = pcl_ground_->points[it].z;
    ipt.intensity = static_graph_.getNodeWeight(it);    
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void GlobalPlanner::pubStaticGraph(){
 
  //@edge visualization 
  //@This is just for visulization, therefore reduce edges to let rviz less lag
  
  std::set<std::pair<unsigned int, unsigned int>> duplicate_check;
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = global_frame_;
  markerEdge.header.stamp = clock_->now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "edges";
  markerEdge.id = 3;
  markerEdge.scale.x = 0.03;
  markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
  markerEdge.color.a = 0.2;

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();

  int cnt = 0;
  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){
    geometry_msgs::msg::Point p;
    p.x = pcl_ground_->points[(*it).first].x;
    p.y = pcl_ground_->points[(*it).first].y;
    p.z = pcl_ground_->points[(*it).first].z;
    for(auto it_set = (*it).second.begin();it_set != (*it).second.end();it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = (*it).first;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = (*it).first;
      if( !duplicate_check.insert(edge_marker).second )
      {   
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {   
        continue;
      }
      markerEdge.points.push_back(p);
      p.x = pcl_ground_->points[(*it_set).first].x;
      p.y = pcl_ground_->points[(*it_set).first].y;
      p.z = pcl_ground_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

}
