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
#include <perception_3d/multilayer_spinning_lidar.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

#include <pcl/filters/filter.h>
#include <perception_3d/cloud_preprocessor.h>
#include <perception_3d/parameter_utils.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::MultiLayerSpinningLidar, perception_3d::Sensor)

namespace perception_3d
{


template<typename T, typename T2>
double getDistanceBTWPoints(T pt, T2 pt2){

  double dx = pt.x-pt2.x;
  double dy = pt.y-pt2.y;
  double dz = pt.z-pt2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

MultiLayerSpinningLidar::MultiLayerSpinningLidar(){
  enable_detail_log_ = false;
  return;
}

MultiLayerSpinningLidar::~MultiLayerSpinningLidar(){
}

void MultiLayerSpinningLidar::ptrInitial(){
  pcl_msg_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pending_cleared_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  export_occupied_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  export_cleared_points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void MultiLayerSpinningLidar::onInitialize()
{ 

  ptrInitial();

  std::string node_name = std::string{node_->get_name()};
  node_->get_parameter("enable_detail_log", enable_detail_log_);

  input_preprocessed_ = node_->declare_parameter(
    name_ + ".input_preprocessed", false);
  topic_ = getStringParameter(
    *node_, name_ + ".topic",
    input_preprocessed_ ? "preprocessed_cloud_topic" : "input_cloud_topic", "");
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic: %s", topic_.c_str());

  node_->declare_parameter(name_ + ".vertical_FOV_top", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".vertical_FOV_top", vertical_FOV_top_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "vertical_FOV_top: %.1f", vertical_FOV_top_);

  node_->declare_parameter(name_ + ".vertical_FOV_bottom", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".vertical_FOV_bottom", vertical_FOV_bottom_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "vertical_FOV_bottom: %.1f", vertical_FOV_bottom_);

  node_->declare_parameter(name_ + ".scan_effective_positive_start", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_positive_start", scan_effective_positive_start_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_positive_start: %.1f", scan_effective_positive_start_);

  node_->declare_parameter(name_ + ".scan_effective_positive_end", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_positive_end", scan_effective_positive_end_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_positive_end: %.1f", scan_effective_positive_end_);

  node_->declare_parameter(name_ + ".scan_effective_negative_start", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_negative_start", scan_effective_negative_start_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_negative_start: %.1f", scan_effective_negative_start_);

  node_->declare_parameter(name_ + ".scan_effective_negative_end", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".scan_effective_negative_end", scan_effective_negative_end_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "scan_effective_negative_end: %.1f", scan_effective_negative_end_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_tolerance", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_tolerance", euclidean_cluster_extraction_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_tolerance: %.1f", euclidean_cluster_extraction_tolerance_);

  node_->declare_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".euclidean_cluster_extraction_min_cluster_size", euclidean_cluster_extraction_min_cluster_size_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "euclidean_cluster_extraction_min_cluster_size: %d", euclidean_cluster_extraction_min_cluster_size_);

  clock_ = node_->get_clock();
  last_observation_time_ = clock_->now();

  node_->declare_parameter(name_ + ".xy_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".xy_resolution", resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "xy_resolution: %.2f", resolution_);

  node_->declare_parameter(name_ + ".height_resolution", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".height_resolution", height_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "height_resolution: %.2f", height_resolution_);

  marking_height_ = getDoubleParameter(
    *node_, name_ + ".marking_height", "marking_height", 0.0);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "marking_height: %.2f", marking_height_);

  min_obstacle_height_ = getDoubleParameter(
    *node_, name_ + ".min_obstacle_height", "min_obstacle_height", 0.0);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_obstacle_height: %.2f", min_obstacle_height_);

  perception_window_size_ = getDoubleParameter(
    *node_, name_ + ".perception_window_size", "perception_window_size", 0.0);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "perception_window_size: %.2f", perception_window_size_);

  node_->declare_parameter(name_ + ".segmentation_ignore_ratio", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".segmentation_ignore_ratio", segmentation_ignore_ratio_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "segmentation_ignore_ratio: %.2f", segmentation_ignore_ratio_);

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);

  expected_sensor_time_ = getDoubleParameter(
    *node_, name_ + ".expected_sensor_time", "expected_sensor_time", 0.0);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "expected_sensor_time: %.2f", expected_sensor_time_);

  node_->declare_parameter(name_ + ".pub_gbl_marking_for_visualization", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".pub_gbl_marking_for_visualization", pub_gbl_marking_for_visualization_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "pub_gbl_marking_for_visualization: %d", pub_gbl_marking_for_visualization_);

  node_->declare_parameter(name_ + ".stitcher_num", rclcpp::ParameterValue(0));
  node_->get_parameter(name_ + ".stitcher_num", stitcher_num_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "stitcher_num: %d", stitcher_num_);  

  sensor_frame_ = getStringParameter(
    *node_, name_ + ".sensor_frame", "sensor_frame", "");
  if(input_preprocessed_ && ((!is_local_planner_ && sensor_frame_.empty()) || stitcher_num_ != 0)){
    throw std::invalid_argument("Preprocessed lidar requires a global sensor_frame and stitcher_num=0");
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_), "input_preprocessed: %d", input_preprocessed_);
  
  node_->declare_parameter(name_ + ".pub_gbl_marking_frequency", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".pub_gbl_marking_frequency", pub_gbl_marking_frequency_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "pub_gbl_marking_frequency: %.1f", pub_gbl_marking_frequency_);  
  
  sensor_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sensor_cb_group_;

  sensor_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, 2, 
    std::bind(&MultiLayerSpinningLidar::cbSensor, this, std::placeholders::_1), sub_options);
  
  std::string pre_topic_name = node_name + "/" + name_;
  pub_current_observation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/current_observation", 2);
  pub_current_window_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/current_window_marking", 2);
  pub_current_projected_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/current_projected", 2);
  pub_current_segmentation_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/current_segmentation", 2);
  pub_gbl_marking_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/global_marking", 2);
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/dGraph", 2);
  pub_lethal_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pre_topic_name + "/lethal", 2);

  pub_casting_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pre_topic_name + "/tracing_objects", 2);

  pct_marking_ = std::make_shared<Marking>(&dGraph_, 
        gbl_utils_->getInscribedRadius(), gbl_utils_->getInflationRadius(), shared_data_, resolution_, height_resolution_);
  get_first_tf_ = false;
  
  marking_pub_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto publish_time = std::chrono::milliseconds(int(1000/pub_gbl_marking_frequency_));
  marking_pub_timer_ = node_->create_wall_timer(publish_time, std::bind(&MultiLayerSpinningLidar::pubUpdateLoop, this), marking_pub_cb_group_);

}

void MultiLayerSpinningLidar::cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if(input_preprocessed_){
    cbPreprocessed(*msg);
    return;
  }

  //@if not stitch, save copy time
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg (new pcl::PointCloud<pcl::PointXYZ>);
  if(stitcher_num_<=0){
    pcl::fromROSMsg(*msg, *pcl_msg);
  }
  else{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *scan_msg);
    
    if(pcl_stitcher_.size()<stitcher_num_){
      pcl_stitcher_.push_back(*scan_msg);
    }
    else{
      pcl_stitcher_.pop_front();
      pcl_stitcher_.push_back(*scan_msg);
    }
    
    for(auto si=pcl_stitcher_.begin(); si!=pcl_stitcher_.end();si++){
      *pcl_msg += (*si);
    }
  }

  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*pcl_msg, *pcl_msg, finite_indices);
  pcl_msg->is_dense = true;

  //@Create two trans, baselink->sensor and map->baselink
  try
  {
    trans_b2s_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getRobotFrame(), msg->header.frame_id, tf2::TimePointZero);

    trans_gbl2b_ = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero);

  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Failed to get transforms: %s", e.what());
    return;
  }

  get_first_tf_ = true;
  
  if(enable_detail_log_){
    RCLCPP_INFO_THROTTLE(
      node_->get_logger().get_child(name_),
      *clock_,
      60000,
      "Receiving Lidar topic: %s",
      topic_.c_str());
  }

  //@Justify affine 3d
  //Eigen::Affine3d a = tf2::transformToEigen(trans_gbl2b_);
  //Eigen::Affine3d b = tf2::transformToEigen(trans_b2s_);
  //Eigen::Affine3d c = tf2::transformToEigen(trans_gbl2s_);
  //Eigen::Affine3d d = a*b;

  //RCLCPP_INFO(node_->get_logger().get_child(name_), "trans: %f,%f,%f ---> %f,%f,%f", c.translation().x(),c.translation().y(),c.translation().z(), d.translation().x(),d.translation().y(),d.translation().z());
  //RCLCPP_INFO_STREAM(node_->get_logger().get_child(name_), "Rotation c: " << c.rotation());
  //RCLCPP_INFO_STREAM(node_->get_logger().get_child(name_), "Rotation d: " << d.rotation());
  
  Eigen::Affine3d trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  pcl::transformPointCloud(*pcl_msg, *pcl_msg, trans_b2s_af3);

  //@Get affine tf from gbl to sensor
  Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
  trans_gbl2s_af3_ = trans_gbl2b_af3*trans_b2s_af3;
  trans_gbl2s_ = tf2::eigenToTransform (trans_gbl2s_af3_);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-perception_window_size_, perception_window_size_);
  pass.filter (*pcl_msg);
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("y");
  pass.filter (*pcl_msg);
  pass.setInputCloud (pcl_msg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_obstacle_height_, marking_height_);
  pass.filter (*pcl_msg);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (pcl_msg);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*pcl_msg);
  finite_indices.clear();
  pcl::removeNaNFromPointCloud(*pcl_msg, *pcl_msg, finite_indices);
  pcl_msg->is_dense = true;

  //@Protect Mark/Clear functions
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);

  if(pcl_msg->points.empty()){
    pcl_msg_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_current_observation_->clear();
    last_observation_time_ = clock_->now();

    if(pub_current_observation_->get_subscription_count()>0){
      sensor_msgs::msg::PointCloud2 ros_pc2_msg;
      pcl_msg_->header.frame_id = is_local_planner_ ? gbl_utils_->getGblFrame() : gbl_utils_->getRobotFrame();
      pcl::toROSMsg(*pcl_msg_, ros_pc2_msg);
      pub_current_observation_->publish(ros_pc2_msg);
    }
    return;
  }

  pcl_msg_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_msg_ = pcl_msg;

  if(is_local_planner_){
    //@ put to current observation, different for global/local
    Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
    pcl::transformPointCloud(*pcl_msg_, *pcl_msg_, trans_gbl2b_af3);
    pcl::copyPointCloud(*pcl_msg_, *sensor_current_observation_);
  }

  //@ update time
  last_observation_time_ = clock_->now();

  //@ Raw clouds still carry their own acquisition header; a missing/invalid
  //@ stamp starts no export cycle instead of substituting the receipt clock.
  std::int64_t raw_stamp_ns = 0;
  if(msg->header.stamp.nanosec < 1000000000u &&
    (msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0))
  {
    raw_stamp_ns =
      rclcpp::Time(msg->header.stamp, clock_->get_clock_type()).nanoseconds();
  }
  startSnapshotCycle(raw_stamp_ns);

  if(pub_current_observation_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pcl_msg_->header.frame_id = is_local_planner_ ? gbl_utils_->getGblFrame() : gbl_utils_->getRobotFrame();
    pcl::toROSMsg(*pcl_msg_, ros_pc2_msg);
    pub_current_observation_->publish(ros_pc2_msg);
  }

}

void MultiLayerSpinningLidar::cbPreprocessed(const sensor_msgs::msg::PointCloud2 & msg){
  const auto cb_started_at = std::chrono::steady_clock::now();
  const auto now = clock_->now();
  if(msg.header.frame_id != gbl_utils_->getGblFrame() ||
    !IsCloudStampFresh(msg.header.stamp, now, expected_sensor_time_))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
      "Rejecting shared cloud: expected global frame and fresh acquisition timestamp");
    return;
  }
  const rclcpp::Time stamp(msg.header.stamp, now.get_clock_type());
  // Keep TF, cloud and freshness metadata coherent for clear/mark readers.
  const auto cb_lock_before = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  const auto cb_lock_acquired = std::chrono::steady_clock::now();
  if(have_preprocessed_observation_ && last_observation_time_ <= now &&
    stamp <= last_observation_time_)
  {
    return;
  }
  geometry_msgs::msg::TransformStamped body_to_sensor, global_to_body;
  try{
    // Points have already been transformed. These poses are only for the global
    // marking window, FOV and ray clearing, at the SAME acquisition instant.
    if(!is_local_planner_){
      const auto timeout = rclcpp::Duration::from_seconds(0.05);
      body_to_sensor = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getRobotFrame(), sensor_frame_, stamp, timeout);
      global_to_body = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), stamp, timeout);
    }
  }
  catch(const tf2::TransformException & error){
    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
      "Shared cloud waiting for acquisition-time TF: %s", error.what());
    return;
  }
  if(!IsCloudStampFresh(msg.header.stamp, clock_->now(), expected_sensor_time_)){
    return;
  }
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(msg, *cloud);
  // No passthrough, voxel filter or point-cloud transform in either consumer.
  if(!is_local_planner_){
    trans_b2s_ = body_to_sensor;
    trans_gbl2b_ = global_to_body;
    trans_gbl2s_af3_ = tf2::transformToEigen(global_to_body) * tf2::transformToEigen(body_to_sensor);
    trans_gbl2s_ = tf2::eigenToTransform(trans_gbl2s_af3_);
  }
  pcl_msg_ = cloud;
  // selfClear needs the new observation before selfMark appends centroids.
  pcl_msg_gbl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
  if(is_local_planner_){
    pcl::copyPointCloud(*cloud, *sensor_current_observation_);
  }
  last_observation_time_ = stamp;
  have_preprocessed_observation_ = true;
  startSnapshotCycle(stamp.nanoseconds());
  pending_mark_ = pending_clear_ = true;
  get_first_tf_ = true;
  if(pub_current_observation_->get_subscription_count() > 0){
    pub_current_observation_->publish(msg);
  }
  {
    const auto finished_at = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(finished_at - cb_started_at).count();
    if(total_ms > 5.0){
      RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
        "writer-timing cbPreprocessed total=%.3fms lock_wait=%.3fms held=%.3fms points=%zu",
        total_ms,
        std::chrono::duration<double, std::milli>(cb_lock_acquired - cb_lock_before).count(),
        std::chrono::duration<double, std::milli>(finished_at - cb_lock_acquired).count(),
        cloud->points.size());
    }
  }
}

void MultiLayerSpinningLidar::updateLethalPointCloud(){

  const auto lethal_started_at = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  if(!shared_data_->pcl_ground_){
    return;
  }
  for(auto it=pct_marking_->lethal_map_.begin(); it!=pct_marking_->lethal_map_.end(); it++){
    if((*it).first < 0 ||
      static_cast<size_t>((*it).first) >= shared_data_->pcl_ground_->points.size())
    {
      continue;
    }
    pcl::PointXYZI ipt;
    ipt.x = shared_data_->pcl_ground_->points[(*it).first].x;
    ipt.y = shared_data_->pcl_ground_->points[(*it).first].y;
    ipt.z = shared_data_->pcl_ground_->points[(*it).first].z;
    current_lethal_->push_back(ipt);
    //RCLCPP_INFO(node_->get_logger(), "Lethal: %.2f, %.2f, %2.f", ipt.x, ipt.y, ipt.z);
  }

  if(pub_lethal_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    current_lethal_->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*current_lethal_, ros_pc2_msg);
    pub_lethal_->publish(ros_pc2_msg);
  }
  {
    const double total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - lethal_started_at).count();
    if(total_ms > 5.0){
      RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
        "writer-timing updateLethal total=%.3fms lethal=%zu", total_ms, current_lethal_->points.size());
    }
  }
  
}

void MultiLayerSpinningLidar::selfMark(){
  
  if(!shared_data_){
    return;
  }
  const auto done_mark_started_at = std::chrono::steady_clock::now();
  const auto done_mark_lock_before = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  const auto done_mark_lock_acquired = std::chrono::steady_clock::now();

  if(is_local_planner_){
    return;
  }

  if(!get_first_tf_ || !shared_data_->is_static_layer_ready_){
    return;
  }

  if(!shared_data_->isAllLayersBeenReset()){
    return;
  }

  if(input_preprocessed_ && (!pending_mark_ || !isCurrent())){
    return;
  }

  if(!pct_marking_ || !pcl_msg_ || !shared_data_->pcl_ground_ ||
    !shared_data_->kdtree_ground_ || !shared_data_->pcl_map_ ||
    !shared_data_->kdtree_map_ || shared_data_->pcl_ground_->empty() ||
    shared_data_->pcl_map_->empty())
  {
    return;
  }

  if(pcl_msg_->points.size()<=5){
    pending_mark_ = false;
    //@ A fresh empty acquisition still completes the clear half of the cycle;
    //@ explicit removals captured by selfClear() remain the only evidence.
    finalizeSnapshotObservation();
    return;
  }
  //@ Transform into global frame
  pcl_msg_gbl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if(input_preprocessed_){
    *pcl_msg_gbl_ = *pcl_msg_;
    pending_mark_ = false;
  }
  else{
    Eigen::Affine3d trans_gbl2b_af3 = tf2::transformToEigen(trans_gbl2b_);
    pcl::transformPointCloud(*pcl_msg_, *pcl_msg_gbl_, trans_gbl2b_af3);
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*pcl_msg_gbl_, *pcl_msg_gbl_, finite_indices);
    pcl_msg_gbl_->is_dense = true;
  }
  if(pcl_msg_gbl_->points.size()<=5){
    finalizeSnapshotObservation();
    return;
  }
  const auto done_mark_transformed = std::chrono::steady_clock::now();

  pcl::search::KdTree<pcl::PointXYZ>::Ptr pc_kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  pc_kdtree->setInputCloud (pcl_msg_gbl_);

  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_segmentation;
  ec_segmentation.setClusterTolerance (euclidean_cluster_extraction_tolerance_);
  ec_segmentation.setMinClusterSize (euclidean_cluster_extraction_min_cluster_size_);
  ec_segmentation.setMaxClusterSize (pcl_msg_gbl_->points.size());
  ec_segmentation.setSearchMethod (pc_kdtree);
  ec_segmentation.setInputCloud (pcl_msg_gbl_);
  ec_segmentation.extract (cluster_indices_segmentation);


  float intensity_cnt = 100;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  const auto done_mark_clustered = std::chrono::steady_clock::now();
  double add_ms = 0.0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
  {
    if(it->indices.empty()){
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;
    centroid.intensity = 0.0;
    std::size_t valid_points = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      if(*pit < 0 || static_cast<std::size_t>(*pit) >= pcl_msg_gbl_->points.size()){
        continue;
      }
      const auto & src_pt = pcl_msg_gbl_->points[*pit];
      if(!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) || !std::isfinite(src_pt.z)){
        continue;
      }
      
      //@For visualization purpose
      pcl::PointXYZI i_pt;
      i_pt.x = src_pt.x;
      i_pt.y = src_pt.y;
      i_pt.z = src_pt.z;
      i_pt.intensity = intensity_cnt;
      centroid.x += i_pt.x;
      centroid.y += i_pt.y;
      centroid.z += i_pt.z;

      cloud_cluster->points.push_back(i_pt); 
      cloud_clusters->points.push_back(i_pt);
      valid_points++;

    }
    if(valid_points == 0 || cloud_cluster->points.empty()){
      continue;
    }
    centroid.x/=valid_points;
    centroid.y/=valid_points;
    centroid.z/=valid_points;
    centroid.intensity = intensity_cnt;
    cloud_clusters->points.push_back(centroid);   
    intensity_cnt += 100;
 

    //@ Sometimes the lidar accidently add ground scan (due to lego loam did not segment them correctly)
    //@ Therefore we implement following temporal solution -> when cluster center attaches ground, ignore it!
    std::vector<int> id(1);
    std::vector<float> sqdist(1);
    if(shared_data_->kdtree_ground_->radiusSearch(centroid, 0.05, id, sqdist, 1)){
      continue;
    }


    //@ Test a cluster is in static. If the cluster is in static, we dont need to add it because we can save memory.
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_cluster);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*cloud_cluster);
    if(cloud_cluster->points.empty()){
      continue;
    }
    size_t hit=0;
    id.clear();
    sqdist.clear();
    if(segmentation_ignore_ratio_<=0.999){
      for(auto a_pt=cloud_cluster->points.begin();a_pt!=cloud_cluster->points.end();a_pt++){
        if(shared_data_->kdtree_map_->radiusSearch(centroid, 0.1, id, sqdist, 1)){
          hit++;
          if(hit>cloud_cluster->points.size()*segmentation_ignore_ratio_)
            break;
        }
      }      
    }

    if(hit<=cloud_cluster->points.size()*segmentation_ignore_ratio_){

      //@ Project pc base on robot RPY:
      // This is not the perfect solution, because the robot may stand on the ground but the obstalce in on slope
      // Maybe the best approach is to project base on the ground normal
      
      tf2::Quaternion rotation(trans_gbl2b_.transform.rotation.x, trans_gbl2b_.transform.rotation.y, trans_gbl2b_.transform.rotation.z, trans_gbl2b_.transform.rotation.w);
      tf2::Vector3 vector(0, 0, 1);
      tf2::Vector3 base_link_normal = tf2::quatRotate(rotation, vector);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      coefficients->values.resize (4);
      coefficients->values[0] = base_link_normal[0];
      coefficients->values[1] = base_link_normal[1];
      coefficients->values[2] = base_link_normal[2];
      double d = -trans_gbl2b_.transform.translation.x*base_link_normal[0]-trans_gbl2b_.transform.translation.y*base_link_normal[1]-trans_gbl2b_.transform.translation.z*base_link_normal[2];
      coefficients->values[3] = d;
      // Create the filtering object
      
      //pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      //pcl::ProjectInliers<pcl::PointXYZI> proj;
      //proj.setModelType (pcl::SACMODEL_PLANE);
      //proj.setInputCloud (cloud_cluster);
      //proj.setModelCoefficients (coefficients);
      //proj.filter (*projected_cloud_cluster);
      //*projected_cloud_clusters += (*projected_cloud_cluster);
      

      //@ store the cluster in marking
      //@ consider rounding problem, we have to convert the centroid coordinate back by resolution
      pcl::PointXYZ voxelized_centroid;
      voxelized_centroid.x = int(centroid.x/resolution_) * resolution_;
      voxelized_centroid.y = int(centroid.y/resolution_) * resolution_;
      voxelized_centroid.z = int(centroid.z/height_resolution_) * height_resolution_;

      //@ push_back centroid since it is used to represent the cluster
      pcl::PointXYZ pt_centroid;
      pt_centroid.x = centroid.x; pt_centroid.y = centroid.y; pt_centroid.z = centroid.z;
      pcl_msg_gbl_->push_back(pt_centroid);
      cloud_cluster->push_back(centroid);

      if(isinLidarObservation(voxelized_centroid)){
          const auto add_started_at = std::chrono::steady_clock::now();
          pct_marking_->addPCPtr(centroid.x, centroid.y, centroid.z, cloud_cluster, coefficients);
          add_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - add_started_at).count();
      }

    }
    else{
      RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Reject cluster with size: %lu at %f,%f,%f, because it is located in the static layer", cloud_cluster->points.size(), centroid.x, centroid.y, centroid.z);
    }
    
  }

  //@ Only now is the per-scan clear+mark cycle complete. Freeze the persistent
  //@ active markings plus the explicit removals from this exact acquisition.
  finalizeSnapshotObservation();

  {
    const auto finished_at = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(finished_at - done_mark_started_at).count();
    if(total_ms > 5.0){
      RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
        "writer-timing selfMark total=%.3fms lock_wait=%.3fms xform=%.3fms cluster=%.3fms loop=%.3fms add=%.3fms clusters=%zu points=%zu",
        total_ms,
        std::chrono::duration<double, std::milli>(done_mark_lock_acquired - done_mark_lock_before).count(),
        std::chrono::duration<double, std::milli>(done_mark_transformed - done_mark_lock_acquired).count(),
        std::chrono::duration<double, std::milli>(done_mark_clustered - done_mark_transformed).count(),
        std::chrono::duration<double, std::milli>(finished_at - done_mark_clustered).count(),
        add_ms, cluster_indices_segmentation.size(), pcl_msg_gbl_->points.size());
    }
  }

  if(pub_current_projected_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    projected_cloud_clusters->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*projected_cloud_clusters, ros_pc2_msg);
    pub_current_projected_->publish(ros_pc2_msg);
  }

  if(pub_current_segmentation_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    cloud_clusters->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*cloud_clusters, ros_pc2_msg);
    pub_current_segmentation_->publish(ros_pc2_msg);
  }
  
}

void MultiLayerSpinningLidar::selfClear(){

  if(!shared_data_){
    return;
  }
  const auto clear_started_at = std::chrono::steady_clock::now();
  const auto clear_lock_before = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  const auto clear_lock_acquired = std::chrono::steady_clock::now();

  if(is_local_planner_){
    return;
  }

  if(! get_first_tf_ || ! shared_data_->is_static_layer_ready_){
    return;
  }

  if(!pct_marking_){
    return;
  }

  if(shared_data_->dgraph_update_request_[name_]){
    //@ need to regenerate dynamic graph
    resetdGraph();
    shared_data_->dgraph_update_request_[name_] = false;
  }

  if(input_preprocessed_){
    if(!pending_clear_ || !isCurrent()){
      return;
    }
    pending_clear_ = false;
  }

  //@ This exact clear pass owns the explicit removal evidence. It stays frozen
  //@ in the completed cycle so repeated exporter polls reuse it rather than
  //@ turning a later empty cloud into a fresh clear. With the unified producer
  //@ OFF this record capture is skipped entirely, preserving old behavior. The
  //@ completion marker itself is needed even in legacy-cost-only mode so the
  //@ cost cache advances only after a full clear/mark.
  const bool producer_enabled = shared_data_ &&
    shared_data_->scene_producer_enabled_.load(std::memory_order_acquire);
  const bool snapshot_evidence_enabled = producer_enabled &&
    shared_data_->scene_evidence_enabled_.load(std::memory_order_acquire);
  if(producer_enabled){
    if(snapshot_evidence_enabled){
      pending_cleared_points_->clear();
    }
    export_clear_completed_ = true;
  }else{
    export_clear_completed_ = false;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_last_observation(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_observation(new pcl::PointCloud<pcl::PointXYZ>());
  bool observation_clear = true;

  if(pcl_msg_gbl_ && pcl_msg_gbl_->points.size()>5){
    *last_observation = *pcl_msg_gbl_;
    if(!input_preprocessed_){
      std::vector<int> finite_indices;
      pcl::removeNaNFromPointCloud(*last_observation, *last_observation, finite_indices);
      last_observation->is_dense = true;
    }
    if(last_observation->points.size()>5){
      kdtree_last_observation->setInputCloud(last_observation);
      observation_clear = false;
    }
  }
  
  visualization_msgs::msg::MarkerArray markerArray;
  pc_current_window_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  //@ We queue all observation here for later clearing and remarking value
  //@ This is very important!!!!!!!!
  std::vector<perception_3d::marking_voxel> current_observation_ptr;
  std::vector<perception_3d::per_marking*> markings_to_clear;
  
  //@ find robot location and base on perception window, we extract all nearby marked clusters
  int round_robot_base_x_min = ((trans_gbl2b_.transform.translation.x-perception_window_size_)/resolution_);
  int round_robot_base_x_max = ((trans_gbl2b_.transform.translation.x+perception_window_size_)/resolution_);
  int round_robot_base_y_min = ((trans_gbl2b_.transform.translation.y-perception_window_size_)/resolution_);
  int round_robot_base_y_max = ((trans_gbl2b_.transform.translation.y+perception_window_size_)/resolution_);

  //@ TODO: make this threshold more adaptive and robust
  int round_robot_base_z_min = ((trans_gbl2b_.transform.translation.z-marking_height_)/height_resolution_);
  int round_robot_base_z_max = ((trans_gbl2b_.transform.translation.z+marking_height_)/height_resolution_);


  //@Find min/max iterator
  //auto it_x_min = marking_.lower_bound(round_robot_base_x_min);
  //auto it_x_max = marking_.lower_bound(round_robot_base_x_max);
  auto it_x_min = pct_marking_->getXIter(round_robot_base_x_min);
  auto it_x_max = pct_marking_->getXIter(round_robot_base_x_max);
  if(it_x_min==pct_marking_->getEnd() && it_x_min==it_x_max)
    return;

  size_t cleared_cnt = 0;
  for(auto it_x = it_x_min; it_x!=it_x_max; it_x++){

    auto& y_cells = (*it_x).second;
    auto it_y_min = y_cells.lower_bound(round_robot_base_y_min);
    auto it_y_max = y_cells.lower_bound(round_robot_base_y_max);
    if(it_y_min==y_cells.end() && it_y_min==it_y_max)
      continue;

    for(auto it_y = it_y_min; it_y!=it_y_max; it_y++){
      auto& z_cells = (*it_y).second;
      if(z_cells.empty())
        continue;
      //@ A marked point exists, loop z for sight check
      auto it_z_min = z_cells.lower_bound(round_robot_base_z_min);
      auto it_z_max = z_cells.lower_bound(round_robot_base_z_max);
      if(it_z_min==z_cells.end() && it_z_min==it_z_max)
        continue;

      //@ fast segmentation of z axis
      for(auto it_z = it_z_min; it_z!=it_z_max;it_z++){
        auto& marking = (*it_z).second;

        if(marking.pc_== nullptr || marking.pc_->empty()){
          continue;
        }
        
        pcl::PointXYZ pt;
        /* Restoring the pt is not accurate due to the rounding, we get it from last point in pt, which is pushed when added
        pt.x = (*it_x).first*resolution_;
        pt.y = (*it_y).first*resolution_;
        pt.z = (*it_z).first*height_resolution_;
        */
        pt.x = marking.pc_->back().x;
        pt.y = marking.pc_->back().y;
        pt.z = marking.pc_->back().z;
        pcl::PointCloud<pcl::PointXYZI> casting_check;

        if(!isinLidarObservation(pt)){
  
          perception_3d::marking_voxel a_voxel;
          a_voxel.x = (*it_x).first;
          a_voxel.y = (*it_y).first;
          a_voxel.z = (*it_z).first;
          current_observation_ptr.push_back(a_voxel);
          *pc_current_window_ += (*marking.pc_);
          continue;
        }
        else{
          //@ get point cloud along the ray for casting
          bool skip_clear_this_segmentation = false;
          if(!observation_clear){
            //@ create a pointcloud that actually is a line composed of many descrete points, and then we can check radius along this line
            getCastingPointCloud(pt, casting_check);
            //@ we loop this "line" and do radius search to see if there is any obstacle, if there is an obstacle, it means this line is blocked, so ray trace fail
            for(auto a_pt=casting_check.points.begin(); a_pt!=casting_check.points.end(); a_pt++){
              /*
              //@ index 0 is cluster center
              //@ Check center around to rule out one stand for all (huge segmentation)
              if(a_pt==casting_check.points.begin()){
                //@ find nearest ostacle and then check whether this obstacle belongs to itself
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                pcl::PointXYZ pt_i;
                pt_i.x = (*a_pt).x;
                pt_i.y = (*a_pt).y;
                pt_i.z = (*a_pt).z;
                if(kdtree_last_observation->nearestKSearch(pt_i, 1, pointIdxNKNSearch, pointNKNSquaredDistance)>0){
                  pcl::PointXYZI candidate_pt;
                  candidate_pt.x = pcl_msg_gbl_->points[pointIdxNKNSearch.front()].x;
                  candidate_pt.y = pcl_msg_gbl_->points[pointIdxNKNSearch.front()].y;
                  candidate_pt.z = pcl_msg_gbl_->points[pointIdxNKNSearch.front()].z;
                  if((*it_z).second.pc_->points.size()>4){
                    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_cluster(new pcl::KdTreeFLANN<pcl::PointXYZI>());
                    kdtree_cluster->setInputCloud((*it_z).second.pc_);
                    std::vector<int> id;
                    std::vector<float> sqdist;
                    if(kdtree_cluster->radiusSearch(candidate_pt, 0.5, id, sqdist)>0){
                      //@ ray hits obstacle, we skip clearing this segmentation
                      skip_clear_this_segmentation = true;
                      break;
                    }
                  }
                }
              }
              */
              //@ when casting back for last 5 cm, ignore it, because dirty lidar may cause casting fail
              if((*a_pt).intensity<0.05)
                break;
              if((*a_pt).intensity>perception_window_size_){
                skip_clear_this_segmentation = true;
                break;
              }
              //@ Create a point for kd-tree
              pcl::PointXYZ pt_i;
              pt_i.x = (*a_pt).x;
              pt_i.y = (*a_pt).y;
              pt_i.z = (*a_pt).z;
              //double search_distance =  (*a_pt).intensity/20. + 0.01; //@ decrease spot size, ex: at 1.0 meter look for 5 cm;
              //search_distance = std::min(search_distance, 0.1);
              double search_distance = std::max(resolution_, height_resolution_);
              std::vector<int> id;
              std::vector<float> sqdist;
              if(kdtree_last_observation->radiusSearch(pt_i, search_distance, id, sqdist)>0){
                //@ ray hits obstacle, we skip clearing this segmentation
                skip_clear_this_segmentation = true;
                break;
              }
            }            
          }

          //Hit obstacle when ray tracing, so we skip clearing->meaning that we add this segmentation to the observation
          if(skip_clear_this_segmentation){
            perception_3d::marking_voxel a_voxel;
            a_voxel.x = (*it_x).first;
            a_voxel.y = (*it_y).first;
            a_voxel.z = (*it_z).first;
            current_observation_ptr.push_back(a_voxel);
            *pc_current_window_ += (*marking.pc_);
            //addCastingMarker(pt, current_observation_ptr.size(), markerArray);
            continue;
          }
          
          
          std::vector<int> id;
          std::vector<float> sqdist;
          //@ I am not sure what happen below, looks like I redo check again but the threshold (1) is different
          if(!observation_clear && kdtree_last_observation->radiusSearch(pt, resolution_, id, sqdist)>1){
            *pc_current_window_ += (*marking.pc_);

            perception_3d::marking_voxel a_voxel;
            a_voxel.x = (*it_x).first;
            a_voxel.y = (*it_y).first;
            a_voxel.z = (*it_z).first;
            current_observation_ptr.push_back(a_voxel);
            //addCastingMarker(pt, current_observation_ptr.size(), markerArray);
          }       
          else{
            addCastingMarker(pt, cleared_cnt, markerArray);
            markings_to_clear.push_back(&marking);
            cleared_cnt++;
          } 
                 
        }
      }

    }
  }

  for(auto* marking : markings_to_clear){
    if(!marking){
      continue;
    }
    //@ Capture the points that really existed before removePCPtr() resets the
    //@ pointer. These are producer-owned explicit clear records, not free volume
    //@ inferred from an empty cloud. Off means no snapshot preparation at all.
    if(snapshot_evidence_enabled && marking->pc_){
      for(const auto & point : marking->pc_->points){
        if(std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z))
        {
          pending_cleared_points_->push_back(point);
        }
      }
    }
    pct_marking_->removePCPtr(*marking);
  }

  //pct_marking_->updateCleared(current_observation_ptr);
  //@ put to current observation, different for global/local
  
  if(pub_casting_->get_subscription_count()>0){
    pub_casting_->publish(markerArray);
  }

  if(pub_current_window_marking_->get_subscription_count()>0){
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pc_current_window_->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pc_current_window_, ros_pc2_msg);
    pub_current_window_marking_->publish(ros_pc2_msg);     
  }
  {
    const auto finished_at = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(finished_at - clear_started_at).count();
    if(total_ms > 5.0){
      RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
        "writer-timing selfClear total=%.3fms lock_wait=%.3fms work=%.3fms cleared=%zu window_pts=%zu",
        total_ms,
        std::chrono::duration<double, std::milli>(clear_lock_acquired - clear_lock_before).count(),
        std::chrono::duration<double, std::milli>(finished_at - clear_lock_acquired).count(),
        cleared_cnt, pc_current_window_->points.size());
    }
  }
}

void MultiLayerSpinningLidar::getCastingPointCloud(pcl::PointXYZ& cluster_center, pcl::PointCloud<pcl::PointXYZI>& pc_for_check){

  //@We leverage intensity as distance from cluster_center to check point
  pcl::PointXYZI a_pt_s;
  a_pt_s.x = trans_gbl2s_af3_.translation().x();
  a_pt_s.y = trans_gbl2s_af3_.translation().y();
  a_pt_s.z = trans_gbl2s_af3_.translation().z();

  float dX =
      trans_gbl2s_af3_.translation().x() - cluster_center.x;
  float dY =
      trans_gbl2s_af3_.translation().y() - cluster_center.y;
  float dZ =
      trans_gbl2s_af3_.translation().z() - cluster_center.z;
  
  float distance = sqrt(dX*dX + dY*dY + dZ*dZ);
  //@ Distance is the distance from point sample to sensor
  if(distance <= 1e-6 || !std::isfinite(distance)){
    return;
  }
  distance = distance/0.05; //sample by every 5 cm
  float dt = 1/distance;
  for(float t=0; t<=1.0; t+=dt){
    pcl::PointXYZI a_pt;
    a_pt.intensity = 0.0;
    a_pt.x = cluster_center.x + dX*t;
    a_pt.y = cluster_center.y + dY*t;
    a_pt.z = cluster_center.z + dZ*t;
    a_pt.intensity = getDistanceBTWPoints(a_pt_s, a_pt);  
    pc_for_check.push_back(a_pt); 
  }
  
  //@ Generate t=1.0
  pcl::PointXYZI a_pt;
  a_pt.intensity = 0.0;
  a_pt.x = cluster_center.x + dX;
  a_pt.y = cluster_center.y + dY;
  a_pt.z = cluster_center.z + dZ;
  a_pt.intensity = getDistanceBTWPoints(a_pt_s, a_pt);  
  pc_for_check.push_back(a_pt); 
  /*
  double l = cluster_center.x - trans_gbl2s_af3_.translation().x();
  double m = cluster_center.y - trans_gbl2s_af3_.translation().y();
  double n = cluster_center.z - trans_gbl2s_af3_.translation().z();


  if(n>=0){
    for(auto step_z=cluster_center.z;step_z>=trans_gbl2s_af3_.translation().z();step_z=step_z-0.05){
      pcl::PointXYZI pt;
      pt.z = step_z;
      pt.y = (pt.z- cluster_center.z)/n * m + cluster_center.y;
      pt.x = (pt.z- cluster_center.z)/n * l + cluster_center.x; 
      pt.intensity = getDistanceBTWPoints(cluster_center, pt);  
      pc_for_check.push_back(pt); 
    }
  }
  else{
    for(auto step_z=cluster_center.z;step_z<=trans_gbl2s_af3_.translation().z();step_z=step_z+0.05){
      pcl::PointXYZI pt;
      pt.z = step_z;
      pt.y = (pt.z- cluster_center.z)/n * m + cluster_center.y;
      pt.x = (pt.z- cluster_center.z)/n * l + cluster_center.x;   
      pt.intensity = getDistanceBTWPoints(cluster_center, pt);  
      pc_for_check.push_back(pt); 
    }    
  }
  */
}

bool MultiLayerSpinningLidar::isinLidarObservation(pcl::PointXYZ& pc){
  
  
  //@ Solving vertical distance to sensor plan, so we can compute sin for vertical FOV check
  
  tf2::Quaternion rotation(trans_gbl2s_.transform.rotation.x, trans_gbl2s_.transform.rotation.y, trans_gbl2s_.transform.rotation.z, trans_gbl2s_.transform.rotation.w);
  tf2::Vector3 vector(0, 0, 1);
  tf2::Vector3 sensor_normal = tf2::quatRotate(rotation, vector);

  double d = -trans_gbl2s_.transform.translation.x*sensor_normal[0]-trans_gbl2s_.transform.translation.y*sensor_normal[1]-trans_gbl2s_.transform.translation.z*sensor_normal[2];
  double p2plane = pc.x*sensor_normal[0]+pc.y*sensor_normal[1]+pc.z*sensor_normal[2]+d;
  double dx = pc.x-trans_gbl2s_.transform.translation.x;
  double dy = pc.y-trans_gbl2s_.transform.translation.y; 
  double dz = pc.z-trans_gbl2s_.transform.translation.z;
  double p2s = sqrt(dx*dx+dy*dy+dz*dz);
  if(p2s <= 1e-6 || !std::isfinite(p2s)){
    return false;
  }
  double fov_ratio = p2plane/p2s;
  if(!std::isfinite(fov_ratio)){
    return false;
  }
  fov_ratio = std::max(-1.0, std::min(1.0, fov_ratio));
  double result = asin (fov_ratio) * 180.0 / 3.1415926535;
  if(result<vertical_FOV_bottom_ || result>vertical_FOV_top_)
    return false;
  
  //@ Leverage shortest angle to rule out yaw angle
  
  // Generate a pose pointing from sensor to cloud centroid
  // Remember, here we are all in global frame
  
  double vx,vy,vz;
  vx = pc.x - trans_gbl2s_.transform.translation.x;
  vy = pc.y - trans_gbl2s_.transform.translation.y;
  vz = pc.z - trans_gbl2s_.transform.translation.z;
  double unit = sqrt(vx*vx + vy*vy + vz*vz);
  if(unit <= 1e-6 || !std::isfinite(unit)){
    return false;
  }
  
  tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

  tf2::Vector3 up_vector(1.0, 0.0, 0.0);
  tf2::Vector3 right_vector = axis_vector.cross(up_vector);
  right_vector.normalized();
  tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
  q.normalize();
  
  //@ We generate vector from sensor to cluster, and set origin by sensor translation, so we get sensor->sensor_pointing_centroid 
  tf2::Transform tf2_gbl2sensor_pointing_clustercentroid;
  tf2_gbl2sensor_pointing_clustercentroid.setRotation(q);
  tf2_gbl2sensor_pointing_clustercentroid.setOrigin(tf2::Vector3(trans_gbl2s_.transform.translation.x, trans_gbl2s_.transform.translation.y, trans_gbl2s_.transform.translation.z));
   
  //@Transform trans_gbl2s_ to tf2 -> Get sensor to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2s;
  tf2::fromMsg(trans_gbl2s_, tf2_trans_gbl2s);
  auto tf2_trans_gbl2s_inverse = tf2_trans_gbl2s.inverse();
  //@ Get sensor to cluster centroid
  tf2::Transform tf2_sensor2sensor_pointing_clustercentroid;
  tf2_sensor2sensor_pointing_clustercentroid.mult(tf2_trans_gbl2s_inverse, tf2_gbl2sensor_pointing_clustercentroid);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_sensor2sensor_pointing_clustercentroid.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  yaw = yaw * 180.0 / 3.1415926535;// change to degree
  if(yaw>=0 && (yaw<scan_effective_positive_start_ || yaw>scan_effective_positive_end_))
    return false;
  else if(yaw<0 && (yaw>scan_effective_negative_start_ || yaw<scan_effective_negative_end_))
    return false;
  else
    return true;

}

void MultiLayerSpinningLidar::pubUpdateLoop()
{

  if(!shared_data_ || !pct_marking_){
    return;
  }

  const auto pub_started_at = std::chrono::steady_clock::now();
  const auto pub_lock_before = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  const auto pub_lock_acquired = std::chrono::steady_clock::now();

  if(shared_data_->dgraph_update_request_[name_]){
    return;
  }

  if(pub_gbl_marking_for_visualization_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg (new pcl::PointCloud<pcl::PointXYZI>);
    //for(auto itx=marking_.begin();itx!=marking_.end();itx++){
    for(auto itx=pct_marking_->getBegin();itx!=pct_marking_->getEnd();itx++){
      for(auto ity=(*itx).second.begin();ity!=(*itx).second.end();ity++){
        if((*itx).second[(*ity).first].empty())
          return;
        for(auto itz=(*itx).second[(*ity).first].begin();itz!=(*itx).second[(*ity).first].end();itz++){
          if((*itz).second.pc_== nullptr){
            continue;
          }
          pcl::PointXYZ pt;
          pt.x = (*itx).first*resolution_;
          pt.y = (*ity).first*resolution_;
          pt.z = (*itz).first*height_resolution_;   
          if((*itz).second.pc_->points.size()>1)
            *pcl_msg += (*(*itz).second.pc_);  
        }
      }
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg;
    pcl_msg->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg, ros_pc2_msg);
    pub_gbl_marking_->publish(ros_pc2_msg);
  }
  
  if(!shared_data_->mapping_mode_){
    //we have no choice but to ignore accessing shared_data_->pcl_ground_, otherwise we have to mutex lock here, reducing perception efficiency
    if(!shared_data_->pcl_ground_){
      return;
    }
    const size_t ground_size =
      std::min(shared_data_->static_ground_size_, shared_data_->pcl_ground_->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t index=0;index<ground_size;index++){
      pcl::PointXYZI ipt;
      ipt.x = shared_data_->pcl_ground_->points[index].x;
      ipt.y = shared_data_->pcl_ground_->points[index].y;
      ipt.z = shared_data_->pcl_ground_->points[index].z;   
      ipt.intensity = pct_marking_->get_dGraphValue(index);
      pcl_msg2->push_back(ipt);
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
    pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
    pub_dGraph_->publish(ros_pc2_msg2);
  }
  {
    const auto finished_at = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(finished_at - pub_started_at).count();
    if(total_ms > 5.0){
      RCLCPP_INFO_THROTTLE(node_->get_logger().get_child(name_), *clock_, 2000,
        "writer-timing pubUpdate total=%.3fms lock_wait=%.3fms work=%.3fms ground=%zu",
        total_ms,
        std::chrono::duration<double, std::milli>(pub_lock_acquired - pub_lock_before).count(),
        std::chrono::duration<double, std::milli>(finished_at - pub_lock_acquired).count(),
        shared_data_->pcl_ground_ ? shared_data_->pcl_ground_->points.size() : 0u);
    }
  }
}

void MultiLayerSpinningLidar::addCastingMarker(const pcl::PointXYZ& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray){

    //@ Creater marker
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = gbl_utils_->getGblFrame();;
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    //markerEdge.lifetime = ros::Duration(2.0);
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = "edges";
    markerEdge.scale.x = 0.03;
    markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
    markerEdge.color.a = 0.2;
    //@ mark
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;   
    markerEdge.points.push_back(p);
    p.x = trans_gbl2s_af3_.translation().x();
    p.y = trans_gbl2s_af3_.translation().y();
    p.z = trans_gbl2s_af3_.translation().z();  
    markerEdge.points.push_back(p);
    markerEdge.id = id+1;
    markerArray.markers.push_back(markerEdge);
}


void MultiLayerSpinningLidar::startSnapshotCycle(std::int64_t sensor_stamp_ns){
  if(!shared_data_ ||
    !shared_data_->scene_producer_enabled_.load(std::memory_order_acquire))
  {
    return;
  }
  current_scan_stamp_ns_ = sensor_stamp_ns;
  current_scan_sequence_ += 1;
  export_clear_completed_ = false;
  if(pending_cleared_points_){
    pending_cleared_points_->clear();
  }
}

void MultiLayerSpinningLidar::appendActiveMarkingPoints(
  pcl::PointCloud<pcl::PointXYZI> & cloud) const
{
  if(!pct_marking_){
    return;
  }
  for(auto it_x = pct_marking_->getBegin(); it_x != pct_marking_->getEnd(); ++it_x){
    for(auto it_y = (*it_x).second.begin(); it_y != (*it_x).second.end(); ++it_y){
      for(auto it_z = (*it_y).second.begin(); it_z != (*it_y).second.end(); ++it_z){
        const auto & marking = (*it_z).second;
        if(!marking.pc_ || marking.pc_->empty()){
          continue;
        }
        cloud += *marking.pc_;
      }
    }
  }
}

void MultiLayerSpinningLidar::finalizeSnapshotObservation(){
  if(!shared_data_ ||
    !shared_data_->scene_producer_enabled_.load(std::memory_order_acquire))
  {
    //@ Producer OFF: do not traverse active markings, copy clearing records or
    //@ replace the legacy observation cache. Local direct observation is set by
    //@ cbPreprocessed/cbSensor and is unaffected.
    return;
  }
  if(!export_clear_completed_ || current_scan_sequence_ == 0 ||
    current_scan_stamp_ns_ <= 0)
  {
    return;
  }
  //@ A second mark pass for the same accepted scan must not mint a fresh
  //@ completion revision/receipt for already frozen evidence.
  if(completed_state_scan_sequence_ == current_scan_sequence_)
  {
    export_clear_completed_ = false;
    return;
  }

  //@ Legacy-cost-only mode (producer enabled, evidence disabled) still needs a
  //@ completed-state metadata mutation so the cost cache cannot bind old dGraph
  //@ values to an accepted-but-unprocessed input. Do not traverse markings or
  //@ copy any points in that mode.
  if(!shared_data_->scene_evidence_enabled_.load(std::memory_order_acquire))
  {
    completed_state_revision_ += 1;
    completed_state_stamp_ns_ = current_scan_stamp_ns_;
    completed_state_scan_sequence_ = current_scan_sequence_;
    export_clear_completed_ = false;
    return;
  }
  if(!pending_cleared_points_)
  {
    return;
  }

  auto occupied = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  appendActiveMarkingPoints(*occupied);

  //@ Persistent markings outside the current clearing window stay active; the
  //@ complete replacement therefore carries all currently marked clusters, not
  //@ just this cycle's additions. Non-finite points are not exportable.
  auto finite_occupied = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::set<std::tuple<float, float, float>> occupied_keys;
  for(const auto & point : occupied->points){
    if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z))
    {
      continue;
    }
    finite_occupied->push_back(point);
    occupied_keys.emplace(point.x, point.y, point.z);
  }

  //@ A point re-observed as occupied in the same completed cycle wins over an
  //@ older clear record at that exact coordinate.
  auto cleared = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for(const auto & point : pending_cleared_points_->points){
    if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z))
    {
      continue;
    }
    if(occupied_keys.find(std::make_tuple(point.x, point.y, point.z)) !=
      occupied_keys.end())
    {
      continue;
    }
    cleared->push_back(point);
  }

  const std::string frame_id =
    gbl_utils_ ? gbl_utils_->getGblFrame() : std::string();
  const std::uint64_t stamp_us =
    static_cast<std::uint64_t>(current_scan_stamp_ns_) / 1000ULL;
  finite_occupied->header.frame_id = frame_id;
  finite_occupied->header.stamp = stamp_us;
  cleared->header.frame_id = frame_id;
  cleared->header.stamp = stamp_us;

  export_occupied_points_ = finite_occupied;
  export_cleared_points_ = cleared;
  export_frame_id_ = frame_id;
  export_scan_stamp_ns_ = current_scan_stamp_ns_;
  export_received_steady_ns_ = std::chrono::duration_cast<
    std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  export_source_sequence_ = current_scan_sequence_;
  export_cycle_complete_ = true;
  export_clear_completed_ = false;
  //@ The full clear/mark section is now complete. Advance the metadata that
  //@ keys the legacy cost cache only after this point, using the completed
  //@ state's own revision/stamp rather than the accepted input revision.
  completed_state_revision_ += 1;
  completed_state_stamp_ns_ = current_scan_stamp_ns_;
  completed_state_scan_sequence_ = current_scan_sequence_;

  //@ Keep the legacy global observation cache meaningful without changing the
  //@ local path: it now mirrors the active dynamic marking set at cycle end.
  sensor_current_observation_ =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*finite_occupied);
  pending_cleared_points_->clear();
}

void MultiLayerSpinningLidar::resetdGraph(){
  const auto reset_started_at = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
  pct_marking_ = std::make_shared<Marking>(&dGraph_, 
        gbl_utils_->getInscribedRadius(), gbl_utils_->getInflationRadius(), shared_data_, resolution_, height_resolution_);
  pending_mark_ = pending_clear_ = have_preprocessed_observation_;
  //@ The graph that justified the previous export no longer exists. Keep the
  //@ producer sequence monotonic, but require a new complete cycle.
  export_cycle_complete_ = false;
  export_clear_completed_ = false;
  if(pending_cleared_points_){
    pending_cleared_points_->clear();
  }
  if(sensor_current_observation_){
    sensor_current_observation_->clear();
  }
  //@ A reset is a completed-state mutation even when no new input arrives. It
  //@ invalidates the cost cache revision but never advances the completed
  //@ acquisition stamp, because no new scan was completed.
  if(shared_data_ &&
    shared_data_->scene_producer_enabled_.load(std::memory_order_acquire))
  {
    completed_state_revision_ += 1;
    completed_state_scan_sequence_ = 0;
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s done dynamic graph regeneration (%.3f ms).", name_.c_str(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - reset_started_at).count());
}

double MultiLayerSpinningLidar::get_dGraphValue(const unsigned int index){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(!pct_marking_){
    return gbl_utils_ ? gbl_utils_->getMaxObstacleDistance() : 9999.0;
  }
  return pct_marking_->get_dGraphValue(index);
}

bool MultiLayerSpinningLidar::isCurrent(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(input_preprocessed_){
    current_ = have_preprocessed_observation_ &&
      IsCloudStampFresh(last_observation_time_, clock_->now(), expected_sensor_time_);
    return current_;
  }
  
  auto time_diff = (clock_->now() - last_observation_time_).seconds();
  if(time_diff > expected_sensor_time_)
    current_ = false;
  else
    current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLayerSpinningLidar::getObservation(){
  return sensor_current_observation_;
}

bool MultiLayerSpinningLidar::supportsSnapshotObservation() const{
  return !is_local_planner_ && shared_data_ &&
    shared_data_->scene_evidence_enabled_.load(std::memory_order_acquire);
}

std::uint64_t MultiLayerSpinningLidar::snapshotStateRevision() const{
  //@ Under the producer lock: revision of the COMPLETED clear/mark/reset
  //@ state. Accepting a new input cloud alone must not advance this, otherwise
  //@ a capture would cache old dGraph values under the new accepted revision.
  return completed_state_revision_;
}

std::int64_t MultiLayerSpinningLidar::snapshotSensorStampNs() const{
  //@ Acquisition stamp of the completed state, never the accepted-but-
  //@ unprocessed current_scan_stamp_ns_.
  return completed_state_stamp_ns_;
}

bool MultiLayerSpinningLidar::getSnapshotObservation(
  SensorExportObservation * observation)
{
  if(!observation || is_local_planner_ || !shared_data_ ||
    !shared_data_->scene_evidence_enabled_.load(std::memory_order_acquire))
  {
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(!export_cycle_complete_ || !export_occupied_points_ ||
    !export_cleared_points_ || export_scan_stamp_ns_ <= 0 ||
    export_source_sequence_ == 0 || export_frame_id_.empty())
  {
    return false;
  }

  //@ Deep-copy under the producer lock; the caller never aliases mutable PCL
  //@ storage or gets a partially updated cycle.
  observation->occupied =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*export_occupied_points_);
  observation->cleared =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*export_cleared_points_);
  observation->frame_id = export_frame_id_;
  observation->sensor_stamp_ns = export_scan_stamp_ns_;
  observation->received_steady_ns = export_received_steady_ns_;
  observation->source_sequence = export_source_sequence_;
  observation->complete = true;
  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MultiLayerSpinningLidar::getLethal(){
  return current_lethal_;
}

}//end of name space
