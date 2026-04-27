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
#include <trajectory_generators/ackermann_simple_trajectory_generator_theory.h>

#include <algorithm>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::AckermannSimpleTrajectoryGeneratorTheory, trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

AckermannSimpleTrajectoryGeneratorTheory::AckermannSimpleTrajectoryGeneratorTheory(){
  min_sim_distance_ = 0.0;
  return;
}

void AckermannSimpleTrajectoryGeneratorTheory::onInitialize(){

  limits_ = std::make_shared<trajectory_generators::DDTrajectoryGeneratorLimits>();

  node_->declare_parameter(name_ + ".min_vel_x", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_x", limits_->min_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_x: %.2f", limits_->min_vel_x);

  node_->declare_parameter(name_ + ".max_vel_x", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_x", limits_->max_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_x: %.2f", limits_->max_vel_x);

  node_->declare_parameter(name_ + ".min_vel_theta", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".min_vel_theta", limits_->min_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_theta: %.2f", limits_->min_vel_theta);

  node_->declare_parameter(name_ + ".max_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_theta", limits_->max_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_theta: %.2f", limits_->max_vel_theta);

  node_->declare_parameter(name_ + ".acc_lim_x", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_x", limits_->acc_lim_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_x: %.2f", limits_->acc_lim_x);

  node_->declare_parameter(name_ + ".acc_lim_theta", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".acc_lim_theta", limits_->acc_lim_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_theta: %.2f", limits_->acc_lim_theta);

  node_->declare_parameter(name_ + ".prune_forward", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".prune_forward", limits_->prune_forward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_forward: %.2f", limits_->prune_forward);

  node_->declare_parameter(name_ + ".prune_backward", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".prune_backward", limits_->prune_backward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_backward: %.2f", limits_->prune_backward);

  node_->declare_parameter(name_ + ".deceleration_ratio", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".deceleration_ratio", limits_->deceleration_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "deceleration_ratio: %.2f", limits_->deceleration_ratio);

  if(limits_->min_vel_x<0){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "The min velocity of the Ackermann robot should be positive!");
  }

  node_->declare_parameter(name_ + ".use_motor_constraint", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".use_motor_constraint", limits_->use_motor_constraint);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_motor_constraint: %d", limits_->use_motor_constraint);

  node_->declare_parameter(name_ + ".max_motor_shaft_rpm", rclcpp::ParameterValue(3000.0));
  node_->get_parameter(name_ + ".max_motor_shaft_rpm", limits_->max_motor_shaft_rpm);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_motor_shaft_rpm: %.2f", limits_->max_motor_shaft_rpm);

  node_->declare_parameter(name_ + ".wheel_diameter", rclcpp::ParameterValue(0.15));
  node_->get_parameter(name_ + ".wheel_diameter", limits_->wheel_diameter);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheel_diameter: %.2f", limits_->wheel_diameter);

  node_->declare_parameter(name_ + ".gear_ratio", rclcpp::ParameterValue(30.0));
  node_->get_parameter(name_ + ".gear_ratio", limits_->gear_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "gear_ratio: %.2f", limits_->gear_ratio);

  node_->declare_parameter(name_ + ".robot_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".robot_radius", limits_->robot_radius);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "robot_radius: %.2f", limits_->robot_radius);

  node_->declare_parameter(name_ + ".wheelbase", rclcpp::ParameterValue(0.55));
  node_->get_parameter(name_ + ".wheelbase", wheelbase_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheelbase: %.2f", wheelbase_);

  node_->declare_parameter(name_ + ".max_steer", rclcpp::ParameterValue(0.69));
  node_->get_parameter(name_ + ".max_steer", max_steer_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_steer: %.2f", max_steer_);

  params_ = std::make_shared<trajectory_generators::DDTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);

  node_->declare_parameter(name_ + ".min_sim_distance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".min_sim_distance", min_sim_distance_);
  min_sim_distance_ = std::max(0.0, min_sim_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_sim_distance: %.2f", min_sim_distance_);

  node_->declare_parameter(name_ + ".linear_x_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_x_sample", params_->linear_x_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_x_sample: %.2f", params_->linear_x_sample);

  node_->declare_parameter(name_ + ".angular_z_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".angular_z_sample", params_->angular_z_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_z_sample: %.2f", params_->angular_z_sample);

  node_->declare_parameter(name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_granularity: %.2f", params_->sim_granularity);

  node_->declare_parameter(name_ + ".angular_sim_granularity", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".angular_sim_granularity", params_->angular_sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_sim_granularity: %.2f", params_->angular_sim_granularity);

  RCLCPP_INFO(node_->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<double> p;

  node_->declare_parameter(name_ + ".cuboid.flb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flb= node_->get_parameter(name_ + ".cuboid.flb");
  p = cuboid_flb.as_double_array();
  pcl::PointXYZ pt_flb;
  pt_flb.x = p[0];pt_flb.y = p[1];pt_flb.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.frb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frb= node_->get_parameter(name_ + ".cuboid.frb");
  p = cuboid_frb.as_double_array();
  pcl::PointXYZ pt_frb;
  pt_frb.x = p[0];pt_frb.y = p[1];pt_frb.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.flt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flt= node_->get_parameter(name_ + ".cuboid.flt");
  p = cuboid_flt.as_double_array();
  pcl::PointXYZ pt_flt;
  pt_flt.x = p[0];pt_flt.y = p[1];pt_flt.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.frt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frt= node_->get_parameter(name_ + ".cuboid.frt");
  p = cuboid_frt.as_double_array();
  pcl::PointXYZ pt_frt;
  pt_frt.x = p[0];pt_frt.y = p[1];pt_frt.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.blb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blb= node_->get_parameter(name_ + ".cuboid.blb");
  p = cuboid_blb.as_double_array();
  pcl::PointXYZ pt_blb;
  pt_blb.x = p[0];pt_blb.y = p[1];pt_blb.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.brb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brb= node_->get_parameter(name_ + ".cuboid.brb");
  p = cuboid_brb.as_double_array();
  pcl::PointXYZ pt_brb;
  pt_brb.x = p[0];pt_brb.y = p[1];pt_brb.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.blt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blt= node_->get_parameter(name_ + ".cuboid.blt");
  p = cuboid_blt.as_double_array();
  pcl::PointXYZ pt_blt;
  pt_blt.x = p[0];pt_blt.y = p[1];pt_blt.z = p[2];

  node_->declare_parameter(name_ + ".cuboid.brt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brt= node_->get_parameter(name_ + ".cuboid.brt");
  p = cuboid_brt.as_double_array();
  pcl::PointXYZ pt_brt;
  pt_brt.x = p[0];pt_brt.y = p[1];pt_brt.z = p[2];

  params_->cuboid.push_back(pt_blb);
  params_->cuboid.push_back(pt_brb);
  params_->cuboid.push_back(pt_blt);
  params_->cuboid.push_back(pt_flb);
  params_->cuboid.push_back(pt_brt);
  params_->cuboid.push_back(pt_frt);
  params_->cuboid.push_back(pt_flt);
  params_->cuboid.push_back(pt_frb);

  if(params_->cuboid.size()!=8){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "Cuboid is essential.");
  }
}

double AckermannSimpleTrajectoryGeneratorTheory::getMaxYawRateForSpeed(double linear_x) const {
  double speed = std::abs(linear_x);
  if(speed < 1e-6){
    return 0.0;
  }
  return speed * std::tan(max_steer_) / std::max(wheelbase_, 1e-6);
}

void AckermannSimpleTrajectoryGeneratorTheory::initialise(){
  double max_vel_th = limits_->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  Eigen::Vector3f acc_lim = limits_->getAccLimits();
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits_->min_vel_x;
  double max_vel_x = limits_->max_vel_x;

  if (params_->linear_x_sample * params_->angular_z_sample > 0) {
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    double sim_period = 1.0/params_->controller_frequency;

    if(shared_data_->current_allowed_max_linear_speed_>0.0){
      max_vel_x = std::min(max_vel_x, shared_data_->current_allowed_max_linear_speed_);
    }

    max_vel[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x + acc_lim[0] * sim_period);
    // Ackermann steering is bounded by geometry (wheelbase + max steer), and
    // applying a differential-drive-like angular acceleration window here
    // makes the initial steering samples far too narrow for obstacle
    // avoidance. Sample the full feasible yaw-rate range for each forward
    // speed, then clamp it by steering geometry below.
    max_vel[2] = max_vel_th;

    min_vel[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    min_vel[2] = min_vel_th;

    if(max_vel[0] < min_vel[0]){
      // When starting from rest, acceleration limits can make the reachable
      // velocity window narrower than the configured cruise-speed floor.
      // Keep sampling inside the physically reachable window instead of
      // collapsing the search to the current speed (often zero).
      min_vel[0] = max_vel[0];
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    trajectory_generators::VelocityIterator x_it(min_vel[0], max_vel[0], params_->linear_x_sample);
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();

      double steering_limited_yaw_rate = getMaxYawRateForSpeed(vel_samp[0]);
      double sample_min_vel_th = std::max(static_cast<double>(min_vel[2]), -steering_limited_yaw_rate);
      double sample_max_vel_th = std::min(static_cast<double>(max_vel[2]), steering_limited_yaw_rate);

      if(sample_max_vel_th < sample_min_vel_th){
        continue;
      }

      trajectory_generators::VelocityIterator th_it(sample_min_vel_th, sample_max_vel_th, params_->angular_z_sample);
      for(; !th_it.isFinished(); th_it++) {
        vel_samp[2] = th_it.getVelocity();
        sample_params_.push_back(vel_samp);
      }
    }
  }
}

bool AckermannSimpleTrajectoryGeneratorTheory::hasMoreTrajectories(){
  return next_sample_index_ < sample_params_.size();
}

bool AckermannSimpleTrajectoryGeneratorTheory::nextTrajectory(base_trajectory::Trajectory& _traj){
  bool result = false;
  bool generated_once = false;
  if (hasMoreTrajectories()) {

    if (generateTrajectory(
        sample_params_[next_sample_index_],
        _traj)) {
      result = true;
      generated_once = true;
    }
    else{
      _traj.resetPoints();
    }
  }
  else{
    if(!generated_once){
      RCLCPP_ERROR(node_->get_logger().get_child(name_), "None of Ackermann trajectories is generated, maybe acc is too small or check the generateTrajectory function.");
    }
  }
  next_sample_index_++;
  return result;
}

bool AckermannSimpleTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::Vector3f sample_target_vel,
      base_trajectory::Trajectory& traj) {

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = fabs(sample_target_vel[0]);
  double eps = 1e-4;
  traj.cost_ = 0.0;
  traj.resetPoints();

  if ((limits_->min_vel_x >= 0 && vmag + eps < limits_->min_vel_x) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }

  if (limits_->max_vel_x >=0 && vmag - eps > limits_->max_vel_x) {
    return false;
  }

  double steering_limited_yaw_rate = getMaxYawRateForSpeed(sample_target_vel[0]);
  if(fabs(sample_target_vel[2]) - eps > steering_limited_yaw_rate){
    return false;
  }

  int num_steps;
  double effective_sim_time = params_->sim_time;
  double sim_time_distance = vmag * effective_sim_time;
  if(vmag > eps && min_sim_distance_ > sim_time_distance){
    effective_sim_time = min_sim_distance_ / vmag;
    sim_time_distance = min_sim_distance_;
  }
  double sim_time_angle = fabs(sample_target_vel[2]) * effective_sim_time;
  num_steps =
      ceil(std::max(sim_time_distance / params_->sim_granularity,
          sim_time_angle / params_->angular_sim_granularity));

  if (num_steps == 0) {
    return false;
  }

  double dt = effective_sim_time / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  loop_vel = sample_target_vel;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = 0.0;
  traj.thetav_ = sample_target_vel[2];

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  for (int i = 0; i < num_steps; ++i) {
    pos = computeNewPositions(pos, loop_vel, dt);

    Eigen::Affine3d trans_gbl2traj_af3;
    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];

    trans_gbl2traj_af3 = pos_af3*trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj_ = tf2::eigenToTransform (trans_gbl2traj_af3);
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj_.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj_.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj_.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj_.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(params_->cuboid, pc_out, trans_gbl2traj_af3);

    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if(!traj.addPoint(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }
  }

  return true;
}

Eigen::Vector3f AckermannSimpleTrajectoryGeneratorTheory::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

}//end of name space
