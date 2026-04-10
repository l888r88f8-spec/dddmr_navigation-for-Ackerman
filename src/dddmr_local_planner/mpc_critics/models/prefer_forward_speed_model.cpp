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
#include <algorithm>
#include <mpc_critics/prefer_forward_speed_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::PreferForwardSpeedModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

PreferForwardSpeedModel::PreferForwardSpeedModel(){
  return;
  
}

void PreferForwardSpeedModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".target_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".target_speed", target_speed_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "target_speed: %.2f", target_speed_);

  node_->declare_parameter(name_ + ".goal_slowdown_distance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".goal_slowdown_distance", goal_slowdown_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "goal_slowdown_distance: %.2f", goal_slowdown_distance_);

  node_->declare_parameter(name_ + ".goal_slowdown_min_speed", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".goal_slowdown_min_speed", goal_slowdown_min_speed_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "goal_slowdown_min_speed: %.2f", goal_slowdown_min_speed_);

}

double PreferForwardSpeedModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  double forward_speed = std::max(0.0, static_cast<double>(traj.xv_));
  double effective_target_speed = target_speed_;

  if(goal_slowdown_distance_ > 0.0 &&
     !shared_data_->prune_plan_.poses.empty() &&
     traj.getPointsSize() > 0)
  {
    const auto& goal = shared_data_->prune_plan_.poses.back().pose.position;
    const auto& traj_end = traj.getPoint(traj.getPointsSize()-1).pose.position;
    double dx = goal.x - traj_end.x;
    double dy = goal.y - traj_end.y;
    double dz = goal.z - traj_end.z;
    double goal_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    double ratio = std::clamp(goal_distance / goal_slowdown_distance_, 0.0, 1.0);
    effective_target_speed =
      goal_slowdown_min_speed_ + (target_speed_ - goal_slowdown_min_speed_) * ratio;
  }

  double speed_gap = std::max(0.0, effective_target_speed - forward_speed);
  return speed_gap * weight_;
}

}//end of name space
