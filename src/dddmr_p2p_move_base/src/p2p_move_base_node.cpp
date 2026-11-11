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


#include <exception>
#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <recovery_behaviors/recovery_behaviors_ros.h>
#include <local_planner/local_planner.h>
#include <p2p_move_base/p2p_global_plan_manager.h>
#include <p2p_move_base/p2p_move_base.h>


int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_rb = std::make_shared<recovery_behaviors::Recovery_Behaviors_ROS>("recovery_behaviors", node_p3, node_mc, node_tg);
  auto node_lp = std::make_shared<local_planner::Local_Planner>("local_planner");
  auto node_gpm = std::make_shared<p2p_move_base::P2PGlobalPlanManager>("global_plan_manager");
  auto node_p2p = std::make_shared<p2p_move_base::P2PMoveBase>("p2p_move_base");

  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_lp);
  executor.add_node(node_rb);
  executor.add_node(node_gpm);
  executor.add_node(node_p2p);
  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_rb->initial();
  node_lp->initial(node_p3, node_mc, node_tg);
  node_gpm->initial();
  node_p2p->initial(node_lp, node_gpm);

  int exit_code = 0;
  try{
    executor.spin();
  }
  catch(const std::exception & ex){
    RCLCPP_ERROR(
      node_p2p->get_logger(),
      "executor.spin terminated with exception: %s",
      ex.what());
    exit_code = 1;
  }
  catch(...){
    RCLCPP_ERROR(node_p2p->get_logger(), "executor.spin terminated with unknown exception");
    exit_code = 1;
  }

  // Always stop and join navigation execution before dependencies are released.
  try{
    node_p2p->stopExecution();
  }
  catch(const std::exception & ex){
    RCLCPP_ERROR(
      node_p2p->get_logger(),
      "failed to stop navigation execution worker cleanly: %s",
      ex.what());
    exit_code = 1;
  }
  catch(...){
    RCLCPP_ERROR(
      node_p2p->get_logger(),
      "failed to stop navigation execution worker cleanly: unknown exception");
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
