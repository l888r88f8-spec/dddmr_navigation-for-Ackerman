// Copyright 2026 DDDMobileRobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <exception>
#include <memory>

#include "dddmr_grid_global_planner/grid_global_planner.h"
#include "rclcpp/rclcpp.hpp"

// Dedicated static-PCD-only entry point.
//
// This process owns exactly one node: the grid planner. It does not construct
// the perception stack, a lidar plugin, the cloud preprocessor, localization or
// the legacy global planner. With offline.enabled=true the planner loads the
// configured ground/obstacle PCDs once, builds the immutable offline
// MapSnapshot, and serves the normal /get_plan action. RViz selection and the
// existing bridge are unchanged; TF and map publication remain display-only
// helpers of the launch file.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::NodeOptions options;
    auto planner =
      std::make_shared<dddmr_grid_global_planner::GridGlobalPlanner>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner);
    executor.spin();

    executor.remove_node(planner);
    planner.reset();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("dddmr_grid_offline_planner_node"),
      "startup/runtime exception: %s", exception.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("dddmr_grid_offline_planner_node"),
      "startup/runtime unknown exception");
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
