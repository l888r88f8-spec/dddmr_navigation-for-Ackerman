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

#include "grid_global_planner/grid_global_planner.h"
#include "perception_3d/perception_3d_ros.h"
#include "perception_3d/scene_snapshot.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

// The provider captures the perception node weakly. The planner polls this from
// its own timer, and shutdown destroys the planner before the perception node,
// so a late poll can only observe a failed lock, never a dangling node.
grid_global_planner::GridGlobalPlanner::SceneSnapshotProvider
MakeWeakSceneProvider(
  const std::shared_ptr<perception_3d::Perception3D_ROS> & perception)
{
  std::weak_ptr<perception_3d::Perception3D_ROS> weak_perception = perception;
  return [weak_perception]()
         -> std::shared_ptr<const perception_3d::SceneSnapshot> {
           const std::shared_ptr<perception_3d::Perception3D_ROS> node =
             weak_perception.lock();
           if (!node) {
             return nullptr;
           }
           return node->getLatestSceneSnapshot();
         };
}

// The producer's static layer owns the ground-merge parameters. They are read
// from the already-initialized perception node so Grid can derive the exact
// producer-compatible source fingerprint from its raw mapground input without
// duplicating the configuration or the merge/hash algorithms.
grid_global_planner::StaticSourceInputOptions ReadStaticSourceOptions(
  const std::shared_ptr<perception_3d::Perception3D_ROS> & perception)
{
  grid_global_planner::StaticSourceInputOptions options;
  perception->get_parameter_or<bool>(
    "map.support.merge_ground_layers", options.merge_ground_layers, false);
  perception->get_parameter_or<double>(
    "map.support.merge_ground_xy_resolution",
    options.merge_ground_xy_resolution, 0.1);
  perception->get_parameter_or<double>(
    "map.support.merge_ground_z_tolerance",
    options.merge_ground_z_tolerance, 0.12);
  return options;
}

}  // namespace

// Live grid perception entry point.
//
// One process owns exactly one perception_3d_global instance and one
// GridGlobalPlanner. Both nodes are constructed only after perception
// initial() has declared its parameters/plugins and created its timers, so no
// executor callback can run against a half-initialized perception node. The
// launch intentionally leaves the launch Node name unset: a global __node
// remap would rename BOTH in-process nodes and break their parameter names.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  bool perception_added = false;

  std::shared_ptr<perception_3d::Perception3D_ROS> perception;
  std::shared_ptr<grid_global_planner::GridGlobalPlanner> planner;
  rclcpp::executors::MultiThreadedExecutor executor;

  try {
    perception = std::make_shared<perception_3d::Perception3D_ROS>(
      "perception_3d_global");
    // Register before initialization only so a later exception can be cleaned
    // up uniformly; no callback is dispatched until executor.spin().
    executor.add_node(perception);
    perception_added = true;
    perception->initial();

    // Configure the producer exactly once, before the executor spins: evidence
    // only, no legacy static/cost freeze, static source plugin "map". Ordinary
    // ROS evidence export then publishes the root SceneSnapshot automatically.
    perception->configureSceneProducer(
      perception_3d::SceneProducerConfig{true, true, false, "map"});

    // The provider and static-source options are read only after initial() has
    // declared every plugin parameter, so they can never expose a half-built
    // producer or a guessed merge configuration.
    const auto provider = MakeWeakSceneProvider(perception);
    const grid_global_planner::StaticSourceInputOptions static_source =
      ReadStaticSourceOptions(perception);
    rclcpp::NodeOptions planner_options;
    planner = std::make_shared<grid_global_planner::GridGlobalPlanner>(
      planner_options, provider, static_source);
    executor.add_node(planner);

    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("grid_global_planner_perception_node"),
      "startup/runtime exception: %s", exception.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("grid_global_planner_perception_node"),
      "startup/runtime unknown exception");
    exit_code = 1;
  }

  // Destroy the planner first: its destructor cancels the provider timer and
  // joins its own threads before the perception node's plugins/timers die.
  if (planner) {
    executor.remove_node(planner);
    planner.reset();
  }
  if (perception) {
    if (perception_added) {
      executor.remove_node(perception);
    }
    perception.reset();
  }
  rclcpp::shutdown();
  return exit_code;
}
