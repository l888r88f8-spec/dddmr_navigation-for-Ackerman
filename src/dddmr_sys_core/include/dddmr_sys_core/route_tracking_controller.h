#ifndef DDDMR_SYS_CORE_ROUTE_TRACKING_CONTROLLER_H
#define DDDMR_SYS_CORE_ROUTE_TRACKING_CONTROLLER_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include "dddmr_sys_core/dddmr_enum_states.h"

namespace dddmr_sys_core {

/**
 * Stable controller-side abstraction consumed by the goal orchestrator.
 *
 * The orchestrator should only care about route updates, alignment checks,
 * goal checks, and velocity commands. It should not depend on a concrete
 * rollout implementation or trajectory-generator internals.
 */
class RouteTrackingController
{
public:
  virtual ~RouteTrackingController() = default;

  virtual void setRoute(
    const std::vector<geometry_msgs::msg::PoseStamped> & route,
    std::size_t route_version = 0,
    std::size_t goal_seq = 0,
    const std::string & source_label = "planner_result") = 0;

  virtual PlannerState computeControlCommand(
    const std::string & controller_name,
    geometry_msgs::msg::Twist * cmd_vel) = 0;

  virtual bool isGoalPositionReached() = 0;
  virtual bool isRouteStartAligned() = 0;
  virtual bool isGoalHeadingSatisfied() = 0;
  virtual geometry_msgs::msg::TransformStamped getGlobalPose() = 0;
};

}  // namespace dddmr_sys_core

#endif  // DDDMR_SYS_CORE_ROUTE_TRACKING_CONTROLLER_H
