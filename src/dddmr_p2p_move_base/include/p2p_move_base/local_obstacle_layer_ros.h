#ifndef P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_ROS_H_
#define P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_ROS_H_

#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "p2p_move_base/local_obstacle_layer.h"

namespace p2p_move_base
{

std::vector<LocalPoint> pointCloud2ToLocalPoints(const sensor_msgs::msg::PointCloud2 & cloud);

std_msgs::msg::String snapshotToStatusMessage(const LocalObstacleSnapshot & snapshot);

sensor_msgs::msg::PointCloud2 snapshotToPointCloud2(
  const LocalObstacleSnapshot & snapshot,
  const std_msgs::msg::Header & header);

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_ROS_H_
