/*
* BSD 3-Clause License
*/
#ifndef CUBOID_PARAMETER_UTILS_H_
#define CUBOID_PARAMETER_UTILS_H_

#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace trajectory_generators
{

inline bool readDoubleArrayParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_name,
  std::vector<double> * values)
{
  if(!node->has_parameter(parameter_name)){
    node->declare_parameter(parameter_name, rclcpp::PARAMETER_DOUBLE_ARRAY);
  }

  const auto parameter = node->get_parameter(parameter_name);
  if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    return false;
  }
  if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY){
    throw std::runtime_error(parameter_name + " must be a double array.");
  }

  *values = parameter.as_double_array();
  return true;
}

inline pcl::PointXYZ readCuboidPointParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & plugin_name,
  const std::string & vertex_name)
{
  std::vector<double> values;
  const std::string shared_parameter = "cuboid." + vertex_name;
  const std::string legacy_parameter = plugin_name + ".cuboid." + vertex_name;

  if(!readDoubleArrayParameter(node, shared_parameter, &values) &&
     !readDoubleArrayParameter(node, legacy_parameter, &values)){
    throw std::runtime_error(
      "Missing cuboid parameter: " + shared_parameter + " or " + legacy_parameter);
  }

  if(values.size() != 3){
    throw std::runtime_error(shared_parameter + " must contain exactly 3 values.");
  }

  pcl::PointXYZ point;
  point.x = values[0];
  point.y = values[1];
  point.z = values[2];
  return point;
}

inline pcl::PointCloud<pcl::PointXYZ> loadCuboidParameters(
  const rclcpp::Node::SharedPtr & node,
  const std::string & plugin_name)
{
  const std::array<std::string, 8> vertices = {
    "flb", "frb", "flt", "frt", "blb", "brb", "blt", "brt"};

  std::map<std::string, pcl::PointXYZ> points;
  for(const auto & vertex : vertices){
    const auto point = readCuboidPointParameter(node, plugin_name, vertex);
    points[vertex] = point;
    RCLCPP_INFO(
      node->get_logger().get_child(plugin_name),
      "Cuboid %s: %.2f, %.2f, %.2f",
      vertex.c_str(), point.x, point.y, point.z);
  }

  pcl::PointCloud<pcl::PointXYZ> cuboid;
  cuboid.push_back(points["blb"]);
  cuboid.push_back(points["brb"]);
  cuboid.push_back(points["blt"]);
  cuboid.push_back(points["flb"]);
  cuboid.push_back(points["brt"]);
  cuboid.push_back(points["frt"]);
  cuboid.push_back(points["flt"]);
  cuboid.push_back(points["frb"]);
  return cuboid;
}

}  // namespace trajectory_generators

#endif
