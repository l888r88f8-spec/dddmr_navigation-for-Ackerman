#ifndef PERCEPTION_3D_PARAMETER_UTILS_H_
#define PERCEPTION_3D_PARAMETER_UTILS_H_

#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace perception_3d
{

template<typename T>
T declareOrGetParameter(
  rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

inline std::string getStringParameter(
  rclcpp::Node & node,
  const std::string & legacy_name,
  const std::string & common_name,
  const std::string & default_value)
{
  const std::string legacy = declareOrGetParameter<std::string>(node, legacy_name, "");
  if (!legacy.empty()) {
    return legacy;
  }
  const std::string common = declareOrGetParameter<std::string>(
    node, common_name, default_value);
  return common.empty() ? default_value : common;
}

inline double getDoubleParameter(
  rclcpp::Node & node,
  const std::string & legacy_name,
  const std::string & common_name,
  double default_value)
{
  const double legacy = declareOrGetParameter<double>(
    node, legacy_name, std::numeric_limits<double>::quiet_NaN());
  if (std::isfinite(legacy)) {
    return legacy;
  }
  return declareOrGetParameter<double>(node, common_name, default_value);
}

}  // namespace perception_3d

#endif  // PERCEPTION_3D_PARAMETER_UTILS_H_
