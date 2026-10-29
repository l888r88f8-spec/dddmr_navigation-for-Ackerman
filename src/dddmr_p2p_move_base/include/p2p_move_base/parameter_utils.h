#ifndef P2P_MOVE_BASE_PARAMETER_UTILS_H_
#define P2P_MOVE_BASE_PARAMETER_UTILS_H_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace p2p_move_base
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

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_PARAMETER_UTILS_H_
