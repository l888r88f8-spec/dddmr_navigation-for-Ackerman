#include <gtest/gtest.h>

#include <perception_3d/parameter_utils.h>

#include <memory>

namespace
{
class ParameterUtils : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ParameterUtils, LegacyPluginValueOverridesSharedFallback)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"map_topic", "shared/map"},
    {"map.map_topic", "legacy/map"},
    {"perception_window_size", 4.0},
    {"lidar.perception_window_size", 5.0},
  });
  auto node = std::make_shared<rclcpp::Node>("parameter_utils_legacy", options);

  EXPECT_EQ(
    perception_3d::getStringParameter(*node, "map.map_topic", "map_topic", "default"),
    "legacy/map");
  EXPECT_DOUBLE_EQ(
    perception_3d::getDoubleParameter(
      *node, "lidar.perception_window_size", "perception_window_size", 6.0),
    5.0);
}

TEST_F(ParameterUtils, SharedValueIsUsedWhenLegacyValueIsAbsent)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"ground_topic", "shared/ground"},
    {"marking_height", 1.5},
  });
  auto node = std::make_shared<rclcpp::Node>("parameter_utils_shared", options);

  EXPECT_EQ(
    perception_3d::getStringParameter(*node, "map.ground_topic", "ground_topic", "default"),
    "shared/ground");
  EXPECT_DOUBLE_EQ(
    perception_3d::getDoubleParameter(*node, "lidar.marking_height", "marking_height", 2.0),
    1.5);
}

TEST_F(ParameterUtils, RepeatedDeclarationsAreIdempotent)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"map_topic", "shared/map"}});
  auto node = std::make_shared<rclcpp::Node>("parameter_utils_repeat", options);

  EXPECT_EQ(
    perception_3d::getStringParameter(*node, "missing.map_topic", "map_topic", "default"),
    "shared/map");
  EXPECT_NO_THROW(
    EXPECT_EQ(
      perception_3d::getStringParameter(*node, "missing.map_topic", "map_topic", "default"),
      "shared/map"));
}
}  // namespace
