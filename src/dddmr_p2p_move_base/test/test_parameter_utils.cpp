#include <gtest/gtest.h>

#include <p2p_move_base/parameter_utils.h>

#include <memory>

namespace
{
class P2PParameterUtils : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(P2PParameterUtils, LegacyDiagnosticInputOverridesSharedCloudTopic)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"input_cloud_topic", "shared/raw"},
    {"local_obstacle_layer.input_cloud_topic", "diagnostic/raw"},
    {"robot_base_frame", "shared_base"},
    {"local_obstacle_layer.base_frame", "diagnostic_base"},
  });
  auto node = std::make_shared<rclcpp::Node>("p2p_parameter_utils_legacy", options);

  EXPECT_EQ(
    p2p_move_base::getStringParameter(
      *node, "local_obstacle_layer.input_cloud_topic", "input_cloud_topic", "default"),
    "diagnostic/raw");
  EXPECT_EQ(
    p2p_move_base::getStringParameter(
      *node, "local_obstacle_layer.base_frame", "robot_base_frame", "default"),
    "diagnostic_base");
}

TEST_F(P2PParameterUtils, SharedFallbackIsUsedWhenLegacyValueIsAbsent)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"input_cloud_topic", "shared/raw"},
    {"robot_base_frame", "shared_base"},
  });
  auto node = std::make_shared<rclcpp::Node>("p2p_parameter_utils_shared", options);

  EXPECT_EQ(
    p2p_move_base::getStringParameter(
      *node, "local_obstacle_layer.input_cloud_topic", "input_cloud_topic", "default"),
    "shared/raw");
  EXPECT_EQ(
    p2p_move_base::getStringParameter(
      *node, "local_obstacle_layer.base_frame", "robot_base_frame", "default"),
    "shared_base");
}

TEST_F(P2PParameterUtils, RepeatedDeclarationsAreIdempotent)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"input_cloud_topic", "shared/raw"}});
  auto node = std::make_shared<rclcpp::Node>("p2p_parameter_utils_repeat", options);

  EXPECT_EQ(
    p2p_move_base::getStringParameter(
      *node, "missing.raw", "input_cloud_topic", "default"),
    "shared/raw");
  EXPECT_NO_THROW(
    EXPECT_EQ(
      p2p_move_base::getStringParameter(
        *node, "missing.raw", "input_cloud_topic", "default"),
      "shared/raw"));
}
}  // namespace
