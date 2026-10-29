#include <gtest/gtest.h>

#include <perception_3d/global_utils.h>
#include <perception_3d/shared_data.h>
#include <perception_3d/static_layer.h>

#include <chrono>
#include <memory>
#include <thread>

namespace
{
using namespace std::chrono_literals;

class StaticLayerParameters : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void Initialize(
    const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp::Node>("static_layer_parameter_test", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    utils_ = std::make_shared<perception_3d::GlobalUtils>(
      "map", "base_link", 9999.0, 0.52, 2.0, 0.9, tf_);
    data_ = std::make_shared<perception_3d::SharedData>();
    data_->mapping_mode_ = false;
    layer_.setSharedData(data_);
    layer_.initialize("map", node_, utils_);
    executor_.add_node(node_);
  }

  bool waitForSubscriber(const std::string & topic)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (node_->count_subscribers(topic) >= 1) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return node_->count_subscribers(topic) >= 1;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<perception_3d::GlobalUtils> utils_;
  std::shared_ptr<perception_3d::SharedData> data_;
  perception_3d::StaticLayer layer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(StaticLayerParameters, CommonTopicsAreUsedWhenLegacyPluginTopicsAreAbsent)
{
  Initialize({
    {"map_topic", "shared/map"},
    {"ground_topic", "shared/ground"},
  });

  EXPECT_TRUE(waitForSubscriber("shared/map"));
  EXPECT_TRUE(waitForSubscriber("shared/ground"));
  EXPECT_EQ(node_->count_subscribers("mapcloud"), 0u);
  EXPECT_EQ(node_->count_subscribers("mapground"), 0u);
}

TEST_F(StaticLayerParameters, LegacyPluginTopicsOverrideCommonTopics)
{
  Initialize({
    {"map_topic", "shared/map"},
    {"ground_topic", "shared/ground"},
    {"map.map_topic", "legacy/map"},
    {"map.ground_topic", "legacy/ground"},
  });

  EXPECT_TRUE(waitForSubscriber("legacy/map"));
  EXPECT_TRUE(waitForSubscriber("legacy/ground"));
  EXPECT_EQ(node_->count_subscribers("shared/map"), 0u);
  EXPECT_EQ(node_->count_subscribers("shared/ground"), 0u);
}

TEST_F(StaticLayerParameters, ExistingDefaultsRemainForOtherLaunches)
{
  Initialize({});

  EXPECT_TRUE(waitForSubscriber("mapcloud"));
  EXPECT_TRUE(waitForSubscriber("mapground"));
}

TEST_F(StaticLayerParameters, MultiplePluginsCanShareCommonTopicDeclarations)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"map_topic", "shared/map"},
    {"ground_topic", "shared/ground"},
  });
  node_ = std::make_shared<rclcpp::Node>("static_layer_duplicate_test", options);
  tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  utils_ = std::make_shared<perception_3d::GlobalUtils>(
    "map", "base_link", 9999.0, 0.52, 2.0, 0.9, tf_);
  data_ = std::make_shared<perception_3d::SharedData>();
  data_->mapping_mode_ = false;

  perception_3d::StaticLayer first;
  perception_3d::StaticLayer second;
  first.setSharedData(data_);
  second.setSharedData(data_);
  EXPECT_NO_THROW(first.initialize("map", node_, utils_));
  EXPECT_NO_THROW(second.initialize("second_map", node_, utils_));
  EXPECT_EQ(node_->count_subscribers("shared/map"), 2u);
  EXPECT_EQ(node_->count_subscribers("shared/ground"), 2u);
}
}  // namespace
