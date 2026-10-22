#include <gtest/gtest.h>

#include <perception_3d/cloud_preprocessor.h>
#include <perception_3d/multilayer_spinning_lidar.h>

#include <chrono>
#include <thread>

namespace
{
using namespace std::chrono_literals;

class PreprocessedLidar : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void Initialize(bool local, bool preprocessed = true)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"lidar.topic", "/shared_cloud_test/input"},
      {"lidar.input_preprocessed", preprocessed},
      {"lidar.sensor_frame", local ? "" : "hesai_lidar"},
      {"lidar.is_local_planner", local}, {"lidar.expected_sensor_time", 0.5},
      {"lidar.perception_window_size", 6.0}, {"lidar.xy_resolution", 0.05},
      {"lidar.height_resolution", 0.05}, {"lidar.min_obstacle_height", -0.12},
      {"lidar.marking_height", 2.0}, {"lidar.vertical_FOV_top", 90.0},
      {"lidar.vertical_FOV_bottom", -90.0},
      {"lidar.scan_effective_positive_start", 0.0},
      {"lidar.scan_effective_positive_end", 180.0},
      {"lidar.scan_effective_negative_start", 0.0},
      {"lidar.scan_effective_negative_end", -180.0},
      {"lidar.segmentation_ignore_ratio", 1.1},
      {"lidar.euclidean_cluster_extraction_tolerance", 0.3}});
    node_ = std::make_shared<rclcpp::Node>("preprocessed_lidar_test", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 20.0;
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "test", true);
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "hesai_lidar";
    transform.transform.translation.x = 0.2;
    tf_->setTransform(transform, "test", true);
    const auto utils = std::make_shared<perception_3d::GlobalUtils>(
      "map", "base_link", 9999.0, 0.52, 2.0, 0.9, tf_);
    data_ = std::make_shared<perception_3d::SharedData>();
    data_->mapping_mode_ = false;
    lidar_.setSharedData(data_);
    lidar_.initialize("lidar", node_, utils);
    data_->dgraph_update_request_["lidar"] = false;
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/shared_cloud_test/input", 1);
    executor_.add_node(node_);
  }

  void Send(const pcl::PointCloud<pcl::PointXYZ> & cloud, const rclcpp::Time & stamp,
    const std::string & frame = "map")
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0u);
    std_msgs::msg::Header header;
    header.stamp = stamp;
    publisher_->publish(perception_3d::MakePreprocessedCloudMessage(cloud, header, frame));
    const auto end = std::chrono::steady_clock::now() + 80ms;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<perception_3d::SharedData> data_;
  perception_3d::MultiLayerSpinningLidar lidar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(PreprocessedLidar, RequiresActualObservationBeforeBecomingCurrent)
{
  Initialize(true);
  EXPECT_FALSE(lidar_.isCurrent());
}

TEST_F(PreprocessedLidar, LocalUsesGlobalCoordinatesWithoutCroppingThemAsBodyCoordinates)
{
  Initialize(true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({25.5f, 0.5f, 0.5f});
  Send(cloud, node_->now());
  ASSERT_TRUE(lidar_.isCurrent());
  const auto observation = lidar_.getObservation();
  ASSERT_EQ(observation->size(), 1u);
  EXPECT_FLOAT_EQ(observation->front().x, 25.5f);
  EXPECT_EQ(observation->header.frame_id, "map");
  Send({}, node_->now());
  EXPECT_TRUE(lidar_.getObservation()->empty());
  EXPECT_TRUE(lidar_.isCurrent());
}

TEST_F(PreprocessedLidar, DelayedCloudCannotMakeSensorCurrent)
{
  Initialize(true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({21.0f, 0.5f, 0.5f});
  Send(cloud, node_->now() - rclcpp::Duration::from_seconds(1.0));
  EXPECT_FALSE(lidar_.isCurrent());
  EXPECT_TRUE(lidar_.getObservation()->empty());
}

TEST_F(PreprocessedLidar, RejectsUnexpectedFrameRatherThanApplyingAnotherTransform)
{
  Initialize(true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({1.0f, 0.5f, 0.5f});
  Send(cloud, node_->now(), "base_link");
  EXPECT_FALSE(lidar_.isCurrent());
  EXPECT_TRUE(lidar_.getObservation()->empty());
}

TEST_F(PreprocessedLidar, DuplicateFrameCannotReplaceAcceptedObservation)
{
  Initialize(true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({21.0f, 0.5f, 0.5f});
  const auto stamp = node_->now();
  Send(cloud, stamp);
  Send({}, stamp);
  EXPECT_EQ(lidar_.getObservation()->size(), 1u);
}

TEST_F(PreprocessedLidar, FreshnessExpiresFromAcquisitionRatherThanReceipt)
{
  Initialize(true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({21.0f, 0.5f, 0.5f});
  Send(cloud, node_->now() - rclcpp::Duration::from_seconds(0.3));
  ASSERT_TRUE(lidar_.isCurrent());
  std::this_thread::sleep_for(230ms);
  EXPECT_FALSE(lidar_.isCurrent());
}

TEST_F(PreprocessedLidar, RawModeRetainsBodyFilteringAndGlobalObservation)
{
  Initialize(true, false);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({5.5f, 0.5f, 0.5f});
  cloud.push_back({7.0f, 0.5f, 0.5f});
  Send(cloud, node_->now(), "base_link");
  ASSERT_TRUE(lidar_.isCurrent());
  ASSERT_EQ(lidar_.getObservation()->size(), 1u);
  EXPECT_FLOAT_EQ(lidar_.getObservation()->front().x, 25.5f);
}

TEST_F(PreprocessedLidar, GlobalStillMarksAndClearsGroundGraphFromSharedCloud)
{
  Initialize(false);
  data_->pcl_ground_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int x = 0; x <= 20; ++x) {
    for (int y = 0; y <= 10; ++y) {
      pcl::PointXYZI point;
      point.x = 20.0f + x * 0.2f;
      point.y = y * 0.2f;
      point.z = 0.0f;
      data_->pcl_ground_->push_back(point);
    }
  }
  data_->pcl_map_ = data_->pcl_ground_;
  data_->kdtree_ground_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  data_->kdtree_ground_->setInputCloud(data_->pcl_ground_);
  data_->kdtree_map_ = data_->kdtree_ground_;
  data_->static_ground_size_ = data_->pcl_ground_->size();
  data_->is_static_layer_ready_ = true;
  lidar_.resetdGraph();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < 8; ++i) {cloud.push_back({21.0f + i * 0.12f, 0.5f, 0.5f});}
  Send(cloud, node_->now());
  lidar_.selfClear();
  lidar_.selfMark();
  // Index x=5,y=2 is (21,0.4,0), next to the projected obstacle.
  const unsigned int index = 5 * 11 + 2;
  EXPECT_LT(lidar_.get_dGraphValue(index), 0.52);
  Send({}, node_->now());
  lidar_.selfClear();
  lidar_.selfMark();
  EXPECT_GT(lidar_.get_dGraphValue(index), 1.0);
}
}  // namespace
