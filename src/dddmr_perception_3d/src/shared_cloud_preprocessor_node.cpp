#include <perception_3d/cloud_preprocessor.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace perception_3d
{

class SharedCloudPreprocessor : public rclcpp::Node
{
public:
  SharedCloudPreprocessor()
  : Node("shared_cloud_preprocessor"), buffer_(get_clock()), listener_(buffer_)
  {
    const auto input = declare_parameter<std::string>("input_topic", "lio_sam/cloud_nonground");
    const auto output = declare_parameter<std::string>("output_topic", "navigation/preprocessed_cloud");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "hesai_lidar");
    body_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    config_.window = declare_parameter("perception_window_size", 6.0);
    config_.min_height = declare_parameter("min_obstacle_height", -0.12);
    config_.max_height = declare_parameter("marking_height", 2.0);
    max_age_ = declare_parameter("expected_sensor_time", 0.5);
    if (sensor_frame_.empty() || body_frame_.empty() || global_frame_.empty() ||
      !std::isfinite(max_age_) || max_age_ <= 0.0)
    {
      throw std::invalid_argument("Invalid shared cloud frames or time limits");
    }
    // Validate filter parameters at startup, even before the first observation.
    PreprocessCloud({}, Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), config_);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, 1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {process(*message);});
    RCLCPP_INFO(get_logger(), "Shared cloud: %s -> %s, XY +/-%.1f m, voxel %.2f m",
      input.c_str(), output.c_str(), config_.window, config_.voxel_size);
  }

private:
  void process(const sensor_msgs::msg::PointCloud2 & message)
  {
    const auto now = get_clock()->now();
    if (message.header.frame_id != sensor_frame_ ||
      !IsCloudStampFresh(message.header.stamp, now, max_age_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping nonground cloud with unexpected frame or stale/invalid acquisition stamp");
      return;
    }
    const rclcpp::Time stamp(message.header.stamp, now.get_clock_type());
    // A clock rewind (e.g. a bag restart) starts a new observation sequence.
    if (have_stamp_ && last_stamp_ <= now && stamp <= last_stamp_) {return;}
    try {
      const auto timeout = rclcpp::Duration::from_seconds(0.05);
      const auto sensor_to_body = buffer_.lookupTransform(body_frame_, sensor_frame_, stamp, timeout);
      const auto body_to_global = buffer_.lookupTransform(global_frame_, body_frame_, stamp, timeout);
      pcl::PointCloud<pcl::PointXYZ> input;
      pcl::fromROSMsg(message, input);
      const auto cloud = PreprocessCloud(
        input, tf2::transformToEigen(sensor_to_body), tf2::transformToEigen(body_to_global), config_);
      if (!IsCloudStampFresh(message.header.stamp, get_clock()->now(), max_age_)) {return;}
      publisher_->publish(MakePreprocessedCloudMessage(cloud, message.header, global_frame_));
      last_stamp_ = stamp;
      have_stamp_ = true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Shared cloud waiting for acquisition-time TF: %s", error.what());
    }
  }

  CloudPreprocessorConfig config_;
  std::string sensor_frame_, body_frame_, global_frame_;
  double max_age_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  bool have_stamp_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace perception_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_3d::SharedCloudPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
