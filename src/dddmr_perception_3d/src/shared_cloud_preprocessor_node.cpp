#include <perception_3d/cloud_preprocessor.h>
#include <perception_3d/parameter_utils.h>

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
    const auto input = getStringParameter(
      *this, "input_topic", "input_cloud_topic", "lio_sam/cloud_nonground");
    const auto output = getStringParameter(
      *this, "output_topic", "preprocessed_cloud_topic", "navigation/preprocessed_cloud");
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
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {accept(message);});
    // A scan whose acquisition-time TF is still in flight must be retried from
    // outside the sensor callback: with a timeout inside it, the TF listener
    // needs the same single-threaded executor that is blocked waiting, so the
    // wait always expires and the whole scan is lost.
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {deliverPending();});
    RCLCPP_INFO(get_logger(), "Shared cloud: %s -> %s, XY +/-%.1f m, voxel %.2f m",
      input.c_str(), output.c_str(), config_.window, config_.voxel_size);
  }

private:
  void accept(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    if (!message) {return;}
    const auto now = get_clock()->now();
    if (message->header.frame_id != sensor_frame_ ||
      !IsCloudStampFresh(message->header.stamp, now, max_age_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping nonground cloud with unexpected frame or stale/invalid acquisition stamp");
      return;
    }
    const rclcpp::Time stamp(message->header.stamp, now.get_clock_type());
    // A clock rewind (e.g. a bag restart) starts a new observation sequence.
    if (have_stamp_ && last_stamp_ <= now && stamp <= last_stamp_) {return;}
    // Keep the newest acceptable scan, replacing one that is still waiting
    // for its pose, and try to deliver it right away.
    pending_ = message;
    pending_stamp_ = stamp;
    have_stamp_ = true;
    last_stamp_ = stamp;
    deliverPending();
  }

  void deliverPending()
  {
    if (!pending_) {return;}
    const auto now = get_clock()->now();
    if (!IsCloudStampFresh(pending_->header.stamp, now, max_age_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping nonground cloud: acquisition stamp aged out before its pose arrived");
      pending_.reset();
      return;
    }
    Eigen::Affine3d sensor_to_body;
    Eigen::Affine3d body_to_global;
    try {
      // Never block here. A transform that the listener has not stored yet is
      // simply retried by the timer until the scan ages out or a newer one
      // arrives, which keeps this executor free to process /tf and /tf_static.
      const auto no_wait = rclcpp::Duration::from_seconds(0.0);
      sensor_to_body = tf2::transformToEigen(
        buffer_.lookupTransform(body_frame_, sensor_frame_, pending_stamp_, no_wait));
      body_to_global = tf2::transformToEigen(
        buffer_.lookupTransform(global_frame_, body_frame_, pending_stamp_, no_wait));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Shared cloud waiting for acquisition-time TF: %s", error.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*pending_, input);
    const auto cloud = PreprocessCloud(input, sensor_to_body, body_to_global, config_);
    if (!IsCloudStampFresh(pending_->header.stamp, get_clock()->now(), max_age_)) {
      pending_.reset();
      return;
    }
    publisher_->publish(MakePreprocessedCloudMessage(cloud, pending_->header, global_frame_));
    pending_.reset();
  }

  CloudPreprocessorConfig config_;
  std::string sensor_frame_, body_frame_, global_frame_;
  double max_age_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  bool have_stamp_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  rclcpp::Time pending_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};
}  // namespace perception_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_3d::SharedCloudPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
