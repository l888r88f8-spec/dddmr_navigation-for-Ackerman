// Synthetic, deterministic local-map demo input.
//
// This process is not a sensor driver and does not consume vehicle data. It
// publishes a synthetic ground plane, a fixed box, and (by default) one moving
// box, plus the static TF chain odom <- demo_base_link <- demo_lidar. The demo
// base child frame is deliberately distinct from the real vehicle's base_link
// so starting this source in an existing robot domain does not replace or
// shadow vehicle TF. The moving box is a deterministic function of the node
// clock; this is not a prediction or a real obstacle-tracking claim. The
// synthetic box dimensions are scene fixtures only and are never loaded into
// vehicle geometry. The default 180 x 24 samples (4,320 rays) keep the
// unchanged 0.05 s diagnostics fresh at the 0.05 m profile, where the former
// 480 x 72 dense scan exceeded the age limit.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3& left, const Vec3& right) {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 operator-(const Vec3& left, const Vec3& right) {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 operator*(const Vec3& value, double scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

double norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y +
                   value.z * value.z);
}

struct Box {
  Vec3 center;
  Vec3 half_extent;
};

bool rayBoxEntry(const Vec3& origin, const Vec3& direction, const Box& box,
                 double max_distance, double* entry_distance) {
  if (entry_distance == nullptr) {
    return false;
  }
  const std::array<double, 3> origin_values{origin.x, origin.y, origin.z};
  const std::array<double, 3> direction_values{direction.x, direction.y,
                                               direction.z};
  const std::array<double, 3> centers{box.center.x, box.center.y, box.center.z};
  const std::array<double, 3> halves{box.half_extent.x, box.half_extent.y,
                                     box.half_extent.z};
  double near_distance = 0.0;
  double far_distance = max_distance;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const double lower = centers[axis] - halves[axis];
    const double upper = centers[axis] + halves[axis];
    if (std::abs(direction_values[axis]) < 1e-12) {
      if (origin_values[axis] < lower || origin_values[axis] > upper) {
        return false;
      }
      continue;
    }
    double first = (lower - origin_values[axis]) / direction_values[axis];
    double second = (upper - origin_values[axis]) / direction_values[axis];
    if (first > second) {
      std::swap(first, second);
    }
    near_distance = std::max(near_distance, first);
    far_distance = std::min(far_distance, second);
    if (near_distance > far_distance) {
      return false;
    }
  }
  if (near_distance <= 1e-6 || near_distance >= max_distance) {
    return false;
  }
  *entry_distance = near_distance;
  return true;
}

sensor_msgs::msg::PointCloud2 makePointCloud(
    const std::string& frame, const rclcpp::Time& stamp,
    const std::vector<Vec3>& points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  field.name = "x";
  field.offset = 0;
  cloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud.fields.push_back(field);
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  for (std::size_t index = 0; index < points.size(); ++index) {
    const float x = static_cast<float>(points[index].x);
    const float y = static_cast<float>(points[index].y);
    const float z = static_cast<float>(points[index].z);
    unsigned char* base = cloud.data.data() + index * cloud.point_step;
    std::memcpy(base, &x, sizeof(float));
    std::memcpy(base + 4, &y, sizeof(float));
    std::memcpy(base + 8, &z, sizeof(float));
  }
  return cloud;
}

class LocalMapDemoScene : public rclcpp::Node {
 public:
  LocalMapDemoScene() : Node("local_map_demo_scene") {
    cloud_topic_ = declare_parameter<std::string>(
        "cloud_topic", "/local_map_demo/points");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "demo_base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "demo_lidar");
    sensor_height_ = declare_parameter<double>("sensor_height", 1.4);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    azimuth_samples_ = declare_parameter<int>("azimuth_samples", 180);
    range_samples_ = declare_parameter<int>("range_samples", 24);
    min_ground_radius_ = declare_parameter<double>("min_ground_radius", 0.8);
    max_ground_radius_ = declare_parameter<double>("max_ground_radius", 7.5);
    moving_obstacle_ = declare_parameter<bool>("moving_obstacle", true);
    moving_obstacle_period_sec_ =
        declare_parameter<double>("moving_obstacle_period_sec", 8.0);

    validateParameters();

    tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishStaticTransforms();

    rclcpp::SensorDataQoS cloud_qos;
    cloud_qos.keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_,
                                                                 cloud_qos);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishCloud(); });
    publishCloud();
    RCLCPP_INFO(get_logger(),
                "synthetic local-map demo publishing %s in %s at %.1f Hz "
                "(ground + fixed box%s); this is not vehicle sensor data",
                cloud_topic_.c_str(), sensor_frame_.c_str(), publish_rate_hz_,
                moving_obstacle_ ? " + deterministic moving box" : "");
  }

 private:
  static bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
  }

  void validateParameters() const {
    if (cloud_topic_.empty() || cloud_topic_.front() != '/') {
      throw std::invalid_argument(
          "cloud_topic must be a non-empty absolute topic name");
    }
    if (map_frame_.empty() || base_frame_.empty() || sensor_frame_.empty()) {
      throw std::invalid_argument("map/base/sensor frames must not be empty");
    }
    if (map_frame_ == base_frame_ || map_frame_ == sensor_frame_ ||
        base_frame_ == sensor_frame_) {
      throw std::invalid_argument(
          "demo map/base/sensor frames must be distinct to avoid an ambiguous "
          "TF chain");
    }
    if (!finitePositive(sensor_height_) || sensor_height_ > 3.0) {
      throw std::invalid_argument(
          "sensor_height must be finite and in (0, 3.0] m");
    }
    if (!finitePositive(publish_rate_hz_) || publish_rate_hz_ > 30.0) {
      throw std::invalid_argument(
          "publish_rate_hz must be finite and in (0, 30] Hz");
    }
    if (!finitePositive(max_range_) || max_range_ < 2.0 || max_range_ > 30.0) {
      throw std::invalid_argument(
          "max_range must be finite and in [2.0, 30.0] m");
    }
    // Reject this before sqrt(max_range^2 - sensor_height^2): the subtraction
    // would be negative and std::min would silently mask the resulting NaN.
    if (sensor_height_ >= max_range_) {
      throw std::invalid_argument(
          "sensor_height must be less than max_range so the ground ring is "
          "real");
    }
    if (azimuth_samples_ < 90 || azimuth_samples_ > 2000) {
      throw std::invalid_argument("azimuth_samples must be in [90, 2000]");
    }
    if (range_samples_ < 20 || range_samples_ > 300) {
      throw std::invalid_argument("range_samples must be in [20, 300]");
    }
    if (!finitePositive(min_ground_radius_) ||
        !finitePositive(max_ground_radius_) ||
        min_ground_radius_ >= max_ground_radius_) {
      throw std::invalid_argument(
          "ground radii must be finite, positive, and min < max");
    }
    if (!finitePositive(moving_obstacle_period_sec_) ||
        moving_obstacle_period_sec_ < 1.0 ||
        moving_obstacle_period_sec_ > 60.0) {
      throw std::invalid_argument(
          "moving_obstacle_period_sec must be finite and in [1.0, 60.0] s");
    }
    const double visible_ground =
        std::sqrt(max_range_ * max_range_ - sensor_height_ * sensor_height_) *
        0.98;
    if (std::min(max_ground_radius_, visible_ground) <=
        min_ground_radius_ + 0.1) {
      throw std::invalid_argument(
          "max_range/sensor_height leave no usable ground ring; lower "
          "sensor_height or increase max_range");
    }
  }

  void publishStaticTransforms() {
    const rclcpp::Time stamp = now();
    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = stamp;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_frame_;
    map_to_base.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_base);

    geometry_msgs::msg::TransformStamped base_to_sensor;
    base_to_sensor.header.stamp = stamp;
    base_to_sensor.header.frame_id = base_frame_;
    base_to_sensor.child_frame_id = sensor_frame_;
    base_to_sensor.transform.translation.z = sensor_height_;
    base_to_sensor.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(base_to_sensor);
  }

  void publishCloud() {
    const rclcpp::Time stamp = now();
    const Vec3 sensor_origin{0.0, 0.0, sensor_height_};
    const double t = stamp.seconds();
    const double moving_y =
        moving_obstacle_
            ? 0.9 * std::sin(2.0 * kPi * t / moving_obstacle_period_sec_)
            : 0.0;
    const Box fixed_box{{2.0, 0.0, 0.5}, {0.5, 0.5, 0.5}};
    const Box moving_box{{-2.0, moving_y, 0.35}, {0.35, 0.35, 0.35}};

    const double visible_ground =
        std::sqrt(max_range_ * max_range_ - sensor_height_ * sensor_height_) *
        0.98;
    const double ground_radius =
        std::min(max_ground_radius_, visible_ground);
    std::vector<Vec3> points;
    points.reserve(static_cast<std::size_t>(azimuth_samples_) *
                   static_cast<std::size_t>(range_samples_));
    for (int azimuth_index = 0; azimuth_index < azimuth_samples_;
         ++azimuth_index) {
      const double azimuth =
          2.0 * kPi * static_cast<double>(azimuth_index) /
          static_cast<double>(azimuth_samples_);
      for (int range_index = 0; range_index < range_samples_; ++range_index) {
        const double fraction =
            static_cast<double>(range_index) /
            static_cast<double>(range_samples_ - 1);
        const double radius =
            min_ground_radius_ + (ground_radius - min_ground_radius_) * fraction;
        const Vec3 ground{sensor_origin.x + radius * std::cos(azimuth),
                          sensor_origin.y + radius * std::sin(azimuth),
                          0.0};
        const Vec3 delta = ground - sensor_origin;
        const double ground_distance = norm(delta);
        if (!(ground_distance > 1e-6)) {
          continue;
        }
        const Vec3 direction = delta * (1.0 / ground_distance);
        double best_distance = ground_distance;
        Vec3 hit = ground;

        double box_distance = 0.0;
        if (rayBoxEntry(sensor_origin, direction, fixed_box, best_distance,
                        &box_distance)) {
          best_distance = box_distance;
          hit = sensor_origin + direction * best_distance;
        }
        if (moving_obstacle_ &&
            rayBoxEntry(sensor_origin, direction, moving_box, best_distance,
                        &box_distance)) {
          best_distance = box_distance;
          hit = sensor_origin + direction * best_distance;
        }

        // The sensor orientation is identity in this demo, so the only map to
        // sensor transform is the height offset.
        points.push_back({hit.x, hit.y, hit.z - sensor_height_});
      }
    }

    publisher_->publish(makePointCloud(sensor_frame_, stamp, points));
    if (!logged_first_cloud_) {
      logged_first_cloud_ = true;
      RCLCPP_INFO(get_logger(), "first synthetic cloud: %zu points, frame=%s",
                  points.size(), sensor_frame_.c_str());
    }
  }

  std::string cloud_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  double sensor_height_{1.4};
  double publish_rate_hz_{5.0};
  double max_range_{8.0};
  int azimuth_samples_{180};
  int range_samples_{24};
  double min_ground_radius_{0.8};
  double max_ground_radius_{7.5};
  bool moving_obstacle_{true};
  double moving_obstacle_period_sec_{8.0};
  bool logged_first_cloud_{false};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LocalMapDemoScene>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("local_map_demo_scene"),
                 "failed to start synthetic demo: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
