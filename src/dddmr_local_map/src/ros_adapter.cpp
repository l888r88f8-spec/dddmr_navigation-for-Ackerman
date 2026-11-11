#include "dddmr_local_map/ros_adapter.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.hpp>

namespace dddmr_local_map {
namespace ros {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool finiteScalar(double value) { return std::isfinite(value); }

bool finiteVec(const Eigen::Vector3d& value) { return value.allFinite(); }

// Inclusive segment/AABB overlap. Endpoints may lie outside the box; only the
// closed segment between them is tested. Used to keep valid rays that cross the
// local map window even when the measured return is outside it.
bool segmentIntersectsBox(const Eigen::Vector3d& start,
                          const Eigen::Vector3d& end,
                          const Eigen::Vector3d& lower,
                          const Eigen::Vector3d& upper) {
  double t0 = 0.0;
  double t1 = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double direction = end[axis] - start[axis];
    if (std::abs(direction) < 1e-15) {
      if (start[axis] < lower[axis] || start[axis] > upper[axis]) {
        return false;
      }
      continue;
    }
    double near_t = (lower[axis] - start[axis]) / direction;
    double far_t = (upper[axis] - start[axis]) / direction;
    if (near_t > far_t) {
      std::swap(near_t, far_t);
    }
    t0 = std::max(t0, near_t);
    t1 = std::min(t1, far_t);
    if (t0 > t1) {
      return false;
    }
  }
  return true;
}

double secondsOf(const rclcpp::Time& time) {
  return static_cast<double>(time.nanoseconds()) * 1e-9;
}

// Ingress age of the cloud in the adapter's ROS clock: the node-now used for
// this update minus the cloud header stamp. The independent steady-clock
// freshness checks handle paused simulation clocks.
double ingressAgeMs(const rclcpp::Time& now,
                    const rclcpp::Time& measurement_stamp) {
  const double age_ms =
      (secondsOf(now) - secondsOf(measurement_stamp)) * 1000.0;
  return std::isfinite(age_ms) ? age_ms : -1.0;
}

std::string joinNumbers(const char* name, double low, double high) {
  std::ostringstream stream;
  stream << name << " must be finite and in [" << low << ", " << high << "]";
  return stream.str();
}

constexpr bool kHostLittleEndian =
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && \
    (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    false;
#else
    true;
#endif

void swapBytes(unsigned char* data, std::size_t size) {
  for (std::size_t i = 0; i < size / 2; ++i) {
    std::swap(data[i], data[size - 1 - i]);
  }
}

template <typename T>
T readScalar(const unsigned char* source, bool big_endian_on_wire) {
  T value{};
  std::memcpy(&value, source, sizeof(T));
  if (big_endian_on_wire == kHostLittleEndian) {
    swapBytes(reinterpret_cast<unsigned char*>(&value), sizeof(T));
  }
  return value;
}

bool normalizeFrame(const std::string& input, std::string* output) {
  if (output == nullptr) {
    return false;
  }
  std::size_t first = 0;
  while (first < input.size() && input[first] == '/') {
    ++first;
  }
  *output = input.substr(first);
  return !output->empty();
}

Validation checkRange(const std::string& name, double value, double low,
                      double high) {
  if (!finiteScalar(value) || value < low || value > high) {
    return Validation::failure(joinNumbers(name.c_str(), low, high));
  }
  return Validation::success();
}

Validation readParameter(rclcpp::Node& node, const std::string& name,
                         const std::string& default_value, std::string* value) {
  try {
    if (node.has_parameter(name)) {
      *value = node.get_parameter(name).get_value<std::string>();
    } else {
      *value = node.declare_parameter<std::string>(name, default_value);
    }
  } catch (const std::exception& exception) {
    return Validation::failure("parameter '" + name + "': " + exception.what());
  }
  return Validation::success();
}

Validation readParameter(rclcpp::Node& node, const std::string& name,
                         double default_value, double* value) {
  try {
    if (node.has_parameter(name)) {
      *value = node.get_parameter(name).get_value<double>();
    } else {
      *value = node.declare_parameter<double>(name, default_value);
    }
  } catch (const std::exception& exception) {
    return Validation::failure("parameter '" + name + "': " + exception.what());
  }
  return Validation::success();
}

Validation readParameter(rclcpp::Node& node, const std::string& name,
                         int64_t default_value, int64_t* value) {
  try {
    if (node.has_parameter(name)) {
      *value = node.get_parameter(name).get_value<int64_t>();
    } else {
      *value = node.declare_parameter<int64_t>(name, default_value);
    }
  } catch (const std::exception& exception) {
    return Validation::failure("parameter '" + name + "': " + exception.what());
  }
  return Validation::success();
}

Validation readParameter(rclcpp::Node& node, const std::string& name,
                         bool default_value, bool* value) {
  try {
    if (node.has_parameter(name)) {
      *value = node.get_parameter(name).get_value<bool>();
    } else {
      *value = node.declare_parameter<bool>(name, default_value);
    }
  } catch (const std::exception& exception) {
    return Validation::failure("parameter '" + name + "': " + exception.what());
  }
  return Validation::success();
}

Validation readParameter(rclcpp::Node& node, const std::string& name,
                         const std::vector<double>& default_value,
                         std::vector<double>* value) {
  try {
    if (node.has_parameter(name)) {
      *value = node.get_parameter(name).get_value<std::vector<double>>();
    } else {
      *value =
          node.declare_parameter<std::vector<double>>(name, default_value);
    }
  } catch (const std::exception& exception) {
    return Validation::failure("parameter '" + name + "': " + exception.what());
  }
  return Validation::success();
}

Validation readDoubleParameter(rclcpp::Node& node, const std::string& name,
                               double default_value, double low, double high,
                               double* value) {
  Validation status = readParameter(node, name, default_value, value);
  if (!status.ok) {
    return status;
  }
  return checkRange(name, *value, low, high);
}

Validation readIntParameter(rclcpp::Node& node, const std::string& name,
                            int64_t default_value, int64_t low, int64_t high,
                            int64_t* value) {
  Validation status = readParameter(node, name, default_value, value);
  if (!status.ok) {
    return status;
  }
  if (*value < low || *value > high) {
    std::ostringstream stream;
    stream << name << " must be in [" << low << ", " << high << "]";
    return Validation::failure(stream.str());
  }
  return Validation::success();
}

bool allFinite(const std::vector<double>& values) {
  for (const double value : values) {
    if (!finiteScalar(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

const char* toString(StampRejection rejection) {
  switch (rejection) {
    case StampRejection::kNone:
      return "none";
    case StampRejection::kNonFinite:
      return "non_finite";
    case StampRejection::kZero:
      return "zero";
    case StampRejection::kFuture:
      return "future";
    case StampRejection::kStale:
      return "stale";
    case StampRejection::kNonMonotonic:
      return "non_monotonic";
  }
  return "invalid";
}

EigenTransform makeEigenTransform(const Eigen::Quaterniond& rotation,
                                  const Eigen::Vector3d& translation) {
  EigenTransform result;
  if (!rotation.coeffs().allFinite() || !translation.allFinite() ||
      rotation.norm() < 1e-9) {
    return result;
  }
  Eigen::Quaterniond normalized = rotation.normalized();
  result.rotation = normalized.toRotationMatrix();
  result.translation = translation;
  result.valid = result.rotation.allFinite() && result.translation.allFinite();
  return result;
}

EigenTransform makeEigenTransform(double qx, double qy, double qz, double qw,
                                  const Eigen::Vector3d& translation) {
  return makeEigenTransform(Eigen::Quaterniond(qw, qx, qy, qz), translation);
}

bool isFiniteTransform(const EigenTransform& transform) {
  if (!transform.valid || !transform.rotation.allFinite() ||
      !transform.translation.allFinite()) {
    return false;
  }
  const Eigen::Matrix3d orthogonality =
      transform.rotation.transpose() * transform.rotation -
      Eigen::Matrix3d::Identity();
  return orthogonality.norm() <= 1e-5 &&
         transform.rotation.determinant() > 0.99;
}

StampValidation validateAcquisitionStamp(double stamp_sec, double now_sec,
                                         double last_accepted_sec,
                                         double max_age_sec,
                                         double future_tolerance_sec) {
  StampValidation result;
  if (!finiteScalar(stamp_sec) || !finiteScalar(now_sec) ||
      !finiteScalar(last_accepted_sec) || !finiteScalar(max_age_sec) ||
      !finiteScalar(future_tolerance_sec) || max_age_sec < 0.0 ||
      future_tolerance_sec < 0.0) {
    result.rejection = StampRejection::kNonFinite;
    result.detail = "acquisition stamp inputs must be finite and non-negative";
    return result;
  }
  if (std::abs(stamp_sec) < 1e-12) {
    result.rejection = StampRejection::kZero;
    result.detail = "cloud acquisition stamp is zero";
    return result;
  }
  if (stamp_sec > now_sec + future_tolerance_sec) {
    result.rejection = StampRejection::kFuture;
    std::ostringstream stream;
    stream << "cloud acquisition stamp " << stamp_sec
           << " is in the future relative to now " << now_sec;
    result.detail = stream.str();
    return result;
  }
  if (now_sec - stamp_sec > max_age_sec) {
    result.rejection = StampRejection::kStale;
    std::ostringstream stream;
    stream << "cloud acquisition stamp is " << (now_sec - stamp_sec)
           << " s old (limit " << max_age_sec << " s)";
    result.detail = stream.str();
    return result;
  }
  if (last_accepted_sec > 0.0 && stamp_sec <= last_accepted_sec) {
    result.rejection = StampRejection::kNonMonotonic;
    std::ostringstream stream;
    stream << "cloud acquisition stamp " << stamp_sec
           << " is not newer than last accepted " << last_accepted_sec;
    result.detail = stream.str();
    return result;
  }
  result.ok = true;
  result.rejection = StampRejection::kNone;
  return result;
}

bool validateCloudFrame(const std::string& frame_id,
                        const std::string& expected_frame, std::string* error) {
  std::string actual;
  std::string expected;
  if (!normalizeFrame(frame_id, &actual) ||
      !normalizeFrame(expected_frame, &expected)) {
    if (error != nullptr) {
      *error = "cloud frame_id or configured sensor frame is empty";
    }
    return false;
  }
  if (actual != expected) {
    if (error != nullptr) {
      *error = "cloud frame '" + frame_id + "' does not match configured sensor "
               "frame '" +
               expected_frame + "'";
    }
    return false;
  }
  return true;
}

Validation pointCloudToMapPoints(
    const sensor_msgs::msg::PointCloud2& cloud,
    const EigenTransform& map_from_sensor, const EigenTransform& map_from_base,
    const Eigen::Vector3d& window_center,
    const Eigen::Vector3d& window_half_extent,
    const SelfFilterConfig& self_filter, std::vector<Eigen::Vector3d>* points,
    CloudConversionStats* stats, std::size_t max_points,
    WindowRayPolicy window_policy) {
  if (stats != nullptr) {
    *stats = CloudConversionStats{};
  }
  if (points == nullptr) {
    return Validation::failure("point output buffer is null");
  }
  points->clear();
  if (!isFiniteTransform(map_from_sensor)) {
    return Validation::failure(
        "supplied sensor-to-map transform is invalid or non-finite");
  }
  if (!isFiniteTransform(map_from_base)) {
    return Validation::failure(
        "supplied base-to-map transform is invalid or non-finite");
  }
  if (!finiteVec(window_center) || !finiteVec(window_half_extent) ||
      (window_half_extent.array() <= 0.0).any()) {
    return Validation::failure("map window center/extent is invalid");
  }
  if (self_filter.enabled) {
    if (!finiteVec(self_filter.body_min) || !finiteVec(self_filter.body_max) ||
        (self_filter.body_max.array() <= self_filter.body_min.array()).any()) {
      return Validation::failure(
          "enabled self-filter requires a finite rigid-body AABB with "
          "body_min < body_max on every axis");
    }
  }

  const std::uint64_t width = cloud.width;
  const std::uint64_t height = cloud.height;
  if (width == 0U || height == 0U) {
    return Validation::failure("point cloud is empty");
  }
  const std::uint64_t total = width * height;
  if (total > static_cast<std::uint64_t>(max_points)) {
    std::ostringstream stream;
    stream << "point cloud has " << total << " points, exceeding the safety "
           << "limit " << max_points;
    return Validation::failure(stream.str());
  }
  if (cloud.point_step == 0U) {
    return Validation::failure("point cloud point_step is zero");
  }
  if (height > 1U && cloud.row_step < width * cloud.point_step) {
    return Validation::failure("point cloud row_step is smaller than width * "
                               "point_step");
  }
  const std::uint64_t row_step =
      height > 1U ? cloud.row_step : width * cloud.point_step;
  const std::uint64_t required =
      (height - 1U) * row_step + width * cloud.point_step;
  if (cloud.data.size() < required) {
    std::ostringstream stream;
    stream << "point cloud data has " << cloud.data.size()
           << " bytes but " << required << " are required";
    return Validation::failure(stream.str());
  }

  const sensor_msgs::msg::PointField* field_x = nullptr;
  const sensor_msgs::msg::PointField* field_y = nullptr;
  const sensor_msgs::msg::PointField* field_z = nullptr;
  for (const auto& field : cloud.fields) {
    if (field.name == "x" && field_x == nullptr) {
      field_x = &field;
    } else if (field.name == "y" && field_y == nullptr) {
      field_y = &field;
    } else if (field.name == "z" && field_z == nullptr) {
      field_z = &field;
    }
  }
  const auto valid_field = [&cloud](const sensor_msgs::msg::PointField* field) {
    if (field == nullptr) {
      return false;
    }
    if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
        field->datatype != sensor_msgs::msg::PointField::FLOAT64) {
      return false;
    }
    const std::size_t width = field->datatype ==
                                      sensor_msgs::msg::PointField::FLOAT32
                                  ? sizeof(float)
                                  : sizeof(double);
    return static_cast<std::size_t>(field->offset) + width <=
           static_cast<std::size_t>(cloud.point_step);
  };
  if (!valid_field(field_x) || !valid_field(field_y) || !valid_field(field_z)) {
    return Validation::failure(
        "point cloud must contain finite float32/float64 x/y/z fields");
  }

  const auto read_field = [&cloud](const unsigned char* base,
                                   const sensor_msgs::msg::PointField& field) {
    const unsigned char* address =
        base + static_cast<std::size_t>(field.offset);
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<double>(readScalar<float>(address, cloud.is_bigendian));
    }
    return readScalar<double>(address, cloud.is_bigendian);
  };

  if (stats != nullptr) {
    stats->input_points = static_cast<std::size_t>(total);
  }
  points->reserve(std::min<std::size_t>(static_cast<std::size_t>(total),
                                        65536U));
  const Eigen::Vector3d window_lower = window_center - window_half_extent;
  const Eigen::Vector3d window_upper = window_center + window_half_extent;
  const Eigen::Vector3d sensor_origin = map_from_sensor.translation;
  const Eigen::Matrix3d base_rotation_inverse =
      map_from_base.rotation.transpose();

  for (std::uint64_t row = 0; row < height; ++row) {
    const unsigned char* row_base =
        cloud.data.data() + static_cast<std::size_t>(row * row_step);
    for (std::uint64_t column = 0; column < width; ++column) {
      const unsigned char* point_base =
          row_base + static_cast<std::size_t>(column * cloud.point_step);
      const double x = read_field(point_base, *field_x);
      const double y = read_field(point_base, *field_y);
      const double z = read_field(point_base, *field_z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (stats != nullptr) {
        ++stats->finite_points;
      }
      const Eigen::Vector3d sensor_point(x, y, z);
      const Eigen::Vector3d map_point =
          map_from_sensor.rotation * sensor_point + map_from_sensor.translation;
      if (!map_point.allFinite()) {
        continue;
      }
      const bool inside_window =
          (map_point.array() >= window_lower.array()).all() &&
          (map_point.array() <= window_upper.array()).all();
      if (inside_window) {
        if (stats != nullptr) {
          ++stats->cropped_points;
        }
      } else {
        if (stats != nullptr) {
          ++stats->outside_window_points;
        }
        if (window_policy == WindowRayPolicy::kDropOutsideEndpoints) {
          continue;
        }
        if (!segmentIntersectsBox(sensor_origin, map_point, window_lower,
                                  window_upper)) {
          continue;
        }
        if (stats != nullptr) {
          ++stats->retained_crossing_points;
        }
      }
      if (self_filter.enabled) {
        const Eigen::Vector3d base_point =
            base_rotation_inverse * (map_point - map_from_base.translation);
        const bool inside_body =
            base_point.allFinite() &&
            base_point.x() >= self_filter.body_min.x() &&
            base_point.x() <= self_filter.body_max.x() &&
            base_point.y() >= self_filter.body_min.y() &&
            base_point.y() <= self_filter.body_max.y() &&
            base_point.z() > self_filter.body_min.z() &&
            base_point.z() <= self_filter.body_max.z();
        if (inside_body) {
          if (stats != nullptr) {
            ++stats->self_removed;
          }
          continue;
        }
      }
      points->push_back(map_point);
    }
  }
  if (stats != nullptr) {
    stats->valid_points = points->size();
  }
  return Validation::success();
}

const char* toString(AdapterCode code) {
  switch (code) {
    case AdapterCode::kNotInitialized:
      return "not_initialized";
    case AdapterCode::kOk:
      return "ok";
    case AdapterCode::kInvalidConfig:
      return "invalid_config";
    case AdapterCode::kZeroStamp:
      return "zero_stamp";
    case AdapterCode::kNonFiniteStamp:
      return "non_finite_stamp";
    case AdapterCode::kFutureStamp:
      return "future_stamp";
    case AdapterCode::kStaleStamp:
      return "stale_stamp";
    case AdapterCode::kNonMonotonicStamp:
      return "non_monotonic_stamp";
    case AdapterCode::kWrongFrame:
      return "wrong_frame";
    case AdapterCode::kInvalidCloud:
      return "invalid_cloud";
    case AdapterCode::kNoValidRays:
      return "no_valid_rays";
    case AdapterCode::kTfFailure:
      return "tf_failure";
    case AdapterCode::kCoreRejected:
      return "core_rejected";
    case AdapterCode::kClockRewind:
      return "clock_rewind";
    case AdapterCode::kStaleReceipt:
      return "stale_receipt";
    case AdapterCode::kBuildFailure:
      return "build_failure";
  }
  return "invalid";
}

Validation validateAdapterConfig(const AdapterConfig& config) {
  Validation map_status = config.map.validate();
  if (!map_status.ok) {
    return Validation::failure("map: " + map_status.error);
  }
  Validation terrain_status = config.terrain.validate();
  if (!terrain_status.ok) {
    return Validation::failure("terrain: " + terrain_status.error);
  }
  if (config.cloud_topic.empty()) {
    return Validation::failure("cloud_topic must not be empty");
  }
  if (config.map_frame.empty() || config.base_frame.empty() ||
      config.sensor_frame.empty()) {
    return Validation::failure("map/base/sensor frames must not be empty");
  }
  Validation status = checkRange("tf_timeout_sec", config.tf_timeout_sec, 0.0,
                                 2.0);
  if (!status.ok) {
    return status;
  }
  status = checkRange("max_stamp_age_sec", config.max_stamp_age_sec, 0.01,
                      10.0);
  if (!status.ok) {
    return status;
  }
  status = checkRange("future_tolerance_sec", config.future_tolerance_sec, 0.0,
                      2.0);
  if (!status.ok) {
    return status;
  }
  status = checkRange("clock_rewind_tolerance_sec",
                      config.clock_rewind_tolerance_sec, 0.0, 2.0);
  if (!status.ok) {
    return status;
  }
  status = checkRange("status_log_period_sec", config.status_log_period_sec,
                      0.1, 60.0);
  if (!status.ok) {
    return status;
  }
  if (config.self_filter.enabled) {
    if (!config.self_filter.body_min.allFinite() ||
        !config.self_filter.body_max.allFinite() ||
        (config.self_filter.body_max.array() <=
         config.self_filter.body_min.array())
            .any()) {
      return Validation::failure(
          "enabled self_filter requires a finite rigid-body AABB with "
          "body_min < body_max on every axis");
    }
  }
  return Validation::success();
}

Validation loadAdapterConfig(rclcpp::Node& node, const std::string& prefix_in,
                             AdapterConfig* config) {
  if (config == nullptr) {
    return Validation::failure("output config is null");
  }
  std::string prefix = prefix_in;
  if (prefix.empty()) {
    prefix = "local_map.";
  }
  if (prefix.back() != '.') {
    prefix += '.';
  }
  const auto name = [&prefix](const char* suffix) {
    return prefix + suffix;
  };

  AdapterConfig result;
  result.parameter_prefix = prefix;

  double voxel_size = result.map.voxel_size;
  double size_x = 12.8;
  double size_y = 12.8;
  double size_z = 4.0;
  Validation status = readDoubleParameter(node, name("map.voxel_size"),
                                          result.map.voxel_size, 0.01, 1.0,
                                          &voxel_size);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.size_x"), size_x, 0.1, 100.0,
                               &size_x);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.size_y"), size_y, 0.1, 100.0,
                               &size_y);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.size_z"), size_z, 0.1, 100.0,
                               &size_z);
  if (!status.ok) {
    return status;
  }
  const auto window_dimension = [&name, voxel_size](
                                    const char* axis, double size,
                                    int* output) -> Validation {
    const double cells = size / voxel_size;
    const long long rounded = static_cast<long long>(std::llround(cells));
    if (rounded < 1 || rounded > 4096) {
      std::ostringstream stream;
      stream << name(axis) << " yields " << rounded
             << " voxels, outside [1, 4096]";
      return Validation::failure(stream.str());
    }
    if (std::abs(static_cast<double>(rounded) * voxel_size - size) >
        std::max(1e-9, voxel_size * 1e-6)) {
      std::ostringstream stream;
      stream << name(axis) << " (" << size
             << " m) must be an integer multiple of map.voxel_size ("
             << voxel_size << " m)";
      return Validation::failure(stream.str());
    }
    *output = static_cast<int>(rounded);
    return Validation::success();
  };
  result.map.voxel_size = voxel_size;
  status = window_dimension("map.size_x", size_x, &result.map.window_x);
  if (!status.ok) {
    return status;
  }
  status = window_dimension("map.size_y", size_y, &result.map.window_y);
  if (!status.ok) {
    return status;
  }
  status = window_dimension("map.size_z", size_z, &result.map.window_z);
  if (!status.ok) {
    return status;
  }

  int64_t max_voxels = static_cast<int64_t>(result.map.max_voxels);
  status = readIntParameter(node, name("map.max_voxels"), max_voxels, 1,
                            50'000'000, &max_voxels);
  if (!status.ok) {
    return status;
  }
  result.map.max_voxels = static_cast<std::size_t>(max_voxels);
  status = readDoubleParameter(node, name("map.max_range"), result.map.max_range,
                               0.1, 200.0, &result.map.max_range);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.inflation_radius"),
                               result.map.inflation_radius, 0.0, 10.0,
                               &result.map.inflation_radius);
  if (!status.ok) {
    return status;
  }
  double hit = result.map.hit_log_odds;
  double miss = result.map.miss_log_odds;
  double occupied = result.map.occupied_threshold;
  double free = result.map.free_threshold;
  double min_odds = result.map.min_log_odds;
  double max_odds = result.map.max_log_odds;
  status = readDoubleParameter(node, name("map.hit_log_odds"), hit, 1e-4, 20.0,
                               &hit);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.miss_log_odds"), miss, -20.0,
                               -1e-4, &miss);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.occupied_threshold"), occupied,
                               1e-4, 20.0, &occupied);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.free_threshold"), free, -20.0,
                               -1e-4, &free);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.min_log_odds"), min_odds, -20.0,
                               -1e-4, &min_odds);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("map.max_log_odds"), max_odds, 1e-4,
                               20.0, &max_odds);
  if (!status.ok) {
    return status;
  }
  result.map.hit_log_odds = static_cast<float>(hit);
  result.map.miss_log_odds = static_cast<float>(miss);
  result.map.occupied_threshold = static_cast<float>(occupied);
  result.map.free_threshold = static_cast<float>(free);
  result.map.min_log_odds = static_cast<float>(min_odds);
  result.map.max_log_odds = static_cast<float>(max_odds);

  TerrainConfig& terrain = result.terrain;
  status = readDoubleParameter(node, name("terrain.contact_patch_radius"),
                               terrain.contact_patch_radius, 0.01, 2.0,
                               &terrain.contact_patch_radius);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_contact_search_z"),
                               terrain.max_contact_search_z, 0.01, 3.0,
                               &terrain.max_contact_search_z);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_drop"), terrain.max_drop,
                               0.01, 5.0, &terrain.max_drop);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_step"), terrain.max_step,
                               0.01, 5.0, &terrain.max_step);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_slope_rad"),
                               terrain.max_slope_rad, 1e-4, 1.49,
                               &terrain.max_slope_rad);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_cross_slope_rad"),
                               terrain.max_cross_slope_rad, 1e-4, 1.49,
                               &terrain.max_cross_slope_rad);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_warp"), terrain.max_warp,
                               1e-4, 5.0, &terrain.max_warp);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.min_patch_coverage"),
                               terrain.min_patch_coverage, 0.0, 1.0,
                               &terrain.min_patch_coverage);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.min_plane_confidence"),
                               terrain.min_plane_confidence, 0.0, 1.0,
                               &terrain.min_plane_confidence);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_snapshot_age"),
                               terrain.max_snapshot_age, 0.01, 10.0,
                               &terrain.max_snapshot_age);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.max_observation_age"),
                               terrain.max_observation_age, 0.01, 10.0,
                               &terrain.max_observation_age);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.obstacle_search_margin"),
                               terrain.obstacle_search_margin, 0.0, 10.0,
                               &terrain.obstacle_search_margin);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.cluster_max_dz"),
                               terrain.cluster_max_dz, 1e-4, 2.0,
                               &terrain.cluster_max_dz);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("terrain.plane_inlier_tol"),
                               terrain.plane_inlier_tol, 1e-4, 1.0,
                               &terrain.plane_inlier_tol);
  if (!status.ok) {
    return status;
  }
  int64_t max_patches = static_cast<int64_t>(terrain.max_patches);
  status = readIntParameter(node, name("terrain.max_patches"), max_patches, 1,
                            1'000'000, &max_patches);
  if (!status.ok) {
    return status;
  }
  terrain.max_patches = static_cast<std::size_t>(max_patches);
  std::string unknown_policy = "reject";
  status = readParameter(node, name("terrain.unknown_body_policy"),
                         unknown_policy, &unknown_policy);
  if (!status.ok) {
    return status;
  }
  if (unknown_policy == "reject") {
    terrain.unknown_body_policy = UnknownBodyPolicy::kReject;
  } else if (unknown_policy == "allow") {
    terrain.unknown_body_policy = UnknownBodyPolicy::kAllow;
  } else {
    return Validation::failure(
        "terrain.unknown_body_policy must be exactly 'reject' or 'allow'");
  }

  status = readParameter(node, name("cloud_topic"), result.cloud_topic,
                         &result.cloud_topic);
  if (!status.ok) {
    return status;
  }
  status = readParameter(node, name("frames.map"), result.map_frame,
                         &result.map_frame);
  if (!status.ok) {
    return status;
  }
  status = readParameter(node, name("frames.base"), result.base_frame,
                         &result.base_frame);
  if (!status.ok) {
    return status;
  }
  status = readParameter(node, name("frames.sensor"), result.sensor_frame,
                         &result.sensor_frame);
  if (!status.ok) {
    return status;
  }

  status = readDoubleParameter(node, name("tf_timeout_sec"),
                               result.tf_timeout_sec, 0.0, 2.0,
                               &result.tf_timeout_sec);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("max_stamp_age_sec"),
                               result.max_stamp_age_sec, 0.01, 10.0,
                               &result.max_stamp_age_sec);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("future_tolerance_sec"),
                               result.future_tolerance_sec, 0.0, 2.0,
                               &result.future_tolerance_sec);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("clock_rewind_tolerance_sec"),
                               result.clock_rewind_tolerance_sec, 0.0, 2.0,
                               &result.clock_rewind_tolerance_sec);
  if (!status.ok) {
    return status;
  }
  status = readParameter(node, name("use_steady_receipt_age"),
                         result.use_steady_receipt_age,
                         &result.use_steady_receipt_age);
  if (!status.ok) {
    return status;
  }
  bool self_enabled = true;
  status = readParameter(node, name("self_filter.enabled"), true, &self_enabled);
  if (!status.ok) {
    return status;
  }
  status = readParameter(node, name("log_status"), result.log_status,
                         &result.log_status);
  if (!status.ok) {
    return status;
  }
  status = readDoubleParameter(node, name("status_log_period_sec"),
                               result.status_log_period_sec, 0.1, 60.0,
                               &result.status_log_period_sec);
  if (!status.ok) {
    return status;
  }

  // Geometry is optional. Structural mistakes are reported through
  // geometry_status but never stop the shadow map or throw; the controller
  // gates on control_ready.
  std::vector<double> contacts;
  status = readParameter(node, name("geometry.wheel_contacts"),
                         std::vector<double>{}, &contacts);
  if (!status.ok) {
    return status;
  }
  std::vector<double> body_min;
  status = readParameter(node, name("geometry.body_min"), std::vector<double>{},
                         &body_min);
  if (!status.ok) {
    return status;
  }
  std::vector<double> body_max;
  status = readParameter(node, name("geometry.body_max"), std::vector<double>{},
                         &body_max);
  if (!status.ok) {
    return status;
  }
  double base_contact_height = kNaN;
  status = readParameter(node, name("geometry.base_contact_height"),
                         base_contact_height, &base_contact_height);
  if (!status.ok) {
    return status;
  }
  double body_clearance = kNaN;
  status = readParameter(node, name("geometry.body_clearance"),
                         body_clearance, &body_clearance);
  if (!status.ok) {
    return status;
  }

  const bool geometry_any =
      !contacts.empty() || !body_min.empty() || !body_max.empty() ||
      finiteScalar(base_contact_height) || finiteScalar(body_clearance);
  Validation geometry_status =
      Validation::failure("vehicle geometry is not configured");
  if (geometry_any) {
    if (!contacts.empty() && contacts.size() != 12U) {
      geometry_status = Validation::failure(
          "geometry.wheel_contacts must contain exactly 12 numbers");
    } else if (!body_min.empty() && body_min.size() != 3U) {
      geometry_status = Validation::failure(
          "geometry.body_min must contain exactly 3 numbers");
    } else if (!body_max.empty() && body_max.size() != 3U) {
      geometry_status = Validation::failure(
          "geometry.body_max must contain exactly 3 numbers");
    } else if (!allFinite(contacts) || !allFinite(body_min) ||
               !allFinite(body_max) || !finiteScalar(base_contact_height) ||
               !finiteScalar(body_clearance)) {
      geometry_status = Validation::failure(
          "configured vehicle geometry values must be finite (fill every "
          "field, or leave geometry unset)");
    } else {
      if (contacts.size() == 12U) {
        for (std::size_t index = 0; index < 4U; ++index) {
          result.geometry.wheel_contacts_base[index] =
              Eigen::Vector3d(contacts[index * 3U + 0U],
                              contacts[index * 3U + 1U],
                              contacts[index * 3U + 2U]);
        }
      }
      if (body_min.size() == 3U) {
        result.geometry.body_min =
            Eigen::Vector3d(body_min[0], body_min[1], body_min[2]);
      }
      if (body_max.size() == 3U) {
        result.geometry.body_max =
            Eigen::Vector3d(body_max[0], body_max[1], body_max[2]);
      }
      result.geometry.base_contact_height = base_contact_height;
      result.geometry.body_clearance = body_clearance;
      geometry_status = result.geometry.validate();
    }
  }
  result.geometry_status = geometry_status;
  // Self filtering is never guessed: it is enabled only when the complete
  // rigid-body geometry validates, and then uses that exact AABB. With
  // geometry unset (the shadow default) it stays disabled. An explicit
  // self_filter.enabled=false also keeps it disabled with valid geometry.
  result.self_filter.enabled = self_enabled && geometry_status.ok;
  if (geometry_status.ok) {
    result.self_filter.body_min = result.geometry.body_min;
    result.self_filter.body_max = result.geometry.body_max;
  } else {
    result.self_filter.body_min = Eigen::Vector3d(kNaN, kNaN, kNaN);
    result.self_filter.body_max = Eigen::Vector3d(kNaN, kNaN, kNaN);
  }

  status = validateAdapterConfig(result);
  if (!status.ok) {
    return status;
  }
  *config = std::move(result);
  return Validation::success();
}

struct LocalMapRosAdapter::Impl {
  AdapterConfig config;
  rclcpp::Node* node{nullptr};
  tf2_ros::Buffer* tf_buffer{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
  bool started{false};

  mutable std::mutex update_mutex;
  mutable std::mutex bundle_mutex;
  mutable std::mutex log_mutex;

  std::unique_ptr<VoxelMap> map;
  std::shared_ptr<const AdapterHandle> current;
  AdapterStatus last_status;
  std::atomic<uint64_t> version_counter{0};
  std::atomic<uint64_t> generation{1};

  bool has_last_stamp{false};
  double last_stamp_sec{0.0};
  bool has_last_ros_now{false};
  double last_ros_now_sec{0.0};
  bool has_last_receipt{false};
  std::chrono::steady_clock::time_point last_receipt_steady{};
  bool has_last_log{false};
  std::chrono::steady_clock::time_point last_log_steady{};

  // Steady-clock splits for one ingest call. A fresh instance is created per
  // update and passed by pointer only along that same call path; the throttled
  // status log therefore cannot report a previous update's phases. Phases that
  // did not run keep the explicit 0.0 default.
  struct UpdateTiming {
    double lock_wait_ms{0.0};
    double tf_total_ms{0.0};
    double convert_ms{0.0};
    double recenter_ms{0.0};
    double integrate_ms{0.0};
    double snapshot_ms{0.0};
    double terrain_ms{0.0};
    std::size_t input_points{0};
    std::size_t converted_rays{0};
    std::size_t valid_rays{0};
  };

  static double elapsedMs(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start)
        .count();
  }

  static void appendStageFields(std::ostringstream* stream,
                                const UpdateTiming* timing) {
    if (timing == nullptr) {
      // A synthesized stale-receipt handle has no stage timings for this
      // handle's original accepted update; report them unavailable instead of
      // borrowing another frame's numbers.
      *stream << "lock_wait_ms=n/a tf_total_ms=n/a convert_ms=n/a "
                 "recenter_ms=n/a integrate_ms=n/a snapshot_ms=n/a "
                 "terrain_ms=n/a input_points=n/a converted_rays=n/a "
                 "valid_rays=n/a";
      return;
    }
    *stream << "lock_wait_ms=" << timing->lock_wait_ms
            << " tf_total_ms=" << timing->tf_total_ms
            << " convert_ms=" << timing->convert_ms
            << " recenter_ms=" << timing->recenter_ms
            << " integrate_ms=" << timing->integrate_ms
            << " snapshot_ms=" << timing->snapshot_ms
            << " terrain_ms=" << timing->terrain_ms
            << " input_points=" << timing->input_points
            << " converted_rays=" << timing->converted_rays
            << " valid_rays=" << timing->valid_rays;
  }

  void logStatus(const AdapterStatus& status, double mapping_ms,
                 double ingress_age_ms,
                 const UpdateTiming* timing = nullptr) {
    if (node == nullptr || !config.log_status) {
      return;
    }
    std::lock_guard<std::mutex> lock(log_mutex);
    const auto now = std::chrono::steady_clock::now();
    if (has_last_log &&
        now - last_log_steady <
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(config.status_log_period_sec))) {
      return;
    }
    has_last_log = true;
    last_log_steady = now;
    std::ostringstream stage_stream;
    appendStageFields(&stage_stream, timing);
    const std::string stages = stage_stream.str();
    // mapping_ms spans ingest entry to this status, so it includes callback
    // lock wait, TF lookup wait, core integration, snapshot and terrain build.
    // ingress_age_ms is node-ROS-clock (update now - cloud header stamp), so
    // upstream input age is not confused with local mapping cost. The stage
    // fields come from this same update; unexecuted phases are 0.0 and
    // unavailable stages are n/a.
    if (status.ok() && status.control_ready) {
      RCLCPP_INFO(node->get_logger(),
                  "local_map healthy generation=%llu mapping_ms=%.1f "
                  "ingress_age_ms=%.1f %s",
                  static_cast<unsigned long long>(generation.load()),
                  mapping_ms, ingress_age_ms, stages.c_str());
    } else if (status.ok()) {
      RCLCPP_WARN(node->get_logger(),
                  "local_map map healthy but control not ready: %s "
                  "mapping_ms=%.1f ingress_age_ms=%.1f %s",
                  status.reason.c_str(), mapping_ms, ingress_age_ms,
                  stages.c_str());
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "local_map %s: %s mapping_ms=%.1f ingress_age_ms=%.1f %s",
                  toString(status.code), status.reason.c_str(), mapping_ms,
                  ingress_age_ms, stages.c_str());
    }
  }

  std::shared_ptr<AdapterHandle> makeFailureHandle(
      AdapterCode code, const std::string& reason, double mapping_ms,
      bool preserve_data) {
    auto handle = std::make_shared<AdapterHandle>();
    if (preserve_data) {
      std::shared_ptr<const AdapterHandle> previous;
      {
        std::lock_guard<std::mutex> lock(bundle_mutex);
        previous = current;
      }
      if (previous) {
        handle->snapshot = previous->snapshot;
        handle->terrain = previous->terrain;
        handle->measurement_stamp = previous->measurement_stamp;
        handle->receive_stamp = previous->receive_stamp;
        handle->receive_steady = previous->receive_steady;
      }
    }
    handle->version = version_counter.fetch_add(1) + 1;
    handle->generation = generation.load();
    handle->mapping_ms = mapping_ms;
    handle->status.code = code;
    handle->status.map_healthy = false;
    handle->status.control_ready = false;
    handle->status.reason = reason;
    return handle;
  }

  void publishFailure(AdapterCode code, const std::string& reason,
                      double mapping_ms, double ingress_age_ms,
                      bool preserve_data = true,
                      const UpdateTiming* timing = nullptr) {
    auto handle = makeFailureHandle(code, reason, mapping_ms, preserve_data);
    {
      std::lock_guard<std::mutex> lock(bundle_mutex);
      current = handle;
      last_status = handle->status;
    }
    logStatus(handle->status, mapping_ms, ingress_age_ms, timing);
  }

  void resetMap(const std::string& reason, const rclcpp::Time& now,
                const rclcpp::Time& measurement_stamp,
                std::chrono::steady_clock::time_point received,
                std::chrono::steady_clock::time_point mapping_start,
                const UpdateTiming* timing) {
    generation.fetch_add(1);
    map = std::make_unique<VoxelMap>(config.map, config.map_frame);
    has_last_stamp = false;
    last_stamp_sec = 0.0;
    has_last_receipt = false;
    last_receipt_steady = std::chrono::steady_clock::time_point{};
    has_last_ros_now = true;
    last_ros_now_sec = secondsOf(now);
    const double mapping_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - mapping_start)
            .count();
    publishFailure(AdapterCode::kClockRewind, reason, mapping_ms,
                   ingressAgeMs(now, measurement_stamp), false, timing);
    (void)received;
  }

  bool prepareLocked(const sensor_msgs::msg::PointCloud2& cloud,
                     const rclcpp::Time& measurement_stamp,
                     const rclcpp::Time& now,
                     std::chrono::steady_clock::time_point received,
                     std::chrono::steady_clock::time_point mapping_start,
                     const UpdateTiming* timing, std::string* error) {
    const double now_sec = secondsOf(now);
    const double ingress_age_ms = ingressAgeMs(now, measurement_stamp);
    if (has_last_ros_now &&
        now_sec + config.clock_rewind_tolerance_sec < last_ros_now_sec) {
      std::ostringstream stream;
      stream << "ROS clock rewound from " << last_ros_now_sec << " to "
             << now_sec << "; map cleared for the new history";
      resetMap(stream.str(), now, measurement_stamp, received, mapping_start,
               timing);
    }
    if (!has_last_ros_now || now_sec > last_ros_now_sec) {
      last_ros_now_sec = now_sec;
      has_last_ros_now = true;
    }

    const double stamp_sec = secondsOf(measurement_stamp);
    const StampValidation stamp_status = validateAcquisitionStamp(
        stamp_sec, now_sec, has_last_stamp ? last_stamp_sec : 0.0,
        config.max_stamp_age_sec, config.future_tolerance_sec);
    if (!stamp_status.ok) {
      AdapterCode code = AdapterCode::kNonFiniteStamp;
      switch (stamp_status.rejection) {
        case StampRejection::kZero:
          code = AdapterCode::kZeroStamp;
          break;
        case StampRejection::kFuture:
          code = AdapterCode::kFutureStamp;
          break;
        case StampRejection::kStale:
          code = AdapterCode::kStaleStamp;
          break;
        case StampRejection::kNonMonotonic:
          code = AdapterCode::kNonMonotonicStamp;
          break;
        case StampRejection::kNonFinite:
        case StampRejection::kNone:
          code = AdapterCode::kNonFiniteStamp;
          break;
      }
      const double mapping_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - mapping_start)
              .count();
      publishFailure(code, stamp_status.detail, mapping_ms, ingress_age_ms,
                     true, timing);
      if (error != nullptr) {
        *error = stamp_status.detail;
      }
      return false;
    }

    std::string frame_error;
    if (!validateCloudFrame(cloud.header.frame_id, config.sensor_frame,
                            &frame_error)) {
      const double mapping_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - mapping_start)
              .count();
      publishFailure(AdapterCode::kWrongFrame, frame_error, mapping_ms,
                     ingress_age_ms, true, timing);
      if (error != nullptr) {
        *error = frame_error;
      }
      return false;
    }
    return true;
  }

  bool processLocked(const sensor_msgs::msg::PointCloud2& cloud,
                     const EigenTransform& map_from_sensor,
                     const EigenTransform& map_from_base,
                     const rclcpp::Time& measurement_stamp,
                     const rclcpp::Time& now,
                     std::chrono::steady_clock::time_point received,
                     std::chrono::steady_clock::time_point mapping_start,
                     UpdateTiming* timing, std::string* error) {
    const double stamp_sec = secondsOf(measurement_stamp);
    const double now_sec = secondsOf(now);
    const double ingress_age_ms = ingressAgeMs(now, measurement_stamp);
    const auto fail = [&](AdapterCode code, const std::string& reason) {
      const double mapping_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - mapping_start)
              .count();
      publishFailure(code, reason, mapping_ms, ingress_age_ms, true, timing);
      if (error != nullptr) {
        *error = reason;
      }
      return false;
    };

    if (!isFiniteTransform(map_from_sensor)) {
      return fail(AdapterCode::kTfFailure,
                  "sensor-to-map transform is invalid/non-finite/zero");
    }
    if (!isFiniteTransform(map_from_base)) {
      return fail(AdapterCode::kTfFailure,
                  "base-to-map transform is invalid/non-finite/zero");
    }

    const double voxel = map->config().voxel_size;
    const Eigen::Vector3d window_center = map_from_base.translation;
    const Eigen::Vector3d half_extent(
        0.5 * static_cast<double>(map->config().window_x) * voxel,
        0.5 * static_cast<double>(map->config().window_y) * voxel,
        0.5 * static_cast<double>(map->config().window_z) * voxel);
    if (!window_center.allFinite()) {
      return fail(AdapterCode::kTfFailure,
                  "base position from TF is non-finite");
    }

    std::vector<Eigen::Vector3d> points;
    CloudConversionStats stats;
    const auto convert_start = std::chrono::steady_clock::now();
    const Validation conversion = pointCloudToMapPoints(
        cloud, map_from_sensor, map_from_base, window_center, half_extent,
        config.self_filter, &points, &stats, 5'000'000U,
        WindowRayPolicy::kKeepCrossingRays);
    if (timing != nullptr) {
      timing->convert_ms = elapsedMs(convert_start);
      timing->input_points = stats.input_points;
      timing->converted_rays = points.size();
    }
    if (!conversion.ok) {
      return fail(AdapterCode::kInvalidCloud, conversion.error);
    }
    if (points.empty()) {
      std::ostringstream stream;
      stream << "no valid rays after filtering (input=" << stats.input_points
             << ", finite=" << stats.finite_points
             << ", in_window=" << stats.cropped_points
             << ", self=" << stats.self_removed << ")";
      return fail(AdapterCode::kNoValidRays, stream.str());
    }

    const Eigen::Vector3d sensor_origin = map_from_sensor.translation;
    if (!sensor_origin.allFinite()) {
      return fail(AdapterCode::kTfFailure, "sensor origin from TF is non-finite");
    }

    std::string core_error;
    std::chrono::steady_clock::time_point integrate_start{};
    try {
      const auto recenter_start = std::chrono::steady_clock::now();
      const bool recentered = map->setWindowCenter(window_center, &core_error);
      if (timing != nullptr) {
        timing->recenter_ms = elapsedMs(recenter_start);
      }
      if (!recentered) {
        return fail(AdapterCode::kCoreRejected,
                    "core setWindowCenter failed: " + core_error);
      }
      integrate_start = std::chrono::steady_clock::now();
      const MapUpdateResult update =
          map->integrateRays(sensor_origin, points, stamp_sec);
      if (timing != nullptr) {
        timing->integrate_ms = elapsedMs(integrate_start);
        timing->converted_rays = points.size();
        timing->valid_rays = update.stats.valid_rays;
      }
      if (!update.ok) {
        return fail(AdapterCode::kCoreRejected,
                    "core integrateRays failed: " + update.error);
      }
      if (update.stats.valid_rays == 0U) {
        return fail(AdapterCode::kNoValidRays,
                    "core reported success with zero valid rays");
      }
    } catch (const std::exception& exception) {
      if (timing != nullptr &&
          integrate_start != std::chrono::steady_clock::time_point{}) {
        timing->integrate_ms = elapsedMs(integrate_start);
      }
      return fail(AdapterCode::kCoreRejected,
                  std::string("core mapping exception: ") + exception.what());
    }

    // Expensive work is done. Re-check both ROS-stamp age and steady receipt
    // age before publishing the bundle. A frozen /sim clock cannot hide
    // sensor loss because steady time advances independently.
    const rclcpp::Time now_after = node != nullptr ? node->now() : now;
    const double now_after_sec = secondsOf(now_after);
    if (now_after_sec + config.clock_rewind_tolerance_sec < now_sec) {
      std::ostringstream stream;
      stream << "ROS clock rewound during mapping from " << now_sec << " to "
             << now_after_sec << "; discarding this update";
      resetMap(stream.str(), now_after, measurement_stamp, received,
               mapping_start, timing);
      if (error != nullptr) {
        *error = stream.str();
      }
      return false;
    }
    const StampValidation after_status = validateAcquisitionStamp(
        stamp_sec, now_after_sec, has_last_stamp ? last_stamp_sec : 0.0,
        config.max_stamp_age_sec, config.future_tolerance_sec);
    if (!after_status.ok) {
      AdapterCode code = AdapterCode::kStaleStamp;
      if (after_status.rejection == StampRejection::kFuture) {
        code = AdapterCode::kFutureStamp;
      } else if (after_status.rejection == StampRejection::kNonMonotonic) {
        code = AdapterCode::kNonMonotonicStamp;
      }
      return fail(code, "post-update " + after_status.detail);
    }
    if (config.use_steady_receipt_age) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (steady_now - received >
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(config.max_stamp_age_sec))) {
        return fail(AdapterCode::kStaleReceipt,
                    "steady-clock receipt age exceeded max_stamp_age_sec "
                    "after mapping");
      }
    }

    std::shared_ptr<const MapSnapshot> snapshot;
    std::shared_ptr<const TerrainModel> terrain;
    try {
      const auto snapshot_start = std::chrono::steady_clock::now();
      snapshot = map->snapshot();
      if (timing != nullptr) {
        timing->snapshot_ms = elapsedMs(snapshot_start);
      }
      if (!snapshot) {
        return fail(AdapterCode::kBuildFailure,
                    "core did not produce a post-integration snapshot");
      }
      const auto terrain_start = std::chrono::steady_clock::now();
      terrain = std::make_shared<TerrainModel>(snapshot, config.terrain,
                                               config.geometry);
      if (timing != nullptr) {
        timing->terrain_ms = elapsedMs(terrain_start);
      }
      if (!terrain) {
        return fail(AdapterCode::kBuildFailure, "terrain cache is null");
      }
    } catch (const std::exception& exception) {
      return fail(AdapterCode::kBuildFailure,
                  std::string("terrain build exception: ") + exception.what());
    }

    // Terrain construction is part of the expensive update. Re-check the same
    // freshness rules AFTER it so a frozen /clock plus a slow terrain build
    // cannot publish a bundle whose receipt is already stale.
    const rclcpp::Time now_final = node != nullptr ? node->now() : now;
    const double now_final_sec = secondsOf(now_final);
    if (now_final_sec + config.clock_rewind_tolerance_sec < now_after_sec) {
      std::ostringstream stream;
      stream << "ROS clock rewound during terrain build from " << now_after_sec
             << " to " << now_final_sec << "; discarding this update";
      resetMap(stream.str(), now_final, measurement_stamp, received,
               mapping_start, timing);
      if (error != nullptr) {
        *error = stream.str();
      }
      return false;
    }
    const StampValidation final_status = validateAcquisitionStamp(
        stamp_sec, now_final_sec, has_last_stamp ? last_stamp_sec : 0.0,
        config.max_stamp_age_sec, config.future_tolerance_sec);
    if (!final_status.ok) {
      AdapterCode code = AdapterCode::kStaleStamp;
      if (final_status.rejection == StampRejection::kFuture) {
        code = AdapterCode::kFutureStamp;
      } else if (final_status.rejection == StampRejection::kNonMonotonic) {
        code = AdapterCode::kNonMonotonicStamp;
      }
      return fail(code, "post-terrain " + final_status.detail);
    }
    if (config.use_steady_receipt_age) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (steady_now - received >
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(config.max_stamp_age_sec))) {
        return fail(AdapterCode::kStaleReceipt,
                    "steady-clock receipt age exceeded max_stamp_age_sec "
                    "after terrain build");
      }
    }

    const double mapping_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - mapping_start)
            .count();

    AdapterStatus status;
    status.code = AdapterCode::kOk;
    status.map_healthy = true;
    const Validation terrain_status = terrain->buildStatus();
    if (terrain_status.ok && config.geometry_status.ok) {
      status.control_ready = true;
      status.reason = "ok";
    } else {
      status.control_ready = false;
      status.reason = "map healthy; control not ready: ";
      if (!config.geometry_status.ok) {
        status.reason += config.geometry_status.error;
      } else {
        status.reason += terrain_status.error;
      }
    }

    auto handle = std::make_shared<AdapterHandle>();
    handle->snapshot = snapshot;
    handle->terrain = terrain;
    handle->measurement_stamp = measurement_stamp;
    handle->receive_stamp = now;
    handle->receive_steady = received;
    handle->version = version_counter.fetch_add(1) + 1;
    handle->generation = generation.load();
    handle->mapping_ms = mapping_ms;
    handle->status = status;
    {
      std::lock_guard<std::mutex> lock(bundle_mutex);
      current = handle;
      last_status = status;
    }
    has_last_stamp = true;
    last_stamp_sec = stamp_sec;
    has_last_receipt = true;
    last_receipt_steady = received;
    logStatus(status, mapping_ms, ingress_age_ms, timing);
    return true;
  }

  bool ingestCloudWithTransformsLocked(
      const sensor_msgs::msg::PointCloud2& cloud,
      const EigenTransform& map_from_sensor,
      const EigenTransform& map_from_base,
      const rclcpp::Time& measurement_stamp, const rclcpp::Time& now,
      std::chrono::steady_clock::time_point received, std::string* error) {
    const auto mapping_start = std::chrono::steady_clock::now();
    UpdateTiming timing;
    if (!prepareLocked(cloud, measurement_stamp, now, received, mapping_start,
                       &timing, error)) {
      return false;
    }
    return processLocked(cloud, map_from_sensor, map_from_base,
                         measurement_stamp, now, received, mapping_start,
                         &timing, error);
  }
};

LocalMapRosAdapter::LocalMapRosAdapter(AdapterConfig config)
    : impl_(std::make_unique<Impl>()) {
  Validation status = validateAdapterConfig(config);
  if (!status.ok) {
    throw std::invalid_argument("invalid local-map adapter config: " +
                                status.error);
  }
  impl_->config = std::move(config);
  const Validation geometry_status = impl_->config.geometry.validate();
  if (geometry_status.ok) {
    impl_->config.geometry_status = Validation::success();
  } else if (impl_->config.geometry_status.ok) {
    impl_->config.geometry_status = geometry_status;
  }
  // Programmatic callers that provide valid geometry but leave the self-filter
  // AABB unset get the filter derived from that exact geometry. An explicitly
  // configured AABB with enabled=false stays disabled.
  if (impl_->config.geometry_status.ok &&
      !impl_->config.self_filter.enabled &&
      (!impl_->config.self_filter.body_min.allFinite() ||
       !impl_->config.self_filter.body_max.allFinite())) {
    impl_->config.self_filter.enabled = true;
    impl_->config.self_filter.body_min = impl_->config.geometry.body_min;
    impl_->config.self_filter.body_max = impl_->config.geometry.body_max;
  }
  impl_->map =
      std::make_unique<VoxelMap>(impl_->config.map, impl_->config.map_frame);
  impl_->last_status.code = AdapterCode::kNotInitialized;
  impl_->last_status.reason = "no cloud has been processed";
}

LocalMapRosAdapter::LocalMapRosAdapter(rclcpp::Node& node,
                                       tf2_ros::Buffer& tf_buffer,
                                       AdapterConfig config)
    : LocalMapRosAdapter(std::move(config)) {
  impl_->node = &node;
  impl_->tf_buffer = &tf_buffer;
  const Validation status = start();
  if (!status.ok) {
    throw std::runtime_error("failed to start local-map adapter: " +
                             status.error);
  }
  if (!impl_->config.geometry_status.ok) {
    RCLCPP_WARN(node.get_logger(),
                "local_map mapping enabled without control-ready "
                "geometry: %s",
                impl_->config.geometry_status.error.c_str());
  }
}

LocalMapRosAdapter::~LocalMapRosAdapter() { stop(); }

Validation LocalMapRosAdapter::start() {
  if (impl_ == nullptr) {
    return Validation::failure("adapter implementation is missing");
  }
  if (impl_->started) {
    return Validation::success();
  }
  if (impl_->node == nullptr) {
    return Validation::failure(
        "offline adapter has no node; use ingestCloudWithTransforms()");
  }
  try {
    impl_->callback_group = impl_->node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = impl_->callback_group;
    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    impl_->subscription =
        impl_->node->create_subscription<sensor_msgs::msg::PointCloud2>(
            impl_->config.cloud_topic, qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
              if (message == nullptr) {
                return;
              }
              const auto now = impl_->node->now();
              const auto received = std::chrono::steady_clock::now();
              std::string error;
              (void)ingestCloud(*message, now, received, &error);
            },
            options);
    impl_->started = true;
    RCLCPP_INFO(impl_->node->get_logger(),
                "local_map subscribed to %s (sensor=%s map=%s base=%s)",
                impl_->config.cloud_topic.c_str(),
                impl_->config.sensor_frame.c_str(),
                impl_->config.map_frame.c_str(),
                impl_->config.base_frame.c_str());
  } catch (const std::exception& exception) {
    impl_->subscription.reset();
    impl_->callback_group.reset();
    return Validation::failure(std::string("subscription creation failed: ") +
                               exception.what());
  }
  return Validation::success();
}

void LocalMapRosAdapter::stop() {
  if (impl_ == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->update_mutex);
  impl_->subscription.reset();
  impl_->callback_group.reset();
  impl_->started = false;
}

bool LocalMapRosAdapter::started() const {
  return impl_ != nullptr && impl_->started;
}

std::shared_ptr<const AdapterHandle> LocalMapRosAdapter::handle() const {
  return handleAt(std::chrono::steady_clock::now());
}

std::shared_ptr<const AdapterHandle> LocalMapRosAdapter::handleAt(
    std::chrono::steady_clock::time_point steady_now) const {
  if (impl_ == nullptr) {
    return nullptr;
  }
  std::shared_ptr<const AdapterHandle> handle;
  {
    std::lock_guard<std::mutex> lock(impl_->bundle_mutex);
    handle = impl_->current;
  }
  if (handle == nullptr || !handle->status.map_healthy ||
      !impl_->config.use_steady_receipt_age ||
      handle->receive_steady == std::chrono::steady_clock::time_point{}) {
    return handle;
  }
  const auto max_age =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(impl_->config.max_stamp_age_sec));
  if (steady_now - handle->receive_steady <= max_age) {
    return handle;
  }

  std::shared_ptr<const AdapterHandle> stale;
  {
    std::lock_guard<std::mutex> lock(impl_->bundle_mutex);
    if (impl_->current != handle) {
      return impl_->current;
    }
    if (!impl_->current->status.map_healthy) {
      return impl_->current;
    }
    auto expired = std::make_shared<AdapterHandle>(*impl_->current);
    expired->version = impl_->version_counter.fetch_add(1) + 1;
    expired->status.code = AdapterCode::kStaleReceipt;
    expired->status.map_healthy = false;
    expired->status.control_ready = false;
    expired->status.reason =
        "steady-clock receipt age exceeded max_stamp_age_sec; no new cloud";
    impl_->current = expired;
    impl_->last_status = expired->status;
    stale = expired;
  }
  // Keep ingress_age_ms tied to the last accepted cloud's receipt time. Using
  // node->now() here would mislabel time spent waiting for a new cloud as
  // upstream input latency and would not match this handle's mapping_ms.
  const double stale_ingress_age_ms =
      ingressAgeMs(stale->receive_stamp, stale->measurement_stamp);
  impl_->logStatus(stale->status, stale->mapping_ms, stale_ingress_age_ms);
  return stale;
}

AdapterStatus LocalMapRosAdapter::status() const {
  if (impl_ == nullptr) {
    AdapterStatus status;
    status.code = AdapterCode::kNotInitialized;
    status.reason = "adapter implementation is missing";
    return status;
  }
  std::lock_guard<std::mutex> lock(impl_->bundle_mutex);
  if (impl_->current != nullptr) {
    return impl_->current->status;
  }
  return impl_->last_status;
}

const AdapterConfig& LocalMapRosAdapter::config() const {
  return impl_->config;
}

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
LocalMapRosAdapter::cloudSubscription() const {
  return impl_ != nullptr ? impl_->subscription : nullptr;
}

bool LocalMapRosAdapter::ingestCloud(
    const sensor_msgs::msg::PointCloud2& cloud, const rclcpp::Time& now,
    std::chrono::steady_clock::time_point receive_steady, std::string* error) {
  if (impl_ == nullptr) {
    if (error != nullptr) {
      *error = "adapter implementation is missing";
    }
    return false;
  }
  const auto mapping_start = std::chrono::steady_clock::now();
  const rclcpp::Time measurement_stamp(cloud.header.stamp);
  Impl::UpdateTiming timing;
  std::lock_guard<std::mutex> lock(impl_->update_mutex);
  timing.lock_wait_ms = Impl::elapsedMs(mapping_start);
  const double ingress_age_ms = ingressAgeMs(now, measurement_stamp);
  if (impl_->tf_buffer == nullptr) {
    const double mapping_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - mapping_start)
            .count();
    const std::string reason =
        "no TF buffer is attached to this offline adapter";
    impl_->publishFailure(AdapterCode::kInvalidConfig, reason, mapping_ms,
                          ingress_age_ms, true, &timing);
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  }

  if (!impl_->prepareLocked(cloud, measurement_stamp, now, receive_steady,
                            mapping_start, &timing, error)) {
    return false;
  }

  EigenTransform map_from_sensor;
  EigenTransform map_from_base;
  const auto tf_start = std::chrono::steady_clock::now();
  try {
    const geometry_msgs::msg::TransformStamped sensor_transform =
        impl_->tf_buffer->lookupTransform(
            impl_->config.map_frame, impl_->config.sensor_frame,
            measurement_stamp,
            rclcpp::Duration::from_seconds(impl_->config.tf_timeout_sec));
    const geometry_msgs::msg::TransformStamped base_transform =
        impl_->tf_buffer->lookupTransform(
            impl_->config.map_frame, impl_->config.base_frame,
            measurement_stamp,
            rclcpp::Duration::from_seconds(impl_->config.tf_timeout_sec));
    map_from_sensor =
        makeEigenTransform(sensor_transform.transform.rotation.x,
                           sensor_transform.transform.rotation.y,
                           sensor_transform.transform.rotation.z,
                           sensor_transform.transform.rotation.w,
                           Eigen::Vector3d(sensor_transform.transform.translation.x,
                                           sensor_transform.transform.translation.y,
                                           sensor_transform.transform.translation.z));
    map_from_base =
        makeEigenTransform(base_transform.transform.rotation.x,
                           base_transform.transform.rotation.y,
                           base_transform.transform.rotation.z,
                           base_transform.transform.rotation.w,
                           Eigen::Vector3d(base_transform.transform.translation.x,
                                           base_transform.transform.translation.y,
                                           base_transform.transform.translation.z));
    timing.tf_total_ms = Impl::elapsedMs(tf_start);
  } catch (const tf2::TransformException& exception) {
    timing.tf_total_ms = Impl::elapsedMs(tf_start);
    const double mapping_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - mapping_start)
            .count();
    const std::string reason =
        std::string("TF lookup failed at acquisition stamp: ") +
        exception.what();
    impl_->publishFailure(AdapterCode::kTfFailure, reason, mapping_ms,
                          ingress_age_ms, true, &timing);
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  } catch (const std::exception& exception) {
    timing.tf_total_ms = Impl::elapsedMs(tf_start);
    const double mapping_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - mapping_start)
            .count();
    const std::string reason =
        std::string("TF lookup exception: ") + exception.what();
    impl_->publishFailure(AdapterCode::kTfFailure, reason, mapping_ms,
                          ingress_age_ms, true, &timing);
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  }

  return impl_->processLocked(cloud, map_from_sensor, map_from_base,
                              measurement_stamp, now, receive_steady,
                              mapping_start, &timing, error);
}

bool LocalMapRosAdapter::ingestCloudWithTransforms(
    const sensor_msgs::msg::PointCloud2& cloud,
    const EigenTransform& map_from_sensor, const EigenTransform& map_from_base,
    const rclcpp::Time& measurement_stamp, const rclcpp::Time& now,
    std::chrono::steady_clock::time_point receive_steady, std::string* error) {
  if (impl_ == nullptr) {
    if (error != nullptr) {
      *error = "adapter implementation is missing";
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->update_mutex);
  return impl_->ingestCloudWithTransformsLocked(
      cloud, map_from_sensor, map_from_base, measurement_stamp, now,
      receive_steady, error);
}

}  // namespace ros
}  // namespace dddmr_local_map
