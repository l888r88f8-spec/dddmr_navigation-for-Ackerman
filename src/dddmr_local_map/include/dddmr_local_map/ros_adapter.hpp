#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>

#include "dddmr_local_map/terrain.hpp"
#include "dddmr_local_map/voxel_map.hpp"

namespace dddmr_local_map {
namespace ros {

// A plain Eigen transform supplied by a test or by an existing TF lookup.
// `valid` must be true and the rotation must be a proper, finite rotation.
struct EigenTransform {
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  bool valid{false};

  Eigen::Vector3d apply(const Eigen::Vector3d& point) const {
    return rotation * point + translation;
  }
};

EigenTransform makeEigenTransform(const Eigen::Quaterniond& rotation,
                                  const Eigen::Vector3d& translation);
EigenTransform makeEigenTransform(double qx, double qy, double qz, double qw,
                                  const Eigen::Vector3d& translation);
bool isFiniteTransform(const EigenTransform& transform);

enum class StampRejection : uint8_t {
  kNone = 0,
  kNonFinite,
  kZero,
  kFuture,
  kStale,
  kNonMonotonic,
};

const char* toString(StampRejection rejection);

struct StampValidation {
  bool ok{false};
  StampRejection rejection{StampRejection::kNonFinite};
  std::string detail;
};

// Pure acquisition-stamp policy. `last_accepted_sec <= 0` means "no accepted
// stamp yet". A stamp is accepted only when it is finite, non-zero, not more
// than `future_tolerance_sec` in the future, not more than `max_age_sec` old,
// and strictly newer than the previously accepted stamp.
StampValidation validateAcquisitionStamp(double stamp_sec, double now_sec,
                                         double last_accepted_sec,
                                         double max_age_sec,
                                         double future_tolerance_sec);

// Leading slashes are ignored (ROS 1 bags occasionally contain them); frame
// names are otherwise compared exactly.
bool validateCloudFrame(const std::string& frame_id,
                        const std::string& expected_frame,
                        std::string* error = nullptr);

struct SelfFilterConfig {
  // Disabled by default. The adapter enables it only after the configured
  // vehicle geometry validates, and then copies the measured/configured body
  // AABB here. There is deliberately no guessed cylinder or invented envelope.
  bool enabled{false};
  // Base-frame rigid-body AABB. Points strictly inside x/y and with
  // body_min.z < z <= body_max.z are removed as self returns. Points at or
  // below body_min.z (floor, ramp, wheel contact) are always retained.
  Eigen::Vector3d body_min{kNaN, kNaN, kNaN};
  Eigen::Vector3d body_max{kNaN, kNaN, kNaN};
};

struct CloudConversionStats {
  std::size_t input_points{0};
  std::size_t finite_points{0};
  // Endpoints inside the 3-D map window.
  std::size_t cropped_points{0};
  // Endpoints outside the window, whether or not their ray crosses it.
  std::size_t outside_window_points{0};
  // Outside endpoints retained because the sensor-to-endpoint segment crosses
  // the map window under WindowRayPolicy::kKeepCrossingRays.
  std::size_t retained_crossing_points{0};
  std::size_t self_removed{0};
  std::size_t valid_points{0};
};

// Default keeps the historical converter behavior: endpoints outside the map
// window are dropped. Production mapping uses kKeepCrossingRays so the core
// can clip the ray to max range/window and never mark an out-of-window
// endpoint as a boundary obstacle.
enum class WindowRayPolicy : uint8_t {
  kDropOutsideEndpoints = 0,
  kKeepCrossingRays = 1,
};

// Converts the cloud from sensor frame to map frame using the supplied Eigen
// transform, drops non-finite points, applies the full 3-D map window policy,
// and removes only points strictly inside the configured rigid-body AABB above
// body_min.z. Malformed fields/NaN/Inf/zero data are reported as a Validation
// failure, not an exception. It intentionally performs no height/non-ground or
// max-range crop; the core owns max-ray clipping.
Validation pointCloudToMapPoints(
    const sensor_msgs::msg::PointCloud2& cloud,
    const EigenTransform& map_from_sensor, const EigenTransform& map_from_base,
    const Eigen::Vector3d& window_center,
    const Eigen::Vector3d& window_half_extent,
    const SelfFilterConfig& self_filter, std::vector<Eigen::Vector3d>* points,
    CloudConversionStats* stats = nullptr,
    std::size_t max_points = 5'000'000U,
    WindowRayPolicy window_policy =
        WindowRayPolicy::kDropOutsideEndpoints);

enum class AdapterCode : uint8_t {
  kNotInitialized = 0,
  kOk,
  kInvalidConfig,
  kZeroStamp,
  kNonFiniteStamp,
  kFutureStamp,
  kStaleStamp,
  kNonMonotonicStamp,
  kWrongFrame,
  kInvalidCloud,
  kNoValidRays,
  kTfFailure,
  kCoreRejected,
  kClockRewind,
  kStaleReceipt,
  kBuildFailure,
};

const char* toString(AdapterCode code);

struct AdapterStatus {
  AdapterCode code{AdapterCode::kNotInitialized};
  bool map_healthy{false};
  bool control_ready{false};
  std::string reason;

  bool ok() const { return code == AdapterCode::kOk && map_healthy; }
};

struct AdapterConfig {
  // Defaults describe the shadow window: 12.8 x 12.8 x 4.0 m at 0.1 m.
  MapConfig map;
  TerrainConfig terrain;
  VehicleGeometry geometry;  // NaN/unset by default on purpose.
  Validation geometry_status{Validation::failure(
      "vehicle geometry is not configured; map is usable for inspection but "
      "not ready for control")};

  std::string cloud_topic{"/lio_sam/deskew/cloud_deskewed"};
  std::string sensor_frame{"hesai_lidar"};
  std::string map_frame{"odom"};
  std::string base_frame{"base_link"};

  double tf_timeout_sec{0.10};
  double max_stamp_age_sec{0.50};
  double future_tolerance_sec{0.05};
  double clock_rewind_tolerance_sec{0.10};
  bool use_steady_receipt_age{true};

  SelfFilterConfig self_filter;

  bool log_status{true};
  double status_log_period_sec{2.00};
  std::string parameter_prefix{"local_map."};
};

// Validates map/terrain/frame/timeout/self-filter parameters. Geometry is
// deliberately not required: the shadow map may run with unset geometry and
// reports control_ready=false. Callers that need a control-ready backend
// should reject an invalid `geometry_status` at initialization.
Validation validateAdapterConfig(const AdapterConfig& config);

// Declares/reads parameters under `prefix` (default "local_map."). All names
// are prefixed, so this never introduces a root `global_frame` that can
// collide with the planner's parameters.
Validation loadAdapterConfig(rclcpp::Node& node, const std::string& prefix,
                             AdapterConfig* config);

// One immutable generation of map data plus the status under which it was
// observed. The controller must use the snapshot and terrain from this same
// handle and must not combine them with separate core getters.
struct AdapterHandle {
  std::shared_ptr<const MapSnapshot> snapshot;
  std::shared_ptr<const TerrainModel> terrain;
  rclcpp::Time measurement_stamp{0LL, RCL_ROS_TIME};
  rclcpp::Time receive_stamp{0LL, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point receive_steady{};
  uint64_t version{0};
  uint64_t generation{0};
  double mapping_ms{0.0};
  AdapterStatus status;

  bool mapHealthy() const {
    return status.map_healthy && snapshot != nullptr && terrain != nullptr;
  }
  bool readyForControl() const {
    return mapHealthy() && status.control_ready;
  }
};

// Adapter attached to an existing node and to the planner's own TF buffer.
// The node/buffer must outlive the adapter. The full constructor creates the
// cloud subscription immediately in a dedicated mutually-exclusive callback
// group with SensorDataQoS(depth 1); the node's executor spins it.
//
// The offline constructor creates no node/subscription. Use
// ingestCloudWithTransforms() for deterministic tests without DDS/network.
class LocalMapRosAdapter {
 public:
  LocalMapRosAdapter(rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
                     AdapterConfig config);
  explicit LocalMapRosAdapter(AdapterConfig config);
  ~LocalMapRosAdapter();

  LocalMapRosAdapter(const LocalMapRosAdapter&) = delete;
  LocalMapRosAdapter& operator=(const LocalMapRosAdapter&) = delete;
  LocalMapRosAdapter(LocalMapRosAdapter&&) = delete;
  LocalMapRosAdapter& operator=(LocalMapRosAdapter&&) = delete;

  // Starts the subscription for a node-attached adapter. Idempotent; the
  // full constructor already starts it and throws if creation fails.
  Validation start();
  void stop();
  bool started() const;

  // Current immutable bundle. `handleAt` is the testable/injectable-time
  // variant; it also expires a stale bundle using steady-clock receipt age.
  std::shared_ptr<const AdapterHandle> handle() const;
  std::shared_ptr<const AdapterHandle> handleAt(
      std::chrono::steady_clock::time_point steady_now) const;
  AdapterStatus status() const;

  const AdapterConfig& config() const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
  cloudSubscription() const;

  // Full path: validates acquisition stamp/frame, looks up map<-sensor and
  // map<-base at the acquisition stamp from the supplied TF buffer, then
  // serializes the update. Useful for integration tests with a buffer.
  bool ingestCloud(const sensor_msgs::msg::PointCloud2& cloud,
                   const rclcpp::Time& now,
                   std::chrono::steady_clock::time_point receive_steady,
                   std::string* error = nullptr);

  // Deterministic path with externally supplied transforms. Used by tests and
  // by callers that already obtained both transforms at the acquisition
  // stamp. Both transforms are map<-sensor and map<-base.
  bool ingestCloudWithTransforms(
      const sensor_msgs::msg::PointCloud2& cloud,
      const EigenTransform& map_from_sensor,
      const EigenTransform& map_from_base, const rclcpp::Time& measurement_stamp,
      const rclcpp::Time& now,
      std::chrono::steady_clock::time_point receive_steady,
      std::string* error = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros
}  // namespace dddmr_local_map
