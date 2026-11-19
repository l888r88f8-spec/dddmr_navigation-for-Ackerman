// Copyright 2026 DDDMobileRobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "grid_global_planner/grid_global_planner.h"

#include <perception_3d/perception_snapshot.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "grid_global_planner/static_map_input.h"
#include "grid_global_planner/terrain_visualization.h"
#include "geometry_msgs/msg/point.hpp"
#include "perception_observation_ingestion.h"  // NOLINT(build/include_subdir): private src/ header
#include "request_pose_semantics.h"  // NOLINT(build/include_subdir): private src/ header
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace grid_global_planner
{
namespace
{

// The unified action name is intentionally not configurable: p2p_move_base
// keeps route_action_name: get_plan, so the provider and client contract must
// stay identical to the legacy planner.
constexpr char kActionName[] = "/get_plan";

// Every startup resolution/vehicle/search/observation-profile parameter is
// declared read-only. ROS then rejects a runtime override instead of accepting
// a value the node cached during construction and would silently ignore.
rcl_interfaces::msg::ParameterDescriptor ReadOnlyParameterDescriptor()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description =
    "Startup-only dddmr grid planner configuration; restart to change";
  return descriptor;
}

// The p2p navigation YAML used these legacy names for the shared frame/topic and
// offline fallback fields. Only these documented aliases are accepted; the
// backend/search groups are native (raw_route_backend, graph_a_star.*,
// hybrid_a_star.*) and unsupported or obsolete keys are rejected at startup
// rather than silently ignored.
struct StartupAliasSpec
{
  const char * alias_name;
  rclcpp::ParameterType type;
  bool offline_fallback;
};

const std::map<std::string, StartupAliasSpec> & StartupAliasSpecs()
{
  static const std::map<std::string, StartupAliasSpec> specs{
    {"ros.global_frame",
      {"global_frame", rclcpp::ParameterType::PARAMETER_STRING, false}},
    {"ros.robot_base_frame",
      {"robot_base_frame", rclcpp::ParameterType::PARAMETER_STRING, false}},
    {"ros.static_obstacle_cloud_topic",
      {"map_topic", rclcpp::ParameterType::PARAMETER_STRING, false}},
    {"ros.ground_cloud_topic",
      {"ground_topic", rclcpp::ParameterType::PARAMETER_STRING, false}},
    {"ros.dynamic_obstacle_cloud_topic",
      {"preprocessed_cloud_topic",
        rclcpp::ParameterType::PARAMETER_STRING, false}},
    {"terrain.fine_resolution_m",
      {"offline.resolution_m", rclcpp::ParameterType::PARAMETER_DOUBLE, true}},
    {"terrain.origin_x_m",
      {"offline.origin_x_m", rclcpp::ParameterType::PARAMETER_DOUBLE, true}},
    {"terrain.origin_y_m",
      {"offline.origin_y_m", rclcpp::ParameterType::PARAMETER_DOUBLE, true}},
  };
  return specs;
}

void RejectUnsupportedStartupAliasGroups(
  const std::map<std::string, rclcpp::ParameterValue> & overrides)
{
  const std::array<const char *, 18> hybrid_keys{
    "primitive_length_m",
    "steer_samples",
    "allow_reverse",
    "max_wheel_sample_spacing_m",
    "max_yaw_sample_step_rad",
    "goal_position_tolerance_m",
    "goal_heading_tolerance_rad",
    "goal_connection_max_length_m",
    "goal_connection_samples",
    "max_coarse_routes",
    "reverse_penalty_weight",
    "steering_penalty_weight",
    "terrain_cost_weight",
    "heading_bin_rad",
    "corridor_initial_margin_m",
    "corridor_margin_step_m",
    "corridor_max_enlargements",
    "corridor_max_corridor_cells",
  };
  const auto is_supported_hybrid_key = [&hybrid_keys](
    const std::string & name) {
      return std::any_of(
        hybrid_keys.begin(), hybrid_keys.end(),
        [&name](const char * key) {return name == key;});
    };

  for (const auto & entry : overrides) {
    const std::string & name = entry.first;
    if (name == "search" || name.rfind("search.", 0) == 0) {
      throw std::runtime_error(
              "legacy search parameter '" + name + "' is no longer supported; use "
              "raw_route_backend for backend selection, hybrid_a_star.<key> "
              "for hybrid search settings, and "
              "graph_a_star.max_samples for the search sample budget");
    }
    if (name == "hybrid_astar" ||
      name.rfind("hybrid_astar.", 0) == 0)
    {
      throw std::runtime_error(
              "hybrid_astar parameter group is not supported; use "
              "raw_route_backend=hybrid_a_star and native hybrid_a_star.* "
              "parameters");
    }
    if (name == "hybrid_a_star" ||
      name.rfind("hybrid_a_star.", 0) == 0)
    {
      const std::string suffix =
        name == "hybrid_a_star" ? std::string() : name.substr(14);
      if (!is_supported_hybrid_key(suffix)) {
        std::string supported_keys;
        for (const char * key : hybrid_keys) {
          if (!supported_keys.empty()) {
            supported_keys += ", ";
          }
          supported_keys += std::string("hybrid_a_star.") + key;
        }
        throw std::runtime_error(
                "hybrid_a_star parameter '" + name + "' is not supported; "
                "supported keys are " + supported_keys);
      }
    }
    const auto is_supported_graph_key = [](const std::string & key) {
        return key == "graph_a_star.max_samples" ||
               key == "graph_a_star.slope_penalty_weight" ||
               key == "graph_a_star.roughness_penalty_weight" ||
               key == "graph_a_star.obstacle_penalty_distance_m" ||
               key == "graph_a_star.obstacle_penalty_weight" ||
               key == "graph_a_star.boundary_penalty_distance_m" ||
               key == "graph_a_star.boundary_penalty_weight";
      };
    if ((name == "graph_a_star" ||
      name.rfind("graph_a_star.", 0) == 0) &&
      !is_supported_graph_key(name))
    {
      throw std::runtime_error(
              "graph_a_star parameter '" + name + "' is not supported; "
              "supported keys are max_samples, "
              "slope_penalty_weight, roughness_penalty_weight, "
              "obstacle_penalty_distance_m, obstacle_penalty_weight, "
              "boundary_penalty_distance_m, and boundary_penalty_weight");
    }
  }
}

void ValidateStartupAliasValue(
  const std::string & native_name, const StartupAliasSpec & spec,
  const rclcpp::ParameterValue & value)
{
  if (value.get_type() != spec.type) {
    throw std::runtime_error(
            "parameter alias '" + std::string(spec.alias_name) + "' for '" +
            native_name + "' must be " + rclcpp::to_string(spec.type) +
            ", got " + rclcpp::to_string(value.get_type()));
  }
  if (spec.type == rclcpp::ParameterType::PARAMETER_INTEGER &&
    value.get<std::int64_t>() < 0)
  {
    throw std::runtime_error(
            "parameter alias '" + std::string(spec.alias_name) +
            "' must be a non-negative integer, got " +
            std::to_string(value.get<std::int64_t>()));
  }
}

// New MLS memory/render caps validate the raw signed ROS value before any
// size_t cast. A zero/negative offline cap is a configuration error, not a
// silent cap of one; only an explicit zero measurement-render cap means
// unlimited. Existing legacy parameter readers keep their historical behavior.
std::size_t CheckedPositiveSizeParameter(
  std::int64_t raw, const char * key)
{
  if (raw <= 0) {
    throw std::runtime_error(
            std::string(key) + " must be a positive integer, got " +
            std::to_string(raw));
  }
  if (static_cast<std::uint64_t>(raw) >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
  {
    throw std::runtime_error(
            std::string(key) + " exceeds the platform size_t range");
  }
  return static_cast<std::size_t>(raw);
}

std::size_t CheckedNonNegativeSizeParameter(
  std::int64_t raw, const char * key)
{
  if (raw < 0) {
    throw std::runtime_error(
            std::string(key) + " must be zero or positive, got " +
            std::to_string(raw));
  }
  if (static_cast<std::uint64_t>(raw) >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
  {
    throw std::runtime_error(
            std::string(key) + " exceeds the platform size_t range");
  }
  return static_cast<std::size_t>(raw);
}

// Stable, non-hot-path diagnostic strings.
constexpr char kResultSuccess[] = "succeeded";
constexpr char kResultNotReady[] = "not_ready";
constexpr char kResultVehicleUnconfigured[] = "vehicle-capability-unconfigured";
constexpr char kResultInvalidStart[] = "invalid_start";
constexpr char kResultInvalidGoal[] = "invalid_goal";
constexpr char kResultFrameMismatch[] = "frame_mismatch";
constexpr char kResultCanceled[] = "planner_canceled";
constexpr char kResultInternalError[] = "internal_error";
constexpr char kResultLatestInvalid[] = "latest_validation_invalid";
constexpr char kResultBusy[] = "busy";
constexpr char kResultDeactivated[] = "deactivated";

// Converts a steady-clock elapsed interval to the action's Duration field.
builtin_interfaces::msg::Duration ToRosDuration(std::chrono::nanoseconds ns)
{
  builtin_interfaces::msg::Duration result;
  const int64_t count = std::max<int64_t>(0, ns.count());
  result.sec = static_cast<std::int32_t>(count / 1'000'000'000LL);
  result.nanosec = static_cast<std::uint32_t>(count % 1'000'000'000LL);
  return result;
}

// Converts an rclcpp ROS clock time to a message stamp without relying on an
// implicit conversion operator.
builtin_interfaces::msg::Time ToRosTime(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t ns = std::max<int64_t>(0, time.nanoseconds());
  stamp.sec = static_cast<std::int32_t>(ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000LL);
  return stamp;
}

double DurationSeconds(std::chrono::nanoseconds ns)
{
  return std::chrono::duration<double>(ns).count();
}

// Removes delimiters from free-form diagnostics so the private topic remains a
// single key=value record.
std::string Sanitize(const std::string & input)
{
  std::string output = input;
  for (char & character : output) {
    if (character == ';' || character == '\n' || character == '\r') {
      character = ' ';
    }
  }
  return output;
}

// Stable spelling for worker submission statuses. The core contract does not
// expose a to-string helper for this enum.
const char * MapUpdateStatusToString(MapUpdateStatus status) noexcept
{
  switch (status) {
    case MapUpdateStatus::kAccepted:
      return "accepted";
    case MapUpdateStatus::kRejectedInvalid:
      return "rejected_invalid";
    case MapUpdateStatus::kRejectedStale:
      return "rejected_stale";
    case MapUpdateStatus::kRejectedDuplicate:
      return "rejected_duplicate";
    case MapUpdateStatus::kRejectedCapacity:
      return "rejected_capacity";
    case MapUpdateStatus::kQueueFull:
      return "queue_full";
    case MapUpdateStatus::kQueueFullCoalesced:
      return "queue_full_coalesced";
    case MapUpdateStatus::kStopped:
      return "stopped";
  }
  return "unknown";
}

// Maps core search status to the diagnostic result class used by p2p. The
// action has no error field, so this is the stable failure channel.
const char * SearchStatusToResultClass(SearchStatus status) noexcept
{
  switch (status) {
    case SearchStatus::kSuccess:
      return kResultSuccess;
    case SearchStatus::kNoRoute:
      return "no_route";
    case SearchStatus::kBudgetExhausted:
      return "budget_exhausted";
    case SearchStatus::kCancelled:
      return kResultCanceled;
    case SearchStatus::kInvalidRequest:
      return "invalid_request";
    case SearchStatus::kNotReady:
      return kResultNotReady;
    case SearchStatus::kVehicleCapabilityUnconfigured:
      return kResultVehicleUnconfigured;
    case SearchStatus::kInternalError:
      return kResultInternalError;
  }
  return kResultInternalError;
}

// Applies a TF transform to one map-frame point.
Point3 TransformPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  const Point3 & point)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);
  if (rotation.length2() > 1e-18) {
    rotation.normalize();
  } else {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);
  }
  tf2::Vector3 vector(point.x, point.y, point.z);
  vector = tf2::quatRotate(rotation, vector);
  vector += tf2::Vector3(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
  return Point3{vector.x(), vector.y(), vector.z()};
}

// Builds the final nav_msgs/Path. Roll, pitch, and yaw are all converted from
// the solved PathPose; no orientation component is dropped.
nav_msgs::msg::Path BuildRosPath(
  const std::vector<PathPose> & path,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Path message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.poses.reserve(path.size());
  for (const PathPose & pose : path) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = message.header;
    stamped.pose.position.x = pose.position.x;
    stamped.pose.position.y = pose.position.y;
    stamped.pose.position.z = pose.position.z;
    tf2::Quaternion quaternion;
    quaternion.setRPY(pose.roll, pose.pitch, pose.heading);
    quaternion.normalize();
    stamped.pose.orientation = tf2::toMsg(quaternion);
    message.poses.push_back(std::move(stamped));
  }
  return message;
}

// Builds a bounded x/y/z float cloud for visualization. The algorithm does not
// depend on PCL; the node package links PCL only for the wider navigation
// workspace compatibility required by the vehicle stack.
sensor_msgs::msg::PointCloud2 MakePointCloud2(
  const std::vector<Point3> & points, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.height = 1;
  message.width = static_cast<std::uint32_t>(points.size());
  message.fields.resize(3);
  const char * names[3] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    message.fields[i].name = names[i];
    message.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
    message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[i].count = 1;
  }
  message.is_bigendian = false;
  message.point_step = static_cast<std::uint32_t>(3 * sizeof(float));
  message.row_step = message.point_step * message.width;
  message.is_dense = true;
  message.data.resize(static_cast<std::size_t>(message.row_step));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float values[3] = {static_cast<float>(points[i].x),
      static_cast<float>(points[i].y),
      static_cast<float>(points[i].z)};
    std::memcpy(
      message.data.data() + i * message.point_step, values,
      sizeof(values));
  }
  return message;
}

// Returns the XY yaw of a normalized message quaternion.
double QuaternionYaw(const geometry_msgs::msg::Quaternion & message)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(message, quaternion);
  if (quaternion.length2() <= 1e-18) {
    return 0.0;
  }
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

// Converts an explicit ROS observation point into the planner point type.
Point3 RosPointToPoint3(const geometry_msgs::msg::Point & point)
{
  return Point3{point.x, point.y, point.z};
}

// Maps one explicit ROS volume. Bounds are copied exactly; no enclosing AABB is
// invented from point-only data.
ObservedVolume RosVolumeToObservedVolume(
  const grid_global_planner::msg::ObservedVolume & volume)
{
  ObservedVolume converted;
  converted.min = RosPointToPoint3(volume.min);
  converted.max = RosPointToPoint3(volume.max);
  converted.known = volume.known;
  converted.free_space = volume.free_space;
  converted.headroom_m = volume.headroom_m;
  converted.headroom_known = volume.headroom_known;
  return converted;
}

// Returns true when an explicit ROS volume has finite, ordered bounds and a
// finite non-negative headroom. An unknown volume must not claim headroom.
bool RosVolumeBoundsAreValid(
  const grid_global_planner::msg::ObservedVolume & volume)
{
  if (volume.known) {
    const Point3 min_point = RosPointToPoint3(volume.min);
    const Point3 max_point = RosPointToPoint3(volume.max);
    if (!min_point.IsFinite() || !max_point.IsFinite() ||
      min_point.x > max_point.x || min_point.y > max_point.y ||
      min_point.z > max_point.z)
    {
      return false;
    }
  } else if (volume.headroom_known) {
    return false;
  }
  if (volume.headroom_known) {
    return std::isfinite(volume.headroom_m) && volume.headroom_m >= 0.0;
  }
  return true;
}

// Returns true when one explicit ROS record has finite geometry and a
// non-inverted observed Z slab.
bool RosRecordBoundsAreValid(
  const grid_global_planner::msg::SourceOccupancyRecord & record)
{
  const Point3 point = RosPointToPoint3(record.point);
  if (!point.IsFinite() || !std::isfinite(record.min_z) ||
    !std::isfinite(record.max_z))
  {
    return false;
  }
  return record.min_z <= record.max_z;
}

// Maps one explicit ROS occupied/cleared record. The record carries an observed
// slab; a point-only producer must set min_z == max_z == point.z.
SourceOccupancyRecord RosRecordToOccupancy(
  const grid_global_planner::msg::SourceOccupancyRecord & record)
{
  SourceOccupancyRecord converted;
  converted.point = RosPointToPoint3(record.point);
  converted.min_z = record.min_z;
  converted.max_z = record.max_z;
  return converted;
}

// Freezes one explicit ROS source observation into the immutable core type.
SourceObservationSnapshot ConvertSourceObservationMessage(
  const grid_global_planner::msg::SourceObservation & message,
  std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
  std::uint64_t terrain_revision, bool terrain_revision_known)
{
  const bool is_static =
    message.layer ==
    grid_global_planner::msg::SourceObservation::LAYER_STATIC;
  const SourceLayer layer =
    is_static ? SourceLayer::kStatic : SourceLayer::kDynamic;
  const bool is_exact_delta =
    message.update_kind ==
    grid_global_planner::msg::SourceObservation::UPDATE_EXACT_DELTA;
  const SourceUpdateKind kind =
    is_exact_delta ? SourceUpdateKind::kExactDelta :
    SourceUpdateKind::kCompleteReplacement;
  std::vector<SourceOccupancyRecord> occupied;
  occupied.reserve(message.occupied.size());
  for (const auto & record : message.occupied) {
    occupied.push_back(RosRecordToOccupancy(record));
  }
  std::vector<SourceOccupancyRecord> cleared;
  cleared.reserve(message.cleared.size());
  for (const auto & record : message.cleared) {
    cleared.push_back(RosRecordToOccupancy(record));
  }
  std::vector<ObservedVolume> known_volumes;
  known_volumes.reserve(message.known_volumes.size());
  for (const auto & volume : message.known_volumes) {
    known_volumes.push_back(RosVolumeToObservedVolume(volume));
  }
  return SourceObservationSnapshot(
    message.source_id, message.header.frame_id, message.source_sequence,
    terrain_revision, sensor_stamp_ns, received_steady_ns, layer, kind,
    message.complete, std::move(occupied), std::move(cleared),
    std::move(known_volumes), RosVolumeToObservedVolume(message.headroom));
}

// Converts a steady-clock duration to nanoseconds for timer configuration.
std::chrono::nanoseconds SecondsToSteadyDuration(double seconds)
{
  const double clamped = std::max(0.0, seconds);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(clamped));
}

}  // namespace

double GridGlobalPlanner::StaticMapLoadElapsedSeconds() const
{
  const std::int64_t started_ns =
    static_map_load_started_ns_.load(std::memory_order_relaxed);
  if (started_ns <= 0) {
    return 0.0;
  }
  const auto elapsed =
    std::chrono::steady_clock::now().time_since_epoch() -
    std::chrono::nanoseconds(started_ns);
  return DurationSeconds(
    std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed));
}

GridGlobalPlanner::GridGlobalPlanner(const rclcpp::NodeOptions & options)
: GridGlobalPlanner(options, SceneSnapshotProvider()) {}

GridGlobalPlanner::GridGlobalPlanner(
  const rclcpp::NodeOptions & options,
  SceneSnapshotProvider scene_provider,
  StaticSourceInputOptions static_source)
: rclcpp::Node("grid_global_planner", options),
  scene_snapshot_provider_(std::move(scene_provider)),
  static_source_input_options_(static_source)
{
  // Parameter declaration and loading happen before any callback can run. The
  // worker and executor-facing interfaces are created after the configuration
  // is validated so an invalid node fails closed at startup. If any startup
  // step throws after a thread was created, join it before rethrowing: a
  // constructor that throws never runs ~GridGlobalPlanner, and a joinable
  // std::thread member would otherwise terminate the process.
  try {
    LoadConfiguration();
    CreateRosInterfaces();
    StartMapWorker();
    StartThreads();

    RCLCPP_INFO(
      get_logger(),
      "grid_global_planner ready: action=%s frame=%s base=%s "
      "ground_topic=%s source_observation_topic=%s "
      "raw_cloud_fallback=%d perception_export=%d "
      "static_topic=%s dynamic_topic=%s "
      "fine_resolution=%.3f coarse_resolution=%.3f "
      "wheelbase=%.3f track=%.3f contact=+-0.275/+-0.26/-0.22",
      kActionName, config_.global_frame.c_str(),
      config_.robot_base_frame.c_str(), config_.ground_cloud_topic.c_str(),
      config_.source_observation_topic.c_str(),
      config_.enable_raw_cloud_fallback ? 1 : 0,
      config_.enable_perception_export ? 1 : 0,
      config_.static_obstacle_cloud_topic.c_str(),
      config_.dynamic_obstacle_cloud_topic.c_str(), config_.terrain.resolution,
      config_.coarse.coarse_resolution, config_.vehicle.wheelbase_m,
      config_.vehicle.track_width_m);

    std::string vehicle_detail;
    const VehicleConfigurationStatus vehicle_status =
      config_.vehicle.Validate(&vehicle_detail);
    if (config_.wheel.model ==
      TraversabilityModel::kOfflineKinematicStaticMap)
    {
      // The offline model is explicitly named. It must not be described as a
      // measured full-vehicle validation.
      WheelSupportEvaluator offline_probe(config_.vehicle, config_.wheel);
      RCLCPP_INFO(
        get_logger(),
        "evaluation model=%s configured=%d detail=%s",
        TraversabilityModelToString(config_.wheel.model),
        offline_probe.IsConfigured() ? 1 : 0,
        offline_probe.configuration_detail().c_str());
      if (!offline_probe.IsConfigured()) {
        RCLCPP_WARN(
          get_logger(),
          "offline evaluation model is not configured; /get_plan "
          "will fail closed");
      }
    } else if (vehicle_status == VehicleConfigurationStatus::kConfigured) {
      RCLCPP_INFO(
        get_logger(), "vehicle capability configured: %s",
        vehicle_detail.c_str());
    } else {
      // This is the required fail-closed startup diagnostic. UNSET physical
      // values are never replaced by guessed values.
      RCLCPP_WARN(
        get_logger(),
        "vehicle capability status=%s detail=%s; tire_radius_m/tire_width_m/"
        "suspension_travel_m/tire_compliance_m/body_max are UNSET until "
        "measured",
        VehicleConfigurationStatusToString(vehicle_status),
        vehicle_detail.c_str());
    }
    if (config_.vehicle_capability_declared_configured &&
      vehicle_status != VehicleConfigurationStatus::kConfigured)
    {
      RCLCPP_WARN(
        get_logger(),
        "vehicle.vehicle_capability_configured was declared true but measured "
        "parameters are incomplete; measured values remain authoritative");
    }
  } catch (...) {
    StopThreads();
    throw;
  }
}

GridGlobalPlanner::~GridGlobalPlanner()
{
  // Detach executor-facing callbacks before joining the owned threads. The
  // perception provider captures the perception node weakly, so resetting it
  // here cannot leave a dangling end-of-cycle callback.
  if (terrain_visualization_timer_) {
    terrain_visualization_timer_->cancel();
    terrain_visualization_timer_.reset();
  }
  terrain_visualization_callback_group_.reset();
  if (perception_timer_) {
    perception_timer_->cancel();
    perception_timer_.reset();
  }
  if (source_observation_subscription_) {
    source_observation_subscription_.reset();
  }
  if (static_ground_subscription_) {
    static_ground_subscription_.reset();
  }
  if (static_global_subscription_) {
    static_global_subscription_.reset();
  }
  // StopThreads() disables/resets the action server before dropping any
  // uncertain goal-handle owner, so Humble's weak server callback cannot publish
  // from the implicit ServerGoalHandle destructor during shutdown.
  StopThreads();
  map_worker_.reset();
  snapshot_store_.reset();
  scene_snapshot_provider_ = nullptr;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

std::optional<rclcpp::ParameterValue>
GridGlobalPlanner::ResolveStartupAlias(const std::string & native_name)
{
  const auto & specs = StartupAliasSpecs();
  const auto spec_it = specs.find(native_name);
  if (spec_it == specs.end()) {
    return std::nullopt;
  }
  const auto & overrides =
    get_node_parameters_interface()->get_parameter_overrides();
  const auto alias_it = overrides.find(spec_it->second.alias_name);
  if (alias_it == overrides.end()) {
    return std::nullopt;
  }
  const auto native_it = overrides.find(native_name);
  if (spec_it->second.offline_fallback) {
    // Terrain values are a fallback only: an explicit native terrain value wins
    // silently and offline=false never changes the live terrain defaults.
    if (native_it != overrides.end() || !OfflineEnabledAtStartup()) {
      return std::nullopt;
    }
  } else if (native_it != overrides.end() &&
    native_it->second != alias_it->second)
  {
    throw std::runtime_error(
            "parameter alias conflict: '" + native_name + "'=" +
            rclcpp::Parameter(native_name, native_it->second).value_to_string() +
            " but alias '" + spec_it->second.alias_name + "'=" +
            rclcpp::Parameter(
              spec_it->second.alias_name,
              alias_it->second).value_to_string());
  }
  ValidateStartupAliasValue(native_name, spec_it->second, alias_it->second);
  return alias_it->second;
}

bool GridGlobalPlanner::OfflineEnabledAtStartup()
{
  const auto & overrides =
    get_node_parameters_interface()->get_parameter_overrides();
  const auto it = overrides.find("offline.enabled");
  if (it == overrides.end()) {
    return false;
  }
  if (it->second.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    throw std::runtime_error(
            "offline.enabled must be a bool parameter, got " +
            rclcpp::to_string(it->second.get_type()));
  }
  return it->second.get<bool>();
}

void GridGlobalPlanner::ApplyDetailLogLevel()
{
  const auto & overrides =
    get_node_parameters_interface()->get_parameter_overrides();
  const auto it = overrides.find("enable_detail_log");
  if (it == overrides.end()) {
    // Without the legacy switch, keep the CLI/environment logger level intact.
    return;
  }
  if (it->second.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    throw std::runtime_error(
            "enable_detail_log must be a bool parameter, got " +
            rclcpp::to_string(it->second.get_type()));
  }
  const bool enabled = it->second.get<bool>();
  const rcutils_ret_t result = rcutils_logging_set_logger_level(
    get_logger().get_name(),
    enabled ? RCUTILS_LOG_SEVERITY_INFO : RCUTILS_LOG_SEVERITY_WARN);
  if (result != RCUTILS_RET_OK) {
    throw std::runtime_error(
            "failed to apply enable_detail_log to the node logger");
  }
  // Keep the consumed compatibility switch visible and immutable, but never
  // map it to diagnostics.detailed_query_timing.
  declare_parameter<bool>(
    "enable_detail_log", enabled, ReadOnlyParameterDescriptor(), true);
}

double GridGlobalPlanner::GetDoubleParameter(
  const std::string & name,
  double default_value)
{
  const auto alias = ResolveStartupAlias(name);
  if (alias) {
    declare_parameter<double>(
      name, alias->get<double>(), ReadOnlyParameterDescriptor(), true);
  } else {
    declare_parameter<double>(
      name, default_value, ReadOnlyParameterDescriptor());
  }
  return get_parameter(name).as_double();
}

std::int64_t GridGlobalPlanner::GetIntParameter(
  const std::string & name, std::int64_t default_value)
{
  const auto alias = ResolveStartupAlias(name);
  if (alias) {
    declare_parameter<std::int64_t>(
      name, alias->get<std::int64_t>(), ReadOnlyParameterDescriptor(), true);
  } else {
    declare_parameter<std::int64_t>(
      name, default_value, ReadOnlyParameterDescriptor());
  }
  return get_parameter(name).as_int();
}

bool GridGlobalPlanner::GetBoolParameter(
  const std::string & name,
  bool default_value)
{
  const auto alias = ResolveStartupAlias(name);
  if (alias) {
    declare_parameter<bool>(
      name, alias->get<bool>(), ReadOnlyParameterDescriptor(), true);
  } else {
    declare_parameter<bool>(
      name, default_value, ReadOnlyParameterDescriptor());
  }
  return get_parameter(name).as_bool();
}

std::string GridGlobalPlanner::GetStringParameter(
  const std::string & name, std::string default_value)
{
  const auto alias = ResolveStartupAlias(name);
  if (alias) {
    declare_parameter<std::string>(
      name, alias->get<std::string>(), ReadOnlyParameterDescriptor(), true);
  } else {
    declare_parameter<std::string>(
      name, default_value, ReadOnlyParameterDescriptor());
  }
  return get_parameter(name).as_string();
}

std::vector<double> GridGlobalPlanner::GetDoubleArrayParameter(
  const std::string & name, std::vector<double> default_value)
{
  declare_parameter<std::vector<double>>(
    name, default_value,
    ReadOnlyParameterDescriptor());
  return get_parameter(name).as_double_array();
}

void GridGlobalPlanner::LoadConfiguration()
{
  RejectUnsupportedStartupAliasGroups(
    get_node_parameters_interface()->get_parameter_overrides());
  ApplyDetailLogLevel();

  // ROS-level names and topics are loaded first because the terrain validation
  // policy and frame checks use them.
  LoadRosAndDiagnosticConfiguration();
  LoadTerrainConfiguration();
  LoadVehicleConfiguration();
  LoadObstacleAndWorkerConfiguration();
  LoadWheelAndSearchConfiguration();
  LoadValidationConfiguration();
  LoadOfflineConfiguration();

  config_.worker.builder = config_.terrain;
  config_.worker.coarse = config_.coarse;
  config_.worker.distance = config_.obstacles;
  ValidateConfiguration();
}

void GridGlobalPlanner::LoadTerrainConfiguration()
{
  auto & terrain = config_.terrain;
  terrain.resolution =
    GetDoubleParameter("terrain.fine_resolution_m", 0.10);
  terrain.origin_x = GetDoubleParameter("terrain.origin_x_m", 0.0);
  terrain.origin_y = GetDoubleParameter("terrain.origin_y_m", 0.0);
  terrain.layer_gap_threshold =
    GetDoubleParameter("terrain.layer_gap_threshold_m", 0.20);
  terrain.refine_split_min_gap =
    GetDoubleParameter("terrain.refine_split_min_gap_m", 0.03);
  terrain.refine_min_rms_improvement =
    GetDoubleParameter("terrain.refine_min_rms_improvement_m", 0.005);
  terrain.max_refine_depth = static_cast<std::size_t>(
    std::max<std::int64_t>(
      0, GetIntParameter("terrain.max_refine_depth", 8)));
  terrain.max_plane_rms =
    GetDoubleParameter("terrain.max_plane_rms_m", 0.05);
  terrain.min_samples_per_surface = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("terrain.min_samples_per_surface", 3)));
  terrain.max_layers_per_cell = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("terrain.max_layers_per_cell", 16)));
  terrain.max_step_height =
    GetDoubleParameter("terrain.max_step_height_m", 0.12);
  terrain.max_slope_rad =
    GetDoubleParameter("terrain.max_slope_rad", 0.60);
  terrain.max_normal_angle_rad =
    GetDoubleParameter("terrain.max_normal_angle_rad", 0.35);
  terrain.min_boundary_coverage =
    GetDoubleParameter("terrain.min_boundary_coverage", 0.25);
  terrain.support_radius_factor =
    GetDoubleParameter("terrain.support_radius_factor", 0.75);
  terrain.mark_excessive_slope_prohibited = GetBoolParameter(
    "terrain.mark_excessive_slope_prohibited", true);

  // The terrain validator uses the monotonic receipt timestamp. Sensor stamps
  // are used for TF and ordering only, never for freshness arithmetic.
  terrain.observation_validation.expected_frame_id = config_.global_frame;
  terrain.observation_validation.require_frame_match = true;
  terrain.observation_validation.max_receipt_age_ns =
    static_cast<std::int64_t>(config_.max_sensor_age_sec * 1e9);
  terrain.observation_validation.future_tolerance_ns =
    static_cast<std::int64_t>(config_.tf_future_tolerance_sec * 1e9);
  terrain.observation_validation.max_points = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("terrain.observation_max_points", 20'000'000)));
  terrain.observation_validation.allow_empty = false;

  config_.coarse.coarse_resolution =
    GetDoubleParameter("terrain.coarse_resolution_m", 0.40);
  config_.coarse.surface_z_tolerance =
    GetDoubleParameter("terrain.coarse_surface_z_tolerance_m", 0.20);
  config_.coarse.max_fine_nodes_per_surface = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter(
        "terrain.coarse_max_fine_nodes_per_surface", 200'000)));
}

void GridGlobalPlanner::LoadVehicleConfiguration()
{
  auto & vehicle = config_.vehicle;

  vehicle.wheelbase_m = GetDoubleParameter("vehicle.wheelbase_m", 0.55);
  vehicle.track_width_m = GetDoubleParameter("vehicle.track_width_m", 0.52);
  vehicle.front_track_width_m =
    GetDoubleParameter("vehicle.front_track_width_m", 0.52);
  vehicle.rear_track_width_m =
    GetDoubleParameter("vehicle.rear_track_width_m", 0.52);
  vehicle.base_contact_height_m =
    GetDoubleParameter("vehicle.base_contact_height_m", 0.22);
  vehicle.max_steer_central_rad =
    GetDoubleParameter(
    "vehicle.max_steer_central_rad",
    0.3270520925102273);
  vehicle.max_steer_inner_rad =
    GetDoubleParameter(
    "vehicle.max_steer_inner_rad",
    0.3839724354387525);

  // The three flat arrays represent the confirmed FL/FR/RL/RR contact offsets.
  // Nested YAML arrays are not valid ROS parameter arrays, so the file uses one
  // homogeneous array per axis.
  const std::vector<double> contact_x = GetDoubleArrayParameter(
    "vehicle.wheel_contacts_x_m", {0.275, 0.275, -0.275, -0.275});
  const std::vector<double> contact_y = GetDoubleArrayParameter(
    "vehicle.wheel_contacts_y_m", {0.26, -0.26, 0.26, -0.26});
  const std::vector<double> contact_z = GetDoubleArrayParameter(
    "vehicle.wheel_contacts_z_m", {-0.22, -0.22, -0.22, -0.22});
  if (contact_x.size() == kWheelCount && contact_y.size() == kWheelCount &&
    contact_z.size() == kWheelCount)
  {
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      vehicle.wheel_contact_offsets[i] =
        Point3{contact_x[i], contact_y[i], contact_z[i]};
    }
  } else {
    std::string detail;
    if (!vehicle.DeriveWheelContactOffsets(&detail)) {
      throw std::runtime_error(
              "vehicle.wheel_contacts_* arrays are invalid: " + detail);
    }
  }

  // The shared /** cuboid is the authoritative body envelope when any of its
  // eight vertices is supplied. Legacy vehicle.body_* values are retained only
  // as the standalone compatibility path when no shared vertex exists.
  static constexpr std::array<const char *, 8> kSharedCuboidVertices{
    "cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt",
    "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  const auto & overrides =
    get_node_parameters_interface()->get_parameter_overrides();
  bool has_shared_cuboid = false;
  for (const char * name : kSharedCuboidVertices) {
    if (overrides.find(name) != overrides.end()) {
      has_shared_cuboid = true;
      break;
    }
  }

  if (has_shared_cuboid) {
    std::array<Vector3, 8> vertices;
    for (std::size_t i = 0; i < kSharedCuboidVertices.size(); ++i) {
      const std::string name = kSharedCuboidVertices[i];
      const auto it = overrides.find(name);
      if (it == overrides.end()) {
        throw std::runtime_error(
                "shared cuboid is incomplete: missing '" + name +
                "' (all eight cuboid.<vertex> parameters are required)");
      }

      std::vector<double> coordinates;
      const auto type = it->second.get_type();
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        coordinates = it->second.get<std::vector<double>>();
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto integers = it->second.get<std::vector<std::int64_t>>();
        coordinates.assign(integers.begin(), integers.end());
      } else {
        throw std::runtime_error(
                "shared cuboid vertex '" + name +
                "' must be a numeric [x, y, z] array, got " +
                rclcpp::to_string(type));
      }
      if (coordinates.size() != 3) {
        throw std::runtime_error(
                "shared cuboid vertex '" + name +
                "' must contain exactly 3 values, got " +
                std::to_string(coordinates.size()));
      }
      for (const double coordinate : coordinates) {
        if (!std::isfinite(coordinate)) {
          throw std::runtime_error(
                  "shared cuboid vertex '" + name +
                  "' contains a non-finite value");
        }
      }

      vertices[i] =
        Vector3{coordinates[0], coordinates[1], coordinates[2]};
      // Keep every consumed shared vertex visible and immutable at runtime.
      declare_parameter<std::vector<double>>(
        name, coordinates, ReadOnlyParameterDescriptor(), true);
    }

    Vector3 body_min = vertices.front();
    Vector3 body_max = vertices.front();
    for (const auto & vertex : vertices) {
      body_min.x = std::min(body_min.x, vertex.x);
      body_min.y = std::min(body_min.y, vertex.y);
      body_min.z = std::min(body_min.z, vertex.z);
      body_max.x = std::max(body_max.x, vertex.x);
      body_max.y = std::max(body_max.y, vertex.y);
      body_max.z = std::max(body_max.z, vertex.z);
    }
    if (!(body_max.x > body_min.x) ||
      !(body_max.y > body_min.y) ||
      !(body_max.z > body_min.z))
    {
      throw std::runtime_error(
              "shared cuboid bounds must be strictly positive on x, y, and z");
    }

    vehicle.body_min = body_min;
    vehicle.body_min_configured = true;
    vehicle.body_min_approximate = false;
    vehicle.body_max = body_max;
    vehicle.body_max_configured = true;

    // The native names are the ROS-visible effective envelope. Ignore their
    // stale overrides and publish the values actually derived from the cuboid.
    declare_parameter<std::vector<double>>(
      "vehicle.body_min_m_approximate",
      {body_min.x, body_min.y, body_min.z},
      ReadOnlyParameterDescriptor(), true);
    declare_parameter<bool>(
      "vehicle.body_min_configured", true, ReadOnlyParameterDescriptor(), true);
    declare_parameter<bool>(
      "vehicle.body_min_is_approximate", false,
      ReadOnlyParameterDescriptor(), true);
    declare_parameter<std::vector<double>>(
      "vehicle.body_max_m",
      {body_max.x, body_max.y, body_max.z},
      ReadOnlyParameterDescriptor(), true);
    declare_parameter<bool>(
      "vehicle.body_max_configured", true, ReadOnlyParameterDescriptor(), true);
  } else {
    const std::vector<double> body_min = GetDoubleArrayParameter(
      "vehicle.body_min_m_approximate", {-0.41, -0.32, -0.10});
    if (body_min.size() == 3) {
      vehicle.body_min = Vector3{body_min[0], body_min[1], body_min[2]};
    }
    vehicle.body_min_configured =
      GetBoolParameter("vehicle.body_min_configured", true);
    vehicle.body_min_approximate =
      GetBoolParameter("vehicle.body_min_is_approximate", true);

    const std::vector<double> body_max = GetDoubleArrayParameter(
      "vehicle.body_max_m",
      {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()});
    vehicle.body_max_configured =
      GetBoolParameter("vehicle.body_max_configured", false);
    if (vehicle.body_max_configured && body_max.size() == 3) {
      vehicle.body_max = Vector3{body_max[0], body_max[1], body_max[2]};
    }
  }

  // .nan in the YAML is an explicit UNSET marker. It is never converted to a
  // guessed physical value; the evaluator fails closed while any required
  // measured value remains NaN.
  vehicle.wheel_radius_m =
    GetDoubleParameter(
    "vehicle.wheel_radius_m",
    std::numeric_limits<double>::quiet_NaN());
  vehicle.tire_width_m =
    GetDoubleParameter(
    "vehicle.tire_width_m",
    std::numeric_limits<double>::quiet_NaN());
  vehicle.suspension_travel_m =
    GetDoubleParameter(
    "vehicle.suspension_travel_m",
    std::numeric_limits<double>::quiet_NaN());
  vehicle.tire_compliance_m =
    GetDoubleParameter(
    "vehicle.tire_compliance_m",
    std::numeric_limits<double>::quiet_NaN());
  vehicle.contact_patch_length_m =
    GetDoubleParameter(
    "vehicle.contact_patch_length_m",
    std::numeric_limits<double>::quiet_NaN());
  config_.vehicle_capability_declared_configured =
    GetBoolParameter("vehicle.vehicle_capability_configured", false);

  vehicle.min_contact_coverage =
    GetDoubleParameter("vehicle.min_contact_coverage", 0.60);
  vehicle.max_pitch_rad =
    GetDoubleParameter("vehicle.max_pitch_rad", 0.40);
  vehicle.max_roll_rad =
    GetDoubleParameter("vehicle.max_roll_rad", 0.30);
  vehicle.max_step_m = GetDoubleParameter("vehicle.max_step_m", 0.12);
  vehicle.max_warp_m = GetDoubleParameter("vehicle.max_warp_m", 0.05);
  vehicle.max_pose_residual_m =
    GetDoubleParameter("vehicle.max_pose_residual_m", 0.05);
  vehicle.body_clearance_m =
    GetDoubleParameter("vehicle.body_clearance_m", 0.12);
  vehicle.max_branch_jump_m =
    GetDoubleParameter("vehicle.max_branch_jump_m", 0.12);
}

void GridGlobalPlanner::LoadObstacleAndWorkerConfiguration()
{
  config_.obstacles.truncation_distance_m =
    GetDoubleParameter("obstacles.truncation_distance_m", 2.0);
  config_.obstacles.reject_unknown =
    GetBoolParameter("obstacles.reject_unknown", true);
  config_.obstacles.reject_prohibited =
    GetBoolParameter("obstacles.reject_prohibited", true);

  config_.worker.source_ttl_s =
    GetDoubleParameter("worker.source_ttl_s", 2.0);
  // Dynamic sources expire to unknown; static latched map contributions are
  // immortal unless an operator explicitly configures a finite positive TTL.
  // A non-positive static TTL means "remove only by explicit source removal".
  config_.static_source_ttl_sec = GetDoubleParameter(
    "worker.static_source_ttl_s", -1.0);
  config_.dynamic_source_ttl_sec = GetDoubleParameter(
    "worker.dynamic_source_ttl_s", config_.worker.source_ttl_s);
  config_.worker.static_source_ttl_s = config_.static_source_ttl_sec;
  config_.worker.source_ttl_s = config_.dynamic_source_ttl_sec;
  config_.worker.max_pending_batches = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.max_pending_batches", 4)));
  config_.worker.retained_snapshots = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.retained_snapshots", 4)));
  config_.max_retired_snapshots = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.max_retired_versions", 64)));
  config_.worker.max_sources = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.max_sources", 1024)));
  config_.worker.max_records_total = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.max_records_total", 1'000'000)));
  config_.worker.max_records_per_source = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("worker.max_records_per_source", 100'000)));
  // A nonzero production publication cap batches dirty updates instead of
  // publishing one snapshot per accepted event.
  config_.worker.publish_period_s =
    GetDoubleParameter("worker.publish_period_s", 0.05);
  config_.worker.max_publish_hz =
    GetDoubleParameter("worker.max_publish_hz", 20.0);
  // The association envelope is explicit observed/configured geometry. NaN or
  // non-positive disables non-overlap association, leaving such slabs unknown
  // rather than inventing a vehicle envelope.
  config_.worker.association_min_clearance_m =
    GetDoubleParameter("worker.association_min_clearance_m", 0.0);
  config_.worker.association_max_clearance_m = GetDoubleParameter(
    "worker.association_max_clearance_m",
    std::numeric_limits<double>::quiet_NaN());
  config_.worker.reject_unknown_sources =
    GetBoolParameter("worker.reject_unknown_sources", true);
}

void GridGlobalPlanner::LoadWheelAndSearchConfiguration()
{
  auto & wheel = config_.wheel;
  const std::string model =
    GetStringParameter("wheel_support.evaluation_model", "full_wheel");
  if (model == "full_wheel") {
    wheel.model = TraversabilityModel::kFullWheel;
  } else if (model == "offline_kinematic_static_map") {
    wheel.model = TraversabilityModel::kOfflineKinematicStaticMap;
  } else {
    throw std::runtime_error(
            "wheel_support.evaluation_model must be 'full_wheel' or "
            "'offline_kinematic_static_map', got '" + model + "'");
  }
  wheel.offline_planar_collision_radius_m = GetDoubleParameter(
    "wheel_support.offline_planar_collision_radius_m", 0.52);
  wheel.offline_body_sample_spacing_m = GetDoubleParameter(
    "wheel_support.offline_body_sample_spacing_m", 0.12);
  wheel.offline_contact_height_tolerance_m = GetDoubleParameter(
    "wheel_support.offline_contact_height_tolerance_m", 0.10);
  // Explicit, temporary opt-out of whole-vehicle ground support for the
  // offline model only. Default true preserves the nominal hard gates; the
  // sole static-PCD YAML opts out. Obstacles/body collision, pitch/roll, raised
  // terrain and center terrain remain enforced when false.
  wheel.offline_check_ground_support = GetBoolParameter(
    "wheel_support.offline_check_ground_support", true);
  wheel.footprint_samples_along = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("wheel_support.footprint_samples_along", 5)));
  wheel.footprint_samples_across = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("wheel_support.footprint_samples_across", 3)));
  wheel.max_branch_combinations = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("wheel_support.max_branch_combinations", 256)));
  wheel.max_pose_iterations = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("wheel_support.max_pose_iterations", 10)));
  wheel.branch_height_tolerance_m =
    GetDoubleParameter("wheel_support.branch_height_tolerance_m", 0.03);
  wheel.min_center_support =
    GetDoubleParameter("wheel_support.min_center_support", 1.0);
  wheel.max_contact_height_spread_m = GetDoubleParameter(
    "wheel_support.max_contact_height_spread_m", 0.05);
  wheel.max_deck_search_nodes = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("wheel_support.max_deck_search_nodes", 4096)));
  wheel.body_sample_spacing_m =
    GetDoubleParameter("wheel_support.body_sample_spacing_m", 0.10);
  wheel.obstacle_penalty_distance_m =
    GetDoubleParameter("wheel_support.obstacle_penalty_distance_m", 0.50);
  wheel.obstacle_penalty_weight =
    GetDoubleParameter("wheel_support.obstacle_penalty_weight", 1.0);
  wheel.slope_penalty_weight =
    GetDoubleParameter("wheel_support.slope_penalty_weight", 1.0);
  wheel.residual_penalty_weight =
    GetDoubleParameter("wheel_support.residual_penalty_weight", 10.0);
  wheel.clearance_penalty_weight =
    GetDoubleParameter("wheel_support.clearance_penalty_weight", 1.0);
  // Offline-only soft terrain-boundary cost, also consumed by the terrain
  // heat display. Startup-only read; zero disables the term explicitly.
  wheel.boundary_penalty_distance_m =
    GetDoubleParameter("wheel_support.boundary_penalty_distance_m", 0.50);
  wheel.boundary_penalty_weight =
    GetDoubleParameter("wheel_support.boundary_penalty_weight", 1.0);

  auto & search = config_.search;
  config_.search_backend =
    GetStringParameter("raw_route_backend", "hybrid_a_star");
  if (config_.search_backend != "graph_a_star" &&
    config_.search_backend != "hybrid_a_star")
  {
    throw std::runtime_error(
            "raw_route_backend must be 'graph_a_star' or 'hybrid_a_star', got '" +
            config_.search_backend + "'");
  }
  search.primitive_length_m =
    GetDoubleParameter("hybrid_a_star.primitive_length_m", 0.80);
  search.steer_samples = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("hybrid_a_star.steer_samples", 5)));
  search.allow_reverse =
    GetBoolParameter("hybrid_a_star.allow_reverse", false);
  search.max_wheel_sample_spacing_m =
    GetDoubleParameter("hybrid_a_star.max_wheel_sample_spacing_m", 0.05);
  search.max_yaw_sample_step_rad =
    GetDoubleParameter("hybrid_a_star.max_yaw_sample_step_rad", 0.10);
  search.goal_position_tolerance_m =
    GetDoubleParameter("hybrid_a_star.goal_position_tolerance_m", 0.30);
  search.goal_heading_tolerance_rad =
    GetDoubleParameter("hybrid_a_star.goal_heading_tolerance_rad", 3.14);
  search.goal_connection_max_length_m =
    GetDoubleParameter("hybrid_a_star.goal_connection_max_length_m", 3.0);
  search.goal_connection_samples = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("hybrid_a_star.goal_connection_samples", 12)));
  search.use_coarse_guidance =
    GetBoolParameter("terrain.use_coarse_guidance", true);
  search.max_coarse_routes = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("hybrid_a_star.max_coarse_routes", 2)));
  search.reverse_penalty_weight =
    GetDoubleParameter("hybrid_a_star.reverse_penalty_weight", 0.5);
  search.steering_penalty_weight =
    GetDoubleParameter("hybrid_a_star.steering_penalty_weight", 0.1);
  search.terrain_cost_weight =
    GetDoubleParameter("hybrid_a_star.terrain_cost_weight", 0.2);
  search.heading_bin_rad =
    GetDoubleParameter("hybrid_a_star.heading_bin_rad", 0.19634954084936207);
  search.corridor.initial_margin_m =
    GetDoubleParameter("hybrid_a_star.corridor_initial_margin_m", 0.20);
  search.corridor.margin_step_m =
    GetDoubleParameter("hybrid_a_star.corridor_margin_step_m", 0.20);
  search.corridor.max_enlargements = static_cast<std::size_t>(
    std::max<std::int64_t>(
      0, GetIntParameter("hybrid_a_star.corridor_max_enlargements", 3)));
  search.corridor.max_corridor_cells = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter(
        "hybrid_a_star.corridor_max_corridor_cells",
        2'000'000)));

  config_.max_samples = std::max<std::size_t>(
    1, CheckedNonNegativeSizeParameter(
      GetIntParameter("graph_a_star.max_samples", 2'000'000),
      "graph_a_star.max_samples"));

  // Startup-only soft-preference weights. Zero is an explicit disable, while
  // negative or non-finite values are configuration errors. Build the options
  // locally so the persistent cache is constructed once, after validation.
  const auto load_graph_penalty = [this](
    const char * name, double default_value) {
      const double value = GetDoubleParameter(name, default_value);
      if (!std::isfinite(value) || value < 0.0) {
        throw std::runtime_error(
                std::string(name) + " must be finite and non-negative");
      }
      return value;
    };
  // Load all six graph soft-preference values before constructing the
  // persistent cache, and retain that exact value copy for the terrain display.
  config_.graph = GraphSearchOptions{};
  config_.graph.slope_penalty_weight = load_graph_penalty(
    "graph_a_star.slope_penalty_weight", 0.0);
  config_.graph.roughness_penalty_weight = load_graph_penalty(
    "graph_a_star.roughness_penalty_weight", 10.0);
  config_.graph.obstacle_penalty_distance_m = load_graph_penalty(
    "graph_a_star.obstacle_penalty_distance_m", 0.75);
  config_.graph.obstacle_penalty_weight = load_graph_penalty(
    "graph_a_star.obstacle_penalty_weight", 0.2);
  config_.graph.boundary_penalty_distance_m = load_graph_penalty(
    "graph_a_star.boundary_penalty_distance_m", 0.8);
  config_.graph.boundary_penalty_weight = load_graph_penalty(
    "graph_a_star.boundary_penalty_weight", 3.0);
  graph_search_ = GridGraphAStar(config_.graph);
}

void GridGlobalPlanner::LoadValidationConfiguration()
{
  auto & validation = config_.validation;
  validation.max_samples = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("validation.max_samples", 100'000)));
  validation.max_wheel_sample_spacing_m =
    GetDoubleParameter("validation.max_wheel_sample_spacing_m", 0.05);
  validation.max_yaw_sample_step_rad =
    GetDoubleParameter("validation.max_yaw_sample_step_rad", 0.10);
  validation.changed_pose_tolerance_m =
    GetDoubleParameter("validation.changed_pose_tolerance_m", 0.03);
  validation.segment_endpoint_tolerance_m =
    GetDoubleParameter("validation.segment_endpoint_tolerance_m", 1.0e-3);
  validation.segment_yaw_tolerance_rad =
    GetDoubleParameter("validation.segment_yaw_tolerance_rad", 1.0e-3);
  validation.require_guaranteed_clearance =
    GetBoolParameter("validation.require_guaranteed_clearance", true);
  validation.reject_unknown_obstacles =
    GetBoolParameter("validation.reject_unknown_obstacles", true);

  config_.max_latest_validation_retries = static_cast<int>(
    std::max<std::int64_t>(
      1, GetIntParameter("validation.max_latest_validation_retries", 4)));
  config_.corridor_tile_size_m =
    GetDoubleParameter("validation.corridor_tile_size_m", 3.2);
  config_.corridor_max_samples = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("validation.corridor_max_samples", 50'000)));
  config_.body_sweep_spacing_m =
    GetDoubleParameter("validation.body_sweep_spacing_m", 0.10);

  config_.max_planning_time_sec =
    GetDoubleParameter("planning.max_planning_time_sec", 10.0);
  config_.max_latest_validation_time_sec =
    GetDoubleParameter("planning.max_latest_validation_time_sec", 2.0);
}

void GridGlobalPlanner::LoadOfflineConfiguration()
{
  auto & offline = config_.offline;
  offline.enabled = GetBoolParameter("offline.enabled", false);
  // Declare/read the public switch even when offline input is disabled so its
  // ROS-visible value is always the configured value, never a late default.
  offline.use_global_as_static_obstacles =
    GetBoolParameter("offline.use_global_as_static_obstacles", false);
  if (!offline.enabled) {
    return;
  }
  // Default to the legacy ground-only build; global mode is explicit opt-in.
  // Map data arrives through the latched ROS topics below. No map file path is
  // declared, validated, or read by the operational planner.
  offline.global_cloud_mode =
    GetBoolParameter("offline.global_cloud_mode", false);
  auto & map = offline.map;
  map.resolution = GetDoubleParameter(
    "offline.resolution_m", config_.terrain.resolution);
  map.origin_x = GetDoubleParameter(
    "offline.origin_x_m", config_.terrain.origin_x);
  map.origin_y = GetDoubleParameter(
    "offline.origin_y_m", config_.terrain.origin_y);
  map.neighborhood_radius_m =
    GetDoubleParameter("offline.neighborhood_radius_m", 0.35);
  map.max_sampling_gap_m =
    GetDoubleParameter("offline.max_sampling_gap_m", 0.30);
  map.max_plane_rms_m = GetDoubleParameter(
    "offline.max_plane_rms_m", config_.terrain.max_plane_rms);
  map.max_normal_angle_rad = GetDoubleParameter(
    "offline.max_normal_angle_rad", config_.terrain.max_normal_angle_rad);
  map.max_slope_rad = GetDoubleParameter(
    "offline.max_slope_rad", config_.terrain.max_slope_rad);
  map.max_step_height_m = GetDoubleParameter(
    "offline.max_step_height_m", config_.terrain.max_step_height);
  map.layer_gap_threshold_m = GetDoubleParameter(
    "offline.layer_gap_threshold_m", config_.terrain.layer_gap_threshold);
  map.min_boundary_coverage = GetDoubleParameter(
    "offline.min_boundary_coverage", config_.terrain.min_boundary_coverage);
  map.min_neighborhood_points = static_cast<std::size_t>(
    std::max<std::int64_t>(
      3, GetIntParameter("offline.min_neighborhood_points", 4)));
  map.max_candidate_cells = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("offline.max_candidate_cells", 2'000'000)));
  map.evidence_expansion_step_m = GetDoubleParameter(
    "offline.evidence_expansion_step_m", 0.10);
  map.global_surface_expansion_max_radius_m = GetDoubleParameter(
    "offline.global_surface_expansion_max_radius_m", 0.55);
  map.global_surface_min_angular_coverage = GetDoubleParameter(
    "offline.global_surface_min_angular_coverage", 0.50);
  map.ground_duplicate_tolerance_m =
    GetDoubleParameter("offline.ground_duplicate_tolerance_m", 0.08);
  map.obstacle_blocking_height_m =
    GetDoubleParameter("offline.obstacle_blocking_height_m", 1.00);
  map.obstacle_association_radius_m =
    GetDoubleParameter("offline.obstacle_association_radius_m", 0.25);
  map.obstacle_tile_size_m =
    GetDoubleParameter("offline.obstacle_tile_size_m", 3.2);
  map.global_ground_seed_tolerance_m = GetDoubleParameter(
    "offline.global_ground_seed_tolerance_m", 0.15);
  map.global_hint_surface_tolerance_m = GetDoubleParameter(
    "offline.global_hint_surface_tolerance_m", 0.08);
  // Ground-only bounded continuity pass. Disabled at the API level so legacy
  // callers keep the exact previous construction; the checked-in ground-only
  // profile enables it explicitly. Global-cloud classification never runs it.
  auto & ground_surface = map.ground_surface;
  ground_surface.enabled =
    GetBoolParameter("offline.ground_surface.enabled", false);
  ground_surface.max_height_noise_m = GetDoubleParameter(
    "offline.ground_surface.max_height_noise_m", 0.06);
  ground_surface.max_correction_m = GetDoubleParameter(
    "offline.ground_surface.max_correction_m", 0.06);
  ground_surface.max_normal_correction_rad = GetDoubleParameter(
    "offline.ground_surface.max_normal_correction_rad", 0.15);
  ground_surface.max_association_normal_rad = GetDoubleParameter(
    "offline.ground_surface.max_association_normal_rad", 0.50);
  ground_surface.max_support_residual_m = GetDoubleParameter(
    "offline.ground_surface.max_support_residual_m", 0.06);
  ground_surface.max_candidate_slope_rad = GetDoubleParameter(
    "offline.ground_surface.max_candidate_slope_rad", 0.60);
  ground_surface.max_neighbors = CheckedPositiveSizeParameter(
    GetIntParameter("offline.ground_surface.max_neighbors", 8),
    "offline.ground_surface.max_neighbors");
  ground_surface.max_points_per_node = CheckedPositiveSizeParameter(
    GetIntParameter("offline.ground_surface.max_points_per_node", 12),
    "offline.ground_surface.max_points_per_node");

  // Regional same-layer reconstruction. This is independent of the legacy
  // bounded smoother above: enabling it supersedes that pass and reconstructs
  // real scanline gaps from the checked-in ground map.
  auto & continuous_ground = map.continuous_ground;
  continuous_ground.enabled =
    GetBoolParameter("offline.continuous_ground.enabled", false);
  continuous_ground.regional_radius_m = GetDoubleParameter(
    "offline.continuous_ground.regional_radius_m", 1.00);
  continuous_ground.max_gap_fill_m = GetDoubleParameter(
    "offline.continuous_ground.max_gap_fill_m", 1.00);
  continuous_ground.max_plane_rms_m = GetDoubleParameter(
    "offline.continuous_ground.max_plane_rms_m", 0.15);
  continuous_ground.max_inlier_residual_m = GetDoubleParameter(
    "offline.continuous_ground.max_inlier_residual_m", 0.18);
  continuous_ground.robust_noise_floor_m = GetDoubleParameter(
    "offline.continuous_ground.robust_noise_floor_m", 0.03);
  continuous_ground.min_region_confidence = GetDoubleParameter(
    "offline.continuous_ground.min_region_confidence", 0.35);
  continuous_ground.min_angular_coverage = GetDoubleParameter(
    "offline.continuous_ground.min_angular_coverage", 0.35);
  continuous_ground.corner_association_tolerance_m = GetDoubleParameter(
    "offline.continuous_ground.corner_association_tolerance_m", 0.10);
  continuous_ground.max_association_normal_rad = GetDoubleParameter(
    "offline.continuous_ground.max_association_normal_rad", 0.50);
  continuous_ground.min_region_points = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.min_region_points", 6),
    "offline.continuous_ground.min_region_points");
  continuous_ground.max_region_points = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.max_region_points", 768),
    "offline.continuous_ground.max_region_points");
  continuous_ground.max_layers_per_cell = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.max_layers_per_cell", 2),
    "offline.continuous_ground.max_layers_per_cell");
  continuous_ground.max_candidate_seeds = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.max_candidate_seeds", 256),
    "offline.continuous_ground.max_candidate_seeds");
  continuous_ground.max_support_refs_per_cell = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.max_support_refs_per_cell", 256),
    "offline.continuous_ground.max_support_refs_per_cell");
  continuous_ground.max_inferred_cells = CheckedPositiveSizeParameter(
    GetIntParameter("offline.continuous_ground.max_inferred_cells", 2'000'000),
    "offline.continuous_ground.max_inferred_cells");

  // The static-cloud entry must not wait for live
  // lidar, preprocessed nonground, source observations or a
  // live TF. The action still uses explicit map-frame start/goal poses. This is
  // a wiring override for this profile only, not a change to the live defaults.
  // The in-process frozen perception export is the one optional exception: it is
  // preserved only when a provider was supplied, so the standalone static demo
  // (no provider) keeps the exact disabled behavior.
  config_.enable_ground_cloud = false;
  config_.enable_perception_export =
    config_.enable_perception_export && static_cast<bool>(scene_snapshot_provider_);
  if (config_.enable_perception_export &&
    config_.vehicle.body_max_configured)
  {
    // The shared cuboid supplies the real vehicle top. A lidar return inside
    // that vertical envelope blocks the supporting terrain for graph search;
    // high overhead returns must not be flattened onto a lower road.
    config_.worker.association_blocking_height_m =
      config_.vehicle.base_contact_height_m + config_.vehicle.body_max.z;
  }
  config_.enable_source_observation = false;
  config_.enable_raw_cloud_fallback = false;
  config_.enable_static_obstacle_cloud = false;
  config_.enable_dynamic_obstacle_cloud = false;
}

bool GridGlobalPlanner::UseGlobalAsStaticObstacles() const noexcept
{
  return !config_.offline.global_cloud_mode &&
         config_.offline.use_global_as_static_obstacles;
}

void GridGlobalPlanner::LoadRosAndDiagnosticConfiguration()
{
  config_.global_frame = GetStringParameter("ros.global_frame", "map");
  config_.robot_base_frame =
    GetStringParameter("ros.robot_base_frame", "base_link");

  config_.enable_ground_cloud =
    GetBoolParameter("ros.enable_ground_cloud", !OfflineEnabledAtStartup());
  config_.enable_static_obstacle_cloud =
    GetBoolParameter("ros.enable_static_obstacle_cloud", !OfflineEnabledAtStartup());
  config_.enable_dynamic_obstacle_cloud =
    GetBoolParameter("ros.enable_dynamic_obstacle_cloud", !OfflineEnabledAtStartup());
  // Raw clouds are an explicit fallback only. The default is false so the
  // shared dynamic/static cloud is never silently duplicated alongside the
  // explicit SourceObservation evidence.
  config_.enable_raw_cloud_fallback =
    GetBoolParameter("ros.enable_raw_cloud_fallback", false);
  config_.enable_source_observation =
    GetBoolParameter("ros.enable_source_observation", !OfflineEnabledAtStartup());
  config_.enable_perception_export =
    GetBoolParameter("ros.enable_perception_export", !OfflineEnabledAtStartup());
  config_.ground_cloud_topic =
    GetStringParameter("ros.ground_cloud_topic", "mapground");
  config_.static_obstacle_cloud_topic =
    GetStringParameter("ros.static_obstacle_cloud_topic", "mapcloud");
  config_.dynamic_obstacle_cloud_topic = GetStringParameter(
    "ros.dynamic_obstacle_cloud_topic", "navigation/preprocessed_cloud");
  config_.source_observation_topic = GetStringParameter(
    "ros.source_observation_topic", "~/source_observation");
  config_.perception_publish_rate_hz =
    GetDoubleParameter("ros.perception_publish_rate_hz", 10.0);
  config_.max_source_queue = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("ros.max_source_queue", 4)));
  config_.max_sources = static_cast<std::size_t>(
    std::max<std::int64_t>(1, GetIntParameter("ros.max_sources", 64)));
  config_.max_source_records = static_cast<std::size_t>(
    std::max<std::int64_t>(
      1, GetIntParameter("ros.max_source_records", 200'000)));

  config_.tf_timeout_sec =
    GetDoubleParameter("ros.tf_timeout_sec", 0.20);
  config_.tf_future_tolerance_sec =
    GetDoubleParameter("ros.tf_future_tolerance_sec", 0.50);
  config_.max_sensor_age_sec =
    GetDoubleParameter("ros.max_sensor_age_sec", 2.0);
  config_.allow_latest_tf_fallback =
    GetBoolParameter("ros.allow_latest_tf_fallback", true);
  config_.source_expiry_period_sec =
    GetDoubleParameter("ros.source_expiry_period_sec", 1.0);

  config_.publish_stage_debug =
    GetBoolParameter("diagnostics.publish_stage_debug", true);
  config_.publish_wheel_debug =
    GetBoolParameter("diagnostics.publish_wheel_debug", false);
  config_.publish_terrain_debug =
    GetBoolParameter("diagnostics.publish_terrain_debug", true);
  config_.publish_terrain_cost_debug =
    GetBoolParameter("diagnostics.publish_terrain_cost_debug", true);
  config_.publish_coarse_debug =
    GetBoolParameter("diagnostics.publish_coarse_debug", true);
  config_.publish_corridor_debug =
    GetBoolParameter("diagnostics.publish_corridor_debug", true);
  config_.publish_cost_debug =
    GetBoolParameter("diagnostics.publish_cost_debug", true);
  config_.max_visualization_points = static_cast<std::size_t>(
    std::max<std::int64_t>(
      0, GetIntParameter(
        "diagnostics.max_visualization_points",
        200'000)));
  config_.max_measurement_visualization_patches =
    CheckedNonNegativeSizeParameter(
    GetIntParameter(
      "diagnostics.max_measurement_visualization_patches", 20'000),
    "diagnostics.max_measurement_visualization_patches");
  config_.max_corridor_visualization_cells = static_cast<std::size_t>(
    std::max<std::int64_t>(
      0, GetIntParameter(
        "diagnostics.max_corridor_visualization_cells",
        20'000)));

  config_.path_topic = GetStringParameter("ros.path_topic", "~/path");
  config_.terrain_topic =
    GetStringParameter("ros.terrain_topic", "~/terrain");
  config_.coarse_topic = GetStringParameter("ros.coarse_topic", "~/coarse");
  config_.corridor_topic =
    GetStringParameter("ros.corridor_topic", "~/corridor");
  config_.cost_topic = GetStringParameter("ros.cost_topic", "~/cost");
  config_.diagnostics_topic =
    GetStringParameter("ros.diagnostics_topic", "~/diagnostics");
  config_.version_topic =
    GetStringParameter("ros.version_topic", "~/version");
  config_.fallback_topic =
    GetStringParameter("ros.fallback_topic", "~/fallback");
  config_.wheel_debug_topic =
    GetStringParameter("ros.wheel_debug_topic", "~/wheel_support");
  config_.metrics_topic =
    GetStringParameter("ros.metrics_topic", "~/metrics");
  config_.planner_stage_topic = GetStringParameter(
    "ros.planner_stage_topic", "/debug/planner_route_request_stage");
  config_.detailed_query_timing =
    GetBoolParameter("diagnostics.detailed_query_timing", false);
}

void GridGlobalPlanner::ValidateConfiguration()
{
  if (config_.global_frame.empty() || config_.robot_base_frame.empty()) {
    throw std::runtime_error("ros.global_frame/robot_base_frame must be set");
  }
  if (config_.global_frame == config_.robot_base_frame) {
    throw std::runtime_error(
            "ros.global_frame must differ from ros.robot_base_frame");
  }
  if (!(config_.max_planning_time_sec > 0.0) ||
    !std::isfinite(config_.max_planning_time_sec))
  {
    throw std::runtime_error(
            "planning.max_planning_time_sec must be finite and positive");
  }
  if (!(config_.max_latest_validation_time_sec >= 0.0) ||
    !std::isfinite(config_.max_latest_validation_time_sec))
  {
    throw std::runtime_error(
            "planning.max_latest_validation_time_sec must be finite and >= 0");
  }
  if (config_.max_latest_validation_retries < 1) {
    throw std::runtime_error(
            "validation.max_latest_validation_retries must be >= 1");
  }
  if (!(config_.corridor_tile_size_m > 0.0) ||
    !std::isfinite(config_.corridor_tile_size_m))
  {
    throw std::runtime_error(
            "validation.corridor_tile_size_m must be finite and positive");
  }
  if (!(config_.body_sweep_spacing_m > 0.0) ||
    !std::isfinite(config_.body_sweep_spacing_m))
  {
    throw std::runtime_error(
            "validation.body_sweep_spacing_m must be finite and positive");
  }
  if (!(config_.tf_timeout_sec >= 0.0) ||
    !(config_.tf_future_tolerance_sec >= 0.0) ||
    !(config_.max_sensor_age_sec > 0.0))
  {
    throw std::runtime_error(
            "ros TF/sensor age parameters must be finite and non-negative");
  }
  if (config_.offline.enabled && config_.offline.global_cloud_mode &&
    config_.ground_cloud_topic == config_.static_obstacle_cloud_topic)
  {
    throw std::runtime_error(
            "ros.ground_cloud_topic and ros.static_obstacle_cloud_topic must "
            "differ in offline.global_cloud_mode");
  }

  std::string detail;
  TerrainGridBuilder terrain_builder(config_.terrain);
  if (!terrain_builder.ValidateOptions(&detail)) {
    throw std::runtime_error("terrain configuration invalid: " + detail);
  }
  CoarseTerrainMapBuilder coarse_builder(config_.coarse);
  if (!coarse_builder.ValidateOptions(&detail)) {
    throw std::runtime_error("coarse configuration invalid: " + detail);
  }
  CorridorBuilder corridor_builder(config_.search.corridor);
  if (!corridor_builder.ValidateOptions(&detail)) {
    throw std::runtime_error("corridor configuration invalid: " + detail);
  }

  VehicleConfigurationStatus vehicle_status =
    config_.vehicle.Validate(&detail);
  if (vehicle_status != VehicleConfigurationStatus::kConfigured &&
    vehicle_status !=
    VehicleConfigurationStatus::kVehicleCapabilityUnconfigured)
  {
    throw std::runtime_error("vehicle configuration invalid: " + detail);
  }

  if (!(config_.obstacles.truncation_distance_m > 0.0) ||
    !std::isfinite(config_.obstacles.truncation_distance_m))
  {
    throw std::runtime_error(
            "obstacles.truncation_distance_m must be finite and positive");
  }
  if (!(config_.worker.source_ttl_s >= 0.0) ||
    !std::isfinite(config_.worker.source_ttl_s))
  {
    throw std::runtime_error(
            "worker.dynamic_source_ttl_s must be finite and >= 0");
  }
  if (!std::isfinite(config_.static_source_ttl_sec) ||
    !(config_.dynamic_source_ttl_sec >= 0.0) ||
    !std::isfinite(config_.dynamic_source_ttl_sec))
  {
    throw std::runtime_error(
            "worker.static_source_ttl_s must be finite (<= 0 means immortal) and "
            "worker.dynamic_source_ttl_s must be finite and >= 0");
  }
  if (config_.max_source_queue == 0 || config_.max_sources == 0 ||
    config_.max_source_records == 0)
  {
    throw std::runtime_error(
            "ros.max_source_queue/max_sources/max_source_records must be >= 1");
  }
  if (!(config_.perception_publish_rate_hz > 0.0) ||
    !std::isfinite(config_.perception_publish_rate_hz))
  {
    throw std::runtime_error(
            "ros.perception_publish_rate_hz must be finite and positive");
  }
  if (config_.worker.max_pending_batches == 0 ||
    config_.worker.retained_snapshots == 0 ||
    config_.max_retired_snapshots == 0 ||
    config_.worker.max_sources == 0 ||
    config_.worker.max_records_total == 0 ||
    config_.worker.max_records_per_source == 0)
  {
    throw std::runtime_error(
            "worker queue/source/record/retired bounds must be >= 1");
  }
  if (!(config_.worker.publish_period_s >= 0.0) ||
    !std::isfinite(config_.worker.publish_period_s) ||
    !(config_.worker.max_publish_hz >= 0.0) ||
    !std::isfinite(config_.worker.max_publish_hz) ||
    (config_.worker.publish_period_s <= 0.0 &&
    config_.worker.max_publish_hz <= 0.0))
  {
    throw std::runtime_error(
            "worker.publish_period_s/max_publish_hz must be finite and at least "
            "one must be positive");
  }
  if (!(config_.worker.association_min_clearance_m >= 0.0) ||
    !std::isfinite(config_.worker.association_min_clearance_m))
  {
    throw std::runtime_error(
            "worker.association_min_clearance_m must be finite and >= 0");
  }
  const double association_max =
    config_.worker.association_max_clearance_m;
  if (!std::isnan(association_max) &&
    (!std::isfinite(association_max) || association_max <= 0.0))
  {
    throw std::runtime_error(
            "worker.association_max_clearance_m must be NaN (disabled) or finite "
            "and positive");
  }
}

// ---------------------------------------------------------------------------
// ROS wiring
// ---------------------------------------------------------------------------

void GridGlobalPlanner::CreateRosInterfaces()
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  path_publisher_ =
    create_publisher<nav_msgs::msg::Path>(config_.path_topic, latched_qos);
  terrain_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(config_.terrain_topic, latched_qos);
  if (!config_.publish_terrain_debug) {
    // The display is startup-disabled. Clear geometry left on this transient-
    // local topic by an earlier node, then stay silent: no redraw timer exists.
    visualization_msgs::msg::MarkerArray clear;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = config_.global_frame;
    marker.header.stamp = get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.markers.push_back(marker);
    terrain_publisher_->publish(clear);
  }
  coarse_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.coarse_topic, latched_qos);
  corridor_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.corridor_topic, stream_qos);
  cost_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    config_.cost_topic, latched_qos);
  diagnostics_publisher_ = create_publisher<std_msgs::msg::String>(
    config_.diagnostics_topic, stream_qos);
  version_publisher_ = create_publisher<std_msgs::msg::String>(
    config_.version_topic, latched_qos);
  fallback_publisher_ = create_publisher<std_msgs::msg::String>(
    config_.fallback_topic, latched_qos);
  wheel_debug_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    config_.wheel_debug_topic, stream_qos);
  stage_publisher_ = create_publisher<std_msgs::msg::String>(
    config_.planner_stage_topic, stream_qos);
  metrics_publisher_ = create_publisher<std_msgs::msg::String>(
    config_.metrics_topic, stream_qos);

  action_server_ = rclcpp_action::create_server<GetPlan>(
    this, kActionName,
    std::bind(
      &GridGlobalPlanner::HandleGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&GridGlobalPlanner::HandleCancel, this, std::placeholders::_1),
    std::bind(
      &GridGlobalPlanner::HandleAccepted, this,
      std::placeholders::_1));

  if (config_.enable_ground_cloud) {
    ground_subscription_ = create_subscription<PointCloud2>(
      config_.ground_cloud_topic, latched_qos,
      std::bind(
        &GridGlobalPlanner::OnGroundCloud, this,
        std::placeholders::_1));
  }
  // Raw source clouds are opt-in. When the fallback is disabled the planner
  // neither subscribes nor invents free space from point-only clouds.
  if (config_.enable_raw_cloud_fallback &&
    config_.enable_static_obstacle_cloud)
  {
    static_subscription_ = create_subscription<PointCloud2>(
      config_.static_obstacle_cloud_topic, latched_qos,
      std::bind(
        &GridGlobalPlanner::OnStaticObstacleCloud, this,
        std::placeholders::_1));
  }
  if (config_.enable_raw_cloud_fallback &&
    config_.enable_dynamic_obstacle_cloud)
  {
    dynamic_subscription_ = create_subscription<PointCloud2>(
      config_.dynamic_obstacle_cloud_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(
        &GridGlobalPlanner::OnDynamicObstacleCloud, this,
        std::placeholders::_1));
  }
  if (config_.offline.enabled) {
    // Static maps already use the global frame. Zero timestamps are valid;
    // no live-sensor freshness/TF filter applies to these latched inputs.
    static_ground_subscription_ = create_subscription<PointCloud2>(
      config_.ground_cloud_topic, latched_qos,
      std::bind(
        &GridGlobalPlanner::OnStaticGroundCloud, this,
        std::placeholders::_1));
    // The raw mapcloud is required when it is ground data, injected static
    // obstacle evidence, or the provenance half of the live scene source
    // pair. Metadata-only readiness must never silently drop the subscription.
    if (config_.offline.global_cloud_mode || UseGlobalAsStaticObstacles() ||
      SceneSourceGatingEnabled())
    {
      static_global_subscription_ = create_subscription<PointCloud2>(
        config_.static_obstacle_cloud_topic, latched_qos,
        std::bind(
          &GridGlobalPlanner::OnStaticGlobalCloud, this,
          std::placeholders::_1));
    }
  }
  if (config_.enable_source_observation) {
    // Reliable delivery with an explicit bounded queue. Rejections are reported
    // as backpressure diagnostics rather than silently dropping clears.
    rclcpp::QoS source_qos(rclcpp::KeepLast(10));
    source_qos.reliable();
    source_observation_subscription_ =
      create_subscription<SourceObservationMsg>(
      config_.source_observation_topic, source_qos,
      std::bind(
        &GridGlobalPlanner::OnSourceObservation, this,
        std::placeholders::_1));
  }
  if (config_.enable_perception_export && scene_snapshot_provider_) {
    perception_timer_ = create_wall_timer(
      SecondsToSteadyDuration(1.0 / config_.perception_publish_rate_hz),
      std::bind(&GridGlobalPlanner::OnPerceptionTimer, this));
  }

  if (config_.source_expiry_period_sec > 0.0 &&
    std::isfinite(config_.source_expiry_period_sec))
  {
    expiry_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.source_expiry_period_sec),
      std::bind(&GridGlobalPlanner::OnExpireTimer, this));
  }

  // Map publication only queues a cheap latest snapshot. This dedicated group
  // owns the bounded-rate terrain geometry build and DDS publication, so it can
  // never run concurrently with itself and never waits on the map worker, store,
  // job, or queue locks while drawing. It exists only when both the MASTER base
  // display and the optional cost display are enabled. Base-only display emits
  // one geometry frame and has no redraw timer; MASTER off never has one.
  if (config_.publish_terrain_debug &&
    config_.publish_terrain_cost_debug)
  {
    terrain_visualization_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    terrain_visualization_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GridGlobalPlanner::OnTerrainVisualizationTimer, this),
      terrain_visualization_callback_group_);
  }
}

void GridGlobalPlanner::StartMapWorker()
{
  snapshot_store_ =
    std::make_unique<SnapshotStore>(
    config_.worker.retained_snapshots, config_.max_retired_snapshots);
  map_worker_ = std::make_unique<MapUpdateWorker>(
    config_.worker, snapshot_store_.get(),
    [this](std::shared_ptr<const MapSnapshot> snapshot) {
      OnMapPublished(std::move(snapshot));
    },
    [this](const MapUpdateMetrics & metrics) {
      // Called by the single map writer after queue/store locks are released.
      PublishUpdateMetrics(metrics);
    });
  if (!map_worker_->Start()) {
    throw std::runtime_error("MapUpdateWorker failed to start");
  }
}

bool GridGlobalPlanner::BuildOfflineSnapshotFromClouds(
  const std::vector<Point3> & global_points,
  const std::vector<Point3> & ground_points,
  const perception_3d::SourceMapIdentity & source_input_identity)
{
  // Start before any reconstruction/adoption work for this attempt. The value
  // is consumed only by the success logs after a usable map is truly ready.
  static_map_load_started_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count(),
    std::memory_order_relaxed);
  // Both latched clouds already passed StaticMapInput validation. This method
  // only preserves the legacy reconstruction/publication contract; the planner
  // path performs no file I/O and no TF lookup.
  const std::vector<Point3> no_obstacle_points;
  // Ground-only default keeps mapcloud only as raw provenance/fingerprint
  // input: an EMPTY obstacle set makes the terrain/static fields match the
  // saved test grid. The explicit public switch restores legacy static obstacle
  // injection. Full-cloud mode keeps its existing classification path.
  const std::vector<Point3> & obstacle_points =
    UseGlobalAsStaticObstacles() ? global_points : no_obstacle_points;
  OfflineStaticMapResult result;
  if (config_.offline.global_cloud_mode) {
    RCLCPP_INFO(
      get_logger(),
      "offline global-surface build from latched clouds: global=%zu pts "
      "ground_hint=%zu pts resolution=%.3f neighborhood=%.2f max_gap=%.2f "
      "seed_tolerance=%.3f",
      global_points.size(), ground_points.size(),
      config_.offline.map.resolution,
      config_.offline.map.neighborhood_radius_m,
      config_.offline.map.max_sampling_gap_m,
      config_.offline.map.global_ground_seed_tolerance_m);
    result = BuildOfflineGlobalCloudMap(
      global_points, ground_points, no_obstacle_points, config_.offline.map,
      config_.global_frame);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "offline legacy ground-only build from latched cloud: ground=%zu pts "
      "static_obstacle=%zu pts resolution=%.3f neighborhood=%.2f max_gap=%.2f",
      ground_points.size(), obstacle_points.size(),
      config_.offline.map.resolution,
      config_.offline.map.neighborhood_radius_m,
      config_.offline.map.max_sampling_gap_m);
    result = BuildOfflineStaticMap(
      ground_points, obstacle_points, config_.offline.map,
      config_.global_frame);
  }
  if (!result.ok()) {
    RCLCPP_ERROR(
      get_logger(),
      "offline static-map build rejected: status=%s detail=%s "
      "ground=%zu measurement(points=%zu patches=%zu refs=%zu "
      "budget_rejections=%zu expanded_attempts=%zu expanded_recoveries=%zu "
      "expansion_invalid=%zu) candidate_layers=%zu accepted=%zu "
      "depression=%zu",
      OfflineStaticMapStatusToString(result.status), result.detail.c_str(),
      result.metrics.ground_points, result.metrics.measurement_points,
      result.metrics.measurement_patches,
      result.metrics.measurement_point_refs,
      result.metrics.measurement_budget_rejections,
      result.metrics.expanded_patch_attempts,
      result.metrics.expanded_patch_recoveries,
      result.metrics.rejected_expansion_invalid,
      result.metrics.candidate_layers, result.metrics.accepted_nodes,
      result.metrics.local_depression_rejections);
    return false;
  }
  // Bind the producer's future scene source identity to the exact raw
  // mapground/mapcloud pair that produced this terrain. This runs once per
  // accepted static event, before readiness is published, and never compares
  // the producer revision with Grid terrain revision/obstacle version.
  if (SceneSourceGatingEnabled() && !source_input_identity.known) {
    RCLCPP_ERROR(
      get_logger(),
      "static source identity is unavailable; perception evidence will be "
      "withheld until a verified source binding exists");
  }
  SetExpectedStaticSourceIdentity(source_input_identity);

  if (config_.enable_perception_export && scene_snapshot_provider_) {
    // The in-process profile must let the single map worker own the adopted
    // baseline: it preserves the prebuilt fine-terrain pointer/revision and the
    // static layer while applying later dynamic overlay updates incrementally.
    // Readiness is published from OnMapPublished only after the store handoff.
    if (!map_worker_) {
      RCLCPP_ERROR(
        get_logger(),
        "offline static-map adoption requires the map worker; snapshot not "
        "published");
      return false;
    }
    bool expected = false;
    if (!adoption_publish_pending_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(
        get_logger(),
        "offline static-map adoption was already queued; ignoring duplicate");
      return false;
    }
    const MapUpdateResult adoption =
      map_worker_->AdoptSnapshot(result.snapshot);
    if (adoption.status != MapUpdateStatus::kAccepted) {
      adoption_publish_pending_.store(false, std::memory_order_relaxed);
      RCLCPP_ERROR(
        get_logger(),
        "offline static-map adoption rejected: status=%s detail=%s",
        MapUpdateStatusToString(adoption.status), adoption.detail.c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "offline static-map adoption queued: obstacle_records=%zu "
      "static_obstacles=%zu; readiness follows the worker store handoff",
      result.metrics.obstacle_records,
      result.snapshot->static_obstacles().tiles_layer().tiles().size());
    return true;
  }
  WarmGraphSearch(*result.snapshot);
  if (!snapshot_store_ || !snapshot_store_->TryPublish(result.snapshot)) {
    RCLCPP_ERROR(
      get_logger(),
      "offline static-map snapshot publication was rejected; the accepted "
      "input stays unlatched so a later retry can build it");
    return false;
  }
  // The immutable snapshot is now the planner's current map. Mark readiness
  // before visualization so a plan that arrives during display publication
  // already sees the completed terrain.
  static_map_ready_.store(true, std::memory_order_relaxed);
  // Map-ready latched evidence must be visible to RViz before the first
  // /get_plan request. Each visualization output is isolated: a display-only
  // failure is logged and must not abort offline startup or motion.
  if (result.snapshot) {
    PublishInitialSnapshotVisualization(*result.snapshot);
    std::lock_guard<std::mutex> lock(terrain_refresh_mutex_);
    terrain_refresh_ = TerrainVisualizationRefresh{};
    // The initial full frame above is published for every MASTER-on startup,
    // with or without cost coloring. Only the dynamic cost stream needs a
    // rendered pin so its first 2 Hz tick does not redraw the same snapshot.
    if (config_.publish_terrain_debug &&
      config_.publish_terrain_cost_debug)
    {
      terrain_refresh_.MarkRendered(result.snapshot);
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "offline static map ready: load_elapsed_sec=%.3f supported_nodes=%zu connections=%zu "
    "obstacle_records=%zu overhead_ignored=%zu duplicate_ground=%zu "
    "candidate_layers=%zu accepted=%zu prohibited=%zu "
    "unconfirmed_nodes=%zu unconfirmed_components=%zu ground_components=%zu "
    "rejected(insufficient=%zu gap=%zu unsupported=%zu fit=%zu "
    "rms=%zu inliers=%zu nonfinite=%zu slope_policy=%zu "
    "depression=%zu) "
    "ordinary_fit_failures=%zu ordinary_rms=%zu "
    "robust(attempts=%zu recovered=%zu rejected=%zu iterations=%zu) "
    "measurement(points=%zu patches=%zu refs=%zu depression=%zu "
    "raised_patches=%zu expanded_attempts=%zu expanded_recoveries=%zu "
    "expansion_invalid=%zu adaptive_coverage_rejections=%zu "
    "budget_rejections=%zu) "
    "gate_examples=%zu terrain_ms=%.1f obstacle_ms=%.1f total_ms=%.1f",
    StaticMapLoadElapsedSeconds(),
    result.metrics.supported_nodes, result.metrics.connections,
    result.metrics.obstacle_records,
    result.metrics.ignored_overhead_obstacles,
    result.metrics.duplicate_ground_obstacles,
    result.metrics.candidate_layers, result.metrics.accepted_nodes,
    result.metrics.prohibited_nodes, result.metrics.unconfirmed_nodes,
    result.metrics.unconfirmed_components, result.metrics.ground_components,
    result.metrics.rejected_insufficient_points,
    result.metrics.rejected_sampling_gap,
    result.metrics.rejected_unsupported_center,
    result.metrics.rejected_plane_fit,
    result.metrics.rejected_plane_rms,
    result.metrics.rejected_plane_inliers,
    result.metrics.rejected_nonfinite_plane,
    result.metrics.rejected_prohibited_slope,
    result.metrics.rejected_robust_depression,
    result.metrics.ordinary_plane_failures,
    result.metrics.ordinary_rms_rejections,
    result.metrics.robust_attempts,
    result.metrics.robust_recovered_nodes,
    result.metrics.robust_rejected,
    result.metrics.robust_refinement_iterations,
    result.metrics.measurement_points,
    result.metrics.measurement_patches,
    result.metrics.measurement_point_refs,
    result.metrics.local_depression_rejections,
    result.metrics.patches_with_raised_evidence,
    result.metrics.expanded_patch_attempts,
    result.metrics.expanded_patch_recoveries,
    result.metrics.rejected_expansion_invalid,
    result.metrics.rejected_adaptive_coverage,
    result.metrics.measurement_budget_rejections,
    result.metrics.gate_examples.size(),
    result.metrics.terrain_ms, result.metrics.obstacle_ms,
    result.metrics.total_ms);
  for (const OfflineGateExample & example : result.metrics.gate_examples) {
    RCLCPP_INFO(
      get_logger(),
      "offline gate example: gate=%s cell=(%" PRId64 ",%" PRId64 ") "
      "point=(%.3f,%.3f,%.3f) actual=%.5f threshold=%.5f",
      example.gate.c_str(), example.cell.x, example.cell.y,
      example.point.x, example.point.y, example.point.z,
      example.actual, example.threshold);
  }
  return true;
}

void GridGlobalPlanner::OnStaticGroundCloud(
  const PointCloud2::ConstSharedPtr message)
{
  QueueStaticCloud(false, message);
}

void GridGlobalPlanner::OnStaticGlobalCloud(
  const PointCloud2::ConstSharedPtr message)
{
  QueueStaticCloud(true, message);
}

void GridGlobalPlanner::QueueStaticCloud(
  bool global_cloud, const PointCloud2::ConstSharedPtr & message)
{
  if (!message || static_map_ready_.load(std::memory_order_relaxed) ||
    shutting_down_.load(std::memory_order_relaxed))
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(static_input_mutex_);
    if (static_map_ready_.load(std::memory_order_relaxed)) {
      return;
    }
    if (global_cloud) {
      static_global_message_ = message;
      static_global_pending_ = true;
    } else {
      static_ground_message_ = message;
      static_ground_pending_ = true;
    }
  }
  // Validation and the heavy immutable build run on the owned static-map
  // worker, so a large latched cloud never occupies an executor thread that is
  // needed by the action service.
  static_input_condition_.notify_one();
}

void GridGlobalPlanner::StaticMapLoop()
{
  StaticMapInputOptions input_options;
  input_options.global_cloud_mode = config_.offline.global_cloud_mode;
  input_options.use_global_as_obstacles = UseGlobalAsStaticObstacles();
  // Internal provenance/readiness, not a ROS parameter: an active live scene
  // gate has to derive the producer identity from the exact raw pair Grid used.
  // It requires the mapcloud input but never injects it as obstacles.
  input_options.require_global_source_identity = SceneSourceGatingEnabled();
  input_options.frame_id = config_.global_frame;
  input_options.max_points = config_.terrain.observation_validation.max_points;
  StaticMapInput input(std::move(input_options));
  // The exact messages accepted for the one-shot build. They are retained only
  // to derive the source identity once, from the same raw content that is
  // passed to the Grid terrain builder.
  PointCloud2::ConstSharedPtr accepted_ground_message;
  PointCloud2::ConstSharedPtr accepted_global_message;

  while (!shutting_down_.load(std::memory_order_relaxed)) {
    PointCloud2::ConstSharedPtr ground;
    PointCloud2::ConstSharedPtr global;
    {
      std::unique_lock<std::mutex> lock(static_input_mutex_);
      static_input_condition_.wait(
        lock, [this]() {
          return shutting_down_.load(std::memory_order_relaxed) ||
          static_ground_pending_ || static_global_pending_;
        });
      if (shutting_down_.load(std::memory_order_relaxed)) {
        break;
      }
      ground = std::move(static_ground_message_);
      global = std::move(static_global_message_);
      static_ground_pending_ = false;
      static_global_pending_ = false;
    }

    try {
      const auto accepted = [this](
        const char * label, const StaticMapInputResult & result) {
          if (result.status == StaticMapInputStatus::kAccepted) {
            RCLCPP_INFO(
              get_logger(), "latched %s accepted: %zu points", label,
              result.point_count);
            return true;
          }
          if (result.status == StaticMapInputStatus::kInvalid) {
            RCLCPP_WARN(
              get_logger(), "latched %s rejected: %s", label,
              result.detail.c_str());
          }
          return false;
        };
      bool updated = false;
      if (ground) {
        const StaticMapInputResult result = input.AcceptGround(*ground);
        if (accepted("mapground", result)) {
          accepted_ground_message = ground;
          updated = true;
        }
      }
      if (global) {
        const StaticMapInputResult result = input.AcceptGlobal(*global);
        if (accepted("mapcloud", result)) {
          accepted_global_message = global;
          updated = true;
        }
      }
      if (!updated || !input.ReadyToBuild()) {
        continue;
      }
      perception_3d::SourceMapIdentity source_input_identity;
      std::string identity_detail;
      if (SceneSourceGatingEnabled() &&
        (!accepted_ground_message || !accepted_global_message ||
        !ComputeStaticSourceInputIdentity(
          *accepted_ground_message, *accepted_global_message,
          config_.global_frame, static_source_input_options_,
          &source_input_identity, &identity_detail)))
      {
        RCLCPP_ERROR(
          get_logger(),
          "static source identity could not be derived from the accepted raw "
          "inputs; perception evidence will be withheld: %s",
          identity_detail.c_str());
        source_input_identity = perception_3d::SourceMapIdentity();
      }
      if (BuildOfflineSnapshotFromClouds(
          input.global_points(), input.ground_points(), source_input_identity))
      {
        input.MarkBuilt();
        // Drop any messages queued during the build and release raw inputs.
        std::lock_guard<std::mutex> lock(static_input_mutex_);
        static_ground_message_.reset();
        static_global_message_.reset();
        static_ground_pending_ = false;
        static_global_pending_ = false;
        return;
      }
      // A failed build waits for another valid publication rather than
      // repeating expensive reconstruction of the same data on a timer.
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "static map input/build failed; waiting for new input: %s",
        exception.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "static map input/build failed; waiting for new input");
    }
  }
}

void GridGlobalPlanner::StartThreads()
{
  planning_thread_ = std::thread(&GridGlobalPlanner::PlanningLoop, this);
  ingest_thread_ = std::thread(&GridGlobalPlanner::IngestLoop, this);
  if (config_.offline.enabled) {
    // The one-shot static-map validation/build runs off the executor so the
    // first heavy reconstruction cannot block the action service.
    static_map_thread_ = std::thread(&GridGlobalPlanner::StaticMapLoop, this);
  }
}

void GridGlobalPlanner::StopThreads()
{
  if (terrain_visualization_timer_) {
    terrain_visualization_timer_->cancel();
    terrain_visualization_timer_.reset();
  }
  terrain_visualization_callback_group_.reset();
  if (perception_timer_) {
    perception_timer_->cancel();
  }
  if (expiry_timer_) {
    expiry_timer_->cancel();
  }
  shutting_down_.store(true, std::memory_order_relaxed);
  DeactivateResult deactivated;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (cancel_current_) {
      cancel_current_->store(true, std::memory_order_relaxed);
    }
    // Invalidate the delivery generation so a late DDS result/path cannot
    // publish after shutdown has begun. A committed/delivering/failed owner is
    // retained here and released only after the action server is disabled.
    deactivated = delivery_.Deactivate();
  }
  job_condition_.notify_all();
  {
    std::lock_guard<std::mutex> lock(ingest_mutex_);
  }
  ingest_condition_.notify_all();
  {
    std::lock_guard<std::mutex> lock(static_input_mutex_);
  }
  static_input_condition_.notify_all();
  if (static_map_thread_.joinable()) {
    static_map_thread_.join();
  }
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
  if (ingest_thread_.joinable()) {
    ingest_thread_.join();
  }
  if (map_worker_) {
    map_worker_->Stop();
  }
  // Disable/reset the action server before dropping any uncertain goal handle.
  // Humble's ServerGoalHandle destructor auto-cancels through a weak server
  // callback; with the server reset that callback can no longer publish.
  action_server_.reset();
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_job_.reset();
    plan_active_ = false;
    plan_reserved_ = false;
    plan_reserved_epoch_valid_ = false;
    reserved_goal_uuid_.clear();
    reserved_cancel_requested_ = false;
    cancel_current_.reset();
    active_job_identity_.reset();
  }
  std::shared_ptr<void> shutdown_owner = delivery_.TakeOwnerForShutdown();
  // `deactivated.released_owner` (planning-phase supersede) and
  // `shutdown_owner` (uncertain keepalive) are destroyed here at function exit,
  // after the action server reset and outside every store/job/state lock.
}

// ---------------------------------------------------------------------------
// Action callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse GridGlobalPlanner::HandleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GetPlan::Goal> goal)
{
  if (!goal) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->activate_threading) {
    // The legacy reserve-bypass command is always accepted, even while a normal
    // plan is running. HandleAccepted cancels the running job and returns
    // succeeded with an empty path.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::uint64_t rejected_request_id = 0;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (plan_active_ || plan_reserved_ || pending_job_.has_value()) {
      // A distinct action status is required for a busy planner. REJECT is the
      // only goal response that does not enter the execution pipeline.
      rejected_request_id = request_counter_.fetch_add(1) + 1;
    } else {
      plan_reserved_ = true;
      plan_reserved_epoch_ = lifecycle_epoch_;
      plan_reserved_epoch_valid_ = true;
      // Bind a possible pre-admission cancel to this exact goal. Without this,
      // an old goal's cancel could match whatever job happens to be current.
      reserved_goal_uuid_ = rclcpp_action::to_string(uuid);
      reserved_cancel_requested_ = false;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }
  // Publish telemetry outside job_mutex_. A rejected goal is still a terminal
  // outcome and must be retained by the collector.
  PublishStage(rejected_request_id, 0, "rejected_busy", 0.0, 0, kResultBusy);
  PublishTerminalMetrics(
    rejected_request_id, 0, kResultBusy,
    "planner is busy with another goal");
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse GridGlobalPlanner::HandleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!goal_handle) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  const std::string goal_uuid =
    rclcpp_action::to_string(goal_handle->get_goal_id());
  std::lock_guard<std::mutex> lock(job_mutex_);
  // A cancel that arrives between HandleGoal reservation and HandleAccepted
  // admission is bound to the reserved UUID, never to the current job.
  if (plan_reserved_ && reserved_goal_uuid_ == goal_uuid) {
    reserved_cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  const CancelDecision decision = delivery_.OnCancelRequested(goal_uuid);
  if (decision == CancelDecision::kAcceptPreCommit) {
    if (cancel_current_) {
      cancel_current_->store(true, std::memory_order_relaxed);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  // A postcommit cancel would race the SDK succeed() terminal transition, and
  // a stale old goal must never cancel a newer job. REJECT both.
  return rclcpp_action::CancelResponse::REJECT;
}

void GridGlobalPlanner::HandleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (!goal_handle) {
    return;
  }
  const auto goal = goal_handle->get_goal();
  if (!goal) {
    return;
  }

  if (!goal->activate_threading) {
    const std::uint64_t request_id = request_counter_.fetch_add(1) + 1;
    const std::uint64_t goal_seq = goal_counter_.fetch_add(1) + 1;
    // Reserve-bypass: cancel any running task and clear the published path.
    // This path is intentionally independent of whether a plan is busy.
    CancelActivePlan("deactivate_threading");
    auto result = std::make_shared<GetPlan::Result>();
    result->planning_time = ToRosDuration(std::chrono::nanoseconds(0));
    // A cancel may have been accepted before this branch runs. Humble throws if
    // canceled() is called without a received cancel request, so branch on the
    // actual ROS cancel state and always leave exactly one terminal state.
    try {
      if (goal_handle->is_active()) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->succeed(result);
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "failed to finalize deactivation goal: %s",
        exception.what());
      try {
        if (goal_handle->is_active()) {
          goal_handle->abort(result);
        }
      } catch (...) {
        // The goal already reached a terminal state.
      }
    }
    PublishStage(
      request_id, goal_seq, "finished", 0.0, 0,
      kResultDeactivated);
    PublishDiagnostics(
      request_id, goal_seq, "finished", kResultDeactivated,
      "empty path", 0.0, nullptr);
    PublishTerminalMetrics(
      request_id, goal_seq, kResultDeactivated,
      "empty path");
    return;
  }

  const std::string goal_uuid =
    rclcpp_action::to_string(goal_handle->get_goal_id());
  PlanningJob job;
  job.goal_handle = goal_handle;
  job.cancel = std::make_shared<std::atomic<bool>>(false);
  job.received = std::chrono::steady_clock::now();
  job.request_id = request_counter_.fetch_add(1) + 1;
  job.goal_seq = goal_counter_.fetch_add(1) + 1;
  job.identity.goal_uuid = goal_uuid;
  job.identity.goal_seq = job.goal_seq;
  job.identity.lifecycle_epoch = lifecycle_epoch_;

  bool lost_reservation = false;
  bool reserved_cancel = false;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    lost_reservation =
      !plan_reserved_ || !plan_reserved_epoch_valid_ ||
      plan_reserved_epoch_ != lifecycle_epoch_ || plan_active_ ||
      pending_job_.has_value() || reserved_goal_uuid_ != goal_uuid;
    if (!lost_reservation) {
      job.lifecycle_epoch = lifecycle_epoch_;
      job.identity.lifecycle_epoch = lifecycle_epoch_;
      // Register the exact action identity before exposing the scheduling slot.
      // A stale cancel can then never match this job by accident.
      if (!delivery_.Admit(job.identity, job.goal_handle)) {
        lost_reservation = true;
      } else {
        pending_job_ = job;
        plan_active_ = true;
        plan_reserved_ = false;
        plan_reserved_epoch_valid_ = false;
        cancel_current_ = job.cancel;
        active_job_identity_ = job.identity;
        reserved_cancel = reserved_cancel_requested_;
      }
    }
    if (lost_reservation) {
      plan_reserved_ = false;
      plan_reserved_epoch_valid_ = false;
    }
    reserved_goal_uuid_.clear();
    reserved_cancel_requested_ = false;
  }

  if (lost_reservation) {
    auto result = std::make_shared<GetPlan::Result>();
    result->planning_time = ToRosDuration(std::chrono::nanoseconds(0));
    if (goal_handle->is_active()) {
      goal_handle->abort(result);
    }
    PublishStage(
      job.request_id, job.goal_seq, "finished", 0.0, 0,
      kResultBusy);
    PublishDiagnostics(
      job.request_id, job.goal_seq, "finished", kResultBusy,
      "planner reservation was lost", 0.0, nullptr);
    PublishTerminalMetrics(
      job.request_id, job.goal_seq, kResultBusy,
      "planner reservation was lost");
    return;
  }

  if (reserved_cancel) {
    // The cancel was accepted while this goal was reserved but not yet admitted.
    // Keep the normal queue path so the planning thread emits one terminal
    // canceled result instead of aborting from the accept callback.
    job.cancel->store(true, std::memory_order_relaxed);
  }

  PublishStage(
    job.request_id, job.goal_seq, "accepted", 0.0, 0,
    "in_progress");
  job_condition_.notify_one();
}

// ---------------------------------------------------------------------------
// Map input
// ---------------------------------------------------------------------------

void GridGlobalPlanner::OnGroundCloud(
  const PointCloud2::ConstSharedPtr message)
{
  QueueFrozenCloud(CloudKind::kGround, message);
}

void GridGlobalPlanner::OnStaticObstacleCloud(
  const PointCloud2::ConstSharedPtr message)
{
  QueueFrozenCloud(CloudKind::kStatic, message);
}

void GridGlobalPlanner::OnDynamicObstacleCloud(
  const PointCloud2::ConstSharedPtr message)
{
  QueueFrozenCloud(CloudKind::kDynamic, message);
}

void GridGlobalPlanner::OnSourceObservation(
  const SourceObservationMsg::ConstSharedPtr message)
{
  if (shutting_down_.load(std::memory_order_relaxed) || !message) {
    return;
  }
  // Freeze the immutable message and enqueue it. Conversion and worker
  // submission happen on the managed ingest thread, never in the ROS callback.
  FrozenSourceObservation frozen;
  frozen.message = message;
  frozen.received_steady_ns = SteadyNowNs();
  frozen.queue_sequence = source_observation_sequence_.fetch_add(1) + 1;
  bool dropped = false;
  std::string dropped_source;
  {
    std::lock_guard<std::mutex> lock(ingest_mutex_);
    while (source_observation_queue_.size() >= config_.max_source_queue) {
      if (source_observation_queue_.front().message) {
        dropped_source =
          source_observation_queue_.front().message->source_id;
      }
      source_observation_queue_.pop_front();
      dropped = true;
    }
    source_observation_queue_.push_back(std::move(frozen));
  }
  if (dropped) {
    MarkSourceBackpressure(
      dropped_source,
      "source observation queue full; oldest frozen message dropped "
      "without applying its clear records");
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_queue_full",
      "dropped oldest source observation for '" +
      dropped_source + "'",
      0.0, nullptr);
  }
  ingest_condition_.notify_one();
}

void GridGlobalPlanner::QueueFrozenCloud(
  CloudKind kind, const PointCloud2::ConstSharedPtr & message)
{
  if (shutting_down_.load(std::memory_order_relaxed) || !message) {
    return;
  }
  // The ROS message is const and shared; the conversion thread only reads it.
  // Keeping only the newest complete replacement per source preserves final
  // semantics without blocking the executor callback.
  FrozenCloud frozen;
  frozen.message = message;
  frozen.received_steady_ns = SteadyNowNs();
  frozen.sequence = source_sequence_.fetch_add(1) + 1;
  const std::size_t index = static_cast<std::size_t>(kind);
  {
    std::lock_guard<std::mutex> lock(ingest_mutex_);
    ingest_slots_[index].cloud = std::move(frozen);
    ingest_slots_[index].has_value = true;
  }
  ingest_condition_.notify_one();
}

void GridGlobalPlanner::IngestLoop()
{
  std::size_t next_kind = 0;
  while (!shutting_down_.load(std::memory_order_relaxed)) {
    CloudKind kind = CloudKind::kGround;
    FrozenCloud cloud;
    FrozenSourceObservation source_observation;
    bool have_cloud = false;
    bool have_source = false;
    {
      std::unique_lock<std::mutex> lock(ingest_mutex_);
      ingest_condition_.wait(
        lock, [this]() {
          if (shutting_down_.load(std::memory_order_relaxed)) {
            return true;
          }
          if (!source_observation_queue_.empty()) {
            return true;
          }
          for (const IngestSlot & slot : ingest_slots_) {
            if (slot.has_value) {
              return true;
            }
          }
          return false;
        });
      if (shutting_down_.load(std::memory_order_relaxed)) {
        break;
      }
      if (!source_observation_queue_.empty()) {
        source_observation = std::move(source_observation_queue_.front());
        source_observation_queue_.pop_front();
        have_source = true;
      } else {
        for (std::size_t offset = 0; offset < ingest_slots_.size(); ++offset) {
          const std::size_t index =
            (next_kind + offset) % ingest_slots_.size();
          if (!ingest_slots_[index].has_value) {
            continue;
          }
          cloud = ingest_slots_[index].cloud;
          ingest_slots_[index].has_value = false;
          kind = static_cast<CloudKind>(index);
          next_kind = (index + 1) % ingest_slots_.size();
          have_cloud = true;
          break;
        }
      }
    }
    if (!have_source && !have_cloud) {
      continue;
    }
    try {
      if (have_source) {
        ProcessFrozenSourceObservation(source_observation);
      } else if (kind == CloudKind::kGround) {
        ProcessGroundCloud(cloud);
      } else {
        ProcessSourceCloud(kind, cloud);
      }
    } catch (const std::exception & exception) {
      PublishDiagnostics(
        0, 0, "map_input", "input_error",
        exception.what(), 0.0, nullptr);
    } catch (...) {
      PublishDiagnostics(
        0, 0, "map_input", "input_error",
        "unknown input conversion failure", 0.0, nullptr);
    }
  }
}

bool GridGlobalPlanner::TransformCloudToMap(
  const PointCloud2 & cloud, int64_t stamp_ns, std::vector<Point3> * points,
  std::string * detail) const
{
  if (points == nullptr) {
    if (detail != nullptr) {
      *detail = "null output point vector";
    }
    return false;
  }
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : cloud.fields) {
    has_x = has_x || field.name == "x";
    has_y = has_y || field.name == "y";
    has_z = has_z || field.name == "z";
  }
  if (!has_x || !has_y || !has_z) {
    if (detail != nullptr) {
      *detail = "point cloud is missing x/y/z fields";
    }
    return false;
  }
  if (cloud.header.frame_id.empty()) {
    if (detail != nullptr) {
      *detail = "point cloud frame_id is empty";
    }
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  const bool needs_transform =
    cloud.header.frame_id != config_.global_frame;
  if (needs_transform && stamp_ns <= 0) {
    // A stamped sensor observation may never fall back to the latest TF. If the
    // cloud is not already in the map frame and has no acquisition stamp, its
    // transform provenance is unknown and the observation fails closed.
    if (detail != nullptr) {
      *detail =
        "cloud is not in the map frame but has no acquisition stamp; "
        "latest TF substitution is disabled for sensor observations";
    }
    return false;
  }
  if (needs_transform &&
    !LookupTransform(
      config_.global_frame, cloud.header.frame_id, stamp_ns,
      config_.tf_timeout_sec, &transform, detail))
  {
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  points->clear();
  const std::size_t expected =
    static_cast<std::size_t>(cloud.width) * cloud.height;
  points->reserve(expected);
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    Point3 point{*iter_x, *iter_y, *iter_z};
    if (!point.IsFinite()) {
      continue;
    }
    if (needs_transform) {
      point = TransformPoint(transform, point);
    }
    points->push_back(point);
  }
  return true;
}

void GridGlobalPlanner::ProcessGroundCloud(const FrozenCloud & cloud)
{
  if (!cloud.message || !map_worker_) {
    return;
  }
  std::string detail;
  const int64_t stamp_ns =
    ResolveSensorStampNs(cloud.message->header.stamp, true, &detail);
  if (stamp_ns < 0) {
    PublishDiagnostics(
      0, 0, "map_input", "stale_sensor_input", detail, 0.0,
      nullptr);
    return;
  }
  if (stamp_ns == 0) {
    // A static/zero stamp is allowed only for map-frame ground data. It is
    // recorded as unknown provenance, never replaced with the receipt clock.
    PublishDiagnostics(
      0, 0, "map_input", "static_zero_stamp",
      "ground cloud has no acquisition stamp; treating it as static-zero "
      "unknown provenance",
      0.0, nullptr);
  }
  std::vector<Point3> points;
  if (!TransformCloudToMap(*cloud.message, stamp_ns, &points, &detail)) {
    PublishDiagnostics(
      0, 0, "map_input", "tf_unavailable", detail, 0.0,
      nullptr);
    return;
  }
  if (points.empty()) {
    // The frozen input contract rejects empty/partial clouds. Keeping the prior
    // snapshot is safer than assuming an empty world is free.
    PublishDiagnostics(
      0, 0, "map_input", "empty_ground_cloud",
      "empty ground cloud does not clear terrain", 0.0,
      nullptr);
    return;
  }

  ObservationMetadata metadata;
  metadata.frame_id = config_.global_frame;
  metadata.source = ObservationSource::kLidarGround;
  metadata.sensor_stamp_ns = stamp_ns;
  metadata.received_steady_ns = cloud.received_steady_ns;
  metadata.sequence = cloud.sequence;
  metadata.complete = true;
  metadata.update = ObservationUpdateSemantics::kReplace;

  TerrainPointObservation observation(std::move(metadata), std::move(points));
  const MapUpdateResult result = map_worker_->SubmitTerrain(std::move(observation));
  if (result.status != MapUpdateStatus::kAccepted) {
    PublishDiagnostics(
      0, 0, "map_input", "ground_update_rejected",
      std::string(MapUpdateStatusToString(result.status)) + ":" +
      result.detail,
      0.0, nullptr);
  }
}

void GridGlobalPlanner::ProcessSourceCloud(
  CloudKind kind,
  const FrozenCloud & cloud)
{
  if (!cloud.message || !map_worker_) {
    return;
  }
  std::string detail;
  // Point-only source clouds require a genuine acquisition stamp. A missing
  // stamp is not silently replaced by the receipt clock and cannot be used to
  // claim source-free space.
  const int64_t stamp_ns =
    ResolveSensorStampNs(cloud.message->header.stamp, false, &detail);
  if (stamp_ns < 0) {
    PublishDiagnostics(
      0, 0, "map_input", "stale_sensor_input", detail, 0.0,
      nullptr);
    return;
  }
  std::vector<Point3> points;
  if (!TransformCloudToMap(*cloud.message, stamp_ns, &points, &detail)) {
    PublishDiagnostics(
      0, 0, "map_input", "tf_unavailable", detail, 0.0,
      nullptr);
    return;
  }
  if (points.empty()) {
    // The frozen source snapshot contract rejects empty observations. An empty
    // complete replacement is not submitted; TTL will eventually retire the old
    // contribution instead of silently clearing it.
    PublishDiagnostics(
      0, 0, "map_input", "empty_source_cloud",
      "empty obstacle cloud is not submitted", 0.0, nullptr);
    return;
  }

  const SourceLayer layer = kind == CloudKind::kStatic ?
    SourceLayer::kStatic :
    SourceLayer::kDynamic;
  std::vector<SourceOccupancyRecord> occupied;
  occupied.reserve(points.size());
  for (const Point3 & point : points) {
    SourceOccupancyRecord record;
    record.point = point;
    // A raw point cloud has no explicit vertical extent. Record only the
    // observed point height; headroom/known-volume evidence remains unknown.
    record.min_z = point.z;
    record.max_z = point.z;
    occupied.push_back(record);
  }

  const std::string & source_id =
    kind == CloudKind::kStatic ? config_.static_obstacle_cloud_topic :
    config_.dynamic_obstacle_cloud_topic;
  SourceObservationSnapshot observation(
    source_id, config_.global_frame, cloud.sequence, LatestTerrainRevision(),
    stamp_ns, cloud.received_steady_ns, layer,
    SourceUpdateKind::kCompleteReplacement, true, std::move(occupied),
    std::vector<SourceOccupancyRecord>{},
    std::vector<ObservedVolume>{}, ObservedVolume{});

  const MapUpdateResult result =
    map_worker_->SubmitSource(std::move(observation));
  if (result.status != MapUpdateStatus::kAccepted) {
    PublishDiagnostics(
      0, 0, "map_input", "source_update_rejected",
      std::string(MapUpdateStatusToString(result.status)) + ":" +
      result.detail,
      0.0, nullptr);
  }
}

void GridGlobalPlanner::ProcessFrozenSourceObservation(
  const FrozenSourceObservation & observation)
{
  if (!observation.message || !map_worker_ ||
    shutting_down_.load(std::memory_order_relaxed))
  {
    return;
  }
  const SourceObservationMsg & message = *observation.message;
  if (message.source_id.empty()) {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_invalid",
      "source_id is empty", 0.0, nullptr);
    return;
  }
  if (message.header.frame_id.empty() ||
    message.header.frame_id != config_.global_frame)
  {
    // Never relabel a source-frame/unknown cloud as the configured map frame.
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_unsupported_frame",
      "source '" + message.source_id + "' frame '" +
      message.header.frame_id + "' is not the configured map frame '" +
      config_.global_frame + "'",
      0.0, nullptr);
    return;
  }
  std::string detail;
  const int64_t sensor_stamp_ns =
    ResolveSensorStampNs(message.header.stamp, false, &detail);
  if (sensor_stamp_ns < 0) {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_bad_stamp",
      "source '" + message.source_id + "': " + detail, 0.0,
      nullptr);
    return;
  }
  if (message.terrain_revision_known && message.terrain_revision == 0) {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_bad_revision",
      "source '" + message.source_id +
      "' claims a zero terrain revision",
      0.0, nullptr);
    return;
  }
  const std::size_t record_count = message.occupied.size() +
    message.cleared.size() +
    message.known_volumes.size();
  if (record_count > config_.max_source_records) {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_over_budget",
      "source '" + message.source_id + "' has " +
      std::to_string(record_count) +
      " records above max_source_records",
      0.0, nullptr);
    MarkSourceBackpressure(message.source_id, "record bound exceeded");
    return;
  }
  for (const auto & record : message.occupied) {
    if (!RosRecordBoundsAreValid(record)) {
      PublishDiagnostics(
        0, 0, "map_input", "source_observation_invalid",
        "source '" + message.source_id +
        "' has an invalid occupied record",
        0.0, nullptr);
      return;
    }
  }
  for (const auto & record : message.cleared) {
    if (!RosRecordBoundsAreValid(record)) {
      PublishDiagnostics(
        0, 0, "map_input", "source_observation_invalid",
        "source '" + message.source_id +
        "' has an invalid cleared record",
        0.0, nullptr);
      return;
    }
  }
  for (const auto & volume : message.known_volumes) {
    if (!RosVolumeBoundsAreValid(volume)) {
      PublishDiagnostics(
        0, 0, "map_input", "source_observation_invalid",
        "source '" + message.source_id +
        "' has an invalid known volume",
        0.0, nullptr);
      return;
    }
  }
  if (!RosVolumeBoundsAreValid(message.headroom)) {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_invalid",
      "source '" + message.source_id +
      "' has invalid headroom evidence",
      0.0, nullptr);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(source_state_mutex_);
    if (source_states_.find(message.source_id) == source_states_.end() &&
      source_states_.size() >= config_.max_sources)
    {
      PublishDiagnostics(
        0, 0, "map_input", "source_observation_over_budget",
        "source count exceeds max_sources", 0.0, nullptr);
      return;
    }
  }

  const uint64_t latest_revision = LatestTerrainRevision();
  const uint64_t terrain_revision =
    message.terrain_revision_known ? message.terrain_revision :
    latest_revision;
  const bool terrain_revision_known =
    message.terrain_revision_known || latest_revision != 0;
  if (message.terrain_revision_known && latest_revision != 0 &&
    message.terrain_revision != latest_revision)
  {
    PublishDiagnostics(
      0, 0, "map_input", "source_observation_stale_revision",
      "source '" + message.source_id +
      "' was produced for a different terrain revision",
      0.0, nullptr);
    return;
  }

  SourceObservationSnapshot source_observation =
    ConvertSourceObservationMessage(
    message, sensor_stamp_ns,
    observation.received_steady_ns,
    terrain_revision,
    terrain_revision_known);
  const std::string source_id = source_observation.source_id();
  const std::uint64_t source_sequence =
    source_observation.source_sequence();
  const int64_t received_steady_ns =
    source_observation.received_steady_ns();
  const SourceLayer layer = source_observation.layer();
  if (!SubmitSourceObservation(std::move(source_observation), "source_observation")) {
    return;
  }
  // The worker owns final ordering; this state is only for TTL and diagnostics.
  std::lock_guard<std::mutex> lock(source_state_mutex_);
  SourceRuntimeState & state = source_states_[source_id];
  state.last_sequence = source_sequence;
  state.have_sequence = true;
  state.last_received_steady_ns = received_steady_ns;
  state.layer = layer;
}

void GridGlobalPlanner::ProcessPerceptionScene(
  const perception_3d::SceneSnapshot & scene,
  const perception_3d::PerceptionSnapshot & evidence)
{
  PerceptionIngestionLimits limits;
  limits.max_sources = config_.max_sources;
  limits.max_records = config_.max_source_records;
  // The Grid terrain revision is owned by the map worker. It is the only
  // revision attached to the frozen source observations; the producer's
  // source-map revision/content hash are never compared numerically with it.
  const std::uint64_t terrain_revision = LatestTerrainRevision();
  const PerceptionIngestionResult conversion =
    ConvertPerceptionSnapshotToSourceObservations(
    evidence, config_.global_frame, scene.receivedSteadyNs(),
    terrain_revision, terrain_revision != 0, limits);
  if (!conversion.ok) {
    PublishDiagnostics(
      0, 0, "map_input", "perception_scene_rejected",
      conversion.detail, 0.0, nullptr);
    MarkSourceBackpressure("perception_scene", conversion.detail);
    return;
  }
  ClearSourceBackpressure("perception_scene");
  for (const SourceObservationSnapshot & observation :
    conversion.observations)
  {
    {
      std::lock_guard<std::mutex> lock(source_state_mutex_);
      const auto previous = source_states_.find(observation.source_id());
      if (previous != source_states_.end() && previous->second.have_sequence &&
        observation.source_sequence() <= previous->second.last_sequence)
      {
        continue;
      }
    }
    SourceObservationSnapshot copy = observation;
    const std::string source_id = copy.source_id();
    const std::uint64_t source_sequence = copy.source_sequence();
    const int64_t received_steady_ns = copy.received_steady_ns();
    const SourceLayer layer = copy.layer();
    if (!SubmitSourceObservation(std::move(copy), "perception_scene")) {
      continue;
    }
    std::lock_guard<std::mutex> lock(source_state_mutex_);
    SourceRuntimeState & state = source_states_[source_id];
    state.last_sequence = source_sequence;
    state.have_sequence = true;
    state.last_received_steady_ns = received_steady_ns;
    state.layer = layer;
  }
}

void GridGlobalPlanner::SetExpectedStaticSourceIdentity(
  const perception_3d::SourceMapIdentity & identity)
{
  std::lock_guard<std::mutex> lock(scene_source_mutex_);
  scene_source_gate_.SetExpectedInputIdentity(identity);
}

bool GridGlobalPlanner::SceneSourceGatingEnabled() const noexcept
{
  return config_.enable_perception_export &&
         static_cast<bool>(scene_snapshot_provider_);
}

bool GridGlobalPlanner::TerrainSourceReady(std::string * detail)
{
  if (!SceneSourceGatingEnabled()) {
    // Offline/static demo without an in-process scene provider retains the old
    // planning behavior exactly.
    return true;
  }
  // Poll the weak-captured provider BEFORE taking scene_source_mutex_. The
  // provider only atomically copies the producer's latest immutable root
  // snapshot, so it never calls back into Grid while Grid-owned state is held.
  const std::shared_ptr<const perception_3d::SceneSnapshot> latest =
    scene_snapshot_provider_();
  std::lock_guard<std::mutex> lock(scene_source_mutex_);
  return scene_source_gate_.CheckBoundSource(latest.get(), detail);
}

void GridGlobalPlanner::OnPerceptionTimer()
{
  if (shutting_down_.load(std::memory_order_relaxed) ||
    !scene_snapshot_provider_)
  {
    return;
  }
  // The static map must be owned/accepted by the worker before any perception
  // source is submitted. Returning here (not merely skipping the conversion)
  // also preserves the accepted-scene cursor, so the first post-adoption cycle
  // is processed instead of being mistaken for an already-consumed duplicate.
  if (config_.offline.enabled &&
    !static_map_ready_.load(std::memory_order_relaxed))
  {
    return;
  }
  const std::shared_ptr<const perception_3d::SceneSnapshot> scene =
    scene_snapshot_provider_();
  if (!scene) {
    // No root scene is not a clear. Keep all existing evidence untouched.
    return;
  }

  const std::int64_t now_steady_ns = SteadyNowNs();
  const std::int64_t max_receipt_age_ns =
    static_cast<std::int64_t>(config_.max_sensor_age_sec * 1e9);
  std::shared_ptr<const perception_3d::PerceptionSnapshot> evidence;
  SceneAdmissionStatus admission_status = SceneAdmissionStatus::kRootMissing;
  std::string admission_detail;
  bool latch_invalid = false;
  {
    std::lock_guard<std::mutex> lock(scene_source_mutex_);
    // Repeated ordinary export cycles may publish a new envelope around the
    // exact same frozen evidence. Deduplicate by the completed evidence
    // sequence + source identity + opaque observation revision, not by the
    // scene pointer or a producer revision compared with Grid's terrain.
    SceneIngestCursor cursor;
    cursor.evidence_sequence = scene->evidence() ?
      scene->evidence()->sequence() : 0U;
    cursor.source_revision = scene->sourceMap().revision;
    cursor.source_content_hash = scene->sourceMap().content_hash;
    cursor.observation_revision = scene->observationRevision();
    if (have_scene_cursor_ && scene_cursor_ == cursor &&
      !(scene_source_gate_.invalid() &&
      scene_cursor_.source_content_hash != scene->sourceMap().content_hash))
    {
      return;
    }

    const SceneAdmission admission = scene_source_gate_.Evaluate(
      scene, config_.global_frame, now_steady_ns, max_receipt_age_ns);
    admission_status = admission.status;
    admission_detail = admission.detail;
    if (admission.status == SceneAdmissionStatus::kAccepted) {
      evidence = admission.evidence;
      scene_cursor_ = cursor;
      have_scene_cursor_ = true;
    } else if (scene_source_gate_.invalid()) {
      // Latch the invalid source generation once so a broken source does not
      // produce an unbounded diagnostic/backpressure stream. The old terrain
      // remains in the store but is fenced off from planning and commit.
      latch_invalid = true;
      scene_cursor_ = cursor;
      have_scene_cursor_ = true;
    }
  }

  if (!evidence) {
    if (latch_invalid) {
      RCLCPP_ERROR(
        get_logger(),
        "perception scene source changed; Grid terrain is fenced off until "
        "restart/rebuild: %s", admission_detail.c_str());
      PublishDiagnostics(
        0, 0, "map_input", "perception_scene_source_changed",
        admission_detail, 0.0, nullptr);
      return;
    }
    if (admission_status == SceneAdmissionStatus::kStale ||
      admission_status == SceneAdmissionStatus::kIncomplete ||
      admission_status == SceneAdmissionStatus::kSensorsNotCurrent ||
      admission_status == SceneAdmissionStatus::kSourceIdentityUnknown ||
      admission_status == SceneAdmissionStatus::kEvidenceUnavailable ||
      admission_status == SceneAdmissionStatus::kWrongFrame ||
      admission_status == SceneAdmissionStatus::kSourceChanged ||
      admission_status == SceneAdmissionStatus::kExpectedSourceUnavailable)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "perception scene is not admissible (%s): %s",
        SceneAdmissionStatusToString(admission_status),
        admission_detail.c_str());
      PublishDiagnostics(
        0, 0, "map_input", "perception_scene_rejected",
        std::string(SceneAdmissionStatusToString(admission_status)) + ": " +
        admission_detail, 0.0, nullptr);
    }
    // A rejected scene never yields clear evidence. In particular, a source
    // mismatch before the first successful bind is merely pending; when the
    // producer later exposes the exact Grid input identity, it can still bind.
    return;
  }

  ProcessPerceptionScene(*scene, *evidence);
  PurgeExpiredSources(SteadyNowNs());
}

bool GridGlobalPlanner::SubmitSourceObservation(
  SourceObservationSnapshot observation, const std::string & reason)
{
  if (!map_worker_) {
    return false;
  }
  const std::string source_id = observation.source_id();
  const MapUpdateResult result =
    map_worker_->SubmitSource(std::move(observation));
  if (result.status == MapUpdateStatus::kAccepted) {
    ClearSourceBackpressure(source_id);
    return true;
  }
  if (result.status == MapUpdateStatus::kRejectedDuplicate ||
    result.status == MapUpdateStatus::kRejectedStale)
  {
    // Re-reading an immutable source is expected. It neither submits new
    // evidence nor extends its TTL, and cannot indicate queue backpressure.
    return false;
  }
  // A rejected or queue-full submission may have contained clear records. Do
  // not silently discard that fact: report explicit backpressure and keep the
  // source flagged for the operator.
  MarkSourceBackpressure(
    source_id, std::string(MapUpdateStatusToString(result.status)) + ":" +
    result.detail);
  PublishDiagnostics(
    0, 0, "map_input", "source_observation_backpressure",
    reason + " source='" + source_id + "' status=" +
    MapUpdateStatusToString(result.status) + " detail=" +
    result.detail,
    0.0, nullptr);
  return false;
}

void GridGlobalPlanner::MarkSourceBackpressure(
  const std::string & source_id,
  const std::string & detail)
{
  if (source_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(source_state_mutex_);
  source_backpressure_[source_id] = detail;
}

void GridGlobalPlanner::ClearSourceBackpressure(
  const std::string & source_id)
{
  std::lock_guard<std::mutex> lock(source_state_mutex_);
  source_backpressure_.erase(source_id);
}

void GridGlobalPlanner::PurgeExpiredSources(std::int64_t now_steady_ns)
{
  std::vector<std::string> expired;
  {
    std::lock_guard<std::mutex> lock(source_state_mutex_);
    for (auto iterator = source_states_.begin();
      iterator != source_states_.end(); )
    {
      const double ttl_s = iterator->second.layer == SourceLayer::kStatic ?
        config_.static_source_ttl_sec :
        config_.dynamic_source_ttl_sec;
      const bool expired_source =
        ttl_s > 0.0 && std::isfinite(ttl_s) &&
        (iterator->second.last_received_steady_ns <= 0 ||
        (now_steady_ns > iterator->second.last_received_steady_ns &&
        now_steady_ns - iterator->second.last_received_steady_ns >
        static_cast<std::int64_t>(ttl_s * 1e9)));
      if (!expired_source) {
        ++iterator;
        continue;
      }
      expired.push_back(iterator->first);
      source_backpressure_.erase(iterator->first);
      iterator = source_states_.erase(iterator);
    }
  }
  if (!map_worker_) {
    return;
  }
  for (const std::string & source_id : expired) {
    map_worker_->SubmitSourceRemoval(source_id);
  }
}

void GridGlobalPlanner::OnExpireTimer()
{
  if (!map_worker_ || shutting_down_.load(std::memory_order_relaxed)) {
    return;
  }
  const std::int64_t now_steady_ns = SteadyNowNs();
  // Run the worker sweep when either layer can expire. Static sources are
  // immortal by default (`static_source_ttl_s <= 0`), so a positive dynamic TTL
  // alone must still drive the sweep.
  if (config_.worker.source_ttl_s > 0.0 ||
    config_.worker.static_source_ttl_s > 0.0)
  {
    map_worker_->ExpireSources(now_steady_ns);
  }
  PurgeExpiredSources(now_steady_ns);
}

int64_t GridGlobalPlanner::ResolveSensorStampNs(
  const builtin_interfaces::msg::Time & stamp, bool allow_zero,
  std::string * detail) const
{
  const int64_t stamp_ns = rclcpp::Time(stamp).nanoseconds();
  // rclcpp's const Node::get_clock() returns Clock::ConstSharedPtr while
  // Clock::now() is a non-const stateful call. The node clock object itself is
  // not const, so this keeps the same clock source without changing constness.
  const int64_t now_ns =
    const_cast<rclcpp::Clock *>(get_clock().get())->now().nanoseconds();
  if (stamp_ns <= 0) {
    if (allow_zero) {
      // A static zero stamp is explicit unknown provenance. It is never
      // replaced with the receipt/sensor-processing clock.
      if (detail != nullptr) {
        *detail = "sensor stamp is zero (static unknown provenance)";
      }
      return 0;
    }
    if (detail != nullptr) {
      *detail = "sensor stamp is missing";
    }
    return -1;
  }
  if (now_ns > 0) {
    const int64_t future_tolerance_ns =
      static_cast<int64_t>(config_.tf_future_tolerance_sec * 1e9);
    const int64_t max_age_ns =
      static_cast<int64_t>(config_.max_sensor_age_sec * 1e9);
    if (stamp_ns > now_ns + future_tolerance_ns) {
      if (detail != nullptr) {
        *detail = "sensor stamp is in the future";
      }
      return -1;
    }
    if (now_ns - stamp_ns > max_age_ns) {
      if (detail != nullptr) {
        *detail = "sensor stamp is stale";
      }
      return -1;
    }
  }
  return stamp_ns;
}

bool GridGlobalPlanner::ValidateLatestTransformAge(
  const geometry_msgs::msg::TransformStamped & transform,
  std::string * detail) const
{
  if (transform.header.frame_id.empty()) {
    if (detail != nullptr) {
      *detail = "latest TF has no provenance frame";
    }
    return false;
  }
  const int64_t transform_ns =
    rclcpp::Time(transform.header.stamp).nanoseconds();
  if (transform_ns <= 0) {
    if (detail != nullptr) {
      *detail = "latest TF has no acquisition stamp";
    }
    return false;
  }
  // rclcpp's const Node::get_clock() returns Clock::ConstSharedPtr while
  // Clock::now() is a non-const stateful call. The node clock object itself is
  // not const, so this keeps the same clock source without changing constness.
  const int64_t now_ns =
    const_cast<rclcpp::Clock *>(get_clock().get())->now().nanoseconds();
  if (now_ns <= 0) {
    if (detail != nullptr) {
      *detail = "ROS clock is unavailable for latest-TF age validation";
    }
    return false;
  }
  const int64_t future_tolerance_ns =
    static_cast<int64_t>(config_.tf_future_tolerance_sec * 1e9);
  const int64_t max_age_ns =
    static_cast<int64_t>(config_.max_sensor_age_sec * 1e9);
  if (transform_ns > now_ns + future_tolerance_ns) {
    if (detail != nullptr) {
      *detail = "latest TF stamp is in the future";
    }
    return false;
  }
  if (now_ns - transform_ns > max_age_ns) {
    if (detail != nullptr) {
      *detail = "latest TF is older than max_sensor_age_sec";
    }
    return false;
  }
  return true;
}

uint64_t GridGlobalPlanner::LatestTerrainRevision() const
{
  if (!snapshot_store_) {
    return 0;
  }
  const auto latest = snapshot_store_->AcquireLatest();
  if (!latest || !latest->fine_terrain()) {
    return 0;
  }
  return latest->fine_terrain()->revision();
}

// ---------------------------------------------------------------------------
// TF helpers
// ---------------------------------------------------------------------------

bool GridGlobalPlanner::LookupTransform(
  const std::string & target_frame, const std::string & source_frame,
  int64_t stamp_ns, double timeout_sec,
  geometry_msgs::msg::TransformStamped * transform,
  std::string * detail) const
{
  if (!tf_buffer_ || transform == nullptr) {
    if (detail != nullptr) {
      *detail = "TF buffer is unavailable";
    }
    return false;
  }
  try {
    if (stamp_ns <= 0) {
      *transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(std::max(0.0, timeout_sec)));
    } else {
      *transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, rclcpp::Time(stamp_ns),
        rclcpp::Duration::from_seconds(std::max(0.0, timeout_sec)));
    }
    return true;
  } catch (const tf2::TransformException & exception) {
    if (detail != nullptr) {
      *detail = exception.what();
    }
    return false;
  }
}

bool GridGlobalPlanner::ResolveRequestPose(
  const geometry_msgs::msg::PoseStamped & request, bool empty_is_base_link,
  bool require_non_empty, Point3 * position, double * heading,
  bool * has_heading, std::string * detail) const
{
  if (position == nullptr || heading == nullptr || has_heading == nullptr) {
    if (detail != nullptr) {
      *detail = "null pose output";
    }
    return false;
  }
  if (!PosePositionIsFinite(request.pose)) {
    if (detail != nullptr) {
      *detail = "pose position is not finite";
    }
    return false;
  }

  const bool zero_quaternion = QuaternionIsZero(request.pose.orientation);
  const auto request_semantics =
    request_pose::InterpretRequestPose(request, require_non_empty);
  const bool empty_pose = request_semantics.is_unspecified;
  if (request_semantics.reject_as_empty) {
    if (detail != nullptr) {
      *detail = "goal pose is empty";
    }
    return false;
  }

  std::string frame = request.header.frame_id;
  if (frame.empty()) {
    frame = config_.global_frame;
  }
  const int64_t stamp_ns =
    ResolveSensorStampNs(request.header.stamp, true, detail);
  if (stamp_ns < 0) {
    return false;
  }
  if (!empty_pose && frame != config_.global_frame && stamp_ns <= 0) {
    // Latest TF may only be substituted for the explicit empty start/request
    // semantic; an explicit pose in another frame needs a genuine stamp.
    if (detail != nullptr) {
      *detail =
        "explicit pose in a non-map frame has no stamp; latest TF "
        "substitution is disabled";
    }
    return false;
  }

  if (empty_pose && empty_is_base_link) {
    // p2p sends no start. The legacy planner never consumed it either; it
    // resolved the live map<-base_link pose. Reproduce that behavior, but only
    // through the explicit opt-in and only after validating the ROS-age of the
    // actual transform stamp.
    if (!config_.allow_latest_tf_fallback) {
      if (detail != nullptr) {
        *detail = "empty start requires latest TF but it is disabled";
      }
      return false;
    }
    geometry_msgs::msg::TransformStamped transform;
    if (!LookupTransform(
        config_.global_frame, config_.robot_base_frame,
        stamp_ns, config_.tf_timeout_sec, &transform,
        detail))
    {
      return false;
    }
    if (!ValidateLatestTransformAge(transform, detail)) {
      return false;
    }
    position->x = transform.transform.translation.x;
    position->y = transform.transform.translation.y;
    position->z = transform.transform.translation.z;
    *heading = QuaternionYaw(transform.transform.rotation);
    *has_heading = true;
    return true;
  }

  if (frame == config_.global_frame) {
    position->x = request.pose.position.x;
    position->y = request.pose.position.y;
    position->z = request.pose.position.z;
    *has_heading = !zero_quaternion;
    *heading = zero_quaternion ? 0.0 : QuaternionYaw(request.pose.orientation);
    if (zero_quaternion && empty_is_base_link && stamp_ns > 0 &&
      config_.allow_latest_tf_fallback)
    {
      // An explicit position without orientation still gets the base yaw, but
      // only at an explicit stamp. Never substitute latest TF silently.
      geometry_msgs::msg::TransformStamped transform;
      if (LookupTransform(
          config_.global_frame, config_.robot_base_frame,
          stamp_ns, config_.tf_timeout_sec, &transform,
          detail) &&
        ValidateLatestTransformAge(transform, detail))
      {
        *heading = QuaternionYaw(transform.transform.rotation);
        *has_heading = true;
      }
    }
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  if (!LookupTransform(
      config_.global_frame, frame, stamp_ns,
      config_.tf_timeout_sec, &transform, detail))
  {
    return false;
  }
  geometry_msgs::msg::PoseStamped input = request;
  input.header.frame_id = frame;
  input.header.stamp = ToRosTime(rclcpp::Time(stamp_ns));
  geometry_msgs::msg::PoseStamped output;
  tf2::doTransform(input, output, transform);
  position->x = output.pose.position.x;
  position->y = output.pose.position.y;
  position->z = output.pose.position.z;
  *has_heading = !QuaternionIsZero(output.pose.orientation);
  *heading = *has_heading ? QuaternionYaw(output.pose.orientation) : 0.0;
  return PosePositionIsFinite(output.pose);
}

bool GridGlobalPlanner::QuaternionIsZero(
  const geometry_msgs::msg::Quaternion & q)
{
  return request_pose::QuaternionIsZero(q);
}

double GridGlobalPlanner::YawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q)
{
  return QuaternionYaw(q);
}

bool GridGlobalPlanner::PosePositionIsFinite(
  const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z);
}

// ---------------------------------------------------------------------------
// Planning job
// ---------------------------------------------------------------------------

void GridGlobalPlanner::PlanningLoop()
{
  while (true) {
    PlanningJob job;
    bool have_job = false;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_condition_.wait(
        lock, [this]() {
          return shutting_down_.load(std::memory_order_relaxed) ||
          pending_job_.has_value();
        });
      if (pending_job_.has_value()) {
        job = *pending_job_;
        pending_job_.reset();
        have_job = true;
      } else if (shutting_down_.load(std::memory_order_relaxed)) {
        break;
      }
    }
    if (!have_job) {
      if (shutting_down_.load(std::memory_order_relaxed)) {
        break;
      }
      continue;
    }

    if (shutting_down_.load(std::memory_order_relaxed)) {
      job.cancel->store(true, std::memory_order_relaxed);
    }
    try {
      RunPlanningJob(job);
    } catch (...) {
      // RunPlanningJob owns its exception handling; this is only a final guard
      // so the managed thread cannot terminate the process.
    }

    std::shared_ptr<void> released_owner;
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      const ReleaseDecision decision =
        delivery_.ReleaseActive(job.identity, &released_owner);
      const bool owns_slot =
        active_job_identity_.has_value() &&
        *active_job_identity_ == job.identity;
      if (owns_slot && decision != ReleaseDecision::kRetainedDeliveryFailed) {
        // Clear only this job's scheduling slot. A newer pending job (possible
        // after a deactivation generation change) is never clobbered.
        plan_active_ = false;
        plan_reserved_ = false;
        plan_reserved_epoch_valid_ = false;
        cancel_current_.reset();
        active_job_identity_.reset();
      }
      if (decision == ReleaseDecision::kRetainedDeliveryFailed) {
        RCLCPP_ERROR(
          get_logger(),
          "grid plan goal_seq=%" PRIu64 " retained an unproven delivery "
          "reservation; terminality is unknown and admission stays "
          "fail-closed until a terminal state is proven or restart",
          static_cast<std::uint64_t>(job.goal_seq));
      }
    }
    // Any goal-handle owner released by this iteration is destroyed here,
    // outside job_mutex_ and before the next job can be admitted.
    released_owner.reset();
    if (shutting_down_.load(std::memory_order_relaxed)) {
      break;
    }
  }
}

void GridGlobalPlanner::CancelActivePlan(const std::string & reason)
{
  (void)reason;
  std::shared_ptr<std::atomic<bool>> cancel;
  DeactivateResult deactivated;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    ++lifecycle_epoch_;
    plan_reserved_ = false;
    plan_reserved_epoch_valid_ = false;
    reserved_goal_uuid_.clear();
    reserved_cancel_requested_ = false;
    cancel = cancel_current_;
    // Invalidate the private-path generation before the empty clear. A planning
    // job is superseded; a committed/delivering/failed job keeps its identity
    // and real goal-handle owner but can no longer publish its private path.
    deactivated = delivery_.Deactivate();
  }
  if (cancel) {
    cancel->store(true, std::memory_order_relaxed);
  }
  // The job lock is released before serializing the empty path. The path lock
  // performs the actual DDS clear and is never held with job/store locks.
  PublishEmptyPath();
  // `deactivated.released_owner` (only for a superseded planning job) is
  // destroyed at function exit, outside every job/store/state lock.
}

namespace
{

// Inputs for one request telemetry record. It carries values copied from the
// planning job, so serialization never reads mutable job/search state.
struct RequestMetricsInput
{
  std::uint64_t request_id{0};
  std::uint64_t goal_seq{0};
  std::string status;
  std::string detail;
  std::string result_class;
  SteadyNs started_steady_ns{0};
  SteadyNs finished_steady_ns{0};
  SteadyNs commit_steady_ns{0};
  double queue_wait_ms{0.0};
  double search_ms{0.0};
  double validation_ms{0.0};
  double commit_ms{0.0};
  SearchStats search;
  std::shared_ptr<const MapSnapshot> snapshot;
  std::string search_status;
  std::string search_detail;
  std::string search_backend;
  bool kinematic_checked{false};
  std::vector<std::string> stages;
  double max_planning_time_s{0.0};
  double fine_resolution_m{0.0};
  double coarse_resolution_m{0.0};
  bool coarse_guidance{false};
  bool detailed_query_timing{false};
};

std::string MeasurementJson(
  bool measured, double value, const char * unit,
  const std::string & reason)
{
  std::ostringstream out;
  out << "{\"status\":\"" << (measured ? "measured" : "unmeasured")
      << "\"";
  if (measured) {
    out << ",\"value\":" << JsonNumber(value) << ",\"unit\":"
        << JsonString(unit);
  } else {
    out << ",\"reason\":" << JsonString(reason);
  }
  out << ",\"source\":\"node_metrics\"}";
  return out.str();
}

std::string CategoryJson(
  int category, const std::string & status,
  const std::string & reason)
{
  std::ostringstream out;
  out << "{\"category\":" << category << ",\"status\":"
      << JsonString(status) << ",\"reason\":" << JsonString(reason) << "}";
  return out.str();
}

std::string BuildRequestMetricsJson(const RequestMetricsInput & input)
{
  std::ostringstream out;
  const std::uint64_t terrain_revision =
    input.snapshot && input.snapshot->fine_terrain() ?
    input.snapshot->fine_terrain()->revision() :
    0U;
  const std::uint64_t terrain_hash =
    input.snapshot && input.snapshot->fine_terrain() ?
    input.snapshot->fine_terrain()->content_hash() :
    0U;
  const std::uint64_t obstacle_version =
    input.snapshot ? input.snapshot->obstacle_version() : 0U;
  const std::uint64_t coarse_version =
    input.snapshot && input.snapshot->coarse_terrain() ?
    input.snapshot->coarse_terrain()->revision() :
    0U;
  const double total_ms =
    SteadyDurationMs(input.started_steady_ns, input.finished_steady_ns);
  const std::int64_t ground_receipt_steady_ns =
    input.snapshot && input.snapshot->fine_terrain() ?
    input.snapshot->fine_terrain()->received_steady_ns() :
    0;
  const std::int64_t sensor_stamp_ns =
    input.snapshot && input.snapshot->fine_terrain() ?
    input.snapshot->fine_terrain()->sensor_stamp_ns() :
    0;
  const auto resources = ReadProcessResourceSnapshot();

  out << "{";
  out << "\"schema_version\":" << kMetricsSchemaVersion;
  out << ",\"record_type\":\"request\"";
  out << ",\"backend\":\"grid\"";
  out << ",\"mode\":\"ros2_action\"";
  out << ",\"status\":" << JsonString(input.status);
  out << ",\"result_class\":" << JsonString(input.result_class);
  out << ",\"failure_reason\":" << JsonString(input.detail);
  out << ",\"request_id\":" << JsonString(
    std::to_string(input.request_id));
  out << ",\"goal_seq\":" << input.goal_seq;
  out << ",\"started_at_utc\":" << JsonString(UtcNowIso8601());
  out << ",\"monotonic_start_ns\":" << input.started_steady_ns;
  out << ",\"monotonic_end_ns\":" << input.finished_steady_ns;
  out << ",\"ground_receipt_steady_ns\":" << ground_receipt_steady_ns;
  out << ",\"sensor_stamp_ns\":" << sensor_stamp_ns;
  out << ",\"server_commit_steady_ns\":" << input.commit_steady_ns;
  out << ",\"duration_ms\":" << JsonNumber(total_ms);
  out << ",\"planning_time_ms\":" << JsonNumber(total_ms);
  out << ",\"terrain_version\":" << terrain_revision;
  out << ",\"terrain_content_hash\":" << terrain_hash;
  out << ",\"obstacle_version\":" << obstacle_version;
  out << ",\"coarse_version\":" << coarse_version;
  out << ",\"resolution\":" << JsonNumber(input.fine_resolution_m);
  out << ",\"config\":{";
  out << "\"fine_resolution_m\":" << JsonNumber(input.fine_resolution_m);
  out << ",\"coarse_resolution_m\":"
      << JsonNumber(input.coarse_resolution_m);
  out << ",\"coarse_guidance\":"
      << (input.coarse_guidance ? "true" : "false");
  out << ",\"update_mode\":\"on_or_off_not_synthesized\"";
  out << ",\"search_backend\":" << JsonString(input.search_backend);
  out << ",\"kinematic_checked\":"
      << (input.kinematic_checked ? "true" : "false");
  out << ",\"max_planning_time_s\":"
      << JsonNumber(input.max_planning_time_s);
  out << ",\"config_identity\":"
      << JsonString(
    "fine=" + JsonNumber(input.fine_resolution_m) +
    ";coarse=" + JsonNumber(input.coarse_resolution_m) +
    ";search=" + input.search_backend + ";guidance=" +
    (input.coarse_guidance ? "on" : "off"));
  out << "}";
  out << ",\"phases_ms\":{";
  out << "\"queue_wait\":" << JsonNumber(input.queue_wait_ms);
  out << ",\"search\":" << JsonNumber(input.search_ms);
  out << ",\"search_total\":" << JsonNumber(input.search_ms);
  out << ",\"coarse_search\":" << JsonNumber(input.search.coarse_ms);
  out << ",\"fine_search\":" << JsonNumber(input.search.fine_ms);
  out << ",\"fallback_search\":" << JsonNumber(input.search.fallback_ms);
  out << ",\"reconstruction\":" << JsonNumber(
    input.search.reconstruction_ms);
  out << ",\"four_wheel_eval\":" << JsonNumber(
    input.search.support_eval_ms);
  out << ",\"validation\":" << JsonNumber(input.validation_ms);
  out << ",\"commit\":" << JsonNumber(input.commit_ms);
  out << ",\"total\":" << JsonNumber(total_ms);
  out << "}";
  out << ",\"search\":{";
  out << "\"status\":" << JsonString(input.search_status);
  out << ",\"detail\":" << JsonString(input.search_detail);
  out << ",\"expanded_nodes\":" << input.search.expanded_nodes;
  out << ",\"unique_states\":" << input.search.unique_states;
  out << ",\"sampled_poses\":" << input.search.sampled_poses;
  out << ",\"support_evaluations\":" << input.search.support_evaluations;
  out << ",\"support_sample_count\":"
      << input.search.support_sample_count;
  out << ",\"support_branch_combinations\":"
      << input.search.support_branch_combinations;
  out << ",\"map_queries\":" << input.search.map_queries;
  out << ",\"terrain_queries\":" << input.search.terrain_queries;
  out << ",\"cell_queries\":" << input.search.cell_queries;
  out << ",\"surface_queries\":" << input.search.surface_queries;
  out << ",\"neighbor_queries\":" << input.search.neighbor_queries;
  out << ",\"obstacle_queries\":" << input.search.obstacle_queries;
  out << ",\"distance_queries\":" << input.search.distance_queries;
  out << ",\"footprint_ms\":" << JsonNumber(input.search.footprint_ms);
  out << ",\"branch_solve_ms\":"
      << JsonNumber(input.search.branch_solve_ms);
  out << ",\"pose_solve_ms\":" << JsonNumber(input.search.pose_solve_ms);
  out << ",\"body_check_ms\":" << JsonNumber(input.search.body_check_ms);
  out << ",\"rejection_histogram\":[";
  for (std::size_t index = 0;
    index < input.search.rejection_histogram.size(); ++index)
  {
    if (index != 0U) {
      out << ",";
    }
    out << input.search.rejection_histogram[index];
  }
  out << "]";
  out << ",\"stages\":[";
  for (std::size_t index = 0; index < input.stages.size(); ++index) {
    if (index != 0U) {
      out << ",";
    }
    out << JsonString(input.stages[index]);
  }
  out << "]";
  out << "}";
  out << ",\"detailed_query_timing_enabled\":"
      << (input.detailed_query_timing ? "true" : "false");
  out << ",\"resources\":{";
  out << "\"available\":" << (resources.available ? "true" : "false");
  out << ",\"process_cpu_ms\":" << JsonNumber(resources.process_cpu_ms);
  out << ",\"thread_cpu_ms\":" << JsonNumber(resources.thread_cpu_ms);
  out << ",\"peak_rss_mib\":" << JsonNumber(resources.peak_rss_mib);
  out << ",\"scope\":\"process_self_and_calling_thread_boundary\"";
  out << "}";

  // Benchmark-compatible measurement map. Only actual node values are marked
  // measured; everything else stays explicitly unmeasured with a reason.
  std::vector<std::string> measurements;
  measurements.push_back(
    "\"search_ms\":" +
    MeasurementJson(true, input.search_ms, "ms", ""));
  measurements.push_back(
    "\"coarse_search_ms\":" +
    MeasurementJson(
      input.search.coarse_ms > 0.0,
      input.search.coarse_ms, "ms",
      "coarse guidance was not attempted"));
  measurements.push_back(
    "\"fine_search_ms\":" +
    MeasurementJson(
      input.search.fine_ms > 0.0, input.search.fine_ms, "ms",
      "fine search not run"));
  measurements.push_back(
    "\"fallback_search_ms\":" +
    MeasurementJson(
      input.search.fallback_ms > 0.0,
      input.search.fallback_ms, "ms",
      "full-fine fallback was not needed"));
  measurements.push_back(
    "\"four_wheel_eval_ms\":" +
    MeasurementJson(
      input.search.support_evaluations > 0,
      input.search.support_eval_ms, "ms",
      "no support evaluation was run"));
  measurements.push_back(
    "\"planning_response_ms\":" +
    MeasurementJson(true, total_ms, "ms", ""));
  measurements.push_back(
    "\"expanded_nodes\":" +
    MeasurementJson(
      true, static_cast<double>(input.search.expanded_nodes),
      "count", ""));
  measurements.push_back(
    "\"unique_states\":" +
    MeasurementJson(
      true, static_cast<double>(input.search.unique_states),
      "count", ""));
  measurements.push_back(
    "\"map_queries\":" +
    MeasurementJson(
      true, static_cast<double>(input.search.map_queries),
      "count", ""));
  measurements.push_back(
    "\"peak_rss_mib\":" +
    MeasurementJson(
      resources.available, resources.peak_rss_mib, "MiB",
      "getrusage unavailable on this platform"));
  measurements.push_back(
    "\"map_query_ms\":" + MeasurementJson(
      false, 0.0, "ms",
      "detailed per-query latency is OFF by default"));
  measurements.push_back(
    "\"pointcloud_to_path_e2e_ms\":" +
    MeasurementJson(
      false, 0.0, "ms",
      "metric 22 requires a pre-submitted cold target and a validated "
      "co-host monotonic clock; not synthesized"));
  measurements.push_back(
    "\"lock_wait_ms\":" + MeasurementJson(
      false, 0.0, "ms",
      "grid planner has no shared perception lock to wait on; "
      "legacy lock evidence remains unmeasured"));
  out << ",\"measurements\":{" <<[&measurements]() {
    std::string joined;
    for (std::size_t index = 0; index < measurements.size(); ++index) {
      if (index != 0U) {
        joined += ",";
      }
      joined += measurements[index];
    }
    return joined;
    } () << "}";

  std::vector<std::string> categories;
  categories.push_back(
    CategoryJson(
      1, "unmeasured",
      "terrain-build phase record is published on the same telemetry topic; "
      "join by terrain_version"));
  categories.push_back(
    CategoryJson(
      2, "unmeasured", "point-cloud throughput is in the terrain-build record; "
      "join by terrain_version"));
  categories.push_back(
    CategoryJson(
      3, "unmeasured", "per-update record is published separately; join by "
      "obstacle_version"));
  categories.push_back(
    CategoryJson(
      4, "unmeasured", "per-update copy/publish counts are in the update record"));
  categories.push_back(
    CategoryJson(
      5, "unmeasured", "distance-field phase is in the per-update record"));
  categories.push_back(
    CategoryJson(
      6, "unmeasured", "distance-field work ratio needs a full-rebuild reference run"));
  categories.push_back(
    CategoryJson(
      7, "measured", "search_ms plus fine/coarse/fallback/support phases are in phases_ms"));
  categories.push_back(
    CategoryJson(
      8, "measured",
      "server receive/queue/search/validation/commit/total are in phases_ms; "
      "client latency is collector-side and separate"));
  categories.push_back(
    CategoryJson(
      9, "measured", "expanded_nodes/unique_states/support_evaluations are actual counts"));
  categories.push_back(
    CategoryJson(
      10, "measured", "expanded/query/time fields are actual measured values"));
  categories.push_back(
    CategoryJson(
      11, "unmeasured",
      "per-query latency histograms are OFF by default; "
      "only instrumented query counts are published"));
  categories.push_back(
    CategoryJson(
      12, "unmeasured", "grid-vs-reference query latency needs a separate reference run"));
  categories.push_back(
    CategoryJson(
      13, "measured",
      "process peak RSS is sampled with getrusage at the request boundary; "
      "map/search attribution is not claimed"));
  categories.push_back(
    CategoryJson(
      14, "unmeasured",
      "map structure counts are exposed by worker diagnostics, not by this request record"));
  categories.push_back(
    CategoryJson(
      15, "unmeasured", "success rate requires a completed fixed request set"));
  categories.push_back(
    CategoryJson(
      16, "unmeasured", "reliability requires repeated replay/stress evidence"));
  categories.push_back(
    CategoryJson(
      17, "unmeasured", "path length and swept clearance need path geometry analysis"));
  categories.push_back(
    CategoryJson(
      18, "unmeasured", "path quality needs a completed comparison run"));
  categories.push_back(
    CategoryJson(
      19, "unmeasured", "curvature/terrain-cost analysis is not implemented in this record"));
  categories.push_back(
    CategoryJson(
      20, "unmeasured", "legacy-vs-grid comparison requires a synchronized legacy run"));
  categories.push_back(
    CategoryJson(
      21, "unmeasured", "update on/off comparison requires two controlled runs"));
  categories.push_back(
    CategoryJson(
      22, "unmeasured",
      "cold pointcloud-to-path requires pre-submitted target and "
      "validated cross-process monotonic domain"));
  out << ",\"category_coverage\":[";
  for (std::size_t index = 0; index < categories.size(); ++index) {
    if (index != 0U) {
      out << ",";
    }
    out << categories[index];
  }
  out << "]";
  out << ",\"notes\":[\"sample counts are evidence sufficiency gates, not confidence values\"";
  out << ",\"server commit is not client delivery\"";
  out << ",\"sensor stamps are not subtracted from steady-clock instants\"";
  out << "]}";
  return out.str();
}

}  // namespace

void GridGlobalPlanner::PublishTerminalMetrics(
  std::uint64_t request_id, std::uint64_t goal_seq,
  const std::string & result_class, const std::string & detail)
{
  RequestMetricsInput input;
  input.request_id = request_id;
  input.goal_seq = goal_seq;
  input.status = "failure";
  input.result_class = result_class;
  input.detail = detail;
  const SteadyNs now = SteadyNowNs();
  input.started_steady_ns = now;
  input.finished_steady_ns = now;
  input.max_planning_time_s = config_.max_planning_time_sec;
  input.fine_resolution_m = config_.terrain.resolution;
  input.coarse_resolution_m = config_.coarse.coarse_resolution;
  input.coarse_guidance = config_.search.use_coarse_guidance;
  input.search_backend = config_.search_backend;
  input.kinematic_checked = config_.search_backend != "graph_a_star";
  input.detailed_query_timing = config_.detailed_query_timing;
  PublishMetrics(BuildRequestMetricsJson(input));
}

void GridGlobalPlanner::RunPlanningJob(const PlanningJob & job)
{
  auto result = std::make_shared<GetPlan::Result>();
  const auto started = job.received;
  bool finished = false;
  std::size_t expansions = 0;
  std::shared_ptr<const MapSnapshot> diagnostic_snapshot;
  double finished_elapsed_sec = 0.0;
  std::string finished_detail;
  SearchResult last_search;
  SearchStats search_stats;
  std::string search_status;
  std::string search_detail;
  double search_ms = 0.0;
  double validation_ms = 0.0;
  double commit_ms = 0.0;
  SteadyNs commit_steady_ns = 0;
  const auto run_start = std::chrono::steady_clock::now();
  const double queue_wait_ms =
    std::max(0.0, DurationToMs(run_start - job.received));

  // Diagnostics and telemetry are deliberately separate from the terminal action
  // transition. Both run outside every job/commit/store guard.
  // The final commit path calls this only after both the store handoff mutex
  // and job_mutex_ have been released.
  auto report_finished = [&](const std::string & final_class,
      const std::string & detail) {
      if (final_class == kResultSuccess) {
        double path_length_m = 0.0;
        for (std::size_t index = 1; index < result->path.poses.size(); ++index) {
          const auto & previous = result->path.poses[index - 1].pose.position;
          const auto & current = result->path.poses[index].pose.position;
          path_length_m += std::hypot(
            current.x - previous.x, current.y - previous.y, current.z - previous.z);
        }
        RCLCPP_INFO(
          get_logger(),
          "plan succeeded: path_length_m=%.3f planning_time_sec=%.6f "
          "path_poses=%zu backend=%s kinematic_checked=%s",
          path_length_m, finished_elapsed_sec, result->path.poses.size(),
          config_.search_backend.c_str(),
          config_.search_backend == "graph_a_star" ? "false" : "true");
      } else if (final_class != kResultDeactivated && final_class != kResultCanceled) {
        // The result class alone cannot distinguish a stale goal stamp from a
        // pose that failed to resolve, and downstream callers only log the
        // class. Print the resolver's own detail here so the cause of an
        // invalid_start / invalid_goal is recoverable from the run log. A
        // successful deactivation (empty path) is not a failure and is skipped.
        RCLCPP_ERROR(
          get_logger(),
          "plan failed: result_class=%s detail=%s planning_time_sec=%.6f "
          "request_id=%" PRIu64 " goal_seq=%" PRIu64 " backend=%s",
          final_class.c_str(), detail.c_str(), finished_elapsed_sec,
          static_cast<std::uint64_t>(job.request_id),
          static_cast<std::uint64_t>(job.goal_seq),
          config_.search_backend.c_str());
      }
      PublishDiagnostics(
        job.request_id, job.goal_seq, "finished", final_class,
        detail, finished_elapsed_sec,
        diagnostic_snapshot.get());
      RequestMetricsInput input;
      input.request_id = job.request_id;
      input.goal_seq = job.goal_seq;
      input.status = final_class == kResultSuccess ? "success" : "failure";
      input.result_class = final_class;
      input.detail = detail;
      input.started_steady_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        job.received.time_since_epoch())
        .count();
      input.finished_steady_ns = SteadyNowNs();
      input.commit_steady_ns = commit_steady_ns;
      input.queue_wait_ms = queue_wait_ms;
      input.search_ms = search_ms;
      input.validation_ms = validation_ms;
      input.commit_ms = commit_ms;
      input.search = search_stats;
      input.snapshot = diagnostic_snapshot;
      input.search_status = search_status;
      input.search_detail = search_detail;
      input.stages = last_search.stages;
      input.max_planning_time_s = config_.max_planning_time_sec;
      input.fine_resolution_m = config_.terrain.resolution;
      input.coarse_resolution_m = config_.coarse.coarse_resolution;
      input.coarse_guidance = config_.search.use_coarse_guidance;
      input.search_backend = config_.search_backend;
      input.kinematic_checked = config_.search_backend != "graph_a_star";
      input.detailed_query_timing = config_.detailed_query_timing;
      PublishMetrics(BuildRequestMetricsJson(input));
      PublishStage(
        job.request_id, job.goal_seq, "finished", finished_elapsed_sec,
        expansions, final_class);
    };

  auto finish = [&](bool success, const std::string & result_class,
      const std::string & detail) {
      if (finished) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - started);
      result->planning_time = ToRosDuration(elapsed);
      finished_elapsed_sec = DurationSeconds(elapsed);
      finished_detail = detail;
      const bool canceled =
        job.cancel && job.cancel->load(std::memory_order_relaxed);
      // ROS Humble's ServerGoalHandle::canceled() throws unless a ROS cancel
      // request was actually received. The internal stop/deactivation flag is not
      // a ROS cancel: executing goals must be aborted, not falsely canceled, or
      // the client never receives a terminal state.
      bool ros_cancel_requested = false;
      if (job.goal_handle) {
        try {
          ros_cancel_requested = job.goal_handle->is_canceling();
        } catch (const std::exception & exception) {
          RCLCPP_WARN(
            get_logger(), "is_canceling() failed: %s",
            exception.what());
        }
      }
      if (!success || canceled) {
        // A cancellation racing the final validation must never leave a known
        // path published or attached to a canceled action result.
        result->path.poses.clear();
        PublishEmptyPath();
      }
      std::string final_class =
        (canceled || ros_cancel_requested) ? kResultCanceled : result_class;
      std::string terminal_error;
      try {
        if (job.goal_handle && job.goal_handle->is_active()) {
          if (ros_cancel_requested) {
            job.goal_handle->canceled(result);
          } else if (success && !canceled) {
            job.goal_handle->succeed(result);
          } else {
            job.goal_handle->abort(result);
          }
        }
      } catch (const std::exception & exception) {
        terminal_error = exception.what();
        RCLCPP_ERROR(
          get_logger(), "failed to finalize action goal: %s",
          exception.what());
      } catch (...) {
        terminal_error = "unknown terminal exception";
        RCLCPP_ERROR(get_logger(), "failed to finalize action goal");
      }
      if (success && !canceled && !terminal_error.empty()) {
        // The SDK refused the success transition. Do not report success or leave
        // a path that looks committed to downstream consumers.
        result->path.poses.clear();
        PublishEmptyPath();
        final_class = kResultInternalError;
        finished_detail = "action terminal transition failed: " + terminal_error;
      }
      // Mark finished only after the terminal transition attempt. A thrown
      // succeed() is therefore never reported as a completed success.
      finished = true;
      report_finished(final_class, finished_detail);
    };

  try {
    if (!job.goal_handle) {
      finish(false, kResultInvalidGoal, "goal handle is null");
      return;
    }
    const auto goal = job.goal_handle->get_goal();
    if (!goal) {
      finish(false, kResultInvalidGoal, "goal is null");
      return;
    }
    if (job.goal_handle->is_canceling()) {
      job.cancel->store(true, std::memory_order_relaxed);
    }
    if (!goal->activate_threading) {
      // This should be handled by HandleAccepted, but keep the action semantics
      // correct even if a future caller queues it.
      finish(true, kResultDeactivated, "empty path");
      return;
    }
    if (job.cancel->load(std::memory_order_relaxed) ||
      shutting_down_.load(std::memory_order_relaxed))
    {
      finish(false, kResultCanceled, "canceled before planning started");
      return;
    }

    Point3 start;
    double start_heading = 0.0;
    bool start_has_heading = false;
    std::string detail;
    if (!ResolveRequestPose(
        goal->start, true, false, &start, &start_heading,
        &start_has_heading, &detail))
    {
      finish(false, kResultInvalidStart, detail);
      return;
    }
    Point3 goal_position;
    double goal_heading = 0.0;
    bool goal_has_heading = false;
    if (!ResolveRequestPose(
        goal->goal, false, true, &goal_position,
        &goal_heading, &goal_has_heading, &detail))
    {
      finish(false, kResultInvalidGoal, detail);
      return;
    }

    if (config_.offline.enabled &&
      !static_map_ready_.load(std::memory_order_relaxed))
    {
      // A request that beats the one-shot static builder fails closed instead
      // of waiting on the worker or inventing an empty terrain.
      finish(false, kResultNotReady, "offline static map is not ready");
      return;
    }
    {
      std::string source_detail;
      if (!TerrainSourceReady(&source_detail)) {
        finish(
          false, kResultNotReady,
          "terrain source is not ready for live planning: " + source_detail);
        return;
      }
    }
    PublishStage(
      job.request_id, job.goal_seq, "acquiring_snapshot", 0.0, 0,
      "in_progress");
    // Exactly one latest-snapshot acquisition for the planning job. The pinned
    // shared_ptr keeps the whole search/validation chain on one immutable map.
    const auto snapshot = snapshot_store_ ? snapshot_store_->AcquireLatest() :
      nullptr;
    if (!snapshot || !snapshot->IsReady() || !snapshot->fine_terrain()) {
      finish(false, kResultNotReady, "no usable terrain snapshot");
      return;
    }
    diagnostic_snapshot = snapshot;
    if (snapshot->fine_terrain()->frame_id() != config_.global_frame) {
      finish(
        false, kResultFrameMismatch,
        "snapshot frame does not match ros.global_frame");
      return;
    }
    if (!snapshot->HasReadyDistanceField() || !snapshot->distance_field() ||
      !snapshot->distance_field()->ready())
    {
      finish(false, kResultNotReady, "pinned distance field is not ready");
      return;
    }

    SearchBudget budget;
    budget.has_deadline = true;
    budget.deadline = started + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        config_.max_planning_time_sec));
    budget.cancel = job.cancel;
    budget.max_samples = config_.max_samples;

    PublishStage(
      job.request_id, job.goal_seq, "searching", 0.0, 0,
      "in_progress");
    const auto search_start = std::chrono::steady_clock::now();
    SearchResult search;
    if (config_.search_backend == "graph_a_star") {
      GraphSearchRequest request;
      request.use_points = true;
      request.nearest_layer = true;
      request.start_point = start;
      request.goal_point = goal_position;
      request.start_point.z -= config_.vehicle.base_contact_height_m;
      request.goal_point.z -= config_.vehicle.base_contact_height_m;
      request.budget = budget;
      search = graph_search_.Search(
        *snapshot, *snapshot->distance_field(), request);
      for (auto & pose : search.path) {
        pose.position.z += config_.vehicle.base_contact_height_m;
      }
      search.search_algorithm = "ground_graph_astar";
      search.traversability_model = "ground_graph_nodes_only";
      search.stages.push_back("graph_a_star_search");
    } else {
      AckermannHybridAStar planner(config_.vehicle, config_.wheel,
        config_.obstacles, config_.search);
      if (!planner.evaluator().IsConfigured()) {
        finish(
          false, kResultVehicleUnconfigured,
          planner.evaluator().configuration_detail());
        return;
      }
      HybridSearchRequest request;
      request.start = start;
      request.start_heading = start_heading;
      request.goal = goal_position;
      request.goal_heading = goal_heading;
      request.has_goal_heading = goal_has_heading;
      request.budget = budget;
      search = planner.Search(*snapshot, request);
    }
    search_ms = DurationToMs(std::chrono::steady_clock::now() - search_start);
    search_stats = search.stats;
    last_search = search;
    search_status = SearchStatusToString(search.status);
    search_detail = search.detail;
    expansions = search.stats.expanded_nodes;

    if (job.cancel->load(std::memory_order_relaxed) ||
      search.status == SearchStatus::kCancelled)
    {
      finish(false, kResultCanceled, "planner canceled during search");
      return;
    }
    if (!search.ok() || search.path.empty()) {
      finish(false, SearchStatusToResultClass(search.status), search.detail);
      return;
    }

    PublishStage(
      job.request_id, job.goal_seq, "validating_pinned_snapshot", 0.0,
      expansions, "in_progress");
    const auto validation_start = std::chrono::steady_clock::now();
    auto validation = ValidatePinned(
      job, snapshot, search.path,
      budget.deadline);
    validation_ms += DurationToMs(
      std::chrono::steady_clock::now() - validation_start);
    {
      if (!validation.ok) {
        finish(
          false, LatestValidationResultClass(validation.status),
          validation.detail);
        return;
      }
      if (validation.path.empty() || !validation.snapshot) {
        finish(false, kResultLatestInvalid, "validated path is empty");
        return;
      }

      result->path =
        BuildRosPath(
        validation.path,
        validation.snapshot->fine_terrain()->frame_id(),
        ToRosTime(get_clock()->now()));
      diagnostic_snapshot = validation.snapshot;
      // Expensive rendering happens before serialized final acceptance. The
      // commit callback below performs only the prepared path publication and
      // the terminal action transition.
      PublishPlanDebugOutputs(
        *validation.snapshot, search, validation,
        job.request_id, job.goal_seq);
      if (job.cancel->load(std::memory_order_relaxed) ||
        shutting_down_.load(std::memory_order_relaxed))
      {
        finish(false, kResultCanceled, "canceled before final commit");
        return;
      }
      finished_detail =
        "validated=" + std::to_string(validation.path.size()) +
        " poses; attempts=" + std::to_string(validation.attempts);
      if (config_.search_backend == "graph_a_star") {
        finished_detail += "; backend=graph_a_star; kinematic_checked=false";
      }
      const auto commit_start = std::chrono::steady_clock::now();
      std::string accept_detail;
      CommitDeliveryHooks hooks;
      hooks.accept_commit = [&](const std::function<bool()> & mark_committed) {
          // SnapshotStore::CommitLatest() holds its handoff mutex here, so no
          // publication can land between acceptance and the committed mark.
          // The pinned-A proof does NOT compare this latest store object; the
          // handoff only preserves commit/cancel linearization. Acquiring
          // job_mutex_ establishes store -> job order. This hook only performs
          // the short in-memory epoch/cancel/identity check and then marks the
          // committed phase under the SAME job guard. It never queries or calls
          // the action SDK, publishes, or blocks.
          std::lock_guard<std::mutex> lock(job_mutex_);
          if (finished) {
            accept_detail = "job lifecycle is no longer current";
            return false;
          }
          if (shutting_down_.load(std::memory_order_relaxed) ||
            job.lifecycle_epoch != lifecycle_epoch_)
          {
            accept_detail = "job lifecycle is no longer current";
            return false;
          }
          if (!active_job_identity_.has_value() ||
            *active_job_identity_ != job.identity)
          {
            accept_detail = "job lifecycle is no longer current";
            return false;
          }
          if (job.cancel && job.cancel->load(std::memory_order_relaxed)) {
            accept_detail = "job canceled";
            return false;
          }
          if (!job.goal_handle) {
            accept_detail = "job lifecycle is no longer current";
            return false;
          }
          const auto now = std::chrono::steady_clock::now();
          const auto elapsed = std::chrono::duration_cast<
            std::chrono::nanoseconds>(now - started);
          result->planning_time = ToRosDuration(elapsed);
          finished_elapsed_sec = DurationSeconds(elapsed);
          // The cancel handler takes this same job_mutex_, so it either observes
          // kCommitted after this call or wins first and is caught by the cancel
          // token check above. There is no check-to-mark cancel gap.
          const bool marked = mark_committed();
          if (!marked && accept_detail.empty()) {
            accept_detail = "job lifecycle is no longer current";
          }
          return marked;
        };
      // These hooks run only after CommitAndDeliver() has released the store
      // handoff mutex and job_mutex_.
      hooks.terminal_success = [&]() {job.goal_handle->succeed(result);};
      hooks.terminal_failure = [&]() -> bool {
          if (!job.goal_handle) {
            return false;
          }
          try {
            if (job.goal_handle->is_active()) {
              job.goal_handle->abort(result);
            }
            // Positive evidence: the handle is no longer active after the abort
            // attempt. It may already have reached SUCCEEDED before the DDS
            // result transport threw, which is still a known terminal state.
            return !job.goal_handle->is_active();
          } catch (const std::exception & exception) {
            RCLCPP_WARN(
              get_logger(),
              "terminal evidence query/abort failed: %s",
              exception.what());
            return false;
          } catch (...) {
            return false;
          }
        };
      hooks.publish_path = [&]() {PublishPreparedPath(result->path);};
      const CommitDeliveryResult delivery_result =
        CommitValidatedPlan(job, validation, hooks);
      commit_ms += DurationToMs(
        std::chrono::steady_clock::now() - commit_start);
      switch (delivery_result.outcome) {
        case CommitDeliveryOutcome::kTerminalSuccess:
          finished = true;
          commit_steady_ns = SteadyNowNs();
          if (!delivery_result.detail.empty()) {
            RCLCPP_WARN(
              get_logger(),
              "private path delivery detail after action success: %s",
              delivery_result.detail.c_str());
          }
          // Diagnostics/stage are deliberately outside both guards.
          report_finished(kResultSuccess, finished_detail);
          return;
        case CommitDeliveryOutcome::kCommitRejected:
          if (!accept_detail.empty()) {
            finish(false, kResultCanceled, accept_detail);
            return;
          }
          if (delivery_result.detail.find("superseded") !=
            std::string::npos)
          {
            finish(false, kResultCanceled, delivery_result.detail);
            return;
          }
          // With pinned-A proof there is no latest-map predicate to retry.
          // Rejection here is lifecycle/ownership or an unavailable store.
          finish(
            false, kResultInternalError,
            "pinned-A commit rejected: " + delivery_result.detail);
          return;
        case CommitDeliveryOutcome::kStoreUnavailable:
          finish(false, kResultNotReady, delivery_result.detail);
          return;
        case CommitDeliveryOutcome::kSkippedStaleIdentity:
          finish(false, kResultCanceled, delivery_result.detail);
          return;
        case CommitDeliveryOutcome::kDeliveryErrorKnownTerminal:
          // The success transport threw, but terminality is known (possibly
          // already SUCCEEDED). Never report success and never claim an
          // abort/cancel that did not happen.
          finish(false, kResultInternalError, delivery_result.detail);
          return;
        case CommitDeliveryOutcome::kTransportUnresolved: {
            // Terminality is unproven. Do not finish/clear/release: the planning
            // loop keeps this goal's failed reservation fail-closed. Record an
            // explicit desynchronized-delivery diagnostic for recovery.
            const double unresolved_elapsed =
              DurationSeconds(std::chrono::steady_clock::now() - started);
            RCLCPP_ERROR(
              get_logger(),
              "grid plan goal_seq=%" PRIu64 " delivery unresolved: %s; reservation "
              "retained and new admission is blocked until restart",
              static_cast<std::uint64_t>(job.goal_seq),
              delivery_result.detail.c_str());
            PublishStage(
              job.request_id, job.goal_seq, "delivery_unresolved",
              unresolved_elapsed, expansions, kResultInternalError);
            PublishDiagnostics(
              job.request_id, job.goal_seq,
              "delivery_unresolved", kResultInternalError,
              delivery_result.detail, unresolved_elapsed,
              diagnostic_snapshot.get());
            result->path.poses.clear();
            return;
          }
      }
      // Unreachable: every switch outcome above returns. Kept explicit so a
      // future outcome addition cannot silently fall through to a latest-map
      // retry, which the pinned-A policy forbids.
      finish(false, kResultInternalError, "pinned-A commit fell through");
      return;
    }
  } catch (const std::exception & exception) {
    finish(false, kResultInternalError, exception.what());
  } catch (...) {
    finish(false, kResultInternalError, "unknown exception");
  }
}

LatestValidationConfig GridGlobalPlanner::MakeLatestValidationConfig() const
{
  LatestValidationConfig validation_config;
  validation_config.vehicle = config_.vehicle;
  validation_config.wheel = config_.wheel;
  validation_config.obstacles = config_.obstacles;
  validation_config.validation = config_.validation;
  validation_config.max_latest_validation_time_sec =
    config_.max_latest_validation_time_sec;
  validation_config.max_latest_validation_retries =
    config_.max_latest_validation_retries;
  validation_config.corridor_tile_size_m = config_.corridor_tile_size_m;
  validation_config.corridor_max_samples = config_.corridor_max_samples;
  validation_config.body_sweep_spacing_m = config_.body_sweep_spacing_m;
  return validation_config;
}

LatestValidationResult GridGlobalPlanner::ValidatePinned(
  const PlanningJob & job,
  const std::shared_ptr<const MapSnapshot> & pinned_snapshot,
  const std::vector<PathPose> & searched_path,
  std::chrono::steady_clock::time_point overall_deadline)
{
  // A was acquired exactly once before search. Both backends validate only
  // that same immutable object: no store acquisition, latest-map predicate, or
  // retry can substitute a newer map B.
  const LatestValidationConfig validation_config = MakeLatestValidationConfig();
  if (config_.search_backend == "graph_a_star") {
    std::vector<TerrainNodeId> node_path;
    node_path.reserve(searched_path.size());
    for (const auto & pose : searched_path) {
      node_path.push_back(pose.support_nodes[0]);
    }
    return ValidateLatestGraphSnapshot(
      validation_config, pinned_snapshot, searched_path, node_path,
      overall_deadline, job.cancel, [pinned_snapshot]() {
        return pinned_snapshot;
      });
  }
  return ValidatePinnedSnapshot(
    validation_config, pinned_snapshot, searched_path, overall_deadline,
    job.cancel);
}

// ---------------------------------------------------------------------------
// Outputs and diagnostics
// ---------------------------------------------------------------------------


void GridGlobalPlanner::PublishMetrics(const std::string & json)
{
  if (!metrics_publisher_) {
    return;
  }
  std_msgs::msg::String message;
  message.data = json;
  metrics_publisher_->publish(message);
}

void GridGlobalPlanner::PublishUpdateMetrics(
  const MapUpdateMetrics & metrics)
{
  if (!metrics_publisher_) {
    return;
  }
  std::ostringstream out;
  out << "{";
  out << "\"schema_version\":" << kMetricsSchemaVersion;
  out << ",\"record_type\":\"map_update\"";
  out << ",\"backend\":\"grid\"";
  out << ",\"mode\":\"map_update\"";
  out << ",\"kind\":" << JsonString(MapUpdateMetricKindToString(metrics.kind));
  const char * update_status = "processed";
  if (metrics.publish_attempted) {
    update_status = metrics.publish_succeeded ?
      "published" :
      (metrics.backpressured ? "backpressured" :
      "publish_failed");
  }
  out << ",\"status\":" << JsonString(update_status);
  out << ",\"source_id\":" << JsonString(metrics.source_id);
  out << ",\"source_sequence\":" << metrics.source_sequence;
  out << ",\"started_at_utc\":" << JsonString(UtcNowIso8601());
  out << ",\"monotonic_start_ns\":" << metrics.started_steady_ns;
  out << ",\"monotonic_end_ns\":" << metrics.completed_steady_ns;
  out << ",\"input_received_steady_ns\":"
      << metrics.input_received_steady_ns;
  out << ",\"queue_enqueue_ns\":" << metrics.enqueued_steady_ns;
  out << ",\"publish_steady_ns\":" << metrics.published_steady_ns;
  out << ",\"queue_wait_ms\":" << JsonNumber(metrics.queue_wait_ms);
  out << ",\"duration_ms\":" << JsonNumber(metrics.total_ms);
  out << ",\"obstacle_version\":" << metrics.version;
  out << ",\"publish_attempted\":"
      << (metrics.publish_attempted ? "true" : "false");
  out << ",\"publish_succeeded\":"
      << (metrics.publish_succeeded ? "true" : "false");
  out << ",\"backpressured\":"
      << (metrics.backpressured ? "true" : "false");
  out << ",\"detail\":" << JsonString(metrics.detail);
  out << ",\"phases_ms\":{";
  out << "\"contribution\":" << JsonNumber(metrics.contribution_ms);
  out << ",\"cow\":" << JsonNumber(metrics.cow_ms);
  out << ",\"coarse_overlay\":" << JsonNumber(metrics.coarse_ms);
  out << ",\"distance_field\":" << JsonNumber(metrics.distance_ms);
  out << ",\"snapshot\":" << JsonNumber(metrics.snapshot_ms);
  out << ",\"publish\":" << JsonNumber(metrics.publish_ms);
  out << ",\"reclaim\":" << JsonNumber(metrics.reclaim_ms);
  out << ",\"total\":" << JsonNumber(metrics.total_ms);
  out << "}";
  out << ",\"counts\":{";
  out << "\"live_versions\":" << metrics.live_versions;
  out << ",\"retired_versions\":" << metrics.retired_versions;
  out << ",\"reclaimed_versions_total\":"
      << metrics.reclaimed_versions_total;
  out << ",\"version_counts_available\":"
      << (metrics.version_counts_available ? "true" : "false");
  out << ",\"dirty_nodes\":" << metrics.dirty_nodes;
  out << ",\"dirty_cells\":" << metrics.dirty_cells;
  out << ",\"updated_tiles\":" << metrics.updated_tiles;
  out << ",\"copy_bytes_estimated\":" << metrics.copy_bytes_estimated;
  out << "}";
  out << ",\"terrain\":{";
  out << "\"measured\":" << (metrics.terrain.measured ? "true" : "false");
  out << ",\"status\":" << JsonString(metrics.terrain.status);
  out << ",\"detail\":" << JsonString(metrics.terrain.detail);
  out << ",\"input_points\":" << metrics.terrain.input_points;
  out << ",\"valid_points\":" << metrics.terrain.valid_points;
  out << ",\"cells\":" << metrics.terrain.cells;
  out << ",\"surfaces\":" << metrics.terrain.surfaces;
  out << ",\"connections\":" << metrics.terrain.connections;
  out << ",\"revision\":" << metrics.terrain.revision;
  out << ",\"content_hash\":" << metrics.terrain.content_hash;
  out << ",\"observation_sequence\":"
      << metrics.terrain.observation_sequence;
  out << ",\"sensor_stamp_ns\":" << metrics.terrain.sensor_stamp_ns;
  out << ",\"received_steady_ns\":"
      << metrics.terrain.received_steady_ns;
  out << ",\"phases_ms\":{";
  out << "\"total\":" << JsonNumber(metrics.terrain.total_ms);
  out << ",\"input_validation\":"
      << JsonNumber(metrics.terrain.input_validation_ms);
  out << ",\"content_hash\":" << JsonNumber(metrics.terrain.content_hash_ms);
  out << ",\"binning\":" << JsonNumber(metrics.terrain.binning_ms);
  out << ",\"segmentation\":" << JsonNumber(metrics.terrain.segmentation_ms);
  out << ",\"segmentation_fit_inclusive\":"
      << JsonNumber(metrics.terrain.segmentation_fit_ms_inclusive);
  out << ",\"segmentation_fit_exclusive\":"
      << JsonNumber(metrics.terrain.segmentation_fit_ms_exclusive);
  out << ",\"attribute\":" << JsonNumber(metrics.terrain.attribute_ms);
  out << ",\"attribute_fit_inclusive\":"
      << JsonNumber(metrics.terrain.attribute_fit_ms_inclusive);
  out << ",\"attribute_fit_exclusive\":"
      << JsonNumber(metrics.terrain.attribute_fit_ms_exclusive);
  out << ",\"connections\":" << JsonNumber(metrics.terrain.connections_ms);
  out << "},\"inclusive_phases\":{\"segmentation\":true,"
    "\"attribute\":true}";
  out << "}";
  out << ",\"resources\":{";
  out << "\"available\":"
      << (metrics.resources.available ? "true" : "false");
  out << ",\"process_cpu_ms\":"
      << JsonNumber(metrics.resources.process_cpu_ms);
  out << ",\"thread_cpu_ms\":"
      << JsonNumber(metrics.resources.thread_cpu_ms);
  out << ",\"peak_rss_mib\":"
      << JsonNumber(metrics.resources.peak_rss_mib);
  out << ",\"scope\":\"process_self_and_calling_thread_boundary\"";
  out << "}";
  std::vector<std::string> measurements;
  measurements.push_back(
    "\"obstacle_queue_wait_ms\":" +
    MeasurementJson(
      metrics.enqueued_steady_ns > 0,
      metrics.queue_wait_ms, "ms",
      "queue instant was not recorded"));
  measurements.push_back(
    "\"obstacle_incremental_update_ms\":" +
    MeasurementJson(
      metrics.kind == MapUpdateMetrics::Kind::kPublish,
      metrics.total_ms, "ms",
      "not a publish record"));
  measurements.push_back(
    "\"obstacle_contribution_ms\":" +
    MeasurementJson(
      metrics.contribution_ms > 0.0,
      metrics.contribution_ms, "ms",
      "no contribution phase on this record"));
  measurements.push_back(
    "\"coarse_map_aggregation_ms\":" +
    MeasurementJson(
      metrics.coarse_ms > 0.0, metrics.coarse_ms, "ms",
      "coarse overlay not run"));
  measurements.push_back(
    "\"snapshot_publish_ms\":" +
    MeasurementJson(
      metrics.publish_attempted, metrics.publish_ms, "ms",
      "publication not attempted"));
  measurements.push_back(
    "\"update_batch_wall_ms\":" +
    MeasurementJson(
      metrics.total_ms > 0.0, metrics.total_ms, "ms",
      "no elapsed time"));
  measurements.push_back(
    "\"update_batch_cpu_ms\":" +
    MeasurementJson(
      metrics.resources.available,
      metrics.resources.process_cpu_ms, "ms",
      "getrusage unavailable"));
  measurements.push_back(
    "\"update_copy_bytes\":" +
    MeasurementJson(
      metrics.kind == MapUpdateMetrics::Kind::kPublish,
      static_cast<double>(metrics.copy_bytes_estimated),
      "bytes",
      "no copy estimate on this record"));
  measurements.push_back(
    "\"updated_tiles\":" +
    MeasurementJson(
      metrics.kind == MapUpdateMetrics::Kind::kPublish,
      static_cast<double>(metrics.updated_tiles), "count",
      "no tile count on this record"));
  measurements.push_back(
    "\"changed_cells\":" +
    MeasurementJson(
      metrics.kind == MapUpdateMetrics::Kind::kPublish,
      static_cast<double>(metrics.dirty_cells), "count",
      "no changed-cell count on this record"));
  measurements.push_back(
    "\"coarse_map_ms\":" +
    MeasurementJson(
      metrics.coarse_ms > 0.0, metrics.coarse_ms, "ms",
      "coarse overlay not run"));
  measurements.push_back(
    "\"distance_field_ms\":" +
    MeasurementJson(
      metrics.distance_ms > 0.0, metrics.distance_ms, "ms",
      "distance field not run"));
  measurements.push_back(
    "\"distance_field_incremental_ms\":" +
    MeasurementJson(
      metrics.distance_ms > 0.0, metrics.distance_ms, "ms",
      "distance field not run"));
  measurements.push_back(
    "\"terrain_build_ms\":" +
    MeasurementJson(
      metrics.terrain.measured, metrics.terrain.total_ms,
      "ms", "terrain build not part of this record"));
  out << ",\"measurements\":{";
  for (std::size_t index = 0; index < measurements.size(); ++index) {
    if (index != 0U) {
      out << ",";
    }
    out << measurements[index];
  }
  out << "}";
  out << "}";
  PublishMetrics(out.str());
}

void GridGlobalPlanner::PublishStage(
  std::uint64_t request_id,
  std::uint64_t goal_seq,
  const std::string & stage,
  double elapsed_sec,
  std::size_t expansions,
  const std::string & result_class)
{
  if (!config_.publish_stage_debug || !stage_publisher_) {
    return;
  }
  std::ostringstream stream;
  stream << "request_id=" << request_id << ";goal_seq=" << goal_seq
         << ";stage=" << Sanitize(stage) << ";stage_elapsed_sec="
         << std::fixed << std::setprecision(6) << elapsed_sec
         << ";expansions=" << expansions
         << ";result_class=" << Sanitize(result_class);
  std_msgs::msg::String message;
  message.data = stream.str();
  stage_publisher_->publish(message);
}

void GridGlobalPlanner::PublishDiagnostics(
  std::uint64_t request_id, std::uint64_t goal_seq,
  const std::string & stage, const std::string & result_class,
  const std::string & detail, double elapsed_sec,
  const MapSnapshot * snapshot)
{
  if (!diagnostics_publisher_) {
    return;
  }
  std::ostringstream stream;
  stream << "request_id=" << request_id << ";goal_seq=" << goal_seq
         << ";stage=" << Sanitize(stage)
         << ";result_class=" << Sanitize(result_class)
         << ";detail=" << Sanitize(detail) << ";planning_time_sec="
         << std::fixed << std::setprecision(6) << elapsed_sec;
  if (snapshot != nullptr && snapshot->fine_terrain()) {
    stream << ";terrain_revision=" << snapshot->fine_terrain()->revision()
           << ";content_hash=" << snapshot->fine_terrain()->content_hash()
           << ";obstacle_version=" << snapshot->obstacle_version()
           << ";map_resolution_m=" << snapshot->fine_terrain()->resolution()
           << ";frame_id=" << snapshot->fine_terrain()->frame_id();
  }
  std_msgs::msg::String message;
  message.data = stream.str();
  diagnostics_publisher_->publish(message);
}

CommitDeliveryResult GridGlobalPlanner::CommitValidatedPlan(
  const PlanningJob & job, const LatestValidationResult & validation,
  const CommitDeliveryHooks & hooks)
{
  CommitDeliveryResult result;
  if (!validation.snapshot || !validation.snapshot->fine_terrain()) {
    result.outcome = CommitDeliveryOutcome::kCommitRejected;
    result.detail = "validated snapshot is unavailable";
    return result;
  }
  // Pinned-A production policy: the proof names only A and keeps it alive
  // through the atomic commit/delivery. It deliberately does not compare the
  // store's latest snapshot, so B (including a new blocker on A's route) cannot
  // reject or revalidate this already-accepted request.
  const LatestCommitProof proof = BuildPinnedCommitProof(validation);
  if (proof.kind == LatestCommitProofKind::kNone) {
    result.outcome = CommitDeliveryOutcome::kCommitRejected;
    result.detail = "no pinned-snapshot commit proof";
    return result;
  }
  if (!snapshot_store_) {
    result.outcome = CommitDeliveryOutcome::kStoreUnavailable;
    result.detail = "snapshot store is unavailable";
    return result;
  }
  return delivery_.CommitAndDeliver(
    *snapshot_store_, proof, job.identity,
    hooks);
}

void GridGlobalPlanner::PublishPlanDebugOutputs(
  const MapSnapshot & snapshot, const SearchResult & search,
  const LatestValidationResult & validation, std::uint64_t request_id,
  std::uint64_t goal_seq)
{
  (void)request_id;
  (void)goal_seq;
  // Visualization must never turn a valid plan into an action failure, so each
  // output is isolated and failures are reported only as a warning.
  auto safe = [this](const char * label, const auto & operation) {
      try {
        operation();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(), "failed to publish %s: %s", label,
          exception.what());
      }
    };
  // Terrain display follows map publication on its own bounded-rate timer.
  // It is deliberately not repainted here: a pinned request snapshot A must
  // never overwrite the view of a newer published map B after the job ends.
  if (config_.publish_coarse_debug) {
    safe("coarse", [this, &snapshot]() {PublishCoarse(snapshot);});
  }
  if (config_.publish_corridor_debug) {
    safe(
      "corridor", [this, &snapshot, &validation]() {
        PublishCorridor(snapshot, validation.path);
      });
  }
  if (config_.publish_cost_debug) {
    safe("cost", [this, &validation]() {PublishCost(validation.path);});
  }
  safe(
    "version", [this, &snapshot]() {
      PublishVersion(snapshot, "pinned_snapshot");
    });
  safe(
    "fallback", [this, &search, &validation]() {
      PublishFallback(search, validation);
    });
  if (config_.publish_wheel_debug && config_.search_backend != "graph_a_star") {
    safe(
      "wheel_debug", [this, &snapshot, &validation]() {
        PublishWheelDebug(snapshot, validation.path);
      });
  }
}

void GridGlobalPlanner::PublishPlanOutputs(
  const MapSnapshot & snapshot, const SearchResult & search,
  const LatestValidationResult & validation, std::uint64_t request_id,
  std::uint64_t goal_seq)
{
  PublishPlanDebugOutputs(snapshot, search, validation, request_id, goal_seq);
  PublishPath(validation.path);
}

void GridGlobalPlanner::PublishPath(const std::vector<PathPose> & path)
{
  if (!path_publisher_) {
    return;
  }
  path_publisher_->publish(
    BuildRosPath(path, config_.global_frame, ToRosTime(get_clock()->now())));
}

void GridGlobalPlanner::PublishPreparedPath(const nav_msgs::msg::Path & path)
{
  if (!path_publisher_) {
    return;
  }
  // The message was built and validated before the final commit. This runs
  // after the store handoff mutex and job_mutex_ are released; the commit
  // delivery path serializes it with the empty deactivation clear.
  path_publisher_->publish(path);
}

void GridGlobalPlanner::PublishInitialSnapshotVisualization(
  const MapSnapshot & snapshot)
{
  auto safe = [this](const char * label, const auto & operation) {
      try {
        operation();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(), "failed to publish initial %s: %s", label,
          exception.what());
      } catch (...) {
        RCLCPP_WARN(
          get_logger(), "failed to publish initial %s: unknown error",
          label);
      }
    };
  if (config_.publish_terrain_debug) {
    // MASTER on: always publish this full frame, even when cost coloring and
    // its 2 Hz stream are off, so the base grid stays visible.
    safe("terrain", [this, &snapshot]() {PublishTerrain(snapshot);});
  }
  if (config_.publish_coarse_debug) {
    safe("coarse", [this, &snapshot]() {PublishCoarse(snapshot);});
  }
  safe(
    "version", [this, &snapshot]() {
      PublishVersion(snapshot, "offline_map_ready");
    });
}

void GridGlobalPlanner::PublishTerrain(const MapSnapshot & snapshot)
{
  if (!terrain_publisher_ || !config_.publish_terrain_debug) {
    return;
  }
  TerrainVisualizationOptions options;
  options.max_cells = config_.max_visualization_points;
  options.max_measurement_render_patches =
    config_.max_measurement_visualization_patches;
  options.static_map_only = config_.offline.enabled;
  // Use the exact same startup parameters and immutable boundary cache as the
  // offline evaluator; this display is a per-cell risk hint, not feasibility.
  options.boundary_penalty_distance_m =
    config_.wheel.boundary_penalty_distance_m;
  options.boundary_penalty_weight = config_.wheel.boundary_penalty_weight;
  // The production heat layer uses the exact graph obstacle soft-term values and
  // the merged (static+dynamic) field state, including in offline static-map
  // display mode. Cost display is optional: with it off this frame still uses
  // the static occupancy/boundary palette, and enabling it never changes graph
  // costs or search behavior.
  options.obstacle_heat_enabled = config_.publish_terrain_cost_debug;
  options.obstacle_penalty_distance_m =
    config_.graph.obstacle_penalty_distance_m;
  options.obstacle_penalty_weight = config_.graph.obstacle_penalty_weight;
  const auto result = BuildTerrainVisualization(
    snapshot, ToRosTime(get_clock()->now()), options);
  if (result.truncated || result.measurement_truncated ||
    result.invalid_cells != 0)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Terrain display: rendered_cells=%zu cell_limit=%zu truncated=%d "
      "rendered_measurement_patches=%zu measurement_limit=%zu "
      "measurement_truncated=%d invalid_geometry=%zu; display omissions do "
      "not change the planning map",
      result.rendered_cells, options.max_cells,
      static_cast<int>(result.truncated),
      result.rendered_measurement_patches,
      options.max_measurement_render_patches,
      static_cast<int>(result.measurement_truncated), result.invalid_cells);
  }
  terrain_publisher_->publish(result.markers);
}

void GridGlobalPlanner::OnTerrainVisualizationTimer()
{
  if (shutting_down_.load(std::memory_order_relaxed) ||
    !config_.publish_terrain_debug ||
    !config_.publish_terrain_cost_debug)
  {
    return;
  }
  // Taking the latest snapshot is the only operation under the refresh mutex.
  // The expensive geometry build and DDS publication deliberately happen after
  // the lock is released, and no worker/store/job/queue mutex is ever held.
  std::shared_ptr<const MapSnapshot> pending;
  {
    std::lock_guard<std::mutex> lock(terrain_refresh_mutex_);
    pending = terrain_refresh_.TakePending();
  }
  if (!pending) {
    return;
  }
  try {
    PublishTerrain(*pending);
    std::lock_guard<std::mutex> lock(terrain_refresh_mutex_);
    terrain_refresh_.MarkRendered(pending);
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(), "failed to publish terrain: %s", exception.what());
  } catch (...) {
    RCLCPP_WARN(get_logger(), "failed to publish terrain: unknown error");
  }
}

void GridGlobalPlanner::PublishCoarse(const MapSnapshot & snapshot)
{
  if (!coarse_publisher_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.global_frame;
  marker.header.stamp = ToRosTime(get_clock()->now());
  marker.ns = "coarse_surfaces";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.color.a = 0.8F;
  marker.color.r = 0.2F;
  marker.color.g = 0.8F;
  marker.color.b = 0.2F;

  // Display-only fallback: the offline planner intentionally creates its
  // MapSnapshot with a null coarse map. Build a transient coarse summary from
  // the immutable fine terrain just for this marker; never write it back to the
  // snapshot or use it for planning.
  std::shared_ptr<const CoarseTerrainMap> coarse = snapshot.coarse_terrain();
  if (!coarse && snapshot.fine_terrain()) {
    coarse = OverlayCoarseTerrain(
      nullptr, snapshot.fine_terrain(), {}, config_.coarse);
  }
  if (coarse) {
    const std::size_t limit =
      config_.max_visualization_points == 0 ?
      coarse->surfaces().size() :
      std::min(
      coarse->surfaces().size(),
      config_.max_visualization_points);
    const std::size_t stride =
      limit == 0 ?
      1 :
      std::max<std::size_t>(1, coarse->surfaces().size() / limit);
    for (std::size_t i = 0; i < coarse->surfaces().size() &&
      marker.points.size() < limit;
      i += stride)
    {
      const auto & surface = coarse->surfaces()[i];
      geometry_msgs::msg::Point point;
      point.x = surface.representative.x;
      point.y = surface.representative.y;
      point.z = surface.representative.z;
      marker.points.push_back(point);
      if (surface.status == TerrainStatus::kProhibited) {
        marker.colors.push_back(
          []() {
            std_msgs::msg::ColorRGBA color;
            color.a = 0.9F;
            color.r = 0.9F;
            color.g = 0.1F;
            color.b = 0.1F;
            return color;
          }());
      } else if (surface.status == TerrainStatus::kUnknown) {
        marker.colors.push_back(
          []() {
            std_msgs::msg::ColorRGBA color;
            color.a = 0.6F;
            color.r = 0.5F;
            color.g = 0.5F;
            color.b = 0.5F;
            return color;
          }());
      } else {
        marker.colors.push_back(marker.color);
      }
    }
  }
  markers.markers.push_back(marker);
  coarse_publisher_->publish(markers);
}

void GridGlobalPlanner::PublishCorridor(
  const MapSnapshot & snapshot,
  const std::vector<PathPose> & path)
{
  if (!corridor_publisher_ || !snapshot.fine_terrain()) {
    return;
  }
  std::vector<Point3> poses;
  poses.reserve(path.size());
  for (const PathPose & pose : path) {
    poses.push_back(pose.position);
  }
  CorridorBuilder builder(config_.search.corridor);
  const Corridor corridor = builder.BuildAroundPoses(
    *snapshot.fine_terrain(), poses,
    config_.search.corridor.initial_margin_m);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker cells_marker;
  cells_marker.header.frame_id = config_.global_frame;
  cells_marker.header.stamp = ToRosTime(get_clock()->now());
  cells_marker.ns = "corridor_cells";
  cells_marker.id = 0;
  cells_marker.type = visualization_msgs::msg::Marker::POINTS;
  cells_marker.action = visualization_msgs::msg::Marker::ADD;
  cells_marker.pose.orientation.w = 1.0;
  cells_marker.scale.x = std::max(0.02, snapshot.fine_terrain()->resolution());
  cells_marker.scale.y = cells_marker.scale.x;
  cells_marker.color.a = 0.5F;
  cells_marker.color.r = 0.1F;
  cells_marker.color.g = 0.4F;
  cells_marker.color.b = 1.0F;
  const std::size_t limit =
    config_.max_corridor_visualization_cells == 0 ?
    corridor.cells.size() :
    std::min(
    corridor.cells.size(),
    config_.max_corridor_visualization_cells);
  const std::size_t stride =
    limit == 0 ? 1 :
    std::max<std::size_t>(1, corridor.cells.size() / limit);
  for (std::size_t i = 0; i < corridor.cells.size() &&
    cells_marker.points.size() < limit;
    i += stride)
  {
    const Point3 center = FineCellCenter(
      corridor.cells[i], snapshot.fine_terrain()->origin_x(),
      snapshot.fine_terrain()->origin_y(),
      snapshot.fine_terrain()->resolution());
    geometry_msgs::msg::Point point;
    point.x = center.x;
    point.y = center.y;
    point.z = center.z;
    cells_marker.points.push_back(point);
  }
  markers.markers.push_back(cells_marker);

  visualization_msgs::msg::Marker route_marker;
  route_marker.header = cells_marker.header;
  route_marker.ns = "corridor_route";
  route_marker.id = 1;
  route_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  route_marker.action = visualization_msgs::msg::Marker::ADD;
  route_marker.pose.orientation.w = 1.0;
  route_marker.scale.x = 0.04;
  route_marker.color.a = 1.0F;
  route_marker.color.r = 1.0F;
  route_marker.color.g = 0.8F;
  route_marker.color.b = 0.1F;
  for (const PathPose & pose : path) {
    geometry_msgs::msg::Point point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    route_marker.points.push_back(point);
  }
  markers.markers.push_back(route_marker);
  corridor_publisher_->publish(markers);
}

void GridGlobalPlanner::PublishCost(const std::vector<PathPose> & path)
{
  if (!cost_publisher_) {
    return;
  }
  std_msgs::msg::Float64MultiArray message;
  message.layout.dim.resize(1);
  message.layout.dim[0].label = "pose";
  message.layout.dim[0].size = static_cast<std::uint32_t>(path.size());
  message.layout.dim[0].stride = 4;
  message.data.reserve(path.size() * 4);
  for (const PathPose & pose : path) {
    message.data.push_back(pose.cumulative_cost);
    message.data.push_back(pose.cumulative_distance_m);
    message.data.push_back(pose.heading);
    message.data.push_back(pose.steering_central_rad);
  }
  cost_publisher_->publish(message);
}

void GridGlobalPlanner::PublishVersion(
  const MapSnapshot & snapshot,
  const std::string & source)
{
  published_obstacle_version_.store(
    snapshot.obstacle_version(),
    std::memory_order_relaxed);
  if (!version_publisher_) {
    return;
  }
  std::ostringstream stream;
  stream << "source=" << Sanitize(source);
  if (snapshot.fine_terrain()) {
    stream << ";terrain_revision=" << snapshot.fine_terrain()->revision()
           << ";content_hash=" << snapshot.fine_terrain()->content_hash()
           << ";fine_resolution_m=" << snapshot.fine_terrain()->resolution()
           << ";frame_id=" << snapshot.fine_terrain()->frame_id();
  }
  stream << ";obstacle_version=" << snapshot.obstacle_version()
         << ";coarse_resolution_m=" << config_.coarse.coarse_resolution;
  std_msgs::msg::String message;
  message.data = stream.str();
  version_publisher_->publish(message);
}

void GridGlobalPlanner::PublishFallback(
  const SearchResult & search,
  const LatestValidationResult & validation)
{
  if (!fallback_publisher_) {
    return;
  }
  std::ostringstream stream;
  stream << "search_status=" << SearchStatusToString(search.status)
         << ";coarse_used=" << (search.coarse_used ? 1 : 0)
         << ";fine_fallback=" << (search.fine_fallback ? 1 : 0)
         << ";search_stages=";
  for (std::size_t i = 0; i < search.stages.size(); ++i) {
    if (i != 0) {
      stream << ",";
    }
    stream << Sanitize(search.stages[i]);
  }
  stream << ";latest_validation_replayed=" << (validation.replayed ? 1 : 0)
         << ";latest_validation_attempts=" << validation.attempts;
  std_msgs::msg::String message;
  message.data = stream.str();
  fallback_publisher_->publish(message);
}

void GridGlobalPlanner::PublishWheelDebug(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path)
{
  if (!wheel_debug_publisher_ || !snapshot.fine_terrain()) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  std::size_t marker_id = 0;
  const std::size_t max_poses =
    config_.max_visualization_points == 0 ?
    path.size() :
    std::min(path.size(), config_.max_visualization_points / 4);
  for (const PathPose & pose : path) {
    if (markers.markers.size() >= max_poses * kWheelCount) {
      break;
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const TerrainNodeId node_id = pose.support_nodes[wheel];
      const TerrainNode * node = snapshot.fine_terrain()->FindNode(node_id);
      if (node == nullptr) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = config_.global_frame;
      marker.header.stamp = ToRosTime(get_clock()->now());
      marker.ns = "wheel_support";
      marker.id = static_cast<int>(marker_id++);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = node->representative.x;
      marker.pose.position.y = node->representative.y;
      marker.pose.position.z = node->representative.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      marker.color.a = 0.9F;
      switch (wheel) {
        case 0:
          marker.color.r = 1.0F;
          break;
        case 1:
          marker.color.g = 1.0F;
          break;
        case 2:
          marker.color.b = 1.0F;
          break;
        default:
          marker.color.r = 1.0F;
          marker.color.g = 1.0F;
          break;
      }
      markers.markers.push_back(std::move(marker));
    }
  }
  wheel_debug_publisher_->publish(markers);
}

void GridGlobalPlanner::PublishEmptyPath()
{
  if (!path_publisher_) {
    return;
  }
  try {
    // Serialize the empty clear with every committed private path publication.
    // The helper checks the deactivation generation inside the publication
    // lock so an older blocked result cannot resurrect a stale path afterwards.
    delivery_.PublishEmptyPath([this]() {PublishPath({});});
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(), "failed to publish empty path clear: %s",
      exception.what());
  } catch (...) {
    RCLCPP_WARN(get_logger(), "failed to publish empty path clear");
  }
}

void GridGlobalPlanner::WarmGraphSearch(const MapSnapshot & snapshot)
{
  if (config_.search_backend != "graph_a_star") {
    return;
  }
  const SearchStatus status = graph_search_.Prepare(snapshot, SearchBudget{});
  if (status != SearchStatus::kSuccess) {
    RCLCPP_WARN(
      get_logger(), "graph cache prewarm deferred: %s",
      SearchStatusToString(status));
  }
}

void GridGlobalPlanner::OnMapPublished(
  std::shared_ptr<const MapSnapshot> snapshot)
{
  if (!snapshot || !snapshot->IsReady()) {
    return;
  }
  try {
    WarmGraphSearch(*snapshot);
    // An adopted offline baseline becomes ready to serve /get_plan only here:
    // this callback runs after SnapshotStore::TryPublish accepted it, and the
    // graph cache has already been warmed for the exact snapshot.
    if (adoption_publish_pending_.exchange(false, std::memory_order_relaxed)) {
      static_map_ready_.store(true, std::memory_order_relaxed);
      // The adopted baseline keeps its immediate transient-local publication so
      // late subscribers see terrain as soon as the map is ready, for MASTER on
      // regardless of cost coloring. Mark it rendered only for the dynamic cost
      // stream so a base-only view gets no redundant tick and no extra frame.
      PublishInitialSnapshotVisualization(*snapshot);
      {
        std::lock_guard<std::mutex> lock(terrain_refresh_mutex_);
        terrain_refresh_ = TerrainVisualizationRefresh{};
        if (config_.publish_terrain_debug &&
          config_.publish_terrain_cost_debug)
        {
          terrain_refresh_.MarkRendered(snapshot);
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "offline static map ready (worker-adopted baseline): "
        "load_elapsed_sec=%.3f version=%" PRIu64,
        StaticMapLoadElapsedSeconds(), snapshot->obstacle_version());
    } else if (config_.publish_terrain_debug &&
      config_.publish_terrain_cost_debug)
    {
      // Latest-wins, no geometry build and no DDS publish on the map worker.
      // Base-only display has no timer and therefore queues nothing.
      std::lock_guard<std::mutex> lock(terrain_refresh_mutex_);
      terrain_refresh_.Queue(snapshot);
    }
    PublishVersion(*snapshot, "map_update");
  } catch (...) {
    // A visualization/version publication must never terminate the map worker.
  }
}

}  // namespace grid_global_planner
