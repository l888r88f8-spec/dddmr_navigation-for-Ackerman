#include "local_planner/mppi_local_map_rollout.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace local_planner {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfiniteClearance =
    std::numeric_limits<double>::infinity();

bool finiteScalar(double value) { return std::isfinite(value); }

bool finitePose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite();
}

// An Eigen::Isometry3d does not enforce its contract when callers write into
// linear()/matrix(). Context and initial-pose inputs therefore need an explicit
// proper-rigid check: orthonormal rotation, determinant +1, and a homogeneous
// last row. This rejects finite scale/shear/malformed-that-finite matrices.
bool rigidPose(const Eigen::Isometry3d& pose) {
  if (!finitePose(pose)) {
    return false;
  }
  const Eigen::Matrix3d rotation = pose.linear();
  if (!(rotation.transpose() * rotation)
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    return false;
  }
  if (std::abs(rotation.determinant() - 1.0) > 1e-6) {
    return false;
  }
  const Eigen::Matrix4d matrix = pose.matrix();
  return std::abs(matrix(3, 0)) <= 1e-9 &&
         std::abs(matrix(3, 1)) <= 1e-9 &&
         std::abs(matrix(3, 2)) <= 1e-9 &&
         std::abs(matrix(3, 3) - 1.0) <= 1e-9;
}

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

void rpyFromRotation(const Eigen::Matrix3d& rotation, double* roll,
                     double* pitch, double* yaw) {
  *roll = std::atan2(rotation(2, 1), rotation(2, 2));
  *pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  *yaw = std::atan2(rotation(1, 0), rotation(0, 0));
}

double yawFromRotation(const Eigen::Matrix3d& rotation) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rpyFromRotation(rotation, &roll, &pitch, &yaw);
  return yaw;
}

dddmr_local_map::TerrainQuery makeTerrainQuery(
    const MppiLocalMapContext& context, const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation) {
  dddmr_local_map::TerrainQuery query;
  query.stamp = context.query_stamp;
  query.position = position;
  rpyFromRotation(rotation, &query.roll, &query.pitch, &query.yaw);
  return query;
}

bool poseFromTerrainResult(const dddmr_local_map::TerrainResult& terrain,
                           Eigen::Isometry3d* pose) {
  if (!terrain.projected_position.allFinite() ||
      !terrain.projected_orientation.coeffs().allFinite()) {
    return false;
  }
  const double quaternion_norm = terrain.projected_orientation.norm();
  if (!finiteScalar(quaternion_norm) || quaternion_norm < 1e-9) {
    return false;
  }
  Eigen::Quaterniond orientation = terrain.projected_orientation;
  orientation.normalize();
  Eigen::Isometry3d projected = Eigen::Isometry3d::Identity();
  projected.linear() = orientation.toRotationMatrix();
  projected.translation() = terrain.projected_position;
  if (!finitePose(projected)) {
    return false;
  }
  *pose = projected;
  return true;
}

MppiLocalMapRolloutStatus statusForTerrainFailure(
    dddmr_local_map::TerrainStatus status, bool initial_pose) {
  switch (status) {
    case dddmr_local_map::TerrainStatus::kStale:
      return MppiLocalMapRolloutStatus::kStale;
    case dddmr_local_map::TerrainStatus::kOutside:
      return MppiLocalMapRolloutStatus::kOutside;
    default:
      return initial_pose ? MppiLocalMapRolloutStatus::kInitialPoseRejected
                          : MppiLocalMapRolloutStatus::kTerrainRejected;
  }
}

std::string terrainFailureDetail(
    const dddmr_local_map::TerrainResult& terrain, std::size_t step,
    bool initial_pose) {
  std::ostringstream stream;
  if (initial_pose) {
    stream << "initial pose rejected";
  } else {
    stream << "terrain rejected at step " << step;
  }
  stream << ": " << dddmr_local_map::toString(terrain.status);
  if (!terrain.detail.empty()) {
    stream << ": " << terrain.detail;
  }
  return stream.str();
}

MppiLocalMapRolloutResult rejectedResult(
    const MppiLocalMapContext& context, MppiLocalMapRolloutStatus status,
    const std::string& detail) {
  MppiLocalMapRolloutResult result;
  result.valid = false;
  result.status = status;
  result.detail = detail;
  result.map_version = context.mapVersion();
  result.map_stamp = context.mapStamp();
  return result;
}

}  // namespace

const char* toString(MppiEnvironmentBackend value) {
  switch (value) {
    case MppiEnvironmentBackend::kLegacyPoints:
      return "legacy_points";
    case MppiEnvironmentBackend::kLocalMap:
      return "local_map";
  }
  return "unknown";
}

const char* mppiEnvironmentBackendParameterName() {
  return "mppi.environment_backend";
}

bool parseMppiEnvironmentBackend(const std::string& value,
                                 MppiEnvironmentBackend* backend) {
  if (backend == nullptr) {
    return false;
  }
  if (value == "legacy_points") {
    *backend = MppiEnvironmentBackend::kLegacyPoints;
    return true;
  }
  if (value == "local_map") {
    *backend = MppiEnvironmentBackend::kLocalMap;
    return true;
  }
  return false;
}

const char* toString(MppiLocalMapRolloutStatus value) {
  switch (value) {
    case MppiLocalMapRolloutStatus::kOk:
      return "ok";
    case MppiLocalMapRolloutStatus::kMissingMap:
      return "missing_map";
    case MppiLocalMapRolloutStatus::kInvalidContext:
      return "invalid_context";
    case MppiLocalMapRolloutStatus::kInvalidTransform:
      return "invalid_transform";
    case MppiLocalMapRolloutStatus::kInvalidConfig:
      return "invalid_config";
    case MppiLocalMapRolloutStatus::kEmptyControls:
      return "empty_controls";
    case MppiLocalMapRolloutStatus::kInvalidControl:
      return "invalid_control";
    case MppiLocalMapRolloutStatus::kInvalidInitialPose:
      return "invalid_initial_pose";
    case MppiLocalMapRolloutStatus::kInitialPoseRejected:
      return "initial_pose_rejected";
    case MppiLocalMapRolloutStatus::kStale:
      return "stale";
    case MppiLocalMapRolloutStatus::kOutside:
      return "outside";
    case MppiLocalMapRolloutStatus::kTerrainRejected:
      return "terrain_rejected";
    case MppiLocalMapRolloutStatus::kNonFiniteResult:
      return "non_finite_result";
  }
  return "unknown";
}

bool MppiLocalMapContext::valid(std::string* detail) const {
  const auto fail = [detail](const char* message) {
    if (detail != nullptr) {
      *detail = message;
    }
    return false;
  };

  if (snapshot == nullptr) {
    return fail("snapshot is null");
  }
  if (terrain == nullptr) {
    return fail("terrain is null");
  }
  if (!finiteScalar(snapshot->stamp())) {
    return fail("snapshot stamp is not finite");
  }
  if (!finiteScalar(query_stamp)) {
    return fail("query_stamp is not finite");
  }
  if (!rigidPose(planner_to_map)) {
    return fail("planner_to_map is not a proper rigid transform");
  }
  if (!planner_to_map.inverse().matrix().allFinite()) {
    return fail("planner_to_map inverse is not finite");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

std::uint64_t MppiLocalMapContext::mapVersion() const {
  return snapshot == nullptr ? 0 : snapshot->version();
}

double MppiLocalMapContext::mapStamp() const {
  return snapshot == nullptr ? kNaN : snapshot->stamp();
}

const char* toString(MppiLocalMapHealthStatus value) {
  switch (value) {
    case MppiLocalMapHealthStatus::kOk:
      return "ok";
    case MppiLocalMapHealthStatus::kMissingBundle:
      return "missing_bundle";
    case MppiLocalMapHealthStatus::kUnhealthyMap:
      return "unhealthy_map";
    case MppiLocalMapHealthStatus::kNotControlReady:
      return "not_control_ready";
    case MppiLocalMapHealthStatus::kInvalidTimestamp:
      return "invalid_timestamp";
    case MppiLocalMapHealthStatus::kFutureTimestamp:
      return "future_timestamp";
    case MppiLocalMapHealthStatus::kStaleRosAge:
      return "stale_ros_age";
    case MppiLocalMapHealthStatus::kStaleReceiptAge:
      return "stale_receipt_age";
    case MppiLocalMapHealthStatus::kClockRewind:
      return "clock_rewind";
    case MppiLocalMapHealthStatus::kGenerationMismatch:
      return "generation_mismatch";
    case MppiLocalMapHealthStatus::kInvalidContext:
      return "invalid_context";
  }
  return "unknown";
}

MppiLocalMapHealth MppiLocalMapHealth::validate(
    const MppiLocalMapHealthInput& input, const bool check_generation,
    const std::uint64_t expected_generation) {
  MppiLocalMapHealth health;
  const auto reject = [&health](MppiLocalMapHealthStatus status,
                                std::string detail) {
    health.ok = false;
    health.status = status;
    health.detail = std::move(detail);
    return health;
  };

  if (!input.handle_present) {
    return reject(MppiLocalMapHealthStatus::kMissingBundle,
                  "local map adapter has no bundle");
  }
  std::string context_detail;
  if (!input.context.valid(&context_detail)) {
    return reject(MppiLocalMapHealthStatus::kInvalidContext,
                  "invalid local map context: " + context_detail);
  }
  if (!input.map_healthy) {
    return reject(MppiLocalMapHealthStatus::kUnhealthyMap,
                  "local map adapter reports an unhealthy map");
  }
  if (!input.control_ready) {
    return reject(MppiLocalMapHealthStatus::kNotControlReady,
                  "local map is not control ready (geometry or terrain)");
  }
  if (!finiteScalar(input.measurement_stamp_sec) ||
      !finiteScalar(input.now_ros_sec) ||
      !finiteScalar(input.receive_steady_sec) ||
      !finiteScalar(input.now_steady_sec) || input.measurement_stamp_sec == 0.0 ||
      !finiteScalar(input.max_age_sec) || input.max_age_sec <= 0.0 ||
      !finiteScalar(input.future_tolerance_sec) ||
      input.future_tolerance_sec < 0.0 ||
      !finiteScalar(input.clock_rewind_tolerance_sec) ||
      input.clock_rewind_tolerance_sec < 0.0) {
    return reject(MppiLocalMapHealthStatus::kInvalidTimestamp,
                  "local map timestamps or age limits are not finite");
  }
  if (input.measurement_stamp_sec >
      input.now_ros_sec + input.future_tolerance_sec) {
    std::ostringstream stream;
    stream << "local map acquisition stamp is in the future by "
           << (input.measurement_stamp_sec - input.now_ros_sec) << " s";
    return reject(MppiLocalMapHealthStatus::kFutureTimestamp, stream.str());
  }
  const double ros_age = input.now_ros_sec - input.measurement_stamp_sec;
  if (ros_age > input.max_age_sec) {
    std::ostringstream stream;
    stream << "local map ROS age " << ros_age << " s exceeds "
           << input.max_age_sec << " s";
    return reject(MppiLocalMapHealthStatus::kStaleRosAge, stream.str());
  }
  const double receipt_age =
      input.now_steady_sec - input.receive_steady_sec;
  if (receipt_age < -input.clock_rewind_tolerance_sec) {
    std::ostringstream stream;
    stream << "steady receipt time moved backwards by " << -receipt_age
           << " s";
    return reject(MppiLocalMapHealthStatus::kClockRewind, stream.str());
  }
  if (receipt_age > input.max_age_sec) {
    std::ostringstream stream;
    stream << "local map steady receipt age " << receipt_age << " s exceeds "
           << input.max_age_sec << " s";
    return reject(MppiLocalMapHealthStatus::kStaleReceiptAge, stream.str());
  }
  if (check_generation && input.generation != expected_generation) {
    std::ostringstream stream;
    stream << "local map generation changed from " << expected_generation
           << " to " << input.generation;
    return reject(MppiLocalMapHealthStatus::kGenerationMismatch, stream.str());
  }

  health.ok = true;
  health.status = MppiLocalMapHealthStatus::kOk;
  health.detail.clear();
  return health;
}

MppiLocalMapValidation validateMppiLocalMapRollout(
    const MppiLocalMapContext& context,
    const MppiLocalMapRolloutResult& result,
    const std::size_t expected_control_count) {
  MppiLocalMapValidation validation;
  const auto reject = [&validation](std::string detail) {
    validation.ok = false;
    validation.detail = std::move(detail);
    return validation;
  };

  std::string context_detail;
  if (!context.valid(&context_detail)) {
    return reject("invalid pinned local map context: " + context_detail);
  }
  if (!result.valid || result.status != MppiLocalMapRolloutStatus::kOk) {
    return reject("final rollout is not valid: " +
                  std::string(toString(result.status)) +
                  (result.detail.empty() ? "" : ": " + result.detail));
  }
  if (result.terrain_status != dddmr_local_map::TerrainStatus::kOk) {
    return reject("final rollout terrain status is not ok");
  }
  if (expected_control_count == 0) {
    return reject("final rollout has no expected controls");
  }
  if (result.evaluated_steps != expected_control_count ||
      result.projected_poses.size() != expected_control_count ||
      result.trajectory.controls.size() != expected_control_count ||
      result.trajectory.points.size() != expected_control_count) {
    std::ostringstream stream;
    stream << "final rollout is incomplete: evaluated="
           << result.evaluated_steps << ", poses="
           << result.projected_poses.size() << ", controls="
           << result.trajectory.controls.size() << ", points="
           << result.trajectory.points.size() << ", expected="
           << expected_control_count;
    return reject(stream.str());
  }
  if (result.map_version != context.mapVersion() ||
      result.map_stamp != context.mapStamp()) {
    return reject("final rollout map metadata differs from the pinned context");
  }
  if (!finitePose(result.projected_start_pose)) {
    return reject("final projected start pose is not finite");
  }

  for (std::size_t index = 0; index < result.projected_poses.size(); ++index) {
    const Eigen::Isometry3d& pose = result.projected_poses[index];
    const MppiTrajectoryPoint& point = result.trajectory.points[index];
    const MppiControl& control = result.trajectory.controls[index];
    if (!finitePose(pose)) {
      std::ostringstream stream;
      stream << "final projected pose " << index << " is not finite";
      return reject(stream.str());
    }
    const Eigen::Matrix3d rotation = pose.linear();
    if (!(rotation.transpose() * rotation)
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
        std::abs(rotation.determinant() - 1.0) > 1e-6) {
      std::ostringstream stream;
      stream << "final projected pose " << index
             << " is not a proper rotation";
      return reject(stream.str());
    }
    if (!finiteScalar(point.x) || !finiteScalar(point.y) ||
        !finiteScalar(point.yaw) || !finiteScalar(control.vx) ||
        !finiteScalar(control.wz)) {
      std::ostringstream stream;
      stream << "final trajectory point or control " << index
             << " is not finite";
      return reject(stream.str());
    }
    if (point.x != pose.translation().x() ||
        point.y != pose.translation().y() ||
        point.yaw != yawFromRotation(pose.linear())) {
      std::ostringstream stream;
      stream << "final trajectory point " << index
             << " does not match its projected pose";
      return reject(stream.str());
    }
    if (!point.collision_evaluated || point.in_collision) {
      std::ostringstream stream;
      stream << "final trajectory point " << index
             << " is not an evaluated collision-free pose";
      return reject(stream.str());
    }
    if (std::isnan(point.obstacle_clearance) ||
        point.obstacle_clearance < 0.0) {
      std::ostringstream stream;
      stream << "final trajectory point " << index
             << " has an invalid obstacle clearance";
      return reject(stream.str());
    }
  }

  validation.ok = true;
  validation.detail.clear();
  return validation;
}

MppiLocalMapRolloutResult rolloutMppiLocalMap(
    const MppiLocalMapContext& context,
    const MppiLocalMapRolloutInput& input) {
  if (context.snapshot == nullptr || context.terrain == nullptr) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kMissingMap,
                          "snapshot and terrain bundle is required");
  }
  if (!finiteScalar(context.snapshot->stamp())) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kInvalidContext,
                          "snapshot stamp is not finite");
  }
  if (!finiteScalar(context.query_stamp)) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kInvalidContext,
                          "query_stamp is not finite");
  }
  if (!rigidPose(context.planner_to_map)) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kInvalidTransform,
                          "planner_to_map is not a proper rigid transform");
  }
  const Eigen::Isometry3d map_to_planner = context.planner_to_map.inverse();
  if (!finitePose(map_to_planner)) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kInvalidTransform,
                          "planner_to_map inverse is not finite");
  }
  if (!finiteScalar(input.dt) || input.dt <= 0.0) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kInvalidConfig,
                          "dt must be finite and positive");
  }
  if (input.controls.empty()) {
    return rejectedResult(context, MppiLocalMapRolloutStatus::kEmptyControls,
                          "controls must not be empty");
  }
  if (!rigidPose(input.initial_pose_planner)) {
    return rejectedResult(context,
                          MppiLocalMapRolloutStatus::kInvalidInitialPose,
                          "initial planner pose is not a proper rigid transform");
  }
  for (std::size_t index = 0; index < input.controls.size(); ++index) {
    const MppiControl& control = input.controls[index];
    if (!finiteScalar(control.vx) || !finiteScalar(control.wz)) {
      std::ostringstream stream;
      stream << "control " << index << " is not finite";
      return rejectedResult(context,
                            MppiLocalMapRolloutStatus::kInvalidControl,
                            stream.str());
    }
  }

  const Eigen::Isometry3d initial_map =
      context.planner_to_map * input.initial_pose_planner;
  if (!finitePose(initial_map)) {
    return rejectedResult(context,
                          MppiLocalMapRolloutStatus::kInvalidInitialPose,
                          "initial pose in map frame is not finite");
  }

  // Evaluate the unprojected initial pose before consuming any control. This is
  // the fail-closed check that prevents a hazardous start from being accepted
  // just because the first commands move away from it.
  const Eigen::Matrix3d initial_rotation = initial_map.linear();
  dddmr_local_map::TerrainResult initial_terrain =
      context.terrain->evaluate(
          makeTerrainQuery(context, initial_map.translation(),
                           initial_rotation));
  if (!initial_terrain.ok()) {
    MppiLocalMapRolloutResult result = rejectedResult(
        context,
        statusForTerrainFailure(initial_terrain.status, true),
        terrainFailureDetail(initial_terrain, 0, true));
    result.terrain_status = initial_terrain.status;
    return result;
  }

  Eigen::Isometry3d projected_map_pose = Eigen::Isometry3d::Identity();
  if (!poseFromTerrainResult(initial_terrain, &projected_map_pose)) {
    return rejectedResult(context,
                          MppiLocalMapRolloutStatus::kNonFiniteResult,
                          "initial terrain projection is not finite");
  }
  const Eigen::Isometry3d projected_start_planner =
      map_to_planner * projected_map_pose;
  if (!finitePose(projected_start_planner)) {
    return rejectedResult(context,
                          MppiLocalMapRolloutStatus::kNonFiniteResult,
                          "projected start pose in planner frame is not finite");
  }

  MppiLocalMapRolloutResult result;
  result.valid = false;
  result.status = MppiLocalMapRolloutStatus::kOk;
  result.terrain_status = dddmr_local_map::TerrainStatus::kOk;
  result.map_version = context.snapshot->version();
  result.map_stamp = context.snapshot->stamp();
  result.projected_start_pose = projected_start_planner;
  result.projected_poses.reserve(input.controls.size());
  result.trajectory.controls.reserve(input.controls.size());
  result.trajectory.points.reserve(input.controls.size());

  double previous_roll = 0.0;
  double previous_pitch = 0.0;
  double previous_yaw = 0.0;
  rpyFromRotation(projected_map_pose.linear(), &previous_roll,
                  &previous_pitch, &previous_yaw);

  for (std::size_t step = 0; step < input.controls.size(); ++step) {
    const MppiControl& control = input.controls[step];

    // vx is body-forward speed. Advance fully in 3-D using the previous
    // projected orientation with the midpoint yaw; wz is the yaw rate.
    const double midpoint_yaw = previous_yaw + 0.5 * control.wz * input.dt;
    const Eigen::Matrix3d midpoint_rotation = rotationFromRpy(
        previous_roll, previous_pitch, midpoint_yaw);
    const Eigen::Vector3d predicted_map_position =
        projected_map_pose.translation() +
        control.vx * input.dt * midpoint_rotation.col(0);
    if (!predicted_map_position.allFinite()) {
      std::ostringstream stream;
      stream << "predicted position at step " << step << " is not finite";
      return rejectedResult(context,
                            MppiLocalMapRolloutStatus::kNonFiniteResult,
                            stream.str());
    }

    const double predicted_yaw = previous_yaw + control.wz * input.dt;
    const Eigen::Matrix3d predicted_rotation =
        rotationFromRpy(previous_roll, previous_pitch, predicted_yaw);

    // The terrain model is always queried in the map frame, using the cached
    // bundle. The returned full 3-D pose is transformed back immediately.
    dddmr_local_map::TerrainResult terrain = context.terrain->evaluate(
        makeTerrainQuery(context, predicted_map_position,
                         predicted_rotation));
    if (!terrain.ok()) {
      MppiLocalMapRolloutResult rejected = rejectedResult(
          context,
          statusForTerrainFailure(terrain.status, false),
          terrainFailureDetail(terrain, step, false));
      rejected.terrain_status = terrain.status;
      rejected.map_version = result.map_version;
      rejected.map_stamp = result.map_stamp;
      rejected.projected_start_pose = result.projected_start_pose;
      rejected.trajectory = result.trajectory;
      rejected.projected_poses = result.projected_poses;
      rejected.evaluated_steps = result.evaluated_steps;
      return rejected;
    }

    Eigen::Isometry3d next_map_pose = Eigen::Isometry3d::Identity();
    if (!poseFromTerrainResult(terrain, &next_map_pose)) {
      std::ostringstream stream;
      stream << "terrain projection at step " << step << " is not finite";
      return rejectedResult(context,
                            MppiLocalMapRolloutStatus::kNonFiniteResult,
                            stream.str());
    }
    const Eigen::Isometry3d next_planner_pose =
        map_to_planner * next_map_pose;
    if (!finitePose(next_planner_pose)) {
      std::ostringstream stream;
      stream << "projected pose at step " << step
             << " is not finite in planner frame";
      return rejectedResult(context,
                            MppiLocalMapRolloutStatus::kNonFiniteResult,
                            stream.str());
    }

    MppiTrajectoryPoint point;
    point.x = next_planner_pose.translation().x();
    point.y = next_planner_pose.translation().y();
    point.yaw = yawFromRotation(next_planner_pose.linear());
    point.in_collision = false;
    // Terrain support/body clearance is not horizontal obstacle clearance.
    // TerrainResult::obstacle_clearance is the separate raw-occupancy 3-D gap;
    // +inf means no non-support obstacle was in the bounded search volume.
    point.obstacle_clearance = terrain.obstacle_clearance;
    if (std::isnan(point.obstacle_clearance) ||
        point.obstacle_clearance < 0.0) {
      point.obstacle_clearance = kInfiniteClearance;
    }
    point.collision_evaluated = true;

    result.projected_poses.push_back(next_planner_pose);
    result.trajectory.controls.push_back(control);
    result.trajectory.points.push_back(point);
    result.evaluated_steps = step + 1;

    projected_map_pose = next_map_pose;
    rpyFromRotation(projected_map_pose.linear(), &previous_roll,
                    &previous_pitch, &previous_yaw);
  }

  result.valid = true;
  result.status = MppiLocalMapRolloutStatus::kOk;
  result.terrain_status = dddmr_local_map::TerrainStatus::kOk;
  result.detail.clear();
  return result;
}

}  // namespace local_planner
