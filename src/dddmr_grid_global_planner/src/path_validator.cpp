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

#include "dddmr_grid_global_planner/path_validator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/ackermann_kinematics.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Normalizes an angle into [-pi, pi).
double NormalizeAngle(double angle) noexcept
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  double normalized = std::fmod(angle + kPi, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized - kPi;
}

// Returns the absolute minimal angular difference between two headings.
double AngleDifference(double first, double second) noexcept
{
  return std::abs(NormalizeAngle(first - second));
}

// Maps a shared budget status onto the validation status.
PathValidationStatus ToValidationStatus(
  BudgetTracker::Status status) noexcept
{
  switch (status) {
    case BudgetTracker::Status::kOk:
      return PathValidationStatus::kValid;
    case BudgetTracker::Status::kCancelled:
      return PathValidationStatus::kCancelled;
    case BudgetTracker::Status::kExhausted:
    case BudgetTracker::Status::kInvalidBudget:
      return PathValidationStatus::kBudgetExhausted;
  }
  return PathValidationStatus::kInvalidPath;
}

// Maps a four-wheel support rejection onto the validation status.
PathValidationStatus RejectionToStatus(SupportRejection rejection) noexcept
{
  switch (rejection) {
    case SupportRejection::kNone:
      return PathValidationStatus::kValid;
    case SupportRejection::kVehicleCapabilityUnconfigured:
      return PathValidationStatus::kVehicleCapabilityUnconfigured;
    case SupportRejection::kObstacleCollision:
    case SupportRejection::kOverheadCollision:
    case SupportRejection::kChassisCollision:
      return PathValidationStatus::kCollision;
    case SupportRejection::kUnknownObstacle:
      return PathValidationStatus::kUnknownObstacle;
    case SupportRejection::kInvalidQuery:
    case SupportRejection::kSnapshotMismatch:
      return PathValidationStatus::kInvalidPath;
    case SupportRejection::kPoseSolveBudget:
      // A deterministic branch/pose solve budget is not evidence that the path
      // is blocked. Propagate it as incomplete so the action never reports a
      // proven no-route from an unsearched branch.
      return PathValidationStatus::kBudgetExhausted;
    default:
      return PathValidationStatus::kInsufficientSupport;
  }
}

// Verifies local bounded clear evidence for each selected support branch. The
// four-wheel/body evaluator has already enforced explicit covered volume and
// headroom. A truncated distance is a valid lower bound; no exact
// sub-truncation obstacle existence or global guaranteed clearance is required.
// Occupied branch state and bounded local unknown influence still reject.
PathValidationStatus CheckLocalClearEvidence(
  const ObstacleDistanceField & distance_field,
  const WheelSupportEvaluation & evaluation,
  const PathValidationOptions & options, std::string * detail)
{
  // Offline static maps have no dynamic observation layer. Honor their map
  // policy, as the offline evaluator already does; full-wheel validation keeps
  // its independent unknown rejection. Missing records and occupied nodes
  // still reject below for either model.
  const bool reject_unknown = options.reject_unknown_obstacles &&
    (evaluation.model != TraversabilityModel::kOfflineKinematicStaticMap ||
    distance_field.options().reject_unknown);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    const TerrainNodeId node_id = evaluation.branch_nodes[index];
    if (node_id == kInvalidTerrainNodeId) {
      continue;
    }
    if (distance_field.FindRecord(node_id) == nullptr) {
      if (detail != nullptr) {
        *detail = "wheel support branch has no local distance-field record";
      }
      return PathValidationStatus::kUnknownObstacle;
    }
    const ObstacleAttributes attributes =
      distance_field.MergedObstacle(node_id);
    const ObstacleState state = attributes.CombinedState();
    if (state == ObstacleState::kOccupied) {
      if (detail != nullptr) {
        *detail = "wheel support branch is occupied on the latest map";
      }
      return PathValidationStatus::kCollision;
    }
    if (state == ObstacleState::kUnknown) {
      if (reject_unknown) {
        if (detail != nullptr) {
          *detail = "wheel support branch has an unknown obstacle state";
        }
        return PathValidationStatus::kUnknownObstacle;
      }
      continue;
    }
    // Explicitly clear support. Local unknown influence inside the truncation
    // radius is the only remaining uncertainty; a distant truncated obstacle
    // does not invalidate a locally observed clear branch.
    if (reject_unknown &&
      distance_field.HasLocalUnknownInfluence(node_id))
    {
      if (detail != nullptr) {
        *detail =
          "wheel support branch has local unknown obstacle influence within "
          "the truncation radius";
      }
      return PathValidationStatus::kUnknownObstacle;
    }
  }
  return PathValidationStatus::kValid;
}

}  // namespace

const char * PathValidationStatusToString(
  PathValidationStatus status) noexcept
{
  switch (status) {
    case PathValidationStatus::kValid:
      return "valid";
    case PathValidationStatus::kInvalidPath:
      return "invalid_path";
    case PathValidationStatus::kInvalidOptions:
      return "invalid_options";
    case PathValidationStatus::kNotReady:
      return "not_ready";
    case PathValidationStatus::kStaleTerrainRevision:
      return "stale_terrain_revision";
    case PathValidationStatus::kStaleContentHash:
      return "stale_content_hash";
    case PathValidationStatus::kMissingSupportNode:
      return "missing_support_node";
    case PathValidationStatus::kChangedCorridor:
      return "changed_corridor";
    case PathValidationStatus::kInsufficientSupport:
      return "insufficient_support";
    case PathValidationStatus::kCollision:
      return "collision";
    case PathValidationStatus::kUnknownObstacle:
      return "unknown_obstacle";
    case PathValidationStatus::kBudgetExhausted:
      return "budget_exhausted";
    case PathValidationStatus::kCancelled:
      return "cancelled";
    case PathValidationStatus::kVehicleCapabilityUnconfigured:
      return "vehicle_capability_unconfigured";
  }
  return "unknown";
}

PathValidator::PathValidator(
  VehicleCapabilityConfig vehicle,
  WheelSupportOptions wheel_options,
  ObstacleDistanceOptions distance_options,
  PathValidationOptions validation_options)
: options_(std::move(validation_options)),
  distance_options_(std::move(distance_options)),
  evaluator_(std::move(vehicle), std::move(wheel_options)) {}

PathValidationResult PathValidator::Validate(
  const MapSnapshot & latest, const std::vector<PathPose> & path,
  std::uint64_t path_terrain_revision, std::uint64_t path_content_hash,
  const SearchBudget & budget) const
{
  const auto start_time = std::chrono::steady_clock::now();
  PathValidationResult result;
  result.status = PathValidationStatus::kInvalidPath;
  const auto finish = [&result, &start_time]() {
      result.elapsed_ms =
        std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_time)
        .count();
      return result;
    };
  if (!evaluator_.IsConfigured()) {
    result.status = PathValidationStatus::kVehicleCapabilityUnconfigured;
    result.detail = evaluator_.configuration_detail();
    return finish();
  }
  if (!latest.IsReady() || latest.fine_terrain() == nullptr) {
    result.status = PathValidationStatus::kNotReady;
    result.detail = "latest map snapshot is not ready";
    return finish();
  }
  if (path.empty()) {
    result.status = PathValidationStatus::kInvalidPath;
    result.detail = "path is empty";
    return finish();
  }
  if (options_.max_samples == 0 ||
    !std::isfinite(options_.max_wheel_sample_spacing_m) ||
    options_.max_wheel_sample_spacing_m <= 0.0 ||
    !std::isfinite(options_.max_yaw_sample_step_rad) ||
    options_.max_yaw_sample_step_rad <= 0.0 ||
    !std::isfinite(options_.changed_pose_tolerance_m) ||
    options_.changed_pose_tolerance_m < 0.0 ||
    !std::isfinite(options_.segment_endpoint_tolerance_m) ||
    options_.segment_endpoint_tolerance_m < 0.0 ||
    !std::isfinite(options_.segment_yaw_tolerance_rad) ||
    options_.segment_yaw_tolerance_rad < 0.0)
  {
    result.status = PathValidationStatus::kInvalidOptions;
    result.detail = "path validation options are invalid";
    return finish();
  }
  if (path_terrain_revision == 0 || path_content_hash == 0) {
    result.status = PathValidationStatus::kInvalidPath;
    result.detail = "path has no terrain revision/content identity";
    return finish();
  }

  // The shared budget gates every pose/query below, starting before the first
  // support evaluation. Validation options may impose a tighter sample ceiling
  // than the caller budget; the smaller ceiling always wins.
  SearchBudget effective_budget = budget;
  effective_budget.max_samples =
    std::min(budget.max_samples, options_.max_samples);
  BudgetTracker tracker(effective_budget);
  const BudgetTracker::Status initial_budget = tracker.Check();
  if (initial_budget != BudgetTracker::Status::kOk) {
    result.status = ToValidationStatus(initial_budget);
    result.detail = "validation budget expired before the first pose";
    return finish();
  }

  const TerrainSnapshot & terrain = *latest.fine_terrain();
  // Node IDs are version-scoped. Without a full reprojection pass, a changed
  // revision or content hash must be reported as stale instead of silently
  // validating IDs that now denote different surfaces.
  if (terrain.revision() != path_terrain_revision) {
    result.status = PathValidationStatus::kStaleTerrainRevision;
    result.detail =
      "latest terrain revision differs from the path revision";
    return finish();
  }
  if (terrain.content_hash() != path_content_hash) {
    result.status = PathValidationStatus::kStaleContentHash;
    result.detail = "latest terrain content hash differs from the path hash";
    return finish();
  }
  for (std::size_t index = 0; index < path.size(); ++index) {
    for (const TerrainNodeId support : path[index].support_nodes) {
      if (support != kInvalidTerrainNodeId &&
        terrain.FindNode(support) == nullptr)
      {
        result.status = PathValidationStatus::kMissingSupportNode;
        result.first_failure_index = index;
        result.detail = "path references a support node absent from the map";
        return finish();
      }
    }
  }

  // Bind the pinned field. The validator never rebuilds it from request
  // options; the field's baked options are authoritative.
  if (!latest.HasReadyDistanceField()) {
    result.status = PathValidationStatus::kNotReady;
    result.detail = "latest map snapshot has no ready pinned distance field";
    return finish();
  }
  decltype(auto) distance_field_holder = latest.distance_field();
  const ObstacleDistanceField * distance_field =
    ResolveDistanceFieldPointer(distance_field_holder);
  if (distance_field == nullptr || !distance_field->ready()) {
    result.status = PathValidationStatus::kNotReady;
    result.detail = "latest distance field is null or not ready";
    return finish();
  }
  if (distance_field->terrain().get() != latest.fine_terrain().get()) {
    result.status = PathValidationStatus::kNotReady;
    result.detail =
      "pinned distance field belongs to a different fine snapshot";
    return finish();
  }
  const AckermannGeometry geometry =
    MakeAckermannGeometry(evaluator_.vehicle());
  if (!geometry.valid) {
    result.status = PathValidationStatus::kInvalidOptions;
    result.detail = "Ackermann geometry is invalid: " + geometry.detail;
    return finish();
  }

  // Validate the first pose metadata, then re-evaluate it under the same budget.
  if (!path.front().position.IsFinite() ||
    !IsSafeWorldCoordinate(path.front().position.x) ||
    !IsSafeWorldCoordinate(path.front().position.y) ||
    !std::isfinite(path.front().heading) ||
    !std::isfinite(path.front().steering_central_rad) ||
    !std::isfinite(path.front().signed_arc_length_m) ||
    !std::isfinite(path.front().direction))
  {
    result.status = PathValidationStatus::kInvalidPath;
    result.detail = "first path pose contains a non-finite value";
    return finish();
  }
  if (std::abs(path.front().signed_arc_length_m) > 1.0e-9) {
    result.status = PathValidationStatus::kInvalidPath;
    result.detail = "first path pose must have zero signed arc length";
    return finish();
  }
  WheelSupportEvaluation previous;
  bool has_previous = false;
  const BudgetTracker::Status first_budget = tracker.ConsumeSamples(1);
  if (first_budget != BudgetTracker::Status::kOk) {
    result.status = ToValidationStatus(first_budget);
    result.detail = "validation sample budget expired at the first pose";
    return finish();
  }
  {
    WheelSupportQuery query;
    query.center = path.front().position;
    query.heading = path.front().heading;
    query.steering_central_rad = path.front().steering_central_rad;
    const WheelSupportEvaluation evaluation =
      evaluator_.Evaluate(latest, *distance_field, query);
    ++result.checked_samples;
    if (!evaluation.feasible) {
      result.status = RejectionToStatus(evaluation.rejection);
      result.first_failure_index = 0;
      result.detail = evaluation.detail;
      return finish();
    }
    std::string clearance_detail;
    const PathValidationStatus clearance_status =
      CheckLocalClearEvidence(
      *distance_field, evaluation, options_,
      &clearance_detail);
    if (clearance_status != PathValidationStatus::kValid) {
      result.status = clearance_status;
      result.first_failure_index = 0;
      result.detail = clearance_detail;
      return finish();
    }
    if (std::abs(evaluation.pose.base_origin.z - path.front().position.z) >
      options_.changed_pose_tolerance_m)
    {
      result.status = PathValidationStatus::kChangedCorridor;
      result.first_failure_index = 0;
      result.detail = "first pose height changed on the latest map";
      return finish();
    }
    PathPose updated = path.front();
    updated.position = evaluation.pose.base_origin;
    updated.roll = evaluation.roll_rad;
    updated.pitch = evaluation.pitch_rad;
    updated.support_nodes = evaluation.branch_nodes;
    updated.steering_central_rad = path.front().steering_central_rad;
    updated.signed_arc_length_m = 0.0;
    result.updated_path.push_back(updated);
    previous = evaluation;
    has_previous = true;
  }

  const AckermannSamplingOptions sampling_options = [&]() {
      AckermannSamplingOptions sampling;
      sampling.max_wheel_sample_spacing_m =
        options_.max_wheel_sample_spacing_m;
      sampling.max_yaw_sample_step_rad =
        options_.max_yaw_sample_step_rad;
      sampling.map_resolution_m = terrain.resolution();
      sampling.resolution_spacing_fraction = 0.5;
      return sampling;
    }();

  for (std::size_t segment = 0; segment + 1 < path.size(); ++segment) {
    const BudgetTracker::Status segment_budget = tracker.Check();
    if (segment_budget != BudgetTracker::Status::kOk) {
      result.status = ToValidationStatus(segment_budget);
      result.first_failure_index = segment;
      result.detail = "validation budget expired before a path segment";
      return finish();
    }
    const PathPose & from = path[segment];
    const PathPose & to = path[segment + 1];
    if (!from.position.IsFinite() || !std::isfinite(from.heading) ||
      !IsSafeWorldCoordinate(from.position.x) ||
      !IsSafeWorldCoordinate(from.position.y) ||
      !to.position.IsFinite() || !std::isfinite(to.heading) ||
      !IsSafeWorldCoordinate(to.position.x) ||
      !IsSafeWorldCoordinate(to.position.y) ||
      !std::isfinite(to.steering_central_rad) ||
      !std::isfinite(to.signed_arc_length_m) ||
      !std::isfinite(to.direction))
    {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail = "path segment contains a non-finite replay field";
      return finish();
    }
    if (std::abs(std::abs(to.direction) - 1.0) > 1.0e-6) {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail = "path segment direction must be +1 or -1";
      return finish();
    }
    if (std::abs(to.signed_arc_length_m) > 1.0e-9 &&
      (to.direction < 0.0) != (to.signed_arc_length_m < 0.0))
    {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail = "segment direction disagrees with signed arc length";
      return finish();
    }

    // Replay the recorded exact steering/signed arc from the stored pose. The
    // endpoint must match the stored next pose; otherwise this is a different
    // trajectory and is rejected instead of being validated.
    const AckermannPose from_pose{from.position.x, from.position.y,
      from.position.z, from.heading};
    AckermannPose expected_endpoint;
    std::string endpoint_detail;
    if (!IntegrateBaseArc(
        geometry, from_pose, to.steering_central_rad,
        to.signed_arc_length_m, &expected_endpoint,
        &endpoint_detail))
    {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail = "segment endpoint integration failed: " +
        endpoint_detail;
      return finish();
    }
    const double endpoint_position_error =
      std::hypot(
      expected_endpoint.x - to.position.x,
      expected_endpoint.y - to.position.y);
    if (endpoint_position_error > options_.segment_endpoint_tolerance_m) {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail =
        "recorded segment endpoint does not match the stored next pose";
      return finish();
    }
    if (AngleDifference(expected_endpoint.yaw, to.heading) >
      options_.segment_yaw_tolerance_rad)
    {
      result.status = PathValidationStatus::kInvalidPath;
      result.first_failure_index = segment;
      result.detail =
        "recorded segment yaw does not match the stored next pose";
      return finish();
    }

    AckermannArcSamples samples = SampleAckermannArc(
      geometry, evaluator_.vehicle(), from_pose,
      to.steering_central_rad, to.signed_arc_length_m, sampling_options,
      tracker.remaining_samples());
    if (!samples.ok()) {
      if (samples.status ==
        AckermannSampleStatus::kSampleBudgetExceeded)
      {
        result.status = PathValidationStatus::kBudgetExhausted;
      } else {
        result.status = PathValidationStatus::kInvalidPath;
      }
      result.first_failure_index = segment;
      result.detail = "segment replay sampling failed: " + samples.detail;
      return finish();
    }
    const BudgetTracker::Status consumed =
      tracker.ConsumeSamples(samples.sample_count);
    if (consumed != BudgetTracker::Status::kOk) {
      result.status = ToValidationStatus(consumed);
      result.first_failure_index = segment;
      result.detail = "validation sample budget expired in a segment";
      return finish();
    }

    const double signed_step = samples.step_arc_length_m;
    for (std::size_t sample = 0; sample < samples.base_poses.size();
      ++sample)
    {
      const BudgetTracker::Status sample_budget = tracker.Check();
      if (sample_budget != BudgetTracker::Status::kOk) {
        result.status = ToValidationStatus(sample_budget);
        result.first_failure_index = segment;
        result.detail = "validation budget expired during a segment";
        return finish();
      }
      const AckermannPose & sample_pose = samples.base_poses[sample];
      const double fraction =
        samples.sample_count == 0 ?
        1.0 :
        static_cast<double>(sample + 1) /
        static_cast<double>(samples.sample_count);
      const double expected_z =
        from.position.z + (to.position.z - from.position.z) * fraction;
      WheelSupportQuery query;
      query.center = Point3{sample_pose.x, sample_pose.y, expected_z};
      query.heading = sample_pose.yaw;
      query.steering_central_rad = to.steering_central_rad;
      query.has_previous = has_previous;
      query.previous = previous;
      const WheelSupportEvaluation evaluation =
        evaluator_.Evaluate(latest, *distance_field, query);
      ++result.checked_samples;
      if (!evaluation.feasible) {
        result.status = RejectionToStatus(evaluation.rejection);
        result.first_failure_index = segment;
        result.detail = evaluation.detail;
        return finish();
      }
      std::string clearance_detail;
      const PathValidationStatus clearance_status =
        CheckLocalClearEvidence(
        *distance_field, evaluation, options_,
        &clearance_detail);
      if (clearance_status != PathValidationStatus::kValid) {
        result.status = clearance_status;
        result.first_failure_index = segment;
        result.detail = clearance_detail;
        return finish();
      }
      if (std::abs(evaluation.pose.base_origin.z - expected_z) >
        options_.changed_pose_tolerance_m)
      {
        result.status = PathValidationStatus::kChangedCorridor;
        result.first_failure_index = segment;
        result.detail =
          "terrain support height changed along the validated corridor";
        return finish();
      }
      PathPose updated = to;
      updated.position = evaluation.pose.base_origin;
      updated.heading = sample_pose.yaw;
      updated.roll = evaluation.roll_rad;
      updated.pitch = evaluation.pitch_rad;
      updated.support_nodes = evaluation.branch_nodes;
      updated.direction = to.direction;
      updated.steering_central_rad = to.steering_central_rad;
      updated.signed_arc_length_m = signed_step;
      updated.cumulative_distance_m =
        result.updated_path.empty() ?
        0.0 :
        result.updated_path.back().cumulative_distance_m +
        std::abs(signed_step);
      updated.cumulative_cost =
        from.cumulative_cost +
        (to.cumulative_cost - from.cumulative_cost) * fraction;
      result.updated_path.push_back(updated);
      previous = evaluation;
      has_previous = true;
    }
  }

  result.status = PathValidationStatus::kValid;
  result.detail = "path is valid on the latest snapshot";
  return finish();
}

}  // namespace dddmr_grid_global_planner
