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

#include "dddmr_grid_global_planner/ackermann_kinematics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1.0e-12;

// Normalizes an angle into [-pi, pi). Non-finite input is treated as invalid by
// the callers; this function returns zero only to keep the helper total.
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

// Returns the maximum radial-distance ratio of any wheel or body corner to the
// rear-axle turn centre. The front outer wheel sweeps the longest arc, and a
// configured body overhang can sweep even farther, so both are included.
bool OuterSweepFactor(
  const AckermannGeometry & geometry,
  const VehicleCapabilityConfig & vehicle,
  double steering_central_rad, double * factor,
  std::string * detail)
{
  if (factor == nullptr) {
    return false;
  }
  *factor = 1.0;
  const double curvature =
    std::tan(steering_central_rad) / geometry.wheelbase_m;
  if (!std::isfinite(curvature)) {
    if (detail != nullptr) {
      *detail = "steering produces a non-finite curvature";
    }
    return false;
  }
  const double absolute_curvature = std::abs(curvature);
  if (absolute_curvature <= kEpsilon) {
    return true;
  }
  const double turn_radius = 1.0 / absolute_curvature;
  if (!std::isfinite(turn_radius) || turn_radius <= 0.0) {
    if (detail != nullptr) {
      *detail = "steering produces a non-finite turn radius";
    }
    return false;
  }
  // The turn centre is fixed relative to the rear axle. In base coordinates it
  // is the rear-axle midpoint plus the lateral turn radius: +y for a left turn
  // and -y for a right turn. A base-frame point (x, y) is at radial distance
  // hypot(x - center_x, y - center_y) from that centre.
  const double center_x = geometry.rear_axle_offset.x;
  const double center_y =
    geometry.rear_axle_offset.y +
    (curvature > 0.0 ? turn_radius : -turn_radius);

  double tire_extent = 0.0;
  if (std::isfinite(vehicle.contact_patch_length_m) &&
    vehicle.contact_patch_length_m > 0.0 &&
    std::isfinite(vehicle.tire_width_m) && vehicle.tire_width_m > 0.0)
  {
    tire_extent = 0.5 * std::hypot(
      vehicle.contact_patch_length_m,
      vehicle.tire_width_m);
  } else if (std::isfinite(vehicle.tire_width_m) &&
    vehicle.tire_width_m > 0.0)
  {
    tire_extent = 0.5 * vehicle.tire_width_m;
  }

  double maximum_factor = 1.0;
  for (const Point3 & offset : vehicle.wheel_contact_offsets) {
    if (!offset.IsFinite()) {
      if (detail != nullptr) {
        *detail = "wheel contact offset is non-finite";
      }
      return false;
    }
    const double radial =
      std::hypot(offset.x - center_x, offset.y - center_y);
    maximum_factor =
      std::max(maximum_factor, (radial + tire_extent) / turn_radius);
  }

  const bool body_configured =
    vehicle.body_min_configured && vehicle.body_max_configured &&
    vehicle.body_min.IsFinite() && vehicle.body_max.IsFinite() &&
    vehicle.body_min.x < vehicle.body_max.x &&
    vehicle.body_min.y < vehicle.body_max.y;
  if (body_configured) {
    for (const double x : {vehicle.body_min.x, vehicle.body_max.x}) {
      for (const double y : {vehicle.body_min.y, vehicle.body_max.y}) {
        const double radial = std::hypot(x - center_x, y - center_y);
        maximum_factor =
          std::max(maximum_factor, radial / turn_radius);
      }
    }
  }

  if (!std::isfinite(maximum_factor) || maximum_factor < 1.0) {
    if (detail != nullptr) {
      *detail = "wheel/body sweep factor is not finite";
    }
    return false;
  }
  *factor = maximum_factor;
  return true;
}

}  // namespace

const char * AckermannSampleStatusToString(
  AckermannSampleStatus status) noexcept
{
  switch (status) {
    case AckermannSampleStatus::kOk:
      return "ok";
    case AckermannSampleStatus::kInvalidGeometry:
      return "invalid_geometry";
    case AckermannSampleStatus::kInvalidInput:
      return "invalid_input";
    case AckermannSampleStatus::kSampleBudgetExceeded:
      return "sample_budget_exceeded";
  }
  return "unknown";
}

bool AckermannPose::IsFinite() const noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::isfinite(yaw);
}

AckermannGeometry MakeAckermannGeometry(
  const VehicleCapabilityConfig & vehicle)
{
  AckermannGeometry geometry;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (!vehicle.wheel_contact_offsets[wheel].IsFinite()) {
      geometry.detail = "wheel contact offset is non-finite";
      return geometry;
    }
  }
  const Point3 & front_left = vehicle.wheel_contact_offsets[0];
  const Point3 & front_right = vehicle.wheel_contact_offsets[1];
  const Point3 & rear_left = vehicle.wheel_contact_offsets[2];
  const Point3 & rear_right = vehicle.wheel_contact_offsets[3];

  const double front_x = 0.5 * (front_left.x + front_right.x);
  const double front_y = 0.5 * (front_left.y + front_right.y);
  const double front_z = 0.5 * (front_left.z + front_right.z);
  const double rear_x = 0.5 * (rear_left.x + rear_right.x);
  const double rear_y = 0.5 * (rear_left.y + rear_right.y);
  const double rear_z = 0.5 * (rear_left.z + rear_right.z);
  if (!std::isfinite(front_x) || !std::isfinite(front_y) ||
    !std::isfinite(front_z) || !std::isfinite(rear_x) ||
    !std::isfinite(rear_y) || !std::isfinite(rear_z))
  {
    geometry.detail = "axle midpoint is non-finite";
    return geometry;
  }

  const double wheelbase = front_x - rear_x;
  if (!std::isfinite(wheelbase) || wheelbase <= 0.0) {
    geometry.detail =
      "front axle midpoint must be ahead of the rear axle midpoint";
    return geometry;
  }
  if (std::isfinite(vehicle.wheelbase_m) && vehicle.wheelbase_m > 0.0) {
    // The measured offsets are authoritative; the configured scalar is only a
    // consistency cross-check and never replaces a measured offset.
    const double tolerance = std::max(0.05, 1.0e-3 * wheelbase);
    if (std::abs(wheelbase - vehicle.wheelbase_m) > tolerance) {
      geometry.detail =
        "configured wheelbase disagrees with the measured wheel offsets";
      return geometry;
    }
  }

  const double front_track = std::abs(front_left.y - front_right.y);
  const double rear_track = std::abs(rear_left.y - rear_right.y);
  if (!std::isfinite(front_track) || front_track <= 0.0 ||
    !std::isfinite(rear_track) || rear_track <= 0.0)
  {
    geometry.detail = "front/rear track must be finite and positive";
    return geometry;
  }

  geometry.wheelbase_m = wheelbase;
  geometry.front_track_m = front_track;
  geometry.rear_track_m = rear_track;
  geometry.max_track_m = std::max(front_track, rear_track);
  geometry.front_axle_offset = Point3{front_x, front_y, front_z};
  geometry.rear_axle_offset = Point3{rear_x, rear_y, rear_z};
  geometry.valid = true;
  geometry.detail.clear();
  return geometry;
}

AckermannPose BaseToRearAxle(
  const AckermannGeometry & geometry,
  const AckermannPose & base_pose) noexcept
{
  AckermannPose rear_axle_pose;
  const double cosine = std::cos(base_pose.yaw);
  const double sine = std::sin(base_pose.yaw);
  const Point3 & offset = geometry.rear_axle_offset;
  rear_axle_pose.x =
    base_pose.x + cosine * offset.x - sine * offset.y;
  rear_axle_pose.y =
    base_pose.y + sine * offset.x + cosine * offset.y;
  // The configured contact Z is a ground-contact height, not a measured axle
  // centre height. The planar model therefore carries the plan-view Z value
  // unchanged instead of inventing an axle height.
  rear_axle_pose.z = base_pose.z;
  rear_axle_pose.yaw = base_pose.yaw;
  return rear_axle_pose;
}

AckermannPose RearAxleToBase(
  const AckermannGeometry & geometry,
  const AckermannPose & rear_axle_pose) noexcept
{
  AckermannPose base_pose;
  const double cosine = std::cos(rear_axle_pose.yaw);
  const double sine = std::sin(rear_axle_pose.yaw);
  const Point3 & offset = geometry.rear_axle_offset;
  base_pose.x =
    rear_axle_pose.x - (cosine * offset.x - sine * offset.y);
  base_pose.y =
    rear_axle_pose.y - (sine * offset.x + cosine * offset.y);
  // Inverse of the plan-view transform above: Z is carried unchanged.
  base_pose.z = rear_axle_pose.z;
  base_pose.yaw = rear_axle_pose.yaw;
  return base_pose;
}

bool IntegrateRearAxleArc(
  const AckermannGeometry & geometry,
  const AckermannPose & start_rear_axle_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  AckermannPose * end_rear_axle_pose,
  std::string * detail)
{
  if (end_rear_axle_pose == nullptr) {
    if (detail != nullptr) {
      *detail = "end pose pointer is null";
    }
    return false;
  }
  if (!geometry.valid || !std::isfinite(geometry.wheelbase_m) ||
    geometry.wheelbase_m <= 0.0)
  {
    if (detail != nullptr) {
      *detail = "Ackermann geometry is invalid: " + geometry.detail;
    }
    return false;
  }
  if (!start_rear_axle_pose.IsFinite() ||
    !std::isfinite(steering_central_rad) ||
    !std::isfinite(signed_arc_length_m))
  {
    if (detail != nullptr) {
      *detail = "arc start pose/steering/arc length is non-finite";
    }
    return false;
  }
  if (std::abs(signed_arc_length_m) > kMaxAckermannArcLengthM) {
    if (detail != nullptr) {
      *detail = "signed arc length exceeds the absurd-input bound";
    }
    return false;
  }
  const double curvature =
    std::tan(steering_central_rad) / geometry.wheelbase_m;
  if (!std::isfinite(curvature) ||
    std::abs(curvature) > kMaxAckermannCurvaturePerM)
  {
    if (detail != nullptr) {
      *detail = "curvature is non-finite or exceeds the absurd-input bound";
    }
    return false;
  }
  const double heading_change = curvature * signed_arc_length_m;
  if (!std::isfinite(heading_change) ||
    std::abs(heading_change) > kMaxAckermannTurnAngleRad)
  {
    if (detail != nullptr) {
      *detail = "heading change exceeds the absurd-input bound";
    }
    return false;
  }

  // The midpoint/sinc form is algebraically identical to the exact bicycle
  // integral and stays stable as curvature times arc length tends to zero.
  // For an exactly straight arc the sinc factor is one.
  const double half_angle = 0.5 * heading_change;
  double sinc = 1.0;
  if (std::abs(half_angle) >= kEpsilon) {
    sinc = std::sin(half_angle) / half_angle;
  } else {
    const double squared = half_angle * half_angle;
    sinc = 1.0 - squared / 6.0 + squared * squared / 120.0;
  }
  const double midpoint_heading =
    start_rear_axle_pose.yaw + half_angle;
  AckermannPose end_pose;
  end_pose.x = start_rear_axle_pose.x +
    signed_arc_length_m * sinc * std::cos(midpoint_heading);
  end_pose.y = start_rear_axle_pose.y +
    signed_arc_length_m * sinc * std::sin(midpoint_heading);
  end_pose.z = start_rear_axle_pose.z;
  end_pose.yaw =
    NormalizeAngle(start_rear_axle_pose.yaw + heading_change);
  if (!end_pose.IsFinite()) {
    if (detail != nullptr) {
      *detail = "integrated arc endpoint is non-finite";
    }
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  *end_rear_axle_pose = end_pose;
  return true;
}

bool IntegrateBaseArc(
  const AckermannGeometry & geometry,
  const AckermannPose & start_base_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  AckermannPose * end_base_pose,
  std::string * detail)
{
  if (end_base_pose == nullptr) {
    if (detail != nullptr) {
      *detail = "end pose pointer is null";
    }
    return false;
  }
  if (!geometry.valid) {
    if (detail != nullptr) {
      *detail = "Ackermann geometry is invalid: " + geometry.detail;
    }
    return false;
  }
  const AckermannPose start_rear_axle_pose =
    BaseToRearAxle(geometry, start_base_pose);
  AckermannPose end_rear_axle_pose;
  if (!IntegrateRearAxleArc(
      geometry, start_rear_axle_pose,
      steering_central_rad, signed_arc_length_m,
      &end_rear_axle_pose, detail))
  {
    return false;
  }
  const AckermannPose end_pose =
    RearAxleToBase(geometry, end_rear_axle_pose);
  if (!end_pose.IsFinite()) {
    if (detail != nullptr) {
      *detail = "base-frame arc endpoint is non-finite";
    }
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  *end_base_pose = end_pose;
  return true;
}

std::size_t SaturatingArcSampleCount(
  double distance_m, double spacing_m,
  std::size_t max_count) noexcept
{
  std::size_t count = 0;
  if (!TryArcSampleCount(distance_m, spacing_m, max_count, &count)) {
    return max_count;
  }
  return count;
}

bool TryArcSampleCount(
  double distance_m, double spacing_m,
  std::size_t max_count,
  std::size_t * count) noexcept
{
  if (count == nullptr) {
    return false;
  }
  *count = 0;
  if (!std::isfinite(distance_m) || !std::isfinite(spacing_m) ||
    distance_m < 0.0 || spacing_m <= 0.0)
  {
    return false;
  }
  if (distance_m <= 0.0) {
    return true;
  }
  if (max_count == 0) {
    return false;
  }
  const double raw_count = std::ceil(distance_m / spacing_m);
  if (!std::isfinite(raw_count) || raw_count < 1.0) {
    return false;
  }
  // Compare in double before converting so a huge ratio cannot overflow size_t.
  if (raw_count > static_cast<double>(max_count)) {
    return false;
  }
  *count = static_cast<std::size_t>(raw_count);
  return true;
}

AckermannArcSamples SampleAckermannArc(
  const AckermannGeometry & geometry,
  const VehicleCapabilityConfig & vehicle,
  const AckermannPose & start_base_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  const AckermannSamplingOptions & options,
  std::size_t max_samples)
{
  AckermannArcSamples result;
  result.start_base_pose = start_base_pose;
  result.steering_central_rad = steering_central_rad;
  result.signed_arc_length_m = signed_arc_length_m;
  if (!geometry.valid || !std::isfinite(geometry.wheelbase_m) ||
    geometry.wheelbase_m <= 0.0)
  {
    result.status = AckermannSampleStatus::kInvalidGeometry;
    result.detail = "Ackermann geometry is invalid: " + geometry.detail;
    return result;
  }
  if (!start_base_pose.IsFinite() ||
    !std::isfinite(steering_central_rad) ||
    !std::isfinite(signed_arc_length_m))
  {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "start pose/steering/arc length is non-finite";
    return result;
  }
  if (std::abs(signed_arc_length_m) > kMaxAckermannArcLengthM) {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "signed arc length exceeds the absurd-input bound";
    return result;
  }
  if (!std::isfinite(vehicle.max_steer_central_rad) ||
    vehicle.max_steer_central_rad <= 0.0 ||
    std::abs(steering_central_rad) >
    vehicle.max_steer_central_rad + 1.0e-9)
  {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "steering angle exceeds the configured central limit";
    return result;
  }
  if (!std::isfinite(options.max_wheel_sample_spacing_m) ||
    options.max_wheel_sample_spacing_m <= 0.0 ||
    !std::isfinite(options.max_yaw_sample_step_rad) ||
    options.max_yaw_sample_step_rad <= 0.0 ||
    !std::isfinite(options.map_resolution_m) ||
    options.map_resolution_m <= 0.0 ||
    !std::isfinite(options.resolution_spacing_fraction) ||
    options.resolution_spacing_fraction <= 0.0 ||
    options.resolution_spacing_fraction > 1.0)
  {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "Ackermann sampling options are invalid";
    return result;
  }

  AckermannPose end_base_pose;
  if (!IntegrateBaseArc(
      geometry, start_base_pose, steering_central_rad,
      signed_arc_length_m, &end_base_pose,
      &result.detail))
  {
    result.status = AckermannSampleStatus::kInvalidInput;
    return result;
  }
  result.end_base_pose = end_base_pose;
  result.end_rear_axle_pose =
    BaseToRearAxle(geometry, end_base_pose);
  if (!result.end_rear_axle_pose.IsFinite()) {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "rear-axle endpoint is non-finite";
    return result;
  }
  if (std::abs(signed_arc_length_m) <= kEpsilon) {
    result.status = AckermannSampleStatus::kOk;
    result.detail.clear();
    return result;
  }

  double sweep_factor = 1.0;
  if (!OuterSweepFactor(
      geometry, vehicle, steering_central_rad,
      &sweep_factor, &result.detail))
  {
    result.status = AckermannSampleStatus::kInvalidInput;
    return result;
  }
  const double curvature =
    std::tan(steering_central_rad) / geometry.wheelbase_m;
  const double absolute_curvature = std::abs(curvature);
  double max_step_m =
    options.max_wheel_sample_spacing_m / sweep_factor;
  if (absolute_curvature > kEpsilon) {
    max_step_m = std::min(
      max_step_m, options.max_yaw_sample_step_rad / absolute_curvature);
  }
  // A sample may never jump more than a fraction of one fine cell, regardless
  // of how large the configured wheel spacing is. This keeps support evidence
  // from being manufactured by skipping cells.
  max_step_m = std::min(
    max_step_m,
    options.map_resolution_m * options.resolution_spacing_fraction);
  if (!std::isfinite(max_step_m) || max_step_m <= 0.0) {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "adaptive sample spacing is not finite/positive";
    return result;
  }

  std::size_t sample_count = 0;
  if (!TryArcSampleCount(
      std::abs(signed_arc_length_m), max_step_m,
      max_samples, &sample_count))
  {
    result.status = AckermannSampleStatus::kSampleBudgetExceeded;
    result.detail = "arc requires more samples than the remaining budget";
    return result;
  }
  result.sample_count = sample_count;
  result.max_sample_spacing_m = max_step_m;
  result.step_arc_length_m =
    signed_arc_length_m / static_cast<double>(sample_count);
  result.base_poses.reserve(sample_count);
  result.rear_axle_poses.reserve(sample_count);

  const AckermannPose start_rear_axle_pose =
    BaseToRearAxle(geometry, start_base_pose);
  if (!start_rear_axle_pose.IsFinite()) {
    result.status = AckermannSampleStatus::kInvalidInput;
    result.detail = "rear-axle start pose is non-finite";
    return result;
  }
  for (std::size_t sample = 1; sample <= sample_count; ++sample) {
    const double signed_sample_arc =
      sample == sample_count ?
      signed_arc_length_m :
      signed_arc_length_m * static_cast<double>(sample) /
      static_cast<double>(sample_count);
    AckermannPose sample_rear_axle_pose;
    if (!IntegrateRearAxleArc(
        geometry, start_rear_axle_pose,
        steering_central_rad, signed_sample_arc,
        &sample_rear_axle_pose, &result.detail))
    {
      result.status = AckermannSampleStatus::kInvalidInput;
      result.base_poses.clear();
      result.rear_axle_poses.clear();
      result.sample_count = 0;
      return result;
    }
    const AckermannPose sample_base_pose =
      RearAxleToBase(geometry, sample_rear_axle_pose);
    if (!sample_base_pose.IsFinite() ||
      !sample_rear_axle_pose.IsFinite())
    {
      result.status = AckermannSampleStatus::kInvalidInput;
      result.detail = "sampled arc pose is non-finite";
      result.base_poses.clear();
      result.rear_axle_poses.clear();
      result.sample_count = 0;
      return result;
    }
    result.base_poses.push_back(sample_base_pose);
    result.rear_axle_poses.push_back(sample_rear_axle_pose);
  }
  result.status = AckermannSampleStatus::kOk;
  result.detail.clear();
  return result;
}

}  // namespace dddmr_grid_global_planner
