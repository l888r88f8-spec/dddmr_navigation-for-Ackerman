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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__ACKERMANN_KINEMATICS_H_
#define DDDMR_GRID_GLOBAL_PLANNER__ACKERMANN_KINEMATICS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"
#include "dddmr_grid_global_planner/wheel_support.h"

namespace dddmr_grid_global_planner
{

// Longest signed arc accepted by the shared integrator. Longer requests are
// rejected instead of clamped; no planner primitive is expected to approach it.
inline constexpr double kMaxAckermannArcLengthM = 1.0e5;

// Largest absolute curvature accepted by the shared integrator. It keeps the
// propagated heading and trigonometric terms finite for malformed input.
inline constexpr double kMaxAckermannCurvaturePerM = 1.0e6;

// Largest absolute heading change (curvature times arc length) accepted by the
// shared integrator. This is another explicit absurd-input guard.
inline constexpr double kMaxAckermannTurnAngleRad = 1.0e6;

// A planar pose used by the exact rear-axle bicycle model. The heading is the
// same for base_link and the rear axle because this model has no side slip.
// z is carried through the planar transform and is not integrated.
struct AckermannPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};

  // Returns true only when all four components are finite.
  bool IsFinite() const noexcept;
};

// Ackermann geometry derived from the configured wheel contact offsets. The
// rear axle midpoint is the midpoint of the rear-left/rear-right contacts and
// the wheelbase is the longitudinal separation of the front/rear midpoints.
// Front and rear tracks are kept independently so arbitrary measured layouts
// are supported.
struct AckermannGeometry
{
  bool valid{false};
  std::string detail;
  double wheelbase_m{0.0};
  double front_track_m{0.0};
  double rear_track_m{0.0};
  double max_track_m{0.0};
  // base_link -> front axle midpoint in base coordinates.
  Point3 front_axle_offset{};
  // base_link -> rear axle midpoint in base coordinates.
  Point3 rear_axle_offset{};
};

// Builds the geometry from measured wheel_contact_offsets. vehicle.wheelbase_m
// is used only as a cross-check when it is finite; a missing or inconsistent
// measured offset is reported instead of being replaced by an assumed value.
AckermannGeometry MakeAckermannGeometry(
  const VehicleCapabilityConfig & vehicle);

// Converts a base_link pose to the rear-axle pose. The planar offset is rotated
// by the base yaw and the heading is unchanged.
AckermannPose BaseToRearAxle(
  const AckermannGeometry & geometry,
  const AckermannPose & base_pose) noexcept;

// Converts a rear-axle pose back to the base_link pose. This is the exact
// inverse of BaseToRearAxle for a valid geometry.
AckermannPose RearAxleToBase(
  const AckermannGeometry & geometry,
  const AckermannPose & rear_axle_pose) noexcept;

// Integrates the rear-axle bicycle equations exactly over a signed arc length
// with curvature tan(steering)/wheelbase. Reverse motion is represented by a
// negative signed arc length. Returns false and fills detail for invalid
// geometry, non-finite input, absurd input, or a non-finite result.
bool IntegrateRearAxleArc(
  const AckermannGeometry & geometry,
  const AckermannPose & start_rear_axle_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  AckermannPose * end_rear_axle_pose,
  std::string * detail = nullptr);

// Exact endpoint in base_link coordinates. It converts the start pose to the
// rear axle, integrates the bicycle model, and converts the endpoint back.
bool IntegrateBaseArc(
  const AckermannGeometry & geometry,
  const AckermannPose & start_base_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  AckermannPose * end_base_pose,
  std::string * detail = nullptr);

// Result state for the single shared arc sampler.
enum class AckermannSampleStatus : std::uint8_t
{
  kOk = 0,
  kInvalidGeometry,
  kInvalidInput,
  kSampleBudgetExceeded,
};

// Returns a stable diagnostic spelling for an arc sample status.
const char * AckermannSampleStatusToString(
  AckermannSampleStatus status) noexcept;

// Spacing policy shared by search, reconstruction and path validation.
struct AckermannSamplingOptions
{
  // Maximum swept distance allowed for the fastest wheel/body point per sample.
  double max_wheel_sample_spacing_m{0.05};
  // Maximum heading change per sample.
  double max_yaw_sample_step_rad{0.10};
  // Map resolution used to cap the sample spacing.
  double map_resolution_m{0.1};
  // Fraction of one fine cell that a sample is allowed to advance. A value
  // above one would allow a sample to skip a cell and is rejected.
  double resolution_spacing_fraction{0.5};
};

// Intermediate poses for one exact arc segment. base_poses/rear_axle_poses
// exclude the start pose and include the exact endpoint, so sample_count equals
// the number of evaluated intermediate poses.
struct AckermannArcSamples
{
  AckermannSampleStatus status{AckermannSampleStatus::kInvalidGeometry};
  std::string detail;
  AckermannPose start_base_pose{};
  AckermannPose end_base_pose{};
  AckermannPose end_rear_axle_pose{};
  double steering_central_rad{0.0};
  double signed_arc_length_m{0.0};
  double step_arc_length_m{0.0};
  double max_sample_spacing_m{0.0};
  std::size_t sample_count{0};
  std::vector<AckermannPose> base_poses;
  std::vector<AckermannPose> rear_axle_poses;

  // Returns true only for a successfully generated sample set.
  bool ok() const noexcept {return status == AckermannSampleStatus::kOk;}
};

// Single sampling entry point used by primitive simulation, reconstruction,
// terminal goal connection and path validation. It derives adaptive spacing
// from the fastest swept wheel/body point, the yaw-change limit and the fine
// map resolution. It returns an explicit budget status when the exact sample
// count exceeds max_samples.
AckermannArcSamples SampleAckermannArc(
  const AckermannGeometry & geometry,
  const VehicleCapabilityConfig & vehicle,
  const AckermannPose & start_base_pose,
  double steering_central_rad,
  double signed_arc_length_m,
  const AckermannSamplingOptions & options,
  std::size_t max_samples);

// Returns ceil(distance/spacing) clamped to max_count. Invalid input or an
// exact count above max_count saturates at max_count. The computation never
// converts a value to size_t until it is proven to fit.
std::size_t SaturatingArcSampleCount(
  double distance_m, double spacing_m,
  std::size_t max_count) noexcept;

// Computes ceil(distance/spacing) only when the exact count fits max_count.
// Returns false for non-finite input, non-positive spacing, or count overflow.
bool TryArcSampleCount(
  double distance_m, double spacing_m,
  std::size_t max_count,
  std::size_t * count) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__ACKERMANN_KINEMATICS_H_
