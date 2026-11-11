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

#include "dddmr_grid_global_planner/measurement_evidence.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "dddmr_grid_global_planner/ground_surface_reconstruction.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kBoundTolerance = 1e-9;
constexpr double kUnitNormalTolerance = 1e-6;
// Component-wise absolute bound for two finite unit normals. A dot-product
// lower bound only measures the squared angular difference and can accept a
// visible per-component error far larger than this numerical tolerance.
constexpr double kUnitNormalComponentTolerance = 1e-6;
constexpr double kNodeGeometryTolerance = 1e-6;
// Conservative floating-point operation-error budget, not a physical
// threshold. It bounds accumulation in the offset dot product; the physical
// Z-height allowance is the separate kNodeGeometryTolerance term.
constexpr double kFloatingPointErrorBudget = 64.0;
constexpr double kMachineEpsilon = std::numeric_limits<double>::epsilon();
constexpr std::uint64_t kHashOffset = 14695981039346656037ULL;
constexpr std::uint64_t kHashPrime = 1099511628211ULL;
constexpr std::uint64_t kCanonicalNaNBits = 0x7ff8000000000001ULL;

bool Fail(std::string * detail, const std::string & message)
{
  if (detail != nullptr) {
    *detail = message;
  }
  return false;
}

bool NearlyEqual(double first, double second, double tolerance = kBoundTolerance)
{
  return std::fabs(first - second) <= tolerance;
}

bool IsValidStatus(MeasurementPatchStatus status) noexcept
{
  switch (status) {
    case MeasurementPatchStatus::kObservedOnly:
    case MeasurementPatchStatus::kFitRejected:
    case MeasurementPatchStatus::kProhibited:
    case MeasurementPatchStatus::kSupported:
    case MeasurementPatchStatus::kUnconfirmed:
      return true;
  }
  return false;
}

bool IsValidReason(MeasurementRejectionReason reason) noexcept
{
  switch (reason) {
    case MeasurementRejectionReason::kNone:
    case MeasurementRejectionReason::kInsufficientPoints:
    case MeasurementRejectionReason::kSamplingGap:
    case MeasurementRejectionReason::kUnsupportedCenter:
    case MeasurementRejectionReason::kDegenerateGeometry:
    case MeasurementRejectionReason::kHighResidual:
    case MeasurementRejectionReason::kInsufficientInliers:
    case MeasurementRejectionReason::kNonFinitePlane:
    case MeasurementRejectionReason::kExcessiveSlope:
    case MeasurementRejectionReason::kLocalDepression:
    case MeasurementRejectionReason::kLocalRaisedConflict:
    case MeasurementRejectionReason::kExpansionInvalid:
    case MeasurementRejectionReason::kDiscontinuousTerrain:
    case MeasurementRejectionReason::kInsufficientCoverage:
    case MeasurementRejectionReason::kUnconfirmedComponent:
      return true;
  }
  return false;
}

bool IsObservedOnlyReason(MeasurementRejectionReason reason) noexcept
{
  switch (reason) {
    case MeasurementRejectionReason::kInsufficientPoints:
    case MeasurementRejectionReason::kSamplingGap:
    case MeasurementRejectionReason::kUnsupportedCenter:
    case MeasurementRejectionReason::kDegenerateGeometry:
    case MeasurementRejectionReason::kNonFinitePlane:
      return true;
    default:
      return false;
  }
}

bool IsFitRejectedReason(MeasurementRejectionReason reason) noexcept
{
  switch (reason) {
    case MeasurementRejectionReason::kDegenerateGeometry:
    case MeasurementRejectionReason::kHighResidual:
    case MeasurementRejectionReason::kInsufficientInliers:
    case MeasurementRejectionReason::kNonFinitePlane:
    case MeasurementRejectionReason::kLocalDepression:
    // These gates can reject a patch after a real candidate plane exists; the
    // fit-rejected status is then the only form that can retain that plane.
    case MeasurementRejectionReason::kSamplingGap:
    case MeasurementRejectionReason::kUnsupportedCenter:
    case MeasurementRejectionReason::kExcessiveSlope:
    case MeasurementRejectionReason::kDiscontinuousTerrain:
    case MeasurementRejectionReason::kUnconfirmedComponent:
      return true;
    default:
      return false;
  }
}

bool IsExpansionFailureReason(MeasurementRejectionReason reason) noexcept
{
  return IsValidReason(reason) && reason != MeasurementRejectionReason::kNone &&
         reason != MeasurementRejectionReason::kExcessiveSlope;
}

// Validates one ascending, duplicate-free index vector against the point table.
bool ValidateIndices(
  const std::vector<std::uint32_t> & indices,
  std::size_t point_count,
  const std::string & name,
  std::string * detail)
{
  for (std::size_t index = 0; index < indices.size(); ++index) {
    if (static_cast<std::size_t>(indices[index]) >= point_count) {
      return Fail(
        detail, name + " references a point index outside the evidence table");
    }
    if (index > 0U && indices[index - 1U] >= indices[index]) {
      return Fail(detail, name + " must be strictly ascending without duplicates");
    }
  }
  return true;
}

bool IsSubset(
  const std::vector<std::uint32_t> & subset,
  const std::vector<std::uint32_t> & superset) noexcept
{
  return std::includes(superset.begin(), superset.end(), subset.begin(), subset.end());
}

void ComputeZBounds(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  double * z_min,
  double * z_max)
{
  if (indices.empty()) {
    *z_min = std::numeric_limits<double>::quiet_NaN();
    *z_max = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  *z_min = points[indices.front()].z;
  *z_max = points[indices.front()].z;
  for (const std::uint32_t index : indices) {
    *z_min = std::min(*z_min, points[index].z);
    *z_max = std::max(*z_max, points[index].z);
  }
}

// Height residual used by offline_geometry: project the 3-D offset onto the
// unit normal, then divide by the normal Z component so the result is a Z
// height residual for a non-vertical plane.
bool ComputeResidual(
  const std::vector<Point3> & points,
  std::uint32_t index,
  const Point3 & centroid,
  const Vector3 & normal,
  double * residual)
{
  const Point3 & point = points[index];
  const double numerator =
    normal.x * (point.x - centroid.x) +
    normal.y * (point.y - centroid.y) +
    normal.z * (point.z - centroid.z);
  const double value = numerator / normal.z;
  if (!std::isfinite(value)) {
    return false;
  }
  *residual = value;
  return true;
}

bool ComputeResidualExtrema(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & centroid,
  const Vector3 & normal,
  double * min_signed,
  double * max_signed)
{
  if (indices.empty()) {
    return false;
  }
  bool first = true;
  double minimum = 0.0;
  double maximum = 0.0;
  for (const std::uint32_t index : indices) {
    double residual = 0.0;
    if (!ComputeResidual(points, index, centroid, normal, &residual)) {
      return false;
    }
    if (first) {
      minimum = residual;
      maximum = residual;
      first = false;
    } else {
      minimum = std::min(minimum, residual);
      maximum = std::max(maximum, residual);
    }
  }
  *min_signed = minimum;
  *max_signed = maximum;
  return true;
}

bool ComputeRms(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & centroid,
  const Vector3 & normal,
  double * rms)
{
  if (indices.empty()) {
    return false;
  }
  double squared = 0.0;
  for (const std::uint32_t index : indices) {
    double residual = 0.0;
    if (!ComputeResidual(points, index, centroid, normal, &residual)) {
      return false;
    }
    squared += residual * residual;
  }
  *rms = std::sqrt(squared / static_cast<double>(indices.size()));
  return std::isfinite(*rms);
}

bool PointEquals(const Point3 & first, const Point3 & second) noexcept
{
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

bool PointOrderLess(const Point3 & first, const Point3 & second) noexcept
{
  if (first.x != second.x) {
    return first.x < second.x;
  }
  if (first.y != second.y) {
    return first.y < second.y;
  }
  return first.z < second.z;
}

bool IsZeroPoint(const Point3 & point) noexcept
{
  return point.x == 0.0 && point.y == 0.0 && point.z == 0.0;
}

bool ValidateResidualStats(
  const MeasurementFitAttempt & attempt,
  const std::vector<Point3> & points,
  const std::string & name,
  std::string * detail)
{
  double expected_min = 0.0;
  double expected_max = 0.0;
  if (!ComputeResidualExtrema(
      points, attempt.point_indices, attempt.centroid, attempt.normal,
      &expected_min, &expected_max))
  {
    return Fail(detail, name + " could not reconstruct finite residuals");
  }
  if (!NearlyEqual(attempt.min_signed_residual, expected_min)) {
    return Fail(detail, name + " min signed residual does not match samples");
  }
  if (!NearlyEqual(attempt.max_signed_residual, expected_max)) {
    return Fail(detail, name + " max signed residual does not match samples");
  }
  const double expected_depression = std::max(0.0, -expected_min);
  const double expected_raised = std::max(0.0, expected_max);
  if (!NearlyEqual(attempt.max_depression_m, expected_depression)) {
    return Fail(detail, name + " max depression does not match samples");
  }
  if (!NearlyEqual(attempt.max_raised_m, expected_raised)) {
    return Fail(detail, name + " max raised does not match samples");
  }
  return true;
}

// Validates one fit attempt's structural/geometric fields. Thresholds such as
// min4/RMS/gap are builder policy and are deliberately not hardcoded here.
bool ValidateFitAttempt(
  const MeasurementFitAttempt & attempt,
  const std::vector<Point3> & points,
  const std::string & name,
  std::string * detail)
{
  if (!std::isfinite(attempt.radius_m) || attempt.radius_m <= 0.0) {
    return Fail(detail, name + " radius must be finite and positive");
  }
  if (attempt.point_indices.empty()) {
    return Fail(detail, name + " has no sample refs");
  }
  if (!ValidateIndices(
      attempt.point_indices, points.size(), name + ".point_indices", detail) ||
    !ValidateIndices(
      attempt.inlier_indices, points.size(), name + ".inlier_indices", detail))
  {
    return false;
  }
  if (!IsSubset(attempt.inlier_indices, attempt.point_indices)) {
    return Fail(detail, name + " inliers are not a subset of attempt samples");
  }
  if (!std::isfinite(attempt.nearest_support_xy_m) ||
    attempt.nearest_support_xy_m < 0.0)
  {
    return Fail(detail, name + " nearest support distance must be finite/non-negative");
  }
  if (attempt.support_gate_ok && !attempt.center_in_support_hull) {
    return Fail(detail, name + " support gate cannot pass outside the support hull");
  }

  if (!attempt.plane_valid) {
    if (!attempt.inlier_indices.empty() ||
      !std::isnan(attempt.rms_residual) ||
      attempt.conflict_sample_count != 0U ||
      attempt.raised_conflict_sample_count != 0U ||
      !std::isnan(attempt.min_signed_residual) ||
      !std::isnan(attempt.max_signed_residual) ||
      attempt.max_depression_m != 0.0 ||
      attempt.max_raised_m != 0.0 ||
      !IsZeroPoint(attempt.conflict_point))
    {
      return Fail(detail, name + " has plane/residual data without a valid plane");
    }
    return true;
  }

  if (!attempt.centroid.IsFinite()) {
    return Fail(detail, name + " has a non-finite centroid");
  }
  if (!attempt.normal.IsFinite()) {
    return Fail(detail, name + " has a non-finite normal");
  }
  const double norm = attempt.normal.Norm();
  if (!std::isfinite(norm) ||
    std::fabs(norm - 1.0) > kUnitNormalTolerance)
  {
    return Fail(detail, name + " normal must be finite and unit length");
  }
  if (!(attempt.normal.z > 0.0)) {
    return Fail(detail, name + " normal must have a positive Z component");
  }
  if (attempt.inlier_indices.empty()) {
    return Fail(detail, name + " valid plane has no inlier refs");
  }
  double expected_rms = 0.0;
  if (!ComputeRms(
      points, attempt.inlier_indices, attempt.centroid, attempt.normal,
      &expected_rms))
  {
    return Fail(detail, name + " could not reconstruct finite RMS");
  }
  if (!NearlyEqual(attempt.rms_residual, expected_rms)) {
    return Fail(detail, name + " RMS does not match inlier samples");
  }
  if (attempt.conflict_sample_count != attempt.point_indices.size()) {
    return Fail(detail, name + " conflict count must cover every attempt sample");
  }
  if (attempt.raised_conflict_sample_count > attempt.conflict_sample_count) {
    return Fail(detail, name + " raised count exceeds scanned samples");
  }
  if (!ValidateResidualStats(attempt, points, name, detail)) {
    return false;
  }

  bool conflict_member = false;
  bool conflict_extremum = false;
  for (const std::uint32_t index : attempt.point_indices) {
    if (!PointEquals(points[index], attempt.conflict_point)) {
      continue;
    }
    conflict_member = true;
    double residual = 0.0;
    if (ComputeResidual(
        points, index, attempt.centroid, attempt.normal, &residual) &&
      (NearlyEqual(residual, attempt.min_signed_residual) ||
      NearlyEqual(residual, attempt.max_signed_residual)))
    {
      conflict_extremum = true;
      break;
    }
  }
  if (!conflict_member) {
    return Fail(detail, name + " conflict point is not an actual attempt member");
  }
  if (!conflict_extremum) {
    return Fail(detail, name + " conflict point is not a residual extremum");
  }
  return true;
}

bool ValidateStatusReason(
  const MeasurementPatch & patch,
  std::string * detail)
{
  if (!IsValidStatus(patch.status) || !IsValidReason(patch.reason)) {
    return Fail(detail, "patch status/reason contains an invalid enum value");
  }
  switch (patch.status) {
    case MeasurementPatchStatus::kSupported:
      if (patch.reason != MeasurementRejectionReason::kNone) {
        return Fail(detail, "supported patch must have reason kNone");
      }
      return true;
    case MeasurementPatchStatus::kProhibited:
      if (patch.reason != MeasurementRejectionReason::kExcessiveSlope &&
        patch.reason != MeasurementRejectionReason::kDiscontinuousTerrain)
      {
        return Fail(
          detail,
          "prohibited patch must record kExcessiveSlope or "
          "kDiscontinuousTerrain");
      }
      return true;
    case MeasurementPatchStatus::kObservedOnly:
      if (!IsObservedOnlyReason(patch.reason) || patch.fit.plane_valid) {
        return Fail(detail, "observed-only patch has incompatible plane/reason");
      }
      return true;
    case MeasurementPatchStatus::kFitRejected:
      if (patch.reason == MeasurementRejectionReason::kLocalDepression &&
        !patch.fit.plane_valid)
      {
        return Fail(detail, "local depression requires a valid final plane");
      }
      return IsFitRejectedReason(patch.reason) ?
             true : Fail(detail, "fit-rejected patch has incompatible reason");
    case MeasurementPatchStatus::kUnconfirmed:
      if (patch.reason !=
        MeasurementRejectionReason::kUnconfirmedComponent)
      {
        return Fail(
          detail,
          "unconfirmed patch must record kUnconfirmedComponent");
      }
      return patch.fit.plane_valid ?
             true : Fail(detail, "unconfirmed patch requires valid geometry");
  }
  return Fail(detail, "patch has an invalid status");
}

// The linked node support must be exactly the measured inlier multiset, not just
// a matching count. Both sides are sorted so index order and builder point order
// cannot hide a substituted or duplicated point.
bool ValidateLinkedNodeSupport(
  const MeasurementFitAttempt & fit,
  const TerrainNode & node,
  const std::vector<Point3> & points,
  std::string * detail)
{
  const std::size_t inlier_count = fit.inlier_indices.size();
  if (node.support.points.size() != inlier_count ||
    node.support.sample_count != inlier_count)
  {
    return Fail(detail, "linked node support count does not match patch inliers");
  }
  // Reject non-finite support coordinates before copying or sorting. The exact
  // support comparison uses a floating lexicographic order; NaN is equivalent
  // to every value while distinct finite x values remain ordered, so a NaN in
  // this vector would violate std::sort's strict-weak-ordering precondition.
  // The finite input table is already validated above, so only the node support
  // vector needs this fail-fast guard.
  for (std::size_t index = 0; index < node.support.points.size(); ++index) {
    if (!node.support.points[index].IsFinite()) {
      return Fail(
        detail,
        "linked node support point " + std::to_string(index) +
        " has nonfinite coordinates");
    }
  }
  std::vector<Point3> expected;
  expected.reserve(inlier_count);
  for (const std::uint32_t index : fit.inlier_indices) {
    expected.push_back(points[index]);
  }
  std::vector<Point3> actual = node.support.points;
  std::sort(expected.begin(), expected.end(), PointOrderLess);
  std::sort(actual.begin(), actual.end(), PointOrderLess);
  if (!std::equal(actual.begin(), actual.end(), expected.begin(), PointEquals)) {
    return Fail(
      detail,
      "linked node support points are not the exact patch inliers");
  }
  double expected_z_min = 0.0;
  double expected_z_max = 0.0;
  ComputeZBounds(
    points, fit.inlier_indices, &expected_z_min, &expected_z_max);
  if (!NearlyEqual(node.support.z_min, expected_z_min) ||
    !NearlyEqual(node.support.z_max, expected_z_max))
  {
    return Fail(
      detail, "linked node support Z bounds do not match patch inliers");
  }
  return true;
}

bool ValidateUnitUpNormal(
  const Vector3 & normal, const std::string & name, std::string * detail)
{
  if (!normal.IsFinite()) {
    return Fail(detail, name + " must be finite");
  }
  const double norm = normal.Norm();
  if (!std::isfinite(norm) ||
    std::fabs(norm - 1.0) > kUnitNormalTolerance)
  {
    return Fail(detail, name + " must be finite and unit length");
  }
  if (!(normal.z > 0.0)) {
    return Fail(detail, name + " must have a positive Z component");
  }
  return true;
}

bool NearlySamePoint(
  const Point3 & first, const Point3 & second, double tolerance) noexcept
{
  return std::fabs(first.x - second.x) <= tolerance &&
         std::fabs(first.y - second.y) <= tolerance &&
         std::fabs(first.z - second.z) <= tolerance;
}

bool NearlySameNormal(
  const Vector3 & first, const Vector3 & second) noexcept
{
  return std::fabs(first.x - second.x) <= kUnitNormalComponentTolerance &&
         std::fabs(first.y - second.y) <= kUnitNormalComponentTolerance &&
         std::fabs(first.z - second.z) <= kUnitNormalComponentTolerance;
}

bool SameSurfaceEvidence(
  const SurfaceReconstructionEvidence & first,
  const SurfaceReconstructionEvidence & second) noexcept
{
  return first.attempted == second.attempted &&
         first.applied == second.applied &&
         first.status == second.status &&
         NearlySamePoint(
    first.original_centroid, second.original_centroid,
    kNodeGeometryTolerance) &&
         NearlySameNormal(
    first.original_normal, second.original_normal) &&
         NearlySamePoint(
    first.reconstructed_centroid, second.reconstructed_centroid,
    kNodeGeometryTolerance) &&
         NearlySameNormal(
    first.reconstructed_normal, second.reconstructed_normal) &&
         NearlyEqual(first.original_height_m, second.original_height_m) &&
         NearlyEqual(
    first.reconstructed_height_m, second.reconstructed_height_m) &&
         NearlyEqual(first.correction_m, second.correction_m) &&
         NearlyEqual(
    first.original_rms_residual_m, second.original_rms_residual_m) &&
         NearlyEqual(
    first.reconstruction_rms_residual_m,
    second.reconstruction_rms_residual_m) &&
         NearlyEqual(
    first.own_support_rms_residual_m,
    second.own_support_rms_residual_m) &&
         NearlyEqual(first.confidence, second.confidence) &&
         first.support_nodes == second.support_nodes &&
         first.support_points == second.support_points &&
         first.inferred == second.inferred &&
         first.shared_corners_valid == second.shared_corners_valid &&
         first.regional_support_cells == second.regional_support_cells &&
         NearlySamePoint(
    first.shared_corners[0], second.shared_corners[0],
    kNodeGeometryTolerance) &&
         NearlySamePoint(
    first.shared_corners[1], second.shared_corners[1],
    kNodeGeometryTolerance) &&
         NearlySamePoint(
    first.shared_corners[2], second.shared_corners[2],
    kNodeGeometryTolerance) &&
         NearlySamePoint(
    first.shared_corners[3], second.shared_corners[3],
    kNodeGeometryTolerance);
}

// A bounded ground-surface correction may move the planning node away from the
// original patch fit only when the complete provenance is present and every
// field is internally consistent. Raw support and patch.fit are checked
// separately and are never rewritten.
bool ValidateAppliedSurfaceReconstruction(
  const MeasurementPatch & patch,
  const TerrainNode & node,
  double origin_x,
  double origin_y,
  double resolution,
  std::string * detail)
{
  const SurfaceReconstructionEvidence & evidence =
    node.surface_reconstruction;
  if (!evidence.attempted ||
    evidence.status != SurfaceReconstructionStatus::kApplied)
  {
    return Fail(
      detail,
      "linked node claims an applied reconstruction without the applied status");
  }
  if (!SameSurfaceEvidence(evidence, patch.surface_reconstruction) ||
    !patch.surface_reconstruction.applied)
  {
    return Fail(
      detail,
      "linked patch reconstruction provenance does not mirror its node");
  }
  if (!ValidateUnitUpNormal(
      evidence.original_normal, "reconstruction original normal", detail) ||
    !ValidateUnitUpNormal(
      evidence.reconstructed_normal, "reconstruction reconstructed normal",
      detail))
  {
    return false;
  }
  if (!evidence.original_centroid.IsFinite() ||
    !evidence.reconstructed_centroid.IsFinite())
  {
    return Fail(detail, "reconstruction centroid must be finite");
  }
  if (!std::isfinite(evidence.original_height_m) ||
    !std::isfinite(evidence.reconstructed_height_m) ||
    !std::isfinite(evidence.correction_m) ||
    !std::isfinite(evidence.original_rms_residual_m) ||
    !std::isfinite(evidence.reconstruction_rms_residual_m) ||
    !std::isfinite(evidence.own_support_rms_residual_m) ||
    !std::isfinite(evidence.confidence))
  {
    return Fail(detail, "reconstruction metrics must be finite");
  }
  const Point3 & original = evidence.original_centroid;
  const Vector3 & original_normal = evidence.original_normal;
  const double original_height_at_cell = original.z -
    (original_normal.x * (node.representative.x - original.x) +
    original_normal.y * (node.representative.y - original.y)) /
    original_normal.z;
  if (!std::isfinite(original_height_at_cell) ||
    !std::isfinite(evidence.reconstructed_centroid.z) ||
    !NearlyEqual(
      evidence.original_height_m, original_height_at_cell) ||
    !NearlyEqual(
      evidence.reconstructed_height_m,
      evidence.reconstructed_centroid.z))
  {
    return Fail(
      detail,
      "reconstruction heights must match their planes at the cell centre");
  }
  if (!NearlyEqual(
      evidence.correction_m,
      evidence.reconstructed_height_m - evidence.original_height_m))
  {
    return Fail(detail, "reconstruction correction does not match centroids");
  }
  // The original half of the provenance must still describe the untouched
  // patch fit; this is what makes the estimated/raw distinction auditable.
  if (!NearlySamePoint(
      evidence.original_centroid, patch.fit.centroid,
      kNodeGeometryTolerance) ||
    !NearlySameNormal(evidence.original_normal, patch.fit.normal) ||
    !NearlyEqual(
      evidence.original_height_m, original_height_at_cell) ||
    !NearlyEqual(
      evidence.original_rms_residual_m, patch.fit.rms_residual))
  {
    return Fail(
      detail,
      "reconstruction original geometry does not match the patch fit");
  }
  // The estimated half must describe the node that search will actually use.
  if (!NearlySamePoint(
      evidence.reconstructed_centroid, node.plane.centroid,
      kNodeGeometryTolerance) ||
    !NearlySameNormal(
      evidence.reconstructed_normal, node.plane.normal) ||
    !NearlyEqual(
      evidence.reconstructed_height_m, node.plane.centroid.z) ||
    !NearlyEqual(
      evidence.reconstruction_rms_residual_m, node.plane.rms_residual) ||
    !NearlySamePoint(
      evidence.reconstructed_centroid, node.representative,
      kNodeGeometryTolerance))
  {
    return Fail(
      detail,
      "reconstruction geometry does not match the linked node plane");
  }
  if (evidence.shared_corners_valid) {
    // Regional reconstruction stores the actual measured seed-cell count. Its
    // configured region/ref budgets bound that count; the legacy nine-neighbour
    // diagnostic ceiling must not fabricate a smaller number here.
    const double x0 = origin_x + static_cast<double>(patch.cell.x) * resolution;
    const double y0 = origin_y + static_cast<double>(patch.cell.y) * resolution;
    const double x1 = x0 + resolution;
    const double y1 = y0 + resolution;
    const std::array<Point3, 4> & corners = evidence.shared_corners;
    const std::array<std::array<double, 2>, 4> expected_xy{{
      {{x0, y0}}, {{x1, y0}}, {{x1, y1}}, {{x0, y1}}}};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      if (!corners[i].IsFinite() ||
        std::fabs(corners[i].x - expected_xy[i][0]) >
        kNodeGeometryTolerance ||
        std::fabs(corners[i].y - expected_xy[i][1]) >
        kNodeGeometryTolerance)
      {
        return Fail(
          detail,
          "reconstruction shared corner does not match the exact cell square");
      }
    }
    if (evidence.support_nodes == 0U ||
      evidence.support_nodes != evidence.regional_support_cells)
    {
      return Fail(
        detail,
        "regional support count must equal the actual support cells");
    }
    if (evidence.regional_support_cells > evidence.support_points) {
      return Fail(
        detail,
        "regional support cell count exceeds its real support points");
    }
    if (evidence.inferred) {
      if (evidence.support_nodes < 2U ||
        patch.own_cell_sample_count != 0U ||
        !patch.own_cell_point_indices.empty())
      {
        return Fail(
          detail,
          "inferred reconstruction must have two support cells and no own refs");
      }
    } else if (patch.own_cell_sample_count == 0U ||
      patch.own_cell_point_indices.empty())
    {
      return Fail(
        detail,
        "observed reconstruction must retain its own-cell raw refs");
    }
  } else {
    if (evidence.inferred) {
      return Fail(
        detail,
        "inferred reconstruction requires valid shared corners");
    }
    if (evidence.support_nodes < 2U ||
      evidence.support_nodes > kMaxGroundSurfaceNeighbors + 1U ||
      evidence.regional_support_cells != 0U)
    {
      return Fail(
        detail,
        "legacy reconstruction support counts are outside verified bounds");
    }
  }
  if (evidence.support_points < 4U ||
    node.plane.inlier_count < 4U ||
    node.plane.inlier_count > evidence.support_points ||
    node.plane.sample_count != evidence.support_points)
  {
    return Fail(
      detail,
      "reconstruction support counts are outside the verified bounds");
  }
  if (evidence.confidence < 0.0 || evidence.confidence > 1.0) {
    return Fail(detail, "reconstruction confidence must be in [0, 1]");
  }
  if (node.support.points.empty()) {
    return Fail(detail, "applied reconstruction has no raw support points");
  }
  double squared = 0.0;
  for (const Point3 & point : node.support.points) {
    if (!point.IsFinite()) {
      return Fail(
        detail,
        "applied reconstruction raw support contains a non-finite point");
    }
    const double predicted =
      (node.plane.offset - node.plane.normal.x * point.x -
      node.plane.normal.y * point.y) / node.plane.normal.z;
    if (!std::isfinite(predicted)) {
      return Fail(
        detail,
        "applied reconstruction plane cannot evaluate raw support height");
    }
    const double residual = point.z - predicted;
    squared += residual * residual;
  }
  const double own_rms = std::sqrt(
    squared / static_cast<double>(node.support.points.size()));
  if (!std::isfinite(own_rms) ||
    !NearlyEqual(evidence.own_support_rms_residual_m, own_rms))
  {
    return Fail(
      detail,
      "reconstruction own-support RMS does not match raw support residuals");
  }
  return true;
}

bool ValidateLinkedNode(
  const MeasurementPatch & patch,
  const std::unordered_map<TerrainNodeId, const TerrainNode *> & nodes,
  const std::vector<Point3> & points,
  double origin_x,
  double origin_y,
  double resolution,
  std::string * detail)
{
  if (patch.node_id == kInvalidTerrainNodeId) {
    return Fail(detail, "linked patch has no node ID");
  }
  const auto iterator = nodes.find(patch.node_id);
  if (iterator == nodes.end()) {
    return Fail(detail, "patch links an absent node ID");
  }
  const TerrainNode & node = *iterator->second;
  if (!(node.cell == patch.cell)) {
    return Fail(detail, "patch cell does not match its linked node cell");
  }
  if (patch.status == MeasurementPatchStatus::kSupported &&
    node.status != TerrainStatus::kSupported)
  {
    return Fail(detail, "supported patch links a non-supported node");
  }
  if (patch.status == MeasurementPatchStatus::kProhibited &&
    node.status != TerrainStatus::kProhibited)
  {
    return Fail(detail, "prohibited patch links a non-prohibited node");
  }
  if (patch.status == MeasurementPatchStatus::kUnconfirmed &&
    node.status != TerrainStatus::kUnknown)
  {
    return Fail(detail, "unconfirmed patch links a non-unknown node");
  }
  if (!patch.fit.plane_valid || !patch.fit.support_gate_ok ||
    !patch.fit.center_in_support_hull || patch.fit.inlier_indices.empty())
  {
    return Fail(detail, "linked patch lacks valid plane/support evidence");
  }
  if (!node.plane.valid || node.support.sample_count == 0U) {
    return Fail(detail, "linked node lacks valid plane/support evidence");
  }
  if (!ValidateLinkedNodeSupport(patch.fit, node, points, detail)) {
    return false;
  }
  const bool reconstructed = node.surface_reconstruction.applied;
  if (reconstructed) {
    if (!ValidateAppliedSurfaceReconstruction(
        patch, node, origin_x, origin_y, resolution, detail))
    {
      return false;
    }
  } else {
    if (patch.surface_reconstruction.applied ||
      (node.surface_reconstruction.attempted &&
      node.surface_reconstruction.status ==
      SurfaceReconstructionStatus::kApplied))
    {
      return Fail(
        detail,
        "reconstruction provenance claims applied without an applied node");
    }
    const double dx = patch.fit.centroid.x - node.plane.centroid.x;
    const double dy = patch.fit.centroid.y - node.plane.centroid.y;
    const double dz = patch.fit.centroid.z - node.plane.centroid.z;
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz) ||
      std::sqrt(dx * dx + dy * dy + dz * dz) > kNodeGeometryTolerance)
    {
      return Fail(detail, "patch plane centroid does not match the linked node");
    }
    if (node.plane.sample_count != patch.fit.point_indices.size() ||
      node.plane.inlier_count != patch.fit.inlier_indices.size())
    {
      return Fail(
        detail,
        "linked node plane counts do not match the original patch fit");
    }
  }
  const double node_normal_norm = node.plane.normal.Norm();
  const double fit_normal_norm = patch.fit.normal.Norm();
  if (!node.plane.normal.IsFinite() || !std::isfinite(node_normal_norm) ||
    std::fabs(node_normal_norm - 1.0) > kUnitNormalTolerance ||
    !(node.plane.normal.z > 0.0))
  {
    return Fail(
      detail, "linked node plane normal must be finite, unit length, and up");
  }
  if (!patch.fit.normal.IsFinite() || !std::isfinite(fit_normal_norm) ||
    std::fabs(fit_normal_norm - 1.0) > kUnitNormalTolerance)
  {
    return Fail(detail, "patch plane normal must be finite and unit length");
  }
  if (!reconstructed) {
    if (Dot(patch.fit.normal, node.plane.normal) <
      1.0 - kNodeGeometryTolerance)
    {
      return Fail(detail, "patch plane normal does not match the linked node");
    }
    // Both normals are finite and unit length here, so compare components with
    // a direct absolute numerical bound instead of relying only on the dot
    // lower bound (which measures the squared angular difference).
    if (std::fabs(patch.fit.normal.x - node.plane.normal.x) >
      kUnitNormalComponentTolerance ||
      std::fabs(patch.fit.normal.y - node.plane.normal.y) >
      kUnitNormalComponentTolerance ||
      std::fabs(patch.fit.normal.z - node.plane.normal.z) >
      kUnitNormalComponentTolerance)
    {
      return Fail(detail, "patch plane normal does not match the linked node");
    }
  }
  // Active builder convention is the positive offset n . centroid; wheel support
  // evaluates z=(offset-nx*x-ny*y)/nz with it directly. The offset is a dot
  // product, not a Z height: the first tolerance term is the fixed
  // microgeometry Z-height allowance projected onto the unit normal, and the
  // second is a conservative floating-point evaluation-error bound. Scaling by
  // global coordinate magnitude would turn that error bound into a metre-level
  // physical allowance.
  const Vector3 node_centroid{
    node.plane.centroid.x,
    node.plane.centroid.y,
    node.plane.centroid.z};
  const double expected_offset = Dot(node.plane.normal, node_centroid);
  const double offset_scale =
    std::fabs(node.plane.normal.x * node_centroid.x) +
    std::fabs(node.plane.normal.y * node_centroid.y) +
    std::fabs(node.plane.normal.z * node_centroid.z);
  const double offset_tolerance =
    kNodeGeometryTolerance * std::fabs(node.plane.normal.z) +
    kFloatingPointErrorBudget * kMachineEpsilon * (1.0 + offset_scale);
  if (!std::isfinite(node.plane.offset) ||
    !std::isfinite(expected_offset) ||
    !std::isfinite(offset_scale) ||
    !std::isfinite(offset_tolerance) ||
    std::fabs(node.plane.offset - expected_offset) > offset_tolerance)
  {
    return Fail(
      detail, "linked node plane offset does not match its normal/centroid");
  }
  if (!node.representative.IsFinite()) {
    return Fail(detail, "linked node representative is non-finite");
  }
  if (!std::isfinite(patch.fit.normal.z) ||
    !(std::fabs(patch.fit.normal.z) > 0.0))
  {
    return Fail(detail, "patch plane normal has no usable vertical component");
  }
  // Verify the representative against the actual fitted patch plane, centered
  // at the patch centroid. This does not consult the node offset, so a coupled
  // offset/representative error cannot validate itself.
  if (!reconstructed) {
    const Vector3 representative_delta{
      node.representative.x - patch.fit.centroid.x,
      node.representative.y - patch.fit.centroid.y,
      node.representative.z - patch.fit.centroid.z};
    const double representative_scale =
      std::fabs(patch.fit.normal.x * representative_delta.x) +
      std::fabs(patch.fit.normal.y * representative_delta.y) +
      std::fabs(patch.fit.normal.z * representative_delta.z);
    const double centered_z_residual =
      Dot(patch.fit.normal, representative_delta) / patch.fit.normal.z;
    const double representative_tolerance =
      kNodeGeometryTolerance +
      kFloatingPointErrorBudget * kMachineEpsilon *
      (1.0 + representative_scale / std::fabs(patch.fit.normal.z));
    if (!std::isfinite(representative_scale) ||
      !std::isfinite(centered_z_residual) ||
      !std::isfinite(representative_tolerance) ||
      std::fabs(centered_z_residual) > representative_tolerance)
    {
      return Fail(
        detail, "linked node representative is not on the fitted plane");
    }
    if (!NearlyEqual(patch.fit.rms_residual, node.plane.rms_residual)) {
      return Fail(detail, "patch RMS does not match the linked node");
    }
    if (patch.fit.inlier_indices.size() != node.plane.inlier_count) {
      return Fail(detail, "patch inlier count does not match the linked node");
    }
  }
  if (node.support.sample_count < patch.fit.inlier_indices.size()) {
    return Fail(detail, "linked node support is smaller than patch inliers");
  }
  return true;
}

bool ValidateExpansion(
  const MeasurementPatch & patch,
  const std::vector<Point3> & points,
  std::string * detail)
{
  if (!patch.expansion_attempted) {
    if (patch.expansion_used) {
      return Fail(detail, "unattempted expansion cannot be used");
    }
    if (patch.expansion_source != MeasurementExpansionSource::kNone) {
      return Fail(detail, "unattempted expansion must not carry a source");
    }
    if (patch.context_original_reason != MeasurementRejectionReason::kNone ||
      !std::isnan(patch.context_original_slope_rad))
    {
      return Fail(
        detail, "unattempted context re-estimation must not carry provenance");
    }
    if (!std::isnan(patch.expansion.radius_m) ||
      !patch.expansion.point_indices.empty() ||
      patch.expansion.plane_valid ||
      !patch.expansion.inlier_indices.empty() ||
      !std::isnan(patch.expansion.rms_residual) ||
      !std::isnan(patch.expansion.nearest_support_xy_m) ||
      patch.expansion.support_gate_ok ||
      patch.expansion.center_in_support_hull ||
      patch.expansion.conflict_sample_count != 0U ||
      patch.expansion.raised_conflict_sample_count != 0U ||
      !std::isnan(patch.expansion.min_signed_residual) ||
      !std::isnan(patch.expansion.max_signed_residual) ||
      patch.expansion.max_depression_m != 0.0 ||
      patch.expansion.max_raised_m != 0.0 ||
      !IsZeroPoint(patch.expansion.conflict_point) ||
      !IsZeroPoint(patch.expansion.centroid) ||
      patch.expansion.normal.x != 0.0 ||
      patch.expansion.normal.y != 0.0 ||
      patch.expansion.normal.z != 1.0 ||
      patch.expansion_reason != MeasurementRejectionReason::kNone)
    {
      return Fail(detail, "unattempted expansion must not carry attempt data");
    }
    if (patch.fit.point_indices != patch.base_point_indices) {
      return Fail(
        detail, "unattempted fit samples must equal the base point refs");
    }
    return true;
  }

  // Older measured-evidence producers predate this optional source tag. Keep
  // their unspecified retries valid, while requiring context provenance only
  // for an explicitly tagged context re-estimation. All fit/ref checks below
  // still apply to legacy records.
  if (patch.expansion_source == MeasurementExpansionSource::kContextReestimation) {
    if (!IsValidReason(patch.context_original_reason)) {
      return Fail(
        detail, "context re-estimation must preserve the original reason");
    }
    // Insufficiency has no original candidate plane, so an honest NaN slope is
    // valid and must never be fabricated as zero. Every quality/over-slope
    // decision did fit a plane and must preserve its finite non-negative slope.
    const bool insufficient_original =
      patch.context_original_reason ==
      MeasurementRejectionReason::kInsufficientPoints ||
      patch.context_original_reason == MeasurementRejectionReason::kSamplingGap ||
      patch.context_original_reason == MeasurementRejectionReason::kUnsupportedCenter;
    if (insufficient_original) {
      if (!std::isnan(patch.context_original_slope_rad) &&
        (!std::isfinite(patch.context_original_slope_rad) ||
        patch.context_original_slope_rad < 0.0))
      {
        return Fail(
          detail,
          "insufficient context original slope must be NaN or finite/non-negative");
      }
    } else if (!std::isfinite(patch.context_original_slope_rad) ||
      patch.context_original_slope_rad < 0.0)
    {
      return Fail(
        detail,
        "context re-estimation must preserve the original finite slope");
    }
  } else if (patch.context_original_reason != MeasurementRejectionReason::kNone ||
    !std::isnan(patch.context_original_slope_rad))
  {
    return Fail(
      detail, "non-context expansion must not carry context provenance");
  }
  if (!ValidateFitAttempt(patch.expansion, points, "expansion", detail)) {
    return false;
  }
  if (!patch.expansion.plane_valid && patch.expansion_used) {
    return Fail(detail, "used expansion must have a valid candidate plane");
  }
  if (!IsSubset(
      patch.base_point_indices, patch.expansion.point_indices))
  {
    return Fail(detail, "base refs must remain a subset of expansion attempt refs");
  }
  if (patch.expansion_used) {
    if (patch.expansion_reason != MeasurementRejectionReason::kNone) {
      return Fail(detail, "used expansion must not carry a rejection reason");
    }
    if (patch.fit.point_indices != patch.expansion.point_indices ||
      !NearlyEqual(patch.fit.radius_m, patch.expansion.radius_m) ||
      patch.fit.plane_valid != patch.expansion.plane_valid ||
      !PointEquals(patch.fit.centroid, patch.expansion.centroid) ||
      !PointEquals(
        Point3{
          patch.fit.normal.x, patch.fit.normal.y, patch.fit.normal.z},
        Point3{
          patch.expansion.normal.x, patch.expansion.normal.y,
          patch.expansion.normal.z}) ||
      !NearlyEqual(patch.fit.rms_residual, patch.expansion.rms_residual) ||
      patch.fit.inlier_indices != patch.expansion.inlier_indices ||
      patch.fit.conflict_sample_count !=
      patch.expansion.conflict_sample_count ||
      !NearlyEqual(
        patch.fit.min_signed_residual,
        patch.expansion.min_signed_residual) ||
      !NearlyEqual(
        patch.fit.max_signed_residual,
        patch.expansion.max_signed_residual) ||
      !NearlyEqual(
        patch.fit.max_depression_m,
        patch.expansion.max_depression_m) ||
      !NearlyEqual(
        patch.fit.max_raised_m,
        patch.expansion.max_raised_m) ||
      patch.fit.raised_conflict_sample_count !=
      patch.expansion.raised_conflict_sample_count ||
      patch.fit.support_gate_ok != patch.expansion.support_gate_ok ||
      patch.fit.center_in_support_hull !=
      patch.expansion.center_in_support_hull ||
      !NearlyEqual(
        patch.fit.nearest_support_xy_m,
        patch.expansion.nearest_support_xy_m) ||
      !PointEquals(
        patch.fit.conflict_point, patch.expansion.conflict_point))
    {
      return Fail(detail, "used expansion final fit does not match the candidate");
    }
  } else {
    if (patch.expansion_reason == MeasurementRejectionReason::kNone ||
      !IsExpansionFailureReason(patch.expansion_reason))
    {
      return Fail(detail, "failed expansion must record a valid failure reason");
    }
    if (patch.fit.point_indices != patch.base_point_indices) {
      return Fail(detail, "failed expansion must keep the base point refs");
    }
    if (!(patch.expansion.radius_m > patch.fit.radius_m)) {
      return Fail(detail, "failed expansion radius must exceed the base radius");
    }
  }
  return true;
}

bool ValidatePatch(
  const MeasurementPatch & patch,
  const std::vector<Point3> & points,
  const std::unordered_map<TerrainNodeId, const TerrainNode *> & nodes,
  double origin_x,
  double origin_y,
  double resolution,
  std::string * detail)
{
  if (patch.id == 0U) {
    return Fail(detail, "patch ID zero is reserved");
  }
  if (!IsSafeFineCell(patch.cell)) {
    return Fail(detail, "patch has an unsafe cell index");
  }
  if (patch.local_layer_index < 0) {
    return Fail(detail, "patch has a negative local layer index");
  }
  if (!ValidateIndices(
      patch.base_point_indices, points.size(),
      "base_point_indices", detail))
  {
    return false;
  }
  if (patch.base_point_indices.empty()) {
    return Fail(detail, "patch has no base sample refs");
  }
  if (patch.base_sample_count != patch.base_point_indices.size()) {
    return Fail(detail, "base_sample_count does not match base point refs");
  }
  if (!ValidateFitAttempt(patch.fit, points, "fit", detail)) {
    return false;
  }
  if (!IsSubset(patch.base_point_indices, patch.fit.point_indices)) {
    return Fail(detail, "base point refs are not a subset of final refs");
  }
  if (!ValidateExpansion(patch, points, detail)) {
    return false;
  }

  if (!ValidateIndices(
      patch.own_cell_point_indices, points.size(),
      "own_cell_point_indices", detail))
  {
    return false;
  }
  if (patch.own_cell_sample_count != patch.own_cell_point_indices.size()) {
    return Fail(detail, "own_cell_sample_count does not match own point refs");
  }
  std::vector<std::uint32_t> expected_own;
  for (const std::uint32_t index : patch.base_point_indices) {
    const auto cell = FineCellOf(
      points[index], origin_x, origin_y, resolution);
    if (cell.has_value() && *cell == patch.cell) {
      expected_own.push_back(index);
    }
  }
  if (patch.own_cell_point_indices != expected_own) {
    return Fail(
      detail,
      "own-cell refs must equal the complete base-refs subset in patch.cell");
  }

  double expected_own_z_min = 0.0;
  double expected_own_z_max = 0.0;
  ComputeZBounds(
    points, patch.own_cell_point_indices,
    &expected_own_z_min, &expected_own_z_max);
  if (patch.own_cell_point_indices.empty()) {
    if (!std::isnan(patch.own_z_min) || !std::isnan(patch.own_z_max)) {
      return Fail(detail, "unobserved own cell must keep own Z bounds NaN");
    }
    if (!std::isnan(patch.own_cell_min_signed_residual) ||
      !std::isnan(patch.own_cell_max_signed_residual) ||
      patch.own_cell_max_depression_m != 0.0 ||
      patch.own_cell_max_raised_m != 0.0)
    {
      return Fail(detail, "unobserved own cell must not carry own residuals");
    }
  } else if (!NearlyEqual(patch.own_z_min, expected_own_z_min) ||
    !NearlyEqual(patch.own_z_max, expected_own_z_max))
  {
    return Fail(detail, "own Z bounds do not match own-cell sample refs");
  }

  double expected_z_min = 0.0;
  double expected_z_max = 0.0;
  ComputeZBounds(points, patch.fit.point_indices, &expected_z_min, &expected_z_max);
  if (!NearlyEqual(patch.z_min, expected_z_min) ||
    !NearlyEqual(patch.z_max, expected_z_max))
  {
    return Fail(detail, "z_min/z_max do not match the final sample refs");
  }

  if (!patch.own_cell_point_indices.empty()) {
    if (!patch.fit.plane_valid) {
      if (!std::isnan(patch.own_cell_min_signed_residual) ||
        !std::isnan(patch.own_cell_max_signed_residual) ||
        patch.own_cell_max_depression_m != 0.0 ||
        patch.own_cell_max_raised_m != 0.0)
      {
        return Fail(detail, "own residuals require a valid final plane");
      }
    } else {
      double own_min = 0.0;
      double own_max = 0.0;
      if (!ComputeResidualExtrema(
          points, patch.own_cell_point_indices, patch.fit.centroid,
          patch.fit.normal, &own_min, &own_max))
      {
        return Fail(detail, "could not reconstruct own-cell residuals");
      }
      if (!NearlyEqual(patch.own_cell_min_signed_residual, own_min) ||
        !NearlyEqual(patch.own_cell_max_signed_residual, own_max) ||
        !NearlyEqual(
          patch.own_cell_max_depression_m, std::max(0.0, -own_min)) ||
        !NearlyEqual(
          patch.own_cell_max_raised_m, std::max(0.0, own_max)))
      {
        return Fail(detail, "own-cell residual statistics do not match samples");
      }
    }
  }

  if (!ValidateStatusReason(patch, detail)) {
    return false;
  }
  if (patch.status == MeasurementPatchStatus::kSupported ||
    patch.status == MeasurementPatchStatus::kProhibited ||
    patch.status == MeasurementPatchStatus::kUnconfirmed)
  {
    return ValidateLinkedNode(
      patch, nodes, points, origin_x, origin_y, resolution, detail);
  }
  if (patch.node_id != kInvalidTerrainNodeId) {
    return Fail(detail, "unlinked patch must not carry a node ID");
  }
  return true;
}

void HashBytes(std::uint64_t * hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= kHashPrime;
  }
}

void HashU64(std::uint64_t * hash, std::uint64_t value)
{
  HashBytes(hash, &value, sizeof(value));
}

void HashSize(std::uint64_t * hash, std::size_t value)
{
  HashU64(hash, static_cast<std::uint64_t>(value));
}

void HashBool(std::uint64_t * hash, bool value)
{
  const std::uint8_t byte = value ? 1U : 0U;
  HashBytes(hash, &byte, sizeof(byte));
}

void HashDouble(std::uint64_t * hash, double value)
{
  std::uint64_t bits = 0;
  if (std::isnan(value)) {
    bits = kCanonicalNaNBits;
  } else {
    const double canonical = value == 0.0 ? 0.0 : value;
    std::memcpy(&bits, &canonical, sizeof(bits));
  }
  HashU64(hash, bits);
}

void HashPoint(std::uint64_t * hash, const Point3 & point)
{
  HashDouble(hash, point.x);
  HashDouble(hash, point.y);
  HashDouble(hash, point.z);
}

void HashVector(std::uint64_t * hash, const Vector3 & vector)
{
  HashDouble(hash, vector.x);
  HashDouble(hash, vector.y);
  HashDouble(hash, vector.z);
}

void HashIndexVector(
  std::uint64_t * hash,
  const std::vector<std::uint32_t> & indices)
{
  HashSize(hash, indices.size());
  for (const std::uint32_t index : indices) {
    HashU64(hash, static_cast<std::uint64_t>(index));
  }
}

void HashFitAttempt(
  std::uint64_t * hash,
  const MeasurementFitAttempt & attempt)
{
  HashDouble(hash, attempt.radius_m);
  HashIndexVector(hash, attempt.point_indices);
  HashIndexVector(hash, attempt.inlier_indices);
  HashBool(hash, attempt.plane_valid);
  HashPoint(hash, attempt.centroid);
  HashVector(hash, attempt.normal);
  HashDouble(hash, attempt.rms_residual);
  HashDouble(hash, attempt.nearest_support_xy_m);
  HashBool(hash, attempt.support_gate_ok);
  HashBool(hash, attempt.center_in_support_hull);
  HashSize(hash, attempt.conflict_sample_count);
  HashDouble(hash, attempt.min_signed_residual);
  HashDouble(hash, attempt.max_signed_residual);
  HashDouble(hash, attempt.max_depression_m);
  HashDouble(hash, attempt.max_raised_m);
  HashSize(hash, attempt.raised_conflict_sample_count);
  HashPoint(hash, attempt.conflict_point);
}

void HashSurfaceReconstruction(
  std::uint64_t * hash, const SurfaceReconstructionEvidence & evidence)
{
  HashBool(hash, evidence.attempted);
  HashBool(hash, evidence.applied);
  HashU64(hash, static_cast<std::uint64_t>(evidence.status));
  HashPoint(hash, evidence.original_centroid);
  HashVector(hash, evidence.original_normal);
  HashPoint(hash, evidence.reconstructed_centroid);
  HashVector(hash, evidence.reconstructed_normal);
  HashDouble(hash, evidence.original_height_m);
  HashDouble(hash, evidence.reconstructed_height_m);
  HashDouble(hash, evidence.correction_m);
  HashDouble(hash, evidence.original_rms_residual_m);
  HashDouble(hash, evidence.reconstruction_rms_residual_m);
  HashDouble(hash, evidence.own_support_rms_residual_m);
  HashDouble(hash, evidence.confidence);
  HashSize(hash, evidence.support_nodes);
  HashSize(hash, evidence.support_points);
  HashBool(hash, evidence.inferred);
  HashBool(hash, evidence.shared_corners_valid);
  for (const Point3 & corner : evidence.shared_corners) {
    HashPoint(hash, corner);
  }
  HashSize(hash, evidence.regional_support_cells);
}

void HashPatch(std::uint64_t * hash, const MeasurementPatch & patch)
{
  HashU64(hash, patch.id);
  HashU64(hash, static_cast<std::uint64_t>(patch.cell.x));
  HashU64(hash, static_cast<std::uint64_t>(patch.cell.y));
  HashU64(hash, static_cast<std::uint64_t>(patch.local_layer_index));

  HashIndexVector(hash, patch.base_point_indices);
  HashSize(hash, patch.base_sample_count);
  HashFitAttempt(hash, patch.fit);

  HashBool(hash, patch.expansion_attempted);
  HashBool(hash, patch.expansion_used);
  HashFitAttempt(hash, patch.expansion);
  HashU64(hash, static_cast<std::uint64_t>(patch.expansion_reason));
  HashU64(hash, static_cast<std::uint64_t>(patch.expansion_source));
  HashU64(hash, static_cast<std::uint64_t>(patch.context_original_reason));
  HashDouble(hash, patch.context_original_slope_rad);

  HashIndexVector(hash, patch.own_cell_point_indices);
  HashSize(hash, patch.own_cell_sample_count);
  HashDouble(hash, patch.own_z_min);
  HashDouble(hash, patch.own_z_max);
  HashDouble(hash, patch.own_cell_min_signed_residual);
  HashDouble(hash, patch.own_cell_max_signed_residual);
  HashDouble(hash, patch.own_cell_max_depression_m);
  HashDouble(hash, patch.own_cell_max_raised_m);

  HashDouble(hash, patch.z_min);
  HashDouble(hash, patch.z_max);

  HashU64(hash, static_cast<std::uint64_t>(patch.status));
  HashU64(hash, static_cast<std::uint64_t>(patch.reason));
  HashSurfaceReconstruction(hash, patch.surface_reconstruction);
  HashU64(hash, patch.node_id);
}

}  // namespace

const char * MeasurementPatchStatusToString(
  MeasurementPatchStatus status) noexcept
{
  switch (status) {
    case MeasurementPatchStatus::kObservedOnly:
      return "observed_only";
    case MeasurementPatchStatus::kFitRejected:
      return "fit_rejected";
    case MeasurementPatchStatus::kProhibited:
      return "prohibited";
    case MeasurementPatchStatus::kSupported:
      return "supported";
    case MeasurementPatchStatus::kUnconfirmed:
      return "unconfirmed";
  }
  return "unknown_measurement_status";
}

const char * MeasurementRejectionReasonToString(
  MeasurementRejectionReason reason) noexcept
{
  switch (reason) {
    case MeasurementRejectionReason::kNone:
      return "none";
    case MeasurementRejectionReason::kInsufficientPoints:
      return "insufficient_points";
    case MeasurementRejectionReason::kSamplingGap:
      return "sampling_gap";
    case MeasurementRejectionReason::kUnsupportedCenter:
      return "unsupported_center";
    case MeasurementRejectionReason::kDegenerateGeometry:
      return "degenerate_geometry";
    case MeasurementRejectionReason::kHighResidual:
      return "high_residual";
    case MeasurementRejectionReason::kInsufficientInliers:
      return "insufficient_inliers";
    case MeasurementRejectionReason::kNonFinitePlane:
      return "nonfinite_plane";
    case MeasurementRejectionReason::kExcessiveSlope:
      return "excessive_slope";
    case MeasurementRejectionReason::kLocalDepression:
      return "local_depression";
    case MeasurementRejectionReason::kLocalRaisedConflict:
      return "local_raised_conflict";
    case MeasurementRejectionReason::kExpansionInvalid:
      return "expansion_invalid";
    case MeasurementRejectionReason::kDiscontinuousTerrain:
      return "discontinuous_terrain";
    case MeasurementRejectionReason::kInsufficientCoverage:
      return "insufficient_coverage";
    case MeasurementRejectionReason::kUnconfirmedComponent:
      return "unconfirmed_component";
  }
  return "unknown_measurement_reason";
}

bool ValidateMeasurementEvidence(
  const MeasurementEvidence & evidence,
  const std::vector<TerrainNode> & nodes,
  double origin_x,
  double origin_y,
  double resolution,
  std::string * detail)
{
  if (!IsSafeWorldCoordinate(origin_x) ||
    !IsSafeWorldCoordinate(origin_y) ||
    !IsValidResolution(resolution))
  {
    return Fail(detail, "measurement evidence frame is invalid");
  }
  if (evidence.points.size() >
    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    return Fail(
      detail, "measurement point table exceeds the uint32 index range");
  }
  for (std::size_t index = 0; index < evidence.points.size(); ++index) {
    const Point3 & point = evidence.points[index];
    if (!point.IsFinite()) {
      return Fail(detail, "measurement point table contains a non-finite point");
    }
    if (index > 0U) {
      const Point3 & previous = evidence.points[index - 1U];
      const bool strictly_increasing =
        previous.x < point.x ||
        (previous.x == point.x &&
        (previous.y < point.y ||
        (previous.y == point.y && previous.z < point.z)));
      if (!strictly_increasing) {
        return Fail(
          detail, "measurement points must be strictly increasing by (x,y,z)");
      }
    }
  }

  std::unordered_map<TerrainNodeId, const TerrainNode *> node_by_id;
  node_by_id.reserve(nodes.size());
  for (const TerrainNode & node : nodes) {
    node_by_id.emplace(node.id, &node);
  }

  std::unordered_set<std::uint64_t> patch_ids;
  patch_ids.reserve(evidence.patches.size());
  std::unordered_set<TerrainNodeId> linked_nodes;
  linked_nodes.reserve(evidence.patches.size());
  for (std::size_t index = 0; index < evidence.patches.size(); ++index) {
    const MeasurementPatch & patch = evidence.patches[index];
    if (!patch_ids.insert(patch.id).second) {
      return Fail(
        detail, "duplicate measurement patch ID " +
        std::to_string(patch.id));
    }
    if (index > 0U) {
      const MeasurementPatch & previous = evidence.patches[index - 1U];
      const bool ordered =
        previous.cell.x < patch.cell.x ||
        (previous.cell.x == patch.cell.x &&
        (previous.cell.y < patch.cell.y ||
        (previous.cell.y == patch.cell.y &&
        (previous.local_layer_index < patch.local_layer_index ||
        (previous.local_layer_index == patch.local_layer_index &&
        previous.id < patch.id)))));
      if (!ordered) {
        return Fail(
          detail,
          "measurement patches must be strictly ordered by cell/layer/id");
      }
    }
    if (!ValidatePatch(
        patch, evidence.points, node_by_id, origin_x, origin_y, resolution,
        detail))
    {
      return false;
    }
    if (patch.node_id != kInvalidTerrainNodeId &&
      !linked_nodes.insert(patch.node_id).second)
    {
      return Fail(detail, "one node is linked by more than one measurement patch");
    }
  }
  return true;
}

std::uint64_t HashMeasurementEvidence(
  const MeasurementEvidence & evidence) noexcept
{
  if (evidence.empty()) {
    return 0U;
  }
  std::uint64_t hash = kHashOffset;
  HashU64(&hash, 4U);  // Evidence content version after surface provenance.

  HashSize(&hash, evidence.points.size());
  for (const Point3 & point : evidence.points) {
    HashPoint(&hash, point);
  }

  HashSize(&hash, evidence.patches.size());
  for (const MeasurementPatch & patch : evidence.patches) {
    HashPatch(&hash, patch);
  }
  return hash == 0U ? 1U : hash;
}

}  // namespace dddmr_grid_global_planner
