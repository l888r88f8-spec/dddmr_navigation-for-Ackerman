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

#include "dddmr_grid_global_planner/offline_surface_evidence.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/offline_static_map.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

struct Point2
{
  double x{0.0};
  double y{0.0};
};

double Cross2(
  const Point2 & origin, const Point2 & first,
  const Point2 & second) noexcept
{
  return (first.x - origin.x) * (second.y - origin.y) -
         (first.y - origin.y) * (second.x - origin.x);
}

// Monotone-chain hull over the actual measured support points. No point is
// added, moved or interpolated.
std::vector<Point2> ConvexHull(std::vector<Point2> points)
{
  if (points.size() < 3U) {
    return points;
  }
  std::sort(
    points.begin(), points.end(),
    [](const Point2 & lhs, const Point2 & rhs) {
      return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    });
  points.erase(
    std::unique(
      points.begin(), points.end(),
      [](const Point2 & lhs, const Point2 & rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
      }),
    points.end());
  if (points.size() < 3U) {
    return points;
  }
  std::vector<Point2> hull(points.size() * 2U);
  std::size_t size = 0U;
  for (const Point2 & point : points) {
    while (size >= 2U &&
      Cross2(hull[size - 2U], hull[size - 1U], point) <= 0.0)
    {
      --size;
    }
    hull[size++] = point;
  }
  const std::size_t lower_size = size;
  for (std::size_t index = points.size() - 1U; index-- > 0U; ) {
    const Point2 & point = points[index];
    while (size > lower_size &&
      Cross2(hull[size - 2U], hull[size - 1U], point) <= 0.0)
    {
      --size;
    }
    hull[size++] = point;
  }
  hull.resize(size - 1U);
  return hull;
}

bool PointInHull(
  const std::vector<Point2> & hull, double x, double y,
  double tolerance) noexcept
{
  if (hull.size() < 3U) {
    return false;
  }
  for (std::size_t index = 0; index < hull.size(); ++index) {
    const Point2 & first = hull[index];
    const Point2 & second = hull[(index + 1U) % hull.size()];
    const double edge_length =
      std::hypot(second.x - first.x, second.y - first.y);
    if (Cross2(first, second, Point2{x, y}) < -tolerance * edge_length) {
      return false;
    }
  }
  return true;
}

std::vector<Point3> Materialize(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices)
{
  std::vector<Point3> samples;
  samples.reserve(indices.size());
  for (const std::uint32_t index : indices) {
    samples.push_back(points[index]);
  }
  return samples;
}

double NearestSupportXY(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center) noexcept
{
  double nearest = std::numeric_limits<double>::infinity();
  for (const std::uint32_t index : indices) {
    nearest = std::min(
      nearest, std::hypot(
        points[index].x - center.x, points[index].y - center.y));
  }
  return nearest;
}

// Z-height residual using the same centered normal convention as
// MeasurementFitAttempt validation.
bool PlaneResidual(
  const offline_geometry::PlaneFit & plane, const Point3 & point,
  double * residual) noexcept
{
  if (!plane.valid || !std::isfinite(plane.normal.z) ||
    !(std::fabs(plane.normal.z) > 0.0))
  {
    return false;
  }
  const double numerator =
    plane.normal.x * (point.x - plane.centroid.x) +
    plane.normal.y * (point.y - plane.centroid.y) +
    plane.normal.z * (point.z - plane.centroid.z);
  const double value = numerator / plane.normal.z;
  if (!std::isfinite(value)) {
    return false;
  }
  *residual = value;
  return true;
}

bool SamePoint(const Point3 & first, const Point3 & second) noexcept
{
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

// Maps robust inlier coordinates back to their exact source refs. FitRobustPlane
// preserves the materialized sample order, so this is a deterministic
// subsequence match and never invents an index for a duplicated coordinate.
std::vector<std::uint32_t> MapInlierRefs(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & source_indices,
  const std::vector<Point3> & inliers)
{
  std::vector<std::uint32_t> refs;
  refs.reserve(inliers.size());
  std::size_t search = 0U;
  for (const Point3 & inlier : inliers) {
    while (search < source_indices.size() &&
      !SamePoint(points[source_indices[search]], inlier))
    {
      ++search;
    }
    if (search >= source_indices.size()) {
      return {};
    }
    refs.push_back(source_indices[search]);
    ++search;
  }
  return refs;
}

MeasurementRejectionReason RobustFailureReason(
  offline_geometry::PlaneFitStatus status) noexcept
{
  switch (status) {
    case offline_geometry::PlaneFitStatus::kInsufficientPoints:
      return MeasurementRejectionReason::kInsufficientInliers;
    case offline_geometry::PlaneFitStatus::kDegenerate:
      return MeasurementRejectionReason::kDegenerateGeometry;
    case offline_geometry::PlaneFitStatus::kRmsTooHigh:
      return MeasurementRejectionReason::kHighResidual;
    case offline_geometry::PlaneFitStatus::kOk:
      return MeasurementRejectionReason::kNone;
  }
  return MeasurementRejectionReason::kDegenerateGeometry;
}

}  // namespace

SurfaceSupportGate ValidateMeasuredSupport(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center,
  const OfflineStaticMapOptions & options,
  double * nearest_xy)
{
  if (nearest_xy == nullptr || indices.empty()) {
    return SurfaceSupportGate::kUnsupportedCenter;
  }
  *nearest_xy = NearestSupportXY(points, indices, center);
  if (!std::isfinite(*nearest_xy)) {
    return SurfaceSupportGate::kUnsupportedCenter;
  }
  if (*nearest_xy > options.max_sampling_gap_m) {
    return SurfaceSupportGate::kSamplingGap;
  }
  std::vector<Point2> xy;
  xy.reserve(indices.size());
  for (const std::uint32_t index : indices) {
    xy.push_back(Point2{points[index].x, points[index].y});
  }
  const std::vector<Point2> hull = ConvexHull(std::move(xy));
  if (!PointInHull(
      hull, center.x, center.y, 0.5 * options.resolution))
  {
    return SurfaceSupportGate::kUnsupportedCenter;
  }
  return SurfaceSupportGate::kOk;
}

SurfaceAttempt EvaluateSurfaceAttempt(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center,
  const FineCellIndex & cell,
  const OfflineStaticMapOptions & options)
{
  SurfaceAttempt attempt;
  attempt.point_indices = indices;
  attempt.sample_count = indices.size();
  attempt.nearest_support_xy_m = NearestSupportXY(points, indices, center);

  // Complete own-cell refs and their actual Z bounds are retained even when no
  // plane is recovered, so an unobserved cell cannot be confused with a
  // neighbourhood attempt.
  for (const std::uint32_t index : indices) {
    const auto point_cell = FineCellOf(
      points[index], options.origin_x, options.origin_y,
      options.resolution);
    if (point_cell.has_value() && *point_cell == cell) {
      attempt.own_cell_indices.push_back(index);
    }
  }
  if (!attempt.own_cell_indices.empty()) {
    attempt.own_z_min = points[attempt.own_cell_indices.front()].z;
    attempt.own_z_max = attempt.own_z_min;
    for (const std::uint32_t index : attempt.own_cell_indices) {
      attempt.own_z_min = std::min(attempt.own_z_min, points[index].z);
      attempt.own_z_max = std::max(attempt.own_z_max, points[index].z);
    }
  }

  if (indices.size() < options.min_neighborhood_points) {
    attempt.reason = MeasurementRejectionReason::kInsufficientPoints;
    return attempt;
  }

  double nearest = std::numeric_limits<double>::quiet_NaN();
  attempt.support_gate = ValidateMeasuredSupport(
    points, indices, center, options, &nearest);
  attempt.nearest_support_xy_m = nearest;
  attempt.center_in_support_hull =
    attempt.support_gate == SurfaceSupportGate::kOk;
  if (attempt.support_gate != SurfaceSupportGate::kOk) {
    attempt.reason = attempt.support_gate == SurfaceSupportGate::kSamplingGap ?
      MeasurementRejectionReason::kSamplingGap :
      MeasurementRejectionReason::kUnsupportedCenter;
    return attempt;
  }

  const std::vector<Point3> samples = Materialize(points, indices);
  offline_geometry::PlaneFit ordinary;
  attempt.ordinary_fit_valid =
    offline_geometry::FitPlane(samples, &ordinary);
  attempt.ordinary_rms_accepted =
    attempt.ordinary_fit_valid &&
    ordinary.rms <= options.max_plane_rms_m;

  offline_geometry::PlaneFit candidate = ordinary;
  bool candidate_valid = attempt.ordinary_fit_valid;
  std::vector<std::uint32_t> candidate_inliers = indices;
  bool robust_used = false;
  MeasurementRejectionReason gate_reason = MeasurementRejectionReason::kNone;

  if (!attempt.ordinary_rms_accepted) {
    attempt.robust_attempted = true;
    const offline_geometry::RobustPlaneFit robust =
      offline_geometry::FitRobustPlane(
      samples, options.min_neighborhood_points,
      options.max_plane_rms_m);
    attempt.robust_status = robust.status;
    attempt.robust_iterations = robust.iterations;
    attempt.robust_geometry_accepted = robust.valid();
    if (robust.valid()) {
      std::vector<std::uint32_t> robust_refs =
        MapInlierRefs(points, indices, robust.inliers);
      if (robust_refs.size() != robust.inliers.size() ||
        robust_refs.empty())
      {
        candidate_valid = false;
        gate_reason = MeasurementRejectionReason::kDegenerateGeometry;
      } else {
        candidate = robust.plane;
        candidate_valid = true;
        candidate_inliers = std::move(robust_refs);
        robust_used = true;
        attempt.robust_used = true;
        double robust_nearest = std::numeric_limits<double>::quiet_NaN();
        attempt.support_gate = ValidateMeasuredSupport(
          points, candidate_inliers, center, options, &robust_nearest);
        attempt.nearest_support_xy_m = robust_nearest;
        attempt.center_in_support_hull =
          attempt.support_gate == SurfaceSupportGate::kOk;
        if (attempt.support_gate != SurfaceSupportGate::kOk) {
          gate_reason =
            attempt.support_gate == SurfaceSupportGate::kSamplingGap ?
            MeasurementRejectionReason::kSamplingGap :
            MeasurementRejectionReason::kUnsupportedCenter;
        }
      }
    } else {
      // A valid ordinary plane remains real rejected-attempt evidence even when
      // the bounded robust fallback cannot recover a final surface.
      candidate_valid = attempt.ordinary_fit_valid;
      candidate = ordinary;
      candidate_inliers = indices;
      gate_reason = RobustFailureReason(robust.status);
      if (gate_reason == MeasurementRejectionReason::kNone) {
        gate_reason = MeasurementRejectionReason::kDegenerateGeometry;
      }
    }
  }

  if (!candidate_valid) {
    attempt.reason = gate_reason;
    if (attempt.reason != MeasurementRejectionReason::kInsufficientPoints &&
      attempt.reason != MeasurementRejectionReason::kSamplingGap &&
      attempt.reason != MeasurementRejectionReason::kUnsupportedCenter &&
      attempt.reason != MeasurementRejectionReason::kDegenerateGeometry &&
      attempt.reason != MeasurementRejectionReason::kNonFinitePlane)
    {
      // With no retained candidate plane, the observed-only evidence contract
      // can only report a geometry/non-finite failure; the robust diagnostic
      // status still remains available in robust_status.
      attempt.reason = MeasurementRejectionReason::kDegenerateGeometry;
    }
    return attempt;
  }

  const double center_z =
    candidate.a * center.x + candidate.b * center.y + candidate.c;
  if (!std::isfinite(center_z)) {
    attempt.reason = MeasurementRejectionReason::kNonFinitePlane;
    return attempt;
  }

  // Full-sample residual scan. This finds own-cell negative evidence outside
  // robust inliers and records every real positive obstacle sample.
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  std::uint32_t minimum_index = indices.front();
  std::uint32_t maximum_index = indices.front();
  for (const std::uint32_t index : indices) {
    double residual = 0.0;
    if (!PlaneResidual(candidate, points[index], &residual)) {
      attempt.reason = MeasurementRejectionReason::kNonFinitePlane;
      return attempt;
    }
    if (residual < minimum) {
      minimum = residual;
      minimum_index = index;
    }
    if (residual > maximum) {
      maximum = residual;
      maximum_index = index;
    }
    if (residual > options.ground_duplicate_tolerance_m + kEpsilon &&
      residual <= options.obstacle_blocking_height_m + kEpsilon)
    {
      attempt.raised_conflict_indices.push_back(index);
    }
  }
  attempt.plane = candidate;
  attempt.inlier_indices = candidate_inliers;
  attempt.min_signed_residual = minimum;
  attempt.max_signed_residual = maximum;
  attempt.max_depression_m = std::max(0.0, -minimum);
  attempt.max_raised_m = std::max(0.0, maximum);
  attempt.raised_conflict_sample_count =
    attempt.raised_conflict_indices.size();
  attempt.conflict_point = minimum < 0.0 ?
    points[minimum_index] : points[maximum_index];

  // Complete own-cell scan, including robust-excluded points. Only this own-cell
  // depression extends the original physical step protection; the full-sample
  // negative minimum is diagnostic and never a neighbourhood-wide veto.
  bool own_plane_valid = true;
  if (!attempt.own_cell_indices.empty()) {
    double own_min = std::numeric_limits<double>::infinity();
    double own_max = -std::numeric_limits<double>::infinity();
    for (const std::uint32_t index : attempt.own_cell_indices) {
      double residual = 0.0;
      if (!PlaneResidual(candidate, points[index], &residual)) {
        own_plane_valid = false;
        break;
      }
      own_min = std::min(own_min, residual);
      own_max = std::max(own_max, residual);
    }
    if (own_plane_valid) {
      attempt.own_cell_min_signed_residual = own_min;
      attempt.own_cell_max_signed_residual = own_max;
      attempt.own_cell_max_depression_m = std::max(0.0, -own_min);
      attempt.own_cell_max_raised_m = std::max(0.0, own_max);
    }
  }
  if (!own_plane_valid) {
    attempt.reason = MeasurementRejectionReason::kNonFinitePlane;
    return attempt;
  }

  if (attempt.own_cell_max_depression_m >
    options.max_step_height_m + kEpsilon)
  {
    attempt.reason = MeasurementRejectionReason::kLocalDepression;
    return attempt;
  }
  if (gate_reason != MeasurementRejectionReason::kNone) {
    attempt.reason = gate_reason;
    return attempt;
  }
  if (!attempt.ordinary_rms_accepted && !robust_used) {
    attempt.reason = MeasurementRejectionReason::kHighResidual;
    return attempt;
  }
  attempt.accepted = true;
  attempt.reason = MeasurementRejectionReason::kNone;
  return attempt;
}

}  // namespace dddmr_grid_global_planner
