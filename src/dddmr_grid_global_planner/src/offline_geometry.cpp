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

#include "grid_global_planner/offline_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace grid_global_planner
{
namespace offline_geometry
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-12;

// Stable residual against a plane whose coefficients were recovered in centred
// coordinates. Subtracting the centroid before evaluating the slope terms avoids
// cancellation at large valid world coordinates.
double PlaneResidual(const PlaneFit & plane, const Point3 & point)
{
  return point.z - plane.centroid.z -
         plane.a * (point.x - plane.centroid.x) -
         plane.b * (point.y - plane.centroid.y);
}

bool SameMembership(
  const std::vector<Point3> & first,
  const std::vector<Point3> & second)
{
  if (first.size() != second.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < first.size(); ++index) {
    if (first[index].x != second[index].x ||
      first[index].y != second[index].y ||
      first[index].z != second[index].z)
    {
      return false;
    }
  }
  return true;
}

bool InliersMatchPlane(
  const PlaneFit & plane, const std::vector<Point3> & inliers,
  double max_rms)
{
  for (const Point3 & point : inliers) {
    const double residual = PlaneResidual(plane, point);
    if (!std::isfinite(residual) ||
      std::abs(residual) > max_rms + kGeometryEpsilon)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

const char * PlaneFitStatusToString(PlaneFitStatus status) noexcept
{
  switch (status) {
    case PlaneFitStatus::kOk:
      return "ok";
    case PlaneFitStatus::kInsufficientPoints:
      return "insufficient_points";
    case PlaneFitStatus::kDegenerate:
      return "degenerate";
    case PlaneFitStatus::kRmsTooHigh:
      return "rms_too_high";
  }
  return "unknown";
}

bool FitPlane(const std::vector<Point3> & points, PlaneFit * plane)
{
  if (plane == nullptr || points.size() < 3U) {
    return false;
  }
  const double count = static_cast<double>(points.size());
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  for (const Point3 & point : points) {
    if (!point.IsFinite()) {
      return false;
    }
    center_x += point.x;
    center_y += point.y;
    center_z += point.z;
  }
  center_x /= count;
  center_y /= count;
  center_z /= count;

  // Normal equations in centred local coordinates. This avoids subtracting
  // nearly equal absolute sums at large valid world coordinates.
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  double sum_xz = 0.0;
  double sum_yz = 0.0;
  for (const Point3 & point : points) {
    const double x = point.x - center_x;
    const double y = point.y - center_y;
    const double z = point.z - center_z;
    sum_xx += x * x;
    sum_xy += x * y;
    sum_yy += y * y;
    sum_xz += x * z;
    sum_yz += y * z;
  }
  const double xy_scale = sum_xx * sum_yy;
  if (!std::isfinite(xy_scale) || xy_scale <= 0.0) {
    return false;
  }
  const double determinant = xy_scale - sum_xy * sum_xy;
  if (!std::isfinite(determinant) ||
    determinant / xy_scale <= 1.0e-12)
  {
    return false;
  }
  const double a =
    (sum_xz * sum_yy - sum_yz * sum_xy) / determinant;
  const double b =
    (sum_yz * sum_xx - sum_xz * sum_xy) / determinant;
  const double c = center_z - a * center_x - b * center_y;
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) {
    return false;
  }

  double squared = 0.0;
  for (const Point3 & point : points) {
    const double residual = point.z - center_z -
      a * (point.x - center_x) - b * (point.y - center_y);
    squared += residual * residual;
  }
  const double rms = std::sqrt(squared / count);
  const double norm = std::sqrt(a * a + b * b + 1.0);
  plane->centroid = Point3{center_x, center_y, center_z};
  plane->a = a;
  plane->b = b;
  plane->c = c;
  plane->rms = rms;
  plane->normal = Vector3{-a / norm, -b / norm, 1.0 / norm};
  plane->valid = std::isfinite(rms) && rms >= 0.0;
  return plane->valid;
}

RobustPlaneFit FitRobustPlane(
  const std::vector<Point3> & points, std::size_t min_points,
  double max_rms)
{
  RobustPlaneFit result;
  if (points.size() < min_points || min_points < 3U ||
    !std::isfinite(max_rms) || max_rms <= 0.0)
  {
    result.status = PlaneFitStatus::kInsufficientPoints;
    return result;
  }
  PlaneFit current;
  if (!FitPlane(points, &current)) {
    result.status = PlaneFitStatus::kDegenerate;
    return result;
  }

  // Fixed final consensus relative to the original layer. Four of many points
  // is never a defensible surface merely because its hull happens to cover the
  // centre.
  const std::size_t required_inliers = std::max(
    min_points, points.size() - points.size() / 3U);
  constexpr std::size_t kTrimPasses = 4U;
  constexpr std::size_t kGroupRefits = 2U;
  std::vector<std::size_t> order(points.size());
  for (std::size_t pass = 0U; pass < kTrimPasses; ++pass) {
    for (std::size_t index = 0U; index < points.size(); ++index) {
      order[index] = index;
    }
    std::sort(
      order.begin(), order.end(),
      [&](std::size_t lhs, std::size_t rhs) {
        const double lhs_residual = std::abs(
          PlaneResidual(current, points[lhs]));
        const double rhs_residual = std::abs(
          PlaneResidual(current, points[rhs]));
        if (lhs_residual != rhs_residual) {
          return lhs_residual < rhs_residual;
        }
        if (points[lhs].x != points[rhs].x) {
          return points[lhs].x < points[rhs].x;
        }
        if (points[lhs].y != points[rhs].y) {
          return points[lhs].y < points[rhs].y;
        }
        return points[lhs].z < points[rhs].z;
      });
    std::vector<Point3> selected;
    selected.reserve(required_inliers);
    for (std::size_t index = 0U; index < required_inliers; ++index) {
      selected.push_back(points[order[index]]);
    }
    PlaneFit next;
    if (!FitPlane(selected, &next)) {
      result.status = PlaneFitStatus::kDegenerate;
      return result;
    }
    ++result.iterations;
    const bool converged =
      std::abs(next.a - current.a) <= 1.0e-12 &&
      std::abs(next.b - current.b) <= 1.0e-12 &&
      std::abs(next.c - current.c) <= 1.0e-12;
    current = next;
    if (converged) {
      break;
    }
  }

  const auto group_inliers = [&](const PlaneFit & plane) {
      std::vector<Point3> grouped;
      grouped.reserve(points.size());
      for (const Point3 & point : points) {
        if (std::abs(PlaneResidual(plane, point)) <=
          max_rms + kGeometryEpsilon)
        {
          grouped.push_back(point);
        }
      }
      return grouped;
    };

  std::vector<Point3> inliers = group_inliers(current);
  if (inliers.size() < required_inliers) {
    result.status = PlaneFitStatus::kRmsTooHigh;
    return result;
  }
  PlaneFit final_plane;
  if (!FitPlane(inliers, &final_plane)) {
    result.status = PlaneFitStatus::kDegenerate;
    return result;
  }
  ++result.iterations;
  if (final_plane.rms > max_rms + kGeometryEpsilon) {
    result.status = PlaneFitStatus::kRmsTooHigh;
    return result;
  }

  // Refit only while the actual grouped membership changes. Cardinality alone
  // is never treated as convergence.
  for (std::size_t refit = 0U; refit < kGroupRefits; ++refit) {
    std::vector<Point3> regrouped = group_inliers(final_plane);
    if (regrouped.size() < required_inliers) {
      result.status = PlaneFitStatus::kRmsTooHigh;
      return result;
    }
    if (SameMembership(regrouped, inliers)) {
      if (!InliersMatchPlane(final_plane, inliers, max_rms)) {
        result.status = PlaneFitStatus::kRmsTooHigh;
        return result;
      }
      result.status = PlaneFitStatus::kOk;
      result.plane = final_plane;
      result.inliers = std::move(inliers);
      return result;
    }
    PlaneFit refined;
    if (!FitPlane(regrouped, &refined)) {
      result.status = PlaneFitStatus::kDegenerate;
      return result;
    }
    ++result.iterations;
    if (refined.rms > max_rms + kGeometryEpsilon) {
      result.status = PlaneFitStatus::kRmsTooHigh;
      return result;
    }
    final_plane = refined;
    inliers = std::move(regrouped);
  }

  // The fixed refit budget must end on a plane whose grouped membership and
  // residuals are consistent; otherwise the result is ambiguous, not valid.
  const std::vector<Point3> regrouped = group_inliers(final_plane);
  if (regrouped.size() < required_inliers ||
    !SameMembership(regrouped, inliers) ||
    !InliersMatchPlane(final_plane, inliers, max_rms))
  {
    result.status = PlaneFitStatus::kRmsTooHigh;
    return result;
  }
  result.status = PlaneFitStatus::kOk;
  result.plane = final_plane;
  result.inliers = std::move(inliers);
  return result;
}

std::vector<Interval> ProjectSupportOntoEdge(
  const std::vector<Point3> & points, const EdgeFrame & edge,
  double support_radius, double boundary_band)
{
  std::vector<Interval> intervals;
  if (!std::isfinite(edge.length) || edge.length <= kGeometryEpsilon ||
    !std::isfinite(support_radius) || support_radius < 0.0 ||
    !std::isfinite(boundary_band) || boundary_band < 0.0)
  {
    return intervals;
  }
  intervals.reserve(points.size());
  for (const Point3 & point : points) {
    if (!point.IsFinite()) {
      continue;
    }
    const double relative_x = point.x - edge.origin_x;
    const double relative_y = point.y - edge.origin_y;
    const double normal_distance =
      std::abs(
      relative_x * edge.normal_x + relative_y * edge.normal_y);
    if (normal_distance > boundary_band + kGeometryEpsilon) {
      continue;
    }
    const double tangent =
      relative_x * edge.tangent_x + relative_y * edge.tangent_y;
    if (tangent < -support_radius ||
      tangent > edge.length + support_radius)
    {
      continue;
    }
    // Clip the actual sample footprint to the finite edge. The tangent origin
    // is the first endpoint, so the edge is [0, length] in both orientations.
    const double begin = std::max(0.0, tangent - support_radius);
    const double end = std::min(edge.length, tangent + support_radius);
    if (end - begin <= kGeometryEpsilon) {
      continue;
    }
    intervals.push_back(Interval{begin, end});
  }
  return MergeAndClipIntervals(std::move(intervals), edge.length);
}

std::vector<Interval> ProjectSupportGapOntoEdge(
  const std::vector<Point3> & points, const EdgeFrame & edge,
  double max_sampling_gap)
{
  std::vector<Interval> intervals;
  if (!std::isfinite(edge.length) || edge.length <= kGeometryEpsilon ||
    !std::isfinite(max_sampling_gap) || max_sampling_gap <= 0.0)
  {
    return intervals;
  }
  intervals.reserve(points.size());
  for (const Point3 & point : points) {
    if (!point.IsFinite()) {
      continue;
    }
    const double relative_x = point.x - edge.origin_x;
    const double relative_y = point.y - edge.origin_y;
    const double normal_distance = std::abs(
      relative_x * edge.normal_x + relative_y * edge.normal_y);
    if (normal_distance > max_sampling_gap + kGeometryEpsilon) {
      continue;
    }
    const double half_width = std::sqrt(
      std::max(
        0.0, max_sampling_gap * max_sampling_gap -
        normal_distance * normal_distance));
    if (half_width <= kGeometryEpsilon) {
      continue;
    }
    const double tangent =
      relative_x * edge.tangent_x + relative_y * edge.tangent_y;
    const double begin = std::max(0.0, tangent - half_width);
    const double end = std::min(edge.length, tangent + half_width);
    if (end - begin <= kGeometryEpsilon) {
      continue;
    }
    intervals.push_back(Interval{begin, end});
  }
  return MergeAndClipIntervals(std::move(intervals), edge.length);
}

std::vector<Interval> MergeAndClipIntervals(
  std::vector<Interval> intervals, double length)
{
  std::vector<Interval> merged;
  if (!std::isfinite(length) || length <= kGeometryEpsilon) {
    return merged;
  }
  for (const Interval & interval : intervals) {
    if (!std::isfinite(interval.begin) || !std::isfinite(interval.end)) {
      continue;
    }
    const double begin = std::max(0.0, interval.begin);
    const double end = std::min(length, interval.end);
    if (end - begin <= kGeometryEpsilon) {
      continue;
    }
    merged.push_back(Interval{begin, end});
  }
  std::sort(
    merged.begin(), merged.end(),
    [](const Interval & lhs, const Interval & rhs) {
      return lhs.begin < rhs.begin ||
      (lhs.begin == rhs.begin && lhs.end < rhs.end);
    });
  std::vector<Interval> result;
  for (const Interval & interval : merged) {
    if (result.empty() ||
      interval.begin > result.back().end + kGeometryEpsilon)
    {
      result.push_back(interval);
    } else {
      result.back().end = std::max(result.back().end, interval.end);
    }
  }
  return result;
}

std::vector<Interval> IntersectIntervals(
  const std::vector<Interval> & first,
  const std::vector<Interval> & second)
{
  std::vector<Interval> common;
  std::size_t first_index = 0U;
  std::size_t second_index = 0U;
  while (first_index < first.size() && second_index < second.size()) {
    const Interval & left = first[first_index];
    const Interval & right = second[second_index];
    const double begin = std::max(left.begin, right.begin);
    const double end = std::min(left.end, right.end);
    if (end - begin > kGeometryEpsilon) {
      common.push_back(Interval{begin, end});
    }
    if (left.end < right.end) {
      ++first_index;
    } else if (right.end < left.end) {
      ++second_index;
    } else {
      ++first_index;
      ++second_index;
    }
  }
  return common;
}

double TotalIntervalLength(const std::vector<Interval> & intervals)
{
  double total = 0.0;
  for (const Interval & interval : intervals) {
    total += std::max(0.0, interval.end - interval.begin);
  }
  return total;
}

EdgeEvaluation EvaluateEdgeSupport(
  const std::vector<Interval> & source_intervals,
  const std::vector<Interval> & target_intervals,
  double edge_length, double delta_at_origin, double delta_per_unit)
{
  EdgeEvaluation evaluation;
  if (!std::isfinite(edge_length) ||
    edge_length <= kGeometryEpsilon ||
    !std::isfinite(delta_at_origin) ||
    !std::isfinite(delta_per_unit))
  {
    return evaluation;
  }
  const std::vector<Interval> source =
    MergeAndClipIntervals(source_intervals, edge_length);
  const std::vector<Interval> target =
    MergeAndClipIntervals(target_intervals, edge_length);
  const std::vector<Interval> common = IntersectIntervals(source, target);
  if (common.empty()) {
    return evaluation;
  }
  evaluation.coverage = std::clamp(
    TotalIntervalLength(common) / edge_length, 0.0, 1.0);
  double maximum_delta = 0.0;
  for (const Interval & interval : common) {
    maximum_delta = std::max(
      maximum_delta,
      std::abs(delta_at_origin + delta_per_unit * interval.begin));
    maximum_delta = std::max(
      maximum_delta,
      std::abs(delta_at_origin + delta_per_unit * interval.end));
  }
  evaluation.max_abs_height_delta = maximum_delta;
  return evaluation;
}

}  // namespace offline_geometry
}  // namespace grid_global_planner
