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

#ifndef GRID_GLOBAL_PLANNER__OFFLINE_GEOMETRY_H_
#define GRID_GLOBAL_PLANNER__OFFLINE_GEOMETRY_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "grid_global_planner/terrain_types.h"

namespace grid_global_planner
{
namespace offline_geometry
{

// Bounded deterministic robust fit budget. The implementation performs at most
// this many plane refits per candidate layer; there is no random sampling.
inline constexpr std::size_t kOfflineRobustMaxIterations = 7U;

struct PlaneFit
{
  bool valid{false};
  Point3 centroid{};
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double rms{0.0};
  Vector3 normal{0.0, 0.0, 1.0};
};

enum class PlaneFitStatus : std::uint8_t
{
  kOk = 0,
  kInsufficientPoints,
  kDegenerate,
  kRmsTooHigh,
};

// Returns a stable diagnostic spelling for a reconstruction fit status.
const char * PlaneFitStatusToString(PlaneFitStatus status) noexcept;

// Ordinary least-squares height plane z = a*x + b*y + c. Requires at least
// three finite, non-collinear points and returns false for a singular system.
bool FitPlane(const std::vector<Point3> & points, PlaneFit * plane);

struct RobustPlaneFit
{
  PlaneFitStatus status{PlaneFitStatus::kInsufficientPoints};
  PlaneFit plane;
  std::vector<Point3> inliers;
  std::size_t iterations{0};

  bool valid() const noexcept
  {
    return status == PlaneFitStatus::kOk && plane.valid &&
           !inliers.empty();
  }
};

// Bounded deterministic trimmed-least-squares fallback. It starts from all
// measured points, deterministically keeps the best two thirds by absolute
// residual for a fixed small number of refits, then groups and refits only
// points whose residual is within max_rms. No random subset is sampled and no
// threshold is widened. The returned inliers are the exact grouped points that
// must be revalidated for hull, sampling gap and minimum-count evidence.
RobustPlaneFit FitRobustPlane(
  const std::vector<Point3> & points, std::size_t min_points,
  double max_rms);

// One straight shared edge with tangent origin at its first endpoint. Tangent
// coordinates are therefore in [0, length]; normal coordinates are measured
// from the edge line. All vectors are unit vectors in the map frame.
struct EdgeFrame
{
  double origin_x{0.0};
  double origin_y{0.0};
  double tangent_x{1.0};
  double tangent_y{0.0};
  double normal_x{0.0};
  double normal_y{1.0};
  double length{0.0};
};

// One measured-support span along the edge tangent, clipped to [0, length].
struct Interval
{
  double begin{0.0};
  double end{0.0};
};

// Exact edge evidence. Coverage is the positive-length common support divided
// by the real edge length. The height delta is the maximum absolute fitted
// plane-height delta at every common-interval endpoint.
struct EdgeEvaluation
{
  double coverage{0.0};
  double max_abs_height_delta{
    std::numeric_limits<double>::quiet_NaN()};
};

// Projects real same-layer support points onto one shared edge. A candidate
// must lie within boundary_band of the edge and within one support_radius of
// the finite edge segment. Its tangent sample footprint is clipped to the real
// edge and merged; endpoint-only zero-measure contacts are discarded.
std::vector<Interval> ProjectSupportOntoEdge(
  const std::vector<Point3> & points, const EdgeFrame & edge,
  double support_radius, double boundary_band);

// Projects the existing sampling-gap support footprint of real points onto
// one edge. For a point at perpendicular distance d <= gap, the line
// intersection is [tangent-half, tangent+half] with
// half=sqrt(gap^2-d^2), clipped to the finite edge and merged. This is not a
// dilated support domain; callers intersect it with the measured inlier hull.
std::vector<Interval> ProjectSupportGapOntoEdge(
  const std::vector<Point3> & points, const EdgeFrame & edge,
  double max_sampling_gap);

// Clips every interval to [0, length], drops zero-measure spans, sorts, and
// merges overlapping/touching spans. The result is disjoint and increasing.
std::vector<Interval> MergeAndClipIntervals(
  std::vector<Interval> intervals, double length);

// Exact positive-length intersection of two disjoint, increasing interval
// lists. No sampling and no endpoint-only intersections are returned.
std::vector<Interval> IntersectIntervals(
  const std::vector<Interval> & first,
  const std::vector<Interval> & second);

// Sum of interval lengths. Inputs must already be disjoint.
double TotalIntervalLength(const std::vector<Interval> & intervals);

// Evaluates exact common support and affine height continuity. delta_at_origin
// and delta_per_unit describe (target_plane - source_plane) along the edge
// tangent. With no positive common support, coverage is zero and the height
// delta is NaN; no unsupported point is allowed to set a step.
EdgeEvaluation EvaluateEdgeSupport(
  const std::vector<Interval> & source_intervals,
  const std::vector<Interval> & target_intervals,
  double edge_length, double delta_at_origin, double delta_per_unit);

}  // namespace offline_geometry
}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__OFFLINE_GEOMETRY_H_
