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

#include "dddmr_grid_global_planner/offline_static_map.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <new>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/distance_field.h"
#include "dddmr_grid_global_planner/ground_surface_reconstruction.h"
#include "dddmr_grid_global_planner/obstacle_tile.h"
#include "dddmr_grid_global_planner/offline_geometry.h"
#include "dddmr_grid_global_planner/offline_surface_evidence.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
// Bound per-patch fit work independently of user-supplied radius increments.
constexpr std::size_t kMaxAdaptiveExpansionAttempts = 16U;
// Bounded ground-mode context radius. It is never larger than the existing
// neighborhood + max_sampling_gap contract and never a new global range.
constexpr double kMaxGroundContextRadius = 0.65;
constexpr double kMinGroundContextAngularCoverage = 0.5;

// A slope can span more than the step limit inside one cell. Detect a
// sufficiently observed height gap in the context instead of treating the
// total own-cell Z span as a physical step. Single isolated samples do not
// establish both sides of a discontinuity; the unchanged residual gates still
// check every own-cell observation against the proposed model.
bool ContextHasObservedHeightGap(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices, const Point3 & centre,
  double max_step_height_m, double max_rms_m)
{
  constexpr std::size_t kMinimumClusterPoints = 3U;
  if (indices.size() < 2U * kMinimumClusterPoints) {
    return false;
  }
  std::vector<Point3> samples;
  samples.reserve(indices.size());
  for (const std::uint32_t index : indices) {
    if (index >= points.size() || !points[index].IsFinite()) {
      return true;
    }
    samples.push_back(points[index]);
  }
  std::sort(
    samples.begin(), samples.end(),
    [](const Point3 & first, const Point3 & second) {return first.z < second.z;});
  std::size_t largest_split = 0U;
  double largest_gap = max_step_height_m + kEpsilon;
  for (std::size_t split = kMinimumClusterPoints;
    split + kMinimumClusterPoints <= samples.size(); ++split)
  {
    const double gap = samples[split].z - samples[split - 1U].z;
    if (gap > largest_gap) {
      largest_gap = gap;
      largest_split = split;
    }
  }
  if (largest_split == 0U) {
    return false;
  }
  // Sparse samples on a genuine slope can also have large Z gaps. Require
  // independently supported planes that disagree at the SAME XY position;
  // a single continuous inclined plane predicts equal heights on both sides.
  const auto middle = samples.begin() + static_cast<std::ptrdiff_t>(largest_split);
  const std::vector<Point3> first(samples.begin(), middle);
  const std::vector<Point3> second(middle, samples.end());
  offline_geometry::PlaneFit first_fit;
  offline_geometry::PlaneFit second_fit;
  if (!offline_geometry::FitPlane(first, &first_fit) ||
    !offline_geometry::FitPlane(second, &second_fit) ||
    first_fit.rms > max_rms_m || second_fit.rms > max_rms_m)
  {
    return false;
  }
  const double first_height = first_fit.centroid.z +
    first_fit.a * (centre.x - first_fit.centroid.x) +
    first_fit.b * (centre.y - first_fit.centroid.y);
  const double second_height = second_fit.centroid.z +
    second_fit.a * (centre.x - second_fit.centroid.x) +
    second_fit.b * (centre.y - second_fit.centroid.y);
  return !std::isfinite(first_height) || !std::isfinite(second_height) ||
         std::abs(first_height - second_height) > max_step_height_m + kEpsilon;
}

double DurationToMs(std::chrono::steady_clock::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

// One XY point used by the local convex-hull support-domain test.
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

// Monotone-chain convex hull in deterministic lexicographic order. The hull is
// the actual support domain of the measured neighbourhood; no point is added.
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
  std::size_t size = 0;
  for (const Point2 & point : points) {
    while (size >= 2U &&
      Cross2(hull[size - 2U], hull[size - 1U], point) <= 0.0)
    {
      --size;
    }
    hull[size++] = point;
  }
  const std::size_t lower_size = size;
  // The lower chain already ends at the last sorted point. Walk the remaining
  // points backwards, including the first point as the closing vertex. The
  // final resize removes that duplicate closure, not a real upper-chain vertex.
  // Starting at the last point again and stopping at index 1 drops a genuine
  // hull corner, which falsely rejects measured interior terrain cells.
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

// Inclusive point-in-convex-polygon test with a metric tolerance. A point on
// the hull boundary is inside the measured support domain.
bool PointInHull(
  const std::vector<Point2> & hull, double x, double y,
  double tolerance) noexcept
{
  if (hull.empty()) {
    return false;
  }
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

// Plane fitting and bounded robust refinement live in offline_geometry so their
// residual grouping, determinism and budget are directly unit-tested.

// Uses the shared reconstructed surface query when the node carries welded
// corners, so boundary continuity and obstacle association consume the same
// geometry as planning/rendering instead of the raw fitted plane.
double PlaneHeightAt(const TerrainNode & node, double x, double y) noexcept
{
  if (!node.plane.valid || std::abs(node.plane.normal.z) < kEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double z = TerrainSurfaceHeight(node, x, y);
  return std::isfinite(z) ?
         z : std::numeric_limits<double>::quiet_NaN();
}

// Shared edge or corner between two adjacent fine cells. An edge has a real
// tangent length; a diagonal pair touches at one point and is evaluated at that
// corner rather than inventing a shared edge.
struct SharedBoundary
{
  Point2 first{0.0, 0.0};
  Point2 second{0.0, 0.0};
  double tangent_x{0.0};
  double tangent_y{0.0};
  double normal_x{0.0};
  double normal_y{0.0};
  double length{0.0};
  bool corner{true};
};

SharedBoundary SharedBoundaryBetween(
  const TerrainNode & source, const TerrainNode & target,
  double resolution) noexcept
{
  SharedBoundary boundary;
  const std::int64_t delta_x = target.cell.x - source.cell.x;
  const std::int64_t delta_y = target.cell.y - source.cell.y;
  const double half = 0.5 * resolution;
  const double boundary_x =
    0.5 * (source.representative.x + target.representative.x);
  const double boundary_y =
    0.5 * (source.representative.y + target.representative.y);
  if (delta_x != 0 && delta_y != 0) {
    boundary.first = Point2{boundary_x, boundary_y};
    boundary.second = boundary.first;
    const double norm = std::hypot(
      static_cast<double>(delta_x), static_cast<double>(delta_y));
    boundary.tangent_x = static_cast<double>(delta_x) / norm;
    boundary.tangent_y = static_cast<double>(delta_y) / norm;
    boundary.normal_x = boundary.tangent_x;
    boundary.normal_y = boundary.tangent_y;
    boundary.corner = true;
    return boundary;
  }
  boundary.corner = false;
  if (delta_x != 0) {
    boundary.first = Point2{boundary_x, source.representative.y - half};
    boundary.second = Point2{boundary_x, source.representative.y + half};
    boundary.tangent_x = 0.0;
    boundary.tangent_y = 1.0;
    boundary.normal_x = delta_x > 0 ? 1.0 : -1.0;
    boundary.normal_y = 0.0;
  } else {
    boundary.first = Point2{source.representative.x - half, boundary_y};
    boundary.second = Point2{source.representative.x + half, boundary_y};
    boundary.tangent_x = 1.0;
    boundary.tangent_y = 0.0;
    boundary.normal_x = 0.0;
    boundary.normal_y = delta_y > 0 ? 1.0 : -1.0;
  }
  boundary.length = std::hypot(
    boundary.second.x - boundary.first.x,
    boundary.second.y - boundary.first.y);
  if (!std::isfinite(boundary.length) || boundary.length <= kEpsilon) {
    // Degenerate edge geometry has no measurable shared boundary.
    boundary.corner = true;
    boundary.first = Point2{boundary_x, boundary_y};
    boundary.second = boundary.first;
    boundary.length = 0.0;
  }
  return boundary;
}

// Intersects a measured inlier convex hull with the shared edge. The hull is
// the actual same-layer support domain; this removes radial sample-phase
// artifacts without inventing points or expanding the domain. A positive-length
// result is required, so a corner-only contact contributes no edge coverage.
std::vector<offline_geometry::Interval> ClipHullToEdge(
  const std::vector<Point2> & hull, const SharedBoundary & boundary)
{
  std::vector<offline_geometry::Interval> intervals;
  if (hull.size() < 3U || boundary.length <= kEpsilon) {
    return intervals;
  }
  double low = 0.0;
  double high = boundary.length;
  for (std::size_t index = 0; index < hull.size(); ++index) {
    const Point2 & first = hull[index];
    const Point2 & second = hull[(index + 1U) % hull.size()];
    const double edge_x = second.x - first.x;
    const double edge_y = second.y - first.y;
    if (!std::isfinite(edge_x) || !std::isfinite(edge_y)) {
      return {};
    }
    // The hull is counter-clockwise, so the inside half-plane is cross >= 0.
    const double start =
      edge_x * (boundary.first.y - first.y) -
      edge_y * (boundary.first.x - first.x);
    const double slope =
      edge_x * boundary.tangent_y - edge_y * boundary.tangent_x;
    if (std::abs(slope) <= kEpsilon) {
      if (start < -kEpsilon) {
        return {};
      }
      continue;
    }
    const double tangent = -start / slope;
    if (slope > 0.0) {
      low = std::max(low, tangent);
    } else {
      high = std::min(high, tangent);
    }
    if (high - low <= kEpsilon) {
      return {};
    }
  }
  low = std::max(0.0, low);
  high = std::min(boundary.length, high);
  if (high - low <= kEpsilon) {
    return {};
  }
  intervals.push_back(offline_geometry::Interval{low, high});
  return intervals;
}

// Continuity is evaluated only where both same-layer measured support regions
// reach the shared geometry. Height differences use fitted plane heights at
// that shared boundary; no centre-to-centre height stands in for a step.
struct BoundaryContinuity
{
  bool supported{false};
  double coverage{0.0};
  double step_height{std::numeric_limits<double>::quiet_NaN()};
};

BoundaryContinuity EvaluateCornerContinuity(
  const TerrainNode & source, const TerrainNode & target,
  const SharedBoundary & boundary,
  const std::vector<Point2> & source_hull,
  const std::vector<Point2> & target_hull,
  const OfflineStaticMapOptions & options)
{
  BoundaryContinuity continuity;
  const double point_tolerance = 1.0e-9;
  if (!PointInHull(
      source_hull, boundary.first.x, boundary.first.y, point_tolerance) ||
    !PointInHull(
      target_hull, boundary.first.x, boundary.first.y, point_tolerance))
  {
    return continuity;
  }
  const auto nearest_support = [&](const TerrainNode & node) {
      double nearest = std::numeric_limits<double>::infinity();
      for (const Point3 & point : node.support.points) {
        nearest = std::min(
          nearest, std::hypot(
            point.x - boundary.first.x,
            point.y - boundary.first.y));
      }
      return nearest;
    };
  if (nearest_support(source) > options.max_sampling_gap_m + kEpsilon ||
    nearest_support(target) > options.max_sampling_gap_m + kEpsilon)
  {
    return continuity;
  }
  const double source_height = PlaneHeightAt(
    source, boundary.first.x, boundary.first.y);
  const double target_height = PlaneHeightAt(
    target, boundary.first.x, boundary.first.y);
  if (!std::isfinite(source_height) || !std::isfinite(target_height)) {
    return continuity;
  }
  continuity.supported = true;
  continuity.coverage = 1.0;
  continuity.step_height = std::abs(target_height - source_height);
  return continuity;
}

BoundaryContinuity EvaluateEdgeContinuity(
  const TerrainNode & source, const TerrainNode & target,
  const SharedBoundary & boundary,
  const std::vector<Point2> & source_hull,
  const std::vector<Point2> & target_hull,
  const OfflineStaticMapOptions & options)
{
  BoundaryContinuity continuity;
  const offline_geometry::EdgeFrame frame{
    boundary.first.x, boundary.first.y,
    boundary.tangent_x, boundary.tangent_y,
    boundary.normal_x, boundary.normal_y,
    boundary.length};
  const std::vector<offline_geometry::Interval> source_support =
    offline_geometry::IntersectIntervals(
    ClipHullToEdge(source_hull, boundary),
    offline_geometry::ProjectSupportGapOntoEdge(
      source.support.points, frame, options.max_sampling_gap_m));
  const std::vector<offline_geometry::Interval> target_support =
    offline_geometry::IntersectIntervals(
    ClipHullToEdge(target_hull, boundary),
    offline_geometry::ProjectSupportGapOntoEdge(
      target.support.points, frame, options.max_sampling_gap_m));
  const double source_first = PlaneHeightAt(
    source, boundary.first.x, boundary.first.y);
  const double target_first = PlaneHeightAt(
    target, boundary.first.x, boundary.first.y);
  const double source_second = PlaneHeightAt(
    source, boundary.second.x, boundary.second.y);
  const double target_second = PlaneHeightAt(
    target, boundary.second.x, boundary.second.y);
  if (!std::isfinite(source_first) || !std::isfinite(target_first) ||
    !std::isfinite(source_second) || !std::isfinite(target_second))
  {
    return continuity;
  }
  const double delta_at_origin = target_first - source_first;
  const double delta_at_end = target_second - source_second;
  const double delta_per_unit =
    (delta_at_end - delta_at_origin) / boundary.length;
  const offline_geometry::EdgeEvaluation evaluation =
    offline_geometry::EvaluateEdgeSupport(
    source_support, target_support, boundary.length,
    delta_at_origin, delta_per_unit);
  continuity.coverage = evaluation.coverage;
  if (std::isfinite(evaluation.max_abs_height_delta)) {
    continuity.supported = true;
    continuity.step_height = evaluation.max_abs_height_delta;
  }
  return continuity;
}

// Reconstructed same-layer cells carry exact shared corners. Their continuous
// surface, not a local raw-edge sample density, proves the link: the stored
// regional fit already supplies confidence and the corners weld only when the
// real step/normal bounds are satisfied. Real floor separation is preserved
// because the boundary height is still evaluated from the exact surfaces.
BoundaryContinuity EvaluateReconstructedContinuity(
  const TerrainNode & source, const TerrainNode & target,
  const SharedBoundary & boundary)
{
  BoundaryContinuity continuity;
  const double source_first = PlaneHeightAt(
    source, boundary.first.x, boundary.first.y);
  const double target_first = PlaneHeightAt(
    target, boundary.first.x, boundary.first.y);
  if (!std::isfinite(source_first) || !std::isfinite(target_first)) {
    return continuity;
  }
  double step = std::abs(target_first - source_first);
  if (!boundary.corner) {
    const double source_second = PlaneHeightAt(
      source, boundary.second.x, boundary.second.y);
    const double target_second = PlaneHeightAt(
      target, boundary.second.x, boundary.second.y);
    if (!std::isfinite(source_second) || !std::isfinite(target_second)) {
      return continuity;
    }
    step = std::max(step, std::abs(target_second - source_second));
  }
  continuity.supported = true;
  continuity.step_height = step;
  return continuity;
}

BoundaryContinuity EvaluateBoundaryContinuity(
  const TerrainNode & source, const TerrainNode & target,
  const SharedBoundary & boundary,
  const std::vector<Point2> & source_hull,
  const std::vector<Point2> & target_hull,
  const OfflineStaticMapOptions & options)
{
  if (boundary.corner) {
    return EvaluateCornerContinuity(
      source, target, boundary, source_hull, target_hull, options);
  }
  return EvaluateEdgeContinuity(
    source, target, boundary, source_hull, target_hull, options);
}

// Splits the actual candidate indices into measured Z layers before any fit.
// Sorting by (z, source ref) makes equal-Z membership independent of input
// order; every returned layer ref is ascending so evidence is canonical.
std::vector<std::vector<std::uint32_t>> SplitIndexLayers(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  double gap_threshold)
{
  std::vector<std::uint32_t> ordered = indices;
  std::sort(
    ordered.begin(), ordered.end(),
    [&points](std::uint32_t lhs, std::uint32_t rhs) {
      if (points[lhs].z != points[rhs].z) {
        return points[lhs].z < points[rhs].z;
      }
      return lhs < rhs;
    });
  std::vector<std::vector<std::uint32_t>> layers;
  for (const std::uint32_t index : ordered) {
    if (layers.empty() ||
      points[index].z - points[layers.back().back()].z > gap_threshold)
    {
      layers.push_back({});
    }
    layers.back().push_back(index);
  }
  for (std::vector<std::uint32_t> & layer : layers) {
    std::sort(layer.begin(), layer.end());
  }
  return layers;
}

// Only evidence insufficiency may take the one bounded same-floor retry. RMS,
// inlier, degeneracy, depression and slope failures are quality/conflict
// decisions and must never be diluted by a larger neighbourhood.
bool IsEvidenceInsufficientReason(
  MeasurementRejectionReason reason) noexcept
{
  return reason == MeasurementRejectionReason::kInsufficientPoints ||
         reason == MeasurementRejectionReason::kSamplingGap ||
         reason == MeasurementRejectionReason::kUnsupportedCenter;
}

// Selects one expanded layer from actual canonical point refs. The selected
// layer must contain every base ref of the current layer, no ref from another
// base layer in this cell, and its actual Z span may exceed the current base
// span by at most max_step_height_m. On failure `selected` still receives the
// real attempted layer (or the complete expanded ref set when no layer contains
// the base), so annulus evidence is never silently dropped.
bool SelectExpandedSameFloorLayer(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & base_indices,
  const std::vector<std::vector<std::uint32_t>> & base_layers,
  std::size_t base_layer_index,
  const std::vector<std::uint32_t> & expanded_indices,
  const OfflineStaticMapOptions & options,
  std::vector<std::uint32_t> * selected,
  MeasurementRejectionReason * reason)
{
  if (selected == nullptr || reason == nullptr || points.empty() ||
    expanded_indices.empty() || base_indices.empty() ||
    base_layer_index >= base_layers.size())
  {
    return false;
  }
  *selected = expanded_indices;
  std::sort(selected->begin(), selected->end());
  selected->erase(
    std::unique(selected->begin(), selected->end()), selected->end());
  *reason = MeasurementRejectionReason::kExpansionInvalid;

  std::vector<std::uint32_t> base_refs = base_indices;
  std::sort(base_refs.begin(), base_refs.end());
  std::vector<std::uint32_t> expected_base = base_layers[base_layer_index];
  std::sort(expected_base.begin(), expected_base.end());
  if (base_refs != expected_base) {
    return false;
  }
  for (const std::uint32_t index : *selected) {
    if (static_cast<std::size_t>(index) >= points.size()) {
      return false;
    }
  }

  const std::vector<std::vector<std::uint32_t>> expanded_layers =
    SplitIndexLayers(points, *selected, options.layer_gap_threshold_m);
  const std::vector<std::uint32_t> * candidate = nullptr;
  for (const std::vector<std::uint32_t> & layer : expanded_layers) {
    if (std::includes(
        layer.begin(), layer.end(), base_refs.begin(), base_refs.end()))
    {
      candidate = &layer;
      break;
    }
  }
  if (candidate == nullptr) {
    return false;
  }
  *selected = *candidate;

  std::vector<std::uint32_t> other_base_refs;
  for (std::size_t layer_index = 0; layer_index < base_layers.size();
    ++layer_index)
  {
    if (layer_index == base_layer_index) {
      continue;
    }
    other_base_refs.insert(
      other_base_refs.end(), base_layers[layer_index].begin(),
      base_layers[layer_index].end());
  }
  std::sort(other_base_refs.begin(), other_base_refs.end());
  for (const std::uint32_t index : *selected) {
    if (std::binary_search(
        other_base_refs.begin(), other_base_refs.end(), index))
    {
      return false;
    }
  }

  auto z_bounds = [&points](
    const std::vector<std::uint32_t> & indices, double * minimum,
    double * maximum) {
      *minimum = points[indices.front()].z;
      *maximum = *minimum;
      for (const std::uint32_t index : indices) {
        *minimum = std::min(*minimum, points[index].z);
        *maximum = std::max(*maximum, points[index].z);
      }
    };
  double base_min = 0.0;
  double base_max = 0.0;
  double selected_min = 0.0;
  double selected_max = 0.0;
  z_bounds(base_refs, &base_min, &base_max);
  z_bounds(*selected, &selected_min, &selected_max);
  if (selected_max - selected_min >
    (base_max - base_min) + options.max_step_height_m + kEpsilon)
  {
    return false;
  }
  *reason = MeasurementRejectionReason::kNone;
  return true;
}

// Height residual of a hint point against one fitted local attempt plane. This
// keeps a GroundMap hint associated with the measured height, not just its XY
// cell, so stacked deck/box-top components cannot all be seeded by one hint.
double FitPlaneHeightResidual(
  const MeasurementFitAttempt & fit, const Point3 & point) noexcept
{
  if (!fit.plane_valid || !std::isfinite(fit.normal.z) ||
    !(std::fabs(fit.normal.z) > 0.0))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double numerator =
    fit.normal.x * (point.x - fit.centroid.x) +
    fit.normal.y * (point.y - fit.centroid.y) +
    fit.normal.z * (point.z - fit.centroid.z);
  const double residual = numerator / fit.normal.z;
  return std::isfinite(residual) ?
         residual : std::numeric_limits<double>::quiet_NaN();
}

// Fraction of angular sectors containing at least one selected same-floor
// sample. Each sector contributes at most once, so dense one-sided samples
// cannot satisfy spatial coverage by point count.
double SelectedSectorCoverage(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center) noexcept
{
  std::array<bool, kSupportSectorCount> occupied{};
  for (const std::uint32_t index : indices) {
    if (static_cast<std::size_t>(index) >= points.size()) {
      continue;
    }
    const Point3 & point = points[index];
    const double dx = point.x - center.x;
    const double dy = point.y - center.y;
    if (!std::isfinite(dx) || !std::isfinite(dy) ||
      std::hypot(dx, dy) <= kEpsilon)
    {
      continue;
    }
    double angle = std::atan2(dy, dx);
    if (angle < 0.0) {
      angle += 2.0 * 3.14159265358979323846;
    }
    const std::size_t sector = std::min(
      kSupportSectorCount - 1U,
      static_cast<std::size_t>(
        std::floor(
          angle / (2.0 * 3.14159265358979323846) *
          static_cast<double>(kSupportSectorCount))));
    occupied[sector] = true;
  }
  std::size_t count = 0U;
  for (const bool value : occupied) {
    count += value ? 1U : 0U;
  }
  return static_cast<double>(count) /
         static_cast<double>(kSupportSectorCount);
}

// Copies one actual evaluator attempt into the persistent fit record. The
// caller supplies the concrete collection radius; no plane or residual value is
// invented when the attempt has no candidate plane.
void CopyFitAttempt(
  const SurfaceAttempt & attempt, double radius,
  MeasurementFitAttempt * fit)
{
  fit->radius_m = radius;
  fit->point_indices = attempt.point_indices;
  fit->nearest_support_xy_m = attempt.nearest_support_xy_m;
  fit->support_gate_ok = attempt.support_gate == SurfaceSupportGate::kOk;
  fit->center_in_support_hull = attempt.center_in_support_hull;
  if (attempt.plane.valid) {
    fit->plane_valid = true;
    fit->inlier_indices = attempt.inlier_indices;
    fit->centroid = attempt.plane.centroid;
    fit->normal = attempt.plane.normal;
    fit->rms_residual = attempt.plane.rms;
    fit->conflict_sample_count = attempt.point_indices.size();
    fit->min_signed_residual = attempt.min_signed_residual;
    fit->max_signed_residual = attempt.max_signed_residual;
    fit->max_depression_m = attempt.max_depression_m;
    fit->max_raised_m = attempt.max_raised_m;
    fit->raised_conflict_sample_count =
      attempt.raised_conflict_sample_count;
    fit->conflict_point = attempt.conflict_point;
  } else {
    fit->plane_valid = false;
    fit->inlier_indices.clear();
    fit->centroid = Point3{};
    fit->normal = Vector3{0.0, 0.0, 1.0};
    fit->rms_residual = std::numeric_limits<double>::quiet_NaN();
    fit->conflict_sample_count = 0U;
    fit->min_signed_residual = std::numeric_limits<double>::quiet_NaN();
    fit->max_signed_residual = std::numeric_limits<double>::quiet_NaN();
    fit->max_depression_m = 0.0;
    fit->max_raised_m = 0.0;
    fit->raised_conflict_sample_count = 0U;
    fit->conflict_point = Point3{};
  }
}

// Recomputes own-cell residuals for the selected final plane. The patch own
// refs stay the immutable base-refs subset; an expanded fit must not replace
// them with ring-only samples.
bool ComputeOwnCellResiduals(
  const SurfaceAttempt & selected_attempt,
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & own_indices,
  double * minimum, double * maximum)
{
  if (!selected_attempt.plane.valid || own_indices.empty() ||
    !(std::fabs(selected_attempt.plane.normal.z) > 0.0))
  {
    return false;
  }
  const Point3 & centroid = selected_attempt.plane.centroid;
  const Vector3 & normal = selected_attempt.plane.normal;
  bool first = true;
  for (const std::uint32_t index : own_indices) {
    const Point3 & point = points[index];
    const double numerator =
      normal.x * (point.x - centroid.x) +
      normal.y * (point.y - centroid.y) +
      normal.z * (point.z - centroid.z);
    const double residual = numerator / normal.z;
    if (!std::isfinite(residual)) {
      return false;
    }
    if (first) {
      *minimum = residual;
      *maximum = residual;
      first = false;
    } else {
      *minimum = std::min(*minimum, residual);
      *maximum = std::max(*maximum, residual);
    }
  }
  return !first;
}

// Checked ref-budget accumulation. `cap == 0` is rejected by option validation,
// so subtraction is always well-defined here.
bool AddRefBudget(
  std::size_t * total, std::size_t count, std::size_t cap) noexcept
{
  if (total == nullptr || *total > cap || count > cap - *total) {
    return false;
  }
  *total += count;
  return true;
}

// Residual statistics of an actual regional sample set against its own fitted
// plane. Used by the continuous reconstruction so persisted conflict fields are
// recomputed from real refs instead of trusted from the module/attempt.
struct ContinuousFitStats
{
  double rms {std::numeric_limits<double>::quiet_NaN()};
  double minimum {std::numeric_limits<double>::quiet_NaN()};
  double maximum {std::numeric_limits<double>::quiet_NaN()};
  double max_depression {0.0};
  double max_raised {0.0};
  std::uint32_t conflict_index {0U};
  bool valid {false};
};

bool ComputeContinuousFitStats(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & refs,
  const Point3 & centroid, const Vector3 & normal,
  ContinuousFitStats * stats)
{
  if (stats == nullptr || refs.empty() ||
    !std::isfinite(normal.z) || std::fabs(normal.z) <= kEpsilon)
  {
    return false;
  }
  double squared = 0.0;
  bool first = true;
  double best_extremum = 0.0;
  for (const std::uint32_t index : refs) {
    const Point3 & point = points[index];
    const double residual =
      (normal.x * (point.x - centroid.x) +
      normal.y * (point.y - centroid.y) +
      normal.z * (point.z - centroid.z)) / normal.z;
    if (!std::isfinite(residual)) {
      return false;
    }
    squared += residual * residual;
    if (first) {
      stats->minimum = residual;
      stats->maximum = residual;
      stats->conflict_index = index;
      best_extremum = std::fabs(residual);
      first = false;
    } else {
      stats->minimum = std::min(stats->minimum, residual);
      stats->maximum = std::max(stats->maximum, residual);
      if (std::fabs(residual) > best_extremum) {
        best_extremum = std::fabs(residual);
        stats->conflict_index = index;
      }
    }
  }
  if (!first) {
    stats->rms = std::sqrt(
      squared / static_cast<double>(refs.size()));
    stats->max_depression = std::max(0.0, -stats->minimum);
    stats->max_raised = std::max(0.0, stats->maximum);
    stats->valid = true;
  }
  return stats->valid;
}

// Final planning plane through three exact shared corners. Corner order is
// (x0,y0),(x1,y0),(x1,y1),(x0,y1); c0,c1,c3 span the cell square.
bool ContinuousPlaneFromCorners(
  const std::array<Point3, 4> & corners, const Point3 & center,
  Point3 * centroid, Vector3 * normal)
{
  const double width = corners[1].x - corners[0].x;
  const double height = corners[3].y - corners[0].y;
  if (!(width > kEpsilon) || !(height > kEpsilon)) {return false;}
  // Average the gradients of the renderer's two triangles, and anchor the
  // tangent plane on their shared diagonal at the actual cell-centre height.
  const double a = ((corners[1].z - corners[0].z) +
    (corners[2].z - corners[3].z)) / (2.0 * width);
  const double b = ((corners[3].z - corners[0].z) +
    (corners[2].z - corners[1].z)) / (2.0 * height);
  Vector3 fitted{-a, -b, 1.0};
  const double length = fitted.Norm();
  if (!std::isfinite(length) || length <= kEpsilon) {
    return false;
  }
  fitted = Vector3{fitted.x / length, fitted.y / length, fitted.z / length};
  if (fitted.z < 0.0) {
    fitted = Vector3{-fitted.x, -fitted.y, -fitted.z};
  }
  if (!std::isfinite(fitted.z) || fitted.z <= kEpsilon) {
    return false;
  }
  const double z = 0.5 * (corners[0].z + corners[2].z);
  if (!std::isfinite(z)) {
    return false;
  }
  *centroid = Point3{center.x, center.y, z};
  *normal = fitted;
  return true;
}

// Generated raised evidence keeps the supported layer that actually measured
// the positive residual. Explicit obstacle points still use nearest-surface
// association; this provenance is only for measured generated conflicts.
struct RaisedEvidence
{
  std::uint32_t point_index{0U};
  TerrainNodeId source_node{kInvalidTerrainNodeId};
  // Actual candidate plane that produced the positive witness. This survives a
  // rejected source patch, so association never depends on a planning node.
  bool source_plane_valid{false};
  Point3 source_centroid{};
  Vector3 source_normal{0.0, 0.0, 1.0};
  double source_ground_z{std::numeric_limits<double>::quiet_NaN()};
};

bool RaisedEvidenceLess(
  const RaisedEvidence & lhs, const RaisedEvidence & rhs) noexcept
{
  if (lhs.point_index != rhs.point_index) {
    return lhs.point_index < rhs.point_index;
  }
  if (lhs.source_node != rhs.source_node) {
    return lhs.source_node < rhs.source_node;
  }
  if (lhs.source_ground_z != rhs.source_ground_z) {
    return lhs.source_ground_z < rhs.source_ground_z;
  }
  if (lhs.source_centroid.x != rhs.source_centroid.x) {
    return lhs.source_centroid.x < rhs.source_centroid.x;
  }
  if (lhs.source_centroid.y != rhs.source_centroid.y) {
    return lhs.source_centroid.y < rhs.source_centroid.y;
  }
  return lhs.source_centroid.z < rhs.source_centroid.z;
}

bool SameRaisedEvidence(
  const RaisedEvidence & lhs, const RaisedEvidence & rhs) noexcept
{
  return lhs.point_index == rhs.point_index &&
         lhs.source_node == rhs.source_node &&
         lhs.source_ground_z == rhs.source_ground_z &&
         lhs.source_centroid.x == rhs.source_centroid.x &&
         lhs.source_centroid.y == rhs.source_centroid.y &&
         lhs.source_centroid.z == rhs.source_centroid.z;
}

// Computes the actual candidate plane's ground height at the measured point.
// No interpolation or covariance is invented; failure means the witness has no
// usable source plane and must remain unknown rather than spread occupancy.
bool RaisedSourceGroundZ(
  const offline_geometry::PlaneFit & plane, const Point3 & point,
  double * ground_z) noexcept
{
  if (ground_z == nullptr || !plane.valid ||
    !std::isfinite(plane.normal.z) ||
    !(std::fabs(plane.normal.z) > 0.0))
  {
    return false;
  }
  const double value = plane.centroid.z -
    (plane.normal.x * (point.x - plane.centroid.x) +
    plane.normal.y * (point.y - plane.centroid.y)) / plane.normal.z;
  if (!std::isfinite(value)) {
    return false;
  }
  *ground_z = value;
  return true;
}

// Canonical finite ground table. Exact duplicate coordinates are collapsed once;
// this is deduplication of identical measured samples, not interpolation or
// invented data.
void SortAndDeduplicatePoints(std::vector<Point3> * points)
{
  std::sort(
    points->begin(), points->end(),
    [](const Point3 & lhs, const Point3 & rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      if (lhs.y != rhs.y) {
        return lhs.y < rhs.y;
      }
      return lhs.z < rhs.z;
    });
  points->erase(
    std::unique(
      points->begin(), points->end(),
      [](const Point3 & lhs, const Point3 & rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
      }),
    points->end());
}

void HashU64(std::uint64_t * hash, std::uint64_t value)
{
  *hash ^= value + 0x9e3779b97f4a7c15ULL + (*hash << 6U) + (*hash >> 2U);
}

void HashSize(std::uint64_t * hash, std::size_t value)
{
  HashU64(hash, static_cast<std::uint64_t>(value));
}

void HashBool(std::uint64_t * hash, bool value)
{
  HashU64(hash, value ? 1U : 0U);
}

// Checked candidate span generation. Cell coordinates are already bounded by
// FineCellOf, but halo arithmetic is still checked explicitly before use.
bool CheckedSpan(
  std::int64_t minimum, std::int64_t maximum,
  std::int64_t halo, std::int64_t * span) noexcept
{
  if (span == nullptr || maximum < minimum || halo < 0) {
    return false;
  }
  if (minimum < 0 && maximum > std::numeric_limits<std::int64_t>::max() +
    minimum)
  {
    return false;
  }
  const std::int64_t base = maximum - minimum;
  if (base > std::numeric_limits<std::int64_t>::max() - 2 * halo - 1) {
    return false;
  }
  *span = base + 2 * halo + 1;
  return *span > 0;
}

// Records at most one bounded example per stable gate name and never more than
// the public cap. This is diagnostic evidence, not an unbounded history.
void RecordGateExample(
  OfflineStaticMapMetrics * metrics, const std::string & gate,
  const FineCellIndex & cell, const Point3 & point,
  double actual, double threshold)
{
  if (metrics == nullptr ||
    metrics->gate_examples.size() >= kOfflineGateExampleLimit)
  {
    return;
  }
  for (const OfflineGateExample & existing : metrics->gate_examples) {
    if (existing.gate == gate) {
      return;
    }
  }
  OfflineGateExample example;
  example.gate = gate;
  example.cell = cell;
  example.point = point;
  example.actual = actual;
  example.threshold = threshold;
  metrics->gate_examples.push_back(std::move(example));
}

// Packs a finite world coordinate into the content hash.
void HashDouble(std::uint64_t * hash, double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
  if (std::isnan(value)) {
    bits = 0x7ff8000000000001ULL;
  } else {
    const double canonical = value == 0.0 ? 0.0 : value;
    std::memcpy(&bits, &canonical, sizeof(bits));
  }
  *hash ^= bits + 0x9e3779b97f4a7c15ULL + (*hash << 6U) + (*hash >> 2U);
}

// Deterministic content hash over every option that can change the published
// map, every accepted node, every evaluated connection and the full canonical
// measurement evidence. Counter/budget limits are included because crossing one
// changes the build outcome.
void HashSurfaceReconstruction(
  std::uint64_t * hash, const SurfaceReconstructionEvidence & evidence)
{
  HashBool(hash, evidence.attempted);
  HashBool(hash, evidence.applied);
  HashU64(hash, static_cast<std::uint64_t>(evidence.status));
  HashDouble(hash, evidence.original_centroid.x);
  HashDouble(hash, evidence.original_centroid.y);
  HashDouble(hash, evidence.original_centroid.z);
  HashDouble(hash, evidence.original_normal.x);
  HashDouble(hash, evidence.original_normal.y);
  HashDouble(hash, evidence.original_normal.z);
  HashDouble(hash, evidence.reconstructed_centroid.x);
  HashDouble(hash, evidence.reconstructed_centroid.y);
  HashDouble(hash, evidence.reconstructed_centroid.z);
  HashDouble(hash, evidence.reconstructed_normal.x);
  HashDouble(hash, evidence.reconstructed_normal.y);
  HashDouble(hash, evidence.reconstructed_normal.z);
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
    HashDouble(hash, corner.x);
    HashDouble(hash, corner.y);
    HashDouble(hash, corner.z);
  }
  HashSize(hash, evidence.regional_support_cells);
}

std::uint64_t HashBuildContent(
  const OfflineStaticMapOptions & options,
  const std::string & frame_id,
  const std::vector<TerrainNode> & nodes,
  const std::vector<TerrainConnection> & connections,
  const MeasurementEvidence & evidence)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const char character : frame_id) {
    HashU64(&hash, static_cast<unsigned char>(character));
  }
  HashDouble(&hash, options.resolution);
  HashDouble(&hash, options.origin_x);
  HashDouble(&hash, options.origin_y);
  HashDouble(&hash, options.neighborhood_radius_m);
  HashDouble(&hash, options.evidence_expansion_step_m);
  HashDouble(&hash, options.global_surface_expansion_max_radius_m);
  HashDouble(&hash, options.global_surface_min_angular_coverage);
  HashDouble(&hash, options.max_sampling_gap_m);
  HashDouble(&hash, options.max_plane_rms_m);
  HashDouble(&hash, options.max_normal_angle_rad);
  HashDouble(&hash, options.max_slope_rad);
  HashDouble(&hash, options.max_step_height_m);
  HashDouble(&hash, options.layer_gap_threshold_m);
  HashDouble(&hash, options.min_boundary_coverage);
  HashSize(&hash, options.min_neighborhood_points);
  HashBool(&hash, options.mark_excessive_slope_prohibited);
  HashSize(&hash, options.max_candidate_cells);
  HashSize(&hash, options.max_measurement_points);
  HashSize(&hash, options.max_measurement_patches);
  HashSize(&hash, options.max_measurement_point_refs);
  HashSize(&hash, options.max_points_per_measurement_patch);
  HashDouble(&hash, options.ground_duplicate_tolerance_m);
  HashDouble(&hash, options.obstacle_blocking_height_m);
  HashDouble(&hash, options.obstacle_association_radius_m);
  HashDouble(&hash, options.obstacle_tile_size_m);
  HashDouble(&hash, options.global_ground_seed_tolerance_m);
  HashDouble(&hash, options.global_hint_surface_tolerance_m);
  HashBool(&hash, options.ground_surface.enabled);
  HashDouble(&hash, options.ground_surface.max_height_noise_m);
  HashDouble(&hash, options.ground_surface.max_correction_m);
  HashDouble(&hash, options.ground_surface.max_normal_correction_rad);
  HashDouble(&hash, options.ground_surface.max_association_normal_rad);
  HashDouble(&hash, options.ground_surface.max_support_residual_m);
  HashDouble(&hash, options.ground_surface.max_candidate_slope_rad);
  HashSize(&hash, options.ground_surface.max_neighbors);
  HashSize(&hash, options.ground_surface.max_points_per_node);
  HashBool(&hash, options.continuous_ground.enabled);
  HashDouble(&hash, options.continuous_ground.regional_radius_m);
  HashDouble(&hash, options.continuous_ground.max_gap_fill_m);
  HashDouble(&hash, options.continuous_ground.max_plane_rms_m);
  HashDouble(&hash, options.continuous_ground.max_inlier_residual_m);
  HashDouble(&hash, options.continuous_ground.robust_noise_floor_m);
  HashDouble(&hash, options.continuous_ground.min_region_confidence);
  HashDouble(&hash, options.continuous_ground.min_angular_coverage);
  HashDouble(&hash, options.continuous_ground.corner_association_tolerance_m);
  HashDouble(&hash, options.continuous_ground.max_association_normal_rad);
  HashSize(&hash, options.continuous_ground.min_region_points);
  HashSize(&hash, options.continuous_ground.max_region_points);
  HashSize(&hash, options.continuous_ground.max_layers_per_cell);
  HashSize(&hash, options.continuous_ground.max_candidate_seeds);
  HashSize(&hash, options.continuous_ground.max_support_refs_per_cell);
  HashSize(&hash, options.continuous_ground.max_inferred_cells);

  for (const TerrainNode & node : nodes) {
    HashU64(&hash, node.id);
    HashU64(&hash, static_cast<std::uint64_t>(node.cell.x));
    HashU64(&hash, static_cast<std::uint64_t>(node.cell.y));
    HashU64(&hash, static_cast<std::uint64_t>(node.local_surface_index));
    HashDouble(&hash, node.representative.x);
    HashDouble(&hash, node.representative.y);
    HashDouble(&hash, node.representative.z);
    HashU64(&hash, static_cast<std::uint64_t>(node.status));
    HashU64(&hash, static_cast<std::uint64_t>(node.reason));
    HashBool(&hash, node.plane.valid);
    HashDouble(&hash, node.plane.centroid.x);
    HashDouble(&hash, node.plane.centroid.y);
    HashDouble(&hash, node.plane.centroid.z);
    HashDouble(&hash, node.plane.normal.x);
    HashDouble(&hash, node.plane.normal.y);
    HashDouble(&hash, node.plane.normal.z);
    HashDouble(&hash, node.plane.offset);
    HashDouble(&hash, node.plane.rms_residual);
    HashSize(&hash, node.plane.inlier_count);
    HashSize(&hash, node.plane.sample_count);
    HashSize(&hash, node.support.sample_count);
    HashDouble(&hash, node.support.z_min);
    HashDouble(&hash, node.support.z_max);
    HashDouble(&hash, node.support.edge_support_coverage);
    for (const bool supported : node.support.supported_sectors) {
      HashBool(&hash, supported);
    }
    for (const Point3 & point : node.support.points) {
      HashDouble(&hash, point.x);
      HashDouble(&hash, point.y);
      HashDouble(&hash, point.z);
    }
    HashSurfaceReconstruction(&hash, node.surface_reconstruction);
  }
  for (const TerrainConnection & connection : connections) {
    HashU64(&hash, connection.source);
    HashU64(&hash, connection.target);
    HashU64(&hash, static_cast<std::uint64_t>(connection.status));
    HashU64(&hash, static_cast<std::uint64_t>(connection.reason));
    HashDouble(&hash, connection.height_delta);
    HashDouble(&hash, connection.horizontal_distance);
    HashDouble(&hash, connection.slope);
    HashDouble(&hash, connection.boundary_coverage);
    HashDouble(&hash, connection.normal_dot);
  }
  HashU64(&hash, HashMeasurementEvidence(evidence));
  return hash;
}

// Fine bin key used only for neighbourhood lookup, never as a node identity.
std::int64_t BinCoord(double value, double origin, double bin_size) noexcept
{
  return static_cast<std::int64_t>(
    std::floor((value - origin) / bin_size));
}

}  // namespace

const char * OfflineStaticMapStatusToString(
  OfflineStaticMapStatus status) noexcept
{
  switch (status) {
    case OfflineStaticMapStatus::kOk:
      return "ok";
    case OfflineStaticMapStatus::kInvalidOptions:
      return "invalid_options";
    case OfflineStaticMapStatus::kEmptyGround:
      return "empty_ground";
    case OfflineStaticMapStatus::kTooManyCandidateCells:
      return "too_many_candidate_cells";
    case OfflineStaticMapStatus::kMeasurementBudgetExceeded:
      return "measurement_budget_exceeded";
    case OfflineStaticMapStatus::kBuildFailed:
      return "build_failed";
  }
  return "unknown";
}

namespace
{

OfflineStaticMapResult BuildOfflineStaticMapImpl(
  const std::vector<Point3> & ground_points,
  const std::vector<Point3> & obstacle_points,
  const OfflineStaticMapOptions & options,
  const std::string & frame_id,
  const std::vector<Point3> * ground_hints,
  bool classify_structure)
{
  OfflineStaticMapResult result;
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&](OfflineStaticMapStatus status,
      std::string detail) {
      result.status = status;
      result.detail = std::move(detail);
      result.metrics.total_ms =
        DurationToMs(std::chrono::steady_clock::now() - started);
      return result;
    };
  const auto finish_budget = [&](std::string detail) {
      ++result.metrics.measurement_budget_rejections;
      return finish(
        OfflineStaticMapStatus::kMeasurementBudgetExceeded,
        std::move(detail));
    };

  // Validate every option before touching input. A bad offline profile must
  // fail closed instead of producing a plausible-looking map. Zero memory caps
  // are reported by key so the caller can correct the exact option.
  if (options.max_measurement_points == 0U) {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "max_measurement_points must be positive");
  }
  if (options.max_measurement_patches == 0U) {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "max_measurement_patches must be positive");
  }
  if (options.max_measurement_point_refs == 0U) {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "max_measurement_point_refs must be positive");
  }
  if (options.max_points_per_measurement_patch == 0U) {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "max_points_per_measurement_patch must be positive");
  }
  if (!IsValidResolution(options.resolution) ||
    !std::isfinite(options.origin_x) || !std::isfinite(options.origin_y) ||
    !std::isfinite(options.neighborhood_radius_m) ||
    options.neighborhood_radius_m <= 0.0 ||
    !std::isfinite(options.max_sampling_gap_m) ||
    options.max_sampling_gap_m <= 0.0 ||
    !std::isfinite(options.max_plane_rms_m) ||
    options.max_plane_rms_m <= 0.0 ||
    !std::isfinite(options.max_normal_angle_rad) ||
    options.max_normal_angle_rad <= 0.0 ||
    options.max_normal_angle_rad >= 0.5 * 3.14159265358979323846 ||
    !std::isfinite(options.max_slope_rad) || options.max_slope_rad <= 0.0 ||
    options.max_slope_rad >= 0.5 * 3.14159265358979323846 ||
    !std::isfinite(options.max_step_height_m) ||
    options.max_step_height_m <= 0.0 ||
    !std::isfinite(options.layer_gap_threshold_m) ||
    options.layer_gap_threshold_m <= 0.0 ||
    !std::isfinite(options.min_boundary_coverage) ||
    options.min_boundary_coverage < 0.0 ||
    options.min_boundary_coverage > 1.0 ||
    options.min_neighborhood_points < 3U ||
    options.max_candidate_cells == 0U ||
    options.max_measurement_points == 0U ||
    options.max_measurement_patches == 0U ||
    options.max_measurement_point_refs == 0U ||
    options.max_points_per_measurement_patch == 0U ||
    !std::isfinite(options.ground_duplicate_tolerance_m) ||
    options.ground_duplicate_tolerance_m < 0.0 ||
    !std::isfinite(options.obstacle_blocking_height_m) ||
    options.obstacle_blocking_height_m <= 0.0 ||
    !std::isfinite(options.obstacle_association_radius_m) ||
    options.obstacle_association_radius_m <= 0.0 ||
    !std::isfinite(options.obstacle_tile_size_m) ||
    options.obstacle_tile_size_m <= 0.0)
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "offline static-map thresholds are outside their valid ranges");
  }
  // Global-only defaults must not invalidate legacy ground-only callers that
  // use different neighbourhood/layer thresholds and never consume them.
  if (classify_structure &&
    (!std::isfinite(options.global_ground_seed_tolerance_m) ||
    options.global_ground_seed_tolerance_m < 0.0 ||
    !std::isfinite(options.global_hint_surface_tolerance_m) ||
    options.global_hint_surface_tolerance_m <= 0.0 ||
    options.global_hint_surface_tolerance_m >=
    options.layer_gap_threshold_m ||
    !std::isfinite(options.global_surface_expansion_max_radius_m) ||
    options.global_surface_expansion_max_radius_m <= 0.0 ||
    !std::isfinite(options.global_surface_min_angular_coverage) ||
    options.global_surface_min_angular_coverage <= 0.0 ||
    options.global_surface_min_angular_coverage > 1.0))
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "global surface thresholds are outside their valid ranges");
  }
  // Regional continuous reconstruction is an independent ground-only mode: it
  // must not be silently disabled by the legacy bounded-smoother option. When
  // active it supersedes that smoother, while legacy callers with both options
  // disabled keep byte-identical output.
  const bool continuous_ground_active =
    !classify_structure && options.continuous_ground.enabled;
  const bool reconstruct_ground_surface = !classify_structure &&
    (options.ground_surface.enabled || continuous_ground_active);
  if (continuous_ground_active) {
    const ContinuousGroundSurfaceOptions & continuous =
      options.continuous_ground;
    if (!std::isfinite(continuous.regional_radius_m) ||
      continuous.regional_radius_m <= 0.0 ||
      !std::isfinite(continuous.max_gap_fill_m) ||
      continuous.max_gap_fill_m <= 0.0 ||
      !std::isfinite(continuous.max_plane_rms_m) ||
      continuous.max_plane_rms_m <= 0.0 ||
      !std::isfinite(continuous.max_inlier_residual_m) ||
      continuous.max_inlier_residual_m <= 0.0 ||
      !std::isfinite(continuous.robust_noise_floor_m) ||
      continuous.robust_noise_floor_m <= 0.0 ||
      !std::isfinite(continuous.min_region_confidence) ||
      continuous.min_region_confidence < 0.0 ||
      continuous.min_region_confidence > 1.0 ||
      !std::isfinite(continuous.min_angular_coverage) ||
      continuous.min_angular_coverage < 0.0 ||
      continuous.min_angular_coverage > 1.0 ||
      !std::isfinite(continuous.corner_association_tolerance_m) ||
      continuous.corner_association_tolerance_m <= 0.0 ||
      continuous.corner_association_tolerance_m >
      options.max_step_height_m + kEpsilon ||
      !std::isfinite(continuous.max_association_normal_rad) ||
      continuous.max_association_normal_rad <= 0.0 ||
      continuous.max_association_normal_rad >=
      0.5 * 3.14159265358979323846 ||
      continuous.min_region_points < 4U ||
      continuous.max_region_points < continuous.min_region_points ||
      continuous.max_layers_per_cell == 0U ||
      continuous.max_candidate_seeds == 0U ||
      continuous.max_support_refs_per_cell < 4U ||
      continuous.max_inferred_cells == 0U)
    {
      return finish(
        OfflineStaticMapStatus::kInvalidOptions,
        "continuous ground surface thresholds/bounds are outside their "
        "valid ranges");
    }
  }
  // Ground-mode quality-failure context re-estimation uses the existing
  // raw-point budget and same-floor layer selector at one bounded radius. The
  // disabled default keeps this at zero and therefore keeps legacy behavior.
  const double ground_context_radius = reconstruct_ground_surface ?
    std::min(
    options.neighborhood_radius_m + options.max_sampling_gap_m,
    kMaxGroundContextRadius) : 0.0;
  if (reconstruct_ground_surface) {
    const GroundSurfaceReconstructionOptions & ground =
      options.ground_surface;
    if (!std::isfinite(ground.max_height_noise_m) ||
      ground.max_height_noise_m <= 0.0 ||
      !std::isfinite(ground.max_correction_m) ||
      ground.max_correction_m < 0.0 ||
      !std::isfinite(ground.max_normal_correction_rad) ||
      ground.max_normal_correction_rad <= 0.0 ||
      ground.max_normal_correction_rad >=
      0.5 * 3.14159265358979323846 ||
      !std::isfinite(ground.max_association_normal_rad) ||
      ground.max_association_normal_rad <= 0.0 ||
      ground.max_association_normal_rad >=
      0.5 * 3.14159265358979323846 ||
      !std::isfinite(ground.max_support_residual_m) ||
      ground.max_support_residual_m <= 0.0 ||
      !std::isfinite(ground.max_candidate_slope_rad) ||
      ground.max_candidate_slope_rad <= 0.0 ||
      ground.max_candidate_slope_rad >=
      0.5 * 3.14159265358979323846 ||
      ground.max_neighbors == 0U ||
      ground.max_neighbors > kMaxGroundSurfaceNeighbors ||
      ground.max_points_per_node == 0U ||
      ground.max_points_per_node > kMaxGroundSurfacePointsPerNode)
    {
      return finish(
        OfflineStaticMapStatus::kInvalidOptions,
        "ground surface reconstruction thresholds/direction/sample bounds "
        "are outside their valid ranges");
    }
  }
  if (!std::isfinite(options.evidence_expansion_step_m) ||
    options.evidence_expansion_step_m <= 0.0 ||
    options.evidence_expansion_step_m > options.resolution ||
    options.evidence_expansion_step_m >
    0.5 * options.max_sampling_gap_m + kEpsilon ||
    !std::isfinite(
      options.neighborhood_radius_m + options.evidence_expansion_step_m))
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "evidence_expansion_step_m must be finite, positive, no larger "
      "than resolution, and no larger than max_sampling_gap_m/2");
  }
  const double minimum_global_radius =
    options.neighborhood_radius_m + options.evidence_expansion_step_m;
  const double maximum_global_radius =
    options.neighborhood_radius_m + options.max_sampling_gap_m;
  if (classify_structure &&
    (minimum_global_radius <= options.neighborhood_radius_m ||
    options.global_surface_expansion_max_radius_m <=
    options.neighborhood_radius_m ||
    options.global_surface_expansion_max_radius_m <
    minimum_global_radius - kEpsilon ||
    options.global_surface_expansion_max_radius_m >
    maximum_global_radius + kEpsilon))
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "global_surface_expansion_max_radius_m must lie in "
      "[neighborhood_radius_m + evidence_expansion_step_m, "
      "neighborhood_radius_m + max_sampling_gap_m]");
  }
  if (classify_structure) {
    const double attempts = std::ceil(
      (options.global_surface_expansion_max_radius_m -
      options.neighborhood_radius_m) / options.evidence_expansion_step_m);
    if (!std::isfinite(attempts) ||
      attempts > static_cast<double>(kMaxAdaptiveExpansionAttempts))
    {
      return finish(
        OfflineStaticMapStatus::kInvalidOptions,
        "global surface expansion exceeds the 16-attempt per-patch budget");
    }
  }
  if (options.max_points_per_measurement_patch <
    options.min_neighborhood_points)
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "max_points_per_measurement_patch must be at least "
      "min_neighborhood_points");
  }
  const double halo_radius = continuous_ground_active ?
    std::max(
    options.continuous_ground.max_gap_fill_m,
    std::max(options.neighborhood_radius_m, ground_context_radius)) :
    (reconstruct_ground_surface ?
    std::max(options.neighborhood_radius_m, ground_context_radius) :
    options.neighborhood_radius_m);
  const double halo_value =
    std::ceil(halo_radius / options.resolution) + 1.0;
  if (!std::isfinite(halo_value) ||
    halo_value >
    static_cast<double>(std::numeric_limits<std::int64_t>::max() / 4))
  {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "offline static-map neighborhood halo is outside the supported range");
  }
  if (frame_id.empty()) {
    return finish(
      OfflineStaticMapStatus::kInvalidOptions,
      "offline static-map frame_id is empty");
  }
  if (ground_points.empty()) {
    return finish(
      OfflineStaticMapStatus::kEmptyGround,
      "ground point cloud is empty");
  }

  result.metrics.ground_points = ground_points.size();
  result.metrics.obstacle_points = obstacle_points.size();

  // The public ref type is uint32. Check the raw input table before allocating
  // the canonical table or any index vector.
  if (ground_points.size() >
    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    return finish_budget(
      "ground point table exceeds the uint32 index range");
  }
  if (ground_points.size() > options.max_measurement_points) {
    return finish_budget(
      "ground point count " + std::to_string(ground_points.size()) +
      " exceeds max_measurement_points (" +
      std::to_string(options.max_measurement_points) + ")");
  }

  // 1. Build the canonical finite real point table. Sorting and exact
  //    deduplication make source order and repeated coordinates deterministic;
  //    no point is generated or interpolated.
  std::vector<Point3> finite_ground;
  finite_ground.reserve(ground_points.size());
  for (const Point3 & point : ground_points) {
    if (!point.IsFinite()) {
      continue;
    }
    const auto cell = FineCellOf(
      point, options.origin_x, options.origin_y,
      options.resolution);
    if (!cell.has_value()) {
      continue;
    }
    finite_ground.push_back(point);
  }
  if (finite_ground.empty()) {
    return finish(
      OfflineStaticMapStatus::kEmptyGround,
      "ground point cloud has no finite point");
  }
  SortAndDeduplicatePoints(&finite_ground);
  if (finite_ground.size() >
    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    return finish_budget(
      "canonical ground point table exceeds the uint32 index range");
  }
  result.metrics.measurement_points = finite_ground.size();

  // Fine lookup bins store canonical table indices. A bin is at least one
  // neighbourhood radius wide, so the bounded 3x3 scan is exact.
  const double legacy_expanded_radius =
    options.neighborhood_radius_m + options.evidence_expansion_step_m;
  // Global mode must collect up to its explicit bounded maximum; legacy mode
  // keeps exactly the previous one-step annulus.
  const double expanded_radius = classify_structure ?
    std::max(
    legacy_expanded_radius,
    options.global_surface_expansion_max_radius_m) :
    legacy_expanded_radius;
  const double bin_size = std::max(
    std::max(expanded_radius, ground_context_radius), options.resolution);
  // Temporary neighbour collection is bounded by the smaller of the point-table
  // and stored-ref policies. Per-layer capacity is enforced separately so floors
  // are never charged against one another.
  const std::size_t temporary_ref_cap = std::min(
    options.max_measurement_points, options.max_measurement_point_refs);
  std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>>
  lookup_bins;
  std::int64_t min_cell_x = std::numeric_limits<std::int64_t>::max();
  std::int64_t min_cell_y = std::numeric_limits<std::int64_t>::max();
  std::int64_t max_cell_x = std::numeric_limits<std::int64_t>::min();
  std::int64_t max_cell_y = std::numeric_limits<std::int64_t>::min();
  for (std::size_t index = 0; index < finite_ground.size(); ++index) {
    const Point3 & point = finite_ground[index];
    const auto cell = FineCellOf(
      point, options.origin_x, options.origin_y,
      options.resolution);
    if (!cell.has_value()) {
      continue;
    }
    const auto bin_x = BinCoord(point.x, options.origin_x, bin_size);
    const auto bin_y = BinCoord(point.y, options.origin_y, bin_size);
    lookup_bins[{bin_x, bin_y}].push_back(index);
    min_cell_x = std::min(min_cell_x, cell->x);
    min_cell_y = std::min(min_cell_y, cell->y);
    max_cell_x = std::max(max_cell_x, cell->x);
    max_cell_y = std::max(max_cell_y, cell->y);
  }

  // 2. Checked candidate span. Candidate cells include a one-radius halo, but
  //    the support gate below still decides whether any surface is real.
  const std::int64_t halo = static_cast<std::int64_t>(halo_value);
  std::int64_t span_x = 0;
  std::int64_t span_y = 0;
  if (!CheckedSpan(min_cell_x, max_cell_x, halo, &span_x) ||
    !CheckedSpan(min_cell_y, max_cell_y, halo, &span_y))
  {
    return finish(
      OfflineStaticMapStatus::kTooManyCandidateCells,
      "candidate-cell span arithmetic overflowed");
  }
  const double candidate_count =
    static_cast<double>(span_x) * static_cast<double>(span_y);
  if (!std::isfinite(candidate_count) ||
    candidate_count > static_cast<double>(options.max_candidate_cells))
  {
    return finish(
      OfflineStaticMapStatus::kTooManyCandidateCells,
      "candidate-cell span exceeds max_candidate_cells (" +
      std::to_string(options.max_candidate_cells) + ")");
  }

  const auto terrain_started = std::chrono::steady_clock::now();
  std::vector<TerrainNode> nodes;
  std::vector<MeasurementPatch> patches;
  // Real positive residual refs from every attempt become independent obstacle
  // evidence. Generated refs keep their source supported node so a second floor
  // cannot re-associate them as duplicate ground. They never enter node
  // support/hull/fit metadata and negative residuals are never promoted.
  std::vector<RaisedEvidence> raised_evidence;
  TerrainNodeId next_node_id = 1;
  std::uint64_t next_patch_id = 1;

  const auto make_node = [&](const SurfaceAttempt & attempt,
      const FineCellIndex & cell, const Point3 & center, bool prohibited) {
      TerrainNode node;
      node.id = next_node_id++;
      node.cell = cell;
      node.local_surface_index = -1;
      const double plane_z =
        attempt.plane.a * center.x + attempt.plane.b * center.y +
        attempt.plane.c;
      node.representative = Point3{center.x, center.y, plane_z};
      node.plane.valid = true;
      node.plane.centroid = attempt.plane.centroid;
      node.plane.normal = attempt.plane.normal;
      node.plane.offset = Dot(
        node.plane.normal,
        Vector3{node.plane.centroid.x, node.plane.centroid.y,
          node.plane.centroid.z});
      node.plane.rms_residual = attempt.plane.rms;
      node.plane.inlier_count = attempt.inlier_indices.size();
      node.plane.sample_count = attempt.point_indices.size();
      node.support.points.reserve(attempt.inlier_indices.size());
      for (const std::uint32_t index : attempt.inlier_indices) {
        node.support.points.push_back(finite_ground[index]);
      }
      node.support.z_min = std::numeric_limits<double>::infinity();
      node.support.z_max = -std::numeric_limits<double>::infinity();
      for (const Point3 & point : node.support.points) {
        node.support.z_min = std::min(node.support.z_min, point.z);
        node.support.z_max = std::max(node.support.z_max, point.z);
      }
      node.support.sample_count = node.support.points.size();
      node.support.supported_sectors.fill(false);
      for (const Point3 & point : node.support.points) {
        const double dx = point.x - center.x;
        const double dy = point.y - center.y;
        const double angle = std::atan2(dy, dx);
        const double normalized =
          (angle + 3.14159265358979323846) /
          (2.0 * 3.14159265358979323846);
        const auto sector = static_cast<std::size_t>(
          std::floor(normalized * static_cast<double>(kSupportSectorCount)));
        node.support.supported_sectors[
          std::min(sector, kSupportSectorCount - 1U)] = true;
      }
      node.support.edge_support_coverage = node.support.SectorCoverage();
      node.status = prohibited ?
        TerrainStatus::kProhibited : TerrainStatus::kSupported;
      node.reason = prohibited ?
        ProhibitedReason::kExcessiveSlope : ProhibitedReason::kNone;
      return node;
    };

  const auto record_rejection = [&](const SurfaceAttempt & attempt,
      const FineCellIndex & cell, const Point3 & center) {
      const Point3 example = attempt.conflict_point;
      switch (attempt.reason) {
        case MeasurementRejectionReason::kInsufficientPoints:
          ++result.metrics.rejected_insufficient_points;
          RecordGateExample(
            &result.metrics, "insufficient_points", cell, center,
            static_cast<double>(attempt.sample_count),
            static_cast<double>(options.min_neighborhood_points));
          break;
        case MeasurementRejectionReason::kSamplingGap:
          ++result.metrics.rejected_sampling_gap;
          RecordGateExample(
            &result.metrics, "sampling_gap", cell, center,
            attempt.nearest_support_xy_m, options.max_sampling_gap_m);
          break;
        case MeasurementRejectionReason::kUnsupportedCenter:
          ++result.metrics.rejected_unsupported_center;
          RecordGateExample(
            &result.metrics, "unsupported_center", cell, center,
            attempt.nearest_support_xy_m,
            std::numeric_limits<double>::quiet_NaN());
          break;
        case MeasurementRejectionReason::kHighResidual:
          ++result.metrics.rejected_plane_rms;
          RecordGateExample(
            &result.metrics, "plane_rms", cell, example,
            attempt.plane.valid ? attempt.plane.rms :
            std::numeric_limits<double>::quiet_NaN(),
            options.max_plane_rms_m);
          break;
        case MeasurementRejectionReason::kInsufficientInliers:
          ++result.metrics.rejected_plane_inliers;
          RecordGateExample(
            &result.metrics, "plane_inliers", cell, example,
            static_cast<double>(attempt.inlier_indices.size()),
            static_cast<double>(options.min_neighborhood_points));
          break;
        case MeasurementRejectionReason::kNonFinitePlane:
          ++result.metrics.rejected_nonfinite_plane;
          RecordGateExample(
            &result.metrics, "nonfinite_plane", cell, center,
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN());
          break;
        case MeasurementRejectionReason::kLocalDepression:
          ++result.metrics.rejected_robust_depression;
          ++result.metrics.local_depression_rejections;
          RecordGateExample(
            &result.metrics, "robust_depression", cell, example,
            attempt.own_cell_max_depression_m, options.max_step_height_m);
          break;
        case MeasurementRejectionReason::kDegenerateGeometry:
        case MeasurementRejectionReason::kNone:
          ++result.metrics.rejected_plane_fit;
          RecordGateExample(
            &result.metrics, "plane_fit", cell, example,
            static_cast<double>(attempt.sample_count),
            static_cast<double>(options.min_neighborhood_points));
          break;
        case MeasurementRejectionReason::kExcessiveSlope:
        case MeasurementRejectionReason::kLocalRaisedConflict:
        case MeasurementRejectionReason::kExpansionInvalid:
        case MeasurementRejectionReason::kInsufficientCoverage:
        case MeasurementRejectionReason::kUnconfirmedComponent:
        case MeasurementRejectionReason::kDiscontinuousTerrain:
          ++result.metrics.rejected_plane_fit;
          break;
      }
    };

  if (continuous_ground_active) {
    // Regional same-layer reconstruction replaces the legacy per-cell fit/veto.
    // It runs on the canonical table before any per-cell acceptance decision, so
    // a sparse scanline can seed a real surface and interior cells can be
    // inferred. Raw points and patch.fit refs stay measured evidence.
    std::vector<ContinuousSurfaceCell> continuous_cells;
    ContinuousGroundSurfaceMetrics continuous_metrics;
    if (!BuildContinuousGroundSurface(
        finite_ground, options.continuous_ground, options.resolution,
        options.origin_x, options.origin_y, options.max_step_height_m,
        options.max_slope_rad, &continuous_cells, &continuous_metrics))
    {
      return finish(
        OfflineStaticMapStatus::kBuildFailed,
        "continuous ground surface reconstruction violated its bounds");
    }
    if (continuous_cells.size() > options.max_candidate_cells) {
      return finish(
        OfflineStaticMapStatus::kTooManyCandidateCells,
        "continuous ground surface exceeds max_candidate_cells (" +
        std::to_string(options.max_candidate_cells) + ")");
    }
    result.metrics.candidate_cells += continuous_cells.size();
    std::sort(
      continuous_cells.begin(), continuous_cells.end(),
      [](const ContinuousSurfaceCell & lhs, const ContinuousSurfaceCell & rhs) {
        if (lhs.cell.x != rhs.cell.x) {return lhs.cell.x < rhs.cell.x;}
        if (lhs.cell.y != rhs.cell.y) {return lhs.cell.y < rhs.cell.y;}
        if (lhs.center.z != rhs.center.z) {return lhs.center.z < rhs.center.z;}
        return lhs.inferred < rhs.inferred;
      });
    FineCellIndex current_cell{};
    int local_layer = -1;
    bool have_cell = false;
    for (const ContinuousSurfaceCell & cell : continuous_cells) {
      if (!have_cell || !(cell.cell == current_cell)) {
        current_cell = cell.cell;
        local_layer = 0;
        have_cell = true;
      } else {
        ++local_layer;
      }
      const Point3 cell_center = FineCellCenter(
        cell.cell, options.origin_x, options.origin_y, options.resolution);
      const Point3 & raw_centroid = cell.plane.centroid;
      const Vector3 & raw_normal = cell.plane.normal;
      const double normal_length = raw_normal.Norm();
      if (!cell.plane.valid || !raw_centroid.IsFinite() ||
        !raw_normal.IsFinite() || !std::isfinite(normal_length) ||
        std::fabs(normal_length - 1.0) > 1.0e-6 || !(raw_normal.z > kEpsilon))
      {
        continue;
      }
      const double raw_center_z = raw_centroid.z -
        (raw_normal.x * (cell_center.x - raw_centroid.x) +
        raw_normal.y * (cell_center.y - raw_centroid.y)) / raw_normal.z;
      if (!std::isfinite(raw_center_z)) {
        continue;
      }
      std::vector<std::uint32_t> refs = cell.point_indices;
      std::sort(refs.begin(), refs.end());
      refs.erase(std::unique(refs.begin(), refs.end()), refs.end());
      // The module already capped the refs it fitted. Never truncate persisted
      // provenance here: doing so would make patch.fit disagree with the plane
      // and RMS that were actually fitted. Fail the budget before mutation.
      if (refs.size() > options.max_points_per_measurement_patch) {
        return finish_budget(
          "continuous ground cell refs reached "
          "max_points_per_measurement_patch (" +
          std::to_string(options.max_points_per_measurement_patch) + ")");
      }
      if (refs.size() < 4U) {
        continue;
      }
      std::vector<std::uint32_t> inliers;
      for (const std::uint32_t index : cell.inlier_indices) {
        if (std::binary_search(refs.begin(), refs.end(), index)) {
          inliers.push_back(index);
        }
      }
      std::sort(inliers.begin(), inliers.end());
      inliers.erase(std::unique(inliers.begin(), inliers.end()), inliers.end());
      if (inliers.size() < 4U) {
        continue;
      }
      ContinuousFitStats ref_stats;
      ContinuousFitStats inlier_stats;
      if (!ComputeContinuousFitStats(
          finite_ground, refs, raw_centroid, raw_normal, &ref_stats) ||
        !ComputeContinuousFitStats(
          finite_ground, inliers, raw_centroid, raw_normal, &inlier_stats))
      {
        continue;
      }
      // Actual measured seed-cell count, not a display/validator-shaped cap.
      // The module already bounds it by max_candidate_seeds and the ref budget.
      const std::size_t support_nodes = cell.regional_support_cells;
      Point3 node_centroid = raw_centroid;
      Vector3 node_normal = raw_normal;
      double node_center_z = raw_center_z;
      const bool applied = cell.shared_corners_valid && support_nodes > 0U;
      if (applied) {
        Point3 derived_centroid{};
        Vector3 derived_normal{0.0, 0.0, 1.0};
        if (!ContinuousPlaneFromCorners(
            cell.shared_corners, cell_center, &derived_centroid,
            &derived_normal))
        {
          continue;
        }
        if (std::atan2(
            std::hypot(derived_normal.x, derived_normal.y),
            derived_normal.z) > options.max_slope_rad + kEpsilon)
        {
          continue;
        }
        node_centroid = derived_centroid;
        node_normal = derived_normal;
        node_center_z = derived_centroid.z;
      }
      ContinuousFitStats node_stats;
      if (!ComputeContinuousFitStats(
          finite_ground, inliers, node_centroid, node_normal, &node_stats))
      {
        continue;
      }
      std::vector<std::uint32_t> own_indices;
      for (const std::uint32_t index : refs) {
        const auto point_cell = FineCellOf(
          finite_ground[index], options.origin_x, options.origin_y,
          options.resolution);
        if (point_cell.has_value() && *point_cell == cell.cell) {
          own_indices.push_back(index);
        }
      }
      if (nodes.size() >= options.max_measurement_patches ||
        patches.size() >= options.max_measurement_patches)
      {
        return finish_budget(
          "measurement nodes/patches reached max_measurement_patches (" +
          std::to_string(options.max_measurement_patches) + ")");
      }
      // Exact persisted ref cost for this branch: base refs + fit refs + fit
      // inliers + own-cell refs. The inliers vector is stored and must be
      // charged; no fitted ref is silently dropped.
      std::size_t continuous_refs = result.metrics.measurement_point_refs;
      if (!AddRefBudget(
          &continuous_refs, refs.size(),
          options.max_measurement_point_refs) ||
        !AddRefBudget(
          &continuous_refs, refs.size(),
          options.max_measurement_point_refs) ||
        !AddRefBudget(
          &continuous_refs, inliers.size(),
          options.max_measurement_point_refs) ||
        !AddRefBudget(
          &continuous_refs, own_indices.size(),
          options.max_measurement_point_refs))
      {
        return finish_budget(
          "continuous ground refs exceed max_measurement_point_refs (" +
          std::to_string(options.max_measurement_point_refs) + ")");
      }
      SurfaceReconstructionEvidence evidence;
      evidence.attempted = true;
      evidence.applied = applied;
      evidence.status = applied ?
        SurfaceReconstructionStatus::kApplied :
        SurfaceReconstructionStatus::kInsufficientSupport;
      evidence.original_centroid = raw_centroid;
      evidence.original_normal = raw_normal;
      evidence.reconstructed_centroid = node_centroid;
      evidence.reconstructed_normal = node_normal;
      evidence.original_height_m = raw_center_z;
      evidence.reconstructed_height_m = node_center_z;
      evidence.correction_m = node_center_z - raw_center_z;
      evidence.original_rms_residual_m = inlier_stats.rms;
      evidence.reconstruction_rms_residual_m = node_stats.rms;
      evidence.own_support_rms_residual_m = node_stats.rms;
      evidence.confidence = std::clamp(cell.confidence, 0.0, 1.0);
      evidence.support_nodes = support_nodes;
      evidence.support_points = inliers.size();
      evidence.inferred = cell.inferred;
      evidence.shared_corners_valid = cell.shared_corners_valid;
      evidence.shared_corners = cell.shared_corners;
      evidence.regional_support_cells = cell.regional_support_cells;

      TerrainNode node;
      node.id = next_node_id++;
      node.cell = cell.cell;
      node.local_surface_index = local_layer;
      node.representative = Point3{
        cell_center.x, cell_center.y, node_center_z};
      node.plane.valid = true;
      node.plane.centroid = node_centroid;
      node.plane.normal = node_normal;
      node.plane.offset = Dot(
        node_normal,
        Vector3{node_centroid.x, node_centroid.y, node_centroid.z});
      node.plane.rms_residual = node_stats.rms;
      node.plane.inlier_count = inliers.size();
      node.plane.sample_count = inliers.size();
      node.support.points.reserve(inliers.size());
      node.support.z_min = std::numeric_limits<double>::infinity();
      node.support.z_max = -std::numeric_limits<double>::infinity();
      node.support.supported_sectors.fill(false);
      for (const std::uint32_t index : inliers) {
        const Point3 & point = finite_ground[index];
        node.support.points.push_back(point);
        node.support.z_min = std::min(node.support.z_min, point.z);
        node.support.z_max = std::max(node.support.z_max, point.z);
        const double angle = std::atan2(
          point.y - cell_center.y, point.x - cell_center.x);
        const double normalized =
          (angle + 3.14159265358979323846) /
          (2.0 * 3.14159265358979323846);
        const auto sector = static_cast<std::size_t>(
          std::floor(normalized * static_cast<double>(kSupportSectorCount)));
        node.support.supported_sectors[
          std::min(sector, kSupportSectorCount - 1U)] = true;
      }
      node.support.sample_count = node.support.points.size();
      node.support.edge_support_coverage = node.support.SectorCoverage();
      node.status = TerrainStatus::kSupported;
      node.reason = ProhibitedReason::kNone;
      node.surface_reconstruction = evidence;

      MeasurementPatch patch;
      patch.id = next_patch_id++;
      patch.cell = cell.cell;
      patch.local_layer_index = local_layer;
      patch.base_point_indices = refs;
      patch.base_sample_count = refs.size();
      patch.fit.radius_m = options.continuous_ground.regional_radius_m;
      patch.fit.point_indices = refs;
      patch.fit.inlier_indices = inliers;
      patch.fit.plane_valid = true;
      patch.fit.centroid = raw_centroid;
      patch.fit.normal = raw_normal;
      patch.fit.rms_residual = inlier_stats.rms;
      double nearest_support_xy_m = std::numeric_limits<double>::infinity();
      for (const std::uint32_t index : refs) {
        nearest_support_xy_m = std::min(
          nearest_support_xy_m,
          std::hypot(
            finite_ground[index].x - cell_center.x,
            finite_ground[index].y - cell_center.y));
      }
      patch.fit.nearest_support_xy_m = nearest_support_xy_m;
      patch.fit.support_gate_ok = true;
      patch.fit.center_in_support_hull = true;
      patch.fit.conflict_sample_count = refs.size();
      patch.fit.min_signed_residual = ref_stats.minimum;
      patch.fit.max_signed_residual = ref_stats.maximum;
      patch.fit.max_depression_m = ref_stats.max_depression;
      patch.fit.max_raised_m = ref_stats.max_raised;
      patch.fit.raised_conflict_sample_count = 0U;
      patch.fit.conflict_point = finite_ground[ref_stats.conflict_index];
      patch.own_cell_point_indices = own_indices;
      patch.own_cell_sample_count = own_indices.size();
      if (!own_indices.empty()) {
        ContinuousFitStats own_stats;
        if (ComputeContinuousFitStats(
            finite_ground, own_indices, raw_centroid, raw_normal, &own_stats))
        {
          patch.own_z_min = std::numeric_limits<double>::infinity();
          patch.own_z_max = -std::numeric_limits<double>::infinity();
          for (const std::uint32_t index : own_indices) {
            patch.own_z_min = std::min(
              patch.own_z_min, finite_ground[index].z);
            patch.own_z_max = std::max(
              patch.own_z_max, finite_ground[index].z);
          }
          patch.own_cell_min_signed_residual = own_stats.minimum;
          patch.own_cell_max_signed_residual = own_stats.maximum;
          patch.own_cell_max_depression_m = own_stats.max_depression;
          patch.own_cell_max_raised_m = own_stats.max_raised;
        }
      }
      patch.z_min = std::numeric_limits<double>::infinity();
      patch.z_max = -std::numeric_limits<double>::infinity();
      for (const std::uint32_t index : refs) {
        patch.z_min = std::min(patch.z_min, finite_ground[index].z);
        patch.z_max = std::max(patch.z_max, finite_ground[index].z);
      }
      patch.status = MeasurementPatchStatus::kSupported;
      patch.reason = MeasurementRejectionReason::kNone;
      patch.node_id = node.id;
      patch.surface_reconstruction = evidence;

      result.metrics.measurement_point_refs = continuous_refs;
      ++result.metrics.accepted_nodes;
      nodes.push_back(std::move(node));
      patches.push_back(std::move(patch));
    }
  } else {
    for (std::int64_t cell_x = min_cell_x - halo;
      cell_x <= max_cell_x + halo; ++cell_x)
    {
      for (std::int64_t cell_y = min_cell_y - halo;
        cell_y <= max_cell_y + halo; ++cell_y)
      {
        ++result.metrics.candidate_cells;
        const FineCellIndex cell{cell_x, cell_y};
        const Point3 center = FineCellCenter(
          cell, options.origin_x,
          options.origin_y,
          options.resolution);

        const auto center_bin_x =
          BinCoord(center.x, options.origin_x, bin_size);
        const auto center_bin_y =
          BinCoord(center.y, options.origin_y, bin_size);
        std::vector<std::uint32_t> neighbors;
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          for (std::int64_t dy = -1; dy <= 1; ++dy) {
            const auto iterator =
              lookup_bins.find({center_bin_x + dx, center_bin_y + dy});
            if (iterator == lookup_bins.end()) {
              continue;
            }
            for (const std::size_t index : iterator->second) {
              const Point3 & point = finite_ground[index];
              if (std::hypot(point.x - center.x, point.y - center.y) <=
                options.neighborhood_radius_m + kEpsilon)
              {
                if (neighbors.size() >= temporary_ref_cap) {
                  return finish_budget(
                    "neighbourhood refs reached the temporary budget (" +
                    std::to_string(temporary_ref_cap) + ")");
                }
                neighbors.push_back(static_cast<std::uint32_t>(index));
              }
            }
          }
        }
        if (neighbors.empty()) {
          ++result.metrics.unknown_candidate_cells;
          continue;
        }

        const std::vector<std::vector<std::uint32_t>> layers =
          SplitIndexLayers(
          finite_ground, neighbors, options.layer_gap_threshold_m);
        std::vector<TerrainNode> cell_nodes;
        for (std::size_t layer_index = 0; layer_index < layers.size();
          ++layer_index)
        {
          const std::vector<std::uint32_t> & layer = layers[layer_index];
          ++result.metrics.candidate_layers;
          // Per-patch capacity belongs to one actual measured layer, not the
          // aggregate of different floors in this XY neighbourhood.
          if (layer.size() > options.max_points_per_measurement_patch) {
            return finish_budget(
              "layer refs reached max_points_per_measurement_patch (" +
              std::to_string(
                options.max_points_per_measurement_patch) + ")");
          }
          // This layer is a real additional patch; reject here rather than while
          // visiting a later empty halo cell.
          if (patches.size() >= options.max_measurement_patches) {
            return finish_budget(
              "measurement patches reached max_measurement_patches (" +
              std::to_string(options.max_measurement_patches) + ")");
          }

          std::size_t base_own_count = 0U;
          for (const std::uint32_t index : layer) {
            const auto point_cell = FineCellOf(
              finite_ground[index], options.origin_x, options.origin_y,
              options.resolution);
            if (point_cell.has_value() && *point_cell == cell) {
              ++base_own_count;
            }
          }
          // Preflight only the known minimum persisted cost (base refs, the
          // selected-fit refs that are at least the base refs, and own refs).
          // Inliers are temporary until the exact post-evaluation check.
          std::size_t base_preflight = result.metrics.measurement_point_refs;
          const bool base_budget_ok =
            AddRefBudget(
            &base_preflight, layer.size(),
            options.max_measurement_point_refs) &&
            AddRefBudget(
            &base_preflight, layer.size(),
            options.max_measurement_point_refs) &&
            AddRefBudget(
            &base_preflight, base_own_count,
            options.max_measurement_point_refs);
          if (!base_budget_ok) {
            return finish_budget(
              "measurement point refs exceed max_measurement_point_refs (" +
              std::to_string(options.max_measurement_point_refs) + ")");
          }

          SurfaceAttempt base_attempt = EvaluateSurfaceAttempt(
            finite_ground, layer, center, cell, options);
          bool expansion_attempted = false;
          bool expansion_used = false;
          double expansion_attempt_radius = legacy_expanded_radius;
          double expansion_used_radius = options.neighborhood_radius_m;
          SurfaceAttempt expansion_attempt;
          MeasurementRejectionReason expansion_reason =
            MeasurementRejectionReason::kNone;
          MeasurementExpansionSource expansion_source =
            MeasurementExpansionSource::kNone;
          std::vector<std::uint32_t> expanded_selected;
          const double base_slope_rad = base_attempt.plane.valid ?
            std::atan(
            std::hypot(base_attempt.plane.a, base_attempt.plane.b)) :
            std::numeric_limits<double>::quiet_NaN();
          // A base layer can already contain both sides of a real step when the
          // measured gap is below layer_gap_threshold_m, and its ordinary fit can
          // still be accepted when only one side lies in the own cell. Detect
          // that independently observed same-XY plane gap before any expansion;
          // this is not a raw Z-span gate and therefore does not reject a
          // continuous slope.
          const bool base_has_observed_gap =
            reconstruct_ground_surface &&
            ContextHasObservedHeightGap(
            finite_ground, layer, center,
            options.max_step_height_m, options.max_plane_rms_m);
          if (base_has_observed_gap && base_attempt.plane.valid) {
            base_attempt.accepted = false;
            base_attempt.reason = MeasurementRejectionReason::kDiscontinuousTerrain;
          }

          // Ground-mode pre-gate context re-estimation. Only a layer with real
          // own-cell observations and an actual quality failure (low-RMS but
          // over-slope local model, high RMS, own negative residual, or clearly
          // insufficient evidence) is eligible. It gathers actual same-floor raw
          // refs once at the hard bounded maximum, then tries at most three
          // deterministic real-point scales and keeps the first sufficient one.
          // Unobserved holes never enter. A proven base gap never expands: an
          // expansion must not dilute the discontinuity that is already measured.
          const bool ground_context_candidate =
            reconstruct_ground_surface &&
            !base_has_observed_gap &&
            base_attempt.HasOwnCellObservation() &&
            (base_attempt.reason == MeasurementRejectionReason::kHighResidual ||
            base_attempt.reason == MeasurementRejectionReason::kLocalDepression ||
            IsEvidenceInsufficientReason(base_attempt.reason) ||
            (base_attempt.accepted &&
            base_slope_rad > options.max_slope_rad + kEpsilon));
          if (ground_context_candidate &&
            ground_context_radius > options.neighborhood_radius_m + kEpsilon)
          {
            ++result.metrics.context_reestimation_attempts;
            std::vector<std::uint32_t> context_neighbors;
            for (std::int64_t dx = -2; dx <= 2; ++dx) {
              for (std::int64_t dy = -2; dy <= 2; ++dy) {
                const auto iterator = lookup_bins.find(
                  {center_bin_x + dx, center_bin_y + dy});
                if (iterator == lookup_bins.end()) {
                  continue;
                }
                for (const std::size_t index : iterator->second) {
                  const Point3 & point = finite_ground[index];
                  if (std::hypot(
                      point.x - center.x,
                      point.y - center.y) <= ground_context_radius + kEpsilon)
                  {
                    if (context_neighbors.size() >= temporary_ref_cap) {
                      return finish_budget(
                        "context neighbourhood refs reached the temporary "
                        "budget (" + std::to_string(temporary_ref_cap) + ")");
                    }
                    context_neighbors.push_back(
                      static_cast<std::uint32_t>(index));
                  }
                }
              }
            }
            std::sort(context_neighbors.begin(), context_neighbors.end());
            context_neighbors.erase(
              std::unique(
                context_neighbors.begin(), context_neighbors.end()),
              context_neighbors.end());

            // At most three deterministic scales inside the existing hard cap:
            // the legacy expanded radius, its midpoint with the cap, and the cap.
            // Duplicate values are removed; no floating-point sweep is performed.
            std::vector<double> context_radii;
            const double legacy_context_radius = std::min(
              legacy_expanded_radius, ground_context_radius);
            const auto add_context_radius = [&](double radius) {
                if (radius <= options.neighborhood_radius_m + kEpsilon ||
                  radius > ground_context_radius + kEpsilon)
                {
                  return;
                }
                for (const double existing : context_radii) {
                  if (std::abs(existing - radius) <= kEpsilon) {
                    return;
                  }
                }
                context_radii.push_back(radius);
              };
            add_context_radius(legacy_context_radius);
            add_context_radius(
              0.5 * (legacy_context_radius + ground_context_radius));
            add_context_radius(ground_context_radius);

            bool use_context = false;
            for (const double radius : context_radii) {
              std::vector<std::uint32_t> radius_refs;
              radius_refs.reserve(context_neighbors.size());
              for (const std::uint32_t index : context_neighbors) {
                const Point3 & point = finite_ground[index];
                if (std::hypot(
                    point.x - center.x,
                    point.y - center.y) <= radius + kEpsilon)
                {
                  radius_refs.push_back(index);
                }
              }

              MeasurementRejectionReason selection_reason =
                MeasurementRejectionReason::kExpansionInvalid;
              const bool selected_same_floor = SelectExpandedSameFloorLayer(
                finite_ground, layer, layers, layer_index, radius_refs,
                options, &expanded_selected, &selection_reason);
              if (expanded_selected.size() >
                options.max_points_per_measurement_patch)
              {
                return finish_budget(
                  "context layer refs reached "
                  "max_points_per_measurement_patch (" +
                  std::to_string(
                    options.max_points_per_measurement_patch) + ")");
              }
              std::size_t context_preflight =
                result.metrics.measurement_point_refs;
              const bool context_budget_ok =
                AddRefBudget(
                &context_preflight, layer.size(),
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &context_preflight, layer.size(),
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &context_preflight, base_own_count,
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &context_preflight, expanded_selected.size(),
                options.max_measurement_point_refs);
              if (!context_budget_ok) {
                return finish_budget(
                  "context measurement refs exceed "
                  "max_measurement_point_refs (" +
                  std::to_string(options.max_measurement_point_refs) + ")");
              }
              expansion_attempt = EvaluateSurfaceAttempt(
                finite_ground, expanded_selected, center, cell, options);
              expansion_attempted = true;
              expansion_attempt_radius = radius;
              expansion_source =
                MeasurementExpansionSource::kContextReestimation;
              ++result.metrics.expanded_patch_attempts;
              if (!selected_same_floor) {
                expansion_reason = MeasurementRejectionReason::kExpansionInvalid;
                ++result.metrics.rejected_expansion_invalid;
                continue;
              }
              if (!expansion_attempt.accepted) {
                expansion_reason = expansion_attempt.reason;
                if (expansion_reason == MeasurementRejectionReason::kNone) {
                  expansion_reason =
                    MeasurementRejectionReason::kDegenerateGeometry;
                }
                continue;
              }
              const double context_slope = std::atan(
                std::hypot(
                  expansion_attempt.plane.a, expansion_attempt.plane.b));
              // The old local model is replaced, so every own-cell raw sample and
              // every selected full-sample negative must be re-explained by the
              // new model within the unchanged .12 m step bound. Positive own
              // residuals are also bounded here: a real raised step or obstacle
              // must stay obstacle evidence, never become a traversable plane.
              const bool step_ok =
                expansion_attempt.own_cell_max_depression_m <=
                options.max_step_height_m + kEpsilon &&
                expansion_attempt.own_cell_max_raised_m <=
                options.max_step_height_m + kEpsilon &&
                !ContextHasObservedHeightGap(
                finite_ground, expanded_selected, center,
                options.max_step_height_m, options.max_plane_rms_m) &&
                expansion_attempt.max_depression_m <=
                options.max_step_height_m + kEpsilon;
              const double coverage = SelectedSectorCoverage(
                finite_ground, expanded_selected, center);
              if (context_slope > options.max_slope_rad + kEpsilon ||
                !step_ok)
              {
                expansion_reason =
                  MeasurementRejectionReason::kDiscontinuousTerrain;
                continue;
              }
              if (coverage + kEpsilon <
                kMinGroundContextAngularCoverage)
              {
                expansion_reason =
                  MeasurementRejectionReason::kInsufficientCoverage;
                ++result.metrics.rejected_adaptive_coverage;
                continue;
              }
              expansion_used = true;
              expansion_used_radius = radius;
              expansion_reason = MeasurementRejectionReason::kNone;
              use_context = true;
              ++result.metrics.expanded_patch_recoveries;
              ++result.metrics.context_reestimation_recoveries;
              break;
            }
            if (!use_context) {
              ++result.metrics.context_reestimation_rejections;
            }
          } else if (!base_has_observed_gap &&
            IsEvidenceInsufficientReason(base_attempt.reason))
          {
            std::vector<std::uint32_t> expanded_neighbors;
            for (std::int64_t dx = -1; dx <= 1; ++dx) {
              for (std::int64_t dy = -1; dy <= 1; ++dy) {
                const auto iterator = lookup_bins.find(
                  {center_bin_x + dx, center_bin_y + dy});
                if (iterator == lookup_bins.end()) {
                  continue;
                }
                for (const std::size_t index : iterator->second) {
                  const Point3 & point = finite_ground[index];
                  if (std::hypot(
                      point.x - center.x,
                      point.y - center.y) <= expanded_radius + kEpsilon)
                  {
                    if (expanded_neighbors.size() >= temporary_ref_cap) {
                      return finish_budget(
                        "expanded neighbourhood refs reached the temporary "
                        "budget (" + std::to_string(temporary_ref_cap) + ")");
                    }
                    expanded_neighbors.push_back(
                      static_cast<std::uint32_t>(index));
                  }
                }
              }
            }
            std::sort(expanded_neighbors.begin(), expanded_neighbors.end());
            expanded_neighbors.erase(
              std::unique(
                expanded_neighbors.begin(), expanded_neighbors.end()),
              expanded_neighbors.end());

            // Legacy keeps exactly one actual retry. Global mode tries the
            // bounded radii in increasing order and accepts the first one whose
            // same-floor fit also has spatially balanced sector coverage.
            std::vector<double> expansion_radii;
            if (classify_structure) {
              double radius = legacy_expanded_radius;
              while (radius <
                options.global_surface_expansion_max_radius_m - kEpsilon &&
                expansion_radii.size() + 1U < kMaxAdaptiveExpansionAttempts)
              {
                expansion_radii.push_back(radius);
                radius += options.evidence_expansion_step_m;
              }
              expansion_radii.push_back(
                options.global_surface_expansion_max_radius_m);
            } else {
              expansion_radii.push_back(legacy_expanded_radius);
            }

            for (const double radius : expansion_radii) {
              std::vector<std::uint32_t> radius_refs;
              radius_refs.reserve(expanded_neighbors.size());
              for (const std::uint32_t index : expanded_neighbors) {
                const Point3 & point = finite_ground[index];
                if (std::hypot(
                    point.x - center.x,
                    point.y - center.y) <= radius + kEpsilon)
                {
                  radius_refs.push_back(index);
                }
              }

              MeasurementRejectionReason selection_reason =
                MeasurementRejectionReason::kExpansionInvalid;
              const bool selected_same_floor = SelectExpandedSameFloorLayer(
                finite_ground, layer, layers, layer_index, radius_refs,
                options, &expanded_selected, &selection_reason);
              if (expanded_selected.size() >
                options.max_points_per_measurement_patch)
              {
                return finish_budget(
                  "expanded layer refs reached "
                  "max_points_per_measurement_patch (" +
                  std::to_string(
                    options.max_points_per_measurement_patch) + ")");
              }
              // The known minimum after this attempt adds the base/fit/own set
              // plus the actual attempt refs. Inliers and the expanded fit copy
              // are checked exactly after evaluation, before persistent vectors
              // are copied.
              std::size_t expansion_preflight =
                result.metrics.measurement_point_refs;
              const bool expansion_budget_ok =
                AddRefBudget(
                &expansion_preflight, layer.size(),
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &expansion_preflight, layer.size(),
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &expansion_preflight, base_own_count,
                options.max_measurement_point_refs) &&
                AddRefBudget(
                &expansion_preflight, expanded_selected.size(),
                options.max_measurement_point_refs);
              if (!expansion_budget_ok) {
                return finish_budget(
                  "expanded measurement refs exceed "
                  "max_measurement_point_refs (" +
                  std::to_string(options.max_measurement_point_refs) + ")");
              }
              expansion_attempt = EvaluateSurfaceAttempt(
                finite_ground, expanded_selected, center, cell, options);
              expansion_attempted = true;
              expansion_attempt_radius = radius;
              expansion_source =
                MeasurementExpansionSource::kInsufficientEvidence;
              ++result.metrics.expanded_patch_attempts;
              bool use_this_radius = false;
              if (!selected_same_floor) {
                expansion_reason = MeasurementRejectionReason::kExpansionInvalid;
                ++result.metrics.rejected_expansion_invalid;
              } else if (expansion_attempt.accepted &&
                expansion_attempt.max_depression_m >
                options.max_step_height_m + kEpsilon)
              {
                // Base ordinary/robust full-sample negatives remain diagnostic
                // only. A NEW expanded recovery must not silently bridge an
                // annulus depression that the base evidence could not see.
                expansion_reason = MeasurementRejectionReason::kLocalDepression;
              } else if (expansion_attempt.accepted) {
                const double coverage = SelectedSectorCoverage(
                  finite_ground, expanded_selected, center);
                if (classify_structure &&
                  coverage + kEpsilon <
                  options.global_surface_min_angular_coverage)
                {
                  expansion_reason =
                    MeasurementRejectionReason::kInsufficientCoverage;
                  ++result.metrics.rejected_adaptive_coverage;
                } else {
                  use_this_radius = true;
                }
              } else {
                expansion_reason = expansion_attempt.reason;
                if (expansion_reason == MeasurementRejectionReason::kNone) {
                  expansion_reason =
                    MeasurementRejectionReason::kDegenerateGeometry;
                }
              }

              if (use_this_radius) {
                expansion_used = true;
                expansion_used_radius = radius;
                expansion_reason = MeasurementRejectionReason::kNone;
                ++result.metrics.expanded_patch_recoveries;
                break;
              }
              if (!classify_structure) {
                break;
              }
            }
          }

          const SurfaceAttempt & selected_attempt =
            expansion_used ? expansion_attempt : base_attempt;
          const bool plane_kept = selected_attempt.plane.valid;

          // Exact stored-ref cost, including duplicated base/fit/own vectors and
          // any failed expansion attempt. This runs before copying into a patch.
          std::size_t patch_refs = 0U;
          bool patch_budget_ok =
            AddRefBudget(
            &patch_refs, layer.size(),
            options.max_measurement_point_refs) &&
            AddRefBudget(
            &patch_refs, selected_attempt.point_indices.size(),
            options.max_measurement_point_refs) &&
            AddRefBudget(
            &patch_refs, selected_attempt.inlier_indices.size(),
            options.max_measurement_point_refs) &&
            AddRefBudget(
            &patch_refs, base_attempt.own_cell_indices.size(),
            options.max_measurement_point_refs);
          if (expansion_attempted) {
            patch_budget_ok = patch_budget_ok &&
              AddRefBudget(
              &patch_refs, expansion_attempt.point_indices.size(),
              options.max_measurement_point_refs) &&
              AddRefBudget(
              &patch_refs, expansion_attempt.inlier_indices.size(),
              options.max_measurement_point_refs);
          }
          std::size_t total_refs = result.metrics.measurement_point_refs;
          patch_budget_ok = patch_budget_ok &&
            AddRefBudget(
            &total_refs, patch_refs, options.max_measurement_point_refs);
          if (!patch_budget_ok) {
            return finish_budget(
              "measurement point refs exceed max_measurement_point_refs (" +
              std::to_string(options.max_measurement_point_refs) + ")");
          }

          MeasurementPatch patch;
          patch.id = next_patch_id++;
          patch.cell = cell;
          patch.local_layer_index = static_cast<int>(layer_index);
          patch.base_point_indices = layer;
          patch.base_sample_count = layer.size();
          CopyFitAttempt(
            selected_attempt,
            expansion_used ?
            expansion_used_radius : options.neighborhood_radius_m,
            &patch.fit);
          patch.own_cell_point_indices = base_attempt.own_cell_indices;
          patch.own_cell_sample_count =
            base_attempt.own_cell_indices.size();
          patch.own_z_min = base_attempt.own_z_min;
          patch.own_z_max = base_attempt.own_z_max;
          if (base_attempt.own_cell_indices.empty()) {
            patch.own_cell_min_signed_residual =
              std::numeric_limits<double>::quiet_NaN();
            patch.own_cell_max_signed_residual =
              std::numeric_limits<double>::quiet_NaN();
            patch.own_cell_max_depression_m = 0.0;
            patch.own_cell_max_raised_m = 0.0;
          } else if (expansion_used) {
            double own_min = 0.0;
            double own_max = 0.0;
            if (!ComputeOwnCellResiduals(
                selected_attempt, finite_ground,
                base_attempt.own_cell_indices, &own_min, &own_max))
            {
              return finish(
                OfflineStaticMapStatus::kBuildFailed,
                "expanded own-cell residual reconstruction failed");
            }
            patch.own_cell_min_signed_residual = own_min;
            patch.own_cell_max_signed_residual = own_max;
            patch.own_cell_max_depression_m = std::max(0.0, -own_min);
            patch.own_cell_max_raised_m = std::max(0.0, own_max);
          } else if (base_attempt.plane.valid) {
            patch.own_cell_min_signed_residual =
              base_attempt.own_cell_min_signed_residual;
            patch.own_cell_max_signed_residual =
              base_attempt.own_cell_max_signed_residual;
            patch.own_cell_max_depression_m =
              base_attempt.own_cell_max_depression_m;
            patch.own_cell_max_raised_m =
              base_attempt.own_cell_max_raised_m;
          } else {
            patch.own_cell_min_signed_residual =
              std::numeric_limits<double>::quiet_NaN();
            patch.own_cell_max_signed_residual =
              std::numeric_limits<double>::quiet_NaN();
            patch.own_cell_max_depression_m = 0.0;
            patch.own_cell_max_raised_m = 0.0;
          }
          patch.expansion_attempted = expansion_attempted;
          patch.expansion_used = expansion_used;
          if (expansion_attempted) {
            CopyFitAttempt(
              expansion_attempt, expansion_attempt_radius, &patch.expansion);
            patch.expansion_reason = expansion_reason;
            patch.expansion_source = expansion_source;
            if (expansion_source ==
              MeasurementExpansionSource::kContextReestimation)
            {
              patch.context_original_reason = base_attempt.reason;
              patch.context_original_slope_rad = base_slope_rad;
            }
          }
          const std::vector<std::uint32_t> & final_refs =
            patch.fit.point_indices;
          patch.z_min = finite_ground[final_refs.front()].z;
          patch.z_max = patch.z_min;
          for (const std::uint32_t index : final_refs) {
            patch.z_min = std::min(patch.z_min, finite_ground[index].z);
            patch.z_max = std::max(patch.z_max, finite_ground[index].z);
          }

          if (base_attempt.robust_attempted) {
            ++result.metrics.robust_attempts;
            result.metrics.robust_refinement_iterations +=
              base_attempt.robust_iterations;
            if (!base_attempt.robust_geometry_accepted) {
              ++result.metrics.robust_rejected;
            }
          }
          if (base_attempt.ordinary_fit_valid &&
            !base_attempt.ordinary_rms_accepted)
          {
            ++result.metrics.ordinary_rms_rejections;
          }
          if (!base_attempt.ordinary_fit_valid) {
            ++result.metrics.ordinary_plane_failures;
          }

          bool accepted_node = false;
          bool slope_dropped = false;
          TerrainNode node;
          if (selected_attempt.accepted) {
            const double slope = std::atan(
              std::hypot(selected_attempt.plane.a, selected_attempt.plane.b));
            if (slope > options.max_slope_rad + kEpsilon) {
              if (options.mark_excessive_slope_prohibited) {
                node = make_node(selected_attempt, cell, center, true);
                accepted_node = true;
                patch.status = MeasurementPatchStatus::kProhibited;
                patch.reason = MeasurementRejectionReason::kExcessiveSlope;
              } else {
                slope_dropped = true;
                patch.status = MeasurementPatchStatus::kFitRejected;
                patch.reason = MeasurementRejectionReason::kExcessiveSlope;
              }
            } else {
              node = make_node(selected_attempt, cell, center, false);
              accepted_node = true;
              patch.status = MeasurementPatchStatus::kSupported;
              patch.reason = MeasurementRejectionReason::kNone;
            }
          } else {
            patch.status = plane_kept ?
              MeasurementPatchStatus::kFitRejected :
              MeasurementPatchStatus::kObservedOnly;
            patch.reason = base_attempt.reason;
          }
          patch.node_id = accepted_node ?
            node.id : kInvalidTerrainNodeId;

          if (accepted_node) {
            ++result.metrics.accepted_nodes;
            if (node.status == TerrainStatus::kProhibited) {
              ++result.metrics.prohibited_nodes;
            }
            if (selected_attempt.robust_used) {
              ++result.metrics.robust_recovered_nodes;
            }
            cell_nodes.push_back(std::move(node));
          } else if (slope_dropped) {
            ++result.metrics.rejected_prohibited_slope;
            Point3 example = center;
            example.z = selected_attempt.plane.a * center.x +
              selected_attempt.plane.b * center.y + selected_attempt.plane.c;
            RecordGateExample(
              &result.metrics, "prohibited_slope", cell, example,
              std::atan(
                std::hypot(
                  selected_attempt.plane.a, selected_attempt.plane.b)),
              options.max_slope_rad);
          } else {
            record_rejection(base_attempt, cell, center);
            if (base_attempt.robust_attempted &&
              base_attempt.robust_geometry_accepted)
            {
              ++result.metrics.robust_rejected;
            }
          }

          const std::size_t final_raised_count =
            expansion_source ==
            MeasurementExpansionSource::kContextReestimation ?
            (expansion_used ? expansion_attempt : base_attempt).
            raised_conflict_sample_count :
            base_attempt.raised_conflict_sample_count +
            (expansion_attempted ?
            expansion_attempt.raised_conflict_sample_count : 0U);
          if (final_raised_count > 0U) {
            ++result.metrics.patches_with_raised_evidence;
          }
          const TerrainNodeId raised_source = accepted_node ?
            patch.node_id : kInvalidTerrainNodeId;
          const auto append_raised = [&](const SurfaceAttempt & attempt) {
              for (const std::uint32_t index :
                attempt.raised_conflict_indices)
              {
                const Point3 & point = finite_ground[index];
                double source_ground_z = 0.0;
                if (!RaisedSourceGroundZ(
                    attempt.plane, point, &source_ground_z))
                {
                  continue;
                }
                RaisedEvidence raised;
                raised.point_index = index;
                raised.source_node = raised_source;
                raised.source_plane_valid = true;
                raised.source_centroid = attempt.plane.centroid;
                raised.source_normal = attempt.plane.normal;
                raised.source_ground_z = source_ground_z;
                raised_evidence.push_back(std::move(raised));
              }
            };
          if (expansion_source ==
            MeasurementExpansionSource::kContextReestimation)
          {
            // Planning obstacle association follows the selected final model.
            // A failed context attempt keeps the base model and never exports
            // the unused broader attempt's positive residuals.
            append_raised(expansion_used ? expansion_attempt : base_attempt);
          } else {
            append_raised(base_attempt);
            if (expansion_attempted) {
              append_raised(expansion_attempt);
            }
          }

          result.metrics.measurement_point_refs = total_refs;
          patches.push_back(std::move(patch));
        }
        if (cell_nodes.empty()) {
          ++result.metrics.unknown_candidate_cells;
          continue;
        }
        // Stable local layer order is required by the snapshot contract. The
        // ordinal is local to this cell and never a global floor identity.
        std::sort(
          cell_nodes.begin(), cell_nodes.end(),
          [](const TerrainNode & lhs, const TerrainNode & rhs) {
            if (lhs.representative.z != rhs.representative.z) {
              return lhs.representative.z < rhs.representative.z;
            }
            return lhs.id < rhs.id;
          });
        for (std::size_t index = 0; index < cell_nodes.size(); ++index) {
          cell_nodes[index].local_surface_index =
            static_cast<int>(index);
          nodes.push_back(std::move(cell_nodes[index]));
        }
      }
    }
  }  // end non-continuous legacy per-cell branch
  result.metrics.supported_nodes = static_cast<std::size_t>(
    std::count_if(
      nodes.begin(), nodes.end(),
      [](const TerrainNode & node) {return node.IsSupported();}));
  result.metrics.terrain_ms =
    DurationToMs(std::chrono::steady_clock::now() - terrain_started);
  result.metrics.measurement_patches = patches.size();

  if (nodes.empty()) {
    // Keep the complete rejected measurement evidence queryable while the
    // planner path remains failclosed: no TerrainSnapshot/MapSnapshot is built.
    MeasurementEvidence diagnostic;
    diagnostic.points = std::move(finite_ground);
    diagnostic.patches = std::move(patches);
    result.diagnostic_evidence =
      std::make_shared<const MeasurementEvidence>(std::move(diagnostic));
    return finish(
      OfflineStaticMapStatus::kBuildFailed,
      "no measured surface passed the offline support/rejection "
      "gates");
  }

  // 2b. Ground-only bounded continuity pass. It runs before connections,
  // obstacles and snapshot publication so every downstream consumer sees the
  // same reconstructed node plane and mirrored patch provenance. Global-cloud
  // model classification and the disabled builder default are untouched. The
  // canonical raw points/patch.fit are never rewritten. Continuous regional
  // reconstruction supersedes this local smoother so welded corners are not
  // later re-split by a conflicting per-node correction.
  if (reconstruct_ground_surface && !continuous_ground_active) {
    GroundSurfaceReconstructionOptions reconstruction_options =
      options.ground_surface;
    reconstruction_options.max_candidate_slope_rad = std::min(
      reconstruction_options.max_candidate_slope_rad, options.max_slope_rad);
    const GroundSurfaceReconstructionMetrics reconstruction =
      ReconstructGroundSurfaces(
      nodes, patches, reconstruction_options, options.resolution,
      &nodes, &patches);
    result.metrics.surface_reconstruction_considered =
      reconstruction.considered_nodes;
    result.metrics.surface_reconstruction_applied =
      reconstruction.applied_nodes;
    result.metrics.surface_reconstruction_preserved_discontinuity =
      reconstruction.preserved_discontinuities;
    result.metrics.surface_reconstruction_rejected_support =
      reconstruction.rejected_insufficient_support;
    result.metrics.surface_reconstruction_rejected_correction =
      reconstruction.rejected_correction;
    result.metrics.surface_reconstruction_rejected_normal =
      reconstruction.rejected_normal;
    result.metrics.surface_reconstruction_rejected_evidence =
      reconstruction.rejected_evidence;
    result.metrics.surface_reconstruction_max_correction_m =
      reconstruction.max_applied_correction_m;
  }

  // 3. Evaluate adjacent-cell candidate links with the same height, slope,
  //    normal and directional-boundary gates as the live terrain model. Only
  //    adjacent cells are paired, so a bridge deck cannot connect to the road
  //    below through an unrelated cell.
  const auto connection_started = std::chrono::steady_clock::now();
  std::unordered_map<FineCellIndex, std::vector<std::size_t>,
    FineCellIndexHash>
  cell_to_nodes;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    cell_to_nodes[nodes[index].cell].push_back(index);
  }
  std::vector<std::vector<Point2>> support_hulls(nodes.size());
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    std::vector<Point2> xy;
    xy.reserve(nodes[index].support.points.size());
    for (const Point3 & point : nodes[index].support.points) {
      xy.push_back(Point2{point.x, point.y});
    }
    support_hulls[index] = ConvexHull(std::move(xy));
  }
  std::vector<TerrainConnection> connections;
  constexpr std::array<std::array<int, 2>, 4> kNeighborOffsets{{
    {{1, 0}}, {{0, 1}}, {{1, 1}}, {{-1, 1}}}};
  const double cos_normal_limit =
    std::cos(options.max_normal_angle_rad);
  for (std::size_t source_index = 0; source_index < nodes.size();
    ++source_index)
  {
    const TerrainNode & source = nodes[source_index];
    for (const auto & offset : kNeighborOffsets) {
      const FineCellIndex neighbor{
        source.cell.x + offset[0], source.cell.y + offset[1]};
      const auto iterator = cell_to_nodes.find(neighbor);
      if (iterator == cell_to_nodes.end()) {
        continue;
      }
      for (const std::size_t target_index : iterator->second) {
        const TerrainNode & target = nodes[target_index];
        TerrainConnection connection;
        connection.source = source.id;
        connection.target = target.id;
        const double center_distance =
          std::hypot(
          target.representative.x - source.representative.x,
          target.representative.y - source.representative.y);
        const double height_delta =
          target.representative.z - source.representative.z;
        // The stored summary remains centre-to-centre for diagnostics. The
        // continuity gate below uses the measured shared-boundary step.
        const double slope =
          center_distance > kEpsilon ?
          std::atan2(std::abs(height_delta), center_distance) :
          std::numeric_limits<double>::infinity();
        const double normal_dot = std::clamp(
          Dot(source.plane.normal, target.plane.normal), -1.0, 1.0);
        const SharedBoundary boundary =
          SharedBoundaryBetween(source, target, options.resolution);
        const bool reconstructed_pair =
          source.surface_reconstruction.applied &&
          source.surface_reconstruction.shared_corners_valid &&
          target.surface_reconstruction.applied &&
          target.surface_reconstruction.shared_corners_valid;
        const BoundaryContinuity continuity = reconstructed_pair ?
          EvaluateReconstructedContinuity(source, target, boundary) :
          EvaluateBoundaryContinuity(
          source, target, boundary,
          support_hulls[source_index], support_hulls[target_index],
          options);
        connection.height_delta = height_delta;
        connection.horizontal_distance = center_distance;
        connection.slope = slope;
        connection.normal_dot = normal_dot;
        connection.boundary_coverage = continuity.coverage;
        connection.status = ConnectionStatus::kConnected;
        connection.reason = ConnectionReason::kNone;
        if (!std::isfinite(center_distance) ||
          !std::isfinite(height_delta) || !std::isfinite(slope) ||
          !std::isfinite(normal_dot))
        {
          connection.status = ConnectionStatus::kInsufficientEvidence;
          connection.reason = ConnectionReason::kNonFinite;
        } else if (!source.IsSupported() || !target.IsSupported()) {
          connection.status = ConnectionStatus::kDiscontinuous;
          connection.reason = ConnectionReason::kProhibitedSurface;
        } else if (continuity.supported &&
          continuity.step_height > options.max_step_height_m + kEpsilon)
        {
          connection.status = ConnectionStatus::kDiscontinuous;
          connection.reason = ConnectionReason::kHeightStep;
        } else if (slope > options.max_slope_rad + kEpsilon) {
          connection.status = ConnectionStatus::kDiscontinuous;
          connection.reason = ConnectionReason::kSlopeLimit;
        } else if (normal_dot < cos_normal_limit - kEpsilon) {
          connection.status = ConnectionStatus::kInconsistentNormals;
          connection.reason = ConnectionReason::kNormalMismatch;
        } else if (!continuity.supported ||
          (!reconstructed_pair &&
          connection.boundary_coverage <
          options.min_boundary_coverage - kEpsilon))
        {
          connection.status = ConnectionStatus::kInsufficientEvidence;
          connection.reason = ConnectionReason::kBoundaryCoverage;
        }
        connections.push_back(connection);
      }
    }
  }
  result.metrics.connections = connections.size();
  result.metrics.connection_ms =
    DurationToMs(std::chrono::steady_clock::now() - connection_started);

  // 3b. Full-cloud semantic split. The measured surface graph above contains
  // every accepted planar layer, including overhead decks and horizontal box
  // tops. A component is confirmed ground only when it is continuous with the
  // lowest measured surface or when an optional auxiliary ground hint matches
  // its fitted height. Unseeded components are retained as unknown geometry,
  // not fabricated box tops: geometry alone cannot prove the difference. They
  // are never joined to the road by same-XY or height-only shortcuts. This
  // classification is deliberately independent of wheel/vehicle feasibility.
  std::vector<std::size_t> unconfirmed_node_indices;
  if (classify_structure && !nodes.empty()) {
    std::unordered_map<TerrainNodeId, std::size_t> id_to_node_index;
    id_to_node_index.reserve(nodes.size());
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      id_to_node_index[nodes[index].id] = index;
    }
    std::vector<std::size_t> parent(nodes.size());
    std::vector<std::size_t> rank(nodes.size(), 0U);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      parent[index] = index;
    }
    const auto find_root = [&](std::size_t index) {
        std::size_t root = index;
        while (parent[root] != root) {
          root = parent[root];
        }
        while (parent[index] != index) {
          const std::size_t next = parent[index];
          parent[index] = root;
          index = next;
        }
        return root;
      };
    const auto unite = [&](std::size_t first, std::size_t second) {
        std::size_t first_root = find_root(first);
        std::size_t second_root = find_root(second);
        if (first_root == second_root) {
          return;
        }
        if (rank[first_root] < rank[second_root]) {
          std::swap(first_root, second_root);
        }
        parent[second_root] = first_root;
        if (rank[first_root] == rank[second_root]) {
          ++rank[first_root];
        }
      };
    for (const TerrainConnection & connection : connections) {
      if (!connection.IsConnected()) {
        continue;
      }
      const auto source = id_to_node_index.find(connection.source);
      const auto target = id_to_node_index.find(connection.target);
      if (source != id_to_node_index.end() &&
        target != id_to_node_index.end())
      {
        unite(source->second, target->second);
      }
    }

    std::unordered_map<FineCellIndex, std::vector<Point3>, FineCellIndexHash>
    hint_points_by_cell;
    if (ground_hints != nullptr) {
      hint_points_by_cell.reserve(ground_hints->size());
      for (const Point3 & point : *ground_hints) {
        if (!point.IsFinite()) {
          continue;
        }
        const auto cell = FineCellOf(
          point, options.origin_x, options.origin_y, options.resolution);
        if (cell.has_value()) {
          hint_points_by_cell[*cell].push_back(point);
        }
      }
    }

    std::unordered_map<TerrainNodeId, std::size_t> patch_index_by_node;
    patch_index_by_node.reserve(patches.size());
    for (std::size_t index = 0; index < patches.size(); ++index) {
      if (patches[index].node_id != kInvalidTerrainNodeId) {
        patch_index_by_node[patches[index].node_id] = index;
      }
    }
    double global_min_supported_z = std::numeric_limits<double>::infinity();
    for (const TerrainNode & node : nodes) {
      if (node.IsSupported()) {
        global_min_supported_z =
          std::min(global_min_supported_z, node.representative.z);
      }
    }
    if (std::isfinite(global_min_supported_z)) {
      std::vector<double> component_min_z(
        nodes.size(), std::numeric_limits<double>::infinity());
      std::vector<char> has_node(nodes.size(), 0);
      std::vector<char> seeded(nodes.size(), 0);
      for (const TerrainNode & node : nodes) {
        const std::size_t root = find_root(
          id_to_node_index.at(node.id));
        has_node[root] = 1;
        if (node.IsSupported()) {
          component_min_z[root] =
            std::min(component_min_z[root], node.representative.z);
        }
        const auto hint_iterator = hint_points_by_cell.find(node.cell);
        if (hint_iterator != hint_points_by_cell.end()) {
          const auto patch_iterator = patch_index_by_node.find(node.id);
          if (patch_iterator != patch_index_by_node.end()) {
            const MeasurementFitAttempt & fit =
              patches[patch_iterator->second].fit;
            for (const Point3 & hint : hint_iterator->second) {
              const double residual = FitPlaneHeightResidual(fit, hint);
              if (std::isfinite(residual) &&
                std::abs(residual) <=
                options.global_hint_surface_tolerance_m + kEpsilon)
              {
                seeded[root] = 1;
                break;
              }
            }
          }
        }
      }
      const double seed_limit =
        global_min_supported_z + options.global_ground_seed_tolerance_m;
      for (std::size_t index = 0; index < nodes.size(); ++index) {
        const std::size_t root = find_root(index);
        if (!seeded[root] && component_min_z[root] <= seed_limit + kEpsilon) {
          seeded[root] = 1;
        }
      }
      for (std::size_t index = 0; index < nodes.size(); ++index) {
        const std::size_t root = find_root(index);
        if (!has_node[root]) {
          continue;
        }
        if (seeded[root]) {
          continue;
        }
        TerrainNode & node = nodes[index];
        if (node.IsSupported()) {
          // Preserve the fitted surface and its refs, but do not claim either
          // ground or a box top. Search/validation ignore unknown nodes.
          node.status = TerrainStatus::kUnknown;
          node.reason = ProhibitedReason::kUnconfirmedComponent;
          unconfirmed_node_indices.push_back(index);
          const auto patch_iterator =
            patch_index_by_node.find(node.id);
          if (patch_iterator != patch_index_by_node.end()) {
            MeasurementPatch & patch = patches[patch_iterator->second];
            patch.status = MeasurementPatchStatus::kUnconfirmed;
            patch.reason =
              MeasurementRejectionReason::kUnconfirmedComponent;
          }
        }
      }
      std::vector<char> counted(nodes.size(), 0);
      for (std::size_t index = 0; index < nodes.size(); ++index) {
        const std::size_t root = find_root(index);
        if (counted[root] != 0) {
          continue;
        }
        counted[root] = 1;
        if (seeded[root]) {
          ++result.metrics.ground_components;
        } else {
          ++result.metrics.unconfirmed_components;
        }
      }
      result.metrics.unconfirmed_nodes = unconfirmed_node_indices.size();
      // Geometry remains recorded, but reachability from a confirmed seed is
      // unknown rather than a fabricated prohibited surface.
      for (TerrainConnection & connection : connections) {
        if (!connection.IsConnected()) {
          continue;
        }
        const auto source = id_to_node_index.find(connection.source);
        const auto target = id_to_node_index.find(connection.target);
        if (source != id_to_node_index.end() &&
          !nodes[source->second].IsSupported())
        {
          connection.status = ConnectionStatus::kInsufficientEvidence;
          connection.reason = ConnectionReason::kUnconfirmedComponent;
        } else if (target != id_to_node_index.end() &&
          !nodes[target->second].IsSupported())
        {
          connection.status = ConnectionStatus::kInsufficientEvidence;
          connection.reason = ConnectionReason::kUnconfirmedComponent;
        }
      }
    }
  }
  result.metrics.supported_nodes = static_cast<std::size_t>(
    std::count_if(
      nodes.begin(), nodes.end(),
      [](const TerrainNode & node) {return node.IsSupported();}));
  result.metrics.prohibited_nodes = static_cast<std::size_t>(
    std::count_if(
      nodes.begin(), nodes.end(),
      [](const TerrainNode & node) {
        return node.status == TerrainStatus::kProhibited;
      }));

  // 4. Build the offline static obstacle layer. Positive measured residual refs
  //    are combined with explicit obstacles and associated to the nearest
  //    supported surface; no support node is deleted by this step.
  const auto obstacle_started = std::chrono::steady_clock::now();
  std::unordered_map<TerrainNodeId, ObstacleAttributes> attributes;
  std::sort(
    raised_evidence.begin(), raised_evidence.end(), RaisedEvidenceLess);
  raised_evidence.erase(
    std::unique(
      raised_evidence.begin(), raised_evidence.end(), SameRaisedEvidence),
    raised_evidence.end());
  const std::int64_t association_halo = static_cast<std::int64_t>(
    std::ceil(
      options.obstacle_association_radius_m /
      options.resolution));
  // Marks one concrete supported node occupied. This is the only occupancy
  // mutation; callers decide which node corresponds to the actual point.
  const auto mark_occupied_node = [&](const TerrainNode & node) {
      ObstacleAttributes & record = attributes[node.id];
      record.static_state = ObstacleState::kOccupied;
      record.static_distance_m = 0.0;
      record.occupied_vertical_extent_known = false;
      record.observation_stamp_ns = 0;
      record.received_steady_ns = 0;
    };
  // Applies the existing duplicate/blocking-height policy to one concrete
  // supported surface. This remains the exact policy for explicit obstacles.
  const auto mark_point_for_node = [&](const TerrainNode & node,
      const Point3 & point) {
      if (!point.IsFinite() || !node.IsSupported()) {
        return;
      }
      const double ground_z = PlaneHeightAt(node, point.x, point.y);
      if (!std::isfinite(ground_z)) {
        return;
      }
      const double delta = point.z - ground_z;
      if (std::abs(delta) <= options.ground_duplicate_tolerance_m) {
        ++result.metrics.duplicate_ground_obstacles;
        return;
      }
      if (delta <= 0.0) {
        return;
      }
      if (delta > options.obstacle_blocking_height_m) {
        ++result.metrics.ignored_overhead_obstacles;
        return;
      }
      mark_occupied_node(node);
    };
  const auto associate_point = [&](const Point3 & point) {
      if (!point.IsFinite()) {
        return;
      }
      const auto point_cell = FineCellOf(
        point, options.origin_x,
        options.origin_y,
        options.resolution);
      if (!point_cell.has_value()) {
        return;
      }
      const TerrainNode * best_node = nullptr;
      double best_delta = std::numeric_limits<double>::infinity();
      double best_xy = std::numeric_limits<double>::infinity();
      for (std::int64_t dx = -association_halo; dx <= association_halo;
        ++dx)
      {
        for (std::int64_t dy = -association_halo; dy <= association_halo;
          ++dy)
        {
          const FineCellIndex candidate_cell{
            point_cell->x + dx, point_cell->y + dy};
          const auto iterator = cell_to_nodes.find(candidate_cell);
          if (iterator == cell_to_nodes.end()) {
            continue;
          }
          for (const std::size_t index : iterator->second) {
            const TerrainNode & node = nodes[index];
            if (!node.IsSupported()) {
              continue;
            }
            const double ground_z = PlaneHeightAt(node, point.x, point.y);
            if (!std::isfinite(ground_z)) {
              continue;
            }
            const double signed_delta = point.z - ground_z;
            if (signed_delta < -options.ground_duplicate_tolerance_m) {
              continue;
            }
            const double xy_distance =
              std::hypot(
              point.x - node.representative.x,
              point.y - node.representative.y);
            if (xy_distance > options.obstacle_association_radius_m +
              options.resolution)
            {
              continue;
            }
            const double delta = std::abs(signed_delta);
            // Occupancy belongs at the point's horizontal position. Height
            // breaks ties only between equally close XY surfaces.
            if (xy_distance < best_xy ||
              (xy_distance == best_xy && delta < best_delta))
            {
              best_delta = delta;
              best_xy = xy_distance;
              best_node = &node;
            }
          }
        }
      }
      if (best_node == nullptr) {
        return;
      }
      mark_point_for_node(*best_node, point);
    };
  // Generated raised evidence keeps both its measured source surface and its
  // actual XY. Occupancy is only emitted for a supported node in the point's
  // own cell whose source-plane ground height matches the measuring surface.
  // This avoids fit-radius dilation and target-fit duplicate absorption; if no
  // corresponding point-location node exists, the evidence remains unknown.
  for (const RaisedEvidence & evidence : raised_evidence) {
    const Point3 & point = finite_ground[evidence.point_index];
    if (!evidence.source_plane_valid ||
      !std::isfinite(evidence.source_ground_z))
    {
      continue;
    }
    const auto point_cell = FineCellOf(
      point, options.origin_x, options.origin_y, options.resolution);
    if (!point_cell.has_value()) {
      continue;
    }
    const double source_ground_z = evidence.source_ground_z;
    const double source_delta = point.z - source_ground_z;
    if (std::abs(source_delta) <= options.ground_duplicate_tolerance_m) {
      ++result.metrics.duplicate_ground_obstacles;
      continue;
    }
    if (source_delta <= 0.0) {
      continue;
    }
    if (source_delta > options.obstacle_blocking_height_m) {
      ++result.metrics.ignored_overhead_obstacles;
      continue;
    }
    const auto iterator = cell_to_nodes.find(*point_cell);
    if (iterator == cell_to_nodes.end()) {
      continue;
    }
    const TerrainNode * target = nullptr;
    double best_height_delta = std::numeric_limits<double>::infinity();
    for (const std::size_t index : iterator->second) {
      const TerrainNode & candidate = nodes[index];
      if (!candidate.IsSupported()) {
        continue;
      }
      const double candidate_ground_z =
        PlaneHeightAt(candidate, point.x, point.y);
      if (!std::isfinite(candidate_ground_z)) {
        continue;
      }
      const double height_delta =
        std::abs(candidate_ground_z - source_ground_z);
      if (height_delta > options.max_step_height_m + kEpsilon) {
        continue;
      }
      if (target == nullptr || height_delta < best_height_delta ||
        (height_delta == best_height_delta && candidate.id < target->id))
      {
        target = &candidate;
        best_height_delta = height_delta;
      }
    }
    if (target != nullptr) {
      mark_occupied_node(*target);
    }
  }
  for (const Point3 & point : obstacle_points) {
    associate_point(point);
  }
  // Give every supported node an explicit record in the offline static-map
  // model. A node without an assigned obstacle point is "clear at this 2.5-D
  // map node", not observed free volume: the vertical observed extent remains
  // unknown and no headroom is claimed.
  for (const TerrainNode & node : nodes) {
    if (!node.IsSupported()) {
      continue;
    }
    ObstacleAttributes & record = attributes[node.id];
    if (record.static_state != ObstacleState::kOccupied) {
      record.static_state = ObstacleState::kClear;
      record.static_distance_m = 0.0;
    }
    record.observed_vertical_extent_known = false;
    record.clearance_known = false;
    record.occupied_vertical_extent_known = false;
  }
  // Unconfirmed surfaces are real measured geometry. Keep one occupied record
  // per retained component with its measured vertical extent so box tops and
  // overhead slabs remain explicit obstacles instead of disappearing with the
  // ground classification. This does not make them traversable; the offline
  // evaluator still selects only supported nodes.
  for (const std::size_t index : unconfirmed_node_indices) {
    if (index >= nodes.size()) {
      continue;
    }
    const TerrainNode & node = nodes[index];
    if (node.status != TerrainStatus::kUnknown) {
      continue;
    }
    ObstacleAttributes & record = attributes[node.id];
    record.static_state = ObstacleState::kOccupied;
    record.static_distance_m = 0.0;
    record.occupied_vertical_extent_known =
      node.support.z_min <= node.support.z_max;
    record.occupied_min_z = node.support.z_min;
    record.occupied_max_z = node.support.z_max;
    record.observation_stamp_ns = 0;
    record.received_steady_ns = 0;
  }
  result.metrics.obstacle_records = attributes.size();

  // 5. Publish the same immutable MapSnapshot type consumed by search and
  //    validation. The full measurement evidence shares the TerrainSnapshot
  //    with obstacle COW snapshots; it is never copied per obstacle update.
  const auto snapshot_started = std::chrono::steady_clock::now();
  MeasurementEvidence evidence;
  evidence.points = std::move(finite_ground);
  evidence.patches = std::move(patches);
  result.metrics.measurement_points = evidence.points.size();
  result.metrics.measurement_patches = evidence.patches.size();
  const std::uint64_t content_hash = HashBuildContent(
    options, frame_id, nodes, connections, evidence);
  std::shared_ptr<const TerrainSnapshot> terrain;
  try {
    terrain = std::make_shared<const TerrainSnapshot>(
      std::move(nodes), std::move(connections),
      TerrainSnapshotMetadata{frame_id, options.resolution,
        options.origin_x, options.origin_y, 1U,
        content_hash, 0, 0, 1U},
      std::move(evidence));
  } catch (const std::exception & exception) {
    return finish(
      OfflineStaticMapStatus::kBuildFailed,
      std::string("TerrainSnapshot construction failed: ") +
      exception.what());
  }
  ObstacleTileLayer static_tiles = ObstacleTileLayer::FromFlat(
    *terrain, attributes,
    ObstacleTileOptions{options.obstacle_tile_size_m, false}, 0, 0, 1U,
    true);
  result.metrics.obstacle_ms =
    DurationToMs(std::chrono::steady_clock::now() - obstacle_started);
  ObstacleDistanceOptions distance_options;
  distance_options.truncation_distance_m = 2.0;
  distance_options.reject_unknown = false;
  distance_options.reject_prohibited = true;
  try {
    result.snapshot = MapSnapshot::Create(
      terrain, nullptr, ObstacleLayer(std::move(static_tiles)),
      ObstacleLayer(), 1U, distance_options);
  } catch (const std::exception & exception) {
    return finish(
      OfflineStaticMapStatus::kBuildFailed,
      std::string("MapSnapshot::Create failed: ") +
      exception.what());
  }
  result.metrics.snapshot_ms =
    DurationToMs(std::chrono::steady_clock::now() - snapshot_started);
  return finish(OfflineStaticMapStatus::kOk, "ok");
}  // NOLINT(readability/fn_size): single-pass build; split would duplicate traversal state

}  // namespace

OfflineStaticMapResult BuildOfflineStaticMap(
  const std::vector<Point3> & ground_points,
  const std::vector<Point3> & obstacle_points,
  const OfflineStaticMapOptions & options,
  const std::string & frame_id)
{
  try {
    return BuildOfflineStaticMapImpl(
      ground_points, obstacle_points, options, frame_id, nullptr, false);
  } catch (const std::bad_alloc &) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = "measurement evidence allocation failed";
    return result;
  } catch (const std::length_error &) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = "measurement evidence allocation failed";
    return result;
  } catch (const std::exception & exception) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = std::string("offline static-map build failed: ") +
      exception.what();
    return result;
  }
}

OfflineStaticMapResult BuildOfflineGlobalCloudMap(
  const std::vector<Point3> & global_points,
  const std::vector<Point3> & auxiliary_ground_points,
  const std::vector<Point3> & obstacle_points,
  const OfflineStaticMapOptions & options,
  const std::string & frame_id)
{
  try {
    // The auxiliary hint is also a real measured sample source; exact
    // duplicates are removed by the canonical table. It never replaces the
    // global cloud and may be empty.
    std::vector<Point3> surface_points = global_points;
    surface_points.reserve(
      global_points.size() + auxiliary_ground_points.size());
    surface_points.insert(
      surface_points.end(),
      auxiliary_ground_points.begin(), auxiliary_ground_points.end());
    // Global geometry always participates in obstacle/clearance evidence. An
    // extra obstacle cloud is additive; it never replaces the global cloud.
    std::vector<Point3> merged_obstacles;
    const std::vector<Point3> * obstacles = &global_points;
    if (!obstacle_points.empty() && &obstacle_points != &global_points) {
      merged_obstacles.reserve(
        global_points.size() + obstacle_points.size());
      merged_obstacles.insert(
        merged_obstacles.end(), global_points.begin(), global_points.end());
      merged_obstacles.insert(
        merged_obstacles.end(), obstacle_points.begin(),
        obstacle_points.end());
      obstacles = &merged_obstacles;
    }
    return BuildOfflineStaticMapImpl(
      surface_points, *obstacles, options, frame_id,
      &auxiliary_ground_points, true);
  } catch (const std::bad_alloc &) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = "global-cloud surface allocation failed";
    return result;
  } catch (const std::length_error &) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = "global-cloud surface allocation failed";
    return result;
  } catch (const std::exception & exception) {
    OfflineStaticMapResult result;
    result.status = OfflineStaticMapStatus::kBuildFailed;
    result.detail = std::string("global-cloud surface build failed: ") +
      exception.what();
    return result;
  }
}

}  // namespace dddmr_grid_global_planner
