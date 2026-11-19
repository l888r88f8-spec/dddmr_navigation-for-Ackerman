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

#include "grid_global_planner/terrain_grid_builder.h"

#include "grid_global_planner/continuous_ground_surface.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>

namespace grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr std::uint64_t kBuilderSchemaVersion = 2;

// Maximum points per side used by the deterministic boundary-continuity probe.
// The stride sample keeps the check bounded on unusually dense fine cells while
// still observing both surfaces in deterministic lexicographic order.
constexpr std::size_t kMaxBoundarySamples = 512;

// Bounded two-model region-growth iterations. The loop is deliberately small:
// it only needs to separate the low/high seeds, not converge an arbitrary
// clustering, and every iteration is deterministic over lexicographic points.
constexpr int kMaxRegionGrowingIterations = 4;

// A split is accepted as "decisive" without strong cross-boundary support only
// when the observed height gap is this multiple of refine_split_min_gap and the
// parent surface is already too rough to be supported.
constexpr double kDecisiveGapFactor = 2.0;

// The low-RMS split path requires the two surface means to be this many parent
// RMS values apart. This keeps random roughness from being fragmented into
// artificial decks while still exposing a real sub-cell curb.
constexpr double kSeparatedMeanFactor = 1.5;

// Deterministic ordering for cell keys.
struct CellIndexLess
{
  bool operator()(const FineCellIndex & lhs, const FineCellIndex & rhs) const noexcept
  {
    return lhs < rhs;
  }
};

using CellMap = std::map<FineCellIndex, std::vector<Point3>, CellIndexLess>;

// Adds the wall time of one FitHeightPlane call to a caller-owned accumulator.
// The accumulator is local to one Build() call, so no global metric state or
// lock is introduced. Nested fitting is therefore measured, not estimated.
class FitWallTimer
{
public:
  FitWallTimer(double * accumulator, std::chrono::steady_clock::time_point start)
  : accumulator_(accumulator), start_(start) {}
  ~FitWallTimer()
  {
    if (accumulator_ != nullptr) {
      *accumulator_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_)
        .count();
    }
  }
  FitWallTimer(const FitWallTimer &) = delete;
  FitWallTimer & operator=(const FitWallTimer &) = delete;

private:
  double * accumulator_;
  std::chrono::steady_clock::time_point start_;
};

// Lexicographic point ordering. All points are finite by the time this runs.
bool PointLess(const Point3 & lhs, const Point3 & rhs) noexcept
{
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  if (lhs.y != rhs.y) {
    return lhs.y < rhs.y;
  }
  return lhs.z < rhs.z;
}

// Compares two layers by mean height and then by their smallest point, making
// local surface indices stable across input orderings.
double MeanZ(const std::vector<Point3> & points) noexcept
{
  if (points.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const Point3 & point : points) {
    sum += point.z;
  }
  return sum / static_cast<double>(points.size());
}

bool LayerLess(
  const std::vector<Point3> & lhs, const std::vector<Point3> & rhs) noexcept
{
  const double lhs_mean = MeanZ(lhs);
  const double rhs_mean = MeanZ(rhs);
  if (lhs_mean != rhs_mean) {
    return lhs_mean < rhs_mean;
  }
  if (lhs.empty() || rhs.empty()) {
    return lhs.size() < rhs.size();
  }
  return PointLess(lhs.front(), rhs.front());
}

// Appends raw bytes to an FNV-1a hash.
void HashBytes(
  std::uint64_t * hash, const void * data, std::size_t size) noexcept
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= kFnvPrime;
  }
}

// Appends a double by its exact bit pattern. The same platform and deterministic
// point ordering therefore produce the same content hash after reordering.
void HashDouble(std::uint64_t * hash, double value) noexcept
{
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  HashBytes(hash, &bits, sizeof(bits));
}

// Appends an unsigned 64-bit value.
void HashUint64(std::uint64_t * hash, std::uint64_t value) noexcept
{
  HashBytes(hash, &value, sizeof(value));
}

// Appends a length-prefixed string so different frame names cannot alias.
void HashString(std::uint64_t * hash, const std::string & value) noexcept
{
  HashUint64(hash, static_cast<std::uint64_t>(value.size()));
  HashBytes(hash, value.data(), value.size());
}

// Splits a sorted-by-Z point list wherever the adjacent height gap exceeds the
// threshold. This is only the initial layer pre-segmentation.
std::vector<std::vector<Point3>> SplitByZGap(
  const std::vector<Point3> & points, double gap_threshold)
{
  std::vector<Point3> by_z = points;
  std::stable_sort(
    by_z.begin(), by_z.end(),
    [](const Point3 & lhs, const Point3 & rhs) {
      if (lhs.z != rhs.z) {
        return lhs.z < rhs.z;
      }
      return PointLess(lhs, rhs);
    });
  std::vector<std::vector<Point3>> layers;
  layers.emplace_back();
  for (const Point3 & point : by_z) {
    if (!layers.back().empty() &&
      point.z - layers.back().back().z > gap_threshold)
    {
      layers.emplace_back();
    }
    layers.back().push_back(point);
  }
  return layers;
}

// Fits z = a*x + b*y + c by ordinary least squares. This height-field model is
// intentional: near-vertical/cliff faces are not drivable surfaces and are kept
// unknown instead of receiving a fabricated normal. Returns false when the XY
// support is degenerate (for example all points share one XY line).
bool FitHeightPlane(
  const std::vector<Point3> & points, double inlier_tolerance,
  PlaneEvidence * evidence, double * fit_ms = nullptr)
{
  const FitWallTimer fit_timer(
    fit_ms, std::chrono::steady_clock::now());
  if (evidence == nullptr) {
    return false;
  }
  *evidence = PlaneEvidence{};
  evidence->sample_count = points.size();
  if (points.empty()) {
    return false;
  }
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (const Point3 & point : points) {
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }
  const double count = static_cast<double>(points.size());
  const double centroid_x = sum_x / count;
  const double centroid_y = sum_y / count;
  const double centroid_z = sum_z / count;
  evidence->centroid = Point3{centroid_x, centroid_y, centroid_z};

  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  double sxz = 0.0;
  double syz = 0.0;
  for (const Point3 & point : points) {
    const double dx = point.x - centroid_x;
    const double dy = point.y - centroid_y;
    const double dz = point.z - centroid_z;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
    sxz += dx * dz;
    syz += dy * dz;
  }

  double matrix[3][4] = {
    {sxx, sxy, 0.0, sxz},
    {sxy, syy, 0.0, syz},
    {0.0, 0.0, count, 0.0},
  };
  // The third row/column encodes the constant term. The system is solved in
  // centred coordinates and shifted back afterwards.
  matrix[0][2] = 0.0;
  matrix[1][2] = 0.0;
  matrix[2][0] = 0.0;
  matrix[2][1] = 0.0;
  matrix[2][3] = 0.0;

  // Gaussian elimination with partial pivoting.
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    double pivot_abs = std::abs(matrix[pivot][column]);
    for (int row = column + 1; row < 3; ++row) {
      const double candidate = std::abs(matrix[row][column]);
      if (candidate > pivot_abs) {
        pivot = row;
        pivot_abs = candidate;
      }
    }
    if (pivot_abs < 1.0e-15) {
      return false;
    }
    if (pivot != column) {
      for (int k = column; k < 4; ++k) {
        std::swap(matrix[pivot][k], matrix[column][k]);
      }
    }
    const double diagonal = matrix[column][column];
    for (int row = column + 1; row < 3; ++row) {
      const double factor = matrix[row][column] / diagonal;
      for (int k = column; k < 4; ++k) {
        matrix[row][k] -= factor * matrix[column][k];
      }
    }
  }
  double coefficients[3] = {0.0, 0.0, 0.0};
  for (int row = 2; row >= 0; --row) {
    double rhs = matrix[row][3];
    for (int column = row + 1; column < 3; ++column) {
      rhs -= matrix[row][column] * coefficients[column];
    }
    coefficients[row] = rhs / matrix[row][row];
  }
  const double a = coefficients[0];
  const double b = coefficients[1];

  double residual_sum = 0.0;
  std::size_t inliers = 0;
  for (const Point3 & point : points) {
    const double predicted = a * point.x + b * point.y +
      (centroid_z - a * centroid_x - b * centroid_y);
    const double residual = point.z - predicted;
    residual_sum += residual * residual;
    if (std::abs(residual) <= inlier_tolerance) {
      ++inliers;
    }
  }
  evidence->valid = true;
  evidence->normal = Vector3{-a, -b, 1.0}.Normalized();
  if (evidence->normal.z < 0.0) {
    evidence->normal = Vector3{
      -evidence->normal.x, -evidence->normal.y, -evidence->normal.z};
  }
  evidence->offset = Dot(
    evidence->normal, Vector3{
        centroid_x, centroid_y, centroid_z});
  evidence->rms_residual = std::sqrt(residual_sum / count);
  evidence->inlier_count = inliers;
  return true;
}

// Sorts points by Z and then by their full coordinates. Used only to find the
// largest internal gap; the resulting split sides are re-sorted
// lexicographically before they leave the candidate stage.
void SortByZ(std::vector<Point3> * points)
{
  if (points == nullptr) {
    return;
  }
  std::stable_sort(
    points->begin(), points->end(),
    [](const Point3 & lhs, const Point3 & rhs) {
      if (lhs.z != rhs.z) {
        return lhs.z < rhs.z;
      }
      return PointLess(lhs, rhs);
    });
}

// Sorts a surface's retained points lexicographically so support hull evidence
// and local surface ordering are independent of split/region-growth traversal.
void SortLexicographically(std::vector<Point3> * points)
{
  if (points == nullptr) {
    return;
  }
  std::sort(points->begin(), points->end(), PointLess);
}

// Returns the median of a mutable value list, or zero when the list is empty.
double Median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

// Deterministic stride subsample used by the boundary probe. The sampled points
// preserve lexicographic order so the same input always yields the same probe.
std::vector<Point3> SubsampleForBoundary(const std::vector<Point3> & points)
{
  if (points.size() <= kMaxBoundarySamples) {
    return points;
  }
  std::vector<Point3> sampled;
  sampled.reserve(kMaxBoundarySamples);
  const double stride =
    static_cast<double>(points.size()) / static_cast<double>(kMaxBoundarySamples);
  for (std::size_t index = 0; index < kMaxBoundarySamples; ++index) {
    std::size_t source_index =
      static_cast<std::size_t>(static_cast<double>(index) * stride);
    if (source_index >= points.size()) {
      source_index = points.size() - 1U;
    }
    sampled.push_back(points[source_index]);
  }
  return sampled;
}

// Local cross-surface continuity evidence. For every sampled point the nearest
// XY point of the other surface is found; a curb produces a consistent height
// step across that nearest pair while a continuous ramp produces a near-zero
// step. This is boundary support, not a mean-height comparison.
struct BoundaryEvidence
{
  double coverage{0.0};
  double median_step{0.0};
};

// Evaluates nearest-XY cross-surface height steps in both directions. Coverage
// is the fraction of nearest pairs whose absolute height step reaches min_step;
// median_step summarizes the overall local separation used by split gating.
BoundaryEvidence EvaluateBoundaryEvidence(
  const std::vector<Point3> & first, const std::vector<Point3> & second,
  double min_step)
{
  BoundaryEvidence evidence;
  if (first.empty() || second.empty()) {
    return evidence;
  }
  const std::vector<Point3> first_samples = SubsampleForBoundary(first);
  const std::vector<Point3> second_samples = SubsampleForBoundary(second);
  std::vector<double> steps;
  steps.reserve(first_samples.size() + second_samples.size());
  std::size_t strong = 0;
  const auto accumulate =
    [&steps, &strong, min_step](
    const std::vector<Point3> & from, const std::vector<Point3> & to) {
      for (const Point3 & point : from) {
        double best_distance = std::numeric_limits<double>::infinity();
        double best_step = 0.0;
        for (const Point3 & other : to) {
          const double dx = point.x - other.x;
          const double dy = point.y - other.y;
          const double distance = dx * dx + dy * dy;
          const double step = std::abs(point.z - other.z);
          if (distance < best_distance ||
            (distance == best_distance && step > best_step))
          {
            best_distance = distance;
            best_step = step;
          }
        }
        if (std::isfinite(best_distance)) {
          steps.push_back(best_step);
          if (best_step >= min_step) {
            ++strong;
          }
        }
      }
    };
  accumulate(first_samples, second_samples);
  accumulate(second_samples, first_samples);
  if (steps.empty()) {
    return evidence;
  }
  evidence.median_step = Median(std::move(steps));
  evidence.coverage =
    static_cast<double>(strong) / static_cast<double>(
    first_samples.size() + second_samples.size());
  return evidence;
}

// One candidate split of an initial layer. Both sides are lexicographically
// sorted before they are returned so IDs and retained hull points stay stable.
struct SplitCandidate
{
  enum class Source
  {
    kZGap,
    kRegionGrowing,
  };

  bool valid{false};
  Source source{Source::kZGap};
  std::vector<Point3> first;
  std::vector<Point3> second;
  double gap{0.0};
  double combined_rms{std::numeric_limits<double>::infinity()};
};

// Computes the count-weighted RMS of two child plane fits. Returns false when
// either child cannot form a valid height plane, because such a split would
// replace one unusable surface with two unusable surfaces.
bool ComputeCombinedRms(
  const std::vector<Point3> & first, const std::vector<Point3> & second,
  double inlier_tolerance, double * combined_rms,
  double * fit_ms = nullptr)
{
  if (combined_rms == nullptr || first.empty() || second.empty()) {
    return false;
  }
  PlaneEvidence first_plane;
  PlaneEvidence second_plane;
  if (!FitHeightPlane(first, inlier_tolerance, &first_plane, fit_ms) ||
    !FitHeightPlane(second, inlier_tolerance, &second_plane, fit_ms))
  {
    return false;
  }
  if (!std::isfinite(first_plane.rms_residual) ||
    !std::isfinite(second_plane.rms_residual))
  {
    return false;
  }
  const double first_count = static_cast<double>(first.size());
  const double second_count = static_cast<double>(second.size());
  const double total = first_count + second_count;
  const double residual_sum =
    first_count * first_plane.rms_residual * first_plane.rms_residual +
    second_count * second_plane.rms_residual * second_plane.rms_residual;
  *combined_rms = std::sqrt(residual_sum / total);
  return std::isfinite(*combined_rms);
}

// Candidate split at the largest internal Z gap. This is the primary detector
// for a single fine cell that contains two nearly flat surfaces.
bool LargestZGapSplit(
  const std::vector<Point3> & points, double min_gap,
  SplitCandidate * candidate)
{
  if (candidate == nullptr || points.size() < 2U) {
    return false;
  }
  std::vector<Point3> by_z = points;
  SortByZ(&by_z);
  std::size_t split_index = 0;
  double largest_gap = -1.0;
  for (std::size_t index = 1; index < by_z.size(); ++index) {
    const double gap = by_z[index].z - by_z[index - 1].z;
    if (gap > largest_gap) {
      largest_gap = gap;
      split_index = index;
    }
  }
  if (split_index == 0U || largest_gap < min_gap) {
    return false;
  }
  candidate->first.assign(
    by_z.begin(), by_z.begin() + static_cast<std::ptrdiff_t>(split_index));
  candidate->second.assign(
    by_z.begin() + static_cast<std::ptrdiff_t>(split_index), by_z.end());
  SortLexicographically(&candidate->first);
  SortLexicographically(&candidate->second);
  candidate->source = SplitCandidate::Source::kZGap;
  candidate->gap = largest_gap;
  candidate->valid = true;
  return true;
}

// Candidate split from bounded region growing seeded by the lowest and highest
// points. This exposes a sub-cell curb whose Z samples overlap across the two
// surfaces (for example a diagonal curb) and therefore has no clean largest Z
// gap. The assignment is re-fitted for a small fixed number of iterations.
bool RegionGrowingSplit(
  const std::vector<Point3> & points, double min_gap,
  double inlier_tolerance, SplitCandidate * candidate,
  double * fit_ms = nullptr)
{
  if (candidate == nullptr || points.size() < 2U) {
    return false;
  }
  std::vector<Point3> ordered = points;
  SortLexicographically(&ordered);
  std::size_t low_index = 0;
  std::size_t high_index = 0;
  for (std::size_t index = 1; index < ordered.size(); ++index) {
    if (ordered[index].z < ordered[low_index].z ||
      (ordered[index].z == ordered[low_index].z &&
      PointLess(ordered[index], ordered[low_index])))
    {
      low_index = index;
    }
    if (ordered[index].z > ordered[high_index].z ||
      (ordered[index].z == ordered[high_index].z &&
      PointLess(ordered[index], ordered[high_index])))
    {
      high_index = index;
    }
  }
  if (low_index == high_index) {
    return false;
  }
  const double low_z = ordered[low_index].z;
  const double high_z = ordered[high_index].z;
  if (!(high_z - low_z >= min_gap)) {
    return false;
  }

  const double midpoint = 0.5 * (low_z + high_z);
  std::vector<Point3> first;
  std::vector<Point3> second;
  for (const Point3 & point : ordered) {
    if (point.z <= midpoint) {
      first.push_back(point);
    } else {
      second.push_back(point);
    }
  }
  if (first.empty() || second.empty()) {
    return false;
  }

  for (int iteration = 0; iteration < kMaxRegionGrowingIterations; ++iteration) {
    PlaneEvidence first_plane;
    PlaneEvidence second_plane;
    const bool first_valid =
      FitHeightPlane(first, inlier_tolerance, &first_plane, fit_ms);
    const bool second_valid =
      FitHeightPlane(second, inlier_tolerance, &second_plane, fit_ms);
    const double first_mean = MeanZ(first);
    const double second_mean = MeanZ(second);
    std::vector<Point3> next_first;
    std::vector<Point3> next_second;
    next_first.reserve(first.size());
    next_second.reserve(second.size());
    for (const Point3 & point : ordered) {
      double first_residual = std::abs(point.z - first_mean);
      double second_residual = std::abs(point.z - second_mean);
      if (first_valid) {
        const Vector3 value{point.x, point.y, point.z};
        first_residual = std::abs(
          Dot(first_plane.normal, value) - first_plane.offset);
      }
      if (second_valid) {
        const Vector3 value{point.x, point.y, point.z};
        second_residual = std::abs(
          Dot(second_plane.normal, value) - second_plane.offset);
      }
      // Ties stay with the lower seed so the assignment is deterministic.
      if (first_residual <= second_residual) {
        next_first.push_back(point);
      } else {
        next_second.push_back(point);
      }
    }
    if (next_first.empty() || next_second.empty()) {
      break;
    }
    first.swap(next_first);
    second.swap(next_second);
  }

  if (first.empty() || second.empty()) {
    return false;
  }
  SortLexicographically(&first);
  SortLexicographically(&second);
  candidate->first = std::move(first);
  candidate->second = std::move(second);
  candidate->source = SplitCandidate::Source::kRegionGrowing;
  candidate->gap = std::abs(MeanZ(candidate->first) - MeanZ(candidate->second));
  candidate->valid = true;
  return true;
}

// Decides whether one candidate split materially improves local evidence. A
// curb survives the decision because it has both a local boundary step and a
// large residual improvement; a continuous ramp or random roughness lacks the
// cross-boundary step and is kept as one surface.
bool FindAcceptedSplit(
  const std::vector<Point3> & points, const PlaneEvidence & parent_plane,
  bool parent_valid, const TerrainGridBuilderOptions & options,
  SplitCandidate * accepted, double * fit_ms = nullptr)
{
  if (accepted == nullptr ||
    points.size() < 2U * options.min_samples_per_surface)
  {
    return false;
  }
  *accepted = SplitCandidate{};
  SplitCandidate candidates[2];
  const bool have_zgap = LargestZGapSplit(
    points, options.refine_split_min_gap, &candidates[0]);
  const bool have_region = RegionGrowingSplit(
    points, options.refine_split_min_gap, options.max_plane_rms,
    &candidates[1], fit_ms);
  for (int index = 0; index < 2; ++index) {
    if ((index == 0 && !have_zgap) || (index == 1 && !have_region)) {
      continue;
    }
    const SplitCandidate & candidate = candidates[index];
    if (candidate.first.size() < options.min_samples_per_surface ||
      candidate.second.size() < options.min_samples_per_surface)
    {
      continue;
    }
    double combined_rms = std::numeric_limits<double>::infinity();
    if (!ComputeCombinedRms(
        candidate.first, candidate.second, options.max_plane_rms,
        &combined_rms, fit_ms))
    {
      continue;
    }
    const BoundaryEvidence boundary = EvaluateBoundaryEvidence(
      candidate.first, candidate.second, options.refine_split_min_gap);
    const double separation = std::max(candidate.gap, boundary.median_step);
    if (!std::isfinite(separation) ||
      separation < options.refine_split_min_gap)
    {
      continue;
    }

    const bool parent_rms_known =
      parent_valid && std::isfinite(parent_plane.rms_residual);
    if (parent_rms_known) {
      if (parent_plane.rms_residual > options.max_plane_rms) {
        // A rough parent must become strictly better after the split.
        if (!(combined_rms < parent_plane.rms_residual)) {
          continue;
        }
      } else {
        const double improvement =
          parent_plane.rms_residual - combined_rms;
        if (improvement < options.refine_min_rms_improvement) {
          continue;
        }
        // With a globally passing plane, require the two surface means to be
        // clearly separated relative to the parent residual. This prevents
        // ordinary roughness from becoming artificial multi-layer decks.
        const double required_separation = std::max(
          options.refine_split_min_gap,
          kSeparatedMeanFactor * parent_plane.rms_residual);
        if (separation < required_separation) {
          continue;
        }
      }
    }

    if (boundary.coverage < options.min_boundary_coverage) {
      const bool decisive_gap =
        separation >= kDecisiveGapFactor * options.refine_split_min_gap;
      const bool parent_rough =
        parent_rms_known && parent_plane.rms_residual > options.max_plane_rms;
      if (!(decisive_gap && parent_rough)) {
        continue;
      }
    }

    bool better = !accepted->valid;
    if (accepted->valid) {
      if (combined_rms < accepted->combined_rms - 1.0e-12) {
        better = true;
      } else if (std::abs(combined_rms - accepted->combined_rms) <= 1.0e-12 &&
        separation > accepted->gap)
      {
        better = true;
      }
    }
    if (better) {
      *accepted = candidate;
      accepted->combined_rms = combined_rms;
      accepted->gap = separation;
    }
  }
  return accepted->valid;
}

// Recursively applies the bounded split decision. Every accepted split spends
// one max_refine_depth unit; leaves retain exact points so an unresolved
// degenerate/rough surface stays unknown instead of being interpolated.
void RefineLayer(
  const std::vector<Point3> & points, const TerrainGridBuilderOptions & options,
  int depth, std::vector<std::vector<Point3>> * output,
  double * fit_ms = nullptr)
{
  if (output == nullptr) {
    return;
  }
  PlaneEvidence parent_plane;
  const bool parent_valid =
    !points.empty() && FitHeightPlane(
    points, options.max_plane_rms,
    &parent_plane, fit_ms);
  SplitCandidate split;
  if (depth < static_cast<int>(options.max_refine_depth) &&
    FindAcceptedSplit(
      points, parent_plane, parent_valid, options, &split,
      fit_ms))
  {
    RefineLayer(split.first, options, depth + 1, output, fit_ms);
    RefineLayer(split.second, options, depth + 1, output, fit_ms);
    return;
  }
  std::vector<Point3> leaf = points;
  SortLexicographically(&leaf);
  output->push_back(std::move(leaf));
}

// Marks the angular support sectors around a cell centre. This is directional
// boundary evidence, not a tire contact patch.
void ComputeSupportSectors(
  const std::vector<Point3> & points, const Point3 & cell_center,
  double radius, SupportEvidence * support)
{
  if (support == nullptr) {
    return;
  }
  support->supported_sectors.fill(false);
  if (!std::isfinite(radius) || radius <= 0.0) {
    return;
  }
  for (const Point3 & point : points) {
    const double dx = point.x - cell_center.x;
    const double dy = point.y - cell_center.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance > radius ||
      (std::abs(dx) < 1.0e-12 && std::abs(dy) < 1.0e-12))
    {
      continue;
    }
    const double angle = std::atan2(dy, dx);
    const double normalized = (angle + kPi) / (2.0 * kPi);
    int sector = static_cast<int>(
      std::floor(normalized * static_cast<double>(kSupportSectorCount)));
    if (sector < 0) {
      sector = 0;
    }
    if (sector >= static_cast<int>(kSupportSectorCount)) {
      sector = static_cast<int>(kSupportSectorCount) - 1;
    }
    support->supported_sectors[static_cast<std::size_t>(sector)] = true;
  }
  support->edge_support_coverage = support->SectorCoverage();
}

// Rebuilds all raw support evidence for one finalized surface. The retained
// points are the exact points assigned to this surface, so wheel support can
// query the per-surface hull. z_min/z_max, directional sectors, and coverage are
// refreshed after every split; stale parent evidence is never carried over.
void RefreshSupportEvidence(
  const std::vector<Point3> & points, const FineCellIndex & cell,
  const TerrainGridBuilderOptions & options, SupportEvidence * support)
{
  if (support == nullptr) {
    return;
  }
  support->points = points;
  support->sample_count = points.size();
  support->z_min = std::numeric_limits<double>::quiet_NaN();
  support->z_max = std::numeric_limits<double>::quiet_NaN();
  support->edge_support_coverage = 0.0;
  support->supported_sectors.fill(false);
  if (points.empty()) {
    return;
  }
  support->z_min = points.front().z;
  support->z_max = points.front().z;
  for (const Point3 & point : points) {
    support->z_min = std::min(support->z_min, point.z);
    support->z_max = std::max(support->z_max, point.z);
  }
  const Point3 cell_center = FineCellCenter(
    cell, options.origin_x, options.origin_y, options.resolution);
  ComputeSupportSectors(
    points, cell_center, options.support_radius_factor * options.resolution,
    support);
}

// Builds one node from a finalized layer. The caller assigns the global ID and
// local surface index after deterministic layer ordering.
TerrainNode MakeNode(
  const std::vector<Point3> & layer, const FineCellIndex & cell,
  const TerrainGridBuilderOptions & options, bool excessive_layer,
  double * fit_ms = nullptr)
{
  TerrainNode node;
  node.cell = cell;
  // Sorting here makes the retained support hull and fallback representative
  // independent of the order in which recursive splitting assembled the leaf.
  std::vector<Point3> surface = layer;
  SortLexicographically(&surface);
  RefreshSupportEvidence(surface, cell, options, &node.support);

  const bool fit = FitHeightPlane(
    surface, options.max_plane_rms, &node.plane, fit_ms);
  if (fit) {
    node.representative = node.plane.centroid;
  } else if (!surface.empty()) {
    Point3 mean{};
    for (const Point3 & point : surface) {
      mean.x += point.x;
      mean.y += point.y;
      mean.z += point.z;
    }
    const double count = static_cast<double>(surface.size());
    node.representative = Point3{mean.x / count, mean.y / count, mean.z / count};
  }

  if (excessive_layer) {
    node.status = TerrainStatus::kProhibited;
    node.reason = ProhibitedReason::kExcessiveLayers;
  } else if (surface.size() < options.min_samples_per_surface) {
    node.status = TerrainStatus::kUnknown;
    node.reason = ProhibitedReason::kInsufficientSamples;
  } else if (!fit || !node.plane.valid) {
    node.status = TerrainStatus::kUnknown;
    node.reason = ProhibitedReason::kDegenerateGeometry;
  } else if (!std::isfinite(node.plane.rms_residual) ||
    node.plane.rms_residual > options.max_plane_rms)
  {
    node.status = TerrainStatus::kUnknown;
    node.reason = ProhibitedReason::kHighResidual;
  } else {
    const double normal_z = std::max(
      -1.0, std::min(1.0, node.plane.normal.z));
    const double slope = std::acos(normal_z);
    if (options.mark_excessive_slope_prohibited &&
      slope > options.max_slope_rad)
    {
      node.status = TerrainStatus::kProhibited;
      node.reason = ProhibitedReason::kExcessiveSlope;
    } else {
      node.status = TerrainStatus::kSupported;
      node.reason = ProhibitedReason::kNone;
    }
  }
  return node;
}

// Returns the fitted plane height at one XY position. A valid plane is always
// preferred; a missing/degenerate plane falls back to the representative height
// so the caller can still emit explicit non-finite/insufficient evidence.
bool PlaneHeightAt(
  const TerrainNode & node, double x, double y, double * height)
{
  if (height == nullptr) {
    return false;
  }
  // Shared reconstructed corners are part of the planning surface, so the live
  // grid/connection path must not evaluate the raw fitted plane behind them.
  const double value = TerrainSurfaceHeight(node, x, y);
  if (std::isfinite(value)) {
    *height = value;
    return true;
  }
  if (std::isfinite(node.representative.z)) {
    *height = node.representative.z;
    return true;
  }
  return false;
}

// Evaluates one adjacent-cell surface pair and stores explicit evidence even
// when the pair is rejected. Continuity is evaluated at the shared cell boundary
// from the fitted planes, and directional boundary coverage must be present on
// both sides; a mean-height delta alone is never sufficient for a link.
TerrainConnection EvaluateConnection(
  const TerrainNode & source, const TerrainNode & target,
  const TerrainGridBuilderOptions & options, double resolution)
{
  TerrainConnection connection;
  connection.source = std::min(source.id, target.id);
  connection.target = std::max(source.id, target.id);
  // Same-cell surfaces are separate local support models by construction. The
  // builder never pairs them, and this explicit reason documents the
  // kSameCellNoLink contract if the evaluator is ever called that way.
  if (source.cell == target.cell) {
    connection.status = ConnectionStatus::kDiscontinuous;
    connection.reason = ConnectionReason::kSameCellNoLink;
    return connection;
  }
  const Point3 source_center = FineCellCenter(
    source.cell, options.origin_x, options.origin_y, resolution);
  const Point3 target_center = FineCellCenter(
    target.cell, options.origin_x, options.origin_y, resolution);
  const double dx = target_center.x - source_center.x;
  const double dy = target_center.y - source_center.y;
  const double horizontal = std::hypot(dx, dy);
  connection.horizontal_distance = horizontal;
  const double boundary_x = 0.5 * (source_center.x + target_center.x);
  const double boundary_y = 0.5 * (source_center.y + target_center.y);
  double source_z = source.representative.z;
  double target_z = target.representative.z;
  const bool source_height_known =
    PlaneHeightAt(source, boundary_x, boundary_y, &source_z);
  const bool target_height_known =
    PlaneHeightAt(target, boundary_x, boundary_y, &target_z);
  // The stored height_delta remains the representative-cell summary used by
  // diagnostics and coarse edges. Continuity gates use the fitted plane heights
  // at the shared boundary, which is the actual local surface evidence.
  connection.height_delta =
    target.representative.z - source.representative.z;
  const double boundary_height_delta = target_z - source_z;
  connection.normal_dot = std::max(
    -1.0, std::min(1.0, Dot(source.plane.normal, target.plane.normal)));
  connection.boundary_coverage = std::min(
    source.support.CoverageToward(dx, dy),
    target.support.CoverageToward(-dx, -dy));
  // Regional reconstruction already established a shared-corner surface from
  // real regional support. Requiring that same cell to also carry raw support
  // in this exact direction would disconnect every infilled scan-hole cell.
  // The height-step, slope and normal gates below still consume the exact
  // shared surface, so real steps/floors remain separate.
  const bool reconstructed_pair =
    source.surface_reconstruction.applied &&
    source.surface_reconstruction.shared_corners_valid &&
    target.surface_reconstruction.applied &&
    target.surface_reconstruction.shared_corners_valid;
  const bool source_boundary_support = reconstructed_pair ||
    source.support.HasSupportToward(dx, dy);
  const bool target_boundary_support = reconstructed_pair ||
    target.support.HasSupportToward(-dx, -dy);
  if (!source_height_known || !target_height_known ||
    !source.representative.IsFinite() ||
    !target.representative.IsFinite() ||
    !std::isfinite(boundary_height_delta) ||
    !std::isfinite(connection.height_delta) ||
    !std::isfinite(connection.normal_dot))
  {
    connection.status = ConnectionStatus::kInsufficientEvidence;
    connection.reason = ConnectionReason::kNonFinite;
    return connection;
  }
  if (source.status == TerrainStatus::kProhibited ||
    target.status == TerrainStatus::kProhibited)
  {
    connection.status = ConnectionStatus::kDiscontinuous;
    connection.reason = ConnectionReason::kProhibitedSurface;
    return connection;
  }
  if (!source.plane.valid || !target.plane.valid ||
    source.support.sample_count < options.min_samples_per_surface ||
    target.support.sample_count < options.min_samples_per_surface)
  {
    connection.status = ConnectionStatus::kInsufficientEvidence;
    connection.reason = ConnectionReason::kSparseSurface;
    return connection;
  }
  if (std::abs(boundary_height_delta) > options.max_step_height) {
    connection.status = ConnectionStatus::kDiscontinuous;
    connection.reason = ConnectionReason::kHeightStep;
    return connection;
  }
  const double slope_reference = std::max(horizontal, resolution);
  connection.slope = std::atan2(
    std::abs(boundary_height_delta), slope_reference);
  if (!std::isfinite(connection.slope) ||
    connection.slope > options.max_slope_rad)
  {
    connection.status = ConnectionStatus::kDiscontinuous;
    connection.reason = ConnectionReason::kSlopeLimit;
    return connection;
  }
  if (connection.normal_dot <
    std::cos(options.max_normal_angle_rad))
  {
    connection.status = ConnectionStatus::kInconsistentNormals;
    connection.reason = ConnectionReason::kNormalMismatch;
    return connection;
  }
  if (!source_boundary_support || !target_boundary_support ||
    (!reconstructed_pair &&
    (!std::isfinite(connection.boundary_coverage) ||
    connection.boundary_coverage < options.min_boundary_coverage)))
  {
    connection.status = ConnectionStatus::kInsufficientEvidence;
    connection.reason = ConnectionReason::kBoundaryCoverage;
    return connection;
  }
  connection.status = ConnectionStatus::kConnected;
  connection.reason = ConnectionReason::kNone;
  return connection;
}

// Hashes canonical cell content plus geometry-affecting options. Timestamps and
// point counts alone are deliberately excluded from revision identity.
std::uint64_t ContentHash(
  const CellMap & cells, const TerrainGridBuilderOptions & options,
  const std::string & frame_id)
{
  std::uint64_t hash = kFnvOffset;
  HashUint64(&hash, kBuilderSchemaVersion);
  HashString(&hash, frame_id);
  HashDouble(&hash, options.resolution);
  HashDouble(&hash, options.origin_x);
  HashDouble(&hash, options.origin_y);
  HashDouble(&hash, options.layer_gap_threshold);
  HashDouble(&hash, options.refine_split_min_gap);
  HashDouble(&hash, options.refine_min_rms_improvement);
  HashUint64(&hash, static_cast<std::uint64_t>(options.max_refine_depth));
  HashDouble(&hash, options.max_plane_rms);
  HashDouble(&hash, options.max_step_height);
  HashDouble(&hash, options.max_slope_rad);
  HashDouble(&hash, options.max_normal_angle_rad);
  HashDouble(&hash, options.min_boundary_coverage);
  HashDouble(&hash, options.support_radius_factor);
  HashUint64(
    &hash, static_cast<std::uint64_t>(options.min_samples_per_surface));
  HashUint64(&hash, static_cast<std::uint64_t>(options.max_layers_per_cell));
  HashUint64(&hash, options.mark_excessive_slope_prohibited ? 1U : 0U);
  for (const auto & cell_entry : cells) {
    const FineCellIndex & cell = cell_entry.first;
    const std::vector<Point3> & points = cell_entry.second;
    HashUint64(&hash, static_cast<std::uint64_t>(cell.x));
    HashUint64(&hash, static_cast<std::uint64_t>(cell.y));
    HashUint64(&hash, static_cast<std::uint64_t>(points.size()));
    for (const Point3 & point : points) {
      HashDouble(&hash, point.x);
      HashDouble(&hash, point.y);
      HashDouble(&hash, point.z);
    }
  }
  return hash;
}

// Converts the observation into deterministic XY cells, rejecting any finite
// point that cannot be safely indexed. Returns an empty detail on success.
bool CollectCells(
  const TerrainPointObservation & observation,
  const TerrainGridBuilderOptions & options, CellMap * cells,
  std::string * detail)
{
  cells->clear();
  for (std::size_t index = 0; index < observation.points().size(); ++index) {
    const Point3 & point = observation.points()[index];
    const auto cell = FineCellOf(
      point, options.origin_x, options.origin_y, options.resolution);
    if (!cell.has_value()) {
      if (detail != nullptr) {
        *detail = "point " + std::to_string(index) +
          " is outside the safe finite cell/index bounds";
      }
      return false;
    }
    (*cells)[*cell].push_back(point);
  }
  for (auto & entry : *cells) {
    std::sort(entry.second.begin(), entry.second.end(), PointLess);
  }
  return true;
}

// Fixed eight-neighbour order makes candidate connection generation stable.
constexpr std::array<std::array<int, 2>, 8> kNeighborOffsets{{
  {{1, 0}},
  {{1, 1}},
  {{0, 1}},
  {{-1, 1}},
  {{-1, 0}},
  {{-1, -1}},
  {{0, -1}},
  {{1, -1}},
}};

}  // namespace

TerrainGridBuilder::TerrainGridBuilder(TerrainGridBuilderOptions options)
: options_(std::move(options))
{
}

bool TerrainGridBuilder::ValidateOptions(std::string * detail) const
{
  const auto reject = [detail](const std::string & message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return false;
    };
  if (!IsValidResolution(options_.resolution)) {
    return reject("resolution must be finite and positive");
  }
  if (!std::isfinite(options_.origin_x) || !std::isfinite(options_.origin_y) ||
    std::abs(options_.origin_x) > kMaxWorldCoordinate ||
    std::abs(options_.origin_y) > kMaxWorldCoordinate)
  {
    return reject("origin must be finite and within the safe world bound");
  }
  if (!std::isfinite(options_.layer_gap_threshold) ||
    options_.layer_gap_threshold <= 0.0)
  {
    return reject("layer_gap_threshold must be finite and positive");
  }
  if (!std::isfinite(options_.refine_split_min_gap) ||
    options_.refine_split_min_gap < 0.0)
  {
    return reject("refine_split_min_gap must be finite and non-negative");
  }
  if (!std::isfinite(options_.refine_min_rms_improvement) ||
    options_.refine_min_rms_improvement <= 0.0)
  {
    return reject("refine_min_rms_improvement must be finite and positive");
  }
  if (!std::isfinite(options_.max_plane_rms) ||
    options_.max_plane_rms <= 0.0)
  {
    return reject("max_plane_rms must be finite and positive");
  }
  if (options_.max_refine_depth == 0U || options_.max_refine_depth > 64U) {
    return reject("max_refine_depth must be in [1, 64]");
  }
  if (options_.min_samples_per_surface < 3U) {
    return reject("min_samples_per_surface must be at least 3 for a plane fit");
  }
  if (options_.max_layers_per_cell == 0U) {
    return reject("max_layers_per_cell must be positive");
  }
  if (!std::isfinite(options_.max_step_height) ||
    options_.max_step_height <= 0.0)
  {
    return reject("max_step_height must be finite and positive");
  }
  if (!std::isfinite(options_.max_slope_rad) ||
    options_.max_slope_rad <= 0.0 || options_.max_slope_rad >= kPi * 0.5)
  {
    return reject("max_slope_rad must be in (0, pi/2)");
  }
  if (!std::isfinite(options_.max_normal_angle_rad) ||
    options_.max_normal_angle_rad < 0.0 ||
    options_.max_normal_angle_rad > kPi)
  {
    return reject("max_normal_angle_rad must be in [0, pi]");
  }
  if (!std::isfinite(options_.min_boundary_coverage) ||
    options_.min_boundary_coverage < 0.0 ||
    options_.min_boundary_coverage > 1.0)
  {
    return reject("min_boundary_coverage must be in [0, 1]");
  }
  if (!std::isfinite(options_.support_radius_factor) ||
    options_.support_radius_factor <= 0.0)
  {
    return reject("support_radius_factor must be finite and positive");
  }
  const ObservationValidationOptions & validation =
    options_.observation_validation;
  if (validation.max_receipt_age_ns < 0 ||
    validation.future_tolerance_ns < 0 ||
    validation.max_points == 0U)
  {
    return reject("observation validation limits are invalid");
  }
  if (validation.require_frame_match && validation.expected_frame_id.empty()) {
    return reject("expected_frame_id is required when frame matching is enabled");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

TerrainBuildResult TerrainGridBuilder::Build(
  const TerrainPointObservation & observation,
  std::int64_t now_steady_ns)
{
  const auto build_start = std::chrono::steady_clock::now();
  TerrainBuildMetrics metrics;
  metrics.measured = true;
  metrics.accepted_steady_ns = SteadyNowNs();
  metrics.input_points = observation.points().size();
  metrics.observation_sequence = observation.metadata().sequence;
  metrics.sensor_stamp_ns = observation.metadata().sensor_stamp_ns;
  metrics.received_steady_ns = observation.metadata().received_steady_ns;

  // Completes a rejected attempt with real elapsed time. The same metric object
  // is copied into every result, so failed observations are retained by the
  // collector instead of becoming silent gaps.
  auto finish_failure = [&](TerrainBuildStatus status, std::string detail) {
      metrics.completed_steady_ns = SteadyNowNs();
      metrics.total_ms = DurationToMs(
        std::chrono::steady_clock::now() - build_start);
      metrics.status = TerrainBuildStatusToString(status);
      metrics.detail = detail;
      TerrainBuildResult result;
      result.status = status;
      result.detail = std::move(detail);
      result.metrics = metrics;
      return result;
    };

  std::string detail;
  if (!ValidateOptions(&detail)) {
    return finish_failure(
      TerrainBuildStatus::kInvalidOptions,
      std::move(detail));
  }
  if (observation.metadata().update != ObservationUpdateSemantics::kReplace) {
    return finish_failure(
      TerrainBuildStatus::kUnsupportedUpdateSemantics,
      "batch 2 terrain builder only accepts replace observations; got " +
      std::string(
        ObservationUpdateSemanticsToString(
          observation.metadata().update)));
  }
  const ObservationValidation validation = observation.Validate(
    options_.observation_validation, now_steady_ns);
  if (!validation.ok) {
    return finish_failure(
      TerrainBuildStatus::kObservationRejected,
      std::string(ObservationValidationStatusToString(validation.status)) +
      ": " + validation.detail);
  }

  metrics.input_validation_ms = DurationToMs(
    std::chrono::steady_clock::now() - build_start);
  const auto binning_start = std::chrono::steady_clock::now();
  CellMap cells;
  if (!CollectCells(observation, options_, &cells, &detail)) {
    metrics.binning_ms = DurationToMs(
      std::chrono::steady_clock::now() - binning_start);
    return finish_failure(
      TerrainBuildStatus::kObservationRejected,
      std::move(detail));
  }
  metrics.binning_ms = DurationToMs(
    std::chrono::steady_clock::now() - binning_start);
  std::size_t valid_points = 0;
  for (const auto & cell_entry : cells) {
    valid_points += cell_entry.second.size();
  }
  metrics.valid_points = valid_points;
  metrics.cells = cells.size();

  const auto hash_start = std::chrono::steady_clock::now();
  const std::uint64_t content_hash = ContentHash(
    cells, options_, observation.metadata().frame_id);
  metrics.content_hash_ms = DurationToMs(
    std::chrono::steady_clock::now() - hash_start);
  if (!have_content_hash_ || content_hash != last_content_hash_) {
    ++revision_;
    last_content_hash_ = content_hash;
    have_content_hash_ = true;
  }
  metrics.revision = revision_;
  metrics.content_hash = content_hash;

  std::vector<TerrainNode> nodes;
  nodes.reserve(cells.size());
  TerrainNodeId next_id = 1;
  double segmentation_ms = 0.0;
  double attribute_ms = 0.0;
  for (const auto & cell_entry : cells) {
    const FineCellIndex & cell = cell_entry.first;
    const std::vector<Point3> & points = cell_entry.second;
    const auto segmentation_start = std::chrono::steady_clock::now();
    std::vector<std::vector<Point3>> layers = SplitByZGap(
      points, options_.layer_gap_threshold);
    std::vector<std::vector<Point3>> refined;
    for (const std::vector<Point3> & layer : layers) {
      RefineLayer(
        layer, options_, 0, &refined,
        &metrics.segmentation_fit_ms_inclusive);
    }
    segmentation_ms += DurationToMs(
      std::chrono::steady_clock::now() - segmentation_start);
    std::sort(refined.begin(), refined.end(), LayerLess);
    const auto attribute_start = std::chrono::steady_clock::now();
    for (std::size_t layer_index = 0; layer_index < refined.size();
      ++layer_index)
    {
      TerrainNode node = MakeNode(
        refined[layer_index], cell, options_,
        layer_index >= options_.max_layers_per_cell,
        &metrics.attribute_fit_ms_inclusive);
      node.id = next_id++;
      node.local_surface_index = static_cast<int>(layer_index);
      nodes.push_back(std::move(node));
    }
    attribute_ms += DurationToMs(
      std::chrono::steady_clock::now() - attribute_start);
  }
  metrics.segmentation_ms = segmentation_ms;
  metrics.attribute_ms = attribute_ms;
  // The fit portions are measured inside the two inclusive phases, so the
  // exclusive values are arithmetic differences, not a second measurement.
  metrics.segmentation_fit_ms_exclusive =
    std::max(
    0.0, segmentation_ms -
    metrics.segmentation_fit_ms_inclusive);
  metrics.attribute_fit_ms_exclusive =
    std::max(0.0, attribute_ms - metrics.attribute_fit_ms_inclusive);
  metrics.surfaces = nodes.size();

  // Build the cell index once so candidate links can be evaluated without
  // scanning the whole node array per neighbour.
  std::unordered_map<FineCellIndex, std::vector<std::size_t>, FineCellIndexHash>
  cell_to_indices;
  cell_to_indices.reserve(nodes.size());
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    cell_to_indices[nodes[index].cell].push_back(index);
  }
  // Only adjacent fine cells are paired. Surfaces that were split from the same
  // fine cell are disjoint support models separated by an internal boundary, so
  // they deliberately receive no candidate link (kSameCellNoLink semantics).
  const auto connections_start = std::chrono::steady_clock::now();
  std::vector<TerrainConnection> connections;
  for (std::size_t source_index = 0; source_index < nodes.size(); ++source_index) {
    const TerrainNode & source = nodes[source_index];
    for (const auto & offset : kNeighborOffsets) {
      const FineCellIndex neighbour{
        source.cell.x + static_cast<std::int64_t>(offset[0]),
        source.cell.y + static_cast<std::int64_t>(offset[1])};
      const auto iterator = cell_to_indices.find(neighbour);
      if (iterator == cell_to_indices.end()) {
        continue;
      }
      for (const std::size_t target_index : iterator->second) {
        if (target_index <= source_index) {
          continue;
        }
        TerrainConnection connection = EvaluateConnection(
          source, nodes[target_index], options_, options_.resolution);
        if (connection.source == kInvalidTerrainNodeId ||
          connection.target == kInvalidTerrainNodeId)
        {
          continue;
        }
        connections.push_back(connection);
      }
    }
  }

  metrics.connections_ms = DurationToMs(
    std::chrono::steady_clock::now() - connections_start);
  metrics.connections = connections.size();
  TerrainSnapshotMetadata metadata;
  metadata.frame_id = observation.metadata().frame_id;
  metadata.resolution = options_.resolution;
  metadata.origin_x = options_.origin_x;
  metadata.origin_y = options_.origin_y;
  metadata.revision = revision_;
  metadata.content_hash = content_hash;
  metadata.sensor_stamp_ns = observation.metadata().sensor_stamp_ns;
  metadata.received_steady_ns = observation.metadata().received_steady_ns;
  metadata.observation_sequence = observation.metadata().sequence;
  auto snapshot = std::make_shared<const TerrainSnapshot>(
    std::move(nodes), std::move(connections), std::move(metadata));
  metrics.completed_steady_ns = SteadyNowNs();
  metrics.total_ms = DurationToMs(
    std::chrono::steady_clock::now() - build_start);
  metrics.status = TerrainBuildStatusToString(TerrainBuildStatus::kOk);
  TerrainBuildResult result;
  result.status = TerrainBuildStatus::kOk;
  result.snapshot = std::move(snapshot);
  result.metrics = metrics;
  return result;
}

void TerrainGridBuilder::Reset() noexcept
{
  revision_ = 0;
  last_content_hash_ = 0;
  have_content_hash_ = false;
}

const char * TerrainBuildStatusToString(TerrainBuildStatus status) noexcept
{
  switch (status) {
    case TerrainBuildStatus::kOk:
      return "ok";
    case TerrainBuildStatus::kInvalidOptions:
      return "invalid_options";
    case TerrainBuildStatus::kObservationRejected:
      return "observation_rejected";
    case TerrainBuildStatus::kUnsupportedUpdateSemantics:
      return "unsupported_update_semantics";
    case TerrainBuildStatus::kContentUnchanged:
      return "content_unchanged";
  }
  return "unknown";
}

}  // namespace grid_global_planner
