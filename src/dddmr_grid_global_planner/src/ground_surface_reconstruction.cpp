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

#include "dddmr_grid_global_planner/ground_surface_reconstruction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/offline_geometry.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

struct BoundarySample
{
  double x{0.0};
  double y{0.0};
};

bool PlaneHeightAt(const TerrainNode & node, double x, double y, double * z)
{
  if (z == nullptr || !node.plane.valid ||
    !std::isfinite(node.plane.normal.z) ||
    std::abs(node.plane.normal.z) <= kEpsilon ||
    !std::isfinite(node.plane.offset))
  {
    return false;
  }
  const double value =
    (node.plane.offset - node.plane.normal.x * x -
    node.plane.normal.y * y) / node.plane.normal.z;
  if (!std::isfinite(value)) {
    return false;
  }
  *z = value;
  return true;
}

double VectorAngle(const Vector3 & first, const Vector3 & second)
{
  const double dot = std::clamp(Dot(first, second), -1.0, 1.0);
  return std::acos(dot);
}

bool IsSupportedCandidate(const TerrainNode & node)
{
  return node.IsSupported() && node.plane.valid &&
         node.representative.IsFinite() && !node.support.points.empty();
}

// Deterministic spatially balanced sample of the exact raw support points.
// Lexicographic stride is not enough: a dense left-hand pile would still fill
// the sample cap and a sparse right-hand region could vanish. Bin the real XY
// footprint into a bounded grid, then take one representative from every
// non-empty bin per round. No point is copied, interpolated or synthesized, and
// every contributing node uses the same cap.
void AppendBalancedSamples(
  const TerrainNode & node, std::size_t cap, std::vector<Point3> * points)
{
  if (points == nullptr || cap == 0U || node.support.points.empty()) {
    return;
  }
  const std::vector<Point3> & support = node.support.points;
  const std::size_t count = support.size();
  const std::size_t take = std::min(cap, count);
  if (take == 0U) {
    return;
  }
  if (take == count) {
    points->insert(points->end(), support.begin(), support.end());
    return;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::size_t finite_count = 0U;
  for (const Point3 & point : support) {
    if (!point.IsFinite()) {
      continue;
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    ++finite_count;
  }
  if (finite_count == 0U) {
    return;
  }
  if (take >= finite_count) {
    for (const Point3 & point : support) {
      if (point.IsFinite()) {
        points->push_back(point);
      }
    }
    return;
  }

  const std::size_t bins_per_axis = std::clamp<std::size_t>(
    static_cast<std::size_t>(
      std::ceil(std::sqrt(static_cast<double>(take)))),
    1U, 8U);
  const std::size_t bin_count = bins_per_axis * bins_per_axis;
  std::vector<std::vector<std::size_t>> bins(bin_count);
  const double span_x = max_x - min_x;
  const double span_y = max_y - min_y;
  for (std::size_t index = 0U; index < count; ++index) {
    const Point3 & point = support[index];
    if (!point.IsFinite()) {
      continue;
    }
    const std::size_t bin_x = span_x > kEpsilon ?
      std::min(
      bins_per_axis - 1U,
      static_cast<std::size_t>(
        (point.x - min_x) / span_x *
        static_cast<double>(bins_per_axis))) : 0U;
    const std::size_t bin_y = span_y > kEpsilon ?
      std::min(
      bins_per_axis - 1U,
      static_cast<std::size_t>(
        (point.y - min_y) / span_y *
        static_cast<double>(bins_per_axis))) : 0U;
    bins[bin_x * bins_per_axis + bin_y].push_back(index);
  }

  std::vector<std::size_t> non_empty;
  non_empty.reserve(bins.size());
  std::size_t max_bin_size = 0U;
  for (std::size_t bin = 0U; bin < bins.size(); ++bin) {
    if (bins[bin].empty()) {
      continue;
    }
    non_empty.push_back(bin);
    max_bin_size = std::max(max_bin_size, bins[bin].size());
  }
  if (non_empty.empty()) {
    return;
  }

  // Center-out order inside each bin. The first representative is the bin's
  // central lexicographic sample, then alternating neighbours, so the first
  // pass is not biased toward a corner of the bin.
  std::vector<std::vector<std::size_t>> bin_order(non_empty.size());
  for (std::size_t slot = 0U; slot < non_empty.size(); ++slot) {
    const std::vector<std::size_t> & bin = bins[non_empty[slot]];
    std::vector<std::size_t> & order = bin_order[slot];
    order.reserve(bin.size());
    const std::size_t center = bin.size() / 2U;
    order.push_back(bin[center]);
    for (std::size_t step = 1U; order.size() < bin.size(); ++step) {
      if (center + step < bin.size()) {
        order.push_back(bin[center + step]);
      }
      if (center >= step && order.size() < bin.size()) {
        order.push_back(bin[center - step]);
      }
    }
  }

  points->reserve(points->size() + take);
  if (take < non_empty.size()) {
    // The cap cannot cover every occupied bin. Spread the selected bins across
    // the full occupied footprint instead of taking the first row only.
    for (std::size_t index = 0U; index < take; ++index) {
      const std::size_t slot =
        (index * non_empty.size()) / take;
      points->push_back(support[bin_order[slot].front()]);
    }
    return;
  }

  std::size_t emitted = 0U;
  for (std::size_t round = 0U; round < max_bin_size && emitted < take;
    ++round)
  {
    for (std::size_t slot = 0U;
      slot < non_empty.size() && emitted < take; ++slot)
    {
      if (round >= bin_order[slot].size()) {
        continue;
      }
      points->push_back(support[bin_order[slot][round]]);
      ++emitted;
    }
  }
}

// Real shared-boundary samples for adjacent fine cells: two edge endpoints plus
// the midpoint for an orthogonal neighbour, or the single shared corner for a
// diagonal neighbour. Representative positions are used only for the centre;
// the tangent offsets are the actual cell resolution, so a midpoint-only
// agreement can no longer hide an edge split.
std::array<BoundarySample, 3> SharedBoundarySamples(
  const TerrainNode & source, const TerrainNode & target,
  int direction_x, int direction_y, double resolution,
  std::size_t * sample_count)
{
  std::array<BoundarySample, 3> samples{};
  *sample_count = 1U;
  samples[0] = BoundarySample{
    0.5 * (source.representative.x + target.representative.x),
    0.5 * (source.representative.y + target.representative.y)};
  if (direction_x == 0 || direction_y == 0) {
    const double half = 0.5 * std::max(resolution, 0.0);
    const double tangent_x = direction_y != 0 ? 1.0 : 0.0;
    const double tangent_y = direction_x != 0 ? 1.0 : 0.0;
    samples[1] = BoundarySample{
      samples[0].x - tangent_x * half,
      samples[0].y - tangent_y * half};
    samples[2] = BoundarySample{
      samples[0].x + tangent_x * half,
      samples[0].y + tangent_y * half};
    *sample_count = 3U;
  }
  return samples;
}

bool CandidateGeometry(
  const offline_geometry::PlaneFit & fit, Point3 * centroid,
  Vector3 * normal, double * offset)
{
  if (!fit.valid || !fit.centroid.IsFinite() || !fit.normal.IsFinite()) {
    return false;
  }
  const double norm = fit.normal.Norm();
  if (!std::isfinite(norm) || norm <= kEpsilon) {
    return false;
  }
  const Vector3 unit{
    fit.normal.x / norm, fit.normal.y / norm, fit.normal.z / norm};
  if (unit.z < 0.0 || !std::isfinite(unit.z) || unit.z <= kEpsilon) {
    return false;
  }
  const double plane_offset = Dot(
    unit, Vector3{
        fit.centroid.x, fit.centroid.y, fit.centroid.z});
  if (!std::isfinite(plane_offset)) {
    return false;
  }
  *centroid = fit.centroid;
  *normal = unit;
  *offset = plane_offset;
  return true;
}

double OwnSupportRms(
  const TerrainNode & node, const Vector3 & normal,
  double offset)
{
  if (node.support.points.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum = 0.0;
  std::size_t count = 0U;
  for (const Point3 & point : node.support.points) {
    if (!point.IsFinite()) {
      continue;
    }
    const double predicted =
      (offset - normal.x * point.x - normal.y * point.y) / normal.z;
    if (!std::isfinite(predicted)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double residual = point.z - predicted;
    sum += residual * residual;
    ++count;
  }
  if (count == 0U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::sqrt(sum / static_cast<double>(count));
}

void RecordOriginalGeometry(
  const TerrainNode & node, SurfaceReconstructionEvidence * evidence)
{
  evidence->original_centroid = node.plane.centroid;
  evidence->original_normal = node.plane.normal;
  evidence->original_height_m = node.representative.z;
  evidence->original_rms_residual_m = node.plane.rms_residual;
}

void RecordCandidateGeometry(
  const Point3 & centroid, const Vector3 & normal,
  const SurfaceReconstructionEvidence & original,
  SurfaceReconstructionEvidence * evidence)
{
  evidence->reconstructed_centroid = centroid;
  evidence->reconstructed_normal = normal;
  evidence->reconstructed_height_m = centroid.z;
  evidence->correction_m = centroid.z - original.original_height_m;
}

double Confidence(
  double own_rms, double max_residual, std::size_t support_nodes,
  std::size_t directions)
{
  const double scale = std::max(max_residual, kEpsilon);
  const double residual_quality =
    std::clamp(1.0 - own_rms / scale, 0.0, 1.0);
  const double direction_quality =
    std::min(
    1.0, static_cast<double>(
      std::min(directions, std::size_t{8})) / 8.0);
  const double node_quality = support_nodes == 0U ? 0.0 : 1.0;
  return residual_quality * direction_quality * node_quality;
}

// One adjustable cell in the common-edge regularizer. The plane is represented
// at the fixed lattice centre as z = h + (p/L)*(x-cx) + (q/L)*(y-cy), so h and
// the two tangent slopes p,q are directly comparable local unknowns. The data
// priors are the raw same-XY height and the already accepted robust normal; the
// starting point is the bounded neighbourhood estimate from the first pass.
struct JointCellState
{
  std::size_t node_index{0};
  double centre_x{0.0};
  double centre_y{0.0};
  double height{0.0};
  double slope_x{0.0};        // (dz/dx) * cell_resolution
  double slope_y{0.0};        // (dz/dy) * cell_resolution
  double height_prior{0.0};   // raw original height at the same XY
  double slope_x_prior{0.0};  // robust neighbourhood normal, scaled
  double slope_y_prior{0.0};
  double original_height{0.0};
  Vector3 original_normal{0.0, 0.0, 1.0};
};

// A same-layer common edge selected from the original support map. Endpoints
// and midpoint are expressed as dimensionless offsets from each fixed cell
// centre; the diagonal case contributes its one shared corner only.
struct JointEdgeSample
{
  double first_u{0.0};
  double first_v{0.0};
  double second_u{0.0};
  double second_v{0.0};
};

struct JointEdge
{
  std::size_t first{0};
  std::size_t second{0};
  std::size_t sample_count{0};
  std::array<JointEdgeSample, 3> samples{};
};

// Small deterministic 3x3 solve (no global dense matrix). The prior terms make
// the system positive definite; a singular/degenerate solve is simply skipped
// and the previous valid estimate is retained.
bool SolveJointSystem(
  const double matrix[3][3], const double rhs[3], double solution[3])
{
  double augmented[3][4]{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      augmented[row][column] = matrix[row][column];
    }
    augmented[row][3] = rhs[row];
  }
  for (std::size_t column = 0U; column < 3U; ++column) {
    std::size_t pivot = column;
    double best = std::fabs(augmented[column][column]);
    for (std::size_t row = column + 1U; row < 3U; ++row) {
      const double candidate = std::fabs(augmented[row][column]);
      if (candidate > best) {
        best = candidate;
        pivot = row;
      }
    }
    if (!std::isfinite(best) || best <= 1.0e-12) {
      return false;
    }
    if (pivot != column) {
      for (std::size_t entry = column; entry < 4U; ++entry) {
        std::swap(augmented[column][entry], augmented[pivot][entry]);
      }
    }
    const double divisor = augmented[column][column];
    if (!std::isfinite(divisor) || std::abs(divisor) <= 1.0e-12) {
      return false;
    }
    for (std::size_t entry = column; entry < 4U; ++entry) {
      augmented[column][entry] /= divisor;
    }
    for (std::size_t row = 0U; row < 3U; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = augmented[row][column];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t entry = column; entry < 4U; ++entry) {
        augmented[row][entry] -= factor * augmented[column][entry];
      }
    }
  }
  for (std::size_t row = 0U; row < 3U; ++row) {
    solution[row] = augmented[row][3];
    if (!std::isfinite(solution[row])) {
      return false;
    }
  }
  return true;
}

bool JointPlane(
  const JointCellState & state, double resolution, Point3 * centroid,
  Vector3 * normal, double * offset, double * slope)
{
  if (centroid == nullptr || normal == nullptr || offset == nullptr ||
    slope == nullptr || !std::isfinite(resolution) || resolution <= 0.0)
  {
    return false;
  }
  const double a = state.slope_x / resolution;
  const double b = state.slope_y / resolution;
  const double norm = std::sqrt(a * a + b * b + 1.0);
  if (!std::isfinite(norm) || norm <= kEpsilon) {
    return false;
  }
  const Vector3 unit{-a / norm, -b / norm, 1.0 / norm};
  if (!unit.IsFinite() || unit.z <= kEpsilon) {
    return false;
  }
  const Point3 centre{state.centre_x, state.centre_y, state.height};
  const double plane_offset = Dot(
    unit, Vector3{centre.x, centre.y, centre.z});
  const double candidate_slope =
    std::acos(std::clamp(unit.z, -1.0, 1.0));
  if (!centre.IsFinite() || !std::isfinite(plane_offset) ||
    !std::isfinite(candidate_slope))
  {
    return false;
  }
  *centroid = centre;
  *normal = unit;
  *offset = plane_offset;
  *slope = candidate_slope;
  return true;
}

// Every regularizer limit is evaluated absolutely against the untouched input
// node, never accumulated round by round. A candidate that loses a raw-support,
// normal, slope or same-XY correction gate is discarded and the previous valid
// estimate remains in force.
bool JointCandidateAllowed(
  const TerrainNode & original, const JointCellState & state,
  double resolution, const GroundSurfaceReconstructionOptions & options,
  Point3 * centroid, Vector3 * normal, double * offset, double * own_rms)
{
  double slope = 0.0;
  if (!JointPlane(state, resolution, centroid, normal, offset, &slope)) {
    return false;
  }
  const double correction = state.height - state.original_height;
  if (!std::isfinite(correction) ||
    std::abs(correction) > options.max_correction_m + kEpsilon)
  {
    return false;
  }
  const double normal_angle =
    VectorAngle(state.original_normal, *normal);
  if (!std::isfinite(normal_angle) ||
    normal_angle > options.max_normal_correction_rad + kEpsilon)
  {
    return false;
  }
  if (!std::isfinite(slope) ||
    slope > options.max_candidate_slope_rad + kEpsilon)
  {
    return false;
  }
  const double rms = OwnSupportRms(original, *normal, *offset);
  if (!std::isfinite(rms) ||
    rms > options.max_support_residual_m + kEpsilon)
  {
    return false;
  }
  *own_rms = rms;
  return true;
}

// Counts the original raw points that the final plane explains. This count is
// allowed only as node metadata; raw support points and patch fits are never
// rewritten. The returned value is capped by the real bounded joint sample so
// the existing provenance bound inlier_count <= support_points still holds.
std::size_t RawInlierCount(
  const TerrainNode & node, const Vector3 & normal, double offset,
  double max_residual, std::size_t cap)
{
  if (cap == 0U || !std::isfinite(max_residual) ||
    !std::isfinite(offset) || !normal.IsFinite() ||
    normal.z <= kEpsilon)
  {
    return 0U;
  }
  std::size_t inliers = 0U;
  for (const Point3 & point : node.support.points) {
    if (!point.IsFinite()) {
      continue;
    }
    const double predicted =
      (offset - normal.x * point.x - normal.y * point.y) / normal.z;
    if (std::isfinite(predicted) &&
      std::abs(point.z - predicted) <= max_residual + kEpsilon)
    {
      ++inliers;
    }
  }
  return std::min(inliers, cap);
}

}  // namespace

GroundSurfaceReconstructionMetrics ReconstructGroundSurfaces(
  const std::vector<TerrainNode> & nodes,
  const std::vector<MeasurementPatch> & patches,
  const GroundSurfaceReconstructionOptions & options,
  double cell_resolution_m,
  std::vector<TerrainNode> * reconstructed_nodes,
  std::vector<MeasurementPatch> * reconstructed_patches)
{
  GroundSurfaceReconstructionMetrics metrics;
  if (reconstructed_nodes == nullptr || reconstructed_patches == nullptr) {
    return metrics;
  }

  // The caller may pass the same object as input and output. Keep locally owned
  // copies in that case so no node/patch is read after being overwritten.
  std::vector<TerrainNode> alias_nodes;
  const std::vector<TerrainNode> * source_nodes = &nodes;
  if (reconstructed_nodes == &nodes) {
    alias_nodes = nodes;
    source_nodes = &alias_nodes;
  }
  std::vector<MeasurementPatch> alias_patches;
  const std::vector<MeasurementPatch> * source_patches = &patches;
  if (reconstructed_patches == &patches) {
    alias_patches = patches;
    source_patches = &alias_patches;
  }

  if (!options.enabled) {
    *reconstructed_nodes = *source_nodes;
    *reconstructed_patches = *source_patches;
    return metrics;
  }

  const std::size_t max_neighbors = options.max_neighbors;
  const std::size_t cap = options.max_points_per_node;
  if (max_neighbors == 0U ||
    max_neighbors > kMaxGroundSurfaceNeighbors ||
    cap == 0U || cap > kMaxGroundSurfacePointsPerNode ||
    !std::isfinite(cell_resolution_m) || cell_resolution_m <= 0.0 ||
    !std::isfinite(options.max_correction_m) ||
    options.max_correction_m < 0.0 ||
    !std::isfinite(options.max_normal_correction_rad) ||
    options.max_normal_correction_rad <= 0.0 ||
    options.max_normal_correction_rad >= 0.5 * kPi ||
    !std::isfinite(options.max_association_normal_rad) ||
    options.max_association_normal_rad <= 0.0 ||
    options.max_association_normal_rad >= 0.5 * kPi ||
    !std::isfinite(options.max_support_residual_m) ||
    options.max_support_residual_m <= 0.0 ||
    !std::isfinite(options.max_candidate_slope_rad) ||
    options.max_candidate_slope_rad <= 0.0 ||
    options.max_candidate_slope_rad >= 0.5 * kPi ||
    !std::isfinite(options.max_height_noise_m) ||
    options.max_height_noise_m <= 0.0)
  {
    *reconstructed_nodes = *source_nodes;
    *reconstructed_patches = *source_patches;
    return metrics;
  }

  // One O(N) cell index. Neighbour lookup is then at most eight hash probes per
  // node, never a whole-node scan.
  std::unordered_map<FineCellIndex, std::vector<std::size_t>,
    FineCellIndexHash> cell_nodes;
  cell_nodes.reserve(source_nodes->size() * 2U);
  for (std::size_t index = 0U; index < source_nodes->size(); ++index) {
    cell_nodes[(*source_nodes)[index].cell].push_back(index);
  }

  *reconstructed_nodes = *source_nodes;
  *reconstructed_patches = *source_patches;

  constexpr int kDirectionX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDirectionY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  std::vector<std::size_t> selected;
  selected.reserve(max_neighbors);
  // One bounded same-layer relation set per node, taken from the original
  // support map. The joint regularizer reuses exactly these relations; it never
  // invents a neighbour or bridges a step that the first pass preserved.
  std::vector<std::vector<std::size_t>> selected_neighbors(
    source_nodes->size());
  std::vector<Point3> joint_points;
  joint_points.reserve((max_neighbors + 1U) * cap);

  for (std::size_t node_index = 0U; node_index < source_nodes->size();
    ++node_index)
  {
    const TerrainNode & original = (*source_nodes)[node_index];
    TerrainNode & output = (*reconstructed_nodes)[node_index];
    if (!original.IsSupported()) {
      continue;
    }
    ++metrics.considered_nodes;

    SurfaceReconstructionEvidence & evidence =
      output.surface_reconstruction;
    evidence = SurfaceReconstructionEvidence{};
    evidence.attempted = true;
    RecordOriginalGeometry(original, &evidence);

    if (!original.plane.valid || !original.representative.IsFinite() ||
      !original.plane.normal.IsFinite() ||
      !original.plane.centroid.IsFinite() ||
      original.support.points.empty())
    {
      evidence.status = SurfaceReconstructionStatus::kInsufficientSupport;
      ++metrics.rejected_insufficient_support;
      continue;
    }

    // Select at most one supported node per adjacent cell: the same-layer node
    // whose fitted plane is closest to this plane at the shared boundary.
    selected.clear();
    std::size_t adjacent_supported = 0U;
    for (std::size_t direction = 0U; direction < max_neighbors; ++direction) {
      const FineCellIndex neighbor_cell{
        original.cell.x + kDirectionX[direction],
        original.cell.y + kDirectionY[direction]};
      const auto bucket = cell_nodes.find(neighbor_cell);
      if (bucket == cell_nodes.end()) {
        continue;
      }
      bool has_supported = false;
      std::size_t best_index = 0U;
      double best_delta = std::numeric_limits<double>::infinity();
      for (const std::size_t candidate_index : bucket->second) {
        const TerrainNode & candidate = (*source_nodes)[candidate_index];
        if (!IsSupportedCandidate(candidate)) {
          continue;
        }
        has_supported = true;
        std::size_t sample_count = 0U;
        const std::array<BoundarySample, 3> samples =
          SharedBoundarySamples(
          original, candidate, kDirectionX[direction],
          kDirectionY[direction], cell_resolution_m, &sample_count);
        double max_delta = 0.0;
        bool continuous = true;
        for (std::size_t sample = 0U; sample < sample_count; ++sample) {
          double original_z = 0.0;
          double candidate_z = 0.0;
          if (!PlaneHeightAt(
              original, samples[sample].x, samples[sample].y, &original_z) ||
            !PlaneHeightAt(
              candidate, samples[sample].x, samples[sample].y, &candidate_z))
          {
            continuous = false;
            break;
          }
          const double delta = std::abs(candidate_z - original_z);
          if (!std::isfinite(delta) ||
            delta > options.max_height_noise_m + kEpsilon)
          {
            continuous = false;
            break;
          }
          max_delta = std::max(max_delta, delta);
        }
        if (!continuous) {
          continue;
        }
        const double angle =
          VectorAngle(original.plane.normal, candidate.plane.normal);
        if (!std::isfinite(angle) ||
          angle > options.max_association_normal_rad + kEpsilon)
        {
          continue;
        }
        if (max_delta < best_delta) {
          best_delta = max_delta;
          best_index = candidate_index;
        }
      }
      if (has_supported) {
        ++adjacent_supported;
      }
      if (std::isfinite(best_delta)) {
        selected.push_back(best_index);
      }
    }

    selected_neighbors[node_index] = selected;

    // A real layer/step edge has supported neighbours but none agrees at the
    // shared boundary. It is preserved rather than bridged.
    if (selected.empty()) {
      if (adjacent_supported > 0U) {
        evidence.status = SurfaceReconstructionStatus::kPreservedDiscontinuity;
        ++metrics.preserved_discontinuities;
      } else {
        evidence.status = SurfaceReconstructionStatus::kInsufficientSupport;
        ++metrics.rejected_insufficient_support;
      }
      continue;
    }
    // At least two independent neighbour directions are required so a single
    // one-sided boundary cannot pull the plane across a real edge.
    if (selected.size() < 2U) {
      evidence.status = SurfaceReconstructionStatus::kInsufficientSupport;
      ++metrics.rejected_insufficient_support;
      continue;
    }

    joint_points.clear();
    AppendBalancedSamples(original, cap, &joint_points);
    for (const std::size_t selected_index : selected) {
      AppendBalancedSamples(
        (*source_nodes)[selected_index], cap, &joint_points);
    }
    // Adjacent patches overlap: count each actual observation once rather
    // than overweighting points repeated in several support neighbourhoods.
    std::sort(
      joint_points.begin(), joint_points.end(),
      [](const Point3 & first, const Point3 & second) {
        if (first.x != second.x) {return first.x < second.x;}
        if (first.y != second.y) {return first.y < second.y;}
        return first.z < second.z;
      });
    joint_points.erase(
      std::unique(
        joint_points.begin(), joint_points.end(),
        [](const Point3 & first, const Point3 & second) {
          return first.x == second.x && first.y == second.y &&
          first.z == second.z;
        }),
      joint_points.end());
    if (joint_points.size() < 4U) {
      evidence.status = SurfaceReconstructionStatus::kInsufficientSupport;
      ++metrics.rejected_insufficient_support;
      continue;
    }

    const offline_geometry::RobustPlaneFit robust =
      offline_geometry::FitRobustPlane(
      joint_points, 4U, options.max_support_residual_m);
    offline_geometry::PlaneFit fit;
    if (robust.valid()) {
      fit = robust.plane;
    } else if (!offline_geometry::FitPlane(joint_points, &fit)) {
      evidence.status = SurfaceReconstructionStatus::kGeometryRejected;
      ++metrics.rejected_geometry;
      continue;
    }

    Point3 centroid{};
    Vector3 normal{};
    double offset = 0.0;
    if (!CandidateGeometry(fit, &centroid, &normal, &offset)) {
      evidence.status = SurfaceReconstructionStatus::kGeometryRejected;
      ++metrics.rejected_geometry;
      continue;
    }
    // A fit centroid may lie outside the cell. Sample the plane at the
    // original lattice centre: only height/normal may change, never XY.
    centroid.x = original.representative.x;
    centroid.y = original.representative.y;
    centroid.z =
      (offset - normal.x * centroid.x - normal.y * centroid.y) / normal.z;
    if (!centroid.IsFinite()) {
      evidence.status = SurfaceReconstructionStatus::kGeometryRejected;
      ++metrics.rejected_geometry;
      continue;
    }
    RecordCandidateGeometry(centroid, normal, evidence, &evidence);
    evidence.reconstruction_rms_residual_m = fit.rms;
    evidence.support_nodes = selected.size() + 1U;
    evidence.support_points = joint_points.size();

    const double slope = std::acos(std::clamp(normal.z, -1.0, 1.0));
    if (!std::isfinite(slope) ||
      slope > options.max_candidate_slope_rad + kEpsilon)
    {
      evidence.status = SurfaceReconstructionStatus::kGeometryRejected;
      ++metrics.rejected_geometry;
      continue;
    }

    const double correction =
      std::abs(evidence.correction_m);
    if (!std::isfinite(correction) ||
      correction > options.max_correction_m + kEpsilon)
    {
      evidence.status = SurfaceReconstructionStatus::kCorrectionExceeded;
      ++metrics.rejected_correction;
      continue;
    }

    const double normal_angle =
      VectorAngle(evidence.original_normal, normal);
    if (!std::isfinite(normal_angle) ||
      normal_angle > options.max_normal_correction_rad + kEpsilon)
    {
      evidence.status = SurfaceReconstructionStatus::kNormalExceeded;
      ++metrics.rejected_normal;
      continue;
    }

    const double own_rms = OwnSupportRms(original, normal, offset);
    evidence.own_support_rms_residual_m = own_rms;
    if (!std::isfinite(own_rms) ||
      own_rms > options.max_support_residual_m + kEpsilon)
    {
      evidence.status =
        SurfaceReconstructionStatus::kEvidenceResidualExceeded;
      ++metrics.rejected_evidence;
      continue;
    }

    output.plane.valid = true;
    output.plane.centroid = centroid;
    output.plane.normal = normal;
    output.plane.offset = offset;
    output.plane.rms_residual = fit.rms;
    output.plane.inlier_count = robust.valid() ?
      robust.inliers.size() : joint_points.size();
    output.plane.sample_count = joint_points.size();
    output.representative = centroid;

    evidence.applied = true;
    evidence.status = SurfaceReconstructionStatus::kApplied;
    evidence.confidence = Confidence(
      own_rms, options.max_support_residual_m,
      evidence.support_nodes, selected.size());
    ++metrics.applied_nodes;
    metrics.max_applied_correction_m = std::max(
      metrics.max_applied_correction_m, correction);
  }

  // ---------------------------------------------------------------------
  // Explicit same-layer common-edge joint regularization.
  //
  // The first pass still owns the bounded neighbourhood fit. This pass only
  // rebalances those accepted estimates around one local low-dimensional least
  // squares problem with 3 unknowns per cell: centre height h and the two
  // tangent slopes (p,q), with z = h + (p/L)(x-cx) + (q/L)(y-cy).
  //
  // Objective per cell:
  //   w_h (h - h_raw)^2 + w_s ((p-p0)^2 + (q-q0)^2)
  //   + sum over incident same-layer edges, at edge endpoints and midpoint,
  //     w_e (z_i(s) - z_j(s))^2.
  // The height prior is the original same-XY raw height; the slope prior is the
  // already accepted robust neighbourhood normal. Edges are exactly the
  // same-layer relations selected from the original support map. A
  // discontinuity has no edge term, so its original height step is protected
  // rather than flattened; no bridge-under point can be moved onto a bridge
  // deck because no new neighbour relation is created here.
  //
  // This is deterministic block Gauss-Seidel over at most eight rounds, with a
  // fresh 3x3 solve per cell and no global dense matrix. Every candidate is
  // checked absolutely against the original node for same-XY correction,
  // original-normal change, candidate slope and raw own-support RMS. A failed
  // update keeps the previous valid estimate.
  constexpr std::size_t kJointRegularizationRounds = 8U;
  constexpr double kHeightPriorWeight = 1.0;
  constexpr double kSlopePriorWeight = 1.0;
  constexpr double kCommonEdgeWeight = 1.0;

  metrics.max_applied_correction_m = 0.0;
  std::vector<std::size_t> joint_nodes;
  std::vector<int> joint_local(source_nodes->size(), -1);
  for (std::size_t index = 0U; index < reconstructed_nodes->size(); ++index) {
    if ((*reconstructed_nodes)[index].surface_reconstruction.applied) {
      joint_local[index] = static_cast<int>(joint_nodes.size());
      joint_nodes.push_back(index);
    }
  }

  if (!joint_nodes.empty()) {
    std::vector<JointCellState> joint_states(joint_nodes.size());
    for (std::size_t local = 0U; local < joint_nodes.size(); ++local) {
      const std::size_t node_index = joint_nodes[local];
      const TerrainNode & initial = (*reconstructed_nodes)[node_index];
      const SurfaceReconstructionEvidence & evidence =
        initial.surface_reconstruction;
      JointCellState & state = joint_states[local];
      state.node_index = node_index;
      state.centre_x = initial.representative.x;
      state.centre_y = initial.representative.y;
      state.height = initial.representative.z;
      state.original_height = evidence.original_height_m;
      state.original_normal = evidence.original_normal;
      state.height_prior = evidence.original_height_m;
      const double slope_x =
        -initial.plane.normal.x / initial.plane.normal.z;
      const double slope_y =
        -initial.plane.normal.y / initial.plane.normal.z;
      state.slope_x = slope_x * cell_resolution_m;
      state.slope_y = slope_y * cell_resolution_m;
      state.slope_x_prior = state.slope_x;
      state.slope_y_prior = state.slope_y;
    }

    // Build the undirected same-layer edge list from the first-pass selection.
    // The relation is added once even when both endpoints selected each other.
    std::vector<JointEdge> joint_edges;
    std::vector<std::vector<std::size_t>> incident(
      joint_states.size());
    for (std::size_t local_a = 0U; local_a < joint_states.size(); ++local_a) {
      const std::size_t node_i = joint_nodes[local_a];
      for (const std::size_t node_j : selected_neighbors[node_i]) {
        const int local_b_value = joint_local[node_j];
        if (local_b_value < 0) {
          continue;
        }
        const std::size_t local_b =
          static_cast<std::size_t>(local_b_value);
        if (local_a == local_b) {
          continue;
        }
        const bool reverse_selected =
          std::find(
          selected_neighbors[node_j].begin(),
          selected_neighbors[node_j].end(), node_i) !=
          selected_neighbors[node_j].end();
        if (local_a > local_b && reverse_selected) {
          continue;
        }
        const TerrainNode & first_node = (*source_nodes)[node_i];
        const TerrainNode & second_node = (*source_nodes)[node_j];
        const int direction_x = static_cast<int>(
          second_node.cell.x - first_node.cell.x);
        const int direction_y = static_cast<int>(
          second_node.cell.y - first_node.cell.y);
        if (direction_x < -1 || direction_x > 1 ||
          direction_y < -1 || direction_y > 1)
        {
          continue;
        }
        std::size_t sample_count = 0U;
        const std::array<BoundarySample, 3> samples =
          SharedBoundarySamples(
          first_node, second_node, direction_x, direction_y,
          cell_resolution_m, &sample_count);
        JointEdge edge;
        edge.first = local_a;
        edge.second = local_b;
        edge.sample_count = sample_count;
        const JointCellState & first_state = joint_states[local_a];
        const JointCellState & second_state = joint_states[local_b];
        for (std::size_t sample = 0U; sample < sample_count; ++sample) {
          edge.samples[sample].first_u =
            (samples[sample].x - first_state.centre_x) /
            cell_resolution_m;
          edge.samples[sample].first_v =
            (samples[sample].y - first_state.centre_y) /
            cell_resolution_m;
          edge.samples[sample].second_u =
            (samples[sample].x - second_state.centre_x) /
            cell_resolution_m;
          edge.samples[sample].second_v =
            (samples[sample].y - second_state.centre_y) /
            cell_resolution_m;
        }
        const std::size_t edge_index = joint_edges.size();
        joint_edges.push_back(edge);
        incident[local_a].push_back(edge_index);
        incident[local_b].push_back(edge_index);
      }
    }

    for (std::size_t round = 0U; round < kJointRegularizationRounds; ++round) {
      double max_change = 0.0;
      for (std::size_t local = 0U; local < joint_states.size(); ++local) {
        JointCellState & state = joint_states[local];
        double matrix[3][3] = {
          {kHeightPriorWeight, 0.0, 0.0},
          {0.0, kSlopePriorWeight, 0.0},
          {0.0, 0.0, kSlopePriorWeight}};
        double rhs[3] = {
          kHeightPriorWeight * state.height_prior,
          kSlopePriorWeight * state.slope_x_prior,
          kSlopePriorWeight * state.slope_y_prior};
        for (const std::size_t edge_index : incident[local]) {
          const JointEdge & edge = joint_edges[edge_index];
          const bool is_first = edge.first == local;
          const std::size_t other_index =
            is_first ? edge.second : edge.first;
          const JointCellState & other = joint_states[other_index];
          for (std::size_t sample = 0U; sample < edge.sample_count; ++sample) {
            const double u = is_first ?
              edge.samples[sample].first_u :
              edge.samples[sample].second_u;
            const double v = is_first ?
              edge.samples[sample].first_v :
              edge.samples[sample].second_v;
            const double other_u = is_first ?
              edge.samples[sample].second_u :
              edge.samples[sample].first_u;
            const double other_v = is_first ?
              edge.samples[sample].second_v :
              edge.samples[sample].first_v;
            const double other_height = other.height +
              other.slope_x * other_u + other.slope_y * other_v;
            const double jacobian[3] = {1.0, u, v};
            for (std::size_t row = 0U; row < 3U; ++row) {
              for (std::size_t column = 0U; column < 3U; ++column) {
                matrix[row][column] +=
                  kCommonEdgeWeight * jacobian[row] * jacobian[column];
              }
              rhs[row] += kCommonEdgeWeight * jacobian[row] * other_height;
            }
          }
        }
        double solution[3] = {0.0, 0.0, 0.0};
        if (!SolveJointSystem(matrix, rhs, solution)) {
          continue;
        }
        JointCellState candidate = state;
        candidate.height = solution[0];
        candidate.slope_x = solution[1];
        candidate.slope_y = solution[2];
        Point3 centroid{};
        Vector3 normal{};
        double offset = 0.0;
        double own_rms = 0.0;
        const TerrainNode & original = (*source_nodes)[state.node_index];
        if (!JointCandidateAllowed(
            original, candidate, cell_resolution_m, options, &centroid,
            &normal, &offset, &own_rms))
        {
          continue;
        }
        max_change = std::max(
          max_change,
          std::abs(candidate.height - state.height) +
          std::abs(candidate.slope_x - state.slope_x) +
          std::abs(candidate.slope_y - state.slope_y));
        state = candidate;
      }
      if (max_change <= 1.0e-10) {
        break;
      }
    }

    // Publish the true final plane and real raw-support evidence. The virtual
    // edge samples only influenced the solve; they never become support points,
    // patch fits, inlier refs or a claimed measured RMS.
    for (const JointCellState & state : joint_states) {
      const std::size_t node_index = state.node_index;
      const TerrainNode & original = (*source_nodes)[node_index];
      TerrainNode & output = (*reconstructed_nodes)[node_index];
      SurfaceReconstructionEvidence & evidence =
        output.surface_reconstruction;
      Point3 centroid{};
      Vector3 normal{};
      double offset = 0.0;
      double own_rms = 0.0;
      if (!JointCandidateAllowed(
          original, state, cell_resolution_m, options, &centroid, &normal,
          &offset, &own_rms))
      {
        // The starting estimate was already gated, so this is only a defensive
        // fallback; keep the untouched initial accepted metadata.
        metrics.max_applied_correction_m = std::max(
          metrics.max_applied_correction_m,
          std::abs(evidence.correction_m));
        continue;
      }
      const std::size_t raw_inliers = RawInlierCount(
        original, normal, offset, options.max_support_residual_m,
        original.support.points.size());
      if (raw_inliers < 4U) {
        // Insufficient observations cannot be repaired by inflating a count.
        // Keep the previously verified neighbourhood estimate and metadata.
        metrics.max_applied_correction_m = std::max(
          metrics.max_applied_correction_m,
          std::abs(evidence.correction_m));
        continue;
      }
      output.plane.valid = true;
      output.plane.centroid = centroid;
      output.plane.normal = normal;
      output.plane.offset = offset;
      output.plane.rms_residual = own_rms;
      output.plane.inlier_count = raw_inliers;
      output.plane.sample_count = original.support.points.size();
      output.representative = centroid;

      evidence.reconstructed_centroid = centroid;
      evidence.reconstructed_normal = normal;
      evidence.reconstructed_height_m = centroid.z;
      evidence.correction_m = centroid.z - evidence.original_height_m;
      evidence.reconstruction_rms_residual_m = own_rms;
      evidence.own_support_rms_residual_m = own_rms;
      // The final RMS/inlier summary uses these actual raw support samples;
      // virtual edge constraints never contribute to observation counts.
      evidence.support_points = original.support.points.size();
      evidence.confidence = Confidence(
        own_rms, options.max_support_residual_m,
        evidence.support_nodes,
        selected_neighbors[node_index].size());
      metrics.max_applied_correction_m = std::max(
        metrics.max_applied_correction_m,
        std::abs(evidence.correction_m));
    }
  }

  // Mirror the exact node provenance into every linked patch. Raw patch fit and
  // all point-index/support references remain untouched because only the
  // dedicated provenance member is assigned.
  std::unordered_map<TerrainNodeId, std::size_t> node_by_id;
  node_by_id.reserve(reconstructed_nodes->size());
  for (std::size_t index = 0U; index < reconstructed_nodes->size(); ++index) {
    node_by_id[(*reconstructed_nodes)[index].id] = index;
  }
  for (MeasurementPatch & patch : *reconstructed_patches) {
    const auto found = node_by_id.find(patch.node_id);
    if (found == node_by_id.end()) {
      continue;
    }
    patch.surface_reconstruction =
      (*reconstructed_nodes)[found->second].surface_reconstruction;
  }

  return metrics;
}

}  // namespace dddmr_grid_global_planner
