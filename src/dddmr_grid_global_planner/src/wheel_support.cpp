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

#include "dddmr_grid_global_planner/wheel_support.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/continuous_ground_surface.h"
#include "dddmr_grid_global_planner/terrain_boundary_cost.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

const char * const kWheelNames[kWheelCount] = {"front_left", "front_right",
  "rear_left", "rear_right"};

// Rotates a base-frame vector by roll about +X and then pitch about +Y. Yaw is
// applied separately by the caller because it does not change the world Z.
Vector3 RotateBody(double roll, double pitch, const Vector3 & point) noexcept
{
  const double cos_roll = std::cos(roll);
  const double sin_roll = std::sin(roll);
  const double cos_pitch = std::cos(pitch);
  const double sin_pitch = std::sin(pitch);
  const double y_rolled = point.y * cos_roll - point.z * sin_roll;
  const double z_rolled = point.y * sin_roll + point.z * cos_roll;
  return Vector3{
    point.x * cos_pitch + z_rolled * sin_pitch,
    y_rolled,
    -point.x * sin_pitch + z_rolled * cos_pitch};
}

// Applies yaw to an XY vector.
Vector3 RotateYaw(double yaw, const Vector3 & value) noexcept
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return Vector3{value.x * cos_yaw - value.y * sin_yaw,
    value.x * sin_yaw + value.y * cos_yaw, value.z};
}

// Transforms one base-frame point into world coordinates.
Point3 BodyToWorld(const VehiclePose & pose, const Point3 & point) noexcept
{
  const Vector3 rotated =
    RotateYaw(
    pose.yaw, RotateBody(
      pose.roll, pose.pitch,
      Vector3{point.x, point.y, point.z}));
  return Point3{pose.base_origin.x + rotated.x,
    pose.base_origin.y + rotated.y,
    pose.base_origin.z + rotated.z};
}

// Returns the wheelbase measured between front and rear axle midpoints, falling
// back to the explicit configured value only when the contact offsets are not
// yet usable (for example during partial setup).
double EffectiveWheelbaseM(const VehicleCapabilityConfig & vehicle) noexcept
{
  const double measured = vehicle.MeasuredWheelbaseM();
  if (std::isfinite(measured) && measured > 0.0) {
    return measured;
  }
  return std::isfinite(vehicle.wheelbase_m) && vehicle.wheelbase_m > 0.0 ?
         vehicle.wheelbase_m :
         0.0;
}

// Returns the front track used by Ackermann geometry. The independent field is
// authoritative; the legacy aggregate field is only a compatibility fallback.
double EffectiveFrontTrackWidthM(
  const VehicleCapabilityConfig & vehicle) noexcept
{
  if (std::isfinite(vehicle.front_track_width_m) &&
    vehicle.front_track_width_m > 0.0)
  {
    return vehicle.front_track_width_m;
  }
  return vehicle.track_width_m;
}

// Returns the configured contact-patch length, or twice the measured tire
// radius when no explicit patch length is available.
double ConfiguredContactPatchLength(
  const VehicleCapabilityConfig & vehicle) noexcept
{
  if (std::isfinite(vehicle.contact_patch_length_m) &&
    vehicle.contact_patch_length_m > 0.0)
  {
    return vehicle.contact_patch_length_m;
  }
  if (std::isfinite(vehicle.wheel_radius_m) && vehicle.wheel_radius_m > 0.0) {
    return 2.0 * vehicle.wheel_radius_m;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// Returns the configured contact-patch width, which is the measured tire width.
double ConfiguredContactPatchWidth(
  const VehicleCapabilityConfig & vehicle) noexcept
{
  return vehicle.tire_width_m;
}

// One oriented tire contact plane. The along/across axes include the complete
// body roll/pitch/yaw attitude and the front steering angle.
struct WheelContactFrame
{
  Point3 center{};
  Vector3 along{};
  Vector3 across{};
  double half_length{0.0};
  double half_width{0.0};
  double contact_z{0.0};
};

// Builds the full-attitude contact frame for one wheel. Returns false when the
// patch dimensions are not configured or any transformed quantity is non-finite.
bool BuildWheelContactFrame(
  const VehicleCapabilityConfig & vehicle,
  const VehiclePose & pose, std::size_t wheel,
  double wheel_steer, double patch_length,
  double patch_width,
  WheelContactFrame * frame) noexcept
{
  if (frame == nullptr) {
    return false;
  }
  const double cos_steer = std::cos(wheel_steer);
  const double sin_steer = std::sin(wheel_steer);
  const Vector3 body_along{cos_steer, sin_steer, 0.0};
  const Vector3 body_across{-sin_steer, cos_steer, 0.0};
  const Vector3 along =
    RotateYaw(pose.yaw, RotateBody(pose.roll, pose.pitch, body_along));
  const Vector3 across =
    RotateYaw(pose.yaw, RotateBody(pose.roll, pose.pitch, body_across));
  const Point3 center =
    BodyToWorld(pose, vehicle.wheel_contact_offsets[wheel]);
  if (!center.IsFinite() || !along.IsFinite() || !across.IsFinite() ||
    !std::isfinite(patch_length) || patch_length <= 0.0 ||
    !std::isfinite(patch_width) || patch_width <= 0.0)
  {
    return false;
  }
  frame->center = center;
  frame->along = along;
  frame->across = across;
  frame->half_length = 0.5 * patch_length;
  frame->half_width = 0.5 * patch_width;
  frame->contact_z = center.z;
  return true;
}

// True when a world point lies inside the actual configured tire footprint at
// the supplied contact height. The vertical band is a validated contact-plane
// tolerance, never an arbitrary body/obstacle exemption.
bool InsideTireContactVolume(
  const WheelContactFrame & frame,
  const Point3 & world, double vertical_band) noexcept
{
  if (!world.IsFinite() || !std::isfinite(vertical_band) ||
    vertical_band < 0.0)
  {
    return false;
  }
  if (std::abs(world.z - frame.contact_z) > vertical_band) {
    return false;
  }
  const Vector3 delta{world.x - frame.center.x, world.y - frame.center.y,
    world.z - frame.center.z};
  const double along = Dot(delta, frame.along);
  const double across = Dot(delta, frame.across);
  return std::abs(along) <= frame.half_length + kEpsilon &&
         std::abs(across) <= frame.half_width + kEpsilon;
}

// Returns the conservative contact-plane vertical band used by the narrow tire
// legal-ground exemption. It is tied to the validated support tolerance.
double TireContactBandM(const WheelSupportOptions & options) noexcept
{
  return std::max(options.branch_height_tolerance_m, 1.0e-3);
}

// Returns the plane height at one XY point. Invalid/degenerate planes fall back
// to the representative point, which is finite by snapshot contract.
double PlaneHeight(const TerrainNode & node, double x, double y) noexcept
{
  // Shared reconstructed corners are the planning surface, not just a display
  // artifact. TerrainSurfaceHeight falls back to the fitted plane internally.
  const double z = TerrainSurfaceHeight(node, x, y);
  if (std::isfinite(z)) {
    return z;
  }
  return node.representative.z;
}

// Solves a 3x3 linear system with partial pivoting. Returns false on a
// degenerate system; no arbitrary regularization is hidden in the solve.
bool Solve3x3(
  const double matrix[3][3], const double rhs[3],
  double solution[3]) noexcept
{
  double augmented[3][4] = {
    {matrix[0][0], matrix[0][1], matrix[0][2], rhs[0]},
    {matrix[1][0], matrix[1][1], matrix[1][2], rhs[1]},
    {matrix[2][0], matrix[2][1], matrix[2][2], rhs[2]},
  };
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    double pivot_abs = std::abs(augmented[pivot][column]);
    for (int row = column + 1; row < 3; ++row) {
      const double candidate = std::abs(augmented[row][column]);
      if (candidate > pivot_abs) {
        pivot = row;
        pivot_abs = candidate;
      }
    }
    if (pivot_abs < 1.0e-14) {
      return false;
    }
    if (pivot != column) {
      for (int k = column; k < 4; ++k) {
        std::swap(augmented[pivot][k], augmented[column][k]);
      }
    }
    const double diagonal = augmented[column][column];
    for (int row = column + 1; row < 3; ++row) {
      const double factor = augmented[row][column] / diagonal;
      for (int k = column; k < 4; ++k) {
        augmented[row][k] -= factor * augmented[column][k];
      }
    }
  }
  for (int row = 2; row >= 0; --row) {
    double value = augmented[row][3];
    for (int column = row + 1; column < 3; ++column) {
      value -= augmented[row][column] * solution[column];
    }
    solution[row] = value / augmented[row][row];
  }
  return std::isfinite(solution[0]) && std::isfinite(solution[1]) &&
         std::isfinite(solution[2]);
}

// Solves z0/roll/pitch from four contact heights with bounded Gauss-Newton
// iterations. The model is the exact rigid-body Z of each wheel contact.
struct PoseFit
{
  bool ok{false};
  double z0{0.0};
  double roll{0.0};
  double pitch{0.0};
  double residual_rms{0.0};
  double max_abs_residual{0.0};
  double twist{0.0};
  std::string detail;
};

PoseFit FitVehiclePose(
  const std::array<Point3, kWheelCount> & offsets,
  const std::array<double, kWheelCount> & contact_z, std::size_t max_iterations,
  double step_tolerance)
{
  PoseFit fit;
  double z0 = 0.0;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    z0 += contact_z[wheel] - offsets[wheel].z;
  }
  z0 /= static_cast<double>(kWheelCount);
  double roll = 0.0;
  double pitch = 0.0;

  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    double normal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Point3 & p = offsets[wheel];
      const double cos_roll = std::cos(roll);
      const double sin_roll = std::sin(roll);
      const double cos_pitch = std::cos(pitch);
      const double sin_pitch = std::sin(pitch);
      const double rolled_y = p.y * sin_roll + p.z * cos_roll;
      const double predicted = z0 - p.x * sin_pitch + rolled_y * cos_pitch;
      const double residual = contact_z[wheel] - predicted;
      const double jacobian[3] = {
        1.0, (p.y * cos_roll - p.z * sin_roll) * cos_pitch,
        -p.x * cos_pitch - rolled_y * sin_pitch};
      for (int row = 0; row < 3; ++row) {
        rhs[row] += jacobian[row] * residual;
        for (int column = 0; column < 3; ++column) {
          normal[row][column] += jacobian[row] * jacobian[column];
        }
      }
    }
    for (int diagonal = 0; diagonal < 3; ++diagonal) {
      normal[diagonal][diagonal] += 1.0e-12;
    }
    double step[3] = {0.0, 0.0, 0.0};
    if (!Solve3x3(normal, rhs, step)) {
      fit.detail = "pose normal equations are degenerate";
      return fit;
    }
    z0 += step[0];
    roll += step[1];
    pitch += step[2];
    if (!std::isfinite(z0) || !std::isfinite(roll) ||
      !std::isfinite(pitch))
    {
      fit.detail = "pose solve produced a non-finite state";
      return fit;
    }
    if (std::abs(step[0]) + std::abs(step[1]) + std::abs(step[2]) <
      step_tolerance)
    {
      break;
    }
  }

  double sum_squared = 0.0;
  double min_residual = std::numeric_limits<double>::infinity();
  double max_residual = -std::numeric_limits<double>::infinity();
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    const Point3 & p = offsets[wheel];
    const double cos_roll = std::cos(roll);
    const double sin_roll = std::sin(roll);
    const double cos_pitch = std::cos(pitch);
    const double sin_pitch = std::sin(pitch);
    const double rolled_y = p.y * sin_roll + p.z * cos_roll;
    const double predicted = z0 - p.x * sin_pitch + rolled_y * cos_pitch;
    const double residual = contact_z[wheel] - predicted;
    sum_squared += residual * residual;
    min_residual = std::min(min_residual, residual);
    max_residual = std::max(max_residual, residual);
  }
  fit.ok = true;
  fit.z0 = z0;
  fit.roll = roll;
  fit.pitch = pitch;
  fit.residual_rms =
    std::sqrt(sum_squared / static_cast<double>(kWheelCount));
  fit.max_abs_residual =
    std::max(std::abs(min_residual), std::abs(max_residual));
  fit.twist = max_residual - min_residual;
  return fit;
}

struct FootprintSample
{
  Point3 world{};
  FineCellIndex cell{};
  bool center{false};
};

struct WheelCandidate
{
  bool valid{false};
  TerrainNodeId node_id{kInvalidTerrainNodeId};
  FineCellIndex cell{};
  int local_surface_index{-1};
  Point3 wheel_center{};
  double contact_z{0.0};
  double coverage{0.0};
  double sample_min_z{0.0};
  double sample_max_z{0.0};
  bool center_supported{false};
};

// Returns normalized sample offsets in [-0.5, 0.5] that always contain the
// exact geometric center. Even nominal counts therefore produce one extra
// sample instead of designating an off-center sample as the center.
std::vector<double> SymmetricSampleFractions(std::size_t count)
{
  std::vector<double> fractions;
  if (count == 0) {
    return fractions;
  }
  if (count == 1) {
    fractions.push_back(0.0);
    return fractions;
  }
  if (count % 2U != 0U) {
    fractions.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      fractions.push_back(
        static_cast<double>(index) /
        static_cast<double>(count - 1U) -
        0.5);
    }
    return fractions;
  }
  // Include the exact center and an odd number of symmetric offsets.
  fractions.reserve(count + 1U);
  for (std::size_t index = 0; index <= count; ++index) {
    fractions.push_back(
      static_cast<double>(index) /
      static_cast<double>(count) -
      0.5);
  }
  return fractions;
}

// Builds the finite tire footprint samples for one wheel using the complete
// body roll/pitch/yaw attitude and the solved wheel steering angle. Sample
// spacing scales with fine resolution; a footprint smaller than one fine cell is
// conservatively evaluated at one-cell extent so it cannot manufacture coverage
// from a sub-cell point. Those samples are measured by finite evidence, not
// inference from the resolved extent.
std::vector<FootprintSample> BuildFootprint(
  const VehicleCapabilityConfig & vehicle, const WheelSupportOptions & options,
  const VehiclePose & pose, std::size_t wheel, double wheel_steer,
  double resolution, Point3 * wheel_center, std::string * detail)
{
  const double patch_length = ConfiguredContactPatchLength(vehicle);
  const double patch_width = ConfiguredContactPatchWidth(vehicle);
  if (!std::isfinite(patch_length) || patch_length <= 0.0 ||
    !std::isfinite(patch_width) || patch_width <= 0.0)
  {
    *detail = "tire footprint dimensions are not configured";
    return {};
  }
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    *detail = "fine resolution is invalid for footprint sampling";
    return {};
  }
  const std::size_t nominal_along = options.footprint_samples_along;
  const std::size_t nominal_across = options.footprint_samples_across;
  if (nominal_along < 2 || nominal_across < 2) {
    *detail = "footprint sample counts must be at least 2";
    return {};
  }
  const double sampled_length = std::max(patch_length, resolution);
  const double sampled_width = std::max(patch_width, resolution);
  const std::size_t evidence_along =
    static_cast<std::size_t>(std::ceil(sampled_length / resolution)) + 1U;
  const std::size_t evidence_across =
    static_cast<std::size_t>(std::ceil(sampled_width / resolution)) + 1U;
  const std::size_t along = std::max(nominal_along, evidence_along);
  const std::size_t across = std::max(nominal_across, evidence_across);
  const std::vector<double> along_fractions =
    SymmetricSampleFractions(along);
  const std::vector<double> across_fractions =
    SymmetricSampleFractions(across);
  WheelContactFrame frame;
  if (!BuildWheelContactFrame(
      vehicle, pose, wheel, wheel_steer,
      sampled_length, sampled_width, &frame))
  {
    *detail = "wheel contact frame is invalid";
    return {};
  }
  *wheel_center = frame.center;
  std::vector<FootprintSample> samples;
  samples.reserve(along_fractions.size() * across_fractions.size());
  for (const double along_fraction : along_fractions) {
    for (const double across_fraction : across_fractions) {
      FootprintSample sample;
      sample.world = Point3{
        frame.center.x + frame.along.x * sampled_length * along_fraction +
        frame.across.x * sampled_width * across_fraction,
        frame.center.y + frame.along.y * sampled_length * along_fraction +
        frame.across.y * sampled_width * across_fraction,
        frame.center.z + frame.along.z * sampled_length * along_fraction +
        frame.across.z * sampled_width * across_fraction};
      sample.center = std::abs(along_fraction) <= 1.0e-12 &&
        std::abs(across_fraction) <= 1.0e-12;
      if (!sample.world.IsFinite()) {
        *detail = "wheel footprint sample is non-finite";
        return {};
      }
      samples.push_back(sample);
    }
  }
  return samples;
}

// Converts footprints to map cells once the map origin is known.
bool AssignFootprintCells(
  const TerrainSnapshot & terrain,
  std::vector<FootprintSample> * samples,
  std::string * detail)
{
  for (FootprintSample & sample : *samples) {
    const auto cell =
      FineCellOf(
      sample.world, terrain.origin_x(), terrain.origin_y(),
      terrain.resolution());
    if (!cell.has_value()) {
      *detail = "wheel footprint sample is outside the safe cell bound";
      return false;
    }
    sample.cell = *cell;
  }
  return true;
}

// One XY point used by the raw-support convex hull.
struct Point2
{
  double x{0.0};
  double y{0.0};
};

// Signed twice-area cross product for three XY points.
double Cross2(
  const Point2 & first, const Point2 & second,
  const Point2 & third) noexcept
{
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
}

// Builds the convex hull of the finite raw support-point XY projections. The
// hull bounds the plane model so a sample cannot extrapolate beyond actual
// contact evidence.
std::vector<Point2> SupportHullXY(const SupportEvidence & support)
{
  std::vector<Point2> points;
  points.reserve(support.points.size());
  for (const Point3 & point : support.points) {
    if (point.IsFinite()) {
      points.push_back(Point2{point.x, point.y});
    }
  }
  std::sort(
    points.begin(), points.end(),
    [](const Point2 & lhs, const Point2 & rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      return lhs.y < rhs.y;
    });
  if (points.size() <= 2U) {
    return points;
  }
  std::vector<Point2> hull;
  for (const Point2 & point : points) {
    while (hull.size() >= 2U &&
      Cross2(hull[hull.size() - 2U], hull.back(), point) <= 1.0e-15)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (std::size_t index = points.size() - 1U; index > 0U; --index) {
    const Point2 & point = points[index - 1U];
    while (hull.size() > lower_size &&
      Cross2(hull[hull.size() - 2U], hull.back(), point) <= 1.0e-15)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

// True when an XY point is inside (or within a small evidence tolerance of) a
// convex support hull. Degenerate one/two-point evidence is accepted only on
// the point/segment. The tolerance is bounded by the fine resolution and never
// allows an unbounded plane extrapolation.
bool PointInHull(
  const std::vector<Point2> & hull, double x, double y,
  double tolerance) noexcept
{
  if (hull.empty() || !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  const Point2 point{x, y};
  if (hull.size() == 1U) {
    return std::hypot(point.x - hull.front().x, point.y - hull.front().y) <=
           tolerance;
  }
  if (hull.size() == 2U) {
    const double cross =
      std::abs(Cross2(hull[0], hull[1], point));
    const double length =
      std::hypot(hull[1].x - hull[0].x, hull[1].y - hull[0].y);
    if (length <= tolerance || cross > tolerance * length) {
      return false;
    }
    const double dot =
      (point.x - hull[0].x) * (hull[1].x - hull[0].x) +
      (point.y - hull[0].y) * (hull[1].y - hull[0].y);
    return dot >= -tolerance * length &&
           dot <= length * length + tolerance * length;
  }
  for (std::size_t index = 0; index < hull.size(); ++index) {
    const Point2 & first = hull[index];
    const Point2 & second = hull[(index + 1U) % hull.size()];
    const double edge_length =
      std::hypot(second.x - first.x, second.y - first.y);
    // Cross2 is an area (m^2); normalize the metric tolerance by the edge
    // length so a short edge cannot admit a point far outside the hull.
    if (Cross2(first, second, point) < -tolerance * edge_length) {
      return false;
    }
  }
  return true;
}

// Evaluates one support candidate using raw finite contact evidence in each
// sample cell. A sample counts only when a supported node's fitted plane is
// valid, the sample lies inside that node's raw support hull, the node is
// connected to the candidate branch, and the accumulated height is the matched
// node's actual plane height. Center-node height is never substituted for an
// unmatched sample and no in-cell hole is filled.
WheelCandidate EvaluateWheelCandidate(
  const TerrainSnapshot & terrain, const TerrainNode & center_node,
  const Point3 & wheel_center, const std::vector<FootprintSample> & samples,
  const WheelSupportOptions & options)
{
  WheelCandidate candidate;
  candidate.node_id = center_node.id;
  candidate.cell = center_node.cell;
  candidate.local_surface_index = center_node.local_surface_index;
  candidate.wheel_center = wheel_center;
  const double branch_reference_z =
    PlaneHeight(center_node, wheel_center.x, wheel_center.y);
  candidate.contact_z = std::isfinite(branch_reference_z) ?
    branch_reference_z :
    center_node.representative.z;
  double sum_z = 0.0;
  std::size_t matched = 0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  bool center_supported = false;
  std::unordered_map<TerrainNodeId, std::vector<Point2>> hull_cache;
  std::unordered_map<TerrainNodeId, bool> connectivity_cache;
  const double hull_tolerance =
    std::max(1.0e-9, 0.10 * terrain.resolution());
  for (const FootprintSample & sample : samples) {
    const std::vector<TerrainNodeId> * nodes =
      terrain.FindNodesInCell(sample.cell);
    if (nodes == nullptr) {
      continue;
    }
    double best_z = std::numeric_limits<double>::quiet_NaN();
    double best_delta = std::numeric_limits<double>::infinity();
    for (const TerrainNodeId id : *nodes) {
      const TerrainNode * node = terrain.FindNode(id);
      if (node == nullptr || !node->IsSupported() || !node->plane.valid) {
        continue;
      }
      const auto hull_iterator = hull_cache.find(id);
      if (hull_iterator == hull_cache.end()) {
        hull_cache.emplace(id, SupportHullXY(node->support));
      }
      const std::vector<Point2> & hull = hull_cache.at(id);
      if (!PointInHull(
          hull, sample.world.x, sample.world.y,
          hull_tolerance))
      {
        continue;
      }
      const double z = PlaneHeight(*node, sample.world.x, sample.world.y);
      if (!std::isfinite(z)) {
        continue;
      }
      if (id != center_node.id) {
        if (!std::isfinite(branch_reference_z) ||
          std::abs(z - branch_reference_z) >
          options.branch_height_tolerance_m)
        {
          continue;
        }
        const auto cache_iterator = connectivity_cache.find(id);
        bool connected = false;
        if (cache_iterator == connectivity_cache.end()) {
          connected = terrain.AreConnected(
            id, center_node.id,
            options.max_deck_search_nodes);
          connectivity_cache.emplace(id, connected);
        } else {
          connected = cache_iterator->second;
        }
        if (!connected) {
          continue;
        }
      }
      const double delta =
        std::isfinite(branch_reference_z) ?
        std::abs(z - branch_reference_z) :
        0.0;
      if (delta < best_delta) {
        best_delta = delta;
        best_z = z;
      }
    }
    if (std::isfinite(best_z)) {
      ++matched;
      if (sample.center) {
        center_supported = true;
      }
      sum_z += best_z;
      min_z = std::min(min_z, best_z);
      max_z = std::max(max_z, best_z);
    }
  }
  const double total = static_cast<double>(samples.size());
  candidate.coverage =
    total > 0.0 ? static_cast<double>(matched) / total : 0.0;
  candidate.center_supported = center_supported;
  if (matched > 0) {
    candidate.contact_z = sum_z / static_cast<double>(matched);
    candidate.sample_min_z = min_z;
    candidate.sample_max_z = max_z;
    candidate.valid = std::isfinite(candidate.contact_z);
  }
  return candidate;
}

// Conservative vertical interval of one body cell column.
struct BodyCellEnvelope
{
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
};

// True when two convex XY point sets are separated on the supplied axis.
bool SeparatedOnAxis(
  const std::vector<Point2> & first,
  const std::vector<Point2> & second, double axis_x,
  double axis_y) noexcept
{
  const double length = std::hypot(axis_x, axis_y);
  if (length <= kEpsilon) {
    return false;
  }
  const double unit_x = axis_x / length;
  const double unit_y = axis_y / length;
  double first_min = std::numeric_limits<double>::infinity();
  double first_max = -std::numeric_limits<double>::infinity();
  for (const Point2 & point : first) {
    const double projection = point.x * unit_x + point.y * unit_y;
    first_min = std::min(first_min, projection);
    first_max = std::max(first_max, projection);
  }
  double second_min = std::numeric_limits<double>::infinity();
  double second_max = -std::numeric_limits<double>::infinity();
  for (const Point2 & point : second) {
    const double projection = point.x * unit_x + point.y * unit_y;
    second_min = std::min(second_min, projection);
    second_max = std::max(second_max, projection);
  }
  return first_max < second_min - 1.0e-9 ||
         second_max < first_min - 1.0e-9;
}

// True when the convex OBB XY projection intersects one cell rectangle.
bool HullIntersectsCell(
  const std::vector<Point2> & hull,
  const FineCellIndex & cell, double origin_x,
  double origin_y, double resolution) noexcept
{
  const Point3 center =
    FineCellCenter(cell, origin_x, origin_y, resolution);
  const double half = 0.5 * resolution;
  const std::vector<Point2> rectangle{
    Point2{center.x - half, center.y - half},
    Point2{center.x + half, center.y - half},
    Point2{center.x + half, center.y + half},
    Point2{center.x - half, center.y + half},
  };
  if (SeparatedOnAxis(hull, rectangle, 1.0, 0.0) ||
    SeparatedOnAxis(hull, rectangle, 0.0, 1.0))
  {
    return false;
  }
  for (std::size_t index = 0; index < hull.size(); ++index) {
    const Point2 & first = hull[index];
    const Point2 & second = hull[(index + 1U) % hull.size()];
    if (SeparatedOnAxis(
        hull, rectangle, second.y - first.y,
        first.x - second.x))
    {
      return false;
    }
  }
  return true;
}

// Builds a conservative cell-to-vertical-interval envelope for the full
// roll/pitch/yaw body OBB. A 3D grid includes top, bottom and side samples; the
// projected OBB hull is additionally rasterized so a cell touched only by an
// overhang is not missed. Cells reached only through rasterization use the full
// OBB vertical range, keeping side and top bounds conservative.
bool BuildBodyEnvelopeCells(
  const TerrainSnapshot & terrain,
  const VehicleCapabilityConfig & vehicle,
  const WheelSupportOptions & options,
  const VehiclePose & pose,
  std::map<FineCellIndex, BodyCellEnvelope> * cells,
  std::string * detail)
{
  if (!vehicle.body_min.IsFinite() || !vehicle.body_max.IsFinite() ||
    vehicle.body_min.x >= vehicle.body_max.x ||
    vehicle.body_min.y >= vehicle.body_max.y ||
    vehicle.body_min.z >= vehicle.body_max.z)
  {
    *detail = "body envelope is not configured as a finite box";
    return false;
  }
  const double spacing = std::max(0.01, options.body_sample_spacing_m);
  const int nx = std::max(
    1, static_cast<int>(std::ceil(
      (vehicle.body_max.x - vehicle.body_min.x) /
      spacing)));
  const int ny = std::max(
    1, static_cast<int>(std::ceil(
      (vehicle.body_max.y - vehicle.body_min.y) /
      spacing)));
  const int nz = std::max(
    1, static_cast<int>(std::ceil(
      (vehicle.body_max.z - vehicle.body_min.z) /
      spacing)));
  double global_min_z = std::numeric_limits<double>::infinity();
  double global_max_z = -std::numeric_limits<double>::infinity();
  for (int ix = 0; ix <= nx; ++ix) {
    const double x =
      vehicle.body_min.x +
      (vehicle.body_max.x - vehicle.body_min.x) *
      static_cast<double>(ix) / static_cast<double>(nx);
    for (int iy = 0; iy <= ny; ++iy) {
      const double y =
        vehicle.body_min.y +
        (vehicle.body_max.y - vehicle.body_min.y) *
        static_cast<double>(iy) / static_cast<double>(ny);
      for (int iz = 0; iz <= nz; ++iz) {
        const double z =
          vehicle.body_min.z +
          (vehicle.body_max.z - vehicle.body_min.z) *
          static_cast<double>(iz) / static_cast<double>(nz);
        const Point3 world = BodyToWorld(pose, Point3{x, y, z});
        if (!world.IsFinite()) {
          *detail = "body envelope sample is non-finite";
          return false;
        }
        const auto cell =
          FineCellOf(
          world, terrain.origin_x(), terrain.origin_y(),
          terrain.resolution());
        if (!cell.has_value()) {
          *detail = "body envelope sample is outside the safe cell bound";
          return false;
        }
        BodyCellEnvelope & envelope = (*cells)[*cell];
        envelope.min_z = std::min(envelope.min_z, world.z);
        envelope.max_z = std::max(envelope.max_z, world.z);
        global_min_z = std::min(global_min_z, world.z);
        global_max_z = std::max(global_max_z, world.z);
      }
    }
  }
  std::array<Point3, 8> corners{};
  std::size_t corner_index = 0;
  for (int ix = 0; ix <= 1; ++ix) {
    for (int iy = 0; iy <= 1; ++iy) {
      for (int iz = 0; iz <= 1; ++iz) {
        corners[corner_index++] = BodyToWorld(
          pose, Point3{ix == 0 ? vehicle.body_min.x : vehicle.body_max.x,
            iy == 0 ? vehicle.body_min.y : vehicle.body_max.y,
            iz == 0 ? vehicle.body_min.z : vehicle.body_max.z});
      }
    }
  }
  std::vector<Point2> projected;
  projected.reserve(corners.size());
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Point3 & corner : corners) {
    if (!corner.IsFinite()) {
      *detail = "body envelope corner is non-finite";
      return false;
    }
    projected.push_back(Point2{corner.x, corner.y});
    min_x = std::min(min_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_x = std::max(max_x, corner.x);
    max_y = std::max(max_y, corner.y);
  }
  std::sort(
    projected.begin(), projected.end(),
    [](const Point2 & lhs, const Point2 & rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      return lhs.y < rhs.y;
    });
  std::vector<Point2> hull;
  for (const Point2 & point : projected) {
    while (hull.size() >= 2U &&
      Cross2(hull[hull.size() - 2U], hull.back(), point) <= 1.0e-15)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (std::size_t index = projected.size() - 1U; index > 0U; --index) {
    const Point2 & point = projected[index - 1U];
    while (hull.size() > lower_size &&
      Cross2(hull[hull.size() - 2U], hull.back(), point) <= 1.0e-15)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  const auto min_cell = FineCellOf(
    min_x, min_y, terrain.origin_x(),
    terrain.origin_y(), terrain.resolution());
  const auto max_cell = FineCellOf(
    max_x, max_y, terrain.origin_x(),
    terrain.origin_y(), terrain.resolution());
  if (!min_cell.has_value() || !max_cell.has_value()) {
    *detail = "projected body envelope is outside the safe cell bound";
    return false;
  }
  for (std::int64_t x = min_cell->x; x <= max_cell->x; ++x) {
    for (std::int64_t y = min_cell->y; y <= max_cell->y; ++y) {
      const FineCellIndex cell{x, y};
      if (!HullIntersectsCell(
          hull, cell, terrain.origin_x(),
          terrain.origin_y(), terrain.resolution()))
      {
        continue;
      }
      if (cells->find(cell) == cells->end()) {
        (*cells)[cell] = BodyCellEnvelope{global_min_z, global_max_z};
      }
    }
  }
  return true;
}

// Returns the conservative low/high plane height of one terrain node over a
// fine cell. Sampling the cell center and corners prevents a sloping surface
// from being represented only by its center height.
std::pair<double, double> NodeHeightRangeInCell(
  const TerrainNode & node, const FineCellIndex & cell,
  const TerrainSnapshot & terrain)
{
  const Point3 center = FineCellCenter(
    cell, terrain.origin_x(),
    terrain.origin_y(),
    terrain.resolution());
  const double half = 0.5 * terrain.resolution();
  const std::array<Point3, 5> samples{{
    center,
    Point3{center.x - half, center.y - half, 0.0},
    Point3{center.x + half, center.y - half, 0.0},
    Point3{center.x + half, center.y + half, 0.0},
    Point3{center.x - half, center.y + half, 0.0},
  }};
  double low = std::numeric_limits<double>::infinity();
  double high = -std::numeric_limits<double>::infinity();
  for (const Point3 & sample : samples) {
    const double z = PlaneHeight(node, sample.x, sample.y);
    if (!std::isfinite(z)) {
      continue;
    }
    low = std::min(low, z);
    high = std::max(high, z);
  }
  if (!std::isfinite(low) || !std::isfinite(high)) {
    return {node.representative.z, node.representative.z};
  }
  return {low, high};
}

// Vertical relation of one terrain node range to the pitched/rolled body OBB.
// Classification happens before any terrain status or obstacle read so an
// unrelated upper deck cannot poison the lower road.
enum class BodyTerrainRelation : std::uint8_t
{
  kBelowBody = 0,
  kIntersectsBody,
  kAboveBody,
};

// Classifies a terrain surface vertical range against the body OBB range. A
// range strictly below the body bottom is support ground; a range strictly
// above the body top is overhead; anything overlapping the body is an
// intersection candidate.
BodyTerrainRelation ClassifyBodyTerrain(
  double surface_low,
  double surface_high,
  double body_min_z, double body_max_z,
  double tolerance) noexcept
{
  if (surface_high < body_min_z - tolerance) {
    return BodyTerrainRelation::kBelowBody;
  }
  if (surface_low > body_max_z + tolerance) {
    return BodyTerrainRelation::kAboveBody;
  }
  return BodyTerrainRelation::kIntersectsBody;
}

// Vertical relation of an explicit obstacle slab to the body OBB. A slab whose
// bottom is above the body top is overhead/headroom evidence, not lower-road
// occupancy. An unset or non-finite interval stays unknown and must never be
// treated as clear.
enum class ObstacleBodyRelation : std::uint8_t
{
  kUnknownExtent = 0,
  kBelowBody,
  kIntersectsBody,
  kAboveBody,
};

// Classifies one explicit occupied slab against the body OBB vertical range.
// Only a validated finite interval is classified; otherwise the caller must
// fall back to the conservative unknown/occupied policy.
ObstacleBodyRelation ClassifyObstacleBody(
  const ObstacleAttributes & attributes,
  double body_min_z, double body_max_z,
  double tolerance) noexcept
{
  if (!attributes.occupied_vertical_extent_known ||
    !std::isfinite(attributes.occupied_min_z) ||
    !std::isfinite(attributes.occupied_max_z) ||
    attributes.occupied_min_z > attributes.occupied_max_z)
  {
    return ObstacleBodyRelation::kUnknownExtent;
  }
  if (attributes.occupied_max_z < body_min_z - tolerance) {
    return ObstacleBodyRelation::kBelowBody;
  }
  if (attributes.occupied_min_z > body_max_z + tolerance) {
    return ObstacleBodyRelation::kAboveBody;
  }
  return ObstacleBodyRelation::kIntersectsBody;
}

// Checks the full pitched/rolled body OBB against terrain, obstacle occupancy
// and observed headroom. Each terrain node is first classified by its vertical
// range against the body OBB. Only lower-road/contact-volume nodes are then
// queried for obstacle state; an upper-deck obstacle/unknown/prohibited node
// above the body contributes only overhead geometry, never a lower-road
// obstacle. Tire ground contact is exempt only inside the actual tire footprint
// at the contact-plane band and only for terrain connected to the selected
// wheel branch. Unknown headroom is rejected by default.
SupportRejection CheckBodyEnvelope(
  const MapSnapshot & map, const ObstacleDistanceField & distance_field,
  const VehicleCapabilityConfig & vehicle, const WheelSupportOptions & options,
  const VehiclePose & pose,
  const std::array<double, kWheelCount> & wheel_steer,
  const std::array<TerrainNodeId, kWheelCount> & branch_nodes,
  const std::array<double, kWheelCount> & wheel_contact_z,
  std::string * detail, double * chassis_clearance,
  double * overhead_clearance)
{
  const TerrainSnapshot & terrain = *map.fine_terrain();
  std::map<FineCellIndex, BodyCellEnvelope> body_cells;
  std::string envelope_detail;
  if (!BuildBodyEnvelopeCells(
      terrain, vehicle, options, pose, &body_cells,
      &envelope_detail))
  {
    *detail = envelope_detail;
    return SupportRejection::kInvalidQuery;
  }
  if (body_cells.empty()) {
    *detail = "body envelope does not intersect the terrain snapshot";
    return SupportRejection::kNoTerrain;
  }
  const double patch_length = ConfiguredContactPatchLength(vehicle);
  const double patch_width = ConfiguredContactPatchWidth(vehicle);
  const double contact_band = TireContactBandM(options);
  std::array<WheelContactFrame, kWheelCount> tire_frames{};
  std::array<bool, kWheelCount> tire_frame_valid{};
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    tire_frame_valid[wheel] = BuildWheelContactFrame(
      vehicle, pose, wheel, wheel_steer[wheel], patch_length, patch_width,
      &tire_frames[wheel]);
    if (tire_frame_valid[wheel] && std::isfinite(wheel_contact_z[wheel])) {
      tire_frames[wheel].contact_z = wheel_contact_z[wheel];
    }
  }
  std::unordered_map<TerrainNodeId, bool> wheel_connectivity;
  const auto connected_to_wheels = [&](TerrainNodeId id) {
      const auto iterator = wheel_connectivity.find(id);
      if (iterator != wheel_connectivity.end()) {
        return iterator->second;
      }
      bool connected = false;
      for (const TerrainNodeId branch : branch_nodes) {
        if (branch == kInvalidTerrainNodeId) {
          continue;
        }
        if (id == branch ||
          terrain.AreConnected(id, branch, options.max_deck_search_nodes))
        {
          connected = true;
          break;
        }
      }
      wheel_connectivity.emplace(id, connected);
      return connected;
    };

  // Relevant lower-road/contact volume obstacle gate. It intentionally does not
  // require an exact sub-truncation obstacle distance or a global clearance
  // guarantee: a clear, observed, locally-known node is usable, while occupied
  // state and bounded local unknown influence still fail.
  const auto contact_obstacle_rejection = [&](TerrainNodeId id,
      const ObstacleAttributes &
      attributes,
      bool slab_above_body) {
      // A missing distance-field record is absent observation evidence, not clear
      // ground. This also keeps the local unknown check meaningful.
      if (distance_field.FindRecord(id) == nullptr) {
        *detail = "no local obstacle record in the lower-road/contact volume";
        return SupportRejection::kUnknownObstacle;
      }
      const ObstacleState state = attributes.CombinedState();
      if (state == ObstacleState::kOccupied) {
        // An explicit slab entirely above the body is overhead evidence. It is
        // still tested by the headroom branch below; it must not be converted into
        // unconditionally occupied lower-road ground.
        if (!slab_above_body) {
          *detail = "obstacle occupies the lower-road/contact volume";
          return SupportRejection::kObstacleCollision;
        }
      } else if (state == ObstacleState::kUnknown &&
        distance_field.options().reject_unknown)
      {
        *detail = "unknown obstacle state in the lower-road/contact volume";
        return SupportRejection::kUnknownObstacle;
      }
      if (distance_field.options().reject_unknown &&
        distance_field.HasLocalUnknownInfluence(id))
      {
        *detail =
          "unobserved obstacle influence within the local truncation radius";
        return SupportRejection::kUnknownObstacle;
      }
      return SupportRejection::kNone;
    };

  struct GroundSurface
  {
    const TerrainNode * node{nullptr};
    double height{0.0};
    bool tire_contact{false};
  };

  const double tolerance = std::max(options.branch_height_tolerance_m, 1.0e-6);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  double minimum_overhead = std::numeric_limits<double>::infinity();
  // Defer missing-headroom rejection until every body cell has been classified.
  // A known overhead collision anywhere is more specific than absent evidence,
  // so it must win over the safe-default unknown rejection.
  bool missing_headroom = false;
  std::string missing_headroom_detail;
  for (const auto & cell_entry : body_cells) {
    const FineCellIndex & cell = cell_entry.first;
    const BodyCellEnvelope & envelope = cell_entry.second;
    const std::vector<TerrainNodeId> * nodes = terrain.FindNodesInCell(cell);
    if (nodes == nullptr) {
      *detail = "chassis overhangs an unobserved/hole cell";
      return SupportRejection::kHole;
    }
    const Point3 cell_center = FineCellCenter(
      cell, terrain.origin_x(), terrain.origin_y(), terrain.resolution());
    std::vector<GroundSurface> grounds;
    bool has_overhead = false;
    for (const TerrainNodeId id : *nodes) {
      const TerrainNode * node = terrain.FindNode(id);
      if (node == nullptr) {
        continue;
      }
      const std::pair<double, double> surface_range =
        NodeHeightRangeInCell(*node, cell, terrain);
      const BodyTerrainRelation relation = ClassifyBodyTerrain(
        surface_range.first, surface_range.second, envelope.min_z,
        envelope.max_z, tolerance);

      // Above-body terrain is overhead geometry only. Its terrain status,
      // obstacle occupancy and unknown attributes do not describe the lower
      // road and must not turn into a lower-road rejection.
      if (relation == BodyTerrainRelation::kAboveBody) {
        has_overhead = true;
        continue;
      }

      // Classify an explicit obstacle slab before the occupied/unknown tests.
      // A slab entirely above the body is headroom evidence for this lower node
      // and must not be read as occupied road; a slab intersecting the body is a
      // collision regardless of which layer supplied it.
      const ObstacleAttributes node_attributes = map.MergedObstacle(id);
      const ObstacleBodyRelation slab_relation = ClassifyObstacleBody(
        node_attributes, envelope.min_z, envelope.max_z, tolerance);
      if (slab_relation == ObstacleBodyRelation::kIntersectsBody) {
        *detail = "obstacle slab intersects the body envelope";
        return SupportRejection::kObstacleCollision;
      }
      const bool slab_above_body =
        slab_relation == ObstacleBodyRelation::kAboveBody;
      if (slab_above_body) {
        has_overhead = true;
      }

      // Terrain status is inspected only for nodes that are below or intersect
      // the body envelope.
      if (node->status == TerrainStatus::kProhibited) {
        *detail = "prohibited terrain under the body";
        return SupportRejection::kProhibitedTerrain;
      }
      if (!node->IsSupported()) {
        *detail = "unknown terrain under the body";
        return SupportRejection::kUnknownTerrain;
      }

      // Tire contact needs the exact contact band, the actual tire footprint
      // and a branch connected to one of the selected wheels.
      bool tire_contact = false;
      double tire_contact_height = std::numeric_limits<double>::quiet_NaN();
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        if (!tire_frame_valid[wheel] ||
          !std::isfinite(wheel_contact_z[wheel]))
        {
          continue;
        }
        const double contact_z = wheel_contact_z[wheel];
        if (surface_range.second < contact_z - contact_band ||
          surface_range.first > contact_z + contact_band)
        {
          continue;
        }
        const Point3 contact_point{cell_center.x, cell_center.y, contact_z};
        if (!InsideTireContactVolume(
            tire_frames[wheel], contact_point,
            contact_band))
        {
          continue;
        }
        if (!connected_to_wheels(id)) {
          continue;
        }
        tire_contact = true;
        tire_contact_height = contact_z;
        break;
      }

      if (relation == BodyTerrainRelation::kBelowBody) {
        // Supported terrain below the body must be on the selected wheel
        // branch; a disconnected lower deck is a different surface, not ground
        // under this vehicle.
        if (!connected_to_wheels(id)) {
          *detail = "supported terrain under the body is on another branch";
          return SupportRejection::kInconsistentDecks;
        }
        const SupportRejection obstacle_rejection =
          contact_obstacle_rejection(id, node_attributes, slab_above_body);
        if (obstacle_rejection != SupportRejection::kNone) {
          return obstacle_rejection;
        }
        grounds.push_back(
          GroundSurface{node, surface_range.second, false});
        continue;
      }

      // The node intersects the body envelope. A legal tire contact is the only
      // exempt intersection; every other supported surface collides.
      if (tire_contact) {
        const SupportRejection obstacle_rejection =
          contact_obstacle_rejection(id, node_attributes, slab_above_body);
        if (obstacle_rejection != SupportRejection::kNone) {
          return obstacle_rejection;
        }
        grounds.push_back(GroundSurface{node, tire_contact_height, true});
        continue;
      }
      const bool unconnected = !connected_to_wheels(id);
      if (unconnected && surface_range.first > envelope.min_z + tolerance) {
        *detail = "overhead/deck intersects the body envelope";
        return SupportRejection::kOverheadCollision;
      }
      *detail = "terrain intersects the body envelope";
      return SupportRejection::kChassisCollision;
    }
    if (grounds.empty()) {
      if (has_overhead) {
        *detail = "only overhead terrain exists under the body; headroom is "
          "unknown";
        return SupportRejection::kUnknownObstacle;
      }
      *detail = "no ground surface exists under the body envelope";
      return SupportRejection::kUnknownTerrain;
    }
    bool known_headroom = false;
    for (const GroundSurface & ground : grounds) {
      const double clearance = envelope.min_z - ground.height;
      minimum_clearance = std::min(minimum_clearance, clearance);
      if (!ground.tire_contact &&
        clearance + 1.0e-6 < vehicle.body_clearance_m)
      {
        *detail = "chassis clearance " + std::to_string(clearance) +
          " m is below " + std::to_string(vehicle.body_clearance_m) +
          " m";
        return SupportRejection::kChassisCollision;
      }
      const ObstacleAttributes attributes =
        map.MergedObstacle(ground.node->id);
      const bool headroom_known =
        attributes.clearance_known &&
        std::isfinite(attributes.clearance_above_m);
      if (!headroom_known) {
        // Absence of overhead points is not proof of free vertical space.
        // The safe default requires an explicit observed vertical extent.
        if (options.require_observed_headroom && !missing_headroom) {
          missing_headroom = true;
          missing_headroom_detail =
            "observed headroom is missing above the lower road; unknown "
            "vertical extent is rejected by the safe-default policy";
        }
        continue;
      }
      const double available = ground.height + attributes.clearance_above_m;
      const double margin = available - envelope.max_z;
      known_headroom = true;
      minimum_overhead = std::min(minimum_overhead, margin);
      if (margin < -1.0e-6) {
        *detail = "known overhead clearance is below the body top";
        return SupportRejection::kOverheadCollision;
      }
    }
    if (has_overhead && !known_headroom && !missing_headroom) {
      missing_headroom = true;
      missing_headroom_detail =
        "overhead terrain has unknown headroom above the body";
    }
  }
  if (missing_headroom) {
    *detail = missing_headroom_detail;
    return SupportRejection::kUnknownObstacle;
  }

  *chassis_clearance = minimum_clearance;
  *overhead_clearance =
    std::isfinite(minimum_overhead) ?
    minimum_overhead :
    std::numeric_limits<double>::quiet_NaN();
  return SupportRejection::kNone;
}

// Offline kinematic/static-map evaluator. This is a separate, explicitly named
// policy: it checks the configured Ackermann contact geometry, measured map
// connectivity, slope/step consistency and static 2.5-D obstacles. It does not
// model tire contact patches, suspension travel, tire compliance, body_max or
// observed headroom, and its result must never be read as real-vehicle
// feasibility.
WheelSupportEvaluation EvaluateOfflineKinematicStaticMap(
  const MapSnapshot & map, const VehicleCapabilityConfig & vehicle,
  const WheelSupportOptions & options, const WheelSupportQuery & query)
{
  WheelSupportEvaluation result;
  result.model = TraversabilityModel::kOfflineKinematicStaticMap;
  const TerrainSnapshot & terrain = *map.fine_terrain();

  // Selects the supported node whose plane is closest to the expected ground
  // height at one wheel. A cell with no node is a hole; a prohibited-only cell
  // is prohibited; an unsupported-only cell is unknown terrain.
  const auto select_node =
    [&terrain](const Point3 & wheel, double expected_z,
      std::string * detail) -> const TerrainNode * {
      const auto cell = FineCellOf(
        wheel, terrain.origin_x(),
        terrain.origin_y(), terrain.resolution());
      if (!cell.has_value()) {
        *detail = "wheel query is outside the safe cell bound";
        return nullptr;
      }
      const std::vector<TerrainNodeId> * nodes =
        terrain.FindNodesInCell(*cell);
      if (nodes == nullptr) {
        *detail = "wheel cell is unobserved";
        return nullptr;
      }
      const TerrainNode * best = nullptr;
      double best_delta = std::numeric_limits<double>::infinity();
      bool saw_prohibited = false;
      for (const TerrainNodeId id : *nodes) {
        const TerrainNode * node = terrain.FindNode(id);
        if (node == nullptr) {
          continue;
        }
        if (node->status == TerrainStatus::kProhibited) {
          saw_prohibited = true;
          continue;
        }
        if (!node->IsSupported() || !node->plane.valid) {
          continue;
        }
        const double height = PlaneHeight(*node, wheel.x, wheel.y);
        if (!std::isfinite(height)) {
          continue;
        }
        const double delta = std::abs(height - expected_z);
        if (delta < best_delta) {
          best_delta = delta;
          best = node;
        }
      }
      if (best == nullptr) {
        *detail = saw_prohibited ? "wheel cell is prohibited" :
          "wheel has no supported terrain plane";
      }
      return best;
    };

  const double cos_yaw = std::cos(query.heading);
  const double sin_yaw = std::sin(query.heading);
  std::array<Point3, kWheelCount> wheel_points{};
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    const Point3 & offset = vehicle.wheel_contact_offsets[wheel];
    wheel_points[wheel] = Point3{
      query.center.x + cos_yaw * offset.x - sin_yaw * offset.y,
      query.center.y + sin_yaw * offset.x + cos_yaw * offset.y,
      query.center.z};
  }

  const bool check_ground_support = options.offline_check_ground_support;
  std::array<const TerrainNode *, kWheelCount> nodes{};
  std::array<double, kWheelCount> contact_z{};
  if (check_ground_support) {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const double reference_z =
        wheel == 0U ? query.center.z :
        contact_z[wheel - 1U];
      std::string detail;
      const TerrainNode * node =
        select_node(wheel_points[wheel], reference_z, &detail);
      if (node == nullptr) {
        result.rejection = detail == "wheel cell is prohibited" ?
          SupportRejection::kProhibitedTerrain :
          SupportRejection::kUnsupportedWheel;
        result.detail = std::string("offline model: wheel ") +
          kWheelNames[wheel] + " " + detail;
        return result;
      }
      nodes[wheel] = node;
      contact_z[wheel] =
        PlaneHeight(*node, wheel_points[wheel].x, wheel_points[wheel].y);
      if (!std::isfinite(contact_z[wheel])) {
        result.rejection = SupportRejection::kUnsupportedWheel;
        result.detail = std::string("offline model: wheel ") +
          kWheelNames[wheel] + " has a non-finite plane height";
        return result;
      }
    }
  } else {
    // Ground-support opt-out: keep the actual center terrain requirement, then
    // estimate four virtual contact heights from that one supported center
    // plane. No actual wheel cell, contact-patch evidence or pairwise deck
    // connection is claimed; obstacles, prohibited/raised terrain and body
    // collision are still enforced by the footprint scan below.
    const auto center_cell = FineCellOf(
      query.center, terrain.origin_x(), terrain.origin_y(),
      terrain.resolution());
    if (!center_cell.has_value()) {
      result.rejection = SupportRejection::kNoTerrain;
      result.detail = "offline model (support unchecked): query center is "
        "outside the safe cell bound";
      return result;
    }
    const std::vector<TerrainNodeId> * center_nodes =
      terrain.FindNodesInCell(*center_cell);
    const TerrainNode * center_node = nullptr;
    double center_best_delta = std::numeric_limits<double>::infinity();
    bool center_saw_prohibited = false;
    if (center_nodes != nullptr) {
      for (const TerrainNodeId id : *center_nodes) {
        const TerrainNode * node = terrain.FindNode(id);
        if (node == nullptr) {
          continue;
        }
        if (node->status == TerrainStatus::kProhibited) {
          center_saw_prohibited = true;
          continue;
        }
        if (!node->IsSupported() || !node->plane.valid) {
          continue;
        }
        const double height =
          PlaneHeight(*node, query.center.x, query.center.y);
        if (!std::isfinite(height)) {
          continue;
        }
        const double delta = std::abs(height - query.center.z);
        if (delta < center_best_delta) {
          center_best_delta = delta;
          center_node = node;
        }
      }
    }
    if (center_node == nullptr) {
      result.rejection = center_saw_prohibited ?
        SupportRejection::kProhibitedTerrain :
        SupportRejection::kNoTerrain;
      result.detail = center_saw_prohibited ?
        "offline model (support unchecked): query center is prohibited" :
        "offline model (support unchecked): query center has no supported "
        "terrain plane";
      return result;
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      nodes[wheel] = center_node;
      // Support is unchecked: estimate a rigid pose from the centre tangent
      // plane. Extending one cell's two triangles out to distant virtual wheel
      // positions would manufacture contact warp from a local mesh crease.
      const auto & plane = center_node->plane;
      contact_z[wheel] = plane.centroid.z -
        (plane.normal.x * (wheel_points[wheel].x - plane.centroid.x) +
        plane.normal.y * (wheel_points[wheel].y - plane.centroid.y)) / plane.normal.z;
      if (!std::isfinite(contact_z[wheel])) {
        result.rejection = SupportRejection::kNoTerrain;
        result.detail = "offline model (support unchecked): center plane has "
          "a non-finite virtual contact height";
        return result;
      }
    }
  }

  const PoseFit fit =
    FitVehiclePose(
    vehicle.wheel_contact_offsets, contact_z,
    options.max_pose_iterations, 1.0e-6);
  if (!fit.ok) {
    result.rejection = SupportRejection::kPoseSolveBudget;
    result.detail = "offline model: " + fit.detail;
    return result;
  }
  if (fit.residual_rms > vehicle.max_pose_residual_m) {
    result.rejection = SupportRejection::kResidualExceeded;
    result.detail = "offline model: Ackermann contact residual exceeds the "
      "configured limit";
    return result;
  }
  if (fit.pitch > vehicle.max_pitch_rad ||
    fit.pitch < -vehicle.max_pitch_rad)
  {
    result.rejection = SupportRejection::kPitchExceeded;
    result.detail = "offline model: map pitch exceeds the configured limit";
    return result;
  }
  if (fit.roll > vehicle.max_roll_rad ||
    fit.roll < -vehicle.max_roll_rad)
  {
    result.rejection = SupportRejection::kRollExceeded;
    result.detail = "offline model: map roll exceeds the configured limit";
    return result;
  }

  // All four contact nodes must belong to one connected measured deck. This is
  // the same snapshot connectivity contract used by the full-wheel model. With
  // ground support explicitly unchecked, the four contacts are virtual heights
  // on the center plane, so there is no wheel-deck connection evidence to
  // require.
  if (check_ground_support) {
    for (std::size_t first = 0; first < kWheelCount; ++first) {
      for (std::size_t second = first + 1U; second < kWheelCount; ++second) {
        if (!terrain.AreConnected(
            nodes[first]->id, nodes[second]->id,
            options.max_deck_search_nodes))
        {
          result.rejection = SupportRejection::kInconsistentDecks;
          result.detail =
            "offline model: wheel surfaces are on different measured decks";
          return result;
        }
      }
    }
  }

  VehiclePose pose;
  pose.base_origin = Point3{query.center.x, query.center.y, fit.z0};
  pose.yaw = query.heading;
  pose.roll = fit.roll;
  pose.pitch = fit.pitch;
  // Uses the same rigid-body Z model as FitVehiclePose:
  //   z = z0 + body_z*cos(roll)*cos(pitch) - x*sin(pitch)
  //       + y*sin(roll)*cos(pitch)
  // A plane rising toward +x yields a negative pitch. Evaluating both the
  // contact plane (body_z = -base_contact_height) and the lower body envelope
  // (body_z = body_min.z) through this one model keeps the chassis check
  // consistent on slopes instead of mixing sin/tan signs.
  const double cos_roll = std::cos(fit.roll);
  const double cos_pitch = std::cos(fit.pitch);
  const double sin_roll = std::sin(fit.roll);
  const double sin_pitch = std::sin(fit.pitch);
  const auto plane_z = [&](double x, double y, double body_z) {
      const double dx = x - query.center.x;
      const double dy = y - query.center.y;
      return pose.base_origin.z + body_z * cos_roll * cos_pitch -
             dx * sin_pitch + dy * sin_roll * cos_pitch;
    };
  const auto expected_ground_z = [&](double x, double y) {
      return plane_z(x, y, -vehicle.base_contact_height_m);
    };
  const auto body_bottom_z = [&](double x, double y) {
      return plane_z(x, y, vehicle.body_min.z);
    };
  // body_min includes the tires and is measured on the ground datum, so its
  // lower Z equals the tire contact plane. That is not an independent chassis
  // clearance: comparing it with noisy measured terrain would reject a valid
  // road over numerical micrometres. Only enforce the terrain-vs-body check
  // when the configured lower envelope actually sits above the contact plane;
  // otherwise the offline model reports chassis clearance as not validated
  // rather than inventing a raised body_min.
  const bool has_measured_chassis_clearance =
    vehicle.body_min.z > -vehicle.base_contact_height_m + 1.0e-6;

  const auto occupied_at = [&map](TerrainNodeId id) {
      return map.MergedObstacle(id).IsOccupied();
    };
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (occupied_at(nodes[wheel]->id)) {
      result.rejection = SupportRejection::kObstacleCollision;
      result.detail = std::string("offline model: wheel ") +
        kWheelNames[wheel] +
        " contacts an occupied static-map obstacle";
      return result;
    }
  }

  // Planar collision outline. The measured body XY rectangle is rotated by the
  // query heading; otherwise a circle derived from the measured body corners is
  // the conservative fallback. Candidate cells cover the closed world AABB plus
  // a one-cell halo, then an exact shape test discards cells that do not touch
  // the closed footprint. The halo is not a sampling gap: it only ensures that
  // a cell touching an exact MIN gridline (which floor() would otherwise omit)
  // is enumerated, and the exact test keeps every hard gate and the risk max on
  // the same cells.
  const bool use_measured_rectangle =
    vehicle.body_min_configured && vehicle.body_max_configured &&
    vehicle.body_min.IsFinite() && vehicle.body_max.IsFinite() &&
    vehicle.body_max.x >= vehicle.body_min.x &&
    vehicle.body_max.y >= vehicle.body_min.y;
  double min_dx = 0.0;
  double max_dx = 0.0;
  double min_dy = 0.0;
  double max_dy = 0.0;
  double outline_radius = 0.0;
  const double cos_heading = std::cos(query.heading);
  const double sin_heading = std::sin(query.heading);
  const auto rotate_local = [&](double x, double y) {
      return Point3{
      query.center.x + cos_heading * x - sin_heading * y,
      query.center.y + sin_heading * x + cos_heading * y, 0.0};
    };
  double world_min_x = 0.0;
  double world_max_x = 0.0;
  double world_min_y = 0.0;
  double world_max_y = 0.0;
  if (use_measured_rectangle) {
    min_dx = std::min(vehicle.body_min.x, vehicle.body_max.x);
    max_dx = std::max(vehicle.body_min.x, vehicle.body_max.x);
    min_dy = std::min(vehicle.body_min.y, vehicle.body_max.y);
    max_dy = std::max(vehicle.body_min.y, vehicle.body_max.y);
    const std::array<Point3, 4> corners{
      rotate_local(min_dx, min_dy), rotate_local(min_dx, max_dy),
      rotate_local(max_dx, min_dy), rotate_local(max_dx, max_dy)};
    world_min_x = corners[0].x;
    world_max_x = corners[0].x;
    world_min_y = corners[0].y;
    world_max_y = corners[0].y;
    for (const Point3 & corner : corners) {
      world_min_x = std::min(world_min_x, corner.x);
      world_max_x = std::max(world_max_x, corner.x);
      world_min_y = std::min(world_min_y, corner.y);
      world_max_y = std::max(world_max_y, corner.y);
    }
  } else {
    outline_radius = options.offline_planar_collision_radius_m;
    min_dx = -outline_radius;
    max_dx = outline_radius;
    min_dy = -outline_radius;
    max_dy = outline_radius;
    world_min_x = query.center.x - outline_radius;
    world_max_x = query.center.x + outline_radius;
    world_min_y = query.center.y - outline_radius;
    world_max_y = query.center.y + outline_radius;
  }
  const double outline_resolution = terrain.resolution();
  const auto first_cell = FineCellOf(
    world_min_x - outline_resolution, world_min_y - outline_resolution,
    terrain.origin_x(), terrain.origin_y(), outline_resolution);
  const auto last_cell = FineCellOf(
    world_max_x + outline_resolution, world_max_y + outline_resolution,
    terrain.origin_x(), terrain.origin_y(), outline_resolution);
  if (!first_cell.has_value() || !last_cell.has_value()) {
    result.rejection = SupportRejection::kInvalidQuery;
    result.detail = "offline model: footprint is outside the safe cell bound";
    return result;
  }
  // Exact closed-shape intersection for one axis-aligned cell. Rectangle cells
  // use SAT on the four separating axes (world X/Y and body local X/Y); the
  // square's projection on each axis is halfwidth * res * (|cos| + |sin|).
  // Circle fallback is heading-independent and uses the nearest cell point.
  const double local_center_x = 0.5 * (min_dx + max_dx);
  const double local_center_y = 0.5 * (min_dy + max_dy);
  const double half_length = 0.5 * (max_dx - min_dx);
  const double half_width = 0.5 * (max_dy - min_dy);
  const double body_center_world_x =
    cos_heading * local_center_x - sin_heading * local_center_y;
  const double body_center_world_y =
    sin_heading * local_center_x + cos_heading * local_center_y;
  const double cell_half = 0.5 * outline_resolution;
  const double cell_projection_on_body_axis =
    cell_half * (std::abs(cos_heading) + std::abs(sin_heading));
  const auto intersects_outline =
    [&](double x0, double x1, double y0, double y1) {
      if (!use_measured_rectangle) {
        const double nearest_x = x0 > 0.0 ? x0 : (x1 < 0.0 ? x1 : 0.0);
        const double nearest_y = y0 > 0.0 ? y0 : (y1 < 0.0 ? y1 : 0.0);
        return std::hypot(nearest_x, nearest_y) <=
               outline_radius + kEpsilon;
      }
      const double dx = 0.5 * (x0 + x1);
      const double dy = 0.5 * (y0 + y1);
      // World X/Y axes.
      const double body_extent_x =
        half_length * std::abs(cos_heading) +
        half_width * std::abs(sin_heading);
      const double body_extent_y =
        half_length * std::abs(sin_heading) +
        half_width * std::abs(cos_heading);
      if (std::abs(dx - body_center_world_x) >
        cell_half + body_extent_x + kEpsilon)
      {
        return false;
      }
      if (std::abs(dy - body_center_world_y) >
        cell_half + body_extent_y + kEpsilon)
      {
        return false;
      }
      // Body local X/Y axes.
      const double local_cell_x =
        dx * cos_heading + dy * sin_heading;
      const double local_cell_y =
        -dx * sin_heading + dy * cos_heading;
      if (std::abs(local_cell_x - local_center_x) >
        half_length + cell_projection_on_body_axis + kEpsilon)
      {
        return false;
      }
      if (std::abs(local_cell_y - local_center_y) >
        half_width + cell_projection_on_body_axis + kEpsilon)
      {
        return false;
      }
      return true;
    };
  double minimum_ground = std::numeric_limits<double>::infinity();
  double boundary_risk = 0.0;
  std::size_t cell_queries = 0;
  for (std::int64_t cell_x = first_cell->x; cell_x <= last_cell->x; ++cell_x) {
    for (std::int64_t cell_y = first_cell->y; cell_y <= last_cell->y;
      ++cell_y)
    {
      const FineCellIndex cell{cell_x, cell_y};
      const Point3 cell_center = FineCellCenter(
        cell, terrain.origin_x(), terrain.origin_y(), outline_resolution);
      const double x0 =
        cell_center.x - 0.5 * outline_resolution - query.center.x;
      const double x1 =
        cell_center.x + 0.5 * outline_resolution - query.center.x;
      const double y0 =
        cell_center.y - 0.5 * outline_resolution - query.center.y;
      const double y1 =
        cell_center.y + 0.5 * outline_resolution - query.center.y;
      if (!intersects_outline(x0, x1, y0, y1)) {
        continue;
      }
      ++cell_queries;
      const std::vector<TerrainNodeId> * cell_nodes =
        terrain.FindNodesInCell(cell);
      if (cell_nodes == nullptr) {
        // With ground support unchecked, a missing footprint cell is a support
        // evidence gap, not a collision. Occupied obstacles, prohibited or
        // raised actual terrain in observed cells are still rejected below.
        if (!check_ground_support) {
          continue;
        }
        result.rejection = SupportRejection::kHole;
        result.detail = "offline model: footprint overhangs an unobserved "
          "cell";
        return result;
      }
      const double x = cell_center.x;
      const double y = cell_center.y;
      const double expected = expected_ground_z(x, y);
      const double band =
        std::max(
        options.offline_contact_height_tolerance_m,
        vehicle.max_step_m);
      const TerrainNode * selected = nullptr;
      double selected_delta = std::numeric_limits<double>::infinity();
      bool saw_prohibited = false;
      bool saw_raised = false;
      bool saw_occupied = false;
      for (const TerrainNodeId id : *cell_nodes) {
        const TerrainNode * node = terrain.FindNode(id);
        if (node == nullptr) {
          continue;
        }
        if (!check_ground_support) {
          const double height = node->plane.valid ?
            PlaneHeight(*node, x, y) : node->representative.z;
          const double body_top =
            vehicle.body_max_configured && vehicle.body_max.IsFinite() ?
            plane_z(x, y, vehicle.body_max.z) : expected + band;
          // Ground/obstacle attributes belong to a particular floor. A lower
          // deck or a distant overhead deck cannot block this footprint just
          // because it shares XY. Keep nearby raised terrain within the body
          // envelope, including an actual step above the contact band.
          if (!std::isfinite(height) || height < expected - band - 1.0e-6 ||
            height > std::max(expected + band, body_top) + 1.0e-6)
          {
            continue;
          }
          saw_occupied = saw_occupied || occupied_at(id);
          saw_raised = saw_raised || height > expected + band + 1.0e-6;
        }
        if (node->status == TerrainStatus::kProhibited) {
          saw_prohibited = true;
          continue;
        }
        if (!node->IsSupported() || !node->plane.valid) {
          continue;
        }
        const double height = PlaneHeight(*node, x, y);
        if (!std::isfinite(height)) {
          continue;
        }
        const double delta = std::abs(height - expected);
        if (delta <= band + 1.0e-6 && delta < selected_delta) {
          selected = node;
          selected_delta = delta;
        }
      }
      if (!check_ground_support && saw_occupied) {
        result.rejection = SupportRejection::kObstacleCollision;
        result.detail =
          "offline model (support unchecked): static-map obstacle "
          "intersects the planar footprint";
        return result;
      }
      if (selected == nullptr) {
        if (saw_prohibited) {
          result.rejection = SupportRejection::kProhibitedTerrain;
          result.detail =
            "offline model: footprint crosses prohibited terrain";
          return result;
        }
        if (!check_ground_support) {
          if (saw_raised) {
            result.rejection = SupportRejection::kHeightStep;
            result.detail =
              "offline model (support unchecked): footprint has actual "
              "raised terrain above the accepted step band";
            return result;
          }
          // Missing/lower support under this footprint cell is exactly the
          // ground-presence evidence this opt-out suppresses.
          continue;
        }
        result.rejection = SupportRejection::kHole;
        result.detail = "offline model: footprint has no measured "
          "ground on the selected deck";
        return result;
      }
      if (occupied_at(selected->id)) {
        result.rejection = SupportRejection::kObstacleCollision;
        result.detail =
          "offline model: static-map obstacle intersects the planar "
          "footprint";
        return result;
      }
      const double ground = PlaneHeight(*selected, x, y);
      minimum_ground = std::min(minimum_ground, ground);
      // The soft boundary term is the worst accepted footprint cell, using the
      // same immutable measured-edge and insufficient-connection cache and the
      // same guarded conservative penalty as visualization. An infinite
      // uncertainty distance with no seed contributes zero instead of becoming
      // full risk. It is added only after every hard hole/prohibited/obstacle/
      // chassis gate above passed, so zero weight disables only this soft term.
      boundary_risk = std::max(
        boundary_risk,
        TerrainBoundaryEvidencePenalty(
          terrain.MeasuredSupportBoundaryDistanceM(selected->id),
          terrain.InsufficientConnectionDistanceM(selected->id),
          options.boundary_penalty_distance_m,
          options.boundary_penalty_weight));
      if (has_measured_chassis_clearance &&
        ground > body_bottom_z(x, y) + 1.0e-6)
      {
        result.rejection = SupportRejection::kChassisCollision;
        result.detail =
          "offline model: terrain intersects the planar body lower bound";
        return result;
      }
    }
  }

  result.model = TraversabilityModel::kOfflineKinematicStaticMap;
  result.physical_coverage_known = false;
  result.feasible = true;
  result.rejection = SupportRejection::kNone;
  result.detail =
    has_measured_chassis_clearance ?
    "offline kinematic/static-map model accepted" :
    "offline kinematic/static-map model accepted (chassis clearance "
    "not validated: body_min includes tires at the contact plane)";
  if (!check_ground_support) {
    result.detail +=
      " (support unchecked: four wheel/footprint ground contacts are "
      "virtual estimates from the actual center supported plane; no measured "
      "wheel contact or physical tire coverage is claimed)";
  }
  result.pose = pose;
  result.roll_rad = fit.roll;
  result.pitch_rad = fit.pitch;
  result.residual_rms_m = fit.residual_rms;
  result.twist_m = fit.twist;
  result.max_travel_m = fit.max_abs_residual;
  result.chassis_min_clearance_m =
    has_measured_chassis_clearance && std::isfinite(minimum_ground) ?
    body_bottom_z(query.center.x, query.center.y) - minimum_ground :
    std::numeric_limits<double>::quiet_NaN();
  // Headroom is intentionally not claimed by the offline model.
  result.overhead_min_clearance_m =
    std::numeric_limits<double>::quiet_NaN();
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    result.branch_nodes[wheel] = nodes[wheel]->id;
    WheelSupport support;
    support.valid = true;
    support.node_id = nodes[wheel]->id;
    support.cell = nodes[wheel]->cell;
    support.local_surface_index = nodes[wheel]->local_surface_index;
    support.contact = Point3{wheel_points[wheel].x, wheel_points[wheel].y,
      contact_z[wheel]};
    // Physical tire-patch coverage is not measured by this model: keep the
    // per-wheel evidence explicitly unknown rather than claiming 1.0.
    support.coverage_known = false;
    support.coverage = std::numeric_limits<double>::quiet_NaN();
    support.contact_residual_m = 0.0;
    support.sample_min_z = contact_z[wheel];
    support.sample_max_z = contact_z[wheel];
    result.wheels[wheel] = support;
  }
  result.cost =
    options.slope_penalty_weight *
    (std::abs(fit.pitch) + std::abs(fit.roll)) +
    options.residual_penalty_weight * fit.residual_rms +
    boundary_risk;
  result.timing.sample_count = cell_queries;
  result.timing.cell_queries = cell_queries;
  return result;
}

}  // namespace

const char * WheelIndexToString(WheelIndex wheel) noexcept
{
  const auto index = static_cast<std::size_t>(wheel);
  return index < kWheelCount ? kWheelNames[index] : "unknown";
}

const char * VehicleConfigurationStatusToString(
  VehicleConfigurationStatus status) noexcept
{
  switch (status) {
    case VehicleConfigurationStatus::kConfigured:
      return "configured";
    case VehicleConfigurationStatus::kVehicleCapabilityUnconfigured:
      return "vehicle-capability-unconfigured";
    case VehicleConfigurationStatus::kInvalidGeometry:
      return "invalid-geometry";
    case VehicleConfigurationStatus::kInconsistentSteeringGeometry:
      return "inconsistent-steering-geometry";
    case VehicleConfigurationStatus::kInvalidThresholds:
      return "invalid-thresholds";
  }
  return "unknown";
}

AckermannSteeringAngles SolveAckermannSteering(
  double central_rad, const VehicleCapabilityConfig & vehicle) noexcept
{
  AckermannSteeringAngles angles;
  if (!std::isfinite(central_rad)) {
    angles.detail = "central steering angle is non-finite";
    return angles;
  }
  if (std::abs(central_rad) > vehicle.max_steer_central_rad + kEpsilon) {
    angles.detail = "central steering angle exceeds the configured limit";
    return angles;
  }
  if (std::abs(central_rad) < kEpsilon) {
    angles.valid = true;
    return angles;
  }
  const double tan_central = std::tan(central_rad);
  const double wheelbase = EffectiveWheelbaseM(vehicle);
  const double front_track = EffectiveFrontTrackWidthM(vehicle);
  if (!std::isfinite(wheelbase) || wheelbase <= 0.0 ||
    !std::isfinite(front_track) || front_track <= 0.0)
  {
    angles.detail = "Ackermann geometry is not configured";
    return angles;
  }
  const double turn_radius = wheelbase / tan_central;
  const double inner_radius =
    std::abs(turn_radius) - 0.5 * front_track;
  const double outer_radius =
    std::abs(turn_radius) + 0.5 * front_track;
  if (inner_radius <= kEpsilon || outer_radius <= kEpsilon) {
    angles.detail = "Ackermann turn radius intersects the vehicle axle";
    return angles;
  }
  const double inner = std::atan2(wheelbase, inner_radius);
  const double outer = std::atan2(wheelbase, outer_radius);
  if (inner > vehicle.max_steer_inner_rad + 0.05) {
    angles.detail = "computed inner steering angle exceeds the configured "
      "inner limit";
    return angles;
  }
  if (central_rad > 0.0) {
    angles.front_left_rad = inner;
    angles.front_right_rad = outer;
  } else {
    angles.front_left_rad = -outer;
    angles.front_right_rad = -inner;
  }
  angles.inner_rad = inner;
  angles.outer_rad = outer;
  angles.valid = true;
  return angles;
}

double VehicleCapabilityConfig::FrontAxleMidpointX() const noexcept
{
  return 0.5 * (wheel_contact_offsets[0].x + wheel_contact_offsets[1].x);
}

double VehicleCapabilityConfig::RearAxleMidpointX() const noexcept
{
  return 0.5 * (wheel_contact_offsets[2].x + wheel_contact_offsets[3].x);
}

double VehicleCapabilityConfig::MeasuredWheelbaseM() const noexcept
{
  return FrontAxleMidpointX() - RearAxleMidpointX();
}

double VehicleCapabilityConfig::MeasuredFrontTrackWidthM() const noexcept
{
  return wheel_contact_offsets[0].y - wheel_contact_offsets[1].y;
}

double VehicleCapabilityConfig::MeasuredRearTrackWidthM() const noexcept
{
  return wheel_contact_offsets[2].y - wheel_contact_offsets[3].y;
}

std::array<Point3, kWheelCount>
VehicleCapabilityConfig::ComposeWheelContactOffsets() const noexcept
{
  std::array<Point3, kWheelCount> offsets{};
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
    !std::isfinite(front_track_width_m) || front_track_width_m <= 0.0 ||
    !std::isfinite(rear_track_width_m) || rear_track_width_m <= 0.0 ||
    !std::isfinite(base_contact_height_m) || base_contact_height_m <= 0.0)
  {
    return offsets;
  }
  const double half_wheelbase = 0.5 * wheelbase_m;
  const double half_front_track = 0.5 * front_track_width_m;
  const double half_rear_track = 0.5 * rear_track_width_m;
  const double z = -base_contact_height_m;
  offsets[0] = Point3{half_wheelbase, half_front_track, z};
  offsets[1] = Point3{half_wheelbase, -half_front_track, z};
  offsets[2] = Point3{-half_wheelbase, half_rear_track, z};
  offsets[3] = Point3{-half_wheelbase, -half_rear_track, z};
  return offsets;
}

bool VehicleCapabilityConfig::DeriveWheelContactOffsets(std::string * detail)
{
  const std::array<Point3, kWheelCount> offsets =
    ComposeWheelContactOffsets();
  if (!offsets[0].IsFinite() || offsets[0].x <= 0.0 ||
    offsets[0].z >= 0.0)
  {
    if (detail != nullptr) {
      *detail = "wheelbase/tracks/base_contact_height are not valid for "
        "composing wheel contacts";
    }
    return false;
  }
  wheel_contact_offsets = offsets;
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

VehicleConfigurationStatus
VehicleCapabilityConfig::ValidateGeometryAndThresholds(
  std::string * detail) const
{
  const auto reject = [detail](VehicleConfigurationStatus status,
      const std::string & message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return status;
    };
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
    !std::isfinite(track_width_m) || track_width_m <= 0.0 ||
    !std::isfinite(front_track_width_m) || front_track_width_m <= 0.0 ||
    !std::isfinite(rear_track_width_m) || rear_track_width_m <= 0.0 ||
    !std::isfinite(base_contact_height_m) ||
    base_contact_height_m <= 0.0)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "wheelbase/front/rear track/base_contact_height must be "
      "finite and positive");
  }
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (!wheel_contact_offsets[wheel].IsFinite()) {
      return reject(
        VehicleConfigurationStatus::kInvalidGeometry,
        "wheel contact offset is non-finite");
    }
  }
  if (wheel_contact_offsets[0].x <= 0.0 || wheel_contact_offsets[0].y <= 0.0 ||
    wheel_contact_offsets[1].x <= 0.0 ||
    wheel_contact_offsets[1].y >= 0.0 ||
    wheel_contact_offsets[2].x >= 0.0 ||
    wheel_contact_offsets[2].y <= 0.0 ||
    wheel_contact_offsets[3].x >= 0.0 ||
    wheel_contact_offsets[3].y >= 0.0)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "wheel contact offsets must be FL,FR,RL,RR around base_link");
  }
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (std::abs(wheel_contact_offsets[wheel].z + base_contact_height_m) >
      0.05)
    {
      return reject(
        VehicleConfigurationStatus::kInvalidGeometry,
        "wheel contact Z does not match base_contact_height");
    }
  }
  if (std::abs(wheel_contact_offsets[0].x - wheel_contact_offsets[1].x) >
    0.05 ||
    std::abs(wheel_contact_offsets[2].x - wheel_contact_offsets[3].x) >
    0.05)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "front/rear contact pairs are not on common axle lines");
  }
  const double measured_wheelbase = MeasuredWheelbaseM();
  const double measured_front_track = MeasuredFrontTrackWidthM();
  const double measured_rear_track = MeasuredRearTrackWidthM();
  if (!std::isfinite(measured_wheelbase) || measured_wheelbase <= 0.0 ||
    !std::isfinite(measured_front_track) || measured_front_track <= 0.0 ||
    !std::isfinite(measured_rear_track) || measured_rear_track <= 0.0 ||
    std::abs(measured_wheelbase - wheelbase_m) > 0.05 ||
    std::abs(measured_front_track - front_track_width_m) > 0.05 ||
    std::abs(measured_rear_track - rear_track_width_m) > 0.05)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "wheelbase/front/rear track do not match the wheel contact "
      "offsets");
  }
  // The legacy aggregate is meaningful only when the two axles share a track.
  // Independent front/rear tracks intentionally make it a compatibility field.
  if (std::abs(front_track_width_m - rear_track_width_m) <= 0.05 &&
    std::abs(front_track_width_m - track_width_m) > 0.05)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "track_width_m does not match the common front/rear track");
  }
  if (!std::isfinite(max_steer_central_rad) ||
    max_steer_central_rad <= 0.0 ||
    max_steer_central_rad >= kPi * 0.5 ||
    !std::isfinite(max_steer_inner_rad) || max_steer_inner_rad <= 0.0)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidGeometry,
      "steering limits must be finite and positive");
  }
  const AckermannSteeringAngles max_angles =
    SolveAckermannSteering(max_steer_central_rad, *this);
  if (!max_angles.valid) {
    return reject(
      VehicleConfigurationStatus::kInconsistentSteeringGeometry,
      max_angles.detail);
  }
  if (!std::isfinite(min_contact_coverage) ||
    min_contact_coverage <= 0.0 || min_contact_coverage > 1.0 ||
    !std::isfinite(max_pitch_rad) || max_pitch_rad <= 0.0 ||
    max_pitch_rad >= kPi * 0.5 || !std::isfinite(max_roll_rad) ||
    max_roll_rad <= 0.0 || max_roll_rad >= kPi * 0.5 ||
    !std::isfinite(max_step_m) || max_step_m <= 0.0 ||
    !std::isfinite(max_warp_m) || max_warp_m < 0.0 ||
    !std::isfinite(max_pose_residual_m) || max_pose_residual_m <= 0.0 ||
    !std::isfinite(body_clearance_m) || body_clearance_m < 0.0 ||
    !std::isfinite(max_branch_jump_m) || max_branch_jump_m <= 0.0)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidThresholds,
      "acceptance thresholds are outside their valid ranges");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return VehicleConfigurationStatus::kConfigured;
}

VehicleConfigurationStatus VehicleCapabilityConfig::Validate(
  std::string * detail) const
{
  const auto reject = [detail](VehicleConfigurationStatus status,
      const std::string & message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return status;
    };
  const VehicleConfigurationStatus geometry =
    ValidateGeometryAndThresholds(detail);
  if (geometry != VehicleConfigurationStatus::kConfigured) {
    return geometry;
  }
  if (body_min_configured != true || !body_min.IsFinite() ||
    !body_max_configured || !body_max.IsFinite() ||
    body_min.x >= body_max.x || body_min.y >= body_max.y ||
    body_min.z >= body_max.z)
  {
    if (detail != nullptr) {
      *detail =
        "vehicle-capability-unconfigured: full body envelope (body_max) is "
        "not measured";
    }
    return VehicleConfigurationStatus::kVehicleCapabilityUnconfigured;
  }
  const bool patch_length_configured =
    std::isfinite(contact_patch_length_m) && contact_patch_length_m > 0.0;
  if ((!std::isfinite(wheel_radius_m) || wheel_radius_m <= 0.0) &&
    !patch_length_configured)
  {
    if (detail != nullptr) {
      *detail =
        "vehicle-capability-unconfigured: wheel radius is not measured and "
        "no explicit contact patch length is configured";
    }
    return VehicleConfigurationStatus::kVehicleCapabilityUnconfigured;
  }
  if (!std::isfinite(tire_width_m) || tire_width_m <= 0.0 ||
    !std::isfinite(suspension_travel_m) || suspension_travel_m < 0.0 ||
    !std::isfinite(tire_compliance_m) || tire_compliance_m < 0.0)
  {
    if (detail != nullptr) {
      *detail =
        "vehicle-capability-unconfigured: tire width, suspension travel, or "
        "tire compliance is not measured";
    }
    return VehicleConfigurationStatus::kVehicleCapabilityUnconfigured;
  }
  if (std::isfinite(contact_patch_length_m) &&
    contact_patch_length_m <= 0.0)
  {
    return reject(
      VehicleConfigurationStatus::kInvalidThresholds,
      "contact_patch_length_m must be positive when configured");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return VehicleConfigurationStatus::kConfigured;
}

bool WheelSupportEvaluation::HasFourWheelSupport() const noexcept
{
  if (!feasible || model != TraversabilityModel::kFullWheel ||
    !physical_coverage_known)
  {
    return false;
  }
  for (const WheelSupport & wheel : wheels) {
    if (!wheel.valid || wheel.node_id == kInvalidTerrainNodeId ||
      !wheel.coverage_known || !std::isfinite(wheel.coverage))
    {
      return false;
    }
  }
  return true;
}

bool WheelSupportEvaluation::HasOfflineKinematicSupport() const noexcept
{
  if (!feasible || model != TraversabilityModel::kOfflineKinematicStaticMap) {
    return false;
  }
  for (const WheelSupport & wheel : wheels) {
    if (!wheel.valid || wheel.node_id == kInvalidTerrainNodeId) {
      return false;
    }
  }
  return true;
}

const char * TraversabilityModelToString(
  TraversabilityModel model) noexcept
{
  switch (model) {
    case TraversabilityModel::kFullWheel:
      return "full_wheel";
    case TraversabilityModel::kOfflineKinematicStaticMap:
      return "offline_kinematic_static_map";
  }
  return "unknown";
}

const char * SupportRejectionToString(SupportRejection rejection) noexcept
{
  switch (rejection) {
    case SupportRejection::kNone:
      return "none";
    case SupportRejection::kVehicleCapabilityUnconfigured:
      return "vehicle_capability_unconfigured";
    case SupportRejection::kInvalidQuery:
      return "invalid_query";
    case SupportRejection::kSnapshotMismatch:
      return "snapshot_mismatch";
    case SupportRejection::kNoTerrain:
      return "no_terrain";
    case SupportRejection::kInsufficientFootprintCoverage:
      return "insufficient_footprint_coverage";
    case SupportRejection::kUnknownTerrain:
      return "unknown_terrain";
    case SupportRejection::kProhibitedTerrain:
      return "prohibited_terrain";
    case SupportRejection::kHole:
      return "hole";
    case SupportRejection::kHeightStep:
      return "height_step";
    case SupportRejection::kTwistExceeded:
      return "twist_exceeded";
    case SupportRejection::kResidualExceeded:
      return "residual_exceeded";
    case SupportRejection::kTravelExceeded:
      return "travel_exceeded";
    case SupportRejection::kPitchExceeded:
      return "pitch_exceeded";
    case SupportRejection::kRollExceeded:
      return "roll_exceeded";
    case SupportRejection::kInconsistentDecks:
      return "inconsistent_decks";
    case SupportRejection::kBranchJump:
      return "branch_jump";
    case SupportRejection::kChassisCollision:
      return "chassis_collision";
    case SupportRejection::kObstacleCollision:
      return "obstacle_collision";
    case SupportRejection::kOverheadCollision:
      return "overhead_collision";
    case SupportRejection::kUnknownObstacle:
      return "unknown_obstacle";
    case SupportRejection::kPoseSolveBudget:
      return "pose_solve_budget";
    case SupportRejection::kUnsupportedWheel:
      return "unsupported_wheel";
  }
  return "unknown";
}

WheelSupportEvaluator::WheelSupportEvaluator(
  VehicleCapabilityConfig vehicle,
  WheelSupportOptions options)
: vehicle_(std::move(vehicle)), options_(std::move(options))
{
  if (options_.model == TraversabilityModel::kOfflineKinematicStaticMap) {
    // The offline model consumes only wheelbase/tracks/contact offsets,
    // steering limits and acceptance thresholds. Validate exactly those
    // inputs; never fill missing measured tire/body values with invented
    // probe geometry. The full model below keeps the complete fail-closed
    // measured-capability validation.
    configuration_status_ =
      vehicle_.ValidateGeometryAndThresholds(&configuration_detail_);
    if (configuration_status_ != VehicleConfigurationStatus::kConfigured) {
      configuration_detail_ =
        "offline kinematic/static-map geometry is invalid: " +
        configuration_detail_;
      return;
    }
    configuration_detail_ =
      "offline-kinematic-static-map model configured from wheelbase/track/"
      "steering limits; supplied vehicle measurements are preserved; "
      "physical tire-patch coverage, suspension response and observed "
      "headroom are not validated by this model";
  } else {
    configuration_status_ = vehicle_.Validate(&configuration_detail_);
  }
  if (configuration_status_ != VehicleConfigurationStatus::kConfigured) {
    return;
  }
  if (options_.model == TraversabilityModel::kOfflineKinematicStaticMap &&
    (!std::isfinite(options_.offline_planar_collision_radius_m) ||
    options_.offline_planar_collision_radius_m <= 0.0 ||
    !std::isfinite(options_.offline_body_sample_spacing_m) ||
    options_.offline_body_sample_spacing_m <= 0.0 ||
    !std::isfinite(options_.offline_contact_height_tolerance_m) ||
    options_.offline_contact_height_tolerance_m <= 0.0))
  {
    configuration_status_ = VehicleConfigurationStatus::kInvalidThresholds;
    configuration_detail_ =
      "offline kinematic/static-map thresholds are invalid";
    return;
  }
  if (options_.footprint_samples_along < 2 ||
    options_.footprint_samples_across < 2 ||
    options_.max_branch_combinations == 0 ||
    options_.max_pose_iterations == 0 ||
    options_.max_deck_search_nodes == 0 ||
    !std::isfinite(options_.branch_height_tolerance_m) ||
    options_.branch_height_tolerance_m <= 0.0 ||
    !std::isfinite(options_.min_center_support) ||
    options_.min_center_support < 0.0 ||
    options_.min_center_support > 1.0 ||
    !std::isfinite(options_.max_contact_height_spread_m) ||
    options_.max_contact_height_spread_m <= 0.0 ||
    !std::isfinite(options_.body_sample_spacing_m) ||
    options_.body_sample_spacing_m <= 0.0 ||
    !std::isfinite(options_.obstacle_penalty_distance_m) ||
    options_.obstacle_penalty_distance_m < 0.0 ||
    !std::isfinite(options_.obstacle_penalty_weight) ||
    options_.obstacle_penalty_weight < 0.0 ||
    !std::isfinite(options_.slope_penalty_weight) ||
    options_.slope_penalty_weight < 0.0 ||
    !std::isfinite(options_.residual_penalty_weight) ||
    options_.residual_penalty_weight < 0.0 ||
    !std::isfinite(options_.clearance_penalty_weight) ||
    options_.clearance_penalty_weight < 0.0 ||
    !std::isfinite(options_.boundary_penalty_distance_m) ||
    options_.boundary_penalty_distance_m < 0.0 ||
    !std::isfinite(options_.boundary_penalty_weight) ||
    options_.boundary_penalty_weight < 0.0)
  {
    configuration_status_ = VehicleConfigurationStatus::kInvalidThresholds;
    configuration_detail_ = "wheel support sampling/penalty budgets are invalid";
  }
}

WheelSupportEvaluation WheelSupportEvaluator::Evaluate(
  const MapSnapshot & map, const ObstacleDistanceField & distance_field,
  const WheelSupportQuery & query) const
{
  WheelSupportEvaluation result;
  result.model = options_.model;
  const auto evaluation_start = std::chrono::steady_clock::now();
  double footprint_ms = 0.0;
  double pose_solve_ms = 0.0;
  double body_check_ms = 0.0;
  std::size_t sample_count = 0;
  std::size_t branch_combinations = 0;
  std::size_t cell_queries = 0;
  std::size_t surface_queries = 0;
  std::size_t neighbor_queries = 0;
  std::size_t obstacle_queries = 0;
  std::size_t distance_queries = 0;
  std::chrono::steady_clock::time_point branch_start{};
  bool branch_started = false;
  // Every path returns through this helper so even an early rejection carries
  // real elapsed time and the counts accumulated before the rejection.
  auto finish = [&](WheelSupportEvaluation evaluation) {
      const auto branch_end = std::chrono::steady_clock::now();
      evaluation.timing.total_ms =
        DurationToMs(branch_end - evaluation_start);
      evaluation.timing.footprint_ms = footprint_ms;
      evaluation.timing.pose_solve_ms = pose_solve_ms;
      evaluation.timing.body_check_ms = body_check_ms;
      evaluation.timing.branch_solve_ms =
        branch_started ?
        std::max(
        0.0, DurationToMs(branch_end - branch_start) -
        pose_solve_ms - body_check_ms) :
        0.0;
      evaluation.timing.sample_count = sample_count;
      evaluation.timing.branch_combinations = branch_combinations;
      evaluation.timing.cell_queries = cell_queries;
      evaluation.timing.surface_queries = surface_queries;
      evaluation.timing.neighbor_queries = neighbor_queries;
      evaluation.timing.obstacle_queries = obstacle_queries;
      evaluation.timing.distance_queries = distance_queries;
      return evaluation;
    };
  if (configuration_status_ != VehicleConfigurationStatus::kConfigured) {
    result.rejection = SupportRejection::kVehicleCapabilityUnconfigured;
    result.detail = configuration_detail_;
    return finish(std::move(result));
  }
  if (!map.IsReady()) {
    result.rejection = SupportRejection::kNoTerrain;
    result.detail = "map snapshot is not ready";
    return finish(std::move(result));
  }
  if (!distance_field.ready() || distance_field.terrain().get() !=
    map.fine_terrain().get())
  {
    result.rejection = SupportRejection::kSnapshotMismatch;
    result.detail = "distance field does not belong to the supplied snapshot";
    return finish(std::move(result));
  }
  if (!query.center.IsFinite() || !std::isfinite(query.heading) ||
    !std::isfinite(query.steering_central_rad))
  {
    result.rejection = SupportRejection::kInvalidQuery;
    result.detail = "query pose/steering is non-finite";
    return finish(std::move(result));
  }
  const AckermannSteeringAngles steering =
    SolveAckermannSteering(query.steering_central_rad, vehicle_);
  if (!steering.valid) {
    result.rejection = SupportRejection::kInvalidQuery;
    result.detail = steering.detail;
    return finish(std::move(result));
  }

  // Explicit model switch: the production full-wheel path below is unchanged.
  // The offline path is a separate policy over the same immutable snapshot and
  // the same Ackermann query interface.
  if (options_.model == TraversabilityModel::kOfflineKinematicStaticMap) {
    return finish(
      EvaluateOfflineKinematicStaticMap(
        map, vehicle_, options_, query));
  }

  const TerrainSnapshot & terrain = *map.fine_terrain();
  const std::array<double, kWheelCount> wheel_steer{
    steering.front_left_rad, steering.front_right_rad, 0.0, 0.0};
  std::array<double, kWheelCount> contact_z{};
  std::array<std::vector<FootprintSample>, kWheelCount> footprints;
  std::array<std::vector<WheelCandidate>, kWheelCount> candidates;

  VehiclePose seed_pose;
  seed_pose.base_origin = query.center;
  seed_pose.yaw = query.heading;

  // Initial wheel footprints use a level body. The final selected branch is
  // rechecked with the solved roll/pitch below.
  const auto footprint_start = std::chrono::steady_clock::now();
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    Point3 wheel_center{};
    std::string detail;
    footprints[wheel] = BuildFootprint(
      vehicle_, options_, seed_pose, wheel,
      wheel_steer[wheel], terrain.resolution(),
      &wheel_center, &detail);
    sample_count += footprints[wheel].size();
    if (footprints[wheel].empty()) {
      result.rejection = SupportRejection::kInvalidQuery;
      result.detail = detail;
      return finish(std::move(result));
    }
    if (!AssignFootprintCells(terrain, &footprints[wheel], &detail)) {
      result.rejection = SupportRejection::kInvalidQuery;
      result.detail = detail;
      return finish(std::move(result));
    }
    const auto center_cell = FineCellOf(
      wheel_center, terrain.origin_x(),
      terrain.origin_y(),
      terrain.resolution());
    if (!center_cell.has_value()) {
      result.rejection = SupportRejection::kNoTerrain;
      result.detail = "wheel center is outside the safe cell bound";
      return finish(std::move(result));
    }
    ++cell_queries;
    const std::vector<TerrainNodeId> * center_nodes =
      terrain.FindNodesInCell(*center_cell);
    if (center_nodes == nullptr) {
      result.rejection = SupportRejection::kHole;
      result.detail = std::string("wheel ") + kWheelNames[wheel] +
        " center cell is unobserved";
      return finish(std::move(result));
    }
    bool saw_supported = false;
    for (const TerrainNodeId id : *center_nodes) {
      ++surface_queries;
      const TerrainNode * node = terrain.FindNode(id);
      if (node == nullptr) {
        continue;
      }
      if (node->status == TerrainStatus::kProhibited) {
        result.rejection = SupportRejection::kProhibitedTerrain;
        result.detail = std::string("wheel ") + kWheelNames[wheel] +
          " center is prohibited";
        return finish(std::move(result));
      }
      if (!node->IsSupported() || !node->plane.valid) {
        continue;
      }
      saw_supported = true;
      candidates[wheel].push_back(
        EvaluateWheelCandidate(
          terrain, *node, wheel_center, footprints[wheel], options_));
    }
    if (!saw_supported || candidates[wheel].empty()) {
      result.rejection = SupportRejection::kUnsupportedWheel;
      result.detail = std::string("wheel ") + kWheelNames[wheel] +
        " has no supported terrain plane";
      return finish(std::move(result));
    }
  }

  footprint_ms += DurationToMs(
    std::chrono::steady_clock::now() - footprint_start);
  std::array<std::size_t, kWheelCount> choice{};
  std::size_t combination_count = 0;
  bool found = false;
  SupportRejection first_rejection = SupportRejection::kNone;
  std::string first_detail;
  WheelSupportEvaluation best;

  // Bounded joint branch enumeration. Every combination is a real four-wheel
  // branch set; a per-wheel nearest choice is never accepted in isolation.
  branch_start = std::chrono::steady_clock::now();
  branch_started = true;
  std::function<void(std::size_t)> visit = [&](std::size_t wheel) {
      if (!found && combination_count >= options_.max_branch_combinations) {
        if (first_rejection == SupportRejection::kNone) {
          first_rejection = SupportRejection::kPoseSolveBudget;
          first_detail = "branch combination budget exhausted";
        }
        return;
      }
      if (wheel == kWheelCount) {
        ++combination_count;
        ++branch_combinations;
        std::array<WheelCandidate, kWheelCount> selected{};
        for (std::size_t index = 0; index < kWheelCount; ++index) {
          selected[index] = candidates[index][choice[index]];
          if (!selected[index].valid) {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kUnsupportedWheel;
              first_detail = "selected wheel branch is invalid";
            }
            return;
          }
          if (selected[index].coverage < vehicle_.min_contact_coverage ||
            (!selected[index].center_supported &&
            options_.min_center_support > 0.0))
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kInsufficientFootprintCoverage;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " footprint coverage is insufficient";
            }
            return;
          }
          if (selected[index].sample_max_z - selected[index].sample_min_z >
            options_.max_contact_height_spread_m)
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kHeightStep;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " footprint spans a height step";
            }
            return;
          }
          contact_z[index] = selected[index].contact_z;
        }
        const auto pose_start = std::chrono::steady_clock::now();
        const PoseFit fit =
          FitVehiclePose(
          vehicle_.wheel_contact_offsets, contact_z,
          options_.max_pose_iterations, 1.0e-6);
        pose_solve_ms += DurationToMs(
          std::chrono::steady_clock::now() - pose_start);
        if (!fit.ok) {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kPoseSolveBudget;
            first_detail = fit.detail;
          }
          return;
        }
        const double travel_limit = vehicle_.suspension_travel_m +
          vehicle_.tire_compliance_m +
          vehicle_.max_pose_residual_m;
        if (fit.residual_rms > vehicle_.max_pose_residual_m) {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kResidualExceeded;
            first_detail = "rigid four-wheel pose residual exceeds the limit";
          }
          return;
        }
        if (fit.max_abs_residual > travel_limit) {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kTravelExceeded;
            first_detail = "required suspension/tire travel exceeds the limit";
          }
          return;
        }
        if (fit.twist > vehicle_.max_warp_m) {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kTwistExceeded;
            first_detail = "four-wheel support twist exceeds the limit";
          }
          return;
        }
        if (fit.pitch > vehicle_.max_pitch_rad ||
          fit.pitch < -vehicle_.max_pitch_rad)
        {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kPitchExceeded;
            first_detail = "vehicle pitch exceeds the configured limit";
          }
          return;
        }
        if (fit.roll > vehicle_.max_roll_rad ||
          fit.roll < -vehicle_.max_roll_rad)
        {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kRollExceeded;
            first_detail = "vehicle roll exceeds the configured limit";
          }
          return;
        }

        VehiclePose pose;
        pose.base_origin = Point3{query.center.x, query.center.y, fit.z0};
        pose.yaw = query.heading;
        pose.roll = fit.roll;
        pose.pitch = fit.pitch;

        // Recompute the finite footprint coverage with the solved attitude. This
        // catches a branch that only looked continuous for a level body.
        std::array<double, kWheelCount> final_contact_z = contact_z;
        for (std::size_t index = 0; index < kWheelCount; ++index) {
          Point3 final_center{};
          std::string detail;
          std::vector<FootprintSample> final_samples =
            BuildFootprint(
            vehicle_, options_, pose, index,
            wheel_steer[index], terrain.resolution(),
            &final_center, &detail);
          sample_count += final_samples.size();
          if (final_samples.empty() ||
            !AssignFootprintCells(terrain, &final_samples, &detail))
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kInvalidQuery;
              first_detail = detail;
            }
            return;
          }
          ++surface_queries;
          const TerrainNode * selected_node =
            terrain.FindNode(selected[index].node_id);
          if (selected_node == nullptr) {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kUnsupportedWheel;
              first_detail = "selected wheel node disappeared";
            }
            return;
          }
          const WheelCandidate rechecked = EvaluateWheelCandidate(
            terrain, *selected_node, final_center, final_samples, options_);
          if (!rechecked.valid ||
            rechecked.coverage < vehicle_.min_contact_coverage)
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection =
                SupportRejection::kInsufficientFootprintCoverage;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " loses coverage under the solved attitude";
            }
            return;
          }
          final_contact_z[index] = rechecked.contact_z;
        }
        const auto final_pose_start = std::chrono::steady_clock::now();
        const PoseFit final_fit =
          FitVehiclePose(
          vehicle_.wheel_contact_offsets, final_contact_z,
          options_.max_pose_iterations, 1.0e-6);
        pose_solve_ms += DurationToMs(
          std::chrono::steady_clock::now() - final_pose_start);
        if (!final_fit.ok ||
          final_fit.residual_rms > vehicle_.max_pose_residual_m ||
          final_fit.max_abs_residual > travel_limit ||
          final_fit.twist > vehicle_.max_warp_m ||
          final_fit.pitch > vehicle_.max_pitch_rad ||
          final_fit.pitch < -vehicle_.max_pitch_rad ||
          final_fit.roll > vehicle_.max_roll_rad ||
          final_fit.roll < -vehicle_.max_roll_rad)
        {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = SupportRejection::kResidualExceeded;
            first_detail = "solved attitude fails the final pose/twist checks";
          }
          return;
        }
        pose.base_origin.z = final_fit.z0;
        pose.roll = final_fit.roll;
        pose.pitch = final_fit.pitch;

        std::array<TerrainNodeId, kWheelCount> branch_nodes{};
        double support_cost = 0.0;
        for (std::size_t index = 0; index < kWheelCount; ++index) {
          branch_nodes[index] = selected[index].node_id;
          support_cost +=
            (1.0 - selected[index].coverage) * 2.0 +
            std::abs(final_contact_z[index] - contact_z[index]);
          double distance = std::numeric_limits<double>::quiet_NaN();
          ++distance_queries;
          if (distance_field.HasGuaranteedClearance(branch_nodes[index])) {
            ++distance_queries;
            distance = distance_field.GuaranteedClearanceM(branch_nodes[index]);
          } else if (distance_field.IsDistanceKnown(branch_nodes[index])) {
            ++distance_queries;
            // A locally guaranteed clear node uses its bounded metric above. When
            // local unknown influence removes that guarantee, charge the
            // strongest soft penalty instead of treating it as free space.
            distance = 0.0;
          }
          if (std::isfinite(distance) &&
            options_.obstacle_penalty_distance_m > 0.0 &&
            distance < options_.obstacle_penalty_distance_m)
          {
            support_cost +=
              options_.obstacle_penalty_weight *
              (options_.obstacle_penalty_distance_m - distance) /
              options_.obstacle_penalty_distance_m;
          }
        }
        for (std::size_t first = 0; first < kWheelCount; ++first) {
          for (std::size_t second = first + 1; second < kWheelCount; ++second) {
            ++neighbor_queries;
            if (!terrain.AreConnected(
                branch_nodes[first],
                branch_nodes[second],
                options_.max_deck_search_nodes))
            {
              if (first_rejection == SupportRejection::kNone) {
                first_rejection = SupportRejection::kInconsistentDecks;
                first_detail = "selected wheel surfaces are on different decks";
              }
              return;
            }
          }
        }
        // A legal tire contact still cannot overlap an occupied or unobserved
        // obstacle record. Chassis samples inside the tire footprint are skipped
        // later, so this check is the tire-level obstacle gate.
        for (std::size_t index = 0; index < kWheelCount; ++index) {
          ++obstacle_queries;
          if (distance_field.FindRecord(branch_nodes[index]) == nullptr) {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kUnknownObstacle;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " has no local obstacle record";
            }
            return;
          }
          ++obstacle_queries;
          const ObstacleAttributes attributes =
            map.MergedObstacle(branch_nodes[index]);
          const ObstacleState state = attributes.CombinedState();
          if (state == ObstacleState::kOccupied) {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kObstacleCollision;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " contacts an occupied obstacle";
            }
            return;
          }
          if (state == ObstacleState::kUnknown &&
            distance_field.options().reject_unknown)
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kUnknownObstacle;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " has an unknown obstacle state";
            }
            return;
          }
          if (state == ObstacleState::kClear &&
            distance_field.options().reject_unknown &&
            distance_field.HasLocalUnknownInfluence(branch_nodes[index]))
          {
            if (first_rejection == SupportRejection::kNone) {
              first_rejection = SupportRejection::kUnknownObstacle;
              first_detail = std::string("wheel ") + kWheelNames[index] +
                " has local unknown obstacle influence";
            }
            return;
          }
        }

        if (query.has_previous && query.previous.HasFourWheelSupport()) {
          for (std::size_t index = 0; index < kWheelCount; ++index) {
            if (std::abs(
                selected[index].contact_z -
                query.previous.wheels[index].contact.z) >
              vehicle_.max_branch_jump_m)
            {
              if (first_rejection == SupportRejection::kNone) {
                first_rejection = SupportRejection::kBranchJump;
                first_detail = std::string("wheel ") + kWheelNames[index] +
                  " jumps support branch";
              }
              return;
            }
          }
        }

        std::string body_detail;
        double chassis_clearance = std::numeric_limits<double>::quiet_NaN();
        double overhead_clearance = std::numeric_limits<double>::quiet_NaN();
        const auto body_start = std::chrono::steady_clock::now();
        const SupportRejection body_rejection = CheckBodyEnvelope(
          map, distance_field, vehicle_, options_, pose, wheel_steer,
          branch_nodes, final_contact_z, &body_detail, &chassis_clearance,
          &overhead_clearance);
        body_check_ms += DurationToMs(
          std::chrono::steady_clock::now() - body_start);
        if (body_rejection != SupportRejection::kNone) {
          if (first_rejection == SupportRejection::kNone) {
            first_rejection = body_rejection;
            first_detail = body_detail;
          }
          return;
        }

        WheelSupportEvaluation evaluation;
        evaluation.model = TraversabilityModel::kFullWheel;
        evaluation.physical_coverage_known = true;
        evaluation.feasible = true;
        evaluation.rejection = SupportRejection::kNone;
        evaluation.pose = pose;
        evaluation.roll_rad = final_fit.roll;
        evaluation.pitch_rad = final_fit.pitch;
        evaluation.residual_rms_m = final_fit.residual_rms;
        evaluation.twist_m = final_fit.twist;
        evaluation.max_travel_m = final_fit.max_abs_residual;
        evaluation.chassis_min_clearance_m = chassis_clearance;
        evaluation.overhead_min_clearance_m = overhead_clearance;
        evaluation.branch_nodes = branch_nodes;
        for (std::size_t index = 0; index < kWheelCount; ++index) {
          WheelSupport support;
          support.valid = true;
          support.node_id = selected[index].node_id;
          support.cell = selected[index].cell;
          support.local_surface_index = selected[index].local_surface_index;
          support.contact = Point3{
            selected[index].wheel_center.x, selected[index].wheel_center.y,
            final_contact_z[index]};
          support.coverage_known = true;
          support.coverage = selected[index].coverage;
          const Point3 & offset = vehicle_.wheel_contact_offsets[index];
          const double predicted =
            pose.base_origin.z - offset.x * std::sin(pose.pitch) +
            (offset.y * std::sin(pose.roll) +
            offset.z * std::cos(pose.roll)) *
            std::cos(pose.pitch);
          support.contact_residual_m = final_contact_z[index] - predicted;
          support.sample_min_z = selected[index].sample_min_z;
          support.sample_max_z = selected[index].sample_max_z;
          evaluation.wheels[index] = support;
        }
        support_cost +=
          options_.slope_penalty_weight *
          (std::abs(final_fit.pitch) + std::abs(final_fit.roll)) +
          options_.residual_penalty_weight * final_fit.residual_rms;
        evaluation.cost = support_cost;
        if (!found || evaluation.cost < best.cost) {
          found = true;
          best = evaluation;
        }
        return;
      }
      for (std::size_t candidate_index = 0;
        candidate_index < candidates[wheel].size(); ++candidate_index)
      {
        choice[wheel] = candidate_index;
        visit(wheel + 1);
        if (combination_count >= options_.max_branch_combinations) {
          break;
        }
      }
    };
  visit(0);

  if (!found) {
    result.rejection =
      first_rejection == SupportRejection::kNone ?
      SupportRejection::kUnsupportedWheel :
      first_rejection;
    result.detail = first_detail;
    return finish(std::move(result));
  }
  return finish(std::move(best));
}  // NOLINT(readability/fn_size): dense support evaluation; split would duplicate state

}  // namespace dddmr_grid_global_planner
