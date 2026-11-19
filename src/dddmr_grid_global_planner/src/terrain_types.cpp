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

#include "grid_global_planner/terrain_types.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Returns the sector index for a direction, or -1 for a zero/non-finite vector.
int SectorOf(double dx, double dy) noexcept
{
  if (!std::isfinite(dx) || !std::isfinite(dy) ||
    (std::abs(dx) < 1.0e-12 && std::abs(dy) < 1.0e-12))
  {
    return -1;
  }
  const double angle = std::atan2(dy, dx);
  const double normalized = (angle + kPi) / kTwoPi;
  int sector =
    static_cast<int>(std::floor(normalized * static_cast<double>(kSupportSectorCount)));
  if (sector < 0) {
    sector = 0;
  }
  if (sector >= static_cast<int>(kSupportSectorCount)) {
    sector = static_cast<int>(kSupportSectorCount) - 1;
  }
  return sector;
}

}  // namespace

bool Point3::IsFinite() const noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool Vector3::IsFinite() const noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double Vector3::SquaredNorm() const noexcept
{
  return x * x + y * y + z * z;
}

double Vector3::Norm() const noexcept
{
  return std::sqrt(SquaredNorm());
}

Vector3 Vector3::Normalized() const noexcept
{
  const double norm = Norm();
  if (!std::isfinite(norm) || norm < 1.0e-12) {
    return Vector3{0.0, 0.0, 0.0};
  }
  return Vector3{x / norm, y / norm, z / norm};
}

double Dot(const Vector3 & lhs, const Vector3 & rhs) noexcept
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vector3 Cross(const Vector3 & lhs, const Vector3 & rhs) noexcept
{
  return Vector3{
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x};
}

Vector3 operator-(const Point3 & lhs, const Point3 & rhs) noexcept
{
  return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vector3 operator-(const Vector3 & lhs, const Vector3 & rhs) noexcept
{
  return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

std::size_t FineCellIndexHash::operator()(const FineCellIndex & cell) const noexcept
{
  const std::size_t hx = std::hash<std::int64_t>{}(cell.x);
  const std::size_t hy = std::hash<std::int64_t>{}(cell.y);
  return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
}

std::size_t CoarseCellIndexHash::operator()(const CoarseCellIndex & cell) const noexcept
{
  const std::size_t hx = std::hash<std::int64_t>{}(cell.x);
  const std::size_t hy = std::hash<std::int64_t>{}(cell.y);
  return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
}

bool IsSafeWorldCoordinate(double value) noexcept
{
  return std::isfinite(value) && std::abs(value) <= kMaxWorldCoordinate;
}

bool IsSafeCellIndex(std::int64_t value) noexcept
{
  return value >= -kMaxCellCoordinate && value <= kMaxCellCoordinate;
}

bool IsSafeFineCell(const FineCellIndex & cell) noexcept
{
  return IsSafeCellIndex(cell.x) && IsSafeCellIndex(cell.y);
}

bool IsValidResolution(double resolution) noexcept
{
  return std::isfinite(resolution) && resolution > 0.0;
}

std::optional<FineCellIndex> FineCellOf(
  double x, double y, double origin_x,
  double origin_y,
  double resolution) noexcept
{
  if (!IsValidResolution(resolution) || !IsSafeWorldCoordinate(x) ||
    !IsSafeWorldCoordinate(y) || !IsSafeWorldCoordinate(origin_x) ||
    !IsSafeWorldCoordinate(origin_y))
  {
    return std::nullopt;
  }
  const double cell_x = std::floor((x - origin_x) / resolution);
  const double cell_y = std::floor((y - origin_y) / resolution);
  if (!std::isfinite(cell_x) || !std::isfinite(cell_y)) {
    return std::nullopt;
  }
  const double limit = static_cast<double>(kMaxCellCoordinate);
  if (cell_x < -limit || cell_x > limit || cell_y < -limit ||
    cell_y > limit)
  {
    return std::nullopt;
  }
  const auto index_x = static_cast<std::int64_t>(cell_x);
  const auto index_y = static_cast<std::int64_t>(cell_y);
  if (!IsSafeCellIndex(index_x) || !IsSafeCellIndex(index_y)) {
    return std::nullopt;
  }
  return FineCellIndex{index_x, index_y};
}

std::optional<FineCellIndex> FineCellOf(
  const Point3 & point, double origin_x,
  double origin_y,
  double resolution) noexcept
{
  return FineCellOf(point.x, point.y, origin_x, origin_y, resolution);
}

Point3 FineCellCenter(
  const FineCellIndex & cell, double origin_x,
  double origin_y, double resolution) noexcept
{
  return Point3{
    origin_x + (static_cast<double>(cell.x) + 0.5) * resolution,
    origin_y + (static_cast<double>(cell.y) + 0.5) * resolution, 0.0};
}

std::int64_t FloorDivide(
  std::int64_t numerator,
  std::int64_t denominator) noexcept
{
  // The public contract requires a positive denominator. A non-positive value
  // returns zero instead of invoking undefined signed-division behaviour.
  if (denominator <= 0) {
    return 0;
  }
  const std::int64_t quotient = numerator / denominator;
  const std::int64_t remainder = numerator % denominator;
  if (remainder != 0 && numerator < 0) {
    return quotient - 1;
  }
  return quotient;
}

double SupportEvidence::SectorCoverage() const noexcept
{
  std::size_t supported = 0;
  for (const bool present : supported_sectors) {
    if (present) {
      ++supported;
    }
  }
  if (kSupportSectorCount == 0) {
    return 0.0;
  }
  return static_cast<double>(supported) /
         static_cast<double>(kSupportSectorCount);
}

bool SupportEvidence::HasSupportToward(double dx, double dy) const noexcept
{
  const int sector = SectorOf(dx, dy);
  if (sector < 0) {
    return false;
  }
  return supported_sectors[static_cast<std::size_t>(sector)];
}

double SupportEvidence::CoverageToward(double dx, double dy) const noexcept
{
  const int sector = SectorOf(dx, dy);
  if (sector < 0) {
    return 0.0;
  }
  const std::size_t first = static_cast<std::size_t>(sector);
  const std::size_t second = (first + 1U) % kSupportSectorCount;
  const double a = supported_sectors[first] ? 1.0 : 0.0;
  const double b = supported_sectors[second] ? 1.0 : 0.0;
  return 0.5 * (a + b);
}

bool TerrainNode::HasValidGeometry() const noexcept
{
  return plane.valid && representative.IsFinite() && support.sample_count > 0;
}

ObstacleState ObstacleAttributes::CombinedState() const noexcept
{
  if (static_state == ObstacleState::kOccupied ||
    dynamic_state == ObstacleState::kOccupied)
  {
    return ObstacleState::kOccupied;
  }
  if (static_state == ObstacleState::kClear &&
    dynamic_state == ObstacleState::kClear)
  {
    return ObstacleState::kClear;
  }
  return ObstacleState::kUnknown;
}

double ObstacleAttributes::NearestDistanceM() const noexcept
{
  const bool static_known = std::isfinite(static_distance_m);
  const bool dynamic_known = std::isfinite(dynamic_distance_m);
  if (static_known && dynamic_known) {
    return std::min(static_distance_m, dynamic_distance_m);
  }
  if (static_known) {
    return static_distance_m;
  }
  if (dynamic_known) {
    return dynamic_distance_m;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

bool ObstacleAttributes::IsClear() const noexcept
{
  return static_state == ObstacleState::kClear &&
         dynamic_state == ObstacleState::kClear;
}

bool ObstacleAttributes::IsOccupied() const noexcept
{
  return static_state == ObstacleState::kOccupied ||
         dynamic_state == ObstacleState::kOccupied;
}

const char * TerrainStatusToString(TerrainStatus status) noexcept
{
  switch (status) {
    case TerrainStatus::kUnknown:
      return "unknown";
    case TerrainStatus::kSupported:
      return "supported";
    case TerrainStatus::kProhibited:
      return "prohibited";
  }
  return "unknown";
}

const char * ProhibitedReasonToString(ProhibitedReason reason) noexcept
{
  switch (reason) {
    case ProhibitedReason::kNone:
      return "none";
    case ProhibitedReason::kInsufficientSamples:
      return "insufficient_samples";
    case ProhibitedReason::kDegenerateGeometry:
      return "degenerate_geometry";
    case ProhibitedReason::kHighResidual:
      return "high_residual";
    case ProhibitedReason::kExcessiveLayers:
      return "excessive_layers";
    case ProhibitedReason::kNonFiniteInput:
      return "non_finite_input";
    case ProhibitedReason::kExcessiveSlope:
      return "excessive_slope";
    case ProhibitedReason::kExplicitNoEntry:
      return "explicit_no_entry";
    case ProhibitedReason::kObstacle:
      return "obstacle";
    case ProhibitedReason::kDiscontinuousTerrain:
      return "discontinuous_terrain";
    case ProhibitedReason::kUnconfirmedComponent:
      return "unconfirmed_component";
  }
  return "unknown";
}

const char * ConnectionStatusToString(ConnectionStatus status) noexcept
{
  switch (status) {
    case ConnectionStatus::kUnknown:
      return "unknown";
    case ConnectionStatus::kConnected:
      return "connected";
    case ConnectionStatus::kInsufficientEvidence:
      return "insufficient_evidence";
    case ConnectionStatus::kDiscontinuous:
      return "discontinuous";
    case ConnectionStatus::kInconsistentNormals:
      return "inconsistent_normals";
  }
  return "unknown";
}

const char * ConnectionReasonToString(ConnectionReason reason) noexcept
{
  switch (reason) {
    case ConnectionReason::kNone:
      return "none";
    case ConnectionReason::kMissingNeighbor:
      return "missing_neighbor";
    case ConnectionReason::kSparseSurface:
      return "sparse_surface";
    case ConnectionReason::kBoundaryCoverage:
      return "boundary_coverage";
    case ConnectionReason::kHeightStep:
      return "height_step";
    case ConnectionReason::kSlopeLimit:
      return "slope_limit";
    case ConnectionReason::kNormalMismatch:
      return "normal_mismatch";
    case ConnectionReason::kProhibitedSurface:
      return "prohibited_surface";
    case ConnectionReason::kNonFinite:
      return "non_finite";
    case ConnectionReason::kSameCellNoLink:
      return "same_cell_no_link";
    case ConnectionReason::kUnconfirmedComponent:
      return "unconfirmed_component";
  }
  return "unknown";
}

const char * ObstacleStateToString(ObstacleState state) noexcept
{
  switch (state) {
    case ObstacleState::kUnknown:
      return "unknown";
    case ObstacleState::kClear:
      return "clear";
    case ObstacleState::kOccupied:
      return "occupied";
  }
  return "unknown";
}

}  // namespace grid_global_planner
