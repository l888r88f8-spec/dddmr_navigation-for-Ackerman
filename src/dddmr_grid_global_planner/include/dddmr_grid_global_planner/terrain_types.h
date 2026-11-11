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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_TYPES_H_
#define DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_TYPES_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace dddmr_grid_global_planner
{

// A deterministic identifier for one extracted surface. IDs are assigned by a
// single builder and are stable only inside that terrain revision; they must not
// be persisted across revisions.
  using TerrainNodeId = std::uint64_t;

// Node ID zero is reserved so every default-constructed ID is visibly invalid.
  inline constexpr TerrainNodeId kInvalidTerrainNodeId = 0;

// Input coordinates beyond this bound are rejected instead of being clamped.
// The bound keeps cell conversion finite and far away from integer overflow.
  inline constexpr double kMaxWorldCoordinate = 1.0e7;

// Cell indices beyond this bound are rejected. With a 0.1 m resolution this
// still covers +/-100000 km, far beyond any navigation map.
  inline constexpr std::int64_t kMaxCellCoordinate = 1'000'000'000;

// Number of angular sectors used to summarize finite contact/support coverage.
  inline constexpr std::size_t kSupportSectorCount = 8;

// A finite 3-D point in metres. Construction alone does not validate finiteness;
// callers must call IsFinite() before geometry is used.
  struct Point3
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};

    // Returns true only when all three components are finite.
    bool IsFinite() const noexcept;
  };

// A finite 3-D vector in metres or a unit normal.
  struct Vector3
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};

    // Returns true only when all three components are finite.
    bool IsFinite() const noexcept;

    // Returns the squared Euclidean length.
    double SquaredNorm() const noexcept;

    // Returns the Euclidean length.
    double Norm() const noexcept;

    // Returns a unit vector, or a zero vector when this vector is degenerate.
    Vector3 Normalized() const noexcept;
  };

// Dot product of two vectors.
  double Dot(const Vector3 & lhs, const Vector3 & rhs) noexcept;

// Cross product of two vectors.
  Vector3 Cross(const Vector3 & lhs, const Vector3 & rhs) noexcept;

// Subtraction of two points, producing a vector.
  Vector3 operator - (const Point3 & lhs, const Point3 & rhs)noexcept;

// Subtraction of two vectors.
  Vector3 operator - (const Vector3 & lhs, const Vector3 & rhs)noexcept;

// A planar pose used as a search or validation query. The heading is the yaw
// angle about +Z in radians. The Z field is the projected reference height and is
// not a solved vehicle attitude; WheelSupportEvaluator solves z/roll/pitch.
  struct Pose2D
  {
    double x {0.0};
    double y {0.0};
    double heading {0.0};
  };

// A signed XY cell index at the fine ("authoritative") resolution. Negative
// indices are floor-based so cell -1 contains [-resolution, 0), not
// [0, resolution).
  struct FineCellIndex
  {
    std::int64_t x {0};
    std::int64_t y {0};

    // Exact equality of both integer indices.
    bool operator == (const FineCellIndex & other) const noexcept
    {
      return x == other.x && y == other.y;
    }

    // Deterministic ordering used by builders and hash-free iteration.
    bool operator < (const FineCellIndex & other) const noexcept
    {
      return x < other.x || (x == other.x && y < other.y);
    }
  };

// Hash with a fixed mixing rule. The hash is only for lookup, never for IDs.
  struct FineCellIndexHash
  {
    std::size_t operator()(const FineCellIndex & cell) const noexcept;
  };

// A coarse cell index. It is derived from fine indices by integer floor
// division, never by re-projecting a rounded world coordinate.
  struct CoarseCellIndex
  {
    std::int64_t x {0};
    std::int64_t y {0};

    // Exact equality of both integer indices.
    bool operator == (const CoarseCellIndex & other) const noexcept
    {
      return x == other.x && y == other.y;
    }

    // Deterministic ordering used by coarse map construction.
    bool operator < (const CoarseCellIndex & other) const noexcept
    {
      return x < other.x || (x == other.x && y < other.y);
    }
  };

// Hash for coarse-cell lookup.
  struct CoarseCellIndexHash
  {
    std::size_t operator()(const CoarseCellIndex & cell) const noexcept;
  };

// True when a world coordinate is finite and inside the explicit safe bound.
// Callers must reject, not clamp, when this returns false.
  bool IsSafeWorldCoordinate(double value) noexcept;

// True when a floor-based cell index is inside the explicit safe bound.
  bool IsSafeCellIndex(std::int64_t value) noexcept;

// True when both indices of a fine cell are safe.
  bool IsSafeFineCell(const FineCellIndex & cell) noexcept;

// Converts one world XY point to a floor-based fine cell. Returns nullopt for a
// non-finite input, an invalid resolution/origin, or a cell outside the safe
// integer bound. This is the only supported world-to-cell conversion.
  std::optional < FineCellIndex > FineCellOf(
    double x, double y, double origin_x, double origin_y,
    double resolution) noexcept;

// Convenience overload using Point3. The Z component is ignored.
  std::optional < FineCellIndex > FineCellOf(
    const Point3 & point, double origin_x, double origin_y,
    double resolution) noexcept;

// Returns the XY centre of a fine cell. Callers must pass a safe cell produced
// by FineCellOf or IsSafeFineCell; the result is not clamped.
  Point3 FineCellCenter(
    const FineCellIndex & cell, double origin_x,
    double origin_y, double resolution) noexcept;

// Floor division for a positive denominator. It is used to map fine cells to
// coarse cells without floating-point rounding at negative coordinates.
  std::int64_t FloorDivide(
    std::int64_t numerator,
    std::int64_t denominator) noexcept;

// True when a resolution is finite and strictly positive.
  bool IsValidResolution(double resolution) noexcept;

// Terrain support state. Unknown is the default and must never be interpreted
// as free space. Prohibited is a hard exclusion, not a soft cost.
  enum class TerrainStatus: std::uint8_t
  {
    kUnknown = 0,
    kSupported = 1,
    kProhibited = 2,
  };

// Why a surface is unknown or prohibited. kNone is used only for supported
// surfaces.
  enum class ProhibitedReason: std::uint8_t
  {
    kNone = 0,
    kInsufficientSamples,
    kDegenerateGeometry,
    kHighResidual,
    kExcessiveLayers,
    kNonFiniteInput,
    kExcessiveSlope,
    kExplicitNoEntry,
    kObstacle,
    kDiscontinuousTerrain,
    // Real fitted geometry that is not reachable from a confirmed ground seed.
    // Unknown status is deliberate: geometry alone cannot prove a box top.
    kUnconfirmedComponent,
  };

// Candidate connection state between two surfaces in adjacent fine cells.
// Missing local evidence stays kUnknown or kInsufficientEvidence.
  enum class ConnectionStatus: std::uint8_t
  {
    kUnknown = 0,
    kConnected = 1,
    kInsufficientEvidence = 2,
    kDiscontinuous = 3,
    kInconsistentNormals = 4,
  };

// Reason attached to a candidate connection, including rejected candidates.
  enum class ConnectionReason: std::uint8_t
  {
    kNone = 0,
    kMissingNeighbor,
    kSparseSurface,
    kBoundaryCoverage,
    kHeightStep,
    kSlopeLimit,
    kNormalMismatch,
    kProhibitedSurface,
    kNonFinite,
    kSameCellNoLink,
    // Geometry is contiguous but the component is not a confirmed ground seed.
    kUnconfirmedComponent,
  };

// A fitted local plane and its evidence. A plane is valid only when the fit had
// enough non-collinear support; validity is not a vehicle-traversability claim.
  struct PlaneEvidence
  {
    bool valid {false};
    Point3 centroid {};
    Vector3 normal {0.0, 0.0, 1.0};
    double offset {0.0};
    double rms_residual {std::numeric_limits < double > ::quiet_NaN()};
    std::size_t inlier_count {0};
    std::size_t sample_count {0};
  };

// Raw support evidence retained for later contact-patch and boundary checks.
// The points are the exact finite input points assigned to this surface, sorted
// lexicographically by the builder for deterministic content hashes.
  struct SupportEvidence
  {
    std::vector < Point3 > points;
    double z_min {std::numeric_limits < double > ::quiet_NaN()};
    double z_max {std::numeric_limits < double > ::quiet_NaN()};
    double edge_support_coverage {0.0};
    std::array < bool, kSupportSectorCount > supported_sectors {};
    std::size_t sample_count {0};

    // Returns the fraction of angular sectors that contain support.
    double SectorCoverage() const noexcept;

    // Returns true when the sector facing the given XY direction contains
    // support. Missing support must not be treated as a valid boundary.
    bool HasSupportToward(double dx, double dy) const noexcept;

    // Returns a conservative coverage estimate toward an XY direction, averaging
    // the nearest sector and its neighbour so a single point cannot claim a full
    // boundary.
    double CoverageToward(double dx, double dy) const noexcept;
  };

// One immutable fine terrain surface. The node deliberately does not contain
// search g/h/parent/open/closed state or obstacle attributes; those belong to a
// planning job or a separate obstacle layer.
//
// Reconstruction provenance. A ground-mode continuous surface may adjust the
// fitted plane/representative within a bounded map-noise correction. Raw
// measured support stays in SupportEvidence; the original fitted geometry stays
// here and in the linked MeasurementPatch, so a corrected height is never
// presented as a raw measurement.
  enum class SurfaceReconstructionStatus: std::uint8_t
  {
    kNotAttempted = 0,
    kApplied,
    kPreservedDiscontinuity,
    kInsufficientSupport,
    kCorrectionExceeded,
    kNormalExceeded,
    kEvidenceResidualExceeded,
    kGeometryRejected,
  };

  struct SurfaceReconstructionEvidence
  {
    bool attempted {false};
    bool applied {false};
    SurfaceReconstructionStatus status {
      SurfaceReconstructionStatus::kNotAttempted
    };

    Point3 original_centroid {};
    Vector3 original_normal {0.0, 0.0, 1.0};
    Point3 reconstructed_centroid {};
    Vector3 reconstructed_normal {0.0, 0.0, 1.0};

    double original_height_m {std::numeric_limits < double > ::quiet_NaN()};
    double reconstructed_height_m {std::numeric_limits < double > ::quiet_NaN()};
    // Signed reconstructed minus original height at the cell centre.
    double correction_m {0.0};
    double original_rms_residual_m {std::numeric_limits < double > ::quiet_NaN()};
    double reconstruction_rms_residual_m {
      std::numeric_limits < double > ::quiet_NaN()
    };
    double own_support_rms_residual_m {
      std::numeric_limits < double > ::quiet_NaN()
    };
    double confidence {0.0};
    std::size_t support_nodes {0};
    std::size_t support_points {0};

    // Regional continuous-surface provenance. `inferred` means the cell had no
    // own raw sample; `regional_support_cells` counts real measured seed cells
    // that justified it. `shared_corners_valid` carries four absolute welded
    // corner elevations ordered (x0,y0),(x1,y0),(x1,y1),(x0,y1). When valid,
    // planning and display evaluate this bilinear surface instead of the fitted
    // plane, so adjacent cells agree exactly on common edges. These fields stay
    // default/zero for legacy callers and old local smoothing.
    bool inferred {false};
    bool shared_corners_valid {false};
    std::array < Point3, 4 > shared_corners {};
    std::size_t regional_support_cells {0};
  };

  struct TerrainNode
  {
    TerrainNodeId id {kInvalidTerrainNodeId};
    FineCellIndex cell {};
    int local_surface_index {-1};
    Point3 representative {};
    PlaneEvidence plane {};
    SupportEvidence support {};
    TerrainStatus status {TerrainStatus::kUnknown};
    ProhibitedReason reason {ProhibitedReason::kInsufficientSamples};
    SurfaceReconstructionEvidence surface_reconstruction {};

    // Returns true only for a fit with valid geometry and at least one finite
    // support point.
    bool HasValidGeometry() const noexcept;

    // Returns true when the surface can be considered supported terrain. This is
    // not a wheel-support or vehicle-feasibility result.
    bool IsSupported() const noexcept
    {
      return status == TerrainStatus::kSupported;
    }
  };

// One candidate link between two surfaces in adjacent fine cells. Both accepted
// and rejected candidates are stored so missing/weak evidence stays explicit.
  struct TerrainConnection
  {
    TerrainNodeId source {kInvalidTerrainNodeId};
    TerrainNodeId target {kInvalidTerrainNodeId};
    ConnectionStatus status {ConnectionStatus::kUnknown};
    ConnectionReason reason {ConnectionReason::kMissingNeighbor};
    double height_delta {std::numeric_limits < double > ::quiet_NaN()};
    double horizontal_distance {std::numeric_limits < double > ::quiet_NaN()};
    double slope {std::numeric_limits < double > ::quiet_NaN()};
    double boundary_coverage {0.0};
    double normal_dot {0.0};

    // True only when local continuity and boundary evidence passed.
    bool IsConnected() const noexcept
    {
      return status == ConnectionStatus::kConnected;
    }
  };

// Obstacle occupancy state. Unknown is not clear: a missing observation must
// never be converted into an assumed free path.
  enum class ObstacleState: std::uint8_t
  {
    kUnknown = 0,
    kClear = 1,
    kOccupied = 2,
  };

// Obstacle attributes are stored separately from TerrainNode and linked by node
// ID. This avoids copying every terrain node during a dynamic obstacle update.
  struct ObstacleAttributes
  {
    ObstacleState static_state {ObstacleState::kUnknown};
    ObstacleState dynamic_state {ObstacleState::kUnknown};
    double static_distance_m {std::numeric_limits < double > ::quiet_NaN()};
    double dynamic_distance_m {std::numeric_limits < double > ::quiet_NaN()};
    double clearance_above_m {std::numeric_limits < double > ::quiet_NaN()};
    bool clearance_known {false};
    // Vertical interval of the obstacle slab that contributed to this node. This
    // is explicit evidence, not a synthetic vehicle height: an overhead bridge
    // deck can be clear at road level while still carrying a known slab interval
    // that a body-envelope check must test. Unset means the slab extent is
    // unknown and a physical-envelope consumer must fail closed.
    bool occupied_vertical_extent_known {false};
    double occupied_min_z {std::numeric_limits < double > ::quiet_NaN()};
    double occupied_max_z {std::numeric_limits < double > ::quiet_NaN()};
    // Vertical interval actually covered by an explicit known/clear observation
    // volume for this node. It is evidence about observed free space, never an
    // inferred bounding box of occupied points.
    bool observed_vertical_extent_known {false};
    double observed_min_z {std::numeric_limits < double > ::quiet_NaN()};
    double observed_max_z {std::numeric_limits < double > ::quiet_NaN()};
    std::int64_t observation_stamp_ns {0};
    std::int64_t received_steady_ns {0};

    // Combines layers conservatively: either occupied layer prohibits, both clear
    // is clear, and any unknown layer remains unknown.
    ObstacleState CombinedState() const noexcept;

    // Returns the nearest known obstacle distance, or NaN when neither is known.
    double NearestDistanceM() const noexcept;

    // Returns true only when both layers are explicitly clear.
    bool IsClear() const noexcept;

    // Returns true when either layer is explicitly occupied.
    bool IsOccupied() const noexcept;
  };

// Returns a stable diagnostic spelling for a terrain status.
  const char * TerrainStatusToString(TerrainStatus status) noexcept;

// Returns a stable diagnostic spelling for a prohibited reason.
  const char * ProhibitedReasonToString(ProhibitedReason reason) noexcept;

// Returns a stable diagnostic spelling for a connection status.
  const char * ConnectionStatusToString(ConnectionStatus status) noexcept;

// Returns a stable diagnostic spelling for a connection reason.
  const char * ConnectionReasonToString(ConnectionReason reason) noexcept;

// Returns a stable diagnostic spelling for an obstacle state.
  const char * ObstacleStateToString(ObstacleState state) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_TYPES_H_
