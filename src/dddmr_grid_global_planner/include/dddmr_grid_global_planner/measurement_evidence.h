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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__MEASUREMENT_EVIDENCE_H_
#define DDDMR_GRID_GLOBAL_PLANNER__MEASUREMENT_EVIDENCE_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// How a measured patch relates to the traversable graph. Only kSupported and
// kProhibited link a TerrainNode; kObservedOnly/kFitRejected preserve rejected
// measurement evidence without inventing a floor. A supported patch is still a
// geometry declaration, not a wheel-support or free-space claim.
  enum class MeasurementPatchStatus: std::uint8_t
  {
    kObservedOnly = 0, // Real samples exist, but no usable plane was fitted.
    kFitRejected = 1,  // A fit was attempted and rejected by a gate/conflict.
    kProhibited = 2,   // Linked to a prohibited node (for example steep slope).
    kSupported = 3,    // Linked to a supported node; still not vehicle clearance.
    // Linked to a real fitted surface whose reachability could not be confirmed.
    // The geometry and refs are retained, but the node is unknown, not ground and
    // not a proven prohibited structure/box top.
    kUnconfirmed = 4,
  };

// Why a measured patch has no usable/supported plane. kLocalRaisedConflict is
// retained for diagnostics only: a raised sample is obstacle evidence and must
// not by itself erase otherwise valid underlying supported geometry.
  enum class MeasurementRejectionReason: std::uint8_t
  {
    kNone = 0,
    kInsufficientPoints,
    kSamplingGap,
    kUnsupportedCenter,
    kDegenerateGeometry,
    kHighResidual,
    kInsufficientInliers,
    kNonFinitePlane,
    kExcessiveSlope,
    kLocalDepression,
    kLocalRaisedConflict,
    kExpansionInvalid,
    kDiscontinuousTerrain,
    // Adaptive expansion had a usable plane but not enough angular sectors.
    kInsufficientCoverage,
    // Geometry is retained, but the component is not a confirmed ground seed.
    kUnconfirmedComponent,
  };

// One actual fitting attempt over a concrete sample set. Both the final/base fit
// and an expanded attempt use this same value and validation path, so residual
// statistics cannot be duplicated or diverge between ordinary/robust/expanded
// code. Residuals are the existing offline_geometry Z-height residuals against
// the centered plane: Dot(normal, point - centroid) / normal.z.
  struct MeasurementFitAttempt
  {
    // Actual collection radius for this attempt. It is not a global neighbourhood
    // change and must be finite/positive.
    double radius_m {std::numeric_limits < double > ::quiet_NaN()};

    // Actual attempt samples, ascending and duplicate-free.
    std::vector < std::uint32_t > point_indices;
    // Exact samples used for the reported RMS/inlier count. For a robust fit these
    // are the robust inliers; for an ordinary fit they are the fitted samples.
    std::vector < std::uint32_t > inlier_indices;

    bool plane_valid {false};          // Geometry exists only; not traversability.
    Point3 centroid {};
    Vector3 normal {0.0, 0.0, 1.0};
    double rms_residual {std::numeric_limits < double > ::quiet_NaN()};
    double nearest_support_xy_m {std::numeric_limits < double > ::quiet_NaN()};
    bool support_gate_ok {false};
    bool center_in_support_hull {false};

    // Full-sample conflict scan. All extrema are recomputed by validation from
    // point_indices and the centered plane, never trusted as arbitrary values.
    std::size_t conflict_sample_count {0};
    double min_signed_residual {std::numeric_limits < double > ::quiet_NaN()};
    double max_signed_residual {std::numeric_limits < double > ::quiet_NaN()};
    double max_depression_m {0.0};
    double max_raised_m {0.0};
    std::size_t raised_conflict_sample_count {0};
    Point3 conflict_point {};          // Must be an exact member of point_indices.
  };

// Why the one bounded expansion/context attempt was made. The actual refs,
// radius and candidate plane stay in MeasurementPatch::expansion; this tag
// distinguishes the legacy evidence-insufficiency retry from the ground-mode
// quality-failure context re-estimation without adding a second fit structure.
  enum class MeasurementExpansionSource: std::uint8_t
  {
    // Also denotes an unspecified source in legacy expansion records.
    kNone = 0,
    kInsufficientEvidence = 1,
    kContextReestimation = 2,
  };

// Immutable per-cell-per-layer measurement evidence. Points are the exact finite
// input points sorted by (x,y,z); every index vector refers into that table and
// is sorted ascending without duplicates.
//
// Raw own-cell observation is deliberately separate from the neighbourhood fit,
// and a failed expansion keeps its actual attempted refs/radius/candidate plane/
// conflict values while the original base refs remain an undeletable subset.
  struct MeasurementPatch
  {
    std::uint64_t id {0};              // Unique within one snapshot, never persisted.
    FineCellIndex cell {};
    int local_layer_index {-1};        // Diagnostic ordinal in this cell only.

    // Original R-neighbourhood refs. Must remain a subset of fit.point_indices and
    // may never be removed by expansion or robust regrouping.
    std::vector < std::uint32_t > base_point_indices;
    std::size_t base_sample_count {0}; // Must equal base_point_indices.size().

    // Finally selected fit: the base fit, or the expanded fit when used.
    MeasurementFitAttempt fit;

    // Failed or successful one-step expansion. A failed attempt retains its actual
    // candidate plane/inliers/refs/radius/reason so annulus residuals remain
    // reconstructable; when no candidate plane exists, plane and residual fields
    // are unavailable rather than invented.
    bool expansion_attempted {false};
    bool expansion_used {false};
    MeasurementFitAttempt expansion;
    MeasurementRejectionReason expansion_reason {MeasurementRejectionReason::kNone};
    MeasurementExpansionSource expansion_source {MeasurementExpansionSource::kNone};

    // Ground-mode context provenance. When expansion_source is
    // kContextReestimation these preserve the original local rejection/steep
    // decision that triggered the broader same-floor re-estimation. They stay
    // empty for every other patch, including the legacy insufficient retry.
    MeasurementRejectionReason context_original_reason {MeasurementRejectionReason::kNone};
    double context_original_slope_rad {std::numeric_limits < double > ::quiet_NaN()};

    // Actual raw own-cell samples: exactly the base-refs subset whose actual
    // FineCellOf(point, origin, resolution) equals patch.cell. A neighbourhood
    // attempt may exist without these, but then the cell is not observed and must
    // not be drawn as a measured per-cell patch.
    std::vector < std::uint32_t > own_cell_point_indices;
    std::size_t own_cell_sample_count {0};
    double own_z_min {std::numeric_limits < double > ::quiet_NaN()};
    double own_z_max {std::numeric_limits < double > ::quiet_NaN()};
    // Final-plane residuals over every own-cell raw sample, including non-inliers.
    double own_cell_min_signed_residual {std::numeric_limits < double > ::quiet_NaN()};
    double own_cell_max_signed_residual {std::numeric_limits < double > ::quiet_NaN()};
    double own_cell_max_depression_m {0.0};
    double own_cell_max_raised_m {0.0};

    // Actual Z range of the final fit samples. It is not the plane, free volume or
    // clearance.
    double z_min {std::numeric_limits < double > ::quiet_NaN()};
    double z_max {std::numeric_limits < double > ::quiet_NaN()};

    MeasurementPatchStatus status {MeasurementPatchStatus::kObservedOnly};
    MeasurementRejectionReason reason {MeasurementRejectionReason::kNone};
    TerrainNodeId node_id {kInvalidTerrainNodeId};

    // Default ground-mode continuous reconstruction provenance. `fit` above
    // remains the raw measured plane; when applied, the linked TerrainNode plane
    // equals this reconstructed geometry instead of `fit`. Correction amplitude,
    // original/reconstruction residuals and confidence are explicit so RViz and
    // consumers can distinguish measured from bounded estimated geometry.
    SurfaceReconstructionEvidence surface_reconstruction {};

    // True only when real own-cell samples exist. Display code must use this
    // predicate rather than a neighbourhood fit or expansion refs.
    bool HasOwnCellObservation() const noexcept
    {
      return !own_cell_point_indices.empty();
    }
  };

// Canonical measurement evidence. The builder sorts raw input first, remaps
// indices and sorts patches; validation never silently reorders or deduplicates.
  struct MeasurementEvidence
  {
    std::vector < Point3 > points;
    std::vector < MeasurementPatch > patches;

    bool empty() const noexcept {return points.empty() && patches.empty();}
  };

// Stable diagnostic spelling.
  const char * MeasurementPatchStatusToString(
    MeasurementPatchStatus status) noexcept;
  const char * MeasurementRejectionReasonToString(
    MeasurementRejectionReason reason) noexcept;

// Fully semantic validation with the map frame needed to check own-cell
// membership. It validates canonical order, every index vector, exact own-cell
// subset, base/own/expansion subset relations, reconstructable plane/residual
// statistics, status/reason/node/cell consistency, finite support fields for
// linked nodes, and failed-expansion metadata. On failure writes one concrete
// detail when detail is non-null and does not allocate a partial snapshot.
  bool ValidateMeasurementEvidence(
    const MeasurementEvidence & evidence,
    const std::vector < TerrainNode > & nodes,
    double origin_x,
    double origin_y,
    double resolution,
    std::string * detail);

// Deterministic 64-bit content hash over every field. Vector lengths are hashed
// before their elements, NaN uses a fixed payload and -0.0 is canonicalized to
// +0.0. Empty evidence returns zero.
  std::uint64_t HashMeasurementEvidence(
    const MeasurementEvidence & evidence) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__MEASUREMENT_EVIDENCE_H_
