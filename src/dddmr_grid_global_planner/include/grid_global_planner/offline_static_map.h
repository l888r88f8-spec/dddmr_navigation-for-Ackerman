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

#ifndef GRID_GLOBAL_PLANNER__OFFLINE_STATIC_MAP_H_
#define GRID_GLOBAL_PLANNER__OFFLINE_STATIC_MAP_H_

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "grid_global_planner/continuous_ground_surface.h"
#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/terrain_types.h"

namespace grid_global_planner
{

// Default ground-mode continuous-surface policy. These are map-noise and
// model bounds, not vehicle limits: enable the module only for the legacy
// ground path; global-cloud mode keeps its existing behaviour. A neighbour is
// associated only when the two fitted planes agree at their shared boundary
// within max_height_noise_m, so a real step/layer edge is never smoothed across.
  struct GroundSurfaceReconstructionOptions
  {
    bool enabled {false};
    // Maximum shared-boundary height disagreement treated as map noise. Larger
    // disagreement is preserved as a real discontinuity and that neighbour is
    // excluded from the joint local surface.
    double max_height_noise_m {0.06};
    // Hard per-cell vertical correction bound. Exceeding it keeps the original
    // measured fit unchanged and records the diagnostic reason.
    double max_correction_m {0.06};
    double max_normal_correction_rad {0.15};
    double max_association_normal_rad {0.50};
    // The reconstructed plane must still explain this node's own raw support
    // points within this RMS. It never replaces or relabels those points.
    double max_support_residual_m {0.06};
    // Candidate plane must remain within the existing offline slope safety bound.
    double max_candidate_slope_rad {0.60};
    // Bounded symmetric neighbourhood (8 adjacent cells) and per-node spatial
    // sample cap. Each contributed node is capped equally so a dense one-sided
    // patch cannot dominate the joint estimate.
    std::size_t max_neighbors {8};
    std::size_t max_points_per_node {12};
  };

// Explicit offline map-construction policy.
//
// This builder represents a *measured 2.5-D surface* at the fine resolution by
// fitting one plane from the real same-layer neighbourhood points. It never
// invents points, never interpolates across a gap larger than the configured
// sampling scale, and never joins different floors. Every threshold below is an
// offline algorithm assumption, not a measured vehicle capability.
  struct OfflineStaticMapOptions
  {
    // Fine representation grid shared with the live terrain snapshot.
    double resolution {0.10};
    double origin_x {0.0};
    double origin_y {0.0};

    // Neighbourhood used to fit one fine-cell plane. The maximum sampling gap is
    // the largest centre-to-support distance still represented as measured
    // surface. A hole wider than this remains unknown; a sub-sampling-scale hole
    // cannot be proven from the input and is explicitly not claimed.
    double neighborhood_radius_m {0.35};
    double max_sampling_gap_m {0.30};

    // Legacy entry (and global mode below): evidence-insufficient layers only.
    // The legacy expanded radius is neighborhood_radius_m + this step; it is
    // never a new global neighbourhood and never rescues an RMS/conflict failure.
    double evidence_expansion_step_m {0.10};

    // Global-cloud mode only. The adaptive same-floor retry starts at
    // neighborhood_radius_m + evidence_expansion_step_m, adds one step at a time,
    // and stops at the first accepted radius with enough spatially balanced
    // coverage. The maximum is bounded by neighborhood_radius_m +
    // max_sampling_gap_m; there is no point/ref/patch capacity budget. The default
    // 0.55 m is below a 0.60 m hole so that hole stays unknown.
    double global_surface_expansion_max_radius_m {0.55};

    // Global-cloud mode only. Fraction of the eight angular sectors that must
    // contain selected same-floor samples at the chosen radius. Sectors count
    // once regardless of point density, so a dense one-sided patch cannot win by
    // raw point count. This is an algorithm assumption, not a vehicle limit.
    double global_surface_min_angular_coverage {0.50};

    // Plane/evidence gates copied from the live terrain evidence policy (not
    // vehicle limits).
    double max_plane_rms_m {0.05};
    double max_normal_angle_rad {0.35};
    double max_slope_rad {0.60};
    double max_step_height_m {0.12};
    double layer_gap_threshold_m {0.20};
    double min_boundary_coverage {0.25};
    std::size_t min_neighborhood_points {4};
    bool mark_excessive_slope_prohibited {true};

    // Candidate-cell safety bound. The default is far above a normal PCD and
    // prevents an accidental whole-world expansion.
    std::size_t max_candidate_cells {2'000'000};

    // Measured evidence has no artificial point/patch/ref capacity limit.
    // Real numeric-integrity checks (uint32 index representability and size_t
    // overflow) remain in the builder and still fail closed.

    // Obstacle classification against the reconstructed ground surface.
    // Points within the duplicate tolerance of the local ground are treated as
    // ground (the default map_dir == ground_dir case). Points above ground and
    // at most blocking height above it are static 2.5-D obstacles. Points above
    // that band are counted as ignored overhead evidence, because this model
    // does not validate vehicle headroom.
    double ground_duplicate_tolerance_m {0.08};
    double obstacle_blocking_height_m {1.00};
    double obstacle_association_radius_m {0.25};

    // Tile policy for the node-keyed static obstacle layer.
    double obstacle_tile_size_m {3.2};

    // Global-cloud surface mode only. A component is a confirmed ground seed
    // when it reaches the lowest measured surface within this tolerance, or when
    // an optional auxiliary ground point matches its fitted local surface.
    // Other components stay as unconfirmed geometry: they are neither proven
    // ground nor proven box tops. This is an algorithm assumption, not a vehicle
    // capability threshold.
    double global_ground_seed_tolerance_m {0.15};

    // Global-cloud mode only. A hint point seeds a component only when its
    // height residual against that component's fitted plane is within this
    // tolerance. The default equals ground_duplicate_tolerance_m and validation
    // keeps it below layer_gap_threshold_m so it cannot select the other floor.
    // Global-cloud mode only. A hint point seeds a component only when its
    // height residual against that component's fitted plane is within this
    // tolerance. The default equals ground_duplicate_tolerance_m and validation
    // keeps it below layer_gap_threshold_m so it cannot select the other floor.
    double global_hint_surface_tolerance_m {0.08};

    // Default ground-mode same-layer continuous reconstruction. Disabled by
    // default in the builder so existing unit callers keep exact legacy output;
    // the checked-in launch profile enables it explicitly.
    GroundSurfaceReconstructionOptions ground_surface;

    // Regional continuous layered reconstruction. When enabled it runs before
    // the legacy per-cell RMS veto, may create inferred cells without own
    // samples, and supersedes the old local smoothing pass where they conflict.
    // Disabled at the API level; the checked-in YAML opts in.
    ContinuousGroundSurfaceOptions continuous_ground;
  };

// Build status. A failure never yields a partial or assumed-free snapshot.
  enum class OfflineStaticMapStatus: std::uint8_t
  {
    kOk = 0,
    kInvalidOptions,
    kEmptyGround,
    kTooManyCandidateCells,
    // Real numeric-integrity failure (uint32 index range or size_t overflow),
    // not a configured measurement capacity.
    kMeasurementBudgetExceeded,
    kBuildFailed,
  };

// Stable diagnostic spelling.
  const char * OfflineStaticMapStatusToString(
    OfflineStaticMapStatus status) noexcept;

// Bounded reconstruction diagnostic example. Examples are one per failed gate
// and capped by kOfflineGateExampleLimit; they never contain per-point history.
  inline constexpr std::size_t kOfflineGateExampleLimit = 8U;

  struct OfflineGateExample
  {
    // Stable gate names: insufficient_points, sampling_gap,
    // unsupported_center, plane_fit, plane_rms, plane_inliers,
    // nonfinite_plane, prohibited_slope, robust_depression.
    std::string gate;
    FineCellIndex cell {};
    // Candidate cell centre with the fitted/sample Z where available.
    Point3 point {};
    double actual {std::numeric_limits < double > ::quiet_NaN()};
    double threshold {std::numeric_limits < double > ::quiet_NaN()};
  };

// Actual measured construction counts and wall times. These are build
// diagnostics, not planning success or vehicle-feasibility claims.
  struct OfflineStaticMapMetrics
  {
    std::size_t ground_points {0};
    std::size_t obstacle_points {0};
    std::size_t candidate_cells {0};
    std::size_t supported_nodes {0};
    std::size_t unknown_candidate_cells {0};
    std::size_t connections {0};
    std::size_t obstacle_records {0};
    std::size_t duplicate_ground_obstacles {0};
    std::size_t ignored_overhead_obstacles {0};
    // Global-cloud mode: accepted geometry not connected to a seeded ground
    // component is retained with unknown status (not proven ground, not proven
    // box top), plus the number of seeded ground and unconfirmed components.
    std::size_t unconfirmed_nodes {0};
    std::size_t unconfirmed_components {0};
    std::size_t ground_components {0};

    // Measured evidence actually stored in this build, not the raw input count.
    std::size_t measurement_points {0};
    std::size_t measurement_patches {0};
    std::size_t measurement_point_refs {0};
    // Numeric-integrity failures only; there is no configured measurement
    // capacity that can increment this counter.
    std::size_t measurement_budget_rejections {0};

    // Reconstruction scopes. candidate_layers counts every measured Z layer;
    // accepted_nodes counts every retained node (supported or prohibited).
    // accepted_nodes + all rejected_* equals candidate_layers.
    std::size_t candidate_layers {0};
    std::size_t accepted_nodes {0};
    std::size_t prohibited_nodes {0};
    std::size_t rejected_insufficient_points {0};
    std::size_t rejected_sampling_gap {0};
    std::size_t rejected_unsupported_center {0};
    std::size_t rejected_plane_fit {0};
    std::size_t rejected_plane_rms {0};
    std::size_t rejected_plane_inliers {0};
    std::size_t rejected_nonfinite_plane {0};
    std::size_t rejected_prohibited_slope {0};
    std::size_t rejected_robust_depression {0};

    // Ordinary-fit failures that entered bounded robust refinement, and the
    // resulting accepted nodes. Attempt counters are diagnostics, not rejection
    // totals; final rejections are the rejected_* fields above.
    std::size_t ordinary_plane_failures {0};
    std::size_t ordinary_rms_rejections {0};
    std::size_t robust_attempts {0};
    std::size_t robust_recovered_nodes {0};
    std::size_t robust_rejected {0};
    std::size_t robust_refinement_iterations {0};

    // Retained rejected-layer diagnostics. These are additional accounting and do
    // not replace the legacy rejected_* counters used by the candidate_layers
    // invariant. Expanded attempts count one bounded same-floor retry; recoveries
    // count attempts whose expanded fit became the selected patch fit.
    std::size_t local_depression_rejections {0};
    std::size_t patches_with_raised_evidence {0};
    std::size_t expanded_patch_attempts {0};
    std::size_t expanded_patch_recoveries {0};
    // Ground-mode quality-failure context re-estimation, counted separately
    // from the legacy evidence-insufficiency expansion. Recoveries are a subset
    // of expanded_patch_recoveries; rejections never create a node.
    std::size_t context_reestimation_attempts {0};
    std::size_t context_reestimation_recoveries {0};
    std::size_t context_reestimation_rejections {0};
    std::size_t rejected_expansion_invalid {0};
    // Global-cloud adaptive scales that fit geometrically but were rejected
    // because the selected same-floor samples did not cover enough sectors.
    std::size_t rejected_adaptive_coverage {0};

    // Default ground-mode continuous reconstruction. Every applied node is still
    // backed by its original measured support; these counters never relax the
    // physical gates above.
    std::size_t surface_reconstruction_considered {0};
    std::size_t surface_reconstruction_applied {0};
    std::size_t surface_reconstruction_preserved_discontinuity {0};
    std::size_t surface_reconstruction_rejected_support {0};
    std::size_t surface_reconstruction_rejected_correction {0};
    std::size_t surface_reconstruction_rejected_normal {0};
    std::size_t surface_reconstruction_rejected_evidence {0};
    double surface_reconstruction_max_correction_m {0.0};

    std::vector < OfflineGateExample > gate_examples;

    double terrain_ms {0.0};
    double connection_ms {0.0};
    double obstacle_ms {0.0};
    double snapshot_ms {0.0};
    double total_ms {0.0};
  };

  struct OfflineStaticMapResult
  {
    OfflineStaticMapStatus status {OfflineStaticMapStatus::kInvalidOptions};
    std::string detail;
    std::shared_ptr < const MapSnapshot > snapshot;

    // Full diagnostic evidence is retained only when a planner snapshot cannot be
    // published (for example no layer passed). It is deliberately separate from a
    // usable snapshot and is never treated as traversability evidence.
    std::shared_ptr < const MeasurementEvidence > diagnostic_evidence;
    OfflineStaticMapMetrics metrics;

    bool ok() const noexcept
    {
      return status == OfflineStaticMapStatus::kOk && snapshot != nullptr;
    }

    // Returns the one immutable evidence store for this build: the snapshot-owned
    // terrain evidence when usable, otherwise the separate failclosed diagnostic
    // evidence. Points are never duplicated between the two paths.
    const MeasurementEvidence * measurement_evidence() const noexcept
    {
      if (snapshot != nullptr && snapshot->fine_terrain() != nullptr) {
        return &snapshot->fine_terrain()->measurement_evidence();
      }
      return diagnostic_evidence.get();
    }
  };

// Builds one immutable static-map snapshot from authoritative ground and
// obstacle point clouds. The same TerrainSnapshot/MapSnapshot types and the
// same AckermannHybridAStar/PathValidator interfaces consume this output.
//
// The returned snapshot carries an offline 2.5-D static obstacle layer keyed by
// terrain node. It is not ObservedVolume/Ray free-space evidence and must never
// be presented as such.
  OfflineStaticMapResult BuildOfflineStaticMap(
    const std::vector < Point3 > & ground_points,
    const std::vector < Point3 > & obstacle_points,
    const OfflineStaticMapOptions & options,
    const std::string & frame_id);

// Full-cloud surface entry. Every measured layer in global_points is a
// candidate ground/ramp/deck surface; points are not pre-labelled as ground.
// auxiliary_ground_points is an optional hint (for example GroundMap.pcd): it
// is merged as extra measured samples and seeds otherwise disconnected but
// explicitly ground-labelled components. It may be empty. The global cloud is
// always retained as obstacle evidence; obstacle_points is an optional
// additive source and never replaces it. Components not connected to a seeded
// ground component remain fitted geometry with unknown status, so they are
// neither silently joined to the road nor asserted to be box tops. Measured
// obstacle points from those components remain occupied evidence. This is not
// an implementation of upstream PCT.
  OfflineStaticMapResult BuildOfflineGlobalCloudMap(
    const std::vector < Point3 > & global_points,
    const std::vector < Point3 > & auxiliary_ground_points,
    const std::vector < Point3 > & obstacle_points,
    const OfflineStaticMapOptions & options,
    const std::string & frame_id);

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__OFFLINE_STATIC_MAP_H_
