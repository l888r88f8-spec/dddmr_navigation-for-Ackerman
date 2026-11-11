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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_GRID_BUILDER_H_
#define DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_GRID_BUILDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "dddmr_grid_global_planner/input_contract.h"
#include "dddmr_grid_global_planner/map_snapshot.h"
#include "dddmr_grid_global_planner/metrics.h"

namespace dddmr_grid_global_planner
{

// Builder configuration. Geometric thresholds are terrain-extraction evidence
// limits, not vehicle capability claims. Vehicle-specific support checks belong
// to the later wheel-support evaluator.
struct TerrainGridBuilderOptions
{
  // Authoritative fine XY resolution and shared map origin.
  double resolution{0.1};
  double origin_x{0.0};
  double origin_y{0.0};

  // Initial per-cell Z grouping gap. This is only a coarse pre-segmentation
  // value; final surfaces are split again by fit residual and largest Z gap so
  // a curb inside one cell is not averaged into one plane.
  double layer_gap_threshold{0.20};

  // Smallest local height separation considered by residual/boundary-driven
  // recursive splitting. This value also gates the cross-surface nearest-point
  // continuity probe used to distinguish a curb from a continuous slope.
  double refine_split_min_gap{0.03};

  // Smallest plane-RMS improvement that justifies an extra split when the
  // parent plane is already within max_plane_rms. A small curb can otherwise
  // hide inside a passing global fit: a 4 cm step split evenly across one cell
  // has only 2 cm RMS, so residual alone would never request a split.
  double refine_min_rms_improvement{0.005};

  // Maximum residual-driven split depth per initial layer. This is an explicit
  // segmentation budget, not a capability or confidence value. The depth is
  // capped at 64 so the recursion remains a deliberate bounded budget.
  std::size_t max_refine_depth{8};

  // Maximum plane RMS residual for a supported surface. Larger residuals stay
  // unknown instead of being reported as a flat/supported plane.
  double max_plane_rms{0.05};

  // Minimum raw samples for a valid plane fit. Sample count is not a confidence
  // value; it is only an evidence sufficiency gate.
  std::size_t min_samples_per_surface{3};

  // Safety bound per cell. Surfaces above the bound keep their evidence but are
  // marked prohibited with reason excessive_layers.
  std::size_t max_layers_per_cell{16};

  // Candidate-connection continuity limits.
  double max_step_height{0.12};
  double max_slope_rad{0.60};
  double max_normal_angle_rad{0.35};
  double min_boundary_coverage{0.25};

  // Support-sector radius as a fraction of the fine resolution. Sectors are
  // angular coverage evidence; they are not a wheel contact model.
  double support_radius_factor{0.75};

  // Mark slopes above max_slope_rad as prohibited terrain. This is a terrain
  // sanity gate, not a measured suspension/tire capability.
  bool mark_excessive_slope_prohibited{true};

  // Input validation policy, including frame and monotonic freshness.
  ObservationValidationOptions observation_validation{};
};

// Builder outcome. A rejected observation leaves the prior snapshot untouched
// and never produces an assumed-free map.
enum class TerrainBuildStatus : std::uint8_t
{
  kOk = 0,
  kInvalidOptions,
  kObservationRejected,
  kUnsupportedUpdateSemantics,
  kContentUnchanged,
};

// One build result with a shared immutable snapshot and a diagnostic detail.
struct TerrainBuildResult
{
  TerrainBuildStatus status{TerrainBuildStatus::kInvalidOptions};
  std::string detail;
  std::shared_ptr<const TerrainSnapshot> snapshot;
  // Actual phase timings and content counts for this build attempt. They are
  // filled even when the observation is rejected.
  TerrainBuildMetrics metrics;

  // True only when a snapshot was produced successfully.
  bool ok() const noexcept {return status == TerrainBuildStatus::kOk;}
};

// Single-writer deterministic terrain builder. It converts one frozen ground
// observation into one immutable fine snapshot:
//
// 1. validate frame/completeness/freshness and reject non-finite input;
// 2. floor-index every finite point into 0.1 m XY cells using the shared origin;
// 3. split each cell into multiple Z layers, then recursively split layers by
//    the largest internal Z gap and by two-model region growth seeded by the
//    height extremes; a split is accepted only when both sides have enough
//    samples, the cross-surface boundary is locally discontinuous, and the
//    residual improves materially (curbs/stairs must not be averaged);
// 4. fit each surface, retain its exact raw support points, z bounds, normals,
//    and directional edge evidence so wheel support can query per-surface hulls;
// 5. evaluate adjacent-cell candidate links with boundary-plane height, slope,
//    normal, and directional boundary-support gates, storing rejected
//    candidates explicitly; same-cell surfaces are kept as explicit
//    kSameCellNoLink evidence and are never merged;
// 6. hash canonical content and advance the revision only when content changes.
//
// The builder is not thread-safe; stage 4 introduces the single update worker.
class TerrainGridBuilder
{
public:
  // Constructs a builder with validated-by-Build options.
  explicit TerrainGridBuilder(TerrainGridBuilderOptions options = {});

  // Returns the immutable option record.
  const TerrainGridBuilderOptions & options() const noexcept {return options_;}

  // Returns the last published content revision. Zero means nothing built yet.
  std::uint64_t revision() const noexcept {return revision_;}

  // Validates option ranges without mutating builder state.
  bool ValidateOptions(std::string * detail = nullptr) const;

  // Builds one snapshot from a frozen observation. The observation must use
  // kReplace semantics in this batch; merge/clear are rejected explicitly.
  TerrainBuildResult Build(
    const TerrainPointObservation & observation,
    std::int64_t now_steady_ns);

  // Clears revision tracking so the next build starts a new revision sequence.
  void Reset() noexcept;

private:
  TerrainGridBuilderOptions options_;
  std::uint64_t revision_{0};
  std::uint64_t last_content_hash_{0};
  bool have_content_hash_{false};
};

// Returns a stable diagnostic spelling for a build status.
const char * TerrainBuildStatusToString(TerrainBuildStatus status) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_GRID_BUILDER_H_
