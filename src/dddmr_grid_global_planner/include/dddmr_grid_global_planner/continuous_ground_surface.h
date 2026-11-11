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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__CONTINUOUS_GROUND_SURFACE_H_
#define DDDMR_GRID_GLOBAL_PLANNER__CONTINUOUS_GROUND_SURFACE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Regional same-layer ground reconstruction policy. These are offline
// model/noise bounds, not vehicle capabilities. The module runs before the
// legacy per-cell RMS veto: one robust regional fit may represent many fine
// cells, including cells that contain no raw sample. Raw input points are
// immutable; every result keeps the actual supporting point indices.
  struct ContinuousGroundSurfaceOptions
  {
    // Legacy API default is disabled so existing callers keep exact output.
    bool enabled {false};

    // Same-layer evidence radius. The user-approved scale is approximately 1 m.
    double regional_radius_m {1.00};

    // An inferred (no own sample) cell must lie within this distance of real
    // observed seed cells. Exterior cells are additionally rejected by the
    // interior directional-coverage test; the value bounds, but does not by
    // itself authorise, extrapolation.
    double max_gap_fill_m {1.00};

    // Robust regional plane gates. The raw per-cell RMS gate is intentionally not
    // used here: a coherent 1 m fit may accept a cell whose own samples are noisy.
    double max_plane_rms_m {0.15};
    double max_inlier_residual_m {0.18};
    double robust_noise_floor_m {0.03};
    double min_region_confidence {0.35};
    double min_angular_coverage {0.35};

    // Same-layer association at shared cell edges. It must stay at or below the
    // real step bound so a >0.12 m step is never welded into one surface.
    double corner_association_tolerance_m {0.10};
    double max_association_normal_rad {0.50};

    std::size_t min_region_points {6};
    std::size_t max_region_points {768};
    std::size_t max_layers_per_cell {2};
    std::size_t max_candidate_seeds {256};
    std::size_t max_support_refs_per_cell {256};
    std::size_t max_inferred_cells {2'000'000};
  };

// One reconstructed same-layer cell. `point_indices` are real raw input refs;
// `inlier_indices` are the robust fit's actual inliers. `inferred` is true when
// the cell contained no raw point of its own. The four shared corners are in
// absolute XY and are identical for every adjacent cell of the same welded
// layer patch. They are ordered (x0,y0),(x1,y0),(x1,y1),(x0,y1).
  struct ContinuousSurfaceCell
  {
    FineCellIndex cell {};
    Point3 center {};
    PlaneEvidence plane {};
    std::vector < std::uint32_t > point_indices;
    std::vector < std::uint32_t > inlier_indices;
    std::array < Point3, 4 > shared_corners {};
    bool shared_corners_valid {false};
    bool inferred {false};
    double confidence {0.0};
    std::size_t regional_support_cells {0};
    std::size_t own_cell_point_count {0};
  };

  struct ContinuousGroundSurfaceMetrics
  {
    std::size_t measured_cells {0};
    std::size_t seed_layers {0};
    std::size_t observed_surfaces {0};
    std::size_t inferred_cells {0};
    std::size_t inferred_surfaces {0};
    std::size_t rejected_exterior {0};
    std::size_t rejected_confidence {0};
    std::size_t rejected_steps {0};
    std::size_t welded_cells {0};
    std::size_t welded_patches {0};
  };

// Builds the bounded regional reconstruction for the canonical raw point table.
// The returned cells are deterministic (cell X, cell Y, height, inference flag).
// A false return means an input/bound violation and leaves no partial surface.
  bool BuildContinuousGroundSurface(
    const std::vector < Point3 > & points,
    const ContinuousGroundSurfaceOptions & options,
    double resolution,
    double origin_x,
    double origin_y,
    double max_step_height_m,
    double max_slope_rad,
    std::vector < ContinuousSurfaceCell > * cells,
    ContinuousGroundSurfaceMetrics * metrics);

// Shared reconstructed surface query. When shared corner elevations exist, the
// surface uses the same two triangles as the renderer; otherwise this is the legacy
// fitted-plane fallback. Planning and display must both use this function so a
// welded corner is not only a rendering artifact.
  double TerrainSurfaceHeight(
    const TerrainNode & node, double x, double y) noexcept;

// Returns the exact shared cell corners when the node carries a valid welded
// surface. When false, callers should fall back to the fitted plane.
  bool TerrainSurfaceCorners(
    const TerrainNode & node, std::array < Point3, 4 > * corners) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__CONTINUOUS_GROUND_SURFACE_H_
