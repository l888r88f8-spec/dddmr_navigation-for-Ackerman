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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__GROUND_SURFACE_RECONSTRUCTION_H_
#define DDDMR_GRID_GLOBAL_PLANNER__GROUND_SURFACE_RECONSTRUCTION_H_

#include <cstddef>
#include <vector>

#include "dddmr_grid_global_planner/measurement_evidence.h"
#include "dddmr_grid_global_planner/offline_static_map.h"
#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Bounded neighbourhood/sample policy. The neighbour set is exactly the eight
// adjacent fine cells; the per-node cap prevents one dense measured patch from
// dominating the joint fit and keeps reservation arithmetic overflow-free.
  inline constexpr std::size_t kMaxGroundSurfaceNeighbors = 8U;
  inline constexpr std::size_t kMaxGroundSurfacePointsPerNode = 4096U;

// One deterministic same-layer regularization pass over accepted ground nodes.
// For each supported node it gathers adjacent-cell planes whose shared-boundary
// height agrees within the map-noise bound, fits a robust local height plane to
// a spatially balanced sample of the real support points (equal per-node cap),
// and applies a bounded height/normal correction. The result is written into
// the planning TerrainNode and mirrored into the linked MeasurementPatch
// provenance. Raw support points, raw fit evidence and physical gates are never
// rewritten. Disabled or unsafe cases leave the original node untouched.
  struct GroundSurfaceReconstructionMetrics
  {
    std::size_t considered_nodes {0};
    std::size_t applied_nodes {0};
    std::size_t preserved_discontinuities {0};
    std::size_t rejected_insufficient_support {0};
    std::size_t rejected_correction {0};
    std::size_t rejected_normal {0};
    std::size_t rejected_evidence {0};
    std::size_t rejected_geometry {0};
    double max_applied_correction_m {0.0};
  };

  GroundSurfaceReconstructionMetrics ReconstructGroundSurfaces(
    const std::vector < TerrainNode > & nodes,
    const std::vector < MeasurementPatch > & patches,
    const GroundSurfaceReconstructionOptions & options,
    double cell_resolution_m,
    std::vector < TerrainNode > * reconstructed_nodes,
    std::vector < MeasurementPatch > * reconstructed_patches);

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__GROUND_SURFACE_RECONSTRUCTION_H_
