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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_BOUNDARY_COST_H_
#define DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_BOUNDARY_COST_H_

namespace dddmr_grid_global_planner
{

// Shared soft risk for search and visualization: weight * (1-d/range)^2 inside
// the range, zero outside. Zero range/weight explicitly disables this term.
// Invalid ranges/weights throw; unknown/non-finite distances charge full risk.
// This is a topology-based preference, not a collision-clearance certificate.
double TerrainBoundaryPenalty(double distance_m, double range_m, double weight);

// Applies the same bounded soft kernel to one evidence family. Positive
// infinity means "no seed of this family" and contributes zero; NaN or a
// negative distance is invalid evidence and charges the full weight. Zero
// range/weight still explicitly disables the term. This is the guarded form for
// the separate cache arrays; do not feed their positive infinity to
// TerrainBoundaryPenalty(), which deliberately maps every non-finite distance
// to full risk.
double TerrainBoundaryEvidenceComponentPenalty(
  double distance_m, double range_m, double weight);

// Shared conservative risk for the separate measured-support-edge and
// insufficient-connection distances. Returns max of the two component
// penalties, which equals the penalty of the minimum finite distance because
// the kernel is monotone. If both distances are positive infinity there is no
// classified boundary in the component; this compatibility case charges the
// full bounded weight, matching the previous infinity-to-full-risk behavior.
// Otherwise an infinite family contributes zero, so a missing uncertainty seed
// never floods a finite geometric distance with full risk. Zero range/weight
// disables the soft term without changing or invalidating the evidence labels.
double TerrainBoundaryEvidencePenalty(
  double measured_support_distance_m,
  double insufficient_connection_distance_m,
  double range_m, double weight);

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_BOUNDARY_COST_H_
