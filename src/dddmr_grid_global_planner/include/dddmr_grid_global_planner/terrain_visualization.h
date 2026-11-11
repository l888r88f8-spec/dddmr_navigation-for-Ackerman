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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_VISUALIZATION_H_
#define DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_VISUALIZATION_H_

#include <cstddef>

#include "builtin_interfaces/msg/time.hpp"
#include "dddmr_grid_global_planner/map_snapshot.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace dddmr_grid_global_planner
{

// Cells per triangle/edge marker pair. Batching avoids one RViz object per cell
// and bounds each marker's vertex array even when the map is large.
constexpr std::size_t kTerrainVisualizationBatchCells = 2048;
// Measurement wireframes are smaller (24 points per patch), so they use their
// own batch bound and an independent render cap.
constexpr std::size_t kMeasurementVisualizationBatchPatches = 1024;

struct TerrainVisualizationOptions
{
  // Legacy diagnostics.max_visualization_points now bounds rendered surfaces.
  // Zero means unlimited. Reaching the limit is reported, never hidden as an
  // apparently unknown patch of terrain.
  std::size_t max_cells{200000};
  // Independent bound for gray rejected/observed measurement wireframes. Zero
  // means unlimited. This never changes the planning map and never renders a
  // supported/prohibited patch a second time.
  std::size_t max_measurement_render_patches{20000};
  // The offline static-map model has no dynamic observer. In that explicit
  // mode colors describe static occupancy only; live mode requires both layers
  // clear before drawing a green surface. Neither is vehicle certification.
  bool static_map_only{false};

  // Per-surface terrain-boundary evidence display shared with the offline
  // evaluator. Clear supported cells use the immutable measured-edge and
  // insufficient-connection distances and the guarded conservative penalty:
  // a known support-domain edge heats green toward yellow/warm, while a cell
  // carrying insufficient connection evidence uses a distinct violet/blue
  // label and is never called a yellow geometric edge or safe green. The
  // hard occupied/prohibited/unknown/unsupported colors always take
  // precedence. Zero range or weight disables only the soft heat/intensity;
  // an own insufficient-evidence seed keeps its warning label because zero
  // cost must not erase the evidence classification. This full/static/offline
  // display is a single-cell evidence hint, not a full-body feasibility or
  // hardware-readiness claim. Values must be finite and non-negative.
  double boundary_penalty_distance_m{0.50};
  double boundary_penalty_weight{1.0};
};

struct TerrainVisualizationResult
{
  visualization_msgs::msg::MarkerArray markers;
  std::size_t rendered_cells{0};
  // Supported cells whose measurement patch was recovered by the bounded
  // same-floor expansion. They keep the normal cost/occupancy color but get a
  // cyan face tint and cyan edge so measured and estimated surfaces are
  // distinguishable without one marker per cell.
  std::size_t estimated_cells{0};
  std::size_t rendered_measurement_patches{0};
  std::size_t invalid_cells{0};
  bool truncated{false};
  bool measurement_truncated{false};
};

// Builds complete XY cell faces at the fitted plane heights plus grid edges.
// Every local surface is independent, including multiple floors in one XY cell.
// Missing/invalid surfaces are never invented. The first marker clears this
// dedicated terrain display so empty/shrunk snapshots cannot leave stale faces.
// This pure message builder performs no publication and never mutates the map.
TerrainVisualizationResult BuildTerrainVisualization(
  const MapSnapshot & snapshot, const builtin_interfaces::msg::Time & stamp,
  const TerrainVisualizationOptions & options = TerrainVisualizationOptions{});

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__TERRAIN_VISUALIZATION_H_
