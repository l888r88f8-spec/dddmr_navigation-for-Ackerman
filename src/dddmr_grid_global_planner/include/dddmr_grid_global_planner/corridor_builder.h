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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__CORRIDOR_BUILDER_H_
#define DDDMR_GRID_GLOBAL_PLANNER__CORRIDOR_BUILDER_H_

#include <cstddef>
#include <string>
#include <vector>

#include "dddmr_grid_global_planner/map_snapshot.h"
#include "dddmr_grid_global_planner/multiresolution_map.h"
#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Corridor enlargement schedule. The initial margin must already cover the
// projected vehicle envelope; the schedule is a bounded search policy, not a
// claim that a narrow corridor is drivable.
struct CorridorOptions
{
  double initial_margin_m{0.20};
  double margin_step_m{0.20};
  std::size_t max_enlargements{3};

  // Safety budget for one read-only corridor view. If exceeded the corridor is
  // marked truncated and the search must fall back instead of assuming complete
  // coverage.
  std::size_t max_corridor_cells{2'000'000};
};

// One read-only fine-cell corridor. No terrain data is copied.
struct Corridor
{
  std::vector<FineCellIndex> cells;
  double margin_m{0.0};
  bool truncated{false};
};

// Builds corridor views around a coarse route or a set of fine route poses. It
// never turns a coarse summary back into terrain and never promotes a coarse
// no-route result into a fine no-route result.
class CorridorBuilder
{
public:
  explicit CorridorBuilder(CorridorOptions options = {});

  const CorridorOptions & options() const noexcept {return options_;}

  bool ValidateOptions(std::string * detail = nullptr) const;

  // Bounded sequence of margins used by coarse->fine fallback.
  std::vector<double> MarginSchedule() const;

  // Builds a corridor covering all fine cells of the coarse route plus the
  // requested margin. Returns an empty corridor when the route is empty.
  Corridor BuildAroundCoarseRoute(
    const TerrainSnapshot & terrain,
    const CoarseTerrainMap & coarse,
    const std::vector<CoarseSurfaceId> & route,
    double margin_m) const;

  // Builds a corridor around projected route poses.
  Corridor BuildAroundPoses(
    const TerrainSnapshot & terrain,
    const std::vector<Point3> & poses,
    double margin_m) const;

  // Builds a corridor around one fine-cell path.
  Corridor BuildAroundCells(
    const TerrainSnapshot & terrain,
    const std::vector<FineCellIndex> & cells,
    double margin_m) const;

private:
  Corridor BuildFromCells(
    const TerrainSnapshot & terrain,
    const std::vector<FineCellIndex> & seeds,
    double margin_m) const;

  CorridorOptions options_;
};

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__CORRIDOR_BUILDER_H_
