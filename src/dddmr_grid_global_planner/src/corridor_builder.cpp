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

#include "dddmr_grid_global_planner/corridor_builder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dddmr_grid_global_planner
{

CorridorBuilder::CorridorBuilder(CorridorOptions options)
: options_(std::move(options)) {}

bool CorridorBuilder::ValidateOptions(std::string * detail) const
{
  const auto reject = [detail](const std::string & message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return false;
    };
  if (!std::isfinite(options_.initial_margin_m) ||
    options_.initial_margin_m < 0.0)
  {
    return reject("initial_margin_m must be finite and non-negative");
  }
  if (!std::isfinite(options_.margin_step_m) ||
    options_.margin_step_m < 0.0)
  {
    return reject("margin_step_m must be finite and non-negative");
  }
  if (options_.max_corridor_cells == 0) {
    return reject("max_corridor_cells must be positive");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

std::vector<double> CorridorBuilder::MarginSchedule() const
{
  std::vector<double> margins;
  if (!ValidateOptions(nullptr)) {
    return margins;
  }
  margins.reserve(options_.max_enlargements + 1);
  margins.push_back(options_.initial_margin_m);
  for (std::size_t step = 0; step < options_.max_enlargements; ++step) {
    const double margin =
      options_.initial_margin_m +
      options_.margin_step_m * static_cast<double>(step + 1);
    if (!std::isfinite(margin)) {
      break;
    }
    margins.push_back(margin);
  }
  return margins;
}

Corridor CorridorBuilder::BuildAroundCoarseRoute(
  const TerrainSnapshot & terrain, const CoarseTerrainMap & coarse,
  const std::vector<CoarseSurfaceId> & route, double margin_m) const
{
  Corridor corridor;
  corridor.margin_m = margin_m;
  if (!ValidateOptions(nullptr) || !terrain.IsUsable() ||
    !coarse.IsUsable() || route.empty())
  {
    return corridor;
  }
  if (coarse.metadata().revision != terrain.revision() ||
    coarse.fine_resolution() != terrain.resolution() ||
    coarse.origin_x() != terrain.origin_x() ||
    coarse.origin_y() != terrain.origin_y())
  {
    return corridor;
  }
  std::vector<FineCellIndex> seeds;
  for (const CoarseSurfaceId id : route) {
    const CoarseSurfaceSummary * surface = coarse.FindSurface(id);
    if (surface == nullptr) {
      continue;
    }
    for (const TerrainNodeId fine_node : surface->fine_nodes) {
      const TerrainNode * node = terrain.FindNode(fine_node);
      if (node != nullptr) {
        seeds.push_back(node->cell);
      }
    }
  }
  return BuildFromCells(terrain, seeds, margin_m);
}

Corridor CorridorBuilder::BuildAroundPoses(
  const TerrainSnapshot & terrain,
  const std::vector<Point3> & poses,
  double margin_m) const
{
  Corridor corridor;
  corridor.margin_m = margin_m;
  if (!ValidateOptions(nullptr) || !terrain.IsUsable()) {
    return corridor;
  }
  std::vector<FineCellIndex> seeds;
  seeds.reserve(poses.size());
  for (const Point3 & pose : poses) {
    if (!pose.IsFinite()) {
      continue;
    }
    const auto cell = FineCellOf(
      pose, terrain.origin_x(), terrain.origin_y(),
      terrain.resolution());
    if (cell.has_value()) {
      seeds.push_back(*cell);
    }
  }
  return BuildFromCells(terrain, seeds, margin_m);
}

Corridor CorridorBuilder::BuildAroundCells(
  const TerrainSnapshot & terrain, const std::vector<FineCellIndex> & cells,
  double margin_m) const
{
  Corridor corridor;
  corridor.margin_m = margin_m;
  if (!ValidateOptions(nullptr) || !terrain.IsUsable()) {
    return corridor;
  }
  return BuildFromCells(terrain, cells, margin_m);
}

Corridor CorridorBuilder::BuildFromCells(
  const TerrainSnapshot & terrain, const std::vector<FineCellIndex> & seeds,
  double margin_m) const
{
  Corridor corridor;
  corridor.margin_m = margin_m;
  if (seeds.empty() || !std::isfinite(margin_m) || margin_m < 0.0) {
    return corridor;
  }
  const double raw_radius = margin_m / terrain.resolution();
  if (!std::isfinite(raw_radius)) {
    corridor.truncated = true;
    return corridor;
  }
  const auto radius =
    static_cast<std::int64_t>(std::ceil(std::max(0.0, raw_radius)));
  std::unordered_set<FineCellIndex, FineCellIndexHash> unique_cells;
  unique_cells.reserve(std::min(options_.max_corridor_cells, seeds.size() * 16));
  for (const FineCellIndex & seed : seeds) {
    for (std::int64_t dx = -radius; dx <= radius; ++dx) {
      for (std::int64_t dy = -radius; dy <= radius; ++dy) {
        const FineCellIndex cell{seed.x + dx, seed.y + dy};
        if (!IsSafeFineCell(cell)) {
          continue;
        }
        if (unique_cells.size() >= options_.max_corridor_cells) {
          corridor.truncated = true;
          break;
        }
        unique_cells.insert(cell);
      }
      if (corridor.truncated) {
        break;
      }
    }
    if (corridor.truncated) {
      break;
    }
  }
  corridor.cells.assign(unique_cells.begin(), unique_cells.end());
  std::sort(corridor.cells.begin(), corridor.cells.end());
  return corridor;
}

}  // namespace dddmr_grid_global_planner
