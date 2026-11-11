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

#include "dddmr_grid_global_planner/multiresolution_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/distance_field.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr std::uint64_t kCoarseSchemaVersion = 2;

// Appends raw bytes to an FNV-1a hash.
void HashBytes(std::uint64_t * hash, const void * data, std::size_t size) noexcept
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= kFnvPrime;
  }
}

void HashUint64(std::uint64_t * hash, std::uint64_t value) noexcept
{
  HashBytes(hash, &value, sizeof(value));
}

void HashDouble(std::uint64_t * hash, double value) noexcept
{
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  HashBytes(hash, &bits, sizeof(bits));
}

// Computes `value_lo / value_hi` as an exact floor division used by fine->coarse
// cell conversion. The caller must have validated a positive ratio.
std::int64_t CoarseIndex(std::int64_t fine_index, std::int64_t ratio) noexcept
{
  return FloorDivide(fine_index, ratio);
}

// Returns the number of fine cells covered by one coarse cell: ratio * ratio,
// saturated to the size_t maximum so an extreme ratio cannot overflow. A coarse
// cell with fewer observed fine cells than this has unobserved members.
std::size_t ExpectedFineCellCount(std::int64_t ratio) noexcept
{
  if (ratio <= 0) {
    return 0;
  }
  const auto value = static_cast<std::uint64_t>(ratio);
  const auto maximum =
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max());
  if (value > maximum / value) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(value * value);
}

struct DisjointSet
{
  explicit DisjointSet(std::size_t size)
  : parent(size), rank(size, std::uint8_t{0})
  {
    std::iota(parent.begin(), parent.end(), std::size_t{0});
  }

  std::size_t Find(std::size_t value)
  {
    while (parent[value] != value) {
      parent[value] = parent[parent[value]];
      value = parent[value];
    }
    return value;
  }

  void Union(std::size_t first, std::size_t second)
  {
    first = Find(first);
    second = Find(second);
    if (first == second) {
      return;
    }
    if (rank[first] < rank[second]) {
      std::swap(first, second);
    }
    parent[second] = first;
    if (rank[first] == rank[second]) {
      ++rank[first];
    }
  }

  std::vector<std::size_t> parent;
  std::vector<std::uint8_t> rank;
};

struct FineCellProjection
{
  CoarseCellIndex cell{};
  std::size_t fine_index{0};
};

[[noreturn]] void RejectCoarse(const std::string & detail)
{
  throw std::invalid_argument("CoarseTerrainMap: " + detail);
}

// Summarizes one coarse surface from its own member IDs. This is the only place
// that scans a surface's fine nodes, and it runs in the builder/update path, not
// in search.
CoarseObstacleSummary SummarizeSurface(
  const CoarseSurfaceSummary & surface, const ObstacleDistanceField & field)
{
  CoarseObstacleSummary summary;
  summary.surface_id = surface.id;
  summary.fine_node_count = surface.fine_nodes.size();
  bool all_known_clear = true;
  for (const TerrainNodeId fine_node : surface.fine_nodes) {
    const TerrainNode * node = field.terrain() == nullptr ?
      nullptr :
      field.terrain()->FindNode(fine_node);
    if (node == nullptr || node->status == TerrainStatus::kUnknown) {
      summary.has_unknown = true;
      all_known_clear = false;
      continue;
    }
    if (node->status == TerrainStatus::kProhibited) {
      summary.has_prohibited = true;
      all_known_clear = false;
      continue;
    }
    const ObstacleDistanceRecord * record = field.FindRecord(fine_node);
    if (record == nullptr) {
      summary.has_unknown = true;
      ++summary.missing_record_count;
      all_known_clear = false;
      continue;
    }
    const ObstacleState state = record->attributes.CombinedState();
    if (state == ObstacleState::kOccupied) {
      summary.has_occupied = true;
      summary.observed = true;
      all_known_clear = false;
      summary.min_clearance_m = 0.0;
      summary.clearance_truncated = false;
      continue;
    }
    if (state == ObstacleState::kUnknown) {
      summary.has_unknown = true;
      all_known_clear = false;
      continue;
    }
    // Clear member. A missing bounded metric cannot support a clear claim.
    if (!record->nearest_known ||
      !std::isfinite(record->nearest_observed_distance_m))
    {
      summary.has_unknown = true;
      all_known_clear = false;
      continue;
    }
    summary.observed = true;
    if (!std::isfinite(summary.min_clearance_m) ||
      record->nearest_observed_distance_m < summary.min_clearance_m)
    {
      summary.min_clearance_m = record->nearest_observed_distance_m;
      summary.clearance_truncated = record->truncated;
    }
  }
  summary.clear = all_known_clear && !summary.has_occupied &&
    !summary.has_unknown && !summary.has_prohibited &&
    summary.observed && summary.fine_node_count > 0;
  return summary;
}

// Builds the full inverse fine-node -> coarse-surface index once.
std::shared_ptr<const CoarseObstacleOverlay::FineSurfaceIndex>
BuildFineSurfaceIndex(const CoarseTerrainMap & coarse)
{
  auto index = std::make_shared<CoarseObstacleOverlay::FineSurfaceIndex>();
  index->reserve(coarse.mapping().size());
  for (const auto & entry : coarse.mapping()) {
    index->emplace(entry.first, entry.second.surface_id);
  }
  return index;
}

// Builds one full coarse obstacle overlay from a coarse map and field.
std::shared_ptr<const CoarseObstacleOverlay> BuildFullCoarseObstacleOverlay(
  const CoarseTerrainMap & coarse, const ObstacleDistanceField & field,
  std::uint64_t obstacle_layer_version)
{
  CoarseObstacleOverlay::SummaryMap summaries;
  summaries.reserve(coarse.surfaces().size());
  for (const CoarseSurfaceSummary & surface : coarse.surfaces()) {
    summaries.emplace(
      surface.id,
      std::make_shared<const CoarseObstacleSummary>(
        SummarizeSurface(surface, field)));
  }
  CoarseObstacleOverlayMetadata metadata;
  metadata.fine_revision = coarse.metadata().revision;
  metadata.fine_content_hash = coarse.metadata().fine_content_hash;
  metadata.coarse_revision = coarse.metadata().revision;
  metadata.coarse_content_hash = coarse.metadata().content_hash;
  metadata.obstacle_layer_version = obstacle_layer_version;
  return std::make_shared<const CoarseObstacleOverlay>(
    std::move(summaries), BuildFineSurfaceIndex(coarse),
    std::move(metadata));
}

}  // namespace

CoarseTerrainMap::CoarseTerrainMap(
  std::vector<CoarseSurfaceSummary> surfaces,
  std::vector<CoarseTerrainConnection> connections,
  std::unordered_map<TerrainNodeId, FineToCoarseMapping> mapping,
  CoarseTerrainMetadata metadata)
: surfaces_(std::move(surfaces)),
  connections_(std::move(connections)),
  mapping_(std::move(mapping)),
  metadata_(std::move(metadata))
{
  if (metadata_.frame_id.empty()) {
    RejectCoarse("frame_id is empty");
  }
  if (!IsValidResolution(metadata_.fine_resolution) ||
    !IsValidResolution(metadata_.coarse_resolution))
  {
    RejectCoarse("resolutions must be finite and positive");
  }
  if (metadata_.coarse_resolution <= metadata_.fine_resolution) {
    RejectCoarse("coarse resolution must exceed fine resolution");
  }
  if (!IsSafeWorldCoordinate(metadata_.origin_x) ||
    !IsSafeWorldCoordinate(metadata_.origin_y))
  {
    RejectCoarse("origin is outside the safe world bound");
  }
  const double raw_ratio =
    metadata_.coarse_resolution / metadata_.fine_resolution;
  if (!std::isfinite(raw_ratio) || raw_ratio < 2.0) {
    RejectCoarse("coarse/fine resolution ratio must be at least 2");
  }
  const auto ratio = static_cast<std::int64_t>(std::llround(raw_ratio));
  if (ratio < 2 ||
    std::abs(raw_ratio - static_cast<double>(ratio)) > 1.0e-9)
  {
    RejectCoarse("coarse resolution must be an integer fine-resolution multiple");
  }
  if (std::abs(
      static_cast<double>(ratio) * metadata_.fine_resolution -
      metadata_.coarse_resolution) > 1.0e-9)
  {
    RejectCoarse("coarse/fine resolutions are not aligned");
  }

  id_to_index_.reserve(surfaces_.size());
  for (std::size_t index = 0; index < surfaces_.size(); ++index) {
    const CoarseSurfaceSummary & surface = surfaces_[index];
    if (surface.id == kInvalidCoarseSurfaceId) {
      RejectCoarse("coarse surface ID zero is reserved");
    }
    if (!id_to_index_.emplace(surface.id, index).second) {
      RejectCoarse(
        "duplicate coarse surface ID " +
        std::to_string(surface.id));
    }
    if (!IsSafeCellIndex(surface.cell.x) ||
      !IsSafeCellIndex(surface.cell.y))
    {
      RejectCoarse("coarse surface has an unsafe cell index");
    }
    if (surface.fine_nodes.empty()) {
      RejectCoarse("coarse surface has no fine member nodes");
    }
    if (!surface.representative.IsFinite() ||
      !std::isfinite(surface.z_min) || !std::isfinite(surface.z_max) ||
      !std::isfinite(surface.z_mean) ||
      !std::isfinite(surface.min_slope_rad) ||
      !std::isfinite(surface.max_slope_rad))
    {
      RejectCoarse("coarse surface has non-finite geometry");
    }
    if (surface.z_min > surface.z_max) {
      RejectCoarse("coarse surface has z_min greater than z_max");
    }
    if (surface.local_surface_index < 0) {
      RejectCoarse("coarse surface has a negative local index");
    }
    if (surface.status == TerrainStatus::kSupported &&
      (surface.contains_unknown || surface.contains_prohibited ||
      surface.contains_missing_fine_cells))
    {
      RejectCoarse(
        "supported coarse surface retains unknown/prohibited/missing "
        "evidence");
    }
    if (surface.same_cell_no_link !=
      !surface.same_cell_no_link_surfaces.empty())
    {
      RejectCoarse(
        "same_cell_no_link flag disagrees with the relation list");
    }
    for (std::size_t relation_index = 0;
      relation_index < surface.same_cell_no_link_surfaces.size();
      ++relation_index)
    {
      const CoarseSurfaceId other =
        surface.same_cell_no_link_surfaces[relation_index];
      if (other == kInvalidCoarseSurfaceId || other == surface.id) {
        RejectCoarse("same-cell no-link relation has an invalid endpoint");
      }
      if (relation_index > 0 &&
        other <= surface.same_cell_no_link_surfaces[relation_index - 1])
      {
        RejectCoarse("same-cell no-link relations are not strictly ordered");
      }
    }
    cell_to_surfaces_[surface.cell].push_back(surface.id);
  }
  for (const CoarseSurfaceSummary & surface : surfaces_) {
    for (const CoarseSurfaceId other : surface.same_cell_no_link_surfaces) {
      if (id_to_index_.find(other) == id_to_index_.end()) {
        RejectCoarse("same-cell no-link relation references an unknown surface");
      }
    }
  }
  for (auto & entry : cell_to_surfaces_) {
    std::sort(entry.second.begin(), entry.second.end());
  }

  for (const auto & entry : mapping_) {
    const auto surface_iterator = id_to_index_.find(entry.second.surface_id);
    if (surface_iterator == id_to_index_.end()) {
      RejectCoarse("mapping references an unknown coarse surface ID");
    }
    const CoarseSurfaceSummary & surface = surfaces_[surface_iterator->second];
    if (!(surface.cell == entry.second.cell) ||
      surface.local_surface_index != entry.second.local_surface_index)
    {
      RejectCoarse("mapping cell/local index disagrees with its coarse surface");
    }
  }

  for (const CoarseTerrainConnection & connection : connections_) {
    if (connection.source == kInvalidCoarseSurfaceId ||
      connection.target == kInvalidCoarseSurfaceId ||
      connection.source == connection.target)
    {
      RejectCoarse("coarse connection has an invalid endpoint");
    }
    if (id_to_index_.find(connection.source) == id_to_index_.end() ||
      id_to_index_.find(connection.target) == id_to_index_.end())
    {
      RejectCoarse("coarse connection references an unknown surface");
    }
    if (!std::isfinite(connection.horizontal_distance) ||
      !std::isfinite(connection.height_delta) ||
      !std::isfinite(connection.cost) || connection.cost < 0.0)
    {
      RejectCoarse("coarse connection has invalid geometry/cost");
    }
  }
}

const CoarseSurfaceSummary * CoarseTerrainMap::FindSurface(
  CoarseSurfaceId id) const noexcept
{
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end()) {
    return nullptr;
  }
  return &surfaces_[iterator->second];
}

const CoarseSurfaceSummary * CoarseTerrainMap::FindSurfaceForFineNode(
  TerrainNodeId fine_node) const noexcept
{
  const auto iterator = mapping_.find(fine_node);
  if (iterator == mapping_.end()) {
    return nullptr;
  }
  return FindSurface(iterator->second.surface_id);
}

const CoarseSurfaceSummary * CoarseTerrainMap::FindSurfaceAt(
  const CoarseCellIndex & cell, int local_surface_index) const noexcept
{
  const auto iterator = cell_to_surfaces_.find(cell);
  if (iterator == cell_to_surfaces_.end()) {
    return nullptr;
  }
  for (const CoarseSurfaceId id : iterator->second) {
    const CoarseSurfaceSummary * surface = FindSurface(id);
    if (surface != nullptr &&
      surface->local_surface_index == local_surface_index)
    {
      return surface;
    }
  }
  return nullptr;
}

const std::vector<CoarseSurfaceId> * CoarseTerrainMap::FindSurfacesInCell(
  const CoarseCellIndex & cell) const noexcept
{
  const auto iterator = cell_to_surfaces_.find(cell);
  if (iterator == cell_to_surfaces_.end()) {
    return nullptr;
  }
  return &iterator->second;
}

std::int64_t CoarseTerrainMap::resolution_ratio() const noexcept
{
  if (!IsValidResolution(metadata_.fine_resolution) ||
    !IsValidResolution(metadata_.coarse_resolution))
  {
    return 0;
  }
  const double raw_ratio =
    metadata_.coarse_resolution / metadata_.fine_resolution;
  if (!std::isfinite(raw_ratio) || raw_ratio < 2.0) {
    return 0;
  }
  const auto ratio = static_cast<std::int64_t>(std::llround(raw_ratio));
  if (ratio < 2 ||
    std::abs(raw_ratio - static_cast<double>(ratio)) > 1.0e-9)
  {
    return 0;
  }
  return ratio;
}

bool CoarseTerrainMap::IsUsable() const noexcept
{
  return !surfaces_.empty() && IsValidResolution(metadata_.fine_resolution) &&
         IsValidResolution(metadata_.coarse_resolution) &&
         resolution_ratio() >= 2 &&
         IsSafeWorldCoordinate(metadata_.origin_x) &&
         IsSafeWorldCoordinate(metadata_.origin_y);
}

CoarseTerrainMapBuilder::CoarseTerrainMapBuilder(
  CoarseTerrainMapOptions options)
: options_(std::move(options)) {}

bool CoarseTerrainMapBuilder::ValidateOptions(std::string * detail) const
{
  const auto reject = [detail](const std::string & message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return false;
    };
  if (!IsValidResolution(options_.coarse_resolution)) {
    return reject("coarse_resolution must be finite and positive");
  }
  if (!std::isfinite(options_.surface_z_tolerance) ||
    options_.surface_z_tolerance <= 0.0)
  {
    return reject("surface_z_tolerance must be finite and positive");
  }
  if (options_.max_fine_nodes_per_surface == 0) {
    return reject("max_fine_nodes_per_surface must be positive");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

CoarseBuildResult CoarseTerrainMapBuilder::Build(
  const TerrainSnapshot & fine_terrain) const
{
  std::string detail;
  if (!ValidateOptions(&detail)) {
    return CoarseBuildResult{CoarseBuildStatus::kInvalidOptions, detail,
      nullptr};
  }
  if (!fine_terrain.IsUsable()) {
    return CoarseBuildResult{CoarseBuildStatus::kInvalidFineSnapshot,
      "fine terrain is not usable", nullptr};
  }
  if (options_.coarse_resolution <= fine_terrain.resolution()) {
    return CoarseBuildResult{
      CoarseBuildStatus::kInvalidOptions,
      "coarse_resolution must be strictly greater than the fine resolution",
      nullptr};
  }
  const double raw_ratio =
    options_.coarse_resolution / fine_terrain.resolution();
  const auto ratio = static_cast<std::int64_t>(std::llround(raw_ratio));
  if (!std::isfinite(raw_ratio) || ratio < 2 ||
    std::abs(raw_ratio - static_cast<double>(ratio)) > 1.0e-9)
  {
    return CoarseBuildResult{
      CoarseBuildStatus::kInvalidOptions,
      "coarse_resolution must be an integer multiple of the fine resolution",
      nullptr};
  }

  const std::vector<TerrainNode> & fine_nodes = fine_terrain.nodes();
  std::unordered_map<TerrainNodeId, std::size_t> id_to_fine_index;
  id_to_fine_index.reserve(fine_nodes.size());
  std::map<CoarseCellIndex, std::vector<std::size_t>> cell_nodes;
  // Unique observed fine cells per coarse cell. A coarse cell covers an
  // integer ratio-by-ratio fine block, so a smaller count means some expected
  // fine cells were never observed and the coarse summary must stay unknown.
  std::map<CoarseCellIndex, std::set<FineCellIndex>> coarse_fine_cells;
  for (std::size_t index = 0; index < fine_nodes.size(); ++index) {
    const TerrainNode & node = fine_nodes[index];
    id_to_fine_index.emplace(node.id, index);
    const CoarseCellIndex coarse_cell{
      CoarseIndex(node.cell.x, ratio), CoarseIndex(node.cell.y, ratio)};
    cell_nodes[coarse_cell].push_back(index);
    coarse_fine_cells[coarse_cell].insert(node.cell);
  }
  const std::size_t expected_fine_cells = ExpectedFineCellCount(ratio);

  // Index connected fine links that stay inside one coarse cell. This keeps the
  // per-cell union pass proportional to local links instead of all fine links.
  std::map<CoarseCellIndex, std::vector<const TerrainConnection *>>
  intra_cell_connections;
  for (const TerrainConnection & connection : fine_terrain.connections()) {
    if (!connection.IsConnected()) {
      continue;
    }
    const auto source_iterator = id_to_fine_index.find(connection.source);
    const auto target_iterator = id_to_fine_index.find(connection.target);
    if (source_iterator == id_to_fine_index.end() ||
      target_iterator == id_to_fine_index.end())
    {
      continue;
    }
    const TerrainNode & source = fine_nodes[source_iterator->second];
    const TerrainNode & target = fine_nodes[target_iterator->second];
    const CoarseCellIndex source_cell{CoarseIndex(source.cell.x, ratio),
      CoarseIndex(source.cell.y, ratio)};
    const CoarseCellIndex target_cell{CoarseIndex(target.cell.x, ratio),
      CoarseIndex(target.cell.y, ratio)};
    if (source_cell == target_cell) {
      intra_cell_connections[source_cell].push_back(&connection);
    }
  }

  // Union fine surfaces inside one coarse cell only when an explicit fine
  // candidate connection exists and the merged group stays inside the
  // independent Z tolerance. Grouping never uses local ordering or shared XY
  // alone, so same-cell split surfaces cannot be merged implicitly.
  std::vector<CoarseSurfaceSummary> surfaces;
  std::unordered_map<TerrainNodeId, FineToCoarseMapping> mapping;
  mapping.reserve(fine_nodes.size());
  std::map<FineCellIndex, std::vector<CoarseSurfaceId>> fine_cell_surfaces;
  CoarseSurfaceId next_surface_id = 1;
  for (const auto & cell_entry : cell_nodes) {
    const CoarseCellIndex & coarse_cell = cell_entry.first;
    const std::vector<std::size_t> & indices = cell_entry.second;
    const auto observed_cells = coarse_fine_cells.find(coarse_cell);
    const std::size_t observed_fine_cell_count =
      observed_cells == coarse_fine_cells.end() ?
      0U :
      observed_cells->second.size();
    const bool has_missing_fine_cells =
      observed_fine_cell_count < expected_fine_cells;
    std::unordered_map<std::size_t, std::size_t> local_position;
    local_position.reserve(indices.size());
    for (std::size_t local = 0; local < indices.size(); ++local) {
      local_position.emplace(indices[local], local);
    }
    DisjointSet sets(indices.size());
    std::vector<double> group_z_min(indices.size());
    std::vector<double> group_z_max(indices.size());
    for (std::size_t local = 0; local < indices.size(); ++local) {
      const double z = fine_nodes[indices[local]].representative.z;
      group_z_min[local] = z;
      group_z_max[local] = z;
    }
    const auto local_connections = intra_cell_connections.find(coarse_cell);
    if (local_connections != intra_cell_connections.end()) {
      // Sort by endpoint IDs so chained tolerance decisions are independent of
      // the snapshot's connection storage order.
      std::vector<const TerrainConnection *> ordered_connections =
        local_connections->second;
      std::sort(
        ordered_connections.begin(), ordered_connections.end(),
        [](const TerrainConnection * lhs, const TerrainConnection * rhs) {
          if (lhs->source != rhs->source) {
            return lhs->source < rhs->source;
          }
          return lhs->target < rhs->target;
        });
      for (const TerrainConnection * connection : ordered_connections) {
        const auto source_position = local_position.find(
          id_to_fine_index.at(connection->source));
        const auto target_position = local_position.find(
          id_to_fine_index.at(connection->target));
        if (source_position == local_position.end() ||
          target_position == local_position.end())
        {
          continue;
        }
        const std::size_t first_root = sets.Find(source_position->second);
        const std::size_t second_root = sets.Find(target_position->second);
        if (first_root == second_root) {
          continue;
        }
        const double merged_min =
          std::min(group_z_min[first_root], group_z_min[second_root]);
        const double merged_max =
          std::max(group_z_max[first_root], group_z_max[second_root]);
        if (merged_max - merged_min > options_.surface_z_tolerance) {
          continue;
        }
        sets.Union(first_root, second_root);
        const std::size_t merged_root = sets.Find(first_root);
        group_z_min[merged_root] = merged_min;
        group_z_max[merged_root] = merged_max;
      }
    }

    std::map<std::size_t, std::vector<std::size_t>> groups;
    for (std::size_t local = 0; local < indices.size(); ++local) {
      groups[sets.Find(local)].push_back(indices[local]);
    }
    std::vector<std::vector<std::size_t>> ordered_groups;
    ordered_groups.reserve(groups.size());
    for (auto & group : groups) {
      std::sort(
        group.second.begin(), group.second.end(),
        [&fine_nodes](std::size_t lhs, std::size_t rhs) {
          return fine_nodes[lhs].id < fine_nodes[rhs].id;
        });
      ordered_groups.push_back(std::move(group.second));
    }
    std::sort(
      ordered_groups.begin(), ordered_groups.end(),
      [&fine_nodes](const std::vector<std::size_t> & lhs,
      const std::vector<std::size_t> & rhs) {
        const double lhs_z = fine_nodes[lhs.front()].representative.z;
        const double rhs_z = fine_nodes[rhs.front()].representative.z;
        if (lhs_z != rhs_z) {
          return lhs_z < rhs_z;
        }
        return fine_nodes[lhs.front()].id < fine_nodes[rhs.front()].id;
      });

    for (std::size_t local_index = 0; local_index < ordered_groups.size();
      ++local_index)
    {
      const std::vector<std::size_t> & group = ordered_groups[local_index];
      CoarseSurfaceSummary summary;
      summary.id = next_surface_id++;
      summary.cell = coarse_cell;
      summary.local_surface_index = static_cast<int>(local_index);
      summary.fine_nodes.reserve(group.size());
      std::set<FineCellIndex> group_fine_cells;
      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_z = 0.0;
      double z_min = std::numeric_limits<double>::infinity();
      double z_max = -std::numeric_limits<double>::infinity();
      double min_slope = std::numeric_limits<double>::infinity();
      double max_slope = 0.0;
      bool all_supported = true;
      bool has_prohibited = false;
      bool has_unknown = false;
      ProhibitedReason first_prohibited = ProhibitedReason::kNone;
      ProhibitedReason first_unknown = ProhibitedReason::kInsufficientSamples;
      for (const std::size_t fine_index : group) {
        const TerrainNode & node = fine_nodes[fine_index];
        summary.fine_nodes.push_back(node.id);
        group_fine_cells.insert(node.cell);
        sum_x += node.representative.x;
        sum_y += node.representative.y;
        sum_z += node.representative.z;
        // Retain the raw support envelope of each member instead of averaging
        // it into the representative height. This keeps a high/dangerous sliver
        // visible in the coarse Z range.
        const double fine_z_min = std::isfinite(node.support.z_min) ?
          node.support.z_min :
          node.representative.z;
        const double fine_z_max = std::isfinite(node.support.z_max) ?
          node.support.z_max :
          node.representative.z;
        z_min = std::min(z_min, fine_z_min);
        z_max = std::max(z_max, fine_z_max);
        const double normal_z =
          std::max(-1.0, std::min(1.0, node.plane.normal.z));
        const double slope =
          node.plane.valid ? std::acos(normal_z) : kPi * 0.5;
        min_slope = std::min(min_slope, slope);
        max_slope = std::max(max_slope, slope);
        if (node.status == TerrainStatus::kProhibited) {
          has_prohibited = true;
          all_supported = false;
          if (first_prohibited == ProhibitedReason::kNone) {
            first_prohibited = node.reason;
          }
        } else if (node.status != TerrainStatus::kSupported) {
          has_unknown = true;
          all_supported = false;
          if (first_unknown == ProhibitedReason::kInsufficientSamples) {
            first_unknown = node.reason;
          }
        }
      }
      const double count = static_cast<double>(group.size());
      summary.representative =
        Point3{sum_x / count, sum_y / count, sum_z / count};
      summary.z_min = z_min;
      summary.z_max = z_max;
      summary.z_mean = sum_z / count;
      summary.min_slope_rad = std::isfinite(min_slope) ? min_slope : 0.0;
      summary.max_slope_rad = max_slope;
      summary.contains_missing_fine_cells = has_missing_fine_cells;
      summary.contains_unknown = has_unknown || has_missing_fine_cells;
      summary.contains_prohibited = has_prohibited;
      if (has_prohibited) {
        summary.status = TerrainStatus::kProhibited;
        summary.reason = first_prohibited;
      } else if (has_unknown || has_missing_fine_cells || !all_supported) {
        summary.status = TerrainStatus::kUnknown;
        summary.reason = has_unknown ? first_unknown :
          ProhibitedReason::kInsufficientSamples;
      } else {
        summary.status = TerrainStatus::kSupported;
        summary.reason = ProhibitedReason::kNone;
      }
      if (group.size() > options_.max_fine_nodes_per_surface) {
        summary.status = TerrainStatus::kProhibited;
        summary.reason = ProhibitedReason::kExcessiveLayers;
        summary.contains_prohibited = true;
      }
      for (const TerrainNodeId fine_node : summary.fine_nodes) {
        mapping.emplace(
          fine_node,
          FineToCoarseMapping{coarse_cell, summary.id,
            summary.local_surface_index});
      }
      for (const FineCellIndex & fine_cell : group_fine_cells) {
        fine_cell_surfaces[fine_cell].push_back(summary.id);
      }
      surfaces.push_back(std::move(summary));
    }
  }

  // Derive coarse edges from connected fine candidate links. Only supported
  // endpoints are linked; unknown/prohibited summaries remain isolated.
  std::unordered_map<CoarseSurfaceId, std::size_t> summary_index;
  summary_index.reserve(surfaces.size());
  for (std::size_t index = 0; index < surfaces.size(); ++index) {
    summary_index.emplace(surfaces[index].id, index);
  }

  // Resolve explicit same-fine-cell no-link relations. Surfaces from the same
  // fine cell were split as distinct support models and have no fine candidate
  // link, so they are never merged by the union pass above. Recording the
  // relation keeps that kSameCellNoLink contract visible to coarse consumers.
  for (const auto & entry : fine_cell_surfaces) {
    const std::vector<CoarseSurfaceId> & ids = entry.second;
    if (ids.size() < 2U) {
      continue;
    }
    for (const CoarseSurfaceId id : ids) {
      const auto surface_iterator = summary_index.find(id);
      if (surface_iterator == summary_index.end()) {
        continue;
      }
      std::vector<CoarseSurfaceId> & relations =
        surfaces[surface_iterator->second].same_cell_no_link_surfaces;
      for (const CoarseSurfaceId other : ids) {
        if (other != id) {
          relations.push_back(other);
        }
      }
    }
  }
  for (CoarseSurfaceSummary & surface : surfaces) {
    std::vector<CoarseSurfaceId> & relations =
      surface.same_cell_no_link_surfaces;
    std::sort(relations.begin(), relations.end());
    relations.erase(
      std::unique(relations.begin(), relations.end()), relations.end());
    surface.same_cell_no_link = !relations.empty();
  }

  std::map<std::pair<CoarseSurfaceId, CoarseSurfaceId>,
    CoarseTerrainConnection>
  edge_map;
  for (const TerrainConnection & connection : fine_terrain.connections()) {
    if (!connection.IsConnected()) {
      continue;
    }
    const auto source_mapping = mapping.find(connection.source);
    const auto target_mapping = mapping.find(connection.target);
    if (source_mapping == mapping.end() ||
      target_mapping == mapping.end() ||
      source_mapping->second.surface_id == target_mapping->second.surface_id)
    {
      continue;
    }
    const CoarseSurfaceId first =
      std::min(
      source_mapping->second.surface_id,
      target_mapping->second.surface_id);
    const CoarseSurfaceId second =
      std::max(
      source_mapping->second.surface_id,
      target_mapping->second.surface_id);
    const auto first_index = summary_index.find(first);
    const auto second_index = summary_index.find(second);
    if (first_index == summary_index.end() ||
      second_index == summary_index.end())
    {
      continue;
    }
    const CoarseSurfaceSummary & first_summary = surfaces[first_index->second];
    const CoarseSurfaceSummary & second_summary = surfaces[second_index->second];
    if (!first_summary.IsSupported() || !second_summary.IsSupported()) {
      continue;
    }
    CoarseTerrainConnection edge;
    edge.source = first;
    edge.target = second;
    const double dx =
      second_summary.representative.x - first_summary.representative.x;
    const double dy =
      second_summary.representative.y - first_summary.representative.y;
    edge.horizontal_distance = std::hypot(dx, dy);
    edge.height_delta = second_summary.representative.z -
      first_summary.representative.z;
    edge.cost = std::sqrt(
      edge.horizontal_distance * edge.horizontal_distance +
      edge.height_delta * edge.height_delta);
    const auto key = std::make_pair(first, second);
    const auto existing = edge_map.find(key);
    if (existing == edge_map.end() || edge.cost < existing->second.cost) {
      edge_map[key] = edge;
    }
  }
  std::vector<CoarseTerrainConnection> connections;
  connections.reserve(edge_map.size());
  for (auto & entry : edge_map) {
    connections.push_back(std::move(entry.second));
  }

  CoarseTerrainMetadata metadata;
  metadata.frame_id = fine_terrain.frame_id();
  metadata.fine_resolution = fine_terrain.resolution();
  metadata.coarse_resolution = options_.coarse_resolution;
  metadata.origin_x = fine_terrain.origin_x();
  metadata.origin_y = fine_terrain.origin_y();
  metadata.revision = fine_terrain.revision();
  metadata.fine_content_hash = fine_terrain.content_hash();
  std::uint64_t hash = kFnvOffset;
  HashUint64(&hash, kCoarseSchemaVersion);
  HashUint64(&hash, metadata.fine_content_hash);
  HashDouble(&hash, metadata.fine_resolution);
  HashDouble(&hash, metadata.coarse_resolution);
  HashDouble(&hash, metadata.origin_x);
  HashDouble(&hash, metadata.origin_y);
  HashDouble(&hash, options_.surface_z_tolerance);
  for (const CoarseSurfaceSummary & surface : surfaces) {
    HashUint64(&hash, surface.id);
    HashUint64(&hash, static_cast<std::uint64_t>(surface.cell.x));
    HashUint64(&hash, static_cast<std::uint64_t>(surface.cell.y));
    HashUint64(&hash, static_cast<std::uint64_t>(surface.local_surface_index));
    HashDouble(&hash, surface.z_min);
    HashDouble(&hash, surface.z_max);
    HashDouble(&hash, surface.z_mean);
    HashUint64(&hash, static_cast<std::uint64_t>(surface.status));
    HashUint64(&hash, static_cast<std::uint64_t>(surface.reason));
    HashUint64(&hash, surface.contains_unknown ? 1U : 0U);
    HashUint64(&hash, surface.contains_prohibited ? 1U : 0U);
    HashUint64(&hash, surface.contains_missing_fine_cells ? 1U : 0U);
    for (const CoarseSurfaceId other : surface.same_cell_no_link_surfaces) {
      HashUint64(&hash, other);
    }
    for (const TerrainNodeId fine_node : surface.fine_nodes) {
      HashUint64(&hash, fine_node);
    }
  }
  for (const CoarseTerrainConnection & connection : connections) {
    HashUint64(&hash, connection.source);
    HashUint64(&hash, connection.target);
    HashDouble(&hash, connection.cost);
  }
  metadata.content_hash = hash;

  try {
    auto map = std::make_shared<const CoarseTerrainMap>(
      std::move(surfaces), std::move(connections), std::move(mapping),
      std::move(metadata));
    return CoarseBuildResult{CoarseBuildStatus::kOk, "", std::move(map)};
  } catch (const std::invalid_argument & error) {
    return CoarseBuildResult{CoarseBuildStatus::kInvalidFineSnapshot,
      error.what(), nullptr};
  }
}

std::shared_ptr<const CoarseTerrainMap> OverlayCoarseTerrain(
  std::shared_ptr<const CoarseTerrainMap> previous,
  std::shared_ptr<const TerrainSnapshot> fine,
  const std::vector<CoarseCellIndex> & changed_cells,
  const CoarseTerrainMapOptions & options)
{
  if (fine == nullptr) {
    return nullptr;
  }
  // `changed_cells` routes obstacle-dirty coarse cells. Coarse terrain geometry
  // is independent of obstacle evidence, so those cells never force a terrain
  // re-cluster; validate them here instead of silently dropping the request. The
  // obstacle summaries themselves are maintained by OverlayCoarseObstacles.
  for (const CoarseCellIndex & cell : changed_cells) {
    if (!IsSafeCellIndex(cell.x) || !IsSafeCellIndex(cell.y)) {
      throw std::invalid_argument(
              "OverlayCoarseTerrain: obstacle-dirty coarse cell is out of bounds");
    }
  }
  if (previous != nullptr &&
    previous->metadata().revision == fine->revision() &&
    previous->metadata().fine_content_hash == fine->content_hash())
  {
    return previous;
  }
  CoarseTerrainMapBuilder builder(options);
  const CoarseBuildResult result = builder.Build(*fine);
  if (!result.ok()) {
    return nullptr;
  }
  return result.map;
}

CoarseObstacleOverlay::CoarseObstacleOverlay(
  SummaryMap surface_summaries,
  std::shared_ptr<const FineSurfaceIndex> fine_surface_index,
  CoarseObstacleOverlayMetadata metadata)
: surface_summaries_(std::move(surface_summaries)),
  fine_surface_index_(std::move(fine_surface_index)),
  metadata_(std::move(metadata))
{
  if (fine_surface_index_ == nullptr) {
    throw std::invalid_argument(
            "CoarseObstacleOverlay: inverse fine-node index is null");
  }
  for (const auto & entry : surface_summaries_) {
    if (entry.second == nullptr) {
      throw std::invalid_argument("CoarseObstacleOverlay: null summary");
    }
    if (entry.first == kInvalidCoarseSurfaceId ||
      entry.second->surface_id != entry.first)
    {
      throw std::invalid_argument(
              "CoarseObstacleOverlay: summary surface ID mismatch");
    }
    if (std::isfinite(entry.second->min_clearance_m) &&
      entry.second->min_clearance_m < 0.0)
    {
      throw std::invalid_argument(
              "CoarseObstacleOverlay: negative minimum clearance");
    }
    if (entry.second->clear &&
      (entry.second->has_occupied || entry.second->has_unknown ||
      entry.second->has_prohibited || !entry.second->observed))
    {
      throw std::invalid_argument(
              "CoarseObstacleOverlay: clear summary retains adverse evidence");
    }
  }
}

const CoarseObstacleSummary * CoarseObstacleOverlay::FindSummary(
  CoarseSurfaceId surface_id) const noexcept
{
  const auto iterator = surface_summaries_.find(surface_id);
  if (iterator == surface_summaries_.end() || iterator->second == nullptr) {
    return nullptr;
  }
  return iterator->second.get();
}

const CoarseSurfaceId * CoarseObstacleOverlay::FindSurfaceForFineNode(
  TerrainNodeId fine_node) const noexcept
{
  if (fine_surface_index_ == nullptr) {
    return nullptr;
  }
  const auto iterator = fine_surface_index_->find(fine_node);
  if (iterator == fine_surface_index_->end()) {
    return nullptr;
  }
  return &iterator->second;
}

std::shared_ptr<const CoarseObstacleOverlay> OverlayCoarseObstacles(
  std::shared_ptr<const CoarseObstacleOverlay> previous,
  const CoarseTerrainMap & coarse, const ObstacleDistanceField & field,
  const std::vector<TerrainNodeId> & changed_fine_ids,
  std::uint64_t obstacle_layer_version)
{
  // Never mix revisions: the field must describe the same fine snapshot the
  // coarse map was built from. A mismatch is a caller bug, not a silent rebuild.
  if (field.terrain() == nullptr ||
    field.terrain()->revision() != coarse.metadata().revision ||
    field.terrain()->content_hash() != coarse.metadata().fine_content_hash)
  {
    throw std::invalid_argument(
            "OverlayCoarseObstacles: distance field terrain does not match the "
            "coarse map revision");
  }
  const bool terrain_metadata_matches =
    previous != nullptr &&
    previous->fine_revision() == coarse.metadata().revision &&
    previous->fine_content_hash() == coarse.metadata().fine_content_hash &&
    previous->coarse_revision() == coarse.metadata().revision &&
    previous->coarse_content_hash() == coarse.metadata().content_hash;
  if (!terrain_metadata_matches) {
    return BuildFullCoarseObstacleOverlay(
      coarse, field,
      obstacle_layer_version);
  }
  if (changed_fine_ids.empty()) {
    // No obstacle content changed. Reuse the previous overlay only when its
    // layer version is still current; otherwise a version-only rebuild is
    // required so the published metadata can never lag the layer.
    return previous->obstacle_layer_version() == obstacle_layer_version ?
           previous :
           BuildFullCoarseObstacleOverlay(
      coarse, field,
      obstacle_layer_version);
  }

  // Route changed fine IDs through the shared inverse index. Only their coarse
  // surfaces are recomputed; every other summary stays pointer-identical. The
  // obstacle layer version advances on every update, so it does not force a
  // rebuild while changed IDs identify exactly the affected surfaces.
  std::set<CoarseSurfaceId> touched_surfaces;
  for (const TerrainNodeId fine_node : changed_fine_ids) {
    const CoarseSurfaceId * surface_id =
      previous->FindSurfaceForFineNode(fine_node);
    if (surface_id != nullptr) {
      touched_surfaces.insert(*surface_id);
    }
  }
  if (touched_surfaces.empty()) {
    if (previous->obstacle_layer_version() == obstacle_layer_version) {
      return previous;
    }
    CoarseObstacleOverlayMetadata metadata = previous->metadata();
    metadata.obstacle_layer_version = obstacle_layer_version;
    return std::make_shared<const CoarseObstacleOverlay>(
      previous->surface_summaries(), previous->fine_surface_index(),
      std::move(metadata));
  }
  CoarseObstacleOverlay::SummaryMap summaries = previous->surface_summaries();
  for (const CoarseSurfaceId surface_id : touched_surfaces) {
    const CoarseSurfaceSummary * surface = coarse.FindSurface(surface_id);
    if (surface == nullptr) {
      // The previous index referenced a surface that no longer exists; rebuild
      // the whole overlay rather than publish a mixed revision.
      return BuildFullCoarseObstacleOverlay(
        coarse, field,
        obstacle_layer_version);
    }
    summaries[surface_id] = std::make_shared<const CoarseObstacleSummary>(
      SummarizeSurface(*surface, field));
  }
  CoarseObstacleOverlayMetadata metadata = previous->metadata();
  metadata.obstacle_layer_version = obstacle_layer_version;
  return std::make_shared<const CoarseObstacleOverlay>(
    std::move(summaries), previous->fine_surface_index(),
    std::move(metadata));
}

const char * CoarseBuildStatusToString(CoarseBuildStatus status) noexcept
{
  switch (status) {
    case CoarseBuildStatus::kOk:
      return "ok";
    case CoarseBuildStatus::kInvalidOptions:
      return "invalid_options";
    case CoarseBuildStatus::kInvalidFineSnapshot:
      return "invalid_fine_snapshot";
  }
  return "unknown";
}

}  // namespace dddmr_grid_global_planner
