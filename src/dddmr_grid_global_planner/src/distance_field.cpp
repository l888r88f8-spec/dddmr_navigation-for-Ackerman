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

#include "dddmr_grid_global_planner/distance_field.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kDistanceEpsilon = 1.0e-12;

// Returns the finite minimum, preferring the known value when only one exists.
double KnownMinimum(double first, double second) noexcept
{
  const bool first_known = std::isfinite(first);
  const bool second_known = std::isfinite(second);
  if (first_known && second_known) {
    return std::min(first, second);
  }
  if (first_known) {
    return first;
  }
  if (second_known) {
    return second;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// Merges one node from two immutable layers. Any unknown layer remains unknown.
ObstacleAttributes MergeLayers(
  const ObstacleLayer & static_layer,
  const ObstacleLayer & dynamic_layer,
  TerrainNodeId id)
{
  ObstacleAttributes merged;
  const ObstacleAttributes * static_attributes = static_layer.Find(id);
  const ObstacleAttributes * dynamic_attributes = dynamic_layer.Find(id);
  if (static_attributes != nullptr) {
    merged.static_state = static_attributes->static_state;
    merged.static_distance_m = static_attributes->static_distance_m;
    merged.clearance_above_m = static_attributes->clearance_above_m;
    merged.clearance_known = static_attributes->clearance_known;
    merged.occupied_vertical_extent_known =
      static_attributes->occupied_vertical_extent_known;
    merged.occupied_min_z = static_attributes->occupied_min_z;
    merged.occupied_max_z = static_attributes->occupied_max_z;
    merged.observed_vertical_extent_known =
      static_attributes->observed_vertical_extent_known;
    merged.observed_min_z = static_attributes->observed_min_z;
    merged.observed_max_z = static_attributes->observed_max_z;
    merged.observation_stamp_ns = static_attributes->observation_stamp_ns;
    merged.received_steady_ns = static_attributes->received_steady_ns;
  }
  if (dynamic_attributes != nullptr) {
    merged.dynamic_state = dynamic_attributes->dynamic_state;
    merged.dynamic_distance_m = dynamic_attributes->dynamic_distance_m;
    if (dynamic_attributes->clearance_known) {
      merged.clearance_above_m =
        KnownMinimum(
        merged.clearance_above_m,
        dynamic_attributes->clearance_above_m);
      merged.clearance_known = true;
    }
    if (dynamic_attributes->occupied_vertical_extent_known) {
      if (merged.occupied_vertical_extent_known) {
        merged.occupied_min_z = std::min(
          merged.occupied_min_z, dynamic_attributes->occupied_min_z);
        merged.occupied_max_z = std::max(
          merged.occupied_max_z, dynamic_attributes->occupied_max_z);
      } else {
        merged.occupied_vertical_extent_known = true;
        merged.occupied_min_z = dynamic_attributes->occupied_min_z;
        merged.occupied_max_z = dynamic_attributes->occupied_max_z;
      }
    }
    if (dynamic_attributes->observed_vertical_extent_known) {
      if (merged.observed_vertical_extent_known) {
        const double intersection_min = std::max(
          merged.observed_min_z, dynamic_attributes->observed_min_z);
        const double intersection_max = std::min(
          merged.observed_max_z, dynamic_attributes->observed_max_z);
        if (intersection_min <= intersection_max) {
          merged.observed_min_z = intersection_min;
          merged.observed_max_z = intersection_max;
        } else {
          merged.observed_vertical_extent_known = false;
          merged.observed_min_z = std::numeric_limits<double>::quiet_NaN();
          merged.observed_max_z = std::numeric_limits<double>::quiet_NaN();
        }
      } else {
        merged.observed_vertical_extent_known = true;
        merged.observed_min_z = dynamic_attributes->observed_min_z;
        merged.observed_max_z = dynamic_attributes->observed_max_z;
      }
    }
    merged.observation_stamp_ns =
      std::max(
      merged.observation_stamp_ns,
      dynamic_attributes->observation_stamp_ns);
    merged.received_steady_ns =
      std::max(
      merged.received_steady_ns,
      dynamic_attributes->received_steady_ns);
  }
  return merged;
}

// True when two option sets describe the same bounded metric policy.
bool SameOptions(
  const ObstacleDistanceOptions & first,
  const ObstacleDistanceOptions & second) noexcept
{
  return first.truncation_distance_m == second.truncation_distance_m &&
         first.reject_unknown == second.reject_unknown &&
         first.reject_prohibited == second.reject_prohibited;
}

// A supported clear node with no occupied node inside the truncation radius is
// reported as the explicit lower bound R. The metric never claims that an
// obstacle exists at exactly R; IsTruncated() records the lower-bound nature.
bool IsExactDistance(
  double nearest_distance, bool nearest_reached,
  double truncation_distance) noexcept
{
  return nearest_reached && std::isfinite(nearest_distance) &&
         nearest_distance + kDistanceEpsilon < truncation_distance;
}

// Converts terrain/obstacle/observability evidence into the explicit bounded
// record contract shared by production and reference code. Unknown influence is
// a local radius result, not a whole-component flood. A clear node is usable
// when its reported bounded clearance is not contradicted by an unknown
// boundary inside that same radius.
ObstacleDistanceRecord MakeRecord(
  const TerrainNode & node, const ObstacleAttributes & attributes,
  double nearest_distance, bool nearest_reached,
  double unknown_boundary_distance, bool unknown_boundary,
  bool adjacent_unknown, double truncation_distance)
{
  ObstacleDistanceRecord record;
  record.attributes = attributes;
  const double kUnsetDistance = std::numeric_limits<double>::quiet_NaN();
  record.distance_m = kUnsetDistance;
  record.distance_known = false;
  record.nearest_observed_distance_m = kUnsetDistance;
  record.nearest_known = false;
  record.truncated = false;
  record.clearance_guaranteed = false;
  record.adjacent_unknown = adjacent_unknown;
  record.unknown_boundary = unknown_boundary;
  record.unknown_boundary_distance_m =
    std::isfinite(unknown_boundary_distance) ?
    std::min(unknown_boundary_distance, truncation_distance) :
    kInfinity;
  record.local_unknown_influence =
    record.unknown_boundary_distance_m <=
    truncation_distance + kDistanceEpsilon;

  // Unknown or absent terrain and unknown obstacle attributes cannot provide a
  // bounded metric.
  if (!node.IsSupported()) {
    return record;
  }
  const ObstacleState state = attributes.CombinedState();
  if (state == ObstacleState::kUnknown) {
    return record;
  }
  if (state == ObstacleState::kOccupied) {
    record.distance_m = 0.0;
    record.distance_known = true;
    record.nearest_observed_distance_m = 0.0;
    record.nearest_known = true;
    return record;
  }

  // Clear supported node. The reported value is min(distance, truncation). An
  // exact value below R is a measured distance; otherwise R is an explicit
  // lower bound derived from the local search radius.
  const bool exact =
    IsExactDistance(nearest_distance, nearest_reached, truncation_distance);
  const double reported =
    exact ? std::max(0.0, nearest_distance) : truncation_distance;
  record.distance_m = reported;
  record.distance_known = true;
  record.nearest_observed_distance_m = reported;
  record.nearest_known = true;
  record.truncated = !exact;
  // Any unknown boundary inside the truncation radius can hide a closer
  // obstacle, so it removes the local clearance guarantee. A distant truncated
  // obstacle beyond the radius never removes local clear evidence.
  record.clearance_guaranteed = !record.local_unknown_influence;
  return record;
}

// True when one of the eight neighbor cells inside the observed extent is
// missing. A missing interior cell is unobserved evidence even when the terrain
// builder emitted no node/connection for it.
bool MissingInteriorNeighbor(
  const TerrainSnapshot & terrain,
  const TerrainNode & node,
  std::int64_t min_cell_x,
  std::int64_t max_cell_x,
  std::int64_t min_cell_y,
  std::int64_t max_cell_y) noexcept
{
  for (std::int64_t dx = -1; dx <= 1; ++dx) {
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const std::int64_t nx = node.cell.x + dx;
      const std::int64_t ny = node.cell.y + dy;
      if (nx < min_cell_x || nx > max_cell_x || ny < min_cell_y ||
        ny > max_cell_y)
      {
        continue;
      }
      if (terrain.FindNodesInCell(FineCellIndex{nx, ny}) == nullptr) {
        return true;
      }
    }
  }
  return false;
}

// Converts one candidate connection into a finite positive edge weight. Invalid
// evidence falls back to the fine resolution instead of producing NaN/inf.
double EdgeWeight(const TerrainConnection & connection, double resolution)
{
  double weight = resolution;
  if (std::isfinite(connection.horizontal_distance)) {
    weight = std::max(1.0e-9, std::abs(connection.horizontal_distance));
  }
  if (!std::isfinite(weight) || weight <= 0.0) {
    weight = resolution;
  }
  return weight;
}

// Bounded multi-source Dijkstra over the connected terrain edges. Seeds start at
// zero and no candidate beyond `truncation_distance` is stored. This is the
// only propagation used by the production metric; it never floods a whole
// connected component.
std::vector<double> BoundedMultiSourceDijkstra(
  const std::vector<std::vector<std::pair<std::size_t, double>>> & adjacency,
  const std::vector<std::size_t> & seeds, double truncation_distance)
{
  std::vector<double> distances(adjacency.size(), kInfinity);
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  open;
  for (const std::size_t seed : seeds) {
    if (seed >= distances.size()) {
      continue;
    }
    distances[seed] = 0.0;
    open.emplace(0.0, seed);
  }
  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    if (entry.first > distances[entry.second] + kDistanceEpsilon) {
      continue;
    }
    if (entry.first > truncation_distance + kDistanceEpsilon) {
      continue;
    }
    for (const auto & edge : adjacency[entry.second]) {
      const double candidate = entry.first + edge.second;
      if (!std::isfinite(candidate) ||
        candidate > truncation_distance + kDistanceEpsilon)
      {
        continue;
      }
      if (candidate + kDistanceEpsilon < distances[edge.first]) {
        distances[edge.first] = candidate;
        open.emplace(candidate, edge.first);
      }
    }
  }
  return distances;
}

}  // namespace

// Builds the immutable bounded metric min(nearest observed occupied distance,
// truncation), local unknown-influence radius, and clearance records once. Every
// connected-component-wide flood was removed; the only propagation is bounded
// by truncation_distance_m.
ObstacleDistanceField::ObstacleDistanceField(
  std::shared_ptr<const TerrainSnapshot> terrain,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  ObstacleDistanceOptions options)
: terrain_(std::move(terrain)),
  static_obstacles_(std::move(static_obstacles)),
  dynamic_obstacles_(std::move(dynamic_obstacles)),
  options_(options)
{
  if (terrain_ == nullptr || !terrain_->IsUsable()) {
    validation_detail_ = "terrain snapshot is null or unusable";
    return;
  }
  if (!std::isfinite(options_.truncation_distance_m) ||
    options_.truncation_distance_m <= 0.0)
  {
    validation_detail_ =
      "truncation_distance_m must be finite and positive";
    return;
  }

  const std::vector<TerrainNode> & nodes = terrain_->nodes();
  const std::size_t node_count = nodes.size();
  auto id_to_index = std::make_shared<
    std::unordered_map<TerrainNodeId, std::size_t>>();
  id_to_index->reserve(node_count);
  for (std::size_t index = 0; index < node_count; ++index) {
    id_to_index->emplace(nodes[index].id, index);
  }
  id_to_index_ = id_to_index;

  // Build the same-surface adjacency once from exactly the connected candidate
  // edges. Unconnected candidates never contribute a distance shortcut.
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(
    node_count);
  auto id_adjacency = std::make_shared<std::unordered_map<
        TerrainNodeId, std::vector<std::pair<TerrainNodeId, double>>>>();
  id_adjacency->reserve(node_count);
  for (const TerrainConnection & connection : terrain_->connections()) {
    if (!connection.IsConnected()) {
      continue;
    }
    const auto source_iterator = id_to_index->find(connection.source);
    const auto target_iterator = id_to_index->find(connection.target);
    if (source_iterator == id_to_index->end() ||
      target_iterator == id_to_index->end())
    {
      continue;
    }
    const double weight = EdgeWeight(connection, terrain_->resolution());
    if (!std::isfinite(weight) || weight <= 0.0) {
      continue;
    }
    adjacency[source_iterator->second].emplace_back(
      target_iterator->second,
      weight);
    adjacency[target_iterator->second].emplace_back(
      source_iterator->second,
      weight);
    (*id_adjacency)[connection.source].emplace_back(connection.target, weight);
    (*id_adjacency)[connection.target].emplace_back(connection.source, weight);
  }
  adjacency_ = id_adjacency;

  // Observed bounding box. A cell with no node inside this box is an interior
  // unknown/hole; cells beyond the map extent are also unobserved but are not
  // treated as interior holes here.
  min_cell_x_ = nodes.empty() ? 0 : nodes.front().cell.x;
  max_cell_x_ = min_cell_x_;
  min_cell_y_ = nodes.empty() ? 0 : nodes.front().cell.y;
  max_cell_y_ = min_cell_y_;
  for (const TerrainNode & node : nodes) {
    min_cell_x_ = std::min(min_cell_x_, node.cell.x);
    max_cell_x_ = std::max(max_cell_x_, node.cell.x);
    min_cell_y_ = std::min(min_cell_y_, node.cell.y);
    max_cell_y_ = std::max(max_cell_y_, node.cell.y);
  }

  std::vector<ObstacleAttributes> merged_attributes(node_count);
  std::vector<ObstacleState> states(node_count, ObstacleState::kUnknown);
  std::vector<bool> occupied(node_count, false);
  std::vector<bool> unknown_boundary(node_count, false);
  for (std::size_t index = 0; index < node_count; ++index) {
    merged_attributes[index] = MergeLayers(
      static_obstacles_, dynamic_obstacles_, nodes[index].id);
    states[index] = merged_attributes[index].CombinedState();
    occupied[index] = states[index] == ObstacleState::kOccupied;
    // Unknown terrain, unknown obstacle attributes, or an interior missing cell
    // can hide a closer obstacle. These are the bounded unknown-influence seeds.
    unknown_boundary[index] =
      nodes[index].status == TerrainStatus::kUnknown ||
      states[index] == ObstacleState::kUnknown ||
      MissingInteriorNeighbor(
      *terrain_, nodes[index], min_cell_x_,
      max_cell_x_, min_cell_y_, max_cell_y_);
  }

  // Record the one-cell diagnostic independently of the bounded radius result.
  std::vector<bool> adjacent_unknown(node_count, false);
  for (std::size_t index = 0; index < node_count; ++index) {
    // The node's own unknown-boundary evidence (including a missing interior
    // cell) is a one-cell diagnostic even when the node itself is clear.
    adjacent_unknown[index] = unknown_boundary[index];
    for (const auto & edge : adjacency[index]) {
      const std::size_t neighbor = edge.first;
      if (nodes[neighbor].status == TerrainStatus::kUnknown ||
        states[neighbor] == ObstacleState::kUnknown)
      {
        adjacent_unknown[index] = true;
        break;
      }
    }
  }

  // Bounded unknown-influence Dijkstra. No connected-component flood exists.
  std::vector<std::size_t> unknown_seeds;
  unknown_seeds.reserve(node_count);
  for (std::size_t index = 0; index < node_count; ++index) {
    if (unknown_boundary[index]) {
      unknown_seeds.push_back(index);
    }
  }
  const std::vector<double> unknown_distances = BoundedMultiSourceDijkstra(
    adjacency, unknown_seeds, options_.truncation_distance_m);

  // Bounded multi-source Dijkstra from occupied nodes produces the nearest
  // observed metric. Overflowed or over-range candidates are discarded.
  std::vector<std::size_t> occupied_seeds;
  occupied_seeds.reserve(node_count);
  for (std::size_t index = 0; index < node_count; ++index) {
    if (occupied[index]) {
      occupied_seeds.push_back(index);
    }
  }
  const std::vector<double> distances = BoundedMultiSourceDijkstra(
    adjacency, occupied_seeds, options_.truncation_distance_m);

  // Populate the immutable record pages once for a full build. The node -> page
  // index is also built once and then shared by every incremental field that
  // keeps the same terrain pointer.
  std::map<ObstacleRecordTileKey, ObstacleRecordTile> mutable_tiles;
  auto record_id_index = std::make_shared<ObstacleRecordIdIndex>();
  record_id_index->reserve(node_count);
  for (std::size_t index = 0; index < node_count; ++index) {
    const TerrainNode & node = nodes[index];
    const ObstacleRecordTileKey key{
      FloorDivide(node.cell.x, kObstacleRecordTileCells),
      FloorDivide(node.cell.y, kObstacleRecordTileCells)};
    mutable_tiles[key].records.emplace(
      node.id,
      MakeRecord(
        node, merged_attributes[index], distances[index],
        std::isfinite(distances[index]), unknown_distances[index],
        unknown_boundary[index], adjacent_unknown[index],
        options_.truncation_distance_m));
    record_id_index->emplace(node.id, key);
  }
  records_.clear();
  for (auto & entry : mutable_tiles) {
    records_.emplace(
      entry.first,
      std::make_shared<const ObstacleRecordTile>(std::move(entry.second)));
  }
  record_id_index_ = std::move(record_id_index);
  ready_ = true;
}

// Convenience builder from a pinned map snapshot. It intentionally constructs a
// new value; production searches should use MapSnapshot::distance_field()
// instead of rebuilding the field for every search.
ObstacleDistanceField ObstacleDistanceField::Build(
  const MapSnapshot & map, ObstacleDistanceOptions options)
{
  if (map.fine_terrain() == nullptr) {
    return ObstacleDistanceField{};
  }
  return ObstacleDistanceField(
    map.fine_terrain(), map.static_obstacles(),
    map.dynamic_obstacles(), options);
}

// Bounded incremental update. The page directory is copied so the previous field
// stays immutable, but only pages intersecting the halo within the truncation
// radius of the changed cells are cloned and recomputed. Boundary seeds from the
// unchanged outside records make deletions correct; the full build path remains
// for first/stale inputs.
std::shared_ptr<const ObstacleDistanceField>
ObstacleDistanceField::UpdateIncremental(
  std::shared_ptr<const ObstacleDistanceField> previous,
  std::shared_ptr<const TerrainSnapshot> terrain,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  const std::vector<FineCellIndex> & changed_cells,
  ObstacleDistanceOptions options)
{
  const auto full_rebuild =
    [terrain, static_obstacles, dynamic_obstacles,
      options]() -> std::shared_ptr<const ObstacleDistanceField> {
      return std::make_shared<ObstacleDistanceField>(
        terrain, static_obstacles, dynamic_obstacles, options);
    };

  if (previous == nullptr || !previous->ready_ || terrain == nullptr ||
    !terrain->IsUsable())
  {
    return full_rebuild();
  }
  if (previous->terrain_.get() != terrain.get() ||
    !SameOptions(previous->options_, options))
  {
    return full_rebuild();
  }
  const bool layers_unchanged =
    previous->static_obstacles_.version() == static_obstacles.version() &&
    previous->dynamic_obstacles_.version() == dynamic_obstacles.version();
  if (changed_cells.empty()) {
    return layers_unchanged ? previous : full_rebuild();
  }
  if (previous->adjacency_ == nullptr || previous->id_to_index_ == nullptr ||
    previous->record_id_index_ == nullptr ||
    previous->id_to_index_->size() != terrain->nodes().size() ||
    previous->record_id_index_->size() != terrain->nodes().size())
  {
    return full_rebuild();
  }

  // Resolve the changed cells to node IDs. A missing record means the previous
  // field does not describe this terrain and must be rebuilt fully.
  std::vector<TerrainNodeId> changed_ids;
  for (const FineCellIndex & cell : changed_cells) {
    const std::vector<TerrainNodeId> * nodes = terrain->FindNodesInCell(cell);
    if (nodes == nullptr) {
      continue;
    }
    for (const TerrainNodeId id : *nodes) {
      if (previous->FindRecord(id) == nullptr) {
        return full_rebuild();
      }
      changed_ids.push_back(id);
    }
  }
  std::sort(changed_ids.begin(), changed_ids.end());
  changed_ids.erase(
    std::unique(changed_ids.begin(), changed_ids.end()),
    changed_ids.end());
  if (changed_ids.empty()) {
    return layers_unchanged ? previous : full_rebuild();
  }

  // Halo = every node within the truncation radius of a changed node. Nodes
  // outside this set keep their previous records; their evidence cannot change
  // because a changed node is farther than R away from them.
  std::unordered_set<TerrainNodeId> halo(changed_ids.begin(),
    changed_ids.end());
  std::unordered_map<TerrainNodeId, double> halo_distance;
  using QueueEntry = std::pair<double, TerrainNodeId>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  open;
  for (const TerrainNodeId id : changed_ids) {
    halo_distance.emplace(id, 0.0);
    open.emplace(0.0, id);
  }
  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    const auto distance_iterator = halo_distance.find(entry.second);
    if (distance_iterator == halo_distance.end() ||
      entry.first > distance_iterator->second + kDistanceEpsilon)
    {
      continue;
    }
    const auto adjacency_iterator = previous->adjacency_->find(entry.second);
    if (adjacency_iterator == previous->adjacency_->end()) {
      continue;
    }
    for (const auto & edge : adjacency_iterator->second) {
      const double candidate = entry.first + edge.second;
      if (!std::isfinite(candidate) ||
        candidate > options.truncation_distance_m + kDistanceEpsilon)
      {
        continue;
      }
      const auto existing = halo_distance.find(edge.first);
      if (existing == halo_distance.end() ||
        candidate + kDistanceEpsilon < existing->second)
      {
        halo_distance[edge.first] = candidate;
        halo.insert(edge.first);
        open.emplace(candidate, edge.first);
      }
    }
  }

  auto updated = std::make_shared<ObstacleDistanceField>();
  updated->terrain_ = terrain;
  updated->static_obstacles_ = std::move(static_obstacles);
  updated->dynamic_obstacles_ = std::move(dynamic_obstacles);
  updated->options_ = options;
  updated->adjacency_ = previous->adjacency_;
  updated->id_to_index_ = previous->id_to_index_;
  updated->min_cell_x_ = previous->min_cell_x_;
  updated->max_cell_x_ = previous->max_cell_x_;
  updated->min_cell_y_ = previous->min_cell_y_;
  updated->max_cell_y_ = previous->max_cell_y_;
  // The page directory copy is O(number of pages) and copies shared_ptrs only.
  updated->records_ = previous->records_;
  updated->record_id_index_ = previous->record_id_index_;
  updated->ready_ = true;

  // Clone only the record pages that contain a halo node. Every other page keeps
  // the previous shared_ptr, so its backing records stay pointer-identical.
  std::map<ObstacleRecordTileKey, std::shared_ptr<ObstacleRecordTile>>
  touched_tiles;
  for (const TerrainNodeId id : halo) {
    const auto index_iterator = updated->record_id_index_->find(id);
    if (index_iterator == updated->record_id_index_->end()) {
      return full_rebuild();
    }
    const ObstacleRecordTileKey key = index_iterator->second;
    if (touched_tiles.find(key) != touched_tiles.end()) {
      continue;
    }
    auto tile = std::make_shared<ObstacleRecordTile>();
    const auto base_iterator = updated->records_.find(key);
    if (base_iterator == updated->records_.end() ||
      base_iterator->second == nullptr)
    {
      // The shared index and the page directory disagree; a partial page would
      // silently drop the non-halo records that share this page.
      return full_rebuild();
    }
    tile->records = base_iterator->second->records;
    touched_tiles.emplace(key, tile);
    updated->records_[key] = tile;
  }

  // Recompute layer attributes and local unknown-boundary seeds for the halo.
  std::unordered_map<TerrainNodeId, ObstacleAttributes> halo_attributes;
  std::unordered_map<TerrainNodeId, ObstacleState> halo_states;
  std::unordered_map<TerrainNodeId, bool> halo_unknown_boundary;
  std::unordered_map<TerrainNodeId, double> occupied_distance;
  std::unordered_map<TerrainNodeId, double> unknown_distance;
  halo_attributes.reserve(halo.size());
  halo_states.reserve(halo.size());
  halo_unknown_boundary.reserve(halo.size());
  occupied_distance.reserve(halo.size());
  unknown_distance.reserve(halo.size());
  for (const TerrainNodeId id : halo) {
    const TerrainNode * node = terrain->FindNode(id);
    if (node == nullptr) {
      return full_rebuild();
    }
    const ObstacleAttributes attributes =
      MergeLayers(updated->static_obstacles_, updated->dynamic_obstacles_, id);
    const ObstacleState state = attributes.CombinedState();
    halo_attributes.emplace(id, attributes);
    halo_states.emplace(id, state);
    const bool unknown_boundary =
      node->status == TerrainStatus::kUnknown ||
      state == ObstacleState::kUnknown ||
      MissingInteriorNeighbor(
      *terrain, *node, updated->min_cell_x_,
      updated->max_cell_x_, updated->min_cell_y_,
      updated->max_cell_y_);
    halo_unknown_boundary.emplace(id, unknown_boundary);
    occupied_distance.emplace(
      id, state == ObstacleState::kOccupied ? 0.0 : kInfinity);
    unknown_distance.emplace(id, unknown_boundary ? 0.0 : kInfinity);
  }

  // Seed the halo from unchanged outside records. An exact occupied distance is
  // a valid boundary seed; a truncated value is the cap R and cannot improve a
  // halo record, so it is intentionally ignored. The same rule applies to the
  // bounded unknown-boundary distance. This is what makes deletions correct.
  for (const TerrainNodeId id : halo) {
    const auto adjacency_iterator = previous->adjacency_->find(id);
    if (adjacency_iterator == previous->adjacency_->end()) {
      continue;
    }
    for (const auto & edge : adjacency_iterator->second) {
      if (halo.find(edge.first) != halo.end()) {
        continue;
      }
      const ObstacleDistanceRecord * record = previous->FindRecord(edge.first);
      if (record == nullptr) {
        continue;
      }
      if (record->nearest_known && !record->truncated &&
        std::isfinite(record->nearest_observed_distance_m))
      {
        const double candidate =
          record->nearest_observed_distance_m + edge.second;
        if (candidate <= options.truncation_distance_m + kDistanceEpsilon) {
          occupied_distance[id] = std::min(occupied_distance[id], candidate);
        }
      }
      if (std::isfinite(record->unknown_boundary_distance_m)) {
        const double candidate =
          record->unknown_boundary_distance_m + edge.second;
        if (candidate <= options.truncation_distance_m + kDistanceEpsilon) {
          unknown_distance[id] = std::min(unknown_distance[id], candidate);
        }
      }
    }
  }

  // Relax only inside the halo using the same edge weights as the full build.
  const auto relax_halo = [&halo, &previous, &options](
    std::unordered_map<TerrainNodeId, double> *
    distances) {
      std::priority_queue<QueueEntry, std::vector<QueueEntry>,
        std::greater<QueueEntry>>
      queue;
      for (const TerrainNodeId id : halo) {
        if (std::isfinite((*distances)[id])) {
          queue.emplace((*distances)[id], id);
        }
      }
      while (!queue.empty()) {
        const QueueEntry entry = queue.top();
        queue.pop();
        if (entry.first > (*distances)[entry.second] + kDistanceEpsilon) {
          continue;
        }
        const auto adjacency_iterator = previous->adjacency_->find(entry.second);
        if (adjacency_iterator == previous->adjacency_->end()) {
          continue;
        }
        for (const auto & edge : adjacency_iterator->second) {
          if (halo.find(edge.first) == halo.end()) {
            continue;
          }
          const double candidate = entry.first + edge.second;
          if (!std::isfinite(candidate) ||
            candidate > options.truncation_distance_m + kDistanceEpsilon)
          {
            continue;
          }
          double & neighbor_distance = (*distances)[edge.first];
          if (candidate + kDistanceEpsilon < neighbor_distance) {
            neighbor_distance = candidate;
            queue.emplace(candidate, edge.first);
          }
        }
      }
    };
  relax_halo(&occupied_distance);
  relax_halo(&unknown_distance);

  // Rebuild the records for the halo only. Everything outside stays untouched.
  for (const TerrainNodeId id : halo) {
    const TerrainNode * node = terrain->FindNode(id);
    if (node == nullptr) {
      return full_rebuild();
    }
    bool adjacent_unknown = halo_unknown_boundary[id];
    if (!adjacent_unknown) {
      const auto adjacency_iterator = previous->adjacency_->find(id);
      if (adjacency_iterator != previous->adjacency_->end()) {
        for (const auto & edge : adjacency_iterator->second) {
          const auto state_iterator = halo_states.find(edge.first);
          const TerrainNode * neighbor_node = terrain->FindNode(edge.first);
          if (neighbor_node == nullptr) {
            continue;
          }
          bool unknown_neighbor =
            neighbor_node->status == TerrainStatus::kUnknown;
          if (!unknown_neighbor) {
            if (state_iterator != halo_states.end()) {
              unknown_neighbor =
                state_iterator->second == ObstacleState::kUnknown;
            } else {
              const ObstacleDistanceRecord * neighbor_record =
                previous->FindRecord(edge.first);
              unknown_neighbor =
                neighbor_record != nullptr &&
                neighbor_record->attributes.CombinedState() ==
                ObstacleState::kUnknown;
            }
          }
          if (unknown_neighbor) {
            adjacent_unknown = true;
            break;
          }
        }
      }
    }
    const auto index_iterator = updated->record_id_index_->find(id);
    const auto tile_iterator =
      index_iterator == updated->record_id_index_->end() ?
      touched_tiles.end() :
      touched_tiles.find(index_iterator->second);
    if (tile_iterator == touched_tiles.end() ||
      tile_iterator->second == nullptr)
    {
      return full_rebuild();
    }
    tile_iterator->second->records[id] = MakeRecord(
      *node, halo_attributes[id], occupied_distance[id],
      std::isfinite(occupied_distance[id]), unknown_distance[id],
      halo_unknown_boundary[id], adjacent_unknown,
      options.truncation_distance_m);
  }
  return updated;
}

// Merges both immutable layers for one node. The tiled record is the primary
// source so queries never rescan the layer tables; an ID without a record falls
// back to the layer merge to preserve the legacy behavior for foreign IDs.
ObstacleAttributes ObstacleDistanceField::MergedObstacle(
  TerrainNodeId id) const
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  if (record != nullptr) {
    return record->attributes;
  }
  return MergeLayers(static_obstacles_, dynamic_obstacles_, id);
}

// Returns the conservative merged obstacle state for one node.
ObstacleState ObstacleDistanceField::State(TerrainNodeId id) const
{
  return MergedObstacle(id).CombinedState();
}

// Traversability depends on support and the configured unknown/prohibited
// policy, not on whether an observed obstacle happens to exist or on an exact
// sub-truncation distance.
bool ObstacleDistanceField::IsTraversable(TerrainNodeId id) const noexcept
{
  if (!ready_ || terrain_ == nullptr) {
    return false;
  }
  const TerrainNode * node = terrain_->FindNode(id);
  if (node == nullptr || !node->IsSupported()) {
    return false;
  }
  const ObstacleDistanceRecord * record = FindRecord(id);
  if (record == nullptr) {
    return false;
  }
  const ObstacleState state = record->attributes.CombinedState();
  if (state == ObstacleState::kOccupied) {
    return false;
  }
  if (state == ObstacleState::kUnknown && options_.reject_unknown) {
    return false;
  }
  if (node->status == TerrainStatus::kProhibited &&
    options_.reject_prohibited)
  {
    return false;
  }
  return true;
}

// True when a bounded metric is known, including a truncated lower bound.
bool ObstacleDistanceField::IsDistanceKnown(TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  return record != nullptr && record->nearest_known;
}

// Returns the bounded metric min(distance, truncation), or NaN when the node
// has no supported/known evidence.
double ObstacleDistanceField::DistanceM(TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  if (record == nullptr || !record->nearest_known ||
    !std::isfinite(record->nearest_observed_distance_m))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return record->nearest_observed_distance_m;
}

// True when the reported nearest distance is the explicit truncation cap.
bool ObstacleDistanceField::IsTruncated(TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  return record != nullptr && record->truncated;
}

// True when the bounded local clearance is not contradicted by an unknown
// boundary inside the same radius.
bool ObstacleDistanceField::HasGuaranteedClearance(
  TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  return record != nullptr && record->clearance_guaranteed;
}

// Returns the bounded clearance, or NaN when no local guarantee exists. For a
// truncated record the truncation distance is returned as an explicit lower
// bound; inspect IsTruncated() to distinguish it from an exact distance.
double ObstacleDistanceField::GuaranteedClearanceM(
  TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  if (record == nullptr || !record->clearance_guaranteed ||
    !std::isfinite(record->nearest_observed_distance_m))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return record->nearest_observed_distance_m;
}

// True when a connected neighbor is unknown terrain or has unknown obstacle
// attributes. This is a one-cell diagnostic.
bool ObstacleDistanceField::NearUnobservedBoundary(
  TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  return record != nullptr && record->adjacent_unknown;
}

// True when an unknown boundary is reachable inside the truncation radius.
bool ObstacleDistanceField::HasLocalUnknownInfluence(
  TerrainNodeId id) const noexcept
{
  const ObstacleDistanceRecord * record = FindRecord(id);
  return record != nullptr && record->local_unknown_influence;
}

// Resolves one node through the shared geometry index and its immutable page.
// No lock is taken: both the index and every page are read-only after publish.
const ObstacleDistanceRecord * ObstacleDistanceField::FindRecord(
  TerrainNodeId id) const noexcept
{
  if (record_id_index_ == nullptr) {
    return nullptr;
  }
  const auto index_iterator = record_id_index_->find(id);
  if (index_iterator == record_id_index_->end()) {
    return nullptr;
  }
  const auto tile_iterator = records_.find(index_iterator->second);
  if (tile_iterator == records_.end() || tile_iterator->second == nullptr) {
    return nullptr;
  }
  const auto record_iterator = tile_iterator->second->records.find(id);
  if (record_iterator == tile_iterator->second->records.end()) {
    return nullptr;
  }
  return &record_iterator->second;
}

// Returns the immutable page directory entry by reference. No lock and no
// reference-count change: the page map is read-only after publication.
const std::shared_ptr<const ObstacleRecordTile> &
ObstacleDistanceField::FindRecordTile(
  ObstacleRecordTileKey key) const noexcept
{
  const auto tile_iterator = records_.find(key);
  if (tile_iterator == records_.end()) {
    static const std::shared_ptr<const ObstacleRecordTile> kEmptyTile;
    return kEmptyTile;
  }
  return tile_iterator->second;
}

// Independent Bellman-Ford reference for the bounded metric. Unknown influence
// and clearance guarantees are intentionally not recomputed here; this remains
// a test-only cross-check of min(distance, truncation).
double ReferenceSameSurfaceDistance(
  const TerrainSnapshot & terrain, const ObstacleLayer & static_obstacles,
  const ObstacleLayer & dynamic_obstacles, TerrainNodeId target,
  const ObstacleDistanceOptions & options, bool * known, bool * truncated)
{
  if (known != nullptr) {
    *known = false;
  }
  if (truncated != nullptr) {
    *truncated = false;
  }
  if (!std::isfinite(options.truncation_distance_m) ||
    options.truncation_distance_m <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const TerrainNode * target_node = terrain.FindNode(target);
  if (target_node == nullptr) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::vector<TerrainNode> & nodes = terrain.nodes();
  std::unordered_map<TerrainNodeId, std::size_t> id_to_index;
  id_to_index.reserve(nodes.size());
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    id_to_index.emplace(nodes[index].id, index);
  }
  std::vector<double> distances(nodes.size(), kInfinity);
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const ObstacleAttributes attributes =
      MergeLayers(static_obstacles, dynamic_obstacles, nodes[index].id);
    if (attributes.CombinedState() == ObstacleState::kOccupied) {
      distances[index] = 0.0;
    }
  }

  const std::size_t iteration_limit = std::max<std::size_t>(1, nodes.size());
  for (std::size_t iteration = 0; iteration < iteration_limit; ++iteration) {
    bool changed = false;
    for (const TerrainConnection & connection : terrain.connections()) {
      if (!connection.IsConnected()) {
        continue;
      }
      const auto source_iterator = id_to_index.find(connection.source);
      const auto target_iterator = id_to_index.find(connection.target);
      if (source_iterator == id_to_index.end() ||
        target_iterator == id_to_index.end())
      {
        continue;
      }
      const double weight = EdgeWeight(connection, terrain.resolution());
      if (!std::isfinite(weight) || weight <= 0.0) {
        continue;
      }
      const double candidate = distances[source_iterator->second] + weight;
      if (std::isfinite(candidate) &&
        candidate <= options.truncation_distance_m + kDistanceEpsilon &&
        candidate + kDistanceEpsilon < distances[target_iterator->second])
      {
        distances[target_iterator->second] = candidate;
        changed = true;
      }
      const double reverse_candidate =
        distances[target_iterator->second] + weight;
      if (std::isfinite(reverse_candidate) &&
        reverse_candidate <=
        options.truncation_distance_m + kDistanceEpsilon &&
        reverse_candidate + kDistanceEpsilon <
        distances[source_iterator->second])
      {
        distances[source_iterator->second] = reverse_candidate;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }

  const auto target_iterator = id_to_index.find(target);
  if (target_iterator == id_to_index.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const ObstacleAttributes attributes =
    MergeLayers(static_obstacles, dynamic_obstacles, target);
  const ObstacleDistanceRecord record = MakeRecord(
    *target_node, attributes, distances[target_iterator->second],
    std::isfinite(distances[target_iterator->second]), kInfinity, false,
    false, options.truncation_distance_m);
  if (known != nullptr) {
    *known = record.nearest_known;
  }
  if (truncated != nullptr) {
    *truncated = record.truncated;
  }
  return record.nearest_observed_distance_m;
}

}  // namespace dddmr_grid_global_planner
