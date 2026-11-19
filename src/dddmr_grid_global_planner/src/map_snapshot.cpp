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

#include "grid_global_planner/map_snapshot.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/multiresolution_map.h"

namespace grid_global_planner
{
namespace
{

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

// Rejects a structural snapshot contract violation with one diagnostic.
[[noreturn]] void RejectSnapshot(const std::string & detail)
{
  throw std::invalid_argument("TerrainSnapshot: " + detail);
}

}  // namespace

TerrainSnapshot::TerrainSnapshot(
  std::vector<TerrainNode> nodes,
  std::vector<TerrainConnection> connections,
  TerrainSnapshotMetadata metadata)
: TerrainSnapshot(
    std::move(nodes), std::move(connections), std::move(metadata),
    MeasurementEvidence{})
{
}

TerrainSnapshot::TerrainSnapshot(
  std::vector<TerrainNode> nodes,
  std::vector<TerrainConnection> connections,
  TerrainSnapshotMetadata metadata,
  MeasurementEvidence measurement_evidence)
: nodes_(std::move(nodes)),
  connections_(std::move(connections)),
  metadata_(std::move(metadata)),
  measurement_evidence_(std::move(measurement_evidence))
{
  if (metadata_.frame_id.empty()) {
    RejectSnapshot("frame_id is empty");
  }
  if (!IsValidResolution(metadata_.resolution)) {
    RejectSnapshot("resolution must be finite and positive");
  }
  if (!IsSafeWorldCoordinate(metadata_.origin_x) ||
    !IsSafeWorldCoordinate(metadata_.origin_y))
  {
    RejectSnapshot("origin is outside the safe world coordinate bound");
  }

  id_to_index_.reserve(nodes_.size());
  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    const TerrainNode & node = nodes_[index];
    if (node.id == kInvalidTerrainNodeId) {
      RejectSnapshot("node ID zero is reserved");
    }
    if (!id_to_index_.emplace(node.id, index).second) {
      RejectSnapshot("duplicate node ID " + std::to_string(node.id));
    }
    // Keep an empty adjacency entry for isolated nodes so ConnectedNeighbors
    // can distinguish an absent ID from a node with no connected neighbors.
    connected_neighbors_.try_emplace(node.id);
    if (!IsSafeFineCell(node.cell)) {
      RejectSnapshot(
        "node " + std::to_string(node.id) +
        " has an unsafe cell index");
    }
    if (!node.representative.IsFinite()) {
      RejectSnapshot(
        "node " + std::to_string(node.id) +
        " has a non-finite representative point");
    }
    if (node.local_surface_index < 0) {
      RejectSnapshot(
        "node " + std::to_string(node.id) +
        " has a negative local surface index");
    }
    if (node.status == TerrainStatus::kSupported &&
      (!node.plane.valid || node.support.sample_count == 0))
    {
      RejectSnapshot(
        "supported node " + std::to_string(node.id) +
        " has no fitted plane/support evidence");
    }
    if (node.status == TerrainStatus::kSupported &&
      node.reason != ProhibitedReason::kNone)
    {
      RejectSnapshot(
        "supported node " + std::to_string(node.id) +
        " has a prohibited reason");
    }
    cell_to_nodes_[node.cell].push_back(node.id);
  }
  node_to_connections_.reserve(nodes_.size());
  for (std::size_t index = 0; index < connections_.size(); ++index) {
    const TerrainConnection & connection = connections_[index];
    if (connection.source == kInvalidTerrainNodeId ||
      connection.target == kInvalidTerrainNodeId)
    {
      RejectSnapshot("connection has an invalid endpoint ID");
    }
    if (connection.source == connection.target) {
      RejectSnapshot("connection links a node to itself");
    }
    if (id_to_index_.find(connection.source) == id_to_index_.end() ||
      id_to_index_.find(connection.target) == id_to_index_.end())
    {
      RejectSnapshot("connection references an unknown node ID");
    }
    node_to_connections_[connection.source].push_back(index);
    node_to_connections_[connection.target].push_back(index);
    if (connection.IsConnected()) {
      connected_neighbors_[connection.source].push_back(connection.target);
      connected_neighbors_[connection.target].push_back(connection.source);
    }
  }
  // Sort and deduplicate once so connectivity checks see a stable, read-only
  // adjacency list even if duplicate candidate connections were supplied.
  for (auto & entry : connected_neighbors_) {
    std::vector<TerrainNodeId> & neighbors = entry.second;
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(
      std::unique(neighbors.begin(), neighbors.end()),
      neighbors.end());
  }

  // Assign one stable label per connected component while the snapshot is
  // still being built. The live/offline deck check then compares two O(1)
  // labels instead of running a budgeted DFS per pose. The label is the
  // smallest node ID in the component so the result is deterministic and does
  // not depend on hash iteration order.
  component_label_.reserve(nodes_.size());
  std::vector<TerrainNodeId> members;
  std::vector<TerrainNodeId> component_stack;
  for (const TerrainNode & node : nodes_) {
    if (component_label_.find(node.id) != component_label_.end()) {
      continue;
    }
    members.clear();
    component_stack.clear();
    component_stack.push_back(node.id);
    component_label_.emplace(node.id, node.id);
    while (!component_stack.empty()) {
      const TerrainNodeId current = component_stack.back();
      component_stack.pop_back();
      members.push_back(current);
      const auto iterator = connected_neighbors_.find(current);
      if (iterator == connected_neighbors_.end()) {
        continue;
      }
      for (const TerrainNodeId next : iterator->second) {
        if (component_label_.emplace(next, node.id).second) {
          component_stack.push_back(next);
        }
      }
    }
    TerrainNodeId label = node.id;
    for (const TerrainNodeId member : members) {
      label = std::min(label, member);
    }
    for (const TerrainNodeId member : members) {
      component_label_[member] = label;
    }
  }

  // Validate measurement evidence after node identity/cell maps exist. A
  // failure throws before any index is published, so no partial planner
  // snapshot can be observed.
  std::string measurement_detail;
  if (!ValidateMeasurementEvidence(
      measurement_evidence_, nodes_,
      metadata_.origin_x, metadata_.origin_y, metadata_.resolution,
      &measurement_detail))
  {
    RejectSnapshot("invalid measurement evidence: " + measurement_detail);
  }
  measurement_hash_ = HashMeasurementEvidence(measurement_evidence_);
  patch_id_to_index_.reserve(measurement_evidence_.patches.size());
  node_to_patch_id_.reserve(measurement_evidence_.patches.size());
  for (std::size_t index = 0;
    index < measurement_evidence_.patches.size(); ++index)
  {
    const MeasurementPatch & patch = measurement_evidence_.patches[index];
    patch_id_to_index_.emplace(patch.id, index);
    cell_to_patch_ids_[patch.cell].push_back(patch.id);
    if (patch.node_id != kInvalidTerrainNodeId) {
      node_to_patch_id_.emplace(patch.node_id, patch.id);
    }
  }
  for (auto & entry : cell_to_patch_ids_) {
    std::sort(entry.second.begin(), entry.second.end());
  }

  // Adjacency, component labels and all structural validation are complete.
  // Build the immutable boundary-distance cache exactly once; obstacle-only
  // snapshots share this TerrainSnapshot, so they share the cache too.
  BuildSupportBoundaryDistances();
}

const MeasurementPatch * TerrainSnapshot::FindMeasurementPatch(
  std::uint64_t id) const noexcept
{
  const auto iterator = patch_id_to_index_.find(id);
  if (iterator == patch_id_to_index_.end()) {
    return nullptr;
  }
  return &measurement_evidence_.patches[iterator->second];
}

const std::vector<std::uint64_t> *
TerrainSnapshot::FindMeasurementPatchesInCell(
  const FineCellIndex & cell) const noexcept
{
  const auto iterator = cell_to_patch_ids_.find(cell);
  if (iterator == cell_to_patch_ids_.end()) {
    return nullptr;
  }
  return &iterator->second;
}

const MeasurementPatch * TerrainSnapshot::FindMeasurementPatchForNode(
  TerrainNodeId id) const noexcept
{
  const auto iterator = node_to_patch_id_.find(id);
  if (iterator == node_to_patch_id_.end()) {
    return nullptr;
  }
  return FindMeasurementPatch(iterator->second);
}

const TerrainNode * TerrainSnapshot::FindNode(TerrainNodeId id) const noexcept
{
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end()) {
    return nullptr;
  }
  return &nodes_[iterator->second];
}

const TerrainNode * TerrainSnapshot::FindNodeForPoint(
  const Point3 & point) const noexcept
{
  if (!point.IsFinite()) {
    return nullptr;
  }
  const auto cell = FineCellOf(
    point, metadata_.origin_x, metadata_.origin_y,
    metadata_.resolution);
  if (!cell.has_value()) {
    return nullptr;
  }
  const auto iterator = cell_to_nodes_.find(*cell);
  if (iterator == cell_to_nodes_.end() || iterator->second.size() != 1U) {
    return nullptr;
  }
  return FindNode(iterator->second.front());
}

const std::vector<TerrainNodeId> * TerrainSnapshot::FindNodesInCell(
  const FineCellIndex & cell) const noexcept
{
  const auto iterator = cell_to_nodes_.find(cell);
  if (iterator == cell_to_nodes_.end()) {
    return nullptr;
  }
  return &iterator->second;
}

std::vector<TerrainNodeId> TerrainSnapshot::NodesInCell(
  const FineCellIndex & cell) const
{
  const auto * nodes = FindNodesInCell(cell);
  return nodes == nullptr ? std::vector<TerrainNodeId>{} : *nodes;
}

const std::vector<TerrainNodeId> * TerrainSnapshot::ConnectedNeighbors(
  TerrainNodeId id) const noexcept
{
  const auto iterator = connected_neighbors_.find(id);
  if (iterator == connected_neighbors_.end()) {
    return nullptr;
  }
  return &iterator->second;
}

const TerrainConnection * TerrainSnapshot::FindConnection(
  TerrainNodeId first, TerrainNodeId second) const noexcept
{
  const auto iterator = node_to_connections_.find(first);
  if (iterator == node_to_connections_.end()) {
    return nullptr;
  }
  for (const std::size_t index : iterator->second) {
    const TerrainConnection & connection = connections_[index];
    if ((connection.source == first && connection.target == second) ||
      (connection.source == second && connection.target == first))
    {
      return &connection;
    }
  }
  return nullptr;
}

ConnectionStatus TerrainSnapshot::ConnectionStatusBetween(
  TerrainNodeId first, TerrainNodeId second) const noexcept
{
  const TerrainConnection * connection = FindConnection(first, second);
  return connection == nullptr ? ConnectionStatus::kUnknown :
         connection->status;
}

bool TerrainSnapshot::IsUsable() const noexcept
{
  return !nodes_.empty() && IsValidResolution(metadata_.resolution) &&
         IsSafeWorldCoordinate(metadata_.origin_x) &&
         IsSafeWorldCoordinate(metadata_.origin_y);
}

bool TerrainSnapshot::AreConnected(
  TerrainNodeId first, TerrainNodeId second,
  std::size_t max_nodes) const noexcept
{
  // The labels are precomputed in the constructor, so this is an exact
  // component query rather than a traversal that can exhaust a budget on a
  // large but genuinely connected road. max_nodes is retained for callers that
  // pass the configured bound; zero still means "do not answer".
  if (max_nodes == 0) {
    return false;
  }
  if (first == second) {
    return FindNode(first) != nullptr;
  }
  const auto first_label = component_label_.find(first);
  const auto second_label = component_label_.find(second);
  if (first_label == component_label_.end() ||
    second_label == component_label_.end())
  {
    return false;
  }
  return first_label->second == second_label->second;
}

ObstacleLayer::ObstacleLayer(
  std::unordered_map<TerrainNodeId, ObstacleAttributes> attributes,
  std::int64_t observation_stamp_ns, std::int64_t received_steady_ns,
  std::uint64_t version, bool observed)
{
  // The compatibility constructor has no terrain cell information. Store every
  // record in tile {0,0} and index each node to that tile; Find() remains an
  // O(1) node lookup and never scans the map.
  auto tile = std::make_shared<ObstacleTile>();
  tile->key = ObstacleTileKey{};
  tile->attributes = std::move(attributes);
  std::unordered_map<TerrainNodeId, ObstacleTileKey> id_tiles;
  id_tiles.reserve(tile->attributes.size());
  for (const auto & entry : tile->attributes) {
    id_tiles.emplace(entry.first, ObstacleTileKey{});
  }
  ObstacleTileMap tiles;
  tiles.emplace(tile->key, std::move(tile));
  tiles_ = ObstacleTileLayer(
    std::move(tiles), std::move(id_tiles),
    observation_stamp_ns, received_steady_ns, version,
    observed);
}

ObstacleLayer::ObstacleLayer(ObstacleTileLayer tiles)
: tiles_(std::move(tiles)) {}

const ObstacleAttributes * ObstacleLayer::Find(TerrainNodeId id) const noexcept
{
  return tiles_.Find(id);
}

// Legacy constructor. It deliberately leaves distance_field_ and the coarse
// obstacle overlay empty so callers that still build a standalone snapshot keep
// compiling; new code must use Create() to pin precomputed inputs.
MapSnapshot::MapSnapshot(
  std::shared_ptr<const TerrainSnapshot> fine_terrain,
  std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  std::uint64_t obstacle_version,
  std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacle_overlay)
: MapSnapshot(std::move(fine_terrain), std::move(coarse_terrain),
    std::move(static_obstacles), std::move(dynamic_obstacles),
    obstacle_version, nullptr,
    std::move(coarse_obstacle_overlay)) {}

// Shared construction path. It validates the same coarse/fine identity checks
// for every construction route and additionally validates a pinned field and
// coarse obstacle overlay when supplied.
MapSnapshot::MapSnapshot(
  std::shared_ptr<const TerrainSnapshot> fine_terrain,
  std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  std::uint64_t obstacle_version,
  std::shared_ptr<const ObstacleDistanceField> pinned_distance_field,
  std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacle_overlay)
: fine_terrain_(std::move(fine_terrain)),
  coarse_terrain_(std::move(coarse_terrain)),
  static_obstacles_(std::move(static_obstacles)),
  dynamic_obstacles_(std::move(dynamic_obstacles)),
  obstacle_version_(obstacle_version),
  distance_field_(std::move(pinned_distance_field)),
  coarse_obstacle_overlay_(std::move(coarse_obstacle_overlay))
{
  if (fine_terrain_ == nullptr) {
    throw std::invalid_argument("MapSnapshot: fine terrain is null");
  }
  if (coarse_terrain_ != nullptr) {
    if (coarse_terrain_->frame_id() != fine_terrain_->frame_id()) {
      throw std::invalid_argument("MapSnapshot: coarse/fine frame mismatch");
    }
    if (coarse_terrain_->origin_x() != fine_terrain_->origin_x() ||
      coarse_terrain_->origin_y() != fine_terrain_->origin_y())
    {
      throw std::invalid_argument("MapSnapshot: coarse/fine origin mismatch");
    }
    if (coarse_terrain_->fine_resolution() != fine_terrain_->resolution()) {
      throw std::invalid_argument(
              "MapSnapshot: coarse map was built from a different fine resolution");
    }
    if (coarse_terrain_->metadata().revision != fine_terrain_->revision() ||
      coarse_terrain_->metadata().fine_content_hash !=
      fine_terrain_->content_hash())
    {
      throw std::invalid_argument(
              "MapSnapshot: coarse summary does not match the fine revision");
    }
    if (coarse_terrain_->resolution_ratio() < 2) {
      throw std::invalid_argument(
              "MapSnapshot: coarse resolution must be an integer multiple greater "
              "than the fine resolution");
    }
  }
  if (distance_field_ != nullptr) {
    if (!distance_field_->ready()) {
      throw std::invalid_argument(
              "MapSnapshot: pinned distance field is not ready: " +
              distance_field_->validation_detail());
    }
    if (distance_field_->terrain().get() != fine_terrain_.get()) {
      throw std::invalid_argument(
              "MapSnapshot: pinned distance field is not bound to the fine "
              "terrain snapshot");
    }
  }
  if (coarse_obstacle_overlay_ != nullptr) {
    if (coarse_terrain_ == nullptr) {
      throw std::invalid_argument(
              "MapSnapshot: coarse obstacle overlay requires a coarse terrain map");
    }
    const CoarseObstacleOverlayMetadata & overlay_metadata =
      coarse_obstacle_overlay_->metadata();
    if (overlay_metadata.fine_revision != fine_terrain_->revision() ||
      overlay_metadata.fine_content_hash != fine_terrain_->content_hash())
    {
      throw std::invalid_argument(
              "MapSnapshot: coarse obstacle overlay does not match the fine "
              "revision");
    }
    if (overlay_metadata.coarse_revision != coarse_terrain_->revision() ||
      overlay_metadata.coarse_content_hash != coarse_terrain_->content_hash())
    {
      throw std::invalid_argument(
              "MapSnapshot: coarse obstacle overlay does not match the coarse "
              "revision");
    }
    if (overlay_metadata.obstacle_layer_version != obstacle_version_) {
      throw std::invalid_argument(
              "MapSnapshot: coarse obstacle overlay was built for a different "
              "obstacle layer version");
    }
  }
}

// Builds the field once from the supplied immutable inputs and pins it to the
// returned snapshot. Failure is reported as std::invalid_argument so a
// partially ready snapshot is never published.
std::shared_ptr<const MapSnapshot> MapSnapshot::Create(
  std::shared_ptr<const TerrainSnapshot> fine,
  std::shared_ptr<const CoarseTerrainMap> coarse,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  std::uint64_t obstacle_version,
  ObstacleDistanceOptions distance_options,
  std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacle_overlay)
{
  if (fine == nullptr) {
    throw std::invalid_argument("MapSnapshot::Create: fine terrain is null");
  }
  std::shared_ptr<const ObstacleDistanceField> field =
    std::make_shared<ObstacleDistanceField>(
    fine, static_obstacles, dynamic_obstacles, distance_options);
  if (!field->ready()) {
    throw std::invalid_argument(
            "MapSnapshot::Create: distance field is not ready: " +
            field->validation_detail());
  }
  if (field->terrain().get() != fine.get()) {
    throw std::invalid_argument(
            "MapSnapshot::Create: distance field is not bound to the pinned fine "
            "terrain snapshot");
  }
  return std::shared_ptr<const MapSnapshot>(
    new MapSnapshot(
      std::move(fine), std::move(coarse), std::move(static_obstacles),
      std::move(dynamic_obstacles), obstacle_version, std::move(field),
      std::move(coarse_obstacle_overlay)));
}

// Pins an already-built field. The shared constructor below validates readiness
// and pointer identity, so a stale or mismatched field or overlay can never be
// published.
std::shared_ptr<const MapSnapshot> MapSnapshot::CreateWithPinnedField(
  std::shared_ptr<const TerrainSnapshot> fine,
  std::shared_ptr<const CoarseTerrainMap> coarse,
  ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
  std::uint64_t obstacle_version,
  std::shared_ptr<const ObstacleDistanceField> field,
  std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacle_overlay)
{
  if (fine == nullptr) {
    throw std::invalid_argument(
            "MapSnapshot::CreateWithPinnedField: fine terrain is null");
  }
  if (field == nullptr) {
    throw std::invalid_argument(
            "MapSnapshot::CreateWithPinnedField: distance field is null");
  }
  return std::shared_ptr<const MapSnapshot>(
    new MapSnapshot(
      std::move(fine), std::move(coarse), std::move(static_obstacles),
      std::move(dynamic_obstacles), obstacle_version, std::move(field),
      std::move(coarse_obstacle_overlay)));
}

ObstacleAttributes MapSnapshot::MergedObstacle(TerrainNodeId id) const
{
  ObstacleAttributes merged;
  const ObstacleAttributes * static_attributes = static_obstacles_.Find(id);
  const ObstacleAttributes * dynamic_attributes = dynamic_obstacles_.Find(id);
  if (static_attributes == nullptr && dynamic_attributes == nullptr) {
    return merged;
  }
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
    // A union of occupied slabs is conservative: a body test must consider
    // every vertical interval that either layer observed.
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
    // Observed free extents intersect: merging two layers can only reduce the
    // vertical range that both explicitly observed as clear.
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

bool MapSnapshot::IsReady() const noexcept
{
  return fine_terrain_ != nullptr && fine_terrain_->IsUsable();
}

bool MapSnapshot::CoarseMatchesFine() const noexcept
{
  if (fine_terrain_ == nullptr || coarse_terrain_ == nullptr) {
    return false;
  }
  return coarse_terrain_->frame_id() == fine_terrain_->frame_id() &&
         coarse_terrain_->origin_x() == fine_terrain_->origin_x() &&
         coarse_terrain_->origin_y() == fine_terrain_->origin_y() &&
         coarse_terrain_->fine_resolution() == fine_terrain_->resolution() &&
         coarse_terrain_->metadata().revision == fine_terrain_->revision() &&
         coarse_terrain_->metadata().fine_content_hash ==
         fine_terrain_->content_hash();
}

bool MapSnapshot::HasReadyDistanceField() const noexcept
{
  return distance_field_ != nullptr && distance_field_->ready() &&
         fine_terrain_ != nullptr &&
         distance_field_->terrain().get() == fine_terrain_.get();
}

CorridorView::CorridorView(
  std::shared_ptr<const TerrainSnapshot> fine_terrain,
  std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
  std::vector<FineCellIndex> cells)
: fine_terrain_(std::move(fine_terrain)),
  coarse_terrain_(std::move(coarse_terrain)),
  cells_(std::move(cells))
{
  if (fine_terrain_ == nullptr) {
    throw std::invalid_argument("CorridorView: fine terrain is null");
  }
  if (coarse_terrain_ != nullptr &&
    coarse_terrain_->metadata().revision != fine_terrain_->revision())
  {
    throw std::invalid_argument(
            "CorridorView: coarse/fine revisions do not match");
  }
  cell_index_.reserve(cells_.size());
  for (std::size_t index = 0; index < cells_.size(); ++index) {
    if (!IsSafeFineCell(cells_[index])) {
      throw std::invalid_argument("CorridorView: unsafe fine cell index");
    }
    cell_index_.emplace(cells_[index], index);
  }
}

bool CorridorView::ContainsFineCell(const FineCellIndex & cell) const noexcept
{
  return cell_index_.find(cell) != cell_index_.end();
}

std::vector<TerrainNodeId> CorridorView::FineNodesInCell(
  const FineCellIndex & cell) const
{
  if (fine_terrain_ == nullptr || !ContainsFineCell(cell)) {
    return {};
  }
  return fine_terrain_->NodesInCell(cell);
}

}  // namespace grid_global_planner
