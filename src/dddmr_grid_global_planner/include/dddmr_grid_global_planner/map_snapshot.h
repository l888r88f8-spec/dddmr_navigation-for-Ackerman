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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__MAP_SNAPSHOT_H_
#define DDDMR_GRID_GLOBAL_PLANNER__MAP_SNAPSHOT_H_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dddmr_grid_global_planner/measurement_evidence.h"
#include "dddmr_grid_global_planner/obstacle_tile.h"
#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

class CoarseTerrainMap;
class CoarseObstacleOverlay;
class ObstacleDistanceField;

// Map-level identity for one immutable fine terrain snapshot. Resolution,
// origin, frame, and vehicle configuration are map-level data, not node fields.
struct TerrainSnapshotMetadata
{
  std::string frame_id {"map"};
  double resolution {0.1};
  double origin_x {0.0};
  double origin_y {0.0};
  std::uint64_t revision {0};
  std::uint64_t content_hash {0};
  std::int64_t sensor_stamp_ns {0};
  std::int64_t received_steady_ns {0};
  std::uint64_t observation_sequence {0};
};

// Why one node's measured support domain ends. The two causes are independent
// evidence families and both flags may be set for the same node; this is not an
// enum priority. A measured edge means the support domain itself ends (no
// adjacent viable surface, a validated deck/cliff/prohibited rejection, or the
// outside map), which is not proof of a physical cliff or free volume. The
// insufficient-connection flag means a viable adjacent supported surface exists
// but its link was never evaluated or failed evidence, so the physical extent
// remains unknown and must stay conservative.
struct SupportBoundaryClassification
{
  bool measured_support_edge {false};
  bool insufficient_connection_evidence {false};
};

// Immutable fine terrain snapshot. Construction validates and builds lookup
// indexes once; no method mutates the node or connection arrays. The snapshot
// owns raw support evidence and candidate connections so later wheel-support
// checks do not need to query the live perception graph. Invalid structural
// input, unsafe cell coordinates, duplicate IDs, or dangling connection IDs
// throw std::invalid_argument rather than publishing a partially valid map.
class TerrainSnapshot
{
public:
  // Legacy three-argument constructor: delegates to the four-argument overload
  // with empty measurement evidence so existing callers remain compatible.
  TerrainSnapshot(
    std::vector<TerrainNode> nodes,
    std::vector<TerrainConnection> connections,
    TerrainSnapshotMetadata metadata);

  // Stores and validates immutable measurement evidence, then builds the
  // patch-id/cell/node indexes once. Invalid evidence throws
  // std::invalid_argument; no partially valid snapshot is published.
  TerrainSnapshot(
    std::vector<TerrainNode> nodes,
    std::vector<TerrainConnection> connections,
    TerrainSnapshotMetadata metadata,
    MeasurementEvidence measurement_evidence);

  // Returns all nodes in deterministic builder order.
  const std::vector<TerrainNode> & nodes() const noexcept {return nodes_;}

  // Returns all candidate connections, including rejected candidates.
  const std::vector<TerrainConnection> & connections() const noexcept
  {
    return connections_;
  }

  // Returns the node for one ID, or nullptr when the ID is absent.
  const TerrainNode * FindNode(TerrainNodeId id) const noexcept;

  // Returns the unique node at a world XY point. Ambiguous multi-layer cells
  // return nullptr so callers cannot silently pick a deck.
  const TerrainNode * FindNodeForPoint(const Point3 & point) const noexcept;

  // Returns the nodes in one cell, or nullptr when the cell is unobserved.
  const std::vector<TerrainNodeId> * FindNodesInCell(
    const FineCellIndex & cell) const noexcept;

  // Convenience copy of the node IDs in one cell.
  std::vector<TerrainNodeId> NodesInCell(const FineCellIndex & cell) const;

  // Returns the precomputed read-only adjacency list for one node. Only
  // connections with ConnectionStatus::kConnected are included. The pointer is
  // nullptr for an absent node ID and otherwise points at the snapshot-owned
  // list, which remains valid while the snapshot is alive.
  const std::vector<TerrainNodeId> * ConnectedNeighbors(
    TerrainNodeId id) const noexcept;

  // Compatibility conservative distance to the nearest boundary of either
  // evidence family: the minimum finite value of
  // MeasuredSupportBoundaryDistanceM() and InsufficientConnectionDistanceM().
  // It therefore preserves the previous risk envelope, because the shared
  // penalty kernel is monotone and max(p(d1), p(d2)) == p(min(d1, d2)). Both
  // arrays are infinite when a closed component has no classified boundary;
  // this method then returns positive infinity and callers must use the guarded
  // TerrainBoundaryEvidencePenalty() rather than feeding infinity directly to
  // TerrainBoundaryPenalty(). Propagation uses connected adjacent-XY supported
  // edges only; same-XY vertical/nonlocal links never propagate. Cached once at
  // construction, O(1) lookup, NaN for an absent ID. Not an exact Euclidean
  // clearance bound.
  double SupportBoundaryDistanceM(TerrainNodeId id) const noexcept;

  // Distance to the nearest node whose own measured support domain ends at a
  // known geometric, prohibited, deck-transition or outside-map edge. Positive
  // infinity means this component has no such seed (zero geometric
  // contribution, not missing data). NaN for an absent ID.
  double MeasuredSupportBoundaryDistanceM(TerrainNodeId id) const noexcept;

  // Distance to the nearest node with a viable adjacent supported surface whose
  // candidate link is missing or insufficient evidence. Positive infinity means
  // this component has no uncertainty seed (zero uncertainty contribution, not
  // missing data). NaN for an absent ID. This is the conservative unknown
  // margin; it is not reused from the dynamic-obstacle all-unknown field.
  double InsufficientConnectionDistanceM(TerrainNodeId id) const noexcept;

  // Per-node seed classification. Returns both flags when both causes apply.
  // Absent IDs return a default (false/false) classification; distance queries
  // still return NaN so missing data can fail closed.
  SupportBoundaryClassification SupportBoundaryClassificationAt(
    TerrainNodeId id) const noexcept;

  // Returns the candidate link in either direction, or nullptr when the pair
  // was never evaluated (unknown evidence).
  const TerrainConnection * FindConnection(
    TerrainNodeId first,
    TerrainNodeId second) const noexcept;

  // Returns the candidate status in either direction, or kUnknown when no
  // candidate was evaluated.
  ConnectionStatus ConnectionStatusBetween(
    TerrainNodeId first, TerrainNodeId second) const noexcept;

  // True when the snapshot has at least one node and valid map metadata.
  bool IsUsable() const noexcept;

  // True when two node IDs belong to the same connected supported component.
  //
  // Component labels are computed once while this immutable snapshot is
  // constructed, so the answer is exact regardless of component size; a
  // DFS bound would otherwise report a large but connected deck as separate
  // only because the budget ran out. ``max_nodes`` is retained for API
  // compatibility and still rejects a zero bound.
  bool AreConnected(
    TerrainNodeId first, TerrainNodeId second,
    std::size_t max_nodes) const noexcept;

  const std::string & frame_id() const noexcept {return metadata_.frame_id;}
  double resolution() const noexcept {return metadata_.resolution;}
  double origin_x() const noexcept {return metadata_.origin_x;}
  double origin_y() const noexcept {return metadata_.origin_y;}

  // Returns the content-driven revision. Equal content keeps this revision.
  std::uint64_t revision() const noexcept {return metadata_.revision;}

  // Returns the deterministic content hash used to drive the revision.
  std::uint64_t content_hash() const noexcept {return metadata_.content_hash;}

  // Returns the source sensor timestamp in its own clock domain.
  std::int64_t sensor_stamp_ns() const noexcept
  {
    return metadata_.sensor_stamp_ns;
  }

  // Returns the monotonic receipt timestamp used for freshness checks.
  std::int64_t received_steady_ns() const noexcept
  {
    return metadata_.received_steady_ns;
  }

  // Returns the source observation sequence.
  std::uint64_t observation_sequence() const noexcept
  {
    return metadata_.observation_sequence;
  }

  const TerrainSnapshotMetadata & metadata() const noexcept {return metadata_;}

  // Returns the immutable measurement evidence shared by every obstacle COW
  // snapshot that retains this terrain snapshot. It is never copied by an
  // obstacle-only update.
  const MeasurementEvidence & measurement_evidence() const noexcept
  {
    return measurement_evidence_;
  }

  // Returns the exact finite raw measurement point table (canonical order).
  const std::vector<Point3> & measurement_points() const noexcept
  {
    return measurement_evidence_.points;
  }

  // Returns every observed/fit attempt in canonical cell/layer/id order.
  const std::vector<MeasurementPatch> & measurement_patches() const noexcept
  {
    return measurement_evidence_.patches;
  }

  // Returns one patch by ID, or nullptr when absent. Lookup is O(1).
  const MeasurementPatch * FindMeasurementPatch(
    std::uint64_t id) const noexcept;

  // Returns the patch IDs in one cell sorted ascending, or nullptr when the
  // cell has no measurement evidence. A non-null vector can contain rejected
  // and unlinked patches; it is not a traversability query.
  const std::vector<std::uint64_t> * FindMeasurementPatchesInCell(
    const FineCellIndex & cell) const noexcept;

  // Returns the unique patch linked to a node, or nullptr when none exists.
  const MeasurementPatch * FindMeasurementPatchForNode(
    TerrainNodeId id) const noexcept;

  // Deterministic content hash of all measurement evidence. Empty evidence is 0.
  std::uint64_t measurement_hash() const noexcept {return measurement_hash_;}

private:
  // Constructs the immutable measured-edge and insufficient-connection distance
  // caches plus per-node cause flags after adjacency is valid. Called once.
  void BuildSupportBoundaryDistances();

  std::vector<TerrainNode> nodes_;
  std::vector<TerrainConnection> connections_;
  TerrainSnapshotMetadata metadata_;
  MeasurementEvidence measurement_evidence_;
  std::uint64_t measurement_hash_ {0};
  std::unordered_map<std::uint64_t, std::size_t> patch_id_to_index_;
  std::unordered_map<FineCellIndex, std::vector<std::uint64_t>,
    FineCellIndexHash>
  cell_to_patch_ids_;
  std::unordered_map<TerrainNodeId, std::uint64_t> node_to_patch_id_;
  std::unordered_map<TerrainNodeId, std::size_t> id_to_index_;
  std::unordered_map<FineCellIndex, std::vector<TerrainNodeId>,
    FineCellIndexHash>
  cell_to_nodes_;
  std::unordered_map<TerrainNodeId, std::vector<std::size_t>>
  node_to_connections_;
  std::unordered_map<TerrainNodeId, std::vector<TerrainNodeId>>
  connected_neighbors_;
  // Stable connected-component label per node (the smallest node ID in the
  // component). Precomputed so deck continuity never depends on a traversal
  // budget or on the order in which frontier nodes happen to be visited.
  std::unordered_map<TerrainNodeId, TerrainNodeId> component_label_;
  // Separate immutable evidence families, indexed by node order. Positive
  // infinity means "no seed of this family"; both arrays are built once from
  // validated adjacency and are never mutated afterward.
  std::vector<SupportBoundaryClassification> support_boundary_classifications_;
  std::vector<double> measured_support_boundary_distances_m_;
  std::vector<double> insufficient_connection_distances_m_;
};

// One static or dynamic obstacle layer keyed by terrain node ID. This class is a
// compatibility wrapper over an immutable tiled layer: construction from a flat
// unordered_map is retained for existing callers/tests, but lookup goes through
// the tiled node->tile index so a COW publish never scans or copies every node.
// There is no mutable accessor after construction; MapSnapshot shares this
// immutable view.
class ObstacleLayer
{
public:
  // Constructs an unobserved layer. Missing attributes remain unknown.
  ObstacleLayer() = default;

  // Compatibility constructor. Without terrain cells it stores every record in
  // the single tile {0,0}; Find() still resolves each ID through the sharded
  // node->tile index. Callers that have terrain should use FromFlat.
  ObstacleLayer(
    std::unordered_map<TerrainNodeId, ObstacleAttributes> attributes,
    std::int64_t observation_stamp_ns, std::int64_t received_steady_ns,
    std::uint64_t version, bool observed);

  // Wraps an already-tiled immutable layer. The tile map and index shards are
  // shared by pointer; no node table is copied.
  explicit ObstacleLayer(ObstacleTileLayer tiles);

  // Returns one node's attributes, or nullptr when the layer has no observation
  // for that node. Lookup is O(1) through the sharded index and one tile.
  const ObstacleAttributes * Find(TerrainNodeId id) const noexcept;

  // Returns the immutable tiled layer. Unchanged tiles are shared by pointer.
  const ObstacleTileLayer & tiles_layer() const noexcept {return tiles_;}

  // Returns the immutable tile map for diagnostics/tests.
  const ObstacleTileMap & tiles() const noexcept {return tiles_.tiles();}

  bool observed() const noexcept {return tiles_.observed();}
  std::int64_t observation_stamp_ns() const noexcept
  {
    return tiles_.observation_stamp_ns();
  }
  std::int64_t received_steady_ns() const noexcept
  {
    return tiles_.received_steady_ns();
  }
  std::uint64_t version() const noexcept {return tiles_.version();}

private:
  ObstacleTileLayer tiles_;
};

// Fixed-snapshot obstacle distance options. Distances are measured along
// connected fine terrain edges, never by straight-line XY proximity, so a
// bridge obstacle cannot leak onto the road below.
struct ObstacleDistanceOptions
{
  double truncation_distance_m {2.0};

  // Unknown obstacle attributes are not traversable by default. Setting this to
  // false is an explicit policy choice, not an assumption made by the code.
  bool reject_unknown {true};

  // Prohibited terrain is never traversable.
  bool reject_prohibited {true};
};

// Immutable planning snapshot: one fine terrain revision, an optional coarse
// summary revision, separate static/dynamic obstacle layers, an optional
// distance field pinned to those same inputs, and an optional coarse obstacle
// overlay pinned to the same revision/version. All pointers are const and remain
// valid while the holder keeps the snapshot alive.
class MapSnapshot
{
public:
  // Takes shared ownership of the terrain/coarse maps and moves immutable
  // obstacle layers into the snapshot. Throws std::invalid_argument when the
  // fine/coarse metadata disagree (frame, origin, ratio, or revision). The
  // optional coarse obstacle overlay is validated against the fine/coarse
  // revision, content hashes, and obstacle layer version when supplied.
  MapSnapshot(
    std::shared_ptr<const TerrainSnapshot> fine_terrain,
    std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
    ObstacleLayer static_obstacles,
    ObstacleLayer dynamic_obstacles,
    std::uint64_t obstacle_version,
    std::shared_ptr<const CoarseObstacleOverlay>
    coarse_obstacle_overlay = nullptr);

  // Creates one immutable snapshot with the obstacle distance field built
  // exactly once and pinned to the supplied fine terrain and layers. Throws
  // std::invalid_argument when the supplied inputs are inconsistent, when the
  // field fails readiness, when the field is not bound to the pinned fine
  // snapshot, or when a supplied coarse obstacle overlay does not match the
  // pinned fine/coarse revision and obstacle version.
  static std::shared_ptr<const MapSnapshot> Create(
    std::shared_ptr<const TerrainSnapshot> fine,
    std::shared_ptr<const CoarseTerrainMap> coarse,
    ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
    std::uint64_t obstacle_version,
    ObstacleDistanceOptions distance_options,
    std::shared_ptr<const CoarseObstacleOverlay>
    coarse_obstacle_overlay = nullptr);

  // Creates one immutable snapshot around an already-built distance field. The
  // field must be ready and bound to the supplied fine snapshot; this path
  // performs no field build, which is what the update worker uses after
  // ObstacleDistanceField::UpdateIncremental. Throws std::invalid_argument when
  // the field is null, not ready, bound to a different fine terrain, or when a
  // supplied coarse obstacle overlay does not match the pinned inputs.
  static std::shared_ptr<const MapSnapshot> CreateWithPinnedField(
    std::shared_ptr<const TerrainSnapshot> fine,
    std::shared_ptr<const CoarseTerrainMap> coarse,
    ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
    std::uint64_t obstacle_version,
    std::shared_ptr<const ObstacleDistanceField> field,
    std::shared_ptr<const CoarseObstacleOverlay>
    coarse_obstacle_overlay = nullptr);

  // Returns the pinned fine terrain snapshot.
  const std::shared_ptr<const TerrainSnapshot> & fine_terrain() const noexcept
  {
    return fine_terrain_;
  }

  // Returns the pinned coarse summary, or nullptr in single-resolution mode.
  const std::shared_ptr<const CoarseTerrainMap> & coarse_terrain() const noexcept
  {
    return coarse_terrain_;
  }

  // Returns the optional pinned coarse obstacle overlay, or nullptr when none
  // was supplied. Search should read this directly instead of scanning fine
  // members.
  const std::shared_ptr<const CoarseObstacleOverlay> & coarse_obstacle_overlay()
  const noexcept
  {
    return coarse_obstacle_overlay_;
  }

  // True when a coarse obstacle overlay is pinned.
  bool HasCoarseObstacleOverlay() const noexcept
  {
    return coarse_obstacle_overlay_ != nullptr;
  }

  // Returns the pinned precomputed distance field, or an empty shared pointer
  // when this snapshot was built through the legacy five-argument constructor.
  const std::shared_ptr<const ObstacleDistanceField> & distance_field()
  const noexcept
  {
    return distance_field_;
  }

  const ObstacleLayer & static_obstacles() const noexcept
  {
    return static_obstacles_;
  }
  const ObstacleLayer & dynamic_obstacles() const noexcept
  {
    return dynamic_obstacles_;
  }

  // Returns the obstacle aggregation version.
  std::uint64_t obstacle_version() const noexcept {return obstacle_version_;}

  // Merges both layers for one node. An absent layer remains unknown; missing
  // attributes are never interpreted as clear.
  ObstacleAttributes MergedObstacle(TerrainNodeId id) const;

  // True when a usable fine terrain snapshot is present.
  bool IsReady() const noexcept;

  // True when a coarse summary is pinned with the fine terrain.
  bool HasCoarseMap() const noexcept {return coarse_terrain_ != nullptr;}

  // True when the coarse summary is aligned with the pinned fine revision.
  bool CoarseMatchesFine() const noexcept;

  // True when a ready pinned distance field is present and its terrain pointer
  // equals the pinned fine snapshot pointer. The legacy five-argument
  // constructor always returns false here.
  bool HasReadyDistanceField() const noexcept;

private:
  // Shared construction path that optionally pins an already-built distance
  // field and coarse obstacle overlay. Throws std::invalid_argument on
  // coarse/fine mismatch, when a supplied field is not ready or not bound to
  // fine_terrain_, or when a supplied overlay does not match the pinned inputs.
  MapSnapshot(
    std::shared_ptr<const TerrainSnapshot> fine_terrain,
    std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
    ObstacleLayer static_obstacles,
    ObstacleLayer dynamic_obstacles,
    std::uint64_t obstacle_version,
    std::shared_ptr<const ObstacleDistanceField>
    pinned_distance_field,
    std::shared_ptr<const CoarseObstacleOverlay>
    coarse_obstacle_overlay);

  std::shared_ptr<const TerrainSnapshot> fine_terrain_;
  std::shared_ptr<const CoarseTerrainMap> coarse_terrain_;
  ObstacleLayer static_obstacles_;
  ObstacleLayer dynamic_obstacles_;
  std::uint64_t obstacle_version_ {0};
  // Pinned at construction by Create(); null for the legacy constructor.
  std::shared_ptr<const ObstacleDistanceField> distance_field_;
  // Optional coarse obstacle summary overlay pinned with the same inputs.
  std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacle_overlay_;
};

// Read-only corridor view. It holds shared references to the pinned maps plus a
// list of fine cells; it does not deep-copy the full fine map or coarse map.
class CorridorView
{
public:
  // Creates a view over the supplied cells. Throws std::invalid_argument when
  // the fine map is null or a cell is outside the safe index bound.
  CorridorView(
    std::shared_ptr<const TerrainSnapshot> fine_terrain,
    std::shared_ptr<const CoarseTerrainMap> coarse_terrain,
    std::vector<FineCellIndex> cells);

  // Returns true when the corridor contains the fine cell.
  bool ContainsFineCell(const FineCellIndex & cell) const noexcept;

  // Returns the fine node IDs in one corridor cell without copying terrain.
  std::vector<TerrainNodeId> FineNodesInCell(const FineCellIndex & cell) const;

  // Returns all corridor fine cells in the supplied deterministic order.
  const std::vector<FineCellIndex> & fine_cells() const noexcept {return cells_;}

  std::size_t size() const noexcept {return cells_.size();}

  const std::shared_ptr<const TerrainSnapshot> & fine_terrain() const noexcept
  {
    return fine_terrain_;
  }

  const std::shared_ptr<const CoarseTerrainMap> & coarse_terrain() const noexcept
  {
    return coarse_terrain_;
  }

private:
  std::shared_ptr<const TerrainSnapshot> fine_terrain_;
  std::shared_ptr<const CoarseTerrainMap> coarse_terrain_;
  std::vector<FineCellIndex> cells_;
  std::unordered_map<FineCellIndex, std::size_t, FineCellIndexHash> cell_index_;
};

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__MAP_SNAPSHOT_H_
