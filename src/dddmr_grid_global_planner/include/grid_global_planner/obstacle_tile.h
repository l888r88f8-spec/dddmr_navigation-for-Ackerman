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

#ifndef GRID_GLOBAL_PLANNER__OBSTACLE_TILE_H_
#define GRID_GLOBAL_PLANNER__OBSTACLE_TILE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "grid_global_planner/terrain_types.h"

namespace grid_global_planner
{

class TerrainSnapshot;

// One fixed XY tile key. Tile keys are floor divisions of fine cell indices, so
// a tile boundary is independent of the map origin and can never be affected by
// rounding a world coordinate.
struct ObstacleTileKey
{
  std::int64_t x{0};
  std::int64_t y{0};

  // Exact equality of both tile indices.
  bool operator==(const ObstacleTileKey & other) const noexcept
  {
    return x == other.x && y == other.y;
  }

  // Deterministic ordering used by the immutable tile map.
  bool operator<(const ObstacleTileKey & other) const noexcept
  {
    return x < other.x || (x == other.x && y < other.y);
  }
};

// Immutable tile payload. Attributes are keyed by terrain node ID; terrain nodes
// are never copied into a tile. The tile object itself is const after it is
// published, so a reader may keep a shared_ptr and observe no mutation.
struct ObstacleTile
{
  ObstacleTileKey key{};
  std::unordered_map<TerrainNodeId, ObstacleAttributes> attributes;
};

// Ordered immutable tile map. Map order is deterministic and shared tiles are
// referenced by the same shared_ptr, which is the copy-on-write publish unit.
using ObstacleTileMap =
  std::map<ObstacleTileKey, std::shared_ptr<const ObstacleTile>>;

// Tile policy. Tile size is quantized to an integer number of fine cells for a
// given fine resolution. reject_unknown only documents the downstream distance
// policy; an unknown attribute is still an explicit unknown record here and is
// never silently converted into clear space.
struct ObstacleTileOptions
{
  double tile_size_m{3.2};
  bool reject_unknown{true};
};

// Number of shards in the node-id -> tile-key index. Sharding allows the
// copy-on-write builder to clone only the index shards that contain touched
// node IDs instead of scanning or copying the whole node table.
inline constexpr std::size_t kObstacleTileIndexShardCount = 64U;

// One shard of the node-id -> tile-key index.
using ObstacleNodeTileShard =
  std::unordered_map<TerrainNodeId, ObstacleTileKey>;

// Immutable node-id -> fixed tile-key geometry index. It is built once from a
// terrain snapshot and can be shared by every obstacle payload revision that
// belongs to the same terrain revision. The shards are never mutated after
// publication; a builder that must change one entry clones only that shard.
struct ObstacleTileGeometryIndex
{
  std::array<std::shared_ptr<const ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  shards{};
  double fine_resolution_m{0.1};
  std::int64_t tile_cells{32};
  double tile_size_m{3.2};
  // True when this index was derived from an actual terrain snapshot rather
  // than from the legacy flat compatibility table.
  bool terrain_derived{false};
};

// Builds one immutable geometry index for a terrain snapshot. This is the only
// O(number of nodes) node->tile pass; callers can reuse the result for static
// and dynamic payloads on the same terrain revision instead of rebuilding it.
std::shared_ptr<const ObstacleTileGeometryIndex> BuildGeometryIndex(
  const TerrainSnapshot & terrain, const ObstacleTileOptions & options);

// Immutable tiled obstacle layer. Node lookup first resolves one sharded index
// entry and then one tile entry; it never scans all nodes. Default construction
// means "unobserved" and Find() returns nullptr for every ID.
class ObstacleTileLayer
{
public:
  // Constructs an unobserved layer. Missing attributes remain unknown.
  ObstacleTileLayer() = default;

  // Builds a layer from a pre-partitioned tile map plus a flat node->tile
  // index. Used by compatibility callers and by the worker after it has already
  // grouped attributes. The flat index is sharded once at construction.
  ObstacleTileLayer(
    ObstacleTileMap tiles,
    std::unordered_map<TerrainNodeId, ObstacleTileKey> id_tiles,
    std::int64_t observation_stamp_ns, std::int64_t received_steady_ns,
    std::uint64_t version, bool observed);

  // Partitions a flat attribute table into tiles using the terrain cell of each
  // node. Attributes whose node is absent from the terrain are kept in tile
  // {0,0} so Find() still resolves them; they are never treated as free space.
  static ObstacleTileLayer FromFlat(
    const TerrainSnapshot & terrain,
    const std::unordered_map<TerrainNodeId, ObstacleAttributes> & attributes,
    const ObstacleTileOptions & options, std::int64_t stamp_ns,
    std::int64_t received_ns, std::uint64_t version, bool observed);

  // Returns one node's attributes, or nullptr when this layer has no explicit
  // record for the node. Never scans the tile map or the node table.
  const ObstacleAttributes * Find(TerrainNodeId id) const noexcept;

  // Returns the immutable tile for a key, or a shared empty pointer when the
  // tile is absent. The returned reference remains valid while this layer lives.
  const std::shared_ptr<const ObstacleTile> & FindTile(
    ObstacleTileKey key) const;

  // Returns the fixed tile key that owns one fine cell. The quantization comes
  // from the layer's actual geometry index, so a commit predicate can compare
  // the same payload tile the worker would copy-on-write.
  ObstacleTileKey TileKeyForCell(const FineCellIndex & cell) const noexcept;

  // Returns the immutable tile map. Unchanged tiles are shared by pointer.
  const ObstacleTileMap & tiles() const noexcept {return tiles_;}

  // Returns the shared geometry index. Two payload revisions built from the same
  // terrain revision return the same pointer unless a node actually changed
  // tiles, in which case only the touched shard differs.
  const std::shared_ptr<const ObstacleTileGeometryIndex> & geometry_index()
  const noexcept
  {
    return geometry_index_;
  }

  bool observed() const noexcept {return observed_;}
  std::int64_t observation_stamp_ns() const noexcept
  {
    return observation_stamp_ns_;
  }
  std::int64_t received_steady_ns() const noexcept
  {
    return received_steady_ns_;
  }
  std::uint64_t version() const noexcept {return version_;}

  // Fine resolution used to derive tile keys. The compatibility constructor has
  // no terrain, so it uses the conservative default of 0.1 m.
  double fine_resolution_m() const noexcept {return fine_resolution_m_;}

private:
  friend class ObstacleTileBuilder;

  // Private construction path used by the builder after it has already cloned
  // the touched tiles and index shards.
  ObstacleTileLayer(
    ObstacleTileMap tiles,
    std::shared_ptr<const ObstacleTileGeometryIndex> geometry_index,
    double fine_resolution_m, std::int64_t observation_stamp_ns,
    std::int64_t received_steady_ns, std::uint64_t version, bool observed);

  ObstacleTileMap tiles_;
  std::shared_ptr<const ObstacleTileGeometryIndex> geometry_index_;
  double fine_resolution_m_{0.1};
  std::int64_t observation_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  std::uint64_t version_{0};
  bool observed_{false};
};

// One-writer copy-on-write delta builder. The base layer is never modified;
// only touched tiles and, on an actual tile-key change, one index shard are
// cloned. All other tiles and shards are shared with the base layer by pointer,
// so a publish does not copy every node and does not scan the full node table.
// A builder is single-use: Build() seals it and freezes the touched payloads, so
// a later Set/Erase throws std::logic_error instead of rewriting a published
// layer.
class ObstacleTileBuilder
{
public:
  // Captures the base layer and validates the tile policy. Reuses the base
  // layer's geometry index when it has one. Throws std::invalid_argument for a
  // non-finite/non-positive tile size.
  ObstacleTileBuilder(
    const ObstacleTileLayer & base,
    ObstacleTileOptions options);

  // Captures the base layer payload and uses the supplied shared geometry index
  // for node -> tile resolution. The index must belong to the same terrain
  // revision and tile policy as the base layer; this lets static and dynamic
  // payload builders reuse one O(N) index instead of rebuilding it.
  ObstacleTileBuilder(
    const ObstacleTileLayer & base,
    std::shared_ptr<const ObstacleTileGeometryIndex> geometry,
    ObstacleTileOptions options);

  // Copying a builder would alias its mutable touched tiles/shards, so a copy
  // could rewrite a layer already published by another builder. Copying is
  // therefore deleted; moving transfers the single-writer mutable state and is
  // safe because the moved-from builder is left with empty maps.
  ObstacleTileBuilder(const ObstacleTileBuilder &) = delete;
  ObstacleTileBuilder & operator=(const ObstacleTileBuilder &) = delete;
  ObstacleTileBuilder(ObstacleTileBuilder &&) noexcept = default;
  ObstacleTileBuilder & operator=(ObstacleTileBuilder &&) noexcept = default;

  // Sets one node's attributes in the tile derived from its fine cell. A move
  // between tiles removes the old record from the old tile. Throws
  // std::logic_error when called after Build(). A same-key update never clones
  // the geometry index.
  void Set(FineCellIndex cell, TerrainNodeId id, ObstacleAttributes value);

  // Removes one node's attributes. The indexed tile is used when known;
  // otherwise the tile derived from cell is used. The geometry index entry is
  // deliberately retained; a missing tile record is already resolved as absent.
  // Throws std::logic_error when called after Build().
  void Erase(FineCellIndex cell, TerrainNodeId id);

  // Publishes one immutable layer and seals this builder. Touched tiles are
  // deep-frozen into new const tiles so a later builder can never rewrite this
  // layer; unchanged tiles remain shared with the base layer. Empty touched
  // tiles are dropped from the result.
  ObstacleTileLayer Build(
    std::int64_t stamp_ns, std::int64_t received_ns,
    std::uint64_t version, bool observed) const;

private:
  // Returns the mutable working tile for key, cloning the base tile on first
  // touch. The returned pointer is owned by this builder.
  std::shared_ptr<ObstacleTile> MutableTile(const ObstacleTileKey & key);

  // Returns the current tile payload for key, preferring the touched clone.
  const ObstacleTile * CurrentTile(const ObstacleTileKey & key) const;

  // Returns the current tile key for id, or nullptr when id is unindexed.
  const ObstacleTileKey * IndexedKey(TerrainNodeId id) const noexcept;

  // Clones one index shard on first write. Only an actual key change (new node
  // or move between tiles) calls this.
  std::shared_ptr<ObstacleNodeTileShard> MutableShard(std::size_t shard);

  // Converts a fine cell to its fixed tile key.
  ObstacleTileKey KeyForCell(const FineCellIndex & cell) const noexcept;

  // Throws std::logic_error when Build() has already sealed this builder.
  void ThrowIfSealed() const;

  ObstacleTileOptions options_;
  double fine_resolution_m_{0.1};
  std::int64_t tile_cells_{32};
  ObstacleTileMap base_tiles_;
  std::map<ObstacleTileKey, std::shared_ptr<ObstacleTile>> touched_tiles_;
  std::shared_ptr<const ObstacleTileGeometryIndex> geometry_index_;
  std::array<std::shared_ptr<ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  touched_shards_{};
  // Mutable because Build() is const but still publishes/seals this builder.
  mutable bool sealed_{false};
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__OBSTACLE_TILE_H_
