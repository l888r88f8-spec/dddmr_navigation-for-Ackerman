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

#include "grid_global_planner/obstacle_tile.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "grid_global_planner/map_snapshot.h"

namespace grid_global_planner
{
namespace
{

// Returns the shard that owns one node ID. The shard count is a power of two so
// the mask is exact; the shift mixes sequential builder IDs across shards.
std::size_t ShardIndex(TerrainNodeId id) noexcept
{
  const std::uint64_t mixed = id ^ (id >> 7U);
  return static_cast<std::size_t>(mixed) &
         (kObstacleTileIndexShardCount - 1U);
}

// Partitions a flat node->tile index into immutable shards. This is used only
// when a layer is created from an already-flat table, never on a COW publish.
std::array<std::shared_ptr<const ObstacleNodeTileShard>,
  kObstacleTileIndexShardCount>
BuildShards(
  const std::unordered_map<TerrainNodeId, ObstacleTileKey> & id_tiles)
{
  std::array<std::shared_ptr<ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  shards{};
  for (const auto & entry : id_tiles) {
    const std::size_t shard = ShardIndex(entry.first);
    if (shards[shard] == nullptr) {
      shards[shard] = std::make_shared<ObstacleNodeTileShard>();
    }
    shards[shard]->emplace(entry.first, entry.second);
  }
  std::array<std::shared_ptr<const ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = shards[index];
  }
  return result;
}

// Number of fine cells that fit in one tile. Tile size is quantized to whole
// fine cells so a tile key is always an exact cell partition.
std::int64_t TileCellCount(double tile_size_m, double fine_resolution_m)
{
  if (!std::isfinite(tile_size_m) || tile_size_m <= 0.0 ||
    !IsValidResolution(fine_resolution_m))
  {
    throw std::invalid_argument(
            "ObstacleTile: tile size and fine resolution must be finite/positive");
  }
  double raw_cells = tile_size_m / fine_resolution_m;
  if (!std::isfinite(raw_cells) || raw_cells < 1.0) {
    raw_cells = 1.0;
  }
  // Keep the divisor inside a safe integer range. A 3.2 m tile at 0.1 m is 32
  // cells; this bound only prevents an absurd explicit option from overflowing.
  constexpr double kMaxTileCells = 1.0e9;
  if (raw_cells > kMaxTileCells) {
    raw_cells = kMaxTileCells;
  }
  const auto cells = static_cast<std::int64_t>(std::llround(raw_cells));
  return cells < 1 ? 1 : cells;
}

// Wraps an immutable shard array plus its partition metadata into a shared
// geometry index. The array is moved, so nothing mutable escapes.
std::shared_ptr<const ObstacleTileGeometryIndex> MakeGeometryIndex(
  std::array<std::shared_ptr<const ObstacleNodeTileShard>,
  kObstacleTileIndexShardCount>
  shards,
  double fine_resolution_m, std::int64_t tile_cells, double tile_size_m,
  bool terrain_derived)
{
  auto index = std::make_shared<ObstacleTileGeometryIndex>();
  index->shards = std::move(shards);
  index->fine_resolution_m = fine_resolution_m;
  index->tile_cells = tile_cells;
  index->tile_size_m = tile_size_m;
  index->terrain_derived = terrain_derived;
  return index;
}

}  // namespace

// Builds one immutable node -> tile-key index from the terrain cells. The
// partition is quantized exactly as the tile layer, so keys match FromFlat and
// the builder for the same options.
std::shared_ptr<const ObstacleTileGeometryIndex> BuildGeometryIndex(
  const TerrainSnapshot & terrain, const ObstacleTileOptions & options)
{
  const std::int64_t tile_cells =
    TileCellCount(options.tile_size_m, terrain.resolution());
  std::array<std::shared_ptr<ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  mutable_shards{};
  for (const TerrainNode & node : terrain.nodes()) {
    const std::size_t shard = ShardIndex(node.id);
    if (mutable_shards[shard] == nullptr) {
      mutable_shards[shard] = std::make_shared<ObstacleNodeTileShard>();
    }
    mutable_shards[shard]->emplace(
      node.id,
      ObstacleTileKey{FloorDivide(node.cell.x, tile_cells),
        FloorDivide(node.cell.y, tile_cells)});
  }
  std::array<std::shared_ptr<const ObstacleNodeTileShard>,
    kObstacleTileIndexShardCount>
  frozen_shards{};
  for (std::size_t index = 0; index < frozen_shards.size(); ++index) {
    frozen_shards[index] = mutable_shards[index];
  }
  return MakeGeometryIndex(
    std::move(frozen_shards), terrain.resolution(),
    tile_cells, options.tile_size_m, true);
}

ObstacleTileLayer::ObstacleTileLayer(
  ObstacleTileMap tiles,
  std::unordered_map<TerrainNodeId, ObstacleTileKey> id_tiles,
  std::int64_t observation_stamp_ns, std::int64_t received_steady_ns,
  std::uint64_t version, bool observed)
: ObstacleTileLayer(
    std::move(tiles),
    MakeGeometryIndex(BuildShards(id_tiles), 0.1,
    TileCellCount(3.2, 0.1), 3.2, false),
    // Compatibility path has no terrain, so keep the documented 0.1 m
    // conservative fine resolution used for the synthesized index.
    0.1, observation_stamp_ns, received_steady_ns, version, observed) {}

ObstacleTileLayer::ObstacleTileLayer(
  ObstacleTileMap tiles,
  std::shared_ptr<const ObstacleTileGeometryIndex> geometry_index,
  double fine_resolution_m, std::int64_t observation_stamp_ns,
  std::int64_t received_steady_ns, std::uint64_t version, bool observed)
: tiles_(std::move(tiles)),
  geometry_index_(std::move(geometry_index)),
  fine_resolution_m_(fine_resolution_m),
  observation_stamp_ns_(observation_stamp_ns),
  received_steady_ns_(received_steady_ns),
  version_(version),
  observed_(observed) {}

ObstacleTileLayer ObstacleTileLayer::FromFlat(
  const TerrainSnapshot & terrain,
  const std::unordered_map<TerrainNodeId, ObstacleAttributes> & attributes,
  const ObstacleTileOptions & options, std::int64_t stamp_ns,
  std::int64_t received_ns, std::uint64_t version, bool observed)
{
  const std::int64_t tile_cells =
    TileCellCount(options.tile_size_m, terrain.resolution());
  // Fill mutable tiles first, then freeze them into the immutable tile map.
  std::map<ObstacleTileKey, std::shared_ptr<ObstacleTile>> mutable_tiles;
  std::unordered_map<TerrainNodeId, ObstacleTileKey> id_tiles;
  id_tiles.reserve(attributes.size());
  for (const auto & entry : attributes) {
    ObstacleTileKey key{};
    const TerrainNode * node = terrain.FindNode(entry.first);
    if (node != nullptr) {
      key = ObstacleTileKey{FloorDivide(node->cell.x, tile_cells),
        FloorDivide(node->cell.y, tile_cells)};
    }
    auto tile_iterator = mutable_tiles.find(key);
    if (tile_iterator == mutable_tiles.end()) {
      auto tile = std::make_shared<ObstacleTile>();
      tile->key = key;
      tile_iterator = mutable_tiles.emplace(key, std::move(tile)).first;
    }
    tile_iterator->second->attributes.emplace(entry.first, entry.second);
    id_tiles.emplace(entry.first, key);
  }
  ObstacleTileMap tiles;
  for (auto & entry : mutable_tiles) {
    tiles.emplace(entry.first, std::move(entry.second));
  }
  return ObstacleTileLayer(
    std::move(tiles),
    MakeGeometryIndex(
      BuildShards(id_tiles), terrain.resolution(),
      tile_cells, options.tile_size_m, true),
    terrain.resolution(), stamp_ns, received_ns, version, observed);
}

const ObstacleAttributes * ObstacleTileLayer::Find(
  TerrainNodeId id) const noexcept
{
  if (geometry_index_ == nullptr) {
    return nullptr;
  }
  const std::shared_ptr<const ObstacleNodeTileShard> & shard =
    geometry_index_->shards[ShardIndex(id)];
  if (shard == nullptr) {
    return nullptr;
  }
  const auto index_iterator = shard->find(id);
  if (index_iterator == shard->end()) {
    return nullptr;
  }
  const auto tile_iterator = tiles_.find(index_iterator->second);
  if (tile_iterator == tiles_.end() || tile_iterator->second == nullptr) {
    return nullptr;
  }
  const auto attribute_iterator = tile_iterator->second->attributes.find(id);
  if (attribute_iterator == tile_iterator->second->attributes.end()) {
    return nullptr;
  }
  return &attribute_iterator->second;
}

const std::shared_ptr<const ObstacleTile> & ObstacleTileLayer::FindTile(
  ObstacleTileKey key) const
{
  const auto iterator = tiles_.find(key);
  if (iterator == tiles_.end()) {
    static const std::shared_ptr<const ObstacleTile> kEmptyTile;
    return kEmptyTile;
  }
  return iterator->second;
}

ObstacleTileKey ObstacleTileLayer::TileKeyForCell(
  const FineCellIndex & cell) const noexcept
{
  // Published layers always carry the geometry index that produced their tile
  // partition. A default-constructed compatibility layer has no index and uses
  // the same 3.2 m / 0.1 m default as its compatibility constructor.
  const std::int64_t tile_cells =
    geometry_index_ != nullptr && geometry_index_->tile_cells > 0 ?
    geometry_index_->tile_cells :
    32;
  return ObstacleTileKey{FloorDivide(cell.x, tile_cells),
    FloorDivide(cell.y, tile_cells)};
}

ObstacleTileBuilder::ObstacleTileBuilder(
  const ObstacleTileLayer & base,
  ObstacleTileOptions options)
: ObstacleTileBuilder(base, base.geometry_index(), options) {}

ObstacleTileBuilder::ObstacleTileBuilder(
  const ObstacleTileLayer & base,
  std::shared_ptr<const ObstacleTileGeometryIndex> geometry,
  ObstacleTileOptions options)
: options_(options),
  base_tiles_(base.tiles()),
  geometry_index_(std::move(geometry))
{
  if (geometry_index_ == nullptr) {
    // Default/legacy base without an index. Derive an empty, non-terrain index
    // from the base resolution so the builder still works.
    const double fine_resolution =
      IsValidResolution(base.fine_resolution_m()) ? base.fine_resolution_m() :
      0.1;
    const std::int64_t tile_cells =
      TileCellCount(options_.tile_size_m, fine_resolution);
    std::array<std::shared_ptr<const ObstacleNodeTileShard>,
      kObstacleTileIndexShardCount>
    empty_shards{};
    geometry_index_ =
      MakeGeometryIndex(
      std::move(empty_shards), fine_resolution, tile_cells,
      options_.tile_size_m, false);
  }
  fine_resolution_m_ = geometry_index_->fine_resolution_m;
  if (!IsValidResolution(fine_resolution_m_)) {
    fine_resolution_m_ = 0.1;
  }
  // Keep the legacy option validation even when a shared geometry index supplies
  // the partition, so an invalid tile size still throws at construction.
  const std::int64_t option_tile_cells =
    TileCellCount(options_.tile_size_m, fine_resolution_m_);
  tile_cells_ = geometry_index_->tile_cells > 0 ?
    geometry_index_->tile_cells :
    option_tile_cells;
}

void ObstacleTileBuilder::ThrowIfSealed() const
{
  if (sealed_) {
    throw std::logic_error(
            "ObstacleTileBuilder: Set/Erase after Build() is not allowed");
  }
}

ObstacleTileKey ObstacleTileBuilder::KeyForCell(
  const FineCellIndex & cell) const noexcept
{
  return ObstacleTileKey{FloorDivide(cell.x, tile_cells_),
    FloorDivide(cell.y, tile_cells_)};
}

std::shared_ptr<ObstacleNodeTileShard> ObstacleTileBuilder::MutableShard(
  std::size_t shard)
{
  if (touched_shards_[shard] != nullptr) {
    return touched_shards_[shard];
  }
  auto clone = std::make_shared<ObstacleNodeTileShard>();
  if (geometry_index_ != nullptr &&
    geometry_index_->shards[shard] != nullptr)
  {
    *clone = *geometry_index_->shards[shard];
  }
  touched_shards_[shard] = clone;
  return clone;
}

const ObstacleTileKey * ObstacleTileBuilder::IndexedKey(
  TerrainNodeId id) const noexcept
{
  const std::size_t shard = ShardIndex(id);
  if (touched_shards_[shard] != nullptr) {
    const auto iterator = touched_shards_[shard]->find(id);
    if (iterator != touched_shards_[shard]->end()) {
      return &iterator->second;
    }
  }
  if (geometry_index_ != nullptr &&
    geometry_index_->shards[shard] != nullptr)
  {
    const auto iterator = geometry_index_->shards[shard]->find(id);
    if (iterator != geometry_index_->shards[shard]->end()) {
      return &iterator->second;
    }
  }
  return nullptr;
}

std::shared_ptr<ObstacleTile> ObstacleTileBuilder::MutableTile(
  const ObstacleTileKey & key)
{
  const auto touched_iterator = touched_tiles_.find(key);
  if (touched_iterator != touched_tiles_.end()) {
    return touched_iterator->second;
  }
  auto tile = std::make_shared<ObstacleTile>();
  tile->key = key;
  const auto base_iterator = base_tiles_.find(key);
  if (base_iterator != base_tiles_.end() && base_iterator->second != nullptr) {
    tile->attributes = base_iterator->second->attributes;
  }
  touched_tiles_.emplace(key, tile);
  return tile;
}

const ObstacleTile * ObstacleTileBuilder::CurrentTile(
  const ObstacleTileKey & key) const
{
  const auto touched_iterator = touched_tiles_.find(key);
  if (touched_iterator != touched_tiles_.end()) {
    return touched_iterator->second.get();
  }
  const auto base_iterator = base_tiles_.find(key);
  if (base_iterator != base_tiles_.end() && base_iterator->second != nullptr) {
    return base_iterator->second.get();
  }
  return nullptr;
}

void ObstacleTileBuilder::Set(
  FineCellIndex cell, TerrainNodeId id,
  ObstacleAttributes value)
{
  ThrowIfSealed();
  const ObstacleTileKey key = KeyForCell(cell);
  const ObstacleTileKey * current = IndexedKey(id);
  // Copy the old key before any index shard mutation; a move must remove the
  // record from the old tile so the two structures stay consistent.
  const bool is_new = current == nullptr;
  const bool moved = current != nullptr && !(*current == key);
  const ObstacleTileKey old_key = current != nullptr ? *current : key;
  if (moved) {
    const ObstacleTile * old_tile = CurrentTile(old_key);
    if (old_tile != nullptr &&
      old_tile->attributes.find(id) != old_tile->attributes.end())
    {
      MutableTile(old_key)->attributes.erase(id);
    }
  }
  MutableTile(key)->attributes[id] = std::move(value);
  // A same-id/same-key update mutates only the tile payload. Only a new ID or an
  // actual tile move may clone/update the one owning index shard.
  if (is_new || moved) {
    MutableShard(ShardIndex(id))->insert_or_assign(id, key);
  }
}

void ObstacleTileBuilder::Erase(FineCellIndex cell, TerrainNodeId id)
{
  ThrowIfSealed();
  const ObstacleTileKey * current = IndexedKey(id);
  const ObstacleTileKey key = current != nullptr ? *current : KeyForCell(cell);
  const ObstacleTile * tile = CurrentTile(key);
  if (tile != nullptr && tile->attributes.find(id) != tile->attributes.end()) {
    MutableTile(key)->attributes.erase(id);
  }
  // Also remove from the cell-derived tile when an unindexed compatibility
  // record was stored there.
  const ObstacleTileKey cell_key = KeyForCell(cell);
  if (!(cell_key == key)) {
    const ObstacleTile * cell_tile = CurrentTile(cell_key);
    if (cell_tile != nullptr &&
      cell_tile->attributes.find(id) != cell_tile->attributes.end())
    {
      MutableTile(cell_key)->attributes.erase(id);
    }
  }
  // The geometry index entry is deliberately retained. Find() already resolves a
  // missing tile record as absent, and a later same-key Set can reuse the entry
  // without cloning a shard.
}

ObstacleTileLayer ObstacleTileBuilder::Build(
  std::int64_t stamp_ns,
  std::int64_t received_ns,
  std::uint64_t version,
  bool observed) const
{
  sealed_ = true;
  // Copying the tile map copies only tile pointers, never individual node
  // records. Touched payloads are deep-frozen into new const tiles so later
  // builder activity can never rewrite this published layer.
  ObstacleTileMap result = base_tiles_;
  for (const auto & entry : touched_tiles_) {
    if (entry.second == nullptr || entry.second->attributes.empty()) {
      result.erase(entry.first);
    } else {
      result[entry.first] =
        std::make_shared<const ObstacleTile>(*entry.second);
    }
  }
  bool touched_any_shard = false;
  for (const auto & shard : touched_shards_) {
    if (shard != nullptr) {
      touched_any_shard = true;
      break;
    }
  }
  std::shared_ptr<const ObstacleTileGeometryIndex> geometry = geometry_index_;
  if (touched_any_shard) {
    // Rebuild only the 64-entry directory around the one cloned shard. All
    // untouched shards remain pointer-identical to the shared index.
    auto merged = std::make_shared<ObstacleTileGeometryIndex>(*geometry_index_);
    for (std::size_t index = 0; index < touched_shards_.size(); ++index) {
      if (touched_shards_[index] != nullptr) {
        merged->shards[index] = touched_shards_[index];
      }
    }
    geometry = std::move(merged);
  }
  return ObstacleTileLayer(
    std::move(result), std::move(geometry),
    fine_resolution_m_, stamp_ns, received_ns, version,
    observed);
}

}  // namespace grid_global_planner
