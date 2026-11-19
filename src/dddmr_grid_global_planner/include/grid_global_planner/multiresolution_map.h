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

#ifndef GRID_GLOBAL_PLANNER__MULTIRESOLUTION_MAP_H_
#define GRID_GLOBAL_PLANNER__MULTIRESOLUTION_MAP_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/terrain_types.h"

namespace grid_global_planner
{

class ObstacleDistanceField;

// Coarse summary surface ID. Like TerrainNodeId it is version-scoped and must
// not be persisted across terrain revisions.
using CoarseSurfaceId = std::uint64_t;

inline constexpr CoarseSurfaceId kInvalidCoarseSurfaceId = 0;

// Mapping of one fine terrain node into the coarse summary. The fine node is
// never duplicated; this is only an index relationship.
struct FineToCoarseMapping
{
  CoarseCellIndex cell{};
  CoarseSurfaceId surface_id{kInvalidCoarseSurfaceId};
  int local_surface_index{-1};
};

// One conservative coarse summary. It retains all fine node IDs that compose
// the surface and is marked unknown or prohibited if any member is unknown or
// prohibited. Coarse Z grouping uses an independent tolerance and is never
// scaled with the coarse cell size.
// The summary is guidance only: a coarse no-route result is never authoritative
// and must not override the fine, unknown-aware terrain contract.
struct CoarseSurfaceSummary
{
  CoarseSurfaceId id{kInvalidCoarseSurfaceId};
  CoarseCellIndex cell{};
  int local_surface_index{-1};
  std::vector<TerrainNodeId> fine_nodes;
  Point3 representative{};
  double z_min{0.0};
  double z_max{0.0};
  double z_mean{0.0};
  double min_slope_rad{0.0};
  double max_slope_rad{0.0};
  TerrainStatus status{TerrainStatus::kUnknown};
  ProhibitedReason reason{ProhibitedReason::kInsufficientSamples};
  bool contains_unknown{false};
  bool contains_prohibited{false};

  // True when at least one expected fine cell in this coarse cell has no
  // observed fine node. Missing fine evidence is unobserved, so the summary
  // becomes unknown instead of inheriting support from a surviving neighbor.
  bool contains_missing_fine_cells{false};

  // True when this summary shares a fine cell with another coarse surface. The
  // fine builder deliberately emits no same-cell candidate link
  // (kSameCellNoLink), so such surfaces must never be merged at coarse level.
  bool same_cell_no_link{false};

  // Coarse surfaces that share at least one fine cell with this summary but are
  // deliberately unlinked. Sorted by ID for deterministic hashing.
  std::vector<CoarseSurfaceId> same_cell_no_link_surfaces;

  // True only when every fine member is supported and every expected fine cell
  // in the coarse cell was observed.
  bool IsSupported() const noexcept {return status == TerrainStatus::kSupported;}
};

// One coarse graph edge derived from at least one connected fine candidate
// connection. Both endpoints must map to different coarse surfaces.
struct CoarseTerrainConnection
{
  CoarseSurfaceId source{kInvalidCoarseSurfaceId};
  CoarseSurfaceId target{kInvalidCoarseSurfaceId};
  double horizontal_distance{0.0};
  double height_delta{0.0};
  double cost{0.0};
};

// Coarse-map identity. The coarse and fine maps must share frame, origin and
// revision; the fine resolution must divide the coarse resolution exactly.
struct CoarseTerrainMetadata
{
  std::string frame_id{"map"};
  double fine_resolution{0.1};
  double coarse_resolution{0.4};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t revision{0};
  std::uint64_t fine_content_hash{0};
  std::uint64_t content_hash{0};
};

// Immutable coarse guidance map. Construction validates resolution/origin
// alignment and every fine->coarse mapping, and no accessor exposes mutable
// storage. It stores summaries only; the authoritative terrain remains in the
// pinned fine snapshot.
class CoarseTerrainMap
{
public:
  CoarseTerrainMap(
    std::vector<CoarseSurfaceSummary> surfaces,
    std::vector<CoarseTerrainConnection> connections,
    std::unordered_map<TerrainNodeId, FineToCoarseMapping> mapping,
    CoarseTerrainMetadata metadata);

  // Returns all coarse surfaces in deterministic builder order.
  const std::vector<CoarseSurfaceSummary> & surfaces() const noexcept
  {
    return surfaces_;
  }

  // Returns all coarse graph edges, deduplicated by unordered surface pair.
  const std::vector<CoarseTerrainConnection> & connections() const noexcept
  {
    return connections_;
  }

  // Returns the fine-node mapping table.
  const std::unordered_map<TerrainNodeId, FineToCoarseMapping> & mapping()
  const noexcept
  {
    return mapping_;
  }

  // Returns a coarse surface by ID, or nullptr when absent.
  const CoarseSurfaceSummary * FindSurface(CoarseSurfaceId id) const noexcept;

  // Looks up the coarse surface containing one fine node.
  const CoarseSurfaceSummary * FindSurfaceForFineNode(
    TerrainNodeId fine_node) const noexcept;

  // Looks up one coarse cell and local surface index.
  const CoarseSurfaceSummary * FindSurfaceAt(
    const CoarseCellIndex & cell,
    int local_surface_index)
  const noexcept;

  // Returns every coarse surface in a cell, or nullptr when the cell is absent.
  const std::vector<CoarseSurfaceId> * FindSurfacesInCell(
    const CoarseCellIndex & cell) const noexcept;

  // True when both the fine and coarse metadata are usable.
  bool IsUsable() const noexcept;

  const std::string & frame_id() const noexcept {return metadata_.frame_id;}
  double fine_resolution() const noexcept
  {
    return metadata_.fine_resolution;
  }
  double coarse_resolution() const noexcept
  {
    return metadata_.coarse_resolution;
  }

  // Integer fine->coarse resolution ratio. Returns 0 when the metadata is
  // invalid instead of guessing a ratio.
  std::int64_t resolution_ratio() const noexcept;
  double origin_x() const noexcept {return metadata_.origin_x;}
  double origin_y() const noexcept {return metadata_.origin_y;}
  std::uint64_t revision() const noexcept {return metadata_.revision;}
  std::uint64_t content_hash() const noexcept {return metadata_.content_hash;}
  const CoarseTerrainMetadata & metadata() const noexcept {return metadata_;}

private:
  std::vector<CoarseSurfaceSummary> surfaces_;
  std::vector<CoarseTerrainConnection> connections_;
  std::unordered_map<TerrainNodeId, FineToCoarseMapping> mapping_;
  CoarseTerrainMetadata metadata_;
  std::unordered_map<CoarseSurfaceId, std::size_t> id_to_index_;
  std::unordered_map<CoarseCellIndex, std::vector<CoarseSurfaceId>,
    CoarseCellIndexHash>
  cell_to_surfaces_;
};

// One coarse-surface obstacle aggregate. It deliberately stores only summary
// evidence, never the fine member list, so a coarse search can read the overlay
// without scanning fine nodes. Occupied and unknown evidence are conservative:
// unknown is never collapsed into clear space.
struct CoarseObstacleSummary
{
  CoarseSurfaceId surface_id{kInvalidCoarseSurfaceId};

  // True when any fine member currently has occupied obstacle evidence.
  bool has_occupied{false};

  // True when any fine member has unknown terrain/obstacle evidence, or has no
  // distance-field record at all. Missing evidence is unknown, not clear.
  bool has_unknown{false};

  // True when any fine member terrain is prohibited.
  bool has_prohibited{false};

  // Minimum bounded nearest-obstacle distance over members with a known
  // record. NaN when no member has known evidence. A value of zero means an
  // occupied member; `clearance_truncated` marks a truncation lower bound.
  double min_clearance_m{std::numeric_limits<double>::quiet_NaN()};

  // True when at least one member has a known bounded distance record.
  bool observed{false};

  // True only when every member was observed clear with a known record and no
  // occupied/unknown/prohibited evidence was found.
  bool clear{false};

  // True when min_clearance_m is the truncation cap (a lower bound), not an
  // exact observed distance.
  bool clearance_truncated{false};

  // Number of fine member nodes represented by this summary.
  std::size_t fine_node_count{0};

  // Number of fine members with no distance-field record.
  std::size_t missing_record_count{0};
};

// Identity of one coarse obstacle overlay revision. The overlay is valid only
// together with the exact fine/coarse terrain revision and obstacle layer
// version it was built from.
struct CoarseObstacleOverlayMetadata
{
  std::uint64_t fine_revision{0};
  std::uint64_t fine_content_hash{0};
  std::uint64_t coarse_revision{0};
  std::uint64_t coarse_content_hash{0};
  std::uint64_t obstacle_layer_version{0};
};

// Immutable coarse obstacle summary overlay. `surface_summaries` is keyed by
// coarse surface ID and holds shared const summaries, so an incremental overlay
// can replace only touched surfaces while unaffected entries keep pointer
// identity. The inverse fine-node index lets an update route changed fine IDs to
// their surface without scanning any surface member list.
class CoarseObstacleOverlay
{
public:
  using SummaryMap =
    std::unordered_map<CoarseSurfaceId,
      std::shared_ptr<const CoarseObstacleSummary>>;
  using FineSurfaceIndex =
    std::unordered_map<TerrainNodeId, CoarseSurfaceId>;

  // Takes ownership of immutable summaries and the inverse fine-node index.
  // Throws std::invalid_argument for a null index, a zero/mismatched surface ID,
  // a null summary, or invalid summary evidence.
  CoarseObstacleOverlay(
    SummaryMap surface_summaries,
    std::shared_ptr<const FineSurfaceIndex> fine_surface_index,
    CoarseObstacleOverlayMetadata metadata);

  // Returns the immutable summary map.
  const SummaryMap & surface_summaries() const noexcept
  {
    return surface_summaries_;
  }

  // Returns the summary for one coarse surface, or nullptr when absent.
  const CoarseObstacleSummary * FindSummary(
    CoarseSurfaceId surface_id) const noexcept;

  // Returns the coarse surface ID owning one fine node, or nullptr when absent.
  const CoarseSurfaceId * FindSurfaceForFineNode(
    TerrainNodeId fine_node) const noexcept;

  // Returns the shared inverse fine-node -> coarse-surface index.
  const std::shared_ptr<const FineSurfaceIndex> & fine_surface_index()
  const noexcept
  {
    return fine_surface_index_;
  }

  // Returns the overlay metadata (revision hashes and obstacle layer version).
  const CoarseObstacleOverlayMetadata & metadata() const noexcept
  {
    return metadata_;
  }
  // Returns the fine terrain revision this overlay was built from.
  std::uint64_t fine_revision() const noexcept
  {
    return metadata_.fine_revision;
  }
  // Returns the fine terrain content hash this overlay was built from.
  std::uint64_t fine_content_hash() const noexcept
  {
    return metadata_.fine_content_hash;
  }
  // Returns the coarse terrain revision this overlay was built from.
  std::uint64_t coarse_revision() const noexcept
  {
    return metadata_.coarse_revision;
  }
  // Returns the coarse terrain content hash this overlay was built from.
  std::uint64_t coarse_content_hash() const noexcept
  {
    return metadata_.coarse_content_hash;
  }
  // Returns the obstacle layer version whose evidence this overlay summarizes.
  std::uint64_t obstacle_layer_version() const noexcept
  {
    return metadata_.obstacle_layer_version;
  }

private:
  SummaryMap surface_summaries_;
  std::shared_ptr<const FineSurfaceIndex> fine_surface_index_;
  CoarseObstacleOverlayMetadata metadata_;
};

// Coarse-map construction options. The Z tolerance is a surface-identity
// parameter, not a vehicle capability and not a function of cell size.
struct CoarseTerrainMapOptions
{
  double coarse_resolution{0.4};
  double surface_z_tolerance{0.20};

  // Safety budget for one coarse summary. Exceeding it marks the summary
  // prohibited instead of silently dropping fine nodes.
  std::size_t max_fine_nodes_per_surface{200'000};
};

enum class CoarseBuildStatus : std::uint8_t
{
  kOk = 0,
  kInvalidOptions,
  kInvalidFineSnapshot,
};

struct CoarseBuildResult
{
  CoarseBuildStatus status{CoarseBuildStatus::kInvalidOptions};
  std::string detail;
  std::shared_ptr<const CoarseTerrainMap> map;

  bool ok() const noexcept {return status == CoarseBuildStatus::kOk;}
};

// Deterministic coarse summarizer. It groups fine nodes by coarse cell, unions
// only explicitly connected fine surfaces within the independent Z tolerance,
// retains unknown/prohibited members conservatively, and derives coarse edges
// from connected fine candidate links.
class CoarseTerrainMapBuilder
{
public:
  explicit CoarseTerrainMapBuilder(CoarseTerrainMapOptions options = {});

  const CoarseTerrainMapOptions & options() const noexcept {return options_;}

  // Validates resolution alignment and tolerance bounds without building.
  bool ValidateOptions(std::string * detail = nullptr) const;

  // Builds one immutable coarse summary from an already-final fine snapshot.
  CoarseBuildResult Build(const TerrainSnapshot & fine_terrain) const;

private:
  CoarseTerrainMapOptions options_;
};

// Copy-on-write coarse terrain-geometry overlay. It handles terrain geometry
// only: coarse summaries are terrain-only, so an obstacle-only change does not
// alter any coarse surface. When `previous` already matches the fine
// revision/content hash this returns the same map by pointer (zero-copy). A
// terrain content change or a missing/mismatched previous map builds a fresh
// full baseline. `changed_cells` names obstacle-dirty coarse cells; it is
// validated here and is the routing seam for the separate coarse obstacle
// overlay (see OverlayCoarseObstacles), never a request to re-cluster terrain.
std::shared_ptr<const CoarseTerrainMap> OverlayCoarseTerrain(
  std::shared_ptr<const CoarseTerrainMap> previous,
  std::shared_ptr<const TerrainSnapshot> fine,
  const std::vector<CoarseCellIndex> & changed_cells,
  const CoarseTerrainMapOptions & options);

// Builds or incrementally updates the immutable per-coarse-surface obstacle
// overlay. A full build is used when `previous` is missing or its fine/coarse
// revision or content hash does not match. Otherwise only coarse surfaces
// touched by `changed_fine_ids` are recomputed from their own member IDs; all
// other summary shared_ptrs are reused. An empty changed list with a matching
// obstacle layer version returns `previous`; an empty list with a different
// version forces a full rebuild. Invalid/stale inputs never silently produce a
// mixed-revision overlay.
std::shared_ptr<const CoarseObstacleOverlay> OverlayCoarseObstacles(
  std::shared_ptr<const CoarseObstacleOverlay> previous,
  const CoarseTerrainMap & coarse, const ObstacleDistanceField & field,
  const std::vector<TerrainNodeId> & changed_fine_ids,
  std::uint64_t obstacle_layer_version);

// Returns a stable diagnostic spelling for a coarse build status.
const char * CoarseBuildStatusToString(CoarseBuildStatus status) noexcept;

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__MULTIRESOLUTION_MAP_H_
