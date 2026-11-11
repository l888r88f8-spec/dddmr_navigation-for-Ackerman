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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__DISTANCE_FIELD_H_
#define DDDMR_GRID_GLOBAL_PLANNER__DISTANCE_FIELD_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/map_snapshot.h"
#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// One per-node obstacle/distance record. The attributes are copied from the
// immutable layers; the reported distance is the bounded same-surface metric
// min(nearest_observed_occupied_distance, truncation_distance). The metric is
// deliberately local: unknown influence is propagated only within the
// truncation radius, never across a whole connected component. A truncated
// value is a lower bound derived from the local search radius, not a claim that
// an obstacle exists at exactly that distance.
struct ObstacleDistanceRecord
{
  ObstacleAttributes attributes{};

  // Compatibility mirror of nearest_observed_distance_m. New code should read
  // nearest_observed_distance_m directly.
  double distance_m{std::numeric_limits<double>::quiet_NaN()};

  // Compatibility mirror of nearest_known. New code should read nearest_known
  // directly.
  bool distance_known{false};

  // Bounded metric min(nearest observed occupied distance, truncation). For a
  // supported clear node with no occupied node inside the radius this is the
  // explicit truncation lower bound, not NaN. NaN only when the node itself has
  // unknown terrain/obstacle evidence.
  double nearest_observed_distance_m{
    std::numeric_limits<double>::quiet_NaN()};

  // True when a bounded metric (exact or truncated lower bound) is reported.
  bool nearest_known{false};

  // True when nearest_observed_distance_m is the truncation cap rather than an
  // exact observed distance. The cap is an explicit lower bound and must not be
  // read as a measured obstacle distance.
  bool truncated{false};

  // True when the reported bounded clearance is not contradicted by an unknown
  // boundary inside that radius. For a clear node this is local evidence, not a
  // global component-wide guarantee; a distant truncated obstacle does not
  // invalidate it.
  bool clearance_guaranteed{false};

  // True when any connected neighbor is unknown terrain or has unknown obstacle
  // attributes. Diagnostic only; use local_unknown_influence for the bounded
  // radius decision.
  bool adjacent_unknown{false};

  // True when this node itself is unknown terrain, has unknown obstacle
  // attributes, or borders a missing interior cell. This is the local seed used
  // by the bounded unknown-influence propagation.
  bool unknown_boundary{false};

  // Same-surface distance to the nearest unknown boundary, capped at the
  // truncation distance. Infinity means no unknown boundary was reached inside
  // the radius.
  double unknown_boundary_distance_m{
    std::numeric_limits<double>::infinity()};

  // True when an unknown boundary lies inside the truncation radius. This is
  // the only unknown-influence flag collision checks should consult; it is
  // bounded and never propagated across the whole connected component.
  bool local_unknown_influence{false};
};

// Fixed record-page size in fine cells. Tile keys are floor divisions of the
// original fine cell index, so negative cells remain stable and do not depend on
// the map origin.
inline constexpr std::int64_t kObstacleRecordTileCells = 32;

// One fixed-XY page key for the copy-on-write distance-record store.
struct ObstacleRecordTileKey
{
  std::int64_t x{0};
  std::int64_t y{0};

  // Exact equality of both page indices.
  bool operator==(const ObstacleRecordTileKey & other) const noexcept
  {
    return x == other.x && y == other.y;
  }

  // Deterministic ordering for the immutable page directory.
  bool operator<(const ObstacleRecordTileKey & other) const noexcept
  {
    return x < other.x || (x == other.x && y < other.y);
  }
};

// Immutable record page. The page is shared by pointer across field revisions;
// a page's records are never mutated after publication.
struct ObstacleRecordTile
{
  std::unordered_map<TerrainNodeId, ObstacleDistanceRecord> records;
};

// Immutable ordered page directory. Copying the directory is O(number of pages)
// and copies only shared_ptrs, never individual node records.
using ObstacleRecordTileMap =
  std::map<ObstacleRecordTileKey, std::shared_ptr<const ObstacleRecordTile>>;

// Shared node-id -> page-key geometry index. It is built once per terrain
// pointer and reused verbatim by every incremental field for that terrain.
using ObstacleRecordIdIndex =
  std::unordered_map<TerrainNodeId, ObstacleRecordTileKey>;

// Immutable fixed-snapshot obstacle map and bounded same-surface truncated
// distance field. The metric is min(nearest observed occupied distance,
// truncation_distance_m); unknown influence is similarly bounded to the same
// radius. Construction for a full build does all work once; query methods take
// no locks and do not mutate shared state. UpdateIncremental copies only the
// page directory and clones the fixed record pages touched by the bounded halo,
// so records outside those pages remain pointer-identical.
class ObstacleDistanceField
{
public:
  ObstacleDistanceField() = default;

  // Builds the field from an immutable terrain snapshot and independent static
  // and dynamic layers. Invalid options produce a non-ready field with a
  // diagnostic instead of silently enabling an unsafe policy.
  ObstacleDistanceField(
    std::shared_ptr<const TerrainSnapshot> terrain,
    ObstacleLayer static_obstacles,
    ObstacleLayer dynamic_obstacles,
    ObstacleDistanceOptions options);

  // Convenience builder from a pinned map snapshot.
  static ObstacleDistanceField Build(
    const MapSnapshot & map,
    ObstacleDistanceOptions options);

  // Bounded incremental rebuild. Returns a new immutable field sharing the same
  // terrain pointer/options as `previous` when possible, recomputing only the
  // truncation-radius halo around `changed_cells`. The page directory is copied,
  // but only pages touched by the halo are cloned; all other pages (and their
  // records) remain pointer-identical. Boundary seeds are taken from the
  // unchanged outside records, so a deletion/add/move/TTL change produces the
  // same result as a full build inside the halo while outside records remain
  // valid. A null, not-ready, different-terrain, or differently-configured
  // previous field (and any empty list whose layer versions changed) falls back
  // to a full build. The returned pointer is never null.
  static std::shared_ptr<const ObstacleDistanceField> UpdateIncremental(
    std::shared_ptr<const ObstacleDistanceField> previous,
    std::shared_ptr<const TerrainSnapshot> terrain,
    ObstacleLayer static_obstacles, ObstacleLayer dynamic_obstacles,
    const std::vector<FineCellIndex> & changed_cells,
    ObstacleDistanceOptions options);

  bool ready() const noexcept {return ready_;}
  const std::string & validation_detail() const noexcept
  {
    return validation_detail_;
  }

  const std::shared_ptr<const TerrainSnapshot> & terrain() const noexcept
  {
    return terrain_;
  }
  const ObstacleLayer & static_obstacles() const noexcept
  {
    return static_obstacles_;
  }
  const ObstacleLayer & dynamic_obstacles() const noexcept
  {
    return dynamic_obstacles_;
  }
  const ObstacleDistanceOptions & options() const noexcept {return options_;}
  double truncation_distance_m() const noexcept
  {
    return options_.truncation_distance_m;
  }

  // Merged obstacle attributes for one node. Missing layer records remain
  // unknown.
  ObstacleAttributes MergedObstacle(TerrainNodeId id) const;

  // Conservative merged state for one node.
  ObstacleState State(TerrainNodeId id) const;

  // True when the node has a supported terrain surface and an obstacle state
  // permitted by the configured unknown/prohibited policy. A known distance is
  // not required: an all-clear map with no observed obstacle is traversable.
  bool IsTraversable(TerrainNodeId id) const noexcept;

  // True when a bounded nearest metric is known, including a truncated lower
  // bound. A clear supported node inside the observed radius is usable even when
  // the nearest occupied node is farther than the truncation distance.
  bool IsDistanceKnown(TerrainNodeId id) const noexcept;

  // Bounded metric min(nearest observed occupied distance, truncation).
  // Returns the truncation distance when no occupied node is reachable inside
  // the radius; IsTruncated() is then true. NaN only when the node has no
  // supported/known evidence.
  double DistanceM(TerrainNodeId id) const noexcept;

  // True when the reported nearest distance is the explicit truncation cap
  // (a lower bound), not an exact observed distance.
  bool IsTruncated(TerrainNodeId id) const noexcept;

  // True when the reported bounded clearance is not contradicted by an unknown
  // boundary inside that radius. A distant truncated obstacle never removes a
  // local guarantee; a local unknown boundary always does.
  bool HasGuaranteedClearance(TerrainNodeId id) const noexcept;

  // Returns the bounded clearance in metres, or NaN unless
  // HasGuaranteedClearance(id) is true. For a truncated record this returns the
  // truncation distance (an explicit lower bound), so callers must also inspect
  // IsTruncated() when they need to distinguish exact from bounded evidence.
  double GuaranteedClearanceM(TerrainNodeId id) const noexcept;

  // True when any connected neighbor is unknown terrain or has unknown obstacle
  // attributes. This is a one-cell diagnostic; use HasLocalUnknownInfluence for
  // the truncation-radius decision.
  bool NearUnobservedBoundary(TerrainNodeId id) const noexcept;

  // True when an unknown terrain/obstacle boundary or missing interior cell is
  // reachable within the truncation radius. This is the bounded local-unknown
  // test used by collision and path validation.
  bool HasLocalUnknownInfluence(TerrainNodeId id) const noexcept;

  // Returns the full record, or nullptr for an unknown node ID. The pointer is
  // stable while this field lives because every record page is immutable.
  const ObstacleDistanceRecord * FindRecord(TerrainNodeId id) const noexcept;

  // Returns the immutable record page for one fixed page key, or an empty
  // shared_ptr when the page is absent. The returned reference is stable while
  // this field lives and takes no reference count, so a commit predicate can
  // compare page identity for a bounded set of corridor pages in O(1) each.
  const std::shared_ptr<const ObstacleRecordTile> & FindRecordTile(
    ObstacleRecordTileKey key) const noexcept;

private:
  std::shared_ptr<const TerrainSnapshot> terrain_;
  ObstacleLayer static_obstacles_;
  ObstacleLayer dynamic_obstacles_;
  ObstacleDistanceOptions options_;
  // Immutable record pages. Only pages intersecting the incremental halo are
  // cloned; every other page shared_ptr is copied from the previous field.
  ObstacleRecordTileMap records_;
  // Shared id -> page-key index for the terrain. It is built once per terrain
  // pointer and shared by all incremental fields for that terrain.
  std::shared_ptr<const ObstacleRecordIdIndex> record_id_index_;
  // Id-keyed connected-terrain adjacency reused by incremental halo recompute.
  // Shared because it is immutable and identical for the same terrain pointer.
  std::shared_ptr<const std::unordered_map<
      TerrainNodeId, std::vector<std::pair<TerrainNodeId, double>>>>
  adjacency_;
  std::shared_ptr<const std::unordered_map<TerrainNodeId, std::size_t>>
  id_to_index_;
  // Observed interior bounding box used to detect missing/unknown cells.
  std::int64_t min_cell_x_{0};
  std::int64_t max_cell_x_{0};
  std::int64_t min_cell_y_{0};
  std::int64_t max_cell_y_{0};
  bool ready_{false};
  std::string validation_detail_;
};

// Independent reference implementation used by tests. It relaxes all connected
// edges repeatedly (Bellman-Ford style) instead of using the production
// Dijkstra pass, so a shared bug is less likely to be self-confirming. It
// reports the bounded metric min(distance, truncation); clearance guarantees and
// unknown influence are intentionally not part of this reference output.
double ReferenceSameSurfaceDistance(
  const TerrainSnapshot & terrain,
  const ObstacleLayer & static_obstacles,
  const ObstacleLayer & dynamic_obstacles,
  TerrainNodeId target,
  const ObstacleDistanceOptions & options,
  bool * known, bool * truncated);

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__DISTANCE_FIELD_H_
