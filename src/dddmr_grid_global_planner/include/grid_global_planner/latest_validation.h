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

#ifndef GRID_GLOBAL_PLANNER__LATEST_VALIDATION_H_
#define GRID_GLOBAL_PLANNER__LATEST_VALIDATION_H_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/path_validator.h"
#include "grid_global_planner/wheel_support.h"

namespace grid_global_planner
{

// Logical corridor tile key used only for latest-map comparison. It is not a
// storage identity: the core snapshot exposes immutable obstacle layers, so the
// caller compares a conservative content fingerprint for each tile.
struct CorridorTileKey
{
  std::int64_t x{0};
  std::int64_t y{0};

  bool operator<(const CorridorTileKey & other) const noexcept
  {
    return x < other.x || (x == other.x && y < other.y);
  }

  bool operator==(const CorridorTileKey & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

// Why corridor coverage did not reach a complete relevant-tile token set.
// Budget exhaustion permits the bounded whole-snapshot fallback; invalid
// geometry or unconfigured body inputs remain hard failures.
enum class CorridorCoverageStatus : std::uint8_t
{
  kComplete = 0,
  kBudgetExhausted,
  kInvalidGeometry,
};

struct CorridorCoverage
{
  std::map<CorridorTileKey, std::uint64_t> tile_fingerprints;
  CorridorCoverageStatus status{CorridorCoverageStatus::kComplete};
  std::size_t sampled_points{0};
  std::size_t cells{0};

  bool complete() const noexcept
  {
    return status == CorridorCoverageStatus::kComplete;
  }
  bool truncated() const noexcept {return !complete();}
};

// Precomputed outside the final commit locks. Each entry is the actual
// immutable payload/page pointer for a corridor-affecting tile of the validated
// snapshot. Pointer identity is the conservative relevant-content token: an
// unrelated update keeps the pointer, while a copy-on-write touch of an
// affected tile changes it.
struct CorridorCommitTokens
{
  std::uint64_t terrain_revision{0};
  std::uint64_t terrain_hash{0};
  bool complete{false};
  std::map<ObstacleTileKey, const ObstacleTile *> static_tiles;
  std::map<ObstacleTileKey, const ObstacleTile *> dynamic_tiles;
  std::map<ObstacleRecordTileKey, const ObstacleRecordTile *> distance_tiles;
};

// Explicit latest-validation outcome. Budget exhaustion, cancellation and the
// overall deadline are distinct from "the map kept changing" so a bounded
// whole-snapshot validation failure never masquerades as instability.
enum class LatestValidationStatus : std::uint8_t
{
  kOk = 0,
  kNotReady,
  kStaleTerrain,
  kStaleSupportIds,
  kStaleContentHash,
  kInvalid,
  kInvalidConfiguration,
  kVehicleCapabilityUnconfigured,
  kBudgetExhausted,
  kCanceled,
  kDeadlineExceeded,
  kStaleMap,
};

const char * LatestValidationStatusToString(
  LatestValidationStatus status) noexcept;

// Maps the validation status to the stable result class used by the action
// diagnostics. It preserves the existing strings and never reports a map change
// for a budget/canceled/deadline outcome.
const char * LatestValidationResultClass(
  LatestValidationStatus status) noexcept;

struct LatestValidationResult
{
  bool ok{false};
  LatestValidationStatus status{LatestValidationStatus::kNotReady};
  std::string detail;
  std::vector<PathPose> path;
  // Exact immutable snapshot that produced `path`. Kept alive by this shared
  // pointer for the whole commit attempt.
  std::shared_ptr<const MapSnapshot> snapshot;
  // Exact immutable snapshot on which the selected path validator ran. A
  // later relevant-tile confirmation may advance `snapshot`, so whole-snapshot
  // proof requires this pointer to still equal `snapshot`.
  std::shared_ptr<const MapSnapshot> validated_snapshot;
  bool replayed{false};
  // True when the selected mode checked the entire path on this snapshot.
  bool whole_snapshot_validated{false};
  // Geometric paths use exact-snapshot proofs, without vehicle feasibility.
  bool geometric_graph_path{false};
  std::size_t validated_samples{0};
  std::size_t attempts{0};
};

// ROS-independent inputs copied from the node configuration. The core helper
// consumes exactly the same option types as search and PathValidator.
struct LatestValidationConfig
{
  VehicleCapabilityConfig vehicle;
  WheelSupportOptions wheel;
  ObstacleDistanceOptions obstacles;
  PathValidationOptions validation;
  double max_latest_validation_time_sec{2.0};
  int max_latest_validation_retries{4};
  double corridor_tile_size_m{3.2};
  std::size_t corridor_max_samples{50'000};
  double body_sweep_spacing_m{0.10};
};

// Computes the conservative relevant-corridor fingerprints. A budget truncation
// is reported explicitly instead of silently treating partial coverage as a
// complete unchanged corridor.
CorridorCoverage ComputeCorridorCoverage(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path,
  const LatestValidationConfig & config);

// Builds immutable token pointers for the corridor. Returns an incomplete token
// set when coverage is truncated; incomplete tokens must never be accepted.
CorridorCommitTokens BuildCorridorCommitTokens(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path,
  const LatestValidationConfig & config);

// True when the latest snapshot still has the exact same relevant tiles/pages
// and terrain identity as the token set.
bool CorridorCommitTokensMatch(
  const MapSnapshot & snapshot,
  const CorridorCommitTokens & tokens);

// Bounded latest-map validation. Normal short routes keep the relevant-tile
// fingerprint path and accept unrelated updates. When corridor coverage
// exhausts its work budget, this runs the existing PathValidator over the exact
// trajectory on one pinned immutable latest snapshot and returns a
// whole-snapshot proof. If that fallback itself exhausts budget, is canceled or
// misses the deadline, the explicit status says so. Invalid geometry, unknown
// obstacles, collisions and missing support evidence always fail closed.
LatestValidationResult ValidateLatestSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & searched_snapshot,
  const std::vector<PathPose> & searched_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel,
  const std::function<std::shared_ptr<const MapSnapshot>()> & acquire_latest);

// Geometric graph-path validation only; no vehicle kinematics or footprint.
LatestValidationResult ValidateLatestGraphSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & searched_snapshot,
  const std::vector<PathPose> & searched_path,
  const std::vector<TerrainNodeId> & node_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel,
  const std::function<std::shared_ptr<const MapSnapshot>()> & acquire_latest);

enum class LatestCommitProofKind : std::uint8_t
{
  kNone = 0,
  kRelevantTiles,
  kWholeSnapshot,
  // Production Grid policy: the request pinned immutable snapshot A before
  // search. A path validated on A is committed without comparing or
  // revalidating the store's current latest snapshot B.
  kPinnedSnapshot,
};

// Final commit proof. `snapshot` owns the validated immutable object and keeps
// every raw token pointer alive until the commit attempt completes.
struct LatestCommitProof
{
  LatestCommitProofKind kind{LatestCommitProofKind::kNone};
  CorridorCommitTokens tokens;
  std::shared_ptr<const MapSnapshot> snapshot;
};

// Builds the commit proof from a successful validation. Relevant tokens are
// preferred so unrelated updates remain allowed. If tokens are incomplete, a
// whole-snapshot proof is available only when the exact validated path was
// fully checked on that exact immutable snapshot.
LatestCommitProof BuildLatestCommitProof(
  const LatestValidationConfig & config,
  const LatestValidationResult & validation);

// Builds the production Grid commit proof from a successful validation on the
// request's pinned snapshot A. The proof keeps A alive through delivery and is
// deliberately independent of later publications of a newer map B.
LatestCommitProof BuildPinnedCommitProof(
  const LatestValidationResult & validation);

// Validates the searched path once on the exact pinned immutable snapshot A.
// No newer snapshot is acquired and no latest-map predicate is consulted. This
// is the only same-snapshot validation used by production Grid for the
// non-graph (hybrid_a_star) backend.
LatestValidationResult ValidatePinnedSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & pinned_snapshot,
  const std::vector<PathPose> & searched_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel);

// Compares a proof against the actual latest snapshot under the store handoff
// mutex. A whole-snapshot proof requires exact MapSnapshot object identity, not
// merely equal revision/content-hash/version fields. ABA is impossible because
// `proof.snapshot` keeps the validated object alive for the comparison. A
// pinned-snapshot proof intentionally ignores the latest object: the accepted
// plan is the immutable A that produced it.
bool LatestCommitProofMatches(
  const LatestCommitProof & proof,
  const MapSnapshot & latest);

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__LATEST_VALIDATION_H_
