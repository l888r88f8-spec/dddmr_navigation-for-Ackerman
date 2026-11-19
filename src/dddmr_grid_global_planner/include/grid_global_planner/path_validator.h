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

#ifndef GRID_GLOBAL_PLANNER__PATH_VALIDATOR_H_
#define GRID_GLOBAL_PLANNER__PATH_VALIDATOR_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "grid_global_planner/grid_search.h"
#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/wheel_support.h"

namespace grid_global_planner
{

// Explicit latest-map validation outcome. Stale reasons are distinct from a
// geometrically invalid path so the caller can decide to replan.
enum class PathValidationStatus : std::uint8_t
{
  kValid = 0,
  kInvalidPath,
  kInvalidOptions,
  kNotReady,
  kStaleTerrainRevision,
  kStaleContentHash,
  kMissingSupportNode,
  kChangedCorridor,
  kInsufficientSupport,
  kCollision,
  kUnknownObstacle,
  kBudgetExhausted,
  kCancelled,
  kVehicleCapabilityUnconfigured,
};

const char * PathValidationStatusToString(
  PathValidationStatus status) noexcept;

// Validation policy. max_samples bounds the full wheel/body sweep; it is a
// validation budget, not a claim that a short sample proves an unbounded path.
struct PathValidationOptions
{
  std::size_t max_samples{100'000};
  double max_wheel_sample_spacing_m{0.05};
  double max_yaw_sample_step_rad{0.10};
  double changed_pose_tolerance_m{0.03};
  // Tolerance for comparing the exact replayed segment endpoint position with
  // the stored next pose. A mismatch rejects the path as kInvalidPath instead of
  // validating a different trajectory.
  double segment_endpoint_tolerance_m{1.0e-3};
  // Tolerance for comparing the exact replayed segment endpoint yaw with the
  // stored next pose. A mismatch rejects the path as kInvalidPath.
  double segment_yaw_tolerance_rad{1.0e-3};
  // Deprecated compatibility flag retained so existing callers compile. Local
  // bounded clear evidence is always used; a truncated distance is a valid
  // lower bound and is not required to be an exact obstacle distance. Unknown
  // local influence is controlled by reject_unknown_obstacles.
  bool require_guaranteed_clearance{true};
  bool reject_unknown_obstacles{true};
};

struct PathValidationResult
{
  PathValidationStatus status{PathValidationStatus::kInvalidPath};
  std::string detail;
  std::size_t checked_samples{0};
  std::size_t first_failure_index{0};
  std::vector<PathPose> updated_path;
  double elapsed_ms{0.0};

  bool ok() const noexcept {return status == PathValidationStatus::kValid;}
};

// Validates an existing trajectory against the latest immutable MapSnapshot.
// It replays each segment from the recorded exact steering angle and signed arc
// length through the shared Ackermann integrator, checks that the replayed
// endpoint matches the stored next pose, runs the same four-wheel/body evaluator
// (which enforces explicit covered volume/headroom) and local bounded clear
// evidence, and reports whether corridor geometry changed. A changed terrain
// revision or content hash is reported as stale because node IDs are
// version-scoped. The validator never rebuilds the distance field; it binds the
// pinned field carried by the snapshot.
class PathValidator
{
public:
  PathValidator(
    VehicleCapabilityConfig vehicle,
    WheelSupportOptions wheel_options,
    ObstacleDistanceOptions distance_options,
    PathValidationOptions validation_options = {});

  const PathValidationOptions & options() const noexcept {return options_;}
  const WheelSupportEvaluator & evaluator() const noexcept {return evaluator_;}

  // path_terrain_revision/content_hash identify the snapshot that produced the
  // path. Node IDs are version-scoped: a changed revision or content hash is
  // rejected as stale rather than reusing IDs from another terrain version.
  PathValidationResult Validate(
    const MapSnapshot & latest,
    const std::vector<PathPose> & path,
    std::uint64_t path_terrain_revision,
    std::uint64_t path_content_hash,
    const SearchBudget & budget) const;

private:
  PathValidationOptions options_;
  ObstacleDistanceOptions distance_options_;
  WheelSupportEvaluator evaluator_;
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__PATH_VALIDATOR_H_
