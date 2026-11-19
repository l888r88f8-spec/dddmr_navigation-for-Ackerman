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

#ifndef GRID_GLOBAL_PLANNER__GRID_SEARCH_H_
#define GRID_GLOBAL_PLANNER__GRID_SEARCH_H_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "grid_global_planner/ackermann_kinematics.h"
#include "grid_global_planner/corridor_builder.h"
#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/multiresolution_map.h"
#include "grid_global_planner/terrain_types.h"
#include "grid_global_planner/wheel_support.h"

namespace grid_global_planner
{

// Terminal search status. Budget exhaustion is intentionally distinct from a
// proven discrete no-route result.
enum class SearchStatus : std::uint8_t
{
  kSuccess = 0,
  kNoRoute,
  kBudgetExhausted,
  kCancelled,
  kInvalidRequest,
  kNotReady,
  kVehicleCapabilityUnconfigured,
  kInternalError,
};

const char * SearchStatusToString(SearchStatus status) noexcept;

// Shared monotonic budget. The deadline is a steady-clock instant; cancellation
// is a caller-owned flag. Both are checked on every search iteration and sample.
struct SearchBudget
{
  bool has_deadline{false};
  std::chrono::steady_clock::time_point deadline{};
  std::shared_ptr<std::atomic<bool>> cancel;
  std::size_t max_samples{2'000'000};
};

// Shared monotonic budget tracker used by graph search, hybrid search,
// reconstruction and path validation. One instance covers every search
// iteration and every map/support query so a caller's deadline and
// cancellation flag apply to the whole request, not just the first stage.
// Status values keep cancellation, a proven spent budget and an invalid zero
// ceiling distinct.
class BudgetTracker
{
public:
  // Status returned by every check/consume method.
  enum class Status : std::uint8_t
  {
    kOk = 0,
    kExhausted,
    kCancelled,
    kInvalidBudget,
  };

  // Stores a reference to the caller-owned budget. The budget must outlive this
  // tracker.
  explicit BudgetTracker(const SearchBudget & budget) noexcept;

  // Checks cancellation, deadline and the remaining sample ceiling without
  // consuming anything. This is the required pre-work gate.
  Status Check() const noexcept;

  // Checks cancellation/deadline and whether additional sample slots remain,
  // without consuming them.
  Status CheckSamples(std::size_t additional) const noexcept;

  // Checks cancellation/deadline and consumes count sample slots. The
  // comparison and addition are overflow-safe.
  Status ConsumeSamples(std::size_t count) noexcept;

  // Returns the number of samples consumed so far.
  std::size_t samples() const noexcept {return samples_;}

  // Returns the remaining sample slots, or zero when none remain.
  std::size_t remaining_samples() const noexcept;

  // Returns the underlying request budget.
  const SearchBudget & budget() const noexcept {return budget_;}

  // Returns ceil(distance/spacing) clamped to max_count. Invalid input and
  // exact counts above max_count saturate at max_count; no intermediate value
  // is converted to size_t before it is proven to fit.
  static std::size_t SaturatingSampleCount(
    double distance_m,
    double spacing_m,
    std::size_t max_count) noexcept;

  // Computes ceil(distance/spacing) only when the exact count fits within
  // max_count. Returns false for invalid input or when the count would exceed
  // the supplied ceiling.
  static bool TrySampleCount(
    double distance_m, double spacing_m,
    std::size_t max_count,
    std::size_t * count) noexcept;

private:
  // Returns kInvalidBudget when the configured sample ceiling is zero.
  Status ValidateCeilings() const noexcept;

  const SearchBudget & budget_;
  std::size_t samples_{0};
};

// Resolves the pinned distance field returned by the snapshot. The parallel
// map-snapshot API may return a reference, pointer, shared_ptr or unique_ptr;
// this helper always yields a non-owning pointer while the caller keeps the
// returned holder alive.
inline const ObstacleDistanceField * ResolveDistanceFieldPointer(
  const ObstacleDistanceField & field) noexcept
{
  return &field;
}

// Pointer overload for a snapshot that returns a raw pointer.
inline const ObstacleDistanceField * ResolveDistanceFieldPointer(
  const ObstacleDistanceField * field) noexcept
{
  return field;
}

// Shared-ownership overload for a snapshot that returns shared_ptr.
template<typename T>
inline const ObstacleDistanceField * ResolveDistanceFieldPointer(
  const std::shared_ptr<T> & field) noexcept
{
  return field.get();
}

// Unique-ownership overload for a snapshot that returns unique_ptr.
template<typename T>
inline const ObstacleDistanceField * ResolveDistanceFieldPointer(
  const std::unique_ptr<T> & field) noexcept
{
  return field.get();
}

// One full trajectory pose. Position is the solved base_link origin. The four
// support node IDs are part of the state identity and are retained for latest-
// map validation.
struct PathPose
{
  Point3 position{};
  double heading{0.0};
  double roll{0.0};
  double pitch{0.0};
  std::array<TerrainNodeId, kWheelCount> support_nodes{};
  // +1 for forward motion and -1 for reverse motion on the segment that ends at
  // this pose. The first pose stores +1 because it has no incoming segment.
  double direction{1.0};
  // Exact central Ackermann steering angle used to integrate this pose from the
  // previous pose. The first pose stores zero. Reconstruction and validation
  // replay this value; they never infer steering from a chord or yaw delta.
  double steering_central_rad{0.0};
  // Signed rear-axle arc length from the previous pose to this pose in metres.
  // Negative values encode reverse motion and the first pose stores zero. The
  // value is the exact increment used by the shared Ackermann sampler.
  double signed_arc_length_m{0.0};
  double cumulative_distance_m{0.0};
  double cumulative_cost{0.0};
};

// BudgetTracker inline implementations. The tracker is header-only so every
// translation unit in the package shares exactly one implementation.
inline BudgetTracker::BudgetTracker(const SearchBudget & budget) noexcept
: budget_(budget) {}

inline BudgetTracker::Status BudgetTracker::ValidateCeilings() const noexcept
{
  if (budget_.max_samples == 0) {
    return Status::kInvalidBudget;
  }
  return Status::kOk;
}

inline BudgetTracker::Status BudgetTracker::Check() const noexcept
{
  if (budget_.cancel != nullptr &&
    budget_.cancel->load(std::memory_order_relaxed))
  {
    return Status::kCancelled;
  }
  if (budget_.has_deadline &&
    std::chrono::steady_clock::now() >= budget_.deadline)
  {
    return Status::kExhausted;
  }
  const Status ceilings = ValidateCeilings();
  if (ceilings != Status::kOk) {
    return ceilings;
  }
  if (samples_ >= budget_.max_samples) {
    return Status::kExhausted;
  }
  return Status::kOk;
}

inline BudgetTracker::Status BudgetTracker::CheckSamples(
  std::size_t additional) const noexcept
{
  if (budget_.cancel != nullptr &&
    budget_.cancel->load(std::memory_order_relaxed))
  {
    return Status::kCancelled;
  }
  if (budget_.has_deadline &&
    std::chrono::steady_clock::now() >= budget_.deadline)
  {
    return Status::kExhausted;
  }
  if (budget_.max_samples == 0) {
    return Status::kInvalidBudget;
  }
  const std::size_t remaining = remaining_samples();
  if (additional > remaining) {
    return Status::kExhausted;
  }
  return Status::kOk;
}

inline BudgetTracker::Status BudgetTracker::ConsumeSamples(
  std::size_t count) noexcept
{
  const Status status = CheckSamples(count);
  if (status != Status::kOk) {
    return status;
  }
  samples_ += count;
  return Status::kOk;
}

inline std::size_t BudgetTracker::remaining_samples() const noexcept
{
  return budget_.max_samples > samples_ ?
         budget_.max_samples - samples_ :
         0;
}

inline std::size_t BudgetTracker::SaturatingSampleCount(
  double distance_m, double spacing_m,
  std::size_t max_count) noexcept
{
  return SaturatingArcSampleCount(distance_m, spacing_m, max_count);
}

inline bool BudgetTracker::TrySampleCount(
  double distance_m,
  double spacing_m,
  std::size_t max_count,
  std::size_t * count) noexcept
{
  return TryArcSampleCount(distance_m, spacing_m, max_count, count);
}

struct SearchStats
{
  std::size_t expanded_nodes{0};
  std::size_t unique_states{0};
  std::size_t sampled_poses{0};
  // Number of four-wheel/body support evaluations. This is a task-level count,
  // not a map lookup count.
  std::size_t support_evaluations{0};
  // Aggregate of the instrumented terrain/obstacle/distance lookups below. It
  // is intentionally NOT incremented once per support evaluation: doing that
  // made the old field misleading. Use support_evaluations for that count.
  std::size_t map_queries{0};
  std::size_t coarse_routes{0};
  std::size_t corridor_runs{0};
  std::size_t goal_connection_attempts{0};
  // Actual instrumented lookups made by the search stage. Evaluator-internal
  // lookups are not individually counted, so these are explicit lower bounds.
  std::size_t terrain_queries{0};
  std::size_t obstacle_queries{0};
  std::size_t distance_queries{0};
  std::size_t cell_queries{0};
  std::size_t surface_queries{0};
  std::size_t neighbor_queries{0};
  // Graph topology cache activity. A build constructs the immutable adjacency
  // index for one terrain revision; a hit reuses it for another request.
  std::size_t graph_cache_builds{0};
  std::size_t graph_cache_hits{0};

  // Actual measured wall times. Nested values are labelled in comments:
  // fine_ms includes support evaluation and reconstruction; support_eval_ms is
  // the sum of evaluator calls and therefore must not be added to fine_ms.
  double graph_ms{0.0};
  double coarse_ms{0.0};
  double fine_ms{0.0};
  double fallback_ms{0.0};
  double reconstruction_ms{0.0};
  double support_eval_ms{0.0};
  double footprint_ms{0.0};
  double branch_solve_ms{0.0};
  double pose_solve_ms{0.0};
  double body_check_ms{0.0};
  std::size_t support_sample_count{0};
  std::size_t support_branch_combinations{0};
  // Rejection counts indexed by SupportRejection. This is an actual count of
  // evaluated wheel-support outcomes, not a confidence value.
  std::array<std::size_t,
    static_cast<std::size_t>(SupportRejection::kUnsupportedWheel) + 1>
  rejection_histogram{};
  // Detailed per-query latency histograms are OFF by default. When false the
  // query counts above remain instrumented lower bounds and latency is omitted.
  bool detailed_query_timing{false};
};

struct SearchResult
{
  SearchStatus status{SearchStatus::kInvalidRequest};
  std::string detail;
  std::vector<PathPose> path;
  std::vector<TerrainNodeId> node_path;
  bool coarse_used{false};
  bool fine_fallback{false};
  std::vector<std::string> stages;
  SearchStats stats;
  double elapsed_ms{0.0};
  // Metadata for the actual implementation, never a success claim. The search
  // algorithm is the same AckermannHybridAStar used by the live pipeline; the
  // traversability model names how that search accepted poses.
  std::string search_algorithm{"ackermann_hybrid_astar"};
  std::string traversability_model{"full_wheel"};

  bool ok() const noexcept {return status == SearchStatus::kSuccess;}
};

// Graph-A* options and request. This is the topology/candidate-route search; it
// does not replace the four-wheel fine feasibility check.
struct GraphSearchOptions
{
  double obstacle_penalty_distance_m{0.75};
  double obstacle_penalty_weight{2.0};
  double slope_penalty_weight{1.0};
  double roughness_penalty_weight{1.0};
  // Same-surface measured support-boundary soft cost, independent of both
  // obstacle distance and the wheel_support.* boundary profile. It is additive
  // per edge: edge.length * weight * max(0, 1 - d / range)^2, where d is the
  // minimum endpoint clearance in the immutable measured-support-edge cache
  // (equivalently maximum endpoint risk). Positive-infinity no-seed sentinel
  // contributes zero; insufficient-connection scan gaps are not a drop edge.
  // Zero range/weight disables exactly. Negative/non-finite values are rejected.
  // This is a preference, never a traversability veto.
  double boundary_penalty_distance_m{0.8};
  double boundary_penalty_weight{3.0};
};
struct GraphSearchRequest
{
  TerrainNodeId start_node{kInvalidTerrainNodeId};
  TerrainNodeId goal_node{kInvalidTerrainNodeId};
  Point3 start_point{};
  Point3 goal_point{};
  bool use_points{false};
  SearchBudget budget{};
  // Opt-in nearest ground layer by Z; legacy callers reject ambiguity.
  bool nearest_layer{false};
};

// Deterministic graph A* over fine terrain candidate connections. It requires
// supported/clear start and goal nodes and never traverses unknown or prohibited
// surfaces.
class GridGraphAStar
{
public:
  explicit GridGraphAStar(GraphSearchOptions options = {});

  const GraphSearchOptions & options() const noexcept {return options_;}

  // Prebuilds the immutable topology cache for the snapshot's fine terrain.
  // The cache is keyed by owning terrain identity and is bounded to the latest
  // revision. This is optional; Search builds the same cache on demand.
  SearchStatus Prepare(
    const MapSnapshot & map,
    const SearchBudget & budget,
    SearchStats * stats = nullptr) const;

  SearchResult Search(
    const MapSnapshot & map,
    const ObstacleDistanceField & distance_field,
    const GraphSearchRequest & request) const;

private:
  struct Impl;

  GraphSearchOptions options_;
  std::shared_ptr<Impl> state_;
};

// Ackermann Hybrid-A* options. Every budget/tolerance is an explicit search
// engineering value; vehicle capability values live in VehicleCapabilityConfig.
struct HybridSearchOptions
{
  // Bicycle primitive and steering sampling.
  double primitive_length_m{0.80};
  std::size_t steer_samples{5};
  bool allow_reverse{false};

  // Adaptive sample spacing. These bounds control the maximum wheel swept
  // distance, yaw change and fine-cell crossing between support evaluations.
  double max_wheel_sample_spacing_m{0.05};
  double max_yaw_sample_step_rad{0.10};

  // Goal connection.
  double goal_position_tolerance_m{0.30};
  double goal_heading_tolerance_rad{3.14};
  double goal_connection_max_length_m{3.0};
  std::size_t goal_connection_samples{12};

  // Coarse-to-fine guidance. Search never builds a coarse map: it only binds a
  // coarse summary already pinned in MapSnapshot. When no matching coarse map is
  // present the fine search runs directly.
  bool use_coarse_guidance{true};
  // Deprecated compatibility flag. It is kept so existing callers compile, but
  // it is intentionally false and ignored: coarse construction is not a search
  // responsibility and a coarse no-route is never authoritative.
  double coarse_resolution_m{0.40};
  double coarse_surface_z_tolerance_m{0.20};
  std::size_t max_coarse_routes{2};
  CorridorOptions corridor{};

  // Cost shaping.
  double reverse_penalty_weight{0.5};
  double steering_penalty_weight{0.1};
  double terrain_cost_weight{0.2};
  double heading_bin_rad{0.19634954084936207};  // pi/16
};

struct HybridSearchRequest
{
  Point3 start{};
  double start_heading{0.0};
  Point3 goal{};
  double goal_heading{0.0};
  bool has_goal_heading{false};
  SearchBudget budget{};
};

// Ackermann Hybrid A* over the fixed fine snapshot. It owns no map update lock,
// reuses one immutable snapshot, and uses the same four-wheel evaluator for
// primitive interiors, terminal goal connection and final trajectory output.
class AckermannHybridAStar
{
public:
  AckermannHybridAStar(
    VehicleCapabilityConfig vehicle,
    WheelSupportOptions wheel_options,
    ObstacleDistanceOptions distance_options,
    HybridSearchOptions search_options = {});

  const WheelSupportEvaluator & evaluator() const noexcept {return evaluator_;}
  const HybridSearchOptions & options() const noexcept {return options_;}

  // Runs coarse route/corridor search and bounded fine fallback under one total
  // deadline and sample budget.
  SearchResult Search(
    const MapSnapshot & map,
    const HybridSearchRequest & request) const;

private:
  WheelSupportEvaluator evaluator_;
  ObstacleDistanceOptions distance_options_;
  HybridSearchOptions options_;
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__GRID_SEARCH_H_
