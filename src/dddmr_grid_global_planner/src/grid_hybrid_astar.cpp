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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "grid_global_planner/ackermann_kinematics.h"
#include "grid_global_planner/grid_search.h"
#include "grid_global_planner/multiresolution_map.h"

namespace grid_global_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Defined below; the primitive expansion/replay helpers above it call this.
void RecordEvaluationTiming(
  SearchStats * stats,
  const WheelSupportEvaluation & evaluation,
  double elapsed_ms);

// Normalizes an angle into [-pi, pi).
double NormalizeAngle(double angle) noexcept
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  double normalized = std::fmod(angle + kPi, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized - kPi;
}

// Returns the absolute minimal angular difference between two headings.
double AngleDifference(double first, double second) noexcept
{
  return std::abs(NormalizeAngle(first - second));
}

// Maps a shared budget status onto the terminal search status.
SearchStatus ToSearchStatus(BudgetTracker::Status status) noexcept
{
  switch (status) {
    case BudgetTracker::Status::kOk:
      return SearchStatus::kSuccess;
    case BudgetTracker::Status::kCancelled:
      return SearchStatus::kCancelled;
    case BudgetTracker::Status::kExhausted:
    case BudgetTracker::Status::kInvalidBudget:
      return SearchStatus::kBudgetExhausted;
  }
  return SearchStatus::kInternalError;
}

// Builds the shared sampling policy from search options and the fine map
// resolution. Search, reconstruction and validation all use this policy shape.
AckermannSamplingOptions MakeArcSamplingOptions(
  const HybridSearchOptions & options, double resolution)
{
  AckermannSamplingOptions sampling;
  sampling.max_wheel_sample_spacing_m =
    options.max_wheel_sample_spacing_m;
  sampling.max_yaw_sample_step_rad =
    options.max_yaw_sample_step_rad;
  sampling.map_resolution_m = resolution;
  // One sample may advance at most half a fine cell; this prevents configured
  // widths from letting a sample skip support evidence.
  sampling.resolution_spacing_fraction = 0.5;
  return sampling;
}

// Four-wheel branch-aware search state key. The four support node IDs are kept
// explicitly; branches are never lossily combined into center+heading alone.
struct HybridStateKey
{
  std::int64_t x_bin{0};
  std::int64_t y_bin{0};
  std::int64_t heading_bin{0};
  std::array<TerrainNodeId, kWheelCount> branch{};

  bool operator==(const HybridStateKey & other) const noexcept
  {
    return x_bin == other.x_bin && y_bin == other.y_bin &&
           heading_bin == other.heading_bin && branch == other.branch;
  }
};

struct HybridStateKeyHash
{
  std::size_t operator()(const HybridStateKey & key) const noexcept
  {
    std::size_t hash = std::hash<std::int64_t>{}(key.x_bin);
    const auto mix = [&hash](std::size_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
      };
    mix(std::hash<std::int64_t>{}(key.y_bin));
    mix(std::hash<std::int64_t>{}(key.heading_bin));
    for (const TerrainNodeId id : key.branch) {
      mix(std::hash<TerrainNodeId>{}(id));
    }
    return hash;
  }
};

struct HybridNode
{
  Pose2D pose{};
  WheelSupportEvaluation evaluation{};
  std::size_t parent{std::numeric_limits<std::size_t>::max()};
  double g{0.0};
  double f{0.0};
  double steer{0.0};
  double arc_length{0.0};
  double direction{1.0};
  bool has_primitive{false};
  bool closed{false};
};

struct StageOutcome
{
  SearchStatus status{SearchStatus::kNoRoute};
  std::string detail;
  std::vector<PathPose> path;
  std::vector<TerrainNodeId> node_path;
};

struct CoarseOutcome
{
  SearchStatus status{SearchStatus::kNoRoute};
  std::string detail;
  std::vector<CoarseSurfaceId> route;
};

struct GoalSegment
{
  bool found{false};
  double steer{0.0};
  double arc_length{0.0};
  double direction{1.0};
  double cost{0.0};
  WheelSupportEvaluation evaluation{};
  Pose2D pose{};
};

struct ArcOutcome
{
  SearchStatus status{SearchStatus::kNoRoute};
  std::string detail;
  double cost{0.0};
  WheelSupportEvaluation evaluation{};
  Pose2D pose{};
};

// Builds the composite state key from a solved evaluation. Position/heading are
// binned, but the four branch IDs remain exact.
HybridStateKey MakeStateKey(
  const TerrainSnapshot & terrain,
  const WheelSupportQuery & query,
  const WheelSupportEvaluation & evaluation,
  double heading_bin_rad)
{
  HybridStateKey key;
  const double bin_size = std::max(0.01, 0.5 * terrain.resolution());
  key.x_bin = static_cast<std::int64_t>(
    std::floor(query.center.x / bin_size));
  key.y_bin = static_cast<std::int64_t>(
    std::floor(query.center.y / bin_size));
  const double heading = NormalizeAngle(query.heading);
  key.heading_bin = static_cast<std::int64_t>(
    std::floor((heading + kPi) / std::max(1.0e-6, heading_bin_rad)));
  key.branch = evaluation.branch_nodes;
  return key;
}

// Converts a full exact Ackermann pose to the planar pose used by the search
// state. z is carried by the support evaluation, not by the planar state.
Pose2D ToPose2D(const AckermannPose & pose) noexcept
{
  return Pose2D{pose.x, pose.y, pose.yaw};
}

// Builds one output trajectory pose with its exact segment replay metadata.
PathPose MakePathPose(
  const WheelSupportEvaluation & evaluation,
  const WheelSupportQuery & query, double direction,
  double steering_central_rad,
  double signed_arc_length_m,
  double cumulative_distance,
  double cumulative_cost)
{
  PathPose pose;
  pose.position = evaluation.pose.base_origin;
  pose.heading = NormalizeAngle(query.heading);
  pose.roll = evaluation.roll_rad;
  pose.pitch = evaluation.pitch_rad;
  pose.support_nodes = evaluation.branch_nodes;
  pose.direction = direction;
  pose.steering_central_rad = steering_central_rad;
  pose.signed_arc_length_m = signed_arc_length_m;
  pose.cumulative_distance_m = cumulative_distance;
  pose.cumulative_cost = cumulative_cost;
  return pose;
}

// Simulates one constant-curvature primitive with the shared exact Ackermann
// sampler and evaluates every interior sample with the four-wheel evaluator.
ArcOutcome SimulateArc(
  const TerrainSnapshot & terrain, const MapSnapshot & map,
  const ObstacleDistanceField & distance_field,
  const WheelSupportEvaluator & evaluator,
  const HybridSearchOptions & options, const AckermannGeometry & geometry,
  const Pose2D & start_pose, const WheelSupportEvaluation & start_evaluation,
  double steer, double signed_length, const CorridorView * corridor,
  BudgetTracker * budget, SearchStats * stats, bool collect_path,
  std::vector<PathPose> * path, double start_distance, double start_cost)
{
  ArcOutcome outcome;
  outcome.status = SearchStatus::kNoRoute;
  const AckermannPose start_base_pose{
    start_pose.x, start_pose.y,
    start_evaluation.pose.base_origin.z, start_pose.heading};
  AckermannArcSamples samples = SampleAckermannArc(
    geometry, evaluator.vehicle(), start_base_pose, steer, signed_length,
    MakeArcSamplingOptions(options, terrain.resolution()),
    budget->remaining_samples());
  if (!samples.ok()) {
    if (samples.status ==
      AckermannSampleStatus::kSampleBudgetExceeded)
    {
      outcome.status = SearchStatus::kBudgetExhausted;
    } else {
      outcome.status = SearchStatus::kInternalError;
    }
    outcome.detail = "primitive sampling failed: " + samples.detail;
    return outcome;
  }
  const BudgetTracker::Status consumed =
    budget->ConsumeSamples(samples.sample_count);
  if (consumed != BudgetTracker::Status::kOk) {
    outcome.status = ToSearchStatus(consumed);
    outcome.detail = "sample budget expired during primitive expansion";
    return outcome;
  }

  const double signed_step = samples.step_arc_length_m;
  const double direction = signed_length >= 0.0 ? 1.0 : -1.0;
  WheelSupportEvaluation previous = start_evaluation;
  double distance = start_distance;
  double cost = start_cost;
  for (std::size_t index = 0; index < samples.base_poses.size(); ++index) {
    const BudgetTracker::Status budget_status = budget->Check();
    if (budget_status != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(budget_status);
      outcome.detail = "primitive sampling budget expired";
      return outcome;
    }
    const AckermannPose & sample = samples.base_poses[index];
    WheelSupportQuery query;
    query.center = Point3{sample.x, sample.y,
      previous.pose.base_origin.z};
    query.heading = sample.yaw;
    query.steering_central_rad = steer;
    query.has_previous = true;
    query.previous = previous;
    const auto evaluation_start = std::chrono::steady_clock::now();
    const WheelSupportEvaluation evaluation =
      evaluator.Evaluate(map, distance_field, query);
    ++stats->sampled_poses;
    ++stats->support_evaluations;
    RecordEvaluationTiming(
      stats, evaluation,
      DurationToMs(std::chrono::steady_clock::now() - evaluation_start));
    if (!evaluation.feasible) {
      // A solve/branch budget is incompleteness, not proof that this primitive
      // is blocked. It must terminate the search as budget-exhausted so the
      // caller does not publish a proven no-route result.
      outcome.status =
        evaluation.rejection == SupportRejection::kPoseSolveBudget ?
        SearchStatus::kBudgetExhausted :
        SearchStatus::kNoRoute;
      outcome.detail = std::string("primitive rejected: ") +
        SupportRejectionToString(evaluation.rejection) +
        " (" + evaluation.detail + ")";
      return outcome;
    }
    if (corridor != nullptr) {
      for (const WheelSupport & wheel : evaluation.wheels) {
        if (!wheel.valid) {
          outcome.status = SearchStatus::kNoRoute;
          outcome.detail = "primitive produced an invalid wheel branch";
          return outcome;
        }
        const auto cell =
          FineCellOf(
          wheel.contact, terrain.origin_x(), terrain.origin_y(),
          terrain.resolution());
        if (!cell.has_value() || !corridor->ContainsFineCell(*cell)) {
          outcome.status = SearchStatus::kNoRoute;
          outcome.detail = "primitive leaves the corridor";
          return outcome;
        }
      }
    }
    distance += std::abs(signed_step);
    cost += std::abs(signed_step) *
      (1.0 + options.terrain_cost_weight * evaluation.cost);
    if (signed_length < 0.0) {
      cost += options.reverse_penalty_weight * std::abs(signed_step);
    }
    cost += options.steering_penalty_weight * std::abs(steer) *
      std::abs(signed_step);
    if (collect_path && path != nullptr) {
      path->push_back(
        MakePathPose(
          evaluation, query, direction, steer,
          signed_step, distance, cost));
    }
    previous = evaluation;
  }
  outcome.status = SearchStatus::kSuccess;
  outcome.cost = cost;
  outcome.evaluation = previous;
  outcome.pose = ToPose2D(samples.end_base_pose);
  return outcome;
}

// Appends the start pose to a reconstructed trajectory. It has no incoming
// segment, so its signed arc length and steering are both zero.
void AppendStartPose(
  const WheelSupportQuery & start_query,
  const WheelSupportEvaluation & start_evaluation,
  std::vector<PathPose> * full_path,
  std::vector<TerrainNodeId> * node_path)
{
  full_path->push_back(
    MakePathPose(
      start_evaluation, start_query, 1.0, 0.0,
      0.0, 0.0, 0.0));
  node_path->push_back(start_evaluation.branch_nodes[0]);
}

// Tries bounded exact terminal arcs to the goal. The same integrator and
// sampler as the search primitives are used; no chord interpolation is added.
GoalSegment TryGoalConnection(
  const TerrainSnapshot & terrain, const MapSnapshot & map,
  const ObstacleDistanceField & distance_field,
  const WheelSupportEvaluator & evaluator,
  const HybridSearchOptions & options, const AckermannGeometry & geometry,
  const Pose2D & start_pose, const WheelSupportEvaluation & start_evaluation,
  const Point3 & goal, bool has_goal_heading, double goal_heading,
  bool allow_reverse, const CorridorView * corridor, BudgetTracker * budget,
  SearchStats * stats, double start_distance, double start_cost,
  SearchStatus * failure_status, std::string * failure_detail)
{
  GoalSegment best;
  const double distance = std::hypot(
    goal.x - start_pose.x,
    goal.y - start_pose.y);
  if (distance > options.goal_connection_max_length_m) {
    return best;
  }
  const double max_length =
    std::min(
    options.goal_connection_max_length_m,
    std::max(
      options.primitive_length_m,
      distance * 1.5 + 0.5 * options.primitive_length_m));
  const std::size_t sample_count =
    std::max<std::size_t>(2, options.goal_connection_samples);
  const std::size_t steer_count = std::max<std::size_t>(2, options.steer_samples);
  const AckermannPose start_base_pose{
    start_pose.x, start_pose.y,
    start_evaluation.pose.base_origin.z, start_pose.heading};
  for (int direction_sign = 1; direction_sign >= (allow_reverse ? -1 : 1);
    direction_sign -= 2)
  {
    const double direction = static_cast<double>(direction_sign);
    for (std::size_t steer_index = 0; steer_index < steer_count;
      ++steer_index)
    {
      const double ratio =
        steer_count == 1 ?
        0.5 :
        static_cast<double>(steer_index) /
        static_cast<double>(steer_count - 1);
      const double steer =
        (2.0 * ratio - 1.0) *
        evaluator.vehicle().max_steer_central_rad;
      for (std::size_t sample = 1; sample <= sample_count; ++sample) {
        // Every candidate consumes one query slot and is checked against the
        // shared deadline/cancellation flag before any geometry work.
        const BudgetTracker::Status candidate_budget =
          budget->ConsumeSamples(1);
        if (candidate_budget != BudgetTracker::Status::kOk) {
          if (failure_status != nullptr) {
            *failure_status = ToSearchStatus(candidate_budget);
          }
          if (failure_detail != nullptr) {
            *failure_detail =
              "goal-connection budget expired before candidate evaluation";
          }
          return GoalSegment{};
        }
        const double length =
          max_length * static_cast<double>(sample) /
          static_cast<double>(sample_count);
        AckermannPose endpoint;
        std::string endpoint_detail;
        if (!IntegrateBaseArc(
            geometry, start_base_pose, steer,
            direction * length, &endpoint,
            &endpoint_detail))
        {
          if (failure_status != nullptr) {
            *failure_status = SearchStatus::kInternalError;
          }
          if (failure_detail != nullptr) {
            *failure_detail =
              "goal-connection endpoint integration failed: " +
              endpoint_detail;
          }
          return GoalSegment{};
        }
        if (std::hypot(goal.x - endpoint.x, goal.y - endpoint.y) >
          options.goal_position_tolerance_m)
        {
          continue;
        }
        if (has_goal_heading &&
          AngleDifference(endpoint.yaw, goal_heading) >
          options.goal_heading_tolerance_rad)
        {
          continue;
        }
        ++stats->goal_connection_attempts;
        const ArcOutcome arc = SimulateArc(
          terrain, map, distance_field, evaluator, options, geometry,
          start_pose, start_evaluation, steer, direction * length, corridor,
          budget, stats, false, nullptr, start_distance, start_cost);
        if (arc.status == SearchStatus::kBudgetExhausted ||
          arc.status == SearchStatus::kCancelled ||
          arc.status == SearchStatus::kInternalError)
        {
          if (failure_status != nullptr) {
            *failure_status = arc.status;
          }
          if (failure_detail != nullptr) {
            *failure_detail = arc.detail;
          }
          return GoalSegment{};
        }
        if (arc.status != SearchStatus::kSuccess) {
          continue;
        }
        if (!best.found || arc.cost < best.cost) {
          best.found = true;
          best.steer = steer;
          best.arc_length = direction * length;
          best.direction = direction;
          best.cost = arc.cost;
          best.evaluation = arc.evaluation;
          best.pose = arc.pose;
        }
      }
    }
  }
  return best;
}

// Reconstructs the full trajectory by replaying every accepted primitive with
// the recorded steering angle and signed arc length through the same exact
// integrator and sampler. No Euler x/y update is used anywhere.
StageOutcome ReconstructPath(
  const TerrainSnapshot & terrain, const MapSnapshot & map,
  const ObstacleDistanceField & distance_field,
  const WheelSupportEvaluator & evaluator,
  const HybridSearchOptions & options, const AckermannGeometry & geometry,
  const std::vector<HybridNode> & nodes, std::size_t goal_index,
  const WheelSupportEvaluation & start_evaluation,
  const WheelSupportQuery & start_query, const CorridorView * corridor,
  BudgetTracker * budget, SearchStats * stats)
{
  StageOutcome outcome;
  if (budget == nullptr) {
    outcome.status = SearchStatus::kInternalError;
    outcome.detail = "reconstruction budget tracker is null";
    return outcome;
  }
  const BudgetTracker::Status initial_budget = budget->Check();
  if (initial_budget != BudgetTracker::Status::kOk) {
    outcome.status = ToSearchStatus(initial_budget);
    outcome.detail = "budget expired before trajectory reconstruction";
    return outcome;
  }
  if (goal_index >= nodes.size()) {
    outcome.status = SearchStatus::kInternalError;
    outcome.detail = "hybrid goal index is outside the node array";
    return outcome;
  }
  std::vector<std::size_t> chain;
  std::size_t current = goal_index;
  while (true) {
    if (chain.size() > nodes.size()) {
      outcome.status = SearchStatus::kInternalError;
      outcome.detail = "hybrid parent chain exceeds the node count";
      return outcome;
    }
    chain.push_back(current);
    if (current == 0) {
      break;
    }
    if (nodes[current].parent == std::numeric_limits<std::size_t>::max() ||
      nodes[current].parent >= nodes.size())
    {
      outcome.status = SearchStatus::kInternalError;
      outcome.detail = "hybrid parent chain is incomplete";
      return outcome;
    }
    current = nodes[current].parent;
    const BudgetTracker::Status chain_budget = budget->Check();
    if (chain_budget != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(chain_budget);
      outcome.detail = "budget expired while walking the parent chain";
      return outcome;
    }
  }
  std::reverse(chain.begin(), chain.end());
  for (std::size_t index = 0; index < chain.size(); ++index) {
    const HybridNode & node = nodes[chain[index]];
    if (index == 0) {
      if (!node.has_primitive) {
        AppendStartPose(
          start_query, start_evaluation, &outcome.path,
          &outcome.node_path);
      }
      continue;
    }
    const BudgetTracker::Status segment_budget = budget->Check();
    if (segment_budget != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(segment_budget);
      outcome.detail = "budget expired before segment reconstruction";
      return outcome;
    }
    const HybridNode & parent = nodes[chain[index - 1]];
    const AckermannPose start_base_pose{
      parent.pose.x, parent.pose.y,
      parent.evaluation.pose.base_origin.z, parent.pose.heading};
    AckermannArcSamples samples = SampleAckermannArc(
      geometry, evaluator.vehicle(), start_base_pose, node.steer,
      node.arc_length,
      MakeArcSamplingOptions(options, terrain.resolution()),
      budget->remaining_samples());
    if (!samples.ok()) {
      outcome.status =
        samples.status == AckermannSampleStatus::kSampleBudgetExceeded ?
        SearchStatus::kBudgetExhausted :
        SearchStatus::kInternalError;
      outcome.detail = "segment replay sampling failed: " + samples.detail;
      return outcome;
    }
    const BudgetTracker::Status consumed =
      budget->ConsumeSamples(samples.sample_count);
    if (consumed != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(consumed);
      outcome.detail = "sample budget expired during segment replay";
      return outcome;
    }
    const double signed_step = samples.step_arc_length_m;
    WheelSupportEvaluation previous = parent.evaluation;
    for (std::size_t sample = 0; sample < samples.base_poses.size();
      ++sample)
    {
      const BudgetTracker::Status sample_budget = budget->Check();
      if (sample_budget != BudgetTracker::Status::kOk) {
        outcome.status = ToSearchStatus(sample_budget);
        outcome.detail = "budget expired during segment replay";
        return outcome;
      }
      const AckermannPose & sample_pose = samples.base_poses[sample];
      WheelSupportQuery query;
      query.center = Point3{sample_pose.x, sample_pose.y,
        previous.pose.base_origin.z};
      query.heading = sample_pose.yaw;
      query.steering_central_rad = node.steer;
      query.has_previous = true;
      query.previous = previous;
      const auto evaluation_start = std::chrono::steady_clock::now();
      const WheelSupportEvaluation evaluation =
        evaluator.Evaluate(map, distance_field, query);
      ++stats->support_evaluations;
      RecordEvaluationTiming(
        stats, evaluation,
        DurationToMs(std::chrono::steady_clock::now() - evaluation_start));
      if (!evaluation.feasible) {
        outcome.status =
          evaluation.rejection == SupportRejection::kPoseSolveBudget ?
          SearchStatus::kBudgetExhausted :
          SearchStatus::kInternalError;
        outcome.detail =
          "accepted primitive failed deterministic replay: " +
          evaluation.detail;
        return outcome;
      }
      const double cumulative_distance =
        outcome.path.empty() ?
        0.0 :
        outcome.path.back().cumulative_distance_m +
        std::abs(signed_step);
      double step_cost =
        std::abs(signed_step) *
        (1.0 + options.terrain_cost_weight * evaluation.cost);
      if (node.arc_length < 0.0) {
        step_cost +=
          options.reverse_penalty_weight * std::abs(signed_step);
      }
      step_cost += options.steering_penalty_weight * std::abs(node.steer) *
        std::abs(signed_step);
      const double cumulative_cost =
        outcome.path.empty() ?
        0.0 :
        outcome.path.back().cumulative_cost + step_cost;
      outcome.path.push_back(
        MakePathPose(
          evaluation, query, node.direction, node.steer, signed_step,
          cumulative_distance, cumulative_cost));
      outcome.node_path.push_back(evaluation.branch_nodes[0]);
      previous = evaluation;
    }
    const double endpoint_distance = std::hypot(
      samples.end_base_pose.x - node.pose.x,
      samples.end_base_pose.y - node.pose.y);
    if (endpoint_distance > 1.0e-6 ||
      AngleDifference(samples.end_base_pose.yaw, node.pose.heading) >
      1.0e-6)
    {
      outcome.status = SearchStatus::kInternalError;
      outcome.detail = "segment replay diverged from the recorded endpoint";
      return outcome;
    }
  }
  (void)corridor;
  outcome.status = SearchStatus::kSuccess;
  outcome.detail = "hybrid trajectory reconstructed";
  return outcome;
}

// Runs the fine Hybrid A* stage. A null corridor means the full fine map.
StageOutcome RunFineSearch(
  const TerrainSnapshot & terrain, const MapSnapshot & map,
  const ObstacleDistanceField & distance_field,
  const WheelSupportEvaluator & evaluator,
  const HybridSearchOptions & options, const AckermannGeometry & geometry,
  const HybridSearchRequest & request, const Pose2D & start_pose,
  const WheelSupportEvaluation & start_evaluation,
  const CorridorView * corridor, BudgetTracker * budget, SearchStats * stats)
{
  StageOutcome outcome;
  const WheelSupportQuery start_query{request.start, request.start_heading,
    0.0, false, {}};
  const double start_to_goal =
    std::hypot(
    request.goal.x - start_pose.x,
    request.goal.y - start_pose.y);
  if (start_to_goal <= options.goal_position_tolerance_m &&
    (!request.has_goal_heading ||
    AngleDifference(start_pose.heading, request.goal_heading) <=
    options.goal_heading_tolerance_rad))
  {
    AppendStartPose(
      start_query, start_evaluation, &outcome.path,
      &outcome.node_path);
    outcome.status = SearchStatus::kSuccess;
    outcome.detail = "start already satisfies the goal tolerance";
    return outcome;
  }

  std::vector<HybridNode> nodes;
  nodes.reserve(1024);
  HybridNode start_node;
  start_node.pose = start_pose;
  start_node.evaluation = start_evaluation;
  start_node.g = 0.0;
  start_node.f = start_to_goal;
  nodes.push_back(start_node);

  std::unordered_map<HybridStateKey, std::size_t, HybridStateKeyHash>
  best_index;
  const HybridStateKey start_key =
    MakeStateKey(
    terrain, start_query, start_evaluation,
    options.heading_bin_rad);
  best_index.emplace(start_key, 0);

  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  open;
  open.emplace(nodes[0].f, 0);
  std::size_t goal_index = std::numeric_limits<std::size_t>::max();
  const std::size_t steer_count =
    std::max<std::size_t>(2, options.steer_samples);

  while (!open.empty()) {
    const BudgetTracker::Status loop_budget =
      budget->Check();
    if (loop_budget != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(loop_budget);
      outcome.detail = "hybrid search budget expired";
      return outcome;
    }
    const QueueEntry entry = open.top();
    open.pop();
    const std::size_t current_index = entry.second;
    if (nodes[current_index].closed) {
      continue;
    }
    if (entry.first > nodes[current_index].f + 1.0e-12) {
      continue;
    }
    nodes[current_index].closed = true;
    const HybridNode current = nodes[current_index];
    ++stats->expanded_nodes;
    stats->unique_states = std::max(stats->unique_states, best_index.size());
    if (std::hypot(
        request.goal.x - current.pose.x,
        request.goal.y - current.pose.y) <=
      options.goal_position_tolerance_m &&
      (!request.has_goal_heading ||
      AngleDifference(current.pose.heading, request.goal_heading) <=
      options.goal_heading_tolerance_rad))
    {
      goal_index = current_index;
      break;
    }

    SearchStatus terminal_failure = SearchStatus::kNoRoute;
    std::string terminal_detail;
    const GoalSegment goal_segment = TryGoalConnection(
      terrain, map, distance_field, evaluator, options, geometry,
      current.pose, current.evaluation, request.goal,
      request.has_goal_heading, request.goal_heading,
      options.allow_reverse, corridor, budget, stats, current.g, current.g,
      &terminal_failure, &terminal_detail);
    if (terminal_failure == SearchStatus::kBudgetExhausted ||
      terminal_failure == SearchStatus::kCancelled ||
      terminal_failure == SearchStatus::kInternalError)
    {
      outcome.status = terminal_failure;
      outcome.detail = terminal_detail;
      return outcome;
    }
    if (goal_segment.found) {
      HybridNode goal_node;
      goal_node.pose = goal_segment.pose;
      goal_node.evaluation = goal_segment.evaluation;
      goal_node.parent = current_index;
      goal_node.g = goal_segment.cost;
      goal_node.f = goal_segment.cost;
      goal_node.steer = goal_segment.steer;
      goal_node.arc_length = goal_segment.arc_length;
      goal_node.direction = goal_segment.direction;
      goal_node.has_primitive = true;
      nodes.push_back(goal_node);
      goal_index = nodes.size() - 1;
      break;
    }

    for (int direction_sign = 1;
      direction_sign >= (options.allow_reverse ? -1 : 1);
      direction_sign -= 2)
    {
      const double direction = static_cast<double>(direction_sign);
      for (std::size_t steer_index = 0; steer_index < steer_count;
        ++steer_index)
      {
        const double ratio =
          static_cast<double>(steer_index) /
          static_cast<double>(steer_count - 1);
        const double steer =
          (2.0 * ratio - 1.0) *
          evaluator.vehicle().max_steer_central_rad;
        const double signed_length = direction * options.primitive_length_m;
        ArcOutcome arc = SimulateArc(
          terrain, map, distance_field, evaluator, options, geometry,
          current.pose, current.evaluation, steer, signed_length, corridor,
          budget, stats, false, nullptr, current.g, current.g);
        if (arc.status == SearchStatus::kBudgetExhausted ||
          arc.status == SearchStatus::kCancelled ||
          arc.status == SearchStatus::kInternalError)
        {
          outcome.status = arc.status;
          outcome.detail = arc.detail;
          return outcome;
        }
        if (arc.status != SearchStatus::kSuccess) {
          continue;
        }
        WheelSupportQuery endpoint_query;
        endpoint_query.center =
          Point3{arc.pose.x, arc.pose.y,
          arc.evaluation.pose.base_origin.z};
        endpoint_query.heading = arc.pose.heading;
        endpoint_query.steering_central_rad = steer;
        endpoint_query.has_previous = true;
        endpoint_query.previous = arc.evaluation;
        const HybridStateKey key =
          MakeStateKey(
          terrain, endpoint_query, arc.evaluation,
          options.heading_bin_rad);
        const auto existing = best_index.find(key);
        if (existing != best_index.end() &&
          nodes[existing->second].g <= arc.cost + 1.0e-12)
        {
          continue;
        }
        HybridNode node;
        node.pose = arc.pose;
        node.evaluation = arc.evaluation;
        node.parent = current_index;
        node.g = arc.cost;
        node.f = arc.cost +
          std::hypot(
          request.goal.x - arc.pose.x,
          request.goal.y - arc.pose.y);
        node.steer = steer;
        node.arc_length = signed_length;
        node.direction = direction;
        node.has_primitive = true;
        const std::size_t node_index = nodes.size();
        nodes.push_back(node);
        best_index[key] = node_index;
        open.emplace(node.f, node_index);
        if (std::hypot(
            request.goal.x - arc.pose.x,
            request.goal.y - arc.pose.y) <=
          options.goal_position_tolerance_m &&
          (!request.has_goal_heading ||
          AngleDifference(arc.pose.heading, request.goal_heading) <=
          options.goal_heading_tolerance_rad))
        {
          goal_index = node_index;
          break;
        }
      }
      if (goal_index != std::numeric_limits<std::size_t>::max()) {
        break;
      }
    }
    if (goal_index != std::numeric_limits<std::size_t>::max()) {
      break;
    }
  }

  if (goal_index == std::numeric_limits<std::size_t>::max()) {
    outcome.status = SearchStatus::kNoRoute;
    outcome.detail = "hybrid A* exhausted all reachable states";
    return outcome;
  }
  const auto reconstruction_start = std::chrono::steady_clock::now();
  StageOutcome reconstructed = ReconstructPath(
    terrain, map, distance_field, evaluator, options, geometry, nodes,
    goal_index, start_evaluation, start_query, corridor, budget, stats);
  stats->reconstruction_ms += DurationToMs(
    std::chrono::steady_clock::now() - reconstruction_start);
  return reconstructed;
}

// True when all fine nodes of a coarse summary are traversable. This is the
// conservative coarse guidance contract; it may close narrow passages, which is
// why coarse no-route never becomes final no-route. When the snapshot carries a
// matching coarse obstacle overlay, the precomputed summary is authoritative for
// this query; otherwise the legacy per-member fallback is used.
bool CoarseSurfaceBlocked(
  const CoarseSurfaceSummary & surface,
  const CoarseObstacleOverlay * overlay,
  const ObstacleDistanceField & distance_field,
  SearchStats * stats)
{
  if (!surface.IsSupported()) {
    return true;
  }
  if (overlay != nullptr) {
    ++stats->obstacle_queries;
    const CoarseObstacleSummary * summary = overlay->FindSummary(surface.id);
    if (summary == nullptr) {
      // Missing overlay evidence is unknown, not clear.
      return true;
    }
    return summary->has_occupied || summary->has_unknown ||
           summary->has_prohibited || !summary->clear;
  }
  for (const TerrainNodeId id : surface.fine_nodes) {
    ++stats->obstacle_queries;
    if (!distance_field.IsTraversable(id)) {
      return true;
    }
  }
  return false;
}

// Runs coarse graph A* for guidance only. It checks the shared deadline/sample
// budget per popped coarse surface and returns explicitly on cancellation/deadline.
CoarseOutcome RunCoarseAStar(
  const CoarseTerrainMap & coarse, const CoarseObstacleOverlay * overlay,
  const ObstacleDistanceField & distance_field, const Point3 & start,
  const Point3 & goal, const std::unordered_set<CoarseSurfaceId> & forbidden,
  std::size_t coarse_ratio, double obstacle_range, double obstacle_weight,
  BudgetTracker * budget, SearchStats * stats)
{
  CoarseOutcome outcome;
  const auto goal_cell = FineCellOf(
    goal, coarse.origin_x(), coarse.origin_y(),
    coarse.fine_resolution());
  const auto start_cell =
    FineCellOf(
    start, coarse.origin_x(), coarse.origin_y(),
    coarse.fine_resolution());
  if (!goal_cell.has_value() || !start_cell.has_value()) {
    outcome.status = SearchStatus::kInvalidRequest;
    outcome.detail = "coarse start/goal point is outside the map";
    return outcome;
  }
  const auto ratio = static_cast<std::int64_t>(coarse_ratio);
  const CoarseCellIndex start_coarse{FloorDivide(start_cell->x, ratio),
    FloorDivide(start_cell->y, ratio)};
  const CoarseCellIndex goal_coarse{FloorDivide(goal_cell->x, ratio),
    FloorDivide(goal_cell->y, ratio)};

  const BudgetTracker::Status prework_budget = budget->Check();
  if (prework_budget != BudgetTracker::Status::kOk) {
    outcome.status = ToSearchStatus(prework_budget);
    outcome.detail = "coarse search budget expired before graph construction";
    return outcome;
  }
  std::unordered_map<CoarseSurfaceId, std::size_t> id_to_index;
  id_to_index.reserve(coarse.surfaces().size());
  std::vector<char> blocked(coarse.surfaces().size(), 0);
  for (std::size_t index = 0; index < coarse.surfaces().size(); ++index) {
    id_to_index.emplace(coarse.surfaces()[index].id, index);
    blocked[index] =
      CoarseSurfaceBlocked(
      coarse.surfaces()[index], overlay, distance_field,
      stats) ?
      1 :
      0;
  }
  const auto choose_surface = [&](const CoarseCellIndex & cell,
      double target_z) -> CoarseSurfaceId {
      const std::vector<CoarseSurfaceId> * candidates =
        coarse.FindSurfacesInCell(cell);
      if (candidates == nullptr) {
        return kInvalidCoarseSurfaceId;
      }
      CoarseSurfaceId best = kInvalidCoarseSurfaceId;
      double best_score = kInfinity;
      for (const CoarseSurfaceId id : *candidates) {
        if (forbidden.find(id) != forbidden.end()) {
          continue;
        }
        const auto surface_iterator = id_to_index.find(id);
        if (surface_iterator == id_to_index.end() ||
          blocked[surface_iterator->second] != 0)
        {
          continue;
        }
        const CoarseSurfaceSummary * surface = coarse.FindSurface(id);
        if (surface == nullptr) {
          continue;
        }
        const double score =
          std::abs(surface->representative.z - target_z);
        if (score < best_score || (score == best_score && id < best)) {
          best = id;
          best_score = score;
        }
      }
      return best;
    };
  const CoarseSurfaceId start_surface =
    choose_surface(start_coarse, start.z);
  const CoarseSurfaceId goal_surface = choose_surface(goal_coarse, goal.z);
  if (start_surface == kInvalidCoarseSurfaceId ||
    goal_surface == kInvalidCoarseSurfaceId)
  {
    outcome.status = SearchStatus::kNoRoute;
    outcome.detail = "coarse start/goal surface is unavailable";
    return outcome;
  }
  const auto start_iterator = id_to_index.find(start_surface);
  const auto goal_iterator = id_to_index.find(goal_surface);
  if (start_iterator == id_to_index.end() ||
    goal_iterator == id_to_index.end())
  {
    outcome.status = SearchStatus::kInternalError;
    outcome.detail = "coarse surface index is incomplete";
    return outcome;
  }
  const std::size_t start_index = start_iterator->second;
  const std::size_t goal_index = goal_iterator->second;
  const CoarseSurfaceSummary & goal_summary =
    coarse.surfaces()[goal_index];

  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(
    coarse.surfaces().size());
  for (const CoarseTerrainConnection & connection : coarse.connections()) {
    const auto source_iterator = id_to_index.find(connection.source);
    const auto target_iterator = id_to_index.find(connection.target);
    if (source_iterator == id_to_index.end() ||
      target_iterator == id_to_index.end())
    {
      continue;
    }
    if (forbidden.find(connection.source) != forbidden.end() ||
      forbidden.find(connection.target) != forbidden.end())
    {
      continue;
    }
    if (blocked[source_iterator->second] != 0 ||
      blocked[target_iterator->second] != 0)
    {
      continue;
    }
    double cost = std::max(1.0e-9, connection.cost);
    ++stats->distance_queries;
    const double distance =
      distance_field.DistanceM(
      coarse.surfaces()[target_iterator->second].fine_nodes.front());
    if (std::isfinite(distance) && obstacle_range > 0.0 &&
      distance < obstacle_range)
    {
      cost += obstacle_weight * (obstacle_range - distance) / obstacle_range;
    }
    adjacency[source_iterator->second].emplace_back(
      target_iterator->second,
      cost);
    adjacency[target_iterator->second].emplace_back(
      source_iterator->second,
      cost);
  }

  std::vector<double> g_score(coarse.surfaces().size(), kInfinity);
  std::vector<std::size_t> parent(coarse.surfaces().size(),
    std::numeric_limits<std::size_t>::max());
  std::vector<bool> closed(coarse.surfaces().size(), false);
  const auto heuristic = [&](std::size_t index) {
      const Point3 & point = coarse.surfaces()[index].representative;
      return std::sqrt(
        (point.x - goal_summary.representative.x) *
        (point.x - goal_summary.representative.x) +
        (point.y - goal_summary.representative.y) *
        (point.y - goal_summary.representative.y) +
        (point.z - goal_summary.representative.z) *
        (point.z - goal_summary.representative.z));
    };
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  open;
  g_score[start_index] = 0.0;
  open.emplace(heuristic(start_index), start_index);
  bool found = false;
  while (!open.empty()) {
    const BudgetTracker::Status budget_status = budget->Check();
    if (budget_status != BudgetTracker::Status::kOk) {
      outcome.status = ToSearchStatus(budget_status);
      outcome.detail = "coarse search budget expired";
      return outcome;
    }
    const QueueEntry entry = open.top();
    open.pop();
    const std::size_t current = entry.second;
    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    ++stats->expanded_nodes;
    if (current == goal_index) {
      found = true;
      break;
    }
    for (const auto & edge : adjacency[current]) {
      if (closed[edge.first]) {
        continue;
      }
      const double candidate = g_score[current] + edge.second;
      if (candidate + 1.0e-12 < g_score[edge.first]) {
        g_score[edge.first] = candidate;
        parent[edge.first] = current;
        open.emplace(candidate + heuristic(edge.first), edge.first);
      }
    }
  }
  if (!found) {
    outcome.status = SearchStatus::kNoRoute;
    outcome.detail = "coarse guidance found no connected route";
    return outcome;
  }
  const auto coarse_reconstruction_start = std::chrono::steady_clock::now();
  std::size_t current = goal_index;
  std::vector<CoarseSurfaceId> reverse_route;
  while (true) {
    reverse_route.push_back(coarse.surfaces()[current].id);
    if (current == start_index) {
      break;
    }
    if (parent[current] == std::numeric_limits<std::size_t>::max()) {
      stats->reconstruction_ms += DurationToMs(
        std::chrono::steady_clock::now() - coarse_reconstruction_start);
      outcome.status = SearchStatus::kInternalError;
      outcome.detail = "coarse parent chain is incomplete";
      return outcome;
    }
    current = parent[current];
    const BudgetTracker::Status chain_budget = budget->Check();
    if (chain_budget != BudgetTracker::Status::kOk) {
      stats->reconstruction_ms += DurationToMs(
        std::chrono::steady_clock::now() - coarse_reconstruction_start);
      outcome.status = ToSearchStatus(chain_budget);
      outcome.detail = "coarse route reconstruction budget expired";
      return outcome;
    }
  }
  std::reverse(reverse_route.begin(), reverse_route.end());
  stats->reconstruction_ms += DurationToMs(
    std::chrono::steady_clock::now() - coarse_reconstruction_start);
  outcome.status = SearchStatus::kSuccess;
  outcome.detail = "coarse route found";
  outcome.route = std::move(reverse_route);
  return outcome;
}

// Accumulates the actual measured evaluator sections and the rejection class of
// one wheel-support evaluation. support_evaluations itself is incremented by
// the caller so this helper cannot double count.
void RecordEvaluationTiming(
  SearchStats * stats,
  const WheelSupportEvaluation & evaluation,
  double elapsed_ms)
{
  if (stats == nullptr) {
    return;
  }
  stats->support_eval_ms += elapsed_ms;
  stats->footprint_ms += evaluation.timing.footprint_ms;
  stats->branch_solve_ms += evaluation.timing.branch_solve_ms;
  stats->pose_solve_ms += evaluation.timing.pose_solve_ms;
  stats->body_check_ms += evaluation.timing.body_check_ms;
  stats->support_sample_count += evaluation.timing.sample_count;
  stats->support_branch_combinations +=
    evaluation.timing.branch_combinations;
  stats->cell_queries += evaluation.timing.cell_queries;
  stats->surface_queries += evaluation.timing.surface_queries;
  stats->neighbor_queries += evaluation.timing.neighbor_queries;
  stats->obstacle_queries += evaluation.timing.obstacle_queries;
  stats->distance_queries += evaluation.timing.distance_queries;
  const std::size_t rejection_index =
    static_cast<std::size_t>(evaluation.rejection);
  if (rejection_index < stats->rejection_histogram.size()) {
    ++stats->rejection_histogram[rejection_index];
  }
}

// Adds elapsed wall time to a terminal result.
SearchResult Finalize(
  SearchResult result,
  const std::chrono::steady_clock::time_point & start)
{
  // map_queries is the aggregate of actual instrumented lookups, never a
  // support-evaluation count. Evaluator-internal body-envelope lookups are not
  // individually instrumented, so this is an explicit lower bound. The
  // per-class fields remain separate so a report does not treat one evaluation
  // as one query.
  result.stats.map_queries = result.stats.terrain_queries +
    result.stats.cell_queries +
    result.stats.surface_queries +
    result.stats.neighbor_queries +
    result.stats.obstacle_queries +
    result.stats.distance_queries;
  result.elapsed_ms =
    std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start)
    .count();
  return result;
}

}  // namespace

AckermannHybridAStar::AckermannHybridAStar(
  VehicleCapabilityConfig vehicle, WheelSupportOptions wheel_options,
  ObstacleDistanceOptions distance_options, HybridSearchOptions search_options)
: evaluator_(std::move(vehicle), std::move(wheel_options)),
  distance_options_(std::move(distance_options)),
  options_(std::move(search_options)) {}

SearchResult AckermannHybridAStar::Search(
  const MapSnapshot & map, const HybridSearchRequest & request) const
{
  const auto start_time = std::chrono::steady_clock::now();
  SearchResult result;
  // Explicit metadata: the path comes from this AckermannHybridAStar instance
  // and the named traversability model, never from a straight-line fallback.
  result.search_algorithm = "ackermann_hybrid_astar";
  result.traversability_model =
    TraversabilityModelToString(evaluator_.model());
  if (!evaluator_.IsConfigured()) {
    result.status = SearchStatus::kVehicleCapabilityUnconfigured;
    result.detail = evaluator_.configuration_detail();
    return Finalize(std::move(result), start_time);
  }
  if (!map.IsReady() || map.fine_terrain() == nullptr) {
    result.status = SearchStatus::kNotReady;
    result.detail = "map snapshot is not ready";
    return Finalize(std::move(result), start_time);
  }
  // Bind the pinned distance field. Search never builds a field or coarse map.
  if (!map.HasReadyDistanceField()) {
    result.status = SearchStatus::kNotReady;
    result.detail = "map snapshot has no ready pinned distance field";
    return Finalize(std::move(result), start_time);
  }
  decltype(auto) distance_field_holder = map.distance_field();
  const ObstacleDistanceField * distance_field =
    ResolveDistanceFieldPointer(distance_field_holder);
  if (distance_field == nullptr || !distance_field->ready()) {
    result.status = SearchStatus::kNotReady;
    result.detail = "map snapshot distance field is null or not ready";
    return Finalize(std::move(result), start_time);
  }
  if (distance_field->terrain().get() != map.fine_terrain().get()) {
    result.status = SearchStatus::kNotReady;
    result.detail =
      "pinned distance field belongs to a different fine snapshot";
    return Finalize(std::move(result), start_time);
  }
  if (!request.start.IsFinite() || !request.goal.IsFinite() ||
    !std::isfinite(request.start_heading) ||
    (request.has_goal_heading && !std::isfinite(request.goal_heading)))
  {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "start/goal pose contains a non-finite value";
    return Finalize(std::move(result), start_time);
  }
  if (!IsSafeWorldCoordinate(request.start.x) ||
    !IsSafeWorldCoordinate(request.start.y) ||
    !IsSafeWorldCoordinate(request.goal.x) ||
    !IsSafeWorldCoordinate(request.goal.y))
  {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "start/goal point is outside the safe world bound";
    return Finalize(std::move(result), start_time);
  }
  if (request.budget.max_samples == 0) {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "search budgets must be positive";
    return Finalize(std::move(result), start_time);
  }
  if (!std::isfinite(options_.primitive_length_m) ||
    options_.primitive_length_m <= 0.0 || options_.steer_samples < 2 ||
    !std::isfinite(options_.max_wheel_sample_spacing_m) ||
    options_.max_wheel_sample_spacing_m <= 0.0 ||
    !std::isfinite(options_.max_yaw_sample_step_rad) ||
    options_.max_yaw_sample_step_rad <= 0.0 ||
    !std::isfinite(options_.goal_position_tolerance_m) ||
    options_.goal_position_tolerance_m < 0.0 ||
    !std::isfinite(options_.goal_heading_tolerance_rad) ||
    options_.goal_heading_tolerance_rad < 0.0 ||
    !std::isfinite(options_.goal_connection_max_length_m) ||
    options_.goal_connection_max_length_m <= 0.0 ||
    options_.goal_connection_samples < 2 ||
    !std::isfinite(options_.heading_bin_rad) ||
    options_.heading_bin_rad <= 0.0 || options_.heading_bin_rad > kPi ||
    !std::isfinite(options_.coarse_resolution_m) ||
    !IsValidResolution(options_.coarse_resolution_m) ||
    !std::isfinite(options_.coarse_surface_z_tolerance_m) ||
    options_.coarse_surface_z_tolerance_m <= 0.0 ||
    options_.max_coarse_routes == 0 ||
    !std::isfinite(options_.reverse_penalty_weight) ||
    options_.reverse_penalty_weight < 0.0 ||
    !std::isfinite(options_.steering_penalty_weight) ||
    options_.steering_penalty_weight < 0.0 ||
    !std::isfinite(options_.terrain_cost_weight) ||
    options_.terrain_cost_weight < 0.0)
  {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "hybrid search options are invalid";
    return Finalize(std::move(result), start_time);
  }
  CorridorBuilder corridor_builder(options_.corridor);
  if (!corridor_builder.ValidateOptions(nullptr)) {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "corridor options are invalid";
    return Finalize(std::move(result), start_time);
  }
  const AckermannGeometry geometry =
    MakeAckermannGeometry(evaluator_.vehicle());
  if (!geometry.valid) {
    result.status = SearchStatus::kInternalError;
    result.detail = "Ackermann geometry is invalid: " + geometry.detail;
    return Finalize(std::move(result), start_time);
  }

  const TerrainSnapshot & terrain = *map.fine_terrain();
  const Pose2D start_pose{request.start.x, request.start.y,
    request.start_heading};
  // The shared budget is created before the first pose/query. The start support
  // check below is itself a sampled query.
  BudgetTracker budget(request.budget);
  const BudgetTracker::Status start_budget = budget.ConsumeSamples(1);
  if (start_budget != BudgetTracker::Status::kOk) {
    result.status = ToSearchStatus(start_budget);
    result.detail = "budget expired before start support evaluation";
    return Finalize(std::move(result), start_time);
  }
  WheelSupportQuery start_query;
  start_query.center = request.start;
  start_query.heading = request.start_heading;
  start_query.steering_central_rad = 0.0;
  const auto start_evaluation_start = std::chrono::steady_clock::now();
  const WheelSupportEvaluation start_evaluation =
    evaluator_.Evaluate(map, *distance_field, start_query);
  ++result.stats.sampled_poses;
  ++result.stats.support_evaluations;
  RecordEvaluationTiming(
    &result.stats, start_evaluation,
    DurationToMs(
      std::chrono::steady_clock::now() -
      start_evaluation_start));
  if (!start_evaluation.feasible) {
    if (start_evaluation.rejection ==
      SupportRejection::kVehicleCapabilityUnconfigured)
    {
      result.status = SearchStatus::kVehicleCapabilityUnconfigured;
    } else if (start_evaluation.rejection ==
      SupportRejection::kPoseSolveBudget)
    {
      // The start support itself could not be solved within its deterministic
      // branch budget, so no reachable-state proof exists.
      result.status = SearchStatus::kBudgetExhausted;
    } else {
      result.status = SearchStatus::kNoRoute;
    }
    result.detail = std::string("start support rejected: ") +
      SupportRejectionToString(start_evaluation.rejection) +
      " (" + start_evaluation.detail + ")";
    return Finalize(std::move(result), start_time);
  }

  std::shared_ptr<const CoarseTerrainMap> coarse = map.coarse_terrain();
  const std::shared_ptr<const CoarseObstacleOverlay> coarse_obstacles =
    map.coarse_obstacle_overlay();
  bool coarse_attempted = false;
  const bool coarse_matches_fine =
    coarse != nullptr && map.CoarseMatchesFine() && coarse->IsUsable();
  if (options_.use_coarse_guidance && coarse_matches_fine) {
    coarse_attempted = true;
    std::unordered_set<CoarseSurfaceId> forbidden;
    std::vector<std::vector<CoarseSurfaceId>> routes;
    const std::int64_t ratio = coarse->resolution_ratio();
    const std::size_t max_attempts =
      options_.max_coarse_routes >
      std::numeric_limits<std::size_t>::max() / 2 ?
      std::numeric_limits<std::size_t>::max() :
      options_.max_coarse_routes * 2;
    for (std::size_t attempt = 0;
      attempt < max_attempts && routes.size() < options_.max_coarse_routes;
      ++attempt)
    {
      const auto coarse_start = std::chrono::steady_clock::now();
      const CoarseOutcome coarse_outcome = RunCoarseAStar(
        *coarse, coarse_obstacles.get(), *distance_field, request.start,
        request.goal, forbidden,
        static_cast<std::size_t>(std::max<std::int64_t>(1, ratio)), 0.0,
        0.0, &budget, &result.stats);
      result.stats.coarse_ms += DurationToMs(
        std::chrono::steady_clock::now() - coarse_start);
      if (coarse_outcome.status == SearchStatus::kBudgetExhausted ||
        coarse_outcome.status == SearchStatus::kCancelled ||
        coarse_outcome.status == SearchStatus::kInvalidRequest ||
        coarse_outcome.status == SearchStatus::kInternalError)
      {
        result.status = coarse_outcome.status;
        result.detail = coarse_outcome.detail;
        return Finalize(std::move(result), start_time);
      }
      if (coarse_outcome.status != SearchStatus::kSuccess) {
        // Coarse no-route is never authoritative: fall through to fine search.
        break;
      }
      bool duplicate = false;
      for (const auto & existing : routes) {
        if (existing == coarse_outcome.route) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        routes.push_back(coarse_outcome.route);
        ++result.stats.coarse_routes;
      }
      bool forbidden_added = false;
      for (std::size_t index = coarse_outcome.route.size() / 2;
        index + 1 < coarse_outcome.route.size(); ++index)
      {
        if (forbidden.insert(coarse_outcome.route[index]).second) {
          forbidden_added = true;
          break;
        }
      }
      if (!forbidden_added) {
        break;
      }
    }
    result.stages.push_back("coarse_routes=" + std::to_string(routes.size()));
    const std::vector<double> margins = corridor_builder.MarginSchedule();
    for (const auto & route : routes) {
      for (const double margin : margins) {
        const Corridor corridor = corridor_builder.BuildAroundCoarseRoute(
          terrain, *coarse, route, margin);
        if (corridor.cells.empty()) {
          continue;
        }
        CorridorView view(map.fine_terrain(), map.coarse_terrain(),
          corridor.cells);
        ++result.stats.corridor_runs;
        const auto fine_start = std::chrono::steady_clock::now();
        StageOutcome stage = RunFineSearch(
          terrain, map, *distance_field, evaluator_, options_, geometry,
          request, start_pose, start_evaluation, &view, &budget,
          &result.stats);
        result.stats.fine_ms += DurationToMs(
          std::chrono::steady_clock::now() - fine_start);
        if (stage.status == SearchStatus::kSuccess) {
          result.status = SearchStatus::kSuccess;
          result.detail = stage.detail;
          result.path = std::move(stage.path);
          result.node_path = std::move(stage.node_path);
          result.coarse_used = true;
          result.stages.push_back(
            "corridor_success_margin=" +
            std::to_string(margin));
          return Finalize(std::move(result), start_time);
        }
        if (stage.status == SearchStatus::kBudgetExhausted ||
          stage.status == SearchStatus::kCancelled ||
          stage.status == SearchStatus::kInternalError)
        {
          result.status = stage.status;
          result.detail = stage.detail;
          result.stages.push_back(
            "corridor_terminal=" +
            std::string(
              SearchStatusToString(
                stage.status)));
          return Finalize(std::move(result), start_time);
        }
        result.stages.push_back(
          "corridor_no_route_margin=" +
          std::to_string(margin));
      }
    }
    result.stages.push_back("coarse_exhausted_fallback_to_fine");
  }

  const auto fallback_start = std::chrono::steady_clock::now();
  StageOutcome fine_stage = RunFineSearch(
    terrain, map, *distance_field, evaluator_, options_, geometry, request,
    start_pose, start_evaluation, nullptr, &budget, &result.stats);
  const double full_fine_ms = DurationToMs(
    std::chrono::steady_clock::now() - fallback_start);
  result.stats.fine_ms += full_fine_ms;
  if (coarse_attempted) {
    result.stats.fallback_ms = full_fine_ms;
  }
  if (fine_stage.status == SearchStatus::kSuccess) {
    result.status = SearchStatus::kSuccess;
    result.detail = fine_stage.detail;
    result.path = std::move(fine_stage.path);
    result.node_path = std::move(fine_stage.node_path);
    result.fine_fallback = coarse_attempted;
    result.stages.push_back("full_fine_success");
    return Finalize(std::move(result), start_time);
  }
  result.status = fine_stage.status;
  result.detail = fine_stage.detail;
  result.stages.push_back(
    "full_fine_terminal=" +
    std::string(SearchStatusToString(fine_stage.status)));
  return Finalize(std::move(result), start_time);
}

}  // namespace grid_global_planner
