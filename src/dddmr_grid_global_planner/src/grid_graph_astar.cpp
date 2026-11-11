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
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/grid_search.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Maps a shared budget status onto the terminal graph-search status.
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

// Resolves an explicit node ID, or one unambiguous node at a point. Ambiguous
// multi-layer points are rejected instead of silently choosing a deck.
const TerrainNode * ResolveNode(
  const TerrainSnapshot & terrain,
  TerrainNodeId node_id, const Point3 & point,
  bool use_points, bool nearest_layer)
{
  if (!use_points || node_id != kInvalidTerrainNodeId) {
    return terrain.FindNode(node_id);
  }
  if (!point.IsFinite()) {
    return nullptr;
  }
  const auto cell = FineCellOf(
    point, terrain.origin_x(), terrain.origin_y(),
    terrain.resolution());
  if (!cell.has_value()) {
    return nullptr;
  }
  const std::vector<TerrainNodeId> * nodes = terrain.FindNodesInCell(*cell);
  if (nodes == nullptr || nodes->empty() ||
    (!nearest_layer && nodes->size() != 1U))
  {
    return nullptr;
  }
  const TerrainNode * best = nullptr;
  double best_delta = kInfinity;
  for (const TerrainNodeId id : *nodes) {
    const TerrainNode * node = terrain.FindNode(id);
    if (node == nullptr) {
      continue;
    }
    const double delta = std::abs(node->representative.z - point.z);
    if (delta < best_delta ||
      (delta == best_delta && (best == nullptr || id < best->id)))
    {
      best = node;
      best_delta = delta;
    }
  }
  return best;
}

double NodeSlope(const TerrainNode & node) noexcept
{
  if (!node.plane.valid) {
    return 0.0;
  }
  const double normal_z =
    std::max(-1.0, std::min(1.0, node.plane.normal.z));
  return std::acos(normal_z);
}

}  // namespace

const char * SearchStatusToString(SearchStatus status) noexcept
{
  switch (status) {
    case SearchStatus::kSuccess:
      return "success";
    case SearchStatus::kNoRoute:
      return "no_route";
    case SearchStatus::kBudgetExhausted:
      return "budget_exhausted";
    case SearchStatus::kCancelled:
      return "cancelled";
    case SearchStatus::kInvalidRequest:
      return "invalid_request";
    case SearchStatus::kNotReady:
      return "not_ready";
    case SearchStatus::kVehicleCapabilityUnconfigured:
      return "vehicle_capability_unconfigured";
    case SearchStatus::kInternalError:
      return "internal_error";
  }
  return "unknown";
}

GridGraphAStar::GridGraphAStar(GraphSearchOptions options)
: options_(std::move(options)) {}

SearchResult GridGraphAStar::Search(
  const MapSnapshot & map, const ObstacleDistanceField & distance_field,
  const GraphSearchRequest & request) const
{
  const auto start_time = std::chrono::steady_clock::now();
  SearchResult result;
  if (!map.IsReady() || map.fine_terrain() == nullptr) {
    result.status = SearchStatus::kNotReady;
    result.detail = "map snapshot is not ready";
    return result;
  }
  if (!distance_field.ready() ||
    distance_field.terrain().get() != map.fine_terrain().get())
  {
    result.status = SearchStatus::kNotReady;
    result.detail = "distance field is not bound to the supplied snapshot";
    return result;
  }
  if (request.budget.max_expansions == 0 || request.budget.max_samples == 0) {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "search budgets must be positive";
    return result;
  }
  if (!std::isfinite(options_.obstacle_penalty_distance_m) ||
    options_.obstacle_penalty_distance_m < 0.0 ||
    !std::isfinite(options_.obstacle_penalty_weight) ||
    options_.obstacle_penalty_weight < 0.0 ||
    !std::isfinite(options_.slope_penalty_weight) ||
    options_.slope_penalty_weight < 0.0 ||
    !std::isfinite(options_.roughness_penalty_weight) ||
    options_.roughness_penalty_weight < 0.0)
  {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "graph search cost options are invalid";
    return result;
  }
  // The shared budget is checked before node resolution, index construction or
  // adjacency construction so cancellation/deadline covers all pre-work.
  BudgetTracker tracker(request.budget);
  const BudgetTracker::Status initial_budget = tracker.Check();
  if (initial_budget != BudgetTracker::Status::kOk) {
    result.status = ToSearchStatus(initial_budget);
    result.detail = "graph search budget expired before node resolution";
    return result;
  }
  const TerrainSnapshot & terrain = *map.fine_terrain();
  const TerrainNode * start =
    ResolveNode(
    terrain, request.start_node, request.start_point,
    request.use_points, request.nearest_layer);
  const TerrainNode * goal =
    ResolveNode(
    terrain, request.goal_node, request.goal_point,
    request.use_points, request.nearest_layer);
  // Two node resolutions were attempted, each a terrain-index lookup.
  result.stats.terrain_queries += 2;
  result.stats.map_queries += 2;
  if (start == nullptr || goal == nullptr) {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "start/goal node could not be resolved unambiguously";
    return result;
  }
  result.stats.obstacle_queries += 2;
  result.stats.map_queries += 2;
  if (!start->IsSupported() || !goal->IsSupported() ||
    !distance_field.IsTraversable(start->id) ||
    !distance_field.IsTraversable(goal->id))
  {
    result.status = SearchStatus::kNoRoute;
    result.detail = "start or goal is not supported/clear";
    return result;
  }

  result.status = SearchStatus::kNoRoute;
  result.detail = "graph search did not reach the goal";

  const std::vector<TerrainNode> & nodes = terrain.nodes();
  std::unordered_map<TerrainNodeId, std::size_t> id_to_index;
  id_to_index.reserve(nodes.size());
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    id_to_index.emplace(nodes[index].id, index);
  }
  const auto start_iterator = id_to_index.find(start->id);
  const auto goal_iterator = id_to_index.find(goal->id);
  if (start_iterator == id_to_index.end() ||
    goal_iterator == id_to_index.end())
  {
    result.status = SearchStatus::kInternalError;
    result.detail = "resolved node is missing from the index";
    return result;
  }
  const std::size_t start_index = start_iterator->second;
  const std::size_t goal_index = goal_iterator->second;

  const BudgetTracker::Status adjacency_budget = tracker.Check();
  if (adjacency_budget != BudgetTracker::Status::kOk) {
    result.status = ToSearchStatus(adjacency_budget);
    result.detail = "graph search budget expired before adjacency build";
    return result;
  }
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(
    nodes.size());
  for (const TerrainConnection & connection : terrain.connections()) {
    if (!connection.IsConnected()) {
      continue;
    }
    const auto source_iterator = id_to_index.find(connection.source);
    const auto target_iterator = id_to_index.find(connection.target);
    if (source_iterator == id_to_index.end() ||
      target_iterator == id_to_index.end())
    {
      continue;
    }
    const TerrainNode & source = nodes[source_iterator->second];
    const TerrainNode & target = nodes[target_iterator->second];
    result.stats.obstacle_queries += 2;
    result.stats.map_queries += 2;
    if (!source.IsSupported() || !target.IsSupported() ||
      !distance_field.IsTraversable(source.id) ||
      !distance_field.IsTraversable(target.id))
    {
      continue;
    }
    const double horizontal =
      std::isfinite(connection.horizontal_distance) ?
      std::abs(connection.horizontal_distance) :
      terrain.resolution();
    const double vertical =
      std::isfinite(connection.height_delta) ?
      std::abs(connection.height_delta) :
      0.0;
    const double distance = std::hypot(horizontal, vertical);
    const double slope = std::max(NodeSlope(source), NodeSlope(target));
    const double roughness =
      std::max(
      std::isfinite(source.plane.rms_residual) ?
      source.plane.rms_residual :
      0.0,
      std::isfinite(target.plane.rms_residual) ?
      target.plane.rms_residual :
      0.0);
    ++result.stats.distance_queries;
    ++result.stats.map_queries;
    const double obstacle_distance = distance_field.DistanceM(target.id);
    double obstacle_penalty = 0.0;
    if (std::isfinite(obstacle_distance) &&
      options_.obstacle_penalty_distance_m > 0.0 &&
      obstacle_distance < options_.obstacle_penalty_distance_m)
    {
      obstacle_penalty =
        options_.obstacle_penalty_weight *
        (options_.obstacle_penalty_distance_m - obstacle_distance) /
        options_.obstacle_penalty_distance_m;
    }
    const double cost =
      distance *
      (1.0 + options_.slope_penalty_weight * slope +
      options_.roughness_penalty_weight * roughness) +
      obstacle_penalty;
    adjacency[source_iterator->second].emplace_back(
      target_iterator->second,
      cost);
    adjacency[target_iterator->second].emplace_back(
      source_iterator->second,
      cost);
  }

  std::vector<double> g_score(nodes.size(), kInfinity);
  std::vector<std::size_t> parent(nodes.size(),
    std::numeric_limits<std::size_t>::max());
  std::vector<bool> closed(nodes.size(), false);
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  open;
  g_score[start_index] = 0.0;
  const auto heuristic = [&goal](const TerrainNode & node) {
      return std::sqrt(
        (node.representative.x - goal->representative.x) *
        (node.representative.x - goal->representative.x) +
        (node.representative.y - goal->representative.y) *
        (node.representative.y - goal->representative.y) +
        (node.representative.z - goal->representative.z) *
        (node.representative.z - goal->representative.z));
    };
  open.emplace(heuristic(*start), start_index);
  bool found = false;
  while (!open.empty()) {
    const BudgetTracker::Status expansion_budget =
      tracker.ConsumeExpansion();
    if (expansion_budget != BudgetTracker::Status::kOk) {
      result.status = ToSearchStatus(expansion_budget);
      result.detail = "graph search expansion budget expired";
      break;
    }
    const QueueEntry entry = open.top();
    open.pop();
    const std::size_t current = entry.second;
    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    ++result.stats.expanded_nodes;
    ++result.stats.unique_states;
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
        open.emplace(candidate + heuristic(nodes[edge.first]), edge.first);
      }
    }
  }
  if (!found) {
    if (result.status == SearchStatus::kInvalidRequest ||
      result.status == SearchStatus::kNotReady)
    {
      // Keep an earlier validation status.
    } else if (result.status == SearchStatus::kBudgetExhausted ||
      result.status == SearchStatus::kCancelled)
    {
      // Keep the explicit budget/cancel status.
    } else {
      result.status = SearchStatus::kNoRoute;
      result.detail = "graph A* exhausted all reachable supported states";
    }
  } else {
    std::vector<std::size_t> indices;
    std::size_t current = goal_index;
    while (true) {
      const BudgetTracker::Status chain_budget = tracker.Check();
      if (chain_budget != BudgetTracker::Status::kOk) {
        result.status = ToSearchStatus(chain_budget);
        result.detail =
          "graph route reconstruction budget expired";
        result.path.clear();
        result.node_path.clear();
        break;
      }
      indices.push_back(current);
      if (current == start_index) {
        break;
      }
      if (parent[current] == std::numeric_limits<std::size_t>::max()) {
        result.status = SearchStatus::kInternalError;
        result.detail = "graph parent chain is incomplete";
        result.path.clear();
        result.node_path.clear();
        break;
      }
      current = parent[current];
    }
    if (result.status != SearchStatus::kInternalError) {
      std::reverse(indices.begin(), indices.end());
      result.status = SearchStatus::kSuccess;
      result.detail = "graph route found";
      double distance = 0.0;
      for (std::size_t index = 0; index < indices.size(); ++index) {
        const TerrainNode & node = nodes[indices[index]];
        result.node_path.push_back(node.id);
        if (index > 0) {
          const TerrainNode & previous = nodes[indices[index - 1]];
          distance += std::sqrt(
            (node.representative.x - previous.representative.x) *
            (node.representative.x - previous.representative.x) +
            (node.representative.y - previous.representative.y) *
            (node.representative.y - previous.representative.y) +
            (node.representative.z - previous.representative.z) *
            (node.representative.z - previous.representative.z));
        }
        PathPose pose;
        pose.position = node.representative;
        // Graph identity only; the other slots are not wheel contact evidence.
        pose.support_nodes[0] = node.id;
        pose.cumulative_distance_m = distance;
        pose.cumulative_cost = g_score[indices[index]];
        result.path.push_back(pose);
      }
    }
  }
  // map_queries is maintained as the aggregate of the actual instrumented
  // terrain/obstacle/distance lookups. It is never a support-evaluation count.
  result.stats.graph_ms =
    std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time)
    .count();
  result.elapsed_ms = result.stats.graph_ms;
  return result;
}

}  // namespace dddmr_grid_global_planner
