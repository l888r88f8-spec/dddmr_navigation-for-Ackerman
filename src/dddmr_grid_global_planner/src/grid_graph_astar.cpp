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
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dddmr_grid_global_planner/grid_search.h"

namespace dddmr_grid_global_planner
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr std::size_t kBudgetCheckInterval = 4096;

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

double NodeRoughness(const TerrainNode & node) noexcept
{
  return std::isfinite(node.plane.rms_residual) ?
         node.plane.rms_residual :
         0.0;
}

}  // namespace

// Per-planner cache state. The topology is immutable and keyed by owning
// TerrainSnapshot identity; the owning shared pointer prevents address reuse.
// Only one terrain revision is retained. Search workspaces are exclusively
// leased so concurrent searches never share mutable scratch storage.
struct GridGraphAStar::Impl
{
  struct Edge
  {
    // Adjacency target index in TerrainSnapshot::nodes() order.
    std::size_t target{0};
    // Canonical connection endpoints. The old undirected adjacency charged the
    // same cost in both directions, including the obstacle distance of the
    // original connection.target, so both are needed to reproduce it exactly.
    std::size_t canonical_source{0};
    std::size_t canonical_target{0};
    // Static geometric length, including the old finite-value fallbacks.
    double length{0.0};
  };

  struct Topology
  {
    std::shared_ptr<const TerrainSnapshot> terrain;
    std::unordered_map<TerrainNodeId, std::size_t> id_to_index;
    std::vector<std::size_t> offsets;
    std::vector<Edge> edges;
  };

  struct Workspace
  {
    using QueueEntry = std::pair<double, std::size_t>;

    std::vector<std::uint64_t> g_generation;
    std::vector<double> g_score;
    std::vector<std::size_t> parent;
    std::vector<std::uint64_t> closed_generation;
    // Populated only when the corresponding penalty weight is positive.
    std::vector<double> node_slope;
    std::vector<double> node_roughness;
    std::vector<QueueEntry> open;
    std::uint64_t generation{0};

    void Begin(std::size_t node_count)
    {
      if (generation == std::numeric_limits<std::uint64_t>::max()) {
        // Generation-stamp overflow: clear once, then start a fresh epoch.
        std::fill(g_generation.begin(), g_generation.end(), 0);
        std::fill(closed_generation.begin(), closed_generation.end(), 0);
        generation = 1;
      } else {
        ++generation;
      }
      if (g_generation.size() < node_count) {
        g_generation.resize(node_count, 0);
        g_score.resize(node_count);
        parent.resize(
          node_count, std::numeric_limits<std::size_t>::max());
        closed_generation.resize(node_count, 0);
      }
      open.clear();
    }

    bool HasG(std::size_t index) const noexcept
    {
      return g_generation[index] == generation;
    }

    bool IsClosed(std::size_t index) const noexcept
    {
      return closed_generation[index] == generation;
    }

    void SetG(
      std::size_t index, double score, std::size_t predecessor) noexcept
    {
      g_generation[index] = generation;
      g_score[index] = score;
      parent[index] = predecessor;
    }

    void Close(std::size_t index) noexcept
    {
      closed_generation[index] = generation;
    }
  };

  static constexpr std::size_t kMaxIdleWorkspaces = 4;

  mutable std::mutex mutex;
  // Latest terrain revision only. A shared pointer held here also keeps the
  // terrain alive, so a raw pointer cannot be recycled into a false cache hit.
  std::shared_ptr<const Topology> topology;
  std::vector<std::unique_ptr<Workspace>> idle_workspaces;

  void ReleaseWorkspace(std::unique_ptr<Workspace> workspace)
  {
    if (!workspace) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    if (idle_workspaces.size() < kMaxIdleWorkspaces) {
      idle_workspaces.push_back(std::move(workspace));
    }
  }

  struct WorkspaceReleaser
  {
    Impl * owner{nullptr};

    void operator()(Workspace * workspace) const noexcept
    {
      if (owner != nullptr && workspace != nullptr) {
        owner->ReleaseWorkspace(std::unique_ptr<Workspace>(workspace));
      }
    }
  };

  using WorkspaceHandle = std::unique_ptr<Workspace, WorkspaceReleaser>;

  WorkspaceHandle AcquireWorkspace()
  {
    std::unique_lock<std::mutex> lock(mutex);
    Workspace * raw = nullptr;
    if (!idle_workspaces.empty()) {
      raw = idle_workspaces.back().release();
      idle_workspaces.pop_back();
    } else {
      raw = new Workspace();
    }
    return WorkspaceHandle(raw, WorkspaceReleaser{this});
  }

  std::shared_ptr<const Topology> BuildTopology(
    const std::shared_ptr<const TerrainSnapshot> & terrain,
    BudgetTracker & tracker, SearchStatus * status, std::string * detail) const
  {
    const auto fail_budget =
      [status, detail](BudgetTracker::Status budget_status,
        const char * message) -> std::shared_ptr<const Topology>
      {
        *status = ToSearchStatus(budget_status);
        *detail = message;
        return nullptr;
      };

    auto built = std::make_shared<Topology>();
    built->terrain = terrain;
    const std::vector<TerrainNode> & nodes = terrain->nodes();
    const std::vector<TerrainConnection> & connections =
      terrain->connections();

    built->id_to_index.reserve(nodes.size());
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if ((index % kBudgetCheckInterval) == 0U) {
        const BudgetTracker::Status budget_status = tracker.Check();
        if (budget_status != BudgetTracker::Status::kOk) {
          return fail_budget(
            budget_status, "graph topology build budget expired");
        }
      }
      built->id_to_index.emplace(nodes[index].id, index);
    }

    // Count degrees in connection iteration order, exactly like the old
    // adjacency construction. Static supported-node rules are applied here;
    // live traversability is deliberately not cached.
    std::vector<std::size_t> degree(nodes.size(), 0);
    for (std::size_t connection_index = 0;
      connection_index < connections.size(); ++connection_index)
    {
      if ((connection_index % kBudgetCheckInterval) == 0U) {
        const BudgetTracker::Status budget_status = tracker.Check();
        if (budget_status != BudgetTracker::Status::kOk) {
          return fail_budget(
            budget_status, "graph topology build budget expired");
        }
      }
      const TerrainConnection & connection = connections[connection_index];
      if (!connection.IsConnected()) {
        continue;
      }
      const auto source_iterator =
        built->id_to_index.find(connection.source);
      const auto target_iterator =
        built->id_to_index.find(connection.target);
      if (source_iterator == built->id_to_index.end() ||
        target_iterator == built->id_to_index.end())
      {
        continue;
      }
      const std::size_t source = source_iterator->second;
      const std::size_t target = target_iterator->second;
      if (!nodes[source].IsSupported() || !nodes[target].IsSupported()) {
        continue;
      }
      ++degree[source];
      ++degree[target];
    }

    built->offsets.resize(nodes.size() + 1U);
    std::size_t total = 0;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if ((index % kBudgetCheckInterval) == 0U) {
        const BudgetTracker::Status budget_status = tracker.Check();
        if (budget_status != BudgetTracker::Status::kOk) {
          return fail_budget(
            budget_status, "graph topology build budget expired");
        }
      }
      if (degree[index] > std::numeric_limits<std::size_t>::max() - total) {
        *status = SearchStatus::kInternalError;
        *detail = "graph topology edge count overflow";
        return nullptr;
      }
      built->offsets[index] = total;
      total += degree[index];
    }
    built->offsets[nodes.size()] = total;
    built->edges.resize(total);

    std::vector<std::size_t> cursor(nodes.size(), 0);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      cursor[index] = built->offsets[index];
    }

    // Fill in the same connection order. Each accepted undirected connection
    // appends its forward edge and then its reverse edge, preserving parallel
    // duplicates and per-node neighbor ordering from the old implementation.
    for (std::size_t connection_index = 0;
      connection_index < connections.size(); ++connection_index)
    {
      if ((connection_index % kBudgetCheckInterval) == 0U) {
        const BudgetTracker::Status budget_status = tracker.Check();
        if (budget_status != BudgetTracker::Status::kOk) {
          return fail_budget(
            budget_status, "graph topology build budget expired");
        }
      }
      const TerrainConnection & connection = connections[connection_index];
      if (!connection.IsConnected()) {
        continue;
      }
      const auto source_iterator =
        built->id_to_index.find(connection.source);
      const auto target_iterator =
        built->id_to_index.find(connection.target);
      if (source_iterator == built->id_to_index.end() ||
        target_iterator == built->id_to_index.end())
      {
        continue;
      }
      const std::size_t source = source_iterator->second;
      const std::size_t target = target_iterator->second;
      if (!nodes[source].IsSupported() || !nodes[target].IsSupported()) {
        continue;
      }
      const double horizontal =
        std::isfinite(connection.horizontal_distance) ?
        std::abs(connection.horizontal_distance) :
        terrain->resolution();
      const double vertical =
        std::isfinite(connection.height_delta) ?
        std::abs(connection.height_delta) :
        0.0;
      const double length = std::hypot(horizontal, vertical);
      built->edges[cursor[source]++] = Edge{target, source, target, length};
      built->edges[cursor[target]++] = Edge{source, source, target, length};
    }

    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if (cursor[index] != built->offsets[index + 1U]) {
        *status = SearchStatus::kInternalError;
        *detail = "graph topology build is incomplete";
        return nullptr;
      }
    }
    const BudgetTracker::Status final_status = tracker.Check();
    if (final_status != BudgetTracker::Status::kOk) {
      return fail_budget(
        final_status, "graph topology build budget expired");
    }
    *status = SearchStatus::kSuccess;
    detail->clear();
    return built;
  }

  std::shared_ptr<const Topology> AcquireTopology(
    const std::shared_ptr<const TerrainSnapshot> & terrain,
    BudgetTracker & tracker, SearchStats & stats, SearchStatus * status,
    std::string * detail)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto ready = tracker.Check();
      if (ready != BudgetTracker::Status::kOk) {
        *status = ToSearchStatus(ready);
        *detail = "graph cache acquisition budget expired";
        return nullptr;
      }
      if (topology != nullptr && topology->terrain.get() == terrain.get()) {
        ++stats.graph_cache_hits;
        *status = SearchStatus::kSuccess;
        detail->clear();
        return topology;
      }
    }
    // Build outside the cache/pool mutex so existing searches can release their
    // workspaces and cancellation is never held behind another map's build.
    auto built = BuildTopology(terrain, tracker, status, detail);
    if (built == nullptr) {
      return nullptr;
    }
    ++stats.graph_cache_builds;
    std::shared_ptr<const Topology> retired;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (topology != nullptr && topology->terrain.get() == terrain.get()) {
        ++stats.graph_cache_hits;
        return topology;
      }
      retired = std::move(topology);
      topology = built;
    }
    // Any large retired terrain is released after unlocking.
    *status = SearchStatus::kSuccess;
    detail->clear();
    return built;
  }
};

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
: options_(std::move(options)), state_(std::make_shared<Impl>()) {}

SearchStatus GridGraphAStar::Prepare(
  const MapSnapshot & map, const SearchBudget & budget,
  SearchStats * stats) const
{
  if (!map.IsReady() || map.fine_terrain() == nullptr) {
    return SearchStatus::kNotReady;
  }
  if (budget.max_expansions == 0 || budget.max_samples == 0) {
    return SearchStatus::kInvalidRequest;
  }
  SearchStats local_stats;
  SearchStats & output_stats = stats != nullptr ? *stats : local_stats;
  BudgetTracker tracker(budget);
  const BudgetTracker::Status initial_budget = tracker.Check();
  if (initial_budget != BudgetTracker::Status::kOk) {
    return ToSearchStatus(initial_budget);
  }
  SearchStatus status = SearchStatus::kSuccess;
  std::string detail;
  if (state_->AcquireTopology(
      map.fine_terrain(), tracker, output_stats, &status, &detail) == nullptr)
  {
    return status;
  }
  auto workspace = state_->AcquireWorkspace();
  workspace->Begin(map.fine_terrain()->nodes().size());
  const auto final_budget = tracker.Check();
  return final_budget == BudgetTracker::Status::kOk ?
         SearchStatus::kSuccess : ToSearchStatus(final_budget);
}

SearchResult GridGraphAStar::Search(
  const MapSnapshot & map, const ObstacleDistanceField & distance_field,
  const GraphSearchRequest & request) const
{
  const auto start_time = std::chrono::steady_clock::now();
  SearchResult result;
  const auto finish = [&]() -> SearchResult {
      result.stats.graph_ms =
        std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_time)
        .count();
      result.elapsed_ms = result.stats.graph_ms;
      return std::move(result);
    };
  if (!map.IsReady() || map.fine_terrain() == nullptr) {
    result.status = SearchStatus::kNotReady;
    result.detail = "map snapshot is not ready";
    return finish();
  }
  if (!distance_field.ready() ||
    distance_field.terrain().get() != map.fine_terrain().get())
  {
    result.status = SearchStatus::kNotReady;
    result.detail = "distance field is not bound to the supplied snapshot";
    return finish();
  }
  if (request.budget.max_expansions == 0 || request.budget.max_samples == 0) {
    result.status = SearchStatus::kInvalidRequest;
    result.detail = "search budgets must be positive";
    return finish();
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
    return finish();
  }
  // The shared budget is checked before node resolution, index construction or
  // adjacency construction so cancellation/deadline covers all pre-work.
  BudgetTracker tracker(request.budget);
  const BudgetTracker::Status initial_budget = tracker.Check();
  if (initial_budget != BudgetTracker::Status::kOk) {
    result.status = ToSearchStatus(initial_budget);
    result.detail = "graph search budget expired before node resolution";
    return finish();
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
    return finish();
  }
  result.stats.obstacle_queries += 2;
  result.stats.map_queries += 2;
  if (!start->IsSupported() || !goal->IsSupported() ||
    !distance_field.IsTraversable(start->id) ||
    !distance_field.IsTraversable(goal->id))
  {
    result.status = SearchStatus::kNoRoute;
    result.detail = "start or goal is not supported/clear";
    return finish();
  }

  result.status = SearchStatus::kNoRoute;
  result.detail = "graph search did not reach the goal";

  SearchStatus cache_status = SearchStatus::kSuccess;
  std::string cache_detail;
  const std::shared_ptr<const Impl::Topology> topology =
    state_->AcquireTopology(
    map.fine_terrain(), tracker, result.stats, &cache_status, &cache_detail);
  if (topology == nullptr) {
    result.status = cache_status;
    result.detail = cache_detail;
    return finish();
  }
  const std::vector<TerrainNode> & nodes = terrain.nodes();
  const auto start_iterator = topology->id_to_index.find(start->id);
  const auto goal_iterator = topology->id_to_index.find(goal->id);
  if (start_iterator == topology->id_to_index.end() ||
    goal_iterator == topology->id_to_index.end())
  {
    result.status = SearchStatus::kInternalError;
    result.detail = "resolved node is missing from the index";
    return finish();
  }
  const std::size_t start_index = start_iterator->second;
  const std::size_t goal_index = goal_iterator->second;

  Impl::WorkspaceHandle workspace_handle = state_->AcquireWorkspace();
  if (!workspace_handle) {
    result.status = SearchStatus::kInternalError;
    result.detail = "graph search workspace allocation failed";
    return finish();
  }
  Impl::Workspace & workspace = *workspace_handle;
  workspace.Begin(nodes.size());

  // Slope/roughness are geometric node attributes, not obstacle state. They are
  // evaluated only when their weight is positive so the zero-weight production
  // path does no acos/rms work and performs no distance-field queries.
  const bool slope_enabled = options_.slope_penalty_weight > 0.0;
  const bool roughness_enabled = options_.roughness_penalty_weight > 0.0;
  if (slope_enabled) {
    workspace.node_slope.resize(nodes.size());
  }
  if (roughness_enabled) {
    workspace.node_roughness.resize(nodes.size());
  }
  if (slope_enabled || roughness_enabled) {
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if ((index % kBudgetCheckInterval) == 0U) {
        const BudgetTracker::Status budget_status = tracker.Check();
        if (budget_status != BudgetTracker::Status::kOk) {
          result.status = ToSearchStatus(budget_status);
          result.detail = "graph search budget expired before edge costs";
          return finish();
        }
      }
      if (slope_enabled) {
        workspace.node_slope[index] = NodeSlope(nodes[index]);
      }
      if (roughness_enabled) {
        workspace.node_roughness[index] = NodeRoughness(nodes[index]);
      }
    }
  }

  using QueueEntry = Impl::Workspace::QueueEntry;
  const auto greater = std::greater<QueueEntry>();
  const auto heuristic = [&goal](const TerrainNode & node) {
      return std::sqrt(
        (node.representative.x - goal->representative.x) *
        (node.representative.x - goal->representative.x) +
        (node.representative.y - goal->representative.y) *
        (node.representative.y - goal->representative.y) +
        (node.representative.z - goal->representative.z) *
        (node.representative.z - goal->representative.z));
    };
  workspace.SetG(
    start_index, 0.0, std::numeric_limits<std::size_t>::max());
  workspace.open.emplace_back(heuristic(nodes[start_index]), start_index);
  std::push_heap(
    workspace.open.begin(), workspace.open.end(), greater);

  bool found = false;
  const bool obstacle_penalty_enabled =
    options_.obstacle_penalty_distance_m > 0.0 &&
    options_.obstacle_penalty_weight > 0.0;
  while (!workspace.open.empty()) {
    const BudgetTracker::Status expansion_budget =
      tracker.ConsumeExpansion();
    if (expansion_budget != BudgetTracker::Status::kOk) {
      result.status = ToSearchStatus(expansion_budget);
      result.detail = "graph search expansion budget expired";
      break;
    }
    const QueueEntry entry = workspace.open.front();
    std::pop_heap(
      workspace.open.begin(), workspace.open.end(), greater);
    workspace.open.pop_back();
    const std::size_t current = entry.second;
    if (workspace.IsClosed(current)) {
      continue;
    }
    workspace.Close(current);
    ++result.stats.expanded_nodes;
    ++result.stats.unique_states;
    if (current == goal_index) {
      found = true;
      break;
    }
    const double current_g = workspace.g_score[current];
    for (std::size_t edge_index = topology->offsets[current];
      edge_index < topology->offsets[current + 1U]; ++edge_index)
    {
      const Impl::Edge & edge = topology->edges[edge_index];
      const std::size_t next = edge.target;
      if (workspace.IsClosed(next)) {
        continue;
      }
      // Live traversability, not a cached obstacle decision. The current node
      // was already accepted (or is the checked start), so checking the
      // neighbor is equivalent to the old both-endpoint filter.
      // A discovered node was already checked in this immutable obstacle
      // snapshot. Reuse that result within this request only; generation stamps
      // make a subsequent request recheck the new snapshot.
      if (!workspace.HasG(next)) {
        ++result.stats.obstacle_queries;
        ++result.stats.map_queries;
        if (!distance_field.IsTraversable(nodes[next].id)) {
          continue;
        }
      }

      double cost = edge.length;
      double slope = 0.0;
      if (slope_enabled) {
        slope = std::max(
          workspace.node_slope[edge.canonical_source],
          workspace.node_slope[edge.canonical_target]);
      }
      double roughness = 0.0;
      if (roughness_enabled) {
        roughness = std::max(
          workspace.node_roughness[edge.canonical_source],
          workspace.node_roughness[edge.canonical_target]);
      }
      cost *= 1.0 + options_.slope_penalty_weight * slope +
        options_.roughness_penalty_weight * roughness;

      if (obstacle_penalty_enabled) {
        ++result.stats.distance_queries;
        ++result.stats.map_queries;
        const double obstacle_distance =
          distance_field.DistanceM(nodes[edge.canonical_target].id);
        if (std::isfinite(obstacle_distance) &&
          obstacle_distance < options_.obstacle_penalty_distance_m)
        {
          cost += options_.obstacle_penalty_weight *
            (options_.obstacle_penalty_distance_m - obstacle_distance) /
            options_.obstacle_penalty_distance_m;
        }
      }

      const double candidate = current_g + cost;
      if (candidate + 1.0e-12 <
        (workspace.HasG(next) ? workspace.g_score[next] : kInfinity))
      {
        workspace.SetG(next, candidate, current);
        workspace.open.emplace_back(
          candidate + heuristic(nodes[next]), next);
        std::push_heap(
          workspace.open.begin(), workspace.open.end(), greater);
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
    bool chain_complete = true;
    while (true) {
      const BudgetTracker::Status chain_budget = tracker.Check();
      if (chain_budget != BudgetTracker::Status::kOk) {
        result.status = ToSearchStatus(chain_budget);
        result.detail =
          "graph route reconstruction budget expired";
        result.path.clear();
        result.node_path.clear();
        chain_complete = false;
        break;
      }
      indices.push_back(current);
      if (current == start_index) {
        break;
      }
      if (workspace.parent[current] ==
        std::numeric_limits<std::size_t>::max())
      {
        result.status = SearchStatus::kInternalError;
        result.detail = "graph parent chain is incomplete";
        result.path.clear();
        result.node_path.clear();
        chain_complete = false;
        break;
      }
      current = workspace.parent[current];
    }
    if (chain_complete && result.status != SearchStatus::kInternalError) {
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
        pose.cumulative_cost = workspace.g_score[indices[index]];
        result.path.push_back(pose);
      }
    }
  }
  // map_queries is maintained as the aggregate of the actual instrumented
  // terrain/obstacle/distance lookups. It is never a support-evaluation count.
  return finish();
}

}  // namespace dddmr_grid_global_planner
