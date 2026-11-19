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

#include "grid_global_planner/terrain_boundary_cost.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "grid_global_planner/map_snapshot.h"

namespace grid_global_planner
{
namespace
{

// Eight XY directions that make a surface a support boundary when the matching
// local connected supported neighbor is absent. Diagonal steps cost
// resolution * sqrt(2); floors are never mixed.
constexpr std::array<std::array<std::int64_t, 2>, 8> kBoundaryDirections{{
  {{-1, -1}}, {{-1, 0}}, {{-1, 1}}, {{0, -1}},
  {{0, 1}}, {{1, -1}}, {{1, 0}}, {{1, 1}}}};

}  // namespace

double TerrainBoundaryPenalty(double distance_m, double range_m, double weight)
{
  if (!std::isfinite(range_m) || range_m < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryPenalty: range must be finite and non-negative");
  }
  if (!std::isfinite(weight) || weight < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryPenalty: weight must be finite and non-negative");
  }
  if (range_m == 0.0 || weight == 0.0) {
    return 0.0;
  }
  // Missing/unknown/negative distances fail closed to the full bounded risk.
  // This is a topology preference, not proof of a collision.
  if (!std::isfinite(distance_m) || distance_m < 0.0) {
    return weight;
  }
  const double normalized =
    std::clamp(1.0 - distance_m / range_m, 0.0, 1.0);
  return weight * normalized * normalized;
}

double TerrainBoundaryEvidenceComponentPenalty(
  double distance_m, double range_m, double weight)
{
  if (!std::isfinite(range_m) || range_m < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryEvidenceComponentPenalty: range must be finite and "
            "non-negative");
  }
  if (!std::isfinite(weight) || weight < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryEvidenceComponentPenalty: weight must be finite and "
            "non-negative");
  }
  if (range_m == 0.0 || weight == 0.0) {
    return 0.0;
  }
  // Positive infinity is the immutable cache's "no seed of this family"
  // sentinel, not missing evidence. It contributes zero.
  if (std::isinf(distance_m) && distance_m > 0.0) {
    return 0.0;
  }
  if (!std::isfinite(distance_m) || distance_m < 0.0) {
    return weight;
  }
  return TerrainBoundaryPenalty(distance_m, range_m, weight);
}

double TerrainBoundaryEvidencePenalty(
  double measured_support_distance_m,
  double insufficient_connection_distance_m,
  double range_m, double weight)
{
  if (!std::isfinite(range_m) || range_m < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryEvidencePenalty: range must be finite and "
            "non-negative");
  }
  if (!std::isfinite(weight) || weight < 0.0) {
    throw std::invalid_argument(
            "TerrainBoundaryEvidencePenalty: weight must be finite and "
            "non-negative");
  }
  if (range_m == 0.0 || weight == 0.0) {
    return 0.0;
  }
  const bool measured_no_seed =
    std::isinf(measured_support_distance_m) &&
    measured_support_distance_m > 0.0;
  const bool insufficient_no_seed =
    std::isinf(insufficient_connection_distance_m) &&
    insufficient_connection_distance_m > 0.0;
  if (measured_no_seed && insufficient_no_seed) {
    // No classified boundary in this component: preserve the old conservative
    // infinity-to-full-risk behavior instead of silently removing unknown risk.
    return weight;
  }
  return std::max(
    TerrainBoundaryEvidenceComponentPenalty(
      measured_support_distance_m, range_m, weight),
    TerrainBoundaryEvidenceComponentPenalty(
      insufficient_connection_distance_m, range_m, weight));
}

double TerrainSnapshot::SupportBoundaryDistanceM(
  TerrainNodeId id) const noexcept
{
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end() ||
    iterator->second >= measured_support_boundary_distances_m_.size() ||
    iterator->second >= insufficient_connection_distances_m_.size())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t index = iterator->second;
  const double measured = measured_support_boundary_distances_m_[index];
  const double insufficient = insufficient_connection_distances_m_[index];
  const bool measured_known = std::isfinite(measured);
  const bool insufficient_known = std::isfinite(insufficient);
  if (measured_known && insufficient_known) {
    return std::min(measured, insufficient);
  }
  if (measured_known) {
    return measured;
  }
  if (insufficient_known) {
    return insufficient;
  }
  // Neither family has a seed in this component. Positive infinity is the
  // cache's "no boundary seed" sentinel; guarded penalty callers treat it as
  // zero contribution, while the legacy TerrainBoundaryPenalty keeps charging
  // full risk if a caller feeds it directly.
  return std::numeric_limits<double>::infinity();
}

double TerrainSnapshot::MeasuredSupportBoundaryDistanceM(
  TerrainNodeId id) const noexcept
{
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end() ||
    iterator->second >= measured_support_boundary_distances_m_.size())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return measured_support_boundary_distances_m_[iterator->second];
}

double TerrainSnapshot::InsufficientConnectionDistanceM(
  TerrainNodeId id) const noexcept
{
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end() ||
    iterator->second >= insufficient_connection_distances_m_.size())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return insufficient_connection_distances_m_[iterator->second];
}

SupportBoundaryClassification TerrainSnapshot::SupportBoundaryClassificationAt(
  TerrainNodeId id) const noexcept
{
  SupportBoundaryClassification classification;
  const auto iterator = id_to_index_.find(id);
  if (iterator == id_to_index_.end() ||
    iterator->second >= support_boundary_classifications_.size())
  {
    return classification;
  }
  return support_boundary_classifications_[iterator->second];
}

void TerrainSnapshot::BuildSupportBoundaryDistances()
{
  const double infinity = std::numeric_limits<double>::infinity();
  measured_support_boundary_distances_m_.assign(nodes_.size(), infinity);
  insufficient_connection_distances_m_.assign(nodes_.size(), infinity);
  support_boundary_classifications_.assign(nodes_.size(), {});
  if (nodes_.empty()) {
    return;
  }
  const double resolution = metadata_.resolution;

  // Evidence of one candidate link. A missing record or an explicit
  // kUnknown/kInsufficientEvidence status is insufficient evidence, never a
  // known physical edge. kDiscontinuous/kInconsistentNormals are validated
  // known rejections, so they describe a measured support-domain limit. No
  // height/slope threshold is re-derived here; the builder owns those gates.
  enum class LinkEvidence : std::uint8_t
  {
    kConnected = 0,
    kInsufficient = 1,
    kKnownRejection = 2,
  };
  const auto link_evidence =
    [this](TerrainNodeId first, TerrainNodeId second) {
      const auto iterator = node_to_connections_.find(first);
      if (iterator == node_to_connections_.end()) {
        return LinkEvidence::kInsufficient;
      }
      bool found = false;
      bool insufficient = false;
      for (const std::size_t connection_index : iterator->second) {
        const TerrainConnection & connection = connections_[connection_index];
        if (!((connection.source == first &&
          connection.target == second) ||
          (connection.source == second &&
          connection.target == first)))
        {
          continue;
        }
        found = true;
        if (connection.IsConnected()) {
          return LinkEvidence::kConnected;
        }
        if (connection.status == ConnectionStatus::kUnknown ||
          connection.status == ConnectionStatus::kInsufficientEvidence)
        {
          insufficient = true;
        }
      }
      if (!found || insufficient) {
        return LinkEvidence::kInsufficient;
      }
      return LinkEvidence::kKnownRejection;
    };

  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    const TerrainNode & node = nodes_[index];
    // Unsupported surfaces carry no traversable support. They keep the legacy
    // zero compatibility distance so callers that still treat them as hard
    // states retain full risk, but they are never propagation sources.
    if (!node.IsSupported()) {
      measured_support_boundary_distances_m_[index] = 0.0;
      insufficient_connection_distances_m_[index] = 0.0;
      continue;
    }
    bool measured_edge = false;
    bool insufficient_edge = false;
    for (const auto & direction : kBoundaryDirections) {
      const FineCellIndex neighbor_cell{
        node.cell.x + direction[0], node.cell.y + direction[1]};
      const auto cell_iterator = cell_to_nodes_.find(neighbor_cell);
      bool connected = false;
      bool insufficient = false;
      if (cell_iterator != cell_to_nodes_.end()) {
        for (const TerrainNodeId neighbor_id : cell_iterator->second) {
          const TerrainNode * other = FindNode(neighbor_id);
          if (other == nullptr) {
            continue;
          }
          if (!other->IsSupported()) {
            // Prohibited/unknown surfaces are known non-support, not a viable
            // same-deck candidate. The direction remains a measured end.
            continue;
          }
          const LinkEvidence evidence = link_evidence(node.id, other->id);
          if (evidence == LinkEvidence::kConnected) {
            // Any actual connected same-deck neighbor supplies this XY
            // direction, regardless of other rejected deck candidates here.
            connected = true;
            break;
          }
          if (evidence == LinkEvidence::kInsufficient) {
            insufficient = true;
          }
        }
      }
      if (connected) {
        continue;
      }
      if (insufficient) {
        insufficient_edge = true;
      } else {
        // No viable same-deck candidate, or every candidate is a validated
        // rejection/prohibited/unsupported surface.
        measured_edge = true;
      }
    }
    SupportBoundaryClassification classification;
    if (measured_edge) {
      classification.measured_support_edge = true;
      measured_support_boundary_distances_m_[index] = 0.0;
    }
    if (insufficient_edge) {
      classification.insufficient_connection_evidence = true;
      insufficient_connection_distances_m_[index] = 0.0;
    }
    support_boundary_classifications_[index] = classification;
  }

  // Multi-source Dijkstra for either family over valid local supported edges
  // only. Every edge weight is the projected XY grid step, so a deck boundary
  // is never reached through another floor or a same-XY vertical link.
  const auto propagate =
    [this, resolution](std::vector<double> * distances) {
      using QueueEntry = std::pair<double, std::size_t>;
      std::priority_queue<
        QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
      for (std::size_t index = 0; index < nodes_.size(); ++index) {
        if (nodes_[index].IsSupported() && (*distances)[index] == 0.0) {
          frontier.emplace(0.0, index);
        }
      }
      while (!frontier.empty()) {
        const QueueEntry entry = frontier.top();
        frontier.pop();
        if (entry.first > (*distances)[entry.second]) {
          continue;
        }
        const TerrainNode & node = nodes_[entry.second];
        const auto * neighbors = ConnectedNeighbors(node.id);
        if (neighbors == nullptr) {
          continue;
        }
        for (const TerrainNodeId neighbor_id : *neighbors) {
          const auto neighbor_iterator = id_to_index_.find(neighbor_id);
          if (neighbor_iterator == id_to_index_.end()) {
            continue;
          }
          const std::size_t neighbor_index = neighbor_iterator->second;
          const TerrainNode & other = nodes_[neighbor_index];
          if (!other.IsSupported()) {
            continue;
          }
          const std::int64_t dx = other.cell.x - node.cell.x;
          const std::int64_t dy = other.cell.y - node.cell.y;
          if (std::abs(dx) > 1 || std::abs(dy) > 1 ||
            (dx == 0 && dy == 0))
          {
            continue;
          }
          const double step =
            resolution * std::hypot(
            static_cast<double>(dx), static_cast<double>(dy));
          const double candidate = entry.first + step;
          if (candidate < (*distances)[neighbor_index]) {
            (*distances)[neighbor_index] = candidate;
            frontier.emplace(candidate, neighbor_index);
          }
        }
      }
    };
  propagate(&measured_support_boundary_distances_m_);
  propagate(&insufficient_connection_distances_m_);
}

}  // namespace grid_global_planner
