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

#include "grid_global_planner/terrain_visualization.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "grid_global_planner/continuous_ground_surface.h"
#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/terrain_boundary_cost.h"

namespace grid_global_planner
{
namespace
{

using Marker = visualization_msgs::msg::Marker;
using Corners = std::array<geometry_msgs::msg::Point, 4>;

// Colors describe terrain/occupancy evidence only, never wheel feasibility.
// Hard occupied/prohibited/unknown states always win over the soft heat.
std_msgs::msg::ColorRGBA ObstacleHeatColor(double normalized_risk)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.9F;
  const double risk = std::clamp(normalized_risk, 0.0, 1.0);
  if (risk <= 0.5) {
    // Green -> yellow for the first half of the obstacle range.
    const double fraction = risk / 0.5;
    color.r = static_cast<float>(0.18 + 0.82 * fraction);
    color.g = static_cast<float>(0.72 + 0.28 * fraction);
    color.b = static_cast<float>(0.40 * (1.0 - fraction));
  } else {
    // Yellow -> orange as the obstacle gets closer, so the nearest supported
    // clear cells are unmistakably warmer than the boundary-only ramp.
    const double fraction = (risk - 0.5) / 0.5;
    color.r = 1.0F;
    color.g = static_cast<float>(1.0 - 0.55 * fraction);
    color.b = 0.0F;
  }
  return color;
}

std_msgs::msg::ColorRGBA SurfaceColor(
  const MapSnapshot & snapshot, const TerrainNode & node, bool static_map_only,
  double boundary_penalty_distance_m, double boundary_penalty_weight,
  bool obstacle_heat_enabled, double obstacle_penalty_distance_m,
  double obstacle_penalty_weight)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.9F;
  // An unconfirmed drivable layer is not a deletion of measured geometry: the
  // occupied record and its cell face remain, but the display must show this
  // node as unknown gray. This takes precedence over occupancy coloring.
  if (node.status == TerrainStatus::kUnknown &&
    node.reason == ProhibitedReason::kUnconfirmedComponent)
  {
    color.r = color.g = color.b = 0.55F;
    return color;
  }

  const auto & field = snapshot.distance_field();
  const bool obstacle_display =
    obstacle_heat_enabled && field && field->ready();
  const ObstacleAttributes * static_record = snapshot.static_obstacles().Find(node.id);
  ObstacleState occupancy = ObstacleState::kUnknown;
  if (obstacle_display) {
    // Use the same pinned merged field the graph reads, so a dynamic occupied
    // record is red even when the terrain faces come from the offline static
    // map. The field's actual traversal policy decides whether static-clear
    // ground with unobserved dynamic evidence can enter the soft display.
    occupancy = field->State(node.id);
  } else {
    occupancy = static_map_only ?
      (static_record == nullptr ? ObstacleState::kUnknown :
      static_record->static_state) :
      snapshot.MergedObstacle(node.id).CombinedState();
  }
  const bool allowed_static_unknown = obstacle_display && static_map_only &&
    occupancy == ObstacleState::kUnknown && static_record != nullptr &&
    static_record->static_state == ObstacleState::kClear &&
    field->IsTraversable(node.id);
  if (occupancy == ObstacleState::kOccupied) {
    color.r = 0.90F;
    color.g = 0.12F;
    color.b = 0.10F;
  } else if (node.status == TerrainStatus::kProhibited) {
    color.r = 1.0F;
    color.g = 0.55F;
    color.b = 0.05F;
  } else if (!node.IsSupported() ||
    (occupancy != ObstacleState::kClear && !allowed_static_unknown))
  {
    color.r = color.g = color.b = 0.55F;
  } else {
    // Outside the obstacle heat range, retain the offline evaluator's boundary
    // palette: green is zero risk, yellow is measured-boundary heat, and
    // violet/blue identifies insufficient-connection evidence. The obstacle
    // gradient takes display priority only on policy-traversable ground.
    // A zero range/weight explicitly disables only the
    // soft cost: known-boundary heat disappears, but an insufficient-evidence
    // seed keeps its warning label. This full/static/offline display is a
    // single-cell evidence hint, not a full-body feasibility or hardware claim.
    // Invalid values are rendered unknown gray, never safe green.
    double normalized_risk = 0.0;
    // Invalid values take precedence over the zero short-circuit: a config
    // error must not be displayed as safe green merely because the other value
    // happens to be zero.
    const bool heat_invalid =
      !std::isfinite(boundary_penalty_distance_m) ||
      boundary_penalty_distance_m < 0.0 ||
      !std::isfinite(boundary_penalty_weight) ||
      boundary_penalty_weight < 0.0;
    if (heat_invalid) {
      color.r = color.g = color.b = 0.55F;
      return color;
    }
    const bool heat_explicitly_disabled =
      boundary_penalty_distance_m == 0.0 || boundary_penalty_weight == 0.0;
    const TerrainSnapshot * terrain = snapshot.fine_terrain().get();
    if (terrain == nullptr) {
      color.r = color.g = color.b = 0.55F;
      return color;
    }

    // The obstacle term is finite-only and zero outside the graph radius or
    // when either graph value is zero. A non-finite distance never becomes
    // full heat; it simply contributes nothing, exactly like the graph.
    const double obstacle_risk = obstacle_display ?
      NormalizedObstacleHeatRisk(
      field->DistanceM(node.id), obstacle_penalty_distance_m,
      obstacle_penalty_weight) : 0.0;
    if (obstacle_risk > 0.0) {
      // Inside the obstacle range show its distance gradient explicitly. A
      // saturated boundary-warning hue must not hide this information. Hard
      // states were handled above; outside the range retain the original colors.
      return ObstacleHeatColor(obstacle_risk);
    }

    const SupportBoundaryClassification classification =
      terrain->SupportBoundaryClassificationAt(node.id);
    const double measured_distance =
      terrain->MeasuredSupportBoundaryDistanceM(node.id);
    const double insufficient_distance =
      terrain->InsufficientConnectionDistanceM(node.id);
    const bool uncertainty_in_range =
      !heat_explicitly_disabled &&
      std::isfinite(insufficient_distance) &&
      insufficient_distance <= boundary_penalty_distance_m;
    const bool boundary_uncertainty =
      classification.insufficient_connection_evidence || uncertainty_in_range;
    if (boundary_uncertainty) {
      // The conservative combined risk (max of the two same-shaped penalties)
      // drives intensity, but the hue identifies unknown connectivity evidence
      // rather than a measured geometric edge. With the soft term disabled, an
      // own-seed label is still shown at full uncertainty intensity because
      // zero weight must not erase the evidence itself.
      double uncertainty_fraction = 1.0;
      if (!heat_explicitly_disabled) {
        uncertainty_fraction = std::clamp(
          TerrainBoundaryEvidencePenalty(
            measured_distance, insufficient_distance,
            boundary_penalty_distance_m, boundary_penalty_weight) /
          boundary_penalty_weight, 0.0, 1.0);
      }
      color.r = static_cast<float>(0.20 + 0.55 * uncertainty_fraction);
      color.g = static_cast<float>(0.72 - 0.45 * uncertainty_fraction);
      color.b = static_cast<float>(0.55 + 0.40 * uncertainty_fraction);
      return color;
    }
    if (!heat_explicitly_disabled) {
      const double penalty = TerrainBoundaryEvidencePenalty(
        measured_distance, insufficient_distance,
        boundary_penalty_distance_m, boundary_penalty_weight);
      normalized_risk = std::clamp(
        penalty / boundary_penalty_weight, 0.0, 1.0);
    }
    color.r = static_cast<float>(0.18 + 0.82 * normalized_risk);
    color.g = static_cast<float>(0.72 + 0.28 * normalized_risk);
    color.b = static_cast<float>(0.40 * (1.0 - normalized_risk));
  }
  return color;
}

// Returns the actual finite support Z range. The snapshot stores it directly;
// the point fallback keeps synthetic/diagnostic nodes bounded without inventing
// evidence when those summary fields are unavailable.
bool SupportHeightRange(
  const TerrainNode & node, double * z_min, double * z_max)
{
  double minimum = node.support.z_min;
  double maximum = node.support.z_max;
  if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
    minimum > maximum)
  {
    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (const Point3 & point : node.support.points) {
      if (!point.IsFinite()) {
        continue;
      }
      minimum = std::min(minimum, point.z);
      maximum = std::max(maximum, point.z);
    }
  }
  if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
    minimum > maximum)
  {
    return false;
  }
  *z_min = minimum;
  *z_max = maximum;
  return true;
}

// Computes the exact XY square of this map cell, independently of where its
// support points/representative happened to lie. Heights satisfy the local
// plane through its centroid. Unsafe extrapolation is omitted rather than
// replaced by an invented horizontal face. The node and its raw observations
// remain untouched; the caller records an invalid display cell.
bool CellCorners(
  const TerrainSnapshot & terrain, const TerrainNode & node, Corners * corners)
{
  std::array<Point3, 4> shared;
  if (TerrainSurfaceCorners(node, &shared)) {
    for (std::size_t i = 0; i < corners->size(); ++i) {
      if (!shared[i].IsFinite()) {
        return false;
      }
      (*corners)[i].x = shared[i].x;
      (*corners)[i].y = shared[i].y;
      (*corners)[i].z = shared[i].z;
    }
    return true;
  }
  const auto & plane = node.plane;
  const auto & normal = plane.normal;
  const double normal_length = std::hypot(normal.x, normal.y, normal.z);
  if (!plane.valid || !plane.centroid.IsFinite() ||
    !std::isfinite(normal_length) || normal_length <= 0.0 ||
    std::abs(normal.z) <= 1e-8 * normal_length)
  {
    return false;
  }
  const double resolution = terrain.resolution();
  const double x0 = terrain.origin_x() + static_cast<double>(node.cell.x) * resolution;
  const double y0 = terrain.origin_y() + static_cast<double>(node.cell.y) * resolution;
  const double x1 = x0 + resolution;
  const double y1 = y0 + resolution;
  if (!std::isfinite(x0) || !std::isfinite(y0) ||
    !std::isfinite(x1) || !std::isfinite(y1) || x1 <= x0 || y1 <= y0)
  {
    return false;
  }
  const std::array<double, 4> xs{x0, x1, x1, x0};
  const std::array<double, 4> ys{y0, y0, y1, y1};

  double support_min = 0.0;
  double support_max = 0.0;
  const bool has_support_range =
    SupportHeightRange(node, &support_min, &support_max);
  const double z_lower = support_min - resolution;
  const double z_upper = support_max + resolution;
  // Preserve exact corner evaluation for ordinary height-field slopes. The
  // observed-height guard addresses steep-plane extrapolation only; 45 degrees
  // is a display guard, not a change to the planner's vehicle/slope policy.
  const bool stable_height_field =
    std::abs(normal.z) >= std::sqrt(0.5) * normal_length;
  if (stable_height_field ||
    (has_support_range && std::isfinite(z_lower) && std::isfinite(z_upper)))
  {
    for (std::size_t i = 0; i < corners->size(); ++i) {
      auto & corner = (*corners)[i];
      corner.x = xs[i];
      corner.y = ys[i];
      corner.z = plane.centroid.z -
        (normal.x * (corner.x - plane.centroid.x) +
        normal.y * (corner.y - plane.centroid.y)) / normal.z;
      if (!std::isfinite(corner.z)) {
        return false;
      }
    }
    if (stable_height_field) {
      return true;
    }
    bool within_measured_range = true;
    for (const auto & corner : *corners) {
      within_measured_range = within_measured_range &&
        corner.z >= z_lower && corner.z <= z_upper;
    }
    if (within_measured_range) {
      return true;
    }
  }
  return false;
}

// Creates a world-coordinate marker: geometry has already been transformed to
// the map frame, so pose and scale must be identity and lifetime must be zero.
Marker MakeMarker(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp,
  const std::string & marker_namespace, std::int32_t id, std::int32_t type)
{
  Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0F;
  return marker;
}

// Allocates a bounded triangle/edge batch instead of a marker for every cell.
void AppendBatch(
  const TerrainSnapshot & terrain, const builtin_interfaces::msg::Time & stamp,
  std::size_t capacity, std::int32_t batch_id,
  visualization_msgs::msg::MarkerArray * markers)
{
  Marker faces = MakeMarker(
    terrain.frame_id(), stamp, "terrain_cells", batch_id, Marker::TRIANGLE_LIST);
  faces.points.reserve(capacity * 6);
  faces.colors.reserve(capacity * 6);
  Marker edges = MakeMarker(
    terrain.frame_id(), stamp, "terrain_grid_edges", batch_id, Marker::LINE_LIST);
  edges.scale.x = std::clamp(terrain.resolution() * 0.015, 0.001, 0.01);
  edges.color.r = edges.color.g = edges.color.b = 0.08F;
  edges.color.a = 0.95F;
  edges.points.reserve(capacity * 8);
  edges.colors.reserve(capacity * 8);
  markers->markers.push_back(std::move(faces));
  markers->markers.push_back(std::move(edges));
}

// Appends one gray LINE_LIST measurement batch. The wireframe is the actual
// own-cell measured Z range, not the fitted plane or the whole-neighbour Z
// range; supported/prohibited patches are never duplicated here.
Marker MakeMeasurementMarker(
  const TerrainSnapshot & terrain, const builtin_interfaces::msg::Time & stamp,
  std::int32_t batch_id)
{
  Marker marker = MakeMarker(
    terrain.frame_id(), stamp, "terrain_measurement_patches", batch_id,
    Marker::LINE_LIST);
  marker.scale.x = std::clamp(terrain.resolution() * 0.01, 0.001, 0.01);
  marker.color.r = 0.55F;
  marker.color.g = 0.55F;
  marker.color.b = 0.55F;
  marker.color.a = 0.8F;
  return marker;
}

// Adds one 12-segment wireframe for the patch's own measured XY cell and actual
// own Z bounds. No triangle is created and no missing own observation is drawn.
bool AppendMeasurementWireframe(
  const TerrainSnapshot & terrain, const MeasurementPatch & patch,
  Marker * marker)
{
  if (marker == nullptr || patch.node_id != kInvalidTerrainNodeId ||
    !patch.HasOwnCellObservation() ||
    (patch.status != MeasurementPatchStatus::kObservedOnly &&
    patch.status != MeasurementPatchStatus::kFitRejected) ||
    !std::isfinite(patch.own_z_min) || !std::isfinite(patch.own_z_max))
  {
    return false;
  }
  const double resolution = terrain.resolution();
  const double x0 =
    terrain.origin_x() + static_cast<double>(patch.cell.x) * resolution;
  const double y0 =
    terrain.origin_y() + static_cast<double>(patch.cell.y) * resolution;
  const double x1 = x0 + resolution;
  const double y1 = y0 + resolution;
  if (!std::isfinite(x0) || !std::isfinite(y0) ||
    !std::isfinite(x1) || !std::isfinite(y1) || x1 <= x0 || y1 <= y0)
  {
    return false;
  }
  const std::array<double, 4> xs{x0, x1, x1, x0};
  const std::array<double, 4> ys{y0, y0, y1, y1};
  const double z_min = patch.own_z_min;
  const double z_max = patch.own_z_max;
  marker->points.reserve(marker->points.size() + 24U);
  const auto append_segment = [&](double ax, double ay, double az,
      double bx, double by, double bz) {
      geometry_msgs::msg::Point first;
      first.x = ax;
      first.y = ay;
      first.z = az;
      geometry_msgs::msg::Point second;
      second.x = bx;
      second.y = by;
      second.z = bz;
      marker->points.push_back(first);
      marker->points.push_back(second);
    };
  for (std::size_t edge = 0; edge < 4U; ++edge) {
    const std::size_t next = (edge + 1U) % 4U;
    append_segment(
      xs[edge], ys[edge], z_min, xs[next], ys[next], z_min);
  }
  for (std::size_t edge = 0; edge < 4U; ++edge) {
    const std::size_t next = (edge + 1U) % 4U;
    append_segment(
      xs[edge], ys[edge], z_max, xs[next], ys[next], z_max);
  }
  for (std::size_t corner = 0; corner < 4U; ++corner) {
    append_segment(
      xs[corner], ys[corner], z_min, xs[corner], ys[corner], z_max);
  }
  return true;
}

}  // namespace

double NormalizedObstacleHeatRisk(
  double distance_m, double range_m, double weight)
{
  if (!std::isfinite(distance_m) || !std::isfinite(range_m) ||
    !std::isfinite(weight) || range_m <= 0.0 || weight <= 0.0)
  {
    return 0.0;
  }
  if (distance_m >= range_m) {
    return 0.0;
  }
  // Exactly the graph's linear (R-d)/R shape, normalized to 0..1. The weight
  // enables the soft term but is not used to rescale the display fraction.
  return std::clamp((range_m - distance_m) / range_m, 0.0, 1.0);
}

bool TerrainVisualizationRefresh::Queue(
  std::shared_ptr<const MapSnapshot> snapshot)
{
  if (!snapshot) {
    return false;
  }
  const std::uint64_t version = snapshot->obstacle_version();
  if (has_rendered_ && snapshot == rendered_ && version == rendered_version_) {
    return false;
  }
  if (has_rendered_ && version < rendered_version_) {
    return false;
  }
  // Monotonic across TakePending(): while C is being painted there is no
  // pending entry, but an older B must still be rejected so it cannot be drawn
  // after C. Equal versions remain eligible when the immutable object differs.
  if (has_newest_version_ && version < newest_version_) {
    return false;
  }
  if (has_pending_ && snapshot == pending_ && version == pending_version_) {
    return false;
  }
  if (has_pending_ && version < pending_version_) {
    return false;
  }
  pending_ = std::move(snapshot);
  pending_version_ = version;
  newest_version_ = version;
  has_pending_ = true;
  has_newest_version_ = true;
  return true;
}

std::shared_ptr<const MapSnapshot> TerrainVisualizationRefresh::TakePending()
{
  if (!has_pending_) {
    return nullptr;
  }
  has_pending_ = false;
  return std::move(pending_);
}

void TerrainVisualizationRefresh::MarkRendered(
  const std::shared_ptr<const MapSnapshot> & snapshot)
{
  if (!snapshot) {
    return;
  }
  const std::uint64_t version = snapshot->obstacle_version();
  rendered_ = snapshot;
  rendered_version_ = version;
  has_rendered_ = true;
  if (!has_newest_version_ || version > newest_version_) {
    newest_version_ = version;
    has_newest_version_ = true;
  }
}

TerrainVisualizationResult BuildTerrainVisualization(
  const MapSnapshot & snapshot, const builtin_interfaces::msg::Time & stamp,
  const TerrainVisualizationOptions & options)
{
  TerrainVisualizationResult result;
  const auto & terrain = snapshot.fine_terrain();
  // This topic belongs exclusively to the terrain display. A replacement starts
  // by clearing all its batches, including batches no longer present after a
  // shrink or an empty map. Latched late subscribers get the same replacement.
  Marker clear;
  clear.header.frame_id = terrain ? terrain->frame_id() : "map";
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  result.markers.markers.push_back(std::move(clear));
  if (!terrain) {
    return result;
  }
  const auto & nodes = terrain->nodes();
  const std::size_t budget = options.max_cells == 0 ?
    nodes.size() : std::min(nodes.size(), options.max_cells);
  const std::size_t batches = budget / kTerrainVisualizationBatchCells +
    static_cast<std::size_t>(budget % kTerrainVisualizationBatchCells != 0);
  result.markers.markers.reserve(1 + 2 * batches);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (options.max_cells != 0 && result.rendered_cells >= options.max_cells) {
      result.truncated = true;
      break;
    }
    Corners corners;
    if (!CellCorners(*terrain, nodes[i], &corners)) {
      ++result.invalid_cells;
      continue;
    }
    if (result.rendered_cells % kTerrainVisualizationBatchCells == 0) {
      AppendBatch(
        *terrain, stamp,
        std::min(kTerrainVisualizationBatchCells, budget - result.rendered_cells),
        static_cast<std::int32_t>(result.rendered_cells / kTerrainVisualizationBatchCells),
        &result.markers);
    }
    auto & faces = result.markers.markers[result.markers.markers.size() - 2];
    auto & edges = result.markers.markers.back();
    auto color = SurfaceColor(
      snapshot, nodes[i], options.static_map_only,
      options.boundary_penalty_distance_m,
      options.boundary_penalty_weight, options.obstacle_heat_enabled,
      options.obstacle_penalty_distance_m,
      options.obstacle_penalty_weight);
    // Estimated vs measured provenance. `expansion_used` is the existing
    // bounded same-floor recovery flag; a recovered cell keeps its cost or
    // unknown-gray face color and is marked only by cyan edge vertices. No
    // extra marker is emitted.
    const MeasurementPatch * patch =
      terrain->FindMeasurementPatchForNode(nodes[i].id);
    const bool estimated =
      (patch != nullptr && patch->expansion_used) ||
      nodes[i].surface_reconstruction.applied;
    if (estimated) {
      ++result.estimated_cells;
    }
    // Counterclockwise triangles expose the upper face even for a plane whose
    // producer stored the equivalent downward-facing normal.
    for (const std::size_t corner : {0U, 1U, 2U, 0U, 2U, 3U}) {
      faces.points.push_back(corners[corner]);
      faces.colors.push_back(color);
    }
    // Offset edges only, by 0.5% of a cell, to avoid z-fighting. Surface heights
    // remain exact; this is a rendering separation, not invented map evidence.
    std_msgs::msg::ColorRGBA edge_color;
    edge_color.r = estimated ? 0.00F : 0.08F;
    edge_color.g = estimated ? 0.78F : 0.08F;
    edge_color.b = estimated ? 0.95F : 0.08F;
    edge_color.a = 0.95F;
    for (std::size_t edge = 0; edge < corners.size(); ++edge) {
      auto start = corners[edge];
      auto end = corners[(edge + 1) % corners.size()];
      start.z += terrain->resolution() * 0.005;
      end.z += terrain->resolution() * 0.005;
      edges.points.push_back(start);
      edges.points.push_back(end);
      edges.colors.push_back(edge_color);
      edges.colors.push_back(edge_color);
    }
    ++result.rendered_cells;
  }

  // Measurement evidence is a separate gray diagnostic display. Only actual
  // own-cell observations of unlinked rejected/observed patches are shown; the
  // planning map, supported faces and edge batches above are unchanged.
  if (terrain) {
    const auto & measurement_patches = terrain->measurement_patches();
    for (const MeasurementPatch & patch : measurement_patches) {
      if (patch.node_id != kInvalidTerrainNodeId ||
        !patch.HasOwnCellObservation() ||
        (patch.status != MeasurementPatchStatus::kObservedOnly &&
        patch.status != MeasurementPatchStatus::kFitRejected))
      {
        continue;
      }
      if (options.max_measurement_render_patches != 0 &&
        result.rendered_measurement_patches >=
        options.max_measurement_render_patches)
      {
        result.measurement_truncated = true;
        break;
      }
      if (result.rendered_measurement_patches %
        kMeasurementVisualizationBatchPatches == 0U)
      {
        const std::size_t remaining =
          options.max_measurement_render_patches == 0 ?
          kMeasurementVisualizationBatchPatches :
          std::min(
          kMeasurementVisualizationBatchPatches,
          options.max_measurement_render_patches -
          result.rendered_measurement_patches);
        Marker batch = MakeMeasurementMarker(
          *terrain, stamp,
          static_cast<std::int32_t>(
            result.rendered_measurement_patches /
            kMeasurementVisualizationBatchPatches));
        batch.points.reserve(remaining * 24U);
        result.markers.markers.push_back(std::move(batch));
      }
      if (!AppendMeasurementWireframe(
          *terrain, patch, &result.markers.markers.back()))
      {
        continue;
      }
      ++result.rendered_measurement_patches;
    }
  }
  return result;
}

}  // namespace grid_global_planner
