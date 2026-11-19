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

#include "grid_global_planner/latest_validation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/obstacle_tile.h"

namespace grid_global_planner
{
namespace
{

// FNV-1a byte hash. It is used only for local corridor change detection, not
// for terrain node IDs or persistent identity.
void HashBytes(std::uint64_t * hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const std::uint8_t *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= 1099511628211ULL;
  }
}

// Hashes finite distances at micrometre resolution and NaN explicitly. Exact
// clock/timestamp fields are intentionally excluded so an unchanged obstacle
// snapshot with a new source stamp does not invalidate a valid path.
void HashDouble(std::uint64_t * hash, double value)
{
  if (std::isfinite(value)) {
    const std::int64_t quantized =
      static_cast<std::int64_t>(std::llround(value * 1e6));
    HashBytes(hash, &quantized, sizeof(quantized));
  } else {
    const std::uint8_t marker = 0xFF;
    HashBytes(hash, &marker, sizeof(marker));
  }
}

// Hashes every obstacle field that can change collision/headroom semantics.
void HashAttributes(
  std::uint64_t * hash,
  const ObstacleAttributes & attributes)
{
  const std::uint8_t static_state =
    static_cast<std::uint8_t>(attributes.static_state);
  const std::uint8_t dynamic_state =
    static_cast<std::uint8_t>(attributes.dynamic_state);
  HashBytes(hash, &static_state, sizeof(static_state));
  HashBytes(hash, &dynamic_state, sizeof(dynamic_state));
  HashDouble(hash, attributes.static_distance_m);
  HashDouble(hash, attributes.dynamic_distance_m);
  HashDouble(hash, attributes.clearance_above_m);
  const std::uint8_t clearance_known = attributes.clearance_known ? 1U : 0U;
  HashBytes(hash, &clearance_known, sizeof(clearance_known));
  const std::uint8_t occupied_extent_known =
    attributes.occupied_vertical_extent_known ? 1U : 0U;
  const std::uint8_t observed_extent_known =
    attributes.observed_vertical_extent_known ? 1U : 0U;
  HashBytes(hash, &occupied_extent_known, sizeof(occupied_extent_known));
  HashBytes(hash, &observed_extent_known, sizeof(observed_extent_known));
  HashDouble(hash, attributes.occupied_min_z);
  HashDouble(hash, attributes.occupied_max_z);
  HashDouble(hash, attributes.observed_min_z);
  HashDouble(hash, attributes.observed_max_z);
}

// Adds one sampled point cell while distinguishing a work-budget stop from
// invalid geometry. A budget stop is recoverable through whole-snapshot
// validation; an invalid coordinate is not.
CorridorCoverageStatus AddCorridorPoint(
  const MapSnapshot & snapshot, const Point3 & point,
  const LatestValidationConfig & config, std::set<FineCellIndex> * cells,
  CorridorCoverage * coverage)
{
  if (cells == nullptr || coverage == nullptr || !snapshot.fine_terrain()) {
    return CorridorCoverageStatus::kInvalidGeometry;
  }
  if (coverage->sampled_points >= config.corridor_max_samples) {
    return CorridorCoverageStatus::kBudgetExhausted;
  }
  const auto cell = FineCellOf(
    point, snapshot.fine_terrain()->origin_x(),
    snapshot.fine_terrain()->origin_y(),
    snapshot.fine_terrain()->resolution());
  if (!cell.has_value()) {
    return CorridorCoverageStatus::kInvalidGeometry;
  }
  ++coverage->sampled_points;
  cells->insert(*cell);
  return CorridorCoverageStatus::kComplete;
}

// Rotates one body-frame point into the world using the pose's ZYX roll/pitch/
// yaw. This is the same transform the node's tf2 quaternion path applies, kept
// local so the core helper stays ROS/tf2-independent.
Point3 RotateAndTranslate(const PathPose & pose, const Point3 & local)
{
  const double cr = std::cos(pose.roll);
  const double sr = std::sin(pose.roll);
  const double cp = std::cos(pose.pitch);
  const double sp = std::sin(pose.pitch);
  const double cy = std::cos(pose.heading);
  const double sy = std::sin(pose.heading);

  const double y1 = local.y * cr - local.z * sr;
  const double z1 = local.y * sr + local.z * cr;
  const double x2 = local.x * cp + z1 * sp;
  const double z2 = -local.x * sp + z1 * cp;
  const double x3 = x2 * cy - y1 * sy;
  const double y3 = x2 * sy + y1 * cy;
  return Point3{x3 + pose.position.x, y3 + pose.position.y,
    z2 + pose.position.z};
}

// Generates the four-wheel and full-body sweep samples for one pose. Exceeding
// the corridor work budget is recoverable; missing/invalid body inputs are
// rejected rather than being mislabelled as a budget stop.
void AddBodyAndWheelSamples(
  const LatestValidationConfig & config,
  const PathPose & pose,
  std::vector<Point3> * samples,
  CorridorCoverageStatus * status)
{
  if (samples == nullptr || status == nullptr) {
    return;
  }
  const auto & vehicle = config.vehicle;
  const double spacing = std::max(0.02, config.body_sweep_spacing_m);
  auto append = [&](const Point3 & local) {
      samples->push_back(RotateAndTranslate(pose, local));
    };

  double half_length = 0.0;
  if (std::isfinite(vehicle.contact_patch_length_m)) {
    half_length = std::max(0.0, vehicle.contact_patch_length_m * 0.5);
  } else if (std::isfinite(vehicle.wheel_radius_m)) {
    half_length = std::max(0.0, vehicle.wheel_radius_m);
  }
  const double half_width =
    std::isfinite(vehicle.tire_width_m) ?
    std::max(0.0, vehicle.tire_width_m * 0.5) :
    0.0;
  for (const Point3 & offset : vehicle.wheel_contact_offsets) {
    const double local_x[3] = {offset.x - half_length, offset.x,
      offset.x + half_length};
    const double local_y[3] = {offset.y - half_width, offset.y,
      offset.y + half_width};
    for (const double x : local_x) {
      for (const double y : local_y) {
        append(Point3{x, y, offset.z});
        if (samples->size() > config.corridor_max_samples) {
          *status = CorridorCoverageStatus::kBudgetExhausted;
          return;
        }
      }
    }
  }

  if (!vehicle.body_max_configured || !vehicle.body_min.IsFinite() ||
    !vehicle.body_max.IsFinite())
  {
    *status = CorridorCoverageStatus::kInvalidGeometry;
    return;
  }
  const Point3 body_min{vehicle.body_min.x, vehicle.body_min.y,
    vehicle.body_min.z};
  const Point3 body_max{vehicle.body_max.x, vehicle.body_max.y,
    vehicle.body_max.z};
  if (body_max.x < body_min.x || body_max.y < body_min.y ||
    body_max.z < body_min.z)
  {
    *status = CorridorCoverageStatus::kInvalidGeometry;
    return;
  }
  for (double x = body_min.x; x <= body_max.x + 1e-9; x += spacing) {
    for (double y = body_min.y; y <= body_max.y + 1e-9; y += spacing) {
      for (double z = body_min.z; z <= body_max.z + 1e-9; z += spacing) {
        append(Point3{x, y, z});
        if (samples->size() > config.corridor_max_samples) {
          *status = CorridorCoverageStatus::kBudgetExhausted;
          return;
        }
      }
    }
  }
}

// Fills the set of fine cells that the swept vehicle body can touch. The sparse
// sample grid is expanded to the pose bounding box plus one sample spacing, so
// a rotated body cannot hide a corridor cell between samples.
CorridorCoverage CollectCorridorCoverage(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path,
  const LatestValidationConfig & config, std::set<FineCellIndex> * cells)
{
  CorridorCoverage coverage;
  if (cells == nullptr || !snapshot.fine_terrain()) {
    coverage.status = CorridorCoverageStatus::kInvalidGeometry;
    return coverage;
  }
  cells->clear();
  coverage.tile_fingerprints.clear();
  coverage.sampled_points = 0;
  coverage.cells = 0;
  const auto & terrain = *snapshot.fine_terrain();
  for (const PathPose & pose : path) {
    if (!pose.position.IsFinite()) {
      coverage.status = CorridorCoverageStatus::kInvalidGeometry;
      break;
    }
    CorridorCoverageStatus status =
      AddCorridorPoint(snapshot, pose.position, config, cells, &coverage);
    if (status != CorridorCoverageStatus::kComplete) {
      coverage.status = status;
      break;
    }
    std::vector<Point3> samples;
    AddBodyAndWheelSamples(config, pose, &samples, &status);
    if (status != CorridorCoverageStatus::kComplete) {
      coverage.status = status;
      break;
    }
    for (const Point3 & sample : samples) {
      status =
        AddCorridorPoint(snapshot, sample, config, cells, &coverage);
      if (status != CorridorCoverageStatus::kComplete) {
        break;
      }
    }
    if (status != CorridorCoverageStatus::kComplete) {
      coverage.status = status;
      break;
    }
    if (config.vehicle.body_max_configured &&
      config.vehicle.body_min.IsFinite() &&
      config.vehicle.body_max.IsFinite() && !samples.empty())
    {
      double min_x = samples.front().x;
      double max_x = samples.front().x;
      double min_y = samples.front().y;
      double max_y = samples.front().y;
      for (const Point3 & sample : samples) {
        min_x = std::min(min_x, sample.x);
        max_x = std::max(max_x, sample.x);
        min_y = std::min(min_y, sample.y);
        max_y = std::max(max_y, sample.y);
      }
      const double expansion =
        std::max(config.body_sweep_spacing_m, terrain.resolution());
      min_x -= expansion;
      max_x += expansion;
      min_y -= expansion;
      max_y += expansion;
      const auto min_cell =
        FineCellOf(
        Point3{min_x, min_y, 0.0}, terrain.origin_x(),
        terrain.origin_y(), terrain.resolution());
      const auto max_cell =
        FineCellOf(
        Point3{max_x, max_y, 0.0}, terrain.origin_x(),
        terrain.origin_y(), terrain.resolution());
      if (!min_cell.has_value() || !max_cell.has_value()) {
        coverage.status = CorridorCoverageStatus::kInvalidGeometry;
        break;
      }
      bool swept_truncated = false;
      for (std::int64_t x = std::min(min_cell->x, max_cell->x);
        x <= std::max(min_cell->x, max_cell->x) && !swept_truncated; ++x)
      {
        for (std::int64_t y = std::min(min_cell->y, max_cell->y);
          y <= std::max(min_cell->y, max_cell->y); ++y)
        {
          if (coverage.sampled_points >= config.corridor_max_samples) {
            swept_truncated = true;
            break;
          }
          ++coverage.sampled_points;
          cells->insert(FineCellIndex{x, y});
        }
      }
      if (swept_truncated) {
        coverage.status = CorridorCoverageStatus::kBudgetExhausted;
        break;
      }
    }
  }
  coverage.cells = cells->size();
  return coverage;
}

// Maps a failed PathValidator status to the explicit latest-validation status.
LatestValidationStatus ToLatestValidationStatus(
  PathValidationStatus status) noexcept
{
  switch (status) {
    case PathValidationStatus::kValid:
      return LatestValidationStatus::kOk;
    case PathValidationStatus::kInvalidPath:
    case PathValidationStatus::kChangedCorridor:
    case PathValidationStatus::kInsufficientSupport:
    case PathValidationStatus::kCollision:
    case PathValidationStatus::kUnknownObstacle:
      return LatestValidationStatus::kInvalid;
    case PathValidationStatus::kInvalidOptions:
      return LatestValidationStatus::kInvalidConfiguration;
    case PathValidationStatus::kNotReady:
      return LatestValidationStatus::kNotReady;
    case PathValidationStatus::kStaleTerrainRevision:
      return LatestValidationStatus::kStaleTerrain;
    case PathValidationStatus::kStaleContentHash:
      return LatestValidationStatus::kStaleContentHash;
    case PathValidationStatus::kMissingSupportNode:
      return LatestValidationStatus::kStaleSupportIds;
    case PathValidationStatus::kBudgetExhausted:
      return LatestValidationStatus::kBudgetExhausted;
    case PathValidationStatus::kCancelled:
      return LatestValidationStatus::kCanceled;
    case PathValidationStatus::kVehicleCapabilityUnconfigured:
      return LatestValidationStatus::kVehicleCapabilityUnconfigured;
  }
  return LatestValidationStatus::kInvalid;
}

}  // namespace

const char * LatestValidationStatusToString(
  LatestValidationStatus status) noexcept
{
  switch (status) {
    case LatestValidationStatus::kOk:
      return "ok";
    case LatestValidationStatus::kNotReady:
      return "not_ready";
    case LatestValidationStatus::kStaleTerrain:
      return "stale_terrain";
    case LatestValidationStatus::kStaleSupportIds:
      return "stale_support_ids";
    case LatestValidationStatus::kStaleContentHash:
      return "stale_content_hash";
    case LatestValidationStatus::kInvalid:
      return "invalid";
    case LatestValidationStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case LatestValidationStatus::kVehicleCapabilityUnconfigured:
      return "vehicle_capability_unconfigured";
    case LatestValidationStatus::kBudgetExhausted:
      return "budget_exhausted";
    case LatestValidationStatus::kCanceled:
      return "canceled";
    case LatestValidationStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case LatestValidationStatus::kStaleMap:
      return "stale_map";
  }
  return "unknown";
}

const char * LatestValidationResultClass(
  LatestValidationStatus status) noexcept
{
  switch (status) {
    case LatestValidationStatus::kOk:
      return "succeeded";
    case LatestValidationStatus::kNotReady:
      return "not_ready";
    case LatestValidationStatus::kStaleTerrain:
      return "stale_terrain_revision";
    case LatestValidationStatus::kStaleSupportIds:
      return "stale_support_ids";
    case LatestValidationStatus::kStaleContentHash:
      return "stale_content_hash";
    case LatestValidationStatus::kInvalid:
      return "latest_validation_invalid";
    case LatestValidationStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case LatestValidationStatus::kVehicleCapabilityUnconfigured:
      return "vehicle-capability-unconfigured";
    case LatestValidationStatus::kBudgetExhausted:
      return "latest_validation_budget_exhausted";
    case LatestValidationStatus::kCanceled:
      return "planner_canceled";
    case LatestValidationStatus::kDeadlineExceeded:
      return "latest_validation_deadline_exceeded";
    case LatestValidationStatus::kStaleMap:
      return "stale_map_kept_changing";
  }
  return "internal_error";
}

CorridorCoverage ComputeCorridorCoverage(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path,
  const LatestValidationConfig & config)
{
  std::set<FineCellIndex> cells;
  CorridorCoverage coverage =
    CollectCorridorCoverage(snapshot, path, config, &cells);
  if (coverage.status != CorridorCoverageStatus::kComplete) {
    return coverage;
  }
  const auto & terrain = *snapshot.fine_terrain();
  const double resolution = terrain.resolution();
  const double tile_size = std::max(config.corridor_tile_size_m, resolution);
  const std::int64_t cells_per_tile = std::max<std::int64_t>(
    1, static_cast<std::int64_t>(std::llround(tile_size / resolution)));
  for (const FineCellIndex & cell : cells) {
    const CorridorTileKey key{FloorDivide(cell.x, cells_per_tile),
      FloorDivide(cell.y, cells_per_tile)};
    std::uint64_t & hash = coverage.tile_fingerprints[key];
    if (hash == 0) {
      hash = 1469598103934665603ULL;
    }
    HashBytes(&hash, &cell.x, sizeof(cell.x));
    HashBytes(&hash, &cell.y, sizeof(cell.y));
    const auto * node_ids = terrain.FindNodesInCell(cell);
    if (node_ids == nullptr) {
      continue;
    }
    std::vector<TerrainNodeId> sorted_ids = *node_ids;
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const TerrainNodeId id : sorted_ids) {
      HashBytes(&hash, &id, sizeof(id));
      const ObstacleAttributes attributes = snapshot.MergedObstacle(id);
      HashAttributes(&hash, attributes);
    }
  }
  return coverage;
}

CorridorCommitTokens BuildCorridorCommitTokens(
  const MapSnapshot & snapshot, const std::vector<PathPose> & path,
  const LatestValidationConfig & config)
{
  CorridorCommitTokens tokens;
  if (!snapshot.fine_terrain() || !snapshot.HasReadyDistanceField() ||
    !snapshot.distance_field() || !snapshot.distance_field()->ready())
  {
    return tokens;
  }
  std::set<FineCellIndex> cells;
  const CorridorCoverage coverage =
    CollectCorridorCoverage(snapshot, path, config, &cells);
  if (coverage.status != CorridorCoverageStatus::kComplete) {
    return tokens;
  }
  tokens.terrain_revision = snapshot.fine_terrain()->revision();
  tokens.terrain_hash = snapshot.fine_terrain()->content_hash();
  const ObstacleTileLayer & static_layer =
    snapshot.static_obstacles().tiles_layer();
  const ObstacleTileLayer & dynamic_layer =
    snapshot.dynamic_obstacles().tiles_layer();
  const ObstacleDistanceField & field = *snapshot.distance_field();
  for (const FineCellIndex & cell : cells) {
    const ObstacleTileKey static_key = static_layer.TileKeyForCell(cell);
    const ObstacleTileKey dynamic_key = dynamic_layer.TileKeyForCell(cell);
    tokens.static_tiles.emplace(
      static_key,
      static_layer.FindTile(static_key).get());
    tokens.dynamic_tiles.emplace(
      dynamic_key,
      dynamic_layer.FindTile(dynamic_key).get());
    const ObstacleRecordTileKey record_key{
      FloorDivide(cell.x, kObstacleRecordTileCells),
      FloorDivide(cell.y, kObstacleRecordTileCells)};
    tokens.distance_tiles.emplace(
      record_key,
      field.FindRecordTile(record_key).get());
  }
  tokens.complete = true;
  return tokens;
}

bool CorridorCommitTokensMatch(
  const MapSnapshot & snapshot,
  const CorridorCommitTokens & tokens)
{
  if (!tokens.complete || !snapshot.IsReady() || !snapshot.fine_terrain() ||
    !snapshot.HasReadyDistanceField() || !snapshot.distance_field() ||
    !snapshot.distance_field()->ready())
  {
    return false;
  }
  if (snapshot.fine_terrain()->revision() != tokens.terrain_revision ||
    snapshot.fine_terrain()->content_hash() != tokens.terrain_hash)
  {
    return false;
  }
  const ObstacleTileLayer & static_layer =
    snapshot.static_obstacles().tiles_layer();
  const ObstacleTileLayer & dynamic_layer =
    snapshot.dynamic_obstacles().tiles_layer();
  const ObstacleDistanceField & field = *snapshot.distance_field();
  for (const auto & entry : tokens.static_tiles) {
    if (static_layer.FindTile(entry.first).get() != entry.second) {
      return false;
    }
  }
  for (const auto & entry : tokens.dynamic_tiles) {
    if (dynamic_layer.FindTile(entry.first).get() != entry.second) {
      return false;
    }
  }
  for (const auto & entry : tokens.distance_tiles) {
    if (field.FindRecordTile(entry.first).get() != entry.second) {
      return false;
    }
  }
  return true;
}

LatestValidationResult ValidateLatestSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & searched_snapshot,
  const std::vector<PathPose> & searched_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel,
  const std::function<std::shared_ptr<const MapSnapshot>()> & acquire_latest)
{
  LatestValidationResult result;
  result.path = searched_path;
  result.status = LatestValidationStatus::kNotReady;
  if (!searched_snapshot || !searched_snapshot->fine_terrain()) {
    result.detail = "searched snapshot is unavailable";
    return result;
  }

  const std::uint64_t searched_revision =
    searched_snapshot->fine_terrain()->revision();
  const std::uint64_t searched_hash =
    searched_snapshot->fine_terrain()->content_hash();
  PathValidator validator(config.vehicle, config.wheel, config.obstacles,
    config.validation);

  CorridorCoverage baseline =
    ComputeCorridorCoverage(*searched_snapshot, result.path, config);
  if (baseline.status == CorridorCoverageStatus::kInvalidGeometry) {
    result.status = LatestValidationStatus::kInvalid;
    result.detail =
      "searched corridor coverage has invalid geometry or body inputs";
    return result;
  }

  const auto validation_start = std::chrono::steady_clock::now();
  auto validation_deadline =
    validation_start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(
      config.max_latest_validation_time_sec));
  if (config.max_latest_validation_time_sec <= 0.0 ||
    validation_deadline > overall_deadline)
  {
    validation_deadline = overall_deadline;
  }

  // Converts a failed PathValidator result into an explicit latest-validation
  // status. A sample/deadline budget stop is reported as deadline when the
  // bounded validation deadline has actually elapsed, never as a map change.
  const auto assign_validation_failure =
    [&result, validation_deadline](const PathValidationResult & validation,
      const std::string & prefix) {
      result.status = ToLatestValidationStatus(validation.status);
      if (result.status == LatestValidationStatus::kBudgetExhausted &&
        std::chrono::steady_clock::now() >= validation_deadline)
      {
        result.status = LatestValidationStatus::kDeadlineExceeded;
      }
      result.detail = prefix + validation.detail;
    };

  SearchBudget validation_budget;
  validation_budget.has_deadline = true;
  validation_budget.deadline = validation_deadline;
  validation_budget.cancel = cancel;
  validation_budget.max_samples = config.validation.max_samples;

  for (int attempt = 0; attempt < config.max_latest_validation_retries;
    ++attempt)
  {
    result.attempts = static_cast<std::size_t>(attempt) + 1;
    if (cancel && cancel->load(std::memory_order_relaxed)) {
      result.status = LatestValidationStatus::kCanceled;
      result.detail = "canceled during latest-map validation";
      return result;
    }
    if (std::chrono::steady_clock::now() >= validation_deadline) {
      result.status = LatestValidationStatus::kDeadlineExceeded;
      result.detail = "latest-map validation deadline exceeded";
      return result;
    }

    // Exactly one acquisition per bounded iteration. The shared pointer is
    // retained while fingerprints and the full validator run on it.
    const auto latest = acquire_latest ? acquire_latest() : nullptr;
    if (!latest || !latest->IsReady() || !latest->fine_terrain()) {
      result.status = LatestValidationStatus::kNotReady;
      result.detail = "latest snapshot is not usable";
      return result;
    }
    const std::uint64_t latest_revision =
      latest->fine_terrain()->revision();
    const std::uint64_t latest_hash = latest->fine_terrain()->content_hash();

    // Terrain replacement invalidates every version-scoped TerrainNodeId. The
    // old path cannot be reprojected onto new IDs, so report a distinct stale
    // result rather than pretending to validate it.
    if (latest_revision != searched_revision ||
      latest_hash != searched_hash)
    {
      result.status = LatestValidationStatus::kStaleTerrain;
      result.detail =
        "terrain revision/content hash changed; version-scoped node IDs are "
        "invalid";
      return result;
    }
    if (!latest->HasReadyDistanceField() || !latest->distance_field() ||
      !latest->distance_field()->ready())
    {
      result.status = LatestValidationStatus::kNotReady;
      result.detail = "latest pinned distance field is not ready";
      return result;
    }

    const CorridorCoverage coverage =
      ComputeCorridorCoverage(*latest, result.path, config);
    if (coverage.status == CorridorCoverageStatus::kInvalidGeometry) {
      result.status = LatestValidationStatus::kInvalid;
      result.detail = "latest corridor coverage has invalid geometry";
      return result;
    }
    const bool baseline_budget =
      baseline.status == CorridorCoverageStatus::kBudgetExhausted;
    const bool coverage_budget =
      coverage.status == CorridorCoverageStatus::kBudgetExhausted;
    // Relevant-content confirmation: the corridor fingerprints, not the global
    // obstacle version, decide whether the map changed under the plan. A
    // second acquisition whose relevant tiles are unchanged is accepted even if
    // unrelated tiles kept receiving updates.
    if (attempt > 0 && !baseline_budget && !coverage_budget &&
      coverage.tile_fingerprints == baseline.tile_fingerprints)
    {
      result.ok = true;
      result.status = LatestValidationStatus::kOk;
      result.snapshot = latest;
      result.detail = "relevant corridor unchanged on confirmation";
      return result;
    }

    if (baseline_budget || coverage_budget) {
      // The relevant-tile token set cannot be completed within the bounded
      // corridor work budget. Fall back to the existing PathValidator over the
      // exact trajectory on this one pinned immutable snapshot. A stable long
      // route therefore succeeds instead of exhausting retries as a fake map
      // change. The returned shared pointer is the whole-snapshot proof.
      const PathValidationResult validation = validator.Validate(
        *latest, result.path, searched_revision, searched_hash,
        validation_budget);
      if (!validation.ok()) {
        assign_validation_failure(
          validation,
          "whole-snapshot validation failed: ");
        return result;
      }
      if (validation.updated_path.empty()) {
        result.status = LatestValidationStatus::kInvalid;
        result.detail = "whole-snapshot validator returned an empty trajectory";
        return result;
      }
      result.path = validation.updated_path;
      result.snapshot = latest;
      result.validated_snapshot = latest;
      result.replayed = true;
      result.whole_snapshot_validated = true;
      result.validated_samples = validation.checked_samples;
      result.ok = true;
      result.status = LatestValidationStatus::kOk;
      result.detail =
        "corridor budget exhausted; exact trajectory validated on one "
        "immutable whole snapshot";
      return result;
    }

    const bool related =
      coverage.tile_fingerprints != baseline.tile_fingerprints;
    if (related) {
      const PathValidationResult validation = validator.Validate(
        *latest, result.path, searched_revision, searched_hash,
        validation_budget);
      if (!validation.ok()) {
        assign_validation_failure(validation, "");
        return result;
      }
      if (validation.updated_path.empty()) {
        result.status = LatestValidationStatus::kInvalid;
        result.detail = "validator returned an empty trajectory";
        return result;
      }
      result.path = validation.updated_path;
      result.validated_snapshot = latest;
      result.replayed = true;
      result.validated_samples = validation.checked_samples;
      baseline = coverage;
    }

    result.snapshot = latest;
  }

  result.status = LatestValidationStatus::kStaleMap;
  result.detail =
    "bounded latest-map validation retries exhausted; map kept changing";
  return result;
}

LatestValidationResult ValidatePinnedSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & pinned_snapshot,
  const std::vector<PathPose> & searched_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel)
{
  LatestValidationResult result;
  result.path = searched_path;
  result.status = LatestValidationStatus::kNotReady;
  if (!pinned_snapshot || !pinned_snapshot->IsReady() ||
    !pinned_snapshot->fine_terrain() ||
    !pinned_snapshot->HasReadyDistanceField() ||
    !pinned_snapshot->distance_field() ||
    !pinned_snapshot->distance_field()->ready())
  {
    result.detail = "pinned snapshot A is not ready";
    return result;
  }
  if (searched_path.empty()) {
    result.status = LatestValidationStatus::kInvalid;
    result.detail = "pinned path is empty";
    return result;
  }
  if (!std::isfinite(config.max_latest_validation_time_sec) ||
    config.max_latest_validation_time_sec <= 0.0 ||
    config.validation.max_samples == 0)
  {
    result.status = LatestValidationStatus::kInvalidConfiguration;
    result.detail = "pinned validation budgets are invalid";
    return result;
  }
  if (cancel && cancel->load(std::memory_order_relaxed)) {
    result.status = LatestValidationStatus::kCanceled;
    result.detail = "canceled before pinned-snapshot validation";
    return result;
  }
  auto validation_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config.max_latest_validation_time_sec));
  validation_deadline = std::min(validation_deadline, overall_deadline);
  if (std::chrono::steady_clock::now() >= validation_deadline) {
    result.status = LatestValidationStatus::kDeadlineExceeded;
    result.detail = "pinned-snapshot validation deadline exceeded";
    return result;
  }
  SearchBudget budget;
  budget.has_deadline = true;
  budget.deadline = validation_deadline;
  budget.cancel = cancel;
  budget.max_samples = config.validation.max_samples;
  PathValidator validator(
    config.vehicle, config.wheel, config.obstacles, config.validation);
  const auto & terrain = *pinned_snapshot->fine_terrain();
  const PathValidationResult checked = validator.Validate(
    *pinned_snapshot, result.path, terrain.revision(), terrain.content_hash(),
    budget);
  result.attempts = 1;
  if (!checked.ok()) {
    result.status = ToLatestValidationStatus(checked.status);
    if (result.status == LatestValidationStatus::kBudgetExhausted &&
      std::chrono::steady_clock::now() >= validation_deadline)
    {
      result.status = LatestValidationStatus::kDeadlineExceeded;
    }
    result.detail = "pinned-snapshot validation failed: " + checked.detail;
    return result;
  }
  if (checked.updated_path.empty()) {
    result.status = LatestValidationStatus::kInvalid;
    result.detail = "pinned-snapshot validator returned an empty trajectory";
    return result;
  }
  result.path = checked.updated_path;
  result.snapshot = pinned_snapshot;
  result.validated_snapshot = pinned_snapshot;
  result.replayed = true;
  result.whole_snapshot_validated = true;
  result.validated_samples = checked.checked_samples;
  result.ok = true;
  result.status = LatestValidationStatus::kOk;
  result.detail = "path validated on the exact pinned snapshot A";
  return result;
}

LatestCommitProof BuildPinnedCommitProof(
  const LatestValidationResult & validation)
{
  LatestCommitProof proof;
  if (!validation.ok || !validation.snapshot ||
    !validation.snapshot->fine_terrain() || validation.path.empty())
  {
    return proof;
  }
  proof.kind = LatestCommitProofKind::kPinnedSnapshot;
  proof.snapshot = validation.snapshot;
  return proof;
}

LatestCommitProof BuildLatestCommitProof(
  const LatestValidationConfig & config,
  const LatestValidationResult & validation)
{
  LatestCommitProof proof;
  if (!validation.ok || !validation.snapshot ||
    !validation.snapshot->fine_terrain() || validation.path.empty())
  {
    return proof;
  }
  proof.snapshot = validation.snapshot;
  if (validation.geometric_graph_path) {
    if (validation.whole_snapshot_validated && validation.replayed &&
      validation.validated_snapshot.get() == validation.snapshot.get())
    {
      proof.kind = LatestCommitProofKind::kWholeSnapshot;
    }
    return proof;
  }
  CorridorCommitTokens tokens = BuildCorridorCommitTokens(
    *validation.snapshot, validation.path, config);
  if (tokens.complete) {
    proof.kind = LatestCommitProofKind::kRelevantTiles;
    proof.tokens = std::move(tokens);
    return proof;
  }
  // A truncated token set is acceptable only when the exact validated path was
  // fully run through PathValidator on this exact immutable snapshot. A later
  // relevant-tile confirmation may have advanced `validation.snapshot`; in that
  // case there is no whole-snapshot proof for the newer object.
  if (validation.replayed && validation.validated_snapshot != nullptr &&
    validation.snapshot.get() == validation.validated_snapshot.get())
  {
    proof.kind = LatestCommitProofKind::kWholeSnapshot;
  }
  return proof;
}

bool LatestCommitProofMatches(
  const LatestCommitProof & proof,
  const MapSnapshot & latest)
{
  switch (proof.kind) {
    case LatestCommitProofKind::kRelevantTiles:
      return CorridorCommitTokensMatch(latest, proof.tokens);
    case LatestCommitProofKind::kWholeSnapshot:
      // Exact object identity, not revision/version equality. The shared
      // pointer in the proof keeps the validated object alive, so a different
      // object cannot reuse its address before this comparison.
      return proof.snapshot != nullptr &&
             &latest == proof.snapshot.get() &&
             latest.fine_terrain().get() ==
             proof.snapshot->fine_terrain().get();
    case LatestCommitProofKind::kPinnedSnapshot:
      // Production Grid policy: A was pinned before search and is the plan's
      // immutable source. B may already be the store's latest snapshot; it must
      // not reject, revalidate, or starve the accepted A path.
      return proof.snapshot != nullptr;
    case LatestCommitProofKind::kNone:
      return false;
  }
  return false;
}

LatestValidationResult ValidateLatestGraphSnapshot(
  const LatestValidationConfig & config,
  const std::shared_ptr<const MapSnapshot> & searched_snapshot,
  const std::vector<PathPose> & searched_path,
  const std::vector<TerrainNodeId> & node_path,
  std::chrono::steady_clock::time_point overall_deadline,
  const std::shared_ptr<std::atomic<bool>> & cancel,
  const std::function<std::shared_ptr<const MapSnapshot>()> & acquire_latest)
{
  LatestValidationResult result;
  if (!searched_snapshot || !searched_snapshot->fine_terrain()) {
    result.detail = "searched graph snapshot is unavailable";
    return result;
  }
  if (searched_path.empty() || searched_path.size() != node_path.size()) {
    result.status = LatestValidationStatus::kInvalid;
    result.detail = "graph path and node IDs are empty or inconsistent";
    return result;
  }
  if (!std::isfinite(config.max_latest_validation_time_sec) ||
    config.max_latest_validation_time_sec <= 0.0 ||
    config.validation.max_samples == 0)
  {
    result.status = LatestValidationStatus::kInvalidConfiguration;
    result.detail = "graph validation budgets are invalid";
    return result;
  }
  SearchBudget budget;
  budget.has_deadline = true;
  budget.deadline = std::min(
    overall_deadline, std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config.max_latest_validation_time_sec)));
  budget.cancel = cancel;
  budget.max_samples = config.validation.max_samples;
  BudgetTracker tracker(budget);
  const auto check_budget = [&](bool consume) {
      const auto status = consume ? tracker.ConsumeSamples(1) : tracker.Check();
      if (status == BudgetTracker::Status::kOk) {
        return true;
      }
      result.status = status == BudgetTracker::Status::kCancelled ?
        LatestValidationStatus::kCanceled :
        (std::chrono::steady_clock::now() >= budget.deadline ?
        LatestValidationStatus::kDeadlineExceeded :
        LatestValidationStatus::kBudgetExhausted);
      result.detail = "graph validation canceled or budget/deadline exhausted";
      return false;
    };
  if (!check_budget(false)) {
    return result;
  }
  const auto latest = acquire_latest ? acquire_latest() : nullptr;
  if (!latest || !latest->IsReady() || !latest->fine_terrain() ||
    !latest->HasReadyDistanceField() || !latest->distance_field())
  {
    result.detail = "latest graph snapshot is unavailable";
    return result;
  }
  const auto & terrain = *latest->fine_terrain();
  if (terrain.revision() != searched_snapshot->fine_terrain()->revision() ||
    terrain.content_hash() != searched_snapshot->fine_terrain()->content_hash())
  {
    result.status = LatestValidationStatus::kStaleTerrain;
    result.detail = "terrain changed; graph node IDs are stale";
    return result;
  }
  const TerrainNode * previous = nullptr;
  double display_height_offset = 0.0;
  for (std::size_t index = 0; index < node_path.size(); ++index) {
    if (!check_budget(true)) {
      return result;
    }
    const auto * node = terrain.FindNode(node_path[index]);
    if (!node || !node->IsSupported() ||
      !latest->distance_field()->IsTraversable(node_path[index]))
    {
      result.status = LatestValidationStatus::kInvalid;
      result.detail = "graph node is missing, unsupported or blocked on latest map";
      return result;
    }
    const auto & point = searched_path[index].position;
    if (index == 0) {
      display_height_offset = point.z - node->representative.z;
    }
    if (!point.IsFinite() ||
      std::abs(point.x - node->representative.x) > 1.0e-6 ||
      std::abs(point.y - node->representative.y) > 1.0e-6 ||
      std::abs(point.z - node->representative.z - display_height_offset) > 1.0e-6)
    {
      result.status = LatestValidationStatus::kInvalid;
      result.detail = "graph path geometry does not match its node IDs";
      return result;
    }
    if (previous) {
      const auto * edge = terrain.FindConnection(previous->id, node->id);
      if (!edge || !edge->IsConnected()) {
        result.status = LatestValidationStatus::kInvalid;
        result.detail = "graph edge is missing or disconnected on latest map";
        return result;
      }
    }
    previous = node;
  }
  result.path = searched_path;
  for (std::size_t index = 0; index + 1 < result.path.size(); ++index) {
    const auto & next = result.path[index + 1].position;
    const auto & point = result.path[index].position;
    result.path[index].heading = std::atan2(next.y - point.y, next.x - point.x);
  }
  if (result.path.size() > 1) {
    result.path.back().heading = result.path[result.path.size() - 2].heading;
  }
  result.ok = true;
  result.status = LatestValidationStatus::kOk;
  result.snapshot = latest;
  result.validated_snapshot = latest;
  result.replayed = true;
  result.whole_snapshot_validated = true;
  result.geometric_graph_path = true;
  result.validated_samples = node_path.size();
  result.attempts = 1;
  result.detail = "geometric graph path validated; kinematic_checked=false";
  return result;
}

}  // namespace grid_global_planner
