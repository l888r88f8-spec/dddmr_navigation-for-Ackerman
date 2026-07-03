#include "p2p_move_base/local_obstacle_layer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace p2p_move_base
{
namespace
{
struct VoxelKey
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto tuple_hash = std::hash<std::int64_t>{}(key.x) ^
      (std::hash<std::int64_t>{}(key.y) << 1);
    return tuple_hash ^ (std::hash<std::int64_t>{}(key.z) << 2);
  }
};

VoxelKey makeVoxelKey(const LocalPoint & point, const double leaf_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / leaf_size)),
    static_cast<std::int64_t>(std::floor(point.y / leaf_size)),
    static_cast<std::int64_t>(std::floor(point.z / leaf_size))};
}

bool staleByTime(const double stamp_sec, const double now_sec, const double stale_timeout_sec)
{
  if (stale_timeout_sec <= 0.0 || !std::isfinite(stamp_sec) || !std::isfinite(now_sec)) {
    return false;
  }
  return (now_sec - stamp_sec) > stale_timeout_sec;
}

double horizontalDistance(const LocalPoint & point, const Pose2D & pose)
{
  return std::hypot(point.x - pose.x, point.y - pose.y);
}
}  // namespace

LocalObstacleLayer::LocalObstacleLayer() = default;

LocalObstacleLayer::LocalObstacleLayer(const LocalObstacleLayerConfig & config)
: config_(config)
{
}

void LocalObstacleLayer::setConfig(const LocalObstacleLayerConfig & config)
{
  config_ = config;
}

const LocalObstacleLayerConfig & LocalObstacleLayer::config() const
{
  return config_;
}

std::shared_ptr<const LocalObstacleSnapshot> LocalObstacleLayer::buildSnapshot(
  const std::vector<LocalPoint> & points,
  const double stamp_sec,
  const double now_sec,
  const std::string & frame_id) const
{
  const auto start = std::chrono::steady_clock::now();
  auto snapshot = std::make_shared<LocalObstacleSnapshot>();
  snapshot->stamp_sec = stamp_sec;
  snapshot->frame_id = frame_id;
  snapshot->raw_points = points.size();
  snapshot->perception_stale = staleByTime(stamp_sec, now_sec, config_.stale_timeout_sec);

  std::vector<LocalPoint> filtered_points;
  filtered_points.reserve(std::min(points.size(), config_.max_snapshot_points));

  if (config_.voxel_leaf_size > 0.0 && std::isfinite(config_.voxel_leaf_size)) {
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    occupied_voxels.reserve(points.size());
    for (const auto & point : points) {
      if (!inRoi(point)) {
        continue;
      }
      const auto key = makeVoxelKey(point, config_.voxel_leaf_size);
      if (!occupied_voxels.insert(key).second) {
        continue;
      }
      filtered_points.push_back(point);
    }
  } else {
    for (const auto & point : points) {
      if (inRoi(point)) {
        filtered_points.push_back(point);
      }
    }
  }

  snapshot->over_budget = filtered_points.size() > config_.max_snapshot_points;
  const auto retained_count = std::min(filtered_points.size(), config_.max_snapshot_points);
  snapshot->obstacle_points.reserve(retained_count);
  for (std::size_t index = 0; index < retained_count; ++index) {
    snapshot->obstacle_points.push_back(filtered_points[index]);
  }
  snapshot->filtered_points = snapshot->obstacle_points.size();

  const auto elapsed = std::chrono::steady_clock::now() - start;
  snapshot->update_time_ms = std::chrono::duration<double, std::milli>(elapsed).count();
  if (config_.max_update_time_ms > 0.0 &&
    snapshot->update_time_ms > config_.max_update_time_ms)
  {
    snapshot->over_budget = true;
  }

  return snapshot;
}

bool LocalObstacleLayer::inRoi(const LocalPoint & point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  return point.x >= -config_.roi_backward &&
         point.x <= config_.roi_forward &&
         point.y >= -config_.roi_right &&
         point.y <= config_.roi_left &&
         point.z >= config_.min_z &&
         point.z <= config_.max_z;
}

CollisionQuery::CollisionQuery(
  std::shared_ptr<const LocalObstacleSnapshot> snapshot,
  const LocalObstacleLayerConfig & config)
: snapshot_(std::move(snapshot)), config_(config)
{
}

bool CollisionQuery::footprintCollides(const Pose2D & pose) const
{
  if (!snapshot_) {
    return false;
  }
  for (const auto & point : snapshot_->obstacle_points) {
    if (pointInsideFootprint(point, pose)) {
      return true;
    }
  }
  return false;
}

bool CollisionQuery::trajectoryCollides(const std::vector<Pose2D> & trajectory) const
{
  for (const auto & pose : trajectory) {
    if (footprintCollides(pose)) {
      return true;
    }
  }
  return false;
}

double CollisionQuery::minDistanceToObstacle(const Pose2D & pose) const
{
  if (!snapshot_ || snapshot_->obstacle_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto & point : snapshot_->obstacle_points) {
    min_distance = std::min(min_distance, horizontalDistance(point, pose));
  }
  return min_distance;
}

bool CollisionQuery::pointInsideFootprint(const LocalPoint & point, const Pose2D & pose) const
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    return false;
  }

  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  return local_x >= -config_.footprint_back &&
         local_x <= config_.footprint_front &&
         local_y >= -config_.footprint_right &&
         local_y <= config_.footprint_left;
}

}  // namespace p2p_move_base
