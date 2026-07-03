#ifndef P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_H_
#define P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace p2p_move_base
{

struct LocalPoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct LocalObstacleLayerConfig
{
  double roi_forward = 6.0;
  double roi_backward = 1.0;
  double roi_left = 3.0;
  double roi_right = 3.0;
  double min_z = -0.15;
  double max_z = 2.0;
  double voxel_leaf_size = 0.08;
  std::size_t max_snapshot_points = 30000;
  double max_update_time_ms = 20.0;
  double stale_timeout_sec = 0.3;
  bool publish_debug_cloud = false;
  double footprint_front = 0.50;
  double footprint_back = 0.38;
  double footprint_left = 0.32;
  double footprint_right = 0.32;
};

struct LocalObstacleSnapshot
{
  double stamp_sec = 0.0;
  std::string frame_id;
  std::vector<LocalPoint> obstacle_points;
  std::size_t raw_points = 0;
  std::size_t filtered_points = 0;
  bool perception_stale = false;
  bool over_budget = false;
  double update_time_ms = 0.0;
};

class LocalObstacleLayer
{
public:
  LocalObstacleLayer();
  explicit LocalObstacleLayer(const LocalObstacleLayerConfig & config);

  void setConfig(const LocalObstacleLayerConfig & config);
  const LocalObstacleLayerConfig & config() const;

  std::shared_ptr<const LocalObstacleSnapshot> buildSnapshot(
    const std::vector<LocalPoint> & points,
    double stamp_sec,
    double now_sec,
    const std::string & frame_id) const;

private:
  bool inRoi(const LocalPoint & point) const;

  LocalObstacleLayerConfig config_;
};

class CollisionQuery
{
public:
  CollisionQuery(
    std::shared_ptr<const LocalObstacleSnapshot> snapshot,
    const LocalObstacleLayerConfig & config);

  bool footprintCollides(const Pose2D & pose) const;
  bool trajectoryCollides(const std::vector<Pose2D> & trajectory) const;
  double minDistanceToObstacle(const Pose2D & pose) const;

private:
  bool pointInsideFootprint(const LocalPoint & point, const Pose2D & pose) const;

  std::shared_ptr<const LocalObstacleSnapshot> snapshot_;
  LocalObstacleLayerConfig config_;
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_LOCAL_OBSTACLE_LAYER_H_
