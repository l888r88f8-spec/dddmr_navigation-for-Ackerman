// Copyright 2026 DDDMobileRobot
//
// Implementation of the immutable SceneSnapshot envelope and its owned static /
// cost views. This translation unit has no ROS dependency; PCL is used only for
// owned point clouds and query trees.

#include <perception_3d/scene_snapshot.h>

#include <cstring>
#include <limits>
#include <utility>

namespace perception_3d
{
namespace
{

constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void HashBytes(
  std::uint64_t * hash, const void * data, std::size_t size) noexcept
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= static_cast<std::uint64_t>(bytes[i]);
    *hash *= kFnvPrime;
  }
}

template<typename T>
void HashValue(std::uint64_t * hash, const T & value) noexcept
{
  HashBytes(hash, &value, sizeof(T));
}

void HashCloud(
  std::uint64_t * hash,
  const pcl::PointCloud<pcl::PointXYZI> & cloud) noexcept
{
  // The separator prevents two differently grouped clouds from colliding when
  // their concatenated coordinate bytes happen to be equal.
  const std::uint64_t count = static_cast<std::uint64_t>(cloud.points.size());
  HashValue(hash, count);
  const std::uint64_t marker = 0x9E3779B97F4A7C15ULL;
  HashValue(hash, marker);
  for (const pcl::PointXYZI & point : cloud.points) {
    HashValue(hash, point.x);
    HashValue(hash, point.y);
    HashValue(hash, point.z);
    HashValue(hash, point.intensity);
  }
}

void SetDetail(std::string * detail, const std::string & value)
{
  if (detail != nullptr) {
    *detail = value;
  }
}

}  // namespace

LegacyStaticGraph::LegacyStaticGraph(
  std::vector<edges_t> edges,
  std::vector<float> node_weights)
: edges_(std::move(edges)),
  node_weights_(std::move(node_weights)) {}

const edges_t & LegacyStaticGraph::getEdge(unsigned int node) const noexcept
{
  static const edges_t kEmpty;
  return node < edges_.size() ? edges_[node] : kEmpty;
}

float LegacyStaticGraph::getNodeWeight(unsigned int node) const noexcept
{
  return node < node_weights_.size() ? node_weights_[node] : 0.0F;
}

LegacyStaticView::LegacyStaticView(
  std::string frame_id, SourceMapIdentity identity,
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> ground_points,
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> map_points,
  std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> ground_kdtree,
  std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> map_kdtree,
  LegacyStaticGraph graph)
: frame_id_(std::move(frame_id)),
  identity_(std::move(identity)),
  ground_points_(std::move(ground_points)),
  map_points_(std::move(map_points)),
  ground_kdtree_(std::move(ground_kdtree)),
  map_kdtree_(std::move(map_kdtree)),
  graph_(std::move(graph)) {}

std::size_t LegacyStaticView::groundPointCount() const noexcept
{
  return ground_points_ ? ground_points_->points.size() : 0U;
}

std::size_t LegacyStaticView::mapPointCount() const noexcept
{
  return map_points_ ? map_points_->points.size() : 0U;
}

LegacyCostView::LegacyCostView(
  std::string frame_id, SourceMapIdentity static_identity,
  std::uint64_t observation_revision,
  std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
  bool sensors_current, bool complete,
  std::shared_ptr<const LegacyStaticView> static_view,
  std::vector<double> min_dgraph,
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> lethal_points)
: frame_id_(std::move(frame_id)),
  static_identity_(std::move(static_identity)),
  observation_revision_(observation_revision),
  sensor_stamp_ns_(sensor_stamp_ns),
  received_steady_ns_(received_steady_ns),
  sensors_current_(sensors_current),
  complete_(complete),
  static_view_(std::move(static_view)),
  min_dgraph_(std::move(min_dgraph)),
  lethal_points_(std::move(lethal_points)) {}

double LegacyCostView::minDGraphValue(std::size_t index) const noexcept
{
  return index < min_dgraph_.size() ? min_dgraph_[index] : 9999.0;
}

SceneSnapshot::SceneSnapshot(
  std::string frame_id, SourceMapIdentity source_map,
  std::uint64_t observation_revision,
  std::int64_t sensor_stamp_ns, std::int64_t cycle_stamp_ns,
  std::int64_t received_steady_ns,
  bool sensors_current, bool complete,
  std::shared_ptr<const PerceptionSnapshot> evidence,
  std::shared_ptr<const LegacyStaticView> legacy_static,
  std::shared_ptr<const LegacyCostView> legacy_cost,
  std::string detail)
: frame_id_(std::move(frame_id)),
  source_map_(std::move(source_map)),
  observation_revision_(observation_revision),
  sensor_stamp_ns_(sensor_stamp_ns),
  cycle_stamp_ns_(cycle_stamp_ns),
  received_steady_ns_(received_steady_ns),
  sensors_current_(sensors_current),
  complete_(complete),
  evidence_(std::move(evidence)),
  legacy_static_(std::move(legacy_static)),
  legacy_cost_(std::move(legacy_cost)),
  detail_(std::move(detail)) {}

bool SceneSnapshot::IsFresh(
  std::int64_t now_steady_ns, std::int64_t max_receipt_age_ns,
  std::string * detail) const
{
  if (!complete_) {
    SetDetail(
      detail, detail_.empty() ? "scene snapshot is incomplete" : detail_);
    return false;
  }
  if (!sensors_current_) {
    SetDetail(detail, "scene snapshot has an expired or missing sensor");
    return false;
  }
  if (!hasEvidence() && !hasLegacyCost() && !hasLegacyStatic()) {
    SetDetail(detail, "scene snapshot has no requested view");
    return false;
  }
  if (received_steady_ns_ <= 0) {
    SetDetail(detail, "scene snapshot has a missing receipt stamp");
    return false;
  }
  if (max_receipt_age_ns < 0) {
    SetDetail(detail, "negative max receipt age is invalid");
    return false;
  }
  if (now_steady_ns < received_steady_ns_) {
    SetDetail(detail, "scene snapshot receipt time is in the future");
    return false;
  }
  if (now_steady_ns - received_steady_ns_ > max_receipt_age_ns) {
    SetDetail(detail, "scene snapshot is stale");
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

std::uint64_t ComputePointCloudContentHash(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::string & frame_id) noexcept
{
  std::uint64_t hash = kFnvOffset;
  const std::uint64_t frame_size =
    static_cast<std::uint64_t>(frame_id.size());
  HashValue(&hash, frame_size);
  HashBytes(&hash, frame_id.data(), frame_id.size());
  HashCloud(&hash, cloud);
  return hash;
}

std::uint64_t ComputeStaticContentHash(
  const pcl::PointCloud<pcl::PointXYZI> & ground,
  const pcl::PointCloud<pcl::PointXYZI> & map,
  const std::string & frame_id) noexcept
{
  std::uint64_t hash = kFnvOffset;
  const std::uint64_t frame_size =
    static_cast<std::uint64_t>(frame_id.size());
  HashValue(&hash, frame_size);
  HashBytes(&hash, frame_id.data(), frame_id.size());
  HashCloud(&hash, ground);
  HashCloud(&hash, map);
  return hash;
}

}  // namespace perception_3d
