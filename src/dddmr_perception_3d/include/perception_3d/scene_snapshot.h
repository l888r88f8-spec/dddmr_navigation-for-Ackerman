// Copyright 2026 DDDMobileRobot
//
// Unified immutable in-process perception scene envelope.
//
// SceneSnapshot is the single hand-off object for the optional evidence view
// (the compatibility PerceptionSnapshot) and the two original-planner views:
// an owned immutable static map/graph view and an on-demand fused cost view.
// It never contains Grid MapSnapshot/terrain types and never aliases mutable
// SharedData or plugin PCL storage.
//
// Revision semantics:
// - SourceMapIdentity::revision changes only on an accepted static event
//   (initial map+ground, or detected changed static content). It is 0 when
//   unknown and never changes because of a lidar scan.
// - content_hash covers the frame, both point clouds, their counts and their
//   original order, so a same-size replacement is a new identity.
// - SceneSnapshot::observationRevision is an opaque FNV-1a equality token over
//   the COMPLETED producer state of every dynamic source. It is not a
//   monotonic counter and must only be compared for equality. Ordering uses
//   each source observation's source_sequence, never this token. Accepting a
//   new lidar cloud does not change it; a completed clear/mark/reset mutation
//   does. Repeated polls of the same completed state keep the same token and
//   must not mint a new source receipt.

#ifndef PERCEPTION_3D_SCENE_SNAPSHOT_H_
#define PERCEPTION_3D_SCENE_SNAPSHOT_H_

#include <perception_3d/perception_snapshot.h>
#include <perception_3d/static_graph.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace perception_3d
{

// Identity of the accepted static source map. This is deliberately independent
// of Grid terrain revisions and old GroundMap IDs.
struct SourceMapIdentity
{
  std::string frame_id;
  std::uint64_t revision{0};
  std::uint64_t content_hash{0};
  std::size_t ground_point_count{0};
  std::size_t map_point_count{0};
  bool known{false};

  bool SameContent(const SourceMapIdentity & other) const noexcept
  {
    return known && other.known && revision == other.revision &&
           content_hash == other.content_hash && frame_id == other.frame_id &&
           ground_point_count == other.ground_point_count &&
           map_point_count == other.map_point_count;
  }
};

// Selects which optional views an on-demand capture should freeze.
// include_legacy_static is appended so the existing aggregate initializers
// {include_evidence, include_legacy_cost} keep their meaning and there is no
// layout change to those two flags. A static-only request reuses/builds the
// immutable LegacyStaticView but never evaluates any dynamic dGraph value.
struct SceneSnapshotRequest
{
  bool include_evidence{false};
  bool include_legacy_cost{false};
  bool include_legacy_static{false};
};

// Explicit producer mode. All flags default off, so an old node does no new
// snapshot preparation/traversal/cache maintenance.
struct SceneProducerConfig
{
  bool enabled{false};
  bool evidence{false};
  bool legacy_cost{false};
  std::string static_source_name{"map"};
};

// Deep-copied immutable static graph. It preserves the original edge-set
// ordering (std::set<edge_t>) and float precision.
class LegacyStaticGraph
{
public:
  LegacyStaticGraph() = default;
  LegacyStaticGraph(
    std::vector<edges_t> edges,
    std::vector<float> node_weights);

  // Empty set for an out-of-range node, matching a missing unordered_map entry.
  const edges_t & getEdge(unsigned int node) const noexcept;
  // 0.0 for a missing weight, matching the old operator[] default.
  float getNodeWeight(unsigned int node) const noexcept;

  std::size_t nodeCount() const noexcept {return edges_.size();}
  std::size_t nodeWeightCount() const noexcept {return node_weights_.size();}

private:
  std::vector<edges_t> edges_;
  std::vector<float> node_weights_;
};

// Immutable owned view of the original planner's static map resources.
// ground_points keeps the original point order and carries the original PCL
// intensity; the FLANN trees are built from these owned copies exactly once per
// accepted static identity.
class LegacyStaticView
{
public:
  LegacyStaticView(
    std::string frame_id, SourceMapIdentity identity,
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> ground_points,
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> map_points,
    std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> ground_kdtree,
    std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> map_kdtree,
    LegacyStaticGraph graph);

  const std::string & frame_id() const noexcept {return frame_id_;}
  const SourceMapIdentity & identity() const noexcept {return identity_;}
  const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &
  groundPoints() const noexcept {return ground_points_;}
  const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &
  mapPoints() const noexcept {return map_points_;}
  const std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> &
  groundKdtree() const noexcept {return ground_kdtree_;}
  const std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> &
  mapKdtree() const noexcept {return map_kdtree_;}
  const LegacyStaticGraph & graph() const noexcept {return graph_;}
  std::size_t groundPointCount() const noexcept;
  std::size_t mapPointCount() const noexcept;

private:
  std::string frame_id_;
  SourceMapIdentity identity_;
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> ground_points_;
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> map_points_;
  std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> ground_kdtree_;
  std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> map_kdtree_;
  LegacyStaticGraph graph_;
};

// Immutable on-demand original-planner dynamic view. min_dgraph has exactly one
// finite fused minimum per original ground index, in the same order as the
// bound static view. lethal_points are owned copies for line-of-sight queries.
class LegacyCostView
{
public:
  LegacyCostView(
    std::string frame_id, SourceMapIdentity static_identity,
    std::uint64_t observation_revision,
    std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
    bool sensors_current, bool complete,
    std::shared_ptr<const LegacyStaticView> static_view,
    std::vector<double> min_dgraph,
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> lethal_points);

  const std::string & frame_id() const noexcept {return frame_id_;}
  const SourceMapIdentity & staticIdentity() const noexcept
  {
    return static_identity_;
  }
  std::uint64_t observationRevision() const noexcept
  {
    return observation_revision_;
  }
  std::int64_t sensorStampNs() const noexcept {return sensor_stamp_ns_;}
  std::int64_t receivedSteadyNs() const noexcept {return received_steady_ns_;}
  bool sensorsCurrent() const noexcept {return sensors_current_;}
  bool complete() const noexcept {return complete_;}
  const std::shared_ptr<const LegacyStaticView> & staticView() const noexcept
  {
    return static_view_;
  }
  const std::vector<double> & minDGraph() const noexcept {return min_dgraph_;}
  double minDGraphValue(std::size_t index) const noexcept;
  const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &
  lethalPoints() const noexcept {return lethal_points_;}

private:
  std::string frame_id_;
  SourceMapIdentity static_identity_;
  std::uint64_t observation_revision_{0};
  std::int64_t sensor_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  bool sensors_current_{false};
  bool complete_{false};
  std::shared_ptr<const LegacyStaticView> static_view_;
  std::vector<double> min_dgraph_;
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> lethal_points_;
};

// Complete immutable scene envelope. At least one optional view may be absent;
// consumers must check exactly the view they requested.
class SceneSnapshot
{
public:
  SceneSnapshot(
    std::string frame_id, SourceMapIdentity source_map,
    std::uint64_t observation_revision,
    std::int64_t sensor_stamp_ns, std::int64_t cycle_stamp_ns,
    std::int64_t received_steady_ns,
    bool sensors_current, bool complete,
    std::shared_ptr<const PerceptionSnapshot> evidence,
    std::shared_ptr<const LegacyStaticView> legacy_static,
    std::shared_ptr<const LegacyCostView> legacy_cost,
    std::string detail);

  const std::string & frame_id() const noexcept {return frame_id_;}
  const SourceMapIdentity & sourceMap() const noexcept {return source_map_;}
  std::uint64_t observationRevision() const noexcept
  {
    return observation_revision_;
  }
  std::int64_t sensorStampNs() const noexcept {return sensor_stamp_ns_;}
  std::int64_t cycleStampNs() const noexcept {return cycle_stamp_ns_;}
  std::int64_t receivedSteadyNs() const noexcept
  {
    return received_steady_ns_;
  }
  bool sensorsCurrent() const noexcept {return sensors_current_;}
  bool complete() const noexcept {return complete_;}
  const std::shared_ptr<const PerceptionSnapshot> & evidence() const noexcept
  {
    return evidence_;
  }
  const std::shared_ptr<const LegacyStaticView> & legacyStatic() const noexcept
  {
    return legacy_static_;
  }
  const std::shared_ptr<const LegacyCostView> & legacyCost() const noexcept
  {
    return legacy_cost_;
  }
  bool hasEvidence() const noexcept {return evidence_ != nullptr;}
  bool hasLegacyStatic() const noexcept {return legacy_static_ != nullptr;}
  bool hasLegacyCost() const noexcept {return legacy_cost_ != nullptr;}
  const std::string & detail() const noexcept {return detail_;}

  // True only for a complete scene with current sensors and at least one view,
  // and a receipt age within max_receipt_age_ns. Negative age is not fresh.
  bool IsFresh(
    std::int64_t now_steady_ns, std::int64_t max_receipt_age_ns,
    std::string * detail = nullptr) const;

private:
  std::string frame_id_;
  SourceMapIdentity source_map_;
  std::uint64_t observation_revision_{0};
  std::int64_t sensor_stamp_ns_{0};
  std::int64_t cycle_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  bool sensors_current_{false};
  bool complete_{false};
  std::shared_ptr<const PerceptionSnapshot> evidence_;
  std::shared_ptr<const LegacyStaticView> legacy_static_;
  std::shared_ptr<const LegacyCostView> legacy_cost_;
  std::string detail_;
};

// Stable content fingerprint of an accepted static source map. It includes the
// frame, both clouds, their counts and original point order. It is not a
// cryptographic hash and must not be compared with Grid terrain revisions.
std::uint64_t ComputeStaticContentHash(
  const pcl::PointCloud<pcl::PointXYZI> & ground,
  const pcl::PointCloud<pcl::PointXYZI> & map,
  const std::string & frame_id) noexcept;

// Per-cloud fingerprint used by StaticLayer to detect a same-size replacement
// before the coherent static event is applied.
std::uint64_t ComputePointCloudContentHash(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::string & frame_id) noexcept;

}  // namespace perception_3d

#endif  // PERCEPTION_3D_SCENE_SNAPSHOT_H_
