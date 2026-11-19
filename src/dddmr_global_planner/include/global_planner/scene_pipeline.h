// Copyright 2026 DDDMobileRobot
//
// Pinned, immutable scene context and pure query/validation helpers for the
// original GlobalPlanner route pipeline.
//
// The context binds exactly one complete SceneSnapshot's LegacyStaticView and
// LegacyCostView plus the former live search parameters. Once bound, every
// projection, graph/pre-graph search, hybrid search, LOS/smoothing query and
// latest-path validation reads only these immutable views. There is no
// fallback to SharedData, get_min_dGraphValue() or aggregateLethal().

#ifndef GLOBAL_PLANNER_SCENE_PIPELINE_H_
#define GLOBAL_PLANNER_SCENE_PIPELINE_H_

#include <global_planner/legacy_scene_adapter.h>

#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace global_planner
{

class ScenePipelineContext
{
public:
  ScenePipelineContext() = default;

  // Validates and pins one complete snapshot. On failure the context is reset
  // and rejection_reason explains the refusal; no live view is substituted.
  bool Bind(
    const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot,
    const LegacySearchParameters & parameters,
    std::string * rejection_reason = nullptr);

  void Reset();

  bool valid() const noexcept {return valid_;}
  const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot() const noexcept
  {
    return snapshot_;
  }
  const std::shared_ptr<const perception_3d::LegacyStaticView> & staticView() const noexcept
  {
    return static_view_;
  }
  const std::shared_ptr<const perception_3d::LegacyCostView> & costView() const noexcept
  {
    return cost_view_;
  }
  const LegacySearchParameters & parameters() const noexcept {return parameters_;}
  const std::string & rejectionReason() const noexcept {return rejection_reason_;}

  // Static identity includes frame, both cloud counts and the content hash.
  // Index binding is only valid while this is true.
  bool SameStaticMap(const ScenePipelineContext & other) const noexcept;

  // Observation identity is an opaque FNV equality token. A zero/unknown token
  // is never treated as equal. Equality is meaningful only for the same static
  // identity.
  bool SameObservation(const ScenePipelineContext & other) const noexcept;

  bool SamePinnedViews(const ScenePipelineContext & other) const noexcept;

  // Original start/goal ground radius search against the pinned static view.
  bool ProjectOnGround(
    const pcl::PointXYZI & query,
    double search_radius,
    std::vector<int> * indices,
    std::vector<float> * squared_distances) const;

  double FusedClearance(std::size_t ground_index) const noexcept;
  bool HasFusedClearance(std::size_t ground_index) const noexcept;

private:
  bool valid_{false};
  std::shared_ptr<const perception_3d::SceneSnapshot> snapshot_;
  std::shared_ptr<const perception_3d::LegacyStaticView> static_view_;
  std::shared_ptr<const perception_3d::LegacyCostView> cost_view_;
  LegacySearchParameters parameters_;
  std::string rejection_reason_;
};

// Observable relation between a route's pinned scene and the latest captured
// scene. It reports root sensor status; it never changes the stale policy.
struct SceneRelation
{
  bool latest_available{false};
  bool static_map_same{false};
  bool observation_same{false};
  bool sensors_current{false};
  std::string detail;
};

SceneRelation RelatePinnedToLatest(
  const ScenePipelineContext & pinned,
  const ScenePipelineContext & latest);

struct ScenePathValidation
{
  enum class Result
  {
    kNotReady,
    kAccepted,
    kDynamicConflictReplan,
    kStaticMismatchReject,
  };

  Result result{Result::kNotReady};
  bool static_map_same{false};
  bool observation_same{false};
  bool dynamic_changed{false};
  bool path_affected{false};
  std::size_t conflict_sample_index{0};
  double conflict_x{0.0};
  double conflict_y{0.0};
  double conflict_z{0.0};
  std::string reason;
};

// Validates a complete latest pinned scene against the route built from the
// pinned scene. Same static + same observation token accepts immediately.
// Otherwise every path segment is densely sampled so a sparse pose pair cannot
// skip an obstacle. Collision checks reuse the original thresholds:
// - frozen lethal cloud radiusSearch(sample, 2 * inscribed_radius), > 1 hit;
// - nearest ground fused cost < inscribed_radius.
// No live perception read is performed.
ScenePathValidation ValidatePathAgainstLatest(
  const ScenePipelineContext & pinned,
  const ScenePipelineContext & latest,
  const nav_msgs::msg::Path & path,
  double sample_step);

// Deterministic dense polyline used by validation and tests. Endpoints are
// included exactly once; intermediate points are inserted at most sample_step
// apart on every segment.
std::vector<pcl::PointXYZI> SamplePathForValidation(
  const nav_msgs::msg::Path & path,
  double sample_step);

// Original bounded Ackermann smoothing, expressed against one pinned static
// view so it can be unit-tested without a ROS node. The tunables are passed
// explicitly; the math is the original implementation unchanged.
std::size_t SmoothPathAgainstScene(
  nav_msgs::msg::Path * path,
  const ScenePipelineContext & scene,
  bool enable,
  int iterations,
  double weight,
  double max_shift,
  double lock_start_distance,
  double lock_goal_distance,
  double ground_search_radius);

// Original orientation refresh shared by smoothing and hybrid composition.
void RefreshPathOrientations(nav_msgs::msg::Path * path);

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_SCENE_PIPELINE_H_
