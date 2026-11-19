// Copyright 2026 DDDMobileRobot
//
// Read-only bridge between an immutable SceneSnapshot and the original A*
// search code. It never aliases live SharedData and never silently substitutes
// a live view when a requested snapshot is incomplete or inconsistently bound.

#ifndef GLOBAL_PLANNER_LEGACY_SCENE_ADAPTER_H_
#define GLOBAL_PLANNER_LEGACY_SCENE_ADAPTER_H_

#include <global_planner/nanoflann_pcl.hpp>

#include <perception_3d/scene_snapshot.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace global_planner
{

// Search parameters that used to be read from Perception3D_ROS::GlobalUtils
// while a search was running. The caller pins them together with the scene so
// the frozen backend performs no live perceptor reads at all.
struct LegacySearchParameters
{
  double inscribed_radius{0.0};
  double inflation_descending_rate{0.0};
  double max_obstacle_distance{0.0};
};

// Optional observable trace for CPU regressions. Production callers pass
// nullptr, so this adds no search work to the normal path.
struct AStarPlanTrace
{
  std::vector<unsigned int> expanded_nodes;
  std::vector<unsigned int> neighbor_order;
  double goal_g{0.0};
  bool found{false};

  void reset();
};

// Validates that all views requested from the envelope are complete and bound
// to one source-map identity, one frame, one observation token and one static
// view pointer. On failure both outputs are cleared and rejection_reason
// explains the refusal. This function performs no fallback to live data.
bool BuildLegacySceneReadView(
  const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot,
  const LegacySearchParameters & parameters,
  std::shared_ptr<const perception_3d::LegacyStaticView> * static_view,
  std::shared_ptr<const perception_3d::LegacyCostView> * cost_view,
  std::string * rejection_reason);

// Builds one read-only LOS index over the cost view's owned lethal cloud. The
// returned tree is cached by the caller for the lifetime of the pinned view;
// no point copies occur per LOS sample. Null means the lethal set is empty.
std::shared_ptr<const nanoflann::KdTreeFLANN<pcl::PointXYZI>>
BuildLegacyLethalKdTree(
  const std::shared_ptr<const perception_3d::LegacyCostView> & cost_view);

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_LEGACY_SCENE_ADAPTER_H_
