// Copyright 2026 DDDMobileRobot

#include <global_planner/legacy_scene_adapter.h>

#include <cmath>
#include <utility>

namespace global_planner
{

void AStarPlanTrace::reset()
{
  expanded_nodes.clear();
  neighbor_order.clear();
  goal_g = 0.0;
  found = false;
}

namespace
{

bool Reject(std::string * reason, const std::string & detail)
{
  if(reason != nullptr){
    *reason = detail;
  }
  return false;
}

}  // namespace

bool BuildLegacySceneReadView(
  const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot,
  const LegacySearchParameters & parameters,
  std::shared_ptr<const perception_3d::LegacyStaticView> * static_view,
  std::shared_ptr<const perception_3d::LegacyCostView> * cost_view,
  std::string * rejection_reason)
{
  (void)parameters;
  if(static_view != nullptr){
    static_view->reset();
  }
  if(cost_view != nullptr){
    cost_view->reset();
  }

  if(!snapshot){
    return Reject(rejection_reason, "scene snapshot is null");
  }
  if(!snapshot->complete()){
    const std::string detail = snapshot->detail().empty() ?
      "scene snapshot is incomplete" :
      "scene snapshot is incomplete: " + snapshot->detail();
    return Reject(rejection_reason, detail);
  }

  const perception_3d::SourceMapIdentity & source = snapshot->sourceMap();
  if(!source.known){
    return Reject(rejection_reason, "scene source-map identity is unknown");
  }
  if(source.ground_point_count == 0 || source.map_point_count == 0){
    return Reject(rejection_reason, "scene source-map identity has empty clouds");
  }
  if(snapshot->frame_id().empty() || source.frame_id != snapshot->frame_id()){
    return Reject(
      rejection_reason, "scene frame does not match the source-map identity");
  }

  const std::shared_ptr<const perception_3d::LegacyStaticView> & pinned_static =
    snapshot->legacyStatic();
  const std::shared_ptr<const perception_3d::LegacyCostView> & pinned_cost =
    snapshot->legacyCost();
  if(!pinned_static){
    return Reject(rejection_reason, "requested legacy static view is absent");
  }
  if(!pinned_cost){
    return Reject(rejection_reason, "requested legacy cost view is absent");
  }
  if(!pinned_cost->complete()){
    return Reject(rejection_reason, "legacy cost view is incomplete");
  }

  if(pinned_static->frame_id() != snapshot->frame_id()){
    return Reject(
      rejection_reason, "legacy static view frame does not match the scene");
  }
  if(pinned_cost->frame_id() != snapshot->frame_id()){
    return Reject(
      rejection_reason, "legacy cost view frame does not match the scene");
  }
  if(!pinned_static->identity().SameContent(source)){
    return Reject(
      rejection_reason,
      "legacy static identity does not match the scene source-map identity");
  }
  if(!pinned_cost->staticIdentity().SameContent(source)){
    return Reject(
      rejection_reason,
      "legacy cost identity does not match the scene source-map identity");
  }
  if(pinned_cost->staticView() != pinned_static){
    return Reject(
      rejection_reason,
      "legacy cost view is bound to a different static view");
  }
  if(snapshot->observationRevision() != pinned_cost->observationRevision()){
    return Reject(
      rejection_reason,
      "legacy cost observation revision does not match the scene");
  }

  if(!pinned_static->groundPoints() || !pinned_static->mapPoints()){
    return Reject(rejection_reason, "legacy static point clouds are absent");
  }
  if(pinned_static->groundPoints()->points.size() != source.ground_point_count ||
    pinned_static->mapPoints()->points.size() != source.map_point_count)
  {
    return Reject(
      rejection_reason,
      "legacy static point counts do not match the source-map identity");
  }
  if(!pinned_static->groundKdtree() || !pinned_static->mapKdtree()){
    return Reject(rejection_reason, "legacy static kd-trees are absent");
  }

  const std::size_t ground_count = source.ground_point_count;
  if(pinned_cost->minDGraph().size() != ground_count){
    return Reject(
      rejection_reason,
      "legacy cost min_dgraph size does not match the ground index count");
  }
  for(const double value : pinned_cost->minDGraph()){
    if(!std::isfinite(value)){
      return Reject(
        rejection_reason, "legacy cost min_dgraph contains a non-finite value");
    }
  }
  if(!pinned_cost->lethalPoints()){
    return Reject(rejection_reason, "legacy cost lethal point cloud is absent");
  }

  if(pinned_static->graph().nodeCount() != ground_count ||
    pinned_static->graph().nodeWeightCount() != ground_count)
  {
    return Reject(
      rejection_reason,
      "legacy static graph size does not match the ground index count");
  }

  if(static_view != nullptr){
    *static_view = pinned_static;
  }
  if(cost_view != nullptr){
    *cost_view = pinned_cost;
  }
  if(rejection_reason != nullptr){
    rejection_reason->clear();
  }
  return true;
}

std::shared_ptr<const nanoflann::KdTreeFLANN<pcl::PointXYZI>>
BuildLegacyLethalKdTree(
  const std::shared_ptr<const perception_3d::LegacyCostView> & cost_view)
{
  if(!cost_view || !cost_view->lethalPoints() ||
    cost_view->lethalPoints()->empty())
  {
    return nullptr;
  }
  auto tree = std::make_shared<nanoflann::KdTreeFLANN<pcl::PointXYZI>>();
  tree->setInputCloud(cost_view->lethalPoints());
  return tree;
}

}  // namespace global_planner
