// Copyright 2026 DDDMobileRobot

#include <global_planner/scene_pipeline.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace global_planner
{
namespace
{

constexpr double kMinimumValidationStep = 0.05;

bool IsFinite(double value)
{
  return std::isfinite(value);
}

}  // namespace

bool ScenePipelineContext::Bind(
  const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot,
  const LegacySearchParameters & parameters,
  std::string * rejection_reason)
{
  Reset();

  std::shared_ptr<const perception_3d::LegacyStaticView> static_view;
  std::shared_ptr<const perception_3d::LegacyCostView> cost_view;
  std::string reason;
  if(!BuildLegacySceneReadView(snapshot, parameters, &static_view, &cost_view, &reason)){
    rejection_reason_ = reason;
    if(rejection_reason != nullptr){
      *rejection_reason = reason;
    }
    return false;
  }

  if(!IsFinite(parameters.inscribed_radius) ||
    !IsFinite(parameters.inflation_descending_rate) ||
    !IsFinite(parameters.max_obstacle_distance))
  {
    rejection_reason_ = "search parameters contain a non-finite value";
    if(rejection_reason != nullptr){
      *rejection_reason = rejection_reason_;
    }
    return false;
  }

  snapshot_ = snapshot;
  static_view_ = std::move(static_view);
  cost_view_ = std::move(cost_view);
  parameters_ = parameters;
  valid_ = true;
  rejection_reason_.clear();
  if(rejection_reason != nullptr){
    rejection_reason->clear();
  }
  return true;
}

void ScenePipelineContext::Reset()
{
  valid_ = false;
  snapshot_.reset();
  static_view_.reset();
  cost_view_.reset();
  parameters_ = LegacySearchParameters{};
  rejection_reason_.clear();
}

bool ScenePipelineContext::SameStaticMap(
  const ScenePipelineContext & other) const noexcept
{
  if(!valid_ || !other.valid_ || !snapshot_ || !other.snapshot_ ||
    !static_view_ || !other.static_view_)
  {
    return false;
  }
  //@ The immutable content identity, not the wrapper pointer, defines whether
  //@ the original ground/map index binding is still valid. A capture may hand
  //@ back a new immutable view object for unchanged content.
  if(!snapshot_->sourceMap().SameContent(other.snapshot_->sourceMap())){
    return false;
  }
  return static_view_->identity().SameContent(other.static_view_->identity());
}

bool ScenePipelineContext::SameObservation(
  const ScenePipelineContext & other) const noexcept
{
  if(!SameStaticMap(other) || !snapshot_ || !other.snapshot_){
    return false;
  }
  const std::uint64_t token = snapshot_->observationRevision();
  return token != 0 && token == other.snapshot_->observationRevision();
}

bool ScenePipelineContext::SamePinnedViews(
  const ScenePipelineContext & other) const noexcept
{
  return valid_ && other.valid_ &&
         static_view_.get() == other.static_view_.get() &&
         cost_view_.get() == other.cost_view_.get();
}

bool ScenePipelineContext::ProjectOnGround(
  const pcl::PointXYZI & query,
  double search_radius,
  std::vector<int> * indices,
  std::vector<float> * squared_distances) const
{
  if(!valid_ || !static_view_ || !static_view_->groundKdtree() ||
    indices == nullptr || squared_distances == nullptr ||
    !IsFinite(search_radius) || search_radius <= 0.0)
  {
    if(indices != nullptr){
      indices->clear();
    }
    if(squared_distances != nullptr){
      squared_distances->clear();
    }
    return false;
  }
  indices->clear();
  squared_distances->clear();
  return static_view_->groundKdtree()->radiusSearch(
    query, search_radius, *indices, *squared_distances) > 0;
}

double ScenePipelineContext::FusedClearance(std::size_t ground_index) const noexcept
{
  if(!valid_ || !cost_view_){
    return 0.0;
  }
  return cost_view_->minDGraphValue(ground_index);
}

bool ScenePipelineContext::HasFusedClearance(std::size_t ground_index) const noexcept
{
  if(!valid_ || !cost_view_ || !static_view_ ||
    ground_index >= static_view_->groundPointCount())
  {
    return false;
  }
  const double clearance = cost_view_->minDGraphValue(ground_index);
  return IsFinite(clearance) &&
         IsFinite(parameters_.inscribed_radius) &&
         clearance >= parameters_.inscribed_radius;
}

SceneRelation RelatePinnedToLatest(
  const ScenePipelineContext & pinned,
  const ScenePipelineContext & latest)
{
  SceneRelation relation;
  relation.latest_available = latest.valid();
  if(!pinned.valid() || !latest.valid() || !latest.snapshot()){
    relation.detail = latest.valid() ?
      "pinned scene is not valid" : latest.rejectionReason();
    return relation;
  }

  relation.sensors_current = latest.snapshot()->sensorsCurrent();
  relation.static_map_same = latest.SameStaticMap(pinned);
  relation.observation_same = latest.SameObservation(pinned);
  if(!relation.static_map_same){
    relation.detail = "latest source-map identity differs from the pinned scene";
  }else if(relation.observation_same){
    relation.detail = "latest scene reuses the pinned static and observation tokens";
  }else{
    relation.detail = "latest scene has a changed dynamic observation token";
  }
  if(!relation.sensors_current){
    relation.detail += "; root sensor status is stale (reported only)";
  }
  return relation;
}

std::vector<pcl::PointXYZI> SamplePathForValidation(
  const nav_msgs::msg::Path & path,
  double sample_step)
{
  std::vector<pcl::PointXYZI> samples;
  if(path.poses.empty()){
    return samples;
  }

  const double step = (IsFinite(sample_step) && sample_step > 0.0) ?
    sample_step : kMinimumValidationStep;

  const auto append_point = [&samples](double x, double y, double z) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(z);
    point.intensity = 0.0F;
    samples.push_back(point);
  };

  const auto & first = path.poses.front().pose.position;
  if(!IsFinite(first.x) || !IsFinite(first.y) || !IsFinite(first.z)){
    return samples;
  }
  append_point(first.x, first.y, first.z);

  for(std::size_t i = 1; i < path.poses.size(); ++i){
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & current = path.poses[i].pose.position;
    if(!IsFinite(current.x) || !IsFinite(current.y) || !IsFinite(current.z)){
      samples.clear();
      return samples;
    }

    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double dz = current.z - previous.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if(!IsFinite(distance)){
      samples.clear();
      return samples;
    }

    if(distance <= 1e-9){
      append_point(current.x, current.y, current.z);
      continue;
    }

    const std::size_t intermediate_count =
      static_cast<std::size_t>(std::ceil(distance / step));
    for(std::size_t sample_index = 1; sample_index < intermediate_count; ++sample_index){
      const double ratio =
        static_cast<double>(sample_index) / static_cast<double>(intermediate_count);
      append_point(
        previous.x + dx * ratio,
        previous.y + dy * ratio,
        previous.z + dz * ratio);
    }
    append_point(current.x, current.y, current.z);
  }

  return samples;
}

ScenePathValidation ValidatePathAgainstLatest(
  const ScenePipelineContext & pinned,
  const ScenePipelineContext & latest,
  const nav_msgs::msg::Path & path,
  double sample_step)
{
  ScenePathValidation validation;
  if(!pinned.valid() || !latest.valid()){
    validation.result = ScenePathValidation::Result::kNotReady;
    validation.reason = !pinned.valid() ?
      "pinned scene is not valid" : latest.rejectionReason();
    return validation;
  }

  validation.static_map_same = latest.SameStaticMap(pinned);
  if(!validation.static_map_same){
    validation.result = ScenePathValidation::Result::kStaticMismatchReject;
    validation.reason =
      "latest source-map identity differs; route indices are bound to the old static view";
    return validation;
  }

  validation.observation_same = latest.SameObservation(pinned);
  if(validation.observation_same){
    validation.result = ScenePathValidation::Result::kAccepted;
    validation.reason = "same static identity and observation token; pinned route reused";
    return validation;
  }

  validation.dynamic_changed = true;
  const double inscribed_radius = latest.parameters().inscribed_radius;
  if(!IsFinite(inscribed_radius) || inscribed_radius <= 0.0){
    validation.result = ScenePathValidation::Result::kNotReady;
    validation.reason = "inscribed radius is not usable for latest-scene validation";
    return validation;
  }

  const std::vector<pcl::PointXYZI> samples =
    SamplePathForValidation(path, sample_step);
  if(samples.empty()){
    validation.result = ScenePathValidation::Result::kNotReady;
    validation.reason = "route has no usable samples for latest-scene validation";
    return validation;
  }

  const std::shared_ptr<const perception_3d::LegacyCostView> & cost_view =
    latest.costView();
  const std::shared_ptr<const perception_3d::LegacyStaticView> & static_view =
    latest.staticView();
  if(!cost_view || !static_view || !static_view->groundKdtree() ||
    !static_view->groundPoints())
  {
    validation.result = ScenePathValidation::Result::kNotReady;
    validation.reason = "latest pinned views are incomplete";
    return validation;
  }

  const auto lethal_tree = BuildLegacyLethalKdTree(cost_view);
  const double lethal_radius = 2.0 * inscribed_radius;

  for(std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index){
    const pcl::PointXYZI & sample = samples[sample_index];
    bool conflict = false;
    std::string conflict_reason;

    if(lethal_tree && lethal_tree->getInputCloud() &&
      !lethal_tree->getInputCloud()->empty())
    {
      std::vector<int> lethal_indices;
      std::vector<float> lethal_squared_distances;
      const int lethal_hits = lethal_tree->radiusSearch(
        sample, static_cast<float>(lethal_radius),
        lethal_indices, lethal_squared_distances);
      if(lethal_hits > 1){
        conflict = true;
        conflict_reason = "frozen lethal points intersect the route corridor";
      }
    }

    if(!conflict){
      std::vector<int> ground_indices;
      std::vector<float> ground_squared_distances;
      if(static_view->groundKdtree()->radiusSearch(
          sample, 1.0, ground_indices, ground_squared_distances) > 0)
      {
        std::size_t closest = 0;
        for(std::size_t i = 1;
          i < ground_indices.size() && i < ground_squared_distances.size(); ++i)
        {
          if(ground_squared_distances[i] < ground_squared_distances[closest]){
            closest = i;
          }
        }
        if(closest < ground_indices.size() && ground_indices[closest] >= 0){
          const std::size_t ground_index =
            static_cast<std::size_t>(ground_indices[closest]);
          const double clearance = cost_view->minDGraphValue(ground_index);
          if(!IsFinite(clearance) || clearance < inscribed_radius){
            conflict = true;
            conflict_reason = "latest fused cost is below the inscribed radius";
          }
        }
      }
    }

    if(conflict){
      validation.result = ScenePathValidation::Result::kDynamicConflictReplan;
      validation.path_affected = true;
      validation.conflict_sample_index = sample_index;
      validation.conflict_x = sample.x;
      validation.conflict_y = sample.y;
      validation.conflict_z = sample.z;
      std::ostringstream stream;
      stream << "dynamic observation changed and affects the route at sample "
             << sample_index << " (" << sample.x << ", " << sample.y << ", "
             << sample.z << "): " << conflict_reason;
      validation.reason = stream.str();
      return validation;
    }
  }

  validation.result = ScenePathValidation::Result::kAccepted;
  validation.path_affected = false;
  validation.reason =
    "dynamic observation changed but the entire route corridor remains clear";
  return validation;
}

void RefreshPathOrientations(nav_msgs::msg::Path * path)
{
  if(path == nullptr || path->poses.empty()){
    return;
  }
  if(path->poses.size() == 1){
    return;
  }

  for(std::size_t i = 0; i < path->poses.size(); ++i){
    const std::size_t prev_idx = (i > 0) ? i - 1 : i;
    const std::size_t next_idx = (i + 1 < path->poses.size()) ? i + 1 : i;
    if(prev_idx == next_idx){
      continue;
    }

    const double vx =
      path->poses[next_idx].pose.position.x - path->poses[prev_idx].pose.position.x;
    const double vy =
      path->poses[next_idx].pose.position.y - path->poses[prev_idx].pose.position.y;
    const double vz =
      path->poses[next_idx].pose.position.z - path->poses[prev_idx].pose.position.z;
    const double tangent_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if(tangent_norm < 1e-6){
      continue;
    }

    tf2::Quaternion q;
    if(std::fabs(vz) > 1e-6){
      tf2::Vector3 axis_vector(vx / tangent_norm, vy / tangent_norm, vz / tangent_norm);
      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      const double right_norm = right_vector.length();
      if(right_norm > 1e-6){
        right_vector /= right_norm;
        const double axis_dot =
          std::clamp(static_cast<double>(axis_vector.dot(up_vector)), -1.0, 1.0);
        q = tf2::Quaternion(right_vector, -1.0 * std::acos(axis_dot));
      }else{
        q.setRPY(0.0, 0.0, std::atan2(vy, vx));
      }
    }else{
      q.setRPY(0.0, 0.0, std::atan2(vy, vx));
    }
    q.normalize();
    path->poses[i].pose.orientation.x = q.getX();
    path->poses[i].pose.orientation.y = q.getY();
    path->poses[i].pose.orientation.z = q.getZ();
    path->poses[i].pose.orientation.w = q.getW();
  }
}

std::size_t SmoothPathAgainstScene(
  nav_msgs::msg::Path * path,
  const ScenePipelineContext & scene,
  bool enable,
  int iterations,
  double weight,
  double max_shift,
  double lock_start_distance,
  double lock_goal_distance,
  double ground_search_radius)
{
  if(path == nullptr || !scene.valid() || !scene.staticView()){
    return 0;
  }
  if(path->poses.size() < 5){
    return 0;
  }
  if(!enable || iterations <= 0 || weight <= 0.0 || max_shift <= 0.0){
    return 0;
  }

  const std::shared_ptr<const perception_3d::LegacyStaticView> & static_view =
    scene.staticView();
  const std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> & ground_kdtree =
    static_view->groundKdtree();
  const std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>> & map_kdtree =
    static_view->mapKdtree();
  if(!ground_kdtree || !map_kdtree){
    return 0;
  }

  std::vector<double> arc_lengths(path->poses.size(), 0.0);
  for(std::size_t i = 1; i < path->poses.size(); ++i){
    const double dx =
      path->poses[i].pose.position.x - path->poses[i - 1].pose.position.x;
    const double dy =
      path->poses[i].pose.position.y - path->poses[i - 1].pose.position.y;
    const double dz =
      path->poses[i].pose.position.z - path->poses[i - 1].pose.position.z;
    arc_lengths[i] = arc_lengths[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  const double total_length = arc_lengths.back();
  if(!std::isfinite(total_length) || total_length < 0.5){
    return 0;
  }

  std::vector<double> reference_x(path->poses.size(), 0.0);
  std::vector<double> reference_y(path->poses.size(), 0.0);
  for(std::size_t i = 0; i < path->poses.size(); ++i){
    reference_x[i] = path->poses[i].pose.position.x;
    reference_y[i] = path->poses[i].pose.position.y;
  }

  const double inscribed_radius =
    std::max(0.05, scene.parameters().inscribed_radius);

  std::size_t total_adjusted_points = 0;
  for(int iteration = 0; iteration < iterations; ++iteration){
    const auto previous = path->poses;
    bool iteration_changed = false;

    for(std::size_t i = 1; i + 1 < path->poses.size(); ++i){
      if(arc_lengths[i] <= lock_start_distance){
        continue;
      }
      if((total_length - arc_lengths[i]) <= lock_goal_distance){
        continue;
      }

      const double prev_x = previous[i - 1].pose.position.x;
      const double prev_y = previous[i - 1].pose.position.y;
      const double next_x = previous[i + 1].pose.position.x;
      const double next_y = previous[i + 1].pose.position.y;
      const double curr_x = previous[i].pose.position.x;
      const double curr_y = previous[i].pose.position.y;

      double target_x = curr_x + weight * (0.5 * (prev_x + next_x) - curr_x);
      double target_y = curr_y + weight * (0.5 * (prev_y + next_y) - curr_y);

      const double reference_dx = target_x - reference_x[i];
      const double reference_dy = target_y - reference_y[i];
      const double reference_shift = std::hypot(reference_dx, reference_dy);
      if(reference_shift > max_shift && reference_shift > 1e-6){
        const double scale = max_shift / reference_shift;
        target_x = reference_x[i] + reference_dx * scale;
        target_y = reference_y[i] + reference_dy * scale;
      }

      const double candidate_step = std::hypot(target_x - curr_x, target_y - curr_y);
      if(candidate_step < 1e-5){
        continue;
      }

      pcl::PointXYZI candidate;
      candidate.x = static_cast<float>(target_x);
      candidate.y = static_cast<float>(target_y);
      candidate.z = static_cast<float>(previous[i].pose.position.z);

      std::vector<int> ground_indices;
      std::vector<float> ground_squared_distances;
      if(ground_kdtree->radiusSearch(
          candidate, ground_search_radius, ground_indices,
          ground_squared_distances) < 1)
      {
        continue;
      }

      std::vector<int> map_indices;
      std::vector<float> map_squared_distances;
      if(map_kdtree->radiusSearch(
          candidate, inscribed_radius, map_indices, map_squared_distances) > 1)
      {
        continue;
      }

      path->poses[i].pose.position.x = target_x;
      path->poses[i].pose.position.y = target_y;
      iteration_changed = true;
      ++total_adjusted_points;
    }

    if(!iteration_changed){
      break;
    }
  }

  path->poses.front().pose.position.x = reference_x.front();
  path->poses.front().pose.position.y = reference_y.front();
  path->poses.back().pose.position.x = reference_x.back();
  path->poses.back().pose.position.y = reference_y.back();

  RefreshPathOrientations(path);
  return total_adjusted_points;
}

}  // namespace global_planner
