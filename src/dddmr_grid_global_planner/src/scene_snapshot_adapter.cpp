// Copyright 2026 DDDMobileRobot
//
// Grid-only SceneSnapshot admission and static source identity binding.

#include "grid_global_planner/scene_snapshot_adapter.h"

#include <perception_3d/ground_surface_filter.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <exception>
#include <utility>

namespace grid_global_planner
{
namespace
{

bool ConvertCloud(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & frame_id,
  pcl::PointCloud<pcl::PointXYZI> * cloud,
  std::string * detail)
{
  if (cloud == nullptr) {
    if (detail != nullptr) {
      *detail = "point cloud output is null";
    }
    return false;
  }
  if (message.header.frame_id != frame_id) {
    if (detail != nullptr) {
      *detail = "static source frame '" + message.header.frame_id +
        "' does not match '" + frame_id + "'";
    }
    return false;
  }
  try {
    pcl::fromROSMsg(message, *cloud);
  } catch (const std::exception & exception) {
    if (detail != nullptr) {
      *detail = std::string("point cloud conversion failed: ") +
        exception.what();
    }
    return false;
  } catch (...) {
    if (detail != nullptr) {
      *detail = "point cloud conversion failed";
    }
    return false;
  }
  if (cloud->points.empty()) {
    if (detail != nullptr) {
      *detail = "static source cloud is empty";
    }
    return false;
  }
  return true;
}

SceneAdmission Rejected(
  SceneAdmissionStatus status, std::string detail)
{
  SceneAdmission admission;
  admission.status = status;
  admission.detail = std::move(detail);
  return admission;
}

std::string SourceMismatchDetail(
  const perception_3d::SourceMapIdentity & expected,
  const perception_3d::SourceMapIdentity & actual)
{
  return "producer source map identity does not match the Grid terrain input: "
         "expected frame='" + expected.frame_id +
         "' ground_count=" + std::to_string(expected.ground_point_count) +
         " map_count=" + std::to_string(expected.map_point_count) +
         " content_hash=" + std::to_string(expected.content_hash) +
         "; scene frame='" + actual.frame_id +
         "' ground_count=" + std::to_string(actual.ground_point_count) +
         " map_count=" + std::to_string(actual.map_point_count) +
         " content_hash=" + std::to_string(actual.content_hash);
}

}  // namespace

const char * SceneAdmissionStatusToString(
  SceneAdmissionStatus status) noexcept
{
  switch (status) {
    case SceneAdmissionStatus::kAccepted:
      return "accepted";
    case SceneAdmissionStatus::kRootMissing:
      return "root_missing";
    case SceneAdmissionStatus::kExpectedSourceUnavailable:
      return "expected_source_unavailable";
    case SceneAdmissionStatus::kSourceIdentityUnknown:
      return "source_identity_unknown";
    case SceneAdmissionStatus::kIncomplete:
      return "incomplete";
    case SceneAdmissionStatus::kSensorsNotCurrent:
      return "sensors_not_current";
    case SceneAdmissionStatus::kWrongFrame:
      return "wrong_frame";
    case SceneAdmissionStatus::kStale:
      return "stale";
    case SceneAdmissionStatus::kEvidenceUnavailable:
      return "evidence_unavailable";
    case SceneAdmissionStatus::kSourceChanged:
      return "source_changed";
  }
  return "unknown";
}

bool ComputeStaticSourceInputIdentity(
  const sensor_msgs::msg::PointCloud2 & ground,
  const sensor_msgs::msg::PointCloud2 & map,
  const std::string & frame_id,
  const StaticSourceInputOptions & options,
  perception_3d::SourceMapIdentity * identity,
  std::string * detail)
{
  if (identity == nullptr) {
    if (detail != nullptr) {
      *detail = "identity output is null";
    }
    return false;
  }
  *identity = perception_3d::SourceMapIdentity();
  if (frame_id.empty()) {
    if (detail != nullptr) {
      *detail = "static source frame is empty";
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI> ground_cloud;
  pcl::PointCloud<pcl::PointXYZI> map_cloud;
  std::string conversion_detail;
  if (!ConvertCloud(ground, frame_id, &ground_cloud, &conversion_detail)) {
    if (detail != nullptr) {
      *detail = "mapground: " + conversion_detail;
    }
    return false;
  }
  if (!ConvertCloud(map, frame_id, &map_cloud, &conversion_detail)) {
    if (detail != nullptr) {
      *detail = "mapcloud: " + conversion_detail;
    }
    return false;
  }

  // The producer's StaticLayer applies FilterGroundSurface before freezing its
  // source identity. Reuse that exact exported algorithm with the producer's
  // actual runtime merge parameters, so Grid's independently derived identity
  // covers the same points/order/intensity as the producer fingerprint.
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ground;
  if (options.merge_ground_layers) {
    perception_3d::GroundSurfaceFilterConfig filter;
    filter.merge_ground_layers = true;
    filter.merge_ground_xy_resolution = options.merge_ground_xy_resolution;
    filter.merge_ground_z_tolerance = options.merge_ground_z_tolerance;
    filtered_ground = perception_3d::FilterGroundSurface(
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(ground_cloud)), filter);
    if (!filtered_ground || filtered_ground->points.empty()) {
      if (detail != nullptr) {
        *detail = "producer ground merge produced no points";
      }
      return false;
    }
  } else {
    filtered_ground.reset(new pcl::PointCloud<pcl::PointXYZI>(ground_cloud));
  }

  identity->frame_id = frame_id;
  identity->content_hash =
    perception_3d::ComputeStaticContentHash(
    *filtered_ground, map_cloud, frame_id);
  identity->ground_point_count = filtered_ground->points.size();
  identity->map_point_count = map_cloud.points.size();
  identity->revision = 0;
  identity->known = true;
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

bool SameStaticInputContent(
  const perception_3d::SourceMapIdentity & expected_input,
  const perception_3d::SourceMapIdentity & producer_scene)
{
  // revision is deliberately excluded. The producer revision is local to the
  // producer's accepted static events and is not a Grid terrain identity.
  return expected_input.known && producer_scene.known &&
         expected_input.frame_id == producer_scene.frame_id &&
         expected_input.content_hash == producer_scene.content_hash &&
         expected_input.ground_point_count ==
         producer_scene.ground_point_count &&
         expected_input.map_point_count ==
         producer_scene.map_point_count;
}

void SceneSnapshotGate::SetExpectedInputIdentity(
  const perception_3d::SourceMapIdentity & identity)
{
  expected_input_ = identity;
  expected_input_known_ = identity.known;
  if (state_ == State::kAwaitingExpected && expected_input_known_) {
    state_ = State::kAwaitingScene;
  }
  if (state_ == State::kBound && expected_input_known_ &&
    !SameStaticInputContent(expected_input_, bound_source_map_))
  {
    state_ = State::kInvalid;
    invalid_detail_ =
      "Grid expected static source identity changed after binding";
  }
}

bool SceneSnapshotGate::CheckBoundSource(
  const perception_3d::SceneSnapshot * latest,
  std::string * detail)
{
  if (state_ == State::kInvalid) {
    if (detail != nullptr) {
      *detail = invalid_detail_;
    }
    return false;
  }
  if (state_ != State::kBound) {
    if (detail != nullptr) {
      *detail = "initial source binding has not been verified from the Grid "
        "terrain input";
    }
    return false;
  }
  if (latest == nullptr) {
    if (detail != nullptr) {
      *detail = "latest root SceneSnapshot is missing";
    }
    return false;
  }
  const perception_3d::SourceMapIdentity & source_map = latest->sourceMap();
  if (!source_map.known) {
    // A transient missing producer identity blocks new live planning but is not
    // a permanent source change. It must never be converted into clear data.
    if (detail != nullptr) {
      *detail = "latest root scene source identity is unknown";
    }
    return false;
  }
  if (!source_map.SameContent(bound_source_map_)) {
    state_ = State::kInvalid;
    invalid_detail_ = SourceMismatchDetail(bound_source_map_, source_map) +
      "; old Grid terrain is no longer the current source. A rebuild/restart "
      "is required; no silent rebind or hot map switch is implemented";
    if (detail != nullptr) {
      *detail = invalid_detail_;
    }
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

SceneAdmission SceneSnapshotGate::Evaluate(
  const std::shared_ptr<const perception_3d::SceneSnapshot> & scene,
  const std::string & expected_frame,
  std::int64_t now_steady_ns,
  std::int64_t max_receipt_age_ns)
{
  if (!scene) {
    return Rejected(
      SceneAdmissionStatus::kRootMissing,
      "latest root SceneSnapshot is missing");
  }
  if (!expected_input_known_) {
    // The Grid terrain input identity is unavailable. Never bind an arbitrary
    // first scene to terrain whose source provenance cannot be verified.
    return Rejected(
      SceneAdmissionStatus::kExpectedSourceUnavailable,
      "Grid static source identity was not derived from the terrain input");
  }
  if (state_ == State::kInvalid) {
    return Rejected(SceneAdmissionStatus::kSourceChanged, invalid_detail_);
  }
  const perception_3d::SourceMapIdentity & source_map = scene->sourceMap();
  if (!source_map.known) {
    return Rejected(
      SceneAdmissionStatus::kSourceIdentityUnknown,
      "root scene has no known producer source map identity");
  }
  // Identity/frame changes after a verified bind are a permanent source
  // change. Latch them BEFORE completeness/sensor/freshness so source B with
  // stale, incomplete or non-current evidence cannot leave old terrain
  // plannable or committable.
  if (state_ == State::kBound && !source_map.SameContent(bound_source_map_)) {
    state_ = State::kInvalid;
    invalid_detail_ = SourceMismatchDetail(bound_source_map_, source_map) +
      "; old Grid terrain is no longer the current source. A rebuild/restart "
      "is required; no silent rebind or hot map switch is implemented";
    return Rejected(SceneAdmissionStatus::kSourceChanged, invalid_detail_);
  }
  if (scene->frame_id() != expected_frame ||
    source_map.frame_id != expected_frame)
  {
    return Rejected(
      SceneAdmissionStatus::kWrongFrame,
      "root scene frame '" + scene->frame_id() + "' / source frame '" +
      source_map.frame_id + "' is not the configured map frame '" +
      expected_frame + "'");
  }
  if (!scene->complete()) {
    return Rejected(
      SceneAdmissionStatus::kIncomplete,
      "root scene is not complete");
  }
  if (!scene->sensorsCurrent()) {
    return Rejected(
      SceneAdmissionStatus::kSensorsNotCurrent,
      "root scene sensors are not current");
  }
  if (!scene->hasEvidence() || !scene->evidence()) {
    return Rejected(
      SceneAdmissionStatus::kEvidenceUnavailable,
      "root scene has no frozen evidence view");
  }
  std::string freshness_detail;
  if (!scene->IsFresh(now_steady_ns, max_receipt_age_ns, &freshness_detail)) {
    return Rejected(
      SceneAdmissionStatus::kStale,
      freshness_detail.empty() ? "root scene receipt is stale" :
      freshness_detail);
  }

  if (state_ != State::kBound) {
    if (!SameStaticInputContent(expected_input_, source_map)) {
      // Still waiting for the producer to accept the exact pair Grid built
      // from. Reject evidence but do not latch invalid: the producer may still
      // be exposing its previous static pair at startup.
      return Rejected(
        SceneAdmissionStatus::kSourceChanged,
        SourceMismatchDetail(expected_input_, source_map) +
        "; evidence is withheld until the producer source matches the Grid "
        "terrain input");
    }
    bound_source_map_ = source_map;
    state_ = State::kBound;
  }

  SceneAdmission admission;
  admission.status = SceneAdmissionStatus::kAccepted;
  admission.evidence = scene->evidence();
  admission.detail.clear();
  return admission;
}

}  // namespace grid_global_planner
