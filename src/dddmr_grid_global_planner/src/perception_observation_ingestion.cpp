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

#include "perception_observation_ingestion.h"  // NOLINT(build/include_subdir): private src/ header

#include <utility>

namespace grid_global_planner
{
namespace
{

// Converts a frozen perception point into a planner point.
Point3 ToPoint3(const perception_3d::PerceptionPoint & point)
{
  return Point3{point.x, point.y, point.z};
}

// Maps one explicit perception volume to the planner volume type. Bounds are
// copied exactly; the adapter never widens an observed volume into an AABB.
ObservedVolume ToObservedVolume(
  const perception_3d::PerceptionObservedVolume & volume)
{
  ObservedVolume converted;
  converted.min = ToPoint3(volume.min);
  converted.max = ToPoint3(volume.max);
  converted.known = volume.known;
  // Copy the producer's explicit declaration. Never infer free space from
  // `known`: a known occupied extent (free_space == false) must stay occupied.
  converted.free_space = volume.free_space;
  converted.headroom_m = volume.headroom_m;
  converted.headroom_known = volume.headroom_known;
  return converted;
}

// Maps the perception export's layer/update enums onto the planner's enums.
// Both sides use the same explicit values, but an explicit switch keeps the
// adapter independent of any future renumbering.
SourceLayer ToSourceLayer(perception_3d::PerceptionSourceLayer layer)
{
  switch (layer) {
    case perception_3d::PerceptionSourceLayer::kStatic:
      return SourceLayer::kStatic;
    case perception_3d::PerceptionSourceLayer::kDynamic:
      return SourceLayer::kDynamic;
  }
  return SourceLayer::kStatic;
}

SourceUpdateKind ToSourceUpdateKind(
  perception_3d::PerceptionUpdateKind kind)
{
  switch (kind) {
    case perception_3d::PerceptionUpdateKind::kCompleteReplacement:
      return SourceUpdateKind::kCompleteReplacement;
    case perception_3d::PerceptionUpdateKind::kExactDelta:
      return SourceUpdateKind::kExactDelta;
  }
  return SourceUpdateKind::kCompleteReplacement;
}

// Converts one point-only occupied record. The point cloud carries no explicit
// vertical slab, so min_z == max_z == point.z and free volume stays unknown.
SourceOccupancyRecord ToPointRecord(
  const perception_3d::PerceptionPoint & point)
{
  SourceOccupancyRecord record;
  record.point = ToPoint3(point);
  record.min_z = point.z;
  record.max_z = point.z;
  return record;
}

}  // namespace

PerceptionIngestionResult ConvertPerceptionSnapshotToSourceObservations(
  const perception_3d::PerceptionSnapshot & snapshot,
  const std::string & expected_frame, std::int64_t received_steady_ns,
  std::uint64_t terrain_revision, bool terrain_revision_known,
  const PerceptionIngestionLimits & limits)
{
  PerceptionIngestionResult result;
  if (expected_frame.empty()) {
    result.detail = "expected map frame is empty";
    return result;
  }
  if (!snapshot.IsUsable(&result.detail)) {
    return result;
  }
  if (snapshot.frame_id() != expected_frame) {
    result.detail = "perception snapshot frame '" + snapshot.frame_id() +
      "' is not the configured map frame '" + expected_frame +
      "'";
    return result;
  }
  if (snapshot.sources().size() > limits.max_sources) {
    result.detail = "perception export exceeds max_sources";
    return result;
  }

  for (const perception_3d::PerceptionSourceObservation & source :
    snapshot.sources())
  {
    if (source.frame_id() != expected_frame) {
      result.detail = "perception source '" + source.source_id() +
        "' is not expressed in the configured map frame";
      return result;
    }
    if (source.occupied().size() + source.cleared().size() +
      source.known_volumes().size() >
      limits.max_records)
    {
      result.detail = "perception source '" + source.source_id() +
        "' exceeds max_records";
      return result;
    }

    std::vector<SourceOccupancyRecord> occupied;
    occupied.reserve(source.occupied().size());
    for (const perception_3d::PerceptionPoint & point : source.occupied()) {
      occupied.push_back(ToPointRecord(point));
    }
    std::vector<SourceOccupancyRecord> cleared;
    cleared.reserve(source.cleared().size());
    for (const perception_3d::PerceptionPoint & point : source.cleared()) {
      cleared.push_back(ToPointRecord(point));
    }
    std::vector<ObservedVolume> known_volumes;
    known_volumes.reserve(source.known_volumes().size());
    for (const perception_3d::PerceptionObservedVolume & volume :
      source.known_volumes())
    {
      known_volumes.push_back(ToObservedVolume(volume));
    }

    // Preserve per-source provenance even when a newer export cycle contains
    // the same scan. Polling must not extend the source's lifetime.
    const std::int64_t receipt =
      source.received_steady_ns() > 0 ? source.received_steady_ns() :
      (received_steady_ns > 0 ? received_steady_ns : snapshot.received_steady_ns());
    SourceObservationSnapshot observation(
      source.source_id(), source.frame_id(), source.source_sequence(),
      terrain_revision, source.sensor_stamp_ns(), receipt,
      ToSourceLayer(source.layer()), ToSourceUpdateKind(source.kind()),
      source.complete(), std::move(occupied), std::move(cleared),
      std::move(known_volumes), ToObservedVolume(source.headroom()));

    std::string detail;
    if (!observation.IsUsable(&detail)) {
      result.detail = "perception source '" + source.source_id() +
        "' conversion is unusable: " + detail;
      return result;
    }
    result.observations.push_back(std::move(observation));
  }

  result.ok = true;
  return result;
}

}  // namespace grid_global_planner
