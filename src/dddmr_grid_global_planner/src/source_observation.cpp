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

#include "dddmr_grid_global_planner/source_observation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace dddmr_grid_global_planner
{
namespace
{

// Returns true when one record has finite geometry and a non-inverted Z slab.
bool RecordIsStructurallyValid(const SourceOccupancyRecord & record) noexcept
{
  if (!record.point.IsFinite() || !std::isfinite(record.min_z) ||
    !std::isfinite(record.max_z))
  {
    return false;
  }
  return record.min_z <= record.max_z;
}

// Returns true when an explicit headroom value is finite and non-negative.
bool HeadroomIsStructurallyValid(const ObservedVolume & volume) noexcept
{
  if (!volume.headroom_known) {
    return true;
  }
  return std::isfinite(volume.headroom_m) && volume.headroom_m >= 0.0;
}

// Returns true when a volume that claims to be known has finite, ordered bounds
// and, when supplied, a finite non-negative headroom.
bool VolumeIsStructurallyValid(const ObservedVolume & volume) noexcept
{
  if (!volume.known) {
    return true;
  }
  if (!volume.min.IsFinite() || !volume.max.IsFinite()) {
    return false;
  }
  if (volume.min.x > volume.max.x || volume.min.y > volume.max.y ||
    volume.min.z > volume.max.z)
  {
    return false;
  }
  return HeadroomIsStructurallyValid(volume);
}

}  // namespace

SourceObservationSnapshot::SourceObservationSnapshot(
  std::string source_id, std::string frame_id,
  std::uint64_t source_sequence, std::uint64_t terrain_revision,
  std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
  SourceLayer layer, SourceUpdateKind kind, bool complete,
  std::vector<SourceOccupancyRecord> occupied,
  std::vector<SourceOccupancyRecord> cleared,
  std::vector<ObservedVolume> known_volumes, ObservedVolume headroom)
: source_id_(std::move(source_id)),
  frame_id_(std::move(frame_id)),
  source_sequence_(source_sequence),
  terrain_revision_(terrain_revision),
  sensor_stamp_ns_(sensor_stamp_ns),
  received_steady_ns_(received_steady_ns),
  layer_(layer),
  kind_(kind),
  complete_(complete),
  occupied_(std::move(occupied)),
  cleared_(std::move(cleared)),
  known_volumes_(std::move(known_volumes)),
  headroom_(headroom) {}

bool SourceObservationSnapshot::IsUsable(std::string * detail) const
{
  const auto reject = [detail](const char * message) {
      if (detail != nullptr) {
        *detail = message;
      }
      return false;
    };
  if (source_id_.empty()) {
    return reject("source_id is empty");
  }
  if (frame_id_.empty()) {
    return reject("frame_id is empty");
  }
  if (!complete_) {
    return reject("source observation is incomplete");
  }
  // A receipt stamp is required for TTL/freshness. A non-positive value is
  // missing or in the future relative to the monotonic clock domain.
  if (received_steady_ns_ <= 0) {
    return reject("received_steady_ns is missing or non-positive");
  }
  if (sensor_stamp_ns_ < 0) {
    return reject("sensor_stamp_ns is negative");
  }
  // Explicit clearing evidence is either an explicit cleared record or a known
  // volume that declares free_space. A known occupied extent is not removal
  // evidence, and a bare empty observation must never remove occupancy.
  bool has_free_volume = false;
  for (const ObservedVolume & volume : known_volumes_) {
    if (!VolumeIsStructurallyValid(volume)) {
      return reject("known volume has invalid bounds");
    }
    if (volume.known && volume.free_space) {
      has_free_volume = true;
    }
  }
  const bool has_clear_evidence = !cleared_.empty() || has_free_volume;
  if (occupied_.empty()) {
    if (kind_ == SourceUpdateKind::kExactDelta) {
      if (!has_clear_evidence) {
        return reject(
          "exact delta has no occupied, cleared, or known-volume evidence");
      }
    } else if (!has_clear_evidence) {
      return reject("source observation is empty and has no clear evidence");
    }
  }
  for (const SourceOccupancyRecord & record : occupied_) {
    if (!RecordIsStructurallyValid(record)) {
      return reject("occupied record has invalid geometry");
    }
  }
  for (const SourceOccupancyRecord & record : cleared_) {
    if (!RecordIsStructurallyValid(record)) {
      return reject("cleared record has invalid geometry");
    }
  }
  if (!HeadroomIsStructurallyValid(headroom_)) {
    return reject("headroom is invalid");
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

const char * SourceLayerToString(SourceLayer layer) noexcept
{
  switch (layer) {
    case SourceLayer::kStatic:
      return "static";
    case SourceLayer::kDynamic:
      return "dynamic";
  }
  return "unknown";
}

const char * SourceUpdateKindToString(SourceUpdateKind kind) noexcept
{
  switch (kind) {
    case SourceUpdateKind::kCompleteReplacement:
      return "complete_replacement";
    case SourceUpdateKind::kExactDelta:
      return "exact_delta";
  }
  return "unknown";
}

}  // namespace dddmr_grid_global_planner
