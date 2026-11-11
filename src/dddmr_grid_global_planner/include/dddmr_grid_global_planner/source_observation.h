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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__SOURCE_OBSERVATION_H_
#define DDDMR_GRID_GLOBAL_PLANNER__SOURCE_OBSERVATION_H_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Which obstacle layer a source contributes to. Static occupancy may only be
// changed by a static source; a dynamic clear never clears static occupancy.
enum class SourceLayer : std::uint8_t {kStatic = 0, kDynamic = 1};

// Update semantic of one frozen source observation.
enum class SourceUpdateKind : std::uint8_t
{
  // Replaces this source_id's contribution exactly. Old records not present in
  // the new snapshot are removed; new records are the complete contribution.
  kCompleteReplacement = 0,
  // Adds the listed occupied records and removes/clears only the listed cleared
  // records. Other records for the same source survive.
  kExactDelta = 1,
};

// Explicit observed vertical evidence for one source volume. `known` says the
// volume was observed; `free_space` says the whole box is observed FREE. A
// known volume with free_space == false is only an occupied extent and is never
// rasterized as free space. `free_space` defaults false: a producer must set it
// explicitly true to claim the whole box was observed free. A missing volume
// leaves headroom unknown.
struct ObservedVolume
{
  Point3 min{};
  Point3 max{};
  bool known{false};
  bool free_space{false};
  double headroom_m{std::numeric_limits<double>::quiet_NaN()};
  bool headroom_known{false};
};

// One frozen occupied/cleared point record. The point is in the map frame; the
// vertical extent is the observed obstacle slab at that XY location.
struct SourceOccupancyRecord
{
  Point3 point{};
  double min_z{0.0};
  double max_z{0.0};
};

// Immutable frozen source observation. Construction deep-copies or moves the
// supplied vectors, and there are no mutable accessors: once published, a
// producer cannot rewrite the record list. The worker owns association to
// terrain nodes; this object only carries frozen input evidence.
class SourceObservationSnapshot
{
public:
  // Freezes all supplied fields by move/copy. No validation happens here; call
  // IsUsable() before submitting the observation to an update worker.
  SourceObservationSnapshot(
    std::string source_id, std::string frame_id,
    std::uint64_t source_sequence, std::uint64_t terrain_revision,
    std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
    SourceLayer layer, SourceUpdateKind kind, bool complete,
    std::vector<SourceOccupancyRecord> occupied,
    std::vector<SourceOccupancyRecord> cleared,
    std::vector<ObservedVolume> known_volumes, ObservedVolume headroom);

  // Returns the producer-assigned stable source identifier.
  const std::string & source_id() const noexcept {return source_id_;}
  // Returns the frame in which every point and volume bound is expressed.
  const std::string & frame_id() const noexcept {return frame_id_;}
  // Returns the per-source monotonic sequence used for stale rejection.
  std::uint64_t source_sequence() const noexcept {return source_sequence_;}
  // Returns the terrain revision this observation was produced against, or 0
  // when the producer did not know it.
  std::uint64_t terrain_revision() const noexcept {return terrain_revision_;}
  // Returns the source sensor timestamp in its own clock domain.
  std::int64_t sensor_stamp_ns() const noexcept {return sensor_stamp_ns_;}
  // Returns the monotonic receipt timestamp used for TTL checks.
  std::int64_t received_steady_ns() const noexcept
  {
    return received_steady_ns_;
  }
  // Returns the static/dynamic layer this source contributes to.
  SourceLayer layer() const noexcept {return layer_;}
  // Returns whether this observation replaces or deltas the source.
  SourceUpdateKind kind() const noexcept {return kind_;}
  // Returns whether the producer marked the observation complete.
  bool complete() const noexcept {return complete_;}
  // Returns the frozen occupied records.
  const std::vector<SourceOccupancyRecord> & occupied() const noexcept
  {
    return occupied_;
  }
  // Returns the frozen explicitly cleared records.
  const std::vector<SourceOccupancyRecord> & cleared() const noexcept
  {
    return cleared_;
  }
  // Returns the explicit observed volumes; only known=true volumes are evidence.
  const std::vector<ObservedVolume> & known_volumes() const noexcept
  {
    return known_volumes_;
  }
  // Returns the optional whole-observation headroom/clearance evidence.
  const ObservedVolume & headroom() const noexcept {return headroom_;}

  // Rejects an incomplete, structurally invalid, or obviously future/invalid
  // stamp observation. An explicitly observed empty complete replacement is
  // usable only when it also carries explicit cleared records or a valid known
  // volume; an empty no-evidence observation is never interpreted as clear. An
  // exact delta likewise must carry explicit cleared records or a valid known
  // volume. Freshness against a monotonic clock and TTL are checked by the
  // update worker, which owns the now_steady value.
  bool IsUsable(std::string * detail) const;

private:
  std::string source_id_;
  std::string frame_id_;
  std::uint64_t source_sequence_{0};
  std::uint64_t terrain_revision_{0};
  std::int64_t sensor_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  SourceLayer layer_{SourceLayer::kStatic};
  SourceUpdateKind kind_{SourceUpdateKind::kCompleteReplacement};
  bool complete_{false};
  std::vector<SourceOccupancyRecord> occupied_;
  std::vector<SourceOccupancyRecord> cleared_;
  std::vector<ObservedVolume> known_volumes_;
  ObservedVolume headroom_{};
};

// Returns a stable diagnostic spelling for a source layer.
const char * SourceLayerToString(SourceLayer layer) noexcept;

// Returns a stable diagnostic spelling for a source update kind.
const char * SourceUpdateKindToString(SourceUpdateKind kind) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__SOURCE_OBSERVATION_H_
