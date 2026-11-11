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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__INPUT_CONTRACT_H_
#define DDDMR_GRID_GLOBAL_PLANNER__INPUT_CONTRACT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Where a ground observation came from. Source identity is diagnostic evidence;
// it does not imply that the geometry is complete or traversable.
enum class ObservationSource : std::uint8_t
{
  kUnknown = 0,
  kStaticMap,
  kGroundSegmentation,
  kLidarGround,
  kDepthGround,
};

// Update semantics for one frozen ground observation.
//
// kReplace: the observation is the complete terrain input for a new immutable
//           revision. Stage 2 only supports this mode.
// kMerge:   reserved for future incremental/Tile updates; rejected by the
//           stage 2 builder instead of being silently treated as replacement.
// kClear:   removes observed terrain. It never means "assume free".
enum class ObservationUpdateSemantics : std::uint8_t
{
  kReplace = 0,
  kMerge,
  kClear,
};

// Why an observation was accepted or rejected.
enum class ObservationValidationStatus : std::uint8_t
{
  kValid = 0,
  kEmptyFrame,
  kFrameMismatch,
  kIncomplete,
  kEmptyObservation,
  kTooManyPoints,
  kNonFinitePoint,
  kFutureReceipt,
  kStaleReceipt,
  kInvalidOptions,
};

// Validation result with a human-readable diagnostic.
struct ObservationValidation
{
  bool ok{false};
  ObservationValidationStatus status{ObservationValidationStatus::kEmptyFrame};
  std::string detail;
};

// Inputs required to freeze one ground observation. Sensor timestamps are kept
// for ordering/TF correlation; freshness is checked with the monotonic receipt
// time because sensor and steady clocks must not be subtracted from each other.
struct ObservationMetadata
{
  std::string frame_id;
  ObservationSource source{ObservationSource::kUnknown};
  std::int64_t sensor_stamp_ns{0};
  std::int64_t received_steady_ns{0};
  std::uint64_t sequence{0};

  // True only when the producer guarantees that the point list is a complete,
  // frozen observation. Partial writes are rejected.
  bool complete{false};

  ObservationUpdateSemantics update{ObservationUpdateSemantics::kReplace};
};

// Validation policy. Defaults are conservative: a known map frame is required,
// the observation must be complete and non-empty, and receipt age must be at
// most one second.
struct ObservationValidationOptions
{
  std::string expected_frame_id{"map"};
  bool require_frame_match{true};
  std::int64_t max_receipt_age_ns{1'000'000'000};
  std::int64_t future_tolerance_ns{50'000'000};
  std::size_t max_points{20'000'000};
  bool allow_empty{false};
};

// A frozen ground-point observation in the frame named by metadata().frame_id.
// Construction copies both metadata and points, and there is intentionally no
// mutable accessor: an observation cannot be aliased and rewritten after it is
// published. Validation never inserts synthetic points or assumes an unobserved
// cell is free.
class TerrainPointObservation
{
public:
  // Constructs an empty, incomplete observation. This is intentionally not a
  // ready-to-plan input.
  TerrainPointObservation() = default;

  // Deep-copies/moves metadata and points into one frozen observation object.
  TerrainPointObservation(
    ObservationMetadata metadata,
    std::vector<Point3> points);

  // Const metadata accessor. There is no mutable overload.
  const ObservationMetadata & metadata() const noexcept {return metadata_;}

  // Const point accessor. There is no mutable overload.
  const std::vector<Point3> & points() const noexcept {return points_;}

  // Number of frozen input points.
  std::size_t point_count() const noexcept {return points_.size();}

  // True when the frozen point list is empty.
  bool empty() const noexcept {return points_.empty();}

  // Validates frame, completeness, size, finiteness, and monotonic receipt age.
  // now_steady_ns must come from the same monotonic clock as the receipt time.
  ObservationValidation Validate(
    const ObservationValidationOptions & options,
    std::int64_t now_steady_ns) const;

  // Returns now_steady_ns - received_steady_ns. A negative value means the
  // receipt time is in the future relative to the supplied monotonic clock.
  std::int64_t ReceiptAgeNs(std::int64_t now_steady_ns) const noexcept;

private:
  ObservationMetadata metadata_;
  std::vector<Point3> points_;
};

// Returns a stable diagnostic spelling for an observation source.
const char * ObservationSourceToString(ObservationSource source) noexcept;

// Returns a stable diagnostic spelling for an update semantic.
const char * ObservationUpdateSemanticsToString(
  ObservationUpdateSemantics semantics) noexcept;

// Returns a stable diagnostic spelling for a validation status.
const char * ObservationValidationStatusToString(
  ObservationValidationStatus status) noexcept;

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__INPUT_CONTRACT_H_
