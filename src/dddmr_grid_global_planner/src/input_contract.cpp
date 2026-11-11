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

#include "dddmr_grid_global_planner/input_contract.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace dddmr_grid_global_planner
{
namespace
{

// ROS frame IDs may appear with a single leading slash. Normalize only that
// spelling difference; do not rewrite or guess a different frame.
std::string NormalizeFrameId(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

// Saturating int64 subtraction so receipt-age validation is safe even with
// adversarial timestamps.
std::int64_t SaturatingSub(std::int64_t lhs, std::int64_t rhs) noexcept
{
  if (rhs < 0 && lhs > std::numeric_limits<std::int64_t>::max() + rhs) {
    return std::numeric_limits<std::int64_t>::max();
  }
  if (rhs > 0 && lhs < std::numeric_limits<std::int64_t>::min() + rhs) {
    return std::numeric_limits<std::int64_t>::min();
  }
  return lhs - rhs;
}

ObservationValidation Reject(
  ObservationValidationStatus status,
  std::string detail)
{
  return ObservationValidation{false, status, std::move(detail)};
}

}  // namespace

TerrainPointObservation::TerrainPointObservation(
  ObservationMetadata metadata, std::vector<Point3> points)
: metadata_(std::move(metadata)), points_(std::move(points)) {}

ObservationValidation TerrainPointObservation::Validate(
  const ObservationValidationOptions & options,
  std::int64_t now_steady_ns) const
{
  if (options.max_receipt_age_ns < 0 || options.future_tolerance_ns < 0 ||
    options.max_points == 0)
  {
    return Reject(
      ObservationValidationStatus::kInvalidOptions,
      "max_receipt_age_ns and future_tolerance_ns must be non-negative, "
      "and max_points must be positive");
  }
  if (options.require_frame_match && options.expected_frame_id.empty()) {
    return Reject(
      ObservationValidationStatus::kInvalidOptions,
      "expected_frame_id must be set when frame matching is required");
  }
  if (metadata_.frame_id.empty()) {
    return Reject(
      ObservationValidationStatus::kEmptyFrame,
      "observation frame_id is empty");
  }
  if (options.require_frame_match &&
    NormalizeFrameId(metadata_.frame_id) !=
    NormalizeFrameId(options.expected_frame_id))
  {
    return Reject(
      ObservationValidationStatus::kFrameMismatch,
      "observation frame '" + metadata_.frame_id +
      "' does not match expected frame '" +
      options.expected_frame_id + "'");
  }
  if (!metadata_.complete) {
    return Reject(
      ObservationValidationStatus::kIncomplete,
      "observation is not marked complete/frozen");
  }
  if (points_.empty() && !options.allow_empty) {
    return Reject(
      ObservationValidationStatus::kEmptyObservation,
      "observation has no points; empty terrain is unknown, not "
      "free");
  }
  if (points_.size() > options.max_points) {
    return Reject(
      ObservationValidationStatus::kTooManyPoints,
      "observation has " + std::to_string(points_.size()) +
      " points, limit is " +
      std::to_string(options.max_points));
  }
  for (std::size_t index = 0; index < points_.size(); ++index) {
    if (!points_[index].IsFinite()) {
      return Reject(
        ObservationValidationStatus::kNonFinitePoint,
        "observation contains a non-finite point at index " +
        std::to_string(index));
    }
  }
  const std::int64_t age_ns = ReceiptAgeNs(now_steady_ns);
  if (age_ns < -options.future_tolerance_ns) {
    return Reject(
      ObservationValidationStatus::kFutureReceipt,
      "observation receipt is in the future by " +
      std::to_string(-age_ns) + " ns");
  }
  if (age_ns > options.max_receipt_age_ns) {
    return Reject(
      ObservationValidationStatus::kStaleReceipt,
      "observation receipt age " + std::to_string(age_ns) +
      " ns exceeds " +
      std::to_string(options.max_receipt_age_ns) + " ns");
  }
  return ObservationValidation{true, ObservationValidationStatus::kValid,
    "ok"};
}

std::int64_t TerrainPointObservation::ReceiptAgeNs(
  std::int64_t now_steady_ns) const noexcept
{
  return SaturatingSub(now_steady_ns, metadata_.received_steady_ns);
}

const char * ObservationSourceToString(ObservationSource source) noexcept
{
  switch (source) {
    case ObservationSource::kUnknown:
      return "unknown";
    case ObservationSource::kStaticMap:
      return "static_map";
    case ObservationSource::kGroundSegmentation:
      return "ground_segmentation";
    case ObservationSource::kLidarGround:
      return "lidar_ground";
    case ObservationSource::kDepthGround:
      return "depth_ground";
  }
  return "unknown";
}

const char * ObservationUpdateSemanticsToString(
  ObservationUpdateSemantics semantics) noexcept
{
  switch (semantics) {
    case ObservationUpdateSemantics::kReplace:
      return "replace";
    case ObservationUpdateSemantics::kMerge:
      return "merge";
    case ObservationUpdateSemantics::kClear:
      return "clear";
  }
  return "unknown";
}

const char * ObservationValidationStatusToString(
  ObservationValidationStatus status) noexcept
{
  switch (status) {
    case ObservationValidationStatus::kValid:
      return "valid";
    case ObservationValidationStatus::kEmptyFrame:
      return "empty_frame";
    case ObservationValidationStatus::kFrameMismatch:
      return "frame_mismatch";
    case ObservationValidationStatus::kIncomplete:
      return "incomplete";
    case ObservationValidationStatus::kEmptyObservation:
      return "empty_observation";
    case ObservationValidationStatus::kTooManyPoints:
      return "too_many_points";
    case ObservationValidationStatus::kNonFinitePoint:
      return "non_finite_point";
    case ObservationValidationStatus::kFutureReceipt:
      return "future_receipt";
    case ObservationValidationStatus::kStaleReceipt:
      return "stale_receipt";
    case ObservationValidationStatus::kInvalidOptions:
      return "invalid_options";
  }
  return "unknown";
}

}  // namespace dddmr_grid_global_planner
