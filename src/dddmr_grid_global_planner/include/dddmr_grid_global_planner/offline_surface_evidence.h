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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__OFFLINE_SURFACE_EVIDENCE_H_
#define DDDMR_GRID_GLOBAL_PLANNER__OFFLINE_SURFACE_EVIDENCE_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "dddmr_grid_global_planner/measurement_evidence.h"
#include "dddmr_grid_global_planner/offline_geometry.h"
#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

struct OfflineStaticMapOptions;

// Result of the real measured-support domain gate. Numeric values retain the
// previous builder enum semantics: zero still means accepted support.
enum class SurfaceSupportGate : std::uint8_t
{
  kOk = 0,
  kSamplingGap = 1,
  kUnsupportedCenter = 2,
};

// One single-layer evaluation over actual point refs. Ordinary, robust and any
// future bounded expanded attempt use this same structure and the same full
// sample/own-cell residual policy.
struct SurfaceAttempt
{
  bool accepted {false};
  SurfaceSupportGate support_gate {SurfaceSupportGate::kUnsupportedCenter};

  // Candidate plane. A rejected attempt may still carry its real fitted plane;
  // `valid` is false only when no ordinary/robust plane was recovered.
  offline_geometry::PlaneFit plane;

  // Exact attempt refs. `point_indices` is every sample in this layer,
  // `inlier_indices` is the ordinary or robust final inlier subset.
  std::vector<std::uint32_t> point_indices;
  std::vector<std::uint32_t> inlier_indices;

  bool ordinary_fit_valid {false};
  bool ordinary_rms_accepted {false};
  bool robust_attempted {false};
  bool robust_used {false};
  bool robust_geometry_accepted {false};
  offline_geometry::PlaneFitStatus robust_status {
    offline_geometry::PlaneFitStatus::kInsufficientPoints
  };
  std::size_t robust_iterations {0};

  MeasurementRejectionReason reason {MeasurementRejectionReason::kNone};
  std::size_t sample_count {0};
  double nearest_support_xy_m {std::numeric_limits<double>::quiet_NaN()};
  bool center_in_support_hull {false};

  // Full-sample signed Z residual summary. Positive samples are obstacle
  // evidence; negative samples are not promoted into an all-neighbourhood
  // physical rejection policy.
  double min_signed_residual {std::numeric_limits<double>::quiet_NaN()};
  double max_signed_residual {std::numeric_limits<double>::quiet_NaN()};
  double max_depression_m {0.0};
  double max_raised_m {0.0};
  std::size_t raised_conflict_sample_count {0U};
  Point3 conflict_point {};
  std::vector<std::uint32_t> raised_conflict_indices;

  // Actual own-cell subset of point_indices, including robust-excluded and
  // regrouped samples. The own-cell step gate is applied to this complete set.
  std::vector<std::uint32_t> own_cell_indices;
  double own_z_min {std::numeric_limits<double>::quiet_NaN()};
  double own_z_max {std::numeric_limits<double>::quiet_NaN()};
  double own_cell_min_signed_residual {
    std::numeric_limits<double>::quiet_NaN()
  };
  double own_cell_max_signed_residual {
    std::numeric_limits<double>::quiet_NaN()
  };
  double own_cell_max_depression_m {0.0};
  double own_cell_max_raised_m {0.0};

  bool HasOwnCellObservation() const noexcept
  {
    return !own_cell_indices.empty();
  }
};

// Validates the exact actual support domain used for one fit. This is the
// previous builder support gate moved into the shared evaluator; it does not
// fabricate or interpolate points.
SurfaceSupportGate ValidateMeasuredSupport(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center,
  const OfflineStaticMapOptions & options,
  double * nearest_xy);

// Evaluates one measured layer in deterministic order:
// minimum count -> support gate -> ordinary fit -> robust fallback -> robust
// support gate -> full-sample residuals -> complete own-cell residuals.
// The own-cell negative gate remains max_step_height_m; no wider negative
// rejection is introduced.
SurfaceAttempt EvaluateSurfaceAttempt(
  const std::vector<Point3> & points,
  const std::vector<std::uint32_t> & indices,
  const Point3 & center,
  const FineCellIndex & cell,
  const OfflineStaticMapOptions & options);

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__OFFLINE_SURFACE_EVIDENCE_H_
