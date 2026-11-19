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

#ifndef PERCEPTION_OBSERVATION_INGESTION_H_
#define PERCEPTION_OBSERVATION_INGESTION_H_

#include <perception_3d/perception_snapshot.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "grid_global_planner/source_observation.h"

namespace grid_global_planner
{

// Hard bounds applied while freezing an immutable perception export into
// planner source observations. A source that exceeds a bound is rejected as a
// whole; it is never truncated into a misleading partial clear.
struct PerceptionIngestionLimits
{
  std::size_t max_sources{64};
  std::size_t max_records{200'000};
};

// Conversion result. observations is populated only when ok is true.
struct PerceptionIngestionResult
{
  bool ok{false};
  std::string detail;
  std::vector<SourceObservationSnapshot> observations;
};

// Converts one immutable perception export into planner source observations.
//
// Only the expected map frame is accepted: this adapter never relabels an
// unknown or source-frame cloud by assigning the configured map frame. Known
// volumes are copied through as explicit evidence; a point-only occupied record
// keeps min_z == max_z and therefore contributes occupied/unknown evidence
// only. Empty observations with valid volume/clearing evidence remain usable.
PerceptionIngestionResult ConvertPerceptionSnapshotToSourceObservations(
  const perception_3d::PerceptionSnapshot & snapshot,
  const std::string & expected_frame, std::int64_t received_steady_ns,
  std::uint64_t terrain_revision, bool terrain_revision_known,
  const PerceptionIngestionLimits & limits);

}  // namespace grid_global_planner

#endif  // PERCEPTION_OBSERVATION_INGESTION_H_
