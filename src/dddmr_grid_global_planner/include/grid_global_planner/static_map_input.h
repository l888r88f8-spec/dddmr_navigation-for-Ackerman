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

#ifndef GRID_GLOBAL_PLANNER__STATIC_MAP_INPUT_H_
#define GRID_GLOBAL_PLANNER__STATIC_MAP_INPUT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "grid_global_planner/terrain_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace grid_global_planner
{

struct StaticMapInputOptions
{
  // false: ground-only (mapground). true: full-cloud (mapcloud) plus the
  // mapground surface hint. The hint is required in full-cloud mode so the
  // first build is deterministic rather than racing the two latched topics.
  bool global_cloud_mode{false};
  // Static map clouds must already be expressed in this frame. There is no TF
  // lookup for the one-shot startup map.
  std::string frame_id{"map"};
  std::size_t max_points{20'000'000};
  // Explicit legacy GlobalMap static obstacle injection. When true, obstacle
  // points from mapcloud are injected even in ground-only terrain mode. It
  // controls obstacle injection only; source identity readiness is separate.
  bool use_global_as_obstacles{false};
  // Internal readiness option (never a ROS parameter). The live scene gate
  // must derive its source identity from the exact raw mapground/mapcloud pair
  // Grid built from. When set, both raw clouds are required and the raw
  // mapcloud is retained for provenance even though it is not injected.
  bool require_global_source_identity{false};
};

enum class StaticMapInputStatus : std::uint8_t
{
  kAccepted = 0,
  kIgnored = 1,
  kInvalid = 2,
};

struct StaticMapInputResult
{
  StaticMapInputStatus status{StaticMapInputStatus::kInvalid};
  std::size_t point_count{0};
  std::string detail;
};

// One-shot accumulator for the static map startup contract.
//
// It validates complete PointCloud2 replacements and keeps the last valid
// ground and full cloud. It performs no TF, file I/O, or reconstruction. The
// node owns the one-build latch: after MarkBuilt() every later publication is
// ignored, while an invalid cloud never advances readiness. Keeping this logic
// ROS-message-only (no node, executor, or sockets) makes the validation and
// readiness contract directly unit-testable.
class StaticMapInput
{
public:
  explicit StaticMapInput(StaticMapInputOptions options);

  StaticMapInputResult AcceptGround(
    const sensor_msgs::msg::PointCloud2 & cloud);
  StaticMapInputResult AcceptGlobal(
    const sensor_msgs::msg::PointCloud2 & cloud);

  // True exactly when all inputs required by the configured mode are present
  // and no successful build has been latched yet.
  bool ReadyToBuild() const noexcept;
  bool IsBuilt() const noexcept;
  void MarkBuilt() noexcept;

  const std::vector<Point3> & ground_points() const noexcept
  {
    return ground_points_;
  }

  const std::vector<Point3> & global_points() const noexcept
  {
    return global_points_;
  }

private:
  StaticMapInputResult Convert(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<Point3> * points) const;

  // True when the raw mapcloud is needed at all: as ground data, as injected
  // static evidence, or only for source provenance/readiness.
  bool RequiresGlobalRaw() const noexcept
  {
    return options_.global_cloud_mode || options_.use_global_as_obstacles ||
           options_.require_global_source_identity;
  }

  StaticMapInputOptions options_;
  std::vector<Point3> ground_points_;
  std::vector<Point3> global_points_;
  bool have_ground_{false};
  bool have_global_{false};
  bool built_{false};
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__STATIC_MAP_INPUT_H_
