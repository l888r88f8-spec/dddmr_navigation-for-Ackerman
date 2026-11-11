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

#ifndef OFFLINE_PCD_LOADER_H_
#define OFFLINE_PCD_LOADER_H_

#include <string>
#include <vector>

#include "dddmr_grid_global_planner/terrain_types.h"

namespace dddmr_grid_global_planner
{

// Loads the x/y/z fields from one PCD file into the ROS-independent point type.
// Supported schema: the x, y and z fields must each be present with COUNT 1
// and datatype FLOAT32, and each field offset must fit inside point_step.
// Additional fields (for example intensity) are ignored. Missing, FLOAT64,
// integer, multi-count or misaligned coordinate fields are rejected with a
// diagnostic instead of being defaulted to zero. The file is read once at
// startup; this is not a sensor subscription and it never invents missing
// points. Non-finite points are retained for the builder to count/reject
// explicitly instead of being silently dropped here.
bool LoadOfflinePcdPoints(
  const std::string & path,
  std::vector<Point3> * points,
  std::string * detail);

}  // namespace dddmr_grid_global_planner

#endif  // OFFLINE_PCD_LOADER_H_
