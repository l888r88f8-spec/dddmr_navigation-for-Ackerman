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

#ifndef REQUEST_POSE_SEMANTICS_H_  // NOLINT(build/header_guard)
#define REQUEST_POSE_SEMANTICS_H_  // NOLINT(build/header_guard)

#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace grid_global_planner
{
namespace request_pose
{

// The all-zero quaternion is the legacy "unset" value; ROS messages normally
// default to the identity quaternion (0, 0, 0, 1), so both spellings must be
// recognized as an unspecified orientation for an otherwise empty pose.
constexpr double kQuaternionTolerance = 1.0e-12;

inline bool QuaternionIsZero(const geometry_msgs::msg::Quaternion & q)
{
  return std::abs(q.x) < kQuaternionTolerance &&
         std::abs(q.y) < kQuaternionTolerance &&
         std::abs(q.z) < kQuaternionTolerance &&
         std::abs(q.w) < kQuaternionTolerance;
}

inline bool QuaternionIsIdentity(const geometry_msgs::msg::Quaternion & q)
{
  return std::abs(q.x) < kQuaternionTolerance &&
         std::abs(q.y) < kQuaternionTolerance &&
         std::abs(q.z) < kQuaternionTolerance &&
         std::abs(q.w - 1.0) < kQuaternionTolerance;
}

inline bool PositionIsZero(const geometry_msgs::msg::Pose & pose)
{
  return pose.position.x == 0.0 &&
         pose.position.y == 0.0 &&
         pose.position.z == 0.0;
}

struct Semantics
{
  bool is_unspecified;
  bool reject_as_empty;
};

// The header stamp is intentionally not part of the empty-pose test. An
// unspecified start with an explicit stamp still resolves TF at that stamp in
// ResolveRequestPose; the stamp is never discarded or replaced with latest TF.
inline Semantics InterpretRequestPose(
  const geometry_msgs::msg::PoseStamped & request,
  bool require_non_empty)
{
  const bool unspecified_orientation =
    QuaternionIsZero(request.pose.orientation) ||
    QuaternionIsIdentity(request.pose.orientation);
  const bool is_unspecified =
    request.header.frame_id.empty() && PositionIsZero(request.pose) &&
    unspecified_orientation;
  return Semantics{is_unspecified, is_unspecified && require_non_empty};
}

}  // namespace request_pose
}  // namespace grid_global_planner

#endif  // REQUEST_POSE_SEMANTICS_H_
