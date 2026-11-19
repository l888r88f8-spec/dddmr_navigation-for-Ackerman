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

#include "grid_global_planner/static_map_input.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "sensor_msgs/msg/point_field.hpp"

namespace grid_global_planner
{
namespace
{

StaticMapInputResult Invalid(std::string detail)
{
  StaticMapInputResult result;
  result.status = StaticMapInputStatus::kInvalid;
  result.detail = std::move(detail);
  return result;
}

const sensor_msgs::msg::PointField * FindField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

}  // namespace

StaticMapInput::StaticMapInput(StaticMapInputOptions options)
: options_(std::move(options))
{
  if (options_.max_points == 0) {
    options_.max_points = 1;
  }
}

StaticMapInputResult StaticMapInput::Convert(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<Point3> * points) const
{
  if (points == nullptr) {
    return Invalid("null point output");
  }
  if (options_.frame_id.empty()) {
    return Invalid("static map frame is empty");
  }
  if (cloud.header.frame_id != options_.frame_id) {
    return Invalid(
      "cloud frame '" + cloud.header.frame_id + "' does not match '" +
      options_.frame_id + "'");
  }
  if (cloud.is_bigendian) {
    return Invalid("big-endian point clouds are not accepted");
  }
  if (cloud.width == 0 || cloud.height == 0) {
    return Invalid("point cloud is empty");
  }
  const std::uint64_t count =
    static_cast<std::uint64_t>(cloud.width) * cloud.height;
  if (count > static_cast<std::uint64_t>(options_.max_points)) {
    return Invalid(
      "point cloud has " + std::to_string(count) +
      " points, above the configured limit");
  }
  const sensor_msgs::msg::PointField * x_field = FindField(cloud, "x");
  const sensor_msgs::msg::PointField * y_field = FindField(cloud, "y");
  const sensor_msgs::msg::PointField * z_field = FindField(cloud, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    return Invalid("point cloud is missing x/y/z fields");
  }
  const auto valid_field = [](const sensor_msgs::msg::PointField & field) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
             field.count == 1;
    };
  if (!valid_field(*x_field) || !valid_field(*y_field) ||
    !valid_field(*z_field))
  {
    return Invalid("x/y/z fields must be scalar FLOAT32");
  }
  if (cloud.point_step == 0) {
    return Invalid("point_step is zero");
  }
  const std::uint64_t row_width =
    static_cast<std::uint64_t>(cloud.point_step) * cloud.width;
  if (cloud.row_step < row_width) {
    return Invalid("row_step is smaller than one point row");
  }
  const std::uint64_t required =
    static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
  if (required > cloud.data.size()) {
    return Invalid("point cloud data is smaller than its declared shape");
  }
  const std::uint64_t point_step = cloud.point_step;
  if (static_cast<std::uint64_t>(x_field->offset) + sizeof(float) > point_step ||
    static_cast<std::uint64_t>(y_field->offset) + sizeof(float) > point_step ||
    static_cast<std::uint64_t>(z_field->offset) + sizeof(float) > point_step)
  {
    return Invalid("x/y/z field offset exceeds point_step");
  }

  points->clear();
  points->reserve(static_cast<std::size_t>(count));
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::size_t row_offset =
      static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset =
        row_offset + static_cast<std::size_t>(column) * cloud.point_step;
      float x = std::numeric_limits<float>::quiet_NaN();
      float y = std::numeric_limits<float>::quiet_NaN();
      float z = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(&x, cloud.data.data() + offset + x_field->offset, sizeof(x));
      std::memcpy(&y, cloud.data.data() + offset + y_field->offset, sizeof(y));
      std::memcpy(&z, cloud.data.data() + offset + z_field->offset, sizeof(z));
      const Point3 point{x, y, z};
      if (!point.IsFinite()) {
        points->clear();
        return Invalid("point cloud contains a non-finite point");
      }
      points->push_back(point);
    }
  }
  if (points->empty()) {
    return Invalid("point cloud is empty");
  }
  StaticMapInputResult result;
  result.status = StaticMapInputStatus::kAccepted;
  result.point_count = points->size();
  return result;
}

StaticMapInputResult StaticMapInput::AcceptGround(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (built_) {
    StaticMapInputResult result;
    result.status = StaticMapInputStatus::kIgnored;
    result.detail = "static map is already built";
    return result;
  }
  std::vector<Point3> points;
  StaticMapInputResult result = Convert(cloud, &points);
  if (result.status != StaticMapInputStatus::kAccepted) {
    return result;
  }
  ground_points_ = std::move(points);
  have_ground_ = true;
  return result;
}

StaticMapInputResult StaticMapInput::AcceptGlobal(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (built_) {
    StaticMapInputResult result;
    result.status = StaticMapInputStatus::kIgnored;
    result.detail = "static map is already built";
    return result;
  }
  if (!RequiresGlobalRaw()) {
    StaticMapInputResult result;
    result.status = StaticMapInputStatus::kIgnored;
    result.detail = "global cloud is ignored in ground-only mode";
    return result;
  }
  std::vector<Point3> points;
  StaticMapInputResult result = Convert(cloud, &points);
  if (result.status != StaticMapInputStatus::kAccepted) {
    return result;
  }
  global_points_ = std::move(points);
  have_global_ = true;
  return result;
}

bool StaticMapInput::ReadyToBuild() const noexcept
{
  if (built_) {
    return false;
  }
  if (!have_ground_) {
    return false;
  }
  return !RequiresGlobalRaw() || have_global_;
}

bool StaticMapInput::IsBuilt() const noexcept
{
  return built_;
}

void StaticMapInput::MarkBuilt() noexcept
{
  built_ = true;
}

}  // namespace grid_global_planner
