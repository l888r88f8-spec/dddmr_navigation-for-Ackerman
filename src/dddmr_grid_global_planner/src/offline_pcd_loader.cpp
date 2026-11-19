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

#include "offline_pcd_loader.h"  // NOLINT(build/include_subdir): private src/ header

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <exception>
#include <string>
#include <vector>

namespace grid_global_planner
{
namespace
{

// Map a PCL point-field datatype code to the name shown in loader diagnostics.
std::string FieldDatatypeName(std::uint8_t datatype)
{
  switch (datatype) {
    case pcl::PCLPointField::INT8:
      return "INT8";
    case pcl::PCLPointField::UINT8:
      return "UINT8";
    case pcl::PCLPointField::INT16:
      return "INT16";
    case pcl::PCLPointField::UINT16:
      return "UINT16";
    case pcl::PCLPointField::INT32:
      return "INT32";
    case pcl::PCLPointField::UINT32:
      return "UINT32";
    case pcl::PCLPointField::FLOAT32:
      return "FLOAT32";
    case pcl::PCLPointField::FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN(" + std::to_string(datatype) + ")";
  }
}

// Reject any header that PCL could map incompletely (leaving coordinates at
// their default zero value) or with the wrong in-buffer width.
bool ValidateXyzSchema(
  const pcl::PCLPointCloud2 & container,
  std::string * detail)
{
  constexpr const char * kRequiredFields[] = {"x", "y", "z"};
  for (const char * required : kRequiredFields) {
    const pcl::PCLPointField * field = nullptr;
    for (const pcl::PCLPointField & candidate : container.fields) {
      if (candidate.name == required) {
        field = &candidate;
        break;
      }
    }
    if (field == nullptr) {
      std::string available;
      for (const pcl::PCLPointField & candidate : container.fields) {
        if (!available.empty()) {
          available += ", ";
        }
        available += candidate.name;
      }
      if (available.empty()) {
        available = "<none>";
      }
      if (detail != nullptr) {
        *detail = "PCD header is missing required field '" +
          std::string(required) + "' (fields: " + available +
          "); offline loader requires FLOAT32 x/y/z";
      }
      return false;
    }
    if (field->count != 1U) {
      if (detail != nullptr) {
        *detail = "PCD field '" + field->name + "' has unsupported count " +
          std::to_string(field->count) +
          "; offline loader requires COUNT 1";
      }
      return false;
    }
    if (field->datatype != pcl::PCLPointField::FLOAT32) {
      if (detail != nullptr) {
        *detail = "PCD field '" + field->name +
          "' has unsupported datatype " +
          FieldDatatypeName(field->datatype) +
          "; offline loader requires FLOAT32";
      }
      return false;
    }
    if (static_cast<std::uint64_t>(field->offset) + sizeof(float) >
      static_cast<std::uint64_t>(container.point_step))
    {
      if (detail != nullptr) {
        *detail = "PCD field '" + field->name + "' offset " +
          std::to_string(field->offset) +
          " plus FLOAT32 exceeds point_step " +
          std::to_string(container.point_step);
      }
      return false;
    }
  }

  const std::uint64_t minimum_row_step =
    static_cast<std::uint64_t>(container.point_step) * container.width;
  if (container.row_step < minimum_row_step) {
    if (detail != nullptr) {
      *detail = "PCD row_step " + std::to_string(container.row_step) +
        " is smaller than point_step*width " +
        std::to_string(minimum_row_step);
    }
    return false;
  }
  const std::uint64_t minimum_data_size =
    static_cast<std::uint64_t>(container.row_step) * container.height;
  if (container.data.size() < minimum_data_size) {
    if (detail != nullptr) {
      *detail = "PCD data size " + std::to_string(container.data.size()) +
        " is smaller than row_step*height " +
        std::to_string(minimum_data_size);
    }
    return false;
  }
  return true;
}

}  // namespace

bool LoadOfflinePcdPoints(
  const std::string & path,
  std::vector<Point3> * points,
  std::string * detail)
{
  if (points == nullptr) {
    if (detail != nullptr) {
      *detail = "output point vector is null";
    }
    return false;
  }
  points->clear();
  if (path.empty()) {
    if (detail != nullptr) {
      *detail = "PCD path is empty";
    }
    return false;
  }

  // Read the generic PCD container first. This keeps files with or without the
  // optional intensity field readable and lets the PCD reader validate the
  // header before any point is exposed.
  pcl::PCLPointCloud2 container;
  try {
    if (pcl::io::loadPCDFile(path, container) != 0) {
      if (detail != nullptr) {
        *detail = "PCL could not read PCD file: " + path;
      }
      return false;
    }
    if (!ValidateXyzSchema(container, detail)) {
      return false;
    }
    const std::uint64_t expected_points =
      static_cast<std::uint64_t>(container.width) * container.height;
    if (expected_points == 0U) {
      return true;
    }
    // The schema check above guarantees that PCL can materialize all three
    // FLOAT32 coordinates; a short conversion is still treated as a failure.
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(container, cloud);
    if (static_cast<std::uint64_t>(cloud.points.size()) != expected_points) {
      if (detail != nullptr) {
        *detail = "PCL converted " +
          std::to_string(cloud.points.size()) + " of " +
          std::to_string(expected_points) + " PCD points";
      }
      return false;
    }
    points->reserve(cloud.points.size());
    for (const pcl::PointXYZ & point : cloud.points) {
      points->push_back(
        Point3{static_cast<double>(point.x),
          static_cast<double>(point.y),
          static_cast<double>(point.z)});
    }
  } catch (const std::exception & exception) {
    if (detail != nullptr) {
      *detail = "PCD load failed for '" + path + "': " + exception.what();
    }
    points->clear();
    return false;
  }
  if (detail != nullptr) {
    *detail = "loaded " + std::to_string(points->size()) + " points";
  }
  return true;
}

}  // namespace grid_global_planner
