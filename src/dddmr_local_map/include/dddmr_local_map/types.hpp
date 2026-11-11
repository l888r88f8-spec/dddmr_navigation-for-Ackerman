#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace dddmr_local_map {

using Vec3 = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

struct VoxelKey {
  int64_t x{0};
  int64_t y{0};
  int64_t z{0};

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept;
};

struct ColumnKey {
  int64_t x{0};
  int64_t y{0};

  bool operator==(const ColumnKey& other) const {
    return x == other.x && y == other.y;
  }
};

struct ColumnKeyHash {
  std::size_t operator()(const ColumnKey& key) const noexcept;
};

struct Validation {
  bool ok{true};
  std::string error;

  explicit operator bool() const { return ok; }
  static Validation success() { return Validation{}; }
  static Validation failure(std::string message) {
    return Validation{false, std::move(message)};
  }
};

enum class Occupancy : uint8_t { kUnknown = 0, kFree = 1, kOccupied = 2 };

const char* toString(Occupancy value);

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct MapLookup {
  bool in_bounds{false};
  bool observed{false};
  bool inflated{false};
  Occupancy occupancy{Occupancy::kUnknown};
  // Timestamp of the last actual evidence applied to this voxel. NaN for
  // invalid / unobserved cells. This is per-cell observation age, not the
  // overall snapshot stamp.
  double last_observed_stamp{kNaN};
  // Separate HIT and MISS timestamps. A voxel can still be log-odds occupied
  // after a current miss; support must not treat that stale floor as fresh.
  double last_hit_stamp{kNaN};
  double last_miss_stamp{kNaN};
  // Mean of the actual ray endpoints that hit this voxel in the latest
  // accepted cloud frame. NaN when the voxel has never been hit. This is the
  // measured surface sample, not the occupied voxel top.
  Vec3 hit_position{Vec3(kNaN, kNaN, kNaN)};
  // Highest actual HIT endpoint z in this voxel in the latest accepted cloud
  // frame. NaN when the voxel has never been hit. A low centroid must not hide
  // a higher return in the same voxel.
  double hit_max_z{kNaN};
};

struct Ray {
  Vec3 origin{Vec3::Zero()};
  Vec3 endpoint{Vec3::Zero()};
};

}  // namespace dddmr_local_map
