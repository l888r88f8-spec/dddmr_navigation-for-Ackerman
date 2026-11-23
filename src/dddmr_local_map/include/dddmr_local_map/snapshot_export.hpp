#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace dddmr_local_map {

// Flat, owning, CUDA-neutral export of one immutable map snapshot plus the
// TerrainModel arrays built from it. Every field is a scalar, a std::array or
// a std::vector; no Eigen, ROS or CUDA type crosses this boundary.
//
// The export is a deep copy. Once returned it is independent of both the
// VoxelMap and the TerrainModel: publishing a newer map or destroying the
// model cannot change it, and callers never receive a writable reference to
// internal storage.
//
// Cells keep the map's exact circular-buffer layout:
//   linear = (floorMod(x, dx) * dy + floorMod(y, dy)) * dz
//            + floorMod(z, dz)
// with dx/dy/dz taken from dims. Together with origin, dims, voxel_size and
// the raw thresholds this is a lossless representation of MapSnapshot cells;
// no occupancy/terrain grid is approximated.
struct MapCellExport {
  double last_observed_stamp{std::numeric_limits<double>::quiet_NaN()};
  double last_hit_stamp{std::numeric_limits<double>::quiet_NaN()};
  double last_miss_stamp{std::numeric_limits<double>::quiet_NaN()};
  float log_odds{0.0f};
  float hit_x{std::numeric_limits<float>::quiet_NaN()};
  float hit_y{std::numeric_limits<float>::quiet_NaN()};
  float hit_z{std::numeric_limits<float>::quiet_NaN()};
  float hit_max_z{std::numeric_limits<float>::quiet_NaN()};
  uint32_t inflation{0};
  uint8_t observed{0};
  uint8_t padding[3]{0, 0, 0};
};

struct MapConfigExport {
  double voxel_size{0.1};
  int32_t window_x{0};
  int32_t window_y{0};
  int32_t window_z{0};
  double max_range{0.0};
  float hit_log_odds{0.0f};
  float miss_log_odds{0.0f};
  float occupied_threshold{0.0f};
  float free_threshold{0.0f};
  float min_log_odds{0.0f};
  float max_log_odds{0.0f};
  double inflation_radius{0.0};
  uint64_t max_voxels{0};
};

struct TerrainConfigExport {
  double contact_patch_radius{0.0};
  double max_contact_search_z{0.0};
  double max_drop{0.0};
  double max_step{0.0};
  double max_slope_rad{0.0};
  double max_cross_slope_rad{0.0};
  double max_warp{0.0};
  double min_patch_coverage{0.0};
  double min_plane_confidence{0.0};
  double max_snapshot_age{0.0};
  double max_observation_age{0.0};
  double obstacle_search_margin{0.0};
  double cluster_max_dz{0.0};
  double plane_inlier_tol{0.0};
  uint64_t max_patches{0};
  uint8_t unknown_body_policy{0};
};

struct VehicleGeometryExport {
  double wheel_contacts_base[4][3]{};
  double base_contact_height{0.0};
  double body_min[3]{};
  double body_max[3]{};
  double body_clearance{0.0};
};

// One TerrainModel build-time surface candidate. `index` is the candidate's
// position in TerrainSnapshotExport::candidates and is also the value stored
// in candidate_column_indices. All geometry, confidence, residual and stamp
// fields stay double, exactly as computed during the build.
struct TerrainCandidateExport {
  uint64_t index{0};
  int64_t ix{0};
  int64_t iy{0};
  int64_t iz{0};
  int32_t patch_id{-1};
  double point[3]{};
  double local_coefficients[3]{};
  double local_confidence{0.0};
  double local_residual{0.0};
  double last_hit_stamp{std::numeric_limits<double>::quiet_NaN()};
  double last_miss_stamp{std::numeric_limits<double>::quiet_NaN()};
};

// One connected surface patch. candidate_begin/candidate_end delimit the
// patch's candidates in TerrainSnapshotExport::candidates.
struct TerrainPatchExport {
  int32_t patch_id{0};
  uint64_t candidate_begin{0};
  uint64_t candidate_end{0};
  double coefficients[3]{};
  double normal[3]{};
  double rms{0.0};
  double confidence{0.0};
};

struct TerrainSnapshotExport {
  bool valid{false};
  std::string error;
  std::string frame;
  uint64_t snapshot_version{0};
  double snapshot_stamp{std::numeric_limits<double>::quiet_NaN()};
  double voxel_size{0.0};
  std::array<int64_t, 3> origin{{0, 0, 0}};
  std::array<int32_t, 3> dims{{0, 0, 0}};
  float occupied_threshold{0.0f};
  float free_threshold{0.0f};
  MapConfigExport map_config;
  TerrainConfigExport terrain_config;
  VehicleGeometryExport geometry;

  // Raw voxel cells in circular-buffer order, length == columns * dims[2].
  std::vector<MapCellExport> cells;
  std::vector<TerrainCandidateExport> candidates;
  std::vector<TerrainPatchExport> patches;
  // Patch candidate membership. patch_candidate_start has patches.size() + 1
  // entries; values index into candidates. TerrainPatchExport::candidate_begin
  // and candidate_end are the matching [start, end) pair.
  std::vector<uint64_t> patch_candidate_start;
  std::vector<int32_t> patch_candidate_indices;
  // Column index = (iy - origin[1]) * dims[0] + (ix - origin[0]).
  std::vector<uint64_t> candidate_column_start;   // columns + 1
  std::vector<int32_t> candidate_column_indices;  // candidate indices
  std::vector<uint64_t> obstacle_column_start;    // columns + 1
  std::vector<int64_t> obstacle_column_cells;     // absolute voxel z
  std::vector<uint8_t> observed_column_bits;      // one byte per column

  std::size_t columnCount() const {
    if (dims[0] <= 0 || dims[1] <= 0) {
      return 0;
    }
    return static_cast<std::size_t>(dims[0]) *
           static_cast<std::size_t>(dims[1]);
  }
};

static_assert(std::is_trivially_copyable<MapCellExport>::value,
              "MapCellExport must be a flat POD");
static_assert(std::is_trivially_copyable<TerrainCandidateExport>::value,
              "TerrainCandidateExport must be a flat POD");
static_assert(std::is_trivially_copyable<TerrainPatchExport>::value,
              "TerrainPatchExport must be a flat POD");
static_assert(std::is_trivially_copyable<VehicleGeometryExport>::value,
              "VehicleGeometryExport must be a flat POD");

}  // namespace dddmr_local_map
