#pragma once

// CUDA-neutral interface for GPU reproduction of
// dddmr_local_map::TerrainModel::evaluate().
//
// This header is deliberately free of Eigen/ROS/CUDA includes so that a device
// rollout kernel may include mppi_cuda_terrain_math.cuh (which includes this
// file) and call the inline __device__ evaluate() once per candidate pose
// without returning to the CPU.
//
// All geometry and thresholds stay double. The snapshot is immutable and is
// uploaded once per map generation by MppiCudaTerrain::upload(). The device
// view points into storage owned by that host object; it stays valid until the
// next successful upload() or destruction.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dddmr_local_map/snapshot_export.hpp"

namespace dddmr_local_map {
struct TerrainQuery;
struct TerrainResult;
}  // namespace dddmr_local_map

namespace local_planner {

// ABI tag embedded in TerrainDeviceView. A device library built against a
// different layout must not consume a view from another build.
constexpr uint32_t kTerrainDeviceAbiVersion = 1U;

// Result of TerrainModel::evaluate(). Numeric values intentionally match
// dddmr_local_map::TerrainStatus so the conversion is a checked cast.
enum class TerrainDeviceStatus : uint8_t {
  kOk = 0,
  kInvalidGeometry,
  kInvalidConfig,
  kInvalidQuery,
  kStale,
  kOutside,
  kUnknownSupport,
  kNoSupport,
  kDropoff,
  kStep,
  kSlope,
  kCrossSlope,
  kWarp,
  kInsufficientCoverage,
  kLowConfidence,
  kInsufficientBodyClearance,
  kBodyCollision,
  kUnknownBody,
  kBuildFailure
};

// Compact detail selector. TerrainResult carries a std::string, which a device
// query cannot allocate; the device writes this code and the host conversion
// expands it to the same human-readable detail text.
enum class TerrainDeviceDetail : uint8_t {
  kNone = 0,
  kInvalidGeometry,
  kInvalidConfig,
  kBuildFailure,
  kInvalidQuery,
  kSnapshotStampInvalid,
  kSnapshotAge,
  kBaseOutside,
  kWheelPatchOutside,
  kProjectedWheelPatchOutside,
  kBodyAabbOutside,
  kObstacleSearchOutside,
  kBodySearchOutside,
  kWheelColumnUnobserved,
  kWheelStale,
  kWheelCoverage,
  kWheelLowConfidence,
  kBracketedLayers,
  kDropoff,
  kNoSupportNearPredicted,
  kNoCandidateUnderWheel,
  kWheelPathStale,
  kWheelPathMissing,
  kWheelPathStep,
  kWheelPlaneRankDeficient,
  kDegeneratePlane,
  kWheelStep,
  kWarp,
  kSlope,
  kCrossSlope,
  kProjectedStale,
  kProjectedUnobserved,
  kProjectedCoverage,
  kProjectedLowConfidence,
  kProjectedDropoff,
  kProjectedNoSupport,
  kProjectedRankDeficient,
  kNotConverged,
  kContactOffSupport,
  kBodyClearance,
  kBodyCollision,
  kBodyStale,
  kBodyUnknown
};

struct TerrainDeviceWheel {
  uint8_t supported{0};
  int32_t patch_id{-1};
  double predicted_z{0.0};
  double support_z{0.0};
  double coverage{0.0};
};

// Pure POD query/result. Orientation is stored as Eigen's coefficient order
// (x, y, z, w); the host conversion constructs the same Quat.
struct TerrainDeviceQuery {
  double stamp{0.0};
  double position[3]{0.0, 0.0, 0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct TerrainDeviceResult {
  uint8_t status{static_cast<uint8_t>(TerrainDeviceStatus::kBuildFailure)};
  uint8_t detail{static_cast<uint8_t>(TerrainDeviceDetail::kNone)};
  uint8_t reserved[2]{0, 0};
  int32_t selected_patch{-1};
  double projected_position[3]{0.0, 0.0, 0.0};
  double projected_orientation[4]{0.0, 0.0, 0.0, 1.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double clearance{0.0};
  double obstacle_clearance{0.0};
  TerrainDeviceWheel wheels[4]{};
};

// Flattened scalar configuration, identical to TerrainConfigExport. Kept as a
// plain struct so a device compilation unit needs no parser for the export.
struct TerrainDeviceConfig {
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

struct TerrainDeviceGeometry {
  double wheel_contacts_base[4][3]{};
  double base_contact_height{0.0};
  double body_min[3]{0.0, 0.0, 0.0};
  double body_max[3]{0.0, 0.0, 0.0};
  double body_clearance{0.0};
};

// Snapshot scalars plus raw device pointers. No pointer is dereferenced unless
// `valid` and the corresponding count allow it. Cells keep the exact circular
// buffer order exported by TerrainSnapshotExport.
struct TerrainDeviceView {
  uint32_t abi_version{kTerrainDeviceAbiVersion};
  uint8_t valid{0};
  uint8_t geometry_valid{0};
  uint8_t config_valid{0};
  uint8_t reserved{0};
  uint64_t snapshot_version{0};
  double snapshot_stamp{0.0};
  double voxel_size{0.0};
  int64_t origin[3]{0, 0, 0};
  int32_t dims[3]{0, 0, 0};
  float occupied_threshold{0.0f};
  float free_threshold{0.0f};
  size_t column_count{0};
  size_t cell_count{0};
  size_t candidate_count{0};
  size_t patch_count{0};
  size_t candidate_column_count{0};
  size_t obstacle_column_count{0};

  const dddmr_local_map::MapCellExport* cells{nullptr};
  const dddmr_local_map::TerrainCandidateExport* candidates{nullptr};
  const dddmr_local_map::TerrainPatchExport* patches{nullptr};
  const uint64_t* candidate_column_start{nullptr};
  const int32_t* candidate_column_indices{nullptr};
  const uint64_t* obstacle_column_start{nullptr};
  const int64_t* obstacle_column_cells{nullptr};
  const uint8_t* observed_column_bits{nullptr};

  TerrainDeviceConfig config{};
  TerrainDeviceGeometry geometry{};
};

// Host conversion. The .cu and the no-CUDA stub both provide this symbol, so a
// caller may convert a memorized device result after cudaMemcpy without
// linking a CUDA runtime.
dddmr_local_map::TerrainResult terrainResultFromDevice(
    const TerrainDeviceResult& device);
bool terrainResultFromDevice(const TerrainDeviceResult& device,
                             dddmr_local_map::TerrainResult* out,
                             std::string* error);
const char* terrainDeviceDetailText(TerrainDeviceDetail detail);
const char* terrainDeviceStatusText(TerrainDeviceStatus status);

// Host-side owner of one immutable uploaded terrain snapshot. There is no
// global state and no CPU fallback: if CUDA is unavailable every operation
// fails with an explicit error and available() returns false.
class MppiCudaTerrain {
 public:
  MppiCudaTerrain();
  ~MppiCudaTerrain();
  MppiCudaTerrain(const MppiCudaTerrain&) = delete;
  MppiCudaTerrain& operator=(const MppiCudaTerrain&) = delete;
  MppiCudaTerrain(MppiCudaTerrain&&) noexcept;
  MppiCudaTerrain& operator=(MppiCudaTerrain&&) noexcept;

  // True only for a real CUDA implementation with a usable device. The stub
  // always returns false.
  bool available() const;

  // Deep-copies `snapshot` into reusable device buffers. A failed upload leaves
  // the previous snapshot intact (no mixed-generation view). Device buffers are
  // reused across cycles where allocation sizes permit.
  bool upload(const dddmr_local_map::TerrainSnapshotExport& snapshot,
              std::string* error);

  // Evaluates every query on the GPU with the exact TerrainModel::evaluate
  // algorithm. Output size always equals query size on success. On failure the
  // previous output contents are unspecified and false is returned.
  bool evaluateBatch(const std::vector<dddmr_local_map::TerrainQuery>& queries,
                     std::vector<dddmr_local_map::TerrainResult>* results,
                     std::string* error);
  bool evaluateBatch(const std::vector<dddmr_local_map::TerrainQuery>& queries,
                     std::vector<dddmr_local_map::TerrainResult>& results,
                     std::string* error);

  // View pointing at device storage owned by this object. It remains valid
  // until the next successful upload() or destruction. The const reference is
  // the cheapest form for a rollout kernel.
  const TerrainDeviceView& deviceView() const;

  bool hasSnapshot() const;
  uint64_t snapshotVersion() const;
  double snapshotStamp() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<MppiCudaTerrain> makeMppiCudaTerrain();

static_assert(sizeof(TerrainDeviceWheel) == 32,
              "TerrainDeviceWheel must stay a stable POD");
static_assert(sizeof(TerrainDeviceQuery) == 56,
              "TerrainDeviceQuery must stay a stable POD");

}  // namespace local_planner
