#include "local_planner/mppi_cuda_terrain.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

#include "dddmr_local_map/terrain.hpp"
#include "local_planner/mppi_cuda_terrain_math.cuh"

namespace local_planner {
namespace {

using dddmr_local_map::TerrainSnapshotExport;

std::string cudaErrorText(const char* operation, cudaError_t error) {
  std::ostringstream stream;
  stream << operation << " failed: " << cudaGetErrorString(error);
  return stream.str();
}

bool finiteScalar(double value) { return std::isfinite(value); }

// One generational buffer. Arrays keep their allocation capacity so repeated
// uploads into the same buffer need only cudaMemcpy; growing an array allocates
// a replacement before releasing the old storage. The view is never published
// until every array copied successfully.
struct DeviceSnapshot {
  static constexpr size_t kArrayCount = 8;

  struct Allocation {
    void* data{nullptr};
    size_t capacity{0};
  };

  TerrainDeviceView view;
  std::array<Allocation, kArrayCount> arrays;

  DeviceSnapshot() = default;
  ~DeviceSnapshot() {
    for (Allocation& allocation : arrays) {
      if (allocation.data != nullptr) {
        cudaFree(allocation.data);
      }
    }
  }
  DeviceSnapshot(const DeviceSnapshot&) = delete;
  DeviceSnapshot& operator=(const DeviceSnapshot&) = delete;

  bool ensure(const size_t index, const size_t bytes, std::string* error) {
    Allocation& allocation = arrays[index];
    if (bytes <= allocation.capacity) {
      return true;
    }
    void* replacement = nullptr;
    const cudaError_t status = cudaMalloc(&replacement, bytes);
    if (status != cudaSuccess) {
      if (error != nullptr) {
        *error = cudaErrorText("cudaMalloc(snapshot)", status);
      }
      return false;
    }
    if (allocation.data != nullptr) {
      cudaFree(allocation.data);
    }
    allocation.data = replacement;
    allocation.capacity = bytes;
    return true;
  }
};

template <typename T>
bool uploadArray(const std::vector<T>& source, DeviceSnapshot& device,
                 const size_t index, const T** destination,
                 std::string* error) {
  if (source.size() > std::numeric_limits<size_t>::max() / sizeof(T)) {
    if (error != nullptr) {
      *error = "snapshot array byte size overflows size_t";
    }
    return false;
  }
  if (!device.ensure(index, source.size() * sizeof(T), error)) {
    return false;
  }
  void* memory = device.arrays[index].data;
  if (!source.empty()) {
    const cudaError_t copy =
        cudaMemcpy(memory, source.data(), source.size() * sizeof(T),
                   cudaMemcpyHostToDevice);
    if (copy != cudaSuccess) {
      if (error != nullptr) {
        *error = cudaErrorText("cudaMemcpy(snapshot)", copy);
      }
      return false;
    }
  }
  *destination = static_cast<const T*>(memory);
  return true;
}

__global__ void evaluateTerrainKernel(TerrainDeviceView view,
                                      const TerrainDeviceQuery* queries,
                                      TerrainDeviceResult* results,
                                      size_t count) {
  const size_t index =
      static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count) {
    results[index] = cuda_terrain::evaluate(view, queries[index]);
  }
}

bool validateExport(const TerrainSnapshotExport& snapshot,
                    std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = "invalid terrain snapshot export: " + message;
    }
    return false;
  };
  if (!snapshot.valid) {
    return fail(snapshot.error.empty() ? "export is marked invalid"
                                       : snapshot.error);
  }
  if (snapshot.dims[0] <= 0 || snapshot.dims[1] <= 0 ||
      snapshot.dims[2] <= 0) {
    return fail("dims must be positive");
  }
  if (!finiteScalar(snapshot.voxel_size) || snapshot.voxel_size <= 0.0) {
    return fail("voxel_size must be finite and positive");
  }
  if (!finiteScalar(snapshot.snapshot_stamp)) {
    return fail("snapshot_stamp must be finite");
  }
  if (!std::isfinite(snapshot.terrain_config.contact_patch_radius) ||
      snapshot.terrain_config.contact_patch_radius <= 0.0 ||
      !std::isfinite(snapshot.terrain_config.plane_inlier_tol) ||
      snapshot.terrain_config.plane_inlier_tol <= 0.0) {
    return fail("terrain tolerances must be finite and positive");
  }
  const size_t columns = snapshot.columnCount();
  const size_t depth = static_cast<size_t>(snapshot.dims[2]);
  if (columns > std::numeric_limits<size_t>::max() / depth) {
    return fail("cell count overflows size_t");
  }
  const size_t cells = columns * depth;
  if (columns == 0 || snapshot.cells.size() != cells) {
    return fail("cell array does not match origin/dims");
  }
  if (snapshot.candidate_column_start.size() != columns + 1U ||
      snapshot.obstacle_column_start.size() != columns + 1U ||
      snapshot.observed_column_bits.size() != columns) {
    return fail("column arrays do not match dims");
  }
  const auto valid_csr = [](const std::vector<uint64_t>& start,
                            size_t payload) {
    if (start.empty() || start.back() != payload) {
      return false;
    }
    for (size_t i = 0; i + 1U < start.size(); ++i) {
      if (start[i] > start[i + 1U] || start[i + 1U] > payload) {
        return false;
      }
    }
    return true;
  };
  if (!valid_csr(snapshot.candidate_column_start,
                 snapshot.candidate_column_indices.size()) ||
      !valid_csr(snapshot.obstacle_column_start,
                 snapshot.obstacle_column_cells.size()) ||
      !valid_csr(snapshot.patch_candidate_start,
                 snapshot.patch_candidate_indices.size())) {
    return fail("CSR arrays are inconsistent");
  }
  for (int32_t index : snapshot.candidate_column_indices) {
    if (index < 0 ||
        static_cast<size_t>(index) >= snapshot.candidates.size()) {
      return fail("candidate CSR references an invalid candidate");
    }
  }
  for (int64_t z : snapshot.obstacle_column_cells) {
    if (z < snapshot.origin[2] ||
        z >= snapshot.origin[2] + snapshot.dims[2]) {
      return fail("obstacle CSR references an out-of-bounds cell");
    }
  }
  for (const auto& candidate : snapshot.candidates) {
    if (candidate.patch_id < 0 ||
        static_cast<size_t>(candidate.patch_id) >= snapshot.patches.size()) {
      return fail("candidate patch id is out of range");
    }
    if (!finiteScalar(candidate.last_hit_stamp)) {
      return fail("candidate hit stamp must be finite");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (!finiteScalar(candidate.point[axis]) ||
          !finiteScalar(candidate.local_coefficients[axis])) {
        return fail("candidate geometry must be finite");
      }
    }
    if (!finiteScalar(candidate.local_confidence)) {
      return fail("candidate confidence must be finite");
    }
  }
  for (uint8_t bit : snapshot.observed_column_bits) {
    if (bit > 1U) {
      return fail("observed column bit must be 0 or 1");
    }
  }
  const dddmr_local_map::VehicleGeometryExport& geometry = snapshot.geometry;
  if (!finiteScalar(geometry.base_contact_height) ||
      geometry.base_contact_height <= 0.0 ||
      !finiteScalar(geometry.body_clearance) ||
      geometry.body_clearance < 0.0) {
    return fail("vehicle geometry scalars are invalid");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!finiteScalar(geometry.body_min[axis]) ||
        !finiteScalar(geometry.body_max[axis]) ||
        geometry.body_max[axis] <= geometry.body_min[axis]) {
      return fail("body bounds are invalid");
    }
  }
  return true;
}

bool buildDeviceSnapshot(const TerrainSnapshotExport& snapshot,
                         DeviceSnapshot* device, std::string* error) {
  TerrainDeviceView& view = device->view;
  view = TerrainDeviceView{};
  view.valid = 0;
  view.geometry_valid = 0;
  view.config_valid = 0;
  view.snapshot_version = snapshot.snapshot_version;
  view.snapshot_stamp = snapshot.snapshot_stamp;
  view.voxel_size = snapshot.voxel_size;
  for (int axis = 0; axis < 3; ++axis) {
    view.origin[axis] = snapshot.origin[axis];
    view.dims[axis] = snapshot.dims[axis];
  }
  view.occupied_threshold = snapshot.occupied_threshold;
  view.free_threshold = snapshot.free_threshold;
  view.column_count = snapshot.columnCount();
  view.cell_count = snapshot.cells.size();
  view.candidate_count = snapshot.candidates.size();
  view.patch_count = snapshot.patches.size();
  view.candidate_column_count = snapshot.candidate_column_indices.size();
  view.obstacle_column_count = snapshot.obstacle_column_cells.size();

  const auto& terrain = snapshot.terrain_config;
  view.config.contact_patch_radius = terrain.contact_patch_radius;
  view.config.max_contact_search_z = terrain.max_contact_search_z;
  view.config.max_drop = terrain.max_drop;
  view.config.max_step = terrain.max_step;
  view.config.max_slope_rad = terrain.max_slope_rad;
  view.config.max_cross_slope_rad = terrain.max_cross_slope_rad;
  view.config.max_warp = terrain.max_warp;
  view.config.min_patch_coverage = terrain.min_patch_coverage;
  view.config.min_plane_confidence = terrain.min_plane_confidence;
  view.config.max_snapshot_age = terrain.max_snapshot_age;
  view.config.max_observation_age = terrain.max_observation_age;
  view.config.obstacle_search_margin = terrain.obstacle_search_margin;
  view.config.cluster_max_dz = terrain.cluster_max_dz;
  view.config.plane_inlier_tol = terrain.plane_inlier_tol;
  view.config.max_patches = terrain.max_patches;
  view.config.unknown_body_policy = terrain.unknown_body_policy;

  const auto& geometry = snapshot.geometry;
  for (int wheel = 0; wheel < 4; ++wheel) {
    for (int axis = 0; axis < 3; ++axis) {
      view.geometry.wheel_contacts_base[wheel][axis] =
          geometry.wheel_contacts_base[wheel][axis];
    }
  }
  view.geometry.base_contact_height = geometry.base_contact_height;
  for (int axis = 0; axis < 3; ++axis) {
    view.geometry.body_min[axis] = geometry.body_min[axis];
    view.geometry.body_max[axis] = geometry.body_max[axis];
  }
  view.geometry.body_clearance = geometry.body_clearance;

  if (!uploadArray(snapshot.cells, *device, 0, &view.cells, error) ||
      !uploadArray(snapshot.candidates, *device, 1, &view.candidates, error) ||
      !uploadArray(snapshot.patches, *device, 2, &view.patches, error) ||
      !uploadArray(snapshot.candidate_column_start, *device, 3,
                   &view.candidate_column_start, error) ||
      !uploadArray(snapshot.candidate_column_indices, *device, 4,
                   &view.candidate_column_indices, error) ||
      !uploadArray(snapshot.obstacle_column_start, *device, 5,
                   &view.obstacle_column_start, error) ||
      !uploadArray(snapshot.obstacle_column_cells, *device, 6,
                   &view.obstacle_column_cells, error) ||
      !uploadArray(snapshot.observed_column_bits, *device, 7,
                   &view.observed_column_bits, error)) {
    return false;
  }
  view.valid = 1;
  view.geometry_valid = 1;
  view.config_valid = 1;
  return true;
}

}  // namespace

const char* terrainDeviceDetailText(TerrainDeviceDetail detail) {
  using D = TerrainDeviceDetail;
  switch (detail) {
    case D::kNone:
      return "";
    case D::kInvalidGeometry:
      return "invalid vehicle geometry";
    case D::kInvalidConfig:
      return "invalid terrain config";
    case D::kBuildFailure:
      return "terrain model build failed";
    case D::kInvalidQuery:
      return "query pose/stamp must be finite";
    case D::kSnapshotStampInvalid:
      return "snapshot has no valid map timestamp";
    case D::kSnapshotAge:
      return "snapshot age exceeds max_snapshot_age";
    case D::kBaseOutside:
      return "predicted base position is outside the map window";
    case D::kWheelPatchOutside:
      return "a wheel contact patch is outside the map window";
    case D::kProjectedWheelPatchOutside:
      return "a projected wheel contact patch is outside the map window";
    case D::kBodyAabbOutside:
      return "full body OBB extends outside the map window; it is not clipped";
    case D::kObstacleSearchOutside:
      return "bounded obstacle-clearance search volume leaves the map window";
    case D::kBodySearchOutside:
      return "body collision search left the map window";
    case D::kWheelColumnUnobserved:
      return "contact column is unobserved";
    case D::kWheelStale:
      return "support observation is older than max_observation_age";
    case D::kWheelCoverage:
      return "fresh contact patch coverage is insufficient";
    case D::kWheelLowConfidence:
      return "local plane confidence is insufficient";
    case D::kBracketedLayers:
      return "predicted layer is bracketed by disconnected surfaces";
    case D::kDropoff:
      return "support exists but is below max_drop";
    case D::kNoSupportNearPredicted:
      return "no support plane is near the predicted height";
    case D::kNoCandidateUnderWheel:
      return "no surface candidate under a wheel";
    case D::kWheelPathStale:
      return "wheel-path support layer is stale";
    case D::kWheelPathMissing:
      return "wheel-path support is missing (narrow dip or unobserved gap)";
    case D::kWheelPathStep:
      return "wheel-path surface height jumps across adjacent columns";
    case D::kWheelPlaneRankDeficient:
      return "four-wheel support samples are rank deficient";
    case D::kDegeneratePlane:
      return "degenerate four-wheel support plane";
    case D::kWheelStep:
      return "wheel support heights are non-planar (step)";
    case D::kWarp:
      return "wheel support plane residual exceeds max_warp";
    case D::kSlope:
      return "support slope exceeds max_slope_rad";
    case D::kCrossSlope:
      return "cross slope exceeds max_cross_slope_rad";
    case D::kProjectedStale:
      return "projected wheel support observation is stale";
    case D::kProjectedUnobserved:
      return "projected wheel contact column is unobserved";
    case D::kProjectedCoverage:
      return "projected wheel fresh contact patch coverage is insufficient";
    case D::kProjectedLowConfidence:
      return "projected wheel local plane confidence is insufficient";
    case D::kProjectedDropoff:
      return "projected support is below max_drop";
    case D::kProjectedNoSupport:
      return "projected wheel contact has no cached support";
    case D::kProjectedRankDeficient:
      return "projected contact supports are rank deficient";
    case D::kNotConverged:
      return "projected contact supports did not converge within 3 iterations";
    case D::kContactOffSupport:
      return "projected contact does not lie on cached fresh support";
    case D::kBodyClearance:
      return "body clearance is below configured body_clearance";
    case D::kBodyCollision:
      return "rigid body OBB intersects a raw occupied voxel volume";
    case D::kBodyStale:
      return "body volume contains observations older than max_observation_age";
    case D::kBodyUnknown:
      return "body OBB intersects unobserved voxel volumes and policy is reject";
  }
  return "terrain query failed";
}

const char* terrainDeviceStatusText(TerrainDeviceStatus status) {
  return dddmr_local_map::toString(
      static_cast<dddmr_local_map::TerrainStatus>(status));
}

bool terrainResultFromDevice(const TerrainDeviceResult& device,
                             dddmr_local_map::TerrainResult* out,
                             std::string* error) {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "terrain result output must not be null";
    }
    return false;
  }
  if (device.status >
      static_cast<uint8_t>(TerrainDeviceStatus::kBuildFailure)) {
    if (error != nullptr) {
      *error = "device terrain result has an unknown status";
    }
    return false;
  }
  dddmr_local_map::TerrainResult result;
  result.status =
      static_cast<dddmr_local_map::TerrainStatus>(device.status);
  result.detail = terrainDeviceDetailText(
      static_cast<TerrainDeviceDetail>(device.detail));
  result.projected_position =
      dddmr_local_map::Vec3(device.projected_position[0],
                            device.projected_position[1],
                            device.projected_position[2]);
  result.projected_orientation =
      dddmr_local_map::Quat(device.projected_orientation[3],
                            device.projected_orientation[0],
                            device.projected_orientation[1],
                            device.projected_orientation[2]);
  result.roll = device.roll;
  result.pitch = device.pitch;
  result.yaw = device.yaw;
  result.clearance = device.clearance;
  result.obstacle_clearance = device.obstacle_clearance;
  result.selected_patch = device.selected_patch;
  for (int wheel = 0; wheel < 4; ++wheel) {
    result.wheels[wheel].supported = device.wheels[wheel].supported != 0;
    result.wheels[wheel].predicted_z = device.wheels[wheel].predicted_z;
    result.wheels[wheel].support_z = device.wheels[wheel].support_z;
    result.wheels[wheel].coverage = device.wheels[wheel].coverage;
    result.wheels[wheel].patch_id = device.wheels[wheel].patch_id;
  }
  *out = std::move(result);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

dddmr_local_map::TerrainResult terrainResultFromDevice(
    const TerrainDeviceResult& device) {
  dddmr_local_map::TerrainResult result;
  std::string error;
  if (!terrainResultFromDevice(device, &result, &error)) {
    result = dddmr_local_map::TerrainResult{};
    result.status = dddmr_local_map::TerrainStatus::kBuildFailure;
    result.detail = error;
  }
  return result;
}

struct MppiCudaTerrain::Impl {
  bool cuda_available{false};
  std::string device_name;
  // Double buffering: uploads build the staging snapshot, then swap it with the
  // active snapshot only after every array copied successfully. A failed
  // upload leaves the active view and all of its arrays untouched. The old
  // active buffer becomes staging and its capacities are reused next time.
  std::unique_ptr<DeviceSnapshot> active;
  std::unique_ptr<DeviceSnapshot> staging;
  TerrainDeviceQuery* queries{nullptr};
  size_t query_capacity{0};
  TerrainDeviceResult* results{nullptr};
  size_t result_capacity{0};

  ~Impl() {
    if (queries != nullptr) {
      cudaFree(queries);
    }
    if (results != nullptr) {
      cudaFree(results);
    }
  }

  bool reserveQueries(size_t count) {
    if (count <= query_capacity) {
      return true;
    }
    if (count > std::numeric_limits<size_t>::max() /
                    sizeof(TerrainDeviceQuery)) {
      return false;
    }
    TerrainDeviceQuery* replacement = nullptr;
    const cudaError_t status =
        cudaMalloc(&replacement, count * sizeof(TerrainDeviceQuery));
    if (status != cudaSuccess) {
      return false;
    }
    if (queries != nullptr) {
      cudaFree(queries);
    }
    queries = replacement;
    query_capacity = count;
    return true;
  }

  bool reserveResults(size_t count) {
    if (count <= result_capacity) {
      return true;
    }
    if (count > std::numeric_limits<size_t>::max() /
                    sizeof(TerrainDeviceResult)) {
      return false;
    }
    TerrainDeviceResult* replacement = nullptr;
    const cudaError_t status =
        cudaMalloc(&replacement, count * sizeof(TerrainDeviceResult));
    if (status != cudaSuccess) {
      return false;
    }
    if (results != nullptr) {
      cudaFree(results);
    }
    results = replacement;
    result_capacity = count;
    return true;
  }
};

MppiCudaTerrain::MppiCudaTerrain() : impl_(std::make_unique<Impl>()) {
  int device_count = 0;
  const cudaError_t status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess || device_count <= 0) {
    // Leave the runtime error latched as an explicit unavailable state.
    cudaGetLastError();
    return;
  }
  cudaDeviceProp properties{};
  if (cudaGetDeviceProperties(&properties, 0) != cudaSuccess) {
    cudaGetLastError();
    return;
  }
  impl_->cuda_available = true;
  impl_->device_name = properties.name;
}

MppiCudaTerrain::~MppiCudaTerrain() = default;
MppiCudaTerrain::MppiCudaTerrain(MppiCudaTerrain&&) noexcept = default;
MppiCudaTerrain& MppiCudaTerrain::operator=(MppiCudaTerrain&&) noexcept =
    default;

bool MppiCudaTerrain::available() const { return impl_->cuda_available; }

bool MppiCudaTerrain::upload(const TerrainSnapshotExport& snapshot,
                             std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (!impl_->cuda_available) {
    if (error != nullptr) {
      *error = "CUDA terrain backend is unavailable; no CPU fallback exists";
    }
    return false;
  }
  if (!validateExport(snapshot, error)) {
    return false;
  }
  if (!impl_->staging) {
    impl_->staging = std::make_unique<DeviceSnapshot>();
  }
  if (!buildDeviceSnapshot(snapshot, impl_->staging.get(), error)) {
    return false;
  }
  // Publish only after every array copied; the old generation stays intact on
  // failure. The previous active buffer becomes staging for the next upload.
  std::swap(impl_->active, impl_->staging);
  return true;
}

bool MppiCudaTerrain::evaluateBatch(
    const std::vector<dddmr_local_map::TerrainQuery>& queries,
    std::vector<dddmr_local_map::TerrainResult>* results,
    std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (results == nullptr) {
    if (error != nullptr) {
      *error = "terrain result vector must not be null";
    }
    return false;
  }
  if (!impl_->cuda_available) {
    if (error != nullptr) {
      *error = "CUDA terrain backend is unavailable; no CPU fallback exists";
    }
    return false;
  }
  if (!impl_->active) {
    if (error != nullptr) {
      *error = "no terrain snapshot has been uploaded";
    }
    return false;
  }
  if (queries.empty()) {
    results->clear();
    return true;
  }
  const size_t count = queries.size();
  if (!impl_->reserveQueries(count) || !impl_->reserveResults(count)) {
    if (error != nullptr) {
      *error = "failed to grow CUDA terrain scratch buffers";
    }
    return false;
  }
  std::vector<TerrainDeviceQuery> host_queries(count);
  for (size_t index = 0; index < count; ++index) {
    const auto& source = queries[index];
    TerrainDeviceQuery& target = host_queries[index];
    target.stamp = source.stamp;
    target.position[0] = source.position.x();
    target.position[1] = source.position.y();
    target.position[2] = source.position.z();
    target.roll = source.roll;
    target.pitch = source.pitch;
    target.yaw = source.yaw;
  }
  cudaError_t status =
      cudaMemcpy(impl_->queries, host_queries.data(),
                 count * sizeof(TerrainDeviceQuery), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    if (error != nullptr) {
      *error = cudaErrorText("cudaMemcpy(queries)", status);
    }
    return false;
  }
  const int threads = 256;
  const size_t blocks = (count + threads - 1U) / threads;
  evaluateTerrainKernel<<<static_cast<unsigned int>(blocks), threads>>>(
      impl_->active->view, impl_->queries, impl_->results, count);
  status = cudaGetLastError();
  if (status != cudaSuccess) {
    if (error != nullptr) {
      *error = cudaErrorText("evaluateTerrainKernel", status);
    }
    return false;
  }
  std::vector<TerrainDeviceResult> host_results(count);
  status = cudaMemcpy(host_results.data(), impl_->results,
                      count * sizeof(TerrainDeviceResult),
                      cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    if (error != nullptr) {
      *error = cudaErrorText("cudaMemcpy(results)", status);
    }
    return false;
  }
  results->resize(count);
  for (size_t index = 0; index < count; ++index) {
    if (!terrainResultFromDevice(host_results[index], &(*results)[index],
                                 error)) {
      return false;
    }
  }
  return true;
}

bool MppiCudaTerrain::evaluateBatch(
    const std::vector<dddmr_local_map::TerrainQuery>& queries,
    std::vector<dddmr_local_map::TerrainResult>& results,
    std::string* error) {
  return evaluateBatch(queries, &results, error);
}

const TerrainDeviceView& MppiCudaTerrain::deviceView() const {
  static const TerrainDeviceView empty{};
  return impl_->active ? impl_->active->view : empty;
}

bool MppiCudaTerrain::hasSnapshot() const {
  return impl_->active != nullptr;
}

uint64_t MppiCudaTerrain::snapshotVersion() const {
  return impl_->active ? impl_->active->view.snapshot_version : 0U;
}

double MppiCudaTerrain::snapshotStamp() const {
  return impl_->active ? impl_->active->view.snapshot_stamp
                       : std::numeric_limits<double>::quiet_NaN();
}

std::unique_ptr<MppiCudaTerrain> makeMppiCudaTerrain() {
  return std::make_unique<MppiCudaTerrain>();
}

}  // namespace local_planner
