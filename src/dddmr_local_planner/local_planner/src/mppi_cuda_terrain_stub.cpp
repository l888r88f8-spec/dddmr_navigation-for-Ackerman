#include "local_planner/mppi_cuda_terrain.h"

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "dddmr_local_map/terrain.hpp"

namespace local_planner {
namespace {

const char* kUnavailable =
    "CUDA terrain backend is unavailable (built without CUDA); no CPU "
    "fallback exists";

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

struct MppiCudaTerrain::Impl {};

MppiCudaTerrain::MppiCudaTerrain() : impl_(std::make_unique<Impl>()) {}
MppiCudaTerrain::~MppiCudaTerrain() = default;
MppiCudaTerrain::MppiCudaTerrain(MppiCudaTerrain&&) noexcept = default;
MppiCudaTerrain& MppiCudaTerrain::operator=(MppiCudaTerrain&&) noexcept =
    default;

bool MppiCudaTerrain::available() const { return false; }

bool MppiCudaTerrain::upload(const dddmr_local_map::TerrainSnapshotExport&,
                             std::string* error) {
  if (error != nullptr) {
    *error = kUnavailable;
  }
  return false;
}

bool MppiCudaTerrain::evaluateBatch(
    const std::vector<dddmr_local_map::TerrainQuery>&,
    std::vector<dddmr_local_map::TerrainResult>*, std::string* error) {
  if (error != nullptr) {
    *error = kUnavailable;
  }
  return false;
}

bool MppiCudaTerrain::evaluateBatch(
    const std::vector<dddmr_local_map::TerrainQuery>&,
    std::vector<dddmr_local_map::TerrainResult>&, std::string* error) {
  if (error != nullptr) {
    *error = kUnavailable;
  }
  return false;
}

const TerrainDeviceView& MppiCudaTerrain::deviceView() const {
  static const TerrainDeviceView empty{};
  return empty;
}

bool MppiCudaTerrain::hasSnapshot() const { return false; }

uint64_t MppiCudaTerrain::snapshotVersion() const { return 0U; }

double MppiCudaTerrain::snapshotStamp() const {
  return std::numeric_limits<double>::quiet_NaN();
}

std::unique_ptr<MppiCudaTerrain> makeMppiCudaTerrain() {
  return std::make_unique<MppiCudaTerrain>();
}

}  // namespace local_planner
