// GPU local-map rollout: one CUDA thread per candidate, time steps advanced in
// order, with every terrain query evaluated inline on the device. The numeric
// path mirrors src/mppi_local_map_rollout.cpp and keeps all geometry in double.

#include "local_planner/mppi_cuda_local_map_rollout.h"

#include <cuda_runtime.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "local_planner/mppi_cuda_terrain_math.cuh"
#include "local_planner/mppi_cuda_launch.h"

namespace local_planner {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfiniteClearance = std::numeric_limits<double>::infinity();

// Device-only detail phase for a non-finite failure. Terrain failures use
// terrain_detail instead; this distinguishes the CPU rollout's different
// non-finite messages.
enum DeviceFailure : uint8_t {
  kFailureNone = 0,
  kInitialMapNonFinite,
  kInitialProjection,
  kInitialPlanner,
  kPredictedPosition,
  kStepProjection,
  kStepPlanner,
};

struct DeviceInput {
  int64_t control_offset{0};
  int32_t control_count{0};
  int32_t reserved{0};
  double dt{0.1};
  double initial_pose[16]{1.0, 0.0, 0.0, 0.0,
                          0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0, 1.0};
};

struct DeviceControl {
  double vx{0.0};
  double wz{0.0};
};

struct DeviceOutput {
  uint8_t status{static_cast<uint8_t>(MppiLocalMapRolloutStatus::kOk)};
  uint8_t terrain_status{
      static_cast<uint8_t>(dddmr_local_map::TerrainStatus::kOk)};
  uint8_t preserve_prefix{0};
  uint8_t failure{static_cast<uint8_t>(DeviceFailure::kFailureNone)};
  uint8_t terrain_detail{static_cast<uint8_t>(TerrainDeviceDetail::kNone)};
  uint8_t reserved[3]{0, 0, 0};
  int32_t failed_step{-1};
  int32_t evaluated_steps{0};
  int32_t reserved2{0};
  double projected_start[16]{1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0};
};

struct DMat3 {
  double m[9]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

struct DMat4 {
  double m[16]{1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0};
};

__device__ inline bool finite(double value) { return isfinite(value); }

__device__ inline double clampDouble(double value, double lower,
                                     double upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

__device__ DMat3 rotationFromRpy(double roll, double pitch, double yaw) {
  const double sr = sin(roll);
  const double cr = cos(roll);
  const double sp = sin(pitch);
  const double cp = cos(pitch);
  const double sy = sin(yaw);
  const double cy = cos(yaw);
  DMat3 rotation;
  rotation.m[0] = cy * cp;
  rotation.m[1] = cy * sp * sr - sy * cr;
  rotation.m[2] = cy * sp * cr + sy * sr;
  rotation.m[3] = sy * cp;
  rotation.m[4] = sy * sp * sr + cy * cr;
  rotation.m[5] = sy * sp * cr - cy * sr;
  rotation.m[6] = -sp;
  rotation.m[7] = cp * sr;
  rotation.m[8] = cp * cr;
  return rotation;
}

__device__ void rpyFromRotation(const DMat3& rotation, double* roll,
                                double* pitch, double* yaw) {
  *roll = atan2(rotation.m[7], rotation.m[8]);
  *pitch = asin(clampDouble(-rotation.m[6], -1.0, 1.0));
  *yaw = atan2(rotation.m[3], rotation.m[0]);
}

__device__ DMat3 quaternionToRotation(const double q[4]) {
  // q is Eigen coefficient order (x, y, z, w).
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  DMat3 rotation;
  rotation.m[0] = 1.0 - 2.0 * (yy + zz);
  rotation.m[1] = 2.0 * (xy - wz);
  rotation.m[2] = 2.0 * (xz + wy);
  rotation.m[3] = 2.0 * (xy + wz);
  rotation.m[4] = 1.0 - 2.0 * (xx + zz);
  rotation.m[5] = 2.0 * (yz - wx);
  rotation.m[6] = 2.0 * (xz - wy);
  rotation.m[7] = 2.0 * (yz + wx);
  rotation.m[8] = 1.0 - 2.0 * (xx + yy);
  return rotation;
}

__device__ DMat4 multiply(const DMat4& a, const DMat4& b) {
  DMat4 out;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      double value = 0.0;
      for (int k = 0; k < 4; ++k) {
        value += a.m[row * 4 + k] * b.m[k * 4 + col];
      }
      out.m[row * 4 + col] = value;
    }
  }
  return out;
}

__device__ bool finiteMatrix(const double m[16]) {
  for (int index = 0; index < 16; ++index) {
    if (!finite(m[index])) {
      return false;
    }
  }
  return true;
}

__device__ bool properRigid(const double m[16]) {
  if (!finiteMatrix(m)) {
    return false;
  }
  // R^T R == I and det(R) == +1, matching the CPU rigidPose() tolerance.
  double max_error = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int k = 0; k < 3; ++k) {
        value += m[k * 4 + row] * m[k * 4 + col];
      }
      const double target = (row == col) ? 1.0 : 0.0;
      max_error = fmax(max_error, fabs(value - target));
    }
  }
  if (max_error > 1e-6) {
    return false;
  }
  const double determinant =
      m[0] * (m[5] * m[10] - m[6] * m[9]) -
      m[1] * (m[4] * m[10] - m[6] * m[8]) +
      m[2] * (m[4] * m[9] - m[5] * m[8]);
  if (fabs(determinant - 1.0) > 1e-6) {
    return false;
  }
  return fabs(m[12]) <= 1e-9 && fabs(m[13]) <= 1e-9 && fabs(m[14]) <= 1e-9 &&
         fabs(m[15] - 1.0) <= 1e-9;
}

__device__ bool poseFromTerrain(const TerrainDeviceResult& terrain,
                                DMat4* pose) {
  for (int axis = 0; axis < 3; ++axis) {
    if (!finite(terrain.projected_position[axis])) {
      return false;
    }
  }
  for (int index = 0; index < 4; ++index) {
    if (!finite(terrain.projected_orientation[index])) {
      return false;
    }
  }
  const double norm = sqrt(
      terrain.projected_orientation[0] * terrain.projected_orientation[0] +
      terrain.projected_orientation[1] * terrain.projected_orientation[1] +
      terrain.projected_orientation[2] * terrain.projected_orientation[2] +
      terrain.projected_orientation[3] * terrain.projected_orientation[3]);
  if (!finite(norm) || norm < 1e-9) {
    return false;
  }
  double q[4];
  for (int index = 0; index < 4; ++index) {
    q[index] = terrain.projected_orientation[index] / norm;
  }
  const DMat3 rotation = quaternionToRotation(q);
  DMat4 projected;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      projected.m[row * 4 + col] = rotation.m[row * 3 + col];
    }
    projected.m[row * 4 + 3] = terrain.projected_position[row];
  }
  if (!finiteMatrix(projected.m)) {
    return false;
  }
  *pose = projected;
  return true;
}

__device__ MppiLocalMapRolloutStatus statusForTerrainFailure(
    TerrainDeviceStatus status, bool initial_pose) {
  if (status == TerrainDeviceStatus::kStale) {
    return MppiLocalMapRolloutStatus::kStale;
  }
  if (status == TerrainDeviceStatus::kOutside) {
    return MppiLocalMapRolloutStatus::kOutside;
  }
  return initial_pose ? MppiLocalMapRolloutStatus::kInitialPoseRejected
                      : MppiLocalMapRolloutStatus::kTerrainRejected;
}

__device__ void writeRejected(DeviceOutput* output,
                              MppiLocalMapRolloutStatus status,
                              DeviceFailure failure, int step) {
  output->status = static_cast<uint8_t>(status);
  output->terrain_status =
      static_cast<uint8_t>(dddmr_local_map::TerrainStatus::kOk);
  output->terrain_detail = static_cast<uint8_t>(TerrainDeviceDetail::kNone);
  output->failure = static_cast<uint8_t>(failure);
  output->failed_step = step;
  output->preserve_prefix = 0;
  output->evaluated_steps = 0;
}

__global__ void rolloutBatchKernel(TerrainDeviceView view, DMat4 planner_to_map,
                                   DMat4 map_to_planner, double query_stamp,
                                   const DeviceInput* inputs,
                                   const DeviceControl* controls,
                                   double* poses, double* clearances,
                                   DeviceOutput* outputs, size_t candidate_count,
                                   int32_t stride) {
  const size_t candidate =
      static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (candidate >= candidate_count) {
    return;
  }
  const DeviceInput& input = inputs[candidate];
  DeviceOutput& output = outputs[candidate];
  output = DeviceOutput{};

  if (!finite(input.dt) || input.dt <= 0.0) {
    writeRejected(&output, MppiLocalMapRolloutStatus::kInvalidConfig,
                  DeviceFailure::kFailureNone, -1);
    return;
  }
  if (input.control_count == 0) {
    writeRejected(&output, MppiLocalMapRolloutStatus::kEmptyControls,
                  DeviceFailure::kFailureNone, -1);
    return;
  }

  DMat4 initial_planner;
  for (int index = 0; index < 16; ++index) {
    initial_planner.m[index] = input.initial_pose[index];
  }
  if (!properRigid(initial_planner.m)) {
    writeRejected(&output,
                  MppiLocalMapRolloutStatus::kInvalidInitialPose,
                  DeviceFailure::kFailureNone, -1);
    return;
  }
  for (int32_t index = 0; index < input.control_count; ++index) {
    const DeviceControl& control = controls[input.control_offset + index];
    if (!finite(control.vx) || !finite(control.wz)) {
      writeRejected(&output, MppiLocalMapRolloutStatus::kInvalidControl,
                    DeviceFailure::kFailureNone, index);
      return;
    }
  }

  const DMat4 initial_map = multiply(planner_to_map, initial_planner);
  if (!finiteMatrix(initial_map.m)) {
    writeRejected(&output,
                  MppiLocalMapRolloutStatus::kInvalidInitialPose,
                  DeviceFailure::kInitialMapNonFinite, -1);
    return;
  }

  TerrainDeviceQuery query{};
  query.stamp = query_stamp;
  query.position[0] = initial_map.m[3];
  query.position[1] = initial_map.m[7];
  query.position[2] = initial_map.m[11];
  DMat3 initial_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      initial_rotation.m[row * 3 + col] = initial_map.m[row * 4 + col];
    }
  }
  rpyFromRotation(initial_rotation, &query.roll, &query.pitch, &query.yaw);
  const TerrainDeviceResult initial_terrain = cuda_terrain::evaluate(view, query);
  if (initial_terrain.status !=
      static_cast<uint8_t>(TerrainDeviceStatus::kOk)) {
    writeRejected(&output,
                  statusForTerrainFailure(
                      static_cast<TerrainDeviceStatus>(initial_terrain.status),
                      true),
                  DeviceFailure::kFailureNone, -1);
    output.terrain_status = initial_terrain.status;
    output.terrain_detail = initial_terrain.detail;
    return;
  }

  DMat4 projected_map;
  if (!poseFromTerrain(initial_terrain, &projected_map)) {
    writeRejected(&output, MppiLocalMapRolloutStatus::kNonFiniteResult,
                  DeviceFailure::kInitialProjection, -1);
    return;
  }
  const DMat4 projected_start_planner =
      multiply(map_to_planner, projected_map);
  if (!finiteMatrix(projected_start_planner.m)) {
    writeRejected(&output, MppiLocalMapRolloutStatus::kNonFiniteResult,
                  DeviceFailure::kInitialPlanner, -1);
    return;
  }
  for (int index = 0; index < 16; ++index) {
    output.projected_start[index] = projected_start_planner.m[index];
  }

  double previous_roll = 0.0;
  double previous_pitch = 0.0;
  double previous_yaw = 0.0;
  DMat3 projected_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      projected_rotation.m[row * 3 + col] = projected_map.m[row * 4 + col];
    }
  }
  rpyFromRotation(projected_rotation, &previous_roll, &previous_pitch,
                  &previous_yaw);

  for (int32_t step = 0; step < input.control_count; ++step) {
    const DeviceControl& control = controls[input.control_offset + step];
    const double midpoint_yaw = previous_yaw + 0.5 * control.wz * input.dt;
    const DMat3 midpoint_rotation =
        rotationFromRpy(previous_roll, previous_pitch, midpoint_yaw);
    const double predicted_x =
        projected_map.m[3] + control.vx * input.dt * midpoint_rotation.m[0];
    const double predicted_y =
        projected_map.m[7] + control.vx * input.dt * midpoint_rotation.m[3];
    const double predicted_z =
        projected_map.m[11] + control.vx * input.dt * midpoint_rotation.m[6];
    if (!finite(predicted_x) || !finite(predicted_y) || !finite(predicted_z)) {
      writeRejected(&output, MppiLocalMapRolloutStatus::kNonFiniteResult,
                    DeviceFailure::kPredictedPosition, step);
      return;
    }

    const double predicted_yaw = previous_yaw + control.wz * input.dt;
    const DMat3 predicted_rotation =
        rotationFromRpy(previous_roll, previous_pitch, predicted_yaw);
    query.position[0] = predicted_x;
    query.position[1] = predicted_y;
    query.position[2] = predicted_z;
    rpyFromRotation(predicted_rotation, &query.roll, &query.pitch, &query.yaw);
    const TerrainDeviceResult terrain = cuda_terrain::evaluate(view, query);
    if (terrain.status !=
        static_cast<uint8_t>(TerrainDeviceStatus::kOk)) {
      output.status = static_cast<uint8_t>(statusForTerrainFailure(
          static_cast<TerrainDeviceStatus>(terrain.status), false));
      output.terrain_status = terrain.status;
      output.terrain_detail = terrain.detail;
      output.failure = static_cast<uint8_t>(DeviceFailure::kFailureNone);
      output.failed_step = step;
      output.preserve_prefix = 1;
      output.evaluated_steps = step;
      return;
    }

    DMat4 next_map;
    if (!poseFromTerrain(terrain, &next_map)) {
      writeRejected(&output, MppiLocalMapRolloutStatus::kNonFiniteResult,
                    DeviceFailure::kStepProjection, step);
      return;
    }
    const DMat4 next_planner = multiply(map_to_planner, next_map);
    if (!finiteMatrix(next_planner.m)) {
      writeRejected(&output, MppiLocalMapRolloutStatus::kNonFiniteResult,
                    DeviceFailure::kStepPlanner, step);
      return;
    }

    const size_t base =
        (candidate * static_cast<size_t>(stride) +
         static_cast<size_t>(step)) *
        16U;
    for (int index = 0; index < 16; ++index) {
      poses[base + static_cast<size_t>(index)] = next_planner.m[index];
    }
    clearances[candidate * static_cast<size_t>(stride) +
               static_cast<size_t>(step)] = terrain.obstacle_clearance;
    output.evaluated_steps = step + 1;

    projected_map = next_map;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        projected_rotation.m[row * 3 + col] =
            projected_map.m[row * 4 + col];
      }
    }
    rpyFromRotation(projected_rotation, &previous_roll, &previous_pitch,
                    &previous_yaw);
  }

  output.status = static_cast<uint8_t>(MppiLocalMapRolloutStatus::kOk);
  output.terrain_status =
      static_cast<uint8_t>(dddmr_local_map::TerrainStatus::kOk);
  output.terrain_detail = static_cast<uint8_t>(TerrainDeviceDetail::kNone);
  output.failure = static_cast<uint8_t>(DeviceFailure::kFailureNone);
  output.failed_step = -1;
  output.preserve_prefix = 1;
  output.evaluated_steps = input.control_count;
}

bool finiteScalar(double value) { return std::isfinite(value); }

bool finitePose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite();
}

bool rigidPose(const Eigen::Isometry3d& pose) {
  if (!finitePose(pose)) {
    return false;
  }
  const Eigen::Matrix3d rotation = pose.linear();
  if (!(rotation.transpose() * rotation)
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    return false;
  }
  if (std::abs(rotation.determinant() - 1.0) > 1e-6) {
    return false;
  }
  const Eigen::Matrix4d matrix = pose.matrix();
  return std::abs(matrix(3, 0)) <= 1e-9 &&
         std::abs(matrix(3, 1)) <= 1e-9 &&
         std::abs(matrix(3, 2)) <= 1e-9 &&
         std::abs(matrix(3, 3) - 1.0) <= 1e-9;
}

void rpyFromRotation(const Eigen::Matrix3d& rotation, double* roll,
                     double* pitch, double* yaw) {
  *roll = std::atan2(rotation(2, 1), rotation(2, 2));
  *pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  *yaw = std::atan2(rotation(1, 0), rotation(0, 0));
}

double yawFromRotation(const Eigen::Matrix3d& rotation) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rpyFromRotation(rotation, &roll, &pitch, &yaw);
  return yaw;
}

void copyMatrix(const Eigen::Isometry3d& pose, double out[16]) {
  const Eigen::Matrix4d matrix = pose.matrix();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      out[row * 4 + col] = matrix(row, col);
    }
  }
}

Eigen::Isometry3d poseFromMatrix(const double in[16]) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = in[row * 4 + col];
    }
  }
  pose.matrix() = matrix;
  return pose;
}

std::string cudaErrorText(const char* operation, cudaError_t error) {
  std::ostringstream stream;
  stream << operation << " failed: " << cudaGetErrorString(error);
  return stream.str();
}

std::string terrainFailureDetail(MppiLocalMapRolloutStatus status,
                                 dddmr_local_map::TerrainStatus terrain_status,
                                 TerrainDeviceDetail terrain_detail, int step,
                                 bool initial_pose) {
  std::ostringstream stream;
  if (initial_pose) {
    stream << "initial pose rejected";
  } else {
    stream << "terrain rejected at step " << step;
  }
  stream << ": " << dddmr_local_map::toString(terrain_status);
  const char* detail = terrainDeviceDetailText(terrain_detail);
  if (detail != nullptr && detail[0] != '\0') {
    stream << ": " << detail;
  }
  (void)status;
  return stream.str();
}

std::string nonFiniteDetail(DeviceFailure failure, int step) {
  std::ostringstream stream;
  switch (failure) {
    case DeviceFailure::kInitialMapNonFinite:
      return "initial pose in map frame is not finite";
    case DeviceFailure::kInitialProjection:
      return "initial terrain projection is not finite";
    case DeviceFailure::kInitialPlanner:
      return "projected start pose in planner frame is not finite";
    case DeviceFailure::kPredictedPosition:
      stream << "predicted position at step " << step << " is not finite";
      return stream.str();
    case DeviceFailure::kStepProjection:
      stream << "terrain projection at step " << step << " is not finite";
      return stream.str();
    case DeviceFailure::kStepPlanner:
      stream << "projected pose at step " << step
             << " is not finite in planner frame";
      return stream.str();
    case DeviceFailure::kFailureNone:
      break;
  }
  return "non-finite rollout result";
}

MppiLocalMapRolloutResult baseRejected(
    const MppiLocalMapContext& context, MppiLocalMapRolloutStatus status,
    const std::string& detail) {
  MppiLocalMapRolloutResult result;
  result.valid = false;
  result.status = status;
  result.detail = detail;
  result.map_version = context.mapVersion();
  result.map_stamp = context.mapStamp();
  return result;
}

// Mirrors the CPU pre-terrain validation order exactly. Returns true only when
// the candidate reaches the device terrain loop.
bool validateContext(const MppiLocalMapContext& context,
                     MppiLocalMapRolloutStatus* status, std::string* detail) {
  if (context.snapshot == nullptr || context.terrain == nullptr) {
    *status = MppiLocalMapRolloutStatus::kMissingMap;
    *detail = "snapshot and terrain bundle is required";
    return false;
  }
  if (!finiteScalar(context.snapshot->stamp())) {
    *status = MppiLocalMapRolloutStatus::kInvalidContext;
    *detail = "snapshot stamp is not finite";
    return false;
  }
  if (!finiteScalar(context.query_stamp)) {
    *status = MppiLocalMapRolloutStatus::kInvalidContext;
    *detail = "query_stamp is not finite";
    return false;
  }
  if (!rigidPose(context.planner_to_map)) {
    *status = MppiLocalMapRolloutStatus::kInvalidTransform;
    *detail = "planner_to_map is not a proper rigid transform";
    return false;
  }
  if (!finitePose(context.planner_to_map.inverse())) {
    *status = MppiLocalMapRolloutStatus::kInvalidTransform;
    *detail = "planner_to_map inverse is not finite";
    return false;
  }
  return true;
}

template <typename T>
bool reserveDevice(T** pointer, size_t* capacity, size_t count,
                   const char* operation, std::string* error) {
  if (count <= *capacity) {
    return true;
  }
  T* replacement = nullptr;
  const cudaError_t status = cudaMalloc(&replacement, count * sizeof(T));
  if (status != cudaSuccess) {
    *error = cudaErrorText(operation, status);
    return false;
  }
  if (*pointer != nullptr) {
    cudaFree(*pointer);
  }
  *pointer = replacement;
  *capacity = count;
  return true;
}

}  // namespace

struct MppiCudaLocalMapRollout::Impl {
  bool cuda_available{false};
  std::string device_name;
  int sm_count{1};

  DeviceInput* inputs{nullptr};
  size_t input_capacity{0};
  DeviceControl* controls{nullptr};
  size_t control_capacity{0};
  DeviceOutput* outputs{nullptr};
  size_t output_capacity{0};
  double* poses{nullptr};
  size_t pose_capacity{0};
  double* clearances{nullptr};
  size_t clearance_capacity{0};

  ~Impl() {
    if (inputs != nullptr) {
      cudaFree(inputs);
    }
    if (controls != nullptr) {
      cudaFree(controls);
    }
    if (outputs != nullptr) {
      cudaFree(outputs);
    }
    if (poses != nullptr) {
      cudaFree(poses);
    }
    if (clearances != nullptr) {
      cudaFree(clearances);
    }
  }
};

MppiCudaLocalMapRollout::MppiCudaLocalMapRollout()
    : impl_(std::make_unique<Impl>()) {
  int device_count = 0;
  const cudaError_t status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess || device_count <= 0) {
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
  impl_->sm_count = std::max(1, properties.multiProcessorCount);
}

MppiCudaLocalMapRollout::~MppiCudaLocalMapRollout() = default;
MppiCudaLocalMapRollout::MppiCudaLocalMapRollout(
    MppiCudaLocalMapRollout&&) noexcept = default;
MppiCudaLocalMapRollout& MppiCudaLocalMapRollout::operator=(
    MppiCudaLocalMapRollout&&) noexcept = default;

bool MppiCudaLocalMapRollout::available(std::string* reason) const {
  if (reason != nullptr) {
    if (impl_->cuda_available) {
      *reason = impl_->device_name + " (" +
                std::to_string(impl_->sm_count) + " SMs)";
    } else {
      *reason =
          "CUDA local-map rollout backend is unavailable; no CPU fallback "
          "exists";
    }
  }
  return impl_->cuda_available;
}

bool MppiCudaLocalMapRollout::rolloutBatch(
    const MppiLocalMapContext& context,
    const MppiCudaTerrain& uploaded_terrain,
    const std::vector<MppiLocalMapRolloutInput>& inputs,
    std::vector<MppiLocalMapRolloutResult>& outputs, std::string& error) {
  error.clear();
  outputs.clear();
  if (inputs.empty()) {
    return true;
  }
  if (!impl_->cuda_available) {
    error =
        "CUDA local-map rollout backend is unavailable; no CPU fallback "
        "exists";
    return false;
  }

  MppiLocalMapRolloutStatus context_status = MppiLocalMapRolloutStatus::kOk;
  std::string context_detail;
  if (!validateContext(context, &context_status, &context_detail)) {
    outputs.reserve(inputs.size());
    for (size_t index = 0; index < inputs.size(); ++index) {
      outputs.push_back(
          baseRejected(context, context_status, context_detail));
    }
    return true;
  }

  const TerrainDeviceView view = uploaded_terrain.deviceView();
  const auto origin = context.snapshot->originVoxel();
  const auto dims = context.snapshot->windowDims();
  bool bounds_match =
      view.voxel_size == context.snapshot->config().voxel_size;
  for(int axis = 0; axis < 3; ++axis){
    bounds_match = bounds_match && view.origin[axis] == origin[axis] &&
        view.dims[axis] == dims[axis];
  }
  if (view.valid == 0 || view.abi_version != kTerrainDeviceAbiVersion ||
      view.snapshot_version != context.mapVersion() ||
      view.snapshot_stamp != context.mapStamp() || !bounds_match) {
    std::ostringstream stream;
    stream << "uploaded CUDA terrain snapshot does not match the pinned "
              "local-map context (device version "
           << view.snapshot_version << " stamp " << view.snapshot_stamp
           << ", context version " << context.mapVersion() << " stamp "
           << context.mapStamp() << "; map bounds match=" << bounds_match << ")";
    error = stream.str();
    return false;
  }

  size_t max_steps = 0;
  size_t total_controls = 0;
  for (const auto& input : inputs) {
    max_steps = std::max(max_steps, input.controls.size());
    total_controls += input.controls.size();
  }
  const size_t candidate_count = inputs.size();
  if (candidate_count >
          std::numeric_limits<unsigned int>::max() / 1U ||
      total_controls > static_cast<size_t>(std::numeric_limits<int64_t>::max()) ||
      max_steps >
          static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
    error = "local-map rollout batch is too large for CUDA indexing";
    return false;
  }

  const size_t pose_values = candidate_count * max_steps * 16U;
  const size_t clearance_values = candidate_count * max_steps;
  if (!reserveDevice(&impl_->inputs, &impl_->input_capacity, candidate_count,
                     "cudaMalloc(rollout_inputs)", &error) ||
      !reserveDevice(&impl_->controls, &impl_->control_capacity,
                     std::max<size_t>(total_controls, 1U),
                     "cudaMalloc(rollout_controls)", &error) ||
      !reserveDevice(&impl_->outputs, &impl_->output_capacity,
                     candidate_count, "cudaMalloc(rollout_outputs)", &error) ||
      !reserveDevice(&impl_->poses, &impl_->pose_capacity,
                     std::max<size_t>(pose_values, 1U),
                     "cudaMalloc(rollout_poses)", &error) ||
      !reserveDevice(&impl_->clearances, &impl_->clearance_capacity,
                     std::max<size_t>(clearance_values, 1U),
                     "cudaMalloc(rollout_clearances)", &error)) {
    return false;
  }

  std::vector<DeviceInput> host_inputs(candidate_count);
  std::vector<DeviceControl> host_controls(total_controls);
  size_t control_offset = 0;
  for (size_t index = 0; index < candidate_count; ++index) {
    const MppiLocalMapRolloutInput& source = inputs[index];
    DeviceInput& target = host_inputs[index];
    target.control_offset = static_cast<int64_t>(control_offset);
    target.control_count = static_cast<int32_t>(source.controls.size());
    target.dt = source.dt;
    copyMatrix(source.initial_pose_planner, target.initial_pose);
    for (const MppiControl& control : source.controls) {
      host_controls[control_offset] = DeviceControl{control.vx, control.wz};
      ++control_offset;
    }
  }

  cudaError_t status = cudaMemcpy(
      impl_->inputs, host_inputs.data(),
      candidate_count * sizeof(DeviceInput), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    error = cudaErrorText("cudaMemcpy(rollout_inputs)", status);
    return false;
  }
  if (total_controls > 0) {
    status = cudaMemcpy(impl_->controls, host_controls.data(),
                        total_controls * sizeof(DeviceControl),
                        cudaMemcpyHostToDevice);
    if (status != cudaSuccess) {
      error = cudaErrorText("cudaMemcpy(rollout_controls)", status);
      return false;
    }
  }

  DMat4 planner_to_map;
  DMat4 map_to_planner;
  copyMatrix(context.planner_to_map, planner_to_map.m);
  copyMatrix(context.planner_to_map.inverse(), map_to_planner.m);
  const MppiCudaLaunchGeometry launch =
      mppiCudaSelectLaunch(candidate_count, impl_->sm_count);
  const int32_t stride = static_cast<int32_t>(max_steps);
  rolloutBatchKernel<<<launch.grid, launch.block_dim>>>(
      view, planner_to_map, map_to_planner, context.query_stamp,
      impl_->inputs, impl_->controls, impl_->poses, impl_->clearances,
      impl_->outputs, candidate_count, stride);
  status = cudaGetLastError();
  if (status != cudaSuccess) {
    error = cudaErrorText("rolloutBatchKernel", status);
    return false;
  }

  std::vector<DeviceOutput> host_outputs(candidate_count);
  status = cudaMemcpy(host_outputs.data(), impl_->outputs,
                      candidate_count * sizeof(DeviceOutput),
                      cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    error = cudaErrorText("cudaMemcpy(rollout_outputs)", status);
    return false;
  }
  std::vector<double> host_poses(pose_values);
  std::vector<double> host_clearances(clearance_values);
  if (pose_values > 0) {
    status = cudaMemcpy(host_poses.data(), impl_->poses,
                        pose_values * sizeof(double), cudaMemcpyDeviceToHost);
    if (status != cudaSuccess) {
      error = cudaErrorText("cudaMemcpy(rollout_poses)", status);
      return false;
    }
  }
  if (clearance_values > 0) {
    status = cudaMemcpy(host_clearances.data(), impl_->clearances,
                        clearance_values * sizeof(double),
                        cudaMemcpyDeviceToHost);
    if (status != cudaSuccess) {
      error = cudaErrorText("cudaMemcpy(rollout_clearances)", status);
      return false;
    }
  }

  outputs.reserve(candidate_count);
  for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
    const DeviceOutput& device = host_outputs[candidate];
    const MppiLocalMapRolloutInput& source = inputs[candidate];
    MppiLocalMapRolloutResult result;
    result.valid = false;
    result.status =
        static_cast<MppiLocalMapRolloutStatus>(device.status);
    result.terrain_status =
        static_cast<dddmr_local_map::TerrainStatus>(device.terrain_status);
    result.map_version = context.mapVersion();
    result.map_stamp = context.mapStamp();
    result.evaluated_steps =
        std::max<int32_t>(0, device.evaluated_steps);
    if (device.preserve_prefix != 0) {
      result.projected_start_pose =
          poseFromMatrix(device.projected_start);
    }

    if (result.status == MppiLocalMapRolloutStatus::kOk) {
      result.valid = true;
      result.detail.clear();
      result.projected_poses.reserve(source.controls.size());
      result.trajectory.controls.reserve(source.controls.size());
      result.trajectory.points.reserve(source.controls.size());
    } else if (device.preserve_prefix != 0) {
      result.projected_poses.reserve(result.evaluated_steps);
      result.trajectory.controls.reserve(result.evaluated_steps);
      result.trajectory.points.reserve(result.evaluated_steps);
    }

    const size_t prefix_steps =
        result.valid ? source.controls.size()
                     : (device.preserve_prefix != 0
                            ? static_cast<size_t>(result.evaluated_steps)
                            : 0U);
    for (size_t step = 0; step < prefix_steps; ++step) {
      const size_t base =
          (candidate * max_steps + step) * 16U;
      Eigen::Isometry3d pose =
          poseFromMatrix(&host_poses[base]);
      MppiTrajectoryPoint point;
      point.x = pose.translation().x();
      point.y = pose.translation().y();
      point.yaw = yawFromRotation(pose.linear());
      point.in_collision = false;
      point.obstacle_clearance =
          host_clearances[candidate * max_steps + step];
      if (std::isnan(point.obstacle_clearance) ||
          point.obstacle_clearance < 0.0) {
        point.obstacle_clearance = kInfiniteClearance;
      }
      point.collision_evaluated = true;
      result.projected_poses.push_back(pose);
      result.trajectory.controls.push_back(source.controls[step]);
      result.trajectory.points.push_back(point);
    }

    if (result.status == MppiLocalMapRolloutStatus::kOk) {
      result.evaluated_steps = source.controls.size();
    } else if (device.preserve_prefix == 0) {
      result.projected_start_pose = Eigen::Isometry3d::Identity();
      result.evaluated_steps = 0;
    }

    if (result.status == MppiLocalMapRolloutStatus::kOk) {
      result.detail.clear();
    } else if (result.status == MppiLocalMapRolloutStatus::kTerrainRejected ||
               result.status ==
                   MppiLocalMapRolloutStatus::kInitialPoseRejected ||
               result.status == MppiLocalMapRolloutStatus::kStale ||
               result.status == MppiLocalMapRolloutStatus::kOutside) {
      const bool initial_pose = device.failed_step < 0;
      result.detail = terrainFailureDetail(
          result.status, result.terrain_status,
          static_cast<TerrainDeviceDetail>(device.terrain_detail),
          device.failed_step, initial_pose);
    } else if (result.status ==
               MppiLocalMapRolloutStatus::kInvalidControl) {
      std::ostringstream stream;
      stream << "control " << device.failed_step << " is not finite";
      result.detail = stream.str();
    } else if (result.status ==
               MppiLocalMapRolloutStatus::kNonFiniteResult) {
      result.detail = nonFiniteDetail(
          static_cast<DeviceFailure>(device.failure), device.failed_step);
    } else if (result.status ==
               MppiLocalMapRolloutStatus::kInvalidInitialPose) {
      result.detail =
          (device.failure ==
           static_cast<uint8_t>(DeviceFailure::kInitialMapNonFinite))
              ? "initial pose in map frame is not finite"
              : "initial planner pose is not a proper rigid transform";
    } else if (result.status ==
               MppiLocalMapRolloutStatus::kInvalidConfig) {
      result.detail = "dt must be finite and positive";
    } else if (result.status ==
               MppiLocalMapRolloutStatus::kEmptyControls) {
      result.detail = "controls must not be empty";
    } else {
      result.detail = "local-map rollout was rejected";
    }
    outputs.push_back(std::move(result));
  }
  return true;
}

std::unique_ptr<MppiCudaLocalMapRollout> makeMppiCudaLocalMapRollout() {
  return std::make_unique<MppiCudaLocalMapRollout>();
}

}  // namespace local_planner
