#pragma once

// Device-side exact reproduction of dddmr_local_map::TerrainModel::evaluate().
//
// Every numeric field is double and the code paths mirror terrain.cpp helper by
// helper: circular-cell lookup, fresh hit/miss evidence, candidate-local planes,
// 3x3 footprint coverage, raw-surface wheel-path continuity, four-wheel plane
// iteration, raw-voxel OBB SAT collision, exact OBB/AABB distance, floor
// exclusion, unknown-body policy and bounded search volumes. No grid
// approximation, sphere distance or host evaluate() call is used.
//
// Include this from a rollout kernel translation unit. The view must be the one
// produced by MppiCudaTerrain::upload() for the current map generation.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "local_planner/mppi_cuda_terrain.h"

namespace local_planner {
namespace cuda_terrain {

__device__ inline bool finiteScalar(double value) { return isfinite(value); }

template <typename T>
__device__ inline T maxValue(T a, T b) {
  return a > b ? a : b;
}
template <typename T>
__device__ inline T minValue(T a, T b) {
  return a < b ? a : b;
}
__device__ inline double infinityD() {
  return __longlong_as_double(0x7ff0000000000000ULL);
}
__device__ inline double quietNaND() {
  return __longlong_as_double(0x7ff8000000000001ULL);
}

__device__ inline double clampDouble(double value, double lower,
                                     double upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

__device__ inline Vec3 makeVec3(double x, double y, double z) {
  Vec3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

__device__ inline Vec3 add(const Vec3& a, const Vec3& b) {
  return makeVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__device__ inline Vec3 sub(const Vec3& a, const Vec3& b) {
  return makeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}
__device__ inline Vec3 mul(const Vec3& a, double s) {
  return makeVec3(a.x * s, a.y * s, a.z * s);
}
__device__ inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
__device__ inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return makeVec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}
__device__ inline double norm(const Vec3& a) {
  return std::sqrt(dot(a, a));
}
__device__ inline Vec3 normalized(const Vec3& a) {
  const double length = norm(a);
  return length > 0.0 ? mul(a, 1.0 / length) : a;
}

struct Mat3 {
  double m[3][3]{};
};

__device__ inline Vec3 mul(const Mat3& r, const Vec3& v) {
  return makeVec3(r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
                  r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
                  r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z);
}
__device__ inline Vec3 transposeMul(const Mat3& r, const Vec3& v) {
  return makeVec3(r.m[0][0] * v.x + r.m[1][0] * v.y + r.m[2][0] * v.z,
                  r.m[0][1] * v.x + r.m[1][1] * v.y + r.m[2][1] * v.z,
                  r.m[0][2] * v.x + r.m[1][2] * v.y + r.m[2][2] * v.z);
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll), with Eigen's matrix convention.
__device__ inline Mat3 rotationFromRpy(double roll, double pitch,
                                       double yaw) {
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  Mat3 r;
  r.m[0][0] = cy * cp;
  r.m[0][1] = cy * sp * sr - sy * cr;
  r.m[0][2] = cy * sp * cr + sy * sr;
  r.m[1][0] = sy * cp;
  r.m[1][1] = sy * sp * sr + cy * cr;
  r.m[1][2] = sy * sp * cr - cy * sr;
  r.m[2][0] = -sp;
  r.m[2][1] = cp * sr;
  r.m[2][2] = cp * cr;
  return r;
}

// Eigen's Quaternion(rotation matrix) conversion (Shoemake). Returns x,y,z,w.
__device__ inline void quaternionFromRotation(const Mat3& a, double q[4]) {
  const double t = a.m[0][0] + a.m[1][1] + a.m[2][2];
  if (t > 0.0) {
    double s = std::sqrt(t + 1.0);
    q[3] = 0.5 * s;
    s = 0.5 / s;
    q[0] = (a.m[2][1] - a.m[1][2]) * s;
    q[1] = (a.m[0][2] - a.m[2][0]) * s;
    q[2] = (a.m[1][0] - a.m[0][1]) * s;
  } else {
    int i = 0;
    if (a.m[1][1] > a.m[0][0]) {
      i = 1;
    }
    if (a.m[2][2] > a.m[i][i]) {
      i = 2;
    }
    const int j = (i + 1) % 3;
    const int k = (j + 1) % 3;
    double s = std::sqrt(a.m[i][i] - a.m[j][j] - a.m[k][k] + 1.0);
    q[i] = 0.5 * s;
    s = 0.5 / s;
    q[3] = (a.m[k][j] - a.m[j][k]) * s;
    q[j] = (a.m[j][i] + a.m[i][j]) * s;
    q[k] = (a.m[k][i] + a.m[i][k]) * s;
  }
  const double length =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (length > 0.0) {
    const double inv = 1.0 / length;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
}

__device__ inline Vec3 normalFromCoefficients(const Vec3& c) {
  return normalized(makeVec3(-c.x, -c.y, 1.0));
}

__device__ inline void orientationFromNormalYaw(const Vec3& normal, double yaw,
                                                Mat3* rotation, double q[4]) {
  Vec3 heading = makeVec3(std::cos(yaw), std::sin(yaw), 0.0);
  Vec3 forward = sub(heading, mul(normal, dot(heading, normal)));
  if (norm(forward) < 1e-9) {
    heading = makeVec3(-std::sin(yaw), std::cos(yaw), 0.0);
    forward = sub(heading, mul(normal, dot(heading, normal)));
  }
  if (norm(forward) < 1e-9) {
    *rotation = Mat3{};
    rotation->m[0][0] = 1.0;
    rotation->m[1][1] = 1.0;
    rotation->m[2][2] = 1.0;
    q[0] = q[1] = q[2] = 0.0;
    q[3] = 1.0;
    return;
  }
  forward = normalized(forward);
  Vec3 left = cross(normal, forward);
  if (norm(left) < 1e-9) {
    *rotation = Mat3{};
    rotation->m[0][0] = 1.0;
    rotation->m[1][1] = 1.0;
    rotation->m[2][2] = 1.0;
    q[0] = q[1] = q[2] = 0.0;
    q[3] = 1.0;
    return;
  }
  left = normalized(left);
  const Vec3 up = normalized(cross(forward, left));
  rotation->m[0][0] = forward.x;
  rotation->m[1][0] = forward.y;
  rotation->m[2][0] = forward.z;
  rotation->m[0][1] = left.x;
  rotation->m[1][1] = left.y;
  rotation->m[2][1] = left.z;
  rotation->m[0][2] = up.x;
  rotation->m[1][2] = up.y;
  rotation->m[2][2] = up.z;
  quaternionFromRotation(*rotation, q);
}

__device__ inline Vec3 basePositionOnPlane(const Vec3& c, double x, double y,
                                           double base_height) {
  const Vec3 normal = normalFromCoefficients(c);
  const double length =
      std::sqrt(c.x * c.x + c.y * c.y + 1.0);
  const double plane_d = c.z / length;
  return makeVec3(
      x, y,
      (plane_d + base_height - normal.x * x - normal.y * y) / normal.z);
}

__device__ inline int64_t floorMod(int64_t value, int64_t modulus) {
  int64_t result = value % modulus;
  if (result < 0) {
    result += modulus;
  }
  return result;
}

// ---------------------------------------------------------------------------
// Raw voxel lookup (MapSnapshot::lookupVoxel(..., include_inflation=false)).
// ---------------------------------------------------------------------------

struct CellView {
  bool in_bounds{false};
  bool observed{false};
  uint8_t occupancy{0};  // 0 unknown, 1 free, 2 occupied
  double last_observed_stamp{quietNaND()};
  double last_hit_stamp{quietNaND()};
  double last_miss_stamp{quietNaND()};
  double hit_x{quietNaND()};
  double hit_y{quietNaND()};
  double hit_z{quietNaND()};
  double hit_max_z{quietNaND()};
};

__device__ inline bool inBoundsVoxel(const TerrainDeviceView& view, int64_t x,
                                     int64_t y, int64_t z) {
  return x >= view.origin[0] && x < view.origin[0] + view.dims[0] &&
         y >= view.origin[1] && y < view.origin[1] + view.dims[1] &&
         z >= view.origin[2] && z < view.origin[2] + view.dims[2];
}

// Decode a physical (already wrapped) cell index. Callers that iterate a dense
// voxel range hoist the index arithmetic and use this directly, so the hot
// body loop performs no per-cell division while preserving lookupVoxel's
// exact circular-index semantics.
__device__ inline CellView decodePhysicalCell(const TerrainDeviceView& view,
                                              size_t index) {
  CellView out;
  out.in_bounds = true;
  const dddmr_local_map::MapCellExport& cell = view.cells[index];
  out.observed = cell.observed != 0;
  if (out.observed) {
    out.last_observed_stamp = cell.last_observed_stamp;
    out.last_hit_stamp = cell.last_hit_stamp;
    out.last_miss_stamp = cell.last_miss_stamp;
    if (finiteScalar(cell.hit_x) && finiteScalar(cell.hit_y) &&
        finiteScalar(cell.hit_z)) {
      out.hit_x = static_cast<double>(cell.hit_x);
      out.hit_y = static_cast<double>(cell.hit_y);
      out.hit_z = static_cast<double>(cell.hit_z);
      out.hit_max_z = static_cast<double>(cell.hit_max_z);
    }
  }
  if (!out.observed) {
    out.occupancy = 0;
  } else if (cell.log_odds >= view.occupied_threshold) {
    out.occupancy = 2;
  } else if (cell.log_odds <= view.free_threshold) {
    out.occupancy = 1;
  } else {
    out.occupancy = 0;
  }
  return out;
}

__device__ inline CellView lookupVoxel(const TerrainDeviceView& view, int64_t x,
                                       int64_t y, int64_t z) {
  CellView out;
  if (!inBoundsVoxel(view, x, y, z)) {
    return out;
  }
  const int64_t rx = floorMod(x, view.dims[0]);
  const int64_t ry = floorMod(y, view.dims[1]);
  const int64_t rz = floorMod(z, view.dims[2]);
  const size_t index = static_cast<size_t>(
      (rx * static_cast<int64_t>(view.dims[1]) + ry) *
          static_cast<int64_t>(view.dims[2]) +
      rz);
  return decodePhysicalCell(view, index);
}

// ---------------------------------------------------------------------------
// Support queries.
// ---------------------------------------------------------------------------

__device__ inline bool freshObservation(const TerrainDeviceView& view,
                                        double observation,
                                        double query_stamp) {
  return finiteScalar(observation) &&
         std::abs(query_stamp - observation) <=
             view.config.max_observation_age + 1e-9;
}

__device__ inline bool validActualHit(const TerrainDeviceView& view,
                                      double hit_stamp, double miss_stamp,
                                      double query_stamp) {
  if (!freshObservation(view, hit_stamp, query_stamp)) {
    return false;
  }
  return !(finiteScalar(miss_stamp) && miss_stamp > hit_stamp + 1e-9);
}

__device__ inline bool validSupportObservation(
    const TerrainDeviceView& view,
    const dddmr_local_map::TerrainCandidateExport& candidate,
    double query_stamp) {
  return validActualHit(view, candidate.last_hit_stamp,
                        candidate.last_miss_stamp, query_stamp);
}

__device__ inline bool validLookupHit(const TerrainDeviceView& view,
                                      const CellView& lookup,
                                      double query_stamp) {
  return validActualHit(view, lookup.last_hit_stamp, lookup.last_miss_stamp,
                        query_stamp);
}

__device__ inline double candidateHeight(
    const dddmr_local_map::TerrainCandidateExport& candidate, double x,
    double y) {
  return candidate.local_coefficients[0] * x +
         candidate.local_coefficients[1] * y +
         candidate.local_coefficients[2];
}

__device__ inline bool columnFor(const TerrainDeviceView& view, double x,
                                 double y, int64_t* ix, int64_t* iy,
                                 size_t* column) {
  if (!finiteScalar(x) || !finiteScalar(y)) {
    return false;
  }
  const double voxel = view.voxel_size;
  const int64_t vx = static_cast<int64_t>(std::floor(x / voxel));
  const int64_t vy = static_cast<int64_t>(std::floor(y / voxel));
  if (vx < view.origin[0] || vx >= view.origin[0] + view.dims[0] ||
      vy < view.origin[1] || vy >= view.origin[1] + view.dims[1]) {
    return false;
  }
  const size_t index =
      static_cast<size_t>(vy - view.origin[1]) *
          static_cast<size_t>(view.dims[0]) +
      static_cast<size_t>(vx - view.origin[0]);
  if (index + 1U >= view.column_count + 1U || view.column_count == 0U) {
    return false;
  }
  *ix = vx;
  *iy = vy;
  *column = index;
  return index < view.column_count;
}

__device__ inline bool columnViewIndex(const TerrainDeviceView& view, int64_t x,
                                       int64_t y, size_t* column) {
  if (view.candidate_column_start == nullptr || view.column_count == 0U) {
    return false;
  }
  const int64_t lx = x - view.origin[0];
  const int64_t ly = y - view.origin[1];
  if (lx < 0 || ly < 0 || lx >= view.dims[0] || ly >= view.dims[1]) {
    return false;
  }
  const size_t index = static_cast<size_t>(ly) *
                           static_cast<size_t>(view.dims[0]) +
                       static_cast<size_t>(lx);
  if (index >= view.column_count) {
    return false;
  }
  *column = index;
  return true;
}

__device__ inline double footprintCoverage(const TerrainDeviceView& view,
                                           int candidate_index, double x,
                                           double y, double query_stamp) {
  if (candidate_index < 0 ||
      static_cast<size_t>(candidate_index) >= view.candidate_count) {
    return 0.0;
  }
  const dddmr_local_map::TerrainCandidateExport& selected =
      view.candidates[candidate_index];
  const int patch_id = selected.patch_id;
  const double tolerance =
      maxValue(2.0 * view.voxel_size, 2.0 * view.config.plane_inlier_tol);
  int supported = 0;
  int total = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      ++total;
      const double px =
          x + static_cast<double>(dx) * view.config.contact_patch_radius;
      const double py =
          y + static_cast<double>(dy) * view.config.contact_patch_radius;
      int64_t ix = 0;
      int64_t iy = 0;
      size_t column = 0;
      if (!columnFor(view, px, py, &ix, &iy, &column)) {
        continue;
      }
      bool found = false;
      for (uint64_t position = view.candidate_column_start[column];
           position < view.candidate_column_start[column + 1U]; ++position) {
        const int index = view.candidate_column_indices[position];
        const dddmr_local_map::TerrainCandidateExport& candidate =
            view.candidates[index];
        if (candidate.patch_id != patch_id) {
          continue;
        }
        if (!validSupportObservation(view, candidate, query_stamp)) {
          continue;
        }
        if (std::abs(candidate.point[2] - candidateHeight(selected, px, py)) <=
            tolerance) {
          found = true;
          break;
        }
      }
      if (found) {
        ++supported;
      }
    }
  }
  return static_cast<double>(supported) / static_cast<double>(total);
}

struct SupportHit {
  bool found{false};
  bool saw_candidate{false};
  bool stale{false};
  bool unknown_column{false};
  bool coverage_failure{false};
  bool confidence_failure{false};
  bool lower_surface{false};
  bool upper_surface{false};
  int candidate_index{-1};
  int patch_id{-1};
  double height{0.0};
  double coverage{0.0};
  double confidence{0.0};
};

__device__ inline SupportHit supportAt(const TerrainDeviceView& view, double x,
                                       double y, double near_z,
                                       double query_stamp,
                                       bool require_coverage,
                                       bool require_confidence) {
  SupportHit hit;
  int64_t ix = 0;
  int64_t iy = 0;
  size_t column = 0;
  if (!columnFor(view, x, y, &ix, &iy, &column)) {
    return hit;
  }
  const uint64_t candidate_begin = view.candidate_column_start[column];
  const uint64_t candidate_end = view.candidate_column_start[column + 1U];
  if (candidate_begin == candidate_end) {
    hit.unknown_column = view.observed_column_bits[column] == 0U;
    return hit;
  }
  hit.saw_candidate = true;
  double best_distance = infinityD();
  for (uint64_t position = candidate_begin; position < candidate_end;
       ++position) {
    const int index = view.candidate_column_indices[position];
    const dddmr_local_map::TerrainCandidateExport& candidate =
        view.candidates[index];
    const double plane_z = candidateHeight(candidate, x, y);
    if (plane_z < near_z - view.config.max_drop) {
      hit.lower_surface = true;
    }
    if (plane_z > near_z + view.config.max_drop) {
      hit.upper_surface = true;
    }
    if (std::abs(plane_z - near_z) > view.config.max_contact_search_z) {
      continue;
    }
    if (!validSupportObservation(view, candidate, query_stamp)) {
      hit.stale = true;
      continue;
    }
    const double coverage =
        require_coverage
            ? footprintCoverage(view, index, x, y, query_stamp)
            : 1.0;
    if (require_coverage && coverage < view.config.min_patch_coverage) {
      hit.coverage_failure = true;
      continue;
    }
    if (candidate.local_confidence < view.config.min_plane_confidence) {
      hit.confidence_failure = true;
      if (require_confidence) {
        continue;
      }
    }
    const double distance = std::abs(plane_z - near_z);
    if (distance < best_distance) {
      best_distance = distance;
      hit.found = true;
      hit.candidate_index = index;
      hit.patch_id = candidate.patch_id;
      hit.height = plane_z;
      hit.coverage = coverage;
      hit.confidence = candidate.local_confidence;
    }
  }
  return hit;
}

struct RawSurface {
  bool found{false};
  bool stale{false};
  int64_t ix{0};
  int64_t iy{0};
  double z{0.0};
};

__device__ inline RawSurface rawSurfaceAt(const TerrainDeviceView& view,
                                          double x, double y, double near_z,
                                          double query_stamp) {
  RawSurface surface;
  int64_t ix = 0;
  int64_t iy = 0;
  size_t column = 0;
  if (!columnFor(view, x, y, &ix, &iy, &column)) {
    return surface;
  }
  surface.ix = ix;
  surface.iy = iy;
  double best_distance = infinityD();
  for (uint64_t position = view.candidate_column_start[column];
       position < view.candidate_column_start[column + 1U]; ++position) {
    const int index = view.candidate_column_indices[position];
    const dddmr_local_map::TerrainCandidateExport& candidate =
        view.candidates[index];
    const double distance = std::abs(candidate.point[2] - near_z);
    if (distance > view.config.max_contact_search_z) {
      continue;
    }
    if (!validSupportObservation(view, candidate, query_stamp)) {
      surface.stale = true;
      continue;
    }
    if (distance >= best_distance) {
      continue;
    }
    best_distance = distance;
    surface.found = true;
    surface.z = candidate.point[2];
  }
  return surface;
}

// Fixed 3x3 column-pivoted Householder QR least squares. This is the same
// mathematical model (and the same Eigen rank threshold formula) as the build
// and four-wheel planeFit, without an Eigen device dependency. It is used at
// runtime only for four wheel samples.
__device__ inline bool planeFit(const Vec3* points, int count, Vec3* coefficients,
                                double* rms, double* confidence) {
  if (count < 3) {
    return false;
  }
  constexpr int kMaxPoints = 4;
  double a[kMaxPoints][3]{};
  double b[kMaxPoints]{};
  for (int i = 0; i < count; ++i) {
    a[i][0] = points[i].x;
    a[i][1] = points[i].y;
    a[i][2] = 1.0;
    b[i] = points[i].z;
  }
  const double epsilon = 2.2204460492503131e-16;
  double column_norm[3];
  double max_norm = 0.0;
  for (int j = 0; j < 3; ++j) {
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
      sum += a[i][j] * a[i][j];
    }
    column_norm[j] = std::sqrt(sum);
    max_norm = maxValue(max_norm, column_norm[j]);
  }
  const double threshold_helper =
      (max_norm * epsilon) * (max_norm * epsilon) /
      static_cast<double>(count);
  int nonzero_pivots = 3;
  int permutation[3] = {0, 1, 2};
  double r[3][3]{};
  double qtb[kMaxPoints]{};
  for (int i = 0; i < count; ++i) {
    qtb[i] = b[i];
  }
  for (int k = 0; k < 3; ++k) {
    int best = k;
    double best_norm = -1.0;
    for (int j = k; j < 3; ++j) {
      double sum = 0.0;
      for (int i = k; i < count; ++i) {
        sum += a[i][j] * a[i][j];
      }
      if (sum > best_norm) {
        best_norm = sum;
        best = j;
      }
    }
    if (nonzero_pivots == 3 &&
        best_norm < threshold_helper * static_cast<double>(count - k)) {
      nonzero_pivots = k;
    }
    if (best != k) {
      for (int i = 0; i < count; ++i) {
        const double tmp = a[i][k];
        a[i][k] = a[i][best];
        a[i][best] = tmp;
      }
      const int tmp = permutation[k];
      permutation[k] = permutation[best];
      permutation[best] = tmp;
      const double tmp_norm = column_norm[k];
      column_norm[k] = column_norm[best];
      column_norm[best] = tmp_norm;
    }
    double norm_v = 0.0;
    for (int i = k; i < count; ++i) {
      norm_v += a[i][k] * a[i][k];
    }
    norm_v = std::sqrt(norm_v);
    if (norm_v == 0.0) {
      r[k][k] = 0.0;
      continue;
    }
    const double alpha = a[k][k] >= 0.0 ? -norm_v : norm_v;
    double v[kMaxPoints]{};
    double vnorm2 = 0.0;
    for (int i = k; i < count; ++i) {
      v[i] = a[i][k];
      if (i == k) {
        v[i] -= alpha;
      }
      vnorm2 += v[i] * v[i];
    }
    for (int j = k; j < 3; ++j) {
      double proj = 0.0;
      for (int i = k; i < count; ++i) {
        proj += v[i] * a[i][j];
      }
      const double factor = 2.0 * proj / vnorm2;
      for (int i = k; i < count; ++i) {
        a[i][j] -= factor * v[i];
      }
    }
    {
      double proj = 0.0;
      for (int i = k; i < count; ++i) {
        proj += v[i] * qtb[i];
      }
      const double factor = 2.0 * proj / vnorm2;
      for (int i = k; i < count; ++i) {
        qtb[i] -= factor * v[i];
      }
    }
    for (int j = 0; j < 3; ++j) {
      r[k][j] = a[k][j];
    }
    r[k][k] = alpha;
  }
  if (nonzero_pivots < 3) {
    return false;
  }
  // Solve the same least-squares normal equations with a fixed 3x3 pivoted
  // elimination. The QR above is retained only for Eigen-compatible rank
  // detection; for the four wheel contacts this is numerically safe and keeps
  // the exact model (z = ax + by + c).
  double ata[3][3]{};
  double atb[3]{};
  for (int i = 0; i < count; ++i) {
    const double row[3] = {points[i].x, points[i].y, 1.0};
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        ata[j][k] += row[j] * row[k];
      }
      atb[j] += row[j] * points[i].z;
    }
  }
  double augmented[3][4]{};
  double max_abs = 0.0;
  for (int j = 0; j < 3; ++j) {
    for (int k = 0; k < 3; ++k) {
      augmented[j][k] = ata[j][k];
      max_abs = maxValue(max_abs, std::abs(ata[j][k]));
    }
    augmented[j][3] = atb[j];
  }
  const double pivot_tolerance = epsilon * maxValue(1.0, max_abs) * 3.0;
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(augmented[row][col]) >
          std::abs(augmented[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(augmented[pivot][col]) <= pivot_tolerance) {
      return false;
    }
    if (pivot != col) {
      for (int k = col; k < 4; ++k) {
        const double tmp = augmented[col][k];
        augmented[col][k] = augmented[pivot][k];
        augmented[pivot][k] = tmp;
      }
    }
    const double diagonal = augmented[col][col];
    for (int row = col + 1; row < 3; ++row) {
      const double factor = augmented[row][col] / diagonal;
      for (int k = col; k < 4; ++k) {
        augmented[row][k] -= factor * augmented[col][k];
      }
    }
  }
  double solution[3]{};
  for (int row = 2; row >= 0; --row) {
    double sum = augmented[row][3];
    for (int col = row + 1; col < 3; ++col) {
      sum -= augmented[row][col] * solution[col];
    }
    solution[row] = sum / augmented[row][row];
  }
  if (!finiteScalar(solution[0]) || !finiteScalar(solution[1]) ||
      !finiteScalar(solution[2])) {
    return false;
  }
  coefficients->x = solution[0];
  coefficients->y = solution[1];
  coefficients->z = solution[2];
  double sse = 0.0;
  double mean_z = 0.0;
  for (int i = 0; i < count; ++i) {
    mean_z += points[i].z;
  }
  mean_z /= static_cast<double>(count);
  double sst = 0.0;
  for (int i = 0; i < count; ++i) {
    const double prediction = solution[0] * points[i].x +
                              solution[1] * points[i].y + solution[2];
    const double residual = points[i].z - prediction;
    sse += residual * residual;
    const double centered = points[i].z - mean_z;
    sst += centered * centered;
  }
  *rms = std::sqrt(sse / static_cast<double>(count));
  if (sst < 1e-12) {
    *confidence = (*rms < 1e-6) ? 1.0 : 0.0;
  } else {
    *confidence = clampDouble(1.0 - sse / sst, 0.0, 1.0);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Geometry.
// ---------------------------------------------------------------------------

static __device__ __noinline__ bool obbIntersectsAabb(const Vec3& obb_center,
                                         const Vec3& half_extents,
                                         const Mat3& rotation,
                                         const Vec3& aabb_min,
                                         const Vec3& aabb_max) {
  const Vec3 aabb_center = mul(add(aabb_min, aabb_max), 0.5);
  const Vec3 aabb_half = mul(sub(aabb_max, aabb_min), 0.5);
  const Vec3 translation = sub(aabb_center, obb_center);
  double abs_rotation[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_rotation[i][j] = std::abs(rotation.m[i][j]);
    }
  }
  const double half[3] = {half_extents.x, half_extents.y, half_extents.z};
  const double aabb_h[3] = {aabb_half.x, aabb_half.y, aabb_half.z};
  const double t[3] = {translation.x, translation.y, translation.z};
  for (int i = 0; i < 3; ++i) {
    double ra = 0.0;
    for (int j = 0; j < 3; ++j) {
      ra += half[j] * abs_rotation[i][j];
    }
    if (std::abs(t[i]) > aabb_h[i] + ra + 1e-12) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    const Vec3 column =
        makeVec3(rotation.m[0][i], rotation.m[1][i], rotation.m[2][i]);
    double rb = 0.0;
    for (int j = 0; j < 3; ++j) {
      rb += aabb_h[j] * std::abs(rotation.m[j][i]);
    }
    if (std::abs(dot(translation, column)) > half[i] + rb + 1e-12) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    const Vec3 column =
        makeVec3(rotation.m[0][i], rotation.m[1][i], rotation.m[2][i]);
    for (int j = 0; j < 3; ++j) {
      Vec3 unit = makeVec3(j == 0 ? 1.0 : 0.0, j == 1 ? 1.0 : 0.0,
                           j == 2 ? 1.0 : 0.0);
      Vec3 axis = cross(column, unit);
      const double length = norm(axis);
      if (length < 1e-12) {
        continue;
      }
      axis = mul(axis, 1.0 / length);
      double ra = 0.0;
      for (int k = 0; k < 3; ++k) {
        const Vec3 col_k =
            makeVec3(rotation.m[0][k], rotation.m[1][k], rotation.m[2][k]);
        ra += half[k] * std::abs(dot(col_k, axis));
      }
      double rb = 0.0;
      const double axis_array[3] = {axis.x, axis.y, axis.z};
      for (int k = 0; k < 3; ++k) {
        rb += aabb_h[k] * std::abs(axis_array[k]);
      }
      if (std::abs(dot(translation, axis)) > ra + rb + 1e-12) {
        return false;
      }
    }
  }
  return true;
}

__device__ inline double pointAabbDistance(const Vec3& point,
                                           const Vec3& aabb_min,
                                           const Vec3& aabb_max) {
  Vec3 delta = makeVec3(0.0, 0.0, 0.0);
  const double p[3] = {point.x, point.y, point.z};
  const double lo[3] = {aabb_min.x, aabb_min.y, aabb_min.z};
  const double hi[3] = {aabb_max.x, aabb_max.y, aabb_max.z};
  double d[3] = {0.0, 0.0, 0.0};
  for (int axis = 0; axis < 3; ++axis) {
    if (p[axis] < lo[axis]) {
      d[axis] = lo[axis] - p[axis];
    } else if (p[axis] > hi[axis]) {
      d[axis] = p[axis] - hi[axis];
    }
  }
  return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

__device__ inline double pointObbDistance(const Vec3& point,
                                          const Vec3& obb_center,
                                          const Vec3& half_extents,
                                          const Mat3& rotation) {
  const Vec3 local = transposeMul(rotation, sub(point, obb_center));
  double delta[3] = {
      maxValue(0.0, std::abs(local.x) - half_extents.x),
      maxValue(0.0, std::abs(local.y) - half_extents.y),
      maxValue(0.0, std::abs(local.z) - half_extents.z)};
  return std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] +
                   delta[2] * delta[2]);
}

__device__ inline double obbAabbDistance(const Vec3& obb_center,
                                         const Vec3& half_extents,
                                         const Mat3& rotation,
                                         const Vec3& aabb_min,
                                         const Vec3& aabb_max) {
  if (obbIntersectsAabb(obb_center, half_extents, rotation, aabb_min,
                        aabb_max)) {
    return 0.0;
  }
  double best = infinityD();
  const Vec3 aabb_center = mul(add(aabb_min, aabb_max), 0.5);
  const Vec3 aabb_half = mul(sub(aabb_max, aabb_min), 0.5);
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const Vec3 sign = makeVec3(static_cast<double>(sx),
                                   static_cast<double>(sy),
                                   static_cast<double>(sz));
        const Vec3 obb_corner =
            add(obb_center,
                mul(rotation, makeVec3(half_extents.x * sign.x,
                                       half_extents.y * sign.y,
                                       half_extents.z * sign.z)));
        best = minValue(best,
                        pointAabbDistance(obb_corner, aabb_min, aabb_max));
        const Vec3 aabb_corner =
            add(aabb_center, makeVec3(aabb_half.x * sign.x,
                                      aabb_half.y * sign.y,
                                      aabb_half.z * sign.z));
        best = minValue(best, pointObbDistance(aabb_corner, obb_center,
                                               half_extents, rotation));
      }
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Bounds.
// ---------------------------------------------------------------------------

__device__ inline bool snapshotInBounds(const TerrainDeviceView& view,
                                        const Vec3& point) {
  if (!finiteScalar(point.x) || !finiteScalar(point.y) ||
      !finiteScalar(point.z)) {
    return false;
  }
  const double voxel = view.voxel_size;
  const double lower[3] = {view.origin[0] * voxel, view.origin[1] * voxel,
                           view.origin[2] * voxel};
  const double upper[3] = {lower[0] + view.dims[0] * voxel,
                           lower[1] + view.dims[1] * voxel,
                           lower[2] + view.dims[2] * voxel};
  const double p[3] = {point.x, point.y, point.z};
  return p[0] >= lower[0] && p[0] < upper[0] && p[1] >= lower[1] &&
         p[1] < upper[1] && p[2] >= lower[2] && p[2] < upper[2];
}

__device__ inline bool patchInsideBounds(const TerrainDeviceView& view,
                                         const Vec3& point) {
  const double voxel = view.voxel_size;
  const double lower[3] = {view.origin[0] * voxel, view.origin[1] * voxel,
                           view.origin[2] * voxel};
  const double upper[3] = {lower[0] + view.dims[0] * voxel,
                           lower[1] + view.dims[1] * voxel,
                           lower[2] + view.dims[2] * voxel};
  const double radius = view.config.contact_patch_radius;
  return point.x - radius >= lower[0] && point.x + radius <= upper[0] &&
         point.y - radius >= lower[1] && point.y + radius <= upper[1];
}

// ---------------------------------------------------------------------------
// wheelPathContinuity.
// ---------------------------------------------------------------------------

__device__ inline bool wheelPathContinuity(
    const TerrainDeviceView& view, const Vec3 contacts[4],
    const Vec3 supports[4], double query_stamp, TerrainDeviceStatus* status,
    TerrainDeviceDetail* detail) {
  const int edges[4][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}};
  const double voxel = view.voxel_size;
  const double threshold = maxValue(view.config.max_step, voxel);
  for (int e = 0; e < 4; ++e) {
    const int a = edges[e][0];
    const int b = edges[e][1];
    const Vec3 start = makeVec3(contacts[a].x, contacts[a].y, 0.0);
    const Vec3 end = makeVec3(contacts[b].x, contacts[b].y, 0.0);
    const double z_start = supports[a].z;
    const double z_end = supports[b].z;
    const double length = norm(sub(end, start));
    if (!finiteScalar(length) || length < 1e-9) {
      continue;
    }
    const int steps =
        maxValue(2, static_cast<int>(std::ceil(length / (0.5 * voxel))));
    int64_t previous_x = 0;
    int64_t previous_y = 0;
    double previous_z = 0.0;
    bool have_previous = false;
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const Vec3 xy = add(start, mul(sub(end, start), t));
      const double expected_z = z_start + (z_end - z_start) * t;
      const RawSurface raw = rawSurfaceAt(view, xy.x, xy.y, expected_z,
                                          query_stamp);
      if (!raw.found) {
        if (raw.stale) {
          *status = TerrainDeviceStatus::kStale;
          *detail = TerrainDeviceDetail::kWheelPathStale;
        } else {
          *status = TerrainDeviceStatus::kNoSupport;
          *detail = TerrainDeviceDetail::kWheelPathMissing;
        }
        return false;
      }
      if (have_previous && raw.ix == previous_x && raw.iy == previous_y) {
        continue;
      }
      if (have_previous && std::abs(raw.z - previous_z) > threshold + 1e-9) {
        *status = TerrainDeviceStatus::kStep;
        *detail = TerrainDeviceDetail::kWheelPathStep;
        return false;
      }
      previous_x = raw.ix;
      previous_y = raw.iy;
      previous_z = raw.z;
      have_previous = true;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// evaluate().
// ---------------------------------------------------------------------------

__device__ inline TerrainDeviceResult reject(TerrainDeviceStatus status,
                                              TerrainDeviceDetail detail) {
  TerrainDeviceResult result;
  result.status = static_cast<uint8_t>(status);
  result.detail = static_cast<uint8_t>(detail);
  result.projected_position[0] = quietNaND();
  result.projected_position[1] = quietNaND();
  result.projected_position[2] = quietNaND();
  result.projected_orientation[0] = 0.0;
  result.projected_orientation[1] = 0.0;
  result.projected_orientation[2] = 0.0;
  result.projected_orientation[3] = 1.0;
  result.roll = quietNaND();
  result.pitch = quietNaND();
  result.yaw = quietNaND();
  result.clearance = quietNaND();
  result.obstacle_clearance = quietNaND();
  for (int wheel = 0; wheel < 4; ++wheel) {
    result.wheels[wheel].supported = 0;
    result.wheels[wheel].patch_id = -1;
    result.wheels[wheel].predicted_z = quietNaND();
    result.wheels[wheel].support_z = quietNaND();
    result.wheels[wheel].coverage = 0.0;
  }
  return result;
}

__device__ inline TerrainDeviceResult describeProjectedMiss(
    const SupportHit& hit, int wheel) {
  (void)wheel;
  if (hit.stale) {
    return reject(TerrainDeviceStatus::kStale,
                  TerrainDeviceDetail::kProjectedStale);
  }
  if (hit.unknown_column) {
    return reject(TerrainDeviceStatus::kUnknownSupport,
                  TerrainDeviceDetail::kProjectedUnobserved);
  }
  if (hit.coverage_failure) {
    return reject(TerrainDeviceStatus::kInsufficientCoverage,
                  TerrainDeviceDetail::kProjectedCoverage);
  }
  if (hit.confidence_failure) {
    return reject(TerrainDeviceStatus::kLowConfidence,
                  TerrainDeviceDetail::kProjectedLowConfidence);
  }
  if (hit.lower_surface) {
    return reject(TerrainDeviceStatus::kDropoff,
                  TerrainDeviceDetail::kProjectedDropoff);
  }
  return reject(TerrainDeviceStatus::kNoSupport,
                TerrainDeviceDetail::kProjectedNoSupport);
}

// Pure device query. Mirrors TerrainModel::evaluate line by line.
__device__ inline TerrainDeviceResult evaluate(
    const TerrainDeviceView& view, const TerrainDeviceQuery& query) {
  if (view.abi_version != kTerrainDeviceAbiVersion || view.valid == 0 ||
      view.cells == nullptr ||
      (view.candidate_count != 0 && view.candidates == nullptr) ||
      view.candidate_column_start == nullptr ||
      (view.candidate_column_count != 0 &&
       view.candidate_column_indices == nullptr) ||
      view.obstacle_column_start == nullptr ||
      (view.obstacle_column_count != 0 &&
       view.obstacle_column_cells == nullptr) ||
      view.observed_column_bits == nullptr) {
    return reject(TerrainDeviceStatus::kBuildFailure,
                  TerrainDeviceDetail::kBuildFailure);
  }
  if (view.geometry_valid == 0) {
    return reject(TerrainDeviceStatus::kInvalidGeometry,
                  TerrainDeviceDetail::kInvalidGeometry);
  }
  if (view.config_valid == 0) {
    return reject(TerrainDeviceStatus::kInvalidConfig,
                  TerrainDeviceDetail::kInvalidConfig);
  }
  if (!finiteScalar(query.position[0]) || !finiteScalar(query.position[1]) ||
      !finiteScalar(query.position[2]) || !finiteScalar(query.stamp) ||
      !finiteScalar(query.roll) || !finiteScalar(query.pitch) ||
      !finiteScalar(query.yaw)) {
    return reject(TerrainDeviceStatus::kInvalidQuery,
                  TerrainDeviceDetail::kInvalidQuery);
  }
  if (!finiteScalar(view.snapshot_stamp)) {
    return reject(TerrainDeviceStatus::kStale,
                  TerrainDeviceDetail::kSnapshotStampInvalid);
  }
  if (std::abs(query.stamp - view.snapshot_stamp) >
      view.config.max_snapshot_age) {
    return reject(TerrainDeviceStatus::kStale,
                  TerrainDeviceDetail::kSnapshotAge);
  }
  const Vec3 base = makeVec3(query.position[0], query.position[1],
                             query.position[2]);
  if (!snapshotInBounds(view, base)) {
    return reject(TerrainDeviceStatus::kOutside,
                  TerrainDeviceDetail::kBaseOutside);
  }

  const Mat3 predicted_rotation =
      rotationFromRpy(query.roll, query.pitch, query.yaw);
  Vec3 contact_points[4];
  Vec3 contact_support[4];
  bool contact_found[4] = {false, false, false, false};
  int contact_patches[4] = {-1, -1, -1, -1};
  double contact_coverage[4] = {0.0, 0.0, 0.0, 0.0};
  double contact_confidence[4] = {0.0, 0.0, 0.0, 0.0};
  bool saw_patch = false;
  bool saw_stale = false;
  bool saw_unknown_support = false;
  bool saw_coverage_failure = false;
  bool saw_confidence_failure = false;
  bool saw_lower_surface = false;
  bool saw_upper_surface = false;
  TerrainDeviceDetail stale_detail = TerrainDeviceDetail::kWheelStale;
  TerrainDeviceDetail unknown_support_detail =
      TerrainDeviceDetail::kWheelColumnUnobserved;
  TerrainDeviceDetail coverage_failure_detail =
      TerrainDeviceDetail::kWheelCoverage;
  TerrainDeviceDetail confidence_failure_detail =
      TerrainDeviceDetail::kWheelLowConfidence;

  for (int wheel = 0; wheel < 4; ++wheel) {
    const Vec3 contact =
        add(base, mul(predicted_rotation,
                      makeVec3(view.geometry.wheel_contacts_base[wheel][0],
                               view.geometry.wheel_contacts_base[wheel][1],
                               view.geometry.wheel_contacts_base[wheel][2])));
    contact_points[wheel] = contact;
    contact_support[wheel] = contact;
    if (!snapshotInBounds(view, contact) ||
        !patchInsideBounds(view, contact)) {
      return reject(TerrainDeviceStatus::kOutside,
                    TerrainDeviceDetail::kWheelPatchOutside);
    }
    const SupportHit hit = supportAt(view, contact.x, contact.y, contact.z,
                                     query.stamp, true, false);
    if (hit.found) {
      contact_found[wheel] = true;
      contact_patches[wheel] = hit.patch_id;
      contact_support[wheel] = makeVec3(contact.x, contact.y, hit.height);
      contact_coverage[wheel] = hit.coverage;
      contact_confidence[wheel] = hit.confidence;
      if (hit.confidence < view.config.min_plane_confidence) {
        saw_confidence_failure = true;
        confidence_failure_detail = TerrainDeviceDetail::kWheelLowConfidence;
      }
    } else {
      saw_patch = saw_patch || hit.saw_candidate;
      saw_stale = saw_stale || hit.stale;
      saw_unknown_support = saw_unknown_support || hit.unknown_column;
      saw_coverage_failure = saw_coverage_failure || hit.coverage_failure;
      if (hit.unknown_column) {
        unknown_support_detail =
            TerrainDeviceDetail::kWheelColumnUnobserved;
      }
      if (hit.confidence_failure) {
        saw_confidence_failure = true;
        confidence_failure_detail = TerrainDeviceDetail::kWheelLowConfidence;
      }
      saw_lower_surface = saw_lower_surface || hit.lower_surface;
      saw_upper_surface = saw_upper_surface || hit.upper_surface;
      if (hit.stale) {
        stale_detail = TerrainDeviceDetail::kWheelStale;
      }
      if (hit.coverage_failure) {
        coverage_failure_detail = TerrainDeviceDetail::kWheelCoverage;
      }
    }
  }
  bool any_missing = false;
  for (int wheel = 0; wheel < 4; ++wheel) {
    any_missing = any_missing || !contact_found[wheel];
  }
  if (any_missing) {
    if (saw_stale) {
      return reject(TerrainDeviceStatus::kStale, stale_detail);
    }
    if (saw_unknown_support) {
      return reject(TerrainDeviceStatus::kUnknownSupport,
                    unknown_support_detail);
    }
    if (saw_coverage_failure) {
      return reject(TerrainDeviceStatus::kInsufficientCoverage,
                    coverage_failure_detail);
    }
    if (saw_confidence_failure) {
      return reject(TerrainDeviceStatus::kLowConfidence,
                    confidence_failure_detail);
    }
    if (saw_lower_surface && saw_upper_surface) {
      return reject(TerrainDeviceStatus::kNoSupport,
                    TerrainDeviceDetail::kBracketedLayers);
    }
    if (saw_lower_surface) {
      return reject(TerrainDeviceStatus::kDropoff,
                    TerrainDeviceDetail::kDropoff);
    }
    if (saw_patch) {
      return reject(TerrainDeviceStatus::kNoSupport,
                    TerrainDeviceDetail::kNoSupportNearPredicted);
    }
    return reject(TerrainDeviceStatus::kNoSupport,
                  TerrainDeviceDetail::kNoCandidateUnderWheel);
  }

  TerrainDeviceStatus continuity_status = TerrainDeviceStatus::kOk;
  TerrainDeviceDetail continuity_detail = TerrainDeviceDetail::kNone;
  if (!wheelPathContinuity(view, contact_points, contact_support, query.stamp,
                           &continuity_status, &continuity_detail)) {
    return reject(continuity_status, continuity_detail);
  }
  if (saw_confidence_failure) {
    return reject(TerrainDeviceStatus::kLowConfidence,
                  confidence_failure_detail);
  }

  Vec3 coefficients = makeVec3(0.0, 0.0, 0.0);
  double rms = 0.0;
  double plane_confidence = 0.0;
  {
    Vec3 points[4];
    for (int i = 0; i < 4; ++i) {
      points[i] = contact_support[i];
    }
    if (!planeFit(points, 4, &coefficients, &rms, &plane_confidence)) {
      return reject(TerrainDeviceStatus::kNoSupport,
                    TerrainDeviceDetail::kWheelPlaneRankDeficient);
    }
    Vec3 normal = makeVec3(-coefficients.x, -coefficients.y, 1.0);
    if (!finiteScalar(normal.x) || !finiteScalar(normal.y) ||
        !finiteScalar(normal.z) || norm(normal) < 1e-9) {
      return reject(TerrainDeviceStatus::kNoSupport,
                    TerrainDeviceDetail::kDegeneratePlane);
    }
  }
  double max_residual = 0.0;
  for (int wheel = 0; wheel < 4; ++wheel) {
    const double predicted =
        coefficients.x * contact_support[wheel].x +
        coefficients.y * contact_support[wheel].y + coefficients.z;
    max_residual =
        maxValue(max_residual, std::abs(contact_support[wheel].z - predicted));
  }
  if (max_residual > view.config.max_step) {
    return reject(TerrainDeviceStatus::kStep,
                  TerrainDeviceDetail::kWheelStep);
  }
  if (rms > view.config.max_warp) {
    return reject(TerrainDeviceStatus::kWarp, TerrainDeviceDetail::kWarp);
  }

  {
    const Vec3 normal = makeVec3(-coefficients.x, -coefficients.y, 1.0);
    const Vec3 unit_normal = normalized(normal);
    const double slope =
        std::acos(clampDouble(unit_normal.z, -1.0, 1.0));
    if (slope > view.config.max_slope_rad) {
      return reject(TerrainDeviceStatus::kSlope,
                    TerrainDeviceDetail::kSlope);
    }
    const double cy = std::cos(query.yaw);
    const double sy = std::sin(query.yaw);
    const Vec3 body_normal =
        makeVec3(cy * unit_normal.x + sy * unit_normal.y,
                 -sy * unit_normal.x + cy * unit_normal.y, unit_normal.z);
    const double cross_slope =
        std::asin(clampDouble(-body_normal.y, -1.0, 1.0));
    if (std::abs(cross_slope) > view.config.max_cross_slope_rad) {
      return reject(TerrainDeviceStatus::kCrossSlope,
                    TerrainDeviceDetail::kCrossSlope);
    }
  }

  const double base_height = view.geometry.base_contact_height;
  const double pose_tolerance =
      maxValue(0.005, 0.25 * view.config.plane_inlier_tol);
  const double normal_tolerance =
      maxValue(0.002, 0.25 * view.config.plane_inlier_tol);
  const double contact_tolerance = maxValue(0.02, 2.0 * pose_tolerance);
  Vec3 projected_position = makeVec3(0.0, 0.0, 0.0);
  Mat3 projection{};
  projection.m[0][0] = 1.0;
  projection.m[1][1] = 1.0;
  projection.m[2][2] = 1.0;
  double projected_orientation[4] = {0.0, 0.0, 0.0, 1.0};
  Vec3 final_contacts[4];
  Vec3 final_supports[4];
  TerrainDeviceWheel final_wheels[4];
  bool converged = false;

  for (int iteration = 0; iteration < 3; ++iteration) {
    const Vec3 normal = normalFromCoefficients(coefficients);
    Mat3 rotation;
    double orientation[4];
    orientationFromNormalYaw(normal, query.yaw, &rotation, orientation);
    const Vec3 pose =
        basePositionOnPlane(coefficients, query.position[0],
                            query.position[1], base_height);
    Vec3 trial_contacts[4];
    Vec3 trial_supports[4];
    TerrainDeviceWheel trial_wheels[4];
    for (int wheel = 0; wheel < 4; ++wheel) {
      const Vec3 local =
          makeVec3(view.geometry.wheel_contacts_base[wheel][0],
                   view.geometry.wheel_contacts_base[wheel][1],
                   view.geometry.wheel_contacts_base[wheel][2]);
      const Vec3 contact = add(pose, mul(rotation, local));
      trial_contacts[wheel] = contact;
      if (!snapshotInBounds(view, contact) ||
          !patchInsideBounds(view, contact)) {
        return reject(TerrainDeviceStatus::kOutside,
                      TerrainDeviceDetail::kProjectedWheelPatchOutside);
      }
      const SupportHit hit = supportAt(view, contact.x, contact.y, contact.z,
                                       query.stamp, true, true);
      if (!hit.found) {
        return describeProjectedMiss(hit, wheel);
      }
      trial_supports[wheel] = makeVec3(contact.x, contact.y, hit.height);
      trial_wheels[wheel].supported = 1;
      trial_wheels[wheel].predicted_z = contact.z;
      trial_wheels[wheel].support_z = hit.height;
      trial_wheels[wheel].coverage = hit.coverage;
      trial_wheels[wheel].patch_id = hit.patch_id;
    }
    Vec3 next_coefficients;
    double next_rms = 0.0;
    double next_confidence = 0.0;
    if (!planeFit(trial_supports, 4, &next_coefficients, &next_rms,
                  &next_confidence)) {
      return reject(TerrainDeviceStatus::kNoSupport,
                    TerrainDeviceDetail::kProjectedRankDeficient);
    }
    double max_delta = 0.0;
    for (int wheel = 0; wheel < 4; ++wheel) {
      max_delta = maxValue(
          max_delta, std::abs(trial_contacts[wheel].z -
                              trial_supports[wheel].z));
    }
    const Vec3 old_normal = normalFromCoefficients(coefficients);
    const Vec3 new_normal = normalFromCoefficients(next_coefficients);
    const double normal_angle =
        std::acos(clampDouble(dot(old_normal, new_normal), -1.0, 1.0));
    coefficients = next_coefficients;
    projected_position = pose;
    projected_orientation[0] = orientation[0];
    projected_orientation[1] = orientation[1];
    projected_orientation[2] = orientation[2];
    projected_orientation[3] = orientation[3];
    projection = rotation;
    for (int wheel = 0; wheel < 4; ++wheel) {
      final_contacts[wheel] = trial_contacts[wheel];
      final_supports[wheel] = trial_supports[wheel];
      final_wheels[wheel] = trial_wheels[wheel];
    }
    if (max_delta <= pose_tolerance && normal_angle <= normal_tolerance) {
      converged = true;
      break;
    }
  }
  if (!converged) {
    return reject(TerrainDeviceStatus::kNoSupport,
                  TerrainDeviceDetail::kNotConverged);
  }

  {
    const Vec3 normal = normalFromCoefficients(coefficients);
    Mat3 rotation;
    double orientation[4];
    orientationFromNormalYaw(normal, query.yaw, &rotation, orientation);
    const Vec3 pose =
        basePositionOnPlane(coefficients, query.position[0],
                            query.position[1], base_height);
    TerrainDeviceWheel verified[4];
    for (int wheel = 0; wheel < 4; ++wheel) {
      const Vec3 local =
          makeVec3(view.geometry.wheel_contacts_base[wheel][0],
                   view.geometry.wheel_contacts_base[wheel][1],
                   view.geometry.wheel_contacts_base[wheel][2]);
      const Vec3 contact = add(pose, mul(rotation, local));
      if (!snapshotInBounds(view, contact) ||
          !patchInsideBounds(view, contact)) {
        return reject(TerrainDeviceStatus::kOutside,
                      TerrainDeviceDetail::kProjectedWheelPatchOutside);
      }
      const SupportHit hit = supportAt(view, contact.x, contact.y, contact.z,
                                       query.stamp, true, true);
      if (!hit.found) {
        return describeProjectedMiss(hit, wheel);
      }
      if (std::abs(hit.height - contact.z) > contact_tolerance) {
        return reject(TerrainDeviceStatus::kNoSupport,
                      TerrainDeviceDetail::kContactOffSupport);
      }
      verified[wheel].supported = 1;
      verified[wheel].predicted_z = contact.z;
      verified[wheel].support_z = hit.height;
      verified[wheel].coverage = hit.coverage;
      verified[wheel].patch_id = hit.patch_id;
      final_contacts[wheel] = contact;
      final_supports[wheel] = makeVec3(contact.x, contact.y, hit.height);
    }
    projected_position = pose;
    projected_orientation[0] = orientation[0];
    projected_orientation[1] = orientation[1];
    projected_orientation[2] = orientation[2];
    projected_orientation[3] = orientation[3];
    projection = rotation;
    for (int wheel = 0; wheel < 4; ++wheel) {
      final_wheels[wheel] = verified[wheel];
    }
  }

  {
    const Vec3 unit_normal = normalized(
        makeVec3(-coefficients.x, -coefficients.y, 1.0));
    const double slope =
        std::acos(clampDouble(unit_normal.z, -1.0, 1.0));
    if (slope > view.config.max_slope_rad) {
      return reject(TerrainDeviceStatus::kSlope,
                    TerrainDeviceDetail::kSlope);
    }
    const double cy = std::cos(query.yaw);
    const double sy = std::sin(query.yaw);
    const Vec3 body_normal =
        makeVec3(cy * unit_normal.x + sy * unit_normal.y,
                 -sy * unit_normal.x + cy * unit_normal.y, unit_normal.z);
    const double cross_slope =
        std::asin(clampDouble(-body_normal.y, -1.0, 1.0));
    if (std::abs(cross_slope) > view.config.max_cross_slope_rad) {
      return reject(TerrainDeviceStatus::kCrossSlope,
                    TerrainDeviceDetail::kCrossSlope);
    }
  }

  const Vec3 body_center_local = makeVec3(
      0.5 * (view.geometry.body_min[0] + view.geometry.body_max[0]),
      0.5 * (view.geometry.body_min[1] + view.geometry.body_max[1]),
      0.5 * (view.geometry.body_min[2] + view.geometry.body_max[2]));
  const Vec3 body_half = makeVec3(
      0.5 * (view.geometry.body_max[0] - view.geometry.body_min[0]),
      0.5 * (view.geometry.body_max[1] - view.geometry.body_min[1]),
      0.5 * (view.geometry.body_max[2] - view.geometry.body_min[2]));
  const Vec3 body_center =
      add(projected_position, mul(projection, body_center_local));
  const double voxel = view.voxel_size;
  const double map_lower[3] = {view.origin[0] * voxel,
                               view.origin[1] * voxel,
                               view.origin[2] * voxel};
  const double map_upper[3] = {map_lower[0] + view.dims[0] * voxel,
                               map_lower[1] + view.dims[1] * voxel,
                               map_lower[2] + view.dims[2] * voxel};
  const double body_half_array[3] = {body_half.x, body_half.y, body_half.z};
  double aabb_half_array[3];
  for (int i = 0; i < 3; ++i) {
    aabb_half_array[i] = std::abs(projection.m[i][0]) * body_half_array[0] +
                         std::abs(projection.m[i][1]) * body_half_array[1] +
                         std::abs(projection.m[i][2]) * body_half_array[2];
  }
  const Vec3 aabb_half = makeVec3(aabb_half_array[0], aabb_half_array[1],
                                  aabb_half_array[2]);
  const Vec3 body_aabb_min = sub(body_center, aabb_half);
  const Vec3 body_aabb_max = add(body_center, aabb_half);
  const double aabb_min_array[3] = {body_aabb_min.x, body_aabb_min.y,
                                    body_aabb_min.z};
  const double aabb_max_array[3] = {body_aabb_max.x, body_aabb_max.y,
                                    body_aabb_max.z};
  if (aabb_min_array[0] < map_lower[0] ||
      aabb_min_array[1] < map_lower[1] ||
      aabb_min_array[2] < map_lower[2] ||
      aabb_max_array[0] >= map_upper[0] ||
      aabb_max_array[1] >= map_upper[1] ||
      aabb_max_array[2] >= map_upper[2]) {
    return reject(TerrainDeviceStatus::kOutside,
                  TerrainDeviceDetail::kBodyAabbOutside);
  }

  double clearance = infinityD();
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const Vec3 local =
            makeVec3(body_center_local.x + sx * body_half.x,
                     body_center_local.y + sy * body_half.y,
                     body_center_local.z + sz * body_half.z);
        const Vec3 corner = add(projected_position, mul(projection, local));
        const double local_support = coefficients.x * corner.x +
                                     coefficients.y * corner.y +
                                     coefficients.z;
        clearance = minValue(clearance, corner.z - local_support);
      }
    }
  }
  if (clearance < view.geometry.body_clearance - 1e-9) {
    TerrainDeviceResult result =
        reject(TerrainDeviceStatus::kInsufficientBodyClearance,
               TerrainDeviceDetail::kBodyClearance);
    result.clearance = clearance;
    return result;
  }

  int64_t ix0 = static_cast<int64_t>(std::floor(body_aabb_min.x / voxel));
  int64_t iy0 = static_cast<int64_t>(std::floor(body_aabb_min.y / voxel));
  int64_t iz0 = static_cast<int64_t>(std::floor(body_aabb_min.z / voxel));
  int64_t ix1 = static_cast<int64_t>(std::floor(body_aabb_max.x / voxel));
  int64_t iy1 = static_cast<int64_t>(std::floor(body_aabb_max.y / voxel));
  int64_t iz1 = static_cast<int64_t>(std::floor(body_aabb_max.z / voxel));
  ix0 = maxValue(ix0, view.origin[0]);
  iy0 = maxValue(iy0, view.origin[1]);
  iz0 = maxValue(iz0, view.origin[2]);
  ix1 = minValue(ix1, view.origin[0] + view.dims[0] - 1);
  iy1 = minValue(iy1, view.origin[1] + view.dims[1] - 1);
  iz1 = minValue(iz1, view.origin[2] + view.dims[2] - 1);

  bool body_collision = false;
  bool unknown_body = false;
  bool stale_body = false;
  const double available_body_clearance =
      minValue(view.geometry.body_clearance, clearance);
  const double floor_tolerance =
      maxValue(0.0, minValue(0.5 * available_body_clearance,
                            view.config.plane_inlier_tol));
  int64_t rx = floorMod(ix0, view.dims[0]);
  for (int64_t ix = ix0; ix <= ix1 && !body_collision;
       ++ix, rx = (rx + 1 == view.dims[0]) ? 0 : rx + 1) {
    int64_t ry = floorMod(iy0, view.dims[1]);
    for (int64_t iy = iy0; iy <= iy1 && !body_collision;
         ++iy, ry = (ry + 1 == view.dims[1]) ? 0 : ry + 1) {
      const size_t column_index = static_cast<size_t>(
          (rx * static_cast<int64_t>(view.dims[1]) + ry) *
          static_cast<int64_t>(view.dims[2]));
      int64_t rz = floorMod(iz0, view.dims[2]);
      for (int64_t iz = iz0; iz <= iz1;
           ++iz, rz = (rz + 1 == view.dims[2]) ? 0 : rz + 1) {
        const CellView lookup =
            decodePhysicalCell(view, column_index + static_cast<size_t>(rz));
        if (!lookup.in_bounds) {
          return reject(TerrainDeviceStatus::kOutside,
                        TerrainDeviceDetail::kBodySearchOutside);
        }
        const Vec3 cell_min = makeVec3(static_cast<double>(ix) * voxel,
                                       static_cast<double>(iy) * voxel,
                                       static_cast<double>(iz) * voxel);
        const Vec3 cell_max =
            add(cell_min, makeVec3(voxel, voxel, voxel));
        if (!lookup.observed) {
          if (view.config.unknown_body_policy == 0U) {
            if (obbIntersectsAabb(body_center, body_half, projection, cell_min,
                                  cell_max)) {
              unknown_body = true;
            }
          }
          continue;
        }
        if (!freshObservation(view, lookup.last_observed_stamp, query.stamp)) {
          stale_body = true;
          continue;
        }
        const bool actual_hit = validLookupHit(view, lookup, query.stamp);
        if (!actual_hit && lookup.occupancy == 1U) {
          continue;
        }
        if (!actual_hit && lookup.occupancy == 0U) {
          if (view.config.unknown_body_policy == 0U) {
            if (obbIntersectsAabb(body_center, body_half, projection, cell_min,
                                  cell_max)) {
              unknown_body = true;
            }
          }
          continue;
        }
        if (actual_hit) {
          const double x0 = cell_min.x;
          const double x1 = cell_max.x;
          const double y0 = cell_min.y;
          const double y1 = cell_max.y;
          const double support_at_cell =
              minValue(minValue(coefficients.x * x0 + coefficients.y * y0,
                                coefficients.x * x0 + coefficients.y * y1),
                       minValue(coefficients.x * x1 + coefficients.y * y0,
                                coefficients.x * x1 + coefficients.y * y1)) +
              coefficients.z;
          if (finiteScalar(lookup.hit_max_z) &&
              lookup.hit_max_z <= support_at_cell + floor_tolerance) {
            continue;
          }
        }
        const bool overlaps = obbIntersectsAabb(
            body_center, body_half, projection, cell_min, cell_max);
        if (overlaps && actual_hit) {
          body_collision = true;
          break;
        }
        if (lookup.occupancy == 2U) {
          if (overlaps) {
            body_collision = true;
            break;
          }
        } else if (lookup.occupancy == 0U) {
          if (overlaps && view.config.unknown_body_policy == 0U) {
            unknown_body = true;
          }
        }
      }
    }
  }
  if (body_collision) {
    TerrainDeviceResult result =
        reject(TerrainDeviceStatus::kBodyCollision,
               TerrainDeviceDetail::kBodyCollision);
    result.clearance = clearance;
    return result;
  }
  if (stale_body) {
    TerrainDeviceResult result =
        reject(TerrainDeviceStatus::kStale, TerrainDeviceDetail::kBodyStale);
    result.clearance = clearance;
    return result;
  }
  if (unknown_body) {
    TerrainDeviceResult result =
        reject(TerrainDeviceStatus::kUnknownBody,
               TerrainDeviceDetail::kBodyUnknown);
    result.clearance = clearance;
    return result;
  }

  const double search_margin = view.config.obstacle_search_margin;
  const Vec3 search_min =
      sub(body_aabb_min, makeVec3(search_margin, search_margin, search_margin));
  const Vec3 search_max =
      add(body_aabb_max, makeVec3(search_margin, search_margin, search_margin));
  const double search_min_array[3] = {search_min.x, search_min.y,
                                      search_min.z};
  const double search_max_array[3] = {search_max.x, search_max.y,
                                      search_max.z};
  if (search_min_array[0] < map_lower[0] ||
      search_min_array[1] < map_lower[1] ||
      search_min_array[2] < map_lower[2] ||
      search_max_array[0] >= map_upper[0] ||
      search_max_array[1] >= map_upper[1] ||
      search_max_array[2] >= map_upper[2]) {
    return reject(TerrainDeviceStatus::kOutside,
                  TerrainDeviceDetail::kObstacleSearchOutside);
  }
  int64_t ox0 = maxValue(
      static_cast<int64_t>(std::floor(search_min.x / voxel)), view.origin[0]);
  int64_t oy0 = maxValue(
      static_cast<int64_t>(std::floor(search_min.y / voxel)), view.origin[1]);
  int64_t oz0 = maxValue(
      static_cast<int64_t>(std::floor(search_min.z / voxel)), view.origin[2]);
  int64_t ox1 = minValue(
      static_cast<int64_t>(std::floor(search_max.x / voxel)),
      view.origin[0] + view.dims[0] - 1);
  int64_t oy1 = minValue(
      static_cast<int64_t>(std::floor(search_max.y / voxel)),
      view.origin[1] + view.dims[1] - 1);
  int64_t oz1 = minValue(
      static_cast<int64_t>(std::floor(search_max.z / voxel)),
      view.origin[2] + view.dims[2] - 1);
  double obstacle_clearance = infinityD();
  const int64_t obstacle_z_origin = view.origin[2];
  const int64_t obstacle_z_bias = floorMod(obstacle_z_origin, view.dims[2]);
  int64_t orx = floorMod(ox0, view.dims[0]);
  for (int64_t ix = ox0; ix <= ox1;
       ++ix, orx = (orx + 1 == view.dims[0]) ? 0 : orx + 1) {
    int64_t ory = floorMod(oy0, view.dims[1]);
    for (int64_t iy = oy0; iy <= oy1;
         ++iy, ory = (ory + 1 == view.dims[1]) ? 0 : ory + 1) {
      size_t column = 0;
      if (!columnViewIndex(view, ix, iy, &column)) {
        continue;
      }
      const size_t column_index = static_cast<size_t>(
          (orx * static_cast<int64_t>(view.dims[1]) + ory) *
          static_cast<int64_t>(view.dims[2]));
      for (uint64_t position = view.obstacle_column_start[column];
           position < view.obstacle_column_start[column + 1U]; ++position) {
        const int64_t iz = view.obstacle_column_cells[position];
        if (iz < oz0 || iz > oz1) {
          continue;
        }
        int64_t rz = obstacle_z_bias + (iz - obstacle_z_origin);
        if (rz >= view.dims[2]) {
          rz -= view.dims[2];
        }
        const CellView lookup =
            decodePhysicalCell(view, column_index + static_cast<size_t>(rz));
        const bool actual_hit = validLookupHit(view, lookup, query.stamp);
        if (!actual_hit && lookup.occupancy != 2U) {
          continue;
        }
        const Vec3 cell_min = makeVec3(static_cast<double>(ix) * voxel,
                                       static_cast<double>(iy) * voxel,
                                       static_cast<double>(iz) * voxel);
        const Vec3 cell_max =
            add(cell_min, makeVec3(voxel, voxel, voxel));
        const double x0 = cell_min.x;
        const double x1 = cell_max.x;
        const double y0 = cell_min.y;
        const double y1 = cell_max.y;
        const double support_at_cell =
            minValue(minValue(coefficients.x * x0 + coefficients.y * y0,
                              coefficients.x * x0 + coefficients.y * y1),
                     minValue(coefficients.x * x1 + coefficients.y * y0,
                              coefficients.x * x1 + coefficients.y * y1)) +
            coefficients.z;
        if (actual_hit && finiteScalar(lookup.hit_max_z) &&
            lookup.hit_max_z <= support_at_cell + floor_tolerance) {
          continue;
        }
        const double distance = obbAabbDistance(
            body_center, body_half, projection, cell_min, cell_max);
        obstacle_clearance = minValue(obstacle_clearance, distance);
      }
    }
  }

  int selected_patch = -1;
  for (int wheel = 0; wheel < 4; ++wheel) {
    if (selected_patch < 0) {
      selected_patch = final_wheels[wheel].patch_id;
    }
  }

  TerrainDeviceResult result;
  result.status = static_cast<uint8_t>(TerrainDeviceStatus::kOk);
  result.detail = static_cast<uint8_t>(TerrainDeviceDetail::kNone);
  result.projected_position[0] = projected_position.x;
  result.projected_position[1] = projected_position.y;
  result.projected_position[2] = projected_position.z;
  for (int i = 0; i < 4; ++i) {
    result.projected_orientation[i] = projected_orientation[i];
  }
  result.roll = std::atan2(projection.m[2][1], projection.m[2][2]);
  result.pitch =
      std::asin(clampDouble(-projection.m[2][0], -1.0, 1.0));
  result.yaw = std::atan2(projection.m[1][0], projection.m[0][0]);
  result.clearance = clearance;
  result.obstacle_clearance = obstacle_clearance;
  result.selected_patch = selected_patch;
  for (int wheel = 0; wheel < 4; ++wheel) {
    result.wheels[wheel] = final_wheels[wheel];
  }
  for (int wheel = 0; wheel < 4; ++wheel) {
    if (!result.wheels[wheel].supported) {
      result.wheels[wheel].predicted_z = final_contacts[wheel].z;
      result.wheels[wheel].support_z = final_supports[wheel].z;
    }
  }
  return result;
}

}  // namespace cuda_terrain
}  // namespace local_planner
