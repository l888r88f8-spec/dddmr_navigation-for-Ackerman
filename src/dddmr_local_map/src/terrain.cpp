#include "dddmr_local_map/terrain.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dddmr_local_map {
namespace {

bool finite(const Vec3& value) { return value.allFinite(); }

bool finiteScalar(double value) { return std::isfinite(value); }

Mat3 rotationFromRpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
          Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3::UnitX()))
      .toRotationMatrix();
}

bool planeFit(const std::vector<Vec3>& points, Vec3* coefficients,
              double* rms, double* confidence) {
  // A plane is only determined by at least three non-collinear samples. Never
  // report confident support for a small or rank-deficient sample; callers
  // must decide whether a horizontal fallback with zero confidence is useful.
  if (points.size() < 3) {
    return false;
  }
  Eigen::MatrixXd a(static_cast<Eigen::Index>(points.size()), 3);
  Eigen::VectorXd b(static_cast<Eigen::Index>(points.size()));
  for (std::size_t i = 0; i < points.size(); ++i) {
    a(static_cast<Eigen::Index>(i), 0) = points[i].x();
    a(static_cast<Eigen::Index>(i), 1) = points[i].y();
    a(static_cast<Eigen::Index>(i), 2) = 1.0;
    b(static_cast<Eigen::Index>(i)) = points[i].z();
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
  if (qr.rank() < 3) {
    return false;
  }
  const Vec3 solution = qr.solve(b);
  if (!solution.allFinite()) {
    return false;
  }
  *coefficients = solution;
  double sse = 0.0;
  double mean_z = 0.0;
  for (const auto& point : points) {
    mean_z += point.z();
  }
  mean_z /= static_cast<double>(points.size());
  double sst = 0.0;
  for (const auto& point : points) {
    const double prediction =
        solution.x() * point.x() + solution.y() * point.y() + solution.z();
    const double residual = point.z() - prediction;
    sse += residual * residual;
    const double centered = point.z() - mean_z;
    sst += centered * centered;
  }
  *rms = std::sqrt(sse / static_cast<double>(points.size()));
  if (sst < 1e-12) {
    *confidence = (*rms < 1e-6) ? 1.0 : 0.0;
  } else {
    *confidence = std::clamp(1.0 - sse / sst, 0.0, 1.0);
  }
  return true;
}

bool obbIntersectsAabb(const Vec3& obb_center, const Vec3& half_extents,
                       const Mat3& rotation, const Vec3& aabb_min,
                       const Vec3& aabb_max) {
  const Vec3 aabb_center = 0.5 * (aabb_min + aabb_max);
  const Vec3 aabb_half = 0.5 * (aabb_max - aabb_min);
  const Vec3 translation = aabb_center - obb_center;
  const Mat3 abs_rotation = rotation.cwiseAbs();

  for (int i = 0; i < 3; ++i) {
    if (std::abs(translation[i]) >
        aabb_half[i] + half_extents.dot(abs_rotation.row(i)) + 1e-12) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (std::abs(translation.dot(rotation.col(i))) >
        half_extents[i] + aabb_half.dot(abs_rotation.col(i)) + 1e-12) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Vec3 axis = rotation.col(i).cross(Vec3::Unit(j));
      const double length = axis.norm();
      if (length < 1e-12) {
        continue;
      }
      axis /= length;
      const double ra =
          half_extents.dot((rotation.transpose() * axis).cwiseAbs());
      const double rb = aabb_half.dot(axis.cwiseAbs());
      if (std::abs(translation.dot(axis)) > ra + rb + 1e-12) {
        return false;
      }
    }
  }
  return true;
}

double pointAabbDistance(const Vec3& point, const Vec3& aabb_min,
                         const Vec3& aabb_max) {
  Vec3 delta = Vec3::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] < aabb_min[axis]) {
      delta[axis] = aabb_min[axis] - point[axis];
    } else if (point[axis] > aabb_max[axis]) {
      delta[axis] = point[axis] - aabb_max[axis];
    }
  }
  return delta.norm();
}

double pointObbDistance(const Vec3& point, const Vec3& obb_center,
                        const Vec3& half_extents, const Mat3& rotation) {
  const Vec3 local = rotation.transpose() * (point - obb_center);
  Vec3 delta = Vec3::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    delta[axis] = std::max(0.0, std::abs(local[axis]) - half_extents[axis]);
  }
  return delta.norm();
}

// Minimum Euclidean distance between a rigid OBB and an axis-aligned voxel
// box. For convex polytopes the closest pair includes a vertex of one of the
// two boxes, so checking both vertex sets is sufficient.
double obbAabbDistance(const Vec3& obb_center, const Vec3& half_extents,
                       const Mat3& rotation, const Vec3& aabb_min,
                       const Vec3& aabb_max) {
  if (obbIntersectsAabb(obb_center, half_extents, rotation, aabb_min,
                        aabb_max)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  const Vec3 aabb_center = 0.5 * (aabb_min + aabb_max);
  const Vec3 aabb_half = 0.5 * (aabb_max - aabb_min);
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const Vec3 sign(static_cast<double>(sx), static_cast<double>(sy),
                        static_cast<double>(sz));
        const Vec3 obb_corner =
            obb_center + rotation * half_extents.cwiseProduct(sign);
        best = std::min(best,
                        pointAabbDistance(obb_corner, aabb_min, aabb_max));
        const Vec3 aabb_corner =
            aabb_center + aabb_half.cwiseProduct(sign);
        best = std::min(best, pointObbDistance(aabb_corner, obb_center,
                                               half_extents, rotation));
      }
    }
  }
  return best;
}

}  // namespace

const char* toString(TerrainStatus value) {
  switch (value) {
    case TerrainStatus::kOk:
      return "ok";
    case TerrainStatus::kInvalidGeometry:
      return "invalid_geometry";
    case TerrainStatus::kInvalidConfig:
      return "invalid_config";
    case TerrainStatus::kInvalidQuery:
      return "invalid_query";
    case TerrainStatus::kStale:
      return "stale";
    case TerrainStatus::kOutside:
      return "outside";
    case TerrainStatus::kUnknownSupport:
      return "unknown_support";
    case TerrainStatus::kNoSupport:
      return "no_support";
    case TerrainStatus::kDropoff:
      return "dropoff";
    case TerrainStatus::kStep:
      return "step";
    case TerrainStatus::kSlope:
      return "slope";
    case TerrainStatus::kCrossSlope:
      return "cross_slope";
    case TerrainStatus::kWarp:
      return "warp";
    case TerrainStatus::kInsufficientCoverage:
      return "insufficient_coverage";
    case TerrainStatus::kLowConfidence:
      return "low_confidence";
    case TerrainStatus::kInsufficientBodyClearance:
      return "insufficient_body_clearance";
    case TerrainStatus::kBodyCollision:
      return "body_collision";
    case TerrainStatus::kUnknownBody:
      return "unknown_body";
    case TerrainStatus::kBuildFailure:
      return "build_failure";
  }
  return "invalid";
}

Validation VehicleGeometry::validate() const {
  if (!finiteScalar(base_contact_height) || base_contact_height <= 0.0) {
    return Validation::failure(
        "base_contact_height must be finite and > 0");
  }
  for (std::size_t i = 0; i < wheel_contacts_base.size(); ++i) {
    const auto& contact = wheel_contacts_base[i];
    if (!finite(contact)) {
      return Validation::failure("wheel contact offsets must be finite");
    }
    if (std::abs(contact.z() + base_contact_height) > 1e-6) {
      return Validation::failure(
          "each wheel contact z must equal -base_contact_height");
    }
  }
  bool distinct = true;
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = i + 1; j < 4; ++j) {
      if ((wheel_contacts_base[i].head<2>() -
           wheel_contacts_base[j].head<2>())
              .norm() <= 1e-6) {
        distinct = false;
      }
    }
  }
  if (!distinct) {
    return Validation::failure(
        "wheel contact offsets must be pairwise distinct in x/y");
  }
  bool non_collinear = false;
  for (std::size_t i = 0; i < 4 && !non_collinear; ++i) {
    for (std::size_t j = i + 1; j < 4 && !non_collinear; ++j) {
      for (std::size_t k = j + 1; k < 4; ++k) {
        const Vec3 edge_a(wheel_contacts_base[j].x() -
                              wheel_contacts_base[i].x(),
                          wheel_contacts_base[j].y() -
                              wheel_contacts_base[i].y(),
                          0.0);
        const Vec3 edge_b(wheel_contacts_base[k].x() -
                              wheel_contacts_base[i].x(),
                          wheel_contacts_base[k].y() -
                              wheel_contacts_base[i].y(),
                          0.0);
        if (std::abs(edge_a.x() * edge_b.y() - edge_a.y() * edge_b.x()) >
            1e-9) {
          non_collinear = true;
          break;
        }
      }
    }
  }
  if (!non_collinear) {
    return Validation::failure(
        "wheel contact offsets must not be collinear");
  }
  // Contact order is front-left, front-right, rear-left, rear-right. Positively
  // separated axle pairs and a positive front/rear wheelbase prevent a
  // degenerate "four wheel" plane that is really a line or a point.
  const double front_track =
      (wheel_contacts_base[0].head<2>() - wheel_contacts_base[1].head<2>())
          .norm();
  const double rear_track =
      (wheel_contacts_base[2].head<2>() - wheel_contacts_base[3].head<2>())
          .norm();
  const double front_x =
      0.5 * (wheel_contacts_base[0].x() + wheel_contacts_base[1].x());
  const double rear_x =
      0.5 * (wheel_contacts_base[2].x() + wheel_contacts_base[3].x());
  if (front_track <= 1e-6 || rear_track <= 1e-6 ||
      std::abs(front_x - rear_x) <= 1e-6) {
    return Validation::failure(
        "wheel geometry requires positive track on both axles and a positive "
        "wheelbase");
  }
  if (!finite(body_min) || !finite(body_max) ||
      (body_max.array() <= body_min.array()).any()) {
    return Validation::failure(
        "body_min/body_max must be finite with min < max on every axis");
  }
  if (!finiteScalar(body_clearance) || body_clearance < 0.0) {
    return Validation::failure(
        "body_clearance must be finite and non-negative");
  }
  return Validation::success();
}

Validation TerrainConfig::validate() const {
  const auto positive = [](double value) {
    return finiteScalar(value) && value > 0.0;
  };
  if (!positive(contact_patch_radius) || !positive(max_contact_search_z) ||
      !positive(max_drop) || !positive(max_step) || !positive(max_slope_rad) ||
      !positive(max_cross_slope_rad) || !positive(max_warp) ||
      !positive(max_snapshot_age) || !positive(max_observation_age) ||
      !positive(cluster_max_dz) || !positive(plane_inlier_tol)) {
    return Validation::failure(
        "terrain distances/tolerances and observation ages must be finite "
        "and > 0");
  }
  if (!finiteScalar(obstacle_search_margin) ||
      obstacle_search_margin < 0.0) {
    return Validation::failure(
        "obstacle_search_margin must be finite and non-negative");
  }
  if (min_patch_coverage < 0.0 || min_patch_coverage > 1.0 ||
      !finiteScalar(min_patch_coverage)) {
    return Validation::failure("min_patch_coverage must be in [0, 1]");
  }
  if (min_plane_confidence < 0.0 || min_plane_confidence > 1.0 ||
      !finiteScalar(min_plane_confidence)) {
    return Validation::failure("min_plane_confidence must be in [0, 1]");
  }
  if (max_slope_rad >= 1.5 || max_cross_slope_rad >= 1.5) {
    return Validation::failure("slope limits must be < 1.5 rad");
  }
  if (max_patches == 0 || max_patches > 1'000'000) {
    return Validation::failure("max_patches is outside the safety range");
  }
  if (unknown_body_policy != UnknownBodyPolicy::kReject &&
      unknown_body_policy != UnknownBodyPolicy::kAllow) {
    return Validation::failure("unknown_body_policy is invalid");
  }
  return Validation::success();
}

struct TerrainModel::Impl {
  struct Candidate {
    int64_t ix{0};
    int64_t iy{0};
    int64_t iz{0};
    // Measured mean endpoint of the latest accepted hit frame. This is the
    // geometric surface sample; the occupied voxel volume is only used for
    // conservative body collision.
    Vec3 point{Vec3::Zero()};
    Vec3 local_coefficients{Vec3::Zero()};
    double local_confidence{0.0};
    double local_residual{std::numeric_limits<double>::infinity()};
    double last_hit_stamp{kNaN};
    double last_miss_stamp{kNaN};
    int patch_id{-1};
  };

  struct Patch {
    Vec3 coefficients{Vec3::Zero()};
    Vec3 normal{Vec3::UnitZ()};
    double rms{0.0};
    double confidence{1.0};
    std::vector<int> candidates;
  };

  std::shared_ptr<const MapSnapshot> snapshot;
  TerrainConfig config;
  VehicleGeometry geometry;
  Validation geometry_status;
  Validation config_status;
  Validation build_status;
  std::vector<Candidate> candidates;
  std::vector<Patch> patches;
  std::unordered_map<ColumnKey, std::vector<int>, ColumnKeyHash>
      candidates_by_column;
  std::unordered_set<ColumnKey, ColumnKeyHash> observed_columns;
  // Per-map-frame sparse index for the bounded soft-obstacle clearance scan.
  // A cell can be a soft-obstacle candidate for some query only if it is
  // currently occupied or carries an actual-HIT record; every other cell fails
  // the "actual_hit || occupied" test for every query stamp. The list keeps
  // the exact per-query freshness/floor checks below.
  std::unordered_map<ColumnKey, std::vector<int64_t>, ColumnKeyHash>
      obstacle_cells_by_column;

  // Frozen origin-relative column views. They are built exactly once, after
  // extractCandidates() and before any query, from the same per-column data.
  // A query converts a world column to a dense index with the snapshot origin
  // and dims, so negative world coordinates and a rolled window need no
  // special casing and the hot path avoids hash lookups. The start arrays are
  // prefix sums (CSR layout); candidates within each column keep their
  // extraction order. observed_column_bits distinguishes an observed column
  // with no candidate from a column that was never observed.
  std::vector<std::size_t> candidate_column_start;
  std::vector<int> candidate_column_indices;
  std::vector<std::size_t> obstacle_column_start;
  std::vector<int64_t> obstacle_column_cells;
  std::vector<uint8_t> observed_column_bits;

  bool columnViewIndex(const ColumnKey& key, std::size_t* index) const {
    if (!snapshot || candidate_column_start.empty()) {
      return false;
    }
    const auto origin = snapshot->originVoxel();
    const auto dims = snapshot->windowDims();
    const int64_t local_x = key.x - origin[0];
    const int64_t local_y = key.y - origin[1];
    if (local_x < 0 || local_y < 0 || local_x >= dims.x() ||
        local_y >= dims.y()) {
      return false;
    }
    *index = static_cast<std::size_t>(local_y) *
                 static_cast<std::size_t>(dims.x()) +
             static_cast<std::size_t>(local_x);
    return *index < candidate_column_start.size() - 1U;
  }

  double planeHeight(const Patch& patch, double x, double y) const {
    return patch.coefficients.x() * x + patch.coefficients.y() * y +
           patch.coefficients.z();
  }

  double candidateHeight(const Candidate& candidate, double x,
                         double y) const {
    return candidate.local_coefficients.x() * x +
           candidate.local_coefficients.y() * y +
           candidate.local_coefficients.z();
  }

  // Support height is evaluated from the candidate-local plane fitted to the
  // measured hit centroids. There is no voxel-top floor and no patch-global
  // slope/tilt bias: a distant connected ramp must not move a local flat
  // surface, and a measured surface must not be lifted by a voxel boundary.
  double supportHeight(const Candidate& candidate, double x, double y) const {
    return candidateHeight(candidate, x, y);
  }

  void computeLocalPlanes() {
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      Candidate& candidate = candidates[index];
      std::vector<Vec3> local_points;
      local_points.push_back(candidate.point);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          std::size_t column = 0;
          if (!columnViewIndex(
                  ColumnKey{candidate.ix + dx, candidate.iy + dy}, &column)) {
            continue;
          }
          for (std::size_t position = candidate_column_start[column];
               position < candidate_column_start[column + 1U]; ++position) {
            const int neighbour = candidate_column_indices[position];
            const Candidate& other =
                candidates[static_cast<std::size_t>(neighbour)];
            if (std::abs(other.point.z() - candidate.point.z()) <=
                config.cluster_max_dz + 1e-9) {
              local_points.push_back(other.point);
            }
          }
        }
      }
      Vec3 coefficients = Vec3::Zero();
      double rms = 0.0;
      double confidence = 1.0;
      if (!planeFit(local_points, &coefficients, &rms, &confidence)) {
        coefficients = Vec3(0.0, 0.0, candidate.point.z());
        confidence = 0.0;
        rms = 0.0;
      }
      candidate.local_coefficients = coefficients;
      candidate.local_residual =
          std::abs(candidate.point.z() -
                   candidateHeight(candidate, candidate.point.x(),
                                   candidate.point.y()));
      // A high local residual means the neighborhood is not a single local
      // plane (for example a narrow dip or a layer edge). Do not let the
      // plane smooth that feature and still report high confidence.
      if (candidate.local_residual > config.plane_inlier_tol) {
        confidence = 0.0;
      }
      candidate.local_confidence = confidence;
    }
  }

  bool insideMapColumn(int64_t x, int64_t y) const {
    if (!snapshot) {
      return false;
    }
    const auto origin = snapshot->originVoxel();
    const auto dims = snapshot->windowDims();
    return x >= origin[0] && x < origin[0] + dims.x() && y >= origin[1] &&
           y < origin[1] + dims.y();
  }

  void extractCandidates() {
    if (!snapshot) {
      return;
    }
    const auto origin = snapshot->originVoxel();
    const auto dims = snapshot->windowDims();
    for (int64_t ix = origin[0]; ix < origin[0] + dims.x(); ++ix) {
      for (int64_t iy = origin[1]; iy < origin[1] + dims.y(); ++iy) {
        bool observed = false;
        bool in_occupied_run = false;
        int64_t run_top = 0;
        const auto add_run_candidate = [&]() -> bool {
          const auto top = snapshot->lookupVoxel(ix, iy, run_top, false);
          if (!top.hit_position.allFinite()) {
            // Occupancy must come from a real hit; without measured geometry
            // there is no support candidate (fail closed rather than invent a
            // voxel-top height).
            return true;
          }
          Candidate candidate;
          candidate.ix = ix;
          candidate.iy = iy;
          candidate.iz = run_top;
          candidate.point = top.hit_position;
          candidate.last_hit_stamp = top.last_hit_stamp;
          candidate.last_miss_stamp = top.last_miss_stamp;
          candidates.push_back(candidate);
          candidates_by_column[ColumnKey{ix, iy}].push_back(
              static_cast<int>(candidates.size()) - 1);
          if (candidates.size() > 2'000'000U) {
            build_status =
                Validation::failure("surface candidate safety cap exceeded");
            return false;
          }
          return true;
        };
        for (int64_t iz = origin[2]; iz < origin[2] + dims.z(); ++iz) {
          const auto lookup = snapshot->lookupVoxel(ix, iy, iz, false);
          // Build the sparse soft-obstacle index once per map frame. Only an
          // occupied voxel or a voxel with an actual-HIT record can ever pass
          // the clearance loop's "actual_hit || occupied" gate; empty cells
          // cannot, so they are never visited by the per-query scan.
          if (lookup.occupancy == Occupancy::kOccupied ||
              finiteScalar(lookup.last_hit_stamp)) {
            obstacle_cells_by_column[ColumnKey{ix, iy}].push_back(iz);
          }
          observed = observed || lookup.observed;
          const bool occupied = lookup.occupancy == Occupancy::kOccupied;
          if (occupied) {
            in_occupied_run = true;
            run_top = iz;
          } else if (in_occupied_run) {
            if (!add_run_candidate()) {
              return;
            }
            in_occupied_run = false;
          }
        }
        if (in_occupied_run) {
          if (!add_run_candidate()) {
            return;
          }
        }
        if (observed) {
          observed_columns.insert(ColumnKey{ix, iy});
        }
      }
    }
  }

  void freezeColumnViews() {
    if (!snapshot) {
      return;
    }
    const auto origin = snapshot->originVoxel();
    const auto dims = snapshot->windowDims();
    const std::size_t width = static_cast<std::size_t>(dims.x());
    const std::size_t height = static_cast<std::size_t>(dims.y());
    const std::size_t columns = width * height;

    const auto column_index = [&](const ColumnKey& key,
                                  std::size_t* index) -> bool {
      const int64_t local_x = key.x - origin[0];
      const int64_t local_y = key.y - origin[1];
      if (local_x < 0 || local_y < 0 || local_x >= dims.x() ||
          local_y >= dims.y()) {
        return false;
      }
      *index = static_cast<std::size_t>(local_y) * width +
               static_cast<std::size_t>(local_x);
      return true;
    };

    // Counting pass, then prefix sum and a stable fill pass. The iteration
    // order of the temporary hash maps is irrelevant because every payload is
    // placed at its own dense column.
    candidate_column_start.assign(columns + 1U, 0U);
    obstacle_column_start.assign(columns + 1U, 0U);
    observed_column_bits.assign(columns, 0U);
    for (const auto& entry : candidates_by_column) {
      std::size_t index = 0;
      if (column_index(entry.first, &index)) {
        candidate_column_start[index + 1U] = entry.second.size();
      }
    }
    for (const auto& entry : obstacle_cells_by_column) {
      std::size_t index = 0;
      if (column_index(entry.first, &index)) {
        obstacle_column_start[index + 1U] = entry.second.size();
      }
    }
    for (std::size_t index = 0; index < columns; ++index) {
      candidate_column_start[index + 1U] += candidate_column_start[index];
      obstacle_column_start[index + 1U] += obstacle_column_start[index];
    }
    candidate_column_indices.resize(candidate_column_start[columns]);
    obstacle_column_cells.resize(obstacle_column_start[columns]);
    std::vector<std::size_t> candidate_cursor(candidate_column_start.begin(),
                                              candidate_column_start.end() - 1);
    std::vector<std::size_t> obstacle_cursor(obstacle_column_start.begin(),
                                             obstacle_column_start.end() - 1);
    for (const auto& entry : candidates_by_column) {
      std::size_t index = 0;
      if (!column_index(entry.first, &index)) {
        continue;
      }
      std::size_t& cursor = candidate_cursor[index];
      for (const int candidate : entry.second) {
        candidate_column_indices[cursor++] = candidate;
      }
    }
    for (const auto& entry : obstacle_cells_by_column) {
      std::size_t index = 0;
      if (!column_index(entry.first, &index)) {
        continue;
      }
      std::size_t& cursor = obstacle_cursor[index];
      for (const int64_t iz : entry.second) {
        obstacle_column_cells[cursor++] = iz;
      }
    }
    for (const auto& key : observed_columns) {
      std::size_t index = 0;
      if (column_index(key, &index)) {
        observed_column_bits[index] = 1U;
      }
    }

    // The dense views are now the only query-time representation. Release the
    // scratch hash containers so the immutable model does not retain both.
    candidates_by_column.clear();
    candidates_by_column.rehash(0);
    obstacle_cells_by_column.clear();
    obstacle_cells_by_column.rehash(0);
    observed_columns.clear();
    observed_columns.rehash(0);
  }

  void clusterCandidates() {
    std::vector<uint8_t> visited(candidates.size(), 0);
    std::deque<int> queue;
    for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
      if (visited[seed] != 0) {
        continue;
      }
      std::vector<int> cluster;
      visited[seed] = 1;
      queue.push_back(static_cast<int>(seed));
      while (!queue.empty()) {
        const int current = queue.front();
        queue.pop_front();
        cluster.push_back(current);
        const Candidate& base = candidates[static_cast<std::size_t>(current)];
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            std::size_t column = 0;
            if (!columnViewIndex(ColumnKey{base.ix + dx, base.iy + dy},
                                 &column)) {
              continue;
            }
            for (std::size_t position = candidate_column_start[column];
                 position < candidate_column_start[column + 1U]; ++position) {
              const int neighbour = candidate_column_indices[position];
              if (visited[static_cast<std::size_t>(neighbour)] != 0) {
                continue;
              }
              if (std::abs(candidates[static_cast<std::size_t>(neighbour)]
                               .point.z() -
                           base.point.z()) >
                  config.cluster_max_dz + 1e-9) {
                continue;
              }
              visited[static_cast<std::size_t>(neighbour)] = 1;
              queue.push_back(neighbour);
            }
          }
        }
      }

      Patch patch;
      patch.candidates = std::move(cluster);
      std::vector<Vec3> points;
      points.reserve(patch.candidates.size());
      for (const int index : patch.candidates) {
        points.push_back(candidates[static_cast<std::size_t>(index)].point);
      }
      if (!planeFit(points, &patch.coefficients, &patch.rms,
                    &patch.confidence)) {
        // Tiny or collinear components cannot define a confident plane. Keep a
        // horizontal fallback with zero confidence so queries fail closed with
        // kLowConfidence instead of claiming confident support.
        double mean_z = 0.0;
        for (const auto& point : points) {
          mean_z += point.z();
        }
        mean_z /= static_cast<double>(points.size());
        patch.coefficients = Vec3(0.0, 0.0, mean_z);
        patch.rms = 0.0;
        patch.confidence = 0.0;
      }
      const Vec3 normal(-patch.coefficients.x(), -patch.coefficients.y(), 1.0);
      patch.normal = normal.normalized();
      const int patch_id = static_cast<int>(patches.size());
      for (const int index : patch.candidates) {
        candidates[static_cast<std::size_t>(index)].patch_id = patch_id;
      }
      patches.push_back(std::move(patch));
      if (patches.size() > config.max_patches) {
        build_status =
            Validation::failure("surface patch count exceeds max_patches");
        return;
      }
    }
  }

  bool freshObservation(double observation, double query_stamp) const {
    return finiteScalar(observation) &&
           std::abs(query_stamp - observation) <=
               config.max_observation_age + 1e-9;
  }

  // Support must be backed by a fresh measured HIT that no later MISS has
  // contradicted. This is deliberately independent of the log-odds occupancy:
  // a floor can still read occupied after the current rays have already cleared
  // it, and that stale floor must not support the vehicle.
  bool validActualHit(double hit_stamp, double miss_stamp,
                      double query_stamp) const {
    if (!freshObservation(hit_stamp, query_stamp)) {
      return false;
    }
    return !(finiteScalar(miss_stamp) && miss_stamp > hit_stamp + 1e-9);
  }

  bool validSupportObservation(const Candidate& candidate,
                               double query_stamp) const {
    return validActualHit(candidate.last_hit_stamp, candidate.last_miss_stamp,
                          query_stamp);
  }

  bool validLookupHit(const MapLookup& lookup, double query_stamp) const {
    return validActualHit(lookup.last_hit_stamp, lookup.last_miss_stamp,
                          query_stamp);
  }

  bool columnFor(double x, double y, ColumnKey* key) const {
    if (!snapshot || !finiteScalar(x) || !finiteScalar(y)) {
      return false;
    }
    const double voxel = snapshot->config().voxel_size;
    const int64_t ix = static_cast<int64_t>(std::floor(x / voxel));
    const int64_t iy = static_cast<int64_t>(std::floor(y / voxel));
    if (!insideMapColumn(ix, iy)) {
      return false;
    }
    *key = ColumnKey{ix, iy};
    return true;
  }

  bool patchInsideBounds(const Vec3& point) const {
    if (!snapshot) {
      return false;
    }
    const Vec3 lower = snapshot->originWorld();
    const Vec3 upper =
        lower + snapshot->windowDims().cast<double>() *
                    snapshot->config().voxel_size;
    const double radius = config.contact_patch_radius;
    return point.x() - radius >= lower.x() &&
           point.x() + radius <= upper.x() &&
           point.y() - radius >= lower.y() &&
           point.y() + radius <= upper.y();
  }

  double footprintCoverage(int candidate_index, double x, double y,
                           double query_stamp) const {
    if (candidate_index < 0 ||
        candidate_index >= static_cast<int>(candidates.size())) {
      return 0.0;
    }
    const Candidate& selected =
        candidates[static_cast<std::size_t>(candidate_index)];
    const int patch_id = selected.patch_id;
    const double voxel = snapshot->config().voxel_size;
    const double tolerance =
        std::max(2.0 * voxel, 2.0 * config.plane_inlier_tol);
    int supported = 0;
    int total = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        ++total;
        const double px =
            x + static_cast<double>(dx) * config.contact_patch_radius;
        const double py =
            y + static_cast<double>(dy) * config.contact_patch_radius;
        ColumnKey key;
        if (!columnFor(px, py, &key)) {
          continue;
        }
        std::size_t column = 0;
        if (!columnViewIndex(key, &column)) {
          continue;
        }
        bool found = false;
        for (std::size_t position = candidate_column_start[column];
             position < candidate_column_start[column + 1U]; ++position) {
          const int index = candidate_column_indices[position];
          const Candidate& candidate =
              candidates[static_cast<std::size_t>(index)];
          if (candidate.patch_id != patch_id) {
            continue;
          }
          // Coverage is evidence about this query time, not the global
          // snapshot stamp. A stale or currently-cleared layer cannot support
          // a fresh pose.
          if (!validSupportObservation(candidate, query_stamp)) {
            continue;
          }
          if (std::abs(candidate.point.z() -
                       candidateHeight(selected, px, py)) <= tolerance) {
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
    double height{kNaN};
    double coverage{0.0};
    double confidence{0.0};
  };

  SupportHit supportAt(double x, double y, double near_z, double query_stamp,
                       bool require_coverage,
                       bool require_confidence) const {
    SupportHit hit;
    ColumnKey key;
    if (!columnFor(x, y, &key)) {
      return hit;
    }
    std::size_t column = 0;
    if (!columnViewIndex(key, &column)) {
      return hit;
    }
    const std::size_t candidate_begin = candidate_column_start[column];
    const std::size_t candidate_end = candidate_column_start[column + 1U];
    if (candidate_begin == candidate_end) {
      hit.unknown_column = observed_column_bits[column] == 0U;
      return hit;
    }
    hit.saw_candidate = true;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t position = candidate_begin; position < candidate_end;
         ++position) {
      const int index = candidate_column_indices[position];
      const Candidate& candidate =
          candidates[static_cast<std::size_t>(index)];
      const double plane_z = supportHeight(candidate, x, y);
      if (plane_z < near_z - config.max_drop) {
        hit.lower_surface = true;
      }
      if (plane_z > near_z + config.max_drop) {
        hit.upper_surface = true;
      }
      if (std::abs(plane_z - near_z) > config.max_contact_search_z) {
        continue;
      }
      if (!validSupportObservation(candidate, query_stamp)) {
        hit.stale = true;
        continue;
      }
      const double coverage =
          require_coverage
              ? footprintCoverage(index, x, y, query_stamp)
              : 1.0;
      if (require_coverage && coverage < config.min_patch_coverage) {
        hit.coverage_failure = true;
        continue;
      }
      if (candidate.local_confidence < config.min_plane_confidence) {
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
    double z{kNaN};
  };

  RawSurface rawSurfaceAt(double x, double y, double near_z,
                          double query_stamp) const {
    RawSurface surface;
    ColumnKey key;
    if (!columnFor(x, y, &key)) {
      return surface;
    }
    surface.ix = key.x;
    surface.iy = key.y;
    std::size_t column = 0;
    if (!columnViewIndex(key, &column)) {
      return surface;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t position = candidate_column_start[column];
         position < candidate_column_start[column + 1U]; ++position) {
      const int index = candidate_column_indices[position];
      const Candidate& candidate =
          candidates[static_cast<std::size_t>(index)];
      const double distance = std::abs(candidate.point.z() - near_z);
      if (distance > config.max_contact_search_z) {
        continue;
      }
      if (!validSupportObservation(candidate, query_stamp)) {
        surface.stale = true;
        continue;
      }
      if (distance >= best_distance) {
        continue;
      }
      best_distance = distance;
      surface.found = true;
      surface.z = candidate.point.z();
    }
    return surface;
  }

  bool wheelPathContinuity(const std::array<Vec3, 4>& contacts,
                           const std::array<Vec3, 4>& supports,
                           double query_stamp, TerrainStatus* status,
                           std::string* detail) const {
    constexpr std::array<std::array<int, 2>, 4> edges{{
        {{0, 1}}, {{2, 3}}, {{0, 2}}, {{1, 3}}}};
    const double voxel = snapshot->config().voxel_size;
    // A discrete axle step is a large change between adjacent columns. A
    // continuous ramp with the same net wheel height gain changes by only a
    // fraction of a voxel per sample. Use at least one sample per half voxel.
    const double threshold = std::max(config.max_step, voxel);
    for (const auto& edge : edges) {
      const int a = edge[0];
      const int b = edge[1];
      const Vec3 start(contacts[static_cast<std::size_t>(a)].x(),
                       contacts[static_cast<std::size_t>(a)].y(), 0.0);
      const Vec3 end(contacts[static_cast<std::size_t>(b)].x(),
                     contacts[static_cast<std::size_t>(b)].y(), 0.0);
      const double z_start = supports[static_cast<std::size_t>(a)].z();
      const double z_end = supports[static_cast<std::size_t>(b)].z();
      const double length = (end - start).norm();
      if (!finiteScalar(length) || length < 1e-9) {
        continue;
      }
      const int steps =
          std::max(2, static_cast<int>(std::ceil(length / (0.5 * voxel))));
      ColumnKey previous_column{};
      double previous_z = 0.0;
      bool have_previous = false;
      for (int step = 0; step <= steps; ++step) {
        const double t = static_cast<double>(step) /
                         static_cast<double>(steps);
        const Vec3 xy = start + (end - start) * t;
        const double expected_z = z_start + (z_end - z_start) * t;
        const RawSurface raw =
            rawSurfaceAt(xy.x(), xy.y(), expected_z, query_stamp);
        if (!raw.found) {
          if (raw.stale) {
            *status = TerrainStatus::kStale;
            *detail = "wheel-path support layer is stale";
          } else {
            *status = TerrainStatus::kNoSupport;
            *detail =
                "wheel-path support is missing (narrow dip or unobserved gap)";
          }
          return false;
        }
        const ColumnKey column{raw.ix, raw.iy};
        if (have_previous && column.x == previous_column.x &&
            column.y == previous_column.y) {
          continue;
        }
        if (have_previous && std::abs(raw.z - previous_z) > threshold + 1e-9) {
          *status = TerrainStatus::kStep;
          *detail =
              "wheel-path surface height jumps across adjacent columns";
          return false;
        }
        previous_column = column;
        previous_z = raw.z;
        have_previous = true;
      }
    }
    return true;
  }

  static Vec3 normalFromCoefficients(const Vec3& coefficients) {
    Vec3 normal(-coefficients.x(), -coefficients.y(), 1.0);
    normal.normalize();
    return normal;
  }

  static Quat orientationFromNormalYaw(const Vec3& normal, double yaw) {
    Vec3 heading(std::cos(yaw), std::sin(yaw), 0.0);
    Vec3 forward = heading - normal * heading.dot(normal);
    if (forward.norm() < 1e-9) {
      heading = Vec3(-std::sin(yaw), std::cos(yaw), 0.0);
      forward = heading - normal * heading.dot(normal);
    }
    if (forward.norm() < 1e-9) {
      return Quat::Identity();
    }
    forward.normalize();
    Vec3 left = normal.cross(forward);
    if (left.norm() < 1e-9) {
      return Quat::Identity();
    }
    left.normalize();
    Vec3 up = forward.cross(left);
    up.normalize();
    Mat3 rotation;
    rotation.col(0) = forward;
    rotation.col(1) = left;
    rotation.col(2) = up;
    Quat orientation(rotation);
    orientation.normalize();
    return orientation;
  }

  Vec3 basePositionOnPlane(const Vec3& coefficients, double x, double y,
                           double base_height) const {
    const Vec3 normal = normalFromCoefficients(coefficients);
    const double norm =
        std::sqrt(coefficients.x() * coefficients.x() +
                  coefficients.y() * coefficients.y() + 1.0);
    const double plane_d = coefficients.z() / norm;
    return Vec3(x, y,
                (plane_d + base_height - normal.x() * x - normal.y() * y) /
                    normal.z());
  }

  TerrainResult reject(TerrainStatus status, std::string detail) const {
    TerrainResult result;
    result.status = status;
    result.detail = std::move(detail);
    return result;
  }
};

TerrainModel::TerrainModel(std::shared_ptr<const MapSnapshot> snapshot,
                           TerrainConfig config, VehicleGeometry geometry)
    : impl_(std::make_unique<Impl>()) {
  impl_->snapshot = std::move(snapshot);
  impl_->config = config;
  impl_->geometry = geometry;
  impl_->geometry_status = geometry.validate();
  impl_->config_status = config.validate();
  if (!impl_->snapshot) {
    impl_->build_status = Validation::failure("snapshot is null");
    return;
  }
  if (!impl_->config_status.ok || !impl_->geometry_status.ok) {
    return;
  }
  impl_->extractCandidates();
  if (!impl_->build_status.ok) {
    return;
  }
  // Freeze the origin-relative dense views before any dependent build step or
  // query. Candidates, observed bits, and obstacle cells are immutable from
  // here on.
  impl_->freezeColumnViews();
  impl_->computeLocalPlanes();
  impl_->clusterCandidates();
}

TerrainModel::~TerrainModel() = default;
TerrainModel::TerrainModel(TerrainModel&&) noexcept = default;
TerrainModel& TerrainModel::operator=(TerrainModel&&) noexcept = default;

std::size_t TerrainModel::patchCount() const { return impl_->patches.size(); }

std::size_t TerrainModel::patchCellCount(int patch_id) const {
  if (patch_id < 0 || patch_id >= static_cast<int>(impl_->patches.size())) {
    return 0;
  }
  return impl_->patches[static_cast<std::size_t>(patch_id)].candidates.size();
}

int TerrainModel::patchAtColumn(double x, double y, double near_z) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(near_z) ||
      !impl_->snapshot) {
    return -1;
  }
  const double voxel = impl_->snapshot->config().voxel_size;
  const int64_t ix = static_cast<int64_t>(std::floor(x / voxel));
  const int64_t iy = static_cast<int64_t>(std::floor(y / voxel));
  std::size_t column = 0;
  if (!impl_->columnViewIndex(ColumnKey{ix, iy}, &column)) {
    return -1;
  }
  int best = -1;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t position = impl_->candidate_column_start[column];
       position < impl_->candidate_column_start[column + 1U]; ++position) {
    const int index = impl_->candidate_column_indices[position];
    const auto& candidate = impl_->candidates[static_cast<std::size_t>(index)];
    const double distance =
        std::abs(impl_->candidateHeight(candidate, x, y) - near_z);
    if (distance < best_distance) {
      best_distance = distance;
      best = candidate.patch_id;
    }
  }
  return best;
}

const Validation& TerrainModel::buildStatus() const {
  return impl_->build_status;
}

TerrainResult TerrainModel::evaluate(const TerrainQuery& query) const {
  if (!impl_->geometry_status.ok) {
    return impl_->reject(TerrainStatus::kInvalidGeometry,
                         impl_->geometry_status.error);
  }
  if (!impl_->config_status.ok) {
    return impl_->reject(TerrainStatus::kInvalidConfig,
                         impl_->config_status.error);
  }
  if (!impl_->build_status.ok) {
    return impl_->reject(TerrainStatus::kBuildFailure,
                         impl_->build_status.error);
  }
  if (!impl_->snapshot) {
    return impl_->reject(TerrainStatus::kBuildFailure, "snapshot is null");
  }
  if (!query.position.allFinite() || !finiteScalar(query.stamp) ||
      !finiteScalar(query.roll) || !finiteScalar(query.pitch) ||
      !finiteScalar(query.yaw)) {
    return impl_->reject(TerrainStatus::kInvalidQuery,
                         "query pose/stamp must be finite");
  }
  const auto& snapshot = *impl_->snapshot;
  if (!finiteScalar(snapshot.stamp())) {
    return impl_->reject(TerrainStatus::kStale,
                         "snapshot has no valid map timestamp");
  }
  // Snapshot freshness and per-voxel freshness are independent. Unrelated rays
  // may refresh the global stamp while a support or body cell remains old.
  if (std::abs(query.stamp - snapshot.stamp()) >
      impl_->config.max_snapshot_age) {
    return impl_->reject(TerrainStatus::kStale,
                         "snapshot age exceeds max_snapshot_age");
  }
  if (!snapshot.inBounds(query.position)) {
    return impl_->reject(TerrainStatus::kOutside,
                         "predicted base position is outside the map window");
  }

  const Mat3 predicted_rotation =
      rotationFromRpy(query.roll, query.pitch, query.yaw);
  std::array<Vec3, 4> contact_points{};
  std::array<Vec3, 4> contact_support{};
  std::array<bool, 4> contact_found{false, false, false, false};
  std::array<int, 4> contact_patches{-1, -1, -1, -1};
  std::array<double, 4> contact_coverage{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> contact_confidence{0.0, 0.0, 0.0, 0.0};
  bool saw_patch = false;
  bool saw_stale = false;
  bool saw_unknown_support = false;
  bool saw_coverage_failure = false;
  bool saw_confidence_failure = false;
  bool saw_lower_surface = false;
  bool saw_upper_surface = false;
  std::string stale_detail;
  std::string unknown_support_detail;
  std::string coverage_failure_detail;
  std::string confidence_failure_detail;

  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    const Vec3 contact =
        query.position +
        predicted_rotation * impl_->geometry.wheel_contacts_base[wheel];
    contact_points[wheel] = contact;
    contact_support[wheel] = contact;
    if (!snapshot.inBounds(contact) || !impl_->patchInsideBounds(contact)) {
      return impl_->reject(
          TerrainStatus::kOutside,
          "a wheel contact patch is outside the map window");
    }
    const auto hit = impl_->supportAt(contact.x(), contact.y(), contact.z(),
                                      query.stamp, true, false);
    if (hit.found) {
      contact_found[wheel] = true;
      contact_patches[wheel] = hit.patch_id;
      contact_support[wheel] =
          Vec3(contact.x(), contact.y(), hit.height);
      contact_coverage[wheel] = hit.coverage;
      contact_confidence[wheel] = hit.confidence;
      if (hit.confidence < impl_->config.min_plane_confidence) {
        saw_confidence_failure = true;
        confidence_failure_detail =
            "wheel " + std::to_string(wheel) + " local plane confidence " +
            std::to_string(hit.confidence) + " < " +
            std::to_string(impl_->config.min_plane_confidence);
      }
    } else {
      saw_patch = saw_patch || hit.saw_candidate;
      saw_stale = saw_stale || hit.stale;
      saw_unknown_support = saw_unknown_support || hit.unknown_column;
      saw_coverage_failure = saw_coverage_failure || hit.coverage_failure;
      if (hit.unknown_column) {
        unknown_support_detail =
            "wheel " + std::to_string(wheel) +
            " contact column is unobserved";
      }
      if (hit.confidence_failure) {
        saw_confidence_failure = true;
        confidence_failure_detail =
            "wheel " + std::to_string(wheel) +
            " has insufficient local plane confidence";
      }
      saw_lower_surface = saw_lower_surface || hit.lower_surface;
      saw_upper_surface = saw_upper_surface || hit.upper_surface;
      if (hit.stale) {
        stale_detail =
            "wheel " + std::to_string(wheel) +
            " support observation is older than max_observation_age";
      }
      if (hit.coverage_failure) {
        coverage_failure_detail =
            "wheel " + std::to_string(wheel) +
            " fresh contact patch coverage is insufficient";
      }
    }
  }
  if (std::any_of(contact_found.begin(), contact_found.end(),
                  [](bool value) { return !value; })) {
    if (saw_stale) {
      return impl_->reject(TerrainStatus::kStale, stale_detail);
    }
    if (saw_unknown_support) {
      return impl_->reject(TerrainStatus::kUnknownSupport,
                           unknown_support_detail);
    }
    if (saw_coverage_failure) {
      return impl_->reject(TerrainStatus::kInsufficientCoverage,
                           coverage_failure_detail);
    }
    if (saw_confidence_failure) {
      return impl_->reject(TerrainStatus::kLowConfidence,
                           confidence_failure_detail);
    }
    if (saw_lower_surface && saw_upper_surface) {
      return impl_->reject(
          TerrainStatus::kNoSupport,
          "predicted layer is bracketed by disconnected surfaces");
    }
    if (saw_lower_surface) {
      return impl_->reject(TerrainStatus::kDropoff,
                           "support exists but is below max_drop");
    }
    if (saw_patch) {
      return impl_->reject(TerrainStatus::kNoSupport,
                           "no support plane is near the predicted height");
    }
    return impl_->reject(TerrainStatus::kNoSupport,
                         "no surface candidate under a wheel");
  }

  // Detect a discrete axle step or missing narrow dip before trusting a
  // four-wheel plane: four perfect wheel heights can still hide a jump between
  // them. A continuous ramp with the same net gain changes only gradually.
  TerrainStatus continuity_status = TerrainStatus::kOk;
  std::string continuity_detail;
  if (!impl_->wheelPathContinuity(contact_points, contact_support,
                                  query.stamp, &continuity_status,
                                  &continuity_detail)) {
    return impl_->reject(continuity_status, continuity_detail);
  }
  if (saw_confidence_failure) {
    return impl_->reject(TerrainStatus::kLowConfidence,
                         confidence_failure_detail);
  }

  Vec3 coefficients = Vec3::Zero();
  double rms = 0.0;
  double plane_confidence = 0.0;
  std::vector<Vec3> contact_xyz;
  contact_xyz.reserve(4);
  for (const auto& point : contact_support) {
    contact_xyz.push_back(point);
  }
  if (!planeFit(contact_xyz, &coefficients, &rms, &plane_confidence)) {
    return impl_->reject(TerrainStatus::kNoSupport,
                         "four-wheel support samples are rank deficient");
  }
  {
    Vec3 normal(-coefficients.x(), -coefficients.y(), 1.0);
    if (!normal.allFinite() || normal.norm() < 1e-9) {
      return impl_->reject(TerrainStatus::kNoSupport,
                           "degenerate four-wheel support plane");
    }
  }

  double max_residual = 0.0;
  for (const auto& point : contact_support) {
    const double predicted = coefficients.x() * point.x() +
                             coefficients.y() * point.y() + coefficients.z();
    max_residual = std::max(max_residual, std::abs(point.z() - predicted));
  }
  if (max_residual > impl_->config.max_step) {
    return impl_->reject(TerrainStatus::kStep,
                         "wheel support heights are non-planar (step)");
  }
  if (rms > impl_->config.max_warp) {
    return impl_->reject(TerrainStatus::kWarp,
                         "wheel support plane residual exceeds max_warp");
  }

  const auto check_slopes = [&](const Vec3& plane) -> TerrainResult {
    const Vec3 normal(-plane.x(), -plane.y(), 1.0);
    const Vec3 unit_normal = normal.normalized();
    const double slope = std::acos(std::clamp(unit_normal.z(), -1.0, 1.0));
    if (slope > impl_->config.max_slope_rad) {
      return impl_->reject(TerrainStatus::kSlope,
                           "support slope exceeds max_slope_rad");
    }
    const Mat3 yaw_rotation =
        Eigen::AngleAxisd(query.yaw, Vec3::UnitZ()).toRotationMatrix();
    const Vec3 body_normal = yaw_rotation.transpose() * unit_normal;
    const double cross_slope =
        std::asin(std::clamp(-body_normal.y(), -1.0, 1.0));
    if (std::abs(cross_slope) > impl_->config.max_cross_slope_rad) {
      return impl_->reject(TerrainStatus::kCrossSlope,
                           "cross slope exceeds max_cross_slope_rad");
    }
    return TerrainResult{};
  };
  {
    const TerrainResult slope_result = check_slopes(coefficients);
    if (!slope_result.ok() && slope_result.status != TerrainStatus::kOk) {
      // TerrainResult{} defaults to kBuildFailure, so only reject a real
      // slope/cross-slope failure here.
      if (slope_result.status == TerrainStatus::kSlope ||
          slope_result.status == TerrainStatus::kCrossSlope) {
        return slope_result;
      }
    }
  }

  const double base_height = impl_->geometry.base_contact_height;
  // Cached local planes are fitted to measured hit centroids. Keep a small
  // tolerance tied to the plane inlier scale for mixed smooth slopes, and still
  // verify the final contacts against cached support below.
  const double pose_tolerance =
      std::max(0.005, 0.25 * impl_->config.plane_inlier_tol);
  const double normal_tolerance =
      std::max(0.002, 0.25 * impl_->config.plane_inlier_tol);
  const double contact_tolerance =
      std::max(0.02, 2.0 * pose_tolerance);
  Vec3 projected_position = Vec3::Zero();
  Quat projected_orientation = Quat::Identity();
  Mat3 projection = Mat3::Identity();
  std::array<Vec3, 4> final_contacts{};
  std::array<Vec3, 4> final_supports{};
  std::array<WheelSupport, 4> final_wheels{};
  bool converged = false;

  const auto describe_miss = [&](const Impl::SupportHit& hit,
                                 std::size_t wheel) -> TerrainResult {
    if (hit.stale) {
      return impl_->reject(
          TerrainStatus::kStale,
          "projected wheel " + std::to_string(wheel) +
              " support observation is stale");
    }
    if (hit.unknown_column) {
      return impl_->reject(
          TerrainStatus::kUnknownSupport,
          "projected wheel " + std::to_string(wheel) +
              " contact column is unobserved");
    }
    if (hit.coverage_failure) {
      return impl_->reject(
          TerrainStatus::kInsufficientCoverage,
          "projected wheel " + std::to_string(wheel) +
              " fresh contact patch coverage is insufficient");
    }
    if (hit.confidence_failure) {
      return impl_->reject(
          TerrainStatus::kLowConfidence,
          "projected wheel " + std::to_string(wheel) +
              " local plane confidence is insufficient");
    }
    if (hit.lower_surface) {
      return impl_->reject(TerrainStatus::kDropoff,
                           "projected support is below max_drop");
    }
    return impl_->reject(TerrainStatus::kNoSupport,
                         "projected wheel contact has no cached support");
  };

  for (int iteration = 0; iteration < 3; ++iteration) {
    const Vec3 normal = Impl::normalFromCoefficients(coefficients);
    const Quat orientation =
        Impl::orientationFromNormalYaw(normal, query.yaw);
    const Mat3 rotation = orientation.toRotationMatrix();
    const Vec3 pose = impl_->basePositionOnPlane(
        coefficients, query.position.x(), query.position.y(), base_height);

    std::array<Vec3, 4> trial_contacts{};
    std::array<Vec3, 4> trial_supports{};
    std::array<WheelSupport, 4> trial_wheels{};
    for (std::size_t wheel = 0; wheel < 4; ++wheel) {
      const Vec3 contact =
          pose + rotation * impl_->geometry.wheel_contacts_base[wheel];
      trial_contacts[wheel] = contact;
      if (!snapshot.inBounds(contact) || !impl_->patchInsideBounds(contact)) {
        return impl_->reject(
            TerrainStatus::kOutside,
            "a projected wheel contact patch is outside the map window");
      }
      const auto hit = impl_->supportAt(contact.x(), contact.y(), contact.z(),
                                        query.stamp, true, true);
      if (!hit.found) {
        return describe_miss(hit, wheel);
      }
      trial_supports[wheel] = Vec3(contact.x(), contact.y(), hit.height);
      trial_wheels[wheel].supported = true;
      trial_wheels[wheel].predicted_z = contact.z();
      trial_wheels[wheel].support_z = hit.height;
      trial_wheels[wheel].coverage = hit.coverage;
      trial_wheels[wheel].patch_id = hit.patch_id;
    }

    Vec3 next_coefficients = Vec3::Zero();
    double next_rms = 0.0;
    double next_confidence = 0.0;
    std::vector<Vec3> trial_points(trial_supports.begin(),
                                   trial_supports.end());
    if (!planeFit(trial_points, &next_coefficients, &next_rms,
                  &next_confidence)) {
      return impl_->reject(
          TerrainStatus::kNoSupport,
          "projected contact supports are rank deficient");
    }
    double max_delta = 0.0;
    for (std::size_t wheel = 0; wheel < 4; ++wheel) {
      max_delta = std::max(
          max_delta,
          std::abs(trial_contacts[wheel].z() - trial_supports[wheel].z()));
    }
    const Vec3 old_normal = Impl::normalFromCoefficients(coefficients);
    const Vec3 new_normal = Impl::normalFromCoefficients(next_coefficients);
    const double normal_angle =
        std::acos(std::clamp(old_normal.dot(new_normal), -1.0, 1.0));

    coefficients = next_coefficients;
    projected_position = pose;
    projected_orientation = orientation;
    projection = rotation;
    final_contacts = trial_contacts;
    final_supports = trial_supports;
    final_wheels = trial_wheels;
    if (max_delta <= pose_tolerance &&
        normal_angle <= normal_tolerance) {
      converged = true;
      break;
    }
  }
  if (!converged) {
    return impl_->reject(
        TerrainStatus::kNoSupport,
        "projected contact supports did not converge within 3 iterations");
  }

  // Recompute the final pose from the converged plane and verify every
  // projected contact against cached fresh support. This catches a rotation
  // that moved a wheel onto a different local support layer.
  {
    const Vec3 normal = Impl::normalFromCoefficients(coefficients);
    const Quat orientation =
        Impl::orientationFromNormalYaw(normal, query.yaw);
    const Mat3 rotation = orientation.toRotationMatrix();
    const Vec3 pose = impl_->basePositionOnPlane(
        coefficients, query.position.x(), query.position.y(), base_height);
    std::array<WheelSupport, 4> verified_wheels{};
    for (std::size_t wheel = 0; wheel < 4; ++wheel) {
      const Vec3 contact =
          pose + rotation * impl_->geometry.wheel_contacts_base[wheel];
      if (!snapshot.inBounds(contact) || !impl_->patchInsideBounds(contact)) {
        return impl_->reject(
            TerrainStatus::kOutside,
            "a projected wheel contact patch is outside the map window");
      }
      const auto hit = impl_->supportAt(contact.x(), contact.y(), contact.z(),
                                        query.stamp, true, true);
      if (!hit.found) {
        return describe_miss(hit, wheel);
      }
      if (std::abs(hit.height - contact.z()) > contact_tolerance) {
        return impl_->reject(
            TerrainStatus::kNoSupport,
            "projected contact does not lie on cached fresh support");
      }
      verified_wheels[wheel].supported = true;
      verified_wheels[wheel].predicted_z = contact.z();
      verified_wheels[wheel].support_z = hit.height;
      verified_wheels[wheel].coverage = hit.coverage;
      verified_wheels[wheel].patch_id = hit.patch_id;
      final_contacts[wheel] = contact;
      final_supports[wheel] = Vec3(contact.x(), contact.y(), hit.height);
    }
    projected_position = pose;
    projected_orientation = orientation;
    projection = rotation;
    final_wheels = verified_wheels;
  }

  const TerrainResult final_slope_result = check_slopes(coefficients);
  if (final_slope_result.status == TerrainStatus::kSlope ||
      final_slope_result.status == TerrainStatus::kCrossSlope) {
    return final_slope_result;
  }

  const Vec3 body_center_local =
      0.5 * (impl_->geometry.body_min + impl_->geometry.body_max);
  const Vec3 body_half =
      0.5 * (impl_->geometry.body_max - impl_->geometry.body_min);
  const Vec3 body_center =
      projected_position + projection * body_center_local;

  const Vec3 map_lower = snapshot.originWorld();
  const Vec3 map_upper =
      map_lower + snapshot.windowDims().cast<double>() * snapshot.config().voxel_size;
  const Vec3 aabb_half = projection.cwiseAbs() * body_half;
  const Vec3 body_aabb_min = body_center - aabb_half;
  const Vec3 body_aabb_max = body_center + aabb_half;
  if ((body_aabb_min.array() < map_lower.array()).any() ||
      (body_aabb_max.array() >= map_upper.array()).any()) {
    return impl_->reject(
        TerrainStatus::kOutside,
        "full body OBB extends outside the map window; it is not clipped");
  }

  double clearance = std::numeric_limits<double>::infinity();
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const Vec3 local =
            body_center_local +
            Vec3(sx * body_half.x(), sy * body_half.y(), sz * body_half.z());
        const Vec3 corner = projected_position + projection * local;
        const double local_support =
            coefficients.x() * corner.x() +
            coefficients.y() * corner.y() + coefficients.z();
        clearance = std::min(clearance, corner.z() - local_support);
      }
    }
  }
  if (clearance < impl_->geometry.body_clearance - 1e-9) {
    TerrainResult result = impl_->reject(
        TerrainStatus::kInsufficientBodyClearance,
        "body clearance " + std::to_string(clearance) +
            " m below configured body_clearance " +
            std::to_string(impl_->geometry.body_clearance) + " m");
    result.clearance = clearance;
    return result;
  }

  const double voxel = snapshot.config().voxel_size;
  const auto origin = snapshot.originVoxel();
  const auto dims = snapshot.windowDims();
  int64_t ix0 =
      static_cast<int64_t>(std::floor(body_aabb_min.x() / voxel));
  int64_t iy0 =
      static_cast<int64_t>(std::floor(body_aabb_min.y() / voxel));
  int64_t iz0 =
      static_cast<int64_t>(std::floor(body_aabb_min.z() / voxel));
  int64_t ix1 =
      static_cast<int64_t>(std::floor(body_aabb_max.x() / voxel));
  int64_t iy1 =
      static_cast<int64_t>(std::floor(body_aabb_max.y() / voxel));
  int64_t iz1 =
      static_cast<int64_t>(std::floor(body_aabb_max.z() / voxel));
  ix0 = std::max(ix0, origin[0]);
  iy0 = std::max(iy0, origin[1]);
  iz0 = std::max(iz0, origin[2]);
  ix1 = std::min(ix1, origin[0] + dims.x() - 1);
  iy1 = std::min(iy1, origin[1] + dims.y() - 1);
  iz1 = std::min(iz1, origin[2] + dims.z() - 1);

  bool body_collision = false;
  bool unknown_body = false;
  bool stale_body = false;
  // A floor return may be excluded from collision only when the highest real
  // return in the whole voxel is provably below the body. Bound the allowance
  // by half the available body-to-ground clearance, and never exceed the
  // plane inlier tolerance. This prevents a low centroid from hiding a higher
  // point that intrudes into the body.
  const double available_body_clearance =
      std::min(impl_->geometry.body_clearance, clearance);
  const double floor_tolerance =
      std::max(0.0, std::min(0.5 * available_body_clearance,
                            impl_->config.plane_inlier_tol));
  for (int64_t ix = ix0; ix <= ix1 && !body_collision; ++ix) {
    for (int64_t iy = iy0; iy <= iy1 && !body_collision; ++iy) {
      for (int64_t iz = iz0; iz <= iz1; ++iz) {
        // Final collision is against RAW occupancy. Inflation is only a
        // conservative broadphase helper and must never make a flat occupied
        // floor collide with a body that has real physical clearance.
        const auto lookup = snapshot.lookupVoxel(ix, iy, iz, false);
        if (!lookup.in_bounds) {
          return impl_->reject(
              TerrainStatus::kOutside,
              "body collision search left the map window");
        }
        // Cheap evidence tests first: a fresh observed truly-free cell with no
        // current actual HIT cannot be a collision or an unknown-body cell, so
        // no OBB/floor geometry is needed. Stale cells are still reported as
        // stale, and unobserved/unknown cells still apply the configured
        // unknown-body policy.
        if (!lookup.observed) {
          if (impl_->config.unknown_body_policy == UnknownBodyPolicy::kReject) {
            const Vec3 cell_min(static_cast<double>(ix) * voxel,
                                static_cast<double>(iy) * voxel,
                                static_cast<double>(iz) * voxel);
            const Vec3 cell_max = cell_min + Vec3::Constant(voxel);
            const bool overlaps = obbIntersectsAabb(
                body_center, body_half, projection, cell_min, cell_max);
            if (overlaps) {
              unknown_body = true;
            }
          }
          continue;
        }
        if (!impl_->freshObservation(lookup.last_observed_stamp,
                                     query.stamp)) {
          stale_body = true;
          continue;
        }
        const bool actual_hit = impl_->validLookupHit(lookup, query.stamp);
        const Occupancy occupancy = lookup.occupancy;
        if (!actual_hit && occupancy == Occupancy::kFree) {
          continue;
        }
        if (!actual_hit && occupancy == Occupancy::kUnknown) {
          if (impl_->config.unknown_body_policy == UnknownBodyPolicy::kReject) {
            const Vec3 cell_min(static_cast<double>(ix) * voxel,
                                static_cast<double>(iy) * voxel,
                                static_cast<double>(iz) * voxel);
            const Vec3 cell_max = cell_min + Vec3::Constant(voxel);
            const bool overlaps = obbIntersectsAabb(
                body_center, body_half, projection, cell_min, cell_max);
            if (overlaps) {
              unknown_body = true;
            }
          }
          continue;
        }
        // Occupied voxel volumes remain the conservative obstacle geometry
        // for non-floor obstacles. A floor voxel may be excluded only when a
        // fresh, uncontradicted actual HIT exists and the highest actual HIT
        // in the whole voxel is below the conservative local support plane
        // across the entire voxel XY footprint. The coarse whole-cell top
        // comparison is deliberately gone: it could delete a real obstacle
        // return merely because the low voxel volume was near the floor.
        const Vec3 cell_min(static_cast<double>(ix) * voxel,
                            static_cast<double>(iy) * voxel,
                            static_cast<double>(iz) * voxel);
        const Vec3 cell_max = cell_min + Vec3::Constant(voxel);
        if (actual_hit) {
          const double x0 = cell_min.x();
          const double x1 = cell_max.x();
          const double y0 = cell_min.y();
          const double y1 = cell_max.y();
          const double support_at_cell =
              std::min(std::min(coefficients.x() * x0 + coefficients.y() * y0,
                                coefficients.x() * x0 + coefficients.y() * y1),
                       std::min(coefficients.x() * x1 + coefficients.y() * y0,
                                coefficients.x() * x1 + coefficients.y() * y1)) +
              coefficients.z();
          if (std::isfinite(lookup.hit_max_z) &&
              lookup.hit_max_z <= support_at_cell + floor_tolerance) {
            // The highest measured point in this voxel is proven to be on the
            // support layer and below the body.
            continue;
          }
        }
        const bool overlaps = obbIntersectsAabb(
            body_center, body_half, projection, cell_min, cell_max);
        // A current measured HIT is hard geometry even when previous free
        // evidence means the log-odds only rises to unknown. It must block the
        // body regardless of unknown_body_policy.
        if (overlaps && actual_hit) {
          body_collision = true;
          break;
        }
        if (occupancy == Occupancy::kOccupied) {
          if (overlaps) {
            body_collision = true;
            break;
          }
        } else if (occupancy == Occupancy::kUnknown) {
          if (overlaps &&
              impl_->config.unknown_body_policy == UnknownBodyPolicy::kReject) {
            unknown_body = true;
          }
        }
      }
    }
  }
  if (body_collision) {
    TerrainResult result =
        impl_->reject(TerrainStatus::kBodyCollision,
                      "rigid body OBB intersects a raw occupied voxel volume");
    result.clearance = clearance;
    return result;
  }
  if (stale_body) {
    TerrainResult result = impl_->reject(
        TerrainStatus::kStale,
        "body volume contains observations older than max_observation_age");
    result.clearance = clearance;
    return result;
  }
  if (unknown_body) {
    TerrainResult result = impl_->reject(
        TerrainStatus::kUnknownBody,
        "body OBB intersects unobserved voxel volumes and policy is reject");
    result.clearance = clearance;
    return result;
  }

  // Separate bounded 3-D raw-occupancy clearance for soft obstacle costs.
  // Support-floor voxels are excluded here; ground clearance above owns them.
  // If the bounded search volume cannot fit in the map, fail closed rather
  // than silently clipping the obstacle search.
  const double search_margin = impl_->config.obstacle_search_margin;
  const Vec3 search_min = body_aabb_min - Vec3::Constant(search_margin);
  const Vec3 search_max = body_aabb_max + Vec3::Constant(search_margin);
  if ((search_min.array() < map_lower.array()).any() ||
      (search_max.array() >= map_upper.array()).any()) {
    return impl_->reject(
        TerrainStatus::kOutside,
        "bounded obstacle-clearance search volume leaves the map window");
  }
  int64_t ox0 = std::max(
      static_cast<int64_t>(std::floor(search_min.x() / voxel)), origin[0]);
  int64_t oy0 = std::max(
      static_cast<int64_t>(std::floor(search_min.y() / voxel)), origin[1]);
  int64_t oz0 = std::max(
      static_cast<int64_t>(std::floor(search_min.z() / voxel)), origin[2]);
  int64_t ox1 = std::min(
      static_cast<int64_t>(std::floor(search_max.x() / voxel)),
      origin[0] + dims.x() - 1);
  int64_t oy1 = std::min(
      static_cast<int64_t>(std::floor(search_max.y() / voxel)),
      origin[1] + dims.y() - 1);
  int64_t oz1 = std::min(
      static_cast<int64_t>(std::floor(search_max.z() / voxel)),
      origin[2] + dims.z() - 1);
  double obstacle_clearance = std::numeric_limits<double>::infinity();
  for (int64_t ix = ox0; ix <= ox1; ++ix) {
    for (int64_t iy = oy0; iy <= oy1; ++iy) {
      std::size_t column = 0;
      if (!impl_->columnViewIndex(ColumnKey{ix, iy}, &column)) {
        continue;
      }
      for (std::size_t position = impl_->obstacle_column_start[column];
           position < impl_->obstacle_column_start[column + 1U]; ++position) {
        const int64_t iz = impl_->obstacle_column_cells[position];
        if (iz < oz0 || iz > oz1) {
          continue;
        }
        const auto lookup = snapshot.lookupVoxel(ix, iy, iz, false);
        const bool actual_hit =
            impl_->validLookupHit(lookup, query.stamp);
        // A current measured HIT is an obstacle even when historical free
        // evidence leaves the log-odds unknown. Conversely a floor voxel is
        // excluded only when its highest fresh actual HIT is below the local
        // plane by the same conservative floor tolerance used for the body.
        if (!actual_hit && lookup.occupancy != Occupancy::kOccupied) {
          continue;
        }
        const Vec3 cell_min(static_cast<double>(ix) * voxel,
                            static_cast<double>(iy) * voxel,
                            static_cast<double>(iz) * voxel);
        const Vec3 cell_max = cell_min + Vec3::Constant(voxel);
        const double x0 = cell_min.x();
        const double x1 = cell_max.x();
        const double y0 = cell_min.y();
        const double y1 = cell_max.y();
        const double support_at_cell =
            std::min(std::min(coefficients.x() * x0 + coefficients.y() * y0,
                              coefficients.x() * x0 + coefficients.y() * y1),
                     std::min(coefficients.x() * x1 + coefficients.y() * y0,
                              coefficients.x() * x1 + coefficients.y() * y1)) +
            coefficients.z();
        if (actual_hit && std::isfinite(lookup.hit_max_z) &&
            lookup.hit_max_z <= support_at_cell + floor_tolerance) {
          // This voxel is part of the support floor/layer, not an intruding
          // lateral or overhead obstacle.
          continue;
        }
        const double distance = obbAabbDistance(
            body_center, body_half, projection, cell_min, cell_max);
        obstacle_clearance = std::min(obstacle_clearance, distance);
      }
    }
  }

  int selected_patch = -1;
  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    if (selected_patch < 0) {
      selected_patch = final_wheels[wheel].patch_id;
    }
  }

  TerrainResult result;
  result.status = TerrainStatus::kOk;
  result.projected_position = projected_position;
  result.projected_orientation = projected_orientation;
  const Mat3 rotation = projected_orientation.toRotationMatrix();
  result.roll = std::atan2(rotation(2, 1), rotation(2, 2));
  result.pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  result.yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  result.clearance = clearance;
  result.obstacle_clearance = obstacle_clearance;
  result.selected_patch = selected_patch;
  result.wheels = final_wheels;
  for (std::size_t wheel = 0; wheel < 4; ++wheel) {
    if (!result.wheels[wheel].supported) {
      result.wheels[wheel].predicted_z = final_contacts[wheel].z();
      result.wheels[wheel].support_z = final_supports[wheel].z();
    }
  }
  return result;
}

}  // namespace dddmr_local_map
