#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "dddmr_local_map/types.hpp"
#include "dddmr_local_map/voxel_map.hpp"

namespace dddmr_local_map {

// Contact offsets are expressed in the base frame. z is normally
// -base_contact_height. These values are configuration inputs, not measured
// vehicle dimensions. The four-wheel model is a geometric plane fit, not a
// suspension simulation.
//
// Contact order is front-left, front-right, rear-left, rear-right. Validation
// requires four distinct, non-collinear x/y points, a positive track on each
// axle (contacts 0-1 and 2-3) and a positive wheelbase between the mean front
// and rear x coordinates.
struct VehicleGeometry {
  std::array<Vec3, 4> wheel_contacts_base{
      Vec3{kNaN, kNaN, kNaN}, Vec3{kNaN, kNaN, kNaN},
      Vec3{kNaN, kNaN, kNaN}, Vec3{kNaN, kNaN, kNaN}};
  double base_contact_height{kNaN};
  Vec3 body_min{Vec3{kNaN, kNaN, kNaN}};
  Vec3 body_max{Vec3{kNaN, kNaN, kNaN}};
  double body_clearance{kNaN};

  Validation validate() const;
};

enum class UnknownBodyPolicy : uint8_t { kReject = 0, kAllow = 1 };

struct TerrainConfig {
  double contact_patch_radius{0.18};
  double max_contact_search_z{0.35};
  double max_drop{0.30};
  double max_step{0.12};
  double max_slope_rad{0.40};
  double max_cross_slope_rad{0.30};
  double max_warp{0.05};
  double min_patch_coverage{0.60};
  double min_plane_confidence{0.10};
  // Global snapshot freshness limit. This bounds query.stamp relative to the
  // immutable snapshot stamp.
  double max_snapshot_age{0.50};
  // Per-voxel freshness limit. Support candidates and body free/occupied cells
  // use MapLookup::last_observed_stamp, so unrelated rays refreshing the
  // global snapshot stamp cannot keep an old support or old free volume alive.
  // This is intentionally independent of max_snapshot_age.
  double max_observation_age{0.50};
  // Bounded local search margin beyond the body AABB for the separate 3-D
  // obstacle clearance. If the expanded search volume would leave the map,
  // evaluate() fails closed with kOutside instead of silently clipping it.
  double obstacle_search_margin{0.30};
  double cluster_max_dz{0.22};
  double plane_inlier_tol{0.06};
  std::size_t max_patches{20000};
  UnknownBodyPolicy unknown_body_policy{UnknownBodyPolicy::kReject};

  Validation validate() const;
};

struct TerrainQuery {
  double stamp{kNaN};
  Vec3 position{Vec3{kNaN, kNaN, kNaN}};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

enum class TerrainStatus : uint8_t {
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

const char* toString(TerrainStatus value);

struct WheelSupport {
  bool supported{false};
  double predicted_z{kNaN};
  double support_z{kNaN};
  double coverage{0.0};
  int patch_id{-1};
};

struct TerrainResult {
  TerrainStatus status{TerrainStatus::kBuildFailure};
  std::string detail;
  Vec3 projected_position{Vec3{kNaN, kNaN, kNaN}};
  Quat projected_orientation{Quat::Identity()};
  double roll{kNaN};
  double pitch{kNaN};
  // Heading azimuth in the world x/y plane. The projected body x axis has this
  // azimuth, so the planned yaw is preserved even on mixed longitudinal and
  // cross slopes. This is not necessarily the ZYX Euler yaw.
  double yaw{kNaN};
  // Minimum vertical gap from an actual body corner to the projected support
  // terrain plane. This is ground clearance; compare it with
  // VehicleGeometry::body_clearance.
  double clearance{kNaN};
  // Minimum 3-D gap from the rigid body OBB to raw occupied voxel volumes that
  // are not identified as the local support floor. Inflation is not used for
  // this final value. Lateral/overhead obstacles reduce it; a flat support
  // floor alone does not. Positive infinity means no non-support obstacle was
  // present in the bounded local search volume. 0 means touching/intruding.
  double obstacle_clearance{kNaN};
  std::array<WheelSupport, 4> wheels{};
  int selected_patch{-1};

  bool ok() const { return status == TerrainStatus::kOk; }
};

// TerrainModel is constructed once per immutable map snapshot. Surface
// candidates, connected patches, plane coefficients, normals and local
// plane/confidence caches are built in the constructor and reused for every
// MPPI pose query.
//
// evaluate() semantics:
//  - support and body cells must be fresh by their per-cell evidence stamps
//    within TerrainConfig::max_observation_age; support specifically needs a
//    fresh measured HIT with no later MISS, and stale data is never accepted;
//  - support uses candidate-local planes fitted to measured hit centroids,
//    while connected patch IDs are only used to keep coverage on the same
//    layer; occupied voxel volumes remain the conservative collision geometry;
//  - the full rigid body and wheel contact patches must fit inside the map;
//  - final body collision uses raw occupancy only (inflation is broadphase);
//  - the returned pose puts the base at base_contact_height above the support
//    plane along its normal and keeps the planned heading yaw; contacts are
//    rechecked after rotation for up to three conservative iterations;
//  - result.clearance is ground clearance; result.obstacle_clearance is a
//    separate bounded 3-D raw-occupancy clearance for soft obstacle costs.
class TerrainModel {
 public:
  TerrainModel(std::shared_ptr<const MapSnapshot> snapshot,
               TerrainConfig config, VehicleGeometry geometry);
  ~TerrainModel();
  TerrainModel(const TerrainModel&) = delete;
  TerrainModel& operator=(const TerrainModel&) = delete;
  TerrainModel(TerrainModel&&) noexcept;
  TerrainModel& operator=(TerrainModel&&) noexcept;

  TerrainResult evaluate(const TerrainQuery& query) const;
  std::size_t patchCount() const;
  std::size_t patchCellCount(int patch_id) const;
  int patchAtColumn(double x, double y, double near_z) const;
  const Validation& buildStatus() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dddmr_local_map
