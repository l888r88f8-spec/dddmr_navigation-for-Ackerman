#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "dddmr_local_map/local_map.hpp"
#include "local_planner/mppi_controller.h"
#include "local_planner/mppi_critic_manager.h"

namespace local_planner {

// Environment source for the MPPI obstacle/traversability critic data. The
// parameter name is mppi.environment_backend. Only these exact spellings are
// accepted; unknown values are rejected instead of silently selecting the
// legacy point cloud path.
//
// kTiltedPlane2D freezes the current base_link pose once per control cycle,
// filters the map obstacle snapshot into the [-0.20, 0.60] m band measured in
// that plane, flattens the accepted points to plane z=0, projects the tracking
// reference into the same plane and runs the existing planar Ackermann MPPI
// there. Output poses carry the full frozen attitude back into the map frame.
enum class MppiEnvironmentBackend : std::uint8_t {
  kLegacyPoints = 0,
  kLocalMap = 1,
  kTiltedPlane2D = 2,
};

const char* toString(MppiEnvironmentBackend value);
const char* mppiEnvironmentBackendParameterName();
bool parseMppiEnvironmentBackend(const std::string& value,
                                 MppiEnvironmentBackend* backend);

// Explicit result state for a terrain-checked MPPI rollout. A rollout is only
// accepted when valid == true and status == kOk.
enum class MppiLocalMapRolloutStatus : std::uint8_t {
  kOk = 0,
  kMissingMap,
  kInvalidContext,
  kInvalidTransform,
  kInvalidConfig,
  kEmptyControls,
  kInvalidControl,
  kInvalidInitialPose,
  kInitialPoseRejected,
  kStale,
  kOutside,
  kTerrainRejected,
  kNonFiniteResult
};

const char* toString(MppiLocalMapRolloutStatus value);

// One immutable map cycle bundle acquired by the caller. Keep the same context
// for every candidate in a batch and for the final chosen sequence; re-querying
// the live map or publishing a newer snapshot while scoring is deliberately not
// part of this helper.
struct MppiLocalMapContext {
  std::shared_ptr<const dddmr_local_map::MapSnapshot> snapshot;
  std::shared_ptr<const dddmr_local_map::TerrainModel> terrain;
  // Rigid transform from the planner frame to the map frame.
  Eigen::Isometry3d planner_to_map{Eigen::Isometry3d::Identity()};
  // Current planning time on the snapshot clock, forwarded to terrain queries.
  double query_stamp{std::numeric_limits<double>::quiet_NaN()};

  bool valid(std::string* detail = nullptr) const;
  std::uint64_t mapVersion() const;
  double mapStamp() const;
};

struct MppiLocalMapRolloutInput {
  // One control per rollout step. Empty controls are invalid.
  std::vector<MppiControl> controls;
  double dt{0.1};
  // Full initial pose in the planner frame.
  Eigen::Isometry3d initial_pose_planner{Eigen::Isometry3d::Identity()};
};

struct MppiLocalMapRolloutResult {
  bool valid{false};
  MppiLocalMapRolloutStatus status{MppiLocalMapRolloutStatus::kInvalidContext};
  // Meaningful for terrain failures; kOk otherwise.
  dddmr_local_map::TerrainStatus terrain_status{
      dddmr_local_map::TerrainStatus::kOk};
  std::string detail;
  std::uint64_t map_version{0};
  double map_stamp{std::numeric_limits<double>::quiet_NaN()};

  // Initial pose after the first full 3-D terrain projection, planner frame.
  // Only meaningful when valid == true.
  Eigen::Isometry3d projected_start_pose{Eigen::Isometry3d::Identity()};
  // One projected full 3-D pose per control, planner frame. These exact poses
  // are the source for trajectory.points, so rendering must not query again.
  std::vector<Eigen::Isometry3d> projected_poses;
  MppiScoredTrajectory trajectory;
  // Number of control steps fully evaluated before success or first failure.
  std::size_t evaluated_steps{0};
};

// Pure health inputs derived from one immutable adapter handle. Keeping this
// independent of ROS types lets controller wiring and regressions use the exact
// same freshness/health policy without a live node or DDS.
enum class MppiLocalMapHealthStatus : std::uint8_t {
  kOk = 0,
  kMissingBundle,
  kUnhealthyMap,
  kNotControlReady,
  kInvalidTimestamp,
  kFutureTimestamp,
  kStaleRosAge,
  kStaleReceiptAge,
  kClockRewind,
  kGenerationMismatch,
  kInvalidContext,
};

const char* toString(MppiLocalMapHealthStatus value);

struct MppiLocalMapHealthInput {
  MppiLocalMapContext context;
  bool handle_present{true};
  bool map_healthy{false};
  bool control_ready{false};
  double measurement_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double now_ros_sec{std::numeric_limits<double>::quiet_NaN()};
  double receive_steady_sec{std::numeric_limits<double>::quiet_NaN()};
  double now_steady_sec{std::numeric_limits<double>::quiet_NaN()};
  double max_age_sec{0.50};
  double future_tolerance_sec{0.05};
  double clock_rewind_tolerance_sec{0.10};
  std::uint64_t generation{0};
};

struct MppiLocalMapHealth {
  bool ok{false};
  MppiLocalMapHealthStatus status{
      MppiLocalMapHealthStatus::kMissingBundle};
  std::string detail;

  // Shared by the pre-optimization acquisition check and the post-optimization
  // recheck. When check_generation is true, generation must equal
  // expected_generation; a newer map version is allowed but is never mixed into
  // the pinned rollout context.
  static MppiLocalMapHealth validate(const MppiLocalMapHealthInput& input,
                                     bool check_generation = false,
                                     std::uint64_t expected_generation = 0);
};

struct MppiLocalMapValidation {
  bool ok{false};
  std::string detail;
};

// Validates the final weighted sequence against the same pinned context that
// produced every candidate. It requires a full rollout (no partial poses),
// every pose evaluated and collision-free, exact pose-to-point agreement, and
// matching map metadata. The caller still performs the independent speed-cap
// check and the post-optimization adapter freshness recheck.
MppiLocalMapValidation validateMppiLocalMapRollout(
    const MppiLocalMapContext& context,
    const MppiLocalMapRolloutResult& result,
    std::size_t expected_control_count);

// Pure C++/Eigen rollout. ROS-independent and safe for the shared scoring/RViz
// path: no ROS message, node, TF lookup, old point-cloud snapshot, or live map
// owner is consulted. Terrain support/body checks always run, independent of
// optional critics. The first invalid pose stops the rollout and invalidates the
// whole result; partial poses must never be accepted.
MppiLocalMapRolloutResult rolloutMppiLocalMap(
    const MppiLocalMapContext& context,
    const MppiLocalMapRolloutInput& input);

}  // namespace local_planner
