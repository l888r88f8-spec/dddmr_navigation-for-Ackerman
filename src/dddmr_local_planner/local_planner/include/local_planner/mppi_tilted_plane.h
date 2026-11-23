#ifndef LOCAL_PLANNER_MPPI_TILTED_PLANE_H_
#define LOCAL_PLANNER_MPPI_TILTED_PLANE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "local_planner/mppi_critic_manager.h"
#include "local_planner/mppi_planar_obstacles.h"

namespace local_planner
{

// Simple MPPI 2-D avoidance uses the current base_link attitude as one frozen
// plane for the whole control tick. The plane origin is the base_link origin
// and its local x/y axes are the base_link x/y axes; the robot therefore starts
// at (0, 0) with heading 0 in that plane. The height band and its shared
// predicate (kMppiPlanarMinObstacleZ, kMppiPlanarMaxObstacleZ,
// mppiPlanarObstacleBandContains) live in mppi_planar_obstacles.h so the CUDA
// translation units can use the same device-callable rule.

struct MppiPlanarFrame
{
  bool valid{false};
  // Pose of base_link in the planner frame (map <- base_link) at the instant
  // the frame was frozen. Applying the inverse maps planner points into the
  // plane; applying the forward transform maps plane points back to the map.
  Eigen::Isometry3d planner_to_base{Eigen::Isometry3d::Identity()};

  Eigen::Vector3d toPlanePoint(const Eigen::Vector3d & planner_point) const;
  Eigen::Vector3d toPlannerPoint(double x, double y, double z = 0.0) const;
  Eigen::Quaterniond toPlannerOrientation(double planar_yaw) const;
  double toPlanarYaw(const Eigen::Quaterniond & planner_orientation) const;
};

// Validates and normalizes a complete 3-D base pose expressed as
// map <- base_link. The caller must use the same planner frame for the
// obstacle cloud, tracking reference and output.
MppiPlanarFrame makeMppiPlanarFrame(const Eigen::Isometry3d & planner_to_base);

// Conversion between the current cycle's plane frame B_k and a map-anchored
// frame A that is frozen when a directed-retreat search starts. Every method
// name states the direction it produces: anchor_from_base is A <- B_k, and the
// validator conversion produces plane (B_k) coordinates from anchor ones.
// Keeping the search state in A is what lets a retained route survive
// cycle-to-cycle attitude changes while the obstacle query stays in B_k.
struct MppiPlanarAnchorTransfer
{
  bool valid{false};
  // Anchor (map) <- current base_link plane frame.
  Eigen::Isometry3d anchor_from_base{Eigen::Isometry3d::Identity()};

  Eigen::Vector2d planePointToAnchor(double x, double y) const;
  // Rotates a plane heading into the anchor frame.
  double planeYawToAnchor(double plane_yaw) const;
  // Converts an anchor-frame proposal into the current plane. Returns
  // (x, y, yaw) in B_k.
  Eigen::Vector3d anchorPoseToPlane(double x, double y, double yaw) const;
  // Anchors every path point; z stays 0 because both frames are planar.
  std::vector<MppiPathPoint> planePathToAnchor(
    const std::vector<MppiPathPoint> & path) const;
};

// Builds the transfer from the map <- anchor pose that was frozen at search
// start and the current cycle's plane frame. Returns an invalid transfer for a
// non-finite anchor pose or an invalid current frame.
MppiPlanarAnchorTransfer makeMppiPlanarAnchorTransfer(
  const Eigen::Isometry3d & planner_from_anchor,
  const MppiPlanarFrame & frame);

// Terminal state of the shared planar Ackermann integration. Starting at
// (0, 0, 0), each control advances by the same midpoint-heading step the
// candidate scorer uses. The caller passes the clamped model_dt.
struct MppiPlanarRolloutTerminal
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

MppiPlanarRolloutTerminal mppiPlanarRolloutTerminal(
  const std::vector<MppiControl> & controls, double time_step);

// One immutable tilted-plane cycle. The obstacle set is a uniform XY grid over
// the height-filtered plane points; no 3-D PCL snapshot is kept because every
// collision and clearance query below is purely planar. The path has already
// been projected into the plane (z == 0), so its arc length, occupancy and
// heading never mix map and plane coordinates.
struct MppiPlanarScene
{
  bool valid{false};
  // Monotonic per-cycle identity assigned by the caller that composed this
  // scene. A new scene is built every control cycle, so a changed identity
  // covers obstacle, frozen-attitude and reference changes at once. Zero means
  // "unassigned" and must not be cached.
  std::uint64_t identity{0};
  MppiPlanarFrame frame;
  MppiPlanarObstacleGrid grid;
  MppiPlanarBodyRect body;
  std::vector<MppiPathPoint> path;
  double path_length{0.0};
  std::size_t blocked_path_points{0};
  MppiPlanarQueryResult path_query_counts;
  std::size_t raw_obstacle_points{0};
  std::size_t filtered_obstacle_points{0};
  double distance_field_build_ms{-1.0};
};

// Configuration-time check of the configured body envelope: eight finite
// vertices whose flattened form still encloses plane z=0 with a usable height
// and a non-degenerate 2-D XY rectangle. The tilted-plane backend refuses to
// start when this fails, instead of silently scoring every candidate
// collision-free.
bool validateMppiPlanarBodyEnvelope(
  const std::vector<pcl::PointXYZ> & base_vertices,
  std::string * error = nullptr);

// Transforms the raw local obstacle cloud into the frozen plane, keeps points
// whose plane-frame z is inside [kMppiPlanarMinObstacleZ,
// kMppiPlanarMaxObstacleZ] inclusive, drops the height and builds the 2-D
// grid exactly once per cycle. The reference path is projected with the same
// frame; its heading comes from the plane-frame forward vector of each pose
// orientation, falling back to the projected neighbour tangent when that
// vector is (nearly) perpendicular to the plane or the orientation is
// unusable. Returns false with a detail string for an invalid frame/body/grid
// or a non-finite path position.
bool buildMppiTiltedPlaneScene(
  const pcl::PointCloud<pcl::PointXYZI> & raw_obstacles,
  const std::vector<geometry_msgs::msg::PoseStamped> & planner_path,
  const std::vector<pcl::PointXYZ> & base_cuboid_vertices,
  const MppiPlanarFrame & frame,
  const MppiPlanarObstacleGridConfig & grid_config,
  MppiPlanarScene * scene,
  std::string * error = nullptr);

// The frozen plane is a snapshot of base_link, not a persistent TF frame.
// These pure helpers are used by tests and by diagnostics that need to place
// an already-computed plane point back into the public planner/map frame.
Eigen::Vector3d mppiPlanarPointToPlanner(
  const MppiPlanarFrame & frame, double x, double y, double z = 0.0);
Eigen::Quaterniond mppiPlanarYawToPlanner(
  const MppiPlanarFrame & frame, double yaw);
// Convenience wrapper used by debug poses, probes and tests: the plane x/y/yaw
// plus the frozen attitude become one full planner-frame isometry. Production
// trajectory output composes its map poses from the point/yaw helpers above.
Eigen::Isometry3d mppiPlanarPoseToPlanner(
  const MppiPlanarFrame & frame, double x, double y, double yaw);

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_TILTED_PLANE_H_
