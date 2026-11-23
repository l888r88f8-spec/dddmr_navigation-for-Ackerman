#include "local_planner/mppi_tilted_plane.h"

#include <angles/angles.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace local_planner
{
namespace
{

constexpr double kMinimumQuaternionNorm = 1e-9;
constexpr double kMinimumSegmentLength = 1e-6;
constexpr double kMinimumCuboidHeight = 1e-6;

double yawFromQuaternion(const Eigen::Quaterniond & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w() * quaternion.z() +
           quaternion.x() * quaternion.y()),
    1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                 quaternion.z() * quaternion.z()));
}

bool finiteVector(const Eigen::Vector3d & value)
{
  return value.allFinite();
}

// One projected reference pose before the heading fallback is resolved. The
// plane-frame forward vector is the primary heading source; a pose whose
// forward vector is (nearly) perpendicular to the plane has no usable planar
// heading and must borrow the neighbour tangent instead.
struct ProjectedPathPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double orientation_yaw{0.0};
  bool orientation_heading_valid{false};
};

bool buildPlanarPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & planner_path,
  const MppiPlanarFrame & frame,
  std::vector<MppiPathPoint> * path,
  double * path_length,
  std::string * error)
{
  if(path == nullptr || path_length == nullptr){
    if(error != nullptr){
      *error = "planar path output is null";
    }
    return false;
  }
  path->clear();
  *path_length = 0.0;
  if(planner_path.empty()){
    return true;
  }

  const Eigen::Matrix3d planner_to_plane = frame.planner_to_base.linear().transpose();
  std::vector<ProjectedPathPose> projected;
  projected.reserve(planner_path.size());
  for(std::size_t i = 0; i < planner_path.size(); ++i){
    const auto & pose = planner_path[i].pose;
    const Eigen::Vector3d planner_position(
      pose.position.x, pose.position.y, pose.position.z);
    if(!finiteVector(planner_position)){
      if(error != nullptr){
        *error = "tracking reference contains a non-finite position at index " +
          std::to_string(i);
      }
      return false;
    }
    const Eigen::Quaterniond planner_orientation(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
    // An unusable orientation only loses its heading source; the projected
    // position is still valid, so the path continues with the neighbour
    // tangent or the planar-yaw fallback instead of failing the whole cycle.
    const bool orientation_valid =
      planner_orientation.coeffs().allFinite() &&
      planner_orientation.norm() >= kMinimumQuaternionNorm;

    ProjectedPathPose entry;
    entry.position = frame.toPlanePoint(planner_position);
    if(orientation_valid){
      const Eigen::Quaterniond normalized = planner_orientation.normalized();
      const Eigen::Vector3d plane_forward =
        planner_to_plane * (normalized * Eigen::Vector3d::UnitX());
      // A forward vector that is perpendicular to the plane projects to (0, 0)
      // and cannot define a heading; the caller resolves those from neighbours.
      entry.orientation_heading_valid =
        std::hypot(plane_forward.x(), plane_forward.y()) > kMinimumSegmentLength;
      entry.orientation_yaw = entry.orientation_heading_valid ?
        std::atan2(plane_forward.y(), plane_forward.x()) :
        frame.toPlanarYaw(normalized);
    }
    else{
      entry.orientation_heading_valid = false;
      entry.orientation_yaw = frame.toPlanarYaw(planner_orientation);
    }
    projected.push_back(entry);

    if(i > 0){
      *path_length +=
        (projected[i].position.head<2>() -
        projected[i - 1].position.head<2>()).norm();
    }
  }

  const auto neighbourYaw = [&](const std::size_t index, double * yaw) {
    for(std::size_t next = index + 1; next < projected.size(); ++next){
      const Eigen::Vector2d delta =
        projected[next].position.head<2>() - projected[index].position.head<2>();
      if(delta.norm() > kMinimumSegmentLength){
        *yaw = std::atan2(delta.y(), delta.x());
        return true;
      }
    }
    for(std::size_t previous = index; previous > 0; --previous){
      const Eigen::Vector2d delta =
        projected[index].position.head<2>() -
        projected[previous - 1].position.head<2>();
      if(delta.norm() > kMinimumSegmentLength){
        *yaw = std::atan2(delta.y(), delta.x());
        return true;
      }
    }
    return false;
  };

  path->reserve(projected.size());
  for(std::size_t i = 0; i < projected.size(); ++i){
    MppiPathPoint point;
    point.x = projected[i].position.x();
    point.y = projected[i].position.y();
    point.z = 0.0;
    point.obstacle_free = true;
    if(projected[i].orientation_heading_valid){
      point.yaw = projected[i].orientation_yaw;
    }
    else if(!neighbourYaw(i, &point.yaw)){
      // A degenerate single point has neither a planar forward vector nor a
      // tangent; the projected orientation yaw is the only defined fallback.
      point.yaw = projected[i].orientation_yaw;
    }
    path->push_back(point);
  }
  return true;
}

}  // namespace

Eigen::Vector3d MppiPlanarFrame::toPlanePoint(
  const Eigen::Vector3d & planner_point) const
{
  return planner_to_base.linear().transpose() *
         (planner_point - planner_to_base.translation());
}

Eigen::Vector3d MppiPlanarFrame::toPlannerPoint(
  const double x, const double y, const double z) const
{
  return planner_to_base.translation() +
         planner_to_base.linear() * Eigen::Vector3d(x, y, z);
}

Eigen::Quaterniond MppiPlanarFrame::toPlannerOrientation(
  const double planar_yaw) const
{
  Eigen::Quaterniond orientation(planner_to_base.linear());
  orientation.normalize();
  return (orientation *
          Eigen::Quaterniond(Eigen::AngleAxisd(
            planar_yaw, Eigen::Vector3d::UnitZ()))).normalized();
}

double MppiPlanarFrame::toPlanarYaw(
  const Eigen::Quaterniond & planner_orientation) const
{
  if(!planner_orientation.coeffs().allFinite() ||
     planner_orientation.norm() < kMinimumQuaternionNorm)
  {
    return 0.0;
  }
  Eigen::Quaterniond frame_orientation(planner_to_base.linear());
  frame_orientation.normalize();
  const Eigen::Quaterniond local_orientation =
    (frame_orientation.inverse() * planner_orientation.normalized()).normalized();
  return yawFromQuaternion(local_orientation);
}

MppiPlanarFrame makeMppiPlanarFrame(
  const Eigen::Isometry3d & planner_to_base)
{
  MppiPlanarFrame frame;
  if(!planner_to_base.matrix().allFinite()){
    return frame;
  }
  const Eigen::Quaterniond orientation(planner_to_base.linear());
  if(!orientation.coeffs().allFinite() ||
     orientation.norm() < kMinimumQuaternionNorm)
  {
    return frame;
  }
  frame.planner_to_base = planner_to_base;
  frame.planner_to_base.linear() =
    orientation.normalized().toRotationMatrix();
  frame.valid = true;
  return frame;
}

MppiPlanarAnchorTransfer makeMppiPlanarAnchorTransfer(
  const Eigen::Isometry3d & planner_from_anchor,
  const MppiPlanarFrame & frame)
{
  MppiPlanarAnchorTransfer transfer;
  if(!frame.valid || !planner_from_anchor.matrix().allFinite()){
    return transfer;
  }
  const Eigen::Quaterniond anchor_orientation(planner_from_anchor.linear());
  if(!anchor_orientation.coeffs().allFinite() ||
     anchor_orientation.norm() < kMinimumQuaternionNorm)
  {
    return transfer;
  }
  Eigen::Isometry3d normalized_anchor = Eigen::Isometry3d::Identity();
  normalized_anchor.translation() = planner_from_anchor.translation();
  normalized_anchor.linear() =
    anchor_orientation.normalized().toRotationMatrix();
  // planner_from_anchor is map <- anchor, so its inverse composed with the
  // current map <- base_link pose is anchor <- base_link.
  transfer.anchor_from_base = normalized_anchor.inverse() * frame.planner_to_base;
  if(!transfer.anchor_from_base.matrix().allFinite()){
    return transfer;
  }
  transfer.valid = true;
  return transfer;
}

Eigen::Vector2d MppiPlanarAnchorTransfer::planePointToAnchor(
  const double x, const double y) const
{
  if(!valid){
    return Eigen::Vector2d::Zero();
  }
  return (anchor_from_base * Eigen::Vector3d(x, y, 0.0)).head<2>();
}

double MppiPlanarAnchorTransfer::planeYawToAnchor(
  const double plane_yaw) const
{
  if(!valid){
    return 0.0;
  }
  const Eigen::Vector3d forward = anchor_from_base.linear() *
    Eigen::Vector3d(std::cos(plane_yaw), std::sin(plane_yaw), 0.0);
  return std::atan2(forward.y(), forward.x());
}

Eigen::Vector3d MppiPlanarAnchorTransfer::anchorPoseToPlane(
  const double x, const double y, const double yaw) const
{
  if(!valid){
    return Eigen::Vector3d::Zero();
  }
  const Eigen::Isometry3d base_from_anchor = anchor_from_base.inverse();
  const Eigen::Vector3d point = base_from_anchor * Eigen::Vector3d(x, y, 0.0);
  const Eigen::Vector3d forward = base_from_anchor.linear() *
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::Vector3d::UnitX());
  return Eigen::Vector3d(
    point.x(), point.y(), std::atan2(forward.y(), forward.x()));
}

std::vector<MppiPathPoint> MppiPlanarAnchorTransfer::planePathToAnchor(
  const std::vector<MppiPathPoint> & path) const
{
  if(!valid){
    return {};
  }
  std::vector<MppiPathPoint> anchored;
  anchored.reserve(path.size());
  for(const auto & point : path){
    const Eigen::Vector2d xy = planePointToAnchor(point.x, point.y);
    MppiPathPoint anchored_point = point;
    anchored_point.x = xy.x();
    anchored_point.y = xy.y();
    anchored_point.z = 0.0;
    anchored_point.yaw = planeYawToAnchor(point.yaw);
    anchored.push_back(anchored_point);
  }
  return anchored;
}

MppiPlanarRolloutTerminal mppiPlanarRolloutTerminal(
  const std::vector<MppiControl> & controls, const double time_step)
{
  MppiPlanarRolloutTerminal terminal;
  for(const auto & control : controls){
    terminal.x += control.vx *
      std::cos(terminal.yaw + control.wz * time_step * 0.5) * time_step;
    terminal.y += control.vx *
      std::sin(terminal.yaw + control.wz * time_step * 0.5) * time_step;
    terminal.yaw = angles::normalize_angle(terminal.yaw + control.wz * time_step);
  }
  return terminal;
}

bool buildMppiTiltedPlaneScene(
  const pcl::PointCloud<pcl::PointXYZI> & raw_obstacles,
  const std::vector<geometry_msgs::msg::PoseStamped> & planner_path,
  const std::vector<pcl::PointXYZ> & base_cuboid_vertices,
  const MppiPlanarFrame & frame,
  const MppiPlanarObstacleGridConfig & grid_config,
  MppiPlanarScene * scene,
  std::string * error)
{
  if(error != nullptr){
    error->clear();
  }
  if(scene == nullptr){
    if(error != nullptr){
      *error = "planar scene output is null";
    }
    return false;
  }
  *scene = MppiPlanarScene{};
  if(!frame.valid){
    if(error != nullptr){
      *error = "frozen plane frame is invalid";
    }
    return false;
  }
  scene->frame = frame;

  scene->body = mppiPlanarBodyRectFromCuboid(base_cuboid_vertices);
  if(!scene->body.valid){
    if(error != nullptr){
      *error = "configured body envelope has no usable 2-D rectangle";
    }
    return false;
  }

  scene->raw_obstacle_points = raw_obstacles.points.size();
  pcl::PointCloud<pcl::PointXYZI> plane_points;
  plane_points.reserve(raw_obstacles.points.size());
  for(const auto & point : raw_obstacles.points){
    if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
       !std::isfinite(point.z))
    {
      continue;
    }
    const Eigen::Vector3d planner_point(point.x, point.y, point.z);
    const Eigen::Vector3d plane_point = frame.toPlanePoint(planner_point);
    // One shared predicate keeps the band rule identical here, in the grid
    // query and in the device kernel, including its inclusive edges.
    if(!mppiPlanarObstacleBandContains(plane_point.z())){
      continue;
    }
    pcl::PointXYZI flattened;
    flattened.x = static_cast<float>(plane_point.x());
    flattened.y = static_cast<float>(plane_point.y());
    // Everything in the accepted band collapses onto the plane. The 2-D
    // geometry must never see the original height.
    flattened.z = 0.0f;
    flattened.intensity = 0.0f;
    plane_points.push_back(flattened);
  }
  plane_points.width = static_cast<std::uint32_t>(plane_points.size());
  plane_points.height = 1;
  plane_points.is_dense = true;
  scene->filtered_obstacle_points = plane_points.points.size();

  if(!buildMppiPlanarObstacleGrid(plane_points, grid_config, &scene->grid, error)){
    return false;
  }
  if(!buildPlanarPath(
      planner_path, frame, &scene->path, &scene->path_length, error))
  {
    return false;
  }

  scene->valid = true;
  return true;
}

bool validateMppiPlanarBodyEnvelope(
  const std::vector<pcl::PointXYZ> & base_vertices,
  std::string * error)
{
  if(error != nullptr){
    error->clear();
  }
  if(base_vertices.size() < 8){
    if(error != nullptr){
      *error = "planar cuboid requires the eight configured body vertices";
    }
    return false;
  }

  double z_min = std::numeric_limits<double>::infinity();
  double z_max = -std::numeric_limits<double>::infinity();
  for(const auto & vertex : base_vertices){
    if(!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
       !std::isfinite(vertex.z))
    {
      if(error != nullptr){
        *error = "planar cuboid has a non-finite body vertex";
      }
      return false;
    }
    z_min = std::min(z_min, static_cast<double>(vertex.z));
    z_max = std::max(z_max, static_cast<double>(vertex.z));
  }
  // Flattened obstacle points live at z=0. Clamp the volume so that plane
  // occupies the same vertical interval as the physical body even when the
  // configured cuboid starts above or ends below the base_link origin.
  z_min = std::min(z_min, 0.0);
  z_max = std::max(z_max, 0.0);
  if(z_max - z_min < kMinimumCuboidHeight){
    if(error != nullptr){
      *error = "configured body volume has no usable height in the 2-D plane";
    }
    return false;
  }
  // A zero-area XY footprint would make every candidate collision-free, so it
  // must fail startup rather than every cycle's scene build.
  if(!mppiPlanarBodyRectFromCuboid(base_vertices).valid){
    if(error != nullptr){
      *error = "configured body envelope has no usable 2-D XY rectangle";
    }
    return false;
  }
  return true;
}

Eigen::Vector3d mppiPlanarPointToPlanner(
  const MppiPlanarFrame & frame, const double x, const double y,
  const double z)
{
  return frame.toPlannerPoint(x, y, z);
}

Eigen::Quaterniond mppiPlanarYawToPlanner(
  const MppiPlanarFrame & frame, const double yaw)
{
  return frame.toPlannerOrientation(yaw);
}

Eigen::Isometry3d mppiPlanarPoseToPlanner(
  const MppiPlanarFrame & frame, const double x, const double y,
  const double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = mppiPlanarYawToPlanner(frame, yaw).toRotationMatrix();
  pose.translation() = mppiPlanarPointToPlanner(frame, x, y, 0.0);
  return pose;
}

}  // namespace local_planner
