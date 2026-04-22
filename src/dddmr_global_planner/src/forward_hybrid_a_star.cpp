#include <global_planner/forward_hybrid_a_star.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool IsFinite(double value)
{
  return std::isfinite(value);
}

double SquaredDistance2D(double x0, double y0, double x1, double y1)
{
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return dx * dx + dy * dy;
}

double Distance3D(
  double x0, double y0, double z0,
  double x1, double y1, double z1)
{
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  const double dz = z0 - z1;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

namespace global_planner
{

ForwardHybridAStar::ForwardHybridAStar(
  std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground,
  const rclcpp::Logger & logger)
: perception_3d_(std::move(perception_3d)),
  pcl_ground_(std::move(pcl_ground)),
  kdtree_ground_(std::move(kdtree_ground)),
  logger_(logger)
{
  SetConfig(config_);
}

void ForwardHybridAStar::SetConfig(const Config & config)
{
  config_ = config;
  config_.wheelbase = std::max(config_.wheelbase, 1e-3);
  config_.max_steer = std::max(std::abs(config_.max_steer), 1e-3);
  config_.heading_bin_count = std::max(config_.heading_bin_count, 8);
  config_.primitive_length = std::max(config_.primitive_length, 1e-3);
  config_.primitive_step = std::max(config_.primitive_step, 1e-3);
  config_.projection_search_radius = std::max(config_.projection_search_radius, 1e-3);
  config_.goal_position_tolerance = std::max(config_.goal_position_tolerance, 1e-3);
  config_.goal_heading_tolerance = std::max(config_.goal_heading_tolerance, 1e-3);
  config_.turn_side_hysteresis_penalty = std::max(config_.turn_side_hysteresis_penalty, 0.0);
  config_.rearward_check_distance = std::max(config_.rearward_check_distance, 0.0);
  config_.rearward_allowance = std::max(config_.rearward_allowance, 0.0);
  config_.rearward_excursion_penalty = std::max(config_.rearward_excursion_penalty, 0.0);
  config_.rearward_hard_reject_distance = std::max(config_.rearward_hard_reject_distance, 0.0);
  if (config_.rearward_hard_reject_distance < config_.rearward_allowance) {
    config_.rearward_hard_reject_distance = config_.rearward_allowance;
  }

  primitive_steers_.clear();
  primitive_steers_.reserve(5);
  primitive_steers_.push_back(-config_.max_steer);
  primitive_steers_.push_back(-0.5 * config_.max_steer);
  primitive_steers_.push_back(0.0);
  primitive_steers_.push_back(0.5 * config_.max_steer);
  primitive_steers_.push_back(config_.max_steer);
}

void ForwardHybridAStar::SetGlobalFrame(const std::string & global_frame)
{
  global_frame_ = global_frame;
}

double ForwardHybridAStar::NormalizeAngle(double angle) const
{
  if (!IsFinite(angle)) {
    return 0.0;
  }

  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

int ForwardHybridAStar::QuantizeYaw(double yaw) const
{
  const double normalized = NormalizeAngle(yaw);
  const double wrapped = normalized + kPi;
  const double bin_size = kTwoPi / static_cast<double>(config_.heading_bin_count);

  int heading_bin = static_cast<int>(std::floor(wrapped / bin_size));
  if (heading_bin < 0) {
    heading_bin = 0;
  }
  if (heading_bin >= config_.heading_bin_count) {
    heading_bin = config_.heading_bin_count - 1;
  }
  return heading_bin;
}

double ForwardHybridAStar::DequantizeYaw(int heading_bin) const
{
  if (config_.heading_bin_count <= 0) {
    return 0.0;
  }

  const int clamped_bin = std::clamp(heading_bin, 0, config_.heading_bin_count - 1);
  const double bin_size = kTwoPi / static_cast<double>(config_.heading_bin_count);
  return NormalizeAngle(-kPi + (static_cast<double>(clamped_bin) + 0.5) * bin_size);
}

bool ForwardHybridAStar::ExtractYaw(const geometry_msgs::msg::PoseStamped & pose, double * yaw) const
{
  if (yaw == nullptr) {
    return false;
  }

  const auto & q_msg = pose.pose.orientation;
  if (!IsFinite(q_msg.x) || !IsFinite(q_msg.y) || !IsFinite(q_msg.z) || !IsFinite(q_msg.w)) {
    return false;
  }

  const double norm =
    q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w;
  if (norm < 1e-9) {
    return false;
  }

  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw_value = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_value);
  if (!IsFinite(yaw_value)) {
    return false;
  }

  *yaw = NormalizeAngle(yaw_value);
  return true;
}

bool ForwardHybridAStar::ComputeStateId(
  std::size_t ground_index,
  int heading_bin,
  std::size_t * state_id) const
{
  if (state_id == nullptr) {
    return false;
  }
  if (heading_bin < 0 || heading_bin >= config_.heading_bin_count) {
    return false;
  }
  if (pcl_ground_ == nullptr || ground_index >= pcl_ground_->points.size()) {
    return false;
  }

  const std::size_t heading_bin_size = static_cast<std::size_t>(config_.heading_bin_count);
  if (heading_bin_size == 0) {
    return false;
  }

  const std::size_t max_size = std::numeric_limits<std::size_t>::max();
  if (ground_index > max_size / heading_bin_size) {
    return false;
  }

  const std::size_t base = ground_index * heading_bin_size;
  const std::size_t heading_bin_unsigned = static_cast<std::size_t>(heading_bin);
  if (base > max_size - heading_bin_unsigned) {
    return false;
  }

  *state_id = base + heading_bin_unsigned;
  return true;
}

bool ForwardHybridAStar::PoseToGroundIndex(
  double x,
  double y,
  double z_hint,
  std::size_t * ground_index,
  double * projected_z,
  ProjectionDiagnostics * diagnostics) const
{
  if (ground_index == nullptr || projected_z == nullptr) {
    return false;
  }

  if (diagnostics != nullptr) {
    *diagnostics = ProjectionDiagnostics();
    diagnostics->query_x = x;
    diagnostics->query_y = y;
    diagnostics->query_z = z_hint;
  }

  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z_hint)) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "invalid_query";
    }
    return false;
  }

  if (kdtree_ground_ == nullptr || pcl_ground_ == nullptr || pcl_ground_->points.empty()) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "ground_unavailable";
    }
    return false;
  }

  pcl::PointXYZI query;
  query.x = static_cast<float>(x);
  query.y = static_cast<float>(y);
  query.z = static_cast<float>(z_hint);
  query.intensity = 0.0f;

  std::vector<int> point_indices;
  std::vector<float> point_squared_distances;

  const int found_in_radius =
    kdtree_ground_->radiusSearch(
    query,
    config_.projection_search_radius,
    point_indices,
    point_squared_distances);

  if (found_in_radius > 0 && !point_indices.empty() && !point_squared_distances.empty()) {
    std::size_t closest_idx = 0;
    for (std::size_t i = 1; i < point_indices.size() && i < point_squared_distances.size(); ++i) {
      if (point_squared_distances[i] < point_squared_distances[closest_idx]) {
        closest_idx = i;
      }
    }

    const int nearest_index = point_indices[closest_idx];
    if (nearest_index < 0 || static_cast<std::size_t>(nearest_index) >= pcl_ground_->points.size()) {
      return false;
    }

    const pcl::PointXYZI & nearest_pt = pcl_ground_->points[static_cast<std::size_t>(nearest_index)];
    if (!IsFinite(nearest_pt.z)) {
      if (diagnostics != nullptr) {
        diagnostics->fallback = "invalid_projected_point";
      }
      return false;
    }

    *ground_index = static_cast<std::size_t>(nearest_index);
    *projected_z = nearest_pt.z;
    if (diagnostics != nullptr) {
      diagnostics->success = true;
      diagnostics->ground_index = *ground_index;
      diagnostics->ground_x = nearest_pt.x;
      diagnostics->ground_y = nearest_pt.y;
      diagnostics->ground_z = nearest_pt.z;
      diagnostics->projection_distance =
        Distance3D(x, y, z_hint, nearest_pt.x, nearest_pt.y, nearest_pt.z);
      diagnostics->fallback = "none";
      diagnostics->fallback_z_before = z_hint;
      diagnostics->fallback_z_after = nearest_pt.z;
      diagnostics->fallback_nearest_distance = std::sqrt(point_squared_distances[closest_idx]);
    }
    return true;
  }

  point_indices.clear();
  point_squared_distances.clear();
  if (kdtree_ground_->nearestKSearch(query, 1, point_indices, point_squared_distances) <= 0) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_not_found";
    }
    return false;
  }
  if (point_indices.empty() || point_squared_distances.empty()) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_empty";
    }
    return false;
  }

  const int nearest_index = point_indices.front();
  if (nearest_index < 0 || static_cast<std::size_t>(nearest_index) >= pcl_ground_->points.size()) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_invalid_index";
    }
    return false;
  }

  const double nearest_distance = std::sqrt(point_squared_distances.front());
  if (!IsFinite(nearest_distance) || nearest_distance > config_.projection_search_radius * 2.0) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_rejected_too_far";
      diagnostics->fallback_nearest_distance = nearest_distance;
    }
    return false;
  }

  const pcl::PointXYZI & nearest_pt = pcl_ground_->points[static_cast<std::size_t>(nearest_index)];
  if (!IsFinite(nearest_pt.z)) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_invalid_projected_point";
    }
    return false;
  }

  *ground_index = static_cast<std::size_t>(nearest_index);
  *projected_z = nearest_pt.z;
  if (diagnostics != nullptr) {
    diagnostics->success = true;
    diagnostics->ground_index = *ground_index;
    diagnostics->ground_x = nearest_pt.x;
    diagnostics->ground_y = nearest_pt.y;
    diagnostics->ground_z = nearest_pt.z;
    diagnostics->projection_distance =
      Distance3D(x, y, z_hint, nearest_pt.x, nearest_pt.y, nearest_pt.z);
    diagnostics->fallback = "nearest_k_fallback";
    diagnostics->fallback_z_before = z_hint;
    diagnostics->fallback_z_after = nearest_pt.z;
    diagnostics->fallback_nearest_distance = nearest_distance;
  }
  return true;
}

bool ForwardHybridAStar::ValidateSample(
  double x,
  double y,
  double yaw,
  double z_hint,
  std::size_t * ground_index,
  double * projected_z,
  double * dgraph_value) const
{
  if (ground_index == nullptr || projected_z == nullptr || dgraph_value == nullptr) {
    return false;
  }
  if (perception_3d_ == nullptr || pcl_ground_ == nullptr) {
    return false;
  }

  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(yaw) || !IsFinite(z_hint)) {
    return false;
  }

  std::size_t projected_ground_index = 0;
  double projected_ground_z = 0.0;
  if (!PoseToGroundIndex(x, y, z_hint, &projected_ground_index, &projected_ground_z)) {
    return false;
  }

  if (projected_ground_index >= pcl_ground_->points.size()) {
    return false;
  }
  const pcl::PointXYZI & projected_pt = pcl_ground_->points[projected_ground_index];
  if (!IsFinite(projected_pt.x) || !IsFinite(projected_pt.y) || !IsFinite(projected_pt.z)) {
    return false;
  }
  const double max_projection_distance = config_.projection_search_radius * 1.5;
  if (SquaredDistance2D(x, y, projected_pt.x, projected_pt.y) >
    max_projection_distance * max_projection_distance)
  {
    return false;
  }

  const double dgraph =
    perception_3d_->get_min_dGraphValue(static_cast<unsigned int>(projected_ground_index));
  const double inscribed_radius = perception_3d_->getGlobalUtils()->getInscribedRadius();

  if (!IsFinite(dgraph) || !IsFinite(inscribed_radius)) {
    return false;
  }

  if (dgraph < inscribed_radius) {
    return false;
  }

  *ground_index = projected_ground_index;
  *projected_z = projected_ground_z;
  *dgraph_value = dgraph;
  return true;
}

bool ForwardHybridAStar::IsProjectedGroundTransitionValid(
  std::size_t from_ground_index,
  std::size_t to_ground_index) const
{
  if (pcl_ground_ == nullptr) {
    return false;
  }
  if (from_ground_index >= pcl_ground_->points.size() ||
    to_ground_index >= pcl_ground_->points.size())
  {
    return false;
  }
  if (from_ground_index == to_ground_index) {
    return true;
  }

  const pcl::PointXYZI & from_pt = pcl_ground_->points[from_ground_index];
  const pcl::PointXYZI & to_pt = pcl_ground_->points[to_ground_index];
  if (!IsFinite(from_pt.x) || !IsFinite(from_pt.y) || !IsFinite(to_pt.x) || !IsFinite(to_pt.y)) {
    return false;
  }

  const double max_projection_jump =
    std::max(config_.projection_search_radius, config_.primitive_step * 3.0);
  return SquaredDistance2D(from_pt.x, from_pt.y, to_pt.x, to_pt.y) <=
         max_projection_jump * max_projection_jump;
}

double ForwardHybridAStar::ComputeHeuristic(
  double x,
  double y,
  double yaw,
  const geometry_msgs::msg::PoseStamped & goal,
  double goal_yaw,
  bool has_goal_yaw,
  bool use_goal_heading) const
{
  const double dx = goal.pose.position.x - x;
  const double dy = goal.pose.position.y - y;
  const double distance = std::sqrt(dx * dx + dy * dy);

  double target_yaw = goal_yaw;
  if (!use_goal_heading) {
    if (distance > 1e-6) {
      target_yaw = std::atan2(dy, dx);
    } else if (!has_goal_yaw) {
      target_yaw = yaw;
    }
  }

  const double heading_error = std::abs(NormalizeAngle(target_yaw - yaw));
  return distance + config_.heuristic_heading_weight * heading_error;
}

bool ForwardHybridAStar::IsGoalReached(
  const NodeRecord & node,
  const geometry_msgs::msg::PoseStamped & goal,
  double goal_yaw,
  bool has_goal_yaw,
  bool use_goal_heading) const
{
  const double distance_squared =
    SquaredDistance2D(node.x, node.y, goal.pose.position.x, goal.pose.position.y);
  if (distance_squared >
    config_.goal_position_tolerance * config_.goal_position_tolerance)
  {
    return false;
  }

  if (!use_goal_heading) {
    return true;
  }

  if (!has_goal_yaw) {
    return false;
  }

  const double heading_error = std::abs(NormalizeAngle(goal_yaw - node.yaw));
  return heading_error <= config_.goal_heading_tolerance;
}

double ForwardHybridAStar::GetPrimitiveSteerByIndex(int primitive_index) const
{
  if (primitive_index < 0 ||
    primitive_index >= static_cast<int>(primitive_steers_.size()))
  {
    return 0.0;
  }
  return primitive_steers_[static_cast<std::size_t>(primitive_index)];
}

int ForwardHybridAStar::GetPrimitiveTurnSign(int primitive_index) const
{
  const double steer = GetPrimitiveSteerByIndex(primitive_index);
  const double steer_eps = std::max(1e-4, config_.max_steer * 0.05);
  if (steer > steer_eps) {
    return 1;
  }
  if (steer < -steer_eps) {
    return -1;
  }
  return 0;
}

double ForwardHybridAStar::ComputeTransitionCost(
  const NodeRecord & parent,
  int primitive_index,
  const PrimitiveResult & primitive_result,
  int preferred_initial_turn_sign) const
{
  const double steer = GetPrimitiveSteerByIndex(primitive_index);
  const double parent_steer = GetPrimitiveSteerByIndex(parent.primitive_index);
  const double steer_norm = std::max(config_.max_steer, 1e-6);

  double cost = primitive_result.path_length;
  cost += config_.steering_penalty * (std::abs(steer) / steer_norm);
  cost +=
    config_.steering_change_penalty *
    (std::abs(steer - parent_steer) / steer_norm);
  cost += config_.heading_change_penalty * primitive_result.heading_change;
  cost += config_.obstacle_penalty_weight * primitive_result.obstacle_penalty;
  cost += primitive_result.rearward_penalty;

  // Hysteresis for turn-side stability: penalize selecting the opposite side
  // on the first non-zero steering decision.
  const int primitive_turn_sign = GetPrimitiveTurnSign(primitive_index);
  if (preferred_initial_turn_sign != 0 &&
    parent.initial_turn_sign == 0 &&
    primitive_turn_sign != 0 &&
    primitive_turn_sign != preferred_initial_turn_sign)
  {
    cost += config_.turn_side_hysteresis_penalty;
  }

  return std::max(cost, 1e-6);
}

bool ForwardHybridAStar::RolloutPrimitive(
  const NodeRecord & parent,
  int primitive_index,
  double planning_start_x,
  double planning_start_y,
  double planning_start_yaw,
  PrimitiveResult * primitive_result,
  std::vector<SamplePose> * sampled_poses) const
{
  if (primitive_result == nullptr) {
    return false;
  }
  if (perception_3d_ == nullptr || pcl_ground_ == nullptr) {
    return false;
  }
  if (primitive_index < 0 ||
    primitive_index >= static_cast<int>(primitive_steers_.size()))
  {
    return false;
  }

  if (sampled_poses != nullptr) {
    sampled_poses->clear();
  }

  double x = parent.x;
  double y = parent.y;
  double z = parent.z;
  double yaw = parent.yaw;
  double remaining = config_.primitive_length;
  const double step = std::max(config_.primitive_step, 1e-3);
  const double steer = primitive_steers_[static_cast<std::size_t>(primitive_index)];

  const double inscribed_radius = perception_3d_->getGlobalUtils()->getInscribedRadius();
  const double inflation_descending_rate =
    perception_3d_->getGlobalUtils()->getInflationDescendingRate();

  double heading_change = 0.0;
  double obstacle_penalty_sum = 0.0;
  double rearward_penalty_sum = 0.0;
  std::size_t sample_count = 0;
  std::size_t end_ground_index = parent.ground_index;
  double end_projected_z = z;
  std::size_t previous_ground_index = parent.ground_index;

  if (pcl_ground_ == nullptr || previous_ground_index >= pcl_ground_->points.size()) {
    return false;
  }

  while (remaining > 1e-9) {
    const double ds = std::min(step, remaining);
    const double previous_yaw = yaw;

    x += ds * std::cos(yaw);
    y += ds * std::sin(yaw);
    yaw = NormalizeAngle(yaw + ds / config_.wheelbase * std::tan(steer));

    // The trajectory is continuous in SE(2), then each sample is projected to
    // nearest ground points for terrain/collision semantics.
    std::size_t sample_ground_index = 0;
    double sample_z = 0.0;
    double sample_dgraph = 0.0;
    if (!ValidateSample(
        x, y, yaw, z,
        &sample_ground_index, &sample_z, &sample_dgraph))
    {
      return false;
    }
    if (!IsProjectedGroundTransitionValid(previous_ground_index, sample_ground_index)) {
      return false;
    }

    // Rearward excursion is evaluated in the current planning-start frame.
    // It suppresses local reconnect arcs that fold into the vehicle rear side.
    const double dx_start = x - planning_start_x;
    const double dy_start = y - planning_start_y;
    const double distance_from_start = std::hypot(dx_start, dy_start);
    if (distance_from_start <= config_.rearward_check_distance) {
      const double x_local =
        std::cos(planning_start_yaw) * dx_start + std::sin(planning_start_yaw) * dy_start;
      if (x_local < -config_.rearward_hard_reject_distance) {
        return false;
      }
      if (x_local < -config_.rearward_allowance) {
        rearward_penalty_sum += config_.rearward_excursion_penalty;
      }
    }

    if (sampled_poses != nullptr) {
      sampled_poses->push_back(SamplePose{x, y, sample_z, yaw});
    }

    z = sample_z;
    end_ground_index = sample_ground_index;
    end_projected_z = sample_z;
    previous_ground_index = sample_ground_index;

    heading_change += std::abs(NormalizeAngle(yaw - previous_yaw));
    const double obstacle_factor =
      std::exp(-1.0 * inflation_descending_rate * (sample_dgraph - inscribed_radius));
    obstacle_penalty_sum += obstacle_factor;

    remaining -= ds;
    ++sample_count;
  }

  primitive_result->end_x = x;
  primitive_result->end_y = y;
  primitive_result->end_z = end_projected_z;
  primitive_result->end_yaw = yaw;
  primitive_result->end_ground_index = end_ground_index;
  primitive_result->end_heading_bin = QuantizeYaw(yaw);
  primitive_result->path_length = config_.primitive_length;
  primitive_result->heading_change = heading_change;
  primitive_result->obstacle_penalty =
    sample_count > 0 ? obstacle_penalty_sum / static_cast<double>(sample_count) : 0.0;
  primitive_result->rearward_penalty =
    sample_count > 0 ? rearward_penalty_sum / static_cast<double>(sample_count) : 0.0;

  return true;
}

void ForwardHybridAStar::AppendPose(
  double x,
  double y,
  double z,
  double yaw,
  nav_msgs::msg::Path * ros_path) const
{
  if (ros_path == nullptr) {
    return;
  }

  if (!ros_path->poses.empty()) {
    const auto & last = ros_path->poses.back();
    const double distance_squared = SquaredDistance2D(
      x, y,
      last.pose.position.x, last.pose.position.y);
    double last_yaw = 0.0;
    const bool has_last_yaw = ExtractYaw(last, &last_yaw);

    if (distance_squared < 1e-8 &&
      (!has_last_yaw || std::abs(NormalizeAngle(last_yaw - yaw)) < 1e-5))
    {
      return;
    }
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = ros_path->header;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, NormalizeAngle(yaw));
  q.normalize();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  ros_path->poses.push_back(pose);
}

const ForwardHybridAStar::NodeRecord * ForwardHybridAStar::FindNode(std::size_t state_id) const
{
  const auto it = nodes_.find(state_id);
  if (it == nodes_.end()) {
    return nullptr;
  }
  return &it->second;
}

ForwardHybridAStar::NodeRecord * ForwardHybridAStar::FindNode(std::size_t state_id)
{
  const auto it = nodes_.find(state_id);
  if (it == nodes_.end()) {
    return nullptr;
  }
  return &it->second;
}

bool ForwardHybridAStar::ReconstructPath(
  std::size_t goal_state_id,
  const geometry_msgs::msg::PoseStamped & goal,
  double goal_yaw,
  bool has_goal_yaw,
  bool use_goal_heading,
  nav_msgs::msg::Path * ros_path) const
{
  if (ros_path == nullptr) {
    return false;
  }
  const NodeRecord * goal_node = FindNode(goal_state_id);
  if (goal_node == nullptr) {
    return false;
  }

  std::vector<std::size_t> state_chain;
  state_chain.reserve(256);

  std::size_t current_state_id = goal_state_id;
  std::size_t guard = 0;
  const std::size_t guard_limit = std::max<std::size_t>(nodes_.size(), 1);
  while (current_state_id != kInvalidStateId) {
    const NodeRecord * node = FindNode(current_state_id);
    if (node == nullptr) {
      return false;
    }

    state_chain.push_back(current_state_id);

    if (node->parent_state_id == kInvalidStateId) {
      break;
    }
    if (node->parent_state_id == current_state_id) {
      return false;
    }

    current_state_id = node->parent_state_id;
    ++guard;
    if (guard > guard_limit) {
      return false;
    }
  }

  if (state_chain.empty()) {
    return false;
  }

  std::reverse(state_chain.begin(), state_chain.end());

  ros_path->poses.clear();
  ros_path->header.frame_id = global_frame_;
  ros_path->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  const NodeRecord * start_node = FindNode(state_chain.front());
  if (start_node == nullptr) {
    return false;
  }
  AppendPose(start_node->x, start_node->y, start_node->z, start_node->yaw, ros_path);

  for (std::size_t i = 1; i < state_chain.size(); ++i) {
    const NodeRecord * parent = FindNode(state_chain[i - 1]);
    const NodeRecord * child = FindNode(state_chain[i]);
    if (parent == nullptr || child == nullptr) {
      return false;
    }

    if (child->primitive_index < 0) {
      AppendPose(child->x, child->y, child->z, child->yaw, ros_path);
      continue;
    }

    PrimitiveResult primitive_result;
    std::vector<SamplePose> sampled_poses;
    if (!RolloutPrimitive(
        *parent,
        child->primitive_index,
        start_node->x,
        start_node->y,
        start_node->yaw,
        &primitive_result,
        &sampled_poses))
    {
      AppendPose(child->x, child->y, child->z, child->yaw, ros_path);
      continue;
    }

    for (const SamplePose & sample_pose : sampled_poses) {
      AppendPose(sample_pose.x, sample_pose.y, sample_pose.z, sample_pose.yaw, ros_path);
    }
  }

  if (ros_path->poses.empty()) {
    return false;
  }

  const auto & last_pose = ros_path->poses.back();
  const double goal_distance = std::sqrt(
    SquaredDistance2D(
      goal.pose.position.x,
      goal.pose.position.y,
      last_pose.pose.position.x,
      last_pose.pose.position.y));
  if (goal_distance > 0.05 && goal_distance <= config_.goal_position_tolerance) {
    std::size_t projected_goal_index = 0;
    double projected_goal_z = goal.pose.position.z;
    if (!PoseToGroundIndex(
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z,
        &projected_goal_index,
        &projected_goal_z))
    {
      projected_goal_z = last_pose.pose.position.z;
    }

    double final_yaw = goal_node->yaw;
    if (use_goal_heading && has_goal_yaw) {
      final_yaw = goal_yaw;
    }
    AppendPose(
      goal.pose.position.x,
      goal.pose.position.y,
      projected_goal_z,
      final_yaw,
      ros_path);
  }

  return !ros_path->poses.empty();
}

bool ForwardHybridAStar::MakePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path * ros_path,
  bool force_position_only_goal,
  bool force_use_goal_heading,
  int preferred_initial_turn_sign,
  const CancelRequestedCallback & cancel_requested,
  bool * was_canceled,
  const std::string & debug_label,
  SearchDiagnostics * search_diagnostics,
  ProjectionDiagnostics * start_projection,
  ProjectionDiagnostics * goal_projection,
  const ProgressCallback & progress_callback)
{
  if (was_canceled != nullptr) {
    *was_canceled = false;
  }
  if (search_diagnostics != nullptr) {
    *search_diagnostics = SearchDiagnostics();
  }

  const auto planning_started_at = std::chrono::steady_clock::now();
  const auto elapsed_seconds =
    [&planning_started_at]() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - planning_started_at).count();
    };

  const auto update_search_diagnostics =
    [search_diagnostics](std::size_t expansions, double planning_time_sec, bool canceled, bool success, std::size_t path_pose_count) {
      if (search_diagnostics != nullptr) {
        search_diagnostics->expansions = expansions;
        search_diagnostics->planning_time_sec = planning_time_sec;
        search_diagnostics->canceled = canceled;
        search_diagnostics->success = success;
        search_diagnostics->path_pose_count = path_pose_count;
      }
    };

  const auto publish_progress =
    [&](std::size_t expansions, bool canceled) {
      const double planning_time_sec = elapsed_seconds();
      update_search_diagnostics(expansions, planning_time_sec, canceled, false, 0);
      if (progress_callback) {
        SearchDiagnostics progress;
        progress.expansions = expansions;
        progress.planning_time_sec = planning_time_sec;
        progress.canceled = canceled;
        progress.success = false;
        progress.path_pose_count = 0;
        progress_callback(progress);
      }
    };

  auto check_cancel_requested =
    [&cancel_requested, was_canceled]() -> bool {
      if (cancel_requested && cancel_requested()) {
        if (was_canceled != nullptr) {
          *was_canceled = true;
        }
        return true;
      }
      return false;
    };

  if (ros_path == nullptr) {
    RCLCPP_WARN(logger_, "Hybrid A*: ros_path pointer is null.");
    return false;
  }

  ros_path->poses.clear();
  ros_path->header.frame_id = global_frame_;
  ros_path->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  if (check_cancel_requested()) {
    if (!debug_label.empty()) {
      RCLCPP_WARN(
        logger_,
        "%s planning canceled, expansions=%zu, planning_time=%.3f",
        debug_label.c_str(),
        static_cast<std::size_t>(0),
        elapsed_seconds());
    } else {
      RCLCPP_INFO(logger_, "Hybrid A*: planning canceled before search start.");
    }
    update_search_diagnostics(0, elapsed_seconds(), true, false, 0);
    return false;
  }

  if (perception_3d_ == nullptr || pcl_ground_ == nullptr || kdtree_ground_ == nullptr) {
    RCLCPP_WARN(logger_, "Hybrid A*: planner context is not ready.");
    return false;
  }
  if (pcl_ground_->points.empty()) {
    RCLCPP_WARN(logger_, "Hybrid A*: ground point cloud is empty.");
    return false;
  }

  if (!IsFinite(start.pose.position.x) || !IsFinite(start.pose.position.y) ||
    !IsFinite(start.pose.position.z) || !IsFinite(goal.pose.position.x) ||
    !IsFinite(goal.pose.position.y) || !IsFinite(goal.pose.position.z))
  {
    RCLCPP_WARN(logger_, "Hybrid A*: start or goal pose has non-finite position.");
    return false;
  }

  double start_yaw = 0.0;
  if (!ExtractYaw(start, &start_yaw)) {
    start_yaw = std::atan2(
      goal.pose.position.y - start.pose.position.y,
      goal.pose.position.x - start.pose.position.x);
  }
  start_yaw = NormalizeAngle(start_yaw);

  double goal_yaw = 0.0;
  const bool has_goal_yaw = ExtractYaw(goal, &goal_yaw);
  const bool use_goal_heading =
    !force_position_only_goal && (force_use_goal_heading || config_.use_goal_heading);
  if (use_goal_heading && !has_goal_yaw) {
    RCLCPP_WARN(logger_, "Hybrid A*: use_goal_heading enabled but goal yaw is invalid.");
    return false;
  }
  if (preferred_initial_turn_sign > 0) {
    preferred_initial_turn_sign = 1;
  } else if (preferred_initial_turn_sign < 0) {
    preferred_initial_turn_sign = -1;
  } else {
    preferred_initial_turn_sign = 0;
  }

  std::size_t start_ground_index = 0;
  double start_projected_z = 0.0;
  if (!PoseToGroundIndex(
      start.pose.position.x,
      start.pose.position.y,
      start.pose.position.z,
      &start_ground_index,
      &start_projected_z,
      start_projection))
  {
    if (!debug_label.empty()) {
      RCLCPP_WARN(logger_, "%s projection failed for start", debug_label.c_str());
    } else {
      RCLCPP_WARN(logger_, "Hybrid A*: failed to project start pose onto ground.");
    }
    update_search_diagnostics(0, elapsed_seconds(), false, false, 0);
    return false;
  }

  std::size_t goal_ground_index = 0;
  double goal_projected_z = 0.0;
  if (!PoseToGroundIndex(
      goal.pose.position.x,
      goal.pose.position.y,
      goal.pose.position.z,
      &goal_ground_index,
      &goal_projected_z,
      goal_projection))
  {
    if (!debug_label.empty()) {
      RCLCPP_WARN(logger_, "%s projection failed for goal", debug_label.c_str());
    } else {
      RCLCPP_WARN(logger_, "Hybrid A*: failed to project goal pose onto ground.");
    }
    update_search_diagnostics(0, elapsed_seconds(), false, false, 0);
    return false;
  }

  if (!debug_label.empty() && start_projection != nullptr && goal_projection != nullptr) {
    RCLCPP_INFO(
      logger_,
      "%s planning start: start_world=(%.2f, %.2f, %.2f) -> ground=(%.2f, %.2f, %.2f) idx=%zu dist=%.3f fallback=%s | goal_world=(%.2f, %.2f, %.2f) -> ground=(%.2f, %.2f, %.2f) idx=%zu dist=%.3f fallback=%s",
      debug_label.c_str(),
      start_projection->query_x,
      start_projection->query_y,
      start_projection->query_z,
      start_projection->ground_x,
      start_projection->ground_y,
      start_projection->ground_z,
      start_projection->ground_index,
      start_projection->projection_distance,
      start_projection->fallback.c_str(),
      goal_projection->query_x,
      goal_projection->query_y,
      goal_projection->query_z,
      goal_projection->ground_x,
      goal_projection->ground_y,
      goal_projection->ground_z,
      goal_projection->ground_index,
      goal_projection->projection_distance,
      goal_projection->fallback.c_str());
    RCLCPP_INFO(
      logger_,
      "%s start projection detail: fallback=%s, z_before=%.2f, z_after=%.2f, nearest_ground_distance=%.3f",
      debug_label.c_str(),
      start_projection->fallback.c_str(),
      start_projection->fallback_z_before,
      start_projection->fallback_z_after,
      start_projection->fallback_nearest_distance);
    RCLCPP_INFO(
      logger_,
      "%s goal projection detail: fallback=%s, z_before=%.2f, z_after=%.2f, nearest_ground_distance=%.3f",
      debug_label.c_str(),
      goal_projection->fallback.c_str(),
      goal_projection->fallback_z_before,
      goal_projection->fallback_z_after,
      goal_projection->fallback_nearest_distance);
  }

  publish_progress(0, false);

  (void)goal_ground_index;
  (void)goal_projected_z;

  const std::size_t heading_bin_size = static_cast<std::size_t>(config_.heading_bin_count);
  if (heading_bin_size == 0) {
    return false;
  }

  const std::size_t ground_size = pcl_ground_->points.size();
  const std::size_t max_size = std::numeric_limits<std::size_t>::max();
  if (ground_size > max_size / heading_bin_size) {
    RCLCPP_WARN(logger_, "Hybrid A*: state space overflow.");
    return false;
  }

  const std::size_t state_space_size = ground_size * heading_bin_size;
  nodes_.clear();
  nodes_.reserve(std::min<std::size_t>(state_space_size, 200000));

  const int start_heading_bin = QuantizeYaw(start_yaw);
  std::size_t start_state_id = kInvalidStateId;
  if (!ComputeStateId(start_ground_index, start_heading_bin, &start_state_id)) {
    RCLCPP_WARN(logger_, "Hybrid A*: failed to compute start state id.");
    return false;
  }

  NodeRecord start_node;
  start_node.opened = true;
  start_node.closed = false;
  start_node.g = 0.0;
  start_node.h = ComputeHeuristic(
    start.pose.position.x,
    start.pose.position.y,
    start_yaw,
    goal,
    goal_yaw,
    has_goal_yaw,
    use_goal_heading);
  start_node.f = start_node.g + start_node.h;
  start_node.parent_state_id = kInvalidStateId;
  start_node.primitive_index = -1;
  start_node.initial_turn_sign = 0;
  start_node.ground_index = start_ground_index;
  start_node.heading_bin = start_heading_bin;
  start_node.x = start.pose.position.x;
  start_node.y = start.pose.position.y;
  start_node.z = start_projected_z;
  start_node.yaw = start_yaw;
  nodes_.emplace(start_state_id, start_node);

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open_list;
  open_list.push(OpenEntry{start_node.f, start_state_id});

  std::size_t goal_state_id = kInvalidStateId;

  std::size_t expand_count = 0;
  const std::size_t max_expand_count = std::max<std::size_t>(state_space_size, 1);
  while (!open_list.empty()) {
    if (check_cancel_requested()) {
      if (!debug_label.empty()) {
        RCLCPP_WARN(
          logger_,
          "%s planning canceled, expansions=%zu, planning_time=%.3f",
          debug_label.c_str(),
          expand_count,
          elapsed_seconds());
      } else {
        RCLCPP_INFO(
          logger_,
          "Hybrid A*: planning canceled during search after %zu expansions.",
          expand_count);
      }
      update_search_diagnostics(expand_count, elapsed_seconds(), true, false, 0);
      publish_progress(expand_count, true);
      return false;
    }

    const OpenEntry top_entry = open_list.top();
    open_list.pop();

    NodeRecord * current_ptr = FindNode(top_entry.state_id);
    if (current_ptr == nullptr) {
      continue;
    }
    if (!current_ptr->opened || current_ptr->closed) {
      continue;
    }
    if (!IsFinite(current_ptr->f) || top_entry.f > current_ptr->f + 1e-9) {
      continue;
    }

    current_ptr->closed = true;
    const NodeRecord current = *current_ptr;

    if (IsGoalReached(current, goal, goal_yaw, has_goal_yaw, use_goal_heading)) {
      goal_state_id = top_entry.state_id;
      break;
    }

    for (int primitive_index = 0;
      primitive_index < static_cast<int>(primitive_steers_.size());
      ++primitive_index)
    {
      PrimitiveResult primitive_result;
      if (!RolloutPrimitive(
          current,
          primitive_index,
          start.pose.position.x,
          start.pose.position.y,
          start_yaw,
          &primitive_result,
          nullptr))
      {
        continue;
      }

      std::size_t next_state_id = kInvalidStateId;
      if (!ComputeStateId(
          primitive_result.end_ground_index,
          primitive_result.end_heading_bin,
          &next_state_id))
      {
        continue;
      }

      NodeRecord * neighbor = FindNode(next_state_id);
      if (neighbor != nullptr && neighbor->closed) {
        continue;
      }

      const double transition_cost =
        ComputeTransitionCost(
        current, primitive_index, primitive_result, preferred_initial_turn_sign);
      const double tentative_g = current.g + transition_cost;
      if (neighbor != nullptr && neighbor->opened && tentative_g >= neighbor->g - 1e-9) {
        continue;
      }

      NodeRecord updated_neighbor;
      if (neighbor != nullptr) {
        updated_neighbor = *neighbor;
      }
      updated_neighbor.opened = true;
      updated_neighbor.closed = false;
      updated_neighbor.g = tentative_g;
      updated_neighbor.h = ComputeHeuristic(
        primitive_result.end_x,
        primitive_result.end_y,
        primitive_result.end_yaw,
        goal,
        goal_yaw,
        has_goal_yaw,
        use_goal_heading);
      updated_neighbor.f = updated_neighbor.g + updated_neighbor.h;
      updated_neighbor.parent_state_id = top_entry.state_id;
      updated_neighbor.primitive_index = primitive_index;
      updated_neighbor.initial_turn_sign = current.initial_turn_sign;
      const int primitive_turn_sign = GetPrimitiveTurnSign(primitive_index);
      if (updated_neighbor.initial_turn_sign == 0 && primitive_turn_sign != 0) {
        updated_neighbor.initial_turn_sign = primitive_turn_sign;
      }
      updated_neighbor.ground_index = primitive_result.end_ground_index;
      updated_neighbor.heading_bin = primitive_result.end_heading_bin;
      updated_neighbor.x = primitive_result.end_x;
      updated_neighbor.y = primitive_result.end_y;
      updated_neighbor.z = primitive_result.end_z;
      updated_neighbor.yaw = primitive_result.end_yaw;

      if (neighbor != nullptr) {
        *neighbor = updated_neighbor;
      } else {
        nodes_.emplace(next_state_id, updated_neighbor);
      }

      open_list.push(OpenEntry{updated_neighbor.f, next_state_id});
    }

    ++expand_count;
    if (progress_callback && (expand_count == 1 || expand_count % 500 == 0)) {
      publish_progress(expand_count, false);
    }
    if (expand_count > max_expand_count) {
      break;
    }
  }

  if (goal_state_id == kInvalidStateId) {
    if (!debug_label.empty()) {
      RCLCPP_WARN(
        logger_,
        "%s planning finished, success=0, expansions=%zu, planning_time=%.3f, path_poses=%zu",
        debug_label.c_str(),
        expand_count,
        elapsed_seconds(),
        static_cast<std::size_t>(0));
    }
    update_search_diagnostics(expand_count, elapsed_seconds(), false, false, 0);
    return false;
  }

  const bool reconstructed = ReconstructPath(
    goal_state_id,
    goal,
    goal_yaw,
    has_goal_yaw,
    use_goal_heading,
    ros_path);
  if (!debug_label.empty()) {
    RCLCPP_INFO(
      logger_,
      "%s planning finished, success=%d, expansions=%zu, planning_time=%.3f, path_poses=%zu",
      debug_label.c_str(),
      reconstructed ? 1 : 0,
      expand_count,
      elapsed_seconds(),
      ros_path != nullptr ? ros_path->poses.size() : 0);
  }
  update_search_diagnostics(
    expand_count,
    elapsed_seconds(),
    false,
    reconstructed,
    ros_path != nullptr ? ros_path->poses.size() : 0);
  return reconstructed;
}

}  // namespace global_planner
