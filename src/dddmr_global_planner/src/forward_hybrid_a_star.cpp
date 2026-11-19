#include <global_planner/forward_hybrid_a_star.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr std::size_t kHybridRefineSegmentMaxExpansions = 2500;

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

double ElapsedMs(const std::chrono::steady_clock::time_point & started_at)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started_at).count();
}

// Per-thread, per-MakePlan accumulation. ValidateSample() is only called from
// the MakePlan() search on the same thread, so this avoids cross-request races
// without adding a lock to the hot path.
struct ThreadDGraphTiming
{
  std::size_t calls = 0;
  double total_ms = 0.0;
  double max_ms = 0.0;
  std::size_t over_1ms = 0;
  std::size_t batch_lock_calls = 0;
  double batch_lock_wait_ms = 0.0;
  double batch_lock_max_ms = 0.0;

  void reset()
  {
    calls = 0;
    total_ms = 0.0;
    max_ms = 0.0;
    over_1ms = 0;
    batch_lock_calls = 0;
    batch_lock_wait_ms = 0.0;
    batch_lock_max_ms = 0.0;
  }

  void add(double elapsed_ms)
  {
    ++calls;
    total_ms += elapsed_ms;
    if (elapsed_ms > max_ms) {
      max_ms = elapsed_ms;
    }
    if (elapsed_ms > 1.0) {
      ++over_1ms;
    }
  }

  void addBatchLock(double elapsed_ms)
  {
    ++batch_lock_calls;
    batch_lock_wait_ms += elapsed_ms;
    if (elapsed_ms > batch_lock_max_ms) {
      batch_lock_max_ms = elapsed_ms;
    }
  }
};

thread_local ThreadDGraphTiming g_dgraph_timing;

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
  config_.steer_sample_count = std::clamp(config_.steer_sample_count, 3, 9);
  if ((config_.steer_sample_count % 2) == 0) {
    ++config_.steer_sample_count;
    if (config_.steer_sample_count > 9) {
      config_.steer_sample_count = 9;
    }
  }
  config_.heading_bin_count = std::max(config_.heading_bin_count, 8);
  config_.primitive_length = std::max(config_.primitive_length, 1e-3);
  config_.primitive_step = std::max(config_.primitive_step, 1e-3);
  config_.projection_search_radius = std::max(config_.projection_search_radius, 1e-3);
  config_.goal_position_tolerance = std::max(config_.goal_position_tolerance, 1e-3);
  config_.goal_heading_tolerance = std::max(config_.goal_heading_tolerance, 1e-3);
  config_.edge_weight_penalty_weight = std::max(config_.edge_weight_penalty_weight, 0.0);
  config_.edge_weight_safe_threshold = std::max(config_.edge_weight_safe_threshold, 0.0);
  config_.edge_weight_soft_cap = std::max(config_.edge_weight_soft_cap, 0.0);
  config_.edge_weight_hard_reject_threshold =
    std::max(config_.edge_weight_hard_reject_threshold, 0.0);
  config_.max_projected_pitch = std::clamp(config_.max_projected_pitch, 0.05, 1.4);
  config_.max_projected_vertical_jump = std::max(config_.max_projected_vertical_jump, 0.02);

  primitive_steers_.clear();
  primitive_steers_.reserve(static_cast<std::size_t>(config_.steer_sample_count));
  const int midpoint = config_.steer_sample_count / 2;
  for (int i = 0; i < config_.steer_sample_count; ++i) {
    if (midpoint == 0) {
      primitive_steers_.push_back(0.0);
      continue;
    }
    const double normalized =
      static_cast<double>(i - midpoint) / static_cast<double>(midpoint);
    primitive_steers_.push_back(normalized * config_.max_steer);
  }
}

void ForwardHybridAStar::SetGlobalFrame(const std::string & global_frame)
{
  global_frame_ = global_frame;
}

bool ForwardHybridAStar::SetSceneSnapshot(
  const ScenePipelineContext & scene,
  std::string * rejection_reason)
{
  scene_.Reset();
  if (!scene.valid()) {
    if (rejection_reason != nullptr) {
      *rejection_reason = scene.rejectionReason().empty() ?
        "pinned scene is not valid" : scene.rejectionReason();
    }
    return false;
  }
  scene_ = scene;
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

void ForwardHybridAStar::ClearSceneSnapshot()
{
  scene_.Reset();
}

const pcl::PointCloud<pcl::PointXYZI> * ForwardHybridAStar::GroundCloud() const
{
  if (scene_.valid()) {
    return scene_.staticView() ? scene_.staticView()->groundPoints().get() : nullptr;
  }
  return pcl_ground_.get();
}

std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>>
ForwardHybridAStar::GroundTree() const
{
  if (scene_.valid()) {
    return scene_.staticView() ? scene_.staticView()->groundKdtree() : nullptr;
  }
  return kdtree_ground_;
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
  const auto * ground_cloud = GroundCloud();
  if (ground_cloud == nullptr || ground_index >= ground_cloud->points.size()) {
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
  std::size_t previous_ground_index,
  bool allow_nearest_fallback,
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

  const auto * ground_cloud = GroundCloud();
  const auto ground_tree = GroundTree();
  if (ground_tree == nullptr || ground_cloud == nullptr || ground_cloud->points.empty()) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "ground_unavailable";
    }
    return false;
  }

  const auto select_candidate =
    [this, &ground_cloud, x, y, z_hint, previous_ground_index](
    const std::vector<int> & candidate_indices,
    const std::vector<float> & candidate_squared_distances,
    std::size_t * selected_ground_index,
    float * selected_squared_distance) -> bool {
      if (selected_ground_index == nullptr || selected_squared_distance == nullptr) {
        return false;
      }
      if (candidate_indices.empty()) {
        return false;
      }

      double best_score = std::numeric_limits<double>::infinity();
      bool found = false;
      std::size_t best_ground_index = 0;
      float best_squared_distance = 0.0f;

      for (std::size_t i = 0; i < candidate_indices.size(); ++i) {
        const int candidate_index_signed = candidate_indices[i];
        if (candidate_index_signed < 0) {
          continue;
        }
        const std::size_t candidate_index = static_cast<std::size_t>(candidate_index_signed);
        if (candidate_index >= ground_cloud->points.size()) {
          continue;
        }
        if (!IsGroundIndexInWeightMap(candidate_index)) {
          continue;
        }
        if (previous_ground_index != kInvalidStateId &&
          !IsProjectedGroundTransitionValid(previous_ground_index, candidate_index))
        {
          continue;
        }

        const pcl::PointXYZI & candidate_pt = ground_cloud->points[candidate_index];
        if (!IsFinite(candidate_pt.x) || !IsFinite(candidate_pt.y) || !IsFinite(candidate_pt.z)) {
          continue;
        }

        const double dx = static_cast<double>(candidate_pt.x) - x;
        const double dy = static_cast<double>(candidate_pt.y) - y;
        const double dz = static_cast<double>(candidate_pt.z) - z_hint;
        const double score = dx * dx + dy * dy + 9.0 * dz * dz;
        if (!std::isfinite(score)) {
          continue;
        }

        if (!found || score < best_score) {
          best_score = score;
          best_ground_index = candidate_index;
          if (i < candidate_squared_distances.size()) {
            best_squared_distance = candidate_squared_distances[i];
          } else {
            best_squared_distance = static_cast<float>(dx * dx + dy * dy + dz * dz);
          }
          found = true;
        }
      }

      if (!found) {
        return false;
      }

      *selected_ground_index = best_ground_index;
      *selected_squared_distance = best_squared_distance;
      return true;
    };

  pcl::PointXYZI query;
  query.x = static_cast<float>(x);
  query.y = static_cast<float>(y);
  query.z = static_cast<float>(z_hint);
  query.intensity = 0.0f;

  std::vector<int> point_indices;
  std::vector<float> point_squared_distances;

  const int found_in_radius =
    ground_tree->radiusSearch(
    query,
    config_.projection_search_radius,
    point_indices,
    point_squared_distances);

  if (found_in_radius > 0 && !point_indices.empty() && !point_squared_distances.empty()) {
    std::size_t selected_ground_index = 0;
    float selected_squared_distance = 0.0f;
    if (select_candidate(
        point_indices,
        point_squared_distances,
        &selected_ground_index,
        &selected_squared_distance))
    {
      const pcl::PointXYZI & selected_pt = ground_cloud->points[selected_ground_index];
      *ground_index = selected_ground_index;
      *projected_z = selected_pt.z;
      if (diagnostics != nullptr) {
        diagnostics->success = true;
        diagnostics->ground_index = *ground_index;
        diagnostics->ground_x = selected_pt.x;
        diagnostics->ground_y = selected_pt.y;
        diagnostics->ground_z = selected_pt.z;
        diagnostics->projection_distance =
          Distance3D(x, y, z_hint, selected_pt.x, selected_pt.y, selected_pt.z);
        diagnostics->fallback = "none";
        diagnostics->fallback_z_before = z_hint;
        diagnostics->fallback_z_after = selected_pt.z;
        diagnostics->fallback_nearest_distance = std::sqrt(selected_squared_distance);
      }
      return true;
    }

    if (diagnostics != nullptr) {
      diagnostics->fallback = "radius_candidates_rejected";
    }
    if (!allow_nearest_fallback) {
      return false;
    }
  } else if (!allow_nearest_fallback) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "radius_not_found";
    }
    return false;
  }

  point_indices.clear();
  point_squared_distances.clear();
  if (ground_tree->nearestKSearch(query, 8, point_indices, point_squared_distances) <= 0) {
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

  std::size_t nearest_index = 0;
  float nearest_squared_distance = 0.0f;
  if (!select_candidate(
      point_indices,
      point_squared_distances,
      &nearest_index,
      &nearest_squared_distance))
  {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_rejected";
    }
    return false;
  }

  const double nearest_distance = std::sqrt(nearest_squared_distance);
  if (!IsFinite(nearest_distance) || nearest_distance > config_.projection_search_radius * 2.0) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_rejected_too_far";
      diagnostics->fallback_nearest_distance = nearest_distance;
    }
    return false;
  }

  const pcl::PointXYZI & nearest_pt = ground_cloud->points[nearest_index];
  if (!IsFinite(nearest_pt.z)) {
    if (diagnostics != nullptr) {
      diagnostics->fallback = "nearest_k_invalid_projected_point";
    }
    return false;
  }

  *ground_index = nearest_index;
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

bool ForwardHybridAStar::IsGroundIndexInWeightMap(std::size_t ground_index) const
{
  return weight_map_size_ > 0 && ground_index < weight_map_size_;
}

bool ForwardHybridAStar::ProjectSample(
  double x,
  double y,
  double yaw,
  double z_hint,
  std::size_t * ground_index,
  double * projected_z,
  std::size_t previous_ground_index,
  bool allow_nearest_fallback) const
{
  if (ground_index == nullptr || projected_z == nullptr) {
    return false;
  }
  if (!scene_.valid() && (perception_3d_ == nullptr || pcl_ground_ == nullptr)) {
    return false;
  }

  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(yaw) || !IsFinite(z_hint)) {
    return false;
  }

  std::size_t projected_ground_index = 0;
  double projected_ground_z = 0.0;
  if (!PoseToGroundIndex(
      x,
      y,
      z_hint,
      &projected_ground_index,
      &projected_ground_z,
      previous_ground_index,
      allow_nearest_fallback))
  {
    return false;
  }

  const auto * ground_cloud = GroundCloud();
  if (ground_cloud == nullptr || projected_ground_index >= ground_cloud->points.size()) {
    return false;
  }
  if (!IsGroundIndexInWeightMap(projected_ground_index)) {
    return false;
  }
  const pcl::PointXYZI & projected_pt = ground_cloud->points[projected_ground_index];
  if (!IsFinite(projected_pt.x) || !IsFinite(projected_pt.y) || !IsFinite(projected_pt.z)) {
    return false;
  }
  const double max_projection_distance = config_.projection_search_radius * 1.5;
  if (SquaredDistance2D(x, y, projected_pt.x, projected_pt.y) >
    max_projection_distance * max_projection_distance)
  {
    return false;
  }

  *ground_index = projected_ground_index;
  *projected_z = projected_ground_z;
  return true;
}

bool ForwardHybridAStar::ReadSampleDGraph(
  std::size_t ground_index,
  double * dgraph_value) const
{
  if (dgraph_value == nullptr) {
    return false;
  }
  if (!scene_.valid() && perception_3d_ == nullptr) {
    return false;
  }

  const auto dgraph_query_started_at = std::chrono::steady_clock::now();
  const double dgraph = scene_.valid() ?
    scene_.costView()->minDGraphValue(ground_index) :
    perception_3d_->get_min_dGraphValue(static_cast<unsigned int>(ground_index));
  g_dgraph_timing.add(ElapsedMs(dgraph_query_started_at));
  const double inscribed_radius = scene_.valid() ?
    scene_.parameters().inscribed_radius :
    perception_3d_->getGlobalUtils()->getInscribedRadius();

  if (!IsFinite(dgraph) || !IsFinite(inscribed_radius)) {
    return false;
  }

  if (dgraph < inscribed_radius) {
    return false;
  }

  *dgraph_value = dgraph;
  return true;
}

bool ForwardHybridAStar::ValidateSample(
  double x,
  double y,
  double yaw,
  double z_hint,
  std::size_t * ground_index,
  double * projected_z,
  double * dgraph_value,
  std::size_t previous_ground_index,
  bool allow_nearest_fallback) const
{
  if (ground_index == nullptr || projected_z == nullptr || dgraph_value == nullptr) {
    return false;
  }

  std::size_t projected_ground_index = 0;
  double projected_ground_z = 0.0;
  if (!ProjectSample(
      x, y, yaw, z_hint,
      &projected_ground_index, &projected_ground_z,
      previous_ground_index, allow_nearest_fallback))
  {
    return false;
  }

  double dgraph = 0.0;
  if (!ReadSampleDGraph(projected_ground_index, &dgraph)) {
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
  const auto * ground_cloud = GroundCloud();
  if (ground_cloud == nullptr) {
    return false;
  }
  if (from_ground_index >= ground_cloud->points.size() ||
    to_ground_index >= ground_cloud->points.size())
  {
    return false;
  }
  if (from_ground_index == to_ground_index) {
    return true;
  }
  if (!IsGroundIndexInWeightMap(from_ground_index) ||
    !IsGroundIndexInWeightMap(to_ground_index))
  {
    return false;
  }

  const pcl::PointXYZI & from_pt = ground_cloud->points[from_ground_index];
  const pcl::PointXYZI & to_pt = ground_cloud->points[to_ground_index];
  if (!IsFinite(from_pt.x) || !IsFinite(from_pt.y) || !IsFinite(from_pt.z) ||
    !IsFinite(to_pt.x) || !IsFinite(to_pt.y) || !IsFinite(to_pt.z))
  {
    return false;
  }

  const double dx = static_cast<double>(to_pt.x) - static_cast<double>(from_pt.x);
  const double dy = static_cast<double>(to_pt.y) - static_cast<double>(from_pt.y);
  const double dz = std::abs(static_cast<double>(to_pt.z) - static_cast<double>(from_pt.z));
  const double planar_distance = std::hypot(dx, dy);
  if (planar_distance < 1e-6) {
    return dz <= config_.max_projected_vertical_jump;
  }

  const double max_transition_pitch_tan = std::tan(config_.max_projected_pitch);
  if (dz > config_.max_projected_vertical_jump + max_transition_pitch_tan * planar_distance) {
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

double ForwardHybridAStar::ComputeTransitionCost(
  const NodeRecord & parent,
  int primitive_index,
  const PrimitiveResult & primitive_result) const
{
  (void)parent;
  (void)primitive_index;
  double cost = primitive_result.path_length;
  cost += config_.obstacle_penalty_weight * primitive_result.obstacle_penalty;
  cost += config_.edge_weight_penalty_weight * primitive_result.edge_penalty;

  return std::max(cost, 1e-6);
}

bool ForwardHybridAStar::RolloutPrimitive(
  const NodeRecord & parent,
  int primitive_index,
  PrimitiveResult * primitive_result,
  std::vector<SamplePose> * sampled_poses) const
{
  if (primitive_result == nullptr) {
    return false;
  }
  const bool pinned = scene_.valid();
  if (!pinned && (perception_3d_ == nullptr || pcl_ground_ == nullptr)) {
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

  const double inscribed_radius = pinned ?
    scene_.parameters().inscribed_radius :
    perception_3d_->getGlobalUtils()->getInscribedRadius();
  const double inflation_descending_rate = pinned ?
    scene_.parameters().inflation_descending_rate :
    perception_3d_->getGlobalUtils()->getInflationDescendingRate();

  double obstacle_penalty_sum = 0.0;
  double edge_penalty_sum = 0.0;
  std::size_t sample_count = 0;
  std::size_t end_ground_index = parent.ground_index;
  double end_projected_z = z;
  std::size_t previous_ground_index = parent.ground_index;

  const auto * ground_cloud = GroundCloud();
  if (ground_cloud == nullptr || previous_ground_index >= ground_cloud->points.size()) {
    return false;
  }

  struct ProjectedSample
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double projected_z = 0.0;
    std::size_t ground_index = 0;
  };

  // Phase 1: project candidate poses and validate geometry against the
  // planner's private ground copy, outside the shared-map guard.
  std::vector<ProjectedSample> projected_samples;
  projected_samples.reserve(64);
  while (remaining > 1e-9) {
    const double ds = std::min(step, remaining);
    x += ds * std::cos(yaw);
    y += ds * std::sin(yaw);
    yaw = NormalizeAngle(yaw + ds / config_.wheelbase * std::tan(steer));

    ProjectedSample sample;
    sample.x = x;
    sample.y = y;
    sample.yaw = yaw;
    if (!ProjectSample(
        x, y, yaw, z,
        &sample.ground_index, &sample.projected_z,
        previous_ground_index,
        config_.allow_sample_nearest_fallback))
    {
      return false;
    }

    z = sample.projected_z;
    previous_ground_index = sample.ground_index;
    projected_samples.push_back(sample);

    remaining -= ds;
  }

  // Phase 2: read the pinned cost/static views only. The legacy branch keeps
  // the original shared-map guard exactly as before.
  std::vector<double> sample_dgraph(projected_samples.size(), 0.0);
  std::vector<double> sample_edge_weight(projected_samples.size(), 0.0);
  if (pinned) {
    for (std::size_t i = 0; i < projected_samples.size(); ++i) {
      const std::size_t sample_ground_index = projected_samples[i].ground_index;
      double dgraph = 0.0;
      if (!ReadSampleDGraph(sample_ground_index, &dgraph)) {
        return false;
      }
      const double sample_edge = std::max(
        0.0,
        static_cast<double>(scene_.staticView()->graph().getNodeWeight(
            static_cast<unsigned int>(sample_ground_index))));
      if (!IsFinite(sample_edge)) {
        return false;
      }
      if (config_.edge_weight_hard_reject_threshold > 0.0 &&
        sample_edge >= config_.edge_weight_hard_reject_threshold)
      {
        return false;
      }
      sample_dgraph[i] = dgraph;
      sample_edge_weight[i] = sample_edge;
    }
  } else {
    const auto shared_data = perception_3d_->getSharedDataPtr();
    if (shared_data == nullptr || shared_data->sGraph_ptr_ == nullptr) {
      return false;
    }
    const auto batch_lock_started_at = std::chrono::steady_clock::now();
    std::unique_lock<std::recursive_mutex> batch_lock(shared_data->ground_kdtree_cb_mutex_);
    g_dgraph_timing.addBatchLock(ElapsedMs(batch_lock_started_at));

    for (std::size_t i = 0; i < projected_samples.size(); ++i) {
      const std::size_t sample_ground_index = projected_samples[i].ground_index;
      double dgraph = 0.0;
      if (!ReadSampleDGraph(sample_ground_index, &dgraph)) {
        return false;
      }
      const double sample_edge = std::max(
        0.0,
        static_cast<double>(shared_data->sGraph_ptr_->getNodeWeight(
            static_cast<unsigned int>(sample_ground_index))));
      if (!IsFinite(sample_edge)) {
        return false;
      }
      if (config_.edge_weight_hard_reject_threshold > 0.0 &&
        sample_edge >= config_.edge_weight_hard_reject_threshold)
      {
        return false;
      }
      sample_dgraph[i] = dgraph;
      sample_edge_weight[i] = sample_edge;
    }
  }

  // Phase 3: accumulate penalties and emitted poses outside the guard.
  for (std::size_t i = 0; i < projected_samples.size(); ++i) {
    const ProjectedSample & sample = projected_samples[i];
    double edge_over = sample_edge_weight[i] - config_.edge_weight_safe_threshold;
    if (edge_over > 0.0) {
      if (config_.edge_weight_soft_cap > 0.0) {
        edge_over = std::min(edge_over, config_.edge_weight_soft_cap);
      }
      edge_penalty_sum += edge_over;
    }

    if (sampled_poses != nullptr) {
      sampled_poses->push_back(SamplePose{sample.x, sample.y, sample.projected_z, sample.yaw});
    }

    end_ground_index = sample.ground_index;
    end_projected_z = sample.projected_z;

    const double obstacle_factor =
      std::exp(-1.0 * inflation_descending_rate * (sample_dgraph[i] - inscribed_radius));
    obstacle_penalty_sum += obstacle_factor;

    ++sample_count;
  }

  primitive_result->end_x = x;
  primitive_result->end_y = y;
  primitive_result->end_z = end_projected_z;
  primitive_result->end_yaw = yaw;
  primitive_result->end_ground_index = end_ground_index;
  primitive_result->end_heading_bin = QuantizeYaw(yaw);
  primitive_result->path_length = config_.primitive_length;
  primitive_result->obstacle_penalty =
    sample_count > 0 ? obstacle_penalty_sum / static_cast<double>(sample_count) : 0.0;
  primitive_result->edge_penalty =
    sample_count > 0 ? edge_penalty_sum / static_cast<double>(sample_count) : 0.0;

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
  g_dgraph_timing.reset();
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
        search_diagnostics->dgraph_calls = g_dgraph_timing.calls;
        search_diagnostics->dgraph_query_ms = g_dgraph_timing.total_ms;
        search_diagnostics->dgraph_max_ms = g_dgraph_timing.max_ms;
        search_diagnostics->dgraph_over_1ms = g_dgraph_timing.over_1ms;
        search_diagnostics->batch_lock_calls = g_dgraph_timing.batch_lock_calls;
        search_diagnostics->batch_lock_wait_ms = g_dgraph_timing.batch_lock_wait_ms;
        search_diagnostics->batch_lock_max_ms = g_dgraph_timing.batch_lock_max_ms;
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

  const bool pinned = scene_.valid();
  if (!pinned &&
    (perception_3d_ == nullptr || pcl_ground_ == nullptr || kdtree_ground_ == nullptr))
  {
    RCLCPP_WARN(logger_, "Hybrid A*: planner context is not ready.");
    return false;
  }
  const auto * ground_cloud = GroundCloud();
  if (ground_cloud == nullptr || ground_cloud->points.empty()) {
    RCLCPP_WARN(logger_, "Hybrid A*: ground point cloud is empty.");
    return false;
  }
  weight_map_size_ = 0;
  if (pinned) {
    weight_map_size_ =
      static_cast<std::size_t>(scene_.staticView()->graph().nodeWeightCount());
  } else {
    const auto shared_data = perception_3d_->getSharedDataPtr();
    if (shared_data != nullptr && shared_data->sGraph_ptr_ != nullptr) {
      weight_map_size_ =
        static_cast<std::size_t>(shared_data->sGraph_ptr_->getNodeWeightSize());
    }
  }
  if (weight_map_size_ == 0) {
    RCLCPP_WARN(logger_, "Hybrid A*: ground weight map is empty, reject planning request.");
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
    RCLCPP_WARN(logger_, "Hybrid A*: start yaw is invalid.");
    return false;
  }
  start_yaw = NormalizeAngle(start_yaw);

  double goal_yaw = 0.0;
  const bool has_goal_yaw = ExtractYaw(goal, &goal_yaw);
  const bool use_goal_heading =
    !force_position_only_goal && force_use_goal_heading;
  if (use_goal_heading && !has_goal_yaw) {
    RCLCPP_WARN(logger_, "Hybrid A*: forced goal heading enabled but goal yaw is invalid.");
    return false;
  }
  std::size_t start_ground_index = 0;
  double start_projected_z = 0.0;
  const auto projection_started_at = std::chrono::steady_clock::now();
  if (!PoseToGroundIndex(
      start.pose.position.x,
      start.pose.position.y,
      start.pose.position.z,
      &start_ground_index,
      &start_projected_z,
      kInvalidStateId,
      true,
      start_projection))
  {
    if (search_diagnostics != nullptr) {
      search_diagnostics->projection_ms = ElapsedMs(projection_started_at);
    }
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
      kInvalidStateId,
      true,
      goal_projection))
  {
    if (search_diagnostics != nullptr) {
      search_diagnostics->projection_ms = ElapsedMs(projection_started_at);
    }
    if (!debug_label.empty()) {
      RCLCPP_WARN(logger_, "%s projection failed for goal", debug_label.c_str());
    } else {
      RCLCPP_WARN(logger_, "Hybrid A*: failed to project goal pose onto ground.");
    }
    update_search_diagnostics(0, elapsed_seconds(), false, false, 0);
    return false;
  }
  if (search_diagnostics != nullptr) {
    search_diagnostics->projection_ms = ElapsedMs(projection_started_at);
  }

  if (!debug_label.empty() && start_projection != nullptr && goal_projection != nullptr) {
    RCLCPP_DEBUG(
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
    RCLCPP_DEBUG(
      logger_,
      "%s start projection detail: fallback=%s, z_before=%.2f, z_after=%.2f, nearest_ground_distance=%.3f",
      debug_label.c_str(),
      start_projection->fallback.c_str(),
      start_projection->fallback_z_before,
      start_projection->fallback_z_after,
      start_projection->fallback_nearest_distance);
    RCLCPP_DEBUG(
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

  const std::size_t ground_size = ground_cloud->points.size();
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
  std::size_t max_expand_count = std::max<std::size_t>(state_space_size, 1);
  if(debug_label.find("hybrid_refine_segment") != std::string::npos){
    max_expand_count = std::min(max_expand_count, kHybridRefineSegmentMaxExpansions);
  }
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
        ComputeTransitionCost(current, primitive_index, primitive_result);
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
      RCLCPP_DEBUG(
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
    RCLCPP_DEBUG(
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
