#include "local_planner/mppi_directed_retreat.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Search shape. The extent and node budget are local visibility/compute bounds,
// not an accumulated reverse-distance or elapsed-time limit.
constexpr double kReverseRootLength = 0.35;
constexpr int kReverseRootSteps = 8;
constexpr double kSearchSegmentLength = 0.28;
constexpr double kSearchSweepResolution = 0.04;
// Sampling bound covers translation plus the rotational displacement of a
// production-sized body. The backend final rollout remains independently
// mandatory.
constexpr double kSearchBodySweepRadius = 1.5;
constexpr double kSearchHalfExtent = 8.0;
constexpr double kSearchMaxCost = 17.0;
constexpr int kSearchMaxExpansions = 12000;
constexpr auto kSearchTimeSlice = std::chrono::milliseconds(12);
constexpr double kRootCostFactor = 1.15;
constexpr double kHeuristicDistanceWeight = 2.0;
constexpr double kHeuristicYawWeight = 0.8;
constexpr double kSegmentCurvatureCost = 0.012;
constexpr double kSearchStartResetDistance = 0.10;
constexpr double kSearchStartResetYaw = 0.10;
constexpr double kKeyPositionResolution = 0.16;
constexpr double kKeyYawResolution = 0.12;
constexpr std::size_t kSearchNodeReserve = 22000;

// The downstream target must be beyond the contiguous blocked region and
// followed by more free route. A tiny collision-free forward movement after
// the reverse landing is explicitly not enough to end retreat.
constexpr double kDownstreamTargetDistance = 2.0;
constexpr double kDownstreamContinuationDistance = 0.5;
constexpr double kMinimumDownstreamDistance = 1.8;
constexpr double kMinimumContinuationDistance = 0.45;
constexpr double kMaximumStartGoalDistance = 9.0;

// Rejoin and tracking thresholds. Search termination is strictly inside the
// final completion envelope so a selected approximate endpoint can never be
// unreachable by the tracker's own completion check.
constexpr double kRejoinPositionTolerance = 0.22;
constexpr double kRejoinYawTolerance = 0.22;
constexpr double kRouteEndpointTolerance = kRejoinPositionTolerance + 0.03;
constexpr double kSearchTerminalPositionTolerance = 0.18;
constexpr double kSearchTerminalYawTolerance = 0.20;
constexpr double kReverseLandingPositionTolerance = 0.09;
constexpr double kReverseLandingYawTolerance = 0.22;
constexpr double kStopSpeed = 0.005;
constexpr double kMaxBypassSpeed = 0.32;
constexpr double kBrakeReserve = 0.025;
constexpr std::size_t kProgressWindowPoints = 40;
constexpr double kLookaheadDistance = 0.30;
constexpr double kMinWheelbase = 0.01;
constexpr double kMinLateralDistanceSquared = 0.02;
constexpr double kMinBrakeDeceleration = 0.001;

double angle(double value)
{
  return std::remainder(value, 2.0 * kPi);
}

double distance(const MppiRetreatPose & a, const MppiRetreatPose & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double heuristic(const MppiRetreatPose & pose, const MppiRetreatPose & goal)
{
  return kHeuristicDistanceWeight * distance(pose, goal) +
         kHeuristicYawWeight * std::abs(angle(pose.yaw - goal.yaw));
}

struct SearchDeadline
{
  using Clock = std::chrono::steady_clock;

  Clock::time_point end;

  bool expired() const { return Clock::now() >= end; }
};

// Advances one bounded bicycle primitive and validates the swept body at every
// sample. A primitive is atomic: the search checks its soft deadline only
// between primitives, so a slow validator cannot force the same unfinished
// primitive to be rolled back and restarted on every update.
bool primitive(
  MppiRetreatPose & pose,
  double length,
  double curvature,
  const MppiDirectedRetreat::PoseValidator & validator,
  std::vector<MppiRetreatPose> * output = nullptr)
{
  const int count = std::max(
    1, static_cast<int>(std::ceil(
         std::abs(length) *
         (1.0 + kSearchBodySweepRadius * std::abs(curvature)) /
         kSearchSweepResolution)));
  for(int step = 0; step < count; ++step){
    pose = MppiDirectedRetreat::advance(
      pose, {length / count, curvature * length / count}, 1.0);
    if(!validator(pose)){
      return false;
    }
    if(output != nullptr){
      output->push_back(pose);
    }
  }
  return true;
}

struct Key
{
  int x;
  int y;
  int yaw;

  bool operator==(const Key & other) const
  {
    return x == other.x && y == other.y && yaw == other.yaw;
  }
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const
  {
    return (std::hash<int>()(key.x) * 73856093u) ^
           (std::hash<int>()(key.y) * 19349663u) ^
           (std::hash<int>()(key.yaw) * 83492791u);
  }
};

Key searchKey(const MppiRetreatPose & pose)
{
  return {
    static_cast<int>(std::lround(pose.x / kKeyPositionResolution)),
    static_cast<int>(std::lround(pose.y / kKeyPositionResolution)),
    static_cast<int>(std::lround(angle(pose.yaw) / kKeyYawResolution))};
}

bool validateRoutePoint(
  const MppiDirectedRetreat::PoseValidator & validator,
  std::size_t * route_validation_calls,
  const MppiRetreatPose & route_pose)
{
  ++*route_validation_calls;
  auto checked = route_pose;
  return validator(checked);
}

}  // namespace

MppiRetreatPose MppiDirectedRetreat::advance(
  const MppiRetreatPose & pose, const MppiControl & control, double dt)
{
  auto next = pose;
  const double turn = control.wz * dt;
  const double forward = control.vx * std::cos(pose.pitch);
  next.z -= control.vx * dt * std::sin(pose.pitch);
  if(std::abs(control.wz) > 1e-9){
    next.x += forward / control.wz *
              (std::sin(pose.yaw + turn) - std::sin(pose.yaw));
    next.y -= forward / control.wz *
              (std::cos(pose.yaw + turn) - std::cos(pose.yaw));
  }
  else{
    next.x += forward * dt * std::cos(pose.yaw);
    next.y += forward * dt * std::sin(pose.yaw);
  }
  next.yaw = angle(pose.yaw + turn);
  return next;
}

struct MppiDirectedRetreat::SearchState
{
  struct Node
  {
    MppiRetreatPose pose;
    double g = 0.0;
    int parent = -1;
    int root = 0;
    double curvature = 0.0;
    // Next outgoing edge to evaluate. Persisted so a soft deadline between
    // edges resumes the same expansion instead of restarting it.
    int next_edge = 0;
  };

  struct OpenItem
  {
    double f = 0.0;
    int index = 0;

    bool operator<(const OpenItem & other) const { return f > other.f; }
  };

  MppiRetreatPose start;
  MppiRetreatPose goal;
  double curvature = 0.0;
  std::vector<Node> nodes;
  std::vector<std::vector<MppiRetreatPose>> roots;
  std::priority_queue<OpenItem> open;
  std::unordered_map<Key, double, KeyHash> best;
  std::size_t expansions = 0;
  // Index of the node whose expansion was suspended at the last soft deadline,
  // or -1 when the next iteration should pop a fresh frontier node.
  int expanding = -1;
  int initial_arc = 0;
  int initial_step = 0;
  std::vector<MppiRetreatPose> reverse;
};

void MppiDirectedRetreat::reset()
{
  route_.clear();
  search_.reset();
  braking_ = false;
  reverse_end_ = progress_ = 0;
  direction_ = 0;
  phase_ = Phase::Reverse;
  reason_.clear();
}

bool MppiDirectedRetreat::search(
  const MppiRetreatPose & start,
  const std::vector<MppiPathPoint> & path,
  const MppiConfig & config,
  const PoseValidator & validator)
{
  const SearchDeadline deadline{
    std::chrono::steady_clock::now() + kSearchTimeSlice};
  if(search_ &&
     (distance(start, search_->start) > kSearchStartResetDistance ||
      std::abs(angle(start.yaw - search_->start.yaw)) > kSearchStartResetYaw))
  {
    // Evidence: a restart here discards every expansion already paid for and
    // restarts the scan from the first root, which is the only way a sliced
    // search can fail to converge while the vehicle is moving.
    ++search_restarts_;
    search_.reset();
  }
  if(!search_){
    reason_ = "no_downstream_reconnection";
    if(path.size() < 3 || !validator){
      return false;
    }

    std::size_t first_blocked = 0;
    while(first_blocked < path.size() && path[first_blocked].obstacle_free){
      ++first_blocked;
    }
    if(first_blocked == path.size()){
      return false;
    }
    std::size_t last_blocked = first_blocked;
    while(last_blocked + 1 < path.size() &&
          !path[last_blocked + 1].obstacle_free)
    {
      ++last_blocked;
    }

    double downstream = 0.0;
    std::size_t target = last_blocked;
    for(;
        target + 1 < path.size() && downstream < kDownstreamTargetDistance;
        ++target)
    {
      downstream += std::hypot(
        path[target + 1].x - path[target].x,
        path[target + 1].y - path[target].y);
    }
    if(downstream < kMinimumDownstreamDistance){
      return false;
    }

    double continuation = 0.0;
    for(std::size_t i = target;
        i + 1 < path.size() && continuation < kDownstreamContinuationDistance;
        ++i)
    {
      if(!path[i].obstacle_free || !path[i + 1].obstacle_free){
        return false;
      }
      continuation += std::hypot(
        path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    }
    if(continuation < kMinimumContinuationDistance){
      return false;
    }

    const MppiRetreatPose goal{
      path[target].x,
      path[target].y,
      path[target].yaw,
      start.z,
      start.roll,
      start.pitch};
    if(distance(start, goal) > kMaximumStartGoalDistance){
      return false;
    }

    auto projected_start = start;
    if(!validator(projected_start)){
      reason_ = "initial_pose_rejected";
      return false;
    }

    search_ = std::make_shared<SearchState>();
    search_->start = projected_start;
    search_->goal = goal;
    search_->curvature =
      std::tan(config.max_steer) / std::max(kMinWheelbase, config.wheelbase);
    if(!std::isfinite(search_->curvature) || search_->curvature <= 0.0){
      search_.reset();
      return false;
    }
    search_->nodes.reserve(kSearchNodeReserve);
  }

  auto & state = *search_;
  const double curvature = state.curvature;
  const double curvatures[] = {
    0.0, -curvature, curvature, -0.5 * curvature, 0.5 * curvature};

  // Resume root generation across stopped control cycles. Roots retain their
  // projected poses; the complete selected route is rechecked before use. The
  // deadline is checked between bounded atomic primitives, so a slow validator
  // delays one update but never discards a partially completed primitive.
  while(state.initial_arc < 5){
    if(deadline.expired()){
      reason_ = "search_in_progress";
      last_search_expansions_ = state.expansions;
      return false;
    }
    if(state.initial_step == 0){
      state.reverse = {state.start};
    }
    auto pose = state.reverse.back();
    const double k = curvatures[state.initial_arc];
    const std::size_t prior_points = state.reverse.size();
    if(!primitive(
         pose, -kReverseRootLength, k, validator, &state.reverse))
    {
      // A rejected primitive is not retried. Drop its valid prefix and move on
      // to the next bounded arc.
      state.reverse.resize(prior_points);
      ++state.initial_arc;
      state.initial_step = 0;
      continue;
    }

    ++state.initial_step;
    const int root = static_cast<int>(state.roots.size());
    state.roots.push_back(state.reverse);
    const double g =
      state.initial_step * kReverseRootLength * kRootCostFactor;
    state.nodes.push_back({pose, g, -1, root, k, 0});
    const int index = static_cast<int>(state.nodes.size()) - 1;
    state.best[searchKey(pose)] = g;
    state.open.push({g + heuristic(pose, state.goal), index});
    if(state.initial_step == kReverseRootSteps){
      ++state.initial_arc;
      state.initial_step = 0;
    }
  }

  int found = -1;
  while((!state.open.empty() || state.expanding >= 0) &&
        state.expansions < kSearchMaxExpansions)
  {
    int index = -1;
    if(state.expanding >= 0){
      // Continue the exact node/edge that the previous slice suspended. Do not
      // pop it again or replay already-evaluated outgoing primitives.
      index = state.expanding;
    }
    else{
      if(deadline.expired()){
        reason_ = "search_in_progress";
        last_search_expansions_ = state.expansions;
        return false;
      }
      index = state.open.top().index;
      state.open.pop();
      const auto node = state.nodes[static_cast<std::size_t>(index)];
      const auto known = state.best.find(searchKey(node.pose));
      if(known != state.best.end() && node.g > known->second + 1e-9){
        continue;
      }
      if(distance(node.pose, state.goal) < kSearchTerminalPositionTolerance &&
         std::abs(angle(node.pose.yaw - state.goal.yaw)) <
           kSearchTerminalYawTolerance)
      {
        found = index;
        break;
      }
    }

    // Copy scalar node fields before any push_back can reallocate the node
    // vector; only index-based updates are used across edge insertions.
    const MppiRetreatPose node_pose = state.nodes[static_cast<std::size_t>(index)].pose;
    const double node_g = state.nodes[static_cast<std::size_t>(index)].g;
    const int node_root = state.nodes[static_cast<std::size_t>(index)].root;
    for(int edge = state.nodes[static_cast<std::size_t>(index)].next_edge;
        edge < 5; ++edge)
    {
      if(deadline.expired()){
        state.nodes[static_cast<std::size_t>(index)].next_edge = edge;
        state.expanding = index;
        reason_ = "search_in_progress";
        last_search_expansions_ = state.expansions;
        return false;
      }
      const double k = curvatures[edge];
      auto pose = node_pose;
      if(distance(pose, state.start) > kSearchHalfExtent ||
         node_g > kSearchMaxCost)
      {
        state.nodes[static_cast<std::size_t>(index)].next_edge = edge + 1;
        continue;
      }
      if(!primitive(pose, kSearchSegmentLength, k, validator)){
        state.nodes[static_cast<std::size_t>(index)].next_edge = edge + 1;
        continue;
      }
      const double g = node_g + kSearchSegmentLength +
                       kSegmentCurvatureCost * std::abs(k) / curvature;
      const auto best = state.best.find(searchKey(pose));
      if(best != state.best.end() && best->second <= g){
        state.nodes[static_cast<std::size_t>(index)].next_edge = edge + 1;
        continue;
      }
      state.best[searchKey(pose)] = g;
      state.nodes.push_back({pose, g, index, node_root, k, 0});
      state.open.push(
        {g + heuristic(pose, state.goal),
         static_cast<int>(state.nodes.size()) - 1});
      state.nodes[static_cast<std::size_t>(index)].next_edge = edge + 1;
    }
    state.nodes[static_cast<std::size_t>(index)].next_edge = 5;
    state.expanding = -1;
    ++state.expansions;
  }

  if(found < 0){
    reason_ = state.expansions >= kSearchMaxExpansions ?
      "search_budget_exhausted" : "no_complete_bypass";
    last_search_expansions_ = state.expansions;
    search_.reset();
    return false;
  }

  route_ = state.roots[static_cast<std::size_t>(state.nodes[found].root)];
  reverse_end_ = route_.size() - 1;
  std::vector<int> chain;
  for(int i = found; state.nodes[static_cast<std::size_t>(i)].parent >= 0;
      i = state.nodes[static_cast<std::size_t>(i)].parent)
  {
    chain.push_back(i);
  }
  std::reverse(chain.begin(), chain.end());
  auto pose = route_.back();
  for(int i : chain){
    if(!primitive(
         pose, kSearchSegmentLength,
         state.nodes[static_cast<std::size_t>(i)].curvature, validator,
         &route_))
    {
      reset();
      reason_ = "changed_environment";
      return false;
    }
  }

  // Roots/nodes may have been validated in an earlier control cycle because
  // search is sliced. Recheck the complete selected route against this cycle's
  // immutable environment before retaining it; never trust a stale snapshot.
  for(const auto & route_pose : route_){
    if(!validateRoutePoint(validator, &route_validation_calls_, route_pose)){
      reset();
      reason_ = "changed_environment";
      return false;
    }
  }

  const double reverse_yaw = angle(route_[reverse_end_].yaw - start.yaw);
  direction_ = reverse_yaw > 1e-3 ? 1 : (reverse_yaw < -1e-3 ? -1 : 0);
  anchor_ = state.start;
  reconnect_goal_ = state.goal;
  last_search_expansions_ = state.expansions;
  ++routes_selected_;
  search_.reset();
  progress_ = 0;
  phase_ = Phase::Reverse;
  reason_ = "route_selected";
  return true;
}

MppiResult MppiDirectedRetreat::compute(
  const MppiRetreatPose & current,
  const MppiControl & measured,
  const std::vector<MppiPathPoint> & path,
  const MppiConfig & config,
  double speed_cap,
  const PoseValidator & validator,
  std::uint64_t environment_id)
{
  // Compatibility argument only. Snapshot identity cannot represent
  // query_stamp or per-voxel observation age, so it is never used to skip the
  // per-cycle remaining-route validation.
  (void)environment_id;
  const auto cycle_start = std::chrono::steady_clock::now();
  struct Timer
  {
    std::chrono::steady_clock::time_point start;
    double & milliseconds;

    ~Timer()
    {
      milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    }
  } timer{cycle_start, cycle_ms_};

  validation_calls_ = 0;
  route_validation_calls_ = 0;
  const PoseValidator valid = [&](MppiRetreatPose & pose) {
    ++validation_calls_;
    return validator && validator(pose);
  };

  MppiResult result;
  if(!config.reverse_recovery_enabled || !validator){
    return result;
  }

  bool route_selected_this_cycle = false;
  if(route_.empty()){
    if(braking_ && std::abs(measured.vx) < kStopSpeed){
      braking_ = false;
    }
    if(!braking_){
      route_selected_this_cycle =
        search(current, path, config, valid);
    }
    if(route_.empty()){
      if(!active()){
        result.reverse_candidates_generated = true;
        return result;
      }
      return brakingCommand(current, measured, config, valid);
    }
  }

  result.reverse_recovery_active = true;
  result.reverse_candidates_generated = true;

  // Revalidate the remaining route against this cycle's immutable environment
  // on every update. Snapshot identity is intentionally not consulted: even an
  // unchanged snapshot can have newly stale far-route support because
  // query_stamp and per-voxel observation age also govern validity. A freshly
  // selected route was already fully checked by search this cycle.
  if(!route_selected_this_cycle){
    for(std::size_t i = progress_; i < route_.size(); ++i){
      if(!validateRoutePoint(valid, &route_validation_calls_, route_[i])){
        reset();
        braking_ = true;
        reason_ = "retained_route_invalid";
        // Keep recovery authoritative: brake instead of returning no control,
        // which Local_Planner would surface as a route-blocked replan request.
        return brakingCommand(current, measured, config, valid);
      }
    }
  }

  const auto reverse_endpoint = route_[reverse_end_];
  const bool at_reverse_end =
    distance(current, reverse_endpoint) < kReverseLandingPositionTolerance;
  const bool reverse_yaw_aligned =
    std::abs(angle(current.yaw - reverse_endpoint.yaw)) <
    kReverseLandingYawTolerance;
  // Normally the reverse landing requires both position and heading. If the
  // tracker can only converge the position (heading is carried by the forward
  // bypass) and the braking distance collapses to the stopping reserve, the
  // reverse phase must still hand over; otherwise the command latches at zero
  // and the retreat stalls with a selected route.
  const bool reverse_stopped_at_end =
    distance(current, reverse_endpoint) <= kBrakeReserve;
  if(phase_ == Phase::Reverse && at_reverse_end &&
     (reverse_yaw_aligned || reverse_stopped_at_end))
  {
    phase_ = Phase::BrakeToForward;
  }
  if(phase_ == Phase::BrakeToForward &&
     std::abs(measured.vx) < kStopSpeed)
  {
    phase_ = Phase::FollowBypass;
    progress_ = reverse_end_;
  }
  if(phase_ == Phase::FollowBypass &&
     distance(current, reconnect_goal_) < kRejoinPositionTolerance &&
     std::abs(angle(current.yaw - reconnect_goal_.yaw)) <
       kRejoinYawTolerance &&
     distance(current, route_.back()) < kRouteEndpointTolerance)
  {
    reset();
    reason_ = "rejoined";
    return {};
  }

  reason_ = phase_ == Phase::Reverse ? "reverse" :
    (phase_ == Phase::BrakeToForward ? "brake_to_forward" : "follow_bypass");
  const bool reverse = phase_ == Phase::Reverse;
  result.reverse_motion_active = phase_ != Phase::FollowBypass;
  const double cap =
    speed_cap >= 0.0 ? speed_cap : std::numeric_limits<double>::infinity();
  const double target_speed = reverse ?
    -std::min(config.reverse_max_speed, cap) :
    std::min({kMaxBypassSpeed, config.vx_max, cap});

  auto pose = current;
  auto previous = measured;
  std::size_t cursor = progress_;
  const double curvature =
    std::tan(config.max_steer) /
    std::max(kMinWheelbase, config.wheelbase);
  const std::size_t end = reverse ? reverse_end_ : route_.size() - 1;
  if(config.shift_control_sequence){
    result.optimal_sequence.push_back(measured);
  }
  const auto reject_tracking = [&]() {
    // The retained geometric route may still validate while its constrained
    // tracking horizon does not. Drop the unexecutable route, latch braking,
    // and return a bounded braking sequence so recovery keeps ownership. A
    // no-control result here would be reported by Local_Planner as a blocked
    // route and escalate to a global replan in the middle of recovery.
    invalidate();
    reason_ = "tracking_horizon_rejected";
    return brakingCommand(current, measured, config, valid);
  };
  for(std::size_t step = result.optimal_sequence.size();
      step < config.time_steps; ++step)
  {
    const double dt = step == (config.shift_control_sequence ? 1u : 0u) ?
      config.controller_period : config.model_dt;

    // Advance progress only locally, preserving ordering where reverse and
    // forward route sections overlap.
    double nearest = distance(pose, route_[cursor]);
    for(std::size_t j = cursor + 1;
        j <= end && j < cursor + kProgressWindowPoints; ++j)
    {
      const double candidate = distance(pose, route_[j]);
      if(candidate < nearest){
        nearest = candidate;
        cursor = j;
      }
    }
    if(step == (config.shift_control_sequence ? 1u : 0u)){
      progress_ = cursor;
    }

    std::size_t look = cursor;
    double arc = 0.0;
    while(look < end && arc < kLookaheadDistance){
      arc += distance(route_[look], route_[look + 1]);
      ++look;
    }
    const auto target = route_[look];
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double lateral =
      -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
    double requested = target_speed;
    const double remaining = distance(pose, route_[end]);
    const double brake = reverse ?
      std::max(kMinBrakeDeceleration, config.ax_max) :
      std::max(kMinBrakeDeceleration, -config.ax_min);
    requested = std::copysign(
      std::min(
        std::abs(requested),
        std::sqrt(2.0 * brake * std::max(0.0, remaining - kBrakeReserve))),
      requested);
    if(phase_ == Phase::BrakeToForward){
      requested = 0.0;
    }
    // Every real sign change first reaches zero. A measured speed inside the
    // stop deadband is already "zero" for that purpose: comparing the raw
    // product would latch the first command at exactly zero when odometry
    // reports a tiny residual of the old direction, and because the vehicle
    // never moves, the same residual is measured on every following cycle.
    if((previous.vx > kStopSpeed && requested < 0.0) ||
       (previous.vx < -kStopSpeed && requested > 0.0))
    {
      requested = 0.0;
    }
    const double vx = std::clamp(
      requested, previous.vx + config.ax_min * dt,
      previous.vx + config.ax_max * dt);
    const double wanted_curvature = std::clamp(
      2.0 * lateral /
        std::max(kMinLateralDistanceSquared, dx * dx + dy * dy),
      -curvature, curvature);
    double wz = std::clamp(
      vx * wanted_curvature,
      previous.wz - config.az_max * dt,
      previous.wz + config.az_max * dt);
    wz = std::clamp(
      wz, -std::min(config.wz_max, std::abs(vx) * curvature),
      std::min(config.wz_max, std::abs(vx) * curvature));
    const MppiControl control{vx, wz};

    // Validate the actual constrained tracking controls, including their swept
    // body, exactly as they will be executed.
    if(std::abs(vx) > 1e-9){
      if(!primitive(pose, vx * dt, wz / vx, valid)){
        return reject_tracking();
      }
    }
    else{
      pose = advance(pose, control, dt);
      if(!valid(pose)){
        return reject_tracking();
      }
    }
    result.optimal_sequence.push_back(control);
    previous = control;
  }

  result.has_valid_control = !result.optimal_sequence.empty();
  const std::size_t command_index =
    config.shift_control_sequence && result.optimal_sequence.size() > 1 ?
    1u : 0u;
  if(result.has_valid_control){
    result.command = result.optimal_sequence[command_index];
  }
  result.best_sample_sequence = result.optimal_sequence;
  result.best_cost = 0.0;
  return result;
}

MppiResult MppiDirectedRetreat::brakingCommand(
  const MppiRetreatPose & current,
  const MppiControl & measured,
  const MppiConfig & config,
  const PoseValidator & validator)
{
  MppiResult result;
  result.reverse_recovery_active = active();
  result.reverse_candidates_generated = true;
  if(!active()){
    return result;
  }

  // While computing a route, or after the retained route became unexecutable,
  // brake toward rest. The sequence is validated exactly as it will execute so
  // the caller always receives a bounded command instead of a blocked-path
  // signal that would escalate to a global replan from inside recovery.
  auto pose = current;
  auto control = measured;
  if(config.shift_control_sequence){
    result.optimal_sequence.push_back(control);
  }
  const double curvature =
    std::tan(config.max_steer) /
    std::max(kMinWheelbase, config.wheelbase);
  // At rest the pose does not change, so one validation covers every trailing
  // stationary step; repeating it would cost a full sequence of identical
  // local-map queries on every search cycle.
  bool rest_validated = false;
  for(std::size_t i = result.optimal_sequence.size();
      i < config.time_steps; ++i)
  {
    const double dt = i == (config.shift_control_sequence ? 1u : 0u) ?
      config.controller_period : config.model_dt;
    control.vx = std::clamp(
      0.0, control.vx + config.ax_min * dt,
      control.vx + config.ax_max * dt);
    control.wz = std::clamp(
      0.0, control.wz - config.az_max * dt,
      control.wz + config.az_max * dt);
    control.wz = std::clamp(
      control.wz, -std::abs(control.vx) * curvature,
      std::abs(control.vx) * curvature);
    if(std::abs(control.vx) > 1e-9){
      rest_validated = false;
      if(!primitive(
           pose, control.vx * dt, control.wz / control.vx, validator))
      {
        // No validated braking command exists this cycle. There is nothing
        // safe to command; keep braking latched so the next cycle retries.
        result.optimal_sequence.clear();
        result.has_valid_control = false;
        return result;
      }
    }
    else if(!rest_validated && !validator(pose)){
      result.optimal_sequence.clear();
      result.has_valid_control = false;
      return result;
    }
    else{
      rest_validated = true;
    }
    result.optimal_sequence.push_back(control);
  }
  result.has_valid_control = !result.optimal_sequence.empty();
  if(result.has_valid_control){
    result.command = result.optimal_sequence[
      config.shift_control_sequence && result.optimal_sequence.size() > 1 ?
      1u : 0u];
  }
  result.reverse_motion_active = true;
  result.best_sample_sequence = result.optimal_sequence;
  result.best_cost = 0.0;
  return result;
}

}  // namespace local_planner
