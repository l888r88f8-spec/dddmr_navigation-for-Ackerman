#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "local_planner/mppi_critic_manager.h"

namespace local_planner
{

// Full 3-D pose used by the local retreat search. Only x/y/yaw drive the
// planar search; z/roll/pitch retain the backend's terrain projection.
struct MppiRetreatPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
};

// Plans and follows one reverse retreat followed by an accepted forward bypass.
//
// The search is deliberately independent of ROS, the runtime map owner and the
// live clock. The caller supplies an immutable-cycle pose validator, so every
// candidate and every retained-route point is checked against the same
// environment the backend will use for the final rollout.
class MppiDirectedRetreat
{
public:
  using PoseValidator = std::function<bool(MppiRetreatPose &)>;

  // environment_id is retained as an ABI-compatible caller argument and is
  // deliberately ignored. A snapshot version alone cannot certify per-voxel
  // observation freshness (query_stamp/age also matter), so the remaining route
  // is revalidated every update regardless of the supplied identity.
  MppiResult compute(
    const MppiRetreatPose & current,
    const MppiControl & measured,
    const std::vector<MppiPathPoint> & path,
    const MppiConfig & config,
    double speed_cap,
    const PoseValidator & validator,
    std::uint64_t environment_id = 0);

  static MppiRetreatPose advance(
    const MppiRetreatPose & pose, const MppiControl & control, double dt);

  bool active() const { return !route_.empty() || search_ || braking_; }
  const std::vector<MppiRetreatPose> & route() const { return route_; }
  std::size_t reverseEndIndex() const { return reverse_end_; }
  int direction() const { return direction_; }
  void reset();
  void invalidate()
  {
    reset();
    braking_ = true;
  }

  // Diagnostics: total validator calls in the last update and validator calls
  // spent specifically on retained-route/selection revalidation.
  std::size_t lastValidationCalls() const { return validation_calls_; }
  std::size_t lastRouteValidationCalls() const { return route_validation_calls_; }
  double lastCycleMs() const { return cycle_ms_; }
  const std::string & reason() const { return reason_; }
  // Evidence for a stalling retreat: how many times the sliced search was
  // discarded because the vehicle had moved past its restart tolerance, how
  // many A* expansions the last search reached, and how many complete searches
  // have produced a route. Counters are cumulative for the lifetime of this
  // object; reset() deliberately keeps them so a stop/retry cycle stays visible.
  std::size_t searchRestartCount() const { return search_restarts_; }
  std::size_t lastSearchExpansions() const { return last_search_expansions_; }
  std::size_t routesSelectedCount() const { return routes_selected_; }
  void resetSearchCounters()
  {
    search_restarts_ = 0;
    last_search_expansions_ = 0;
    routes_selected_ = 0;
  }

private:
  enum class Phase
  {
    Reverse,
    BrakeToForward,
    FollowBypass
  };

  struct SearchState;

  std::vector<MppiRetreatPose> route_;
  std::size_t reverse_end_ = 0;
  std::size_t progress_ = 0;
  int direction_ = 0;
  Phase phase_ = Phase::Reverse;
  MppiRetreatPose anchor_;
  MppiRetreatPose reconnect_goal_;
  std::string reason_;
  std::shared_ptr<SearchState> search_;
  bool braking_ = false;
  std::size_t validation_calls_ = 0;
  std::size_t route_validation_calls_ = 0;
  double cycle_ms_ = 0.0;
  std::size_t search_restarts_ = 0;
  std::size_t last_search_expansions_ = 0;
  std::size_t routes_selected_ = 0;

  bool search(
    const MppiRetreatPose & start,
    const std::vector<MppiPathPoint> & path,
    const MppiConfig & config,
    const PoseValidator & validator);
  // Builds the bounded deceleration-to-zero sequence that owns the cycle while
  // a route is being searched, invalidated, or rejected. Active recovery must
  // never return "no control": that is a braking state, not a blocked route.
  MppiResult brakingCommand(
    const MppiRetreatPose & current,
    const MppiControl & measured,
    const MppiConfig & config,
    const PoseValidator & validator);
};

}  // namespace local_planner
