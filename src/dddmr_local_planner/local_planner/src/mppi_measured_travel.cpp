#include "local_planner/mppi_measured_travel.h"

#include <cmath>

namespace local_planner
{
namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kTravelEpsilon = 1e-9;

bool finiteSample(const OdometryTravelSample & sample)
{
  return std::isfinite(sample.time_sec) &&
         std::isfinite(sample.x) &&
         std::isfinite(sample.y) &&
         std::isfinite(sample.yaw);
}

double shortestAngularDistance(const double from, const double to)
{
  // Result is in [-pi, pi] without depending on ROS angle helpers.
  double delta = std::fmod(to - from + M_PI, kTwoPi);
  if(delta < 0.0){
    delta += kTwoPi;
  }
  return delta - M_PI;
}
}  // namespace

MeasuredTravelIncrement MppiMeasuredTravelTracker::update(
  const OdometryTravelSample & sample)
{
  MeasuredTravelIncrement increment;
  if(!finiteSample(sample)){
    increment.status = TravelObservationStatus::kInvalid;
    return increment;
  }

  if(!have_last_){
    last_ = sample;
    have_last_ = true;
    increment.status = TravelObservationStatus::kBaseline;
    return increment;
  }

  if(sample.frame_id != last_.frame_id){
    // Poses in different frames are not comparable. Re-baseline from this
    // observation instead of manufacturing travel.
    last_ = sample;
    increment.status = TravelObservationStatus::kFrameChanged;
    return increment;
  }

  if(sample.time_sec == last_.time_sec){
    increment.status = TravelObservationStatus::kDuplicate;
    return increment;
  }
  if(sample.time_sec < last_.time_sec){
    increment.status = TravelObservationStatus::kRewound;
    return increment;
  }

  const double dx = sample.x - last_.x;
  const double dy = sample.y - last_.y;
  // Project the world-frame displacement onto the body direction using the
  // midpoint heading. This is stable for the small heading changes between
  // observations and also handles the +/-pi wrap.
  const double mid_yaw = last_.yaw +
    0.5 * shortestAngularDistance(last_.yaw, sample.yaw);
  const double signed_body_displacement =
    dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw);

  if(signed_body_displacement > kTravelEpsilon){
    increment.forward_distance = signed_body_displacement;
  }
  else if(signed_body_displacement < -kTravelEpsilon){
    increment.reverse_distance = -signed_body_displacement;
  }

  increment.status = TravelObservationStatus::kIntegrated;
  increment.integrated = true;
  last_ = sample;
  return increment;
}

void MppiMeasuredTravelTracker::reset()
{
  have_last_ = false;
  last_ = OdometryTravelSample{};
}

}  // namespace local_planner
