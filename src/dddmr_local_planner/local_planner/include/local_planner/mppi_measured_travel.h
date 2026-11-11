#ifndef LOCAL_PLANNER__MPPI_MEASURED_TRAVEL_H_
#define LOCAL_PLANNER__MPPI_MEASURED_TRAVEL_H_

#include <string>

namespace local_planner
{

struct OdometryTravelSample
{
  double time_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id;
};

enum class TravelObservationStatus
{
  kBaseline,
  kIntegrated,
  kInvalid,
  kDuplicate,
  kRewound,
  kFrameChanged,
};

struct MeasuredTravelIncrement
{
  TravelObservationStatus status{TravelObservationStatus::kBaseline};
  // True only when this sample was compared with a previous, compatible
  // observation and therefore produced measured travel.
  bool integrated{false};
  double forward_distance{0.0};
  double reverse_distance{0.0};
};

// Deterministic, ROS-free accumulator that turns consecutive odometry poses
// into signed body-frame travel. It deliberately has no dt cap: a long gap is
// integrated as the real displacement between the two observations instead of
// inventing travel from a nominal control period.
class MppiMeasuredTravelTracker
{
public:
  MeasuredTravelIncrement update(const OdometryTravelSample & sample);
  void reset();

  bool initialized() const {return have_last_;}
  const OdometryTravelSample & lastSample() const {return last_;}

private:
  bool have_last_{false};
  OdometryTravelSample last_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__MPPI_MEASURED_TRAVEL_H_
