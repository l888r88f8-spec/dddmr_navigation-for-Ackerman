#ifndef LOCAL_PLANNER_MPPI_CUDA_TRAJECTORY_BATCH_H_
#define LOCAL_PLANNER_MPPI_CUDA_TRAJECTORY_BATCH_H_

#include <cstddef>

namespace local_planner
{

struct MppiControl;

struct MppiCudaTrajectoryPoint
{
  double x;
  double y;
  double yaw;
  double obstacle_clearance;
  unsigned char in_collision;
  unsigned char collision_evaluated;
  unsigned long long collision_node_checks;
  unsigned long long collision_point_checks;
  unsigned long long clearance_node_checks;
  unsigned long long clearance_point_checks;
};

// Non-owning device pointers for the matching host rollout batch. Points and
// controls are contiguous in candidate order. Probe storage has a fixed-size
// slice for each nonempty candidate when probing is enabled. The obstacle
// backend owns the storage until its next rollout or destruction.
struct MppiCudaTrajectoryBatchView
{
  const MppiCudaTrajectoryPoint * points = nullptr;
  const MppiCudaTrajectoryPoint * probes = nullptr;
  const MppiControl * controls = nullptr;
  std::size_t candidate_count = 0;
  std::size_t point_count = 0;
  std::size_t control_count = 0;
  std::size_t probe_capacity = 0;
  std::size_t probe_stride = 0;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CUDA_TRAJECTORY_BATCH_H_
