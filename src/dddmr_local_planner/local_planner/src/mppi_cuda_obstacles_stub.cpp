#include "local_planner/mppi_cuda_obstacles.h"

#include <memory>
#include <string>

namespace local_planner
{
namespace
{

constexpr const char * kNoCudaReason =
  "local_planner was built without CUDA: nvcc/CUDAToolkit was not found at "
  "configure time; neither the legacy_points nor the tilted_plane_2d obstacle "
  "rollout has a CPU fallback";

class MppiCudaObstaclesStub final : public MppiCudaObstacles
{
public:
  bool available(std::string * reason) const override
  {
    if(reason != nullptr){
      *reason = kNoCudaReason;
    }
    return false;
  }

  bool upload(
    const MppiObstacleSnapshot &,
    std::string & error) override
  {
    error = kNoCudaReason;
    return false;
  }

  bool rolloutBatch(
    const MppiCudaObstaclePose &,
    const pcl::PointCloud<pcl::PointXYZ> &,
    const std::vector<MppiCandidate> &,
    const MppiCudaObstacleConfig &,
    std::vector<MppiScoredTrajectory> &,
    std::string & error) override
  {
    error = kNoCudaReason;
    return false;
  }

  bool uploadPlanar(
    const MppiPlanarObstacleGrid &,
    std::string & error) override
  {
    upload_called_ = true;
    error = kNoCudaReason;
    return false;
  }

  bool rolloutPlanarBatch(
    const MppiPlanarBodyRect &,
    const std::vector<MppiCandidate> & candidates,
    const MppiCudaObstacleConfig &,
    std::vector<MppiScoredTrajectory> &,
    std::string & error,
    MppiCudaTrajectoryBatchView * device_batch) override
  {
    if(device_batch != nullptr){
      *device_batch = {};
    }
    // Match the device backend's "last non-empty batch" contract: a
    // metadata-only batch must not publish a rollout timestamp.
    if(!candidates.empty()){
      rollout_called_ = true;
    }
    error = kNoCudaReason;
    return false;
  }

  MppiCudaPlanarStats lastPlanarStats() const override
  {
    // Match the device backend's contract: -1 before the first call, 0 for a
    // call that completed no device work. This backend never uploads a grid.
    MppiCudaPlanarStats stats;
    if(upload_called_){
      stats.upload_ms = 0.0;
    }
    if(rollout_called_){
      stats.rollout_ms = 0.0;
    }
    return stats;
  }

private:
  mutable bool upload_called_ = false;
  mutable bool rollout_called_ = false;
};

}  // namespace

std::unique_ptr<MppiCudaObstacles> makeMppiCudaObstacles()
{
  return std::make_unique<MppiCudaObstaclesStub>();
}

}  // namespace local_planner
