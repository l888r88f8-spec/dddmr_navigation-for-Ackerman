#include "local_planner/mppi_cuda_backend.h"

#include <memory>
#include <string>

namespace local_planner
{
namespace
{

constexpr const char * kNoCudaReason =
  "local_planner was built without CUDA: nvcc/CUDAToolkit was not found at "
  "configure time (CPU fallback is explicitly disabled for MppiComputeBackend::Cuda)";

class MppiCudaBackendStub final : public MppiCudaBackend
{
public:
  bool available(std::string * reason) const override
  {
    if(reason != nullptr){
      *reason = kNoCudaReason;
    }
    return false;
  }

  bool generateCandidates(
    const MppiGenerationRequest &,
    MppiCandidateBatch &,
    std::string & error) override
  {
    error = kNoCudaReason;
    return false;
  }

  bool weightedUpdate(
    const MppiWeightedUpdateRequest &,
    MppiWeightedUpdateResult &,
    std::string & error) override
  {
    error = kNoCudaReason;
    return false;
  }
};

}  // namespace

std::unique_ptr<MppiCudaBackend> makeMppiCudaBackend()
{
  return std::make_unique<MppiCudaBackendStub>();
}

}  // namespace local_planner
