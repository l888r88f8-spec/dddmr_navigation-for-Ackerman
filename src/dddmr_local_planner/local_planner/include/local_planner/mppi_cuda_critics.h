#ifndef LOCAL_PLANNER_MPPI_CUDA_CRITICS_H_
#define LOCAL_PLANNER_MPPI_CUDA_CRITICS_H_

#include <memory>
#include <string>

#include "local_planner/mppi_critic_manager.h"
#include "local_planner/mppi_cuda_trajectory_batch.h"

namespace local_planner
{

// GPU implementation of the complete MPPI critic pass.
//
// The class deliberately has the same input/output types as MppiCriticManager:
// callers provide MppiCriticConfig/MppiCriticData and receive the full
// MppiCriticResult (scores, term_costs, fail_flag and path diagnostics).  All
// critic arithmetic runs in CUDA kernels; host code only flattens the input and
// copies the result back.
//
// Device buffers are retained by the implementation and grown on demand, so a
// caller can reuse one instance for every control cycle.
//
// There is no CPU fallback.  If the library was built without CUDA, or no
// usable device/driver exists, available() returns false with a reason and
// score() fails with an explicit error.  When available() is true but a CUDA
// operation fails, score() also returns false and leaves an empty result.
class MppiCudaCritics
{
public:
  MppiCudaCritics();
  ~MppiCudaCritics();

  MppiCudaCritics(const MppiCudaCritics &) = delete;
  MppiCudaCritics & operator=(const MppiCudaCritics &) = delete;
  MppiCudaCritics(MppiCudaCritics &&) noexcept;
  MppiCudaCritics & operator=(MppiCudaCritics &&) noexcept;

  bool available(std::string * reason = nullptr) const;

  // Scores every candidate exactly like MppiCriticManager::score, but in CUDA.
  // `error` is cleared on success.  On failure the result is reset to its
  // default value (no partially-scored data is exposed).
  bool score(
    const MppiCriticConfig & config,
    const MppiCriticData & data,
    MppiCriticResult & result,
    std::string & error);

  // Score the matching host batch's device-resident rollout without uploading
  // its points, probes or controls again.
  bool scoreDevice(
    const MppiCriticConfig & config,
    const MppiCriticData & data,
    const MppiCudaTrajectoryBatchView & device_batch,
    MppiCriticResult & result,
    std::string & error);

private:
  bool scoreImpl(
    const MppiCriticConfig & config,
    const MppiCriticData & data,
    const MppiCudaTrajectoryBatchView * device_batch,
    MppiCriticResult & result,
    std::string & error);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Always returns a non-null object.  The CUDA translation unit and the CPU-only
// stub provide alternative definitions of this factory; exactly one of them
// must be linked into a target.
std::unique_ptr<MppiCudaCritics> makeMppiCudaCritics();

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CUDA_CRITICS_H_
