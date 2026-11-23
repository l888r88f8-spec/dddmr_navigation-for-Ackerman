#pragma once

// GPU reproduction of rolloutMppiLocalMap() for a whole candidate batch.
//
// The caller uploads one MppiCudaTerrain snapshot per map cycle. rolloutBatch()
// then executes one CUDA thread per candidate. Each thread advances its
// candidate through the requested time steps in order and calls the in-kernel
// cuda_terrain::evaluate() directly; no CPU per-step loop and no CPU terrain
// evaluation is involved.
//
// Business rejections (stale/outside/support/body/control/config/transform)
// are written into the corresponding output element. The boolean return is
// reserved for CUDA execution failure or a device terrain snapshot that does
// not match the pinned MppiLocalMapContext.

#include <memory>
#include <string>
#include <vector>

#include "local_planner/mppi_cuda_terrain.h"
#include "local_planner/mppi_local_map_rollout.h"

namespace local_planner {

class MppiCudaLocalMapRollout {
 public:
  MppiCudaLocalMapRollout();
  ~MppiCudaLocalMapRollout();
  MppiCudaLocalMapRollout(const MppiCudaLocalMapRollout&) = delete;
  MppiCudaLocalMapRollout& operator=(const MppiCudaLocalMapRollout&) = delete;
  MppiCudaLocalMapRollout(MppiCudaLocalMapRollout&&) noexcept;
  MppiCudaLocalMapRollout& operator=(MppiCudaLocalMapRollout&&) noexcept;

  // True only for a real CUDA implementation with a usable device. When false,
  // `reason` (if provided) receives the explicit unavailable reason. The
  // always-built stub reports false and performs no CPU fallback.
  bool available(std::string* reason = nullptr) const;

  // Evaluates every input candidate against the already-uploaded terrain
  // snapshot. `outputs` is resized to inputs.size(); each element is a complete
  // MppiLocalMapRolloutResult (projected_start_pose, projected_poses,
  // trajectory controls/points, evaluated_steps, map metadata, status/detail).
  //
  // Empty `inputs` succeeds with an empty `outputs`. Returns false only for
  // CUDA execution failure or when `uploaded_terrain` does not carry the same
  // map version/stamp as `context`; `error` then explains the failure.
  bool rolloutBatch(const MppiLocalMapContext& context,
                    const MppiCudaTerrain& uploaded_terrain,
                    const std::vector<MppiLocalMapRolloutInput>& inputs,
                    std::vector<MppiLocalMapRolloutResult>& outputs,
                    std::string& error);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<MppiCudaLocalMapRollout> makeMppiCudaLocalMapRollout();

}  // namespace local_planner
