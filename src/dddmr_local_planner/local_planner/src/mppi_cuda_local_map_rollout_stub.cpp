// Fail-closed no-CUDA implementation of MppiCudaLocalMapRollout. This file is
// selected when the package is built without a CUDA toolkit; it never falls
// back to the CPU rollout and never pretends a candidate succeeded.

#include "local_planner/mppi_cuda_local_map_rollout.h"

#include <string>
#include <utility>
#include <vector>

namespace local_planner {

namespace {

constexpr const char* kUnavailableReason =
    "CUDA local-map rollout backend is unavailable; no CPU fallback exists";

}  // namespace

struct MppiCudaLocalMapRollout::Impl {};

MppiCudaLocalMapRollout::MppiCudaLocalMapRollout()
    : impl_(std::make_unique<Impl>()) {}

MppiCudaLocalMapRollout::~MppiCudaLocalMapRollout() = default;
MppiCudaLocalMapRollout::MppiCudaLocalMapRollout(
    MppiCudaLocalMapRollout&&) noexcept = default;
MppiCudaLocalMapRollout& MppiCudaLocalMapRollout::operator=(
    MppiCudaLocalMapRollout&&) noexcept = default;

bool MppiCudaLocalMapRollout::available(std::string* reason) const {
  if (reason != nullptr) {
    *reason = kUnavailableReason;
  }
  return false;
}

bool MppiCudaLocalMapRollout::rolloutBatch(
    const MppiLocalMapContext& /*context*/,
    const MppiCudaTerrain& /*uploaded_terrain*/,
    const std::vector<MppiLocalMapRolloutInput>& /*inputs*/,
    std::vector<MppiLocalMapRolloutResult>& outputs, std::string& error) {
  outputs.clear();
  error = kUnavailableReason;
  return false;
}

std::unique_ptr<MppiCudaLocalMapRollout> makeMppiCudaLocalMapRollout() {
  return std::make_unique<MppiCudaLocalMapRollout>();
}

}  // namespace local_planner
