#include "local_planner/mppi_cuda_critics.h"

#include <memory>
#include <string>

namespace local_planner
{

struct MppiCudaCritics::Impl
{
  bool available = false;
  std::string reason = "built without CUDA";
};

MppiCudaCritics::MppiCudaCritics()
: impl_(std::make_unique<Impl>())
{
}

MppiCudaCritics::~MppiCudaCritics() = default;

MppiCudaCritics::MppiCudaCritics(MppiCudaCritics &&) noexcept = default;

MppiCudaCritics & MppiCudaCritics::operator=(MppiCudaCritics &&) noexcept = default;

bool MppiCudaCritics::available(std::string * reason) const
{
  if(!impl_) {
    if(reason != nullptr) {
      *reason = "CPU-only MPPI CUDA critic stub is not initialized";
    }
    return false;
  }
  if(reason != nullptr) {
    *reason = impl_->reason;
  }
  return impl_->available;
}

bool MppiCudaCritics::score(
  const MppiCriticConfig &,
  const MppiCriticData &,
  MppiCriticResult & result,
  std::string & error)
{
  result = MppiCriticResult{};
  error = "MPPI CUDA critics are unavailable: " +
    (impl_ ? impl_->reason : std::string("CPU-only stub is not initialized"));
  return false;
}

bool MppiCudaCritics::scoreDevice(
  const MppiCriticConfig & config,
  const MppiCriticData & data,
  const MppiCudaTrajectoryBatchView &,
  MppiCriticResult & result,
  std::string & error)
{
  return score(config, data, result, error);
}

std::unique_ptr<MppiCudaCritics> makeMppiCudaCritics()
{
  return std::make_unique<MppiCudaCritics>();
}

}  // namespace local_planner
