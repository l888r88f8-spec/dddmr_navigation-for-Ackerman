#ifndef LOCAL_PLANNER_MPPI_CUDA_BACKEND_H_
#define LOCAL_PLANNER_MPPI_CUDA_BACKEND_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "local_planner/mppi_controller.h"

namespace local_planner
{

// Device-independent description of one candidate-generation batch. The CPU
// reference below and the CUDA implementation consume exactly the same input,
// so fixed-input comparisons do not depend on the ROS graph.
struct MppiGenerationParams
{
  std::size_t batch_size = 1;
  std::size_t time_steps = 1;
  std::size_t structured_candidate_count = 0;
  double model_dt = 0.1;
  double controller_period = 0.1;
  double gamma = 0.0;
  double vx_std = 0.0;
  double wz_std = 0.0;
  double vx_min = 0.0;
  double vx_max = 0.0;
  double wz_max = 0.0;
  double max_steer = 0.0;
  double wheelbase = 1.0;
  double ax_min = 0.0;
  double ax_max = 0.0;
  double az_max = 0.0;
  double structured_min_vx = 0.0;
  bool shift_control_sequence = true;
  bool use_debug_noise = false;
  // CUDA normal path: generate the whole Gaussian batch on device from
  // (seed, noise_generation). Set only when no host noise batch is supplied.
  bool use_device_noise = false;
  std::uint32_t seed = 7;
  std::uint64_t noise_generation = 0;
  // Reverse (directed retreat) candidate generation. Reverse reuses the same
  // host/device constraint and noise math as forward sampling.
  bool reverse_mode = false;
  double reverse_speed_ceiling = 0.0;
  MppiBypassSide bypass_side = MppiBypassSide::Neutral;
  MppiAvoidanceStage avoidance_stage = MppiAvoidanceStage::Neutral;
};

struct MppiGenerationRequest
{
  MppiGenerationParams params;
  MppiControl measured_speed;
  // Warm-start sequence, row length params.time_steps.
  std::vector<MppiControl> control_sequence;
  // Noise samples, row-major with row length params.time_steps. The length may
  // be smaller than params.batch_size for seeded debug noise; indexing wraps
  // exactly like the controller's debug path.
  std::size_t noise_batch_size = 0;
  std::vector<MppiControl> noise;
};

struct MppiCandidateBatch
{
  std::size_t batch_size = 0;
  std::size_t time_steps = 0;
  // Row-major [sample * time_steps + step].
  std::vector<MppiControl> controls;
  std::vector<double> control_energy;
  std::vector<unsigned char> structured;
};

struct MppiWeightedUpdateRequest
{
  std::size_t batch_size = 0;
  std::size_t time_steps = 1;
  std::size_t command_index = 0;
  double temperature = 1.0;
  // Row-major candidate controls and costs (valid candidates only).
  std::vector<MppiControl> sequences;
  // Costs used for the selected weighted control update (may include Depart
  // soft penalties).
  std::vector<double> costs;
  // Raw valid costs used only for the softmax diagnostics (ESS and directional
  // weight masses). Empty means "same as costs".
  std::vector<double> diagnostic_costs;
  // Host-side directional mask and structured flags.
  std::vector<unsigned char> selected;
  std::vector<unsigned char> structured;
};

struct MppiWeightedUpdateResult
{
  std::vector<MppiControl> control_sequence;
  double effective_sample_size = 0.0;
  double positive_weight_mass = 0.0;
  double negative_weight_mass = 0.0;
  double neutral_weight_mass = 0.0;
  double structured_weight_mass = 0.0;
  std::size_t selected_count = 0;
};

// CPU reference with the exact semantics of MppiController::buildNoisedSequence,
// buildStructuredSequence and the selected-weight/control-update block. It is the
// differential oracle for the CUDA implementation and is always compiled.
bool mppiGenerateCandidatesCpu(
  const MppiGenerationRequest & request,
  MppiCandidateBatch & batch,
  std::string & error);

bool mppiWeightedUpdateCpu(
  const MppiWeightedUpdateRequest & request,
  MppiWeightedUpdateResult & result,
  std::string & error);

class MppiCudaBackend
{
public:
  virtual ~MppiCudaBackend() = default;
  virtual bool available(std::string * reason) const = 0;
  virtual bool generateCandidates(
    const MppiGenerationRequest & request,
    MppiCandidateBatch & batch,
    std::string & error) = 0;
  virtual bool weightedUpdate(
    const MppiWeightedUpdateRequest & request,
    MppiWeightedUpdateResult & result,
    std::string & error) = 0;
};

// Always returns a non-null backend. When the package was built without CUDA,
// or no usable device exists, available() returns false with an explicit reason;
// calls never fall back to the CPU implementation.
std::unique_ptr<MppiCudaBackend> makeMppiCudaBackend();

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CUDA_BACKEND_H_
