#include "local_planner/mppi_cuda_backend.h"

#include <cuda_runtime.h>
#include <math_constants.h>

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "local_planner/mppi_batch_math.h"
#include "local_planner/mppi_cuda_launch.h"

namespace local_planner
{
namespace
{

constexpr int kThreads = 256;
constexpr double kAngularDeadband = mppi_math::kAngularDeadband;

bool checkCuda(const cudaError_t status, const char * operation, std::string & error)
{
  if(status == cudaSuccess){
    return true;
  }
  std::ostringstream message;
  message << "CUDA " << operation << " failed: " << cudaGetErrorString(status) <<
    " (" << static_cast<int>(status) << ")";
  error = message.str();
  return false;
}

bool checkKernel(const char * name, std::string & error)
{
  if(!checkCuda(cudaGetLastError(), name, error)){
    return false;
  }
  return checkCuda(cudaDeviceSynchronize(), name, error);
}

template<typename T>
bool ensureCapacity(T * & pointer, std::size_t & capacity, const std::size_t needed, std::string & error)
{
  if(needed <= capacity){
    return true;
  }
  if(pointer != nullptr){
    if(!checkCuda(cudaFree(pointer), "cudaFree", error)){
      pointer = nullptr;
      capacity = 0;
      return false;
    }
    pointer = nullptr;
    capacity = 0;
  }
  void * allocated = nullptr;
  if(!checkCuda(cudaMalloc(&allocated, needed * sizeof(T)), "cudaMalloc", error)){
    return false;
  }
  pointer = static_cast<T *>(allocated);
  capacity = needed;
  return true;
}

__global__ void generateCandidatesKernel(
  const MppiGenerationParams params,
  const MppiControl measured_speed,
  const MppiControl * warm,
  const MppiControl * noise,
  const std::size_t noise_count,
  const std::size_t noise_batch_size,
  const std::size_t structured_count,
  MppiControl * controls,
  double * energies,
  unsigned char * structured)
{
  const std::size_t sample =
    static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(sample >= params.batch_size){
    return;
  }
  MppiControl * row = controls + sample * params.time_steps;
  if(params.reverse_mode){
    const bool is_structured = sample < structured_count &&
      params.reverse_speed_ceiling > 1e-6;
    structured[sample] = is_structured ? 1u : 0u;
    double energy = 0.0;
    mppi_math::generateReverseSample(
      params, measured_speed, noise, noise_count, noise_batch_size,
      sample, is_structured, row, energy);
    energies[sample] = is_structured ? 0.0 : energy;
  }
  else if(sample < structured_count){
    structured[sample] = 1u;
    mppi_math::generateStructuredSample(
      params, measured_speed, warm, params.time_steps, sample, row);
    energies[sample] = 0.0;
  }
  else{
    structured[sample] = 0u;
    double energy = 0.0;
    mppi_math::generateNoisedSample(
      params,
      measured_speed,
      warm,
      params.time_steps,
      noise,
      noise_count,
      noise_batch_size,
      sample,
      row,
      energy);
    energies[sample] = energy;
  }
}

__device__ double blockSum(double * shared, const std::size_t thread_index, double value)
{
  shared[thread_index] = value;
  __syncthreads();
  for(int stride = blockDim.x / 2; stride > 0; stride >>= 1){
    if(thread_index < static_cast<std::size_t>(stride)){
      shared[thread_index] += shared[thread_index + stride];
    }
    __syncthreads();
  }
  return shared[0];
}

// Stable softmax statistics in one parallel pass over all candidates. The
// global minimum is reduced first (below), so exp() arguments are <= 0 and
// cannot overflow for finite costs.
__global__ void weightedStatisticsKernel(
  const double * diagnostic_costs,
  const double * costs,
  const MppiControl * sequences,
  const unsigned char * selected,
  const unsigned char * structured,
  const std::size_t count,
  const std::size_t time_steps,
  const std::size_t command_index,
  const double temperature,
  const double min_cost,
  const double selected_min_cost,
  double * all_sum,
  double * all_squared_sum,
  double * positive_mass,
  double * negative_mass,
  double * neutral_mass,
  double * selected_sum,
  double * structured_mass,
  std::size_t * selected_count)
{
  __shared__ double sums[7][kThreads];
  __shared__ double count_shared[kThreads];
  const std::size_t thread_index = threadIdx.x;
  double local_all = 0.0;
  double local_all_sq = 0.0;
  double local_positive = 0.0;
  double local_negative = 0.0;
  double local_neutral = 0.0;
  double local_selected = 0.0;
  double local_structured = 0.0;
  double local_count = 0.0;
  for(std::size_t sample = thread_index; sample < count; sample += blockDim.x){
    const double weight = exp(-(diagnostic_costs[sample] - min_cost) / temperature);
    const double normalized = weight;  // normalized by all_sum after reduction
    local_all += normalized;
    local_all_sq += normalized * normalized;
    const double command_wz =
      sequences[sample * time_steps + command_index].wz;
    if(command_wz > kAngularDeadband){
      local_positive += normalized;
    }
    else if(command_wz < -kAngularDeadband){
      local_negative += normalized;
    }
    else{
      local_neutral += normalized;
    }
    if(selected[sample]){
      ++local_count;
      const double selected_weight =
        exp(-(costs[sample] - selected_min_cost) / temperature);
      local_selected += selected_weight;
      if(structured[sample]){
        local_structured += selected_weight;
      }
    }
  }

  const double reduced_all = blockSum(sums[0], thread_index, local_all);
  const double reduced_all_sq = blockSum(sums[1], thread_index, local_all_sq);
  const double reduced_positive = blockSum(sums[2], thread_index, local_positive);
  const double reduced_negative = blockSum(sums[3], thread_index, local_negative);
  const double reduced_neutral = blockSum(sums[4], thread_index, local_neutral);
  const double reduced_selected = blockSum(sums[5], thread_index, local_selected);
  const double reduced_structured = blockSum(sums[6], thread_index, local_structured);
  count_shared[thread_index] = local_count;
  __syncthreads();
  for(int stride = blockDim.x / 2; stride > 0; stride >>= 1){
    if(thread_index < static_cast<std::size_t>(stride)){
      count_shared[thread_index] += count_shared[thread_index + stride];
    }
    __syncthreads();
  }
  if(thread_index == 0){
    *all_sum = reduced_all;
    *all_squared_sum = reduced_all_sq;
    *positive_mass = reduced_positive;
    *negative_mass = reduced_negative;
    *neutral_mass = reduced_neutral;
    *selected_sum = reduced_selected;
    *structured_mass = reduced_structured;
    *selected_count = static_cast<std::size_t>(count_shared[0]);
  }
}

__global__ void minCostKernel(
  const double * diagnostic_costs,
  const double * costs,
  const unsigned char * selected,
  const std::size_t count,
  double * all_min,
  double * selected_min)
{
  __shared__ double all_shared[kThreads];
  __shared__ double selected_shared[kThreads];
  const std::size_t thread_index = threadIdx.x;
  double local_all = CUDART_INF;
  double local_selected = CUDART_INF;
  for(std::size_t sample = thread_index; sample < count; sample += blockDim.x){
    local_all = mppi_math::minValue(local_all, diagnostic_costs[sample]);
    if(selected[sample]){
      local_selected = mppi_math::minValue(local_selected, costs[sample]);
    }
  }
  all_shared[thread_index] = local_all;
  selected_shared[thread_index] = local_selected;
  __syncthreads();
  for(int stride = blockDim.x / 2; stride > 0; stride >>= 1){
    if(thread_index < static_cast<std::size_t>(stride)){
      all_shared[thread_index] = mppi_math::minValue(
        all_shared[thread_index], all_shared[thread_index + stride]);
      selected_shared[thread_index] = mppi_math::minValue(
        selected_shared[thread_index], selected_shared[thread_index + stride]);
    }
    __syncthreads();
  }
  if(thread_index == 0){
    *all_min = all_shared[0];
    *selected_min = selected_shared[0];
  }
}

// One thread per control step performs the selected weighted average in the
// controller's sample order. All arithmetic is double precision.
__global__ void weightedControlKernel(
  const MppiControl * sequences,
  const double * costs,
  const unsigned char * selected,
  const std::size_t count,
  const std::size_t time_steps,
  const double temperature,
  const double selected_min_cost,
  const double selected_sum,
  MppiControl * updated)
{
  const std::size_t step = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if(step >= time_steps){
    return;
  }
  double vx = 0.0;
  double wz = 0.0;
  for(std::size_t sample = 0; sample < count; ++sample){
    if(!selected[sample]){
      continue;
    }
    const double normalized = exp(
      -(costs[sample] - selected_min_cost) / temperature) / selected_sum;
    vx += normalized * sequences[sample * time_steps + step].vx;
    wz += normalized * sequences[sample * time_steps + step].wz;
  }
  updated[step] = MppiControl{vx, wz};
}

class MppiCudaBackendImpl final : public MppiCudaBackend
{
public:
  MppiCudaBackendImpl()
  {
    int device_count = 0;
    if(cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0){
      cudaGetLastError();
      return;
    }
    cudaDeviceProp properties{};
    if(cudaGetDeviceProperties(&properties, 0) != cudaSuccess){
      cudaGetLastError();
      return;
    }
    sm_count_ = std::max(1, properties.multiProcessorCount);
  }
  ~MppiCudaBackendImpl() override
  {
    release();
  }

  bool available(std::string * reason) const override
  {
    int device_count = 0;
    const cudaError_t status = cudaGetDeviceCount(&device_count);
    if(status != cudaSuccess || device_count <= 0){
      if(reason != nullptr){
        *reason = status != cudaSuccess ?
          std::string("cudaGetDeviceCount failed: ") + cudaGetErrorString(status) :
          "no CUDA device is present";
      }
      return false;
    }
    cudaDeviceProp properties{};
    const cudaError_t property_status = cudaGetDeviceProperties(&properties, 0);
    if(property_status != cudaSuccess){
      if(reason != nullptr){
        *reason = std::string("cudaGetDeviceProperties failed: ") +
          cudaGetErrorString(property_status);
      }
      return false;
    }
    if(reason != nullptr){
      std::ostringstream message;
      message << properties.name << " (compute capability " <<
        properties.major << "." << properties.minor << ", " <<
        sm_count_ << " SMs)";
      *reason = message.str();
    }
    return true;
  }

  bool generateCandidates(
    const MppiGenerationRequest & request,
    MppiCandidateBatch & batch,
    std::string & error) override
  {
    batch = MppiCandidateBatch{};
    MppiGenerationParams params = request.params;
    if(request.noise.empty() && !params.use_debug_noise){
      params.use_device_noise = true;
    }
    if(params.batch_size == 0 || params.time_steps == 0){
      error = "candidate generation requires non-zero batch_size and time_steps";
      return false;
    }
    if(request.control_sequence.size() < params.time_steps){
      error = "warm-start control sequence is shorter than time_steps";
      return false;
    }
    if(!(params.model_dt > 0.0) || !(params.controller_period > 0.0) ||
      !(params.wheelbase > 0.0))
    {
      error = "candidate generation received non-positive dt or wheelbase";
      return false;
    }
    if(!available(&error)){
      error = "CUDA backend unavailable: " + error;
      return false;
    }

    std::size_t total = 0;
    if(!mppi_math::checkedMulSize(
        params.batch_size, params.time_steps, total))
    {
      error = "candidate generation size overflow";
      return false;
    }
    if(!ensureCapacity(workspace_.controls, workspace_.controls_capacity, total, error) ||
      !ensureCapacity(workspace_.energies, workspace_.energy_capacity,
        params.batch_size, error) ||
      !ensureCapacity(workspace_.structured, workspace_.structured_capacity,
        params.batch_size, error) ||
      !ensureCapacity(workspace_.warm, workspace_.warm_capacity,
        params.time_steps, error) ||
      !ensureCapacity(workspace_.noise, workspace_.noise_capacity,
        request.noise.size(), error))
    {
      return false;
    }

    std::size_t warm_bytes = 0;
    if(!mppi_math::checkedBytes(
        params.time_steps, sizeof(MppiControl), warm_bytes))
    {
      error = "warm-start sequence byte size overflow";
      return false;
    }
    if(!checkCuda(cudaMemcpy(workspace_.warm, request.control_sequence.data(),
        warm_bytes, cudaMemcpyHostToDevice),
        "copy warm-start sequence", error))
    {
      return false;
    }
    std::size_t noise_bytes = 0;
    if(!mppi_math::checkedBytes(
        request.noise.size(), sizeof(MppiControl), noise_bytes))
    {
      error = "noise batch byte size overflow";
      return false;
    }
    if(!request.noise.empty() &&
      !checkCuda(cudaMemcpy(workspace_.noise, request.noise.data(),
        noise_bytes, cudaMemcpyHostToDevice),
        "copy noise batch", error))
    {
      return false;
    }

    const std::size_t structured_count =
      params.use_debug_noise ? 0 :
      std::min(params.structured_candidate_count, params.batch_size);
    const MppiCudaLaunchGeometry generate_launch =
      mppiCudaSelectLaunch(params.batch_size, sm_count_);
    generateCandidatesKernel<<<generate_launch.grid,
      generate_launch.block_dim>>>(
      params,
      request.measured_speed,
      workspace_.warm,
      request.noise.empty() ? nullptr : workspace_.noise,
      request.noise.size(),
      request.noise_batch_size,
      structured_count,
      workspace_.controls,
      workspace_.energies,
      workspace_.structured);
    if(!checkKernel("generateCandidatesKernel", error)){
      return false;
    }

    std::size_t control_bytes = 0;
    std::size_t energy_bytes = 0;
    if(!mppi_math::checkedBytes(
        total, sizeof(MppiControl), control_bytes) ||
      !mppi_math::checkedBytes(
        params.batch_size, sizeof(double), energy_bytes))
    {
      error = "candidate output byte size overflow";
      return false;
    }
    MppiCandidateBatch generated;
    generated.batch_size = params.batch_size;
    generated.time_steps = params.time_steps;
    generated.controls.resize(total);
    generated.control_energy.resize(params.batch_size);
    generated.structured.resize(params.batch_size);
    if(!checkCuda(cudaMemcpy(generated.controls.data(), workspace_.controls,
        control_bytes, cudaMemcpyDeviceToHost),
        "copy candidate controls", error) ||
      !checkCuda(cudaMemcpy(generated.control_energy.data(), workspace_.energies,
        energy_bytes, cudaMemcpyDeviceToHost),
        "copy candidate energy", error) ||
      !checkCuda(cudaMemcpy(generated.structured.data(), workspace_.structured,
        params.batch_size * sizeof(unsigned char), cudaMemcpyDeviceToHost),
        "copy structured flags", error))
    {
      return false;
    }
    batch = std::move(generated);
    return true;
  }

  bool weightedUpdate(
    const MppiWeightedUpdateRequest & request,
    MppiWeightedUpdateResult & result,
    std::string & error) override
  {
    result = MppiWeightedUpdateResult{};
    const std::size_t count = request.batch_size;
    if(count == 0 || request.time_steps == 0){
      error = "weighted update requires non-zero batch_size and time_steps";
      return false;
    }
    std::size_t total = 0;
    if(!mppi_math::checkedMulSize(count, request.time_steps, total)){
      error = "weighted update size overflow";
      return false;
    }
    if(request.sequences.size() != total ||
      request.costs.size() != count ||
      (!request.diagnostic_costs.empty() &&
      request.diagnostic_costs.size() != count) ||
      request.selected.size() != count ||
      request.structured.size() != count)
    {
      error = "weighted update input sizes do not match batch_size/time_steps";
      return false;
    }
    if(!(request.temperature > 0.0)){
      error = "weighted update requires a positive temperature";
      return false;
    }
    if(!available(&error)){
      error = "CUDA backend unavailable: " + error;
      return false;
    }

    if(!ensureCapacity(workspace_.sequences, workspace_.sequence_capacity, total, error) ||
      !ensureCapacity(workspace_.costs, workspace_.cost_capacity, count, error) ||
      !ensureCapacity(workspace_.diagnostic_costs, workspace_.diagnostic_cost_capacity,
        count, error) ||
      !ensureCapacity(workspace_.selected, workspace_.selected_capacity, count, error) ||
      !ensureCapacity(workspace_.structured_in, workspace_.structured_in_capacity,
        count, error) ||
      !ensureCapacity(workspace_.updated, workspace_.updated_capacity,
        request.time_steps, error) ||
      !ensureCapacity(workspace_.scalar, workspace_.scalar_capacity, 10, error) ||
      !ensureCapacity(workspace_.count_scalar, workspace_.count_capacity, 1, error))
    {
      return false;
    }

    std::size_t sequence_bytes = 0;
    std::size_t cost_bytes = 0;
    if(!mppi_math::checkedBytes(total, sizeof(MppiControl), sequence_bytes) ||
      !mppi_math::checkedBytes(count, sizeof(double), cost_bytes))
    {
      error = "weighted update byte size overflow";
      return false;
    }
    const std::vector<double> & diagnostic_costs =
      request.diagnostic_costs.empty() ? request.costs : request.diagnostic_costs;
    if(!checkCuda(cudaMemcpy(workspace_.sequences, request.sequences.data(),
        sequence_bytes, cudaMemcpyHostToDevice),
        "copy selected sequences", error) ||
      !checkCuda(cudaMemcpy(workspace_.costs, request.costs.data(),
        cost_bytes, cudaMemcpyHostToDevice),
        "copy candidate costs", error) ||
      !checkCuda(cudaMemcpy(workspace_.diagnostic_costs, diagnostic_costs.data(),
        cost_bytes, cudaMemcpyHostToDevice),
        "copy diagnostic candidate costs", error) ||
      !checkCuda(cudaMemcpy(workspace_.selected, request.selected.data(),
        count * sizeof(unsigned char), cudaMemcpyHostToDevice),
        "copy selected mask", error) ||
      !checkCuda(cudaMemcpy(workspace_.structured_in, request.structured.data(),
        count * sizeof(unsigned char), cudaMemcpyHostToDevice),
        "copy structured mask", error))
    {
      return false;
    }

    // Stable softmax: reduce the minima before exponentiating.
    minCostKernel<<<1, kThreads>>>(
      workspace_.diagnostic_costs,
      workspace_.costs,
      workspace_.selected,
      count,
      &workspace_.scalar[0],
      &workspace_.scalar[1]);
    if(!checkKernel("minCostKernel", error)){
      return false;
    }
    double minima[2] = {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
    if(!checkCuda(cudaMemcpy(minima, workspace_.scalar,
        2 * sizeof(double), cudaMemcpyDeviceToHost),
        "copy softmax minima", error))
    {
      return false;
    }
    if(!std::isfinite(minima[0])){
      error = "weighted update requires at least one finite candidate cost";
      return false;
    }
    if(!std::isfinite(minima[1])){
      error = "weighted update has no selected candidate";
      return false;
    }

    weightedStatisticsKernel<<<1, kThreads>>>(
      workspace_.diagnostic_costs,
      workspace_.costs,
      workspace_.sequences,
      workspace_.selected,
      workspace_.structured_in,
      count,
      request.time_steps,
      std::min(request.command_index, request.time_steps - 1),
      request.temperature,
      minima[0],
      minima[1],
      &workspace_.scalar[2],
      &workspace_.scalar[3],
      &workspace_.scalar[4],
      &workspace_.scalar[5],
      &workspace_.scalar[6],
      &workspace_.scalar[7],
      &workspace_.scalar[8],
      workspace_.count_scalar);
    if(!checkKernel("weightedStatisticsKernel", error)){
      return false;
    }

    double statistics[7] = {};
    std::size_t selected_count = 0;
    if(!checkCuda(cudaMemcpy(statistics, &workspace_.scalar[2],
        7 * sizeof(double), cudaMemcpyDeviceToHost),
        "copy softmax statistics", error) ||
      !checkCuda(cudaMemcpy(&selected_count, workspace_.count_scalar,
        sizeof(std::size_t), cudaMemcpyDeviceToHost),
        "copy selected count", error))
    {
      return false;
    }
    const double all_sum = statistics[0];
    const double all_squared_sum = statistics[1];
    const double selected_sum = statistics[5];
    if(!(all_sum > 1e-12) || !std::isfinite(all_sum)){
      error = "weighted update normalization failed";
      return false;
    }
    if(!(selected_sum > 1e-12) || !std::isfinite(selected_sum)){
      error = "selected weight normalization failed";
      return false;
    }

    const MppiCudaLaunchGeometry update_launch =
      mppiCudaSelectLaunch(request.time_steps, sm_count_);
    weightedControlKernel<<<update_launch.grid, update_launch.block_dim>>>(
      workspace_.sequences,
      workspace_.costs,
      workspace_.selected,
      count,
      request.time_steps,
      request.temperature,
      minima[1],
      selected_sum,
      workspace_.updated);
    if(!checkKernel("weightedControlKernel", error)){
      return false;
    }

    std::size_t updated_bytes = 0;
    if(!mppi_math::checkedBytes(
        request.time_steps, sizeof(MppiControl), updated_bytes))
    {
      error = "updated control sequence byte size overflow";
      return false;
    }
    MppiWeightedUpdateResult updated;
    updated.control_sequence.resize(request.time_steps);
    if(!checkCuda(cudaMemcpy(updated.control_sequence.data(), workspace_.updated,
        updated_bytes, cudaMemcpyDeviceToHost),
        "copy updated control sequence", error))
    {
      return false;
    }
    updated.positive_weight_mass = statistics[2] / all_sum;
    updated.negative_weight_mass = statistics[3] / all_sum;
    updated.neutral_weight_mass = statistics[4] / all_sum;
    updated.structured_weight_mass = statistics[6] / selected_sum;
    updated.selected_count = selected_count;
    if(all_squared_sum > 1e-12){
      updated.effective_sample_size = all_sum * all_sum / all_squared_sum;
    }
    result = std::move(updated);
    return true;
  }

private:
  struct Workspace
  {
    MppiControl * controls = nullptr;
    std::size_t controls_capacity = 0;
    double * energies = nullptr;
    std::size_t energy_capacity = 0;
    unsigned char * structured = nullptr;
    std::size_t structured_capacity = 0;
    MppiControl * warm = nullptr;
    std::size_t warm_capacity = 0;
    MppiControl * noise = nullptr;
    std::size_t noise_capacity = 0;
    MppiControl * sequences = nullptr;
    std::size_t sequence_capacity = 0;
    double * costs = nullptr;
    std::size_t cost_capacity = 0;
    double * diagnostic_costs = nullptr;
    std::size_t diagnostic_cost_capacity = 0;
    unsigned char * selected = nullptr;
    std::size_t selected_capacity = 0;
    unsigned char * structured_in = nullptr;
    std::size_t structured_in_capacity = 0;
    MppiControl * updated = nullptr;
    std::size_t updated_capacity = 0;
    double * scalar = nullptr;
    std::size_t scalar_capacity = 0;
    std::size_t * count_scalar = nullptr;
    std::size_t count_capacity = 0;
  };

  void release()
  {
    // Reusable buffers live for the backend's lifetime; release only at
    // destruction (or when a larger allocation replaces one).
    cudaFree(workspace_.controls);
    cudaFree(workspace_.energies);
    cudaFree(workspace_.structured);
    cudaFree(workspace_.warm);
    cudaFree(workspace_.noise);
    cudaFree(workspace_.sequences);
    cudaFree(workspace_.costs);
    cudaFree(workspace_.diagnostic_costs);
    cudaFree(workspace_.selected);
    cudaFree(workspace_.structured_in);
    cudaFree(workspace_.updated);
    cudaFree(workspace_.scalar);
    cudaFree(workspace_.count_scalar);
    workspace_ = Workspace{};
  }

  mutable Workspace workspace_;
  int sm_count_{1};
};

}  // namespace

std::unique_ptr<MppiCudaBackend> makeMppiCudaBackend()
{
  return std::make_unique<MppiCudaBackendImpl>();
}

}  // namespace local_planner
