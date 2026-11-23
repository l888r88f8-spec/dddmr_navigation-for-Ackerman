#include "local_planner/mppi_cuda_backend.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "local_planner/mppi_batch_math.h"

namespace local_planner
{
namespace
{

using mppi_math::kAngularDeadband;

}  // namespace

bool mppiGenerateCandidatesCpu(
  const MppiGenerationRequest & request,
  MppiCandidateBatch & batch,
  std::string & error)
{
  batch = MppiCandidateBatch{};
  const auto & params = request.params;
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

  std::size_t total = 0;
  if(!mppi_math::checkedMulSize(params.batch_size, params.time_steps, total)){
    error = "candidate generation size overflow";
    return false;
  }
  batch.batch_size = params.batch_size;
  batch.time_steps = params.time_steps;
  batch.controls.assign(total, MppiControl{});
  batch.control_energy.assign(params.batch_size, 0.0);
  batch.structured.assign(params.batch_size, 0u);

  const std::size_t structured_count =
    params.use_debug_noise ? 0 :
    std::min(params.structured_candidate_count, params.batch_size);
  for(std::size_t sample = 0; sample < params.batch_size; ++sample){
    auto * row = batch.controls.data() + sample * params.time_steps;
    if(params.reverse_mode){
      const bool structured = sample < structured_count &&
        params.reverse_speed_ceiling > 1e-6;
      batch.structured[sample] = structured ? 1u : 0u;
      double energy = 0.0;
      mppi_math::generateReverseSample(
        params,
        request.measured_speed,
        request.noise.data(),
        request.noise.size(),
        request.noise_batch_size,
        sample,
        structured,
        row,
        energy);
      batch.control_energy[sample] = structured ? 0.0 : energy;
    }
    else if(sample < structured_count){
      batch.structured[sample] = 1u;
      mppi_math::generateStructuredSample(
        params,
        request.measured_speed,
        request.control_sequence.data(),
        request.control_sequence.size(),
        sample,
        row);
      batch.control_energy[sample] = 0.0;
    }
    else{
      double energy = 0.0;
      mppi_math::generateNoisedSample(
        params,
        request.measured_speed,
        request.control_sequence.data(),
        request.control_sequence.size(),
        request.noise.data(),
        request.noise.size(),
        request.noise_batch_size,
        sample,
        row,
        energy);
      batch.control_energy[sample] = energy;
    }
  }
  return true;
}

bool mppiWeightedUpdateCpu(
  const MppiWeightedUpdateRequest & request,
  MppiWeightedUpdateResult & result,
  std::string & error)
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

  // Diagnostics always use the raw valid costs. Selection costs may include
  // Depart soft penalties and must never overwrite these statistics.
  const std::vector<double> & diagnostic_costs =
    request.diagnostic_costs.empty() ? request.costs : request.diagnostic_costs;
  double min_cost = std::numeric_limits<double>::infinity();
  for(const double cost : diagnostic_costs){
    min_cost = std::min(min_cost, cost);
  }
  if(!std::isfinite(min_cost)){
    error = "weighted update requires at least one finite candidate cost";
    return false;
  }

  std::vector<double> weights(count, 0.0);
  double weight_sum = 0.0;
  double squared_weight_sum = 0.0;
  for(std::size_t sample = 0; sample < count; ++sample){
    weights[sample] = std::exp(
      -(diagnostic_costs[sample] - min_cost) / request.temperature);
    weight_sum += weights[sample];
  }
  if(!(weight_sum > 1e-12) || !std::isfinite(weight_sum)){
    error = "weighted update normalization failed";
    return false;
  }

  const std::size_t command_index = std::min(
    request.command_index, request.time_steps - 1);
  result = MppiWeightedUpdateResult{};
  for(std::size_t sample = 0; sample < count; ++sample){
    const double normalized = weights[sample] / weight_sum;
    squared_weight_sum += normalized * normalized;
    const double command_wz =
      request.sequences[sample * request.time_steps + command_index].wz;
    if(command_wz > kAngularDeadband){
      result.positive_weight_mass += normalized;
    }
    else if(command_wz < -kAngularDeadband){
      result.negative_weight_mass += normalized;
    }
    else{
      result.neutral_weight_mass += normalized;
    }
  }
  if(squared_weight_sum > 1e-12){
    result.effective_sample_size = 1.0 / squared_weight_sum;
  }

  double selected_min_cost = std::numeric_limits<double>::infinity();
  for(std::size_t sample = 0; sample < count; ++sample){
    if(request.selected[sample]){
      selected_min_cost = std::min(selected_min_cost, request.costs[sample]);
      ++result.selected_count;
    }
  }
  if(result.selected_count == 0 || !std::isfinite(selected_min_cost)){
    error = "weighted update has no selected candidate";
    return false;
  }

  std::vector<double> selected_weights(count, 0.0);
  double selected_weight_sum = 0.0;
  for(std::size_t sample = 0; sample < count; ++sample){
    if(request.selected[sample]){
      selected_weights[sample] = std::exp(
        -(request.costs[sample] - selected_min_cost) / request.temperature);
      selected_weight_sum += selected_weights[sample];
    }
  }
  if(!(selected_weight_sum > 1e-12) || !std::isfinite(selected_weight_sum)){
    error = "selected weight normalization failed";
    return false;
  }

  result.control_sequence.assign(request.time_steps, MppiControl{});
  for(std::size_t sample = 0; sample < count; ++sample){
    if(!request.selected[sample]){
      continue;
    }
    const double normalized = selected_weights[sample] / selected_weight_sum;
    if(request.structured[sample]){
      result.structured_weight_mass += normalized;
    }
    for(std::size_t step = 0; step < request.time_steps; ++step){
      const auto & control = request.sequences[sample * request.time_steps + step];
      result.control_sequence[step].vx += normalized * control.vx;
      result.control_sequence[step].wz += normalized * control.wz;
    }
  }
  return true;
}

}  // namespace local_planner
