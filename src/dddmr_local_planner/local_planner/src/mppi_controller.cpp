#include "local_planner/mppi_controller.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace local_planner
{

bool mppiSequenceRespectsSpeedCap(
  const std::vector<MppiControl> & controls,
  const double measured_vx,
  const double speed_cap,
  const bool first_control_is_measured_state,
  const double ax_min,
  const double model_dt,
  const double controller_period)
{
  if(speed_cap <= 0.0){
    return true;
  }

  const std::size_t first_checked_control =
    first_control_is_measured_state && !controls.empty() ? 1 : 0;
  double elapsed = first_control_is_measured_state ? 0.0 : controller_period;
  for(std::size_t step = first_checked_control; step < controls.size(); ++step){
    if(first_control_is_measured_state || step > first_checked_control){
      elapsed += model_dt;
    }
    const double vx = controls[step].vx;
    const double braking_envelope = std::max(
      speed_cap,
      measured_vx + std::min(0.0, ax_min) * elapsed);
    if(vx > braking_envelope + 1e-6){
      return false;
    }
  }
  return true;
}

MppiController::MppiController()
: MppiController(MppiConfig())
{
}

MppiController::MppiController(const MppiConfig & config)
{
  setConfig(config);
}

void MppiController::setConfig(const MppiConfig & config)
{
  config_ = config;
  normalizeConfig();
  generator_.seed(config_.seed);
  use_debug_noise_for_test_ = false;
  control_sequence_.assign(config_.time_steps, MppiControl{0.0, 0.0});
  control_history_.fill(MppiControl{});
  regenerateNoise();
}

const MppiConfig & MppiController::config() const
{
  return config_;
}

MppiControl MppiController::debugNoiseAt(const std::size_t sample_index, const std::size_t step_index) const
{
  if(noise_.empty()){
    return {};
  }
  const auto & sample = noise_[sample_index % noise_.size()];
  if(sample.empty()){
    return {};
  }
  return sample[step_index % sample.size()];
}

const std::vector<MppiControl> & MppiController::controlSequence() const
{
  return control_sequence_;
}

void MppiController::resetControlSequence(const MppiControl & command)
{
  control_sequence_.assign(config_.time_steps, command);
  control_history_.fill(command);
  latched_bypass_side_ = MppiBypassSide::Neutral;
  latched_side_infeasible_cycles_ = 0;
}

void MppiController::setDebugNoiseForTest(const std::vector<std::vector<MppiControl>> & noise)
{
  noise_ = noise;
  use_debug_noise_for_test_ = true;
}

MppiResult MppiController::optimize(
  const MppiControl & measured_speed,
  const ScoreFunction & score_function)
{
  if(!score_function){
    MppiResult result;
    result.rejected_samples = config_.batch_size;
    result.optimal_sequence = control_sequence_;
    return result;
  }

  return optimizeBatch(
    measured_speed,
    [&](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores;
      scores.reserve(candidates.size());
      for(const auto & candidate : candidates){
        scores.push_back(score_function(candidate));
      }
      return scores;
    });
}

MppiResult MppiController::optimizeBatch(
  const MppiControl & measured_speed,
  const BatchScoreFunction & score_function,
  const ModeSelectionFunction & mode_selection_function)
{
  MppiResult result;
  result.structured_candidates = use_debug_noise_for_test_ ? 0 :
    config_.structured_candidate_count;
  if(!score_function){
    result.rejected_samples = config_.batch_size;
    result.optimal_sequence = control_sequence_;
    return result;
  }

  for(std::size_t iteration = 0; iteration < config_.iteration_count; ++iteration){
    if(config_.shift_control_sequence && !control_sequence_.empty()){
      control_sequence_.front() = measured_speed;
    }
    if(config_.regenerate_noises && !use_debug_noise_for_test_){
      regenerateNoise();
    }
    std::vector<std::vector<MppiControl>> valid_sequences;
    std::vector<double> valid_costs;
    std::vector<MppiBypassSide> valid_sides;
    std::vector<bool> collision_free;
    result.rejected_samples = 0;
    result.best_cost = std::numeric_limits<double>::infinity();
    result.best_sample_sequence.clear();

    std::vector<std::vector<MppiControl>> sampled_sequences;
    std::vector<MppiCandidate> candidates;
    sampled_sequences.reserve(config_.batch_size);
    candidates.reserve(config_.batch_size);
    for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
      sampled_sequences.push_back(buildNoisedSequence(measured_speed, sample));
      candidates.push_back(MppiCandidate{sampled_sequences.back()});
    }

    const auto scores = score_function(candidates);
    for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
      const auto & sequence = sampled_sequences[sample];
      const auto score =
        sample < scores.size() ? scores[sample] : MppiCandidateScore{false, -1.0};
      if(!score.valid || !std::isfinite(score.cost) || score.cost < 0.0){
        ++result.rejected_samples;
        continue;
      }

      double control_energy = 0.0;
      for(std::size_t step = 0; step < sequence.size(); ++step){
        if(config_.vx_std > 1e-9){
          const double bounded_noise_vx = sequence[step].vx - control_sequence_[step].vx;
          control_energy +=
            config_.gamma / (config_.vx_std * config_.vx_std) *
            control_sequence_[step].vx * bounded_noise_vx;
        }
        if(config_.wz_std > 1e-9){
          const double bounded_noise_wz = sequence[step].wz - control_sequence_[step].wz;
          control_energy +=
            config_.gamma / (config_.wz_std * config_.wz_std) *
            control_sequence_[step].wz * bounded_noise_wz;
        }
      }
      const double total_cost = score.cost + control_energy;

      if(total_cost < result.best_cost){
        result.best_cost = total_cost;
        result.best_sample_index = sample;
        result.best_sample_sequence = sequence;
      }

      valid_sequences.push_back(sequence);
      valid_costs.push_back(total_cost);
      valid_sides.push_back(score.bypass_side);
      collision_free.push_back(score.collision_free);
    }

    const bool select_directional_mode =
      mode_selection_function && mode_selection_function();
    const auto statistics = applySoftmaxUpdate(
      valid_sequences,
      valid_costs,
      valid_sides,
      collision_free,
      select_directional_mode);
    result.effective_sample_size = statistics.effective_sample_size;
    result.positive_weight_mass = statistics.positive_weight_mass;
    result.negative_weight_mass = statistics.negative_weight_mass;
    result.neutral_weight_mass = statistics.neutral_weight_mass;
    result.prefilter_command = statistics.prefilter_command;
    result.selected_bypass_side = latched_bypass_side_;
    result.side_infeasible_cycles = latched_side_infeasible_cycles_;
    if(!statistics.valid){
      result.has_valid_control = false;
      result.optimal_sequence = control_sequence_;
      return result;
    }
  }

  result.has_valid_control = true;
  smoothControlSequence(measured_speed);
  result.optimal_sequence = control_sequence_;
  const std::size_t command_index =
    config_.shift_control_sequence && control_sequence_.size() > 1 ? 1 : 0;
  result.command = control_sequence_[command_index];
  updateControlHistory(result.command);
  shiftControlSequence();
  return result;
}

void MppiController::normalizeConfig()
{
  config_.batch_size = std::max<std::size_t>(1, config_.batch_size);
  config_.structured_candidate_count = std::min(
    config_.structured_candidate_count,
    config_.batch_size);
  config_.structured_candidate_count -= config_.structured_candidate_count % 2;
  config_.time_steps = std::max<std::size_t>(1, config_.time_steps);
  config_.iteration_count = std::max<std::size_t>(1, config_.iteration_count);
  config_.model_dt = std::max(1e-3, config_.model_dt);
  config_.controller_period = std::max(1e-3, config_.controller_period);
  config_.temperature = std::max(1e-6, config_.temperature);
  config_.wheelbase = std::max(1e-6, config_.wheelbase);
  config_.max_steer = std::max(1e-6, std::fabs(config_.max_steer));
  config_.vx_max = std::max(config_.vx_min, config_.vx_max);
  config_.wz_max = std::max(0.0, config_.wz_max);
  config_.ax_max = std::fabs(config_.ax_max);
  config_.ax_min = config_.ax_min > 0.0 ? -config_.ax_min : config_.ax_min;
  config_.az_max = std::fabs(config_.az_max);
  config_.vx_std = std::max(0.0, config_.vx_std);
  config_.wz_std = std::max(0.0, config_.wz_std);
}

void MppiController::regenerateNoise()
{
  std::normal_distribution<double> vx_dist(0.0, config_.vx_std);
  std::normal_distribution<double> wz_dist(0.0, config_.wz_std);

  noise_.assign(config_.batch_size, std::vector<MppiControl>(config_.time_steps));
  for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
    for(std::size_t step = 0; step < config_.time_steps; ++step){
      noise_[sample][step].vx = vx_dist(generator_);
      noise_[sample][step].wz = wz_dist(generator_);
    }
  }
}

double MppiController::clamp(const double value, const double low, const double high) const
{
  return std::max(low, std::min(high, value));
}

double MppiController::ackermannWzLimit(const double vx) const
{
  const double steering_limit = std::fabs(vx) * std::tan(config_.max_steer) / config_.wheelbase;
  return std::min(config_.wz_max, steering_limit);
}

MppiControl MppiController::constrainControl(
  const MppiControl & previous,
  const MppiControl & requested,
  const double dt) const
{
  MppiControl constrained;
  constrained.vx = clamp(
    requested.vx,
    previous.vx + config_.ax_min * dt,
    previous.vx + config_.ax_max * dt);
  constrained.vx = clamp(constrained.vx, config_.vx_min, config_.vx_max);

  const double requested_wz = clamp(
    requested.wz,
    previous.wz - config_.az_max * dt,
    previous.wz + config_.az_max * dt);
  const double wz_limit = ackermannWzLimit(constrained.vx);
  constrained.wz = clamp(requested_wz, -wz_limit, wz_limit);
  return constrained;
}

std::vector<MppiControl> MppiController::buildNoisedSequence(
  const MppiControl & measured_speed,
  const std::size_t sample_index) const
{
  if(!use_debug_noise_for_test_ && sample_index < config_.structured_candidate_count){
    return buildStructuredSequence(measured_speed, sample_index);
  }

  std::vector<MppiControl> sequence(config_.time_steps);
  MppiControl previous = measured_speed;
  std::size_t first_noised_step = 0;
  if(config_.shift_control_sequence && !sequence.empty()){
    sequence.front() = measured_speed;
    previous = sequence.front();
    first_noised_step = 1;
  }
  for(std::size_t step = first_noised_step; step < config_.time_steps; ++step){
    const auto noise = debugNoiseAt(sample_index, step);
    MppiControl requested;
    requested.vx = control_sequence_[step].vx + noise.vx;
    const double dt = step == 0 ? config_.controller_period : config_.model_dt;
    const MppiControl vx_only = constrainControl(
      previous,
      MppiControl{requested.vx, previous.wz},
      dt);
    const double angular_noise_scale = use_debug_noise_for_test_ ? 1.0 :
      (config_.wz_std > 1e-9 ?
      std::min(1.0, ackermannWzLimit(vx_only.vx) / config_.wz_std) : 0.0);
    requested.wz = control_sequence_[step].wz + noise.wz * angular_noise_scale;
    sequence[step] = constrainControl(previous, requested, dt);
    previous = sequence[step];
  }
  return sequence;
}

std::vector<MppiControl> MppiController::buildStructuredSequence(
  const MppiControl & measured_speed,
  const std::size_t sample_index) const
{
  std::vector<MppiControl> sequence(config_.time_steps);
  MppiControl previous = measured_speed;
  std::size_t first_control_step = 0;
  if(config_.shift_control_sequence && !sequence.empty()){
    sequence.front() = measured_speed;
    first_control_step = 1;
  }

  const std::size_t template_index = sample_index / 2;
  const double direction = sample_index % 2 == 0 ? 1.0 : -1.0;
  const double intensity = template_index % 2 == 0 ? 0.55 : 0.90;
  const bool s_curve = template_index >= 2;
  const std::size_t controlled_steps = config_.time_steps - first_control_step;

  for(std::size_t step = first_control_step; step < config_.time_steps; ++step){
    const double dt = step == 0 ? config_.controller_period : config_.model_dt;
    const MppiControl vx_only = constrainControl(
      previous,
      MppiControl{control_sequence_[step].vx, previous.wz},
      dt);
    const std::size_t phase_step = step - first_control_step;
    const double phase_direction =
      s_curve && phase_step >= controlled_steps / 2 ? -direction : direction;
    const MppiControl requested{
      control_sequence_[step].vx,
      phase_direction * intensity * ackermannWzLimit(vx_only.vx)};
    sequence[step] = constrainControl(previous, requested, dt);
    previous = sequence[step];
  }
  return sequence;
}

MppiController::UpdateStatistics MppiController::applySoftmaxUpdate(
  const std::vector<std::vector<MppiControl>> & valid_sequences,
  const std::vector<double> & valid_costs,
  const std::vector<MppiBypassSide> & valid_sides,
  const std::vector<bool> & collision_free,
  const bool select_directional_mode)
{
  UpdateStatistics statistics;
  if(!select_directional_mode){
    latched_bypass_side_ = MppiBypassSide::Neutral;
    latched_side_infeasible_cycles_ = 0;
  }

  if(valid_sequences.empty() || valid_sequences.size() != valid_costs.size() ||
    valid_sequences.size() != valid_sides.size() ||
    valid_sequences.size() != collision_free.size())
  {
    if(select_directional_mode && latched_bypass_side_ != MppiBypassSide::Neutral){
      latched_side_infeasible_cycles_ = std::min(
        latched_side_infeasible_cycles_ + 1,
        kSideSwitchConfirmCycles);
    }
    return statistics;
  }

  const double min_cost = *std::min_element(valid_costs.begin(), valid_costs.end());
  std::vector<double> weights(valid_costs.size(), 0.0);
  double weight_sum = 0.0;
  for(std::size_t i = 0; i < valid_costs.size(); ++i){
    weights[i] = std::exp(-(valid_costs[i] - min_cost) / config_.temperature);
    weight_sum += weights[i];
  }
  if(weight_sum <= 1e-12 || !std::isfinite(weight_sum)){
    return statistics;
  }

  std::vector<MppiControl> updated(config_.time_steps);
  const std::size_t command_index =
    config_.shift_control_sequence && config_.time_steps > 1 ? 1 : 0;
  double squared_weight_sum = 0.0;
  constexpr double angular_deadband = 1e-4;
  for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
    const double normalized_weight = weights[sample] / weight_sum;
    squared_weight_sum += normalized_weight * normalized_weight;
    const double command_wz = valid_sequences[sample][command_index].wz;
    if(command_wz > angular_deadband){
      statistics.positive_weight_mass += normalized_weight;
    }
    else if(command_wz < -angular_deadband){
      statistics.negative_weight_mass += normalized_weight;
    }
    else{
      statistics.neutral_weight_mass += normalized_weight;
    }
  }
  if(squared_weight_sum > 1e-12){
    statistics.effective_sample_size = 1.0 / squared_weight_sum;
  }

  std::vector<bool> selected(valid_sequences.size(), true);
  if(select_directional_mode){
    const auto has_feasible_side = [&](const MppiBypassSide side) {
        for(std::size_t sample = 0; sample < valid_sides.size(); ++sample){
          if(collision_free[sample] && valid_sides[sample] == side){
            return true;
          }
        }
        return false;
      };

    if(latched_bypass_side_ == MppiBypassSide::Neutral){
      double eligible_min_cost = std::numeric_limits<double>::infinity();
      for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
        if(collision_free[sample] &&
          valid_sides[sample] != MppiBypassSide::Neutral)
        {
          eligible_min_cost = std::min(eligible_min_cost, valid_costs[sample]);
        }
      }

      double left_weight_mass = 0.0;
      double right_weight_mass = 0.0;
      for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
        if(!collision_free[sample] ||
          valid_sides[sample] == MppiBypassSide::Neutral)
        {
          continue;
        }
        const double eligible_weight = std::exp(
          -(valid_costs[sample] - eligible_min_cost) / config_.temperature);
        if(valid_sides[sample] == MppiBypassSide::Left){
          left_weight_mass += eligible_weight;
        }
        else if(valid_sides[sample] == MppiBypassSide::Right){
          right_weight_mass += eligible_weight;
        }
      }

      if(left_weight_mass > right_weight_mass){
        latched_bypass_side_ = MppiBypassSide::Left;
      }
      else if(right_weight_mass > left_weight_mass){
        latched_bypass_side_ = MppiBypassSide::Right;
      }
      else{
        double lowest_cost = std::numeric_limits<double>::infinity();
        for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
          if(collision_free[sample] &&
            valid_sides[sample] != MppiBypassSide::Neutral &&
            valid_costs[sample] < lowest_cost)
          {
            lowest_cost = valid_costs[sample];
            latched_bypass_side_ = valid_sides[sample];
          }
        }
      }
    }

    if(latched_bypass_side_ == MppiBypassSide::Neutral){
      return statistics;
    }

    if(has_feasible_side(latched_bypass_side_)){
      latched_side_infeasible_cycles_ = 0;
    }
    else{
      latched_side_infeasible_cycles_ = std::min(
        latched_side_infeasible_cycles_ + 1,
        kSideSwitchConfirmCycles);
      if(latched_side_infeasible_cycles_ < kSideSwitchConfirmCycles){
        return statistics;
      }

      const MppiBypassSide opposite_side =
        latched_bypass_side_ == MppiBypassSide::Left ?
        MppiBypassSide::Right : MppiBypassSide::Left;
      if(!has_feasible_side(opposite_side)){
        return statistics;
      }
      latched_bypass_side_ = opposite_side;
      latched_side_infeasible_cycles_ = 0;
    }

    for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
      selected[sample] = collision_free[sample] &&
        valid_sides[sample] == latched_bypass_side_;
    }
  }

  double selected_min_cost = std::numeric_limits<double>::infinity();
  for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
    if(selected[sample]){
      selected_min_cost = std::min(selected_min_cost, valid_costs[sample]);
    }
  }
  std::vector<double> selected_weights(valid_sequences.size(), 0.0);
  double selected_weight_sum = 0.0;
  for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
    if(selected[sample]){
      selected_weights[sample] =
        std::exp(-(valid_costs[sample] - selected_min_cost) / config_.temperature);
      selected_weight_sum += selected_weights[sample];
    }
  }
  if(selected_weight_sum <= 1e-12 || !std::isfinite(selected_weight_sum)){
    return statistics;
  }

  for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
    const double normalized_weight =
      selected_weights[sample] / selected_weight_sum;
    for(std::size_t step = 0; step < config_.time_steps; ++step){
      updated[step].vx += normalized_weight * valid_sequences[sample][step].vx;
      updated[step].wz += normalized_weight * valid_sequences[sample][step].wz;
    }
  }
  control_sequence_ = updated;
  statistics.valid = true;
  statistics.prefilter_command = updated[command_index];
  return statistics;
}

void MppiController::smoothControlSequence(const MppiControl & measured_speed)
{
  if(control_sequence_.size() < 20){
    return;
  }

  static constexpr std::array<double, 9> coefficients{
    -21.0 / 231.0, 14.0 / 231.0, 39.0 / 231.0,
    54.0 / 231.0, 59.0 / 231.0, 54.0 / 231.0,
    39.0 / 231.0, 14.0 / 231.0, -21.0 / 231.0};
  const auto unfiltered = control_sequence_;
  std::vector<MppiControl> filtered(unfiltered.size());

  const auto sample_at = [&](const long index) -> const MppiControl & {
      if(index < 0){
        return control_history_[static_cast<std::size_t>(index + 4)];
      }
      if(static_cast<std::size_t>(index) >= unfiltered.size()){
        return unfiltered.back();
      }
      return unfiltered[static_cast<std::size_t>(index)];
    };

  for(std::size_t step = 0; step < unfiltered.size(); ++step){
    MppiControl value;
    for(std::size_t tap = 0; tap < coefficients.size(); ++tap){
      const long sample_index =
        static_cast<long>(step) + static_cast<long>(tap) - 4L;
      const auto & sample = sample_at(sample_index);
      value.vx += coefficients[tap] * sample.vx;
      value.wz += coefficients[tap] * sample.wz;
    }
    filtered[step] = value;
  }

  MppiControl previous = measured_speed;
  std::size_t first_filtered_step = 0;
  if(config_.shift_control_sequence){
    control_sequence_.front() = measured_speed;
    first_filtered_step = 1;
  }
  for(std::size_t step = first_filtered_step; step < filtered.size(); ++step){
    const double dt = step == 0 ? config_.controller_period : config_.model_dt;
    const auto constrained = constrainControl(previous, filtered[step], dt);
    if(!std::isfinite(constrained.vx) || !std::isfinite(constrained.wz)){
      control_sequence_ = unfiltered;
      return;
    }
    control_sequence_[step] = constrained;
    previous = constrained;
  }
}

void MppiController::updateControlHistory(const MppiControl & command)
{
  for(std::size_t i = 0; i + 1 < control_history_.size(); ++i){
    control_history_[i] = control_history_[i + 1];
  }
  control_history_.back() = command;
}

void MppiController::shiftControlSequence()
{
  if(control_sequence_.size() < 2 || !config_.shift_control_sequence){
    return;
  }
  for(std::size_t i = 0; i + 1 < control_sequence_.size(); ++i){
    control_sequence_[i] = control_sequence_[i + 1];
  }
  control_sequence_.back() = control_sequence_[control_sequence_.size() - 2];
}

}  // namespace local_planner
