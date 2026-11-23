#include "local_planner/mppi_controller.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "local_planner/mppi_batch_math.h"
#include "local_planner/mppi_cuda_backend.h"

namespace local_planner
{

const char * toString(const MppiBypassSide side)
{
  switch(side){
    case MppiBypassSide::Left:
      return "left";
    case MppiBypassSide::Right:
      return "right";
    case MppiBypassSide::Neutral:
      return "neutral";
  }
  return "neutral";
}

const char * toString(const MppiComputeBackend backend)
{
  return backend == MppiComputeBackend::Cuda ? "cuda" : "cpu";
}

bool parseMppiComputeBackend(
  const std::string & value, MppiComputeBackend & backend)
{
  if(value == "cpu"){
    backend = MppiComputeBackend::Cpu;
    return true;
  }
  if(value == "cuda"){
    backend = MppiComputeBackend::Cuda;
    return true;
  }
  return false;
}

const char * toString(const MppiAvoidanceStage stage)
{
  switch(stage){
    case MppiAvoidanceStage::Depart:
      return "depart";
    case MppiAvoidanceStage::Bypass:
      return "bypass";
    case MppiAvoidanceStage::Neutral:
      return "neutral";
  }
  return "neutral";
}

bool mppiSequenceRespectsSpeedCap(
  const std::vector<MppiControl> & controls,
  const double measured_vx,
  const double speed_cap,
  const bool first_control_is_measured_state,
  const double ax_min,
  const double model_dt,
  const double controller_period,
  const double ax_max)
{
  if(speed_cap <= 0.0){
    return true;
  }

  const double negative_accel = std::min(0.0, ax_min);
  const double positive_accel = std::isfinite(ax_max) && ax_max >= 0.0 ?
    std::fabs(ax_max) : std::fabs(ax_min);
  const std::size_t first_checked_control =
    first_control_is_measured_state && !controls.empty() ? 1 : 0;
  double previous = measured_vx;
  for(std::size_t step = first_checked_control; step < controls.size(); ++step){
    const double dt = (step == first_checked_control && !first_control_is_measured_state) ?
      controller_period : model_dt;
    const double reachable_low = previous + negative_accel * dt;
    const double reachable_high = previous + positive_accel * dt;
    const double requested = controls[step].vx;
    if(requested < reachable_low - 1e-9 || requested > reachable_high + 1e-9){
      // A new cap or a zero crossing must still be reached under the physical
      // acceleration limits. Clamping straight to the cap would teleport.
      return false;
    }

    // Above the magnitude cap, the only legal motion is a deceleration within
    // the appropriate braking envelope. Forward braking uses ax_min and
    // reverse braking (toward zero) uses ax_max.
    double allowed_low = -speed_cap;
    double allowed_high = speed_cap;
    if(previous > speed_cap){
      allowed_high = std::max(speed_cap, previous + negative_accel * dt);
    }
    if(previous < -speed_cap){
      allowed_low = std::min(-speed_cap, previous + positive_accel * dt);
    }
    if(requested < allowed_low - 1e-9 || requested > allowed_high + 1e-9){
      return false;
    }
    previous = requested;
  }
  return true;
}

double mppiGoalApproachSpeedCap(
  const double distance_to_goal,
  const double cruise_cap,
  const double slowdown_distance,
  const double min_speed)
{
  if(!std::isfinite(cruise_cap) || cruise_cap <= 0.0){
    return std::max(0.0, cruise_cap);
  }
  const double floor = std::max(0.0, std::min(min_speed, cruise_cap));
  if(!std::isfinite(distance_to_goal) || slowdown_distance <= 1e-3 ||
    distance_to_goal >= slowdown_distance)
  {
    return cruise_cap;
  }
  const double ratio =
    std::max(0.0, std::min(1.0, distance_to_goal / slowdown_distance));
  return floor + (cruise_cap - floor) * ratio;
}

MppiController::MppiController()
: MppiController(MppiConfig())
{
}

MppiController::MppiController(const MppiConfig & config)
{
  setConfig(config);
}

MppiController::~MppiController() = default;

void MppiController::setConfig(const MppiConfig & config)
{
  config_ = config;
  normalizeConfig();
  generator_.seed(config_.seed);
  cuda_noise_generation_ = 0;
  use_debug_noise_for_test_ = false;
  control_sequence_.assign(config_.time_steps, MppiControl{0.0, 0.0});
  reverse_control_sequence_.assign(config_.time_steps, MppiControl{0.0, 0.0});
  control_history_.fill(MppiControl{});
  latched_bypass_side_ = MppiBypassSide::Neutral;
  latched_side_infeasible_cycles_ = 0;
  latched_side_quality_advantage_cycles_ = 0;
  geometry_switch_confirm_cycles_ = 0;
  avoidance_stage_ = MppiAvoidanceStage::Neutral;
  reverse_recovery_active_ = false;
  reverse_motion_active_ = false;
  reverse_distance_used_ = 0.0;
  reverse_forward_travel_ = 0.0;
  reverse_forward_exit_cycles_ = 0;
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

MppiAvoidanceState MppiController::avoidanceState() const
{
  return MppiAvoidanceState{
    latched_bypass_side_,
    latched_side_infeasible_cycles_,
    avoidance_stage_,
    latched_side_quality_advantage_cycles_};
}

bool MppiController::reverseRecoveryActive() const
{
  return reverse_recovery_active_;
}

bool MppiController::reverseMotionActive() const
{
  return reverse_motion_active_;
}

double MppiController::reverseDistanceUsed() const
{
  return reverse_distance_used_;
}

double MppiController::reverseDistanceRemaining() const
{
  return std::numeric_limits<double>::infinity();
}

void MppiController::noteMeasuredReverseTravel(const double distance)
{
  if(!config_.reverse_recovery_enabled || !reverse_recovery_active_){
    return;
  }
  if(!std::isfinite(distance) || distance <= 0.0){
    return;
  }
  reverse_distance_used_ += distance;
}

void MppiController::noteMeasuredForwardTravel(const double distance)
{
  if(!config_.reverse_recovery_enabled || !reverse_recovery_active_){
    return;
  }
  if(!std::isfinite(distance) || distance <= 0.0){
    return;
  }
  reverse_forward_travel_ += distance;

}

void MppiController::resetReverseRecoveryEpisode()
{
  reverse_recovery_active_ = false;
  reverse_motion_active_ = false;
  reverse_control_sequence_.assign(config_.time_steps, MppiControl{0.0, 0.0});
  reverse_distance_used_ = 0.0;
  reverse_forward_travel_ = 0.0;
  reverse_forward_exit_cycles_ = 0;
}

void MppiController::resetControlSequence(const MppiControl & command)
{
  if(reverse_motion_active_){
    // A stop/replan retry during reverse recovery must not erase the forward
    // bypass latch or the separate reverse seed. The measured-travel budget is
    // likewise intentionally preserved by this function.
    reverse_control_sequence_.assign(config_.time_steps, command);
    return;
  }
  control_sequence_.assign(config_.time_steps, command);
  control_history_.fill(command);
  latched_bypass_side_ = MppiBypassSide::Neutral;
  latched_side_infeasible_cycles_ = 0;
  latched_side_quality_advantage_cycles_ = 0;
  geometry_switch_confirm_cycles_ = 0;
  avoidance_stage_ = MppiAvoidanceStage::Neutral;
}

void MppiController::resetControlSequenceForRetry(
  const MppiControl & command,
  const MppiAvoidanceState & avoidance_state)
{
  if(reverse_motion_active_){
    reverse_control_sequence_.assign(config_.time_steps, command);
    return;
  }
  control_sequence_.assign(config_.time_steps, command);
  control_history_.fill(command);

  if(avoidance_state.side != MppiBypassSide::Left &&
    avoidance_state.side != MppiBypassSide::Right)
  {
    latched_bypass_side_ = MppiBypassSide::Neutral;
    latched_side_infeasible_cycles_ = 0;
    latched_side_quality_advantage_cycles_ = 0;
    geometry_switch_confirm_cycles_ = 0;
    avoidance_stage_ = MppiAvoidanceStage::Neutral;
    return;
  }

  // A geometric transition confirmation is deliberately not restored here: a
  // retry must re-observe the geometry instead of inheriting stale evidence.
  geometry_switch_confirm_cycles_ = 0;
  latched_bypass_side_ = avoidance_state.side;
  latched_side_infeasible_cycles_ = std::min(
    avoidance_state.side_infeasible_cycles,
    kSideSwitchConfirmCycles);
  latched_side_quality_advantage_cycles_ = std::min(
    avoidance_state.side_quality_advantage_cycles,
    kSideSwitchConfirmCycles);
  avoidance_stage_ = avoidance_state.stage == MppiAvoidanceStage::Bypass ?
    MppiAvoidanceStage::Bypass : MppiAvoidanceStage::Depart;
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
    result.avoidance_stage = avoidance_stage_;
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
  const ModeSelectionFunction & mode_selection_function,
  const double runtime_speed_limit,
  const RecoveryFunction & recovery_function)
{
  recovery_function_ = recovery_function;
  if(recovery_function_ && reverse_recovery_active_){
    auto recovery = recovery_function_(true);
    reverse_recovery_active_ = recovery.reverse_recovery_active;
    reverse_motion_active_ = recovery.reverse_motion_active;
    recovery.reverse_distance_used = reverse_distance_used_;
    recovery.reverse_distance_remaining = reverseDistanceRemaining();
    if(recovery.reverse_recovery_active || recovery.has_valid_control){
      return recovery;
    }
  }
  MppiResult result;
  result.avoidance_stage = avoidance_stage_;
  if(!score_function){
    result.rejected_samples = config_.batch_size;
    result.optimal_sequence = control_sequence_;
    result.reverse_recovery_active = reverse_recovery_active_;
    result.reverse_motion_active = reverse_motion_active_;
    result.reverse_distance_used = reverse_distance_used_;
    result.reverse_distance_remaining = reverseDistanceRemaining();
    return result;
  }

  if(config_.compute_backend == MppiComputeBackend::Cuda){
    if(!cuda_backend_){
      cuda_backend_ = makeMppiCudaBackend();
      cuda_backend_ready_ = false;
    }
    std::string reason;
    if(!cuda_backend_ || !cuda_backend_->available(&reason)){
      cuda_backend_unavailable_reason_ = reason.empty() ?
        "CUDA backend reported unavailable" : reason;
      result.backend_error =
        "CUDA backend unavailable: " + cuda_backend_unavailable_reason_;
      result.rejected_samples = config_.batch_size;
      result.optimal_sequence = control_sequence_;
      result.avoidance_stage = avoidance_stage_;
      result.reverse_recovery_active = reverse_recovery_active_;
      result.reverse_motion_active = reverse_motion_active_;
      result.reverse_distance_used = reverse_distance_used_;
      result.reverse_distance_remaining = reverseDistanceRemaining();
      return result;
    }
    cuda_backend_ready_ = true;
  }

  const double reverse_ceiling = reverseSpeedCeiling(runtime_speed_limit);

  const auto entry_control_sequence = control_sequence_;
  const MppiBypassSide entry_bypass_side = latched_bypass_side_;
  MppiBypassSide provisional_bypass_side = latched_bypass_side_;
  std::size_t provisional_infeasible_cycles = latched_side_infeasible_cycles_;
  std::size_t provisional_quality_advantage_cycles =
    latched_side_quality_advantage_cycles_;
  MppiBypassSide settled_bypass_side = provisional_bypass_side;
  std::size_t settled_infeasible_cycles = provisional_infeasible_cycles;
  std::size_t settled_quality_advantage_cycles = provisional_quality_advantage_cycles;
  MppiAvoidanceStage provisional_avoidance_stage = avoidance_stage_;
  MppiAvoidanceStage settled_avoidance_stage = provisional_avoidance_stage;
  bool side_state_settled = false;
  bool directional_mode_selected = false;
  bool select_directional_mode = false;
  MppiDirectionalContext directional_context;
  MppiBypassSide generation_bypass_side = entry_bypass_side;
  MppiAvoidanceStage generation_avoidance_stage = avoidance_stage_;
  bool reverse_attempted_this_cycle = false;
  bool reverse_selected = false;
  bool reverse_budget_exhausted_this_cycle = false;

  const auto set_reverse_diagnostics = [&]() {
      result.reverse_recovery_active = reverse_recovery_active_;
      result.reverse_motion_active = reverse_motion_active_;
      result.reverse_budget_exhausted =
        reverse_budget_exhausted_this_cycle ||
        (config_.reverse_recovery_enabled &&
        reverseDistanceRemaining() <= kReverseDistanceEpsilon);
      result.reverse_distance_used = reverse_distance_used_;
      result.reverse_distance_remaining = reverseDistanceRemaining();
    };

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
    std::vector<bool> valid_structured;
    std::vector<double> lateral_progress;
    result.rejected_samples = 0;
    result.best_cost = std::numeric_limits<double>::infinity();
    result.best_sample_index = 0;
    result.best_sample_sequence.clear();
    result.best_critic_cost = std::numeric_limits<double>::infinity();
    result.best_control_energy = 0.0;
    result.best_sample_side = MppiBypassSide::Neutral;
    result.feasible_left_candidates = 0;
    result.feasible_right_candidates = 0;
    result.feasible_neutral_candidates = 0;
    result.best_left_cost = std::numeric_limits<double>::infinity();
    result.best_right_cost = std::numeric_limits<double>::infinity();

    std::vector<SampledSequence> sampled_sequences;
    std::vector<MppiCandidate> candidates;
    const auto generate_candidates = [&]() -> bool {
        result.structured_candidates = use_debug_noise_for_test_ ? 0 :
          activeStructuredCandidateCount(generation_bypass_side);
        sampled_sequences.clear();
        candidates.clear();
        sampled_sequences.reserve(config_.batch_size);
        candidates.reserve(config_.batch_size);
        if(config_.compute_backend == MppiComputeBackend::Cuda){
          MppiGenerationRequest request;
          auto & params = request.params;
          params.batch_size = config_.batch_size;
          params.time_steps = config_.time_steps;
          params.structured_candidate_count = result.structured_candidates;
          params.model_dt = config_.model_dt;
          params.controller_period = config_.controller_period;
          params.gamma = config_.gamma;
          params.vx_std = config_.vx_std;
          params.wz_std = config_.wz_std;
          params.vx_min = config_.vx_min;
          params.vx_max = config_.vx_max;
          params.wz_max = config_.wz_max;
          params.max_steer = config_.max_steer;
          params.wheelbase = config_.wheelbase;
          params.ax_min = config_.ax_min;
          params.ax_max = config_.ax_max;
          params.az_max = config_.az_max;
          params.structured_min_vx = config_.structured_min_vx;
          params.shift_control_sequence = config_.shift_control_sequence;
          params.use_debug_noise = use_debug_noise_for_test_;
          params.bypass_side = generation_bypass_side;
          params.avoidance_stage = generation_avoidance_stage;
          params.seed = config_.seed;
          params.noise_generation = cuda_noise_generation_;
          params.use_device_noise = !use_debug_noise_for_test_;
          request.measured_speed = measured_speed;
          request.control_sequence = control_sequence_;
          if(use_debug_noise_for_test_){
            std::size_t noise_total = 0;
            if(!mppi_math::checkedMulSize(
                noise_.size(), config_.time_steps, noise_total))
            {
              result.backend_error = "CUDA debug noise batch size overflow";
              result.rejected_samples = config_.batch_size;
              result.has_valid_control = false;
              control_sequence_ = entry_control_sequence;
              result.optimal_sequence = control_sequence_;
              return false;
            }
            request.noise_batch_size = noise_.size();
            request.noise.reserve(noise_total);
            for(const auto & row : noise_){
              request.noise.insert(request.noise.end(), row.begin(), row.end());
            }
          }
          MppiCandidateBatch batch;
          std::string error;
          if(!cuda_backend_ || !cuda_backend_->generateCandidates(request, batch, error)){
            result.backend_error = error.empty() ?
              "CUDA candidate generation failed" : error;
            result.rejected_samples = config_.batch_size;
            result.has_valid_control = false;
            control_sequence_ = entry_control_sequence;
            result.optimal_sequence = control_sequence_;
            return false;
          }
          std::size_t expected_total = 0;
          if(!mppi_math::checkedMulSize(
              config_.batch_size, config_.time_steps, expected_total) ||
            batch.batch_size != config_.batch_size ||
            batch.time_steps != config_.time_steps ||
            batch.controls.size() != expected_total ||
            batch.control_energy.size() != config_.batch_size ||
            batch.structured.size() != config_.batch_size)
          {
            result.backend_error = "CUDA candidate generation returned inconsistent dimensions";
            result.rejected_samples = config_.batch_size;
            result.has_valid_control = false;
            control_sequence_ = entry_control_sequence;
            result.optimal_sequence = control_sequence_;
            return false;
          }
          for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
            SampledSequence sampled;
            sampled.controls.assign(
              batch.controls.begin() + sample * config_.time_steps,
              batch.controls.begin() + (sample + 1) * config_.time_steps);
            sampled.control_energy = batch.control_energy[sample];
            sampled.structured = batch.structured[sample] != 0u;
            sampled_sequences.push_back(std::move(sampled));
            candidates.push_back(MppiCandidate{sampled_sequences.back().controls});
          }
          return true;
        }
        for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
          sampled_sequences.push_back(buildNoisedSequence(
              measured_speed,
              sample,
              generation_bypass_side,
              generation_avoidance_stage));
          candidates.push_back(MppiCandidate{sampled_sequences.back().controls});
        }
        return true;
      };
    if(!generate_candidates()){
      return result;
    }
    auto scores = score_function(candidates);

    const auto valid_soft_score = [](const MppiCandidateScore & score) {
        return score.valid && std::isfinite(score.cost) && score.cost >= 0.0;
      };
    const auto usable_score = [&](const MppiCandidateScore & score) {
        return valid_soft_score(score) && score.collision_free;
      };
    const auto has_usable_score =
      [&](const std::vector<MppiCandidateScore> & batch_scores) {
        for(const auto & score : batch_scores){
          if(usable_score(score)){
            return true;
          }
        }
        return false;
      };

    // Finite soft costs on collision candidates are not a feasible forward
    // path. Only genuine forward infeasibility may start reverse recovery; a
    // directional/hysteresis selection failure with a usable forward batch is
    // not allowed to replace normal forward avoidance.
    const bool forward_batch_usable = has_usable_score(scores);
    const std::size_t confirm_command_index =
      config_.shift_control_sequence && config_.time_steps > 1 ? 1 : 0;

    if(reverse_motion_active_){
      bool forward_probe_confirmed = false;
      if(forward_batch_usable){
        for(std::size_t sample = 0; sample < sampled_sequences.size(); ++sample){
          const auto & score = sample < scores.size() ? scores[sample] :
            MppiCandidateScore{false, -1.0};
          const auto & controls = sampled_sequences[sample].controls;
          // A feasible forward plan may first brake a reverse-moving vehicle
          // to zero. Its first command need not already be positive.
          const bool plans_forward_motion = std::any_of(
            controls.begin() + confirm_command_index, controls.end(),
            [](const MppiControl & control) {return control.vx > 0.0;});
          if(usable_score(score) && plans_forward_motion)
          {
            forward_probe_confirmed = true;
            break;
          }
        }
      }
      if(forward_probe_confirmed){
        reverse_forward_exit_cycles_ = std::min(
          reverse_forward_exit_cycles_ + 1,
          kReverseForwardExitCycles);
      }
      else{
        reverse_forward_exit_cycles_ = 0;
      }

      // Reverse motion ends after three consecutive cycles that contain a
      // usable sample planning forward motion after braking. The persistent distance
      // episode is deliberately not cleared here: its already-spent budget
      // remains charged until measured forward travel repays it. This lets the
      // post-reverse forward probe move normally without refreshing reverse
      // budget on a stationary or single-cycle feasibility blip.
      if(reverse_forward_exit_cycles_ >= kReverseForwardExitCycles){
        reverse_motion_active_ = false;
        reverse_forward_exit_cycles_ = 0;
        reverse_control_sequence_.assign(config_.time_steps, MppiControl{});
        // Do not let reverse commands bleed through the forward Savitzky-Golay
        // filter on the first post-reverse probe.
        control_history_.fill(measured_speed);
      }
      else{
        reverse_attempted_this_cycle = true;
        if(canAttemptReverseRecovery() &&
          tryReverseFallback(measured_speed, score_function, reverse_ceiling, result))
        {
          reverse_selected = true;
          break;
        }
        if(!result.backend_error.empty()){
          control_sequence_ = entry_control_sequence;
          result.has_valid_control = false;
          result.optimal_sequence = control_sequence_;
          set_reverse_diagnostics();
          return result;
        }
        reverse_budget_exhausted_this_cycle =
          reverseDistanceRemaining() <= kReverseDistanceEpsilon;
        // Keep retreating even though this batch contains a usable forward
        // sample. The forward latch/stage and forward warm start stay intact
        // for the validated exit.
        control_sequence_ = entry_control_sequence;
        result.has_valid_control = false;
        result.optimal_sequence = control_sequence_;
        result.selected_bypass_side = latched_bypass_side_;
        result.side_infeasible_cycles = latched_side_infeasible_cycles_;
        result.side_quality_advantage_cycles =
          latched_side_quality_advantage_cycles_;
        result.avoidance_stage = avoidance_stage_;
        set_reverse_diagnostics();
        return result;
      }
    }

    if(!reverse_motion_active_ && !forward_batch_usable &&
      !reverse_attempted_this_cycle && config_.reverse_recovery_enabled)
    {
      reverse_attempted_this_cycle = true;
      if(canAttemptReverseRecovery()){
        if(tryReverseFallback(
            measured_speed, score_function, reverse_ceiling, result))
        {
          reverse_selected = true;
          break;
        }
        if(!result.backend_error.empty()){
          control_sequence_ = entry_control_sequence;
          result.has_valid_control = false;
          result.optimal_sequence = control_sequence_;
          set_reverse_diagnostics();
          return result;
        }
      }
      else{
        reverse_budget_exhausted_this_cycle =
          reverseDistanceRemaining() <= kReverseDistanceEpsilon;
      }
    }

    if(reverse_selected){
      break;
    }
    if(reverse_attempted_this_cycle && !forward_batch_usable){
      directional_mode_selected = true;
      select_directional_mode = false;
    }
    else if(!directional_mode_selected){
      directional_context = mode_selection_function ?
        mode_selection_function() : MppiDirectionalContext{};
      select_directional_mode = directional_context.path_blocked;
      directional_mode_selected = true;
      if(!select_directional_mode){
        avoidance_stage_ = MppiAvoidanceStage::Neutral;
        result.avoidance_stage = MppiAvoidanceStage::Neutral;
        if(generation_bypass_side != MppiBypassSide::Neutral){
          generation_bypass_side = MppiBypassSide::Neutral;
          generation_avoidance_stage = MppiAvoidanceStage::Neutral;
          if(!generate_candidates()){
            return result;
          }
          scores = score_function(candidates);
        }
      }
    }

    for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
      const auto & sampled_sequence = sampled_sequences[sample];
      const auto & sequence = sampled_sequence.controls;
      const auto score =
        sample < scores.size() ? scores[sample] : MppiCandidateScore{false, -1.0};
      if(!score.valid || !std::isfinite(score.cost) || score.cost < 0.0){
        ++result.rejected_samples;
        continue;
      }

      const double control_energy =
        sampled_sequence.structured ? 0.0 : sampled_sequence.control_energy;
      const double total_cost = score.cost + control_energy;
      if(!std::isfinite(total_cost)){
        ++result.rejected_samples;
        continue;
      }

      if(total_cost < result.best_cost){
        result.best_cost = total_cost;
        result.best_critic_cost = score.cost;
        result.best_control_energy = control_energy;
        result.best_sample_side = score.bypass_side;
        result.best_sample_index = sample;
        result.best_sample_sequence = sequence;
      }

      if(score.collision_free){
        switch(score.bypass_side){
          case MppiBypassSide::Left:
            ++result.feasible_left_candidates;
            result.best_left_cost = std::min(result.best_left_cost, total_cost);
            break;
          case MppiBypassSide::Right:
            ++result.feasible_right_candidates;
            result.best_right_cost = std::min(result.best_right_cost, total_cost);
            break;
          case MppiBypassSide::Neutral:
            ++result.feasible_neutral_candidates;
            break;
        }
      }

      valid_sequences.push_back(sequence);
      valid_costs.push_back(total_cost);
      valid_sides.push_back(score.bypass_side);
      collision_free.push_back(score.collision_free);
      valid_structured.push_back(sampled_sequence.structured);
      lateral_progress.push_back(score.signed_lateral_progress);
    }

    const MppiBypassSide side_before_update = provisional_bypass_side;
    const std::size_t infeasible_cycles_before_update = provisional_infeasible_cycles;
    const std::size_t quality_cycles_before_update =
      provisional_quality_advantage_cycles;
    const bool side_state_was_settled = side_state_settled;
    std::string update_backend_error;
    const auto statistics = applySoftmaxUpdate(
      valid_sequences,
      valid_costs,
      valid_sides,
      collision_free,
      valid_structured,
      lateral_progress,
      select_directional_mode,
      directional_context,
      provisional_bypass_side,
      provisional_infeasible_cycles,
      provisional_quality_advantage_cycles,
      provisional_avoidance_stage,
      iteration == 0,
      update_backend_error);
    if(side_state_was_settled){
      provisional_bypass_side = settled_bypass_side;
      provisional_infeasible_cycles = settled_infeasible_cycles;
      provisional_quality_advantage_cycles = settled_quality_advantage_cycles;
      provisional_avoidance_stage = settled_avoidance_stage;
    }
    else if(provisional_bypass_side != side_before_update ||
      provisional_infeasible_cycles != infeasible_cycles_before_update ||
      provisional_quality_advantage_cycles != quality_cycles_before_update)
    {
      side_state_settled = true;
      settled_bypass_side = provisional_bypass_side;
      settled_infeasible_cycles = provisional_infeasible_cycles;
      settled_quality_advantage_cycles = provisional_quality_advantage_cycles;
      settled_avoidance_stage = provisional_avoidance_stage;
    }
    result.effective_sample_size = statistics.effective_sample_size;
    result.positive_weight_mass = statistics.positive_weight_mass;
    result.negative_weight_mass = statistics.negative_weight_mass;
    result.neutral_weight_mass = statistics.neutral_weight_mass;
    result.structured_weight_mass = statistics.structured_weight_mass;
    result.prefilter_command = statistics.prefilter_command;
    result.phase_compatible_candidates = statistics.phase_compatible_candidates;
    result.bypass_guard_applied = statistics.bypass_guard_applied;
    result.bypass_guard_removed = statistics.bypass_guard_removed;
    if(!update_backend_error.empty()){
      result.backend_error = update_backend_error;
      result.has_valid_control = false;
      control_sequence_ = entry_control_sequence;
      result.optimal_sequence = control_sequence_;
      result.selected_bypass_side = provisional_bypass_side;
      result.side_infeasible_cycles = provisional_infeasible_cycles;
      result.side_quality_advantage_cycles = provisional_quality_advantage_cycles;
      result.avoidance_stage = provisional_avoidance_stage;
      set_reverse_diagnostics();
      return result;
    }
    result.selected_bypass_side = provisional_bypass_side;
    result.side_infeasible_cycles = provisional_infeasible_cycles;
    result.side_quality_advantage_cycles = provisional_quality_advantage_cycles;
    result.avoidance_stage = provisional_avoidance_stage;
    if(!statistics.valid){
      if(!reverse_attempted_this_cycle && !forward_batch_usable &&
        config_.reverse_recovery_enabled)
      {
        reverse_attempted_this_cycle = true;
        if(canAttemptReverseRecovery()){
          if(tryReverseFallback(
              measured_speed, score_function, reverse_ceiling, result))
          {
            reverse_selected = true;
            break;
          }
          if(!result.backend_error.empty()){
            control_sequence_ = entry_control_sequence;
            result.has_valid_control = false;
            result.optimal_sequence = control_sequence_;
            set_reverse_diagnostics();
            return result;
          }
        }
        else{
          reverse_budget_exhausted_this_cycle =
            reverseDistanceRemaining() <= kReverseDistanceEpsilon;
        }
      }
      control_sequence_ = entry_control_sequence;
      latched_bypass_side_ = provisional_bypass_side;
      latched_side_infeasible_cycles_ = provisional_infeasible_cycles;
      latched_side_quality_advantage_cycles_ = provisional_quality_advantage_cycles;
      avoidance_stage_ = provisional_avoidance_stage;
      result.has_valid_control = false;
      result.optimal_sequence = control_sequence_;
      result.avoidance_stage = avoidance_stage_;
      set_reverse_diagnostics();
      return result;
    }

    // Later iterations must generate templates for the side/stage that this
    // iteration settled on. In particular, after an iteration-0 side switch
    // the next batch cannot keep generating the old side, which would leave
    // the newly selected side without any candidates to aggregate.
    generation_bypass_side = provisional_bypass_side;
    generation_avoidance_stage = provisional_avoidance_stage;
  }

  if(reverse_selected){
    // Keep the forward bypass latch/stage and the forward control sequence.
    // Reverse is a temporary motion state, not a replacement avoidance plan.
    result.selected_bypass_side = latched_bypass_side_;
    result.side_infeasible_cycles = latched_side_infeasible_cycles_;
    result.side_quality_advantage_cycles = latched_side_quality_advantage_cycles_;
    result.avoidance_stage = avoidance_stage_;
    result.has_valid_control = true;
    // Keep the exact scored candidate. Filtering a reverse arc can change its
    // collision geometry even though its first command remains unchanged.
    result.optimal_sequence = reverse_control_sequence_;
    const std::size_t reverse_command_index =
      config_.shift_control_sequence && reverse_control_sequence_.size() > 1 ? 1 : 0;
    result.command = reverse_control_sequence_[reverse_command_index];
    result.prefilter_command = result.command;
    // Shift only the reverse seed. The forward seed and bypass commitment stay
    // untouched for the post-reverse forward probe.
    if(reverse_control_sequence_.size() > 1 && config_.shift_control_sequence){
      for(std::size_t i = 0; i + 1 < reverse_control_sequence_.size(); ++i){
        reverse_control_sequence_[i] = reverse_control_sequence_[i + 1];
      }
      reverse_control_sequence_.back() =
        reverse_control_sequence_[reverse_control_sequence_.size() - 2];
    }
    set_reverse_diagnostics();
    return result;
  }

  latched_bypass_side_ = provisional_bypass_side;
  latched_side_infeasible_cycles_ = provisional_infeasible_cycles;
  latched_side_quality_advantage_cycles_ = provisional_quality_advantage_cycles;
  avoidance_stage_ = provisional_avoidance_stage;
  result.has_valid_control = true;
  smoothControlSequence(measured_speed);
  result.optimal_sequence = control_sequence_;
  const std::size_t command_index =
    config_.shift_control_sequence && control_sequence_.size() > 1 ? 1 : 0;
  result.command = control_sequence_[command_index];
  updateControlHistory(result.command);
  shiftControlSequence();
  result.avoidance_stage = avoidance_stage_;
  set_reverse_diagnostics();
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
  config_.bypass_detour_weight = std::max(0.0, config_.bypass_detour_weight);
  config_.wheelbase = std::max(1e-6, config_.wheelbase);
  config_.max_steer = std::max(1e-6, std::fabs(config_.max_steer));
  config_.vx_max = std::max(config_.vx_min, config_.vx_max);
  config_.reverse_max_speed = std::max(0.0, config_.reverse_max_speed);
  config_.reverse_episode_distance = std::max(
    0.0, config_.reverse_episode_distance);
  config_.reverse_structured_candidate_count = std::min(
    config_.reverse_structured_candidate_count,
    config_.batch_size);
  config_.reverse_structured_candidate_count -=
    config_.reverse_structured_candidate_count % 2;
  config_.structured_min_vx = clamp(
    config_.structured_min_vx, config_.vx_min, config_.vx_max);
  config_.departure_lateral_offset = std::max(0.0, config_.departure_lateral_offset);
  config_.wz_max = std::max(0.0, config_.wz_max);
  config_.ax_max = std::fabs(config_.ax_max);
  config_.ax_min = config_.ax_min > 0.0 ? -config_.ax_min : config_.ax_min;
  config_.az_max = std::fabs(config_.az_max);
  config_.vx_std = std::max(0.0, config_.vx_std);
  config_.wz_std = std::max(0.0, config_.wz_std);
}

void MppiController::regenerateNoise()
{
  if(config_.compute_backend == MppiComputeBackend::Cuda &&
    !use_debug_noise_for_test_)
  {
    // The CUDA normal path samples on device from (seed, generation). Only the
    // counter advances here; no host batch is generated or uploaded.
    ++cuda_noise_generation_;
    noise_.clear();
    return;
  }
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

MppiAvoidanceStage MppiController::advanceAvoidanceStage(
  const MppiAvoidanceStage current,
  const MppiBypassSide latched_side,
  const MppiDirectionalContext & context) const
{
  if(!context.path_blocked || latched_side == MppiBypassSide::Neutral){
    return MppiAvoidanceStage::Neutral;
  }
  if(current == MppiAvoidanceStage::Bypass){
    // A committed bypass is monotonic while the path remains blocked. The
    // measured lateral offset naturally levels out during the maneuver, and
    // demoting the stage to Depart would apply an outward-progress soft cost
    // that makes the planner prefer slow near-straight candidates again until
    // the obstacle is too close to pass. Only the !path_blocked branch above
    // (or losing the latched side) releases the maneuver.
    return MppiAvoidanceStage::Bypass;
  }
  const double side = static_cast<double>(latched_side);
  if(context.offset_valid &&
    side * context.signed_path_offset >= config_.departure_lateral_offset)
  {
    return MppiAvoidanceStage::Bypass;
  }
  return MppiAvoidanceStage::Depart;
}

std::size_t MppiController::activeStructuredCandidateCount(
  const MppiBypassSide bypass_side) const
{
  return bypass_side == MppiBypassSide::Neutral ?
    config_.structured_candidate_count :
    std::min<std::size_t>(4, config_.structured_candidate_count);
}

MppiControl MppiController::constrainControl(
  const MppiControl & previous,
  const MppiControl & requested,
  const double dt) const
{
  return constrainControl(
    previous, requested, dt, config_.vx_min, config_.vx_max);
}

MppiControl MppiController::constrainControl(
  const MppiControl & previous,
  const MppiControl & requested,
  const double dt,
  const double vx_low,
  const double vx_high) const
{
  const double low = std::min(vx_low, vx_high);
  const double high = std::max(vx_low, vx_high);
  const double reachable_low = previous.vx + config_.ax_min * dt;
  const double reachable_high = previous.vx + config_.ax_max * dt;
  MppiControl constrained;
  constrained.vx = clamp(requested.vx, reachable_low, reachable_high);

  // When a new cap moves inside the current measured speed, saturated clipping
  // to the cap would command an acceleration the vehicle cannot achieve.
  // Keep the reachable braking envelope until the state enters the band.
  double bounded_low = low;
  double bounded_high = high;
  if(previous.vx < low){
    bounded_low = std::min(low, reachable_high);
  }
  else if(previous.vx > high){
    bounded_high = std::max(high, reachable_low);
  }
  constrained.vx = clamp(
    constrained.vx, std::min(bounded_low, bounded_high),
    std::max(bounded_low, bounded_high));

  const double requested_wz = clamp(
    requested.wz,
    previous.wz - config_.az_max * dt,
    previous.wz + config_.az_max * dt);
  // ackermannWzLimit() already uses |vx|, so it enforces the correct
  // curvature relation for reverse motion and collapses to zero at rest.
  const double wz_limit = ackermannWzLimit(constrained.vx);
  constrained.wz = clamp(requested_wz, -wz_limit, wz_limit);
  return constrained;
}

bool MppiController::canAttemptReverseRecovery() const
{
  return config_.reverse_recovery_enabled &&
         config_.ax_max > 0.0 &&
         config_.reverse_max_speed > kReverseDistanceEpsilon;
}

double MppiController::effectiveReverseSpeedLimit(
  const double runtime_speed_limit) const
{
  double limit = std::max(0.0, config_.reverse_max_speed);
  if(runtime_speed_limit > 0.0){
    limit = std::min(limit, runtime_speed_limit);
  }
  return limit;
}

double MppiController::reverseSpeedCeiling(
  const double runtime_speed_limit) const
{
  return config_.ax_max > 0.0 ? effectiveReverseSpeedLimit(runtime_speed_limit) : 0.0;
}

std::size_t MppiController::activeReverseStructuredCandidateCount() const
{
  if(!canAttemptReverseRecovery()){
    return 0;
  }
  return std::min(
    config_.reverse_structured_candidate_count,
    config_.batch_size);
}

bool MppiController::tryReverseFallback(
  const MppiControl & measured_speed,
  const BatchScoreFunction & score_function,
  const double reverse_speed_ceiling,
  MppiResult & result)
{
  if(recovery_function_){
    result = recovery_function_(false);
    reverse_recovery_active_ = result.reverse_recovery_active;
    reverse_motion_active_ = result.reverse_motion_active;
    reverse_control_sequence_ = result.optimal_sequence;
    result.reverse_distance_used = reverse_distance_used_;
    result.reverse_distance_remaining = reverseDistanceRemaining();
    return result.has_valid_control;
  }
  if(!canAttemptReverseRecovery() || !score_function){
    return false;
  }

  result.structured_candidates = use_debug_noise_for_test_ ? 0 :
    activeReverseStructuredCandidateCount();
  std::vector<SampledSequence> sampled_sequences;
  std::vector<MppiCandidate> candidates;
  sampled_sequences.reserve(config_.batch_size);
  candidates.reserve(config_.batch_size);
  if(config_.compute_backend == MppiComputeBackend::Cuda){
    MppiGenerationRequest request;
    auto & params = request.params;
    params.batch_size = config_.batch_size;
    params.time_steps = config_.time_steps;
    params.structured_candidate_count = result.structured_candidates;
    params.model_dt = config_.model_dt;
    params.controller_period = config_.controller_period;
    params.gamma = config_.gamma;
    params.vx_std = config_.vx_std;
    params.wz_std = config_.wz_std;
    params.vx_min = config_.vx_min;
    params.vx_max = config_.vx_max;
    params.wz_max = config_.wz_max;
    params.max_steer = config_.max_steer;
    params.wheelbase = config_.wheelbase;
    params.ax_min = config_.ax_min;
    params.ax_max = config_.ax_max;
    params.az_max = config_.az_max;
    params.structured_min_vx = config_.structured_min_vx;
    params.shift_control_sequence = config_.shift_control_sequence;
    params.use_debug_noise = use_debug_noise_for_test_;
    params.use_device_noise = !use_debug_noise_for_test_;
    params.seed = config_.seed;
    params.noise_generation = cuda_noise_generation_;
    params.reverse_mode = true;
    params.reverse_speed_ceiling = reverse_speed_ceiling;
    params.bypass_side = MppiBypassSide::Neutral;
    params.avoidance_stage = MppiAvoidanceStage::Neutral;
    request.measured_speed = measured_speed;
    request.control_sequence = control_sequence_;
    if(use_debug_noise_for_test_){
      std::size_t noise_total = 0;
      if(!mppi_math::checkedMulSize(
          noise_.size(), config_.time_steps, noise_total))
      {
        result.backend_error = "CUDA reverse debug noise batch size overflow";
        result.rejected_samples = config_.batch_size;
        result.has_valid_control = false;
        return false;
      }
      request.noise_batch_size = noise_.size();
      request.noise.reserve(noise_total);
      for(const auto & row : noise_){
        request.noise.insert(request.noise.end(), row.begin(), row.end());
      }
    }
    MppiCandidateBatch batch;
    std::string error;
    if(!cuda_backend_ || !cuda_backend_->generateCandidates(request, batch, error)){
      result.backend_error = error.empty() ?
        "CUDA reverse candidate generation failed" : error;
      result.rejected_samples = config_.batch_size;
      result.has_valid_control = false;
      return false;
    }
    std::size_t expected_total = 0;
    if(!mppi_math::checkedMulSize(
        config_.batch_size, config_.time_steps, expected_total) ||
      batch.batch_size != config_.batch_size ||
      batch.time_steps != config_.time_steps ||
      batch.controls.size() != expected_total ||
      batch.control_energy.size() != config_.batch_size ||
      batch.structured.size() != config_.batch_size)
    {
      result.backend_error =
        "CUDA reverse candidate generation returned inconsistent dimensions";
      result.rejected_samples = config_.batch_size;
      result.has_valid_control = false;
      return false;
    }
    for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
      SampledSequence sampled;
      sampled.controls.assign(
        batch.controls.begin() + sample * config_.time_steps,
        batch.controls.begin() + (sample + 1) * config_.time_steps);
      sampled.control_energy = batch.control_energy[sample];
      sampled.structured = batch.structured[sample] != 0u;
      sampled_sequences.push_back(std::move(sampled));
      candidates.push_back(MppiCandidate{sampled_sequences.back().controls});
    }
  }
  else{
    for(std::size_t sample = 0; sample < config_.batch_size; ++sample){
      sampled_sequences.push_back(buildReverseSequence(
          measured_speed, sample, reverse_speed_ceiling));
      candidates.push_back(MppiCandidate{sampled_sequences.back().controls});
    }
  }

  result.reverse_candidates_generated = true;
  const auto scores = score_function(candidates);
  std::size_t best_sample = candidates.size();
  std::size_t best_structured_sample = candidates.size();
  double best_total_cost = std::numeric_limits<double>::infinity();
  double best_structured_cost = std::numeric_limits<double>::infinity();
  std::size_t rejected = 0;
  for(std::size_t sample = 0; sample < candidates.size(); ++sample){
    const auto score = sample < scores.size() ? scores[sample] :
      MppiCandidateScore{false, -1.0};
    if(!score.valid || !std::isfinite(score.cost) || score.cost < 0.0 ||
      !score.collision_free)
    {
      ++rejected;
      continue;
    }
    const double control_energy = sampled_sequences[sample].structured ?
      0.0 : sampled_sequences[sample].control_energy;
    const double total_cost = score.cost + control_energy;
    if(!std::isfinite(total_cost)){
      ++rejected;
      continue;
    }
    if(total_cost < best_total_cost){
      best_total_cost = total_cost;
      best_sample = sample;
    }
    if(sampled_sequences[sample].structured &&
      total_cost < best_structured_cost)
    {
      best_structured_cost = total_cost;
      best_structured_sample = sample;
    }
  }
  result.rejected_samples = rejected;
  // Reverse retreat must be a coherent, deterministic motion. When validated
  // structured templates exist, never let a cheaper random/hybrid sample (or
  // a near-zero noise sample) win solely on a path/energy cost.
  if(best_structured_sample < candidates.size()){
    best_sample = best_structured_sample;
    best_total_cost = best_structured_cost;
  }
  if(best_sample >= candidates.size()){
    return false;
  }

  // A reverse fallback is directional: selecting one fully validated arc is
  // safer than a softmax mixture of mirrored arcs, which can average a blocked
  // straight command even when both curved candidates are collision-free.
  const auto & best_score = scores[best_sample];
  const bool starting_new_episode = !reverse_recovery_active_;
  reverse_recovery_active_ = true;
  reverse_motion_active_ = true;
  if(starting_new_episode){
    reverse_forward_exit_cycles_ = 0;
  }
  reverse_control_sequence_ = sampled_sequences[best_sample].controls;
  const std::size_t command_index =
    config_.shift_control_sequence && reverse_control_sequence_.size() > 1 ? 1 : 0;
  result.best_cost = best_total_cost;
  result.best_critic_cost = best_score.cost;
  result.best_control_energy = sampled_sequences[best_sample].structured ?
    0.0 : sampled_sequences[best_sample].control_energy;
  result.best_sample_index = best_sample;
  result.best_sample_side = best_score.bypass_side;
  result.best_sample_sequence = reverse_control_sequence_;
  result.effective_sample_size = 1.0;
  result.structured_weight_mass = sampled_sequences[best_sample].structured ? 1.0 : 0.0;
  result.positive_weight_mass = 0.0;
  result.negative_weight_mass = 0.0;
  result.neutral_weight_mass = 1.0;
  result.phase_compatible_candidates = 0;
  result.prefilter_command = reverse_control_sequence_[command_index];
  if(best_score.bypass_side == MppiBypassSide::Neutral){
    ++result.feasible_neutral_candidates;
  }
  else if(best_score.bypass_side == MppiBypassSide::Left){
    ++result.feasible_left_candidates;
  }
  else{
    ++result.feasible_right_candidates;
  }
  return true;
}

MppiController::SampledSequence MppiController::buildNoisedSequence(
  const MppiControl & measured_speed,
  const std::size_t sample_index,
  const MppiBypassSide bypass_side,
  const MppiAvoidanceStage avoidance_stage) const
{
  if(!use_debug_noise_for_test_ &&
    sample_index < activeStructuredCandidateCount(bypass_side))
  {
    return buildStructuredSequence(
      measured_speed, sample_index, bypass_side, avoidance_stage);
  }

  SampledSequence sampled_sequence;
  auto & sequence = sampled_sequence.controls;
  sequence.resize(config_.time_steps);
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
    if(config_.vx_std > 1e-9){
      sampled_sequence.control_energy +=
        config_.gamma / (config_.vx_std * config_.vx_std) *
        control_sequence_[step].vx * noise.vx;
    }
    if(config_.wz_std > 1e-9 && angular_noise_scale > 1e-9){
      // Conditioned on the vx-constrained sample, angular noise has standard
      // deviation wz_std * angular_noise_scale.
      const double effective_wz_std = config_.wz_std * angular_noise_scale;
      const double effective_wz_noise = noise.wz * angular_noise_scale;
      sampled_sequence.control_energy +=
        config_.gamma / (effective_wz_std * effective_wz_std) *
        control_sequence_[step].wz * effective_wz_noise;
    }
    sequence[step] = constrainControl(previous, requested, dt);
    previous = sequence[step];
  }
  return sampled_sequence;
}

MppiController::SampledSequence MppiController::buildStructuredSequence(
  const MppiControl & measured_speed,
  const std::size_t sample_index,
  const MppiBypassSide bypass_side,
  const MppiAvoidanceStage avoidance_stage) const
{
  SampledSequence sampled_sequence;
  sampled_sequence.structured = true;
  auto & sequence = sampled_sequence.controls;
  sequence.resize(config_.time_steps);
  MppiControl previous = measured_speed;
  std::size_t first_control_step = 0;
  if(config_.shift_control_sequence && !sequence.empty()){
    sequence.front() = measured_speed;
    first_control_step = 1;
  }

  const bool has_latched_side = bypass_side != MppiBypassSide::Neutral;
  struct StructuredProfile
  {
    double speed_scale;
    double curvature_fraction;
    bool s_curve;
  };
  // Explicit profile map instead of reusing sample_index/2 for speed. Neutral
  // uses 4 profiles x 2 mirrored directions; a latched side uses the same 4
  // profiles on that side only (at most 4 samples). Keep the practical
  // .55/.90 curvature fractions, add a 100% curvature profile, and retain
  // 1.0/.75/1.25/1.5 nominal speed coverage. Profile 2 remains an S-curve;
  // profile 3 (1.25x, 100%) is a full outward arc so a from-rest departure
  // can reach departure_lateral_offset within one horizon.
  static constexpr std::array<StructuredProfile, 4> profiles{{
    {1.00, 0.55, false},
    {0.75, 0.90, false},
    {1.50, 0.90, true},
    {1.25, 1.00, false},
  }};
  const std::size_t profile_index = has_latched_side ?
    sample_index : sample_index / 2;
  const auto & profile = profiles[profile_index % profiles.size()];
  const double direction = has_latched_side ?
    static_cast<double>(bypass_side) :
    (sample_index % 2 == 0 ? 1.0 : -1.0);
  const double intensity = profile.curvature_fraction;
  const bool s_curve = !has_latched_side && profile.s_curve;
  const double target_speed = std::min(
    config_.vx_max,
    config_.structured_min_vx * profile.speed_scale);

  for(std::size_t step = first_control_step; step < config_.time_steps; ++step){
    const double dt = step == 0 ? config_.controller_period : config_.model_dt;
    const double requested_vx = std::max(
      control_sequence_[step].vx, target_speed);
    const MppiControl vx_only = constrainControl(
      previous,
      MppiControl{requested_vx, previous.wz},
      dt);
    double phase_direction = direction;
    if(has_latched_side){
      if(avoidance_stage == MppiAvoidanceStage::Bypass){
        switch(sample_index % 4){
          case 0:
            phase_direction = direction;
            break;
          case 1:
            phase_direction = -direction;
            break;
          case 2:
            phase_direction = 0.0;
            break;
          default:
            phase_direction = direction;
            break;
        }
      }
    }
    else if(s_curve &&
      step - first_control_step >=
      (config_.time_steps - first_control_step) / 2)
    {
      phase_direction = -direction;
    }
    const MppiControl requested{
      requested_vx,
      phase_direction * intensity * ackermannWzLimit(vx_only.vx)};
    sequence[step] = constrainControl(previous, requested, dt);
    previous = sequence[step];
  }
  return sampled_sequence;
}

MppiController::SampledSequence MppiController::buildReverseSequence(
  const MppiControl & measured_speed,
  const std::size_t sample_index,
  const double speed_limit) const
{
  SampledSequence sampled_sequence;
  sampled_sequence.structured = false;
  auto & sequence = sampled_sequence.controls;
  sequence.resize(config_.time_steps);
  MppiControl previous = measured_speed;
  std::size_t first_control_step = 0;
  if(config_.shift_control_sequence && !sequence.empty()){
    sequence.front() = measured_speed;
    first_control_step = 1;
  }

  struct ReverseProfile
  {
    double speed_scale;
    double curvature_fraction;
    double direction;
  };
  // Deterministic mirrored templates: straight, gentle arcs, and full-lock
  // arcs in both directions. Every profile uses the same negative speed
  // ceiling so aggregation cannot cancel opposite linear directions.
  static constexpr std::array<ReverseProfile, 8> profiles{{
    {1.00, 0.00, 0.0},
    {0.60, 0.00, 0.0},
    {1.00, 0.55, 1.0},
    {1.00, 0.55, -1.0},
    {1.00, 0.90, 1.0},
    {1.00, 0.90, -1.0},
    {0.75, 1.00, 1.0},
    {0.75, 1.00, -1.0},
  }};
  const bool structured =
    !use_debug_noise_for_test_ &&
    sample_index < activeReverseStructuredCandidateCount() &&
    speed_limit > kReverseDistanceEpsilon;
  const double ceiling = std::max(0.0, speed_limit);

  if(structured){
    sampled_sequence.structured = true;
    const auto & profile = profiles[sample_index % profiles.size()];
    const double target_speed = -ceiling * profile.speed_scale;
    for(std::size_t step = first_control_step; step < config_.time_steps; ++step){
      const double dt = step == 0 ? config_.controller_period : config_.model_dt;
      // A braking reverse trajectory may still have positive vx while it
      // crosses zero. Allow only the measured forward speed to decay; never
      // accelerate in the forward direction.
      const double high = std::max(0.0, previous.vx);
      const MppiControl vx_only = constrainControl(
        previous,
        MppiControl{target_speed, previous.wz},
        dt,
        -ceiling,
        high);
      double requested_wz = 0.0;
      if(std::fabs(profile.curvature_fraction) > 1e-9 &&
         vx_only.vx < -kReverseDistanceEpsilon)
      {
        requested_wz = profile.direction * profile.curvature_fraction *
          ackermannWzLimit(vx_only.vx);
      }
      sequence[step] = constrainControl(
        previous,
        MppiControl{target_speed, requested_wz},
        dt,
        -ceiling,
        high);
      previous = sequence[step];
    }
    return sampled_sequence;
  }

  const double angular_noise_scale = config_.wz_std > 1e-9 ?
    std::min(1.0, ackermannWzLimit(ceiling) / config_.wz_std) : 0.0;
  for(std::size_t step = first_control_step; step < config_.time_steps; ++step){
    const double dt = step == 0 ? config_.controller_period : config_.model_dt;
    const auto noise = debugNoiseAt(sample_index, step);
    const double high = std::max(0.0, previous.vx);
    const double requested_vx = clamp(
      -ceiling + noise.vx,
      -ceiling,
      high);
    const double requested_wz = previous.wz +
      noise.wz * angular_noise_scale;
    if(config_.vx_std > 1e-9){
      sampled_sequence.control_energy +=
        config_.gamma / (config_.vx_std * config_.vx_std) *
        (-ceiling) * noise.vx;
    }
    if(config_.wz_std > 1e-9 && angular_noise_scale > 1e-9){
      const double effective_wz_std = config_.wz_std * angular_noise_scale;
      const double effective_wz_noise = noise.wz * angular_noise_scale;
      sampled_sequence.control_energy +=
        config_.gamma / (effective_wz_std * effective_wz_std) *
        previous.wz * effective_wz_noise;
    }
    sequence[step] = constrainControl(
      previous,
      MppiControl{requested_vx, requested_wz},
      dt,
      -ceiling,
      high);
    previous = sequence[step];
  }
  return sampled_sequence;
}

MppiController::UpdateStatistics MppiController::applySoftmaxUpdate(
  const std::vector<std::vector<MppiControl>> & valid_sequences,
  const std::vector<double> & valid_costs,
  const std::vector<MppiBypassSide> & valid_sides,
  const std::vector<bool> & collision_free,
  const std::vector<bool> & valid_structured,
  const std::vector<double> & lateral_progress,
  const bool select_directional_mode,
  const MppiDirectionalContext & directional_context,
  MppiBypassSide & bypass_side,
  std::size_t & side_infeasible_cycles,
  std::size_t & side_quality_advantage_cycles,
  MppiAvoidanceStage & avoidance_stage,
  const bool allow_side_transition,
  std::string & backend_error)
{
  UpdateStatistics statistics;
  if(!select_directional_mode){
    bypass_side = MppiBypassSide::Neutral;
    side_infeasible_cycles = 0;
    side_quality_advantage_cycles = 0;
    geometry_switch_confirm_cycles_ = 0;
    avoidance_stage = MppiAvoidanceStage::Neutral;
  }

  if(valid_sequences.empty() || valid_sequences.size() != valid_costs.size() ||
    valid_sequences.size() != valid_sides.size() ||
    valid_sequences.size() != collision_free.size() ||
    valid_sequences.size() != valid_structured.size() ||
    valid_sequences.size() != lateral_progress.size())
  {
    if(select_directional_mode && bypass_side != MppiBypassSide::Neutral){
      side_quality_advantage_cycles = 0;
      if(allow_side_transition){
        side_infeasible_cycles = std::min(
          side_infeasible_cycles + 1,
          kSideSwitchConfirmCycles);
      }
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
  std::vector<double> selection_costs = valid_costs;
  if(select_directional_mode){
    const MppiBypassSide side_before_selection = bypass_side;
    const auto has_feasible_side = [&](const MppiBypassSide side) {
        for(std::size_t sample = 0; sample < valid_sides.size(); ++sample){
          if(collision_free[sample] && valid_sides[sample] == side){
            return true;
          }
        }
        return false;
      };

    double best_left_cost = std::numeric_limits<double>::infinity();
    double best_right_cost = std::numeric_limits<double>::infinity();
    for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
      if(!collision_free[sample]){
        continue;
      }
      if(valid_sides[sample] == MppiBypassSide::Left){
        best_left_cost = std::min(best_left_cost, valid_costs[sample]);
      }
      else if(valid_sides[sample] == MppiBypassSide::Right){
        best_right_cost = std::min(best_right_cost, valid_costs[sample]);
      }
    }

    // Distance from the current vehicle offset to each side's usable band,
    // priced in cost units. Zero unless the profile reported both bands and
    // the caller marked the vehicle offset valid.
    const auto detour_cost = [&](const MppiBypassSide side) {
        if(!directional_context.detour_valid ||
          !std::isfinite(directional_context.signed_path_offset))
        {
          return 0.0;
        }
        const double detour = side == MppiBypassSide::Left ?
          directional_context.left_detour :
          (side == MppiBypassSide::Right ?
          directional_context.right_detour : 0.0);
        return std::isfinite(detour) ?
          config_.bypass_detour_weight * std::max(0.0, detour) : 0.0;
      };

    if(bypass_side == MppiBypassSide::Neutral){
      const MppiBypassSide preferred_side = directional_context.preferred_side;
      if(preferred_side != MppiBypassSide::Neutral){
        // The blocking-obstacle geometry already fixed which side of the route
        // is legal. Never let a marginal cost difference choose the obstacle's
        // own side. When the required side has no collision-free candidate,
        // stay neutral and let the normal stop/reverse recovery handle it
        // instead of gambling on the other side.
        if(has_feasible_side(preferred_side)){
          bypass_side = preferred_side;
        }
        else{
          side_quality_advantage_cycles = 0;
          return statistics;
        }
      }
      // Initial selection compares one best safe sample per physical side, so
      // duplicating samples cannot vote a side in. When geometry has no
      // preference, the cheaper side also has to be worth reaching: the detour
      // cost keeps a manoeuvre on the side the vehicle already occupies.
      else{
        const double left_cost =
          best_left_cost + detour_cost(MppiBypassSide::Left);
        const double right_cost =
          best_right_cost + detour_cost(MppiBypassSide::Right);
        if(left_cost < right_cost){
          bypass_side = MppiBypassSide::Left;
        }
        else if(right_cost < left_cost){
          bypass_side = MppiBypassSide::Right;
        }
        else if(std::isfinite(left_cost)){
          bypass_side = MppiBypassSide::Left;
        }
      }
      if(bypass_side == MppiBypassSide::Neutral){
        side_quality_advantage_cycles = 0;
        return statistics;
      }
      // A fresh commitment starts a new geometric transition window.
      geometry_switch_confirm_cycles_ = 0;
    }

    const MppiBypassSide opposite_side =
      bypass_side == MppiBypassSide::Left ?
      MppiBypassSide::Right : MppiBypassSide::Left;
    const bool current_side_safe = has_feasible_side(bypass_side);
    const bool opposite_side_safe = has_feasible_side(opposite_side);
    const double current_best_cost = bypass_side == MppiBypassSide::Left ?
      best_left_cost : best_right_cost;
    const double opposite_best_cost = bypass_side == MppiBypassSide::Left ?
      best_right_cost : best_left_cost;
    // A cost-driven switch must also pay for the lateral travel to reach the
    // other band. Geometry-forced sides already decided the direction, so the
    // bias only applies while the geometry is neutral.
    const bool detour_applies =
      directional_context.detour_valid &&
      directional_context.preferred_side == MppiBypassSide::Neutral;
    const double current_effective_cost = current_best_cost +
      (detour_applies ? detour_cost(bypass_side) : 0.0);
    const double opposite_effective_cost = opposite_best_cost +
      (detour_applies ? detour_cost(opposite_side) : 0.0);

    if(current_side_safe){
      side_infeasible_cycles = 0;
      // The geometry already requires this side; a cheaper opposite arc is not
      // a reason to pass the obstacle on its own side.
      const bool geometry_prefers_current_side =
        directional_context.preferred_side != MppiBypassSide::Neutral &&
        directional_context.preferred_side == bypass_side;
      // A committed side is not permanent. When the blocking-obstacle geometry
      // puts the obstacle on the side we committed to, every collision-free
      // candidate on that side is avoiding an obstacle that is not in its way,
      // so the cost ranking between the two sides carries no information. In
      // that case the geometry alone decides, but only after the preference has
      // persisted far longer than a cost or infeasibility confirmation: the
      // profile is re-measured every cycle and its sign flips within a few
      // cycles when the obstacle straddles the route. Chasing those flips
      // crosses the route repeatedly and produces a much longer detour.
      const bool geometry_demands_opposite_side =
        directional_context.preferred_side != MppiBypassSide::Neutral &&
        directional_context.preferred_side == opposite_side &&
        opposite_side_safe;
      if(allow_side_transition){
        if(geometry_prefers_current_side){
          // The geometry now confirms the committed side: a pending transition
          // is contradicted and its evidence is discarded.
          geometry_switch_confirm_cycles_ = 0;
        }
        else{
          if(geometry_demands_opposite_side){
            geometry_switch_confirm_cycles_ = std::min(
              geometry_switch_confirm_cycles_ + 1,
              kGeometrySwitchConfirmCycles);
          }
          else if(directional_context.preferred_side != MppiBypassSide::Neutral){
            // The opposite side is preferred but has no collision-free
            // candidate: the pending transition is not actionable.
            geometry_switch_confirm_cycles_ = 0;
          }
          // A single neutral measurement is a profile dropout, not a
          // contradiction; it neither adds nor discards confirmation.
          if(geometry_switch_confirm_cycles_ >= kGeometrySwitchConfirmCycles){
            bypass_side = opposite_side;
            geometry_switch_confirm_cycles_ = 0;
            side_quality_advantage_cycles = 0;
            side_infeasible_cycles = 0;
          }
          else{
            const double material_advantage =
              config_.temperature * std::log(4.0);
            if(opposite_side_safe &&
              current_effective_cost - opposite_effective_cost > material_advantage)
            {
              side_quality_advantage_cycles = std::min(
                side_quality_advantage_cycles + 1,
                kSideSwitchConfirmCycles);
            }
            else{
              side_quality_advantage_cycles = 0;
            }
            if(side_quality_advantage_cycles >= kSideSwitchConfirmCycles){
              bypass_side = opposite_side;
              side_quality_advantage_cycles = 0;
              side_infeasible_cycles = 0;
              geometry_switch_confirm_cycles_ = 0;
            }
          }
        }
      }
    }
    else{
      side_quality_advantage_cycles = 0;
      if(!allow_side_transition){
        // Later iterations of the same control cycle cannot consume another
        // infeasibility or quality confirmation, nor aggregate a side that was
        // not selected for this cycle.
        return statistics;
      }
      side_infeasible_cycles = std::min(
        side_infeasible_cycles + 1,
        kSideSwitchConfirmCycles);
      if(side_infeasible_cycles < kSideSwitchConfirmCycles){
        return statistics;
      }
      if(!opposite_side_safe){
        return statistics;
      }
      bypass_side = opposite_side;
      side_infeasible_cycles = 0;
      geometry_switch_confirm_cycles_ = 0;
    }

    const bool side_switched =
      side_before_selection != MppiBypassSide::Neutral &&
      side_before_selection != bypass_side;
    avoidance_stage = side_switched ? MppiAvoidanceStage::Depart :
      advanceAvoidanceStage(avoidance_stage, bypass_side, directional_context);
    const double outward_direction = static_cast<double>(bypass_side);
    const double departure_scale = std::max(config_.departure_lateral_offset, 0.1);
    const double remaining_offset = directional_context.offset_valid &&
      std::isfinite(directional_context.signed_path_offset) ?
      std::max(0.0, config_.departure_lateral_offset -
        outward_direction * directional_context.signed_path_offset) : 0.0;
    for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
      const bool physical_side_selected = collision_free[sample] &&
        valid_sides[sample] == bypass_side;
      selected[sample] = physical_side_selected;
      if(!physical_side_selected || !std::isfinite(lateral_progress[sample])){
        continue;
      }
      const double outward_progress = outward_direction * lateral_progress[sample];
      // This diagnostic now counts geometric progress, not angular-velocity sign.
      if(outward_progress > 0.0){
        ++statistics.phase_compatible_candidates;
      }
      if(avoidance_stage == MppiAvoidanceStage::Depart &&
        directional_context.offset_valid && remaining_offset > 0.0)
      {
        // A path-left bypass on a right bend can still require right steering.
        // Prefer outward progress softly; never discard a safe same-side arc
        // because of its first steering command. Bound the weight reduction to
        // a factor of four even at low softmax temperatures.
        const double shortfall = clamp(
          (remaining_offset - outward_progress) / departure_scale, 0.0, 1.0);
        selection_costs[sample] += config_.temperature * std::log(4.0) * shortfall;
      }
    }

    // A committed bypass must not let the blocked corridor pull it back in.
    // Plain cost softmax over every same-side candidate lets the cheapest slow
    // near-straight candidate win, and PathAlign then drags the vehicle back to
    // the path until all fast candidates collide and forward planning fails.
    // The guard is therefore "do not close on the path", not "keep opening":
    // once the current offset already clears the obstacle, a straight candidate
    // is safe and must stay selectable. Requiring strictly positive progress
    // instead re-issues the departure command every cycle and drives the
    // vehicle ever further from the route while the path ahead stays blocked.
    // Depart keeps its soft preference so it can still select safe arcs that
    // do not progress yet. Fall back to the full same-side set when no candidate
    // opens clearance at all, so a transient geometry gap cannot strand the
    // controller.
    constexpr double kBypassClosingTolerance = 0.05;
    if(avoidance_stage == MppiAvoidanceStage::Bypass &&
      statistics.phase_compatible_candidates > 0)
    {
      statistics.bypass_guard_applied = true;
      for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
        if(!selected[sample]){
          continue;
        }
        const double progress = outward_direction * lateral_progress[sample];
        if(!(std::isfinite(progress) && progress >= -kBypassClosingTolerance)){
          selected[sample] = false;
          ++statistics.bypass_guard_removed;
        }
      }
    }
  }

  MppiWeightedUpdateRequest update_request;
  update_request.batch_size = valid_sequences.size();
  update_request.time_steps = config_.time_steps;
  update_request.command_index = command_index;
  update_request.temperature = config_.temperature;
  update_request.sequences.reserve(valid_sequences.size() * config_.time_steps);
  for(const auto & sequence : valid_sequences){
    update_request.sequences.insert(
      update_request.sequences.end(), sequence.begin(), sequence.end());
  }
  update_request.costs = selection_costs;
  // Diagnostics (ESS and directional masses) must stay on the raw valid costs;
  // the Depart soft penalty above only biases the selected control update.
  update_request.diagnostic_costs = valid_costs;
  update_request.selected.resize(selected.size());
  for(std::size_t sample = 0; sample < selected.size(); ++sample){
    update_request.selected[sample] = selected[sample] ? 1u : 0u;
  }
  update_request.structured.resize(valid_structured.size());
  for(std::size_t sample = 0; sample < valid_structured.size(); ++sample){
    update_request.structured[sample] = valid_structured[sample] ? 1u : 0u;
  }

  MppiWeightedUpdateResult update_result;
  std::string error;
  bool updated_ok = false;
  if(config_.compute_backend == MppiComputeBackend::Cuda){
    if(!cuda_backend_ || !cuda_backend_->weightedUpdate(
        update_request, update_result, error))
    {
      backend_error = error.empty() ?
        "CUDA weighted update failed" : error;
      return statistics;
    }
    updated_ok = true;
  }
  else{
    updated_ok = mppiWeightedUpdateCpu(update_request, update_result, error);
  }
  if(!updated_ok || update_result.control_sequence.size() != config_.time_steps){
    return statistics;
  }
  control_sequence_ = update_result.control_sequence;
  statistics.valid = true;
  statistics.prefilter_command = control_sequence_[command_index];
  statistics.effective_sample_size = update_result.effective_sample_size;
  statistics.positive_weight_mass = update_result.positive_weight_mass;
  statistics.negative_weight_mass = update_result.negative_weight_mass;
  statistics.neutral_weight_mass = update_result.neutral_weight_mass;
  statistics.structured_weight_mass = update_result.structured_weight_mass;
  return statistics;
}

void MppiController::smoothControlSequence(
  const MppiControl & measured_speed)
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
  const std::size_t command_index =
    config_.shift_control_sequence && filtered.size() > 1 ? 1 : 0;
  if(command_index < filtered.size()){
    // Keep the optimized first executable command. Filtering it against old
    // straight-line history was erasing a valid turn before it reached cmd_vel.
    filtered[command_index] = unfiltered[command_index];
  }
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
