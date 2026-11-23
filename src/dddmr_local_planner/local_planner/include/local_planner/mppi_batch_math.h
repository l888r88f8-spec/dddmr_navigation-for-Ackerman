#ifndef LOCAL_PLANNER_MPPI_BATCH_MATH_H_
#define LOCAL_PLANNER_MPPI_BATCH_MATH_H_

// Host/device shared pure math for MPPI candidate generation and weighting.
// These routines contain no STL containers so the CPU reference and the CUDA
// kernels execute literally the same constraint/energy expressions. Keep this
// header limited to arithmetic: it is compiled by nvcc as device code as well.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "local_planner/mppi_cuda_backend.h"

#if defined(__CUDACC__)
#define MPPI_HD __host__ __device__
#else
#define MPPI_HD
#endif

namespace local_planner
{
namespace mppi_math
{

MPPI_HD inline double clampValue(const double value, const double low, const double high)
{
  return value < low ? low : (value > high ? high : value);
}

MPPI_HD inline double minValue(const double a, const double b)
{
  return a < b ? a : b;
}

MPPI_HD inline double maxValue(const double a, const double b)
{
  return a > b ? a : b;
}

// Overflow-checked size arithmetic shared by the CPU reference and CUDA
// staging. Failed checks must clear outputs and return an error, never wrap.
MPPI_HD inline bool checkedMulSize(
  const std::size_t a, const std::size_t b, std::size_t & out)
{
  if(a != 0 && b > static_cast<std::size_t>(-1) / a){
    return false;
  }
  out = a * b;
  return true;
}

MPPI_HD inline bool checkedBytes(
  const std::size_t count, const std::size_t element_size, std::size_t & out)
{
  return checkedMulSize(count, element_size, out);
}

// Counter-based SplitMix64 + Box-Muller. Every (seed, generation, sample,
// step, component) maps to an independent standard normal, so the same
// backend request replays exactly and no host RNG batch is needed.
MPPI_HD inline std::uint64_t splitMix64(std::uint64_t value)
{
  value += 0x9E3779B97F4A7C15ULL;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ULL;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBULL;
  return value ^ (value >> 31);
}

MPPI_HD inline double uniformOpen01(const std::uint64_t bits)
{
  // 53-bit mantissa in (0,1): never 0 (log) and never 1 (sqrt/log boundary).
  return (static_cast<double>(bits >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

MPPI_HD inline MppiControl generatedNoise(
  const MppiGenerationParams & params,
  const std::size_t sample,
  const std::size_t step)
{
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  std::uint64_t key = static_cast<std::uint64_t>(params.seed) ^
    (params.noise_generation * 0xD1B54A32D192ED03ULL);
  key = splitMix64(key + static_cast<std::uint64_t>(sample) * 0x9E3779B97F4A7C15ULL);
  key = splitMix64(key + static_cast<std::uint64_t>(step) * 0xC2B2AE3D27D4EB4FULL);
  const std::uint64_t bits0 = splitMix64(key + 0x165667B19E3779F9ULL);
  const std::uint64_t bits1 = splitMix64(bits0 + 0x27D4EB2F165667C5ULL);
  const double u1 = uniformOpen01(bits0);
  const double u2 = uniformOpen01(bits1);
  const double radius = sqrt(-2.0 * log(u1));
  const double theta = kTwoPi * u2;
  MppiControl noise;
  noise.vx = params.vx_std * radius * cos(theta);
  noise.wz = params.wz_std * radius * sin(theta);
  return noise;
}

MPPI_HD inline double ackermannLimit(const MppiGenerationParams & params, const double vx)
{
  const double steering_limit =
    fabs(vx) * tan(params.max_steer) / params.wheelbase;
  return minValue(params.wz_max, steering_limit);
}

MPPI_HD inline MppiControl constrain(
  const MppiGenerationParams & params,
  const MppiControl & previous,
  const MppiControl & requested,
  const double dt,
  const double vx_low,
  const double vx_high)
{
  const double low = minValue(vx_low, vx_high);
  const double high = maxValue(vx_low, vx_high);
  const double reachable_low = previous.vx + params.ax_min * dt;
  const double reachable_high = previous.vx + params.ax_max * dt;
  MppiControl constrained;
  constrained.vx = clampValue(requested.vx, reachable_low, reachable_high);

  double bounded_low = low;
  double bounded_high = high;
  if(previous.vx < low){
    bounded_low = minValue(low, reachable_high);
  }
  else if(previous.vx > high){
    bounded_high = maxValue(high, reachable_low);
  }
  constrained.vx = clampValue(
    constrained.vx, minValue(bounded_low, bounded_high),
    maxValue(bounded_low, bounded_high));

  const double requested_wz = clampValue(
    requested.wz,
    previous.wz - params.az_max * dt,
    previous.wz + params.az_max * dt);
  const double limit = ackermannLimit(params, constrained.vx);
  constrained.wz = clampValue(requested_wz, -limit, limit);
  return constrained;
}

MPPI_HD inline MppiControl constrainDefault(
  const MppiGenerationParams & params,
  const MppiControl & previous,
  const MppiControl & requested,
  const double dt)
{
  return constrain(
    params, previous, requested, dt, params.vx_min, params.vx_max);
}

MPPI_HD inline MppiControl noiseAt(
  const MppiGenerationParams & params,
  const MppiControl * noise,
  const std::size_t noise_count,
  const std::size_t noise_batch_size,
  const std::size_t sample,
  const std::size_t step)
{
  if(params.use_device_noise){
    return generatedNoise(params, sample, step);
  }
  if(noise_batch_size == 0 || noise_count == 0 || noise == nullptr){
    return MppiControl{};
  }
  const std::size_t wrapped_sample = sample % noise_batch_size;
  const std::size_t row = wrapped_sample * params.time_steps;
  const std::size_t index = row + (step % params.time_steps);
  return index < noise_count ? noise[index] : MppiControl{};
}

// Writes params.time_steps controls and the control energy for one noised
// sample. warm/control sequence may be shorter than time_steps; missing warm
// entries are treated as zero exactly like the controller's historical path.
MPPI_HD inline void generateNoisedSample(
  const MppiGenerationParams & params,
  const MppiControl & measured_speed,
  const MppiControl * warm,
  const std::size_t warm_count,
  const MppiControl * noise,
  const std::size_t noise_count,
  const std::size_t noise_batch_size,
  const std::size_t sample,
  MppiControl * out,
  double & energy)
{
  MppiControl previous = measured_speed;
  std::size_t first_noised_step = 0;
  if(params.shift_control_sequence && params.time_steps > 0){
    out[0] = measured_speed;
    previous = out[0];
    first_noised_step = 1;
  }
  energy = 0.0;
  for(std::size_t step = first_noised_step; step < params.time_steps; ++step){
    const MppiControl noise_value = noiseAt(
      params, noise, noise_count, noise_batch_size, sample, step);
    const double warm_vx = step < warm_count ? warm[step].vx : 0.0;
    const double warm_wz = step < warm_count ? warm[step].wz : 0.0;
    const double dt = step == 0 ? params.controller_period : params.model_dt;
    MppiControl requested;
    requested.vx = warm_vx + noise_value.vx;
    const MppiControl vx_only = constrainDefault(
      params, previous, MppiControl{requested.vx, previous.wz}, dt);
    const double angular_noise_scale = params.use_debug_noise ? 1.0 :
      (params.wz_std > 1e-9 ?
      minValue(1.0, ackermannLimit(params, vx_only.vx) / params.wz_std) : 0.0);
    requested.wz = warm_wz + noise_value.wz * angular_noise_scale;
    if(params.vx_std > 1e-9){
      energy += params.gamma / (params.vx_std * params.vx_std) *
        warm_vx * noise_value.vx;
    }
    if(params.wz_std > 1e-9 && angular_noise_scale > 1e-9){
      const double effective_wz_std = params.wz_std * angular_noise_scale;
      const double effective_wz_noise = noise_value.wz * angular_noise_scale;
      energy += params.gamma / (effective_wz_std * effective_wz_std) *
        warm_wz * effective_wz_noise;
    }
    out[step] = constrainDefault(params, previous, requested, dt);
    previous = out[step];
  }
}

MPPI_HD inline void generateStructuredSample(
  const MppiGenerationParams & params,
  const MppiControl & measured_speed,
  const MppiControl * warm,
  const std::size_t warm_count,
  const std::size_t sample,
  MppiControl * out)
{
  struct StructuredProfile
  {
    double speed_scale;
    double curvature_fraction;
    bool s_curve;
  };
  // Same explicit profile map as MppiController::buildStructuredSequence.
  const StructuredProfile profiles[4] = {
    {1.00, 0.55, false},
    {0.75, 0.90, false},
    {1.50, 0.90, true},
    {1.25, 1.00, false},
  };
  const bool has_latched_side = params.bypass_side != MppiBypassSide::Neutral;
  const std::size_t profile_index = has_latched_side ? sample : sample / 2;
  const StructuredProfile & profile = profiles[profile_index % 4];
  const double direction = has_latched_side ?
    static_cast<double>(params.bypass_side) :
    (sample % 2 == 0 ? 1.0 : -1.0);
  const double intensity = profile.curvature_fraction;
  const bool s_curve = !has_latched_side && profile.s_curve;
  const double target_speed = minValue(
    params.vx_max, params.structured_min_vx * profile.speed_scale);

  MppiControl previous = measured_speed;
  std::size_t first_control_step = 0;
  if(params.shift_control_sequence && params.time_steps > 0){
    out[0] = measured_speed;
    first_control_step = 1;
  }
  for(std::size_t step = first_control_step; step < params.time_steps; ++step){
    const double dt = step == 0 ? params.controller_period : params.model_dt;
    const double warm_vx = step < warm_count ? warm[step].vx : 0.0;
    const double requested_vx = maxValue(warm_vx, target_speed);
    const MppiControl vx_only = constrainDefault(
      params, previous, MppiControl{requested_vx, previous.wz}, dt);
    double phase_direction = direction;
    if(has_latched_side){
      if(params.avoidance_stage == MppiAvoidanceStage::Bypass){
        switch(sample % 4){
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
    else if(s_curve && step - first_control_step >=
      (params.time_steps - first_control_step) / 2)
    {
      phase_direction = -direction;
    }
    const MppiControl requested{
      requested_vx,
      phase_direction * intensity * ackermannLimit(params, vx_only.vx)};
    out[step] = constrainDefault(params, previous, requested, dt);
    previous = out[step];
  }
}

// Mirrors MppiController::buildReverseSequence. Structured profiles are
// deterministic mirrored retreat arcs; the remaining samples use the same
// Gaussian noise and energy expressions as forward noised samples. The vx
// envelope never accelerates forward above the measured speed.
MPPI_HD inline void generateReverseSample(
  const MppiGenerationParams & params,
  const MppiControl & measured_speed,
  const MppiControl * noise,
  const std::size_t noise_count,
  const std::size_t noise_batch_size,
  const std::size_t sample,
  const bool structured,
  MppiControl * out,
  double & energy)
{
  const double ceiling = maxValue(0.0, params.reverse_speed_ceiling);
  MppiControl previous = measured_speed;
  std::size_t first_control_step = 0;
  if(params.shift_control_sequence && params.time_steps > 0){
    out[0] = measured_speed;
    previous = out[0];
    first_control_step = 1;
  }
  energy = 0.0;

  if(structured){
    struct ReverseProfile
    {
      double speed_scale;
      double curvature_fraction;
      double direction;
    };
    const ReverseProfile profiles[8] = {
      {1.00, 0.00, 0.0},
      {0.60, 0.00, 0.0},
      {1.00, 0.55, 1.0},
      {1.00, 0.55, -1.0},
      {1.00, 0.90, 1.0},
      {1.00, 0.90, -1.0},
      {0.75, 1.00, 1.0},
      {0.75, 1.00, -1.0},
    };
    const ReverseProfile & profile = profiles[sample % 8];
    const double target_speed = -ceiling * profile.speed_scale;
    for(std::size_t step = first_control_step; step < params.time_steps; ++step){
      const double dt = step == 0 ? params.controller_period : params.model_dt;
      const double high = maxValue(0.0, previous.vx);
      const MppiControl vx_only = constrain(
        params, previous, MppiControl{target_speed, previous.wz}, dt,
        -ceiling, high);
      double requested_wz = 0.0;
      if(fabs(profile.curvature_fraction) > 1e-9 && vx_only.vx < -1e-6){
        requested_wz = profile.direction * profile.curvature_fraction *
          ackermannLimit(params, vx_only.vx);
      }
      out[step] = constrain(
        params, previous, MppiControl{target_speed, requested_wz}, dt,
        -ceiling, high);
      previous = out[step];
    }
    return;
  }

  const double angular_noise_scale = params.wz_std > 1e-9 ?
    minValue(1.0, ackermannLimit(params, ceiling) / params.wz_std) : 0.0;
  for(std::size_t step = first_control_step; step < params.time_steps; ++step){
    const double dt = step == 0 ? params.controller_period : params.model_dt;
    const MppiControl noise_value = noiseAt(
      params, noise, noise_count, noise_batch_size, sample, step);
    const double high = maxValue(0.0, previous.vx);
    const double requested_vx = clampValue(
      -ceiling + noise_value.vx, -ceiling, high);
    const double requested_wz = previous.wz +
      noise_value.wz * angular_noise_scale;
    if(params.vx_std > 1e-9){
      energy += params.gamma / (params.vx_std * params.vx_std) *
        (-ceiling) * noise_value.vx;
    }
    if(params.wz_std > 1e-9 && angular_noise_scale > 1e-9){
      const double effective_wz_std = params.wz_std * angular_noise_scale;
      const double effective_wz_noise = noise_value.wz * angular_noise_scale;
      energy += params.gamma / (effective_wz_std * effective_wz_std) *
        previous.wz * effective_wz_noise;
    }
    out[step] = constrain(
      params, previous, MppiControl{requested_vx, requested_wz}, dt,
      -ceiling, high);
    previous = out[step];
  }
}

constexpr double kAngularDeadband = 1e-4;

}  // namespace mppi_math
}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_BATCH_MATH_H_
