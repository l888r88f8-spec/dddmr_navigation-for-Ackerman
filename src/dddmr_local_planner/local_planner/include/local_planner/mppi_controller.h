#ifndef LOCAL_PLANNER_MPPI_CONTROLLER_H_
#define LOCAL_PLANNER_MPPI_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <vector>

namespace local_planner
{

struct MppiControl
{
  double vx = 0.0;
  double wz = 0.0;
};

struct MppiConfig
{
  std::size_t batch_size = 384;
  std::size_t structured_candidate_count = 8;
  std::size_t time_steps = 25;
  std::size_t iteration_count = 1;
  double model_dt = 0.1;
  double controller_period = 0.1;
  double temperature = 0.3;
  double gamma = 0.015;
  double vx_std = 0.18;
  double wz_std = 0.25;
  double vx_min = 0.05;
  double vx_max = 0.8;
  double structured_min_vx = 0.20;
  double departure_lateral_offset = 0.42;
  double wz_max = 1.2;
  double ax_min = -0.8;
  double ax_max = 0.8;
  double az_max = 1.5;
  double wheelbase = 0.55;
  double max_steer = 0.69;
  bool shift_control_sequence = true;
  bool regenerate_noises = false;
  std::uint32_t seed = 7;
};

struct MppiCandidate
{
  std::vector<MppiControl> controls;
};

enum class MppiBypassSide : int
{
  Right = -1,
  Neutral = 0,
  Left = 1
};

const char * toString(MppiBypassSide side);

enum class MppiAvoidanceStage : int
{
  Neutral = 0,
  Depart = 1,
  Bypass = 2
};

const char * toString(MppiAvoidanceStage stage);

struct MppiDirectionalContext
{
  MppiDirectionalContext() = default;
  MppiDirectionalContext(const bool blocked)
  : path_blocked(blocked) {}
  MppiDirectionalContext(
    const bool blocked,
    const double path_offset,
    const bool valid = true)
  : path_blocked(blocked), signed_path_offset(path_offset), offset_valid(valid) {}

  bool path_blocked = false;
  double signed_path_offset = 0.0;
  bool offset_valid = true;
};

struct MppiAvoidanceState
{
  MppiBypassSide side = MppiBypassSide::Neutral;
  std::size_t side_infeasible_cycles = 0;
  std::size_t structured_phase_step = 0;
  MppiAvoidanceStage stage = MppiAvoidanceStage::Neutral;
};

struct MppiCandidateScore
{
  bool valid = false;
  double cost = std::numeric_limits<double>::infinity();
  MppiBypassSide bypass_side = MppiBypassSide::Neutral;
  bool collision_free = true;
};

bool mppiSequenceRespectsSpeedCap(
  const std::vector<MppiControl> & controls,
  double measured_vx,
  double speed_cap,
  bool first_control_is_measured_state,
  double ax_min,
  double model_dt,
  double controller_period);

struct MppiResult
{
  bool has_valid_control = false;
  MppiControl command;
  double best_cost = std::numeric_limits<double>::infinity();
  std::size_t best_sample_index = 0;
  std::size_t rejected_samples = 0;
  std::vector<MppiControl> optimal_sequence;
  std::vector<MppiControl> best_sample_sequence;
  double effective_sample_size = 0.0;
  double positive_weight_mass = 0.0;
  double negative_weight_mass = 0.0;
  double neutral_weight_mass = 0.0;
  double structured_weight_mass = 0.0;
  MppiControl prefilter_command;
  std::size_t structured_candidates = 0;
  MppiBypassSide selected_bypass_side = MppiBypassSide::Neutral;
  std::size_t side_infeasible_cycles = 0;
  std::size_t structured_phase_step = 0;
  double best_critic_cost = std::numeric_limits<double>::infinity();
  double best_control_energy = 0.0;
  MppiBypassSide best_sample_side = MppiBypassSide::Neutral;
  std::size_t feasible_left_candidates = 0;
  std::size_t feasible_right_candidates = 0;
  std::size_t feasible_neutral_candidates = 0;
  std::size_t phase_compatible_candidates = 0;
  MppiAvoidanceStage avoidance_stage = MppiAvoidanceStage::Neutral;
};

class MppiController
{
public:
  using ScoreFunction = std::function<MppiCandidateScore(const MppiCandidate &)>;
  using BatchScoreFunction =
    std::function<std::vector<MppiCandidateScore>(const std::vector<MppiCandidate> &)>;
  using ModeSelectionFunction = std::function<MppiDirectionalContext()>;

  MppiController();
  explicit MppiController(const MppiConfig & config);

  void setConfig(const MppiConfig & config);
  const MppiConfig & config() const;

  MppiControl debugNoiseAt(std::size_t sample_index, std::size_t step_index) const;
  const std::vector<MppiControl> & controlSequence() const;
  MppiAvoidanceState avoidanceState() const;
  void resetControlSequence(const MppiControl & command);
  void resetControlSequenceForRetry(
    const MppiControl & command,
    const MppiAvoidanceState & avoidance_state);
  void setDebugNoiseForTest(const std::vector<std::vector<MppiControl>> & noise);

  MppiResult optimize(const MppiControl & measured_speed, const ScoreFunction & score_function);
  MppiResult optimizeBatch(
    const MppiControl & measured_speed,
    const BatchScoreFunction & score_function,
    const ModeSelectionFunction & mode_selection_function = {});

private:
  struct SampledSequence
  {
    std::vector<MppiControl> controls;
    double control_energy = 0.0;
    bool structured = false;
  };

  struct UpdateStatistics
  {
    bool valid = false;
    double effective_sample_size = 0.0;
    double positive_weight_mass = 0.0;
    double negative_weight_mass = 0.0;
    double neutral_weight_mass = 0.0;
    double structured_weight_mass = 0.0;
    MppiControl prefilter_command;
    std::size_t phase_compatible_candidates = 0;
  };

  MppiConfig config_;
  std::vector<MppiControl> control_sequence_;
  std::array<MppiControl, 4> control_history_{};
  std::vector<std::vector<MppiControl>> noise_;
  std::mt19937 generator_;
  bool use_debug_noise_for_test_ = false;
  MppiBypassSide latched_bypass_side_ = MppiBypassSide::Neutral;
  std::size_t latched_side_infeasible_cycles_ = 0;
  std::size_t structured_phase_step_ = 0;
  MppiAvoidanceStage avoidance_stage_ = MppiAvoidanceStage::Neutral;
  static constexpr std::size_t kSideSwitchConfirmCycles = 3;
  static constexpr double kMaterialPhaseCurvatureRatio = 0.25;

  void normalizeConfig();
  void regenerateNoise();
  double clamp(double value, double low, double high) const;
  double ackermannWzLimit(double vx) const;
  MppiAvoidanceStage advanceAvoidanceStage(
    MppiAvoidanceStage current,
    MppiBypassSide latched_side,
    const MppiDirectionalContext & context) const;
  double expectedPhaseDirection(
    MppiBypassSide bypass_side,
    std::size_t structured_phase_step) const;
  bool isPhaseCompatible(
    const MppiControl & command,
    double expected_direction) const;
  std::size_t activeStructuredCandidateCount(MppiBypassSide bypass_side) const;
  MppiControl constrainControl(
    const MppiControl & previous,
    const MppiControl & requested,
    double dt) const;
  SampledSequence buildNoisedSequence(
    const MppiControl & measured_speed,
    std::size_t sample_index,
    MppiBypassSide bypass_side,
    std::size_t structured_phase_step) const;
  SampledSequence buildStructuredSequence(
    const MppiControl & measured_speed,
    std::size_t sample_index,
    MppiBypassSide bypass_side,
    std::size_t structured_phase_step) const;
  UpdateStatistics applySoftmaxUpdate(
    const std::vector<std::vector<MppiControl>> & valid_sequences,
    const std::vector<double> & valid_costs,
    const std::vector<MppiBypassSide> & valid_sides,
    const std::vector<bool> & collision_free,
    const std::vector<bool> & valid_structured,
    bool select_directional_mode,
    std::size_t structured_phase_step,
    MppiBypassSide & bypass_side,
    std::size_t & side_infeasible_cycles);
  void smoothControlSequence(const MppiControl & measured_speed);
  void updateControlHistory(const MppiControl & command);
  void shiftControlSequence();
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CONTROLLER_H_
