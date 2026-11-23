#ifndef LOCAL_PLANNER_MPPI_CONTROLLER_H_
#define LOCAL_PLANNER_MPPI_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace local_planner
{

class MppiCudaBackend;

enum class MppiComputeBackend : int
{
  Cpu = 0,
  Cuda = 1
};

const char * toString(MppiComputeBackend backend);

// Strict, case-sensitive parameter parser: only "cpu" and "cuda" are accepted.
// Unknown values must be rejected by callers instead of silently selecting CPU.
bool parseMppiComputeBackend(
  const std::string & value, MppiComputeBackend & backend);

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
  // Cost per metre of lateral travel needed to reach the bypass band on a
  // side. Only applied while the blocking geometry does not force a side.
  // 0 disables the detour preference and restores pure cost comparison.
  double bypass_detour_weight = 1.0;
  double gamma = 0.015;
  double vx_std = 0.18;
  double wz_std = 0.25;
  double vx_min = 0.05;
  double vx_max = 0.8;
  // Distance-parameterized collision probe. The time-parameterized rollout
  // above stops at |vx| * time_steps * model_dt, so at low speed it never
  // reaches the obstacle it is creeping toward and the collision term cannot
  // act. The probe continues the terminal command as a constant-curvature arc
  // sampled by arc length, collision-only (no clearance query), and exits on
  // the first contact. Zero disables it.
  double evaluation_distance = 6.5;  // m
  double evaluation_step = 0.15;     // m
  std::size_t evaluation_max_points = 64;
  // Recovery is opt-in and only starts after forward infeasibility.
  bool reverse_recovery_enabled = false;
  double reverse_max_speed = 0.15;
  // Deprecated compatibility parameter, ignored. There is no accumulated travel cap.
  double reverse_episode_distance = 0.5;
  std::size_t reverse_structured_candidate_count = 8;
  double structured_min_vx = 0.20;
  double departure_lateral_offset = 0.42;
  double wz_max = 1.2;
  double ax_min = -0.8;
  double ax_max = 0.8;
  double az_max = 1.5;
  double wheelbase = 0.550;
  double max_steer = 0.3270520925102273;  // HunterSEParams virtual central angle (22 deg inner wheel)
  bool shift_control_sequence = true;
  bool regenerate_noises = false;
  // Cpu keeps the historical in-process path. Cuda executes candidate
  // generation and the selected weighted update on the GPU and fails closed
  // with backend_error when no usable device exists; it never silently falls
  // back to the CPU implementation.
  MppiComputeBackend compute_backend = MppiComputeBackend::Cpu;
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
    const bool valid = true,
    const MppiBypassSide preferred = MppiBypassSide::Neutral)
  : path_blocked(blocked), signed_path_offset(path_offset), offset_valid(valid),
    preferred_side(preferred) {}

  bool path_blocked = false;
  double signed_path_offset = 0.0;
  bool offset_valid = true;
  // Side required by the blocking-obstacle geometry (Autoware-style). Neutral
  // means the geometry is ambiguous or that side lacks clearance, so the
  // controller keeps its previous cost-based selection.
  MppiBypassSide preferred_side = MppiBypassSide::Neutral;
  // Lateral distance (m) from the current vehicle offset to the usable bypass
  // band on each side. Used only while no side is geometrically forced, to
  // keep the manoeuvre on whichever side is cheaper to reach.
  double left_detour = 0.0;
  double right_detour = 0.0;
  bool detour_valid = false;
};

struct MppiAvoidanceState
{
  MppiBypassSide side = MppiBypassSide::Neutral;
  std::size_t side_infeasible_cycles = 0;
  MppiAvoidanceStage stage = MppiAvoidanceStage::Neutral;
  std::size_t side_quality_advantage_cycles = 0;
};

struct MppiCandidateScore
{
  bool valid = false;
  double cost = std::numeric_limits<double>::infinity();
  MppiBypassSide bypass_side = MppiBypassSide::Neutral;
  bool collision_free = true;
  // Change in signed path offset over the first second of the rollout.
  // Positive means movement toward the path's left, regardless of steering sign.
  double signed_lateral_progress = std::numeric_limits<double>::quiet_NaN();
};

bool mppiSequenceRespectsSpeedCap(
  const std::vector<MppiControl> & controls,
  double measured_vx,
  double speed_cap,
  bool first_control_is_measured_state,
  double ax_min,
  double model_dt,
  double controller_period,
  double ax_max = std::numeric_limits<double>::quiet_NaN());

// Linear goal-approach speed cap for the MPPI backends, mirroring the
// regulated pure-pursuit goal slowdown: cruise_cap at slowdown_distance and
// beyond, min_speed at the goal, linear in between. A non-finite distance or a
// non-positive slowdown distance leaves the cap untouched.
double mppiGoalApproachSpeedCap(
  double distance_to_goal,
  double cruise_cap,
  double slowdown_distance,
  double min_speed);

struct MppiResult
{
  bool has_valid_control = false;
  // Non-empty only when an explicitly requested backend could not execute.
  std::string backend_error;
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
  std::size_t side_quality_advantage_cycles = 0;
  double best_left_cost = std::numeric_limits<double>::infinity();
  double best_right_cost = std::numeric_limits<double>::infinity();
  double best_critic_cost = std::numeric_limits<double>::infinity();
  double best_control_energy = 0.0;
  MppiBypassSide best_sample_side = MppiBypassSide::Neutral;
  std::size_t feasible_left_candidates = 0;
  std::size_t feasible_right_candidates = 0;
  std::size_t feasible_neutral_candidates = 0;
  // Safe candidates on the selected side with positive path-relative progress.
  std::size_t phase_compatible_candidates = 0;
  // Diagnostic only: whether the committed-bypass "do not close on the path"
  // filter was armed this cycle, and how many same-side candidates it removed.
  bool bypass_guard_applied = false;
  std::size_t bypass_guard_removed = 0;
  MppiAvoidanceStage avoidance_stage = MppiAvoidanceStage::Neutral;
  bool reverse_recovery_active = false;
  bool reverse_motion_active = false;
  bool reverse_candidates_generated = false;
  bool reverse_budget_exhausted = false;
  double reverse_distance_used = 0.0;
  double reverse_distance_remaining = 0.0;
};

class MppiController
{
public:
  using ScoreFunction = std::function<MppiCandidateScore(const MppiCandidate &)>;
  using BatchScoreFunction =
    std::function<std::vector<MppiCandidateScore>(const std::vector<MppiCandidate> &)>;
  using RecoveryFunction = std::function<MppiResult(bool)>;
  using ModeSelectionFunction = std::function<MppiDirectionalContext()>;

  MppiController();
  explicit MppiController(const MppiConfig & config);
  ~MppiController();

  void setConfig(const MppiConfig & config);
  const MppiConfig & config() const;

  MppiControl debugNoiseAt(std::size_t sample_index, std::size_t step_index) const;
  const std::vector<MppiControl> & controlSequence() const;
  MppiAvoidanceState avoidanceState() const;
  bool reverseRecoveryActive() const;
  bool reverseMotionActive() const;
  double reverseDistanceUsed() const;
  double reverseDistanceRemaining() const;
  // Measured travel telemetry; it never disables recovery or rearms a budget.
  void noteMeasuredReverseTravel(double distance);
  void noteMeasuredForwardTravel(double distance);
  // Explicit lifecycle reset; ordinary stop/retry must retain the directed route.
  void resetReverseRecoveryEpisode();
  void resetControlSequence(const MppiControl & command);
  void resetControlSequenceForRetry(
    const MppiControl & command,
    const MppiAvoidanceState & avoidance_state);
  void setDebugNoiseForTest(const std::vector<std::vector<MppiControl>> & noise);

  MppiResult optimize(const MppiControl & measured_speed, const ScoreFunction & score_function);
  MppiResult optimizeBatch(
    const MppiControl & measured_speed,
    const BatchScoreFunction & score_function,
    const ModeSelectionFunction & mode_selection_function = {},
    double runtime_speed_limit = -1.0,
    const RecoveryFunction & recovery_function = {});

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
    bool bypass_guard_applied = false;
    std::size_t bypass_guard_removed = 0;
  };

  MppiConfig config_;
  std::unique_ptr<MppiCudaBackend> cuda_backend_;
  bool cuda_backend_ready_ = false;
  std::string cuda_backend_unavailable_reason_;
  RecoveryFunction recovery_function_;
  std::vector<MppiControl> control_sequence_;
  std::array<MppiControl, 4> control_history_{};
  std::vector<std::vector<MppiControl>> noise_;
  std::mt19937 generator_;
  std::uint64_t cuda_noise_generation_ = 0;
  bool use_debug_noise_for_test_ = false;
  MppiBypassSide latched_bypass_side_ = MppiBypassSide::Neutral;
  std::size_t latched_side_infeasible_cycles_ = 0;
  std::size_t latched_side_quality_advantage_cycles_ = 0;
  // Consecutive cycles in which the blocking-obstacle geometry has asked to
  // move the committed side to the other side. Kept separately from the cost
  // counter so a slow, sustained geometric correction is not confused with a
  // short cost advantage, and vice versa.
  std::size_t geometry_switch_confirm_cycles_ = 0;
  MppiAvoidanceStage avoidance_stage_ = MppiAvoidanceStage::Neutral;
  // Persistent recovery ownership and current motion phase.
  bool reverse_recovery_active_ = false;
  bool reverse_motion_active_ = false;
  std::vector<MppiControl> reverse_control_sequence_;
  double reverse_distance_used_ = 0.0;
  double reverse_forward_travel_ = 0.0;
  std::size_t reverse_forward_exit_cycles_ = 0;
  static constexpr std::size_t kSideSwitchConfirmCycles = 3;
  // A geometric side preference must persist over a much longer window than a
  // cost or infeasibility confirmation. The lateral profile is re-measured
  // every cycle around a blocking obstacle, and its sign can oscillate within
  // a few cycles when the obstacle straddles the route; switching a committed
  // side on that noise makes the vehicle cross the route twice. A sustained
  // preference (the observed wrong-side case lasted several seconds) still
  // completes this window quickly.
  static constexpr std::size_t kGeometrySwitchConfirmCycles = 30;
  static constexpr std::size_t kReverseForwardExitCycles = 3;
  static constexpr double kReverseDistanceEpsilon = 1e-6;
  static constexpr double kReverseForwardMotionEpsilon = 1e-3;

  void normalizeConfig();
  void regenerateNoise();
  double clamp(double value, double low, double high) const;
  double ackermannWzLimit(double vx) const;
  MppiAvoidanceStage advanceAvoidanceStage(
    MppiAvoidanceStage current,
    MppiBypassSide latched_side,
    const MppiDirectionalContext & context) const;
  std::size_t activeStructuredCandidateCount(MppiBypassSide bypass_side) const;
  MppiControl constrainControl(
    const MppiControl & previous,
    const MppiControl & requested,
    double dt) const;
  MppiControl constrainControl(
    const MppiControl & previous,
    const MppiControl & requested,
    double dt,
    double vx_low,
    double vx_high) const;
  bool canAttemptReverseRecovery() const;
  double effectiveReverseSpeedLimit(double runtime_speed_limit) const;
  double reverseSpeedCeiling(double runtime_speed_limit) const;
  std::size_t activeReverseStructuredCandidateCount() const;
  bool tryReverseFallback(
    const MppiControl & measured_speed,
    const BatchScoreFunction & score_function,
    double reverse_speed_ceiling,
    MppiResult & result);
  SampledSequence buildNoisedSequence(
    const MppiControl & measured_speed,
    std::size_t sample_index,
    MppiBypassSide bypass_side,
    MppiAvoidanceStage avoidance_stage) const;
  SampledSequence buildStructuredSequence(
    const MppiControl & measured_speed,
    std::size_t sample_index,
    MppiBypassSide bypass_side,
    MppiAvoidanceStage avoidance_stage) const;
  SampledSequence buildReverseSequence(
    const MppiControl & measured_speed,
    std::size_t sample_index,
    double speed_limit) const;
  UpdateStatistics applySoftmaxUpdate(
    const std::vector<std::vector<MppiControl>> & valid_sequences,
    const std::vector<double> & valid_costs,
    const std::vector<MppiBypassSide> & valid_sides,
    const std::vector<bool> & collision_free,
    const std::vector<bool> & valid_structured,
    const std::vector<double> & lateral_progress,
    bool select_directional_mode,
    const MppiDirectionalContext & directional_context,
    MppiBypassSide & bypass_side,
    std::size_t & side_infeasible_cycles,
    std::size_t & side_quality_advantage_cycles,
    MppiAvoidanceStage & avoidance_stage,
    bool allow_side_transition,
    std::string & backend_error);
  void smoothControlSequence(const MppiControl & measured_speed);
  void updateControlHistory(const MppiControl & command);
  void shiftControlSequence();
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CONTROLLER_H_
