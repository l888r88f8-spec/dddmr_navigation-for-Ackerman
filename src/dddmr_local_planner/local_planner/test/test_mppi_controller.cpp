#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "local_planner/mppi_controller.h"
#include "local_planner/mppi_critic_manager.h"

using local_planner::MppiCandidate;
using local_planner::MppiCandidateScore;
using local_planner::MppiAvoidanceState;
using local_planner::MppiAvoidanceStage;
using local_planner::MppiBypassSide;
using local_planner::MppiConfig;
using local_planner::MppiControl;
using local_planner::MppiController;
using local_planner::MppiDirectionalContext;
using local_planner::MppiPathPoint;
using local_planner::MppiScoredTrajectory;
using local_planner::mppiSequenceRespectsSpeedCap;
using local_planner::toString;

namespace
{

MppiConfig makeBypassLatchConfig()
{
  MppiConfig config;
  config.batch_size = 2;
  config.structured_candidate_count = 0;
  config.time_steps = 4;
  config.shift_control_sequence = false;
  config.temperature = 0.1;
  config.gamma = 0.0;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;
  return config;
}

std::vector<std::vector<MppiControl>> makeAngularNoises(
  const MppiConfig & config,
  const double first_wz,
  const double second_wz)
{
  return {
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, first_wz}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, second_wz})};
}

std::vector<MppiCandidateScore> makeSideScores(
  const double left_cost,
  const bool left_collision_free,
  const double right_cost,
  const bool right_collision_free)
{
  return {
    MppiCandidateScore{true, left_cost, MppiBypassSide::Left, left_collision_free},
    MppiCandidateScore{true, right_cost, MppiBypassSide::Right, right_collision_free}};
}

MppiConfig makeStructuredPhaseConfig()
{
  MppiConfig config;
  config.batch_size = 8;
  config.structured_candidate_count = 8;
  config.time_steps = 12;
  config.iteration_count = 1;
  config.shift_control_sequence = false;
  config.temperature = 0.1;
  config.gamma = 0.0;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.46;
  config.max_steer = 0.238;
  config.vx_std = 0.0;
  config.wz_std = 0.0;
  return config;
}

MppiConfig makeControlEnergyConfig()
{
  MppiConfig config;
  config.batch_size = 1;
  config.structured_candidate_count = 0;
  config.time_steps = 1;
  config.iteration_count = 1;
  config.shift_control_sequence = false;
  config.temperature = 1.0;
  config.gamma = 0.5;
  config.vx_std = 0.2;
  config.wz_std = 0.5;
  config.vx_min = 0.0;
  config.vx_max = 2.0;
  config.wz_max = 2.0;
  config.ax_min = -100.0;
  config.ax_max = 100.0;
  config.az_max = 100.0;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;
  return config;
}

local_planner::MppiResult latchLeftStructured(MppiController & controller)
{
  return controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 100.0, MppiBypassSide::Right, true});
      scores[4] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
      return scores;
    },
    []() {return true;});
}

MppiAvoidanceState makeLateLeftAvoidanceState(MppiController & controller)
{
  const auto initial = latchLeftStructured(controller);
  EXPECT_TRUE(initial.has_valid_control);
  EXPECT_EQ(initial.selected_bypass_side, MppiBypassSide::Left);

  for(std::size_t cycle = 1; cycle < 7; ++cycle){
    const auto advanced = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [](const std::vector<MppiCandidate> & candidates) {
        std::vector<MppiCandidateScore> scores(
          candidates.size(), MppiCandidateScore{false, -1.0});
        scores.front() = MppiCandidateScore{
          true, 0.0, MppiBypassSide::Left, true};
        return scores;
      },
      []() {return true;});
    EXPECT_TRUE(advanced.has_valid_control);
  }

  const auto force_right = [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
      scores.front() = MppiCandidateScore{
        true, 0.0, MppiBypassSide::Left, false};
      return scores;
    };
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, 0.0}, force_right, []() {return true;})
    .has_valid_control);
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, 0.0}, force_right, []() {return true;})
    .has_valid_control);
  return controller.avoidanceState();
}

MppiScoredTrajectory integrateWithMidpointModel(
  const std::vector<MppiControl> & controls,
  const double model_dt)
{
  MppiScoredTrajectory trajectory;
  trajectory.controls = controls;
  trajectory.points.reserve(controls.size());
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for(const auto & control : controls){
    x += control.vx * std::cos(yaw + control.wz * model_dt * 0.5) * model_dt;
    y += control.vx * std::sin(yaw + control.wz * model_dt * 0.5) * model_dt;
    yaw += control.wz * model_dt;
    trajectory.points.push_back(local_planner::MppiTrajectoryPoint{x, y, yaw});
  }
  return trajectory;
}

void expectLatchedStructuredSCurveCompletes(
  const MppiBypassSide selected_side,
  const std::size_t initial_s_candidate)
{
  const auto config = makeStructuredPhaseConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  const MppiBypassSide opposite_side =
    selected_side == MppiBypassSide::Left ?
    MppiBypassSide::Right : MppiBypassSide::Left;
  const double side_sign = static_cast<double>(selected_side);

  for(std::size_t cycle = 0; cycle < config.time_steps + 2; ++cycle){
    SCOPED_TRACE(cycle);
    const auto result = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [cycle, initial_s_candidate, selected_side, opposite_side](
        const std::vector<MppiCandidate> & candidates)
      {
        std::vector<MppiCandidateScore> scores(
          candidates.size(),
          MppiCandidateScore{true, 100.0, selected_side, true});
        const std::size_t winning_candidate = cycle == 0 ? initial_s_candidate : 0u;
        scores[winning_candidate].cost = 0.0;
        if(cycle == 0){
          for(std::size_t sample = 0; sample < scores.size(); ++sample){
            if(sample != initial_s_candidate){
              scores[sample].bypass_side = opposite_side;
            }
          }
        }
        return scores;
      },
      []() {return true;});

    ASSERT_TRUE(result.has_valid_control);
    EXPECT_EQ(result.selected_bypass_side, selected_side);
    EXPECT_EQ(
      result.structured_phase_step,
      std::min(cycle + 1, config.time_steps));
    if(cycle < config.time_steps / 2){
      EXPECT_GT(side_sign * result.command.wz, 0.0);
    }
    else if(cycle < config.time_steps){
      EXPECT_LT(side_sign * result.command.wz, 0.0);
    }
    else{
      EXPECT_NEAR(result.command.wz, 0.0, 1e-12);
    }
  }
}

}  // namespace

TEST(MppiControllerTest, BlockedPathBelowOffsetReportsDepartStage)
{
  auto config = makeBypassLatchConfig();
  config.departure_lateral_offset = 0.42;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 100.0, false);
    },
    []() {return MppiDirectionalContext{true, 0.05};});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.avoidance_stage, MppiAvoidanceStage::Depart);
}

TEST(MppiControllerTest, BypassSideDiagnosticStringsAreStable)
{
  EXPECT_STREQ(toString(MppiBypassSide::Left), "left");
  EXPECT_STREQ(toString(MppiBypassSide::Right), "right");
  EXPECT_STREQ(toString(MppiBypassSide::Neutral), "neutral");
}

TEST(MppiControllerTest, RetryResetRestoresAvoidanceSnapshotAndMeasuredWarmStart)
{
  auto config = makeStructuredPhaseConfig();
  config.time_steps = 25;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto snapshot = makeLateLeftAvoidanceState(controller);
  ASSERT_EQ(snapshot.side, MppiBypassSide::Left);
  ASSERT_EQ(snapshot.side_infeasible_cycles, 2u);
  ASSERT_EQ(snapshot.structured_phase_step, 7u);

  const auto force_right = [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
      scores.front() = MppiCandidateScore{
        true, 0.0, MppiBypassSide::Left, false};
      return scores;
    };
  const auto changed = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, force_right, []() {return true;});
  ASSERT_TRUE(changed.has_valid_control);
  ASSERT_EQ(changed.selected_bypass_side, MppiBypassSide::Right);
  ASSERT_EQ(changed.structured_phase_step, 0u);

  const MppiControl measured{0.2, 0.0};
  controller.resetControlSequenceForRetry(measured, snapshot);
  const auto restored = controller.avoidanceState();
  EXPECT_EQ(restored.side, snapshot.side);
  EXPECT_EQ(restored.side_infeasible_cycles, snapshot.side_infeasible_cycles);
  EXPECT_EQ(restored.structured_phase_step, snapshot.structured_phase_step);
  for(const auto & control : controller.controlSequence()){
    EXPECT_DOUBLE_EQ(control.vx, measured.vx);
    EXPECT_DOUBLE_EQ(control.wz, measured.wz);
  }

  controller.setDebugNoiseForTest(std::vector<std::vector<MppiControl>>(
      config.batch_size,
      std::vector<MppiControl>(config.time_steps, MppiControl{})));
  const auto after_retry = controller.optimizeBatch(
    measured,
    [](const std::vector<MppiCandidate> & candidates) {
      return std::vector<MppiCandidateScore>(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, true});
    },
    []() {return false;});
  ASSERT_TRUE(after_retry.has_valid_control);
  for(const auto & control : after_retry.optimal_sequence){
    EXPECT_DOUBLE_EQ(control.vx, measured.vx);
    EXPECT_DOUBLE_EQ(control.wz, measured.wz);
  }
}

TEST(MppiControllerTest, RetrySnapshotKeepsFeasibleLatchedSideDespiteCheaperOpposite)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);
  const auto snapshot = controller.avoidanceState();
  ASSERT_EQ(snapshot.side, MppiBypassSide::Left);

  controller.resetControlSequenceForRetry(MppiControl{0.2, 0.0}, snapshot);
  const auto retry = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(1000.0, true, 0.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(retry.has_valid_control);
  EXPECT_EQ(retry.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(retry.side_infeasible_cycles, 0u);
}

TEST(MppiControllerTest, NeutralRetrySnapshotAllowsNormalSideSelection)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  const auto neutral_snapshot = controller.avoidanceState();
  ASSERT_EQ(neutral_snapshot.side, MppiBypassSide::Neutral);
  ASSERT_EQ(neutral_snapshot.side_infeasible_cycles, 0u);
  ASSERT_EQ(neutral_snapshot.structured_phase_step, 0u);

  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto latched = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_EQ(latched.selected_bypass_side, MppiBypassSide::Left);

  controller.resetControlSequenceForRetry(
    MppiControl{0.2, 0.0}, neutral_snapshot);
  const auto retry = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(1.0, true, 0.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(retry.has_valid_control);
  EXPECT_EQ(retry.selected_bypass_side, MppiBypassSide::Right);
}

TEST(MppiControllerTest, FullResetAndSetConfigStillClearAvoidanceState)
{
  const auto config = makeStructuredPhaseConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto snapshot = makeLateLeftAvoidanceState(controller);
  ASSERT_EQ(snapshot.side, MppiBypassSide::Left);
  ASSERT_EQ(snapshot.side_infeasible_cycles, 2u);
  ASSERT_GT(snapshot.structured_phase_step, 0u);

  controller.resetControlSequence(MppiControl{0.2, 0.0});
  auto cleared = controller.avoidanceState();
  EXPECT_EQ(cleared.side, MppiBypassSide::Neutral);
  EXPECT_EQ(cleared.side_infeasible_cycles, 0u);
  EXPECT_EQ(cleared.structured_phase_step, 0u);

  controller.resetControlSequenceForRetry(MppiControl{0.2, 0.0}, snapshot);
  ASSERT_EQ(controller.avoidanceState().side, MppiBypassSide::Left);
  controller.setConfig(config);
  cleared = controller.avoidanceState();
  EXPECT_EQ(cleared.side, MppiBypassSide::Neutral);
  EXPECT_EQ(cleared.side_infeasible_cycles, 0u);
  EXPECT_EQ(cleared.structured_phase_step, 0u);
}

TEST(MppiControllerTest, RetryResetNormalizesUnsafeAvoidanceStateValues)
{
  auto config = makeStructuredPhaseConfig();
  config.shift_control_sequence = true;
  MppiController controller(config);
  const auto unsafe_size = std::numeric_limits<std::size_t>::max();

  controller.resetControlSequenceForRetry(
    MppiControl{0.2, 0.0},
    MppiAvoidanceState{
      static_cast<MppiBypassSide>(99), unsafe_size, unsafe_size});
  auto normalized = controller.avoidanceState();
  EXPECT_EQ(normalized.side, MppiBypassSide::Neutral);
  EXPECT_EQ(normalized.side_infeasible_cycles, 0u);
  EXPECT_EQ(normalized.structured_phase_step, 0u);

  controller.resetControlSequenceForRetry(
    MppiControl{0.2, 0.0},
    MppiAvoidanceState{MppiBypassSide::Left, unsafe_size, unsafe_size});
  normalized = controller.avoidanceState();
  EXPECT_EQ(normalized.side, MppiBypassSide::Left);
  EXPECT_EQ(normalized.side_infeasible_cycles, 3u);
  EXPECT_EQ(normalized.structured_phase_step, config.time_steps - 1u);
}

TEST(MppiControllerTest, ReportsAlignedBestSampleAndCollisionFreePhysicalSideCounts)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 4;
  config.time_steps = 1;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({
    {MppiControl{0.0, -0.4}},
    {MppiControl{0.0, 0.2}},
    {MppiControl{0.0, 0.3}},
    {MppiControl{0.0, 0.0}}});

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>{
        {true, 0.5, MppiBypassSide::Left, true},
        {true, 1.0, MppiBypassSide::Left, false},
        {true, 2.0, MppiBypassSide::Right, true},
        {true, 3.0, MppiBypassSide::Neutral, true}};
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.best_sample_side, MppiBypassSide::Left);
  EXPECT_EQ(result.best_sample_index, 0u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 0.5);
  EXPECT_DOUBLE_EQ(result.best_control_energy, 0.0);
  EXPECT_EQ(result.feasible_left_candidates, 1u);
  EXPECT_EQ(result.feasible_right_candidates, 1u);
  EXPECT_EQ(result.feasible_neutral_candidates, 1u);
  ASSERT_EQ(result.best_sample_sequence.size(), 1u);
  EXPECT_LT(result.best_sample_sequence.front().wz, 0.0);
}

TEST(MppiControllerTest, FinalIterationReplacesEarlierSideDiagnostics)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 3;
  config.time_steps = 1;
  config.iteration_count = 2;
  MppiController controller(config);
  controller.setDebugNoiseForTest({
    {MppiControl{0.0, 0.1}},
    {MppiControl{0.0, -0.2}},
    {MppiControl{0.0, 0.3}}});
  std::size_t score_calls = 0;

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> &) {
      if(score_calls++ == 0){
        return std::vector<MppiCandidateScore>{
          {true, 0.1, MppiBypassSide::Left, true},
          {true, 0.2, MppiBypassSide::Left, true},
          {true, 0.3, MppiBypassSide::Neutral, false}};
      }
      return std::vector<MppiCandidateScore>{
        {true, 2.0, MppiBypassSide::Left, false},
        {true, 0.4, MppiBypassSide::Right, true},
        {true, 1.0, MppiBypassSide::Neutral, true}};
    },
    []() {return false;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(result.best_sample_side, MppiBypassSide::Right);
  EXPECT_EQ(result.best_sample_index, 1u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 0.4);
  EXPECT_EQ(result.feasible_left_candidates, 0u);
  EXPECT_EQ(result.feasible_right_candidates, 1u);
  EXPECT_EQ(result.feasible_neutral_candidates, 1u);
}

TEST(MppiControllerTest, SeededGaussianNoiseIsRepeatable)
{
  MppiConfig config;
  config.batch_size = 16;
  config.time_steps = 8;
  config.seed = 42;
  config.vx_std = 0.2;
  config.wz_std = 0.4;

  MppiController first(config);
  MppiController second(config);

  const auto a = first.debugNoiseAt(7, 3);
  const auto b = second.debugNoiseAt(7, 3);

  EXPECT_DOUBLE_EQ(a.vx, b.vx);
  EXPECT_DOUBLE_EQ(a.wz, b.wz);
}

TEST(MppiControllerTest, DifferentSeedsProduceDifferentNoise)
{
  MppiConfig config_a;
  config_a.batch_size = 16;
  config_a.time_steps = 8;
  config_a.seed = 42;

  MppiConfig config_b = config_a;
  config_b.seed = 43;

  MppiController first(config_a);
  MppiController second(config_b);

  const auto a = first.debugNoiseAt(7, 3);
  const auto b = second.debugNoiseAt(7, 3);

  EXPECT_TRUE(std::fabs(a.vx - b.vx) > 1e-9 || std::fabs(a.wz - b.wz) > 1e-9);
}

TEST(MppiControllerTest, ConfigClampsInvalidSizes)
{
  MppiConfig config;
  config.batch_size = 0;
  config.time_steps = 0;
  config.model_dt = 0.0;
  config.temperature = 0.0;

  MppiController controller(config);

  EXPECT_EQ(controller.config().batch_size, 1u);
  EXPECT_EQ(controller.config().time_steps, 1u);
  EXPECT_GT(controller.config().model_dt, 0.0);
  EXPECT_GT(controller.config().temperature, 0.0);
}

TEST(MppiControllerTest, StructuredMinimumSpeedClampsToConfiguredVelocityBounds)
{
  MppiConfig below;
  below.vx_min = 0.05;
  below.vx_max = 0.5;
  below.structured_min_vx = -1.0;
  EXPECT_DOUBLE_EQ(MppiController(below).config().structured_min_vx, below.vx_min);

  MppiConfig above = below;
  above.structured_min_vx = 1.0;
  EXPECT_DOUBLE_EQ(MppiController(above).config().structured_min_vx, above.vx_max);
}

TEST(MppiControllerTest, DeployedStructuredTemplatesFromRestProduceClassifiableMirroredBypasses)
{
  MppiConfig config;
  config.batch_size = 8;
  config.structured_candidate_count = 8;
  config.time_steps = 75;
  config.model_dt = 0.0666667;
  config.controller_period = 0.0666667;
  config.shift_control_sequence = true;
  config.vx_min = 0.05;
  config.vx_max = 0.5;
  config.structured_min_vx = 0.2;
  config.wz_max = 0.526;
  config.ax_min = -3.0;
  config.ax_max = 3.0;
  config.az_max = 3.5;
  config.wheelbase = 0.46;
  config.max_steer = 0.238;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  std::vector<MppiCandidate> captured;
  controller.optimizeBatch(
    MppiControl{0.0, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      captured = candidates;
      return std::vector<MppiCandidateScore>(
        candidates.size(), MppiCandidateScore{true, 0.0});
    });

  ASSERT_EQ(captured.size(), config.batch_size);
  const std::vector<MppiPathPoint> straight_path{
    MppiPathPoint{0.0, 0.0, 0.0}, MppiPathPoint{10.0, 0.0, 0.0}};
  const auto left = integrateWithMidpointModel(captured[2].controls, config.model_dt);
  const auto right = integrateWithMidpointModel(captured[3].controls, config.model_dt);
  ASSERT_EQ(left.points.size(), config.time_steps);
  ASSERT_EQ(right.points.size(), config.time_steps);
  EXPECT_EQ(
    local_planner::classifyMppiBypassSide(left, straight_path),
    MppiBypassSide::Left) << "terminal_y=" << left.points.back().y;
  EXPECT_EQ(
    local_planner::classifyMppiBypassSide(right, straight_path),
    MppiBypassSide::Right) << "terminal_y=" << right.points.back().y;
  EXPECT_GT(left.points.back().y, 0.10);
  EXPECT_LT(right.points.back().y, -0.10);

  for(const auto & candidate : captured){
    ASSERT_EQ(candidate.controls.size(), config.time_steps);
    EXPECT_DOUBLE_EQ(candidate.controls.front().vx, 0.0);
    MppiControl previous{0.0, 0.0};
    for(std::size_t step = 1; step < candidate.controls.size(); ++step){
      const auto & control = candidate.controls[step];
      EXPECT_GE(control.vx, config.vx_min);
      EXPECT_LE(control.vx, config.vx_max);
      EXPECT_GE(control.vx, previous.vx + config.ax_min * config.model_dt - 1e-12);
      EXPECT_LE(control.vx, previous.vx + config.ax_max * config.model_dt + 1e-12);
      EXPECT_LE(std::fabs(control.wz - previous.wz), config.az_max * config.model_dt + 1e-12);
      const double ackermann_limit = std::min(
        config.wz_max,
        control.vx * std::tan(config.max_steer) / config.wheelbase);
      EXPECT_LE(std::fabs(control.wz), ackermann_limit + 1e-12);
      previous = control;
    }
  }
}

TEST(MppiControllerTest, SpeedCapAllowsPhysicalDecelerationFromMeasuredSpeed)
{
  const std::vector<MppiControl> decelerating{
    MppiControl{0.5, 0.0},
    MppiControl{0.3, 0.0},
    MppiControl{0.2, 0.0},
    MppiControl{0.2, 0.0}};
  const std::vector<MppiControl> not_decelerating{
    MppiControl{0.5, 0.0},
    MppiControl{0.5, 0.0},
    MppiControl{0.2, 0.0}};
  const std::vector<MppiControl> exceeds_after_reaching_cap{
    MppiControl{0.5, 0.0},
    MppiControl{0.2, 0.0},
    MppiControl{0.25, 0.0}};
  const std::vector<MppiControl> decelerates_too_slowly{
    MppiControl{0.5, 0.0},
    MppiControl{0.49, 0.0},
    MppiControl{0.48, 0.0},
    MppiControl{0.47, 0.0}};

  EXPECT_TRUE(mppiSequenceRespectsSpeedCap(
    decelerating, 0.5, 0.2, true, -2.0, 0.1, 0.1));
  EXPECT_FALSE(mppiSequenceRespectsSpeedCap(
    not_decelerating, 0.5, 0.2, true, -2.0, 0.1, 0.1));
  EXPECT_FALSE(mppiSequenceRespectsSpeedCap(
    exceeds_after_reaching_cap, 0.5, 0.2, true, -2.0, 0.1, 0.1));
  EXPECT_FALSE(mppiSequenceRespectsSpeedCap(
    decelerates_too_slowly, 0.5, 0.2, true, -2.0, 0.1, 0.1));
}

TEST(MppiControllerTest, CandidateControlsRespectAccelerationAndAckermannLimits)
{
  MppiConfig config;
  config.batch_size = 32;
  config.time_steps = 10;
  config.model_dt = 0.1;
  config.controller_period = 0.1;
  config.vx_min = 0.05;
  config.vx_max = 0.8;
  config.wz_max = 2.0;
  config.ax_min = 0.0;
  config.ax_max = 0.5;
  config.az_max = 0.5;
  config.wheelbase = 0.55;
  config.max_steer = 0.69;
  config.vx_std = 10.0;
  config.wz_std = 10.0;

  MppiController controller(config);
  MppiControl measured_speed{0.20, 0.0};

  const auto result = controller.optimize(
    measured_speed,
    [&](const MppiCandidate & candidate) {
      MppiControl previous = measured_speed;
      for(const auto & control : candidate.controls){
        EXPECT_GE(control.vx, config.vx_min);
        EXPECT_LE(control.vx, config.vx_max);
        EXPECT_LE(control.vx, previous.vx + config.ax_max * config.model_dt + 1e-9);
        EXPECT_GE(control.vx, previous.vx + config.ax_min * config.model_dt - 1e-9);
        EXPECT_LE(std::fabs(control.wz - previous.wz), config.az_max * config.model_dt + 1e-9);
        const double steering_wz = control.vx * std::tan(config.max_steer) / config.wheelbase;
        EXPECT_LE(std::fabs(control.wz), steering_wz + 1e-9);
        previous = control;
      }
      return MppiCandidateScore{true, std::fabs(candidate.controls.front().wz)};
    });

  EXPECT_TRUE(result.has_valid_control);
}

TEST(MppiControllerTest, StructuredCandidatesProvideMirroredArcsAndSCurves)
{
  MppiConfig config;
  config.batch_size = 8;
  config.structured_candidate_count = 8;
  config.time_steps = 12;
  config.model_dt = 0.1;
  config.controller_period = 0.1;
  config.shift_control_sequence = false;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.46;
  config.max_steer = 0.238;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  std::vector<MppiCandidate> captured;
  controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      captured = candidates;
      return std::vector<MppiCandidateScore>(
        candidates.size(), MppiCandidateScore{true, 0.0});
    });

  ASSERT_EQ(captured.size(), 8u);
  for(std::size_t pair = 0; pair < captured.size(); pair += 2){
    ASSERT_EQ(captured[pair].controls.size(), config.time_steps);
    ASSERT_EQ(captured[pair + 1].controls.size(), config.time_steps);
    for(std::size_t step = 0; step < config.time_steps; ++step){
      const auto & left = captured[pair].controls[step];
      const auto & right = captured[pair + 1].controls[step];
      EXPECT_NEAR(left.vx, right.vx, 1e-12);
      EXPECT_NEAR(left.wz, -right.wz, 1e-12);
      const double limit = left.vx * std::tan(config.max_steer) / config.wheelbase;
      EXPECT_LE(std::fabs(left.wz), limit + 1e-12);
      EXPECT_LE(std::fabs(right.wz), limit + 1e-12);
    }
  }

  EXPECT_GT(captured[0].controls.front().wz, 0.0);
  EXPECT_GT(captured[0].controls.back().wz, 0.0);
  EXPECT_GT(captured[4].controls.front().wz, 0.0);
  EXPECT_LT(captured[4].controls.back().wz, 0.0);
}

TEST(MppiControllerTest, LatchedStructuredSCurveAdvancesToCountersteerAndStraightTail)
{
  expectLatchedStructuredSCurveCompletes(MppiBypassSide::Left, 4u);
}

TEST(MppiControllerTest, LatchedRightStructuredSCurveMirrorsPersistentManeuver)
{
  expectLatchedStructuredSCurveCompletes(MppiBypassSide::Right, 5u);
}

TEST(MppiControllerTest, MultiIterationAdvancesStructuredPhaseOncePerBatch)
{
  auto config = makeStructuredPhaseConfig();
  config.iteration_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  std::size_t score_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      ++score_calls;
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 100.0, MppiBypassSide::Right, true});
      scores[4] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
      return scores;
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.structured_phase_step, 1u);
}

TEST(MppiControllerTest, StructuredWinnerReportsUnitMassAndAdvancesPhase)
{
  auto config = makeStructuredPhaseConfig();
  config.batch_size = 4;
  config.structured_candidate_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      scores[0] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
      return scores;
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_DOUBLE_EQ(result.structured_weight_mass, 1.0);
  EXPECT_EQ(result.structured_phase_step, 1u);
}

TEST(MppiControllerTest, WrongPhaseRandomCandidateDoesNotBypassStructuredCommitment)
{
  auto config = makeStructuredPhaseConfig();
  config.batch_size = 4;
  config.structured_candidate_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      scores[2] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
      return scores;
    },
    []() {return true;});

  EXPECT_FALSE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.phase_compatible_candidates, 0u);
  EXPECT_EQ(result.structured_phase_step, 0u);
}

TEST(MppiControllerTest, DeploymentScaleDirectionalAggregationRejectsWrongCurrentPhase)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 96;
  config.structured_candidate_count = 8;
  config.time_steps = 4;
  config.shift_control_sequence = false;
  config.wz_max = 0.4;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  std::vector<std::vector<MppiControl>> noises(
    config.batch_size,
    std::vector<MppiControl>(config.time_steps));
  for(std::size_t sample = 0; sample < noises.size(); ++sample){
    std::fill(
      noises[sample].begin(),
      noises[sample].end(),
      MppiControl{0.0, sample < config.batch_size / 2 ? -0.10 : 0.10});
  }
  controller.setDebugNoiseForTest(noises);

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      return std::vector<MppiCandidateScore>(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(result.phase_compatible_candidates, config.batch_size / 2);
  EXPECT_NEAR(result.prefilter_command.wz, -0.10, 1e-12);
}

TEST(MppiControllerTest, ExecutedRandomCandidateAdvancesStructuredPhase)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 1;
  config.structured_candidate_count = 0;
  config.time_steps = 6;
  config.shift_control_sequence = false;
  config.wz_max = 0.4;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, -0.10})});

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      return std::vector<MppiCandidateScore>(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_DOUBLE_EQ(result.structured_weight_mass, 0.0);
  EXPECT_LT(result.command.wz, 0.0);
  EXPECT_EQ(result.structured_phase_step, 1u);
}

TEST(MppiControllerTest, InactiveRebuildDiscardsStructuredMassWithoutLeavingStaleDiagnostic)
{
  auto config = makeStructuredPhaseConfig();
  config.batch_size = 4;
  config.structured_candidate_count = 2;
  config.iteration_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto latched = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      scores[0] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(latched.has_valid_control);
  ASSERT_EQ(latched.selected_bypass_side, MppiBypassSide::Left);
  ASSERT_EQ(latched.structured_phase_step, 1u);

  std::size_t score_calls = 0;
  const auto inactive = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      ++score_calls;
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      const std::size_t winner = score_calls == 1 ? 0u : 2u;
      scores[winner] = MppiCandidateScore{
        true, 0.0, MppiBypassSide::Neutral, true};
      return scores;
    },
    []() {return false;});

  ASSERT_TRUE(inactive.has_valid_control);
  EXPECT_EQ(score_calls, 3u);
  EXPECT_EQ(inactive.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_DOUBLE_EQ(inactive.structured_weight_mass, 0.0);
  EXPECT_EQ(inactive.structured_phase_step, 0u);
}

TEST(MppiControllerTest, InvalidBatchPreservesStructuredPhaseAndEntrySequence)
{
  auto config = makeStructuredPhaseConfig();
  config.iteration_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  const auto initial = latchLeftStructured(controller);
  ASSERT_TRUE(initial.has_valid_control);
  ASSERT_EQ(initial.structured_phase_step, 1u);
  const auto entry_sequence = controller.controlSequence();

  std::size_t score_calls = 0;
  const auto invalid = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      if(score_calls++ == 0){
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
      }
      return std::vector<MppiCandidateScore>(
        candidates.size(), MppiCandidateScore{false, -1.0});
    },
    []() {return true;});

  ASSERT_FALSE(invalid.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_DOUBLE_EQ(invalid.structured_weight_mass, 0.0);
  EXPECT_EQ(invalid.structured_phase_step, 1u);
  ASSERT_EQ(controller.controlSequence().size(), entry_sequence.size());
  for(std::size_t step = 0; step < entry_sequence.size(); ++step){
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].vx, entry_sequence[step].vx);
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].wz, entry_sequence[step].wz);
  }

  const auto missing_score = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, MppiController::BatchScoreFunction{});
  EXPECT_FALSE(missing_score.has_valid_control);
  EXPECT_DOUBLE_EQ(missing_score.structured_weight_mass, 0.0);
  EXPECT_EQ(missing_score.structured_phase_step, 1u);
}

TEST(MppiControllerTest, InactiveModeResetAndSetConfigClearStructuredPhase)
{
  const auto config = makeStructuredPhaseConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  ASSERT_EQ(latchLeftStructured(controller).structured_phase_step, 1u);
  const auto inactive = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      return std::vector<MppiCandidateScore>(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, true});
    },
    []() {return false;});
  ASSERT_TRUE(inactive.has_valid_control);
  EXPECT_EQ(inactive.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_EQ(inactive.structured_phase_step, 0u);

  ASSERT_EQ(latchLeftStructured(controller).structured_phase_step, 1u);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  const auto after_reset = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, MppiController::BatchScoreFunction{});
  EXPECT_EQ(after_reset.structured_phase_step, 0u);

  ASSERT_EQ(latchLeftStructured(controller).structured_phase_step, 1u);
  controller.setConfig(config);
  const auto after_config = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, MppiController::BatchScoreFunction{});
  EXPECT_EQ(after_config.structured_phase_step, 0u);
}

TEST(MppiControllerTest, LatchedStructuredCandidatesStayOnSideAndRespectLimits)
{
  auto config = makeStructuredPhaseConfig();
  config.vx_min = 0.05;
  config.vx_max = 0.8;
  config.ax_min = -0.3;
  config.ax_max = 0.5;
  config.az_max = 0.2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  ASSERT_TRUE(latchLeftStructured(controller).has_valid_control);

  std::vector<MppiCandidate> captured;
  const MppiControl measured_speed{0.2, 0.0};
  const auto result = controller.optimizeBatch(
    measured_speed,
    [&](const std::vector<MppiCandidate> & candidates) {
      captured = candidates;
      return std::vector<MppiCandidateScore>(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.structured_candidates, 2u);
  ASSERT_EQ(captured.size(), config.batch_size);
  for(std::size_t sample = 0; sample < captured.size(); ++sample){
    const auto & candidate = captured[sample];
    MppiControl previous = measured_speed;
    ASSERT_EQ(candidate.controls.size(), config.time_steps);
    if(sample < result.structured_candidates){
      EXPECT_GT(candidate.controls.front().wz, 0.0);
    }
    for(std::size_t step = 0; step < candidate.controls.size(); ++step){
      const auto & control = candidate.controls[step];
      const double dt = step == 0 ? config.controller_period : config.model_dt;
      EXPECT_LE(control.vx, previous.vx + config.ax_max * dt + 1e-12);
      EXPECT_GE(control.vx, previous.vx + config.ax_min * dt - 1e-12);
      EXPECT_LE(std::fabs(control.wz - previous.wz), config.az_max * dt + 1e-12);
      const double ackermann_limit = std::min(
        config.wz_max,
        std::fabs(control.vx) * std::tan(config.max_steer) / config.wheelbase);
      EXPECT_LE(std::fabs(control.wz), ackermann_limit + 1e-12);
      previous = control;
    }
  }
}

TEST(MppiControllerTest, ConfirmedSideSwitchRestartsStructuredPhaseForNewSide)
{
  const auto config = makeStructuredPhaseConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  ASSERT_EQ(latchLeftStructured(controller).structured_phase_step, 1u);

  for(std::size_t cycle = 1; cycle < config.time_steps; ++cycle){
    const auto advanced = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [](const std::vector<MppiCandidate> & candidates) {
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
      },
      []() {return true;});
    ASSERT_TRUE(advanced.has_valid_control);
  }

  const auto force_right = [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
      scores.front() = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, false};
      return scores;
    };
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, 0.0}, force_right, []() {return true;})
    .has_valid_control);
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, 0.0}, force_right, []() {return true;})
    .has_valid_control);
  const auto switched = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, force_right, []() {return true;});

  ASSERT_TRUE(switched.has_valid_control);
  EXPECT_EQ(switched.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_LT(switched.structured_weight_mass, 0.5);
  EXPECT_EQ(switched.structured_phase_step, 0u);

  const auto new_side_turn = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      scores[0] = MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true};
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(new_side_turn.has_valid_control);
  EXPECT_EQ(new_side_turn.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_DOUBLE_EQ(new_side_turn.structured_weight_mass, 1.0);
  EXPECT_EQ(new_side_turn.structured_phase_step, 1u);
  EXPECT_LT(new_side_turn.command.wz, 0.0);
}

TEST(MppiControllerTest, InvalidLaterIterationResetsPhaseWhenSideSwitchCommits)
{
  auto config = makeStructuredPhaseConfig();
  config.iteration_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  ASSERT_EQ(latchLeftStructured(controller).structured_phase_step, 1u);

  for(std::size_t cycle = 1; cycle < config.time_steps; ++cycle){
    const auto advanced = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [](const std::vector<MppiCandidate> & candidates) {
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
      },
      []() {return true;});
    ASSERT_TRUE(advanced.has_valid_control);
  }

  const auto force_right = [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(),
        MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true});
      scores.front() = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, false};
      return scores;
    };
  const auto first_infeasible = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, force_right, []() {return true;});
  const auto second_infeasible = controller.optimizeBatch(
    MppiControl{0.2, 0.0}, force_right, []() {return true;});
  ASSERT_FALSE(first_infeasible.has_valid_control);
  ASSERT_EQ(first_infeasible.side_infeasible_cycles, 1u);
  ASSERT_FALSE(second_infeasible.has_valid_control);
  ASSERT_EQ(second_infeasible.side_infeasible_cycles, 2u);

  std::size_t score_calls = 0;
  const auto invalid_switch = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      if(score_calls++ == 0){
        return force_right(candidates);
      }
      return std::vector<MppiCandidateScore>(
        candidates.size(), MppiCandidateScore{false, -1.0});
    },
    []() {return true;});

  ASSERT_FALSE(invalid_switch.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(invalid_switch.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(invalid_switch.side_infeasible_cycles, 0u);
  EXPECT_EQ(invalid_switch.structured_phase_step, 0u);

  const auto right_turn = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      scores[0] = MppiCandidateScore{true, 0.0, MppiBypassSide::Right, true};
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(right_turn.has_valid_control);
  EXPECT_EQ(right_turn.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_DOUBLE_EQ(right_turn.structured_weight_mass, 1.0);
  EXPECT_EQ(right_turn.structured_phase_step, 1u);
  EXPECT_LT(right_turn.command.wz, 0.0);
}

TEST(MppiControllerTest, InactiveTransitionRebuildsMirroredCandidatesBeforeAggregation)
{
  auto config = makeStructuredPhaseConfig();
  config.iteration_count = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  ASSERT_TRUE(latchLeftStructured(controller).has_valid_control);

  std::vector<std::vector<MppiCandidate>> scored_batches;
  std::size_t mode_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      scored_batches.push_back(candidates);
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{false, -1.0});
      if(scored_batches.size() == 1){
        std::fill(
          scores.begin(), scores.end(),
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
      }
      else{
        scores[1] = MppiCandidateScore{
          true,
          scored_batches.size() == 2 ? 2.0 : 1.0,
          MppiBypassSide::Neutral,
          true};
      }
      return scores;
    },
    [&]() {
      ++mode_calls;
      return false;
    });

  ASSERT_TRUE(result.has_valid_control);
  ASSERT_EQ(scored_batches.size(), 3u);
  EXPECT_EQ(mode_calls, 1u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_EQ(result.structured_phase_step, 0u);
  EXPECT_EQ(result.structured_candidates, config.structured_candidate_count);
  EXPECT_EQ(result.best_sample_side, MppiBypassSide::Neutral);
  EXPECT_EQ(result.best_sample_index, 1u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 1.0);
  EXPECT_EQ(result.feasible_left_candidates, 0u);
  EXPECT_EQ(result.feasible_right_candidates, 0u);
  EXPECT_EQ(result.feasible_neutral_candidates, 1u);
  EXPECT_GT(
    scored_batches[0][0].controls.front().wz *
    scored_batches[0][1].controls.front().wz,
    0.0);
  for(std::size_t batch = 1; batch < scored_batches.size(); ++batch){
    for(std::size_t pair = 0; pair < config.structured_candidate_count; pair += 2){
      for(std::size_t step = 0; step < config.time_steps; ++step){
        EXPECT_NEAR(
          scored_batches[batch][pair].controls[step].wz,
          -scored_batches[batch][pair + 1].controls[step].wz,
          1e-12);
      }
    }
  }
  EXPECT_NEAR(result.command.wz, scored_batches[2][1].controls.front().wz, 1e-12);
}

TEST(MppiControllerTest, LatchedStructuredCandidateCountUsesOnlyTwoSlots)
{
  for(const std::size_t configured_count : {0u, 2u, 4u, 6u, 8u}){
    SCOPED_TRACE(configured_count);
    auto config = makeStructuredPhaseConfig();
    config.structured_candidate_count = configured_count;
    config.vx_std = 0.05;
    config.wz_std = 0.3;
    MppiController controller(config);
    controller.resetControlSequence(MppiControl{0.2, 0.0});

    const auto initial = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [](const std::vector<MppiCandidate> & candidates) {
        std::vector<MppiCandidateScore> scores(
          candidates.size(),
          MppiCandidateScore{true, 100.0, MppiBypassSide::Right, true});
        scores[0] = MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
        return scores;
      },
      []() {return true;});
    ASSERT_TRUE(initial.has_valid_control);
    EXPECT_EQ(initial.structured_candidates, configured_count);

    std::vector<MppiCandidate> captured;
    const auto active = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [&](const std::vector<MppiCandidate> & candidates) {
        captured = candidates;
        return std::vector<MppiCandidateScore>(
          candidates.size(),
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true});
      },
      []() {return true;});

    ASSERT_TRUE(active.has_valid_control);
    const std::size_t expected_active_count = std::min<std::size_t>(2, configured_count);
    EXPECT_EQ(active.structured_candidates, expected_active_count);
    if(configured_count > 2){
      bool third_differs_from_structured = false;
      for(std::size_t step = 0; step < config.time_steps; ++step){
        third_differs_from_structured = third_differs_from_structured ||
          std::fabs(captured[2].controls[step].vx - captured[0].controls[step].vx) > 1e-9 ||
          std::fabs(captured[2].controls[step].wz - captured[0].controls[step].wz) > 1e-9;
      }
      EXPECT_TRUE(third_differs_from_structured);
    }
  }
}

TEST(MppiControllerTest, ShiftedOddStructuredHorizonHasFinitePhaseBoundaries)
{
  auto config = makeStructuredPhaseConfig();
  config.time_steps = 6;
  config.shift_control_sequence = true;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});

  for(std::size_t cycle = 0; cycle < 7; ++cycle){
    const auto result = controller.optimizeBatch(
      MppiControl{0.2, 0.0},
      [cycle](const std::vector<MppiCandidate> & candidates) {
        std::vector<MppiCandidateScore> scores(
          candidates.size(),
          MppiCandidateScore{true, 100.0, MppiBypassSide::Left, true});
        scores[cycle == 0 ? 4u : 0u].cost = 0.0;
        return scores;
      },
      []() {return true;});
    ASSERT_TRUE(result.has_valid_control);
    EXPECT_EQ(result.structured_phase_step, std::min<std::size_t>(cycle + 1, 5u));
    if(cycle < 2){
      EXPECT_GT(result.command.wz, 0.0);
    }
    else if(cycle < 5){
      EXPECT_LT(result.command.wz, 0.0);
    }
    else{
      EXPECT_NEAR(result.command.wz, 0.0, 1e-12);
    }
  }
}

TEST(MppiControllerTest, ShiftedTinyStructuredHorizonsDoNotUnderflow)
{
  for(const std::size_t time_steps : {1u, 2u}){
    SCOPED_TRACE(time_steps);
    auto config = makeStructuredPhaseConfig();
    config.time_steps = time_steps;
    config.shift_control_sequence = true;
    config.az_max = 0.2;
    MppiController controller(config);
    controller.resetControlSequence(MppiControl{0.2, 0.0});

    for(std::size_t cycle = 0; cycle < 3; ++cycle){
      const auto result = controller.optimizeBatch(
        MppiControl{0.2, 0.0},
        [cycle](const std::vector<MppiCandidate> & candidates) {
          std::vector<MppiCandidateScore> scores(
            candidates.size(),
            MppiCandidateScore{true, 100.0, MppiBypassSide::Right, true});
          scores[cycle == 0 ? 4u : 0u] =
            MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true};
          return scores;
        },
        []() {return true;});
      ASSERT_TRUE(result.has_valid_control);
      const std::size_t controlled_steps = time_steps - 1;
      EXPECT_EQ(result.structured_phase_step, controlled_steps);
      const double ackermann_limit = std::min(
        config.wz_max,
        std::fabs(result.command.vx) * std::tan(config.max_steer) / config.wheelbase);
      EXPECT_LE(std::fabs(result.command.wz), ackermann_limit + 1e-12);
      if(time_steps == 1 || cycle > 0){
        EXPECT_NEAR(result.command.wz, 0.0, 1e-12);
      }
    }
  }
}

TEST(MppiControllerTest, SavitzkyGolayFilterReducesAlternatingAngularCommands)
{
  MppiConfig config;
  config.batch_size = 1;
  config.structured_candidate_count = 0;
  config.time_steps = 25;
  config.shift_control_sequence = false;
  config.model_dt = 0.1;
  config.controller_period = 0.1;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  std::vector<MppiControl> alternating(config.time_steps, MppiControl{0.0, 0.0});
  for(std::size_t i = 0; i < alternating.size(); ++i){
    alternating[i].wz = i % 2 == 0 ? 0.2 : -0.2;
  }

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({alternating});
  const auto result = controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 0.0};});

  ASSERT_TRUE(result.has_valid_control);
  double total_variation = 0.0;
  for(std::size_t i = 1; i < result.optimal_sequence.size(); ++i){
    total_variation += std::fabs(
      result.optimal_sequence[i].wz - result.optimal_sequence[i - 1].wz);
    EXPECT_TRUE(std::isfinite(result.optimal_sequence[i].wz));
  }
  EXPECT_LT(total_variation, 4.0);
}

TEST(MppiControllerTest, WarmStartPreservesSelectedDirectionWhenLaterCostsAreSymmetric)
{
  MppiConfig config;
  config.batch_size = 16;
  config.structured_candidate_count = 8;
  config.time_steps = 25;
  config.shift_control_sequence = false;
  config.temperature = 0.02;
  config.gamma = 0.0;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.46;
  config.max_steer = 0.238;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  const auto selected = controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate & candidate) {
      const double wz = candidate.controls.front().wz;
      return MppiCandidateScore{true, wz > 1e-4 ? 0.0 : (wz < -1e-4 ? 0.1 : 0.05)};
    });
  ASSERT_TRUE(selected.has_valid_control);
  ASSERT_GT(selected.command.wz, 0.0);

  const auto symmetric = controller.optimize(
    MppiControl{0.2, selected.command.wz},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 0.0};});

  ASSERT_TRUE(symmetric.has_valid_control);
  EXPECT_GT(symmetric.command.wz, 0.0);
}

TEST(MppiControllerTest, PhysicalLeftCandidateCanSelectNegativeAngularCommand)
{
  const auto config = makeBypassLatchConfig();

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, -0.3, 0.3));
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.side_infeasible_cycles, 0u);
  EXPECT_LT(result.command.wz, 0.0);
}

TEST(MppiControllerTest, LatchedSideDoesNotSwitchForOneOppositeCostAdvantage)
{
  const auto config = makeBypassLatchConfig();

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);
  ASSERT_EQ(initial.selected_bypass_side, MppiBypassSide::Left);

  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.0, 0.0));
  const auto advantaged_right = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(1.0, true, 0.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(advantaged_right.has_valid_control);
  EXPECT_EQ(advantaged_right.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(advantaged_right.side_infeasible_cycles, 0u);
}

TEST(MppiControllerTest, LatchedFeasibleSideSurvivesOppositeSoftmaxUnderflow)
{
  const auto config = makeBypassLatchConfig();

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_EQ(initial.selected_bypass_side, MppiBypassSide::Left);

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(1000.0, true, 0.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_GT(result.command.wz, 0.0);
}

TEST(MppiControllerTest, InitialSideMassIgnoresNeutralSoftmaxUnderflow)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 4;
  config.temperature = 1.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.0}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, -0.3}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, -0.1}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.3})});

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>{
        MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, true},
        MppiCandidateScore{true, 1000.1, MppiBypassSide::Left, true},
        MppiCandidateScore{true, 1000.1, MppiBypassSide::Left, true},
        MppiCandidateScore{true, 1000.0, MppiBypassSide::Right, true}};
    },
    []() {return true;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_LT(result.command.wz, 0.0);
}

TEST(MppiControllerTest, LatchedSideReturnsInvalidForFirstTwoInfeasibleCycles)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);

  const auto blocked_scores = [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    };
  const auto first = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz}, blocked_scores, []() {return true;});
  const auto second = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz}, blocked_scores, []() {return true;});

  EXPECT_FALSE(first.has_valid_control);
  EXPECT_EQ(first.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(first.side_infeasible_cycles, 1u);
  EXPECT_NEAR(first.effective_sample_size, 2.0, 1e-12);
  EXPECT_FALSE(second.has_valid_control);
  EXPECT_EQ(second.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(second.side_infeasible_cycles, 2u);
  EXPECT_NEAR(second.effective_sample_size, 2.0, 1e-12);
}

TEST(MppiControllerTest, LatchedSideSwitchesOnThirdInfeasibleCycle)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);

  const auto blocked_scores = [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    };
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, initial.command.wz}, blocked_scores, []() {return true;})
    .has_valid_control);
  EXPECT_FALSE(controller.optimizeBatch(
      MppiControl{0.2, initial.command.wz}, blocked_scores, []() {return true;})
    .has_valid_control);
  const auto switched = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz}, blocked_scores, []() {return true;});

  ASSERT_TRUE(switched.has_valid_control);
  EXPECT_EQ(switched.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(switched.side_infeasible_cycles, 0u);
}

TEST(MppiControllerTest, LatchedSideRemainsInvalidWhenBothSidesAreUnavailable)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);

  for(std::size_t cycle = 1; cycle <= 4; ++cycle){
    const auto blocked = controller.optimizeBatch(
      MppiControl{0.2, initial.command.wz},
      [](const std::vector<MppiCandidate> &) {
        return makeSideScores(0.0, false, 0.0, false);
      },
      []() {return true;});
    EXPECT_FALSE(blocked.has_valid_control);
    EXPECT_EQ(blocked.selected_bypass_side, MppiBypassSide::Left);
    EXPECT_EQ(blocked.side_infeasible_cycles, std::min<std::size_t>(cycle, 3u));
  }
}

TEST(MppiControllerTest, NonDirectionalModeAndResetClearPhysicalSideLatch)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);
  const auto blocked = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    },
    []() {return true;});
  ASSERT_FALSE(blocked.has_valid_control);
  ASSERT_EQ(blocked.side_infeasible_cycles, 1u);

  controller.setDebugNoiseForTest(makeAngularNoises(config, -0.1, 0.1));
  const auto normal = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>{
        MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, false},
        MppiCandidateScore{true, 1.0, MppiBypassSide::Neutral, false}};
    },
    []() {return false;});
  ASSERT_TRUE(normal.has_valid_control);
  EXPECT_EQ(normal.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_EQ(normal.side_infeasible_cycles, 0u);

  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.0, 0.0));
  const auto relatched = controller.optimizeBatch(
    MppiControl{0.2, normal.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_EQ(relatched.selected_bypass_side, MppiBypassSide::Left);

  controller.resetControlSequence(MppiControl{0.2, 0.0});
  const auto after_reset = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(after_reset.has_valid_control);
  EXPECT_EQ(after_reset.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(after_reset.side_infeasible_cycles, 0u);
}

TEST(MppiControllerTest, DirectionalModeRejectsNeutralOnlyCandidatesWithoutLatch)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, -0.2, 0.2));

  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>(
        2, MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, true});
    },
    []() {return true;});

  EXPECT_FALSE(result.has_valid_control);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_EQ(result.side_infeasible_cycles, 0u);
  EXPECT_NEAR(result.effective_sample_size, 2.0, 1e-12);
}

TEST(MppiControllerTest, MultiIterationInvalidResultRollsBackSequenceAndNewLatchFailure)
{
  auto config = makeBypassLatchConfig();
  config.iteration_count = 2;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  const auto entry_sequence = controller.controlSequence();
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.1, 0.3));
  std::size_t score_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [&](const std::vector<MppiCandidate> &) {
      if(score_calls++ == 0){
        return std::vector<MppiCandidateScore>{
          MppiCandidateScore{true, 1.0, MppiBypassSide::Left, true},
          MppiCandidateScore{true, 0.0, MppiBypassSide::Left, true}};
      }
      return std::vector<MppiCandidateScore>(
        2, MppiCandidateScore{false, -1.0});
    },
    []() {return true;});

  ASSERT_FALSE(result.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(result.best_sample_index, 0u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_EQ(result.side_infeasible_cycles, 0u);
  ASSERT_EQ(controller.controlSequence().size(), entry_sequence.size());
  for(std::size_t step = 0; step < entry_sequence.size(); ++step){
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].vx, entry_sequence[step].vx);
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].wz, entry_sequence[step].wz);
  }
}

TEST(MppiControllerTest, MultiIterationSwitchSettlesSideOnlyOnceOnLaterInvalid)
{
  auto config = makeBypassLatchConfig();
  config.iteration_count = 2;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_TRUE(initial.has_valid_control);
  ASSERT_EQ(initial.selected_bypass_side, MppiBypassSide::Left);

  const auto left_blocked = [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    };
  const auto first_failure = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz}, left_blocked, []() {return true;});
  const auto second_failure = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz}, left_blocked, []() {return true;});
  ASSERT_FALSE(first_failure.has_valid_control);
  ASSERT_EQ(first_failure.side_infeasible_cycles, 1u);
  ASSERT_FALSE(second_failure.has_valid_control);
  ASSERT_EQ(second_failure.side_infeasible_cycles, 2u);

  const auto entry_sequence = controller.controlSequence();
  std::size_t score_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [&](const std::vector<MppiCandidate> &) {
      if(score_calls++ == 0){
        return makeSideScores(0.0, false, 0.0, true);
      }
      return makeSideScores(0.0, true, 0.0, false);
    },
    []() {return true;});

  ASSERT_FALSE(result.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(result.side_infeasible_cycles, 0u);
  ASSERT_EQ(controller.controlSequence().size(), entry_sequence.size());
  for(std::size_t step = 0; step < entry_sequence.size(); ++step){
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].vx, entry_sequence[step].vx);
    EXPECT_DOUBLE_EQ(controller.controlSequence()[step].wz, entry_sequence[step].wz);
  }
}

TEST(MppiControllerTest, MultiIterationFreezesInitialDirectionalModeWhenCallbackTurnsFalse)
{
  auto config = makeBypassLatchConfig();
  config.iteration_count = 2;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.3, -0.3));
  std::size_t mode_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 0.0, true);
    },
    [&]() {return mode_calls++ == 0;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(mode_calls, 1u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_NEAR(result.command.wz, 0.6, 1e-12);
}

TEST(MppiControllerTest, MultiIterationFreezesInitialNormalModeWhenCallbackTurnsTrue)
{
  auto config = makeBypassLatchConfig();
  config.iteration_count = 2;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.3, -0.3));
  std::size_t mode_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 0.0, true);
    },
    [&]() {return mode_calls++ != 0;});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(mode_calls, 1u);
  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Neutral);
  EXPECT_NEAR(result.command.wz, 0.0, 1e-12);
}

TEST(MppiControllerTest, SetConfigClearsPhysicalSideLatch)
{
  const auto config = makeBypassLatchConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto initial = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, true, 1.0, true);
    },
    []() {return true;});
  ASSERT_EQ(initial.selected_bypass_side, MppiBypassSide::Left);
  const auto blocked = controller.optimizeBatch(
    MppiControl{0.2, initial.command.wz},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    },
    []() {return true;});
  ASSERT_FALSE(blocked.has_valid_control);
  ASSERT_EQ(blocked.side_infeasible_cycles, 1u);

  controller.setConfig(config);
  controller.setDebugNoiseForTest(makeAngularNoises(config, 0.2, -0.2));
  const auto after_reconfigure = controller.optimizeBatch(
    MppiControl{0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return makeSideScores(0.0, false, 0.0, true);
    },
    []() {return true;});

  ASSERT_TRUE(after_reconfigure.has_valid_control);
  EXPECT_EQ(after_reconfigure.selected_bypass_side, MppiBypassSide::Right);
  EXPECT_EQ(after_reconfigure.side_infeasible_cycles, 0u);
}

TEST(MppiControllerTest, SoftmaxUpdateUsesMoreThanBestSample)
{
  MppiConfig config;
  config.batch_size = 3;
  config.time_steps = 4;
  config.temperature = 10.0;
  config.vx_min = 0.0;
  config.vx_max = 1.0;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;
  config.shift_control_sequence = false;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(config.time_steps, MppiControl{-0.2, 0.0}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.0}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.2, 0.0})
  });

  const auto result = controller.optimize(
    MppiControl{0.4, 0.0},
    [](const MppiCandidate & candidate) {
      const double vx = candidate.controls.front().vx;
      const double cost = std::fabs(vx - 0.6);
      return MppiCandidateScore{true, cost};
    });

  EXPECT_TRUE(result.has_valid_control);
  EXPECT_GT(result.optimal_sequence.front().vx, 0.4);
  EXPECT_LT(result.optimal_sequence.front().vx, 0.6);
}

TEST(MppiControllerTest, ReportsSoftmaxDirectionalWeightMassAndEffectiveSampleSize)
{
  MppiConfig config;
  config.batch_size = 3;
  config.structured_candidate_count = 0;
  config.time_steps = 4;
  config.shift_control_sequence = false;
  config.temperature = 1.0;
  config.gamma = 0.0;
  config.vx_min = 0.4;
  config.vx_max = 0.4;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.2}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, -0.2}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.0})
  });

  const auto result = controller.optimize(
    MppiControl{0.4, 0.0},
    [](const MppiCandidate & candidate) {
      return MppiCandidateScore{
        true,
        candidate.controls.front().wz > 0.1 ? 0.0 : std::log(2.0)};
    });

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_NEAR(result.positive_weight_mass, 0.5, 1e-9);
  EXPECT_NEAR(result.negative_weight_mass, 0.25, 1e-9);
  EXPECT_NEAR(result.neutral_weight_mass, 0.25, 1e-9);
  EXPECT_NEAR(result.effective_sample_size, 8.0 / 3.0, 1e-9);
  EXPECT_NEAR(result.prefilter_command.wz, 0.05, 1e-9);
  EXPECT_EQ(result.structured_candidates, 0u);
}

TEST(MppiControllerTest, BatchScoringScoresAllCandidatesAtOnce)
{
  MppiConfig config;
  config.batch_size = 3;
  config.time_steps = 3;
  config.temperature = 0.1;
  config.vx_min = 0.0;
  config.vx_max = 1.0;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;
  config.shift_control_sequence = false;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(config.time_steps, MppiControl{-0.2, 0.0}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.0, 0.0}),
    std::vector<MppiControl>(config.time_steps, MppiControl{0.2, 0.0})
  });

  std::size_t batch_score_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.4, 0.0},
    [&](const std::vector<MppiCandidate> & candidates) {
      ++batch_score_calls;
      std::vector<MppiCandidateScore> scores;
      scores.reserve(candidates.size());
      for(const auto & candidate : candidates){
        scores.push_back(MppiCandidateScore{
          true,
          std::fabs(candidate.controls.front().vx - 0.6)});
      }
      return scores;
    });

  EXPECT_TRUE(result.has_valid_control);
  EXPECT_EQ(batch_score_calls, 1u);
  EXPECT_GT(result.optimal_sequence.front().vx, 0.5);
}

TEST(MppiControllerTest, ControlEnergyScalesWithSamplingVariance)
{
  const auto optimized_vx = [](const double vx_std) {
      MppiConfig config;
      config.batch_size = 2;
      config.time_steps = 1;
      config.shift_control_sequence = false;
      config.temperature = 1.0;
      config.gamma = 0.1;
      config.vx_min = 0.0;
      config.vx_max = 1.0;
      config.wz_max = 1.0;
      config.ax_min = -10.0;
      config.ax_max = 10.0;
      config.az_max = 10.0;
      config.vx_std = vx_std;
      config.wz_std = 0.0;

      MppiController controller(config);
      controller.resetControlSequence(MppiControl{0.5, 0.0});
      controller.setDebugNoiseForTest({
        {MppiControl{-0.1, 0.0}},
        {MppiControl{0.1, 0.0}}
      });

      const auto result = controller.optimize(
        MppiControl{0.5, 0.0},
        [](const MppiCandidate &) {return MppiCandidateScore{true, 0.0};});
      return result.optimal_sequence.front().vx;
    };

  EXPECT_LT(optimized_vx(0.2), optimized_vx(0.4));
}

TEST(MppiControllerTest, StructuredCandidatesHaveZeroControlEnergy)
{
  auto config = makeControlEnergyConfig();
  config.batch_size = 4;
  config.structured_candidate_count = 4;
  config.time_steps = 3;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.2});

  const auto result = controller.optimize(
    MppiControl{0.4, 0.2},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 5.0};});

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.structured_candidates, config.structured_candidate_count);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 5.0);
  EXPECT_DOUBLE_EQ(result.best_control_energy, 0.0);
  EXPECT_DOUBLE_EQ(result.best_cost, 5.0);
}

TEST(MppiControllerTest, DebugRandomCandidateReportsRawControlEnergy)
{
  auto config = makeControlEnergyConfig();
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.2});
  controller.setDebugNoiseForTest({{MppiControl{-0.2, -0.1}}});

  const auto result = controller.optimize(
    MppiControl{0.4, 0.2},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 0.5};});

  const double expected_energy =
    config.gamma / (config.vx_std * config.vx_std) * 0.4 * -0.2 +
    config.gamma / (config.wz_std * config.wz_std) * 0.2 * -0.1;
  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.structured_candidates, 0u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 0.5);
  EXPECT_NEAR(result.best_control_energy, expected_energy, 1e-12);
  EXPECT_NEAR(result.best_cost, 0.5 + expected_energy, 1e-12);
  EXPECT_LT(result.best_cost, 0.0);
}

TEST(MppiControllerTest, ControlEnergyUsesRawNoiseWhenControlsAreClipped)
{
  auto config = makeControlEnergyConfig();
  config.gamma = 0.2;
  config.vx_std = 0.5;
  config.wz_std = 0.25;
  config.vx_max = 0.6;
  config.wz_max = 0.3;
  config.ax_min = -1.0;
  config.ax_max = 1.0;
  config.az_max = 1.0;
  config.controller_period = 0.1;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.5, 0.2});
  controller.setDebugNoiseForTest({{MppiControl{2.0, 1.0}}});

  const auto result = controller.optimize(
    MppiControl{0.5, 0.2},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 2.0};});

  const double expected_energy =
    config.gamma / (config.vx_std * config.vx_std) * 0.5 * 2.0 +
    config.gamma / (config.wz_std * config.wz_std) * 0.2 * 1.0;
  ASSERT_TRUE(result.has_valid_control);
  ASSERT_EQ(result.best_sample_sequence.size(), 1u);
  EXPECT_NEAR(result.best_sample_sequence.front().vx, 0.6, 1e-12);
  EXPECT_NEAR(result.best_sample_sequence.front().wz, 0.3, 1e-12);
  EXPECT_NEAR(result.best_control_energy, expected_energy, 1e-12);
  EXPECT_NEAR(result.best_cost, 2.0 + expected_energy, 1e-12);
}

TEST(MppiControllerTest, ShiftedMeasuredStepDoesNotContributeControlEnergy)
{
  auto config = makeControlEnergyConfig();
  config.time_steps = 3;
  config.shift_control_sequence = true;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.2});
  controller.setDebugNoiseForTest({{
    MppiControl{100.0, 100.0},
    MppiControl{0.1, -0.05},
    MppiControl{-0.2, 0.1}
  }});

  const auto result = controller.optimize(
    MppiControl{0.4, 0.2},
    [](const MppiCandidate &) {return MppiCandidateScore{true, 1.0};});

  const double expected_energy =
    config.gamma / (config.vx_std * config.vx_std) * 0.4 * (0.1 - 0.2) +
    config.gamma / (config.wz_std * config.wz_std) * 0.2 * (-0.05 + 0.1);
  ASSERT_TRUE(result.has_valid_control);
  EXPECT_NEAR(result.best_control_energy, expected_energy, 1e-12);
  EXPECT_NEAR(result.best_cost, 1.0 + expected_energy, 1e-12);
}

TEST(MppiControllerTest, LatchedStructuredAndRandomCandidatesKeepDistinctEnergy)
{
  auto config = makeStructuredPhaseConfig();
  config.batch_size = 4;
  config.structured_candidate_count = 4;
  config.time_steps = 4;
  config.gamma = 0.2;
  config.vx_std = 0.1;
  config.wz_std = 0.2;
  config.seed = 19;
  config.temperature = 0.01;
  config.vx_min = 0.2;
  config.vx_max = 0.6;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.3, 0.1});

  const auto latched = controller.optimizeBatch(
    MppiControl{0.3, 0.1},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{true, 100.0, MppiBypassSide::Left, true});
      scores[0].cost = 0.0;
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(latched.has_valid_control);
  ASSERT_EQ(latched.selected_bypass_side, MppiBypassSide::Left);

  const auto structured = controller.optimizeBatch(
    MppiControl{0.3, 0.1},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{true, 10000.0, MppiBypassSide::Left, true});
      scores[0].cost = 3.0;
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(structured.has_valid_control);
  ASSERT_EQ(structured.structured_candidates, 2u);
  EXPECT_EQ(structured.best_sample_index, 0u);
  EXPECT_DOUBLE_EQ(structured.best_critic_cost, 3.0);
  EXPECT_DOUBLE_EQ(structured.best_control_energy, 0.0);
  EXPECT_DOUBLE_EQ(structured.best_cost, 3.0);

  const auto nominal = controller.controlSequence();
  double expected_random_energy = 0.0;
  for(std::size_t step = 0; step < config.time_steps; ++step){
    const auto noise = controller.debugNoiseAt(2, step);
    expected_random_energy +=
      config.gamma / (config.vx_std * config.vx_std) * nominal[step].vx * noise.vx +
      config.gamma / (config.wz_std * config.wz_std) * nominal[step].wz * noise.wz;
  }
  ASSERT_GT(std::fabs(expected_random_energy), 1e-9);

  const auto random = controller.optimizeBatch(
    MppiControl{0.3, 0.1},
    [](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(
        candidates.size(), MppiCandidateScore{true, 10000.0, MppiBypassSide::Left, true});
      scores[2].cost = 5.0;
      return scores;
    },
    []() {return true;});
  ASSERT_TRUE(random.has_valid_control);
  EXPECT_EQ(random.structured_candidates, 2u);
  EXPECT_EQ(random.best_sample_index, 2u);
  EXPECT_DOUBLE_EQ(random.best_critic_cost, 5.0);
  EXPECT_NEAR(random.best_control_energy, expected_random_energy, 1e-12);
  EXPECT_NEAR(random.best_cost, 5.0 + expected_random_energy, 1e-12);
}

TEST(MppiControllerTest, BestCostDiagnosticsResetForEachIteration)
{
  auto config = makeControlEnergyConfig();
  config.batch_size = 2;
  config.iteration_count = 2;
  config.gamma = 0.0;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.2});
  controller.setDebugNoiseForTest({
    {MppiControl{-0.1, 0.0}},
    {MppiControl{0.1, 0.0}}
  });

  std::size_t score_calls = 0;
  const auto result = controller.optimizeBatch(
    MppiControl{0.4, 0.2},
    [&](const std::vector<MppiCandidate> &) {
      ++score_calls;
      if(score_calls == 1){
        return std::vector<MppiCandidateScore>{
          MppiCandidateScore{true, 1.0}, MppiCandidateScore{true, 2.0}};
      }
      return std::vector<MppiCandidateScore>{
        MppiCandidateScore{true, 8.0}, MppiCandidateScore{true, 7.0}};
    });

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(score_calls, 2u);
  EXPECT_EQ(result.best_sample_index, 1u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 7.0);
  EXPECT_DOUBLE_EQ(result.best_control_energy, 0.0);
  EXPECT_DOUBLE_EQ(result.best_cost, 7.0);
}

TEST(MppiControllerTest, RejectsNonfiniteTotalCostBeforeSoftmax)
{
  auto config = makeControlEnergyConfig();
  config.batch_size = 2;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.4, 0.2});
  controller.setDebugNoiseForTest({
    {MppiControl{std::numeric_limits<double>::infinity(), 0.0}},
    {MppiControl{0.0, 0.0}}
  });

  const auto result = controller.optimizeBatch(
    MppiControl{0.4, 0.2},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>{
        MppiCandidateScore{true, 0.0}, MppiCandidateScore{true, 1.0}};
    });

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.rejected_samples, 1u);
  EXPECT_EQ(result.best_sample_index, 1u);
  EXPECT_DOUBLE_EQ(result.best_critic_cost, 1.0);
  EXPECT_DOUBLE_EQ(result.best_control_energy, 0.0);
  EXPECT_DOUBLE_EQ(result.best_cost, 1.0);
}

TEST(MppiControllerTest, SeededRandomAngularEnergyUsesEffectiveCovariance)
{
  auto config = makeControlEnergyConfig();
  config.structured_candidate_count = 0;
  config.seed = 23;
  config.gamma = 0.4;
  config.vx_std = 0.0;
  config.wz_std = 1.0;
  config.vx_min = 0.2;
  config.vx_max = 0.2;
  config.wz_max = 2.0;
  config.wheelbase = 0.2;
  config.max_steer = std::atan(0.5);

  const MppiControl nominal{0.2, 0.3};
  MppiController controller(config);
  controller.resetControlSequence(nominal);
  const double raw_angular_noise = controller.debugNoiseAt(0, 0).wz;
  const double angular_limit = std::min(
    config.wz_max,
    std::fabs(nominal.vx) * std::tan(config.max_steer) / config.wheelbase);
  const double angular_noise_scale = std::min(1.0, angular_limit / config.wz_std);
  ASSERT_GT(angular_noise_scale, 0.0);
  ASSERT_LT(angular_noise_scale, 1.0);
  ASSERT_GT(std::fabs(raw_angular_noise), 1e-9);

  const auto result = controller.optimize(
    nominal,
    [](const MppiCandidate &) {return MppiCandidateScore{true, 4.0};});

  const double effective_std = config.wz_std * angular_noise_scale;
  const double effective_noise = raw_angular_noise * angular_noise_scale;
  const double expected_energy =
    config.gamma / (effective_std * effective_std) * nominal.wz * effective_noise;
  const double old_energy =
    config.gamma / (config.wz_std * config.wz_std) * nominal.wz * effective_noise;
  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.structured_candidates, 0u);
  EXPECT_NEAR(result.best_control_energy, expected_energy, 1e-12);
  EXPECT_NEAR(result.best_cost, 4.0 + expected_energy, 1e-12);
  EXPECT_GT(std::fabs(expected_energy - old_energy), 1e-6);
}

TEST(MppiControllerTest, SeededVxNoiseCarriesPerSampleAngularCovariance)
{
  auto config = makeControlEnergyConfig();
  config.batch_size = 2;
  config.structured_candidate_count = 0;
  config.seed = 31;
  config.gamma = 0.1;
  config.vx_std = 0.12;
  config.wz_std = 0.8;
  config.vx_min = 0.05;
  config.vx_max = 0.5;
  config.wz_max = 2.0;
  config.wheelbase = 0.5;
  config.max_steer = 0.5;

  const MppiControl nominal{0.2, 0.1};
  MppiController controller(config);
  controller.resetControlSequence(nominal);
  const std::vector<MppiControl> raw_noises{
    controller.debugNoiseAt(0, 0),
    controller.debugNoiseAt(1, 0)};
  std::vector<MppiCandidate> captured;

  const auto result = controller.optimizeBatch(
    nominal,
    [&](const std::vector<MppiCandidate> & candidates) {
      captured = candidates;
      return std::vector<MppiCandidateScore>(
        candidates.size(), MppiCandidateScore{true, 2.0});
    });

  ASSERT_TRUE(result.has_valid_control);
  ASSERT_EQ(captured.size(), 2u);
  std::vector<double> scales(2, 0.0);
  std::vector<double> expected_energies(2, 0.0);
  for(std::size_t sample = 0; sample < 2; ++sample){
    const double angular_limit = std::min(
      config.wz_max,
      std::fabs(captured[sample].controls.front().vx) *
      std::tan(config.max_steer) / config.wheelbase);
    scales[sample] = std::min(1.0, angular_limit / config.wz_std);
    expected_energies[sample] =
      config.gamma / (config.vx_std * config.vx_std) *
      nominal.vx * raw_noises[sample].vx;
    if(scales[sample] > 1e-9){
      const double effective_std = config.wz_std * scales[sample];
      const double effective_noise = raw_noises[sample].wz * scales[sample];
      expected_energies[sample] +=
        config.gamma / (effective_std * effective_std) *
        nominal.wz * effective_noise;
    }
  }
  ASSERT_GT(std::fabs(scales[0] - scales[1]), 1e-6);
  const std::size_t expected_best =
    expected_energies[0] < expected_energies[1] ? 0u : 1u;
  EXPECT_EQ(result.best_sample_index, expected_best);
  EXPECT_NEAR(result.best_control_energy, expected_energies[expected_best], 1e-12);
  EXPECT_NEAR(result.best_cost, 2.0 + expected_energies[expected_best], 1e-12);
}

TEST(MppiControllerTest, ReusesNoiseAcrossOptimizationCyclesByDefault)
{
  MppiConfig config;
  config.batch_size = 2;
  config.time_steps = 2;
  config.vx_std = 0.2;
  config.wz_std = 0.4;
  config.seed = 7;

  MppiController controller(config);
  const auto first_noise = controller.debugNoiseAt(0, 0);
  controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate &) {
      return MppiCandidateScore{true, 0.0};
    });
  const auto second_noise = controller.debugNoiseAt(0, 0);

  EXPECT_DOUBLE_EQ(first_noise.vx, second_noise.vx);
  EXPECT_DOUBLE_EQ(first_noise.wz, second_noise.wz);
}

TEST(MppiControllerTest, RegeneratesNoiseWhenConfigured)
{
  MppiConfig config;
  config.batch_size = 2;
  config.time_steps = 2;
  config.vx_std = 0.2;
  config.wz_std = 0.4;
  config.seed = 7;
  config.regenerate_noises = true;

  MppiController controller(config);
  const auto first_noise = controller.debugNoiseAt(0, 0);
  controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate &) {
      return MppiCandidateScore{true, 0.0};
    });
  const auto second_noise = controller.debugNoiseAt(0, 0);

  EXPECT_TRUE(
    std::fabs(first_noise.vx - second_noise.vx) > 1e-9 ||
    std::fabs(first_noise.wz - second_noise.wz) > 1e-9);
}

TEST(MppiControllerTest, RejectsAllInvalidSamples)
{
  MppiConfig config;
  config.batch_size = 5;
  config.time_steps = 3;

  MppiController controller(config);
  const auto result = controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate &) {
      return MppiCandidateScore{false, -1.0};
    });

  EXPECT_FALSE(result.has_valid_control);
  EXPECT_EQ(result.rejected_samples, config.batch_size);
}

TEST(MppiControllerTest, ShiftsSequenceAfterCommandExtraction)
{
  MppiConfig config;
  config.batch_size = 2;
  config.time_steps = 4;
  config.shift_control_sequence = true;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.3, 0.1});

  const auto result = controller.optimize(
    MppiControl{0.3, 0.1},
    [](const MppiCandidate & candidate) {
      return MppiCandidateScore{true, candidate.controls.front().vx};
    });

  ASSERT_TRUE(result.has_valid_control);
  ASSERT_EQ(controller.controlSequence().size(), 4u);
  EXPECT_DOUBLE_EQ(controller.controlSequence()[0].vx, result.optimal_sequence[1].vx);
  EXPECT_DOUBLE_EQ(controller.controlSequence()[3].vx, result.optimal_sequence[3].vx);
}

TEST(MppiControllerTest, ShiftedControlSequenceCommandsSecondTimestep)
{
  MppiConfig config;
  config.batch_size = 1;
  config.time_steps = 3;
  config.shift_control_sequence = true;
  config.model_dt = 0.1;
  config.controller_period = 0.1;
  config.vx_min = 0.0;
  config.vx_max = 1.0;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.5;
  config.max_steer = 1.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({
    {
      MppiControl{0.0, 0.0},
      MppiControl{0.0, 0.4},
      MppiControl{0.0, 0.4}
    }
  });

  const auto result = controller.optimize(
    MppiControl{0.2, 0.0},
    [](const MppiCandidate &) {
      return MppiCandidateScore{true, 0.0};
    });

  ASSERT_TRUE(result.has_valid_control);
  ASSERT_GE(result.optimal_sequence.size(), 2u);
  EXPECT_DOUBLE_EQ(result.optimal_sequence.front().wz, 0.0);
  EXPECT_DOUBLE_EQ(result.command.wz, result.optimal_sequence[1].wz);
  EXPECT_GT(result.command.wz, 0.0);
}

TEST(MppiControllerTest, ShiftedControlSequencePinsFirstTimestepToMeasuredSpeed)
{
  MppiConfig config;
  config.batch_size = 1;
  config.time_steps = 3;
  config.shift_control_sequence = true;
  config.model_dt = 0.1;
  config.controller_period = 0.1;
  config.vx_min = 0.0;
  config.vx_max = 1.0;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = 10.0;
  config.az_max = 10.0;
  config.wheelbase = 0.5;
  config.max_steer = 1.0;
  config.vx_std = 0.0;
  config.wz_std = 0.0;

  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest({{
    MppiControl{0.5, 0.5},
    MppiControl{0.0, 0.2},
    MppiControl{0.0, 0.2}
  }});

  const MppiControl measured_speed{0.2, 0.0};
  bool first_timestep_is_measured = false;
  const auto result = controller.optimize(
    measured_speed,
    [&](const MppiCandidate & candidate) {
      first_timestep_is_measured =
        candidate.controls.front().vx == measured_speed.vx &&
        candidate.controls.front().wz == measured_speed.wz;
      return MppiCandidateScore{true, 0.0};
    });

  ASSERT_TRUE(result.has_valid_control);
  EXPECT_TRUE(first_timestep_is_measured);
  EXPECT_DOUBLE_EQ(result.optimal_sequence.front().vx, measured_speed.vx);
  EXPECT_DOUBLE_EQ(result.optimal_sequence.front().wz, measured_speed.wz);
}
