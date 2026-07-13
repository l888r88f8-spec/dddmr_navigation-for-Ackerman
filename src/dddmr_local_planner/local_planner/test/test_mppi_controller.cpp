#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "local_planner/mppi_controller.h"

using local_planner::MppiCandidate;
using local_planner::MppiCandidateScore;
using local_planner::MppiBypassSide;
using local_planner::MppiConfig;
using local_planner::MppiControl;
using local_planner::MppiController;
using local_planner::mppiSequenceRespectsSpeedCap;

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

}  // namespace

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
