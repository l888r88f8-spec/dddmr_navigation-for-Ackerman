# MPPI Bypass-Side and Steering-Phase Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent Ackermann MPPI from alternating between mirrored obstacle-avoidance trajectories while still allowing a same-side maneuver to countersteer and rejoin the path.

**Architecture:** The trajectory-scoring layer classifies each predicted trajectory by physical lateral bypass side. The MPPI controller latches that side independently of steering sign, advances a finite S-maneuver phase across receding-horizon cycles, and applies stochastic control-energy correction only to random samples. Existing collision validation and Ackermann constraints remain authoritative.

**Tech Stack:** C++17, ROS 2 Humble, ament/colcon, GoogleTest, existing MPPI critic and trajectory-generation code.

---

### Task 1: Classify physical bypass side from predicted geometry

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h`
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_critic_manager.h`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_critic_manager.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_critic_manager.cpp`

- [ ] **Step 1: Write failing geometry-classification tests**

Add tests with a straight path on the x-axis. Require a trajectory at positive y to be `Left`, its mirror at negative y to be `Right`, a trajectory within `0.10 m` to be `Neutral`, and a left-side trajectory whose final controls countersteer right to remain `Left`.

```cpp
TEST(MppiCriticManagerTest, ClassifiesBypassSideFromLateralTrajectoryPosition)
{
  const std::vector<MppiPathPoint> path{
    {0.0, 0.0, 0.0, true}, {1.0, 0.0, 0.0, true}, {2.0, 0.0, 0.0, true}};
  MppiScoredTrajectory left;
  left.points = {{0.3, 0.12}, {1.0, 0.35}, {1.8, 0.20}};
  left.controls = {{0.2, 0.08}, {0.2, -0.05}};
  auto right = left;
  for(auto & point : right.points) {point.y = -point.y;}
  MppiScoredTrajectory neutral;
  neutral.points = {{0.5, 0.03}, {1.5, 0.05}};

  EXPECT_EQ(classifyMppiBypassSide(left, path), MppiBypassSide::Left);
  EXPECT_EQ(classifyMppiBypassSide(right, path), MppiBypassSide::Right);
  EXPECT_EQ(classifyMppiBypassSide(neutral, path), MppiBypassSide::Neutral);
}
```

- [ ] **Step 2: Build and run the focused test to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner
colcon test --packages-select local_planner --ctest-args -R test_mppi_critic_manager --output-on-failure
```

Expected: compilation fails because `MppiBypassSide` and `classifyMppiBypassSide()` do not exist.

- [ ] **Step 3: Add side metadata and the classifier**

Add the shared enum and score fields:

```cpp
enum class MppiBypassSide : int
{
  Right = -1,
  Neutral = 0,
  Left = 1
};

struct MppiCandidateScore
{
  bool valid = false;
  double cost = std::numeric_limits<double>::infinity();
  MppiBypassSide bypass_side = MppiBypassSide::Neutral;
  bool collision_free = true;
};
```

Declare the classifier in `mppi_critic_manager.h` and implement nearest-segment signed offsets in `mppi_critic_manager.cpp`. For each trajectory point, project onto every non-degenerate path segment, retain the closest projection, and calculate the signed cross-track offset with the segment unit tangent. Use:

```cpp
const double metric = 0.7 * peak_signed_offset + 0.3 * terminal_signed_offset;
if(metric > 0.10) {return MppiBypassSide::Left;}
if(metric < -0.10) {return MppiBypassSide::Right;}
return MppiBypassSide::Neutral;
```

Return `Neutral` for paths shorter than two points, empty trajectories, non-finite results, or paths containing no non-degenerate segment.

- [ ] **Step 4: Run the focused tests to verify GREEN**

Run the build and test command from Step 2. Expected: classifier tests and existing critic tests pass.

- [ ] **Step 5: Commit the geometry unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h \
  src/dddmr_local_planner/local_planner/include/local_planner/mppi_critic_manager.h \
  src/dddmr_local_planner/local_planner/src/mppi_critic_manager.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_critic_manager.cpp
git commit -m "feat: classify MPPI trajectories by bypass side"
```

### Task 2: Latch bypass side independently of steering sign

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`

- [ ] **Step 1: Replace steering-sign mode tests with failing side-latch tests**

Add three deterministic batch tests. The first uses two real control sequences whose steering signs deliberately oppose their geometric side metadata:

```cpp
TEST(MppiControllerTest, CountersteerDoesNotChangeLatchedPhysicalSide)
{
  MppiConfig config;
  config.batch_size = 2;
  config.structured_candidate_count = 0;
  config.time_steps = 4;
  config.shift_control_sequence = false;
  config.temperature = 0.1;
  config.gamma = 0.0;
  config.vx_min = config.vx_max = 0.2;
  config.wz_max = 1.0;
  config.ax_min = -10.0;
  config.ax_max = config.az_max = 10.0;
  config.wheelbase = 0.1;
  config.max_steer = 1.0;

  MppiController controller(config);
  controller.resetControlSequence({0.2, 0.0});
  controller.setDebugNoiseForTest({
    std::vector<MppiControl>(4, {0.0, -0.10}),
    std::vector<MppiControl>(4, {0.0, 0.10})});
  const auto result = controller.optimizeBatch(
    {0.2, 0.0},
    [](const std::vector<MppiCandidate> &) {
      return std::vector<MppiCandidateScore>{
        {true, 0.0, MppiBypassSide::Left, true},
        {true, 0.4, MppiBypassSide::Right, true}};
    },
    [] {return true;});

  EXPECT_EQ(result.selected_bypass_side, MppiBypassSide::Left);
  EXPECT_LT(result.command.wz, 0.0);
}
```

Also require one infeasible-left cycle to return invalid without changing the latch, and the third consecutive infeasible-left cycle to switch to a feasible right side. Helpers must construct real `MppiCandidateScore` vectors; no production behavior may be mocked.

- [ ] **Step 2: Run the controller test to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner
colcon test --packages-select local_planner --ctest-args -R test_mppi_controller --output-on-failure
```

Expected: compile failure on `selected_bypass_side` or behavioral failure because the current code selects by command-step `wz`.

- [ ] **Step 3: Implement explicit side-latch state**

Add controller fields and public diagnostics:

```cpp
MppiBypassSide latched_bypass_side_ = MppiBypassSide::Neutral;
std::size_t latched_side_infeasible_cycles_ = 0;
static constexpr std::size_t kSideSwitchConfirmCycles = 3;
```

Pass side and collision-free metadata alongside `valid_sequences` and `valid_costs` into `applySoftmaxUpdate()`. When directional mode is inactive, clear the latch and failure counter and aggregate all samples normally. When it is active:

```cpp
if(latched_bypass_side_ == MppiBypassSide::Neutral) {
  latched_bypass_side_ = chooseSideBySoftmaxMassThenBestCost(
    valid_sides, collision_free, weights, valid_costs);
}
if(hasCollisionFreeCandidate(latched_bypass_side_)) {
  latched_side_infeasible_cycles_ = 0;
  aggregateOnly(latched_bypass_side_);
} else if(++latched_side_infeasible_cycles_ < kSideSwitchConfirmCycles) {
  UpdateStatistics invalid;
  invalid.valid = false;
  return invalid;
} else if(hasCollisionFreeCandidate(opposite(latched_bypass_side_))) {
  latched_bypass_side_ = opposite(latched_bypass_side_);
  latched_side_infeasible_cycles_ = 0;
  aggregateOnly(latched_bypass_side_);
} else {
  UpdateStatistics invalid;
  invalid.valid = false;
  return invalid;
}
```

Neutral candidates never participate in directional aggregation. Add `bool valid` to `UpdateStatistics`; make `optimizeBatch()` return `has_valid_control=false` without shifting history when directional aggregation is invalid.

Clear the latch and counter in `resetControlSequence()`.

- [ ] **Step 4: Run focused tests to verify GREEN**

Run the command from Step 2. Expected: same-side countersteer, delayed switch, reset, existing softmax, and constraint tests pass.

- [ ] **Step 5: Commit the side-latch unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h \
  src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "fix: latch MPPI physical bypass side"
```

### Task 3: Advance structured S-maneuver phase across cycles

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`

- [ ] **Step 1: Write a failing receding-horizon phase test**

Configure a 12-step horizon with eight structured candidates, force sample four (the strong left S candidate during initial selection) to win, and run seven directional optimization cycles. On later cycles mark every structured candidate as the already latched left side while continuing to prefer sample four. Capture each command and phase:

```cpp
std::vector<double> commands;
for(std::size_t cycle = 0; cycle < 7; ++cycle) {
  const auto result = controller.optimizeBatch(
    {0.2, commands.empty() ? 0.0 : commands.back()},
    [cycle](const std::vector<MppiCandidate> & candidates) {
      std::vector<MppiCandidateScore> scores(candidates.size());
      for(std::size_t i = 0; i < scores.size(); ++i) {
        scores[i].valid = true;
        scores[i].cost = i == 4 ? 0.0 : 10.0;
        scores[i].bypass_side = cycle == 0 && i % 2 == 1 ?
          MppiBypassSide::Right : MppiBypassSide::Left;
        scores[i].collision_free = true;
      }
      return scores;
    },
    [] {return true;});
  ASSERT_TRUE(result.has_valid_control);
  EXPECT_EQ(result.structured_phase_step, cycle + 1);
  commands.push_back(result.command.wz);
}
EXPECT_GT(commands.front(), 0.0);
EXPECT_LT(commands.back(), 0.0);
controller.resetControlSequence({0.2, 0.0});
const auto reset_result = controller.optimizeBatch(
  {0.2, 0.0},
  [](const std::vector<MppiCandidate> & candidates) {
    return std::vector<MppiCandidateScore>(
      candidates.size(),
      MppiCandidateScore{true, 0.0, MppiBypassSide::Neutral, true});
  },
  [] {return false;});
EXPECT_EQ(reset_result.structured_phase_step, 0u);
```

- [ ] **Step 2: Run the focused test to verify RED**

Run the controller command from Task 2. Expected: the command remains in the initial turn because `buildStructuredSequence()` restarts its phase each call.

- [ ] **Step 3: Implement finite persistent template phase**

Add `std::size_t structured_phase_step_ = 0;`. Before a side is latched, retain the paired sustained arc/S templates. Once latched, every structured slot uses the latched physical direction and one of the existing `0.55` or `0.90` intensity levels:

```cpp
const std::size_t absolute_phase = structured_phase_step_ + phase_step;
double phase_sign = 0.0;
if(absolute_phase < controlled_steps / 2) {
  phase_sign = sideSign(latched_bypass_side_);
} else if(absolute_phase < controlled_steps) {
  phase_sign = -sideSign(latched_bypass_side_);
}
requested.wz = phase_sign * intensity * ackermannWzLimit(vx_only.vx);
```

Do not generate sustained full-horizon arcs after the side is latched. Increment the phase once after a valid directional command, saturating at `config_.time_steps`. Reset it when directional mode clears or `resetControlSequence()` runs.

- [ ] **Step 4: Run controller tests to verify GREEN**

Expected: the transition reaches the command index across calls, the straight tail does not restart, and all constraint/filter tests pass.

- [ ] **Step 5: Commit the persistent-phase unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h \
  src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "fix: advance MPPI structured maneuver phase"
```

### Task 4: Correct structured-candidate energy treatment

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`

- [ ] **Step 1: Write failing energy-accounting tests**

With critic cost fixed at `5.0`, force a deterministic structured sample to be best and require its reported energy to equal zero and total best cost to equal five. In a separate random-sample test with known debug noise, require the existing `gamma / variance * nominal * noise` correction.

```cpp
EXPECT_DOUBLE_EQ(structured.best_critic_cost, 5.0);
EXPECT_DOUBLE_EQ(structured.best_control_energy, 0.0);
EXPECT_DOUBLE_EQ(structured.best_cost, 5.0);
EXPECT_NE(random.best_control_energy, 0.0);
EXPECT_NEAR(random.best_cost,
  random.best_critic_cost + random.best_control_energy, 1e-12);
```

- [ ] **Step 2: Run the focused test to verify RED**

Expected: the structured candidate receives reconstructed constrained-sequence energy and may report a negative total cost.

- [ ] **Step 3: Carry sample metadata through generation**

Introduce a private sample type:

```cpp
struct SampledSequence
{
  std::vector<MppiControl> controls;
  double control_energy = 0.0;
  bool structured = false;
};
```

For structured samples, set `structured=true` and `control_energy=0.0`. For random samples, accumulate energy from the stored sampled perturbation before constraints:

```cpp
energy += gamma / (vx_std * vx_std) * nominal.vx * noise.vx;
energy += gamma / (wz_std * wz_std) * nominal.wz *
  (noise.wz * angular_noise_scale);
```

Do not reconstruct either term as `constrained_sequence - nominal`. Populate `best_critic_cost` and `best_control_energy` whenever the best total cost changes.

- [ ] **Step 4: Run controller tests to verify GREEN**

Expected: structured cost remains critic-only, random correction tests pass, and no existing softmax test regresses.

- [ ] **Step 5: Commit the energy-accounting unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h \
  src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "fix: separate structured MPPI candidate energy"
```

### Task 5: Integrate side metadata, reset propagation, and diagnostics

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/src/local_planner.cpp`
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/local_planner.h`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_critic_manager.cpp`

- [ ] **Step 1: Write failing integration assertions**

Add a test/helper-level assertion proving that critic results receive side and collision metadata from the corresponding generated trajectory. Extend reset tests to require side, failure count, and phase to return to neutral/zero.

- [ ] **Step 2: Run local planner tests to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner
colcon test --packages-select local_planner --event-handlers console_direct+
```

Expected: metadata remains neutral because `score_function` does not yet populate it, or new diagnostics are absent.

- [ ] **Step 3: Populate score metadata in the batch scoring lambda**

After critic scoring and speed-cap validation, iterate by candidate index:

```cpp
for(std::size_t i = 0; i < critic_result.scores.size(); ++i) {
  critic_result.scores[i].bypass_side =
    classifyMppiBypassSide(critic_data.trajectories[i], critic_data.path);
  critic_result.scores[i].collision_free =
    isMppiTrajectoryCollisionFree(critic_data.trajectories[i]);
}
```

Invalid scores remain ineligible regardless of side. Existing controller resets used for route changes and failed optimal-sequence validation now clear side/phase through `resetControlSequence()`.

- [ ] **Step 4: Extend runtime diagnostics**

Add result fields for selected and best side, feasible left/right/neutral counts, side-failure count, phase, best sample index, critic cost, and energy. Append stable names to the existing throttled line:

```text
selected_side=%s, best_side=%s, feasible_left=%zu, feasible_right=%zu,
feasible_neutral=%zu, side_failures=%zu, structured_phase=%zu,
best_sample=%zu, best_critic_cost=%.3f, best_energy=%.3f
```

Use a total `toString(MppiBypassSide)` helper returning `left`, `right`, or `neutral`.

- [ ] **Step 5: Run all local planner tests to verify GREEN**

Run the command from Step 2. Expected: all local planner tests pass with no failures.

- [ ] **Step 6: Commit the integration unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/local_planner.h \
  src/dddmr_local_planner/local_planner/src/local_planner.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_critic_manager.cpp
git commit -m "feat: integrate MPPI bypass side diagnostics"
```

### Task 6: Full verification and real-robot handoff

**Files:**
- Review all changed MPPI controller, critic, local-planner, and test files.

- [ ] **Step 1: Build both affected packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner p2p_move_base
```

Expected: both packages finish successfully.

- [ ] **Step 2: Run affected test suites**

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select local_planner p2p_move_base --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero errors and zero failures.

- [ ] **Step 3: Check formatting and scope**

```bash
git diff --check
git status --short
git diff --stat HEAD~5..HEAD
```

Expected: no whitespace errors; only MPPI implementation, focused tests, and approved docs are part of this work.

- [ ] **Step 4: Review diagnostics against invariants**

Inspect the code paths and tests to confirm:

```text
selected_side may remain left while cmd_wz becomes negative;
structured_phase increases once per valid blocked-path command;
side changes only at side_failures=3;
structured best_energy is 0;
collision-invalid candidates never enter directional aggregation.
```

- [ ] **Step 5: Record robot validation procedure**

Rerun the same centered-obstacle scenario and capture unthrottled `/cmd_vel` or `/cmd_vel_stamped`, odometry, and best trajectory. The throttled diagnostic must show one selected side through the maneuver, a monotonic structured phase, a permitted same-side countersteer, no repeated full-steering left/right alternation, and controller runtime within the 15 Hz budget.
