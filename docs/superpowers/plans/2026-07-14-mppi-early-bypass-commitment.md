# MPPI Early Bypass Commitment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent same-side Ackermann rollouts with opposing immediate steering from averaging into a near-straight command, and advance the finite S-maneuver only when the issued command executes its current phase.

**Architecture:** Add one shared phase-direction calculation to `MppiController`. Directional softmax first retains the existing physical-side and collision-free checks, then filters the selected set by command-step phase compatibility; phase progress is driven by the constrained, smoothed output rather than structured-candidate weight. Existing final trajectory validation remains unchanged.

**Tech Stack:** C++17, ROS 2 Humble, GoogleTest, ament/colcon.

---

### Task 1: Reproduce same-side command cancellation

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`
- Modify: `src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`

- [ ] **Step 1: Write the failing phase-compatible selection test**

Add a deterministic test using two non-structured debug-noise candidates. Mark both as collision-free `Right` rollouts even though one command turns right and the other turns left. Give them equal cost and require directional aggregation to retain only the current outward-right command:

```cpp
TEST(MppiControllerTest, DirectionalAggregationRejectsSameSideCandidateWithWrongCurrentPhase)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 2;
  config.structured_candidate_count = 0;
  config.time_steps = 4;
  config.shift_control_sequence = false;
  MppiController controller(config);
  controller.resetControlSequence(MppiControl{0.2, 0.0});
  controller.setDebugNoiseForTest(makeAngularNoises(config, -0.10, 0.10));

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
  EXPECT_EQ(result.phase_compatible_candidates, 1u);
  EXPECT_NEAR(result.prefilter_command.wz, -0.10, 1e-12);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner
colcon test --packages-select local_planner --ctest-args -R test_mppi_controller --output-on-failure
```

Expected: compilation fails because `MppiResult::phase_compatible_candidates` does not exist. This proves the test observes the new contract rather than existing behavior.

- [ ] **Step 3: Add the shared phase contract and result diagnostic**

Add these private helpers and statistics fields:

```cpp
struct UpdateStatistics
{
  // existing fields...
  std::size_t phase_compatible_candidates = 0;
};

double expectedPhaseDirection(
  MppiBypassSide bypass_side,
  std::size_t structured_phase_step) const;
bool isPhaseCompatible(
  const MppiControl & command,
  double expected_direction) const;
```

Add `std::size_t phase_compatible_candidates = 0;` to `MppiResult`. Define one material-curvature ratio in the implementation:

```cpp
constexpr double kMaterialPhaseCurvatureRatio = 0.25;
```

`expectedPhaseDirection()` must return the physical side sign in the first half, its negative in the second half, and `0.0` after the finite maneuver. `isPhaseCompatible()` must return true for a completed maneuver; otherwise require:

```cpp
expected_direction * command.wz >=
  kMaterialPhaseCurvatureRatio * ackermannWzLimit(command.vx);
```

- [ ] **Step 4: Filter the selected set by physical side and phase**

Pass `structured_phase_step_` into `applySoftmaxUpdate()`. After choosing the physical side, build `selected` with:

```cpp
const double expected_direction =
  expectedPhaseDirection(bypass_side, structured_phase_step);
for(std::size_t sample = 0; sample < valid_sequences.size(); ++sample){
  const bool physical_side_selected = collision_free[sample] &&
    valid_sides[sample] == bypass_side;
  selected[sample] = physical_side_selected &&
    isPhaseCompatible(valid_sequences[sample][command_index], expected_direction);
  if(selected[sample]){
    ++statistics.phase_compatible_candidates;
  }
}
```

If no compatible candidate exists, leave `statistics.valid=false`; do not restore the old side-only selected set.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run the Task 1 Step 2 commands. Expected: `DirectionalAggregationRejectsSameSideCandidateWithWrongCurrentPhase` passes.

- [ ] **Step 6: Commit the selection unit**

```bash
git add src/dddmr_local_planner/local_planner/include/local_planner/mppi_controller.h \
  src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "fix: require immediate MPPI bypass commitment"
```

### Task 2: Drive phase from the issued command

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`

- [ ] **Step 1: Write the failing non-structured progress test**

Add a test proving that a valid random candidate can advance the maneuver after it executes the expected phase, even with zero structured weight:

```cpp
TEST(MppiControllerTest, ExecutedRandomCandidateAdvancesStructuredPhase)
{
  auto config = makeBypassLatchConfig();
  config.batch_size = 1;
  config.structured_candidate_count = 0;
  config.time_steps = 6;
  config.shift_control_sequence = false;
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
```

- [ ] **Step 2: Run the focused test and verify RED**

Run the Task 1 Step 2 commands. Expected: the new test fails because the current `structured_weight_mass >= 0.5` condition leaves phase at zero.

- [ ] **Step 3: Replace structured-mass progress with output progress**

Before smoothing, capture the current expected direction. After smoothing and assigning `result.command`, replace the structured-weight threshold with:

```cpp
if(select_directional_mode && latched_bypass_side_ != MppiBypassSide::Neutral){
  reset_structured_phase_for_side_change(latched_bypass_side_);
  const double expected_direction =
    expectedPhaseDirection(latched_bypass_side_, structured_phase_step_);
  if(expected_direction != 0.0 &&
    isPhaseCompatible(result.command, expected_direction))
  {
    const std::size_t first_control_step =
      config_.shift_control_sequence ? 1u : 0u;
    const std::size_t controlled_steps = config_.time_steps - first_control_step;
    if(structured_phase_step_ < controlled_steps){
      ++structured_phase_step_;
    }
  }
}
```

Retain `structured_weight_mass` calculation only for diagnostics.

- [ ] **Step 4: Add wrong-direction and sub-threshold hold tests**

Add two cases with a latched `Right` side. One issues `+0.10`; the other issues a negative command below `25%` of its Ackermann limit. Assert both leave `structured_phase_step` unchanged. Their score metadata stays `Right` to prove the hold is caused by execution-phase mismatch, not physical-side relabeling.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run the Task 1 Step 2 commands. Expected: executed-progress, wrong-direction hold, sub-threshold hold, and existing controller tests pass.

- [ ] **Step 6: Commit the execution-progress unit**

```bash
git add src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "fix: advance MPPI phase from executed steering"
```

### Task 3: Preserve same-side countersteer and fail closed

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`
- Modify: `src/dddmr_local_planner/local_planner/src/mppi_controller.cpp`

- [ ] **Step 1: Write the countersteer and no-compatible-candidate tests**

Advance a `Left` controller state to the second half using `resetControlSequenceForRetry()` with `structured_phase_step = controlled_steps / 2`. Provide one collision-free `Left` rollout with negative command and assert it remains selected and valid. In a separate test provide only positive-command `Left` rollouts at that phase and assert `has_valid_control=false` while `selected_bypass_side` remains `Left`.

```cpp
MppiAvoidanceState countersteer_state;
countersteer_state.side = MppiBypassSide::Left;
countersteer_state.structured_phase_step = 3;
controller.resetControlSequenceForRetry(MppiControl{0.2, 0.0}, countersteer_state);
```

- [ ] **Step 2: Run the focused test and verify RED or expose compatibility gaps**

Run the Task 1 Step 2 commands. Expected before the final compatibility cleanup: at least one new assertion fails if countersteer boundaries or invalid selected-set handling disagree with structured generation.

- [ ] **Step 3: Align all phase boundaries with structured generation**

Use `config_.time_steps - first_control_step` in both `expectedPhaseDirection()` and `buildStructuredSequence()`. For odd horizons, use integer `controlled_steps / 2` consistently. Ensure a zero expected direction after completion removes only the phase constraint, not physical-side or collision-free filtering.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the Task 1 Step 2 commands. Expected: same-side countersteer is valid, wrong-phase candidates fail closed, and all pre-existing phase/reset/switch tests pass after updating only expectations superseded by execution-driven progress.

- [ ] **Step 5: Commit the boundary unit**

```bash
git add src/dddmr_local_planner/local_planner/src/mppi_controller.cpp \
  src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp
git commit -m "test: cover MPPI phase-compatible countersteer"
```

### Task 4: Expose phase compatibility in runtime diagnostics

**Files:**
- Modify: `src/dddmr_local_planner/local_planner/src/local_planner.cpp`
- Test: `src/dddmr_local_planner/local_planner/test/test_mppi_controller.cpp`

- [ ] **Step 1: Reuse the result diagnostic contract established test-first in Task 1**

The Task 1 test already requires `MppiResult::phase_compatible_candidates` to report the exact selected count before production support exists. Re-run that test immediately before wiring the field into the ROS log so the diagnostic value is known green and no untested counter is introduced.

- [ ] **Step 2: Run the relevant focused test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner
colcon test --packages-select local_planner --event-handlers console_direct+ --return-code-on-test-failure
```

Expected: the controller diagnostic value test passes; `local_planner.cpp` still lacks the ROS log field and is the only production change in this task.

- [ ] **Step 3: Add the runtime field**

Insert `phase_compatible=%zu` next to the feasible-side counts in the `RCLCPP_INFO_THROTTLE` format and pass `result.phase_compatible_candidates` at the matching argument position.

- [ ] **Step 4: Run package tests and verify GREEN**

Run the Task 4 Step 2 commands. Expected: all `local_planner` tests pass.

- [ ] **Step 5: Commit diagnostics**

```bash
git add src/dddmr_local_planner/local_planner/src/local_planner.cpp \
  src/dddmr_local_planner/local_planner/test
git commit -m "feat: report MPPI phase-compatible samples"
```

### Task 5: Final verification and review

**Files:**
- Verify: all files changed by Tasks 1-4

- [ ] **Step 1: Build the affected package cleanly**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select local_planner --cmake-clean-cache
```

Expected: exit code zero with no compilation errors.

- [ ] **Step 2: Run the complete affected-package suite**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select local_planner --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 3: Review the complete diff**

Run:

```bash
git diff a536930..HEAD --check
git diff a536930..HEAD -- src/dddmr_local_planner/local_planner
git status --short
```

Expected: only MPPI controller, tests, diagnostics, and the user's pre-existing `.gitignore` modification are visible; `.gitignore` is not staged or committed.

- [ ] **Step 4: Record real-robot validation expectations**

On the next robot run, require `phase_compatible>0`, a material command sign matching `selected_side` during the outward half, monotonic execution-driven phase progress, then a same-side countersteer. Collision counts and final optimal validation remain the safety indicators.
