# MPPI Bypass-Side and Steering-Phase Stability Design

## Context

The first Ackermann MPPI stability change made obstacle-aware trajectories available, but the latest robot run exposes a second failure mode. The controller holds approximately `+0.095 rad/s` for several seconds, switches abruptly to `-0.095 rad/s`, and then alternates direction while approaching the same obstacle. At the switch, the path occupancy remains unchanged and the effective sample size is one, so the change is produced by the optimizer rather than by a new route or perception event.

The current directional-mode implementation equates the sign of the command-step angular velocity with the physical bypass side. That equivalence is false for Ackermann obstacle avoidance: a left-side bypass normally begins with one steering sign and later uses the opposite sign to straighten and rejoin the path. The deterministic S-curve candidates compound the problem because their phase restarts at the beginning of every receding-horizon cycle. Finally, the stochastic control-energy correction is applied to deterministic structured candidates even though those candidates were not drawn from the configured Gaussian distribution. This can produce large negative total costs and a one-sample softmax collapse.

## Goals

- Keep one physical bypass side stable while the same local path segment remains blocked.
- Allow the steering command to change sign as part of a valid same-side countersteer maneuver.
- Advance deterministic maneuver phase across control cycles instead of restarting it every cycle.
- Prevent deterministic candidates from receiving an invalid stochastic control-energy correction.
- Preserve collision validation, fail-closed behavior, the 96-sample runtime budget, and existing Ackermann constraints.

## Non-goals

- Identifying or tracking individual point-cloud obstacle objects.
- Replacing the local planner with Nav2 or adding a 2D costmap dependency.
- Adding reverse motion or changing the global replanning state machine.
- Guaranteeing completion when the selected side has no collision-free Ackermann route.

## Alternatives Considered

### Configuration-only damping

Set `gamma` to zero, increase temperature, reduce angular acceleration, or disable structured candidates. This may reduce oscillation, but it does not correct the mistaken relationship between steering sign and bypass side. Disabling the candidates also risks restoring the original failure to initiate a bypass.

### Steering-sign latch

Hold positive or negative angular velocity for a fixed time. This suppresses rapid switches but prevents the countersteer needed to finish an S maneuver and can make the vehicle follow a long circular arc.

### Topological side with persistent maneuver phase

Represent bypass side separately from instantaneous steering sign, classify candidates using their predicted lateral position relative to the local path, and retain a structured-candidate phase while the bypass is active. This directly addresses both observed failure mechanisms and is the selected design.

## Design

### 1. Candidate bypass-side metadata

Extend each batch score with a bypass-side classification: `Left`, `Right`, or `Neutral`. The classification is calculated from the already generated predicted trajectory, not from its first angular command.

For each trajectory, compute signed lateral offsets against the nearest local-path segment over the useful prediction horizon. Let `peak_offset` be the signed offset with the greatest absolute magnitude and `terminal_offset` be the final signed offset. Use `0.7 * peak_offset + 0.3 * terminal_offset` as the side metric. A metric greater than `0.10 m` is `Left`, one below `-0.10 m` is `Right`, and the remainder is `Neutral`. This means a trajectory can steer right during its countersteer phase while remaining a `Left` bypass because its predicted body position stays on the left side of the blocked path.

The local planner owns this geometry calculation because it already has the candidate trajectories and transformed path. The MPPI controller only consumes the resulting side metadata during aggregation.

### 2. Bypass-side latch and release rules

Add explicit controller state for the selected bypass side. Directional mode remains active only while at least three local path points are footprint-blocked.

When directional mode first activates, calculate softmax mass separately for valid, non-colliding left and right candidates and select the side with greater mass. On an exact tie, select the side of the lowest-cost non-neutral candidate; the stable candidate order resolves a remaining exact cost tie. Once selected, keep that side while it has at least one valid, non-colliding candidate. Neutral candidates do not participate in directional aggregation because including them can recreate the straight collision-course average.

Do not switch merely because the opposite side has a lower softmax cost for one cycle. Switch only after the latched side has no feasible candidate for three consecutive optimization cycles. The first two such cycles return an invalid result to the existing safe stop/retry path. On the third cycle, switch only if the opposite side has a valid candidate; otherwise remain invalid and keep the latch. This short confirmation window rejects single-cycle point-cloud or sampling noise while still allowing escape from a genuinely blocked side.

Clear the latch and its failure counter when directional mode becomes inactive, the controller is reset, a new route is installed, or the existing optimal-sequence validator triggers a reset. Route and reset behavior must not carry a previous obstacle's side preference into a new planning problem.

### 3. Persistent structured-maneuver phase

Associate structured S candidates with their physical bypass side and a controller-owned phase offset. The offset advances by one control step after each successful command while directional mode and the side latch remain active.

For a horizon of `N` controllable steps, define a finite template over `N` absolute phase steps: the first `N/2` request curvature toward the selected side and the remainder request the same intensity in the countersteer direction. Candidate generation uses `absolute_phase = phase_offset + local_step`. Values beyond the finite template request zero angular velocity, producing a straight/path-rejoin tail instead of repeating another S cycle. Consequently, the S transition moves toward the command index as real time advances instead of staying fixed at the midpoint of every newly created horizon.

Reset the phase together with the bypass-side latch. Sustained-arc templates remain available only during the first side-selection cycle. After a side is latched, the structured slots contain phase-advanced S candidates for that side at the configured intensity levels plus their constrained straight-tail variants; they must not contain a newly restarted full-horizon arc. Warm-start random candidates continue to refine the shifted nominal sequence around the structured seed.

### 4. Correct cost treatment

Keep the configured MPPI control-energy correction for random candidates generated from the configured noise tensor. Do not apply that correction to deterministic structured candidates. Their total cost is the critic cost only.

Use the sampled perturbation before acceleration and Ackermann clipping for the correction: the stored longitudinal noise and the angular noise after feasible-range scaling. Do not reconstruct it from the constrained sequence, because clipping can make the reconstructed delta differ from the sampled Gaussian perturbation. This design does not require setting `gamma` to zero globally.

The controller continues to reject non-finite critic scores. Negative total cost is permitted mathematically for stochastic samples, but diagnostics will report critic cost and energy contribution separately so a future collapse can be attributed directly.

### 5. Softmax and output behavior

Directional aggregation filters by latched bypass-side metadata, not command angular-velocity sign. Countersteering candidates on the selected side therefore remain eligible.

The Savitzky-Golay filter remains in place, followed by acceleration, velocity, curvature, and angular-velocity constraints. The bypass-side state does not force a steering sign after filtering. Existing full-sequence validation remains the final authority; no candidate or filtered sequence may bypass collision checking.

### 6. Diagnostics

Extend the throttled MPPI diagnostic with:

- selected bypass side;
- number of feasible left, right, and neutral candidates;
- consecutive latched-side failure count;
- structured maneuver phase;
- best sample index and best sample side;
- critic-only best cost and its control-energy contribution.

These fields distinguish a physical side switch from a valid same-side countersteer and make another effective-sample-size collapse diagnosable without relying on a 2-second sequence of command signs.

## Error Handling and Safety

- If no candidate is valid, retain the current fail-closed result and let the existing retry/replanning path run.
- If the latched side is infeasible for fewer than three cycles, do not substitute an unvalidated command; use the existing safe invalid-result handling for that cycle.
- A side switch never overrides collision validity. It only changes which already valid candidates are eligible for aggregation.
- Non-finite lateral offsets or malformed path segments classify a candidate as neutral and cannot create a side latch.
- All phase-generated controls pass through the existing Ackermann and acceleration constraints.

## Testing

Tests are written and observed failing before production changes.

Controller tests must cover:

- a left-bypass trajectory whose command step countersteers right remains classified and selected as left;
- a one-cycle opposite-side cost advantage does not change the latch;
- three consecutive cycles with no feasible latched-side candidate permit a side change;
- the side latch and phase reset on controller reset and when directional mode clears;
- the structured S transition advances across repeated receding-horizon calls;
- deterministic structured candidates receive zero control-energy correction;
- random candidates retain the configured correction;
- all generated and filtered controls satisfy acceleration and Ackermann limits.

Integration-level tests must cover candidate-side classification from synthetic local paths and trajectories, plus route-reset propagation. Existing critic, collision-validation, configuration, and package tests must remain green.

## Success Criteria

- In the reported centered-obstacle scenario, the selected physical side does not change while that side retains a collision-free candidate.
- A steering-sign reversal used to straighten the vehicle is logged as the same bypass side, not a side switch.
- The structured phase advances monotonically during the maneuver and resets after the blocked segment clears.
- Structured candidates no longer produce large negative energy-biased best costs.
- Effective sample size no longer remains at one solely because a deterministic candidate received stochastic energy bias.
- The vehicle starts a bypass, countersteers, and returns toward the path without the observed repeated left/right full-steering alternation.
- The controller remains within the 15 Hz runtime envelope and all automated tests and builds pass.

Real-robot validation remains required because the repository has no synchronized replay fixture containing the reported obstacle, odometry, and steering response.
