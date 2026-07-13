# MPPI Ackermann Obstacle-Avoidance Stability Design

## Context

The current controller detects obstacles and finds collision-free samples, but it does not commit to one side of a centered obstacle. Runtime evidence shows best samples repeatedly reaching the Ackermann steering limit while the weighted command remains near zero and often has the opposite sign. The vehicle therefore continues forward until nearly all sampled trajectories collide.

The failure is caused by the sampling and aggregation behavior rather than missing collision detection:

- The 96-sample noise tensor is regenerated every optimization cycle, so symmetric left/right solutions change dominance between cycles.
- At approximately 0.2 m/s, the Ackermann angular-velocity limit is about 0.105 rad/s, while `wz_std` is 0.4 rad/s. Most angular samples are clipped to either steering limit and can flip sign independently at adjacent time steps.
- Independent per-step noise rarely produces the sustained arc or arc-counterarc shape required to pass an obstacle.
- Softmax arithmetic averaging can combine a valid left trajectory and a valid right trajectory into a near-straight control sequence. The collision-free trajectory set is non-convex even though the per-step control bounds are convex.
- Path occupancy currently uses the obstacle repulsion radius. This disables PathAlign as soon as obstacles enter the broad 1.5 m influence region, removing a useful continuous path-shape term too early.

## Goals

- Produce temporally coherent Ackermann candidate trajectories that include sustained turns and S-shaped bypass maneuvers.
- Keep the selected bypass side stable across control cycles.
- Preserve the existing full-sequence collision validator and fail-closed behavior.
- Keep the 15 Hz runtime budget by retaining a 96-sample batch.
- Expose enough optimizer diagnostics to prove whether left/right modes are still cancelling.

## Non-goals

- Replacing the controller with the Nav2 plugin or adding a costmap dependency.
- Raising the batch size to Nav2's typical 1000 samples.
- Changing the global planner or blocked-wait state machine.
- Adding reverse-motion recovery in this change.

## Design

### 1. Stable noise lifecycle

Add a `regenerate_noises` MPPI parameter, defaulting to `false`, matching Nav2's stable-noise default. Generate the random noise tensor during configuration/reset and reuse it across control cycles. If enabled, regenerate it once per optimizer iteration for diagnostic compatibility.

Resetting the nominal control sequence must not implicitly replace the noise tensor. This lets warm-start updates accumulate a consistent directional bias instead of seeing a different random left/right population every cycle.

### 2. Ackermann-aware coherent candidates

Retain the configured batch size. Reserve a small deterministic portion of each batch for paired maneuver templates:

- sustained left and right curvature;
- left-then-right and right-then-left S curves;
- two intensity levels for each shape.

Templates are expressed as fractions of the speed-dependent Ackermann angular-velocity limit, then passed through the existing acceleration and hard-control constraints. The remaining candidates use random noise, but angular noise is normalized to the current feasible angular-velocity range instead of allowing a 0.4 rad/s distribution to collapse into a two-point clipped distribution.

The templates remain paired to avoid introducing a fixed left/right preference. Direction selection comes from trajectory costs and the existing warm-start sequence.

### 3. Nav2-style control-sequence filtering

Store the last four commands actually returned by the optimizer. After the softmax update, apply Nav2's nine-point quadratic Savitzky-Golay filter to `vx` and `wz`, using command history at the beginning of the sequence. Reapply acceleration, velocity, angular-velocity, and Ackermann curvature constraints after filtering.

Filtering is skipped for sequences too short to support the nine-point window. Reset operations also reset command history to the measured/reset command so stale steering is not carried into a new route.

### 4. Path occupancy semantics

Separate PathAlign validity from obstacle repulsion distance. For each path pose, use the transformed robot cuboid and collision margin to decide whether the pose is occupied. Calculate occupancy from those actually blocked path points, not every obstacle within `inflation_radius`.

Only disable PathAlign when there are at least three blocked path points and their ratio exceeds `max_path_occupancy_ratio`, following the intent of Nav2's invalid-path handling. `MppiPathPoint::obstacle_free` uses the same footprint-aware result, so PathFollow does not select a target whose robot footprint intersects an obstacle.

### 5. Parameter calibration

Keep `batch_size=96` and `iteration_count=1`. Reduce `wz_std` from 0.4 to a feasible Ackermann-scale value; 0.08 rad/s is the initial value for this platform. Increase the repulsion influence radius from 1.5 m to 2.2 m because the configured minimum turning radius is approximately 1.9 m. The radius affects obstacle preference only and no longer disables PathAlign.

Do not increase the repulsion weight in the first implementation. Earlier activation plus coherent candidates isolates the sampling fix from critic-weight changes.

### 6. Diagnostics

Extend `MppiResult` with optimizer-level statistics:

- effective sample size from normalized softmax weights;
- positive, negative, and near-zero command-step weight mass;
- weighted command before filtering;
- number of structured candidates.

Add these fields to the throttled MPPI debug line. A successful centered-obstacle run should show one directional weight mass becoming dominant and `cmd_wz` retaining that sign before the collision count grows.

## Error handling and safety

- Candidate construction continues to use finite-value checks and hard Ackermann constraints.
- If all scores are invalid, behavior remains fail-closed and triggers the existing blocked/replanning path.
- The filtered optimal sequence is revalidated by the existing full-sequence collision checker.
- If filtering produces an invalid value, retain the last constrained unfiltered sequence rather than emitting a non-finite command.

## Testing

Unit tests will be written before production changes and must cover:

- noise reuse across cycles when `regenerate_noises=false` and regeneration when true;
- paired coherent maneuver shapes and speed-dependent Ackermann limits;
- Savitzky-Golay filtering with command-history boundary handling and post-filter constraints;
- footprint-based path occupancy independent of repulsion radius;
- a symmetric synthetic scoring case where coherent candidates and warm start preserve a selected turn direction;
- correctness and finiteness of weight-mass/effective-sample-size diagnostics.

The focused unit suite, complete `local_planner` tests, `p2p_move_base` configuration tests, and package builds must pass. Final confirmation still requires an on-robot run because the repository does not contain a replayable obstacle snapshot with synchronized odometry.

## Success criteria

- In the same centered-obstacle scenario, the commanded angular velocity develops and maintains one sign while safe bypass candidates exist.
- The best-sample and commanded steering signs no longer repeatedly oppose each other during the approach.
- Collision trajectories do not monotonically grow to 96 because the command stayed near zero.
- The controller remains within the existing runtime envelope without increasing the batch size.
- All automated tests and builds pass.
