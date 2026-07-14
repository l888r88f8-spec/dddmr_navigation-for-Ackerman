# MPPI Geometric Bypass Gate Design

## Problem

The current Ackermann MPPI controller treats a fixed number of compatible control cycles as proof that a bypass maneuver has progressed. With the deployed 5 s horizon, 15 Hz loop, approximately 1.9 m minimum turning radius, and observed speeds around 0.15--0.20 m/s, the first half of the fixed S curve produces too little lateral displacement. The controller begins counter-steering while the vehicle is still close to the reference path. When the fixed phase reaches its final step, the directional command filter is removed even though the reference path remains blocked. The optimal sequence then alternates among outward steering, counter-steering, and braking until no collision-free candidate remains.

The bypass side latch is still useful because the 96-candidate deployment batch is much smaller than a typical Nav2 MPPI batch and otherwise averages left and right solutions. The repair must retain that latch while replacing elapsed-cycle progress with observed geometry.

## Selected Design

Use a monotonic, geometry-gated avoidance stage with three externally visible values:

- `neutral`: the reference path is not blocked and all valid candidates may contribute.
- `depart`: the reference path is blocked, a physical bypass side is latched, and the vehicle has not yet established sufficient lateral separation from the reference path.
- `bypass`: the vehicle has established lateral separation. The physical bypass side remains latched, but the immediate angular-velocity sign is no longer forced so the critic-weighted optimizer can counter-steer, travel past the obstacle, and stabilize its heading.

The only normal transitions are:

```text
neutral -> depart -> bypass -> neutral
```

A confirmed physical side switch resets `depart` for the new side. While the path remains blocked, `bypass` never falls back to `depart` because temporary pose noise around the lateral threshold must not reverse the maneuver state.

## Geometry Signal

Project the current robot position onto the closest valid segment of the pruned reference path using the same signed-offset convention as `MppiBypassSideClassifier`: positive is left and negative is right. Expose this projection through the controller's per-cycle directional-mode context.

The `depart -> bypass` transition occurs when:

```text
latched_side * signed_robot_path_offset >= departure_lateral_offset
```

`departure_lateral_offset` is an MPPI parameter. Its deployment value is `0.42 m`, derived from the configured vehicle half-width (`0.32 m`) plus obstacle collision margin (`0.10 m`). Invalid or unavailable path geometry cannot complete departure; it keeps the conservative outward commitment until normal failure handling requests replanning.

The path-blocked signal remains `blocked_path_points >= 3`. When that signal clears, avoidance stage and side state reset to neutral. Collision checking and the final optimal-trajectory validator remain authoritative for every stage.

## Candidate Selection and Generation

During `depart`:

- Select only collision-free candidates classified on the latched physical side.
- Require their immediate command to have material angular velocity toward the latched side.
- Generate the two latched structured candidates as continuous outward arcs at different intensities. They no longer switch direction halfway through an elapsed-time template.

During `bypass`:

- Continue selecting only collision-free candidates classified on the latched physical side.
- Do not filter them by immediate angular-velocity sign.
- Provide one structured counter-steering candidate and one low-curvature/straight candidate so the 96-sample batch can recover from the outward control distribution without relying on a rare Gaussian sample.

During `neutral`, retain the existing paired left/right structured candidates used to discover a bypass side.

The prior `structured_phase_step` timer is removed from control decisions and diagnostics. Diagnostics report `avoidance_stage`, signed path offset, and the configured departure offset so field logs show why a transition did or did not occur.

## State Preservation and Failure Handling

Optimization retry snapshots preserve the latched side, side-infeasible confirmation count, and avoidance stage. A retry must not fabricate geometric progress. Controller reset, route replacement, configuration replacement, or an unblocked path resets the stage to neutral. A confirmed side change resets the stage to depart.

If no phase-compatible outward candidate exists in `depart`, optimization fails rather than issuing a command that turns back toward the blocked path. Existing collision-validation retry and route-replan behavior handles this result.

## Verification

Add controller tests that prove:

- Repeated compatible commands cannot complete `depart` while signed lateral offset remains below `0.42 m`, including more cycles than the former 74-step limit.
- Crossing the signed threshold changes the stage to `bypass` and permits a same-side counter-steering command.
- Offset of the wrong sign cannot complete departure.
- `bypass` does not fall back to `depart` if measured offset later dips below the threshold while the path remains blocked.
- Clearing the blocked signal resets stage and side.
- A confirmed side switch returns the new side to `depart`.
- Retry snapshots restore the avoidance stage.
- Deployment-scale 96-candidate selection retains outward commitment before the threshold and has a counter-steering candidate after it.

Add path-projection tests for left, right, degenerate, and invalid paths. Run the focused MPPI tests, the complete local planner test target, and a clean package build before completion.

