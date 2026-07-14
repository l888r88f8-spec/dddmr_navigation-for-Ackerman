# MPPI Early Bypass Commitment Design

## Context

The deployed Ackermann MPPI controller detects the obstacle and keeps a stable physical bypass side, but it still drives nearly straight. Once the controller latches `Right`, almost every five-second rollout is classified as `Right` even though the command-step angular velocities still contain both signs. Averaging those rollouts cancels the immediate steering action. The two structured rollouts carry only `0` to `2.7%` of the selected softmax mass, so the current `50%` structured-mass condition leaves the maneuver phase at zero indefinitely.

The fix must preserve the physical-side latch and permit a later countersteer on the same side. It must not advance a time-based S-curve while the filtered command has not actually executed the corresponding steering phase.

## Approaches Considered

### Force a minimum structured softmax mass

Rescale the two structured candidates to own at least half of the selected weight. This produces an outward command, but the threshold is arbitrary and mixes absolute control sequences after scoring. A weighted average of individually collision-free trajectories is not necessarily collision-free, so this also increases reliance on the retry validator.

### Replace the nominal sequence with a structured template

After side selection, use a structured S-curve as the nominal sequence and sample all random perturbations around it. This matches the original intent that random samples refine a committed maneuver, but choosing between the two template intensities adds controller state and makes the change larger than necessary.

### Phase-compatible selected-set filtering

Keep the current nominal sequence and physical-side latch, but add a second eligibility condition during the finite maneuver: the command-step control must match the steering direction required by the current maneuver phase and must have material Ackermann curvature. Aggregate only collision-free candidates that satisfy both the physical-side and current-phase conditions. Advance the phase only after the final filtered command materially matches that phase. This directly closes the observed seam while preserving cost-based optimization among all compatible random and structured rollouts.

This is the selected approach.

## Design

### Phase-direction helper

The controller will expose one internal helper that maps the latched physical side and `structured_phase_step` to an expected command sign:

- first half of the controllable horizon: steer toward the selected bypass side;
- second half: countersteer while retaining the same physical side;
- completed phase: no steering-sign restriction.

`Left` remains positive and `Right` negative, matching the structured-template generator. The helper will use the same shifted command index and phase boundaries as `buildStructuredSequence()` so generation, selection, and progress cannot disagree.

### Phase-compatible aggregation

After choosing or restoring the physical side, directional aggregation will require:

1. valid, finite score;
2. collision-free rollout;
3. rollout classified on the latched physical side;
4. during an active maneuver phase, command-step angular velocity in the expected phase direction with a minimum fraction of that command's Ackermann angular limit.

The minimum fraction prevents a rollout with an infinitesimal correct sign from postponing useful lateral motion. Both structured and random rollouts may remain eligible, so critics still select speed, intensity, and longer-horizon shape. A same-side countersteering rollout becomes eligible automatically in the second phase.

If the physical side has collision-free rollouts but none are phase-compatible, the update is invalid and uses the existing stop/retry path. It must not silently fall back to the side-only average that caused the regression. Side switching continues to use physical collision feasibility and its existing three-cycle confirmation; failure to meet a maneuver phase does not immediately relabel physical topology.

### Execution-driven phase progress

Remove the `structured_weight_mass >= 0.5` phase gate. After smoothing and constraints, compare the actual command against the current expected phase direction and Ackermann limit. Increment one phase step only when the issued command has material curvature in that direction. If filtering or acceleration history delays a steering reversal, the phase remains fixed and subsequent cycles keep requesting that phase until the output catches up.

`structured_weight_mass` remains diagnostic only. Reset, route replacement, directional-mode release, and confirmed side changes retain their existing phase reset behavior.

### Diagnostics

Add the number of phase-compatible candidates to the existing MPPI debug record. This separates these cases:

- physical side unavailable;
- physical side available but no rollout commits the required immediate motion;
- compatible rollouts available but the filtered command has not yet reached material curvature.

## Safety and Error Handling

- Collision-free metadata remains mandatory before a candidate enters directional aggregation.
- The existing full weighted-sequence rollout validator remains authoritative after aggregation.
- No forced command bypasses Ackermann, acceleration, velocity, smoothing, or final collision checks.
- A missing phase-compatible set returns an invalid optimization result and follows the existing safe retry/replanning flow.
- Completed maneuvers keep physical-side filtering but remove the steering-sign restriction so normal path rejoin optimization can continue.

## Testing

Controller regression tests will first reproduce the deployed failure:

- deployment-scale candidates are all marked on one physical side;
- two opposite command-step steering samples have costs chosen to cancel in the old side-only softmax;
- the new selection must retain only the phase-compatible outward sample and produce material prefilter steering;
- a random, non-structured winning command that materially executes the expected phase must advance phase, proving progress no longer depends on structured softmax mass;
- an opposite or sub-threshold filtered command must not advance phase;
- countersteer becomes eligible after the midpoint without changing the physical-side latch;
- no compatible candidate returns an invalid result.

Existing phase-reset, side-switch, Ackermann-limit, smoothing, collision-validation, critic, and package tests must remain green. A clean package build and full `colcon test` run are required before completion.

## Success Criteria

- In the reported obstacle case, `selected_side` remains stable and `phase_compatible_candidates` is nonzero while a bypass is viable.
- The issued command establishes material curvature toward the selected side instead of averaging near zero.
- Maneuver phase advances only with matching issued controls, then permits the same-side countersteer.
- The controller fails safely when no rollout can execute the required phase.
- The 15 Hz scoring budget and existing final trajectory collision validation are preserved.
