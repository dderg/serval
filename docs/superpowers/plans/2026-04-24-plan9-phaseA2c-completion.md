# Plan 9 Phase A2c — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Branch:** `magnum-opus`
**Commits:**
- `a40e89eb` plan9-A2c: add max_jerk config knob on ToolHead
- `001815a0` plan9-A2c: Move captures j_max from toolhead.max_jerk
- `5ca6deef` plan9-A2c: add Move.reachable_v_from_v_end helper
- `bd2be5fe` plan9-A2c: jerk-aware LookAheadQueue.flush reverse pass
- `a472aba8` plan9-A2c: Move.set_junction uses jerk_profile.compute_profile
- `8128c85b` plan9-A2c: end-to-end jerk integration test

## What shipped

- `[printer] max_jerk` is a real user knob, default 100000 mm/s³, live-mutable via `SET_VELOCITY_LIMIT JERK=`. Surfaced in `get_status` and restored on `RESET_VELOCITY_LIMIT`.
- `Move.j_max` snapshot from toolhead at Move construction; unchanged by `limit_speed`.
- `Move.reachable_v_from_v_end(v_end)` — jerk-aware reachable velocity via A2b's `jerk_math.reachable_v_end`. Symmetry of the accel-side jerk profile means the reverse pass reuses the same primitive.
- `LookAheadQueue.flush` reverse pass uses jerk-aware reachable-velocity on BOTH the regular path (using `move.accel`) and the smoothed-accel path (using `move.toolhead.max_accel_to_decel`). Remaining loop body (delayed-move handling, peak_cruise_v2 propagation, set_junction call) unchanged.
- `Move.set_junction` computes and stores a `jerk_profile.compute_profile` result. Back-compat `accel_t/cruise_t/decel_t/start_v/cruise_v/end_v` populated by collapsing the 7-segment profile (J+,A+,J- → accel_t; C → cruise_t; J-d,A-,J+d → decel_t). The trapezoid-in-v integral of the populated fields equals `move_d` by construction (verified in `test_set_junction_integrated_distance_equals_move_d`). `self.accel` is intentionally left at its pre-set_junction value — the emit-path baker's quadratic `(start_v+cruise_v)*0.5*accel_t` identity holds regardless of what `accel` we carry.
- `Move.calc_junction` centripetal cap rewritten as the physical form `0.5 * move_d * accel * tan(θ/2)`, decoupling it from the constant-accel `delta_v2` approximation. Numerically identical to the prior form; the decoupling lets future work retire the `delta_v2 = 2*move_d*accel` precomputation.

## Validation

- `test/test_toolhead_jerk_wiring.py`: 10 new passing unit tests (j_max capture, reachable_v helper, flush reverse pass distinguishing 215 vs 316 mm/s, 4 set_junction tests).
- `test/test_toolhead_jerk_integration.py`: 5 new integration tests — 2 config-loading tests through `PrinterShim`, and 3 real `Move` + `LookAheadQueue` end-to-end tests distinguishing jerk from trapezoid math.
- Targeted Plan 9 suite (10 files): 573 passed, 4 skipped, 0 failed.
- Full repo suite: 879 passed, 84 failed (all pre-existing env issues unrelated to A2c), 4 skipped, 1 xfailed.
- A Task 2 review-gap was discovered and fixed in Task 4: two `_StubToolhead` classes in `test/test_blendplanner.py` needed `max_jerk` added. Flagged as a lesson for future `Move.__init__` additions — always grep `test/` for direct Move constructor calls to find stubs missing required attributes.

## Known limits (A2c scope)

- Emit path is still trapezoidal — kinematic moves go through `append_trapezoid_as_quintic`; the jerk profile stored on `Move.jerk_profile` is not yet routed to trapq. **A2d flips emit.**
- `QuinticBlendMove` in `blendplanner.py` retains its TOPP trapezoid-in-s profile and its own `delta_v2` formula. A3 / A2d integrates blend moves with the jerk math.
- `Move.calc_junction`'s forward cap (`prev.max_start_v2 + prev.delta_v2`) still uses the constant-accel approximation. Safe (upper bound; reverse pass retightens) but lossy.
- Per-axis `max_jerk_x/y/z/e` not plumbed — scalar `max_jerk` only. A4 adds kinematic coupling.
- Extruder emit path (`extruder.move` → `append_trapezoid_e_only_as_quintic`) reads the back-compat `accel_t/cruise_t/decel_t` fields. Trapezoidal approximation; A5 rewrites with a jerk-aware E polynomial.
- The integration test bypasses the prepass/blender `BlendPipelineLookAheadQueue` wrapper that production `ToolHead` uses — some moderate overlap with Task 5 unit tests. A future phase with richer harness infrastructure can close both gaps.
- The smoothed reverse pass accesses `move.toolhead.max_accel_to_decel` as a runtime-snapshot (not a per-move attribute). Consistent with pre-A2c semantics; future work can promote to a per-move snapshot if SET_VELOCITY_LIMIT live-mutation correctness matters for in-flight moves.

## Next — A2d

Scope candidates for A2d:
- Route `Move.jerk_profile` through `append_jerk_profile_as_quintic` (A2a's emitter) in `_process_moves`. Requires coordination with the extruder emit path — likely bundled with A5.
- Retire `Move.delta_v2` forward cap in `calc_junction`, replacing with jerk-aware reachable via `reachable_v_from_v_end`.
- `LookAheadQueue.flush` rewrite as an explicit two-pass (forward + reverse) with jerk-aware cruise_v² computation.
- End-to-end `test_plan9_integration.py` that validates multi-move convergence at the ToolHead level (with real prepass/blender wrapping).
- Evaluate the `max_accel_to_decel` per-move snapshot question (minor).
