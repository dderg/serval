# Plan 9 Phase A5 — completion report

**Status:** COMPLETE
**Date:** 2026-04-25
**Branch:** `magnum-opus`
**Commits (10 total — 1 plan + 8 implementation + 1 acceptance):**
- `84a9b99d` plan9-A5-plan: jerk-native lookahead rewrite design
- `8067f25b` plan9-A5T1: jerk-aware `max_reachable_cruise_v` bisection primitive in `klippy/jerk_math.py`
- `e14a55f4` plan9-A5T1-fixup: correct min vs max predicate in docstring + plan
- `db4e535c` plan9-A5T2: jerk-native `LookAheadQueue.flush` reverse pass; retire smoothed pass / `peak_cruise_v²` averaging / trapezoidal cruise cap; `Move` shrink (no `smooth_delta_v2`/`max_smoothed_v2`/`delta_v2`)
- `d277114a` plan9-A5T2-fixup: `QuinticBlendMove.j_max` parity, drop dead `reachable_v_from_v_end` paths, add forward-cap test
- `5ac4d0e7` plan9-A5T3: `QuinticBlendMove` attribute parity with `Move`; trim `_copy_caller_state` of retired fields
- `12161c94` plan9-A5T4: retire `max_accel_to_decel` / `minimum_cruise_ratio` config + gcode knobs; delete `ToolHead.max_accel_to_decel` property + `MINIMUM_CRUISE_RATIO` constant; fix `resonance_tester` crash from removed property
- `b66c2eba` plan9-A5T4-fixup: drop `MINIMUM_CRUISE_RATIO` references from `klippy/extras/resonance_tester.py` + sample config
- `31d888cb` plan9-A5T5: test-stub migration (drop `max_accel_to_decel`/`smooth_delta_v2`/`max_smoothed_v2` plumbing); flip `getattr` shim direction; QBM parity test
- this commit — plan9-A5T6: bed_mesh acceptance test + completion report

**Plus orthogonal hardware-deploy fix (not A5 itself, landed in the same wave):**
- `a4945a24` kalico-fix: extruder stepper `active_flags=AF_E` (latent chunk3 bug, surfaced when `magnum-opus` first deployed to Trident hardware)

## What shipped

- **`jerk_math.max_reachable_cruise_v(v_start, v_end, a_max, j_max, L, v_cruise_cap)`** — closed-form-equivalent bisection primitive that returns the largest `cruise_v ≤ v_cruise_cap` such that an accel ramp from `v_start` to `cruise_v` and a decel ramp from `cruise_v` to `v_end` together fit in `L` under `(a_max, j_max)` jerk-limited motion. Bisects on `L_accel ∈ [0, L]` (the split point of accel-side runway), exploiting the monotonicity of `reachable_v_end` to drive `ramp_from_start(L_accel) → ramp_from_end(L - L_accel)`. 25 iterations converges to 1e-8 mm precision in single-digit microseconds. Two short-circuit paths: trivial-at-cruise (both ramps reach `v_cruise_cap` independently) and infeasible-end (`reachable_v_end(start_v, …, L) <= v_end`, which triggers a clamp upstream).

- **Jerk-native `LookAheadQueue.flush` reverse pass.** Today's flush propagates `next_end_v²` backwards; at each `Move` it clips `cruise_v` via `max_reachable_cruise_v`, tightens `start_v² = min(start_v², cruise_v²)` and `end_v² = min(end_v², cruise_v²)` so the tuple `set_junction` receives is jerk-feasible **by construction**, then calls `set_junction`. Gone: the smoothed pass (`reachable_smoothed_v²`, `delayed[]`, `peak_cruise_v²` averaging), the trapezoidal cruise cap `(start_v² + reachable_start_v²) * 0.5`, the `delta_v²` forward cap in `calc_junction` (replaced with `reachable_v_end(prev_start_v, prev_accel, prev_j_max, prev_move.move_d)²`). One single-pass loop handles both kinematic and pure-E moves uniformly — pure-E moves carry the toolhead's `max_jerk` and a non-binding accel sentinel (`99999999.9`), so the same primitive applies.

- **`Move` attribute shrink.** Retired fields: `delta_v2`, `smooth_delta_v2`, `max_smoothed_v2`. The forward reachability that `delta_v2` backed is now computed inline from `reachable_v_end`. The smoothed pass is gone, so `smooth_delta_v2` and `max_smoothed_v2` had no remaining consumers.

- **`QuinticBlendMove` attribute parity with `Move`.** The blender's `QuinticBlendMove` carries the same retired-field shape as `Move` (so any code that introspects either class via duck-typing sees a consistent surface). `j_max` is now a first-class attribute on QBM matching `Move.j_max` for downstream symmetry. `_copy_caller_state` no longer plumbs `smooth_delta_v2` / `max_smoothed_v2` / `delta_v2`.

- **Retired config / gcode knobs.** Deleted: `max_accel_to_decel` (config + the `ToolHead.max_accel_to_decel` property + the deprecation branch in `ToolHead.__init__`), `minimum_cruise_ratio` (config), `MINIMUM_CRUISE_RATIO` (module-level constant in `toolhead.py`), the `ACCEL_TO_DECEL` g-code option, and the `max_accel_to_decel` deprecate path in `klippy/extras/trad_rack.py`. The sample config and `resonance_tester` no longer reference these knobs. **Per `feedback_fork_as_gate`**, this is a clean removal — no runtime flags, no compat shim.

- **End-to-end pipeline coverage.** The acceptance gate (`test_a5_bed_mesh_exact_crash_tuple_replay`) drives the exact crash tuple through real `Move` + `LookAheadQueue.flush` + `set_junction` + `compute_profile` + `build_unshaped_payload` + `finalize_shape`. Pre-A5: this raises `klippy.gcode.CommandError: Jerk profile infeasible for move (...)`. Post-A5: flush completes, `cruise_v` is clipped to `~375.86` mm/s, `jerk_profile.status == JP_OK`, and `quintic_trapq_payload` is populated.

## Validation

- **Targeted Plan-9-specific suite: 544 passed, 4 skipped, 0 failed** (was 543 before T6; T6 adds `test_a5_bed_mesh_exact_crash_tuple_replay`). Pre-A5 baseline was 529 — A5 added 15 tests across T1–T6 (T1: 7 `max_reachable_cruise_v` bisection cases in `test_jerk_math.py`; T2: forward-cap test in `test_toolhead_jerk_integration.py` + various scope; T3: QBM parity + j_max test; T4: `test_toolhead_has_no_max_accel_to_decel` + `test_toolhead_has_no_min_cruise_ratio`; T5: stub migration + QBM parity test; T6: bed_mesh acceptance gate).
- **Bed-mesh tuple closure** — verified analytically and by black-box replay through the production pipeline:
  - **Inputs:** `start_v=374.7 mm/s`, `cruise_v_request=469.8 mm/s`, `end_v=469.8 mm/s`, `move_d=1.143 mm`, `accel=70000 mm/s²`, `j_max=500000 mm/s³`.
  - **Pre-A5 verification (verifier-confirmed):** under `j_max=500k`, the 374.7 → 469.8 jerk-aware ramp needs `reachable_v_end → 11.65 mm` of runway; the trapezoidal `(v_end² - v_start²) / (2·a)` formula computes `0.574 mm`. **Off by ~20×.** The trapezoidal cruise cap blessed the tuple; `set_junction` then raised in `jerk_profile.compute_profile`.
  - **Post-A5 verification:** `max_reachable_cruise_v(374.7, 469.8, 70k, 500k, 1.143, 469.8) = 375.86 mm/s` (clipped from the requested 469.8). The `(start_v=374.7, cruise_v=375.86, end_v=375.86, L=1.143, a=70k, j=500k)` tuple is jerk-feasible, `compute_profile` returns `JP_OK`, the move emits a 5-phase polynomial, and `quintic_trapq_payload` is populated.
- **Subagent verification of the bisection math** (T1) — the verifier independently re-derived the monotonicity argument (`ramp_from_start` monotone increasing in `L_accel`, `ramp_from_end` monotone decreasing) and confirmed bisection convergence on the crossover point.
- **No regressions** in `test_blendextruder_integration.py`, `test_chunk3_pa_integration.py`, `test_blendplanner.py`, `test_blendprepass.py`, `test_toolhead_jerk_wiring.py`, `test_toolhead_shape_bake.py`, `test_toolhead_shape_bake_pipeline.py`, `test_plan5_integration.py`.

## Architecture notes

- **Bisection on `L_accel` split point, not closed-form on `cruise_v`.** `reachable_v_end` is itself a two-regime piecewise function (triangular vs trapezoidal jerk profile, see A2b). Chasing a closed analytic solution for the `cruise_v` crossover across regime boundaries is brittle; bisecting on the runway split is monotone-stable in both regimes. Cost is negligible (~25 calls to `reachable_v_end` per move; one call is single-digit µs).

- **Single-pass reverse loop.** The Klipper-era second "smoothed" pass existed because trapezoidal motion is snap-limited and a gentler smoothed-acceleration pass produced better cornering decisions. Under jerk-limited motion, the smoothness is **already in** the per-move jerk profile — the smoothed pass was dead weight. Deleting it eliminated `delayed[]` queue, `peak_cruise_v²` propagation, `max_accel_to_decel` config knob, and ~80 lines of flush logic.

- **Pure-E moves no longer special-cased in flush.** Klipper's old reverse pass tested `is_kinematic_move` to skip the smoothed/centripetal logic for pure-E. With A5, the same `max_reachable_cruise_v` primitive applies — pure-E carries `accel = 99999999.9` (sentinel: effectively non-binding) and `j_max = toolhead.max_jerk`. The bisection collapses to the trivial-at-cruise short-circuit because the sentinel accel makes the ramps fit in any positive runway. So pure-E preserves its old behavior naturally without a branch.

- **Forward cap in `calc_junction` is now jerk-aware.** The old `prev_max_start_v² + prev_delta_v²` term (where `delta_v² = 2 * move_d * accel` is the constant-accel forward reachability) was a huge over-estimate under jerk: with `a=70k`, `move_d=40 mm` it computes `dv ≈ 2366 mm/s`, irrelevant for any physical motion. Replaced with `reachable_v_end(prev_start_v, prev_accel, prev_j_max, prev_move.move_d)²` — the jerk-aware physical bound. Centripetal cap stays in its A2c geometric form (`0.5 * L * accel * tan(θ/2)`) — it's a corner-radius cap, not a trapezoidal artifact.

- **`getattr` shim direction flipped** (T5). Pre-A5, several test stubs read `getattr(toolhead, "max_jerk", legacy_default)`. Post-A5, the production code is the source of truth for `max_jerk` and tests assert the attribute is present rather than tolerating a fallback.

## Hardware regression target

A5 closes the **`bed_mesh_calibrate` "Jerk profile infeasible" crash on Trident**. The crash was triggered every time the user ran `BED_MESH_CALIBRATE` with the post-A2c jerk pipeline, because the trapezoidal cruise cap survived the A2c rewrite (A2c only replaced the accel-side `reachable_v_from_v_end`). A5 replaces the cruise cap with `max_reachable_cruise_v`, making the reverse pass jerk-feasible by construction.

**Hardware validation status:** the bed_mesh crash is **fixed by construction** — the acceptance test exercises the exact crash tuple through the real `Move` + `LookAheadQueue.flush` and confirms no raise. Hardware confirmation on Trident is the immediate next gate.

## Known limits / followups

- **Centripetal cap is still in constant-accel form (per A2c).** The `0.5 * L * accel * tan(θ/2)` corner-radius cap was decoupled from `delta_v²` in A2c, but `accel` is still the per-move accel limit, not a jerk-aware effective cap. Under heavy jerk-limiting at sharp corners, the cap may over-estimate the achievable junction speed. **Verifying / re-deriving the centripetal cap under jerk-limited motion is a separate future task** — A5 deliberately leaves it in its A2c form because it is a geometric cap on radius (not a kinematic bound on velocity along the move), and the bed_mesh crash had no centripetal component (probe moves are linear).

- **Pure-E moves now flow through the jerk-aware reverse pass.** Pre-A5, pure-E was special-cased; post-A5, it goes through `max_reachable_cruise_v` with `accel = 99999999.9` and `j_max = toolhead.max_jerk`. The sentinel accel makes the bisection short-circuit to trivial-at-cruise, so behaviour matches the old path for typical retracts. **However:** if the user sets a finite, low `max_jerk` and runs aggressive retract sequences, retract speeds may now be jerk-clipped where the old path would have allowed them at full speed. Flag if observed in field testing — the fix would be a per-axis or per-extruder `max_jerk` (currently out of scope per the spec).

- **A4 (cross-blend-boundary) is still designed but not executed.** Independent of A5. Plan committed as `docs/superpowers/plans/2026-04-24-plan9-phaseA4-cross-boundary-shape-bake.md`. Not blocking the bed_mesh fix.

- **No integration test through the full upstream pipeline for A5.** All A5 tests inject moves into the inner `LookAheadQueue.queue` directly, bypassing `BlendPipelineLookAheadQueue` → `CollinearCollapser` → `CornerBlender` → inner `LookAheadQueue`. Same gap A3 noted; same followup. Closing it here would require synthesizing a bed_mesh-shaped G-code stream through the prepass — track for follow-up if the hardware Trident test shows residual planner issues.

- **Phase B onwards** — host↔MCU protocol redesign for quintic polynomial emit; MCU firmware (`trapq_append_quintic`); Rust rewrite candidates (per Plan 9 scope expansion 2026-04-24).

## Bonus discovery — chunk3-era extruder fix (`a4945a24`)

While deploying `magnum-opus` to Trident hardware for the first time during the A5 wave, the user surfaced a **latent bug from the chunk3 era** (pre-Plan-9): the extruder stepper was registered with `active_flags=0` instead of `active_flags=AF_E`. This caused the extruder to be excluded from active-stepper-flag checks in some downstream paths. **Not part of Plan 9 A5 itself** — the bug pre-dated the entire Plan 9 work — but landed in the same commit wave because hardware deployment is what surfaced it. Filed as `kalico-fix` rather than `plan9-A5*` to keep the A5 scope clean.
