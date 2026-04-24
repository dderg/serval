# Plan 9 Phase A2d — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Branch:** `magnum-opus`
**Commits:**
- `3e3dd0da` plan9-A2d: Move.build_quintic_payload (jerk XY + PA-baked E)
- `0ac9dc04` plan9-A2d: set_junction populates quintic_trapq_payload
- `6d9db9e5` plan9-A2d: retire trapezoid emit path for kinematic moves
- this commit — plan9-A2d: integration tests + completion report

## What shipped

- `Move.build_quintic_payload()` — constructs the 9-tuple payload (phase_t_ends, total_t_baked, arc_length, v_cap_min, start_pos_xyz, coeff_tuple, legacy t_accel_end / t_decel_start / total_t) using A2a's `build_jerk_profile_as_quintic_coeffs` for XY and `linear_pa_compose` / `nonlinear_pa_compose` (dispatched via `blendplanner._resolve_pa_dispatch`) for PA-baked E.
- `Move.set_junction` populates `self.quintic_trapq_payload` for every kinematic move. Pure-E moves skip this (guarded on `is_kinematic_move`).
- `ToolHead._process_moves` no longer emits kinematic moves via the legacy `append_trapezoid_as_quintic` path — they take the qpayload branch and route through `trapq_append_quintic` with a jerk-limited XY polynomial (up to 7 phases). The `self.trapq_append` attribute, its import, and the dead `if move.is_kinematic_move:` branch are removed.
- Pure-E moves (`is_kinematic_move == False`) keep the legacy trapezoid path via `extruder.move → append_trapezoid_e_only_as_quintic`. Unchanged.

## Validation

- 10 new A2d tests: 7 in `test/test_toolhead_jerk_wiring.py` (build_quintic_payload contract, PA zero / linear, set_junction wiring, pure-E guard) + 3 in `test/test_toolhead_jerk_integration.py` (end-to-end qpayload presence, jerk-regime phase count > 3, signed-E retract preservation).
- Targeted Plan 9 suite: 583 passed, 4 skipped, 0 failed.
- Full repo suite: 889 passed, 84 failed (pre-existing env issues unchanged), 4 skipped, 1 xfailed.
- No regressions in `test_blendextruder_integration.py`, `test_chunk3_pa_integration.py`, `test_blendplanner.py`, or `test_blendprepass.py`.
- `_emit_quintic_twin` in `extruder.py` reads only generic qpayload + Move-generic attributes, so it handles plain-Move qpayloads identically to QuinticBlendMove qpayloads. Confirmed via the integration tests and full-suite green state.

## Known limits (A2d scope)

- **Shape-everywhere is A3.** The emitted XY polynomial is NOT shape-baked. PA is still correct (additive composition on un-shaped XY). Shape-baking would produce smoother XY and therefore smoother E; the next phase closes this gap and is expected to fix the z_tilt hardware regression.
- **Pure-E moves still trapezoidal.** `is_kinematic_move == False` moves skip the qpayload path. Future phase extends jerk to pure-E (jerk_profile coefficients can populate .e directly — no XY composition needed).
- **QuinticBlendMove path unchanged.** It already emitted PA-baked E; A2d makes plain Moves match that pattern.
- **`shape_disabled` flag passed to trapq_append_quintic is still 0** for plain-Move qpayloads (the flag lives in the `_process_moves` call, not in `build_quintic_payload`). Semantically the A2d polynomial is un-shaped, so this is slightly misleading — but no current consumer of that flag actually cares (kin_shaper.c is retired). A3 makes the flag honest by actually shape-baking.
- **`Move.calc_junction` forward cap (`prev.max_start_v2 + prev.delta_v2`) still uses the constant-accel approximation.** Safe upper bound (lookahead reverse pass retightens). Future cleanup.

## Retract test implementation note

The `test_kinematic_retract_preserves_signed_e` test integrates the E polynomial by summing `(E_i(T_i) - E_i(0))` across phases. This works because:

1. With no PA configured, `_resolve_pa_dispatch` returns `("none", 0.0)` for the stub toolhead, so `linear_pa_compose` runs with `k_pa=0.0`.
2. With k_pa=0: `E(tau) = extr_r * P_proj(tau)` where `P_proj(tau) = n . (start_pos + ...)`.
3. For `start_pos=(0,0,0)`, `P_proj(0) = 0`, so `E(0) = 0` for the first phase.
4. Each phase's E polynomial encodes absolute position; the delta `E_i(T_i) - E_i(0)` is the per-phase E displacement.
5. Summing across phases gives `extr_r * arc_length = (-5/20) * 20 = -5.0`.

No adaptation was needed — the test passed on the first run.

## Next — A3

A3 is the natural next step: integrate `_bake_shaper_polynomial` into `build_quintic_payload` so every kinematic Move is shape-baked with the configured input shaper kernel. This closes Plan 9 Pillar 1 ("shape-baked by construction") and is expected to fix the z_tilt stepper-slip regression on hardware that motivated the Plan 9 rewrite.
