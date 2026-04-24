# Plan 9 Phase A2b — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Commits:**
- `3a05c987` — plan9-A2b: reachable-velocity plan + derivation + reference
- `80983156` — plan9-A2b: scaffold jerk_math module
- `100bbfa9` — plan9-A2b: implement reachable_v_end

## What shipped

- `klippy/jerk_math.py` — pure-Python `reachable_v_end(v_start, a_max, j_max, L)` returning the largest v_end reachable from v_start in exactly L distance under (a_max, j_max). Regime-dispatched closed form: Regime A (triangular, depressed cubic via signed cube roots), Regime B (trapezoidal, quadratic). Private helpers: `_signed_cbrt`, `_regime_boundary_distance`, `_reachable_v_end_tri`, `_reachable_v_end_trap`. Input validation raises `ValueError` on negative / non-finite / zero-limits.
- `test/test_jerk_math.py` — 187 tests: 7 basic (sanity, classical-limit, both regime spot checks, zero-distance, monotonicity, bad-input rejection) + 180-case parametrized sweep vs pre-verified Python reference.

## Validation

- **187/187 tests pass** (0.07s).
- **180-case sweep** (v0 ∈ {0, 50, 200, 500}, a_max ∈ {2500, 5000, 10000}, j_max ∈ {50000, 100000, 500000}, L ∈ {0.1, 1, 10, 100, 1000}) matches reference to 1e-9 relative tolerance.
- **Classical-limit test** confirms that as j_max → ∞ the output converges to the old Klipper formula `sqrt(v0² + 2·L·a)` (to 1e-4).
- **Round-trip** against Phase A1's `accel_side_timings` forward primitive: `accel_side_distance(v0, reachable_v_end(v0, a, j, L), a, j) == L` holds to 1e-9 in regime-A and regime-B spot checks.
- **Monotonicity**: v_end strictly increases in L (verified).
- **Bad-input rejection**: negative v0/a/j/L all raise ValueError; L=0 returns v_start exactly.

## Cumulative test count

- Phase A1 (`test_jerk_profile.py`): 56 tests
- Phase A2a (`test_linear_as_jerk_profile.py`): 6 tests
- Phase A2b (`test_jerk_math.py`): 187 tests
- **Total: 249 tests, all passing**

## Architecture notes

- The regime-dispatch + closed-form is pure Python (no C port). Runs at gcode/lookahead rate (~100-500 moves/s), not step-gen rate (~200 kHz). Profiling later can decide if a C port is warranted.
- `reachable_v_end` is the inverse of Phase A1's `accel_side_distance`. This symmetry was derived from scratch and cross-checked against ruckig library source and Biagiotti & Melchiorri §3.4.
- Notable: a widely-cited web source (Analog Devices / Industrial Monitor Direct) contains a dimensionally-wrong S-curve formula. The derivation doc §gotcha explicitly flags this to prevent a C implementer from being misled by web searches.

## Known limits (A2b scope)

- Not yet integrated into `klippy/toolhead.py`. That's Phase A2c.
- The symmetric inverse (decel direction) currently reduces to the same function by argument swap; Phase A2c will decide whether to expose an explicit `reachable_v_start(v_end, a_max, j_max, L)` helper or keep it as a single-argument contract.

## Next — Phase A2c

Wire `reachable_v_end` and `jerk_profile_compute` into `klippy/toolhead.py`:
- `Move.__init__` computes `delta_v2` via `reachable_v_end` instead of `2*move_d*accel`.
- `Move.calc_junction` uses the jerk-aware delta_v2.
- `Move.set_junction` computes and stores a `jerk_profile_result` in place of `accel_t/cruise_t/decel_t`.
- `LookAheadQueue.flush` reverse pass uses `reachable_v_end` for the reachable-start-v computation.
- Must preserve backwards-compat fields (`accel_t`, `cruise_t`, `decel_t`) for downstream consumers (kinematics/extruder, homing, drip_move) until A2d sweeps them.
