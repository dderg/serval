# Plan 9 Phase A1 — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Commits (in order):**
- `ea3dde1c` — plan9-A1: scaffold jerk_profile module
- `c46637b6` — plan9-A1: implement accel_side_timings
- `e8c772a8` — plan9-A1: implement find_v_hat bisection
- `41e58f30` — plan9-A1: add build_accel_side helper
- `0e1ff2c9` — plan9-A1: implement jerk_profile_compute dispatch
- `74fd493e` — plan9-A1: 36-case parity sweep vs reference
- `5bc02fff` — plan9-A1: edge-case + robustness tests

## What shipped

- `klippy/chelper/jerk_profile.c` — C implementation of jerk-limited polynomial profile generator
- `klippy/chelper/jerk_profile.h` — public header (7-seg `jerk_profile_result` struct, status enum, three exposed functions)
- `klippy/chelper/jerk_profile.py` — cffi Python wrapper (`Profile`/`Segment` dataclasses, `compute_profile`/`accel_side_timings`/`find_v_hat` entry points)
- `test/test_jerk_profile.py` — 56 tests, all passing
- Registered in `klippy/chelper/__init__.py` (SOURCE_FILES, OTHER_FILES, defs_jerk_profile, defs_all)

## Validation

- **36-case parity sweep** (v0 ∈ {0, 50, 200}, v1 ∈ {0, 50, 200}, L ∈ {1, 10, 100, 1000} mm, a_max=5000, j_max=100000, v_peak=500) vs Python reference (`docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`): all cases match to 1e-9, first-try pass.
- **C² continuity**: verified at every segment boundary to 1e-9 absolute tolerance on the two `test_profile_c2_continuity` cases (symmetric zero-to-zero + asymmetric non-cruise-collapse).
- **Feasibility detection**: agrees with reference on all infeasible cases in the sweep (the four L=1 cases with nonzero endpoint mismatch).
- **Long-cruise precision**: 10 m cruise at 400 mm/s lands at exactly 10000 mm (within 1e-6 abs).
- **Bad-input rejection**: `v_peak=0`, `L<0`, `v_start > v_peak` all return `JP_BAD_INPUT`.
- **Pure-cruise case**: `v0 == v1 == v_peak` produces exactly one cruise segment.
- Full suite: `56 passed in 0.03s`.

## Architecture summary

The implementation mirrors the Python reference's factoring:
- `jerk_profile_accel_side_timings` — compute `(t_j, t_a, a_peak, distance)` for a one-sided velocity change under `(a_max, j_max)`. Handles both triangular (accel never reaches `a_max`) and trapezoidal (accel hits `a_max` and holds) regimes.
- `jerk_profile_find_v_hat` — bisection over `[max(v0,v1), v_peak]` to find the reduced peak velocity when a full-peak cruise doesn't fit in `L`. Monotonic function on the bracket; 80 iterations with `1e-12 * (v_hi+1.0)` stop criterion.
- `build_accel_side` (static) — emits up to 3 polynomial segments (J+/A+/J-) describing one accel side. Handles sign flip for decel (`v_end < v_start`) via a `sign` variable so decel-side segments are tagged correctly (`J-d`/`A-`/`J+d`).
- `jerk_profile_compute` — top-level dispatch: (1) input validation, (2) feasibility check (`L >= d_floor`), (3) full-peak-fits branch OR reduced-v_hat branch, (4) two `build_accel_side` calls + optional cruise.

All math is in `fp64` throughout — `fp32` was rejected in the derivation phase for unacceptable precision loss on long moves.

## Known limits (Phase A1 scope)

- **Single-move only.** The generator produces a profile for one (v0, v1, v_peak, a_max, j_max, L) tuple. Lookahead queue integration is Phase A2.
- **No kinematic coupling.** `v0, v1` are 1-D scalars. CoreXY / delta / polar per-axis scaling is Phase A4.
- **No extruder constraint coupling.** Extruder caps (`max_extruder_accel`, `max_extruder_rpm`) and PA model derivatives are Phase A5.
- **No shape baking.** The output is a raw un-shaped polynomial; shaper convolution is Phase A3.
- **No trapq integration.** The output is a `jerk_profile_result` struct, not a trapq move. Wiring into the trapq polynomial slots is part of Phase A2/A3.

## Residual code-quality suggestions (deferred)

Non-blocking suggestions from the per-task reviews, deferred to a cleanup pass:
- `__visible` keyword placement inconsistent with sibling chelper files (other files use `<rettype> __visible\n<name>(...)` convention).
- `extern "C"` guard in `jerk_profile.h` is unique to this file among chelper headers.
- Stale Python wrapper docstring calls `find_v_hat` "Newton-Raphson" instead of "bisection".
- Unused imports in the test file (`math`, specific constants) — will likely be consumed by later phases.

## Next — Phase A2

Wire `jerk_profile_compute` into a new `LookAheadQueue` that performs:
- Velocity matching at non-blended junctions (1D pass, left-to-right + right-to-left)
- Collinear move merging (preprocess, 4-gate CollinearCollapser semantics per spec)
- Corner blend detection and insertion (quintic blend primitive from Plan 8)
- `a=0` enforcement at programmed (non-blended) junctions

Phase A2 produces the first end-to-end multi-move pipeline: gcode → lookahead → per-move jerk_profile_compute → polynomial segments (still feeding existing trapq/itersolve in Phase A, MCU-side replacement is Phase C).
