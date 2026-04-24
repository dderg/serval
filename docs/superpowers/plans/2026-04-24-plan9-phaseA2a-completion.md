# Plan 9 Phase A2a — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Commits:**
- `da9c7d55` — plan9-A2a: jerk-profile → quintic emitter plan
- `b7b046d0` — plan9-A2a: scaffold jerk_profile → quintic emitter
- `7c93fd12` — plan9-A2a: implement jerk_profile → quintic emitter
- `3dbf1e7a` — plan9-A2a: round-trip emitter validation test

## What shipped

- `klippy/chelper/linear_quintic.c` — new `build_jerk_profile_as_quintic_coeffs` C function (non-static, `__visible`). Translates a 1-D `jerk_profile_result` into the multi-axis quintic-trapq slot layout (up to MOVE_MAX_PIECES=32 phases × 15 coeffs × 4 axes). Added `#include "jerk_profile.h"`, `#include "trapq.h"`, `#include <stddef.h>` to the file.
- `klippy/chelper/linear_quintic.py` — new Python wrapper `build_jerk_profile_as_quintic_coeffs` with Profile→C-struct marshaling and MOVE_MAX_PIECES / QUINTIC_SLOT_COEFFS / QUINTIC_AXES constants exposed.
- `klippy/chelper/__init__.py` — extended `defs_jerk_profile` cdef block with the new function declaration.
- `test/test_linear_as_jerk_profile.py` — 6 tests: phase-count population, X-axis position fidelity, 3D direction projection, start-position offset, bad-profile rejection, and 1-D round-trip eval on a single-axis 50 mm move.

## Validation

- 6/6 new tests pass.
- 56/56 Phase A1 tests (`test/test_jerk_profile.py`) still pass — no regression.
- Combined suite: 62/62 PASS.

## Architecture notes

- Emitter is fully additive; `build_linear_as_quintic_coeffs` (the trapezoidal path) is untouched and continues to work for existing code.
- Axis-E (index 3) is always set to 0 by the emitter — extruder integration is Phase A5 scope.
- The C2 continuity across phase boundaries is inherent to the Phase A1 `build_accel_side` implementation (threads `*p_cursor` as an absolute scalar across all segments), so the emitter simply copies segment coefficients with axis-ratio scaling and a start-position offset on each c0.

## Known limits (A2a scope)

- Still no lookahead integration — that's A2c.
- Still no trapq_append wrapper function — callers use `trapq_append_quintic` directly with `coeff_buf` and `phase_t_ends`. An explicit `append_jerk_profile_as_quintic` convenience wrapper could come later but isn't a blocker.
- Jerk-aware reachable-velocity math (needed by the lookahead reverse pass) is A2b.

## Next — A2b

Derive + implement jerk-aware reachable-velocity math replacing `delta_v2 = 2*move_d*accel` in Move.__init__, LookAheadQueue.flush reverse pass, and Move.calc_junction. Subagent-based math derivation (mirroring the A1 approach) feeds into Python implementation + unit tests.
