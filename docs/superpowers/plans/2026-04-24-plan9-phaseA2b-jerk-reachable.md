# Plan 9 — Phase A2b — Jerk-aware reachable-velocity function

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Add a Python module `klippy/jerk_math.py` exposing `reachable_v_end(v_start, a_max, j_max, L)` — the jerk-aware replacement for Klipper's constant-accel `delta_v2 = 2*L*a` formula. Verified against a 180-case sweep (Phase A2b derivation) to 1e-9 relative error.

**Architecture:** Pure Python implementation of the regime-dispatched closed forms derived in `docs/superpowers/plans/2026-04-24-plan9-phaseA2b-derivation.md`. Regime A (triangular, `L ≤ L_boundary`) uses a depressed-cubic solution with signed cube roots. Regime B (trapezoidal) uses a quadratic solution that collapses to the classical Klipper formula as `j_max → ∞`. Called at gcode/lookahead rate (~100-500 moves/s), NOT at step-gen rate — Python is fine; C port can come later if profiling shows a bottleneck.

**Tech Stack:** Python stdlib (`math`). pytest for tests.

**Reference:**
- Derivation: `docs/superpowers/plans/2026-04-24-plan9-phaseA2b-derivation.md`
- Reference implementation: `docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py` (verified 180/180 cases, max rel error 7.4e-13)
- Phase A1 reference: `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py` (forward primitive `accel_side_timings`)

**Commit policy:** per `feedback_plan9_autonomous_mode.md`, commit after each passing test. No `Co-Authored-By` trailer.

---

## File structure

**New files:**
- `klippy/jerk_math.py` — production implementation of `reachable_v_end` + helpers
- `test/test_jerk_math.py` — unit tests + 180-case verification sweep

**Modified:** none in Phase A2b. Integration into `klippy/toolhead.py` is Phase A2c.

---

## Task 1: Scaffold module + failing tests

**Files:**
- Create: `klippy/jerk_math.py` (stub)
- Create: `test/test_jerk_math.py`

- [ ] **Step 1.1: Write the failing test file**

Create `test/test_jerk_math.py`:

```python
"""Tests for klippy/jerk_math.py — jerk-aware reachable-velocity.

Plan 9 Phase A2b — verified against the pre-computed Python reference at
docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py and the
forward primitive at docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from klippy import jerk_math


def _load_module(filename: str):
    path = (
        Path(__file__).resolve().parents[1]
        / "docs" / "superpowers" / "plans" / "plan9-derivations" / filename
    )
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REF = _load_module("jerk_reachable_ref.py")
JP = _load_module("jerk_profile_ref.py")


# ---- basic sanity --------------------------------------------------------

def test_module_exposes_reachable_v_end():
    assert callable(jerk_math.reachable_v_end)


# ---- classical-limit check -----------------------------------------------

def test_matches_classical_formula_at_high_jerk():
    """As j_max → ∞, reachable_v_end(v0, a, j, L) → sqrt(v0² + 2·L·a)."""
    v0, a, L = 100.0, 5000.0, 50.0
    classical = math.sqrt(v0 * v0 + 2.0 * L * a)
    actual = jerk_math.reachable_v_end(v0, a, 1e12, L)
    assert actual == pytest.approx(classical, rel=1e-4)


# ---- regime-A and regime-B spot checks -----------------------------------

def test_regime_a_triangular_short_move():
    """Short L → triangular regime → v_end < classical-formula prediction."""
    v0, a, j, L = 0.0, 5000.0, 100000.0, 0.5  # L < L_boundary
    classical = math.sqrt(v0 * v0 + 2.0 * L * a)
    v_end = jerk_math.reachable_v_end(v0, a, j, L)
    assert v_end < classical  # jerk-limited must underestimate
    # Round-trip: the forward primitive must give back L.
    _, _, _, dist = JP.accel_side_timings(v0, v_end, a, j)
    assert dist == pytest.approx(L, rel=1e-9, abs=1e-9)


def test_regime_b_trapezoidal_long_move():
    """Long L → trapezoidal regime → v_end close to classical-formula."""
    v0, a, j, L = 0.0, 5000.0, 100000.0, 100.0  # L >> L_boundary
    v_end = jerk_math.reachable_v_end(v0, a, j, L)
    # Round-trip: the forward primitive must give back L.
    _, _, _, dist = JP.accel_side_timings(v0, v_end, a, j)
    assert dist == pytest.approx(L, rel=1e-9, abs=1e-9)


# ---- edge cases ----------------------------------------------------------

def test_zero_distance_returns_v_start():
    assert jerk_math.reachable_v_end(100.0, 5000.0, 100000.0, 0.0) == pytest.approx(100.0)


def test_monotonic_in_L():
    """Doubling L must increase v_end."""
    v0, a, j = 50.0, 3000.0, 80000.0
    v1 = jerk_math.reachable_v_end(v0, a, j, 10.0)
    v2 = jerk_math.reachable_v_end(v0, a, j, 20.0)
    assert v2 > v1


def test_rejects_negative_inputs():
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(-1.0, 5000.0, 100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, -5000.0, 100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, 5000.0, -100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, 5000.0, 100000.0, -10.0)


# ---- 180-case sweep vs pre-verified reference ----------------------------

_SWEEP_V0 = [0.0, 50.0, 200.0, 500.0]
_SWEEP_A  = [2500.0, 5000.0, 10000.0]
_SWEEP_J  = [50000.0, 100000.0, 500000.0]
_SWEEP_L  = [0.1, 1.0, 10.0, 100.0, 1000.0]

_SWEEP_CASES = [
    (v0, a, j, L)
    for v0 in _SWEEP_V0
    for a in _SWEEP_A
    for j in _SWEEP_J
    for L in _SWEEP_L
]


@pytest.mark.parametrize("v0,a,j,L", _SWEEP_CASES,
                         ids=[f"v0={v0},a={a},j={j},L={L}" for v0, a, j, L in _SWEEP_CASES])
def test_sweep_matches_reference(v0, a, j, L):
    ref = REF.reachable_v_end(v0, a, j, L)
    actual = jerk_math.reachable_v_end(v0, a, j, L)
    assert actual == pytest.approx(ref, rel=1e-9, abs=1e-9)
```

- [ ] **Step 1.2: Create stub module**

Create `klippy/jerk_math.py`:

```python
"""Jerk-aware reachable-velocity math for the Kalico motion pipeline.

Plan 9 Phase A2b. Replaces the legacy constant-accel approximation
  delta_v2 = 2 * move_d * max_accel
with the closed-form regime-dispatched solution that accounts for the
time spent ramping acceleration up/down under a jerk limit.

Reference implementation + derivation:
  docs/superpowers/plans/2026-04-24-plan9-phaseA2b-derivation.md
  docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py
"""
from __future__ import annotations

import math


def reachable_v_end(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Stub — see Task 2 for the real implementation."""
    raise NotImplementedError("Task 2 impl")
```

- [ ] **Step 1.3: Run tests — expect 10 ERROR / FAIL, 0 PASS**

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_jerk_math.py -v`
Expected: the basic tests + sweep all raise `NotImplementedError`. `test_module_exposes_reachable_v_end` PASSES (function is defined, even if it raises).

- [ ] **Step 1.4: Commit scaffolding**

```bash
git add klippy/jerk_math.py test/test_jerk_math.py
git commit -m "plan9-A2b: scaffold jerk_math module"
```

---

## Task 2: Implement reachable_v_end (port from reference)

**Files:**
- Modify: `klippy/jerk_math.py` (replace stub)

- [ ] **Step 2.1: Read the reference implementation**

Open `docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py` and study `reachable_v_end`. The file is ~150 lines and contains:
- Input validation
- `L_boundary` computation
- Regime-A solver (depressed-cubic via signed cube roots)
- Regime-B solver (quadratic)
- The dispatch

The reference is the source of truth. Your production implementation should be a faithful port with these adjustments:
- Top-of-file module docstring matches the production file
- Use `math.cbrt` if available (Python 3.11+); fall back to a signed-cube-root helper otherwise
- Raise `ValueError` with clear messages on bad inputs (not `assert`)

- [ ] **Step 2.2: Port the implementation**

Replace the body of `klippy/jerk_math.py`:

```python
"""Jerk-aware reachable-velocity math for the Kalico motion pipeline.

Plan 9 Phase A2b. Replaces the legacy constant-accel approximation
  delta_v2 = 2 * move_d * max_accel
with the closed-form regime-dispatched solution that accounts for the
time spent ramping acceleration up/down under a jerk limit.

Reference implementation + derivation:
  docs/superpowers/plans/2026-04-24-plan9-phaseA2b-derivation.md
  docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py
"""
from __future__ import annotations

import math


def _signed_cbrt(x: float) -> float:
    """Real cube root for any sign of x."""
    if x >= 0.0:
        return x ** (1.0 / 3.0)
    return -((-x) ** (1.0 / 3.0))


def reachable_v_end(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Return the largest v_end reachable from v_start in exactly L units of
    distance under acceleration cap `a_max` and jerk cap `j_max`.

    Inverse of `accel_side_distance` from Phase A1 jerk_profile.c.

    Parameters
    ----------
    v_start : float
        Starting velocity, >= 0.
    a_max : float
        Absolute maximum acceleration, > 0.
    j_max : float
        Absolute maximum jerk, > 0.
    L : float
        Traversed distance, >= 0.

    Returns
    -------
    v_end : float
        End velocity. >= v_start. When L == 0, returns v_start exactly.

    Raises
    ------
    ValueError
        If any input is negative or a_max / j_max is zero.
    """
    # PASTE THE REFERENCE BODY HERE — see
    # docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py
    # Copy reachable_v_end() verbatim (modulo the docstring/name changes above),
    # including the L_boundary dispatch and both regime solvers.
    #
    # If the reference uses `assert` for input guards, replace with `raise
    # ValueError(...)` with clear messages.
    raise NotImplementedError(
        "REMOVE THIS LINE and port the reference reachable_v_end body here")
```

**Do not copy-paste from memory or guess the algebra.** Open the reference file and port. Cross-check every line against the derivation doc §Part 3 and §Part 4. The regime A solver uses `_signed_cbrt` to handle the depressed-cubic discriminant properly.

- [ ] **Step 2.3: Run tests — expect full pass**

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_jerk_math.py -v`
Expected: all pass (9 basic + 180 sweep = 189 total).

If any case fails:
- Check `L_boundary` computation — it's `(2 * v_start + a_max² / j_max) * (a_max / j_max)` per derivation §Part 2.
- Regime A discriminant: `D = L² * j_max / 4 + (2 * v_start / 3)³`. Always >= 0.
- Regime A: `u = cbrt(L * sqrt(j_max) / 2 + sqrt(D)) + cbrt(L * sqrt(j_max) / 2 - sqrt(D))`, then `dv = u²`, `v_end = v_start + dv`. The second cube root argument can be negative — use `_signed_cbrt`.
- Regime B: `dv = (-b + sqrt(b² - 4·c)) / 2` where `b = 2·v_start + a²/j` and `c = 2·v_start·(a²/j) - 2·L·a`.
- Do NOT widen test tolerances. If tests fail, the port is wrong.

- [ ] **Step 2.4: Commit**

```bash
git add klippy/jerk_math.py
git commit -m "plan9-A2b: implement reachable_v_end"
```

---

## Task 3: Completion report

**Files:**
- Create: `docs/superpowers/plans/2026-04-24-plan9-phaseA2b-completion.md`

- [ ] **Step 3.1: Run full test count**

Run: `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/test_jerk_profile.py test/test_linear_as_jerk_profile.py test/test_jerk_math.py -v`
Expected: 56 + 6 + 189 = 251 PASS (confirm the exact number — if jerk_math tests differ from 189 due to pytest parametrize-id collision quirks, use the actual count).

- [ ] **Step 3.2: Write completion doc**

Create `docs/superpowers/plans/2026-04-24-plan9-phaseA2b-completion.md`:

```markdown
# Plan 9 Phase A2b — completion report

**Status:** COMPLETE
**Date:** 2026-04-24
**Commits:**
- `<sha>` — plan9-A2b: scaffold jerk_math module
- `<sha>` — plan9-A2b: implement reachable_v_end

## What shipped

- `klippy/jerk_math.py` — `reachable_v_end(v_start, a_max, j_max, L)` + `_signed_cbrt` helper. Closed-form regime-dispatched solver (Regime A triangular via depressed cubic, Regime B trapezoidal via quadratic).
- `test/test_jerk_math.py` — basic sanity + round-trip + monotonicity + edge-cases + 180-case sweep vs pre-verified Python reference.

## Validation

- 180-case sweep (v0 ∈ {0, 50, 200, 500}, a_max ∈ {2500, 5000, 10000}, j_max ∈ {50000, 100000, 500000}, L ∈ {0.1, 1, 10, 100, 1000}) passes to 1e-9 relative tolerance vs `jerk_reachable_ref.py`.
- Classical-limit verified: j=1e12 gives the old Klipper formula `sqrt(v0² + 2·L·a)` to 1e-4.
- Round-trip: `accel_side_distance(v0, reachable_v_end(v0, a, j, L), a, j) == L` to 1e-9 across sweep.
- Monotonicity: v_end strictly increases in L (spot-checked).
- Bad-input rejection: negative v0/a/j/L all raise ValueError.
- Zero-L returns v_start exactly.

## Next — A2c

Wire `reachable_v_end` and `jerk_profile_compute` into `klippy/toolhead.py`:
- `Move.__init__` computes `delta_v2` via `reachable_v_end` instead of `2*move_d*accel`.
- `Move.calc_junction` uses the jerk-aware delta_v2.
- `Move.set_junction` computes and stores a `jerk_profile_result` in place of `accel_t/cruise_t/decel_t`.
- `LookAheadQueue.flush` reverse pass uses `reachable_v_end` for the reachable-start-v computation.
- Must preserve backwards-compat fields (`accel_t`, `cruise_t`, `decel_t`) for downstream consumers (kinematics/extruder, homing, drip_move) until A2d sweeps them.
```

- [ ] **Step 3.3: Commit**

```bash
git add docs/superpowers/plans/2026-04-24-plan9-phaseA2b-completion.md
git commit -m "plan9-A2b: completion report"
```

---

## References

- Spec: `docs/superpowers/specs/2026-04-24-plan9-greenfield-motion-design.md`
- Derivation: `docs/superpowers/plans/2026-04-24-plan9-phaseA2b-derivation.md`
- Reference impl: `docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py`
- Phase A1 forward primitive: `klippy/chelper/jerk_profile.c::jerk_profile_accel_side_timings`
- Phase A1 reference: `docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py`
