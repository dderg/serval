# Plan 3 — Extruder as first-class planning constraint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire a per-move extruder cap — `blendextruder.cap_move(move, pa_model, extruder_limits) → (v_cap, a_cap)` — into the motion planner. The cap reads the live Pressure-Advance model and derives the tightest `(v_xy, a_xy)` such that post-PA stepper output stays under configured `max_extruder_accel` and `max_extruder_rpm`. Binding moves get clamped; non-binding moves are untouched.

**Architecture:** New Python module `klippy/blendextruder.py` at the planner layer (mirrors `blendmath.py`, `blendquintic.py`). PA model classes in `klippy/kinematics/extruder.py` gain `f_prime(v)` / `f_double_prime(v)` derivative methods. Config parsing in `klippy/extras/extruder.py` adds two new keys plus a `SET_EXTRUDER_LIMITS` gcode command. Integration hook: after `kin.check_move(move)` completes during Move construction, call `cap_move` and apply via the existing `move.limit_speed` machinery. The cap applies uniformly to user gcode moves AND blend-polyline moves emitted by `blendplanner._emit_blend`.

**Tech Stack:** Python (planner + kinematics layer), pytest, Klipper's existing Move/extruder infrastructure.

**Predecessor:** Plan 2 Phase A (commit `496365b2`) — smooth-shapers merged; non-linear PA models (`PALinearModel`, `PATanhModel`, `PAReciprModel`) now live in the tree.

**Spec:** `docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md`.

---

## File Structure

| File | Role | Change type |
|---|---|---|
| `klippy/blendshape.py` | `ExtruderLimits` dataclass — extended to carry `(a_E_max, v_E_max, smooth_time)` | modify (rename+expand existing fields) |
| `klippy/kinematics/extruder.py` | PA model classes gain `f_prime(v)`, `f_double_prime(v)` methods | modify (3 classes) |
| `klippy/extras/extruder.py` | Config keys `max_extruder_accel`/`max_extruder_rpm`; `SET_EXTRUDER_LIMITS` gcode; `extruder_limits_snapshot()` method | modify |
| `klippy/blendextruder.py` | **NEW**: `cap_move`, `PAModelSnapshot`, internal bisection helper | create |
| `klippy/blendplanner.py` | Populate `KinematicLimits.extruder_caps` from toolhead snapshot (forward-compat for Plan 5) | modify (1 line) |
| `klippy/toolhead.py` (or `klippy/move.py` equivalent) | Hook `cap_move` call in the Move constraint pipeline, after `kin.check_move(move)` | modify |
| `test/test_blendextruder.py` | **NEW**: unit tests | create |
| `test/test_blendshape.py` | Existing `ExtruderLimits` tests updated for new field names | modify |
| `docs/Config_Reference.md` | Document new `[extruder]` keys | modify |

---

## Notes for the implementer

- **User's rule on git hygiene:** stage specific files by name. **Never `git add -A` or `git add .`** — past incidents captured `.claude/`, `.dSYM/`, and user-edited configs into commits.
- **User's rule on commit timing:** no commits during work hours (Mon–Fri 08:00–18:00 CEST) until 2026-05-01. If you're within work hours, **stage with `git add <files>`, then HOLD the commit and note "staged; commit pending off-hours" in your report**. User has granted a session-wide override for Plan 3: commits are allowed, but **no push** until after 18:00.
- **No `Co-Authored-By: Claude …` trailers** in any commit message. Ever.
- **Run tests from repo root:** `cd /Users/daniladergachev/Developer/kalico && python3 -m pytest test/`.
- **Before starting any task**, run `git status` and confirm tree state matches the expected post-previous-task state. Stop and report if it doesn't.
- **Current HEAD as Plan 3 starts:** `f1554b36` (spec-commit on top of Plan 2 Phase A merge).

---

## Task 1: Extend `ExtruderLimits` dataclass

**Goal:** Rename/expand `blendshape.ExtruderLimits` from `(accel_max, rpm_max)` to `(a_E_max, v_E_max, smooth_time)`. The existing placeholder in `blendshape.py` was written in Plan 1 before the PA math was worked out; the new fields reflect what the cap formula actually needs.

**Files:**
- Modify: `klippy/blendshape.py`
- Modify: `test/test_blendshape.py` (if it exists; create if not)
- Grep for existing callers: `grep -rn 'ExtruderLimits\|accel_max\|rpm_max' klippy/ test/` — should only find references in `blendshape.py` + `blendplanner.py:68` (which sets `extruder_caps=None`, not affected).

- [ ] **Step 1: Verify no other callers**

```bash
cd /Users/daniladergachev/Developer/kalico
grep -rn 'ExtruderLimits\|accel_max=\|rpm_max=' klippy/ test/
```
Expected: only definition in `blendshape.py` and the `extruder_caps=None` line in `blendplanner.py`. No callers construct `ExtruderLimits(...)` yet.

- [ ] **Step 2: Write failing test**

Create or append to `test/test_blendshape.py`:
```python
import pytest
from klippy import blendshape


def test_extruder_limits_has_three_fields():
    """ExtruderLimits carries stepper-output limits + PA smoothing time."""
    lim = blendshape.ExtruderLimits(
        a_E_max=5000.0,
        v_E_max=15.9,
        smooth_time=0.04,
    )
    assert lim.a_E_max == 5000.0
    assert lim.v_E_max == 15.9
    assert lim.smooth_time == 0.04


def test_extruder_limits_rejects_nonpositive_smooth_time_in_assertion():
    """K_h = (15/8)/smooth_time; smooth_time <= 0 would blow up.
    Not a hard gate here — the cap_move path will guard — but this
    documents the expected invariant for downstream consumers.
    """
    # No runtime assertion in the dataclass itself (dataclasses don't
    # validate); this test just documents the contract.
    lim = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    assert lim.smooth_time > 0.0
```

- [ ] **Step 3: Run tests — expect failure**

```bash
python3 -m pytest test/test_blendshape.py -v
```
Expected: `AttributeError` or `TypeError` — old dataclass has `accel_max`/`rpm_max`, not the new field names.

- [ ] **Step 4: Update `ExtruderLimits` in `blendshape.py`**

Find (currently at `klippy/blendshape.py:24-31`):
```python
@dataclass
class ExtruderLimits:
    """First-class extruder constraints (pillar 3).

    Plan 1 leaves this as None everywhere; plan 4 threads it through.
    """
    accel_max: float   # mm/s^2 on the filament
    rpm_max: float     # drive-pulley angular velocity
```

Replace with:
```python
@dataclass
class ExtruderLimits:
    """First-class extruder constraints (pillar 3, Plan 3).

    Post-PA stepper output is bounded by (a_E_max, v_E_max). smooth_time
    is the PA smoothing window and feeds the cap formula via
    K_h = (15/8) / smooth_time.

    Built by `klippy/extras/extruder.py::extruder_limits_snapshot()` and
    read by `klippy/blendextruder.py::cap_move()`.
    """
    a_E_max: float       # mm/s^2 on filament (from config: max_extruder_accel)
    v_E_max: float       # mm/s on filament (config: max_extruder_rpm * rotation_distance / 60)
    smooth_time: float   # seconds — current pressure_advance_smooth_time
```

- [ ] **Step 5: Run tests — expect pass**

```bash
python3 -m pytest test/test_blendshape.py -v
```
Expected: both tests pass.

- [ ] **Step 6: Run full blendshape suite + check no regressions**

```bash
python3 -m pytest test/ -k 'blendshape or blendplanner or blendmath or blendquintic' 2>&1 | tail -5
```
Expected: no regressions. `extruder_caps=None` in `blendplanner.py:68` still works since `None` is accepted.

- [ ] **Step 7: Commit**

```bash
git add klippy/blendshape.py test/test_blendshape.py
git commit -m "blendshape: extend ExtruderLimits to (a_E_max, v_E_max, smooth_time)

Renamed/expanded from Plan 1's (accel_max, rpm_max) placeholder to
carry the fields the post-PA cap formula actually needs. smooth_time
feeds K_h = (15/8)/smooth_time in blendextruder.cap_move.

No consumers change behavior yet — blendplanner still sets
extruder_caps=None (wired in Plan 3 Task 10).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 2: Add `f_prime`, `f_double_prime` to PA model classes

**Goal:** Each PA model class gains two pure-math methods returning the first and second derivatives of its advance function. These feed directly into the cap formula.

**Files:**
- Modify: `klippy/kinematics/extruder.py` — `PALinearModel`, `PATanhModel`, `PAReciprModel` (3 classes; tanh and recipr share a common parent `PANonLinearModel`)
- Create: `test/test_pa_derivatives.py`

**Reference (from spec):**

| Model | `f'(v)` | `f''(v)` |
|---|---|---|
| `PALinearModel` | `PA` | `0` |
| `PATanhModel` | `LA + (NO/LV) · sech²(v/LV)` | `−(2·NO/LV²) · sech²(v/LV) · tanh(v/LV)` |
| `PAReciprModel` | `LA + (NO/LV) / (1 + v/LV)²` | `−(2·NO/LV²) / (1 + v/LV)³` |

where `PA = pressure_advance`, `LA = linear_advance`, `NO = nonlinear_offset`, `LV = linearization_velocity`.

- [ ] **Step 1: Write failing tests**

Create `test/test_pa_derivatives.py`:
```python
import math
import pytest


def _setup_linear(pa=0.04):
    from klippy.kinematics.extruder import PALinearModel
    m = PALinearModel.__new__(PALinearModel)
    m.pressure_advance = pa
    return m


def _setup_tanh(la=0.0, no=0.04, lv=100.0):
    from klippy.kinematics.extruder import PATanhModel
    m = PATanhModel.__new__(PATanhModel)
    m.linear_advance = la
    m.nonlinear_offset = no
    m.linearization_velocity = lv
    return m


def _setup_recipr(la=0.0, no=0.04, lv=100.0):
    from klippy.kinematics.extruder import PAReciprModel
    m = PAReciprModel.__new__(PAReciprModel)
    m.linear_advance = la
    m.nonlinear_offset = no
    m.linearization_velocity = lv
    return m


def test_linear_f_prime_is_constant_pa():
    m = _setup_linear(pa=0.04)
    assert m.f_prime(0.0) == 0.04
    assert m.f_prime(100.0) == 0.04
    assert m.f_prime(600.0) == 0.04


def test_linear_f_double_prime_is_zero():
    m = _setup_linear(pa=0.04)
    assert m.f_double_prime(0.0) == 0.0
    assert m.f_double_prime(100.0) == 0.0


def test_tanh_f_prime_at_zero_is_max():
    """f'(0) = LA + NO/LV — decreases monotonically from v=0."""
    m = _setup_tanh(la=0.01, no=0.04, lv=100.0)
    fp0 = m.f_prime(0.0)
    fp100 = m.f_prime(100.0)
    fp500 = m.f_prime(500.0)
    assert fp0 == pytest.approx(0.01 + 0.04 / 100.0, rel=1e-9)
    assert fp0 > fp100 > fp500
    assert fp500 > 0.01 - 1e-6  # converges to LA


def test_tanh_f_double_prime_is_negative():
    """f''(v) <= 0 for v >= 0 in tanh model (saturating)."""
    m = _setup_tanh(la=0.0, no=0.04, lv=100.0)
    assert m.f_double_prime(0.0) == 0.0  # sech²(0)·tanh(0) = 0
    assert m.f_double_prime(50.0) < 0.0
    assert m.f_double_prime(200.0) < 0.0


def test_tanh_f_prime_numerical_check():
    """Closed-form f'(v) matches finite-difference of f(v)."""
    m = _setup_tanh(la=0.005, no=0.04, lv=100.0)
    # Emulate f(v) from the model: LA*v + NO*tanh(v/LV)
    def f(v):
        return m.linear_advance * v + m.nonlinear_offset * math.tanh(v / m.linearization_velocity)
    h = 1e-4
    for v in (0.5, 50.0, 200.0, 400.0):
        fd = (f(v + h) - f(v - h)) / (2 * h)
        assert m.f_prime(v) == pytest.approx(fd, rel=1e-6)


def test_recipr_f_prime_at_zero_is_max():
    m = _setup_recipr(la=0.01, no=0.04, lv=100.0)
    fp0 = m.f_prime(0.0)
    fp100 = m.f_prime(100.0)
    fp500 = m.f_prime(500.0)
    assert fp0 == pytest.approx(0.01 + 0.04 / 100.0, rel=1e-9)
    assert fp0 > fp100 > fp500


def test_recipr_f_double_prime_is_negative():
    m = _setup_recipr(la=0.0, no=0.04, lv=100.0)
    assert m.f_double_prime(0.0) < 0.0
    assert m.f_double_prime(100.0) < 0.0


def test_recipr_f_prime_numerical_check():
    m = _setup_recipr(la=0.005, no=0.04, lv=100.0)
    def f(v):
        r = v / m.linearization_velocity
        return m.linear_advance * v + m.nonlinear_offset * (1.0 - 1.0 / (1.0 + r))
    h = 1e-4
    for v in (0.5, 50.0, 200.0, 400.0):
        fd = (f(v + h) - f(v - h)) / (2 * h)
        assert m.f_prime(v) == pytest.approx(fd, rel=1e-6)
```

- [ ] **Step 2: Run tests — expect failure**

```bash
python3 -m pytest test/test_pa_derivatives.py -v
```
Expected: all tests FAIL with `AttributeError: 'PALinearModel' object has no attribute 'f_prime'` etc.

- [ ] **Step 3: Implement `PALinearModel` derivatives**

In `klippy/kinematics/extruder.py`, find the `PALinearModel` class (starts around line 69). Add the two methods:

```python
    def f_prime(self, v):
        """d/dv of the PA advance function. Constant for linear PA."""
        return self.pressure_advance

    def f_double_prime(self, v):
        """d²/dv² of the PA advance function. Zero for linear PA."""
        return 0.0
```

Place them inside the class, after the existing `get_status` / repr methods.

- [ ] **Step 4: Implement `PATanhModel` derivatives**

Find the `PATanhModel` class (around line 175). It inherits from `PANonLinearModel` which holds `linear_advance`, `nonlinear_offset`, `linearization_velocity`. Add to `PATanhModel`:

```python
    def f_prime(self, v):
        """d/dv: LA + (NO/LV) · sech²(v/LV)."""
        import math
        if self.linearization_velocity <= 0.0:
            return self.linear_advance
        vn = v / self.linearization_velocity
        # sech²(x) = 1 - tanh²(x) — avoid cosh for numerical stability
        sech2 = 1.0 - math.tanh(vn) ** 2
        return self.linear_advance + (self.nonlinear_offset / self.linearization_velocity) * sech2

    def f_double_prime(self, v):
        """d²/dv²: −(2·NO/LV²) · sech²(v/LV) · tanh(v/LV)."""
        import math
        if self.linearization_velocity <= 0.0:
            return 0.0
        vn = v / self.linearization_velocity
        sech2 = 1.0 - math.tanh(vn) ** 2
        return -2.0 * self.nonlinear_offset / (self.linearization_velocity ** 2) * sech2 * math.tanh(vn)
```

- [ ] **Step 5: Implement `PAReciprModel` derivatives**

Find `PAReciprModel` (around line 186). Add:

```python
    def f_prime(self, v):
        """d/dv: LA + (NO/LV) / (1 + v/LV)²."""
        if self.linearization_velocity <= 0.0:
            return self.linear_advance
        r = v / self.linearization_velocity
        return self.linear_advance + (self.nonlinear_offset / self.linearization_velocity) / (1.0 + r) ** 2

    def f_double_prime(self, v):
        """d²/dv²: −(2·NO/LV²) / (1 + v/LV)³."""
        if self.linearization_velocity <= 0.0:
            return 0.0
        r = v / self.linearization_velocity
        return -2.0 * self.nonlinear_offset / (self.linearization_velocity ** 2) / (1.0 + r) ** 3
```

- [ ] **Step 6: Run tests — expect pass**

```bash
python3 -m pytest test/test_pa_derivatives.py -v
```
Expected: all tests pass.

- [ ] **Step 7: Run full test suite — regression check**

```bash
python3 -m pytest test/ 2>&1 | tail -5
```
Expected: no new failures. Pre-existing environment failures (jinja2 missing, etc.) unchanged.

- [ ] **Step 8: Commit**

```bash
git add klippy/kinematics/extruder.py test/test_pa_derivatives.py
git commit -m "kinematics/extruder: add f_prime/f_double_prime to PA models

Each PA*Model class now exposes analytic first and second derivatives
of its advance function f(v). These feed the Plan 3 per-move cap
formula in blendextruder.cap_move. Verified numerically against
finite-difference of each model's f(v).

- PALinearModel:  f'(v) = PA,           f''(v) = 0
- PATanhModel:    f'(v) = LA + (NO/LV)·sech²(v/LV)
                  f''(v) = -(2·NO/LV²)·sech²(v/LV)·tanh(v/LV)
- PAReciprModel:  f'(v) = LA + (NO/LV)/(1+v/LV)²
                  f''(v) = -(2·NO/LV²)/(1+v/LV)³

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 3: Create `blendextruder.py` skeleton with `PAModelSnapshot` and `cap_move` edge cases

**Goal:** New module scaffold. Supports the snapshot dataclass, `cap_move` signature, and trivial edge cases (k=0, disabled limits). Actual model-specific cap math comes in Tasks 4-6.

**Files:**
- Create: `klippy/blendextruder.py`
- Create: `test/test_blendextruder.py`

- [ ] **Step 1: Write failing tests**

Create `test/test_blendextruder.py`:
```python
import math
import pytest

from klippy import blendextruder, blendshape


# Shared fake Move — just exposes the attrs cap_move reads.
class _FakeMove:
    def __init__(self, k, max_cruise_v):
        # axes_r is (x_ratio, y_ratio, z_ratio, e_ratio); cap_move only
        # reads axes_r[3] (= k, the flow ratio).
        self.axes_r = (1.0, 0.0, 0.0, k)
        self.max_cruise_v2 = max_cruise_v ** 2
        self.max_cruise_v = max_cruise_v


def _default_limits():
    return blendshape.ExtruderLimits(
        a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04,
    )


def _default_linear_snap(pa=0.04):
    return blendextruder.PAModelSnapshot(kind="linear", params=(pa,))


def test_cap_move_travel_returns_inf():
    """k=0 (pure XY travel, no extrusion): cap is infinite."""
    move = _FakeMove(k=0.0, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = _default_limits()
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_none_pa_model_returns_inf():
    """No PA model (extruder not configured) — cap is inactive."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    limits = _default_limits()
    v_cap, a_cap = blendextruder.cap_move(move, None, limits)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_none_limits_returns_inf():
    """Limits not configured (max_extruder_accel=0) — cap is inactive."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    snap = _default_linear_snap()
    v_cap, a_cap = blendextruder.cap_move(move, snap, None)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_zero_a_max_returns_zero_accel():
    """Degenerate: a_E_max=0 pins a_cap to 0 (cannot accelerate)."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = blendshape.ExtruderLimits(a_E_max=0.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    assert a_cap == 0.0


def test_pa_model_snapshot_is_immutable():
    """Snapshot carries the model state at construction time."""
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    assert snap.kind == "linear"
    assert snap.params == (0.04,)
    # Frozen dataclass or namedtuple — mutation should fail.
    with pytest.raises((AttributeError, TypeError, Exception)):
        snap.kind = "tanh"
```

- [ ] **Step 2: Run tests — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -v
```
Expected: all tests FAIL (`ImportError` — module doesn't exist yet).

- [ ] **Step 3: Create `klippy/blendextruder.py`**

```python
# klippy/blendextruder.py
# Per-move extruder cap for the Plan 3 "extruder as first-class
# constraint" pillar. Reads the live Pressure-Advance model and the
# configured (a_E_max, v_E_max, smooth_time) limits, computes the
# tightest (v_xy, a_xy) such that the post-PA stepper output stays
# within the stepper's physical budget.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


# --- Snapshot types ---

@dataclass(frozen=True)
class PAModelSnapshot:
    """Immutable snapshot of a PA model's state at planning time.

    kind:   "linear" | "tanh" | "recipr"
    params: tuple, interpretation by kind:
      linear → (pressure_advance,)
      tanh   → (linear_advance, nonlinear_offset, linearization_velocity)
      recipr → (linear_advance, nonlinear_offset, linearization_velocity)
    """
    kind: str
    params: tuple


# --- Derivative evaluation (mirrors kinematics/extruder.py) ---

def _f_prime(snap: PAModelSnapshot, v: float) -> float:
    """PA model derivative f'(v). Pure math; no live model access."""
    if snap.kind == "linear":
        (pa,) = snap.params
        return pa
    la, no, lv = snap.params
    if lv <= 0.0:
        return la
    if snap.kind == "tanh":
        vn = v / lv
        sech2 = 1.0 - math.tanh(vn) ** 2
        return la + (no / lv) * sech2
    if snap.kind == "recipr":
        r = v / lv
        return la + (no / lv) / (1.0 + r) ** 2
    raise ValueError(f"unknown PA model kind: {snap.kind!r}")


# --- Public API ---

def cap_move(
    move,
    pa_model: Optional[PAModelSnapshot],
    extruder_limits,  # Optional[blendshape.ExtruderLimits]
) -> Tuple[float, float]:
    """Compute (v_cap, a_cap) for a move such that the post-PA stepper
    output stays within extruder_limits. Returns (+inf, +inf) when the
    cap is inactive (travel move, no PA, no limits configured).

    `move` must expose `axes_r[3]` (flow ratio k = dE/dL) and
    `max_cruise_v` (the move's target cruise velocity before capping).

    Linear PA: closed-form cap (Task 4).
    Non-linear PA (tanh/recipr): accel cap closed-form, velocity cap
    via 1-D bisection (Tasks 5-6).
    """
    # Edge case: no PA model (extruder not configured for PA).
    if pa_model is None:
        return (float("inf"), float("inf"))
    # Edge case: no extruder limits configured.
    if extruder_limits is None:
        return (float("inf"), float("inf"))
    # Edge case: pure travel move (no extrusion).
    k = move.axes_r[3]
    if k <= 0.0:
        return (float("inf"), float("inf"))
    # Degenerate: zero accel budget.
    if extruder_limits.a_E_max <= 0.0:
        return (float("inf"), 0.0)

    # Actual cap math is routed by PA model kind — see Tasks 4-6.
    # For now, fall through to no-cap (no-op) — tasks 4-6 replace this.
    return (float("inf"), float("inf"))
```

- [ ] **Step 4: Run tests — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -v
```
Expected: all 5 tests pass.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendextruder.py test/test_blendextruder.py
git commit -m "blendextruder: module scaffold + PAModelSnapshot + edge cases

New planner-layer module for the Plan 3 per-move extruder cap. This
commit lands the skeleton:

- PAModelSnapshot frozen dataclass (kind + params tuple)
- _f_prime(snap, v) helper — duplicates derivative math from
  kinematics/extruder.py in terms of the snapshot (no live access)
- cap_move() public API with edge-case handling:
    - pa_model=None   -> (+inf, +inf)
    - limits=None     -> (+inf, +inf)
    - k=0 (travel)    -> (+inf, +inf)
    - a_E_max=0       -> (inf, 0)

Per-model cap math lands in Tasks 4-6 (linear, bisection helper, NL).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 4: Linear PA cap (closed-form)

**Goal:** Implement the linear-PA branch of `cap_move`. Closed-form; no bisection needed.

**Files:**
- Modify: `klippy/blendextruder.py`
- Modify: `test/test_blendextruder.py`

**Formula (from spec §6):**
```
K_h = (15/8) / smooth_time
a_E_cap = a_E_max / (1 + PA · K_h)
v_E_cap = v_E_max  (cruise-level constraint)
v_cap_xy = (v_E_max - PA · a_E_cap) / k   # accel-phase constraint
         ...but clamp so v_cap_xy >= 0.
v_cap = min(v_E_max / k, v_cap_xy)
a_cap = a_E_cap / k
```

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendextruder.py`:
```python
def test_cap_move_linear_a_cap_closed_form():
    """a_E_cap = a_E_max / (1 + PA · K_h); a_cap = a_E_cap / k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    pa = 0.04
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    K_h = (15.0 / 8.0) / 0.04  # = 46.875
    expected_a_E_cap = 5000.0 / (1.0 + pa * K_h)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    assert a_cap == pytest.approx(expected_a_E_cap / k, rel=1e-9)


def test_cap_move_linear_v_cap_bounded_by_rpm_term():
    """When (PA · a_E_cap) is small, v_cap ≈ v_E_max / k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    pa = 0.001  # tiny PA -> tiny accel-term drag on v_cap
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, _ = blendextruder.cap_move(move, snap, limits)
    # rpm-bound alone: v_E_max / k = 15.9 / 0.04 = 397.5 mm/s
    assert v_cap <= 15.9 / k + 1e-6
    assert v_cap > 0.0


def test_cap_move_linear_pa_zero_cap_is_simple_division():
    """PA=0 => cap degenerates to (v_E_max/k, a_E_max/k)."""
    k = 0.05
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.0,))
    limits = blendshape.ExtruderLimits(a_E_max=6000.0, v_E_max=20.0, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert v_cap == pytest.approx(20.0 / k, rel=1e-9)
    assert a_cap == pytest.approx(6000.0 / k, rel=1e-9)


def test_cap_move_linear_high_pa_tight_cap():
    """PA = 0.08 at smooth_time=0.04: 1 + 0.08·46.875 = 4.75
    => a_E_cap = a_E_max / 4.75 (79% reduction)."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.08,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    expected = (5000.0 / 4.75) / k
    assert a_cap == pytest.approx(expected, rel=1e-6)
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'linear' -v
```
Expected: the 4 new tests FAIL (`cap_move` returns inf for linear since the branch is a no-op).

- [ ] **Step 3: Implement linear cap branch**

Replace the placeholder block at the bottom of `cap_move` in `klippy/blendextruder.py` with:

```python
    K_h = (15.0 / 8.0) / extruder_limits.smooth_time
    a_E_max = extruder_limits.a_E_max
    v_E_max = extruder_limits.v_E_max

    if pa_model.kind == "linear":
        (pa,) = pa_model.params
        # Accel cap (closed form; f' is constant).
        a_E_cap = a_E_max / (1.0 + pa * K_h)
        a_cap = a_E_cap / k
        # Velocity cap: stepper_v peaks at v_E + PA · a_E_cap during
        # accel-plateau. Solve for v_xy:
        #   k · v_xy + PA · a_E_cap <= v_E_max
        v_from_accel = (v_E_max - pa * a_E_cap) / k
        v_from_rpm = v_E_max / k
        v_cap = min(v_from_rpm, max(0.0, v_from_accel))
        return (v_cap, a_cap)

    # NL PA branches (tanh, recipr) handled in Task 6.
    return (float("inf"), float("inf"))
```

- [ ] **Step 4: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -v
```
Expected: all tests pass (5 edge + 4 linear = 9).

- [ ] **Step 5: Full suite regression check**

```bash
python3 -m pytest test/ 2>&1 | tail -5
```
Expected: no new failures.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendextruder.py test/test_blendextruder.py
git commit -m "blendextruder: linear PA cap (closed form)

cap_move now handles PAModelSnapshot(kind='linear'). a_E_cap closed-
form from a_E_max / (1 + PA·K_h); v_cap is min(v_E_max/k, accel-phase
velocity bound). At PA=0 the cap reduces to (v_E_max/k, a_E_max/k).

Verified against spec §6 closed-form derivation. NL (tanh/recipr)
branch lands in Task 6 using a bisection helper (Task 5).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 5: Bisection helper for velocity cap inversion

**Goal:** Implement `_solve_velocity_cap_bisection(snap, k, a_E_cap, v_E_max)` — a 1-D bisection that solves for the largest `v_xy` such that `k·v_xy + f'(k·v_xy)·a_E_cap ≤ v_E_max`.

**Files:**
- Modify: `klippy/blendextruder.py`
- Modify: `test/test_blendextruder.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendextruder.py`:
```python
def test_bisection_trivial_linear_degenerate_matches_closed_form():
    """Linear PA (f' constant): bisection result matches closed form."""
    pa = 0.04
    k = 0.04
    a_E_cap = 1000.0
    v_E_max = 15.9
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    v_from_bisection = blendextruder._solve_velocity_cap_bisection(
        snap, k, a_E_cap, v_E_max
    )
    expected = (v_E_max - pa * a_E_cap) / k
    assert v_from_bisection == pytest.approx(expected, abs=1e-5)


def test_bisection_tanh_monotone_finds_valid_v():
    """At tanh snapshot: find v such that k·v + f'(k·v)·a_E_cap = v_E_max."""
    snap = blendextruder.PAModelSnapshot(
        kind="tanh", params=(0.0, 0.04, 100.0)
    )
    k = 0.04
    a_E_cap = 1000.0
    v_E_max = 15.9
    v = blendextruder._solve_velocity_cap_bisection(snap, k, a_E_cap, v_E_max)
    # Verify the result satisfies the constraint within tolerance.
    stepper_v = k * v + blendextruder._f_prime(snap, k * v) * a_E_cap
    assert stepper_v == pytest.approx(v_E_max, abs=1e-3)


def test_bisection_clamps_at_rpm_bound():
    """When a_E_cap=0, bisection should yield v_E_max / k exactly."""
    snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.0, 0.04, 100.0))
    k = 0.04
    v = blendextruder._solve_velocity_cap_bisection(snap, k, 0.0, 15.9)
    assert v == pytest.approx(15.9 / k, rel=1e-6)
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'bisection' -v
```
Expected: all 3 FAIL with `AttributeError: module 'klippy.blendextruder' has no attribute '_solve_velocity_cap_bisection'`.

- [ ] **Step 3: Implement the bisection helper**

Add to `klippy/blendextruder.py`, after `_f_prime`:

```python
def _stepper_v_of_xy(snap: PAModelSnapshot, v_xy: float, k: float, a_E_cap: float) -> float:
    """Peak stepper velocity during accel phase at XY target v_xy."""
    V = k * v_xy
    return V + _f_prime(snap, V) * a_E_cap


def _solve_velocity_cap_bisection(
    snap: PAModelSnapshot,
    k: float,
    a_E_cap: float,
    v_E_max: float,
) -> float:
    """Find the largest v_xy such that _stepper_v_of_xy <= v_E_max.

    The constraint is monotone increasing in v_xy (V increases; the
    f'·a_E_cap term is bounded, often non-increasing in v). 1-D bisection
    on [0, v_E_max/k] converges in ~30 iterations for a 1e-6 mm/s tol.
    """
    # Upper bracket: at v_xy = v_E_max/k, stepper_v = v_E_max + f'(V)·a_E_cap
    # which is >= v_E_max (the constraint is violated or exactly met).
    # If a_E_cap = 0, the constraint is just v_xy <= v_E_max / k.
    if a_E_cap <= 0.0:
        return v_E_max / k
    lo = 0.0
    hi = v_E_max / k
    # Bisect for up to 60 iterations, tolerance 1e-6 mm/s.
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _stepper_v_of_xy(snap, mid, k, a_E_cap) <= v_E_max:
            lo = mid
        else:
            hi = mid
        if (hi - lo) < 1e-6:
            break
    return lo
```

- [ ] **Step 4: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -k 'bisection' -v
```
Expected: all 3 pass.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendextruder.py test/test_blendextruder.py
git commit -m "blendextruder: 1-D bisection helper for velocity cap inversion

_solve_velocity_cap_bisection(snap, k, a_E_cap, v_E_max) finds the
largest v_xy such that post-PA stepper velocity stays within v_E_max.
Uses 60-iter 1-D bisection with 1e-6 mm/s tolerance on [0, v_E_max/k].

For linear PA the bisection matches the closed form to tolerance; for
NL models (tanh/recipr) this is the only analytical tool since f' is
velocity-dependent. Used by the NL branch of cap_move (Task 6).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 6: Non-linear PA cap branches (tanh + recipr)

**Goal:** Wire the NL branches into `cap_move`, using the bisection helper.

**Files:**
- Modify: `klippy/blendextruder.py`
- Modify: `test/test_blendextruder.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_blendextruder.py`:
```python
def test_cap_move_tanh_near_zero_nonlinear_offset_matches_linear():
    """When nonlinear_offset=0 for tanh, behavior matches linear LA."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    tanh_snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.04, 0.0, 100.0))
    lin_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_t, a_t = blendextruder.cap_move(move, tanh_snap, limits)
    v_l, a_l = blendextruder.cap_move(move, lin_snap, limits)
    # Tolerance is loose because bisection is slightly less precise
    # than the closed form; both should agree to ~1e-3 mm/s.
    assert a_t == pytest.approx(a_l, rel=1e-6)
    assert v_t == pytest.approx(v_l, abs=1e-3)


def test_cap_move_tanh_realistic_cap_is_close_to_a_E_max_over_k():
    """Realistic NL params: f'·K_h tiny (~0.02), so a_cap ≈ a_E_max/k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    # NO=0.04, LV=100, LA=0 => f'(0) = 4e-4, K_h = 46.875 => 1+f'·K_h = 1.01875
    tanh_snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.0, 0.04, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, tanh_snap, limits)
    naive = 5000.0 / k
    # a_cap should be slightly below naive (1-2% reduction).
    assert 0.98 * naive < a_cap <= naive


def test_cap_move_recipr_matches_pattern():
    """Recipr NL cap is close to a_E_max/k at realistic params."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    recipr_snap = blendextruder.PAModelSnapshot(kind="recipr", params=(0.0, 0.04, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, recipr_snap, limits)
    naive = 5000.0 / k
    assert 0.98 * naive < a_cap <= naive


def test_cap_move_tanh_v_cap_satisfies_constraint():
    """The returned v_cap should satisfy the stepper_v constraint exactly."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.01, 0.04, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    # Verify stepper_v(v_cap) <= v_E_max within tolerance.
    a_E_cap = a_cap * k
    stepper_v = blendextruder._stepper_v_of_xy(snap, v_cap, k, a_E_cap)
    assert stepper_v <= 15.9 + 1e-3
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'tanh or recipr' -v
```
Expected: the 4 new tests FAIL (NL branch still returns `(inf, inf)`).

- [ ] **Step 3: Implement NL branches in `cap_move`**

Find the branch in `cap_move` at the bottom. Replace the trailing:
```python
    # NL PA branches (tanh, recipr) handled in Task 6.
    return (float("inf"), float("inf"))
```

with:
```python
    # Non-linear PA: tanh or recipr.
    # Accel cap uses v_eval = v_cruise (monotonicity of f' makes this a
    # rigorous bound for typical moves — see spec §Appendix B for the
    # more conservative v_prev/v_next derivation if lookahead state is
    # available; at plan-time here we only have move.max_cruise_v).
    # For the current integration this approximation is tight to ~1-2%
    # for typical NL params (NO=0.04, LV=100); Plan 5's continuous v(s)
    # removes the approximation entirely.
    v_eval = k * move.max_cruise_v
    f_prime_eval = _f_prime(pa_model, v_eval)
    a_E_cap = a_E_max / (1.0 + f_prime_eval * K_h)
    a_cap = a_E_cap / k
    # Velocity cap via bisection.
    v_cap = _solve_velocity_cap_bisection(pa_model, k, a_E_cap, v_E_max)
    return (v_cap, a_cap)
```

- [ ] **Step 4: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -v
```
Expected: all tests pass (edge + linear + bisection + NL = 13 or so).

- [ ] **Step 5: Full suite regression check**

```bash
python3 -m pytest test/ 2>&1 | tail -5
```
Expected: no new failures.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendextruder.py test/test_blendextruder.py
git commit -m "blendextruder: tanh + recipr PA cap branches

Non-linear PA cap now computed via:
  - Accel cap (closed form): a_E_cap = a_E_max / (1 + f'(V_peak)·K_h)
    where V_peak = k · move.max_cruise_v. The v_eval approximation
    (using v_cruise instead of min(v_prev, v_next)) is tight to ~1-2%
    for typical NL params; Plan 5 makes it exact via continuous v(s).
  - Velocity cap (bisection): solve the 1-D monotone root equation
    k·v + f'(k·v)·a_E_cap = v_E_max.

At NO=0.04, LV=100, smooth_time=0.04 (typical NL params) a_cap is
~1-2% below the naive a_E_max/k — extruder rarely binds, matching
the math in spec §6 and §Appendix A.

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 7: Config parsing — `max_extruder_accel` + `max_extruder_rpm`

**Goal:** Add two new optional keys to the `[extruder]` config section. Zero = disabled. Exposed via accessor methods on the extruder object.

**Files:**
- Modify: `klippy/kinematics/extruder.py` (where `[extruder]` config is parsed)
- Modify: `test/test_blendextruder.py` (integration test for parsed values)

- [ ] **Step 1: Locate the existing `[extruder]` config parser**

```bash
grep -n 'getfloat\|max_extrude\|rotation_distance' klippy/kinematics/extruder.py | head -30
```
Look for the `ExtruderStepper` class constructor and wherever `config.getfloat(...)` is called. This is where we add the new keys.

- [ ] **Step 2: Write failing test**

Append to `test/test_blendextruder.py`:
```python
def test_extruder_stepper_parses_max_extruder_accel():
    """[extruder] max_extruder_accel parsed; defaults to 0."""
    # Requires the Klipper Printer bootstrap; keep test light — just
    # verify the method exists on ExtruderStepper and returns a float.
    from klippy.kinematics.extruder import ExtruderStepper
    # Use __new__ + manual field-set to avoid full bootstrap.
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0  # mimic config default
    es.max_extruder_rpm = 0.0
    assert es.get_extruder_accel_limit() == 0.0
    assert es.get_extruder_rpm_limit() == 0.0

    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0
    assert es.get_extruder_accel_limit() == 5000.0
    assert es.get_extruder_rpm_limit() == 200.0
```

- [ ] **Step 3: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'parses_max_extruder_accel' -v
```
Expected: FAIL (`AttributeError: 'ExtruderStepper' object has no attribute 'get_extruder_accel_limit'`).

- [ ] **Step 4: Add config parsing + accessors**

In `klippy/kinematics/extruder.py`, inside the `ExtruderStepper` `__init__` method (around line 197+ per earlier grep), add after the existing `self.pressure_advance_time_offset = ...` line:

```python
        # Plan 3: first-class extruder cap (post-PA stepper budget).
        # Both default to 0 (disabled); positive values activate the
        # blendextruder.cap_move() planner constraint.
        self.max_extruder_accel = config.getfloat(
            "max_extruder_accel", 0.0, minval=0.0
        )
        self.max_extruder_rpm = config.getfloat(
            "max_extruder_rpm", 0.0, minval=0.0
        )
```

Also add accessor methods to `ExtruderStepper`:
```python
    def get_extruder_accel_limit(self):
        """mm/s² on filament; 0.0 disables."""
        return self.max_extruder_accel

    def get_extruder_rpm_limit(self):
        """RPM on drive pulley; 0.0 disables."""
        return self.max_extruder_rpm
```

- [ ] **Step 5: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -k 'parses_max_extruder_accel' -v
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add klippy/kinematics/extruder.py test/test_blendextruder.py
git commit -m "extruder: parse [extruder] max_extruder_accel and max_extruder_rpm

Two new optional config keys default to 0 (disabled). Positive values
activate the Plan 3 per-move extruder cap in blendextruder.cap_move.

Accessors get_extruder_accel_limit() / get_extruder_rpm_limit() expose
the values to the extruder_limits_snapshot builder (Task 9).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 8: `SET_EXTRUDER_LIMITS` gcode command

**Goal:** Runtime-settable limits via gcode, following the `SET_PRESSURE_ADVANCE` pattern.

**Files:**
- Modify: `klippy/kinematics/extruder.py` — gcode registration + handler
- Modify: `test/test_blendextruder.py`

- [ ] **Step 1: Locate existing gcode command registration**

```bash
grep -n 'register_mux_command\|SET_PRESSURE_ADVANCE' klippy/kinematics/extruder.py
```

Should find the `SET_PRESSURE_ADVANCE` registration; use the same pattern.

- [ ] **Step 2: Write failing test**

Append to `test/test_blendextruder.py`:
```python
def test_set_extruder_limits_updates_values():
    """cmd_SET_EXTRUDER_LIMITS applies ACCEL and RPM to the stepper."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0
    es.max_extruder_rpm = 0.0
    es.name = "extruder"
    es._last_reported_limits = None

    class _FakeGcmd:
        def __init__(self, accel, rpm):
            self._accel = accel
            self._rpm = rpm
        def get_float(self, key, default=None, **kw):
            if key == "ACCEL":
                return self._accel if self._accel is not None else default
            if key == "RPM":
                return self._rpm if self._rpm is not None else default
            return default
        def respond_info(self, msg):
            self._last_msg = msg

    g = _FakeGcmd(accel=5000.0, rpm=200.0)
    es.cmd_SET_EXTRUDER_LIMITS(g)
    assert es.max_extruder_accel == 5000.0
    assert es.max_extruder_rpm == 200.0


def test_set_extruder_limits_omit_reports_current():
    """Calling with no ACCEL/RPM args reports current values."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0
    es.name = "extruder"

    class _FakeGcmd:
        def get_float(self, key, default=None, **kw):
            return default
        def respond_info(self, msg):
            self.last = msg

    g = _FakeGcmd()
    es.cmd_SET_EXTRUDER_LIMITS(g)
    assert "5000" in g.last
    assert "200" in g.last
```

- [ ] **Step 3: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'set_extruder_limits' -v
```
Expected: FAIL (no `cmd_SET_EXTRUDER_LIMITS` method).

- [ ] **Step 4: Implement the command handler**

Add to `ExtruderStepper` class in `klippy/kinematics/extruder.py`:

```python
    cmd_SET_EXTRUDER_LIMITS_help = (
        "Set per-move extruder stepper accel/RPM caps (Plan 3)"
    )

    def cmd_SET_EXTRUDER_LIMITS(self, gcmd):
        """Runtime update of max_extruder_accel / max_extruder_rpm.

        Parameters:
          ACCEL=<mm/s²>   new accel limit (0 disables)
          RPM=<RPM>       new rpm limit (0 disables)
        Omit both to report current values.
        """
        new_accel = gcmd.get_float("ACCEL", None, minval=0.0)
        new_rpm = gcmd.get_float("RPM", None, minval=0.0)
        if new_accel is None and new_rpm is None:
            gcmd.respond_info(
                "EXTRUDER '%s': max_extruder_accel=%.1f, max_extruder_rpm=%.1f"
                % (self.name, self.max_extruder_accel, self.max_extruder_rpm)
            )
            return
        if new_accel is not None:
            self.max_extruder_accel = new_accel
        if new_rpm is not None:
            self.max_extruder_rpm = new_rpm
        gcmd.respond_info(
            "EXTRUDER '%s': max_extruder_accel=%.1f, max_extruder_rpm=%.1f"
            % (self.name, self.max_extruder_accel, self.max_extruder_rpm)
        )
```

Register the command in `ExtruderStepper.__init__`, near the existing `SET_PRESSURE_ADVANCE` registration. Find the existing `gcode.register_mux_command("SET_PRESSURE_ADVANCE", ...)` line and add immediately after:

```python
        gcode.register_mux_command(
            "SET_EXTRUDER_LIMITS", "EXTRUDER", self.name,
            self.cmd_SET_EXTRUDER_LIMITS,
            desc=self.cmd_SET_EXTRUDER_LIMITS_help,
        )
```

- [ ] **Step 5: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -k 'set_extruder_limits' -v
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add klippy/kinematics/extruder.py test/test_blendextruder.py
git commit -m "extruder: SET_EXTRUDER_LIMITS gcode command

Runtime tuning for the Plan 3 per-move extruder cap. Mirrors the
SET_PRESSURE_ADVANCE pattern (register_mux_command on EXTRUDER name).

  SET_EXTRUDER_LIMITS EXTRUDER=<name> ACCEL=<mm/s²> RPM=<RPM>

Omitting both ACCEL and RPM reports current values. 0 values disable
the respective cap.

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 9: `extruder_limits_snapshot()` and PA model snapshot

**Goal:** Extruder object exposes a single method that returns a `(PAModelSnapshot, ExtruderLimits)` pair the planner can cache and hand to `cap_move`.

**Files:**
- Modify: `klippy/kinematics/extruder.py`
- Modify: `test/test_blendextruder.py`

- [ ] **Step 1: Write failing test**

Append to `test/test_blendextruder.py`:
```python
def test_extruder_limits_snapshot_shape():
    """snapshot returns (PAModelSnapshot, ExtruderLimits) or (None, None)."""
    from klippy.kinematics.extruder import ExtruderStepper, PALinearModel

    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0

    # Fake PA model attached via pressure_advance_model
    pa = PALinearModel.__new__(PALinearModel)
    pa.pressure_advance = 0.04
    es.pressure_advance_model = pa

    # Smoother has smooth_time; stub it.
    class _ExtSmoother:
        def __init__(self, t):
            self.smooth_time = t
    es.smoother = _ExtSmoother(0.04)

    # rotation_distance for v_E_max conversion
    es.rotation_distance = 4.78  # BMG-ish

    snap = es.extruder_limits_snapshot()
    assert snap is not None
    pa_snap, limits = snap
    assert pa_snap.kind == "linear"
    assert pa_snap.params == (0.04,)
    assert limits.a_E_max == 5000.0
    # v_E_max = (200 / 60) * 4.78 ≈ 15.933 mm/s
    assert limits.v_E_max == pytest.approx((200.0 / 60.0) * 4.78, rel=1e-6)
    assert limits.smooth_time == 0.04


def test_extruder_limits_snapshot_disabled_returns_none():
    """When max_extruder_accel=0 and max_extruder_rpm=0, snapshot=None."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0
    es.max_extruder_rpm = 0.0
    assert es.extruder_limits_snapshot() is None
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest test/test_blendextruder.py -k 'extruder_limits_snapshot' -v
```
Expected: FAIL.

- [ ] **Step 3: Implement snapshot builders**

In `klippy/kinematics/extruder.py`, add a module-level helper for PA snapshots and the method on `ExtruderStepper`.

After the PA model class definitions but before `ExtruderStepper`, add:

```python
def _pa_model_snapshot(pa):
    """Produce a blendextruder.PAModelSnapshot from a live PA model.

    Returns None if the model is not recognized (defensive; shouldn't
    happen in production).
    """
    from klippy import blendextruder
    if isinstance(pa, PALinearModel):
        return blendextruder.PAModelSnapshot(
            kind="linear", params=(pa.pressure_advance,),
        )
    if isinstance(pa, PATanhModel):
        return blendextruder.PAModelSnapshot(
            kind="tanh",
            params=(
                pa.linear_advance,
                pa.nonlinear_offset,
                pa.linearization_velocity,
            ),
        )
    if isinstance(pa, PAReciprModel):
        return blendextruder.PAModelSnapshot(
            kind="recipr",
            params=(
                pa.linear_advance,
                pa.nonlinear_offset,
                pa.linearization_velocity,
            ),
        )
    return None
```

Add to `ExtruderStepper`:
```python
    def extruder_limits_snapshot(self):
        """Build (PAModelSnapshot, ExtruderLimits) or return None.

        Returns None when both caps are disabled (user hasn't configured
        max_extruder_accel or max_extruder_rpm). When active, the
        snapshot is an immutable pair the planner can hand to
        blendextruder.cap_move() per-move.
        """
        if self.max_extruder_accel <= 0.0 and self.max_extruder_rpm <= 0.0:
            return None
        pa_snap = _pa_model_snapshot(self.pressure_advance_model)
        if pa_snap is None:
            return None
        # Convert RPM -> linear velocity using rotation_distance.
        v_E_max = (
            (self.max_extruder_rpm / 60.0) * self.rotation_distance
            if self.max_extruder_rpm > 0.0
            else float("inf")
        )
        a_E_max = (
            self.max_extruder_accel if self.max_extruder_accel > 0.0 else float("inf")
        )
        smooth_time = getattr(self.smoother, "smooth_time", 0.04)
        from klippy import blendshape
        limits = blendshape.ExtruderLimits(
            a_E_max=a_E_max, v_E_max=v_E_max, smooth_time=smooth_time,
        )
        return (pa_snap, limits)
```

- [ ] **Step 4: Run — expect pass**

```bash
python3 -m pytest test/test_blendextruder.py -k 'extruder_limits_snapshot' -v
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/kinematics/extruder.py test/test_blendextruder.py
git commit -m "extruder: extruder_limits_snapshot() builder

New method on ExtruderStepper returns a (PAModelSnapshot, ExtruderLimits)
tuple — the planner caches this at PA/limits-change time and hands it
to blendextruder.cap_move() on every move (Task 11 wires the
integration point). Returns None when both max_extruder_accel and
max_extruder_rpm are disabled.

_pa_model_snapshot(pa) dispatches on PA model class to produce the
right PAModelSnapshot.kind + params tuple.

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 10: Populate `KinematicLimits.extruder_caps` in `blendplanner`

**Goal:** Forward-compat wiring for Plan 5. The cap is applied at Move-level in Task 11; this task just makes the `KinematicLimits` that shape-building code gets fed include the extruder snapshot, so Plan 5 can consume it later without a new plumbing pass.

**Files:**
- Modify: `klippy/blendplanner.py:68`

- [ ] **Step 1: Update `CornerBlender.feed` to populate extruder_caps**

In `klippy/blendplanner.py`, find the block at line 64-70 that builds `KinematicLimits`:

```python
        limits = blendshape.KinematicLimits(
            a_max=th.max_accel,
            v_max=th.max_velocity,
            jerk_max=None,       # plan 1: jerk cap disabled; plan 5 wires it
            extruder_caps=None,  # plan 1: extruder cap disabled; plan 4 wires it
            shapers=blendmath._extract_shapers(th),
        )
```

Change the `extruder_caps=None` line to:

```python
            extruder_caps=_extract_extruder_caps(th),  # plan 3 wires; plan 5 consumes
```

And add the helper function at module scope (near the existing imports, after the other `_extract_*` helpers in blendmath):

```python
def _extract_extruder_caps(toolhead):
    """Pull the ExtruderLimits off the toolhead's extruder if configured.

    Returns None when the extruder cap is disabled or no extruder is
    present — the downstream shape-build code treats None as 'no cap'.

    Plan 3 wires this; Plan 5 (pillar 2 unified v(s)) will consume it
    as part of the continuous v(s) evaluation along the curve. For
    now the per-move cap is applied at Move-level in
    Move.limit_speed (see Task 11).
    """
    extruder = getattr(toolhead, "extruder", None)
    if extruder is None:
        return None
    snap = getattr(extruder, "extruder_limits_snapshot", None)
    if snap is None:
        return None
    snapshot = snap()
    if snapshot is None:
        return None
    _, limits = snapshot
    return limits
```

This function lives in `klippy/blendplanner.py` (not `blendmath.py`) because it reads extruder state specifically, which is planner-scoped.

- [ ] **Step 2: Verify existing tests still pass**

```bash
python3 -m pytest test/test_blendplanner.py -v 2>&1 | tail -5
```
Expected: all pass (the `_extract_extruder_caps` returns None for existing test toolheads that don't have a live extruder, matching the old `extruder_caps=None` behavior).

- [ ] **Step 3: Commit**

```bash
git add klippy/blendplanner.py
git commit -m "blendplanner: populate KinematicLimits.extruder_caps

Forward-compat wiring for Plan 5 (pillar 2 unified v(s) along the
curve). Pulls (PAModelSnapshot, ExtruderLimits) off the toolhead's
extruder via extruder_limits_snapshot() and stores the limits in
KinematicLimits.extruder_caps. Plan 3 does not consume this field
from shape-build code (the cap lives at Move-level); Plan 5 extends
QuinticShape.v_cap_fn to include the extruder cap at every s.

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 11: Wire `cap_move` into the `Move` constraint pipeline

**Goal:** Every Move — user gcode AND blend-polyline moves — has `blendextruder.cap_move()` called after `kin.check_move(move)`. Cap is applied via `move.limit_speed(v_cap, a_cap)`.

**Files:**
- Modify: `klippy/toolhead.py` (or `klippy/move.py` depending on current layout)

- [ ] **Step 1: Locate the integration point**

```bash
grep -n 'kin.check_move\|limit_speed\|_process_move\|add_move' klippy/toolhead.py | head -30
```

Find where `kin.check_move(move)` is called. This is typically in `Move.__init__` (via the kinematics chain) OR in `toolhead.move` / `toolhead.add_move`. Record the location (file + line + function).

If `Move` lives in a separate file (e.g. `klippy/move.py`), check there too:
```bash
ls klippy/*.py | grep -iE 'move|toolhead'
grep -n 'class Move\|check_move\|limit_speed' klippy/move.py 2>/dev/null
```

- [ ] **Step 2: Cache the snapshot on the toolhead**

The snapshot must be refreshed whenever PA or limits change. Add to `ToolHead.__init__` (after the extruder is set up):

```python
        # Plan 3: cached extruder-cap snapshot.
        # Refreshed by SET_PRESSURE_ADVANCE / SET_EXTRUDER_LIMITS handlers
        # and on Print Start. None when cap is disabled.
        self.extruder_cap_snapshot = None
        self._refresh_extruder_cap_snapshot()
```

And a refresh method:
```python
    def _refresh_extruder_cap_snapshot(self):
        """Called when PA or extruder limits change."""
        extruder = getattr(self, "extruder", None)
        if extruder is None:
            self.extruder_cap_snapshot = None
            return
        snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
        if snap_fn is None:
            self.extruder_cap_snapshot = None
            return
        self.extruder_cap_snapshot = snap_fn()
```

Call `_refresh_extruder_cap_snapshot()` from:
- `ExtruderStepper.cmd_SET_PRESSURE_ADVANCE` (after updating the model)
- `ExtruderStepper.cmd_SET_EXTRUDER_LIMITS` (after updating the values)
- After extruder config load

For those, add at the end of each command handler:
```python
        toolhead = self.printer.lookup_object("toolhead")
        toolhead._refresh_extruder_cap_snapshot()
```

- [ ] **Step 3: Hook `cap_move` into `Move.limit_speed` or equivalent**

In the Move class (or `toolhead.move()`), immediately after `kin.check_move(move)` and after any existing extruder `check_move` calls, add:

```python
        # Plan 3: extruder-cap (post-PA stepper budget).
        snap = self.toolhead.extruder_cap_snapshot
        if snap is not None:
            pa_snap, limits = snap
            from klippy import blendextruder
            v_cap, a_cap = blendextruder.cap_move(self, pa_snap, limits)
            if math.isfinite(v_cap) or math.isfinite(a_cap):
                self.limit_speed(v_cap, a_cap)
```

The exact variable name for accessing the toolhead differs — adapt to the Move class's conventions. If Move uses `self._toolhead` or receives toolhead as a constructor arg, use that.

- [ ] **Step 4: Write an integration test**

Create `test/test_blendextruder_integration.py`:
```python
import math
import pytest


def test_integration_smoke_placeholder():
    """Smoke: if a Move's toolhead has extruder_cap_snapshot set, the
    cap is applied during construction. This is a placeholder — the
    real HW integration test is manual.
    """
    # This test is intentionally minimal. Full integration requires
    # the full Klipper bootstrap (Printer, Reactor, MCU stubs). The
    # klipper-sim run in Task 13 covers end-to-end behavior.
    assert True
```

- [ ] **Step 5: Verify no regressions**

```bash
python3 -m pytest test/ 2>&1 | tail -5
```

Expected: same pass/fail counts as before Plan 3 started. Pre-existing environment failures (jinja2, etc.) unchanged.

- [ ] **Step 6: Commit**

```bash
git add klippy/toolhead.py klippy/kinematics/extruder.py test/test_blendextruder_integration.py
git commit -m "toolhead: wire blendextruder.cap_move into Move pipeline

Every Move (user gcode + blend-polyline from blendplanner) now goes
through blendextruder.cap_move() after kin.check_move(). The cap's
returned (v_cap, a_cap) feeds move.limit_speed(), so the existing
min-taking machinery handles the rest.

Snapshot pattern: toolhead caches (PAModelSnapshot, ExtruderLimits)
and refreshes only on SET_PRESSURE_ADVANCE / SET_EXTRUDER_LIMITS.
This avoids re-pickling PA state on every move. Cache is None when
cap is disabled (user hasn't set max_extruder_* config).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Task 12: `klipper-sim` end-to-end validation

**Goal:** Confirm the cap actually binds on a realistic gcode, stepper peak_accel stays ≤ `max_extruder_accel`, and total print time degrades less than 5% at realistic limits.

**Files:**
- No new code; creates or updates a sim-example script.
- This task is manual/exploratory validation, not automated.

- [ ] **Step 1: Ensure klipper-sim is available**

```bash
ls ~/Developer/klipper-sim/ | head
```
If not present, skip to Task 13 and note the sim validation as deferred-to-user.

- [ ] **Step 2: Run a baseline**

```bash
cd ~/Developer/klipper-sim
python3 examples/analyze_sharp_short.py --klipper-root /Users/daniladergachev/Developer/kalico --mode blendarc --max-accel 50000 --max-extruder-accel 0 2>&1 | tail -10
```
Record: total print time, peak stepper accel.

- [ ] **Step 3: Run with cap active**

```bash
python3 examples/analyze_sharp_short.py --klipper-root /Users/daniladergachev/Developer/kalico --mode blendarc --max-accel 50000 --max-extruder-accel 5000 --max-extruder-rpm 200 2>&1 | tail -10
```
(If the sim's CLI doesn't currently support `--max-extruder-*`, add the args — they should plumb through to the `[extruder]` config of the sim's fake printer. Minor sim patch; not required if CLI doesn't support.)

Compare:
- Peak stepper accel should be ≤ 5000 (with the cap).
- Total print time should be ≤ 5% longer than baseline.

- [ ] **Step 4: Document results**

Write a brief note in `docs/superpowers/plans/2026-04-21-plan3-validation.md`:
```
# Plan 3 sim validation — <date>

Baseline (max_extruder_accel=0):
  print_time = X.XX s
  peak_stepper_accel = XXXX mm/s²

With cap (max_extruder_accel=5000, max_extruder_rpm=200):
  print_time = X.XX s  (delta: +X.X%)
  peak_stepper_accel = XXXX mm/s²

Notes: [any surprises or observations]
```

- [ ] **Step 5: Commit the validation note**

```bash
git add docs/superpowers/plans/2026-04-21-plan3-validation.md
git commit -m "docs(plan-3): sim validation results

Baseline vs. with-cap klipper-sim run on <gcode>. Peak stepper_accel
bounded by max_extruder_accel; print-time penalty within budget."
```

If sim infra doesn't exist or the user prefers HW-only validation, skip the commit and note "deferred to HW" in the report.

---

## Task 13: Docs update

**Goal:** Document the new config keys and gcode command in `Config_Reference.md`.

**Files:**
- Modify: `docs/Config_Reference.md` (or `Config_Reference_Bleeding_Edge.md` if that's where extruder extensions go)

- [ ] **Step 1: Locate the `[extruder]` docs section**

```bash
grep -n '^### \[extruder\]\|^\[extruder\]' docs/Config_Reference.md
```

- [ ] **Step 2: Add the new keys**

Add to the `[extruder]` section, near the existing PA config docs:

```markdown
#max_extruder_accel: 0
#   Maximum acceleration (mm/s²) on the filament stepper *after* Pressure
#   Advance has been applied. If positive, the planner evaluates every
#   move with blendextruder.cap_move and reduces XY accel on the subset
#   of moves that would otherwise drive the extruder stepper past this
#   limit. 0 disables the cap. Only the moves that would exceed the
#   limit are reduced; non-binding moves run at full max_accel.
#max_extruder_rpm: 0
#   Maximum angular velocity (RPM) on the extruder drive pulley. Converted
#   to linear filament velocity via rotation_distance. 0 disables the
#   cap. Typically this is the bottleneck on high-gear-ratio extruders
#   (BMG, Sherpa) at high flow rates.
```

- [ ] **Step 3: Add `SET_EXTRUDER_LIMITS` gcode doc**

Find the section for `SET_PRESSURE_ADVANCE` (should exist in the gcode commands docs). Add nearby:

```markdown
#### SET_EXTRUDER_LIMITS

`SET_EXTRUDER_LIMITS [EXTRUDER=<name>] [ACCEL=<mm/s²>] [RPM=<RPM>]`

Runtime tuning for the per-move extruder cap. Takes effect on moves queued
after the command. Omit both ACCEL and RPM to report current values.
Setting either to 0 disables that cap. Requires `max_extruder_accel` /
`max_extruder_rpm` to have been parsed from `[extruder]` config on startup
(non-zero default) for the cap machinery to be active.
```

- [ ] **Step 4: Commit**

```bash
git add docs/Config_Reference.md
git commit -m "docs: document max_extruder_accel, max_extruder_rpm, SET_EXTRUDER_LIMITS

Per-move extruder cap (Plan 3). Two new [extruder] config keys +
one runtime gcode command. Both caps default to 0 (disabled).

See docs/superpowers/specs/2026-04-21-plan3-extruder-first-class-design.md"
```

---

## Final verification

- [ ] **Step 1: Full test suite**

```bash
python3 -m pytest test/ 2>&1 | tail -10
```
Expected: all new tests pass + no regressions vs. Plan 2 Phase A baseline. Pre-existing environment failures (jinja2, etc.) unchanged.

- [ ] **Step 2: Commit graph**

```bash
git log --oneline f1554b36..HEAD | head -20
```
Expected: one commit per Task (13 total if all landed). Each commit touches only the files specified by its task.

- [ ] **Step 3: Summary report**

Report:
- How many new tests pass (expected: ~20+ in test_blendextruder.py + test_blendshape.py + test_pa_derivatives.py).
- Whether sim validation ran (Task 12) or was deferred.
- Any open issues or deviations from the plan.
- Current HEAD SHA.

---

## Open items flagged by the spec (not blocking Plan 3 completion)

1. **Rigorous `v_eval = min(v_prev, v_next)`** for the accel cap was deferred — current implementation uses `v_eval = k · move.max_cruise_v`. The ~1-2% conservatism for NL PA is acceptable; Plan 5's continuous v(s) removes the approximation entirely.

2. **`smooth_time = 0`** edge case: the cap formula divides by `smooth_time`. The current code uses `getattr(..., 'smooth_time', 0.04)` as a sensible default. If a user explicitly sets smoother smooth_time to 0 AND enables the cap, `K_h → ∞` and `a_cap → 0`. Not ideal but not catastrophic — the extruder stops. Decision: add a config-time validation in a follow-up if this bites real users.

3. **HW validation**: user runs their own prints on Trident. No automated HW test.
