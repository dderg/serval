# j_eff Derivation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `klippy/blendshaper.py` (the pure-math shaper-derived jerk bound module) and extend `klippy/blendmath.py`'s `blend_from_moves` to pull shapers from a Kalico toolhead, exactly as specified in `docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md`.

**Architecture:** Two files touched. `klippy/blendshaper.py` is a new standalone module with zero Kalico imports in its core (dataclasses, `shaper_span`, `axis_projections`, `axis_in_plane`, `compute_shaper_bounds`). `klippy/blendmath.py` gains a `_extract_shapers` helper and extends `blend_from_moves` with an optional `toolhead` parameter that triggers a two-pass `blend_geometry` iteration + `v_step_cap` application. `blend_geometry`, `BlendArc`, and `segment_arc` are untouched.

**Tech Stack:** Python 3 stdlib only in `blendshaper.py` (`math`, `dataclasses`, `typing`). The adapter in `blendmath.py` touches `klippy.extras.shaper_calibrate.ShaperCalibrate.find_shaper_max_accel` (already importable; runs without numpy for this code path, verified) and `klippy.extras.input_shaper.InputShaper` via `toolhead.lookup_object("input_shaper")`. Pytest for tests. No new dependencies.

---

## Reference material

Before starting, read these:

- `docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md` — the design spec this plan implements.
- `docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md` — the sibling spec for the module this extends.
- `klippy/blendmath.py` — the module being extended; read the existing `blend_from_moves` and its test fixtures in `test/test_blendmath.py`.
- `klippy/extras/shaper_defs.py` — shaper pulse tables.
- `klippy/extras/shaper_calibrate.py:361-371` — `find_shaper_max_accel` (the value we anchor to).
- `klippy/extras/input_shaper.py:14-65,123-144` — `InputShaperParams`, `AxisInputShaper`, `InputShaper`.

## File structure

- **Create `klippy/blendshaper.py`** — the new module. Dataclasses, pure functions, no Kalico imports.
- **Modify `klippy/blendmath.py`** — add `_extract_shapers` helper, extend `blend_from_moves` with toolhead path.
- **Create `test/test_blendshaper.py`** — new pytest module for the pure-math core.
- **Modify `test/test_blendmath.py`** — add integration fixtures exercising the new `blend_from_moves` path through a fake toolhead.

Nothing else is created or modified. No config parser changes, no toolhead changes, no kinematics changes.

## Conventions used throughout the code

- All public functions in `blendshaper.py` take and return plain Python values (tuples, dicts, floats, dataclasses). No numpy.
- Axis keys are lowercase strings (`"x"`, `"y"`, `"z"`) matching Kalico's `AxisInputShaper.axis` attribute.
- `Vec3 = tuple[float, float, float]`, reused from `blendmath.Vec3`.
- Epsilons: `PROJECTION_EPS = 1e-9` for "significant axis projection" tests in Bounds (b) and (c).
- For tests, prefer `pytest.approx(value, rel=1e-9)` for derived numeric values and exact equality for integer/structural properties.

---

## Task 1: Module skeleton with dataclasses

**Files:**
- Create: `klippy/blendshaper.py`
- Create: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_blendshaper.py`:

```python
# test/test_blendshaper.py
import math

import pytest

from klippy import blendshaper


def test_axis_shaper_snapshot_fields():
    snap = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=150.0,
        damping_ratio=0.1,
        A_axis=87685.6,
    )
    assert snap.axis == "x"
    assert snap.shaper_type == "zv"
    assert snap.shaper_freq == 150.0
    assert snap.damping_ratio == 0.1
    assert snap.A_axis == 87685.6


def test_shaper_bounds_fields():
    bounds = blendshaper.ShaperBounds(
        j_eff=3.97e6,
        v_step_cap=132.8,
    )
    assert bounds.j_eff == 3.97e6
    assert bounds.v_step_cap == 132.8
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'klippy.blendshaper'`.

- [ ] **Step 3: Create the module with dataclasses**

Create `klippy/blendshaper.py`:

```python
# klippy/blendshaper.py
# Shaper-derived jerk bound module for corner blending.
#
# Given a toolhead's per-axis input-shaper configuration and a
# blend-arc corner geometry, computes the effective jerk ceiling
# (j_eff) passed to blendmath.blend_geometry plus a per-axis
# entry-step velocity cap (v_step_cap) applied post-hoc.
#
# Pure math: zero Kalico imports. All per-axis shaper state is
# carried in AxisShaperSnapshot records created by the adapter.
#
# See docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

Vec3 = Tuple[float, float, float]

PROJECTION_EPS = 1e-9


@dataclass(frozen=True)
class AxisShaperSnapshot:
    axis: str
    shaper_type: Optional[str]
    shaper_freq: float
    damping_ratio: float
    A_axis: float


@dataclass(frozen=True)
class ShaperBounds:
    j_eff: float
    v_step_cap: float
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: add module skeleton with dataclasses"
```

---

## Task 2: shaper_span function

**Files:**
- Modify: `klippy/blendshaper.py`
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendshaper.py`:

```python
def test_shaper_span_zv():
    # t_d = 1/(f·sqrt(1-zeta^2)); T_span = 0.5 * t_d
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("zv", f, zeta) == pytest.approx(
        0.5 * t_d, rel=1e-12
    )


def test_shaper_span_mzv():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("mzv", f, zeta) == pytest.approx(
        0.75 * t_d, rel=1e-12
    )


def test_shaper_span_zvd():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("zvd", f, zeta) == pytest.approx(
        1.0 * t_d, rel=1e-12
    )


def test_shaper_span_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("ei", f, zeta) == pytest.approx(
        1.0 * t_d, rel=1e-12
    )


def test_shaper_span_2hump_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("2hump_ei", f, zeta) == pytest.approx(
        1.5 * t_d, rel=1e-12
    )


def test_shaper_span_3hump_ei():
    f = 100.0
    zeta = 0.1
    t_d = 1.0 / (f * math.sqrt(1.0 - zeta * zeta))
    assert blendshaper.shaper_span("3hump_ei", f, zeta) == pytest.approx(
        2.0 * t_d, rel=1e-12
    )


def test_shaper_span_damping_effect():
    # Higher damping ratio stretches t_d.
    f = 100.0
    span_low = blendshaper.shaper_span("zv", f, 0.05)
    span_high = blendshaper.shaper_span("zv", f, 0.2)
    assert span_high > span_low


def test_shaper_span_unknown_raises():
    with pytest.raises(ValueError):
        blendshaper.shaper_span("not_a_shaper", 100.0, 0.1)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: 8 FAIL with `AttributeError: module 'klippy.blendshaper' has no attribute 'shaper_span'`.

- [ ] **Step 3: Add shaper_span to the module**

Append to `klippy/blendshaper.py`:

```python
# Pulse-sequence span in units of the damped period, keyed by shaper name.
# Values match klippy/extras/shaper_defs.py exactly (last T[i] of each).
_SHAPER_SPAN_FACTOR = {
    "zv": 0.5,
    "mzv": 0.75,
    "zvd": 1.0,
    "ei": 1.0,
    "2hump_ei": 1.5,
    "3hump_ei": 2.0,
}


def shaper_span(shaper_type: str, shaper_freq: float, damping_ratio: float) -> float:
    if shaper_type not in _SHAPER_SPAN_FACTOR:
        raise ValueError("unknown shaper type: %r" % (shaper_type,))
    factor = _SHAPER_SPAN_FACTOR[shaper_type]
    t_d = 1.0 / (shaper_freq * math.sqrt(1.0 - damping_ratio * damping_ratio))
    return factor * t_d
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: shaper pulse-sequence span table"
```

---

## Task 3: axis_projections and axis_in_plane helpers

**Files:**
- Modify: `klippy/blendshaper.py`
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendshaper.py`:

```python
def test_axis_projections_unit_x():
    projs = blendshaper.axis_projections((1.0, 0.0, 0.0))
    assert projs["x"] == pytest.approx(1.0, abs=1e-12)
    assert projs["y"] == pytest.approx(0.0, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_projections_45_deg_xy():
    s = 1.0 / math.sqrt(2.0)
    projs = blendshaper.axis_projections((s, s, 0.0))
    assert projs["x"] == pytest.approx(s, abs=1e-12)
    assert projs["y"] == pytest.approx(s, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_projections_negative_components_return_abs():
    projs = blendshaper.axis_projections((-0.6, 0.8, 0.0))
    assert projs["x"] == pytest.approx(0.6, abs=1e-12)
    assert projs["y"] == pytest.approx(0.8, abs=1e-12)
    assert projs["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_in_plane_xy_plane():
    # Arc plane normal along +Z: x and y lie fully in the plane.
    in_plane = blendshaper.axis_in_plane((0.0, 0.0, 1.0))
    assert in_plane["x"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["z"] == pytest.approx(0.0, abs=1e-12)


def test_axis_in_plane_yz_plane():
    # Arc plane normal along +X: y and z lie fully in the plane.
    in_plane = blendshaper.axis_in_plane((1.0, 0.0, 0.0))
    assert in_plane["x"] == pytest.approx(0.0, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)
    assert in_plane["z"] == pytest.approx(1.0, abs=1e-12)


def test_axis_in_plane_tilted():
    # Plane normal at 45° between X and Z: x and z partially in-plane.
    s = 1.0 / math.sqrt(2.0)
    in_plane = blendshaper.axis_in_plane((s, 0.0, s))
    # sqrt(1 - (1/sqrt(2))^2) = sqrt(1 - 0.5) = sqrt(0.5) = 1/sqrt(2)
    assert in_plane["x"] == pytest.approx(s, abs=1e-12)
    assert in_plane["y"] == pytest.approx(1.0, abs=1e-12)  # perpendicular to normal
    assert in_plane["z"] == pytest.approx(s, abs=1e-12)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: 6 FAIL with `AttributeError: module 'klippy.blendshaper' has no attribute 'axis_projections'`.

- [ ] **Step 3: Add the helpers to the module**

Append to `klippy/blendshaper.py`:

```python
_AXES = ("x", "y", "z")


def axis_projections(n_hat: Vec3) -> dict:
    """|n̂·ê_axis| per axis. Used by Bound (b) entry-step."""
    return {
        "x": abs(n_hat[0]),
        "y": abs(n_hat[1]),
        "z": abs(n_hat[2]),
    }


def axis_in_plane(p_hat: Vec3) -> dict:
    """√(1 - |p̂·ê_axis|²) per axis — projection of each basis
    axis onto the arc plane. 1 for fully in-plane axes, 0 for
    fully out-of-plane. Used by Bound (c) rotation jerk."""
    return {
        "x": math.sqrt(max(0.0, 1.0 - p_hat[0] * p_hat[0])),
        "y": math.sqrt(max(0.0, 1.0 - p_hat[1] * p_hat[1])),
        "z": math.sqrt(max(0.0, 1.0 - p_hat[2] * p_hat[2])),
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: axis projection helpers"
```

---

## Task 4: compute_shaper_bounds — Bound (b) entry-step, one axis

**Files:**
- Modify: `klippy/blendshaper.py`
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendshaper.py`:

```python
def test_compute_shaper_bounds_step_single_axis_x_projection():
    # Contrived n̂ with |n̂·x̂|=1/√2 and |n̂·ŷ|=1/√2 so the single shaped axis
    # (X) contributes to Bound (b). Unit test of the formula; n̂ here is a
    # direct input, not derived from a corner.
    # v_step_cap = √(A_x · R / (1/√2)) = √(A_x · R · √2)
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    expected_v_step = math.sqrt(10000.0 * 0.5 * math.sqrt(2.0))
    assert bounds.v_step_cap == pytest.approx(expected_v_step, rel=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv-test/bin/pytest test/test_blendshaper.py::test_compute_shaper_bounds_step_single_axis_90deg -v`
Expected: FAIL with `AttributeError: module 'klippy.blendshaper' has no attribute 'compute_shaper_bounds'`.

- [ ] **Step 3: Implement minimal compute_shaper_bounds covering only Bound (b)**

Append to `klippy/blendshaper.py`:

```python
def compute_shaper_bounds(
    shapers: Iterable[AxisShaperSnapshot],
    R: float,
    n_hat: Vec3,
    p_hat: Vec3,
) -> ShaperBounds:
    """Compute (j_eff, v_step_cap) for a blend arc.

    shapers: per-axis shaper snapshots. Axes with shaper_freq <= 0
             contribute no bound.
    R:       arc radius (mm).
    n_hat:   unit arc normal at entry (toward arc center).
    p_hat:   unit arc plane normal.
    """
    n_projs = axis_projections(n_hat)

    v_step_cap = float("inf")
    for snap in shapers:
        if snap.shaper_freq is None or snap.shaper_freq <= 0.0:
            continue
        proj = n_projs.get(snap.axis, 0.0)
        if proj < PROJECTION_EPS:
            continue
        v_axis = math.sqrt(snap.A_axis * R / proj)
        if v_axis < v_step_cap:
            v_step_cap = v_axis

    # j_eff filled in a subsequent task.
    return ShaperBounds(j_eff=float("inf"), v_step_cap=v_step_cap)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: all PASS (previous tests still green).

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: compute per-axis entry-step velocity cap"
```

---

## Task 5: compute_shaper_bounds — Bound (c) rotation jerk, one axis

**Files:**
- Modify: `klippy/blendshaper.py`
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendshaper.py`:

```python
def test_compute_shaper_bounds_jerk_single_axis_in_plane():
    # Single shaped axis X, arc in XY plane → axis_in_plane_x = 1.
    # j_eff = A_x / T_x.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == pytest.approx(10000.0 / T_x, rel=1e-9)


def test_compute_shaper_bounds_jerk_axis_partially_in_plane():
    # Single shaped axis X, arc plane normal at 45° between X and Z:
    # axis_in_plane_x = sqrt(1 - 0.5) = 1/sqrt(2).
    # j_x_effective = A_x / (T_x · (1/sqrt(2))) = A_x · sqrt(2) / T_x.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x",
        shaper_type="zv",
        shaper_freq=100.0,
        damping_ratio=0.1,
        A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    s = 1.0 / math.sqrt(2.0)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(s, 0.0, s),   # arbitrary unit vector
        p_hat=(s, 0.0, s),   # plane normal at 45° in XZ
    )
    expected_j = 10000.0 / (T_x * s)
    assert bounds.j_eff == pytest.approx(expected_j, rel=1e-9)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: 2 FAIL — `j_eff` still hardcoded to `inf`.

- [ ] **Step 3: Add Bound (c) to compute_shaper_bounds**

Edit `klippy/blendshaper.py` — replace the body of `compute_shaper_bounds` with:

```python
def compute_shaper_bounds(
    shapers: Iterable[AxisShaperSnapshot],
    R: float,
    n_hat: Vec3,
    p_hat: Vec3,
) -> ShaperBounds:
    """Compute (j_eff, v_step_cap) for a blend arc.

    shapers: per-axis shaper snapshots. Axes with shaper_freq <= 0
             contribute no bound.
    R:       arc radius (mm).
    n_hat:   unit arc normal at entry (toward arc center).
    p_hat:   unit arc plane normal.
    """
    n_projs = axis_projections(n_hat)
    in_plane = axis_in_plane(p_hat)

    v_step_cap = float("inf")
    j_eff = float("inf")
    for snap in shapers:
        if snap.shaper_freq is None or snap.shaper_freq <= 0.0:
            continue
        # Bound (b) entry-step.
        proj = n_projs.get(snap.axis, 0.0)
        if proj >= PROJECTION_EPS:
            v_axis = math.sqrt(snap.A_axis * R / proj)
            if v_axis < v_step_cap:
                v_step_cap = v_axis
        # Bound (c) rotation jerk.
        ip = in_plane.get(snap.axis, 0.0)
        if ip >= PROJECTION_EPS:
            T_a = shaper_span(snap.shaper_type, snap.shaper_freq, snap.damping_ratio)
            j_axis = snap.A_axis / (T_a * ip)
            if j_axis < j_eff:
                j_eff = j_axis

    return ShaperBounds(j_eff=j_eff, v_step_cap=v_step_cap)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py test/test_blendshaper.py
git commit -m "blendshaper: per-axis rotation-jerk bound"
```

---

## Task 6: Multi-axis reduction and degenerate cases

**Files:**
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the failing / pinning tests**

Append to `test/test_blendshaper.py`:

```python
def test_compute_shaper_bounds_y_binds_over_x():
    # X at 150Hz, Y at 80Hz; Y has smaller A/T → Y binds on Bound (c).
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type="zv", shaper_freq=150.0,
        damping_ratio=0.1, A_axis=87000.0,
    )
    snap_y = blendshaper.AxisShaperSnapshot(
        axis="y", shaper_type="zv", shaper_freq=80.0,
        damping_ratio=0.1, A_axis=25000.0,
    )
    T_x = blendshaper.shaper_span("zv", 150.0, 0.1)
    T_y = blendshaper.shaper_span("zv", 80.0, 0.1)
    assert 25000.0 / T_y < 87000.0 / T_x  # Y is stricter for jerk

    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x, snap_y],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == pytest.approx(25000.0 / T_y, rel=1e-9)


def test_compute_shaper_bounds_no_shapers_returns_infinity():
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[],
        R=0.5,
        n_hat=(1.0, 0.0, 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == float("inf")
    assert bounds.v_step_cap == float("inf")


def test_compute_shaper_bounds_unshaped_axis_contributes_nothing():
    # freq=0 means no shaper — axis is skipped.
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type=None, shaper_freq=0.0,
        damping_ratio=0.1, A_axis=0.0,
    )
    snap_y = blendshaper.AxisShaperSnapshot(
        axis="y", shaper_type="zv", shaper_freq=80.0,
        damping_ratio=0.1, A_axis=25000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x, snap_y],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),   # n̂ along +y
        p_hat=(0.0, 0.0, 1.0),
    )
    # Only Y contributes.
    T_y = blendshaper.shaper_span("zv", 80.0, 0.1)
    assert bounds.j_eff == pytest.approx(25000.0 / T_y, rel=1e-9)
    assert bounds.v_step_cap == pytest.approx(
        math.sqrt(25000.0 * 0.5 / 1.0), rel=1e-9
    )


def test_compute_shaper_bounds_out_of_plane_shaper_contributes_nothing():
    # XY arc, only Z shaped: axis_in_plane_z = 0, |n̂·ẑ| = 0.
    # Z contributes to neither bound → both return infinity.
    snap_z = blendshaper.AxisShaperSnapshot(
        axis="z", shaper_type="zv", shaper_freq=50.0,
        damping_ratio=0.1, A_axis=5000.0,
    )
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_z],
        R=0.5,
        n_hat=(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.j_eff == float("inf")
    assert bounds.v_step_cap == float("inf")


def test_compute_shaper_bounds_small_projection_axis_skipped_for_step():
    # X shaped, but n̂ is (0, 1, 0) — no X projection for step bound.
    # Bound (b) contributes nothing from X; Bound (c) still does
    # (axis_in_plane_x = 1).
    snap_x = blendshaper.AxisShaperSnapshot(
        axis="x", shaper_type="zv", shaper_freq=100.0,
        damping_ratio=0.1, A_axis=10000.0,
    )
    T_x = blendshaper.shaper_span("zv", 100.0, 0.1)
    bounds = blendshaper.compute_shaper_bounds(
        shapers=[snap_x],
        R=0.5,
        n_hat=(0.0, 1.0, 0.0),   # no X component
        p_hat=(0.0, 0.0, 1.0),
    )
    assert bounds.v_step_cap == float("inf")  # no X-projected step
    assert bounds.j_eff == pytest.approx(10000.0 / T_x, rel=1e-9)  # X still in plane
```

- [ ] **Step 2: Run tests to verify they pass (or fail)**

Run: `.venv-test/bin/pytest test/test_blendshaper.py -v`
Expected: all PASS. (Task 5's implementation already handles these cases — these tests pin the behavior.)

- [ ] **Step 3: Commit**

```bash
git add test/test_blendshaper.py
git commit -m "blendshaper: pin multi-axis and degenerate-case behavior"
```

---

## Task 7: _extract_shapers adapter helper

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

Context: the adapter needs to pull shaper state off a running toolhead into a list of `AxisShaperSnapshot`. For tests we use a fake toolhead; the real one exposes `lookup_object("input_shaper")` → `InputShaper` object with a `get_shapers()` method returning a list of `AxisInputShaper`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendmath.py`:

```python
class _FakeAxisInputShaper:
    def __init__(self, axis, shaper_type, freq, damping_ratio=0.1):
        self.axis = axis
        self._type = shaper_type
        self._freq = freq
        self._damping = damping_ratio

    class _Params:
        def __init__(self, outer):
            self.shaper_type = outer._type
            self.shaper_freq = outer._freq
            self.damping_ratio = outer._damping

    @property
    def params(self):
        return self._Params(self)


class _FakeInputShaper:
    def __init__(self, shapers):
        self._shapers = shapers

    def get_shapers(self):
        return list(self._shapers)


class _FakePrinterObject:
    def __init__(self, input_shaper):
        self._is = input_shaper

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


class _FakeToolheadWithShapers:
    def __init__(self, input_shaper):
        self.printer = _FakePrinterObject(input_shaper)


def test_extract_shapers_two_axes():
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 150.0
    assert snaps_by_axis["x"].shaper_type == "zv"
    assert snaps_by_axis["y"].shaper_freq == 80.0
    # A_axis is populated from find_shaper_max_accel — positive for shaped axes.
    assert snaps_by_axis["x"].A_axis > 0.0
    assert snaps_by_axis["y"].A_axis > 0.0
    # X should have larger A_axis (higher frequency, more accel budget).
    assert snaps_by_axis["x"].A_axis > snaps_by_axis["y"].A_axis


def test_extract_shapers_none_toolhead_returns_empty():
    assert blendmath._extract_shapers(None) == []


def test_extract_shapers_no_input_shaper_module_returns_empty():
    class _FakePrinterObjectNoIS:
        def lookup_object(self, name, default=None):
            return default

    class _FakeToolhead:
        printer = _FakePrinterObjectNoIS()

    assert blendmath._extract_shapers(_FakeToolhead()) == []


def test_extract_shapers_unshaped_axis_has_zero_A():
    # Axis with shaper_freq=0 is unshaped → snapshot carries A_axis=0.
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 0.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 0.0
    assert snaps_by_axis["x"].A_axis == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendmath.py -v -k extract_shapers`
Expected: FAIL with `AttributeError: module 'klippy.blendmath' has no attribute '_extract_shapers'`.

- [ ] **Step 3: Implement _extract_shapers**

Add to the top of `klippy/blendmath.py` (near other imports / after the existing module docstring):

```python
from klippy import blendshaper
```

Then append the helper near `blend_from_moves`:

```python
def _extract_shapers(toolhead):
    """Pull per-axis shaper snapshots off a Kalico toolhead.

    Returns an empty list if `toolhead` is None or no `input_shaper`
    module is loaded. Unshaped axes (shaper_freq == 0) are included
    with A_axis = 0 so the caller sees them and can still reason
    about missing axes.
    """
    if toolhead is None:
        return []
    printer = getattr(toolhead, "printer", None)
    if printer is None:
        return []
    is_obj = printer.lookup_object("input_shaper", None)
    if is_obj is None:
        return []

    # Lazy-import ShaperCalibrate to avoid a hard dependency when
    # blendmath is imported in a non-Kalico context (e.g. pytest).
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs

    sc = ShaperCalibrate(printer=None)
    shaper_factory = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

    snaps = []
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        freq = float(params.shaper_freq)
        shaper_type = params.shaper_type if freq > 0.0 else None
        damping_ratio = float(params.damping_ratio)
        if freq > 0.0 and shaper_type in shaper_factory:
            impulses = shaper_factory[shaper_type](freq, damping_ratio)
            A_axis = float(sc.find_shaper_max_accel(impulses, scv=0.0))
        else:
            A_axis = 0.0
        snaps.append(blendshaper.AxisShaperSnapshot(
            axis=axis_shaper.axis,
            shaper_type=shaper_type,
            shaper_freq=freq,
            damping_ratio=damping_ratio,
            A_axis=A_axis,
        ))
    return snaps
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendmath.py -v`
Expected: all PASS, including the new `_extract_shapers` cases.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: toolhead shaper extraction adapter"
```

---

## Task 8: Extend blend_from_moves with toolhead path and two-pass iteration

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

Context: the existing `blend_from_moves(prev_move, next_move, corner_deviation, j_eff)` single-passes through `blend_geometry`. We add an optional `toolhead` parameter. When given, we:
1. First-pass `blend_geometry` with `j_eff=inf` to get initial `R`, `entry_pt`, `center`, `plane_normal`.
2. Compute `ShaperBounds` via `blendshaper.compute_shaper_bounds`.
3. Second-pass `blend_geometry` with the derived `j_eff`.
4. Return a `BlendArc` with `v_cap = min(second-pass v_cap, bounds.v_step_cap)`.

Backward compatibility: existing callers pass a scalar `j_eff` and no toolhead. Their behavior is unchanged.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendmath.py`:

```python
def test_blend_from_moves_with_toolhead_derives_j_eff():
    # Set up a 90° XY corner with X=ZV@150Hz, Y=ZV@80Hz. Expect
    # v_cap to match the spec's numeric sanity: ~99.8 mm/s at R=0.5mm.
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e12,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e12,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    # corner_deviation is loose enough that R_tol is large; R_mid caps
    # at min(L)·cot(45°) = 50, so R_tol binds. We still expect R ≈ 0.5mm
    # if we set corner_deviation to produce that.
    # R_tol = corner_deviation · cos(45°)/(1-cos(45°)) = corner_dev · 2.414
    # Solving corner_deviation = 0.5/2.414 ≈ 0.207 mm:
    corner_dev = 0.5 / (math.sqrt(2)/2 / (1 - math.sqrt(2)/2))
    result = blendmath.blend_from_moves(
        prev_move=prev,
        next_move=nxt,
        corner_deviation=corner_dev,
        toolhead=toolhead,
    )
    assert result is not None
    assert result.R == pytest.approx(0.5, rel=1e-6)
    # Final v_cap ~ 99.8 mm/s per spec sanity section (Y rotation-jerk binds).
    assert result.v_cap == pytest.approx(99.8, rel=0.05)


def test_blend_from_moves_without_toolhead_preserves_old_behavior():
    # Pass j_eff directly, no toolhead: identical to pre-change behavior.
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    j_eff = 1e8
    corner_dev = 0.02
    adapter_result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=corner_dev, j_eff=j_eff,
    )
    core_result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0), next_dir=(0.0, 1.0, 0.0),
        L_prev=50.0, L_next=50.0,
        corner_deviation=corner_dev, a_max=50000.0, j_eff=j_eff,
    )
    assert adapter_result.R == pytest.approx(core_result.R, rel=1e-12)
    assert adapter_result.v_cap == pytest.approx(core_result.v_cap, rel=1e-12)


def test_blend_from_moves_collinear_with_toolhead_returns_none():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=0.02, toolhead=toolhead,
    )
    assert result is None


def test_blend_from_moves_u_turn_with_toolhead_returns_zero_arc():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[-1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=0.02, toolhead=toolhead,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv-test/bin/pytest test/test_blendmath.py -v -k "blend_from_moves_with_toolhead or blend_from_moves_without_toolhead or blend_from_moves_collinear_with_toolhead or blend_from_moves_u_turn_with_toolhead"`
Expected: FAIL — `blend_from_moves` does not yet accept a `toolhead` kwarg.

- [ ] **Step 3: Extend blend_from_moves**

Replace the body of `blend_from_moves` in `klippy/blendmath.py` with the new two-path implementation. Full updated function:

```python
def blend_from_moves(
    prev_move,
    next_move,
    corner_deviation: float,
    j_eff: float = float("inf"),
    toolhead=None,
) -> Optional[BlendArc]:
    """Adapter: compute a blend arc from a pair of Kalico Move-like objects.

    Skips the blend if either move is non-kinematic (E-only). The
    effective a_max is the stricter of the two moves' accel values.

    If `toolhead` is given, derives `j_eff` and an additional per-axis
    entry-step velocity cap from the toolhead's input shaper module.
    In that case any explicit `j_eff` argument is ignored.

    If `toolhead` is None (default), `blend_geometry` is called once
    with the given `j_eff` (default +inf) — preserves the pre-shaper
    behavior used by existing tests.
    """
    if not getattr(prev_move, "is_kinematic_move", True):
        return None
    if not getattr(next_move, "is_kinematic_move", True):
        return None

    prev_dir: Vec3 = (
        prev_move.axes_r[0],
        prev_move.axes_r[1],
        prev_move.axes_r[2],
    )
    next_dir: Vec3 = (
        next_move.axes_r[0],
        next_move.axes_r[1],
        next_move.axes_r[2],
    )
    a_max = min(prev_move.accel, next_move.accel)

    if toolhead is None:
        return blend_geometry(
            prev_dir=prev_dir, next_dir=next_dir,
            L_prev=prev_move.move_d, L_next=next_move.move_d,
            corner_deviation=corner_deviation,
            a_max=a_max, j_eff=j_eff,
        )

    shapers = _extract_shapers(toolhead)

    # First pass: no jerk constraint — we need R, entry_pt, center,
    # plane_normal to compute per-axis bounds.
    arc_0 = blend_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=prev_move.move_d, L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max, j_eff=float("inf"),
    )
    if arc_0 is None or arc_0.R == 0.0 or not shapers:
        return arc_0

    n_hat = vnormalize(vsub(arc_0.center, arc_0.entry_pt))
    bounds = blendshaper.compute_shaper_bounds(
        shapers=shapers,
        R=arc_0.R,
        n_hat=n_hat,
        p_hat=arc_0.plane_normal,
    )

    # Second pass: with the derived j_eff.
    arc = blend_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=prev_move.move_d, L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max, j_eff=bounds.j_eff,
    )
    if arc is None or arc.R == 0.0:
        return arc

    # Re-evaluate Bound (b) against the final R / n_hat (Bound (b) is
    # mildly R-dependent; second evaluation is near-free and keeps the
    # bound honest on corners where the second-pass R differs from R_0).
    n_hat_final = vnormalize(vsub(arc.center, arc.entry_pt))
    bounds_final = blendshaper.compute_shaper_bounds(
        shapers=shapers,
        R=arc.R,
        n_hat=n_hat_final,
        p_hat=arc.plane_normal,
    )
    v_cap = min(arc.v_cap, bounds_final.v_step_cap)
    # BlendArc is frozen; return a copy with the capped v_cap.
    from dataclasses import replace
    return replace(arc, v_cap=v_cap)
```

`vnormalize` already exists in `blendmath.py` from Task 1 of the Blend Geometry Module plan. It raises `ValueError` on zero-magnitude input; in the code path above we guard `if arc_0.R == 0.0: return arc_0` first, so `arc_0.center != arc_0.entry_pt` whenever we reach `vnormalize`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv-test/bin/pytest test/test_blendmath.py test/test_blendshaper.py -v`
Expected: all PASS. The pre-existing `test_blend_from_moves_matches_pure_math` still passes because the no-toolhead path is preserved.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: two-pass blend_from_moves with shaper bounds"
```

---

## Task 9: Numeric sanity regression + monotonicity property tests

**Files:**
- Modify: `test/test_blendshaper.py`

- [ ] **Step 1: Write the regression / property tests**

Append to `test/test_blendshaper.py`:

```python
def _zv_A(f, zeta=0.1):
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs
    sc = ShaperCalibrate(printer=None)
    shaper = shaper_defs.get_zv_shaper(f, zeta)
    return sc.find_shaper_max_accel(shaper, scv=0.0)


def test_numeric_sanity_user_regime_90deg_corner():
    """Matches docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md §Testing point 5.

    Setup: X=ZV@150Hz, Y=ZV@80Hz, ζ=0.1. 90° +X→+Y corner at R=0.5mm.
    Real n̂ at entry for this corner is (0, 1, 0) — pure Y direction
    (centripetal accel appears entirely on Y as the toolhead starts
    turning from +X into +Y). So X's entry-step is not triggered;
    only Y contributes to Bound (b). Bound (c) binds.
    """
    f_x, f_y = 150.0, 80.0
    zeta = 0.1
    A_x = _zv_A(f_x, zeta)
    A_y = _zv_A(f_y, zeta)
    T_x = blendshaper.shaper_span("zv", f_x, zeta)
    T_y = blendshaper.shaper_span("zv", f_y, zeta)

    snaps = [
        blendshaper.AxisShaperSnapshot("x", "zv", f_x, zeta, A_x),
        blendshaper.AxisShaperSnapshot("y", "zv", f_y, zeta, A_y),
    ]
    R = 0.5
    n_hat = (0.0, 1.0, 0.0)
    p_hat = (0.0, 0.0, 1.0)
    bounds = blendshaper.compute_shaper_bounds(snaps, R, n_hat, p_hat)

    # j_eff expected to bind on Y: j_y = A_y / T_y.
    assert bounds.j_eff == pytest.approx(A_y / T_y, rel=1e-9)

    # v_step_cap expected on Y only (X has |n̂·x̂|=0): √(A_y · R).
    expected_v_step = math.sqrt(A_y * R)
    assert bounds.v_step_cap == pytest.approx(expected_v_step, rel=1e-9)

    # End-to-end v_jerk from j_eff and this R.
    v_jerk = (R * R * bounds.j_eff) ** (1.0 / 3.0)
    # Centripetal cap.
    a_max = 50000.0
    v_centripetal = math.sqrt((math.sqrt(3) / 2) * a_max * R)
    # Rotation jerk should bind: v_jerk < others.
    assert v_jerk < v_centripetal
    assert v_jerk < expected_v_step
    # Cross-check: ~99.8 mm/s per the spec sanity section.
    assert v_jerk == pytest.approx(99.8, rel=0.05)


@pytest.mark.parametrize("f", [50.0, 80.0, 100.0, 150.0, 200.0])
def test_j_eff_monotone_in_frequency(f):
    """Higher shaper frequency → higher j_eff. All else equal."""
    zeta = 0.1
    A = 10000.0  # hold A constant so we isolate the T dependence
    T_f = blendshaper.shaper_span("zv", f, zeta)
    j_f = A / T_f
    # A higher frequency gives a shorter T and thus larger j.
    if f < 200.0:
        T_higher = blendshaper.shaper_span("zv", f * 1.5, zeta)
        j_higher = A / T_higher
        assert j_higher > j_f


def test_j_eff_monotone_in_damping():
    """Higher damping ratio → larger t_d → smaller j_eff."""
    f = 100.0
    T_low = blendshaper.shaper_span("zv", f, 0.05)
    T_high = blendshaper.shaper_span("zv", f, 0.2)
    # With A constant:
    A = 10000.0
    assert A / T_high < A / T_low


def test_j_eff_monotone_in_shaper_type():
    """ZV has shortest T → largest j_eff for given f; 3HUMP_EI longest T → smallest."""
    f = 100.0
    zeta = 0.1
    A = 10000.0
    t_zv = blendshaper.shaper_span("zv", f, zeta)
    t_zvd = blendshaper.shaper_span("zvd", f, zeta)
    t_3hump = blendshaper.shaper_span("3hump_ei", f, zeta)
    assert A / t_zv > A / t_zvd > A / t_3hump
```

- [ ] **Step 2: Run the suite**

Run: `.venv-test/bin/pytest test/test_blendshaper.py test/test_blendmath.py -v`
Expected: all PASS. In particular `test_numeric_sanity_user_regime_90deg_corner` should match the spec's illustrative numbers (v_jerk ≈ 99.8 mm/s).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendshaper.py
git commit -m "blendshaper: numeric sanity + monotonicity regression"
```

---

## Final verification checklist

After Task 9 commits, run once more from the repo root:

```bash
.venv-test/bin/pytest test/test_blendshaper.py test/test_blendmath.py -v
```

Everything should pass. Target counts:
- `test/test_blendshaper.py` — ~25 tests (all new).
- `test/test_blendmath.py` — original ~109 tests + ~7 new `_extract_shapers` / two-pass `blend_from_moves` tests.

Manual sanity check — open each modified file and confirm:

- `klippy/blendshaper.py`:
  - Module header comment matches convention with existing Kalico files.
  - No unused imports. Zero Kalico imports in the core functions.
  - All public functions have a one-line docstring.
  - `PROJECTION_EPS` and `_SHAPER_SPAN_FACTOR` are module-level constants.
- `klippy/blendmath.py`:
  - `from klippy import blendshaper` and the ShaperCalibrate lazy import are the only new Kalico-touching imports.
  - `blend_from_moves` preserves pre-existing behavior when `toolhead` is None.
  - `_extract_shapers` handles the three None/missing-module/unshaped-axis cases.

## What is *not* done by this plan

This plan lands the pure-math module and the adapter extension only. The following are next steps, each with their own spec/plan:

- **Planner integration** — wiring `blend_from_moves` (now toolhead-aware) into `toolhead.py` / `LookAheadQueue`, emitting polyline points as `Move` objects through `trapq`.
- **SCV / `junction_deviation` removal** — deletes the now-dead code paths in `toolhead.py`.
- **Shake&Tune / `find_shaper_max_accel` reformulation** — if the SCV-removal sub-spec changes `find_shaper_max_accel`'s signature, update this module's adapter call site.
- **Hardware validation** — running the validation recipe from the design spec against a real printer.
- **Naive-CAM prepass** — independent sub-spec, can land in parallel.
