# Blend Geometry Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the pure-math blend geometry module at `klippy/blendmath.py` that computes G¹ tangent-arc corner blends, exactly as defined in `docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md`.

**Architecture:** Single Python module with three top-level callables (`blend_geometry`, `segment_arc`, `blend_from_moves`) plus one dataclass (`BlendArc`). Core functions have zero Kalico imports and take/return flat tuples (`Vec3 = tuple[float, float, float]`). A thin adapter accepts Kalico `Move`-like duck-typed objects. Tested exhaustively in isolation via pytest; no planner integration in this plan.

**Tech Stack:** Python 3 stdlib only (`math`, `dataclasses`, `typing`). pytest for tests. No new dependencies.

---

## Reference material

Before starting, read these:

- `docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md` — the design spec this plan implements.
- `docs/superpowers/specs/2026-04-16-phase0-research/00-summary.md` — the synthesis that justifies the design.
- `JUNCTION_DEVIATION_ANALYSIS.md` — the original research notes.
- `klippy/toolhead.py:79-122` — the existing SCV-based `calc_junction` this module will eventually replace (not touched in this plan).

## File structure

- **Create `klippy/blendmath.py`** — the module. Vec helpers, `BlendArc` dataclass, `blend_geometry`, `segment_arc`, `blend_from_moves`.
- **Create `test/test_blendmath.py`** — pytest test module. Unit tests, property tests, regression fixtures.

Nothing else is created or modified in this plan. The module is standalone and unwired.

## Conventions used throughout the code

- `Vec3` is a type alias `tuple[float, float, float]`. All vector data is flat tuples; no custom class.
- Deflection angle `θ`: 0 = collinear, π = U-turn.
- `prev_dir`, `next_dir`: head-to-tail unit direction vectors matching `Move.axes_r[:3]` semantics.
- Degenerate thresholds: `COLLINEAR_EPS = 1e-6` on `sin(θ/2)`; `REVERSAL_EPS = 1e-6` on `cos(θ/2)`.

---

## Task 1: Module skeleton with Vec3 helpers

**Files:**
- Create: `klippy/blendmath.py`
- Test: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

```python
# test/test_blendmath.py
import math

import pytest

from klippy import blendmath


def test_vec_dot():
    assert blendmath.vdot((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == 0.0
    assert blendmath.vdot((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == 32.0


def test_vec_cross():
    assert blendmath.vcross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == (0.0, 0.0, 1.0)
    assert blendmath.vcross((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)) == (0.0, 0.0, -1.0)


def test_vec_norm():
    assert blendmath.vnorm((3.0, 4.0, 0.0)) == 5.0
    assert blendmath.vnorm((0.0, 0.0, 0.0)) == 0.0


def test_vec_scale():
    assert blendmath.vscale((1.0, 2.0, 3.0), 2.0) == (2.0, 4.0, 6.0)


def test_vec_add_sub():
    assert blendmath.vadd((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == (5.0, 7.0, 9.0)
    assert blendmath.vsub((4.0, 5.0, 6.0), (1.0, 2.0, 3.0)) == (3.0, 3.0, 3.0)


def test_vec_normalize():
    n = blendmath.vnormalize((3.0, 4.0, 0.0))
    assert n == pytest.approx((0.6, 0.8, 0.0))

    with pytest.raises(ValueError):
        blendmath.vnormalize((0.0, 0.0, 0.0))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py -v`
Expected: all FAIL with "ModuleNotFoundError: No module named 'klippy.blendmath'".

- [ ] **Step 3: Create the module with Vec3 helpers**

```python
# klippy/blendmath.py
# Corner-blending geometry module.
#
# Pure-math primitives: given two adjacent linear moves and a
# chord-tolerance parameter, returns a G1 tangent circular arc that
# smooths the corner, along with the maximum velocity it may be
# traversed at and a fine-segmented polyline approximation.
#
# See docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6


def vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a: Vec3) -> float:
    return math.sqrt(vdot(a, a))


def vscale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vnormalize(a: Vec3) -> Vec3:
    n = vnorm(a)
    if n == 0.0:
        raise ValueError("cannot normalize zero vector")
    return vscale(a, 1.0 / n)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: add Vec3 helper primitives"
```

---

## Task 2: BlendArc dataclass

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_arc_dataclass_fields():
    arc = blendmath.BlendArc(
        R=5.0,
        theta=math.pi / 2,
        d_consumed=5.0,
        v_cap=100.0,
        center=(0.0, 5.0, 0.0),
        entry_pt=(-5.0, 0.0, 0.0),
        exit_pt=(0.0, 5.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    assert arc.R == 5.0
    assert arc.theta == math.pi / 2
    assert arc.d_consumed == 5.0
    assert arc.v_cap == 100.0
    assert arc.center == (0.0, 5.0, 0.0)
    assert arc.entry_pt == (-5.0, 0.0, 0.0)
    assert arc.exit_pt == (0.0, 5.0, 0.0)
    assert arc.entry_tangent == (1.0, 0.0, 0.0)
    assert arc.exit_tangent == (0.0, 1.0, 0.0)
    assert arc.plane_normal == (0.0, 0.0, 1.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest test/test_blendmath.py::test_blend_arc_dataclass_fields -v`
Expected: FAIL with "AttributeError: module 'klippy.blendmath' has no attribute 'BlendArc'".

- [ ] **Step 3: Add the dataclass to the module**

Append to `klippy/blendmath.py` (after the Vec helpers):

```python
@dataclass(frozen=True)
class BlendArc:
    R: float
    theta: float
    d_consumed: float
    v_cap: float
    center: Vec3
    entry_pt: Vec3
    exit_pt: Vec3
    entry_tangent: Vec3
    exit_tangent: Vec3
    plane_normal: Vec3
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest test/test_blendmath.py::test_blend_arc_dataclass_fields -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: add BlendArc dataclass"
```

---

## Task 3: blend_geometry — collinear returns None

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_collinear_returns_none():
    # Same direction → deflection = 0 → no blend needed
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (1.0, 0.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is None


def test_blend_geometry_near_collinear_returns_none():
    # Tiny deflection below threshold → also None
    prev_dir = (1.0, 0.0, 0.0)
    # 1e-8 rad deflection
    eps = 1e-8
    next_dir = (math.cos(eps), math.sin(eps), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py::test_blend_geometry_collinear_returns_none test/test_blendmath.py::test_blend_geometry_near_collinear_returns_none -v`
Expected: FAIL with "AttributeError: module 'klippy.blendmath' has no attribute 'blend_geometry'".

- [ ] **Step 3: Add minimal `blend_geometry` handling collinear case**

Append to `klippy/blendmath.py`:

```python
def blend_geometry(
    prev_dir: Vec3,
    next_dir: Vec3,
    L_prev: float,
    L_next: float,
    corner_deviation: float,
    a_max: float,
    j_eff: float,
) -> Optional[BlendArc]:
    # Deflection angle theta: 0 = collinear, pi = U-turn.
    # With head-to-tail unit directions:
    #   cos(theta) = -(prev_dir . next_dir)
    #   cos(theta/2) = sqrt((1 + prev_dir.next_dir) / 2)
    #   sin(theta/2) = sqrt((1 - prev_dir.next_dir) / 2)
    dp = vdot(prev_dir, next_dir)
    # Clamp for numerical safety; dp should lie in [-1, 1] for unit vectors.
    dp = max(-1.0, min(1.0, dp))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))

    if sin_half < COLLINEAR_EPS:
        # Collinear: no blend required.
        return None

    raise NotImplementedError("only collinear branch handled so far")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all three tests PASS (vec helpers, dataclass, and the two collinear tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: handle collinear corner (no blend)"
```

---

## Task 4: blend_geometry — U-turn returns zero-radius stop arc

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_u_turn_returns_zero_arc():
    # Anti-parallel directions: theta = pi.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_blend_geometry_near_u_turn_returns_zero_arc():
    prev_dir = (1.0, 0.0, 0.0)
    # 1e-8 rad shy of U-turn
    eps = 1e-8
    next_dir = (-math.cos(eps), math.sin(eps), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py -v`
Expected: the two new tests FAIL with `NotImplementedError`.

- [ ] **Step 3: Add U-turn branch**

Edit `klippy/blendmath.py` — replace the `raise NotImplementedError` line in `blend_geometry` with:

```python
    if cos_half < REVERSAL_EPS:
        # U-turn: no tangent arc exists. Caller must stop at the junction.
        return BlendArc(
            R=0.0,
            theta=math.pi,
            d_consumed=0.0,
            v_cap=0.0,
            center=(0.0, 0.0, 0.0),
            entry_pt=(0.0, 0.0, 0.0),
            exit_pt=(0.0, 0.0, 0.0),
            entry_tangent=prev_dir,
            exit_tangent=next_dir,
            plane_normal=(0.0, 0.0, 0.0),
        )

    raise NotImplementedError("general corner not handled yet")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: handle U-turn corner (zero radius stop)"
```

---

## Task 5: blend_geometry — tolerance-driven radius

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_90deg_tolerance_radius():
    # 90 degree corner, X -> Y.
    # theta = pi/2, so cos(theta/2) = sqrt(2)/2.
    # R_tol = corner_deviation * (sqrt(2)/2) / (1 - sqrt(2)/2)
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02  # mm
    # Adjacent segments much longer than the arc, jerk and accel loose:
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,      # trivial acceleration → v_cap not the limiting factor here
        j_eff=1e30,     # jerk floor effectively disabled
    )
    assert result is not None
    expected_R = corner_dev * (math.sqrt(2) / 2) / (1 - math.sqrt(2) / 2)
    assert result.R == pytest.approx(expected_R, rel=1e-9)
    assert result.theta == pytest.approx(math.pi / 2, rel=1e-9)


def test_blend_geometry_60deg_tolerance_radius():
    # 60 degree deflection: prev along +X, next rotated 60 degrees counter-clockwise.
    prev_dir = (1.0, 0.0, 0.0)
    theta = math.pi / 3
    next_dir = (math.cos(theta), math.sin(theta), 0.0)
    corner_dev = 0.05
    cos_half = math.cos(theta / 2)
    expected_R = corner_dev * cos_half / (1 - cos_half)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    assert result.R == pytest.approx(expected_R, rel=1e-9)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py -v`
Expected: the two new tests FAIL with `NotImplementedError`.

- [ ] **Step 3: Add general arc geometry — tolerance radius path only**

Edit `klippy/blendmath.py` — replace the `raise NotImplementedError("general corner not handled yet")` line in `blend_geometry` with:

```python
    # Deflection angle (rad).
    theta = 2.0 * math.atan2(sin_half, cos_half)

    # Tolerance-driven radius:
    R_tol = corner_deviation * cos_half / (1.0 - cos_half)

    # Midpoint / adjacent-segment cap. cot(theta/2) = cos_half / sin_half.
    R_mid = min(L_prev, L_next) * cos_half / sin_half

    R = min(R_tol, R_mid)

    # Placeholder v_cap; refined in later tasks.
    v_cap = float("inf")

    return BlendArc(
        R=R,
        theta=theta,
        d_consumed=R * sin_half / cos_half,
        v_cap=v_cap,
        center=(0.0, 0.0, 0.0),
        entry_pt=(0.0, 0.0, 0.0),
        exit_pt=(0.0, 0.0, 0.0),
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=(0.0, 0.0, 0.0),
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: compute tolerance- and midpoint-capped radius"
```

---

## Task 6: blend_geometry — midpoint cap dominates on short segments

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_midpoint_cap_binds_on_short_segment():
    # 90 deg corner, but one adjacent segment is short.
    # R_mid = min(L_prev, L_next) * cot(theta/2) = 0.5 * 1.0 = 0.5 mm
    # R_tol should be much larger given the tolerance; verify R_mid wins.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 5.0  # absurdly loose tolerance so R_tol is the larger value
    L_short = 0.5
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=L_short,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    cos_half = math.sqrt(2) / 2
    sin_half = math.sqrt(2) / 2
    expected_R_mid = L_short * cos_half / sin_half  # = 0.5
    assert result.R == pytest.approx(expected_R_mid, rel=1e-9)
    # d_consumed should equal L_short (90 deg case: d = R).
    assert result.d_consumed == pytest.approx(L_short, rel=1e-9)
```

- [ ] **Step 2: Run test to verify it passes (sanity)**

Run: `pytest test/test_blendmath.py::test_blend_geometry_midpoint_cap_binds_on_short_segment -v`
Expected: PASS (we already implemented the min in Task 5; this test just pins the behavior).

- [ ] **Step 3: Commit**

```bash
git add test/test_blendmath.py
git commit -m "blendmath: pin midpoint cap behavior"
```

---

## Task 7: blend_geometry — arc center, entry/exit points, plane normal

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_90deg_geometry_positioning():
    # Corner at origin: prev move ends at (0,0,0) heading +X,
    # next move starts at (0,0,0) heading +Y.
    # In this pure-geometry API we don't pass the vertex; entry/exit are
    # expressed in a local frame relative to the corner vertex. Convention:
    # corner vertex is the origin.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    R = result.R
    d = result.d_consumed  # should equal R for 90 deg
    # Entry point sits distance d back along prev_dir from the vertex (origin).
    # prev_dir is the direction the toolhead WAS heading, so the entry point
    # lies at origin - d*prev_dir (upstream of the vertex along the incoming ray).
    expected_entry = (-d, 0.0, 0.0)
    expected_exit = (0.0, d, 0.0)
    # Center sits on the angle bisector interior to the corner, distance
    # R from each tangent point. For this 90 deg +X -> +Y corner it's at
    # (-d, d, 0) i.e. (-R, R, 0) in the corner frame.
    expected_center = (-R, R, 0.0)
    # Plane normal: prev_dir x next_dir = (1,0,0) x (0,1,0) = (0,0,1).
    expected_normal = (0.0, 0.0, 1.0)
    assert result.entry_pt == pytest.approx(expected_entry, abs=1e-12)
    assert result.exit_pt == pytest.approx(expected_exit, abs=1e-12)
    assert result.center == pytest.approx(expected_center, abs=1e-12)
    assert result.plane_normal == pytest.approx(expected_normal, abs=1e-12)
    assert result.entry_tangent == prev_dir
    assert result.exit_tangent == next_dir
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest test/test_blendmath.py::test_blend_geometry_90deg_geometry_positioning -v`
Expected: FAIL on entry_pt/exit_pt/center/plane_normal equality (placeholders from Task 5 are all zeros).

- [ ] **Step 3: Fill out arc frame in the module**

Replace the final `return BlendArc(...)` block in `blend_geometry` with a version that computes center, entry/exit points, and the plane normal. Working against a corner whose vertex is the origin (callers translate as needed):

```python
    # Tangent points on the adjacent rays. prev_dir points *toward* the
    # vertex, so the entry tangent point sits at -d * prev_dir (upstream).
    # next_dir points *away from* the vertex, so exit sits at +d * next_dir.
    d = R * sin_half / cos_half
    entry_pt = vscale(prev_dir, -d)
    exit_pt = vscale(next_dir, d)

    # Plane normal (ambiguous sign for collinear / reversal; safe here since
    # those cases already returned). Choose prev x next for consistent
    # right-handed orientation.
    raw_normal = vcross(prev_dir, next_dir)
    raw_norm_n = vnorm(raw_normal)
    if raw_norm_n == 0.0:
        plane_normal: Vec3 = (0.0, 0.0, 0.0)
    else:
        plane_normal = vscale(raw_normal, 1.0 / raw_norm_n)

    # Arc center: perpendicular to prev_dir at entry_pt, offset by R toward
    # the interior of the corner. The interior direction is
    # normalize(next_dir - prev_dir * cos_theta) -- but it's simpler to
    # compute via the inward perpendicular n_prev = plane_normal x prev_dir
    # (with the sign chosen so that stepping from entry_pt by +R*n_prev
    # lands on the arc center).
    n_prev = vcross(plane_normal, prev_dir)
    # Choose sign so n_prev points from entry_pt toward the corner interior.
    # The interior is on the next_dir side; dot with next_dir should be >= 0.
    if vdot(n_prev, next_dir) < 0.0:
        n_prev = vscale(n_prev, -1.0)
    center = vadd(entry_pt, vscale(n_prev, R))

    return BlendArc(
        R=R,
        theta=theta,
        d_consumed=d,
        v_cap=v_cap,
        center=center,
        entry_pt=entry_pt,
        exit_pt=exit_pt,
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=plane_normal,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: compute arc center, tangent points, plane normal"
```

---

## Task 8: blend_geometry — centripetal velocity cap

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_centripetal_cap():
    # 90 deg corner with tight accel budget; jerk floor effectively disabled.
    # v_cap_centripetal = sqrt((sqrt(3)/2) * a_max * R)
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    a_max = 50000.0
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=1e30,
    )
    assert result is not None
    expected_v = math.sqrt((math.sqrt(3) / 2) * a_max * result.R)
    assert result.v_cap == pytest.approx(expected_v, rel=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest test/test_blendmath.py::test_blend_geometry_centripetal_cap -v`
Expected: FAIL — `v_cap` is still `inf`.

- [ ] **Step 3: Add centripetal cap in the module**

Edit `klippy/blendmath.py` — replace the `v_cap = float("inf")` line in `blend_geometry` with:

```python
    # Velocity caps. LinuxCNC's Pythagorean split:
    #   a_t <= 0.5 * a_max (tangential)
    #   a_n <= (sqrt(3)/2) * a_max (normal)
    # yielding a_t^2 + a_n^2 <= a_max^2 (total vector budget).
    a_n_max = (math.sqrt(3.0) / 2.0) * a_max
    v_centripetal = math.sqrt(a_n_max * R) if R > 0.0 else 0.0

    v_cap = v_centripetal
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: apply centripetal velocity cap"
```

---

## Task 9: blend_geometry — jerk floor velocity cap

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_blend_geometry_jerk_floor_dominates():
    # Tight jerk budget: v_cap should drop to (R * sqrt(j))^(2/3).
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    a_max = 50000.0
    j_eff = 1e4  # very tight jerk → jerk cap should dominate
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=j_eff,
    )
    assert result is not None
    expected_v_jerk = (result.R * math.sqrt(j_eff)) ** (2.0 / 3.0)
    expected_v_centripetal = math.sqrt((math.sqrt(3) / 2) * a_max * result.R)
    # Jerk cap should win.
    assert expected_v_jerk < expected_v_centripetal
    assert result.v_cap == pytest.approx(expected_v_jerk, rel=1e-9)


def test_blend_geometry_jerk_floor_loose_does_not_bind():
    # Very loose jerk: centripetal should still dominate.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e30,
    )
    assert result is not None
    expected_v_centripetal = math.sqrt((math.sqrt(3) / 2) * 50000.0 * result.R)
    assert result.v_cap == pytest.approx(expected_v_centripetal, rel=1e-9)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py -v`
Expected: the jerk-dominated test FAILS; loose test PASSES.

- [ ] **Step 3: Add jerk floor cap in the module**

Edit `klippy/blendmath.py` — replace the two `v_cap = v_centripetal` / jerk block with:

```python
    a_n_max = (math.sqrt(3.0) / 2.0) * a_max
    v_centripetal = math.sqrt(a_n_max * R) if R > 0.0 else 0.0

    # Jerk floor: R >= v^(3/2) / sqrt(j_eff)  =>  v <= (R * sqrt(j_eff))^(2/3)
    if R > 0.0 and j_eff > 0.0:
        v_jerk = (R * math.sqrt(j_eff)) ** (2.0 / 3.0)
    else:
        v_jerk = 0.0

    v_cap = min(v_centripetal, v_jerk)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: apply jerk-floor velocity cap"
```

---

## Task 10: blend_geometry — property tests across random corners

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the property tests**

Append to `test/test_blendmath.py`:

```python
import random


def _rand_unit_vec(rng: random.Random) -> blendmath.Vec3:
    # Uniform direction on the XY plane is enough for property tests.
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return (math.cos(phi), math.sin(phi), 0.0)


@pytest.mark.parametrize("seed", range(50))
def test_blend_geometry_property_random_corners(seed):
    rng = random.Random(seed)
    # Random first direction.
    prev_dir = _rand_unit_vec(rng)
    # Random deflection in (0.01 rad, pi - 0.01 rad) to stay away from degenerates.
    theta = rng.uniform(0.01, math.pi - 0.01)
    # Rotate prev_dir by theta about +Z to get next_dir.
    c, s = math.cos(theta), math.sin(theta)
    next_dir = (
        c * prev_dir[0] - s * prev_dir[1],
        s * prev_dir[0] + c * prev_dir[1],
        0.0,
    )
    L_prev = rng.uniform(0.5, 100.0)
    L_next = rng.uniform(0.5, 100.0)
    corner_dev = rng.uniform(0.001, 0.1)
    a_max = rng.uniform(1000.0, 100000.0)
    j_eff = rng.uniform(1e5, 1e9)

    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=L_prev,
        L_next=L_next,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=j_eff,
    )
    assert result is not None
    R = result.R
    d = result.d_consumed

    # 1. Consumed length fits inside both segments.
    assert d <= L_prev + 1e-9
    assert d <= L_next + 1e-9

    # 2. Chord deviation of this single arc: epsilon = R*(1/cos(theta/2) - 1).
    # Must not exceed corner_deviation (unless midpoint cap made R smaller,
    # in which case epsilon <= corner_deviation trivially).
    cos_half = math.cos(theta / 2)
    eps_arc = R * (1.0 / cos_half - 1.0)
    assert eps_arc <= corner_dev + 1e-9

    # 3. v_cap respects centripetal bound.
    a_n_max = (math.sqrt(3) / 2) * a_max
    assert result.v_cap ** 2 <= a_n_max * R + 1e-6

    # 4. v_cap respects jerk floor.
    #    v^(3/2) <= R * sqrt(j_eff)
    assert result.v_cap ** 1.5 <= R * math.sqrt(j_eff) + 1e-6

    # 5. Tangent points lie on the adjacent rays.
    #    entry_pt should be collinear with prev_dir (at -d * prev_dir).
    assert result.entry_pt == pytest.approx(
        (-d * prev_dir[0], -d * prev_dir[1], -d * prev_dir[2]), abs=1e-9
    )
    assert result.exit_pt == pytest.approx(
        (d * next_dir[0], d * next_dir[1], d * next_dir[2]), abs=1e-9
    )

    # 6. Center is distance R from both tangent points.
    from_entry = blendmath.vsub(result.center, result.entry_pt)
    from_exit = blendmath.vsub(result.center, result.exit_pt)
    assert blendmath.vnorm(from_entry) == pytest.approx(R, rel=1e-6)
    assert blendmath.vnorm(from_exit) == pytest.approx(R, rel=1e-6)

    # 7. Center lies on the interior side of the corner (dot with next_dir > 0 from entry_pt).
    interior_check = blendmath.vdot(blendmath.vsub(result.center, result.entry_pt), next_dir)
    assert interior_check > -1e-9
```

- [ ] **Step 2: Run the property test**

Run: `pytest test/test_blendmath.py::test_blend_geometry_property_random_corners -v`
Expected: all 50 parameterized cases PASS. If any fail, investigate the specific seed.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendmath.py
git commit -m "blendmath: property tests over 50 random corners"
```

---

## Task 11: segment_arc — basic polyline emission

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_segment_arc_90deg_basic():
    # Build a 90 deg arc with R=10, max_chord_err=0.01.
    # Delta phi per segment: 2*acos(1 - 0.01/10) = 2*acos(0.999) rad ~= 0.0894 rad.
    # Total arc angle (theta) = pi/2 rad. Expected segments ~= (pi/2)/0.0894 ~= 17.56, so 18.
    arc = blendmath.BlendArc(
        R=10.0,
        theta=math.pi / 2,
        d_consumed=10.0,
        v_cap=100.0,
        center=(-10.0, 10.0, 0.0),
        entry_pt=(-10.0, 0.0, 0.0),
        exit_pt=(0.0, 10.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    polyline = blendmath.segment_arc(arc, max_chord_err=0.01)

    # First and last points are entry and exit.
    assert polyline[0] == pytest.approx(arc.entry_pt, abs=1e-9)
    assert polyline[-1] == pytest.approx(arc.exit_pt, abs=1e-9)

    # Every point lies on the arc (distance R from center).
    for pt in polyline:
        d = blendmath.vnorm(blendmath.vsub(pt, arc.center))
        assert d == pytest.approx(arc.R, rel=1e-9)

    # Reasonable point count (theta / delta_phi + 1).
    delta_phi_max = 2.0 * math.acos(1.0 - 0.01 / 10.0)
    expected_segments = math.ceil(arc.theta / delta_phi_max)
    assert len(polyline) == expected_segments + 1


def test_segment_arc_zero_radius_returns_degenerate_polyline():
    # R=0 (U-turn case): polyline is just [entry_pt, exit_pt] (both equal).
    arc = blendmath.BlendArc(
        R=0.0,
        theta=math.pi,
        d_consumed=0.0,
        v_cap=0.0,
        center=(0.0, 0.0, 0.0),
        entry_pt=(0.0, 0.0, 0.0),
        exit_pt=(0.0, 0.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(-1.0, 0.0, 0.0),
        plane_normal=(0.0, 0.0, 0.0),
    )
    polyline = blendmath.segment_arc(arc, max_chord_err=0.01)
    assert polyline == [(0.0, 0.0, 0.0)]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py::test_segment_arc_90deg_basic test/test_blendmath.py::test_segment_arc_zero_radius_returns_degenerate_polyline -v`
Expected: FAIL with "AttributeError: module 'klippy.blendmath' has no attribute 'segment_arc'".

- [ ] **Step 3: Implement `segment_arc`**

Append to `klippy/blendmath.py`:

```python
def segment_arc(arc: BlendArc, max_chord_err: float = 1e-2) -> list:
    """Return a polyline approximating the arc, with chord error <= max_chord_err."""
    if arc.R <= 0.0:
        # Degenerate: single point at the (coincident) entry.
        return [arc.entry_pt]

    # Step angle such that chord deviation from the arc is <= max_chord_err.
    # chord error e = R * (1 - cos(dphi/2))  =>  dphi = 2 * acos(1 - e/R).
    e_over_r = max_chord_err / arc.R
    if e_over_r >= 1.0:
        # Absurd tolerance: one segment is enough.
        return [arc.entry_pt, arc.exit_pt]
    dphi_max = 2.0 * math.acos(1.0 - e_over_r)

    num_segments = max(1, math.ceil(arc.theta / dphi_max))
    dphi = arc.theta / num_segments

    # Direction of rotation: from (entry_pt - center) toward (exit_pt - center).
    # Rodrigues' rotation around arc.plane_normal by angle phi, applied to
    # the radial vector from center.
    r0 = vsub(arc.entry_pt, arc.center)
    axis = arc.plane_normal

    points: list = [arc.entry_pt]
    for i in range(1, num_segments):
        phi = dphi * i
        r = _rotate(r0, axis, phi)
        points.append(vadd(arc.center, r))
    points.append(arc.exit_pt)
    return points


def _rotate(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rotate vector v around unit axis by angle (radians). Rodrigues."""
    c = math.cos(angle)
    s = math.sin(angle)
    ax_dot_v = vdot(axis, v)
    ax_cross_v = vcross(axis, v)
    return (
        v[0] * c + ax_cross_v[0] * s + axis[0] * ax_dot_v * (1.0 - c),
        v[1] * c + ax_cross_v[1] * s + axis[1] * ax_dot_v * (1.0 - c),
        v[2] * c + ax_cross_v[2] * s + axis[2] * ax_dot_v * (1.0 - c),
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: add segment_arc polyline emitter"
```

---

## Task 12: segment_arc — chord error property test

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the property test**

Append to `test/test_blendmath.py`:

```python
@pytest.mark.parametrize("seed", range(30))
def test_segment_arc_chord_error_bound(seed):
    rng = random.Random(seed + 10_000)
    # Build a valid arc from blend_geometry on a random corner.
    prev_dir = _rand_unit_vec(rng)
    theta = rng.uniform(0.05, math.pi - 0.05)
    c, s = math.cos(theta), math.sin(theta)
    next_dir = (
        c * prev_dir[0] - s * prev_dir[1],
        s * prev_dir[0] + c * prev_dir[1],
        0.0,
    )
    arc = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=100.0,
        L_next=100.0,
        corner_deviation=rng.uniform(0.01, 0.2),
        a_max=50000.0,
        j_eff=1e8,
    )
    assert arc is not None

    max_chord_err = rng.uniform(0.0005, 0.05)
    polyline = blendmath.segment_arc(arc, max_chord_err=max_chord_err)

    # Each consecutive pair: midpoint's deviation from the arc should be
    # <= max_chord_err (with a small numeric slack).
    for p0, p1 in zip(polyline, polyline[1:]):
        midpoint = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2)
        # Deviation = R - |midpoint - center| (on arc side, midpoint is inside).
        dist_from_center = blendmath.vnorm(blendmath.vsub(midpoint, arc.center))
        chord_err = arc.R - dist_from_center
        # chord_err should be in [0, max_chord_err + small slack]
        assert chord_err >= -1e-9
        assert chord_err <= max_chord_err + 1e-6
```

- [ ] **Step 2: Run property test**

Run: `pytest test/test_blendmath.py::test_segment_arc_chord_error_bound -v`
Expected: all 30 cases PASS.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendmath.py
git commit -m "blendmath: property test for segment_arc chord error"
```

---

## Task 13: blend_from_moves adapter — 3D kinematic path

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
class _FakeMove:
    """Minimal duck-typed stand-in for Kalico's Move class."""

    def __init__(self, axes_r, move_d, accel, max_cruise_v2, is_kinematic_move=True):
        # Kalico's Move.axes_r is a 4-vector [x, y, z, e]; only [:3] is used here.
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
        self.max_cruise_v2 = max_cruise_v2
        self.is_kinematic_move = is_kinematic_move


def test_blend_from_moves_matches_pure_math():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0],
        move_d=50.0,
        accel=50000.0,
        max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0],
        move_d=50.0,
        accel=50000.0,
        max_cruise_v2=1e6,
    )
    corner_dev = 0.02
    j_eff = 1e8

    adapter_result = blendmath.blend_from_moves(
        prev_move=prev,
        next_move=nxt,
        corner_deviation=corner_dev,
        j_eff=j_eff,
    )
    core_result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(0.0, 1.0, 0.0),
        L_prev=50.0,
        L_next=50.0,
        corner_deviation=corner_dev,
        a_max=50000.0,  # min(prev.accel, nxt.accel)
        j_eff=j_eff,
    )
    assert adapter_result is not None
    assert core_result is not None
    assert adapter_result.R == pytest.approx(core_result.R, rel=1e-12)
    assert adapter_result.v_cap == pytest.approx(core_result.v_cap, rel=1e-12)


def test_blend_from_moves_non_kinematic_returns_none():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=1.0, accel=1.0, max_cruise_v2=1.0
    )
    nxt = _FakeMove(
        axes_r=[0.0, 0.0, 0.0, 1.0],
        move_d=1.0,
        accel=1.0,
        max_cruise_v2=1.0,
        is_kinematic_move=False,
    )
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt, corner_deviation=0.02, j_eff=1e8
    )
    assert result is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest test/test_blendmath.py::test_blend_from_moves_matches_pure_math test/test_blendmath.py::test_blend_from_moves_non_kinematic_returns_none -v`
Expected: FAIL with "AttributeError: module 'klippy.blendmath' has no attribute 'blend_from_moves'".

- [ ] **Step 3: Implement the adapter**

Append to `klippy/blendmath.py`:

```python
def blend_from_moves(
    prev_move,
    next_move,
    corner_deviation: float,
    j_eff: float,
) -> Optional[BlendArc]:
    """Adapter: compute a blend arc from a pair of Kalico Move-like objects.

    Skips the blend if either move is non-kinematic (E-only). The effective
    a_max is the stricter of the two moves' accel values.
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
    return blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=prev_move.move_d,
        L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max,
        j_eff=j_eff,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: add blend_from_moves Move adapter"
```

---

## Task 14: blend_from_moves — E-axis interpolation helper

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendmath.py`:

```python
def test_interpolate_extruder_through_arc():
    # Setup: a blend arc polyline, plus E-axis consumption rates per mm
    # for prev and next moves. The adapter helper should produce a list of
    # (x, y, z, e) points whose E increases monotonically from 0 to the
    # total E consumption across the blend arc length.
    polyline = [
        (-1.0, 0.0, 0.0),
        (-0.9, 0.1, 0.0),
        (-0.5, 0.5, 0.0),
        (-0.1, 0.9, 0.0),
        (0.0, 1.0, 0.0),
    ]
    # Suppose e_per_mm_prev = 0.05, e_per_mm_next = 0.04, and the arc
    # consumes d=1.0 from each side.
    e_per_mm_prev = 0.05
    e_per_mm_next = 0.04
    d_consumed = 1.0

    points_xyze = blendmath.interpolate_extruder(
        polyline,
        d_consumed=d_consumed,
        e_per_mm_prev=e_per_mm_prev,
        e_per_mm_next=e_per_mm_next,
    )

    # First point has E=0 (start of the blend).
    assert points_xyze[0][3] == pytest.approx(0.0, abs=1e-12)
    # Last point has total E = d_consumed * (prev_rate + next_rate) consumed over
    # the two halves of the blend. The blend replaces the final d_consumed mm of
    # the prev move (consuming d_consumed * e_per_mm_prev) plus the first
    # d_consumed mm of the next move (consuming d_consumed * e_per_mm_next).
    expected_total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)
    assert points_xyze[-1][3] == pytest.approx(expected_total_e, rel=1e-9)
    # Monotonic non-decreasing.
    for p0, p1 in zip(points_xyze, points_xyze[1:]):
        assert p1[3] >= p0[3] - 1e-12
    # Length of output matches polyline.
    assert len(points_xyze) == len(polyline)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest test/test_blendmath.py::test_interpolate_extruder_through_arc -v`
Expected: FAIL with "AttributeError: module 'klippy.blendmath' has no attribute 'interpolate_extruder'".

- [ ] **Step 3: Implement `interpolate_extruder`**

Append to `klippy/blendmath.py`:

```python
def interpolate_extruder(
    polyline,
    d_consumed: float,
    e_per_mm_prev: float,
    e_per_mm_next: float,
) -> list:
    """Attach an E coordinate to each polyline point.

    The blend arc replaces the final `d_consumed` mm of the previous move and
    the first `d_consumed` mm of the next move. Total E through the arc is
    conserved: sum across the polyline equals
    `d_consumed * (e_per_mm_prev + e_per_mm_next)`. E is distributed uniformly
    over the polyline's arc-length parameterization.
    """
    if not polyline:
        return []

    total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)

    # Arc length along the polyline (piecewise-linear approximation).
    seg_lens = []
    total_len = 0.0
    for p0, p1 in zip(polyline, polyline[1:]):
        seg_len = vnorm(vsub(p1, p0))
        seg_lens.append(seg_len)
        total_len += seg_len

    if total_len == 0.0:
        # Degenerate polyline (single point or collapsed).
        return [(p[0], p[1], p[2], 0.0) for p in polyline]

    out = [(polyline[0][0], polyline[0][1], polyline[0][2], 0.0)]
    e = 0.0
    for seg_len, p1 in zip(seg_lens, polyline[1:]):
        e += total_e * seg_len / total_len
        out.append((p1[0], p1[1], p1[2], e))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: interpolate extruder axis through blend arc"
```

---

## Task 15: Regression fixtures for all degenerate cases

**Files:**
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Add explicit regression tests**

Append to `test/test_blendmath.py`:

```python
def test_regression_exact_collinear():
    # Exactly parallel directions.
    assert blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(1.0, 0.0, 0.0),
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    ) is None


def test_regression_exact_u_turn():
    result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(-1.0, 0.0, 0.0),
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_regression_collinear_threshold_boundary():
    # sin_half = 1e-7 < COLLINEAR_EPS (1e-6), so should be treated as collinear.
    prev_dir = (1.0, 0.0, 0.0)
    # angle = 2 * asin(1e-7) rad
    angle = 2.0 * math.asin(1e-7)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    assert blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    ) is None


def test_regression_reversal_threshold_boundary():
    # cos_half = 1e-7 < REVERSAL_EPS (1e-6), so should be treated as U-turn.
    prev_dir = (1.0, 0.0, 0.0)
    # deflection of pi - 2e-7 rad
    angle = math.pi - 2.0 * math.asin(1e-7)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_regression_very_short_segment_produces_tiny_arc():
    # Segment shorter than the tolerance-driven arc would want.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=0.01,  # 10 microns
        L_next=1000.0,
        corner_deviation=0.5,
        a_max=50000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == pytest.approx(0.01, rel=1e-9)  # 90 deg: R = L
    assert result.v_cap > 0.0
```

- [ ] **Step 2: Run regression tests**

Run: `pytest test/test_blendmath.py -v`
Expected: all PASS.

- [ ] **Step 3: Final full-suite run and commit**

Run the full blendmath test module one more time to confirm everything is green:

Run: `pytest test/test_blendmath.py -v`
Expected: 100% PASS.

```bash
git add test/test_blendmath.py
git commit -m "blendmath: regression fixtures for degenerate corners"
```

---

## Final verification checklist

After Task 15 commits, run once more from the repo root:

```bash
pytest test/test_blendmath.py -v
```

Everything should pass. The module is self-contained; no other tests should be affected.

Manual sanity check: open `klippy/blendmath.py` and confirm:

- Module header comment matches convention with existing Kalico files.
- No unused imports.
- All public functions have a one-line docstring.
- `Vec3` type alias is declared and used.
- `COLLINEAR_EPS` and `REVERSAL_EPS` are module-level constants.

## What is *not* done by this plan

This plan deliberately stops at "pure math module passes tests". The following are next steps, each their own spec/plan:

- **`j_eff` derivation sub-spec** — currently the module accepts `j_eff` as a bare input; production wiring needs the derived value from shaper properties.
- **Naive-CAM prepass sub-spec** — collapses short collinear slicer segments before feeding them into this module.
- **Planner integration sub-spec** — wires `blend_from_moves` + `segment_arc` + `interpolate_extruder` into `toolhead.py` / `LookAheadQueue`, emits polyline points as `Move` objects through `trapq`.
- **SCV / `junction_deviation` removal sub-spec** — deletes the now-dead code paths in `toolhead.py`.
- **Shake&Tune / `find_shaper_max_accel` sub-spec** — reformulates `offset_90` against the new kinematic model.
- **Cross-stage decisions sub-spec** — final user-facing parameter name, docs, example configs.
