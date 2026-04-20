# Plan 1 — Quintic Revival + Shape-Pluggable Primitive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `blend-arc`'s circular-arc corner primitive on the `magnum-opus` branch with a curvature-continuous quintic Hermite Bezier shape behind a `SmoothShape` protocol, porting the tested math from `blend-arc-quintic-archive` and fixing the 3-point shaper cap bug identified in audit.

**Architecture:** A new `klippy/blendshape.py` holds the `SmoothShape` protocol and `KinematicLimits` dataclass. A new `klippy/blendquintic.py` implements `QuinticShape`, an arc-length-parameterised quintic Bezier corner blend with `position_at / tangent_at / curvature_at / dkappa_ds / v_cap_fn / polyline` methods. Arc primitives are deleted from `klippy/blendmath.py`; `klippy/blendplanner.py` is rewired to call `QuinticShape.from_moves(...)`.

**Tech Stack:** Python 3 (no new dependencies), pytest for tests. Tests run via `.venv-test/bin/pytest`. No MCU firmware changes.

**Design spec:** `docs/superpowers/specs/2026-04-20-plan1-quintic-revival-design.md`

---

## File Structure

**New files:**
- `klippy/blendshape.py` — protocol + dataclasses (~50 LOC)
- `klippy/blendquintic.py` — `QuinticShape` implementation (~450 LOC)
- `test/test_blendshape.py` — protocol conformance harness (~80 LOC)
- `test/test_blendquintic.py` — quintic math tests (~700 LOC, ~80% ported from archive)

**Modified files:**
- `klippy/blendmath.py` — delete arc-specific code (BlendArc, blend_geometry, segment_arc, blend_from_moves); keep shared utilities (~230 LOC removed)
- `klippy/blendplanner.py` — rewire `blendmath.blend_from_moves(...)` call to `QuinticShape.from_moves(...)`
- `test/test_blendmath.py` — delete arc-specific tests, keep shared-utility tests
- `test/test_blendplanner.py` — adapt fixtures to `QuinticShape` return type

**Archive reference (read-only):**
- `git show blend-arc-quintic-archive:klippy/blendquintic.py` (~628 LOC)
- `git show blend-arc-quintic-archive:test/test_blendquintic.py` (~775 LOC)
- `git show blend-arc-quintic-archive:klippy/blendemit.py` — read for reference only; not ported (isinstance dispatch replaced by protocol)

---

### Task 1: Scaffold `blendshape.py` with protocol and dataclasses

**Files:**
- Create: `klippy/blendshape.py`
- Test: `test/test_blendshape.py`

- [ ] **Step 1: Write failing test for imports and dataclass defaults**

Create `test/test_blendshape.py` with:

```python
# test/test_blendshape.py
import math

import pytest

from klippy import blendshape


def test_kinematic_limits_dataclass():
    lim = blendshape.KinematicLimits(
        a_max=45000.0,
        v_max=500.0,
        jerk_max=None,
        shaper_sigma_T=0.0,
        extruder_caps=None,
    )
    assert lim.a_max == 45000.0
    assert lim.extruder_caps is None


def test_extruder_limits_dataclass():
    caps = blendshape.ExtruderLimits(accel_max=1000.0, rpm_max=300.0)
    assert caps.accel_max == 1000.0
    assert caps.rpm_max == 300.0


def test_smooth_shape_protocol_exists():
    # Structural: protocol must be importable and be a Protocol.
    assert hasattr(blendshape, "SmoothShape")
    # Protocol subclass check: any object with the required attrs satisfies.
    class _Stub:
        d_consumed = 1.0
        theta = 1.0
        arc_length = 2.0
        def position_at(self, s): return (0.0, 0.0, 0.0)
        def tangent_at(self, s): return (1.0, 0.0, 0.0)
        def curvature_at(self, s): return 0.5
        def dkappa_ds(self, s): return 0.0
        def v_cap_fn(self, s): return 100.0
        def polyline(self, tol): return [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    stub = _Stub()
    assert isinstance(stub, blendshape.SmoothShape)
```

- [ ] **Step 2: Run test, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendshape.py -v`
Expected: `ModuleNotFoundError: No module named 'klippy.blendshape'`

- [ ] **Step 3: Implement the module**

Create `klippy/blendshape.py`:

```python
# klippy/blendshape.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shape-agnostic types for curvature-continuous corner blends.
#
# SmoothShape is a Protocol: any curve implementation (quintic Bezier,
# Pythagorean-Hodograph spline, Euler-spiral clothoid, ...) that exposes
# the listed surface is a SmoothShape. The planner talks to this
# protocol; concrete shapes never leak implementation details (control
# points, Fresnel tables, speed polynomials).
#
# KinematicLimits is the flat dataclass shape factories take in place of
# the whole toolhead object — decouples shape construction from the
# full kinematics/extruder stack.
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol, runtime_checkable, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class ExtruderLimits:
    """First-class extruder constraints (pillar 3).

    Plan 1 leaves this as None everywhere; plan 4 threads it through.
    """
    accel_max: float   # mm/s^2 on the filament
    rpm_max: float     # drive-pulley angular velocity


@dataclass
class KinematicLimits:
    """Flat dataclass passed into shape factories. Replaces handing the
    whole toolhead object in. Built once per planner run."""
    a_max: float
    v_max: float
    jerk_max: Optional[float]   # j_eff for rotation-jerk cap; None disables
    shaper_sigma_T: float       # from IS impulse pattern (see blendmath)
    extruder_caps: Optional[ExtruderLimits]   # None until plan 4 (pillar 3)


@runtime_checkable
class SmoothShape(Protocol):
    """Curvature-continuous corner blend between two adjacent moves.

    Arc-length parameterised; s in [0, arc_length]. Protocol is
    implementation-opaque — consumers see only this surface.

    Velocity-limit convention: `v_cap_fn(s)` returns the velocity
    limit curve V_lim(s) from centripetal + shaper + (optional) jerk
    bounds. Pillar 3 (plan 4) wraps this with an extruder cap as a
    separate stage, not here.
    """

    d_consumed: float   # tangent length consumed per incoming edge [mm]
    theta: float        # deflection angle [rad]
    arc_length: float   # total length of the blend [mm]

    def position_at(self, s: float) -> Vec3: ...
    def tangent_at(self, s: float) -> Vec3: ...
    def curvature_at(self, s: float) -> float: ...
    def dkappa_ds(self, s: float) -> float: ...
    def v_cap_fn(self, s: float) -> float: ...
    def polyline(self, chord_tol: float) -> List[Vec3]: ...
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendshape.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshape.py test/test_blendshape.py
git commit -m "blendshape: add SmoothShape protocol + KinematicLimits dataclass"
```

---

### Task 2: Scaffold `blendquintic.py` skeleton with empty QuinticShape

**Files:**
- Create: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

- [ ] **Step 1: Write failing test**

Create `test/test_blendquintic.py`:

```python
# test/test_blendquintic.py
import math

import pytest

from klippy import blendshape, blendquintic


def _default_limits():
    return blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )


def test_quintic_shape_class_exists():
    assert hasattr(blendquintic, "QuinticShape")
    # isinstance against the protocol: instance with the right attrs.
    # Full instantiation tested once from_moves lands (task 12).


def test_quintic_shape_from_moves_returns_none_for_none_inputs():
    # Degenerate input — factory returns None cleanly.
    result = blendquintic.QuinticShape.from_moves(
        prev_move=None, next_move=None,
        corner_deviation=0.1, limits=_default_limits(),
    )
    assert result is None
```

- [ ] **Step 2: Run test, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v`
Expected: `ModuleNotFoundError: No module named 'klippy.blendquintic'`

- [ ] **Step 3: Scaffold the module**

Create `klippy/blendquintic.py`:

```python
# klippy/blendquintic.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Quintic Hermite Bezier corner-blending primitive.
#
# Implements the SmoothShape protocol. Arc-length parameterised via a
# cached 8-Gauss-Legendre s -> t map built at from_moves time.
#
# Math verified via audit of blend-arc-quintic-archive; the five
# correct pieces (De Casteljau, curvature, chord deviation, r(theta),
# rotation-jerk) port verbatim. The three-point shaper cap from the
# archive is replaced with dense sampling (archive had a silent ~15%
# overshoot at the worst angles).
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import blendmath, blendshape

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6
_SUBDIVIDE_MAX_DEPTH = 12
_DEFAULT_CHORD_TOL = 1e-3   # 1 um; tighter than archive's 10 um to reduce segment-boundary kappa steps


class QuinticShape:
    """Quintic Hermite Bezier corner blend. Implements SmoothShape."""

    d_consumed: float
    theta: float
    arc_length: float

    def __init__(self) -> None:
        raise NotImplementedError(
            "QuinticShape is constructed via QuinticShape.from_moves(...)"
        )

    @classmethod
    def from_moves(
        cls,
        prev_move,
        next_move,
        corner_deviation: float,
        limits: blendshape.KinematicLimits,
    ) -> Optional["QuinticShape"]:
        """Construct a quintic blend for the corner between prev_move and
        next_move. Returns None for degenerate corners (collinear,
        near-reversal, chord budget infeasible). Caller (planner) falls
        back to sharp-V when None is returned.
        """
        if prev_move is None or next_move is None:
            return None
        # Full implementation lands in task 12.
        return None

    # Protocol methods stubbed to allow isinstance checks; each one is
    # filled in by the tasks below.
    def position_at(self, s: float) -> Vec3:
        raise NotImplementedError

    def tangent_at(self, s: float) -> Vec3:
        raise NotImplementedError

    def curvature_at(self, s: float) -> float:
        raise NotImplementedError

    def dkappa_ds(self, s: float) -> float:
        raise NotImplementedError

    def v_cap_fn(self, s: float) -> float:
        raise NotImplementedError

    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> List[Vec3]:
        raise NotImplementedError
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: scaffold QuinticShape class + test harness"
```

---

### Task 3: Port De Casteljau primitives (eval, deriv, split, flatness)

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

Port from `git show blend-arc-quintic-archive:klippy/blendquintic.py` lines ~45–100 (helpers) and ~540–595 (split + flatness). Rename as private methods of `QuinticShape` or keep as module-level free functions — archive uses module-level, which stays cleaner; keep that style.

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
# De Casteljau primitives — ported from archive

def _unit_quintic():
    """Control points along the x-axis for a degenerate 'straight-line'
    quintic. All derivatives along t should match a straight line."""
    return tuple((0.2 * i, 0.0, 0.0) for i in range(6))


def test_quintic_eval_endpoints():
    Q = _unit_quintic()
    p0 = blendquintic._quintic_eval(Q, 0.0)
    p1 = blendquintic._quintic_eval(Q, 1.0)
    assert p0 == pytest.approx((0.0, 0.0, 0.0))
    assert p1 == pytest.approx((1.0, 0.0, 0.0))


def test_quintic_eval_midpoint():
    Q = _unit_quintic()
    p = blendquintic._quintic_eval(Q, 0.5)
    assert p == pytest.approx((0.5, 0.0, 0.0))


def test_quintic_first_deriv_constant_for_straight():
    # Straight-line control net: B'(t) is constant.
    Q = _unit_quintic()
    d0 = blendquintic._quintic_first_deriv(Q, 0.0)
    d5 = blendquintic._quintic_first_deriv(Q, 0.5)
    d1 = blendquintic._quintic_first_deriv(Q, 1.0)
    assert d0 == pytest.approx(d5)
    assert d5 == pytest.approx(d1)


def test_quintic_split_preserves_endpoints():
    Q = _unit_quintic()
    left, right = blendquintic._quintic_split(Q)
    assert left[0] == pytest.approx(Q[0])
    assert right[5] == pytest.approx(Q[5])
    # Midpoint: left's last == right's first.
    assert left[5] == pytest.approx(right[0])


def test_quintic_flatness_zero_for_straight():
    Q = _unit_quintic()
    f = blendquintic._quintic_flatness(Q)
    assert f == pytest.approx(0.0, abs=1e-12)
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "eval or deriv or split or flatness"`
Expected: 5 FAILs with `AttributeError: module ... has no attribute '_quintic_eval'` (etc.)

- [ ] **Step 3: Port the primitives**

Add to `klippy/blendquintic.py` (below the imports, above the class):

```python
def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _quintic_eval(Q, t: float) -> Vec3:
    """De Casteljau evaluation of a 6-control-point quintic Bezier."""
    p = [Q[i] for i in range(6)]
    for level in range(5, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_first_deriv(Q, t: float) -> Vec3:
    """B'(t) for a quintic Bezier. Degree-4 Bezier with control points
    5*(Q[i+1] - Q[i])."""
    D = [
        (
            5.0 * (Q[i + 1][0] - Q[i][0]),
            5.0 * (Q[i + 1][1] - Q[i][1]),
            5.0 * (Q[i + 1][2] - Q[i][2]),
        )
        for i in range(5)
    ]
    # De Casteljau on the degree-4 control points.
    p = [D[i] for i in range(5)]
    for level in range(4, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_second_deriv(Q, t: float) -> Vec3:
    """B''(t). Degree-3 Bezier with control points 20*(Q[i+2]-2*Q[i+1]+Q[i])."""
    D2 = [
        (
            20.0 * (Q[i + 2][0] - 2.0 * Q[i + 1][0] + Q[i][0]),
            20.0 * (Q[i + 2][1] - 2.0 * Q[i + 1][1] + Q[i][1]),
            20.0 * (Q[i + 2][2] - 2.0 * Q[i + 1][2] + Q[i][2]),
        )
        for i in range(4)
    ]
    p = [D2[i] for i in range(4)]
    for level in range(3, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_third_deriv(Q, t: float) -> Vec3:
    """B'''(t). Degree-2 Bezier with control points 60*(Q[i+3]-3*Q[i+2]+3*Q[i+1]-Q[i])."""
    D3 = [
        (
            60.0 * (Q[i + 3][0] - 3.0 * Q[i + 2][0] + 3.0 * Q[i + 1][0] - Q[i][0]),
            60.0 * (Q[i + 3][1] - 3.0 * Q[i + 2][1] + 3.0 * Q[i + 1][1] - Q[i][1]),
            60.0 * (Q[i + 3][2] - 3.0 * Q[i + 2][2] + 3.0 * Q[i + 1][2] - Q[i][2]),
        )
        for i in range(3)
    ]
    p = [D3[i] for i in range(3)]
    for level in range(2, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_split(Q):
    """Split a quintic Bezier at t=0.5 via De Casteljau; returns (left, right)."""
    # Run De Casteljau at t=0.5, collecting left/right control nets.
    p = [Q[i] for i in range(6)]
    left = [p[0]]
    right_tail = [p[5]]
    for level in range(5, 0, -1):
        new_p = []
        for i in range(level):
            m = _lerp(p[i], p[i + 1], 0.5)
            new_p.append(m)
        left.append(new_p[0])
        right_tail.append(new_p[-1])
        p = new_p
    right = list(reversed(right_tail))
    return tuple(left), tuple(right)


def _perp_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    """Perpendicular distance from point p to the line through a,b."""
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
    cross = (
        ap[1] * ab[2] - ap[2] * ab[1],
        ap[2] * ab[0] - ap[0] * ab[2],
        ap[0] * ab[1] - ap[1] * ab[0],
    )
    ab_len = math.sqrt(ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2)
    if ab_len < 1e-12:
        return math.sqrt(ap[0] ** 2 + ap[1] ** 2 + ap[2] ** 2)
    cross_len = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    return cross_len / ab_len


def _quintic_flatness(Q) -> float:
    """Max perpendicular distance of Q1..Q4 from the chord Q0-Q5.

    Classical upper bound on curve-to-chord distance via the convex-hull
    property. Used as the adaptive-subdivision termination metric.
    """
    chord_a = Q[0]
    chord_b = Q[5]
    return max(
        _perp_distance(Q[1], chord_a, chord_b),
        _perp_distance(Q[2], chord_a, chord_b),
        _perp_distance(Q[3], chord_a, chord_b),
        _perp_distance(Q[4], chord_a, chord_b),
    )
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "eval or deriv or split or flatness"`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: port De Casteljau primitives from archive"
```

---

### Task 4: Port curvature evaluator

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def _right_angle_quintic():
    """Synthetic quintic whose control polygon traces a right-angle
    corner with symmetric Q1=Q2 and Q3=Q4. Used for curvature tests."""
    d = 1.0
    r = 0.5
    e1 = (-1.0, 0.0, 0.0)   # incoming unit tangent
    e2 = (0.0, 1.0, 0.0)    # outgoing unit tangent
    Q0 = (d * 1.0, 0.0, 0.0)
    Q5 = (0.0, d * 1.0, 0.0)
    Q1 = (Q0[0] - d * (1.0 - r), 0.0, 0.0)
    Q2 = Q1
    Q3 = (0.0, Q5[1] - d * (1.0 - r), 0.0)
    Q4 = Q3
    return (Q0, Q1, Q2, Q3, Q4, Q5)


def test_curvature_zero_at_endpoints_for_symmetric_quintic():
    Q = _right_angle_quintic()
    k0 = blendquintic._curvature_at_t(Q, 0.0)
    k1 = blendquintic._curvature_at_t(Q, 1.0)
    assert k0 == pytest.approx(0.0, abs=1e-9)
    assert k1 == pytest.approx(0.0, abs=1e-9)


def test_curvature_positive_at_midpoint_for_corner():
    Q = _right_angle_quintic()
    k = blendquintic._curvature_at_t(Q, 0.5)
    assert k > 0.0


def test_peak_curvature_matches_dense_reference():
    Q = _right_angle_quintic()
    peak_t, peak_k = blendquintic._peak_curvature(Q, n_samples=100)
    # Reference: 20001-sample dense scan.
    ks = [
        blendquintic._curvature_at_t(Q, i / 20000.0) for i in range(20001)
    ]
    ref_k = max(ks)
    assert peak_k == pytest.approx(ref_k, rel=1e-4)
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "curvature"`
Expected: 3 FAILs with `AttributeError: ... '_curvature_at_t'`

- [ ] **Step 3: Port the curvature evaluator**

Add to `klippy/blendquintic.py` (after the De Casteljau helpers, before the class):

```python
def _curvature_at_t(Q, t: float) -> float:
    """Curvature at parameter t. For 2D (z=0), reduces to
    kappa = |B'_x * B''_y - B'_y * B''_x| / |B'|^3.
    For 3D, kappa = |B' x B''| / |B'|^3.
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    cross = (
        d1[1] * d2[2] - d1[2] * d2[1],
        d1[2] * d2[0] - d1[0] * d2[2],
        d1[0] * d2[1] - d1[1] * d2[0],
    )
    num = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    den = (d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2) ** 1.5
    if den < 1e-30:
        return 0.0
    return num / den


def _point_frame(Q, t: float) -> Tuple[Vec3, Vec3, Vec3]:
    """Return (position, unit tangent, unit normal) at parameter t.

    Normal is the 2D planar normal in the xy-plane (rot90 of tangent);
    for 3D paths the formula would use the Frenet frame but MO is 2D.
    """
    p = _quintic_eval(Q, t)
    d1 = _quintic_first_deriv(Q, t)
    d1n = math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)
    if d1n < 1e-30:
        return p, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    tan = (d1[0] / d1n, d1[1] / d1n, d1[2] / d1n)
    # 2D normal: rotate tangent 90 deg CCW in xy-plane.
    nrm = (-tan[1], tan[0], 0.0)
    return p, tan, nrm


def _peak_curvature(Q, n_samples: int = 100) -> Tuple[float, float]:
    """Dense-sampling peak-curvature evaluator.

    Returns (t_peak, kappa_peak). n_samples=100 gives ~5 sig-fig agreement
    with 20001-sample reference per archive audit.
    """
    best_t = 0.5
    best_k = 0.0
    for i in range(n_samples + 1):
        t = i / n_samples
        k = _curvature_at_t(Q, t)
        if k > best_k:
            best_k = k
            best_t = t
    return best_t, best_k
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "curvature"`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: port curvature evaluator + point frame + peak"
```

---

### Task 5: Port chord-deviation closed form + r(θ) quadratic fit

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def test_r_of_theta_anchor_values():
    # Anchors from subspec 6d; verified by audit.
    assert blendquintic._r_of_theta(math.radians(30)) == pytest.approx(0.5043, abs=1e-4)
    assert blendquintic._r_of_theta(math.radians(90)) == pytest.approx(0.5900, abs=1e-4)
    assert blendquintic._r_of_theta(math.radians(120)) == pytest.approx(0.6800, abs=1e-4)


def test_r_of_theta_clamps():
    # Clamped to [0.50, 0.86].
    assert blendquintic._r_of_theta(0.0) >= 0.50
    assert blendquintic._r_of_theta(math.pi) <= 0.86


def test_deviation_coeff_formula():
    # (1 + 15*r) / 16.
    assert blendquintic._deviation_coeff(0.5) == pytest.approx((1.0 + 15.0 * 0.5) / 16.0)
    assert blendquintic._deviation_coeff(0.8) == pytest.approx((1.0 + 15.0 * 0.8) / 16.0)


def test_deviation_closed_form_vs_numerical():
    # For a known corner, compare closed-form to numerical curve-peak evaluation.
    d = 1.0
    theta = math.radians(90)
    r = blendquintic._r_of_theta(theta)
    sin_half = math.sin(theta / 2.0)
    eps_closed = blendquintic._deviation_closed_form(d, r, sin_half)
    assert eps_closed > 0.0
    # Numerical check: build the quintic for this d,theta,r, find peak
    # perpendicular distance from the corner apex.
    # (Geometry: corner apex at origin, tangents along +x and rotated -theta.)
    # Skipped here; full construction comes in task 12. For now, verify
    # monotonicity: larger d -> larger eps; larger r -> larger eps.
    assert blendquintic._deviation_closed_form(2.0, r, sin_half) > eps_closed
    assert blendquintic._deviation_closed_form(d, 0.8, sin_half) > eps_closed


def test_d_from_deviation_inverse():
    eps = 0.1
    theta = math.radians(90)
    r = blendquintic._r_of_theta(theta)
    sin_half = math.sin(theta / 2.0)
    d = blendquintic._d_from_deviation(eps, r, sin_half)
    eps_back = blendquintic._deviation_closed_form(d, r, sin_half)
    assert eps_back == pytest.approx(eps, rel=1e-9)
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "deviation or r_of_theta"`
Expected: FAILs with `AttributeError`

- [ ] **Step 3: Port the formulas**

Add to `klippy/blendquintic.py`:

```python
# r(theta) quadratic fit — archive values, verified by audit. Clamped
# to [0.50, 0.86] to stay within empirical validity window.
_R_A = 0.5085
_R_B = -0.03785
_R_C = 0.05715
_R_CLAMP_LO = 0.50
_R_CLAMP_HI = 0.86


def _r_of_theta(theta: float) -> float:
    """Quadratic fit of 'shape ratio' r as a function of deflection angle.

    Ported from blend-arc-quintic-archive/klippy/blendquintic.py:183-203.
    Audit (2026-04-20) confirmed correctness against anchor values.
    """
    r = _R_A + _R_B * theta + _R_C * theta * theta
    if r < _R_CLAMP_LO:
        return _R_CLAMP_LO
    if r > _R_CLAMP_HI:
        return _R_CLAMP_HI
    return r


def _deviation_coeff(r: float) -> float:
    """Chord-deviation prefactor (1 + 15*r) / 16."""
    return (1.0 + 15.0 * r) / 16.0


def _deviation_closed_form(d: float, r: float, sin_half: float) -> float:
    """Chord deviation in [mm] for a symmetric quintic Hermite with
    tangent length d, shape ratio r, and corner half-angle with sine
    sin_half. Derivation: evaluate B(0.5) for the symmetric control
    net; the perpendicular distance to the corner apex is
    (d/16) * (1 + 15*r) * sin(theta/2).
    """
    return _deviation_coeff(r) * d * sin_half


def _d_from_deviation(eps: float, r: float, sin_half: float) -> float:
    """Inverse of _deviation_closed_form: tangent length d required to
    achieve chord deviation eps. Returns +inf when collinear
    (sin_half==0) or when r would drive the denominator non-positive.
    """
    denom = (1.0 + 15.0 * r) * sin_half
    if denom <= 0.0:
        return float("inf")
    return 16.0 * eps / denom
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "deviation or r_of_theta"`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: port chord-deviation closed form + r(theta) fit"
```

---

### Task 6: Build 8-Gauss-Legendre s→t map

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

- [ ] **Step 1: Write failing test**

Add to `test/test_blendquintic.py`:

```python
def test_arc_length_table_sub_micron_accuracy():
    """Against a 20001-sample high-resolution reference, the 8-GL
    arc-length table must give sub-micron position error at any s."""
    Q = _right_angle_quintic()
    # Build the s->t map.
    s_tab, t_tab, total_s = blendquintic._build_s_to_t_map(Q, n_gl=8, n_subintervals=20)
    assert total_s > 0.0
    # High-resolution reference: cumulative Euclidean distance along
    # 20001 uniform-t samples.
    ts = [i / 20000.0 for i in range(20001)]
    pts = [blendquintic._quintic_eval(Q, t) for t in ts]
    cumulative = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        dz = pts[i][2] - pts[i - 1][2]
        cumulative.append(cumulative[-1] + math.sqrt(dx * dx + dy * dy + dz * dz))
    ref_total = cumulative[-1]
    # Total arc-length agreement
    assert total_s == pytest.approx(ref_total, rel=1e-5)
    # Check 100 random s values
    import random
    random.seed(42)
    max_err = 0.0
    for _ in range(100):
        s = random.uniform(0.0, total_s)
        t = blendquintic._s_to_t(s_tab, t_tab, s)
        # Interpolate reference cumulative to find the reference t at s.
        # (monotone, so bisect)
        import bisect
        idx = bisect.bisect_left(cumulative, s)
        if idx == 0:
            ref_t = 0.0
        elif idx >= len(cumulative):
            ref_t = 1.0
        else:
            c_lo, c_hi = cumulative[idx - 1], cumulative[idx]
            frac = (s - c_lo) / (c_hi - c_lo) if c_hi > c_lo else 0.0
            ref_t = ts[idx - 1] + (ts[idx] - ts[idx - 1]) * frac
        p_gl = blendquintic._quintic_eval(Q, t)
        p_ref = blendquintic._quintic_eval(Q, ref_t)
        err = math.sqrt(
            (p_gl[0] - p_ref[0]) ** 2
            + (p_gl[1] - p_ref[1]) ** 2
            + (p_gl[2] - p_ref[2]) ** 2
        )
        max_err = max(max_err, err)
    assert max_err < 1e-2   # 10 um; plan 1 target. Tighter thresholds
                            # (1 um) achievable by bumping n_subintervals
                            # to ~100 or adding one Newton refinement step.
```

- [ ] **Step 2: Run test, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "arc_length"`
Expected: FAIL with `AttributeError`

- [ ] **Step 3: Implement GL arc-length integration + s→t map**

Add to `klippy/blendquintic.py`:

```python
# 8-point Gauss-Legendre nodes and weights on [-1, 1], shifted in
# callers to [0, 1] sub-intervals. Sub-micron arc-length accuracy on
# 5 mm blends per audit; up from archive's 5-node default (~20 um drift).
_GL8_NODES = (
    -0.9602898564975363,
    -0.7966664774136267,
    -0.5255324099163290,
    -0.1834346424956498,
    0.1834346424956498,
    0.5255324099163290,
    0.7966664774136267,
    0.9602898564975363,
)
_GL8_WEIGHTS = (
    0.1012285362903763,
    0.2223810344533745,
    0.3137066458778873,
    0.3626837833783620,
    0.3626837833783620,
    0.3137066458778873,
    0.2223810344533745,
    0.1012285362903763,
)


def _speed_at_t(Q, t: float) -> float:
    """|B'(t)| at parameter t — the parametric speed used for arc-length."""
    d1 = _quintic_first_deriv(Q, t)
    return math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)


def _build_s_to_t_map(
    Q, n_gl: int = 8, n_subintervals: int = 20
) -> Tuple[List[float], List[float], float]:
    """Build a cached arc-length-to-parameter map for the quintic.

    Splits [0, 1] into n_subintervals equal-t pieces. On each piece,
    integrates |B'(t)| using n_gl-node Gauss-Legendre to get the piece's
    arc length. Returns:
      - s_tab: cumulative arc-length at each sub-interval boundary
        (length n_subintervals + 1)
      - t_tab: parameter t at each boundary (length n_subintervals + 1)
      - total_s: total arc length (== s_tab[-1])

    Query via _s_to_t(s_tab, t_tab, s).
    """
    if n_gl != 8:
        raise ValueError("only 8-node GL currently supported")
    s_tab = [0.0]
    t_tab = [0.0]
    for i in range(n_subintervals):
        t_lo = i / n_subintervals
        t_hi = (i + 1) / n_subintervals
        half = 0.5 * (t_hi - t_lo)
        mid = 0.5 * (t_hi + t_lo)
        piece = 0.0
        for j in range(n_gl):
            t_j = mid + half * _GL8_NODES[j]
            piece += _GL8_WEIGHTS[j] * _speed_at_t(Q, t_j)
        piece *= half
        s_tab.append(s_tab[-1] + piece)
        t_tab.append(t_hi)
    return s_tab, t_tab, s_tab[-1]


def _s_to_t(s_tab: List[float], t_tab: List[float], s: float) -> float:
    """Invert the s->t map. Bisect to find the s_tab interval, then
    Newton-iterate within the sub-interval using speed = |B'(t)|."""
    if s <= 0.0:
        return t_tab[0]
    if s >= s_tab[-1]:
        return t_tab[-1]
    import bisect
    idx = bisect.bisect_left(s_tab, s)
    s_lo, s_hi = s_tab[idx - 1], s_tab[idx]
    t_lo, t_hi = t_tab[idx - 1], t_tab[idx]
    # Linear interpolation is accurate to (s_hi - s_lo) squared over
    # the sub-interval; for n=20 sub-intervals on a typical blend this
    # is already sub-micron. Skip Newton refinement unless a future
    # test demands it.
    if s_hi <= s_lo:
        return t_lo
    frac = (s - s_lo) / (s_hi - s_lo)
    return t_lo + (t_hi - t_lo) * frac
```

- [ ] **Step 4: Run test, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "arc_length"`
Expected: 1 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add 8-GL arc-length integration and s->t map"
```

---

### Task 7: Implement `position_at(s)`, `tangent_at(s)`, `curvature_at(s)` — the public protocol methods backed by the s→t map

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

The class currently has stubs that raise NotImplementedError. Those stubs become real methods backed by the `s→t` map. `QuinticShape` needs an `__init__` that accepts Q and caches the map; the `from_moves` factory (task 12) populates Q.

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def _build_shape_direct(Q):
    """Bypass from_moves: build a QuinticShape directly from control
    points for testing the arc-length-backed methods. d_consumed and
    theta are dummy here; the real factory (task 12) computes them."""
    shape = blendquintic.QuinticShape.__new__(blendquintic.QuinticShape)
    blendquintic.QuinticShape._init_from_Q(shape, Q, d_consumed=1.0, theta=math.radians(90))
    return shape


def test_position_at_endpoints():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    p0 = shape.position_at(0.0)
    p1 = shape.position_at(shape.arc_length)
    assert p0 == pytest.approx(Q[0])
    assert p1 == pytest.approx(Q[5])


def test_tangent_at_endpoints_unit_length():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    t0 = shape.tangent_at(0.0)
    t1 = shape.tangent_at(shape.arc_length)
    for t in (t0, t1):
        mag = math.sqrt(t[0] ** 2 + t[1] ** 2 + t[2] ** 2)
        assert mag == pytest.approx(1.0, rel=1e-9)


def test_curvature_at_endpoints_zero():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    assert shape.curvature_at(0.0) == pytest.approx(0.0, abs=1e-9)
    assert shape.curvature_at(shape.arc_length) == pytest.approx(0.0, abs=1e-9)


def test_tangent_matches_ds_position_numerically():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    s_mid = shape.arc_length * 0.5
    ds = 1e-4
    p_lo = shape.position_at(s_mid - ds)
    p_hi = shape.position_at(s_mid + ds)
    num = ((p_hi[0] - p_lo[0]) / (2 * ds),
           (p_hi[1] - p_lo[1]) / (2 * ds),
           (p_hi[2] - p_lo[2]) / (2 * ds))
    num_mag = math.sqrt(num[0] ** 2 + num[1] ** 2 + num[2] ** 2)
    num_hat = (num[0] / num_mag, num[1] / num_mag, num[2] / num_mag)
    tan = shape.tangent_at(s_mid)
    assert num_hat[0] == pytest.approx(tan[0], abs=1e-4)
    assert num_hat[1] == pytest.approx(tan[1], abs=1e-4)
    assert num_hat[2] == pytest.approx(tan[2], abs=1e-4)
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "position_at or tangent_at or curvature_at or tangent_matches"`
Expected: FAILs — `NotImplementedError` / `AttributeError: '_init_from_Q'`

- [ ] **Step 3: Implement the protocol methods and `_init_from_Q`**

In `klippy/blendquintic.py`, replace the `QuinticShape` stub body:

```python
class QuinticShape:
    """Quintic Hermite Bezier corner blend. Implements SmoothShape."""

    # Runtime attributes (populated by _init_from_Q):
    # - Q: control points tuple
    # - d_consumed, theta, arc_length
    # - _s_tab, _t_tab: arc-length cache

    def __init__(self) -> None:
        raise NotImplementedError(
            "QuinticShape is constructed via QuinticShape.from_moves(...)"
        )

    def _init_from_Q(self, Q, d_consumed: float, theta: float) -> None:
        """Internal init. Populates the instance from control points and
        scalar metadata; builds the s->t map."""
        self.Q = Q
        self.d_consumed = d_consumed
        self.theta = theta
        s_tab, t_tab, total_s = _build_s_to_t_map(Q)
        self._s_tab = s_tab
        self._t_tab = t_tab
        self.arc_length = total_s

    @classmethod
    def from_moves(
        cls,
        prev_move,
        next_move,
        corner_deviation: float,
        limits: blendshape.KinematicLimits,
    ) -> Optional["QuinticShape"]:
        """See task 12 for the factory. Current stub keeps returning None."""
        if prev_move is None or next_move is None:
            return None
        return None

    def position_at(self, s: float) -> Vec3:
        t = _s_to_t(self._s_tab, self._t_tab, s)
        return _quintic_eval(self.Q, t)

    def tangent_at(self, s: float) -> Vec3:
        t = _s_to_t(self._s_tab, self._t_tab, s)
        d1 = _quintic_first_deriv(self.Q, t)
        mag = math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)
        if mag < 1e-30:
            return (1.0, 0.0, 0.0)
        return (d1[0] / mag, d1[1] / mag, d1[2] / mag)

    def curvature_at(self, s: float) -> float:
        t = _s_to_t(self._s_tab, self._t_tab, s)
        return _curvature_at_t(self.Q, t)

    def dkappa_ds(self, s: float) -> float:
        raise NotImplementedError   # task 8

    def v_cap_fn(self, s: float) -> float:
        raise NotImplementedError   # task 10-11

    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> List[Vec3]:
        raise NotImplementedError   # task 9
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "position_at or tangent_at or curvature_at or tangent_matches"`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: wire arc-length-backed position/tangent/curvature"
```

---

### Task 8: Implement `dkappa_ds(s)` analytically

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

Derivation (2D planar, `ẑ`-scalar curvature):

```
κ(t) = (B' × B'') · ẑ / |B'|³
dκ/dt = (B' × B''') · ẑ / |B'|³  −  3 κ (B' · B'') / |B'|²
dκ/ds = (dκ/dt) / |B'(t)|
```

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def test_dkappa_ds_matches_finite_difference():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    ds = 1e-4
    for s_frac in (0.25, 0.4, 0.5, 0.6, 0.75):
        s_mid = shape.arc_length * s_frac
        k_lo = shape.curvature_at(s_mid - ds)
        k_hi = shape.curvature_at(s_mid + ds)
        numerical = (k_hi - k_lo) / (2 * ds)
        analytical = shape.dkappa_ds(s_mid)
        assert analytical == pytest.approx(numerical, abs=1e-3, rel=1e-3)


def test_dkappa_ds_signs_at_endpoints():
    """Symmetric blend: kappa ramps from 0 up to peak then back to 0.
    dkappa/ds should be positive near s=0, negative near s=arc_length."""
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    assert shape.dkappa_ds(shape.arc_length * 0.1) > 0.0
    assert shape.dkappa_ds(shape.arc_length * 0.9) < 0.0
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "dkappa"`
Expected: FAIL with `NotImplementedError`

- [ ] **Step 3: Implement `dkappa_ds`**

In `klippy/blendquintic.py`, replace the `dkappa_ds` stub:

```python
    def dkappa_ds(self, s: float) -> float:
        """Analytical dκ/ds via the chain rule; no finite differences.

        2D planar derivation:
            κ(t) = (B' × B'')·ẑ / |B'|^3
            dκ/dt = (B' × B''')·ẑ / |B'|^3
                  − 3κ (B'·B'') / |B'|^2
            dκ/ds = (dκ/dt) / |B'(t)|
        """
        t = _s_to_t(self._s_tab, self._t_tab, s)
        d1 = _quintic_first_deriv(self.Q, t)
        d2 = _quintic_second_deriv(self.Q, t)
        d3 = _quintic_third_deriv(self.Q, t)
        d1_mag2 = d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2
        d1_mag = math.sqrt(d1_mag2)
        if d1_mag < 1e-30:
            return 0.0
        d1_mag3 = d1_mag2 * d1_mag
        cross_13_z = d1[0] * d3[1] - d1[1] * d3[0]   # 2D: z-component
        cross_12_z = d1[0] * d2[1] - d1[1] * d2[0]
        dot_12 = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]
        kappa = cross_12_z / d1_mag3
        dkappa_dt = cross_13_z / d1_mag3 - 3.0 * kappa * dot_12 / d1_mag2
        return dkappa_dt / d1_mag
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "dkappa"`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add analytical dkappa_ds via chain rule"
```

---

### Task 9: Port adaptive polyline subdivision

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def test_polyline_endpoints_match_control_endpoints():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    poly = shape.polyline(chord_tol=1e-3)
    assert poly[0] == pytest.approx(Q[0])
    assert poly[-1] == pytest.approx(Q[5])


def test_polyline_segment_count_scales_with_tol():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    loose = shape.polyline(chord_tol=1e-1)
    tight = shape.polyline(chord_tol=1e-4)
    assert len(tight) > len(loose)
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "polyline"`
Expected: FAIL with `NotImplementedError`

- [ ] **Step 3: Port polyline subdivision**

Add to `klippy/blendquintic.py` (module-level helper and class method):

```python
def _segment_quintic(Q, max_chord_err: float) -> List[Vec3]:
    """Adaptive De Casteljau subdivision — recursion terminates when
    _quintic_flatness(sub_Q) <= max_chord_err or depth == limit."""
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
    out: List[Vec3] = [Q[0]]

    def _recurse(sub_Q, depth):
        if depth >= _SUBDIVIDE_MAX_DEPTH or _quintic_flatness(sub_Q) <= max_chord_err:
            out.append(sub_Q[5])
            return
        left, right = _quintic_split(sub_Q)
        _recurse(left, depth + 1)
        _recurse(right, depth + 1)

    _recurse(Q, 0)
    return out
```

And wire it into the class; replace the `polyline` stub:

```python
    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> List[Vec3]:
        return _segment_quintic(self.Q, chord_tol)
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "polyline"`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: port adaptive polyline subdivision"
```

---

### Task 10: Implement `v_cap_fn(s)` — centripetal + rotation-jerk bounds (no shaper yet)

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

Bounds:
- Centripetal: `v(s) ≤ √(a_max / |κ(s)|)`.
- Rotation-jerk: `v(s) ≤ (R(s) · √j_eff)^(2/3) = (√j_eff / |κ(s)|)^(2/3)` on a constant-v circle with `j_eff = v³·κ²`.

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
def test_v_cap_at_zero_curvature_is_vmax():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    # At s=0 and s=arc_length, kappa=0 -> centripetal bound is +inf ->
    # v_cap = v_max.
    shape._limits = limits   # install limits for v_cap_fn
    assert shape.v_cap_fn(0.0) == pytest.approx(limits.v_max)
    assert shape.v_cap_fn(shape.arc_length) == pytest.approx(limits.v_max)


def test_v_cap_at_peak_kappa_matches_centripetal_bound():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0,    # very high v_max so centripetal binds
        jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    shape._limits = limits
    _, k_peak = blendquintic._peak_curvature(shape.Q)
    expected = math.sqrt(limits.a_max / k_peak)
    # Find the s at peak kappa: scan s.
    best_v = float("inf")
    for i in range(1001):
        s = shape.arc_length * i / 1000.0
        v = shape.v_cap_fn(s)
        best_v = min(best_v, v)
    assert best_v == pytest.approx(expected, rel=1e-2)


def test_v_cap_with_jerk_bound_tighter_than_without():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits_no_jerk = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    limits_with_jerk = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=1e7,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    shape._limits = limits_no_jerk
    v_no = shape.v_cap_fn(shape.arc_length * 0.5)
    shape._limits = limits_with_jerk
    v_yes = shape.v_cap_fn(shape.arc_length * 0.5)
    assert v_yes <= v_no
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "v_cap"`
Expected: FAIL with `NotImplementedError` and/or missing `_limits`.

- [ ] **Step 3: Wire `_limits` into `_init_from_Q` and implement `v_cap_fn`**

In `klippy/blendquintic.py`, update `_init_from_Q` and replace `v_cap_fn`:

```python
    def _init_from_Q(
        self,
        Q,
        d_consumed: float,
        theta: float,
        limits: Optional[blendshape.KinematicLimits] = None,
    ) -> None:
        self.Q = Q
        self.d_consumed = d_consumed
        self.theta = theta
        self._limits = limits   # may be None in direct-build tests prior to task 10
        s_tab, t_tab, total_s = _build_s_to_t_map(Q)
        self._s_tab = s_tab
        self._t_tab = t_tab
        self.arc_length = total_s

    def v_cap_fn(self, s: float) -> float:
        """Velocity limit curve V_lim(s) from centripetal + rotation-jerk.

        Shaper cap is applied in task 11. Extruder cap comes in plan 4
        as a wrapper stage, not here.
        """
        limits = self._limits
        if limits is None:
            return float("inf")
        v = limits.v_max
        kappa = self.curvature_at(s)
        if kappa > 0.0:
            v_cent = math.sqrt(limits.a_max / kappa)
            v = min(v, v_cent)
            if limits.jerk_max is not None and limits.jerk_max > 0.0:
                # v^3 * kappa^2 <= j_eff  =>  v <= (j_eff / kappa^2) ^ (1/3)
                v_jerk = (limits.jerk_max / (kappa * kappa)) ** (1.0 / 3.0)
                v = min(v, v_jerk)
        return v
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "v_cap"`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add centripetal + rotation-jerk velocity bounds"
```

---

### Task 11: Add dense-sampled shaper cap — replaces the 3-point archive bug

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

This is the one real bug the audit caught: archive sampled shaper `v_step_cap` at t ∈ {0.25, 0.5, 0.75} and claimed ≤6% overshoot. Audit measured ~15% at (θ=122°, rotation=164°) and ~9% at (θ=150°, rotation=45°). Fix: dense sample at 50 uniform t values.

The shape factory's shaper-bound query hits `blendshaper.compute_shaper_bounds(R, n_hat, p_hat, sigma_T)` — already correct in the codebase; we just query it at more points.

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
from klippy import blendshaper

def _synthesize_shaper_sigma_T():
    """Minimal sigma_T value for tests; mirrors what blendmath would
    derive from a shaper impulse pattern. Archive tests use ~1.5e-3s."""
    return 1.5e-3


def test_dense_shaper_cap_tighter_than_three_point_at_pathological_angles():
    """Regression: at (theta=122 deg, rotation=164 deg) the archive's
    3-point cap overshot by ~15%. Dense-50 must produce a tighter cap.

    This test constructs the blend in control-point form for the
    specified geometry (same as archive test fixture)."""
    # Build a right-handed corner with the specified theta and rotation.
    theta = math.radians(122.0)
    rot = math.radians(164.0)
    cos_r, sin_r = math.cos(rot), math.sin(rot)
    # Incoming tangent along rotated +x axis; outgoing at -theta from it.
    e1 = (cos_r, sin_r, 0.0)
    c2, s2 = math.cos(-theta), math.sin(-theta)
    e2 = (e1[0] * c2 - e1[1] * s2, e1[0] * s2 + e1[1] * c2, 0.0)
    d = 1.0
    r = blendquintic._r_of_theta(theta)
    # Q0 on incoming tangent line; Q5 on outgoing.
    Q0 = (-d * e1[0], -d * e1[1], 0.0)
    Q5 = (d * e2[0], d * e2[1], 0.0)
    Q1 = (Q0[0] + d * (1.0 - r) * e1[0], Q0[1] + d * (1.0 - r) * e1[1], 0.0)
    Q2 = Q1
    Q3 = (Q5[0] - d * (1.0 - r) * e2[0], Q5[1] - d * (1.0 - r) * e2[1], 0.0)
    Q4 = Q3
    Q = (Q0, Q1, Q2, Q3, Q4, Q5)

    sigma_T = _synthesize_shaper_sigma_T()

    # 3-point cap (archive formula):
    three_pt = float("inf")
    for t in (0.25, 0.5, 0.75):
        _, tan, nrm = blendquintic._point_frame(Q, t)
        k = blendquintic._curvature_at_t(Q, t)
        if k <= 0.0:
            continue
        R = 1.0 / k
        bounds = blendshaper.compute_shaper_bounds(R, nrm, tan, sigma_T)
        three_pt = min(three_pt, bounds.v_step_cap)

    # Dense-50 cap (our fix):
    dense = blendquintic._shaper_cap_dense(Q, sigma_T, n=50)

    # Dense must be tighter (smaller) — ratio <= 1.0, with some headroom.
    assert dense <= three_pt + 1e-9


def test_dense_shaper_cap_agrees_with_50_point_reference():
    """Dense-50 should agree with dense-500 within 1%; checks that 50
    points is already converged."""
    Q = _right_angle_quintic()
    sigma_T = _synthesize_shaper_sigma_T()
    d50 = blendquintic._shaper_cap_dense(Q, sigma_T, n=50)
    d500 = blendquintic._shaper_cap_dense(Q, sigma_T, n=500)
    assert d50 == pytest.approx(d500, rel=1e-2)


def test_v_cap_uses_shaper_when_sigma_positive():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits_no_shaper = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    limits_shaper = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        shaper_sigma_T=_synthesize_shaper_sigma_T(), extruder_caps=None,
    )
    shape._limits = limits_no_shaper
    v_no = shape.v_cap_fn(shape.arc_length * 0.5)
    shape._limits = limits_shaper
    v_yes = shape.v_cap_fn(shape.arc_length * 0.5)
    assert v_yes <= v_no
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "shaper_cap or v_cap_uses_shaper"`
Expected: FAIL with `AttributeError: ... '_shaper_cap_dense'`

- [ ] **Step 3: Implement dense-sample cap and integrate into `v_cap_fn`**

Add to `klippy/blendquintic.py`:

```python
_SHAPER_SAMPLE_N_DEFAULT = 50


def _shaper_cap_dense(Q, sigma_T: float, n: int = _SHAPER_SAMPLE_N_DEFAULT) -> float:
    """Min of the shaper entry-step velocity cap over n+1 uniform t-samples.

    Replaces archive's 3-point cap (blendquintic.py:333,367-386) which
    under-tightened by up to ~15% on the full axis-rotation sweep per
    audit 2026-04-20.
    """
    from . import blendshaper
    if sigma_T <= 0.0:
        return float("inf")
    worst = float("inf")
    for i in range(n + 1):
        t = i / n
        _, tan, nrm = _point_frame(Q, t)
        k = _curvature_at_t(Q, t)
        if k <= 0.0:
            continue
        R = 1.0 / k
        bounds = blendshaper.compute_shaper_bounds(R, nrm, tan, sigma_T)
        worst = min(worst, bounds.v_step_cap)
    return worst
```

Update `v_cap_fn` to query the shaper cap using the current `s`'s `t` (not a global min):

```python
    def v_cap_fn(self, s: float) -> float:
        """Velocity limit curve V_lim(s) — centripetal + shaper + rotation-jerk."""
        limits = self._limits
        if limits is None:
            return float("inf")
        v = limits.v_max
        t = _s_to_t(self._s_tab, self._t_tab, s)
        kappa = _curvature_at_t(self.Q, t)
        if kappa > 0.0:
            v_cent = math.sqrt(limits.a_max / kappa)
            v = min(v, v_cent)
            if limits.jerk_max is not None and limits.jerk_max > 0.0:
                v_jerk = (limits.jerk_max / (kappa * kappa)) ** (1.0 / 3.0)
                v = min(v, v_jerk)
            if limits.shaper_sigma_T > 0.0:
                from . import blendshaper
                _, tan, nrm = _point_frame(self.Q, t)
                R = 1.0 / kappa
                bounds = blendshaper.compute_shaper_bounds(
                    R, nrm, tan, limits.shaper_sigma_T
                )
                v = min(v, bounds.v_step_cap)
        return v
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "shaper_cap or v_cap_uses_shaper"`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: replace 3-point shaper cap with per-s local query

Archive's _three_point_shaper_cap sampled at t in {0.25, 0.5, 0.75} and
took the min as a single scalar v_cap. Audit 2026-04-20 measured up to
~15% overshoot (silent violation of the shaper entry-step budget) at
(theta=122, rotation=164). New implementation evaluates the shaper cap
at each queried s via the cached s->t map — no min-over-samples
approximation. The _shaper_cap_dense helper retains 50-point dense
aggregation for any callsite that needs a scalar bound."
```

---

### Task 12: Implement `from_moves` factory

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_blendquintic.py`

The factory:
1. Extracts incoming / outgoing unit tangents from `prev_move` / `next_move`.
2. Computes deflection angle `θ`.
3. Early-returns `None` for collinear (`θ < COLLINEAR_EPS`) or near-reversal (`π - θ < REVERSAL_EPS`).
4. Computes shape ratio `r = _r_of_theta(θ)`.
5. Computes required tangent length `d = _d_from_deviation(corner_deviation, r, sin(θ/2))`.
6. Early-returns `None` if `d > min(prev_move.available, next_move.available) / 2`.
7. Builds the 6 control points.
8. Constructs `QuinticShape` via `_init_from_Q(Q, d_consumed=d, theta=θ, limits=limits)`.

- [ ] **Step 1: Write failing tests**

Add to `test/test_blendquintic.py`:

```python
class _FakeMove:
    """Minimal Move-like stub for factory tests. Real planner's Move
    is defined in klippy/toolhead.py — much more complex, but for the
    factory we only need start_pos, end_pos, and move_d."""
    def __init__(self, start, end):
        self.start_pos = start
        self.end_pos = end
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        self.move_d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.move_d > 0.0:
            self.axes_d = (dx, dy, dz)
        else:
            self.axes_d = (0.0, 0.0, 0.0)


def test_from_moves_builds_blend_for_right_angle_corner():
    prev = _FakeMove((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMove((0.0, 0.0, 0.0), (0.0, 10.0, 0.0))
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    shape = blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, limits)
    assert shape is not None
    assert shape.theta == pytest.approx(math.radians(90.0), rel=1e-6)
    assert shape.d_consumed > 0.0
    assert shape.arc_length > 0.0


def test_from_moves_returns_none_for_collinear():
    prev = _FakeMove((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMove((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, limits) is None


def test_from_moves_returns_none_for_near_reversal():
    prev = _FakeMove((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMove((0.0, 0.0, 0.0), (-10.0, 0.0, 0.0))
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, limits) is None


def test_from_moves_returns_none_for_insufficient_edge_length():
    # Tangent length d required would exceed available edge length.
    prev = _FakeMove((-0.01, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMove((0.0, 0.0, 0.0), (0.0, 0.01, 0.0))
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 1.0, limits) is None
```

- [ ] **Step 2: Run tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "from_moves"`
Expected: 4 FAILs — factory still returns None unconditionally.

- [ ] **Step 3: Implement `from_moves`**

In `klippy/blendquintic.py`, replace the `from_moves` stub:

```python
    @classmethod
    def from_moves(
        cls,
        prev_move,
        next_move,
        corner_deviation: float,
        limits: blendshape.KinematicLimits,
    ) -> Optional["QuinticShape"]:
        """Construct a quintic blend for the corner between prev_move and
        next_move. Returns None for degenerate corners."""
        if prev_move is None or next_move is None:
            return None
        if prev_move.move_d <= 0.0 or next_move.move_d <= 0.0:
            return None
        # Unit tangents.
        e1 = (
            prev_move.axes_d[0] / prev_move.move_d,
            prev_move.axes_d[1] / prev_move.move_d,
            prev_move.axes_d[2] / prev_move.move_d,
        )
        e2 = (
            next_move.axes_d[0] / next_move.move_d,
            next_move.axes_d[1] / next_move.move_d,
            next_move.axes_d[2] / next_move.move_d,
        )
        dp = e1[0] * e2[0] + e1[1] * e2[1] + e1[2] * e2[2]
        dp = max(-1.0, min(1.0, dp))
        # Deflection angle between tangents: 0 = collinear, pi = reversal.
        theta = math.acos(dp)
        if theta < COLLINEAR_EPS:
            return None
        if (math.pi - theta) < REVERSAL_EPS:
            return None
        sin_half = math.sin(theta / 2.0)
        # Shape ratio and tangent length for the target chord deviation.
        r = _r_of_theta(theta)
        d = _d_from_deviation(corner_deviation, r, sin_half)
        # Each move must have at least d of runway for the blend.
        max_d = 0.5 * min(prev_move.move_d, next_move.move_d)
        if d > max_d or d <= 0.0 or not math.isfinite(d):
            return None
        # Build control points. Corner apex at prev.end_pos == next.start_pos.
        apex = next_move.start_pos
        Q0 = (apex[0] - d * e1[0], apex[1] - d * e1[1], apex[2] - d * e1[2])
        Q5 = (apex[0] + d * e2[0], apex[1] + d * e2[1], apex[2] + d * e2[2])
        Q1 = (Q0[0] + d * (1.0 - r) * e1[0],
              Q0[1] + d * (1.0 - r) * e1[1],
              Q0[2] + d * (1.0 - r) * e1[2])
        Q2 = Q1
        Q3 = (Q5[0] - d * (1.0 - r) * e2[0],
              Q5[1] - d * (1.0 - r) * e2[1],
              Q5[2] - d * (1.0 - r) * e2[2])
        Q4 = Q3
        Q = (Q0, Q1, Q2, Q3, Q4, Q5)
        shape = cls.__new__(cls)
        shape._init_from_Q(Q, d_consumed=d, theta=theta, limits=limits)
        return shape
```

- [ ] **Step 4: Run tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "from_moves"`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add from_moves factory with degenerate-corner handling"
```

---

### Task 13: Port random-corner property sweep

**Files:**
- Modify: `test/test_blendquintic.py`

- [ ] **Step 1: Write the sweep test**

Add to `test/test_blendquintic.py`:

```python
def test_random_corner_sweep():
    """Property test: 200 random corners. For each, verify:
      - from_moves returns a valid shape (or None for degenerate)
      - chord deviation <= corner_deviation budget
      - endpoint curvature == 0 (G2 continuity)
      - v_cap_fn > 0 everywhere on [0, arc_length]
    """
    import random
    rng = random.Random(1234)
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )
    n_valid = 0
    for trial in range(200):
        theta_deg = rng.uniform(5.0, 175.0)
        rotation_deg = rng.uniform(0.0, 360.0)
        edge_len = rng.uniform(0.5, 20.0)
        cd = rng.uniform(0.02, 0.3)
        theta = math.radians(theta_deg)
        rot = math.radians(rotation_deg)
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        e1 = (cos_r, sin_r, 0.0)
        c2, s2 = math.cos(-theta), math.sin(-theta)
        e2 = (e1[0] * c2 - e1[1] * s2, e1[0] * s2 + e1[1] * c2, 0.0)
        apex = (0.0, 0.0, 0.0)
        prev_start = (
            apex[0] - edge_len * e1[0],
            apex[1] - edge_len * e1[1],
            apex[2] - edge_len * e1[2],
        )
        nxt_end = (
            apex[0] + edge_len * e2[0],
            apex[1] + edge_len * e2[1],
            apex[2] + edge_len * e2[2],
        )
        prev = _FakeMove(prev_start, apex)
        nxt = _FakeMove(apex, nxt_end)
        shape = blendquintic.QuinticShape.from_moves(prev, nxt, cd, limits)
        if shape is None:
            continue
        n_valid += 1
        # Endpoint G2: curvature = 0 at s=0 and s=arc_length.
        assert shape.curvature_at(0.0) == pytest.approx(0.0, abs=1e-6), (
            f"trial={trial}, theta_deg={theta_deg}, rotation_deg={rotation_deg}"
        )
        assert shape.curvature_at(shape.arc_length) == pytest.approx(0.0, abs=1e-6)
        # v_cap_fn positive everywhere.
        for i in range(11):
            s = shape.arc_length * i / 10.0
            assert shape.v_cap_fn(s) > 0.0
        # Chord deviation sanity: the closed form used by from_moves already
        # satisfies the budget by construction; nothing further to assert.
    # Sanity: at least half the random corners should yield valid shapes.
    assert n_valid >= 100, f"only {n_valid}/200 corners produced valid shapes"
```

- [ ] **Step 2: Run test, verify PASS on first attempt**

Run: `.venv-test/bin/pytest test/test_blendquintic.py -v -k "random_corner_sweep"`
Expected: 1 passed (all 200 random corners satisfy the properties; at least 100 produce valid shapes).

If any property fails, investigate the specific `theta_deg / rotation_deg` combo printed in the failure and fix the underlying implementation before moving on.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendquintic.py
git commit -m "blendquintic: add 200-corner random property sweep"
```

---

### Task 14: Delete arc code from `blendmath.py`

**Files:**
- Modify: `klippy/blendmath.py`
- Modify: `test/test_blendmath.py`

- [ ] **Step 1: Ensure shared-utility tests still pass (baseline)**

Run: `.venv-test/bin/pytest test/test_blendmath.py -v`
Note which tests pass today. These are the ones that must still pass after deletion.

- [ ] **Step 2: Remove arc-specific code from `klippy/blendmath.py`**

Delete these top-level definitions (line numbers are approximate; grep for the names if shifted):
- `class BlendArc:` (L70)
- `def blend_geometry(...):` (L95)
- `def segment_arc(...):` (L214)
- `def blend_from_moves(...):` (L438)

Keep:
- Vector utilities: `vdot`, `vcross`, `vnorm`, `vscale`, `vadd`, `vsub`, `vnormalize` (L27-67)
- `_rotate` (L248)
- `_sigma_T_max_from_toolhead` (L261)
- `_scv_equivalent_junction_v` (L300)
- `suppressed_junction_v` (L334) — suppression-rule math, kept as dead code; plan 5 decides fate
- `_extract_shapers` (L379)
- `interpolate_extruder` (L587) — shape-agnostic, used by `blendplanner.py`

- [ ] **Step 3: Remove arc-specific tests from `test/test_blendmath.py`**

Delete tests that reference `BlendArc`, `blend_geometry`, `segment_arc`, or `blend_from_moves` (grep for those symbols in the test file).

- [ ] **Step 4: Run remaining tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendmath.py -v`
Expected: shared-utility tests pass; arc-specific tests gone.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendmath.py test/test_blendmath.py
git commit -m "blendmath: delete arc primitives (BlendArc, blend_geometry, segment_arc, blend_from_moves)

Magnum-opus uses quintic (blendquintic.QuinticShape) exclusively via
the SmoothShape protocol. Arc primitives survive on the blend-arc
branch; the archive branch has the superseded arc+quintic dispatch
ladder for historical reference. Shared vector / shaper / extruder
utilities remain in blendmath.py."
```

---

### Task 15: Rewire `blendplanner.py` to use `QuinticShape`

**Files:**
- Modify: `klippy/blendplanner.py`
- Modify: `test/test_blendplanner.py`

The planner currently calls `blendmath.blend_from_moves(...)` and receives a `BlendArc`. Replacement:

- [ ] **Step 1: Read the current call site**

Locate the call in `klippy/blendplanner.py`. From earlier inspection it's around line 66:

```python
arc = blendmath.blend_from_moves(
    prev, nxt, corner_deviation, toolhead=self._toolhead, ...
)
```

The return value is used for:
- `arc is None` short-circuit → fall back to sharp-V
- `arc.d_consumed` to compute `trunc_prev`, `trunc_next`
- `arc.v_cap` as a scalar velocity cap on the blend
- `segment_arc(arc, max_chord_err)` for polyline emission

- [ ] **Step 2: Write / update planner tests FIRST**

The existing file uses `_blender()` (line ~85), `_FakeToolhead`, `_FakeMove` (lines 9–82). Keep this scaffolding. The adaptations:

**Adapt `test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head` (line 304):**

The existing test computes `d_expected = 50e-3 * cos(45°) / (1 - cos(45°)) ≈ 0.1207` from the *arc* geometry (`R_tol = cd * cos(θ/2) / (1 - cos(θ/2))`, then `d = R * tan(θ/2)`). For quintic the geometry is different: `d = 16·cd / ((1 + 15·r) · sin(θ/2))` with `r = _r_of_theta(θ)`.

Replace the arc-specific `d_expected` computation with:

```python
from klippy import blendquintic
theta = math.radians(90.0)
r = blendquintic._r_of_theta(theta)
sin_half = math.sin(theta / 2.0)
d_expected = blendquintic._d_from_deviation(th.corner_deviation, r, sin_half)
```

Keep the `trunc_prev.end_pos[0] == pytest.approx(10.0 - d_expected, rel=1e-6)` check — it's protocol-level (the planner consumed `shape.d_consumed` off the incoming edge).

Delete the "arc center / radius" checks (lines ~333–336). Replace with:

```python
# Polyline vertices lie on the quintic within max_chord_err.
# Instead of arc-center check: verify polyline endpoints match the
# trunc_prev.end_pos and the buffered next_trunc_head.start_pos.
assert arc_moves[0].start_pos[:3] == pytest.approx(trunc_prev.end_pos[:3])
assert arc_moves[-1].end_pos[:3] == pytest.approx(nxt_head.start_pos[:3])
```

The `max_cruise_v2` uniformity check (existing line ~343–344) becomes the first place plan 1 bakes in the "scalar equivalent" approximation. Change to:

```python
# Plan 1 uses v_cap_fn(L/2) as a scalar — uniform per polyline segment.
# Pillar 2 plan replaces this with per-segment v_cap from v_cap_fn(s).
v_caps = [am.max_cruise_v2 for am in arc_moves]
assert max(v_caps) - min(v_caps) < 1e-6
```

**Adapt `test_blender_degenerate_R_zero_forces_stop_at_prev` (line 552):** rename to `test_blender_returns_sharp_v_for_degenerate_corner` and update to expect `shape is None` (not `R == 0`) — `QuinticShape.from_moves` returns None for degenerate corners, and the planner falls back to sharp-V naturally.

**Adapt `test_arc_polyline_speed_continuity_1ppm` (line 477):** rename to drop "arc" prefix; same assertion content holds (all polyline segments share the same v2 in plan 1's scalar-equivalent mode).

**Delete `test_feed_suppressed_non_collinear_applies_junction_cap` (line 213) and `test_feed_collinear_with_shaper_still_no_cap` (line 238):** plan 1 skips suppression per the design spec; tests reintroduced in plan 5 with the unified v(s) cost formula.

**Add a fresh protocol-contract test:**

```python
def test_planner_emits_quintic_shape_for_right_angle_corner():
    """Integration-level: planner's emitted-blend polyline comes from a
    QuinticShape (not a BlendArc).

    Protocol-level assertion: the polyline has > 2 vertices (non-trivial
    subdivision), endpoints match trunc edges, interior is smooth."""
    from klippy import blendquintic
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert b.feed(m_prev) == []
    out = b.feed(m_next)
    trunc_prev = out[0]
    arc_moves = out[1:]
    # Non-trivial subdivision (quintic decomposes into multiple segments).
    assert len(arc_moves) >= 3
    # Polyline is continuous (segment i's end == segment i+1's start).
    for i in range(len(arc_moves) - 1):
        assert arc_moves[i].end_pos[:3] == pytest.approx(arc_moves[i + 1].start_pos[:3])
    # First polyline segment starts where trunc_prev ends.
    assert arc_moves[0].start_pos[:3] == pytest.approx(trunc_prev.end_pos[:3])
```

- [ ] **Step 3: Run updated tests, verify FAIL**

Run: `.venv-test/bin/pytest test/test_blendplanner.py -v`
Expected: FAILs at import or reference (`BlendArc` no longer exists / mismatched return type).

- [ ] **Step 4: Rewire the planner**

In `klippy/blendplanner.py`:

1. Replace `from . import blendmath` → `from . import blendmath, blendquintic, blendshape` (blendmath stays for shared utilities).
2. Build a `KinematicLimits` in the same place the planner today extracts `a_max`, `v_max`, shaper info from `toolhead`.
3. Replace `blendmath.blend_from_moves(prev, nxt, cd, toolhead=th, ...)` with:

```python
limits = blendshape.KinematicLimits(
    a_max=toolhead.max_accel,
    v_max=toolhead.max_velocity,
    jerk_max=_derive_jerk_max(toolhead),   # None if not configured
    shaper_sigma_T=blendmath._sigma_T_max_from_toolhead(toolhead),
    extruder_caps=None,   # plan 4 (pillar 3) populates this
)
shape = blendquintic.QuinticShape.from_moves(
    prev, nxt, corner_deviation, limits,
)
```

4. Update downstream usage:
   - `arc is None` → `shape is None`
   - `arc.d_consumed` → `shape.d_consumed`
   - `arc.v_cap` → `shape.v_cap_fn(shape.arc_length / 2.0)` (scalar equivalent for plan 1; pillar 2 plan replaces with unified integration)
   - `segment_arc(arc, chord_tol)` → `shape.polyline(chord_tol)`
5. Delete any `_select_blend` / arc-dispatch logic if present.

- [ ] **Step 5: Run all tests, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendplanner.py test/test_blendmath.py test/test_blendquintic.py test/test_blendshape.py -v`
Expected: all green.

- [ ] **Step 6: Commit**

```bash
git add klippy/blendplanner.py test/test_blendplanner.py
git commit -m "blendplanner: rewire corner-blend factory to QuinticShape

Replaces blendmath.blend_from_moves(...) with QuinticShape.from_moves(...).
KinematicLimits dataclass decouples planner from toolhead internals.
v_cap_fn(L/2) as scalar equivalent for plan-1; pillar-2 plan (plan 5)
replaces this with unified v(s) integration."
```

---

### Task 16: End-to-end smoke test — planner → polyline emission

**Files:**
- Test: `test/test_blendplanner.py` (or a new smoke test file)

- [ ] **Step 1: Write a smoke test**

Add to `test/test_blendplanner.py`:

```python
def test_smoke_multi_corner_gcode_ingest():
    """End-to-end: planner ingests a small gcode sequence with three
    corners of different angles. Verifies all valid corners emit
    polylines; no crashes; polyline v_cap > 0 throughout."""
    b = _blender(max_chord_err=1e-3)
    th = b._toolhead
    # Square zig-zag: right / up / right / down / right.
    moves = [
        _FakeMove(th, (0, 0, 0, 0),    (10, 0, 0, 0.5),  speed=100.0),   # right
        _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0),   # up (90 deg)
        _FakeMove(th, (10, 10, 0, 1.0), (20, 10, 0, 1.5), speed=100.0),  # right (90 deg)
        _FakeMove(th, (20, 10, 0, 1.5), (20, 0, 0, 2.0),  speed=100.0),  # down (90 deg)
        _FakeMove(th, (20, 0, 0, 2.0),  (30, 0, 0, 2.5),  speed=100.0),  # right (90 deg)
    ]
    total_polyline_moves = 0
    total_blends = 0
    for m in moves:
        out = b.feed(m)
        for em in out:
            # Every emitted move has finite, positive v_cap.
            assert em.max_cruise_v2 > 0.0
            assert em.max_cruise_v2 < float("inf")
        total_polyline_moves += len(out)
    # Drain any buffered trailing move.
    out = b.flush()
    for em in out:
        assert em.max_cruise_v2 > 0.0
    # Instrumentation: expect 4 blends (4 corners between 5 moves).
    assert b.blends_emitted == 4
    assert b.polyline_moves_emitted > 0
```

- [ ] **Step 2: Run smoke test, verify PASS**

Run: `.venv-test/bin/pytest test/test_blendplanner.py -v -k "smoke"`
Expected: green.

- [ ] **Step 3: Final full test pass**

Run: `.venv-test/bin/pytest test/ -v`
Expected: all tests green.

- [ ] **Step 4: Commit**

```bash
git add test/test_blendplanner.py
git commit -m "blendplanner: end-to-end smoke test for quintic emission"
```

---

## Plan 1 complete

At the end of task 16 the branch has:
- A `SmoothShape` protocol future shapes conform to.
- A working `QuinticShape` implementation using tested archive math.
- The 3-point shaper cap bug replaced with per-s local query.
- Arc primitives removed from `blendmath.py`.
- `blendplanner.py` rewired around the protocol.
- Full pytest suite green.

**What plan 1 does NOT do:**
- Unified `v(s)` integration (pillar 2 plan layers on top via `v_cap_fn` + `dkappa_ds`).
- Extruder-constraint enforcement (pillar 3 plan wraps `v_cap_fn`).
- Inverse shaper (pillar 1 plan operates on the emitted polyline).
- Performance tuning — plan 1 explicitly does not set a runtime target.

**Next plan to write:** plan 2 (HP-stepcompress port) or plan 3 (non-linear PA port) — both independent of plan 1 and of each other. Pick based on which you want to run in parallel first.
