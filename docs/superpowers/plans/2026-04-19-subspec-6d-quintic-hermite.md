# Subspec 6d — Quintic Hermite Geometry Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone pure-math module `klippy/blendquintic.py` that computes a G² symmetric quintic Bézier corner blend — analogous in scope to the existing `klippy/blendmath.py` (which handles G¹ arcs) — with a matching test module `test/test_blendquintic.py`. No integration with `CornerBlender` yet; that is sub-spec 6e.

**Architecture:** Pure functional math. A `QuinticBlend` dataclass parallels `BlendArc`. Public entry points `quintic_geometry` and `blend_from_moves_quintic` mirror the arc-module surface. Internals include a De Casteljau evaluator, a dense-sampling peak-curvature routine, a quadratic `r(θ)` shape-parameter fit, a three-point shaper bound evaluator, and adaptive De Casteljau subdivision for polyline output. Shape parameter `r` is angle-dependent (per design spec); no user knob.

**Tech Stack:** Python 3.x, stdlib `math` and `dataclasses` only for the module itself. Tests use `pytest` and `pytest.approx`. No numpy dependency (matches `blendmath.py` style).

**Spec:** `docs/superpowers/specs/2026-04-19-subspec-6d-quintic-hermite-design.md`

---

## File Structure

**Files to create:**
- `klippy/blendquintic.py` — the math module (~350 LOC). Responsibilities: Bézier evaluation, deviation formulas, shape-parameter fit, peak-curvature, velocity caps, polyline sampling, E-axis parameterization, `blend_from_moves_quintic` adapter.
- `test/test_blendquintic.py` — pytest test module (~650 LOC). Mirrors `test/test_blendmath.py` layout.

**Files to modify:**
- `klipper-sim/examples/shape_ceiling.py` — extend existing simulator comparison to assert quintic post-shaper deviation ≤ arc post-shaper deviation (Task 16). (This file lives in a sibling repo; edit only if the repo is accessible, otherwise skip and commit the rest; see Task 16 for gating.)

**No other files modified.** The module is purely additive; `CornerBlender` integration is 6e's scope.

---

## Repo conventions (read before starting)

- **Angle convention:** deflection angle `theta` — `theta = 0` means collinear/straight, `theta = pi` means U-turn. `cos(theta) = prev_dir · next_dir`. Matches `blendmath.py`.
- **Vec3 type alias:** `Vec3 = Tuple[float, float, float]`.
- **Vector helpers:** defined in `blendmath` (`vdot`, `vcross`, `vnorm`, `vscale`, `vadd`, `vsub`, `vnormalize`). Reuse these by importing `from klippy.blendmath import vdot, vcross, ...` to avoid duplication.
- **Dataclasses:** `@dataclass(frozen=True)`.
- **Epsilons:** use `COLLINEAR_EPS = 1e-6` and `REVERSAL_EPS = 1e-6` constants at module top, matching `blendmath`.
- **Commit style:** imperative mood, lowercase prefix (e.g., `blendquintic: add De Casteljau evaluator`). Never add `Co-Authored-By` trailers.
- **Running tests:** `python3 -m pytest test/test_blendquintic.py -v` from repo root. Individual test: `python3 -m pytest test/test_blendquintic.py::test_name -v`.

---

## Task 1: Module scaffold + import smoke test

**Files:**
- Create: `klippy/blendquintic.py`
- Create: `test/test_blendquintic.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_blendquintic.py` with:

```python
# test/test_blendquintic.py
import math

import pytest

from klippy import blendquintic


def test_module_imports():
    assert blendquintic is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_blendquintic.py::test_module_imports -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'klippy.blendquintic'`

- [ ] **Step 3: Write minimal implementation**

Create `klippy/blendquintic.py` with:

```python
# klippy/blendquintic.py
# Quintic Bezier corner-blending geometry module.
#
# Pure-math primitives: given two adjacent linear moves and a
# chord-tolerance parameter, returns a G2 symmetric 6-point quintic
# Bezier blend that smooths the corner, along with the maximum velocity
# it may be traversed at and a fine-segmented polyline approximation.
#
# Analogous to klippy/blendmath.py (which handles G1 arcs).
#
# See docs/superpowers/specs/2026-04-19-subspec-6d-quintic-hermite-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

from klippy import blendshaper
from klippy.blendmath import (
    _extract_shapers,
    vdot,
    vcross,
    vnorm,
    vscale,
    vadd,
    vsub,
    vnormalize,
)

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_blendquintic.py::test_module_imports -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: scaffold module and test file"
```

---

## Task 2: De Casteljau evaluator for position

**Files:**
- Modify: `klippy/blendquintic.py`
- Modify: `test/test_blendquintic.py`

The De Casteljau algorithm evaluates a Bézier curve at parameter `t` by repeated linear interpolation of control points. For a quintic (6 control points), 5 levels of interpolation give a single point. Numerically stable vs. the raw Bernstein formula.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_quintic_eval_at_endpoints_returns_Q0_and_Q5():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (3.0, 1.0, 0.0),
        (4.0, 2.0, 0.0),
        (5.0, 3.0, 0.0),
    ]
    p0 = blendquintic._quintic_eval(Q, 0.0)
    p1 = blendquintic._quintic_eval(Q, 1.0)
    assert p0 == pytest.approx(Q[0])
    assert p1 == pytest.approx(Q[5])


def test_quintic_eval_mid_matches_bernstein_direct():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 2.0, 0.0),
        (2.0, 4.0, 0.0),
        (3.0, 4.0, 1.0),
        (4.0, 2.0, 1.0),
        (5.0, 0.0, 1.0),
    ]
    t = 0.5
    # Binomial(5, i) = 1, 5, 10, 10, 5, 1
    coeffs = [1, 5, 10, 10, 5, 1]
    omt = 1.0 - t
    expected = (0.0, 0.0, 0.0)
    for i, (c, q) in enumerate(zip(coeffs, Q)):
        w = c * (omt ** (5 - i)) * (t ** i)
        expected = (
            expected[0] + w * q[0],
            expected[1] + w * q[1],
            expected[2] + w * q[2],
        )
    got = blendquintic._quintic_eval(Q, t)
    assert got == pytest.approx(expected, abs=1e-12)


def test_quintic_eval_random_t_matches_bernstein_direct():
    # Randomish but deterministic set of t values.
    Q = [
        (0.1, -0.2, 0.3),
        (1.4, 2.5, -0.6),
        (2.7, 4.8, 0.9),
        (3.0, 3.1, 1.2),
        (4.3, 1.4, 1.5),
        (5.6, -0.7, 1.8),
    ]
    coeffs = [1, 5, 10, 10, 5, 1]
    for t in (0.1, 0.25, 0.37, 0.6, 0.8, 0.95):
        omt = 1.0 - t
        expected = (0.0, 0.0, 0.0)
        for i, (c, q) in enumerate(zip(coeffs, Q)):
            w = c * (omt ** (5 - i)) * (t ** i)
            expected = (
                expected[0] + w * q[0],
                expected[1] + w * q[1],
                expected[2] + w * q[2],
            )
        got = blendquintic._quintic_eval(Q, t)
        assert got == pytest.approx(expected, abs=1e-12)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_eval`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_quintic_eval'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    """Linear interpolation between two 3-vectors."""
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _quintic_eval(Q, t: float) -> Vec3:
    """Evaluate a quintic Bezier at parameter t via De Casteljau.

    Q is an indexable of 6 control points Q0..Q5. Returns the
    position on the curve at parameter t in [0, 1].
    """
    p = [Q[i] for i in range(6)]
    for level in range(5, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_eval`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add De Casteljau position evaluator"
```

---

## Task 3: First and second derivatives

For a quintic Bézier B(t), the derivative B'(t) is a quartic Bézier of degree 4 whose 5 control points are `5·(Q[i+1] − Q[i])` for `i = 0..4`. The second derivative B''(t) is a cubic Bézier of degree 3 with control points `20·(Q[i+2] − 2·Q[i+1] + Q[i])` for `i = 0..3`. We evaluate both via the same De Casteljau algorithm applied at the appropriate degree.

**Key property to test:** by construction, `Q1 = Q2` (coincident pair at entry) and `Q3 = Q4` (coincident pair at exit), so B''(0) = 0 and B''(1) = 0 for valid quintic blends — this enforces G² continuity (zero curvature at endpoints).

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_quintic_first_derivative_at_endpoints_matches_tangent():
    # Symmetric blend control points. e1 = (1,0,0), e2 = (0,1,0).
    # V at origin, d = 2, r = 0.6. Coincident pairs enforce G2.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)
    d = 2.0
    r = 0.6
    Q = [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]
    # B'(0) = 5 * (Q1 - Q0) = 5 * d * (1 - r) * e1
    d_at_0 = blendquintic._quintic_first_deriv(Q, 0.0)
    expected_0 = (5.0 * d * (1.0 - r), 0.0, 0.0)
    assert d_at_0 == pytest.approx(expected_0, abs=1e-12)

    # B'(1) = 5 * (Q5 - Q4) = 5 * d * (1 - r) * e2
    d_at_1 = blendquintic._quintic_first_deriv(Q, 1.0)
    expected_1 = (0.0, 5.0 * d * (1.0 - r), 0.0)
    assert d_at_1 == pytest.approx(expected_1, abs=1e-12)


def test_quintic_second_derivative_zero_at_endpoints():
    # G2 property: B''(0) and B''(1) must be zero for the symmetric
    # quintic with coincident control-point pairs at each end.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)
    d = 2.0
    r = 0.6
    Q = [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]
    dd_at_0 = blendquintic._quintic_second_deriv(Q, 0.0)
    dd_at_1 = blendquintic._quintic_second_deriv(Q, 1.0)
    assert dd_at_0 == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert dd_at_1 == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_quintic_derivatives_match_finite_difference():
    # Verify first derivative matches a centered finite difference.
    Q = [
        (0.1, -0.2, 0.3),
        (1.4, 2.5, -0.6),
        (2.7, 4.8, 0.9),
        (3.0, 3.1, 1.2),
        (4.3, 1.4, 1.5),
        (5.6, -0.7, 1.8),
    ]
    h = 1e-6
    for t in (0.2, 0.5, 0.75):
        p_plus = blendquintic._quintic_eval(Q, t + h)
        p_minus = blendquintic._quintic_eval(Q, t - h)
        fd = (
            (p_plus[0] - p_minus[0]) / (2.0 * h),
            (p_plus[1] - p_minus[1]) / (2.0 * h),
            (p_plus[2] - p_minus[2]) / (2.0 * h),
        )
        d1 = blendquintic._quintic_first_deriv(Q, t)
        assert d1 == pytest.approx(fd, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k derivative`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_quintic_first_deriv'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def _bezier_eval_general(P, t: float) -> Vec3:
    """De Casteljau for a Bezier curve of any degree. P is a list of
    n+1 control points (tuples). Returns the point at parameter t."""
    p = [P[i] for i in range(len(P))]
    level = len(p) - 1
    while level > 0:
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
        level -= 1
    return p[0]


def _quintic_first_deriv(Q, t: float) -> Vec3:
    """Evaluate B'(t) for a quintic Bezier at parameter t.

    The derivative of a degree-5 Bezier with control points Q0..Q5
    is a degree-4 Bezier with control points 5*(Q[i+1] - Q[i]).
    """
    D = [
        (
            5.0 * (Q[i + 1][0] - Q[i][0]),
            5.0 * (Q[i + 1][1] - Q[i][1]),
            5.0 * (Q[i + 1][2] - Q[i][2]),
        )
        for i in range(5)
    ]
    return _bezier_eval_general(D, t)


def _quintic_second_deriv(Q, t: float) -> Vec3:
    """Evaluate B''(t) for a quintic Bezier at parameter t.

    The second derivative is a degree-3 Bezier with control points
    20 * (Q[i+2] - 2*Q[i+1] + Q[i]).
    """
    DD = [
        (
            20.0 * (Q[i + 2][0] - 2.0 * Q[i + 1][0] + Q[i][0]),
            20.0 * (Q[i + 2][1] - 2.0 * Q[i + 1][1] + Q[i][1]),
            20.0 * (Q[i + 2][2] - 2.0 * Q[i + 1][2] + Q[i][2]),
        )
        for i in range(4)
    ]
    return _bezier_eval_general(DD, t)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k derivative`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add first and second derivative evaluators"
```

---

## Task 4: Closed-form chord deviation and its inverse

Per the design spec: with V at the corner and unit tangents e1 (entry) and e2 (exit) at deflection angle θ:

```
deviation(d, r, θ) = ((1 + 15·r) / 16) · d · sin(θ/2)

d(ε, r, θ) = 16·ε / ((1 + 15·r) · sin(θ/2))
```

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def _build_symmetric_Q(V, e1, e2, d, r):
    return [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]


def test_deviation_matches_closed_form_at_r_four_fifths():
    # Known sanity check: r = 0.8, coefficient (1 + 15*0.8)/16 = 13/16
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)  # theta = pi/2, sin(theta/2) = sin(pi/4) = sqrt(2)/2
    d = 1.0
    r = 0.8
    sin_half = math.sin(math.pi / 4.0)
    expected = (13.0 / 16.0) * d * sin_half

    Q = _build_symmetric_Q(V, e1, e2, d, r)
    B_mid = blendquintic._quintic_eval(Q, 0.5)
    got_numerical = math.sqrt(
        B_mid[0] ** 2 + B_mid[1] ** 2 + B_mid[2] ** 2
    )
    got_closed = blendquintic._deviation_closed_form(d, r, sin_half)
    assert got_numerical == pytest.approx(expected, abs=1e-12)
    assert got_closed == pytest.approx(expected, abs=1e-12)


def test_deviation_closed_form_matches_numerical_across_r_and_theta():
    # Sweep (theta, r) and verify closed-form matches Bezier evaluation.
    V = (0.0, 0.0, 0.0)
    for theta in (0.2, 0.5, 1.0, 1.5708, 2.3, 2.9):
        e1 = (1.0, 0.0, 0.0)
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        sin_half = math.sin(theta / 2.0)
        for r in (0.3, 0.5, 0.7, 0.85):
            d = 1.5
            Q = _build_symmetric_Q(V, e1, e2, d, r)
            B_mid = blendquintic._quintic_eval(Q, 0.5)
            dev_numerical = math.sqrt(sum(c * c for c in B_mid))
            dev_closed = blendquintic._deviation_closed_form(d, r, sin_half)
            assert dev_numerical == pytest.approx(dev_closed, abs=1e-10)


def test_d_from_deviation_is_inverse_of_deviation():
    # Pick (r, theta, eps), compute d, rebuild Q, confirm deviation matches.
    V = (0.0, 0.0, 0.0)
    for theta in (0.4, 1.0, 2.0, 2.6):
        e1 = (1.0, 0.0, 0.0)
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        sin_half = math.sin(theta / 2.0)
        for r in (0.50, 0.65, 0.80):
            for eps in (0.05, 0.2, 0.5):
                d = blendquintic._d_from_deviation(eps, r, sin_half)
                Q = _build_symmetric_Q(V, e1, e2, d, r)
                B_mid = blendquintic._quintic_eval(Q, 0.5)
                dev = math.sqrt(sum(c * c for c in B_mid))
                assert dev == pytest.approx(eps, abs=1e-10)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k deviation`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_deviation_closed_form'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def _deviation_coeff(r: float) -> float:
    """The chord-deviation prefactor (1 + 15*r)/16."""
    return (1.0 + 15.0 * r) / 16.0


def _deviation_closed_form(d: float, r: float, sin_half: float) -> float:
    """Chord deviation of a symmetric quintic blend, closed form.

    At the midpoint t=0.5:
        |B(0.5) - V| = ((1 + 15*r) / 16) * d * sin(theta/2)
    """
    return _deviation_coeff(r) * d * sin_half


def _d_from_deviation(eps: float, r: float, sin_half: float) -> float:
    """Inverse: tangent length d required to achieve chord deviation eps.

    d = 16 * eps / ((1 + 15*r) * sin(theta/2))
    """
    denom = (1.0 + 15.0 * r) * sin_half
    if denom <= 0.0:
        raise ValueError("_d_from_deviation: non-positive denominator")
    return 16.0 * eps / denom
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k deviation`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add closed-form chord deviation and inverse"
```

---

## Task 5: Curvature at a parameter value

Curvature of a 3D curve at parameter t is `κ(t) = |B'(t) × B''(t)| / |B'(t)|³`. For planar corners the cross-product magnitude reduces to a scalar, but the 3D form handles arbitrary orientations (including axis-rotated blends).

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_curvature_zero_at_endpoints_of_symmetric_blend():
    # By construction: Q1 = Q2 and Q3 = Q4 force B''(0) = B''(1) = 0.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (math.cos(1.2), math.sin(1.2), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.5, r=0.6)
    assert blendquintic._curvature_at(Q, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert blendquintic._curvature_at(Q, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_curvature_matches_finite_difference_reference():
    # Reference: kappa(t) from centered finite differences of |B'|^3 and B'xB''.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.6)
    h = 1e-5
    for t in (0.25, 0.5, 0.75):
        # Finite-difference second derivative of position.
        p_plus = blendquintic._quintic_eval(Q, t + h)
        p_0 = blendquintic._quintic_eval(Q, t)
        p_minus = blendquintic._quintic_eval(Q, t - h)
        fd_first = (
            (p_plus[0] - p_minus[0]) / (2.0 * h),
            (p_plus[1] - p_minus[1]) / (2.0 * h),
            (p_plus[2] - p_minus[2]) / (2.0 * h),
        )
        fd_second = (
            (p_plus[0] - 2.0 * p_0[0] + p_minus[0]) / (h * h),
            (p_plus[1] - 2.0 * p_0[1] + p_minus[1]) / (h * h),
            (p_plus[2] - 2.0 * p_0[2] + p_minus[2]) / (h * h),
        )
        cross = (
            fd_first[1] * fd_second[2] - fd_first[2] * fd_second[1],
            fd_first[2] * fd_second[0] - fd_first[0] * fd_second[2],
            fd_first[0] * fd_second[1] - fd_first[1] * fd_second[0],
        )
        cross_norm = math.sqrt(sum(c * c for c in cross))
        first_norm = math.sqrt(sum(c * c for c in fd_first))
        fd_kappa = cross_norm / (first_norm ** 3)

        kappa = blendquintic._curvature_at(Q, t)
        assert kappa == pytest.approx(fd_kappa, rel=1e-3)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k curvature`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_curvature_at'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def _curvature_at(Q, t: float) -> float:
    """Curvature of the quintic at parameter t.

    kappa(t) = |B'(t) x B''(t)| / |B'(t)|^3
    Returns 0.0 if |B'(t)| is near zero (degenerate endpoint with
    coincident control points — expected at t=0 and t=1 for symmetric
    quintic blends).
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    d1_norm = vnorm(d1)
    if d1_norm < 1e-12:
        return 0.0
    cx = d1[1] * d2[2] - d1[2] * d2[1]
    cy = d1[2] * d2[0] - d1[0] * d2[2]
    cz = d1[0] * d2[1] - d1[1] * d2[0]
    cross_norm = math.sqrt(cx * cx + cy * cy + cz * cz)
    return cross_norm / (d1_norm ** 3)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k curvature`
Expected: PASS on both tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add per-parameter curvature evaluator"
```

---

## Task 6: Peak-curvature evaluator via dense sampling

Per the design spec: the peak curvature of a symmetric quintic blend (for the shape-ratio range in use) is not at the midpoint in general — it's off-center and the location is the root of a degree-7 polynomial with no closed form. Use dense sampling (20 interior points plus the two endpoints, total 22 samples), return the maximum, plus the t-value at which it occurred.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_peak_curvature_exceeds_midpoint_for_large_r():
    # For r > ~0.3 at non-shallow angles, the true peak is off-center.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.8)
    kappa_peak, t_peak = blendquintic._peak_curvature(Q)
    kappa_mid = blendquintic._curvature_at(Q, 0.5)
    assert kappa_peak > kappa_mid * 1.5
    # Peak should be off-center (not at t=0.5 for this r).
    assert abs(t_peak - 0.5) > 1e-3


def test_peak_curvature_at_midpoint_for_small_r():
    # For r <= ~0.3 the peak is at the midpoint by symmetry.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.2)
    kappa_peak, t_peak = blendquintic._peak_curvature(Q)
    kappa_mid = blendquintic._curvature_at(Q, 0.5)
    assert kappa_peak == pytest.approx(kappa_mid, rel=1e-2)
    assert t_peak == pytest.approx(0.5, abs=0.05)


def test_peak_curvature_matches_dense_reference():
    # The implementation samples 22 points; reference samples 2001.
    # Both should agree within 1%.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    for theta, r in [(0.5, 0.50), (1.0, 0.55), (math.pi / 2, 0.6), (2.5, 0.85)]:
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=r)
        kappa_peak, _ = blendquintic._peak_curvature(Q)
        reference_samples = 2001
        ref_max = 0.0
        for i in range(reference_samples):
            t = i / (reference_samples - 1)
            k = blendquintic._curvature_at(Q, t)
            if k > ref_max:
                ref_max = k
        assert kappa_peak == pytest.approx(ref_max, rel=1e-2)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k peak_curvature`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_peak_curvature'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
_PEAK_KAPPA_SAMPLES = 22  # dense-sample count for peak-curvature search


def _peak_curvature(Q) -> Tuple[float, float]:
    """Return (kappa_max, t_peak) along the quintic.

    Dense sampling at _PEAK_KAPPA_SAMPLES points; returns the maximum
    curvature along the blend and the parameter value where it occurs.
    Endpoints always have kappa = 0 for a symmetric blend, so they are
    included but will not normally win.
    """
    best_k = 0.0
    best_t = 0.5
    for i in range(_PEAK_KAPPA_SAMPLES):
        t = i / (_PEAK_KAPPA_SAMPLES - 1)
        k = _curvature_at(Q, t)
        if k > best_k:
            best_k = k
            best_t = t
    return best_k, best_t
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k peak_curvature`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add dense-sampling peak-curvature evaluator"
```

---

## Task 7: Shape parameter `r(θ)` with safety clamp

Per the design spec: `r(θ) = 0.5085 − 0.03785·θ + 0.05715·θ²` (deflection radians), clamped to `[0.50, 0.86]`. Outside the validity window (`θ < 10°` or `θ > 160°`) the clamp still applies; 6e decides whether to route elsewhere.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_shape_ratio_matches_reference_anchors():
    # Anchor points from the subagent's per-angle optimum table
    # (interior-angle convention converted to deflection).
    # interior 90 deg -> deflection pi/2 -> r = 0.5900
    # interior 60 deg -> deflection 2*pi/3 -> r = 0.6800
    # interior 150 deg -> deflection pi/6 -> r = 0.5044
    r_shallow = blendquintic._shape_ratio(math.radians(30))
    r_mid = blendquintic._shape_ratio(math.radians(90))
    r_wide = blendquintic._shape_ratio(math.radians(120))
    assert r_shallow == pytest.approx(0.5044, abs=0.01)
    assert r_mid == pytest.approx(0.5900, abs=0.01)
    assert r_wide == pytest.approx(0.6800, abs=0.01)


def test_shape_ratio_clamps_to_valid_range():
    # Below clamp floor: 0 rad would give 0.5085, clamped to 0.50.
    # But the formula floor at theta=0 is already >= 0.50, so test
    # extreme small theta doesn't go below 0.50 due to numerical noise.
    r0 = blendquintic._shape_ratio(0.0)
    assert 0.50 <= r0 <= 0.86

    # Far beyond the validity window (theta = pi): r formula -> 0.9539
    # clamped to 0.86.
    r_big = blendquintic._shape_ratio(math.pi)
    assert r_big == 0.86


def test_shape_ratio_monotone_increasing_in_theta():
    # r(theta) should be strictly increasing across the validity window.
    prev = blendquintic._shape_ratio(math.radians(15))
    for deg in range(20, 165, 5):
        r = blendquintic._shape_ratio(math.radians(deg))
        assert r >= prev
        prev = r
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k shape_ratio`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute '_shape_ratio'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
# Quadratic fit coefficients for the minimum-traversal-time shape
# parameter r as a function of the deflection angle theta (radians).
# Derived from the 151-angle x 3-deviation subagent sweep dated
# 2026-04-19; see subspec-6d design spec section "Shape parameter
# r(theta)". Worst-case traversal-time penalty vs the per-angle
# optimum is 0.21% at theta ~ 10 deg (near the validity edge).
_R_FIT_C0 = 0.5085
_R_FIT_C1 = -0.03785
_R_FIT_C2 = 0.05715

_R_CLAMP_MIN = 0.50
_R_CLAMP_MAX = 0.86


def _shape_ratio(theta: float) -> float:
    """Shape parameter r for the quintic blend at deflection theta.

    Quadratic fit (radians) clamped to the [0.50, 0.86] safety range.
    Returns the r value used to place the inner control-point pair at
    +/- r*d along each tangent ray.
    """
    r = _R_FIT_C0 + _R_FIT_C1 * theta + _R_FIT_C2 * theta * theta
    if r < _R_CLAMP_MIN:
        return _R_CLAMP_MIN
    if r > _R_CLAMP_MAX:
        return _R_CLAMP_MAX
    return r
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k shape_ratio`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add r(theta) quadratic fit with safety clamp"
```

---

## Task 8: `QuinticBlend` dataclass

Mirrors `BlendArc` but with quintic-specific state: the six control points, the shape parameter used, peak curvature, arc length (computed later), and the velocity cap.

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendquintic.py`:

```python
def test_quintic_blend_dataclass_fields():
    q = blendquintic.QuinticBlend(
        Q=(
            (-1.0, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (0.0, 0.5, 0.0),
            (0.0, 0.5, 0.0),
            (0.0, 1.0, 0.0),
        ),
        theta=math.pi / 2.0,
        r=0.5900,
        d_consumed=1.0,
        kappa_peak=0.5,
        v_cap=100.0,
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    assert len(q.Q) == 6
    assert q.theta == pytest.approx(math.pi / 2.0)
    assert q.r == pytest.approx(0.5900)
    assert q.d_consumed == 1.0
    assert q.kappa_peak == 0.5
    assert q.v_cap == 100.0
    assert q.plane_normal == (0.0, 0.0, 1.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_blendquintic.py::test_quintic_blend_dataclass_fields -v`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'QuinticBlend'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
@dataclass(frozen=True)
class QuinticBlend:
    """Symmetric quintic Bezier blend between two adjacent moves.

    Coordinates: ``Q`` control points and ``entry_pt``/``exit_pt``
    (derivable as ``Q[0]`` and ``Q[5]``) are in a corner-local frame
    where the corner vertex is at the origin. Callers must translate
    by the vertex position to obtain world coordinates.

    For degenerate corners (returned for U-turns), ``kappa_peak`` is
    0.0, ``v_cap`` is 0.0, and ``Q`` is six copies of (0, 0, 0).

    Fields:
        Q:              6 control points (Q0..Q5)
        theta:          deflection angle (rad), 0 = straight, pi = U-turn
        r:              shape parameter used (in [0.50, 0.86])
        d_consumed:     tangent length along each ray (mm)
        kappa_peak:     maximum curvature along the blend (1/mm)
        v_cap:          maximum traversal velocity (mm/s)
        entry_tangent:  unit vector, same as prev_dir into corner
        exit_tangent:   unit vector, same as next_dir out of corner
        plane_normal:   unit vector orthogonal to the blend plane
    """
    Q: Tuple[Vec3, Vec3, Vec3, Vec3, Vec3, Vec3]
    theta: float
    r: float
    d_consumed: float
    kappa_peak: float
    v_cap: float
    entry_tangent: Vec3
    exit_tangent: Vec3
    plane_normal: Vec3
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_blendquintic.py::test_quintic_blend_dataclass_fields -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add QuinticBlend dataclass"
```

---

## Task 9: `quintic_geometry` — core with degenerate cases, half-segment cap, centripetal velocity

Skeleton `quintic_geometry(prev_dir, next_dir, L_prev, L_next, corner_deviation, a_max)` that:
1. Computes θ via the same dot/sin_half/cos_half setup `blendmath.blend_geometry` uses.
2. Returns `None` if collinear (below COLLINEAR_EPS).
3. Returns a zero-R-equivalent degenerate `QuinticBlend` for U-turn (matches `blendmath`'s behavior).
4. Computes `r = _shape_ratio(theta)`.
5. Computes `d_tol = _d_from_deviation(corner_deviation, r, sin_half)`.
6. Computes the half-segment cap `d_mid = 0.5 * min(L_prev, L_next)`.
7. Uses `d = min(d_tol, d_mid)`.
8. Builds `Q` control points from the corner frame, evaluates `kappa_peak`.
9. Centripetal velocity cap: `v_cent = sqrt(a_max / kappa_peak)`.

Shaper and jerk bounds are added in later tasks.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_quintic_geometry_collinear_returns_none():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (1.0, 0.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
    )
    assert result is None


def test_quintic_geometry_u_turn_returns_degenerate():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
    )
    assert result is not None
    assert result.v_cap == 0.0
    assert result.d_consumed == 0.0


def test_quintic_geometry_right_angle_basic():
    # 90 deg corner, 1 mm segments, eps = 0.1 mm. Expect a QuinticBlend
    # with theta = pi/2, r matches _shape_ratio(pi/2), d > 0, kappa_peak > 0.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    assert result is not None
    assert result.theta == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert result.r == pytest.approx(blendquintic._shape_ratio(math.pi / 2.0))
    assert result.d_consumed > 0.0
    assert result.kappa_peak > 0.0
    # Centripetal-only cap: v_cap^2 <= a_max / kappa_peak
    assert result.v_cap * result.v_cap == pytest.approx(
        50000.0 / result.kappa_peak, rel=1e-9
    )
    assert result.plane_normal == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)


def test_quintic_geometry_half_segment_cap_limits_d():
    # Very loose corner_deviation should hit the L/2 cap.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=2.0,
        L_next=4.0,
        corner_deviation=5.0,  # would demand d much bigger than L_prev/2
        a_max=50000.0,
    )
    assert result is not None
    assert result.d_consumed == pytest.approx(1.0, abs=1e-12)  # 0.5 * min(2, 4)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_geometry`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'quintic_geometry'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def quintic_geometry(
    prev_dir: Vec3,
    next_dir: Vec3,
    L_prev: float,
    L_next: float,
    corner_deviation: float,
    a_max: float,
) -> Optional[QuinticBlend]:
    """Compute the quintic Bezier blend for a corner, or None if no
    blend is needed. Centripetal bound only — shaper and rotation-jerk
    bounds are added by `quintic_geometry_with_shaper` / `blend_from_
    moves_quintic`.

    ``prev_dir`` and ``next_dir`` must be unit vectors. Angle
    convention: deflection theta, where 0 = collinear, pi = U-turn.
    """
    dp = vdot(prev_dir, next_dir)
    dp = max(-1.0, min(1.0, dp))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))

    if sin_half < COLLINEAR_EPS:
        return None

    if cos_half < REVERSAL_EPS:
        return QuinticBlend(
            Q=((0.0, 0.0, 0.0),) * 6,
            theta=math.pi,
            r=_R_CLAMP_MIN,
            d_consumed=0.0,
            kappa_peak=0.0,
            v_cap=0.0,
            entry_tangent=prev_dir,
            exit_tangent=next_dir,
            plane_normal=(0.0, 0.0, 0.0),
        )

    theta = 2.0 * math.atan2(sin_half, cos_half)
    r = _shape_ratio(theta)

    d_tol = _d_from_deviation(corner_deviation, r, sin_half)
    d_mid = 0.5 * min(L_prev, L_next)
    d = min(d_tol, d_mid)

    # Build the 6 control points relative to the corner vertex (V at origin).
    # Q0 = -d * prev_dir, Q5 = +d * next_dir.
    Q = (
        vscale(prev_dir, -d),
        vscale(prev_dir, -r * d),
        vscale(prev_dir, -r * d),
        vscale(next_dir, r * d),
        vscale(next_dir, r * d),
        vscale(next_dir, d),
    )

    kappa_peak, _ = _peak_curvature(Q)

    # Plane normal: right-handed, consistent with blendmath.
    raw_normal = vcross(prev_dir, next_dir)
    raw_norm_n = vnorm(raw_normal)
    if raw_norm_n == 0.0:
        plane_normal: Vec3 = (0.0, 0.0, 0.0)
    else:
        plane_normal = vscale(raw_normal, 1.0 / raw_norm_n)

    # Centripetal bound.
    if kappa_peak > 0.0:
        v_cent = math.sqrt(a_max / kappa_peak)
    else:
        v_cent = 0.0
    v_cap = v_cent

    return QuinticBlend(
        Q=Q,
        theta=theta,
        r=r,
        d_consumed=d,
        kappa_peak=kappa_peak,
        v_cap=v_cap,
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=plane_normal,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_geometry`
Expected: PASS on all four tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add quintic_geometry with centripetal cap"
```

---

## Task 10: Three-point shaper velocity bound

Per the design spec: evaluate `blendshaper.compute_shaper_bounds` at three parameter values `t ∈ {0.25, 0.5, 0.75}` and take the minimum. At each t we need the local curvature (so `R_loc = 1/κ(t)`) and the local inward normal `n̂(t)`.

Helper `_point_frame(Q, t)` returns `(R_loc, n_hat_local)` for a given parameter. Caller passes shaper snapshots; integration with `quintic_geometry` happens via a new function `quintic_geometry_with_shaper` that takes the base geometry and an iterable of `AxisShaperSnapshot`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
from klippy.blendshaper import AxisShaperSnapshot


def test_quintic_geometry_with_shaper_bound_matches_centripetal_when_no_shapers():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=float("inf"),
    )
    assert capped.v_cap == pytest.approx(base.v_cap)


def test_quintic_geometry_with_shaper_bound_tightens_v_cap():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    shapers = [
        AxisShaperSnapshot(
            axis="x",
            shaper_type="mzv",
            shaper_freq=60.0,
            damping_ratio=0.1,
            A_axis=10000.0,
        ),
        AxisShaperSnapshot(
            axis="y",
            shaper_type="mzv",
            shaper_freq=60.0,
            damping_ratio=0.1,
            A_axis=10000.0,
        ),
    ]
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=shapers,
        j_eff=float("inf"),
    )
    assert capped.v_cap <= base.v_cap + 1e-9


def test_quintic_shaper_bound_is_min_across_three_samples():
    # With axis-rotated bisector, single-point evaluation at t=0.5 can
    # overshoot true min; three-point min should be tighter than mid-only.
    prev_dir = (1.0, 0.0, 0.0)
    # 30 deg interior -> deflection 150 deg; rotated 45 deg about z axis
    theta_defl = math.radians(150)
    angle_rot = math.radians(45)
    cos_r = math.cos(angle_rot)
    sin_r = math.sin(angle_rot)
    # Rotate both prev and next so the bisector is NOT aligned with an axis.
    prev_dir = (cos_r, sin_r, 0.0)
    # next_dir = R_rot(Rot_defl(prev_dir))
    tx = math.cos(theta_defl) * cos_r - math.sin(theta_defl) * sin_r
    ty = math.sin(theta_defl) * cos_r + math.cos(theta_defl) * sin_r
    next_dir = (tx, ty, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.2,
        a_max=45000.0,
    )
    shapers = [
        AxisShaperSnapshot(
            axis="x", shaper_type="mzv", shaper_freq=60.0,
            damping_ratio=0.1, A_axis=10000.0,
        ),
        AxisShaperSnapshot(
            axis="y", shaper_type="mzv", shaper_freq=60.0,
            damping_ratio=0.1, A_axis=10000.0,
        ),
    ]
    three_point = blendquintic.quintic_geometry_with_shaper(
        base=base, shapers=shapers, j_eff=float("inf"),
    )
    # Spot-check dense min over 100+ points along the blend.
    dense = blendquintic._dense_shaper_cap(base, shapers, samples=101)
    # Three-point min should not exceed the dense-sampled min by more
    # than a small margin (the spec target is ~6% worst-case overshoot).
    assert three_point.v_cap <= dense * 1.10 + 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_geometry_with_shaper`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'quintic_geometry_with_shaper'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
_SHAPER_SAMPLE_TS = (0.25, 0.5, 0.75)


def _point_frame(Q, t: float) -> Tuple[float, Vec3, Vec3]:
    """Return (R_loc, tangent_hat, normal_hat) at parameter t.

    R_loc = 1 / kappa(t). If the local curvature is near zero (endpoints
    or a nearly-flat stretch), R_loc is +inf — use sparingly. tangent
    and normal are unit vectors in 3D.
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    d1_norm = vnorm(d1)
    if d1_norm < 1e-12:
        return float("inf"), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    tangent = vscale(d1, 1.0 / d1_norm)
    # Kappa = |d1 x d2| / |d1|^3
    cross = vcross(d1, d2)
    cross_norm = vnorm(cross)
    if cross_norm < 1e-12:
        return float("inf"), tangent, (0.0, 0.0, 0.0)
    kappa = cross_norm / (d1_norm ** 3)
    R_loc = 1.0 / kappa
    # Principal normal direction: component of d2 perpendicular to tangent.
    # N = (d2 - (d2 . tangent) * tangent) normalized.
    dot_d2_t = vdot(d2, tangent)
    perp = vsub(d2, vscale(tangent, dot_d2_t))
    perp_norm = vnorm(perp)
    if perp_norm < 1e-12:
        return R_loc, tangent, (0.0, 0.0, 0.0)
    normal = vscale(perp, 1.0 / perp_norm)
    return R_loc, tangent, normal


def _three_point_shaper_cap(blend: "QuinticBlend", shapers) -> float:
    """Minimum of the shaper entry-step velocity cap evaluated at
    t in {0.25, 0.5, 0.75}. Returns +inf if no shapers or no bound."""
    if not shapers:
        return float("inf")
    p_hat = blend.plane_normal
    cap = float("inf")
    for t in _SHAPER_SAMPLE_TS:
        R_loc, _tangent, normal = _point_frame(blend.Q, t)
        if R_loc == float("inf"):
            continue
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_loc,
            n_hat=normal,
            p_hat=p_hat,
        )
        if bounds.v_step_cap < cap:
            cap = bounds.v_step_cap
    return cap


def _dense_shaper_cap(blend: "QuinticBlend", shapers, samples: int = 101) -> float:
    """Reference: dense-sample minimum shaper cap along the blend.

    Used by tests to verify the three-point approximation is close to
    the true minimum. NOT called in production planning paths.
    """
    if not shapers:
        return float("inf")
    cap = float("inf")
    p_hat = blend.plane_normal
    for i in range(samples):
        t = i / (samples - 1)
        R_loc, _tangent, normal = _point_frame(blend.Q, t)
        if R_loc == float("inf"):
            continue
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_loc,
            n_hat=normal,
            p_hat=p_hat,
        )
        if bounds.v_step_cap < cap:
            cap = bounds.v_step_cap
    return cap


def quintic_geometry_with_shaper(
    base: Optional[QuinticBlend],
    shapers,
    j_eff: float,
) -> Optional[QuinticBlend]:
    """Apply shaper bounds on top of a base `QuinticBlend`.

    Tightens v_cap by the min of the three-point shaper bound. Does
    not yet include rotation-jerk bound (added in a subsequent task).
    """
    if base is None:
        return None
    if base.d_consumed == 0.0:
        return base  # degenerate U-turn
    v_shaper = _three_point_shaper_cap(base, shapers)
    v_cap = min(base.v_cap, v_shaper)
    return replace(base, v_cap=v_cap)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k quintic_geometry_with_shaper`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add three-point shaper velocity bound"
```

---

## Task 11: Rotation-jerk velocity bound

Per the design spec: the rotation-jerk cap for a curved path uses `v_jerk = (R * sqrt(j_eff))^(2/3)` evaluated at the peak-curvature point (since that's where the effective rotation rate `v · κ` peaks for a velocity-capped blend). This is the same formula blendmath uses for the arc; the only difference is that `R = 1 / kappa_peak` instead of the constant arc radius.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_rotation_jerk_cap_applied():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    # Very small j_eff: rotation-jerk should dominate v_cap.
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=1e4,
    )
    R_peak = 1.0 / base.kappa_peak
    v_jerk_expected = (R_peak * math.sqrt(1e4)) ** (2.0 / 3.0)
    # v_cap = min(v_cent, v_jerk). v_jerk is smaller at j_eff=1e4.
    assert capped.v_cap == pytest.approx(min(base.v_cap, v_jerk_expected), rel=1e-9)


def test_rotation_jerk_infinite_does_not_affect_v_cap():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.1, a_max=50000.0,
    )
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=float("inf"),
    )
    assert capped.v_cap == pytest.approx(base.v_cap)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k rotation_jerk`
Expected: FAIL — the test expects `j_eff` to bound v_cap but the current implementation ignores `j_eff`.

- [ ] **Step 3: Write the implementation**

In `klippy/blendquintic.py`, modify `quintic_geometry_with_shaper`:

```python
def quintic_geometry_with_shaper(
    base: Optional[QuinticBlend],
    shapers,
    j_eff: float,
) -> Optional[QuinticBlend]:
    """Apply shaper + rotation-jerk bounds on top of a base `QuinticBlend`.

    Tightens v_cap by:
      - the three-point shaper entry-step bound, and
      - the rotation-jerk bound v_jerk = (R_peak * sqrt(j_eff))^(2/3)
        evaluated at the peak-curvature point.

    Pass j_eff = +inf to disable the rotation-jerk bound (useful for
    non-shaper callers and tests).
    """
    if base is None:
        return None
    if base.d_consumed == 0.0:
        return base

    v_shaper = _three_point_shaper_cap(base, shapers)

    if base.kappa_peak > 0.0 and j_eff > 0.0 and j_eff != float("inf"):
        R_peak = 1.0 / base.kappa_peak
        v_jerk = (R_peak * math.sqrt(j_eff)) ** (2.0 / 3.0)
    else:
        v_jerk = float("inf")

    v_cap = min(base.v_cap, v_shaper, v_jerk)
    return replace(base, v_cap=v_cap)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k rotation_jerk`
Expected: PASS on both tests. Also re-run `test_quintic_geometry_with_shaper` tests to confirm no regressions:

```bash
python3 -m pytest test/test_blendquintic.py -v
```

Expected: all prior tests still pass.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add rotation-jerk velocity bound"
```

---

## Task 12: `blend_from_moves_quintic` adapter

Mirrors `blendmath.blend_from_moves`: takes Kalico `Move`-like objects and an optional `toolhead`, extracts per-axis shaper snapshots if the toolhead is present, runs `quintic_geometry` and then `quintic_geometry_with_shaper`. Skips non-kinematic moves.

Note: the existing `blendmath._extract_shapers` helper is reusable — import it rather than duplicating.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
class _FakeMove:
    """Minimal Move stub for pure-math tests."""

    def __init__(self, axes_r, move_d, accel, is_kinematic=True):
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
        self.is_kinematic_move = is_kinematic


def test_blend_from_moves_quintic_skips_non_kinematic():
    prev = _FakeMove((1.0, 0.0, 0.0, 0.0), 10.0, 50000.0, is_kinematic=False)
    nxt = _FakeMove((0.0, 1.0, 0.0, 0.0), 10.0, 50000.0, is_kinematic=True)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is None


def test_blend_from_moves_quintic_returns_blend_for_right_angle():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 50000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 50000.0)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is not None
    assert result.theta == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert result.d_consumed > 0.0


def test_blend_from_moves_quintic_uses_stricter_accel():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 30000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 70000.0)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is not None
    # v_cent^2 = a_max / kappa_peak with a_max = min(30000, 70000) = 30000
    assert result.v_cap * result.v_cap == pytest.approx(
        30000.0 / result.kappa_peak, rel=1e-9
    )


def test_blend_from_moves_quintic_rejects_both_j_eff_and_toolhead():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 50000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 50000.0)

    class _DummyToolhead:
        pass

    with pytest.raises(ValueError):
        blendquintic.blend_from_moves_quintic(
            prev, nxt, 0.1, j_eff=1e7, toolhead=_DummyToolhead(),
        )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k blend_from_moves`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'blend_from_moves_quintic'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py` (note: `_extract_shapers` was already imported in Task 1):

```python
def blend_from_moves_quintic(
    prev_move,
    next_move,
    corner_deviation: float,
    j_eff: float = float("inf"),
    toolhead=None,
) -> Optional[QuinticBlend]:
    """Adapter: compute a quintic blend from a pair of Kalico Move-like
    objects. Mirrors `blendmath.blend_from_moves`.

    Skips the blend if either move is non-kinematic (E-only). The
    effective a_max is the stricter of the two moves' accel values.

    If `toolhead` is given, derives `j_eff` and the shaper velocity
    bound from the toolhead's input shaper module. In that case any
    explicit `j_eff` argument is ignored.
    """
    if toolhead is not None and j_eff != float("inf"):
        raise ValueError(
            "blend_from_moves_quintic: j_eff and toolhead are mutually "
            "exclusive (toolhead derives j_eff from shaper state; "
            "passing both is ambiguous)"
        )
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

    base = quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=prev_move.move_d,
        L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max,
    )
    if base is None or base.d_consumed == 0.0:
        return base

    if toolhead is None:
        return quintic_geometry_with_shaper(
            base=base, shapers=[], j_eff=j_eff,
        )

    shapers = _extract_shapers(toolhead)

    # First pass: derive j_eff from shaper state using the peak-curvature
    # radius as the arc-like input (blendshaper.compute_shaper_bounds
    # expects an R; use R = 1/kappa_peak).
    if base.kappa_peak > 0.0 and shapers:
        R_peak = 1.0 / base.kappa_peak
        # Use the normal at the peak-curvature point for j_eff derivation.
        _, _, n_peak = _point_frame(base.Q, 0.5)  # midpoint normal proxy
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_peak,
            n_hat=n_peak,
            p_hat=base.plane_normal,
        )
        derived_j = bounds.j_eff
    else:
        derived_j = float("inf")

    return quintic_geometry_with_shaper(
        base=base, shapers=shapers, j_eff=derived_j,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k blend_from_moves`
Expected: PASS on all four tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add blend_from_moves_quintic adapter"
```

---

## Task 13: Adaptive De Casteljau polyline subdivision

Return a polyline (list of points) approximating the quintic with max chord error `≤ max_chord_err`. Use recursive De Casteljau subdivision at `t = 0.5`: compute a flatness metric, subdivide if too curvy, emit a segment if flat enough.

Flatness metric: max perpendicular distance from control points `Q1..Q4` to the chord `Q0–Q5`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_segment_quintic_max_chord_error_bound():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    tol = 0.005
    poly = blendquintic.segment_quintic(q, max_chord_err=tol)
    # Every consecutive pair is a segment: no point on the curve
    # should be farther from its chord than tol.
    # Sample many reference points along the curve and for each, find
    # the closest chord; check distance <= tol + slack.
    ref_samples = 201
    ref_pts = [blendquintic._quintic_eval(q.Q, i / (ref_samples - 1))
               for i in range(ref_samples)]
    # For each reference point, find the min distance to any chord of
    # the polyline. This is quadratic and fine for a unit test.

    def _point_chord_dist(p, a, b):
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
        len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
        if len2 == 0:
            dx = p[0] - a[0]; dy = p[1] - a[1]; dz = p[2] - a[2]
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        tt = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2
        tt = max(0.0, min(1.0, tt))
        proj = (a[0] + ab[0] * tt, a[1] + ab[1] * tt, a[2] + ab[2] * tt)
        dx = p[0] - proj[0]; dy = p[1] - proj[1]; dz = p[2] - proj[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    for ref in ref_pts:
        best = min(
            _point_chord_dist(ref, poly[i], poly[i + 1])
            for i in range(len(poly) - 1)
        )
        assert best <= tol * 1.5  # slack for sampling density


def test_segment_quintic_emits_ordered_polyline():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.01)
    assert len(poly) >= 3
    assert poly[0] == pytest.approx(q.Q[0])
    assert poly[-1] == pytest.approx(q.Q[5])


def test_segment_quintic_degenerate_returns_single_point():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.01)
    assert poly == [(0.0, 0.0, 0.0)]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k segment_quintic`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'segment_quintic'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
_SUBDIVIDE_MAX_DEPTH = 12


def _perp_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    """Perpendicular distance from p to the infinite line through a and b."""
    ab = vsub(b, a)
    ab_len = vnorm(ab)
    if ab_len < 1e-12:
        return vnorm(vsub(p, a))
    ap = vsub(p, a)
    cross = vcross(ab, ap)
    return vnorm(cross) / ab_len


def _quintic_flatness(Q) -> float:
    """Max perpendicular distance of Q1..Q4 from the chord Q0-Q5."""
    chord_a = Q[0]
    chord_b = Q[5]
    return max(
        _perp_distance(Q[1], chord_a, chord_b),
        _perp_distance(Q[2], chord_a, chord_b),
        _perp_distance(Q[3], chord_a, chord_b),
        _perp_distance(Q[4], chord_a, chord_b),
    )


def _quintic_split(Q):
    """Split quintic at t=0.5 via De Casteljau, return (left, right)
    control-point tuples each with 6 points."""
    # Level 0 -> 5 by repeated lerp at t=0.5. Capture outer points at each level.
    p = [Q[i] for i in range(6)]
    left = [p[0]]
    right = [p[5]]
    for level in range(5, 0, -1):
        new_p = [_lerp(p[i], p[i + 1], 0.5) for i in range(level)]
        left.append(new_p[0])
        right.append(new_p[-1])
        p = new_p
    # left has 6 points (Q0 to midpoint); right is captured in reverse order.
    right.reverse()
    return tuple(left), tuple(right)


def segment_quintic(
    blend: QuinticBlend,
    max_chord_err: float = 1e-2,
) -> List[Vec3]:
    """Return a polyline approximating the quintic blend with max chord
    error <= max_chord_err. Adaptive De Casteljau subdivision."""
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
    if blend.d_consumed == 0.0:
        return [blend.Q[0]]

    out: List[Vec3] = [blend.Q[0]]

    def _recurse(Q, depth):
        if depth >= _SUBDIVIDE_MAX_DEPTH or _quintic_flatness(Q) <= max_chord_err:
            out.append(Q[5])
            return
        left, right = _quintic_split(Q)
        _recurse(left, depth + 1)
        _recurse(right, depth + 1)

    _recurse(blend.Q, 0)
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k segment_quintic`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add adaptive polyline subdivision"
```

---

## Task 14: E-axis parameterization via Gauss–Legendre arc length

Distribute the extruder E-coordinate uniformly over the true arc length of the quintic. Arc length is not closed-form; compute the per-polyline-segment arc length numerically via Gauss–Legendre 5-point integration of `|B'(t)|`. Then apportion E in proportion to arc length.

Note: the existing `blendmath.interpolate_extruder` distributes E over a *piecewise-linear* polyline length, which is a good approximation for arcs. For quintics with 6–16 polyline points per corner, the same approach is accurate enough for the extruder — there's no material benefit to true arc-length integration unless tests show otherwise. **Choice for 6d: follow `blendmath`'s simpler piecewise-linear approach.** Add Gauss–Legendre only if a test fails.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_blendquintic.py`:

```python
def test_interpolate_extruder_conserves_total_e():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.005)
    e_per_mm_prev = 0.12
    e_per_mm_next = 0.10
    extruded = blendquintic.interpolate_extruder_quintic(
        poly, q.d_consumed, e_per_mm_prev, e_per_mm_next,
    )
    total_e = extruded[-1][3] - extruded[0][3]
    expected_total = q.d_consumed * (e_per_mm_prev + e_per_mm_next)
    assert total_e == pytest.approx(expected_total, rel=1e-6)


def test_interpolate_extruder_monotone_increasing():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.005)
    extruded = blendquintic.interpolate_extruder_quintic(
        poly, q.d_consumed, 0.12, 0.10,
    )
    for i in range(len(extruded) - 1):
        assert extruded[i + 1][3] >= extruded[i][3]


def test_interpolate_extruder_degenerate_polyline():
    # Single-point polyline -> single output point with E = 0.
    poly = [(0.0, 0.0, 0.0)]
    out = blendquintic.interpolate_extruder_quintic(poly, 0.0, 0.12, 0.10)
    assert out == [(0.0, 0.0, 0.0, 0.0)]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_blendquintic.py -v -k interpolate_extruder`
Expected: FAIL with `AttributeError: module 'klippy.blendquintic' has no attribute 'interpolate_extruder_quintic'`

- [ ] **Step 3: Write the implementation**

Append to `klippy/blendquintic.py`:

```python
def interpolate_extruder_quintic(
    polyline: List[Vec3],
    d_consumed: float,
    e_per_mm_prev: float,
    e_per_mm_next: float,
) -> List[Tuple[float, float, float, float]]:
    """Attach an E coordinate to each polyline point.

    The quintic blend replaces the final `d_consumed` mm of the prior
    move and the first `d_consumed` mm of the next move. Total E through
    the blend is conserved: sum across the polyline equals
    `d_consumed * (e_per_mm_prev + e_per_mm_next)`. E is distributed
    uniformly over the polyline's arc-length (piecewise-linear).
    """
    if not polyline:
        return []
    total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)

    seg_lens = []
    total_len = 0.0
    for p0, p1 in zip(polyline, polyline[1:]):
        seg_len = vnorm(vsub(p1, p0))
        seg_lens.append(seg_len)
        total_len += seg_len

    if total_len == 0.0:
        return [(p[0], p[1], p[2], 0.0) for p in polyline]

    out: List[Tuple[float, float, float, float]] = [
        (polyline[0][0], polyline[0][1], polyline[0][2], 0.0),
    ]
    e = 0.0
    for seg_len, p1 in zip(seg_lens, polyline[1:]):
        e += total_e * seg_len / total_len
        out.append((p1[0], p1[1], p1[2], e))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_blendquintic.py -v -k interpolate_extruder`
Expected: PASS on all three tests.

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py test/test_blendquintic.py
git commit -m "blendquintic: add E-axis parameterization"
```

---

## Task 15: Property tests — random-corner sweep

Mirror the style of `test_blendmath.py`'s property tests. For 50 random `(prev_dir, next_dir, corner_deviation, a_max)` triples, verify invariants:
- `result` is not None if `theta` is in the validity band.
- `deviation(d_consumed, r, sin(theta/2)) <= corner_deviation` within numerical slack.
- `v_cap^2 * kappa_peak <= a_max * (1 + epsilon)`.
- `r` within the clamp.
- All six control points are finite.

- [ ] **Step 1: Write the failing test**

Append to `test/test_blendquintic.py`:

```python
def test_random_corners_property_sweep():
    rng = __import__("random").Random(20260419)
    for _ in range(50):
        theta = rng.uniform(math.radians(15), math.radians(160))
        # Random rotation in the XY plane.
        phi = rng.uniform(0.0, 2.0 * math.pi)
        prev_dir = (math.cos(phi), math.sin(phi), 0.0)
        next_dir = (
            math.cos(phi + theta),
            math.sin(phi + theta),
            0.0,
        )
        L_prev = rng.uniform(0.5, 5.0)
        L_next = rng.uniform(0.5, 5.0)
        corner_deviation = rng.uniform(0.02, 0.4)
        a_max = rng.uniform(20000.0, 100000.0)
        q = blendquintic.quintic_geometry(
            prev_dir=prev_dir,
            next_dir=next_dir,
            L_prev=L_prev,
            L_next=L_next,
            corner_deviation=corner_deviation,
            a_max=a_max,
        )
        assert q is not None
        assert 0.50 <= q.r <= 0.86
        # Deviation check: either the tolerance was binding (deviation
        # matches corner_deviation) or the half-segment cap was binding
        # (deviation below corner_deviation, d_consumed == 0.5*min(L)).
        sin_half = math.sin(q.theta / 2.0)
        achieved_dev = blendquintic._deviation_closed_form(
            q.d_consumed, q.r, sin_half,
        )
        assert achieved_dev <= corner_deviation + 1e-9
        # Velocity cap: v^2 * kappa_peak <= a_max
        assert q.v_cap * q.v_cap * q.kappa_peak <= a_max * (1.0 + 1e-6)
        # All control points finite.
        for pt in q.Q:
            for c in pt:
                assert math.isfinite(c)
```

- [ ] **Step 2: Run test to verify it passes immediately** (or reveals bugs)

Run: `python3 -m pytest test/test_blendquintic.py::test_random_corners_property_sweep -v`

If it fails, fix the underlying bug in the affected task's code before committing. If it passes, good — property-test tasks often pass on first try when the incremental tests cover well.

- [ ] **Step 3: Commit**

```bash
git add test/test_blendquintic.py
git commit -m "blendquintic: add random-corner property sweep"
```

---

## Task 16: Simulator parity — extend `klipper-sim/examples/shape_ceiling.py`

The simulator repo (`~/Developer/klipper-sim/`) already has `examples/shape_ceiling.py`. Extend it to assert quintic post-shaper deviation ≤ arc post-shaper deviation across `θ ∈ {30°, 60°, 90°, 120°, 150°}` (deflection), using this new `klippy.blendquintic` module.

**Gating:** This task edits a file in a sibling repo. If that repo is not accessible (or the agent is running in an environment without it), skip this task and leave a note. The core acceptance (all test_blendquintic.py tests green) does not depend on the simulator parity check.

- [ ] **Step 1: Check if klipper-sim is accessible**

Run: `test -d ~/Developer/klipper-sim/examples && echo "present" || echo "absent"`

If "absent": skip this task, commit nothing. Leave a note in the 6d session log that simulator parity was deferred.

If "present": proceed to step 2.

- [ ] **Step 2: Inspect existing shape_ceiling.py**

Run: `cat ~/Developer/klipper-sim/examples/shape_ceiling.py | head -100`

Understand how arcs are currently constructed and how the simulator runs the shaper convolution.

- [ ] **Step 3: Add a quintic branch alongside the existing arc branch**

At the point where the existing script constructs an arc blend and convolves it through the shaper, add an equivalent quintic branch using `klippy.blendquintic.quintic_geometry` to build the control points, `segment_quintic` to produce the polyline, and the same simulator pipeline to measure post-shaper deviation. Collect the per-angle post-shaper deviation for both shapes.

(Exact code depends on the current shape of `shape_ceiling.py`; perform a minimal diff that adds the quintic path without breaking the existing arc path.)

- [ ] **Step 4: Add an assertion and run**

At the end of the script, assert for each tested angle that `post_shaper_dev_quintic <= post_shaper_dev_arc + 1e-6`. Run the script.

Expected: all five angles (30°, 60°, 90°, 120°, 150° deflection) report quintic ≤ arc post-shaper deviation.

- [ ] **Step 5: Commit**

```bash
cd ~/Developer/klipper-sim
git add examples/shape_ceiling.py
git commit -m "shape_ceiling: assert quintic <= arc post-shaper deviation"
```

Then return to the kalico repo. No commit needed here unless `shape_ceiling.py` was vendored in-tree — it isn't.

---

## Final verification

After all 16 tasks:

- [ ] **Run the full test suite**

```bash
python3 -m pytest test/test_blendquintic.py -v
```

Expected: all tests pass.

- [ ] **Confirm no regressions in other tests**

```bash
python3 -m pytest test/ -v
```

Expected: pre-existing tests still pass. `test_blendmath.py`, `test_blendshaper.py`, `test_blendplanner.py`, and `test_blendprepass.py` are particularly relevant.

- [ ] **Sanity-check line counts**

```bash
wc -l klippy/blendquintic.py test/test_blendquintic.py
```

Expected: module ~350 LOC, test ~650 LOC (order-of-magnitude estimates from the spec — exact counts may vary ±50%).

- [ ] **Confirm no CornerBlender integration**

```bash
grep -r "blendquintic" klippy/
```

Expected: only `klippy/blendquintic.py` references itself. No imports of `blendquintic` in any other klippy file (that's 6e's job).
