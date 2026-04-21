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
from typing import Optional, Tuple

from . import blendshape

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6
_SUBDIVIDE_MAX_DEPTH = 12
_DEFAULT_CHORD_TOL = 1e-3   # 1 um; tighter than archive's 10 um to reduce segment-boundary kappa steps


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


def _point_frame(Q, t: float) -> tuple[Vec3, Vec3, Vec3]:
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


def _peak_curvature(Q, n_samples: int = 100) -> tuple[float, float]:
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


# r(theta) quadratic fit — archive values, verified by audit. Clamped
# to [0.50, 0.86] to stay within empirical validity window.
_R_A = 0.5085
_R_B = -0.03785
_R_C = 0.05715
# Lower clamp is a safety rail — with current coefficients the quadratic
# minimum is 0.502 at theta=0.331 rad, so this clamp never fires in
# practice. Kept to guard against future coefficient revisions.
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


def _d_from_deviation(eps: float, r: float, sin_half: float) -> float:
    """Inverse of _deviation_closed_form: tangent length d required to
    achieve chord deviation eps. Returns +inf when collinear
    (sin_half==0) or when r would drive the denominator non-positive.
    """
    denom = (1.0 + 15.0 * r) * sin_half
    if denom <= 0.0:
        return float("inf")
    return 16.0 * eps / denom


def _speed_at_t(Q, t: float) -> float:
    """|B'(t)| at parameter t — the parametric speed used for arc-length."""
    d1 = _quintic_first_deriv(Q, t)
    return math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)


def _build_s_to_t_map(
    Q, n_gl: int = 8, n_subintervals: int = 20
) -> tuple[list[float], list[float], float]:
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


def _s_to_t(s_tab: list[float], t_tab: list[float], s: float) -> float:
    """Invert the s->t map. Bisect to find the s_tab interval, then
    linearly interpolate within the sub-interval."""
    if s <= 0.0:
        return t_tab[0]
    if s >= s_tab[-1]:
        return t_tab[-1]
    import bisect
    idx = bisect.bisect_left(s_tab, s)
    s_lo, s_hi = s_tab[idx - 1], s_tab[idx]
    t_lo, t_hi = t_tab[idx - 1], t_tab[idx]
    if s_hi <= s_lo:
        return t_lo
    frac = (s - s_lo) / (s_hi - s_lo)
    return t_lo + (t_hi - t_lo) * frac


def _s_to_t_refined(
    Q, s_tab: list[float], t_tab: list[float], s: float
) -> float:
    """As _s_to_t, plus one Newton step using the local arc-length
    integrator to sharpen the linear interpolation error.

    One GL8 integration over [t_lo, t_approx] costs 8 speed evals;
    total cost per call ≈ 16 speed evals (one for the GL8, one for the
    Newton correction). Required so that dkappa_ds and curvature_at
    agree to rel=1e-3 in the finite-difference test.
    """
    t_approx = _s_to_t(s_tab, t_tab, s)
    import bisect
    if s <= 0.0 or s >= s_tab[-1]:
        return t_approx
    idx = bisect.bisect_left(s_tab, s)
    s_lo = s_tab[idx - 1]
    t_lo = t_tab[idx - 1]
    # Arc length from t_lo to t_approx via GL8.
    half = 0.5 * (t_approx - t_lo)
    mid_t = 0.5 * (t_approx + t_lo)
    seg = 0.0
    for j in range(8):
        t_j = mid_t + half * _GL8_NODES[j]
        seg += _GL8_WEIGHTS[j] * _speed_at_t(Q, t_j)
    seg *= half
    s_actual = s_lo + seg
    # Newton correction: t += (s_target - s_actual) / speed(t_approx)
    speed = _speed_at_t(Q, t_approx)
    if speed < 1e-30:
        return t_approx
    return t_approx + (s - s_actual) / speed


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
        """Construct a quintic blend for the corner between prev_move and
        next_move. Returns None for degenerate corners. Full implementation
        lands in Task 12."""
        if prev_move is None or next_move is None:
            return None
        return None

    def position_at(self, s: float) -> Vec3:
        t = _s_to_t_refined(self.Q, self._s_tab, self._t_tab, s)
        return _quintic_eval(self.Q, t)

    def tangent_at(self, s: float) -> Vec3:
        t = _s_to_t_refined(self.Q, self._s_tab, self._t_tab, s)
        d1 = _quintic_first_deriv(self.Q, t)
        mag = math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)
        if mag < 1e-30:
            return (1.0, 0.0, 0.0)
        return (d1[0] / mag, d1[1] / mag, d1[2] / mag)

    def curvature_at(self, s: float) -> float:
        t = _s_to_t_refined(self.Q, self._s_tab, self._t_tab, s)
        return _curvature_at_t(self.Q, t)

    def dkappa_ds(self, s: float) -> float:
        """Analytical dκ/ds via the chain rule; no finite differences.

        2D planar derivation:
            κ(t) = (B' × B'')·ẑ / |B'|^3          (signed)
            dκ/dt = (B' × B''')·ẑ / |B'|^3
                  − 3κ (B'·B'') / |B'|²
            dκ/ds = (dκ/dt) / |B'(t)|

        curvature_at returns |κ| (unsigned), so we return
        d(|κ|)/ds = sign(κ) · (dκ/ds) to stay consistent with
        the finite-difference convention used in tests.
        """
        t = _s_to_t_refined(self.Q, self._s_tab, self._t_tab, s)
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
        kappa = cross_12_z / d1_mag3           # signed scalar curvature
        dkappa_dt = cross_13_z / d1_mag3 - 3.0 * kappa * dot_12 / d1_mag2
        dkappa_ds_signed = dkappa_dt / d1_mag
        # Return d(|κ|)/ds = sign(κ)·(dκ/ds); matches unsigned curvature_at.
        if kappa < 0.0:
            return -dkappa_ds_signed
        return dkappa_ds_signed

    def v_cap_fn(self, s: float) -> float:
        raise NotImplementedError   # task 10-11

    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> list[Vec3]:
        raise NotImplementedError   # task 9
