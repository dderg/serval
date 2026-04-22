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

import bisect
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


def _segment_quintic(Q, max_chord_err: float) -> list[Vec3]:
    """Adaptive De Casteljau subdivision — recursion terminates when
    _quintic_flatness(sub_Q) <= max_chord_err or depth == limit.

    If depth limit fires before flatness criterion, the returned segments
    may exceed max_chord_err; callers must not rely on a hard tolerance
    guarantee. Depth limit is _SUBDIVIDE_MAX_DEPTH (=12, giving 4096
    leaves per blend worst case); not expected to fire for normal
    chord_tol values (1e-4 to 1e-2 mm) on well-conditioned blends.
    """
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
    out: list[Vec3] = [Q[0]]

    def _recurse(sub_Q, depth):
        if depth >= _SUBDIVIDE_MAX_DEPTH or _quintic_flatness(sub_Q) <= max_chord_err:
            out.append(sub_Q[5])
            return
        left, right = _quintic_split(sub_Q)
        _recurse(left, depth + 1)
        _recurse(right, depth + 1)

    _recurse(Q, 0)
    return out


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


_SHAPER_SAMPLE_N_DEFAULT = 50
# 2D blend plane normal — all quintic blends are in the XY plane.
_PLANE_NORMAL: Tuple[float, float, float] = (0.0, 0.0, 1.0)

# Axis unit vectors for the per-s saturation cap (Plan 5 Pillar 1 D4).
# Keyed by AxisShaperSnapshot.axis character. Extruder ('e') has no
# geometric XY axis and is intentionally absent — the saturation cap
# treats only the Cartesian (X, Y, Z) motion; extruder saturation acts
# downstream via PA, not centripetal.
_AXIS_UNIT_VECTORS = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


def _compute_G_worst(shapers, tan, nrm) -> float:
    """Per-s, orientation-dependent saturation factor.

    Returns

        G_worst = max over Cartesian shaper axes of
                    G_axis · (|t̂·ê_axis| + |n̂·ê_axis|).

    Falls back to 1.0 when no shapers are provided or none of them map
    to a Cartesian axis (so v_cent collapses to the pre-D4 form).

    Derivation: per_axis_saturation_derivation.md — independent v̇ and
    v²κ caps under the L¹-L∞ convolution bound give the sum-of-
    projections factor, tight everywhere (√2 at diagonal, 1 at axis-
    aligned).
    """
    if not shapers:
        return 1.0
    best = 0.0
    for snap in shapers:
        e_axis = _AXIS_UNIT_VECTORS.get(snap.axis)
        if e_axis is None:
            continue                                   # extruder, skip
        G_axis = getattr(snap, "inverse_G", 1.0)
        if G_axis <= 0.0:
            G_axis = 1.0
        proj_t = abs(tan[0] * e_axis[0] + tan[1] * e_axis[1]
                     + tan[2] * e_axis[2])
        proj_n = abs(nrm[0] * e_axis[0] + nrm[1] * e_axis[1]
                     + nrm[2] * e_axis[2])
        factor = G_axis * (proj_t + proj_n)
        if factor > best:
            best = factor
    if best <= 0.0:
        return 1.0
    return best


def _shaper_cap_dense(Q, shapers, n: int = _SHAPER_SAMPLE_N_DEFAULT) -> float:
    """Min of the shaper entry-step velocity cap over n+1 uniform t-samples.

    Replaces archive's 3-point cap (archive blendquintic.py:367-386) which
    under-tightened by up to ~15% on the full axis-rotation sweep per
    audit 2026-04-20.

    shapers: list of blendshaper.AxisShaperSnapshot; empty/None returns inf.
    n:       number of uniform t-intervals (n+1 sample points).
    """
    from . import blendshaper as _blendshaper
    if not shapers:
        return float("inf")
    worst = float("inf")
    p_hat = _PLANE_NORMAL
    for i in range(n + 1):
        t = i / n
        _, _, nrm = _point_frame(Q, t)
        k = _curvature_at_t(Q, t)
        if k <= 0.0:
            continue
        R = 1.0 / k
        bounds = _blendshaper.compute_shaper_bounds(shapers, R, nrm, p_hat)
        if bounds.v_step_cap < worst:
            worst = bounds.v_step_cap
    return worst


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
    the Newton step adds 1 speed eval at t_approx; total cost ≈ 9
    speed evals per call. Required so that dkappa_ds and curvature_at
    agree to rel=1e-3 in the finite-difference test.
    """
    t_approx = _s_to_t(s_tab, t_tab, s)
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

    def _init_from_Q(
        self,
        Q,
        d_consumed: float,
        theta: float,
        limits: Optional[blendshape.KinematicLimits] = None,
    ) -> None:
        """Internal init. Populates the instance from control points and
        scalar metadata; builds the s->t map."""
        self.Q = Q
        self.d_consumed = d_consumed
        self.theta = theta
        self._limits = limits
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
        next_move. Returns None for degenerate corners (collinear,
        near-reversal, chord budget infeasible). Caller (planner) falls
        back to sharp-V when None is returned."""
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
        """Velocity limit curve V_lim(s) — centripetal + shaper + rotation-jerk.

        Extruder cap comes in plan 4 as a wrapper stage, not here.

        Plan 5 Pillar 1 D4 — saturation cap: the centripetal term is
        tightened by the per-s, orientation-dependent factor

            G_worst(s) = max over shaped axes of
                         G_axis · (|proj_t(s)| + |proj_n(s)|)

        where G_axis = ‖h_axis‖₁ (AxisShaperSnapshot.inverse_G) and
        proj_t, proj_n are the Frenet projections of the axis onto
        t̂(s) and n̂(s). When no feedforward inverse is wired for any
        axis (G_axis == 1 everywhere), G_worst(s) reduces to
        (|proj_t| + |proj_n|) which still varies with blend
        orientation — axis-aligned (∈ {1}) is looser than diagonal
        (→ √2). Derivation: per_axis_saturation_derivation.md.
        """
        limits = self._limits
        if limits is None:
            return float("inf")
        v = limits.v_max
        # Use refined s->t for consistency with curvature_at (Task 8).
        t = _s_to_t_refined(self.Q, self._s_tab, self._t_tab, s)
        kappa = _curvature_at_t(self.Q, t)
        if kappa > 0.0:
            # Frenet frame — shared by D4 saturation cap and the
            # per-s shaper-bandwidth cap below.
            _, tan, nrm = _point_frame(self.Q, t)
            G_worst = _compute_G_worst(limits.shapers, tan, nrm)
            a_eff = limits.a_max / G_worst
            v_cent = math.sqrt(a_eff / kappa)
            v = min(v, v_cent)
            if limits.jerk_max is not None and limits.jerk_max > 0.0:
                v_jerk = (limits.jerk_max / (kappa * kappa)) ** (1.0 / 3.0)
                v = min(v, v_jerk)
            if limits.shapers:
                from . import blendshaper as _blendshaper
                R = 1.0 / kappa
                bounds = _blendshaper.compute_shaper_bounds(
                    limits.shapers, R, nrm, _PLANE_NORMAL
                )
                v = min(v, bounds.v_step_cap)
        return v

    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> list[Vec3]:
        return _segment_quintic(self.Q, chord_tol)

    # ------------------------------------------------------------------
    # Plan 5 D2c / Task 21 — Python-side quintic(s) ∘ s(t) composition.
    # ------------------------------------------------------------------

    def _monomial_coeffs_per_axis(self) -> list[list[float]]:
        """Convert the Bernstein control net Q (6 control points, degree 5)
        into monomial-basis coefficients per axis.

        For a Bezier of degree n = 5 in parameter u in [0, 1]:
          B(u) = sum_{i=0..n} C(n,i) * u^i * (1-u)^(n-i) * Q_i
        Expanding (1-u)^(n-i) in u and regrouping in powers of u gives
        coefficients a_k (k = 0..n) with
          a_k = sum_{i=0..k} (-1)^(k-i) * C(n,i) * C(n-i, k-i) * Q_i
        This is the standard Bernstein -> power-basis change of basis.
        Since s corresponds to arc-length, not u, we additionally
        re-parameterise by the constant factor u = s / L where L is the
        total arc length. That is an approximation (s is not linear in u
        exactly for a Bezier) but is the natural parameterisation to
        feed through compose_phase_polynomials when used downstream on
        degenerate all-cruise profiles, for which the resulting position-
        in-t error is bounded by the s->t cache's bisect-and-Newton
        resolution (see _s_to_t_refined). Proper D7 TOPP emission will
        feed per-s velocity into this composition.

        Returns list of 3 axis coefficient lists (x, y, z), each of
        length 6 (degrees 0..5). Coefficients are in monomial basis
        over the parameter u in [0, 1].
        """
        Q = self.Q
        n = 5
        # Build binomials once.
        def comb(a, b):
            if b < 0 or b > a:
                return 0
            r = 1
            for i in range(b):
                r = r * (a - i) // (i + 1)
            return r
        # axis -> list[6] of monomial-basis coefficients
        out = []
        for axis in range(3):
            qa = [Q[i][axis] for i in range(n + 1)]
            a = [0.0] * (n + 1)
            for k in range(n + 1):
                s = 0.0
                for i in range(k + 1):
                    sign = -1.0 if ((k - i) & 1) else 1.0
                    s += sign * comb(n, i) * comb(n - i, k - i) * qa[i]
                a[k] = s
            out.append(a)
        return out

    def compose_phase_polynomials(
        self,
        v_in: float,
        v_out: float,
        cruise_v: float,
        a_max: float,
    ):
        """Compose the quintic curve position(u) with a trapezoid-in-arc-
        length velocity profile v(s) to yield per-phase position-in-t
        polynomial coefficients.

        Returns
        -------
        (accel_polys, cruise_polys, decel_polys, t_accel_end, t_decel_start,
         total_t, arc_length)

        where each *_polys is a list of 3 axis coefficient lists (x, y, z),
        each a list of length 11 (degrees 0..10) expressing
            position(t) = sum_k c_k * (t - t_phase_start)^k
        in phase-local time. Coefficients beyond the natural polynomial
        degree are zero-padded to 11 so the C-side trapq entry can unpack
        a uniform layout.

        Approximation (D2c first pass): we treat the quintic's natural
        Bezier parameter u ∈ [0, 1] as directly proportional to normalised
        arc length s/L (see _monomial_coeffs_per_axis above). The accel
        phase has s(t) = v_in * t + 0.5 * a_max * t^2, the cruise phase
        has s(t) = s_accel_end + cruise_v * (t - t_accel_end), and the
        decel phase mirrors accel.

        For the **all-cruise degenerate profile** (v_in == v_out ==
        cruise_v), accel_polys and decel_polys are zero-length phases
        (t_accel_end == 0 and t_decel_start == total_t). Use this form
        until D7 TOPP lands.
        """
        import numpy as np

        L = self.arc_length
        if L <= 0.0:
            # Degenerate — return zero-content phases.
            zero_coeffs = [[0.0] * 11, [0.0] * 11, [0.0] * 11]
            return (zero_coeffs, zero_coeffs, zero_coeffs, 0.0, 0.0, 0.0, 0.0)
        # Bezier -> monomial in u on [0, 1], per-axis.
        coeffs_u = self._monomial_coeffs_per_axis()  # [3][6]
        # Build Polynomial objects (in u) per axis.
        poly_u = [np.polynomial.Polynomial(coeffs_u[ax]) for ax in range(3)]
        # u(s) = s / L; poly_s(s) = poly_u(s / L). numpy handles this via
        # composition with an (s / L) Polynomial.
        u_of_s = np.polynomial.Polynomial([0.0, 1.0 / L])  # 0 + (1/L)*s

        # Compute phase durations from the trapezoid profile.
        # s_accel_end, t_accel_end
        if a_max > 0.0:
            t_accel_end = (cruise_v - v_in) / a_max
            t_decel_duration = (cruise_v - v_out) / a_max
        else:
            t_accel_end = 0.0
            t_decel_duration = 0.0
        if t_accel_end < 0.0:
            t_accel_end = 0.0
        if t_decel_duration < 0.0:
            t_decel_duration = 0.0
        s_accel_end = v_in * t_accel_end + 0.5 * a_max * t_accel_end * t_accel_end
        s_decel_start = L - (cruise_v * t_decel_duration
                              - 0.5 * a_max * t_decel_duration * t_decel_duration)
        if s_decel_start < s_accel_end:
            # Not enough arc-length for a cruise plateau — collapse to a
            # symmetric accel+decel with no cruise. For D2c's all-cruise
            # degenerate bootstrap this branch is never taken; D7 handles
            # the general case.
            s_accel_end = s_decel_start = 0.5 * L
            # Recompute t_accel_end and t_decel_duration from the halved
            # arc-length.
            # v_peak^2 = v_in^2 + 2 * a_max * s_accel_end
            v_peak = math.sqrt(max(v_in * v_in + 2.0 * a_max * s_accel_end,
                                   v_out * v_out + 2.0 * a_max * (L - s_decel_start)))
            t_accel_end = (v_peak - v_in) / a_max if a_max > 0.0 else 0.0
            t_decel_duration = (v_peak - v_out) / a_max if a_max > 0.0 else 0.0
            cruise_v = v_peak
        # Cruise-phase duration.
        s_cruise = s_decel_start - s_accel_end
        t_cruise = s_cruise / cruise_v if cruise_v > 0.0 else 0.0
        t_decel_start = t_accel_end + t_cruise
        total_t = t_decel_start + t_decel_duration

        # s(t) per phase, expressed as numpy.polynomial.Polynomial in
        # phase-local time delta_t.
        # accel phase: s_accel(delta_t) = v_in * delta_t + 0.5 * a_max * delta_t^2
        s_accel = np.polynomial.Polynomial([0.0, v_in, 0.5 * a_max])
        # cruise phase: s_cruise_local(delta_t) = s_accel_end + cruise_v * delta_t
        s_cruise_poly = np.polynomial.Polynomial([s_accel_end, cruise_v])
        # decel phase in phase-local delta_t (starts at t = t_decel_start,
        # initial velocity cruise_v, accel = -a_max):
        # s_decel_local(delta_t) = s_decel_start + cruise_v * delta_t
        #                          - 0.5 * a_max * delta_t^2
        s_decel_poly = np.polynomial.Polynomial(
            [s_decel_start, cruise_v, -0.5 * a_max]
        )

        def _pad(coef, n=11):
            out = list(coef) + [0.0] * (n - len(coef))
            return out[:n]

        def _compose(axis_poly_s, phase_s_poly):
            # phase_s_poly is s(delta_t); axis_poly_s is position(s).
            # Compose: position(delta_t) = axis_poly_s(phase_s_poly(delta_t)).
            # numpy.polynomial supports this via Polynomial(np.asarray(p_s.coef))
            # evaluated with the s polynomial substituted in.
            composed = axis_poly_s(phase_s_poly)
            return _pad(composed.coef, 11)

        # position_s_poly per axis = poly_u(u_of_s) — reparam u -> s.
        position_s = [poly_u[ax](u_of_s) for ax in range(3)]

        accel_polys = [_compose(position_s[ax], s_accel) for ax in range(3)]
        cruise_polys = [_compose(position_s[ax], s_cruise_poly) for ax in range(3)]
        decel_polys = [_compose(position_s[ax], s_decel_poly) for ax in range(3)]

        return (
            accel_polys,
            cruise_polys,
            decel_polys,
            t_accel_end,
            t_decel_start,
            total_t,
            L,
        )

    def v_cap_min(self) -> float:
        """Minimum of v_cap_fn over [0, arc_length] sampled at 128 points.
        Used as the Option Z upstream junction cap — the blend's tightest
        velocity constraint, fed back to the planner so the previous linear
        move decelerates into the blend rather than hitting the cap mid-
        curve.
        """
        if self.arc_length <= 0.0:
            return float("inf")
        best = float("inf")
        n = 128
        for i in range(n + 1):
            s = (i / n) * self.arc_length
            v = self.v_cap_fn(s)
            if v < best:
                best = v
        return best
