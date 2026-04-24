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
    """Real cube root that preserves the sign of x."""
    return math.copysign(abs(x) ** (1.0 / 3.0), x)


def _regime_boundary_distance(v_start: float, a_max: float, j_max: float) -> float:
    """Accel-side distance at the triangular/trapezoidal regime boundary.

    At the boundary dv_b = a_max**2 / j_max, T_b = 2*a_max/j_max, so
      L_b = 0.5*(v_start + v_end)*T_b = (2*v_start + dv_b) * (a_max/j_max).
    """
    dv_b = a_max * a_max / j_max
    return (2.0 * v_start + dv_b) * (a_max / j_max)


def _reachable_v_end_tri(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Triangular regime (peak acceleration stays below a_max).

    Substituting u = sqrt(dv) in  L = (2*v_start + dv) * sqrt(dv / j_max)
    gives the depressed cubic
        u^3 + 2*v_start*u - L*sqrt(j_max) = 0.
    Cardano with p = 2*v_start >= 0, q = -L*sqrt(j_max) <= 0 gives
    D = (q/2)^2 + (p/3)^3 >= 0, one real root
        u = cbrt(-q/2 + sqrt(D)) + cbrt(-q/2 - sqrt(D)).
    Both arguments are real; the second may be negative for v_start > 0,
    so use the signed cube root.  dv = u**2, v_end = v_start + dv.
    """
    if v_start <= 0.0:
        # Pure triangular from rest: L = u^3 / sqrt(j_max).
        u = (L * math.sqrt(j_max)) ** (1.0 / 3.0)
        dv = u * u
        return v_start + dv

    p = 2.0 * v_start
    q = -L * math.sqrt(j_max)
    half_q = 0.5 * q
    D = half_q * half_q + (p / 3.0) ** 3
    sqrt_D = math.sqrt(D)
    t1 = -half_q + sqrt_D
    t2 = -half_q - sqrt_D
    u = _signed_cbrt(t1) + _signed_cbrt(t2)
    dv = u * u
    return v_start + dv


def _reachable_v_end_trap(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Trapezoidal regime (peak acceleration saturates at a_max).

    From  2*L = (v_end^2 - v_start^2)/a_max + a_max*(v_end + v_start)/j_max
    with dv = v_end - v_start and C = a_max^2 / j_max:
        dv^2 + (2*v_start + C)*dv + (2*v_start*C - 2*L*a_max) = 0.
    Take the positive root.
    """
    C = a_max * a_max / j_max
    b = 2.0 * v_start + C
    c = 2.0 * v_start * C - 2.0 * L * a_max
    disc = b * b - 4.0 * c
    # disc = (2*v_start - C)^2 + 8*L*a_max >= 0 for valid inputs; guard fp noise.
    if disc < 0.0:
        disc = 0.0
    dv = 0.5 * (-b + math.sqrt(disc))
    return v_start + dv


def reachable_v_end(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Return v_end such that a jerk-limited accel-side group from v_start to
    v_end covers distance L under (a_max, j_max).

    v_start >= 0, a_max > 0, j_max > 0, L >= 0.  Returns v_end >= v_start.
    L == 0 returns v_start.

    Raises ValueError on non-finite inputs or on v_start<0, a_max<=0,
    j_max<=0, L<0.
    """
    if not (math.isfinite(v_start) and math.isfinite(a_max)
            and math.isfinite(j_max) and math.isfinite(L)):
        raise ValueError(
            "reachable_v_end requires finite inputs; got "
            f"v_start={v_start!r}, a_max={a_max!r}, j_max={j_max!r}, L={L!r}"
        )
    if v_start < 0.0:
        raise ValueError(f"reachable_v_end: v_start must be >= 0, got {v_start!r}")
    if a_max <= 0.0:
        raise ValueError(f"reachable_v_end: a_max must be > 0, got {a_max!r}")
    if j_max <= 0.0:
        raise ValueError(f"reachable_v_end: j_max must be > 0, got {j_max!r}")
    if L < 0.0:
        raise ValueError(f"reachable_v_end: L must be >= 0, got {L!r}")
    if L == 0.0:
        return v_start

    L_boundary = _regime_boundary_distance(v_start, a_max, j_max)
    if L <= L_boundary:
        return _reachable_v_end_tri(v_start, a_max, j_max, L)
    return _reachable_v_end_trap(v_start, a_max, j_max, L)
