# klippy/topp.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Time-Optimal Path Parameterization (Pham 2014) forward+backward pass on a
# dense grid, yielding a trapezoid-in-s velocity profile for a single
# corner blend.
#
# Fed by klippy/blendquintic.py::QuinticShape.v_cap_fn(s) and consumed by
# blendplanner.CornerBlender._emit_blend to construct the per-phase
# position-in-t polynomial composition for a single QuinticBlendMove emit.
#
# Design reference: docs/superpowers/plans/plan5-derivations/unified_v_of_s.md
# (TOPP derivation + trapezoid-in-s representation rationale).
from __future__ import annotations

import math
from typing import Callable, Tuple


class TOPPError(RuntimeError):
    """Raised when the TOPP forward+backward pass cannot produce a feasible
    profile — e.g. v_in exceeds v_cap at s=0 or v_out exceeds v_cap at s=L.

    Upstream callers can surface this as a plan error; the lookahead
    produces compatible v_in/v_out by feeding the blend's v_cap_min into
    the junction decision (Option Z), so this is strictly a fallback
    for ill-posed inputs.
    """
    pass


# Number of uniform samples spanning [0, L] for the dense grid. Chosen as
# a balance between profile fidelity and per-blend emit cost: at N=128
# the arc-step Δs ≈ L/128 gives sub-micron resolution on typical blends
# (L ~ 0.1-2 mm) and the forward+backward passes together cost ~256
# v_cap_fn evaluations (each is a min of up to 5 branches).
DEFAULT_N_SAMPLES = 128


def topp_trapezoid(
    v_cap_fn: Callable[[float], float],
    arc_length: float,
    v_in: float,
    v_out: float,
    a_max: float,
    n_samples: int = DEFAULT_N_SAMPLES,
) -> Tuple[float, float, float]:
    """Pham 2014 forward+backward TOPP pass on a uniform grid, fit to a
    single trapezoid-in-s.

    Returns (cruise_v, s_accel_end, s_decel_start) where
        - cruise_v       is the flat speed of the mid-blend plateau,
        - s_accel_end    is the arc-length where the accel ramp meets
                         cruise_v,
        - s_decel_start  is the arc-length where the decel ramp leaves
                         cruise_v.

    Invariants on the returned tuple:
      0 <= s_accel_end <= s_decel_start <= arc_length
      cruise_v <= min_s v_cap(s)  (pointwise cap respected)
      v(0) = v_in   (ramp up from v_in with |a| <= a_max)
      v(L) = v_out  (ramp down to v_out with |a| <= a_max)

    For all-cruise degenerate profiles (v_in == v_out == uniform-cap),
    s_accel_end = 0 and s_decel_start = arc_length.

    The forward+backward profile may ride below cruise_v at the
    shoulders if v_cap dips — the trapezoid fit then takes cruise_v =
    min_s v_opt(s), which is still pointwise-safe. The cost of this
    simplification is ~1.4% time vs full TOPP on the worked 90° case
    (see unified_v_of_s.md §4.3, post-correction).

    Raises TOPPError if v_in > v_cap(0) or v_out > v_cap(arc_length)
    (boundary infeasibility — the lookahead junction-cap contract
    should normally prevent this via the v_cap_min feed).
    """
    if arc_length <= 0.0:
        # Degenerate (zero-length blend). Return a collapsed profile that
        # does not move in s but respects endpoint velocities.
        cruise = min(max(v_in, 0.0), max(v_out, 0.0))
        if cruise <= 0.0:
            cruise = max(v_in, v_out, 0.0)
        return (cruise, 0.0, 0.0)

    if n_samples < 2:
        n_samples = 2

    ds = arc_length / n_samples
    n_pts = n_samples + 1

    # Sample v_cap on the dense grid. Negative / zero caps (degenerate)
    # are clamped to a tiny positive number to avoid sqrt-of-negative
    # downstream; a properly-formed blend has v_cap(0) = v_cap(L) = v_max.
    v_cap = [0.0] * n_pts
    for i in range(n_pts):
        s_i = i * ds
        v = v_cap_fn(s_i)
        if not math.isfinite(v) or v <= 0.0:
            v_cap[i] = float("inf") if v == float("inf") else 1e-30
        else:
            v_cap[i] = v

    # Boundary feasibility: v_in must be reachable at s=0 within v_cap.
    # Use a small relative tolerance to accommodate numerical noise when
    # the lookahead handed us v_in = v_cap(0) - ε.
    if v_in > v_cap[0] * (1.0 + 1e-9):
        raise TOPPError(
            f"v_in={v_in:.6g} exceeds v_cap(0)={v_cap[0]:.6g}"
        )
    if v_out > v_cap[-1] * (1.0 + 1e-9):
        raise TOPPError(
            f"v_out={v_out:.6g} exceeds v_cap(L)={v_cap[-1]:.6g}"
        )

    # Forward pass: v_fwd[i+1]² = min(v_cap[i+1]², v_fwd[i]² + 2·a_max·Δs)
    v_fwd2 = [0.0] * n_pts
    v_fwd2[0] = min(v_in * v_in, v_cap[0] ** 2)
    two_a_ds = 2.0 * max(a_max, 0.0) * ds
    for i in range(n_pts - 1):
        reachable2 = v_fwd2[i] + two_a_ds
        cap2 = v_cap[i + 1] ** 2
        v_fwd2[i + 1] = cap2 if reachable2 > cap2 else reachable2

    # Backward pass: v_bwd[i-1]² = min(v_cap[i-1]², v_bwd[i]² + 2·a_max·Δs)
    v_bwd2 = [0.0] * n_pts
    v_bwd2[-1] = min(v_out * v_out, v_cap[-1] ** 2)
    for i in range(n_pts - 1, 0, -1):
        reachable2 = v_bwd2[i] + two_a_ds
        cap2 = v_cap[i - 1] ** 2
        v_bwd2[i - 1] = cap2 if reachable2 > cap2 else reachable2

    # Optimal v²(s) = min(v_fwd², v_bwd²) pointwise.
    v_opt2 = [min(v_fwd2[i], v_bwd2[i]) for i in range(n_pts)]

    # Fit a single trapezoid-in-s to the v_opt profile.
    # Strategy: the true TOPP-optimal plateau is max_s v_opt(s), which is
    # the highest speed the forward/backward ramps both reach.  We pick
    # cruise_v = that peak, then clamp it by min_s v_cap to stay
    # pointwise-safe.  For a monotone v_cap dip (symmetric corner) the
    # peak sits at the shoulders' v_cap_min and the two coincide; for a
    # purely asymmetric case (v_cap flat, v_in != v_out) the peak is the
    # flat cap value and the ramps degenerate to just the binding side.
    cruise2 = max(v_opt2)
    cap_min2 = min(v_cap[i] ** 2 for i in range(n_pts))
    if cruise2 > cap_min2:
        cruise2 = cap_min2
    if cruise2 < 0.0:
        cruise2 = 0.0
    cruise_v = math.sqrt(cruise2)

    # Accel ramp length: distance to go from v_in to cruise_v at a_max
    # magnitude (sign determined by direction — accel up or decel down).
    if a_max > 0.0:
        s_accel_end = abs(cruise2 - v_in * v_in) / (2.0 * a_max)
    else:
        s_accel_end = 0.0
    if s_accel_end < 0.0:
        s_accel_end = 0.0
    if s_accel_end > arc_length:
        s_accel_end = arc_length

    # Decel ramp length: distance from cruise_v to v_out at a_max
    # magnitude.
    if a_max > 0.0:
        decel_len = abs(cruise2 - v_out * v_out) / (2.0 * a_max)
    else:
        decel_len = 0.0
    if decel_len < 0.0:
        decel_len = 0.0
    if decel_len > arc_length:
        decel_len = arc_length
    s_decel_start = arc_length - decel_len

    # Handle the overlap: if the two ramps don't fit into the arc length
    # together, collapse to a wedge (no cruise plateau). The wedge apex
    # velocity is the highest mutually feasible speed — limited by the
    # arc length and by cruise_v (the min-cap ceiling already computed).
    if s_decel_start < s_accel_end:
        if a_max > 0.0:
            # Peak velocity where forward and backward ramps meet:
            #   v_peak² = (v_in² + v_out²)/2 + a_max·arc_length
            # (midpoint-peak formula for a v_in/v_out wedge across L).
            v_peak2 = 0.5 * (v_in * v_in + v_out * v_out) + a_max * arc_length
            if v_peak2 < 0.0:
                v_peak2 = 0.0
            # Clip to min v_cap so the wedge stays pointwise-safe.
            if v_peak2 > cap_min2:
                v_peak2 = cap_min2
            v_peak = math.sqrt(v_peak2)
            s_accel_end = abs(v_peak2 - v_in * v_in) / (2.0 * a_max)
            s_accel_end = max(0.0, min(arc_length, s_accel_end))
            s_decel_start = s_accel_end
            cruise_v = v_peak
        else:
            s_accel_end = s_decel_start = 0.5 * arc_length
            cruise_v = max(v_in, v_out)

    # Final invariant.
    if cruise_v < 0.0:
        cruise_v = 0.0

    return (cruise_v, s_accel_end, s_decel_start)


def topp_s_to_t_trapezoid(
    cruise_v: float,
    s_accel_end: float,
    s_decel_start: float,
    arc_length: float,
    v_in: float,
    v_out: float,
    a_max: float,
) -> Tuple[float, float, float]:
    """Invert the trapezoid-in-s profile to produce time boundaries.

    Returns (t_accel_end, t_decel_start, total_t).

    Uses the phase-closed-form
        accel:   t = (cruise_v - v_in) / a    (sign depends on direction)
        cruise:  t = (s_decel_start - s_accel_end) / cruise_v
        decel:   t = (cruise_v - v_out) / a   (sign depends on direction)

    For a degenerate all-cruise profile (s_accel_end == 0 and
    s_decel_start == arc_length) returns t_accel_end = 0,
    t_decel_start = arc_length / cruise_v, total_t = arc_length / cruise_v.
    """
    if arc_length <= 0.0 or cruise_v <= 0.0:
        return (0.0, 0.0, 0.0)

    # Accel phase: v goes from v_in to cruise_v over s_accel_end.
    # t_accel = (cruise_v - v_in) / a_signed where a_signed = ±a_max.
    if s_accel_end > 0.0 and abs(cruise_v - v_in) > 1e-12:
        # Time via kinematic integral: t = 2·Δs / (v0 + v1).
        t_accel = 2.0 * s_accel_end / (v_in + cruise_v)
    else:
        t_accel = 0.0

    # Cruise phase.
    s_cruise = s_decel_start - s_accel_end
    if s_cruise < 0.0:
        s_cruise = 0.0
    t_cruise = s_cruise / cruise_v if cruise_v > 0.0 else 0.0

    # Decel phase.
    decel_len = arc_length - s_decel_start
    if decel_len > 0.0 and abs(cruise_v - v_out) > 1e-12:
        t_decel = 2.0 * decel_len / (cruise_v + v_out)
    elif decel_len > 0.0:
        t_decel = decel_len / cruise_v
    else:
        t_decel = 0.0

    t_accel_end = t_accel
    t_decel_start = t_accel + t_cruise
    total_t = t_decel_start + t_decel
    return (t_accel_end, t_decel_start, total_t)
