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
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from klippy import blendshaper

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-4
REVERSAL_EPS = 1e-6


def vdot(a: Vec3, b: Vec3) -> float:
    """Scalar dot product of two 3-vectors."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a: Vec3, b: Vec3) -> Vec3:
    """Cross product of two 3-vectors."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a: Vec3) -> float:
    """Euclidean length of a 3-vector."""
    return math.sqrt(vdot(a, a))


def vscale(a: Vec3, s: float) -> Vec3:
    """Multiply a 3-vector by a scalar."""
    return (a[0] * s, a[1] * s, a[2] * s)


def vadd(a: Vec3, b: Vec3) -> Vec3:
    """Component-wise sum of two 3-vectors."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    """Component-wise difference of two 3-vectors (a - b)."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vnormalize(a: Vec3) -> Vec3:
    """Return a unit vector in the direction of a; raise ValueError if zero."""
    n = vnorm(a)
    if n == 0.0:
        raise ValueError("cannot normalize zero vector")
    return vscale(a, 1.0 / n)


@dataclass(frozen=True)
class BlendArc:
    """Tangent-arc blend geometry between two adjacent moves.

    Coordinates: ``entry_pt``, ``exit_pt``, and ``center`` are in a
    corner-local frame where the corner vertex is at the origin. Callers
    must translate by the vertex position to obtain world coordinates.
    ``entry_tangent``, ``exit_tangent``, and ``plane_normal`` are
    direction vectors and frame-independent.

    For degenerate corners (R = 0, returned for U-turns), entry_pt /
    exit_pt / center are all (0, 0, 0) and plane_normal is (0, 0, 0).
    """

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


def blend_geometry(
    prev_dir: Vec3,
    next_dir: Vec3,
    L_prev: float,
    L_next: float,
    corner_deviation: float,
    a_max: float,
    j_eff: float,
) -> Optional[BlendArc]:
    """Compute the tangent-arc blend for a corner, or None if no blend needed.

    ``prev_dir`` and ``next_dir`` must be unit vectors.
    """
    # Deflection angle theta: 0 = collinear, pi = U-turn.
    # With head-to-tail unit directions:
    #   cos(theta) = prev_dir . next_dir
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

    # Deflection angle (rad).
    theta = 2.0 * math.atan2(sin_half, cos_half)

    # Tolerance-driven radius:
    R_tol = corner_deviation * cos_half / (1.0 - cos_half)

    # Midpoint / adjacent-segment cap (half-segment rule): each blend claims
    # at most half the adjacent segment so two neighbouring corners meet at
    # most at the segment midpoint. cot(theta/2) = cos_half / sin_half.
    # d_consumed = R * tan(theta/2) <= 0.5 * min(L_prev, L_next) by construction.
    R_mid = 0.5 * min(L_prev, L_next) * cos_half / sin_half

    R = min(R_tol, R_mid)

    # Centripetal cap: v² <= a_max · R. During steady-state arc cruise
    # tangential accel is identically zero (constant v_cent), so the
    # full acceleration budget is available for centripetal. LinuxCNC's
    # original Pythagorean split reserved 50% tangential (a_n ≤ √3/2 ·
    # a_max) to handle simultaneous decel-and-turn in one segment; our
    # pipeline instead hands decel/accel to the truncated-linear
    # segments on either side of the arc, so reserving that headroom
    # inside the arc is unused.
    v_centripetal = math.sqrt(a_max * R) if R > 0.0 else 0.0

    # Jerk floor: R >= v^(3/2) / sqrt(j_eff)  =>  v <= (R * sqrt(j_eff))^(2/3)
    if R > 0.0 and j_eff > 0.0:
        v_jerk = (R * math.sqrt(j_eff)) ** (2.0 / 3.0)
    else:
        v_jerk = 0.0

    v_cap = min(v_centripetal, v_jerk)

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


def segment_arc(arc: BlendArc, max_chord_err: float = 1e-2) -> list:
    """Return a polyline approximating the arc, with chord error <= max_chord_err."""
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
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


def _sigma_T_max_from_toolhead(toolhead):
    """Return the max RMS spread (seconds) across axis input shaper impulse
    patterns, or 0.0 if no shaper is loaded or all axes are unshaped.

    sigma_T is a property of the shaper impulse sequence alone — independent
    of target_smoothing — so this is safe to use for suppression decisions
    regardless of the target_smoothing=0 sentinel behaviour in
    _extract_shapers().
    """
    if toolhead is None:
        return 0.0
    printer = getattr(toolhead, "printer", None)
    if printer is None:
        return 0.0
    is_obj = printer.lookup_object("input_shaper", None)
    if is_obj is None:
        return 0.0
    from klippy.extras import shaper_defs as _shaper_defs
    factory = {s.name: s.init_func for s in _shaper_defs.INPUT_SHAPERS}
    sigma_max = 0.0
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        # Smooth-family axes don't carry shaper_freq/shaper_type; their
        # impulse-spread sigma_T is zero by construction, so skip them.
        freq = float(getattr(params, "shaper_freq", 0.0) or 0.0)
        stype = getattr(params, "shaper_type", "") or ""
        damp = float(getattr(params, "damping_ratio", 0.0) or 0.0)
        if freq > 0.0 and stype in factory:
            A, T = factory[stype](freq, damp)
            w_sum = sum(A)
            if w_sum > 0.0:
                t_bar = sum(a * t for a, t in zip(A, T)) / w_sum
                var = sum(
                    a * (t - t_bar) ** 2 for a, t in zip(A, T)
                ) / w_sum
                sigma = math.sqrt(max(0.0, var))
                if sigma > sigma_max:
                    sigma_max = sigma
    return sigma_max


def _scv_equivalent_junction_v(
    cos_half: float,
    sin_half: float,
    corner_deviation: float,
    sigma_T_max: float,
    a_max: float,
) -> float:
    """Klipper junction-deviation velocity cap equivalent to mainline SCV
    at a shaper with RMS impulse spread sigma_T_max, evaluated at a corner
    with the given half-angle geometry.

    Derivation:
      - SCV-equivalent at 90° matching corner_deviation under shaper smear:
            v_scv90 = cd / (sqrt(2) * sigma_T)
      - Klipper's JD formula (jd = SCV^2 * (sqrt(2) - 1) / a_max):
            jd_eq = v_scv90^2 * (sqrt(2) - 1) / a_max
      - Per-corner radius and velocity:
            R_scv = jd_eq * cos(theta/2) / (1 - cos(theta/2))
            v_j   = sqrt(R_scv * a_max)

    Returns +inf for collinear (no cap needed) or when any input is
    non-positive (no cap derivable).
    """
    one_minus_cos = 1.0 - cos_half
    if sin_half <= COLLINEAR_EPS or one_minus_cos <= 1e-12:
        return float("inf")
    if sigma_T_max <= 0.0 or corner_deviation <= 0.0 or a_max <= 0.0:
        return float("inf")
    v_scv90 = corner_deviation / (math.sqrt(2.0) * sigma_T_max)
    jd_eq = v_scv90 * v_scv90 * (math.sqrt(2.0) - 1.0) / a_max
    R_scv = jd_eq * cos_half / one_minus_cos
    return math.sqrt(R_scv * a_max)


def suppressed_junction_v(
    prev_move,
    next_move,
    corner_deviation: float,
    toolhead,
) -> Optional[float]:
    """SCV-equivalent junction velocity to apply when blend_from_moves
    returns None at a non-collinear corner.

    Companion to blend_from_moves: when the arc is suppressed (either by
    the shaper-aware or velocity-aware rule), the fork's calc_junction
    has no built-in JD cap — so without this cap the toolhead would hit
    sharp corners at full commanded velocity, causing step skipping.

    Returns:
        None  — truly collinear junction (no cap needed), or no shaper
                 loaded (no cap derivable; mainline-Kalico calc_junction
                 quarter-tan cap still applies as a lax safety net).
        float — velocity cap to pass to prev.limit_next_junction_speed().
    """
    if toolhead is None:
        return None
    prev_dir: Vec3 = (
        prev_move.axes_r[0], prev_move.axes_r[1], prev_move.axes_r[2],
    )
    next_dir: Vec3 = (
        next_move.axes_r[0], next_move.axes_r[1], next_move.axes_r[2],
    )
    dp = max(-1.0, min(1.0, vdot(prev_dir, next_dir)))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))
    if sin_half < COLLINEAR_EPS:
        return None
    sigma_T = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T <= 0.0:
        return None
    a_max = min(prev_move.accel, next_move.accel)
    v_j = _scv_equivalent_junction_v(
        cos_half, sin_half, corner_deviation, sigma_T, a_max,
    )
    if not math.isfinite(v_j):
        return None
    return v_j


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

    # Pick up user-configured target_smoothing if the input_shaper
    # object carries one — lets operators trade quality vs corner
    # speed via the [input_shaper] target_smoothing config. A mock
    # or older object without the attribute falls back to the
    # ShaperCalibrate default (0.12 mm).
    #
    # Sentinel: target_smoothing == 0 disables the shaper-derived
    # velocity cap entirely (returns [] so compute_shaper_bounds
    # produces inf bounds). Use SET_INPUT_SHAPER TARGET_SMOOTHING=0
    # to isolate arc-planner cost from residual shaper-cap cost.
    target = getattr(is_obj, "target_smoothing", None)
    if target is not None and target <= 0.0:
        return []
    sc = ShaperCalibrate(printer=None, target_smoothing=target)
    shaper_factory = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

    snaps = []
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        # After the BE-v2 smooth-shapers port, axes can carry either
        # TypedInputShaperParams (impulse) or TypedInputSmootherParams
        # (smooth). Arc-blending's velocity cap today only consumes the
        # impulse family, so smooth-family axes are recorded with
        # A_axis=0.0 (no contribution to the shaper-derived cap).
        freq = float(getattr(params, "shaper_freq", 0.0) or 0.0)
        shaper_type = getattr(params, "shaper_type", "") or ""
        damping_ratio = float(getattr(params, "damping_ratio", 0.0) or 0.0)
        if freq > 0.0 and shaper_type in shaper_factory:
            impulses = shaper_factory[shaper_type](freq, damping_ratio)
            A_axis = float(sc.find_shaper_max_accel(impulses))
        else:
            A_axis = 0.0
        snaps.append(blendshaper.AxisShaperSnapshot(
            axis=axis_shaper.get_axis(),
            shaper_type=shaper_type,
            shaper_freq=freq,
            damping_ratio=damping_ratio,
            A_axis=A_axis,
        ))
    return snaps


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
    if toolhead is not None and j_eff != float("inf"):
        raise ValueError(
            "blend_from_moves: j_eff and toolhead are mutually exclusive "
            "(toolhead derives j_eff from shaper state; passing both is ambiguous)"
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

    if toolhead is None:
        return blend_geometry(
            prev_dir=prev_dir, next_dir=next_dir,
            L_prev=prev_move.move_d, L_next=next_move.move_d,
            corner_deviation=corner_deviation,
            a_max=a_max, j_eff=j_eff,
        )

    shapers = _extract_shapers(toolhead)

    # Shaper-aware arc suppression: if the input shaper alone can satisfy
    # the corner_deviation post-shaper tolerance at max cruise speed, skip
    # the arc — let the mainline sharp-V corner + shaper handle it.
    # Callers (blendplanner) must then ask suppressed_junction_v() for the
    # SCV-equivalent junction cap and apply it to the outgoing move, since
    # fork calc_junction has no JD-based cap of its own.
    #
    # sigma_T is the RMS spread of the shaper impulse times and is a
    # property of the impulse pattern alone — independent of
    # target_smoothing, so suppression fires correctly even with ts=0.
    sigma_T_max = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T_max > 0.0:
        dp = max(-1.0, min(1.0, vdot(prev_dir, next_dir)))
        cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
        sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))
        one_minus_cos = 1.0 - cos_half
        max_v2 = min(prev_move.max_cruise_v2, next_move.max_cruise_v2)
        max_v = math.sqrt(max_v2)

        # Rule 1 (shaper-aware): skip if the shaper alone keeps the
        # corner inside the corner_deviation post-shaper budget.
        #     eps_shaper = 2 * v * sin(phi/2) * sigma_T_max
        eps_shaper = 2.0 * max_v * sin_half * sigma_T_max
        if eps_shaper <= corner_deviation:
            return None

        # Rule 2 (velocity-aware): skip if an imaginary mainline-SCV
        # junction at this same corner (sized so its post-shaper
        # deviation matches corner_deviation) would be no slower than
        # our arc.  Catches sharp corners with short adjoining segments
        # where R clamps small and v_arc drops below cruise — arc
        # traversal time then exceeds the ramp savings vs SCV-equivalent.
        if sin_half > COLLINEAR_EPS and one_minus_cos > 1e-12:
            _R_tol = corner_deviation * cos_half / one_minus_cos
            _R_mid = 0.5 * min(
                prev_move.move_d, next_move.move_d
            ) * cos_half / sin_half
            _R_clamped = min(_R_tol, _R_mid)
            if _R_clamped > 0.0:
                _v_arc = math.sqrt(_R_clamped * a_max)
                _theta = 2.0 * math.atan2(sin_half, cos_half)
                _L_arc = _R_clamped * _theta
                _v_j_scv = _scv_equivalent_junction_v(
                    cos_half, sin_half, corner_deviation, sigma_T_max, a_max,
                )
                _v_arc_cap = min(_v_arc, max_v)
                _v_j_cap = min(_v_j_scv, max_v)
                _fork_cost = 2.0 * max(0.0, max_v - _v_arc_cap) / a_max \
                    + (_L_arc / _v_arc_cap if _v_arc_cap > 0.0 else 0.0)
                _main_cost = 2.0 * max(0.0, max_v - _v_j_cap) / a_max
                if _fork_cost >= _main_cost:
                    return None

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
    return replace(arc, v_cap=v_cap)


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
