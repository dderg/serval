# klippy/blendshaper.py
# Shaper-derived jerk bound module for corner blending.
#
# Given a toolhead's per-axis input-shaper configuration and a
# corner-blend geometry, computes the effective jerk ceiling
# (j_eff) passed to shape primitives like blendquintic.QuinticShape plus a per-axis
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
    # L¹ norm of this axis' feedforward inverse kernel, ‖h_axis‖₁.
    # Consumed by Plan 5 Pillar 1 D4 (QuinticShape.v_cap_fn) to tighten
    # the centripetal cap via G_worst(s) = max_axes G_axis · (|proj_t| +
    # |proj_n|). Defaults to 1.0 (identity cascade) so the cap reduces to
    # the pre-D4 form whenever the axis has no inverse wired — classic
    # FIR, disabled shaper, target_smoothing=0 sentinel, or axes without
    # `recompute_fused_kernel` support.
    inverse_G: float = 1.0


@dataclass(frozen=True)
class ShaperBounds:
    j_eff: float
    v_step_cap: float


# Pulse-sequence span in units of the damped period, keyed by FIR shaper name.
# Values match klippy/extras/shaper_defs.py exactly (last T[i] of each).
_SHAPER_SPAN_FACTOR = {
    "zv": 0.5,
    "mzv": 0.75,
    "zvd": 1.0,
    "ei": 1.0,
    "2hump_ei": 1.5,
    "3hump_ei": 2.0,
}

# Cardinal B-spline chain span: T_sm = F_m / shaper_freq, damping-independent
# by kernel construction. F_m constants from shaper_defs._F_M_TABLE (derived
# at zeta=0.1, V=0.05 residual target); see
# docs/superpowers/plans/plan5-derivations/new_shaper_family.md §2.
_SMOOTH_SPAN_FACTOR = {
    "bs1": 1.5553,
    "bs2": 1.9462,
    "bs3": 2.2519,
    "bs4": 2.5061,
    "bs5": 2.7252,
}


def shaper_span(shaper_type: str, shaper_freq: float, damping_ratio: float) -> float:
    """Effective span in seconds for the given shaper configuration.

    FIR shapers: damped-period * per-type factor.
    Smooth-IS shapers: kernel T_sm = factor / shaper_freq (damping-independent).
    """
    if shaper_type in _SHAPER_SPAN_FACTOR:
        factor = _SHAPER_SPAN_FACTOR[shaper_type]
        t_d = 1.0 / (shaper_freq * math.sqrt(1.0 - damping_ratio * damping_ratio))
        return factor * t_d
    if shaper_type in _SMOOTH_SPAN_FACTOR:
        return _SMOOTH_SPAN_FACTOR[shaper_type] / shaper_freq
    raise ValueError("unknown shaper type: %r" % (shaper_type,))


_AXES = ("x", "y", "z")


def axis_projections(n_hat: Vec3) -> dict:
    """|n̂·ê_axis| per axis. Used by Bound (b) entry-step."""
    return {ax: abs(n_hat[i]) for i, ax in enumerate(_AXES)}


def axis_in_plane(p_hat: Vec3) -> dict:
    """√(1 - |p̂·ê_axis|²) per axis — projection of each basis
    axis onto the arc plane. 1 for fully in-plane axes, 0 for
    fully out-of-plane. Used by Bound (c) rotation jerk."""
    return {ax: math.sqrt(max(0.0, 1.0 - p_hat[i] * p_hat[i]))
            for i, ax in enumerate(_AXES)}


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
        if snap.A_axis <= 0.0:
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
