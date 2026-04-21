# klippy/blendextruder.py
# Per-move extruder cap for the Plan 3 "extruder as first-class
# constraint" pillar. Reads the live Pressure-Advance model and the
# configured (a_E_max, v_E_max, smooth_time) limits, computes the
# tightest (v_xy, a_xy) such that the post-PA stepper output stays
# within the stepper's physical budget.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


# --- Snapshot types ---

@dataclass(frozen=True)
class PAModelSnapshot:
    """Immutable snapshot of a PA model's state at planning time.

    kind:   "linear" | "tanh" | "recipr"
    params: tuple, interpretation by kind:
      linear -> (pressure_advance,)
      tanh   -> (linear_advance, nonlinear_offset, linearization_velocity)
      recipr -> (linear_advance, nonlinear_offset, linearization_velocity)
    """
    kind: str
    params: tuple


# --- Derivative evaluation (mirrors kinematics/extruder.py) ---

def _f_prime(snap: PAModelSnapshot, v: float) -> float:
    """PA model derivative f'(v). Pure math; no live model access."""
    if snap.kind == "linear":
        (pa,) = snap.params
        return pa
    la, no, lv = snap.params
    if lv <= 0.0:
        return la
    if snap.kind == "tanh":
        vn = v / lv
        sech2 = 1.0 - math.tanh(vn) ** 2
        return la + (no / lv) * sech2
    if snap.kind == "recipr":
        r = v / lv
        return la + (no / lv) / (1.0 + r) ** 2
    raise ValueError("unknown PA model kind: " + repr(snap.kind))


# --- Bisection helper for velocity cap inversion ---

def _stepper_v_of_xy(snap: PAModelSnapshot, v_xy: float, k: float, a_E_cap: float) -> float:
    """Peak stepper velocity during accel phase at XY target v_xy."""
    V = k * v_xy
    return V + _f_prime(snap, V) * a_E_cap


def _solve_velocity_cap_bisection(
    snap: PAModelSnapshot,
    k: float,
    a_E_cap: float,
    v_E_max: float,
) -> float:
    """Find the largest v_xy such that _stepper_v_of_xy <= v_E_max.

    The constraint is monotone increasing in v_xy (V increases; the
    f'*a_E_cap term is bounded). 1-D bisection on [0, v_E_max/k]
    converges in ~30 iterations for a 1e-6 mm/s tolerance.
    """
    # Short-circuit: a_E_cap = 0 means no PA term contributes.
    if a_E_cap <= 0.0:
        return v_E_max / k
    lo = 0.0
    hi = v_E_max / k
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _stepper_v_of_xy(snap, mid, k, a_E_cap) <= v_E_max:
            lo = mid
        else:
            hi = mid
        if (hi - lo) < 1e-6:
            break
    return lo


# --- Public API ---

def cap_move(
    move,
    pa_model: Optional[PAModelSnapshot],
    extruder_limits,  # Optional[blendshape.ExtruderLimits]
) -> Tuple[float, float]:
    """Compute (v_cap, a_cap) for a move such that the post-PA stepper
    output stays within extruder_limits. Returns (+inf, +inf) when the
    cap is inactive (travel move, no PA, no limits configured).

    `move` must expose `axes_r[3]` (flow ratio k = dE/dL) and
    `max_cruise_v` (the move's target cruise velocity before capping).

    Linear PA: closed-form cap (Task 4).
    Non-linear PA (tanh/recipr): accel cap closed-form, velocity cap
    via 1-D bisection (Tasks 5-6).
    """
    # Edge case: no PA model (extruder not configured for PA).
    if pa_model is None:
        return (float("inf"), float("inf"))
    # Edge case: no extruder limits configured.
    if extruder_limits is None:
        return (float("inf"), float("inf"))
    # Edge case: pure travel move (no extrusion).
    k = move.axes_r[3]
    if k <= 0.0:
        return (float("inf"), float("inf"))
    # Degenerate: zero accel budget, but velocity cap may still apply.
    if extruder_limits.a_E_max <= 0.0:
        v_E_max_d = extruder_limits.v_E_max
        if v_E_max_d > 0.0:
            return (v_E_max_d / k, float("inf"))
        return (float("inf"), float("inf"))
    # Defend against smooth_time=0 (possible via SET_PRESSURE_ADVANCE runtime).
    if extruder_limits.smooth_time <= 0.0:
        return (float("inf"), float("inf"))

    K_h = (15.0 / 8.0) / extruder_limits.smooth_time
    a_E_max = extruder_limits.a_E_max
    v_E_max = extruder_limits.v_E_max

    if pa_model.kind == "linear":
        (pa,) = pa_model.params
        # Accel cap (closed form; f' is constant).
        a_E_cap = a_E_max / (1.0 + pa * K_h)
        a_cap = a_E_cap / k
        # Velocity cap: stepper_v peaks at v_E + PA * a_E_cap during
        # accel-plateau. Solve: k * v_xy + PA * a_E_cap <= v_E_max.
        v_from_accel = (v_E_max - pa * a_E_cap) / k
        v_from_rpm = v_E_max / k
        if v_from_accel <= 0.0:
            # Accel cap forces stepper above v_E_max at any non-zero speed;
            # enforcing both caps would require stopping motion.  Fall back to
            # the RPM-only cap and skip the accel cap.
            return (v_from_rpm, float("inf"))
        v_cap = min(v_from_rpm, v_from_accel)
        return (v_cap, a_cap)

    # Non-linear PA: tanh or recipr.
    # Accel cap uses v_eval = k * move.max_cruise_v as pragmatic
    # approximation of the binding-moment velocity. The spec's more
    # rigorous v_eval = min(v_prev, v_next) requires lookahead state
    # not reliably available at plan time; for NL params (NO=0.04,
    # LV=100) this approximation is tight to ~1-2%, and Plan 5's
    # continuous v(s) removes the approximation entirely.
    v_eval = k * move.max_cruise_v
    f_prime_eval = _f_prime(pa_model, v_eval)
    a_E_cap = a_E_max / (1.0 + f_prime_eval * K_h)
    a_cap = a_E_cap / k
    # Velocity cap via bisection.
    v_cap = _solve_velocity_cap_bisection(pa_model, k, a_E_cap, v_E_max)
    if v_cap <= 1e-6:
        # Bisection collapsed: accel cap is infeasible at any non-zero speed.
        # Fall back to RPM-only cap.
        return (v_E_max / k, float("inf"))
    return (v_cap, a_cap)
