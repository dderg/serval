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
    # Degenerate: zero accel budget.
    if extruder_limits.a_E_max <= 0.0:
        return (float("inf"), 0.0)

    # Actual cap math is routed by PA model kind in Tasks 4-6.
    # For now, fall through to no-cap (no-op) — Tasks 4-6 replace this.
    return (float("inf"), float("inf"))
