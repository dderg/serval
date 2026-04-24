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


def reachable_v_end(v_start: float, a_max: float, j_max: float, L: float) -> float:
    """Stub — see Task 2 for the real implementation."""
    raise NotImplementedError("Task 2 impl")
