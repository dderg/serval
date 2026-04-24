"""Parity tests for klippy/chelper/jerk_profile.c against the Python
reference at docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py.

Plan 9 Phase A1 — jerk-limited polynomial profile generator.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from klippy.chelper import jerk_profile as jp


def _load_reference():
    ref_path = (
        Path(__file__).resolve().parents[1]
        / "docs"
        / "superpowers"
        / "plans"
        / "plan9-derivations"
        / "jerk_profile_ref.py"
    )
    spec = importlib.util.spec_from_file_location("jerk_profile_ref", ref_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REF = _load_reference()


def test_module_importable():
    """Sanity — wrapper and C symbols load cleanly."""
    assert hasattr(jp, "compute_profile")


_ACCEL_CASES = [
    # (v_start, v_end, a_max, j_max, description)
    (0.0,   100.0, 5000.0, 100000.0, "zero to 100"),
    (100.0, 0.0,   5000.0, 100000.0, "100 to zero (decel)"),
    (0.0,   500.0, 5000.0, 100000.0, "zero to 500 (trapezoidal)"),
    (0.0,   50.0,  5000.0, 100000.0, "zero to 50 (triangular, small dv)"),
    (200.0, 200.0, 5000.0, 100000.0, "no change (dv == 0)"),
    (300.0, 100.0, 3000.0, 50000.0,  "decel, different limits"),
    (0.0,   250.0, 2500.0, 25000.0,  "exactly at trap/tri boundary"),
]


@pytest.mark.parametrize(
    "v_start,v_end,a_max,j_max,desc", _ACCEL_CASES,
    ids=[c[4] for c in _ACCEL_CASES])
def test_accel_side_timings_matches_reference(v_start, v_end, a_max, j_max, desc):
    t_j_c, t_a_c, a_p_c, d_c = jp.accel_side_timings(v_start, v_end, a_max, j_max)
    t_j_r, t_a_r, a_p_r, d_r = REF.accel_side_timings(v_start, v_end, a_max, j_max)
    # All four returned quantities must match to 1e-12 (same math on same CPU fp64).
    assert t_j_c == pytest.approx(t_j_r, abs=1e-12), f"t_j mismatch ({desc})"
    assert t_a_c == pytest.approx(t_a_r, abs=1e-12), f"t_a mismatch ({desc})"
    assert a_p_c == pytest.approx(a_p_r, abs=1e-12), f"a_peak mismatch ({desc})"
    assert d_c   == pytest.approx(d_r,   abs=1e-9),  f"dist mismatch ({desc})"


# Cases where cruise collapses — find_v_hat must return something < v_peak.
_V_HAT_CASES = [
    # (v0, v1, v_peak, a_max, j_max, L, desc)
    (0.0, 0.0, 500.0, 5000.0, 100000.0, 10.0,  "short symmetric"),
    (0.0, 100.0, 500.0, 5000.0, 100000.0, 15.0, "short asymmetric"),
    (50.0, 150.0, 500.0, 3000.0, 50000.0, 20.0, "both endpoints nonzero"),
    (200.0, 200.0, 500.0, 5000.0, 100000.0, 8.0, "endpoints equal, nonzero"),
]


@pytest.mark.parametrize(
    "v0,v1,v_peak,a_max,j_max,L,desc", _V_HAT_CASES,
    ids=[c[6] for c in _V_HAT_CASES])
def test_find_v_hat_matches_reference(v0, v1, v_peak, a_max, j_max, L, desc):
    v_hat_c = jp.find_v_hat(v0, v1, v_peak, a_max, j_max, L)
    # Reference's find_v_hat has signature (v0, v1, a_max, j_max, L) — it does
    # NOT take v_peak (brackets by doubling from max(v0,v1)). The C uses v_peak
    # as v_hi instead. Both converge to the same root.
    v_hat_r = REF.find_v_hat(v0, v1, a_max, j_max, L)
    assert v_hat_c == pytest.approx(v_hat_r, rel=1e-9, abs=1e-9), \
        f"v_hat mismatch ({desc}): C={v_hat_c}, ref={v_hat_r}"
    # Sanity: v_hat must be in [max(v0,v1), v_peak].
    assert v_hat_c >= max(v0, v1) - 1e-9
    assert v_hat_c <= v_peak + 1e-9
