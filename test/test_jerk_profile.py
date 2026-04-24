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
