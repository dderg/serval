"""Tests for klippy/jerk_math.py — jerk-aware reachable-velocity.

Plan 9 Phase A2b — verified against the pre-computed Python reference at
docs/superpowers/plans/plan9-derivations/jerk_reachable_ref.py and the
forward primitive at docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from klippy import jerk_math


def _load_module(filename: str):
    path = (
        Path(__file__).resolve().parents[1]
        / "docs" / "superpowers" / "plans" / "plan9-derivations" / filename
    )
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REF = _load_module("jerk_reachable_ref.py")
JP = _load_module("jerk_profile_ref.py")


# ---- basic sanity --------------------------------------------------------

def test_module_exposes_reachable_v_end():
    assert callable(jerk_math.reachable_v_end)


# ---- classical-limit check -----------------------------------------------

def test_matches_classical_formula_at_high_jerk():
    """As j_max -> inf, reachable_v_end(v0, a, j, L) -> sqrt(v0^2 + 2*L*a)."""
    v0, a, L = 100.0, 5000.0, 50.0
    classical = math.sqrt(v0 * v0 + 2.0 * L * a)
    actual = jerk_math.reachable_v_end(v0, a, 1e12, L)
    assert actual == pytest.approx(classical, rel=1e-4)


# ---- regime-A and regime-B spot checks -----------------------------------

def test_regime_a_triangular_short_move():
    """Short L -> triangular regime -> v_end < classical-formula prediction."""
    v0, a, j, L = 0.0, 5000.0, 100000.0, 0.5
    classical = math.sqrt(v0 * v0 + 2.0 * L * a)
    v_end = jerk_math.reachable_v_end(v0, a, j, L)
    assert v_end < classical
    _, _, _, dist = JP.accel_side_timings(v0, v_end, a, j)
    assert dist == pytest.approx(L, rel=1e-9, abs=1e-9)


def test_regime_b_trapezoidal_long_move():
    """Long L -> trapezoidal regime -> v_end close to classical-formula."""
    v0, a, j, L = 0.0, 5000.0, 100000.0, 100.0
    v_end = jerk_math.reachable_v_end(v0, a, j, L)
    _, _, _, dist = JP.accel_side_timings(v0, v_end, a, j)
    assert dist == pytest.approx(L, rel=1e-9, abs=1e-9)


# ---- edge cases ----------------------------------------------------------

def test_zero_distance_returns_v_start():
    assert jerk_math.reachable_v_end(100.0, 5000.0, 100000.0, 0.0) == pytest.approx(100.0)


def test_monotonic_in_L():
    """Doubling L must increase v_end."""
    v0, a, j = 50.0, 3000.0, 80000.0
    v1 = jerk_math.reachable_v_end(v0, a, j, 10.0)
    v2 = jerk_math.reachable_v_end(v0, a, j, 20.0)
    assert v2 > v1


def test_rejects_negative_inputs():
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(-1.0, 5000.0, 100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, -5000.0, 100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, 5000.0, -100000.0, 10.0)
    with pytest.raises(ValueError):
        jerk_math.reachable_v_end(0.0, 5000.0, 100000.0, -10.0)


# ---- 180-case sweep vs pre-verified reference ----------------------------

_SWEEP_V0 = [0.0, 50.0, 200.0, 500.0]
_SWEEP_A  = [2500.0, 5000.0, 10000.0]
_SWEEP_J  = [50000.0, 100000.0, 500000.0]
_SWEEP_L  = [0.1, 1.0, 10.0, 100.0, 1000.0]

_SWEEP_CASES = [
    (v0, a, j, L)
    for v0 in _SWEEP_V0
    for a in _SWEEP_A
    for j in _SWEEP_J
    for L in _SWEEP_L
]


@pytest.mark.parametrize("v0,a,j,L", _SWEEP_CASES,
                         ids=[f"v0={v0},a={a},j={j},L={L}" for v0, a, j, L in _SWEEP_CASES])
def test_sweep_matches_reference(v0, a, j, L):
    ref = REF.reachable_v_end(v0, a, j, L)
    actual = jerk_math.reachable_v_end(v0, a, j, L)
    assert actual == pytest.approx(ref, rel=1e-9, abs=1e-9)
