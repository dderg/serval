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


# ---- Phase A5: max_reachable_cruise_v primitive --------------------------
#
# The inverse of reachable_v_end: given start_v and end_v at either end of
# a segment of length L under (a_max, j_max), compute the largest cruise_v
# such that the move is jerk-feasible.


def test_max_cruise_v_trivial_at_cap_when_long():
    # Long segment: starting and ending at low v, cap at 500 mm/s.
    # 100 mm is plenty to reach 500 mm/s under a=5000, j=1e5.
    v = jerk_math.max_reachable_cruise_v(
        v_start=100.0, v_end=100.0, a_max=5000.0, j_max=100000.0,
        L=100.0, v_cruise_cap=500.0,
    )
    assert v == pytest.approx(500.0, rel=1e-9)


def test_max_cruise_v_equals_endpoints_when_no_distance():
    # No distance: cruise_v collapses to the tighter of the two endpoints.
    v = jerk_math.max_reachable_cruise_v(
        v_start=200.0, v_end=300.0, a_max=5000.0, j_max=100000.0,
        L=0.0, v_cruise_cap=1e9,
    )
    # With L=0 no ramp is possible; the only feasible cruise is
    # min(v_start, v_end) (or, equivalently, the bisection collapses).
    assert v == pytest.approx(200.0, rel=1e-9)


def test_max_cruise_v_symmetric_triangular():
    # Start and end equal, short L -- answer is the triangular peak.
    # With v_start == v_end, by symmetry the optimal split is L_acc = L/2
    # and the achievable cruise_v equals reachable_v_end(v_start, a, j, L/2).
    L = 10.0
    v = jerk_math.max_reachable_cruise_v(
        v_start=100.0, v_end=100.0, a_max=5000.0, j_max=100000.0,
        L=L, v_cruise_cap=1e9,
    )
    expected = jerk_math.reachable_v_end(
        v_start=100.0, a_max=5000.0, j_max=100000.0, L=L * 0.5,
    )
    assert v == pytest.approx(expected, rel=1e-6)


def test_max_cruise_v_bed_mesh_crash_inputs():
    # The exact numbers from the bed_mesh crash. start_v=374.7, end_v=469.8,
    # L=1.143, a=70000, j=500000. Under the trapezoidal cap
    # (sqrt(2*a*L) and cousins) this let infeasible cruise_v through;
    # max_reachable_cruise_v MUST return something feasible for set_junction.
    v = jerk_math.max_reachable_cruise_v(
        v_start=374.7, v_end=469.8, a_max=70000.0, j_max=500000.0,
        L=1.143, v_cruise_cap=469.8,
    )
    # Feasibility: reachable_v_end(v_start, a, j, L_accel) >= v must hold
    # for some 0 <= L_accel <= L, and reachable_v_end(v_end, a, j, L - L_accel) >= v.
    # The bisection finds the crossover; we just check the value is no
    # greater than either endpoint's reach-from-L.
    assert v <= jerk_math.reachable_v_end(374.7, 70000.0, 500000.0, 1.143) + 1e-6
    assert v <= jerk_math.reachable_v_end(469.8, 70000.0, 500000.0, 1.143) + 1e-6
    # And the cruise_v returned must be such that v_start itself is reachable
    # from v (reverse direction): that is, v <= v_start or the decel fits.
    # For this input L is far too short to reach 469.8 from 374.7 -- the
    # answer must clip below 469.8.
    assert v < 469.8


def test_max_cruise_v_bed_mesh_roundtrip_through_jerk_profile():
    # The acceptance test: the returned cruise_v MUST be feasible under
    # jerk_profile.compute_profile. This is the regression gate for the
    # bed_mesh crash.
    from klippy.chelper import jerk_profile as jp_mod
    v = jerk_math.max_reachable_cruise_v(
        v_start=374.7, v_end=469.8, a_max=70000.0, j_max=500000.0,
        L=1.143, v_cruise_cap=469.8,
    )
    # end_v cannot exceed cruise_v -- cap it.
    end_v = min(469.8, v)
    start_v = min(374.7, v)
    prof = jp_mod.compute_profile(
        v0=start_v, v1=end_v, v_peak=v,
        a_max=70000.0, j_max=500000.0, L=1.143,
    )
    assert prof.status == jp_mod.JP_OK, (
        f"Jerk profile rejected A5 cruise_v={v:.6f} start_v={start_v:.6f} "
        f"end_v={end_v:.6f} L=1.143 (status={prof.status})"
    )


def test_max_cruise_v_obeys_cap():
    v = jerk_math.max_reachable_cruise_v(
        v_start=0.0, v_end=0.0, a_max=5000.0, j_max=100000.0,
        L=100.0, v_cruise_cap=250.0,
    )
    assert v == pytest.approx(250.0, rel=1e-9)


def test_max_cruise_v_rejects_non_finite():
    with pytest.raises(ValueError):
        jerk_math.max_reachable_cruise_v(
            v_start=float("nan"), v_end=0.0, a_max=1.0, j_max=1.0,
            L=1.0, v_cruise_cap=1.0,
        )
