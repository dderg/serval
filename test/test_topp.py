# test/test_topp.py
#
# Plan 5 D7 tests for klippy/topp.py — time-optimal path parameterization
# forward+backward pass on a dense grid, fitted to a trapezoid-in-s profile.
import math

import pytest

from klippy import topp


def _const_cap(v):
    """v_cap_fn that returns a constant cap at every s."""
    return lambda s: v


def test_topp_all_cruise_degenerate():
    """v_in = v_out = v_cap_uniform -> flat profile, s_accel_end=0,
    s_decel_start=arc_length."""
    v_uniform = 50.0
    cruise_v, s_accel_end, s_decel_start = topp.topp_trapezoid(
        _const_cap(v_uniform), arc_length=1.0,
        v_in=v_uniform, v_out=v_uniform, a_max=5000.0, n_samples=128,
    )
    assert cruise_v == pytest.approx(v_uniform, rel=1e-6)
    assert s_accel_end == pytest.approx(0.0, abs=1e-9)
    assert s_decel_start == pytest.approx(1.0, abs=1e-9)


def test_topp_symmetric_corner_with_shoulder_dip():
    """v_in = v_out = moderate, v_cap dips at the middle -> symmetric
    trapezoid with cruise at the dip minimum.

    Constructs a v_cap(s) that is high at endpoints (100 mm/s) and dips to
    20 mm/s at the midpoint. v_in/v_out are chosen feasible: 40 mm/s
    requires (40^2 - 20^2) / (2 * 5000) = 0.12 mm of ramp to reach 20,
    comfortably within the 0.5 mm arc.

    TOPP must (a) find cruise_v ~= 20 (the dip min),
    (b) produce symmetric s_accel_end and (L - s_decel_start).
    """
    L = 0.5  # mm

    def v_cap(s):
        # Parabolic dip to 20 at the midpoint; 100 at both ends.
        center = L / 2.0
        dist = abs(s - center) / center
        return 20.0 + (100.0 - 20.0) * (dist * dist)

    cruise_v, s_accel_end, s_decel_start = topp.topp_trapezoid(
        v_cap, arc_length=L,
        v_in=40.0, v_out=40.0, a_max=5000.0, n_samples=200,
    )
    # Cruise must be ~= dip minimum (20 mm/s).
    assert cruise_v == pytest.approx(20.0, rel=0.05)
    # Symmetry: accel ramp length == decel ramp length.
    accel_len = s_accel_end
    decel_len = L - s_decel_start
    assert accel_len == pytest.approx(decel_len, rel=1e-2, abs=5e-3)
    # Both ramps fit inside the arc.
    assert accel_len >= 0.0
    assert decel_len >= 0.0
    assert s_accel_end <= s_decel_start


def test_topp_asymmetric_entry_exit():
    """v_in > v_out -> non-symmetric profile: longer decel ramp."""
    L = 0.5

    cruise_v, s_accel_end, s_decel_start = topp.topp_trapezoid(
        _const_cap(30.0), arc_length=L,
        v_in=30.0, v_out=5.0, a_max=1000.0, n_samples=128,
    )
    # cruise stays at v_cap.
    assert cruise_v == pytest.approx(30.0, rel=1e-6)
    # accel ramp is zero (v_in == cruise_v).
    assert s_accel_end == pytest.approx(0.0, abs=1e-9)
    # decel ramp from 30 to 5 at 1000 mm/s^2: (900 - 25) / 2000 = 0.4375 mm
    # but arc is only 0.5 mm so decel starts at s = 0.5 - 0.4375 = 0.0625.
    expected_decel_start = L - (30.0 * 30.0 - 5.0 * 5.0) / (2.0 * 1000.0)
    assert s_decel_start == pytest.approx(expected_decel_start, rel=1e-4)


def test_topp_boundary_violation_raises():
    """v_in > v_cap(0) -> error."""
    # v_cap constant at 10; v_in asks for 50. Infeasible.
    with pytest.raises(topp.TOPPError):
        topp.topp_trapezoid(
            _const_cap(10.0), arc_length=1.0,
            v_in=50.0, v_out=5.0, a_max=5000.0, n_samples=32,
        )


def test_topp_matches_worked_90deg_example():
    """Rough match to unified_v_of_s.md §8 worked example — 90 deg corner
    with v_cap_peak at shoulders.

    We approximate the 5-source v_cap profile with a two-shoulder dip:
    500 mm/s at endpoints, 17.7 mm/s at the shoulders (s/L = 0.25 and 0.75),
    with a central lift to 34.7 mm/s.

    Expected: cruise_v ~ 17 mm/s (shoulder minimum),  total_t ~ 12-13 ms
    for the 0.18 mm blend at a_max=5000 mm/s^2.
    """
    L = 0.18

    def v_cap(s):
        u = s / L
        # Two shoulders at u = 0.25 and u = 0.75, central lift, endpoints free.
        if u <= 0.02 or u >= 0.98:
            return 500.0
        # Triangular envelope across the blend.
        if u < 0.5:
            t = (u - 0.25) / 0.25
        else:
            t = (0.75 - u) / 0.25
        # t=0 at shoulders, t=+/-1 at endpoints (u=0, u=1) or center (u=0.5).
        if u == 0.25 or u == 0.75:
            return 17.7
        # Simplify: 17.7 at shoulders, 34.7 at center, 500 at endpoints.
        if u < 0.25:
            frac = u / 0.25
            return 500.0 * (1.0 - frac) + 17.7 * frac
        if u < 0.5:
            frac = (u - 0.25) / 0.25
            return 17.7 * (1.0 - frac) + 34.7 * frac
        if u < 0.75:
            frac = (u - 0.5) / 0.25
            return 34.7 * (1.0 - frac) + 17.7 * frac
        frac = (u - 0.75) / 0.25
        return 17.7 * (1.0 - frac) + 500.0 * frac

    # v_in / v_out chosen so the endpoint v-cap > v_in/v_out (feasible boundary)
    v_in = 30.0
    v_out = 30.0
    cruise_v, s_accel_end, s_decel_start = topp.topp_trapezoid(
        v_cap, arc_length=L, v_in=v_in, v_out=v_out, a_max=5000.0,
        n_samples=256,
    )
    # Cruise should drop to the shoulder minimum.
    assert cruise_v == pytest.approx(17.7, rel=0.05)

    # Total traversal time invariant.
    t_accel, t_decel_start, total_t = topp.topp_s_to_t_trapezoid(
        cruise_v, s_accel_end, s_decel_start,
        arc_length=L, v_in=v_in, v_out=v_out, a_max=5000.0,
    )
    # Rough envelope: at cruise ~17.7 mm/s the bulk of 0.18 mm takes
    # ~10 ms; the two 0.03 mm ramps add ~4 ms. Total 10-15 ms.
    assert total_t > 0.0
    assert 0.005 <= total_t <= 0.030


def test_topp_s_to_t_degenerate():
    """Invert an all-cruise profile -> time is purely arc_length / cruise_v."""
    t_accel, t_decel_start, total_t = topp.topp_s_to_t_trapezoid(
        cruise_v=100.0, s_accel_end=0.0, s_decel_start=1.0,
        arc_length=1.0, v_in=100.0, v_out=100.0, a_max=5000.0,
    )
    assert t_accel == pytest.approx(0.0, abs=1e-9)
    assert total_t == pytest.approx(0.01, rel=1e-6)
    assert t_decel_start == pytest.approx(total_t, rel=1e-6)
