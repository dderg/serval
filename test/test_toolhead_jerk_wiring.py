# test/test_toolhead_jerk_wiring.py
"""Phase A2c — jerk-aware toolhead wiring tests."""
from __future__ import annotations

import math

import pytest


class _FakeToolhead:
    def __init__(self, **kw):
        self.max_velocity = kw.get("max_velocity", 500.0)
        self.max_accel = kw.get("max_accel", 5000.0)
        self.max_jerk = kw.get("max_jerk", 100000.0)
        self.max_accel_to_decel = kw.get("max_accel_to_decel", 5000.0)
        self.min_cruise_ratio = kw.get("min_cruise_ratio", 0.0)
        class _K:
            def check_move(self, m): pass
        class _E:
            def check_move(self, m): pass
            def calc_junction(self, *_a): return 1e18
        self.kin = _K()
        self.extruder = _E()


def test_toolhead_max_jerk_default_loaded():
    # Config-driven ToolHead bootstrap is covered in Task 6 end-to-end test;
    # this is a placeholder so the file has a real slot for the integration case.
    pytest.skip("Config-driven ToolHead bootstrap is covered in Task 6 end-to-end test")


def test_fake_toolhead_has_max_jerk():
    """Sanity: _FakeToolhead mirrors the real ToolHead's jerk attribute."""
    th = _FakeToolhead()
    assert th.max_jerk == 100000.0
    th2 = _FakeToolhead(max_jerk=250000.0)
    assert th2.max_jerk == 250000.0


def test_move_captures_j_max_from_toolhead():
    """Move.__init__ must snapshot toolhead.max_jerk as self.j_max."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_jerk=250000.0)
    m = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    assert m.j_max == 250000.0


def test_move_j_max_unchanged_by_limit_speed():
    """limit_speed caps velocity/accel, but j_max is a toolhead property."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_jerk=150000.0)
    m = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m.limit_speed(50.0, 1000.0)
    assert m.j_max == 150000.0


def test_move_reachable_v_from_v_end_matches_jerk_math():
    """Move.reachable_v_from_v_end must delegate to jerk_math.reachable_v_end
    using self.accel, self.j_max, self.move_d."""
    from klippy import jerk_math
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=1000.0)  # move_d = 50 mm
    v_end = 200.0
    expected = jerk_math.reachable_v_end(
        v_start=v_end, a_max=m.accel, j_max=m.j_max, L=m.move_d
    )
    assert m.reachable_v_from_v_end(v_end) == pytest.approx(expected, rel=1e-12)


def test_move_reachable_v_zero_distance_returns_v_end():
    from klippy.toolhead import Move
    th = _FakeToolhead()
    m = Move(th, (0, 0, 0, 0), (1e-6, 0, 0, 0), speed=10.0)  # ~0 move_d
    # reachable_v_from_v_end(v_end) at near-zero L ≈ v_end.
    assert m.reachable_v_from_v_end(50.0) == pytest.approx(50.0, rel=1e-6)


def test_lookahead_flush_uses_jerk_reachable():
    """Two moves: stop at end. Reverse pass must compute max_start_v
    via jerk_math, not 2*a*L.

    Setup: move A (40 mm) → move B (10 mm) → stop. Under trapezoid
    (2*a*L) math move B's reachable start_v² = 2*5000*10 = 1e5, so
    start_v(B) = sqrt(1e5) = 316.2 mm/s. Under jerk_math with
    a=5000, j=100000, L=10 mm starting from v_end=0, regime is
    triangular: u³ = L*sqrt(j) = 10 * sqrt(1e5) ≈ 3162.3, u ≈
    14.68, dv = u² ≈ 215.5, so reachable_v_end(0, ..., 10) ≈ 215.5
    — materially LOWER than 316.2.
    """
    import math as _m
    from klippy.toolhead import Move, LookAheadQueue
    from klippy import jerk_math

    captured = []

    class _StubToolhead:
        def __init__(self):
            self.max_velocity = 500.0
            self.max_accel = 5000.0
            self.max_jerk = 100000.0
            self.max_accel_to_decel = 5000.0
            class _K:
                def check_move(self, m): pass
            class _E:
                def check_move(self, m): pass
                def calc_junction(self, *_a): return 1e18
            self.kin = _K()
            self.extruder = _E()

        def _process_moves(self, moves):
            captured.extend(moves)

    th = _StubToolhead()
    lookahead = LookAheadQueue(th)
    # Colinear moves so calc_junction gives cos_theta=1 (straight
    # junction with no centripetal cap).
    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=500.0)
    m_b = Move(th, (40, 0, 0, 0), (50, 0, 0, 0), speed=500.0)
    lookahead.queue.extend([m_a, m_b])
    m_b.calc_junction(m_a)
    lookahead.flush(lazy=False)
    expected_start_v = jerk_math.reachable_v_end(
        v_start=0.0, a_max=5000.0, j_max=100000.0, L=10.0,
    )
    assert m_b.start_v == pytest.approx(expected_start_v, rel=1e-9)
    assert m_b.end_v == pytest.approx(0.0, abs=1e-12)


def test_set_junction_stores_jerk_profile():
    """set_junction must call jerk_profile.compute_profile and store the
    Profile on self.jerk_profile."""
    from klippy.chelper import jerk_profile as jp_mod
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=200.0)
    # v_start=0, cruise=200, end=0, accel=5000, j=1e5, L=50.
    m.set_junction(start_v2=0.0, cruise_v2=200.0 ** 2, end_v2=0.0)
    assert hasattr(m, "jerk_profile")
    prof = m.jerk_profile
    assert prof.status == jp_mod.JP_OK
    assert prof.segments, "profile must have at least one segment"


def test_set_junction_phase_times_sum_to_jerk_profile_total():
    """accel_t + cruise_t + decel_t must equal sum(seg.T for seg in profile)."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=300.0)
    m.set_junction(0.0, 300.0 ** 2, 0.0)
    profile_total = sum(s.T for s in m.jerk_profile.segments)
    legacy_total = m.accel_t + m.cruise_t + m.decel_t
    assert legacy_total == pytest.approx(profile_total, rel=1e-9)


def test_set_junction_integrated_distance_equals_move_d():
    """Given the back-compat fields, the trapezoidal integral
        accel_d + cruise_d + decel_d
    must equal move_d (round-trip integrity for the emit path)."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=300.0)
    m.set_junction(0.0, 300.0 ** 2, 0.0)
    # Trapezoid-in-v integral (matches extruder.move / append_trapezoid_as_quintic).
    accel_d = (m.start_v + m.cruise_v) * 0.5 * m.accel_t
    cruise_d = m.cruise_v * m.cruise_t
    decel_d = (m.cruise_v + m.end_v) * 0.5 * m.decel_t
    assert accel_d + cruise_d + decel_d == pytest.approx(m.move_d, rel=1e-9)


def test_set_junction_populates_start_v_cruise_v_end_v():
    """start_v, cruise_v, end_v must match sqrt(input v²)."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=300.0)
    m.set_junction(100.0 ** 2, 300.0 ** 2, 50.0 ** 2)
    assert m.start_v == pytest.approx(100.0, rel=1e-9)
    assert m.cruise_v == pytest.approx(300.0, rel=1e-9)
    assert m.end_v == pytest.approx(50.0, rel=1e-9)
