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
