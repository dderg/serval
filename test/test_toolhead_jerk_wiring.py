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


def test_build_quintic_payload_returns_expected_tuple_shape():
    """quintic_trapq_payload (populated by finalize_shape after set_junction)
    must be a 9-tuple matching QuinticBlendMove's payload contract:
      (phase_t_ends_tuple, total_t_baked, arc_length, v_cap_min,
       start_pos_xyz, coeff_tuple,
       legacy_t_accel_end, legacy_t_decel_start, legacy_total_t)

    Plan 9 A3: build_quintic_payload was renamed to build_unshaped_payload
    (3-tuple) + finalize_shape (packs 9-tuple into quintic_trapq_payload).
    """
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (1, 2, 3, 0), (51, 2, 3, 0), speed=200.0)
    m.set_junction(0.0, 200.0 ** 2, 0.0)
    payload = m.quintic_trapq_payload
    assert isinstance(payload, tuple)
    assert len(payload) == 9
    phase_t_ends, total_t, arc_length, v_cap_min, start_pos_xyz, coeff_tuple, \
        t_accel_end, t_decel_start, total_t_legacy = payload
    nonzero_segs = [s for s in m.jerk_profile.segments if s.T > 1e-12]
    assert len(phase_t_ends) == len(nonzero_segs)
    assert total_t == pytest.approx(sum(s.T for s in nonzero_segs), rel=1e-9)
    assert arc_length == pytest.approx(m.move_d, rel=1e-9)
    assert v_cap_min == pytest.approx(0.0, abs=1e-12)
    assert start_pos_xyz == (1.0, 2.0, 3.0)
    assert len(coeff_tuple) == len(phase_t_ends) * 15 * 4
    assert t_accel_end == pytest.approx(m.accel_t, rel=1e-9)
    assert t_decel_start == pytest.approx(m.accel_t + m.cruise_t, rel=1e-9)
    assert total_t_legacy == pytest.approx(
        m.accel_t + m.cruise_t + m.decel_t, rel=1e-9)


def test_build_quintic_payload_xy_polynomial_matches_build_jerk_profile():
    """The XY polynomial in the unshaped payload's coeff_tuple must match
    build_jerk_profile_as_quintic_coeffs output bit-for-bit in the
    .x/.y/.z slots.

    Plan 9 A3: uses _unshaped_payload (3-tuple) instead of the removed
    build_quintic_payload method."""
    from klippy.chelper.linear_quintic import build_jerk_profile_as_quintic_coeffs
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (1, 2, 3, 0), (51, 2, 3, 0), speed=200.0)
    m.set_junction(0.0, 200.0 ** 2, 0.0)
    phase_t_ends, _total_t, coeff_tuple = m._unshaped_payload
    n_phases = len(phase_t_ends)
    expected_n, expected_t_ends, expected_coeff = \
        build_jerk_profile_as_quintic_coeffs(
            m.jerk_profile, m.axes_r[:3], m.start_pos[:3])
    active_len = n_phases * 15 * 4
    expected_active = expected_coeff[:active_len]
    for i in range(n_phases):
        for k in range(15):
            for axis in (0, 1, 2):
                idx = (i * 15 + k) * 4 + axis
                assert coeff_tuple[idx] == expected_active[idx], (
                    f"mismatch at phase {i} coeff {k} axis {axis}")


def test_build_quintic_payload_pa_zero_when_no_pa():
    """With no PA configured, .e slot must be all zeros.

    Plan 9 A3: uses _unshaped_payload (3-tuple) instead of the removed
    build_quintic_payload method."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=200.0)
    m.set_junction(0.0, 200.0 ** 2, 0.0)
    phase_t_ends, _total_t, coeff_tuple = m._unshaped_payload
    n_phases = len(phase_t_ends)
    for i in range(n_phases):
        for k in range(15):
            idx = (i * 15 + k) * 4 + 3
            assert coeff_tuple[idx] == 0.0, (
                f"E slot nonzero with PA disabled at phase {i} coeff {k}")


def test_build_quintic_payload_pa_linear_fills_e_slot():
    """With k_pa > 0, the .e slot should carry a nontrivial polynomial.

    Plan 9 A3: uses build_unshaped_payload() directly instead of the
    removed build_quintic_payload method."""
    from klippy.toolhead import Move
    import klippy.blendplanner

    class _PAFakeToolhead(_FakeToolhead):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.extruder_cap_snapshot = None

    th = _PAFakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    orig = klippy.blendplanner._resolve_pa_dispatch
    klippy.blendplanner._resolve_pa_dispatch = lambda _th: ("linear", 0.05)
    try:
        m = Move(th, (0, 0, 0, 0), (50, 0, 0, 5), speed=200.0)
        m.set_junction(0.0, 200.0 ** 2, 0.0)
        phase_t_ends, _total_t, coeff_tuple = m.build_unshaped_payload()
        n_phases = len(phase_t_ends)
        e_values = []
        for i in range(n_phases):
            for k in range(15):
                idx = (i * 15 + k) * 4 + 3
                e_values.append(coeff_tuple[idx])
        assert any(abs(v) > 1e-12 for v in e_values), (
            "E slot all zero after PA compose with k_pa=0.05")
    finally:
        klippy.blendplanner._resolve_pa_dispatch = orig


def test_set_junction_populates_quintic_trapq_payload():
    """After set_junction, move.quintic_trapq_payload must be a valid
    9-tuple (populated via build_unshaped_payload + finalize_shape)."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=200.0)
    m.set_junction(0.0, 200.0 ** 2, 0.0)
    assert hasattr(m, "quintic_trapq_payload")
    payload = m.quintic_trapq_payload
    assert isinstance(payload, tuple)
    assert len(payload) == 9
    nonzero_segs = [s for s in m.jerk_profile.segments if s.T > 1e-12]
    assert len(payload[0]) == len(nonzero_segs)


def test_set_junction_quintic_payload_total_t_equals_sum_of_phase_times():
    """total_t_baked == accel_t + cruise_t + decel_t (all derived from
    jerk profile)."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=300.0)
    m.set_junction(0.0, 300.0 ** 2, 0.0)
    payload = m.quintic_trapq_payload
    assert payload[1] == pytest.approx(
        m.accel_t + m.cruise_t + m.decel_t, rel=1e-9)


def test_pure_e_move_skips_quintic_trapq_payload():
    """Pure-E (is_kinematic_move == False) moves must NOT populate
    quintic_trapq_payload — they route through the legacy trapezoid
    path in extruder.move for A2d scope."""
    from klippy.toolhead import Move
    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    # Zero XYZ displacement, nonzero E → pure-E move.
    m = Move(th, (0, 0, 0, 0), (0, 0, 0, 10), speed=100.0)
    assert m.is_kinematic_move is False
    m.set_junction(0.0, 100.0 ** 2, 0.0)
    assert not hasattr(m, "quintic_trapq_payload"), (
        "pure-E move must NOT get quintic_trapq_payload — A2d guards on "
        "is_kinematic_move, pure-E routes through the legacy trapezoid path"
    )
