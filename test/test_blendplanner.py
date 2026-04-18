# test/test_blendplanner.py
import math
import random

import pytest

from klippy import blendplanner


class _FakeCheckMove:
    def __init__(self, exc=None):
        self.calls = []
        self._exc = exc

    def check_move(self, move):
        self.calls.append(move)
        if self._exc is not None:
            raise self._exc


class _FakeToolhead:
    def __init__(self, **overrides):
        self.max_velocity = overrides.get("max_velocity", 500.0)
        self.max_accel = overrides.get("max_accel", 10000.0)
        self.max_accel_to_decel = overrides.get("max_accel_to_decel", 10000.0)
        self.junction_deviation = overrides.get("junction_deviation", 0.01)
        self.corner_deviation = overrides.get("corner_deviation", 50e-3)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
    """Reimplements klippy.toolhead.Move.__init__ without pulling pyserial."""

    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.axes_d = axes_d
        move_d = math.sqrt(sum(d * d for d in axes_d[:3]))
        if move_d < 0.000000001:
            self.end_pos = (
                start_pos[0],
                start_pos[1],
                start_pos[2],
                end_pos[3],
            )
            axes_d[0] = axes_d[1] = axes_d[2] = 0.0
            move_d = abs(axes_d[3])
            inv_move_d = 1.0 / move_d if move_d else 0.0
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / move_d
        self.move_d = move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity if velocity else 0.0
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity ** 2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed else 0.0
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed ** 2)


def _blender(toolhead=None, max_chord_err=None):
    th = toolhead or _FakeToolhead()
    return blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=max_chord_err
    )


def test_construct_and_flush_empty():
    b = _blender()
    assert b.flush() == []
    assert b.peek_buffered() == []
    assert b.polyline_moves_emitted == 0
    assert b.blends_emitted == 0


def test_feed_first_move_buffers():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = b.feed(m)
    assert out == []
    assert b._prev is m
    assert b.peek_buffered() == [m]


def test_flush_drains_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    out = b.flush()
    assert out == [m]
    assert b._prev is None


def test_reset_drops_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    b.reset()
    assert b._prev is None
    assert b.flush() == []


def test_feed_non_kinematic_flushes_and_passes():
    b = _blender()
    th = b._toolhead
    m_kin = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m_kin)
    # E-only: XYZ identical, E delta present
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = b.feed(eonly)
    assert out == [m_kin, eonly]
    assert b._prev is None


def test_feed_collinear_pair_passes_through_with_rebuffer():
    b = _blender()
    th = b._toolhead
    # Two exactly collinear moves along +X: blend_from_moves returns None.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # Collinear: emit prev unchanged, buffer next. No velocity cap imposed.
    assert out == [m1]
    assert b._prev is m2
    assert m1.next_junction_v2 == 999999999.9  # unchanged


def test_feed_uturn_emits_prev_with_zero_next_junction():
    b = _blender()
    th = b._toolhead
    # 180° reversal: +X then -X
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # U-turn: emit prev with limit_next_junction_speed(0); buffer next.
    assert out == [m1]
    assert m1.next_junction_v2 == 0.0
    assert b._prev is m2


def _state_src_dst_pair():
    """Build a (src, dst) pair where src is a 'full-length' parent and dst a
    truncated child constructed via the Move ctor against the same toolhead."""
    th = _FakeToolhead()
    src = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 1.0), speed=200.0)
    # Simulate caller mutations on src:
    src.timing_callbacks.append(lambda t: None)
    src.next_junction_v2 = 42.0
    src.max_cruise_v2 = 150.0 ** 2
    src.junction_deviation = 0.05
    src.accel = 5000.0
    src.delta_v2 = 2.0 * src.move_d * src.accel
    src.smooth_delta_v2 = min(src.delta_v2, 2.0 * src.move_d * 2500.0)
    dst = _FakeMove(th, (0, 0, 0, 0), (4, 0, 0, 0.4), speed=200.0)
    return th, src, dst


def test_copy_caller_state_transfers_caller_intent_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # Caller-intent fields pinned verbatim.
    assert dst.timing_callbacks == src.timing_callbacks
    assert dst.timing_callbacks is not src.timing_callbacks  # copy, not alias
    assert dst.next_junction_v2 == 42.0
    assert dst.max_cruise_v2 == 150.0 ** 2
    assert dst.junction_deviation == 0.05
    assert dst.accel == 5000.0


def test_copy_caller_state_recomputes_length_derived_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # delta_v2 recomputed from NEW move_d (4 mm) and pinned accel (5000).
    assert dst.delta_v2 == pytest.approx(2.0 * 4.0 * 5000.0)
    # smooth_delta_v2 preserves the parent ratio; src had smooth/delta = 0.5
    # (max_accel_to_decel/accel = 2500/5000), so dst should follow.
    ratio = src.smooth_delta_v2 / src.delta_v2
    assert dst.smooth_delta_v2 == pytest.approx(
        min(dst.delta_v2, 2.0 * 4.0 * dst.accel * ratio)
    )
    # min_move_t = move_d / sqrt(max_cruise_v2) = 4 / 150 = 0.02667
    assert dst.min_move_t == pytest.approx(4.0 / 150.0)


def test_copy_caller_state_handles_zero_delta_v2():
    th, src, dst = _state_src_dst_pair()
    src.delta_v2 = 0.0
    src.smooth_delta_v2 = 0.0
    blendplanner._copy_caller_state(src, dst)
    # Falls back to ratio=1.0 when src.delta_v2 is zero; dst.smooth_delta_v2
    # collapses to dst.delta_v2 via the min().
    assert dst.smooth_delta_v2 == pytest.approx(dst.delta_v2)


def test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # Two 10mm moves meeting at a 90° corner at (10,0,0).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert b.feed(m_prev) == []
    out = b.feed(m_next)
    # Emission: [trunc_prev, arc[0], ..., arc[N-1]]
    assert len(out) >= 2
    trunc_prev = out[0]
    arc_moves = out[1:]
    # trunc_prev shares start_pos with m_prev.
    assert trunc_prev.start_pos[:3] == m_prev.start_pos[:3]
    # trunc_prev ends before the vertex by arc.d_consumed along +X.
    # R_mid = 0.5 * min(10,10) * cot(45°) = 5. R_tol binds much smaller:
    # R_tol = 50e-3 * cos(45°)/(1-cos(45°)) ≈ 0.1207. So R = R_tol ≈ 0.1207,
    # d = R * tan(45°) ≈ 0.1207.
    d_expected = 50e-3 * (math.sqrt(2)/2) / (1 - math.sqrt(2)/2)
    assert trunc_prev.end_pos[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert trunc_prev.end_pos[1] == pytest.approx(0.0, abs=1e-9)
    # buffered next_trunc_head starts where the arc ends.
    assert b._prev is not None
    assert b._prev is not m_next
    nxt_head = b._prev
    assert nxt_head.start_pos[0] == pytest.approx(10.0, abs=1e-9)
    assert nxt_head.start_pos[1] == pytest.approx(d_expected, rel=1e-6)
    assert nxt_head.end_pos[:3] == (10.0, 10.0, 0.0)
    # Polyline points all lie on the arc within max_chord_err.
    # Arc center: m_prev.end_pos + R*n_hat where n_hat bisects inward.
    # At a 90° corner +X to +Y, center = vertex + R*(-1/sqrt2, 1/sqrt2) rotated;
    # simpler check: every arc_move endpoint must be within R + chord_err of center.
    # We compute center from arc.entry_pt + R in the direction (next-prev)/|...|.
    # For simplicity just verify that arc spans from near (10-d,0) to (10,d).
    first_pt = arc_moves[0].start_pos[:3]
    last_pt = arc_moves[-1].end_pos[:3]
    assert first_pt[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert last_pt[1] == pytest.approx(d_expected, rel=1e-6)
    # All arc moves share the same max_cruise_v2 (arc.v_cap^2 in this case).
    v_caps = [am.max_cruise_v2 for am in arc_moves]
    assert max(v_caps) - min(v_caps) < 1e-6
    # Instrumentation.
    assert b.blends_emitted == 1
    assert b.polyline_moves_emitted == len(arc_moves)


def test_e_conservation_through_blend():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # Drain buffered trunc_next_head.
    out += b.flush()
    total_e = sum(am.axes_d[3] for am in out)
    expected = m_prev.axes_d[3] + m_next.axes_d[3]
    assert total_e == pytest.approx(expected, rel=1e-9, abs=1e-12)


def test_asymmetric_segments_half_segment_rule_caps_consumption():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # 60° corner. Short segment = 2mm, long = 10mm. With LOOSE tolerance so
    # R_tol >> R_mid and the midpoint cap binds.
    # R_mid = 0.5 * min(2, 10) * cot(30°) = 0.5 * 2 * sqrt(3) = sqrt(3)
    # d = R * tan(30°) = sqrt(3) * (1/sqrt(3)) = 1.0 (= L_short / 2)
    th.corner_deviation = 10.0  # absurdly loose so R_tol does not bind
    angle = math.radians(60.0)
    m_prev = _FakeMove(th, (0, 0, 0, 0), (2, 0, 0, 0.1), speed=100.0)
    # Rotate next direction by 60° from +X.
    next_end = (
        2 + 10 * math.cos(angle),
        0 + 10 * math.sin(angle),
        0, 0.6,
    )
    m_next = _FakeMove(th, (2, 0, 0, 0.1), next_end, speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    trunc_prev = out[0]
    # trunc_prev.move_d should equal 2 - 1 = 1 mm (half-segment consumption).
    assert trunc_prev.move_d == pytest.approx(1.0, rel=1e-6)


def test_aggregate_kin_check_move_fires_on_representative_arc_move():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # The representative arc move was passed to kin.check_move exactly once.
    arc_moves = out[1:]
    assert len(th.kin.calls) == 1
    assert th.kin.calls[0] is arc_moves[0]


def test_aggregate_extruder_check_move_fires_when_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    # Extruder check_move called once on the representative arc move (E delta
    # is non-zero because both prev and next extrude).
    assert len(th.extruder.calls) == 1


def test_aggregate_extruder_check_move_skipped_when_not_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # E coordinate identical across prev and next (travel moves).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    assert len(th.extruder.calls) == 0


def test_arc_polyline_smooth_delta_v2_equals_delta_v2():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    for am in arc_moves:
        assert am.smooth_delta_v2 == pytest.approx(am.delta_v2, rel=1e-12)


def test_arc_polyline_speed_continuity_1ppm():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    v2s = [am.max_cruise_v2 for am in arc_moves]
    # All arc moves share the same cap to 1 ppm.
    assert (max(v2s) - min(v2s)) / max(v2s) < 1e-6
