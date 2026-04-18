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
