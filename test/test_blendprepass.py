# test/test_blendprepass.py
import math

import pytest

from klippy import blendprepass


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


def _collapser(toolhead=None):
    th = toolhead or _FakeToolhead()
    return blendprepass.CollinearCollapser(th, move_cls=_FakeMove)


def test_construct_and_flush_empty():
    c = _collapser()
    assert c.flush() == []


def test_feed_zero_length_move_passes_through():
    c = _collapser()
    th = c._toolhead
    # Construct a zero-length move directly; Move.__init__ flags it non-kinematic
    # but also gives it move_d=0 which is the step-1 branch we want to exercise.
    zero = _FakeMove(th, (0, 0, 0, 0), (0, 0, 0, 0), speed=100.0)
    assert zero.move_d == 0.0
    out = c.feed(zero)
    assert out == [zero]
    assert c._chain == []


def test_feed_non_kinematic_flushes_and_passes():
    c = _collapser()
    th = c._toolhead
    # Build a non-empty chain first
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m1)
    assert c._chain == [m1]
    # E-only move: XYZ identical, E delta present => is_kinematic_move=False
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = c.feed(eonly)
    assert out == [m1, eonly]
    assert c._chain == []


def test_first_kinematic_move_starts_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = c.feed(m)
    assert out == []
    assert c._chain == [m]


def test_flush_singleton_returns_move_unchanged():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    out = c.flush()
    assert out == [m]  # single-element chain: identity, not a built merge
    assert c._chain == []


def test_reset_discards_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    c.reset()
    assert c._chain == []
    assert c.flush() == []


def test_speed_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # Speed differs by 1% (> f_rel=1e-6)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=101.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    # Gate (a) rejects; chain flushes as singleton, m2 starts new chain.
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    # Same speed, same direction; different E-per-mm (> epm_rel=1%)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.1), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_within_tolerance_does_not_break():
    c = _collapser()
    th = c._toolhead
    # 0.5 mm E vs 0.5005 mm E over same 10 mm XYZ -> 0.1% diff (< epm_rel)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0005), speed=100.0)
    out = c.feed(m1)
    assert out == []
    # Gate (a) passes: speeds equal. Gate (b) passes: 0.1% < 1%.
    out = c.feed(m2)
    assert out == []
    assert c._chain == [m1, m2]


def test_two_collinear_moves_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    assert len(out) == 1
    merged = out[0]
    assert merged is not m1 and merged is not m2
    assert merged.start_pos == (0, 0, 0, 0)
    assert merged.end_pos[:3] == (20.0, 0.0, 0.0)
    assert merged.axes_d[3] == pytest.approx(1.0, abs=1e-12)
    assert merged.move_d == pytest.approx(20.0, abs=1e-12)


def test_non_collinear_moves_do_not_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # 1 mm perpendicular offset: well beyond 25 µm tolerance
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 1.0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_within_tolerance_offset_merges():
    c = _collapser()
    th = c._toolhead
    # 20 µm perpendicular offset from the A-to-C chord — within 25 µm tolerance.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 20e-3, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 20e-3, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Chord A(0,0,0)->C(20,0,0). Intermediate B=(10, 20e-3, 0). Perpendicular
    # distance from B to chord = 20 µm.
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    assert c._chain == [m1, m2]
