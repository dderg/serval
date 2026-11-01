from __future__ import annotations

import struct

import pytest
import step_tap

OID = 7
FREQ = 84e6


def _record(
    kind: int, oid: int, now: int, payload: bytes, flags: int = 0
) -> bytes:
    return step_tap.HEADER.pack(kind, flags, 0, oid, now) + payload.ljust(
        16, b"\0"
    )


def _tap(*records: bytes) -> bytes:
    header = _record(step_tap.KIND_MCU, 0, 0, struct.pack("<IId", 0, 0, FREQ))
    lane = _record(
        step_tap.KIND_LANE, OID, 0, struct.pack("<IId", 3, 0, 0.0024), flags=1
    )
    return step_tap.MAGIC + header + lane + b"".join(records)


def _reset(low: int, now: int = 0) -> bytes:
    return _record(
        step_tap.KIND_RESET_STEP_CLOCK, OID, now, struct.pack("<I", low)
    )


def _dir(direction: int) -> bytes:
    return _record(step_tap.KIND_SET_NEXT_STEP_DIR, OID, 0, bytes([direction]))


def _steps(interval: int, count: int, add: int, now: int = 0) -> bytes:
    return _record(
        step_tap.KIND_QUEUE_STEP,
        OID,
        now,
        struct.pack("<IHh", interval, count, add),
    )


def test_queue_step_replays_the_mcu_interval_and_add_semantics():
    tap = step_tap.replay(
        _tap(
            _reset(1_000),
            _dir(1),
            _steps(100, 3, 10),
            _dir(0),
            _steps(50, 2, 0),
        )
    )
    lane = tap.lanes[OID]
    assert lane.axis == 3 and lane.invert_dir
    assert lane.step_clocks().tolist() == [1_100, 1_210, 1_330, 1_380, 1_430]
    assert lane.step_dirs().tolist() == [1, 1, 1, 0, 0]
    assert tap.clock_freq == FREQ


def test_negative_add_shrinks_each_interval():
    tap = step_tap.replay(_tap(_reset(0), _dir(1), _steps(1_000, 4, -100)))
    assert tap.lanes[OID].step_clocks().tolist() == [1_000, 1_900, 2_700, 3_400]


@pytest.mark.parametrize(
    ("low", "now", "expected"),
    [
        (50, (1 << 32) + 100, (1 << 32) + 50),
        (20, (1 << 32) - 10, (1 << 32) + 20),
    ],
)
def test_reset_clock_extends_to_the_wrap_nearest_the_send(low, now, expected):
    tap = step_tap.replay(_tap(_reset(low, now), _dir(1), _steps(10, 1, 0)))
    assert tap.lanes[OID].step_clocks().tolist() == [expected + 10]


def test_a_halt_with_steps_still_queued_is_unreplayable():
    halt = _record(step_tap.KIND_HALT, OID, 1_200, b"")
    with pytest.raises(step_tap.TapError, match="halted"):
        step_tap.replay(_tap(_reset(1_000), _dir(1), _steps(100, 3, 0), halt))


def test_a_halt_after_the_last_step_is_harmless():
    halt = _record(step_tap.KIND_HALT, OID, 5_000, b"")
    tap = step_tap.replay(_tap(_reset(1_000), _dir(1), _steps(100, 3, 0), halt))
    assert tap.lanes[OID].step_clocks().size == 3


def test_steps_before_a_clock_reset_fail():
    with pytest.raises(step_tap.TapError, match="before its"):
        step_tap.replay(_tap(_dir(1), _steps(100, 1, 0)))


def test_high_precision_frames_are_refused():
    hp = _record(step_tap.KIND_QUEUE_STEP_HP, OID, 0, b"")
    with pytest.raises(step_tap.TapError, match="high-precision"):
        step_tap.replay(_tap(_reset(0), _dir(1), hp))
