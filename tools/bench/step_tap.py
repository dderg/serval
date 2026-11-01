"""Replay a host step tap (step_tap_mcu<N>.bin) into commanded step clocks.

The host writes the tap when SERVAL_STEP_TAP_DIR is set
(rust/motion-core/src/pump/step_tap.rs): an 8-byte magic, then 32-byte
records `kind u8 | flags u8 | u16 | oid u32 | mcu clock now u64 | payload`,
one per frame the mcu was sent, in send order. Replay follows
src/stepper_classic.c: `reset_step_clock` sets an oid's last step clock,
`set_next_step_dir` the direction of its next move, and `queue_step` emits
`count` steps at `last + interval`, then each further step `interval + add`
later than the previous interval.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MAGIC = b"SVTAP1\0\0"
RECORD_LEN = 32
HEADER = struct.Struct("<BBHIQ")
CLOCK_WRAP = 1 << 32

KIND_MCU = 1
KIND_LANE = 2
KIND_RESET_STEP_CLOCK = 3
KIND_SET_NEXT_STEP_DIR = 4
KIND_QUEUE_STEP = 5
KIND_QUEUE_STEP_HP = 6
KIND_BARRIER = 7
KIND_SET_POSITION = 8
KIND_PROJECTION = 9
KIND_HALT = 10


class TapError(RuntimeError):
    pass


@dataclass
class Lane:
    oid: int
    axis: int
    motor_in_axis: int
    microstep_distance: float
    invert_dir: bool
    clocks: list[np.ndarray] = field(default_factory=list)
    dirs: list[np.ndarray] = field(default_factory=list)

    def step_clocks(self) -> np.ndarray:
        return (
            np.concatenate(self.clocks)
            if self.clocks
            else np.empty(0, np.int64)
        )

    def step_dirs(self) -> np.ndarray:
        return np.concatenate(self.dirs) if self.dirs else np.empty(0, np.int8)


@dataclass
class Projection:
    oid: int
    freq: float
    offset: float


@dataclass
class Tap:
    mcu_id: int
    clock_freq: float
    lanes: dict[int, Lane]
    projections: list[Projection]


def _extend(low: int, near: int) -> int:
    return (
        near + ((low - near + CLOCK_WRAP // 2) % CLOCK_WRAP) - CLOCK_WRAP // 2
    )


class _OidState:
    def __init__(self) -> None:
        self.last_clock: int | None = None
        self.next_dir: int | None = None


def replay(data: bytes) -> Tap:
    if data[: len(MAGIC)] != MAGIC:
        raise TapError("not a step tap: bad magic")
    body = data[len(MAGIC) :]
    if len(body) % RECORD_LEN:
        raise TapError(f"step tap body of {len(body)} B is not whole records")
    mcu_id = None
    clock_freq = None
    lanes: dict[int, Lane] = {}
    projections: list[Projection] = []
    states: dict[int, _OidState] = {}
    for index in range(len(body) // RECORD_LEN):
        record = body[index * RECORD_LEN : (index + 1) * RECORD_LEN]
        kind, flags, _, oid, now = HEADER.unpack_from(record)
        payload = record[16:]
        if kind == KIND_MCU:
            mcu_id, _, clock_freq = struct.unpack_from("<IId", payload)
            continue
        if kind == KIND_LANE:
            axis, motor, step = struct.unpack_from("<IId", payload)
            lanes[oid] = Lane(oid, axis, motor, step, bool(flags))
            states[oid] = _OidState()
            continue
        if oid not in lanes:
            raise TapError(f"record {index}: oid {oid} has no lane header")
        state = states[oid]
        if kind == KIND_RESET_STEP_CLOCK:
            (low,) = struct.unpack_from("<I", payload)
            state.last_clock = _extend(low, now)
        elif kind == KIND_SET_NEXT_STEP_DIR:
            state.next_dir = payload[0]
        elif kind == KIND_QUEUE_STEP:
            interval, count, add = struct.unpack_from("<IHh", payload)
            if state.last_clock is None or state.next_dir is None:
                raise TapError(
                    f"record {index}: oid {oid} queue_step before its "
                    "reset_step_clock/set_next_step_dir"
                )
            k = np.arange(count, dtype=np.int64)
            clocks = (
                state.last_clock + (k + 1) * interval + add * k * (k + 1) // 2
            )
            lanes[oid].clocks.append(clocks)
            lanes[oid].dirs.append(np.full(count, state.next_dir, np.int8))
            state.last_clock = int(clocks[-1])
        elif kind == KIND_QUEUE_STEP_HP:
            raise TapError(
                f"record {index}: high-precision queue_step is not replayed"
            )
        elif kind == KIND_PROJECTION:
            freq, offset = struct.unpack_from("<dd", payload)
            projections.append(Projection(oid, freq, offset))
        elif kind == KIND_HALT:
            if state.last_clock is not None and state.last_clock > now:
                raise TapError(
                    f"record {index}: oid {oid} halted at clock {now} with steps "
                    f"queued to {state.last_clock}; the discarded tail is unknown"
                )
        elif kind not in (KIND_BARRIER, KIND_SET_POSITION):
            raise TapError(f"record {index}: unknown kind {kind}")
    if mcu_id is None or clock_freq is None:
        raise TapError("step tap has no mcu header")
    return Tap(mcu_id, clock_freq, lanes, projections)


def load(path: Path) -> Tap:
    return replay(path.read_bytes())
