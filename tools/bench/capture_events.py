"""Decode an event-mode capture (events.bin) into per-channel edge times.

Stream (little-endian, LEB128 varints, zigzag for signed):
    u8 initial DIR level mask | u8 both-edges mask | block*
    block := u8 mask [anchor if mask & 0x80]
             { per channel bit set in mask (low nibble, ascending):
               varint n_step | varint n_dir
               | n_step * zigzag(interval - previous interval)
               | n_dir * (varint delta from previous DIR edge | u8 new level) }
    anchor := u64 master time | u16 max clock drift (core cycles)
              | CHANNEL_COUNT * u8 max timer skew wander (ticks)
One block per 100 us firmware snapshot. Times are 84 MHz ticks since the
four capture timers started together. An anchor is the master timer's
extended time at that snapshot; every edge in the block precedes it. Its
drift/wander fields are the worst read-to-read disagreement between the
core cycle counter and the master timer, and between each timer and the
master, since the previous anchor.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from capture import CHANNEL_COUNT, EVENT_TICK_HZ, TICK_HZ

ANCHOR_FLAG = 0x80
ANCHOR_LEN = 8 + 2 + CHANNEL_COUNT
CHANNEL_BITS = (1 << CHANNEL_COUNT) - 1
COUNT_TICK = EVENT_TICK_HZ // TICK_HZ
MIN_DIR_STEP_SEPARATION_TICKS = 9


class EventDecodeError(RuntimeError):
    pass


@dataclass
class Channel:
    step_ticks: np.ndarray
    dir_ticks: np.ndarray
    dir_levels: np.ndarray
    initial_dir: int

    def step_signs(self) -> np.ndarray:
        if self.dir_ticks.size == 0:
            level = 1 if self.initial_dir else -1
            return np.full(self.step_ticks.size, level, np.int8)
        edge = np.searchsorted(self.dir_ticks, self.step_ticks, "right") - 1
        levels = np.where(
            edge >= 0, self.dir_levels[np.maximum(edge, 0)], self.initial_dir
        )
        return np.where(levels != 0, 1, -1).astype(np.int8)

    def min_dir_step_separation_ticks(self) -> int | None:
        if self.step_ticks.size == 0 or self.dir_ticks.size == 0:
            return None
        index = np.searchsorted(self.step_ticks, self.dir_ticks)
        before = self.step_ticks[np.maximum(index - 1, 0)]
        after = self.step_ticks[np.minimum(index, self.step_ticks.size - 1)]
        return int(
            np.minimum(
                np.abs(self.dir_ticks - before), np.abs(after - self.dir_ticks)
            ).min()
        )


@dataclass
class EventCapture:
    channels: list[Channel]
    both_edges_mask: int
    blocks: int
    truncated_tail_bytes: int
    max_drift_cycles: int
    max_skew_wander_ticks: list[int]

    def binned_deltas(self) -> np.ndarray:
        last = max(
            (int(c.step_ticks[-1]) for c in self.channels if c.step_ticks.size),
            default=0,
        )
        deltas = np.zeros((last // COUNT_TICK + 1, CHANNEL_COUNT), np.int64)
        for index, channel in enumerate(self.channels):
            np.add.at(
                deltas[:, index],
                channel.step_ticks // COUNT_TICK,
                channel.step_signs(),
            )
        if np.abs(deltas).max(initial=0) > 127:
            raise EventDecodeError("more than 127 steps in one 25 us bin")
        return deltas.astype(np.int8)


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.offset = 0

    def byte(self) -> int:
        value = self.data[self.offset]
        self.offset += 1
        return value

    def varint(self) -> int:
        value = 0
        shift = 0
        while True:
            byte = self.byte()
            value |= (byte & 0x7F) << shift
            if byte < 0x80:
                return value
            shift += 7
            if shift > 63:
                raise EventDecodeError(f"varint overflow at byte {self.offset}")

    def zigzag(self) -> int:
        value = self.varint()
        return (value >> 1) ^ -(value & 1)


def decode(data: bytes) -> EventCapture:
    if len(data) < 2:
        raise EventDecodeError("event stream shorter than its start record")
    reader = _Reader(data)
    initial_dir = reader.byte()
    both_edges_mask = reader.byte()
    steps: list[list[int]] = [[] for _ in range(CHANNEL_COUNT)]
    dirs: list[list[int]] = [[] for _ in range(CHANNEL_COUNT)]
    levels: list[list[int]] = [[] for _ in range(CHANNEL_COUNT)]
    last_step = [0] * CHANNEL_COUNT
    last_interval = [0] * CHANNEL_COUNT
    last_dir = [0] * CHANNEL_COUNT
    level_now = [(initial_dir >> i) & 1 for i in range(CHANNEL_COUNT)]
    blocks = 0
    last_anchor = -1
    max_drift = 0
    max_wander = [0] * CHANNEL_COUNT
    block_start = reader.offset
    try:
        while reader.offset < len(data):
            block_start = reader.offset
            mask = reader.byte()
            if mask & ~(ANCHOR_FLAG | CHANNEL_BITS):
                raise EventDecodeError(
                    f"block {blocks}: reserved mask bits in {mask:#04x}"
                )
            anchor = None
            if mask & ANCHOR_FLAG:
                if reader.offset + ANCHOR_LEN > len(data):
                    raise IndexError
                record = data[reader.offset : reader.offset + ANCHOR_LEN]
                reader.offset += ANCHOR_LEN
                anchor = int.from_bytes(record[:8], "little")
                max_drift = max(
                    max_drift, int.from_bytes(record[8:10], "little")
                )
                max_wander = [
                    max(m, w) for m, w in zip(max_wander, record[10:])
                ]
                if anchor <= last_anchor:
                    raise EventDecodeError(
                        f"block {blocks}: anchor {anchor} not after {last_anchor}"
                    )
                last_anchor = anchor
            for channel in range(CHANNEL_COUNT):
                if not mask & (1 << channel):
                    continue
                n_step = reader.varint()
                n_dir = reader.varint()
                for _ in range(n_step):
                    last_interval[channel] += reader.zigzag()
                    if last_interval[channel] <= 0:
                        raise EventDecodeError(
                            f"block {blocks} channel {channel}: step interval "
                            f"{last_interval[channel]}"
                        )
                    last_step[channel] += last_interval[channel]
                    steps[channel].append(last_step[channel])
                for _ in range(n_dir):
                    delta = reader.varint()
                    level = reader.byte()
                    if delta <= 0 or level not in (0, 1):
                        raise EventDecodeError(
                            f"block {blocks} channel {channel}: DIR record "
                            f"delta={delta} level={level}"
                        )
                    if level == level_now[channel]:
                        raise EventDecodeError(
                            f"block {blocks} channel {channel}: DIR edge to "
                            f"its current level {level}"
                        )
                    last_dir[channel] += delta
                    level_now[channel] = level
                    dirs[channel].append(last_dir[channel])
                    levels[channel].append(level)
                if (
                    anchor is not None
                    and max(last_step[channel], last_dir[channel]) > anchor
                ):
                    raise EventDecodeError(
                        f"block {blocks} channel {channel}: edge after the "
                        f"block's anchor {anchor}"
                    )
            blocks += 1
        truncated = 0
    except IndexError:
        truncated = len(data) - block_start
    channels = [
        Channel(
            step_ticks=np.asarray(steps[i], np.int64),
            dir_ticks=np.asarray(dirs[i], np.int64),
            dir_levels=np.asarray(levels[i], np.int8),
            initial_dir=(initial_dir >> i) & 1,
        )
        for i in range(CHANNEL_COUNT)
    ]
    for index, channel in enumerate(channels):
        separation = channel.min_dir_step_separation_ticks()
        if (
            separation is not None
            and separation < MIN_DIR_STEP_SEPARATION_TICKS
        ):
            raise EventDecodeError(
                f"channel {index}: a DIR edge lies {separation} ticks from a "
                "STEP edge; their order is not resolvable"
            )
    return EventCapture(
        channels, both_edges_mask, blocks, truncated, max_drift, max_wander
    )


def load(run_dir: Path) -> EventCapture:
    return decode((run_dir / "events.bin").read_bytes())
