from __future__ import annotations

import capture_events
import numpy as np
import pytest

CHANNELS = capture_events.CHANNEL_COUNT
TICKS_PER_BLOCK = 8400


def _varint(value: int) -> bytes:
    out = bytearray()
    while value >= 0x80:
        out.append(value & 0x7F | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def _zigzag(value: int) -> bytes:
    return _varint((value << 1) ^ (value >> 63))


def _encode(
    steps: list[list[int]],
    dirs: list[list[tuple[int, int]]],
    initial_dir: int,
    anchor_every: int = 3,
) -> bytes:
    last = max(
        [t for channel in steps for t in channel]
        + [t for channel in dirs for t, _ in channel]
    )
    blocks = last // TICKS_PER_BLOCK + 1
    out = bytearray([initial_dir, 0])
    last_step = [0] * CHANNELS
    last_interval = [0] * CHANNELS
    last_dir = [0] * CHANNELS
    for block in range(blocks):
        end = (block + 1) * TICKS_PER_BLOCK
        lo = block * TICKS_PER_BLOCK
        block_steps = [[t for t in c if lo <= t < end] for c in steps]
        block_dirs = [[d for d in c if lo <= d[0] < end] for c in dirs]
        mask = 0x80 if block % anchor_every == 0 else 0
        for channel in range(CHANNELS):
            if block_steps[channel] or block_dirs[channel]:
                mask |= 1 << channel
        out.append(mask)
        if mask & 0x80:
            out += end.to_bytes(8, "little") + (block * 3).to_bytes(2, "little")
            out += bytes([block % 5] * CHANNELS)
        for channel in range(CHANNELS):
            if not mask & (1 << channel):
                continue
            out += _varint(len(block_steps[channel]))
            out += _varint(len(block_dirs[channel]))
            for t in block_steps[channel]:
                interval = t - last_step[channel]
                out += _zigzag(interval - last_interval[channel])
                last_step[channel], last_interval[channel] = t, interval
            for t, level in block_dirs[channel]:
                out += _varint(t - last_dir[channel]) + bytes([level])
                last_dir[channel] = t
    return bytes(out)


STEPS = [
    [1_000, 1_743, 2_486, 3_229, 40_000, 40_700, 41_390],
    [5_000, 90_000],
    [],
    [12_345, 12_400, 12_455, 30_000],
]
DIRS = [[(20_000, 0), (60_000, 1)], [], [], [(20_000, 1)]]
INITIAL_DIR = 0b0001


def test_round_trip_recovers_every_edge_and_its_sign():
    decoded = capture_events.decode(_encode(STEPS, DIRS, INITIAL_DIR))
    assert decoded.truncated_tail_bytes == 0
    assert decoded.blocks == 11
    assert decoded.max_drift_cycles == 27
    assert decoded.max_skew_wander_ticks == [4] * CHANNELS
    for channel, expected in zip(decoded.channels, STEPS):
        assert channel.step_ticks.tolist() == expected
    a = decoded.channels[0]
    assert a.dir_ticks.tolist() == [20_000, 60_000]
    assert a.step_signs().tolist() == [1, 1, 1, 1, -1, -1, -1]
    assert decoded.channels[3].step_signs().tolist() == [-1, -1, -1, 1]


def test_binned_deltas_carry_the_signed_counts():
    deltas = capture_events.decode(
        _encode(STEPS, DIRS, INITIAL_DIR)
    ).binned_deltas()
    assert deltas.sum(axis=0).tolist() == [1, -2, 0, -2]
    assert deltas[0, 0] == 2


def test_a_stream_cut_mid_block_keeps_the_decoded_prefix():
    data = _encode(STEPS, DIRS, INITIAL_DIR)
    decoded = capture_events.decode(data[:-3])
    assert decoded.truncated_tail_bytes > 0
    assert (
        decoded.channels[0].step_ticks.tolist()
        == STEPS[0][: decoded.channels[0].step_ticks.size]
    )


def test_dir_edge_too_close_to_a_step_is_unresolvable():
    dirs = [[(1_743 + 4, 0)], [], [], []]
    with pytest.raises(capture_events.EventDecodeError, match="not resolvable"):
        capture_events.decode(_encode(STEPS, dirs, INITIAL_DIR))


def test_dir_edge_to_its_current_level_fails():
    dirs = [[(20_000, 1)], [], [], []]
    with pytest.raises(capture_events.EventDecodeError, match="current level"):
        capture_events.decode(_encode(STEPS, dirs, INITIAL_DIR))


def test_edge_after_its_block_anchor_fails():
    data = bytearray(_encode([[100], [], [], []], [[], [], [], []], 0))
    data[3:11] = (50).to_bytes(8, "little")
    with pytest.raises(capture_events.EventDecodeError, match="anchor"):
        capture_events.decode(bytes(data))


def test_reserved_mask_bits_fail():
    with pytest.raises(capture_events.EventDecodeError, match="reserved"):
        capture_events.decode(bytes([0, 0, 0x10]))


def test_step_signs_follow_the_dir_level_in_force():
    channel = capture_events.Channel(
        step_ticks=np.array([10, 100, 200]),
        dir_ticks=np.array([150]),
        dir_levels=np.array([1], np.int8),
        initial_dir=0,
    )
    assert channel.step_signs().tolist() == [-1, -1, 1]
