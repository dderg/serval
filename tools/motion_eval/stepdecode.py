"""Decode the binary host->MCU stream written by `klippy -o` into steps.

The MCU executes `queue_step oid interval count add` as `count` steps, the
first `interval` ticks after the previous step and each later one `add`
ticks further apart than the one before (src/stepper.c).
"""

from __future__ import annotations

import dataclasses

import numpy as np

from klippy import msgproto

CLOCK_MASK = 0xFFFFFFFF


@dataclasses.dataclass
class StepTrace:
    clocks: np.ndarray
    positions: np.ndarray


class _OidState:
    def __init__(self):
        self.last_clock = 0
        self.direction = None
        self.position = 0
        self.clock_chunks = []
        self.position_chunks = []

    def queue_step(self, oid, interval, count, add):
        if self.direction is None:
            raise ValueError(
                "queue_step before set_next_step_dir (oid %d)" % oid
            )
        k = np.arange(count, dtype=np.int64)
        clocks = self.last_clock + (k + 1) * interval + add * k * (k + 1) // 2
        positions = self.position + self.direction * (k + 1)
        self.clock_chunks.append(clocks)
        self.position_chunks.append(positions)
        self.last_clock = int(clocks[-1])
        self.position = int(positions[-1])

    def reset_step_clock(self, clock):
        self.last_clock += (clock - self.last_clock) & CLOCK_MASK

    def trace(self):
        if not self.clock_chunks:
            empty = np.zeros(0, dtype=np.int64)
            return StepTrace(empty, empty)
        return StepTrace(
            np.concatenate(self.clock_chunks),
            np.concatenate(self.position_chunks),
        )


def decode(dictionary_path, output_path, invert_dir_by_oid):
    parser = msgproto.MessageParser()
    with open(dictionary_path, "rb") as f:
        parser.process_identify(f.read(), decompress=False)
    with open(output_path, "rb") as f:
        data = bytearray(f.read())
    states = {oid: _OidState() for oid in invert_dir_by_oid}
    pos = 0
    while pos < len(data):
        msglen = parser.check_packet(data[pos:])
        if msglen <= 0:
            raise ValueError(
                "corrupt packet at byte %d of %s" % (pos, output_path)
            )
        for params in _parse_block(parser, data[pos : pos + msglen]):
            _apply(params, states, invert_dir_by_oid)
        pos += msglen
    return {oid: state.trace() for oid, state in states.items()}


def _parse_block(parser, block):
    pos = msgproto.MESSAGE_HEADER_SIZE
    end = len(block) - msgproto.MESSAGE_TRAILER_SIZE
    while pos < end:
        msgid, _ = parser.msgid_parser.parse(block, pos)
        fmt = parser.messages_by_id.get(msgid)
        if fmt is None:
            raise ValueError("unknown message id %d in output stream" % msgid)
        params, pos = fmt.parse(block, pos)
        params["#name"] = fmt.name
        yield params


def _apply(params, states, invert_dir_by_oid):
    name = params["#name"]
    if name == "queue_step_hp":
        raise NotImplementedError(
            "queue_step_hp (high precision stepcompress) is not decoded"
        )
    if name not in ("queue_step", "set_next_step_dir", "reset_step_clock"):
        return
    oid = params["oid"]
    state = states.get(oid)
    if state is None:
        return
    if name == "queue_step":
        state.queue_step(
            oid, params["interval"], params["count"], params["add"]
        )
    elif name == "set_next_step_dir":
        forward = bool(params["dir"]) != invert_dir_by_oid[oid]
        state.direction = 1 if forward else -1
    else:
        state.reset_step_clock(params["clock"])
