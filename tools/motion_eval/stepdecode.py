"""Decode the binary host->MCU stream written by `klippy -o` into steps.

The MCU executes `queue_step oid interval count add` as `count` steps, the
first `interval` ticks after the previous step and each later one `add`
ticks further apart than the one before (src/stepper.c).

Clocks on the wire are 32 bit, and a step further than 2^32 ticks after the
previous one only has its interval modulo 2^32 on the wire. The absolute
64-bit clock of each command comes from the host's stepcompress history
(one entry per queue_step, captured by capture.py). Every message is checked
against its history entry: count, interval, add, direction, and that the
wire interval places its first step at that clock modulo 2^32.
"""

from __future__ import annotations

import dataclasses

import numpy as np

from klippy import msgproto

CLOCK_MASK = 0xFFFFFFFF


class StreamMismatch(ValueError):
    pass


@dataclasses.dataclass
class StepTrace:
    clocks: np.ndarray
    positions: np.ndarray


class _OidState:
    def __init__(self, oid, invert_dir, history):
        self.oid = oid
        self.invert_dir = invert_dir
        self.history = history
        self.index = 0
        self.last_clock32 = 0
        self.direction = None
        self.position = 0
        self.clock_chunks = []
        self.position_chunks = []

    def queue_step(self, interval, count, add):
        if self.direction is None:
            raise StreamMismatch(
                "queue_step before set_next_step_dir (oid %d)" % self.oid
            )
        if self.index >= len(self.history):
            raise StreamMismatch(
                "oid %d: more queue_step messages than history entries (%d)"
                % (self.oid, len(self.history))
            )
        first, h_count, h_interval, h_add = self.history[self.index]
        expected = (count * self.direction, interval & CLOCK_MASK, add)
        recorded = (h_count, h_interval & CLOCK_MASK, h_add)
        if expected != recorded:
            raise StreamMismatch(
                "oid %d message %d: wire (count, interval, add)=%r, history %r"
                % (self.oid, self.index, expected, recorded)
            )
        if (first - self.last_clock32) & CLOCK_MASK != interval & CLOCK_MASK:
            raise StreamMismatch(
                "oid %d message %d: history clock %d is not interval %d after"
                " the previous step" % (self.oid, self.index, first, interval)
            )
        self.index += 1
        k = np.arange(count, dtype=np.int64)
        clocks = first + k * interval + add * k * (k + 1) // 2
        positions = self.position + self.direction * (k + 1)
        self.clock_chunks.append(clocks)
        self.position_chunks.append(positions)
        self.last_clock32 = int(clocks[-1]) & CLOCK_MASK
        self.position = int(positions[-1])

    def trace(self):
        if self.index != len(self.history):
            raise StreamMismatch(
                "oid %d: %d queue_step messages for %d history entries"
                % (self.oid, self.index, len(self.history))
            )
        if not self.clock_chunks:
            empty = np.zeros(0, dtype=np.int64)
            return StepTrace(empty, empty)
        return StepTrace(
            np.concatenate(self.clock_chunks),
            np.concatenate(self.position_chunks),
        )


def decode(dictionary_path, output_path, steppers_by_oid):
    """steppers_by_oid: {oid: (invert_dir, history entries)}"""
    parser = msgproto.MessageParser()
    with open(dictionary_path, "rb") as f:
        parser.process_identify(f.read(), decompress=False)
    with open(output_path, "rb") as f:
        data = bytearray(f.read())
    states = {
        oid: _OidState(oid, invert, history)
        for oid, (invert, history) in steppers_by_oid.items()
    }
    pos = 0
    while pos < len(data):
        msglen = parser.check_packet(data[pos : pos + msgproto.MESSAGE_MAX])
        if msglen <= 0:
            raise StreamMismatch(
                "corrupt packet at byte %d of %s" % (pos, output_path)
            )
        for params in _parse_block(parser, data[pos : pos + msglen]):
            _apply(params, states)
        pos += msglen
    return {oid: state.trace() for oid, state in states.items()}


def _apply(params, states):
    name = params["#name"]
    if name == "queue_step_hp":
        raise NotImplementedError(
            "queue_step_hp (high precision stepcompress) is not decoded"
        )
    if name not in ("queue_step", "set_next_step_dir", "reset_step_clock"):
        return
    state = states.get(params["oid"])
    if state is None:
        return
    if name == "queue_step":
        state.queue_step(params["interval"], params["count"], params["add"])
    elif name == "set_next_step_dir":
        forward = bool(params["dir"]) != state.invert_dir
        state.direction = 1 if forward else -1
    else:
        state.last_clock32 = params["clock"] & CLOCK_MASK


def _parse_block(parser, block):
    pos = msgproto.MESSAGE_HEADER_SIZE
    end = len(block) - msgproto.MESSAGE_TRAILER_SIZE
    while pos < end:
        msgid, _ = parser.msgid_parser.parse(block, pos)
        fmt = parser.messages_by_id.get(msgid)
        if fmt is None:
            raise StreamMismatch(
                "unknown message id %d in output stream" % msgid
            )
        params, pos = fmt.parse(block, pos)
        params["#name"] = fmt.name
        yield params
