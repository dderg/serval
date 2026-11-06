"""Decode the binary host->MCU stream written by `klippy -o` into steps.

The MCU executes `queue_step oid interval count add` as `count` steps, the
first `interval` ticks after the previous step and each later one `add`
ticks further apart than the one before (src/stepper.c).

Clocks on the wire are 32 bit. A step further than CLOCK_DIFF_MAX after the
previous one is sent alone (count=1) with its interval truncated, and the
MCU relies on receiving it shortly before it is due (stepcompress.c
add_move sets req_clock to its own clock). The stream is ordered by
req_clock across all steppers, so such a step is the first clock congruent
to its truncated value that is not earlier than any request already seen.
That is exact while some request was emitted in the preceding 2^32 ticks;
after a longer full idle it is not, and the model check downstream fails.
"""

from __future__ import annotations

import dataclasses

import numpy as np

from klippy import msgproto

CLOCK_SPAN = 1 << 32
CLOCK_DIFF_MAX = 3 << 28


@dataclasses.dataclass
class StepTrace:
    clocks: np.ndarray
    positions: np.ndarray


def _first_at_or_after(lower, congruent):
    return lower + ((congruent - lower) % CLOCK_SPAN)


class _OidState:
    def __init__(self):
        self.last_clock = 0
        self.direction = None
        self.position = 0
        self.clock_chunks = []
        self.position_chunks = []

    def queue_step(self, interval, count, add, request_floor):
        if count == 1:
            first = _first_at_or_after(
                max(request_floor, self.last_clock + 1),
                self.last_clock + interval,
            )
            far = first - self.last_clock >= CLOCK_DIFF_MAX
            request = first if far else self.last_clock
        else:
            first = self.last_clock + interval
            request = self.last_clock
        k = np.arange(count, dtype=np.int64)
        clocks = first + k * interval + add * k * (k + 1) // 2
        self.clock_chunks.append(clocks)
        if self.direction is not None:
            positions = self.position + self.direction * (k + 1)
            self.position_chunks.append(positions)
            self.position = int(positions[-1])
        self.last_clock = int(clocks[-1])
        return request

    def trace(self):
        if not self.clock_chunks:
            empty = np.zeros(0, dtype=np.int64)
            return StepTrace(empty, empty)
        return StepTrace(
            np.concatenate(self.clock_chunks),
            np.concatenate(self.position_chunks),
        )


class _Decoder:
    def __init__(self, invert_dir_by_oid):
        self.invert = invert_dir_by_oid
        self.states = {}
        self.request_floor = 0

    def state(self, oid):
        if oid not in self.states:
            self.states[oid] = _OidState()
        return self.states[oid]

    def apply(self, params):
        name = params["#name"]
        if name == "queue_step_hp":
            raise NotImplementedError(
                "queue_step_hp (high precision stepcompress) is not decoded"
            )
        if name == "queue_step":
            oid = params["oid"]
            state = self.state(oid)
            if oid in self.invert and state.direction is None:
                raise ValueError(
                    "queue_step before set_next_step_dir (oid %d)" % oid
                )
            request = state.queue_step(
                params["interval"],
                params["count"],
                params["add"],
                self.request_floor,
            )
            self.request_floor = max(self.request_floor, request)
        elif name == "set_next_step_dir":
            oid = params["oid"]
            if oid in self.invert:
                forward = bool(params["dir"]) != self.invert[oid]
                self.state(oid).direction = 1 if forward else -1
        elif name == "reset_step_clock":
            state = self.state(params["oid"])
            state.last_clock = _first_at_or_after(
                self.request_floor, params["clock"]
            )


def decode(dictionary_path, output_path, invert_dir_by_oid):
    parser = msgproto.MessageParser()
    with open(dictionary_path, "rb") as f:
        parser.process_identify(f.read(), decompress=False)
    with open(output_path, "rb") as f:
        data = bytearray(f.read())
    decoder = _Decoder(invert_dir_by_oid)
    pos = 0
    while pos < len(data):
        msglen = parser.check_packet(data[pos : pos + msgproto.MESSAGE_MAX])
        if msglen <= 0:
            raise ValueError(
                "corrupt packet at byte %d of %s" % (pos, output_path)
            )
        for params in _parse_block(parser, data[pos : pos + msglen]):
            decoder.apply(params)
        pos += msglen
    return {oid: decoder.state(oid).trace() for oid in invert_dir_by_oid}


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
