# Codecs for the serval ethercat-rt endpoint socket protocol
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import dataclasses
import enum
import struct

from klippy import msgproto

FRAME_SYNC = 0x55
FRAME_HEADER = struct.Struct("<BHB")
FRAME_CRC = struct.Struct("<H")
FRAME_LEN_FIELD_OVERHEAD = FRAME_HEADER.size - 1 + FRAME_CRC.size
MAX_FRAME_LEN_FIELD = 0xFFFF

CHANNEL_CONTROL = 0
CHANNEL_EVENTS = 1
CHANNEL_PIECES = 2

MESSAGE_HEADER = struct.Struct("<HBI")
MESSAGE_VERSION = 1

LANE_RUN_HEADER = struct.Struct("<BBBiQIH")
SETPOINT_SAMPLE = struct.Struct("<iihf")
MAX_FILL_CYCLES = 256
FLAG_REANCHOR = 0x01
FLAG_TAIL = 0x02

I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1
I16_MIN, I16_MAX = -(1 << 15), (1 << 15) - 1


class Kind(enum.IntEnum):
    CLAIM_HANDSHAKE = 0x0042
    CLAIM_HANDSHAKE_REPLY = 0x0043
    PUSH_SAMPLE_RUNS = 0x0062
    PUSH_SAMPLE_RUNS_RESPONSE = 0x0063
    QUERY_SAMPLE_GRID = 0x0064
    SAMPLE_GRID_RESPONSE = 0x0065
    ARM_SENSORLESS_ENDSTOP = 0x006E
    ARM_SENSORLESS_ENDSTOP_RESPONSE = 0x006F
    SET_TORQUE = 0x0070
    SET_TORQUE_RESPONSE = 0x0071
    STOP = 0x0072
    STOP_RESPONSE = 0x0073
    RESUME_STREAM = 0x0078
    RESUME_STREAM_RESPONSE = 0x0079
    SDO_WRITE = 0x007E
    SDO_WRITE_RESPONSE = 0x007F
    STATUS_HEARTBEAT = 0x0083
    ENDSTOP_TRIP = 0x0085


class WireError(Exception):
    pass


def crc16(data: bytes) -> int:
    hi, lo = msgproto.crc16_ccitt(data)
    return (hi << 8) | lo


def encode_frame(channel: int, payload: bytes) -> bytes:
    len_field = FRAME_LEN_FIELD_OVERHEAD + len(payload)
    if len_field > MAX_FRAME_LEN_FIELD:
        raise WireError(
            "frame payload of %d bytes exceeds the u16 length field"
            % (len(payload),)
        )
    head = FRAME_HEADER.pack(FRAME_SYNC, len_field, channel) + payload
    return head + FRAME_CRC.pack(crc16(head[1:]))


def decode_frame(buf: bytes, offset: int = 0) -> tuple[int, bytes, int]:
    """Decode the frame at buf[offset:]; returns (channel, payload, end)"""
    if len(buf) - offset < FRAME_HEADER.size + FRAME_CRC.size:
        raise WireError("truncated frame at offset %d" % (offset,))
    sync, len_field, channel = FRAME_HEADER.unpack_from(buf, offset)
    if sync != FRAME_SYNC:
        raise WireError("bad sync byte 0x%02x at offset %d" % (sync, offset))
    if len_field < FRAME_LEN_FIELD_OVERHEAD:
        raise WireError("frame length field %d too small" % (len_field,))
    end = offset + 1 + len_field
    if end > len(buf):
        raise WireError("truncated frame at offset %d" % (offset,))
    crc_at = end - FRAME_CRC.size
    (expected,) = FRAME_CRC.unpack_from(buf, crc_at)
    actual = crc16(buf[offset + 1 : crc_at])
    if expected != actual:
        raise WireError(
            "frame crc 0x%04x != computed 0x%04x at offset %d"
            % (expected, actual, offset)
        )
    return channel, bytes(buf[offset + FRAME_HEADER.size : crc_at]), end


def iter_frames(buf: bytes):
    offset = 0
    while offset < len(buf):
        channel, payload, offset = decode_frame(buf, offset)
        yield channel, payload


def encode_message(kind: Kind, correlation_id: int, body: bytes) -> bytes:
    return MESSAGE_HEADER.pack(kind, MESSAGE_VERSION, correlation_id) + body


def decode_message(payload: bytes) -> tuple[int, int, int, bytes]:
    """Returns (kind, version, correlation_id, body)"""
    if len(payload) < MESSAGE_HEADER.size:
        raise WireError("message shorter than its header")
    kind, version, cid = MESSAGE_HEADER.unpack_from(payload)
    return kind, version, cid, payload[MESSAGE_HEADER.size :]


class _Reader:
    def __init__(self, body: bytes):
        self.body = body
        self.pos = 0

    def take(self, fmt: struct.Struct):
        if self.pos + fmt.size > len(self.body):
            raise WireError("message body truncated")
        values = fmt.unpack_from(self.body, self.pos)
        self.pos += fmt.size
        return values

    def finish(self):
        if self.pos != len(self.body):
            raise WireError(
                "%d trailing bytes in message body"
                % (len(self.body) - self.pos)
            )


U8 = struct.Struct("<B")


def _check_range(name, value, lo, hi):
    if not lo <= value <= hi:
        raise WireError("%s %d outside [%d, %d]" % (name, value, lo, hi))


@dataclasses.dataclass(frozen=True)
class SetpointSample:
    pos_counts: int
    vel_ff: int
    torque_ff: int
    acc_mm_s2: float


@dataclasses.dataclass
class LaneRun:
    axis_idx: int
    slot_idx: int
    flags: int
    origin_mm_q16: int
    start_index: int
    interval_ticks: int
    samples: list[SetpointSample]

    @property
    def end_index(self):
        return self.start_index + len(self.samples)


def encode_push_sample_runs(lanes: list[LaneRun]) -> bytes:
    if not lanes:
        raise WireError("PushSampleRuns needs at least one lane")
    out = [U8.pack(len(lanes))]
    for lane in lanes:
        if not 0 < len(lane.samples) <= MAX_FILL_CYCLES:
            raise WireError(
                "lane run of %d samples outside [1, %d]"
                % (len(lane.samples), MAX_FILL_CYCLES)
            )
        _check_range("origin_mm_q16", lane.origin_mm_q16, I32_MIN, I32_MAX)
        out.append(
            LANE_RUN_HEADER.pack(
                lane.axis_idx,
                lane.slot_idx,
                lane.flags,
                lane.origin_mm_q16,
                lane.start_index,
                lane.interval_ticks,
                len(lane.samples),
            )
        )
        flat = []
        for s in lane.samples:
            _check_range("pos_counts", s.pos_counts, I32_MIN, I32_MAX)
            _check_range("vel_ff", s.vel_ff, I32_MIN, I32_MAX)
            _check_range("torque_ff", s.torque_ff, I16_MIN, I16_MAX)
            flat.extend((s.pos_counts, s.vel_ff, s.torque_ff, s.acc_mm_s2))
        out.append(struct.pack("<" + "iihf" * len(lane.samples), *flat))
    return b"".join(out)


def decode_push_sample_runs(body: bytes) -> list[LaneRun]:
    r = _Reader(body)
    (lane_count,) = r.take(U8)
    if not lane_count:
        raise WireError("PushSampleRuns without lanes")
    lanes = []
    for _ in range(lane_count):
        axis, slot, flags, origin, start, interval, count = r.take(
            LANE_RUN_HEADER
        )
        if not count:
            raise WireError("lane run without samples")
        if any(lane.slot_idx == slot for lane in lanes):
            raise WireError("duplicate slot %d in PushSampleRuns" % (slot,))
        samples = [
            SetpointSample(*r.take(SETPOINT_SAMPLE)) for _ in range(count)
        ]
        lanes.append(
            LaneRun(axis, slot, flags, origin, start, interval, samples)
        )
    r.finish()
    return lanes


@dataclasses.dataclass(frozen=True)
class LaneDepth:
    axis_idx: int
    free_cycles: int


@dataclasses.dataclass(frozen=True)
class PushSampleRunsResponse:
    result: int
    arrival_clock: int
    grid_index: int
    grid_clock: int
    lanes: tuple[LaneDepth, ...]


PUSH_RESPONSE_HEAD = struct.Struct("<iQQQ")
LANE_DEPTH = struct.Struct("<BI")


def encode_push_sample_runs_response(resp: PushSampleRunsResponse) -> bytes:
    out = [
        PUSH_RESPONSE_HEAD.pack(
            resp.result, resp.arrival_clock, resp.grid_index, resp.grid_clock
        ),
        U8.pack(len(resp.lanes)),
    ]
    out.extend(LANE_DEPTH.pack(d.axis_idx, d.free_cycles) for d in resp.lanes)
    return b"".join(out)


def decode_push_sample_runs_response(body: bytes) -> PushSampleRunsResponse:
    r = _Reader(body)
    result, arrival, grid_index, grid_clock = r.take(PUSH_RESPONSE_HEAD)
    (count,) = r.take(U8)
    if not count:
        raise WireError("PushSampleRunsResponse without lanes")
    lanes = tuple(LaneDepth(*r.take(LANE_DEPTH)) for _ in range(count))
    r.finish()
    return PushSampleRunsResponse(
        result, arrival, grid_index, grid_clock, lanes
    )


@dataclasses.dataclass(frozen=True)
class SampleGridResponse:
    executor: int
    cycle_ticks: int
    ring_depth_cycles: int
    grid_index: int
    grid_clock: int


SAMPLE_GRID = struct.Struct("<BIIQQ")


def encode_sample_grid_response(resp: SampleGridResponse) -> bytes:
    return SAMPLE_GRID.pack(*dataclasses.astuple(resp))


def decode_sample_grid_response(body: bytes) -> SampleGridResponse:
    r = _Reader(body)
    resp = SampleGridResponse(*r.take(SAMPLE_GRID))
    r.finish()
    return resp


@dataclasses.dataclass(frozen=True)
class SlaveStatus:
    slave_idx: int
    state: int
    fault_code: int


SLAVE_STATUS = struct.Struct("<BBH")


def encode_claim_handshake_reply(statuses: list[SlaveStatus]) -> bytes:
    if not statuses:
        raise WireError("ClaimHandshakeReply needs at least one slave")
    out = [U8.pack(len(statuses))]
    out.extend(SLAVE_STATUS.pack(*dataclasses.astuple(s)) for s in statuses)
    return b"".join(out)


def decode_claim_handshake_reply(body: bytes) -> list[SlaveStatus]:
    r = _Reader(body)
    (count,) = r.take(U8)
    if not count:
        raise WireError("ClaimHandshakeReply without slaves")
    statuses = [SlaveStatus(*r.take(SLAVE_STATUS)) for _ in range(count)]
    r.finish()
    return statuses


ARM_SENSORLESS_ENDSTOP = struct.Struct("<BBHB")


def encode_arm_sensorless_endstop(
    slot, endstop_id, torque_trip_tenth_pct, enable
):
    return ARM_SENSORLESS_ENDSTOP.pack(
        slot, endstop_id, torque_trip_tenth_pct, 1 if enable else 0
    )


SDO_WRITE = struct.Struct("<BHBBq")
SDO_WRITE_RESPONSE = struct.Struct("<iB4s")


def encode_sdo_write(slot, index, subindex, size, value):
    return SDO_WRITE.pack(slot, index, subindex, size, value)


def decode_sdo_write_response(body: bytes) -> tuple[int, bytes]:
    """(result, read-back bytes)"""
    r = _Reader(body)
    result, size, data = r.take(SDO_WRITE_RESPONSE)
    r.finish()
    return result, data[:size]


STOP_RESPONSE = struct.Struct("<iQ")


def decode_stop_response(body: bytes) -> tuple[int, int]:
    """(result, discard_clock)"""
    r = _Reader(body)
    values = r.take(STOP_RESPONSE)
    r.finish()
    return values


SET_TORQUE = struct.Struct("<BQ")


def encode_set_torque(enable: bool, execute_at_ns: int) -> bytes:
    return SET_TORQUE.pack(1 if enable else 0, execute_at_ns)


RESULT = struct.Struct("<i")


def decode_result(body: bytes) -> int:
    r = _Reader(body)
    (result,) = r.take(RESULT)
    r.finish()
    return result


@dataclasses.dataclass(frozen=True)
class StatusHeartbeat:
    engine_state: int
    fault_code: int
    retired_counts: tuple[int, ...]
    playback_clocks: tuple[int, ...]
    ff_saturation_count: int

    @property
    def signed_fault_code(self):
        (signed,) = struct.unpack("<h", struct.pack("<H", self.fault_code))
        return signed


HEARTBEAT_HEAD = struct.Struct("<BHB")
U32 = struct.Struct("<I")


def encode_status_heartbeat(hb: StatusHeartbeat) -> bytes:
    n = len(hb.retired_counts)
    if len(hb.playback_clocks) != n:
        raise WireError("heartbeat arrays differ in length")
    return (
        HEARTBEAT_HEAD.pack(hb.engine_state, hb.fault_code, n)
        + struct.pack("<%dI" % n, *hb.retired_counts)
        + struct.pack("<%dQ" % n, *hb.playback_clocks)
        + U32.pack(hb.ff_saturation_count)
    )


def decode_status_heartbeat(body: bytes) -> StatusHeartbeat:
    r = _Reader(body)
    state, fault, n = r.take(HEARTBEAT_HEAD)
    retired = r.take(struct.Struct("<%dI" % n))
    clocks = r.take(struct.Struct("<%dQ" % n))
    (saturation,) = r.take(U32)
    r.finish()
    return StatusHeartbeat(state, fault, retired, clocks, saturation)


@dataclasses.dataclass(frozen=True)
class EndstopTrip:
    endstop_id: int
    trip_clock: int


ENDSTOP_TRIP = struct.Struct("<BQ")


def encode_endstop_trip(trip: EndstopTrip) -> bytes:
    return ENDSTOP_TRIP.pack(trip.endstop_id, trip.trip_clock)


def decode_endstop_trip(body: bytes) -> EndstopTrip:
    r = _Reader(body)
    trip = EndstopTrip(*r.take(ENDSTOP_TRIP))
    r.finish()
    return trip
