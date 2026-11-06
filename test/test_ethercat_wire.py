"""Codecs for the ethercat-rt socket protocol."""

from __future__ import annotations

import pytest

from klippy import msgproto
from klippy.plugins.ethercat import wire

# serval rust/host-rt/src/host_io/wire/tests.rs pins this value for the
# endpoint's crc16_ccitt; it is the protocol's shared test vector.
SERVAL_CRC_VECTOR = (b"\x05\x10", 0x9E81)


def test_crc_matches_endpoint_vector():
    data, crc = SERVAL_CRC_VECTOR
    assert wire.crc16(data) == crc


def test_frame_crc_is_klippers_crc16_little_endian():
    payload = bytes(range(40))
    frame = wire.encode_frame(wire.CHANNEL_PIECES, payload)
    hi, lo = msgproto.crc16_ccitt(frame[1:-2])
    assert frame[-2:] == bytes([lo, hi])


def test_frame_layout_and_roundtrip():
    payload = b"\x01\x02\x03"
    frame = wire.encode_frame(wire.CHANNEL_CONTROL, payload)
    assert frame[0] == wire.FRAME_SYNC
    assert int.from_bytes(frame[1:3], "little") == len(frame) - 1
    assert frame[3] == wire.CHANNEL_CONTROL
    channel, decoded, end = wire.decode_frame(frame)
    assert (channel, decoded, end) == (
        wire.CHANNEL_CONTROL,
        payload,
        len(frame),
    )


@pytest.mark.parametrize("index", [0, 3, 5])
def test_corrupted_frame_is_rejected(index):
    frame = bytearray(wire.encode_frame(wire.CHANNEL_EVENTS, b"\xaa" * 8))
    frame[index] ^= 0x01
    with pytest.raises(wire.WireError):
        wire.decode_frame(bytes(frame))


def test_truncated_frame_is_rejected():
    frame = wire.encode_frame(wire.CHANNEL_EVENTS, b"\xaa" * 8)
    with pytest.raises(wire.WireError):
        wire.decode_frame(frame[:-1])


def test_message_header_roundtrip():
    payload = wire.encode_message(wire.Kind.QUERY_SAMPLE_GRID, 0xDEADBEEF, b"")
    assert len(payload) == wire.MESSAGE_HEADER.size == 7
    assert wire.decode_message(payload) == (
        wire.Kind.QUERY_SAMPLE_GRID,
        wire.MESSAGE_VERSION,
        0xDEADBEEF,
        b"",
    )


def make_run(slot, count, start=1000, flags=0):
    samples = [
        wire.SetpointSample(i * 7 - 50, -i * 3, (-1) ** i * i, i * 0.5)
        for i in range(count)
    ]
    return wire.LaneRun(
        axis_idx=slot // 2,
        slot_idx=slot,
        flags=flags,
        origin_mm_q16=-(100 << 16),
        start_index=start,
        interval_ticks=250000,
        samples=samples,
    )


def test_push_sample_runs_layout():
    lanes = [make_run(0, 3, flags=wire.FLAG_REANCHOR), make_run(1, 2)]
    body = wire.encode_push_sample_runs(lanes)
    assert wire.LANE_RUN_HEADER.size == 21
    assert wire.SETPOINT_SAMPLE.size == 14
    assert len(body) == 1 + 2 * 21 + (3 + 2) * 14
    assert wire.decode_push_sample_runs(body) == lanes


def test_push_sample_runs_limits():
    with pytest.raises(wire.WireError):
        wire.encode_push_sample_runs([])
    with pytest.raises(wire.WireError):
        wire.encode_push_sample_runs([make_run(0, wire.MAX_FILL_CYCLES + 1)])
    overflow = make_run(0, 1)
    overflow.samples = [wire.SetpointSample(wire.I32_MAX + 1, 0, 0, 0.0)]
    with pytest.raises(wire.WireError):
        wire.encode_push_sample_runs([overflow])
    body = wire.encode_push_sample_runs([make_run(0, 1), make_run(0, 1)])
    with pytest.raises(wire.WireError):
        wire.decode_push_sample_runs(body)


def test_response_and_event_roundtrips():
    resp = wire.PushSampleRunsResponse(
        -317, 123456789, 42, 987654321, (wire.LaneDepth(0, 100),)
    )
    assert (
        wire.decode_push_sample_runs_response(
            wire.encode_push_sample_runs_response(resp)
        )
        == resp
    )
    grid = wire.SampleGridResponse(1, 250000, 4096, 7, 1 << 40)
    assert (
        wire.decode_sample_grid_response(wire.encode_sample_grid_response(grid))
        == grid
    )
    statuses = [wire.SlaveStatus(1, 0, 0), wire.SlaveStatus(2, 2, 0x8611)]
    assert (
        wire.decode_claim_handshake_reply(
            wire.encode_claim_handshake_reply(statuses)
        )
        == statuses
    )
    hb = wire.StatusHeartbeat(3, (-318) & 0xFFFF, (5, 6), (7, 8), 9)
    decoded = wire.decode_status_heartbeat(wire.encode_status_heartbeat(hb))
    assert decoded == hb
    assert decoded.signed_fault_code == -318
    trip = wire.EndstopTrip(3, 1 << 50)
    assert wire.decode_endstop_trip(wire.encode_endstop_trip(trip)) == trip
    assert wire.encode_set_torque(True, 5) == b"\x01" + (5).to_bytes(
        8, "little"
    )
    assert wire.decode_result((-310).to_bytes(4, "little", signed=True)) == -310


def test_trailing_bytes_are_rejected():
    body = wire.encode_endstop_trip(wire.EndstopTrip(1, 2)) + b"\x00"
    with pytest.raises(wire.WireError):
        wire.decode_endstop_trip(body)


def test_homing_and_sdo_layouts():
    arm = wire.encode_arm_sensorless_endstop(2, 3, 450, True)
    assert arm == bytes([2, 3]) + (450).to_bytes(2, "little") + b"\x01"
    sdo = wire.encode_sdo_write(1, 0x6077, 0, 2, -5)
    assert sdo == (
        b"\x01"
        + (0x6077).to_bytes(2, "little")
        + b"\x00\x02"
        + (-5).to_bytes(8, "little", signed=True)
    )
    reply = (0).to_bytes(4, "little") + b"\x02" + b"\xaa\xbb\x00\x00"
    assert wire.decode_sdo_write_response(reply) == (0, b"\xaa\xbb")
    stop = (0).to_bytes(4, "little") + (123).to_bytes(8, "little")
    assert wire.decode_stop_response(stop) == (0, 123)
