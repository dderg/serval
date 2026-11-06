"""The plugin's wire codecs against ethercat-rt-stub.

The stub runs the real endpoint command dispatch, setpoint ring and DC loop
over a simulated drive. Set ETHERCAT_RT_STUB to its binary; without it these
tests are skipped. kv2-notes/gate.sh builds it from the ethercat-rt
repository (extracted from serval):

    CARGO_TARGET_DIR=~/.cache/ethercat-rt-target cargo build --release \
        --manifest-path ~/Developer/ethercat-rt/Cargo.toml --bin ethercat-rt-stub
"""

from __future__ import annotations

import os
import subprocess
import threading
import time

import pytest
from ethercat_support import GCODE, dictionary_path, run_batch, servo_config

from klippy.plugins.ethercat import wire
from klippy.plugins.ethercat.endpoint import EndpointClient

STUB = os.environ.get("ETHERCAT_RT_STUB")
pytestmark = pytest.mark.skipif(
    not STUB, reason="ETHERCAT_RT_STUB (ethercat-rt-stub binary) not set"
)

STUB_CYCLE_NS = 1_000_000
WAIT_S = 5.0
LEAD_CYCLES = 100
ENDSTOP_ID = 3
TRIP_TENTH_PCT = 100
TORQUE_ACTUAL_INDEX = 0x6077


class Session:
    def __init__(self, path, extra_args, slots):
        self.events = []
        self.closed = []
        self.cond = threading.Condition()
        self.process = subprocess.Popen(
            [STUB, "--socket", path, *extra_args]
            + sum((["--slave", str(s), "--axis", str(s)] for s in slots), []),
            stderr=subprocess.PIPE,
            text=True,
        )
        self.client = EndpointClient(path, self._on_event, self._on_closed)

    def _on_event(self, kind, body):
        with self.cond:
            self.events.append((kind, body))
            self.cond.notify_all()

    def _on_closed(self, reason):
        with self.cond:
            self.closed.append(reason)
            self.cond.notify_all()

    def wait_for(self, predicate, what, timeout=WAIT_S):
        with self.cond:
            if not self.cond.wait_for(predicate, timeout):
                raise AssertionError("timed out waiting for " + what)

    def heartbeats(self):
        return [
            wire.decode_status_heartbeat(body)
            for kind, body in list(self.events)
            if kind == wire.Kind.STATUS_HEARTBEAT
        ]

    def faults(self):
        return [hb.fault_code for hb in self.heartbeats() if hb.fault_code]

    def call(self, kind, body=b"", channel=wire.CHANNEL_CONTROL):
        return self.client.call(kind, body, WAIT_S, channel)

    def grid(self):
        kind, body = self.call(wire.Kind.QUERY_SAMPLE_GRID)
        assert kind == wire.Kind.SAMPLE_GRID_RESPONSE
        return wire.decode_sample_grid_response(body)

    def push(self, lanes):
        kind, body = self.call(
            wire.Kind.PUSH_SAMPLE_RUNS,
            wire.encode_push_sample_runs(lanes),
            wire.CHANNEL_PIECES,
        )
        assert kind == wire.Kind.PUSH_SAMPLE_RUNS_RESPONSE
        return wire.decode_push_sample_runs_response(body)

    def claim_and_enable(self):
        kind, body = self.call(wire.Kind.CLAIM_HANDSHAKE)
        statuses = wire.decode_claim_handshake_reply(body)
        kind, body = self.call(
            wire.Kind.SET_TORQUE, wire.encode_set_torque(True, 0)
        )
        assert wire.decode_result(body) == 0
        return statuses

    def close(self):
        self.client.close()
        self.process.terminate()
        _, stderr = self.process.communicate(WAIT_S)
        return stderr


@pytest.fixture
def start_stub(tmp_path):
    sessions = []

    def start(*extra_args):
        path = str(tmp_path / ("ec%d.sock" % len(sessions)))
        sessions.append(Session(path, list(extra_args), [0]))
        return sessions[-1]

    yield start
    for session in sessions:
        session.close()


@pytest.fixture
def stub(start_stub):
    return start_stub()


def run(start, count, flags=0, vel=0, slot=0, origin=0):
    samples = [
        wire.SetpointSample(i * vel // 1000, vel, 0, 0.0) for i in range(count)
    ]
    return wire.LaneRun(
        slot, slot, flags, origin, start, STUB_CYCLE_NS, samples
    )


def test_claim_and_grid(stub):
    statuses = stub.claim_and_enable()
    assert [(s.slave_idx, s.state, s.fault_code) for s in statuses] == [
        (1, 0, 0)
    ]
    first = stub.grid()
    assert first.executor == 1
    assert first.cycle_ticks == STUB_CYCLE_NS
    assert first.ring_depth_cycles >= wire.MAX_FILL_CYCLES
    time.sleep(0.05)
    second = stub.grid()
    assert second.grid_index > first.grid_index
    elapsed = (second.grid_index - first.grid_index) * STUB_CYCLE_NS
    # grid_clock is bridged from CLOCK_MONOTONIC by a paired clock read; its
    # noise is far below a cycle, the grid itself is exact
    drift = second.grid_clock - first.grid_clock - elapsed
    assert abs(drift) < STUB_CYCLE_NS / 100


def test_tail_run_drains_without_fault(stub):
    stub.claim_and_enable()
    start = stub.grid().grid_index + LEAD_CYCLES
    anchor = wire.FLAG_REANCHOR | wire.FLAG_TAIL
    resp = stub.push([run(start, 50, anchor, vel=1000)])
    assert resp.result == 0
    assert resp.lanes == (wire.LaneDepth(0, resp.lanes[0].free_cycles),)
    stub.wait_for(
        lambda: any(sum(hb.retired_counts) >= 50 for hb in stub.heartbeats()),
        "the run to play out",
    )
    time.sleep(0.05)
    assert not stub.faults()


def test_file_endpoint_frames_are_accepted(stub, request, tmp_path):
    batch = run_batch(
        tmp_path,
        servo_config("smooth_mzv", cycle_us=STUB_CYCLE_NS // 1000),
        GCODE,
        dictionary_path(request),
    )
    assert batch.returncode == 0, batch.log[-4000:]
    frames = _frames(batch.servo_file)
    total = sum(len(lanes[0].samples) for lanes in frames)
    stub.claim_and_enable()
    grid = stub.grid()
    shift = grid.grid_index + LEAD_CYCLES - frames[0][0].start_index
    for lanes in frames:
        shifted = [_shift(lane, shift) for lane in lanes]
        end = shifted[0].end_index
        # Pace like a producer that may run at most a ring ahead of play
        while end - stub.grid().grid_index > grid.ring_depth_cycles:
            time.sleep(0.02)
        resp = stub.push(shifted)
        assert resp.result == 0, resp
    stub.wait_for(
        lambda: any(
            sum(hb.retired_counts) >= total for hb in stub.heartbeats()
        ),
        "every replayed cycle to play",
        timeout=WAIT_S + total * STUB_CYCLE_NS / 1e9,
    )
    time.sleep(0.05)
    assert not stub.faults()


def _frames(path):
    out = []
    for channel, payload in wire.iter_frames(path.read_bytes()):
        assert channel == wire.CHANNEL_PIECES
        kind, version, cid, body = wire.decode_message(payload)
        assert kind == wire.Kind.PUSH_SAMPLE_RUNS
        out.append(wire.decode_push_sample_runs(body))
    assert out
    return out


def _shift(lane, shift):
    return wire.LaneRun(
        lane.axis_idx,
        lane.slot_idx,
        lane.flags,
        lane.origin_mm_q16,
        lane.start_index + shift,
        lane.interval_ticks,
        lane.samples,
    )


def _expect_fault(stub, code):
    stub.wait_for(lambda: stub.faults(), "a fault heartbeat")
    assert stub.faults()[0] == code & 0xFFFF


def test_hole_is_rejected_and_latched(stub):
    stub.claim_and_enable()
    start = stub.grid().grid_index + LEAD_CYCLES
    assert stub.push([run(start, 20, wire.FLAG_REANCHOR, vel=1000)]).result == 0
    assert stub.push([run(start + 21, 20, vel=1000)]).result == -321
    _expect_fault(stub, -321)


def test_late_run_is_rejected_and_latched(stub):
    stub.claim_and_enable()
    late_by = 10
    while stub.grid().grid_index <= late_by:
        time.sleep(0.005)
    now = stub.grid().grid_index
    late = run(now - late_by, 20, wire.FLAG_REANCHOR | wire.FLAG_TAIL)
    assert stub.push([late]).result == -317
    _expect_fault(stub, -317)


def test_drained_moving_run_is_an_underrun(stub):
    stub.claim_and_enable()
    start = stub.grid().grid_index + LEAD_CYCLES
    assert stub.push([run(start, 20, wire.FLAG_REANCHOR, vel=1000)]).result == 0
    _expect_fault(stub, -318)


def test_drive_fault_is_reported(start_stub):
    stub = start_stub("--drive-fault-after-cycles", "20")
    stub.claim_and_enable()
    start = stub.grid().grid_index + LEAD_CYCLES
    anchor = wire.FLAG_REANCHOR | wire.FLAG_TAIL
    assert stub.push([run(start, 100, anchor, vel=1000)]).result == 0
    _expect_fault(stub, 0x8611)


def test_sensorless_trip_halts_the_stream(stub):
    stub.claim_and_enable()
    kind, body = stub.call(
        wire.Kind.ARM_SENSORLESS_ENDSTOP,
        wire.encode_arm_sensorless_endstop(0, ENDSTOP_ID, TRIP_TENTH_PCT, True),
    )
    assert wire.decode_result(body) == 0
    start = stub.grid().grid_index + LEAD_CYCLES
    long_run = run(start, wire.MAX_FILL_CYCLES, wire.FLAG_REANCHOR, vel=1000)
    assert stub.push([long_run]).result == 0
    stub.wait_for(
        lambda: any(sum(hb.retired_counts) >= 10 for hb in stub.heartbeats()),
        "the run to start playing",
    )
    kind, body = stub.call(
        wire.Kind.SDO_WRITE,
        wire.encode_sdo_write(0, TORQUE_ACTUAL_INDEX, 0, 2, 2 * TRIP_TENTH_PCT),
    )
    result, readback = wire.decode_sdo_write_response(body)
    assert result == 0 and len(readback) == 2
    stub.wait_for(
        lambda: any(k == wire.Kind.ENDSTOP_TRIP for k, _ in stub.events),
        "the endstop trip",
    )
    trip = next(
        wire.decode_endstop_trip(b)
        for k, b in stub.events
        if k == wire.Kind.ENDSTOP_TRIP
    )
    assert trip.endstop_id == ENDSTOP_ID
    after = stub.grid().grid_index + LEAD_CYCLES
    assert stub.push([run(after, 10, wire.FLAG_REANCHOR)]).result == -315
    kind, body = stub.call(wire.Kind.RESUME_STREAM)
    assert wire.decode_result(body) == 0


def test_endpoint_eof_is_reported(stub):
    stub.claim_and_enable()
    stub.process.terminate()
    stub.wait_for(lambda: stub.closed, "the connection to close")
    assert "closed" in stub.closed[0]
