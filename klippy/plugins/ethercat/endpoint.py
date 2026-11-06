# Destinations for servo lane runs: a frame file or a live ethercat-rt
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import os
import socket
import subprocess
import threading
import time

from . import wire
from .stream import EthercatError, SlotRing
from .timemap import RAW_NS_PER_S, EndpointClockMap, EndpointGrid, SyntheticGrid

EXECUTOR_SETPOINT_RING = 1
ENGINE_STATE_FAULT = 3
SLAVE_STATE_OK = 0
ERR_PIECES_WHILE_HALTED = -315
SOCKET_WAIT_S = 15.0
CALL_TIMEOUT_S = 5.0
TERMINATE_WAIT_S = 5.0

# Keyed by the 16-bit wire value: endpoint ring faults are negative codes
# sign-wrapped into u16, drive faults are the raw CiA402 error code.
ENDPOINT_FAULTS = {
    -172 & 0xFFFF: "internal invariant (grid regression)",
    -304 & 0xFFFF: "run interval differs from the DC cycle",
    -309 & 0xFFFF: "unknown slot",
    -313 & 0xFFFF: "setpoints while the drives are parked",
    -314 & 0xFFFF: "setpoints while a drive is faulted",
    -315 & 0xFFFF: "setpoints while the stream is halted",
    -317 & 0xFFFF: "run late",
    -318 & 0xFFFF: "ring underrun while moving",
    -319 & 0xFFFF: "ring full",
    -320 & 0xFFFF: "unknown lane",
    -321 & 0xFFFF: "run rejected (hole, overlap, no anchor or origin shift)",
    -360 & 0xFFFF: "zero torque trip threshold",
    -361 & 0xFFFF: "slot shares its axis with more than one other slot",
    0x8611: "drive following error",
    0x8700: "drive sync loss",
    0xFE10: "DC frame late",
    0xFE11: "DC cycle skipped",
}


def describe_fault(code):
    key = code & 0xFFFF
    return "%s (code 0x%04x)" % (ENDPOINT_FAULTS.get(key, "unknown fault"), key)


class FileEndpoint:
    """Batch-mode endpoint: writes the exact PushSampleRuns frames a live
    endpoint would receive to a file. Nothing plays in real time, so only
    the ring's continuity rules are checked."""

    def __init__(self, path, cycle_ns):
        self.path = path
        self.cycle_ns = cycle_ns
        self.grid = SyntheticGrid(cycle_ns)
        self._file = None
        self._rings = {}
        self._correlation_id = 0
        self.pushed_cycles = 0
        self.stream_halted = False

    def connect(self, lanes):
        self._file = open(self.path, "wb")

    def now_index(self):
        return None

    def push(self, lanes: list[wire.LaneRun]):
        for run in lanes:
            ring = self._rings.get(run.slot_idx)
            if ring is None:
                ring = self._rings[run.slot_idx] = SlotRing(
                    run.slot_idx, self.cycle_ns, None, 0
                )
            ring.admit(run, self.now_index())
        self._correlation_id += 1
        body = wire.encode_push_sample_runs(lanes)
        payload = wire.encode_message(
            wire.Kind.PUSH_SAMPLE_RUNS, self._correlation_id, body
        )
        self._file.write(wire.encode_frame(wire.CHANNEL_PIECES, payload))
        self.pushed_cycles += len(lanes[0].samples)

    def arm_endstop(self, endstop_id, slot_thresholds, on_trip):
        pass

    def disarm_endstop(self, endstop_id, slots):
        pass

    def get_status(self):
        return {"mode": "file", "pushed_cycles": self.pushed_cycles}

    def close(self):
        if self._file is not None:
            self._file.close()


class EndpointClosed(Exception):
    pass


class EndpointClient:
    """One session on the ethercat-rt socket.

    A reader thread splits the byte stream into frames. Replies are routed
    to the callback registered with their correlation id; events (and any
    frame nobody waits for) go to on_event. Callbacks run on the reader
    thread.
    """

    def __init__(
        self, path, on_event, on_closed, connect_timeout=SOCKET_WAIT_S
    ):
        self._on_event = on_event
        self._on_closed = on_closed
        self._send_lock = threading.Lock()
        self._pending = {}
        self._correlation_id = 0
        self._closing = False
        self._sock = _connect_unix(path, connect_timeout)
        self._thread = threading.Thread(
            target=self._read_loop, name="ethercat-rx", daemon=True
        )
        self._thread.start()

    def send(self, kind, body, on_reply, channel=wire.CHANNEL_CONTROL):
        with self._send_lock:
            self._correlation_id += 1
            cid = self._correlation_id
            self._pending[cid] = on_reply
            payload = wire.encode_message(kind, cid, body)
            self._sock.sendall(wire.encode_frame(channel, payload))
        return cid

    def call(
        self, kind, body, timeout=CALL_TIMEOUT_S, channel=wire.CHANNEL_CONTROL
    ):
        """Blocking request/reply for code that may block its thread."""
        done = threading.Event()
        reply = []

        def on_reply(reply_kind, reply_body):
            reply.append((reply_kind, reply_body))
            done.set()

        self.send(kind, body, on_reply, channel)
        if not done.wait(timeout):
            raise EthercatError(
                "endpoint did not answer %s within %.1f s"
                % (wire.Kind(kind).name, timeout)
            )
        return reply[0]

    def close(self):
        self._closing = True
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()
        self._thread.join(CALL_TIMEOUT_S)

    def _read_loop(self):
        buf = bytearray()
        try:
            while True:
                data = self._sock.recv(65536)
                if not data:
                    raise EndpointClosed("endpoint closed the connection")
                buf += data
                while len(buf) >= wire.FRAME_HEADER.size:
                    total = 1 + int.from_bytes(buf[1:3], "little")
                    if len(buf) < total:
                        break
                    channel, payload, _ = wire.decode_frame(bytes(buf[:total]))
                    del buf[:total]
                    self._dispatch(channel, payload)
        except (OSError, EndpointClosed, wire.WireError, EthercatError) as e:
            if not self._closing:
                self._on_closed(str(e))

    def _dispatch(self, channel, payload):
        kind, version, cid, body = wire.decode_message(payload)
        on_reply = self._pending.pop(cid, None) if cid else None
        if on_reply is not None:
            on_reply(kind, body)
        else:
            self._on_event(kind, body)


def _connect_unix(path, timeout):
    deadline = time.monotonic() + timeout
    while True:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(path)
            return sock
        except OSError as e:
            sock.close()
            if time.monotonic() >= deadline:
                raise EthercatError(
                    "cannot connect to endpoint socket %s: %s" % (path, e)
                )
            time.sleep(0.05)


class LiveEndpoint:
    """A running ethercat-rt (spawned or attached) on the host.

    Pushes go out from the reactor without waiting; their replies, the
    heartbeats and endstop trips arrive on the socket thread. Any fault,
    rejected run or lost connection shuts the printer down: the endpoint
    has latched it and the motion it describes did not happen.
    """

    def __init__(
        self,
        printer,
        socket_path,
        command,
        cycle_ns,
        min_lead_s,
        required_lead_s,
    ):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.socket_path = socket_path
        self.command = command
        self.cycle_ns = cycle_ns
        self.min_lead_cycles = max(
            1, round(min_lead_s * RAW_NS_PER_S / cycle_ns)
        )
        self.required_lead_s = required_lead_s
        self.grid = None
        self.clock_map = None
        self.process = None
        self.client = None
        self.enabled = False
        self._rings = {}
        self._faulted = False
        self.fault_message = ""
        self.last_heartbeat = None
        self.min_free_cycles = None
        self.pushed_cycles = 0
        self.anchors = 0
        self.stream_halted = False
        self._armed = {}

    # Setup
    def connect(self, lanes):
        if self.command is not None:
            self._spawn(lanes)
        self.client = EndpointClient(
            self.socket_path, self._on_event, self._on_closed
        )
        kind, body = self.client.call(wire.Kind.CLAIM_HANDSHAKE, b"")
        statuses = wire.decode_claim_handshake_reply(body)
        bad = [s for s in statuses if s.state != SLAVE_STATE_OK]
        if bad or len(statuses) != len(lanes):
            raise self.printer.config_error(
                "ethercat endpoint claimed %d slaves for %d servos, not ok: %r"
                % (len(statuses), len(lanes), bad)
            )
        asked = self.reactor.monotonic()
        kind, body = self.client.call(wire.Kind.QUERY_SAMPLE_GRID, b"")
        answered = self.reactor.monotonic()
        grid = wire.decode_sample_grid_response(body)
        # The upcoming apply point (plus the group delay) lies within a
        # cycle of the query; anything further off means the endpoint's
        # grid is not on CLOCK_MONOTONIC_RAW and every sample would play
        # at the wrong time.
        lead_ns = self.min_lead_cycles * self.cycle_ns
        offset_ns = grid.grid_clock - 0.5 * (asked + answered) * RAW_NS_PER_S
        if abs(offset_ns) > lead_ns:
            raise self.printer.config_error(
                "ethercat endpoint grid clock is %.3f ms off the host"
                " CLOCK_MONOTONIC_RAW (allowed %.3f ms)"
                % (offset_ns / 1e6, lead_ns / 1e6)
            )
        if grid.executor != EXECUTOR_SETPOINT_RING:
            raise self.printer.config_error(
                "ethercat endpoint executor %d is not the setpoint ring"
                % (grid.executor,)
            )
        if grid.cycle_ticks != self.cycle_ns:
            raise self.printer.config_error(
                "[ethercat] cycle_us %d differs from the endpoint's DC cycle of"
                " %d ns" % (self.cycle_ns // 1000, grid.cycle_ticks)
            )
        ring_s = grid.ring_depth_cycles * self.cycle_ns / RAW_NS_PER_S
        if ring_s < self.required_lead_s:
            raise self.printer.config_error(
                "ethercat endpoint ring holds %d cycles (%.3f s); step"
                " generation runs up to %.3f s ahead"
                % (grid.ring_depth_cycles, ring_s, self.required_lead_s)
            )
        mcu = self.printer.lookup_object("mcu")
        self.clock_map = EndpointClockMap(
            self.reactor, mcu, self.reactor.monotonic()
        )
        self.grid = EndpointGrid(
            self.clock_map, self.cycle_ns, grid.grid_index, grid.grid_clock
        )
        self._rings = {
            lane.slot_idx: SlotRing(
                lane.slot_idx,
                self.cycle_ns,
                grid.ring_depth_cycles,
                self.min_lead_cycles,
            )
            for lane in lanes
        }
        logging.info(
            "ethercat: endpoint %s claimed %d slaves, cycle %d ns, ring %d"
            " cycles, grid index %d",
            self.socket_path,
            len(statuses),
            grid.cycle_ticks,
            grid.ring_depth_cycles,
            grid.grid_index,
        )

    def _spawn(self, lanes):
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        args = list(self.command) + ["--socket", self.socket_path]
        for lane in lanes:
            servo = lane.servo
            args += [
                "--slave",
                str(servo.slave_position),
                "--axis",
                str(servo.axis_index),
                "--counts-per-mm",
                repr(1.0 / servo.get_step_dist()),
            ]
            if servo.get_dir_inverted()[0]:
                args.append("--invert")
        logging.info("ethercat: starting endpoint: %s", " ".join(args))
        self.process = subprocess.Popen(args)

    def enable_drives(self):
        """Called from klippy:connect, where blocking is allowed; the
        CiA402 enable walk takes hundreds of ms on real drives."""
        now_ns = int(self.reactor.monotonic() * RAW_NS_PER_S)
        kind, body = self.client.call(
            wire.Kind.SET_TORQUE, wire.encode_set_torque(True, now_ns)
        )
        result = wire.decode_result(body)
        if result:
            raise EthercatError(
                "endpoint refused drive enable: %s" % describe_fault(result)
            )
        self.enabled = True

    def _call_ok(self, kind, body, what):
        _, reply = self.client.call(kind, body)
        result = wire.decode_result(reply)
        if result:
            raise EthercatError(
                "endpoint refused to %s: %s" % (what, describe_fault(result))
            )

    # Homing
    def arm_endstop(self, endstop_id, slot_thresholds, on_trip):
        """on_trip(contact_clock, playback_clocks) runs on the socket
        thread, with the playback cursors of the cycle before the trip."""
        self._armed[endstop_id] = on_trip
        for slot, tenth_pct in slot_thresholds:
            self._call_ok(
                wire.Kind.ARM_SENSORLESS_ENDSTOP,
                wire.encode_arm_sensorless_endstop(
                    slot, endstop_id, tenth_pct, True
                ),
                "arm endstop %d on slot %d" % (endstop_id, slot),
            )

    def disarm_endstop(self, endstop_id, slots):
        for slot in slots:
            self._call_ok(
                wire.Kind.ARM_SENSORLESS_ENDSTOP,
                wire.encode_arm_sensorless_endstop(slot, endstop_id, 0, False),
                "disarm endstop %d on slot %d" % (endstop_id, slot),
            )
        del self._armed[endstop_id]

    def resume_stream(self):
        self._call_ok(wire.Kind.RESUME_STREAM, b"", "resume the stream")
        for ring in self._rings.values():
            ring.discard()
        self.stream_halted = False

    # Streaming
    def now_index(self):
        return self.grid.index_at_or_after_clock(
            self.reactor.monotonic() * RAW_NS_PER_S
        )

    def push(self, lanes: list[wire.LaneRun]):
        if not self.enabled:
            raise EthercatError("servo motion before the drives were enabled")
        now = self.now_index()
        for run in lanes:
            self._rings[run.slot_idx].admit(run, now)
        body = wire.encode_push_sample_runs(lanes)
        self.client.send(
            wire.Kind.PUSH_SAMPLE_RUNS,
            body,
            self._on_push_reply,
            wire.CHANNEL_PIECES,
        )
        self.pushed_cycles += len(lanes[0].samples)
        if lanes[0].flags & wire.FLAG_REANCHOR:
            self.anchors += 1

    def _on_push_reply(self, kind, body):
        resp = wire.decode_push_sample_runs_response(body)
        if resp.result == ERR_PIECES_WHILE_HALTED and self.stream_halted:
            return
        if resp.result:
            self._fault(
                "endpoint rejected a run: %s" % describe_fault(resp.result)
            )
            return
        try:
            self.grid.update(resp.grid_index, resp.grid_clock)
        except Exception as e:
            self._fault(str(e))
            return
        free = min(d.free_cycles for d in resp.lanes)
        if self.min_free_cycles is None or free < self.min_free_cycles:
            self.min_free_cycles = free

    def _on_event(self, kind, body):
        if kind == wire.Kind.STATUS_HEARTBEAT:
            hb = wire.decode_status_heartbeat(body)
            self.last_heartbeat = hb
            if hb.fault_code or hb.engine_state == ENGINE_STATE_FAULT:
                self._fault(
                    "endpoint fault: %s (engine state %d)"
                    % (describe_fault(hb.fault_code), hb.engine_state)
                )
        elif kind == wire.Kind.ENDSTOP_TRIP:
            trip = wire.decode_endstop_trip(body)
            on_trip = self._armed.get(trip.endstop_id)
            if on_trip is None:
                self._fault(
                    "unexpected endstop trip %d at clock %d"
                    % (trip.endstop_id, trip.trip_clock)
                )
                return
            self.stream_halted = True
            hb = self.last_heartbeat
            on_trip(
                trip.trip_clock, hb.playback_clocks if hb is not None else None
            )
        else:
            self._fault("unexpected endpoint message 0x%04x" % (kind,))

    def _on_closed(self, reason):
        self._fault("lost the endpoint: %s" % (reason,))

    def _fault(self, msg):
        if self._faulted:
            return
        self._faulted = True
        self.fault_message = msg
        logging.error("ethercat: %s", msg)
        self.reactor.register_async_callback(
            lambda eventtime: self.printer.invoke_shutdown("ethercat: " + msg)
        )

    # Lifecycle
    def recalibrate(self, print_time, eventtime):
        if self.clock_map is not None:
            self.clock_map.calibrate_clock(print_time, eventtime)

    def halt(self):
        if self.client is None or self._faulted:
            return
        self._faulted = True
        try:
            self.client.send(wire.Kind.STOP, b"", lambda kind, body: None)
        except OSError as e:
            logging.error("ethercat: cannot halt the endpoint: %s", e)

    def close(self):
        if self.client is not None:
            self._faulted = True
            self.client.close()
            self.client = None
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(TERMINATE_WAIT_S)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process = None

    def get_status(self):
        hb = self.last_heartbeat
        return {
            "mode": "live",
            "pushed_cycles": self.pushed_cycles,
            "anchors": self.anchors,
            "retired_cycles": list(hb.retired_counts) if hb is not None else [],
            "min_free_cycles": self.min_free_cycles,
            "fault": self.fault_message,
        }

    def stats(self):
        status = self.get_status()
        return "ethercat: pushed=%d retired=%s min_free=%s" % (
            status["pushed_cycles"],
            ",".join(str(r) for r in status["retired_cycles"]),
            status["min_free_cycles"],
        )
