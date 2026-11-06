"""G28 on servo rails against ethercat-rt-stub, live.

A proxy sits on klippy's endpoint socket and forwards every frame; the test
injects its own SDO writes through it. The stub's live tap reports each DC
cycle's target, actual position and torque: the test writes torque into
slot 0 (0x6077, what the sensorless endstop reads) once the axis has
travelled to the wall, and reads back where the axis was when the torque
rose. The stub runs with a group delay that is not a whole cycle, so the
trip clock must be on the sample grid for the trigger to land.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

import pytest
from ethercat_live_support import (
    COUNTS_PER_REV,
    INPUT_SMOOTHER,
    SIM_STEPPER_PINS,
    STUB_CYCLE_US,
    Z_STEPPER,
    Printer,
    needs_live,
    short_tmp,
    spawn_stub,
    wait_for,
)

from klippy.plugins.ethercat import wire

pytestmark = needs_live

X_ROTATION_DISTANCE = 40.0
X_COUNTS_PER_MM = COUNTS_PER_REV / X_ROTATION_DISTANCE
X_HOMING_SPEED = 5.0
HOMING_TORQUE_PCT = 50
INJECTED_TORQUE = 600
WALL_TRAVEL_MM = 15.0
GROUP_DELAY_US = 300
TORQUE_ACTUAL_INDEX = 0x6077
PROXY_CID_BASE = 0x40000000
REPLY_TIMEOUT_S = 2.0
# Y as a linux-MCU stepper: 200 full steps x 16 microsteps per 40 mm
Y_STEPS_PER_MM = 80
Y_WALL_TRAVEL_MM = 20.0
Y_OVERSHOOT_STEPS = 2

HOMING_CFG = """
[mcu]
serial: {{mcu_tty}}

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
max_z_velocity: 10
max_z_accel: 100

[ethercat]
cycle_us: {cycle_us}
socket: {socket}

[stepper_x]
servo: ethercat
ethercat_slave: 0
ethercat_axis: 0
encoder_counts_per_rev: {counts_per_rev}
rotation_distance: {x_rotation_distance}
endstop_pin: ethercat:x
position_endstop: {x_max}
position_max: {x_max}
homing_speed: {x_homing_speed}
homing_retract_dist: 0
homing_torque: {homing_torque}

{z_rail}
{input_shaper}
{stepper_y}
"""

SERVO_Y = """[stepper_y]
servo: ethercat
ethercat_slave: 1
ethercat_axis: 1
encoder_counts_per_rev: {counts_per_rev}
rotation_distance: 40
endstop_pin: ethercat:y
position_endstop: 0
position_max: 250
""".format(counts_per_rev=COUNTS_PER_REV)

STEPPER_Y = """[stepper_y]
step_pin: {step_pin}
dir_pin: {dir_pin}
endstop_pin: {endstop_pin}
microsteps: 16
rotation_distance: 40
position_endstop: 0
position_max: 40
homing_speed: 20
""".format(**SIM_STEPPER_PINS)


def _connect(sock, path):
    try:
        sock.connect(str(path))
        return True
    except OSError:
        return False


class StubProxy:
    """Forwards frames between klippy and the stub and sends the test's own
    requests to the stub, whose replies it keeps from klippy."""

    def __init__(self, listen_path, stub_path):
        self.stub_path = stub_path
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.listener.bind(str(listen_path))
        self.listener.listen(1)
        self.stub = None
        self.send_lock = threading.Lock()
        self.next_cid = PROXY_CID_BASE
        self.replies = {}
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        klippy, _ = self.listener.accept()
        stub = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        wait_for(lambda: _connect(stub, self.stub_path), "the stub socket")
        self.stub = stub
        threading.Thread(
            target=self._pump, args=(klippy, self._to_stub), daemon=True
        ).start()
        threading.Thread(
            target=self._pump,
            args=(stub, lambda frame: self._to_klippy(klippy, frame)),
            daemon=True,
        ).start()

    @staticmethod
    def _pump(src, deliver):
        """Ends when either side goes away: the test tears both down."""
        buf = bytearray()
        try:
            while data := src.recv(65536):
                buf += data
                while len(buf) >= wire.FRAME_HEADER.size:
                    total = 1 + int.from_bytes(buf[1:3], "little")
                    if len(buf) < total:
                        break
                    deliver(bytes(buf[:total]))
                    del buf[:total]
        except (ConnectionResetError, BrokenPipeError):
            return

    def _to_stub(self, frame):
        with self.send_lock:
            self.stub.sendall(frame)

    def _to_klippy(self, klippy, frame):
        _, payload, _ = wire.decode_frame(frame)
        _, _, cid, body = wire.decode_message(payload)
        waiter = self.replies.get(cid)
        if waiter is not None:
            waiter.append(body)
            return
        klippy.sendall(frame)

    def sdo_write(self, slot, index, value):
        with self.send_lock:
            cid = self.next_cid
            self.next_cid += 1
            waiter = self.replies[cid] = []
            payload = wire.encode_message(
                wire.Kind.SDO_WRITE,
                cid,
                wire.encode_sdo_write(slot, index, 0, 2, value),
            )
            self.stub.sendall(wire.encode_frame(wire.CHANNEL_CONTROL, payload))
        wait_for(lambda: waiter, "the SDO write reply", REPLY_TIMEOUT_S)
        result, _ = wire.decode_sdo_write_response(waiter[0])
        assert result == 0, "SDO write 0x%04x=%d: %d" % (index, value, result)


class Tap:
    """The stub's live tap, one record per DC cycle. Once armed, it writes
    INJECTED_TORQUE into slot 0 when the slot's target has moved wall_counts
    from where it rested, and keeps the target and actual position of the
    first record whose torque reached the threshold."""

    def __init__(self, path, proxy, threshold):
        self.proxy = proxy
        self.threshold = threshold
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        wait_for(lambda: _connect(self.sock, path), "the live tap")
        self.stream = self.sock.makefile("rb")
        header = json.loads(self.stream.readline())
        offsets = {c["name"]: c["offset"] for c in header["channels"]}
        self.record_size = header["record_size"]
        self.target = offsets["target_counts"]
        self.actual = offsets["position_actual"]
        self.torque = offsets["torque_actual"]
        self.lock = threading.Lock()
        self.last = None
        self.wall = None
        self.rest = None
        self.injected = False
        self.contact = None
        threading.Thread(target=self._read, daemon=True).start()

    def arm(self, wall_counts):
        with self.lock:
            self.wall = wall_counts
            self.rest = None
            self.injected = False
            self.contact = None

    def _read(self):
        while True:
            record = self.stream.read(self.record_size)
            if len(record) < self.record_size:
                return
            (target,) = struct.unpack_from("<i", record, self.target)
            (actual,) = struct.unpack_from("<i", record, self.actual)
            (torque,) = struct.unpack_from("<h", record, self.torque)
            inject = False
            with self.lock:
                self.last = (target, actual, torque)
                if self.wall is not None and not self.injected:
                    if self.rest is None:
                        self.rest = target
                    elif abs(target - self.rest) >= self.wall:
                        self.injected = inject = True
                if self.injected and self.contact is None:
                    if abs(torque) >= self.threshold:
                        self.contact = (target, actual)
            if inject:
                self.proxy.sdo_write(0, TORQUE_ACTUAL_INDEX, INJECTED_TORQUE)

    def settled_target(self):
        """Target of a record taken after this call."""
        with self.lock:
            self.last = None
        wait_for(lambda: self.last is not None, "a tap record", 2.0)
        return self.last[0]


class Rig:
    def __init__(self, tmp, stepper_y, x_max, **printer_args):
        servo_slots = [(0, X_COUNTS_PER_MM)]
        if stepper_y == SERVO_Y:
            servo_slots.append((1, X_COUNTS_PER_MM))
        stub_sock = tmp / "ec.sock"
        self.stub = spawn_stub(
            stub_sock,
            servo_slots,
            ["--group-delay-us", str(GROUP_DELAY_US)],
        )
        self.proxy = StubProxy(tmp / "proxy.sock", stub_sock)
        self.x_max = x_max
        try:
            self.printer = Printer(
                tmp,
                HOMING_CFG.format(
                    cycle_us=STUB_CYCLE_US,
                    socket=tmp / "proxy.sock",
                    counts_per_rev=COUNTS_PER_REV,
                    x_rotation_distance=X_ROTATION_DISTANCE,
                    x_max=x_max,
                    x_homing_speed=X_HOMING_SPEED,
                    homing_torque=HOMING_TORQUE_PCT,
                    stepper_y=stepper_y,
                    z_rail=Z_STEPPER,
                    input_shaper=INPUT_SMOOTHER,
                ),
                **printer_args,
            )
        except Exception:
            self.stub.kill()
            raise
        assert self.printer.state()[0] == "ready", self.printer.state()
        self.tap = Tap(
            str(stub_sock) + ".live", self.proxy, HOMING_TORQUE_PCT * 10
        )

    def position(self):
        return self.printer.api.status("toolhead")["toolhead"]["position"]

    def close(self):
        try:
            self.printer.close()
        finally:
            self.stub.kill()
            self.stub.wait()


@pytest.fixture
def tmp(tmp_path_factory):
    return short_tmp(tmp_path_factory)


def home_x_into_the_wall(rig, script):
    rig.tap.arm(WALL_TRAVEL_MM * X_COUNTS_PER_MM)
    resp = rig.printer.api.script(script)
    assert "error" not in resp, resp
    wait_for(lambda: rig.tap.contact is not None, "the injected torque", 2.0)
    rig.proxy.sdo_write(0, TORQUE_ACTUAL_INDEX, 0)
    return rig.tap.contact


def assert_homed_at_contact(rig, contact):
    """position_endstop names where the rotor stalled; klippy's position
    after homing is that plus how far past it the servo was stopped."""
    _, contact_actual = contact
    halt_target = rig.tap.settled_target()
    expected = rig.x_max + (halt_target - contact_actual) / X_COUNTS_PER_MM
    one_cycle = X_HOMING_SPEED * STUB_CYCLE_US * 1e-6
    rounding = 2.0 / X_COUNTS_PER_MM
    x = rig.position()[0]
    assert abs(x - expected) <= one_cycle + rounding, (x, expected)
    return x, halt_target


def test_g28_homes_a_servo_on_the_torque_trip(tmp):
    rig = Rig(tmp, SERVO_Y, x_max=40.0)
    try:
        contact = home_x_into_the_wall(rig, "G28 X")
        x, halt_target = assert_homed_at_contact(rig, contact)
        # The stream resumed where the servo stopped: a move lands where
        # klippy's homed frame says it does.
        resp = rig.printer.api.script("G1 X30 F600\nM400")
        assert "error" not in resp, resp
        time.sleep(0.2)
        moved = rig.tap.settled_target() - halt_target
        assert abs(moved - (30.0 - x) * X_COUNTS_PER_MM) <= 1.0
        status = rig.printer.api.status("ethercat")["ethercat"]
        assert status["fault"] == "", status
        assert rig.printer.state()[0] == "ready"
    finally:
        rig.close()


def test_homing_without_a_trip_fails_at_the_move_end(tmp):
    rig = Rig(tmp, SERVO_Y, x_max=10.0)
    try:
        resp = rig.printer.api.script("G28 X")
        assert "No trigger on x after full movement" in str(resp), resp
        state = rig.printer.state()
        assert state[0] == "ready", state
        status = rig.printer.api.status("ethercat")["ethercat"]
        assert status["fault"] == "", status
    finally:
        rig.close()


def test_mixed_printer_homes_servo_and_stepper(tmp):
    rig = Rig(
        tmp,
        STEPPER_Y,
        x_max=40.0,
        sim_trip_steps=-round(Y_WALL_TRAVEL_MM * Y_STEPS_PER_MM),
    )
    try:
        contact = home_x_into_the_wall(rig, "G28 X Y")
        assert_homed_at_contact(rig, contact)
        # The MCU samples the endstop between steps and stops the stepper
        # the step after: homed Y is the endstop plus that overshoot
        y = rig.position()[1]
        assert abs(y) <= Y_OVERSHOOT_STEPS / Y_STEPS_PER_MM, rig.position()
        assert rig.printer.state()[0] == "ready"
    finally:
        rig.close()
