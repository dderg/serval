"""Real-time smoke: klippy streams servo samples into ethercat-rt-stub.

klippy runs live against a Klipper linux-process MCU, which supplies the
print_time clock; every axis is an EtherCAT servo, so the MCU drives no pins.
Needs ETHERCAT_RT_STUB (an ethercat-rt-stub whose DC grid is on
CLOCK_MONOTONIC_RAW, see kv2-notes/ethercat-rt-stub-clock.patch) and
KLIPPER_LINUX_MCU (out/klipper.elf built from test/configs/linuxprocess.config);
skipped without them.
"""

from __future__ import annotations

import json
import os
import pathlib
import socket
import subprocess
import sys
import threading
import time

import pytest
from ethercat_support import GCODE, REPO_ROOT

STUB = os.environ.get("ETHERCAT_RT_STUB")
LINUX_MCU = os.environ.get("KLIPPER_LINUX_MCU")
pytestmark = pytest.mark.skipif(
    not (STUB and LINUX_MCU),
    reason="ETHERCAT_RT_STUB and KLIPPER_LINUX_MCU not set",
)

READY_TIMEOUT_S = 30.0
SETTLE_S = 1.5
API_TIMEOUT_S = 60.0
STUB_CYCLE_US = 1000

PRINTER_CFG = """
[mcu]
serial: {mcu_tty}

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 5000
max_z_velocity: 10
max_z_accel: 100

[ethercat]
cycle_us: {cycle_us}
socket: {socket}
{endpoint}

[stepper_x]
servo: ethercat
ethercat_slave: 0
ethercat_axis: 0
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: ethercat:x
position_endstop: 0
position_max: 250

[stepper_y]
servo: ethercat
ethercat_slave: 1
ethercat_axis: 1
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: ethercat:y
position_endstop: 0
position_max: 250

[stepper_z]
servo: ethercat
ethercat_slave: 2
ethercat_axis: 2
encoder_counts_per_rev: 131072
rotation_distance: 8
endstop_pin: ethercat:z
position_endstop: 0
position_max: 200

[input_shaper]
shaper_type: smooth_mzv
shaper_freq_x: 50
shaper_freq_y: 40
"""

LONG_MOVES = "\n".join(
    "G1 X%d Y%d F3000" % (50 + 100 * (i % 2), 50 + 100 * ((i // 2) % 2))
    for i in range(40)
)
SECOND_PRINT = "G1 X60 Y140 Z12 F6000\nG1 X100 Y100 Z10 F6000\nM400"


class Api:
    def __init__(self, path):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(str(path))
        self.sock.settimeout(API_TIMEOUT_S)
        self.buf = b""
        self.next_id = 0
        self.lock = threading.Lock()

    def call(self, method, **params):
        with self.lock:
            self.next_id += 1
            req = {"id": self.next_id, "method": method, "params": params}
            self.sock.sendall(json.dumps(req).encode() + b"\x03")
            while True:
                while b"\x03" not in self.buf:
                    chunk = self.sock.recv(65536)
                    if not chunk:
                        raise ConnectionError("klippy closed the API socket")
                    self.buf += chunk
                msg, self.buf = self.buf.split(b"\x03", 1)
                resp = json.loads(msg)
                if resp.get("id") == req["id"]:
                    return resp

    def status(self, *objects):
        resp = self.call("objects/query", objects={o: None for o in objects})
        return resp["result"]["status"]


class Printer:
    def __init__(self, tmp, endpoint):
        self.tmp = tmp
        self.mcu_tty = tmp / "mcu"
        self.api_path = tmp / "api.sock"
        self.log = tmp / "klippy.log"
        self.mcu = subprocess.Popen([LINUX_MCU, "-I", str(self.mcu_tty)])
        _wait(lambda: self.mcu_tty.exists(), "the linux MCU tty")
        config = tmp / "printer.cfg"
        config.write_text(
            PRINTER_CFG.format(
                mcu_tty=self.mcu_tty,
                cycle_us=STUB_CYCLE_US,
                socket=tmp / "ec.sock",
                endpoint=endpoint,
            )
        )
        self.klippy = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "klippy",
                str(config),
                "-l",
                str(self.log),
                "-a",
                str(self.api_path),
            ],
            cwd=REPO_ROOT,
            env=dict(os.environ, PYTHONPATH=str(REPO_ROOT)),
        )
        _wait(lambda: self.api_path.exists(), "the klippy API socket")
        self.api = Api(self.api_path)
        _wait(
            lambda: self.state()[0] in ("ready", "shutdown", "error"),
            "klippy to start",
            READY_TIMEOUT_S,
        )

    def state(self):
        info = self.api.call("info")["result"]
        return info["state"], info["state_message"]

    def close(self):
        for proc in (self.klippy, self.mcu):
            proc.terminate()
            proc.wait(10)


def _wait(predicate, what, timeout=READY_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting for " + what)
        time.sleep(0.05)


@pytest.fixture
def tmp(tmp_path_factory):
    # Unix socket paths are limited to ~100 bytes; pytest's are long
    return pathlib.Path(tmp_path_factory.mktemp("ec", numbered=True))


def test_live_print_streams_every_cycle_to_the_stub(tmp):
    printer = Printer(tmp, "endpoint_binary: %s" % (STUB,))
    try:
        assert printer.state()[0] == "ready", printer.state()
        resp = printer.api.call("gcode/script", script=GCODE)
        assert "error" not in resp, resp
        # Idle long enough for real time to pass the stream's end, so the
        # next print must re-anchor after verifying the gap held still
        time.sleep(SETTLE_S)
        resp = printer.api.call("gcode/script", script=SECOND_PRINT)
        assert "error" not in resp, resp
        time.sleep(SETTLE_S)
        state = printer.state()
        status = printer.api.status("ethercat")["ethercat"]
        assert state[0] == "ready", state
        assert status["fault"] == ""
        assert status["anchors"] >= 2
        # Every cycle the plugin pushed was played by every slot: no hole,
        # no late run, no underrun, nothing discarded.
        assert status["retired_cycles"] == [status["pushed_cycles"]] * 3
    finally:
        printer.close()
    log = printer.log.read_text()
    assert "ethercat: endpoint" in log and "claimed 3 slaves" in log


def test_endpoint_death_mid_print_shuts_down(tmp):
    sock = tmp / "ec.sock"
    stub = subprocess.Popen(
        [STUB, "--socket", str(sock)]
        + sum((["--slave", str(s), "--axis", str(s)] for s in range(3)), [])
    )
    printer = Printer(tmp, "")
    try:
        assert printer.state()[0] == "ready", printer.state()
        printer.api.call(
            "gcode/script", script="SET_KINEMATIC_POSITION X=100 Y=100 Z=10"
        )
        script_api = Api(printer.api_path)
        script_result = []
        moving = threading.Thread(
            target=lambda: script_result.append(
                script_api.call("gcode/script", script=LONG_MOVES)
            ),
            daemon=True,
        )
        moving.start()

        def playing():
            retired = printer.api.status("ethercat")["ethercat"][
                "retired_cycles"
            ]
            return bool(retired) and min(retired) > 0

        _wait(playing, "the stub to play samples")
        stub.kill()
        _wait(lambda: printer.state()[0] == "shutdown", "klippy to shut down")
        state, message = printer.state()
        assert "ethercat: lost the endpoint" in message, message
        moving.join(API_TIMEOUT_S)
        assert "error" in script_result[0], script_result
    finally:
        stub.kill()
        stub.wait()
        printer.close()
