"""Live klippy against a Klipper linux-process MCU and ethercat-rt-stub.

Shared by the live smoke and the live homing tests. Needs ETHERCAT_RT_STUB
(an ethercat-rt-stub whose DC grid is on CLOCK_MONOTONIC_RAW, as the
ethercat-rt repository's stub is) and KLIPPER_LINUX_MCU (out/klipper.elf
built from test/configs/linuxprocess.config); kv2-notes/gate.sh builds and
exports both. The linux MCU runs SCHED_FIFO (its -r flag) where the user's
RLIMIT_RTPRIO allows it, as klipper-mcu does in production: a time-shared
MCU process misses step deadlines by more than the 1 ms "Stepper too far in
past" bound on a loaded box.
"""

from __future__ import annotations

import json
import os
import pathlib
import resource
import socket
import subprocess
import sys
import tempfile
import threading
import time

import pytest
from ethercat_support import REPO_ROOT

STUB = os.environ.get("ETHERCAT_RT_STUB")
LINUX_MCU = os.environ.get("KLIPPER_LINUX_MCU")
needs_live = pytest.mark.skipif(
    not (STUB and LINUX_MCU),
    reason="ETHERCAT_RT_STUB and KLIPPER_LINUX_MCU not set",
)

READY_TIMEOUT_S = 30.0
API_TIMEOUT_S = 60.0
STUB_CYCLE_US = 1000
COUNTS_PER_REV = 131072
GPIO_SIM_CHIP = 15
# The simulated stepper (gpio_sim.c) is on lines 0-2 of the chip; any
# other line is a plain output, or an input that reads 0
SIM_STEPPER_PINS = {
    "step_pin": "gpiochip%d/gpio0" % (GPIO_SIM_CHIP,),
    "dir_pin": "gpiochip%d/gpio1" % (GPIO_SIM_CHIP,),
    "endstop_pin": "gpiochip%d/gpio2" % (GPIO_SIM_CHIP,),
}
NEVER_TRIPS = -(2**31)

# Servos move only smoothed axes, and [input_shaper] cannot shape Z, so Z
# is a linux-MCU stepper on GPIO lines nobody reads
Z_STEPPER = """[stepper_z]
step_pin: gpiochip{chip}/gpio3
dir_pin: gpiochip{chip}/gpio4
endstop_pin: gpiochip{chip}/gpio5
microsteps: 16
rotation_distance: 8
position_endstop: 0
position_max: 200
""".format(chip=GPIO_SIM_CHIP)

INPUT_SMOOTHER = """[input_shaper]
shaper_type: smooth_mzv
shaper_freq_x: 50
shaper_freq_y: 40
"""

_gpio_sim = None


def gpio_sim_library():
    global _gpio_sim
    if _gpio_sim is None:
        lib = pathlib.Path(tempfile.mkdtemp(prefix="gpio_sim")) / "gpio_sim.so"
        subprocess.run(
            [
                "cc",
                "-shared",
                "-fPIC",
                "-O2",
                "-Wall",
                "-Werror",
                "-o",
                str(lib),
                str(REPO_ROOT / "test" / "gpio_sim.c"),
                "-ldl",
            ],
            check=True,
        )
        _gpio_sim = lib
    return _gpio_sim


def short_tmp(tmp_path_factory):
    # Unix socket paths are limited to ~100 bytes; pytest's are long
    return pathlib.Path(tmp_path_factory.mktemp("ec", numbered=True))


def wait_for(predicate, what, timeout=READY_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting for " + what)
        time.sleep(0.05)


def spawn_stub(sock, slots, extra_args=()):
    """slots: [(axis, counts_per_mm)] in slave order."""
    args = [STUB, "--socket", str(sock)]
    for slave, (axis, counts_per_mm) in enumerate(slots):
        args += [
            "--slave",
            str(slave),
            "--axis",
            str(axis),
            "--counts-per-mm",
            repr(counts_per_mm),
        ]
    return subprocess.Popen(args + list(extra_args))


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

    def script(self, script):
        return self.call("gcode/script", script=script)

    def status(self, *objects):
        resp = self.call("objects/query", objects={o: None for o in objects})
        return resp["result"]["status"]


class Printer:
    """klippy (with an API socket) on a linux-process MCU; config is the
    printer.cfg text with {mcu_tty} still to fill in. The MCU runs with
    gpio_sim; its stepper closes its endstop sim_trip_steps from where it
    started."""

    def __init__(self, tmp, config, sim_trip_steps=NEVER_TRIPS):
        self.tmp = tmp
        self.mcu_tty = tmp / "mcu"
        self.api_path = tmp / "api.sock"
        self.log = tmp / "klippy.log"
        self.mcu = subprocess.Popen(
            [LINUX_MCU, "-I", str(self.mcu_tty)] + realtime_flag(),
            env=dict(
                os.environ,
                LD_PRELOAD=str(gpio_sim_library()),
                KV2_GPIO_SIM_CHIP=str(GPIO_SIM_CHIP),
                KV2_GPIO_SIM_STEP="0",
                KV2_GPIO_SIM_DIR="1",
                KV2_GPIO_SIM_ENDSTOP="2",
                KV2_GPIO_SIM_TRIP_STEPS=str(sim_trip_steps),
            ),
        )
        wait_for(lambda: self.mcu_tty.exists(), "the linux MCU tty")
        config_path = tmp / "printer.cfg"
        config_path.write_text(config.replace("{mcu_tty}", str(self.mcu_tty)))
        self.klippy = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "klippy",
                str(config_path),
                "-l",
                str(self.log),
                "-a",
                str(self.api_path),
            ],
            cwd=REPO_ROOT,
            env=dict(os.environ, PYTHONPATH=str(REPO_ROOT)),
        )
        wait_for(lambda: self.api_path.exists(), "the klippy API socket")
        self.api = Api(self.api_path)
        wait_for(
            lambda: self.state()[0] in ("ready", "shutdown", "error"),
            "klippy to start",
        )

    def state(self):
        info = self.api.call("info")["result"]
        return info["state"], info["state_message"]

    def close(self):
        for proc in (self.klippy, self.mcu):
            proc.terminate()
            proc.wait(10)


def realtime_flag():
    """The linux MCU asks for half the maximum SCHED_FIFO priority."""
    wanted = os.sched_get_priority_max(os.SCHED_FIFO) // 2
    soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
    return ["-r"] if soft >= wanted else []
