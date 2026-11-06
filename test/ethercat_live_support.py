"""Live klippy against a Klipper linux-process MCU and ethercat-rt-stub.

Shared by the live smoke and the live homing tests. Needs ETHERCAT_RT_STUB
(an ethercat-rt-stub whose DC grid is on CLOCK_MONOTONIC_RAW, as the
ethercat-rt repository's stub is) and KLIPPER_LINUX_MCU (out/klipper.elf
built from test/configs/linuxprocess.config); kv2-notes/gate.sh builds and
exports both.
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
    printer.cfg text with {mcu_tty} still to fill in."""

    def __init__(self, tmp, config, mcu_env=None):
        self.tmp = tmp
        self.mcu_tty = tmp / "mcu"
        self.api_path = tmp / "api.sock"
        self.log = tmp / "klippy.log"
        self.mcu = subprocess.Popen(
            [LINUX_MCU, "-I", str(self.mcu_tty)],
            env=dict(os.environ, **(mcu_env or {})),
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
