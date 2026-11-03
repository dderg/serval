from __future__ import annotations

import subprocess
import urllib.error
from pathlib import Path

import pi
import pytest


def _results(*codes):
    outcomes = iter(codes)
    calls = []

    def run(args, **_kwargs):
        calls.append(args)
        code = next(outcomes)
        return subprocess.CompletedProcess(args, code, "", f"scp stderr {code}")

    return run, calls


def test_a_dropped_scp_connection_is_retried(monkeypatch):
    run, calls = _results(255, 0)
    monkeypatch.setattr(pi.subprocess, "run", run)
    monkeypatch.setattr(pi.time, "sleep", lambda _s: None)
    pi.scp_from_pi("/tmp/tap*.bin", Path("/run"))
    assert len(calls) == 2


def test_scp_failing_every_attempt_fails_loudly_with_its_stderr(monkeypatch):
    run, calls = _results(*[255] * pi.SCP_ATTEMPTS)
    monkeypatch.setattr(pi.subprocess, "run", run)
    monkeypatch.setattr(pi.time, "sleep", lambda _s: None)
    with pytest.raises(pi.BenchError, match="scp stderr 255") as raised:
        pi.scp_to_pi(Path("klipper.elf"), "/tmp/klipper.elf")
    assert len(calls) == pi.SCP_ATTEMPTS
    assert f"{pi.SCP_ATTEMPTS} attempts" in str(raised.value)


class _Printer:
    def __init__(self, settles_in: str):
        self.state = settles_in
        self.polls_until_up = 2
        self.restarts = 0

    def http(self, method, path, *_args):
        if self.polls_until_up > 0:
            self.polls_until_up -= 1
            raise urllib.error.URLError("connection refused")
        if path == "/printer/info":
            return {"state": self.state}
        if path == "/printer/firmware_restart":
            self.restarts += 1
            self.state = "ready"
            self.polls_until_up = 2
            return {}
        raise AssertionError(f"unexpected {method} {path}")


@pytest.fixture
def clock(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(pi.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(
        pi.time, "sleep", lambda s: now.__setitem__(0, now[0] + s)
    )
    monkeypatch.setattr(pi, "ssh", lambda _cmd: "")
    monkeypatch.setattr(pi, "push_config", lambda _cfg=None: False)
    return now


def _printer(monkeypatch, settles_in: str) -> _Printer:
    printer = _Printer(settles_in)
    monkeypatch.setattr(pi, "http", printer.http)
    return printer


def test_a_klipper_restart_onto_a_latched_mcu_firmware_restarts_it(
    monkeypatch, clock
):
    printer = _printer(monkeypatch, "shutdown")
    pi.restart_klipper()
    assert printer.restarts == 1
    assert printer.state == "ready"


def test_preparing_a_printer_left_in_shutdown_firmware_restarts_it(
    monkeypatch, clock
):
    printer = _printer(monkeypatch, "shutdown")
    pi.prepare_printer()
    assert printer.restarts == 1
    assert printer.state == "ready"


def test_a_klipper_restart_onto_a_ready_printer_leaves_the_mcu_alone(
    monkeypatch, clock
):
    printer = _printer(monkeypatch, "ready")
    pi.restart_klipper()
    assert printer.restarts == 0


def test_a_printer_that_never_settles_fails_loudly(monkeypatch, clock):
    printer = _printer(monkeypatch, "startup")
    with pytest.raises(pi.BenchError, match="startup"):
        pi.restart_klipper()
    assert printer.restarts == 0
