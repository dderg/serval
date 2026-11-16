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


UNRESOLVED = (
    "ssh: Could not resolve hostname bench-pi.local: "
    "Temporary failure in name resolution"
)


def _ssh_results(*outcomes):
    pending = iter(outcomes)
    calls = []

    def run(args, **_kwargs):
        calls.append(args)
        code, stderr = next(pending)
        return subprocess.CompletedProcess(args, code, "out", stderr)

    return run, calls


def test_ssh_and_scp_reach_the_pi_at_its_address_through_the_alias(
    monkeypatch,
):
    run, calls = _ssh_results((0, ""), (0, ""))
    monkeypatch.setattr(pi.subprocess, "run", run)
    pi.ssh("true")
    pi.scp_to_pi(Path("klipper.elf"), "/tmp/klipper.elf")
    assert pi.PI_ADDRESS[:2] == ("-o", f"HostName={pi.PI_HOST}")
    assert calls[0] == ["ssh", *pi.PI_ADDRESS, pi.PI_SSH, "true"]
    assert calls[1][:-2] == ["scp", "-q", *pi.PI_ADDRESS]


def test_an_ssh_that_never_reached_the_pi_is_retried(monkeypatch):
    run, calls = _ssh_results((255, UNRESOLVED), (0, ""))
    monkeypatch.setattr(pi.subprocess, "run", run)
    monkeypatch.setattr(pi.time, "sleep", lambda _s: None)
    assert pi.ssh("journalctl -u klipper") == "out"
    assert len(calls) == 2


def test_a_pi_unreachable_on_every_attempt_is_a_host_side_error(monkeypatch):
    run, calls = _ssh_results(*[(255, UNRESOLVED)] * pi.SSH_ATTEMPTS)
    monkeypatch.setattr(pi.subprocess, "run", run)
    monkeypatch.setattr(pi.time, "sleep", lambda _s: None)
    with pytest.raises(pi.HostSideError, match="Could not resolve hostname"):
        pi.ssh("journalctl -u klipper")
    assert len(calls) == pi.SSH_ATTEMPTS


@pytest.mark.parametrize(
    "code, stderr", [(1, "No such file"), (255, "exited 255 on the Pi")]
)
def test_a_command_that_ran_on_the_pi_is_not_retried(monkeypatch, code, stderr):
    run, calls = _ssh_results((code, stderr))
    monkeypatch.setattr(pi.subprocess, "run", run)
    with pytest.raises(pi.BenchError) as raised:
        pi.ssh("false")
    assert not isinstance(raised.value, pi.HostSideError)
    assert len(calls) == 1


def test_an_scp_that_never_reached_the_pi_is_a_host_side_error(monkeypatch):
    run, _ = _ssh_results(*[(255, UNRESOLVED)] * pi.SCP_ATTEMPTS)
    monkeypatch.setattr(pi.subprocess, "run", run)
    monkeypatch.setattr(pi.time, "sleep", lambda _s: None)
    with pytest.raises(pi.HostSideError):
        pi.scp_from_pi("/tmp/tap*.bin", Path("/run"))


def _exit_code(main) -> int:
    with pytest.raises(SystemExit) as exited:
        pi.run_main(main)
    return exited.value.code


def test_a_runner_that_never_reached_the_pi_exits_driver_side():
    def main():
        raise pi.HostSideError(UNRESOLVED)

    assert _exit_code(main) == pi.DRIVER_SIDE_EXIT


def test_a_runner_failing_on_a_local_exception_exits_driver_side():
    def main():
        raise FileNotFoundError("run.json")

    assert _exit_code(main) == pi.DRIVER_SIDE_EXIT


def test_a_printer_side_failure_leaves_the_runner_as_a_bench_error():
    def main():
        raise pi.BenchError("klippy did not reach ready")

    with pytest.raises(pi.BenchError, match="did not reach ready"):
        pi.run_main(main)


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


ENDSTOPS = """\
[axis x]
endstop_pin: ^PA9
[axis y]
endstop_pin: ^PA10
[axis z]
endstop_pin: mcu:PA15
[probe]
pin: ~PB0
z_offset: 0
"""


def test_the_f407_driven_lines_may_be_endstops_and_the_probe():
    pi.validate_config(ENDSTOPS)
    pi.validate_config(pi.REPO_CFG.read_text())


@pytest.mark.parametrize(
    ("config", "line"),
    [
        ("[fan]\npin: PA9\n", 2),
        ("[output_pin led]\npin: PB0\n", 2),
        ("[extruder]\nheater_pin: PA10\n", 2),
        ("[motor x]\nstep_pin: PB12\ndir_pin: PA15\n", 3),
        ("[gcode_button b]\npin: PA9\n", 2),
        ("[board_pins]\naliases:\n    EXP1=PB0\n", 3),
        (
            "[axis x]\nendstop_pin: ^PA9\n[static_digital_output s]\npins: PA9\n",
            4,
        ),
    ],
)
def test_a_line_the_f407_drives_is_refused_as_anything_but_an_input(
    config, line
):
    with pytest.raises(
        pi.BenchError,
        match=f"config line {line}: pin P.* is driven by the F407",
    ):
        pi.validate_config(config)


def test_a_pin_named_in_a_comment_is_not_a_use():
    pi.validate_config("[fan]\npin: PB4  # not PA9\n; PB0 spare\n")


def test_every_bench_config_passes_the_pin_guard():
    configs = sorted(Path(pi.__file__).with_name("configs").rglob("*.cfg"))
    assert configs
    for config in configs:
        pi.validate_config(config.read_text())


@pytest.mark.parametrize("pin", ["!PA9", "^!PA9", "!^PA9", "!mcu:PA9"])
def test_an_inverted_f407_driven_endstop_is_refused(pin):
    with pytest.raises(pi.BenchError, match="pin PA9 is inverted"):
        pi.validate_config(f"[axis x]\nendstop_pin: {pin}\n")


def test_an_inverted_endstop_on_another_pin_is_left_alone():
    pi.validate_config("[axis x]\nendstop_pin: !PB3\n")
