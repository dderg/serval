from __future__ import annotations

import subprocess
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
