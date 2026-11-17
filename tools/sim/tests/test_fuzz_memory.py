"""The fuzzer's memory verdict: a process past its RSS budget is killed
and reported `memory` with its peak; one SIGKILLed from outside (the
container's cap) is reported the same way; a quiet one is not."""

import subprocess
import sys

import pytest

from tools.sim.fuzzing.checks import Check
from tools.sim.fuzzing.memory import MemoryWatch

pytestmark = pytest.mark.sim_unit

GROW_MB = 64
TINY_BUDGET_MB = 32
QUIET_BUDGET_MB = 4096
GROWER = (
    f"blob = bytearray({GROW_MB} * 1024 * 1024)\nimport time\ntime.sleep(30)\n"
)
SLEEPER = "import time\ntime.sleep(30)\n"


def _spawn(script: str) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, "-c", script])


def test_a_process_past_its_budget_is_killed_and_named():
    proc = _spawn(GROWER)
    with MemoryWatch(lambda: proc, budget_mb=TINY_BUDGET_MB) as watch:
        proc.wait(timeout=20)
    violation = watch.violation(proc)
    assert violation.check is Check.MEMORY
    assert watch.peak_mb > TINY_BUDGET_MB


def test_an_outside_sigkill_is_a_memory_verdict():
    proc = _spawn(SLEEPER)
    with MemoryWatch(lambda: proc, budget_mb=QUIET_BUDGET_MB) as watch:
        proc.kill()
        proc.wait(timeout=5)
    assert watch.violation(proc).check is Check.MEMORY


def test_a_quiet_process_has_no_memory_verdict():
    proc = _spawn(SLEEPER)
    with MemoryWatch(lambda: proc, budget_mb=QUIET_BUDGET_MB) as watch:
        proc.terminate()
        proc.wait(timeout=5)
    assert watch.violation(proc) is None
