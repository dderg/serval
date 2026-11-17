"""Klippy's resident memory, watched for the whole scenario. A planner
allocation blow-up (#901: very low velocity limits ask for gigabytes)
gets its own verdict instead of swapping the box or running into the
scenario's wall budget: past the budget the watch kills klippy and the
scenario fails `memory` with the peak it saw. Klippy SIGKILLed by
someone else (the container's memory cap) fails the same way."""

from __future__ import annotations

import pathlib
import signal
import threading
from typing import Callable, Optional

from tools.sim.fuzzing.checks import Check, Violation

KLIPPY_RSS_BUDGET_MB = 2048
SAMPLE_REAL_S = 0.25
KIB_PER_MB = 1024


def rss_mb(pid: int) -> Optional[float]:
    """VmRSS of `pid` in MB, None once the process is gone."""
    try:
        status = pathlib.Path(f"/proc/{pid}/status").read_text()
    except OSError:
        return None
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / KIB_PER_MB
    return None


class MemoryWatch:
    """Samples the process `current_proc()` returns (klippy, which a
    restart may replace) every SAMPLE_REAL_S of real time."""

    def __init__(
        self, current_proc: Callable, budget_mb: float = KLIPPY_RSS_BUDGET_MB
    ):
        self._current_proc = current_proc
        self._budget_mb = budget_mb
        self.peak_mb = 0.0
        self.killed_over_budget = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self) -> "MemoryWatch":
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self) -> None:
        while not self._stop.wait(SAMPLE_REAL_S):
            proc = self._current_proc()
            if proc is None or proc.poll() is not None:
                continue
            used = rss_mb(proc.pid)
            if used is None:
                continue
            self.peak_mb = max(self.peak_mb, used)
            if used > self._budget_mb:
                self.killed_over_budget = True
                proc.kill()
                return

    def violation(self, proc) -> Optional[Violation]:
        """The memory verdict once the scenario is over, if any."""
        if self.killed_over_budget:
            return Violation(
                Check.MEMORY,
                f"klippy RSS passed the {self._budget_mb} MB budget "
                f"(peak {self.peak_mb:.0f} MB); the watch killed it",
            )
        if proc is not None and proc.returncode == -signal.SIGKILL:
            return Violation(
                Check.MEMORY,
                f"klippy SIGKILLed (container memory cap?), peak RSS seen "
                f"{self.peak_mb:.0f} MB",
            )
        return None
