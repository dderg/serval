"""Move the calling process into its own transient systemd user scope with a
hard memory cap, so a runaway job is OOM-killed alone. On 11-17 at 00:44 and
00:52 a sweep that grew to ~55 GB inside the SSH login session scope pushed
swap past 90 % and systemd-oomd killed that whole scope: the terminal
multiplexer, every agent and 300+ worker processes. The same sweep in its own
unit was later killed alone.

The scope is created over D-Bus (`busctl --user ... StartTransientUnit` with
PIDs = this process), so the caller and everything it starts afterwards are
inside it; the caller stays alive after a kill (OOMPolicy=continue) and reads
the scope's own memory.events to name the failure.
"""

from __future__ import annotations

import contextlib
import enum
import os
import resource
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

GIB = 2**30
ENV_UNCAPPED = "SERVAL_SLOT_UNCAPPED"
MOVE_SECONDS = 5.0
CGROUP_ROOT = Path("/sys/fs/cgroup")


class NoUserManager(RuntimeError):
    pass


class MemoryCapKill(RuntimeError):
    pass


class CoreDumps(enum.Enum):
    KEEP = "keep"
    DROP = "drop"


@dataclass(frozen=True)
class Caps:
    high_gib: int
    max_gib: int
    swap_gib: int
    cores: CoreDumps


def start_call(unit: str, pid: int, caps: Caps, description: str) -> list[str]:
    properties = [
        ("PIDs", "au", ["1", str(pid)]),
        ("MemoryHigh", "t", [str(caps.high_gib * GIB)]),
        ("MemoryMax", "t", [str(caps.max_gib * GIB)]),
        ("MemorySwapMax", "t", [str(caps.swap_gib * GIB)]),
        ("OOMPolicy", "s", ["continue"]),
        ("CollectMode", "s", ["inactive-or-failed"]),
        ("Description", "s", [description]),
    ]
    flat = [
        item
        for name, kind, values in properties
        for item in (name, kind, *values)
    ]
    return [
        "busctl",
        "--user",
        "call",
        "org.freedesktop.systemd1",
        "/org/freedesktop/systemd1",
        "org.freedesktop.systemd1.Manager",
        "StartTransientUnit",
        "ssa(sv)a(sa(sv))",
        unit,
        "fail",
        str(len(properties)),
        *flat,
        "0",
    ]


def own_cgroup() -> Path:
    line = Path("/proc/self/cgroup").read_text().strip()
    return CGROUP_ROOT / line.split(":", 2)[2].lstrip("/")


def oom_kills(cgroup: Path) -> int:
    for line in (cgroup / "memory.events").read_text().splitlines():
        name, value = line.split()
        if name == "oom_kill":
            return int(value)
    raise RuntimeError(f"{cgroup}/memory.events has no oom_kill line")


@dataclass
class Scope:
    unit: str
    caps: Caps
    cgroup: Path
    kills_before: int

    def killed(self) -> bool:
        return oom_kills(self.cgroup) > self.kills_before

    def kill_report(self, label: str) -> str:
        peak = int((self.cgroup / "memory.peak").read_text()) / GIB
        return (
            f"{label} was killed by its memory cap (MemoryMax="
            f"{self.caps.max_gib} GiB, peak {peak:.1f} GiB, unit {self.unit})"
        )


def enter(label: str, caps: Caps) -> Scope:
    pid = os.getpid()
    unit = f"serval-slot-{pid}-{time.monotonic_ns()}.scope"
    started = subprocess.run(
        start_call(unit, pid, caps, label[:200]),
        capture_output=True,
        text=True,
    )
    if started.returncode != 0:
        raise NoUserManager(
            f"cannot start {unit}: {started.stderr.strip()}. Heavy jobs run "
            "memory-capped in their own systemd user scope; where no user "
            f"manager exists (a container), set {ENV_UNCAPPED}=1"
        )
    deadline = time.monotonic() + MOVE_SECONDS
    while own_cgroup().name != unit:
        if time.monotonic() > deadline:
            raise NoUserManager(f"{unit} started but pid {pid} never moved in")
        time.sleep(0.05)
    cgroup = own_cgroup()
    return Scope(unit, caps, cgroup, oom_kills(cgroup))


@contextlib.contextmanager
def capped(label: str, caps: Caps):
    """Run the block in its own capped scope; a cap kill during it raises
    MemoryCapKill naming the job and its peak."""
    if os.environ.get(ENV_UNCAPPED):
        yield
        return
    scope = enter(label, caps)
    core = resource.getrlimit(resource.RLIMIT_CORE)
    if caps.cores is CoreDumps.DROP:
        resource.setrlimit(resource.RLIMIT_CORE, (0, core[1]))
    try:
        yield
    except BaseException as exc:
        if scope.killed():
            raise MemoryCapKill(scope.kill_report(label)) from exc
        raise
    finally:
        resource.setrlimit(resource.RLIMIT_CORE, core)
    if scope.killed():
        raise MemoryCapKill(scope.kill_report(label))
