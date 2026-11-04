#!/usr/bin/env python3
"""A machine-wide cap on heavy jobs, so agents sharing one box stop stacking
builds and test runs on each other.

    python3 scripts/slots.py run [--exclusive] [--timeout S] -- CMD ...
    python3 scripts/slots.py status

`run` waits for a free slot, holds it while CMD runs and returns CMD's exit
status. A normal job takes one of SERVAL_SLOTS slots (default 3 on a 16-core
box: each heavy job parallelises across cores on its own). `--exclusive`
takes every slot, so nothing else heavy runs next to it; benchmarks use it for
numbers that load does not distort. Waiting is queued and loud (who holds the
slots, every 60 s) and bounded: after --timeout (default 2 h) it fails.

The locks are flock(2) files under SERVAL_SLOTS_DIR (default
~/.cache/serval-slots): the kernel frees them when the holder dies, so a
crashed job cannot wedge the queue. Waiters
form a first-come-first-served queue (a pid file), so an exclusive request is
not starved by a stream of normal jobs and cannot starve them by being
re-queued back to back.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import os
import random
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_SLOTS = 3
DEFAULT_TIMEOUT_SECONDS = 7200.0
POLL_SECONDS = 1.0
NOTIFY_SECONDS = 60.0
ENV_SLOTS = "SERVAL_SLOTS"
ENV_DIR = "SERVAL_SLOTS_DIR"
ENV_HELD = "SERVAL_SLOT_HELD"


class SlotTimeout(RuntimeError):
    pass


def slot_count() -> int:
    count = int(os.environ.get(ENV_SLOTS, DEFAULT_SLOTS))
    if count < 1:
        raise ValueError(f"{ENV_SLOTS} must be at least 1, got {count}")
    return count


def slots_dir() -> Path:
    default = Path.home() / ".cache" / "serval-slots"
    return Path(os.environ.get(ENV_DIR) or default)


def try_lock(fd: int) -> bool:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return False
    return True


def open_lock(path: Path) -> int:
    return os.open(path, os.O_RDWR | os.O_CREAT, 0o666)


def describe_holders(directory: Path, count: int) -> str:
    lines = []
    for index in range(count):
        path = directory / f"slot-{index}"
        if not path.exists():
            continue
        probe = open_lock(path)
        try:
            if try_lock(probe):
                continue
            lines.append(
                f"slot {index}: {path.read_text().strip() or 'unknown'}"
            )
        finally:
            os.close(probe)
    return "; ".join(lines) or "no holder visible"


class Lease:
    def __init__(self, fds: list[int]):
        self.fds = fds

    def release(self) -> None:
        for fd in self.fds:
            os.close(fd)
        self.fds = []


def poll_until(attempt, deadline, notify, waiting_for, started):
    last_notice = started
    while True:
        result = attempt()
        if result is not None:
            return result
        now = time.monotonic()
        if now >= deadline:
            raise SlotTimeout(f"timed out waiting for {waiting_for()}")
        if now - last_notice >= NOTIFY_SECONDS:
            notify(f"slots: waiting for {waiting_for()}")
            last_notice = now
        time.sleep(POLL_SECONDS * (0.5 + random.random()))


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class Queue:
    """A first-come-first-served line of waiting pids, in a file guarded by
    its own flock. Only the pid at the head may take slots, so an exclusive
    request is neither starved by a stream of normal jobs nor able to starve
    them by being re-queued back to back: whoever asked first goes first."""

    def __init__(self, directory: Path):
        self.path = directory / "queue"
        self.guard = directory / "queue.lock"

    @contextlib.contextmanager
    def locked(self):
        fd = open_lock(self.guard)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            os.close(fd)

    def read(self) -> list[int]:
        if not self.path.exists():
            return []
        return [int(x) for x in self.path.read_text().split() if x.isdigit()]

    def write(self, pids: list[int]) -> None:
        self.path.write_text("".join(f"{pid}\n" for pid in pids))

    def join(self, pid: int) -> None:
        with self.locked():
            self.write(
                [p for p in self.read() if pid_alive(p) and p != pid] + [pid]
            )

    def is_head(self, pid: int) -> bool:
        with self.locked():
            live = [p for p in self.read() if pid_alive(p)]
            self.write(live)
            return bool(live) and live[0] == pid

    def leave(self, pid: int) -> None:
        with self.locked():
            self.write([p for p in self.read() if p != pid])


def take_free(directory: Path, count: int, exclusive: bool) -> list[int] | None:
    held: list[int] = []
    for index in range(count):
        fd = open_lock(directory / f"slot-{index}")
        if try_lock(fd):
            held.append(fd)
            if not exclusive:
                return held
        else:
            os.close(fd)
            if exclusive:
                break
    if exclusive and len(held) == count:
        return held
    for fd in held:
        os.close(fd)
    return None


def acquire(
    exclusive: bool = False,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    label: str = "",
    notify=lambda message: print(message, file=sys.stderr, flush=True),
) -> Lease:
    directory = slots_dir()
    directory.mkdir(parents=True, exist_ok=True)
    count = slot_count()
    started = time.monotonic()
    pid = os.getpid()
    who = f"pid {pid} {label or ' '.join(sys.argv)[:80]}".strip()
    queue = Queue(directory)
    queue.join(pid)

    def attempt():
        if not queue.is_head(pid):
            return None
        return take_free(directory, count, exclusive)

    def waiting_for() -> str:
        what = "every slot" if exclusive else "a slot"
        return f"{what} ({describe_holders(directory, count)}; queue {queue.read()})"

    try:
        held = poll_until(
            attempt, started + timeout, notify, waiting_for, started
        )
    finally:
        queue.leave(pid)
    for fd in held:
        os.ftruncate(fd, 0)
        os.write(fd, f"{who}\n".encode())
    return Lease(held)


@contextlib.contextmanager
def slot(
    exclusive: bool = False,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    label: str = "",
):
    if os.environ.get(ENV_HELD) and not exclusive:
        yield
        return
    lease = acquire(exclusive=exclusive, timeout=timeout, label=label)
    try:
        yield
    finally:
        lease.release()


def run(command: list[str], exclusive: bool, timeout: float) -> int:
    with slot(
        exclusive=exclusive, timeout=timeout, label=" ".join(command)[:80]
    ):
        env = {**os.environ, ENV_HELD: "1"}
        return subprocess.run(command, env=env).returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="mode", required=True)
    runner = sub.add_parser("run")
    runner.add_argument("--exclusive", action="store_true")
    runner.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
    runner.add_argument("command", nargs=argparse.REMAINDER)
    sub.add_parser("status")
    args = parser.parse_args()
    if args.mode == "status":
        print(describe_holders(slots_dir(), slot_count()))
        return 0
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("run needs a command after --")
    return run(command, args.exclusive, args.timeout)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SlotTimeout as exc:
        sys.exit(f"slots: {exc}")
