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
crashed job cannot wedge the queue. A `gate` lock serialises entry, so an
exclusive waiter is not starved by a stream of normal jobs.
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
    deadline = started + timeout
    who = f"pid {os.getpid()} {label or ' '.join(sys.argv)[:80]}".strip()

    def waiting_for() -> str:
        what = "every slot" if exclusive else "a slot"
        return f"{what} ({describe_holders(directory, count)})"

    if exclusive:
        held = take_all(
            directory, count, deadline, notify, waiting_for, started
        )
    else:
        held = [
            poll_until(
                lambda: first_free(directory, count),
                deadline,
                notify,
                waiting_for,
                started,
            )
        ]
    for fd in held:
        os.ftruncate(fd, 0)
        os.write(fd, f"{who}\n".encode())
    return Lease(held)


def take_all(
    directory, count, deadline, notify, waiting_for, started
) -> list[int]:
    gate = open_lock(directory / "gate")
    held: list[int] = []
    try:
        poll_until(
            lambda: True if try_lock(gate) else None,
            deadline,
            notify,
            lambda: "the entry gate",
            started,
        )
        for index in range(count):
            fd = open_lock(directory / f"slot-{index}")
            held.append(fd)
            poll_until(
                lambda fd=fd: True if try_lock(fd) else None,
                deadline,
                notify,
                waiting_for,
                started,
            )
        return held
    except BaseException:
        for fd in held:
            os.close(fd)
        raise
    finally:
        os.close(gate)


def first_free(directory: Path, count: int) -> int | None:
    gate = open_lock(directory / "gate")
    try:
        if not try_lock(gate):
            return None
        for index in range(count):
            fd = open_lock(directory / f"slot-{index}")
            if try_lock(fd):
                return fd
            os.close(fd)
        return None
    finally:
        os.close(gate)


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
