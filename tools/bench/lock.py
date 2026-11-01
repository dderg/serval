"""Exclusive bench lock shared by every tool that touches the hardware."""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import json
import os
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import pi

LOCK_PATH = Path("~/bench/.lock").expanduser()
DEFAULT_TIMEOUT_S = 1800.0
REPORT_INTERVAL_S = 30.0
POLL_INTERVAL_S = 0.2


def add_timeout_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--lock-timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="seconds to wait for the bench lock before failing",
    )


def describe_holder(path: Path) -> str:
    try:
        info = json.loads(path.read_text())
    except (OSError, ValueError):
        return "unknown holder"
    held_s = time.time() - info["started"]
    return f"pid {info['pid']} running `{info['command']}` for {held_s:.0f} s"


def try_lock(fd: int) -> bool:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return False
    return True


@contextlib.contextmanager
def bench_lock(timeout_s: float, path: Path = LOCK_PATH) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        start = time.monotonic()
        next_report = start
        while not try_lock(fd):
            now = time.monotonic()
            if now - start >= timeout_s:
                raise pi.BenchError(
                    f"bench lock {path} still held after {timeout_s:.0f} s "
                    f"by {describe_holder(path)}"
                )
            if now >= next_report:
                print(
                    f"waiting for bench lock held by {describe_holder(path)}",
                    flush=True,
                )
                next_report = now + REPORT_INTERVAL_S
            time.sleep(POLL_INTERVAL_S)
        os.ftruncate(fd, 0)
        os.write(
            fd,
            json.dumps(
                {
                    "pid": os.getpid(),
                    "command": " ".join(sys.argv),
                    "started": time.time(),
                }
            ).encode(),
        )
        yield
    finally:
        os.close(fd)
