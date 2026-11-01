"""Exclusive bench lock shared by every tool that touches the hardware.

uv run python tools/bench/lock.py hold [--lock-timeout S] -- <command...>
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import json
import os
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import pi

LOCK_PATH = Path("~/bench/.lock").expanduser()
TOKEN_ENV = "BENCH_LOCK_TOKEN"
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


def holder_pid(path: Path) -> int | None:
    try:
        return int(json.loads(path.read_text())["pid"])
    except (OSError, ValueError, KeyError):
        return None


def token_holds_lock(fd: int, path: Path) -> bool:
    token = os.environ.get(TOKEN_ENV)
    if token is None or holder_pid(path) != int(token):
        return False
    return not try_lock(fd)


def try_lock(fd: int) -> bool:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return False
    return True


@contextlib.contextmanager
def bench_lock(
    timeout_s: float,
    path: Path = LOCK_PATH,
    command: str | None = None,
) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        if token_holds_lock(fd, path):
            yield
            return
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
                    "command": command or " ".join(sys.argv),
                    "started": time.time(),
                }
            ).encode(),
        )
        yield
    finally:
        os.close(fd)


def hold(timeout_s: float, command: list[str]) -> int:
    with bench_lock(timeout_s, command=" ".join(command)):
        env = {**os.environ, TOKEN_ENV: str(os.getpid())}
        return subprocess.run(command, env=env).returncode


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    holder = sub.add_parser("hold", help="run a command under the bench lock")
    add_timeout_argument(holder)
    holder.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("hold needs a command after --")
    sys.exit(hold(args.lock_timeout, command))


if __name__ == "__main__":
    main()
