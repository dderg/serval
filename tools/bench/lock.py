"""Exclusive bench lock shared by every tool that touches the hardware.

uv run python tools/bench/lock.py hold [--lock-timeout S] -- <command...>

The lock is a flock on ~/bench/.lock, whose content names the holder (pid,
command, cwd, worktree, start, deadline). It lives as long as the open file
description that took it: the held command and every bench tool it starts
inherit that description (BENCH_LOCK_FD, passed with inherited_fds()), so
the kernel releases the lock only when the last of them exits, even when
the holder itself is killed first. Processes of the tree re-enter the lock
through BENCH_LOCK_TOKEN, the holder's pid.

A hold lasts at most MAX_HOLD_S from acquisition. Every process inside it,
the holder and each re-entrant tool, arms a timer at the same deadline (the
lock file's); when it fires the process kills its descendants (SIGTERM,
then SIGKILL after KILL_GRACE_S), names the held command and its elapsed
time on stderr and exits HOLD_EXPIRED_EXIT. The holder is a child
subreaper, so a descendant orphaned by a dead parent stays in its tree.
"""

from __future__ import annotations

import _thread
import argparse
import contextlib
import ctypes
import fcntl
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path

import pi

LOCK_PATH = Path("~/bench/.lock").expanduser()
TOKEN_ENV = "BENCH_LOCK_TOKEN"
FD_ENV = "BENCH_LOCK_FD"
DEFAULT_TIMEOUT_S = 1800.0
REPORT_INTERVAL_S = 30.0
POLL_INTERVAL_S = 0.2
MAX_HOLD_S = 1800.0
KILL_GRACE_S = 10.0
HOLD_EXPIRED_EXIT = 124
PR_SET_CHILD_SUBREAPER = 36
PROC = Path("/proc")
ZOMBIE = "Z"


def add_timeout_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--lock-timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="seconds to wait for the bench lock before failing",
    )


def holder_info(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def describe_holder(path: Path) -> str:
    info = holder_info(path)
    if info is None:
        return "unknown holder"
    held_s = time.time() - info["started"]
    where = info.get("worktree") or info.get("cwd") or "an unrecorded tree"
    return (
        f"pid {info['pid']} running `{info['command']}` from {where} "
        f"for {held_s:.0f} s"
    )


def holder_pid(path: Path) -> int | None:
    try:
        return int(holder_info(path)["pid"])
    except (TypeError, ValueError, KeyError):
        return None


def token_holds_lock(fd: int, path: Path) -> bool:
    token = os.environ.get(TOKEN_ENV)
    if token is None or holder_pid(path) != int(token):
        return False
    return not try_lock(fd)


def inherited_fds(path: Path = LOCK_PATH) -> tuple[int, ...]:
    fd = os.environ.get(FD_ENV)
    if fd is None:
        return ()
    try:
        held = os.fstat(int(fd))
    except OSError:
        return ()
    return (int(fd),) if os.path.samestat(held, os.stat(path)) else ()


def try_lock(fd: int) -> bool:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return False
    return True


def worktree_of(cwd: str) -> str | None:
    toplevel = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        cwd=cwd,
        capture_output=True,
        text=True,
    )
    return toplevel.stdout.strip() if toplevel.returncode == 0 else None


def write_holder(fd: int, command: str) -> dict:
    cwd = os.getcwd()
    started = time.time()
    holder = {
        "pid": os.getpid(),
        "command": command,
        "cwd": cwd,
        "worktree": worktree_of(cwd),
        "started": started,
        "started_at": datetime.fromtimestamp(started).isoformat(),
        "max_hold_s": MAX_HOLD_S,
        "deadline_monotonic": time.monotonic() + MAX_HOLD_S,
    }
    os.ftruncate(fd, 0)
    os.pwrite(fd, json.dumps(holder).encode(), 0)
    return holder


def deadline_of(holder: dict) -> float:
    if "deadline_monotonic" in holder:
        return holder["deadline_monotonic"]
    return time.monotonic() + holder["started"] + MAX_HOLD_S - time.time()


def remaining_s(path: Path = LOCK_PATH) -> float:
    """Seconds left in the hold this process is inside."""
    info = holder_info(path)
    if info is None or os.environ.get(TOKEN_ENV) != str(info["pid"]):
        raise pi.BenchError(f"remaining_s() called outside a hold of {path}")
    return deadline_of(info) - time.monotonic()


@contextlib.contextmanager
def exported(values: dict[str, str]) -> Iterator[None]:
    previous = {name: os.environ.get(name) for name in values}
    os.environ.update(values)
    try:
        yield
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def children_by_parent() -> dict[int, list[int]]:
    children: dict[int, list[int]] = {}
    for entry in PROC.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
        except OSError:
            continue
        state, parent = stat.rpartition(")")[2].split()[:2]
        if state != ZOMBIE:
            children.setdefault(int(parent), []).append(int(entry.name))
    return children


def descendants(pid: int) -> list[int]:
    children = children_by_parent()
    found, frontier = [], [pid]
    while frontier:
        kids = children.get(frontier.pop(), [])
        found += kids
        frontier += kids
    return found


def signal_all(pids: list[int], signum: int) -> None:
    for pid in pids:
        with contextlib.suppress(ProcessLookupError):
            os.kill(pid, signum)


def kill_tree(pid: int) -> None:
    signal_all(descendants(pid), signal.SIGTERM)
    terminated_by = time.monotonic() + KILL_GRACE_S
    while descendants(pid) and time.monotonic() < terminated_by:
        time.sleep(POLL_INTERVAL_S)
    killed_by = terminated_by + KILL_GRACE_S
    while (survivors := descendants(pid)) and time.monotonic() < killed_by:
        signal_all(survivors, signal.SIGKILL)
        time.sleep(POLL_INTERVAL_S)


def expire(command: str, started: float) -> None:
    elapsed_s = time.time() - started
    print(
        f"bench lock: the hold of `{command}` reached MAX_HOLD_S = "
        f"{MAX_HOLD_S:.0f} s after {elapsed_s:.0f} s; killing pid "
        f"{os.getpid()} ({' '.join(sys.argv)}) and its process tree",
        file=sys.stderr,
        flush=True,
    )
    _thread.interrupt_main()
    kill_tree(os.getpid())
    os._exit(HOLD_EXPIRED_EXIT)


@contextlib.contextmanager
def expiring(holder: dict) -> Iterator[None]:
    delay_s = max(0.0, deadline_of(holder) - time.monotonic())
    timer = threading.Timer(
        delay_s, expire, (holder["command"], holder["started"])
    )
    timer.daemon = False
    timer.start()
    try:
        yield
    finally:
        timer.cancel()


def become_subreaper() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        errno = ctypes.get_errno()
        raise pi.BenchError(f"prctl(PR_SET_CHILD_SUBREAPER): errno {errno}")


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
            with expiring(holder_info(path)):
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
        holder = write_holder(fd, command or " ".join(sys.argv))
        become_subreaper()
        with (
            exported({TOKEN_ENV: str(os.getpid()), FD_ENV: str(fd)}),
            expiring(holder),
        ):
            yield
    finally:
        os.close(fd)


def hold(timeout_s: float, command: list[str]) -> int:
    with bench_lock(timeout_s, command=" ".join(command)):
        held = inherited_fds()
        returncode = subprocess.run(command, pass_fds=held).returncode
        while descendants(os.getpid()):
            time.sleep(POLL_INTERVAL_S)
        return returncode


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
