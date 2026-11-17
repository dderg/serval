#!/usr/bin/env python3
"""A machine-wide cap on heavy jobs, so agents sharing one box stop stacking
builds and test runs on each other.

    python3 scripts/slots.py run [--exclusive] [--timeout S] -- CMD ...
    python3 scripts/slots.py status

`run` waits for a free slot, holds it while CMD runs and returns CMD's exit
status. A normal job takes one of SERVAL_SLOTS slots (default 3 on a 16-core
box: each heavy job parallelises across cores on its own). `--exclusive`
takes every slot, so nothing else heavy runs next to it; benchmarks use it for
numbers that load does not distort. Waiting is queued and loud (the wait
graph: holders, and the queue with what each waiter wants, every 60 s) and
bounded: after --timeout (default 2 h) it fails, and after STALL_SECONDS it
fails when the head of the queue has not moved although it could start, or
while a slot holder (or a process under one) sits in the queue: that holder
waits for itself.

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
import json
import os
import random
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_SLOTS = 3
DEFAULT_TIMEOUT_SECONDS = 7200.0
POLL_SECONDS = 1.0
NOTIFY_SECONDS = 60.0
STALL_SECONDS = 300.0
ENV_SLOTS = "SERVAL_SLOTS"
ENV_DIR = "SERVAL_SLOTS_DIR"
ENV_HELD = "SERVAL_SLOT_HELD"


class SlotTimeout(RuntimeError):
    pass


class NestedExclusive(RuntimeError):
    pass


class NestedSlot(RuntimeError):
    pass


class SlotStall(RuntimeError):
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


def lock_owners() -> dict[int, list[int]]:
    """Inode to the pids holding an exclusive flock on it, from /proc/locks
    (reading it takes no lock, unlike probing each file). Its device field is
    the superblock's, which on btrfs is not the file's st_dev: match the inode
    here and the open file in `opens`."""
    owners: dict[int, list[int]] = {}
    for line in Path("/proc/locks").read_text().splitlines():
        parts = line.split()
        if len(parts) < 6 or parts[1] != "FLOCK" or parts[3] != "WRITE":
            continue
        inode = int(parts[5].rsplit(":", 1)[1])
        owners.setdefault(inode, []).append(int(parts[4]))
    return owners


def opens(pid: int, path: Path) -> bool:
    target = str(path.resolve())
    fd_dir = Path(f"/proc/{pid}/fd")
    try:
        fds = os.listdir(fd_dir)
    except FileNotFoundError:
        return False
    except PermissionError:
        return True
    for fd in fds:
        with contextlib.suppress(OSError):
            if os.readlink(fd_dir / fd) == target:
                return True
    return False


def holders(directory: Path, count: int) -> dict[int, list[str]]:
    """Holder pid to the labels of the slot files it holds."""
    owners = lock_owners()
    found: dict[int, list[str]] = {}
    for index in range(count):
        path = directory / f"slot-{index}"
        if not path.exists():
            continue
        for pid in owners.get(path.stat().st_ino, []):
            if opens(pid, path):
                label = path.read_text().strip() or f"pid {pid}"
                found.setdefault(pid, []).append(label)
    return found


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


def ancestors(pid: int) -> list[int]:
    chain = [pid]
    while pid > 1:
        try:
            stat = Path(f"/proc/{pid}/stat").read_text()
        except OSError:
            break
        pid = int(stat[stat.rfind(")") + 2 :].split()[1])
        chain.append(pid)
    return chain


def cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return "gone"
    return raw.replace(b"\0", b" ").decode(errors="replace").strip()[:80]


class Queue:
    """A first-come-first-served line of waiting pids, in a file guarded by
    its own flock, with each waiter's request in waiting/<pid>.json. Only the
    pid at the head may take slots, so an exclusive request is neither starved
    by a stream of normal jobs nor able to starve them by being re-queued
    back to back: whoever asked first goes first."""

    def __init__(self, directory: Path):
        self.path = directory / "queue"
        self.guard = directory / "queue.lock"
        self.requests = directory / "waiting"

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

    def join(self, pid: int, request: dict) -> None:
        self.requests.mkdir(exist_ok=True)
        for stale in self.requests.glob("*.json"):
            if not pid_alive(int(stale.stem)):
                stale.unlink(missing_ok=True)
        (self.requests / f"{pid}.json").write_text(json.dumps(request))
        with self.locked():
            self.write(
                [p for p in self.read() if pid_alive(p) and p != pid] + [pid]
            )

    def live(self) -> list[int]:
        with self.locked():
            pids = [p for p in self.read() if pid_alive(p)]
            self.write(pids)
            return pids

    def request(self, pid: int) -> dict | None:
        try:
            return json.loads((self.requests / f"{pid}.json").read_text())
        except (OSError, ValueError):
            return None

    def leave(self, pid: int) -> None:
        with self.locked():
            self.write([p for p in self.read() if p != pid])
        (self.requests / f"{pid}.json").unlink(missing_ok=True)


@dataclass
class Board:
    """One reading of who holds and who waits."""

    total: int
    holders: dict[int, list[str]]
    queue: list[int]
    requests: dict[int, dict]

    @property
    def free(self) -> int:
        return self.total - sum(map(len, self.holders.values()))

    def holding(self, pid: int) -> int | None:
        return next((p for p in ancestors(pid) if p in self.holders), None)

    def head_could_start(self) -> bool:
        request = self.requests.get(self.queue[0]) if self.queue else None
        if request is None:
            return False
        needed = self.total if request["exclusive"] else 1
        return self.free >= needed

    def waits_for_itself(self) -> list[int]:
        return [pid for pid in self.queue if self.holding(pid) is not None]

    def graph(self) -> str:
        lines = [f"{self.free} of {self.total} slots free"]
        for pid, labels in self.holders.items():
            lines.append(
                f"  holds {len(labels)} slot(s): pid {pid} {labels[0]}"
            )
        for place, pid in enumerate(self.queue, 1):
            request = self.requests.get(pid)
            wants = (
                f"{'every slot' if request['exclusive'] else 'a slot'}, "
                f"{time.time() - request['since']:.0f} s, {request['label']}"
                if request
                else f"? (older slots.py) {cmdline(pid)}"
            )
            holder = self.holding(pid)
            cycle = "" if holder is None else f" [pid {holder} HOLDS a slot]"
            lines.append(f"  {place}. pid {pid} wants {wants}{cycle}")
        return "\n".join(lines)


def read_board(directory: Path, count: int, queue: Queue) -> Board:
    pids = queue.live()
    requests = {pid: r for pid in pids if (r := queue.request(pid)) is not None}
    return Board(count, holders(directory, count), pids, requests)


class StallWatch:
    """A head that keeps its place while it could start, or while a holder
    waits in the queue, does not move on its own: fail with the wait graph."""

    def __init__(self, seconds: float):
        self.seconds = seconds
        self.head: int | None = None
        self.since = time.monotonic()

    def stalled(self, board: Board) -> bool:
        head = board.queue[0] if board.queue else None
        stuck = board.head_could_start() or bool(board.waits_for_itself())
        now = time.monotonic()
        if head != self.head or not stuck:
            self.head, self.since = head, now
            return False
        return now - self.since >= self.seconds


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
    holder = read_board(directory, count, queue).holding(pid)
    if holder is not None:
        raise NestedSlot(
            f"pid {pid} would queue while pid {holder} above it holds a slot "
            f"({ENV_HELD} was dropped on the way)"
        )
    queue.join(
        pid,
        dict(exclusive=exclusive, label=who, since=time.time()),
    )
    watch = StallWatch(STALL_SECONDS)

    def attempt():
        board = read_board(directory, count, queue)
        if board.queue and board.queue[0] == pid:
            held = take_free(directory, count, exclusive)
            if held is not None:
                return held
        if watch.stalled(board):
            raise SlotStall(f"the queue head does not move:\n{board.graph()}")
        return None

    def waiting_for() -> str:
        what = "every slot" if exclusive else "a slot"
        return f"{what}:\n{read_board(directory, count, queue).graph()}"

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
    if os.environ.get(ENV_HELD):
        if exclusive:
            raise NestedExclusive(
                f"{label or 'an exclusive job'} asked for every slot inside a "
                "held one: it would wait for itself"
            )
        yield
        return
    lease = acquire(exclusive=exclusive, timeout=timeout, label=label)
    os.environ[ENV_HELD] = "1"
    try:
        yield
    finally:
        del os.environ[ENV_HELD]
        lease.release()


def run(command: list[str], exclusive: bool, timeout: float) -> int:
    with slot(
        exclusive=exclusive, timeout=timeout, label=" ".join(command)[:80]
    ):
        return subprocess.run(command).returncode


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
        directory = slots_dir()
        directory.mkdir(parents=True, exist_ok=True)
        print(read_board(directory, slot_count(), Queue(directory)).graph())
        return 0
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("run needs a command after --")
    return run(command, args.exclusive, args.timeout)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (SlotTimeout, NestedExclusive, NestedSlot, SlotStall) as exc:
        sys.exit(f"slots: {exc}")
