from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import lock
import pytest

BENCH = Path(__file__).resolve().parent
WAIT_S = 10.0
SHORT_TIMEOUT_S = 0.5
CHILD_LIFETIME_S = 60
HOLD_TIMEOUT_S = 5


@pytest.fixture(autouse=True)
def outside_any_hold(monkeypatch):
    for name in (lock.TOKEN_ENV, "BENCH_LOCK_FD"):
        monkeypatch.delenv(name, raising=False)


def _lock_path(home: Path) -> Path:
    return home / "bench" / ".lock"


def _python(script: str) -> list[str]:
    prelude = f"import sys; sys.path.insert(0, {str(BENCH)!r}); "
    return [sys.executable, "-c", prelude + script]


def _hold(home: Path, command: list[str]) -> subprocess.Popen:
    env = {**os.environ, "HOME": str(home)}
    env.pop(lock.TOKEN_ENV, None)
    env.pop("BENCH_LOCK_FD", None)
    return subprocess.Popen(
        [
            sys.executable,
            str(BENCH / "lock.py"),
            "hold",
            "--lock-timeout",
            str(HOLD_TIMEOUT_S),
            "--",
            *command,
        ],
        env=env,
        cwd=BENCH,
    )


def _sleeper(pid_file: Path) -> str:
    return (
        f"import os, time; open({str(pid_file)!r}, 'w').write(str(os.getpid())); "
        f"time.sleep({CHILD_LIFETIME_S})"
    )


def _wait_for_pid(pid_file: Path) -> int:
    deadline = time.monotonic() + WAIT_S
    while time.monotonic() < deadline:
        if pid_file.exists() and pid_file.read_text():
            return int(pid_file.read_text())
        time.sleep(0.05)
    raise AssertionError(f"{pid_file} never appeared")


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _assert_still_held(path: Path) -> None:
    with pytest.raises(lock.pi.BenchError, match="still held"):
        with lock.bench_lock(SHORT_TIMEOUT_S, path):
            pass


def _assert_free(path: Path) -> None:
    with lock.bench_lock(WAIT_S, path):
        pass


def test_a_killed_holder_keeps_the_lock_while_its_command_runs(tmp_path):
    pid_file = tmp_path / "child.pid"
    holder = _hold(tmp_path, _python(_sleeper(pid_file)))
    child = _wait_for_pid(pid_file)
    holder.kill()
    holder.wait()
    assert _alive(child)
    _assert_still_held(_lock_path(tmp_path))
    os.kill(child, signal.SIGKILL)
    _assert_free(_lock_path(tmp_path))


def test_a_bench_tools_own_child_keeps_the_lock_after_its_parents_die(
    tmp_path,
):
    pid_file = tmp_path / "grandchild.pid"
    parent_file = tmp_path / "parent.pid"
    grandchild = _python(_sleeper(pid_file))
    tool = (
        "import os, subprocess, time, lock; "
        f"open({str(parent_file)!r}, 'w').write(str(os.getpid())); "
        f"ctx = lock.bench_lock({SHORT_TIMEOUT_S}); ctx.__enter__(); "
        f"subprocess.Popen({grandchild!r}, pass_fds=lock.inherited_fds()); "
        f"time.sleep({CHILD_LIFETIME_S})"
    )
    holder = _hold(tmp_path, _python(tool))
    leaf = _wait_for_pid(pid_file)
    os.kill(_wait_for_pid(parent_file), signal.SIGKILL)
    holder.kill()
    holder.wait()
    assert _alive(leaf)
    _assert_still_held(_lock_path(tmp_path))
    os.kill(leaf, signal.SIGKILL)
    _assert_free(_lock_path(tmp_path))


def test_the_held_command_reenters_the_lock_without_waiting(tmp_path):
    reenter = f"import lock; lock.bench_lock({SHORT_TIMEOUT_S}).__enter__()"
    assert _hold(tmp_path, _python(reenter)).wait() == 0


def test_a_hold_inside_a_hold_passes_the_lock_on(tmp_path):
    reenter = f"import lock; lock.bench_lock({SHORT_TIMEOUT_S}).__enter__()"
    inner = [
        sys.executable,
        str(BENCH / "lock.py"),
        "hold",
        "--lock-timeout",
        str(SHORT_TIMEOUT_S),
        "--",
        *_python(reenter),
    ]
    assert _hold(tmp_path, inner).wait() == 0


def test_the_holder_records_pid_command_worktree_and_start(tmp_path):
    pid_file = tmp_path / "child.pid"
    holder = _hold(tmp_path, _python(_sleeper(pid_file)))
    child = _wait_for_pid(pid_file)
    info = json.loads(_lock_path(tmp_path).read_text())
    assert info["pid"] == holder.pid
    assert info["command"].startswith(sys.executable)
    assert info["worktree"] == str(BENCH.parents[1])
    assert info["started"] <= time.time()
    description = lock.describe_holder(_lock_path(tmp_path))
    assert f"pid {holder.pid}" in description
    assert str(BENCH.parents[1]) in description
    os.kill(child, signal.SIGKILL)
    holder.wait()


def test_a_waiter_gives_up_after_its_timeout_naming_the_holder(tmp_path):
    pid_file = tmp_path / "child.pid"
    holder = _hold(tmp_path, _python(_sleeper(pid_file)))
    child = _wait_for_pid(pid_file)
    started = time.monotonic()
    with pytest.raises(lock.pi.BenchError, match=f"pid {holder.pid}"):
        with lock.bench_lock(SHORT_TIMEOUT_S, _lock_path(tmp_path)):
            pass
    assert time.monotonic() - started >= SHORT_TIMEOUT_S
    os.kill(child, signal.SIGKILL)
    holder.wait()
