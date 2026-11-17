from __future__ import annotations

import json
import os
import re
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
        stat = Path(f"/proc/{pid}/stat").read_text()
    except FileNotFoundError:
        return False
    return stat.rpartition(")")[2].split()[0] != lock.ZOMBIE


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


CAP_S = 2.0
GRACE_S = 1.0


def _capped_hold(home: Path, command: list[str]) -> subprocess.Popen:
    program = (
        f"import sys, lock; lock.MAX_HOLD_S = {CAP_S}; "
        f"lock.KILL_GRACE_S = {GRACE_S}; "
        f"sys.exit(lock.hold({HOLD_TIMEOUT_S}, {command!r}))"
    )
    env = {**os.environ, "HOME": str(home)}
    env.pop(lock.TOKEN_ENV, None)
    env.pop("BENCH_LOCK_FD", None)
    return subprocess.Popen(
        _python(program), env=env, cwd=BENCH, stderr=subprocess.PIPE, text=True
    )


def _wait_dead(pid: int) -> None:
    deadline = time.monotonic() + WAIT_S
    while _alive(pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _alive(pid), f"pid {pid} outlived the hold"


def _reentrant(pid_file: Path, remaining_file: Path) -> str:
    return (
        "import os, time, lock; lock.MAX_HOLD_S = 600; "
        f"ctx = lock.bench_lock({SHORT_TIMEOUT_S}); ctx.__enter__(); "
        f"open({str(remaining_file)!r}, 'w').write(str(lock.remaining_s())); "
        f"open({str(pid_file)!r}, 'w').write(str(os.getpid())); "
        f"time.sleep({CHILD_LIFETIME_S})"
    )


def test_a_hold_past_its_cap_is_killed_and_names_its_command(tmp_path):
    pid_file = tmp_path / "child.pid"
    command = _python(_sleeper(pid_file))
    started = time.monotonic()
    holder = _capped_hold(tmp_path, command)
    child = _wait_for_pid(pid_file)
    _, stderr = holder.communicate(timeout=WAIT_S)
    assert holder.returncode == lock.HOLD_EXPIRED_EXIT
    assert time.monotonic() - started >= CAP_S
    assert f"the hold of `{' '.join(command)}` reached MAX_HOLD_S = 2 s" in (
        stderr
    )
    assert re.search(r"after \d+ s; killing pid \d+", stderr)
    _wait_dead(child)
    _assert_free(_lock_path(tmp_path))


def test_a_command_ignoring_sigterm_is_killed_after_the_grace(tmp_path):
    pid_file = tmp_path / "child.pid"
    stubborn = (
        "import signal; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        + _sleeper(pid_file)
    )
    holder = _capped_hold(tmp_path, _python(stubborn))
    child = _wait_for_pid(pid_file)
    holder.communicate(timeout=WAIT_S)
    assert holder.returncode == lock.HOLD_EXPIRED_EXIT
    _wait_dead(child)
    _assert_free(_lock_path(tmp_path))


def test_a_background_child_outliving_the_command_ends_at_the_cap(tmp_path):
    pid_file = tmp_path / "child.pid"
    detached = f"sleep {CHILD_LIFETIME_S} & echo $! > {pid_file}"
    started = time.monotonic()
    holder = _capped_hold(tmp_path, ["bash", "-c", detached])
    child = _wait_for_pid(pid_file)
    holder.communicate(timeout=WAIT_S)
    assert holder.returncode == lock.HOLD_EXPIRED_EXIT
    assert time.monotonic() - started >= CAP_S
    _wait_dead(child)
    _assert_free(_lock_path(tmp_path))


def test_a_nested_hold_shares_the_outer_deadline(tmp_path):
    pid_file, remaining = tmp_path / "inner.pid", tmp_path / "remaining"
    holder = _capped_hold(tmp_path, _python(_reentrant(pid_file, remaining)))
    inner = _wait_for_pid(pid_file)
    assert 0 < float(remaining.read_text()) <= CAP_S
    holder.communicate(timeout=WAIT_S)
    assert holder.returncode == lock.HOLD_EXPIRED_EXIT
    _wait_dead(inner)


def test_a_reentrant_tool_ends_at_the_deadline_after_its_holder_dies(
    tmp_path,
):
    pid_file, remaining = tmp_path / "inner.pid", tmp_path / "remaining"
    holder = _capped_hold(tmp_path, _python(_reentrant(pid_file, remaining)))
    inner = _wait_for_pid(pid_file)
    holder.kill()
    holder.wait()
    assert _alive(inner)
    _wait_dead(inner)
    _assert_free(_lock_path(tmp_path))


def test_remaining_time_is_refused_outside_a_hold(tmp_path):
    with pytest.raises(lock.pi.BenchError, match="outside a hold"):
        lock.remaining_s(_lock_path(tmp_path))
