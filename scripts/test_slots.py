import contextlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
import slots

SCRIPT = Path(__file__).with_name("slots.py")
HOLD_SECONDS = 0.8

JOB = (
    "import json, sys, time\n"
    "path, name, hold = sys.argv[1], sys.argv[2], float(sys.argv[3])\n"
    "start = time.time(); time.sleep(hold); end = time.time()\n"
    "open(path, 'a').write(json.dumps([name, start, end]) + '\\n')\n"
)


@pytest.fixture(autouse=True)
def uncapped(monkeypatch):
    monkeypatch.setenv("SERVAL_SLOT_UNCAPPED", "1")


def env(tmp_path, count):
    return {
        **os.environ,
        "SERVAL_SLOTS_DIR": str(tmp_path / "slots"),
        "SERVAL_SLOTS": str(count),
        "SERVAL_SLOT_HELD": "",
    }


def start(tmp_path, count, name, *flags, hold=HOLD_SECONDS):
    return subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            *flags,
            "--",
            sys.executable,
            "-c",
            JOB,
            str(tmp_path / "log"),
            name,
            str(hold),
        ],
        env=env(tmp_path, count),
    )


def records(tmp_path):
    return [
        json.loads(line) for line in (tmp_path / "log").read_text().splitlines()
    ]


def max_overlap(rows):
    events = sorted(
        [(s, 1) for _, s, _ in rows] + [(e, -1) for _, _, e in rows]
    )
    level = peak = 0
    for _, delta in events:
        level += delta
        peak = max(peak, level)
    return peak


def test_no_more_jobs_run_at_once_than_there_are_slots(tmp_path):
    jobs = [start(tmp_path, 2, f"j{i}") for i in range(5)]
    assert [job.wait(timeout=60) for job in jobs] == [0] * 5
    rows = records(tmp_path)
    assert len(rows) == 5
    assert max_overlap(rows) == 2


def test_an_exclusive_job_runs_alone(tmp_path):
    first = start(tmp_path, 3, "a")
    second = start(tmp_path, 3, "b")
    time.sleep(0.3)
    exclusive = start(tmp_path, 3, "x", "--exclusive")
    late = start(tmp_path, 3, "c")
    for job in (first, second, exclusive, late):
        assert job.wait(timeout=60) == 0
    rows = {name: (s, e) for name, s, e in records(tmp_path)}
    xs, xe = rows["x"]
    for name in "abc":
        s, e = rows[name]
        assert e <= xs + 0.05 or s >= xe - 0.05, (
            f"{name} overlapped the exclusive job"
        )


def test_the_commands_exit_status_is_returned(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--",
            sys.executable,
            "-c",
            "raise SystemExit(7)",
        ],
        env=env(tmp_path, 1),
    )
    assert result.returncode == 7


def test_waiting_past_the_timeout_fails_loudly(tmp_path):
    holder = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--",
            sys.executable,
            "-c",
            "import time; time.sleep(6)",
        ],
        env=env(tmp_path, 1),
    )
    wait_for(lambda: holder.pid in slots.holders(tmp_path / "slots", 1))
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--timeout",
            "0.3",
            "--",
            sys.executable,
            "-c",
            "pass",
        ],
        env=env(tmp_path, 1),
        capture_output=True,
        text=True,
    )
    holder.wait(timeout=30)
    assert (
        result.returncode != 0
        and "timed out waiting for a slot" in result.stderr
    )
    assert "pid" in result.stderr


def test_a_nested_job_inside_a_held_slot_does_not_take_another(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.setenv("SERVAL_SLOTS", "1")
    monkeypatch.setenv("SERVAL_SLOT_HELD", "1")
    with slots.slot(slots.Kind.BUILD, timeout=0.2):
        pass


def test_a_crashed_holder_frees_its_slot(tmp_path):
    holder = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--",
            sys.executable,
            "-c",
            "import time; time.sleep(60)",
        ],
        env=env(tmp_path, 1),
    )
    time.sleep(0.5)
    holder.kill()
    holder.wait()
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--timeout",
            "10",
            "--",
            sys.executable,
            "-c",
            "pass",
        ],
        env=env(tmp_path, 1),
    )
    assert result.returncode == 0


def test_zero_slots_is_refused(monkeypatch):
    monkeypatch.setenv("SERVAL_SLOTS", "0")
    with pytest.raises(ValueError):
        slots.slot_count()


def test_a_back_to_back_exclusive_chain_does_not_starve_a_waiting_normal_job(
    tmp_path,
):
    first = start(tmp_path, 3, "A", "--exclusive", hold=3)
    wait_for(lambda: first.pid in slots.holders(tmp_path / "slots", 3))
    normal = start(tmp_path, 3, "N")
    wait_for(lambda: normal.pid in queued_pids(tmp_path))
    second = start(tmp_path, 3, "B", "--exclusive")
    for job in (first, normal, second):
        assert job.wait(timeout=60) == 0
    rows = {name: (s, e) for name, s, e in records(tmp_path)}
    assert rows["N"][0] >= rows["A"][1] - 0.05, (
        "N ran inside the first exclusive job"
    )
    assert rows["N"][0] < rows["B"][0], (
        "the exclusive job queued later jumped the queue"
    )
    assert rows["B"][0] >= rows["N"][1] - 0.05, "B overlapped the normal job"


def test_a_dead_waiter_is_dropped_from_the_queue(tmp_path, monkeypatch):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.setenv("SERVAL_SLOTS", "1")
    (tmp_path / "slots").mkdir()
    gone = subprocess.Popen([sys.executable, "-c", "pass"])
    gone.wait()
    (tmp_path / "slots" / "queue").write_text(f"{gone.pid}\n")
    lease = slots.acquire(timeout=5)
    lease.release()


def test_a_nested_slot_in_the_holding_process_takes_no_second_slot(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.setenv("SERVAL_SLOTS", "1")
    monkeypatch.delenv("SERVAL_SLOT_HELD", raising=False)
    with slots.slot(slots.Kind.BUILD, timeout=5):
        with slots.slot(slots.Kind.BUILD, timeout=0.3):
            pass


def test_an_exclusive_slot_inside_a_held_slot_fails_at_once(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.delenv("SERVAL_SLOT_HELD", raising=False)
    with slots.slot(slots.Kind.BUILD, timeout=5):
        with pytest.raises(slots.NestedExclusive):
            with slots.slot(slots.Kind.SWEEP, exclusive=True, timeout=5):
                pass


BUILD_HOLDER = (
    "import pathlib, sys, time\n"
    "import pending_baselines as pending, slots\n"
    "ready, go = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])\n"
    "with slots.slot(slots.Kind.BUILD, label='bench_compare build'):\n"
    "    ready.touch()\n"
    "    while not go.exists():\n"
    "        time.sleep(0.05)\n"
    "    pending.run(['make', '--version'])\n"
)


def queued_pids(tmp_path) -> list[int]:
    path = tmp_path / "slots" / "queue"
    return [int(x) for x in path.read_text().split()] if path.exists() else []


def wait_for(condition, seconds=10.0):
    deadline = time.monotonic() + seconds
    while not condition():
        assert time.monotonic() < deadline, "condition never held"
        time.sleep(0.05)


def test_a_build_holder_and_an_exclusive_queue_head_do_not_deadlock(
    tmp_path,
):
    ready, go = tmp_path / "ready", tmp_path / "go"
    holder = subprocess.Popen(
        [sys.executable, "-c", BUILD_HOLDER, str(ready), str(go)],
        env={**env(tmp_path, 3), "PYTHONPATH": str(SCRIPT.parent)},
    )
    exclusive = None
    try:
        wait_for(ready.exists)
        exclusive = start(tmp_path, 3, "x", "--exclusive")
        wait_for(lambda: exclusive.pid in queued_pids(tmp_path))
        go.touch()
        assert holder.wait(timeout=30) == 0
        assert exclusive.wait(timeout=30) == 0
    finally:
        for process in (holder, exclusive):
            if process is not None and process.poll() is None:
                process.kill()


def wait_until_held(tmp_path, pid):
    board = lambda: slots.read_board(  # noqa: E731
        tmp_path / "slots", slots.slot_count(), slots.Queue(tmp_path / "slots")
    )
    wait_for(lambda: pid in board().holders)


def test_a_holder_waiting_in_the_queue_is_reported_as_a_stall(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.setenv("SERVAL_SLOTS", "1")
    monkeypatch.delenv("SERVAL_SLOT_HELD", raising=False)
    monkeypatch.setattr(slots, "STALL_SECONDS", 0.5)
    (tmp_path / "slots").mkdir()
    holder = start(tmp_path, 1, "h", hold=20)
    try:
        wait_until_held(tmp_path, holder.pid)
        (tmp_path / "slots" / "queue").write_text(f"{holder.pid}\n")
        with pytest.raises(slots.SlotStall, match=f"pid {holder.pid} HOLDS"):
            slots.acquire(exclusive=True, timeout=20)
    finally:
        holder.kill()
        holder.wait()


def test_a_head_that_could_start_but_does_not_is_reported_as_a_stall(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SERVAL_SLOTS_DIR", str(tmp_path / "slots"))
    monkeypatch.setenv("SERVAL_SLOTS", "1")
    monkeypatch.delenv("SERVAL_SLOT_HELD", raising=False)
    monkeypatch.setattr(slots, "STALL_SECONDS", 0.5)
    stuck = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"]
    )
    try:
        directory = tmp_path / "slots"
        (directory / "waiting").mkdir(parents=True)
        request = dict(exclusive=False, label="stuck", since=0)
        (directory / "waiting" / f"{stuck.pid}.json").write_text(
            json.dumps(request)
        )
        (directory / "queue").write_text(f"{stuck.pid}\n")
        with pytest.raises(slots.SlotStall, match="does not move"):
            slots.acquire(timeout=20)
    finally:
        stuck.kill()
        stuck.wait()


def test_a_child_that_lost_the_held_marker_fails_instead_of_queueing(tmp_path):
    inner = (
        "import os, subprocess, sys\n"
        "env = {k: v for k, v in os.environ.items() if k != 'SERVAL_SLOT_HELD'}\n"
        f"cmd = [sys.executable, {str(SCRIPT)!r}, 'run', '--kind', 'build',\n"
        "       '--timeout', '5', '--', sys.executable, '-c', 'pass']\n"
        "done = subprocess.run(cmd, env=env, capture_output=True, text=True)\n"
        "sys.stderr.write(done.stderr)\n"
        "sys.exit(done.returncode)\n"
    )
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            "--kind",
            "build",
            "--",
            sys.executable,
            "-c",
            inner,
        ],
        env=env(tmp_path, 3),
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode != 0
    assert "holds a slot" in result.stderr


def test_status_shows_the_wait_graph(tmp_path):
    holder = start(tmp_path, 3, "h", "--exclusive", hold=3)
    try:
        wait_for(
            lambda: (
                "0 of 3 slots free"
                in subprocess.run(
                    [sys.executable, str(SCRIPT), "status"],
                    env=env(tmp_path, 3),
                    capture_output=True,
                    text=True,
                ).stdout
            )
        )
        shown = subprocess.run(
            [sys.executable, str(SCRIPT), "status"],
            env=env(tmp_path, 3),
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        assert f"holds 3 slot(s): pid {holder.pid}" in shown
    finally:
        holder.kill()
        holder.wait()


def test_the_engine_sample_takes_its_slot_before_the_scratch_lock(
    monkeypatch, tmp_path
):
    import pending_baselines as pending

    order = []

    @contextlib.contextmanager
    def record(name):
        order.append(name)
        yield

    monkeypatch.setattr(pending, "CACHE", tmp_path)
    monkeypatch.setattr(pending, "ensure_commit", lambda sha: None)
    monkeypatch.setattr(slots, "slot", lambda *_, **__: record("slot"))
    monkeypatch.setattr(pending, "commit_lock", lambda name: record("lock"))
    monkeypatch.setattr(
        pending, "sample_commit_locked", lambda sha, out: order.append("work")
    )
    pending.sample_commit("0123456789abcdef")
    assert order == ["slot", "lock", "work"]
