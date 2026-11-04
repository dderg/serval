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


def env(tmp_path, count):
    return {
        **os.environ,
        "SERVAL_SLOTS_DIR": str(tmp_path / "slots"),
        "SERVAL_SLOTS": str(count),
        "SERVAL_SLOT_HELD": "",
    }


def start(tmp_path, count, name, *flags):
    return subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run",
            *flags,
            "--",
            sys.executable,
            "-c",
            JOB,
            str(tmp_path / "log"),
            name,
            str(HOLD_SECONDS),
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
            "--",
            sys.executable,
            "-c",
            "import time; time.sleep(6)",
        ],
        env=env(tmp_path, 1),
    )
    time.sleep(0.5)
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run",
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
    with slots.slot(timeout=0.2):
        pass


def test_a_crashed_holder_frees_its_slot(tmp_path):
    holder = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run",
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
