from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import ab
import lock
import pytest

SYNCED = "synced: 3 files changed, 0 stale removed"
RESTARTED = "klipper restarted: pid 1 -> 2"
HOLD_LEFT_S = 1500.0


def ab_args(**fields) -> argparse.Namespace:
    return argparse.Namespace(
        **{
            "ref": ["A", "B"],
            "gcode": Path("g.gcode"),
            "config": Path("c.cfg"),
            "runs": 2,
            "host_cpus": None,
            "host_max_mhz": None,
            "lock_timeout": 60.0,
            **fields,
        }
    )


def flag(command: list[str], name: str) -> str:
    return command[command.index(name) + 1]


class FakeBench:
    def __init__(self, exits: dict[int, int]):
        self.exits = exits
        self.events: list[tuple[str, ...]] = []
        self.holding = False

    def hold(self, _lock_timeout_s: float, command: list[str]) -> int:
        self.holding = True
        self.events.append(("hold", flag(command, "--ref")))
        number = len(self.events)
        run_dir = Path(flag(command, "--runs-dir")) / f"run-{number}"
        run_dir.mkdir(parents=True)
        Path(flag(command, "--result")).write_text(
            json.dumps({"started": time.time(), "sync": SYNCED, "error": None})
        )
        self.holding = False
        return self.exits.get(number, 0)

    def analyze(self, run_dir: Path, tree: Path) -> str | None:
        assert not self.holding
        self.events.append(("analyze", tree.name, run_dir.name))
        return None


@pytest.fixture
def bench(monkeypatch):
    def install(exits: dict[int, int]) -> FakeBench:
        fake = FakeBench(exits)
        monkeypatch.setattr(ab, "bench_hold", fake.hold)
        monkeypatch.setattr(ab, "analyze", fake.analyze)
        monkeypatch.setattr(ab.summarize, "summarize", lambda _run: None)
        return fake

    return install


def trees(tmp_path: Path) -> dict[str, Path]:
    return {"A": tmp_path / "tree-A", "B": tmp_path / "tree-B"}


def test_ab_takes_one_hold_per_run_ref_by_ref_and_analyzes_after_them(
    bench, tmp_path
):
    fake = bench({})
    by_ref = ab.measure_refs(ab_args(), trees(tmp_path), tmp_path / "ab")
    assert fake.events == [
        ("hold", "A"),
        ("hold", "A"),
        ("hold", "B"),
        ("hold", "B"),
        ("analyze", "tree-A", "run-1"),
        ("analyze", "tree-A", "run-2"),
        ("analyze", "tree-B", "run-3"),
        ("analyze", "tree-B", "run-4"),
    ]
    first = by_ref["A"][0]
    assert first["error"] is None
    assert first["hold"]["sync"] == SYNCED
    assert first["hold"]["wait_s"] >= 0.0
    assert first["hold"]["held_s"] >= 0.0


def test_a_run_whose_hold_hit_the_cap_fails_and_the_next_run_goes_on(
    bench, tmp_path
):
    fake = bench({1: lock.HOLD_EXPIRED_EXIT})
    by_ref = ab.measure_refs(ab_args(), trees(tmp_path), tmp_path / "ab")
    assert [event for event in fake.events if event[0] == "hold"] == [
        ("hold", "A"),
        ("hold", "A"),
        ("hold", "B"),
        ("hold", "B"),
    ]
    assert "MAX_HOLD_S" in by_ref["A"][0]["error"]
    assert by_ref["A"][1]["error"] is None


def test_a_hold_failing_otherwise_stops_the_ab(bench, tmp_path):
    bench({1: 1})
    with pytest.raises(SystemExit, match="the hold failed"):
        ab.measure_refs(ab_args(), trees(tmp_path), tmp_path / "ab")


def test_ab_refuses_to_run_inside_a_hold(monkeypatch):
    monkeypatch.setenv(lock.TOKEN_ENV, "1")
    monkeypatch.setattr(
        sys, "argv", ["ab.py", "--ref", "A", "--gcode", "g.gcode"]
    )
    with pytest.raises(SystemExit, match="outside lock.py hold"):
        ab.main()


class FakeTools:
    def __init__(self, run_exit: int):
        self.run_exit = run_exit
        self.calls: list[tuple[str, float]] = []

    def python(
        self, script: str, *args: str, timeout_s: float
    ) -> subprocess.CompletedProcess:
        self.calls.append((" ".join([script, *args[:1]]), timeout_s))
        if script == "run.py":
            return subprocess.CompletedProcess([], self.run_exit, "", "boom")
        out = f"{SYNCED}\nbuild noise\n{RESTARTED}\n"
        return subprocess.CompletedProcess([], 0, out, "")


@pytest.fixture
def tools(monkeypatch):
    def install(run_exit: int) -> FakeTools:
        fake = FakeTools(run_exit)
        monkeypatch.setattr(ab, "python", fake.python)
        monkeypatch.setattr(ab.lock, "remaining_s", lambda: HOLD_LEFT_S)
        return fake

    return install


def held_args(tmp_path: Path) -> argparse.Namespace:
    return ab_args(
        ref=["A"],
        tree=tmp_path / "tree-A",
        runs_dir=tmp_path / "runs",
        result=tmp_path / "hold.json",
    )


def test_a_held_run_syncs_then_runs_with_the_hold_time_left(tools, tmp_path):
    fake = tools(0)
    args = held_args(tmp_path)
    ab.held_run(args)
    budget_s = HOLD_LEFT_S - ab.RECOVERY_TIMEOUT_S
    assert fake.calls == [
        ("deploy.py sync", lock.MAX_HOLD_S),
        ("run.py g.gcode", budget_s),
    ]
    held = json.loads(args.result.read_text())
    assert held["error"] is None
    assert held["sync"] == f"{SYNCED}; {RESTARTED}"


def test_a_failed_run_resets_the_target_inside_its_hold(tools, tmp_path):
    fake = tools(1)
    args = held_args(tmp_path)
    ab.held_run(args)
    assert [call for call, _ in fake.calls] == [
        "deploy.py sync",
        "run.py g.gcode",
        "deploy.py reset-target",
    ]
    assert json.loads(args.result.read_text())["error"] == "run.py: boom"


def test_the_dry_run_lists_one_hold_per_run(monkeypatch, capsys):
    monkeypatch.setattr(ab, "resolve", lambda ref: f"{ref}{'0' * 40}")
    ab.print_plan(ab_args())
    holds = [
        line.strip()
        for line in capsys.readouterr().out.splitlines()
        if "deploy.py sync" in line
    ]
    assert [line.split(":")[0] for line in holds] == [
        "1. A run 1/2",
        "2. A run 2/2",
        "3. B run 1/2",
        "4. B run 2/2",
    ]
