from __future__ import annotations

import contextlib
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import suite


def _status(pulsed_ticks, both_edge_ticks=160, simulated=False):
    return {
        "step_benchmark": {
            "pulsed_ticks_per_step": pulsed_ticks,
            "both_edge_ticks_per_step": both_edge_ticks,
            "simulated": simulated,
        },
        "mcu_constants": {"CLOCK_FREQ": 84_000_000},
    }


def test_the_f401_benchmark_lands_inside_the_single_motor_band():
    verdict = suite.step_benchmark_verdict(_status(497))
    assert verdict["pulsed_capacity_steps_per_s"] == pytest.approx(
        84_000_000 / 497
    )
    assert verdict["ratio_to_single_motor_ceiling"] == pytest.approx(
        84_000_000 / 497 / suite.SINGLE_MOTOR_CEILING_STEPS_PER_S
    )
    assert verdict["within_band"]


@pytest.mark.parametrize("pulsed_ticks", [301, 620])
def test_a_benchmark_far_from_the_single_motor_ceiling_is_out_of_band(
    pulsed_ticks,
):
    assert not suite.step_benchmark_verdict(_status(pulsed_ticks))[
        "within_band"
    ]


def test_a_simulated_benchmark_on_the_bench_is_fatal():
    with pytest.raises(suite.SuiteError, match="simulated"):
        suite.step_benchmark_verdict(_status(0, 0, simulated=True))


def test_a_missing_benchmark_is_fatal():
    with pytest.raises(suite.SuiteError, match="step_benchmark"):
        suite.step_benchmark_verdict({"mcu_constants": {"CLOCK_FREQ": 1}})


def _completed(returncode, stdout="", stderr=""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def test_the_engine_builds_with_make_in_the_suites_own_tree(monkeypatch):
    calls = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: calls.append((cmd, kw)) or _completed(0),
    )
    suite.build_local_engine()
    [(cmd, kw)] = calls
    assert cmd == ["make", "-f", "Makefile.rust", "motion-engine-fast"]
    assert kw["cwd"] == suite.REPO


def test_a_failed_engine_build_is_fatal_with_its_output(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: _completed(2, "cargo build", "error[E0425]: nope"),
    )
    with pytest.raises(
        suite.SuiteError, match=r"(?s)exited 2.*cargo build.*E0425\]: nope"
    ):
        suite.build_local_engine()


def _run_suite(monkeypatch, tmp_path, argv, foreign=lambda _row: None):
    order = []
    monkeypatch.setattr(suite, "SUITES_DIR", tmp_path)
    monkeypatch.setattr(sys, "argv", ["suite.py", *argv])
    monkeypatch.setattr(
        suite, "build_local_engine", lambda: order.append("build")
    )
    monkeypatch.setattr(
        suite.lock, "bench_lock", lambda *a, **k: contextlib.nullcontext()
    )
    monkeypatch.setattr(suite.pi, "read_deployed", lambda: {"head": "abc"})
    monkeypatch.setattr(
        suite.pi, "foreign_deploy", lambda _synced: foreign(len(order))
    )
    monkeypatch.setattr(
        suite,
        "python",
        lambda script, *a, **k: order.append(script) or _completed(0),
    )
    monkeypatch.setattr(
        suite,
        "query_step_benchmark",
        lambda: suite.step_benchmark_verdict(_status(301)),
    )

    def run_row(row, suite_dir, lock_timeout_s):
        order.append(row.name)
        return {"name": row.name, "passed": True}

    monkeypatch.setattr(suite, "run_row", run_row)
    monkeypatch.setattr(suite, "print_table", lambda rows: None)
    with pytest.raises(SystemExit) as exited:
        suite.main()
    order.append(exited.value.code)
    return order


def test_the_suite_builds_the_engine_once_before_any_row(monkeypatch, tmp_path):
    order = _run_suite(monkeypatch, tmp_path, [])
    assert order[0] == "build"
    assert order.count("build") == 1
    assert order[1:-1] == ["deploy.py", *(row.name for row in suite.ROWS)]


def test_a_bench_another_session_redeployed_aborts_the_suite_loudly(
    monkeypatch, tmp_path
):
    taken = "the Pi runs /wt/b: another session took the bench"
    rows_before_theft = 3
    order = _run_suite(
        monkeypatch,
        tmp_path,
        ["--group", "session"],
        lambda done: taken if done > rows_before_theft else None,
    )
    assert order[1:-1] == [
        "deploy.py",
        *(scenario.name for scenario in suite.SCENARIOS[:2]),
    ]
    assert order[-1] == f"suite aborted, its rows are void: {taken}"
    [summary] = tmp_path.glob("*/summary.json")
    assert json.loads(summary.read_text())["aborted"] == taken


def test_bench_tools_started_by_the_suite_inherit_the_lock(monkeypatch):
    spawned = []
    monkeypatch.setattr(suite.lock, "inherited_fds", lambda: (7,))
    monkeypatch.setattr(
        subprocess, "run", lambda cmd, **kw: spawned.append(kw) or None
    )
    suite.python("run.py", timeout_s=1.0)
    assert spawned[0]["pass_fds"] == (7,)


def test_the_full_suite_holds_both_groups():
    assert suite.selected_rows(suite.Scope.ALL) == [
        *suite.MATRIX,
        *suite.SESSION_ROWS,
    ]


def test_the_session_group_runs_every_scenario_and_nothing_else(
    monkeypatch, tmp_path
):
    order = _run_suite(monkeypatch, tmp_path, ["--group", "session"])
    names = [scenario.name for scenario in suite.SCENARIOS]
    assert order[1:-1] == ["deploy.py", *names]


def test_quick_stays_the_quick_matrix_rows():
    quick = suite.selected_rows(suite.Scope.QUICK)
    assert [row.name for row in quick] == [
        "first_layers",
        "speed_ladder",
        "epochs",
        "single_motor_cap",
    ]


def _session_runner(checks):
    calls = []

    def python(script, *args, timeout_s):
        calls.append(script)
        if script == "session.py":
            runs_dir = Path(args[args.index("--runs-dir") + 1])
            run_dir = runs_dir / "20261116-000000"
            run_dir.mkdir(parents=True)
            (run_dir / "run.json").write_text(
                json.dumps({"print_state": "complete", "duration_s": 90.0})
            )
        if script == "session_check.py" and checks is not None:
            verdict = {
                "ok": all(check["ok"] for check in checks),
                "guards": "#843",
                "not_analyzed": "several prints",
                "checks": checks,
            }
            (Path(args[0]) / suite.session_check.CHECKS_FILE).write_text(
                json.dumps(verdict)
            )
        return _completed(0 if script != "session_check.py" else 1)

    return python, calls


def test_a_session_row_fails_with_each_failed_check_and_its_reason(
    monkeypatch, tmp_path
):
    checks = [
        {"name": "journal", "ok": True, "detail": "no klippy abort"},
        {"name": "gaps", "ok": False, "detail": "9.71 s > 1 s (#843)"},
    ]
    python, calls = _session_runner(checks)
    monkeypatch.setattr(suite, "python", python)
    row = suite.SessionRow(suite.SCENARIOS[2])
    result = suite.run_row(row, tmp_path, 1.0)
    assert calls == ["session.py", "session_check.py"]
    assert result["error"] is None
    assert not result["passed"]
    assert result["failed_checks"] == ["gaps: 9.71 s > 1 s (#843)"]
    assert result["analyzer"] == "not run: several prints"


def test_a_session_row_whose_checks_pass_passes(monkeypatch, tmp_path):
    checks = [{"name": "gaps", "ok": True, "detail": "0.2 s <= 1 s"}]
    python, _ = _session_runner(checks)
    monkeypatch.setattr(suite, "python", python)
    result = suite.run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path, 1.0)
    assert result["passed"]
    assert result["failed_checks"] == []


def test_a_session_row_without_a_verdict_is_an_error(monkeypatch, tmp_path):
    python, _ = _session_runner(None)
    monkeypatch.setattr(suite, "python", python)
    result = suite.run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path, 1.0)
    assert not result["passed"]
    assert result["error"].startswith("session_check.py")
