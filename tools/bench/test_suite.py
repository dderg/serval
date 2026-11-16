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


def test_the_engine_builds_with_make_in_the_tree_it_analyzes(
    monkeypatch, tmp_path
):
    calls = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: calls.append((cmd, kw)) or _completed(0),
    )
    suite.build_local_engine(tmp_path)
    [(cmd, kw)] = calls
    assert cmd == ["make", "-f", "Makefile.rust", "motion-engine-fast"]
    assert kw["cwd"] == tmp_path


def test_a_failed_engine_build_is_fatal_with_its_output(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: _completed(2, "cargo build", "error[E0425]: nope"),
    )
    with pytest.raises(
        suite.SuiteError, match=r"(?s)exited 2.*cargo build.*E0425\]: nope"
    ):
        suite.build_local_engine(suite.REPO)


def _hold_result(index, rows, **extra):
    return {
        "index": index,
        "asked": 100.0,
        "acquired": 103.0,
        "released": 160.0,
        "ended": 161.0,
        "exit": 0,
        "deployed": {"head": "abc"},
        "firmware": {"target": "v1-fwf00d", "capture": "cap"},
        "restored": [],
        "settled": [],
        "step_benchmark": suite.step_benchmark_verdict(_status(497)),
        "rows": [
            {"name": name, "run_dir": None, "error": None, "held_s": 10.0}
            for name in rows
        ],
        "running": None,
        "deferred": [],
        "needs_target_build": False,
        "aborted": None,
        **extra,
    }


class Bench(list):
    analyzer: Path | None = None
    target_built: list[bool]


def _run_suite(monkeypatch, tmp_path, argv, hold=_hold_result):
    events = Bench()
    events.target_built = []
    monkeypatch.setattr(suite, "SUITES_DIR", tmp_path)
    monkeypatch.setattr(sys, "argv", ["suite.py", *argv])
    monkeypatch.delenv(suite.lock.TOKEN_ENV, raising=False)
    monkeypatch.setattr(
        suite, "build_local_engine", lambda tree: events.append("build")
    )
    monkeypatch.setattr(
        suite.soak_gcode, "ensure_all", lambda: events.append("soak gcode")
    )
    monkeypatch.setattr(
        suite,
        "python",
        lambda script, *a, **k: events.append(script) or _completed(0),
    )

    def take_hold(self, rows):
        events.append(list(rows))
        events.target_built.append(self.target_built)
        return hold(len(self.holds) + 1, rows)

    def judge_row(row, result, analyzer):
        events.analyzer = analyzer
        result.update(passed=result["error"] is None, wall_s=0.0)
        return result

    monkeypatch.setattr(suite.SuiteRun, "take_hold", take_hold)
    monkeypatch.setattr(suite, "judge_row", judge_row)
    for quiet in ("print_table", "print_holds", "print_benchmark"):
        monkeypatch.setattr(suite, quiet, lambda *a: None)
    with pytest.raises(SystemExit) as exited:
        suite.main()
    events.append(exited.value.code)
    summaries = list(tmp_path.glob("*/summary.json"))
    summary = json.loads(summaries[0].read_text()) if summaries else None
    return events, summary


def _holds(events) -> list[list[str]]:
    return [event for event in events if isinstance(event, list)]


def test_the_full_suite_runs_its_planned_holds_after_one_engine_build(
    monkeypatch, tmp_path
):
    events, summary = _run_suite(monkeypatch, tmp_path, [])
    names = [row.name for row in suite.ROWS]
    assert events[0] == "build"
    assert events.count("build") == 1
    assert _holds(events) == suite.hold_plan.plan(names)
    assert len(_holds(events)) > 1
    assert [row["name"] for row in summary["rows"]] == names
    assert summary["plan"] == _holds(events)
    first = summary["holds"][0]
    assert (first["wait_s"], first["held_s"], first["exit"]) == (3.0, 57.0, 0)
    assert first["rows"] == _holds(events)[0]
    assert events[-1] == 0


def test_quick_runs_in_one_hold(monkeypatch, tmp_path):
    events, summary = _run_suite(monkeypatch, tmp_path, ["--quick"])
    assert _holds(events) == [
        [row.name for row in suite.selected_rows(suite.Scope.QUICK)]
    ]
    assert len(summary["holds"]) == 1


def test_rows_a_hold_has_no_time_left_for_run_in_the_next_hold(
    monkeypatch, tmp_path
):
    def hold(index, rows):
        return _hold_result(index, rows[:1], deferred=rows[1:])

    events, summary = _run_suite(monkeypatch, tmp_path, ["--quick"], hold)
    quick = [row.name for row in suite.selected_rows(suite.Scope.QUICK)]
    assert _holds(events) == [quick[n:] for n in range(len(quick))]
    assert summary["holds"][0]["deferred"] == quick[1:]
    assert [row["name"] for row in summary["rows"]] == quick


def test_a_hold_killed_at_the_cap_fails_its_row_and_the_rest_move_on(
    monkeypatch, tmp_path
):
    quick = [row.name for row in suite.selected_rows(suite.Scope.QUICK)]

    def hold(index, rows):
        if index > 1:
            return _hold_result(index, rows)
        killed = {"exit": suite.lock.HOLD_EXPIRED_EXIT, "released": None}
        return _hold_result(
            index, rows[:1], running=rows[1], running_since=150.0, **killed
        )

    events, summary = _run_suite(monkeypatch, tmp_path, ["--quick"], hold)
    assert _holds(events) == [quick, quick[2:]]
    killed = summary["rows"][1]
    assert killed["name"] == quick[1]
    assert not killed["passed"]
    assert "MAX_HOLD_S = 1800 s" in killed["error"]
    assert killed["held_s"] == 11.0
    assert summary["holds"][0]["held_s"] == 58.0
    assert summary["holds"][0]["exit"] == suite.lock.HOLD_EXPIRED_EXIT
    assert events[-1] == 1


def test_a_hold_that_aborts_stops_the_suite_loudly(monkeypatch, tmp_path):
    taken = "the Pi runs /wt/b: another session took the bench"

    def hold(index, rows):
        return _hold_result(index, rows[:1], aborted=taken, exit=1)

    events, summary = _run_suite(
        monkeypatch, tmp_path, ["--group", "session"], hold
    )
    assert len(_holds(events)) == 1
    assert events[-1] == f"suite aborted, its rows are void: {taken}"
    assert summary["aborted"] == taken


def test_holds_that_never_run_a_row_stop_the_suite(monkeypatch, tmp_path):
    def hold(index, rows):
        return _hold_result(index, [], deferred=rows)

    events, summary = _run_suite(monkeypatch, tmp_path, ["--quick"], hold)
    assert len(_holds(events)) == suite.STALLED_HOLDS
    assert "ran no row" in summary["aborted"]


def test_a_reflash_builds_the_target_between_holds_then_retries(
    monkeypatch, tmp_path
):
    def hold(index, rows):
        if index == 1:
            return _hold_result(index, rows[:1], deferred=rows[1:])
        if index == 2:
            restore = {"needs_target_build": True, "deferred": rows}
            return _hold_result(index, [], **restore)
        return _hold_result(index, rows)

    events, summary = _run_suite(monkeypatch, tmp_path, ["--quick"], hold)
    quick = [row.name for row in suite.selected_rows(suite.Scope.QUICK)]
    assert _holds(events) == [quick, quick[1:], quick[1:]]
    assert events.index("deploy.py") > events.index(quick[1:])
    assert events.target_built == [False, False, True]
    assert [row["name"] for row in summary["rows"]] == quick


def test_the_suite_refuses_to_run_inside_a_hold(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "argv", ["suite.py", "--quick"])
    monkeypatch.setenv(suite.lock.TOKEN_ENV, "1234")
    with pytest.raises(SystemExit, match="once per hold"):
        suite.main()


def test_bench_tools_started_by_the_suite_inherit_the_lock(monkeypatch):
    spawned = []
    monkeypatch.setattr(suite.lock, "inherited_fds", lambda: (7,))
    monkeypatch.setattr(
        subprocess, "run", lambda cmd, **kw: spawned.append(kw) or None
    )
    suite.python("run.py", timeout_s=1.0)
    assert spawned[0]["pass_fds"] == (7,)


def test_the_full_suite_holds_both_groups_and_no_soak_row():
    full = suite.selected_rows(suite.Scope.ALL)
    assert full == [*suite.MATRIX, *suite.SESSION_ROWS]
    assert not any(row in suite.SOAK_ROWS for row in full)


def test_the_soak_group_generates_its_prints_then_holds_each_row_alone(
    monkeypatch, tmp_path
):
    events, _ = _run_suite(monkeypatch, tmp_path, ["--group", "soak"])
    soak = [row.name for row in suite.SOAK_ROWS]
    assert soak == ["soak_mesh_session", "soak_throttled"]
    assert events[:-1] == ["soak gcode", "build", *([name] for name in soak)]


def test_named_rows_of_any_group_run_alone(monkeypatch, tmp_path):
    events, _ = _run_suite(
        monkeypatch, tmp_path, ["--row", "soak_throttled", "--row", "epochs"]
    )
    assert events[:-1] == ["soak gcode", "build", ["soak_throttled", "epochs"]]


def test_a_tree_is_analyzed_by_its_own_analyzer(monkeypatch, tmp_path):
    tree = tmp_path / "candidate"
    events, _ = _run_suite(
        monkeypatch, tmp_path, ["--row", "epochs", "--tree", str(tree)]
    )
    assert events.analyzer == tree / "tools" / "bench"


def test_a_soak_row_expected_to_fail_is_not_in_the_full_suite(
    monkeypatch, tmp_path
):
    events, summary = _run_suite(
        monkeypatch, tmp_path, ["--expect-fail", "soak_throttled"]
    )
    assert events == [2]
    assert summary is None


def test_a_throttled_soak_print_also_needs_its_run_wide_checks():
    [row] = suite.SOAK_MATRIX
    analyzed = {"error": None, "print_state": "complete", "analyzer_ok": True}
    assert not suite.row_passed(row, {**analyzed, "run_wide_ok": False})
    assert suite.row_passed(row, {**analyzed, "run_wide_ok": True})
    assert suite.row_passed(suite.MATRIX[0], analyzed)


def test_soak_rows_outlast_the_default_runner_timeout():
    [session_row] = suite.SOAK_ROWS[:1]
    session_s = suite.soak_gcode.SESSION_PRINT.planned_print_s
    long_s = suite.soak_gcode.LONG_PRINT.planned_print_s
    assert session_row.timeout_s > 2 * session_s
    assert suite.SOAK_MATRIX[0].timeout_s > long_s
    assert suite.SESSION_ROWS[0].timeout_s == suite.RUN_TIMEOUT_S


def test_quick_stays_the_quick_matrix_rows():
    quick = suite.selected_rows(suite.Scope.QUICK)
    assert [row.name for row in quick] == [
        "first_layers",
        "speed_ladder",
        "epochs",
        "single_motor_cap",
    ]


RAN_ON = {"target": "v1-fwf00d", "capture": "cap"}
LATER_HOLD = {
    "index": 2,
    "rows": ["epochs", "speed_ladder"],
    "tree": "/wt/a",
    "suite_dir": "/s",
    "lock_timeout": 1.0,
    "deployed": {"head": "abc", "timestamp": "first sync"},
    "firmware": RAN_ON,
    "target_built": False,
    "result": "/s/holds/2.json",
}
FIRST_HOLD = {**LATER_HOLD, "index": 1, "deployed": None, "firmware": None}


def _hold_start(monkeypatch, foreign=None, found=RAN_ON, tree_build="f00d"):
    calls = []
    monkeypatch.setattr(suite.pi, "foreign_deploy", lambda deployed: foreign)
    monkeypatch.setattr(
        suite.pi,
        "read_deployed",
        lambda: {"head": "abc", "timestamp": "this sync"},
    )
    monkeypatch.setattr(suite, "board_firmware", lambda: found)
    monkeypatch.setattr(
        suite,
        "python",
        lambda script, *a, **k: calls.append(f"{script} {a[0]}")
        or _completed(0),
    )
    monkeypatch.setattr(
        suite.deploy,
        "settle",
        lambda config: calls.append(f"settle {config.name}")
        or ["FIRMWARE_RESTART: the Pi ran another printer config"],
    )
    monkeypatch.setattr(suite, "query_step_benchmark", lambda: {"ok": True})
    monkeypatch.setattr(
        suite, "run_rows", lambda plan, result, save: calls.append("rows")
    )
    monkeypatch.setattr(
        suite.firmware_build_id,
        "target_build_id",
        lambda tree, config: tree_build,
    )
    return calls


def _start_hold(plan) -> dict:
    result = {"restored": [], "rows": [], "deferred": []}
    suite.held(plan, result, lambda: None)
    return result


def test_the_first_hold_syncs_and_records_the_boards_firmware(monkeypatch):
    calls = _hold_start(monkeypatch)
    result = _start_hold(FIRST_HOLD)
    assert calls == ["deploy.py sync", "settle printer.cfg", "rows"]
    assert result["deployed"] == {"head": "abc", "timestamp": "this sync"}
    assert result["firmware"] == RAN_ON
    assert result["restored"] == []


def test_a_later_hold_on_an_untouched_bench_only_settles(monkeypatch):
    calls = _hold_start(monkeypatch)
    result = _start_hold(LATER_HOLD)
    assert calls == ["settle printer.cfg", "rows"]
    assert result["deployed"] == LATER_HOLD["deployed"]
    assert result["restored"] == []
    assert result["settled"] == [
        "FIRMWARE_RESTART: the Pi ran another printer config"
    ]


def test_a_tree_another_holder_synced_is_restored_and_recorded(monkeypatch):
    taken = "the Pi runs /wt/b head 123456789, not /wt/a: another session"
    calls = _hold_start(monkeypatch, foreign=taken)
    result = _start_hold(LATER_HOLD)
    assert calls == ["deploy.py sync", "settle printer.cfg", "rows"]
    assert result["restored"] == [f"re-synced /wt/a: {taken}"]
    assert result["deployed"]["timestamp"] == "this sync"


def test_a_reflashed_f401_waits_for_a_build_outside_the_hold(monkeypatch):
    found = {**RAN_ON, "target": "v1-fwbeef"}
    calls = _hold_start(monkeypatch, found=found)
    result = _start_hold(LATER_HOLD)
    assert calls == []
    assert result["needs_target_build"]
    assert result["deferred"] == LATER_HOLD["rows"]


def test_a_reflashed_f401_is_flashed_back_once_built(monkeypatch):
    found = {**RAN_ON, "target": "v1-fwbeef"}
    calls = _hold_start(monkeypatch, found=found)
    result = _start_hold({**LATER_HOLD, "target_built": True})
    assert calls == ["deploy.py ensure-target", "settle printer.cfg", "rows"]
    assert result["restored"] == [
        "F401 reflashed from 'v1-fwbeef' to 'v1-fwf00d'"
    ]


def test_firmware_this_tree_cannot_rebuild_aborts_the_hold(monkeypatch):
    found = {**RAN_ON, "target": "v1-fwbeef"}
    _hold_start(monkeypatch, found=found, tree_build="cafe")
    with pytest.raises(suite.SuiteError, match="cannot be restored"):
        _start_hold(LATER_HOLD)


def test_a_reflashed_capture_board_aborts_the_hold(monkeypatch):
    _hold_start(monkeypatch, found={**RAN_ON, "capture": "other"})
    with pytest.raises(suite.SuiteError, match="reflashed the instrument"):
        _start_hold(LATER_HOLD)


def test_a_row_that_no_longer_fits_the_hold_is_deferred(monkeypatch):
    remaining = iter([2000.0, 500.0])
    ran = []
    monkeypatch.setattr(suite.lock, "remaining_s", lambda: next(remaining))
    monkeypatch.setattr(suite.pi, "foreign_deploy", lambda deployed: None)
    monkeypatch.setattr(
        suite,
        "run_hardware",
        lambda row, suite_dir, timeout: ran.append(row.name)
        or {"name": row.name},
    )
    result = {"deployed": {}, "rows": [], "deferred": []}
    plan = {**LATER_HOLD, "rows": ["first_layers", "cancel_then_print"]}
    suite.run_rows(plan, result, lambda: None)
    assert ran == ["first_layers"]
    assert result["deferred"] == ["cancel_then_print"]


def test_a_hold_that_never_got_the_lock_records_why(monkeypatch, tmp_path):
    plan_path = tmp_path / "1-plan.json"
    result_path = tmp_path / "1.json"
    plan = {**FIRST_HOLD, "result": str(result_path)}
    plan_path.write_text(json.dumps(plan))

    @contextlib.contextmanager
    def queue_timeout(*_args, **_kwargs):
        raise suite.pi.BenchError("bench lock still held after 1 s by pid 9")
        yield

    monkeypatch.setattr(suite.lock, "bench_lock", queue_timeout)
    with pytest.raises(SystemExit) as exited:
        suite.hold_main(plan_path)
    assert exited.value.code == 1
    result = json.loads(result_path.read_text())
    assert result["aborted"] == "bench lock still held after 1 s by pid 9"
    assert result["acquired"] is None


def _run_row(row, tmp_path):
    return suite.judge_row(
        row, suite.run_hardware(row, tmp_path, 1.0), suite.BENCH
    )


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
    result = _run_row(row, tmp_path)
    assert calls == ["session.py", "session_check.py"]
    assert result["error"] is None
    assert not result["passed"]
    assert result["failed_checks"] == ["gaps: 9.71 s > 1 s (#843)"]
    assert result["analyzer"] == "not run: several prints"


def test_a_session_row_whose_checks_pass_passes(monkeypatch, tmp_path):
    checks = [{"name": "gaps", "ok": True, "detail": "0.2 s <= 1 s"}]
    python, _ = _session_runner(checks)
    monkeypatch.setattr(suite, "python", python)
    result = _run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path)
    assert result["passed"]
    assert result["failed_checks"] == []


def test_a_session_row_without_a_verdict_is_an_error(monkeypatch, tmp_path):
    python, _ = _session_runner(None)
    monkeypatch.setattr(suite, "python", python)
    result = _run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path)
    assert not result["passed"]
    assert result["error"].startswith("session_check.py")


def _failing_runner(returncode):
    calls = []

    def python(script, *args, timeout_s):
        calls.append(" ".join([script, *args[:1]]))
        if script == "session.py":
            return _completed(returncode, stderr="the runner's last words")
        return _completed(0)

    return python, calls


def test_a_driver_side_runner_failure_leaves_the_target_alone(
    monkeypatch, tmp_path
):
    python, calls = _failing_runner(suite.pi.DRIVER_SIDE_EXIT)
    monkeypatch.setattr(suite, "python", python)
    result = _run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path)
    assert not any(call.startswith("deploy.py") for call in calls)
    assert "recovered_with_reset_target" not in result
    assert result["error"] == "session.py: the runner's last words"
    assert not result["passed"]


def test_a_printer_side_runner_failure_resets_the_target(monkeypatch, tmp_path):
    python, calls = _failing_runner(1)
    monkeypatch.setattr(suite, "python", python)
    result = _run_row(suite.SessionRow(suite.SCENARIOS[0]), tmp_path)
    assert "deploy.py reset-target" in calls
    assert result["recovered_with_reset_target"]
    assert not result["passed"]
