from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path

import hold_plan
import lock
import pytest
import suite

RUN_DIR_STAMP = "%Y%m%d-%H%M%S"
ESTIMATE_SLACK = 1.2
GROUPS = [suite.selected_rows(scope) for scope in suite.Scope]


def _names(rows) -> list[str]:
    return [row.name for row in rows]


@pytest.mark.parametrize("rows", GROUPS, ids=[s.value for s in suite.Scope])
def test_no_planned_hold_exceeds_its_row_budget(rows):
    holds = hold_plan.plan(_names(rows))
    assert [name for hold in holds for name in hold] == _names(rows)
    for hold in holds:
        assert hold_plan.planned_s(hold) <= hold_plan.HOLD_ROWS_S


def test_the_row_budget_leaves_the_restore_and_the_margin_inside_the_cap():
    reserved = hold_plan.RESTORE_RESERVE_S + hold_plan.HOLD_MARGIN_S
    assert hold_plan.HOLD_ROWS_S + reserved == lock.MAX_HOLD_S


def test_the_full_suite_splits_into_several_holds():
    holds = hold_plan.plan(_names(suite.ROWS))
    assert len(holds) > 1
    assert hold_plan.planned_s(_names(suite.ROWS)) > hold_plan.HOLD_ROWS_S


def test_quick_fits_one_hold():
    assert (
        len(hold_plan.plan(_names(suite.selected_rows(suite.Scope.QUICK)))) == 1
    )


def test_each_soak_row_is_a_hold_of_its_own():
    soak = _names(suite.SOAK_ROWS)
    assert hold_plan.plan(soak) == [[name] for name in soak]


def test_a_row_the_table_does_not_know_gets_a_hold_of_its_own():
    holds = hold_plan.plan(["epochs", "new_row", "epochs"])
    assert holds == [["epochs"], ["new_row"], ["epochs"]]
    assert hold_plan.expected_s("new_row") == hold_plan.HOLD_ROWS_S


def test_a_row_longer_than_a_hold_is_refused(monkeypatch):
    monkeypatch.setitem(
        hold_plan.EXPECTED_ROW_S, "epochs", hold_plan.HOLD_ROWS_S + 1
    )
    with pytest.raises(hold_plan.PlanError, match="shorten the row"):
        hold_plan.plan(["epochs"])


def test_a_row_starts_only_when_it_and_the_margin_fit_the_hold():
    needed = hold_plan.expected_s("first_layers") + hold_plan.HOLD_MARGIN_S
    assert hold_plan.fits("first_layers", needed)
    assert not hold_plan.fits("first_layers", needed - 1)


def test_the_table_names_exactly_the_suite_rows():
    assert set(hold_plan.EXPECTED_ROW_S) == set(suite.ROW_BY_NAME)


def _held_s(row: dict) -> float | None:
    if row.get("held_s") is not None:
        return row["held_s"]
    run_dir = Path(row.get("run_dir") or "")
    run_json = run_dir / "run.json"
    if not run_json.exists():
        return None
    started = datetime.strptime(run_dir.name, RUN_DIR_STAMP).timestamp()
    return os.path.getmtime(run_json) - started


def _same_row(row: dict) -> bool:
    current = suite.row_identity(suite.ROW_BY_NAME[row["name"]])
    return all(row.get(key) == value for key, value in current.items())


def _latest_held_s() -> dict[str, tuple[float, str]]:
    latest: dict[str, tuple[float, str]] = {}
    for summary in sorted(suite.SUITES_DIR.glob("*/summary.json")):
        for row in json.loads(summary.read_text())["rows"]:
            held_s = _held_s(row)
            if row["name"] in suite.ROW_BY_NAME and _same_row(row) and held_s:
                latest[row["name"]] = (held_s, str(summary))
    return latest


def test_the_table_holds_against_each_rows_latest_bench_run():
    latest = _latest_held_s()
    if not latest:
        pytest.skip(f"no suite summary under {suite.SUITES_DIR} on this host")
    over = {
        name: f"{held_s:.0f} s in {summary}"
        for name, (held_s, summary) in latest.items()
        if held_s > ESTIMATE_SLACK * hold_plan.expected_s(name)
    }
    assert not over, f"hold_plan.EXPECTED_ROW_S is stale: {over}"
