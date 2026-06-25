"""Unit tests for the snapshot harness comparison machinery.

Pure logic — no planner, no _motion_engine — so these are ordinary python-unit
tests, collected by the `py` job. The snapshot cases themselves run under the
standalone `run.py`, not pytest.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness  # noqa: E402

_SNAPSHOT = {
    "kin_s": [0.0, 1.0, 2.0],
    "kin_v": [0.0, 50.0, 0.0],
    "kin_kappa": [0.0, 0.1, 0.0],
    "traversal_time_s": 0.123456789,
    "fitted_segments": [{"type": "line", "x0": 0.0, "y0": 0.0}],
}


def _case(tmp_path, name="grp/unit") -> harness.Case:
    case = harness.Case(
        name=name,
        gcode_path=tmp_path / "unit.gcode",
        config_path=tmp_path / "printer.cfg",
        baseline_path=tmp_path / "baselines" / "grp" / "unit.baseline.json.gz",
    )
    case.gcode_path.write_text("G1 X1\n")
    case.config_path.write_text("[printer]\n")
    return case


def test_canonical_json_is_order_independent():
    a = harness.canonical_json({"b": 1, "a": 2})
    b = harness.canonical_json({"a": 2, "b": 1})
    assert a == b


def test_canonical_json_round_trips_floats():
    text = harness.canonical_json(_SNAPSHOT)
    import json

    assert json.loads(text)["traversal_time_s"] == 0.123456789


def test_compare_new_when_no_baseline(tmp_path):
    case = _case(tmp_path)
    assert harness.compare(case, _SNAPSHOT) is harness.Status.NEW


def test_discover_cases_keeps_baselines_separate(tmp_path):
    group = tmp_path / "cases" / "grp"
    group.mkdir(parents=True)
    (group / "printer.cfg").write_text("[printer]\n")
    (group / "unit.gcode").write_text("G1 X1\n")

    cases = harness.discover_cases(tmp_path / "cases", tmp_path / "baselines")

    assert cases == [
        harness.Case(
            name="grp/unit",
            gcode_path=group / "unit.gcode",
            config_path=group / "printer.cfg",
            baseline_path=tmp_path
            / "baselines"
            / "grp"
            / "unit.baseline.json.gz",
        )
    ]


def test_discover_cases_ignores_empty_gcode(tmp_path):
    group = tmp_path / "cases" / "grp"
    group.mkdir(parents=True)
    (group / "printer.cfg").write_text("[printer]\n")
    (group / "empty.gcode").write_text("\n  \t\n")
    (group / "unit.gcode").write_text("G1 X1\n")

    cases = harness.discover_cases(tmp_path / "cases", tmp_path / "baselines")

    assert [case.name for case in cases] == ["grp/unit"]


def test_compare_exact_after_write(tmp_path):
    case = _case(tmp_path)
    harness.write_baseline(case, _SNAPSHOT)
    assert case.baseline_path.exists()
    assert harness.compare(case, _SNAPSHOT) is harness.Status.EXACT


def test_compare_changed_on_deviation(tmp_path):
    case = _case(tmp_path)
    harness.write_baseline(case, _SNAPSHOT)
    perturbed = dict(_SNAPSHOT)
    perturbed["kin_v"] = [0.0, 50.0001, 0.0]
    assert harness.compare(case, perturbed) is harness.Status.CHANGED


def test_baseline_snapshot_round_trips(tmp_path):
    case = _case(tmp_path)
    harness.write_baseline(case, _SNAPSHOT)
    assert harness.baseline_snapshot(case) == _SNAPSHOT


def test_run_case_missing_gcode_raises(tmp_path):
    case = harness.Case(
        name="grp/bad",
        gcode_path=tmp_path / "bad.gcode",
        config_path=tmp_path / "printer.cfg",
        baseline_path=tmp_path / "bad.baseline.json.gz",
    )
    case.config_path.write_text("[printer]\n")
    with pytest.raises(ValueError, match="missing bad.gcode"):
        harness.run_case(case)


def _live_case_with_baseline(tmp_path, stem):
    group = tmp_path / "cases" / "grp"
    group.mkdir(parents=True, exist_ok=True)
    (group / "printer.cfg").write_text("[printer]\n")
    (group / f"{stem}.gcode").write_text("G1 X1\n")
    cases = harness.discover_cases(tmp_path / "cases", tmp_path / "baselines")
    for case in cases:
        harness.write_baseline(case, _SNAPSHOT)
    return cases


def test_prune_removes_orphan_baseline(tmp_path):
    cases = _live_case_with_baseline(tmp_path, "live")
    orphan = tmp_path / "baselines" / "grp" / "gone.baseline.json.gz"
    orphan.write_bytes(b"stale")

    pruned = harness.prune_orphan_baselines(cases, tmp_path / "baselines")

    assert pruned == [orphan]
    assert not orphan.exists()
    assert cases[0].baseline_path.exists()


def test_prune_keeps_live_baselines(tmp_path):
    cases = _live_case_with_baseline(tmp_path, "a")
    cases = _live_case_with_baseline(tmp_path, "b")

    pruned = harness.prune_orphan_baselines(cases, tmp_path / "baselines")

    assert pruned == []
    assert all(case.baseline_path.exists() for case in cases)


def test_prune_removes_now_empty_group_dir(tmp_path):
    baselines = tmp_path / "baselines"
    dead = baselines / "dead"
    dead.mkdir(parents=True)
    (dead / "x.baseline.json.gz").write_bytes(b"stale")

    pruned = harness.prune_orphan_baselines([], baselines)

    assert len(pruned) == 1
    assert not dead.exists()
