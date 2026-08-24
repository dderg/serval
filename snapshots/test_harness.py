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
}


def _case(tmp_path, name="grp/printer/unit") -> harness.Case:
    case = harness.Case(
        name=name,
        gcode_path=tmp_path / "unit.gcode",
        config_path=tmp_path / "printer.cfg",
        baseline_path=tmp_path
        / "baselines"
        / "grp"
        / "printer"
        / "unit.baseline.json.gz",
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
            name="grp/printer/unit",
            gcode_path=group / "unit.gcode",
            config_path=group / "printer.cfg",
            baseline_path=tmp_path
            / "baselines"
            / "grp"
            / "printer"
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

    assert [case.name for case in cases] == ["grp/printer/unit"]


def test_discover_cases_matrix_cross_product(tmp_path):
    group = tmp_path / "cases" / "grp"
    group.mkdir(parents=True)
    (group / "a.cfg").write_text("[printer]\n")
    (group / "b.cfg").write_text("[printer]\n")
    (group / "x.gcode").write_text("G1 X1\n")
    (group / "y.gcode").write_text("G1 Y1\n")

    cases = harness.discover_cases(tmp_path / "cases", tmp_path / "baselines")

    assert [case.name for case in cases] == [
        "grp/a/x",
        "grp/a/y",
        "grp/b/x",
        "grp/b/y",
    ]
    bx = next(case for case in cases if case.name == "grp/b/x")
    assert bx.config_path == group / "b.cfg"
    assert bx.gcode_path == group / "x.gcode"
    assert (
        bx.baseline_path
        == tmp_path / "baselines" / "grp" / "b" / "x.baseline.json.gz"
    )


def test_discover_cases_raises_when_gcode_without_config(tmp_path):
    group = tmp_path / "cases" / "grp"
    group.mkdir(parents=True)
    (group / "x.gcode").write_text("G1 X1\n")

    with pytest.raises(ValueError, match="grp.*no .cfg"):
        harness.discover_cases(tmp_path / "cases", tmp_path / "baselines")


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


def test_compare_tolerates_sub_ulp_float_drift(tmp_path):
    case = _case(tmp_path)
    harness.write_baseline(case, _SNAPSHOT)
    drifted = dict(_SNAPSHOT)
    drifted["kin_v"] = [0.0, 50.0 + 1e-11, 0.0]
    drifted["traversal_time_s"] = 0.123456789 + 1e-13
    assert harness.compare(case, drifted) is harness.Status.EXACT


def test_snapshots_match_flags_drift_above_tolerance():
    a = {"kin_v": [50.0]}
    b = {"kin_v": [50.0 + 1e-3]}
    assert not harness.snapshots_match(a, b)


def test_snapshots_match_is_exact_on_integer_counts():
    assert harness.snapshots_match({"blended": 3}, {"blended": 3})
    assert not harness.snapshots_match({"blended": 3}, {"blended": 4})


def test_compare_flags_velocity_field_change(tmp_path):
    case = _case(tmp_path)
    harness.write_baseline(case, _SNAPSHOT)
    drifted = dict(_SNAPSHOT)
    drifted["kin_v"] = [0.0, 50.5, 0.0]
    assert harness.compare(case, drifted) is harness.Status.CHANGED


def test_snapshots_match_flags_structural_change():
    a = {"segments": [{"type": "line"}, {"type": "arc"}]}
    b = {"segments": [{"type": "line"}]}
    assert not harness.snapshots_match(a, b)
    assert not harness.snapshots_match(
        {"segments": [{"type": "line"}]},
        {"segments": [{"type": "arc"}]},
    )


def test_snapshots_match_rejects_type_mismatch_and_nan():
    assert not harness.snapshots_match(1.0, "1.0")
    assert not harness.snapshots_match(
        {"v": [float("nan")]}, {"v": [float("nan")]}
    )


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


def test_read_printer_config_serializes_document(tmp_path):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(
        "[printer]\n"
        "max_velocity: 300\n"
        "max_accel: 1000\n"
        "square_corner_velocity: 0\n"
        "max_jerk: 100000\n"
        "\n"
        "[post_processor is_xy]\n"
        "type: smooth_mzv\n"
        "frequency_hz: 39.3\n"
        "\n"
        "[axis x]\n"
        "post_processors: is_xy\n"
    )
    data = harness.read_printer_config(cfg)
    assert data.max_velocity == 300.0
    assert data.max_accel == 1000.0
    # The engine re-reads the motion sections from the serialized document;
    # section parsing itself is covered by planner-config's from_doc tests.
    assert "[post_processor is_xy]" in data.config_text
    assert "frequency_hz = 39.3" in data.config_text
    assert "[axis x]" in data.config_text


def test_parse_gcode_set_velocity_limit_accel(tmp_path):
    gcode = tmp_path / "two_pass.gcode"
    gcode.write_text(
        "G1 X10 F6000\n"
        "SET_VELOCITY_LIMIT ACCEL=500\n"
        "G1 X20\n"
        "SET_VELOCITY_LIMIT ACCEL=8000\n"
        "G1 X30\n"
    )
    wp = harness.parse_gcode(gcode, 300.0, 3000.0)
    assert [p[5] for p in wp] == [3000.0, 500.0, 8000.0]


def test_parse_gcode_rejects_unsupported_velocity_limit_param(tmp_path):
    gcode = tmp_path / "bad.gcode"
    gcode.write_text("SET_VELOCITY_LIMIT VELOCITY=100\nG1 X10 F6000\n")
    with pytest.raises(ValueError, match="only ACCEL"):
        harness.parse_gcode(gcode, 300.0, 3000.0)


def test_parse_gcode_rejects_non_positive_accel(tmp_path):
    gcode = tmp_path / "bad.gcode"
    gcode.write_text("SET_VELOCITY_LIMIT ACCEL=0\nG1 X10 F6000\n")
    with pytest.raises(ValueError, match="positive finite"):
        harness.parse_gcode(gcode, 300.0, 3000.0)


def test_drift_envelope_reports_schema_change():
    drift = harness.drift_envelope(
        {"schema_version": 1, "traj_x_pieces": []},
        {"schema_version": 2, "trajectory": {}},
    )
    assert drift["schema_at"] == "<root>"


def test_drift_envelope_reports_numeric_change_inside_same_schema():
    drift = harness.drift_envelope(
        {"schema_version": 2, "trajectory": {"t_end": 1.0}},
        {"schema_version": 2, "trajectory": {"t_end": 2.0}},
    )
    assert drift["schema_at"] == ""
    assert drift["rel"] == 0.5
