import importlib.util
import os

import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_refine_report.py",
)
_spec = importlib.util.spec_from_file_location(
    "servo_refine_report_script", _SCRIPT
)
srr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(srr)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def test_parse_step_name():
    assert srr.parse_step_name("refine_speed_v2500_20260628_120000.scap") == (
        "speed",
        2500,
    )
    assert srr.parse_step_name("refine_position_v400_20260628_120000.scap") == (
        "position",
        400,
    )
    assert srr.parse_step_name(
        "refine_integral_v3184_20260628_120000.scap"
    ) == ("integral", 3184)


def test_parse_step_name_rejects_foreign_names():
    assert srr.parse_step_name("inertia_r70_20260628_120000.scap") == (
        None,
        None,
    )
    assert srr.parse_step_name(
        "cal_p2000_s1250_i1000_20260628_120000.scap"
    ) == (None, None)


def test_named_steps_pick_newest_capture_per_step(tmp_path):
    d = str(tmp_path)
    _touch(d, "refine_speed_v2500_20260628_210000.scap")
    newest = _touch(d, "refine_speed_v2500_20260628_220000.scap")
    files = srr.find_named_steps(d, ["refine_speed_v2500"])
    assert files == [(2500, newest)]


def test_named_steps_exclude_stale_steps_from_other_runs(tmp_path):
    d = str(tmp_path)
    _touch(d, "refine_speed_v3250_20260627_180000.scap")
    kept = _touch(d, "refine_speed_v2500_20260628_220000.scap")
    files = srr.find_named_steps(d, ["refine_speed_v2500"])
    assert files == [(2500, kept)]


def test_named_steps_sort_by_value(tmp_path):
    d = str(tmp_path)
    hi = _touch(d, "refine_speed_v3250_20260628_220010.scap")
    lo = _touch(d, "refine_speed_v1750_20260628_220000.scap")
    files = srr.find_named_steps(
        d, ["refine_speed_v3250", "refine_speed_v1750"]
    )
    assert files == [(1750, lo), (3250, hi)]


def test_named_step_without_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="refine_speed_v2500"):
        srr.find_named_steps(str(tmp_path), ["refine_speed_v2500"])


def test_find_sweep_files_filters_by_param(tmp_path):
    d = str(tmp_path)
    _touch(d, "refine_position_v400_20260628_220000.scap")
    kept = _touch(d, "refine_speed_v2500_20260628_220000.scap")
    files = srr.find_sweep_files(d, "refine", "speed")
    assert files == [(2500, kept)]


def test_explicit_files_and_steps_are_mutually_exclusive(tmp_path):
    scap = _touch(str(tmp_path), "refine_speed_v2500_20260628_220000.scap")
    with pytest.raises(SystemExit, match="not both"):
        srr.main([scap, "--steps", "refine_speed_v2500"])


def test_explicit_file_without_value_field_fails_loudly(tmp_path):
    scap = _touch(str(tmp_path), "track_20260628_220000.scap")
    with pytest.raises(SystemExit, match="value field"):
        srr.main([scap])


def test_no_captures_found_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="no refinement captures found"):
        srr.main(
            [
                "--captures-dir",
                str(tmp_path),
                "--tag",
                "nope",
                "--param",
                "speed",
            ]
        )


def test_tag_resolution_requires_param(tmp_path):
    with pytest.raises(SystemExit, match="--param is required"):
        srr.main(["--captures-dir", str(tmp_path), "--tag", "refine"])
