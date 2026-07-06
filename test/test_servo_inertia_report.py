import importlib.util
import os

import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_inertia_report.py",
)
_spec = importlib.util.spec_from_file_location(
    "servo_inertia_report_script", _SCRIPT
)
sir = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sir)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def test_ratio_from_name():
    assert sir.ratio_from_name("inertia_r70_20260628_120000.scap") == 70
    assert (
        sir.ratio_from_name("cal_p2000_s1250_i1000_20260628_120000.scap")
        is None
    )


def test_named_steps_pick_newest_capture_per_step(tmp_path):
    d = str(tmp_path)
    _touch(d, "inertia_r40_20260628_210000.scap")
    newest = _touch(d, "inertia_r40_20260628_220000.scap")
    files = sir.find_named_steps(d, ["inertia_r40"])
    assert files == [(40, newest)]


def test_named_steps_exclude_stale_steps_from_other_runs(tmp_path):
    d = str(tmp_path)
    _touch(d, "inertia_r130_20260627_180000.scap")
    kept = _touch(d, "inertia_r100_20260628_220000.scap")
    files = sir.find_named_steps(d, ["inertia_r100"])
    assert files == [(100, kept)]


def test_named_steps_sort_by_ratio(tmp_path):
    d = str(tmp_path)
    hi = _touch(d, "inertia_r130_20260628_220010.scap")
    lo = _touch(d, "inertia_r40_20260628_220000.scap")
    files = sir.find_named_steps(d, ["inertia_r130", "inertia_r40"])
    assert files == [(40, lo), (130, hi)]


def test_named_step_without_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="inertia_r40"):
        sir.find_named_steps(str(tmp_path), ["inertia_r40"])


def test_explicit_files_and_steps_are_mutually_exclusive(tmp_path):
    scap = _touch(str(tmp_path), "inertia_r40_20260628_220000.scap")
    with pytest.raises(SystemExit, match="not both"):
        sir.main([scap, "--steps", "inertia_r40"])


def test_explicit_file_without_ratio_field_fails_loudly(tmp_path):
    scap = _touch(str(tmp_path), "track_20260628_220000.scap")
    with pytest.raises(SystemExit, match="ratio field"):
        sir.main([scap])


def test_no_captures_found_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="no sweep captures found"):
        sir.main(["--captures-dir", str(tmp_path), "--tag", "nope"])
