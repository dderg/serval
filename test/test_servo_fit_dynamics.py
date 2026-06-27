import argparse
import importlib.util
import os

import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_fit_dynamics.py",
)
_spec = importlib.util.spec_from_file_location(
    "servo_fit_dynamics_script", _SCRIPT
)
sfd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sfd)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def _args(**overrides):
    base = {
        "structure": "scalar",
        "rated_torque_nm": None,
        "rotor_inertia_kgm2": None,
        "rotation_distance_mm": None,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_resolves_newest_capture_for_name(tmp_path):
    d = str(tmp_path)
    _touch(d, "ident_20260611_210000.scap")
    newest = _touch(d, "ident_20260611_230000.scap")
    assert sfd.resolve_newest_capture(d, "ident") == newest


def test_missing_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="ident"):
        sfd.resolve_newest_capture(str(tmp_path), "ident")


def test_ignores_other_series_sharing_a_name_prefix(tmp_path):
    d = str(tmp_path)
    newest = _touch(d, "ident_20260616_010942.scap")
    _touch(d, "ident_ha_20260611_181313.scap")
    assert sfd.resolve_newest_capture(d, "ident") == newest


def test_other_series_excluded_even_when_it_sorts_after(tmp_path):
    d = str(tmp_path)
    _touch(d, "ident_zzz_20260601_000000.scap")
    newest = _touch(d, "ident_20260616_010942.scap")
    assert sfd.resolve_newest_capture(d, "ident") == newest


def test_profile_name_carries_capture_timestamp(tmp_path):
    path = sfd.profile_path(
        str(tmp_path), "ident", "/x/ident_20260611_230000.scap"
    )
    assert os.path.basename(path) == "dynamics_ident_20260611_230000.toml"


def test_ident_cmd_without_physical_params():
    cmd = sfd.ident_cmd(
        "/bin/servo-ident", "/tmp/c.csv", "node_x", "/o.toml", _args()
    )
    assert cmd == [
        "/bin/servo-ident",
        "--capture",
        "/tmp/c.csv",
        "--structure",
        "scalar",
        "--axes",
        "node_x",
        "--out",
        "/o.toml",
    ]


def _header_with(rotation=None):
    drive = {"name": "x", "counts_per_mm": 3276.8}
    if rotation is not None:
        drive["rotation_distance"] = rotation
    return {"drives": [drive]}


def test_rotation_distance_taken_from_header_when_not_overridden():
    assert sfd.resolve_rotation_distance(_args(), _header_with(40.0), 0) == 40.0


def test_rotation_distance_cli_overrides_header():
    args = _args(rotation_distance_mm=18.0)
    assert sfd.resolve_rotation_distance(args, _header_with(40.0), 0) == 18.0


def test_rotation_distance_missing_from_old_header_is_none():
    assert sfd.resolve_rotation_distance(_args(), _header_with(None), 0) is None


def test_ident_cmd_appends_physical_params():
    cmd = sfd.ident_cmd(
        "/bin/servo-ident",
        "/tmp/c.csv",
        "node_x",
        "/o.toml",
        _args(
            rated_torque_nm=1.27,
            rotor_inertia_kgm2=0.000057,
            rotation_distance_mm=40.0,
        ),
    )
    assert cmd[-6:] == [
        "--rated-torque-nm",
        "1.27",
        "--rotor-inertia-kgm2",
        "5.7e-05",
        "--rotation-distance-mm",
        "40.0",
    ]
