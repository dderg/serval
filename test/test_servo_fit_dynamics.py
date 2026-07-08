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
        "/bin/servo-ident", "/tmp/c.csv", ["node_x"], "/o.toml", _args()
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
    assert (
        sfd.resolve_rotation_distance(_args(), _header_with(40.0), [0]) == 40.0
    )


def test_rotation_distance_cli_overrides_header():
    args = _args(rotation_distance_mm=18.0)
    assert sfd.resolve_rotation_distance(args, _header_with(40.0), [0]) == 18.0


def test_rotation_distance_missing_from_old_header_is_none():
    assert (
        sfd.resolve_rotation_distance(_args(), _header_with(None), [0]) is None
    )


def test_ident_cmd_appends_physical_params():
    cmd = sfd.ident_cmd(
        "/bin/servo-ident",
        "/tmp/c.csv",
        ["node_x"],
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


def test_corexy_layout_two_drives_keeps_capture_order():
    axes, structure = sfd.corexy_layout(["motor_b", "motor_a"], None)
    assert axes == ["motor_b", "motor_a"]
    assert structure == "corexy"


def test_corexy_layout_two_drives_rejects_pairs():
    with pytest.raises(SystemExit, match="4-drive"):
        sfd.corexy_layout(["a", "b"], "a,b;c,d")


def test_corexy_layout_awd_orders_axes_from_pairs():
    axes, structure = sfd.corexy_layout(
        ["motor_b1", "motor_a", "motor_b", "motor_a1"],
        "motor_a,motor_a1;motor_b,motor_b1",
    )
    assert axes == ["motor_a", "motor_a1", "motor_b", "motor_b1"]
    assert structure == "corexy-awd"


def test_corexy_layout_awd_requires_pairs():
    with pytest.raises(SystemExit, match="--pairs"):
        sfd.corexy_layout(["a", "a1", "b", "b1"], None)


def test_corexy_layout_awd_pairs_must_match_capture_drives():
    with pytest.raises(SystemExit, match="do not match"):
        sfd.corexy_layout(["a", "a1", "b", "b1"], "a,a1;b,b2")


def test_corexy_layout_rejects_odd_drive_counts():
    with pytest.raises(SystemExit, match="2-drive or 4-drive"):
        sfd.corexy_layout(["a", "a1", "b"], None)


def test_parse_pairs_rejects_malformed_spec():
    with pytest.raises(SystemExit, match="two belt pairs"):
        sfd.parse_pairs("a,a1,b,b1")
    with pytest.raises(SystemExit, match="two belt pairs"):
        sfd.parse_pairs("a;b")
    assert sfd.parse_pairs("a, a1 ; b,b1") == [["a", "a1"], ["b", "b1"]]


def test_ident_cmd_structure_override_wins():
    cmd = sfd.ident_cmd(
        "/bin/servo-ident",
        "/tmp/c.csv",
        ["a", "a1", "b", "b1"],
        "/o.toml",
        _args(structure="corexy"),
        structure="corexy-awd",
    )
    assert cmd[cmd.index("--structure") + 1] == "corexy-awd"
    assert cmd[cmd.index("--axes") + 1] == "a,a1,b,b1"
