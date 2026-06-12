#!/usr/bin/env python3
from __future__ import annotations

import importlib
import inspect

import pytest

from klippy.motion_toolhead import MotionToolhead, Move

pytestmark = pytest.mark.sim_unit


def test_motion_toolhead_is_standalone():
    assert MotionToolhead.__bases__ == (object,), (
        "MotionToolhead must not inherit a base toolhead: the legacy "
        "lookahead/flush machinery was deleted with klippy/toolhead.py and "
        "must not creep back in via subclassing."
    )


def test_legacy_toolhead_module_is_gone():
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("klippy.toolhead")


EXTRAS_FACING_API = frozenset(
    {
        "check_busy",
        "drip_move",
        "dwell",
        "flush_step_generation",
        "get_extruder",
        "get_kinematics",
        "get_last_move_time",
        "get_max_velocity",
        "get_position",
        "get_status",
        "get_trapq",
        "limit_next_junction_speed",
        "manual_move",
        "move",
        "note_mcu_movequeue_activity",
        "note_step_generation_scan_time",
        "register_lookahead_callback",
        "register_step_generator",
        "reset_accel",
        "set_accel",
        "set_extruder",
        "set_position",
        "stats",
        "wait_moves",
        "wait_moves_and_mcu",
    }
)


def test_extras_facing_api_is_present():
    missing = {
        name
        for name in EXTRAS_FACING_API
        if not callable(getattr(MotionToolhead, name, None))
    }
    assert not missing, (
        "MotionToolhead lost extras-facing toolhead API: %s" % sorted(missing)
    )


def test_move_keeps_validation_surface():
    for name in ("limit_speed", "move_error"):
        assert callable(getattr(Move, name, None))


def test_no_motion_toolhead_path_calls_note_mcu_movequeue_activity():
    forbidden = "note_mcu_movequeue_activity"
    offenders = []
    for name, val in MotionToolhead.__dict__.items():
        if name == forbidden or not callable(val):
            continue
        try:
            src = inspect.getsource(val)
        except (TypeError, OSError):
            continue
        if (".%s(" % forbidden) in src:
            offenders.append(name)
    assert not offenders, (
        "note_mcu_movequeue_activity is a no-op kept only for extras "
        "(output_pin); MotionToolhead methods must not rely on it:\n  %s"
        % offenders
    )
