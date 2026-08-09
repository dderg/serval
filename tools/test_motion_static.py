#!/usr/bin/env python3
from __future__ import annotations

import importlib
import inspect

import pytest

from klippy.motion import Motion, Move

pytestmark = pytest.mark.sim_unit


def test_motion_is_standalone():
    assert Motion.__bases__ == (object,), (
        "Motion must not inherit a base toolhead: the legacy "
        "lookahead/flush machinery was deleted with klippy/toolhead.py and "
        "must not creep back in via subclassing."
    )


def test_legacy_toolhead_module_is_gone():
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("klippy.toolhead")


def test_move_keeps_validation_surface():
    for name in ("limit_speed", "move_error"):
        assert callable(getattr(Move, name, None))


def test_no_motion_path_calls_note_mcu_movequeue_activity():
    forbidden = "note_mcu_movequeue_activity"
    offenders = []
    for name, val in Motion.__dict__.items():
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
        "(output_pin); Motion methods must not rely on it:\n  %s" % offenders
    )
