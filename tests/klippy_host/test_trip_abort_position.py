import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from klippy.extras import homing  # noqa: E402


class FakeCommandError(Exception):
    pass


class FakeGcmd:
    def error(self, msg):
        return FakeCommandError(msg)


class FakeToolhead:
    def __init__(self, position):
        self.position = list(position)
        self.set_position_calls = []

    def get_position(self):
        return list(self.position)

    def set_position(self, newpos, homing_axes=()):
        self.set_position_calls.append((list(newpos), tuple(homing_axes)))
        self.position = list(newpos)


class FakeEngine:
    def __init__(self, abort_result):
        self.abort_result = abort_result
        self.abort_calls = 0

    def home_abort(self):
        self.abort_calls += 1
        return self.abort_result


def make_homing():
    return homing.Homing.__new__(homing.Homing)


def test_adopts_reconciled_stop_position_without_touching_homed_state():
    toolhead = FakeToolhead([150.0, 245.0, 15.0, 7.5])
    engine = FakeEngine([150.0, 245.0, -4.8])
    make_homing()._abort_trip_and_adopt_stop_position(
        FakeGcmd(), toolhead, engine, 2
    )
    assert engine.abort_calls == 1
    assert toolhead.set_position_calls == [
        ([150.0, 245.0, -4.8, 7.5], ()),
    ]


def test_unreconciled_abort_raises_firmware_restart_error():
    toolhead = FakeToolhead([150.0, 245.0, 15.0, 0.0])
    engine = FakeEngine(None)
    with pytest.raises(FakeCommandError, match="FIRMWARE_RESTART"):
        make_homing()._abort_trip_and_adopt_stop_position(
            FakeGcmd(), toolhead, engine, 2
        )
    assert toolhead.set_position_calls == []
