import pytest
from fakes import FakeCommandError, FakeGcmd, FakeToolhead

from klippy.extras import homing


class FakeHomeAbortEngine:
    def __init__(self, abort_result):
        self.abort_result = abort_result
        self.abort_calls = 0

    def home_abort(self):
        self.abort_calls += 1
        return self.abort_result


def make_homing():
    return homing.Homing.__new__(homing.Homing)


def test_adopts_reconciled_stop_position_without_touching_homed_state():
    toolhead = FakeToolhead(position=[150.0, 245.0, 15.0, 7.5])
    engine = FakeHomeAbortEngine([150.0, 245.0, -4.8])
    make_homing()._abort_trip_and_adopt_stop_position(
        FakeGcmd(), toolhead, engine, 2
    )
    assert engine.abort_calls == 1
    assert toolhead.calls == [
        ("set_position", [150.0, 245.0, -4.8, 7.5], ()),
    ]


def test_unreconciled_abort_raises_firmware_restart_error():
    toolhead = FakeToolhead(position=[150.0, 245.0, 15.0, 0.0])
    engine = FakeHomeAbortEngine(None)
    with pytest.raises(FakeCommandError, match="FIRMWARE_RESTART"):
        make_homing()._abort_trip_and_adopt_stop_position(
            FakeGcmd(), toolhead, engine, 2
        )
    assert toolhead.calls == []
