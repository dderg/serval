import pytest
from fakes import FakeEngine, FakeGcmd, FakePrinter, FakeReactor
from fakes import FakeToolhead as _FakeToolhead

from klippy.extras.homing import Homing


class ArmReached(Exception):
    """Sentinel proving trip_move armed the endstop instead of erroring."""


class FakeEndstop:
    def __init__(self, triggered_after_wait):
        self.queued_motion_pending = True
        self.triggered_after_wait = triggered_after_wait
        self.calls = []

    def engine_mcu_handle(self):
        return object()

    def is_triggered(self):
        self.calls.append("is_triggered")
        if self.queued_motion_pending:
            return True
        return self.triggered_after_wait

    def arm(self, poll_period):
        self.calls.append("arm")
        raise ArmReached()


class FakeToolhead(_FakeToolhead):
    def __init__(self, endstop):
        super().__init__(position=[0.0, 0.0, 0.0, 0.0])
        self.endstop = endstop

    def wait_moves(self):
        super().wait_moves()
        self.endstop.calls.append("wait_moves")
        self.endstop.queued_motion_pending = False


def run_trip_move(endstop):
    homing = Homing.__new__(Homing)
    homing.printer = FakePrinter(reactor=FakeReactor())
    toolhead = FakeToolhead(endstop)
    entry = {"endstops": [endstop], "provider": None}
    homing.trip_move(
        FakeGcmd(error=RuntimeError),
        toolhead,
        FakeEngine(),
        2,
        -1,
        5.0,
        40.0,
        entry,
    )


def test_trip_move_arms_when_endstop_already_triggered():
    # Matches mainline: an already-triggered endstop is not a precheck error.
    # The first approach arms the endstop and lets it insta-trip — the MCU
    # samples the live pin and brakes immediately (src/endstop.c) and the
    # engine buffers the early trip.
    endstop = FakeEndstop(triggered_after_wait=True)
    with pytest.raises(ArmReached):
        run_trip_move(endstop)
    assert "arm" in endstop.calls
    assert "is_triggered" not in endstop.calls


def test_trip_move_waits_for_queued_motion_before_arming():
    endstop = FakeEndstop(triggered_after_wait=False)
    with pytest.raises(ArmReached):
        run_trip_move(endstop)
    assert endstop.calls.index("wait_moves") < endstop.calls.index("arm")
