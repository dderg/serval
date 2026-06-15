import pytest

from klippy.extras.homing import Homing


class ArmReached(Exception):
    """Sentinel proving trip_move armed the endstop instead of erroring."""


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


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


class FakeReactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


class FakePrinter:
    def get_reactor(self):
        return FakeReactor()


class FakeEngine:
    def motion_drained(self):
        return True


class FakeToolhead:
    def __init__(self, endstop):
        self.endstop = endstop

    def wait_moves(self):
        self.endstop.calls.append("wait_moves")
        self.endstop.queued_motion_pending = False

    def get_position(self):
        return [0.0, 0.0, 0.0, 0.0]


def run_trip_move(endstop):
    homing = Homing.__new__(Homing)
    homing.printer = FakePrinter()
    toolhead = FakeToolhead(endstop)
    entry = {"endstop": endstop, "provider": None}
    homing.trip_move(
        FakeGcmd(), toolhead, FakeEngine(), 2, -1, 5.0, 40.0, entry
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
