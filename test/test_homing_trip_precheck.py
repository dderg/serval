import pytest

from klippy.extras.homing import Homing


class ArmReached(Exception):
    """Sentinel proving trip_move got past the already-triggered check."""


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


class FakeEndstop:
    def __init__(self, triggered_after_wait):
        self.queued_motion_pending = True
        self.triggered_after_wait = triggered_after_wait
        self.calls = []

    def bridge_mcu_handle(self):
        return object()

    def is_triggered(self):
        self.calls.append("is_triggered")
        if self.queued_motion_pending:
            return True
        return self.triggered_after_wait

    def arm(self, poll_period):
        raise ArmReached()


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
    toolhead = FakeToolhead(endstop)
    entry = {"endstop": endstop, "provider": None}
    homing.trip_move(FakeGcmd(), toolhead, object(), 2, -1, 5.0, 40.0, entry)


def test_pin_triggered_only_while_moves_queued_does_not_error():
    # After Z homes on the probe, the head sits at the trigger point while
    # the lift move is still queued — the live pin must be sampled only
    # after wait_moves, when the lift has physically completed.
    endstop = FakeEndstop(triggered_after_wait=False)
    with pytest.raises(ArmReached):
        run_trip_move(endstop)
    assert endstop.calls.index("wait_moves") < endstop.calls.index(
        "is_triggered"
    )


def test_pin_still_triggered_after_wait_moves_errors():
    endstop = FakeEndstop(triggered_after_wait=True)
    with pytest.raises(RuntimeError, match="already triggered"):
        run_trip_move(endstop)
    assert endstop.calls.index("wait_moves") < endstop.calls.index(
        "is_triggered"
    )
