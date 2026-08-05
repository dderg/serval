import types

import pytest
from fakes import FakeEngine, FakeGcmd, FakePrinter, FakeReactor
from fakes import FakeToolhead as _FakeToolhead

from klippy.extras import homing as homing_mod
from klippy.extras.homing import Homing


@pytest.fixture
def patched_danger(monkeypatch):
    monkeypatch.setattr(
        homing_mod,
        "get_danger_options",
        lambda: types.SimpleNamespace(homing_trip_deadline_margin=5.0),
    )


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


class DisarmTrackingEndstop:
    def __init__(self, name, arm_error=None):
        self.name = name
        self.endstop_id = 0
        self.arm_error = arm_error
        self.armed = False
        self.disarms = 0

    def engine_mcu_handle(self):
        return 1

    def is_triggered(self):
        return False

    def query_trip_state(self):
        return {"tripped": True, "trip_clock": 11}

    def arm(self, poll_period):
        if self.arm_error is not None:
            raise self.arm_error
        self.armed = True

    def disarm(self):
        self.disarms += 1


def run_trip_move_with(endstops, engine):
    homing = Homing.__new__(Homing)
    homing.printer = FakePrinter(reactor=FakeReactor())
    toolhead = _FakeToolhead(position=[0.0, 0.0, 0.0, 0.0])
    entry = {"endstops": list(endstops), "provider": None}
    return homing.trip_move(
        FakeGcmd(error=RuntimeError),
        toolhead,
        engine,
        2,
        -1,
        5.0,
        40.0,
        entry,
    )


class TripEngine:
    def __init__(self, poll_result=None):
        self._poll_result = poll_result
        self.calls = []

    def motion_drained(self):
        return True

    def home_axis_start(self, axis, direction, speed, max_travel, endstops):
        self.calls.append("home_axis_start")

    def home_axis_poll(self):
        return self._poll_result

    def home_abort(self):
        return [0.0, 0.0, 0.0]


def test_every_endstop_disarms_after_a_successful_trip(patched_danger):
    endstops = [DisarmTrackingEndstop("a"), DisarmTrackingEndstop("b")]
    engine = TripEngine(((0.0, 0.0, 1.0), (0.0, 0.0, 0.9), 11))
    trip_pos, final_pos = run_trip_move_with(endstops, engine)
    assert (trip_pos, final_pos) == ((0.0, 0.0, 1.0), (0.0, 0.0, 0.9))
    assert [e.disarms for e in endstops] == [1, 1]


def test_every_endstop_disarms_when_one_arm_raises(patched_danger):
    endstops = [
        DisarmTrackingEndstop("a"),
        DisarmTrackingEndstop("b", arm_error=ArmReached()),
        DisarmTrackingEndstop("c"),
    ]
    with pytest.raises(ArmReached):
        run_trip_move_with(endstops, TripEngine())
    assert [e.disarms for e in endstops] == [1, 1, 1]
