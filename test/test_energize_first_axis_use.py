import pytest
from fakes import FakeKin, FakeMcu, FakePrinter, FakeRail, FakeStepper
from fakes.printer import FakeReactor

from klippy.extras.stepper_enable import (
    EnableTracking,
    PrinterStepperEnable,
    StepperEnablePin,
)
from klippy.gcode import CommandError
from klippy.motion import Motion

NOW = 1000.0
MOTION_LEAD = 0.25
PARKED_REANCHOR_FLOOR = 0.150


class StreamModelEngine:
    """The engine's motion-stream contract as Motion consumes it. A forcing
    fence plans every buffered move to rest; a dwell drains then holds the
    timeline; the next move starts where the committed timeline ends unless
    the stream is parked too close to the playhead, in which case it
    re-anchors one motion lead ahead of it. The earliest-start claim answers
    with that start without touching the buffered moves."""

    def __init__(self, reactor, committed_end):
        self._reactor = reactor
        self.committed_end = committed_end
        self.rests = []
        self._next_id = 1

    def next_move_start(self):
        now = self._reactor.monotonic()
        if self.committed_end - now < PARKED_REANCHOR_FLOOR:
            return now + MOTION_LEAD
        return self.committed_end

    def _issue_id(self):
        issued = self._next_id
        self._next_id += 1
        return issued

    def fence_start(self, force):
        if force:
            self.rests.append(self.committed_end)
        return self._issue_id()

    def fence_print_time_poll(self, fence_id, mcu_handle):
        return self.committed_end

    def claim_earliest_start(self):
        return self._issue_id()

    def earliest_start_print_time_poll(self, claim_id, mcu_handle):
        return self.next_move_start()

    def frontier_print_time(self, mcu_handle):
        return self.committed_end

    def submit_dwell(self, delay):
        self.rests.append(self.committed_end)
        self.committed_end += delay


class EnablePin:
    def __init__(self):
        self.scheduled = []

    def set_digital(self, print_time, value):
        self.scheduled.append((print_time, value))


def make_motion(committed_end, rails=(), follower_steppers=()):
    reactor = FakeReactor(now=NOW)
    printer = FakePrinter(reactor=reactor)
    mcu = FakeMcu(printer=printer)
    motion = Motion.__new__(Motion)
    motion.printer = printer
    motion.reactor = reactor
    motion.mcu = mcu
    motion.all_mcus = [mcu]
    motion.motion_lead = MOTION_LEAD
    motion.kin = FakeKin(rails=list(rails))
    motion.follower_steppers = list(follower_steppers)
    motion.engine = StreamModelEngine(reactor, committed_end)
    motion._engine_wakeup = None
    motion._planner_ready = True
    printer.add_object("toolhead", motion)
    return motion


def track(stepper):
    pin = EnablePin()
    return EnableTracking(stepper, StepperEnablePin(pin, 0)), pin


def test_first_extruder_move_keeps_the_toolhead_moving():
    extruder = FakeStepper(name="extruder")
    _, pin = track(extruder)
    motion = make_motion(
        committed_end=NOW + 1.5,
        rails=[FakeRail(steppers=[FakeStepper(name="stepper_x")])],
        follower_steppers=[extruder],
    )
    move_start = motion.engine.next_move_start()

    motion._fire_active_callbacks((5.0, 0.0, 0.0, 0.4))

    assert motion.engine.rests == [], (
        "energizing the extruder for its first move must not plan the"
        " buffered moves to rest"
    )
    assert pin.scheduled and pin.scheduled[0][0] <= move_start, (
        "the extruder must be enabled no later than the move that"
        " steps it starts"
    )


def test_move_right_after_motor_off_energizes_between_disable_and_start():
    z = FakeStepper(name="stepper_z")
    tracking, pin = track(z)
    motion = make_motion(committed_end=NOW, rails=[FakeRail(steppers=[z])] * 3)
    tracking.motor_enable(NOW + MOTION_LEAD)
    stepper_enable = PrinterStepperEnable.__new__(PrinterStepperEnable)
    stepper_enable.printer = motion.printer
    stepper_enable.enable_lines = {"stepper_z": tracking}

    stepper_enable.motor_off()
    disabled_at = pin.scheduled[-1][0]
    move_start = motion.engine.next_move_start()
    motion._fire_active_callbacks((0.0, 0.0, 2.0, 0.0))
    enabled_at = pin.scheduled[-1][0]

    assert pin.scheduled[-1][1] == 1
    assert disabled_at <= enabled_at <= move_start, (
        "the re-enable must follow the disable on the pin and precede"
        " the move: disable=%.3f enable=%.3f move_start=%.3f"
        % (disabled_at, enabled_at, move_start)
    )


def test_enable_scheduled_before_the_pending_disable_fails_loudly():
    pin = EnablePin()
    enable = StepperEnablePin(pin, 0)
    enable.set_enable(NOW + 1.0)
    enable.set_disable(NOW + 2.0)
    with pytest.raises(CommandError, match="before its pending disable"):
        enable.set_enable(NOW + 1.5)
