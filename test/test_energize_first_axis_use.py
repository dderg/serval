import pytest
from fakes import FakeKin, FakeMcu, FakePrinter, FakeRail, FakeStepper
from fakes.printer import FakeReactor

from klippy.extras.stepper_enable import (
    DISABLE_STALL_TIME,
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
    fence plans every buffered move to rest; a dwell drains then extends
    the timeline by stream seconds, which the MCU clock reads as
    `stream_to_print` print seconds each; an absolute hold keeps the next
    move from starting before a print time. The next move starts where the
    committed timeline ends unless the stream is parked too close to the
    playhead, in which case it re-anchors one motion lead ahead of it. The
    earliest-start claim answers with that start without touching the
    buffered moves."""

    def __init__(self, reactor, committed_end, stream_to_print=1.0):
        self._reactor = reactor
        self.committed_end = committed_end
        self.stream_to_print = stream_to_print
        self.held_until = float("-inf")
        self.rests = []
        self._next_id = 1

    def next_move_start(self):
        now = self._reactor.monotonic()
        if self.committed_end - now < PARKED_REANCHOR_FLOOR:
            natural = now + MOTION_LEAD
        else:
            natural = self.committed_end
        return max(natural, self.held_until)

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
        self.committed_end += delay * self.stream_to_print

    def dwell_until(self, print_time, mcu_handle):
        self.held_until = max(self.held_until, print_time)


class EnablePin:
    def __init__(self):
        self.scheduled = []

    def set_digital(self, print_time, value):
        self.scheduled.append((print_time, value))


def make_motion(
    committed_end,
    rails=(),
    follower_steppers=(),
    secondary_ahead_s=None,
    stream_to_print=1.0,
):
    reactor = FakeReactor(now=NOW)
    printer = FakePrinter(reactor=reactor)
    mcu = FakeMcu(printer=printer)
    motion = Motion.__new__(Motion)
    motion.printer = printer
    motion.reactor = reactor
    motion.mcu = mcu
    motion.all_mcus = [mcu]
    if secondary_ahead_s is not None:
        motion.all_mcus.append(
            FakeMcu(
                printer=printer,
                name="toolboard",
                print_time_offset=secondary_ahead_s,
            )
        )
    motion.motion_lead = MOTION_LEAD
    motion.kin = FakeKin(rails=list(rails))
    motion.follower_steppers = list(follower_steppers)
    motion.engine = StreamModelEngine(reactor, committed_end, stream_to_print)
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

    motion._fire_active_callbacks((5.0, 0.0, 0.0, 0.4))

    assert motion.engine.rests == [], (
        "energizing the extruder for its first move must not plan the"
        " buffered moves to rest"
    )
    assert pin.scheduled


def test_enable_keeps_the_schedule_floor_of_every_mcu():
    extruder = FakeStepper(name="extruder")
    _, pin = track(extruder)
    motion = make_motion(
        committed_end=NOW + 0.16,
        follower_steppers=[extruder],
        secondary_ahead_s=0.03,
    )
    floor = motion._schedule_floor()

    motion._fire_active_callbacks((0.0, 0.0, 0.0, 0.4))
    enabled_at = pin.scheduled[-1][0]
    move_start = motion.engine.next_move_start()

    assert floor <= enabled_at <= move_start, (
        "the enable must keep every MCU's scheduling lead and still land"
        " before the move: floor=%.3f enable=%.3f move_start=%.3f"
        % (floor, enabled_at, move_start)
    )
    assert motion.engine.rests == []


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
    motion._fire_active_callbacks((0.0, 0.0, 2.0, 0.0))
    enabled_at = pin.scheduled[-1][0]
    move_start = motion.engine.next_move_start()

    assert pin.scheduled[-1][1] == 1
    assert disabled_at <= enabled_at <= move_start, (
        "the re-enable must follow the disable on the pin and precede"
        " the move: disable=%.3f enable=%.3f move_start=%.3f"
        % (disabled_at, enabled_at, move_start)
    )


def test_motor_off_after_a_long_idle_holds_motion_past_the_disable():
    idle_s = 3000.0
    crystal_error = -50e-6
    z = FakeStepper(name="stepper_z")
    tracking, pin = track(z)
    motion = make_motion(
        committed_end=NOW - idle_s,
        rails=[FakeRail(steppers=[z])] * 3,
        stream_to_print=1.0 + crystal_error,
    )
    tracking.motor_enable(NOW + MOTION_LEAD)
    stepper_enable = PrinterStepperEnable.__new__(PrinterStepperEnable)
    stepper_enable.printer = motion.printer
    stepper_enable.enable_lines = {"stepper_z": tracking}

    stepper_enable.motor_off()
    disabled_at = pin.scheduled[-1][0]
    motion._fire_active_callbacks((0.0, 0.0, 2.0, 0.0))
    move_start = motion.engine.next_move_start()

    assert move_start >= disabled_at + DISABLE_STALL_TIME, (
        "motion after M84 must wait out the disable and its settle on the"
        " MCU clock: disable=%.3f move_start=%.3f" % (disabled_at, move_start)
    )


def test_enable_scheduled_before_the_pending_disable_fails_loudly():
    pin = EnablePin()
    enable = StepperEnablePin(pin, 0)
    enable.set_enable(NOW + 1.0)
    enable.set_disable(NOW + 2.0)
    with pytest.raises(CommandError, match="before its pending disable"):
        enable.set_enable(NOW + 1.5)


def test_enable_that_cannot_precede_its_move_fails_loudly():
    pin = EnablePin()
    tracking = EnableTracking(
        FakeStepper(name="stepper_z"), StepperEnablePin(pin, 0)
    )
    tracking.enable.set_enable(NOW + 1.0)
    tracking.enable.set_disable(NOW + 2.0)
    with pytest.raises(CommandError, match="before its move"):
        tracking.energize(NOW + 0.5, NOW + 1.5)
