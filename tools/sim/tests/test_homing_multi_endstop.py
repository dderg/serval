"""Dual-motor axes with one endstop switch per motor.

X and Y each drive two motors on a single kinematic lane, each motor
carrying its own switch (keyed `endstop_pin` form). The first switch to
trip must only freeze its own motor -- the lane keeps streaming steps,
the peer motor keeps pulsing its step pin -- and only the last trip of
the group ends the homing move and seeds the axis to position_endstop.

Suppression is proven off the physical step-pin GPIO edge counters of
each motor, not off the synthetic lane counter: a lane that keeps
streaming says nothing about which stepper the mcu actually silenced.
Driving those pins is off in every other sim world -- an ioctl per edge
distorts timing -- so _boot arms it through the shim's `set_step_emit`
control verb before any motion.

Every window is measured against the lane counter's own progress, never
against wall-clock: the simulated world runs on a virtual clock that
stalls under host load, so a fixed real-time window can catch a span in
which nothing moved at all and read like a frozen motor.

homing_retract_dist is 0 in the config, so each G28 is a single approach
and the switches can be driven by hand from the test.
"""

from __future__ import annotations

import threading
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

# Runtime step-queue notification lines (src/linux/runtime_tick_host.c
# step_gpio_lines): cartesian X lane -> gpio18, Y lane -> gpio7.
X_LANE_LINE = 18
Y_LANE_LINE = 7

AXES = {
    "X": (
        0,
        X_LANE_LINE,
        configs.DUAL_MOTOR_X_ENDSTOPS,
        configs.DUAL_MOTOR_X_STEP_PINS,
    ),
    "Y": (
        1,
        Y_LANE_LINE,
        configs.DUAL_MOTOR_Y_ENDSTOPS,
        configs.DUAL_MOTOR_Y_STEP_PINS,
    ),
}

STEPS_PER_MM = (
    configs.DUAL_MOTOR_MICROSTEPS * 200 / configs.DUAL_MOTOR_ROTATION_DISTANCE
)
MOVING_STEPS = int(0.5 * STEPS_PER_MM)
# Steps already pulsed between the switch closing and the mcu applying the
# suppress bit: one endstop poll period (1 ms) of travel at homing speed,
# with margin. Drained from the lane before any suppression baseline.
SUPPRESSED_EDGE_SLOP = 4
SUPPRESS_SETTLE_STEPS = 8
# A running motor pulses at least one edge per lane step; the lane and the
# edge counters are read in separate round trips, so allow the same slop.
RUNNING_EDGE_MIN = MOVING_STEPS - SUPPRESSED_EDGE_SLOP
# Deceleration after the final Stop is ~40 steps at homing speed, so a bound
# this tight separates "still suppressed through the stop" from "the mask was
# dropped and the queued lane steps reached the motor".
POST_STOP_EDGE_SLOP = 8
LANE_PROGRESS_TIMEOUT_S = 60.0
HOME_TIMEOUT_S = 300.0
POSITION_TOLERANCE_MM = 0.1
MAX_STOP_OVERSHOOT_MM = 1.0
ABORT_SPEED_MM_S = 10.0
ABORT_MAX_TRAVEL_MM = 30.0


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.dual_motor_xy_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    control.enable_step_pin_emit()
    return world, control


def _lane_steps(control, line: int) -> int:
    resp = control.send(f"get_steps line={line}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={line}: {resp!r}")
    return int(resp.split()[0].split("=", 1)[1])


def _edges(control, pins) -> list[int]:
    return [control.gpio_edges(chip, line) for chip, line in pins]


def _sample_while_lane_advances(control, pins, line, lane_target):
    """Step-pin edges accumulated over exactly the span in which the lane
    counter advances `lane_target` steps. Returns (lane_advanced, edges)
    so a caller can tell "this motor is frozen" apart from "the whole
    simulated world stopped moving"."""
    lane_start = _lane_steps(control, line)
    start = _edges(control, pins)
    lane_advanced = 0
    advanced = [0] * len(pins)
    deadline = time.monotonic() + LANE_PROGRESS_TIMEOUT_S
    while time.monotonic() < deadline:
        lane_advanced = abs(_lane_steps(control, line) - lane_start)
        advanced = [
            value - base for value, base in zip(_edges(control, pins), start)
        ]
        if lane_advanced >= lane_target:
            break
        time.sleep(0.01)
    return lane_advanced, advanced


def _wait_for_edges(control, pins, start, minimum, timeout=60.0) -> list[int]:
    advanced = [0] * len(pins)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        advanced = [
            value - base for value, base in zip(_edges(control, pins), start)
        ]
        if all(a >= minimum for a in advanced):
            break
        time.sleep(0.01)
    return advanced


def _wait_until_moving(control, line: int, timeout: float = 60.0) -> None:
    start = _lane_steps(control, line)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if abs(_lane_steps(control, line) - start) >= MOVING_STEPS:
            return
        time.sleep(0.01)
    raise AssertionError(f"lane on line {line} never started stepping")


class _GcodeThread(threading.Thread):
    def __init__(self, world, script: str):
        super().__init__(daemon=True)
        self.world = world
        self.script = script
        self.response = None
        self.failure = None

    def run(self):
        try:
            self.response = self.world.gcode(
                self.script, timeout=HOME_TIMEOUT_S
            )
        except Exception as exc:
            self.failure = exc


def _assert_split_after_trip(
    label, switch, lane_advanced, bound_pin, bound, peer_pin, peer
):
    report = (
        f"lane advanced {lane_advanced} steps, bound motor {bound_pin}"
        f" {bound} edges, peer motor {peer_pin} {peer} edges"
    )
    assert lane_advanced >= MOVING_STEPS, (
        f"{label}: the lane stopped streaming after only switch {switch}"
        f" of the group tripped -- the run must continue until the last"
        f" switch ({report})"
    )
    assert bound <= SUPPRESSED_EDGE_SLOP, (
        f"{label}: the motor carrying switch {switch} kept pulsing step pin"
        f" {bound_pin} after its own endstop tripped -- the mcu did not"
        f" suppress that stepper ({report})"
    )
    assert peer >= RUNNING_EDGE_MIN, (
        f"{label}: peer motor step pin {peer_pin} stopped when switch"
        f" {switch} tripped -- the wrong stepper was suppressed, or the"
        f" suppression silenced the whole motor ({report})"
    )


def _assert_both_motors_step(world, control, step_pins, script, label):
    start = _edges(control, step_pins)
    world.gcode_ok(script, timeout=120)
    world.gcode_ok("M400", timeout=120)
    advanced = _wait_for_edges(control, step_pins, start, MOVING_STEPS)
    assert all(a >= MOVING_STEPS for a in advanced), (
        f"{label}: '{script}' pulsed {advanced} step-pin edges on"
        f" {list(step_pins)}; both motors must step again once no endstop"
        f" is armed (a leaked suppress mask silences one of them)"
    )


def _assert_seeded_to_endstop(world, axis_index: int, label: str) -> None:
    position = world.status({"toolhead": None})["toolhead"]["position"]
    seeded = position[axis_index]
    endstop = configs.DUAL_MOTOR_POSITION_ENDSTOP
    overshoot = seeded - endstop
    assert -MAX_STOP_OVERSHOOT_MM < overshoot <= POSITION_TOLERANCE_MM, (
        f"{label} seeded to {seeded}: expected position_endstop {endstop}"
        f" minus a bounded stop-latency overshoot"
        f" (homing direction is negative, retract is 0)"
    )


def _home_with_staggered_trips(world, control, axis: str, first_index: int):
    axis_index, lane_line, endstops, step_pins = AXES[axis]
    first_switch, last_switch = endstops[first_index], endstops[1 - first_index]
    bound_pin, peer_pin = step_pins[first_index], step_pins[1 - first_index]
    watched = (bound_pin, peer_pin)

    homing = _GcodeThread(world, f"G28 {axis}")
    homing.start()
    try:
        _wait_until_moving(control, lane_line)
        lane_pre, (bound_pre, peer_pre) = _sample_while_lane_advances(
            control, watched, lane_line, MOVING_STEPS
        )
        assert bound_pre >= RUNNING_EDGE_MIN and peer_pre >= RUNNING_EDGE_MIN, (
            f"{axis} approach must pulse both motors before any trip: lane"
            f" advanced {lane_pre} steps, {bound_pin} advanced {bound_pre}"
            f" edges, {peer_pin} advanced {peer_pre}"
        )

        control.set_gpio_input(*first_switch, 1)
        _sample_while_lane_advances(
            control, watched, lane_line, SUPPRESS_SETTLE_STEPS
        )
        lane_post, (bound_post, peer_post) = _sample_while_lane_advances(
            control, watched, lane_line, MOVING_STEPS
        )
        _assert_split_after_trip(
            f"G28 {axis}",
            first_switch,
            lane_post,
            bound_pin,
            bound_post,
            peer_pin,
            peer_post,
        )
        assert homing.is_alive(), (
            f"G28 {axis} finished on the first endstop trip; the second"
            " switch was never touched"
        )

        bound_at_last_trip = _edges(control, (bound_pin,))[0]
        control.set_gpio_input(*last_switch, 1)
        homing.join(timeout=HOME_TIMEOUT_S)
        bound_through_stop = _edges(control, (bound_pin,))[0]
    finally:
        control.set_gpio_input(*first_switch, 0)
        control.set_gpio_input(*last_switch, 0)

    assert not homing.is_alive(), f"G28 {axis} never returned"
    assert homing.failure is None, homing.failure
    assert homing.response and not homing.response.get("error"), homing.response
    assert world.shutdown_line() is None, world.log_tail()
    assert bound_through_stop - bound_at_last_trip <= POST_STOP_EDGE_SLOP, (
        f"{axis} motor on switch {first_switch} pulsed step pin {bound_pin}"
        f" {bound_through_stop - bound_at_last_trip} times while the lane"
        f" decelerated after the final Stop: its suppress bit must survive"
        f" until the endstop is disarmed"
    )
    _assert_seeded_to_endstop(world, axis_index, axis)
    _assert_both_motors_step(
        world,
        control,
        step_pins,
        f"G1 {axis}10 F600",
        f"{axis} after homing on switch order {first_switch}",
    )


def test_boot_and_query_reports_every_switch(sim_world):
    world, _control = _boot(sim_world)
    assert world.shutdown_line() is None, world.log_tail()

    world.gcode_ok("QUERY_ENDSTOPS")
    names = world.status({"query_endstops": None})["query_endstops"][
        "last_query"
    ]
    motors = configs.DUAL_MOTOR_X_MOTORS + configs.DUAL_MOTOR_Y_MOTORS
    for motor in motors:
        matches = [name for name in names if motor in name]
        assert len(matches) == 1, (
            f"expected exactly one endstop entry for motor {motor},"
            f" QUERY_ENDSTOPS reported {sorted(names)}"
        )
    assert any(name.startswith("z") for name in names), (
        f"the single-switch Z axis vanished from QUERY_ENDSTOPS: "
        f"{sorted(names)}"
    )


@pytest.mark.parametrize("axis", ["X", "Y"])
@pytest.mark.parametrize("first_index", [0, 1])
def test_staggered_trip_suppresses_only_the_tripped_motor(
    sim_world, axis, first_index
):
    """Both trip orders on both axes: whichever switch closes first, only
    its own motor stops pulsing. Suppressing the peer bit instead trips the
    peer assertion, so a swapped bit mapping cannot pass on both orders."""
    world, control = _boot(sim_world)
    _home_with_staggered_trips(world, control, axis, first_index)


def test_abort_after_first_trip_leaves_no_suppression(sim_world):
    """One switch of the pair never closes. The approach runs out of travel
    and the host fails loudly -- naming the sibling it is still waiting on --
    after trip_move's unwind has disarmed every endstop of the group. That
    disarm is what clears the first motor's suppress bit; if it leaks, the
    motor stays silent for the rest of the session."""
    world, control = _boot(sim_world)
    _axis_index, lane_line, endstops, step_pins = AXES["X"]
    first_switch = endstops[0]
    bound_pin, peer_pin = step_pins[0], step_pins[1]

    homing = _GcodeThread(
        world,
        f"_HOME_TEST AXIS=X SPEED={ABORT_SPEED_MM_S}"
        f" MAX_TRAVEL={ABORT_MAX_TRAVEL_MM}",
    )
    homing.start()
    try:
        _wait_until_moving(control, lane_line)
        control.set_gpio_input(*first_switch, 1)
        _sample_while_lane_advances(
            control, (bound_pin, peer_pin), lane_line, SUPPRESS_SETTLE_STEPS
        )
        lane_post, (bound_post, peer_post) = _sample_while_lane_advances(
            control, (bound_pin, peer_pin), lane_line, MOVING_STEPS
        )
        _assert_split_after_trip(
            "_HOME_TEST AXIS=X",
            first_switch,
            lane_post,
            bound_pin,
            bound_post,
            peer_pin,
            peer_post,
        )
        homing.join(timeout=HOME_TIMEOUT_S)
    finally:
        control.set_gpio_input(*first_switch, 0)

    assert not homing.is_alive(), "_HOME_TEST AXIS=X never returned"
    assert homing.failure is None, homing.failure
    assert homing.response and homing.response.get("error"), (
        "a trip move whose second switch never closed must fail:"
        f" {homing.response}"
    )
    error_text = str(homing.response["error"])
    assert (
        configs.DUAL_MOTOR_X_MOTORS[1] in error_text
        or "did not trigger" in error_text
    ), (
        "the unwind must name the sibling endstop it never heard from, or"
        f" report the exhausted approach: {homing.response}"
    )
    assert world.shutdown_line() is None, world.log_tail()

    world.gcode_ok("SET_KINEMATIC_POSITION X=50 Y=50 Z=50", timeout=60)
    _assert_both_motors_step(
        world, control, step_pins, "G1 X40 F600", "X after an aborted trip"
    )
