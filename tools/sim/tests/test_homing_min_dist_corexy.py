"""min_home_dist early-trigger guard on positive-direction AWD CoreXY.

Each test emulates a physical endstop switch as a *positional wall*: a
thread watches the shim's per-lane step counters, converts them to
cartesian XY, and drives the endstop GPIO high exactly while the axis
sits at/past the wall — the same behavior a real switch (or a StallGuard
trip at a hard stop) has.

Scenario from the bench: sensorless AWD CoreXY homing toward
position_max. X starts near its endstop, so the first X approach trips
early and the guard correctly retracts and rehomes. Y then travels a
long way to its endstop — far more than min_home_dist — and the guard
must NOT classify that trip as early. (A single-lane corexy trip
reconstruction once measured such approaches as |d - x0 - y0| / 2 and
rehomed after every full-bed Y travel — fixed in f6b11496a; these tests
keep it fixed.)
"""

from __future__ import annotations

import re
import threading
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

# The runtime notifies the shim per axis step queue: lane A -> gpio18,
# lane B -> gpio7 (src/linux/runtime_tick_host.c step_gpio_lines).
LANE_A_LINE = 18
LANE_B_LINE = 7
STEPS_PER_MM = 16 * 200 / 40.0

X_ENDSTOP = (0, 10)
Y_ENDSTOP = (0, 11)

NEEDS_REHOME_RE = re.compile(
    r"homing: (?P<axis>[XYZ]) needs rehome: (?P<verdict>True|False) "
    r"\(traveled=(?P<traveled>[-0-9.]+) min_home_dist=[-0-9.]+\)"
)


def _needs_rehome_records(log_text: str, axis: str):
    return [
        (m.group("verdict") == "True", float(m.group("traveled")))
        for m in NEEDS_REHOME_RE.finditer(log_text)
        if m.group("axis") == axis
    ]


class XyTracker:
    """Physical cartesian XY (mm since boot) from the shim step counters."""

    def __init__(self, control):
        self.control = control

    def _lane_mm(self, line: int) -> float:
        resp = self.control.send(f"get_steps line={line}")
        if not resp.startswith("steps="):
            raise AssertionError(f"get_steps line={line}: {resp!r}")
        return int(resp.split()[0].split("=")[1]) / STEPS_PER_MM

    def xy(self) -> tuple:
        a = self._lane_mm(LANE_A_LINE)
        b = self._lane_mm(LANE_B_LINE)
        return ((a + b) / 2.0, (a - b) / 2.0)


class EndstopWall:
    """Drive an endstop GPIO like a switch mounted at `wall_mm` on one
    cartesian axis: high at/past the wall, low below it."""

    def __init__(self, tracker, control, axis_idx, pin, wall_mm):
        self.tracker = tracker
        self.control = control
        self.axis_idx = axis_idx
        self.pin = pin
        self.wall_mm = wall_mm
        self._stop = threading.Event()
        self._thread = None

    def __enter__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2)
        self.control.set_gpio_input(*self.pin, 0)

    def _run(self):
        level = None
        while not self._stop.is_set():
            pos = self.tracker.xy()[self.axis_idx]
            want = 1 if pos >= self.wall_mm else 0
            if want != level:
                self.control.set_gpio_input(*self.pin, want)
                level = want
            time.sleep(0.01)


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.awd_corexy_positive_dir_config(
            w.h7_pty, str(w.gcode_dir)
        ),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    return world, control, XyTracker(control)


def _home_y_against_wall_at_60(world, control, tracker):
    """G28 Y with the wall 60mm out; returns the guard's first decision.

    The G28 response is collected non-fatally: when the guard misfires
    the follow-up rehome can itself error out, and the decision record
    is the evidence this test is after.
    """
    with EndstopWall(tracker, control, 1, Y_ENDSTOP, wall_mm=60.0):
        resp = world.gcode("G28 Y", timeout=300)
    records = _needs_rehome_records(world.log_tail(), "Y")
    assert records, (
        f"no needs_rehome decision logged for Y; G28 response: {resp}"
    )
    early, traveled = records[0]
    assert abs(traveled - 60.0) < 3.0, (
        f"Y traveled 60mm physically but the guard measured {traveled:.3f}mm"
        f" (G28 response: {resp})"
    )
    assert not early, "a 60mm approach was classified as an early trigger"
    assert not resp.get("error"), resp


def test_long_travel_y_is_not_an_early_trigger(sim_world):
    world, control, tracker = _boot(sim_world)
    world.mark_log()
    _home_y_against_wall_at_60(world, control, tracker)
    assert world.shutdown_line() is None


def _wait_until(pred, timeout, what):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return
        time.sleep(0.02)
    raise AssertionError(f"timeout waiting for {what}")


def test_false_trigger_far_from_endstop_fails_loudly(sim_world):
    """A trigger ~0.5mm into the Y approach with no endstop anywhere near
    is a false trigger (StallGuard misfire, bad wiring). By design the
    guard retracts min_home_dist and probes only a 2*min_home_dist
    window: finding nothing there means the trigger lied, and the homing
    must fail loudly rather than silently accept a bogus origin."""
    world, control, tracker = _boot(sim_world)
    world.mark_log()

    resp = {}
    g28 = threading.Thread(
        target=lambda: resp.update(world.gcode("G28 Y", timeout=300))
    )
    y_start = tracker.xy()[1]
    g28.start()

    _wait_until(
        lambda: tracker.xy()[1] >= y_start + 0.5, 90, "Y approach motion"
    )
    control.set_gpio_input(*Y_ENDSTOP, 1)
    y_peak = tracker.xy()[1]

    def backoff_started():
        nonlocal y_peak
        y = tracker.xy()[1]
        y_peak = max(y_peak, y)
        return y <= y_peak - 0.5

    _wait_until(backoff_started, 60, "min_home_dist backoff after the trip")
    control.set_gpio_input(*Y_ENDSTOP, 0)

    g28.join(timeout=200)
    assert not g28.is_alive(), "G28 Y did not finish"

    records = _needs_rehome_records(world.log_tail(), "Y")
    assert records and records[0][0], (
        f"the staged false trigger should have tripped the guard: {records}"
    )
    assert records[0][1] < 5.0, f"false trip was not early: {records}"
    error = resp.get("error")
    assert error and "did not trigger within" in str(error), (
        f"a false trigger with no endstop in the re-approach window must "
        f"fail the homing loudly; G28 response: {resp}"
    )
    assert world.shutdown_line() is None


def test_long_travel_y_after_x_early_rehome(sim_world):
    world, control, tracker = _boot(sim_world)

    world.mark_log()
    with EndstopWall(tracker, control, 0, X_ENDSTOP, wall_mm=3.0):
        world.gcode_ok("G28 X", timeout=300)
    x_records = _needs_rehome_records(world.log_tail(), "X")
    assert x_records and x_records[0][0], (
        f"X was set up to trip 3mm from start (< min_home_dist) and should "
        f"have taken the rehome path; decisions: {x_records}"
    )

    _home_y_against_wall_at_60(world, control, tracker)
    assert world.shutdown_line() is None
