"""Consecutive homes on a CoreXY step/dir (stepcompress) world.

On CoreXY a single-axis home moves BOTH belt motors, so G28 X halts A and
B mid-move and reseeds their step counters from the trip. G28 Y then trips
those same already-halted motors a second time. That second trip is where
the Voron 0 bench saw `stepcompress trip reconcile diverged` on the A
motor; the cartesian stepcompress world never re-trips a previously halted
motor, so it cannot catch it.

The world and the G-code below mirror the bench exactly: its printer.cfg
topology (see configs.stepcompress_corexy_config) and its failing sequence
SET_KINEMATIC_POSITION X=60 Y=60 Z=20 / G28 X / G28 Y.
"""

from __future__ import annotations

import threading
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = configs.STEPCOMPRESS_COREXY_STEPS_PER_MM
START_XY = (60.0, 60.0)
WALLS_MM = {"x": 121.0, "y": 120.0}
MICROSTEP_MM = 1.0 / STEPS_PER_MM


class XyTracker:
    """Machine cartesian XY from the shim's per-lane step counters, which
    count real GPIO pulses since boot and are never reseeded."""

    def __init__(self, control, origin):
        self.control = control
        self.origin = origin

    def _lane_mm(self, line: int) -> float:
        resp = self.control.send(f"get_steps line={line}")
        if not resp.startswith("steps="):
            raise AssertionError(f"get_steps line={line}: {resp!r}")
        return int(resp.split()[0].split("=")[1]) / STEPS_PER_MM

    def xy(self) -> tuple:
        a = self._lane_mm(configs.STEPCOMPRESS_COREXY_STEP_LINES["a"])
        b = self._lane_mm(configs.STEPCOMPRESS_COREXY_STEP_LINES["b"])
        return (self.origin[0] + (a + b) / 2.0, self.origin[1] + (a - b) / 2.0)


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
            time.sleep(0.005)


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.stepcompress_corexy_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )
    control = world.sim_control("f4")
    return world, control, XyTracker(control, START_XY)


def _home(world, axis: str):
    resp = world.gcode(f"G28 {axis}", timeout=300)
    assert not resp.get("error"), (
        f"G28 {axis} failed: {resp.get('error')}\n{world.log_tail()}"
    )


def test_stepcompress_corexy_consecutive_homes(sim_world):
    world, control, tracker = _boot(sim_world)
    world.gcode_ok(
        "SET_KINEMATIC_POSITION X=%g Y=%g Z=20" % START_XY, timeout=60
    )
    world.gcode_ok("M400", timeout=60)

    walls = [
        EndstopWall(
            tracker,
            control,
            axis_idx,
            configs.STEPCOMPRESS_COREXY_ENDSTOPS[axis],
            WALLS_MM[axis],
        )
        for axis_idx, axis in enumerate(("x", "y"))
    ]
    with walls[0], walls[1]:
        world.gcode_ok("G4 P1000", timeout=60)
        _home(world, "X")
        world.gcode_ok("G4 P2000", timeout=60)
        _home(world, "Y")
        world.gcode_ok("G4 P2000", timeout=60)
        _home(world, "X")

    assert world.shutdown_line() is None, world.log_tail()
    homed = world.status()["toolhead"]["homed_axes"].lower()
    assert "x" in homed and "y" in homed, homed
