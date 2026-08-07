"""Endstop already pressed when G28 arms (issue #359 family).

An insta-trip carries a clock at or before the arm window and before any
recorded motion; the engine clamps its position to the run's start instead
of aborting, so the min_home_dist rehome path (or a plain seed) completes
the home — mainline parity for a carriage parked on its switch.
"""

from __future__ import annotations

import threading
import time

import pytest

from tools.sim import configs
from tools.sim.tests.test_homing_min_dist_corexy import (
    Y_ENDSTOP,
    EndstopWall,
    XyTracker,
    _needs_rehome_records,
)
from tools.sim.tests.test_homing_multi_endstop import AXES

pytestmark = pytest.mark.needs_elf

Z_LINE = 15
Z_STEPS_PER_MM = 16 * 200 / 4.0
PROBE_PIN = (0, 30)
BED_PHYS_MM = -3.0


def test_pressed_endstop_recovers_via_rehome(sim_world):
    world = sim_world(
        lambda w: configs.awd_corexy_positive_dir_config(
            w.h7_pty, str(w.gcode_dir)
        ),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    tracker = XyTracker(control)

    world.mark_log()
    y0 = tracker.xy()[1]
    with EndstopWall(tracker, control, 1, Y_ENDSTOP, wall_mm=y0 - 0.2):
        time.sleep(0.3)
        resp = world.gcode("G28 Y", timeout=300)

    records = _needs_rehome_records(world.log_tail(), "Y")
    assert records and records[0][0], (
        f"a switch pressed at arm must read as traveled~0 and take the"
        f" rehome path: {records} (G28: {resp})"
    )
    assert records[0][1] < 1.0, f"insta-trip traveled must be ~0: {records}"
    assert not resp.get("error"), (
        f"pressed-at-arm homing must recover via the rehome pass: {resp}"
    )
    assert world.shutdown_line() is None


def test_multi_endstop_pressed_at_arm_completes(sim_world):
    world = sim_world(
        lambda w: configs.dual_motor_xy_config(w.h7_pty, str(w.gcode_dir))
    )
    control = world.sim_control("h7")
    _, _, endstops, _ = AXES["X"]
    for chip, line in endstops:
        control.set_gpio_input(chip, line, 1)
    try:
        resp = world.gcode("G28 X", timeout=120)
    finally:
        for chip, line in endstops:
            control.set_gpio_input(chip, line, 0)
    assert not resp.get("error"), (
        f"both switches pressed at arm: the run must seed at the endstop"
        f" instead of aborting position-unknown: {resp}"
    )
    pos = world.toolhead_position()
    assert pos[0] == pytest.approx(
        configs.DUAL_MOTOR_POSITION_ENDSTOP, abs=0.5
    ), pos
    assert world.shutdown_line() is None


def _probe_cfg(world):
    cfg = configs.probe_config(world.h7_pty, str(world.gcode_dir), "virtual")
    cfg = cfg.replace("pin: gpiochip0/gpio202", "pin: gpiochip0/gpio30")
    cfg = cfg.replace(
        "[axis z]\nposition_min: -5",
        "[axis z]\nposition_min: -15\nmin_home_dist: 5",
    )
    return cfg


class ZProbe:
    """Probe switch fixed at BED_PHYS_MM of physical Z travel: active at or
    below the bed, open above it — a positional trigger, unlike the sim's
    50-step auto-endstop wall whose trip point moves with each approach."""

    def __init__(self, control):
        self.control = control
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _z_mm(self):
        resp = self.control.send(f"get_steps line={Z_LINE}")
        assert resp.startswith("steps="), resp
        return int(resp.split()[0].split("=")[1]) / Z_STEPS_PER_MM

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2)
        self.control.set_gpio_input(*PROBE_PIN, 0)

    def _run(self):
        level = None
        while not self._stop.is_set():
            want = 1 if self._z_mm() <= BED_PHYS_MM else 0
            if want != level:
                self.control.set_gpio_input(*PROBE_PIN, want)
                level = want
            time.sleep(0.005)


def test_probe_rehome_short_approach(sim_world):
    """Issue #359's reported flow: a short second Z home over a positional
    probe must pass through the min_home_dist backoff and re-approach."""
    world = sim_world(_probe_cfg, dual_mcu=False)
    control = world.sim_control("h7")

    with ZProbe(control):
        world.gcode_ok("G28", timeout=300)
        world.gcode_ok("G90", timeout=30)
        world.gcode_ok("G0 Z3.5 F600", timeout=120)
        world.gcode_ok("M400", timeout=120)
        world.mark_log()
        resp = world.gcode("G28 Z", timeout=300)

    log = world.log_tail()
    assert "needs rehome: True" in log, (
        f"2mm approach should take the rehome path; log={log[-2000:]}"
    )
    assert not resp.get("error"), (
        f"probe re-home failed on the re-approach: {resp}"
    )
    assert world.shutdown_line() is None
