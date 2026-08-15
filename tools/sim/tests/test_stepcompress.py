"""End-to-end coverage for stepping_mode: stepcompress.

The steppers live on the second MCU, built with CONFIG_CLASSIC_STEPPING=y
(tools/sim/configs/sc-sim.config), so the host computes every step time and
ships it over queue_step / set_next_step_dir / reset_step_clock. The piece
-mode case runs in the same file so a stepcompress regression cannot hide
behind a piece-mode pass.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = configs.STEPCOMPRESS_STEPS_PER_MM


def _steps(world, axis: str) -> int:
    line = configs.STEPCOMPRESS_STEP_LINES[axis]
    resp = world.sim_control("f4").send(f"get_steps line={line}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={line}: {resp!r}")
    return int(resp.split()[0].split("=")[1])


def _boot(sim_world):
    return sim_world(
        lambda w: configs.stepcompress_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )


def test_stepcompress_move_completes(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    pos = world.toolhead_position()
    assert pos[0] == pytest.approx(135.0, abs=0.01), pos


def test_stepcompress_step_count_matches_distance(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M400")
    world.sim_control("f4").reset_step_times(
        configs.STEPCOMPRESS_STEP_LINES["x"]
    )
    world.gcode_ok("G1 X145 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    expected = 20.0 * STEPS_PER_MM
    assert _steps(world, "x") == pytest.approx(expected, abs=1.0)


def test_stepcompress_homing_trips(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G28 Z", timeout=120)
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert world.status()["toolhead"]["homed_axes"].lower().find("z") >= 0
    events = world.events_text()
    assert "endstop.trsync_do_trigger" in events, (
        "the trip never reached the mcu-side trsync — the step queues were"
        " cleared by the host's Stop instead"
    )


def test_piece_mode_regression_beside_stepcompress(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert world.toolhead_position()[0] == pytest.approx(135.0, abs=0.01)


SIM_CLOCK_HZ = 50_000_000
ENCODER_WINDOW_SECONDS = (3 << 28) / SIM_CLOCK_HZ


def test_stepcompress_z_survives_an_idle_longer_than_the_encoder_window(
    sim_world,
):
    """Z holds while X/Y print, then moves — all in one motion epoch.

    The host encodes every step as an offset from the clock the mcu stepper
    is anchored on, and that offset tops out at 3<<28 ticks (~16.1 s at the
    sim's 50 MHz). A lane held past that has to be re-anchored with
    reset_step_clock mid-stream; without it the pump dies on the first step
    of the resuming move. The epoch must not be broken by an M400 before
    the Z move — a fresh epoch re-anchors every lane anyway.
    """
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 Z124 F300")
    world.gcode_ok("M400", timeout=60)
    parked_at = _steps(world, "z")
    held_from = world.status()["toolhead"]["estimated_print_time"]

    for _ in range(12):
        world.gcode_ok("G1 X135 Y135 F600")
        world.gcode_ok("G1 X125 Y125 F600")
    world.gcode_ok("G1 Z120 F300")
    world.gcode_ok("M400", timeout=180)

    held_for = world.status()["toolhead"]["estimated_print_time"] - held_from
    assert held_for > ENCODER_WINDOW_SECONDS, (
        f"the xy print only held z for {held_for:.1f}s of mcu time, inside"
        f" the {ENCODER_WINDOW_SECONDS:.1f}s encoder window — the test would"
        " pass without exercising the re-anchor"
    )
    assert world.shutdown_line() is None, world.log_tail()
    assert _steps(world, "z") - parked_at == pytest.approx(
        4.0 * STEPS_PER_MM, abs=1.0
    )
    assert world.toolhead_position()[2] == pytest.approx(120.0, abs=0.01)
