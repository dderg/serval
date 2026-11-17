"""Classic-Klipper SET_PRESSURE_ADVANCE shim against real firmware."""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _extruder_status(world):
    return world.status({"extruder": None})["extruder"]


def test_set_pressure_advance_drives_post_processors(sim_world):
    world = sim_world(
        lambda w: configs.with_extruder_smoothing(
            configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)), 0.04
        ),
        dual_mcu=False,
    )
    status = _extruder_status(world)
    assert status["pressure_advance"] == pytest.approx(0.03)
    assert status["smooth_time"] == pytest.approx(0.04)

    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.05")
    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder SMOOTH_TIME=0.02")
    status = _extruder_status(world)
    assert status["pressure_advance"] == pytest.approx(0.05)
    assert status["smooth_time"] == pytest.approx(0.02)

    world.gcode_ok("SET_POST_PROCESSOR NAME=pa K=0.07")
    assert _extruder_status(world)["pressure_advance"] == pytest.approx(0.07)

    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder")
    assert world.wait_for_log_text("pressure_advance: 0.070000", timeout=10)
    assert world.shutdown_line() is None


def test_set_pressure_advance_reports_disabled_without_processors(sim_world):
    def config(w):
        cfg = configs.neptune_print_config(w.h7_pty, str(w.gcode_dir))
        assert "post_processors: pa\n" in cfg
        return cfg.replace("post_processors: pa\n", "")

    world = sim_world(config, dual_mcu=False)
    status = _extruder_status(world)
    assert "pressure_advance" not in status
    assert "smooth_time" not in status

    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.05")
    assert world.wait_for_log_text("cannot set ADVANCE", timeout=10)
    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder SMOOTH_TIME=0.02")
    assert world.wait_for_log_text("cannot set SMOOTH_TIME", timeout=10)
    assert world.shutdown_line() is None


REST_AFTER_SET_PA_S = 1.0


def test_a_move_long_after_set_pressure_advance_resumes_from_rest(sim_world):
    world = sim_world(
        lambda w: configs.with_extruder_smoothing(
            configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)), 0.04
        ),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
    world.gcode_ok("G1 X110 F3000")
    world.gcode_ok("M400")
    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.05")
    rested_ns = world.vclock.now_ns() + int(REST_AFTER_SET_PA_S * 1e9)
    world.wait_until(
        lambda: world.vclock.now_ns() >= rested_ns,
        timeout=10.0,
        what="a rest well past the advance change",
    )
    world.gcode_ok("G1 X100 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None
