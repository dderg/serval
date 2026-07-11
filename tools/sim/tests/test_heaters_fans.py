"""Heater/fan/pwm host-logic flows, ported from the legacy batch suite
(test/klippy/*.test) which could not run on this fork: batch/debugoutput
mode died with the serialqueue transport, and its AVR dictionaries are not
buildable here. These run the same command sequences against the real
firmware in the sim — including the G28/extrude preambles the batch
harness could only pretend to execute.

The originals asserted nothing beyond a crash-free run; these do the
same (plus an explicit no-shutdown check), so the coverage is boot +
command dispatch + heater/fan control-loop math, not physics.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _cfg(world):
    return configs.heaters_config(world.h7_pty, str(world.gcode_dir))


def _cfg_mpc(world):
    return configs.heaters_config(
        world.h7_pty, str(world.gcode_dir), control="mpc"
    )


def _extrude_preamble(world):
    # min_extrude_temp is 0, so cold E-only and mixed moves are legal.
    world.gcode_ok("G1 E5")
    world.gcode_ok("G1 E-2")
    world.gcode_ok("G1 E7")
    world.gcode_ok("G28", timeout=180)
    world.gcode_ok("G1 X20 Y20 Z1")
    world.gcode_ok("G1 X25 Y25 E7.5")
    world.gcode_ok("M400", timeout=60)


def test_pwm_pins(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    for value in ("0", "0.5", "0.5", "0.25", "1"):
        world.gcode_ok(f"SET_PIN PIN=test_pwm_tool VALUE={value}")
    for value in ("0", "0.5", "1"):
        world.gcode_ok(f"SET_PIN PIN=soft_pwm_pin VALUE={value}")
    for value, cycle in (
        ("0", None),
        ("0.5", None),
        ("1", None),
        ("0", "0.1"),
        ("1", "0.5"),
        ("0.5", "0.001"),
        ("0.75", "0.01"),
        ("0.5", "1"),
        ("0.5", "0.5"),
        ("0.5", "0.5"),
        ("0.75", "0.5"),
        ("0.75", "0.75"),
    ):
        cmd = f"SET_PIN PIN=cycle_pwm_pin VALUE={value}"
        if cycle is not None:
            cmd += f" CYCLE_TIME={cycle}"
        world.gcode_ok(cmd)
    assert world.shutdown_line() is None


def test_fans_and_heated_fan(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    _extrude_preamble(world)
    # fan_pwm_scaling: [fan] min_power/max_power scaling.
    world.gcode_ok("M106 S255")
    world.gcode_ok("M107")
    world.gcode_ok("SET_FAN_SPEED FAN=xxx SPEED=0.5")
    world.gcode_ok("SET_FAN_SPEED FAN=xxx SPEED=0")
    # heated_fan: target drives its heater; fan commands coexist.
    world.gcode_ok("SET_HEATED_FAN_TARGET TARGET=60")
    world.gcode_ok("M106 S255")
    world.gcode_ok("M107")
    world.gcode_ok("SET_HEATED_FAN_TARGET TARGET=0")
    assert world.shutdown_line() is None


def test_extruder_flows(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    _extrude_preamble(world)
    # Pressure-advance retune through the compat shim onto [post_processor pa].
    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.025")
    world.gcode_ok("G1 X30 Y30 E8.0")
    world.gcode_ok("G1 X25 Y25")
    world.gcode_ok("M400", timeout=60)
    world.gcode_ok("M302 P1 S0")
    world.gcode_ok("COLD_EXTRUDE HEATER=extruder ENABLE=0 MIN_EXTRUDE_TEMP=170")
    assert world.shutdown_line() is None


def test_heater_pid_flows(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("M104 S100")
    world.gcode_ok("M105")
    world.gcode_ok("M140 S60")
    world.gcode_ok("M104 S0")
    world.gcode_ok("M140 S0")
    # pid_hot_modify: live gain rewrite.
    world.gcode_ok("SET_HEATER_PID HEATER=extruder KP=25 KI=2 KD=120")
    # pid_profile: save/load/remove against extruder and chamber, plus the
    # config-declared TEST profile.
    world.gcode_ok("PID_PROFILE SAVE=TESTPROFILE HEATER=extruder")
    world.gcode_ok("PID_PROFILE LOAD=TESTPROFILE HEATER=extruder")
    world.gcode_ok("PID_PROFILE LOAD=TEST HEATER=chamber")
    world.gcode_ok("PID_PROFILE SAVE=TESTPROFILE HEATER=chamber")
    world.gcode_ok("PID_PROFILE LOAD=TESTPROFILE HEATER=chamber")
    world.gcode_ok(
        "SET_SMOOTH_TIME HEATER=extruder SMOOTH_TIME=1 SAVE_TO_PROFILE=1"
    )
    world.gcode_ok("PID_PROFILE REMOVE=TEST HEATER=chamber")
    assert world.shutdown_line() is None


def test_mpc_heater(sim_world):
    world = sim_world(_cfg_mpc, dual_mcu=False)
    _extrude_preamble(world)
    world.gcode_ok(
        "MPC_SET HEATER=extruder FILAMENT_DENSITY=1.15 FILAMENT_HEAT_CAPACITY=2.20"
    )
    world.gcode_ok("M105")
    assert world.shutdown_line() is None
