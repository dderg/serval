"""Print interruptions with heaters running: CANCEL_PRINT, PAUSE/RESUME,
end-of-print macros (M84, BED_MESH_CLEAR), a dwell behind queued travel,
the #403 purge macro, and FIRMWARE_RESTART mid-print, on one MCU and with
the XY motors on a second.

Field reports #406/#416/#369 (CANCEL_PRINT), #362 (BED_MESH_CLEAR in
END_PRINT) and #370 ended in `Timer too close` or `Missed scheduling of
next digital out event` on the MCU that drives the heaters, right after a
`reactor_poller_late` as long as the engine's brake-to-rest: a heater PWM
update was scheduled from a temperature sample taken before klippy's
reactor stalled and sent after it. Every scenario keeps two heaters
saturated (the sim's resting ADC reads ~34C, so the PID refreshes full
power every ~2.1 s against a 3 s max_duration) and asserts the reactor never
stalls, no MCU shuts down, and the printer takes commands afterwards.
"""

import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

HEATED_TARGETS = (
    "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=150",
    "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=100",
)
REACTOR_STALL = "reactor_poller_late"
PRINT_LINES = 120
PRINT_TIMEOUT_S = 600
MARK_PIN_NAME = "soft_pwm_pin"
MARK_PIN_LINE = 37
RESTART_DEADLINE_S = 5.0
XY_PIN_LINES = range(0, 6)
SHAPED_ENDSTOPS = ("gpio200", "gpio201")
EXTRUDER_SMOOTH_TIME = 0.02
OWNER_LIKE_LIMITS = (
    "max_velocity: 300\nmax_accel: 3000\n",
    "max_velocity: 500\nmax_accel: 20000\n",
)
SLICED_GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_dense_top_layers.gcode"
)
SLICED_LINES = 1500

CLEANNOZZLE_THEN_PURGE_BLOB = (
    "G91",
    "G0 Z25 F5000",
    "G4 P5000",
    "G0 Z-25",
    "G90",
    "BED_MESH_PROFILE LOAD=flat",
    "G92 E0",
    "G90",
    "G1 X100",
    "G1 Y0 Z1 F18000",
    "G1 Z.6 F600",
    "G91",
    "G1 X10 E30 F150",
    "M106 S200",
    "G90",
    "G0 Y10 F5000",
    "G91",
    "G1 X30 Z5 F5000",
    "G1 X15 Z-5.4 F9000",
    "G1 X10 E2 F400",
    "G1 Z10 F1500",
    "G0 Y20 F24000",
    "M106 S0",
    "G92 E0",
    "G90",
)

SLICER_END_GCODE = (
    "M104 S0",
    "SET_HEATER_TEMPERATURE HEATER=chamber TARGET=0",
    "G91",
    "G1 E-1 F1800",
    "G1 Z5 F600",
    "G90",
    "G0 X180 Y180 F6000",
    "M107",
    "M84",
)

INTERRUPTION_SECTIONS = """
[post_processor shaper_xy]
type: smooth_bell
smooth_time: 0.019125

[verify_heater extruder]
check_gain_time: 3600

[verify_heater chamber]
check_gain_time: 3600

[pause_resume]

[display_status]

[gcode_macro CANCEL_PRINT]
rename_existing: BASE_CANCEL_PRINT
gcode:
  CLEAR_PAUSE
  M220 S100
  M221 S100
  SDCARD_RESET_FILE
  BASE_CANCEL_PRINT
  G91
  G0 Z25 F4500
  G90
  G0 X190 Y190 F6000
  M107
  SET_STEPPER_ENABLE STEPPER=x ENABLE=0
  SET_STEPPER_ENABLE STEPPER=y ENABLE=0
  SET_STEPPER_ENABLE STEPPER=extruder ENABLE=0

[gcode_macro END_PRINT]
gcode:
  G91
  G1 E-0.5 F1800
  G0 Z5 F5000
  G90
  G0 X190 Y190 F6000
  M107
  BED_MESH_CLEAR
  M84

[bed_mesh]
mesh_min: 20, 20
mesh_max: 180, 180
probe_count: 3, 3
zero_reference_position: 100, 100
"""

SAVED_MESH = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh flat]
#*# version = 1
#*# points =
#*#   0.02, 0.00, -0.02
#*#   0.01, 0.00, -0.01
#*#   -0.02, 0.00, 0.02
#*# min_x = 20.0
#*# max_x = 180.0
#*# min_y = 20.0
#*# max_y = 180.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
"""


def _heated_config(world, extra=""):
    config = configs.with_extruder_smoothing(
        configs.heaters_config(world.h7_pty, str(world.gcode_dir)),
        EXTRUDER_SMOOTH_TIME,
    )
    stock_limits, owner_limits = OWNER_LIKE_LIMITS
    assert stock_limits in config, stock_limits
    config = config.replace(stock_limits, owner_limits)
    for endstop in SHAPED_ENDSTOPS:
        anchor = f"endstop_pin: ^gpiochip0/{endstop}\nhoming_speed: 10\n"
        assert anchor in config, anchor
        config = config.replace(anchor, anchor + "post_processors: shaper_xy\n")
    return config + INTERRUPTION_SECTIONS + extra + SAVED_MESH


def _single_mcu(world):
    return _heated_config(world)


def _xy_on_aux_mcu(world):
    config = _heated_config(world, f"\n[mcu aux]\nserial: {world.f4_pty}\n")
    for line in XY_PIN_LINES:
        config = config.replace(
            f"gpiochip0/gpio{line}\n", f"aux:gpiochip0/gpio{line}\n", 1
        )
    return config


TOPOLOGIES = {
    "one_mcu": (_single_mcu, False),
    "xy_on_aux": (_xy_on_aux_mcu, True),
}


def _boot(sim_world, topology):
    config_fn, dual_mcu = TOPOLOGIES[topology]
    world = sim_world(config_fn, dual_mcu=dual_mcu)
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=5")
    for command in HEATED_TARGETS:
        world.gcode_ok(command)
    return world


def _print_lines(mark_fraction, tail):
    lines = ["G90", "M83", "G1 Z0.3 F600", "G1 X60 Y20 F12000"]
    mark_index = int(mark_fraction * PRINT_LINES)
    for index in range(PRINT_LINES):
        if index == mark_index:
            lines.append(f"SET_PIN PIN={MARK_PIN_NAME} VALUE=1")
        x = 140.0 if index % 2 == 0 else 60.0
        y = 20.0 + (index // 2) * 0.4
        lines.append(f"G1 X{x:.1f} Y{y:.2f} E0.3 F6000")
    return lines + list(tail)


def _start_print(world, name, mark_fraction=1.0, tail=()):
    path = world.gcode_dir / name
    path.write_text("\n".join(_print_lines(mark_fraction, tail)) + "\n")
    world.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={name}")


def _wait_mark(world):
    control = world.sim_control("h7")
    world.wait_until(
        lambda: control.gpio_output(0, MARK_PIN_LINE) == 1,
        timeout=PRINT_TIMEOUT_S,
        what="the print reaching its marker line",
        poll=0.05,
    )


def _assert_clean(world):
    assert world.shutdown_line() is None, world.log_tail()[-4000:]
    log = world.settled_log_tail()
    assert world.shutdown_line() is None, log[-4000:]
    stalls = [
        line
        for line in world.events_text().splitlines()
        if REACTOR_STALL in line
    ]
    assert not stalls, "\n".join(stalls)


def _assert_ready_and_moving(world):
    assert world._printer_state() == "ready", world.log_tail()[-4000:]
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=30")
    world.gcode_ok("G1 X120 Y110 F6000")
    world.gcode_ok("M400", timeout=60)
    assert world.toolhead_position()[:2] == pytest.approx([120.0, 110.0])


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_stepper_disable_behind_travel(sim_world, topology):
    world = _boot(sim_world, topology)
    world.gcode_ok(
        "G1 X10 Y10 F6000\nG1 X190 Y190 F6000\n"
        "SET_STEPPER_ENABLE STEPPER=x ENABLE=0",
        timeout=60,
    )
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_dwell_behind_travel(sim_world, topology):
    world = _boot(sim_world, topology)
    world.gcode_ok("G1 X10 Y10 F6000\nG1 X190 Y190 F6000\nG4 P100", timeout=60)
    world.gcode_ok("M400", timeout=60)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_cancel_print_mid_print(sim_world, topology):
    world = _boot(sim_world, topology)
    _start_print(world, "cancel.gcode", mark_fraction=0.2)
    _wait_mark(world)
    world.gcode_ok("CANCEL_PRINT", timeout=120)
    world.gcode_ok("M400", timeout=120)
    _assert_clean(world)
    state = world.status({"print_stats": None})["print_stats"]["state"]
    assert state == "cancelled"
    _assert_ready_and_moving(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_pause_resume_mid_print(sim_world, topology):
    world = _boot(sim_world, topology)
    _start_print(world, "pause.gcode", mark_fraction=0.2)
    _wait_mark(world)
    world.gcode_ok("PAUSE", timeout=120)
    world.gcode_ok("G4 P2000", timeout=120)
    world.gcode_ok("RESUME", timeout=120)
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_slicer_end_gcode_heaters_off_then_m84(sim_world, topology):
    world = _boot(sim_world, topology)
    _start_print(world, "slicer_end.gcode", tail=SLICER_END_GCODE)
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=120)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_sliced_print_then_slicer_end_gcode(sim_world, topology):
    world = _boot(sim_world, topology)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=145 Z=40")
    sliced = SLICED_GCODE.read_text().splitlines()[:SLICED_LINES]
    path = world.gcode_dir / "sliced_end.gcode"
    path.write_text("\n".join(sliced + list(SLICER_END_GCODE)) + "\n")
    world.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={path.name}")
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=120)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_end_print_clears_mesh_with_heaters_on(sim_world, topology):
    world = _boot(sim_world, topology)
    world.gcode_ok("BED_MESH_PROFILE LOAD=flat")
    _start_print(world, "mesh_clear.gcode", tail=["END_PRINT"])
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=120)
    _assert_clean(world)


@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_purge_after_cleannozzle_dwell_and_mesh_load(sim_world, topology):
    world = _boot(sim_world, topology)
    world.gcode_ok("\n".join(CLEANNOZZLE_THEN_PURGE_BLOB), timeout=120)
    world.gcode_ok("M400", timeout=120)
    _assert_clean(world)


@pytest.mark.parametrize("fraction", [0.02, 0.3])
@pytest.mark.parametrize("topology", sorted(TOPOLOGIES))
def test_firmware_restart_mid_print(sim_world, topology, fraction):
    world = _boot(sim_world, topology)
    _start_print(world, "restart.gcode", mark_fraction=fraction)
    _wait_mark(world)
    world.mark_log()
    world.firmware_restart()
    world.expect_log("Restarting printer", timeout=RESTART_DEADLINE_S)
    assert world._wait_ready(120.0), world.log_tail()[-4000:]
    _assert_ready_and_moving(world)
    assert world.shutdown_line() is None, world.log_tail()[-4000:]
