"""Generated printer configs for the simulator.

Pin conventions (MACH_LINUX + libsim_intercept):
- Pin names use gpiochip0/gpioN format, not STM32 PA3 style.
- Auto-endstop walls (libsim_intercept.c): the runtime step queues notify
  the shim on lines X=18 / Y=7 / Z=15; after 50 steps toward the wall the
  shim asserts endstop lines X=gpio200 / Y=gpio201 / Z=gpio202+gpio203.
- Homing speeds stay low (<=10 mm/s) to tolerate Docker scheduler jitter.
"""

from __future__ import annotations

from typing import Optional

COMMON_TAIL = """
[post_processor is_xy]
type: smooth_bell
smooth_time: 0.019125

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _tail(gcode_dir: str) -> str:
    return COMMON_TAIL.format(gcode_dir=gcode_dir)


def neptune_print_config(h7_pty: str, gcode_dir: str) -> str:
    """Neptune 3 Pro bench profile on sim pins: real print limits and an
    extruder follower with the bench's pressure-advance + smoothing
    chain — the setup that reproduces motion-content bugs slicer prints hit."""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 300
max_accel: 4000
max_jerk: 1000000
max_z_velocity: 25
max_z_accel: 200
square_corner_velocity: 8

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10

[axis z]
position_min: -5
position_endstop: 0
position_max: 280
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[axis e]
follows: x, y, z
motors: extruder
post_processors: pa, st

[post_processor pa]
type: linear_pressure_advance
k: 0.03

[post_processor st]
type: smooth_triangle
smooth_time: 0.04

[limit gantry]
axes: x, y
max_velocity: 300
max_accel: 4000

[limit z]
axes: z
max_velocity: 25
max_accel: 200

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 8

[motor extruder]
drive: stepper
step_pin: gpiochip0/gpio20
dir_pin: !gpiochip0/gpio21
enable_pin: !gpiochip0/gpio22
microsteps: 16
rotation_distance: 7.73

[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: gpiochip0/gpio30
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog0
min_temp: 0
max_temp: 250
min_extrude_temp: 0
control: pid
pid_kp: 30.356
pid_ki: 1.857
pid_kd: 124.081

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def minimal_config(h7_pty: str, gcode_dir: str) -> str:
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4
{_tail(gcode_dir)}"""


def corexy_fast_config(h7_pty: str, gcode_dir: str) -> str:
    """CoreXY on the Trident bench's motion limits (max_velocity 2800,
    max_accel 100000, square_corner_velocity 100), single-MCU. Used to
    exercise the beacon rapid-scan path shape in the planner without the
    beacon stream in the loop."""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 2800
max_accel: 100000
# shaper_x kernel share 0.653mm at 100000mm/s^2 + the old scv-100 blend
# budget 0.041mm (corner_deviation is the total incl. the kernel share).
corner_deviation: 0.695
max_z_velocity: 25
max_z_accel: 100

[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: shaper_x

[axis y]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: shaper_y

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 2800
max_accel: 100000

[limit z]
axes: z
max_velocity: 25
max_accel: 100

[motor a]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4

[post_processor shaper_x]
type: smooth_bell
smooth_time: 0.019125

[post_processor shaper_y]
type: smooth_bell
smooth_time: 0.018238636363636363

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def corexy_tracked_config(h7_pty: str, gcode_dir: str) -> str:
    """CoreXY with the A/B motor step queues on the shim's tracked lines
    (a→gpio18, b→gpio7) so get_steps can observe the executed motor tracks.
    Trident-bench motion limits from its printer.cfg (max_velocity 1000,
    max_accel 10000, scv 30); used to replay the servo-ident stroke/dwell
    pattern and audit commanded-position continuity at stroke boundaries."""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 1000
max_accel: 10000
# shaper kernel share 0.173mm at 10000mm/s^2 + the old scv-30 blend
# budget 0.037mm (corner_deviation is the total incl. the kernel share).
corner_deviation: 0.21
max_z_velocity: 25
max_z_accel: 100

[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: shaper_x

[axis y]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: shaper_y

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 1000
max_accel: 10000

[limit z]
axes: z
max_velocity: 25
max_accel: 100

[motor a]
drive: stepper
step_pin: gpiochip0/gpio18
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: gpiochip0/gpio7
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio8
enable_pin: !gpiochip0/gpio9
microsteps: 16
rotation_distance: 4

[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 50

[post_processor shaper_y]
type: smooth_zv
frequency_hz: 44

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


AWD_TMC_RUN_CURRENT = 0.8
AWD_TMC_SENSE_RESISTOR = 0.075
AWD_TMC_HOME_CURRENTS = {"a": 0.45, "a1": 0.50, "b": 0.55, "b1": 0.60}
AWD_TMC_CS_LINES = {"a": 5, "a1": 4, "b": 6, "b1": 3}


def awd_corexy_tmc_config(h7_pty: str, gcode_dir: str) -> str:
    """AWD CoreXY: two motors per belt lane, each with its own TMC5160
    at a distinct home_current. Exercises homing-current switching across
    kinematically coupled lanes."""
    motor_sections = ""
    for i, name in enumerate(("a", "a1", "b", "b1")):
        motor_sections += f"""
[motor {name}]
drive: stepper
step_pin: gpiochip0/gpio{30 + 3 * i}
dir_pin: gpiochip0/gpio{31 + 3 * i}
enable_pin: !gpiochip0/gpio{32 + 3 * i}
microsteps: 16
rotation_distance: 40

[tmc5160 {name}]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio{AWD_TMC_CS_LINES[name]}
run_current: {AWD_TMC_RUN_CURRENT}
home_current: {AWD_TMC_HOME_CURRENTS[name]}
sense_resistor: {AWD_TMC_SENSE_RESISTOR}
"""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a, a1
b_motors: b, b1
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30
{motor_sections}
[motor z]
drive: stepper
step_pin: gpiochip0/gpio50
dir_pin: gpiochip0/gpio51
enable_pin: !gpiochip0/gpio52
microsteps: 16
rotation_distance: 4
{_tail(gcode_dir)}"""


def phase_stepping_config(h7_pty: str, gcode_dir: str) -> str:
    """TMC5160 phase stepping on X."""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 256
rotation_distance: 40
phase_stepping: True

[tmc5160 x]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 16
rotation_distance: 4
{_tail(gcode_dir)}"""


def sensorless_phase_config(h7_pty: str, gcode_dir: str) -> str:
    """Phase-stepping Z with a TMC5160 virtual (StallGuard) endstop.

    The DIAG pin is gpiochip0/gpio203 — the libsim_intercept auto-endstop
    wall asserts it after 50 steps on the runtime Z step-queue line
    (gpio15), standing in for a StallGuard trip during the homing move.
    Z is used because Z homing at 5 mm/s is the sim's validated homing
    path (X full-mode homing times out under the vtime clock race).
    """
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: tmc5160_z:virtual_endstop
homing_speed: 5
homing_retract_dist: 0

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 256
rotation_distance: 40
phase_stepping: True

[tmc5160 z]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075
diag0_pin: gpiochip0/gpio203
driver_SGT: 1
{_tail(gcode_dir)}"""


def beacon_homing_config(
    h7_pty: str,
    f4_pty: Optional[str],
    beacon_pty: str,
    gcode_dir: str,
    bed_mesh: bool = False,
) -> str:
    """Dual-MCU CoreXY with Beacon proximity Z homing.

    The SAVE_CONFIG beacon model must match the stub's frequency model.
    model_domain [1.8359e-7, 1.8936e-7] maps to ~5.28-5.45 MHz:
      z=10mm -> count ~= 70,710,853 (freq ~= 5,268,182 Hz)
      z=0    -> count ~= 73,153,076 (freq ~= 5,450,000 Hz)
    Changing either without the other causes boot-time calibration
    rejection.
    """
    f4_section = ""
    if f4_pty:
        f4_section = f"""
[mcu bottom]
serial: {f4_pty}
"""
    z_step_mcu = "bottom:" if f4_pty else ""
    bed_mesh_section = ""
    saved_mesh_profiles = ""
    if bed_mesh:
        bed_mesh_section = """
[bed_mesh]
mesh_min: 20,20
mesh_max: 280,280
probe_count: 3,3
zero_reference_position: 150, 150
"""
        saved_mesh_profiles = """\
#*# [bed_mesh edge]
#*# version = 1
#*# points =
#*#   -0.026, -0.010, 0.013
#*#   -0.020, -0.005, 0.016
#*#   -0.013, 0.002, 0.024
#*# min_x = 120.0
#*# max_x = 160.0
#*# min_y = 120.0
#*# max_y = 160.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
#*#
"""
    return f"""\
[mcu]
serial: {h7_pty}
{f4_section}
[printer]
max_velocity: 300
max_accel: 3000
# is_xy's kernel share (0.0196mm at 3000mm/s^2) + the old default blend
# budget (scv 5 -> 0.0035mm): corner_deviation is the TOTAL since the
# kernel-share deduction landed, so this keeps the pre-change geometry.
corner_deviation: 0.023

[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z

[axis x]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
position_endstop: 0
position_max: 300
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_max: 250
endstop_pin: probe:z_virtual_endstop
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 300
max_accel: 3000

[limit z]
axes: z
max_velocity: 10
max_accel: 100

[motor a]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: {z_step_mcu}gpiochip0/gpio0
dir_pin: {z_step_mcu}gpiochip0/gpio1
enable_pin: !{z_step_mcu}gpiochip0/gpio2
microsteps: 16
rotation_distance: 4

[beacon]
serial: {beacon_pty}
x_offset: 0
y_offset: 0
home_xy_position: 150, 150
home_z_hop: 5
home_z_hop_speed: 10
home_xy_move_speed: 50
home_method: proximity
home_method_when_homed: proximity
home_autocalibrate: never
# The emulator fires the contact trigger within step/poll latency
# (~0.01mm) of the true bed crossing, so the hardware-default touch
# repeatability gate (0.008) is marginally flaky in the sim.
autocal_tolerance: 0.02
{bed_mesh_section}
[post_processor is_xy]
type: smooth_bell
smooth_time: 0.019125

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
{saved_mesh_profiles}#*# [beacon model default]
#*# model_coef = 1.4366832587589902,
#*#   1.7791425946955506,
#*#   0.8114676630327906,
#*#   0.4077638527717382,
#*#   0.2629778891883896,
#*#   0.21087515838926726,
#*#   -0.15390965626840192,
#*#   -0.21990798533166914,
#*#   0.24377872047881705,
#*#   0.22573604715705745
#*# model_domain = 1.8359521074610915e-07,1.893648763276026e-07
#*# model_range = 0.200000,5.000000
#*# model_temp = 30.886664
#*# model_offset = 0.00000
"""


PROBE_VARIANTS = (
    "virtual",
    "safe-z",
    "gpio-z",
    "no-probe",
    "conflict",
    "pullup",
    "remote",
    "points",
)

PROBE_BOOT_ERRORS = {
    "no-probe": "Unknown pin chip name 'probe'",
    "conflict": "must not set position_endstop",
    "pullup": "Can not pullup/invert probe virtual endstop",
}


def probe_config(
    h7_pty: str, gcode_dir: str, variant: str, f4_pty: Optional[str] = None
) -> str:
    if variant == "gpio-z":
        z_endstop = "endstop_pin: ^gpiochip0/gpio202\nposition_endstop: 0"
        probe_pin = "gpiochip0/gpio203"
    elif variant == "pullup":
        z_endstop = "endstop_pin: ^probe:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio202"
    elif variant == "remote":
        z_endstop = "endstop_pin: sim_remote_endstop:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio203"
    elif variant == "conflict":
        z_endstop = (
            "endstop_pin: probe:z_virtual_endstop\nposition_endstop: 1.0"
        )
        probe_pin = "gpiochip0/gpio202"
    else:
        z_endstop = "endstop_pin: probe:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio202"

    probe_section = ""
    if variant != "no-probe":
        probe_section = f"""
[probe]
pin: {probe_pin}
z_offset: 1.5
speed: 5
x_offset: 24.0
y_offset: 5.0
"""

    safe_z_section = ""
    if variant == "safe-z":
        safe_z_section = """
[safe_z_home]
home_xy_position: 125, 125
z_hop: 10
z_hop_speed: 15
"""

    remote_section = ""
    z_min_home_dist = ""
    if variant == "remote":
        if f4_pty is None:
            raise ValueError(
                "probe remote variant: a second (F4) sim MCU is required so"
                " the trsync lives on a different MCU than the steppers"
            )
        probe_section = ""
        # The trigger is a wall-clock timer, not a position, so the trip
        # point moves with each approach — the min_home_dist re-approach
        # check would reject it. This variant exercises the cross-MCU
        # trsync relay, not the early-trigger guard.
        z_min_home_dist = "min_home_dist: 0"
        remote_section = f"""
[mcu bottom]
serial: {f4_pty}

[sim_remote_endstop]
mcu: bottom
trigger_delay: 1.0
measured_z: 3.25
trigger_height: 0
"""

    z_motors = "z"
    points_sections = ""
    if variant == "points":
        z_motors = "z, z1"
        points_sections = """
[motor z1]
drive: stepper
step_pin: gpiochip0/gpio9
dir_pin: gpiochip0/gpio10
enable_pin: !gpiochip0/gpio11
microsteps: 16
rotation_distance: 4

[z_tilt]
z_positions:
    0, 125
    250, 125
points:
    50, 125
    200, 125
speed: 50
horizontal_move_z: 8

[bed_mesh]
mesh_min: 30, 10
mesh_max: 200, 200
probe_count: 3, 3
speed: 50
horizontal_move_z: 8

[screws_tilt_adjust]
screw1: 50, 50
screw1_name: front left
screw2: 200, 50
screw2_name: front right
screw3: 125, 200
screw3_name: back
speed: 50
horizontal_move_z: 8
screw_thread: CW-M4

[axis_twist_compensation]
calibrate_start_x: 30
calibrate_end_x: 200
calibrate_y: 125
"""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: {z_motors}

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio200
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio201
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_max: 250
homing_speed: 5
{z_min_home_dist}
{z_endstop}

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 30

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4
{safe_z_section}{probe_section}{remote_section}{points_sections}{_tail(gcode_dir)}"""


def bed_mesh_config(h7_pty: str, gcode_dir: str) -> str:
    """Cartesian printer with two stored bed_mesh profiles.

    - "wavy": +/-0.1mm corrections, zero at the (45,45) zero reference,
      loadable without probing via BED_MESH_PROFILE LOAD=wavy. The mesh
      spans only 20..70mm so node-to-node verification moves stay short
      enough for the sim's virtual clock to execute them faithfully.
    - "steep": a +/-4mm linear tilt whose XY-coupled Z demand exceeds the
      [limit z] velocity/accel budget, so activation must be refused by
      the motion engine's gross-error gate.

    Z homes against the auto-endstop wall (step line 15 -> gpio202), the
    same arrangement as probe_config's "gpio-z" variant.
    """
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 100
max_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio200
homing_speed: 10
post_processors: is_xy

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio201
homing_speed: 10
post_processors: is_xy

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio202
homing_speed: 5

[limit gantry]
axes: x, y
max_velocity: 100
max_accel: 1000

[limit z]
axes: z
max_velocity: 10
max_accel: 100

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4

[bed_mesh]
mesh_min: 20, 20
mesh_max: 70, 70
probe_count: 3, 3
zero_reference_position: 45, 45
fade_start: 1
fade_end: 10
fade_target: 0
{_tail(gcode_dir)}
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh wavy]
#*# version = 1
#*# points =
#*#   0.10, 0.00, -0.10
#*#   0.05, 0.00, -0.05
#*#   -0.10, 0.00, 0.10
#*# min_x = 20.0
#*# max_x = 70.0
#*# min_y = 20.0
#*# max_y = 70.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
#*#
#*# [bed_mesh steep]
#*# version = 1
#*# points =
#*#   -4.0, 0.0, 4.0
#*#   -4.0, 0.0, 4.0
#*#   -4.0, 0.0, 4.0
#*# min_x = 20.0
#*# max_x = 70.0
#*# min_y = 20.0
#*# max_y = 70.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
"""


SELF_TEST_GCODE = """\
; Sim self-test: square spiral with Z moves
SET_KINEMATIC_POSITION X=125 Y=125 Z=125
G1 Z120 F300
G1 X10 Y10 F3000
G1 X100 Y10 F3000
G1 X100 Y100 F3000
G1 X10 Y100 F3000
G1 X10 Y10 F3000
G1 X30 Y30 F2000
G1 X80 Y30 F2000
G1 X80 Y80 F2000
G1 X30 Y80 F2000
G1 X30 Y30 F2000
G1 Z125 F300
M400
"""

PHASE_TEST_GCODE = """\
; Phase stepping acceptance test
SET_KINEMATIC_POSITION X=0 Y=125 Z=125
G1 X50 F1000
G1 X100 F2000
G1 X50 F3000
G1 X0 F1000
M400
"""


def heaters_config(
    h7_pty: str, gcode_dir: str, control: str = "pid", heated_fan: bool = False
) -> str:
    """Cartesian world carrying the heater/fan/pwm zoo the legacy batch
    suite (test/klippy) covered: extruder heater (pid or mpc), chamber
    heater with saved PID profiles, plain/scaled/generic/temperature/heated
    fans, output_pin soft PWM, pwm_cycle_time, pwm_tool, and the
    thermistor/ADC sensor family. SPI thermocouples (max6675/31855/31856)
    are omitted: the sim has no emulators for them, so they would read
    garbage instead of exercising anything.
    """
    if control == "mpc":
        extruder_control = """\
control: mpc
heater_power: 50
cooling_fan: fan
filament_density: 1.20
filament_heat_capacity: 1.8
block_heat_capacity: 22.3110
sensor_responsiveness: 0.0998635
ambient_transfer: 0.155082
fan_ambient_transfer: 0.155082, 0.20156, 0.216441
"""
        mpc_sensors = """
[temperature_sensor test_mpc_block]
sensor_type: mpc_block_temperature
heater_name: extruder
min_temp: 0
max_temp: 325
ignore_limits: True

[temperature_sensor test_mpc_ambient]
sensor_type: mpc_ambient_temperature
heater_name: extruder
min_temp: 0
max_temp: 100
ignore_limits: True
"""
    else:
        extruder_control = """\
control: pid
pid_kp: 22.200
pid_ki: 1.080
pid_kd: 114.000
"""
        mpc_sensors = ""
    if heated_fan:
        # heated_fan registers itself as THE fan and refuses to coexist
        # with a [fan] section, exactly like the legacy world split.
        fan_section = """\
[heated_fan]
heater_pin: gpiochip0/gpio34
sensor_type: Generic 3950
sensor_pin: analog2
min_temp: 0
max_temp: 130
control: pid
pid_kp: 63.350
pid_ki: 4.100
pid_kd: 244.691
pin: gpiochip0/gpio35
heater_temp: 50
min_speed: 0.5
idle_timeout: 5
"""
    else:
        fan_section = """\
[fan]
pin: gpiochip0/gpio31
min_power: 0.1
max_power: 1
"""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 300
max_accel: 3000
max_z_velocity: 5
max_z_accel: 100

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis x]
position_min: 0
position_endstop: 0
position_max: 200
endstop_pin: ^gpiochip0/gpio200
homing_speed: 10

[axis y]
position_min: 0
position_endstop: 0
position_max: 200
endstop_pin: ^gpiochip0/gpio201
homing_speed: 10

[axis z]
position_min: -5
position_endstop: 0
position_max: 200
endstop_pin: ^gpiochip0/gpio202
homing_speed: 5

[axis e]
follows: x, y, z
motors: extruder
post_processors: pa

[post_processor pa]
type: linear_pressure_advance
k: 0.02

[motor x]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor y]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 8

[motor extruder]
drive: stepper
step_pin: gpiochip0/gpio20
dir_pin: !gpiochip0/gpio21
enable_pin: !gpiochip0/gpio22
microsteps: 16
rotation_distance: 33.5

[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: gpiochip0/gpio30
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog0
min_temp: 0
max_temp: 210
min_extrude_temp: 0
{extruder_control}
[heater_generic chamber]
heater_pin: gpiochip0/gpio36
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog3
control: pid
pid_kp: 22.200
pid_ki: 1.080
pid_kd: 114.000
min_temp: 0
max_temp: 120

[pid_profile chamber TEST]
pid_version: 1
control: pid
pid_kp: 22.200
pid_ki: 1.080
pid_kd: 114.000

{fan_section}
[fan_generic xxx]
pin: gpiochip0/gpio32
min_power: 0.1
shutdown_speed: 1
max_power: 0.95

[temperature_fan my_temp_fan]
pin: gpiochip0/gpio33
reverse: true
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog1
control: pid
pid_Kp: 22.2
pid_Ki: 1.08
pid_Kd: 114
min_temp: 0
max_temp: 210

[output_pin soft_pwm_pin]
pin: gpiochip0/gpio37
pwm: True
value: 0
shutdown_value: 0
cycle_time: 0.01

[pwm_cycle_time cycle_pwm_pin]
pin: gpiochip0/gpio38
value: 0
shutdown_value: 0
cycle_time: 0.01

[thermistor my_custom_thermistor]
temperature1: 20
resistance1: 100000
beta: 4066

[temperature_fan test_custom_thermistor]
pin: gpiochip0/gpio40
sensor_type: my_custom_thermistor
sensor_pin: analog4
control: watermark
min_temp: 0
max_temp: 210

# Calibrated around the sim shim's resting ADC (3900/4095 at 5V, ~4.76V):
# the linear fit reads ~34C there. PT1000/resistance calibrations are
# omitted — the fixed ADC ratio puts them kilo-degrees out of range.
[adc_temperature my_custom_adc]
temperature1: 20
voltage1: 4.9
temperature2: 60
voltage2: 4.5

[temperature_sensor test_custom_adc]
sensor_type: my_custom_adc
sensor_pin: analog5
min_temp: 0
max_temp: 210

[temperature_sensor test_epcos]
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog6
min_temp: 0
max_temp: 210

[temperature_sensor test_combined]
sensor_type: temperature_combined
sensor_list: temperature_sensor test_custom_adc, temperature_sensor test_epcos
combination_method: max
maximum_deviation: 999
{mpc_sensors}
[controller_fan test_controller_fan]
pin: gpiochip0/gpio41

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""
