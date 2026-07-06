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
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _tail(gcode_dir: str) -> str:
    return COMMON_TAIL.format(gcode_dir=gcode_dir)


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


def multi_z_config(h7_pty: str, gcode_dir: str) -> str:
    """CoreXY with three Z motors, for MOTOR_ADJUST testing."""
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
a_motors: a
b_motors: b
z_motors: z, z1, z2

[axis x]
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio10
homing_speed: 10
post_processors: is_xy

[axis y]
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

[motor z1]
drive: stepper
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
microsteps: 16
rotation_distance: 4

[motor z2]
drive: stepper
step_pin: gpiochip0/gpio16
dir_pin: gpiochip0/gpio17
enable_pin: !gpiochip0/gpio18
microsteps: 16
rotation_distance: 4

[motor_adjust]
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
    if bed_mesh:
        bed_mesh_section = """
[bed_mesh]
mesh_min: 20,20
mesh_max: 280,280
probe_count: 3,3
"""
    return f"""\
[mcu]
serial: {h7_pty}
{f4_section}
[printer]
max_velocity: 300
max_accel: 3000

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
{bed_mesh_section}
[post_processor is_xy]
type: smooth_mzv
frequency_hz: 50

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [beacon model default]
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
    if variant == "remote":
        if f4_pty is None:
            raise ValueError(
                "probe remote variant: a second (F4) sim MCU is required so"
                " the trsync lives on a different MCU than the steppers"
            )
        probe_section = ""
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


SELF_TEST_GCODE = """\
; Sim self-test: square spiral with Z moves
SET_KINEMATIC_POSITION X=125 Y=125 Z=125
G1 Z120 F300
G1 X10 Y10 F3000
G1 X100 Y10 F3000
G1 X100 Y100 F3000
G1 X10 Y100 F3000
G1 X10 Y10 F3000
G1 X20 Y20 F3000
G1 X90 Y20 F3000
G1 X90 Y90 F3000
G1 X20 Y90 F3000
G1 X20 Y20 F3000
G1 X30 Y30 F2000
G1 X80 Y30 F2000
G1 X80 Y80 F2000
G1 X30 Y80 F2000
G1 X30 Y30 F2000
G1 X40 Y40 F1500
G1 X70 Y40 F1500
G1 X70 Y70 F1500
G1 X40 Y70 F1500
G1 X40 Y40 F1500
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
