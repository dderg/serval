"""Doc dogfood: the voron0 bench config translated per Config_Migration.md.

Pins are sim pins; everything else is the bench's real topology — CoreXY
a/b lanes, 64 microsteps at 200 full steps, a gear-reduced Z, a virtual
(StallGuard) X endstop named after its motor section, the measured
smooth_mzv frequencies, and an extruder follower. TMC5160 stands in for
the bench's TMC2209: the sim has one TMC2209 UART emulator at slave 0,
and the section-naming rule under test (register_chip / the required
[motor <name>] lookup) is shared by every tmc driver.
"""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.needs_elf

VORON0 = """\
[mcu]
serial: {h7_pty}

[printer]
max_velocity: 600
max_accel: 20000
max_jerk: 40000
max_z_velocity: 20
max_z_accel: 500
corner_deviation: 0.04

[kinematics]
type: corexy
axis_x: x
a_motors: a_motor
axis_y: y
b_motors: b_motor
axis_z: z
z_motors: z_motor

[motor a_motor]
drive: stepper
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 64
full_steps_per_rotation: 200
rotation_distance: 40

[tmc5160 a_motor]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
interpolate: False
run_current: 0.9
home_current: 0.7
sense_resistor: 0.110
diag0_pin: gpiochip0/gpio200
driver_SGT: 1

[motor b_motor]
drive: stepper
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 64
full_steps_per_rotation: 200
rotation_distance: 40

[motor z_motor]
drive: stepper
step_pin: gpiochip0/gpio6
dir_pin: !gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 32
rotation_distance: 40
gear_ratio: 80:20, 2:1

[motor e_motor]
drive: stepper
step_pin: gpiochip0/gpio22
dir_pin: !gpiochip0/gpio23
enable_pin: !gpiochip0/gpio24
microsteps: 32
full_steps_per_rotation: 200
rotation_distance: 22.095
gear_ratio: 50:10

[axis x]
endstop_pin: tmc5160_a_motor:virtual_endstop
position_endstop: 121
position_max: 121
homing_speed: 40
homing_retract_dist: 10
homing_positive_dir: true
min_home_dist: 0
post_processors: x_shaping

[axis y]
endstop_pin: ^gpiochip0/gpio11
position_endstop: 120
position_max: 120
homing_speed: 40
homing_retract_dist: 10
homing_positive_dir: true
min_home_dist: 0
post_processors: y_shaping

[axis z]
endstop_pin: ^gpiochip0/gpio12
position_min: -5
position_endstop: 116.100
position_max: 120
homing_speed: 40
second_homing_speed: 3.0
homing_retract_dist: 3.0
post_processors: z_shaping

[axis e]
follows: x, y, z
motors: e_motor
post_processors: e_smoothing

[post_processor x_shaping]
type: smooth_mzv
frequency_hz: 112.8

[post_processor y_shaping]
type: smooth_mzv
frequency_hz: 90.2

[post_processor z_shaping]
type: smooth_bell
smooth_time: 0.025

[post_processor e_smoothing]
type: smooth_triangle
smooth_time: 0.01

[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: gpiochip0/gpio30
sensor_type: Generic 3950
sensor_pin: analog0
min_temp: 0
max_temp: 280
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


def test_voron0_migrated_config_boots_and_moves(sim_world):
    world = sim_world(
        lambda w: VORON0.format(h7_pty=w.h7_pty, gcode_dir=str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=60 Y=60 Z=20")
    world.gcode_ok("G90")
    world.gcode_ok("G1 X100 Y100 F18000")
    world.gcode_ok("G1 X20 Y100 F18000")
    world.gcode_ok("G1 Z10 F1200")
    world.gcode_ok("M83")
    world.gcode_ok("G1 X60 Y60 E2 F6000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None
    x, y, z = world.toolhead_position()[:3]
    assert abs(x - 60.0) < 0.01
    assert abs(y - 60.0) < 0.01
    assert abs(z - 10.0) < 0.01
