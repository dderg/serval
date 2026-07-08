import pytest

pytestmark = pytest.mark.needs_elf

TRIDENT_STEPS_PER_MM = 200 * 16 * 37 / 53.65


def trident_topology_config(h7_pty: str, f4_pty: str, gcode_dir: str) -> str:
    """Bench-shaped topology: corexy, a/b/z motors on the bottom MCU, the
    geared extruder alone on the H7 (present=0x8), trident machine limits
    and the bench's PA + smooth_triangle extruder chain."""
    return f"""\
[mcu]
serial: {h7_pty}

[mcu bottom]
serial: {f4_pty}

[printer]
max_velocity: 2800
max_accel: 50000
max_jerk: 4000000
square_corner_velocity: 100
max_z_velocity: 25
max_z_accel: 1000

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
endstop_pin: ^bottom:gpiochip0/gpio10
homing_speed: 10

[axis y]
position_endstop: 0
position_max: 300
endstop_pin: ^bottom:gpiochip0/gpio11
homing_speed: 10

[axis z]
position_min: -5
position_max: 230
endstop_pin: ^bottom:gpiochip0/gpio12
homing_speed: 5

[axis e]
follows: x, y, z
motors: extruder
post_processors: pa, st

[post_processor pa]
type: linear_pressure_advance
k: 0.017

[post_processor st]
type: smooth_triangle
smooth_time: 0.016

[limit gantry]
axes: x, y
max_velocity: 2800
max_accel: 50000

[limit z]
axes: z
max_velocity: 25
max_accel: 1000

[motor a]
drive: stepper
step_pin: bottom:gpiochip0/gpio0
dir_pin: bottom:gpiochip0/gpio1
enable_pin: !bottom:gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: bottom:gpiochip0/gpio3
dir_pin: bottom:gpiochip0/gpio4
enable_pin: !bottom:gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: bottom:gpiochip0/gpio6
dir_pin: bottom:gpiochip0/gpio7
enable_pin: !bottom:gpiochip0/gpio8
microsteps: 16
rotation_distance: 8

[motor extruder]
drive: stepper
step_pin: gpiochip0/gpio20
dir_pin: !gpiochip0/gpio21
enable_pin: !gpiochip0/gpio22
microsteps: 16
gear_ratio: 37:1
rotation_distance: 53.65
full_steps_per_rotation: 200

[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: gpiochip0/gpio30
sensor_type: EPCOS 100K B57560G104F
sensor_pin: analog0
min_temp: 0
max_temp: 325
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


def test_first_extrude_after_boot(sim_world):
    world = sim_world(
        lambda w: trident_topology_config(w.h7_pty, w.f4_pty, str(w.gcode_dir)),
        dual_mcu=True,
    )
    world.gcode_ok("G1 E1 F60")
    world.gcode_ok("M400")
    world.gcode_ok("SET_KINEMATIC_POSITION X=10 Y=10 Z=10")
    world.gcode_ok("G1 E1.5 F60")
    world.gcode_ok("M400")
    resp = world.sim_control("h7").send("get_steps line=20")
    print("extruder track:", resp)
    assert world.shutdown_line() is None, world.log_tail()
    kv = dict(p.split("=") for p in resp.split())
    assert int(kv["min"]) > -17, f"extruder dove negative: {resp}"
    assert abs(int(kv["steps"]) - 1.5 * TRIDENT_STEPS_PER_MM) < 40, (
        f"bad final pos: {resp}"
    )
