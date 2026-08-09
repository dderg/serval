import pathlib

import pytest

pytestmark = pytest.mark.needs_elf


def issue_370_config(h7_pty: str, servo_pty: str, gcode_dir: str) -> str:
    return f"""\
[mcu]
serial: {h7_pty}

[mcu servo]
serial: {servo_pty}

[printer]
max_velocity: 1000
max_accel: 100000
corner_deviation: 0.04
max_z_velocity: 100
max_z_accel: 1000

[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: motor_x, motor_x1
y_motors: motor_y, motor_y1
z_motors: motor_z, motor_z1, motor_z2

[axis x]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^servo:gpiochip0/gpio10
homing_speed: 10

[axis y]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^servo:gpiochip0/gpio11
homing_speed: 10

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[axis e]
follows: x, y, z
motors: extruder

[motor motor_x]
drive: stepper
step_pin: servo:gpiochip0/gpio0
dir_pin: servo:gpiochip0/gpio1
enable_pin: !servo:gpiochip0/gpio2
microsteps: 16
rotation_distance: 40

[motor motor_x1]
drive: stepper
step_pin: servo:gpiochip0/gpio3
dir_pin: servo:gpiochip0/gpio4
enable_pin: !servo:gpiochip0/gpio5
microsteps: 16
rotation_distance: 40

[motor motor_y]
drive: stepper
step_pin: servo:gpiochip0/gpio6
dir_pin: servo:gpiochip0/gpio7
enable_pin: !servo:gpiochip0/gpio8
microsteps: 16
rotation_distance: 40

[motor motor_y1]
drive: stepper
step_pin: servo:gpiochip0/gpio9
dir_pin: servo:gpiochip0/gpio10
enable_pin: !servo:gpiochip0/gpio11
microsteps: 16
rotation_distance: 40

[motor motor_z]
drive: stepper
step_pin: gpiochip0/gpio12
dir_pin: gpiochip0/gpio13
enable_pin: !gpiochip0/gpio14
microsteps: 16
rotation_distance: 4

[motor motor_z1]
drive: stepper
step_pin: gpiochip0/gpio15
dir_pin: gpiochip0/gpio16
enable_pin: !gpiochip0/gpio17
microsteps: 16
rotation_distance: 4

[motor motor_z2]
drive: stepper
step_pin: gpiochip0/gpio18
dir_pin: gpiochip0/gpio19
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 4

[motor extruder]
drive: stepper
step_pin: gpiochip0/gpio21
dir_pin: gpiochip0/gpio22
enable_pin: !gpiochip0/gpio23
microsteps: 16
rotation_distance: 7.73

[extruder]
axis: e
nozzle_diameter: 0.4
filament_diameter: 1.75
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


def issue_370_gcode(path: pathlib.Path) -> None:
    lines = [
        "G90",
        "M83",
        "SET_KINEMATIC_POSITION X=150 Y=150 Z=10",
        "SET_VELOCITY_LIMIT VELOCITY=1000 ACCEL=100000",
        "G1 X20 Y20 F42000",
        "G1 X280 Y280 F42000",
        "SET_VELOCITY_LIMIT ACCEL=10000",
    ]
    points = ((20, 20), (280, 280), (280, 20), (20, 280))
    for cycle in range(96):
        for x, y in points:
            lines.append(f"G1 X{x} Y{y} E0.08 F42000")
        lines.append(f"G1 X{80 + cycle % 80} Y{80 + (cycle * 7) % 80} E0.04 F6000")
    lines.extend(("M400", ""))
    path.write_text("\n".join(lines))


def test_issue_370_high_load_print_does_not_restart(sim_world):
    world = sim_world(
        lambda w: issue_370_config(w.h7_pty, w.f4_pty, str(w.gcode_dir)),
        dual_mcu=True,
    )
    gcode_path = world.gcode_dir / "issue_370.gcode"
    issue_370_gcode(gcode_path)

    print_time = world.print_file(gcode_path, timeout=1800)

    assert print_time > 0
    assert world.shutdown_line() is None
    events = world.events_text()
    assert "runtime_fault" not in events
    assert "diag.rust_fault" not in events
