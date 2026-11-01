"""stepper_classic_halt() must not toggle a both-edge stepper's step pin.

Upstream Klipper's stepper_stop (src/stepper.c at the fork point
b3061d21b) masks its idle-level pin reset with `!(s->flags &
SF_SINGLE_SCHED)`: a both-edge (TMC `step_both_edge`) stepper never
pulses and returns, it just toggles once per step and leaves the pin
there, so forcing it back to idle on every halt is itself an untracked
step whenever the pin was left high.

A single FORCE_MOVE leaves the step pin at an odd real-edge count.
Pre-arming the endstop before G28 starts means homing recognizes the
trip immediately and takes no further legitimate steps, so any edge the
step pin gains between there and shutdown is the halt's own stray
write.
"""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.needs_elf

STEP_CHIP = 0
STEP_LINE = 0
ENDSTOP_CHIP = 0
ENDSTOP_LINE = 10
ROTATION_DISTANCE = 40.0
MICROSTEPS = 16
FULL_STEPS_PER_ROTATION = 200
MICROSTEP_MM = ROTATION_DISTANCE / (FULL_STEPS_PER_ROTATION * MICROSTEPS)


def _cfg(world) -> str:
    return f"""\
[mcu]
serial: {world.h7_pty}

[printer]
max_velocity: 100
max_accel: 1000
max_jerk: 0
max_z_velocity: 10
max_z_accel: 30

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
endstop_pin: ^gpiochip{ENDSTOP_CHIP}/gpio{ENDSTOP_LINE}
homing_speed: 10
homing_retract_dist: 0

[axis y]
position_min: 0
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio11
homing_speed: 10

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^gpiochip0/gpio12
homing_speed: 5

[motor x]
drive: stepper
step_pin: gpiochip{STEP_CHIP}/gpio{STEP_LINE}
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: {MICROSTEPS}
rotation_distance: {ROTATION_DISTANCE}

[tmc5160 x]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075

[motor y]
drive: stepper
step_pin: gpiochip0/gpio30
dir_pin: gpiochip0/gpio31
enable_pin: !gpiochip0/gpio32
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: gpiochip0/gpio33
dir_pin: gpiochip0/gpio34
enable_pin: !gpiochip0/gpio35
microsteps: 16
rotation_distance: 4

[force_move]
enable_force_move: True

[virtual_sdcard]
path: {world.gcode_dir}
"""


def test_halt_does_not_toggle_a_both_edge_step_pin(sim_world):
    world = sim_world(lambda w: _cfg(w), dual_mcu=False)
    control = world.sim_control("h7")
    control.enable_step_pin_emit()

    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=x ENABLE=1")
    world.gcode_ok(f"FORCE_MOVE STEPPER=x DISTANCE={MICROSTEP_MM} VELOCITY=5")
    world.gcode_ok("M400")
    edges_before_halt = control.gpio_edges(STEP_CHIP, STEP_LINE)
    assert edges_before_halt == 1

    control.set_gpio_input(ENDSTOP_CHIP, ENDSTOP_LINE, 1)
    world.gcode_ok("G28 X")

    assert world.shutdown_line() is None, world.log_tail()
    assert control.gpio_edges(STEP_CHIP, STEP_LINE) == edges_before_halt
