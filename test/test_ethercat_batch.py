"""EtherCAT servos in klippy batch mode.

A servo [stepper_x] shares its rail with a real stepper [stepper_x1] (the
shadow): same kinematics, same input shaper, same flushes. The servo's
sampled targets must sit where the shadow's steps say the commanded
position was.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from ethercat_support import (
    decode_servo_runs,
    dictionary_path,
    lane_samples,
    run_batch,
    step_trace,
)

from klippy.plugins.ethercat import wire
from klippy.plugins.ethercat.stream import Q16
from klippy.stepper import MAX_STEPCOMPRESS_ERROR
from tools.motion_eval import analysis

CYCLE_US = 250
START = (100.0, 100.0, 10.0)

PRINTER_CFG = """
[mcu]
serial: /dev/null

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 5000
square_corner_velocity: 5
max_z_velocity: 10
max_z_accel: 100

[ethercat]
cycle_us: {cycle_us}

[stepper_x]
servo: ethercat
ethercat_slave: 0
ethercat_axis: 0
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: {x_endstop}
position_endstop: 0
position_max: 250

[stepper_x1]
step_pin: PA0
dir_pin: PA1
enable_pin: !PA2
microsteps: 16
rotation_distance: 40

[stepper_y]
step_pin: PB0
dir_pin: !PB1
enable_pin: !PB2
microsteps: 16
rotation_distance: 40
endstop_pin: PB3
position_endstop: 0
position_max: 250

[stepper_z]
step_pin: PC0
dir_pin: PC1
enable_pin: !PC2
microsteps: 16
rotation_distance: 8
endstop_pin: PC3
position_endstop: 0
position_max: 200

[input_shaper]
shaper_type: {shaper}
shaper_freq_x: 50
shaper_freq_y: 40
"""

DELTA_CFG = """
[mcu]
serial: /dev/null

[printer]
kinematics: delta
max_velocity: 300
max_accel: 3000
max_z_velocity: 150
delta_radius: 174.75

[ethercat]

[stepper_a]
servo: ethercat
ethercat_slave: 0
ethercat_axis: 0
encoder_counts_per_rev: 131072
rotation_distance: 40
endstop_pin: ethercat:a
position_endstop: 297.05
arm_length: 333.0

[stepper_b]
step_pin: PF6
dir_pin: PF7
microsteps: 16
rotation_distance: 40
endstop_pin: PE1

[stepper_c]
step_pin: PB3
dir_pin: PB1
microsteps: 16
rotation_distance: 40
endstop_pin: PD2
"""

GCODE = """SET_KINEMATIC_POSITION X={0} Y={1} Z={2}
G1 X150 Y120 F9000
G1 X120 Y160 F12000
G1 X180 Y150 F6000
G4 P300
G1 X110 Y110 F15000
G1 X100 Y100 F3000
M400
""".format(*START)


def servo_config(shaper, x_endstop="ethercat:x"):
    return PRINTER_CFG.format(
        cycle_us=CYCLE_US, shaper=shaper, x_endstop=x_endstop
    )


@pytest.fixture(scope="module", params=["mzv", "smooth_mzv"])
def shadow_run(request, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("ethercat_" + request.param)
    run = run_batch(
        tmp, servo_config(request.param), GCODE, dictionary_path(request)
    )
    assert run.returncode == 0, run.log[-4000:]
    return run


def servo_trajectory(run):
    servo = run.steppers["stepper_x"]
    runs = decode_servo_runs(run.servo_file)
    indices, samples = lane_samples(runs, 0)
    origin = runs[0].origin_mm_q16 / Q16
    counts_per_mm = 1.0 / servo["step_dist"]
    pos = origin + np.array([s.pos_counts for s in samples]) / counts_per_mm
    vel = np.array([s.vel_ff for s in samples]) / counts_per_mm
    t = indices * (CYCLE_US * 1e-6)
    # Representation error of a target: the held and the sampled position
    # are each rounded to a count, and the origin to 1/65536 mm.
    resolution = servo["step_dist"] + 0.5 / Q16
    return runs, t, pos, vel, resolution


def test_runs_abut_and_are_anchored_once(shadow_run):
    run = shadow_run
    runs, t, pos, vel, _ = servo_trajectory(run)
    assert runs[0].flags & wire.FLAG_REANCHOR
    assert not any(r.flags & wire.FLAG_REANCHOR for r in runs[1:])
    assert {r.origin_mm_q16 for r in runs} == {runs[0].origin_mm_q16}
    assert {r.interval_ticks for r in runs} == {CYCLE_US * 1000}
    assert runs[-1].flags & wire.FLAG_TAIL
    assert all(len(r.samples) <= wire.MAX_FILL_CYCLES for r in runs)
    assert np.all(np.isfinite(pos)) and np.all(np.isfinite(vel))
    assert vel[0] == 0.0 and vel[-1] == 0.0


def test_servo_targets_follow_the_shaped_trajectory(shadow_run):
    run = shadow_run
    runs, t, pos, vel, resolution = servo_trajectory(run)
    seg = analysis.Segments.from_moves(run.moves)
    kernel = analysis.Kernel.from_shaper(run.shaper["x"])
    jump_times, jumps = seg.velocity_jumps()
    model = analysis.executed_axis(
        seg, kernel, 0, t[0], CYCLE_US * 1e-6, len(t), jump_times, jumps
    )
    # stepper_x of a cartesian printer is the x coordinate itself. The
    # smoother model is a discrete convolution on a fine grid whose error at
    # a velocity kink is far below one encoder count, so the count
    # resolution is the bound.
    error = np.abs(pos - model.pos)
    worst = int(np.argmax(error))
    assert error[worst] <= resolution, (float(t[worst]), float(error[worst]))


def test_servo_follows_shadow_stepper_steps(shadow_run):
    run = shadow_run
    runs, t, pos, vel, resolution = servo_trajectory(run)
    shadow = run.steppers["stepper_x1"]
    stepper_pos = analysis.zoh_stepper_position(shadow, step_trace(shadow), t)
    # Between steps the shadow is within half a step of the commanded
    # position; a step may come late or early by the stepcompress bound
    # plus one MCU tick, during which the commanded position moves at most
    # |v| times that slack.
    slack = MAX_STEPCOMPRESS_ERROR + 1.0 / shadow["clock_freq"]
    reach = math.ceil(slack / (CYCLE_US * 1e-6)) + 1
    speed = np.abs(vel)
    window = np.lib.stride_tricks.sliding_window_view(
        np.pad(speed, reach, mode="edge"), 2 * reach + 1
    )
    peak = window.max(axis=1)
    bound = 0.5 * shadow["step_dist"] + peak * slack + resolution
    error = np.abs(pos - stepper_pos)
    worst = int(np.argmax(error - bound))
    assert error[worst] <= bound[worst], (
        float(t[worst]),
        float(pos[worst]),
        float(stepper_pos[worst]),
        float(bound[worst]),
    )
    end = START[0]
    assert abs(pos[-1] - end) <= resolution
    assert abs(stepper_pos[-1] - end) <= 0.5 * shadow["step_dist"]


def test_nonlinear_kinematics_is_a_config_error(request, tmp_path):
    run = run_batch(tmp_path, DELTA_CFG, "G4 P1\n", dictionary_path(request))
    assert run.returncode != 0
    assert "EtherCAT servos need kinematics linear" in run.log


def test_mcu_endstop_on_servo_rail_is_a_config_error(request, tmp_path):
    run = run_batch(
        tmp_path,
        servo_config("mzv", x_endstop="PA3"),
        "G4 P1\n",
        dictionary_path(request),
    )
    assert run.returncode != 0
    assert "cannot stop EtherCAT servo stepper_x" in run.log


def test_servo_homing_fails_loudly(request, tmp_path):
    run = run_batch(
        tmp_path, servo_config("mzv"), "G28 X\n", dictionary_path(request)
    )
    assert run.returncode != 0
    assert "servo homing needs the live endpoint" in run.log
