"""Helpers shared by the EtherCAT servo tests: trapq construction, batch
klippy runs and decoding of the servo frame file."""

from __future__ import annotations

import dataclasses
import json
import os
import pathlib
import subprocess
import sys

import numpy as np

from klippy import chelper
from klippy.extras import shaper_defs
from klippy.plugins.ethercat import wire
from tools.motion_eval.capture import TRAPQ_APPEND_FIELDS
from tools.motion_eval.stepdecode import StepTrace

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
BATCH_RUNNER = (
    pathlib.Path(__file__).resolve().parent / "ethercat_batch_runner.py"
)
DICTIONARY = "stm32h723.dict"


def dictionary_path(request):
    path = pathlib.Path.cwd() / request.config.getoption("dictdir") / DICTIONARY
    assert path.is_file(), "MCU dictionary %s missing (set DICTDIR)" % (path,)
    return path


# Shaper parameters as input_shaper.py computes them
def shaper_state(shaper_type, freq):
    for cfg in shaper_defs.INPUT_SHAPERS:
        if cfg.name == shaper_type:
            A, T = cfg.init_func(freq, shaper_defs.DEFAULT_DAMPING_RATIO)
            return {"kind": "pulses", "type": shaper_type, "A": A, "T": T}
    for cfg in shaper_defs.INPUT_SMOOTHERS:
        if cfg.name == shaper_type:
            coeffs, smooth_time = cfg.init_func(freq, normalize_coeffs=False)
            return {
                "kind": "smoother",
                "type": shaper_type,
                "coeffs": list(coeffs),
                "smooth_time": smooth_time,
            }
    raise ValueError("unknown shaper %s" % (shaper_type,))


def apply_shaper(sk, axis, state):
    ffi_main, ffi_lib = chelper.get_ffi()
    if state["kind"] == "pulses":
        res = ffi_lib.input_shaper_set_shaper_params(
            sk, axis.encode(), len(state["A"]), state["A"], state["T"]
        )
    else:
        res = ffi_lib.input_shaper_set_smoother_params(
            sk,
            axis.encode(),
            len(state["coeffs"]),
            state["coeffs"],
            state["smooth_time"],
        )
    assert res == 0


def corner_path_moves(rng, start, corners, accel, cruise_v, dwell_after=None):
    """trapq_append argument dicts for a path through random corners.

    Moves are contiguous, start and end at rest and enter every corner at a
    random nonzero speed, so the commanded velocity jumps at each corner.
    An optional dwell inserts a stationary gap (a null move in the trapq).
    """
    moves = []
    pos = np.array(start, dtype=float)
    v_in = 0.0
    print_time = 1.0
    for k in range(corners):
        direction = rng.normal(size=3) * (1.0, 1.0, 0.1)
        direction /= np.linalg.norm(direction)
        length = rng.uniform(20.0, 40.0)
        last = k == corners - 1
        v_out = 0.0 if last or k == dwell_after else rng.uniform(5.0, cruise_v)
        accel_t = (cruise_v - v_in) / accel
        decel_t = (cruise_v - v_out) / accel
        accel_d = (v_in + cruise_v) * 0.5 * accel_t
        decel_d = (v_out + cruise_v) * 0.5 * decel_t
        cruise_t = (length - accel_d - decel_d) / cruise_v
        assert cruise_t > 0.0
        moves.append(
            dict(
                zip(
                    TRAPQ_APPEND_FIELDS,
                    (
                        print_time,
                        accel_t,
                        cruise_t,
                        decel_t,
                        *pos,
                        *direction,
                        v_in,
                        cruise_v,
                        accel,
                    ),
                )
            )
        )
        print_time += accel_t + cruise_t + decel_t
        pos = pos + direction * length
        v_in = v_out
        if k == dwell_after:
            print_time += rng.uniform(0.05, 0.2)
    return moves


def build_trapq(moves):
    ffi_main, ffi_lib = chelper.get_ffi()
    tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    for m in moves:
        ffi_lib.trapq_append(tq, *(m[f] for f in TRAPQ_APPEND_FIELDS))
    return tq


# Batch klippy runs
@dataclasses.dataclass
class BatchRun:
    returncode: int
    log: str
    steppers: dict
    moves: list
    shaper: dict
    servo_file: pathlib.Path


def run_batch(tmp_path, config_text, gcode, dictionary):
    config = tmp_path / "printer.cfg"
    config.write_text(config_text)
    gcode_path = tmp_path / "input.gcode"
    gcode_path.write_text(gcode)
    output = tmp_path / "mcu.out"
    log = tmp_path / "klippy.log"
    capture = tmp_path / "capture.json"
    cmd = [
        sys.executable,
        str(BATCH_RUNNER),
        str(capture),
        str(config),
        "-i",
        str(gcode_path),
        "-o",
        str(output),
        "-d",
        str(dictionary),
        "-l",
        str(log),
    ]
    env = dict(os.environ, PYTHONPATH=str(REPO_ROOT))
    res = subprocess.run(
        cmd, cwd=REPO_ROOT, env=env, capture_output=True, text=True
    )
    captured = json.loads(capture.read_text()) if capture.exists() else {}
    log_text = log.read_text() if log.exists() else ""
    return BatchRun(
        res.returncode,
        log_text + res.stderr,
        captured.get("steppers", {}),
        captured.get("moves", []),
        captured.get("shaper", {}),
        pathlib.Path(str(output) + ".ethercat"),
    )


def decode_servo_runs(path):
    """All lane runs in the frame file, in push order."""
    runs = []
    for channel, payload in wire.iter_frames(path.read_bytes()):
        assert channel == wire.CHANNEL_PIECES
        kind, version, cid, body = wire.decode_message(payload)
        assert kind == wire.Kind.PUSH_SAMPLE_RUNS
        assert version == wire.MESSAGE_VERSION
        runs.extend(wire.decode_push_sample_runs(body))
    return runs


def lane_samples(runs, slot_idx):
    """(indices, samples) of one slot, checking the runs abut."""
    lane = [r for r in runs if r.slot_idx == slot_idx]
    assert lane, "no runs for slot %d" % (slot_idx,)
    indices, samples = [], []
    for prev, run in zip([None] + lane, lane):
        if prev is not None:
            assert run.start_index == prev.end_index, (
                "slot %d: run at %d does not abut previous end %d"
                % (slot_idx, run.start_index, prev.end_index)
            )
        indices.extend(range(run.start_index, run.end_index))
        samples.extend(run.samples)
    return np.array(indices), samples


def step_trace(stepper):
    """Step clocks and resulting MCU positions from the stepcompress history.

    A queue_step entry of count n steps first at first_clock, each later
    step interval + i * add ticks after the one before it (src/stepper.c).
    """
    clocks, positions = [], []
    for h in stepper["history"]:
        assert h["add2"] == 0 and h["shift"] == 0, "high precision steps"
        n = abs(h["step_count"])
        if not n:
            continue
        k = np.arange(n, dtype=np.int64)
        clocks.append(
            h["first_clock"] + k * h["interval"] + h["add"] * k * (k + 1) // 2
        )
        direction = 1 if h["step_count"] > 0 else -1
        positions.append(h["start_position"] + direction * (k + 1))
    assert clocks, "stepper took no steps"
    return StepTrace(np.concatenate(clocks), np.concatenate(positions))


def isclose_to_any(t, events, eps):
    if not len(events):
        return np.zeros(len(t), dtype=bool)
    events = np.sort(np.asarray(events, dtype=float))
    idx = np.clip(np.searchsorted(events, t), 1, len(events) - 1)
    near = np.minimum(np.abs(t - events[idx - 1]), np.abs(t - events[idx]))
    return near <= eps


# A cartesian printer whose X is an EtherCAT servo sharing its rail with a
# real stepper (the shadow)
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
{x_homing}
{x_shadow}
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


GCODE = """SET_KINEMATIC_POSITION X={0} Y={1} Z={2}
G1 X150 Y120 F9000
G1 X120 Y160 F12000
G1 X180 Y150 F6000
G4 P300
G1 X110 Y110 F15000
G1 X100 Y100 F3000
M400
""".format(*START)


X_SHADOW = """[stepper_x1]
step_pin: PA0
dir_pin: PA1
enable_pin: !PA2
microsteps: 16
rotation_distance: 40
"""


def servo_config(
    shaper,
    x_endstop="ethercat:x",
    cycle_us=CYCLE_US,
    x_shadow=True,
    x_homing="homing_torque: 50\nhoming_retract_dist: 0",
):
    return PRINTER_CFG.format(
        cycle_us=cycle_us,
        shaper=shaper,
        x_endstop=x_endstop,
        x_shadow=X_SHADOW if x_shadow else "",
        x_homing=x_homing,
    )
