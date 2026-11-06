"""printer.cfg + gcode -> commanded vs executed motion metrics (JSON-able)."""

from __future__ import annotations

import math

import numpy as np

from . import analysis, mcu_dict, run

DEFAULT_DT = 25e-6
DEFAULT_SAMPLE_DT = 1e-3


def _per_axis_commanded(seg, jumps):
    speed_ends = np.maximum(
        np.abs(seg.v0), np.abs(seg.v0 + seg.accel * seg.dur)
    )
    out = {}
    for i, axis in enumerate(analysis.AXES):
        r = np.abs(seg.axes_r[:, i])
        out[axis] = {
            "peak_velocity_mm_s": float((speed_ends * r).max()),
            "peak_accel_mm_s2": float((np.abs(seg.accel) * r).max()),
            "max_velocity_jump_mm_s": float(
                np.abs(jumps[:, i]).max(initial=0.0)
            ),
            "total_velocity_jump_mm_s": float(np.abs(jumps[:, i]).sum()),
        }
    return out


def _per_axis_executed(motion):
    return {
        axis: {
            "peak_velocity_mm_s": float(np.abs(m.vel).max()),
            "peak_accel_mm_s2": float(np.abs(m.acc).max()),
            "max_velocity_jump_mm_s": float(np.abs(m.jumps).max(initial=0.0)),
        }
        for axis, m in motion.items()
    }


def evaluate(
    config_path,
    gcode: str,
    start_xyz,
    dt: float = DEFAULT_DT,
    sample_dt: float | None = DEFAULT_SAMPLE_DT,
    mcu_config: str = mcu_dict.DEFAULT_MCU_CONFIG,
) -> dict:
    raw = run.run_klippy(config_path, gcode, start_xyz, mcu_config)
    cap = raw.capture
    seg = analysis.Segments.from_moves(cap["moves"])
    kernels = {
        axis: analysis.Kernel.from_shaper(cap["shaper"].get(axis))
        for axis in analysis.AXES
    }
    shift_lo = min(k.support[0] for k in kernels.values())
    shift_hi = max(k.support[1] for k in kernels.values())
    t_start = seg.t0[0] - shift_hi - dt
    t_end = seg.end[-1] - shift_lo + dt
    t = t_start + dt * np.arange(int(math.ceil((t_end - t_start) / dt)) + 1)

    cmd_pos, cmd_vel, cmd_acc = seg.eval(t)
    jump_t, jump_v = seg.velocity_jumps()
    motion = {
        axis: analysis.executed_axis(seg, kernels[axis], i, t, jump_t, jump_v)
        for i, axis in enumerate(analysis.AXES)
    }
    exec_pos = np.stack([motion[a].pos for a in analysis.AXES], axis=-1)
    exec_vel = np.stack([motion[a].vel for a in analysis.AXES], axis=-1)
    exec_acc = np.stack([motion[a].acc for a in analysis.AXES], axis=-1)

    steppers = cap["steppers"]
    step_checks, zoh, final_stepper_pos = {}, {}, {}
    for name, st in steppers.items():
        trace = raw.traces[name]
        row = np.array(st["linear_map"]["row"])
        offset = st["linear_map"]["offset"]
        step_checks[name] = analysis.check_steps_against_model(
            name, st, trace, t, offset + exec_pos @ row, exec_vel @ row
        )
        decoded_final = int(trace.positions[-1]) if len(trace.positions) else 0
        if decoded_final != st["final_mcu_position"]:
            raise analysis.ModelMismatch(
                "%s decoded final mcu position %d != klippy %d"
                % (name, decoded_final, st["final_mcu_position"])
            )
        zoh[name] = analysis.zoh_stepper_position(st, trace, t)
        final_stepper_pos[name] = np.array(
            [st["commanded_pos_at_mcu_zero"] + decoded_final * st["step_dist"]]
        )
    steps_xyz = analysis.xyz_from_steppers(steppers, zoh)
    steps_final = analysis.xyz_from_steppers(steppers, final_stepper_pos)[0]

    window = max(abs(shift_lo), abs(shift_hi)) + dt
    result = {
        "print_time_s": float(seg.end[-1] - seg.t0[0]),
        "settings": cap["settings"],
        "shaper": {
            axis: dict(cap["shaper"].get(axis, {}), model=k.describe())
            for axis, k in kernels.items()
        },
        "commanded": {
            "moves": cap["moves"],
            "segments": int(len(seg.t0)),
            "peak_speed_mm_s": float(
                max(max(m["start_v"], m["cruise_v"]) for m in cap["moves"])
            ),
            "per_axis": _per_axis_commanded(seg, jump_v),
            "final_position": seg.end_position().tolist(),
        },
        "executed": {
            "per_axis": _per_axis_executed(motion),
            "final_position_model": exec_pos[-1].tolist(),
            "final_position_steps": steps_final.tolist(),
            "xy_path_deviation_model": analysis.path_deviation(
                seg, t, exec_pos[:, :2], window
            ),
            "xy_path_deviation_steps": analysis.path_deviation(
                seg, t, steps_xyz[:, :2], window
            ),
        },
        "steppers": {
            name: dict(
                step_dist=st["step_dist"],
                final_mcu_position=st["final_mcu_position"],
                model_vs_steps=step_checks[name],
            )
            for name, st in steppers.items()
        },
        "grid_dt_s": dt,
    }
    if sample_dt:
        stride = max(1, int(round(sample_dt / dt)))
        sl = slice(None, None, stride)
        result["samples"] = {
            "t": t[sl].tolist(),
            "commanded": _xyz_series(cmd_pos[sl], cmd_vel[sl], cmd_acc[sl]),
            "executed": _xyz_series(exec_pos[sl], exec_vel[sl], exec_acc[sl]),
            "steps_position": {
                a: steps_xyz[sl, i].tolist()
                for i, a in enumerate(analysis.AXES)
            },
        }
    return result


def _xyz_series(pos, vel, acc):
    return {
        kind: {a: arr[:, i].tolist() for i, a in enumerate(analysis.AXES)}
        for kind, arr in (("position", pos), ("velocity", vel), ("accel", acc))
    }
