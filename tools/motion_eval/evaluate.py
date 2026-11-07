"""printer.cfg + gcode -> commanded vs executed motion metrics (JSON-able)."""

from __future__ import annotations

import dataclasses
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
    out = {}
    for axis, m in motion.items():
        jumps = np.abs(m.jumps)
        out[axis] = {
            "peak_velocity_mm_s": float(np.abs(m.vel).max()),
            "peak_accel_mm_s2": float(np.abs(m.acc).max()),
            "max_velocity_jump_mm_s": float(jumps.max(initial=0.0)),
            "p99_velocity_jump_mm_s": float(np.percentile(jumps, 99))
            if len(jumps)
            else 0.0,
        }
    return out


def _worst(values, t, xy):
    i = int(np.argmax(values))
    return {
        "max_mm": float(values[i]),
        "at_time_s": float(t[i]),
        "at_xy": [float(v) for v in xy[i]],
    }


def _phase_times(seg):
    accel = seg.accel > 0.0
    decel = seg.accel < 0.0
    return {
        "accel_s": float(seg.dur[accel].sum()),
        "cruise_s": float(seg.dur[~accel & ~decel].sum()),
        "decel_s": float(seg.dur[decel].sum()),
        "idle_s": float(seg.end[-1] - seg.t0[0] - seg.dur.sum()),
    }


def evaluate(
    config_path,
    gcode: str,
    start_xyz,
    dt: float = DEFAULT_DT,
    sample_dt: float | None = DEFAULT_SAMPLE_DT,
    mcu_config: str = mcu_dict.DEFAULT_MCU_CONFIG,
    klippy_root=run.REPO_ROOT,
) -> dict:
    raw = run.run_klippy(config_path, gcode, start_xyz, mcu_config, klippy_root)
    return analyze(raw, dt, sample_dt)


def kernel_support(kernels):
    """Union of the axis kernels' shift ranges."""
    return (
        min(k.support[0] for k in kernels.values()),
        max(k.support[1] for k in kernels.values()),
    )


@dataclasses.dataclass
class Epoch:
    """Moves [first, last) planned under one input shaper setting, its
    per-axis kernels, and the span of time its moves reach through them."""

    kernels: dict
    first: int
    last: int
    window: tuple


def shaper_epochs(cap):
    moves = cap["moves"]
    epochs = []
    first = 0
    for i in range(1, len(moves) + 1):
        if (
            i < len(moves)
            and moves[i]["shaper_epoch"] == moves[first]["shaper_epoch"]
        ):
            continue
        state = cap["shapers"][moves[first]["shaper_epoch"]]
        kernels = {
            axis: analysis.Kernel.from_shaper(state.get(axis))
            for axis in analysis.AXES
        }
        shift_lo, shift_hi = kernel_support(kernels)
        last = moves[i - 1]
        end = (
            last["print_time"]
            + last["accel_t"]
            + last["cruise_t"]
            + last["decel_t"]
        )
        epochs.append(
            Epoch(
                kernels,
                first,
                i,
                (moves[first]["print_time"] - shift_hi, end - shift_lo),
            )
        )
        first = i
    for a, b in zip(epochs, epochs[1:]):
        if b.window[0] < a.window[1]:
            raise analysis.ModelMismatch(
                "shaped motion of moves %d-%d (until %r) overlaps that of"
                " moves %d-%d (from %r) across an input shaper change"
                % (a.first, a.last, a.window[1], b.first, b.last, b.window[0])
            )
    return epochs


def executed_motion(cap, dt):
    """Commanded segments, shaper epochs, sample grid start and count, and
    the executed (shaped) motion of every axis on that grid. Between epochs
    the toolhead is at rest, so each sample takes the motion of the epoch
    whose window holds it, or the commanded position."""
    seg = analysis.Segments.from_moves(cap["moves"])
    epochs = shaper_epochs(cap)
    t_start = epochs[0].window[0] - dt
    n = int(math.ceil((epochs[-1].window[1] + dt - t_start) / dt)) + 1
    t = t_start + dt * np.arange(n)
    pos, _, _ = seg.eval(t)
    motion = {
        axis: analysis.AxisMotion(
            pos[:, i].copy(), np.zeros(n), np.zeros(n), [], []
        )
        for i, axis in enumerate(analysis.AXES)
    }
    for epoch in epochs:
        part = analysis.Segments.from_moves(
            cap["moves"][epoch.first : epoch.last]
        )
        jump_t, jump_v = part.velocity_jumps()
        inside = (t >= epoch.window[0]) & (t <= epoch.window[1])
        for i, axis in enumerate(analysis.AXES):
            m = analysis.executed_axis(
                part, epoch.kernels[axis], i, t_start, dt, n, jump_t, jump_v
            )
            out = motion[axis]
            out.pos[inside] = m.pos[inside]
            out.vel[inside] = m.vel[inside]
            out.acc[inside] = m.acc[inside]
            out.jump_times.append(m.jump_times)
            out.jumps.append(m.jumps)
    for out in motion.values():
        out.jump_times = np.concatenate(out.jump_times)
        out.jumps = np.concatenate(out.jumps)
    _, jump_v = seg.velocity_jumps()
    return seg, epochs, t_start, n, jump_v, motion


def analyze(raw, dt=DEFAULT_DT, sample_dt=DEFAULT_SAMPLE_DT) -> dict:
    cap = raw.capture
    seg, epochs, t_start, n, jump_v, motion = executed_motion(cap, dt)
    t = t_start + dt * np.arange(n)
    exec_pos = np.stack([motion[a].pos for a in analysis.AXES], axis=-1)
    exec_vel = np.stack([motion[a].vel for a in analysis.AXES], axis=-1)
    exec_acc = np.stack([motion[a].acc for a in analysis.AXES], axis=-1)
    exec_jump = np.array(
        [np.abs(motion[a].jumps).max(initial=0.0) for a in analysis.AXES]
    )

    steppers = cap["steppers"]
    step_checks, zoh, final_stepper_pos = {}, {}, {}
    for name, st in steppers.items():
        trace = raw.traces[name]
        row = np.array(st["linear_map"]["row"])
        offset = st["linear_map"]["offset"]
        step_checks[name] = analysis.check_steps_against_model(
            name,
            st,
            trace,
            t,
            offset + exec_pos @ row,
            exec_vel @ row,
            exec_acc @ row,
            float(np.abs(row) @ exec_jump),
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

    window = max(abs(s) for e in epochs for s in kernel_support(e.kernels)) + dt
    exec_xy = exec_pos[:, :2]
    to_cmd = analysis.distance_to_commanded(seg, t, exec_xy, window)
    steps_to_cmd = analysis.distance_to_commanded(
        seg, t, steps_xyz[:, :2], window
    )
    vertices = seg.vertices()
    corner_dev, _, _ = analysis.corner_deviations(
        vertices, t, exec_xy, to_cmd, window
    )
    corners = analysis.deviation_summary(corner_dev)
    if len(corner_dev):
        k = int(np.argmax(corner_dev))
        corners["worst"] = {
            "time_s": float(vertices["t_arrive"][k]),
            "xy": vertices["xy"][k].tolist(),
            "turn_deg": float(vertices["turn_deg"][k]),
            "junction_speed_mm_s": float(vertices["speed"][k]),
        }
    result = {
        "print_time_s": float(seg.end[-1] - seg.t0[0]),
        "phase_time": _phase_times(seg),
        "settings": cap["settings"],
        "timing": cap["timing"],
        "moves_planned": len(cap["moves"]),
        "corner_deviation": cap["corner_deviation"],
        "shapers": [
            {
                axis: dict(
                    cap["shapers"][cap["moves"][e.first]["shaper_epoch"]].get(
                        axis, {}
                    ),
                    model=k.describe(),
                )
                for axis, k in e.kernels.items()
            }
            for e in epochs
        ],
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
            "xy_path_deviation_model": _worst(to_cmd, t, exec_xy),
            "xy_path_deviation_steps": _worst(
                steps_to_cmd, t, steps_xyz[:, :2]
            ),
            "corners": corners,
            "vertices": {
                "t_arrive": vertices["t_arrive"].tolist(),
                "t_leave": vertices["t_leave"].tolist(),
                "speed_mm_s": vertices["speed"].tolist(),
                "deviation_mm": corner_dev.tolist(),
            },
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
        cmd_pos, cmd_vel, cmd_acc = seg.eval(t[sl])
        result["samples"] = {
            "t": t[sl].tolist(),
            "commanded": _xyz_series(cmd_pos, cmd_vel, cmd_acc),
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
