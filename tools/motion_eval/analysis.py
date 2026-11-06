"""Commanded and executed motion from a captured klippy run.

Commanded motion is the toolhead trapq, piecewise constant acceleration.
Executed motion is what kin_shaper.c feeds to the stepper kinematics:
    exec(t) = sum_j w_j * cmd(t + s_j)
with (w_j, s_j) the normalized, centroid-shifted pulses of an input shaper,
or a Gauss-Legendre discretization of an input smoother's polynomial
kernel. The model is checked against every decoded step: itersolve places a
step where the stepper position crosses the midpoint between two step
positions and stepcompress may move it by at most MAX_STEPCOMPRESS_ERROR.
"""

from __future__ import annotations

import dataclasses

import numpy as np
from numpy.polynomial import Polynomial, legendre

from klippy.stepper import MAX_STEPCOMPRESS_ERROR

AXES = "xyz"
CONTIGUOUS_EPS = 1e-9
MIN_VELOCITY_JUMP = 1e-9
GAUSS_NODES = 128
GRID_INTERP_TOLERANCE = 1e-6


class ModelMismatch(AssertionError):
    pass


@dataclasses.dataclass
class Segments:
    t0: np.ndarray
    dur: np.ndarray
    v0: np.ndarray
    accel: np.ndarray
    d0: np.ndarray
    start: np.ndarray
    axes_r: np.ndarray

    @classmethod
    def from_moves(cls, moves):
        rows = []
        for m in moves:
            start = (m["start_x"], m["start_y"], m["start_z"])
            axes_r = (m["axes_r_x"], m["axes_r_y"], m["axes_r_z"])
            phases = (
                (m["accel_t"], m["start_v"], m["accel"]),
                (m["cruise_t"], m["cruise_v"], 0.0),
                (m["decel_t"], m["cruise_v"], -m["accel"]),
            )
            t, d = m["print_time"], 0.0
            for dur, v0, accel in phases:
                if dur > 0.0:
                    rows.append((t, dur, v0, accel, d) + start + axes_r)
                t += dur
                d += (v0 + 0.5 * accel * dur) * dur
        a = np.array(rows, dtype=float)
        order = np.argsort(a[:, 0], kind="stable")
        a = a[order]
        seg = cls(
            t0=a[:, 0],
            dur=a[:, 1],
            v0=a[:, 2],
            accel=a[:, 3],
            d0=a[:, 4],
            start=a[:, 5:8],
            axes_r=a[:, 8:11],
        )
        overlap = seg.t0[1:] < seg.end[:-1] - CONTIGUOUS_EPS
        if overlap.any():
            k = int(np.argmax(overlap))
            raise ModelMismatch(
                "trapq segments overlap at t=%r (previous ends %r)"
                % (seg.t0[k + 1], seg.end[k])
            )
        return seg

    @property
    def end(self):
        return self.t0 + self.dur

    def end_position(self):
        d = (
            self.d0[-1]
            + (self.v0[-1] + 0.5 * self.accel[-1] * self.dur[-1])
            * (self.dur[-1])
        )
        return self.start[-1] + self.axes_r[-1] * d

    def eval(self, t):
        t = np.asarray(t, dtype=float)
        idx = np.clip(np.searchsorted(self.t0, t, side="right") - 1, 0, None)
        tau = t - self.t0[idx]
        dur = self.dur[idx]
        inside = (tau >= 0.0) & (tau < dur)
        tau_c = np.clip(tau, 0.0, dur)
        v0, a = self.v0[idx], self.accel[idx]
        dist = self.d0[idx] + (v0 + 0.5 * a * tau_c) * tau_c
        r = self.axes_r[idx]
        pos = self.start[idx] + r * dist[:, None]
        vel = r * np.where(inside, v0 + a * tau, 0.0)[:, None]
        acc = r * np.where(inside, a, 0.0)[:, None]
        return pos, vel, acc

    def velocity_jumps(self):
        v_start = self.axes_r * self.v0[:, None]
        v_end = self.axes_r * (self.v0 + self.accel * self.dur)[:, None]
        times, dvs = [self.t0[0]], [v_start[0]]
        for k in range(len(self.t0) - 1):
            if self.t0[k + 1] - self.end[k] <= CONTIGUOUS_EPS:
                times.append(self.t0[k + 1])
                dvs.append(v_start[k + 1] - v_end[k])
            else:
                times += [self.end[k], self.t0[k + 1]]
                dvs += [-v_end[k], v_start[k + 1]]
        times.append(self.end[-1])
        dvs.append(-v_end[-1])
        times, dvs = np.array(times), np.array(dvs)
        keep = np.abs(dvs).max(axis=1) > MIN_VELOCITY_JUMP
        return times[keep], dvs[keep]


@dataclasses.dataclass
class Kernel:
    kind: str
    weights: np.ndarray
    shifts: np.ndarray
    density: Polynomial | None = None
    half_width: float = 0.0
    t_offs: float = 0.0

    @classmethod
    def identity(cls):
        return cls("none", np.ones(1), np.zeros(1))

    @classmethod
    def from_shaper(cls, state):
        if state is None:
            return cls.identity()
        if state["kind"] == "pulses":
            if not state["A"]:
                return cls.identity()
            A, T = np.array(state["A"]), np.array(state["T"])
            t_offs = float(np.dot(A, T) / A.sum())
            return cls("pulses", A / A.sum(), t_offs - T)
        if not state["smooth_time"]:
            return cls.identity()
        t_sm = state["smooth_time"]
        hst = 0.5 * t_sm
        density = Polynomial(
            [c / t_sm ** (i + 1) for i, c in enumerate(state["coeffs"])]
        )
        antiderivative = density.integ()
        density = density / (antiderivative(hst) - antiderivative(-hst))
        first_moment = (density * Polynomial([0.0, 1.0])).integ()
        t_offs = float(first_moment(hst) - first_moment(-hst))
        nodes, gauss_w = legendre.leggauss(GAUSS_NODES)
        tau = nodes * hst
        return cls(
            "smoother",
            gauss_w * hst * density(tau),
            t_offs - tau,
            density=density,
            half_width=hst,
            t_offs=t_offs,
        )

    @property
    def support(self):
        return float(self.shifts.min()), float(self.shifts.max())

    def jump_rate(self, dt):
        inside = np.abs(dt + self.t_offs) <= self.half_width
        return np.where(inside, self.density(dt + self.t_offs), 0.0)

    def describe(self):
        out = {
            "kind": self.kind,
            "support_s": list(self.support),
            "l1_norm": float(np.abs(self.weights).sum()),
        }
        if self.kind == "pulses":
            out["pulses"] = [
                [float(s), float(w)] for s, w in zip(self.shifts, self.weights)
            ]
        if self.kind == "smoother":
            grid = np.linspace(-self.half_width, self.half_width, 2001)
            out["peak_density_per_s"] = float(np.abs(self.density(grid)).max())
            out["t_offs_s"] = self.t_offs
        return out


@dataclasses.dataclass
class AxisMotion:
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    jump_times: np.ndarray
    jumps: np.ndarray


def executed_axis(seg, kernel, axis, t, cmd_jump_times, cmd_jumps):
    pos = np.zeros_like(t)
    vel = np.zeros_like(t)
    acc = np.zeros_like(t)
    for w, s in zip(kernel.weights, kernel.shifts):
        p, v, a = seg.eval(t + s)
        pos += w * p[:, axis]
        vel += w * v[:, axis]
        acc += w * a[:, axis]
    dv = cmd_jumps[:, axis]
    if kernel.kind == "smoother":
        lo = t[0]
        dt_grid = t[1] - t[0]
        reach = kernel.half_width + abs(kernel.t_offs)
        for tj, dvj in zip(cmd_jump_times, dv):
            if not dvj:
                continue
            i0 = max(0, int(np.floor((tj - reach - lo) / dt_grid)))
            i1 = min(len(t), int(np.ceil((tj + reach - lo) / dt_grid)) + 1)
            acc[i0:i1] += dvj * kernel.jump_rate(t[i0:i1] - tj)
        return AxisMotion(pos, vel, acc, np.zeros(0), np.zeros(0))
    jt = (cmd_jump_times[:, None] - kernel.shifts[None, :]).ravel()
    jv = (dv[:, None] * kernel.weights[None, :]).ravel()
    order = np.argsort(jt, kind="stable")
    jt, jv = jt[order], jv[order]
    if len(jt):
        group = np.concatenate([[0], np.cumsum(np.diff(jt) > CONTIGUOUS_EPS)])
        jv = np.bincount(group, weights=jv)
        jt = jt[np.concatenate([[0], np.flatnonzero(np.diff(group)) + 1])]
    keep = np.abs(jv) > MIN_VELOCITY_JUMP
    return AxisMotion(pos, vel, acc, jt[keep], jv[keep])


def stepper_positions_mm(stepper, trace):
    return (
        stepper["commanded_pos_at_mcu_zero"]
        + trace.positions * stepper["step_dist"]
    )


def check_steps_against_model(name, stepper, trace, t, model_pos, model_vel):
    if not len(trace.clocks):
        return {"steps": 0, "max_error_mm": 0.0}
    freq = stepper["clock_freq"]
    times = trace.clocks / freq
    if times[0] < t[0] or times[-1] > t[-1]:
        raise ModelMismatch(
            "%s steps [%r, %r] outside model grid [%r, %r]"
            % (name, times[0], times[-1], t[0], t[-1])
        )
    after = stepper_positions_mm(stepper, trace)
    before = np.concatenate(
        [[stepper["commanded_pos_at_mcu_zero"]], after[:-1]]
    )
    midpoint = 0.5 * (before + after)
    predicted = np.interp(times, t, model_pos)
    time_slack = MAX_STEPCOMPRESS_ERROR + 1.0 / freq
    speed = np.maximum.reduce(
        [
            np.abs(np.interp(times + d, t, model_vel))
            for d in (-time_slack, 0.0, time_slack)
        ]
    )
    allowed = speed * time_slack + GRID_INTERP_TOLERANCE
    error = np.abs(predicted - midpoint)
    worst = int(np.argmax(error - allowed))
    if error[worst] > allowed[worst]:
        raise ModelMismatch(
            "%s step %d at t=%r: model %r vs step midpoint %r (allowed %r);"
            " the shaper model no longer matches kin_shaper.c"
            % (
                name,
                worst,
                float(times[worst]),
                float(predicted[worst]),
                float(midpoint[worst]),
                float(allowed[worst]),
            )
        )
    return {
        "steps": int(len(times)),
        "max_error_mm": float(error.max()),
        "max_allowed_mm": float(allowed.max()),
    }


def zoh_stepper_position(stepper, trace, t):
    if not len(trace.clocks):
        return np.full_like(t, stepper["commanded_pos_at_mcu_zero"])
    times = trace.clocks / stepper["clock_freq"]
    idx = np.searchsorted(times, t, side="right") - 1
    counts = np.where(idx >= 0, trace.positions[np.clip(idx, 0, None)], 0)
    return stepper["commanded_pos_at_mcu_zero"] + counts * stepper["step_dist"]


def xyz_from_steppers(steppers, stepper_pos):
    names = list(steppers)
    M = np.array([steppers[n]["linear_map"]["row"] for n in names])
    offset = np.array([steppers[n]["linear_map"]["offset"] for n in names])
    if np.linalg.matrix_rank(M) < 3:
        raise ModelMismatch("stepper linear maps do not span xyz: %r" % M)
    S = np.stack([stepper_pos[n] for n in names], axis=-1) - offset
    return np.linalg.lstsq(M, S.T, rcond=None)[0].T


def path_deviation(seg, t, xy, window):
    best = np.full(len(t), np.inf)
    p0 = seg.start[:, :2] + seg.axes_r[:, :2] * seg.d0[:, None]
    d1 = seg.d0 + (seg.v0 + 0.5 * seg.accel * seg.dur) * seg.dur
    p1 = seg.start[:, :2] + seg.axes_r[:, :2] * d1[:, None]
    prev_end = np.concatenate([[-np.inf], seg.end[:-1]])
    next_start = np.concatenate([seg.t0[1:], [np.inf]])
    lo = np.searchsorted(t, np.minimum(seg.t0 - window, prev_end), side="left")
    hi = np.searchsorted(t, np.maximum(seg.end + window, next_start), "right")
    for k in range(len(seg.t0)):
        pts = xy[lo[k] : hi[k]]
        if not len(pts):
            continue
        ab = p1[k] - p0[k]
        L2 = float(np.dot(ab, ab))
        u = (
            np.zeros(len(pts))
            if L2 == 0.0
            else np.clip(((pts - p0[k]) @ ab) / L2, 0.0, 1.0)
        )
        d = np.linalg.norm(pts - (p0[k] + u[:, None] * ab), axis=1)
        np.minimum(best[lo[k] : hi[k]], d, out=best[lo[k] : hi[k]])
    i = int(np.argmax(best))
    return {
        "max_mm": float(best[i]),
        "at_time_s": float(t[i]),
        "at_xy": [float(v) for v in xy[i]],
    }
