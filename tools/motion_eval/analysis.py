"""Commanded and executed motion from a captured klippy run.

Commanded motion is the toolhead trapq, piecewise constant acceleration.
Executed motion is what kin_shaper.c feeds to the stepper kinematics:
    exec(t) = sum_j w_j * cmd(t + s_j)
with (w_j, s_j) the normalized, centroid-shifted pulses of an input shaper,
or, for an input smoother, its polynomial kernel sampled on the time grid
(a Riemann sum of the convolution, evaluated by FFT). The model is checked
against every decoded step: itersolve places a step where the stepper
position crosses the midpoint between two step positions and stepcompress
may move it by at most MAX_STEPCOMPRESS_ERROR.
"""

from __future__ import annotations

import dataclasses

import numpy as np
from numpy.polynomial import Polynomial

from klippy.stepper import MAX_STEPCOMPRESS_ERROR

AXES = "xyz"
CONTIGUOUS_EPS = 1e-9
MIN_VELOCITY_JUMP = 1e-9
STRAIGHT_COS = 1.0 - 1e-9
FLOAT_TOLERANCE_MM = 1e-9
KERNEL_DESCRIBE_POINTS = 2001
SMOOTHER_OVERSAMPLE = 4


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
    move: np.ndarray

    @classmethod
    def from_moves(cls, moves):
        rows = []
        for index, m in enumerate(moves):
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
                    rows.append(
                        (t, dur, v0, accel, d) + start + axes_r + (index,)
                    )
                t += dur
                d += (v0 + 0.5 * accel * dur) * dur
        a = np.array(rows, dtype=float)
        a = a[np.argsort(a[:, 0], kind="stable")]
        seg = cls(
            t0=a[:, 0],
            dur=a[:, 1],
            v0=a[:, 2],
            accel=a[:, 3],
            d0=a[:, 4],
            start=a[:, 5:8],
            axes_r=a[:, 8:11],
            move=a[:, 11].astype(int),
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

    @property
    def d1(self):
        return self.d0 + (self.v0 + 0.5 * self.accel * self.dur) * self.dur

    def end_position(self):
        return self.start[-1] + self.axes_r[-1] * self.d1[-1]

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
        contiguous = self.t0[1:] - self.end[:-1] <= CONTIGUOUS_EPS
        times = [self.t0[:1]]
        dvs = [v_start[:1]]
        times.append(np.where(contiguous, self.t0[1:], self.end[:-1]))
        dvs.append(
            np.where(contiguous[:, None], v_start[1:] - v_end[:-1], -v_end[:-1])
        )
        gap = ~contiguous
        times.append(self.t0[1:][gap])
        dvs.append(v_start[1:][gap])
        times.append(self.end[-1:])
        dvs.append(-v_end[-1:])
        times, dvs = np.concatenate(times), np.concatenate(dvs)
        order = np.argsort(times, kind="stable")
        times, dvs = times[order], dvs[order]
        keep = np.abs(dvs).max(axis=1) > MIN_VELOCITY_JUMP
        return times[keep], dvs[keep]

    def vertices(self):
        last = np.flatnonzero(np.diff(self.move) != 0)
        nxt = last + 1
        cos = np.einsum("ij,ij->i", self.axes_r[last], self.axes_r[nxt])
        corner = cos < STRAIGHT_COS
        last, nxt, cos = last[corner], nxt[corner], cos[corner]
        pos = self.start[last] + self.axes_r[last] * self.d1[last][:, None]
        return {
            "t_arrive": self.end[last],
            "t_leave": self.t0[nxt],
            "xy": pos[:, :2],
            "turn_deg": np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))),
            "speed": self.v0[nxt],
        }


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
        return cls(
            "smoother",
            np.ones(1),
            np.array([t_offs - hst, t_offs + hst]),
            density=density,
            half_width=hst,
            t_offs=t_offs,
        )

    @property
    def support(self):
        return float(self.shifts.min()), float(self.shifts.max())

    def kernel_at_shift(self, s):
        tau = self.t_offs - s
        inside = np.abs(tau) <= self.half_width
        return np.where(inside, self.density(tau), 0.0)

    def grid_weights(self, dt):
        lo, hi = self.support
        m = np.arange(int(np.ceil(lo / dt)), int(np.floor(hi / dt)) + 1)
        w = self.kernel_at_shift(m * dt)
        return int(m[0]), w / w.sum()

    def second_moment(self):
        if self.kind == "smoother":
            tau = np.linspace(-self.half_width, self.half_width, 20001)
            w = self.density(tau)
            s = self.t_offs - tau
            return float(np.trapezoid(w * s * s, tau))
        return float(np.dot(self.weights, self.shifts**2))

    def describe(self):
        out = {
            "kind": self.kind,
            "support_s": list(self.support),
            "sigma_s": self.second_moment() ** 0.5,
        }
        if self.kind == "smoother":
            tau = np.linspace(
                -self.half_width, self.half_width, KERNEL_DESCRIBE_POINTS
            )
            w = self.density(tau)
            out["l1_norm"] = float(np.trapezoid(np.abs(w), tau))
            out["peak_density_per_s"] = float(np.abs(w).max())
            out["t_offs_s"] = self.t_offs
        else:
            out["l1_norm"] = float(np.abs(self.weights).sum())
        if self.kind == "pulses":
            out["pulses"] = [
                [float(s), float(w)] for s, w in zip(self.shifts, self.weights)
            ]
        return out


@dataclasses.dataclass
class AxisMotion:
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    jump_times: np.ndarray
    jumps: np.ndarray


def _correlate_valid(signal, weights):
    n = len(signal) + len(weights) - 1
    size = 1 << (n - 1).bit_length()
    spec = np.fft.rfft(signal, size) * np.fft.rfft(weights[::-1], size)
    full = np.fft.irfft(spec, size)[:n]
    return full[len(weights) - 1 : len(signal)]


def _merge_jumps(times, values):
    order = np.argsort(times, kind="stable")
    times, values = times[order], values[order]
    if len(times):
        group = np.concatenate(
            [[0], np.cumsum(np.diff(times) > CONTIGUOUS_EPS)]
        )
        values = np.bincount(group, weights=values)
        times = times[np.concatenate([[0], np.flatnonzero(np.diff(group)) + 1])]
    keep = np.abs(values) > MIN_VELOCITY_JUMP
    return times[keep], values[keep]


def executed_axis(seg, kernel, axis, t0, dt, n, cmd_jump_times, cmd_jumps):
    dv = cmd_jumps[:, axis]
    if kernel.kind != "smoother":
        t = t0 + dt * np.arange(n)
        pos = np.zeros(n)
        vel = np.zeros(n)
        acc = np.zeros(n)
        for w, s in zip(kernel.weights, kernel.shifts):
            p, v, a = seg.eval(t + s)
            pos += w * p[:, axis]
            vel += w * v[:, axis]
            acc += w * a[:, axis]
        jt = (cmd_jump_times[:, None] - kernel.shifts[None, :]).ravel()
        jv = (dv[:, None] * kernel.weights[None, :]).ravel()
        jt, jv = _merge_jumps(jt, jv)
        return AxisMotion(pos, vel, acc, jt, jv)
    h = dt / SMOOTHER_OVERSAMPLE
    fine_n = (n - 1) * SMOOTHER_OVERSAMPLE + 1
    m_lo, w = kernel.grid_weights(h)
    ext = t0 + h * np.arange(m_lo, fine_n + m_lo + len(w) - 1)
    p, v, a = seg.eval(ext)
    pos, vel, acc = (
        _correlate_valid(f[:, axis], w)[::SMOOTHER_OVERSAMPLE]
        for f in (p, v, a)
    )
    reach = kernel.half_width + abs(kernel.t_offs)
    t = t0 + dt * np.arange(n)
    for tj, dvj in zip(cmd_jump_times, dv):
        if not dvj:
            continue
        i0 = max(0, int(np.floor((tj - reach - t0) / dt)))
        i1 = min(n, int(np.ceil((tj + reach - t0) / dt)) + 1)
        acc[i0:i1] += dvj * kernel.kernel_at_shift(tj - t[i0:i1])
    return AxisMotion(pos, vel, acc, np.zeros(0), np.zeros(0))


def stepper_positions_mm(stepper, trace):
    return (
        stepper["commanded_pos_at_mcu_zero"]
        + trace.positions * stepper["step_dist"]
    )


def check_steps_against_model(
    name, stepper, trace, t, model_pos, model_vel, model_acc, max_jump
):
    if not len(trace.clocks):
        return {"steps": 0, "max_error_mm": 0.0, "max_allowed_mm": 0.0}
    freq = stepper["clock_freq"]
    times = trace.clocks / freq
    if times[0] < t[0] or times[-1] > t[-1]:
        raise ModelMismatch(
            "%s steps [%r, %r] outside model grid [%r, %r]"
            % (name, times[0], times[-1], t[0], t[-1])
        )
    dt = t[1] - t[0]
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
    grid_error = (
        dt * dt * float(np.abs(model_acc).max())
        + 0.25 * dt * max_jump
        + FLOAT_TOLERANCE_MM
    )
    allowed = speed * time_slack + grid_error
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


def _point_segment_distance(points, a, b):
    ab = b - a
    L2 = (ab * ab).sum(axis=-1)
    u = np.clip(
        ((points - a) * ab).sum(axis=-1) / np.where(L2 > 0.0, L2, 1.0), 0.0, 1.0
    )
    return np.linalg.norm(points - (a + u[..., None] * ab), axis=-1)


def distance_to_commanded(seg, t, xy, window):
    best = np.full(len(t), np.inf)
    p0 = seg.start[:, :2] + seg.axes_r[:, :2] * seg.d0[:, None]
    p1 = seg.start[:, :2] + seg.axes_r[:, :2] * seg.d1[:, None]
    prev_end = np.concatenate([[-np.inf], seg.end[:-1]])
    next_start = np.concatenate([seg.t0[1:], [np.inf]])
    lo = np.searchsorted(t, np.minimum(seg.t0 - window, prev_end), side="left")
    hi = np.searchsorted(t, np.maximum(seg.end + window, next_start), "right")
    for k in range(len(seg.t0)):
        if lo[k] >= hi[k]:
            continue
        d = _point_segment_distance(xy[lo[k] : hi[k]], p0[k], p1[k])
        np.minimum(best[lo[k] : hi[k]], d, out=best[lo[k] : hi[k]])
    if not np.isfinite(best).all():
        raise ModelMismatch("samples outside every commanded segment window")
    return best


def corner_deviations(vertices, t, xy, to_commanded, window):
    lo = np.searchsorted(t, vertices["t_arrive"] - window, side="left")
    hi = np.searchsorted(t, vertices["t_leave"] + window, side="right")
    outward = np.zeros(len(lo))
    inward = np.zeros(len(lo))
    for k in range(len(lo)):
        outward[k] = to_commanded[lo[k] : hi[k]].max()
        path = xy[lo[k] : hi[k]]
        inward[k] = _point_segment_distance(
            vertices["xy"][k], path[:-1], path[1:]
        ).min()
    return np.maximum(outward, inward), outward, inward


def deviation_summary(values):
    if not len(values):
        return {"count": 0, "max_mm": 0.0, "p99_mm": 0.0, "p50_mm": 0.0}
    return {
        "count": int(len(values)),
        "max_mm": float(values.max()),
        "p99_mm": float(np.percentile(values, 99)),
        "p50_mm": float(np.percentile(values, 50)),
    }
