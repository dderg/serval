# Junction speed from the input-shaped corner deviation
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""[corner_deviation]: junction speed from the shaped corner deviation.

Replaces the square_corner_velocity junction term of the toolhead
lookahead with the highest junction speed whose corner, after input
shaping, stays within max_deviation (mm). The shaped path error is
e(t) = (K*P)(t) - P(t) for the commanded path P and shaper kernel K.

An isolated junction (both adjacent moves take longer than the kernel's
time support) is bounded exactly by a table of the canonical corner:
decelerate at the move accel into the vertex, change direction, accelerate
out; the table spans corner angle and orientation and is built from the
active X/Y shapers. Where moves are shorter than the kernel, the bound is
|e| <= sigma^2 * |accel| / 2 + |sum of velocity steps * R(t)|, with
R = K*ramp - ramp, summed over the junctions inside the kernel support;
each new step is also added to the error booked at the earlier junctions
it overlaps, and every junction's speed is capped so that a curve
continuing at its curvature stays within budget. Neighbour timing uses the
fastest feasible traversal of the moves between them, which overlaps the
kernels the most.

A corner whose stop alone (v=0) exceeds the budget gets the accel of both
adjacent moves lowered until the stop fits. With an input smoother the
executed acceleration of a velocity step is |dv| * kernel density: the
steps inside one kernel support may use at most half of max_accel, and the
accel of the adjacent moves is lowered so the total stays within max_accel.
With input shaping inactive on X or Y every direction change stops (as
square_corner_velocity: 0).
"""

import bisect
import logging
import math
from collections import deque

from . import input_shaper

TABLE_ALPHA_DEG = 15.0
TABLE_THETA_DEG = 7.5
TABLE_W_POINTS = 48
TABLE_W_MIN = 1e-5
TABLE_W_MAX = 2.0
MODEL_DT = 2e-4
SMOOTHER_NODES = 256
DENSITY_SAMPLES = 1001
RAMP_ERROR_POINTS = 256
MIN_DIRECTION_CHANGE = 1e-9
JUMP_ACCEL_SHARE = 0.5
ACCEL_SEARCH_STEPS = 24


class Kernel:
    def __init__(self, shifts, weights, density_peak=0.0):
        self.shifts = shifts
        self.weights = weights
        self.density_peak = density_peak
        self.variance = sum(w * s * s for w, s in zip(weights, shifts))
        self.s_lo = min(shifts)
        self.s_hi = max(shifts)
        self.r_lo = -self.s_hi
        self.r_step = (self.s_hi - self.s_lo) / (RAMP_ERROR_POINTS - 1)
        self.r_values = [
            self._exact_ramp_error(self.r_lo + i * self.r_step)
            for i in range(RAMP_ERROR_POINTS)
        ]

    def _exact_ramp_error(self, s):
        shaped = sum(
            w * max(s + shift, 0.0)
            for w, shift in zip(self.weights, self.shifts)
        )
        return shaped - max(s, 0.0)

    def ramp_error(self, s):
        """Shaped minus commanded position at time s after a unit step in
        velocity (zero outside the kernel support)."""
        idx = (s - self.r_lo) / self.r_step
        if idx <= 0.0 or idx >= RAMP_ERROR_POINTS - 1:
            return 0.0
        i = int(idx)
        r = self.r_values
        return r[i] + (idx - i) * (r[i + 1] - r[i])


def kernel_from_shaper(shaper):
    if isinstance(shaper, input_shaper.AxisInputShaper):
        if not shaper.n:
            return None
        total = sum(shaper.A)
        t_offs = sum(a * t for a, t in zip(shaper.A, shaper.T)) / total
        return Kernel(
            [t_offs - t for t in shaper.T], [a / total for a in shaper.A]
        )
    if isinstance(shaper, input_shaper.AxisInputSmoother):
        if not shaper.smooth_time:
            return None
        return _smoother_kernel(shaper.coeffs, shaper.smooth_time)
    raise TypeError("unsupported shaper %r" % (type(shaper),))


def _smoother_kernel(coeffs, t_sm):
    from numpy.polynomial import Polynomial

    hst = 0.5 * t_sm
    density = Polynomial([c / t_sm ** (i + 1) for i, c in enumerate(coeffs)])
    anti = density.integ()
    density = density / (anti(hst) - anti(-hst))
    moment = (density * Polynomial([0.0, 1.0])).integ()
    t_offs = moment(hst) - moment(-hst)
    step = t_sm / SMOOTHER_NODES
    taus = [-hst + (i + 0.5) * step for i in range(SMOOTHER_NODES)]
    weights = [float(density(tau)) * step for tau in taus]
    total = sum(weights)
    peak = max(
        abs(float(density(-hst + i * t_sm / (DENSITY_SAMPLES - 1))))
        for i in range(DENSITY_SAMPLES)
    )
    return Kernel(
        [t_offs - tau for tau in taus], [w / total for w in weights], peak
    )


class CornerTable:
    """G(w = v / accel): deviation of an isolated corner per unit accel, over
    incoming direction and signed turn angle. With different X and Y
    kernels even a straight line through a stop deviates (the axes are
    smoothed differently), so G does not vanish at zero turn."""

    def __init__(self, kx, ky):
        import numpy as np

        alphas = np.radians(np.arange(0.0, 90.0 + 1e-9, TABLE_ALPHA_DEG))
        thetas = np.radians(np.arange(-180.0, 180.0 + 1e-9, TABLE_THETA_DEG))
        w = np.concatenate(
            [[0.0], np.geomspace(TABLE_W_MIN, TABLE_W_MAX, TABLE_W_POINTS - 1)]
        )
        reach = max(abs(kx.s_lo), abs(kx.s_hi), abs(ky.s_lo), abs(ky.s_hi))
        t = np.arange(-reach - MODEL_DT, reach + 2 * MODEL_DT, MODEL_DT)
        moments = [_corner_moments(np, t, k) for k in (kx, ky)]
        N, Q, N2, Q2 = (np.stack([m[i] for m in moments], -1) for i in range(4))
        thetas = np.where(np.abs(thetas) > 1e-6, thetas, 1e-3)
        self.g = []
        isotropic = kx.shifts == ky.shifts and kx.weights == ky.weights
        for alpha in alphas:
            if isotropic and self.g:
                self.g.append(self.g[0])
                continue
            u1 = np.array([math.cos(alpha), math.sin(alpha)])
            u2 = np.stack([np.cos(alpha + thetas), np.sin(alpha + thetas)], -1)
            dev = _corner_deviation(np, w, u1, u2, N, Q, N2, Q2)
            self.g.append(np.maximum.accumulate(dev, axis=-1).tolist())
        self.w = w.tolist()
        self.alpha_step = math.radians(TABLE_ALPHA_DEG)
        self.theta_step = math.radians(TABLE_THETA_DEG)
        self.alpha_max = len(alphas) - 1.000001
        self.theta_max = len(thetas) - 1.000001

    def cell(self, ux, uy, turn):
        alpha = math.atan2(uy, ux)
        if alpha < 0.0:
            alpha, turn = -alpha, -turn
        if alpha > 0.5 * math.pi:
            alpha, turn = math.pi - alpha, -turn
        ia = int(min(alpha / self.alpha_step, self.alpha_max))
        it = int(min((turn + math.pi) / self.theta_step, self.theta_max))
        g = self.g
        return (g[ia][it], g[ia][it + 1], g[ia + 1][it], g[ia + 1][it + 1])

    @staticmethod
    def value(cell, i):
        g00, g01, g10, g11 = cell
        return max(g00[i], g01[i], g10[i], g11[i])

    def max_w(self, cell, level):
        value = self.value
        lo, hi = 0, len(self.w) - 1
        if value(cell, hi) <= level:
            return self.w[hi]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if value(cell, mid) <= level:
                lo = mid
            else:
                hi = mid
        glo, ghi = value(cell, lo), value(cell, hi)
        frac = (level - glo) / (ghi - glo) if ghi > glo else 0.0
        return self.w[lo] + frac * (self.w[hi] - self.w[lo])

    def at(self, cell, w):
        i = min(bisect.bisect_right(self.w, w), len(self.w) - 1)
        lo = i - 1
        frac = (w - self.w[lo]) / (self.w[i] - self.w[lo])
        glo = self.value(cell, lo)
        return glo + min(frac, 1.0) * (self.value(cell, i) - glo)


def _corner_moments(np, t, kernel):
    w = np.array(kernel.weights)
    tau = t[:, None] + np.array(kernel.shifts)[None, :]
    before = np.minimum(tau, 0.0)
    after = np.maximum(tau, 0.0)
    return before @ w, after @ w, (before * before) @ w, (after * after) @ w


def _corner_deviation(np, w, u1, u2, N, Q, N2, Q2):
    """Deviation of the canonical corner per turn (rows of u2) and w."""
    u2 = u2[:, None, None, :]
    B = u1 * N + u2 * Q
    A = 0.5 * (u2 * Q2 - u1 * N2)
    S = A + w[None, :, None, None] * B
    legs = []
    for direction in (-np.broadcast_to(u1, u2.shape), u2):
        along = np.maximum((S * direction).sum(axis=-1), 0.0)
        legs.append(np.linalg.norm(S - along[..., None] * direction, axis=-1))
    outward = np.minimum(*legs).max(axis=-1)
    a, b = S[..., :-1, :], S[..., 1:, :]
    ab = b - a
    L2 = (ab * ab).sum(axis=-1)
    u = np.clip(-(a * ab).sum(axis=-1) / np.where(L2 > 0.0, L2, 1.0), 0.0, 1.0)
    inward = np.linalg.norm(a + u[..., None] * ab, axis=-1).min(axis=-1)
    return np.maximum(outward, inward)


def min_traversal_time(move, start_v2, end_v2):
    accel, length = move.accel, move.move_d
    reach = 2.0 * accel * length
    start_v2 = min(start_v2, end_v2 + reach)
    end_v2 = min(end_v2, start_v2 + reach)
    peak_v2 = min(move.max_cruise_v2, 0.5 * (start_v2 + end_v2 + reach))
    start_v2 = min(start_v2, peak_v2)
    end_v2 = min(end_v2, peak_v2)
    accel_d = (peak_v2 - start_v2) / (2.0 * accel)
    decel_d = (peak_v2 - end_v2) / (2.0 * accel)
    vs, ve, vp = math.sqrt(start_v2), math.sqrt(end_v2), math.sqrt(peak_v2)
    cruise_d = max(length - accel_d - decel_d, 0.0)
    return 2.0 * accel_d / (vs + vp) + cruise_d / vp + 2.0 * decel_d / (vp + ve)


class Junction:
    __slots__ = ("time", "dvx", "dvy", "deviation", "ax", "ay")

    def __init__(self, time, dvx, dvy, deviation, ax, ay):
        self.time = time
        self.dvx = dvx
        self.dvy = dvy
        self.deviation = deviation
        self.ax = ax
        self.ay = ay


class CornerDeviation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.max_deviation = config.getfloat("max_deviation", above=0.0)
        printer_section = config.getsection("printer")
        if printer_section.get("square_corner_velocity", None) is not None:
            raise config.error(
                "[corner_deviation] replaces square_corner_velocity;"
                " remove square_corner_velocity from [printer]"
            )
        self.toolhead = None
        self.input_shaper = None
        self.kernels = None
        self.table = None
        self.variance = 0.0
        self.reported_inactive = False
        self.history = deque()
        self.window = 0.0
        self.last_move = None
        self.last_time = 0.0
        self.stats = {
            "limited": 0,
            "accel_lowered": 0,
            "over_budget": 0,
            "over_accel": 0,
        }
        self.printer.register_event_handler("klippy:connect", self._connect)

    def _connect(self):
        self.input_shaper = self.printer.lookup_object("input_shaper", None)
        if self.input_shaper is None:
            raise self.printer.config_error(
                "[corner_deviation] requires [input_shaper]"
            )
        real_update = self.input_shaper._update_input_shaping

        def update_input_shaping(*args, **kwargs):
            real_update(*args, **kwargs)
            self._refresh_kernels()

        self.input_shaper._update_input_shaping = update_input_shaping
        self._refresh_kernels()
        self.toolhead = self.printer.lookup_object("toolhead")
        self.toolhead.set_junction_limiter(self.junction_v2)
        gcode = self.printer.lookup_object("gcode")
        real_set_velocity_limit = gcode.register_command(
            "SET_VELOCITY_LIMIT", None
        )

        def cmd_SET_VELOCITY_LIMIT(gcmd):
            if gcmd.get("SQUARE_CORNER_VELOCITY", None) is not None:
                raise gcmd.error(
                    "SQUARE_CORNER_VELOCITY has no effect with"
                    " [corner_deviation]; set max_deviation instead"
                )
            real_set_velocity_limit(gcmd)

        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            cmd_SET_VELOCITY_LIMIT,
            desc="Set printer velocity limits",
        )

    def _refresh_kernels(self):
        self.history.clear()
        self.last_move = None
        shapers = {s.get_axis(): s for s in self.input_shaper.get_shapers()}
        kx = kernel_from_shaper(shapers["x"])
        ky = kernel_from_shaper(shapers["y"])
        if kx is None or ky is None:
            self.kernels = self.table = None
            return
        self.kernels = (kx, ky)
        self.table = CornerTable(kx, ky)
        self.window = max(kx.s_hi, ky.s_hi, -kx.s_lo, -ky.s_lo)
        self.variance = max(kx.variance, ky.variance)
        self.reported_inactive = False

    def _advance(self, prev_move, end_v2):
        self.last_time += min_traversal_time(
            prev_move, prev_move.max_start_v2, end_v2
        )
        return self.last_time

    def get_status(self, eventtime):
        return dict(self.stats, max_deviation=self.max_deviation)

    def junction_v2(self, prev_move, move, cap_v2):
        if self.kernels is None:
            if not self.reported_inactive:
                logging.warning(
                    "corner_deviation: input shaping inactive on X or Y;"
                    " every direction change stops"
                )
                self.reported_inactive = True
            return 0.0
        if prev_move is not self.last_move:
            self.history.clear()
            self.last_time = 0.0
        self.last_move = move
        upx, upy = prev_move.axes_r[0], prev_move.axes_r[1]
        umx, umy = move.axes_r[0], move.axes_r[1]
        dux, duy = abs(umx - upx), abs(umy - upy)
        du = math.hypot(dux, duy)
        if du < MIN_DIRECTION_CHANGE:
            self._advance(prev_move, cap_v2)
            return cap_v2
        kx, ky = self.kernels
        budget = self.max_deviation
        accel = max(prev_move.accel, move.accel)
        table = self.table
        turn = math.atan2(upx * umy - upy * umx, upx * umx + upy * umy)
        cell = table.cell(upx, upy, turn)
        floor = table.value(cell, 0)
        room = budget - floor * accel
        speed = 0.0
        if room > 0.0:
            speed = min(
                math.sqrt(cap_v2),
                accel * table.max_w(cell, budget / accel),
            )
            xy_length = prev_move.move_d * math.hypot(upx, upy)
            xy_length += move.move_d * math.hypot(umx, umy)
            curvature = 2.0 * abs(turn) / xy_length
            speed = min(
                speed, math.sqrt(2.0 * room / (self.variance * curvature))
            )
        before = self.last_time
        now = self._advance(prev_move, speed * speed)
        history = self.history
        while history and now - history[0].time >= self.window:
            history.popleft()
        ex = ey = 0.0
        others_x = others_y = 0.0
        for j in history:
            dt = now - j.time
            ex += j.dvx * kx.ramp_error(dt)
            ey += j.dvy * ky.ramp_error(dt)
            others_x = max(others_x, j.ax)
            others_y = max(others_y, j.ay)
            overlap = math.hypot(
                dux * kx.ramp_error(-dt), duy * ky.ramp_error(-dt)
            )
            if overlap > 0.0:
                speed = min(speed, max(budget - j.deviation, 0.0) / overlap)
        isolated = (
            now - before >= self.window
            and min_traversal_time(move, speed * speed, move.max_cruise_v2)
            >= self.window
        )
        if isolated:
            speed, accel, deviation = self._isolated(
                prev_move, move, cell, speed, accel, ex, ey
            )
        else:
            share_x = max(abs(upx), abs(umx))
            share_y = max(abs(upy), abs(umy))
            speed, accel, deviation = self._overlapping(
                prev_move,
                move,
                speed,
                accel,
                ex,
                ey,
                dux * kx.ramp_error(0.0),
                duy * ky.ramp_error(0.0),
                (share_x, share_y, others_x, others_y),
            )
        if kx.density_peak or ky.density_peak:
            speed, accel = self._accel_bound(
                prev_move, move, speed, accel, dux, duy
            )
        cap_v2 = min(cap_v2, prev_move.max_start_v2 + prev_move.delta_v2)
        v2 = min(cap_v2, speed * speed)
        if v2 < cap_v2:
            self.stats["limited"] += 1
        speed = math.sqrt(v2)
        for j in history:
            dt = now - j.time
            j.deviation += speed * math.hypot(
                dux * kx.ramp_error(-dt), duy * ky.ramp_error(-dt)
            )
        if isolated:
            deviation = accel * table.at(cell, speed / accel)
        history.append(
            Junction(
                now,
                speed * dux,
                speed * duy,
                deviation,
                prev_move.accel * abs(upx),
                prev_move.accel * abs(upy),
            )
        )
        return v2

    def _isolated(self, prev_move, move, cell, speed, accel, ex, ey):
        table = self.table
        level = self.max_deviation - math.hypot(ex, ey)
        floor = table.value(cell, 0)
        if level <= 0.0:
            self.stats["over_budget"] += 1
            return 0.0, accel, None
        if floor * accel > level:
            accel = level / floor
            self._lower_accel(prev_move, move, accel)
            return 0.0, accel, None
        return min(speed, accel * table.max_w(cell, level / accel)), accel, None

    def _floor(self, accel, shares):
        share_x, share_y, others_x, others_y = shares
        kx, ky = self.kernels
        return 0.5 * math.hypot(
            kx.variance * max(accel * share_x, others_x),
            ky.variance * max(accel * share_y, others_y),
        )

    def _overlapping(
        self, prev_move, move, speed, accel, ex, ey, cx, cy, shares
    ):
        """Rigorous bound when another junction or accel phase is inside the
        kernel support: |e| <= sigma^2 * |accel| / 2 + |velocity steps|."""
        budget = self.max_deviation
        steps = math.hypot(ex, ey)
        room = budget - self._floor(accel, shares)
        if steps >= room:
            if self._floor(0.0, shares) + steps >= budget:
                self.stats["over_budget"] += 1
                return 0.0, accel, self._floor(accel, shares) + steps
            lo, hi = 0.0, accel
            for _ in range(ACCEL_SEARCH_STEPS):
                mid = 0.5 * (lo + hi)
                if self._floor(mid, shares) + steps <= budget:
                    lo = mid
                else:
                    hi = mid
            self._lower_accel(prev_move, move, lo)
            return 0.0, lo, self._floor(lo, shares) + steps
        cc = cx * cx + cy * cy
        if cc > 0.0:
            ec = ex * cx + ey * cy
            disc = ec * ec - cc * (ex * ex + ey * ey - room * room)
            speed = min(speed, (math.sqrt(max(disc, 0.0)) - ec) / cc)
        deviation = budget - room + math.hypot(ex + speed * cx, ey + speed * cy)
        return speed, accel, deviation

    def _accel_bound(self, prev_move, move, speed, accel, dux, duy):
        kx, ky = self.kernels
        max_accel = self.toolhead.max_accel
        nx = ny = other_x = other_y = 0.0
        for j in self.history:
            nx += j.dvx * kx.density_peak
            ny += j.dvy * ky.density_peak
            other_x = max(other_x, j.ax)
            other_y = max(other_y, j.ay)
        axes = (
            (kx.density_peak, dux, nx, other_x, 0),
            (ky.density_peak, duy, ny, other_y, 1),
        )
        for density, du, steps, other, _ in axes:
            if density and du:
                room = min(JUMP_ACCEL_SHARE * max_accel, max_accel - other)
                speed = min(speed, max(room - steps, 0.0) / (density * du))
        limit = accel
        for density, du, steps, other, axis in axes:
            if other + steps + speed * du * density > max_accel:
                self.stats["over_accel"] += 1
            share = max(abs(prev_move.axes_r[axis]), abs(move.axes_r[axis]))
            if share:
                room = max_accel - speed * du * density - steps
                limit = min(limit, room / share)
        if limit < accel:
            self._lower_accel(prev_move, move, limit)
            accel = limit
        return speed, accel

    def _lower_accel(self, prev_move, move, accel):
        if accel <= 0.0:
            raise self.printer.command_error(
                "corner_deviation: no acceleration left at a corner"
                " (accel %.3f); this is a bug" % (accel,)
            )
        self.stats["accel_lowered"] += 1
        for m in (prev_move, move):
            m.limit_speed(math.sqrt(m.max_cruise_v2), accel)


def load_config(config):
    return CornerDeviation(config)
