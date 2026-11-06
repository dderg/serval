# Junction speed from the input-shaped corner deviation
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""[corner_deviation]: junction speed from the shaped corner deviation.

Replaces the square_corner_velocity junction term of the toolhead
lookahead with the highest junction speed whose corner, after input
shaping, stays within max_deviation (mm). The shaped path error is
e(t) = (K*P)(t) - P(t) for the commanded path P and shaper kernel K.

Each junction's own corner is bounded by a table of the isolated corner,
the worst over each leg decelerating or accelerating at the move accel
(an accelerating-in leg starts from rest, a decelerating-out leg comes to
rest); the table spans corner angle and orientation and is built from the
active X/Y shapers. Neighbouring junctions inside the kernel's time support
add their velocity steps, |dv| * R(t) with R = K*ramp - ramp, and the
stop-corner floors they leave (weighted by how long their accel phases
last against the kernel support); all acceleration-induced error together
is capped by sigma^2 * |accel| / 2. Each new step and floor is also added
to the error booked at the earlier junctions it overlaps. Along a curve,
junction speed and the cruise speed of moves shorter than the kernel
support are capped so that a curve continuing at its curvature stays
within budget. Neighbour timing uses the fastest feasible traversal of the
moves between them, which overlaps the kernels the most.

A corner whose stop alone (v=0) exceeds the budget gets the accel of both
adjacent moves lowered until the stop fits, keeping half of the remaining
room when the next move is shorter than the kernel support (a second stop
may follow). With different X and Y shapers an accelerating diagonal move
leaves the path sideways by |sigma_x^2 - sigma_y^2| * |accel * ux * uy| / 2;
its accel is lowered to keep that within budget. With an input smoother the
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
NEXT_STOP_ROOM = 0.5


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
        order = sorted(range(len(shifts)), key=lambda i: shifts[i])
        self.d_step = (self.s_hi - self.s_lo) / max(len(shifts) - 1, 1)
        peak_weight = max(weights)
        self.d_values = [density_peak * weights[i] / peak_weight for i in order]

    def density_at_shift(self, s):
        """Kernel density (1/s) at shift s; zero for pulse shapers."""
        idx = (s - self.s_lo) / self.d_step
        if not self.density_peak or idx <= 0.0 or idx >= len(self.d_values) - 1:
            return 0.0
        i = int(idx)
        d = self.d_values
        return d[i] + (idx - i) * (d[i + 1] - d[i])

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
        moments = [_stop_moments(np, t, k) for k in (kx, ky)]
        N2, Q2 = (np.stack([m[i] for m in moments], -1) for i in (0, 1))
        legs = [
            [
                np.stack(
                    [_leg_offsets(np, t, k, w, sign) for k in (kx, ky)], -1
                )
                for sign in (1.0, -1.0)
            ]
            for _leg_offsets in (_in_leg_offsets, _out_leg_offsets)
        ]
        thetas = np.where(np.abs(thetas) > 1e-6, thetas, 1e-3)
        self.g = []
        profile = np.zeros(len(t))
        isotropic = kx.shifts == ky.shifts and kx.weights == ky.weights
        for alpha in alphas:
            if isotropic and self.g:
                self.g.append(self.g[0])
                continue
            u1 = np.array([math.cos(alpha), math.sin(alpha)])
            u2 = np.stack([np.cos(alpha + thetas), np.sin(alpha + thetas)], -1)
            dev = _corner_deviation(np, u1, u2, legs)
            self.g.append(np.maximum.accumulate(dev, axis=-1).tolist())
            stop = _stop_profile(np, u1, u2, N2, Q2)
            peak = np.maximum(dev[:, :1], 1e-300)
            profile = np.maximum(profile, (stop / peak).max(axis=0))
        self.w = w.tolist()
        self.alpha_step = math.radians(TABLE_ALPHA_DEG)
        self.theta_step = math.radians(TABLE_THETA_DEG)
        self.alpha_max = len(alphas) - 1.000001
        self.theta_max = len(thetas) - 1.000001
        half = int(round(0.5 * reach / MODEL_DT))
        self.profile = [
            float(profile[max(0, i - half) : i + half + 1].max())
            for i in range(len(profile))
        ]
        self.profile_t0 = float(t[0])

    def stop_share(self, dt):
        """Upper envelope of a stop corner's deviation at time dt from the
        corner, as a share of its peak deviation."""
        idx = (dt - self.profile_t0) / MODEL_DT
        if idx <= 0.0 or idx >= len(self.profile) - 1:
            return 0.0
        i = int(idx)
        return max(self.profile[i], self.profile[i + 1])

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


def _stop_moments(np, t, kernel):
    w = np.array(kernel.weights)
    tau = t[:, None] + np.array(kernel.shifts)[None, :]
    before = np.minimum(tau, 0.0)
    after = np.maximum(tau, 0.0)
    return (before * before) @ w, (after * after) @ w


def _stop_profile(np, u1, u2, N2, Q2):
    """Distance of the shaped stop corner to its legs over time, per turn."""
    u2 = u2[:, None, :]
    S = 0.5 * (u2 * Q2 - u1 * N2)
    legs = []
    for direction in (-np.broadcast_to(u1, u2.shape), u2):
        along = np.maximum((S * direction).sum(axis=-1), 0.0)
        legs.append(np.linalg.norm(S - along[..., None] * direction, axis=-1))
    return np.minimum(*legs)


def _in_leg_offsets(np, t, kernel, w, decelerating):
    """Kernel-weighted position before the vertex per unit accel, for a leg
    reaching the vertex at speed w*accel while decelerating (+1) or
    accelerating (-1); an accelerating leg was at rest until -w."""
    tau = np.minimum(t[:, None] + np.array(kernel.shifts)[None, :], 0.0)
    tau = tau[None, :, :]
    wc = w[:, None, None]
    if decelerating < 0.0:
        tau = np.maximum(tau, -wc)
    return (wc * tau - 0.5 * decelerating * tau * tau) @ np.array(
        kernel.weights
    )


def _out_leg_offsets(np, t, kernel, w, accelerating):
    """Kernel-weighted position after the vertex per unit accel, for a leg
    leaving at speed w*accel while accelerating (+1) or decelerating (-1);
    a decelerating leg comes to rest at w."""
    tau = np.maximum(t[:, None] + np.array(kernel.shifts)[None, :], 0.0)
    tau = tau[None, :, :]
    wc = w[:, None, None]
    if accelerating < 0.0:
        tau = np.minimum(tau, wc)
    return (wc * tau + 0.5 * accelerating * tau * tau) @ np.array(
        kernel.weights
    )


def _corner_deviation(np, u1, u2, legs):
    """Deviation of an isolated corner per turn (rows of u2) and w, the
    worst over the accel sign of each leg."""
    u2 = u2[:, None, None, :]
    worst = None
    for leg_in in legs[0]:
        for leg_out in legs[1]:
            dev = _leg_deviation(np, u1 * leg_in + u2 * leg_out, u1, u2)
            worst = dev if worst is None else np.maximum(worst, dev)
    return worst


def _leg_deviation(np, S, u1, u2):
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
    __slots__ = ("time", "dvx", "dvy", "deviation", "floor", "ax", "ay")

    def __init__(self, time, dvx, dvy, deviation, floor, ax, ay):
        self.time = time
        self.dvx = dvx
        self.dvy = dvy
        self.deviation = deviation
        self.floor = floor
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
        kx, ky = self.kernels
        budget = self.max_deviation
        sideways = 0.5 * abs(kx.variance - ky.variance) * abs(umx * umy)
        if sideways * move.accel > budget:
            self.stats["accel_lowered"] += 1
            move.limit_speed(math.sqrt(move.max_cruise_v2), budget / sideways)
        dux, duy = abs(umx - upx), abs(umy - upy)
        du = math.hypot(dux, duy)
        if du < MIN_DIRECTION_CHANGE:
            self._advance(prev_move, cap_v2)
            return cap_v2
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
            curve_speed = math.sqrt(2.0 * room / (self.variance * curvature))
            curve_reach = 2.0 * curve_speed * self.window
            for m in (prev_move, move):
                if m.move_d < curve_reach and curve_speed**2 < m.max_cruise_v2:
                    m.limit_speed(curve_speed, m.accel)
            speed = min(speed, curve_speed)
        now = self._advance(prev_move, speed * speed)
        history = self.history
        while history and now - history[0].time >= self.window:
            history.popleft()
        steps_x = steps_y = neighbour_floors = others_x = others_y = 0.0
        expected_floor = (
            accel * floor * self._persistence(prev_move, move, speed, accel)
        )
        for j in history:
            dt = now - j.time
            steps_x += j.dvx * kx.ramp_error(dt)
            steps_y += j.dvy * ky.ramp_error(dt)
            neighbour_floors += j.floor * table.stop_share(dt)
            others_x = max(others_x, j.ax)
            others_y = max(others_y, j.ay)
            share = table.stop_share(-dt)
            room_j = budget - j.deviation - expected_floor * share
            overlap = math.hypot(
                dux * kx.ramp_error(-dt), duy * ky.ramp_error(-dt)
            )
            if overlap > 0.0:
                speed = min(speed, max(room_j, 0.0) / overlap)
        shares = (
            max(abs(upx), abs(umx)),
            max(abs(upy), abs(umy)),
            others_x,
            others_y,
        )

        def accel_error(a):
            return min(a * floor + neighbour_floors, self._floor(a, shares))

        level = budget - math.hypot(steps_x, steps_y)
        if accel_error(0.0) >= level:
            self.stats["over_budget"] += 1
            speed = 0.0
        elif accel_error(accel) > level:
            target = level
            if min_traversal_time(move, 0.0, move.max_cruise_v2) < self.window:
                unavoidable = accel_error(0.0)
                target = unavoidable + NEXT_STOP_ROOM * (level - unavoidable)
            lo, hi = 0.0, accel
            for _ in range(ACCEL_SEARCH_STEPS):
                mid = 0.5 * (lo + hi)
                if accel_error(mid) <= target:
                    lo = mid
                else:
                    hi = mid
            speed = 0.0
            if lo > 0.0:
                accel = lo
                self._lower_accel(prev_move, move, accel)
            else:
                self.stats["over_budget"] += 1
        else:
            level -= accel_error(accel) - accel * floor
            speed = min(speed, accel * table.max_w(cell, level / accel))
        if kx.density_peak or ky.density_peak:
            speed, accel = self._accel_bound(
                prev_move, move, speed, accel, dux, duy
            )
        cap_v2 = min(cap_v2, prev_move.max_start_v2 + prev_move.delta_v2)
        v2 = min(cap_v2, speed * speed)
        if v2 < cap_v2:
            self.stats["limited"] += 1
        speed = math.sqrt(v2)
        own_floor = (
            accel * floor * self._persistence(prev_move, move, speed, accel)
        )
        for j in history:
            dt = now - j.time
            j.deviation += own_floor * table.stop_share(
                -dt
            ) + speed * math.hypot(
                dux * kx.ramp_error(-dt), duy * ky.ramp_error(-dt)
            )
        history.append(
            Junction(
                now,
                speed * dux,
                speed * duy,
                math.hypot(steps_x, steps_y)
                + accel_error(accel)
                + accel * (table.at(cell, speed / accel) - floor),
                own_floor,
                prev_move.accel * abs(upx),
                prev_move.accel * abs(upy),
            )
        )
        return v2

    def _floor(self, accel, shares):
        """sigma^2 * |accel| / 2: bound on the shaped error caused by any
        acceleration profile inside the kernel support."""
        share_x, share_y, others_x, others_y = shares
        kx, ky = self.kernels
        return 0.5 * math.hypot(
            kx.variance * max(accel * share_x, others_x),
            ky.variance * max(accel * share_y, others_y),
        )

    def _persistence(self, prev_move, move, speed, accel):
        """Share of a full stop-corner floor a junction produces: the accel
        phases around it last (sqrt(v^2 + accel*L) - v) / accel at most, L
        the longer adjacent move, and only phases spanning the kernel
        support leave the full floor."""
        length = max(prev_move.move_d, move.move_d)
        phase = (math.sqrt(speed * speed + accel * length) - speed) / accel
        return min(1.0, phase / self.window)

    def _accel_bound(self, prev_move, move, speed, accel, dux, duy):
        kx, ky = self.kernels
        max_accel = self.toolhead.max_accel
        nx = ny = other_x = other_y = 0.0
        now = self.last_time
        for j in self.history:
            nx += j.dvx * kx.density_at_shift(j.time - now)
            ny += j.dvy * ky.density_at_shift(j.time - now)
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
