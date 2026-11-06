# Junction speed from the input-shaped corner deviation
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""[corner_deviation]: junction speed referenced to the shaper's stop floor.

Replaces the square_corner_velocity junction term of the toolhead
lookahead. Input shaping rounds every corner, even a full stop: the shaped
path e(t) = (K*P)(t) - P(t) of any acceleration profile stays within the
shaper floor F = |(sigma_x^2 * a_x, sigma_y^2 * a_y)| / 2, sigma^2 being
the kernel's time variance and a_x, a_y the largest per-axis accelerations
around the corner. Each junction runs at the highest speed whose isolated
corner, after shaping, rounds no more than F: corners never round more than
a stop already can. The optional max_deviation lowers junction speeds
further so that corners round at most that much; it never lowers
acceleration, so a corner whose stop alone rounds more than max_deviation
just stops.

The isolated corner is read from a table built from the active X/Y
shapers, the worst over each leg decelerating or accelerating through the
vertex, over incoming direction and turn angle. Along curves the speed is
also capped so that sigma^2 * v^2 * curvature / 2 stays within the room the
corner leaves, and moves shorter than the kernel support get that cap as
their cruise speed.

An input smoother turns a velocity step dv into acceleration dv * density
over its duration. Even a slow corner approached at full deceleration then
exceeds max_accel slightly, so a junction's own step must fit under
(1 + STEP_ALLOWANCE) * max_accel on top of what its legs command, for any
leg cruising or accelerating either way at max_accel; the steps of a curve
continuing with the same spacing get max_accel. Junctions with different
direction changes nearby are not summed, so the bound is approximate:
ACCEL_MARGIN is the stated allowance on the executed peak above max_accel.

With input shaping inactive on X or Y every direction change stops (as
square_corner_velocity: 0).
"""

import logging
import math

from . import input_shaper

TABLE_ALPHA_DEG = 15.0
TABLE_THETA_DEG = 7.5
TABLE_W_POINTS = 48
TABLE_W_MIN = 1e-5
TABLE_W_MAX = 2.0
MODEL_DT = 2e-4
SMOOTHER_NODES = 256
MIN_DIRECTION_CHANGE = 1e-9
STEP_ALLOWANCE = 0.03
ACCEL_MARGIN = 0.05
JUMP_GRID_POINTS = 41


class Kernel:
    def __init__(self, shifts, weights, density_peak=0.0):
        self.shifts = shifts
        self.weights = weights
        self.density_peak = density_peak
        self.variance = sum(w * s * s for w, s in zip(weights, shifts))
        self.s_lo = min(shifts)
        self.s_hi = max(shifts)
        order = sorted(range(len(shifts)), key=lambda i: shifts[i])
        if density_peak:
            self.jump_grid = _jump_room_grid(
                [shifts[i] for i in order], [weights[i] for i in order]
            )

    def jump_room(self, up, um):
        """Largest |dv| * density_peak / max_accel at a junction whose axis
        direction share goes from up to um, keeping the executed accel
        within (1 + STEP_ALLOWANCE) * max_accel while each leg cruises or
        accelerates either way at up to max_accel; the smallest of the
        four surrounding grid entries."""
        scale = 0.5 * (JUMP_GRID_POINTS - 1)
        i = min(int((up + 1.0) * scale), JUMP_GRID_POINTS - 2)
        j = min(int((um + 1.0) * scale), JUMP_GRID_POINTS - 2)
        g = self.jump_grid
        return min(g[i][j], g[i][j + 1], g[i + 1][j], g[i + 1][j + 1])


def _jump_room_grid(shifts, weights):
    """At time -shift_k after the junction a share passed_k of the kernel
    has seen it, so the legs command c_in * (1 - passed_k) + c_out *
    passed_k there while the velocity step adds dv * density_k."""
    import numpy as np

    w = np.array(weights)
    passed = np.cumsum(w[::-1])[::-1]
    density = w / w.max()
    live = density > 0.0
    passed, density = passed[live], density[live]
    u = np.linspace(-1.0, 1.0, JUMP_GRID_POINTS)
    up, um = np.meshgrid(u, u, indexing="ij")
    sign = np.sign(um - up)
    room = np.full(up.shape, np.inf)
    for c_in in (-up, 0.0 * up, up):
        for c_out in (um, 0.0 * um, -um):
            command = (
                c_in[..., None] * (1.0 - passed) + c_out[..., None] * passed
            )
            limit = (1.0 + STEP_ALLOWANCE - sign[..., None] * command) / density
            room = np.minimum(room, limit.min(axis=-1))
    return np.maximum(room, 0.0).tolist()


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
    extremes = [-hst, hst] + [
        r.real
        for r in density.deriv().roots()
        if abs(r.imag) < 1e-12 and -hst <= r.real <= hst
    ]
    return Kernel(
        [t_offs - tau for tau in taus],
        [w / total for w in weights],
        max(abs(float(density(x))) for x in extremes),
    )


class Cell:
    """Table entry for one (direction, turn) cell, conservative over its
    four grid corners: g[i] is the isolated corner's deviation per unit
    accel at w[i] = v / accel, floor the shaper floor per unit accel and
    w_floor the largest w whose corner stays within it."""

    __slots__ = ("g", "floor", "w_floor")

    def __init__(self, g, floor, w_floor):
        self.g = g
        self.floor = floor
        self.w_floor = w_floor


class CornerTable:
    def __init__(self, kx, ky):
        import numpy as np

        alphas = np.radians(np.arange(0.0, 90.0 + 1e-9, TABLE_ALPHA_DEG))
        thetas = np.radians(np.arange(-180.0, 180.0 + 1e-9, TABLE_THETA_DEG))
        thetas = np.where(np.abs(thetas) > 1e-6, thetas, 1e-3)
        w = np.concatenate(
            [[0.0], np.geomspace(TABLE_W_MIN, TABLE_W_MAX, TABLE_W_POINTS - 1)]
        )
        reach = max(abs(kx.s_lo), abs(kx.s_hi), abs(ky.s_lo), abs(ky.s_hi))
        t = np.arange(-reach - MODEL_DT, reach + 2 * MODEL_DT, MODEL_DT)
        legs = [
            [
                np.stack(
                    [_leg_offsets(np, t, k, w, side, s) for k in (kx, ky)], -1
                )
                for s in (1.0, -1.0)
            ]
            for side in (-1.0, 1.0)
        ]
        isotropic = kx.shifts == ky.shifts and kx.weights == ky.weights
        g, floor = [], []
        for alpha in alphas:
            u1 = np.array([math.cos(alpha), math.sin(alpha)])
            u2 = np.stack([np.cos(alpha + thetas), np.sin(alpha + thetas)], -1)
            if isotropic and g:
                g.append(g[0])
            else:
                dev = _corner_deviation(np, u1, u2, legs)
                g.append(np.maximum.accumulate(dev, axis=-1))
            share_x = np.maximum(abs(u1[0]), np.abs(u2[:, 0]))
            share_y = np.maximum(abs(u1[1]), np.abs(u2[:, 1]))
            floor.append(
                0.5 * np.hypot(kx.variance * share_x, ky.variance * share_y)
            )
        self.w = w.tolist()
        self.cells = []
        for ia in range(len(alphas) - 1):
            row = []
            for it in range(len(thetas) - 1):
                corners = [(ia + i, it + j) for i in (0, 1) for j in (0, 1)]
                cg = np.max([g[a][b] for a, b in corners], axis=0).tolist()
                cf = min(float(floor[a][b]) for a, b in corners)
                row.append(Cell(cg, cf, self.max_w(cg, cf)))
            self.cells.append(row)
        self.alpha_step = math.radians(TABLE_ALPHA_DEG)
        self.theta_step = math.radians(TABLE_THETA_DEG)
        self.alpha_max = len(alphas) - 2
        self.theta_max = len(thetas) - 2

    def cell(self, ux, uy, turn):
        alpha = math.atan2(uy, ux)
        if alpha < 0.0:
            alpha, turn = -alpha, -turn
        if alpha > 0.5 * math.pi:
            alpha, turn = math.pi - alpha, -turn
        ia = min(int(alpha / self.alpha_step), self.alpha_max)
        it = min(int((turn + math.pi) / self.theta_step), self.theta_max)
        return self.cells[ia][it]

    def max_w(self, g, level):
        """Largest w with g(w) <= level (g is non-decreasing)."""
        if level < g[0]:
            return 0.0
        lo, hi = 0, len(self.w) - 1
        if g[hi] <= level:
            return self.w[hi]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if g[mid] <= level:
                lo = mid
            else:
                hi = mid
        frac = (level - g[lo]) / (g[hi] - g[lo]) if g[hi] > g[lo] else 0.0
        return self.w[lo] + frac * (self.w[hi] - self.w[lo])


def _leg_offsets(np, t, kernel, w, side, sign):
    """Kernel-weighted position per unit accel of the leg before (side -1)
    or after (side +1) a vertex passed at speed w*accel, the speed growing
    away from the vertex (sign +1: decelerating in, accelerating out) or
    shrinking (sign -1: a leg accelerating in starts from rest at -w, one
    decelerating out stops at +w)."""
    tau = side * np.maximum(
        side * (t[:, None] + np.array(kernel.shifts)[None, :]), 0.0
    )
    tau = tau[None, :, :]
    wc = w[:, None, None]
    if sign < 0.0:
        tau = np.clip(tau, -wc, wc)
    return (wc * tau + 0.5 * side * sign * tau * tau) @ np.array(kernel.weights)


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


class CornerDeviation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.max_deviation = config.getfloat("max_deviation", None, above=0.0)
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
        self.window = 0.0
        self.smoothing = False
        self.reported_inactive = False
        self.stats = {"limited": 0, "stopped_by_cap": 0, "step_limited": 0}
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
        self.smoothing = bool(kx.density_peak or ky.density_peak)
        self.reported_inactive = False

    def get_status(self, eventtime):
        return dict(
            self.stats,
            max_deviation=self.max_deviation,
            accel_margin=ACCEL_MARGIN,
        )

    def junction_v2(self, prev_move, move, cap_v2):
        if self.kernels is None:
            if not self.reported_inactive:
                logging.warning(
                    "corner_deviation: input shaping inactive on X or Y;"
                    " every direction change stops"
                )
                self.reported_inactive = True
            return 0.0
        upx, upy = prev_move.axes_r[0], prev_move.axes_r[1]
        umx, umy = move.axes_r[0], move.axes_r[1]
        dux, duy = abs(umx - upx), abs(umy - upy)
        if math.hypot(dux, duy) < MIN_DIRECTION_CHANGE:
            return cap_v2
        accel = max(prev_move.accel, move.accel)
        turn = math.atan2(upx * umy - upy * umx, upx * umx + upy * umy)
        cell = self.table.cell(upx, upy, turn)
        w, room = cell.w_floor, cell.floor
        if self.max_deviation is not None:
            level = self.max_deviation / accel
            if level < cell.g[0]:
                self.stats["stopped_by_cap"] += 1
                w = room = 0.0
            else:
                w = min(w, self.table.max_w(cell.g, level))
                room = min(room, level)
        speed = 0.0
        if room > 0.0:
            speed = accel * w
            xy_length = prev_move.move_d * math.hypot(upx, upy)
            xy_length += move.move_d * math.hypot(umx, umy)
            curvature = 2.0 * abs(turn) / xy_length
            curve_speed = math.sqrt(
                2.0 * accel * room / (self.variance * curvature)
            )
            if curve_speed < speed:
                speed = curve_speed
                reach = 2.0 * curve_speed * self.window
                for m in (prev_move, move):
                    if m.move_d < reach and speed * speed < m.max_cruise_v2:
                        m.limit_speed(speed, m.accel)
        if self.smoothing:
            speed = self._step_bound(prev_move, move, speed, dux, duy)
        v2 = min(cap_v2, speed * speed)
        if v2 < cap_v2:
            self.stats["limited"] += 1
        return v2

    def _step_bound(self, prev_move, move, speed, dux, duy):
        """The junction's own step adds |dv| * density_peak on top of what
        its legs command. A curve of junctions spaced like these, at most
        the shorter adjacent move apart, sums its steps to about
        |dv| * (density_peak + v / spacing); its legs cruise at the curve's
        speed, so that sum gets max_accel."""
        max_accel = self.toolhead.max_accel
        curve_room = max_accel
        rate = 1.0 / min(prev_move.move_d, move.move_d)
        bound = speed
        for axis, du in ((0, dux), (1, duy)):
            kernel = self.kernels[axis]
            if not kernel.density_peak or not du:
                continue
            linear = du * kernel.density_peak
            own = kernel.jump_room(prev_move.axes_r[axis], move.axes_r[axis])
            quadratic = du * rate
            curve = (
                2.0
                * curve_room
                / (
                    linear
                    + math.sqrt(linear * linear + 4.0 * quadratic * curve_room)
                )
            )
            bound = min(bound, own * max_accel / linear, curve)
        if bound < speed:
            self.stats["step_limited"] += 1
        return bound


def load_config(config):
    return CornerDeviation(config)
