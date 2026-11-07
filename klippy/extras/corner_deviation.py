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
a stop already can. The table models isolated corners, legs at constant
acceleration through the vertex. A move shorter than the distance covered
within the kernel's reach can accelerate and decelerate between two
corners inside one window, which rounds beyond any isolated corner: a
junction next to such a move whose turn exceeds SHORT_MOVE_STOP_DEG stops
(curves turn less per junction and keep the curve cap). A stop has no
velocity step, so it rounds within the floor. Clusters of smaller turns
still round slightly beyond it; FLOOR_MARGIN is the stated allowance,
checked by tools/motion_eval/fuzz.py on clustered corners. There is no
tighter deviation cap: lowering junction speeds cannot bound clusters
below the floor, and acceleration is never lowered.

The isolated corner is read from a table built from the active X/Y
shapers, the worst over each leg decelerating or accelerating through the
vertex, over incoming direction and turn angle. Along curves the junction
speed is also capped so that sigma^2 * v^2 * curvature / 2 stays within
the room the corner leaves, less what a short move can gain between two
such junctions. A junction where either move has no XY component (a pure
Z move) stops.

An input smoother turns a junction's velocity step dv into acceleration
dv * density over its duration, on top of what the legs command. Like
square_corner_velocity, the junction speed does not limit that burst by
default: CornerAccel.worst is the worst executed per-axis acceleration
the planner admits, the legs' acceleration plus the larger of the biggest
step burst (density_peak * dv, dv from the table at the shaper floor and
max_velocity) and the centripetal acceleration the curve cap allows, with
CLUSTER_ACCEL_MARGIN for corners within one kernel reach; it is logged
when a table is ready, reported in the status, and
recommended_max_accel() inverts it for a motor's acceleration limit. The
optional max_corner_accel bounds the burst instead, as an upper bound:
the legs' acceleration along the axis (the largest share among the moves
in the window) plus the steps of this junction and of the ones before it
within the kernel's support must fit under it at every time. Each step
adds dv * |density| at its offset; the offsets are only known to be at
least the moves' shortest traversal times (from the junction speeds and
the moves' requested speeds), so the bound takes the largest sum over all
placements that keep those gaps, per segment of the kernel (Kernel.shape,
a dynamic programme over the earlier steps).

With input shaping inactive on X or Y every direction change stops (as
square_corner_velocity: 0).

The tables are built in Python (numpy) off the reactor thread and cached
on disk; gcode waits until they are ready. The per-junction limit runs in C
(chelper/corner_deviation.c): at print start the toolhead plans about two
seconds of moves in a single reactor callback.
"""

import hashlib
import logging
import math
import os
import pathlib
import tempfile
import threading

from .. import chelper
from . import input_shaper

TABLE_ALPHA_DEG = 15.0
TABLE_THETA_DEG = 7.5
TABLE_W_POINTS = 48
TABLE_W_MIN = 1e-5
TABLE_W_MAX = 2.0
MODEL_DT = 2e-4
SMOOTHER_NODES = 256
MIN_DIRECTION_CHANGE = 1e-9
MIN_XY_SHARE = 1e-6
FLOOR_MARGIN = 0.10
CLUSTER_ACCEL_MARGIN = 0.25
RECOMMEND_TOLERANCE = 1e-6
SHAPE_NODES = 32
SHORT_MOVE_STOP_DEG = 60.0
TABLE_CACHE_DIR = "~/.cache/kalico/corner_deviation"
BUILD_POLL_S = 0.05
BUILD_NICE = 19
MODULE_SOURCE = pathlib.Path(__file__).read_bytes()


class Kernel:
    """A shaper as shifted weights. For a smoother, shape bounds the
    executed acceleration of a velocity step dv at time offset u after it,
    dv * |density(u)|: segmax[i] is the largest |density| over segment i
    ([u0 + i * h, u0 + (i + 1) * h]), exact over the polynomial (endpoints
    and critical points)."""

    def __init__(self, shifts, weights, density_peak=0.0, shape=None):
        self.shifts = shifts
        self.weights = weights
        self.density_peak = density_peak
        self.shape = shape
        self.variance = sum(w * s * s for w, s in zip(weights, shifts))
        self.s_lo = min(shifts)
        self.s_hi = max(shifts)


def shaper_spec(shaper):
    """The shaper's parameters as plain data, read in the reactor thread
    and handed to the table builder; None when the axis is not shaped."""
    if isinstance(shaper, input_shaper.AxisInputShaper):
        if not shaper.n:
            return None
        return ("pulses", tuple(shaper.A), tuple(shaper.T))
    if isinstance(shaper, input_shaper.AxisInputSmoother):
        if not shaper.smooth_time:
            return None
        return ("smoother", tuple(shaper.coeffs), shaper.smooth_time)
    raise TypeError("unsupported shaper %r" % (type(shaper),))


def kernel_from_spec(spec):
    kind, first, second = spec
    if kind == "pulses":
        total = sum(first)
        t_offs = sum(a * t for a, t in zip(first, second)) / total
        return Kernel([t_offs - t for t in second], [a / total for a in first])
    return _smoother_kernel(list(first), second)


def _max_abs(poly, critical, lo, hi):
    points = [lo, hi] + [x for x in critical if lo < x < hi]
    return max(abs(float(poly(x))) for x in points)


def _step_shape(density, critical, hst, t_offs):
    """Kernel.shape in u = tau - t_offs, the time after the step."""
    h = 2.0 * hst / SHAPE_NODES
    return {
        "u0": -hst - t_offs,
        "h": h,
        "segmax": [
            _max_abs(density, critical, -hst + i * h, -hst + (i + 1) * h)
            for i in range(SHAPE_NODES)
        ],
    }


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
    critical = [
        r.real
        for r in density.deriv().roots()
        if abs(r.imag) < 1e-12 and -hst <= r.real <= hst
    ]
    return Kernel(
        [t_offs - tau for tau in taus],
        [w / total for w in weights],
        _max_abs(density, critical, -hst, hst),
        _step_shape(density, critical, hst, t_offs),
    )


def build_table_arrays(kx, ky):
    """The table as arrays: g[ia][it][iw] (cells conservative over their
    four grid corners), floor[ia][it], w_floor[ia][it], and w[iw]."""
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
            np.stack([_leg_offsets(np, t, k, w, side, s) for k in (kx, ky)], -1)
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
    g, floor = np.array(g), np.array(floor)
    cell_g = np.maximum(
        np.maximum(g[:-1, :-1], g[:-1, 1:]), np.maximum(g[1:, :-1], g[1:, 1:])
    )
    cell_floor = np.minimum(
        np.minimum(floor[:-1, :-1], floor[:-1, 1:]),
        np.minimum(floor[1:, :-1], floor[1:, 1:]),
    )
    w_list = w.tolist()
    w_floor = np.array(
        [
            [_max_w(w_list, cg.tolist(), cf) for cg, cf in zip(rg, rf)]
            for rg, rf in zip(cell_g, cell_floor)
        ]
    )
    return {"g": cell_g, "floor": cell_floor, "w_floor": w_floor, "w": w}


def _max_w(w, g, level):
    """Largest w with g(w) <= level (g is non-decreasing)."""
    if level < g[0]:
        return 0.0
    lo, hi = 0, len(w) - 1
    if g[hi] <= level:
        return w[hi]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if g[mid] <= level:
            lo = mid
        else:
            hi = mid
    frac = (level - g[lo]) / (g[hi] - g[lo]) if g[hi] > g[lo] else 0.0
    return w[lo] + frac * (w[hi] - w[lo])


def table_arrays(specs, kx, ky, cache_dir):
    """build_table_arrays, cached in cache_dir by module source and shaper
    parameters."""
    import numpy as np

    key = hashlib.sha256(MODULE_SOURCE + repr(specs).encode()).hexdigest()
    path = os.path.join(cache_dir, key + ".npz")
    if os.path.exists(path):
        with np.load(path, allow_pickle=False) as data:
            return {name: data[name] for name in data.files}
    arrays = build_table_arrays(kx, ky)
    fd, tmp = tempfile.mkstemp(dir=cache_dir, suffix=".npz")
    with os.fdopen(fd, "wb") as f:
        np.savez(f, **arrays)
    os.replace(tmp, path)
    return arrays


class JunctionLimit:
    """The C junction limit (chelper/corner_deviation.c) over one table:
    cell (alpha, turn) holds g, the isolated corner's deviation per unit
    accel at w = v / accel, floor, the shaper floor per unit accel, and
    w_floor, the largest w whose corner stays within it."""

    def __init__(self, arrays, kx, ky, max_corner_accel):
        ffi, lib = chelper.get_ffi()
        self.junction_v2 = lib.cd_junction_v2
        self.accel_model = CornerAccel(arrays, kx, ky)
        self.owned = []
        c = self.c = ffi.new("struct cd_limiter *")
        c.n_alpha, c.n_theta = arrays["w_floor"].shape
        c.alpha_step = math.radians(TABLE_ALPHA_DEG)
        c.theta_step = math.radians(TABLE_THETA_DEG)
        c.floor = self._doubles(ffi, arrays["floor"].ravel())
        c.w_floor = self._doubles(ffi, arrays["w_floor"].ravel())
        c.variance = max(kx.variance, ky.variance)
        c.window = max(kx.s_hi, ky.s_hi, -kx.s_lo, -ky.s_lo)
        c.has_corner_bound = max_corner_accel is not None and bool(
            kx.density_peak or ky.density_peak
        )
        c.max_corner_accel = max_corner_accel or 0.0
        c.density_peak[0] = kx.density_peak
        c.density_peak[1] = ky.density_peak
        for axis, kernel in enumerate((kx, ky)):
            shape = kernel.shape
            if shape is None:
                continue
            c.shape[axis].n = len(shape["segmax"])
            c.shape[axis].u0 = shape["u0"]
            c.shape[axis].h = shape["h"]
            c.shape[axis].segmax = shape["segmax"]
        c.min_direction_change = MIN_DIRECTION_CHANGE
        c.min_xy_share = MIN_XY_SHARE
        c.short_turn = math.radians(SHORT_MOVE_STOP_DEG)

    def _doubles(self, ffi, values):
        array = ffi.new("double[]", [float(v) for v in values])
        self.owned.append(array)
        return array

    def counters(self):
        c = self.c
        return {
            "limited": c.limited,
            "step_limited": c.step_limited,
            "xy_stops": c.xy_stops,
        }


def _max_abs_on(np, lo, hi, func, peak_at):
    """Largest |func| on [lo, hi], func being sin or cos, whose |func| is
    1 at peak_at + k * pi and monotonic in between."""
    k = np.ceil((lo - peak_at) / math.pi)
    contains_peak = peak_at + k * math.pi <= hi
    return np.where(
        contains_peak, 1.0, np.maximum(np.abs(func(lo)), np.abs(func(hi)))
    )


class CornerAccel:
    """The worst executed per-axis acceleration the junction limit admits.
    At an isolated corner the shaped legs command at most accel * share
    along an axis, share the larger axis component of the two directions,
    and the junction at speed v adds at most density_peak * v * du, du the
    direction change along the axis: |du_x| = 2 |sin(alpha + turn/2)|
    |sin(turn/2)|, |du_y| = 2 |cos(alpha + turn/2)| |sin(turn/2)|. Both are
    bounded over each table cell. On a curve at the curve cap the
    tangential accel and the centripetal 2 * accel * room / sigma^2 are
    perpendicular: at most their hypot along any axis. The steps of
    corners closer than the kernel's reach add up beyond both;
    CLUSTER_ACCEL_MARGIN is the stated allowance for that, checked by
    tools/motion_eval/fuzz.py on clustered corners. With max_corner_accel
    the junctions bound that sum themselves."""

    def __init__(self, arrays, kx, ky):
        import numpy as np

        self.arrays = arrays
        self.variance = max(kx.variance, ky.variance)
        self.density_peak = (kx.density_peak, ky.density_peak)
        n_alpha, n_theta = arrays["w_floor"].shape
        alpha_step = math.radians(TABLE_ALPHA_DEG)
        theta_step = math.radians(TABLE_THETA_DEG)
        alpha_lo = alpha_step * np.arange(n_alpha)[:, None]
        alpha_hi = alpha_lo + alpha_step
        theta_lo = -math.pi + theta_step * np.arange(n_theta)[None, :]
        theta_hi = theta_lo + theta_step
        half_turn = np.maximum(
            np.abs(np.sin(0.5 * theta_lo)), np.abs(np.sin(0.5 * theta_hi))
        )
        mid = (alpha_lo + 0.5 * theta_lo, alpha_hi + 0.5 * theta_hi)
        out = (alpha_lo + theta_lo, alpha_hi + theta_hi)
        self.du_max, self.share_max = [], []
        for func, peak_at in ((np.cos, 0.0), (np.sin, 0.5 * math.pi)):
            step_func, step_peak = (
                (np.sin, 0.5 * math.pi) if func is np.cos else (np.cos, 0.0)
            )
            self.du_max.append(
                2.0 * half_turn * _max_abs_on(np, *mid, step_func, step_peak)
            )
            self.share_max.append(
                np.maximum(
                    _max_abs_on(np, alpha_lo, alpha_hi, func, peak_at),
                    _max_abs_on(np, *out, func, peak_at),
                )
            )

    def worst(self, accel, max_velocity, max_corner_accel):
        """(x, y) worst executed acceleration at this max_accel; with
        max_corner_accel, the bound."""
        speed = (accel * self.arrays["w_floor"]).clip(max=max_velocity)
        centripetal = (
            2.0 * accel * float(self.arrays["floor"].max()) / self.variance
        )
        curve = math.hypot(accel, centripetal)
        out = []
        for density_peak, du_max, share_max in zip(
            self.density_peak, self.du_max, self.share_max
        ):
            corner = float(
                (accel * share_max + density_peak * speed * du_max).max()
            )
            axis = (1.0 + CLUSTER_ACCEL_MARGIN) * max(corner, curve)
            if max_corner_accel is not None and density_peak:
                axis = min(axis, max(max_corner_accel, accel))
            out.append(float(axis))
        return tuple(out)

    def recommended_max_accel(
        self, motor_accel, max_velocity, max_corner_accel
    ):
        """The largest max_accel whose worst corner stays within motor_accel
        on both axes (worst grows with max_accel: bisection)."""

        def fits(accel):
            return (
                max(self.worst(accel, max_velocity, max_corner_accel))
                <= motor_accel
            )

        lo, hi = 0.0, motor_accel
        if fits(hi):
            return hi
        while hi - lo > RECOMMEND_TOLERANCE * motor_accel:
            mid = 0.5 * (lo + hi)
            if fits(mid):
                lo = mid
            else:
                hi = mid
        return lo


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
        self.reactor = self.printer.get_reactor()
        self.max_corner_accel = config.getfloat(
            "max_corner_accel", None, above=0.0
        )
        printer_section = config.getsection("printer")
        if printer_section.get("square_corner_velocity", None) is not None:
            raise config.error(
                "[corner_deviation] replaces square_corner_velocity;"
                " remove square_corner_velocity from [printer]"
            )
        self.cache_dir = os.path.expanduser(
            config.get("table_cache_dir", TABLE_CACHE_DIR)
        )
        try:
            os.makedirs(self.cache_dir, exist_ok=True)
        except OSError as e:
            raise config.error(
                "[corner_deviation] table_cache_dir %s: %s"
                % (self.cache_dir, e)
            )
        if not os.access(self.cache_dir, os.W_OK):
            raise config.error(
                "[corner_deviation] table_cache_dir %s is not writable"
                % (self.cache_dir,)
            )
        self.toolhead = None
        self.gcode_mutex = None
        self.holding_gcode = False
        self.ready = False
        self.input_shaper = None
        self.limit = None
        self.last_move = None
        self.reported_inactive = False
        self.building = False
        self.corner_accel_key = self.corner_accel = None
        self.generation = 0
        self.counters = {
            "limited": 0,
            "step_limited": 0,
            "xy_stops": 0,
        }
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("klippy:ready", self._ready)

    def _connect(self):
        self.input_shaper = self.printer.lookup_object("input_shaper", None)
        if self.input_shaper is None:
            raise self.printer.config_error(
                "[corner_deviation] requires [input_shaper]"
            )
        gcode = self.printer.lookup_object("gcode")
        self.gcode_mutex = gcode.get_mutex()
        real_update = self.input_shaper._update_input_shaping

        def update_input_shaping(*args, **kwargs):
            real_update(*args, **kwargs)
            self._refresh_kernels()

        self.input_shaper._update_input_shaping = update_input_shaping
        self._refresh_kernels()
        self.toolhead = self.printer.lookup_object("toolhead")
        self.toolhead.set_junction_limiter(self.junction_v2)
        real_set_velocity_limit = gcode.register_command(
            "SET_VELOCITY_LIMIT", None
        )

        def cmd_SET_VELOCITY_LIMIT(gcmd):
            if gcmd.get("SQUARE_CORNER_VELOCITY", None) is not None:
                raise gcmd.error(
                    "SQUARE_CORNER_VELOCITY has no effect with"
                    " [corner_deviation]"
                )
            real_set_velocity_limit(gcmd)

        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            cmd_SET_VELOCITY_LIMIT,
            desc="Set printer velocity limits",
        )
        gcode.register_command(
            "CORNER_DEVIATION_REPORT",
            self.cmd_CORNER_DEVIATION_REPORT,
            desc=self.cmd_CORNER_DEVIATION_REPORT_help,
        )

    def _ready(self):
        self.ready = True

    def _refresh_kernels(self):
        """Read the shapers here, build kernels and table in a thread: the
        reactor never runs the build, a callback of seconds would starve
        heaters and the MCU queues. No move is planned before the table is
        published: before klippy:ready the gcode mutex is held until then
        (Klipper becomes ready, gcode waits; batch plans do not depend on
        the thread's timing), after it the caller is a gcode command
        (SET_INPUT_SHAPER, holding the mutex) and pauses until then."""
        shapers = {s.get_axis(): s for s in self.input_shaper.get_shapers()}
        specs = (shaper_spec(shapers["x"]), shaper_spec(shapers["y"]))
        self.generation += 1
        self._retire_limit()
        self.building = None not in specs
        if not self.building:
            self._release_gcode()
            return
        chelper.get_ffi()
        thread = threading.Thread(
            target=self._build,
            args=(self.generation, specs),
            name="corner_deviation",
            daemon=True,
        )
        thread.start()
        if not self.ready:
            if not self.holding_gcode:
                self.gcode_mutex.lock()
                self.holding_gcode = True
            return
        while self.building:
            self.reactor.pause(self.reactor.monotonic() + BUILD_POLL_S)

    def _release_gcode(self):
        if self.holding_gcode:
            self.holding_gcode = False
            self.gcode_mutex.unlock()

    def _build(self, generation, specs):
        os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), BUILD_NICE)
        try:
            kx, ky = (kernel_from_spec(s) for s in specs)
            arrays = table_arrays(specs, kx, ky, self.cache_dir)
            limit = JunctionLimit(arrays, kx, ky, self.max_corner_accel)
            result = (limit, None)
        except Exception as e:
            logging.exception("corner_deviation: table build failed")
            result = (None, e)
        self.reactor.register_async_callback(
            lambda eventtime: self._publish(generation, result)
        )

    def _publish(self, generation, result):
        if generation != self.generation:
            return
        limit, error = result
        self.building = False
        self._release_gcode()
        if error is not None:
            self.printer.invoke_shutdown(
                "corner_deviation: table build failed: %s" % (error,)
            )
            return
        self.limit = limit
        self.reported_inactive = False
        logging.info("corner_deviation: table ready; %s", self._accel_summary())

    def _retire_limit(self):
        if self.limit is not None:
            for name, count in self.limit.counters().items():
                self.counters[name] += count
        self.limit = None

    def get_status(self, eventtime):
        counters = dict(self.counters)
        if self.limit is not None:
            for name, count in self.limit.counters().items():
                counters[name] += count
        corner_accel = self._corner_accel()
        return dict(
            counters,
            max_corner_accel=self.max_corner_accel,
            table_ready=self.limit is not None,
            corner_accel_x=corner_accel and corner_accel[0],
            corner_accel_y=corner_accel and corner_accel[1],
        )

    def _corner_accel(self):
        """Worst executed (x, y) acceleration admitted at the toolhead's
        current max_accel and max_velocity; None without a table."""
        if self.limit is None:
            return None
        key = (self.limit, self.toolhead.max_accel, self.toolhead.max_velocity)
        if self.corner_accel_key != key:
            self.corner_accel_key = key
            self.corner_accel = self.limit.accel_model.worst(
                self.toolhead.max_accel,
                self.toolhead.max_velocity,
                self.max_corner_accel,
            )
        return self.corner_accel

    def _accel_summary(self):
        x, y = self._corner_accel()
        return "worst corner accel X %.0f Y %.0f mm/s^2 at max_accel %.0f" % (
            x,
            y,
            self.toolhead.max_accel,
        )

    cmd_CORNER_DEVIATION_REPORT_help = (
        "Report the worst corner acceleration, and the max_accel that keeps"
        " it within MOTOR_ACCEL"
    )

    def cmd_CORNER_DEVIATION_REPORT(self, gcmd):
        motor_accel = gcmd.get_float("MOTOR_ACCEL", None, above=0.0)
        if self.limit is None:
            raise gcmd.error("corner_deviation: no corner table (shaping off)")
        msg = self._accel_summary()
        if motor_accel is not None:
            recommended = self.limit.accel_model.recommended_max_accel(
                motor_accel,
                self.toolhead.max_velocity,
                self.max_corner_accel,
            )
            msg += "\nrecommended_max_accel %.0f for MOTOR_ACCEL %.0f" % (
                recommended,
                motor_accel,
            )
        gcmd.respond_info(msg)

    def junction_v2(self, prev_move, move, cap_v2):
        limit = self.limit
        if limit is None:
            if self.building:
                self.printer.invoke_shutdown(
                    "corner_deviation: junction planned while the corner"
                    " table is building"
                )
            elif not self.reported_inactive:
                logging.warning(
                    "corner_deviation: input shaping inactive on X or Y;"
                    " every direction change stops"
                )
                self.reported_inactive = True
            return 0.0
        prev_r, move_r = prev_move.axes_r, move.axes_r
        chained_from = self.last_move
        self.last_move = move
        return limit.junction_v2(
            limit.c,
            prev_r[0],
            prev_r[1],
            move_r[0],
            move_r[1],
            prev_move.move_d,
            move.move_d,
            prev_move.accel,
            move.accel,
            prev_move.max_cruise_v2,
            prev_move is chained_from,
            cap_v2,
        )


def load_config(config):
    return CornerDeviation(config)
