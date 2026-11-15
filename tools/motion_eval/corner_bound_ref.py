"""Python reference of the max_corner_accel bound of
klippy/chelper/corner_deviation.c, kept operation for operation so the
tests can compare the two (tests/test_corner_bound_ref.py) and analysis
scripts can inspect which segment, row and leg stretch set a junction's
speed.

ReferenceLimit.junction_v2 mirrors cd_junction_v2 for a turning junction
whose table speed is given (the table and curve rules are not part of
the bound) and keeps the same step history as the C limiter.
"""

from __future__ import annotations

import dataclasses
import math

from klippy.extras import corner_deviation as cd

STEPS = 256
SOLVER_STEPS = 6
SOLVER_TOLERANCE = 0.005
WINDOW, PREV_DOWN, NEXT_UP, NEXT_FREE, FUTURE = range(5)
REGIONS = ("window", "prev_down", "next_up", "next_free", "future")
BOUNDARIES = 4


@dataclasses.dataclass
class Step:
    tau: float
    v: float
    accel: float
    du: tuple
    leg: tuple


@dataclasses.dataclass
class RowBound:
    row: tuple
    polarity: float
    own: list
    earlier: list
    weight: list
    base: float = 0.0
    margin: float = 0.0
    rise: list = None
    fall: list = None


@dataclasses.dataclass
class Junction:
    prev_accel: float
    move_accel: float
    forward_v2: float
    move_d: float
    prev_tau: float
    move_tau: float
    bounds: list = None
    at: list = None
    next_end: list = None


def positive(x):
    return x if x > 0.0 else 0.0


def leg_weight(cx, cy, sign):
    if sign:
        return [
            positive(sign * (cx + cy)),
            positive(sign * cx),
            positive(sign * cy),
        ]
    return [abs(cx + cy), abs(cx), abs(cy)]


def traverse_time(length, va, vb, accel, vmax):
    peak = math.sqrt(0.5 * (va * va + vb * vb) + accel * length)
    peak = min(peak, vmax)
    return length / peak if peak > 0.0 else 0.0


class ReferenceLimit:
    def __init__(self, kx, ky, max_corner_accel, rows):
        self.shape = cd.bound_shape(kx, ky)
        self.n = cd.BOUND_SEGMENTS
        self.cap = max_corner_accel
        self.rows = [tuple(r) for r in rows]
        self.steps = []
        self.step_limited = 0
        self.last_junction = None

    def burst_segmax(self, cx, cy):
        sh = self.shape
        wx = sh["wmax"][0] if cx > 0.0 else sh["wmin"][0]
        wy = sh["wmax"][1] if cy > 0.0 else sh["wmin"][1]
        return [positive(cx * wx[i] + cy * wy[i]) for i in range(self.n)]

    def segments_in(self, offset):
        s = math.floor(offset / self.shape["h"])
        return int(min(max(s, -self.n), self.n))

    def shifted(self, later, i, shift):
        k = i + shift
        if k >= self.n:
            return 0.0
        return later[max(k, 0)]

    def earlier_steps(self, window, row, polarity):
        best, gap, any_step = [0.0] * self.n, 0.0, False
        for k in range(len(window) - 1, -1, -1):
            step, _ = window[k]
            scale = polarity * step.v
            cx = scale * row[0] * step.du[0]
            cy = scale * row[1] * step.du[1]
            if k < len(window) - 1:
                gap += window[k + 1][1]
            if cx <= 0.0 and cy <= 0.0:
                continue
            here, running = self.burst_segmax(cx, cy), 0.0
            shift = self.segments_in(gap)
            for i in range(self.n - 1, -1, -1):
                if any_step:
                    here[i] += self.shifted(best, i, shift)
                running = max(running, here[i])
                here[i] = running
            best, any_step, gap = here, True, 0.0
        if not any_step:
            return [0.0] * self.n
        shift = self.segments_in(gap + window[0][1])
        return [self.shifted(best, i, shift) for i in range(self.n)]

    def masses_past(self, s):
        """Kernel masses past a boundary at time s after the junction, at
        each node of the window."""
        sh = self.shape
        shift = min(max(-s, -sh["cdf_pad"]), sh["cdf_pad"])
        x = (shift + sh["cdf_pad"]) / sh["cdf_h"]
        k = min(int(x), cd.BOUND_CDF_NODES - 2 - self.n * cd.BOUND_CDF_SUB)
        f = x - k
        out = []
        for m in range(3):
            table = sh["cdf"][m]
            row = []
            for i in range(self.n + 1):
                p = k + i * cd.BOUND_CDF_SUB
                row.append(table[p] + f * (table[p + 1] - table[p]))
            out.append(row)
        return out

    @staticmethod
    def prev_decel_time(j, v):
        peak = math.sqrt(0.5 * (j.forward_v2 + v * v))
        return min(positive((peak - v) / j.prev_accel), j.prev_tau)

    @staticmethod
    def next_decel_time(j, v):
        reach_v2 = max(2.0 * j.move_accel * j.move_d, v * v)
        t = positive((math.sqrt(0.5 * (reach_v2 + v * v)) - v) / j.move_accel)
        return min(t, j.move_tau)

    def boundary_masses(self, j, v):
        return [
            self.masses_past(-self.prev_decel_time(j, v)),
            j.at,
            self.masses_past(self.next_decel_time(j, v)),
            j.next_end,
        ]

    def legs(self, rb, mass):
        """The legs' bound on each segment."""
        legs = [rb.base + rb.margin] * self.n
        for k in range(BOUNDARIES):
            for m in range(3):
                rise, fall, past = rb.rise[k][m], rb.fall[k][m], mass[k][m]
                if rise:
                    legs = [legs[i] + rise * past[i + 1] for i in range(self.n)]
                if fall:
                    legs = [legs[i] - fall * past[i] for i in range(self.n)]
        return legs

    def fitting_speed(self, j, v, trace=None):
        mass = self.boundary_masses(j, v)
        best = math.inf
        for rb in j.bounds:
            legs = self.legs(rb, mass)
            for i in range(self.n):
                if not rb.own[i]:
                    continue
                room = self.cap - rb.earlier[i] - legs[i]
                if trace is not None:
                    trace.append((rb, i, room, legs[i]))
                if room <= 0.0:
                    return 0.0
                if room < best * rb.own[i]:
                    best = room / rb.own[i]
        return best

    def set_legs(self, rb):
        sh = self.shape
        rb.base = sum(rb.weight[WINDOW][m] * sh["total"][m] for m in range(3))
        rb.margin = 0.0
        rb.rise, rb.fall = [], []
        for r in range(1, 5):
            rise, fall = [], []
            for m in range(3):
                jump = rb.weight[r][m] - rb.weight[r - 1][m]
                rise.append(positive(jump))
                fall.append(positive(-jump))
                rb.margin += abs(jump) * sh["cdf_margin"]
            rb.rise.append(rise)
            rb.fall.append(fall)

    def setup_bounds(self, j, window, up, um):
        du = (um[0] - up[0], um[1] - up[1])
        prev_leg = (j.prev_accel * up[0], j.prev_accel * up[1])
        move_leg = (j.move_accel * um[0], j.move_accel * um[1])
        accel = max([j.prev_accel, j.move_accel] + [s.accel for s, _ in window])
        j.bounds = []
        for row in self.rows:
            pc = (row[0] * prev_leg[0], row[1] * prev_leg[1])
            nc = (row[0] * move_leg[0], row[1] * move_leg[1])
            window_w = leg_weight(pc[0], pc[1], 0.0)
            for step, _ in window:
                w = leg_weight(row[0] * step.leg[0], row[1] * step.leg[1], 0.0)
                window_w = [max(a, b) for a, b in zip(window_w, w)]
            for polarity in (1.0, -1.0):
                cx = polarity * row[0] * du[0]
                cy = polarity * row[1] * du[1]
                if cx <= 0.0 and cy <= 0.0:
                    continue
                weight = [
                    list(window_w),
                    leg_weight(pc[0], pc[1], -polarity),
                    leg_weight(nc[0], nc[1], polarity),
                    leg_weight(nc[0], nc[1], 0.0),
                    [
                        accel * math.hypot(*row),
                        accel * abs(row[0]),
                        accel * abs(row[1]),
                    ],
                ]
                rb = RowBound(
                    row,
                    polarity,
                    self.burst_segmax(cx, cy),
                    self.earlier_steps(window, row, polarity),
                    weight,
                )
                self.set_legs(rb)
                j.bounds.append(rb)
        j.at = self.masses_past(0.0)
        j.next_end = self.masses_past(j.move_tau)

    def corner_bound(self, j, speed):
        fits = self.fitting_speed(j, speed)
        if fits >= speed:
            return speed
        self.step_limited += 1
        safe, unsafe, unsafe_gap = fits, speed, fits - speed
        safe_gap = self.fitting_speed(j, safe) - safe
        for _ in range(SOLVER_STEPS):
            tolerance = SOLVER_TOLERANCE * unsafe
            if unsafe - safe <= tolerance or safe_gap <= tolerance:
                break
            nxt = 0.5 * (safe + unsafe)
            if safe_gap > 0.0 and safe_gap - unsafe_gap > 0.0:
                nxt = safe + safe_gap * (unsafe - safe) / (
                    safe_gap - unsafe_gap
                )
            if not safe < nxt < unsafe:
                nxt = 0.5 * (safe + unsafe)
            fits = self.fitting_speed(j, nxt)
            if fits >= nxt:
                safe, safe_gap = nxt, fits - nxt
            else:
                unsafe, unsafe_gap = nxt, fits - nxt
                if fits > safe:
                    safe = fits
                    safe_gap = self.fitting_speed(j, safe) - safe
        return safe

    def window_steps(self, tau):
        sh = self.shape
        span, elapsed, window = self.n * sh["h"], tau, []
        for step in reversed(self.steps):
            if elapsed > span:
                return window
            gap = window[-1][0].tau if window else tau
            window.append((step, gap))
            elapsed += step.tau
        if elapsed > span or len(self.steps) < STEPS:
            return window
        return None

    def junction_v2(
        self,
        up,
        um,
        prev_d,
        move_d,
        prev_accel,
        move_accel,
        prev_max_v2,
        move_max_v2,
        prev_start_v2,
        cap_v2,
        table_speed,
    ):
        """cd_junction_v2 for a turning junction with the given table
        speed."""
        prev_xy = math.hypot(*up)
        prev_max_v = math.sqrt(prev_max_v2)
        cap = math.sqrt(cap_v2)
        speed = min(table_speed, cap)
        if speed > 0.0:
            speed = self.bounded_speed(
                up,
                um,
                prev_d,
                move_d,
                prev_accel,
                move_accel,
                prev_max_v,
                move_max_v2,
                prev_start_v2,
                speed,
            )
        v2 = speed * speed if speed < cap else cap_v2
        self.note_step(
            traverse_time(
                prev_d * prev_xy,
                math.sqrt(prev_start_v2),
                math.sqrt(v2),
                prev_accel,
                prev_max_v,
            ),
            v2,
            up,
            um,
            prev_accel,
        )
        return v2

    def bounded_speed(
        self,
        up,
        um,
        prev_d,
        move_d,
        prev_accel,
        move_accel,
        prev_max_v,
        move_max_v2,
        prev_start_v2,
        speed,
    ):
        prev_xy, move_xy = math.hypot(*up), math.hypot(*um)
        move_max_v = math.sqrt(move_max_v2)
        j = Junction(
            prev_accel,
            move_accel,
            prev_start_v2 + 2.0 * prev_accel * prev_d,
            move_d,
            traverse_time(
                prev_d * prev_xy,
                math.sqrt(prev_start_v2),
                speed,
                prev_accel,
                prev_max_v,
            ),
            traverse_time(
                move_d * move_xy, speed, move_max_v, move_accel, move_max_v
            ),
        )
        window = self.window_steps(j.prev_tau)
        if window is None:
            return 0.0
        self.setup_bounds(j, window, up, um)
        self.last_junction = j
        return self.corner_bound(j, speed)

    def note_step(self, tau, v2, up, um, prev_accel):
        self.steps.append(
            Step(
                tau,
                math.sqrt(v2),
                prev_accel,
                (um[0] - up[0], um[1] - up[1]),
                (prev_accel * up[0], prev_accel * up[1]),
            )
        )
        del self.steps[:-STEPS]
