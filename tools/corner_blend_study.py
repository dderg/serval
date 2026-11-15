#!/usr/bin/env python3
"""Which corner blend is fastest within the same corner budget?

One isolated corner between two long lines, planned five ways and passed
through the X and Y smoothing kernels of a machine profile:

- ``stop``: no blend, a full stop at the vertex;
- ``clothoid``: the clothoid pair the fitter cuts today (curvature ramps up
  to the apex and down again, the normal acceleration eases in);
- ``arc``: a circular arc tangent to both lines (the normal acceleration
  steps on at its entry and off at its exit);
- ``parabola``: the path one constant acceleration vector traces between
  the lines (the acceleration vector jumps at entry and exit, and is at
  its limit throughout);
- ``sharp at speed``: no blend, the vertex crossed at a speed (the velocity
  turns in one instant, as Klipper's square corner velocity plans it).

The motion is the planner's own canonical one (``shaped_corner.rs``): from
the apex outwards, both ways, at the most tangential acceleration the disk
``|a| <= max_accel`` leaves. The miss is the shaped path's closest approach
to the vertex, worst over the corner's orientation against the axes, and
each plan is held to the planner's prediction of it, margin included.

Per plan the study reports the corner's cost (time lost against turning at
the feedrate in no time), the shaped acceleration the motors are asked
for, and what the corner leaves ringing across the outgoing line in a
damped mode on each axis, at the shaper's frequency and away from it.

    uv run --group prototype python tools/corner_blend_study.py
    uv run --group prototype python tools/corner_blend_study.py --engine

``--engine`` plans the same corners with the snapshot engine
(``make -f Makefile.rust motion-engine-fast``) and compares its time and
shaped miss with the study's model of today's planner.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from pathlib import Path

import numpy as np
from scipy import signal

REPO = Path(__file__).resolve().parent.parent

DT_S = 5e-6
PREDICTION_MARGIN = 1.20
DEFAULT_ALLOWANCE_MM = 0.04
ORIENTATIONS = 12
PLANNER_CUT_CANDIDATES = 16
PLANNER_RATIO_NODES = 5
SEARCH_CUT_OCTAVES = 7
SEARCH_CUTS_PER_OCTAVE = 4
SEARCH_RATIO_NODES = 13
SHARP_SPEED_NODES = 129
SEARCH_SIGMAS = 3.0
RING_TAIL_S = 0.005
MODE_DAMPING_RATIO = 0.1
MODE_FREQUENCY_FACTORS = (0.85, 1.0, 1.15, 2.0, 3.0, 5.0)
FEASIBLE_SLACK_MM = 1e-4
UNSHAPED = "none"
WORKERS = 8
QUADRATURE_NODES = 200001
BLEND_WINDOW_FACTOR = 2.5
WINDOW_MARGIN_S = 1e-3

SMOOTH_ZV_UNIT_COEFFS = (
    0.01966833207740377,
    -1.465471373781904,
    29.52796003014231,
    5.861885495127615,
    -118.4265334338076,
)
SMOOTH_MZV_UNIT_COEFFS = (
    1.713117990217123,
    1.57172781617736,
    -62.18762409216703,
    -37.75923018121473,
    698.0200035767849,
    125.8892756660212,
    -1906.717580206364,
)
SMOOTHERS = {
    "smooth_zv": (SMOOTH_ZV_UNIT_COEFFS, 0.8025),
    "smooth_mzv": (SMOOTH_MZV_UNIT_COEFFS, 0.95625),
}


class Shape(Enum):
    CLOTHOID = "clothoid"
    ARC = "arc"
    PARABOLA = "parabola"
    SHARP = "sharp"


class BudgetKind(Enum):
    OVER_FLOOR = "over_floor"
    SHAPED = "corner_deviation"
    SCV = "square_corner_velocity"


@dataclass(frozen=True)
class Kernel:
    """A smoothing kernel sampled every DT_S, weights of unit sum."""

    frequency_hz: float
    first: int
    weights: np.ndarray

    @classmethod
    def of(cls, kind: str, frequency_hz: float) -> Kernel:
        if kind == UNSHAPED:
            return cls(frequency_hz, 0, np.ones(1))
        coeffs, duration_per_hz = SMOOTHERS[kind]
        duration = duration_per_hz / frequency_hz
        unit = np.linspace(-0.5, 0.5, QUADRATURE_NODES)
        shape = np.polynomial.polynomial.polyval(unit, coeffs)
        mean = np.trapezoid(shape * unit, unit) / np.trapezoid(shape, unit)
        lo = (-0.5 - mean) * duration
        hi = (0.5 - mean) * duration
        offsets = np.arange(math.floor(lo / DT_S), math.ceil(hi / DT_S) + 1)
        at = offsets * DT_S
        weights = np.where(
            (at >= lo) & (at < hi),
            np.polynomial.polynomial.polyval(at / duration + mean, coeffs),
            0.0,
        )
        return cls(frequency_hz, int(offsets[0]), weights / weights.sum())

    @property
    def offsets_s(self) -> np.ndarray:
        return (self.first + np.arange(len(self.weights))) * DT_S

    @property
    def reach_s(self) -> float:
        return float(np.abs(self.offsets_s).max())

    @property
    def sigma_s(self) -> float:
        return math.sqrt(float(np.sum(self.weights * self.offsets_s**2)))

    def shape(self, series: np.ndarray) -> np.ndarray:
        """`series` [N, M] convolved along time, NaN where it runs out."""
        taps = len(self.weights)
        full = signal.fftconvolve(series, self.weights[None, :], axes=1)
        shaped = np.full(series.shape, np.nan)
        lo = taps - 1 + self.first
        hi = series.shape[1] + self.first
        shaped[:, lo:hi] = full[:, taps - 1 : series.shape[1]]
        return shaped


@dataclass(frozen=True)
class Profile:
    machine: str
    accel: float
    feed: float
    x: tuple[str, float]
    y: tuple[str, float]
    budget: BudgetKind
    budget_value: float
    config: str | None = None

    def kernels(self) -> tuple[Kernel, Kernel]:
        return Kernel.of(*self.x), Kernel.of(*self.y)

    @property
    def shaped(self) -> bool:
        return self.x[0] != UNSHAPED or self.y[0] != UNSHAPED

    @property
    def label(self) -> str:
        return f"{self.machine}, {self.budget.value} {self.budget_value:g}"


PROFILES = (
    Profile(
        "no shaper 10k",
        10000.0,
        200.0,
        (UNSHAPED, 50.0),
        (UNSHAPED, 44.0),
        BudgetKind.SHAPED,
        DEFAULT_ALLOWANCE_MM,
    ),
    Profile(
        "bench 10k mzv50/zv44",
        10000.0,
        200.0,
        ("smooth_mzv", 50.0),
        ("smooth_zv", 44.0),
        BudgetKind.OVER_FLOOR,
        DEFAULT_ALLOWANCE_MM,
        "tools/bench/configs/printer_default_corner.cfg",
    ),
    Profile(
        "bench 10k mzv50/zv44",
        10000.0,
        200.0,
        ("smooth_mzv", 50.0),
        ("smooth_zv", 44.0),
        BudgetKind.SHAPED,
        0.21,
        "tools/bench/printer.cfg",
    ),
    Profile(
        "bedslinger 3k mzv42/mzv36",
        3000.0,
        150.0,
        ("smooth_mzv", 42.0),
        ("smooth_mzv", 36.0),
        BudgetKind.OVER_FLOOR,
        DEFAULT_ALLOWANCE_MM,
    ),
    Profile(
        "voron 15k mzv75/mzv62",
        15000.0,
        300.0,
        ("smooth_mzv", 75.0),
        ("smooth_mzv", 62.0),
        BudgetKind.OVER_FLOOR,
        DEFAULT_ALLOWANCE_MM,
    ),
    Profile(
        "voron 15k mzv75/mzv62",
        15000.0,
        300.0,
        ("smooth_mzv", 75.0),
        ("smooth_mzv", 62.0),
        BudgetKind.SCV,
        5.0,
    ),
    Profile(
        "high accel 70k mzv191/mzv129",
        70000.0,
        500.0,
        ("smooth_mzv", 191.0),
        ("smooth_mzv", 129.4),
        BudgetKind.OVER_FLOOR,
        DEFAULT_ALLOWANCE_MM,
    ),
)


def clothoid_half_per_cut(theta: float) -> float:
    along = np.linspace(0.0, 1.0, QUADRATURE_NODES)
    apex_y = np.trapezoid(np.sin(theta * along * along / 2.0), along)
    return math.cos(theta / 2.0) / apex_y


def arc_radius_per_cut(theta: float) -> float:
    return 1.0 / (1.0 / math.cos(theta / 2.0) - 1.0)


def parabola_apex_radius_per_cut(theta: float) -> float:
    return 2.0 / math.tan(theta / 2.0) ** 2


@dataclass
class Blend:
    """N corners of one shape and turn, by cut. A blend is walked from its
    apex by a parameter `q`: the arc length, or on a parabola the distance
    along the apex's tangent."""

    shape: Shape
    theta: float
    cut: np.ndarray
    end: np.ndarray = field(init=False)
    length: np.ndarray = field(init=False)
    kappa_peak: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        theta = self.theta
        if self.shape is Shape.CLOTHOID:
            self.end = self.cut * clothoid_half_per_cut(theta)
            self.length = self.end
            self.kappa_peak = theta / self.end
        elif self.shape is Shape.ARC:
            radius = self.cut * arc_radius_per_cut(theta)
            self.end = radius * theta / 2.0
            self.length = self.end
            self.kappa_peak = 1.0 / radius
        elif self.shape is Shape.PARABOLA:
            radius = self.cut * parabola_apex_radius_per_cut(theta)
            slope = math.tan(theta / 2.0)
            self.end = radius * slope
            self.length = (
                radius
                / 2.0
                * (slope * math.hypot(1.0, slope) + math.asinh(slope))
            )
            self.kappa_peak = 1.0 / radius
        else:
            self.end = np.zeros_like(self.cut)
            self.length = self.end
            self.kappa_peak = np.zeros_like(self.cut)

    def speed_cap(self, accel: float) -> np.ndarray:
        with np.errstate(divide="ignore"):
            return np.sqrt(accel / self.kappa_peak)

    def slope(self, q: np.ndarray) -> np.ndarray:
        return np.minimum(q, self.end) * self.kappa_peak

    def kappa(self, q: np.ndarray) -> np.ndarray:
        inside = q < self.end
        if self.shape is Shape.CLOTHOID:
            return self.kappa_peak * np.clip(1.0 - q / self.end, 0.0, None)
        if self.shape is Shape.ARC:
            return np.where(inside, self.kappa_peak, 0.0)
        if self.shape is Shape.PARABOLA:
            bent = self.kappa_peak * (1.0 + self.slope(q) ** 2) ** -1.5
            return np.where(inside, bent, 0.0)
        return np.zeros_like(q)

    def heading(self, q: np.ndarray) -> np.ndarray:
        if self.shape is Shape.SHARP:
            return np.full_like(q, self.theta)
        if self.shape is Shape.PARABOLA:
            return self.theta / 2.0 + np.arctan(self.slope(q))
        if self.shape is Shape.ARC:
            return self.theta / 2.0 + self.slope(q)
        inside = np.minimum(q, self.end)
        ramp = inside - inside * inside / (2.0 * self.end)
        return self.theta / 2.0 + self.kappa_peak * ramp

    def arc_per_q(self, q: np.ndarray) -> np.ndarray:
        if self.shape is not Shape.PARABOLA:
            return np.ones_like(q)
        return np.where(q < self.end, np.hypot(1.0, self.slope(q)), 1.0)


@dataclass
class Motion:
    """The corners' motion from before to after the apex, every DT_S."""

    blend: Blend
    accel: float
    feed: float
    x: np.ndarray
    y: np.ndarray
    speed_out: np.ndarray
    q_out: np.ndarray

    @property
    def steps(self) -> int:
        return self.speed_out.shape[1] - 1

    @property
    def apex_index(self) -> int:
        return self.steps

    def half_cost_s(self) -> np.ndarray:
        """Time lost from the vertex on, against leaving it at the feed."""
        theta = self.blend.theta
        end_speed = self.speed_out[:, -1]
        past_vertex = self.x[:, -1] * math.cos(theta) + self.y[
            :, -1
        ] * math.sin(theta)
        assert np.all(self.q_out[:, -1] >= self.blend.end), (
            "the window ends inside a blend"
        )
        to_feed = np.clip(self.feed - end_speed, 0.0, None)
        time_at_feed = self.steps * DT_S + to_feed / self.accel
        path_at_feed = past_vertex + (
            np.maximum(self.feed, end_speed) ** 2 - end_speed**2
        ) / (2.0 * self.accel)
        return time_at_feed - path_at_feed / self.feed

    def exit_s(self) -> np.ndarray:
        inside = self.q_out < self.blend.end[:, None]
        return inside.sum(axis=1) * DT_S


def move(
    blend: Blend, apex_speed: np.ndarray, accel: float, feed: float, steps: int
) -> Motion:
    theta = blend.theta
    count = len(blend.cut)
    inward = (-math.sin(theta / 2.0), math.cos(theta / 2.0))

    def rates(state: np.ndarray) -> np.ndarray:
        q, v = state[0], state[1]
        normal = v * v * blend.kappa(q)
        tangential = np.sqrt(
            np.clip(accel * accel - normal * normal, 0.0, None)
        )
        tangential = np.where(v >= feed, 0.0, tangential)
        heading = blend.heading(q)
        return np.stack(
            [
                v / blend.arc_per_q(q),
                tangential,
                v * np.cos(heading),
                v * np.sin(heading),
            ]
        )

    state = np.stack(
        [
            np.zeros(count),
            np.minimum(apex_speed, feed),
            blend.cut * inward[0],
            blend.cut * inward[1],
        ]
    )
    out = np.empty((4, count, steps + 1))
    out[:, :, 0] = state
    h = DT_S
    for k in range(1, steps + 1):
        k1 = rates(state)
        k2 = rates(state + 0.5 * h * k1)
        k3 = rates(state + 0.5 * h * k2)
        k4 = rates(state + h * k3)
        state = state + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        state[1] = np.minimum(state[1], feed)
        out[:, :, k] = state

    q_out, speed_out, x_out, y_out = out
    along_inward = x_out * inward[0] + y_out * inward[1]
    x_in = 2.0 * along_inward * inward[0] - x_out
    y_in = 2.0 * along_inward * inward[1] - y_out
    return Motion(
        blend,
        accel,
        feed,
        np.concatenate([x_in[:, :0:-1], x_out], axis=1),
        np.concatenate([y_in[:, :0:-1], y_out], axis=1),
        speed_out,
        q_out,
    )


@dataclass
class Shaped:
    """A motion through the X and Y kernels, each axis reading both
    coordinates so any orientation of the corner is a mix of the four."""

    x_of_x: np.ndarray
    x_of_y: np.ndarray
    y_of_x: np.ndarray
    y_of_y: np.ndarray

    @classmethod
    def of(cls, motion: Motion, kernels: tuple[Kernel, Kernel]) -> Shaped:
        kx, ky = kernels
        return cls(
            kx.shape(motion.x),
            kx.shape(motion.y),
            ky.shape(motion.x),
            ky.shape(motion.y),
        )

    def at(self, orientation: float) -> tuple[np.ndarray, np.ndarray]:
        sin, cos = math.sin(orientation), math.cos(orientation)
        return (
            cos * self.x_of_x - sin * self.x_of_y,
            sin * self.y_of_x + cos * self.y_of_y,
        )


def orientations() -> list[float]:
    return [math.pi * index / ORIENTATIONS for index in range(ORIENTATIONS)]


def miss_mm(
    shaped: Shaped, window: slice, angles: list[float] | None = None
) -> np.ndarray:
    worst = np.zeros(shaped.x_of_x.shape[0])
    for orientation in angles or orientations():
        x, y = shaped.at(orientation)
        closest = np.hypot(x[:, window], y[:, window]).min(axis=1)
        assert not np.any(np.isnan(closest)), "the search window runs out"
        worst = np.maximum(worst, closest)
    return worst


def crossing(ratio: np.ndarray, rising: np.ndarray, budget: float) -> float:
    limit = budget + FEASIBLE_SLACK_MM
    if rising[0] > limit:
        return math.nan
    over = np.nonzero(rising > limit)[0]
    if len(over) == 0:
        return float(ratio[-1])
    hi = int(over[0])
    rise = rising[hi] - rising[hi - 1]
    share = min(max((budget - rising[hi - 1]) / rise, 0.0), 1.0)
    return float(ratio[hi - 1] + share * (ratio[hi] - ratio[hi - 1]))


class Corner:
    """One turn on one machine profile."""

    def __init__(self, profile: Profile, theta: float):
        self.profile = profile
        self.theta = theta
        self.kernels = profile.kernels()
        self.sigma_s = max(kernel.sigma_s for kernel in self.kernels)
        self.reach_s = max(kernel.reach_s for kernel in self.kernels)
        self.search_s = SEARCH_SIGMAS * self.sigma_s
        self.floor_mm = self.sharp_miss(theta, 0.0)
        self.budget_mm = self.budget()

    def sharp_miss(self, theta: float, speed: float) -> float:
        blend = Blend(Shape.SHARP, theta, np.zeros(1))
        motion = self.run(blend, np.array([speed]), math.inf, 0.0)
        return float(self.raw_miss(motion)[0])

    def budget(self) -> float:
        profile = self.profile
        floor = PREDICTION_MARGIN * self.floor_mm
        if profile.budget is BudgetKind.SHAPED:
            return profile.budget_value
        if profile.budget is BudgetKind.OVER_FLOOR:
            return floor + profile.budget_value
        return floor + self.scv_allowance_mm(profile.budget_value)

    def scv_allowance_mm(self, scv: float) -> float:
        square = math.pi / 2.0
        added = self.sharp_miss(square, scv) - self.sharp_miss(square, 0.0)
        return PREDICTION_MARGIN * max(added, 0.0)

    def cut_room_mm(self) -> float:
        profile = self.profile
        if profile.budget is BudgetKind.SCV:
            return self.scv_allowance_mm(profile.budget_value)
        return profile.budget_value

    def steps(self, blend: Blend, tail_s: float) -> int:
        length = float(blend.length.max())
        stop_s = math.sqrt(2.0 * length / self.profile.accel)
        window_s = (
            self.reach_s
            + self.search_s
            + BLEND_WINDOW_FACTOR * stop_s
            + tail_s
            + WINDOW_MARGIN_S
        )
        return math.ceil(window_s / DT_S)

    def run(
        self, blend: Blend, apex_speed: np.ndarray, feed: float, tail_s: float
    ) -> Motion:
        steps = self.steps(blend, tail_s)
        return move(blend, apex_speed, self.profile.accel, feed, steps)

    def moved(
        self,
        shape: Shape,
        cut: np.ndarray,
        apex_speed: np.ndarray,
        feed: float | None = None,
        tail_s: float = 0.0,
    ) -> Motion:
        blend = Blend(shape, self.theta, cut)
        feed = self.profile.feed if feed is None else feed
        return self.run(blend, apex_speed, feed, tail_s)

    def search_window(self, motion: Motion) -> slice:
        span = math.ceil(self.search_s / DT_S)
        blend = math.ceil(float(motion.exit_s().max()) / DT_S)
        apex = motion.apex_index
        return slice(apex - span - blend, apex + span + blend + 1)

    def raw_miss(
        self, motion: Motion, angles: list[float] | None = None
    ) -> np.ndarray:
        shaped = Shaped.of(motion, self.kernels)
        return miss_mm(shaped, self.search_window(motion), angles)

    def raw_miss_of(
        self, shape: Shape, cut: np.ndarray, apex_speed: np.ndarray
    ) -> np.ndarray:
        return self.raw_miss(self.moved(shape, cut, apex_speed, feed=math.inf))

    def predicted_miss(self, cut: np.ndarray, raw: np.ndarray) -> np.ndarray:
        return cut + PREDICTION_MARGIN * np.clip(raw - cut, 0.0, None)

    def ratios_within_budget(
        self, shape: Shape, cuts: np.ndarray, nodes: int
    ) -> np.ndarray:
        """Per cut, the largest share of its curvature ceiling the apex
        may be crossed at within the budget; NaN where a stop there misses
        by more."""
        ratio = np.linspace(0.0, 1.0, nodes)
        cut_grid = np.repeat(cuts, nodes)
        ratio_grid = np.tile(ratio, len(cuts))
        cap = Blend(shape, self.theta, cut_grid).speed_cap(self.profile.accel)
        raw = self.raw_miss_of(shape, cut_grid, ratio_grid * cap)
        curve = self.predicted_miss(cut_grid, raw).reshape(len(cuts), nodes)
        curve = np.maximum.accumulate(curve, axis=1)
        return np.array([crossing(ratio, row, self.budget_mm) for row in curve])

    def planner_clothoid(self) -> tuple[float, float]:
        """Today's choice: the cut whose apex is crossed fastest, or the
        whole budget where no kernel shapes the path."""
        room = self.cut_room_mm()
        accel = self.profile.accel
        if not self.profile.shaped:
            cap = Blend(Shape.CLOTHOID, self.theta, np.array([room]))
            return room, float(cap.speed_cap(accel)[0])
        cuts = room * 2.0 ** (-0.5 * np.arange(1, PLANNER_CUT_CANDIDATES + 1))
        ratios = self.ratios_within_budget(
            Shape.CLOTHOID, cuts, PLANNER_RATIO_NODES
        )
        cap = Blend(Shape.CLOTHOID, self.theta, cuts).speed_cap(accel)
        speeds = np.where(np.isnan(ratios), -1.0, ratios * cap)
        best = int(speeds.argmax())
        if speeds[best] < 0.0:
            raise ValueError("no clothoid cut fits the budget")
        return float(cuts[best]), float(speeds[best])

    def cheapest(self, shape: Shape) -> tuple[float, float]:
        """The cut and apex speed costing least time within the budget."""
        room = self.cut_room_mm()
        steps = np.arange(SEARCH_CUT_OCTAVES * SEARCH_CUTS_PER_OCTAVE + 1)
        cuts = room * 2.0 ** (-steps / SEARCH_CUTS_PER_OCTAVE)
        ratios = self.ratios_within_budget(shape, cuts, SEARCH_RATIO_NODES)
        fits = ~np.isnan(ratios)
        if not fits.any():
            raise ValueError(f"no {shape.value} cut fits the budget")
        cuts, ratios = cuts[fits], ratios[fits]
        cap = Blend(shape, self.theta, cuts).speed_cap(self.profile.accel)
        speeds = ratios * cap
        motion = self.moved(shape, cuts, speeds)
        predicted = self.predicted_miss(cuts, self.raw_miss(motion))
        cost = motion.half_cost_s()
        cost[predicted > self.budget_mm + FEASIBLE_SLACK_MM] = np.inf
        best = int(cost.argmin())
        return float(cuts[best]), float(speeds[best])

    def fastest_sharp(self) -> float:
        speeds = np.linspace(0.0, self.profile.feed, SHARP_SPEED_NODES)
        raw = self.raw_miss_of(Shape.SHARP, np.zeros_like(speeds), speeds)
        curve = np.maximum.accumulate(PREDICTION_MARGIN * raw)
        share = crossing(
            np.linspace(0.0, 1.0, len(speeds)), curve, self.budget_mm
        )
        if math.isnan(share):
            raise ValueError("the floor is past the budget")
        return share * self.profile.feed


def mode_filter(frequency_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """The mode's deflection under a commanded acceleration."""
    omega = math.tau * frequency_hz
    numerator = [-1.0]
    denominator = [1.0, 2.0 * MODE_DAMPING_RATIO * omega, omega * omega]
    b, a, _ = signal.cont2discrete((numerator, denominator), DT_S, "bilinear")
    return b.ravel(), a


def swing_mm(accel: np.ndarray, frequency_hz: float, clear: int) -> float:
    """How far a mode that rode a steady acceleration into `accel` still
    swings at sample `clear`, from which on the acceleration stays put."""
    b, a = mode_filter(frequency_hz)
    settled = signal.lfilter_zi(b, a) * accel[0]
    deflection, _ = signal.lfilter(b, a, accel, zi=settled)
    omega = math.tau * frequency_hz
    damped = omega * math.sqrt(1.0 - MODE_DAMPING_RATIO**2)
    offset = deflection[clear] + accel[clear] / (omega * omega)
    rate = (deflection[clear + 1] - deflection[clear - 1]) / (2.0 * DT_S)
    return math.hypot(
        offset, (rate + MODE_DAMPING_RATIO * omega * offset) / damped
    )


@dataclass
class Row:
    label: str
    cut_mm: float
    apex_speed: float
    raw_miss_mm: float
    predicted_miss_mm: float
    cost_ms: float
    peak_accel_over_limit: float
    ring_um: list[float]


def measure(
    corner: Corner, label: str, shape: Shape, cut: float, speed: float
) -> Row:
    cuts, speeds = np.array([cut]), np.array([speed])
    timed = corner.moved(shape, cuts, speeds)
    raw = float(corner.raw_miss(timed)[0])
    cost_ms = 2e3 * float(timed.half_cost_s()[0])

    free = corner.moved(shape, cuts, speeds, feed=math.inf, tail_s=RING_TAIL_S)
    shaped = Shaped.of(free, corner.kernels)
    parts = [shaped.x_of_x, shaped.x_of_y, shaped.y_of_x, shaped.y_of_y]
    valid = np.nonzero(~np.isnan(np.sum(parts, axis=0)[0]))[0]
    first, last = int(valid[0]), int(valid[-1])
    accels = [
        np.diff(part[0, first : last + 1], n=2) / DT_S**2 for part in parts
    ]
    after_blend_s = float(free.exit_s()[0]) + corner.reach_s
    clear = free.apex_index + math.ceil(after_blend_s / DT_S) + 2 - first
    assert clear + 2 < len(accels[0]), "the ring window runs out"

    peak_accel = 0.0
    ring = np.zeros(len(MODE_FREQUENCY_FACTORS))
    for orientation in orientations():
        sin, cos = math.sin(orientation), math.cos(orientation)
        axes = (
            cos * accels[0] - sin * accels[1],
            sin * accels[2] + cos * accels[3],
        )
        peak_accel = max(peak_accel, float(np.hypot(*axes).max()))
        heading = orientation + corner.theta
        across = (abs(math.sin(heading)), abs(math.cos(heading)))
        for index, factor in enumerate(MODE_FREQUENCY_FACTORS):
            lateral = sum(
                share * swing_mm(axis, factor * kernel.frequency_hz, clear)
                for share, axis, kernel in zip(across, axes, corner.kernels)
            )
            ring[index] = max(ring[index], 1e3 * lateral)

    return Row(
        label,
        cut,
        speed,
        raw,
        float(corner.predicted_miss(cuts, np.array([raw]))[0]),
        cost_ms,
        peak_accel / corner.profile.accel,
        [float(value) for value in ring],
    )


def study(case: tuple[Profile, float]) -> dict:
    profile, theta_deg = case
    corner = Corner(profile, math.radians(theta_deg))
    result = {
        "profile": profile.label,
        "theta_deg": theta_deg,
        "floor_mm": corner.floor_mm,
        "budget_mm": corner.budget_mm,
        "feed": profile.feed,
        "accel": profile.accel,
        "rows": [],
    }
    if PREDICTION_MARGIN * corner.floor_mm > corner.budget_mm:
        result["skipped"] = (
            "the floor is past the budget: the planner slows the acceleration"
        )
        return result
    plans = [
        ("stop", Shape.SHARP, (0.0, 0.0)),
        ("clothoid (today)", Shape.CLOTHOID, corner.planner_clothoid()),
        ("clothoid", Shape.CLOTHOID, corner.cheapest(Shape.CLOTHOID)),
        ("arc", Shape.ARC, corner.cheapest(Shape.ARC)),
        ("parabola", Shape.PARABOLA, corner.cheapest(Shape.PARABOLA)),
    ]
    if profile.shaped:
        plans.append(
            ("sharp at speed", Shape.SHARP, (0.0, corner.fastest_sharp()))
        )
    for label, shape, (cut, speed) in plans:
        row = measure(corner, label, shape, cut, speed)
        result["rows"].append(asdict(row))
    return result


def table(result: dict) -> str:
    head = (
        f"\n{result['profile']}  turn {result['theta_deg']:g} deg  "
        f"F {result['feed']:g}  floor {result['floor_mm']:.4f} mm  "
        f"budget {result['budget_mm']:.4f} mm"
    )
    if "skipped" in result:
        return f"{head}\n  {result['skipped']}"
    modes = "".join(
        f" {f'{factor:g}f':>7}" for factor in MODE_FREQUENCY_FACTORS
    )
    lines = [
        head,
        f"  {'plan':<17} {'cut':>7} {'v_apex':>7} {'miss':>7} {'pred':>7}"
        f" {'cost_ms':>8} {'saved':>7} {'a/A':>6}  ring um:{modes}",
    ]
    today = result["rows"][1]["cost_ms"]
    for row in result["rows"]:
        ring = "".join(f" {value:7.2f}" for value in row["ring_um"])
        lines.append(
            f"  {row['label']:<17} {row['cut_mm']:7.4f}"
            f" {row['apex_speed']:7.2f} {row['raw_miss_mm']:7.4f}"
            f" {row['predicted_miss_mm']:7.4f} {row['cost_ms']:8.3f}"
            f" {today - row['cost_ms']:7.3f}"
            f" {row['peak_accel_over_limit']:6.2f}          {ring}"
        )
    return "\n".join(lines)


def engine_corner(profile: Profile, theta_deg: float) -> dict:
    """The same corner through the snapshot engine: its cost between two
    points either side of the vertex and the shaped path's miss."""
    import corner_flow

    leg = 60.0
    probe = 30.0
    theta = math.radians(theta_deg)
    start = (100.0, 100.0)
    vertex = (start[0] + leg, start[1])
    out = (math.cos(theta), math.sin(theta))
    end = (vertex[0] + leg * out[0], vertex[1] + leg * out[1])
    program = (
        "G21\nG90\nM83\n"
        f"G1 X{start[0]:g} Y{start[1]:g} F{profile.feed * 60:.0f}\n"
        f"G1 X{vertex[0]:.5f} Y{vertex[1]:.5f}\n"
        f"G1 X{end[0]:.5f} Y{end[1]:.5f}\n"
    )
    config = corner_flow.PrinterConfig.load(REPO / profile.config).text()
    engine = corner_flow.import_engine()
    waypoints = engine.parse_gcode(program, config)
    snapshot, _ = engine.pipeline_snapshot_axis_samples(
        waypoints, config, [0], 2
    )
    t_end = json.loads(snapshot)["trajectory"]["t_end"]
    count = int(t_end / DT_S) + 2
    _, rows = engine.pipeline_snapshot_axis_samples(
        waypoints, config, [0, 1], count
    )
    t = np.linspace(0.0, t_end, count)
    rows = np.array(rows)
    x = rows[0][:, 0] - vertex[0]
    y = rows[1][:, 0] - vertex[1]
    distance = np.hypot(x, y)
    closest = int(distance.argmin())
    before = np.interp(-probe, x[:closest], t[:closest])
    along_out = x[closest:] * out[0] + y[closest:] * out[1]
    after = np.interp(probe, along_out, t[closest:])
    return {
        "cost_ms": 1e3 * (after - before - 2.0 * probe / profile.feed),
        "shaped_miss_mm": float(distance[closest]),
    }


def validate(theta_degs: list[float]) -> None:
    print("engine vs the study's model of today's planner")
    print(
        f"  {'profile':<46} {'turn':>5} {'cost_ms engine':>15} {'model':>8}"
        f" {'miss engine':>12} {'model':>8}"
    )
    for profile in PROFILES:
        if profile.config is None:
            continue
        for theta_deg in theta_degs:
            corner = Corner(profile, math.radians(theta_deg))
            if PREDICTION_MARGIN * corner.floor_mm > corner.budget_mm:
                continue
            cut, speed = corner.planner_clothoid()
            motion = corner.moved(
                Shape.CLOTHOID, np.array([cut]), np.array([speed])
            )
            model_cost = 2e3 * float(motion.half_cost_s()[0])
            model_miss = float(corner.raw_miss(motion, [0.0])[0])
            engine = engine_corner(profile, theta_deg)
            print(
                f"  {profile.label:<46} {theta_deg:5g}"
                f" {engine['cost_ms']:15.3f} {model_cost:8.3f}"
                f" {engine['shaped_miss_mm']:12.4f} {model_miss:8.4f}",
                flush=True,
            )


def planned(profile: Profile, args: argparse.Namespace) -> Profile:
    if args.feed is not None:
        profile = replace(profile, feed=args.feed)
    if args.allowance is not None:
        profile = replace(
            profile, budget=BudgetKind.OVER_FLOOR, budget_value=args.allowance
        )
    return profile


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--turns", default="30,45,60,90,120,135,150")
    parser.add_argument("--profiles", default="", help="substring filter")
    parser.add_argument(
        "--feed", type=float, help="plan every profile at this feedrate"
    )
    parser.add_argument(
        "--allowance",
        type=float,
        help="plan every profile at its floor plus this many mm",
    )
    parser.add_argument("--engine", action="store_true")
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    turns = [float(value) for value in args.turns.split(",")]
    if args.engine:
        validate(turns)
        return
    cases = [
        (planned(profile, args), theta_deg)
        for profile in PROFILES
        if args.profiles in profile.label
        for theta_deg in turns
    ]
    with multiprocessing.Pool(min(WORKERS, len(cases))) as pool:
        results = []
        for result in pool.imap(study, cases):
            results.append(result)
            print(table(result), flush=True)
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
