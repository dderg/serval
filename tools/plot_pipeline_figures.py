#!/usr/bin/env python3
"""Render the README pipeline figures from the real planner.

Drives `_motion_engine.pipeline_snapshot` (the same entry point the snapshot
harness and the WASM playground use) over a small G-code square and plots the
stages into `docs/img/pipeline-*.svg`. The snapshot carries the exact carriers
the firmware executes — analytic move spans over line/arc/clothoid geometry and
B-spline curves — and this script evaluates them as they are: no polynomial
refit, no resampling into pieces.

    make -f Makefile.rust motion-engine-fast
    python3 tools/plot_pipeline_figures.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
from scipy.special import fresnel

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parent.parent
OUT_DIR = REPO / "docs" / "img"

CONFIG = """
[printer]
max_velocity: 300
max_accel: 5000
corner_deviation: 0.2
max_jerk: 0
"""

SHAPED_CHAIN = """
[post_processor is]
type: smooth_bell
smooth_time: 0.018

[axis x]
post_processors: is

[axis y]
post_processors: is
"""

SMOOTH_TIME = 0.018

SQUARE = [(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0), (0.0, 0.0)]

INK = "#1b1b1b"
FORK = "#e06000"
CLASSIC = "#7a7a7a"


def load_engine():
    sys.path.insert(0, str(REPO / "klippy"))
    try:
        import _motion_engine
    except ImportError as exc:
        raise SystemExit(
            "_motion_engine not importable — build it first:\n"
            "  make -f Makefile.rust motion-engine-fast"
        ) from exc
    if not hasattr(_motion_engine, "pipeline_snapshot"):
        raise SystemExit(
            "_motion_engine lacks pipeline_snapshot — rebuild with the "
            "`snapshot` feature: make -f Makefile.rust motion-engine-fast"
        )
    return _motion_engine


def waypoints():
    return [(x, y, 0.0, 0.0, 300.0, 5000.0) for x, y in SQUARE]


def unsupported(detail):
    raise SystemExit(f"plot_pipeline_figures: {detail}")


def clothoid_offset(kappa_0, sigma, s):
    """Exact (u, v) offset of a clothoid at arc length `s`.

    Mirrors `geometry::path::lowering::fresnel::clothoid_offset`: the heading
    is `kappa_0 * s + sigma * s^2 / 2`, so the offset is a pair of Fresnel
    integrals, degenerating to a circular arc and then a straight chord as
    `sigma` and `kappa_0` vanish.
    """
    if sigma == 0.0:
        if kappa_0 == 0.0:
            return s, 0.0
        return (
            math.sin(kappa_0 * s) / kappa_0,
            (1.0 - math.cos(kappa_0 * s)) / kappa_0,
        )
    abs_sigma = abs(sigma)
    sign = math.copysign(1.0, sigma)
    a = kappa_0 * kappa_0 / (2.0 * sigma)
    scale = math.sqrt(abs_sigma / math.pi)
    s0, c0 = fresnel(kappa_0 / sigma * scale)
    s1, c1 = fresnel((s + kappa_0 / sigma) * scale)
    d_c = c1 - c0
    d_s = s1 - s0
    k = math.sqrt(math.pi / abs_sigma)
    return (
        k * (math.cos(a) * d_c + sign * math.sin(a) * d_s),
        k * (sign * math.cos(a) * d_s - math.sin(a) * d_c),
    )


def axpby(a, u, b, v):
    return [a * u[i] + b * v[i] for i in range(3)]


def spatial_frame(spatial, s):
    """Position, unit heading and `dheading/ds` of a spatial segment at `s`."""
    kind = spatial["kind"]
    if kind == "line":
        start, end = spatial["start"], spatial["end"]
        delta = [end[i] - start[i] for i in range(3)]
        length = math.sqrt(sum(d * d for d in delta))
        frac = s / length
        return (
            [start[i] + frac * delta[i] for i in range(3)],
            [d / length for d in delta],
            [0.0, 0.0, 0.0],
        )
    if kind == "arc":
        u, v, radius = spatial["u"], spatial["v"], spatial["radius"]
        sign = math.copysign(1.0, spatial["sweep"])
        theta = spatial["start_angle"] + sign * s / radius
        origin = spatial["origin"]
        offset = axpby(radius * math.cos(theta), u, radius * math.sin(theta), v)
        return (
            [origin[i] + offset[i] for i in range(3)],
            axpby(-sign * math.sin(theta), u, sign * math.cos(theta), v),
            axpby(-math.cos(theta) / radius, u, -math.sin(theta) / radius, v),
        )
    if kind == "clothoid":
        u, v = spatial["u"], spatial["v"]
        kappa_0, sigma = spatial["kappa_0"], spatial["sigma"]
        phi = kappa_0 * s + 0.5 * sigma * s * s
        kappa = kappa_0 + sigma * s
        cu, cv = clothoid_offset(kappa_0, sigma, s)
        pose = spatial["start_pose"]
        offset = axpby(cu, u, cv, v)
        return (
            [pose[i] + offset[i] for i in range(3)],
            axpby(math.cos(phi), u, math.sin(phi), v),
            axpby(-kappa * math.sin(phi), u, kappa * math.cos(phi), v),
        )
    unsupported(f"spatial geometry '{kind}' has no evaluator here")


def active_phase(phases, local_t):
    for phase in phases:
        if phase["t0"] + phase["dt"] >= local_t:
            return phase
    return phases[-1]


class SplineCurve:
    """A scalar B-spline carrier, with its two derivative curves."""

    def __init__(self, curve):
        self.levels = [
            (curve["degree"], curve["knots"], curve["control_points"])
        ]
        for _ in range(2):
            self.levels.append(self._differentiate(*self.levels[-1]))

    @staticmethod
    def _differentiate(degree, knots, cps):
        if degree == 0:
            return 0, knots[1:-1], [0.0] * max(len(cps) - 1, 1)
        derived = []
        for i in range(len(cps) - 1):
            span = knots[i + degree + 1] - knots[i + 1]
            derived.append(
                0.0 if span == 0.0 else degree * (cps[i + 1] - cps[i]) / span
            )
        return degree - 1, knots[1:-1], derived

    def pva(self, t):
        return tuple(self._de_boor(level, t) for level in self.levels)

    @staticmethod
    def _de_boor(level, t):
        degree, knots, cps = level
        if not cps:
            return 0.0
        t = min(max(t, knots[degree]), knots[len(cps)])
        k = degree
        while k + 1 < len(cps) and knots[k + 1] <= t:
            k += 1
        d = [cps[k - degree + j] for j in range(degree + 1)]
        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                left = knots[k - degree + j]
                right = knots[k + 1 + j - r]
                alpha = 0.0 if right == left else (t - left) / (right - left)
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
        return d[degree]


class ExactTrajectory:
    """The snapshot's exact per-axis carriers, evaluated in their own form."""

    def __init__(self, trajectory):
        self.spans = trajectory["spans"]
        self.curves = [SplineCurve(c) for c in trajectory["curves"]]
        self.axes = trajectory["axes"]
        self.t_end = trajectory["t_end"]

    def sample(self, axis, times):
        """Position, velocity and acceleration of one axis at each time."""
        rows = self.axes[axis]
        if not rows:
            unsupported(f"axis {axis} carries no rows")
        pos, vel, acc = [], [], []
        idx = 0
        for t in times:
            while idx + 1 < len(rows) and rows[idx + 1]["t0"] <= t:
                idx += 1
            row = rows[idx]
            p, v, a = self.eval_carrier(
                row["carrier"], axis, min(max(t, row["t0"]), row["t1"])
            )
            pos.append(p)
            vel.append(v)
            acc.append(a)
        return pos, vel, acc

    def eval_carrier(self, carrier, axis, t):
        kind = carrier["kind"]
        if kind == "hold":
            return carrier["position"], 0.0, 0.0
        if kind == "spline":
            return self.curves[carrier["curve"]].pva(t)
        if kind == "relative_spline":
            p, v, a = self.curves[carrier["curve"]].pva(t)
            return p + carrier["base_position"], v, a
        if kind == "piecewise_relative_spline":
            piece = self.owning_piece(carrier["pieces"], t)
            p, v, a = self.curves[piece["curve"]].pva(t)
            return p + piece["base_position"], v, a
        if kind == "analytic":
            return self.eval_analytic(
                self.spans[carrier["span"]], carrier["axis"], t
            )
        unsupported(f"carrier kind '{kind}' has no evaluator here")

    @staticmethod
    def owning_piece(pieces, t):
        for piece in pieces:
            if piece["t_start"] <= t <= piece["t_end"]:
                return piece
        return pieces[-1]

    @staticmethod
    def eval_analytic(span, axis, t):
        if axis >= 3:
            unsupported(
                f"analytic follower axis {axis} is not plotted; only the "
                "spatial axes are"
            )
        spatial = span["spatial"]
        if spatial is None:
            unsupported("analytic span without spatial geometry is not plotted")
        local_t = min(
            max(t - span["t_start"], 0.0), span["t_end"] - span["t_start"]
        )
        phase = active_phase(span["phases"], local_t)
        tau = min(max(local_t - phase["t0"], 0.0), phase["dt"])
        distance = (
            phase["s0"]
            + phase["v0"] * tau
            + 0.5 * phase["a0"] * tau * tau
            + phase["j"] * tau * tau * tau / 6.0
            - span["source_distance_origin"]
        )
        velocity = (
            phase["v0"] + phase["a0"] * tau + 0.5 * phase["j"] * tau * tau
        )
        accel = phase["a0"] + phase["j"] * tau
        point, heading, dheading = spatial_frame(spatial, distance)
        position = point[axis]
        if axis == 2 and span["surface_z_offset"] is not None:
            position += span["surface_z_offset"]
        return (
            position,
            velocity * heading[axis],
            accel * heading[axis] + velocity * velocity * dheading[axis],
        )


def curvature(vx, vy, ax, ay):
    out = []
    for x1, y1, x2, y2 in zip(vx, vy, ax, ay):
        speed2 = x1 * x1 + y1 * y1
        if speed2 < 1e-9:
            out.append(float("nan"))
            continue
        out.append(abs(x1 * y2 - y1 * x2) / speed2**1.5)
    return out


def arclength(px, py):
    s = [0.0]
    for i in range(1, len(px)):
        s.append(
            s[-1] + ((px[i] - px[i - 1]) ** 2 + (py[i] - py[i - 1]) ** 2) ** 0.5
        )
    return s


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.set_xlabel(xlabel, fontsize=8)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25, linewidth=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def save(fig, name):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / name
    fig.savefig(path, bbox_inches="tight", transparent=False)
    plt.close(fig)
    print(f"wrote {path.relative_to(REPO)}")


def main():
    engine = load_engine()
    snap = engine.pipeline_snapshot(waypoints(), CONFIG)

    traj = ExactTrajectory(snap["trajectory"])
    t_end = traj.t_end
    n = 4000
    times = [t_end * i / (n - 1) for i in range(n)]
    px, vx, axx = traj.sample(0, times)
    py, vy, ayy = traj.sample(1, times)
    speed = [(a * a + b * b) ** 0.5 for a, b in zip(vx, vy)]
    kappa = curvature(vx, vy, axx, ayy)
    s = arclength(px, py)

    # 1 — path: G-code polyline vs the fitted, corner-blended path
    fig, ax = plt.subplots(figsize=(4.6, 4.6))
    ax.plot(
        snap["raw_x"],
        snap["raw_y"],
        color=CLASSIC,
        linewidth=1.0,
        linestyle="--",
        label="G-code polyline (sharp corners)",
    )
    ax.plot(
        px, py, color=FORK, linewidth=1.6, label="fitted path (arc + clothoid)"
    )
    ax.set_aspect("equal")
    style(ax, "1 — fitter: corners become blends", "X (mm)", "Y (mm)")
    ax.legend(
        fontsize=7,
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
    )

    zoom = ax.inset_axes([0.55, 0.55, 0.42, 0.42])
    zoom.plot(
        snap["raw_x"],
        snap["raw_y"],
        color=CLASSIC,
        linewidth=1.0,
        linestyle="--",
    )
    zoom.plot(px, py, color=FORK, linewidth=1.6)
    zoom.set_xlim(38.0, 41.0)
    zoom.set_ylim(-1.0, 2.0)
    zoom.set_aspect("equal")
    zoom.set_title("corner, 3 mm across", fontsize=7, color=INK)
    zoom.tick_params(labelsize=6)
    ax.indicate_inset_zoom(zoom, edgecolor=INK, linewidth=0.6)
    save(fig, "pipeline-path.svg")

    # 2 — curvature through one corner
    corner_s = [
        s[
            min(
                range(len(px)),
                key=lambda i: (px[i] - cx) ** 2 + (py[i] - cy) ** 2,
            )
        ]
        for cx, cy in SQUARE[1:-1]
    ]
    cs = corner_s[0]
    window = [i for i in range(len(s)) if abs(s[i] - cs) <= 3.0]
    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    ax.axvline(
        0.0,
        color=CLASSIC,
        linewidth=1.2,
        linestyle="--",
        label="sharp corner: κ jumps to infinity there, zero either side",
    )
    ax.plot(
        [s[i] - cs for i in window],
        [kappa[i] for i in window],
        color=FORK,
        linewidth=1.6,
        label="fitted path κ(s) — continuous, ramped by the clothoids (G2)",
    )
    ax.set_xlim(-3.0, 3.0)
    style(
        ax,
        "2 — curvature through one corner: continuous, not an impulse",
        "path length either side of the corner (mm)",
        "κ (1/mm)",
    )
    ax.legend(fontsize=7, frameon=False)
    save(fig, "pipeline-curvature.svg")

    # 3 — planned speed along the path
    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    ax.plot(times, speed, color=FORK, linewidth=1.4, label="|v| (mm/s)")
    ax.axhline(
        300.0,
        color=CLASSIC,
        linewidth=1.0,
        linestyle="--",
        label="max_velocity",
    )
    style(
        ax,
        "3 — planner: velocity profile riding the limits",
        "time (s)",
        "speed (mm/s)",
    )
    ax.legend(fontsize=7, frameon=False)
    save(fig, "pipeline-velocity.svg")

    # 4 — per-axis tracks the MCUs receive, before and after the shaper chain
    shaped = engine.pipeline_snapshot(waypoints(), CONFIG + SHAPED_CHAIN)
    shaped_traj = ExactTrajectory(shaped["trajectory"])
    st_end = shaped_traj.t_end
    stimes = [st_end * i / (n - 1) for i in range(n)]
    _, svx, saxx = shaped_traj.sample(0, stimes)
    _, svy, sayy = shaped_traj.sample(1, stimes)
    splot = [t - SMOOTH_TIME / 2.0 for t in stimes]

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 4.4), sharex=True)
    axes[0].plot(
        times,
        vx,
        color=CLASSIC,
        linewidth=1.0,
        linestyle="--",
        label="X nominal",
    )
    axes[0].plot(
        times,
        vy,
        color=CLASSIC,
        linewidth=1.0,
        linestyle=":",
        label="Y nominal",
    )
    axes[0].plot(splot, svx, color=FORK, linewidth=1.3, label="X shaped")
    axes[0].plot(splot, svy, color=INK, linewidth=1.3, label="Y shaped")
    style(
        axes[0],
        "4 — lowerer + shaper: per-axis tracks (smooth_bell, 18 ms)",
        "",
        "velocity (mm/s)",
    )
    axes[0].legend(fontsize=7, frameon=False, ncol=4)
    axes[1].plot(
        times,
        axx,
        color=CLASSIC,
        linewidth=1.0,
        linestyle="--",
        label="X nominal",
    )
    axes[1].plot(splot, saxx, color=FORK, linewidth=1.3, label="X shaped")
    axes[1].plot(splot, sayy, color=INK, linewidth=1.3, label="Y shaped")
    style(axes[1], "", "time (s)", "acceleration (mm/s²)")
    axes[1].legend(fontsize=7, frameon=False, ncol=3)
    save(fig, "pipeline-axes.svg")


if __name__ == "__main__":
    main()
