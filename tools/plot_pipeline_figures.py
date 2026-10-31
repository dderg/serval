#!/usr/bin/env python3
"""Render the README pipeline figures from the real planner.

Drives `_motion_engine.pipeline_snapshot_axis_samples` (the same planner the
snapshot harness and the WASM playground use) over a small G-code square and
plots the stages into `docs/img/pipeline-*.svg`. The engine evaluates the exact
carriers the firmware executes and hands back per-axis samples: no polynomial
refit, no resampling into pieces, no carrier math re-derived here.

    make -f Makefile.rust motion-engine-fast
    python3 tools/plot_pipeline_figures.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

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
    if not hasattr(_motion_engine, "pipeline_snapshot_axis_samples"):
        raise SystemExit(
            "_motion_engine lacks pipeline_snapshot_axis_samples — rebuild "
            "with the `snapshot` feature: "
            "make -f Makefile.rust motion-engine-fast"
        )
    return _motion_engine


def waypoints():
    return [(x, y, 0.0, 0.0, 300.0, 5000.0) for x, y in SQUARE]


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
    n = 4000
    snap_json, (sx, sy) = engine.pipeline_snapshot_axis_samples(
        waypoints(), CONFIG, [0, 1], n
    )
    snap = json.loads(snap_json)
    t_end = snap["trajectory"]["t_end"]
    times = [t_end * i / (n - 1) for i in range(n)]
    px, vx, axx = (list(col) for col in zip(*sx))
    py, vy, ayy = (list(col) for col in zip(*sy))
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
    shaped_json, (ssx, ssy) = engine.pipeline_snapshot_axis_samples(
        waypoints(), CONFIG + SHAPED_CHAIN, [0, 1], n
    )
    st_end = json.loads(shaped_json)["trajectory"]["t_end"]
    stimes = [st_end * i / (n - 1) for i in range(n)]
    _, svx, saxx = (list(col) for col in zip(*ssx))
    _, svy, sayy = (list(col) for col in zip(*ssy))
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
