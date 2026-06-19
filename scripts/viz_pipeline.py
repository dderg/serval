#!/usr/bin/env python3
"""Visualize motion at each pipeline stage: raw path, fitted path, velocity profile."""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _linearize_arc(x0, y0, z0, x1, y1, z1, i, j, ccw, feedrate, n_segments=32):
    cx, cy = x0 + i, y0 + j
    r = math.hypot(i, j)
    if r < 1e-9:
        return [(x1, y1, z1, feedrate)]

    a_start = math.atan2(y0 - cy, x0 - cx)
    a_end = math.atan2(y1 - cy, x1 - cx)

    if ccw:
        if a_end <= a_start:
            a_end += 2 * math.pi
    else:
        if a_end >= a_start:
            a_end -= 2 * math.pi

    sweep = a_end - a_start
    arc_len = abs(sweep) * r
    n = max(int(arc_len / 0.5), 4)

    points = []
    z_step = (z1 - z0) / n
    for k in range(1, n + 1):
        t = k / n
        angle = a_start + sweep * t
        px = cx + r * math.cos(angle)
        py = cy + r * math.sin(angle)
        pz = z0 + z_step * k
        points.append((px, py, pz, feedrate))
    return points


def parse_gcode(
    path: Path, max_velocity: float
) -> list[tuple[float, float, float, float]]:
    waypoints: list[tuple[float, float, float, float]] = []
    x, y, z = 0.0, 0.0, 0.0
    feedrate = 100.0
    motion_cmd = re.compile(r"^G0?([0-3])\b", re.IGNORECASE)
    coord = re.compile(r"([XYZFIJ])([-+]?[0-9]*\.?[0-9]+)", re.IGNORECASE)

    for line in path.read_text().splitlines():
        line = line.split(";", 1)[0].strip()
        m = motion_cmd.match(line)
        if not m:
            continue
        cmd = int(m.group(1))
        params = {
            c.group(1).upper(): float(c.group(2)) for c in coord.finditer(line)
        }

        if cmd == 0:
            x = params.get("X", x)
            y = params.get("Y", y)
            z = params.get("Z", z)
            waypoints.append((x, y, z, max_velocity))
        elif cmd == 1:
            x = params.get("X", x)
            y = params.get("Y", y)
            z = params.get("Z", z)
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            waypoints.append((x, y, z, feedrate))
        elif cmd in (2, 3):
            x1 = params.get("X", x)
            y1 = params.get("Y", y)
            z1 = params.get("Z", z)
            i_off = params.get("I", 0.0)
            j_off = params.get("J", 0.0)
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            arc_pts = _linearize_arc(
                x,
                y,
                z,
                x1,
                y1,
                z1,
                i_off,
                j_off,
                ccw=(cmd == 3),
                feedrate=feedrate,
            )
            waypoints.extend(arc_pts)
            x, y, z = x1, y1, z1

    return waypoints


def plot_raw_path(ax, raw_x, raw_y):
    ax.plot(raw_x, raw_y, "-", linewidth=0.5, color="C0")
    ax.plot(raw_x[0], raw_y[0], "o", color="C2", markersize=5, zorder=5)
    ax.set_aspect("equal")
    ax.set_title("Raw G-code path (before fitting)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")


def plot_fitted_path(ax, fitted_x, fitted_y, raw_x, raw_y):
    ax.plot(
        raw_x, raw_y, "-", linewidth=0.3, color="C7", alpha=0.5, label="raw"
    )
    ax.plot(fitted_x, fitted_y, "-", linewidth=0.5, color="C1", label="fitted")
    ax.plot(fitted_x[0], fitted_y[0], "o", color="C2", markersize=5, zorder=5)
    ax.set_aspect("equal")
    ax.set_title("Fitted path (after corner blending)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.legend(fontsize=8)


def plot_velocity(ax, vel_s, vel_v, traversal_time):
    ax.plot(vel_s, vel_v, "-", linewidth=0.6, color="C3")
    ax.set_title(f"Velocity profile (t={traversal_time:.3f}s)")
    ax.set_xlabel("Arc-length s (mm)")
    ax.set_ylabel("Velocity (mm/s)")
    ax.set_ylim(bottom=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gcode", type=Path, help="G-code file to visualize")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("."),
        help="directory for output PNGs (default: cwd)",
    )
    parser.add_argument("--max-velocity", type=float, default=300.0)
    parser.add_argument("--max-accel", type=float, default=3000.0)
    parser.add_argument("--square-corner-velocity", type=float, default=5.0)
    args = parser.parse_args()

    if not args.gcode.exists():
        print(f"File not found: {args.gcode}", file=sys.stderr)
        sys.exit(1)

    waypoints = parse_gcode(args.gcode, args.max_velocity)
    if len(waypoints) < 2:
        print("No spatial moves found in G-code.", file=sys.stderr)
        sys.exit(1)

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "klippy"))
    try:
        import _motion_engine
    except ModuleNotFoundError:
        sys.exit(
            "_motion_engine.so not found — build with: "
            "make -f Makefile.rust motion-engine"
        )

    snapshot = _motion_engine.pipeline_snapshot(
        waypoints,
        args.max_velocity,
        args.max_accel,
        args.square_corner_velocity,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.gcode.stem

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_raw_path(ax, snapshot["raw_x"], snapshot["raw_y"])
    fig.tight_layout()
    fig.savefig(args.output_dir / f"{stem}.01-raw-path.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_fitted_path(
        ax,
        snapshot["fitted_x"],
        snapshot["fitted_y"],
        snapshot["raw_x"],
        snapshot["raw_y"],
    )
    fig.tight_layout()
    fig.savefig(args.output_dir / f"{stem}.02-fitted-path.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    plot_velocity(
        ax, snapshot["vel_s"], snapshot["vel_v"], snapshot["traversal_time_s"]
    )
    fig.tight_layout()
    fig.savefig(args.output_dir / f"{stem}.03-velocity-profile.png", dpi=150)
    plt.close(fig)

    print(
        f"Wrote 3 PNGs to {args.output_dir}/\n"
        f"  corners: {snapshot['blended_corners']} blended, "
        f"{snapshot['unblended_corners']} unblended, "
        f"{snapshot['chain_fits']} chain fits\n"
        f"  traversal: {snapshot['traversal_time_s']:.3f}s"
    )


if __name__ == "__main__":
    main()
