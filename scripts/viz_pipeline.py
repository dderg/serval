#!/usr/bin/env python3
"""Visualize motion at each pipeline stage: raw path, fitted path, velocity profile.

Runs on the printer host where _motion_engine.so and printer.cfg live.
"""

from __future__ import annotations

import argparse
import configparser
import math
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

PRINTER_DATA = Path.home() / "printer_data"
DEFAULT_CONFIG = PRINTER_DATA / "config" / "printer.cfg"
DEFAULT_GCODES = PRINTER_DATA / "gcodes"
DEFAULT_OUTPUT = PRINTER_DATA / "config" / "viz"

SEGMENT_COLORS = {
    "line": "C0",
    "arc": "C2",
    "clothoid": "C1",
}


def read_printer_limits(cfg_path: Path):
    cp = configparser.RawConfigParser()
    cp.read(cfg_path)
    section = "printer"
    return (
        cp.getfloat(section, "max_velocity"),
        cp.getfloat(section, "max_accel"),
        cp.getfloat(section, "square_corner_velocity", fallback=5.0),
    )


def _linearize_arc(x0, y0, z0, x1, y1, z1, i, j, ccw, feedrate):
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
    relative = False
    motion_cmd = re.compile(r"^G0?([0-3])\b", re.IGNORECASE)
    mode_cmd = re.compile(r"^G(90|91)\b", re.IGNORECASE)
    coord = re.compile(r"([XYZFIJ])([-+]?[0-9]*\.?[0-9]+)", re.IGNORECASE)

    for line in path.read_text().splitlines():
        line = line.split(";", 1)[0].strip()

        mm = mode_cmd.match(line)
        if mm:
            relative = mm.group(1) == "91"
            continue

        m = motion_cmd.match(line)
        if not m:
            continue
        if not waypoints:
            waypoints.append((x, y, z, max_velocity))
        cmd = int(m.group(1))
        params = {
            c.group(1).upper(): float(c.group(2)) for c in coord.finditer(line)
        }

        if relative:
            nx = x + params.get("X", 0.0)
            ny = y + params.get("Y", 0.0)
            nz = z + params.get("Z", 0.0)
        else:
            nx = params.get("X", x)
            ny = params.get("Y", y)
            nz = params.get("Z", z)

        if cmd == 0:
            x, y, z = nx, ny, nz
            waypoints.append((x, y, z, max_velocity))
        elif cmd == 1:
            x, y, z = nx, ny, nz
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            waypoints.append((x, y, z, feedrate))
        elif cmd in (2, 3):
            i_off = params.get("I", 0.0)
            j_off = params.get("J", 0.0)
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            arc_pts = _linearize_arc(
                x,
                y,
                z,
                nx,
                ny,
                nz,
                i_off,
                j_off,
                ccw=(cmd == 3),
                feedrate=feedrate,
            )
            waypoints.extend(arc_pts)
            x, y, z = nx, ny, nz

    return waypoints


def plot_raw_path(ax, raw_x, raw_y):
    ax.plot(raw_x, raw_y, "-", linewidth=0.5, color="C0")
    ax.plot(raw_x[0], raw_y[0], "o", color="C2", markersize=5, zorder=5)
    ax.set_aspect("equal")
    ax.set_title("Raw G-code path (before fitting)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")


def plot_fitted_path(ax, segments, raw_x, raw_y):
    ax.plot(
        raw_x,
        raw_y,
        "-",
        linewidth=0.3,
        color="C7",
        alpha=0.4,
        label="raw",
    )
    drawn = set()
    for seg in segments:
        kind = seg["type"]
        color = SEGMENT_COLORS.get(kind, "C4")
        label = kind if kind not in drawn else None
        drawn.add(kind)
        ax.plot(
            seg["x"], seg["y"], "-", linewidth=0.6, color=color, label=label
        )
    ax.plot(
        segments[0]["x"][0],
        segments[0]["y"][0],
        "o",
        color="C3",
        markersize=5,
        zorder=5,
    )
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
    parser.add_argument(
        "gcode",
        type=str,
        help="G-code filename (looked up in ~/printer_data/gcodes/) "
        "or full path",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"directory for output PNGs (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"printer.cfg path (default: {DEFAULT_CONFIG})",
    )
    args = parser.parse_args()

    if not args.config.exists():
        print(f"Config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    max_velocity, max_accel, scv = read_printer_limits(args.config)
    print(f"Config: v={max_velocity} mm/s, a={max_accel} mm/s², scv={scv} mm/s")

    gcode_path = Path(args.gcode)
    if not gcode_path.exists():
        gcode_path = DEFAULT_GCODES / args.gcode
    if not gcode_path.exists():
        print(f"File not found: {args.gcode}", file=sys.stderr)
        sys.exit(1)

    waypoints = parse_gcode(gcode_path, max_velocity)
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
        max_velocity,
        max_accel,
        scv,
    )

    from datetime import datetime

    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = run_dir / f"{gcode_path.stem}_{ts}"
    run_dir.mkdir(parents=True, exist_ok=True)
    stem = gcode_path.stem

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_raw_path(ax, snapshot["raw_x"], snapshot["raw_y"])
    fig.tight_layout()
    fig.savefig(run_dir / f"{stem}.01-raw-path.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_fitted_path(
        ax,
        list(snapshot["fitted_segments"]),
        snapshot["raw_x"],
        snapshot["raw_y"],
    )
    fig.tight_layout()
    fig.savefig(run_dir / f"{stem}.02-fitted-path.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    plot_velocity(
        ax,
        snapshot["vel_s"],
        snapshot["vel_v"],
        snapshot["traversal_time_s"],
    )
    fig.tight_layout()
    fig.savefig(run_dir / f"{stem}.03-velocity-profile.png", dpi=150)
    plt.close(fig)

    seg_counts = {}
    for seg in snapshot["fitted_segments"]:
        seg_counts[seg["type"]] = seg_counts.get(seg["type"], 0) + 1
    seg_summary = ", ".join(f"{v} {k}" for k, v in sorted(seg_counts.items()))

    print(
        f"Wrote 3 PNGs to {run_dir}/\n"
        f"  segments: {seg_summary}\n"
        f"  corners: {snapshot['blended_corners']} blended, "
        f"{snapshot['unblended_corners']} unblended, "
        f"{snapshot['chain_fits']} chain fits\n"
        f"  traversal: {snapshot['traversal_time_s']:.3f}s"
    )


if __name__ == "__main__":
    main()
