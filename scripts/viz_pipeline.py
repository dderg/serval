#!/usr/bin/env python3
"""Visualize motion at each pipeline stage: raw path, fitted path, velocity profile.

Runs on the printer host where _motion_engine.so and printer.cfg live.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from datetime import datetime
from pathlib import Path

KLIPPY_ENV = Path.home() / "klippy-env"


def _reexec_in_printer_env():
    # Run under the printer's virtualenv so viz shares the planner's exact
    # interpreter (cffi/chelper + klippy's config loader). Before importing
    # matplotlib, which the launching interpreter may not have. Compare
    # sys.prefix, not the executable: a venv's bin/python is a symlink to the
    # base interpreter, so resolving the path can't tell the two apart.
    venv_python = KLIPPY_ENV / "bin" / "python"
    in_venv = Path(sys.prefix).resolve() == KLIPPY_ENV.resolve()
    if venv_python.exists() and not in_venv:
        os.execv(
            str(venv_python),
            [str(venv_python), str(Path(__file__).resolve()), *sys.argv[1:]],
        )


_reexec_in_printer_env()

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PRINTER_DATA = Path.home() / "printer_data"
DEFAULT_CONFIG = PRINTER_DATA / "config" / "printer.cfg"
DEFAULT_GCODES = PRINTER_DATA / "gcodes"
DEFAULT_OUTPUT = PRINTER_DATA / "config" / "viz"

SEGMENT_COLORS = {
    "line": "C0",
    "arc": "C2",
    "clothoid": "C1",
}


def read_printer_config(cfg_path: Path):
    # Parse through klippy's own loader so includes resolve and the keys,
    # defaults, and the [arc_fit] knobs match the live printer exactly.
    from klippy import configfile
    from klippy.arc_fit_config import arc_fit_from_config

    loader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
    loader.printer = None
    config = loader.read_config(str(cfg_path))
    printer = config.getsection("printer")
    max_accel = printer.getfloat("max_accel", above=0.0)
    return (
        printer.getfloat("max_velocity", above=0.0),
        max_accel,
        printer.getfloat("square_corner_velocity", 5.0, minval=0.0),
        printer.getfloat("max_jerk", max_accel * 2.0, above=0.0),
        arc_fit_from_config(config),
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
        cmd = int(m.group(1))
        params = {
            c.group(1).upper(): float(c.group(2)) for c in coord.finditer(line)
        }
        has_position = any(axis in params for axis in ("X", "Y", "Z"))

        if relative:
            nx = x + params.get("X", 0.0)
            ny = y + params.get("Y", 0.0)
            nz = z + params.get("Z", 0.0)
        else:
            nx = params.get("X", x)
            ny = params.get("Y", y)
            nz = params.get("Z", z)

        if cmd == 0:
            if not has_position:
                continue
            x, y, z = nx, ny, nz
            waypoints.append((x, y, z, max_velocity))
        elif cmd == 1:
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            if not has_position:
                continue
            x, y, z = nx, ny, nz
            waypoints.append((x, y, z, feedrate))
        elif cmd in (2, 3):
            feedrate = params.get("F", feedrate * 60.0) / 60.0
            if not has_position:
                continue
            i_off = params.get("I", 0.0)
            j_off = params.get("J", 0.0)
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


def _build_time_series(snapshot):
    import numpy as np

    s = np.array(snapshot["kin_s"])
    v = np.array(snapshot["kin_v"])
    hx = np.array(snapshot["kin_heading_x"])
    hy = np.array(snapshot["kin_heading_y"])
    kappa = np.array(snapshot["kin_kappa"])

    mask = np.concatenate([[True], np.diff(s) > 1e-9])
    s, v, hx, hy, kappa = s[mask], v[mask], hx[mask], hy[mask], kappa[mask]

    v_safe = np.maximum(v, 1e-6)
    ds = np.diff(s)
    v_avg = 0.5 * (v_safe[:-1] + v_safe[1:])
    t = np.concatenate([[0.0], np.cumsum(ds / v_avg)])

    vx = v * hx
    vy = v * hy
    v_scalar = v

    dv_ds = np.gradient(v, s)
    a_tangential = v * dv_ds
    a_centripetal = v**2 * kappa
    nx, ny = -hy, hx
    a_x = a_tangential * hx + a_centripetal * nx
    a_y = a_tangential * hy + a_centripetal * ny
    a_scalar = np.sqrt(a_x**2 + a_y**2)

    jx = np.gradient(a_x, t)
    jy = np.gradient(a_y, t)
    j_scalar = np.sqrt(jx**2 + jy**2)

    return t, vx, vy, v_scalar, a_x, a_y, a_scalar, jx, jy, j_scalar


def _plot_derivative(ax, t, comp_x, comp_y, scalar, ylabel, title):
    import numpy as np

    ax.plot(t, np.abs(comp_x), "-", linewidth=0.6, color="C0", label="|X|")
    ax.plot(t, np.abs(comp_y), "-", linewidth=0.6, color="C1", label="|Y|")
    ax.plot(t, scalar, "-", linewidth=0.8, color="C3", label="scalar")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=7, loc="upper right")


def render(snapshot, out_path, stem, ts):
    raw_x, raw_y = snapshot["raw_x"], snapshot["raw_y"]
    segments = list(snapshot["fitted_segments"])

    t, vx, vy, v_sc, ax_d, ay, a_sc, jx, jy, j_sc = _build_time_series(
        snapshot,
    )

    fig, axes = plt.subplots(
        4,
        1,
        figsize=(10, 18),
        gridspec_kw={"height_ratios": [3, 1, 1, 1]},
    )
    ax_path, ax_vel, ax_acc, ax_jrk = axes

    from matplotlib.patches import Arc as ArcPatch

    ax_path.plot(
        raw_x,
        raw_y,
        "-",
        linewidth=0.8,
        color="#d0d0d0",
        zorder=1,
        label="raw",
    )
    drawn = set()
    first_pt = None
    for seg in segments:
        kind = seg["type"]
        color = SEGMENT_COLORS.get(kind, "C4")
        label = kind if kind not in drawn else None
        drawn.add(kind)
        if kind == "line":
            xs = [seg["x0"], seg["x1"]]
            ys = [seg["y0"], seg["y1"]]
            ax_path.plot(
                xs, ys, "-", linewidth=1.0, color=color, label=label, zorder=2
            )
            if first_pt is None:
                first_pt = (seg["x0"], seg["y0"])
        elif kind == "arc":
            ax_path.plot(
                seg["x"],
                seg["y"],
                "-",
                linewidth=1.0,
                color=color,
                label=label,
                zorder=2,
            )
            if first_pt is None:
                first_pt = (seg["x"][0], seg["y"][0])
        elif kind == "clothoid":
            ax_path.plot(
                seg["x"],
                seg["y"],
                "-",
                linewidth=1.0,
                color=color,
                label=label,
                zorder=2,
            )
            if first_pt is None:
                first_pt = (seg["x"][0], seg["y"][0])
    if first_pt:
        ax_path.plot(
            first_pt[0], first_pt[1], "o", color="C3", markersize=5, zorder=3
        )
    ax_path.set_aspect("equal")
    ax_path.set_xlabel("X (mm)")
    ax_path.set_ylabel("Y (mm)")
    ax_path.legend(fontsize=8, loc="upper right")

    seg_counts = {}
    for seg in segments:
        seg_counts[seg["type"]] = seg_counts.get(seg["type"], 0) + 1
    seg_summary = ", ".join(f"{v} {k}" for k, v in sorted(seg_counts.items()))
    ax_path.set_title(
        f"{stem}  [{seg_summary}]  "
        f"{snapshot['blended_corners']} blended, "
        f"{snapshot['chain_fits']} chains",
        fontsize=9,
    )

    _plot_derivative(
        ax_vel,
        t,
        vx,
        vy,
        v_sc,
        "mm/s",
        f"Velocity  (t={snapshot['traversal_time_s']:.3f}s)",
    )
    _plot_derivative(ax_acc, t, ax_d, ay, a_sc, "mm/s²", "Acceleration")
    _plot_derivative(ax_jrk, t, jx, jy, j_sc, "mm/s³", "Jerk")

    fig.tight_layout()
    out_file = out_path / f"{stem}_{ts}.png"
    fig.savefig(out_file, dpi=150)
    plt.close(fig)
    return out_file


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

    repo_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root / "klippy"))
    sys.path.insert(0, str(repo_root))

    max_velocity, max_accel, scv, max_jerk, arc_fit = read_printer_config(
        args.config
    )

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
        max_jerk,
        arc_fit=arc_fit,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_file = render(snapshot, args.output_dir, gcode_path.stem, ts)

    print(
        f"{out_file}\n"
        f"  v={max_velocity} a={max_accel} scv={scv}  "
        f"t={snapshot['traversal_time_s']:.3f}s"
    )


if __name__ == "__main__":
    main()
