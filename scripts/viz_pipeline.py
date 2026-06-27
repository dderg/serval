#!/usr/bin/env python3
"""Visualize motion at each pipeline stage: raw path, fitted path, velocity profile.

Runs on the printer host where _motion_engine.so and printer.cfg live.
"""

from __future__ import annotations

import argparse
import logging
import os
import re
import sys
from contextlib import contextmanager
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


PRINTER_DATA = Path.home() / "printer_data"
DEFAULT_CONFIG = PRINTER_DATA / "config" / "printer.cfg"
DEFAULT_GCODES = PRINTER_DATA / "gcodes"
DEFAULT_OUTPUT = PRINTER_DATA / "config" / "viz"

SEGMENT_COLORS = {
    "line": "C0",
    "arc": "C2",
    "clothoid": "C1",
}

MIN_PATH_VIEW_MM = 10.0


def _equal_scale_with_min_extent(ax, xs, ys, min_span=MIN_PATH_VIEW_MM):
    if xs and ys:
        center_x = 0.5 * (min(xs) + max(xs))
        center_y = 0.5 * (min(ys) + max(ys))
        half_x = max(0.5 * (max(xs) - min(xs)), 0.5 * min_span)
        half_y = max(0.5 * (max(ys) - min(ys)), 0.5 * min_span)
        ax.set_xlim(center_x - half_x, center_x + half_x)
        ax.set_ylim(center_y - half_y, center_y + half_y)
    ax.set_aspect("equal", adjustable="datalim")


@contextmanager
def _matplotlib_logging_silenced():
    log = logging.getLogger("matplotlib")
    previous_level = log.level
    log.setLevel(logging.ERROR)
    try:
        yield
    finally:
        log.setLevel(previous_level)


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


def parse_gcode(
    path: Path, max_velocity: float
) -> list[tuple[float, float, float, float]]:
    waypoints: list[tuple[float, float, float, float]] = []
    x, y, z = 0.0, 0.0, 0.0
    feedrate = max_velocity
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
            raise ValueError(
                f"G{cmd} arc command is not supported: the motion engine has no "
                "native arc ingestion yet, and silently linearizing it here would "
                "let a snapshot claim to exercise an arc while feeding the engine "
                "straight segments"
            )

    return waypoints


def _build_time_series(snapshot):
    import numpy as np

    # The visualizer consumes only the raw trajectory the planner produced --
    # where the toolhead is (position) and how fast it travels there (speed) --
    # and differentiates position itself. It trusts none of the planner's own
    # acceleration or curvature, so these curves are an independent check that
    # the planned motion respects the machine limits.
    x, y = _toolhead_position(snapshot)
    v = np.array(snapshot["kin_v"])

    distinct = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 1e-9])
    x, y, v = x[distinct], y[distinct], v[distinct]

    # Timing is the one thing position alone cannot give: convert the planner's
    # speed profile to a time axis (dt = ds / v), then every derivative below is
    # a numerical derivative of position with respect to that time.
    v_safe = np.maximum(v, 1e-6)
    ds = np.hypot(np.diff(x), np.diff(y))
    v_avg = 0.5 * (v_safe[:-1] + v_safe[1:])
    t = np.concatenate([[0.0], np.cumsum(ds / v_avg)])

    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    v_scalar = np.hypot(vx, vy)

    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    a_scalar = np.hypot(ax, ay)

    jx = np.gradient(ax, t)
    jy = np.gradient(ay, t)
    j_scalar = np.hypot(jx, jy)

    return t, vx, vy, v_scalar, ax, ay, a_scalar, jx, jy, j_scalar


def _toolhead_position(snapshot):
    import numpy as np

    if "kin_x" in snapshot:
        return np.array(snapshot["kin_x"]), np.array(snapshot["kin_y"])
    # Legacy baselines stored heading + arc length instead of position; integrate
    # the unit heading along s to recover the path so they still preview.
    s = np.array(snapshot["kin_s"])
    hx = np.array(snapshot["kin_heading_x"])
    hy = np.array(snapshot["kin_heading_y"])
    ds = np.diff(s, prepend=s[0])
    x = snapshot["raw_x"][0] + np.cumsum(hx * ds)
    y = snapshot["raw_y"][0] + np.cumsum(hy * ds)
    return x, y


def _plot_derivative(
    ax, t, comp_x, comp_y, scalar, ylabel, title, drawstyle="default"
):
    import numpy as np

    def plot(y, **kw):
        ax.plot(t, y, drawstyle=drawstyle, **kw)

    plot(np.abs(comp_x), linewidth=0.6, color="C0", label="|X|")
    plot(np.abs(comp_y), linewidth=0.6, color="C1", label="|Y|")
    plot(scalar, linewidth=0.8, color="C3", label="scalar")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=7, loc="upper right")


def render(snapshot, out_path, stem, ts):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

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
    _equal_scale_with_min_extent(ax_path, list(raw_x), list(raw_y))
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
    _plot_derivative(
        ax_jrk, t, jx, jy, j_sc, "mm/s³", "Jerk", drawstyle="steps-post"
    )

    out_file = out_path / f"{stem}_{ts}.png"
    with _matplotlib_logging_silenced():
        fig.tight_layout()
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
    _reexec_in_printer_env()
    main()
