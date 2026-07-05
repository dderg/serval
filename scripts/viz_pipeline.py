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
    if config.has_section("extruder"):
        extruder = config.getsection("extruder")
        extrude_only_velocity = extruder.getfloat(
            "max_extrude_only_velocity", None, above=0.0
        )
        extrude_only_accel = extruder.getfloat(
            "max_extrude_only_accel", None, above=0.0
        )
    else:
        extrude_only_velocity = extrude_only_accel = None
    max_jerk = printer.getfloat("max_jerk", max_accel * 2.0, minval=0.0)
    return (
        printer.getfloat("max_velocity", above=0.0),
        max_accel,
        printer.getfloat("square_corner_velocity", 5.0, minval=0.0),
        max_jerk if max_jerk > 0.0 else float("inf"),
        arc_fit_from_config(config),
        extrude_only_velocity,
        extrude_only_accel,
        printer.getfloat("max_path_deviation", 0.005, above=0.0, maxval=1.0),
        printer.getfloat("max_accel_deviation", 50.0, above=0.0),
    )


def parse_gcode(
    path: Path, max_velocity: float
) -> list[tuple[float, float, float, float, float]]:
    # Waypoints carry absolute (x, y, z, e, feedrate). E rides as a fifth
    # coordinate so retracts (E-only moves) and extruding moves flow through the
    # pipeline as followers; the engine differences consecutive E to a per-move
    # delta. Extruder mode is M82 (absolute) / M83 (relative), independent of the
    # G90/G91 flag that governs X/Y/Z; under G91 an undeclared extruder rides
    # along as relative, and an E word with no mode declared at all is refused
    # rather than guessed. G92 resets any axis's position (commonly `G92 E0`)
    # without emitting a move.
    waypoints: list[tuple[float, float, float, float, float]] = []
    x, y, z, e = 0.0, 0.0, 0.0, 0.0
    feedrate = max_velocity
    relative = False
    e_relative: bool | None = None
    motion_cmd = re.compile(r"^G0?([0-3])\b", re.IGNORECASE)
    mode_cmd = re.compile(r"^G(90|91)\b", re.IGNORECASE)
    set_pos_cmd = re.compile(r"^G92\b", re.IGNORECASE)
    e_mode_cmd = re.compile(r"^M(82|83)\b", re.IGNORECASE)
    coord = re.compile(r"([XYZEFIJ])([-+]?[0-9]*\.?[0-9]+)", re.IGNORECASE)

    def params_of(line: str) -> dict[str, float]:
        return {
            c.group(1).upper(): float(c.group(2)) for c in coord.finditer(line)
        }

    for line in path.read_text().splitlines():
        line = line.split(";", 1)[0].strip()

        mm = mode_cmd.match(line)
        if mm:
            relative = mm.group(1) == "91"
            continue

        em = e_mode_cmd.match(line)
        if em:
            e_relative = em.group(1) == "83"
            continue

        if set_pos_cmd.match(line):
            params = params_of(line)
            x = params.get("X", x)
            y = params.get("Y", y)
            z = params.get("Z", z)
            e = params.get("E", e)
            continue

        m = motion_cmd.match(line)
        if not m:
            continue
        cmd = int(m.group(1))
        params = params_of(line)
        has_position = any(axis in params for axis in ("X", "Y", "Z"))
        has_extrusion = "E" in params

        if relative:
            nx = x + params.get("X", 0.0)
            ny = y + params.get("Y", 0.0)
            nz = z + params.get("Z", 0.0)
        else:
            nx = params.get("X", x)
            ny = params.get("Y", y)
            nz = params.get("Z", z)

        if has_extrusion:
            if e_relative is None and not relative:
                raise ValueError(
                    f"{path.name}: E word before any M82/M83 (or G91) — the "
                    "extruder mode is ambiguous, and guessing absolute turns "
                    "relative-E slicer output into garbage extrusion ratios. "
                    "Declare the mode (slicer excerpts printed with relative "
                    "extrusion need an 'M83' line at the top)."
                )
            e_is_relative = True if e_relative is None else e_relative
            ne = e + params["E"] if e_is_relative else params["E"]
        else:
            ne = e

        if cmd in (2, 3):
            raise ValueError(
                f"G{cmd} arc command is not supported: the motion engine has no "
                "native arc ingestion yet, and silently linearizing it here would "
                "let a snapshot claim to exercise an arc while feeding the engine "
                "straight segments"
            )
        if cmd == 1:
            feedrate = params.get("F", feedrate * 60.0) / 60.0
        if not (has_position or ne != e):
            continue
        x, y, z, e = nx, ny, nz, ne
        if not waypoints and not has_position:
            # A prime or retract before any positional command: there is no
            # known toolhead position yet, so anchoring a waypoint would invent
            # a move from the parser's arbitrary origin. Fold the E change into
            # the state; the first positional waypoint carries it.
            continue
        move_feedrate = max_velocity if cmd == 0 else feedrate
        waypoints.append((x, y, z, e, move_feedrate))

    return waypoints


def _pad_pieces(pieces):
    # Pieces are [t0, t1, c0, c1, ..., cn] and rows may be ragged (degree
    # varies per row, 4..10 floats). Zero-pad to the row with the highest
    # degree so the rest of the module can evaluate them as one matrix; a
    # missing high coefficient contributes nothing, matching the firmware's
    # own degree-generic reader.
    import numpy as np

    max_len = max(len(row) for row in pieces)
    p = np.zeros((len(pieces), max_len))
    for i, row in enumerate(pieces):
        p[i, : len(row)] = row
    return p


def _eval_pieces(pieces, t):
    # Evaluate the per-axis monomial pieces -- the trajectory the firmware runs
    # -- and their analytic derivatives at times `t`. Each piece is
    # [t0, t1, c0, c1, ..., cn]: pos = sum(ck * tau^k), tau = t - t0, so
    # velocity/acceleration/jerk are exact polynomial derivatives, no
    # differencing. Degree-generic via Horner over the (zero-padded) rows.
    import numpy as np

    p = _pad_pieces(pieces)
    starts = p[:, 0]
    idx = np.clip(np.searchsorted(starts, t, side="right") - 1, 0, len(p) - 1)
    sel = p[idx]
    tau = t - sel[:, 0]
    coeffs = sel[:, 2:]
    n_terms = coeffs.shape[1]

    pos = np.zeros_like(tau)
    vel = np.zeros_like(tau)
    acc = np.zeros_like(tau)
    jerk = np.zeros_like(tau)
    for k in range(n_terms - 1, -1, -1):
        ck = coeffs[:, k]
        pos = pos * tau + ck
        if k >= 1:
            vel = vel * tau + ck * k
        if k >= 2:
            acc = acc * tau + ck * k * (k - 1)
        if k >= 3:
            jerk = jerk * tau + ck * k * (k - 1) * (k - 2)
    return pos, vel, acc, jerk


def _build_time_series(snapshot):
    # New baselines store the lowered cubic trajectory; older ones stored sampled
    # position + speed. Dispatch on which is present so both still render.
    if "traj_x_pieces" in snapshot:
        return _time_series_from_pieces(snapshot)
    return _time_series_from_position(snapshot)


# Axis lane -> (snapshot piece key, plot color). X/Y drive the |v| magnitude
# trace; Z and E are extra per-axis lanes captured from the lowered trajectory.
_AXIS_LANES = (
    ("X", "traj_x_pieces", "C0"),
    ("Y", "traj_y_pieces", "C1"),
    ("Z", "traj_z_pieces", "C2"),
    ("E", "traj_e_pieces", "C4"),
)


def _time_series_from_pieces(snapshot):
    import numpy as np

    # The visualizer reads the lowered trajectory the firmware executes -- per-axis
    # cubic pieces of position vs time -- and differentiates them analytically. Every
    # derivative is exact and continuous (no position differencing, nothing copied
    # from the planner's own acceleration), so the curves stay an independent check.
    xp = snapshot["traj_x_pieces"]
    t_end = float(snapshot["traj_t_end"])
    if not xp or t_end <= 0.0:
        z = np.zeros(1)
        empty = {name: z for name, _, _ in _AXIS_LANES}
        return {
            "t": z,
            "vel": empty,
            "acc": empty,
            "jerk": empty,
            "v_scalar": z,
            "a_scalar": z,
            "j_scalar": z,
        }

    # Every piece boundary is a candidate C1-Hermite acceleration step, and
    # higher-degree pieces (once the writer emits them) can also carry an
    # acceleration peak strictly inside a piece, not just at its edges. Sample
    # each boundary-to-boundary interval on its own dense sub-grid (rather
    # than one uniform grid over the whole trajectory) so both the exact
    # steps and any interior curvature peaks are captured regardless of how
    # many pieces the trajectory has.
    lane_pieces = {name: snapshot.get(key) for name, key, _ in _AXIS_LANES}
    bounds = {0.0, t_end}
    for pieces in lane_pieces.values():
        if pieces:
            for row in pieces:
                t0, t1 = row[0], row[1]
                if 0.0 < t0 < t_end:
                    bounds.add(t0)
                if 0.0 < t1 < t_end:
                    bounds.add(t1)
    bounds = np.array(sorted(bounds))

    n_intervals = max(len(bounds) - 1, 1)
    per_interval = max(int(max(8 * len(xp), 2000) / n_intervals), 32)
    t = np.unique(
        np.concatenate(
            [
                np.linspace(a, b, per_interval)
                for a, b in zip(bounds[:-1], bounds[1:])
            ]
        )
    )

    vel, acc, jerk = {}, {}, {}
    for name, pieces in lane_pieces.items():
        if pieces:
            _, v, a, j = _eval_pieces(pieces, t)
        else:
            v = a = j = np.zeros_like(t)
        vel[name], acc[name], jerk[name] = v, a, j

    return {
        "t": t,
        "vel": vel,
        "acc": acc,
        "jerk": jerk,
        "v_scalar": np.hypot(vel["X"], vel["Y"]),
        "a_scalar": np.hypot(acc["X"], acc["Y"]),
        "j_scalar": np.hypot(jerk["X"], jerk["Y"]),
    }


def _time_series_from_position(snapshot):
    import numpy as np

    # Legacy baselines: only sampled position + speed were stored. Build a time
    # axis from the speed profile (dt = ds / v) and differentiate position
    # numerically -- nothing from the planner's own acceleration is trusted.
    x, y = _toolhead_position(snapshot)
    v = np.array(snapshot["kin_v"])

    distinct = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 1e-9])
    x, y, v = x[distinct], y[distinct], v[distinct]

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

    zeros = np.zeros_like(t)
    return {
        "t": t,
        "vel": {"X": vx, "Y": vy, "Z": zeros, "E": zeros},
        "acc": {"X": ax, "Y": ay, "Z": zeros, "E": zeros},
        "jerk": {"X": jx, "Y": jy, "Z": zeros, "E": zeros},
        "v_scalar": v_scalar,
        "a_scalar": a_scalar,
        "j_scalar": j_scalar,
    }


def _toolhead_position(snapshot):
    import numpy as np

    if "kin_x" in snapshot:
        return np.array(snapshot["kin_x"]), np.array(snapshot["kin_y"])
    # Older baselines stored heading + arc length instead of position; integrate
    # the unit heading along s to recover the path so they still preview.
    s = np.array(snapshot["kin_s"])
    hx = np.array(snapshot["kin_heading_x"])
    hy = np.array(snapshot["kin_heading_y"])
    ds = np.diff(s, prepend=s[0])
    x = snapshot["raw_x"][0] + np.cumsum(hx * ds)
    y = snapshot["raw_y"][0] + np.cumsum(hy * ds)
    return x, y


def _plot_derivative(ax, t, comps, scalar, ylabel, title):
    import numpy as np

    def plot(y, **kw):
        ax.plot(t, y, **kw)

    for name, _, color in _AXIS_LANES:
        lane = comps.get(name)
        if lane is None or not np.any(lane):
            continue
        plot(np.abs(lane), linewidth=0.6, color=color, label=f"|{name}|")
    plot(scalar, linewidth=0.8, color="C3", label="|XY|")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=7, loc="upper right")


def _seam_summary(snapshot) -> str | None:
    # Worst per-axis continuity jumps across piece seams: position/velocity should
    # be ~0 (C1 Hermite), acceleration steps by design. Absent on legacy baselines.
    dp = snapshot.get("seam_max_dp")
    dv = snapshot.get("seam_max_dv")
    da = snapshot.get("seam_max_da")
    if dp is None or dv is None or da is None:
        return None
    return (
        f"seam max  |Δp|={max(dp):.2e} mm  "
        f"|Δv|={max(dv):.2e} mm/s  |Δa|={max(da):.2e} mm/s²"
    )


def render(snapshot, out_path, stem, ts):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw_x, raw_y = snapshot["raw_x"], snapshot["raw_y"]
    segments = list(snapshot["fitted_segments"])

    series = _build_time_series(snapshot)
    t = series["t"]

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
    ax_path.set_title(f"{stem}  [{seg_summary}]", fontsize=9)

    seam_text = _seam_summary(snapshot)
    vel_title = f"Velocity  (t={snapshot['traversal_time_s']:.3f}s)"
    if seam_text:
        vel_title = f"{vel_title}\n{seam_text}"
    _plot_derivative(
        ax_vel, t, series["vel"], series["v_scalar"], "mm/s", vel_title
    )
    _plot_derivative(
        ax_acc, t, series["acc"], series["a_scalar"], "mm/s²", "Acceleration"
    )
    _plot_derivative(
        ax_jrk, t, series["jerk"], series["j_scalar"], "mm/s³", "Jerk"
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

    (
        max_velocity,
        max_accel,
        scv,
        max_jerk,
        arc_fit,
        extrude_only_velocity,
        extrude_only_accel,
        max_path_deviation,
        max_accel_deviation,
    ) = read_printer_config(args.config)

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
        max_extrude_only_velocity=extrude_only_velocity,
        max_extrude_only_accel=extrude_only_accel,
        max_path_deviation=max_path_deviation,
        max_accel_deviation=max_accel_deviation,
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
