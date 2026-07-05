#!/usr/bin/env python3
"""Render an inertia-ratio sweep comparison from SERVO_SWEEP_INERTIA captures.

The sweep macro records one capture per C00.06 load-inertia-ratio step, named
<tag>_r<RATIO>, which SERVO_CAPTURE timestamps into
<name>_<YYYYmmdd_HHMMSS>.scap. This script resolves each step to its newest
capture and renders the same panels as the gain report (cruise following-error
spectrum + time domain, and an overshoot curve), labeled by inertia ratio. The
inertia signal lives in the accel/decel edges, so the overshoot panel is the
one to read; this first cut presents the comparison visually and prints a
metrics table, with no automated recommendation.

The sweep macro passes --steps with the exact step names it just recorded,
so the report covers only that run. Without --steps the script falls back to
every step name matching the tag, which mixes in steps left over from older
runs that used different ratio lists.

Usage:
  servo_inertia_report.py --tag inertia --steps inertia_r40,inertia_r100
  servo_inertia_report.py --captures-dir ~/printer_data/logs/servo_captures \
      --tag inertia --out-dir ~/printer_data/config/servo_calibrate_results
  servo_inertia_report.py inertia_r40_*.scap inertia_r100_*.scap --out r.png
"""

import argparse
import datetime
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_gain_report import (  # noqa: E402
    RESONANCE_BAND_HZ,
    add_resonance_zoom,
    step_metrics,
)

STEP_RE = re.compile(r"_r(\d+)_\d{8}_\d{6}\.scap$")


def ratio_from_name(path):
    m = STEP_RE.search(os.path.basename(path))
    if not m:
        return None
    return int(m.group(1))


def find_sweep_files(captures_dir, tag):
    newest = {}
    pattern = os.path.join(os.path.expanduser(captures_dir), tag + "_r*.scap")
    for path in glob.glob(pattern):
        ratio = ratio_from_name(path)
        if ratio is None:
            continue
        if ratio not in newest or path > newest[ratio]:
            newest[ratio] = path
    return [(k, newest[k]) for k in sorted(newest)]


def find_named_steps(captures_dir, step_names):
    files = []
    for name in step_names:
        pattern = os.path.join(
            os.path.expanduser(captures_dir), name + "_*.scap"
        )
        matches = [p for p in glob.glob(pattern) if STEP_RE.search(p)]
        if not matches:
            raise SystemExit(
                "sweep step %r has no capture in %s" % (name, captures_dir)
            )
        path = max(matches)
        files.append((ratio_from_name(path), path))
    files.sort(key=lambda kp: kp[0])
    return files


def render(steps, out_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = plt.cm.viridis(np.linspace(0.0, 0.85, len(steps)))

    spec_ax, time_ax = axes[0]
    linestyles = ["-", "--", ":", "-."]
    for (ratio, met), color in zip(steps, colors):
        for k, dm in enumerate(met["drives"]):
            label = "inertia %d%%%s%s" % (
                ratio,
                " [%s]" % dm["drive"] if len(met["drives"]) > 1 else "",
                "  RESONANT" if dm["resonant"] else "",
            )
            ls = linestyles[k % len(linestyles)]
            freqs, spectrum = dm["spectrum"]
            spec_ax.loglog(
                freqs[1:],
                np.convolve(spectrum[1:] * 1000.0, np.ones(3) / 3, "same"),
                color=color,
                ls=ls,
                lw=1.0,
                label=label,
            )
            for w, window in enumerate(dm["stop_windows"]):
                time_ax.plot(
                    np.arange(len(window)) / dm["fs"] - dm["stop_lookback_s"],
                    window * 1000.0,
                    color=color,
                    ls=ls,
                    lw=0.7,
                    label=label if w == 0 else None,
                )
    spec_ax.axvspan(*RESONANCE_BAND_HZ, alpha=0.06, color="red")
    add_resonance_zoom(spec_ax, steps, colors, linestyles)
    spec_ax.set_xlabel("Hz")
    spec_ax.set_ylabel("ferr amplitude (um)")
    spec_ax.set_title(
        "Cruise following-error spectrum (red band: resonance watch)"
    )
    spec_ax.legend(fontsize=8)
    spec_ax.grid(True, which="both", alpha=0.3)
    time_ax.axvline(0.0, color="k", lw=0.8, alpha=0.5)
    time_ax.set_xlabel("s relative to stop")
    time_ax.set_ylabel("ferr toward endpoint (um)")
    time_ax.set_title("Following error around each stop (decel edges overlaid)")
    time_ax.grid(alpha=0.3)

    curve_ax, table_ax = axes[1]
    ratios = [r for r, _ in steps]
    for key, label, scale in (
        ("overshoot_max_um", "overshoot max (um)", 1.0),
        ("ferr_std_um", "cruise error std (um)", 1.0),
        ("low_band_um", "low-band disturbance (um)", 1.0),
        ("lag_ms", "lag (ms) x10", 10.0),
    ):
        curve_ax.plot(
            ratios, [m[key] * scale for _, m in steps], marker="o", label=label
        )
    for (ratio, met), x in zip(steps, ratios):
        if met["resonant"]:
            curve_ax.axvline(x, color="red", ls="--", alpha=0.5)
    curve_ax.set_xlabel("inertia ratio (%)")
    curve_ax.set_title("Metrics vs inertia ratio (red dashed: resonant step)")
    curve_ax.legend(fontsize=8)
    curve_ax.grid(alpha=0.3)

    table_ax.axis("off")
    rows = [
        [
            "%d" % ratio,
            "%.1f" % m["lag_ms"],
            "%.0f" % m["ferr_std_um"],
            "%.0f" % m["low_band_um"],
            "%.0f @ %.0fHz" % (m["res_peak_um"], m["res_peak_hz"]),
            "%.0f" % m["overshoot_max_um"],
            "YES" if m["resonant"] else "no",
        ]
        for ratio, m in steps
    ]
    table = table_ax.table(
        cellText=rows,
        colLabels=[
            "ratio %",
            "lag ms",
            "err um",
            "low um",
            "res peak",
            "ovsh um",
            "resonant",
        ],
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table_ax.set_title("C00.06 % / um / ms", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("captures", nargs="*", help="explicit .scap files")
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument("--tag", default="inertia")
    p.add_argument(
        "--steps",
        help="comma list of step names recorded by this sweep run "
        "(<tag>_r<RATIO>); only these steps are reported",
    )
    p.add_argument(
        "--out-dir", default="~/printer_data/config/servo_calibrate_results"
    )
    p.add_argument("--out", help="explicit output PNG path")
    p.add_argument(
        "--drive",
        help="drive name to analyze in a multi-drive capture "
        "(default: the first drive in the file)",
    )
    args = p.parse_args(argv)

    if args.captures and args.steps:
        raise SystemExit("pass explicit .scap files or --steps, not both")
    if args.captures:
        files = []
        for path in args.captures:
            ratio = ratio_from_name(path)
            if ratio is None:
                raise SystemExit(
                    "%s: filename lacks _r<RATIO>_<ts>.scap ratio field"
                    % (path,)
                )
            files.append((ratio, path))
        files.sort(key=lambda kp: kp[0])
    elif args.steps:
        files = find_named_steps(args.captures_dir, args.steps.split(","))
    else:
        files = find_sweep_files(args.captures_dir, args.tag)
    if not files:
        raise SystemExit("no sweep captures found (tag %r)" % (args.tag,))

    steps = [(ratio, step_metrics(path, args.drive)) for ratio, path in files]

    if args.out:
        out_path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    else:
        out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(
            out_dir, "inertia_%s_%s.png" % (args.tag, stamp)
        )
    render(steps, out_path)

    print(
        "%-8s %7s %7s %7s %12s %8s %s"
        % (
            "ratio %",
            "lag ms",
            "err um",
            "low um",
            "res peak",
            "ovsh um",
            "resonant",
        )
    )
    for ratio, met in steps:
        print(
            "%-8d %7.1f %7.0f %7.0f %6.0f@%3.0fHz %8.0f %s"
            % (
                ratio,
                met["lag_ms"],
                met["ferr_std_um"],
                met["low_band_um"],
                met["res_peak_um"],
                met["res_peak_hz"],
                met["overshoot_max_um"],
                "YES" if met["resonant"] else "no",
            )
        )
    print("low-band amplitudes are FYI; read the overshoot column for inertia")
    print("report: %s" % (out_path,))


if __name__ == "__main__":
    main()
