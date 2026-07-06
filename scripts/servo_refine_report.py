#!/usr/bin/env python3
"""Render a gain-refinement sweep comparison from SERVO_REFINE_GAIN captures.

SERVO_REFINE_GAIN sweeps a single drive gain (position, speed or integral)
around the current operating point while holding the other two fixed. It
records one capture per step, named <tag>_<param>_v<VALUE>, which SERVO_CAPTURE
timestamps into <name>_<YYYYmmdd_HHMMSS>.scap. This script resolves each step
to its newest capture and renders the same panels as the gain report (cruise
following-error spectrum + time domain, and a metrics-vs-value curve), labeled
by the parameter value in drive units, with the reference (current) value
marked. It prints a metrics table and a per-parameter hint; it makes no
automated recommendation.

The sweep macro passes --steps with the exact step names it just recorded,
so the report covers only that run. Without --steps the script falls back to
every step name matching the tag/param, which mixes in steps left over from
older runs that used different value lists.

Usage:
  servo_refine_report.py --param speed --tag refine \
      --steps refine_speed_v1750,refine_speed_v2500 --reference 2500
  servo_refine_report.py --captures-dir ~/printer_data/logs/servo_captures \
      --param position --tag refine
  servo_refine_report.py refine_speed_v1750_*.scap --param speed --out r.png
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
    resonance_zoom_panel,
    step_metrics,
)

STEP_RE = re.compile(r"_(position|speed|integral)_v(\d+)_\d{8}_\d{6}\.scap$")

PARAM_INFO = {
    "position": (
        "position loop gain",
        10.0,
        "rad/s",
        "watch overshoot + transient",
        "overshoot_max_um",
    ),
    "speed": (
        "speed loop gain",
        10.0,
        "Hz",
        "watch resonance protrusion + cruise std",
        "ferr_std_um",
    ),
    "integral": (
        "speed integral time",
        100.0,
        "ms",
        "watch low-band + reversal recovery (lower = stiffer)",
        "low_band_um",
    ),
}


def parse_step_name(path):
    m = STEP_RE.search(os.path.basename(path))
    if not m:
        return None, None
    return m.group(1), int(m.group(2))


def value_from_name(path):
    return parse_step_name(path)[1]


def param_from_name(path):
    return parse_step_name(path)[0]


def find_sweep_files(captures_dir, tag, param):
    newest = {}
    pattern = os.path.join(
        os.path.expanduser(captures_dir), "%s_%s_v*.scap" % (tag, param)
    )
    for path in glob.glob(pattern):
        p, value = parse_step_name(path)
        if p != param:
            continue
        if value not in newest or path > newest[value]:
            newest[value] = path
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
        files.append((value_from_name(path), path))
    files.sort(key=lambda kp: kp[0])
    return files


def render(steps, param, reference, out_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    label, scale, unit, hint, highlight = PARAM_INFO[param]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = plt.cm.viridis(np.linspace(0.0, 0.85, len(steps)))

    spec_ax, time_ax = axes[0]
    linestyles = ["-", "--", ":", "-."]
    for (value, met), color in zip(steps, colors):
        for k, dm in enumerate(met["drives"]):
            marker = " (current)" if value == reference else ""
            legend = "%.4g %s%s%s%s" % (
                value / scale,
                unit,
                marker,
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
                label=legend,
            )
            seg = dm["cruise_ferr"][: int(round(1.5 * dm["fs"]))]
            time_ax.plot(
                np.arange(len(seg)) / dm["fs"],
                seg * 1000.0,
                color=color,
                ls=ls,
                lw=0.7,
            )
    spec_ax.axvspan(*RESONANCE_BAND_HZ, alpha=0.06, color="red")
    spec_ax.set_xlabel("Hz")
    spec_ax.set_ylabel("ferr amplitude (um)")
    spec_ax.set_title(
        "Cruise following-error spectrum (red band: resonance watch)"
    )
    spec_ax.legend(fontsize=8)
    spec_ax.grid(True, which="both", alpha=0.3)
    time_ax.set_xlabel("s into cruise")
    time_ax.set_ylabel("ferr (mm)")
    time_ax.set_title("Cruise following error, time domain")
    time_ax.grid(alpha=0.3)

    curve_ax, zoom_ax = axes[1]
    values = [v / scale for v, _ in steps]
    curves = (
        ("overshoot_max_um", "overshoot max (um)", 1.0),
        ("ferr_std_um", "cruise error std (um)", 1.0),
        ("low_band_um", "low-band disturbance (um)", 1.0),
        ("lag_ms", "lag (ms) x10", 10.0),
    )
    for key, name, sc in curves:
        lw = 2.4 if key == highlight else 1.2
        curve_ax.plot(
            values,
            [m[key] * sc for _, m in steps],
            marker="o",
            lw=lw,
            label=name + (" *" if key == highlight else ""),
        )
    for (value, met), x in zip(steps, values):
        if met["resonant"]:
            curve_ax.axvline(x, color="red", ls="--", alpha=0.5)
    curve_ax.axvline(
        reference / scale, color="black", ls=":", alpha=0.7, label="current"
    )
    curve_ax.set_xlabel("%s (%s)" % (label, unit))
    curve_ax.set_title("Metrics vs %s (* = key metric; %s)" % (label, hint))
    curve_ax.legend(fontsize=8)
    curve_ax.grid(alpha=0.3)

    resonance_zoom_panel(zoom_ax, steps, colors, linestyles)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("captures", nargs="*", help="explicit .scap files")
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument(
        "--param",
        choices=sorted(PARAM_INFO),
        help="swept gain parameter (inferred from filenames if omitted)",
    )
    p.add_argument("--tag", default="refine")
    p.add_argument(
        "--steps",
        help="comma list of step names recorded by this sweep run "
        "(<tag>_<param>_v<VALUE>); only these steps are reported",
    )
    p.add_argument(
        "--reference",
        type=int,
        help="the current (reference) value in drive units, marked in output",
    )
    p.add_argument(
        "--out-dir", default="~/printer_data/config/servo_calibrate_results"
    )
    p.add_argument("--out", help="explicit output PNG path")
    p.add_argument(
        "--drive",
        help="drive name to analyze in a multi-drive capture "
        "(default: analyze all drives, merged worst-case)",
    )
    args = p.parse_args(argv)

    if args.captures and args.steps:
        raise SystemExit("pass explicit .scap files or --steps, not both")
    if args.captures:
        files = []
        for path in args.captures:
            param, value = parse_step_name(path)
            if value is None:
                raise SystemExit(
                    "%s: filename lacks _<param>_v<VALUE>_<ts>.scap value field"
                    % (path,)
                )
            files.append((value, path))
        files.sort(key=lambda kp: kp[0])
        inferred = param_from_name(args.captures[0])
    elif args.steps:
        files = find_named_steps(args.captures_dir, args.steps.split(","))
        inferred = param_from_name(files[0][1]) if files else None
    else:
        param = args.param
        if param is None:
            raise SystemExit(
                "--param is required when resolving by --tag (got none)"
            )
        files = find_sweep_files(args.captures_dir, args.tag, param)
        inferred = param
    if not files:
        raise SystemExit("no refinement captures found (tag %r)" % (args.tag,))

    param = args.param or inferred
    if param not in PARAM_INFO:
        raise SystemExit(
            "could not determine PARAM; pass --param position|speed|integral"
        )

    steps = [(value, step_metrics(path, args.drive)) for value, path in files]
    reference = args.reference if args.reference is not None else steps[0][0]

    if args.out:
        out_path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    else:
        out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(
            out_dir, "refine_%s_%s_%s.png" % (param, args.tag, stamp)
        )
    render(steps, param, reference, out_path)

    label, scale, unit, hint, _highlight = PARAM_INFO[param]
    print("refining %s -- %s" % (label, hint))
    print(
        "%-12s %7s %7s %7s %12s %8s %s"
        % (
            unit,
            "lag ms",
            "err um",
            "low um",
            "res peak",
            "ovsh um",
            "resonant",
        )
    )
    for value, met in steps:
        print(
            "%-12s %7.1f %7.0f %7.0f %6.0f@%3.0fHz %8.0f %s"
            % (
                "%.4g%s" % (value / scale, " *" if value == reference else ""),
                met["lag_ms"],
                met["ferr_std_um"],
                met["low_band_um"],
                met["res_peak_um"],
                met["res_peak_hz"],
                met["overshoot_max_um"],
                "YES" if met["resonant"] else "no",
            )
        )
    print("* marks the current (reference) value")
    print("report: %s" % (out_path,))


if __name__ == "__main__":
    main()
