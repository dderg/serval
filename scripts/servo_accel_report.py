#!/usr/bin/env python3
"""Render an accel-sweep torque-saturation report from SERVO_SWEEP_ACCEL captures.

The sweep macro records one capture per acceleration step, named
<tag>_a<ACCEL>, which SERVO_CAPTURE timestamps into
<name>_<YYYYmmdd_HHMMSS>.scap. This script resolves each step to its newest
capture, computes per-drive torque saturation (rail detection) plus the
accel/decel edge following error, writes a PNG, prints a table, and recommends
the highest acceleration whose motors never hit the torque rail.

The sweep macro passes --steps with the exact step names it just recorded,
so the report covers only that run. Without --steps the script falls back to
every step name matching the tag, which mixes in steps left over from older
runs that used different accel lists.

Usage:
  servo_accel_report.py --tag accel --steps accel_a10000,accel_a20000
  servo_accel_report.py --captures-dir ~/printer_data/logs/servo_captures \
      --tag accel --out-dir ~/printer_data/config/servo_calibrate_results
  servo_accel_report.py accel_a10000_*.scap accel_a20000_*.scap --out r.png
"""

import argparse
import datetime
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_capture import load_capture, torque_summary  # noqa: E402
from servo_gain_report import drive_metrics  # noqa: E402

STEP_RE = re.compile(r"_a(\d+)_\d{8}_\d{6}\.scap$")


def accel_from_name(path):
    m = STEP_RE.search(os.path.basename(path))
    if not m:
        return None
    return int(m.group(1))


def find_sweep_files(captures_dir, tag):
    newest = {}
    pattern = os.path.join(os.path.expanduser(captures_dir), tag + "_a*.scap")
    for path in glob.glob(pattern):
        accel = accel_from_name(path)
        if accel is None:
            continue
        if accel not in newest or path > newest[accel]:
            newest[accel] = path
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
        files.append((accel_from_name(path), path))
    files.sort(key=lambda kp: kp[0])
    return files


def accel_drive_metrics(path, drive, torque_limit):
    header, data, drive_idx = load_capture(path, drive)
    fs = 1e9 / header["cycle_ns"]
    tq = torque_summary(data, torque_limit, fs)
    gm = drive_metrics(path, header["drives"][drive_idx]["name"])
    edge_peak_um = 0.0
    for window in gm["stop_windows"]:
        edge_peak_um = max(edge_peak_um, float(np.max(np.abs(window))) * 1000.0)
    t = np.arange(len(data)) / fs
    return {
        "drive": header["drives"][drive_idx]["name"],
        "torque_peak": tq["peak"],
        "rail_detected": tq["rail_detected"],
        "rail_pct": tq["rail_pct_moving"],
        "rail_ms": tq["rail_ms"],
        "edge_peak_um": edge_peak_um,
        "cruise_std_um": gm["ferr_std_um"],
        "torque_trace": data["torque_actual"].astype(np.float64),
        "t": t,
    }


def step_metrics(path, drive, torque_limit):
    header, _, _ = load_capture(path, drive)
    names = (
        [drive] if drive is not None else [d["name"] for d in header["drives"]]
    )
    per_drive = [accel_drive_metrics(path, n, torque_limit) for n in names]
    worst_torque = max(per_drive, key=lambda m: m["torque_peak"])
    return {
        "path": path,
        "torque_peak": max(m["torque_peak"] for m in per_drive),
        "rail_pct": max(m["rail_pct"] for m in per_drive),
        "rail_detected": any(m["rail_detected"] for m in per_drive),
        "edge_peak_um": max(m["edge_peak_um"] for m in per_drive),
        "cruise_std_um": max(m["cruise_std_um"] for m in per_drive),
        "worst_torque_drive": worst_torque,
        "drives": per_drive,
    }


def recommend(steps):
    clean = [(a, m) for a, m in steps if not m["rail_detected"]]
    if not clean:
        return None, "every accel step hit the torque rail — lower the accel"
    accel = max(a for a, _ in clean)
    note = "highest accel with zero rail samples on every motor"
    hit = [a for a, m in steps if m["rail_detected"] and a > accel]
    if hit:
        note += "; %d mm/s^2 rejected (torque rail)" % (min(hit),)
    return accel, note


def render(steps, out_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    accels = [a for a, _ in steps]
    fig = plt.figure(figsize=(14, 9))
    tq_ax = fig.add_subplot(2, 2, 1)
    rail_ax = tq_ax.twinx()
    tq_ax.plot(
        accels,
        [m["torque_peak"] for _, m in steps],
        marker="o",
        color="tab:green",
        label="peak torque (per-mille)",
    )
    rail_ax.plot(
        accels,
        [m["rail_pct"] for _, m in steps],
        marker="s",
        color="tab:red",
        label="rail % of moving samples",
    )
    tq_ax.set_xlabel("accel (mm/s^2)")
    tq_ax.set_ylabel("peak torque (per-mille)", color="tab:green")
    rail_ax.set_ylabel("rail % of moving samples", color="tab:red")
    tq_ax.set_title("Peak torque and rail time vs accel")
    tq_ax.grid(alpha=0.3)

    ferr_ax = fig.add_subplot(2, 2, 2)
    ferr_ax.plot(
        accels,
        [m["edge_peak_um"] for _, m in steps],
        marker="o",
        label="edge peak ferr (um)",
    )
    ferr_ax.plot(
        accels,
        [m["cruise_std_um"] for _, m in steps],
        marker="s",
        label="cruise ferr std (um)",
    )
    ferr_ax.set_xlabel("accel (mm/s^2)")
    ferr_ax.set_ylabel("following error (um)")
    ferr_ax.set_title("Following error vs accel")
    ferr_ax.legend(fontsize=8)
    ferr_ax.grid(alpha=0.3)

    ov_ax = fig.add_subplot(2, 1, 2)
    colors = plt.cm.viridis(np.linspace(0.0, 0.85, len(steps)))
    for (accel, met), color in zip(steps, colors):
        wd = met["worst_torque_drive"]
        ov_ax.plot(
            wd["t"],
            wd["torque_trace"],
            color=color,
            lw=0.7,
            label="%d mm/s^2 [%s]" % (accel, wd["drive"]),
        )
    ov_ax.set_xlabel("time (s)")
    ov_ax.set_ylabel("torque (per-mille)")
    ov_ax.set_title("Torque vs time (worst motor per accel)")
    ov_ax.legend(fontsize=8, ncol=2)
    ov_ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("captures", nargs="*", help="explicit .scap files")
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument("--tag", default="accel")
    p.add_argument(
        "--steps",
        help="comma list of step names recorded by this sweep run "
        "(<tag>_a<ACCEL>); only these steps are reported",
    )
    p.add_argument(
        "--out-dir", default="~/printer_data/config/servo_calibrate_results"
    )
    p.add_argument("--out", help="explicit output PNG path")
    p.add_argument(
        "--drive",
        help="restrict analysis to one drive of a multi-drive capture "
        "(default: analyze all drives, merged worst-case)",
    )
    p.add_argument(
        "--torque-limit",
        type=int,
        default=900,
        help="rail threshold, per-mille of rated (default 900)",
    )
    args = p.parse_args(argv)

    if args.captures and args.steps:
        raise SystemExit("pass explicit .scap files or --steps, not both")
    if args.captures:
        files = []
        for path in args.captures:
            accel = accel_from_name(path)
            if accel is None:
                raise SystemExit(
                    "%s: filename lacks _a<ACCEL>_<ts>.scap accel field"
                    % (path,)
                )
            files.append((accel, path))
        files.sort(key=lambda kp: kp[0])
    elif args.steps:
        files = find_named_steps(args.captures_dir, args.steps.split(","))
    else:
        files = find_sweep_files(args.captures_dir, args.tag)
    if not files:
        raise SystemExit("no sweep captures found (tag %r)" % (args.tag,))

    steps = [
        (accel, step_metrics(path, args.drive, args.torque_limit))
        for accel, path in files
    ]

    if args.out:
        out_path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    else:
        out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, "accel_%s_%s.png" % (args.tag, stamp))
    render(steps, out_path)

    print(
        "%-10s %10s %8s %10s %10s %s"
        % (
            "accel",
            "tq peak",
            "rail %",
            "edge um",
            "cruise um",
            "rail",
        )
    )
    for accel, met in steps:
        print(
            "%-10d %10d %8.1f %10.0f %10.0f %s"
            % (
                accel,
                met["torque_peak"],
                met["rail_pct"],
                met["edge_peak_um"],
                met["cruise_std_um"],
                "YES" if met["rail_detected"] else "no",
            )
        )
    accel, note = recommend(steps)
    if accel is not None:
        print("recommended max accel: %d mm/s^2  (%s)" % (accel, note))
    else:
        print("recommendation: %s" % (note,))
    print("report: %s" % (out_path,))


if __name__ == "__main__":
    main()
