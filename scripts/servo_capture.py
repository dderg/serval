#!/usr/bin/env python3
"""Analyze a servo telemetry capture (.scap) produced by SERVO_CAPTURE_START.

Prints following-error, overshoot/settling, and torque-saturation metrics;
--fft prints resonance peaks (notch-filter candidates); --plot opens a
time-series dashboard.
"""

import argparse
import json
import os
import re
import sys

import numpy as np

CAPTURE_TS_RE = re.compile(r"_(\d{8}_\d{6})\.scap$")

DTYPE_MAP = {
    "u8": "u1",
    "u16": "<u2",
    "i16": "<i2",
    "i32": "<i4",
    "u64": "<u8",
    "f32": "<f4",
}
SUPPORTED_VERSIONS = (1, 2)
FLAG_MOTION_ACTIVE = 1 << 1
SETTLE_HOLD_MS = 50
RECORD_PREFIX_SIZE = 9


def resolve_newest_capture(captures_dir, name):
    directory = os.path.expanduser(captures_dir)
    name_re = re.compile(r"^%s_(\d{8}_\d{6})\.scap$" % re.escape(name))
    stamped = []
    for entry in os.listdir(directory):
        match = name_re.match(entry)
        if match:
            stamped.append((match.group(1), os.path.join(directory, entry)))
    if not stamped:
        raise SystemExit("no capture named %r in %s" % (name, captures_dir))
    return max(stamped)[1]


def select_drive(header, drive_name):
    names = [d["name"] for d in header["drives"]]
    if drive_name is None:
        return 0
    if drive_name not in names:
        raise SystemExit(
            "drive %r not in capture (have: %s)"
            % (drive_name, ", ".join(names))
        )
    return names.index(drive_name)


def load_capture(path, drive=None):
    if path.endswith(".failed.scap"):
        raise SystemExit(
            "%s is a FAILED capture (ring overflow or writer error); its "
            "gaps would poison every metric. Re-run the capture." % (path,)
        )
    with open(path, "rb") as f:
        header = json.loads(f.readline())
        if header.get("version") not in SUPPORTED_VERSIONS:
            raise SystemExit(
                "unsupported capture version %r" % (header.get("version"),)
            )
        n_drives = len(header["drives"])
        if n_drives == 0:
            raise SystemExit("%s header lists no drives" % (path,))
        record_size = header["record_size"]
        body_size = record_size - RECORD_PREFIX_SIZE
        if body_size <= 0 or body_size % n_drives:
            raise SystemExit(
                "record_size %d is not aligned to %d drive block(s)"
                % (record_size, n_drives)
            )
        block_size = body_size // n_drives
        drive_idx = select_drive(header, drive)
        names, formats, offsets = [], [], []
        for c in header["channels"]:
            off = c["offset"]
            if off >= RECORD_PREFIX_SIZE:
                off += drive_idx * block_size
            fmt = DTYPE_MAP[c["dtype"]]
            if off + np.dtype(fmt).itemsize > record_size:
                raise SystemExit(
                    "channel %r at offset %d overruns record_size %d"
                    % (c["name"], off, record_size)
                )
            names.append(c["name"])
            formats.append(fmt)
            offsets.append(off)
        dtype = np.dtype(
            {
                "names": names,
                "formats": formats,
                "offsets": offsets,
                "itemsize": record_size,
            }
        )
        body = f.read()
    whole = len(body) // record_size * record_size
    data = np.frombuffer(body[:whole], dtype=dtype)
    return header, data, drive_idx


def motion_segments(flags):
    if not len(flags):
        return []
    moving = (flags & FLAG_MOTION_ACTIVE) != 0
    edges = np.flatnonzero(np.diff(moving.astype(np.int8)))
    bounds = np.concatenate(([0], edges + 1, [len(moving)]))
    return [
        (int(bounds[i]), int(bounds[i + 1]))
        for i in range(len(bounds) - 1)
        if moving[bounds[i]]
    ]


def _settle_index(err, band, hold):
    inside = np.abs(err) <= band
    if len(inside) < hold:
        return None
    windows = np.lib.stride_tricks.sliding_window_view(inside, hold)
    ok = np.flatnonzero(windows.all(axis=1))
    return int(ok[0]) if len(ok) else None


def compute_metrics(data, settle_band, torque_limit, fs=1000.0):
    if not len(data):
        raise SystemExit("capture contains no records")
    ms_per_sample = 1000.0 / fs
    hold = int(round(SETTLE_HOLD_MS * fs / 1000.0))
    ferr = data["following_error"].astype(np.float64)
    recomputed = data["target_counts"].astype(np.int64) - data[
        "position_actual"
    ].astype(np.int64)
    segs = motion_segments(data["flags"])
    moves = []
    for idx, (s, e) in enumerate(segs):
        move_err = ferr[s:e]
        post_end = segs[idx + 1][0] if idx + 1 < len(segs) else len(ferr)
        post = ferr[e:post_end]
        settle_sample = _settle_index(post, settle_band, hold)
        overshoot_end = (
            settle_sample if settle_sample is not None else len(post)
        )
        settle_ms = (
            float(settle_sample) * ms_per_sample
            if settle_sample is not None
            else None
        )
        moves.append(
            {
                "move": idx,
                "start_ms": float(s) * ms_per_sample,
                "end_ms": float(e) * ms_per_sample,
                "ferr_peak": float(np.max(np.abs(move_err))),
                "ferr_rms": float(np.sqrt(np.mean(move_err**2))),
                "overshoot": float(np.max(np.abs(post[:overshoot_end])))
                if overshoot_end > 0
                else 0.0,
                "settle_ms": settle_ms,
            }
        )
    torque = np.abs(data["torque_actual"].astype(np.int64))
    metrics = {
        "samples": len(data),
        "moves": moves,
        "torque_saturation_pct": float(
            100.0 * np.count_nonzero(torque >= torque_limit) / max(len(data), 1)
        ),
        "ferr_crosscheck_max": int(
            np.max(np.abs(recomputed - ferr.astype(np.int64)))
        ),
    }
    if "velocity_offset" in (data.dtype.names or ()):
        moving = (data["flags"] & FLAG_MOTION_ACTIVE) != 0
        metrics["ff_velocity_offset_max"] = int(
            np.max(np.abs(data["velocity_offset"][moving]))
            if moving.any()
            else 0
        )
        metrics["ff_torque_offset_max"] = int(
            np.max(np.abs(data["torque_offset"][moving])) if moving.any() else 0
        )
    return metrics


def welch_psd(x, fs, nperseg=1024):
    x = np.asarray(x, dtype=np.float64)
    nperseg = min(nperseg, len(x))
    nperseg = 2 ** int(np.log2(nperseg))
    if nperseg < 64:
        raise SystemExit(
            "segment too short for PSD (%d samples; need >= 64)" % (len(x),)
        )
    step = nperseg // 2
    win = np.hanning(nperseg)
    scale = 1.0 / (fs * np.sum(win * win))
    psds = []
    for start in range(0, len(x) - nperseg + 1, step):
        seg = x[start : start + nperseg]
        seg = (seg - np.mean(seg)) * win
        spec = np.fft.rfft(seg)
        psds.append((spec.real**2 + spec.imag**2) * scale)
    psd = np.mean(psds, axis=0)
    psd[1:-1] *= 2.0
    return np.fft.rfftfreq(nperseg, 1.0 / fs), psd


def moving_psd(data, segs, fs):
    if not segs:
        raise SystemExit("no moving segments in capture — nothing to analyze")
    err = np.concatenate(
        [data["following_error"][s:e].astype(np.float64) for s, e in segs]
    )
    return welch_psd(err, fs)


def top_peaks(freqs, psd, count=5):
    local_max = (
        np.flatnonzero((psd[1:-1] > psd[:-2]) & (psd[1:-1] > psd[2:])) + 1
    )
    ranked = local_max[np.argsort(psd[local_max])[::-1]][:count]
    return [(float(freqs[i]), float(psd[i])) for i in ranked]


def _print_metrics(m, counts_per_mm):
    print("capture: %d samples, %d move(s)" % (m["samples"], len(m["moves"])))
    print(
        "torque saturation: %.1f%% of samples at/above limit"
        % (m["torque_saturation_pct"],)
    )
    print(
        "drive-vs-recomputed following error: max delta %d counts"
        % (m["ferr_crosscheck_max"],)
    )
    if "ff_velocity_offset_max" in m:
        print(
            "FF offsets during motion: velocity max %d counts/s (%.1f mm/s), "
            "torque max %d (0.1%% rated)"
            % (
                m["ff_velocity_offset_max"],
                m["ff_velocity_offset_max"] / counts_per_mm,
                m["ff_torque_offset_max"],
            )
        )
    for mv in m["moves"]:
        settle = (
            "%.1f ms" % mv["settle_ms"]
            if mv["settle_ms"] is not None
            else "NEVER"
        )
        print(
            "move %d [%.1f..%.1f ms]: ferr peak %.0f counts (%.4f mm), "
            "rms %.1f counts (%.4f mm), overshoot %.0f counts, settle %s"
            % (
                mv["move"],
                mv["start_ms"],
                mv["end_ms"],
                mv["ferr_peak"],
                mv["ferr_peak"] / counts_per_mm,
                mv["ferr_rms"],
                mv["ferr_rms"] / counts_per_mm,
                mv["overshoot"],
                settle,
            )
        )


def export_ident_csv(path, header, drive_datas):
    # The fitter regresses measured torque against the planner's exact commanded
    # acceleration (accel_cmd) and velocity (vel_cmd) — noise-free and
    # independent of any drive gain or inertia-ratio setting. Differentiating
    # the measured encoder trajectory instead couples the fit to C00.06 through
    # the closed-loop response (and yields negative inertia on a soft loop).
    # drive_datas: [(drive_idx, data)] views of ONE capture, so the per-record
    # prefix channels (cycle_index, flags) are identical across entries.
    for _, data in drive_datas:
        if "accel_cmd" not in (data.dtype.names or ()):
            raise SystemExit(
                "%s predates the commanded-kinematics channels (capture "
                "format v2); re-capture before fitting dynamics" % (path,)
            )
    first = drive_datas[0][1]
    axes = [header["drives"][idx]["name"] for idx, _ in drive_datas]
    cycle_index = first["cycle_index"].astype(np.int64)
    t = (cycle_index - cycle_index[0]) * (header["cycle_ns"] * 1e-9)
    moving = (first["flags"] & FLAG_MOTION_ACTIVE) != 0
    columns = [t[moving]]
    for idx, data in drive_datas:
        counts_per_mm = header["drives"][idx]["counts_per_mm"]
        columns.append(data["accel_cmd"].astype(np.float64)[moving])
        columns.append(data["vel_cmd"].astype(np.float64)[moving])
        columns.append(
            data["velocity_actual"].astype(np.float64)[moving] / counts_per_mm
        )
        columns.append(data["torque_actual"].astype(np.float64)[moving])
    rows = list(zip(*columns))
    fmt = "%.6f" + ",%.9g" * (len(columns) - 1) + "\n"
    with open(path, "w") as f:
        f.write(
            "t,"
            + ",".join(
                "accel_%s,vel_%s,vel_act_%s,torque_%s" % (a, a, a, a)
                for a in axes
            )
            + "\n"
        )
        for row in rows:
            f.write(fmt % row)
    print(
        "wrote %d motion samples for axes %s to %s (feed to servo-ident "
        "--capture)" % (len(rows), ",".join(axes), path)
    )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", nargs="?", help="path to a .scap capture file")
    p.add_argument(
        "--name",
        help="capture base name; analyzes the newest matching capture "
        "in --captures-dir instead of an explicit path",
    )
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument(
        "--drive",
        help="drive name to analyze in a multi-drive capture "
        "(default: the first drive in the file)",
    )
    p.add_argument(
        "--settle-band",
        type=int,
        default=50,
        help="settling band in encoder counts (default 50)",
    )
    p.add_argument(
        "--torque-limit",
        type=int,
        default=900,
        help="saturation threshold, per-mille of rated (default 900)",
    )
    p.add_argument(
        "--fft",
        action="store_true",
        help="print resonance peaks from the moving-segment PSD",
    )
    p.add_argument(
        "--plot",
        action="store_true",
        help="show a time-series dashboard (requires matplotlib)",
    )
    p.add_argument(
        "--csv",
        metavar="PATH",
        help="export t/target/torque in servo-ident's CSV contract "
        "(t in s, target in mm, torque in 0.1%% rated) and exit",
    )
    args = p.parse_args(argv)
    if (args.capture is None) == (args.name is None):
        raise SystemExit("pass a .scap path or --name, not both or neither")
    capture_path = args.capture or resolve_newest_capture(
        args.captures_dir, args.name
    )

    header, data, drive_idx = load_capture(capture_path, args.drive)
    fs = 1e9 / header["cycle_ns"]
    counts_per_mm = header["drives"][drive_idx]["counts_per_mm"]

    if args.csv:
        export_ident_csv(args.csv, header, [(drive_idx, data)])
        return 0

    print("file: %s" % (capture_path,))
    m = compute_metrics(data, args.settle_band, args.torque_limit, fs=fs)
    _print_metrics(m, counts_per_mm)

    if args.fft:
        freqs, psd = moving_psd(data, motion_segments(data["flags"]), fs)
        print("resonance peaks (notch-filter candidates):")
        for f_hz, power in top_peaks(freqs, psd):
            print("  %7.1f Hz  power %.3e" % (f_hz, power))

    if args.plot:
        _plot(header, data, fs, drive_idx)
    return 0


def _plot(header, data, fs, drive_idx=0):
    import matplotlib.pyplot as plt

    t = np.arange(len(data)) / fs
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    axes[0].plot(t, data["position_demand"], label="demand (6062h)")
    axes[0].plot(t, data["position_actual"], label="actual (6064h)")
    axes[0].plot(
        t,
        data["target_counts"],
        label="host target (607Ah)",
        linestyle="--",
        alpha=0.6,
    )
    axes[0].set_ylabel("counts")
    axes[0].legend(loc="upper right")
    axes[1].plot(t, data["following_error"], color="tab:red")
    axes[1].set_ylabel("following error (counts)")
    axes[2].plot(t, data["torque_actual"], color="tab:green")
    axes[2].set_ylabel("torque (per-mille)")
    axes[2].set_xlabel("time (s)")
    moving = (data["flags"] & FLAG_MOTION_ACTIVE) != 0
    for ax in axes:
        ax.fill_between(
            t, *ax.get_ylim(), where=moving, alpha=0.08, color="tab:blue"
        )
    fig.suptitle(
        header["drives"][drive_idx]["name"] + " — " + header["started_utc"]
    )
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
