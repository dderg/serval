#!/usr/bin/env python3
"""Analyze a servo telemetry capture (.scap) produced by SERVO_CAPTURE_START.

Prints per-drive following-error, overshoot/settling, and torque-saturation
metrics; --fft prints resonance peaks (notch-filter candidates); --plot opens a
time-series dashboard and --png saves one; --combine-corexy A,B with --axis adds
the CoreXY combined on-axis (A+B)/2 and cross-axis (A-B)/2 tracking traces.
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


def combine_corexy(a, b):
    """CoreXY motor->axis mix (COREXY_MOTOR_TO_AXIS): X=(A+B)/2, Y=(A-B)/2."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return 0.5 * (a + b), 0.5 * (a - b)


def _drive_view(drive_datas, name):
    for idx, dname, data in drive_datas:
        if dname == name:
            return idx, data
    raise SystemExit(
        "--combine-corexy motor %r not in capture (have: %s)"
        % (name, ", ".join(d[1] for d in drive_datas))
    )


def compute_corexy_combine(header, drive_datas, spec, axis):
    names = [t.strip() for t in spec.split(",")]
    if len(names) != 2:
        raise SystemExit(
            "--combine-corexy needs exactly two motor names (got %r)" % (spec,)
        )
    a_idx, a_data = _drive_view(drive_datas, names[0])
    b_idx, b_data = _drive_view(drive_datas, names[1])
    cpm_a = header["drives"][a_idx]["counts_per_mm"]
    cpm_b = header["drives"][b_idx]["counts_per_mm"]

    def mm(data, cpm, field):
        return data[field].astype(np.float64) / cpm

    x_ferr, y_ferr = combine_corexy(
        mm(a_data, cpm_a, "following_error"),
        mm(b_data, cpm_b, "following_error"),
    )
    x_act, y_act = combine_corexy(
        mm(a_data, cpm_a, "position_actual"),
        mm(b_data, cpm_b, "position_actual"),
    )
    x_tgt, y_tgt = combine_corexy(
        mm(a_data, cpm_a, "target_counts"),
        mm(b_data, cpm_b, "target_counts"),
    )
    axis = (axis or "X").upper()
    if axis == "Y":
        on_ferr, cross_ferr = y_ferr, x_ferr
        on_act, on_tgt = y_act, y_tgt
        on_label, cross_label = "Y", "X"
    else:
        on_ferr, cross_ferr = x_ferr, y_ferr
        on_act, on_tgt = x_act, x_tgt
        on_label, cross_label = "X", "Y"
    return {
        "axis": axis,
        "on_label": on_label,
        "cross_label": cross_label,
        "motors": (names[0], names[1]),
        "on_ferr": on_ferr,
        "cross_ferr": cross_ferr,
        "on_actual": on_act,
        "on_target": on_tgt,
        "moving": (a_data["flags"] & FLAG_MOTION_ACTIVE) != 0,
    }


def _print_combine(c):
    moving = c["moving"]
    on = c["on_ferr"][moving] if moving.any() else c["on_ferr"]
    cross = c["cross_ferr"][moving] if moving.any() else c["cross_ferr"]
    print(
        "combined %s-axis following error: peak %.4f mm, rms %.4f mm "
        "(motors %s+%s)"
        % (
            c["on_label"],
            float(np.max(np.abs(on))),
            float(np.sqrt(np.mean(on**2))),
            c["motors"][0],
            c["motors"][1],
        )
    )
    print(
        "cross-axis (%s) motor-skew error: peak %.4f mm"
        % (c["cross_label"], float(np.max(np.abs(cross))))
    )


def _shade_moving(ax, t, moving):
    ax.fill_between(
        t, *ax.get_ylim(), where=moving, alpha=0.08, color="tab:blue"
    )


def _figure_single(plt, header, data, fs, drive_idx):
    t = np.arange(len(data)) / fs
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
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
        _shade_moving(ax, t, moving)
    fig.suptitle(
        header["drives"][drive_idx]["name"] + " — " + header["started_utc"]
    )
    fig.tight_layout()
    return fig


def _figure_multi(plt, header, drive_datas, fs):
    first = drive_datas[0][2]
    t = np.arange(len(first)) / fs
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    for _, name, data in drive_datas:
        axes[0].plot(t, data["position_actual"], lw=0.8, label=name)
        axes[1].plot(t, data["following_error"], lw=0.8, label=name)
        axes[2].plot(t, data["torque_actual"], lw=0.8, label=name)
    axes[0].set_ylabel("position actual (counts)")
    axes[1].set_ylabel("following error (counts)")
    axes[2].set_ylabel("torque (per-mille)")
    axes[2].set_xlabel("time (s)")
    moving = (first["flags"] & FLAG_MOTION_ACTIVE) != 0
    for ax in axes:
        ax.legend(loc="upper right", fontsize=8)
        _shade_moving(ax, t, moving)
    fig.suptitle("per-motor tracking — " + header["started_utc"])
    fig.tight_layout()
    return fig


def _figure_combined(plt, header, drive_datas, fs, c):
    first = drive_datas[0][2]
    t = np.arange(len(first)) / fs
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    err_ax = axes[0, 0]
    err_ax.plot(
        t,
        c["on_ferr"],
        color="tab:red",
        lw=1.2,
        label="%s on-axis" % c["on_label"],
    )
    err_ax.plot(
        t,
        c["cross_ferr"],
        color="tab:orange",
        lw=0.7,
        alpha=0.8,
        label="%s cross-axis" % c["cross_label"],
    )
    err_ax.set_ylabel("following error (mm)")
    err_ax.set_title("Combined axis tracking error")
    err_ax.legend(loc="upper right", fontsize=8)

    fe_ax = axes[0, 1]
    for _, name, data in drive_datas:
        fe_ax.plot(t, data["following_error"], lw=0.8, label=name)
    fe_ax.set_ylabel("following error (counts)")
    fe_ax.set_title("Per-motor following error")
    fe_ax.legend(loc="upper right", fontsize=8)

    tq_ax = axes[1, 0]
    for _, name, data in drive_datas:
        tq_ax.plot(t, data["torque_actual"], lw=0.8, label=name)
    tq_ax.set_ylabel("torque (per-mille)")
    tq_ax.set_xlabel("time (s)")
    tq_ax.set_title("Per-motor torque")
    tq_ax.legend(loc="upper right", fontsize=8)

    pos_ax = axes[1, 1]
    pos_ax.plot(t, c["on_target"], color="tab:gray", ls="--", label="target")
    pos_ax.plot(t, c["on_actual"], color="tab:blue", lw=0.9, label="actual")
    pos_ax.set_ylabel("%s position (mm)" % c["on_label"])
    pos_ax.set_xlabel("time (s)")
    pos_ax.set_title("Combined axis position")
    pos_ax.legend(loc="upper right", fontsize=8)

    for ax in axes.flat:
        _shade_moving(ax, t, c["moving"])
    fig.suptitle(
        "%s-axis tracking (motors %s+%s) — %s"
        % (c["axis"], c["motors"][0], c["motors"][1], header["started_utc"])
    )
    fig.tight_layout()
    return fig


def build_tracking_figure(header, drive_datas, fs, combine=None):
    import matplotlib.pyplot as plt

    if combine is not None:
        return _figure_combined(plt, header, drive_datas, fs, combine)
    if len(drive_datas) > 1:
        return _figure_multi(plt, header, drive_datas, fs)
    idx, _, data = drive_datas[0]
    return _figure_single(plt, header, data, fs, idx)


def _png_path(capture_path, plot_out, plot_dir):
    if plot_out:
        out = os.path.expanduser(plot_out)
    else:
        base = os.path.splitext(os.path.basename(capture_path))[0]
        out = os.path.join(os.path.expanduser(plot_dir), base + ".png")
    parent = os.path.dirname(out)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return out


def save_tracking_png(header, drive_datas, fs, combine, out_path):
    import matplotlib

    matplotlib.use("Agg")
    fig = build_tracking_figure(header, drive_datas, fs, combine)
    fig.savefig(out_path, dpi=110)


def _load_all_drives(capture_path, only_drive):
    header, _, _ = load_capture(capture_path, only_drive)
    names = (
        [only_drive] if only_drive else [d["name"] for d in header["drives"]]
    )
    drive_datas = []
    for name in names:
        _, data, idx = load_capture(capture_path, name)
        drive_datas.append((idx, name, data))
    return header, drive_datas


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
        help="restrict analysis to one drive in a multi-drive capture "
        "(default: every drive in the file)",
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
        "--png",
        action="store_true",
        help="render and save a tracking dashboard PNG (headless)",
    )
    p.add_argument(
        "--plot-dir",
        default="~/printer_data/config/servo_calibrate_results",
        help="directory for the --png output",
    )
    p.add_argument(
        "--plot-out",
        metavar="PATH",
        help="explicit PNG path (overrides --plot-dir); implies --png",
    )
    p.add_argument(
        "--combine-corexy",
        metavar="A,B",
        help="two motor names; adds CoreXY combined (A+B)/2 on-axis and "
        "(A-B)/2 cross-axis traces",
    )
    p.add_argument(
        "--axis",
        help="axis being measured (X/Y), selects the on-axis for --combine-corexy",
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

    if args.csv:
        header, data, drive_idx = load_capture(capture_path, args.drive)
        export_ident_csv(args.csv, header, [(drive_idx, data)])
        return 0

    header, drive_datas = _load_all_drives(capture_path, args.drive)
    fs = 1e9 / header["cycle_ns"]

    print("file: %s" % (capture_path,))
    for idx, name, data in drive_datas:
        if len(drive_datas) > 1:
            print("drive: %s" % (name,))
        counts_per_mm = header["drives"][idx]["counts_per_mm"]
        m = compute_metrics(data, args.settle_band, args.torque_limit, fs=fs)
        _print_metrics(m, counts_per_mm)
        if args.fft:
            freqs, psd = moving_psd(data, motion_segments(data["flags"]), fs)
            print("resonance peaks (notch-filter candidates):")
            for f_hz, power in top_peaks(freqs, psd):
                print("  %7.1f Hz  power %.3e" % (f_hz, power))

    combine = None
    if args.combine_corexy:
        combine = compute_corexy_combine(
            header, drive_datas, args.combine_corexy, args.axis
        )
        _print_combine(combine)

    if args.png or args.plot_out:
        out_path = _png_path(capture_path, args.plot_out, args.plot_dir)
        save_tracking_png(header, drive_datas, fs, combine, out_path)
        print("report: %s" % (out_path,))

    if args.plot:
        import matplotlib.pyplot as plt

        build_tracking_figure(header, drive_datas, fs, combine)
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
