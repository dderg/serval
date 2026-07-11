#!/usr/bin/env python3
"""Differential (rotor-vs-rotor) FRF report for an AWD belt pair.

SERVO_MEASURE_DIFFERENTIAL drives the two motors of one belt with an
anti-phase position chirp (the engine buzz generator with opposing slot
signs): the carriage holds still while the drives strain the belt against
each other. This script turns that capture into the differential frequency
response. The excitation is the differential commanded position
(target_counts), the response is the differential encoder position, and the
H1 Welch estimate of response over excitation exposes exactly the
inter-motor modes - the resonances excited when paired drives fight. It
prints the detected modes (frequency, closed-loop peak gain, half-power
damping ratio) and renders a PNG with magnitude, phase, coherence and
differential-torque spectrum panels.

Usage:
  servo_diff_report.py --name diff --pair motor_a:1+motor_a1:1 --png
  servo_diff_report.py capture.scap --pair "motor_b:-1+motor_b1:1" --out d.png
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_capture import (  # noqa: E402
    _parse_combine_spec,
    load_capture,
    resolve_newest_capture,
)

MIN_NPERSEG = 256
MIN_SEGMENTS = 4
COHERENCE_MIN = 0.5


def parse_pair(spec):
    belts = _parse_combine_spec(spec)
    if len(belts) != 1 or len(belts[0]) != 2:
        raise SystemExit(
            "--pair needs exactly one belt of two motors, e.g. "
            "'motor_a:1+motor_a1:1' (got %r)" % (spec,)
        )
    return belts[0]


def load_pair(path, terms):
    per_motor = []
    fs = None
    for name, sign in terms:
        header, data, idx = load_capture(path, name)
        cpm = float(header["drives"][idx]["counts_per_mm"])
        fs = 1.0e9 / float(header["cycle_ns"])
        per_motor.append(
            {
                "cmd_mm": sign * data["target_counts"].astype(np.float64) / cpm,
                "act_mm": sign
                * data["position_actual"].astype(np.float64)
                / cpm,
                "torque": sign * data["torque_actual"].astype(np.float64),
            }
        )
    a, b = per_motor
    n = min(len(a["cmd_mm"]), len(b["cmd_mm"]))
    return fs, {
        "cmd": a["cmd_mm"][:n] - b["cmd_mm"][:n],
        "act": a["act_mm"][:n] - b["act_mm"][:n],
        "torque": a["torque"][:n] - b["torque"][:n],
    }


def active_slice(cmd, threshold_frac=0.05):
    dev = np.abs(cmd - np.median(cmd))
    peak = dev.max() if len(dev) else 0.0
    if peak <= 0.0:
        raise SystemExit(
            "capture holds no differential excitation (differential command "
            "is flat); was the anti-phase buzz armed on this pair?"
        )
    idx = np.nonzero(dev > threshold_frac * peak)[0]
    return slice(int(idx[0]), int(idx[-1]) + 1)


def welch_segment_length(n, nperseg):
    while nperseg * (MIN_SEGMENTS + 1) // 2 > n and nperseg > MIN_NPERSEG:
        nperseg //= 2
    if nperseg < MIN_NPERSEG or n < nperseg * (MIN_SEGMENTS + 1) // 2:
        raise SystemExit(
            "capture too short for a Welch FRF: %d active samples but "
            "%d segments of %d are needed; sweep longer or slower"
            % (n, MIN_SEGMENTS, MIN_NPERSEG)
        )
    return nperseg


def welch_frf(x, y, fs, nperseg):
    nperseg = welch_segment_length(len(x), nperseg)
    step = nperseg // 2
    win = np.hanning(nperseg)
    pxx = np.zeros(nperseg // 2 + 1)
    pyy = np.zeros(nperseg // 2 + 1)
    pxy = np.zeros(nperseg // 2 + 1, dtype=np.complex128)
    segments = 0
    for start in range(0, len(x) - nperseg + 1, step):
        xs = x[start : start + nperseg]
        ys = y[start : start + nperseg]
        fx = np.fft.rfft((xs - xs.mean()) * win)
        fy = np.fft.rfft((ys - ys.mean()) * win)
        pxx += np.abs(fx) ** 2
        pyy += np.abs(fy) ** 2
        pxy += np.conj(fx) * fy
        segments += 1
    freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
    nonzero = pxx > 0.0
    frf = np.zeros_like(pxy)
    frf[nonzero] = pxy[nonzero] / pxx[nonzero]
    denom = pxx * pyy
    coherence = np.zeros_like(pxx)
    coherence[denom > 0.0] = np.abs(pxy[denom > 0.0]) ** 2 / denom[denom > 0.0]
    return freqs, frf, coherence, segments


def _half_power_crossing(freqs, mag, i, target, direction):
    j = i
    while 0 < j < len(mag) - 1 and mag[j] > target:
        j += direction
    if mag[j] > target:
        return None
    f0, f1 = freqs[j - direction], freqs[j]
    m0, m1 = mag[j - direction], mag[j]
    if m0 == m1:
        return float(f1)
    return float(f0 + (f1 - f0) * (m0 - target) / (m0 - m1))


def half_power_damping(freqs, mag, i_peak):
    target = mag[i_peak] / np.sqrt(2.0)
    lo = _half_power_crossing(freqs, mag, i_peak, target, -1)
    hi = _half_power_crossing(freqs, mag, i_peak, target, +1)
    if lo is None or hi is None or freqs[i_peak] <= 0.0:
        return None
    return (hi - lo) / (2.0 * freqs[i_peak])


def find_modes(freqs, frf, coherence, lo, hi, max_modes=5):
    band = (freqs >= lo) & (freqs <= hi)
    if not np.any(band & (coherence >= COHERENCE_MIN)):
        raise SystemExit(
            "no coherent differential response in %.0f..%.0f Hz "
            "(max coherence %.2f); raise AMPLITUDE or check that the "
            "buzz really ran anti-phase on this pair"
            % (lo, hi, float(coherence[band].max()) if np.any(band) else 0.0)
        )
    mag = np.abs(frf)
    candidates = []
    idx = np.nonzero(band)[0]
    for i in idx[1:-1]:
        if not (mag[i] > mag[i - 1] and mag[i] >= mag[i + 1]):
            continue
        if coherence[i] < COHERENCE_MIN:
            continue
        candidates.append(i)
    candidates.sort(key=lambda i: -mag[i])
    modes = []
    for i in candidates:
        if any(
            abs(freqs[i] - m["freq_hz"]) < max(3.0, 0.05 * m["freq_hz"])
            for m in modes
        ):
            continue
        modes.append(
            {
                "freq_hz": float(freqs[i]),
                "gain": float(mag[i]),
                "gain_db": float(20.0 * np.log10(mag[i])),
                "damping": half_power_damping(freqs, mag, i),
                "coherence": float(coherence[i]),
            }
        )
        if len(modes) >= max_modes:
            break
    modes.sort(key=lambda m: m["freq_hz"])
    return modes


def analyze(path, terms, freq_start, freq_end, nperseg):
    fs, diff = load_pair(path, terms)
    span = active_slice(diff["cmd"])
    freqs, frf, coherence, segments = welch_frf(
        diff["cmd"][span], diff["act"][span], fs, nperseg
    )
    _, torque_frf, _, _ = welch_frf(
        diff["cmd"][span], diff["torque"][span], fs, nperseg
    )
    modes = find_modes(freqs, frf, coherence, freq_start, freq_end)
    return {
        "fs": fs,
        "segments": segments,
        "freqs": freqs,
        "frf": frf,
        "coherence": coherence,
        "torque_frf": torque_frf,
        "modes": modes,
        "diff": diff,
        "span": span,
    }


def print_modes(modes, pair_label):
    print("differential modes (%s):" % (pair_label,))
    print("  freq      |H| peak      damping    coherence")
    for m in modes:
        damping = "%.4f" % (m["damping"],) if m["damping"] is not None else "-"
        print(
            "  %7.1f Hz  %6.2f dB     %8s     %.2f"
            % (m["freq_hz"], m["gain_db"], damping, m["coherence"])
        )


def save_png(result, out_path, pair_label, freq_start, freq_end):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    freqs = result["freqs"]
    band = (freqs >= max(freq_start * 0.5, freqs[1])) & (
        freqs <= freq_end * 1.2
    )
    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    mag_db = 20.0 * np.log10(np.maximum(np.abs(result["frf"]), 1e-12))
    axes[0].plot(freqs[band], mag_db[band])
    for m in result["modes"]:
        label = "%.1f Hz" % (m["freq_hz"],)
        if m["damping"] is not None:
            label += " z=%.3f" % (m["damping"],)
        axes[0].axvline(m["freq_hz"], color="r", alpha=0.4, linestyle="--")
        axes[0].annotate(
            label,
            (m["freq_hz"], m["gain_db"]),
            textcoords="offset points",
            xytext=(4, 6),
            fontsize=8,
        )
    axes[0].set_ylabel("|diff act / diff cmd| (dB)")
    axes[0].set_title(
        "Differential FRF %s (%d Welch segments)"
        % (pair_label, result["segments"])
    )
    axes[1].plot(freqs[band], np.degrees(np.angle(result["frf"][band])))
    axes[1].set_ylabel("phase (deg)")
    axes[2].plot(freqs[band], result["coherence"][band])
    axes[2].axhline(COHERENCE_MIN, color="r", alpha=0.4, linestyle="--")
    axes[2].set_ylabel("coherence")
    axes[2].set_ylim(0.0, 1.05)
    torque_db = 20.0 * np.log10(np.maximum(np.abs(result["torque_frf"]), 1e-12))
    axes[3].plot(freqs[band], torque_db[band])
    axes[3].set_ylabel("|diff torque / diff cmd| (dB)")
    axes[3].set_xlabel("frequency (Hz)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print("differential FRF plot written to %s" % (out_path,))


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
        "--pair",
        required=True,
        help="the belt pair as 'name[:sign]+name[:sign]'; sign is -1 for a "
        "motor whose servo invert_direction flips its encoder counts out "
        "of the kinematic frame",
    )
    p.add_argument("--freq-start", type=float, default=10.0)
    p.add_argument("--freq-end", type=float, default=300.0)
    p.add_argument(
        "--nperseg",
        type=int,
        default=4096,
        help="Welch segment length (auto-shrinks on short captures)",
    )
    p.add_argument("--png", action="store_true", help="save the FRF plot")
    p.add_argument(
        "--plot-dir",
        default="~/printer_data/config/servo_calibrate_results",
        help="directory for the --png output",
    )
    p.add_argument(
        "--out", help="explicit PNG path (overrides --plot-dir); implies --png"
    )
    args = p.parse_args(argv)
    if bool(args.capture) == bool(args.name):
        raise SystemExit("pass a capture path or --name, not both or neither")
    path = args.capture or resolve_newest_capture(args.captures_dir, args.name)
    terms = parse_pair(args.pair)
    result = analyze(path, terms, args.freq_start, args.freq_end, args.nperseg)
    pair_label = " vs ".join(name for name, _sign in terms)
    print_modes(result["modes"], pair_label)
    if args.png or args.out:
        base = os.path.splitext(os.path.basename(path))[0]
        out_path = args.out or os.path.join(
            os.path.expanduser(args.plot_dir), base + "_diff.png"
        )
        save_png(result, out_path, pair_label, args.freq_start, args.freq_end)


if __name__ == "__main__":
    main()
