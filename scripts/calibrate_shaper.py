#!/usr/bin/env python3
# Shaper auto-calibration script
#
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
# Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import print_function

import math
import optparse
import pathlib
import sys
from textwrap import wrap

import matplotlib
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

from klippy.extras import shaper_calibrate

MAX_TITLE_LENGTH = 65

CHIRP_KEYS = (
    "freq_start",
    "freq_end",
    "duration",
    "ramp",
    "accel_per_hz",
    "amplitude_mm",
)
CHIRP_REQUIRED = ("freq_start", "freq_end", "duration", "accel_per_hz")
DEFAULT_CHIRP_BW = 4.0
HARMONIC_ORDERS = (2, 3)
OFFTRACK_RATIO = 1.31
NYQUIST_FRACTION = 0.9
FRF_BIN_HZ = 0.2
T0_SEARCH_RANGE = (-1.0, 3.0)
SNR_WARN_LEVEL = 3.0
REACHED_TOOLHEAD_MIN_H = 1.0
MIN_FRF_SAMPLES = 32


class ChirpBandTooShort(ValueError):
    pass


def parse_log(logname):
    with open(logname) as f:
        for header in f:
            if not header.startswith("#"):
                break
        if not header.startswith("freq,psd_x,psd_y,psd_z,psd_xyz,accel_per_hz"):
            # Raw accelerometer data
            return np.loadtxt(logname, comments="#", delimiter=",")
    # Parse power spectral density data
    data = np.loadtxt(logname, skiprows=1, comments="#", delimiter=",")
    calibration_data = shaper_calibrate.CalibrationData(
        freq_bins=data[:, 0],
        psd_sum=data[:, 4],
        psd_x=data[:, 1],
        psd_y=data[:, 2],
        psd_z=data[:, 3],
    )
    calibration_data.set_numpy(np)
    # If input shapers are present in the CSV file, the frequency
    # response is already normalized to input frequencies
    if "mzv" not in header:
        calibration_data.normalize_to_frequencies()
    return calibration_data


def parse_accel_per_hz(logname):
    with open(logname) as f:
        for header in f:
            if not header.startswith("#"):
                break
        if not header.startswith("freq,psd_x,psd_y,psd_z,psd_xyz,accel_per_hz"):
            return None  # TODO

    data = np.loadtxt(
        logname, skiprows=1, comments="#", delimiter=",", max_rows=2
    )
    return data[0][5].item()


######################################################################
# Swept-sine (chirp) demodulated frequency response
######################################################################


def _chirp_from_pairs(pairs):
    config = {}
    for pair in pairs:
        pair = pair.strip()
        if not pair:
            continue
        key, sep, value = pair.partition("=")
        if not sep:
            raise ValueError("malformed chirp field %r" % (pair,))
        config[key.strip()] = float(value.strip())
    return config


def parse_chirp_header(logname):
    with open(logname) as f:
        for line in f:
            if not line.startswith("#"):
                break
            body = line[1:].strip()
            if body.startswith("chirp "):
                return _chirp_from_pairs(body[len("chirp ") :].split())
    return None


def resolve_chirp_config(logname, chirp_option):
    config = parse_chirp_header(logname) or {}
    if chirp_option:
        config.update(_chirp_from_pairs(chirp_option.split(",")))
    if not config:
        return None
    missing = [key for key in CHIRP_REQUIRED if key not in config]
    if missing:
        raise ValueError(
            "chirp config missing required keys: %s" % (", ".join(missing))
        )
    config.setdefault("ramp", 0.0)
    config.setdefault("amplitude_mm", 0.0)
    return config


def chirp_aph_eff(config):
    if config["amplitude_mm"] == 0.0:
        return config["accel_per_hz"]
    return config["amplitude_mm"] * 4.0 * math.pi**2 * config["freq_start"]


def chirp_sweep_rate(config):
    return (config["freq_end"] - config["freq_start"]) / config["duration"]


def resample_uniform(data):
    t = data[:, 0]
    dt = float(np.median(np.diff(t)))
    n = int(math.floor((t[-1] - t[0]) / dt)) + 1
    t_uniform = np.arange(n) * dt
    axes = []
    for column in (1, 2, 3):
        signal = np.interp(t_uniform, t - t[0], data[:, column])
        axes.append(signal - signal.mean())
    return t_uniform, dt, 1.0 / dt, np.array(axes)


def _moving_average(x, width):
    w = max(1, int(round(width)))
    if w <= 1:
        return x.copy()
    cumulative = np.cumsum(np.insert(x, 0, 0.0))
    n = len(x)
    idx = np.arange(n)
    half = w // 2
    lo = np.clip(idx - half, 0, n)
    hi = np.clip(idx - half + w, 0, n)
    return (cumulative[hi] - cumulative[lo]) / (hi - lo)


def _demod_phase(freq_track, dt):
    return (
        2.0
        * np.pi
        * dt
        * (np.cumsum(freq_track) - 0.5 * (freq_track + freq_track[0]))
    )


def _triangular_lowpass(x, width):
    return _moving_average(_moving_average(x, width), width)


def tracked_amplitude(signal, carrier, width):
    iq = signal * carrier
    lp = _triangular_lowpass(iq.real, width) + 1j * _triangular_lowpass(
        iq.imag, width
    )
    return 2.0 * np.abs(lp)


def sweep_freq_track(t, t0, config):
    return config["freq_start"] + chirp_sweep_rate(config) * (t - t0)


def sweep_mask(t, t0, config):
    return (t >= t0) & (t <= t0 + config["duration"])


def estimate_t0(t, dt, fs, axes, config, bw):
    """A signal chirping from a start offset t0 differs from the t0=0
    reference demodulation by a pure tone at -slope*t0 Hz, so the whole t0
    scan is one FFT of the reference-demodulated capture: the strongest
    spectral line inside the search window IS the sweep offset."""
    slope = chirp_sweep_rate(config)
    reference = sweep_freq_track(t, 0.0, config)
    carrier = np.exp(-1j * _demod_phase(reference, dt))
    power = np.zeros(len(t))
    for axis in axes:
        power += np.abs(np.fft.fft(axis * carrier)) ** 2
    power = np.fft.fftshift(power)
    tone_freqs = np.fft.fftshift(np.fft.fftfreq(len(t), dt))
    t0_candidates = -tone_freqs / slope
    window = (t0_candidates >= T0_SEARCH_RANGE[0]) & (
        t0_candidates <= T0_SEARCH_RANGE[1]
    )
    if slope < 0:
        window = window[::-1]
        power = power[::-1]
        t0_candidates = t0_candidates[::-1]
    idx = np.flatnonzero(window)
    peak = idx[int(np.argmax(power[idx]))]
    at_boundary = peak in (idx[0], idx[-1])
    flat = bool(power[peak] < 4.0 * np.median(power[idx]))
    t0 = t0_candidates[peak]
    if idx[0] < peak < idx[-1]:
        pm, p0, pp = power[peak - 1], power[peak], power[peak + 1]
        curvature = pm - 2.0 * p0 + pp
        if curvature < 0:
            shift = 0.5 * (pm - pp) / curvature
            bin_step = t0_candidates[peak + 1] - t0_candidates[peak]
            t0 += shift * bin_step
    return float(t0), at_boundary, flat


class ChirpResponse:
    def __init__(
        self,
        config,
        t0,
        fs,
        aph_eff,
        f0,
        transmissibility,
        harmonics,
        snr,
        median_snr,
        peak_transmissibility,
        calibration_data,
        t0_weakly_constrained,
    ):
        self.config = config
        self.t0 = t0
        self.fs = fs
        self.aph_eff = aph_eff
        self.f0 = f0
        self.transmissibility = transmissibility
        self.harmonics = harmonics
        self.snr = snr
        self.median_snr = median_snr
        self.peak_transmissibility = peak_transmissibility
        self.calibration_data = calibration_data
        self.t0_weakly_constrained = t0_weakly_constrained

    def excitation_reached_toolhead(self):
        return self.peak_transmissibility >= REACHED_TOOLHEAD_MIN_H


def analyze_chirp(data, config, bw):
    if chirp_sweep_rate(config) == 0.0:
        raise ChirpBandTooShort(
            "sweep has zero frequency span; nothing to demodulate"
        )
    t, dt, fs, axes = resample_uniform(data)
    nyquist = fs / 2.0
    t0, at_boundary, flat = estimate_t0(t, dt, fs, axes, config, bw)
    mask = sweep_mask(t, t0, config)
    f0_full = sweep_freq_track(t[mask], t0, config)
    trim = 2 * int(fs / bw)
    keep = slice(trim, len(f0_full) - trim)
    f0 = f0_full[keep]
    if len(f0) < MIN_FRF_SAMPLES:
        raise ChirpBandTooShort(
            "only %.2f s of sweep remain after the %.2f s lock-in filter "
            "settling on each side; sweep longer than %.1f s or raise "
            "--chirp-bw" % (len(f0) / fs, trim / fs, 3.0 * trim / fs)
        )
    aph_eff = chirp_aph_eff(config)
    width = fs / bw
    carrier = np.exp(-1j * _demod_phase(f0_full, dt))
    fundamental = [
        tracked_amplitude(axis[mask], carrier, width)[keep] for axis in axes
    ]
    transmissibility = [amp / (aph_eff * f0) for amp in fundamental]
    fundamental_mag = np.sqrt(sum(amp**2 for amp in fundamental))
    harmonics = {}
    harmonic_carrier = carrier
    harmonic_exponent = 1
    for order in sorted(HARMONIC_ORDERS):
        while harmonic_exponent < order:
            harmonic_carrier = harmonic_carrier * carrier
            harmonic_exponent += 1
        valid = (order * f0) < NYQUIST_FRACTION * nyquist
        if valid.sum() < 10:
            continue
        harmonic_mag = np.sqrt(
            sum(
                tracked_amplitude(axis[mask], harmonic_carrier, width)[keep]
                ** 2
                for axis in axes
            )
        )
        harmonics[order] = np.where(
            valid, harmonic_mag / np.maximum(fundamental_mag, 1e-9), np.nan
        )
    offtrack = np.minimum(OFFTRACK_RATIO * f0_full, NYQUIST_FRACTION * nyquist)
    offtrack_carrier = np.exp(-1j * _demod_phase(offtrack, dt))
    noise_mag = np.sqrt(
        sum(
            tracked_amplitude(axis[mask], offtrack_carrier, width)[keep] ** 2
            for axis in axes
        )
    )
    snr = fundamental_mag / np.maximum(noise_mag, 1e-12)
    grid = np.arange(f0.min(), f0.max(), FRF_BIN_HZ)
    binned = [np.interp(grid, f0, h) for h in transmissibility]
    psd = [h**2 for h in binned]
    calibration_data = shaper_calibrate.CalibrationData(
        freq_bins=grid,
        psd_sum=psd[0] + psd[1] + psd[2],
        psd_x=psd[0],
        psd_y=psd[1],
        psd_z=psd[2],
    )
    calibration_data.set_numpy(np)
    peak_transmissibility = float(np.max(np.sqrt(psd[0] + psd[1] + psd[2])))
    return ChirpResponse(
        config=config,
        t0=t0,
        fs=fs,
        aph_eff=aph_eff,
        f0=f0,
        transmissibility=transmissibility,
        harmonics=harmonics,
        snr=snr,
        median_snr=float(np.median(snr)),
        peak_transmissibility=peak_transmissibility,
        calibration_data=calibration_data,
        t0_weakly_constrained=at_boundary or flat,
    )


def fit_second_order_mode(freqs, magnitude):
    """Least-squares fit of g * |1/(1 - r^2 + 2j*zeta*r)|, r = f/fn, to the
    measured transmissibility. The (fn, zeta) pair parameterizes the
    mode_inverse post-processor; g absorbs drive tracking gain."""
    fn_grid = np.linspace(freqs.min(), 1.6 * freqs.max(), 240)
    zeta_grid = np.geomspace(0.02, 0.9, 60)
    h_norm = float(np.sum(magnitude**2))
    best = (math.inf, 0.0, 0.0, 0.0)
    for fn in fn_grid:
        r = freqs / fn
        model = 1.0 / np.sqrt(
            ((1.0 - r**2) ** 2)[None, :]
            + (2.0 * zeta_grid[:, None] * r[None, :]) ** 2
        )
        mh = model @ magnitude
        mm = np.maximum((model**2).sum(axis=1), 1e-12)
        residual = h_norm - mh**2 / mm
        k = int(np.argmin(residual))
        if residual[k] < best[0]:
            best = (
                float(residual[k]),
                float(fn),
                float(zeta_grid[k]),
                float(mh[k] / mm[k]),
            )
    res, fn, zeta, gain = best
    rel_err = math.sqrt(max(res, 0.0) / h_norm)
    return fn, zeta, gain, rel_err


def second_order_magnitude(freqs, fn, zeta, gain):
    r = freqs / fn
    return gain / np.sqrt((1.0 - r**2) ** 2 + (2.0 * zeta * r) ** 2)


def calc_specgram(data, axis):
    N = data.shape[0]
    Fs = N / (data[-1, 0] - data[0, 0])
    M = 1 << int(0.5 * Fs - 1).bit_length()
    window = np.kaiser(M, 6.0)

    def _specgram(x):
        return matplotlib.mlab.specgram(
            x,
            Fs=Fs,
            NFFT=M,
            noverlap=M // 2,
            window=window,
            mode="psd",
            detrend="mean",
            scale_by_freq=False,
        )

    d = {"x": data[:, 1], "y": data[:, 2], "z": data[:, 3]}
    if axis != "all":
        pdata, bins, t = _specgram(d[axis])
    else:
        pdata, bins, t = _specgram(d["x"])
        for ax in "yz":
            pdata += _specgram(d[ax])[0]
    return pdata, bins, t


######################################################################
# Shaper calibration
######################################################################


# Find the best shaper parameters
def calibrate_shaper(
    datas,
    csv_output,
    *,
    shapers,
    damping_ratio,
    scv,
    shaper_freqs,
    max_smoothing,
    test_damping_ratios,
    max_freq,
):
    helper = shaper_calibrate.ShaperCalibrate(printer=None)
    if isinstance(datas[0], shaper_calibrate.CalibrationData):
        calibration_data = datas[0]
        for data in datas[1:]:
            calibration_data.add_data(data)
    else:
        # Process accelerometer data
        calibration_data = helper.process_accelerometer_data(datas[0])
        for data in datas[1:]:
            calibration_data.add_data(helper.process_accelerometer_data(data))
        calibration_data.normalize_to_frequencies()

    shaper, all_shapers = helper.find_best_shaper(
        calibration_data,
        shapers=shapers,
        damping_ratio=damping_ratio,
        scv=scv,
        shaper_freqs=shaper_freqs,
        max_smoothing=max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
        logger=print,
    )
    if not shaper:
        print(
            "No recommended shaper, possibly invalid value for --shapers=%s"
            % (",".join(shapers))
        )
        return None, None, None
    print("Recommended shaper is %s @ %.1f Hz" % (shaper.name, shaper.freq))
    if csv_output is not None:
        helper.save_calibration_data(csv_output, calibration_data, all_shapers)
    return shaper.name, all_shapers, calibration_data


######################################################################
# Plot frequency response and suggested input shapers
######################################################################


def plot_freq_response(
    lognames,
    calibration_data,
    shapers,
    selected_shaper,
    max_freq,
    accels_per_hz,
    raw_data=None,
):
    if raw_data is not None:
        fig, (ax, ax_spec) = matplotlib.pyplot.subplots(
            nrows=2,
            sharex=True,
            gridspec_kw={"height_ratios": [3, 1]},
        )
    else:
        fig, ax = matplotlib.pyplot.subplots()
        ax_spec = None
    draw_freq_response(
        ax,
        ax_spec,
        lognames,
        calibration_data,
        shapers,
        selected_shaper,
        max_freq,
        accels_per_hz,
        raw_data,
    )
    fig.tight_layout()
    return fig


def draw_freq_response(
    ax,
    ax_spec,
    lognames,
    calibration_data,
    shapers,
    selected_shaper,
    max_freq,
    accels_per_hz,
    raw_data,
):
    all_freqs = calibration_data.freq_bins
    psd = calibration_data.psd_sum[all_freqs <= max_freq]
    px = calibration_data.psd_x[all_freqs <= max_freq]
    py = calibration_data.psd_y[all_freqs <= max_freq]
    pz = calibration_data.psd_z[all_freqs <= max_freq]
    freqs = all_freqs[all_freqs <= max_freq]

    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size("x-small")

    ax.set_xlim([0, max_freq])
    ax.set_ylabel("Power spectral density")
    if ax_spec is None:
        ax.set_xlabel("Frequency, Hz")

    ax.plot(freqs, psd, label="X+Y+Z", color="purple")
    ax.plot(freqs, px, label="X", color="red")
    ax.plot(freqs, py, label="Y", color="green")
    ax.plot(freqs, pz, label="Z", color="blue")

    title = "Frequency response and shapers (%s)" % (", ".join(lognames))
    ax.set_title("\n".join(wrap(title, MAX_TITLE_LENGTH)))
    ax.xaxis.set_minor_locator(matplotlib.ticker.MultipleLocator(5))
    ax.yaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())
    ax.ticklabel_format(axis="y", style="scientific", scilimits=(0, 0))
    ax.grid(which="major", color="grey")
    ax.grid(which="minor", color="lightgrey")

    ax2 = ax.twinx()
    ax2.set_ylabel("Shaper vibration reduction (ratio)")
    best_shaper_vals = None
    for shaper in shapers:
        label = "%s (%.1f Hz, vibr=%.1f%%, sm~=%.2f, accel<=%.f)" % (
            shaper.name.upper(),
            shaper.freq,
            shaper.vibrs * 100.0,
            shaper.smoothing,
            round(shaper.max_accel / 100.0) * 100.0,
        )
        linestyle = "dotted"
        if shaper.name == selected_shaper:
            linestyle = "dashdot"
            best_shaper_vals = shaper.vals
        fit_freqs = all_freqs[: len(shaper.vals)]
        fit_band = fit_freqs <= max_freq
        ax2.plot(
            fit_freqs[fit_band],
            shaper.vals[fit_band],
            label=label,
            linestyle=linestyle,
        )
    if best_shaper_vals is not None:
        shaped = min(len(freqs), len(best_shaper_vals))
        ax.plot(
            freqs[:shaped],
            psd[:shaped] * best_shaper_vals[:shaped],
            label="After\nshaper",
            color="cyan",
        )
    # A hack to add a human-readable shaper recommendation to legend
    ax2.plot(
        [],
        [],
        " ",
        label="Recommended shaper: %s" % (str(selected_shaper).upper()),
    )

    ax2.plot(
        [],
        [],
        " ",
        label="accels_per_hz: %s" % (", ".join(str(e) for e in accels_per_hz)),
    )

    ax.legend(loc="upper left", prop=fontP)
    ax2.legend(loc="upper right", prop=fontP)

    if ax_spec is not None:
        pdata, bins, t = calc_specgram(raw_data, "all")
        ax_spec.pcolormesh(
            bins,
            t,
            pdata.T,
            norm=matplotlib.colors.LogNorm(),
            cmap="inferno",
            shading="auto",
        )
        ax_spec.set_xlim([0, max_freq])
        ax_spec.set_ylabel("Time, s")
        ax_spec.set_xlabel("Frequency, Hz")


######################################################################
# Plot demodulated chirp frequency response
######################################################################


def draw_chirp_frf(
    ax_h,
    ax_harm,
    ax_snr,
    lognames,
    response,
    shapers,
    selected_shaper,
    classic_data,
    classic_shaper,
    max_freq,
    mode_fit=None,
):
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size("x-small")

    grid = response.calibration_data.freq_bins
    band = grid <= max_freq
    hx = response.calibration_data.psd_x[band] ** 0.5
    hy = response.calibration_data.psd_y[band] ** 0.5
    hz = response.calibration_data.psd_z[band] ** 0.5
    h_sum = response.calibration_data.psd_sum[band] ** 0.5
    freqs = grid[band]

    ax_h.plot(freqs, h_sum, label="X+Y+Z", color="purple", linewidth=2.0)
    ax_h.plot(freqs, hz, label="Z", color="blue")
    ax_h.plot(freqs, hy, label="Y", color="green")
    ax_h.plot(freqs, hx, label="X", color="red")
    ax_h.set_ylabel("Transmissibility |H|")
    ax_h.set_xlim([0, max_freq])
    ax_h.grid(which="major", color="grey")
    ax_h.grid(which="minor", color="lightgrey")

    sweep_band = (classic_data.freq_bins >= response.f0.min()) & (
        classic_data.freq_bins <= response.f0.max()
    )
    classic_psd = classic_data.psd_sum
    if sweep_band.any() and classic_psd[sweep_band].max() > 0:
        scale = h_sum.max() / classic_psd[sweep_band].max()
        ax_h.plot(
            classic_data.freq_bins[sweep_band],
            classic_psd[sweep_band] * scale,
            label="classic PSD\n(scaled)",
            color="grey",
            linestyle="dashed",
            linewidth=0.8,
        )

    ax2 = ax_h.twinx()
    ax2.set_ylabel("Shaper vibration reduction (ratio)")
    best_shaper_vals = None
    for shaper in shapers:
        label = "%s (%.1f Hz, vibr=%.1f%%, sm~=%.2f)" % (
            shaper.name.upper(),
            shaper.freq,
            shaper.vibrs * 100.0,
            shaper.smoothing,
        )
        linestyle = "dotted"
        if shaper.name == selected_shaper:
            linestyle = "dashdot"
            best_shaper_vals = shaper.vals
        fit_freqs = grid[: len(shaper.vals)]
        fit_band = fit_freqs <= max_freq
        ax2.plot(
            fit_freqs[fit_band],
            shaper.vals[fit_band],
            label=label,
            linestyle=linestyle,
        )
    if best_shaper_vals is not None:
        shaped = min(len(freqs), len(best_shaper_vals))
        ax_h.plot(
            freqs[:shaped],
            h_sum[:shaped] * best_shaper_vals[:shaped],
            label="After\nshaper",
            color="cyan",
        )
    ax2.plot(
        [],
        [],
        " ",
        label="Chirp FRF shaper: %s" % (str(selected_shaper).upper()),
    )
    ax2.plot(
        [],
        [],
        " ",
        label="Classic PSD shaper: %s" % (str(classic_shaper).upper()),
    )
    if mode_fit is not None:
        inv_fn, inv_zeta, inv_gain, _inv_err = mode_fit
        ax_h.plot(
            freqs,
            second_order_magnitude(freqs, inv_fn, inv_zeta, inv_gain),
            color="black",
            linestyle="dotted",
            linewidth=1.0,
            label="2nd-order\nfit",
        )
        ax2.plot(
            [],
            [],
            " ",
            label="mode_inverse: frequency_hz=%.1f damping_ratio=%.3f"
            % (inv_fn, inv_zeta),
        )
    ax_h.legend(loc="upper left", prop=fontP)
    ax2.legend(loc="upper right", prop=fontP)

    names = ", ".join(pathlib.Path(name).name for name in lognames)
    title = "Demodulated chirp FRF (%s)  t0=%.3f s  %.2f Hz/s  aph_eff=%.1f" % (
        names,
        response.t0,
        chirp_sweep_rate(response.config),
        response.aph_eff,
    )
    ax_h.set_title("\n".join(wrap(title, MAX_TITLE_LENGTH)))

    if (
        not response.excitation_reached_toolhead()
        or response.median_snr < SNR_WARN_LEVEL
    ):
        ax_h.set_ylim(top=max(ax_h.get_ylim()[1], h_sum.max() * 1.8))
        ax_h.text(
            0.5,
            0.72,
            "WARNING: excitation likely never reached the toolhead\n"
            "(peak |H|=%.2f, median SNR=%.1f) - recommendation unreliable"
            % (response.peak_transmissibility, response.median_snr),
            transform=ax_h.transAxes,
            ha="center",
            va="center",
            color="red",
            fontsize=10,
            bbox=dict(boxstyle="round", fc="wheat", ec="red"),
        )

    for order in sorted(response.harmonics):
        ratio = np.interp(freqs, response.f0, response.harmonics[order])
        ax_harm.plot(freqs, ratio, label="%dx f0" % (order,))
    ax_harm.set_ylabel("Harmonic / fund.")
    ax_harm.legend(loc="upper right", prop=fontP)
    ax_harm.grid(color="lightgrey")

    snr = np.interp(freqs, response.f0, response.snr)
    ax_snr.axhspan(
        0, SNR_WARN_LEVEL, color="red", alpha=0.12, label="unreliable (<3)"
    )
    ax_snr.plot(freqs, snr, color="black", label="track SNR")
    ax_snr.set_ylabel("Track SNR")
    ax_snr.set_xlabel("Sweep frequency f0, Hz")
    ax_snr.legend(loc="upper right", prop=fontP)
    ax_snr.grid(color="lightgrey")


def plot_classic_with_chirp(
    lognames,
    raw_data,
    classic_data,
    classic_shapers,
    classic_name,
    accels_per_hz,
    response,
    chirp_shapers,
    chirp_name,
    max_freq,
    mode_fit=None,
):
    fig = matplotlib.pyplot.figure()
    outer = fig.add_gridspec(
        nrows=2, ncols=1, height_ratios=[4, 5.6], hspace=0.3
    )
    gs_classic = outer[0].subgridspec(nrows=2, ncols=1, height_ratios=[3, 1])
    gs_chirp = outer[1].subgridspec(
        nrows=3, ncols=1, height_ratios=[3, 1.3, 1.3]
    )
    ax_psd = fig.add_subplot(gs_classic[0])
    ax_spec = fig.add_subplot(gs_classic[1], sharex=ax_psd)
    ax_h = fig.add_subplot(gs_chirp[0])
    ax_harm = fig.add_subplot(gs_chirp[1], sharex=ax_h)
    ax_snr = fig.add_subplot(gs_chirp[2], sharex=ax_h)
    draw_freq_response(
        ax_psd,
        ax_spec,
        lognames,
        classic_data,
        classic_shapers,
        classic_name,
        max_freq,
        accels_per_hz,
        raw_data,
    )
    draw_chirp_frf(
        ax_h,
        ax_harm,
        ax_snr,
        lognames[:1],
        response,
        chirp_shapers,
        chirp_name,
        classic_data,
        classic_name,
        max_freq,
        mode_fit,
    )
    fig.tight_layout()
    return fig


######################################################################
# Chirp calibration orchestration
######################################################################


def run_chirp_mode(
    lognames, data, config, options, shaper_kwargs, accels_per_hz
):
    helper = shaper_calibrate.ShaperCalibrate(printer=None)
    try:
        response = analyze_chirp(data, config, options.chirp_bw)
    except ChirpBandTooShort as e:
        print(
            "Chirp FRF analysis skipped (%s); falling back to the classic "
            "estimate" % (e,)
        )
        return False

    if len(lognames) > 1:
        print(
            "Chirp analysis uses only the first log (%s); ignoring the rest"
            % (lognames[0],)
        )
    print(
        "Chirp sweep: %.1f -> %.1f Hz over %.1f s"
        % (
            config["freq_start"],
            config["freq_end"],
            config["duration"],
        )
    )
    print(
        "Estimated t0 = %.3f s, sweep rate = %.3f Hz/s, aph_eff = %.3f, "
        "sample rate = %.0f Hz"
        % (
            response.t0,
            chirp_sweep_rate(config),
            response.aph_eff,
            response.fs,
        )
    )
    if response.t0_weakly_constrained:
        print(
            "Note: t0 is weakly constrained (energy peak flat or at search "
            "boundary); the frequency response is insensitive to this offset"
        )
    print(
        "Peak transmissibility |H| = %.2f, median track SNR = %.2f"
        % (response.peak_transmissibility, response.median_snr)
    )

    classic_data = helper.process_accelerometer_data(data)
    classic_data.normalize_to_frequencies()
    print("\n=== Classic Welch-PSD estimator ===")
    classic_shaper, classic_shapers = helper.find_best_shaper(
        classic_data, logger=print, **shaper_kwargs
    )
    print("\n=== Chirp demodulated FRF estimator ===")
    chirp_shaper, chirp_shapers = helper.find_best_shaper(
        response.calibration_data, logger=print, **shaper_kwargs
    )

    classic_name = classic_shaper.name if classic_shaper else None
    chirp_name = chirp_shaper.name if chirp_shaper else None
    print("\n--- Recommendations ---")
    if classic_shaper:
        print(
            "Classic PSD : %s @ %.1f Hz"
            % (classic_shaper.name, classic_shaper.freq)
        )
    else:
        print("Classic PSD : no recommendation")
    if chirp_shaper:
        print(
            "Chirp FRF   : %s @ %.1f Hz"
            % (chirp_shaper.name, chirp_shaper.freq)
        )
    else:
        print("Chirp FRF   : no recommendation")
    frf_freqs = response.calibration_data.freq_bins
    frf_mag = np.sqrt(response.calibration_data.psd_sum)
    mode_fit = fit_second_order_mode(frf_freqs, frf_mag)
    inv_fn, inv_zeta, inv_gain, inv_err = mode_fit
    print(
        "mode_inverse: frequency_hz=%.1f damping_ratio=%.3f "
        "(2nd-order fit: gain %.2f, rel err %.0f%%)"
        % (inv_fn, inv_zeta, inv_gain, 100.0 * inv_err)
    )
    if inv_err > 0.25:
        print(
            "  note: the 2nd-order model fits this response poorly; "
            "mode_inverse parameters are approximate"
        )
    if not response.excitation_reached_toolhead():
        print(
            "  note: excitation warning active; mode_inverse parameters "
            "are unreliable"
        )

    if (
        not response.excitation_reached_toolhead()
        or response.median_snr < SNR_WARN_LEVEL
    ):
        print("\n" + "!" * 60)
        print("WARNING: the excitation likely never reached the toolhead.")
        print(
            "Peak transmissibility |H| = %.2f (a healthy sweep amplifies the "
            "commanded input, |H| >> 1)." % (response.peak_transmissibility,)
        )
        print(
            "Median track SNR = %.2f. This often means a wrong motor mapping "
            "or a disconnected axis." % (response.median_snr,)
        )
        print("Shaper recommendations from this run are unreliable.")
        print("!" * 60)

    if options.csv:
        helper.save_calibration_data(
            options.csv,
            response.calibration_data,
            chirp_shapers,
            accel_per_hz=response.aph_eff,
        )

    if not options.csv or options.output:
        setup_matplotlib(options.output is not None)
        fig = plot_classic_with_chirp(
            lognames,
            data,
            classic_data,
            classic_shapers,
            classic_name,
            accels_per_hz,
            response,
            chirp_shapers,
            chirp_name,
            shaper_kwargs["max_freq"],
            mode_fit,
        )
        if options.output is None:
            matplotlib.pyplot.show()
        else:
            fig.set_size_inches(9, 16)
            fig.savefig(options.output)
    return True


######################################################################
# Startup
######################################################################


def setup_matplotlib(output_to_file):
    global matplotlib
    if output_to_file:
        matplotlib.rcParams.update({"figure.autolayout": True})
        matplotlib.use("Agg")
    import matplotlib.colors
    import matplotlib.dates
    import matplotlib.font_manager
    import matplotlib.mlab
    import matplotlib.pyplot
    import matplotlib.ticker


def main():
    # Parse command-line arguments
    usage = "%prog [options] <logs>"
    opts = optparse.OptionParser(usage)
    opts.add_option(
        "-o",
        "--output",
        type="string",
        dest="output",
        default=None,
        help="filename of output graph",
    )
    opts.add_option(
        "-c",
        "--csv",
        type="string",
        dest="csv",
        default=None,
        help="filename of output csv file",
    )
    opts.add_option(
        "-f",
        "--max_freq",
        type="float",
        default=200.0,
        help="maximum frequency to plot",
    )
    opts.add_option(
        "-s",
        "--max_smoothing",
        type="float",
        dest="max_smoothing",
        default=None,
        help="maximum shaper smoothing to allow",
    )
    opts.add_option(
        "--scv",
        "--square_corner_velocity",
        type="float",
        dest="scv",
        default=5.0,
        help="square corner velocity",
    )
    opts.add_option(
        "--shaper_freq",
        type="string",
        dest="shaper_freq",
        default=None,
        help="shaper frequency(-ies) to test, "
        + "either a comma-separated list of floats, or a range in "
        + "the format [start]:end[:step]",
    )
    opts.add_option(
        "--shapers",
        type="string",
        dest="shapers",
        default=None,
        help="a comma-separated list of shapers to test",
    )
    opts.add_option(
        "--damping_ratio",
        type="float",
        dest="damping_ratio",
        default=None,
        help="shaper damping_ratio parameter",
    )
    opts.add_option(
        "--test_damping_ratios",
        type="string",
        dest="test_damping_ratios",
        default=None,
        help="a comma-separated liat of damping ratios to test "
        + "input shaper for",
    )
    opts.add_option(
        "--chirp",
        type="string",
        dest="chirp",
        default=None,
        help="analyze a swept-sine (chirp) raw accelerometer capture; "
        + "value overrides any header, e.g. "
        + '"freq_start=50,freq_end=133,duration=83,accel_per_hz=45" '
        + "(ramp and amplitude_mm optional, default 0)",
    )
    opts.add_option(
        "--chirp-bw",
        type="float",
        dest="chirp_bw",
        default=DEFAULT_CHIRP_BW,
        help="lock-in demodulation bandwidth in Hz for chirp analysis",
    )
    options, args = opts.parse_args()
    if len(args) < 1:
        opts.error("Incorrect number of arguments")
    if options.max_smoothing is not None and options.max_smoothing < 0.05:
        opts.error("Too small max_smoothing specified (must be at least 0.05)")

    max_freq = options.max_freq
    if options.shaper_freq is None:
        shaper_freqs = []
    elif options.shaper_freq.find(":") >= 0:
        freq_start = None
        freq_end = None
        freq_step = None
        try:
            freqs_parsed = options.shaper_freq.partition(":")
            if freqs_parsed[0]:
                freq_start = float(freqs_parsed[0])
            freqs_parsed = freqs_parsed[-1].partition(":")
            freq_end = float(freqs_parsed[0])
            if freq_start and freq_start > freq_end:
                opts.error(
                    "Invalid --shaper_freq param: start range larger "
                    + "than its end"
                )
            if freqs_parsed[-1].find(":") >= 0:
                opts.error("Invalid --shaper_freq param format")
            if freqs_parsed[-1]:
                freq_step = float(freqs_parsed[-1])
        except ValueError:
            opts.error(
                "--shaper_freq param does not specify correct range "
                + "in the format [start]:end[:step]"
            )
        shaper_freqs = (freq_start, freq_end, freq_step)
        max_freq = max(max_freq, freq_end * 4.0 / 3.0)
    else:
        try:
            shaper_freqs = [float(s) for s in options.shaper_freq.split(",")]
        except ValueError:
            opts.error("invalid floating point value in --shaper_freq param")
        max_freq = max(max_freq, max(shaper_freqs) * 4.0 / 3.0)
    if options.test_damping_ratios:
        try:
            test_damping_ratios = [
                float(s) for s in options.test_damping_ratios.split(",")
            ]
        except ValueError:
            opts.error(
                "invalid floating point value in "
                + "--test_damping_ratios param"
            )
    else:
        test_damping_ratios = None
    if options.shapers is None:
        shapers = None
    else:
        shapers = options.shapers.lower().split(",")

    # Parse data
    datas = [parse_log(fn) for fn in args]
    accels_per_hz = [parse_accel_per_hz(fn) for fn in args]

    try:
        chirp_config = resolve_chirp_config(args[0], options.chirp)
    except (ValueError, OSError) as e:
        opts.error("Invalid chirp configuration: %s" % (e,))
    if chirp_config is not None and isinstance(datas[0], np.ndarray):
        shaper_kwargs = dict(
            shapers=shapers,
            damping_ratio=options.damping_ratio,
            scv=options.scv,
            shaper_freqs=shaper_freqs,
            max_smoothing=options.max_smoothing,
            test_damping_ratios=test_damping_ratios,
            max_freq=max_freq,
        )
        if run_chirp_mode(
            args, datas[0], chirp_config, options, shaper_kwargs, accels_per_hz
        ):
            return

    # Calibrate shaper and generate outputs
    selected_shaper, shapers, calibration_data = calibrate_shaper(
        datas,
        options.csv,
        shapers=shapers,
        damping_ratio=options.damping_ratio,
        scv=options.scv,
        shaper_freqs=shaper_freqs,
        max_smoothing=options.max_smoothing,
        test_damping_ratios=test_damping_ratios,
        max_freq=max_freq,
    )
    if selected_shaper is None:
        return

    if not options.csv or options.output:
        # Draw graph
        setup_matplotlib(options.output is not None)

        raw_data = (
            datas[0]
            if not isinstance(datas[0], shaper_calibrate.CalibrationData)
            else None
        )
        fig = plot_freq_response(
            args,
            calibration_data,
            shapers,
            selected_shaper,
            max_freq,
            accels_per_hz,
            raw_data=raw_data,
        )

        # Show graph
        if options.output is None:
            matplotlib.pyplot.show()
        else:
            fig.set_size_inches(8, 8 if raw_data is not None else 6)
            fig.savefig(options.output)


if __name__ == "__main__":
    main()
