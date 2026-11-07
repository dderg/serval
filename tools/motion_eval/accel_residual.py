"""Hardware twin of oscillator.py: ringing after corners and the corner
acceleration burst, measured by a toolhead accelerometer while a
printer_test gcode prints.

    uv run python -m tools.motion_eval.accel_residual \\
        A.csv A.gcode [B.csv B.gcode ...] [--config owner.cfg] [--accel A]

One table row per CSV/gcode pair.

The CSV is what Klipper's ACCELEROMETER_MEASURE writes
(klippy/extras/adxl345.py):
- Path: `<tmpdir>/<chip>-<NAME>.csv`, or `<chip>-<name>-<NAME>.csv` for a
  named chip section; tmpdir is the klippy host's temp dir, normally /tmp
  (AccelCommandHelper.cmd_ACCELEROMETER_MEASURE, lines 184-210).
- Header `#time,accel_x,accel_y,accel_z`, then one `%.6f,%.6f,%.6f,%.6f`
  row per sample (AccelQueryHelper.write_to_file, lines 110-124).
- time: the sample's host print time, in seconds. bulk_sensor maps the
  chip's sample clock onto the MCU clock and that onto print time
  (ClockSyncRegression.get_time_translation, klippy/extras/bulk_sensor.py
  lines 234-239). This is the clock the toolhead plans moves in, so the CSV
  and the offline plan differ by one constant offset; that offset is
  unknown, because the replay starts its print time at zero.
- Only samples between the print times of the measurement's start and
  stop are kept (AccelQueryHelper.get_samples, lines 93-108).
- accel_*: mm/s^2 in printer axes. Raw counts are multiplied by the chip
  scale (ADXL345 SCALE_XY = 0.003774 g/LSB with FREEFALL_ACCEL 9806.65
  mm/s^2, lines 39-41), then permuted and signed by `axes_map`
  (read_axes_map, lines 264-276; _convert_samples, lines 358-376). Gravity
  is not removed. On a tilted sensor it is a constant per-axis offset,
  estimated here and subtracted.
- Rate: the chip's `rate` option, 3200 Hz by default on the ADXL345
  (line 288). lis2dw (1600 Hz), mpu9250 (4000 Hz) and icm20948 (4500 Hz)
  use the same AccelCommandHelper and write the same file. The ADXL345 runs
  at +-16 g (REG_DATA_FORMAT 0x0B, line 393). The mpu9250 runs at +-8 g
  (78 000 mm/s^2, klippy/extras/mpu9250.py line 52), which a 1.6x burst at
  50 000 mm/s^2 clips. A clipped record fails here.

Every gcode carries `; motion_eval key=value` headers (written by
printer_test.py):
- start: the toolhead position the replay starts from.
- setting: the row name.
- corner_deviation: none, floor, or a max_deviation in mm. It selects the
  replay config.
- measured_moves: the trapq move range [first, end) that runs between the
  file's MOTION_EVAL_ACCEL_START and MOTION_EVAL_ACCEL_STOP.
- sync_s: how long that range's opening sync motion lasts.

The gcode is replanned with the harness (run.run_klippy, through a config
that includes --config and defines both macros as no-ops). The executed
(input-shaped) carriage acceleration then comes from
evaluate.executed_motion.

Alignment: both records are low-passed at ALIGN_CUTOFF_RATIO times the
lowest configured shaper frequency, which keeps the planned motion and
drops most of the ringing. The sync motion of the plan is then
cross-correlated, x and y together, against the whole CSV. The best
normalised correlation must reach MIN_CORRELATION. The best peak outside
the main lobe must stay below AMBIGUITY_RATIO of it. The offset is then
refined to a fraction of a sample over the whole measured range. Each axis
must correlate with its plan at MIN_CORRELATION or better. A negative
correlation points at axes_map. Every CHUNK_S piece of the motion must
find its own best offset within CHUNK_TOLERANCE_SAMPLES samples of the
global one. A stall of the live planner, which shifts the rest of the
print, fails here.

Residual: in a constant-velocity window (oscillator.constant_velocity_
windows) the executed carriage acceleration is zero, so the measured
toolhead acceleration is the lag's e''. The lag itself is reported, as
oscillator.residuals does: the largest |e| over the first period after the
window starts, in um. Converting e'' to e needs a frequency, and it is
taken from the data. A damped sinusoid A exp(-s t) cos(w t + phi) is
fitted to the window's measured acceleration by bounded nonlinear least
squares, started from a two-pole linear predictor and from the periodogram
peak (the better fit is kept). Its pole p = -s + iw gives the
displacement e = Re(C/p^2 exp(pt)) directly, so the result is the
displacement of the mode actually ringing, whatever its frequency. A
fixed frequency would misreport any mode away from it by (f_true/f)^2,
about 6x for the 2.5x mode. A fit is kept only if:
- it explains at least MIN_FIT_R2 of the window's signal;
- its frequency lies between MIN_RING_PERIODS per window and
  MAX_FIT_NYQUIST of Nyquist;
- it decays;
- its peak exceeds NOISE_SIGMAS times the noise measured while the
  toolhead stands still before the motion.
Otherwise the window falls back to the configured shaper frequency of the
axis, which is the mode the shaper was tuned to: max|a| over one period
at that frequency, over (2 pi f)^2. Noise-only windows land there and
report the noise floor. The table counts fitted and fallback windows per
axis.

Corner peak: the largest measured |a| (bias removed) within reach of each
vertex after the sync. The reach is the kernel's half width plus its time
offset (as in owner_point). The same is reported for the plan's executed
acceleration, both as multiples of max_accel.
"""

from __future__ import annotations

import argparse
import dataclasses
import functools
import math
import pathlib
import sys
import tempfile

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, correlate, sosfiltfilt

from . import evaluate, oscillator, run, sweep

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "configs" / "owner.cfg"
CSV_HEADER = "#time,accel_x,accel_y,accel_z"
HEADER_PREFIX = "; motion_eval "
REQUIRED_HEADERS = (
    "start",
    "setting",
    "corner_deviation",
    "measured_moves",
    "sync_s",
)
AXES = "xy"
START_MACRO = "MOTION_EVAL_ACCEL_START"
STOP_MACRO = "MOTION_EVAL_ACCEL_STOP"
DT = 25e-6
MIN_SAMPLES = 1000
MAX_INTERVAL_DEVIATION = 0.5
CLIP_RUN = 4
ALIGN_CUTOFF_RATIO = 0.5
ALIGN_FILTER_ORDER = 4
MIN_CORRELATION = 0.7
AMBIGUITY_RATIO = 0.7
REFINE_STEPS_PER_SAMPLE = 40
REFINE_REACH_SAMPLES = 2
CHUNK_S = 1.0
CHUNK_MIN_ENERGY_SHARE = 0.05
CHUNK_TOLERANCE_SAMPLES = 0.5
QUIET_S = 0.2
MIN_QUIET_SAMPLES = 64
MIN_FIT_SAMPLES = 12
MIN_FIT_R2 = 0.5
MIN_RING_PERIODS = 2.0
MAX_FIT_NYQUIST = 0.9
NOISE_SIGMAS = 4.0
PERIOD_POINTS = 257
PERIODOGRAM_PADDING = 8
UM_PER_MM = 1000.0


class AccelResidualError(RuntimeError):
    pass


@dataclasses.dataclass
class Measurement:
    t: np.ndarray
    acc: dict

    @property
    def dt(self):
        return float(np.median(np.diff(self.t)))


@dataclasses.dataclass
class Plan:
    setting: str
    corner_deviation: str
    max_accel: float
    freqs: dict
    t0: float
    acc: dict
    pos: dict
    kernels: dict
    span: tuple
    sync: tuple
    windows: list
    vertices: np.ndarray

    @functools.cached_property
    def grid(self):
        return self.t0 + DT * np.arange(len(self.acc["x"]))

    def acc_at(self, axis, t):
        return np.interp(t, self.grid, self.acc[axis], left=0.0, right=0.0)


@dataclasses.dataclass
class Alignment:
    offset: float
    sync_peak: float
    second_peak: float
    rho: dict
    worst_chunk_slip: float


@dataclasses.dataclass
class WindowResidual:
    start: float
    freq: float
    fitted: bool
    amplitude_mm: float


def read_csv(path):
    path = pathlib.Path(path)
    with path.open() as f:
        header = f.readline().strip()
    if header != CSV_HEADER:
        raise AccelResidualError(
            "%s: header %r is not ACCELEROMETER_MEASURE's %r"
            % (path, header, CSV_HEADER)
        )
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if data.shape[1] != 4 or len(data) < MIN_SAMPLES:
        raise AccelResidualError(
            "%s: %d rows of %d columns, need >= %d rows of 4"
            % (path, len(data), data.shape[1], MIN_SAMPLES)
        )
    meas = Measurement(data[:, 0], {"x": data[:, 1], "y": data[:, 2]})
    _check_sampling(path, meas)
    for axis in AXES:
        _check_clipping(path, axis, meas.acc[axis])
    return meas


def _check_sampling(path, meas):
    step = np.diff(meas.t)
    dt = meas.dt
    bad = np.flatnonzero(np.abs(step - dt) > MAX_INTERVAL_DEVIATION * dt)
    if len(bad):
        raise AccelResidualError(
            "%s: %d irregular sample intervals (median %.1f us); first at"
            " t=%.6f is %.1f us: samples were lost or the clock jumped"
            % (path, len(bad), dt * 1e6, meas.t[bad[0]], step[bad[0]] * 1e6)
        )


def _check_clipping(path, axis, acc):
    extreme = np.abs(acc).max()
    at_extreme = np.abs(acc) == extreme
    run_len = np.convolve(at_extreme, np.ones(CLIP_RUN), mode="valid")
    if (run_len >= CLIP_RUN).any():
        raise AccelResidualError(
            "%s: accel_%s holds its extreme %.0f mm/s^2 for %d samples in a"
            " row: the sensor clipped; use a wider range"
            % (path, axis, extreme, CLIP_RUN)
        )


def parse_headers(text):
    headers = {}
    for line in text.splitlines():
        if not line.startswith(HEADER_PREFIX):
            break
        key, value = line[len(HEADER_PREFIX) :].split("=", 1)
        headers[key.strip()] = value.strip()
    missing = [k for k in REQUIRED_HEADERS if k not in headers]
    if missing:
        raise AccelResidualError(
            "gcode lacks motion_eval headers %s" % (", ".join(missing),)
        )
    return headers


def config_option(path, section, option):
    """Last value of [section] option in a klipper config, following
    [include] sections."""
    path = pathlib.Path(path)
    value = None
    current = None
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if line.startswith("[") and line.endswith("]"):
            current = line[1:-1].strip()
            if current.startswith("include "):
                included = path.parent / current.split(None, 1)[1]
                found = config_option(included, section, option)
                value = found if found is not None else value
            continue
        if current != section or not line or line[0].isspace():
            continue
        for sep in (":", "="):
            key, hit, rest = line.partition(sep)
            if hit and key.strip() == option:
                value = rest.strip()
                break
    return value


def shaper_freqs(config):
    out = {}
    for axis in AXES:
        value = config_option(config, "input_shaper", "shaper_freq_" + axis)
        if value is None:
            raise AccelResidualError(
                "%s has no [input_shaper] shaper_freq_%s" % (config, axis)
            )
        out[axis] = float(value)
    return out


def replay_config(config, corner_deviation, tmpdir):
    """A printer.cfg that includes `config`, defines the accelerometer
    macros as no-ops and adds [corner_deviation] as the header asks."""
    text = "[include %s]\n" % (pathlib.Path(config).resolve(),)
    for macro in (START_MACRO, STOP_MACRO):
        text += "\n[gcode_macro %s]\ngcode:\n" % (macro,)
    if corner_deviation != "none":
        text += "\n[corner_deviation]\n"
        if corner_deviation != "floor":
            text += "max_deviation: %s\n" % (float(corner_deviation),)
    path = pathlib.Path(tmpdir) / "printer.cfg"
    path.write_text(text)
    return path


def replay(text, config, accel=None):
    """run_klippy result of a printer_test gcode under its header's
    corner_deviation setting."""
    headers = parse_headers(text)
    prefix = "" if accel is None else "SET_VELOCITY_LIMIT ACCEL=%r\n" % accel
    with tempfile.TemporaryDirectory(prefix="accel_residual_") as tmp:
        cfg = replay_config(config, headers["corner_deviation"], tmp)
        return run.run_klippy(cfg, prefix + text, sweep.case_start(text))


def _move_end(move):
    return (
        move["print_time"]
        + move["accel_t"]
        + move["cruise_t"]
        + move["decel_t"]
    )


def plan_from_capture(cap, headers, freqs):
    seg, epochs, t0, _, _, motion = evaluate.executed_motion(cap, DT)
    if len(epochs) != 1:
        raise AccelResidualError(
            "%d input shaper settings in one file; printer_test files keep"
            " one" % len(epochs)
        )
    kernels = epochs[0].kernels
    moves = cap["moves"]
    first, end = (int(v) for v in headers["measured_moves"].split(","))
    if not 0 <= first < end <= len(moves):
        raise AccelResidualError(
            "measured_moves %d,%d outside the %d planned moves"
            % (first, end, len(moves))
        )
    max_accels = {m["max_accel"] for m in moves[first:end]}
    if len(max_accels) != 1:
        raise AccelResidualError(
            "max_accel changes inside the measured moves: %s"
            % sorted(max_accels)
        )
    span = (moves[first]["print_time"], _move_end(moves[end - 1]))
    sync = (span[0], span[0] + float(headers["sync_s"]))
    windows = [
        (start, stop)
        for start, stop in oscillator.constant_velocity_windows(
            seg, evaluate.kernel_support(kernels)
        )
        if start >= sync[1] and stop <= span[1]
    ]
    t_arrive = seg.vertices()["t_arrive"]
    return Plan(
        setting=headers["setting"],
        corner_deviation=headers["corner_deviation"],
        max_accel=max_accels.pop(),
        freqs=freqs,
        t0=t0,
        acc={a: motion[a].acc for a in AXES},
        pos={a: motion[a].pos for a in AXES},
        kernels=kernels,
        span=span,
        sync=sync,
        windows=windows,
        vertices=t_arrive[(t_arrive > sync[1]) & (t_arrive < span[1])],
    )


def replan(gcode_path, config=DEFAULT_CONFIG, accel=None):
    text = pathlib.Path(gcode_path).read_text()
    raw = replay(text, config, accel)
    return plan_from_capture(
        raw.capture, parse_headers(text), shaper_freqs(config)
    )


def _lowpass(values, fs, cutoff):
    sos = butter(ALIGN_FILTER_ORDER, cutoff, fs=fs, output="sos")
    return sosfiltfilt(sos, values)


def _align_cutoff(plan, fs):
    cutoff = ALIGN_CUTOFF_RATIO * min(plan.freqs.values())
    if cutoff >= 0.5 * fs:
        raise AccelResidualError(
            "sample rate %.0f Hz too low for the %.0f Hz alignment band"
            % (fs, cutoff)
        )
    return cutoff


def _support(plan):
    lo, hi = evaluate.kernel_support(plan.kernels)
    return max(abs(lo), abs(hi))


def _correlation(meas_lp, plan_lp, t, offset):
    """x+y normalised correlation of the low-passed measurement with the
    plan shifted by offset, and per-axis Pearson coefficients."""
    num, e_m, e_p, rho = 0.0, 0.0, 0.0, {}
    for axis in AXES:
        m = meas_lp[axis]
        p = plan_lp[axis](t - offset)
        num += float(m @ p)
        e_m += float(m @ m)
        e_p += float(p @ p)
        mc, pc = m - m.mean(), p - p.mean()
        rho[axis] = float(mc @ pc / math.sqrt((mc @ mc) * (pc @ pc)))
    return num / math.sqrt(e_m * e_p), rho


def _refine(meas_lp, plan_lp, t, center, reach, step):
    offsets = center + step * np.arange(
        -round(reach / step), round(reach / step) + 1
    )
    scores = np.array(
        [_correlation(meas_lp, plan_lp, t, o)[0] for o in offsets]
    )
    k = int(np.argmax(scores))
    if 0 < k < len(scores) - 1:
        a, b, c = scores[k - 1 : k + 2]
        denom = a - 2.0 * b + c
        if denom < 0.0:
            return offsets[k] + 0.5 * step * (a - c) / denom, k
    return offsets[k], k


def _main_lobe(ncc, k):
    lo = k
    while lo > 0 and ncc[lo - 1] > 0.0:
        lo -= 1
    hi = k
    while hi < len(ncc) - 1 and ncc[hi + 1] > 0.0:
        hi += 1
    return lo, hi


def align(meas, plan):
    fs = 1.0 / meas.dt
    cutoff = _align_cutoff(plan, fs)
    grid = plan.grid
    meas_lp = {
        a: _lowpass(meas.acc[a] - meas.acc[a].mean(), fs, cutoff) for a in AXES
    }
    plan_full = {a: _lowpass(plan.acc[a], 1.0 / DT, cutoff) for a in AXES}

    def plan_interp(axis):
        return lambda t: np.interp(
            t, grid, plan_full[axis], left=0.0, right=0.0
        )

    plan_lp = {a: plan_interp(a) for a in AXES}
    margin = _support(plan)
    template_t = np.arange(
        plan.sync[0] - margin, plan.sync[1] + margin, meas.dt
    )
    if len(template_t) >= len(meas.t):
        raise AccelResidualError(
            "the CSV (%.3f s) is shorter than the sync motion (%.3f s)"
            % (meas.t[-1] - meas.t[0], template_t[-1] - template_t[0])
        )
    num = 0.0
    energy = np.zeros(len(meas.t) - len(template_t) + 1)
    template_energy = 0.0
    for axis in AXES:
        tpl = plan_lp[axis](template_t)
        num = num + correlate(meas_lp[axis], tpl, mode="valid")
        sq = np.concatenate(([0.0], np.cumsum(meas_lp[axis] ** 2)))
        energy += sq[len(tpl) :] - sq[: len(sq) - len(tpl)]
        template_energy += float(tpl @ tpl)
    denom = np.sqrt(energy * template_energy)
    ncc = np.divide(num, denom, out=np.zeros_like(num), where=denom > 0.0)
    k = int(np.argmax(ncc))
    peak = float(ncc[k])
    lo, hi = _main_lobe(ncc, k)
    outside = np.concatenate((ncc[:lo], ncc[hi + 1 :]))
    second = float(outside.max()) if len(outside) else 0.0
    if peak < MIN_CORRELATION:
        raise AccelResidualError(
            "the sync motion correlates at best %.2f with the CSV (need"
            " %.2f): wrong file pair, axes_map or sensor"
            % (peak, MIN_CORRELATION)
        )
    if second >= AMBIGUITY_RATIO * peak:
        raise AccelResidualError(
            "ambiguous alignment: sync correlation %.3f at t=%.4f, %.3f"
            " elsewhere (limit %.2f of the best)"
            % (peak, meas.t[k], second, AMBIGUITY_RATIO)
        )
    coarse = meas.t[k] - template_t[0]
    need_lo = plan.span[0] - margin + coarse
    need_hi = plan.span[1] + margin + coarse
    covered = (meas.t >= need_lo) & (meas.t <= need_hi)
    if meas.t[0] > need_lo or meas.t[-1] < need_hi:
        raise AccelResidualError(
            "the CSV (t %.3f..%.3f) does not cover the measured motion"
            " (t %.3f..%.3f at offset %.6f)"
            % (
                meas.t[0],
                meas.t[-1],
                plan.span[0] + coarse,
                plan.span[1] + coarse,
                coarse,
            )
        )
    t_cov = meas.t[covered]
    m_cov = {a: meas_lp[a][covered] for a in AXES}
    step = meas.dt / REFINE_STEPS_PER_SAMPLE
    offset, _ = _refine(
        m_cov, plan_lp, t_cov, coarse, REFINE_REACH_SAMPLES * meas.dt, step
    )
    slip = _chunk_slip(meas, meas_lp, plan, plan_lp, offset, step)
    _, rho = _correlation(m_cov, plan_lp, t_cov, offset)
    for axis, value in rho.items():
        if value < MIN_CORRELATION:
            raise AccelResidualError(
                "accel_%s correlates %.2f with the planned motion (need %.2f)"
                "%s"
                % (
                    axis,
                    value,
                    MIN_CORRELATION,
                    ": its sign looks inverted, check axes_map"
                    if value < -MIN_CORRELATION
                    else "",
                )
            )
    return Alignment(offset, peak, second, rho, slip)


def _chunk_slip(meas, meas_lp, plan, plan_lp, offset, step):
    count = max(1, int((plan.span[1] - plan.span[0]) // CHUNK_S))
    edges = np.linspace(plan.span[0], plan.span[1], count + 1)
    chunks = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (meas.t >= lo + offset) & (meas.t < hi + offset)
        energy = sum(
            float(np.sum(plan_lp[a](meas.t[sel] - offset) ** 2)) for a in AXES
        )
        chunks.append((sel, energy))
    top = max(e for _, e in chunks)
    reach = REFINE_REACH_SAMPLES * meas.dt
    tolerance = CHUNK_TOLERANCE_SAMPLES * meas.dt
    worst = 0.0
    for sel, energy in chunks:
        if energy < CHUNK_MIN_ENERGY_SHARE * top:
            continue
        t = meas.t[sel]
        m = {a: meas_lp[a][sel] for a in AXES}
        local, k = _refine(m, plan_lp, t, offset, reach, step)
        score, _ = _correlation(m, plan_lp, t, local)
        edge = k in (0, 2 * round(reach / step))
        slip = local - offset
        if edge or abs(slip) > tolerance or score < MIN_CORRELATION:
            raise AccelResidualError(
                "the motion at t=%.3f..%.3f does not follow the global"
                " offset: local best %+.1f us%s, correlation %.2f; the live"
                " planner stalled or the clocks drifted"
                % (
                    t[0],
                    t[-1],
                    slip * 1e6,
                    " (search edge)" if edge else "",
                    score,
                )
            )
        worst = max(worst, abs(slip))
    return worst


def bias_and_noise(meas, plan, offset):
    """Per-axis constant offset (gravity on a tilted sensor) and the noise
    standard deviation while the toolhead stands still before the
    motion."""
    plan_t = meas.t - offset
    inside = (plan_t >= plan.span[0]) & (plan_t <= plan.span[1])
    quiet = (plan_t >= plan.span[0] - QUIET_S) & (
        plan_t < plan.span[0] - _support(plan)
    )
    if quiet.sum() < MIN_QUIET_SAMPLES:
        raise AccelResidualError(
            "only %d samples before the motion starts (need %d): start the"
            " measurement earlier" % (quiet.sum(), MIN_QUIET_SAMPLES)
        )
    bias, noise = {}, {}
    for axis in AXES:
        planned = plan.acc_at(axis, plan_t[inside])
        bias[axis] = float(np.mean(meas.acc[axis][inside] - planned))
        noise[axis] = float(np.std(meas.acc[axis][quiet]))
    return bias, noise


def _basis(tau, sigma, omega):
    envelope = np.exp(-sigma * tau)
    return np.stack(
        [envelope * np.cos(omega * tau), envelope * np.sin(omega * tau)], 1
    )


def _prediction_start(tau, values, dt):
    """Pole of the least-squares two-pole linear predictor, or None."""
    lhs = np.stack([values[1:-1], values[:-2]], 1)
    c, *_ = np.linalg.lstsq(lhs, values[2:], rcond=None)
    disc = c[0] * c[0] + 4.0 * c[1]
    if disc >= 0.0 or c[1] >= 0.0:
        return None
    z = complex(0.5 * c[0], 0.5 * math.sqrt(-disc))
    return -math.log(abs(z)) / dt, math.atan2(z.imag, z.real) / dt


def _periodogram_start(tau, values, dt, band):
    """Strongest in-band frequency of the zero-padded spectrum, with an
    envelope decaying once over the window."""
    n = PERIODOGRAM_PADDING * len(values)
    spectrum = np.abs(np.fft.rfft(values, n))
    freqs = np.fft.rfftfreq(n, dt)
    inside = (freqs >= band[0]) & (freqs <= band[1])
    if not inside.any():
        return None
    peak = freqs[inside][np.argmax(spectrum[inside])]
    return 1.0 / (tau[-1] - tau[0]), 2.0 * math.pi * peak


def fit_ring(tau, values, band):
    """Damped sinusoid through the samples: (sigma, omega, C, r2) with
    values ~ Re(C exp((-sigma + i omega) tau)), or None. Least squares
    from two starts, the linear-prediction pole and the periodogram peak;
    the better fit is kept."""
    if len(values) < MIN_FIT_SAMPLES:
        return None
    dt = (tau[-1] - tau[0]) / (len(tau) - 1)

    def residual(p):
        return _basis(tau, p[0], p[1]) @ p[2:] - values

    lower = [0.0, 2.0 * math.pi * band[0], -np.inf, -np.inf]
    upper = [np.inf, 2.0 * math.pi * band[1], np.inf, np.inf]
    best = None
    for start in (
        _prediction_start(tau, values, dt),
        _periodogram_start(tau, values, dt, band),
    ):
        if start is None:
            continue
        start = np.clip(start, lower[:2], upper[:2])
        ab0, *_ = np.linalg.lstsq(_basis(tau, *start), values, rcond=None)
        sol = least_squares(
            residual,
            [*start, *ab0],
            bounds=(lower, upper),
            x_scale="jac",
        )
        if best is None or sol.cost < best.cost:
            best = sol
    if best is None:
        return None
    sigma, omega, a, b = best.x
    r2 = 1.0 - float(best.fun @ best.fun) / float(values @ values)
    if sigma <= 0.0 or r2 < MIN_FIT_R2:
        return None
    return sigma, omega, complex(a, -b), r2


def _period_peak(pole, coeff, omega):
    tau = np.linspace(0.0, 2.0 * math.pi / omega, PERIOD_POINTS)
    return float(np.abs((coeff * np.exp(pole * tau)).real).max())


def window_residuals(meas, plan, offset, axis, bias, noise):
    """Residual vibration of `axis` in every constant-velocity window of
    the plan, oscillator.residuals' counterpart: windows shorter than one
    period of the configured shaper frequency are skipped."""
    fs = 1.0 / meas.dt
    f_cfg = plan.freqs[axis]
    w_cfg = 2.0 * math.pi * f_cfg
    plan_t = meas.t - offset
    out = []
    for start, stop in plan.windows:
        if stop - start < 1.0 / f_cfg:
            continue
        sel = _between(plan_t, start, stop)
        tau = plan_t[sel] - start
        values = meas.acc[axis][sel] - bias - plan.acc_at(axis, plan_t[sel])
        band = (MIN_RING_PERIODS / (stop - start), MAX_FIT_NYQUIST * 0.5 * fs)
        fit = fit_ring(tau, values, band)
        if fit is not None:
            sigma, omega, coeff, _ = fit
            pole = complex(-sigma, omega)
            if _period_peak(pole, coeff, omega) >= NOISE_SIGMAS * noise:
                out.append(
                    WindowResidual(
                        start,
                        omega / (2.0 * math.pi),
                        True,
                        _period_peak(pole, coeff / (pole * pole), omega),
                    )
                )
                continue
        first = tau <= 1.0 / f_cfg
        out.append(
            WindowResidual(
                start,
                f_cfg,
                False,
                float(np.abs(values[first]).max()) / (w_cfg * w_cfg),
            )
        )
    return out


def _between(times, lo, hi):
    """Slice of the sorted `times` within [lo, hi]."""
    return slice(
        int(np.searchsorted(times, lo, side="left")),
        int(np.searchsorted(times, hi, side="right")),
    )


def corner_peaks(meas, plan, offset, axis, bias):
    """Largest measured and planned |a| within reach of each vertex."""
    kernel = plan.kernels[axis]
    reach = kernel.half_width + abs(kernel.t_offs)
    plan_t = meas.t - offset
    measured, planned = [], []
    for t_v in plan.vertices:
        sel = _between(plan_t, t_v - reach, t_v + reach)
        planned_sel = _between(plan.grid, t_v - reach, t_v + reach)
        measured.append(float(np.abs(meas.acc[axis][sel] - bias).max()))
        planned.append(float(np.abs(plan.acc[axis][planned_sel]).max()))
    return np.array(measured), np.array(planned)


def _stats(values):
    if not len(values):
        raise AccelResidualError("no windows or vertices to report")
    return (
        float(np.percentile(values, 50)),
        float(np.percentile(values, 99)),
        float(np.max(values)),
    )


def analyze(csv_path, gcode_path, config=DEFAULT_CONFIG, accel=None):
    meas = read_csv(csv_path)
    plan = replan(gcode_path, config, accel)
    alignment = align(meas, plan)
    bias, noise = bias_and_noise(meas, plan, alignment.offset)
    axes = {}
    for axis in AXES:
        windows = window_residuals(
            meas, plan, alignment.offset, axis, bias[axis], noise[axis]
        )
        measured, planned = corner_peaks(
            meas, plan, alignment.offset, axis, bias[axis]
        )
        fitted = [w.freq for w in windows if w.fitted]
        axes[axis] = {
            "residual_um": _stats(
                [w.amplitude_mm * UM_PER_MM for w in windows]
            ),
            "fitted": len(fitted),
            "fallback": len(windows) - len(fitted),
            "fit_freq_hz": float(np.median(fitted)) if fitted else None,
            "corner_measured": _stats(measured / plan.max_accel),
            "corner_planned_max": float(planned.max() / plan.max_accel),
            "noise_mm_s2": noise[axis],
        }
    return {"plan": plan, "alignment": alignment, "axes": axes}


def _row(result):
    plan, al, axes = result["plan"], result["alignment"], result["axes"]
    cells = [
        plan.setting,
        "%.6f" % al.offset,
        "%.2f / %.2f" % (al.rho["x"], al.rho["y"]),
        "%.2f / %.2f" % (al.sync_peak, al.second_peak),
    ]
    for axis in AXES:
        a = axes[axis]
        cells.append("%.1f / %.1f / %.1f" % a["residual_um"])
        freq = "—" if a["fit_freq_hz"] is None else "%.0f" % a["fit_freq_hz"]
        cells.append("%s (%d/%d)" % (freq, a["fitted"], a["fallback"]))
    for axis in AXES:
        a = axes[axis]
        cells.append(
            "%.2f / %.2f / %.2f (%.2f)"
            % (a["corner_measured"] + (a["corner_planned_max"],))
        )
    return "| " + " | ".join(cells) + " |"


def table(results):
    head = (
        "| setting | offset s | ρ x / y | sync peak / 2nd |"
        " X residual µm p50 / p99 / max | X ring Hz (fit/fallback) |"
        " Y residual µm p50 / p99 / max | Y ring Hz (fit/fallback) |"
        " X corner accel / max_accel p50 / p99 / max (plan max) |"
        " Y corner accel / max_accel p50 / p99 / max (plan max) |"
    )
    lines = [head, "|" + "---|" * 10]
    lines += [_row(r) for r in results]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m tools.motion_eval.accel_residual"
    )
    ap.add_argument("pairs", nargs="+", metavar="CSV GCODE")
    ap.add_argument("--config", type=pathlib.Path, default=DEFAULT_CONFIG)
    ap.add_argument(
        "--accel",
        type=float,
        help="max_accel the printer ran with, if lowered by"
        " SET_VELOCITY_LIMIT before the print",
    )
    args = ap.parse_args(argv)
    if len(args.pairs) % 2:
        ap.error("arguments come in CSV GCODE pairs")
    pairs = list(zip(args.pairs[::2], args.pairs[1::2]))
    results = [
        analyze(csv, gcode, args.config, args.accel) for csv, gcode in pairs
    ]
    sys.stdout.write(table(results))


if __name__ == "__main__":
    main()
