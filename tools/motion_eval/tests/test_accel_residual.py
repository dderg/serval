"""accel_residual against synthetic accelerometer records.

The toolhead is the carriage plus one damped mode per axis
(oscillator.lag). It is sampled like an ADXL345 at its default rate
(klippy/extras/adxl345.py line 288, LSB from SCALE_XY at line 40), with:
- gravity on a tilted sensor;
- white noise and quantisation;
- `%.6f` timestamps and an unknown time offset.
It is written as ACCELEROMETER_MEASURE writes it."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from tools.motion_eval import accel_residual, oscillator, printer_test

DT = accel_residual.DT
RATE_HZ = 3200.0
LSB_MM_S2 = 0.003774 * 9806.65
NOISE_MM_S2 = 150.0
GRAVITY = {"x": 120.0, "y": -250.0, "z": 9806.65}
OFFSET_S = 1234.567891234
MODES = {"x": (0.9, 0.05), "y": (2.5, 0.05)}
LEAD_S = printer_test.QUIET_LEAD_MS / 1000.0
TAIL_S = printer_test.QUIET_TAIL_MS / 1000.0
TIMESTAMP_ROUNDING_S = 0.5e-6
SIGMAS = 5.0
SEED = 20261107
SETTING = "scv50"
ROW = "| corners_%s |" % SETTING


@dataclasses.dataclass
class Case:
    text: str
    plan: accel_residual.Plan
    lag: dict
    toolhead: dict


def noise_sigma():
    """White noise plus the quantisation step's uniform error."""
    return math.sqrt(NOISE_MM_S2**2 + LSB_MM_S2**2 / 12.0)


def mode(plan, axis):
    ratio, zeta = MODES[axis]
    return ratio * plan.freqs[axis], zeta


def toolhead_acc(e, freq, zeta):
    """x_toolhead'' = x_b'' + e'' = -w^2 e - 2 zeta w e'."""
    w = 2.0 * math.pi * freq
    return -w * w * e - 2.0 * zeta * w * np.gradient(e, DT)


@pytest.fixture(scope="module")
def case():
    text, raw = printer_test.render("corners", SETTING)
    plan = accel_residual.plan_from_capture(
        raw.capture,
        accel_residual.parse_headers(text),
        accel_residual.shaper_freqs(printer_test.CONFIG),
    )
    lag, toolhead = {}, {}
    for axis in accel_residual.AXES:
        freq, zeta = mode(plan, axis)
        lag[axis] = oscillator.lag(plan.pos[axis], DT, freq, zeta)
        toolhead[axis] = toolhead_acc(lag[axis], freq, zeta)
    return Case(text, plan, lag, toolhead)


def sample_times(plan):
    return np.arange(
        plan.span[0] - LEAD_S, plan.span[1] + TAIL_S, 1.0 / RATE_HZ
    )


def write_csv(path, plan_t, acc, noise=True, clip_x=None):
    rng = np.random.default_rng(SEED)
    columns = [plan_t + OFFSET_S]
    for axis in "xyz":
        values = GRAVITY[axis] + acc.get(axis, np.zeros(len(plan_t)))
        if noise:
            values = values + rng.normal(0.0, NOISE_MM_S2, len(plan_t))
        if axis == "x" and clip_x is not None:
            values = np.clip(values, -clip_x, clip_x)
        columns.append(np.round(values / LSB_MM_S2) * LSB_MM_S2)
    np.savetxt(
        path,
        np.stack(columns, 1),
        fmt="%.6f",
        delimiter=",",
        header=accel_residual.CSV_HEADER[1:],
        comments="#",
    )
    return path


def sampled(plan, series, plan_t):
    return np.interp(plan_t, plan.grid, series, left=0.0, right=0.0)


def ringing(case, plan_t):
    return {a: sampled(case.plan, case.toolhead[a], plan_t) for a in "xy"}


def ringing_csv(case, path, noise=True):
    plan_t = sample_times(case.plan)
    return write_csv(path, plan_t, ringing(case, plan_t), noise=noise)


def offset_tolerance(plan):
    """Timestamp rounding, plus SIGMAS times the Cramer-Rao bound of a
    delay estimate: noise sigma over the rms slope of the low-passed plan
    at the sample times."""
    plan_t = sample_times(plan)
    cutoff = accel_residual._align_cutoff(plan, RATE_HZ)
    slope2 = 0.0
    for axis in accel_residual.AXES:
        lp = accel_residual._lowpass(plan.acc[axis], 1.0 / DT, cutoff)
        slope = np.interp(plan_t, plan.grid, np.gradient(lp, DT))
        slope2 += float(slope @ slope)
    return TIMESTAMP_ROUNDING_S + SIGMAS * noise_sigma() / math.sqrt(slope2)


def run_low_level(path, plan):
    meas = accel_residual.read_csv(path)
    alignment = accel_residual.align(meas, plan)
    bias, noise = accel_residual.bias_and_noise(meas, plan, alignment.offset)
    windows = {
        axis: accel_residual.window_residuals(
            meas, plan, alignment.offset, axis, bias[axis], noise[axis]
        )
        for axis in accel_residual.AXES
    }
    return meas, alignment, bias, noise, windows


def window_bounds(case, meas, alignment, noise, axis, windows, timing):
    """(found, expected, allowed) per window. expected is
    oscillator.residuals on the same window. A fitted window may differ
    by SIGMAS times the least-squares amplitude error, sigma sqrt(2/N_eff)
    with N_eff the decay-weighted sample count, plus the effect of the
    offset error and of oscillator.residuals' grid. A fallback window holds
    the measured peak over one configured period: at most the ringing
    peak e'' plus SIGMAS sigma of noise, over w_cfg^2."""
    plan = case.plan
    freq, zeta = mode(plan, axis)
    w = 2.0 * math.pi * freq
    w_cfg = 2.0 * math.pi * plan.freqs[axis]
    sigma = noise_sigma()
    stops = dict(plan.windows)
    plan_t = meas.t - alignment.offset
    out = []
    for found in windows:
        stop = stops[found.start]
        (expected,) = oscillator.residuals(
            case.lag[axis], plan.t0, DT, [(found.start, stop)], freq
        )
        tau = plan_t[(plan_t >= found.start) & (plan_t <= stop)] - found.start
        n_eff = float(np.sum(np.exp(-2.0 * zeta * w * tau)))
        amp_error = SIGMAS * sigma * math.sqrt(2.0 / n_eff)
        peak_acc = expected * w * w
        if found.fitted:
            allowed = amp_error / (w * w) + expected * (
                w * timing + 1.0 - math.cos(0.5 * w * DT)
            )
        else:
            assert (
                peak_acc < accel_residual.NOISE_SIGMAS * noise[axis] + amp_error
            ), (axis, found, expected)
            allowed = (peak_acc + SIGMAS * sigma) / (w_cfg * w_cfg)
            expected = 0.0
        out.append((found, expected, allowed))
    return out


def test_recovers_offset_and_residuals(case, tmp_path):
    plan = case.plan
    clean = ringing_csv(case, tmp_path / "clean.csv", noise=False)
    resampling = abs(run_low_level(clean, plan)[1].offset - OFFSET_S)
    path = ringing_csv(case, tmp_path / "ring.csv")
    meas, alignment, bias, noise, windows = run_low_level(path, plan)
    timing = resampling + offset_tolerance(plan)
    assert abs(alignment.offset - OFFSET_S) <= timing
    for axis in accel_residual.AXES:
        assert bias[axis] == pytest.approx(
            GRAVITY[axis],
            abs=SIGMAS * noise_sigma() / math.sqrt(len(meas.t)) + LSB_MM_S2,
        )
        assert noise[axis] == pytest.approx(
            noise_sigma(),
            rel=SIGMAS / math.sqrt(2.0 * accel_residual.MIN_QUIET_SAMPLES),
        )
        assert windows[axis]
        bounds = window_bounds(
            case, meas, alignment, noise, axis, windows[axis], timing
        )
        for found, expected, allowed in bounds:
            assert abs(found.amplitude_mm - expected) <= allowed, (
                axis,
                found,
                expected,
                allowed,
            )
        assert any(found.fitted for found, _, _ in bounds)


def test_cli_summary_matches_oscillator(case, tmp_path, capsys):
    gcode = tmp_path / "corners.gcode"
    gcode.write_text(case.text)
    path = ringing_csv(case, tmp_path / "ring.csv")
    clean = ringing_csv(case, tmp_path / "clean.csv", noise=False)
    resampling = abs(run_low_level(clean, case.plan)[1].offset - OFFSET_S)
    timing = resampling + offset_tolerance(case.plan)
    meas, alignment, _, noise, windows = run_low_level(path, case.plan)
    result = accel_residual.analyze(path, gcode, printer_test.CONFIG)
    for axis in accel_residual.AXES:
        bounds = window_bounds(
            case, meas, alignment, noise, axis, windows[axis], timing
        )
        expected = [e for _, e, _ in bounds]
        allowed = max(a for _, _, a in bounds) * accel_residual.UM_PER_MM
        p50, p99, peak = result["axes"][axis]["residual_um"]
        um = accel_residual.UM_PER_MM
        assert abs(p50 - np.percentile(expected, 50) * um) <= allowed
        assert abs(p99 - np.percentile(expected, 99) * um) <= allowed
        assert abs(peak - max(expected) * um) <= allowed
    accel_residual.main([str(path), str(gcode), str(clean), str(gcode)])
    rows = [
        line
        for line in capsys.readouterr().out.splitlines()
        if line.startswith(ROW)
    ]
    assert len(rows) == 2


def test_without_ringing_reports_noise_floor(case, tmp_path):
    plan = case.plan
    plan_t = sample_times(plan)
    acc = {a: sampled(plan, plan.acc[a], plan_t) for a in "xy"}
    path = write_csv(tmp_path / "quiet.csv", plan_t, acc)
    _, alignment, _, _, windows = run_low_level(path, plan)
    assert abs(alignment.offset - OFFSET_S) <= offset_tolerance(plan)
    for axis in accel_residual.AXES:
        w_cfg = 2.0 * math.pi * plan.freqs[axis]
        floor = SIGMAS * noise_sigma() / (w_cfg * w_cfg)
        assert windows[axis]
        assert not any(found.fitted for found in windows[axis])
        assert max(found.amplitude_mm for found in windows[axis]) <= floor


def garbage(case, tmp_path, kind):
    plan = case.plan
    plan_t = sample_times(plan)
    acc = ringing(case, plan_t)
    path = tmp_path / (kind + ".csv")
    if kind == "noise":
        return write_csv(path, plan_t, {})
    if kind == "truncated":
        keep = plan_t < 0.5 * (plan.span[0] + plan.span[1])
        return write_csv(
            path, plan_t[keep], {a: v[keep] for a, v in acc.items()}
        )
    if kind == "stall":
        stall = int(0.02 * RATE_HZ)
        mid = len(plan_t) // 2
        moved = {
            a: np.concatenate((v[:mid], np.zeros(stall), v[mid:-stall]))
            for a, v in acc.items()
        }
        return write_csv(path, plan_t, moved)
    if kind == "flipped_x":
        return write_csv(path, plan_t, {"x": -acc["x"], "y": acc["y"]})
    if kind == "repeated":
        span = plan_t[-1] - plan_t[0] + 1.0 / RATE_HZ
        twice = np.concatenate((plan_t, plan_t + span))
        return write_csv(
            path, twice, {a: np.concatenate((v, v)) for a, v in acc.items()}
        )
    if kind == "lost_samples":
        keep = np.ones(len(plan_t), bool)
        keep[len(plan_t) // 3 : len(plan_t) // 3 + 5] = False
        return write_csv(
            path, plan_t[keep], {a: v[keep] for a, v in acc.items()}
        )
    if kind == "clipped":
        return write_csv(path, plan_t, acc, clip_x=0.8 * np.abs(acc["x"]).max())
    if kind == "header":
        write_csv(path, plan_t, acc)
        lines = path.read_text().splitlines()
        path.write_text("\n".join(["#t,x,y,z"] + lines[1:]) + "\n")
        return path
    raise AssertionError(kind)


@pytest.mark.parametrize(
    "kind, message",
    [
        ("noise", "sync motion correlates"),
        ("truncated", "does not cover"),
        ("stall", "does not follow the global offset"),
        ("flipped_x", "axes_map"),
        ("repeated", "ambiguous alignment"),
        ("lost_samples", "irregular sample intervals"),
        ("clipped", "clipped"),
        ("header", "is not ACCELEROMETER_MEASURE"),
    ],
)
def test_bad_records_fail_loudly(case, tmp_path, kind, message):
    path = garbage(case, tmp_path, kind)
    with pytest.raises(accel_residual.AccelResidualError, match=message):
        run_low_level(path, case.plan)


@pytest.mark.parametrize(
    "value, section",
    [
        ("none", None),
        ("floor", "[corner_deviation]\n"),
        ("0.07", "[corner_deviation]\nmax_deviation: 0.07\n"),
        ("bound51500", "[corner_deviation]\nmax_corner_accel: 51500\n"),
    ],
)
def test_corner_deviation_header(value, section):
    assert accel_residual.corner_deviation_section(value) == section


@pytest.mark.parametrize(
    "value", ["", "Floor", "bound", "bound5e4", "cap0.07", "7", "0.07mm"]
)
def test_unknown_corner_deviation_header_fails(value):
    with pytest.raises(accel_residual.AccelResidualError, match="header"):
        accel_residual.corner_deviation_section(value)


def test_generated_settings_parse():
    for _, value in printer_test.CORNER_SETTINGS.values():
        accel_residual.corner_deviation_section(value)
