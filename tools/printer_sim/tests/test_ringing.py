"""Analytic ringing of one mode: rotors held kinematically on the command
(ideal motors), so each axis is the carriage on its two belts."""

from __future__ import annotations

import math

import numpy as np
import pytest

from klippy.extras import shaper_defs
from tools.motion_eval import accel_residual
from tools.printer_sim import mechanics, motor, parts, sim

FREQ = 50.0
ZETA = 0.1
ACCEL = 5.0
ACCEL_END = 0.05
DT_OUT = 25e-6
DURATION = 0.25
T0 = -0.01


@pytest.fixture(scope="module")
def model():
    p = parts.load(
        parts.PARTS_DIR / "voron_48v_a.cfg", accel_residual.DEFAULT_CONFIG
    )
    e = motor.Electrical.from_parts(p)
    mass = 0.4
    w = 2.0 * math.pi * FREQ
    cal = mechanics.Calibrated(
        belt_stiffness=0.5 * mass * w * w,
        toolhead_mass=mass,
        gantry_mass=0.0,
        damping_time=2.0 * ZETA / w,
    )
    system = mechanics.build(p, cal, e.magnetic_stiffness)
    return sim.Model(system, e, ideal_motors=True)


def ramp(t, delays=(0.0,), weights=(1.0,)):
    """Position, velocity of accelerating at ACCEL until ACCEL_END, then
    cruising, as the sum of weighted delayed copies (an input shaper)."""
    pos = np.zeros_like(t)
    vel = np.zeros_like(t)
    for d, wgt in zip(delays, weights):
        u = t - d
        acc = np.clip(u, 0.0, ACCEL_END)
        pos += wgt * (
            0.5 * ACCEL * acc**2
            + ACCEL * ACCEL_END * np.maximum(u - ACCEL_END, 0.0)
        )
        vel += wgt * ACCEL * acc
    return pos, vel


def command_for(model, path):
    p = model.electrical.pole_pairs
    r = model.system.radius

    def command(t):
        x, _ = path(t)
        return np.vstack([p * x / r, p * x / r])

    def rate(t):
        _, v = path(t)
        return np.vstack([p * v / r, p * v / r])

    return command, rate


def run(model, path, substeps):
    command, rate = command_for(model, path)
    n = int(round((DURATION - T0) / DT_OUT))
    res = sim.run(model, T0, DT_OUT, n, command, rate, substeps=substeps)
    return res.t, res.nozzle[:, 0] - path(res.t)[0]


def step_response(t):
    w = 2.0 * math.pi * FREQ
    wd = w * math.sqrt(1.0 - ZETA * ZETA)
    tt = np.maximum(t, 0.0)
    h = 1.0 - np.exp(-ZETA * w * tt) * (
        np.cos(wd * tt) + ZETA / math.sqrt(1.0 - ZETA * ZETA) * np.sin(wd * tt)
    )
    return np.where(t > 0.0, h, 0.0)


def analytic_lag(t, delays=(0.0,), weights=(1.0,)):
    """m e'' + c e' + k e = -m a(t) for a(t) the sum of ACCEL steps up at
    each delay and down ACCEL_END later."""
    w = 2.0 * math.pi * FREQ
    out = np.zeros_like(t)
    for d, wgt in zip(delays, weights):
        out -= (
            wgt
            * ACCEL
            / (w * w)
            * (step_response(t - d) - step_response(t - d - ACCEL_END))
        )
    return out


def richardson_bound(model, path, substeps):
    """First-order integrator: error(h) = 2 (s(h) - s(h/2)) + O(h^2); the
    O(h^2) remainder is bounded by the next halving's difference."""
    _, s1 = run(model, path, substeps)
    _, s2 = run(model, path, 2 * substeps)
    _, s4 = run(model, path, 4 * substeps)
    return s1, 2.0 * np.abs(s1 - s2).max() + np.abs(s2 - s4).max()


def test_unshaped_ramp_rings_at_the_textbook_amplitude_and_decay(model):
    path = ramp
    s, bound = richardson_bound(model, path, sim.SUBSTEPS)
    t = T0 + DT_OUT * np.arange(1, len(s) + 1)
    expected = analytic_lag(t)
    assert np.abs(s - expected).max() <= bound
    w = 2.0 * math.pi * FREQ
    static = ACCEL / (w * w)
    assert bound < 1e-2 * static
    period = 1.0 / (FREQ * math.sqrt(1.0 - ZETA * ZETA))
    peaks = [
        np.abs(
            s[
                (t > ACCEL_END + k * period)
                & (t <= ACCEL_END + (k + 1) * period)
            ]
        ).max()
        for k in range(1, 6)
    ]
    decay = math.exp(-2.0 * math.pi * ZETA / math.sqrt(1.0 - ZETA * ZETA))
    for a, b in zip(peaks, peaks[1:]):
        assert b / a == pytest.approx(decay, abs=2.0 * bound / b)


def test_zv_at_the_true_frequency_cancels_the_residual(model):
    amps, times = shaper_defs.get_zv_shaper(FREQ, ZETA)
    weights = np.array(amps) / sum(amps)

    def shaped(t):
        return ramp(t, times, weights)

    s_plain, _ = richardson_bound(model, ramp, sim.SUBSTEPS)
    s_zv, bound = richardson_bound(model, shaped, sim.SUBSTEPS)
    t = T0 + DT_OUT * np.arange(1, len(s_zv) + 1)
    after = t > ACCEL_END + max(times)
    assert np.abs(analytic_lag(t, times, weights)[after]).max() <= 1e-15
    assert np.abs(s_zv[after]).max() <= bound
    assert (
        np.abs(s_zv[after]).max() <= 1e-2 * np.abs(s_plain[t > ACCEL_END]).max()
    )
