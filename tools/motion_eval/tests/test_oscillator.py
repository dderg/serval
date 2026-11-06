from __future__ import annotations

import cmath
import math

import numpy as np
import pytest

from klippy.extras import shaper_defs
from tools.motion_eval import oscillator

FREQ = 50.0
ZETA = 0.1
SPEED = 100.0
DT = 25e-6
STEP_AT = 0.01
DURATION = 0.3


def shaped_step(pulses):
    """Carriage position for a velocity step of SPEED at STEP_AT, split
    into the shaper's pulses (amplitude, delay)."""
    t = DT * np.arange(int(DURATION / DT) + 1)
    pos = sum(a * SPEED * np.maximum(t - STEP_AT - d, 0.0) for a, d in pulses)
    return t, pos


def envelope(pulses, freq, at):
    """Free-vibration amplitude of the oscillator at time `at`, after every
    pulse: each velocity kick a*SPEED at STEP_AT + d adds a decaying sine of
    amplitude a*SPEED/wd."""
    w = 2.0 * math.pi * freq
    wd = w * math.sqrt(1.0 - ZETA * ZETA)
    phasor = sum(
        a * cmath.exp((ZETA * w - 1j * wd) * (STEP_AT + d)) for a, d in pulses
    )
    return SPEED * abs(phasor) * math.exp(-ZETA * w * at) / wd


def zv_pulses():
    A, T = shaper_defs.get_zv_shaper(FREQ, ZETA)
    return [(a / sum(A), d) for a, d in zip(A, T)]


def residual(pulses, freq):
    t, pos = shaped_step(pulses)
    e = oscillator.lag(pos, DT, freq, ZETA)
    start = STEP_AT + max(d for _, d in pulses) + DT
    return oscillator.residuals(e, 0.0, DT, [(start, DURATION)], freq)[0], start


def test_zv_shaper_cancels_its_own_mode():
    unshaped, start = residual([(1.0, 0.0)], FREQ)
    shaped, _ = residual(zv_pulses(), FREQ)
    discretization = 2.0 * math.pi * FREQ * DT
    assert unshaped == pytest.approx(
        envelope([(1.0, 0.0)], FREQ, start),
        rel=1.0 - math.exp(-2 * math.pi * ZETA),
    )
    assert shaped <= unshaped * discretization


@pytest.mark.parametrize("ratio", (0.8, 1.2, 2.5))
def test_mismatched_mode_rings_as_predicted(ratio):
    freq = FREQ * ratio
    pulses = zv_pulses()
    measured, start = residual(pulses, freq)
    predicted = envelope(pulses, freq, start)
    decay_over_period = math.exp(-2.0 * math.pi * ZETA)
    discretization = 2.0 * math.pi * freq * DT
    assert predicted * decay_over_period * (1.0 - discretization) <= measured
    assert measured <= predicted * (1.0 + discretization)
