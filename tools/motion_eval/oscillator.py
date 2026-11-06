"""Ringing proxy: a damped oscillator driven by the executed axis motion.

The toolhead is a mass on a spring of natural frequency f and damping ratio
zeta, attached to the executed (shaped) carriage position x_b(t). Its lag
e = x_toolhead - x_b obeys e'' + 2 zeta w e' + w^2 e = -x_b''. Between two
samples the carriage is taken to move linearly, which is exact for the
particular solution e = 0, so e evolves freely inside each sample interval
and only the carriage's velocity change at a sample kicks e' by -dv. That
makes e a second-order IIR filter of the per-sample velocity changes.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.signal import lfilter


def lag(pos, dt, freq, zeta):
    """Toolhead lag e (mm) at every sample of the carriage position pos."""
    w = 2.0 * math.pi * freq
    wd = w * math.sqrt(1.0 - zeta * zeta)
    r = math.exp(-zeta * w * dt)
    theta = wd * dt
    slope = np.diff(pos) / dt
    kicks = -np.diff(slope, prepend=0.0, append=slope[-1])
    return lfilter(
        [0.0, r * math.sin(theta) / wd],
        [1.0, -2.0 * r * math.cos(theta), r * r],
        kicks,
    )


def residuals(e, t0, dt, windows, freq):
    """Free-vibration amplitude at the start of each window (a span in which
    the carriage moves at constant velocity): the largest |e| over its
    first period. Windows shorter than one period are skipped."""
    period = 1.0 / freq
    out = []
    for start, end in windows:
        if end - start < period:
            continue
        i0 = int(math.ceil((start - t0) / dt))
        i1 = int(math.floor((start + period - t0) / dt)) + 1
        if i0 < 0 or i1 > len(e):
            continue
        out.append(float(np.abs(e[i0:i1]).max()))
    return np.array(out)


def constant_velocity_windows(seg, support):
    """Spans where the executed motion of every axis is at constant
    velocity: cruise phases of the commanded moves, shortened by the
    kernel support (shift_lo, shift_hi) on each side."""
    shift_lo, shift_hi = support
    cruise = seg.accel == 0.0
    starts = seg.t0[cruise] - shift_lo
    ends = seg.t0[cruise] + seg.dur[cruise] - shift_hi
    keep = ends > starts
    return list(zip(starts[keep], ends[keep]))
