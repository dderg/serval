from __future__ import annotations

import math

import numpy as np
import pytest

from tools.motion_eval import accel_residual, analysis
from tools.printer_sim import drive, report

CASE = "corners_bounded81000"
SDS_FILTER_TIME = 0.000750
POLE_PAIRS = 50


@pytest.fixture(scope="module")
def planned():
    return report._drive(CASE, POLE_PAIRS)[1]


def test_step_command_holds_each_step_until_the_next(planned):
    d = planned
    for m, (times, positions) in enumerate(zip(d.step_times, d.step_positions)):
        cmd = d.step_command(times)[m]
        assert np.allclose(cmd, d._angle(positions[1:]), rtol=0.0, atol=1e-9)
        just_before = d.step_command(times - 1e-9)[m]
        assert np.allclose(
            just_before, d._angle(positions[:-1]), rtol=0.0, atol=1e-9
        )


def test_steps_stay_within_half_a_step_of_the_continuous_command(planned):
    """Klipper steps where the motion crosses a step midpoint, give or
    take MAX_STEPCOMPRESS_ERROR and clock rounding, so the held step
    position is within half a step of the motion plus speed times that
    slack. The executed motion is sampled every DT: linear interpolation
    adds at most accel * DT^2 / 8. Where the motor reverses, stepcompress
    drops a step and its reverse step closer than SDS_FILTER_TIME
    (stepcompress.c), so for that long around a reversal the held
    position may be a whole step off."""
    d = planned
    t = d.t[::4]
    per_mm = d._angle(1.0)
    vel = d.velocity[::4]
    acc = np.gradient(d.velocity, drive.DT, axis=0)
    slack = (
        analysis.MAX_STEPCOMPRESS_ERROR
        + analysis.CLOCK_ROUNDING_TICKS / d.clock_freq
    )
    diff = np.abs(d.step_command(t) - d.continuous_command(t)) / per_mm
    for m, sign in enumerate((1.0, -1.0)):
        step_dist = np.abs(np.diff(d.step_positions[m])).max()
        motor_vel = vel[:, 0] + sign * vel[:, 1]
        speed = np.abs(motor_vel)
        flips = t[1:][np.signbit(motor_vel[1:]) != np.signbit(motor_vel[:-1])]
        near = np.zeros(len(t), dtype=bool)
        for f in flips:
            near |= np.abs(t - f) <= SDS_FILTER_TIME
        acc_max = np.abs(acc[:, 0] + sign * acc[:, 1]).max()
        allowed = (
            np.where(near, 1.0, 0.5) * step_dist
            + speed * slack
            + acc_max * slack * slack
            + acc_max * drive.DT**2 / 8.0
            + analysis.FLOAT_TOLERANCE_MM
        )
        assert (diff[m] <= allowed).all(), float(np.max(diff[m] - allowed))


def test_reference_ends_at_rest_for_the_settling_tail(planned):
    d = planned
    tail = d.reference[-int(math.ceil(drive.SETTLE_S / drive.DT)) :]
    assert np.ptp(tail, axis=0).max() == 0.0
    assert d.windows
    assert accel_residual.DT == drive.DT
