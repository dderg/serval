from __future__ import annotations

import math

import numpy as np
import pytest

from tools.motion_eval import accel_residual
from tools.printer_sim import mechanics, motor, parts


def load(name):
    p = parts.load(
        parts.PARTS_DIR / (name + ".cfg"), accel_residual.DEFAULT_CONFIG
    )
    return p, motor.Electrical.from_parts(p)


@pytest.fixture(scope="module")
def reading_a():
    p, e = load("voron_48v_a")
    cal = mechanics.calibrate(p, e.magnetic_stiffness)
    return p, e, mechanics.build(p, cal, e.magnetic_stiffness)


@pytest.fixture(scope="module")
def reading_b():
    p, e = load("voron_48v_b")
    cal = mechanics.calibrate(p, e.magnetic_stiffness)
    return p, e, mechanics.build(p, cal, e.magnetic_stiffness)


def rigid_translation(system, x, y):
    q = np.zeros(system.n)
    q[mechanics.ROTOR_A] = (x + y) / system.radius
    q[mechanics.ROTOR_B] = (x - y) / system.radius
    q[mechanics.CARRIAGE_X] = x
    q[mechanics.GANTRY_Y] = y
    if system.n > mechanics.TOOLHEAD_X:
        q[mechanics.TOOLHEAD_X] = x
        q[mechanics.TOOLHEAD_Y] = y
    return q


@pytest.mark.parametrize("which", ["reading_a", "reading_b"])
def test_corexy_rigid_motion_stretches_nothing(which, request):
    _, _, system = request.getfixturevalue(which)
    k = system.stiffness
    assert np.array_equal(k, k.T)
    assert np.linalg.eigvalsh(k).min() >= -1e-9 * np.abs(k).max()
    for x, y in ((1e-3, 0.0), (0.0, 1e-3), (2e-3, -7e-4)):
        force = k @ rigid_translation(system, x, y)
        assert np.abs(force).max() <= 1e-12 * np.abs(k).max() * 1e-3


@pytest.mark.parametrize("which", ["reading_a", "reading_b"])
def test_x_and_y_commands_do_not_cross(which, request):
    _, _, system = request.getfixturevalue(which)
    freqs = np.geomspace(5.0, 2000.0, 200)
    k, c = mechanics.linearized(system)
    m = np.diag(system.mass)
    for axis, other in (("x", 1), ("y", 0)):
        direction = mechanics.command_direction(system, axis)
        for f in freqs:
            w = 2.0 * math.pi * f
            drive = (
                system.magnetic_stiffness + 1j * w * system.motor_damping
            ) * direction
            response = np.linalg.solve(k - w * w * m + 1j * w * c, drive)
            assert (
                abs(system.sensor[other] @ response)
                <= 1e-12 * np.abs(response).max()
            )


def test_reading_a_puts_the_response_peaks_on_the_measured_frequencies(
    reading_a,
):
    p, _, system = reading_a
    assert mechanics.peak_frequency(system, "x") == pytest.approx(
        p.resonance_x, rel=mechanics.PEAK_RTOL
    )
    assert mechanics.peak_frequency(system, "y") == pytest.approx(
        p.resonance_y, rel=mechanics.PEAK_RTOL
    )
    assert mechanics.response_peaks(system, "x")[0][0] == pytest.approx(
        p.resonance_x, rel=1e-6
    )


def test_reading_b_puts_undamped_modes_on_the_measured_frequencies(reading_b):
    p, _, system = reading_b
    f = mechanics.modes(system)
    for target in (p.resonance_x, p.resonance_y):
        assert np.abs(f - target).min() <= 1e-9 * target


@pytest.mark.parametrize("which", ["reading_a", "reading_b"])
def test_proportional_damping_gives_beta_w_over_2_on_every_mode(which, request):
    _, _, system = request.getfixturevalue(which)
    n = system.n
    k, c = mechanics.linearized(system)
    inv_m = 1.0 / system.mass
    a = np.zeros((2 * n, 2 * n))
    a[:n, n:] = np.eye(n)
    a[n:, :n] = -inv_m[:, None] * k
    a[n:, n:] = -inv_m[:, None] * c
    lam = np.linalg.eigvals(a)
    beta = system.motor_damping / system.magnetic_stiffness
    for f in mechanics.modes(system):
        w = 2.0 * math.pi * f
        zeta = beta * w / 2.0
        if zeta >= 1.0:
            continue
        expected = complex(-zeta * w, w * math.sqrt(1.0 - zeta * zeta))
        assert np.abs(lam - expected).min() <= 1e-7 * w


def test_damping_ratio_at_the_x_resonance_is_the_parts_value(reading_a):
    p, _, system = reading_a
    f = mechanics.modes(system)
    mode = f[np.argmin(np.abs(f - p.resonance_x))]
    assert mechanics.damping_ratio(system, mode) == pytest.approx(
        p.damping_ratio, rel=1e-12
    )


def test_stock_motors_cannot_reach_200_hz_because_the_rotor_alone_is_lower():
    p, e = load("voron_stock_24v_a")
    rotor_alone = math.sqrt(e.magnetic_stiffness / p.motor.rotor_inertia) / (
        2.0 * math.pi
    )
    assert rotor_alone < p.resonance_x
    with pytest.raises(mechanics.CalibrationError, match="toolhead_mass"):
        mechanics.calibrate(p, e.magnetic_stiffness)


def test_local_spring_rank_one_solution_is_exact(reading_b):
    p, e, system = reading_b
    w = 2.0 * math.pi * p.resonance_x
    k, _ = mechanics.linearized(system)
    a = k - w * w * np.diag(system.mass)
    s = np.linalg.svd(a, compute_uv=False)
    assert s[-1] <= 1e-9 * s[0]
