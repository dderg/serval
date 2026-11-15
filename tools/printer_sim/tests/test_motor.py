"""Pole slip against the analytic torque curve: each rotor drives a rigid
inertia (no belts), commanded to accelerate at a constant rate after a
raised-cosine ramp, so its load torque is J * alpha."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from scipy import optimize

from tools.motion_eval import accel_residual
from tools.printer_sim import mechanics, motor, parts, sim

NATURAL_HZ = 20.0
RAMP_PERIODS = 4
ZETA = 0.1
DT_OUT = 25e-6


@pytest.fixture(scope="module", params=["voron_48v_a", "voron_stock_24v_a"])
def electrical(request):
    p = parts.load(
        parts.PARTS_DIR / (request.param + ".cfg"),
        accel_residual.DEFAULT_CONFIG,
    )
    return dataclasses.replace(
        motor.Electrical.from_parts(p), detent_torque=0.0
    )


def rigid_rotor_model(e):
    w = 2.0 * math.pi * NATURAL_HZ
    inertia = e.magnetic_stiffness / (w * w)
    zero = np.zeros((2, 2))
    system = mechanics.System(
        mass=np.full(2, inertia),
        stiffness=zero,
        damping=zero,
        radius=1.0,
        rotors=(0, 1),
        magnetic_stiffness=e.magnetic_stiffness,
        motor_damping=2.0 * ZETA / w * e.magnetic_stiffness,
        sensor=zero,
        nozzle=zero,
    )
    return sim.Model(system, e), inertia


def ramp_time():
    return RAMP_PERIODS / NATURAL_HZ


def ramp_residual():
    """Free vibration a raised-cosine ramp of the acceleration leaves, as a
    fraction of the static load angle: the ramp's spectrum at the natural
    frequency, pi^2 / |pi^2 - (w T)^2|."""
    wt = 2.0 * math.pi * NATURAL_HZ * ramp_time()
    return math.pi**2 / abs(math.pi**2 - wt * wt)


def accelerating(alpha):
    tr = ramp_time()

    def speed(t):
        t = np.asarray(t)
        up = alpha * (t / 2.0 - tr / (2.0 * math.pi) * np.sin(math.pi * t / tr))
        on = alpha * tr / 2.0 + alpha * (t - tr)
        return np.where(t < 0.0, 0.0, np.where(t < tr, up, on))

    def angle(t):
        t = np.asarray(t)
        up = alpha * (
            t * t / 4.0
            + tr * tr / (2.0 * math.pi**2) * (np.cos(math.pi * t / tr) - 1.0)
        )
        at_tr = alpha * (tr * tr / 4.0 - tr * tr / math.pi**2)
        on = at_tr + alpha * tr / 2.0 * (t - tr) + 0.5 * alpha * (t - tr) ** 2
        return np.where(t < 0.0, 0.0, np.where(t < tr, up, on))

    return angle, speed


def run(model, angle, speed, duration):
    p = model.electrical.pole_pairs

    def command(t):
        a = p * angle(t)
        return np.vstack([a, a])

    def rate(t):
        w = p * speed(t)
        return np.vstack([w, w])

    n = int(round(duration / DT_OUT))
    return sim.run(model, 0.0, DT_OUT, n, command, rate)


def first_slip(res):
    over = np.abs(res.delta[:, 0]) > math.pi
    return int(np.argmax(over)) if over.any() else None


def plateau_end(e):
    """Highest speed at which the sine-limited pull-out torque still equals
    the current-limited K*I."""
    full = e.peak_torque

    def short(w):
        return motor.pullout_torque(e, w, e.supply_voltage) - full * (
            1.0 - 1e-6
        )

    return optimize.brentq(short, 1e-3, 1e4, xtol=1e-3)


def test_below_the_current_limit_holds_above_it_slips(electrical):
    e = electrical
    model, inertia = rigid_rotor_model(e)
    margin = 3.0 * ramp_residual()
    end = plateau_end(e)
    alpha_hold = (1.0 - margin) * e.peak_torque / inertia
    t_end = (
        ramp_time() + (0.9 * end - alpha_hold * ramp_time() / 2.0) / alpha_hold
    )
    res = run(model, *accelerating(alpha_hold), t_end)
    assert first_slip(res) is None
    assert np.abs(res.delta[:, 0]).max() < math.pi / 2.0

    alpha_slip = (1.0 + margin) * e.peak_torque / inertia
    angle, speed = accelerating(alpha_slip)
    res = run(model, angle, speed, t_end)
    i = first_slip(res)
    assert i is not None
    assert speed(res.t[i]) < end


def test_slip_speed_lies_inside_the_voltage_limited_torque_bracket(electrical):
    e = electrical
    model, inertia = rigid_rotor_model(e)
    load = 0.6 * e.peak_torque
    alpha = load / inertia
    angle, speed = accelerating(alpha)

    def crossing(volts):
        return optimize.brentq(
            lambda w: motor.pullout_torque(e, w, volts) - load,
            plateau_end(e),
            1e4,
            xtol=1e-3,
        )

    w_sine = crossing(e.supply_voltage)
    w_square = crossing(motor.SQUARE_WAVE_FUNDAMENTAL * e.supply_voltage)
    tau_e = e.inductance / e.resistance
    lag = alpha * (1.0 / NATURAL_HZ + tau_e)
    duration = ramp_time() + (w_square + 2.0 * lag) / alpha
    res = run(model, angle, speed, duration)
    i = first_slip(res)
    assert i is not None
    w_slip = float(speed(res.t[i]))
    assert w_sine - lag <= w_slip <= w_square + lag


def test_a_slip_leaves_whole_electrical_cycles_lost(electrical):
    e = electrical
    model, inertia = rigid_rotor_model(e)
    alpha = 3.0 * e.peak_torque / inertia
    burst = 0.02
    settle_e_folds = 10.0
    hold = settle_e_folds / (ZETA * 2.0 * math.pi * NATURAL_HZ)

    def speed(t):
        t = np.asarray(t)
        return (
            alpha
            * np.clip(np.minimum(t, 2.0 * burst - t), 0.0, None)
            * (t < 2.0 * burst)
        )

    def angle(t):
        t = np.clip(np.asarray(t), 0.0, 2.0 * burst)
        up = 0.5 * alpha * np.minimum(t, burst) ** 2
        down_t = np.maximum(t - burst, 0.0)
        return up + alpha * burst * down_t - 0.5 * alpha * down_t**2

    res = run(model, angle, speed, 2.0 * burst + hold)
    delta_end = res.delta[-1, 0]
    cycles = round(delta_end / (2.0 * math.pi))
    assert cycles != 0
    assert abs(delta_end - 2.0 * math.pi * cycles) < math.pi * math.exp(
        -settle_e_folds
    )
    assert sim.pole_slips(res.delta[:, :1])[0] >= abs(cycles)
