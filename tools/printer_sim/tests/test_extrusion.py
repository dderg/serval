from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from tools.motion_eval import accel_residual
from tools.printer_sim import extrusion, parts

DT_OUT = 25e-6
T0 = -0.01


@pytest.fixture(scope="module")
def ex():
    p = parts.load(
        parts.PARTS_DIR / "voron_48v_a.cfg", accel_residual.DEFAULT_CONFIG
    )
    return p.extrusion


def feed(speed_before, speed_after, at):
    """Filament position and speed (mm, mm/s) stepping speed at `at`."""

    def pos(t):
        t = np.asarray(t, dtype=float)
        return speed_before * np.maximum(t, 0.0) + (
            speed_after - speed_before
        ) * np.maximum(t - at, 0.0)

    def vel(t):
        t = np.asarray(t, dtype=float)
        return np.where(
            t < 0.0, 0.0, np.where(t < at, speed_before, speed_after)
        )

    return pos, vel


def run(ex, pos, vel, duration, ideal, substeps=extrusion.SUBSTEPS):
    k = extrusion.angle_per_mm(ex)
    n = int(round((duration - T0) / DT_OUT))
    return extrusion.run(
        ex,
        T0,
        DT_OUT,
        n,
        lambda t: k * pos(t),
        lambda t: k * vel(t),
        ideal=ideal,
        substeps=substeps,
    )


def analytic_out(ex, speed_before, speed_after, at, t):
    """e_out for u = e_out + tau e_out' from rest, u ramping at
    speed_before from 0 and speed_after from `at`."""
    tau = ex.time_constant

    def ramp(v, t0):
        u = np.maximum(t - t0, 0.0)
        return v * (u - tau * (1.0 - np.exp(-u / tau)))

    return ramp(speed_before, 0.0) + ramp(speed_after - speed_before, at)


def test_ideal_rotor_melt_is_the_first_order_lag_pressure_advance_assumes(ex):
    pos, vel = feed(5.0, 15.0, 0.1)
    res1 = run(ex, pos, vel, 0.4, True)
    res2 = run(ex, pos, vel, 0.4, True, substeps=2 * extrusion.SUBSTEPS)
    expected = analytic_out(ex, 5.0, 15.0, 0.1, res1.t)
    bound = 2.0 * np.abs(res1.e_out - res2.e_out).max() + 1e-9
    assert np.abs(res1.e_out - expected).max() <= bound
    assert bound < 1e-4 * expected[-1]


def test_steady_flow_equals_the_commanded_flow_with_the_real_motor(ex):
    speed = 20.0
    pos, vel = feed(speed, speed, 1.0)
    res = run(ex, pos, vel, 0.5, False)
    settled = res.t > 10.0 * ex.time_constant
    flow = res.flow[settled]
    assert (
        np.abs(flow - speed * ex.filament_area).max()
        <= 1e-3 * speed * ex.filament_area
    )
    force = speed * ex.filament_area * ex.back_pressure
    assert res.force[settled].mean() == pytest.approx(force, rel=1e-3)


def test_flow_never_exceeds_the_hotend_cap(ex):
    capped = dataclasses.replace(ex, max_flow=10.0)
    pos, vel = feed(2.0, 8.0, 0.05)
    res = run(capped, pos, vel, 0.3, True)
    assert res.flow.max() <= capped.max_flow * (1.0 + 1e-9)
    assert res.flow.max() >= capped.max_flow * (1.0 - 1e-6)


def test_extruded_volume_is_the_filament_volume(ex):
    """At rest the melt drains the compression, so the nozzle has passed
    the commanded filament, up to the rotor's static detent error
    asin(T_d / (K I)) / p radians."""
    pos, vel = feed(10.0, 0.0, 0.1)
    res = run(ex, pos, vel, 0.1 + 20.0 * ex.time_constant, False)
    commanded = pos(res.t[-1]) * ex.filament_area
    e = extrusion.electrical(ex)
    detent = math.asin(e.detent_torque / e.peak_torque) / e.pole_pairs
    allowed = (
        detent * ex.lever * 1e3 * ex.filament_area + math.exp(-15.0) * commanded
    )
    assert abs(res.volume[-1] - commanded) <= allowed
