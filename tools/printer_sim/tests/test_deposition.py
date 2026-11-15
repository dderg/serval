from __future__ import annotations

import math

import numpy as np
import pytest

from tools.motion_eval import accel_residual
from tools.printer_sim import deposition, extrusion, gcodemoves, parts, surface

H = 0.2
W = 0.45
SPEED = 50.0
DT = 25e-6


@pytest.fixture(scope="module")
def ex():
    p = parts.load(
        parts.PARTS_DIR / "voron_48v_a.cfg", accel_residual.DEFAULT_CONFIG
    )
    return p.extrusion


def straight(length, flow_of_t):
    """Nozzle along +x at SPEED from (0, 0), flow (mm^3/s) by time."""
    t = np.arange(0.0, length / SPEED, DT)
    xy = np.stack([SPEED * t, np.zeros_like(t)], axis=-1)
    volume = np.concatenate([[0.0], np.cumsum(flow_of_t(t[1:]) * DT)])
    return t, xy, volume


def lay_one(xy, volume):
    grid = deposition.Grid.around(xy.min(axis=0), xy.max(axis=0))
    z = np.full(len(xy), H)
    return deposition.lay(grid, xy, z, volume, H, 1.0)


def measured_width(layers, x):
    grid = layers.grid
    xc, yc = grid.centres()
    col = int(np.argmin(np.abs(xc - x)))
    rows = np.flatnonzero(layers.covered(0)[:, col])
    return (rows[-1] - rows[0] + 1) * grid.dx


def test_deposited_volume_is_the_volume_the_nozzle_passed():
    t, xy, volume = straight(10.0, lambda t: 5.0 + 3.0 * np.sin(40.0 * t))
    xy = xy + np.stack([np.zeros_like(t), 0.3 * np.sin(7.0 * t)], axis=-1)
    layers = lay_one(xy, volume)
    assert layers.deposited[0] == pytest.approx(layers.flowed[0], rel=1e-12)
    assert layers.flowed[0] == pytest.approx(volume[-1], rel=1e-12)


def test_steady_line_has_the_analytic_width():
    """Bead width flow / (speed * h); cell centres sample each edge to
    within one cell."""
    flow = W * H * SPEED
    t, xy, volume = straight(10.0, lambda t: np.full_like(t, flow))
    layers = lay_one(xy, volume)
    for x in (3.0, 5.0, 7.0):
        assert abs(measured_width(layers, x) - W) <= layers.grid.dx
    peak = layers.fields[0].max()
    assert peak == pytest.approx(4.0 * H / math.pi, rel=0.05)


def test_feed_step_widens_the_bead_with_the_melt_time_constant(ex):
    """The extruder feed steps from w1 to w2 worth of filament; the bead
    follows w(t) = w2 + (w1 - w2) exp(-(t - t_step) / tau). The footprint
    (radius w/2) blurs the edge by at most |dw/dx| * w/2."""
    w1, w2 = 0.35, 0.55
    t_step = 0.1
    length = SPEED * (t_step + 8.0 * ex.time_constant)
    k = extrusion.angle_per_mm(ex)
    fil = lambda w: w * H * SPEED / ex.filament_area  # noqa: E731

    def pos(t):
        return fil(w1) * np.maximum(t, 0.0) + (fil(w2) - fil(w1)) * np.maximum(
            t - t_step, 0.0
        )

    def vel(t):
        return np.where(t < 0.0, 0.0, np.where(t < t_step, fil(w1), fil(w2)))

    n = int(round(length / SPEED / DT))
    res = extrusion.run(
        ex, -DT, DT, n, lambda t: k * pos(t), lambda t: k * vel(t), ideal=True
    )
    t = res.t
    xy = np.stack([SPEED * t, np.zeros_like(t)], axis=-1)
    layers = lay_one(xy, res.volume - res.volume[0])
    tau = ex.time_constant
    expected_w = lambda tt: np.where(  # noqa: E731
        tt < t_step,
        w1 - w1 * np.exp(-tt / tau),
        w2
        + (w1 * (1.0 - np.exp(-t_step / tau)) - w2)
        * np.exp(-(tt - t_step) / tau),
    )
    for tt in np.arange(t_step + 0.2 * tau, t_step + 5.0 * tau, 0.25 * tau):
        x = SPEED * tt
        slope = (
            abs(
                float(
                    np.gradient(
                        expected_w(np.array([tt - 1e-4, tt, tt + 1e-4])), 1e-4
                    )[1]
                )
            )
            / SPEED
        )
        allowed = layers.grid.dx + slope * 0.5 * w2
        assert abs(measured_width(layers, x) - float(expected_w(tt))) <= allowed


def test_injected_vibration_reads_back_as_ghosting_of_that_amplitude_and_wavelength():
    """A square outer wall walked at constant speed with a sinusoidal
    normal offset A sin(2 pi f t): the ghosting is A at spatial frequency
    f / v. A whole number of periods per measured wall leaves only the
    linear detrend's bias, below 1 / (pi N) of A for N periods."""
    amp, freq, speed, side = 0.02, 200.0, 300.0, 30.0
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    moves, caps = [], []
    t_start = 0.0
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        moves.append(
            gcodemoves.Move(a + (H,), b + (H,), 1.0, "WALL-OUTER", 0, k)
        )
        caps.append(
            {
                "print_time": t_start,
                "accel_t": 0.0,
                "cruise_t": side / speed,
                "decel_t": 0.0,
                "cruise_v": speed,
            }
        )
        t_start += side / speed
    t = np.arange(0.0, t_start, 1e-5)
    xy = np.empty((len(t), 2))
    for m, c in zip(moves, caps):
        a, b = np.array(m.start[:2]), np.array(m.end[:2])
        e = (b - a) / side
        n = np.array([e[1], -e[0]])
        sel = (t >= c["print_time"]) & (t < c["print_time"] + side / speed)
        tt = t[sel] - c["print_time"]
        xy[sel] = (
            a
            + np.outer(speed * tt, e)
            + np.outer(amp * np.sin(2.0 * math.pi * freq * tt), n)
        )
    width = np.full(len(t), W)
    walls = surface.walls(list(zip(moves, caps)), t, xy, width, W, 0.0)
    assert len(walls) == 4
    for w in walls:
        periods = (w.s[-1] - w.s[0]) * freq / speed
        a_meas, f_meas = w.ghosting()
        assert a_meas == pytest.approx(amp, rel=1.0 / (math.pi * periods))
        resolution = 1.0 / (w.s[-1] - w.s[0])
        assert abs(f_meas - freq / speed) <= 0.5 * resolution
