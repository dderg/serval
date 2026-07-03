from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import viz_pipeline  # noqa: E402


def _one_piece(c0, c1, c2, c3, t_end, axis="x"):
    # A single cubic piece pos(t) = c0 + c1*t + c2*t^2 + c3*t^3 on one axis; the
    # other axis is held at rest. The visualizer reads these directly -- the
    # trajectory the firmware runs -- and differentiates them analytically.
    moving = [[0.0, t_end, c0, c1, c2, c3]]
    still = [[0.0, t_end, 0.0, 0.0, 0.0, 0.0]]
    return {
        "traj_x_pieces": moving if axis == "x" else still,
        "traj_y_pieces": moving if axis == "y" else still,
        "traj_t_end": t_end,
    }


def test_constant_velocity_gives_flat_speed_and_zero_higher_orders():
    # x(t) = v*t  ->  velocity v, acceleration 0, jerk 0.
    snap = _one_piece(0.0, 20.0, 0.0, 0.0, t_end=1.0)

    series = viz_pipeline._build_time_series(snap)

    assert np.allclose(series["vel"]["X"], 20.0)
    assert np.allclose(series["vel"]["Y"], 0.0)
    assert np.allclose(series["v_scalar"], 20.0)
    assert np.allclose(series["a_scalar"], 0.0)
    assert np.allclose(series["j_scalar"], 0.0)


def test_constant_acceleration_piece_is_exact():
    # x(t) = 0.5*a*t^2  ->  c2 = a/2, acceleration a (constant), jerk 0.
    a = 1000.0
    snap = _one_piece(0.0, 0.0, 0.5 * a, 0.0, t_end=0.1)

    series = viz_pipeline._build_time_series(snap)

    assert np.allclose(series["acc"]["X"], a)
    assert np.allclose(series["a_scalar"], a)
    assert np.allclose(series["j_scalar"], 0.0)


def test_constant_jerk_piece_is_exact():
    # x(t) = j/6 * t^3  ->  c3 = j/6, jerk j (constant).
    j = 50000.0
    snap = _one_piece(0.0, 0.0, 0.0, j / 6.0, t_end=0.05)

    series = viz_pipeline._build_time_series(snap)

    assert np.allclose(series["jerk"]["X"], j)
    assert np.allclose(series["j_scalar"], j)
