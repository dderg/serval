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


def _direct_eval(coeffs, tau):
    # Ground truth independent of viz_pipeline's Horner evaluation: plain
    # term-by-term power sums for pos and its first three derivatives.
    pos = sum(c * tau**k for k, c in enumerate(coeffs))
    vel = sum(k * c * tau ** (k - 1) for k, c in enumerate(coeffs) if k >= 1)
    acc = sum(
        k * (k - 1) * c * tau ** (k - 2) for k, c in enumerate(coeffs) if k >= 2
    )
    jerk = sum(
        k * (k - 1) * (k - 2) * c * tau ** (k - 3)
        for k, c in enumerate(coeffs)
        if k >= 3
    )
    return pos, vel, acc, jerk


# Three consecutive pieces of increasing degree, each row a different length
# (4, 6, 10 floats): linear, cubic, degree-7. Position and velocity are
# continuous at every seam so the trajectory is physically sane, but the row
# widths are deliberately ragged -- exactly what lands once the writer emits
# variable-degree pieces.
_RAGGED_X_PIECES = [
    [0.0, 1.0, 0.0, 10.0],
    [1.0, 2.0, 10.0, 10.0, 5.0, 2.0],
    [2.0, 3.0, 27.0, 26.0, 1.0, -1.0, 0.5, -0.2, 0.1, 0.05],
]
_RAGGED_Y_PIECES = [
    [0.0, 1.0, 0.0, 0.0],
    [1.0, 2.0, 0.0, 0.0, 0.0, 0.0],
    [2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]


def test_eval_pieces_handles_ragged_row_widths():
    t = np.array([0.5, 1.5, 2.5])

    pos, vel, acc, jerk = viz_pipeline._eval_pieces(_RAGGED_X_PIECES, t)

    tau = 0.5  # each piece is 1s wide, so 0.5 lands mid-piece for all three
    for i, row in enumerate(_RAGGED_X_PIECES):
        expected = _direct_eval(row[2:], tau)
        assert np.allclose((pos[i], vel[i], acc[i], jerk[i]), expected)


def test_build_time_series_evaluates_ragged_pieces_per_own_degree():
    snap = {
        "traj_x_pieces": _RAGGED_X_PIECES,
        "traj_y_pieces": _RAGGED_Y_PIECES,
        "traj_t_end": 3.0,
    }

    series = viz_pipeline._build_time_series(snap)
    t = series["t"]

    # Piece ownership at a shared boundary follows `_eval_pieces` (searchsorted
    # side="right"): a piece owns [t0, t1) except the very last, which also
    # owns its closing endpoint.
    for lo, hi, row, inclusive_hi in (
        (0.0, 1.0, _RAGGED_X_PIECES[0], False),
        (1.0, 2.0, _RAGGED_X_PIECES[1], False),
        (2.0, 3.0, _RAGGED_X_PIECES[2], True),
    ):
        mask = (t >= lo) & (t <= hi if inclusive_hi else t < hi)
        assert mask.sum() > 2, "sampling grid must be dense within each piece"
        for ti in t[mask]:
            _, expected_vel, expected_acc, expected_jerk = _direct_eval(
                row[2:], ti - lo
            )
            j = np.flatnonzero(t == ti)[0]
            assert np.isclose(series["vel"]["X"][j], expected_vel)
            assert np.isclose(series["acc"]["X"][j], expected_acc)
            assert np.isclose(series["jerk"]["X"][j], expected_jerk)
