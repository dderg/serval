from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import viz_pipeline  # noqa: E402


def _straight(n, speed, step):
    # Toolhead marching along +x at constant speed: position is all the
    # visualizer is given; velocity/accel/jerk are its own to compute.
    xs = [i * step for i in range(n)]
    return {
        "kin_x": xs,
        "kin_y": [0.0] * n,
        "kin_v": [speed] * n,
    }


def test_velocity_is_recovered_from_position_alone():
    snap = _straight(n=10, speed=20.0, step=1.0)

    _, vx, vy, v_scalar, *_ = viz_pipeline._build_time_series(snap)

    assert vx.tolist() == pytest.approx([20.0] * 10)
    assert vy.tolist() == pytest.approx([0.0] * 10)
    assert v_scalar.tolist() == pytest.approx([20.0] * 10)


def test_constant_velocity_has_zero_acceleration_and_jerk():
    snap = _straight(n=10, speed=20.0, step=1.0)

    *_, ax, ay, a_scalar, jx, jy, j_scalar = viz_pipeline._build_time_series(
        snap
    )

    assert ax.tolist() == pytest.approx([0.0] * 10, abs=1e-6)
    assert a_scalar.tolist() == pytest.approx([0.0] * 10, abs=1e-6)
    assert j_scalar.tolist() == pytest.approx([0.0] * 10, abs=1e-6)


def test_circular_arc_at_constant_speed_shows_centripetal_acceleration():
    # A quarter circle of radius r traversed at constant speed v must show a
    # constant acceleration magnitude v**2 / r, computed purely from position.
    r, v, n = 10.0, 50.0, 400
    xs = [r * math.sin(k / (n - 1) * (math.pi / 2)) for k in range(n)]
    ys = [r * (1.0 - math.cos(k / (n - 1) * (math.pi / 2))) for k in range(n)]
    snap = {"kin_x": xs, "kin_y": ys, "kin_v": [v] * n}

    _, _, _, v_scalar, _, _, a_scalar, *_ = viz_pipeline._build_time_series(
        snap
    )

    assert v_scalar[5:-5].tolist() == pytest.approx([v] * (n - 10), rel=1e-3)
    expected = v**2 / r
    assert a_scalar[5:-5].tolist() == pytest.approx(
        [expected] * (n - 10), rel=1e-2
    )


def test_legacy_heading_snapshot_reconstructs_position():
    # Old baselines lack kin_x/kin_y; position is integrated from heading so the
    # review UI can still preview them.
    snap = {
        "kin_s": [0.0, 1.0, 2.0, 3.0],
        "kin_v": [10.0, 10.0, 10.0, 10.0],
        "kin_heading_x": [1.0, 1.0, 1.0, 1.0],
        "kin_heading_y": [0.0, 0.0, 0.0, 0.0],
        "raw_x": [5.0, 8.0],
        "raw_y": [2.0, 2.0],
    }

    x, y = viz_pipeline._toolhead_position(snap)

    assert x.tolist() == pytest.approx([5.0, 6.0, 7.0, 8.0])
    assert np.all(y == 2.0)
