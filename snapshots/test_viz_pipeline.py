from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import viz_pipeline  # noqa: E402


def test_time_series_acceleration_differentiates_vector_velocity():
    root_half = 2**-0.5
    snapshot = {
        "kin_s": [0.0, 1.0, 2.0],
        "kin_v": [10.0, 10.0, 10.0],
        "kin_heading_x": [1.0, root_half, 0.0],
        "kin_heading_y": [0.0, root_half, 1.0],
        "kin_kappa": [0.0, 0.0, 0.0],
    }

    _, _, _, _, ax, ay, a_sc, _, _, _ = viz_pipeline._build_time_series(snapshot)

    expected_ax = [
        10.0 * (root_half - 1.0) / 0.1,
        -10.0 * root_half / 0.1,
    ]
    expected_ay = [
        10.0 * root_half / 0.1,
        10.0 * (1.0 - root_half) / 0.1,
    ]
    assert ax.tolist() == pytest.approx([expected_ax[0], expected_ax[1], expected_ax[1]])
    assert ay.tolist() == pytest.approx([expected_ay[0], expected_ay[1], expected_ay[1]])
    assert a_sc[0] > 0.0


def test_time_series_acceleration_is_zero_for_constant_vector_velocity():
    snapshot = {
        "kin_s": [0.0, 1.0, 2.0],
        "kin_v": [10.0, 10.0, 10.0],
        "kin_heading_x": [1.0, 1.0, 1.0],
        "kin_heading_y": [0.0, 0.0, 0.0],
        "kin_kappa": [0.0, 0.0, 0.0],
    }

    _, _, _, _, ax, ay, a_sc, _, _, _ = viz_pipeline._build_time_series(snapshot)

    assert ax.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert ay.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert a_sc.tolist() == pytest.approx([0.0, 0.0, 0.0])
