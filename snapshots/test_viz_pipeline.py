from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import viz_pipeline  # noqa: E402


def test_time_series_uses_snapshot_tangential_acceleration():
    snapshot = {
        "kin_s": [0.0, 1.0, 2.0],
        "kin_v": [10.0, 10.0, 10.0],
        "kin_a_t": [0.0, 20.0, 40.0],
        "kin_heading_x": [1.0, 2**-0.5, 0.0],
        "kin_heading_y": [0.0, 2**-0.5, 1.0],
        "kin_kappa": [0.1, 0.1, 0.1],
    }

    _, _, _, _, ax, ay, _, _, _, _ = viz_pipeline._build_time_series(snapshot)

    root_half = 2**-0.5
    assert ax[0] == pytest.approx(0.0)
    assert ay[0] == pytest.approx(10.0)
    assert ax[1] == pytest.approx(20.0 * root_half - 10.0 * root_half)
    assert ay[1] == pytest.approx(20.0 * root_half + 10.0 * root_half)
    assert ax[2] == pytest.approx(-10.0)
    assert ay[2] == pytest.approx(40.0)


def test_time_series_renders_old_snapshots_without_tangential_acceleration():
    snapshot = {
        "kin_s": [0.0, 1.0, 2.0],
        "kin_v": [10.0, 20.0, 30.0],
        "kin_heading_x": [1.0, 1.0, 1.0],
        "kin_heading_y": [0.0, 0.0, 0.0],
        "kin_kappa": [0.0, 0.0, 0.0],
    }

    _, _, _, _, ax, ay, _, _, _, _ = viz_pipeline._build_time_series(snapshot)

    assert ax.tolist() == pytest.approx([150.0, 212.5, 250.0])
    assert ay.tolist() == pytest.approx([0.0, 0.0, 0.0])
