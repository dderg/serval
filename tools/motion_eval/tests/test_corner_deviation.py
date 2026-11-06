from __future__ import annotations

import tempfile

import pytest

from tools.motion_eval import sweep

CASES = (
    "corners_turn90",
    "corners_turn170",
    "zigzag_infill",
    "circles",
    "voron_top_layers",
)
BELOW_FLOOR_BUDGET = 0.05
ABOVE_FLOOR_BUDGET = 0.2


def limited_run(case, config, family, budget):
    run = sweep.Run("budget", case, config, family, sweep.BASE_SCV, 0.5, budget)
    with tempfile.TemporaryDirectory() as tmp:
        return sweep.execute(run, tmp)


@pytest.mark.parametrize("budget", (BELOW_FLOOR_BUDGET, ABOVE_FLOOR_BUDGET))
@pytest.mark.parametrize("family", sweep.FAMILIES)
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_shaped_corners_stay_within_budget(case, config, family, budget):
    result = limited_run(case, config, family, budget)
    grid_resolution = 0.5 * result["max_accel"] * sweep.SWEEP_DT**2
    assert result["corners"]["max_mm"] <= budget + grid_resolution
    assert result["limiter_stats"]["over_budget"] == 0
    if family.startswith("smooth"):
        assert result["peak_accel"] <= result["max_accel"] * (1 + 1e-9)
