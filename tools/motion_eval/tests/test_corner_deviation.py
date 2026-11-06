from __future__ import annotations

import functools
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
BELOW_FLOOR_CAP = 0.1
MODES = (("floor", None), ("cap", BELOW_FLOOR_CAP))


@functools.cache
def executed(kind, case, config, family, cap=None):
    run = sweep.Run(
        kind, case, config, family, sweep.BASE_SCV, sweep.BASE_MCR, cap
    )
    with tempfile.TemporaryDirectory() as tmp:
        return sweep.execute(run, tmp)


@pytest.mark.parametrize("mode", MODES, ids=lambda m: m[0])
@pytest.mark.parametrize("family", sweep.FAMILIES)
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_corners_round_no_more_than_a_stop_can(case, config, family, mode):
    result = executed(mode[0], case, config, family, mode[1])
    grid_resolution = 0.5 * result["max_accel"] * sweep.SWEEP_DT**2
    assert result["corners"]["max_mm"] <= (
        result["shaper_floor_mm"] + grid_resolution
    )


@pytest.mark.parametrize("mode", MODES, ids=lambda m: m[0])
@pytest.mark.parametrize("family", sweep.FAMILIES)
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_acceleration_is_never_lowered(case, config, family, mode):
    result = executed(mode[0], case, config, family, mode[1])
    default = executed("scv", case, config, family)
    assert result["move_accels"] == default["move_accels"]


@pytest.mark.parametrize("mode", MODES, ids=lambda m: m[0])
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_smoother_peak_accel_within_stated_margin(case, config, mode):
    result = executed(mode[0], case, config, "smooth_mzv", mode[1])
    margin = result["limiter_stats"]["accel_margin"]
    assert result["peak_accel"] <= result["max_accel"] * (1.0 + margin)
