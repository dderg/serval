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
BOUND_RATIO = 1.03
MODES = (("floor", None), ("cap", BELOW_FLOOR_CAP))


@functools.cache
def executed(kind, case, config, family, cap=None, max_corner_accel=None):
    run = sweep.Run(
        kind,
        case,
        config,
        family,
        sweep.BASE_SCV,
        sweep.BASE_MCR,
        cap,
        max_corner_accel=max_corner_accel,
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
def test_smoother_peak_accel_within_what_the_planner_admits(case, config, mode):
    result = executed(mode[0], case, config, "smooth_mzv", mode[1])
    stats = result["limiter_stats"]
    admitted = max(stats["corner_accel_x"], stats["corner_accel_y"])
    assert result["peak_accel"] <= admitted * (1.0 + stats["accel_margin"])


@pytest.mark.parametrize("mode", MODES, ids=lambda m: m[0])
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_smoother_peak_accel_within_max_corner_accel(case, config, mode):
    bound = (
        BOUND_RATIO * executed("scv", case, config, "smooth_mzv")["max_accel"]
    )
    result = executed(mode[0], case, config, "smooth_mzv", mode[1], bound)
    margin = result["limiter_stats"]["accel_margin"]
    assert result["peak_accel"] <= bound * (1.0 + margin)
