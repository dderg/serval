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
BOUND_RATIO = 1.03
MODES = ("floor", "bounded")


def max_accel(case, config, family):
    return executed("scv", case, config, family)["max_accel"]


@functools.cache
def executed(kind, case, config, family, max_corner_accel=None):
    run = sweep.Run(
        kind,
        case,
        config,
        family,
        sweep.BASE_SCV,
        sweep.BASE_MCR,
        max_corner_accel=max_corner_accel,
    )
    with tempfile.TemporaryDirectory() as tmp:
        return sweep.execute(run, tmp)


def limited(mode, case, config, family):
    bound = None
    if mode == "bounded":
        bound = BOUND_RATIO * max_accel(case, config, family)
    return executed("floor", case, config, family, bound)


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("family", sweep.FAMILIES)
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_corners_round_no_more_than_a_stop_can(case, config, family, mode):
    result = limited(mode, case, config, family)
    grid_resolution = 0.5 * result["max_accel"] * sweep.SWEEP_DT**2
    assert result["corners"]["max_mm"] <= (
        result["shaper_floor_mm"] + grid_resolution
    )


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("family", sweep.FAMILIES)
@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_acceleration_is_never_lowered(case, config, family, mode):
    result = limited(mode, case, config, family)
    default = executed("scv", case, config, family)
    assert result["move_accels"] == default["move_accels"]


@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_smoother_peak_accel_within_what_the_planner_admits(case, config):
    result = limited("floor", case, config, "smooth_mzv")
    stats = result["limiter_stats"]
    admitted = max(stats["corner_accel_x"], stats["corner_accel_y"])
    assert result["peak_accel"] <= admitted


@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", CASES)
def test_smoother_peak_accel_within_max_corner_accel(case, config):
    bound = BOUND_RATIO * max_accel(case, config, "smooth_mzv")
    result = limited("bounded", case, config, "smooth_mzv")
    assert result["peak_accel"] <= bound
