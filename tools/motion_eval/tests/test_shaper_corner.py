from __future__ import annotations

import pathlib

import pytest

from tools.motion_eval import evaluate, shaper_corner

DATA = pathlib.Path(__file__).parent / "data"
CONFIG = DATA / "printer.cfg"
START = (100.0, 100.0, 10.0)
BUDGET_OVER_FLOOR = (1.5, 3.0)


def corner_run(budget):
    return evaluate.evaluate(
        CONFIG,
        (DATA / "corner.gcode").read_text(),
        START,
        sample_dt=None,
        corner_budget_mm=budget,
    )


@pytest.fixture(scope="module")
def stop_at_corner():
    return corner_run(0.0)


def test_zero_budget_stops_at_the_corner(stop_at_corner):
    worst = stop_at_corner["executed"]["corners"]["worst"]
    assert worst["junction_speed_mm_s"] == 0.0


@pytest.mark.parametrize("over_floor", BUDGET_OVER_FLOOR)
def test_budget_holds_on_the_shaped_corner(stop_at_corner, over_floor):
    floor = stop_at_corner["executed"]["corners"]["max_mm"]
    budget = over_floor * floor
    result = corner_run(budget)
    corners = result["executed"]["corners"]
    model_resolution = (
        result["commanded"]["peak_speed_mm_s"] * shaper_corner.MODEL_DT
    )
    assert corners["max_mm"] <= budget + model_resolution
    assert corners["worst"]["junction_speed_mm_s"] > 0.0
    assert result["print_time_s"] < stop_at_corner["print_time_s"]
