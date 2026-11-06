from __future__ import annotations

from dataclasses import dataclass

import analyze
import corner_budget_bench
import numpy as np
import pytest


@dataclass
class _Motor:
    step_mm: float


@dataclass
class _Events:
    time: np.ndarray
    level: np.ndarray


def test_corexy_quantization_is_one_step_on_each_motor_mapped_back():
    motors = {"a": _Motor(0.0125), "b": _Motor(0.0125)}
    bound = corner_budget_bench.quantization_mm(
        motors, analyze.ROLE_AXIS_WEIGHTS
    )
    assert bound == pytest.approx(0.0125)


def test_a_miss_is_the_closest_chord_not_the_closest_sample():
    xy = np.array([[-1.0, 0.1], [1.0, 0.1]])
    assert corner_budget_bench._miss(xy, np.array([0.0, 0.0])) == pytest.approx(
        0.1
    )


def test_a_motor_between_steps_moves_on_the_line_joining_them():
    events = _Events(np.array([1.0, 2.0, 3.0]), np.array([10, 11, 12]))
    mm = corner_budget_bench._motor_mm(events, 0.5, np.array([0.5, 1.5, 3.5]))
    assert mm == pytest.approx([5.0, 5.25, 6.0])


def test_vertices_are_found_in_the_order_the_nozzle_reaches_them():
    t = np.linspace(0.0, 1.0, 101)
    out_and_back = np.where(t < 0.5, t * 20.0, (1.0 - t) * 20.0)
    xy = np.column_stack([out_and_back, np.zeros_like(t)])
    cursor = corner_budget_bench._Cursor([(t, xy)])
    first = cursor.find(np.array([5.0, 0.0]))
    second = cursor.find(np.array([5.0, 0.0]))
    assert first[1] < 50 < second[1]


def test_a_vertex_the_plan_never_nears_fails_loudly():
    t = np.linspace(0.0, 1.0, 11)
    xy = np.column_stack([t, np.zeros_like(t)])
    with pytest.raises(RuntimeError, match="never nears"):
        corner_budget_bench._Cursor([(t, xy)]).find(np.array([50.0, 50.0]))
