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


def test_corexy_quantization_is_two_steps_on_each_motor_mapped_back():
    motors = {"a": _Motor(0.0125), "b": _Motor(0.0125)}
    bound = corner_budget_bench.quantization_mm(
        motors, analyze.ROLE_AXIS_WEIGHTS
    )
    assert bound == pytest.approx(0.025)


def test_a_miss_is_the_closest_chord_not_the_closest_sample():
    xy = np.array([[-1.0, 0.1], [1.0, 0.1]])
    assert corner_budget_bench._miss(xy, np.array([0.0, 0.0])) == pytest.approx(
        0.1
    )


def test_a_motor_between_steps_moves_on_the_line_joining_them():
    events = _Events(np.array([1.0, 2.0, 3.0]), np.array([10, 11, 12]))
    mm = corner_budget_bench._motor_mm(events, 0.5, np.array([0.5, 1.5, 3.5]))
    assert mm == pytest.approx([5.0, 5.25, 6.0])


def test_a_window_spans_the_plan_within_its_path_length_of_the_apex():
    t = np.linspace(0.0, 1.0, 1001)
    xy = np.column_stack([t * 10.0, np.zeros_like(t)])
    lo, hi = corner_budget_bench._window(t, xy, 0.5)
    assert (lo, hi) == pytest.approx((0.4, 0.6), abs=2e-3)


def test_a_vertex_no_planned_epoch_holds_fails_loudly():
    with pytest.raises(RuntimeError, match="no planned epoch"):
        corner_budget_bench._block_of([{"t_start": 0.0, "t_end": 1.0}], 2.0)
