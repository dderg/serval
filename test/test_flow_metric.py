import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import flow_metric as fm  # noqa: E402

DT_S = 1e-4
SPEED_MM_S = 100.0
CORNER = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
RATIOS = (0.03, 0.06)
HOLD_S = 0.01


def _along(u: np.ndarray) -> np.ndarray:
    first = np.clip(u, 0.0, 10.0)
    second = np.clip(u - 10.0, 0.0, 10.0)
    return np.column_stack([first, second])


def _extruded(u: np.ndarray) -> np.ndarray:
    return RATIOS[0] * np.clip(u, 0.0, 10.0) + RATIOS[1] * np.clip(
        u - 10.0, 0.0, 10.0
    )


def _tracks(t: np.ndarray, u: np.ndarray):
    xy = _along(u)
    e = _extruded(u)
    return [
        np.column_stack([column, np.gradient(column, t)])
        for column in (xy[:, 0], xy[:, 1], e)
    ]


def _plans():
    """The corner traversed at a steady 100 mm/s, and again after a hold
    and with a lag that grows then shrinks: no one shift lays one on the
    other."""
    t_raw = np.arange(0.0, 0.2 + DT_S, DT_S)
    raw = _tracks(t_raw, SPEED_MM_S * t_raw)
    t = np.arange(0.0, 0.21 + DT_S, DT_S)
    late = np.clip(t - HOLD_S, 0.0, None)
    unshaped_time = 0.8 * late + late**2
    shaped = _tracks(t, SPEED_MM_S * unshaped_time)
    return t, shaped, t_raw, raw


def test_the_commanded_ratio_steps_where_the_shaped_path_turns_the_corner():
    t, shaped, t_raw, raw = _plans()
    xy = np.column_stack([shaped[0][:, 0], shaped[1][:, 0]])
    commanded = fm.commanded_bead(t, xy, t_raw, raw, CORNER)
    first_leg = (xy[:, 0] > 0.5) & (xy[:, 0] < 9.5)
    second_leg = (xy[:, 1] > 0.5) & (xy[:, 1] < 9.5)
    assert commanded.ratio[first_leg] == pytest.approx(RATIOS[0], rel=1e-6)
    assert commanded.ratio[second_leg] == pytest.approx(RATIOS[1], rel=1e-6)
    assert commanded.raw_moving[first_leg | second_leg].all()


def test_a_dwell_spans_the_whole_rest_and_a_corner_one_sweep():
    u = np.concatenate(
        [
            np.linspace(0.0, 10.0, 101),
            np.full(50, 10.0),
            np.linspace(10.0, 20.0, 101),
        ]
    )
    dwells = fm.vertex_dwells(_along(u), CORNER)
    assert dwells == [(0, 0), (100, 151), (251, 251)]


def test_close_vertices_each_claim_their_own_samples():
    vertices = [(0.5 * i, 0.25 * (i % 2)) for i in range(12)]
    dense = np.concatenate(
        [
            np.linspace(a, b, 50, endpoint=False)
            for a, b in zip(vertices, vertices[1:])
        ]
        + [np.array([vertices[-1]])]
    )
    dwells = fm.vertex_dwells(dense, vertices)
    assert [first for first, _ in dwells] == [50 * i for i in range(12)]


def test_a_vertex_the_path_misses_anchors_nothing():
    u = np.linspace(0.0, 10.0, 101)
    assert fm.vertex_dwells(
        _along(u), [(0.0, 0.0), (5.0, 3.0), (10.0, 0.0)]
    ) == [
        (0, 0),
        None,
        (100, 100),
    ]


def _ramp_trace(laid_at) -> fm.FlowTrace:
    s = np.arange(0.0, 20.0 + 1e-9, 1e-3)
    middle = np.arange(0.0, 20.0, fm.PATH_STEP_MM) + 0.5 * fm.PATH_STEP_MM
    laid = laid_at(middle - 10.0)
    return fm.FlowTrace(s=s, middle=middle, laid=laid, deviation=laid)


BEAD_AREA_MM2 = 0.08
APEX = 10_000


def test_a_centred_ramp_moves_an_eighth_of_its_width_and_loses_nothing():
    width = 2.0
    trace = _ramp_trace(lambda d: np.clip(0.5 - d / width, 0.0, 1.0))
    step = fm.step_report(trace, APEX, (1.0, 0.0), 5.0, BEAD_AREA_MM2)
    assert step["moved_nl"] == pytest.approx(width / 8.0 * 80.0, rel=1e-3)
    assert step["net_nl"] == pytest.approx(0.0, abs=1e-6)
    assert step["switch_mm"] == pytest.approx(
        0.9 * width, abs=2 * fm.PATH_STEP_MM
    )


def test_a_sharp_switch_moves_nothing():
    trace = _ramp_trace(lambda d: np.where(d < 0.0, 1.0, 1.5))
    step = fm.step_report(trace, APEX, (1.0, 1.5), 5.0, BEAD_AREA_MM2)
    assert step == {
        "s_mm": pytest.approx(10.0),
        "moved_nl": pytest.approx(0.0, abs=1e-9),
        "net_nl": pytest.approx(0.0, abs=1e-9),
        "switch_mm": 0.0,
    }


def test_a_switch_that_stops_early_loses_what_it_did_not_lay():
    trace = _ramp_trace(lambda d: np.where(d < -0.5, 1.0, 0.0))
    step = fm.step_report(trace, APEX, (1.0, 0.0), 5.0, BEAD_AREA_MM2)
    assert step["net_nl"] == pytest.approx(-0.5 * 80.0, rel=1e-6)


@pytest.mark.parametrize(
    ("before", "after", "steps"),
    [(1.0, 1.0, False), (1.0, 1.02, False), (1.0, 1.5, True), (1.0, 0.0, True)],
)
def test_a_ratio_step_is_a_change_of_at_least_five_percent(
    before, after, steps
):
    assert fm.is_ratio_step(before, after) is steps


def test_a_retract_and_prime_at_rest_reach_no_bead():
    melt_k = 0.03
    ratio = 0.03
    t = np.arange(0.0, 0.3 + DT_S, DT_S)
    stop = (t >= 0.1) & (t < 0.2)
    u = SPEED_MM_S * np.where(t < 0.1, t, np.where(stop, 0.1, t - 0.1))
    xy = np.column_stack([u, np.zeros_like(u)])
    commanded = fm.CommandedBead(ratio=np.full_like(t, ratio), raw_moving=~stop)
    retract = np.where((t >= 0.12) & (t < 0.15), -0.8, 0.0)
    plain = fm.flow_trace(t, xy, ratio * u, commanded, melt_k, ratio)
    retracted = fm.flow_trace(
        t, xy, ratio * u + retract, commanded, melt_k, ratio
    )
    assert np.abs(retracted.deviation - plain.deviation).max() < 1e-9
