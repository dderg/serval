"""Unit tests for the snapshot characterisation deltas.

Engine-free: every test drives synthetic piece rows and snapshots, so the
measuring instrument is verified independently of the planner it measures.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import characterise  # noqa: E402


def _track(axis: str, pieces: list[list[float]]) -> characterise.AxisTrack:
    return characterise.AxisTrack(axis, pieces)


def test_piece_state_at_matches_the_cubic_and_its_derivatives():
    piece = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    tau = 0.4
    pos, vel, acc = characterise.piece_state_at(piece, tau)
    assert pos == pytest.approx(2 + 3 * tau + 4 * tau**2 + 5 * tau**3)
    assert vel == pytest.approx(3 + 8 * tau + 15 * tau**2)
    assert acc == pytest.approx(8 + 30 * tau)


def test_piece_state_at_handles_a_constant_piece():
    assert characterise.piece_state_at([0.0, 1.0, 7.0], 0.5) == (7.0, 0.0, 0.0)


def test_axis_track_rejects_a_time_gap():
    with pytest.raises(ValueError, match="piece gap"):
        _track("x", [[0.0, 1.0, 0.0], [1.5, 2.0, 0.0]])


def test_axis_track_rejects_a_row_without_coefficients():
    with pytest.raises(ValueError, match="piece row of length"):
        _track("x", [[0.0, 1.0]])


def test_axis_track_state_at_selects_the_containing_piece():
    track = _track("x", [[0.0, 1.0, 0.0, 1.0], [1.0, 2.0, 1.0, -1.0]])
    assert track.state_at(0.5)[0] == pytest.approx(0.5)
    assert track.state_at(1.5)[0] == pytest.approx(0.5)
    assert track.t_end == 2.0


def test_axis_delta_is_zero_for_identical_tracks():
    pieces = [[0.0, 1.0, 0.0, 2.0, 3.0, 4.0], [1.0, 2.0, 9.0, 1.0]]
    delta = characterise.axis_delta(
        "x", _track("x", pieces), _track("x", pieces)
    )
    assert (delta.max_dp, delta.max_dv, delta.max_da) == (0.0, 0.0, 0.0)
    assert delta.pieces_change == 0


def test_axis_delta_catches_a_deviation_interior_to_a_piece():
    flat = _track("x", [[0.0, 1.0, 0.0]])
    bulge = _track("x", [[0.0, 1.0, 0.0, 1.0, -1.0]])
    delta = characterise.axis_delta("x", flat, bulge)
    assert delta.max_dp == pytest.approx(0.25)
    assert delta.max_dv == pytest.approx(1.0)
    assert delta.max_da == pytest.approx(2.0)


def test_axis_delta_reports_a_respline_as_pieces_only():
    whole = _track("x", [[0.0, 2.0, 0.0, 0.0, 1.0]])
    split = _track("x", [[0.0, 1.0, 0.0, 0.0, 1.0], [1.0, 2.0, 1.0, 2.0, 1.0]])
    delta = characterise.axis_delta("x", whole, split)
    assert (delta.max_dp, delta.max_dv, delta.max_da) == (0.0, 0.0, 0.0)
    assert (delta.pieces_before, delta.pieces_after) == (1, 2)
    assert delta.pieces_change == 1


def test_axis_delta_compares_only_the_overlapping_span():
    longer = _track("x", [[0.0, 1.0, 0.0, 1.0], [1.0, 2.0, 1.0, 1.0]])
    shorter = _track("x", [[0.0, 1.0, 0.0, 1.0]])
    delta = characterise.axis_delta("x", longer, shorter)
    assert (delta.max_dp, delta.max_dv, delta.max_da) == (0.0, 0.0, 0.0)
    assert (delta.pieces_before, delta.pieces_after) == (2, 1)


def test_axis_delta_on_an_empty_axis_reports_no_deviation():
    delta = characterise.axis_delta(
        "z", _track("z", []), _track("z", [[0.0, 1.0, 0.0, 5.0]])
    )
    assert (delta.max_dp, delta.max_dv, delta.max_da) == (0.0, 0.0, 0.0)
    assert (delta.pieces_before, delta.pieces_after) == (0, 1)


def test_verdict_identical_within_tolerance_and_shape():
    base = {"traversal_time_s": 1.0}
    assert (
        characterise._verdict(base, dict(base))
        is characterise.Verdict.IDENTICAL
    )
    assert (
        characterise._verdict(base, {"traversal_time_s": 1.0 + 1e-9})
        is characterise.Verdict.WITHIN_TOL
    )
    assert (
        characterise._verdict(base, {"traversal_time_s": 1.0 + 1e-3})
        is characterise.Verdict.SHAPE
    )


def _delta(
    name: str, verdict: characterise.Verdict, before: float, after: float
):
    return characterise.CaseDelta(
        name=name,
        verdict=verdict,
        time_before=before,
        time_after=after,
        axes=[characterise.AxisDelta("x", 4, 4, 0.0, 0.0, 0.0)],
    )


def test_time_change_is_absolute_and_relative():
    delta = _delta("g/p/c", characterise.Verdict.SHAPE, 2.0, 2.5)
    assert delta.time_change == pytest.approx(0.5)
    assert delta.time_change_rel == pytest.approx(0.25)


def test_rank_puts_shape_changes_above_larger_in_tolerance_drift():
    shape = _delta("shape", characterise.Verdict.SHAPE, 1.0, 1.0)
    drifted = _delta("drift", characterise.Verdict.WITHIN_TOL, 1.0, 2.0)
    assert max([shape, drifted], key=lambda d: d.rank) is shape


def test_rank_orders_shape_changes_by_relative_time_change():
    small = _delta("small", characterise.Verdict.SHAPE, 1.0, 1.01)
    large = _delta("large", characterise.Verdict.SHAPE, 1.0, 1.5)
    assert max([small, large], key=lambda d: d.rank) is large


def test_totals_line_sums_time_and_pieces_and_counts_verdicts():
    line = characterise.format_totals(
        [
            _delta("a", characterise.Verdict.IDENTICAL, 1.0, 1.0),
            _delta("b", characterise.Verdict.SHAPE, 1.0, 2.0),
        ]
    )
    assert "TOTALS 2 cases" in line
    assert "1 identical" in line
    assert "1 shape-changed" in line
    assert "2.000000s -> 3.000000s (+1.000000s, +50.0000%)" in line
    assert "pieces 8 -> 8 (+0)" in line


def test_row_reports_the_piece_count_change_when_it_moves():
    delta = characterise.CaseDelta(
        name="g/p/c",
        verdict=characterise.Verdict.SHAPE,
        time_before=1.0,
        time_after=1.0,
        axes=[characterise.AxisDelta("x", 4, 7, 0.0, 0.0, 0.0)],
    )
    assert "4->7" in characterise.format_row(delta)
    assert "4->7" not in characterise.format_row(
        _delta("g/p/c", characterise.Verdict.IDENTICAL, 1.0, 1.0)
    )


def test_json_payload_carries_every_reported_quantity():
    payload = characterise.as_json(
        _delta("g/p/c", characterise.Verdict.WITHIN_TOL, 1.0, 1.25)
    )
    assert payload["verdict"] == "within-tol"
    assert payload["traversal_time_s"]["change_rel"] == pytest.approx(0.25)
    assert payload["pieces"] == {"before": 4, "after": 4}
    assert [a["axis"] for a in payload["axes"]] == ["x"]
