from __future__ import annotations

import dataclasses

import hold_plan
import pytest
import run
import soak_gcode


@pytest.mark.parametrize("soak", soak_gcode.SOAK_PRINTS)
def test_a_soak_print_only_climbs_one_layer_at_a_time(soak):
    heights = soak_gcode.z_values(soak_gcode.soak_lines(soak.body_repeats))
    steps = [later - earlier for earlier, later in zip(heights, heights[1:])]
    assert heights[0] == 0.2
    assert min(steps) >= 0.0
    assert max(steps) <= soak_gcode.LAYER_HEIGHT_MM + 1e-6


@pytest.mark.parametrize("soak", soak_gcode.SOAK_PRINTS)
def test_a_soak_print_starts_with_the_first_layers_whole(soak):
    first = soak_gcode.FIRST_LAYERS.read_text().splitlines()
    lines = soak_gcode.soak_lines(soak.body_repeats)
    assert lines[: len(first)] == first
    upper = soak_gcode.upper_layers(first)
    dense = soak_gcode.DENSE_TOP.read_text().splitlines()
    block = len(upper) + len(dense)
    assert len(lines) == len(first) + soak.body_repeats * block


def test_the_soak_print_is_one_the_bench_runs():
    lines = soak_gcode.soak_lines(soak_gcode.LONG_PRINT.body_repeats)
    _, stats = run.sanitize("\n".join(lines))
    assert stats["kept"] > 0


def test_the_session_print_twice_and_the_long_print_each_fit_a_hold():
    session_s = hold_plan.expected_s("soak_mesh_session")
    long_s = hold_plan.expected_s("soak_throttled")
    assert session_s > 2 * soak_gcode.SESSION_PRINT.planned_print_s
    assert long_s > soak_gcode.LONG_PRINT.planned_print_s
    assert max(session_s, long_s) <= hold_plan.HOLD_ROWS_S


def test_the_long_print_is_the_longest_that_fits_a_hold():
    session, long = soak_gcode.SESSION_PRINT, soak_gcode.LONG_PRINT
    per_repeat_s = (long.planned_print_s - session.planned_print_s) / (
        long.body_repeats - session.body_repeats
    )
    one_more_s = long.planned_print_s + per_repeat_s
    assert one_more_s + hold_plan.PRINT_OVERHEAD_S > hold_plan.HOLD_ROWS_S


def test_a_shift_moves_z_words_and_leaves_comments():
    assert (
        soak_gcode.shifted("G1 X1 Z.4 F9000 ; Z.4 kept", 1.0)
        == "G1 X1 Z1.400 F9000 ; Z.4 kept"
    )
    assert soak_gcode.shifted("G1 X12 E.5", 1.0) == "G1 X12 E.5"


def test_ensure_writes_the_print_once_and_repairs_it(tmp_path, monkeypatch):
    monkeypatch.setattr(soak_gcode, "GENERATED", tmp_path / "generated")
    soak = dataclasses.replace(soak_gcode.SESSION_PRINT, body_repeats=1)
    path = tmp_path / "generated" / "soak_session.gcode"
    assert soak_gcode.ensure(soak) == path
    written = path.stat().st_mtime_ns
    soak_gcode.ensure(soak)
    assert path.stat().st_mtime_ns == written
    path.write_text("G1 X0\n")
    soak_gcode.ensure(soak)
    assert path.read_text().splitlines() == soak_gcode.soak_lines(1)
