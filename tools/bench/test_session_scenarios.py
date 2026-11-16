from __future__ import annotations

import re

import pytest
import run
import session_scenarios as scenarios
from session_scenarios import (
    SCENARIO_BY_NAME,
    SCENARIOS,
    Act,
    Bound,
    Config,
    Gcode,
)

ISSUE = re.compile(r"^#\d+$")


def test_every_scenario_starts_a_print_first_and_has_a_unique_name():
    assert len(SCENARIO_BY_NAME) == len(SCENARIOS)
    for scenario in SCENARIOS:
        assert scenario.timeline[0].act is Act.START_PRINT, scenario.name


def test_every_bound_names_the_issue_it_guards():
    bounds = [
        value for value in vars(scenarios).values() if isinstance(value, Bound)
    ]
    assert len(bounds) >= 6
    for bound in bounds:
        assert ISSUE.match(bound.issue), bound
    assert ISSUE.match(scenarios.REACTOR_STALL_ISSUE)


def test_a_bound_reports_the_value_against_its_limit_and_issue():
    bound = Bound(6.0, "#844")
    assert bound.judge("stopped after", 6.74) == (
        "stopped after 6.740 s > 6 s (#844)"
    )
    assert bound.holds(6.0)


def test_the_mesh_config_is_printer_cfg_then_macros_then_the_saved_mesh():
    text = scenarios.MESH_CONFIG.text()
    assert text.startswith(scenarios.PRINTER_CFG.read_text().rstrip())
    assert text.index("[gcode_macro CANCEL_PRINT]") < text.index("[bed_mesh]")
    assert text.rstrip().endswith("#*# tension = 0.2")
    assert "#*# [bed_mesh wavy]" in text


def test_a_saved_block_anywhere_but_last_is_refused():
    misplaced = Config(
        fragments=(scenarios.BED_MESH_WAVY, scenarios.PRINT_MACROS)
    )
    with pytest.raises(run.BenchError, match="must come last"):
        misplaced.text()


def test_the_mesh_loads_right_after_the_preamble():
    lines = scenarios.MESH_PRINT.lines()
    preamble = len(run.PREAMBLE)
    assert lines[:preamble] == run.PREAMBLE
    assert lines[preamble] == scenarios.LOAD_WAVY_MESH
    assert lines[-len(run.POSTAMBLE) :] == run.POSTAMBLE


def test_a_cut_print_keeps_its_first_lines_and_its_own_ending():
    end_print = SCENARIO_BY_NAME["end_print_mesh"].gcode.lines()
    short = scenarios.SHORT_PRINT.lines()
    assert len(short) == scenarios.SHORT_PRINT_LINES + len(run.POSTAMBLE)
    assert end_print[-1] == "END_PRINT"
    assert end_print[: len(run.PREAMBLE)] == run.PREAMBLE
    assert (
        end_print[len(run.PREAMBLE) + 1 : -1]
        == short[len(run.PREAMBLE) : -len(run.POSTAMBLE)]
    )


def test_the_estop_dwell_print_ends_with_the_zigzag_then_one_dwell():
    lines = SCENARIO_BY_NAME["estop_g4"].gcode.lines()
    zigzag = scenarios.zigzag_moves()
    tail = len(zigzag) + 1 + len(run.POSTAMBLE)
    assert len(lines) == scenarios.DENSE_PREFIX_LINES + tail
    assert lines[-tail:] == [*zigzag, scenarios.DWELL_LINE, *run.POSTAMBLE]
    assert lines.count(scenarios.DWELL_LINE) == 1


def test_a_gcode_describes_its_source_and_cut():
    assert Gcode(limit=200).describe() == (
        "tools/sim/gcode/voron_first_layers.gcode[:200]"
    )


def test_end_print_clears_the_mesh_with_motion_still_queued():
    scenario = SCENARIO_BY_NAME["end_print_mesh"]
    text = scenario.config.text()
    end_print = text.split("[gcode_macro END_PRINT]")[1].split("\n[")[0]
    body = [line.strip() for line in end_print.splitlines()[2:] if line]
    assert "M400" not in body
    assert "BED_MESH_CLEAR" in body
    assert scenario.gcode.lines()[-1] == "END_PRINT"
