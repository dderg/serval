"""A stepcompress drain pass may only spend `stepcompress_drain_budget` on
the step-root search past the stream the mcu needs next; the rest of a
refill carries over to later passes. The sim runs the same budget path as
hardware. A zero budget spends it on every pass, so each pass roots one
window past that and yields, including the drain a reanchor cut takes to
its cut clock. A crystal off nominal makes the clock re-sync at every rest
adopt a new slope, which cuts every lane there, and a print from file keeps
the run before each cut buffered past what the passes have rooted."""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

RUNGS = 8
MOVES_PER_RUNG = 6
FEEDRATES = (12000, 18000, 24000)


def _rest_ladder() -> str:
    lines = ["G90", "G1 X25 Y75 F6000"]
    for rung in range(RUNGS):
        feed = FEEDRATES[rung % len(FEEDRATES)]
        for move in range(MOVES_PER_RUNG):
            x, y = (225, 175) if move % 2 == 0 else (25, 75)
            lines.append(f"G1 X{x} Y{y} F{feed}")
        lines.append("G4 P200")
    lines.append("G1 X125 Y125 F6000")
    return "\n".join(lines) + "\n"


def _spent_budget_config(world) -> str:
    text = configs.stepcompress_config(
        world.h7_pty, world.f4_pty, str(world.gcode_dir)
    )
    sc_section = f"[mcu sc]\nserial: {world.f4_pty}\n"
    assert sc_section in text
    return (
        text.replace(
            sc_section, sc_section + "stepcompress_drain_budget: 0\n"
        )
        .replace("max_velocity: 100\n", "max_velocity: 500\n")
        .replace("max_accel: 1000\n", "max_accel: 20000\n")
    )


def _steps(world, axis: str) -> int:
    line = configs.STEPCOMPRESS_STEP_LINES[axis]
    return world.sim_control("f4").step_position(line)["steps"]


def test_rests_at_speed_cut_lanes_on_a_spent_drain_budget(sim_world, tmp_path):
    world = sim_world(
        _spent_budget_config, sc_mcu=True, crystal_ppm={"f4": 30.0}
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M400")
    before = {axis: _steps(world, axis) for axis in ("x", "y")}
    gcode = tmp_path / "rest_ladder.gcode"
    gcode.write_text(_rest_ladder())
    world.print_file(gcode)
    world.gcode_ok("M400", timeout=120)

    assert world.shutdown_line() is None, world.log_tail()
    events = world.events_text()
    assert "reanchor_cut_unsent" in events, (
        "no rest cut a lane: the test no longer reaches the reanchor drain"
    )
    for axis in ("x", "y"):
        assert _steps(world, axis) - before[axis] == pytest.approx(
            0, abs=1
        ), f"{axis} did not return to its start"
