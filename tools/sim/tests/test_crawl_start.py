"""A print whose short lead-in runs into a long slow section must start
without braking the pacer (#983, #1003): a stream at rest brakes only once
its producer stopped handing moves on, never on an inbox found empty
between two of them."""

import json
import pathlib

import pytest

from tools.sim import configs
from tools.sim.fuzzing.checks import lattice_counts

pytestmark = pytest.mark.needs_elf

GCODE = pathlib.Path(__file__).resolve().parents[1] / "gcode" / "crawl_s5.gcode"
HOST_COMPUTE_SCALES = ("1.0", "2.0", "3.0")
BENCH_RUN_PY_PRINT_END = "M400\n"
PRINT_TIMEOUT_S = 600
QUIET_PRODUCER_BRAKE = "RunwayExhausted"
FATAL_EVENTS = ("anchor_underrun", "anchor_low_margin", "stream_worker_fatal")
FULL_STEPS_PER_REV = 200
MICROSTEPS = 16
XY_ROTATION_MM = 40.0
Z_ROTATION_MM = 8.0
XY_STEPS_PER_MM = FULL_STEPS_PER_REV * MICROSTEPS / XY_ROTATION_MM
Z_STEPS_PER_MM = FULL_STEPS_PER_REV * MICROSTEPS / Z_ROTATION_MM
MOTOR_A_STEP_LINE = 0
MOTOR_B_STEP_LINE = 3
MOTOR_Z_STEP_LINE = 6
PRINT_END_XYZ = (40.0, 130.0, 5.0)


class BrakeAtStart(AssertionError):
    pass


@pytest.mark.expected_seconds(30)
@pytest.mark.parametrize("compute_scale", HOST_COMPUTE_SCALES)
def test_a_slow_section_after_a_short_lead_in_starts_without_a_pacer_brake(
    sim_world, monkeypatch, tmp_path, compute_scale
):
    monkeypatch.setenv("SERVAL_HOST_COMPUTE_SCALE", compute_scale)
    monkeypatch.setenv("RUST_LOG", "info,motion_core::worker::ingress=debug")
    gcode = tmp_path / GCODE.name
    gcode.write_text(GCODE.read_text() + BENCH_RUN_PY_PRINT_END)
    world = sim_world(
        lambda w: configs.bench_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    world.print_file(gcode, timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    events = _events(world)
    brakes = [
        {key: event.get(key) for key in ("cause", "runway_s", "drained_s")}
        for event in events
        if event.get("event") == "pipe_drain"
    ]
    if brakes:
        raise BrakeAtStart(
            f"the pacer braked a print it planned whole: {brakes}"
        )


def _events(world) -> list[dict]:
    events = []
    for line in world.events_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    return events


def _brakes(world) -> list[dict]:
    return [
        event for event in _events(world) if event.get("event") == "pipe_drain"
    ]


@pytest.mark.expected_seconds(30)
def test_a_print_ending_without_m400_while_its_start_is_held_starts_cleanly(
    sim_world, tmp_path
):
    """A print whose gcode ends without M400 while the planning front still
    holds its start: klippy goes quiet, and the quiet-producer brake sets
    the whole print off from rest. It must play every move to the print's
    end, once, with no step lost."""
    gcode = tmp_path / GCODE.name
    gcode.write_text(GCODE.read_text())
    world = sim_world(
        lambda w: configs.bench_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    start = {
        line: control.step_position(line)["steps"]
        for line in (MOTOR_A_STEP_LINE, MOTOR_B_STEP_LINE, MOTOR_Z_STEP_LINE)
    }
    world.print_file(gcode, timeout=PRINT_TIMEOUT_S)
    world.wait_until(
        lambda: _brakes(world), PRINT_TIMEOUT_S, "the quiet-producer brake"
    )
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    events = _events(world)
    assert not [e for e in events if e.get("event") in FATAL_EVENTS], events
    brakes = [
        {key: e.get(key) for key in ("cause", "runway_s", "drained_s")}
        for e in events
        if e.get("event") == "pipe_drain"
    ]
    assert len(brakes) == 1 and brakes[0]["cause"] == QUIET_PRODUCER_BRAKE, (
        brakes
    )
    assert brakes[0]["runway_s"] == 0.0, f"the stream had set off: {brakes}"
    starts = [e for e in events if e.get("event") == "anchor_decision"]
    assert len(starts) == 1, f"the print set off more than once: {starts}"
    x, y, z = PRINT_END_XYZ
    expected = {
        MOTOR_A_STEP_LINE: (x + y, XY_STEPS_PER_MM),
        MOTOR_B_STEP_LINE: (x - y, XY_STEPS_PER_MM),
        MOTOR_Z_STEP_LINE: (z, Z_STEPS_PER_MM),
    }
    for line, (position_mm, steps_per_mm) in expected.items():
        moved = control.step_position(line)["steps"] - start[line]
        allowed = lattice_counts(position_mm, steps_per_mm)
        assert moved in allowed, (
            f"step line {line} moved {moved} steps, the print ends on {allowed}"
        )
