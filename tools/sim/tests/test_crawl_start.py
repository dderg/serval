"""A print whose short lead-in runs into a long slow section must start
without braking the pacer (#983, #1003): a stream at rest brakes only once
its producer stopped handing moves on, never on an inbox found empty
between two of them."""

import json
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = pathlib.Path(__file__).resolve().parents[1] / "gcode" / "crawl_s5.gcode"
HOST_COMPUTE_SCALES = ("1.0", "2.0", "3.0")


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
    gcode.write_text(GCODE.read_text())
    world = sim_world(
        lambda w: configs.bench_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    world.print_file(gcode, timeout=600)
    world.gcode_ok("M400", timeout=600)

    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    events = []
    for line in world.events_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    brakes = [
        {key: event.get(key) for key in ("cause", "runway_s", "drained_s")}
        for event in events
        if event.get("event") == "pipe_drain"
    ]
    if brakes:
        raise BrakeAtStart(
            f"the pacer braked a print it planned whole: {brakes}"
        )
