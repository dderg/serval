"""A print whose short lead-in runs into a long slow section must start
without braking the pacer (#983). On the Pi bench the same print aborts
klippy: the first anchor commits an 8.4 ms segment while the planning front
holds the rest of the print, and planning it takes longer than the lead.
The sim's clock stands still while the host plans, so here the same start
shows as a pacer brake with zero runway that drains the whole print."""

import json
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = pathlib.Path(__file__).resolve().parents[1] / "gcode" / "crawl_s5.gcode"


class BrakeAtStart(AssertionError):
    pass


@pytest.mark.expected_seconds(30)
@pytest.mark.xfail(strict=True, raises=BrakeAtStart, reason="#983")
def test_a_slow_section_after_a_short_lead_in_starts_without_a_pacer_brake(
    sim_world, monkeypatch, tmp_path
):
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
