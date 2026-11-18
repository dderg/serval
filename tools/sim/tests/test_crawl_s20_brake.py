"""#1003, Pi bench 2026-11-17 (5ea04ef2c, printer.cfg): crawl_s20, a 2.3 s
lead-in at 100 mm/s then 120 s of extruding laps at 20 mm/s, completes but
the pacer orders a `RunwayExhausted` brake 0.36 s after the print starts,
with the stream not yet set off, and the toolhead stops mid slow section
where the drained moves end. At SERVAL_HOST_COMPUTE_SCALE 2 the simulator
does the same; at scale 1 it does not."""

import json
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1] / "gcode" / "crawl_s20.gcode"
)
PI_COMPUTE_SCALE = "2.0"
PRINT_TIMEOUT_S = 600


class PacerBrake(AssertionError):
    pass


@pytest.mark.expected_seconds(40)
@pytest.mark.xfail(strict=True, raises=PacerBrake, reason="#1003")
def test_crawl_s20_prints_without_a_pacer_brake_on_a_pi(sim_world, monkeypatch):
    monkeypatch.setenv("SERVAL_HOST_COMPUTE_SCALE", PI_COMPUTE_SCALE)
    world = sim_world(
        lambda w: configs.bench_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    world.print_file(GCODE, timeout=PRINT_TIMEOUT_S)

    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    events = []
    for line in world.events_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    names = {event.get("event") for event in events}
    assert "host_compute_charged" in names, "the pipeline ran uncharged"
    brakes = [
        {
            key: event.get(key)
            for key in ("cause", "runway_s", "drained_s", "dispatch_wait_s")
        }
        for event in events
        if event.get("event") == "pipe_drain"
    ]
    if brakes:
        raise PacerBrake(f"the pacer braked a healthy host's print: {brakes}")
