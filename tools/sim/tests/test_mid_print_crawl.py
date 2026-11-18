"""#1007: a long slow section between ordinary printing,
`mid_crawl_s5_300s` (20 s of strokes at 100 mm/s, 300 s of extruding laps
at 2.5 mm/s, 20 s of strokes). The turning guard holds the approach and the
whole open crawl run until the run closes, so the run's planning lands in
one step that must fit in the committed runway: 1.4-2.2 s of it at
SERVAL_HOST_COMPUTE_SCALE 1-3, which brakes the stream at scale 1 and
underruns it at 2 and 3."""

import json
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "mid_crawl_s5_300s.gcode"
)
HOST_COMPUTE_SCALES = ("1.0", "2.0", "3.0")
PRINT_TIMEOUT_S = 900
FATAL_EVENTS = ("anchor_underrun", "anchor_low_margin", "stream_worker_fatal")


class PacerBrake(AssertionError):
    pass


@pytest.mark.expected_seconds(60)
@pytest.mark.xfail(strict=True, raises=AssertionError, reason="#1007")
@pytest.mark.parametrize("compute_scale", HOST_COMPUTE_SCALES)
def test_a_long_slow_section_mid_print_prints_without_a_pacer_brake(
    sim_world, monkeypatch, compute_scale
):
    monkeypatch.setenv("SERVAL_HOST_COMPUTE_SCALE", compute_scale)
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
    assert not names & set(FATAL_EVENTS), world.log_tail()[-3000:]
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
