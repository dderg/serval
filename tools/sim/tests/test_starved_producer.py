"""A producer that cannot keep up with playback must brake and stutter, never
abort. The engine's producer throttle (SERVAL_PRODUCER_THROTTLE) makes the
planning side spend longer on the print than it takes to play once it is
under way, so the producer falls behind its playhead over and over: every time, the pacer brakes the machine to rest and resumes it the
whole reserve ahead of the playhead. On the bench a Pi core behind a
20k-accel trident print aborted the print on the third such brake with an
anchor underrun."""

import json
import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_dense_top_layers.gcode"
)
PRINT_LINES = 2500
PRODUCER_REALTIME_FACTOR = 0.6
THROTTLE_FROM_STREAM_S = 3.0
MIN_BRAKES = 3
DEFAULT_LEAD_SECS = 0.25
FATAL_EVENTS = (
    "anchor_underrun",
    "anchor_low_margin",
    "anchor_hole_mid_motion",
    "stream_worker_fatal",
    "pump_piece_in_past",
    "runtime_fault",
    "diag.rust_fault",
)


def _events(world) -> list[dict]:
    events = []
    for line in world.events_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue
    return events


def test_a_producer_behind_its_playhead_brakes_and_resumes_instead_of_aborting(
    sim_world, monkeypatch, tmp_path
):
    monkeypatch.setenv("RUST_LOG", "info,motion_core::worker::ingress=debug")
    monkeypatch.setenv(
        "SERVAL_PRODUCER_THROTTLE",
        f"{PRODUCER_REALTIME_FACTOR}@{THROTTLE_FROM_STREAM_S}",
    )
    gcode = tmp_path / "starved_dense_top_layers.gcode"
    lines = GCODE.read_text().splitlines()[:PRINT_LINES]
    gcode.write_text("\n".join(lines) + "\n")
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    print_time = world.print_file(gcode, timeout=6000)
    assert print_time > 0

    events = _events(world)
    names = [event.get("event") for event in events]
    for fatal in FATAL_EVENTS:
        assert fatal not in names, (
            f"{fatal} while the producer was starved:\n{world.log_tail()[-3000:]}"
        )
    assert world.shutdown_line() is None, world.log_tail()[-3000:]

    brakes = names.count("pipe_drain")
    assert brakes >= MIN_BRAKES, (
        f"a producer at {PRODUCER_REALTIME_FACTOR}x realtime braked only "
        f"{brakes} times"
    )
    reserve_led_resumes = [
        event
        for event in events
        if event.get("event") == "anchor_decision"
        and event.get("lead_secs", 0.0) > DEFAULT_LEAD_SECS
    ]
    assert reserve_led_resumes, (
        f"{brakes} brakes, yet no resume started more than the default "
        f"{DEFAULT_LEAD_SECS}s ahead of the playhead"
    )
