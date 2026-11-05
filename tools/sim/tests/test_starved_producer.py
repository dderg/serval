"""A producer that cannot keep up with playback must brake and stutter, never
abort. The engine's producer throttle (SERVAL_PRODUCER_THROTTLE) makes the
planning side spend longer on the print than it takes to play once it is
under way, so the producer falls behind its playhead over and over: every time, the pacer brakes the machine to rest and resumes it the
whole reserve ahead of the playhead. On the bench a Pi core behind a
20k-accel trident print aborted the print on the third such brake with an
anchor underrun. The host's dispatch record (SERVAL_DISPATCH_RECORD_DIR)
holds every brake and the fresh anchor each resume starts from: the bench
analyzer judges such a run's steps against it.

On that bench a resume that re-anchored 35-50 ms after the end of the stream
the mcu was still stepping held its whole first volley behind a
reset_step_clock until the old stream finished, then sent it with ~40 ms of
margin: one host stall from a stale frame. The throttled producer here
resumes 0.12 s or more after the end of the old stream, so the second test
gets the same short-rest re-anchor from brief dwells mid-print."""

import json
import pathlib
import time

import pytest

from tools.bench import dispatch_record
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
SHORT_REST_PRINT_LINES = 1500
SHORT_REST_EVERY_LINES = 60
SHORT_RESTS_MS = (20, 30, 40, 50, 60)
SEND_MARGIN_WARN_FLOOR_S = 0.050
MIN_SHORT_REST_RESUMES = 5
RECORD_SETTLE_S = 10.0
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


def _record_holding(
    record_dir: pathlib.Path, brakes: int
) -> dispatch_record.DispatchRecord:
    """The live record once its writer has caught up with every brake the
    events logged: the writer drains its channel on its own thread, so the
    file may trail the print by a few entries."""
    deadline = time.monotonic() + RECORD_SETTLE_S
    while True:
        records = [
            dispatch_record.load(path)
            for path in sorted(record_dir.glob("dispatch_record_*.bin"))
        ]
        moving = [r for r in records if len(r)]
        if len(moving) == 1 and len(moving[0].pacer_brakes()) >= brakes:
            return moving[0]
        assert time.monotonic() < deadline, (
            f"the dispatch record holds {[len(r.pacer_brakes()) for r in moving]} "
            f"pacer brakes after {RECORD_SETTLE_S}s; the events logged {brakes}"
        )
        time.sleep(0.05)


def _margin_lows(events: list[dict]) -> list[dict]:
    return [e for e in events if e.get("event") == "step_send_margin_low"]


def _with_short_rests(lines: list[str]) -> list[str]:
    rested = []
    for index, line in enumerate(lines, start=1):
        rested.append(line)
        if index % SHORT_REST_EVERY_LINES == 0:
            rest_ms = SHORT_RESTS_MS[
                (index // SHORT_REST_EVERY_LINES) % len(SHORT_RESTS_MS)
            ]
            rested.append(f"G4 P{rest_ms}")
    return rested


def test_a_resume_after_a_short_rest_keeps_its_step_send_margin(
    sim_world, tmp_path
):
    gcode = tmp_path / "short_rests_dense_top_layers.gcode"
    lines = GCODE.read_text().splitlines()[:SHORT_REST_PRINT_LINES]
    gcode.write_text("\n".join(_with_short_rests(lines)) + "\n")
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    assert world.print_file(gcode, timeout=6000) > 0
    world.gcode_ok("M400", timeout=60)

    events = _events(world)
    names = [event.get("event") for event in events]
    for fatal in FATAL_EVENTS:
        assert fatal not in names, world.log_tail()[-3000:]
    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    short_rests = [
        event["rest_s"]
        for event in events
        if event.get("event") == "clock_resync"
        and event.get("rest_s", 1.0) < SEND_MARGIN_WARN_FLOOR_S
    ]
    assert len(short_rests) >= MIN_SHORT_REST_RESUMES, (
        f"the print re-anchored after only {len(short_rests)} rests shorter than "
        f"{SEND_MARGIN_WARN_FLOOR_S}s: {short_rests}"
    )
    assert not _margin_lows(events), (
        f"steps went out with less than {SEND_MARGIN_WARN_FLOOR_S}s of margin after "
        f"short rests {short_rests}: {_margin_lows(events)}"
    )


def test_a_producer_behind_its_playhead_brakes_and_resumes_instead_of_aborting(
    sim_world, monkeypatch, tmp_path
):
    monkeypatch.setenv("RUST_LOG", "info,motion_core::worker::ingress=debug")
    record_dir = tmp_path / "dispatch_record"
    record_dir.mkdir()
    monkeypatch.setenv("SERVAL_DISPATCH_RECORD_DIR", str(record_dir))
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
    world.gcode_ok("M400", timeout=60)

    events = _events(world)
    names = [event.get("event") for event in events]
    for fatal in FATAL_EVENTS:
        assert fatal not in names, (
            f"{fatal} while the producer was starved:\n{world.log_tail()[-3000:]}"
        )
    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    assert "step_send_margin_low" not in names, _margin_lows(events)

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

    record = _record_holding(record_dir, brakes)
    assert len(record.pacer_brakes()) == brakes
    resumes = {d.after_segments for d in record.pacer_brakes()}
    fresh = (record.flags & dispatch_record.FLAG_FRESH_EPOCH) != 0
    assert all(fresh[at] for at in resumes if at < len(record)), (
        "a resume after a pacer brake kept the anchor it braked on"
    )
    seams = record.host_start()[1:] - record.host_end()[:-1]
    drained = {d.after_segments for d in record.drains}
    continuous = [
        abs(seam) < 1e-9
        for index, seam in enumerate(seams, start=1)
        if not fresh[index] and index not in drained
    ]
    assert all(continuous), "dispatched segments left a gap inside one anchor"
