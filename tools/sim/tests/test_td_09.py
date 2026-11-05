"""A producer that cannot keep up with playback must brake and stutter, never
abort. The engine's producer throttle (SERVAL_PRODUCER_THROTTLE) makes the
planning side spend longer on the print than it takes to play once it is
under way, so the producer falls behind its playhead over and over: every time, the pacer brakes the machine to rest and resumes it the
whole reserve ahead of the playhead. On the bench a Pi core behind a
20k-accel trident print aborted the print on the third such brake with an
anchor underrun. The host's dispatch record (SERVAL_DISPATCH_RECORD_DIR)
holds every brake and the fresh anchor each resume starts from: the bench
analyzer judges such a run's steps against it."""

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
PRINT_LINES = 6000
PRODUCER_REALTIME_FACTOR = 0.9
THROTTLE_FROM_STREAM_S = 3.0
MIN_BRAKES = 3
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


def test_td_09(sim_world, monkeypatch, tmp_path):
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
