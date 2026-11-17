"""Motion buffered ahead of the playhead, within one klippy session.

Bench sweep of origin/main 0b90eeaf3 (2026-11-15, Pi 5 + F401):
- F2: after a CANCEL_PRINT the next print braked to rest mid-print for 3.3,
  6.4, 14.7, 26.2, 49.2 and 94.5 s. The pacer sized its reserve from how long
  a drain took, and a drain behind a full pipeline lasts as long as the runway
  the reserve then demands: every brake resumed a reserve ahead and the next
  drain measured the longer runway.
- F3: PAUSE and CANCEL_PRINT took effect only after the 14-28 s of motion
  buffered ahead of the playhead had played out, and print_stats reported a
  long-move file complete a minute before its motion ended.

Every motion time here is read off the X/Y step edges the MCUs emitted
(shared virtual timeline), never off klippy's own clock."""

import dataclasses
import json
import pathlib

import pytest

from tools.sim import configs

from .test_resonance_buzz import X_STEP_LINE, Y_STEP_LINE

pytestmark = pytest.mark.needs_elf

FIRST_LAYERS = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_first_layers.gcode"
)
DENSE_TOP = FIRST_LAYERS.with_name("voron_dense_top_layers.gcode")
SPEED_LADDER = (
    pathlib.Path(__file__).resolve().parents[2]
    / "bench"
    / "gcode"
    / "speed_ladder.gcode"
)
FIRST_LAYERS_LINES = 1500
LONG_MOVE_COUNT = 60
LONG_MOVE_MM = 80.0
LONG_MOVE_FEED = 6000
DWELL_ZIGZAG_MOVES = 120

ACTION_AT_S = 20.0
MOTION_STILL_S = 2.0
PRINT_TIMEOUT_S = 1200.0
NS_PER_S = 1e9

DURATION_TOLERANCE = 0.10
MAX_RESUME_LEAD_S = 1.0
MAX_RESERVE_S = 1.0
PAUSE_CANCEL_STOP_S_AFTER_860 = 6.0

PACER_LOG = "info,motion_core::worker::ingress=debug"
STOP_BRAKE_CAUSE = "RunwayExhausted"
PIPE_SPACE = "motion pipe space"

TRIDENT_2_CPU_REALTIME_FACTOR = 1.85
DENSE_TOP_FIRST_TRAVEL_END_S = 1.9
FRONT_RELEASE_AT_START_S = 11.0
MAX_PIPE_SPACE_WAIT_S = FRONT_RELEASE_AT_START_S + 1.0

XY_STEP_LINES = (X_STEP_LINE, Y_STEP_LINE)
BENCH_AB_STEP_LINES = (0, 3)

PAUSE_RESUME = "\n[pause_resume]\n"


class _EventCursor:
    """Structured events appended since the cursor was placed."""

    def __init__(self, world):
        self.events_dir = world.log_dir / "events"
        self.offsets = {
            path: path.stat().st_size
            for path in self.events_dir.glob("*.jsonl")
        }

    def new(self) -> list[dict]:
        events = []
        for path in sorted(self.events_dir.glob("*.jsonl")):
            with path.open("rb") as stream:
                stream.seek(self.offsets.get(path, 0))
                text = stream.read().decode(errors="replace")
            for line in text.splitlines():
                try:
                    events.append(json.loads(line))
                except ValueError:
                    continue
        return events


def _boot(sim_world, monkeypatch):
    monkeypatch.setenv("RUST_LOG", PACER_LOG)
    world = sim_world(
        lambda w: (
            configs.neptune_print_config(w.h7_pty, str(w.gcode_dir))
            + PAUSE_RESUME
        ),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    return world


def _first_layers(world) -> pathlib.Path:
    path = world.gcode_dir / "first_layers.gcode"
    lines = FIRST_LAYERS.read_text().splitlines()[:FIRST_LAYERS_LINES]
    path.write_text("\n".join(lines) + "\n")
    return path


def _zigzag_lines(moves: int) -> list[str]:
    lines = ["G90", "M83", "G1 Z0.3 F600", "G1 X60 Y20 F12000"]
    for index in range(moves):
        x = 60.0 + LONG_MOVE_MM if index % 2 == 0 else 60.0
        y = 20.0 + (index // 2) * 0.4
        lines.append(f"G1 X{x:.1f} Y{y:.2f} E0.3 F{LONG_MOVE_FEED}")
    return lines


def _long_moves(world) -> pathlib.Path:
    path = world.gcode_dir / "long_moves.gcode"
    path.write_text("\n".join(_zigzag_lines(LONG_MOVE_COUNT)) + "\n")
    return path


GCODES = {"first_layers": _first_layers, "long_moves": _long_moves}
LATE_STOP = pytest.mark.xfail(
    strict=True,
    reason="#860: the fit stage, the planner and the turning guard hold "
    "0.8 s strokes whole: the stop drains 10.5 s of them",
)
BENCH_STOPS = ["first_layers", pytest.param("long_moves", marks=LATE_STOP)]


def _now_s(world) -> float:
    return world.vclock.now_ns() / NS_PER_S


def _last_step_s(control, lines) -> float:
    return (
        max(control.get_step_times(line)["last_ns"] for line in lines)
        / NS_PER_S
    )


def _wait_motion_still(world, control, lines=XY_STEP_LINES) -> float:
    """The time of the last step on `lines` once they have been still for
    MOTION_STILL_S."""
    world.wait_until(
        lambda: _now_s(world) - _last_step_s(control, lines) >= MOTION_STILL_S,
        timeout=PRINT_TIMEOUT_S,
        what=f"X/Y still for {MOTION_STILL_S}s",
        poll=0.05,
    )
    return _last_step_s(control, lines)


def _start(world, path: pathlib.Path) -> float:
    started_s = _now_s(world)
    world.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={path.name}")
    return started_s


def _wait_print_time(world, started_s: float, elapsed_s: float) -> None:
    world.wait_until(
        lambda: _now_s(world) - started_s >= elapsed_s,
        timeout=PRINT_TIMEOUT_S,
        what=f"{elapsed_s}s of print time",
    )


def _motion_s(world, control, path: pathlib.Path) -> float:
    """Wall-to-wall motion time of one whole print, M400 included."""
    started_s = _start(world, path)
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)
    return _wait_motion_still(world, control) - started_s


@dataclasses.dataclass
class _Stop:
    issued_s: float
    fed_until_s: float
    ended_s: float


def _stop(world, control, command: str, lines=XY_STEP_LINES) -> _Stop:
    """Runs `command`, which returns once klippy has stopped feeding moves,
    and waits for the motion to end."""
    issued_s = _now_s(world)
    world.gcode_ok(command, timeout=PRINT_TIMEOUT_S)
    fed_until_s = _now_s(world)
    return _Stop(
        issued_s, fed_until_s, _wait_motion_still(world, control, lines)
    )


def _closing_brake(events: list[dict]) -> dict:
    brakes = [
        event
        for event in events
        if event.get("event") == "pipe_drain"
        and event.get("cause") == STOP_BRAKE_CAUSE
    ]
    assert brakes, "no pacer brake brought the stream to rest"
    return brakes[0]


def _assert_stopped_within_bound(
    world, what: str, stop: _Stop, events: list[dict]
) -> None:
    """Once klippy stops feeding, the motion left is what stood ahead
    outside the planning front (at most the engine's `peak_ahead_s`) and
    what the front still held, which the closing brake planned to rest
    (`drained_s`). Any buffer outside that account fails here."""
    brake = _closing_brake(events)
    after_feed_s = stop.ended_s - stop.fed_until_s
    bound_s = brake["peak_ahead_s"] + brake["drained_s"]
    record = {
        "what": what,
        "latency_s": stop.ended_s - stop.issued_s,
        "after_feed_s": after_feed_s,
        "bound_s": bound_s,
        **{key: brake[key] for key in ("cap_s", "peak_ahead_s", "drained_s")},
    }
    with (world.log_dir / "stop_latency.jsonl").open("a") as stream:
        stream.write(json.dumps(record) + "\n")
    assert after_feed_s <= bound_s, (
        f"X/Y kept stepping {after_feed_s:.2f}s after {what} stopped the "
        f"feed, past the {bound_s:.2f}s the engine accounts for: {record}"
    )


def _assert_stopped_in_time(what: str, stop: _Stop) -> None:
    """The stop the bench session rows target once #860 is closed, issue
    to last step (`PAUSE_CANCEL_STOP_S_AFTER_860` in
    tools/bench/session_scenarios.py)."""
    latency_s = stop.ended_s - stop.issued_s
    assert latency_s <= PAUSE_CANCEL_STOP_S_AFTER_860, (
        f"X/Y kept stepping {latency_s:.2f}s after {what} was issued, past "
        f"the {PAUSE_CANCEL_STOP_S_AFTER_860}s a stop may take (#860)"
    )


def _print_ended(cursor: _EventCursor) -> bool:
    return any(event.get("event") == "print.end" for event in cursor.new())


def _assert_clean(world) -> None:
    assert world.shutdown_line() is None, world.log_tail()[-4000:]


def _assert_no_unasked_rest(events: list[dict]) -> None:
    held = [
        event
        for event in events
        if event.get("event") == "anchor_idle_resume"
        and event.get("lead_secs", 0.0) > MAX_RESUME_LEAD_S
    ]
    assert not held, (
        f"{len(held)} resumes held more than {MAX_RESUME_LEAD_S}s ahead of the "
        f"playhead, rests no gcode asked for: "
        f"{[round(event['lead_secs'], 2) for event in held]}"
    )


def _assert_reserve_bounded(events: list[dict]) -> None:
    reserves = [
        event["reserve_s"]
        for event in events
        if event.get("event") == "pacer_reserve_raised"
    ]
    assert all(reserve <= MAX_RESERVE_S for reserve in reserves), (
        f"the pacer raised its reserve past {MAX_RESERVE_S}s: "
        f"{[round(reserve, 2) for reserve in reserves]}"
    )


def test_a_second_print_runs_like_the_first(sim_world, monkeypatch):
    world = _boot(sim_world, monkeypatch)
    control = world.sim_control("h7")
    gcode = _first_layers(world)
    whole_session = _EventCursor(world)

    first_s = _motion_s(world, control, gcode)
    second_print = _EventCursor(world)
    second_s = _motion_s(world, control, gcode)

    _assert_clean(world)
    assert abs(second_s - first_s) <= DURATION_TOLERANCE * first_s, (
        f"the first print moved for {first_s:.1f}s, the second for {second_s:.1f}s"
    )
    _assert_no_unasked_rest(second_print.new())
    _assert_reserve_bounded(whole_session.new())


def test_a_print_after_a_cancel_runs_like_the_first(sim_world, monkeypatch):
    world = _boot(sim_world, monkeypatch)
    control = world.sim_control("h7")
    gcode = _first_layers(world)

    reference_s = _motion_s(world, control, gcode)
    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    _stop(world, control, "CANCEL_PRINT")
    after_cancel = _EventCursor(world)
    again_s = _motion_s(world, control, gcode)

    _assert_clean(world)
    assert abs(again_s - reference_s) <= DURATION_TOLERANCE * reference_s, (
        f"the print moved for {reference_s:.1f}s, after a cancel {again_s:.1f}s"
    )
    _assert_no_unasked_rest(after_cancel.new())


@pytest.mark.parametrize("gcode_name", sorted(GCODES))
def test_pause_stops_motion_within_bound(sim_world, monkeypatch, gcode_name):
    world = _boot(sim_world, monkeypatch)
    control = world.sim_control("h7")
    gcode = GCODES[gcode_name](world)

    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    paused = _EventCursor(world)
    stop = _stop(world, control, "PAUSE")
    stop_events = paused.new()
    world.gcode_ok("RESUME", timeout=PRINT_TIMEOUT_S)
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    _assert_clean(world)
    _assert_stopped_within_bound(world, "PAUSE", stop, stop_events)


@pytest.mark.parametrize("gcode_name", sorted(GCODES))
def test_cancel_stops_motion_within_bound(sim_world, monkeypatch, gcode_name):
    world = _boot(sim_world, monkeypatch)
    control = world.sim_control("h7")
    gcode = GCODES[gcode_name](world)

    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    cancelled = _EventCursor(world)
    stop = _stop(world, control, "CANCEL_PRINT")

    _assert_clean(world)
    state = world.status({"print_stats": None})["print_stats"]["state"]
    assert state == "cancelled", state
    _assert_stopped_within_bound(world, "CANCEL_PRINT", stop, cancelled.new())


def test_print_stats_completes_with_the_motion(sim_world, monkeypatch):
    world = _boot(sim_world, monkeypatch)
    control = world.sim_control("h7")
    gcode = _long_moves(world)
    printing = _EventCursor(world)

    _start(world, gcode)
    world.wait_until(
        lambda: _print_ended(printing),
        timeout=PRINT_TIMEOUT_S,
        what="print_stats print.end",
        poll=0.05,
    )
    ended_s = _now_s(world)
    completing = _EventCursor(world)
    stop = _Stop(ended_s, ended_s, _wait_motion_still(world, control))

    _assert_clean(world)
    assert (
        world.status({"print_stats": None})["print_stats"]["state"]
        == "complete"
    )
    _assert_stopped_within_bound(
        world, "print_stats complete", stop, completing.new()
    )


def _boot_bench(sim_world, monkeypatch, config):
    monkeypatch.setenv("RUST_LOG", PACER_LOG)
    world = sim_world(
        lambda w: config(w.h7_pty, str(w.gcode_dir)), dual_mcu=False
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    return world


def _assert_no_pacer_brake(world, events: list[dict]) -> None:
    """A print that ends in M400 is never braked by the pacer: the flush
    drains it, so every `pipe_drain` here stopped the machine mid-print."""
    brakes = [event for event in events if event.get("event") == "pipe_drain"]
    assert not brakes, (
        f"the pacer braked the print to rest {len(brakes)} times: {brakes}"
    )
    _assert_clean(world)


def test_long_strokes_print_without_a_pacer_brake(sim_world, monkeypatch):
    """Pi bench 2026-11-16, speed_ladder on printer.cfg: three pacer brakes
    mid-print, each inside an 18-20 s klippy wait on the intake, with the
    committed runway at the reserve while the motion counted ahead held the
    intake shut. Main printed it without a brake."""
    world = _boot_bench(sim_world, monkeypatch, configs.bench_print_config)
    printing = _EventCursor(world)

    world.print_file(SPEED_LADDER, timeout=PRINT_TIMEOUT_S)

    events = printing.new()
    _assert_no_pacer_brake(world, events)
    waits = [
        event["waited_s"]
        for event in events
        if event.get("event") == "engine_wait_done"
        and event.get("what") == PIPE_SPACE
    ]
    assert max(waits, default=0.0) <= MAX_PIPE_SPACE_WAIT_S, (
        f"klippy sat parked on {PIPE_SPACE!r} for up to {max(waits):.2f}s, "
        f"past the {FRONT_RELEASE_AT_START_S}s of strokes the planning front "
        f"hands on at once at the print start (#860): {waits}"
    )


def test_a_host_faster_than_playback_never_brakes(sim_world, monkeypatch):
    """Pi bench 2026-11-16, trident_kernel on two cores at 1500 MHz: a
    pacer brake 0.3 s into the print, at the runway one unmeasured brake
    takes. The host plans faster than the print plays. The throttle charges
    by stream time, so it starts after the first travel: a Pi plans that
    1.9 s move like any other, the throttle would charge it 1 s."""
    monkeypatch.setenv(
        "SERVAL_PRODUCER_THROTTLE",
        f"{TRIDENT_2_CPU_REALTIME_FACTOR}@{DENSE_TOP_FIRST_TRAVEL_END_S}",
    )
    world = _boot_bench(
        sim_world, monkeypatch, configs.bench_trident_kernel_config
    )
    printing = _EventCursor(world)

    world.print_file(DENSE_TOP, timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    _assert_no_pacer_brake(world, printing.new())


def test_a_dwell_behind_long_moves_starts_without_a_brake(
    sim_world, monkeypatch
):
    """#856, Pi bench 2026-11-15: a Z move, a travel, a zigzag of 80 mm
    moves and `G4 P1` aborted klippy 0.5 s after the first anchor on main
    with `anchor_underrun`: the playhead overran the committed end."""
    world = _boot_bench(sim_world, monkeypatch, configs.bench_print_config)
    path = world.gcode_dir / "zigzag_dwell.gcode"
    lines = _zigzag_lines(DWELL_ZIGZAG_MOVES) + ["G4 P1", "M400"]
    path.write_text("\n".join(lines) + "\n")
    printing = _EventCursor(world)

    world.print_file(path, timeout=PRINT_TIMEOUT_S)

    _assert_no_pacer_brake(world, printing.new())


@pytest.mark.parametrize("gcode_name", BENCH_STOPS)
@pytest.mark.parametrize("command", ("PAUSE", "CANCEL_PRINT"))
def test_a_stop_on_the_bench_printer_ends_within_bound(
    sim_world, monkeypatch, command, gcode_name
):
    """Pi bench 2026-11-16, printer.cfg, 20 s in: on first_layers motion
    ran on 7.4-8.5 s after PAUSE and CANCEL_PRINT, here 9.1 s; on the
    zigzag (`pause_sparse`) the PAUSE never acted. The turning guard held
    the strokes back (#860); the engine must still account for all of it."""
    world = _boot_bench(sim_world, monkeypatch, configs.bench_print_config)
    control = world.sim_control("h7")
    gcode = GCODES[gcode_name](world)

    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    stopping = _EventCursor(world)
    stop = _stop(world, control, command, BENCH_AB_STEP_LINES)

    _assert_clean(world)
    _assert_stopped_within_bound(world, command, stop, stopping.new())
    _assert_stopped_in_time(command, stop)
