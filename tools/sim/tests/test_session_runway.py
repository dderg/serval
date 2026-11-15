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
FIRST_LAYERS_LINES = 1500
LONG_MOVE_COUNT = 60
LONG_MOVE_MM = 80.0
LONG_MOVE_FEED = 6000

ACTION_AT_S = 20.0
MOTION_STILL_S = 2.0
PRINT_TIMEOUT_S = 1200.0
NS_PER_S = 1e9

DURATION_TOLERANCE = 0.10
MAX_RESUME_LEAD_S = 1.0
MAX_RESERVE_S = 1.0
STOP_BOUND_S = 5.0

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


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir))
        + PAUSE_RESUME,
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    return world


def _first_layers(world) -> pathlib.Path:
    path = world.gcode_dir / "first_layers.gcode"
    lines = FIRST_LAYERS.read_text().splitlines()[:FIRST_LAYERS_LINES]
    path.write_text("\n".join(lines) + "\n")
    return path


def _long_moves(world) -> pathlib.Path:
    path = world.gcode_dir / "long_moves.gcode"
    lines = ["G90", "M83", "G1 Z0.3 F600", "G1 X60 Y20 F12000"]
    for index in range(LONG_MOVE_COUNT):
        x = 60.0 + LONG_MOVE_MM if index % 2 == 0 else 60.0
        y = 20.0 + (index // 2) * 0.4
        lines.append(f"G1 X{x:.1f} Y{y:.2f} E0.3 F{LONG_MOVE_FEED}")
    path.write_text("\n".join(lines) + "\n")
    return path


GCODES = {"first_layers": _first_layers, "long_moves": _long_moves}


def _now_s(world) -> float:
    return world.vclock.now_ns() / NS_PER_S


def _last_step_s(control) -> float:
    return (
        max(
            control.get_step_times(X_STEP_LINE)["last_ns"],
            control.get_step_times(Y_STEP_LINE)["last_ns"],
        )
        / NS_PER_S
    )


def _wait_motion_still(world, control) -> float:
    """The time of the last X/Y step once the axes have been still for
    MOTION_STILL_S."""
    world.wait_until(
        lambda: _now_s(world) - _last_step_s(control) >= MOTION_STILL_S,
        timeout=PRINT_TIMEOUT_S,
        what=f"X/Y still for {MOTION_STILL_S}s",
        poll=0.05,
    )
    return _last_step_s(control)


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


def _stop_latency_s(world, control, command: str) -> float:
    issued_s = _now_s(world)
    world.gcode_ok(command, timeout=PRINT_TIMEOUT_S)
    return _wait_motion_still(world, control) - issued_s


def _print_state(world) -> str:
    return (
        world.status({"print_stats": None}).get("print_stats", {}).get("state")
    )


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


def test_a_second_print_runs_like_the_first(sim_world):
    world = _boot(sim_world)
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


def test_a_print_after_a_cancel_runs_like_the_first(sim_world):
    world = _boot(sim_world)
    control = world.sim_control("h7")
    gcode = _first_layers(world)

    reference_s = _motion_s(world, control, gcode)
    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    _stop_latency_s(world, control, "CANCEL_PRINT")
    after_cancel = _EventCursor(world)
    again_s = _motion_s(world, control, gcode)

    _assert_clean(world)
    assert abs(again_s - reference_s) <= DURATION_TOLERANCE * reference_s, (
        f"the print moved for {reference_s:.1f}s, after a cancel {again_s:.1f}s"
    )
    _assert_no_unasked_rest(after_cancel.new())


@pytest.mark.parametrize("gcode_name", sorted(GCODES))
def test_pause_stops_motion_within_bound(sim_world, gcode_name):
    world = _boot(sim_world)
    control = world.sim_control("h7")
    gcode = GCODES[gcode_name](world)

    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    latency_s = _stop_latency_s(world, control, "PAUSE")
    world.gcode_ok("RESUME", timeout=PRINT_TIMEOUT_S)
    world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    _assert_clean(world)
    assert latency_s <= STOP_BOUND_S, (
        f"X/Y kept stepping {latency_s:.1f}s after PAUSE "
        f"(bound {STOP_BOUND_S}s)"
    )


@pytest.mark.parametrize("gcode_name", sorted(GCODES))
def test_cancel_stops_motion_within_bound(sim_world, gcode_name):
    world = _boot(sim_world)
    control = world.sim_control("h7")
    gcode = GCODES[gcode_name](world)

    started_s = _start(world, gcode)
    _wait_print_time(world, started_s, ACTION_AT_S)
    latency_s = _stop_latency_s(world, control, "CANCEL_PRINT")

    _assert_clean(world)
    state = world.status({"print_stats": None})["print_stats"]["state"]
    assert state == "cancelled", state
    assert latency_s <= STOP_BOUND_S, (
        f"X/Y kept stepping {latency_s:.1f}s after CANCEL_PRINT "
        f"(bound {STOP_BOUND_S}s)"
    )


def test_print_stats_completes_with_the_motion(sim_world):
    world = _boot(sim_world)
    control = world.sim_control("h7")
    gcode = _long_moves(world)

    _start(world, gcode)
    world.wait_until(
        lambda: _print_state(world) == "complete",
        timeout=PRINT_TIMEOUT_S,
        what="print_stats complete",
        poll=0.05,
    )
    completed_s = _now_s(world)
    motion_end_s = _wait_motion_still(world, control)

    _assert_clean(world)
    assert motion_end_s - completed_s <= STOP_BOUND_S, (
        f"print_stats reported complete {motion_end_s - completed_s:.1f}s "
        f"before the motion ended (bound {STOP_BOUND_S}s)"
    )
