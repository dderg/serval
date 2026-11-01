"""Two MCUs whose crystals disagree must still step one plan together.

X steps on the main MCU, Y on the secondary; every move drives both the
same distance from lattice-aligned starts, so in a perfect world each step
of Y lands at the same instant as the matching step of X. The secondary's
crystal runs fast (the main's slow), as real boards do; the host only
learns their rates through clock sync, and each dwell between moves is a
re-sync point for the host→MCU clock mapping.

Contract: the clock-sync stability gate admits a slope error of at most
clock_sync_stable_ppm, so a mapping re-synced at every rest cannot let the
cross-MCU timing error grow faster than that; over a session it can grow by
at most the gate times the session length. The sim stamps a step when the
MCU's timer actually toggles the pin, which may trail its scheduled clock
by the vtime pacer slack. A mapping frozen at the first move drifts by that
move's estimate error for as long as the session runs, and never logs a
re-sync.
"""

import json

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

CRYSTAL_PPM = {"h7": -20.0, "f4": 30.0}
X_STEP_LINE = 0
Y_STEP_LINE = 3
MOVES = 80
DWELL_MS = 2000
STEPS_PER_MOVE = 8000
CLOCK_SYNC_STABLE_PPM = 5.0
VTIME_PACER_SLACK_S = 400e-6


def _dual_mcu_cartesian(world):
    text = configs.minimal_config(world.h7_pty, str(world.gcode_dir))
    text = text.replace(
        f"[mcu]\nserial: {world.h7_pty}\n",
        f"[mcu]\nserial: {world.h7_pty}\n\n[mcu bottom]\nserial: {world.f4_pty}\n",
    )
    for pin in ("step_pin: ", "dir_pin: ", "enable_pin: !"):
        line = {"step_pin: ": 3, "dir_pin: ": 4, "enable_pin: !": 5}[pin]
        text = text.replace(
            f"{pin}gpiochip0/gpio{line}\n",
            f"{pin}bottom:gpiochip0/gpio{line}\n",
        )
    assert "bottom:gpiochip0/gpio3" in text
    return text


def _mean_step_time(control, line):
    times = control.get_step_times(line)
    assert times["count"] == STEPS_PER_MOVE, times
    return times["sum_ns"] / times["count"] / 1e9


def test_secondary_mcu_steps_stay_in_time_across_a_long_session(sim_world):
    world = sim_world(_dual_mcu_cartesian, crystal_ppm=CRYSTAL_PPM)
    main = world.sim_control("h7")
    secondary = world.sim_control("f4")
    world.gcode_ok("SET_KINEMATIC_POSITION X=50 Y=50 Z=50", timeout=10)

    moves = []
    for move in range(MOVES):
        target = 150 if move % 2 == 0 else 50
        main.reset_step_times(X_STEP_LINE)
        secondary.reset_step_times(Y_STEP_LINE)
        world.gcode_ok(f"G1 X{target} Y{target} F6000", timeout=30)
        world.gcode_ok("M400", timeout=30)
        main_at = _mean_step_time(main, X_STEP_LINE)
        moves.append(
            (main_at, _mean_step_time(secondary, Y_STEP_LINE) - main_at)
        )
        world.gcode_ok(f"G4 P{DWELL_MS}", timeout=30)
    assert world.shutdown_line() is None

    gate = CLOCK_SYNC_STABLE_PPM * 1e-6
    session_start, start_offset = moves[0]
    report = " ".join(f"{offset * 1e6:+.0f}" for _, offset in moves)
    for at, offset in moves:
        growth = offset - start_offset
        bound = VTIME_PACER_SLACK_S + gate * (at - session_start)
        assert abs(growth) <= bound, (
            f"cross-MCU step timing moved {growth * 1e6:+.0f} us, past the"
            f" {bound * 1e6:.0f} us the clock-sync gate allows"
            f" {at - session_start:.0f} s into the session; per-move"
            f" offsets (us) {report}"
        )

    rests = 0
    resyncs = {}
    for line in world.events_text().splitlines():
        event = json.loads(line)
        if event.get("event") in ("anchor_idle_resume", "anchor_rejoin"):
            rests += 1
        elif event.get("event") == "clock_resync":
            resyncs.setdefault(event["mcu"], []).append(event)
    assert rests >= MOVES - 1, rests
    assert len(resyncs) == 2, resyncs.keys()
    for mcu, events in resyncs.items():
        assert len(events) >= rests, (
            f"mcu {mcu} logged {len(events)} re-syncs for {rests} rests"
        )
        for event in events:
            assert (
                event["applied"]
                and -event["correction_us"] * 1e-6 < (event["rest_s"])
            ), f"a re-sync must land inside its rest: {event}"
