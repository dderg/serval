"""Replay the servo-ident stroke/dwell pattern and audit the executed A/B
motor tracks for silent position welds at stroke boundaries.

Bench ground truth (trident, ident_20260710_002707.scap): every stroke
boundary shows the command stream at rest, then one 250us sample jumping
0.03-0.17 mm (up to 0.72 mm in the tracking captures), then rest again.
Here the shim's step trackers stand in for the drive capture: at every
rest point the counters must (a) hold still through the dwell and the
host's idle think-time, and (b) sit exactly on the ideal kinematic
position — a truncated brake tail or a weld violates one of the two.
"""

import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = 200 * 16 / 40  # 80: full steps x microsteps / rotation_distance
STROKE_MM = 60
STROKES = 12
X_HOME = 100
SPEED_MM_MIN = 30000  # 500 mm/s, the bench ident cruise
DWELL_MS = 1200  # the ident macro's inter-stroke G4
THINK_S = 1.0  # script processing pause; forces the pacer's idle path
WELD_TOL_STEPS = 3  # 3 steps = 0.0375 mm; bench welds are 3-57 steps

A_LINE = 18
B_LINE = 7


def read_track(world, line: int) -> int:
    resp = world.sim_control("h7").send(f"get_steps line={line}")
    kv = dict(p.split("=") for p in resp.split())
    assert "steps" in kv, f"step tracker for line {line} missing: {resp!r}"
    return int(kv["steps"])


def test_ident_stroke_dwell_boundaries_hold_position(sim_world):
    world = sim_world(
        lambda w: configs.corexy_tracked_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok(f"SET_KINEMATIC_POSITION X={X_HOME} Y=100 Z=10")
    world.gcode_ok("M400")
    base_a = read_track(world, A_LINE)
    base_b = read_track(world, B_LINE)

    problems = []
    sign_a = sign_b = None
    x = X_HOME
    for stroke in range(STROKES):
        direction = 1 if stroke % 2 == 0 else -1
        x += direction * STROKE_MM
        world.gcode_ok(f"G1 X{x} F{SPEED_MM_MIN}")
        world.gcode_ok("M400")
        end_a = read_track(world, A_LINE)
        end_b = read_track(world, B_LINE)

        world.gcode_ok(f"G4 P{DWELL_MS}")
        world.gcode_ok("M400")
        dwell_a = read_track(world, A_LINE)
        dwell_b = read_track(world, B_LINE)

        time.sleep(THINK_S)
        idle_a = read_track(world, A_LINE)
        idle_b = read_track(world, B_LINE)

        if sign_a is None:
            sign_a = 1 if end_a >= base_a else -1
            sign_b = 1 if end_b >= base_b else -1
        ideal_a = base_a + sign_a * round((x - X_HOME) * STEPS_PER_MM)
        ideal_b = base_b + sign_b * round((x - X_HOME) * STEPS_PER_MM)

        print(
            f"stroke {stroke}: x={x} a={end_a}/{dwell_a}/{idle_a} "
            f"(ideal {ideal_a}) b={end_b}/{dwell_b}/{idle_b} (ideal {ideal_b})"
        )
        for name, end, dwell, idle, ideal in (
            ("a", end_a, dwell_a, idle_a, ideal_a),
            ("b", end_b, dwell_b, idle_b, ideal_b),
        ):
            if dwell != end or idle != dwell:
                problems.append(
                    f"stroke {stroke} motor {name}: moved at rest "
                    f"(end={end} after-dwell={dwell} after-idle={idle}) — "
                    f"weld executed after the brake"
                )
            err = idle - ideal
            if abs(err) > WELD_TOL_STEPS:
                problems.append(
                    f"stroke {stroke} motor {name}: rest position off ideal "
                    f"by {err} steps ({err / STEPS_PER_MM:+.4f} mm)"
                )

    events = world.events_text()
    for marker in ("anchor_underrun", "junction_jump_anomalous", "pipe_drain"):
        print(f"events {marker}: {events.count(marker)}")

    assert world.shutdown_line() is None, world.log_tail()
    assert not problems, "\n".join(problems)
