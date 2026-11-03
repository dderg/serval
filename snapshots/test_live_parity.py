"""The snapshot path must plan exactly what the live host submits.

Each case feeds the same G-code to klippy's own handlers (``live_replay``)
and to ``_motion_engine.parse_gcode``, then compares the moves move for move:
displacement, feedrate and the effective (velocity, accel, corner_deviation)
the planner runs each under.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import harness  # noqa: E402
import live_replay  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
BENCH_CFG = REPO / "tools" / "bench" / "printer.cfg"


@pytest.fixture(scope="module")
def engine():
    try:
        return harness._import_engine()
    except ImportError as exc:
        pytest.skip(str(exc))


def live_steps(gcode: str, cfg: Path) -> list[tuple]:
    steps = live_replay.replay(gcode, cfg)
    anchor = next(
        i
        for i, step in enumerate(steps)
        if isinstance(step, live_replay.LiveMove) and any(step.delta[:3])
    )
    return [
        ("move", step.delta, step.feedrate, step.limits)
        for step in steps[anchor + 1 :]
        if isinstance(step, live_replay.LiveMove)
    ]


def snapshot_steps(engine, gcode: str, cfg: Path) -> list[tuple]:
    config_text = harness.read_printer_config(cfg).config_text
    waypoints = engine.parse_gcode(gcode, config_text)
    return [
        (
            "move",
            tuple(e - s for s, e in zip(move["start"], move["end"])),
            move["feedrate"],
            (move["velocity"], move["accel"], move["corner_deviation"]),
        )
        for move in engine.snapshot_moves(waypoints, config_text)
    ]


def assert_parity(engine, gcode: str, cfg: Path = BENCH_CFG) -> list[tuple]:
    live = live_steps(gcode, cfg)
    snapshot = snapshot_steps(engine, gcode, cfg)
    assert snapshot == live
    return live


def test_set_velocity_limit_accel_above_the_config_clamps_like_live(engine):
    steps = assert_parity(
        engine,
        "M83\n"
        "G1 X10 Y10 F6000\n"
        "SET_VELOCITY_LIMIT ACCEL=50000\n"
        "G1 X50 Y10 E1 F6000\n"
        "SET_VELOCITY_LIMIT ACCEL=2000\n"
        "G1 X50 Y50 E1 F6000\n",
    )
    assert [accel for _, _, _, (_, accel, _) in steps] == [10_000.0, 2000.0]


def test_set_velocity_limit_velocity_and_corner_fields_match_live(engine):
    steps = assert_parity(
        engine,
        "M83\n"
        "G1 X10 Y10 F6000\n"
        "SET_VELOCITY_LIMIT VELOCITY=2000 SQUARE_CORNER_VELOCITY=12\n"
        "G1 X50 Y10 E1 F6000\n"
        "SET_VELOCITY_LIMIT VELOCITY=400 CORNER_DEVIATION=0.05"
        " MINIMUM_CRUISE_RATIO=0.5 ACCEL_TO_DECEL=100\n"
        "G1 X50 Y50 E1 F6000\n"
        "SET_VELOCITY_LIMIT\n"
        "G1 X10 Y50 E1 F6000\n",
    )
    assert [limits for *_, limits in steps] == [
        (1000.0, 10_000.0, steps[0][3][2]),
        (400.0, 10_000.0, 0.05),
        (400.0, 10_000.0, 0.05),
    ]
    assert steps[0][3][2] != 0.21


def test_reset_velocity_limit_matches_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 F6000\n"
        "SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=800 CORNER_DEVIATION=0.01\n"
        "G1 X50 Y10 F6000\n"
        "RESET_VELOCITY_LIMIT\n"
        "G1 X50 Y50 F6000\n",
    )
    assert [limits for *_, limits in steps] == [
        (300.0, 800.0, 0.01),
        (1000.0, 10_000.0, 0.21),
    ]


def test_m204_matches_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 F6000\n"
        "M204 S700\n"
        "G1 X50 Y10 F6000\n"
        "M204 P900 T800\n"
        "G1 X50 Y50 F6000\n"
        "M204 P100\n"
        "G1 X10 Y50 F6000\n"
        "M204 S50000\n"
        "G1 X10 Y10 F6000\n",
    )
    assert [accel for *_, (_, accel, _) in steps] == [
        700.0,
        800.0,
        800.0,
        10_000.0,
    ]
