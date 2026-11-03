"""The snapshot path must plan exactly what the live host submits.

Each case feeds the same G-code to klippy's own handlers (``live_replay``)
and to ``_motion_engine.parse_gcode``, then compares the moves move for move:
displacement, feedrate and the effective (velocity, accel, corner_deviation)
the planner runs each under, plus every stop at rest between two moves and
the time the toolhead holds there. Everything up to the first move with
displacement, or a SET_KINEMATIC_POSITION before it, is the snapshot's anchor
and stays out of the comparison, as do
rests after the last move: they leave the planned trajectory unchanged. Rest
durations agree to 1e-12 s; the live host derives a hold from absolute print
times, so it carries their rounding.
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
        if isinstance(step, live_replay.LivePosition)
        or (isinstance(step, live_replay.LiveMove) and any(step.delta[:3]))
    )
    compared = []
    rest_s = None
    for step in steps[anchor + 1 :]:
        if isinstance(step, live_replay.LivePosition):
            continue
        if isinstance(step, live_replay.LiveRest):
            rest_s = (rest_s or 0.0) + step.dwell_s
            continue
        if rest_s is not None:
            compared.append(("rest", round(rest_s, 12)))
            rest_s = None
        compared.append(("move", step.delta, step.feedrate, step.limits))
    return compared


def snapshot_steps(engine, gcode: str, cfg: Path) -> list[tuple]:
    config_text = harness.read_printer_config(cfg).config_text
    waypoints = engine.parse_gcode(gcode, config_text)
    compared = []
    for move in engine.snapshot_moves(waypoints, config_text):
        if move["dwell_before_s"] is not None:
            compared.append(("rest", round(move["dwell_before_s"], 12)))
        compared.append(
            (
                "move",
                tuple(e - s for s, e in zip(move["start"], move["end"])),
                move["feedrate"],
                (move["velocity"], move["accel"], move["corner_deviation"]),
            )
        )
    return compared


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


def moves(steps: list[tuple]) -> list[tuple]:
    return [step for step in steps if step[0] == "move"]


def feedrates(steps: list[tuple]) -> list[float]:
    return [feedrate for _, _, feedrate, _ in moves(steps)]


def test_g0_and_g1_feedrates_match_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10\n"
        "G1 X20 Y10\n"
        "G0 X40 F12000\n"
        "G1 X60\n"
        "G0 Y40\n"
        "G1 X60 Y40 F600\n"
        "G1 X70\n",
    )
    assert feedrates(steps) == [25.0, 200.0, 200.0, 200.0, 10.0]


def test_feedrate_caps_match_live(engine):
    steps = assert_parity(
        engine,
        "M83\n"
        "G1 X10 Y10 F120000\n"
        "G1 X50 Y10\n"
        "SET_VELOCITY_LIMIT VELOCITY=80\n"
        "G1 X50 Y50\n"
        "G1 X60 Y50 Z3\n"
        "G1 Z6 F3000\n"
        "G1 X60 Y50 Z7 F120000\n"
        "G1 E-2 F120000\n"
        "G1 X60.0000000001 Y50 E2\n",
    )
    assert feedrates(steps) == [1000.0, 80.0, 80.0, 25.0, 25.0, 2000.0, 2000.0]


def extrusions(steps: list[tuple]) -> list[float]:
    return [delta[3] for _, delta, _, _ in moves(steps)]


def test_g92_shifts_the_coordinate_frame_like_live(engine):
    steps = assert_parity(
        engine,
        "M83\n"
        "G1 X10 Y10 F6000\n"
        "G1 X20 E1\n"
        "G92 E0\n"
        "G1 X30 E1\n"
        "M82\n"
        "G1 X40 E3\n"
        "G92 E0\n"
        "G1 X50 E0.5\n"
        "G92 X0 Y0\n"
        "G1 X10 Y5\n"
        "G92\n"
        "G1 X5\n",
    )
    assert extrusions(steps) == [1.0, 1.0, 2.0, 0.5, 0.0, 0.0]
    assert [delta[:2] for _, delta, _, _ in steps[-2:]] == [
        (10.0, 5.0),
        (5.0, 0.0),
    ]


def test_extrusion_mode_matches_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 E1 F6000\n"
        "G1 X20 E3\n"
        "G91\n"
        "G1 X10 E1\n"
        "M82\n"
        "G1 X10 E1\n"
        "G90\n"
        "G1 X50 E6\n"
        "M83\n"
        "G1 X60 E1\n"
        "G91\n"
        "M82\n"
        "G1 X10 E2\n",
    )
    assert extrusions(steps) == [2.0, 1.0, 1.0, 1.0, 1.0, 2.0]


def rests(steps: list[tuple]) -> list[float]:
    return [step[1] for step in steps if step[0] == "rest"]


def test_stop_at_rest_commands_match_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 F6000\n"
        "G1 X20\n"
        "G4 P250\n"
        "G1 X30\n"
        "M400\n"
        "G4 P100\n"
        "G1 X40\n"
        "SET_KINEMATIC_POSITION X=40 Y=10 Z=0\n"
        "G1 X50\n"
        "M84\n"
        "SET_KINEMATIC_POSITION X=50 Y=10 Z=0\n"
        "G1 X60\n"
        "G4\n"
        "G1 X70\n"
        "M18\n",
    )
    assert rests(steps) == [0.25, 0.1, 0.0, 0.2, 0.0]


def snapshot_refusal(engine, gcode: str, cfg: Path = BENCH_CFG) -> str:
    config_text = harness.read_printer_config(cfg).config_text
    with pytest.raises(ValueError) as refused:
        engine.parse_gcode(gcode, config_text)
    return str(refused.value)


def test_moves_after_motors_off_are_refused_like_live(engine):
    gcode = "G1 X10 Y10 F6000\nM84\nG1 E1\nG1 X20\n"
    with pytest.raises(Exception, match="Must home axis first"):
        live_replay.replay(gcode, BENCH_CFG)
    assert "Must home axis first" in snapshot_refusal(engine, gcode)


def test_set_kinematic_position_away_from_the_toolhead_is_refused(engine):
    refusal = snapshot_refusal(
        engine, "G1 X10 Y10 F6000\nSET_KINEMATIC_POSITION X=0\nG1 X20\n"
    )
    assert "SET_KINEMATIC_POSITION" in refusal


def test_speed_and_extrude_factors_match_live(engine):
    steps = assert_parity(
        engine,
        "M83\n"
        "G1 X10 Y10 F6000\n"
        "M220 S50\n"
        "G1 X20 E1\n"
        "G1 X30 E1 F3000\n"
        "M221 S90\n"
        "G1 X40 E1\n"
        "M82\n"
        "G92 E0\n"
        "G1 X50 E2\n"
        "M220\n"
        "M221\n"
        "G1 X60 E3\n",
    )
    assert feedrates(steps) == [50.0, 25.0, 25.0, 25.0, 50.0]
    assert extrusions(steps) == pytest.approx([1.0, 1.0, 0.9, 1.8, 1.0])


def test_gcode_offsets_and_saved_states_match_live(engine):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 F6000\n"
        "SET_GCODE_OFFSET Z=0.2\n"
        "G1 X20 Z1\n"
        "SET_GCODE_OFFSET Z_ADJUST=-0.05 MOVE=1\n"
        "G1 X30\n"
        "SET_GCODE_OFFSET X=5 MOVE=1 MOVE_SPEED=20\n"
        "G1 X40\n"
        "SAVE_GCODE_STATE NAME=park\n"
        "G91\n"
        "M220 S200\n"
        "G1 X5 Y5\n"
        "RESTORE_GCODE_STATE NAME=park MOVE=1 MOVE_SPEED=50\n"
        "G1 X50\n",
    )
    displacements = [v for _, delta, _, _ in moves(steps) for v in delta[:3]]
    assert displacements == pytest.approx(
        [10.0, 0.0, 1.2]
        + [0.0, 0.0, -0.05]
        + [10.0, 0.0, 0.0]
        + [5.0, 0.0, 0.0]
        + [10.0, 0.0, 0.0]
        + [5.0, 5.0, 0.0]
        + [-5.0, -5.0, 0.0]
        + [10.0, 0.0, 0.0]
    )
    assert feedrates(steps)[-3:] == [200.0, 50.0, 100.0]


@pytest.mark.parametrize(
    ("gcode", "error"),
    [
        ("G1 X10 Y10 F6000\nG20\nG1 X20\n", "G20"),
        ("G1 X10 Y10 F6000\nRESTORE_GCODE_STATE NAME=nope\n", "nope"),
        ("G1 X10 Y10 F6000\nM220 S0\n", "S"),
        ("G1 X10 Y10 F6000\nM221 S-5\n", "S"),
    ],
)
def test_gcode_state_errors_are_refused_like_live(engine, gcode, error):
    with pytest.raises(Exception, match=error):
        live_replay.replay(gcode, BENCH_CFG)
    assert error in snapshot_refusal(engine, gcode)


def test_unknown_commands_are_ignored_whatever_their_arguments_like_live(
    engine,
):
    steps = assert_parity(
        engine,
        "G1 X10 Y10 F6000\n"
        "M117 Layer 2 of 10.5.1\n"
        "M118 hello: world\n"
        'RESPOND MSG="half done"\n'
        "M73 P50 R3\n"
        "M104 S0\n"
        "T0\n"
        "G1 X20\n",
    )
    assert len(steps) == 1


@pytest.mark.parametrize(
    "command",
    [
        "G2 X20 Y10 I5 J0",
        "G3 X20 Y10 I5 J0",
        "G5 X20 Y10 I1 J1 P1 Q1",
        "G5.1 X20 Y10 I1 J1",
        "G10",
        "G11",
        "G28",
        "SET_PRESSURE_ADVANCE ADVANCE=0.03",
        "SET_POST_PROCESSOR NAME=pa K=0.5",
        "BED_MESH_PROFILE LOAD=default",
        "BED_MESH_CLEAR",
        "FORCE_MOVE STEPPER=stepper_x DISTANCE=5 VELOCITY=10",
        "M109 S215",
    ],
)
def test_commands_the_snapshot_cannot_plan_are_refused_by_name(engine, command):
    name = command.split()[0]
    refusal = snapshot_refusal(engine, f"G1 X10 Y10 F6000\n{command}\nG1 X20\n")
    assert f"line 2: {name}:" in refusal


def test_set_kinematic_position_before_any_move_is_where_both_start(engine):
    steps = assert_parity(
        engine,
        "SET_KINEMATIC_POSITION X=5 Y=5 Z=0\n"
        "G92 E0\n"
        "M83\n"
        "G1 X10 Y5 E1 F3000\n"
        "G1 Y20\n",
    )
    assert [delta[:2] for _, delta, _, _ in moves(steps)] == [
        (5.0, 0.0),
        (0.0, 15.0),
    ]
