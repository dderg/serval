"""The corner acceleration burst on owner.cfg (smooth_mzv 200/130 Hz,
50 000 mm/s^2): unbounded by default, within what CornerAccel says the
planner admits, bounded by max_corner_accel when set, and kept within a
motor's limit at recommended_max_accel."""

import math
import pathlib
import tempfile

import numpy as np
import pytest

from klippy.extras import corner_deviation as cd
from klippy.extras import shaper_defs
from tools.motion_eval import evaluate, fuzz, run, sweep

OWNER_CFG = sweep.HERE / "configs" / "owner.cfg"
MAX_ACCEL = 50000.0
MAX_VELOCITY = 800.0
BOUND = 1.03 * MAX_ACCEL
MOTOR_ACCEL = MAX_ACCEL
DT = 25e-6
WORST_CASE = "voron_top_layers"
OWNER_SPECS = tuple(
    ("smoother",)
    + tuple(shaper_defs.get_mzv_smoother(f, normalize_coeffs=False))
    for f in (200.0, 130.0)
)
ALPHA_DEG = 90.0
TURN_DEG = 14.9
START = (150.0, 20.0, 10.0)
LEG_MM = 150.0


def _model():
    kx, ky = (cd.kernel_from_spec(s) for s in OWNER_SPECS)
    arrays = cd.table_arrays(OWNER_SPECS, kx, ky, fuzz.TABLE_CACHE_DIR)
    return cd.CornerAccel(arrays, kx, ky), kx


def _run(section, gcode, start):
    with tempfile.TemporaryDirectory() as tmp:
        config = pathlib.Path(tmp) / "printer.cfg"
        config.write_text("[include %s]\n\n%s" % (OWNER_CFG, section))
        raw = run.run_klippy(config, gcode, start)
    _, _, _, _, _, motion = evaluate.executed_motion(raw.capture, DT)
    peaks = {a: float(np.abs(motion[a].acc).max()) for a in "xy"}
    return raw.capture, peaks


def _case(name):
    text = (sweep.CORPUS / (name + ".gcode")).read_text()
    return text, sweep.case_start(text)


def _fast_shallow_corner(accel):
    """Full speed along +Y, then a 14.9 degree turn: the table cell with
    the largest X velocity step at the floor speed."""
    turn = math.radians(ALPHA_DEG + TURN_DEG)
    x, y, z = START
    corner = (x, y + LEG_MM)
    end = (
        corner[0] + LEG_MM * math.cos(turn),
        corner[1] + LEG_MM * math.sin(turn),
    )
    return "SET_VELOCITY_LIMIT ACCEL=%r\nG1 X%r Y%r F%r\nG1 X%r Y%r\n" % (
        accel,
        corner[0],
        corner[1],
        60.0 * MAX_VELOCITY,
        end[0],
        end[1],
    )


def test_unbounded_is_at_least_as_fast_as_bounded():
    text, start = _case(WORST_CASE)
    free, _ = _run("[corner_deviation]\n", text, start)
    bounded, _ = _run(
        "[corner_deviation]\nmax_corner_accel: %r\n" % (BOUND,), text, start
    )

    def span(capture):
        moves = capture["moves"]
        last = moves[-1]
        end = (
            last["print_time"]
            + last["accel_t"]
            + last["cruise_t"]
            + last["decel_t"]
        )
        return end - moves[0]["print_time"]

    assert span(free) <= span(bounded)


def test_max_corner_accel_holds_the_burst():
    text, start = _case(WORST_CASE)
    _, peaks = _run(
        "[corner_deviation]\nmax_corner_accel: %r\n" % (BOUND,), text, start
    )
    assert max(peaks.values()) <= BOUND * (1.0 + cd.ACCEL_MARGIN)


@pytest.mark.parametrize(
    "case", [WORST_CASE, "corners_turn90", "zigzag_infill"]
)
def test_executed_peak_within_what_the_planner_admits(case):
    text, start = _case(case)
    capture, peaks = _run("[corner_deviation]\n", text, start)
    status = capture["corner_deviation"]
    assert peaks["x"] <= status["corner_accel_x"]
    assert peaks["y"] <= status["corner_accel_y"]


def test_the_admitted_worst_is_reached_at_a_fast_shallow_corner():
    """The worst X burst is the density peak times the X velocity step at
    the cell's floor speed, plus at most the legs' X acceleration."""
    model, kx = _model()
    capture, peaks = _run(
        "[corner_deviation]\n", _fast_shallow_corner(MAX_ACCEL), START
    )
    admitted_x, _ = model.worst(MAX_ACCEL, MAX_VELOCITY, None)
    assert capture["corner_deviation"]["corner_accel_x"] == admitted_x
    turn = math.radians(ALPHA_DEG + TURN_DEG)
    up_x, um_x = math.cos(math.radians(ALPHA_DEG)), math.cos(turn)
    first, second = capture["moves"]
    corner_speed = second["start_v"]
    step = kx.density_peak * corner_speed * abs(um_x - up_x)
    legs = MAX_ACCEL * max(abs(up_x), abs(um_x))
    assert step - legs <= peaks["x"] <= admitted_x


def test_recommended_max_accel_keeps_the_worst_corner_within_the_motor():
    model, _ = _model()
    recommended = model.recommended_max_accel(MOTOR_ACCEL, MAX_VELOCITY, None)
    assert max(model.worst(recommended, MAX_VELOCITY, None)) <= MOTOR_ACCEL
    above = recommended * (1.0 + 10.0 * cd.RECOMMEND_TOLERANCE)
    assert max(model.worst(above, MAX_VELOCITY, None)) > MOTOR_ACCEL
    _, peaks = _run(
        "[corner_deviation]\n", _fast_shallow_corner(recommended), START
    )
    assert max(peaks.values()) <= MOTOR_ACCEL
    text, start = _case(WORST_CASE)
    _, peaks = _run(
        "[corner_deviation]\n",
        "SET_VELOCITY_LIMIT ACCEL=%r\n%s" % (recommended, text),
        start,
    )
    assert max(peaks.values()) <= MOTOR_ACCEL


class _Gcmd:
    error = RuntimeError

    def __init__(self, motor_accel):
        self.motor_accel = motor_accel
        self.responses = []

    def get_float(self, name, default, above):
        assert name == "MOTOR_ACCEL" and default is None and above == 0.0
        return self.motor_accel

    def respond_info(self, msg):
        self.responses.append(msg)


class _Toolhead:
    max_accel = MAX_ACCEL
    max_velocity = MAX_VELOCITY


def test_report_states_the_worst_corner_and_the_recommended_max_accel():
    model, _ = _model()
    kx, ky = (cd.kernel_from_spec(s) for s in OWNER_SPECS)
    limiter = object.__new__(cd.CornerDeviation)
    limiter.limit = cd.JunctionLimit(model.arrays, kx, ky, None)
    limiter.toolhead = _Toolhead()
    limiter.max_corner_accel = None
    limiter.corner_accel_key = limiter.corner_accel = None
    gcmd = _Gcmd(MOTOR_ACCEL)
    limiter.cmd_CORNER_DEVIATION_REPORT(gcmd)
    worst = model.worst(MAX_ACCEL, MAX_VELOCITY, None)
    recommended = model.recommended_max_accel(MOTOR_ACCEL, MAX_VELOCITY, None)
    assert gcmd.responses == [
        "worst corner accel X %.0f Y %.0f mm/s^2 at max_accel %.0f\n"
        "recommended_max_accel %.0f for MOTOR_ACCEL %.0f"
        % (worst + (MAX_ACCEL, recommended, MOTOR_ACCEL))
    ]
