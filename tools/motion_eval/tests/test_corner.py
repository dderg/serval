from __future__ import annotations

import math
import pathlib

import pytest

from tools.motion_eval import evaluate

DATA = pathlib.Path(__file__).parent / "data"
CONFIG = DATA / "printer.cfg"
START = (100.0, 100.0, 10.0)
SHAPER_FREQS = "SHAPER_FREQ_X=50 SHAPER_FREQ_Y=40"
FLOAT_EPS = 1e-9


def gcode_targets(gcode, start):
    pos = list(start)
    targets = []
    for line in gcode.splitlines():
        words = line.split(";")[0].split()
        if not words or words[0] != "G1":
            continue
        params = {w[0]: float(w[1:]) for w in words[1:]}
        pos = [params.get(a, p) for a, p in zip("XYZ", pos)]
        targets.append((tuple(pos), params["F"] / 60.0))
    return targets


def rest_to_rest_time(length, v_max, accel):
    ramp_length = v_max**2 / accel
    if length >= ramp_length:
        return length / v_max + v_max / accel
    return 2.0 * math.sqrt(length / accel)


@pytest.fixture(scope="module", params=["mzv", "smooth_mzv"])
def case(request):
    gcode = (DATA / "corner.gcode").read_text()
    shaped = "SET_INPUT_SHAPER SHAPER_TYPE=%s %s\n%s" % (
        request.param,
        SHAPER_FREQS,
        gcode,
    )
    result = evaluate.evaluate(CONFIG, shaped, START, sample_dt=None)
    return result, gcode_targets(gcode, START)


def test_final_position_matches_command(case):
    result, targets = case
    commanded = result["commanded"]["final_position"]
    assert commanded == pytest.approx(targets[-1][0], abs=FLOAT_EPS)
    assert result["executed"]["final_position_model"] == pytest.approx(
        commanded, abs=FLOAT_EPS
    )
    half_step = 0.5 * max(s["step_dist"] for s in result["steppers"].values())
    assert result["executed"]["final_position_steps"] == pytest.approx(
        commanded, abs=half_step
    )


def test_print_time_not_below_kinematic_bound(case):
    result, targets = case
    settings = result["settings"]
    prev = START
    length = 0.0
    v_max = settings["max_velocity"]
    for pos, feed in targets:
        length += math.dist(prev, pos)
        v_max = min(v_max, feed)
        prev = pos
    bound = rest_to_rest_time(length, v_max, settings["max_accel"])
    assert result["print_time_s"] >= bound - FLOAT_EPS


def test_executed_accel_bounded_by_configured_accel(case):
    result, _ = case
    max_accel = result["settings"]["max_accel"]
    for axis, cmd in result["commanded"]["per_axis"].items():
        executed = result["executed"]["per_axis"][axis]
        kernel = result["shaper"][axis]["model"]
        assert cmd["peak_accel_mm_s2"] <= max_accel * (1 + FLOAT_EPS)
        accel_bound = kernel["l1_norm"] * cmd["peak_accel_mm_s2"]
        jumps = cmd["total_velocity_jump_mm_s"]
        if kernel["kind"] == "smoother":
            accel_bound += kernel["peak_density_per_s"] * jumps
            assert executed["max_velocity_jump_mm_s"] == 0.0
        else:
            assert executed["max_velocity_jump_mm_s"] <= (
                kernel["l1_norm"] * jumps * (1 + FLOAT_EPS)
            )
        assert executed["peak_accel_mm_s2"] <= accel_bound * (1 + FLOAT_EPS)


def test_steps_follow_shaped_model(case):
    result, _ = case
    for name, stepper in result["steppers"].items():
        check = stepper["model_vs_steps"]
        if check["steps"]:
            assert check["max_error_mm"] <= check["max_allowed_mm"], name


def test_shaper_rounds_the_corner(case):
    result, _ = case
    assert result["executed"]["xy_path_deviation_model"]["max_mm"] > 0.0
