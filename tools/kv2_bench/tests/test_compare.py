import json

import numpy as np
import pytest

from tools.kv2_bench import compare, program
from tools.kv2_bench.reference import Steps

FREQ = 84_000_000


def steps(clocks, signs=None):
    clocks = np.asarray(clocks, dtype=np.int64)
    if signs is None:
        signs = np.ones(len(clocks), np.int8)
    return Steps(clocks, np.asarray(signs, dtype=np.int8))


def motors(**given):
    empty = steps([])
    return {name: given.get(name, empty) for name in compare.CHANNELS}


def tap_line(name, first_clock, data):
    return json.dumps(
        {"tap": name, "params": {"first_clock": first_clock, "data": data}}
    )


def test_tap_reconstructs_queue_step_clocks(tmp_path):
    lines = [
        json.dumps({"id": 1, "result": {"header": []}}),
        tap_line("stepper_x", 1000, [[1000, 3, 10, 0, 0], [500, -2, 0, 0, 0]]),
        tap_line("stepper_x", 5000, [[100, 1, 0, 0, 0]]),
    ]
    (tmp_path / "tap.jsonl").write_text("\n".join(lines) + "\n")
    tap = compare.load_tap(tmp_path)
    assert tap["stepper_x"].clocks.tolist() == [
        1000,
        2010,
        3030,
        3530,
        4030,
        5000,
    ]
    assert tap["stepper_x"].signs.tolist() == [1, 1, 1, -1, -1, 1]
    assert len(tap["extruder"].clocks) == 0


def test_tap_overlapping_batches_fail(tmp_path):
    lines = [
        tap_line("stepper_x", 1000, [[1000, 2, 0, 0, 0]]),
        tap_line("stepper_x", 2000, [[100, 1, 0, 0, 0]]),
    ]
    (tmp_path / "tap.jsonl").write_text("\n".join(lines) + "\n")
    with pytest.raises(compare.CompareError, match="overlaps"):
        compare.load_tap(tmp_path)


def test_sequences_find_first_divergence():
    planned = motors(stepper_x=steps([1, 2, 3], [1, -1, 1]))
    executed = motors(stepper_x=steps([1, 2, 3], [1, 1, 1]))
    result = compare.sequences(planned, executed)
    assert not result["all_steps_paired"]
    assert result["per_motor"]["stepper_x"]["first_divergence_index"] == 1
    assert not result["net_steps_match"]
    missing = motors(stepper_x=steps([1, 2], [1, -1]))
    result = compare.sequences(planned, missing)
    assert result["per_motor"]["stepper_x"]["first_divergence_index"] == 2


def two_epochs():
    first = np.arange(0, 2000) * 2100
    second = first[-1] + int(0.5 * FREQ) + np.arange(0, 2000) * 2100
    return np.concatenate([first, second])


def test_timeline_splits_at_rests_and_fits_one_slope():
    ref = two_epochs()
    ppm = 30e-6
    cap = (ref * (1 + ppm)).astype(np.int64) + 123_456
    cap[2000:] += int(0.2 * FREQ)
    rng = np.random.default_rng(1)
    cap += rng.integers(0, 84, len(cap))
    stats, _, _ = compare.timeline(
        motors(stepper_x=steps(ref)),
        motors(stepper_x=steps(cap)),
        FREQ,
        None,
        False,
    )
    assert stats["epochs"] == 2
    assert stats["jumps_mid_motion"] == []
    assert stats["slope_ppm"] == pytest.approx(30.0, abs=1.0)
    assert stats["abs_max_us"] < 1.0


def test_timeline_flags_jump_mid_motion():
    ref = np.arange(0, 4000) * 2100
    cap = ref.copy()
    cap[2000:] += int(300e-6 * FREQ)
    stats, _, _ = compare.timeline(
        motors(stepper_x=steps(ref)),
        motors(stepper_x=steps(cap)),
        FREQ,
        1.0,
        False,
    )
    assert len(stats["jumps_mid_motion"]) == 1
    assert stats["jumps_mid_motion"][0]["jump_us"] == pytest.approx(
        300.0, abs=0.1
    )


def test_program_reads_start_and_repeats(tmp_path):
    source = tmp_path / "case.gcode"
    source.write_text(
        "; motion_eval start=1.5,2,3\nG90\nM104 S200\nG1 X5 E1 ; move\n"
    )
    loaded = program.load(source, repeat=2)
    assert loaded.start_xyz == (1.5, 2.0, 3.0)
    assert loaded.lines == (
        "SET_KINEMATIC_POSITION X=1.5 Y=2.0 Z=3.0",
        "G92 E0",
        "G90",
        "G1 X5 E1",
        "G90",
        "G1 X1.5 Y2.0 Z3.0 F6000",
        "G90",
        "G1 X5 E1",
        "M400",
    )
    source.write_text("G28\nM117 hi\n")
    with pytest.raises(program.ProgramError, match="M117"):
        program.load(source)
