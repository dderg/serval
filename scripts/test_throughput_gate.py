from __future__ import annotations

import pytest
import throughput_gate as gate


def cachegrind(records: str, summary: int) -> str:
    return (
        "# cachegrind out\ncmd: bench\nevents: Ir\n"
        f"{records}summary: {summary}\n"
    )


def test_every_user_space_instruction_counts():
    out = cachegrind(
        "fl=/work/serval/rust/motion-pipeline/src/shaper.rs\nfn=shape\n10 100\n11 5\n"
        "fl=/rustc/abc/library/core/src/iter.rs\nfn=shape\n3 40\n"
        "fl=/home/u/.cargo/registry/src/index.crates.io-x/crossbeam/lib.rs\nfn=send\n1 7\n"
        "fl=./string/../sysdeps/x86_64/multiarch/memmove-vec-unaligned-erms.S\n"
        "fn=memcpy\n2 30\n"
        "fl=???\nfn=__fma_fma3\n0 9\n",
        191,
    )
    assert gate.instruction_count(out) == 191


def test_a_summary_that_disagrees_with_the_records_fails():
    out = cachegrind("fl=/work/serval/rust/a.rs\nfn=f\n1 10\n", 11)
    with pytest.raises(gate.GateError):
        gate.instruction_count(out)


def test_a_missing_summary_fails():
    with pytest.raises(gate.GateError):
        gate.instruction_count("fl=a.rs\nfn=f\n1 10\n")


def test_stream_secs_comes_from_the_bench_summary_line():
    stdout = (
        "moves=10 segments=4 stream_secs=8.3 wall_secs=1.0 overall_x=8.30\n"
    )
    assert gate.stream_secs(stdout) == 8.3
    with pytest.raises(gate.GateError):
        gate.stream_secs("no summary")


def test_the_live_ratio_compares_the_median_run_of_each_side():
    assert gate.live_ratio([0.30, 0.25, 0.90], [0.24, 0.10, 0.50]) == 1.25
    with pytest.raises(gate.GateError):
        gate.live_ratio([], [0.2])
    with pytest.raises(gate.GateError):
        gate.live_ratio([0.2], [])


def test_the_accept_reason_is_the_fenced_block_in_the_body():
    body = "Faster.\n\n```throughput-accept\n  fewer fits per window\n```\nmore"
    assert gate.accept_reason(body) == "fewer fits per window"
    assert gate.accept_reason("```throughput-accept\n\n```") == ""
    assert gate.accept_reason("no block") == ""
    assert gate.accept_reason(None) == ""


def measurement(ir: int = 1000, live: float = 1.0, stream: float = 10.0):
    return gate.Measurement(ir, stream, live, 20.0, 20.0)


def entry(worst: float | None = 2.5) -> dict:
    base = {"ir": 1000, "stream_secs": 10.0}
    if worst is not None:
        base["pi_2core"] = gate.pi_calibration(
            measurement(), [worst, worst + 0.1], "c", accepted=True
        )
    return base


def test_ir_growth_or_shrinkage_past_its_tolerance_fails_and_noise_passes():
    assert gate.judge("c", entry(), measurement(ir=1015)).problems == ()
    grown = gate.judge("c", entry(), measurement(ir=1030))
    assert grown.problems[0].startswith("Ir grew")
    shrunk = gate.judge("c", entry(), measurement(ir=970))
    assert "accept the improvement" in shrunk.problems[0]


def test_a_live_ratio_away_from_one_past_its_tolerance_fails():
    limit = gate.LIVE_TOLERANCE_PCT / 100.0
    assert gate.judge("c", entry(), measurement(live=1 + limit * 0.9)).problems == ()
    grown = gate.judge("c", entry(), measurement(live=1 + limit * 1.1))
    assert grown.problems[0].startswith("live 2-core wall per stream second grew")
    shrunk = gate.judge("c", entry(), measurement(live=1 - limit * 1.1))
    assert "accept the improvement" in shrunk.problems[0]


def test_the_ir_prediction_scales_with_stream_time_per_instruction():
    verdict = gate.judge("c", entry(2.5), measurement(stream=7.6))
    assert verdict.predicted_x == pytest.approx((1.9, 2.5))
    assert "Ir predicts, 1.90x, is below the 2.0x floor" in verdict.problems[0]


def test_the_live_prediction_divides_the_reference_window_by_the_ratio():
    verdict = gate.judge("c", entry(2.01), measurement(live=1.01))
    assert verdict.predicted_x[1] == pytest.approx(2.01 / 1.01)
    assert verdict.problems == (
        "the Pi 2-core worst window the live ratio predicts, 1.99x, "
        "is below the 2.0x floor",
    )


def test_a_calibration_off_the_reference_is_moved_onto_it_by_the_live_ratio():
    slower = measurement(live=1.1)
    pi = gate.pi_calibration(slower, [2.0, 2.2], "head", accepted=False)
    assert pi["worst_x_reference"] == pytest.approx(2.2)
    assert gate.predicted_worst_x({"pi_2core": pi}, slower)[1] == pytest.approx(2.0)
    accepted = gate.pi_calibration(slower, [2.0, 2.2], "head", accepted=True)
    assert accepted["worst_x_reference"] == 2.0


def test_an_accept_carries_the_pi_window_over_by_the_live_ratio():
    pi = gate.pi_calibration(measurement(), [2.4], "old", accepted=True)
    carried = gate.carried_pi(pi, measurement(live=1.2))
    assert carried["worst_x_reference"] == pytest.approx(2.0)
    assert carried["worst_x"] == [2.4]
    assert carried["commit"] == "old"


def test_an_uncalibrated_config_fails():
    verdict = gate.judge("c", entry(None), measurement())
    assert verdict.predicted_x is None
    assert "calibrat" in verdict.problems[0]


def test_a_baseline_without_its_slice_reference_or_configs_is_refused():
    configs = ("a.cfg",)
    current = {"slice_lines": gate.SLICE_LINES, "commit": "abc", "configs": {"a.cfg": {}}}
    assert gate.baseline_problem(current, configs) is None
    assert "lines" in gate.baseline_problem({**current, "slice_lines": 1}, configs)
    assert "reference" in gate.baseline_problem({**current, "commit": ""}, configs)
    assert "no baseline" in gate.baseline_problem(current, ("b.cfg",))


def test_calibrations_name_a_gated_config_by_path_or_stem():
    configs = (
        "tools/bench/configs/a.cfg",
        "tools/bench/configs/profiles/b.cfg",
    )
    assert gate.parse_calibrations(
        ["a=2.1,2.2", "tools/bench/configs/profiles/b.cfg=3"], configs
    ) == {configs[0]: [2.1, 2.2], configs[1]: [3.0]}
    for bad in (["c=2"], ["a"], ["a="]):
        with pytest.raises(gate.GateError):
            gate.parse_calibrations(bad, configs)
