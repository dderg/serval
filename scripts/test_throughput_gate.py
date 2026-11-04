from __future__ import annotations

from pathlib import Path

import pytest
import throughput_gate as gate

RUST_ROOT = Path("/work/serval/rust")


def cachegrind(records: str, summary: int) -> str:
    return (
        "# cachegrind out\ncmd: bench\nevents: Ir\n"
        f"{records}summary: {summary}\n"
    )


def test_only_instructions_in_repo_sources_count():
    out = cachegrind(
        "fl=/work/serval/rust/motion-pipeline/src/shaper.rs\nfn=shape\n10 100\n11 5\n"
        "fl=/rustc/abc/library/core/src/iter.rs\nfn=shape\n3 40\n"
        "fl=/home/u/.cargo/registry/src/index.crates.io-x/crossbeam/lib.rs\nfn=send\n1 7\n"
        "fl=./string/../sysdeps/x86_64/multiarch/memmove-vec-unaligned-erms.S\n"
        "fn=memcpy\n2 30\n"
        "fl=/work/serval/rust/target/release/build/x/out/gen.rs\nfn=gen\n1 9\n"
        "fl=/work/serval/rust/geometry/../geometry/src/lib.rs\nfn=fit\n0 11\n",
        202,
    )
    assert gate.instruction_counts(out, RUST_ROOT) == (116, 202)


def test_a_summary_that_disagrees_with_the_records_fails():
    out = cachegrind("fl=/work/serval/rust/a.rs\nfn=f\n1 10\n", 11)
    with pytest.raises(gate.GateError):
        gate.instruction_counts(out, RUST_ROOT)


def test_no_repo_instructions_at_all_fails():
    out = cachegrind("fl=???\nfn=f\n1 10\n", 10)
    with pytest.raises(gate.GateError):
        gate.instruction_counts(out, RUST_ROOT)


def test_stream_secs_comes_from_the_bench_summary_line():
    stdout = (
        "moves=10 segments=4 stream_secs=8.3 wall_secs=1.0 overall_x=8.30\n"
    )
    assert gate.stream_secs(stdout) == 8.3
    with pytest.raises(gate.GateError):
        gate.stream_secs("no summary")


def test_the_accept_reason_is_the_fenced_block_in_the_body():
    body = "Faster.\n\n```throughput-accept\n  fewer fits per window\n```\nmore"
    assert gate.accept_reason(body) == "fewer fits per window"
    assert gate.accept_reason("```throughput-accept\n\n```") == ""
    assert gate.accept_reason("no block") == ""
    assert gate.accept_reason(None) == ""


def entry(ir: int, k: float | None = None) -> dict:
    base = {"ir": ir, "stream_secs": 10.0, "reason": "r", "commit": "c"}
    if k is not None:
        base["pi_2core"] = {"k": k, "worst_x": [2.5], "commit": "c"}
    return base


def test_growth_or_shrinkage_past_the_threshold_fails_and_noise_passes():
    k = 2.5 * 1000 / 10.0
    within = gate.judge("c", entry(1000, k), gate.Measurement(1015, 0, 10.0))
    assert within.problems == ()
    grown = gate.judge("c", entry(1000, k), gate.Measurement(1030, 0, 10.0))
    assert "regression" in grown.problems[0]
    shrunk = gate.judge("c", entry(1000, k), gate.Measurement(970, 0, 10.0))
    assert "accept the improvement" in shrunk.problems[0]


def test_the_predicted_pi_window_scales_with_stream_time_per_instruction():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge("c", entry(1000, k), gate.Measurement(1000, 0, 7.6))
    assert verdict.predicted_x == pytest.approx(1.9)
    assert "below the 2.0x floor" in verdict.problems[0]


def test_an_uncalibrated_config_fails():
    verdict = gate.judge("c", entry(1000), gate.Measurement(1000, 0, 10.0))
    assert verdict.predicted_x is None
    assert "calibrat" in verdict.problems[0]


def pinned(predicted_x: float) -> dict[str, gate.FloorShortfall]:
    return {"c": gate.FloorShortfall(issue=693, predicted_x=predicted_x)}


def test_a_pinned_floor_shortfall_passes_at_its_pinned_prediction():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge(
        "c", entry(1000, k), gate.Measurement(1000, 0, 6.4), pinned(1.6)
    )
    assert verdict.predicted_x == pytest.approx(1.6)
    assert verdict.problems == ()


def test_a_pinned_floor_shortfall_that_grows_fails_naming_its_issue():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge(
        "c", entry(1000, k), gate.Measurement(1000, 0, 6.0), pinned(1.6)
    )
    assert "grew" in verdict.problems[0]
    assert "#693" in verdict.problems[0]


def test_a_pinned_floor_shortfall_that_shrinks_fails_until_repinned():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge(
        "c", entry(1000, k), gate.Measurement(1000, 0, 7.2), pinned(1.6)
    )
    assert "re-pin" in verdict.problems[0]


def test_a_pinned_floor_shortfall_that_vanishes_fails_until_unpinned():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge(
        "c", entry(1000, k), gate.Measurement(1000, 0, 8.4), pinned(1.6)
    )
    assert "unpin" in verdict.problems[0]
    assert "#693" in verdict.problems[0]


def test_a_pin_on_another_config_leaves_the_floor_in_force():
    k = 2.5 * 1000 / 10.0
    verdict = gate.judge(
        "other", entry(1000, k), gate.Measurement(1000, 0, 6.4), pinned(1.6)
    )
    assert "below the 2.0x floor" in verdict.problems[0]


def test_a_pin_above_the_floor_is_refused():
    with pytest.raises(ValueError):
        gate.FloorShortfall(issue=693, predicted_x=2.1)


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
