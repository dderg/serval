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


def test_the_live_ratio_compares_the_fastest_runs_of_each_side():
    assert gate.LIVE_FASTEST == 2
    assert gate.live_ratio(
        [0.30, 0.20, 0.90], [0.24, 0.16, 0.50]
    ) == pytest.approx(1.25)
    with pytest.raises(gate.GateError):
        gate.live_ratio([0.2], [0.2, 0.3])
    with pytest.raises(gate.GateError):
        gate.live_ratio([0.2, 0.3], [])


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
    assert (
        gate.judge("c", entry(), measurement(live=1 + limit * 0.9)).problems
        == ()
    )
    grown = gate.judge("c", entry(), measurement(live=1 + limit * 1.1))
    assert grown.problems[0].startswith(
        "live 2-core wall per stream second grew"
    )
    shrunk = gate.judge("c", entry(), measurement(live=1 - limit * 1.1))
    assert "accept the improvement" in shrunk.problems[0]


def test_the_ir_prediction_scales_with_stream_time_per_instruction(monkeypatch):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", True)
    verdict = gate.judge("c", entry(2.5), measurement(stream=7.6))
    assert verdict.predicted_x == pytest.approx((1.9, 2.5))
    assert "Ir predicts, 1.90x, is below the 2.0x floor" in verdict.problems[0]


def test_the_live_prediction_divides_the_reference_window_by_the_ratio(
    monkeypatch,
):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", True)
    verdict = gate.judge("c", entry(2.01), measurement(live=1.01))
    assert verdict.predicted_x[1] == pytest.approx(2.01 / 1.01)
    assert verdict.problems == (
        "the Pi 2-core worst window the live ratio predicts, 1.99x, "
        "is below the 2.0x floor",
    )
    assert verdict.warnings == ()


def test_a_floor_miss_only_warns_while_the_floor_checks_are_off(monkeypatch):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge("c", entry(1.9), measurement())
    assert verdict.problems == ()
    assert len(verdict.warnings) == 2
    assert all(
        "below the 2.0x floor" in warning for warning in verdict.warnings
    )


def test_a_calibration_off_the_reference_is_moved_onto_it_by_the_live_ratio():
    slower = measurement(live=1.1)
    pi = gate.pi_calibration(slower, [2.0, 2.2], "head", accepted=False)
    assert pi["worst_x_reference"] == pytest.approx(2.2)
    assert gate.predicted_worst_x({"pi_2core": pi}, slower)[1] == pytest.approx(
        2.0
    )
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


def pinned(predicted_x: float) -> dict[str, gate.FloorShortfall]:
    return {"c": gate.FloorShortfall(issue=693, predicted_x=predicted_x)}


@pytest.fixture
def floor_fails(monkeypatch):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", True)


def test_a_pinned_floor_shortfall_passes_at_its_pinned_prediction(floor_fails):
    verdict = gate.judge("c", entry(1.6), measurement(), pinned(1.6))
    assert verdict.predicted_x == pytest.approx((1.6, 1.6))
    assert verdict.problems == ()


def test_a_pinned_floor_shortfall_that_grows_fails_naming_its_issue(
    floor_fails,
):
    verdict = gate.judge("c", entry(1.6), measurement(stream=9.0), pinned(1.6))
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.44x, grew the floor "
        "shortfall pinned at 1.60x for #693 (-10.00 %)",
    )


def test_a_pinned_floor_shortfall_that_shrinks_fails_until_repinned(
    floor_fails,
):
    verdict = gate.judge("c", entry(1.6), measurement(stream=10.5), pinned(1.6))
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.68x, shrank the floor "
        "shortfall pinned at 1.60x for #693 (+5.00 %): re-pin it",
    )


def test_a_pinned_floor_shortfall_that_vanishes_fails_until_unpinned(
    floor_fails,
):
    verdict = gate.judge("c", entry(1.6), measurement(stream=13.0), pinned(1.6))
    assert "unpin" in verdict.problems[0]
    assert "#693" in verdict.problems[0]


def test_a_pin_on_another_config_leaves_the_floor_in_force(floor_fails):
    verdict = gate.judge("other", entry(1.6), measurement(), pinned(1.6))
    assert len(verdict.problems) == 2
    assert all("below the 2.0x floor" in p for p in verdict.problems)


def test_a_pinned_shortfall_only_warns_while_the_floor_checks_are_off(
    monkeypatch,
):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge("c", entry(1.6), measurement(stream=9.0), pinned(1.6))
    assert verdict.problems == ()
    assert "grew the floor shortfall" in verdict.warnings[0]


def test_a_pin_above_the_floor_is_refused():
    with pytest.raises(ValueError):
        gate.FloorShortfall(issue=693, predicted_x=2.1)


def test_a_baseline_without_its_slice_reference_or_configs_is_refused():
    configs = ("a.cfg",)
    current = {
        "slice_lines": gate.SLICE_LINES,
        "commit": "abc",
        "configs": {"a.cfg": {}},
    }
    assert gate.baseline_problem(current, configs) is None
    assert "lines" in gate.baseline_problem(
        {**current, "slice_lines": 1}, configs
    )
    assert "reference" in gate.baseline_problem(
        {**current, "commit": ""}, configs
    )
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


def test_the_lab_sync_rewrites_only_changed_files_and_drops_the_rest(tmp_path):
    source, dest = tmp_path / "source", tmp_path / "dest"
    (source / "crate" / "src").mkdir(parents=True)
    (source / "target" / "release").mkdir(parents=True)
    (source / "crate" / "src" / "lib.rs").write_text("fn a() {}")
    (source / "Cargo.toml").write_text("[workspace]")
    (source / "target" / "release" / "big").write_text("artifact")
    gate.sync_tree(source, dest)
    assert (dest / "crate" / "src" / "lib.rs").read_text() == "fn a() {}"
    assert not (dest / "target").exists()
    kept = dest / "Cargo.toml"
    kept_mtime = kept.stat().st_mtime_ns
    (dest / "stale.rs").write_text("old")
    (source / "crate" / "src" / "lib.rs").write_text("fn b() {}")
    gate.sync_tree(source, dest)
    assert (dest / "crate" / "src" / "lib.rs").read_text() == "fn b() {}"
    assert kept.stat().st_mtime_ns == kept_mtime
    assert not (dest / "stale.rs").exists()
