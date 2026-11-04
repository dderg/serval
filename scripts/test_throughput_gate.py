from __future__ import annotations

import datetime
import math

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


CPU = "Test CPU 9000"


def measurement(
    ir: int = 1000, live: float = 1.0, stream: float = 10.0, cpu: str = CPU
):
    return gate.Measurement(ir, stream, live, 20.0, 20.0, cpu)


def model(ir_worst: float = 2.5, beta: float = 1.0, alpha: float = 1.0) -> dict:
    """Predicts ir_worst at the default measurement's 100 Ir per stream second."""
    return {
        "ir": {
            "factor": beta,
            "intercept": math.log(ir_worst) + beta * math.log(100),
        },
        "live": {CPU: {"factor": alpha}},
    }


def entry(worst: float | None = 2.5) -> dict:
    base = {"ir": 1000, "stream_secs": 10.0}
    if worst is not None:
        base["pi_2core"] = gate.pi_calibration(
            measurement(), [worst, worst + 0.1], "c", accepted=True, factor=1.0
        )
    return base


def test_ir_growth_or_shrinkage_past_its_tolerance_fails_and_noise_passes():
    assert (
        gate.judge("c", entry(), measurement(ir=1015), model()).problems == ()
    )
    grown = gate.judge("c", entry(), measurement(ir=1030), model())
    assert grown.problems[0].startswith("Ir grew")
    shrunk = gate.judge("c", entry(), measurement(ir=970), model())
    assert "accept the improvement" in shrunk.problems[0]


def test_a_live_ratio_away_from_one_past_its_tolerance_fails():
    limit = gate.LIVE_TOLERANCE_PCT / 100.0
    assert (
        gate.judge(
            "c", entry(), measurement(live=1 + limit * 0.9), model()
        ).problems
        == ()
    )
    grown = gate.judge("c", entry(), measurement(live=1 + limit * 1.1), model())
    assert grown.problems[0].startswith(
        "live 2-core wall per stream second grew"
    )
    shrunk = gate.judge(
        "c", entry(), measurement(live=1 - limit * 1.1), model()
    )
    assert "accept the improvement" in shrunk.problems[0]


def test_the_floor_checks_fail_hard():
    assert gate.PI_FLOOR_FAILS


def test_the_ir_prediction_raises_ir_per_stream_second_to_its_factor():
    verdict = gate.judge("c", entry(2.5), measurement(stream=7.6), model())
    assert verdict.predicted_x == pytest.approx((1.9, 2.5))
    assert "Ir predicts, 1.90x, is below the 2.0x floor" in verdict.problems[0]
    steeper = gate.judge(
        "c", entry(2.5), measurement(stream=9.5), model(beta=2.0)
    )
    assert steeper.predicted_x[0] == pytest.approx(2.5 * (9.5 / 10.0) ** 2)


def test_the_live_prediction_divides_the_reference_window_by_the_ratio_to_its_factor():
    verdict = gate.judge("c", entry(2.01), measurement(live=1.01), model())
    assert verdict.predicted_x[1] == pytest.approx(2.01 / 1.01)
    assert verdict.problems == (
        "the Pi 2-core worst window the live ratio predicts, 1.99x, "
        "is below the 2.0x floor",
    )
    assert verdict.warnings == ()
    steeper = gate.judge(
        "c", entry(2.2), measurement(live=1.01), model(alpha=3.0)
    )
    assert steeper.predicted_x[1] == pytest.approx(2.2 / 1.01**3)


def test_a_cpu_without_a_live_model_fails_loudly():
    with pytest.raises(gate.GateError, match=gate.RECHECK_COMMAND):
        gate.judge("c", entry(), measurement(cpu="Other CPU"), model())


def test_a_floor_miss_only_warns_while_the_floor_checks_are_off(monkeypatch):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge("c", entry(1.9), measurement(), model(1.9))
    assert verdict.problems == ()
    assert len(verdict.warnings) == 2
    assert all(
        "below the 2.0x floor" in warning for warning in verdict.warnings
    )


def test_a_calibration_off_the_reference_is_moved_onto_it_by_the_live_ratio():
    slower = measurement(live=1.1)
    pi = gate.pi_calibration(
        slower, [2.0, 2.2], "head", accepted=False, factor=2.0
    )
    assert pi["worst_x_reference"] == pytest.approx(2.0 * 1.1**2)
    assert gate.predicted_worst_x({"pi_2core": pi}, model(alpha=2.0), slower)[
        1
    ] == pytest.approx(2.0)
    accepted = gate.pi_calibration(
        slower, [2.0, 2.2], "head", accepted=True, factor=2.0
    )
    assert accepted["worst_x_reference"] == 2.0


def test_an_accept_carries_the_pi_window_over_by_the_live_ratio_to_its_factor():
    pi = gate.pi_calibration(
        measurement(), [2.4], "old", accepted=True, factor=2.0
    )
    carried = gate.carried_pi(pi, measurement(live=1.2), 2.0)
    assert carried["worst_x_reference"] == pytest.approx(2.4 / 1.44)
    assert carried["worst_x"] == [2.4]
    assert carried["commit"] == "old"


def test_an_uncalibrated_config_fails():
    verdict = gate.judge("c", entry(None), measurement(), model())
    assert verdict.predicted_x is None
    assert "calibrat" in verdict.problems[0]


def pinned(predicted_x: float) -> dict[str, gate.FloorShortfall]:
    return {"c": gate.FloorShortfall(issue=693, predicted_x=predicted_x)}


def test_a_pinned_floor_shortfall_passes_at_its_pinned_prediction():
    verdict = gate.judge("c", entry(1.6), measurement(), model(1.6), pinned(1.6))
    assert verdict.predicted_x == pytest.approx((1.6, 1.6))
    assert verdict.problems == ()


def test_a_pinned_floor_shortfall_that_grows_fails_naming_its_issue():
    verdict = gate.judge(
        "c", entry(1.6), measurement(stream=9.0), model(1.6), pinned(1.6)
    )
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.44x, grew the floor "
        "shortfall pinned at 1.60x for #693 (-10.00 %)",
    )


def test_a_pinned_floor_shortfall_that_shrinks_fails_until_repinned():
    verdict = gate.judge(
        "c", entry(1.6), measurement(stream=10.5), model(1.6), pinned(1.6)
    )
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.68x, shrank the floor "
        "shortfall pinned at 1.60x for #693 (+5.00 %): re-pin it",
    )


def test_a_pinned_floor_shortfall_that_vanishes_fails_until_unpinned():
    verdict = gate.judge(
        "c", entry(1.6), measurement(stream=13.0), model(1.6), pinned(1.6)
    )
    assert "unpin" in verdict.problems[0]
    assert "#693" in verdict.problems[0]


def test_a_pin_on_another_config_leaves_the_floor_in_force():
    verdict = gate.judge(
        "other", entry(1.6), measurement(), model(1.6), pinned(1.6)
    )
    assert len(verdict.problems) == 2
    assert all("below the 2.0x floor" in p for p in verdict.problems)


def test_a_pinned_shortfall_only_warns_while_the_floor_checks_are_off(
    monkeypatch,
):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge(
        "c", entry(1.6), measurement(stream=9.0), model(1.6), pinned(1.6)
    )
    assert verdict.problems == ()
    assert "grew the floor shortfall" in verdict.warnings[0]


def test_a_pin_above_the_floor_is_refused():
    with pytest.raises(ValueError):
        gate.FloorShortfall(issue=693, predicted_x=2.1)


def test_a_missing_or_stale_pi_calibration_fails_with_the_recheck_command():
    measured = datetime.datetime(2026, 11, 4, 12, tzinfo=datetime.timezone.utc)
    baseline = {
        "pi_calibration": {
            "measured_at": measured.isoformat(),
            "commit": "abc",
            "configs": {"a.cfg": model()},
        }
    }
    bound = datetime.timedelta(days=gate.PI_CALIBRATION_STALE_DAYS)
    fresh = measured + bound
    assert gate.calibration_problem(baseline, ("a.cfg",), fresh) is None
    stale = gate.calibration_problem(
        baseline, ("a.cfg",), fresh + datetime.timedelta(hours=1)
    )
    assert f"{gate.PI_CALIBRATION_STALE_DAYS}-day bound" in stale
    assert gate.RECHECK_COMMAND in stale
    missing = gate.calibration_problem(baseline, ("b.cfg",), fresh)
    assert "b.cfg" in missing
    assert gate.RECHECK_COMMAND in gate.calibration_problem(
        {}, ("a.cfg",), fresh
    )


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
