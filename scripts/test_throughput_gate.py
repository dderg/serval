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
    ir: int = 1000,
    live: float = 1.0,
    stream: float = 10.0,
    vs_calibration: float = 1.0,
    calibration_cost: float = 0.5,
    spread_pct: float = 0.5,
    cpu: str = CPU,
):
    return gate.Measurement(
        ir=ir,
        stream_secs=stream,
        live_ratio=live,
        calibration_ratio=vs_calibration,
        calibration_cost=calibration_cost,
        calibration_spread_pct=spread_pct,
        live_wall_secs=20.0,
        reference_wall_secs=20.0,
        calibration_wall_secs=20.0,
        cpu=cpu,
    )


def model(
    ir_worst: float = 2.5,
    beta: float = 1.0,
    live_worst: float | None = None,
    alpha: float = 1.0,
) -> dict:
    """Predicts ir_worst at the default measurement's 100 Ir per stream
    second and live_worst (ir_worst by default) at the calibration's live
    cost; the Pi replays the calibration 2x as slowly as the default
    measurement's runner, the fit's runners 1.5-3x."""
    return {
        "ir": {
            "factor": beta,
            "intercept": math.log(ir_worst) + beta * math.log(100),
            "max_residual_pct": 5.0,
            "max_loo_residual_pct": 8.0,
        },
        "live": {
            "factor": alpha,
            "intercept": math.log(live_worst or ir_worst),
            "max_residual_pct": 6.0,
            "max_loo_residual_pct": 9.0,
            "pi_cost": 1.0,
            "speed_x": {"min": 1.5, "max": 3.0},
            "by_cpu": {
                CPU: {"max_residual_pct": 6.0, "max_loo_residual_pct": 9.0}
            },
        },
    }


def entry() -> dict:
    return {"ir": 1000, "stream_secs": 10.0}


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
    verdict = gate.judge("c", entry(), measurement(stream=7.6), model())
    assert verdict.predicted_x == pytest.approx((1.9, 2.5))
    assert "Ir predicts, 1.90x, is below the 2.0x floor" in verdict.problems[0]
    steeper = gate.judge("c", entry(), measurement(stream=9.5), model(beta=2.0))
    assert steeper.predicted_x[0] == pytest.approx(2.5 * (9.5 / 10.0) ** 2)


def test_the_live_prediction_divides_the_calibration_window_by_the_cost_over_it():
    verdict = gate.judge(
        "c", entry(), measurement(vs_calibration=1.01), model(2.5, 1.0, 2.01)
    )
    assert verdict.predicted_x[1] == pytest.approx(2.01 / 1.01)
    assert verdict.problems == (
        "the Pi 2-core worst window the live cost predicts, 1.99x, "
        "is below the 2.0x floor",
    )
    assert verdict.warnings == ()
    steeper = gate.judge(
        "c",
        entry(),
        measurement(vs_calibration=1.01),
        model(2.5, 1.0, 2.2, alpha=3.0),
    )
    assert steeper.predicted_x[1] == pytest.approx(2.2 / 1.01**3)


def test_the_live_prediction_ignores_the_reference_and_the_cpu_model():
    moved = gate.judge(
        "c",
        entry(),
        measurement(live=1.01, vs_calibration=0.8, cpu="Unseen CPU"),
        model(),
    )
    assert moved.predicted_x[1] == pytest.approx(2.5 / 0.8)
    assert moved.problems == ()


def test_calibration_replays_too_far_apart_fail():
    limit = gate.CALIBRATION_SPREAD_PCT
    assert (
        gate.judge(
            "c", entry(), measurement(spread_pct=limit * 0.9), model()
        ).problems
        == ()
    )
    noisy = gate.judge(
        "c", entry(), measurement(spread_pct=limit * 1.1), model()
    )
    assert len(noisy.problems) == 1
    assert "too noisy to normalise by" in noisy.problems[0]


def test_a_calibration_speed_past_the_fitted_envelope_and_its_margin_fails():
    margin = 1.0 + gate.CALIBRATION_SPEED_MARGIN_PCT / 100.0
    for inside in (1.5 / margin * 1.01, 3.0 * margin * 0.99):
        cost = 1.0 / inside
        assert (
            gate.judge(
                "c", entry(), measurement(calibration_cost=cost), model()
            ).problems
            == ()
        )
    for outside in (1.5 / margin * 0.99, 3.0 * margin * 1.01):
        verdict = gate.judge(
            "c", entry(), measurement(calibration_cost=1.0 / outside), model()
        )
        assert len(verdict.problems) == 1
        assert gate.RECHECK_COMMAND in verdict.problems[0]
        assert f"{outside:.2f}x as fast as the Pi" in verdict.problems[0]


def test_a_floor_miss_only_warns_while_the_floor_checks_are_off(monkeypatch):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge("c", entry(), measurement(), model(1.9))
    assert verdict.problems == ()
    assert len(verdict.warnings) == 2
    assert all(
        "below the 2.0x floor" in warning for warning in verdict.warnings
    )


def test_the_summary_names_an_unseen_cpu_model_without_failing():
    lines = gate.model_summary(model(), measurement(cpu="Unseen CPU"))
    assert "a CPU model the fit did not see" in lines[0]
    assert "2.00x the Pi's speed" in lines[1]
    seen = gate.model_summary(model(), measurement())
    assert "on this CPU model" in seen[0]


def test_a_side_is_summarised_from_its_fastest_runs():
    head = [(0.52, 10.4), (0.50, 10.0), (0.60, 12.0)]
    reference = [(0.49, 9.8), (0.51, 10.2)]
    calibration = [(0.40, 8.0), (0.42, 8.4), (0.80, 16.0)]
    m = gate.summarise(1000, 10.0, head, reference, calibration, CPU)
    assert m.live_ratio == pytest.approx(0.51 / 0.50)
    assert m.calibration_ratio == pytest.approx(0.51 / 0.41)
    assert m.calibration_cost == pytest.approx(0.41)
    assert m.calibration_spread_pct == pytest.approx(5.0)
    assert m.calibration_wall_secs == pytest.approx(8.2)


def pinned(ir_x: float, live_x: float) -> dict[str, gate.FloorShortfall]:
    return {"c": gate.FloorShortfall(issue=693, predicted_x=(ir_x, live_x))}


def test_a_pinned_floor_shortfall_passes_at_its_pinned_predictions():
    verdict = gate.judge(
        "c", entry(), measurement(), model(1.6, 1.0, 1.4), pinned(1.6, 1.4)
    )
    assert verdict.predicted_x == pytest.approx((1.6, 1.4))
    assert verdict.problems == ()


def test_each_prediction_answers_to_its_own_pin():
    verdict = gate.judge(
        "c", entry(), measurement(), model(1.6, 1.0, 1.4), pinned(1.4, 1.6)
    )
    assert len(verdict.problems) == 2
    assert "Ir predicts, 1.60x, shrank" in verdict.problems[0]
    assert "live cost predicts, 1.40x, grew" in verdict.problems[1]


def test_a_pinned_floor_shortfall_that_grows_fails_naming_its_issue():
    verdict = gate.judge(
        "c", entry(), measurement(stream=9.0), model(1.6), pinned(1.6, 1.6)
    )
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.44x, grew the floor "
        "shortfall pinned at 1.60x for #693 (-10.00 %)",
    )


def test_a_pinned_floor_shortfall_that_shrinks_fails_until_repinned():
    verdict = gate.judge(
        "c", entry(), measurement(stream=10.5), model(1.6), pinned(1.6, 1.6)
    )
    assert verdict.problems == (
        "the Pi 2-core worst window Ir predicts, 1.68x, shrank the floor "
        "shortfall pinned at 1.60x for #693 (+5.00 %): re-pin it",
    )


def test_a_pinned_floor_shortfall_that_vanishes_fails_until_unpinned():
    verdict = gate.judge(
        "c", entry(), measurement(stream=13.0), model(1.6), pinned(1.6, 1.6)
    )
    assert "unpin" in verdict.problems[0]
    assert "#693" in verdict.problems[0]


def test_a_pin_on_another_config_leaves_the_floor_in_force():
    verdict = gate.judge(
        "other", entry(), measurement(), model(1.6), pinned(1.6, 1.6)
    )
    assert len(verdict.problems) == 2
    assert all("below the 2.0x floor" in p for p in verdict.problems)


def test_a_pinned_shortfall_only_warns_while_the_floor_checks_are_off(
    monkeypatch,
):
    monkeypatch.setattr(gate, "PI_FLOOR_FAILS", False)
    verdict = gate.judge(
        "c", entry(), measurement(stream=9.0), model(1.6), pinned(1.6, 1.6)
    )
    assert verdict.problems == ()
    assert "grew the floor shortfall" in verdict.warnings[0]


def test_a_pin_at_or_above_the_floor_is_refused():
    with pytest.raises(ValueError):
        gate.FloorShortfall(issue=693, predicted_x=(1.6, 2.1))
    with pytest.raises(ValueError):
        gate.FloorShortfall(issue=693, predicted_x=(2.0, 1.6))


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
