from __future__ import annotations

import json
import math

import pytest
import throughput_gate as gate
import throughput_recheck as recheck


def test_a_fit_through_the_anchor_recovers_the_factor_and_its_window():
    xs = {"a": -0.1, "b": 0.05, "c": 0.1, "d": 0.2}
    ys = {k: math.log(2.0) - 3.0 * x for k, x in xs.items()}
    fitted = recheck.fit(xs, ys, anchor="b")
    assert fitted["factor"] == pytest.approx(3.0)
    assert fitted["intercept"] == pytest.approx(math.log(2.0))
    assert set(fitted["residual_pct"]) == {"a", "c", "d"}
    assert fitted["max_residual_pct"] == pytest.approx(0.0, abs=1e-9)
    assert fitted["max_loo_residual_pct"] == pytest.approx(0.0, abs=1e-9)


def test_residuals_are_the_gates_error_predicting_each_point_from_the_anchor():
    xs = {"ref": 0.0, "b": 1.0, "c": 2.0}
    ys = {"ref": 0.0, "b": -1.0, "c": -1.0}
    fitted = recheck.fit(xs, ys, anchor="ref")
    assert fitted["factor"] == pytest.approx(0.6)
    assert fitted["residual_pct"]["b"] == pytest.approx(
        100.0 * (math.exp(-0.6 + 1.0) - 1.0), abs=0.01
    )
    assert fitted["loo_residual_pct"]["b"] == pytest.approx(
        100.0 * (math.exp(-0.5 + 1.0) - 1.0), abs=0.01
    )
    assert fitted["loo_residual_pct"]["c"] == pytest.approx(
        100.0 * (math.exp(-2.0 + 1.0) - 1.0), abs=0.01
    )


def test_a_fit_wants_two_points_besides_the_anchor():
    with pytest.raises(gate.GateError):
        recheck.fit(
            {"ref": 0.0, "b": 1.0}, {"ref": 0.0, "b": 1.0}, anchor="ref"
        )


CAL = recheck.CALIBRATION_LABEL


def rows(job: str, cpu: str, costs: dict[str, list[float]]) -> list[dict]:
    return [
        {
            "job": job,
            "cpu": cpu,
            "label": label,
            "config": "c.cfg",
            "cost": cost,
        }
        for label, samples in costs.items()
        for cost in samples
    ]


def test_live_costs_divide_by_the_jobs_calibration_and_average_one_cpu():
    measured = [
        *rows("1/x", "Slow", {CAL: [2.0, 2.0, 9.0], "b": [4.0, 4.0]}),
        *rows("1/y", "Slow", {CAL: [1.0, 1.1], "b": [2.0, 2.2]}),
        *rows("2/x", "Fast", {CAL: [1.0, 1.0], "b": [1.0, 1.0]}),
    ]
    live = recheck.live_costs(measured, [CAL, "b"], ["c.cfg"])["c.cfg"]
    assert live["cpus"]["Slow"]["jobs"] == 2
    assert live["cpus"]["Slow"]["x"]["b"] == pytest.approx(math.log(2.0))
    assert live["cpus"]["Slow"]["x"][CAL] == 0.0
    assert live["cpus"]["Fast"]["x"] == {CAL: 0.0, "b": 0.0}
    assert sorted(live["calibration_costs"]) == pytest.approx([1.0, 1.05, 2.0])
    assert max(live["spreads_pct"]) == pytest.approx(10.0)


def test_a_job_counts_for_the_variants_it_ran_beside_the_calibration():
    measured = [
        *rows("1/x", "Slow", {CAL: [2.0, 2.0], "b": [4.0], "c": [3.0, 3.0]}),
        *rows("1/y", "Slow", {CAL: [1.0, 1.0], "b": [3.0, 3.0]}),
        *rows("1/z", "Slow", {CAL: [1.0], "b": [3.0, 3.0]}),
    ]
    live = recheck.live_costs(measured, [CAL, "b", "c"], ["c.cfg"])["c.cfg"]
    assert live["cpus"]["Slow"]["jobs"] == 2
    assert live["cpus"]["Slow"]["x"]["b"] == pytest.approx(math.log(3.0))
    assert live["cpus"]["Slow"]["x"]["c"] == pytest.approx(math.log(1.5))


def test_one_live_factor_fits_every_cpu_model():
    ys = {
        CAL: math.log(2.0),
        "a": math.log(2.0) - 0.2,
        "b": math.log(2.0) + 0.1,
    }
    xs = {
        "Slow": {CAL: 0.0, "a": 0.1, "b": -0.05},
        "Fast": {CAL: 0.0, "a": 0.2, "b": -0.2},
    }
    fitted = recheck.pooled_fit(xs, ys, {"Slow": 3, "Fast": 1})
    factor = 0.085 / 0.0925
    without_slow = 0.06 / 0.08
    without_a = 0.025 / 0.0425
    assert fitted["factor"] == pytest.approx(factor)
    assert fitted["intercept"] == pytest.approx(math.log(2.0))
    slow = fitted["by_cpu"]["Slow"]
    assert slow["jobs"] == 3
    for name, slope in (
        ("residual_pct", factor),
        ("cpu_out_residual_pct", without_slow),
        ("loo_residual_pct", without_a),
    ):
        assert slow[name]["a"] == pytest.approx(
            100.0 * (math.exp(-slope * 0.1 + 0.2) - 1.0), abs=0.01
        )
    assert fitted["max_cpu_out_residual_pct"] == max(
        cpu["max_cpu_out_residual_pct"] for cpu in fitted["by_cpu"].values()
    )


def test_the_live_fit_wants_two_cpu_models_and_two_variants():
    ys = {CAL: 0.0, "a": -0.1, "b": 0.1}
    with pytest.raises(gate.GateError, match="CPU models"):
        recheck.pooled_fit({"Only": {"a": 0.1, "b": -0.1}}, ys, {"Only": 1})
    with pytest.raises(gate.GateError, match="variants"):
        recheck.pooled_fit(
            {"A": {"a": 0.1}, "B": {"a": 0.2}}, ys, {"A": 1, "B": 1}
        )


def test_the_probe_budget_is_each_ci_jobs_timeout():
    workflow = (
        "jobs:\n"
        "  rust-test:\n"
        "    timeout-minutes: 20\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh rust-test\n"
        "  no-timeout:\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh other\n"
        "  rust-fmt:\n"
        "    timeout-minutes: 5\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh rust-fmt\n"
    )
    assert recheck.job_timeouts(workflow) == {"rust-test": 20, "rust-fmt": 5}


def test_probe_rows_come_from_the_run_log_keyed_by_run_job_and_variant():
    row = {
        "cpu": "X",
        "label": "a",
        "config": "c.cfg",
        "cost": 1.5,
        "job": "rust-test",
    }
    log = (
        "rust-test\tProbe\t2026-11-04T10:00:00Z noise\n"
        f"rust-test\tProbe\t2026-11-04T10:00:01Z RECHECK {json.dumps(row)}\n"
    )
    assert recheck.probe_rows(log, 7, {"a": "sha-a"}) == [
        {**row, "job": "7/rust-test", "key": "sha-a"}
    ]


def test_collected_rows_follow_their_variant_key_and_drop_unlisted_ones(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(recheck, "WORK", tmp_path)
    recheck.save(
        "collect",
        {
            "rows": [
                {"job": "1/x", "label": "reference", "key": "sha-cal"},
                {"job": "1/x", "label": "old", "key": "sha-old"},
                {"job": "1/x", "label": "a", "key": "sha-a"},
            ]
        },
    )
    assert recheck.collected_rows({"a": "sha-a", CAL: "sha-cal"}) == [
        {"job": "1/x", "label": CAL, "key": "sha-cal"},
        {"job": "1/x", "label": "a", "key": "sha-a"},
    ]


def test_variants_add_the_calibration_commit_and_refuse_clashing_labels():
    baseline = {
        "pi_calibration": {"variants": [{"label": "a", "commit": "x"}]},
    }
    assert recheck.variants(baseline, "cal") == [
        {"label": "a", "commit": "x"},
        {"label": CAL, "commit": "cal"},
    ]
    clash = {"pi_calibration": {"variants": [{"label": CAL, "commit": "x"}]}}
    with pytest.raises(gate.GateError):
        recheck.variants(clash, "cal")
    with pytest.raises(gate.GateError):
        recheck.variants({}, "cal")
    itself = {"pi_calibration": {"variants": [{"label": "a", "commit": "cal"}]}}
    with pytest.raises(gate.GateError, match="calibration commit"):
        recheck.variants(itself, "cal")
    patched = {
        "pi_calibration": {
            "variants": [{"label": "a", "commit": "cal", "patch": "p.patch"}]
        }
    }
    assert recheck.variants(patched, "cal")[0]["label"] == "a"


def test_the_calibration_commit_is_the_reference_unless_re_pinned(monkeypatch):
    resolved = []

    def rev_parse(command: list[str], cwd=None) -> str:
        resolved.append(command[-1])
        return "sha\n"

    monkeypatch.setattr(gate, "run", rev_parse)
    assert recheck.calibration_commit({"commit": "ref"}, None) == "sha"
    assert recheck.calibration_commit({"commit": "ref"}, "pin") == "sha"
    assert resolved == ["ref^{commit}", "pin^{commit}"]
    with pytest.raises(gate.GateError, match="--calibration-commit"):
        recheck.calibration_commit({}, None)


def test_the_pi_wall_comes_from_the_stamps_around_the_replay():
    out = "pi_started_ns=1000000000\nmoves=1\npi_ended_ns=3500000000\n"
    assert recheck.pi_wall_secs(out) == pytest.approx(2.5)
    with pytest.raises(gate.GateError):
        recheck.pi_wall_secs("pi_started_ns=1\n")


def test_the_ci_probe_hook_replaces_the_rust_line_once():
    assert recheck.CI_PROBE_HOOK.startswith('RUST="$ROOT/rust"\n')
    assert '--job "${1:-}"' in recheck.CI_PROBE_HOOK
