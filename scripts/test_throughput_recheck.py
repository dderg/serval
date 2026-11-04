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


def test_live_costs_centre_each_job_and_average_jobs_of_one_cpu():
    measured = [
        *rows("1/x", "Slow", {"a": [2.0, 2.0, 9.0], "b": [4.0, 4.0]}),
        *rows("1/y", "Slow", {"a": [1.0, 1.0], "b": [2.0, 2.0]}),
        *rows("2/x", "Fast", {"a": [1.0, 1.0], "b": [1.0, 1.0]}),
    ]
    live = recheck.live_log_costs(measured, ["a", "b"], ["c.cfg"])
    assert live["c.cfg"]["Slow"]["jobs"] == 2
    assert live["c.cfg"]["Slow"]["x"]["b"] - live["c.cfg"]["Slow"]["x"][
        "a"
    ] == (pytest.approx(math.log(2.0)))
    assert live["c.cfg"]["Fast"]["x"] == {"a": 0.0, "b": 0.0}


def test_a_job_missing_runs_of_a_variant_is_left_out():
    measured = [
        *rows("1/x", "Slow", {"a": [2.0, 2.0], "b": [4.0]}),
        *rows("1/y", "Slow", {"a": [1.0, 1.0], "b": [3.0, 3.0]}),
    ]
    live = recheck.live_log_costs(measured, ["a", "b"], ["c.cfg"])
    assert live["c.cfg"]["Slow"]["jobs"] == 1


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


def test_probe_rows_come_from_the_run_log_keyed_by_run_and_job():
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
    assert recheck.probe_rows(log, 7) == [{**row, "job": "7/rust-test"}]


def test_variants_add_the_reference_and_refuse_clashing_labels():
    baseline = {
        "commit": "ref",
        "pi_calibration": {"variants": [{"label": "a", "commit": "x"}]},
    }
    assert recheck.variants(baseline) == [
        {"label": "a", "commit": "x"},
        {"label": recheck.REFERENCE_LABEL, "commit": "ref"},
    ]
    clash = {
        "commit": "ref",
        "pi_calibration": {
            "variants": [{"label": recheck.REFERENCE_LABEL, "commit": "x"}]
        },
    }
    with pytest.raises(gate.GateError):
        recheck.variants(clash)
    with pytest.raises(gate.GateError):
        recheck.variants({"commit": "ref"})


def test_the_ci_probe_hook_replaces_the_rust_line_once():
    assert recheck.CI_PROBE_HOOK.startswith('RUST="$ROOT/rust"\n')
    assert '--job "${1:-}"' in recheck.CI_PROBE_HOOK
