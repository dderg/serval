from __future__ import annotations

import contextlib
import subprocess
import sys
from types import SimpleNamespace

import pytest
import suite


def _status(pulsed_ticks, both_edge_ticks=160, simulated=False):
    return {
        "step_benchmark": {
            "pulsed_ticks_per_step": pulsed_ticks,
            "both_edge_ticks_per_step": both_edge_ticks,
            "simulated": simulated,
        },
        "mcu_constants": {"CLOCK_FREQ": 84_000_000},
    }


def test_the_f401_benchmark_lands_inside_the_ladder_band():
    verdict = suite.step_benchmark_verdict(_status(301))
    assert verdict["pulsed_capacity_steps_per_s"] == pytest.approx(
        84_000_000 / 301
    )
    assert verdict["ratio_to_ladder_ceiling"] == pytest.approx(
        84_000_000 / 301 / suite.LADDER_CEILING_STEPS_PER_S
    )
    assert verdict["within_band"]


@pytest.mark.parametrize("pulsed_ticks", [200, 420])
def test_a_benchmark_far_from_the_ladder_ceiling_is_out_of_band(pulsed_ticks):
    assert not suite.step_benchmark_verdict(_status(pulsed_ticks))[
        "within_band"
    ]


def test_a_simulated_benchmark_on_the_bench_is_fatal():
    with pytest.raises(suite.SuiteError, match="simulated"):
        suite.step_benchmark_verdict(_status(0, 0, simulated=True))


def test_a_missing_benchmark_is_fatal():
    with pytest.raises(suite.SuiteError, match="step_benchmark"):
        suite.step_benchmark_verdict({"mcu_constants": {"CLOCK_FREQ": 1}})


def _completed(returncode, stdout="", stderr=""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def test_the_engine_builds_with_make_in_the_suites_own_tree(monkeypatch):
    calls = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: calls.append((cmd, kw)) or _completed(0),
    )
    suite.build_local_engine()
    [(cmd, kw)] = calls
    assert cmd == ["make", "-f", "Makefile.rust", "motion-engine-fast"]
    assert kw["cwd"] == suite.REPO


def test_a_failed_engine_build_is_fatal_with_its_output(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda cmd, **kw: _completed(2, "cargo build", "error[E0425]: nope"),
    )
    with pytest.raises(
        suite.SuiteError, match=r"(?s)exited 2.*cargo build.*E0425\]: nope"
    ):
        suite.build_local_engine()


def test_the_suite_builds_the_engine_once_before_any_row(monkeypatch, tmp_path):
    order = []
    monkeypatch.setattr(suite, "SUITES_DIR", tmp_path)
    monkeypatch.setattr(sys, "argv", ["suite.py"])
    monkeypatch.setattr(
        suite, "build_local_engine", lambda: order.append("build")
    )
    monkeypatch.setattr(
        suite.lock, "bench_lock", lambda *a, **k: contextlib.nullcontext()
    )
    monkeypatch.setattr(suite.lock, "export_holder_token", lambda: None)
    monkeypatch.setattr(
        suite,
        "python",
        lambda script, *a, **k: order.append(script) or _completed(0),
    )
    monkeypatch.setattr(
        suite,
        "query_step_benchmark",
        lambda: suite.step_benchmark_verdict(_status(301)),
    )

    def run_row(row, suite_dir, lock_timeout_s):
        order.append(row.name)
        return {"name": row.name, "passed": True}

    monkeypatch.setattr(suite, "run_row", run_row)
    monkeypatch.setattr(suite, "print_table", lambda rows: None)
    with pytest.raises(SystemExit):
        suite.main()
    assert order[0] == "build"
    assert order.count("build") == 1
    assert order[1:] == ["deploy.py", *(row.name for row in suite.MATRIX)]
