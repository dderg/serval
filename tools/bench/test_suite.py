from __future__ import annotations

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
