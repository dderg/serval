import logging

import pytest

from klippy.mcu_cpu_headroom import (
    CpuHeadroom,
    CpuLoadWindow,
    ticks_per_step_event,
)

FREQ = 84_000_000.0
WINDOW = 21_000_000
PULSED = 476
BOTH_EDGE = 240


def capacity(*modes):
    return {
        "capacity_fraction": 0.8,
        "pulsed_ticks_per_step": PULSED,
        "both_edge_ticks_per_step": BOTH_EDGE,
        "motors": [{"name": "m%d" % i, "mode": m} for i, m in enumerate(modes)],
    }


PULSED_MCU = capacity("pulsed", "pulsed")


def load(idle, step, nonstep_timer, charged=0.0):
    return {
        "window_ticks": WINDOW,
        "idle_ticks": round(idle * WINDOW),
        "timer_irq_ticks": round((step + nonstep_timer) * WINDOW),
        "nonstep_timer_ticks": round(nonstep_timer * WINDOW),
        "step_events": round(charged * WINDOW / (PULSED / 2)),
    }


def headroom(guarded=PULSED_MCU):
    shutdowns = []
    monitor = CpuHeadroom("mcu", shutdowns.append)
    if guarded is not None:
        monitor.guard(guarded)
    return monitor, shutdowns


def events(caplog):
    return [
        r
        for r in caplog.records
        if getattr(r, "event", None) == "cpu_headroom_exceeded"
    ]


def test_the_shares_split_the_window():
    window = CpuLoadWindow(load(0.5, 0.3, 0.05, charged=0.4), PULSED / 2)
    assert window.idle_share == pytest.approx(0.5)
    assert window.step_share == pytest.approx(0.3)
    assert window.nonstep_share == pytest.approx(0.2)
    assert window.charged_step_share == pytest.approx(0.4, abs=1e-4)


def test_a_step_event_costs_half_a_pulsed_step_or_a_both_edge_step():
    assert ticks_per_step_event(capacity("pulsed")) == PULSED / 2
    assert ticks_per_step_event(capacity("both-edge")) == BOTH_EDGE


def test_mixed_modes_charge_the_cheaper_event():
    assert ticks_per_step_event(capacity("pulsed", "both-edge")) == min(
        PULSED / 2, BOTH_EDGE
    )


def test_an_empty_window_is_refused():
    with pytest.raises(ValueError, match="window"):
        CpuLoadWindow({**load(0.5, 0.3, 0.0), "window_ticks": 0}, 1.0)


def test_load_within_the_headroom_raises_nothing(caplog):
    monitor, shutdowns = headroom()
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(load(0.0, 0.82, 0.1, charged=0.8), 0.0, FREQ)
    assert events(caplog) == []
    assert shutdowns == []


def test_excess_non_step_load_while_steps_are_light_is_reported_not_fatal(
    caplog,
):
    monitor, shutdowns = headroom()
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(load(0.3, 0.4, 0.1, charged=0.5), 0.0, FREQ)
    (event,) = events(caplog)
    assert event.levelno == logging.WARNING
    assert event.nonstep_share == pytest.approx(0.3)
    assert shutdowns == []


def test_excess_non_step_load_with_steps_near_the_cap_shuts_down():
    monitor, shutdowns = headroom()
    monitor.observe(load(0.1, 0.6, 0.2, charged=0.75), 0.0, FREQ)
    (message,) = shutdowns
    assert "MCU 'mcu' non-step load took 30.0% of the CPU" in message
    assert (
        "more than the 20.0% the step capacity fraction 0.80 leaves" in message
    )
    assert "step capacity cap no longer holds" in message


def test_an_mcu_without_a_step_cap_is_only_measured():
    monitor, shutdowns = headroom(guarded=None)
    monitor.observe(load(0.0, 0.5, 0.4), 0.0, FREQ)
    assert shutdowns == []
    assert "mcu_nonstep_max=0.500" in monitor.take_stats()


def test_reports_are_rate_limited_and_count_the_windows(caplog):
    monitor, _ = headroom()
    excess = load(0.3, 0.4, 0.1, charged=0.5)
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(excess, 0.0, FREQ)
        monitor.observe(excess, 10.0, FREQ)
        monitor.observe(excess, 31.0, FREQ)
    first, second = events(caplog)
    assert (first.windows, second.windows) == (1, 2)


def test_stats_report_the_extremes_since_the_last_read():
    monitor, _ = headroom()
    monitor.observe(load(0.9, 0.05, 0.0), 0.0, FREQ)
    monitor.observe(load(0.5, 0.3, 0.0), 0.0, FREQ)
    assert monitor.take_stats() == "mcu_nonstep_max=0.200 mcu_idle_min=0.500"
    assert monitor.take_stats() == "mcu_nonstep_max=0.000 mcu_idle_min=1.000"
