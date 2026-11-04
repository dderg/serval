import logging

import pytest

from klippy.mcu_cpu_headroom import CpuHeadroom, CpuLoadWindow

FREQ = 84_000_000.0
WINDOW = 21_000_000


def load(idle, step, callback):
    return {
        "window_ticks": WINDOW,
        "idle_ticks": round(idle * WINDOW),
        "timer_irq_ticks": round((step + callback) * WINDOW),
        "callback_ticks": round(callback * WINDOW),
    }


def headroom(fraction=0.8):
    shutdowns = []
    monitor = CpuHeadroom("mcu", shutdowns.append)
    if fraction is not None:
        monitor.guard(fraction)
    return monitor, shutdowns


def events(caplog):
    return [
        r
        for r in caplog.records
        if getattr(r, "event", None) == "cpu_headroom_exceeded"
    ]


def test_the_non_step_share_is_what_was_neither_asleep_nor_stepping():
    window = CpuLoadWindow(load(idle=0.5, step=0.3, callback=0.05))
    assert window.idle_share == pytest.approx(0.5)
    assert window.step_share == pytest.approx(0.3)
    assert window.nonstep_share == pytest.approx(0.2)


def test_an_empty_window_is_refused():
    with pytest.raises(ValueError, match="window"):
        CpuLoadWindow({**load(0.5, 0.3, 0.0), "window_ticks": 0})


def test_load_within_the_headroom_raises_nothing(caplog):
    monitor, shutdowns = headroom()
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(load(idle=0.0, step=0.82, callback=0.1), 0.0, FREQ)
    assert events(caplog) == []
    assert shutdowns == []


def test_excess_non_step_load_with_idle_cpu_is_reported_not_fatal(caplog):
    monitor, shutdowns = headroom()
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(load(idle=0.3, step=0.4, callback=0.1), 0.0, FREQ)
    (event,) = events(caplog)
    assert event.levelno == logging.WARNING
    assert event.nonstep_share == pytest.approx(0.3)
    assert shutdowns == []


def test_excess_non_step_load_on_a_saturated_stepping_cpu_shuts_down():
    monitor, shutdowns = headroom()
    monitor.observe(load(idle=0.01, step=0.7, callback=0.2), 0.0, FREQ)
    (message,) = shutdowns
    assert "MCU 'mcu' non-step load took 29.0% of the CPU" in message
    assert "more than the 20.0% the step capacity fraction 0.80 leaves" in message
    assert "saturated" in message


def test_a_saturated_cpu_that_is_not_stepping_does_not_shut_down():
    monitor, shutdowns = headroom()
    monitor.observe(load(idle=0.0, step=0.0, callback=0.5), 0.0, FREQ)
    assert shutdowns == []


def test_an_mcu_without_a_step_cap_is_only_measured():
    monitor, shutdowns = headroom(fraction=None)
    monitor.observe(load(idle=0.0, step=0.5, callback=0.4), 0.0, FREQ)
    assert shutdowns == []
    assert "mcu_nonstep_max=0.500" in monitor.take_stats()


def test_reports_are_rate_limited_and_count_the_windows(caplog):
    monitor, _ = headroom()
    excess = load(idle=0.3, step=0.4, callback=0.1)
    with caplog.at_level(logging.INFO, logger="kalico.event"):
        monitor.observe(excess, 0.0, FREQ)
        monitor.observe(excess, 10.0, FREQ)
        monitor.observe(excess, 31.0, FREQ)
    first, second = events(caplog)
    assert (first.windows, second.windows) == (1, 2)


def test_stats_report_the_extremes_since_the_last_read():
    monitor, _ = headroom()
    monitor.observe(load(idle=0.9, step=0.05, callback=0.0), 0.0, FREQ)
    monitor.observe(load(idle=0.5, step=0.3, callback=0.0), 0.0, FREQ)
    assert monitor.take_stats() == "mcu_nonstep_max=0.200 mcu_idle_min=0.500"
    assert monitor.take_stats() == "mcu_nonstep_max=0.000 mcu_idle_min=1.000"
