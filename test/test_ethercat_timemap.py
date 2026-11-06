"""print_time <-> endpoint clock and DC grid mapping."""

from __future__ import annotations

import pytest

from klippy import clocksync
from klippy.plugins.ethercat.timemap import (
    RAW_NS_PER_S,
    EndpointClockMap,
    EndpointGrid,
    GridRegression,
    SyntheticGrid,
)

MCU_FREQ = 520e6
CYCLE_NS = 250000
# Host-vs-MCU crystal error and its change between regressions, both well
# inside what clocksync tracks on real hardware.
DRIFT_PPM = 40.0
DRIFT_STEP_PPM = 15.0


class Reactor:
    NEVER = 9999999999999999.0

    def register_timer(self, callback, waketime=NEVER):
        return None


def main_sync(freq_ppm):
    sync = clocksync.ClockSync(Reactor())
    sync.mcu_freq = MCU_FREQ
    # The MCU clock reads 0 at host time 10 s and runs freq_ppm fast
    sync.clock_est = (10.0, 0.0, MCU_FREQ * (1.0 + freq_ppm * 1e-6))
    return sync


def test_map_follows_main_mcu_print_time():
    sync = main_sync(DRIFT_PPM)
    cmap = EndpointClockMap(Reactor(), sync, 20.0)
    for host_time in (20.0, 21.5):
        print_time = sync.estimated_print_time(host_time)
        clock = cmap.exact_clock(print_time)
        # SecondarySync anchors on clocks truncated to whole nanoseconds
        assert clock == pytest.approx(host_time * RAW_NS_PER_S, abs=2.0)


def test_recalibration_is_continuous_at_the_generated_print_time():
    sync = main_sync(DRIFT_PPM)
    cmap = EndpointClockMap(Reactor(), sync, 20.0)
    generated = sync.estimated_print_time(20.0) + 0.5
    before = cmap.exact_clock(generated)
    sync.clock_est = (
        sync.clock_est[0],
        sync.clock_est[1],
        MCU_FREQ * (1.0 + (DRIFT_PPM + DRIFT_STEP_PPM) * 1e-6),
    )
    cmap.calibrate_clock(generated, 20.2)
    # Truncation to whole nanoseconds inside SecondarySync is the only
    # discontinuity left at the anchor.
    assert abs(cmap.exact_clock(generated) - before) <= 1.0
    slope = cmap.clock_adj[1] / RAW_NS_PER_S - 1.0
    assert abs(slope) < 2 * (DRIFT_PPM + DRIFT_STEP_PPM) * 1e-6


def test_endpoint_grid_indices_and_print_times_agree():
    sync = main_sync(DRIFT_PPM)
    cmap = EndpointClockMap(Reactor(), sync, 20.0)
    grid = EndpointGrid(cmap, CYCLE_NS, 1000, int(20.5 * RAW_NS_PER_S))
    half = 0.5 * grid.interval
    for index in (1000, 1001, 5000):
        at = grid.print_time(index)
        assert grid.first_index_at_or_after(at - half) == index
        assert grid.first_index_at_or_after(at + half) == index + 1
    step = grid.print_time(1001) - grid.print_time(1000)
    assert step == pytest.approx(grid.interval, rel=1e-9)
    later = grid.print_time(1000) + 0.3 * grid.interval
    assert grid.first_index_at_or_after(later) == 1001


def test_endpoint_grid_refuses_to_go_back():
    sync = main_sync(DRIFT_PPM)
    grid = EndpointGrid(
        EndpointClockMap(Reactor(), sync, 20.0), CYCLE_NS, 10, 0
    )
    grid.update(11, CYCLE_NS)
    with pytest.raises(GridRegression):
        grid.update(10, 0)


def test_synthetic_grid():
    grid = SyntheticGrid(CYCLE_NS)
    assert grid.interval == CYCLE_NS / RAW_NS_PER_S
    half = 0.5 * grid.interval
    assert grid.first_index_at_or_after(0.0) == 0
    assert grid.first_index_at_or_after(grid.print_time(7) - half) == 7
    assert grid.first_index_at_or_after(grid.print_time(7) + half) == 8
