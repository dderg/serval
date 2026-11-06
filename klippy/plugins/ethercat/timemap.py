# Mapping between print_time and the EtherCAT endpoint's DC grid
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math

from klippy import clocksync

RAW_NS_PER_S = 1e9


class GridRegression(Exception):
    pass


class PrintTimeRegression:
    """The main MCU's print_time as the (time, clock, freq) regression
    SecondarySync reads from its main clock, built from the public
    MCU.estimated_print_time(), which is linear in host time between
    clocksync updates. Its "clock" counts print-time seconds."""

    mcu_freq = 1.0

    def __init__(self, mcu):
        self.mcu = mcu
        self.clock_est = (0.0, 0.0, 0.0)

    def refresh(self, eventtime):
        print_time = self.mcu.estimated_print_time(eventtime)
        rate = self.mcu.estimated_print_time(eventtime + 1.0) - print_time
        self.clock_est = (eventtime, print_time, rate)

    def estimated_print_time(self, eventtime):
        return self.mcu.estimated_print_time(eventtime)


class EndpointClockMap(clocksync.SecondarySync):
    """print_time <-> endpoint clock (CLOCK_MONOTONIC_RAW nanoseconds).

    The endpoint runs on the host and stamps everything with the same
    kernel clock as reactor.monotonic(), so it is a secondary clock whose
    value at host time t is exactly t * 1e9. SecondarySync's calibration
    keeps the map continuous at the last generated print_time, so samples
    already emitted never move when the main MCU's regression changes.
    """

    def __init__(self, reactor, mcu, eventtime):
        main = PrintTimeRegression(mcu)
        main.refresh(eventtime)
        clocksync.SecondarySync.__init__(self, reactor, main)
        self.mcu_freq = RAW_NS_PER_S
        self.clock_est = (0.0, 0.0, RAW_NS_PER_S)
        main_print_time = main.estimated_print_time(eventtime)
        self.clock_adj = (main_print_time - eventtime, RAW_NS_PER_S)
        self.calibrate_clock(0.0, eventtime)

    def calibrate_clock(self, print_time, eventtime):
        self.main_sync.refresh(eventtime)
        return clocksync.SecondarySync.calibrate_clock(
            self, print_time, eventtime
        )

    def exact_clock(self, print_time):
        offset, freq = self.clock_adj
        return (print_time - offset) * freq


class SyntheticGrid:
    """Batch-mode grid: index n plays at print_time n * cycle."""

    def __init__(self, cycle_ns):
        self.cycle_ns = cycle_ns
        self.interval = cycle_ns / RAW_NS_PER_S

    def print_time(self, index):
        return index * self.interval

    def first_index_at_or_after(self, print_time):
        estimate = max(0, math.ceil(print_time / self.interval))
        return _settle_at_or_after(self, print_time, estimate)


class EndpointGrid:
    """The endpoint's DC grid as last reported (grid_index, grid_clock).

    grid_clock already includes the endpoint's group delay, so the clock of
    index n is the instant whose commanded motion that cycle must carry.
    The pair is replaced as one tuple: the socket thread updates it while
    the reactor reads it.
    """

    def __init__(self, clock_map, cycle_ns, grid_index, grid_clock):
        self.clock_map = clock_map
        self.cycle_ns = cycle_ns
        self.anchor = (grid_index, grid_clock)

    def update(self, grid_index, grid_clock):
        if grid_index < self.anchor[0]:
            raise GridRegression(
                "endpoint grid index went back from %d to %d"
                % (self.anchor[0], grid_index)
            )
        self.anchor = (grid_index, grid_clock)

    def clock(self, index):
        grid_index, grid_clock = self.anchor
        return grid_clock + (index - grid_index) * self.cycle_ns

    def print_time(self, index):
        return self.clock_map.clock_to_print_time(self.clock(index))

    @property
    def interval(self):
        return self.cycle_ns / self.clock_map.clock_adj[1]

    def index_at_or_after_clock(self, clock):
        grid_index, grid_clock = self.anchor
        return grid_index + math.ceil((clock - grid_clock) / self.cycle_ns)

    def first_index_at_or_after(self, print_time):
        estimate = self.index_at_or_after_clock(
            self.clock_map.exact_clock(print_time)
        )
        return _settle_at_or_after(self, print_time, estimate)


def _settle_at_or_after(grid, print_time, index):
    # The estimate inverts the grid in floating point and may land one
    # index early; samples taken from it must not precede print_time.
    while grid.print_time(index) < print_time:
        index += 1
    return index
