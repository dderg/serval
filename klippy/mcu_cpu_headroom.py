"""The runtime check behind the step capacity cap.

The planner lets steps take STEP_CAPACITY_FRACTION of each benchmarked MCU's
clock and assumes everything else (tasks, other interrupts, non-step timer
callbacks) fits in the rest. Every `cpu_load` window the MCU reports how long
it slept, how long the timer interrupt ran and how much of that was non-step
callbacks; the non-step share is what was neither asleep nor stepping. A
window whose non-step share exceeds 1 - fraction is reported; one where the
CPU was also saturated, so the steps were squeezed, shuts the printer down.
"""

import logging

from . import structured_log

SATURATED_IDLE_SHARE = 0.02
EVENT_INTERVAL_S = 30.0


class CpuLoadWindow:
    def __init__(self, params):
        window = params["window_ticks"]
        if window <= 0:
            raise ValueError("cpu_load window of %d ticks" % (window,))
        step = params["timer_irq_ticks"] - params["callback_ticks"]
        self.window_ticks = window
        self.idle_share = params["idle_ticks"] / window
        self.step_share = step / window
        self.nonstep_share = 1.0 - self.idle_share - self.step_share


class CpuHeadroom:
    def __init__(self, name, shutdown):
        self._name = name
        self._shutdown = shutdown
        self._fraction = None
        self._last_event_time = None
        self._exceeded_since_event = 0
        self.nonstep_max = 0.0
        self.idle_min = 1.0

    def guard(self, capacity_fraction):
        self._fraction = capacity_fraction

    def observe(self, params, eventtime, mcu_freq):
        window = CpuLoadWindow(params)
        self.nonstep_max = max(self.nonstep_max, window.nonstep_share)
        self.idle_min = min(self.idle_min, window.idle_share)
        if self._fraction is None:
            return
        allowance = 1.0 - self._fraction
        if window.nonstep_share <= allowance:
            return
        detail = (
            "MCU '%s' non-step load took %.1f%% of the CPU in a %.3f s window,"
            " more than the %.1f%% the step capacity fraction %.2f leaves"
            " (steps %.1f%%, idle %.1f%%)"
            % (
                self._name,
                100.0 * window.nonstep_share,
                window.window_ticks / mcu_freq,
                100.0 * allowance,
                self._fraction,
                100.0 * window.step_share,
                100.0 * window.idle_share,
            )
        )
        self._exceeded_since_event += 1
        if (
            self._last_event_time is None
            or eventtime - self._last_event_time >= EVENT_INTERVAL_S
        ):
            structured_log.event(
                "mcu",
                "cpu_headroom_exceeded",
                level=logging.WARNING,
                msg=detail,
                mcu=self._name,
                windows=self._exceeded_since_event,
                nonstep_share=window.nonstep_share,
                step_share=window.step_share,
                idle_share=window.idle_share,
                capacity_fraction=self._fraction,
            )
            self._last_event_time = eventtime
            self._exceeded_since_event = 0
        if window.idle_share < SATURATED_IDLE_SHARE and window.step_share > 0.0:
            self._shutdown(
                detail + " while the CPU was saturated: the step capacity"
                " cap's headroom assumption no longer holds"
            )

    def take_stats(self):
        stats = "mcu_nonstep_max=%.3f mcu_idle_min=%.3f" % (
            self.nonstep_max,
            self.idle_min,
        )
        self.nonstep_max = 0.0
        self.idle_min = 1.0
        return stats
