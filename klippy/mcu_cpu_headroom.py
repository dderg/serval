"""The runtime check behind the step capacity cap.

The planner lets steps take STEP_CAPACITY_FRACTION of each benchmarked MCU's
clock, charging every step its benchmarked cost, and assumes everything else
(tasks, other interrupts, non-step timer callbacks) fits in the rest. Every
`cpu_load` window the MCU reports how long it slept, how long the timer
interrupt ran, how much of that non-step timers took, and how many step
timer events it dispatched. The non-step share is what was neither asleep
nor stepping; the charged step share is those events at their benchmarked
cost (a pulsed step is two events, a both-edge step one), taking the cheaper
mode when the MCU steps both, so a shutdown is never charged more steps
than ran.

A window whose non-step share exceeds 1 - fraction violates the cap's
assumption and is reported. When the charged step share and the non-step
share together exceed the CPU, the violation is one that squeezes the steps,
and the printer shuts down.
"""

import logging

from . import structured_log

EVENT_INTERVAL_S = 30.0


def ticks_per_step_event(step_capacity):
    per_event = {
        "pulsed": step_capacity["pulsed_ticks_per_step"] / 2.0,
        "both-edge": step_capacity["both_edge_ticks_per_step"],
    }
    return min(per_event[m["mode"]] for m in step_capacity["motors"])


class CpuLoadWindow:
    def __init__(self, params, ticks_per_step_event):
        window = params["window_ticks"]
        if window <= 0:
            raise ValueError("cpu_load window of %d ticks" % (window,))
        step = params["timer_irq_ticks"] - params["nonstep_timer_ticks"]
        self.window_ticks = window
        self.idle_share = params["idle_ticks"] / window
        self.step_share = step / window
        self.nonstep_share = 1.0 - self.idle_share - self.step_share
        self.charged_step_share = (
            params["step_events"] * ticks_per_step_event / window
        )


class CpuHeadroom:
    def __init__(self, name, shutdown):
        self._name = name
        self._shutdown = shutdown
        self._fraction = None
        self._ticks_per_step_event = 0.0
        self._last_event_time = None
        self._exceeded_since_event = 0
        self.nonstep_max = 0.0
        self.idle_min = 1.0

    def guard(self, step_capacity):
        self._fraction = step_capacity["capacity_fraction"]
        self._ticks_per_step_event = ticks_per_step_event(step_capacity)

    def observe(self, params, eventtime, mcu_freq):
        window = CpuLoadWindow(params, self._ticks_per_step_event)
        self.nonstep_max = max(self.nonstep_max, window.nonstep_share)
        self.idle_min = min(self.idle_min, window.idle_share)
        if self._fraction is None:
            return
        fraction = self._fraction
        allowance = 1.0 - fraction
        if window.nonstep_share <= allowance:
            return
        detail = (
            "MCU '%s' non-step load took %.1f%% of the CPU in a %.3f s window,"
            " more than the %.1f%% the step capacity fraction %.2f leaves"
            " (steps %.1f%% measured, %.1f%% at their benchmarked cost;"
            " idle %.1f%%)"
            % (
                self._name,
                100.0 * window.nonstep_share,
                window.window_ticks / mcu_freq,
                100.0 * allowance,
                fraction,
                100.0 * window.step_share,
                100.0 * window.charged_step_share,
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
                charged_step_share=window.charged_step_share,
                idle_share=window.idle_share,
                capacity_fraction=fraction,
            )
            self._last_event_time = eventtime
            self._exceeded_since_event = 0
        if window.charged_step_share + window.nonstep_share > 1.0:
            self._shutdown(
                detail + ": steps at the cap's charge and this non-step load"
                " exceed the CPU, so the step capacity cap no longer holds"
            )

    def take_stats(self):
        stats = "mcu_nonstep_max=%.3f mcu_idle_min=%.3f" % (
            self.nonstep_max,
            self.idle_min,
        )
        self.nonstep_max = 0.0
        self.idle_min = 1.0
        return stats
