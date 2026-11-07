"""Exercise the printer-object hooks that only a live klippy calls.

    python -m tools.motion_eval.object_probe RESULT_JSON <klippy args...>

Batch mode never starts klippy/extras/statistics.py's 1 s timer (it checks
debugoutput) and serves no API clients, so after klippy:ready this does what
they do: every object's `stats` attribute, collected by the same rule as
PrinterStats.handle_ready, is called with an eventtime and must return
(bool, str); every get_status(eventtime) must return a dict json can
serialize, as webhooks sends it to Moonraker. Failures go to RESULT_JSON,
with the longest stretch the reactor spent without running its timers
while the gcode ran (StallMeter).
"""

from __future__ import annotations

import json
import sys
import time

from klippy import printer, toolhead


def probe(printer_obj):
    eventtime = printer_obj.get_reactor().monotonic()
    failures = []
    for name, obj in printer_obj.lookup_objects():
        if hasattr(obj, "stats"):
            try:
                result = obj.stats(eventtime)
                ok = (
                    isinstance(result, tuple)
                    and len(result) == 2
                    and isinstance(result[1], str)
                )
                if not ok:
                    failures.append("%s.stats returned %r" % (name, result))
            except Exception as e:
                failures.append(
                    "%s.stats: %s: %s" % (name, type(e).__name__, e)
                )
        if hasattr(obj, "get_status"):
            try:
                status = obj.get_status(eventtime)
                if not isinstance(status, dict):
                    failures.append(
                        "%s.get_status returned %r" % (name, status)
                    )
                json.dumps(status)
            except Exception as e:
                failures.append(
                    "%s.get_status: %s: %s" % (name, type(e).__name__, e)
                )
    return failures


STALL_TICK_S = 0.01


class StallMeter:
    """Largest main-thread CPU time between two runs of a reactor timer
    due every STALL_TICK_S: how long one callback held the reactor, the
    time heaters (max_duration), clock sync and the API wait for."""

    def __init__(self, reactor):
        self.reactor = reactor
        self.last = time.thread_time()
        self.max_stall_s = 0.0
        reactor.register_timer(self.tick, reactor.NOW)

    def tick(self, eventtime):
        now = time.thread_time()
        self.max_stall_s = max(self.max_stall_s, now - self.last)
        self.last = now
        return eventtime + STALL_TICK_S


def main():
    result_path = sys.argv[1]
    results = {"failures": None}
    meters = []
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        printer_obj = self.printer

        def on_ready():
            results["failures"] = probe(printer_obj)
            meters.append(StallMeter(printer_obj.get_reactor()))

        printer_obj.register_event_handler("klippy:ready", on_ready)

    toolhead.ToolHead.__init__ = init
    sys.argv = ["klippy"] + sys.argv[2:]
    printer.main()
    with open(result_path, "w") as f:
        results["max_stall_s"] = meters[0].max_stall_s if meters else None
        json.dump(results, f)


if __name__ == "__main__":
    main()
