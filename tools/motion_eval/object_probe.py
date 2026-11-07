"""Exercise the printer-object hooks that only a live klippy calls.

    python -m tools.motion_eval.object_probe RESULT_JSON <klippy args...>

Batch mode never starts klippy/extras/statistics.py's 1 s timer (it checks
debugoutput) and serves no API clients, so after klippy:ready this does what
they do: every object's `stats` attribute, collected by the same rule as
PrinterStats.handle_ready, is called with an eventtime and must return
(bool, str); every get_status(eventtime) must return a dict json can
serialize, as webhooks sends it to Moonraker. Failures go to RESULT_JSON.
"""

from __future__ import annotations

import json
import sys

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


def main():
    result_path = sys.argv[1]
    results = {"failures": None}
    real_init = toolhead.ToolHead.__init__

    def init(self, config):
        real_init(self, config)
        printer_obj = self.printer

        def on_ready():
            results["failures"] = probe(printer_obj)

        printer_obj.register_event_handler("klippy:ready", on_ready)

    toolhead.ToolHead.__init__ = init
    sys.argv = ["klippy"] + sys.argv[2:]
    printer.main()
    with open(result_path, "w") as f:
        json.dump(results, f)


if __name__ == "__main__":
    main()
