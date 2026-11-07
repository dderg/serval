"""Instrumented klippy launcher for bench investigations (Pi side).

    python klippy_probe.py TREE OUT_JSON <klippy args...>

Runs TREE's unmodified klippy with wrappers installed from outside it:
- every reactor timer: how late it ran against the waketime it asked for
  (the reactor's responsiveness; a stall shows as every due timer late),
  and the wall time of each callback (includes reactor.pause() inside it);
- every fd callback's wall time;
- printer events (klippy:connect/ready) and corner_deviation's CornerTable
  construction, on klippy's clock (CLOCK_MONOTONIC_RAW).
OUT_JSON is rewritten once a second by a thread.
"""

import json
import os
import sys
import threading
import time

TOP = 40


def mono():
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def main():
    tree, out_path = sys.argv[1], sys.argv[2]
    sys.argv = ["klippy"] + sys.argv[3:]
    sys.path.insert(0, tree)
    from klippy import printer, reactor

    lock = threading.Lock()
    state = {
        "pid": os.getpid(),
        "process_start": mono(),
        "events": {},
        "table_builds": [],
        "late_timers_startup": [],
        "late_timers_ready": [],
        "long_callbacks_startup": [],
        "long_callbacks_ready": [],
        "per_callback": {},
    }
    requested = {}

    def name_of(callback):
        owner = getattr(callback, "__self__", None)
        base = getattr(callback, "__qualname__", repr(callback))
        return "%s.%s" % (type(owner).__module__, base) if owner else base

    def keep_top(key, item, value_key):
        phase = "ready" if "klippy:ready" in state["events"] else "startup"
        items = state["%s_%s" % (key, phase)]
        items.append(item)
        items.sort(key=lambda i: -i[value_key])
        del items[TOP:]

    def timed(callback, timer=None):
        label = name_of(callback)

        def run(eventtime, *args):
            start = mono()
            if timer is not None:
                want = requested.get(id(timer))
                if (
                    want is not None
                    and 0.0 < want < reactor.SelectReactor.NEVER
                ):
                    late = start - want
                    with lock:
                        keep_top(
                            "late_timers",
                            {"callback": label, "late_s": late, "at": start},
                            "late_s",
                        )
            result = callback(eventtime, *args)
            end = mono()
            if timer is not None:
                requested[id(timer)] = result
            with lock:
                stats = state["per_callback"].setdefault(
                    label, {"count": 0, "total_s": 0.0, "max_s": 0.0}
                )
                stats["count"] += 1
                stats["total_s"] += end - start
                stats["max_s"] = max(stats["max_s"], end - start)
                if end - start > 0.005:
                    keep_top(
                        "long_callbacks",
                        {"callback": label, "wall_s": end - start, "at": start},
                        "wall_s",
                    )
            return result

        return run

    base = reactor.SelectReactor
    real_register_timer = base.register_timer
    real_update_timer = base.update_timer

    def register_timer(self, callback, waketime=reactor.SelectReactor.NEVER):
        timer = real_register_timer(self, callback, waketime)
        timer.callback = timed(callback, timer)
        requested[id(timer)] = waketime
        return timer

    def update_timer(self, timer_handler, waketime):
        requested[id(timer_handler)] = waketime
        return real_update_timer(self, timer_handler, waketime)

    base.register_timer = register_timer
    base.update_timer = update_timer
    for cls in (reactor.PollReactor, reactor.EPollReactor, base):
        real_register_fd = cls.register_fd

        def register_fd(
            self, fd, read_cb, write_cb=None, _real=real_register_fd
        ):
            return _real(
                self,
                fd,
                timed(read_cb),
                None if write_cb is None else timed(write_cb),
            )

        cls.register_fd = register_fd

    real_send_event = printer.Printer.send_event

    def send_event(self, event, *params):
        with lock:
            state["events"].setdefault(event, mono())
        return real_send_event(self, event, *params)

    printer.Printer.send_event = send_event

    try:
        from klippy.extras import corner_deviation
    except ImportError:
        corner_deviation = None
    if corner_deviation is not None and hasattr(
        corner_deviation, "CornerTable"
    ):
        real_table_init = corner_deviation.CornerTable.__init__

        def table_init(self, *args, **kwargs):
            start = mono()
            real_table_init(self, *args, **kwargs)
            with lock:
                state["table_builds"].append({"start": start, "end": mono()})

        corner_deviation.CornerTable.__init__ = table_init

    def write():
        with lock:
            text = json.dumps(state)
        partial = out_path + ".partial"
        with open(partial, "w") as f:
            f.write(text)
        os.replace(partial, out_path)

    def writer():
        while True:
            write()
            time.sleep(1.0)

    threading.Thread(target=writer, daemon=True).start()
    try:
        printer.main()
    finally:
        write()


if __name__ == "__main__":
    main()
