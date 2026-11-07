"""Instrumented klippy launcher for bench investigations (Pi side).

    python klippy_probe.py TREE OUT_JSON <klippy args...>

Runs TREE's unmodified klippy with wrappers installed from outside it:
- every reactor timer: how late it ran against the waketime it asked for
  (the reactor's responsiveness; a stall shows as every due timer late),
  and the wall time of each callback (includes reactor.pause() inside it);
  timers later than LATE_EVENT_S are logged with the toolhead's queuing
  state and lookahead length at that moment;
- a watchdog thread that sees a stall while it is happening: once a timer
  is overdue by STALL_S it records the stack of every thread (the main
  thread's is the stalled reactor callback), the toolhead state, and starts
  a `py-spy dump` (all threads, --native when it works) of this process;
- printer events (connect/ready/shutdown) and corner_deviation's CornerTable
  construction, on klippy's clock (CLOCK_MONOTONIC_RAW).
OUT_JSON is rewritten once a second by a thread.
"""

import json
import os
import subprocess
import sys
import threading
import time
import traceback

TOP = 40
LATE_EVENT_S = 0.020
STALL_S = 0.020
WATCHDOG_PERIOD_S = 0.005
MAX_STALLS = 50
MAX_PYSPY = 20
MAX_LATE_EVENTS = 2000
MAX_LATE_SAMPLES = 20000
LATE_SAMPLE_S = 0.002
HIST_EDGES_MS = (1, 2, 5, 10, 20, 50, 100, 200, 500)
STACK_FRAMES = 30


def mono():
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def main():
    tree, out_path = sys.argv[1], sys.argv[2]
    sys.argv = ["klippy"] + sys.argv[3:]
    sys.path.insert(0, tree)
    from klippy import printer, reactor

    never = reactor.SelectReactor.NEVER
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
        "late_samples": [],
        "late_histogram_ms": {"edges": HIST_EDGES_MS, "counts": [0] * 10},
        "late_events": [],
        "stalls": [],
        "pyspy_native": None,
    }
    requested = {}
    labels = {}
    printers = []

    def name_of(callback):
        owner = getattr(callback, "__self__", None)
        base = getattr(callback, "__qualname__", repr(callback))
        return "%s.%s" % (type(owner).__module__, base) if owner else base

    def toolhead_state():
        if not printers:
            return None
        toolhead = printers[0].lookup_object("toolhead", None)
        if toolhead is None:
            return None
        return {
            "queuing_state": toolhead.special_queuing_state or "main",
            "lookahead_moves": len(toolhead.lookahead.queue),
            "print_time": toolhead.print_time,
        }

    def keep_top(key, item, value_key):
        phase = "ready" if "klippy:ready" in state["events"] else "startup"
        items = state["%s_%s" % (key, phase)]
        items.append(item)
        items.sort(key=lambda i: -i[value_key])
        del items[TOP:]

    def note_late(label, late, at):
        bucket = sum(late * 1e3 >= edge for edge in HIST_EDGES_MS)
        state["late_histogram_ms"]["counts"][bucket] += 1
        if (
            late >= LATE_SAMPLE_S
            and len(state["late_samples"]) < MAX_LATE_SAMPLES
        ):
            state["late_samples"].append((at, late))
        item = {"callback": label, "late_s": late, "at": at}
        keep_top("late_timers", item, "late_s")
        if late >= LATE_EVENT_S and len(state["late_events"]) < MAX_LATE_EVENTS:
            state["late_events"].append(dict(item, toolhead=toolhead_state()))

    def timed(callback, timer=None):
        label = name_of(callback)

        def run(eventtime, *args):
            start = mono()
            if timer is not None:
                want = requested.get(id(timer))
                requested[id(timer)] = never
                if want is not None and 0.0 < want < never:
                    with lock:
                        note_late(label, start - want, start)
            result = callback(eventtime, *args)
            end = mono()
            if timer is not None and id(timer) in requested:
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
    real_unregister_timer = base.unregister_timer

    def register_timer(self, callback, waketime=never):
        timer = real_register_timer(self, callback, waketime)
        timer.callback = timed(callback, timer)
        labels[id(timer)] = name_of(callback)
        requested[id(timer)] = waketime
        return timer

    def update_timer(self, timer_handler, waketime):
        requested[id(timer_handler)] = waketime
        return real_update_timer(self, timer_handler, waketime)

    def unregister_timer(self, timer_handler):
        requested.pop(id(timer_handler), None)
        labels.pop(id(timer_handler), None)
        return real_unregister_timer(self, timer_handler)

    base.register_timer = register_timer
    base.update_timer = update_timer
    base.unregister_timer = unregister_timer
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

    def mark(event):
        def handler(*_):
            with lock:
                state["events"].setdefault(event, mono())

        return handler

    real_printer_init = printer.Printer.__init__

    def printer_init(self, *args, **kwargs):
        real_printer_init(self, *args, **kwargs)
        printers[:] = [self]
        for event in ("klippy:connect", "klippy:ready", "klippy:shutdown"):
            self.register_event_handler(event, mark(event))

    printer.Printer.__init__ = printer_init

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
    if corner_deviation is not None and hasattr(
        corner_deviation, "CornerDeviation"
    ):
        cd_class = corner_deviation.CornerDeviation
        real_build = cd_class._build
        real_publish = cd_class._publish

        def build(self, generation, specs):
            start = mono()
            cached = sorted(os.listdir(self.cache_dir))
            with lock:
                state["table_builds"].append(
                    {
                        "generation": generation,
                        "start": start,
                        "cache_files_before": cached,
                    }
                )
            real_build(self, generation, specs)

        def publish(self, generation, result):
            with lock:
                for item in state["table_builds"]:
                    if (
                        item.get("generation") == generation
                        and "published" not in item
                    ):
                        item["published"] = mono()
                        item["error"] = (
                            None if result[1] is None else repr(result[1])
                        )
            return real_publish(self, generation, result)

        cd_class._build = build
        cd_class._publish = publish

    py_spy = os.path.join(os.path.dirname(sys.executable), "py-spy")
    pyspy_busy = threading.Lock()

    def pyspy_dump(stall):
        if not pyspy_busy.acquire(blocking=False):
            return
        try:
            for native in ((True, False) if state["pyspy_native"] is not False
                           else (False,)):  # fmt: skip
                cmd = [
                    "sudo", "-n", "taskset", "-c", "0-%d" % (os.cpu_count() - 1),
                    py_spy, "dump", "--pid", str(os.getpid()),
                ]  # fmt: skip
                result = subprocess.run(
                    cmd + (["--native"] if native else []),
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                if result.returncode == 0:
                    with lock:
                        state["pyspy_native"] = native
                        stall["pyspy"] = result.stdout
                    return
                with lock:
                    stall.setdefault("pyspy_errors", []).append(
                        result.stderr[-500:]
                    )
                    if native:
                        state["pyspy_native"] = False
        finally:
            pyspy_busy.release()

    def capture_stall(now, overdue, timer_label):
        frames = sys._current_frames()
        names = {t.ident: t.name for t in threading.enumerate()}
        main_ident = threading.main_thread().ident
        stall = {
            "at": now,
            "overdue_s": overdue,
            "max_overdue_s": overdue,
            "overdue_timer": timer_label,
            "toolhead": toolhead_state(),
            "main_stack": traceback.format_stack(frames[main_ident])[
                -STACK_FRAMES:
            ],
            "threads": {
                names.get(ident, str(ident)): traceback.format_stack(frame)[-6:]
                for ident, frame in frames.items()
                if ident not in (main_ident, threading.get_ident())
            },
        }
        with lock:
            state["stalls"].append(stall)
            spy = len(state["stalls"]) <= MAX_PYSPY
        if spy:
            threading.Thread(
                target=pyspy_dump, args=(stall,), daemon=True
            ).start()
        return stall

    def watchdog():
        stall = None
        while True:
            time.sleep(WATCHDOG_PERIOD_S)
            now = mono()
            pending = [
                (want, key)
                for key, want in list(requested.items())
                if 0.0 < want < never
            ]
            if not pending:
                continue
            want, key = min(pending)
            overdue = now - want
            if stall is None and overdue > STALL_S:
                if len(state["stalls"]) < MAX_STALLS:
                    stall = capture_stall(now, overdue, labels.get(key))
            elif stall is not None:
                with lock:
                    if overdue > STALL_S:
                        stall["max_overdue_s"] = max(
                            stall["max_overdue_s"], overdue
                        )
                    else:
                        stall["ended"] = now
                        stall = None

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
    threading.Thread(target=watchdog, daemon=True).start()
    try:
        printer.main()
    finally:
        write()


if __name__ == "__main__":
    main()
