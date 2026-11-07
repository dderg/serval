import json
import subprocess
import sys
import textwrap
from pathlib import Path

PROBE = Path(__file__).resolve().parents[1] / "klippy_probe.py"

REACTOR = """
import time
NOW, NEVER = 0.0, 9999999999999999.0

def monotonic():
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)

class Timer:
    def __init__(self, callback, waketime):
        self.callback, self.waketime = callback, waketime

class SelectReactor:
    NOW, NEVER = NOW, NEVER
    def __init__(self):
        self.timers = []
    def register_timer(self, callback, waketime=NEVER):
        timer = Timer(callback, waketime)
        self.timers.append(timer)
        return timer
    def update_timer(self, timer, waketime):
        timer.waketime = waketime
    def unregister_timer(self, timer):
        self.timers.remove(timer)
    def register_fd(self, fd, read_cb, write_cb=None):
        return None
    def run(self, until):
        while monotonic() < until:
            now = monotonic()
            for t in list(self.timers):
                if now >= t.waketime:
                    t.waketime = NEVER
                    t.waketime = t.callback(now)
            time.sleep(0.001)

class PollReactor(SelectReactor):
    pass

class EPollReactor(SelectReactor):
    pass
"""

PRINTER = """
import time
from . import reactor

class Toolhead:
    special_queuing_state = "Priming"
    print_time = 1.5
    class lookahead:
        queue = [1, 2, 3]

class Printer:
    def __init__(self, r):
        self.reactor = r
        self.handlers = {}
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def lookup_object(self, name, default=None):
        return Toolhead() if name == "toolhead" else default

def stall_for_a_while(eventtime):
    time.sleep(0.15)
    return reactor.NEVER

def main():
    r = reactor.PollReactor()
    p = Printer(r)
    for cb in p.handlers["klippy:ready"]:
        cb()
    start = reactor.monotonic()
    r.register_timer(lambda e: e + 0.002, start)
    r.register_timer(stall_for_a_while, start + 0.05)
    r.run(start + 0.6)
"""


def test_probe_catches_a_stall_while_it_happens(tmp_path):
    package = tmp_path / "tree" / "klippy"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("")
    (package / "reactor.py").write_text(textwrap.dedent(REACTOR))
    (package / "printer.py").write_text(textwrap.dedent(PRINTER))
    out = tmp_path / "probe.json"
    subprocess.run(
        [sys.executable, str(PROBE), str(tmp_path / "tree"), str(out)],
        check=True,
        timeout=30,
    )
    state = json.loads(out.read_text())
    assert "klippy:ready" in state["events"]
    stall = state["stalls"][0]
    assert stall["max_overdue_s"] > 0.08
    assert any("stall_for_a_while" in frame for frame in stall["main_stack"])
    assert stall["toolhead"] == {
        "queuing_state": "Priming",
        "lookahead_moves": 3,
        "print_time": 1.5,
    }
    late = state["late_events"][0]
    assert late["late_s"] > 0.08
    assert late["toolhead"]["queuing_state"] == "Priming"
