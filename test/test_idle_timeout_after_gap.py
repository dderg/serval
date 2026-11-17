"""#993: the first toolhead activity after a long rest must arm idle_timeout's
Printing -> Ready check from the time motion can next start, not from the
engine frontier, which sits GAP_S in the past after the rest."""

from fakes import FakeMcu, FakeReactor

from klippy.extras.idle_timeout import READY_TIMEOUT, IdleTimeout
from klippy.motion import Motion

NOW = 1000.0
GAP_S = 40.0
MOTION_LEAD_S = 0.25
IDLE_TIMEOUT_S = 6.5


class StaleFrontierEngine:
    def __init__(self, frontier):
        self.frontier = frontier

    def frontier_print_time(self, mcu_handle):
        return self.frontier


class DispatchingPrinter:
    def __init__(self):
        self.handlers = {}
        self.events = []

    def register_event_handler(self, name, callback):
        self.handlers.setdefault(name, []).append(callback)

    def send_event(self, name, *params):
        self.events.append((name, params))
        for callback in self.handlers.get(name, []):
            callback(*params)


def _rested_motion(printer, reactor):
    motion = Motion.__new__(Motion)
    motion.printer = printer
    motion.reactor = reactor
    motion.mcu = FakeMcu()
    motion.all_mcus = [motion.mcu]
    motion.motion_lead = MOTION_LEAD_S
    motion._planner_ready = True
    est_now = motion.mcu.estimated_print_time(NOW)
    motion.engine = StaleFrontierEngine(frontier=est_now - GAP_S)
    return motion


def _ready_idle_timeout(printer, reactor):
    it = IdleTimeout.__new__(IdleTimeout)
    it.printer = printer
    it.reactor = reactor
    it.idle_timeout = IDLE_TIMEOUT_S
    it.state = "Ready"
    it.last_print_start_systime = 0.0
    it.timeout_timer = reactor.register_timer(it.timeout_handler)
    printer.register_event_handler(
        "toolhead:sync_print_time", it.handle_sync_print_time
    )
    return it


def _first_activity_after_gap():
    printer = DispatchingPrinter()
    reactor = FakeReactor(now=NOW)
    it = _ready_idle_timeout(printer, reactor)
    motion = _rested_motion(printer, reactor)
    motion._sync_print_time()
    return motion, it, printer


def test_ready_check_is_armed_within_ready_timeout():
    _motion, it, _printer = _first_activity_after_gap()
    assert it.state == "Printing"
    _callback, waketime = it.timeout_timer
    assert waketime <= NOW + READY_TIMEOUT, waketime - NOW


def test_printing_event_is_not_scheduled_in_the_past():
    motion, _it, printer = _first_activity_after_gap()
    est_now = motion.mcu.estimated_print_time(NOW)
    printing = [
        p for name, p in printer.events if name == "idle_timeout:printing"
    ]
    assert len(printing) == 1
    (print_time,) = printing[0]
    assert print_time >= est_now, est_now - print_time
