"""#989: a gcode wait (M109, TEMPERATURE_WAIT) declares itself to the engine,
so the pacer brake on the motion queued before it is logged as that wait's
planned stop instead of a starved producer."""

from fakes import FakeEngine, FakePrinter, FakeReactor

from klippy.motion import Motion
from klippy.printer import Printer

WAIT_BEGIN = "printer:wait_begin"


class InterruptCounter:
    def get_interrupt_counter(self):
        return 0


class WaitingPrinter:
    wait_while = Printer.wait_while

    def __init__(self):
        self.reactor = FakeReactor()
        self.events = []

    def lookup_object(self, name):
        assert name == "gcode"
        return InterruptCounter()

    def is_shutdown(self):
        return False

    def send_event(self, event, *params):
        self.events.append(event)


def _checks(*results):
    remaining = list(results)
    return lambda eventtime: remaining.pop(0)


def test_a_wait_that_waits_declares_itself_once():
    printer = WaitingPrinter()
    printer.wait_while(_checks(True, True, False))
    assert printer.events == [WAIT_BEGIN]


def test_a_condition_already_met_declares_no_wait():
    printer = WaitingPrinter()
    printer.wait_while(_checks(False))
    assert printer.events == []


def _motion(planner_ready, declare_results):
    motion = Motion.__new__(Motion)
    motion.reactor = FakeReactor()
    motion.printer = FakePrinter(reactor=motion.reactor)
    motion.engine = FakeEngine(declare_wait=list(declare_results))
    motion._planner_ready = planner_ready
    motion._last_reactor_yield = 0.0
    motion._engine_wakeup = None
    return motion


def test_a_declared_wait_reaches_the_engine():
    motion = _motion(planner_ready=True, declare_results=[True])
    motion._declare_wait()
    assert motion.engine.calls == [("declare_wait",)]


def test_a_declared_wait_waits_out_a_full_pipe_instead_of_dropping():
    motion = _motion(planner_ready=True, declare_results=[False, False, True])
    motion._declare_wait()
    assert motion.engine.calls == [("declare_wait",)] * 3


def test_no_wait_is_declared_before_the_planner_runs():
    motion = _motion(planner_ready=False, declare_results=[])
    motion._declare_wait()
    assert motion.engine.calls == []
