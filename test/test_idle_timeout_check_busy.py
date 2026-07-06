from klippy import motion
from klippy.extras.idle_timeout import IdleTimeout


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime


class FakeEngine:
    def __init__(self, queued_secs):
        self.queued_secs = queued_secs

    def queued_motion_secs(self):
        return self.queued_secs


class FakeToolhead:
    check_busy = motion.Motion.check_busy

    def __init__(self, pending_end_time, queued_secs=0.0):
        self.mcu = FakeMcu()
        self.engine = FakeEngine(queued_secs)
        self._mcu_pending_end_time = pending_end_time

    def get_last_move_time(self):
        return self._mcu_pending_end_time


def test_busy_while_motion_is_queued_in_the_engine():
    th = FakeToolhead(pending_end_time=0.0, queued_secs=3.0)
    print_time, est, lookahead_empty = th.check_busy(1000.0)
    assert print_time == 1003.0
    assert not lookahead_empty


def test_busy_while_dispatched_motion_still_executes_on_the_mcu():
    th = FakeToolhead(pending_end_time=1005.0)
    print_time, est, lookahead_empty = th.check_busy(1000.0)
    assert print_time == 1005.0
    assert not lookahead_empty


def test_idle_time_grows_after_motion_drains():
    # Regression: print_time used to be clamped up to est_print_time when the
    # queue was empty, so est - print_time was permanently 0 and idle_timeout
    # could never elapse (servos/steppers stayed powered forever).
    th = FakeToolhead(pending_end_time=500.0)
    print_time, est, lookahead_empty = th.check_busy(1000.0)
    assert lookahead_empty
    assert print_time == 500.0
    assert est - print_time == 500.0
    print_time_later, est_later, _ = th.check_busy(1200.0)
    assert est_later - print_time_later == 700.0


class FakeMutex:
    def test(self):
        return False


class FakeGcode:
    def __init__(self):
        self.scripts = []

    def get_mutex(self):
        return FakeMutex()

    def run_script(self, script):
        self.scripts.append(script)


class FakeTemplate:
    def render(self):
        return "M84"


class FakePrinter:
    def __init__(self):
        self.events = []

    def is_shutdown(self):
        return False

    def send_event(self, name, *args):
        self.events.append(name)


class FakeReactor:
    NEVER = float("inf")


def make_idle_timeout(toolhead):
    it = IdleTimeout.__new__(IdleTimeout)
    it.printer = FakePrinter()
    it.reactor = FakeReactor()
    it.gcode = FakeGcode()
    it.toolhead = toolhead
    it.idle_timeout = 600.0
    it.idle_gcode = FakeTemplate()
    it.state = "Printing"
    return it


def test_idle_timeout_reaches_idle_and_runs_motor_off_gcode():
    # Motion ended at print_time 500; the est clock keeps advancing.
    it = make_idle_timeout(FakeToolhead(pending_end_time=500.0))
    it.timeout_handler(1000.0)
    assert it.state == "Ready"
    waketime = it.timeout_handler(1101.0)
    assert it.state == "Idle"
    assert waketime == it.reactor.NEVER
    assert it.gcode.scripts == ["M84"]
    assert "idle_timeout:idle" in it.printer.events


def test_idle_timeout_waits_out_the_configured_timeout():
    it = make_idle_timeout(FakeToolhead(pending_end_time=500.0))
    it.timeout_handler(1000.0)
    assert it.state == "Ready"
    it.timeout_handler(1050.0)
    assert it.state == "Ready"
    assert it.gcode.scripts == []
