import pytest

from klippy import motion


class FakeCommandError(Exception):
    pass


class FakePrinter:
    command_error = FakeCommandError

    def __init__(self, reactor, shutdown=False):
        self._reactor = reactor
        self._shutdown = shutdown

    def get_reactor(self):
        return self._reactor

    def is_shutdown(self):
        return self._shutdown


class FakeReactor:
    NOW = 0.0

    def __init__(self, step=0.5):
        self.now = 0.0
        self.step = step
        self.pauses = 0

    def monotonic(self):
        return self.now

    def pause(self, wake_time):
        self.pauses += 1
        self.now = max(wake_time, self.now + self.step)


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime


# Mirrors the bridge's entry gate: submit_move pushes into a fixed-capacity
# entry channel and reports full when the number of in-flight moves reaches the
# capacity. In-flight moves drain as the wall clock (reactor) advances;
# `stalled` pins them so the channel never frees.
class FakeEngine:
    def __init__(self, reactor, in_flight=0, capacity=64, stalled=False):
        self.reactor = reactor
        self._in_flight0 = in_flight
        self._t0 = reactor.now
        self._capacity = capacity
        self.stalled = stalled
        self.accepted = 0

    def _in_flight(self):
        if self.stalled:
            return self._in_flight0
        drained = int(self.reactor.now - self._t0)
        return max(0, self._in_flight0 - drained)

    def queued_motion_secs(self):
        return float(self._in_flight())

    def submit_move(self, dx, dy, dz, de, feedrate):
        if self._in_flight() >= self._capacity:
            return False
        self.accepted += 1
        return True

    def dispatched_lead_secs(self):
        return 0.0

    def get_last_move_time(self):
        return 0.0


class FakeMotion:
    _submit_paced = motion.Motion._submit_paced
    _yield_to_reactor_if_due = motion.Motion._yield_to_reactor_if_due

    def __init__(
        self,
        in_flight=0,
        mcu=True,
        drip=False,
        stalled=False,
        capacity=64,
        shutdown=False,
    ):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(self.reactor, shutdown=shutdown)
        self.mcu = FakeMcu() if mcu else None
        self.engine = FakeEngine(
            self.reactor,
            in_flight=in_flight,
            capacity=capacity,
            stalled=stalled,
        )
        self._drip_active = drip
        self._last_reactor_yield = 0.0

    def submit(self):
        self._submit_paced(self.engine.submit_move, 1.0, 0.0, 0.0, 0.0, 100.0)


def test_accepts_without_pause_when_pipe_has_space():
    m = FakeMotion(in_flight=1)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_periodic_reactor_yield_even_when_pipe_has_space():
    m = FakeMotion(in_flight=1)
    m._last_reactor_yield = -1.0  # last yield long ago => due for a yield
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 1


def test_retries_until_pipe_frees_space():
    # channel starts full; in-flight moves retire as the wall clock advances
    # with each reactor pause until the entry channel accepts.
    m = FakeMotion(in_flight=64)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses > 0


def test_drip_submits_without_pacing():
    m = FakeMotion(in_flight=1, drip=True)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_drip_fails_loud_when_pipe_full():
    m = FakeMotion(in_flight=64, drip=True)
    with pytest.raises(FakeCommandError):
        m.submit()
    assert m.reactor.pauses == 0


def test_no_mcu_submits_without_pacing():
    m = FakeMotion(in_flight=1, mcu=False)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_shutdown_breaks_the_wait():
    # an estop/shutdown during a full-pipe wait must break promptly; it is the
    # only thing that ends the wait besides the channel freeing.
    m = FakeMotion(in_flight=64, stalled=True, shutdown=True)
    with pytest.raises(FakeCommandError):
        m.submit()
    assert m.reactor.pauses == 0
