import pytest

from klippy import motion


class FakeCommandError(Exception):
    pass


class FakePrinter:
    command_error = FakeCommandError


class FakeEngine:
    def __init__(self, buffer_secs=0.0, drain_per_call=0.0):
        self.buffer_secs = buffer_secs
        self.drain_per_call = drain_per_call

    def queued_motion_secs(self):
        current = self.buffer_secs
        self.buffer_secs = max(self.buffer_secs - self.drain_per_call, 0.0)
        return current

    def get_last_move_time(self):
        return self.buffer_secs


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


class FakeMotion:
    _check_pause = motion.Motion._check_pause
    _yield_to_reactor_if_due = motion.Motion._yield_to_reactor_if_due

    def __init__(
        self,
        buffer_secs=0.0,
        drain_per_call=0.0,
        mcu=True,
        drip=False,
        buffer_time_high=2.0,
        buffer_time_low=1.0,
    ):
        self.reactor = FakeReactor()
        self.engine = FakeEngine(buffer_secs, drain_per_call)
        self.printer = FakePrinter()
        self.mcu = FakeMcu() if mcu else None
        self._drip_active = drip
        self.buffer_time_high = buffer_time_high
        self.buffer_time_low = buffer_time_low
        self._last_reactor_yield = 0.0


def test_no_pause_when_within_buffer():
    m = FakeMotion(buffer_secs=1.0)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_periodic_reactor_yield_even_when_within_buffer():
    m = FakeMotion(buffer_secs=1.0)
    m._last_reactor_yield = -1.0  # last yield long ago => due for a yield
    m._check_pause()
    assert m.reactor.pauses == 1


def test_pauses_until_buffer_drains_to_low():
    m = FakeMotion(buffer_secs=5.0, drain_per_call=1.0)
    m._check_pause()
    assert m.reactor.pauses > 0
    assert m.engine.queued_motion_secs() <= m.buffer_time_low


def test_timeout_when_buffer_never_drains():
    m = FakeMotion(buffer_secs=5.0, drain_per_call=0.0)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_skips_during_drip():
    m = FakeMotion(buffer_secs=5.0, drip=True)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_skips_when_no_mcu():
    m = FakeMotion(buffer_secs=5.0, mcu=False)
    m._check_pause()
    assert m.reactor.pauses == 0
