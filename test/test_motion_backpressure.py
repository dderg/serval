import pytest

from klippy import motion


class FakeCommandError(Exception):
    pass


class FakePrinter:
    command_error = FakeCommandError


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
    def __init__(self, rate=1.0):
        self.rate = rate

    def estimated_print_time(self, eventtime):
        return eventtime * self.rate


class FakeMotion:
    _check_pause = motion.Motion._check_pause
    _yield_to_reactor_if_due = motion.Motion._yield_to_reactor_if_due

    def __init__(
        self,
        pending_end=0.0,
        rate=1.0,
        mcu=True,
        drip=False,
        buffer_time_high=2.0,
        buffer_time_low=1.0,
    ):
        self.reactor = FakeReactor()
        self.printer = FakePrinter()
        self.mcu = FakeMcu(rate) if mcu else None
        self._mcu_pending_end_time = pending_end
        self._drip_active = drip
        self.buffer_time_high = buffer_time_high
        self.buffer_time_low = buffer_time_low
        self._last_reactor_yield = 0.0


def test_no_pause_when_within_buffer():
    m = FakeMotion(pending_end=1.0)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_periodic_reactor_yield_even_when_within_buffer():
    m = FakeMotion(pending_end=1.0)
    m._last_reactor_yield = -1.0  # last yield long ago => due for a yield
    m._check_pause()
    assert m.reactor.pauses == 1


def test_pauses_until_buffer_drains_to_low():
    # pending_end fixed; est advances with each reactor pause, draining buffer.
    m = FakeMotion(pending_end=5.0)
    m._check_pause()
    assert m.reactor.pauses > 0
    est = m.mcu.estimated_print_time(m.reactor.now)
    assert m._mcu_pending_end_time - est <= m.buffer_time_low


def test_timeout_when_mcu_not_advancing():
    # rate=0 => est frozen => buffer never drains => DRAIN_TIMEOUT raises.
    m = FakeMotion(pending_end=5.0, rate=0.0)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_skips_during_drip():
    m = FakeMotion(pending_end=5.0, drip=True)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_skips_when_no_mcu():
    m = FakeMotion(pending_end=5.0, mcu=False)
    m._check_pause()
    assert m.reactor.pauses == 0
