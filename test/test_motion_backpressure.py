import pytest

from klippy import motion


class FakeCommandError(Exception):
    pass


class FakeConfigError(Exception):
    pass


class FakeConfig:
    def error(self, msg):
        return FakeConfigError(msg)


class FakePrinter:
    command_error = FakeCommandError


class FakeEngine:
    def __init__(self, backlog=0):
        self._backlog = backlog

    def pump_backlog(self):
        return self._backlog


class FakeReactor:
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
    # estimated_print_time advances with wall time at `rate` (rate=0 => stalled).
    def __init__(self, rate=1.0):
        self.rate = rate

    def estimated_print_time(self, eventtime):
        return eventtime * self.rate


class FakeMotion:
    _check_pause = motion.Motion._check_pause

    def __init__(
        self,
        frontier=0.0,
        backlog=0,
        rate=1.0,
        mcu=True,
        drip=False,
        buffer_time_high=2.0,
        buffer_time_low=1.0,
        pump_backlog_high=200,
        pump_backlog_low=100,
    ):
        self.reactor = FakeReactor()
        self.engine = FakeEngine(backlog)
        self.printer = FakePrinter()
        self.mcu = FakeMcu(rate) if mcu else None
        self._mcu_pending_end_time = frontier
        self._drip_active = drip
        self.buffer_time_high = buffer_time_high
        self.buffer_time_low = buffer_time_low
        self.pump_backlog_high = pump_backlog_high
        self.pump_backlog_low = pump_backlog_low


def test_no_pause_when_within_buffer():
    m = FakeMotion(frontier=1.0, backlog=0)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_pauses_until_buffer_drains_to_low():
    m = FakeMotion(frontier=5.0, backlog=0)
    m._check_pause()
    assert m.reactor.pauses > 0
    # drained to <= buffer_time_low: est (== reactor.now at rate 1) >= 4.0
    assert m._mcu_pending_end_time - m.reactor.now <= m.buffer_time_low


def test_pauses_on_pump_backlog_even_when_buffer_ok():
    m = FakeMotion(frontier=0.0, backlog=300, pump_backlog_high=200)
    # backlog never drains in this fake -> must time out, proving it engaged
    with pytest.raises(FakeCommandError):
        m._check_pause()
    assert m.reactor.pauses > 0


def test_timeout_when_mcu_not_advancing():
    m = FakeMotion(frontier=5.0, backlog=0, rate=0.0)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_skips_during_drip():
    m = FakeMotion(frontier=5.0, backlog=300, drip=True)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_skips_when_no_mcu():
    m = FakeMotion(frontier=5.0, backlog=300, mcu=False)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_inverted_watermarks_rejected():
    cfg = FakeConfig()
    with pytest.raises(FakeConfigError):
        motion.Motion._validate_pump_watermarks(cfg, 100, 50)


def test_valid_watermarks_accepted():
    cfg = FakeConfig()
    motion.Motion._validate_pump_watermarks(cfg, 100, 200)
    motion.Motion._validate_pump_watermarks(cfg, 200, 200)
