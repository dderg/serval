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
    def __init__(self, backlogs):
        self._backlogs = list(backlogs)
        self.calls = 0

    def pump_backlog(self):
        self.calls += 1
        idx = min(self.calls, len(self._backlogs)) - 1
        return self._backlogs[idx]


class FakeReactor:
    def __init__(self, time_per_pause=0.010):
        self.now = 0.0
        self.time_per_pause = time_per_pause
        self.pauses = 0

    def monotonic(self):
        return self.now

    def pause(self, wake_time):
        self.pauses += 1
        self.now = max(wake_time, self.now + self.time_per_pause)


class FakeMotion:
    _check_pause = motion.Motion._check_pause

    def __init__(
        self,
        backlogs,
        high=200,
        low=100,
        mcu=True,
        drip=False,
        time_per_pause=0.010,
    ):
        self.engine = FakeEngine(backlogs)
        self.reactor = FakeReactor(time_per_pause)
        self.printer = FakePrinter()
        self.mcu = mcu
        self._drip_active = drip
        self.pump_backlog_high = high
        self.pump_backlog_low = low


def test_no_pause_when_below_high():
    m = FakeMotion(backlogs=[50])
    m._check_pause()
    assert m.reactor.pauses == 0
    assert m.engine.calls == 1


def test_pauses_until_drained_to_low():
    m = FakeMotion(backlogs=[250, 150, 80])
    m._check_pause()
    assert m.reactor.pauses == 2


def test_inverted_watermarks_rejected():
    cfg = FakeConfig()
    with pytest.raises(FakeConfigError):
        motion.Motion._validate_pump_watermarks(cfg, 100, 50)


def test_valid_watermarks_accepted():
    cfg = FakeConfig()
    motion.Motion._validate_pump_watermarks(cfg, 100, 200)
    motion.Motion._validate_pump_watermarks(cfg, 200, 200)


def test_timeout_raises_when_backlog_never_drains():
    m = FakeMotion(backlogs=[250], time_per_pause=100.0)
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_skips_during_drip():
    m = FakeMotion(backlogs=[250], drip=True)
    m._check_pause()
    assert m.reactor.pauses == 0
    assert m.engine.calls == 0


def test_skips_when_no_mcu():
    m = FakeMotion(backlogs=[250], mcu=None)
    m._check_pause()
    assert m.reactor.pauses == 0
    assert m.engine.calls == 0
