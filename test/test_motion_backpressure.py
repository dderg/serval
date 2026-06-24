import pytest

from klippy import motion


class FakeCommandError(Exception):
    pass


class FakePrinter:
    command_error = FakeCommandError

    def __init__(self, shutdown=False):
        self._shutdown = shutdown

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


# Mirrors bridge.queued_motion_secs and pending_channel_moves: each signal
# drains as the wall clock (reactor) advances. `*_stalled` pins a signal so it
# never drains -- the DRAIN_TIMEOUT guard case.
class FakeEngine:
    def __init__(
        self,
        reactor,
        queued=0.0,
        frontier=0.0,
        stalled=False,
        channel=0,
        channel_stalled=False,
        channel_rate=10000.0,
    ):
        self.reactor = reactor
        self._queued0 = queued
        self._t0 = reactor.now
        self._frontier = frontier
        self.stalled = stalled
        self._channel0 = channel
        self.channel_stalled = channel_stalled
        self._channel_rate = channel_rate

    def queued_motion_secs(self):
        if self.stalled:
            return self._queued0
        drained = self.reactor.now - self._t0
        return max(0.0, self._queued0 - drained)

    def pending_channel_moves(self):
        if self.channel_stalled:
            return self._channel0
        drained = int((self.reactor.now - self._t0) * self._channel_rate)
        return max(0, self._channel0 - drained)

    def dispatched_lead_secs(self):
        return 0.0

    def uncommitted_intake_secs(self):
        return 0.0

    def get_last_move_time(self):
        return self._frontier


class FakeMotion:
    _check_pause = motion.Motion._check_pause
    _yield_to_reactor_if_due = motion.Motion._yield_to_reactor_if_due

    def __init__(
        self,
        queued=0.0,
        mcu=True,
        drip=False,
        stalled=False,
        buffer_time_high=2.0,
        buffer_time_low=1.0,
        channel=0,
        channel_stalled=False,
        channel_high=6144,
        channel_low=4096,
        shutdown=False,
    ):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(shutdown=shutdown)
        self.mcu = FakeMcu() if mcu else None
        self.engine = FakeEngine(
            self.reactor,
            queued=queued,
            stalled=stalled,
            channel=channel,
            channel_stalled=channel_stalled,
        )
        self._drip_active = drip
        self.buffer_time_high = buffer_time_high
        self.buffer_time_low = buffer_time_low
        self._channel_high = channel_high
        self._channel_low = channel_low
        self._last_reactor_yield = 0.0


def test_no_pause_when_within_buffer():
    m = FakeMotion(queued=1.0)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_periodic_reactor_yield_even_when_within_buffer():
    m = FakeMotion(queued=1.0)
    m._last_reactor_yield = -1.0  # last yield long ago => due for a yield
    m._check_pause()
    assert m.reactor.pauses == 1


def test_pauses_until_buffer_drains_to_low():
    # queued starts above the high watermark; it drains as the wall clock
    # advances with each reactor pause until it reaches the low watermark.
    m = FakeMotion(queued=5.0)
    m._check_pause()
    assert m.reactor.pauses > 0
    assert m.engine.queued_motion_secs() <= m.buffer_time_low


def test_timeout_when_signal_never_drains():
    # frontier races the host clock => queued pinned above low => DRAIN_TIMEOUT.
    m = FakeMotion(queued=5.0, stalled=True)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_skips_during_drip():
    m = FakeMotion(queued=5.0, drip=True)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_skips_when_no_mcu():
    m = FakeMotion(queued=5.0, mcu=False)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_no_pause_when_channel_under_high():
    # in-channel moves below the high watermark, time signal idle => no throttle.
    m = FakeMotion(queued=0.0, channel=6143)
    m._check_pause()
    assert m.reactor.pauses == 0


def test_pauses_until_channel_drains_to_low():
    # channel occupancy above high with the time signal idle still throttles --
    # this is the overflow-crash path the time gate alone was blind to.
    m = FakeMotion(queued=0.0, channel=7000)
    m._check_pause()
    assert m.reactor.pauses > 0
    assert m.engine.pending_channel_moves() <= m._channel_low


def test_timeout_when_channel_never_drains():
    # planner not pulling => channel pinned above low => DRAIN_TIMEOUT fail-loud.
    m = FakeMotion(queued=0.0, channel=7000, channel_stalled=True)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m._check_pause()


def test_shutdown_breaks_channel_drain():
    # an estop/shutdown during a channel-backpressure wait must break promptly,
    # not spin to DRAIN_TIMEOUT.
    m = FakeMotion(
        queued=0.0, channel=7000, channel_stalled=True, shutdown=True
    )
    with pytest.raises(FakeCommandError):
        m._check_pause()
    assert m.reactor.pauses == 0
