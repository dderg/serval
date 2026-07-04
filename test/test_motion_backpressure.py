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


# Mirrors the bridge's entry gate: submit_move accepts while queued motion is
# below the pipe depth and reports full otherwise. Queued motion drains as the
# wall clock (reactor) advances; `stalled` pins it so it never drains -- the
# DRAIN_TIMEOUT guard case.
class FakeEngine:
    def __init__(self, reactor, queued=0.0, depth=2.0, stalled=False):
        self.reactor = reactor
        self._queued0 = queued
        self._t0 = reactor.now
        self._depth = depth
        self.stalled = stalled
        self.accepted = 0

    def queued_motion_secs(self):
        if self.stalled:
            return self._queued0
        drained = self.reactor.now - self._t0
        return max(0.0, self._queued0 - drained)

    def submit_move(self, dx, dy, dz, de, feedrate):
        if self.queued_motion_secs() >= self._depth:
            return False
        self.accepted += 1
        return True

    def dispatched_lead_secs(self):
        return 0.0

    def uncommitted_intake_secs(self):
        return 0.0

    def get_last_move_time(self):
        return 0.0


class FakeMotion:
    _submit_paced = motion.Motion._submit_paced
    _yield_to_reactor_if_due = motion.Motion._yield_to_reactor_if_due

    def __init__(
        self,
        queued=0.0,
        mcu=True,
        drip=False,
        stalled=False,
        depth=2.0,
        shutdown=False,
    ):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(shutdown=shutdown)
        self.mcu = FakeMcu() if mcu else None
        self.engine = FakeEngine(
            self.reactor, queued=queued, depth=depth, stalled=stalled
        )
        self._drip_active = drip
        self._last_reactor_yield = 0.0

    def submit(self):
        self._submit_paced(self.engine.submit_move, 1.0, 0.0, 0.0, 0.0, 100.0)


def test_accepts_without_pause_when_pipe_has_space():
    m = FakeMotion(queued=1.0)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_periodic_reactor_yield_even_when_pipe_has_space():
    m = FakeMotion(queued=1.0)
    m._last_reactor_yield = -1.0  # last yield long ago => due for a yield
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 1


def test_retries_until_pipe_frees_space():
    # queued starts above the pipe depth; it drains as the wall clock advances
    # with each reactor pause until the entry gate accepts.
    m = FakeMotion(queued=5.0)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses > 0


def test_timeout_when_pipe_never_frees():
    # playhead stalled => queued pinned at the depth => DRAIN_TIMEOUT fail-loud.
    m = FakeMotion(queued=5.0, stalled=True)
    m.reactor.step = 10.0
    with pytest.raises(FakeCommandError):
        m.submit()
    assert m.engine.accepted == 0


def test_drip_submits_without_pacing():
    m = FakeMotion(queued=1.0, drip=True)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_drip_fails_loud_when_pipe_full():
    m = FakeMotion(queued=5.0, drip=True)
    with pytest.raises(FakeCommandError):
        m.submit()
    assert m.reactor.pauses == 0


def test_no_mcu_submits_without_pacing():
    m = FakeMotion(queued=1.0, mcu=False)
    m.submit()
    assert m.engine.accepted == 1
    assert m.reactor.pauses == 0


def test_shutdown_breaks_the_wait():
    # an estop/shutdown during a full-pipe wait must break promptly, not spin
    # to DRAIN_TIMEOUT.
    m = FakeMotion(queued=5.0, stalled=True, shutdown=True)
    with pytest.raises(FakeCommandError):
        m.submit()
    assert m.reactor.pauses == 0
