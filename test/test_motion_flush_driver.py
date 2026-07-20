import pytest
from fakes import FakeConfig, FakePrinter

from klippy import engine_wait
from klippy.extras.output_pin import GCodeRequestQueue
from klippy.mcu import MCU
from klippy.motion import Motion, ToolheadShim

CLOCK_PER_PT = 1_000_000


class RecordingMcu:
    def __init__(self):
        self._flush_callbacks = []
        self.flush_calls = []

    def estimated_print_time(self, eventtime):
        return eventtime + 1.0

    def get_engine_handle(self):
        return 0

    def print_time_to_clock(self, print_time):
        return int(print_time * CLOCK_PER_PT)

    def register_flush_callback(self, callback):
        self._flush_callbacks.append(callback)

    def flush_moves(self, print_time, clear_history_time):
        self.flush_calls.append((print_time, clear_history_time))
        return MCU.flush_moves(self, print_time, clear_history_time)


class SyncReactor:
    NOW = 0.0
    NEVER = float("inf")

    def __init__(self):
        self._time = 100.0
        self._timers = {}

    def monotonic(self):
        return self._time

    def register_timer(self, callback, when=NEVER):
        self._timers[callback] = when
        return callback

    def update_timer(self, handle, when):
        if when == self.NOW or when <= self._time:
            self._timers[handle] = handle(self._time)
        else:
            self._timers[handle] = when


class FakeEngine:
    """Engine stub with the fence protocol: fence_start reports a full pipe
    (None) `full_starts` times, then each fence resolves with the absolute
    print_time of the queued-motion end after `polls_until_resolved` poll
    attempts."""

    def __init__(
        self,
        mcu,
        reactor,
        queued_secs=0.0,
        polls_until_resolved=0,
        full_starts=0,
    ):
        self._mcu = mcu
        self._reactor = reactor
        self.queued_secs = queued_secs
        self.polls_until_resolved = polls_until_resolved
        self.full_starts = full_starts
        self.fences = {}
        self._next_fence = 1

    def queued_motion_secs(self):
        return self.queued_secs

    def fence_start(self, force):
        if self.full_starts > 0:
            self.full_starts -= 1
            return None
        fence_id = self._next_fence
        self._next_fence += 1
        self.fences[fence_id] = self.polls_until_resolved
        return fence_id

    def fence_print_time_poll(self, fence_id, mcu_handle):
        remaining = self.fences[fence_id]
        if remaining > 0:
            self.fences[fence_id] = remaining - 1
            return None
        del self.fences[fence_id]
        est = self._mcu.estimated_print_time(self._reactor.monotonic())
        return est + self.queued_secs


def _make_motion(
    mcus, reactor, queued_secs=0.0, polls_until_resolved=0, full_starts=0
):
    motion = Motion.__new__(Motion)
    motion.reactor = reactor
    motion.all_mcus = mcus
    motion.mcu = mcus[0]
    motion.kin = None
    motion.engine = FakeEngine(
        mcus[0], reactor, queued_secs, polls_until_resolved, full_starts
    )
    motion.motion_lead = 0.25
    motion.need_flush_time = 0.0
    motion.do_kick_flush_timer = True
    motion.flush_timer = reactor.register_timer(motion._flush_handler)
    motion._lookahead_fences = []
    motion._lookahead_fence_timer = reactor.register_timer(
        motion._lookahead_fence_handler
    )
    motion._engine_wakeup = None
    return motion


def test_flush_moves_fires_callbacks_with_print_time_and_clock():
    mcu = RecordingMcu()
    seen = []
    mcu.register_flush_callback(lambda pt, clock: seen.append((pt, clock)))
    MCU.flush_moves(mcu, 12.0, 12.0)
    assert seen == [(12.0, 12 * CLOCK_PER_PT)]


def test_flush_moves_skips_on_negative_clock():
    mcu = RecordingMcu()
    seen = []
    mcu.register_flush_callback(lambda pt, clock: seen.append((pt, clock)))
    MCU.flush_moves(mcu, -1.0, -1.0)
    assert seen == []


def test_note_kicks_timer_and_flushes_all_mcus():
    reactor = SyncReactor()
    mcus = [RecordingMcu(), RecordingMcu()]
    motion = _make_motion(mcus, reactor)

    motion.advance_flush_time(105.0)

    assert mcus[0].flush_calls == [(105.0, 105.0)]
    assert mcus[1].flush_calls == [(105.0, 105.0)]
    assert motion.do_kick_flush_timer is True
    assert motion.need_flush_time == 105.0


def test_handler_converges_when_callback_rebumps_need_flush_time():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion([mcu], reactor)

    bumps = {"n": 0}

    def rebumping_cb(print_time, clock):
        if bumps["n"] == 0:
            bumps["n"] += 1
            motion.advance_flush_time(print_time + 0.1)

    mcu.register_flush_callback(rebumping_cb)
    motion.advance_flush_time(105.0)

    assert [pt for pt, _ in mcu.flush_calls] == [105.0, pytest.approx(105.1)]
    assert motion.do_kick_flush_timer is True


def test_idle_handler_returns_never_and_rekicks():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion([mcu], reactor)

    assert motion._flush_handler(100.0) == reactor.NEVER
    assert motion.do_kick_flush_timer is True

    motion.advance_flush_time(110.0)
    assert mcu.flush_calls[-1] == (110.0, 110.0)


def test_raising_flush_callback_invokes_shutdown_not_reactor_crash():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion([mcu], reactor)

    shutdowns = []

    class RecordingPrinter:
        def invoke_shutdown(self, msg):
            shutdowns.append(msg)

    motion.printer = RecordingPrinter()
    mcu.register_flush_callback(
        lambda pt, clock: (_ for _ in ()).throw(RuntimeError("boom"))
    )

    motion.need_flush_time = 105.0
    assert motion._flush_handler(100.0) == reactor.NEVER
    assert len(shutdowns) == 1


def test_m106_queued_request_drains_on_idle_printer():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion([mcu], reactor)
    shim = ToolheadShim(motion)

    applied = []
    gcrq = GCodeRequestQueue(
        FakeConfig(FakePrinter(reactor=reactor)),
        mcu,
        lambda print_time, value: applied.append((print_time, value)),
    )
    gcrq.toolhead = shim

    gcrq.queue_gcode_request(1.0)

    assert len(applied) == 1
    print_time, value = applied[0]
    assert value == 1.0
    est = mcu.estimated_print_time(reactor.monotonic())
    assert print_time == pytest.approx(est + motion.motion_lead)


def test_lookahead_callback_schedules_at_engine_queued_end_while_printing():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    # 1.5s of motion queued ahead; the host min_move_t counter lags at 0.
    motion = _make_motion([mcu], reactor, queued_secs=1.5)
    shim = ToolheadShim(motion)

    est = mcu.estimated_print_time(reactor.monotonic())
    seen = []
    shim.register_lookahead_callback(seen.append)

    # Scheduled at the fence-resolved queue end (est + 1.5), not the
    # schedule_floor (est + motion_lead) that fired the fan early.
    assert seen == [pytest.approx(est + 1.5)]
    assert seen[0] > est + motion.motion_lead


def test_lookahead_callback_waits_for_the_fence_to_resolve():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion(
        [mcu], reactor, queued_secs=2.0, polls_until_resolved=1
    )
    shim = ToolheadShim(motion)

    seen = []
    shim.register_lookahead_callback(seen.append)
    assert seen == [], "callback must not fire before the fence resolves"

    waketime = motion._lookahead_fence_handler(reactor.monotonic())
    est = mcu.estimated_print_time(reactor.monotonic())
    assert seen == [pytest.approx(est + 2.0)]
    assert waketime == reactor.NEVER


def test_lookahead_callback_survives_a_full_move_channel():
    # Regression: fence_start returning None (move channel at capacity) must
    # never block the reactor — registration parks the callback and the poll
    # timer retries the start until the pipe admits the fence.
    reactor = SyncReactor()
    mcu = RecordingMcu()
    # 3 rejections: one eaten at registration, one by the timer kick that
    # registration fires, one by the first explicit handler call below.
    motion = _make_motion([mcu], reactor, queued_secs=1.0, full_starts=3)
    shim = ToolheadShim(motion)

    seen = []
    shim.register_lookahead_callback(seen.append)
    assert seen == []
    assert motion._lookahead_fences[0][0] is None

    waketime = motion._lookahead_fence_handler(reactor.monotonic())
    assert seen == []
    assert waketime == reactor.monotonic() + engine_wait.PARK_FALLBACK_S

    waketime = motion._lookahead_fence_handler(reactor.monotonic())
    est = mcu.estimated_print_time(reactor.monotonic())
    assert seen == [pytest.approx(est + 1.0)]
    assert waketime == reactor.NEVER


def test_blocking_fence_wait_retries_start_and_yields():
    reactor = SyncReactor()
    mcu = RecordingMcu()
    motion = _make_motion([mcu], reactor, queued_secs=3.0, full_starts=3)

    class NeverShutdownPrinter:
        command_error = RuntimeError

        def get_reactor(self):
            return reactor

        def is_shutdown(self):
            return False

    motion.printer = NeverShutdownPrinter()
    pauses = []
    reactor.pause = pauses.append
    est = mcu.estimated_print_time(reactor.monotonic())
    assert motion.get_last_move_time() == pytest.approx(est + 3.0)
    assert len(pauses) >= 3, "full channel must yield to the reactor, not spin"
