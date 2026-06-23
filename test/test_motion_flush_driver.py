import pytest

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


def _make_motion(mcus, reactor):
    motion = Motion.__new__(Motion)
    motion.reactor = reactor
    motion.all_mcus = mcus
    motion.mcu = mcus[0]
    motion.motion_lead = 0.25
    motion._mcu_pending_end_time = 0.0
    motion.need_flush_time = 0.0
    motion.do_kick_flush_timer = True
    motion.flush_timer = reactor.register_timer(motion._flush_handler)
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

    class FakePrinter:
        def get_reactor(self):
            return reactor

        def register_event_handler(self, event, handler):
            pass

    class FakeConfig:
        def __init__(self, printer):
            self._printer = printer

        def get_printer(self):
            return self._printer

    applied = []
    gcrq = GCodeRequestQueue(
        FakeConfig(FakePrinter()),
        mcu,
        lambda print_time, value: applied.append((print_time, value)),
    )
    gcrq.toolhead = shim

    gcrq.queue_gcode_request(1.0)

    assert len(applied) == 1
    print_time, value = applied[0]
    assert value == 1.0
    assert print_time == pytest.approx(motion.get_last_move_time())
