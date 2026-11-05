import types

import pytest
from fakes import FakeCommandError, FakePrinter, FakeReactor

from klippy.motion import Motion


class _Clocksync:
    def __init__(self, reactor, synced_at):
        self._reactor = reactor
        self.synced_at = synced_at

    def is_synced(self):
        return self.synced_at is not None and self._reactor.now >= self.synced_at


class _Mcu:
    def __init__(self, name, clocksync):
        self._name = name
        self._clocksync = clocksync
        self.non_critical_disconnected = False

    def get_name(self):
        return self._name

    def get_clocksync(self):
        return self._clocksync

    def is_fileoutput(self):
        return False


class _Gcode:
    def __init__(self):
        self.info = []

    def respond_info(self, msg):
        self.info.append(msg)


def _motion(synced_at):
    reactor = FakeReactor()
    gcode = _Gcode()
    printer = FakePrinter(objects={"gcode": gcode}, reactor=reactor)
    mcu = _Mcu("mcu", _Clocksync(reactor, synced_at))
    motion = types.SimpleNamespace(
        printer=printer,
        reactor=reactor,
        mcu=mcu,
        all_mcus=[mcu],
        CLOCK_SYNC_TIMEOUT=Motion.CLOCK_SYNC_TIMEOUT,
    )
    return motion, gcode, reactor


def test_waits_for_convergence_and_says_how_long_it_may_take():
    motion, gcode, reactor = _motion(synced_at=12.0)
    Motion.await_clock_sync(motion)
    assert reactor.now >= 12.0
    assert gcode.info == [
        "Waiting for MCU clock synchronization (mcu: mcu), up to 60 s..."
    ]


def test_already_converged_does_not_wait():
    motion, gcode, reactor = _motion(synced_at=0.0)
    Motion.await_clock_sync(motion)
    assert reactor.pauses == []
    assert gcode.info == []


def test_no_convergence_within_the_bound_is_a_command_error():
    motion, _, reactor = _motion(synced_at=None)
    with pytest.raises(FakeCommandError) as excinfo:
        Motion.await_clock_sync(motion)
    assert str(excinfo.value) == (
        "MCU clock synchronization did not converge within 60 s (mcu: mcu);"
        " check the MCU connection, then retry or FIRMWARE_RESTART"
    )
    assert reactor.now <= 60.0 + 0.2


def test_a_reconnect_after_convergence_waits_again():
    motion, gcode, reactor = _motion(synced_at=0.0)
    Motion.await_clock_sync(motion)
    motion.mcu.get_clocksync().synced_at = 5.0
    reactor.now = 1.0
    Motion.await_clock_sync(motion)
    assert reactor.now >= 5.0
    assert len(gcode.info) == 1


def test_a_disconnected_non_critical_mcu_is_not_waited_for():
    motion, gcode, _ = _motion(synced_at=None)
    motion.mcu.non_critical_disconnected = True
    Motion.await_clock_sync(motion)
    assert gcode.info == []


def _gated(calls):
    engine = types.SimpleNamespace(
        submit_nudge=lambda *args: calls.append("engine.submit_nudge")
    )
    return types.SimpleNamespace(
        engine=engine,
        await_clock_sync=lambda: calls.append("await_clock_sync"),
    )


def test_nudge_awaits_clock_sync_before_reaching_the_engine():
    calls = []
    Motion.submit_nudge(_gated(calls), 0, 0, 0, 0.1, 5.0, 100.0)
    assert calls == ["await_clock_sync", "engine.submit_nudge"]


def test_resonance_buzz_awaits_clock_sync_before_submitting(monkeypatch):
    from klippy.extras import resonance_buzz

    calls = []
    monkeypatch.setattr(
        resonance_buzz,
        "submit_buzz",
        lambda motion, axis_mask, sign_mask, wave: calls.append("submit_buzz"),
    )
    Motion.submit_resonance_buzz(_gated(calls), 1, 0, [])
    assert calls == ["await_clock_sync", "submit_buzz"]
