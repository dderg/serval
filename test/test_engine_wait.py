import pytest

from klippy import engine_wait


class FakeCommandError(Exception):
    pass


class FakeReactor:
    def __init__(self):
        self.now = 0.0
        self.pauses = []

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.pauses.append(waketime)
        self.now = max(self.now, waketime)


class FakePrinter:
    command_error = FakeCommandError

    def __init__(self, shutdown_after_pauses=None):
        self.reactor = FakeReactor()
        self._shutdown_after_pauses = shutdown_after_pauses

    def get_reactor(self):
        return self.reactor

    def is_shutdown(self):
        if self._shutdown_after_pauses is None:
            return False
        return len(self.reactor.pauses) >= self._shutdown_after_pauses


def test_returns_result_without_pausing_when_immediately_done():
    printer = FakePrinter()
    result = engine_wait.wait_for(
        printer, lambda: 3.5, "immediate", engine_wait.UNBOUNDED
    )
    assert result == 3.5
    assert printer.reactor.pauses == []


def test_falsy_non_none_results_complete_the_wait():
    printer = FakePrinter()
    result = engine_wait.wait_for(
        printer, lambda: 0.0, "zero lead", engine_wait.UNBOUNDED
    )
    assert result == 0.0


def test_polls_until_done_at_requested_interval():
    printer = FakePrinter()
    results = iter([None, None, "done"])
    result = engine_wait.wait_for(
        printer,
        lambda: next(results),
        "third try",
        engine_wait.UNBOUNDED,
        interval_s=0.25,
    )
    assert result == "done"
    assert printer.reactor.pauses == [0.25, 0.5]


def test_timeout_raises_engine_wait_timeout():
    printer = FakePrinter()
    with pytest.raises(engine_wait.EngineWaitTimeout, match="never done"):
        engine_wait.wait_for(
            printer, lambda: None, "never done", 1.0, interval_s=0.3
        )


def test_shutdown_aborts_the_wait():
    printer = FakePrinter(shutdown_after_pauses=2)
    with pytest.raises(FakeCommandError, match="shutdown while waiting"):
        engine_wait.wait_for(
            printer, lambda: None, "stuck", engine_wait.UNBOUNDED
        )


def test_slow_wait_emits_structured_log_events(monkeypatch):
    events = []
    monkeypatch.setattr(
        engine_wait.structured_log,
        "event",
        lambda subsystem, event, **fields: events.append((event, fields)),
    )
    printer = FakePrinter()
    results = iter([None, None, "done"])
    engine_wait.wait_for(
        printer,
        lambda: next(results),
        "slow op",
        engine_wait.UNBOUNDED,
        interval_s=engine_wait.SLOW_WAIT_LOG_S,
    )
    assert [name for name, _ in events] == [
        "engine_wait_slow",
        "engine_wait_done",
    ]
    assert all(fields["what"] == "slow op" for _, fields in events)
