import pytest
from fakes import FakeCommandError, FakePrinter

from klippy import engine_wait


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
    printer = FakePrinter()
    calls = [0]

    def poll():
        calls[0] += 1
        if calls[0] > 2:
            printer.invoke_shutdown("test shutdown")
        return None

    with pytest.raises(FakeCommandError, match="shutdown while waiting"):
        engine_wait.wait_for(printer, poll, "stuck", engine_wait.UNBOUNDED)


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
