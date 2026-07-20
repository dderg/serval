import logging

import pytest
from fakes import FakeConfig, FakeGcode, FakePrinter, FakeReactor

from klippy import structured_log
from klippy.extras.print_stats import PrintStats


class CaptureHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


class FakePosition:
    def __init__(self, e=0.0):
        self.e = e


class FakeGCodeMove:
    def __init__(self):
        self.epos = 0.0
        self.extrude_factor = 1.0

    def get_status(self, eventtime):
        return {
            "position": FakePosition(self.epos),
            "extrude_factor": self.extrude_factor,
        }


@pytest.fixture(autouse=True)
def _reset_print_context():
    structured_log.clear_print()
    yield
    structured_log.clear_print()


@pytest.fixture
def capture():
    cap = CaptureHandler()
    evlog = logging.getLogger("kalico.event")
    prev_level = evlog.level
    evlog.setLevel(logging.DEBUG)
    evlog.addHandler(cap)
    yield cap
    evlog.removeHandler(cap)
    evlog.setLevel(prev_level)


@pytest.fixture
def print_stats():
    printer = FakePrinter(
        objects={"gcode_move": FakeGCodeMove(), "gcode": FakeGcode()},
        reactor=FakeReactor(),
    )
    config = FakeConfig(printer)
    return PrintStats(config)


def _events(cap, name=None):
    recs = [r for r in cap.records if hasattr(r, "event")]
    if name is not None:
        recs = [r for r in recs if r.event == name]
    return recs


def test_start_emits_print_start_and_binds_id(print_stats, capture):
    assert structured_log.get_print() == ""
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()

    starts = _events(capture, "print.start")
    assert len(starts) == 1
    assert starts[0].file == "test.gcode"
    assert structured_log.get_print() != ""


def test_pause_then_resume_keeps_same_print_id(print_stats, capture):
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()
    bound_id = structured_log.get_print()

    print_stats.printer.reactor.now = 5.0
    print_stats.note_pause()

    print_stats.printer.reactor.now = 8.0
    print_stats.note_start()

    pauses = _events(capture, "print.pause")
    resumes = _events(capture, "print.resume")
    assert len(pauses) == 1
    assert len(resumes) == 1
    assert resumes[0].pause_duration_s == pytest.approx(3.0)
    assert structured_log.get_print() == bound_id


def test_complete_emits_print_end_and_clears_binding(print_stats, capture):
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()
    print_stats.printer.reactor.now = 10.0
    print_stats.note_complete()

    ends = _events(capture, "print.end")
    assert len(ends) == 1
    assert ends[0].outcome == "complete"
    assert ends[0].duration_s == pytest.approx(10.0)
    assert structured_log.get_print() == ""


def test_error_carries_reason(print_stats, capture):
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()
    print_stats.note_error("nozzle jam")

    ends = _events(capture, "print.end")
    assert len(ends) == 1
    assert ends[0].outcome == "error"
    assert ends[0].reason == "nozzle jam"


def test_double_finish_emits_only_one_print_end(print_stats, capture):
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()
    print_stats.note_complete()
    print_stats.note_complete()

    assert len(_events(capture, "print.end")) == 1


def test_reset_during_active_print_emits_print_end_reset(print_stats, capture):
    print_stats.set_current_file("test.gcode")
    print_stats.note_start()
    print_stats.printer.reactor.now = 4.0

    print_stats.reset()

    ends = _events(capture, "print.end")
    assert len(ends) == 1
    assert ends[0].outcome == "reset"
    assert structured_log.get_print() == ""


def test_reset_without_active_print_emits_nothing(print_stats, capture):
    print_stats.reset()
    assert len(_events(capture, "print.end")) == 0
