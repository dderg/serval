from __future__ import annotations

import io
import json
import urllib.error
from types import SimpleNamespace

import pytest
import run
import session
from session_scenarios import NEXT_PRINT_AFTER_S, SCENARIO_BY_NAME, Act

START = 1_794_775_690.0
PRINT_S = 100.0
READ_AHEAD = 2.0


class Clock:
    def __init__(self):
        self.now = START

    def time(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class InlineRequests:
    def __init__(self, _workers):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False

    def submit(self, request, *args):
        request(*args)


class Printer:
    def __init__(
        self, clock: Clock, file_size: int, print_s: float, read_ahead: float
    ):
        self.clock = clock
        self.file_size = file_size
        self.print_s = print_s
        self.read_ahead = read_ahead
        self.klippy = "ready"
        self.state = "standby"
        self.started = 0.0
        self.performed: list[tuple[Act, float]] = []

    def perform(self, action, _remote_name) -> None:
        self.performed.append((action.act, self.clock.now))
        transitions = {
            Act.START_PRINT: ("ready", "printing"),
            Act.PAUSE: ("ready", "paused"),
            Act.RESUME: ("ready", "printing"),
            Act.CANCEL: ("ready", "cancelled"),
            Act.EMERGENCY_STOP: ("shutdown", "error"),
            Act.FIRMWARE_RESTART: ("ready", "standby"),
        }
        self.klippy, self.state = transitions[action.act]
        if action.act is Act.START_PRINT:
            self.started = self.clock.now

    def sample(self) -> dict:
        elapsed = self.clock.now - self.started
        if self.state == "printing" and elapsed >= self.print_s:
            self.state = "complete"
        printing = self.state == "printing"
        read = min(1.0, self.read_ahead * elapsed / self.print_s)
        read = read if printing else 0
        return {
            "t": self.clock.now,
            "klippy": self.klippy,
            "klippy_message": "",
            "print": self.state,
            "print_message": "",
            "file_position": int(self.file_size * read),
        }

    def times(self, act: Act) -> list[float]:
        return [at for done, at in self.performed if done is act]


class Capture:
    def check(self) -> None:
        pass


@pytest.fixture
def bench(monkeypatch):
    clock = Clock()
    scripts: list[str] = []
    monkeypatch.setattr(session, "time", clock)
    monkeypatch.setattr(session, "ThreadPoolExecutor", InlineRequests)
    monkeypatch.setattr(
        session.pi, "gcode_script", lambda line, _t: scripts.append(line)
    )

    def drive(
        name: str,
        print_s: float = PRINT_S,
        perform=None,
        read_ahead: float = READ_AHEAD,
    ):
        scenario = SCENARIO_BY_NAME[name]
        lines = scenario.gcode.lines()
        size = len(("\n".join(lines) + "\n").encode())
        printer = Printer(clock, size, print_s, read_ahead)
        monkeypatch.setattr(session, "perform", perform or printer.perform)
        monkeypatch.setattr(session, "sample", printer.sample)
        markers = session.marker_offsets(scenario, lines)
        runner = session.Session(scenario, "bench.gcode", markers)
        drive.record = runner.record
        final = runner.drive(Capture())
        return runner.record, printer, final

    drive.clock = clock
    drive.scripts = scripts
    return drive


class RecordingCapture:
    def __init__(self, *_args, **_kwargs):
        pass

    def start(self) -> None:
        pass

    def check(self) -> None:
        pass

    def stop(self):
        return SimpleNamespace(unit="ticks", units=0)


def test_a_scenario_run_keeps_its_session_in_run_json(
    bench, monkeypatch, tmp_path
):
    scenario = SCENARIO_BY_NAME["cancel_plain"]
    size = len(("\n".join(scenario.gcode.lines()) + "\n").encode())
    printer = Printer(bench.clock, size, PRINT_S, READ_AHEAD)
    monkeypatch.setattr(session, "perform", printer.perform)
    monkeypatch.setattr(session, "sample", printer.sample)
    run_dir = tmp_path / "run"

    def start_run(lines, runs_dir, config_text, source):
        run_dir.mkdir()
        assert "[bed_mesh]" not in config_text
        return run_dir, "bench.gcode", {**source, "deployed": {"head": "a"}}

    monkeypatch.setattr(run, "start_run", start_run)
    monkeypatch.setattr(run, "Capture", RecordingCapture)
    monkeypatch.setattr(run.time, "sleep", lambda _s: None)
    monkeypatch.setattr(run, "scp_from_pi", lambda *_a, **_k: None)
    monkeypatch.setattr(run, "foreign_deploy", lambda _expected: None)
    monkeypatch.setattr(run, "klipper_journal", lambda _since: "")
    assert session.run_scenario(scenario, tmp_path) == run_dir
    info = json.loads((run_dir / "run.json").read_text())
    assert info["scenario"] == "cancel_plain"
    assert info["print_state"] == "cancelled"
    recorded = info["session"]
    assert [a["act"] for a in recorded["actions"]] == ["start_print", "cancel"]
    assert recorded["post_check"]["after"]["klippy"] == "ready"
    assert recorded["ended"] >= recorded["post_check"]["started"]


def test_the_next_print_starts_after_the_first_ends(bench):
    record, printer, final = bench("print_twice")
    first, second = printer.times(Act.START_PRINT)
    ended = next(s["t"] for s in record["samples"] if s["print"] == "complete")
    assert second == ended + NEXT_PRINT_AFTER_S
    assert final == {"state": "complete", "message": ""}
    assert record["post_check"]["lines"][-1] == {
        "line": "M400",
        "ok": True,
        "error": None,
    }
    assert bench.scripts == list(session.POST_CHECK)


def test_actions_after_the_previous_one_wait_their_delay(bench):
    record, printer, _ = bench("pause_resume")
    (start,) = printer.times(Act.START_PRINT)
    (paused,) = printer.times(Act.PAUSE)
    (resumed,) = printer.times(Act.RESUME)
    timeline = SCENARIO_BY_NAME["pause_resume"].timeline
    assert paused - start == pytest.approx(timeline[1].delay_s)
    assert resumed - paused == pytest.approx(timeline[2].delay_s)
    assert [a["act"] for a in record["actions"]] == [
        "start_print",
        "pause",
        "resume",
    ]


def test_the_estop_waits_until_the_dwell_line_is_read(bench):
    record, printer, _ = bench("estop_g4")
    scenario = SCENARIO_BY_NAME["estop_g4"]
    lines = scenario.gcode.lines()
    markers = session.marker_offsets(scenario, lines)
    offset = markers[scenario.timeline[1].marker]
    read = next(
        s["t"] for s in record["samples"] if s["file_position"] >= offset
    )
    (stopped,) = printer.times(Act.EMERGENCY_STOP)
    assert stopped == read + scenario.timeline[1].delay_s
    (restarted,) = printer.times(Act.FIRMWARE_RESTART)
    assert restarted - stopped == pytest.approx(scenario.timeline[2].delay_s)
    waited = record["actions"][-1]
    assert waited["act"] == "wait" and waited["ok"]
    assert record["post_check"]["started"] >= waited["sent"]


def test_a_marker_never_seen_read_fires_once_the_print_ended(bench):
    record, printer, _ = bench("estop_g4", read_ahead=0.0)
    ended = next(s["t"] for s in record["samples"] if s["print"] == "complete")
    (stopped,) = printer.times(Act.EMERGENCY_STOP)
    delay = SCENARIO_BY_NAME["estop_g4"].timeline[1].delay_s
    assert stopped == ended + delay


def test_the_session_settles_before_the_post_check(bench):
    record, _, _ = bench("cancel_plain")
    cancelled = next(
        s["t"] for s in record["samples"] if s["print"] == "cancelled"
    )
    assert record["post_check"]["started"] >= cancelled + session.SETTLE_S


def test_a_failed_request_is_recorded_not_raised(bench):
    def refuse(action, _remote_name):
        raise urllib.error.URLError("connection refused")

    record, _, _ = bench("cancel_plain", perform=refuse)
    assert [(a["act"], a["ok"]) for a in record["actions"]] == [
        ("start_print", False),
        ("cancel", False),
    ]
    assert "connection refused" in record["actions"][0]["error"]
    assert record["post_check"] is not None


def test_a_print_that_never_ends_times_out_with_its_record_kept(bench):
    with pytest.raises(run.BenchError, match="start_print timed out"):
        bench("print_twice", print_s=2 * session.TIMELINE_TIMEOUT_S)
    assert [a["act"] for a in bench.record["actions"]] == ["start_print"]
    assert bench.record["samples"][-1]["print"] == "printing"


def test_the_post_check_stops_at_the_first_failed_line(monkeypatch):
    sent = []

    def gcode(line, _timeout):
        sent.append(line)
        if line.startswith("G1"):
            raise urllib.error.URLError("Klippy Disconnected")

    monkeypatch.setattr(session.pi, "gcode_script", gcode)
    monkeypatch.setattr(session, "sample", lambda: {"klippy": "ready"})
    result = session.post_check()
    assert sent == list(session.POST_CHECK[:3])
    assert result["lines"][-1]["ok"] is False
    assert "Klippy Disconnected" in result["lines"][-1]["error"]


def test_a_marker_line_must_occur_exactly_once():
    scenario = SCENARIO_BY_NAME["estop_g4"]
    lines = [*scenario.gcode.lines(), scenario.timeline[1].marker]
    with pytest.raises(run.BenchError, match="occurs 2 times"):
        session.marker_offsets(scenario, lines)


def test_marker_offsets_are_byte_offsets_into_the_uploaded_file():
    scenario = SCENARIO_BY_NAME["estop_g4"]
    lines = scenario.gcode.lines()
    marker = scenario.timeline[1].marker
    uploaded = ("\n".join(lines) + "\n").encode()
    offset = session.marker_offsets(scenario, lines)[marker]
    assert uploaded[offset:].startswith(f"{marker}\n".encode())


def _http_error(body: bytes) -> urllib.error.HTTPError:
    return urllib.error.HTTPError(
        "http://pi/printer/gcode/script", 503, "", None, io.BytesIO(body)
    )


def test_an_http_error_is_described_by_moonrakers_message():
    body = b'{"error": {"code": 503, "message": "Klippy Disconnected"}}'
    assert session.describe_error(_http_error(body)) == (
        "HTTP 503: Klippy Disconnected"
    )
    assert session.describe_error(_http_error(b"<html>")) == "HTTP 503: <html>"


def test_a_sample_is_stamped_when_its_reply_arrives(monkeypatch):
    clock = Clock()
    stalled_s = 88.5

    def http(method, path, *args, **kwargs):
        if path != session.STATUS_QUERY:
            return {"state": "ready", "state_message": "Printer is ready"}
        clock.sleep(stalled_s)
        return {
            "status": {
                "print_stats": {"state": "printing", "message": ""},
                "virtual_sdcard": {"file_position": 7},
            }
        }

    monkeypatch.setattr(session, "time", clock)
    monkeypatch.setattr(session.pi, "http", http)
    assert session.sample()["t"] == START + stalled_s
