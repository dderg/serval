from __future__ import annotations

import json
from datetime import datetime, timezone

import numpy as np
import pytest
import session_check
import session_evidence

TICK_HZ = 40_000.0
START = 1_794_775_690.0
TIME_TOLERANCE_S = 1e-4


def _record(t: float, **fields) -> str:
    stamp = datetime.fromtimestamp(t, timezone.utc).isoformat()
    record = {"_time": stamp.replace("+00:00", "Z"), "level": "info", **fields}
    return json.dumps(record) + "\n"


def _deltas(ticks: int, steps: dict[int, int]) -> np.ndarray:
    deltas = np.zeros((ticks, 4), np.int8)
    for tick, value in steps.items():
        deltas[tick, 0] = value
    return deltas


def test_steps_closer_than_the_join_are_one_span_and_a_pause_splits_them():
    first = {tick: 1 for tick in range(100, 200, 10)}
    second = {50_000: -128, 50_010: 1}
    deltas = _deltas(60_000, {**first, **second})
    spans = session_evidence.activity_spans(deltas, TICK_HZ, START)
    assert [pytest.approx(span, abs=TIME_TOLERANCE_S) for span in spans] == [
        [START + 100 / TICK_HZ, START + 190 / TICK_HZ, 10],
        [START + 50_000 / TICK_HZ, START + 50_010 / TICK_HZ, 129],
    ]


def test_a_capture_without_steps_has_no_spans():
    deltas = _deltas(1000, {})
    assert session_evidence.activity_spans(deltas, TICK_HZ, START) == []


def _state(t: float, klippy: str) -> dict:
    return {"t": t, "klippy": klippy, "print": "standby"}


def test_a_restart_is_timed_to_ready_and_to_the_first_motion_after_it():
    actions = [
        {"act": "pause", "sent": START + 1.0},
        {"act": "firmware_restart", "sent": START + 10.0},
    ]
    states = [
        _state(START, "ready"),
        _state(START + 10.5, "startup"),
        _state(START + 11.3, "ready"),
    ]
    spans = [[START + 2.0, START + 9.0, 50], [START + 27.8, START + 28.0, 9]]

    [timing] = session_evidence.restart_timings(actions, states, spans)

    assert timing["act"] == "firmware_restart"
    assert timing["ready_s"] == pytest.approx(1.3)
    assert timing["first_motion_s"] == pytest.approx(17.8)


def test_a_restart_that_never_moved_again_has_no_first_motion():
    actions = [{"act": "host_restart", "sent": START}]
    states = [_state(START + 0.5, "startup"), _state(START + 1.0, "ready")]

    [timing] = session_evidence.restart_timings(actions, states, [])

    assert timing["ready_s"] == pytest.approx(1.0)
    assert timing["first_motion_s"] is None


def test_only_klippy_and_print_state_changes_are_kept():
    samples = [
        {
            "t": START + i,
            "klippy": klippy,
            "klippy_message": "",
            "print": state,
            "print_message": "",
            "file_position": i,
        }
        for i, (klippy, state) in enumerate(
            [
                ("ready", "printing"),
                ("ready", "printing"),
                ("ready", "complete"),
                ("ready", "complete"),
                ("unreachable", None),
            ]
        )
    ]
    changes = session_evidence.state_changes(samples)
    assert [(c["klippy"], c["print"]) for c in changes] == [
        ("ready", "printing"),
        ("ready", "complete"),
        ("unreachable", None),
    ]
    assert "file_position" not in changes[0]


def test_a_panic_keeps_its_message_line_and_ordinary_lines_are_dropped():
    journal = "\n".join(
        [
            "Nov 15 21:37:30 bench-pi python[1]: Starting klippy",
            "Nov 15 21:37:35 bench-pi python[1]: thread 'kalico-shaper' "
            "panicked at rust/motion-pipeline/src/shaper.rs:853:29:",
            "Nov 15 21:37:35 bench-pi python[1]: shaper: axis 3: follower "
            "seam weld",
            "Nov 15 21:37:35 bench-pi python[1]: note: run with RUST_BACKTRACE",
            "Nov 15 21:37:35 bench-pi systemd[1]: klipper.service: Main "
            "process exited, code=killed, status=6/ABRT",
        ]
    )
    lines = session_evidence.abort_lines(journal)
    assert len(lines) == 3
    assert "panicked at" in lines[0]
    assert "follower seam weld" in lines[1]
    assert "status=6/ABRT" in lines[2]


def test_named_events_keep_their_fields_and_log_lines_get_a_name():
    named = {
        "_time": "x",
        "_msg": "[pacer] slowest brake",
        "event": "pacer_reserve_raised",
        "level": "info",
        "session_id": "k-1",
        "target": "motion_core",
        "subsystem": "motion",
        "reserve_s": 18.05,
    }
    assert session_evidence.describe(named) == {
        "event": "pacer_reserve_raised",
        "level": "info",
        "reserve_s": 18.05,
    }
    shutdown = {
        "_msg": "MCU 'mcu' shutdown: Command request\nDumping config",
        "level": "info",
    }
    assert session_evidence.describe(shutdown) == {
        "event": "mcu_shutdown",
        "level": "info",
        "mcu": "mcu",
        "reason": "Command request",
    }
    assert (
        session_evidence.describe({"_msg": "Stats 1.0", "level": "info"})
        is None
    )


def test_watched_events_come_from_rotated_files_and_the_window_only(tmp_path):
    events = tmp_path / "events"
    events.mkdir()
    (events / "host-py.jsonl.1").write_text(
        _record(START - 10, _msg="print started", event="print.start")
        + _record(START + 1, _msg="print started", event="print.start")
    )
    (events / "host-py.jsonl").write_text(
        _record(START + 3, _msg="Stats 3.0: sysload=0.1")
        + _record(
            START + 2,
            _msg="Transition to shutdown state: Lost communication",
        )
        + _record(START + 99, _msg="print ended", event="print.end")
    )
    found = session_evidence.watched_events(events, START, START + 50)
    assert [(e["t"], e["event"]) for e in found] == [
        (START + 1, "print.start"),
        (START + 2, "klippy_shutdown"),
    ]
    assert found[1]["reason"] == "Lost communication"


def test_an_unreadable_watched_record_is_fatal(tmp_path):
    events = tmp_path / "events"
    events.mkdir()
    (events / "host-rust.jsonl").write_text('{"event":"endpoint_death", "_ti\n')
    with pytest.raises(session_evidence.EvidenceError, match="host-rust"):
        session_evidence.watched_events(events, START, START + 1)


def test_a_run_dir_reduces_to_its_evidence(tmp_path):
    deltas = _deltas(80_000, {40_000: 1, 40_040: 1})
    deltas.tofile(tmp_path / "capture.bin")
    (tmp_path / "capture.json").write_text(
        json.dumps(
            {
                "mode": "counts",
                "tick_hz": TICK_HZ,
                "channel_count": 4,
                "aux": False,
                "started_wall": START,
            }
        )
    )
    (tmp_path / "events").mkdir()
    (tmp_path / "events" / "host-py.jsonl").write_text(
        _record(START + 1.2, _msg="print started", event="print.start")
    )
    (tmp_path / "journal.txt").write_text("Nov 16 klippy: nothing\n")
    sample = {
        "t": START + 1.0,
        "klippy": "ready",
        "klippy_message": "Printer is ready",
        "print": "printing",
        "print_message": "",
        "file_position": 10,
    }
    action = {"act": "start_print", "sent": START + 1.0, "ok": True}
    session = {
        "scenario": "cancel_plain",
        "actions": [action],
        "samples": [sample],
        "post_check": None,
        "ended": START + 2.0,
    }
    (tmp_path / "run.json").write_text(
        json.dumps({"deployed": {"head": "abc"}, "session": session})
    )
    evidence = session_evidence.collect(tmp_path)
    assert evidence["scenario"] == "cancel_plain"
    assert evidence["head"] == "abc"
    assert evidence["window"] == [START, START + 2.0]
    assert evidence["spans"] == [
        pytest.approx([START + 1.0, START + 1.001, 2], abs=TIME_TOLERANCE_S)
    ]
    assert [e["event"] for e in evidence["events"]] == ["print.start"]
    assert evidence["journal"] == []
    assert evidence["actions"] == [action]
    assert [type(value) for value in evidence["spans"][0]] == [
        float,
        float,
        int,
    ]
    json.dumps(session_check.judge(evidence))


def test_a_run_dir_without_a_session_record_is_refused(tmp_path):
    (tmp_path / "run.json").write_text(json.dumps({"print_state": "complete"}))
    with pytest.raises(session_evidence.EvidenceError, match="no session"):
        session_evidence.collect(tmp_path)


def _stats(t: float, memavail: int = 3_500_000, session: str = "k-1") -> str:
    return _record(
        t,
        _msg=f"Stats {t:.1f}: gcodein=0 sysload=0.20 cputime=12.500 "
        f"memavail={memavail} print_time=4.9 buffer_time=0.000",
        session_id=session,
    )


def test_klippy_stats_come_from_every_host_py_file_in_the_window(tmp_path):
    (tmp_path / "host-py.jsonl.1").write_text(_stats(START - 5) + _stats(START))
    (tmp_path / "host-py.jsonl").write_text(
        _stats(START + 1, memavail=3_400_000)
        + _record(START + 1.5, _msg="print started", event="print.start")
        + _stats(START + 60)
    )
    rows = session_evidence.klippy_stats(tmp_path, START, START + 10)
    assert rows == [
        {
            "t": START,
            "session_id": "k-1",
            "memavail_kb": 3_500_000,
            "cputime_s": 12.5,
        },
        {
            "t": START + 1,
            "session_id": "k-1",
            "memavail_kb": 3_400_000,
            "cputime_s": 12.5,
        },
    ]


def test_a_plain_run_reduces_to_its_window_events_and_stats(tmp_path):
    (tmp_path / "events").mkdir()
    (tmp_path / "events" / "host-py.jsonl").write_text(
        _stats(START + 1) + _stats(START + 2)
    )
    (tmp_path / "journal.txt").write_text("")
    begin = datetime.fromtimestamp(START).isoformat()
    end = datetime.fromtimestamp(START + 3).isoformat()
    (tmp_path / "run.json").write_text(
        json.dumps(
            {"deployed": {"head": "abc"}, "host_start": begin, "host_end": end}
        )
    )
    evidence = session_evidence.collect_run(tmp_path)
    assert evidence["scenario"] is None
    assert evidence["window"] == [START, START + 3]
    assert [row["t"] for row in evidence["stats"]] == [START + 1, START + 2]
    assert evidence["actions"] == evidence["spans"] == []
    verdict = session_check.judge(evidence)
    assert verdict["ok"], verdict
