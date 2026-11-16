from __future__ import annotations

import copy
import json
import re
from pathlib import Path

import pytest
import session_check
from session_scenarios import DWELL_PENDING_S

FIXTURES = Path(__file__).with_name("fixtures") / "session"

KNOWN_BAD = [
    ("main-mesh_full_print", "journal", r"panicked at .*shaper\.rs:853"),
    ("main-mesh_full_print", "outcomes", r"ended none \(expected complete\)"),
    ("main-pause_park_resume_mesh", "journal", r"follower seam weld"),
    ("main-print_twice", "gaps", r"50\.986 s > 1 s \(#843\): 9\.71 s at"),
    (
        "main-print_twice",
        "second_print",
        r"182\.1 s vs the first print's 78\.4 s.* > 0\.1 \(#843\)",
    ),
    ("main-print_twice", "pacer_reserve", r"65\.042 s > 2 s \(#843\)"),
    ("main-cancel_then_print", "gaps", r"94\.473 s > 1 s \(#843\)"),
    ("main-cancel_then_print", "pacer_reserve", r"68\.387 s > 2 s \(#843\)"),
    (
        "main-pause_resume",
        "stop_after_pause",
        r"motion did not stop before \+30\.3 s \(#860\)",
    ),
    (
        "main-pause_cancel",
        "stop_after_pause",
        r"motion did not stop before \+28\.1 s \(#860\)",
    ),
    (
        "main-cancel_plain",
        "stop_account_after_cancel",
        r"no closing pacer brake \(pipe_drain RunwayExhausted\)",
    ),
    (
        "main-firmware_restart",
        "stop_after_firmware_restart",
        r"26\.912 s > 6 s \(#844\)",
    ),
    ("main-cancel_macro_mesh", "reactor", r"late_s 6\.896 at .*\(#406\)"),
    ("main-estop_restart_abort", "journal", r"status=6/ABRT"),
    ("main-estop_restart_abort", "post_check", r"Klippy Disconnected"),
    ("reactor_drain-estop_restart_abort", "journal", r"status=6/ABRT"),
    ("reactor_drain-estop_g4", "journal", r"status=6/ABRT"),
    ("pacer_land-pause_sparse", "stop_after_pause", r"> 12 s \(#860\)"),
    (
        "pacer_land-pause_sparse",
        "stop_account_after_pause",
        r"no closing pacer brake",
    ),
]

KNOWN_GOOD = [
    ("main-pause_park_resume_nomesh", "journal"),
    ("reactor_drain-cancel_macro_mesh", "journal"),
    ("pacer_runway-print_twice", "gaps"),
    ("pacer_runway-print_twice", "second_print"),
    ("pacer_runway-print_twice", "pacer_reserve"),
    ("pacer_runway-cancel_then_print", "gaps"),
    ("pacer_runway-cancel_then_print", "pacer_reserve"),
    ("pacer_runway-pause_resume", "stop_after_pause"),
    ("pacer_runway-pause_resume", "stop_account_after_pause"),
    ("pacer_runway-pause_cancel", "stop_after_pause"),
    ("pacer_runway-pause_cancel", "stop_account_after_pause"),
    ("pacer_runway-cancel_plain", "stop_after_cancel"),
    ("pacer_runway-cancel_plain", "stop_account_after_cancel"),
    ("reactor_drain-firmware_restart", "stop_after_firmware_restart"),
    ("reactor_drain-cancel_macro_mesh", "reactor"),
    ("reactor_drain-estop", "journal"),
    ("reactor_drain-estop", "stop_after_emergency_stop"),
    ("reactor_drain-estop_g4", "stop_after_emergency_stop"),
    ("main-estop", "stop_after_emergency_stop"),
    ("main-estop", "shutdown"),
    ("pacer_land-estop", "shutdown"),
    ("pacer_land-estop_g4", "shutdown"),
    ("pacer_land-cancel_plain", "stop_after_cancel"),
    ("pacer_land-cancel_plain", "stop_account_after_cancel"),
    ("pacer_land-cancel_then_print", "second_print"),
]

PASSING_SESSIONS = [
    "main-pause_park_resume_nomesh",
    "pacer_runway-print_twice",
    "pacer_runway-pause_resume",
    "pacer_runway-pause_cancel",
    "pacer_runway-cancel_plain",
    "reactor_drain-cancel_macro_mesh",
    "reactor_drain-firmware_restart",
    "reactor_drain-estop",
    "pacer_land-estop",
    "pacer_land-estop_g4",
    "pacer_land-cancel_plain",
    "pacer_land-cancel_then_print",
]


def _evidence(fixture: str) -> dict:
    return json.loads((FIXTURES / f"{fixture}.json").read_text())


def _verdicts(evidence: dict) -> dict[str, session_check.Verdict]:
    scenario = session_check.SCENARIO_BY_NAME[evidence["scenario"]]
    verdicts = session_check.evaluate(scenario, evidence)
    return {verdict.name: verdict for verdict in verdicts}


@pytest.mark.parametrize("fixture, check, reason", KNOWN_BAD)
def test_a_known_bad_run_fails_its_check_with_the_reason(
    fixture, check, reason
):
    verdict = _verdicts(_evidence(fixture))[check]
    assert not verdict.ok
    assert re.search(reason, verdict.detail), verdict.detail


@pytest.mark.parametrize("fixture, check", KNOWN_GOOD)
def test_a_known_good_run_passes_its_check(fixture, check):
    verdict = _verdicts(_evidence(fixture))[check]
    assert verdict.ok, verdict.detail


@pytest.mark.parametrize("fixture", PASSING_SESSIONS)
def test_a_good_run_of_the_scenarios_timeline_passes_every_check(fixture):
    failed = [v for v in _verdicts(_evidence(fixture)).values() if not v.ok]
    assert failed == []


def test_a_commanded_pause_is_not_a_mid_print_gap():
    evidence = _evidence("pacer_runway-pause_resume")
    (_, paused_at, _), (resumed_at, _, _) = evidence["spans"]
    assert resumed_at - paused_at > session_check.MID_PRINT_GAP_S.limit
    assert _verdicts(evidence)["gaps"].ok
    evidence["actions"] = [
        a for a in evidence["actions"] if a["act"] != "pause"
    ]
    assert not _verdicts(evidence)["gaps"].ok


def test_an_uncommanded_mcu_shutdown_fails():
    evidence = _evidence("pacer_runway-cancel_plain")
    evidence["events"].append(
        {
            "t": evidence["window"][0] + 30.0,
            "event": "mcu_shutdown",
            "level": "info",
            "mcu": "mcu",
            "reason": "ADC out of range",
        }
    )
    verdict = _verdicts(evidence)["shutdown"]
    assert not verdict.ok
    assert "mcu_shutdown at" in verdict.detail
    assert "ADC out of range" in verdict.detail


def test_an_estop_shutdown_logged_before_the_estop_was_sent_fails():
    evidence = _evidence("reactor_drain-estop")
    stop = next(a for a in evidence["actions"] if a["act"] == "emergency_stop")
    for event in evidence["events"]:
        if event["event"] in session_check.SHUTDOWN_EVENTS:
            event["t"] = stop["sent"] - 1.0
    assert not _verdicts(evidence)["shutdown"].ok


def test_a_stop_sent_with_no_motion_in_flight_proves_nothing():
    evidence = _evidence("main-estop")
    stop = next(a for a in evidence["actions"] if a["act"] == "emergency_stop")
    last_step = evidence["spans"][-1][1]
    stop["sent"] = last_step + 1.0
    verdict = _verdicts(evidence)["stop_after_emergency_stop"]
    assert not verdict.ok
    assert "no motion in flight" in verdict.detail


def test_a_session_that_never_settled_fails_outcomes_and_post_check():
    evidence = copy.deepcopy(_evidence("pacer_runway-cancel_plain"))
    evidence["post_check"] = None
    verdicts = _verdicts(evidence)
    assert verdicts["outcomes"].detail == "the session never settled"
    assert not verdicts["post_check"].ok


def test_a_failed_action_request_fails_the_actions_check():
    evidence = _evidence("pacer_runway-cancel_plain")
    evidence["actions"][-1].update(
        ok=False, error="HTTP 503: Klippy Disconnected"
    )
    verdict = _verdicts(evidence)["actions"]
    assert not verdict.ok
    assert "cancel at +20.1 s: HTTP 503" in verdict.detail


def test_an_estop_timed_from_a_stalled_reactors_reply_finds_no_motion():
    evidence = _evidence("main-estop_g4_stalled")
    verdicts = _verdicts(evidence)
    assert "late_s 88.509" in verdicts["reactor"].detail
    [stall] = [
        e for e in evidence["events"] if e["event"] == "reactor_poller_late"
    ]
    stop = next(a for a in evidence["actions"] if a["act"] == "emergency_stop")
    stop["sent"] = stall["t"] + DWELL_PENDING_S
    verdict = _verdicts(evidence)["stop_after_emergency_stop"]
    assert not verdict.ok
    assert "no motion in flight" in verdict.detail


def _closing_brake(evidence: dict) -> dict:
    return next(e for e in evidence["events"] if e["event"] == "pipe_drain")


def test_a_stop_past_the_engine_account_fails():
    evidence = _evidence("pacer_runway-cancel_plain")
    brake = _closing_brake(evidence)
    brake["peak_ahead_s"] = 0.0
    brake["drained_s"] = 0.0
    verdict = _verdicts(evidence)["stop_account_after_cancel"]
    assert not verdict.ok
    assert "> the engine account 0.000 s" in verdict.detail


def test_a_stop_with_no_closing_brake_logged_fails():
    evidence = _evidence("pacer_runway-pause_resume")
    evidence["events"].remove(_closing_brake(evidence))
    verdict = _verdicts(evidence)["stop_account_after_pause"]
    assert not verdict.ok
    assert "no closing pacer brake" in verdict.detail


def test_a_tail_running_into_the_next_action_without_a_fresh_stream_fails():
    evidence = _evidence("pacer_land-cancel_plain")
    evidence["events"] = [
        e for e in evidence["events"] if e["event"] != "anchor_decision"
    ]
    verdict = _verdicts(evidence)["stop_after_cancel"]
    assert not verdict.ok
    assert "motion did not stop before +25.6 s" in verdict.detail


def test_a_cancelled_tail_is_timed_to_the_rest_before_the_fresh_stream():
    verdict = _verdicts(_evidence("pacer_land-cancel_plain"))[
        "stop_after_cancel"
    ]
    assert "stopped after 6.393 s <= 12 s (#860)" in verdict.detail
