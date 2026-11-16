"""Reduce a session run dir (session.py) to the evidence session_check.py
judges: the timeline the runner recorded (actions, klippy and print state
changes, post-check), the capture's motion as spans of steps, the
structured events that bear on the checks, and klippy aborts from the Pi
journal. klippy.log lines are read from their timestamped mirror in
events/host-py.jsonl; klippy.log itself carries no wall time.
"""

from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path

import events
import numpy as np

SPAN_JOIN_S = 0.25
TIME_DECIMALS = 4
EVENT_NAMES = (
    "print.start",
    "print.pause",
    "print.resume",
    "print.end",
    "reactor_poller_late",
    "pacer_reserve_raised",
    "pacer_reserve_capped",
    "anchor_idle_resume",
    "anchor_underrun",
    "stream_worker_fatal",
    "endpoint_death",
    "endpoint_death_watchdog_abort",
    "endpoint_death_settled_by_shutdown",
)
LOG_EVENTS = {
    "klippy_shutdown": re.compile(
        r"^Transition to shutdown state: (?P<reason>.*)"
    ),
    "mcu_shutdown": re.compile(
        r"^MCU '(?P<mcu>[^']+)' shutdown: (?P<reason>.*)"
    ),
    "mcu_reset": re.compile(
        r"^Attempting MCU '(?P<mcu>[^']+)' (?:config_)?reset command"
    ),
}
NAMED_LINE = re.compile(
    r'"event":\s*"(?:' + "|".join(map(re.escape, EVENT_NAMES)) + ')"'
)
LOG_LINE = re.compile(
    r"Transition to shutdown state|' shutdown: |Attempting MCU '"
)
BOILERPLATE_FIELDS = frozenset(
    {"_time", "_msg", "event", "session_id", "target", "subsystem"}
)
JOURNAL_ABORT = re.compile(
    r"panicked at|Main process exited|Failed with result"
)
JOURNAL_PANIC = "panicked at"


class EvidenceError(RuntimeError):
    pass


def collect(run_dir: Path) -> dict:
    import analyze

    info = json.loads((run_dir / "run.json").read_text())
    session = info.get("session")
    if session is None:
        raise EvidenceError(f"{run_dir}/run.json holds no session record")
    capture = analyze.load_capture(run_dir)
    start = capture.meta["started_wall"]
    end = (
        session["ended"] or datetime.fromisoformat(info["host_end"]).timestamp()
    )
    return {
        "run_dir": str(run_dir),
        "head": info["deployed"]["head"],
        "scenario": session["scenario"],
        "window": [_at(start), _at(end)],
        "actions": session["actions"],
        "states": state_changes(session["samples"]),
        "post_check": session["post_check"],
        "spans": activity_spans(capture.deltas, capture.tick_hz, start),
        "events": watched_events(run_dir / "events", start, end),
        "journal": abort_lines((run_dir / "journal.txt").read_text()),
    }


def _at(wall_s: float) -> float:
    return round(wall_s, TIME_DECIMALS)


def state_changes(samples: list[dict]) -> list[dict]:
    changes: list[dict] = []
    for state in samples:
        if changes and (changes[-1]["klippy"], changes[-1]["print"]) == (
            state["klippy"],
            state["print"],
        ):
            continue
        changes.append(
            {
                "t": _at(state["t"]),
                "klippy": state["klippy"],
                "klippy_message": state["klippy_message"],
                "print": state["print"],
                "print_message": state["print_message"],
            }
        )
    return changes


def activity_spans(
    deltas: np.ndarray, tick_hz: float, start: float
) -> list[list]:
    steps = np.abs(deltas.astype(np.int16)).sum(axis=1, dtype=np.int64)
    active = np.flatnonzero(steps)
    if active.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(active) > SPAN_JOIN_S * tick_hz)
    firsts = active[np.concatenate(([0], breaks + 1))]
    lasts = active[np.concatenate((breaks, [active.size - 1]))]
    totals = np.concatenate(([0], np.cumsum(steps)))
    return [
        [
            _at(start + first / tick_hz),
            _at(start + last / tick_hz),
            int(totals[last + 1] - totals[first]),
        ]
        for first, last in zip(firsts, lasts)
    ]


def watched_events(events_dir: Path, start: float, end: float) -> list[dict]:
    found = []
    for path in sorted(events_dir.glob("*.jsonl*")):
        with path.open(errors="replace") as lines:
            for number, line in enumerate(lines, 1):
                if not (NAMED_LINE.search(line) or LOG_LINE.search(line)):
                    continue
                try:
                    record = json.loads(line)
                except ValueError as exc:
                    raise EvidenceError(
                        f"{path}:{number}: unreadable watched record: {exc}"
                    ) from exc
                when = events.record_time(record)
                event = describe(record)
                if event is not None and start <= when <= end:
                    found.append({"t": _at(when), **event})
    return sorted(found, key=lambda event: event["t"])


def describe(record: dict) -> dict | None:
    if record.get("event") in EVENT_NAMES:
        fields = {
            key: value
            for key, value in record.items()
            if key not in BOILERPLATE_FIELDS
        }
        return {"event": record["event"], **fields}
    message = record["_msg"].split("\n")[0]
    for name, pattern in LOG_EVENTS.items():
        match = pattern.match(message)
        if match:
            return {
                "event": name,
                "level": record["level"],
                **match.groupdict(),
            }
    return None


def abort_lines(journal: str) -> list[str]:
    lines = journal.splitlines()
    kept = []
    for index, line in enumerate(lines):
        if not JOURNAL_ABORT.search(line):
            continue
        kept.append(line)
        if JOURNAL_PANIC in line and index + 1 < len(lines):
            kept.append(lines[index + 1])
    return kept
