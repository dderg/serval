"""The invariants every scenario is held to, and the log scans behind the
ones that read klippy's and the engine's records.

Thresholds come from the e2e tests that first asserted them:
- REACTOR_STALL, heater-driven MCU shutdowns: test_print_interruptions.py
- FATAL_EVENTS: test_mesh_follower_print.py, test_firmware_restart.py,
  test_session_runway.py (anchor_underrun, #856)
- DURATION_TOLERANCE, MOTION_STILL_S, pipe_drain: test_session_runway.py
- ESTOP_DEADLINE_S, ESTOP_REASON: test_print_interruptions.py
- ENDPOINT_DEATH_GRACE_S: test_firmware_restart.py
- exact rest steps: test_ident_weld.py (#859)
"""

from __future__ import annotations

import dataclasses
import datetime
import enum
import json
import math
import pathlib
import re
from typing import Iterable, Optional

REACTOR_STALL = "reactor_poller_late"
FATAL_EVENTS = (
    "stream_worker_fatal",
    "anchor_underrun",
    "pump_piece_in_past",
    "endpoint_death_watchdog_abort",
    "runtime_fault",
)
PANIC = "panicked"
SHUTDOWN_TRANSITION = "Transition to shutdown state: "
ESTOP_REASON = "Shutdown due to webhooks request"
ESTOP_DEADLINE_S = 0.1
ENDPOINT_DEATH_GRACE_S = 5.0
DURATION_TOLERANCE = 0.10
MOTION_STILL_S = 2.0
PACER_BRAKE = "pipe_drain"
PACER_LOG = "info,motion_core::worker::ingress=debug"
LATTICE_TOLERANCE_STEPS = 1e-6
POSITION_REL_TOL = 1e-6
POSITION_ABS_TOL = 1e-9
NUMBER = re.compile(r"\d+(\.\d+)?")


class Check(enum.Enum):
    BOOT = "boot"
    KLIPPY_EXIT = "klippy_exit"
    PANIC = "panic"
    FATAL_EVENT = "fatal_event"
    UNCOMMANDED_SHUTDOWN = "uncommanded_shutdown"
    REACTOR_STALL = "reactor_stall"
    REST_STEPS = "rest_steps"
    PRINT_OUTCOME = "print_outcome"
    STOP_STATE = "stop_state"
    RECOVERY = "recovery"
    COMMAND_RESULT = "command_result"
    ESTOP_LATENCY = "estop_latency"
    SESSION_REPEAT = "session_repeat"
    PACER_BRAKE = "pacer_brake"
    IDLE_TIMEOUT = "idle_timeout"
    WEDGE = "wedge"
    HARNESS = "harness"
    STUCK = "stuck"
    MEMORY = "memory"


ROOT_CAUSES = (Check.PANIC, Check.FATAL_EVENT, Check.UNCOMMANDED_SHUTDOWN)


def ordered(
    step: Optional["Violation"], logged: list["Violation"]
) -> list["Violation"]:
    """A panic, fatal engine event or MCU shutdown explains whatever a
    step-level check saw fail after it, so those lead; the step's own
    violation comes next, then the remaining log findings."""
    roots = [v for v in logged if v.check in ROOT_CAUSES]
    rest = [v for v in logged if v.check not in ROOT_CAUSES]
    return roots + ([step] if step else []) + rest


class Violation(Exception):
    def __init__(self, check: Check, evidence: str):
        super().__init__(f"{check.value}: {evidence}")
        self.check = check
        self.evidence = evidence


def signature(violation: Violation) -> str:
    """The check and the first evidence line with numbers masked, so the
    same failure at another position or time dedupes to one entry."""
    first = (
        violation.evidence.strip().splitlines()[0] if violation.evidence else ""
    )
    return f"{violation.check.value}: {NUMBER.sub('N', first)[:200]}"


def lattice_counts(position_mm: float, steps_per_mm: float) -> set[int]:
    """The step counts a motor may rest on at `position_mm`: the line it
    sits on, or either line around it (a rest reached rising stops below
    a line it has not reached, falling above one)."""
    exact = position_mm * steps_per_mm
    nearest = round(exact)
    if abs(exact - nearest) <= LATTICE_TOLERANCE_STEPS:
        return {nearest}
    return {math.floor(exact), math.ceil(exact)}


def same_position(measured: float, expected: float) -> bool:
    return math.isclose(
        measured, expected, rel_tol=POSITION_REL_TOL, abs_tol=POSITION_ABS_TOL
    )


def _lines_with(text: str, needles: Iterable[str]) -> list[str]:
    return [
        line for line in text.splitlines() if any(n in line for n in needles)
    ]


@dataclasses.dataclass
class Logs:
    klippy: str
    stdout: str
    events: str

    @classmethod
    def read(cls, log_dir: pathlib.Path) -> "Logs":
        def text(path: pathlib.Path) -> str:
            return path.read_text(errors="replace") if path.exists() else ""

        events = "".join(
            text(path) for path in sorted((log_dir / "events").glob("*.jsonl"))
        )
        return cls(
            text(log_dir / "klippy.log"),
            text(log_dir / "klippy.stdout"),
            events,
        )


def _event_name(line: str) -> str:
    try:
        return json.loads(line).get("event", "")
    except ValueError:
        return ""


def first_panic(text: str) -> Optional[str]:
    """`<location> -- <message>`: Rust prints the message on the line after
    `thread '...' panicked at <file>:<line>:<col>:`, and the message tells
    apart panics sharing one site (shaper.rs:853 carries several)."""
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if PANIC not in line:
            continue
        message = lines[index + 1].strip() if index + 1 < len(lines) else ""
        return f"{line.strip()} -- {message}"
    return None


def _first_panic(logs: Logs) -> Optional[str]:
    return first_panic(logs.stdout) or first_panic(logs.events)


def _fatal_events(logs: Logs) -> list[str]:
    return [
        line
        for line in logs.events.splitlines()
        if _event_name(line) in FATAL_EVENTS
    ]


def _shutdowns(logs: Logs) -> list[str]:
    return [
        line.split(SHUTDOWN_TRANSITION, 1)[1].strip()
        for line in logs.klippy.splitlines()
        if SHUTDOWN_TRANSITION in line
    ]


def uncommanded_shutdowns(logs: Logs, commanded: list[str]) -> list[str]:
    left = list(commanded)
    unexpected = []
    for reason in _shutdowns(logs):
        match = next((c for c in left if c in reason), None)
        if match is None:
            unexpected.append(reason)
        else:
            left.remove(match)
    return unexpected


def scan(
    logs: Logs, commanded_shutdowns: list[str], klippy_alive: bool
) -> list[Violation]:
    """Every log-borne violation, root causes first."""
    found = []
    panic = _first_panic(logs)
    if panic:
        found.append(Violation(Check.PANIC, panic))
    fatal = _fatal_events(logs)
    if fatal:
        found.append(Violation(Check.FATAL_EVENT, _summarize_event(fatal[0])))
    unexpected = uncommanded_shutdowns(logs, commanded_shutdowns)
    if unexpected:
        found.append(Violation(Check.UNCOMMANDED_SHUTDOWN, unexpected[0]))
    if not klippy_alive:
        found.append(Violation(Check.KLIPPY_EXIT, logs.stdout.strip()[-1500:]))
    stalls = [
        line
        for line in logs.events.splitlines()
        if _event_name(line) == REACTOR_STALL
    ]
    if stalls:
        found.append(
            Violation(Check.REACTOR_STALL, _summarize_event(stalls[0]))
        )
    return found


def _summarize_event(line: str) -> str:
    try:
        record = json.loads(line)
    except ValueError:
        return line[:500]
    keep = {
        k: v
        for k, v in record.items()
        if k not in ("_time", "session_id", "print_id", "thread_id")
    }
    return json.dumps(keep, sort_keys=True)[:800]


def events_since(log_dir: pathlib.Path, offsets: dict) -> list[dict]:
    events = []
    for path in sorted((log_dir / "events").glob("*.jsonl")):
        with path.open("rb") as stream:
            stream.seek(offsets.get(path, 0))
            text = stream.read().decode(errors="replace")
        for line in text.splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                continue
    return events


def event_offsets(log_dir: pathlib.Path) -> dict:
    return {
        path: path.stat().st_size
        for path in (log_dir / "events").glob("*.jsonl")
    }


def event_wall_ns(event: dict) -> int:
    stamp = datetime.datetime.strptime(event["_time"], "%Y-%m-%dT%H:%M:%S.%fZ")
    stamp = stamp.replace(tzinfo=datetime.timezone.utc)
    return int(stamp.timestamp() * 1e9)
