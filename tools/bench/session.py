"""Run one bench session scenario (session_scenarios.py): its config and
gcode, its timeline of actions on one klippy session, then the post-check
moves, all inside one capture. Prints the run dir; session_check.py judges
it.

uv run python tools/bench/session.py <scenario> [--runs-dir DIR]
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.parse
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import lock
import pi
import run
from capture import Capture
from session_scenarios import (
    SCENARIO_BY_NAME,
    Act,
    Action,
    After,
    Scenario,
)

POLL_S = 0.25
SETTLE_S = 4.0
TIMELINE_TIMEOUT_S = 1800.0
ACTION_TIMEOUT_S = 120.0
UNREACHABLE = "unreachable"
ERROR_TEXT_LIMIT = 400
STATUS_QUERY = "/printer/objects/query?print_stats&virtual_sdcard"
ACTIVE_PRINT = frozenset({"printing", "paused"})
ENDED_PRINT = frozenset({"complete", "cancelled", "error", "standby"})
SETTLED_KLIPPY = frozenset({"ready", "shutdown", "error"})
ACTION_PATHS = {
    Act.PAUSE: "/printer/print/pause",
    Act.RESUME: "/printer/print/resume",
    Act.CANCEL: "/printer/print/cancel",
    Act.FIRMWARE_RESTART: "/printer/firmware_restart",
    Act.EMERGENCY_STOP: "/printer/emergency_stop",
}
POST_CHECK = (
    "SET_KINEMATIC_POSITION X=100 Y=100 Z=10",
    "G90",
    "G1 X130 Y110 F6000",
    "G1 X100 Y100 F6000",
    "M400",
)
DueTime = Callable[[], float | None]


def describe_error(exc: BaseException) -> str:
    if not isinstance(exc, urllib.error.HTTPError):
        return repr(exc)[:ERROR_TEXT_LIMIT]
    body = exc.read().decode(errors="replace")
    try:
        message = json.loads(body)["error"]["message"]
    except (ValueError, KeyError, TypeError):
        message = body
    return f"HTTP {exc.code}: {message}"[:ERROR_TEXT_LIMIT]


def sample() -> dict:
    unknown_print = {
        "print": None,
        "print_message": None,
        "file_position": None,
    }
    try:
        info = pi.http("GET", "/printer/info")
    except (OSError, ValueError) as exc:
        return {
            "t": time.time(),
            "klippy": UNREACHABLE,
            "klippy_message": describe_error(exc),
            **unknown_print,
        }
    state = {
        "klippy": info["state"],
        "klippy_message": info.get("state_message", "").split("\n")[0],
    }
    try:
        status = pi.http("GET", STATUS_QUERY)["status"]
    except (OSError, ValueError):
        return {"t": time.time(), **state, **unknown_print}
    stats = status["print_stats"]
    return {
        "t": time.time(),
        **state,
        "print": stats["state"],
        "print_message": stats.get("message"),
        "file_position": status["virtual_sdcard"]["file_position"],
    }


def marker_offsets(scenario: Scenario, lines: list[str]) -> dict[str, int]:
    offsets: dict[str, list[int]] = {}
    position = 0
    for line in lines:
        offsets.setdefault(line, []).append(position)
        position += len(line.encode()) + 1
    markers = {}
    for action in scenario.timeline:
        if action.after is not After.LINE_READ:
            continue
        found = offsets.get(action.marker, [])
        if len(found) != 1:
            raise run.BenchError(
                f"{scenario.name}: marker {action.marker!r} occurs "
                f"{len(found)} times in the gcode; it must occur once"
            )
        markers[action.marker] = found[0]
    return markers


def perform(action: Action, remote_name: str) -> None:
    if action.act is Act.START_PRINT:
        quoted = urllib.parse.quote(remote_name)
        pi.http(
            "POST",
            f"/printer/print/start?filename={quoted}",
            timeout_s=ACTION_TIMEOUT_S,
        )
        return
    if action.act is Act.GCODE:
        pi.gcode_script(action.script, ACTION_TIMEOUT_S)
        return
    pi.http("POST", ACTION_PATHS[action.act], timeout_s=ACTION_TIMEOUT_S)


def post_check() -> dict:
    started = time.time()
    lines = []
    for line in POST_CHECK:
        try:
            pi.gcode_script(line, ACTION_TIMEOUT_S)
        except (OSError, ValueError) as exc:
            lines.append(
                {"line": line, "ok": False, "error": describe_error(exc)}
            )
            break
        lines.append({"line": line, "ok": True, "error": None})
    return {"started": started, "lines": lines, "after": sample()}


class Session:
    def __init__(
        self, scenario: Scenario, remote_name: str, markers: dict[str, int]
    ):
        self.scenario = scenario
        self.remote_name = remote_name
        self.markers = markers
        self.record: dict = {
            "scenario": scenario.name,
            "actions": [],
            "samples": [],
            "post_check": None,
            "ended": None,
        }
        self.last_sent = 0.0

    def drive(self, capture: Capture) -> dict:
        deadline = time.monotonic() + TIMELINE_TIMEOUT_S
        self.last_sent = time.time()
        with ThreadPoolExecutor(len(self.scenario.timeline)) as requests:
            for action in self.scenario.timeline:
                self._wait(
                    self._due(action), capture, deadline, action.act.value
                )
                self._fire(action, requests)
            self._wait(self._settled, capture, deadline, "settle")
        settled = self.record["samples"][-1]
        self.record["post_check"] = post_check()
        self.record["ended"] = time.time()
        return {"state": settled["print"], "message": settled["print_message"]}

    def _wait(
        self, due: DueTime, capture: Capture, deadline: float, what: str
    ) -> None:
        while True:
            capture.check()
            self.record["samples"].append(sample())
            when = due()
            if when is not None and time.time() >= when:
                return
            if time.monotonic() > deadline:
                raise run.BenchError(
                    f"{self.scenario.name}: waiting for {what} timed out "
                    f"after {TIMELINE_TIMEOUT_S:.0f} s of timeline"
                )
            time.sleep(POLL_S)

    def _fire(self, action: Action, requests: ThreadPoolExecutor) -> None:
        entry = {
            "act": action.act.value,
            "after": action.after.value,
            "delay_s": action.delay_s,
            "script": action.script,
            "sent": time.time(),
            "returned": None,
            "ok": None,
            "error": None,
        }
        self.record["actions"].append(entry)
        self.last_sent = entry["sent"]
        if action.act is Act.WAIT:
            entry.update(returned=entry["sent"], ok=True)
            return
        requests.submit(self._request, action, entry)

    def _request(self, action: Action, entry: dict) -> None:
        try:
            perform(action, self.remote_name)
            entry["ok"] = True
        except (OSError, ValueError) as exc:
            entry.update(ok=False, error=describe_error(exc))
        entry["returned"] = time.time()

    def _due(self, action: Action) -> DueTime:
        if action.after is After.PREVIOUS:
            return lambda: self.last_sent + action.delay_s
        if action.after is After.PRINT_END:
            return lambda: self._plus(self._print_ended_at(), action.delay_s)
        offset = self.markers[action.marker]
        return lambda: self._plus(self._line_read_at(offset), action.delay_s)

    @staticmethod
    def _plus(when: float | None, delay_s: float) -> float | None:
        return None if when is None else when + delay_s

    def _since_last_start(self) -> list[dict]:
        starts = [
            entry
            for entry in self.record["actions"]
            if entry["act"] == Act.START_PRINT.value
        ]
        if not starts or starts[-1]["returned"] is None:
            return []
        returned = starts[-1]["returned"]
        return [s for s in self.record["samples"] if s["t"] > returned]

    def _print_ended_at(self) -> float | None:
        for state in self._since_last_start():
            if state["print"] in ENDED_PRINT:
                return state["t"]
        return None

    def _line_read_at(self, offset: int) -> float | None:
        for state in self._since_last_start():
            ended = state["print"] in ENDED_PRINT
            if ended or (
                state["print"] in ACTIVE_PRINT
                and state["file_position"] >= offset
            ):
                return state["t"]
        return None

    def _settled(self) -> float | None:
        if any(entry["returned"] is None for entry in self.record["actions"]):
            return None
        settled_since = None
        for state in self._since_last_start():
            if (
                state["klippy"] in SETTLED_KLIPPY
                and state["print"] in ENDED_PRINT
            ):
                settled_since = settled_since or state["t"]
            else:
                settled_since = None
        return self._plus(settled_since, SETTLE_S)


def run_scenario(scenario: Scenario, runs_dir: Path) -> Path:
    lines = scenario.gcode.lines()
    markers = marker_offsets(scenario, lines)
    run_dir, remote_name, summary = run.start_run(
        lines,
        runs_dir,
        scenario.config.text(),
        {
            "scenario": scenario.name,
            "source_gcode": scenario.gcode.describe(),
            "config": scenario.config.describe(),
        },
    )
    session = Session(scenario, remote_name, markers)
    summary["session"] = session.record
    run.execute(run_dir, summary, False, False, False, None, session.drive)
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=sorted(SCENARIO_BY_NAME))
    parser.add_argument(
        "--runs-dir", type=Path, default=Path("~/bench/runs").expanduser()
    )
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    with lock.bench_lock(args.lock_timeout):
        print(
            run_scenario(
                SCENARIO_BY_NAME[args.scenario], args.runs_dir.expanduser()
            )
        )


if __name__ == "__main__":
    pi.run_main(main)
