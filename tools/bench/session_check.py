"""Judge a bench session run (session.py) by its scenario's checks.

uv run python tools/bench/session_check.py <run_dir>
uv run python tools/bench/session_check.py --evidence <session_evidence.json>
uv run python tools/bench/session_check.py --run-wide <run_dir>

From a run dir it writes session_evidence.json (session_evidence.py) and
session_checks.json next to run.json. Prints one line per check and exits
1 when any fails. Every scenario gets the generic checks; the stop latency,
engine account, second-print, pacer-reserve and soak (SOAK_CHECKS: pacer
brakes and reserve raises outside commanded stops, klippy Stats) checks are
declared per scenario. --run-wide judges a plain run.py run, which has no
timeline, by the generic checks that need none plus SOAK_CHECKS.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path

import session_evidence
from session_scenarios import (
    MID_PRINT_GAP_S,
    PACER_ISSUE,
    REACTOR_STALL_ISSUE,
    SCENARIO_BY_NAME,
    SOAK_CHECKS,
    TERMINATING_ACTS,
    Act,
    NoStrayBrakes,
    NoStrayReserveRaises,
    ReserveBounded,
    Scenario,
    SecondPrintLikeFirst,
    SpecificCheck,
    StatsSteady,
    StopsWithin,
    StopsWithinEngineAccount,
)

EVIDENCE_FILE = "session_evidence.json"
CHECKS_FILE = "session_checks.json"
STOPPED_QUIET_S = 1.0
PI_CLOCK_SKEW_S = 0.05
FRESH_STREAM = "anchor_decision"
IN_FLIGHT_SLACK_S = session_evidence.SPAN_JOIN_S
TERMINATING = frozenset(act.value for act in TERMINATING_ACTS)
STOPPING = frozenset({Act.PAUSE.value, *TERMINATING})
PAUSE_ENDS = frozenset({Act.RESUME.value, Act.CANCEL.value})
ESTOP_SHUTDOWN_REASONS = frozenset(
    {"Shutdown due to webhooks request", "Command request"}
)
SHUTDOWN_EVENTS = frozenset({"klippy_shutdown", "mcu_shutdown"})
JOURNAL_LINES_SHOWN = 3
CLOSING_BRAKE_CAUSE = "RunwayExhausted"
PACER_BRAKE = "pipe_drain"
RESERVE_RAISED = "pacer_reserve_raised"
RUN_WIDE = "run-wide"


@dataclass(frozen=True)
class Verdict:
    name: str
    ok: bool
    detail: str


@dataclass(frozen=True)
class Print:
    start: float
    end: float | None
    outcome: str | None


class Evidence:
    def __init__(self, evidence: dict):
        self.states = evidence["states"]
        self.actions = evidence["actions"]
        self.spans = evidence["spans"]
        self.events = evidence["events"]
        self.stats = evidence.get("stats")
        self.journal = evidence["journal"]
        self.post_check = evidence["post_check"]
        self.window_start, self.window_end = evidence["window"]
        starts = self.sent(Act.START_PRINT)
        self.origin = starts[0] if starts else self.window_start

    def at(self, wall_s: float) -> str:
        return f"+{wall_s - self.origin:.1f} s"

    def sent(self, act: Act) -> list[float]:
        return [a["sent"] for a in self.actions if a["act"] == act.value]

    def named(self, event: str) -> list[dict]:
        return [e for e in self.events if e["event"] == event]

    def settled_at(self) -> float:
        if self.post_check is None:
            return self.window_end
        return self.post_check["started"]

    def next_boundary(self, after: float) -> float:
        later = [
            a["sent"]
            for a in self.actions
            if a["sent"] > after and a["act"] != Act.WAIT.value
        ]
        return min([*later, self.settled_at()])

    def commanded(self, wall_s: float) -> bool:
        marks = [a["sent"] for a in self.actions if a["act"] in STOPPING]
        return any(
            mark - PI_CLOCK_SKEW_S
            <= wall_s
            <= self.next_boundary(mark) + PI_CLOCK_SKEW_S
            for mark in marks
        )

    def stray(self, event: str) -> list[dict]:
        return [e for e in self.named(event) if not self.commanded(e["t"])]

    def prints(self) -> list[Print]:
        prints: list[Print] = []
        for event in self.events:
            if event["event"] == "print.start":
                prints.append(Print(event["t"], None, None))
            elif event["event"] == "print.end" and prints:
                started = prints[-1].start
                prints[-1] = Print(started, event["t"], event["outcome"])
        return prints

    def print_window(
        self, index: int, prints: list[Print]
    ) -> tuple[float, float]:
        current = prints[index]
        stops = [
            a["sent"]
            for a in self.actions
            if a["act"] in TERMINATING and a["sent"] > current.start
        ]
        next_start = [p.start for p in prints[index + 1 : index + 2]]
        ends = [current.end, *stops, *next_start, self.settled_at()]
        return current.start, min(t for t in ends if t is not None)

    def pauses(self) -> list[tuple[float, float]]:
        intervals = []
        for paused in self.sent(Act.PAUSE):
            ends = [
                a["sent"]
                for a in self.actions
                if a["sent"] > paused and a["act"] in PAUSE_ENDS
            ]
            intervals.append((paused, min([*ends, self.window_end])))
        return intervals

    def spans_within(self, lo: float, hi: float) -> list[list]:
        return [s for s in self.spans if lo <= s[0] < hi]

    def anchored_between(self, lo: float, hi: float) -> bool:
        return any(
            lo - PI_CLOCK_SKEW_S <= anchor["t"] <= hi + PI_CLOCK_SKEW_S
            for anchor in self.named(FRESH_STREAM)
        )

    def next_action_moved(self, ended: float, boundary: float) -> bool:
        following = [s for s in self.spans if s[0] > ended]
        if not following or following[0][0] < boundary:
            return False
        return self.anchored_between(max(ended, boundary), following[0][0])

    def ready_before_post_check(self) -> str | None:
        before = [s for s in self.states if s["t"] <= self.settled_at()]
        return before[-1]["klippy"] if before else None


def actions_check(evidence: Evidence) -> Verdict:
    failed = [
        f"{a['act']} at {evidence.at(a['sent'])}: "
        f"{a['error'] or 'never returned'}"
        for a in evidence.actions
        if not a["ok"]
    ]
    if failed:
        return Verdict("actions", False, "; ".join(failed))
    return Verdict("actions", True, f"{len(evidence.actions)} actions ok")


def outcomes_check(evidence: Evidence, scenario: Scenario) -> Verdict:
    if evidence.post_check is None:
        return Verdict("outcomes", False, "the session never settled")
    ended = [p.outcome for p in evidence.prints() if p.outcome is not None]
    wanted = [outcome.value for outcome in scenario.outcomes]
    klippy = evidence.ready_before_post_check()
    detail = (
        f"prints ended {', '.join(ended) or 'none'} (expected "
        f"{', '.join(wanted)}); klippy {klippy} before the post-check"
    )
    return Verdict("outcomes", ended == wanted and klippy == "ready", detail)


def journal_check(evidence: Evidence) -> Verdict:
    if evidence.journal:
        shown = " | ".join(evidence.journal[:JOURNAL_LINES_SHOWN])
        return Verdict("journal", False, f"klippy aborted: {shown}")
    return Verdict("journal", True, "no klippy abort in the journal")


def shutdown_check(evidence: Evidence) -> Verdict:
    stops = evidence.sent(Act.EMERGENCY_STOP)
    unexpected = [
        f"{e['event']} at {evidence.at(e['t'])}: {e['reason']}"
        for e in evidence.events
        if e["event"] in SHUTDOWN_EVENTS
        and not (
            e["reason"] in ESTOP_SHUTDOWN_REASONS
            and any(stop <= e["t"] + PI_CLOCK_SKEW_S for stop in stops)
        )
    ]
    if unexpected:
        return Verdict("shutdown", False, "; ".join(unexpected))
    allowed = "the commanded e-stop only" if stops else "none"
    return Verdict("shutdown", True, f"shutdowns: {allowed}")


def reactor_check(evidence: Evidence) -> Verdict:
    stalls = [
        f"late_s {e['late_s']} at {evidence.at(e['t'])}"
        for e in evidence.named("reactor_poller_late")
    ]
    if stalls:
        detail = (
            f"reactor_poller_late {', '.join(stalls)} ({REACTOR_STALL_ISSUE})"
        )
        return Verdict("reactor", False, detail)
    return Verdict("reactor", True, "no reactor_poller_late")


def mid_print_gaps(evidence: Evidence) -> list[tuple[float, float]]:
    prints = evidence.prints()
    pauses = evidence.pauses()
    gaps = []
    for index in range(len(prints)):
        spans = evidence.spans_within(*evidence.print_window(index, prints))
        for before, after in zip(spans, spans[1:]):
            if not any(lo <= before[1] <= hi for lo, hi in pauses):
                gaps.append((after[0] - before[1], before[1]))
    return gaps


def gaps_check(evidence: Evidence) -> Verdict:
    gaps = mid_print_gaps(evidence)
    longest = max((gap for gap, _ in gaps), default=0.0)
    detail = MID_PRINT_GAP_S.judge("longest mid-print gap", longest)
    long_gaps = [
        f"{gap:.2f} s at {evidence.at(at)}"
        for gap, at in gaps
        if not MID_PRINT_GAP_S.holds(gap)
    ]
    if long_gaps:
        detail += f": {', '.join(long_gaps)}"
    return Verdict("gaps", not long_gaps, detail)


def post_check_check(evidence: Evidence) -> Verdict:
    post = evidence.post_check
    if post is None:
        return Verdict("post_check", False, "no post-check ran")
    failed = [
        f"{line['line']}: {line['error']}"
        for line in post["lines"]
        if not line["ok"]
    ]
    klippy = post["after"]["klippy"]
    if failed or klippy != "ready":
        detail = "; ".join([*failed, f"klippy {klippy} after"])
        return Verdict("post_check", False, detail)
    lines = len(post["lines"])
    return Verdict("post_check", True, f"{lines} lines ok, klippy ready")


def stop_latency(evidence: Evidence, mark: float) -> tuple[float | None, str]:
    boundary = evidence.next_boundary(mark)
    spans = [s for s in evidence.spans if s[0] < boundary]
    flying = [
        index
        for index, span in enumerate(spans)
        if span[0] <= mark <= span[1] + IN_FLIGHT_SLACK_S
    ]
    if not flying:
        return None, "no motion in flight when it was sent"
    for index in range(flying[0], len(spans)):
        ended = spans[index][1]
        resumed = spans[index + 1][0] if index + 1 < len(spans) else boundary
        if resumed - ended >= STOPPED_QUIET_S or evidence.next_action_moved(
            ended, boundary
        ):
            return max(ended - mark, 0.0), ""
    return None, f"motion did not stop before {evidence.at(boundary)}"


def stop_check(evidence: Evidence, check: StopsWithin) -> Verdict:
    name = f"stop_after_{check.act.value}"
    marks = evidence.sent(check.act)
    if not marks:
        return Verdict(name, False, f"no {check.act.value} was sent")
    ok = True
    details = []
    for mark in marks:
        latency, why = stop_latency(evidence, mark)
        if latency is None:
            ok = False
            details.append(
                f"at {evidence.at(mark)}: {why} ({check.bound.issue})"
            )
            continue
        ok = ok and check.bound.holds(latency)
        judged = check.bound.judge("stopped after", latency)
        details.append(f"at {evidence.at(mark)}: {judged}")
    return Verdict(name, ok, "; ".join(details))


def closing_brake(evidence: Evidence, mark: float) -> dict | None:
    boundary = evidence.next_boundary(mark)
    brakes = [
        brake
        for brake in evidence.named("pipe_drain")
        if mark <= brake["t"] <= boundary
        and brake["cause"] == CLOSING_BRAKE_CAUSE
    ]
    return brakes[0] if brakes else None


def stop_accounted(evidence: Evidence, request: dict) -> tuple[bool, str]:
    mark = request["sent"]
    latency, why = stop_latency(evidence, mark)
    if latency is None:
        return False, why
    if request["returned"] is None:
        return False, "the request never returned"
    brake = closing_brake(evidence, mark)
    if brake is None:
        boundary = evidence.at(evidence.next_boundary(mark))
        return False, (
            f"no closing pacer brake (pipe_drain {CLOSING_BRAKE_CAUSE}) "
            f"before {boundary}"
        )
    after_feed = mark + latency - request["returned"]
    account = brake["peak_ahead_s"] + brake["drained_s"]
    held = after_feed <= account
    relation = "<=" if held else ">"
    return held, (
        f"moved {after_feed:.3f} s after the request returned {relation} "
        f"the engine account {account:.3f} s (peak_ahead_s "
        f"{brake['peak_ahead_s']:.3f} + drained_s {brake['drained_s']:.3f})"
    )


def account_check(
    evidence: Evidence, check: StopsWithinEngineAccount
) -> Verdict:
    name = f"stop_account_after_{check.act.value}"
    requests = [a for a in evidence.actions if a["act"] == check.act.value]
    if not requests:
        return Verdict(name, False, f"no {check.act.value} was sent")
    ok = True
    details = []
    for request in requests:
        held, detail = stop_accounted(evidence, request)
        ok = ok and held
        details.append(f"at {evidence.at(request['sent'])}: {detail}")
    return Verdict(name, ok, "; ".join(details))


def motion_s(evidence: Evidence, printed: Print) -> float:
    spans = evidence.spans_within(printed.start, printed.end)
    return spans[-1][1] - spans[0][0] if spans else 0.0


def second_print_check(
    evidence: Evidence, check: SecondPrintLikeFirst
) -> Verdict:
    done = [p for p in evidence.prints() if p.outcome == "complete"]
    if len(done) < 2:
        detail = f"needs two completed prints, got {len(done)}"
        return Verdict("second_print", False, detail)
    first, last = motion_s(evidence, done[0]), motion_s(evidence, done[-1])
    if first == 0.0:
        return Verdict("second_print", False, "the first print never moved")
    change = abs(last - first) / first
    judged = check.bound.judge("relative change", change)
    detail = f"motion {last:.1f} s vs the first print's {first:.1f} s, {judged}"
    return Verdict("second_print", check.bound.holds(change), detail)


def reserve_check(evidence: Evidence, check: ReserveBounded) -> Verdict:
    raised = evidence.named("pacer_reserve_raised")
    if not raised:
        return Verdict("pacer_reserve", True, "no pacer_reserve_raised")
    worst = max(raised, key=lambda e: e["reserve_s"])
    judged = check.bound.judge("pacer reserve", worst["reserve_s"])
    detail = f"{judged} at {evidence.at(worst['t'])}"
    return Verdict(
        "pacer_reserve", check.bound.holds(worst["reserve_s"]), detail
    )


def brakes_check(evidence: Evidence, check: NoStrayBrakes) -> Verdict:
    strays = evidence.stray(PACER_BRAKE)
    at_stops = len(evidence.named(PACER_BRAKE)) - len(strays)
    if strays:
        listed = ", ".join(
            f"{e['cause']} at {evidence.at(e['t'])}" for e in strays
        )
        detail = (
            f"{len(strays)} pacer brakes outside commanded stops "
            f"({check.issue}): {listed}"
        )
        return Verdict("pacer_brakes", False, detail)
    detail = f"no pacer brake outside commanded stops ({at_stops} at them)"
    return Verdict("pacer_brakes", True, detail)


def reserve_growth_check(
    evidence: Evidence, check: NoStrayReserveRaises
) -> Verdict:
    strays = evidence.stray(RESERVE_RAISED)
    if strays:
        listed = ", ".join(
            f"to {e['reserve_s']:.3f} s at {evidence.at(e['t'])}"
            for e in strays
        )
        detail = (
            f"{RESERVE_RAISED} {len(strays)} times outside commanded stops "
            f"({check.issue}): {listed}"
        )
        return Verdict("pacer_reserve_growth", False, detail)
    detail = f"no {RESERVE_RAISED} outside commanded stops"
    return Verdict("pacer_reserve_growth", True, detail)


def stats_check(evidence: Evidence, check: StatsSteady) -> Verdict:
    stats = evidence.stats
    if not stats:
        return Verdict("klippy_stats", False, "no klippy Stats line in the run")
    sessions = {row["session_id"] for row in stats}
    times = [
        evidence.window_start,
        *(row["t"] for row in stats),
        evidence.window_end,
    ]
    longest = max(later - earlier for earlier, later in zip(times, times[1:]))
    first, last = stats[0], stats[-1]
    detail = (
        f"{len(stats)} Stats lines from {len(sessions)} klippy session(s); "
        f"{check.bound.judge('longest gap', longest)}; memavail "
        f"{first['memavail_kb']} -> {last['memavail_kb']} kB; klippy cputime "
        f"+{last['cputime_s'] - first['cputime_s']:.1f} s"
    )
    return Verdict(
        "klippy_stats",
        len(sessions) == 1 and check.bound.holds(longest),
        detail,
    )


def specific_check(evidence: Evidence, check: SpecificCheck) -> Verdict:
    if isinstance(check, StopsWithin):
        return stop_check(evidence, check)
    if isinstance(check, StopsWithinEngineAccount):
        return account_check(evidence, check)
    if isinstance(check, SecondPrintLikeFirst):
        return second_print_check(evidence, check)
    if isinstance(check, NoStrayBrakes):
        return brakes_check(evidence, check)
    if isinstance(check, NoStrayReserveRaises):
        return reserve_growth_check(evidence, check)
    if isinstance(check, StatsSteady):
        return stats_check(evidence, check)
    return reserve_check(evidence, check)


def evaluate(scenario: Scenario, evidence: dict) -> list[Verdict]:
    view = Evidence(evidence)
    return [
        actions_check(view),
        outcomes_check(view, scenario),
        journal_check(view),
        shutdown_check(view),
        reactor_check(view),
        gaps_check(view),
        post_check_check(view),
        *(specific_check(view, check) for check in scenario.checks),
    ]


def evaluate_run(evidence: dict) -> list[Verdict]:
    view = Evidence(evidence)
    return [
        journal_check(view),
        shutdown_check(view),
        reactor_check(view),
        *(specific_check(view, check) for check in SOAK_CHECKS),
    ]


def judge(evidence: dict) -> dict:
    if evidence["scenario"] is None:
        verdicts = evaluate_run(evidence)
        return {
            "scenario": None,
            "guards": f"{REACTOR_STALL_ISSUE} {PACER_ISSUE}",
            "not_analyzed": None,
            "ok": all(v.ok for v in verdicts),
            "checks": [asdict(v) for v in verdicts],
        }
    scenario = SCENARIO_BY_NAME[evidence["scenario"]]
    verdicts = evaluate(scenario, evidence)
    return {
        "scenario": scenario.name,
        "guards": scenario.guards,
        "not_analyzed": scenario.not_analyzed.value,
        "ok": all(v.ok for v in verdicts),
        "checks": [asdict(v) for v in verdicts],
    }


def judge_run_dir(
    run_dir: Path,
    collect: Callable[[Path], dict] = session_evidence.collect,
) -> dict:
    evidence = collect(run_dir)
    (run_dir / EVIDENCE_FILE).write_text(json.dumps(evidence, indent=1))
    verdict = judge(evidence)
    (run_dir / CHECKS_FILE).write_text(json.dumps(verdict, indent=2))
    return verdict


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("run_dir", nargs="?", type=Path)
    source.add_argument("--evidence", type=Path)
    source.add_argument("--run-wide", type=Path, metavar="RUN_DIR")
    args = parser.parse_args()
    if args.run_wide is not None:
        verdict = judge_run_dir(args.run_wide, session_evidence.collect_run)
    elif args.evidence is None:
        verdict = judge_run_dir(args.run_dir)
    else:
        verdict = judge(json.loads(args.evidence.read_text()))
    for check in verdict["checks"]:
        result = "PASS" if check["ok"] else "FAIL"
        print(f"{result} {check['name']}: {check['detail']}")
    result = "PASS" if verdict["ok"] else "FAIL"
    scenario = verdict["scenario"] or RUN_WIDE
    print(f"{scenario} (guards {verdict['guards']}): {result}")
    sys.exit(0 if verdict["ok"] else 1)


if __name__ == "__main__":
    main()
