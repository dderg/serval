#!/usr/bin/env python3
"""Qualify a landed commit on the bench: the slow hardware tier.

    uv run python scripts/qualify.py run [REV] [--expect-fail ROW=ISSUE ...]
    uv run python scripts/qualify.py status [--line]
    uv run python scripts/qualify.py bisect --row NAME [--good TAG] [--bad REV]

PRs land on CI alone, so main is not stable: the last commit a
qualification passed is, tagged qualified-<UTC date>[-n]. Print from the
latest qualified-* tag, not from main.

`run` checks REV (default origin/main) out clean in the scratch tree
RUN_TREE (cargo target pending_baselines.target_dir_for(RUN_TREE)), builds
its engine and runs its units in order, stopping at the first that fails.
`run` never holds the bench lock: each unit's tool takes its own holds of
at most lock.MAX_HOLD_S, each restoring the bench it starts from:
- deploy: deploy.py build-target without the lock, then one hold
  (`qualify.py deploy`) of deploy.py settle, sync and ensure-target; the
  F401 must report REV's firmware build id and the deploy record name it;
- suite-<k>: suite.py --row ... for each hold tools/bench/hold_plan.py
  plans over the full suite; a row may fail only through --expect-fail
  ROW=ISSUE, recorded with its issue;
- soak-<row>: suite.py --row ROW for each soak row;
- pi: throughput_pi.py on REV's tree, judged without the lock by
  `throughput_gate.py --qualify pi-window`;
- calibration: `throughput_gate.py --qualify calibration`; a stale one is
  re-measured by throughput_recheck.py, judged again and committed on a
  local branch for a normal PR (qualification never pushes commits).
Every unit leaves QUALIFICATIONS/<rev>/units/<unit>.json (its key, verdict,
times, holds, facts) and <unit>.log. The key is REV, the F401 target build
id, the unit's rows and their --expect-fail issues, and the sha256 of the
Pi identity read over ssh without the lock (serial, model, kernel, OS, Pi
firmware, dpkg status, boot config, rustc, klippy's python, the capture
board build). A rerun of `run REV` skips a unit that passed with the same
key within QUALIFY_WINDOW_S, so a lost hold costs only its unit, and
rewrites QUALIFICATIONS/<rev>/qualification.json. When every unit passed
with the current key within the window, an annotated tag on REV carries the
JSON summary (parse_message reads it back): the units with their holds, the
bench seconds and the Pi identity. A failure tags nothing and prints the
failed unit, its rows and the bisect command per failed row.

`status` prints the last qualified tag and the qualification debt: the
first-parent merges on origin/main since it, and how many touch each of
DEBT_AREAS. `status --line` is the one line land-pr.sh prints after a
landing.

`bisect` searches the first-parent merges after the last qualified tag (or
--good) up to --bad (default origin/main) for the first on which one suite
row fails: per candidate, the row as --bad defines it runs from RUN_TREE
through `suite.py --row NAME --tree CANDIDATE_TREE` in a hold suite.py
takes, the candidate checked out clean, its engine built and synced the way
ab.py deploys a ref.
"""

from __future__ import annotations

import argparse
import datetime
import enum
import hashlib
import importlib
import itertools
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType

import pending_baselines as pending

SCRIPT = Path(__file__).resolve()
MAIN = "origin/main"
TAG_PREFIX = "qualified-"
DEBT_AREAS = ("rust/", "src/", "klippy/", "tools/bench/")
QUALIFICATIONS = Path("~/bench/qualifications").expanduser()
SCRATCH = pending.CACHE / "qualify"
RUN_TREE = SCRATCH / "run"
CANDIDATE_TREE = SCRATCH / "candidate"
TARGET_PURPOSE = "qualify"
LOCK_NAME = "qualify"
LOCK_TIMEOUT_S = 14400
QUALIFY_WINDOW_S = 24 * 60 * 60
SETTLE_WORST_S = 2 * 60
SYNC_AND_PI_BUILD_WORST_S = 8 * 60
F401_REFLASH_S = 3 * 60
DEPLOY_HOLD_S = SETTLE_WORST_S + SYNC_AND_PI_BUILD_WORST_S + F401_REFLASH_S
RUN_COMMAND = "uv run python scripts/qualify.py run"
BENCH = Path("tools") / "bench"
F401_CONFIG = BENCH / "f401.config"
PI_WINDOW = BENCH / "pi_window.json"
BASELINE = BENCH / "throughput_baseline.json"
UV_PYTHON = ("uv", "run", "python")
DEPLOY_TOOL = (*UV_PYTHON, str(BENCH / "deploy.py"))
ENGINE_BUILD = ("make", "-f", "Makefile.rust", "motion-engine-fast")
GATE = ("python3", "scripts/throughput_gate.py", "--qualify")
SUMMARY_LINE = re.compile(r"^summary: (\S+summary\.json)$", re.M)
SETTLED_LINE = "settled: "
FAIL_LINE = "FAIL: "
PR_MERGE = re.compile(r"^Merge pull request #(\d+)")
EXPECT_FAIL = re.compile(r"^(?P<row>\w+)=#?(?P<issue>\d+)$")
RECORD_FILE = "qualification.json"
UNITS_DIR = "units"
DEPLOY = "deploy"
DEPLOY_HANDOFF = "deploy-hold.json"
FIELD_SEP, RECORD_SEP = "\x1f", "\x1e"
PI_FACTS = {
    "serial": "tr -d '\\0' < /proc/device-tree/serial-number",
    "model": "tr -d '\\0' < /proc/device-tree/model",
    "kernel": "uname -r",
    "os": '. /etc/os-release && echo "$PRETTY_NAME"',
    "pi_firmware": "vcgencmd version",
    "rust_toolchains": ". ~/.cargo/env && rustup toolchain list",
    "klippy_python": "~/klippy-env/bin/python -V",
}
PI_FILES = {
    "dpkg_status": "/var/lib/dpkg/status",
    "config_txt": "/boot/firmware/config.txt",
    "cmdline_txt": "/boot/firmware/cmdline.txt",
}


class QualifyError(RuntimeError):
    pass


class View(enum.Enum):
    TABLE = "table"
    LINE = "line"


class Detail(enum.Enum):
    TALLIES = "tallies"
    ROWS = "rows"


def git(*args: str, cwd: Path | None = None) -> str:
    result = subprocess.run(
        ["git", *args], cwd=cwd, capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise QualifyError(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout.strip()


def git_net(*args: str, cwd: Path | None = None) -> str:
    try:
        return pending.run(["git", *args], cwd=cwd or Path.cwd())
    except pending.PendingBaselinesError as exc:
        raise QualifyError(str(exc)) from exc


@dataclass(frozen=True)
class Merge:
    sha: str
    subject: str
    paths: tuple[str, ...]

    @property
    def pr(self) -> int | None:
        match = PR_MERGE.match(self.subject)
        return int(match.group(1)) if match else None


def last_tag(ref: str, cwd: Path | None = None) -> str | None:
    tags = git(
        "tag",
        "--list",
        f"{TAG_PREFIX}*",
        "--merged",
        ref,
        "--sort=-creatordate",
        cwd=cwd,
    ).splitlines()
    return tags[0] if tags else None


def parse_log(log: str) -> list[Merge]:
    merges = []
    for record in filter(None, log.split(RECORD_SEP)):
        head, _, names = record.partition("\n")
        sha, _, subject = head.partition(FIELD_SEP)
        paths = tuple(name for name in names.splitlines() if name)
        merges.append(Merge(sha, subject, paths))
    return merges


def first_parents(since: str, ref: str, cwd: Path | None = None) -> list[Merge]:
    log = git(
        "log",
        "--first-parent",
        f"--format={RECORD_SEP}%H{FIELD_SEP}%s",
        "--name-only",
        "--diff-merges=first-parent",
        f"{since}..{ref}",
        cwd=cwd,
    )
    return parse_log(log)


def debt(merges: list[Merge]) -> dict[str, int]:
    touched = {
        area: sum(
            any(path.startswith(area) for path in merge.paths)
            for merge in merges
        )
        for area in DEBT_AREAS
    }
    return {"merges": len(merges), **touched}


def areas_of(merge: Merge) -> str:
    return " ".join(
        area
        for area in DEBT_AREAS
        if any(path.startswith(area) for path in merge.paths)
    )


def status(view: View, ref: str = MAIN, cwd: Path | None = None) -> str:
    tag = last_tag(ref, cwd)
    if tag is None:
        if view is View.LINE:
            return f"no {TAG_PREFIX}* tag on {ref} yet; run {RUN_COMMAND}"
        total = git("rev-list", "--first-parent", "--count", ref, cwd=cwd)
        return (
            f"no {TAG_PREFIX}* tag on {ref} yet: none of its {total} "
            f"first-parent commits is qualified; run {RUN_COMMAND}"
        )
    merges = first_parents(tag, ref, cwd)
    owed = debt(merges)
    if view is View.LINE:
        return f"{owed['merges']} merges since {tag}; run {RUN_COMMAND}"
    tagged = git("rev-list", "-n", "1", tag, cwd=cwd)
    date = git("tag", "-l", "--format=%(creatordate:iso-strict)", tag, cwd=cwd)
    areas = ", ".join(f"{area} {owed[area]}" for area in DEBT_AREAS)
    lines = [
        f"last qualified: {tag} at {tagged[:12]}, tagged {date}",
        f"debt: {owed['merges']} first-parent merges on {ref} since it; "
        f"touching {areas}",
    ]
    lines += [
        f"  {m.sha[:12]} #{m.pr} {m.subject} [{areas_of(m)}]" for m in merges
    ]
    return "\n".join(lines)


def next_tag(taken: set[str], day: datetime.date) -> str:
    base = f"{TAG_PREFIX}{day.isoformat()}"
    if base not in taken:
        return base
    return next(
        f"{base}-{n}" for n in itertools.count(2) if f"{base}-{n}" not in taken
    )


def tag_message(summary: dict) -> str:
    body = json.dumps(summary, indent=2, sort_keys=True)
    return f"Qualify {summary['rev'][:12]}\n\n{body}\n"


def parse_message(message: str) -> dict:
    _, _, body = message.partition("\n\n")
    return json.loads(body)


def create_tag(name: str, rev: str, summary: dict, cwd: Path | None = None):
    with tempfile.NamedTemporaryFile("w", suffix=".txt") as message:
        message.write(tag_message(summary))
        message.flush()
        git(
            "tag",
            "-a",
            "--cleanup=verbatim",
            "-F",
            message.name,
            name,
            rev,
            cwd=cwd,
        )


def read_tag(name: str, cwd: Path | None = None) -> dict:
    return parse_message(
        git("tag", "-l", "--format=%(contents)", name, cwd=cwd)
    )


def expect_fail(values: list[str]) -> dict[str, int]:
    parsed = {}
    for value in values:
        match = EXPECT_FAIL.match(value)
        if match is None:
            raise QualifyError(
                f"--expect-fail {value!r}: give ROW=ISSUE (e.g. "
                "pause_sparse=#860); a tolerated failure names the issue "
                "that owns it"
            )
        parsed[match["row"]] = int(match["issue"])
    return parsed


def first_bad(candidates: list[str], passes: Callable[[str], bool]) -> str:
    """The first of `candidates` (oldest first, the commit before them good)
    the row fails on; the last must fail, else there is nothing to find."""
    if passes(candidates[-1]):
        raise QualifyError(
            f"the row passes at {candidates[-1][:12]}, the bad end: it is "
            "flaky or already fixed, nothing to bisect"
        )
    low, high = 0, len(candidates) - 1
    while low < high:
        middle = (low + high) // 2
        if passes(candidates[middle]):
            low = middle + 1
        else:
            high = middle
    return candidates[high]


@dataclass(frozen=True)
class Outcome:
    passed: bool
    facts: dict = field(default_factory=dict)
    failures: tuple[str, ...] = ()
    holds: tuple[dict, ...] = ()


def identity_sha256(identity: dict) -> str:
    canonical = json.dumps(identity, sort_keys=True).encode()
    return hashlib.sha256(canonical).hexdigest()


@dataclass(frozen=True)
class Context:
    rev: str
    tree: Path
    record: Path
    expected: dict[str, int]
    target_id: str
    identity: dict

    def key(self, unit: Unit) -> dict:
        return {
            "rev": self.rev,
            "target_build_id": self.target_id,
            "rows": list(unit.rows),
            "expect_fail": {
                row: self.expected[row]
                for row in sorted(unit.rows)
                if row in self.expected
            },
            "pi_identity_sha256": identity_sha256(self.identity),
        }


@dataclass(frozen=True)
class Unit:
    name: str
    run: Callable[[Context, Unit], Outcome]
    rows: tuple[str, ...] = ()
    detail: Detail = Detail.TALLIES


def utc_now() -> datetime.datetime:
    return datetime.datetime.now(datetime.timezone.utc)


def iso(moment: datetime.datetime) -> str:
    return moment.isoformat(timespec="seconds")


def hours(seconds: float) -> str:
    return f"{seconds / 3600:.1f} h"


def unit_path(record: Path, name: str) -> Path:
    return record / UNITS_DIR / f"{name}.json"


def recorded(record: Path, name: str) -> dict | None:
    path = unit_path(record, name)
    return json.loads(path.read_text()) if path.exists() else None


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def age_s(entry: dict, now: datetime.datetime) -> float:
    finished = datetime.datetime.fromisoformat(entry["finished"])
    return (now - finished).total_seconds()


def changed_basis(entry: dict, key: dict, identity: dict) -> list[str]:
    changed = [name for name in key if entry["key"].get(name) != key[name]]
    if "pi_identity_sha256" in changed:
        before = entry["pi_identity"]
        changed += [
            f"pi {name}"
            for name in identity
            if before.get(name) != identity[name]
        ]
    return changed


def stale_reason(
    entry: dict | None, key: dict, identity: dict, now: datetime.datetime
) -> str | None:
    if entry is None:
        return "no record"
    if not entry["passed"]:
        return f"failed at {entry['finished']}"
    changed = changed_basis(entry, key, identity)
    if changed:
        since = f"since it passed at {entry['finished']}"
        return f"{', '.join(changed)} changed {since}"
    if age_s(entry, now) > QUALIFY_WINDOW_S:
        return (
            f"passed {hours(age_s(entry, now))} ago, past the "
            f"{hours(QUALIFY_WINDOW_S)} window"
        )
    return None


def still_valid(entry: dict, now: datetime.datetime) -> str:
    return (
        f"passed at {entry['finished']}, {hours(age_s(entry, now))} ago "
        f"(window {hours(QUALIFY_WINDOW_S)}), with the same rev, F401 build, "
        "rows, expect-fail issues and Pi identity"
    )


def executed(unit: Unit, context: Context) -> dict:
    started = utc_now()
    try:
        outcome = unit.run(context, unit)
    except QualifyError as exc:
        outcome = Outcome(False, failures=(str(exc),))
    finished = utc_now()
    return {
        "unit": unit.name,
        "key": context.key(unit),
        "pi_identity": context.identity,
        "passed": outcome.passed,
        "started": iso(started),
        "finished": iso(finished),
        "seconds": round((finished - started).total_seconds()),
        "holds": list(outcome.holds),
        "facts": outcome.facts,
        "failures": list(outcome.failures),
    }


def run_units(
    units: list[Unit], context: Context, drift: dict[str, str]
) -> list[dict]:
    done = []
    for unit in units:
        entry = recorded(context.record, unit.name)
        now = utc_now()
        key = context.key(unit)
        reason = stale_reason(entry, key, context.identity, now)
        reason = reason or drift.get(unit.name)
        if reason is None:
            print(f"qualify: skip {unit.name}: {still_valid(entry, now)}")
            done.append(entry)
            continue
        print(f"qualify: unit {unit.name} ({reason})", flush=True)
        entry = executed(unit, context)
        write_json(unit_path(context.record, unit.name), entry)
        done.append(entry)
        if not entry["passed"]:
            break
    return done


def unpublishable(
    units: list[Unit], context: Context, now: datetime.datetime
) -> list[str]:
    reasons = []
    for unit in units:
        entry = recorded(context.record, unit.name)
        key = context.key(unit)
        reason = stale_reason(entry, key, context.identity, now)
        if reason is not None:
            reasons.append(f"{unit.name}: {reason}")
    return reasons


def published(entry: dict) -> dict:
    return {
        name: value
        for name, value in entry.items()
        if name not in ("key", "pi_identity")
    }


def summarized(units: list[Unit], context: Context, subject: str) -> dict:
    entries = [
        recorded(context.record, unit.name)
        or {"unit": unit.name, "missing": True, "holds": []}
        for unit in units
    ]
    holds = [hold for entry in entries for hold in entry["holds"]]
    acquired = [hold for hold in holds if hold["wait_s"] is not None]
    return {
        "rev": context.rev,
        "subject": subject,
        "record": str(context.record),
        "updated": iso(utc_now()),
        "target_build_id": context.target_id,
        "expect_fail": [
            {"row": row, "issue": issue}
            for row, issue in context.expected.items()
        ],
        "pi_identity": context.identity,
        "pi_identity_sha256": identity_sha256(context.identity),
        "bench_s": round(sum(hold["held_s"] for hold in holds), 1),
        "lock_wait_s": round(sum(hold["wait_s"] for hold in acquired), 1),
        "units": [published(entry) for entry in entries],
    }


def bench(tree: Path, name: str) -> ModuleType:
    path = str(tree / BENCH)
    if path not in sys.path:
        sys.path.insert(0, path)
    return importlib.import_module(name)


def tree_script(tree: Path, name: str) -> ModuleType:
    path = str(tree / "scripts")
    if path not in sys.path:
        sys.path.insert(0, path)
    return importlib.import_module(name)


def target_env(tree: Path) -> dict[str, str]:
    target = pending.target_dir_for(tree, TARGET_PURPOSE)
    return {**os.environ, "CARGO_TARGET_DIR": str(target)}


def stream(
    command: tuple[str, ...], cwd: Path, log: Path, target: Path
) -> tuple[int, str]:
    lines = []
    fds = bench(cwd, "lock").inherited_fds()
    with (
        log.open("a") as sink,
        subprocess.Popen(
            command,
            cwd=cwd,
            env=target_env(target),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            pass_fds=fds,
        ) as process,
    ):
        for line in process.stdout:
            sys.stdout.write(line)
            sink.write(line)
            lines.append(line)
    return process.returncode, "".join(lines)


def tool(context: Context, log: str, *command: str) -> tuple[int, str]:
    path = context.record / f"{log}.log"
    return stream(command, context.tree, path, context.tree)


def flagged(flag: str, values: list[str] | tuple[str, ...]) -> list[str]:
    return [part for value in values for part in (flag, value)]


def flashed_target(deployed: dict) -> str | None:
    return deployed.get("firmware", {}).get("target", {}).get("build_id")


def pi_identity(ssh: Callable[[str], str], deployed: dict) -> dict:
    facts = {name: ssh(command).strip() for name, command in PI_FACTS.items()}
    listing = ssh(f"sha256sum {' '.join(PI_FILES.values())}")
    sums = {
        path: digest
        for digest, path in (line.split() for line in listing.splitlines())
    }
    capture = deployed.get("firmware", {}).get("capture")
    if capture is None:
        raise QualifyError(
            "the Pi's deploy record names no capture board firmware: flash "
            "it with deploy.py flash-capture"
        )
    return {
        **facts,
        **{f"{name}_sha256": sums[path] for name, path in PI_FILES.items()},
        "capture_build_id": capture["build_id"],
    }


def deploy_drift(deployed: dict, target_id: str) -> dict[str, str]:
    flashed = flashed_target(deployed)
    if flashed == target_id:
        return {}
    return {
        DEPLOY: f"the deploy record names F401 build {flashed}, not REV's "
        f"{target_id}: the board was flashed since"
    }


def deploy_unit(context: Context, unit: Unit) -> Outcome:
    build = (*DEPLOY_TOOL, "build-target", "--tree", str(context.tree))
    if tool(context, unit.name, *build)[0]:
        failed = f"deploy.py build-target failed: {unit.name}.log"
        return Outcome(False, failures=(failed,))
    handoff = context.record / DEPLOY_HANDOFF
    handoff.unlink(missing_ok=True)
    code = hold(context.tree, [DEPLOY, context.rev])
    expired = bench(context.tree, "lock").HOLD_EXPIRED_EXIT
    return handed_back(handoff, code, expired)


def handed_back(handoff: Path, code: int, expired_exit: int) -> Outcome:
    if not handoff.exists():
        failed = f"the deploy hold exited {code} before taking the lock"
        return Outcome(False, failures=(f"{failed}: {DEPLOY}.log",))
    handed = json.loads(handoff.read_text())
    held_s = round(time.time() - handed["hold"]["acquired"], 1)
    taken = {"held_s": held_s, **handed["hold"], "exit": code}
    if "outcome" not in handed:
        cut = " at MAX_HOLD_S" if code == expired_exit else ""
        failed = f"the deploy hold exited {code}{cut} before it finished"
        failures = (f"{failed}: {DEPLOY}.log",)
        return Outcome(False, failures=failures, holds=(taken,))
    outcome = handed["outcome"]
    failures = tuple(outcome["failures"])
    return Outcome(outcome["passed"], outcome["facts"], failures, (taken,))


def deploy_hold(rev: str, since: float) -> int:
    acquired = time.time()
    record = QUALIFICATIONS / rev
    handoff = record / DEPLOY_HANDOFF
    waited_s = round(acquired - since, 1)
    taken = {"index": 1, "rows": [], "wait_s": waited_s, "acquired": acquired}
    write_json(handoff, {"hold": taken})
    builds = bench(RUN_TREE, "firmware_build_id")
    target_id = builds.target_build_id(RUN_TREE, RUN_TREE / F401_CONFIG)
    context = Context(rev, RUN_TREE, record, {}, target_id, {})
    restored, outcome = deployed_in_hold(context)
    taken |= {
        "held_s": round(time.time() - acquired, 1),
        "restored": restored,
    }
    verdict = {
        "passed": outcome.passed,
        "facts": outcome.facts,
        "failures": list(outcome.failures),
    }
    write_json(handoff, {"hold": taken, "outcome": verdict})
    return 0 if outcome.passed else 1


def deployed_in_hold(context: Context) -> tuple[list[str], Outcome]:
    remaining_s = bench(context.tree, "lock").remaining_s()
    if remaining_s < DEPLOY_HOLD_S:
        short = (
            f"the hold has {remaining_s:.0f} s left, the deploy plans up to "
            f"DEPLOY_HOLD_S = {DEPLOY_HOLD_S} s"
        )
        return [], Outcome(False, failures=(short,))
    code, output = tool(context, DEPLOY, *DEPLOY_TOOL, "settle")
    restored = [
        line.removeprefix(SETTLED_LINE)
        for line in output.splitlines()
        if line.startswith(SETTLED_LINE)
    ]
    if code:
        failed = f"deploy.py settle failed: {DEPLOY}.log"
        return restored, Outcome(False, failures=(failed,))
    tree = ("--tree", str(context.tree))
    if tool(context, DEPLOY, *DEPLOY_TOOL, "sync", *tree)[0]:
        failed = f"deploy.py sync failed: {DEPLOY}.log"
        return restored, Outcome(False, failures=(failed,))
    pi = bench(context.tree, "pi")
    before = pi.mcu_version()
    if tool(context, DEPLOY, *DEPLOY_TOOL, "ensure-target", *tree)[0]:
        failed = f"ensure-target failed: {DEPLOY}.log"
        return restored, Outcome(False, failures=(failed,))
    deployed = pi.read_deployed()
    return restored, firmware(context, deployed, before, pi.mcu_version())


def firmware(
    context: Context, deployed: dict, before: str, reported: str
) -> Outcome:
    builds = bench(context.tree, "firmware_build_id")
    target_id = context.target_id
    boards = deployed.get("firmware", {})
    facts = {
        "deployed_head": deployed["head"],
        "rust_build_id": deployed["rust_build_id"],
        "firmware": {
            "target_build_id": target_id,
            "mcu_version": reported,
            "reflashed": before != reported,
            "target": boards.get("target"),
            "capture": boards.get("capture"),
            "capture_tree_build_id": builds.capture_build_id(context.tree),
        },
    }
    failures = []
    if deployed["head"] != context.rev or deployed["dirty_paths"]:
        failures.append(
            f"the Pi runs {deployed['head']} (dirty: "
            f"{deployed['dirty_paths']}), not clean {context.rev}"
        )
    if not reported.endswith(target_id):
        failures.append(f"the F401 reports {reported!r}, not {target_id}")
    if flashed_target(deployed) != target_id:
        failures.append(
            f"the deploy record names F401 build {flashed_target(deployed)}, "
            f"not {target_id}: flash it with deploy.py flash-target so the "
            "record names what the board runs"
        )
    return Outcome(not failures, facts, tuple(failures))


ROW_KINDS = {
    (True, False): "passed",
    (False, False): "failed",
    (False, True): "xfail",
    (True, True): "xpass",
}


def tallies(rows: list[dict]) -> dict[str, int]:
    counts = dict.fromkeys(ROW_KINDS.values(), 0)
    for row in rows:
        counts[ROW_KINDS[(row["passed"], row["expected_fail"])]] += 1
    return {"rows": len(rows), **counts}


def row_facts(row: dict) -> dict:
    return {
        "name": row["name"],
        "passed": row["passed"],
        "expected_fail": row["expected_fail"],
        "print_state": row.get("print_state"),
        "duration_s": row.get("duration_s"),
        "wall_s": row.get("wall_s"),
        "analyzer_ok": row.get("analyzer_ok"),
        "failed_checks": row.get("failed_checks"),
        "checks": [*row.get("checks", []), *row.get("run_wide_checks", [])],
    }


def foreign_rows(rows: list[dict], context: Context) -> list[str]:
    foreign = []
    for row in rows:
        deployed = row.get("deployed")
        if deployed is None:
            continue
        target = flashed_target(deployed)
        if (
            deployed["head"] == context.rev
            and not deployed["dirty_paths"]
            and target == context.target_id
        ):
            continue
        foreign.append(
            f"{row['name']} ran on {deployed['head'][:12]} (dirty: "
            f"{deployed['dirty_paths']}) with F401 build {target}, not clean "
            f"{context.rev[:12]} with {context.target_id}"
        )
    return foreign


def judged_suite(
    summary_path: Path, code: int, detail: Detail, context: Context
) -> Outcome:
    summary = json.loads(summary_path.read_text())
    rows = summary["rows"]
    bad = [row for row in rows if row["passed"] == row["expected_fail"]]
    failures = [
        f"{row['name']}: {'XPASS' if row['passed'] else 'FAIL'} "
        f"{', '.join(row.get('failed_checks') or [])} {row.get('error') or ''}"
        for row in bad
    ]
    failures += foreign_rows(rows, context)
    if summary["aborted"]:
        failures.append(f"aborted: {summary['aborted']}")
    if not summary["step_benchmark"]["within_band"]:
        failures.append("the F401 step benchmark is out of band")
    facts = {
        "summary": str(summary_path),
        "tallies": tallies(rows),
        "failed_rows": [r["name"] for r in bad if not r["expected_fail"]],
    }
    if detail is Detail.ROWS:
        facts["rows"] = [row_facts(row) for row in rows]
    passed = code == 0 and not failures
    return Outcome(passed, facts, tuple(failures), tuple(summary["holds"]))


def suite_unit(context: Context, unit: Unit) -> Outcome:
    tolerated = [row for row in unit.rows if row in context.expected]
    command = (
        *UV_PYTHON,
        str(BENCH / "suite.py"),
        *flagged("--row", unit.rows),
        *flagged("--expect-fail", tolerated),
        "--tree",
        str(context.tree),
        "--lock-timeout",
        str(LOCK_TIMEOUT_S),
    )
    code, output = tool(context, unit.name, *command)
    match = SUMMARY_LINE.search(output)
    if match is None:
        failed = f"suite.py exited {code}: {unit.name}.log"
        return Outcome(False, failures=(failed,))
    return judged_suite(Path(match.group(1)), code, unit.detail, context)


def gate_failures(output: str) -> tuple[str, ...]:
    return tuple(
        line.removeprefix(FAIL_LINE)
        for line in output.splitlines()
        if line.startswith(FAIL_LINE)
    )


def pi_unit(context: Context, unit: Unit) -> Outcome:
    if tool(context, unit.name, "python3", "scripts/throughput_pi.py")[0]:
        failed = f"throughput_pi.py failed: {unit.name}.log"
        return Outcome(False, failures=(failed,))
    shutil.copy(context.tree / PI_WINDOW, context.record / PI_WINDOW.name)
    window = json.loads((context.tree / PI_WINDOW).read_text())
    code, output = tool(context, unit.name, *GATE, "pi-window")
    facts = {
        "pi_window": {
            key: window[key]
            for key in (
                "measured_at",
                "worst_x",
                "pinned_cores",
                "pinned_cores_worst_x",
            )
        },
        "timing": window["timing"],
    }
    holds = tuple(window["timing"]["holds"])
    return Outcome(code == 0, facts, gate_failures(output), holds)


def recheck_holds(tree: Path) -> tuple[dict, ...]:
    recheck = tree_script(tree, "throughput_recheck")
    return tuple(json.loads(recheck.data_path("pi").read_text())["holds"])


def calibration_unit(context: Context, unit: Unit) -> Outcome:
    code, output = tool(context, unit.name, *GATE, "calibration")
    if code == 0:
        stated = output.splitlines()[0]
        facts = {"calibration": {"refreshed": False, "stated": stated}}
        return Outcome(True, facts)
    recheck = ("python3", "scripts/throughput_recheck.py")
    if tool(context, unit.name, *recheck)[0]:
        failed = f"throughput_recheck.py failed: {unit.name}.log"
        return Outcome(False, failures=(*gate_failures(output), failed))
    holds = recheck_holds(context.tree)
    code, output = tool(context, unit.name, *GATE, "calibration")
    if code:
        return Outcome(False, failures=gate_failures(output), holds=holds)
    branch = commit_calibration(context)
    facts = {"calibration": {"refreshed": True, **branch}}
    return Outcome(True, facts, holds=holds)


def commit_calibration(context: Context) -> dict[str, str]:
    branch = f"qualify-calibration-{context.rev[:12]}"
    message = (
        f"Qualification of {context.rev[:12]} found the Pi calibration past "
        "its bound; throughput_recheck.py re-measured and refit it."
    )
    git("switch", "-q", "-c", branch, cwd=context.tree)
    git(
        "commit",
        "-q",
        "-m",
        "Refresh the Pi calibration",
        "-m",
        message,
        "--",
        str(BASELINE),
        cwd=context.tree,
    )
    body = context.record / "calibration-pr.md"
    body.write_text(f"{message}\n\n```throughput-accept\n{message}\n```\n")
    ship = (
        f"git worktree add .worktrees/{branch} {branch} && cd "
        f".worktrees/{branch} && python3 scripts/ship.py --title "
        f"'Refresh the Pi calibration' --body-file {body}"
    )
    print(f"qualify: calibration refreshed on branch {branch}; ship it: {ship}")
    return {"branch": branch, "pr_body": str(body), "ship": ship}


def units_of(
    suite: ModuleType, plan: Callable[[list[str]], list[list[str]]]
) -> list[Unit]:
    names = [row.name for row in suite.ROWS]
    holds = plan(names)
    planned = [name for rows in holds for name in rows]
    if sorted(planned) != sorted(names):
        raise QualifyError(
            f"hold_plan.plan({names}) returned {holds}, not every suite row "
            "exactly once"
        )
    return [
        Unit(DEPLOY, deploy_unit),
        *(
            Unit(f"suite-{index}", suite_unit, tuple(rows))
            for index, rows in enumerate(holds, 1)
        ),
        *(
            Unit(f"soak-{row.name}", suite_unit, (row.name,), Detail.ROWS)
            for row in suite.SOAK_ROWS
        ),
        Unit("pi", pi_unit),
        Unit("calibration", calibration_unit),
    ]


def checkout(tree: Path, sha: str) -> Path:
    git("worktree", "prune")
    if not (tree / ".git").exists():
        shutil.rmtree(tree, ignore_errors=True)
        tree.parent.mkdir(parents=True, exist_ok=True)
        git("worktree", "add", "--detach", "--force", str(tree), sha)
    git("checkout", "-q", "--detach", "--force", sha, cwd=tree)
    git("clean", "-fdq", cwd=tree)
    dirty = git("status", "--porcelain", cwd=tree)
    if dirty:
        raise QualifyError(f"{tree} is not clean at {sha[:12]}:\n{dirty}")
    return tree


def resolve(rev: str) -> str:
    return git("rev-parse", "--verify", f"{rev}^{{commit}}")


def build_engine(tree: Path) -> None:
    try:
        pending.run(list(ENGINE_BUILD), tree, target_env(tree))
    except pending.PendingBaselinesError as exc:
        raise QualifyError(str(exc)) from exc


def hold(tree: Path, argv: list[str]) -> int:
    held = [
        *UV_PYTHON,
        str(tree / BENCH / "lock.py"),
        "hold",
        "--lock-timeout",
        str(LOCK_TIMEOUT_S),
        "--",
        sys.executable,
        str(SCRIPT),
        *argv,
        "--held-since",
        str(time.time()),
    ]
    return subprocess.run(held, cwd=tree, check=False).returncode


def refuse_held(lock: ModuleType) -> None:
    if os.environ.get(lock.TOKEN_ENV):
        raise QualifyError(
            f"{lock.TOKEN_ENV} is set: qualify's tools take one hold per "
            "unit and refuse to run inside another; run it outside lock.py "
            "hold"
        )


def stamp(now: datetime.datetime) -> str:
    return now.strftime("%Y%m%d-%H%M%S")


def run(rev: str, expected: dict[str, int]) -> int:
    suite = bench(RUN_TREE, "suite")
    unknown = sorted(set(expected) - set(suite.ROW_BY_NAME))
    if unknown:
        raise QualifyError(f"--expect-fail names no suite row: {unknown}")
    units = units_of(suite, bench(RUN_TREE, "hold_plan").plan)
    pi = bench(RUN_TREE, "pi")
    deployed = pi.read_deployed()
    builds = bench(RUN_TREE, "firmware_build_id")
    target_id = builds.target_build_id(RUN_TREE, RUN_TREE / F401_CONFIG)
    record = QUALIFICATIONS / rev
    record.mkdir(parents=True, exist_ok=True)
    identity = pi_identity(pi.ssh, deployed)
    context = Context(rev, RUN_TREE, record, expected, target_id, identity)
    subject = git("log", "-1", "--format=%s", rev)
    return qualified(units, context, subject, deploy_drift(deployed, target_id))


def qualified(
    units: list[Unit], context: Context, subject: str, drift: dict[str, str]
) -> int:
    done = run_units(units, context, drift)
    summary = summarized(units, context, subject)
    write_json(context.record / RECORD_FILE, summary)
    if not done[-1]["passed"]:
        print("\n".join(failure_report(done, context.rev)))
        return 1
    blockers = unpublishable(units, context, utc_now())
    if blockers:
        print(f"qualify: no tag on {context.rev[:12]}: {'; '.join(blockers)}")
        return 1
    print(f"qualify: PASS, tagged {publish(context.record, summary)}")
    return 0


def failure_report(done: list[dict], rev: str) -> list[str]:
    failed = done[-1]
    lines = [f"qualify: FAIL at unit {failed['unit']} on {rev[:12]}; no tag"]
    lines += [f"  {failure}" for failure in failed["failures"]]
    rows = failed["facts"].get("failed_rows", [])
    lines += [
        f"  bisect: uv run python scripts/qualify.py bisect --row {row} "
        f"--bad {rev}"
        for row in rows
    ]
    if not rows:
        lines.append(f"  no row to bisect: {failed['unit']} judges the tree")
    lines.append(
        f"  resume: {RUN_COMMAND} {rev} reruns {failed['unit']} and skips "
        "the units that passed within the window"
    )
    return lines


def publish(record: Path, summary: dict) -> str:
    tagged = git(
        "tag", "--list", f"{TAG_PREFIX}*", "--points-at", summary["rev"]
    ).splitlines()
    name = tagged[0] if tagged else pushed_tag(summary)
    write_json(record / RECORD_FILE, {**summary, "tag": name})
    return name


def pushed_tag(summary: dict) -> str:
    local = git("tag", "--list", f"{TAG_PREFIX}*").splitlines()
    remote = git_net("ls-remote", "--tags", "origin", f"{TAG_PREFIX}*")
    remote_names = [
        line.split("refs/tags/")[-1] for line in remote.splitlines()
    ]
    name = next_tag({*local, *remote_names}, utc_now().date())
    create_tag(name, summary["rev"], {**summary, "tag": name})
    git_net("push", "origin", f"refs/tags/{name}")
    return name


def row_passes(runner: Path, candidate: Path, row: str, log: Path) -> bool:
    suite = (
        *UV_PYTHON,
        str(BENCH / "suite.py"),
        "--row",
        row,
        "--tree",
        str(candidate),
        "--lock-timeout",
        str(LOCK_TIMEOUT_S),
    )
    code, output = stream(suite, runner, log, candidate)
    match = SUMMARY_LINE.search(output)
    if match is None:
        raise QualifyError(f"suite.py exited {code} without a summary: {log}")
    [result] = json.loads(Path(match.group(1)).read_text())["rows"]
    return result["passed"]


def bisect(row: str, good: str, bad: str) -> int:
    candidates = git(
        "rev-list", "--first-parent", "--reverse", f"{good}..{bad}"
    ).splitlines()
    if not candidates:
        raise QualifyError(f"no first-parent commit between {good} and {bad}")
    log = QUALIFICATIONS / f"bisect-{stamp(utc_now())}.log"
    log.parent.mkdir(parents=True, exist_ok=True)

    def passes(sha: str) -> bool:
        candidate = checkout(CANDIDATE_TREE, sha)
        passed = row_passes(RUN_TREE, candidate, row, log)
        print(f"bisect: {row} {'passes' if passed else 'fails'} at {sha[:12]}")
        return passed

    culprit = first_bad(candidates, passes)
    subject = git("log", "-1", "--format=%s", culprit)
    pr = PR_MERGE.match(subject)
    number = f"PR #{pr.group(1)}" if pr else "no PR"
    print(f"bisect: first bad merge {culprit[:12]} ({number}): {subject}")
    return 0


def checked_out(rev: str) -> str:
    git_net("fetch", "-q", "--tags", "origin")
    sha = resolve(rev)
    checkout(RUN_TREE, sha)
    return sha


def last_qualified() -> str:
    tag = last_tag(MAIN)
    if tag is None:
        raise QualifyError(f"no {TAG_PREFIX}* tag on {MAIN}: pass --good REV")
    return tag


def run_command(args: argparse.Namespace) -> int:
    expected = expect_fail(args.expect_fail)
    with pending.commit_lock(LOCK_NAME):
        sha = checked_out(args.rev)
        refuse_held(bench(RUN_TREE, "lock"))
        build_engine(RUN_TREE)
        return run(sha, expected)


def bisect_command(args: argparse.Namespace) -> int:
    with pending.commit_lock(LOCK_NAME):
        bad = checked_out(args.bad)
        refuse_held(bench(RUN_TREE, "lock"))
        good = resolve(args.good or last_qualified())
        return bisect(args.row, good, bad)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("rev", nargs="?", default=MAIN)
    runner.add_argument("--expect-fail", action="append", default=[])
    shown = commands.add_parser("status")
    shown.add_argument("--line", action="store_true")
    search = commands.add_parser("bisect")
    search.add_argument("--row", required=True)
    search.add_argument("--good")
    search.add_argument("--bad", default=MAIN)
    held = commands.add_parser(
        DEPLOY, help="the deploy unit's hold; `run` starts it under lock.py"
    )
    held.add_argument("rev")
    held.add_argument("--held-since", type=float, required=True)
    args = parser.parse_args()
    if args.command == "run":
        return run_command(args)
    if args.command == "bisect":
        return bisect_command(args)
    if args.command == DEPLOY:
        return deploy_hold(args.rev, args.held_since)
    git_net("fetch", "-q", "--tags", "origin")
    print(status(View.LINE if args.line else View.TABLE))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except QualifyError as exc:
        sys.exit(f"qualify: {exc}")
