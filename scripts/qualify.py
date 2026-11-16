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
its engine, takes the bench lock once for everything after, and runs STAGES
in order, stopping at the first that fails:
- deploy: deploy.py sync and ensure-target; the F401 must report REV's
  firmware build id (flashed through flash-target when it did not);
- suite: the full suite.py; a row may fail only through --expect-fail
  ROW=ISSUE, recorded with its issue;
- soak: suite.py --group soak;
- pi: throughput_pi.py on REV's tree, judged by
  `throughput_gate.py --qualify pi-window`; its timing (sync, build on the
  Pi, every replay) goes into the record;
- calibration: `throughput_gate.py --qualify calibration`; a stale one is
  re-measured by throughput_recheck.py, judged again and committed on a
  local branch for a normal PR (qualification never pushes commits).
A pass pushes an annotated tag on REV whose message is the JSON summary
(parse_message reads it back); the record lives in QUALIFICATIONS/<tag>/.
A failure tags nothing and prints the failed stage, its rows and the bisect
command per failed row.

`status` prints the last qualified tag and the qualification debt: the
first-parent merges on origin/main since it, and how many touch each of
DEBT_AREAS. `status --line` is the one line land-pr.sh prints after a
landing.

`bisect` searches the first-parent merges after the last qualified tag (or
--good) up to --bad (default origin/main) for the first on which one suite
row fails, in one lock hold: the row as --bad defines it runs from RUN_TREE
through `suite.py --row NAME --tree CANDIDATE_TREE`, the candidate checked
out clean, its engine built and synced the way ab.py deploys a ref.
"""

from __future__ import annotations

import argparse
import datetime
import enum
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
RUN_COMMAND = "uv run python scripts/qualify.py run"
BENCH = Path("tools") / "bench"
F401_CONFIG = BENCH / "f401.config"
PI_WINDOW = BENCH / "pi_window.json"
BASELINE = BENCH / "throughput_baseline.json"
UV_PYTHON = ("uv", "run", "python")
ENGINE_BUILD = ("make", "-f", "Makefile.rust", "motion-engine-fast")
GATE = ("python3", "scripts/throughput_gate.py", "--qualify")
SOAK_SCOPE = ("--group", "soak")
SUMMARY_LINE = re.compile(r"^summary: (\S+summary\.json)$", re.M)
FAIL_LINE = "FAIL: "
PR_MERGE = re.compile(r"^Merge pull request #(\d+)")
EXPECT_FAIL = re.compile(r"^(?P<row>\w+)=#?(?P<issue>\d+)$")
RECORD_FILE = "qualification.json"
FIELD_SEP, RECORD_SEP = "\x1f", "\x1e"


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


@dataclass(frozen=True)
class Context:
    rev: str
    tree: Path
    record: Path
    expected: dict[str, int]
    soak_rows: frozenset[str]


@dataclass(frozen=True)
class Stage:
    name: str
    run: Callable[[Context], Outcome]


def run_stages(stages: list[Stage], context: Context) -> list[dict]:
    done = []
    for stage in stages:
        print(f"qualify: stage {stage.name}", flush=True)
        started = time.monotonic()
        try:
            outcome = stage.run(context)
        except QualifyError as exc:
            outcome = Outcome(False, failures=(str(exc),))
        done.append(
            {
                "stage": stage.name,
                "seconds": round(time.monotonic() - started),
                "passed": outcome.passed,
                "failures": list(outcome.failures),
                **outcome.facts,
            }
        )
        if not outcome.passed:
            break
    return done


def bench(tree: Path, name: str) -> ModuleType:
    path = str(tree / BENCH)
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


def deploy_stage(context: Context) -> Outcome:
    pi = bench(context.tree, "pi")
    deploy = (*UV_PYTHON, str(BENCH / "deploy.py"))
    if tool(context, "deploy", *deploy, "sync")[0]:
        return Outcome(False, failures=("deploy.py sync failed: deploy.log",))
    before = pi.mcu_version()
    if tool(context, "deploy", *deploy, "ensure-target")[0]:
        return Outcome(False, failures=("ensure-target failed: deploy.log",))
    return firmware(context, pi.read_deployed(), before, pi.mcu_version())


def firmware(
    context: Context, deployed: dict, before: str, reported: str
) -> Outcome:
    builds = bench(context.tree, "firmware_build_id")
    target_id = builds.target_build_id(context.tree, context.tree / F401_CONFIG)
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


def judged_suite(summary_path: Path, code: int, detail: Detail) -> Outcome:
    summary = json.loads(summary_path.read_text())
    rows = summary["rows"]
    bad = [row for row in rows if row["passed"] == row["expected_fail"]]
    failures = [
        f"{row['name']}: {'XPASS' if row['passed'] else 'FAIL'} "
        f"{', '.join(row.get('failed_checks') or [])} {row.get('error') or ''}"
        for row in bad
    ]
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
    return Outcome(code == 0 and not failures, facts, tuple(failures))


def suite_run(
    context: Context,
    log: str,
    scope: tuple[str, ...],
    rows: list[str],
    detail: Detail,
) -> Outcome:
    flags = [flag for row in rows for flag in ("--expect-fail", row)]
    suite = (*UV_PYTHON, str(BENCH / "suite.py"), *scope, *flags)
    code, output = tool(context, log, *suite)
    match = SUMMARY_LINE.search(output)
    if match is None:
        return Outcome(False, failures=(f"suite.py exited {code}: {log}.log",))
    return judged_suite(Path(match.group(1)), code, detail)


def suite_stage(context: Context) -> Outcome:
    rows = [r for r in context.expected if r not in context.soak_rows]
    return suite_run(context, "suite", (), rows, Detail.TALLIES)


def soak_stage(context: Context) -> Outcome:
    rows = [r for r in context.expected if r in context.soak_rows]
    return suite_run(context, "soak", SOAK_SCOPE, rows, Detail.ROWS)


def gate_failures(output: str) -> tuple[str, ...]:
    return tuple(
        line.removeprefix(FAIL_LINE)
        for line in output.splitlines()
        if line.startswith(FAIL_LINE)
    )


def pi_stage(context: Context) -> Outcome:
    if tool(context, "pi", "python3", "scripts/throughput_pi.py")[0]:
        return Outcome(False, failures=("throughput_pi.py failed: pi.log",))
    shutil.copy(context.tree / PI_WINDOW, context.record / PI_WINDOW.name)
    window = json.loads((context.tree / PI_WINDOW).read_text())
    code, output = tool(context, "pi", *GATE, "pi-window")
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
    return Outcome(code == 0, facts, gate_failures(output))


def calibration_stage(context: Context) -> Outcome:
    code, output = tool(context, "calibration", *GATE, "calibration")
    if code == 0:
        stated = output.splitlines()[0]
        facts = {"calibration": {"refreshed": False, "stated": stated}}
        return Outcome(True, facts)
    recheck = ("python3", "scripts/throughput_recheck.py")
    if tool(context, "calibration", *recheck)[0]:
        failed = "throughput_recheck.py failed: calibration.log"
        return Outcome(False, failures=(*gate_failures(output), failed))
    code, output = tool(context, "calibration", *GATE, "calibration")
    if code:
        return Outcome(False, failures=gate_failures(output))
    branch = commit_calibration(context)
    return Outcome(True, {"calibration": {"refreshed": True, **branch}})


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


STAGES = [
    Stage("deploy", deploy_stage),
    Stage("suite", suite_stage),
    Stage("soak", soak_stage),
    Stage("pi", pi_stage),
    Stage("calibration", calibration_stage),
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


def utc_now() -> datetime.datetime:
    return datetime.datetime.now(datetime.timezone.utc)


def stamp(now: datetime.datetime) -> str:
    return now.strftime("%Y%m%d-%H%M%S")


def run(rev: str, expected: dict[str, int], since: float) -> int:
    started = utc_now()
    lock_wait_s = round(started.timestamp() - since)
    if git("rev-parse", "HEAD", cwd=RUN_TREE) != rev:
        raise QualifyError(f"{RUN_TREE} moved off {rev} before the lock")
    suite = bench(RUN_TREE, "suite")
    unknown = sorted(set(expected) - set(suite.ROW_BY_NAME))
    if unknown:
        raise QualifyError(f"--expect-fail names no suite row: {unknown}")
    record = QUALIFICATIONS / f"run-{stamp(started)}"
    record.mkdir(parents=True)
    soak = frozenset(row.name for row in suite.SOAK_ROWS)
    context = Context(rev, RUN_TREE, record, expected, soak)
    stages = run_stages(STAGES, context)
    summary = {
        "rev": rev,
        "subject": git("log", "-1", "--format=%s", rev),
        "started": started.isoformat(timespec="seconds"),
        "finished": utc_now().isoformat(timespec="seconds"),
        "seconds": round((utc_now() - started).total_seconds()),
        "lock_wait_s": lock_wait_s,
        "expect_fail": [{"row": r, "issue": n} for r, n in expected.items()],
        "stages": stages,
    }
    (record / RECORD_FILE).write_text(json.dumps(summary, indent=2) + "\n")
    passed = len(stages) == len(STAGES) and stages[-1]["passed"]
    if not passed:
        print("\n".join(failure_report(stages, rev)))
        return 1
    print(f"qualify: PASS, tagged {publish(record, summary)}")
    return 0


def failure_report(stages: list[dict], rev: str) -> list[str]:
    failed = stages[-1]
    lines = [f"qualify: FAIL at stage {failed['stage']} on {rev[:12]}; no tag"]
    lines += [f"  {failure}" for failure in failed["failures"]]
    rows = failed.get("failed_rows", [])
    lines += [
        f"  bisect: uv run python scripts/qualify.py bisect --row {row} "
        f"--bad {rev}"
        for row in rows
    ]
    if not rows:
        lines.append(f"  no row to bisect: {failed['stage']} judges the tree")
    return lines


def publish(record: Path, summary: dict) -> str:
    local = git("tag", "--list", f"{TAG_PREFIX}*").splitlines()
    remote = git_net("ls-remote", "--tags", "origin", f"{TAG_PREFIX}*")
    remote_names = [
        line.split("refs/tags/")[-1] for line in remote.splitlines()
    ]
    name = next_tag({*local, *remote_names}, utc_now().date())
    final = QUALIFICATIONS / name
    summary = {**summary, "tag": name, "record": str(final)}
    create_tag(name, summary["rev"], summary)
    git_net("push", "origin", f"refs/tags/{name}")
    record.rename(final)
    (final / RECORD_FILE).write_text(json.dumps(summary, indent=2) + "\n")
    return name


def row_passes(runner: Path, candidate: Path, row: str, log: Path) -> bool:
    suite = (
        *UV_PYTHON,
        str(BENCH / "suite.py"),
        "--row",
        row,
        "--tree",
        str(candidate),
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
    if args.held_since is not None:
        return run(args.rev, expected, args.held_since)
    with pending.commit_lock(LOCK_NAME):
        sha = checked_out(args.rev)
        build_engine(RUN_TREE)
        flags = [f"--expect-fail={row}={n}" for row, n in expected.items()]
        return hold(RUN_TREE, ["run", sha, *flags])


def bisect_command(args: argparse.Namespace) -> int:
    if args.held_since is not None:
        return bisect(args.row, args.good, args.bad)
    with pending.commit_lock(LOCK_NAME):
        bad = checked_out(args.bad)
        good = resolve(args.good or last_qualified())
        argv = ["bisect", "--row", args.row, "--good", good, "--bad", bad]
        return hold(RUN_TREE, argv)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("rev", nargs="?", default=MAIN)
    runner.add_argument("--expect-fail", action="append", default=[])
    runner.add_argument("--held-since", type=float, help=argparse.SUPPRESS)
    shown = commands.add_parser("status")
    shown.add_argument("--line", action="store_true")
    search = commands.add_parser("bisect")
    search.add_argument("--row", required=True)
    search.add_argument("--good")
    search.add_argument("--bad", default=MAIN)
    search.add_argument("--held-since", type=float, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.command == "run":
        return run_command(args)
    if args.command == "bisect":
        return bisect_command(args)
    git_net("fetch", "-q", "--tags", "origin")
    print(status(View.LINE if args.line else View.TABLE))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except QualifyError as exc:
        sys.exit(f"qualify: {exc}")
