#!/usr/bin/env python3
"""Land a queue of branches one after another, gating each on its snapshots.

    python3 scripts/land_queue.py QUEUE [--trees DIR] [--dry-run]

QUEUE has one `branch|title|expect` per line (`#` comments and blank lines
ignored). `expect` is a review_baselines.py expectation, empty for a branch
that must not move any snapshot. Beside the queue file, per branch:

- `<branch>.md`          the PR body (required);
- `<branch>.accept.txt`  the baseline commit message (required with expect).

For each branch, in order, in its own worktree under --trees:

1. merge origin/main when the branch is behind it;
2. build the snapshot engine and run scripts/review_baselines.py against
   origin/main. Without expect, any changed, new or removed case stops the
   queue. With expect, every changed case must be EXPECTED (a SUSPICIOUS or
   REGRESSION verdict stops the queue); the changed and new cases are accepted
   and committed with the message file;
3. scripts/land-pr.sh. A red run of the "Sim e2e" workflow is rerun once
   (loudly, see the open flake issues); any other red check stops the queue.

A branch that lands is appended to `QUEUE.landed` and skipped on the next
run, so a stopped queue resumes where it stopped.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import accept_reviewed
import pending_baselines as pending

ROOT = pending.ROOT
DEFAULT_TREES = Path("~/.cache/land-queue").expanduser()
FLAKY_WORKFLOW = "Sim e2e"
MAX_FLAKE_RERUNS = 1
RERUN_SETTLE_SECONDS = 30
REEXEC_VAR = "LAND_QUEUE_REEXEC"


class QueueError(RuntimeError):
    pass


@dataclass(frozen=True)
class Entry:
    branch: str
    title: str
    expect: str


def parse_queue(text: str) -> list[Entry]:
    entries = []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split("|")
        if len(fields) not in (2, 3):
            raise QueueError(
                f"line {number}: want branch|title|expect: {line!r}"
            )
        branch, title = fields[0].strip(), fields[1].strip()
        expect = fields[2].strip() if len(fields) == 3 else ""
        if not branch or not title:
            raise QueueError(f"line {number}: branch and title are required")
        entries.append(Entry(branch, title, expect))
    branches = [entry.branch for entry in entries]
    duplicated = sorted({b for b in branches if branches.count(b) > 1})
    if duplicated:
        raise QueueError(f"branches queued twice: {', '.join(duplicated)}")
    return entries


def snapshot_gate(review: dict, expect: str) -> tuple[list[str], list[str]]:
    """Cases to accept and the reasons the queue must stop."""
    moved = len(review["cases"]) + len(review["summary"]["removed"])
    if not expect:
        new = review["summary"]["new"]
        if moved or new:
            return [], [
                f"{moved} snapshot case(s) changed and {len(new)} are new "
                "but the queue entry has no expectation"
            ]
        return [], []
    cases, errors = accept_reviewed.decide(review, {})
    if review["summary"]["removed"]:
        errors.append(f"cases removed: {review['summary']['removed']}")
    return cases, errors


def failed_flaky_run(runs: list[dict]) -> int | None:
    failed = [
        run
        for run in runs
        if run["conclusion"] == "failure" and run["name"] == FLAKY_WORKFLOW
    ]
    others = [
        run
        for run in runs
        if run["conclusion"] == "failure" and run["name"] != FLAKY_WORKFLOW
    ]
    if others or not failed:
        return None
    return failed[0]["databaseId"]


def read_landed(path: Path) -> set[str]:
    return set(path.read_text().split()) if path.exists() else set()


def git(tree: Path, *args: str) -> str:
    return pending.run(["git", *args], tree).strip()


def ensure_tree(entry: Entry, trees: Path) -> Path:
    listing = git(ROOT, "worktree", "list", "--porcelain")
    current = None
    for line in listing.splitlines():
        if line.startswith("worktree "):
            current = Path(line.split(" ", 1)[1])
        elif line == f"branch refs/heads/{entry.branch}" and current:
            return current
    tree = trees / entry.branch
    trees.mkdir(parents=True, exist_ok=True)
    git(ROOT, "worktree", "add", "-q", str(tree), entry.branch)
    return tree


def merge_main(tree: Path, entry: Entry) -> None:
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", "origin/main", "HEAD"], cwd=tree
    )
    if ancestor.returncode != 0:
        git(
            tree,
            "merge",
            "--no-edit",
            "-q",
            "-m",
            f"Merge main into {entry.branch}",
            "origin/main",
        )


def review_snapshots(tree: Path, entry: Entry) -> dict:
    pending.run(
        ["make", "-f", "Makefile.rust", "motion-engine-fast"],
        tree,
        {"CARGO_TARGET_DIR": str(pending.target_dir_for(tree))},
    )
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "review.json"
        command = [
            "uv",
            "run",
            "python",
            "scripts/review_baselines.py",
            "--base",
            "origin/main",
            "--head",
            "HEAD",
            "--no-audit",
            "--json",
            str(out),
        ]
        if entry.expect:
            command += ["--expect", entry.expect]
        result = subprocess.run(command, cwd=tree, text=True)
        if not out.exists():
            raise QueueError(
                f"{entry.branch}: review_baselines.py failed ({result.returncode})"
            )
        return json.loads(out.read_text())


STATUS_LINE = re.compile(r"^\s*(EXACT|CHANGED|PENDING)\s+(\S+)\s*$")


def not_yet_accepted(cases: list[str], check_output: str) -> list[str]:
    exact = {
        match.group(2)
        for line in check_output.splitlines()
        if (match := STATUS_LINE.match(line)) and match.group(1) == "EXACT"
    }
    return [case for case in cases if case not in exact]


def accept(tree: Path, cases: list[str], message: Path) -> None:
    check = subprocess.run(
        ["uv", "run", "python", "snapshots/run.py"],
        cwd=tree,
        capture_output=True,
        text=True,
    )
    cases = not_yet_accepted(cases, check.stdout)
    if not cases:
        return
    pending.run(
        ["uv", "run", "python", "snapshots/run.py", "--accept", *cases], tree
    )
    git(tree, "add", "snapshots/baselines")
    git(tree, "commit", "-q", "-F", str(message))


def land(tree: Path, entry: Entry, body: str) -> None:
    reruns = 0
    while True:
        result = subprocess.run(
            [
                "scripts/land-pr.sh",
                entry.branch,
                "--title",
                entry.title,
                "--body",
                body,
            ],
            cwd=tree,
        )
        if result.returncode == 0:
            return
        runs = json.loads(
            pending.run(
                [
                    "gh",
                    "run",
                    "list",
                    "--branch",
                    entry.branch,
                    "--limit",
                    "12",
                    "--json",
                    "databaseId,name,conclusion,headSha",
                ],
                tree,
            )
        )
        head = git(tree, "rev-parse", "HEAD")
        run_id = failed_flaky_run([r for r in runs if r["headSha"] == head])
        if run_id is None or reruns >= MAX_FLAKE_RERUNS:
            raise QueueError(f"{entry.branch}: failed to land")
        reruns += 1
        print(
            f"land_queue: rerunning the red {FLAKY_WORKFLOW} run {run_id} once"
        )
        pending.run(["gh", "run", "rerun", str(run_id), "--failed"], tree)
        subprocess.run(["sleep", str(RERUN_SETTLE_SECONDS)])


def process(entry: Entry, queue: Path, trees: Path) -> None:
    process_files(
        entry,
        queue.with_name(f"{entry.branch}.md"),
        queue.with_name(f"{entry.branch}.accept.txt"),
        trees,
    )


def process_files(
    entry: Entry, body_file: Path, accept_file: Path, trees: Path
) -> None:
    if not body_file.exists():
        raise QueueError(f"{entry.branch}: missing PR body {body_file}")
    if entry.expect and not accept_file.exists():
        raise QueueError(
            f"{entry.branch}: missing commit message {accept_file}"
        )
    pending.run(["git", "fetch", "-q", "origin"], ROOT)
    tree = ensure_tree(entry, trees)
    merge_main(tree, entry)
    review = review_snapshots(tree, entry)
    cases, errors = snapshot_gate(review, entry.expect)
    if errors:
        raise QueueError(f"{entry.branch}: " + "; ".join(errors))
    accept(tree, cases, accept_file)
    land(tree, entry, body_file.read_text())


def reexec_current_copy() -> None:
    if os.environ.get(REEXEC_VAR):
        return
    pending.run(["git", "fetch", "-q", "origin"], ROOT)
    fresh = Path(tempfile.mkdtemp(prefix="land-queue-"))
    archive = subprocess.run(
        ["git", "archive", "origin/main", "scripts"],
        cwd=ROOT,
        capture_output=True,
        check=True,
    ).stdout
    subprocess.run(["tar", "-x", "-C", str(fresh)], input=archive, check=True)
    current = fresh / "scripts" / Path(__file__).name
    if current.read_bytes() == Path(__file__).read_bytes():
        shutil.rmtree(fresh)
        return
    print(
        "land_queue: this copy differs from origin/main's; running origin/main's",
        file=sys.stderr,
        flush=True,
    )
    os.execve(
        sys.executable,
        [sys.executable, str(current), *sys.argv[1:]],
        {**os.environ, REEXEC_VAR: "1", "SERVAL_REPO_ROOT": str(ROOT)},
    )


def main() -> int:
    reexec_current_copy()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("queue", type=Path, nargs="?")
    parser.add_argument("--trees", type=Path, default=DEFAULT_TREES)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--branch", help="land one branch without a queue file")
    parser.add_argument("--title")
    parser.add_argument("--body-file", type=Path)
    parser.add_argument("--expect", default="")
    parser.add_argument("--accept-msg", type=Path)
    args = parser.parse_args()
    if args.branch:
        if not (args.title and args.body_file):
            raise QueueError("--branch needs --title and --body-file")
        entry = Entry(args.branch, args.title, args.expect)
        process_files(
            entry,
            args.body_file,
            args.accept_msg or Path("/nonexistent"),
            args.trees,
        )
        print(f"landed {entry.branch}", flush=True)
        return 0
    if args.queue is None:
        raise QueueError(
            "give a queue file, or --branch with --title and --body-file"
        )
    entries = parse_queue(args.queue.read_text())
    landed_file = args.queue.with_name(args.queue.name + ".landed")
    landed = read_landed(landed_file)
    todo = [entry for entry in entries if entry.branch not in landed]
    if args.dry_run:
        for entry in todo:
            print(
                f"{entry.branch}: {entry.title} [{entry.expect or 'no snapshot change'}]"
            )
        return 0
    for entry in todo:
        print(f"=== {entry.branch}", flush=True)
        process(entry, args.queue, args.trees)
        with landed_file.open("a") as handle:
            handle.write(entry.branch + "\n")
        print(f"landed {entry.branch}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (QueueError, pending.PendingBaselinesError) as exc:
        sys.exit(f"land_queue: {exc}")
