#!/usr/bin/env python3
"""Merge an ordered list of PR heads onto origin/main in a throwaway worktree.

    uv run python scripts/integrate_stack.py 443 444 445 [--run CMD ...] [--keep]
    uv run python scripts/integrate_stack.py --order-from-pending --run "./scripts/ci.sh quick"

Each head is merged with a merge commit, locally; nothing is pushed. The first
conflict stops the run with the conflicting files and the pair that collided.
Each --run command executes in the integrated worktree with the cargo target
dir shared with pending_baselines.py. The worktree is removed afterwards unless
--keep. Exit status is non-zero on a conflict or a failing command.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pending_baselines as pending

ROOT = pending.ROOT
WORKTREES = pending.CACHE / "stack"
LOGS = pending.CACHE / "stack-logs"
TALLY_LINES = 2
SEPARATOR_CHARS = set("─-=━ ")


class IntegrateStackError(RuntimeError):
    pass


@dataclass
class Step:
    label: str
    branch: str
    result: str = "pending"
    detail: str = ""
    conflicts: list[str] = field(default_factory=list)


@dataclass
class CommandResult:
    command: str
    code: int
    seconds: float
    tally: str
    log: Path


def git(
    *args: str, cwd: Path = ROOT, check: bool = True
) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["git", *args], cwd=cwd, capture_output=True, text=True
    )
    if check and result.returncode != 0:
        raise IntegrateStackError(
            f"git {' '.join(args)} failed ({result.returncode}):\n"
            f"{result.stdout[-1500:]}{result.stderr[-1500:]}"
        )
    return result


def resolve(item: str) -> Step:
    if item.lstrip("#").isdigit():
        number = int(item.lstrip("#"))
        branch = pending.run(
            [
                "gh",
                "pr",
                "view",
                str(number),
                "--json",
                "headRefName",
                "--jq",
                ".headRefName",
            ]
        ).strip()
        return Step(f"#{number}", branch)
    return Step(item, item)


def order_from_pending() -> list[Step]:
    prs = pending.load_prs([])
    analyzed = []
    for pr in prs:
        try:
            pending.analyze(pr)
            analyzed.append(pr)
        except pending.PendingBaselinesError as exc:
            print(
                f"skipping #{pr.number}: {exc.args[0].splitlines()[-1]}",
                file=sys.stderr,
            )
    return [
        Step(f"#{pr.number}", pr.branch) for pr in pending.merge_order(analyzed)
    ]


def merge_step(tree: Path, previous: list[Step], step: Step) -> bool:
    git("fetch", "origin", step.branch)
    result = git(
        "merge",
        "--no-ff",
        "--no-edit",
        f"origin/{step.branch}",
        cwd=tree,
        check=False,
    )
    if result.returncode == 0:
        step.result = "merged"
        step.detail = git(
            "rev-parse", "--short", "HEAD", cwd=tree
        ).stdout.strip()
        return True
    conflicts = git(
        "diff", "--name-only", "--diff-filter=U", cwd=tree
    ).stdout.split()
    git("merge", "--abort", cwd=tree, check=False)
    step.result = "CONFLICT"
    step.conflicts = conflicts
    with_ = previous[-1].label if previous else "origin/main"
    step.detail = f"with {with_} (everything merged before it)"
    if not conflicts:
        step.detail += f": {result.stderr.strip() or result.stdout.strip()}"
    return False


def tally(output: str) -> str:
    lines = [
        line.strip()
        for line in output.splitlines()
        if line.strip() and not set(line.strip()) <= SEPARATOR_CHARS
    ]
    return " | ".join(lines[-TALLY_LINES:])[:200]


def run_command(tree: Path, command: str, index: int) -> CommandResult:
    import time

    LOGS.mkdir(parents=True, exist_ok=True)
    log = LOGS / f"{tree.name}-{index}.log"
    started = time.monotonic()
    env = {
        **os.environ,
        "CARGO_TARGET_DIR": str(pending.TARGET_DIR),
        "CI_KEEP_CARGO_TARGET_DIR": "1",
        "PATH": f"{Path.home() / '.cargo/bin'}:{os.environ['PATH']}",
    }
    result = subprocess.run(
        command,
        shell=True,
        cwd=tree,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    output = result.stdout
    log.write_text(output)
    return CommandResult(
        command,
        result.returncode,
        time.monotonic() - started,
        tally(output),
        log,
    )


def print_report(
    steps: list[Step], commands: list[CommandResult], tree: Path, kept: bool
) -> None:
    print("\nmerge steps")
    for number, step in enumerate(steps, 1):
        line = f"  {number:>2}. {step.label:<6} {step.branch[:34]:<36} {step.result:<9} {step.detail}"
        print(line)
        for path in step.conflicts:
            print(f"        conflict: {path}")
    if commands:
        print("\ncommands (in the integrated tree)")
        for result in commands:
            status = "ok" if result.code == 0 else f"FAIL({result.code})"
            print(f"  {status:<9} {result.seconds:6.0f}s  {result.command}")
            print(f"            {result.tally}")
            print(f"            log: {result.log}")
    if kept:
        print(f"\nworktree kept: {tree}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "items", nargs="*", help="PR numbers or branch names, in merge order"
    )
    parser.add_argument("--order-from-pending", action="store_true")
    parser.add_argument("--run", action="append", default=[], metavar="CMD")
    parser.add_argument("--keep", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if bool(args.items) == args.order_from_pending:
        parser.error(
            "give PRs/branches or --order-from-pending, not both or neither"
        )
    return args


def main() -> int:
    args = parse_args()
    pending.run(["git", "fetch", "origin", "-q"])
    steps = (
        order_from_pending()
        if args.order_from_pending
        else [resolve(i) for i in args.items]
    )
    if args.dry_run:
        print(
            "merge order: " + " ".join(f"{s.label}({s.branch})" for s in steps)
        )
        for command in args.run:
            print(f"+ run: {command}")
        return 0
    tree = WORKTREES / f"{os.getpid()}"
    tree.parent.mkdir(parents=True, exist_ok=True)
    git("worktree", "add", "--detach", str(tree), "origin/main")
    merged: list[Step] = []
    commands: list[CommandResult] = []
    failed = False
    try:
        for step in steps:
            if not merge_step(tree, merged, step):
                failed = True
                break
            merged.append(step)
        if not failed:
            for index, command in enumerate(args.run):
                result = run_command(tree, command, index)
                commands.append(result)
                failed = failed or result.code != 0
    finally:
        print_report(steps, commands, tree, args.keep)
        if not args.keep:
            git("worktree", "remove", "--force", str(tree), check=False)
            shutil.rmtree(tree, ignore_errors=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
