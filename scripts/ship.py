#!/usr/bin/env python3
"""Ship the branch checked out here: sync, format, gate, snapshot-gate, land.

    python3 scripts/ship.py --title T (--body B | --body-file F)
        [--expect SPEC --accept-msg FILE] [--no-gates] [--dry-run]

One command for the steps every landing repeated by hand (measured over 64
landings: fetch, merge origin/main, cargo fmt, ruff format, ci.sh quick, a
baseline review, accept, land-pr.sh), in order, stopping at the first problem:

1. refuse a dirty tree (commit your work first);
2. merge origin/main when the branch is behind, naming the conflicted files;
3. refuse a branch that moves tools/bench/throughput_baseline.json without a
   ```throughput-accept block in the body (the CI throughput gate's rule,
   see scripts/throughput_gate.py);
4. cargo fmt --all and ruff format; formatting changes are committed;
5. ./scripts/ci.sh quick, plus ./scripts/ci.sh py when klippy/, test/ or any
   Python under tools/ or scripts/ changed (skip with --no-gates);
6. land_queue.py --branch: build the snapshot engine, review the snapshots
   against origin/main (no --expect: nothing may move; with it: every case must
   be EXPECTED, then accepted and committed from --accept-msg), land-pr.sh.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import throughput_gate

PY_GATE_PREFIXES = ("klippy/", "test/")
PY_GATE_DIRS = ("tools/", "scripts/")
THROUGHPUT_BASELINE = str(
    throughput_gate.BASELINE.relative_to(throughput_gate.ROOT)
)


class ShipError(RuntimeError):
    pass


def throughput_accept_problem(changed: list[str], body: str) -> str | None:
    if THROUGHPUT_BASELINE not in changed or throughput_gate.accept_reason(
        body
    ):
        return None
    return (
        f"the branch moves {THROUGHPUT_BASELINE}: say why in a "
        "```throughput-accept block in the PR body"
    )


def needs_py_gate(changed: list[str]) -> bool:
    return any(
        name.startswith(PY_GATE_PREFIXES)
        or (name.startswith(PY_GATE_DIRS) and name.endswith(".py"))
        for name in changed
    )


def gate_commands(changed: list[str], skip: bool) -> list[list[str]]:
    if skip:
        return []
    commands = [["./scripts/ci.sh", "quick"]]
    if needs_py_gate(changed):
        commands.append(["./scripts/ci.sh", "py"])
    return commands


def body_text(body: str | None, body_file: Path | None) -> str:
    if (body is None) == (body_file is None):
        raise ShipError("give exactly one of --body and --body-file")
    return body if body is not None else body_file.read_text()


def git(*args: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", *args], capture_output=True, text=True, check=False
    )
    if result.returncode != 0 and args[:1] in (("fetch",), ("push",)):
        result = run_git_with_retry(args)
    if check and result.returncode != 0:
        raise ShipError(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout.strip()


def step(message: str) -> None:
    print(f"ship: {message}", flush=True)


def run_git_with_retry(args: tuple[str, ...]) -> subprocess.CompletedProcess:
    helper = Path(__file__).with_name("gh_util.py")
    return subprocess.run(
        [sys.executable, str(helper), "--git", *args],
        capture_output=True,
        text=True,
        check=False,
    )


def run(command: list[str], cwd: Path | None = None) -> None:
    step("$ " + " ".join(command))
    if subprocess.run(command, cwd=cwd).returncode != 0:
        raise ShipError(f"failed: {' '.join(command)}")


def sync_main(branch: str) -> None:
    git("fetch", "-q", "origin")
    behind = subprocess.run(
        ["git", "merge-base", "--is-ancestor", "origin/main", "HEAD"]
    ).returncode
    if behind == 0:
        return
    step("merging origin/main")
    merged = subprocess.run(
        [
            "git",
            "merge",
            "--no-edit",
            "-q",
            "-m",
            f"Merge main into {branch}",
            "origin/main",
        ],
        capture_output=True,
        text=True,
    )
    if merged.returncode != 0:
        files = git("diff", "--name-only", "--diff-filter=U", check=False)
        raise ShipError(
            "merging origin/main conflicts; resolve, commit, rerun. Conflicted:\n"
            + files
        )


def format_tree(root: Path) -> None:
    run(["cargo", "fmt", "--all"], root / "rust")
    run(["uv", "run", "ruff", "format", "."], root)
    if git("status", "--porcelain", "--untracked-files=no"):
        git("add", "-u")
        git("commit", "-q", "-m", "Apply cargo fmt and ruff format")
        step("committed formatting changes")


def free_space_gc() -> None:
    subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("gc_caches.py")),
            "--if-free-below",
            "100",
            "--apply",
        ],
        check=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--title", required=True)
    parser.add_argument("--body")
    parser.add_argument("--body-file", type=Path)
    parser.add_argument("--expect", default="")
    parser.add_argument("--accept-msg", type=Path)
    parser.add_argument("--no-gates", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    body = body_text(args.body, args.body_file)
    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    if branch in ("HEAD", "main"):
        raise ShipError(f"check a topic branch out first (on {branch})")
    root = Path(git("rev-parse", "--show-toplevel"))
    if git("status", "--porcelain", "--untracked-files=no"):
        raise ShipError("the tree has uncommitted changes: commit them first")
    free_space_gc()
    sync_main(branch)
    changed = git("diff", "--name-only", "origin/main...HEAD").splitlines()
    problem = throughput_accept_problem(changed, body)
    if problem:
        raise ShipError(problem)
    commands = gate_commands(changed, args.no_gates)
    if args.dry_run:
        step(f"would format, run {commands or 'no gates'}, then land {branch}")
        return 0
    format_tree(root)
    for command in commands:
        run(command, root)
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as handle:
        handle.write(body)
    land = [
        sys.executable,
        str(root / "scripts" / "land_queue.py"),
        "--branch",
        branch,
        "--title",
        args.title,
        "--body-file",
        handle.name,
        "--expect",
        args.expect,
    ]
    if args.accept_msg:
        land += ["--accept-msg", str(args.accept_msg)]
    run(land, root)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ShipError as exc:
        sys.exit(f"ship: {exc}")
