"""A/B a gcode across git refs on the bench, in one lock hold.

uv run python tools/bench/ab.py --ref origin/main --ref HEAD --gcode G
    [--config C] [--runs 3] [--host-cpus N --host-max-mhz M] [--dry-run]

Each ref is checked out clean (committed content only, no working-tree
edits) in a temporary worktree, synced to the Pi with deploy.py sync --tree,
run N times and analyzed. The Pi is left on the last ref. Worktrees are
always removed.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path

import lock
import summarize
from suite import PRINTER_CFG, REPO, last_lines, python

AB_DIR = Path("~/bench/ab").expanduser()
SYNC_TIMEOUT_S = 1800
RUN_TIMEOUT_S = 2400.0
ANALYZE_TIMEOUT_S = 1200.0
RECOVERY_TIMEOUT_S = 600


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, check=True, capture_output=True, text=True
    ).stdout.strip()


def slug(ref: str) -> str:
    return ref.replace("/", "-")


def resolve(ref: str) -> str:
    try:
        return git("rev-parse", "--verify", f"{ref}^{{commit}}")
    except subprocess.CalledProcessError as exc:
        sys.exit(f"unknown ref {ref!r}: {exc.stderr.strip()}")


@contextlib.contextmanager
def worktree(ref: str) -> Iterator[Path]:
    path = Path(tempfile.mkdtemp(prefix=f"bench-ab-{slug(ref)}-"))
    try:
        git("worktree", "add", "--detach", str(path), ref)
        yield path
    finally:
        subprocess.run(
            ["git", "worktree", "remove", "--force", str(path)],
            cwd=REPO,
            check=False,
            capture_output=True,
        )
        shutil.rmtree(path, ignore_errors=True)


def run_once(args: argparse.Namespace, runs_dir: Path) -> dict:
    run = python(
        "run.py",
        str(args.gcode),
        "--config",
        str(args.config),
        "--runs-dir",
        str(runs_dir),
        *throttle_args(args),
        timeout_s=RUN_TIMEOUT_S,
    )
    error = None
    if run.returncode != 0:
        error = f"run.py: {last_lines(run.stderr)}"
        python("deploy.py", "reset-target", timeout_s=RECOVERY_TIMEOUT_S)
    run_dirs = sorted(runs_dir.glob("*")) if runs_dir.exists() else []
    if not run_dirs:
        return {"run": None, "error": error, "summary": None}
    run_dir = run_dirs[-1]
    if (run_dir / "capture.bin").exists():
        analysis = python(
            "analyze.py", str(run_dir), timeout_s=ANALYZE_TIMEOUT_S
        )
        if analysis.returncode != 0:
            error = f"{error or ''} analyze.py: {last_lines(analysis.stderr)}"
    return {
        "run": str(run_dir),
        "error": error,
        "summary": summarize.summarize(run_dir),
    }


def throttle_args(args: argparse.Namespace) -> list[str]:
    out = []
    if args.host_cpus is not None:
        out += ["--host-cpus", str(args.host_cpus)]
    if args.host_max_mhz is not None:
        out += ["--host-max-mhz", str(args.host_max_mhz)]
    return out


def ab_ref(ref: str, args: argparse.Namespace, base: Path) -> list[dict]:
    results = []
    with worktree(ref) as tree:
        sync = python(
            "deploy.py", "sync", "--tree", str(tree), timeout_s=SYNC_TIMEOUT_S
        )
        if sync.returncode != 0:
            sys.exit(f"sync of {ref} failed: {last_lines(sync.stderr, 8)}")
        for index in range(args.runs):
            print(f"{ref}: run {index + 1}/{args.runs}", flush=True)
            results.append(run_once(args, base / slug(ref)))
    return results


def aggregate(results: list[dict]) -> dict:
    rows = [
        r["summary"]
        for r in results
        if r["summary"] and r["summary"]["analyzed"]
    ]
    worst = {}
    for role in summarize.MOTORS:
        values = [
            row["timing_us"][role]["abs_max"]
            for row in rows
            if row["timing_us"][role]
        ]
        worst[role] = max(values) if values else None
    return {
        "runs": len(results),
        "ok": sum(1 for r in rows if r["ok"]),
        "errors": sum(1 for r in results if r["error"]),
        "stalls": sum(r["stalls"] for r in rows),
        "worst_abs_max_us": worst,
    }


def print_report(refs: list[str], by_ref: dict[str, list[dict]]) -> None:
    for ref in refs:
        print(f"\n== {ref}")
        for result in by_ref[ref]:
            if result["summary"]:
                print(summarize.format_line(result["summary"]))
            if result["error"]:
                print(f"  error: {result['error']}")
    print(
        "\nref                          runs ok err stalls worst abs_max us a/b/z/e"
    )
    for ref in refs:
        agg = aggregate(by_ref[ref])
        worst = "/".join(
            "-" if v is None else f"{v:.0f}"
            for v in agg["worst_abs_max_us"].values()
        )
        print(
            f"{ref:<28} {agg['runs']:<4} {agg['ok']:<2} {agg['errors']:<3} "
            f"{agg['stalls']:<6} {worst}"
        )


def print_plan(args: argparse.Namespace) -> None:
    print(f"gcode {args.gcode} config {args.config} runs/ref {args.runs}")
    print(f"throttle {throttle_args(args) or 'none'}")
    for ref in args.ref:
        print(f"ref {ref} = {resolve(ref)[:12]}")
        print("  git worktree add --detach <tmp> <ref>")
        print("  deploy.py sync --tree <tmp>")
        print(f"  run.py + analyze.py x{args.runs}")
        print("  git worktree remove <tmp>")
    print("all under one bench lock hold")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", action="append", required=True)
    parser.add_argument("--gcode", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=PRINTER_CFG)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--host-cpus", type=int)
    parser.add_argument("--host-max-mhz", type=int)
    parser.add_argument("--dry-run", action="store_true")
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    if args.dry_run:
        print_plan(args)
        return
    base = AB_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
    with lock.bench_lock(args.lock_timeout, command=" ".join(sys.argv)):
        os.environ[lock.TOKEN_ENV] = str(os.getpid())
        by_ref = {ref: ab_ref(ref, args, base) for ref in args.ref}
    print_report(args.ref, by_ref)
    print(f"\nruns under {base}")


if __name__ == "__main__":
    main()
