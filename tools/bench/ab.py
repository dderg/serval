"""A/B a gcode across git refs on the bench, one lock hold per run.

uv run python tools/bench/ab.py --ref origin/main --ref HEAD --gcode G
    [--config C] [--runs 3] [--host-cpus N --host-max-mhz M] [--dry-run]

Each ref is checked out clean (committed content only, no working-tree
edits) in a temporary worktree and its analyzer engine built, all before
the first hold. Each run is then one bench lock hold (lock.py hold, at most
lock.MAX_HOLD_S): deploy.py sync --tree of its ref, cheap when the Pi
already runs it and a restore when another holder changed it in between,
then one run.py, which settles the host and pushes the config with a
FIRMWARE_RESTART before it prints. run.py gets the hold's time left less
RECOVERY_TIMEOUT_S; a failed run is followed by deploy.py reset-target in
the same hold, and a run whose hold is killed at the cap is reported failed
while the next run goes on in a hold of its own. A ref's runs come one
after another so the Pi rebuilds once per ref. After the last hold every
run is analyzed by its ref's own analyzer and engine, so each ref's capture
is held against the plan its code commands, and the report gives every
hold's lock wait and held seconds. ab.py releases the lock between runs,
so it refuses to run inside a hold. The Pi is left on the last ref.
Worktrees are always removed.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path

import lock
import summarize
from suite import PRINTER_CFG, REPO, last_lines, python, read_json

AB_DIR = Path("~/bench/ab").expanduser()
HOLDS_DIR = "holds"
LOCK_TOOL = Path(__file__).with_name("lock.py")
ANALYZE_TIMEOUT_S = 1200.0
ENGINE_BUILD_TIMEOUT_S = 3600
RECOVERY_TIMEOUT_S = 600
SYNC_FACTS = ("synced:", "rust/ changed", "klipper restarted:")


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


def measure_refs(
    args: argparse.Namespace, trees: dict[str, Path], base: Path
) -> dict[str, list[dict]]:
    holds = {
        ref: [
            take_hold(args, ref, tree, base, index)
            for index in range(args.runs)
        ]
        for ref, tree in trees.items()
    }
    return {
        ref: [analyzed(hold, trees[ref]) for hold in ref_holds]
        for ref, ref_holds in holds.items()
    }


def take_hold(
    args: argparse.Namespace, ref: str, tree: Path, base: Path, index: int
) -> dict:
    print(f"{ref}: run {index + 1}/{args.runs}", flush=True)
    runs_dir = base / slug(ref)
    result = base / HOLDS_DIR / f"{slug(ref)}-{index + 1}.json"
    result.parent.mkdir(parents=True, exist_ok=True)
    before = run_dirs(runs_dir)
    requested = time.time()
    code = bench_hold(
        args.lock_timeout, held_command(args, ref, tree, runs_dir, result)
    )
    released = time.time()
    held = read_json(result)
    if "started" not in held:
        sys.exit(f"{ref} run {index + 1}: the hold exited {code} unstarted")
    error = held.get("error")
    if code == lock.HOLD_EXPIRED_EXIT:
        error = f"the hold ran past lock.MAX_HOLD_S = {lock.MAX_HOLD_S:.0f} s"
    elif code != 0:
        sys.exit(f"{ref} run {index + 1}: the hold failed (exit {code})")
    created = [path for path in run_dirs(runs_dir) if path not in before]
    return {
        "wait_s": round(held["started"] - requested, 1),
        "held_s": round(released - held["started"], 1),
        "exit": code,
        "sync": held.get("sync"),
        "error": error,
        "run": str(created[-1]) if created else None,
    }


def run_dirs(runs_dir: Path) -> list[Path]:
    return sorted(runs_dir.glob("*")) if runs_dir.exists() else []


def bench_hold(lock_timeout_s: float, command: list[str]) -> int:
    return subprocess.run(
        [
            sys.executable,
            str(LOCK_TOOL),
            "hold",
            "--lock-timeout",
            str(lock_timeout_s),
            "--",
            *command,
        ]
    ).returncode


def held_command(
    args: argparse.Namespace,
    ref: str,
    tree: Path,
    runs_dir: Path,
    result: Path,
) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "--held",
        "--ref",
        ref,
        "--gcode",
        str(args.gcode),
        "--config",
        str(args.config),
        "--tree",
        str(tree),
        "--runs-dir",
        str(runs_dir),
        "--result",
        str(result),
        *throttle_args(args),
    ]


def held_run(args: argparse.Namespace) -> None:
    started = time.time()
    write_json(args.result, {"started": started})
    sync = python(
        "deploy.py",
        "sync",
        "--tree",
        str(args.tree),
        timeout_s=lock.MAX_HOLD_S,
    )
    if sync.returncode != 0:
        sys.exit(f"sync of {args.ref[0]} failed: {last_lines(sync.stderr, 8)}")
    error = run_error(args, lock.remaining_s() - RECOVERY_TIMEOUT_S)
    if error is not None:
        python("deploy.py", "reset-target", timeout_s=RECOVERY_TIMEOUT_S)
    synced = "; ".join(
        line for line in sync.stdout.splitlines() if line.startswith(SYNC_FACTS)
    )
    write_json(
        args.result, {"started": started, "sync": synced, "error": error}
    )


def run_error(args: argparse.Namespace, budget_s: float) -> str | None:
    try:
        run = python(
            "run.py",
            str(args.gcode),
            "--config",
            str(args.config),
            "--runs-dir",
            str(args.runs_dir),
            *throttle_args(args),
            timeout_s=budget_s,
        )
    except subprocess.TimeoutExpired:
        return f"run.py: over the {budget_s:.0f} s its hold had left"
    if run.returncode != 0:
        return f"run.py: {last_lines(run.stderr)}"
    return None


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload) + "\n")


def analyzed(hold: dict, tree: Path) -> dict:
    if hold["run"] is None:
        return {
            "run": None,
            "error": hold["error"],
            "summary": None,
            "hold": hold,
        }
    run_dir = Path(hold["run"])
    errors = [hold["error"], analyze(run_dir, tree)]
    return {
        "run": str(run_dir),
        "error": " ".join(error for error in errors if error) or None,
        "summary": summarize.summarize(run_dir),
        "hold": hold,
    }


def analyze(run_dir: Path, tree: Path) -> str | None:
    if not (run_dir / "capture.bin").exists():
        return None
    analysis = subprocess.run(
        [sys.executable, str(tree / "tools/bench/analyze.py"), str(run_dir)],
        capture_output=True,
        text=True,
        timeout=ANALYZE_TIMEOUT_S,
    )
    if analysis.returncode != 0:
        return f"analyze.py: {last_lines(analysis.stderr)}"
    return None


def throttle_args(args: argparse.Namespace) -> list[str]:
    out = []
    if args.host_cpus is not None:
        out += ["--host-cpus", str(args.host_cpus)]
    if args.host_max_mhz is not None:
        out += ["--host-max-mhz", str(args.host_max_mhz)]
    return out


def build_analyzer_engine(tree: Path, ref: str) -> None:
    build = subprocess.run(
        ["make", "-f", "Makefile.rust", "config-doc", "motion-engine-fast"],
        cwd=tree,
        capture_output=True,
        text=True,
        timeout=ENGINE_BUILD_TIMEOUT_S,
    )
    if build.returncode != 0:
        sys.exit(
            f"building {ref}'s analyzer engine failed: {last_lines(build.stderr, 8)}"
        )


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
        "wait_s": sum(r["hold"]["wait_s"] for r in results),
        "held_s": sum(r["hold"]["held_s"] for r in results),
    }


def print_report(refs: list[str], by_ref: dict[str, list[dict]]) -> None:
    for ref in refs:
        print(f"\n== {ref}")
        for result in by_ref[ref]:
            hold = result["hold"]
            print(
                f"  hold: lock wait {hold['wait_s']:.1f} s, held "
                f"{hold['held_s']:.1f} s; {hold['sync'] or 'no sync'}"
            )
            if result["summary"]:
                print(summarize.format_line(result["summary"]))
            if result["error"]:
                print(f"  error: {result['error']}")
    print(
        "\nref                          runs ok err stalls worst abs_max us "
        "a/b/z/e  lock wait s  held s"
    )
    for ref in refs:
        agg = aggregate(by_ref[ref])
        worst = "/".join(
            "-" if v is None else f"{v:.0f}"
            for v in agg["worst_abs_max_us"].values()
        )
        print(
            f"{ref:<28} {agg['runs']:<4} {agg['ok']:<2} {agg['errors']:<3} "
            f"{agg['stalls']:<6} {worst:<24} {agg['wait_s']:<12.0f} "
            f"{agg['held_s']:.0f}"
        )


def print_plan(args: argparse.Namespace) -> None:
    print(f"gcode {args.gcode} config {args.config} runs/ref {args.runs}")
    print(f"throttle {throttle_args(args) or 'none'}")
    print("before the holds:")
    for ref in args.ref:
        print(f"  ref {ref} = {resolve(ref)[:12]}")
        print(f"    git worktree add --detach <{slug(ref)}> {ref}")
        print(
            "    make -f Makefile.rust config-doc motion-engine-fast  "
            f"(in <{slug(ref)}>)"
        )
    print(f"holds (one lock.py hold each, at most {lock.MAX_HOLD_S:.0f} s):")
    runs = [(ref, index) for ref in args.ref for index in range(args.runs)]
    for number, (ref, index) in enumerate(runs, 1):
        print(
            f"  {number}. {ref} run {index + 1}/{args.runs}: deploy.py sync "
            f"--tree <{slug(ref)}>; run.py (deploy.py reset-target if it fails)"
        )
    print("after the holds:")
    print("  <tree>/tools/bench/analyze.py per run, then the report")
    print("  git worktree remove <tree>")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", action="append", required=True)
    parser.add_argument("--gcode", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=PRINTER_CFG)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--host-cpus", type=int)
    parser.add_argument("--host-max-mhz", type=int)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--held", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--tree", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--runs-dir", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--result", type=Path, help=argparse.SUPPRESS)
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    if args.held:
        held_run(args)
        return
    if args.dry_run:
        print_plan(args)
        return
    if lock.TOKEN_ENV in os.environ:
        sys.exit(
            "ab.py takes one bench lock hold per run and releases it in "
            "between: run it outside lock.py hold"
        )
    base = AB_DIR / datetime.now().strftime("%Y%m%d-%H%M%S")
    with contextlib.ExitStack() as stack:
        trees = {ref: stack.enter_context(worktree(ref)) for ref in args.ref}
        for ref, tree in trees.items():
            build_analyzer_engine(tree, ref)
        by_ref = measure_refs(args, trees, base)
    print_report(args.ref, by_ref)
    print(f"\nruns under {base}")


if __name__ == "__main__":
    main()
