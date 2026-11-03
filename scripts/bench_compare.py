#!/usr/bin/env python3
"""Interleaved replay-throughput comparison of two commits.

    uv run python scripts/bench_compare.py <commitA> <commitB>
        [--configs fast,printer] [--gcode PATH] [--runs 5] [--set-env K=V ...]

Builds `gcode_replay_bench` at each commit in one reusable worktree (shared
cargo target dir, binaries cached by commit), then runs the replay --runs
times per config, alternating which commit goes first so drift and thermal
state hit both alike, and prints median wall seconds, realtime factor and the
per-stage breakdown for A and B with the change B vs A. Run it on an otherwise
idle machine; the load average is printed so a noisy result is recognisable.
"""

from __future__ import annotations

import argparse
import os
import re
import statistics
import subprocess
import sys
from pathlib import Path

import pending_baselines as pending

ROOT = pending.ROOT
BENCH_DIR = pending.CACHE / "bench"
WORKTREE = BENCH_DIR / "worktree"
BINARIES = BENCH_DIR / "bin"
CONFIGS = {
    "fast": "tools/bench/configs/fast.cfg",
    "printer": "tools/bench/printer.cfg",
}
DEFAULT_GCODE = "tools/sim/gcode/voron_dense_top_layers.gcode"
SUMMARY = re.compile(r"wall_secs=(?P<wall>[\d.]+) overall_x=(?P<x>[\d.]+)")
STAGE = re.compile(r"^\s+(?P<name>\S+)\s+(?P<secs>[\d.]+)s\s+[\d.]+%\s+calls=")


class BenchCompareError(RuntimeError):
    pass


def parse_run(output: str) -> dict:
    summary = SUMMARY.search(output)
    if not summary:
        raise BenchCompareError(
            f"no summary line in the replay output:\n{output[-800:]}"
        )
    stages = {
        match["name"]: float(match["secs"])
        for line in output.splitlines()
        if (match := STAGE.match(line))
    }
    return {
        "wall_s": float(summary["wall"]),
        "x": float(summary["x"]),
        "stages": stages,
    }


def median_run(runs: list[dict]) -> dict:
    names = sorted({name for run in runs for name in run["stages"]})
    return {
        "wall_s": statistics.median(r["wall_s"] for r in runs),
        "x": statistics.median(r["x"] for r in runs),
        "stages": {
            name: statistics.median(r["stages"].get(name, 0.0) for r in runs)
            for name in names
        },
    }


def percent(b: float, a: float) -> str:
    return f"{(b / a - 1) * 100:+.1f}%" if a > 0 else "n/a"


def comparison_table(
    config: str, a: dict, b: dict, spread: tuple[float, float]
) -> str:
    lines = [
        f"\n{config}: median of {{runs}}; wall spread A {spread[0]:.1f}%, B {spread[1]:.1f}%",
        f"  {'':<30}{'A':>10}{'B':>10}{'B vs A':>10}",
        f"  {'wall s':<30}{a['wall_s']:>10.2f}{b['wall_s']:>10.2f}{percent(b['wall_s'], a['wall_s']):>10}",
        f"  {'realtime x':<30}{a['x']:>10.2f}{b['x']:>10.2f}{percent(b['x'], a['x']):>10}",
    ]
    names = sorted(
        set(a["stages"]) | set(b["stages"]),
        key=lambda n: -max(a["stages"].get(n, 0), b["stages"].get(n, 0)),
    )
    for name in names:
        sa, sb = a["stages"].get(name, 0.0), b["stages"].get(name, 0.0)
        lines.append(f"  {name:<30}{sa:>10.3f}{sb:>10.3f}{percent(sb, sa):>10}")
    return "\n".join(lines)


def spread_percent(runs: list[dict]) -> float:
    walls = [r["wall_s"] for r in runs]
    return (max(walls) - min(walls)) / statistics.median(walls) * 100


def checkout_and_build(sha: str) -> Path:
    binary = BINARIES / sha / "gcode_replay_bench"
    if binary.exists():
        return binary
    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    if not WORKTREE.exists():
        pending.run(["git", "worktree", "prune"])
        pending.run(["git", "worktree", "add", "--detach", str(WORKTREE), sha])
    pending.ensure_commit(sha)
    pending.run(["git", "checkout", "-q", "--detach", sha], WORKTREE)
    print(
        f"building gcode_replay_bench at {sha[:10]} ...",
        file=sys.stderr,
        flush=True,
    )
    env = {
        "CARGO_TARGET_DIR": str(pending.TARGET_DIR),
        "PATH": f"{Path.home() / '.cargo/bin'}:{os.environ['PATH']}",
    }
    pending.run(
        [
            "cargo",
            "build",
            "--release",
            "-p",
            "motion-core",
            "--example",
            "gcode_replay_bench",
        ],
        WORKTREE / "rust",
        env,
    )
    built = pending.TARGET_DIR / "release" / "examples" / "gcode_replay_bench"
    binary.parent.mkdir(parents=True, exist_ok=True)
    binary.write_bytes(built.read_bytes())
    binary.chmod(0o755)
    return binary


def replay(binary: Path, gcode: Path, config: Path, env: dict) -> dict:
    result = subprocess.run(
        [str(binary), str(gcode), "--config", str(config)],
        capture_output=True,
        text=True,
        env={**os.environ, **env},
        cwd=WORKTREE,
    )
    if result.returncode != 0:
        raise BenchCompareError(
            f"{binary.parent.name} failed ({result.returncode}): {result.stderr[-800:]}"
        )
    return parse_run(result.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("commit_a")
    parser.add_argument("commit_b")
    parser.add_argument("--configs", default="fast,printer")
    parser.add_argument("--gcode", default=DEFAULT_GCODE)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--set-env", action="append", default=[], metavar="K=V")
    args = parser.parse_args()
    unknown = [c for c in args.configs.split(",") if c not in CONFIGS]
    if unknown:
        parser.error(
            f"unknown config(s) {unknown}; choose from {sorted(CONFIGS)}"
        )
    env = dict(item.split("=", 1) for item in args.set_env)
    pending.run(["git", "fetch", "origin", "-q"])
    shas = [
        pending.run(
            ["git", "rev-parse", "--verify", f"{rev}^{{commit}}"]
        ).strip()
        for rev in (args.commit_a, args.commit_b)
    ]
    binaries = [checkout_and_build(sha) for sha in shas]
    gcode = ROOT / args.gcode
    load = os.getloadavg()[0]
    print(
        f"load average {load:.1f} on {os.cpu_count()} cores; gcode {args.gcode}"
    )
    for name in args.configs.split(","):
        config = ROOT / CONFIGS[name]
        runs: list[list[dict]] = [[], []]
        for index in range(args.runs):
            order = (0, 1) if index % 2 == 0 else (1, 0)
            for side in order:
                runs[side].append(replay(binaries[side], gcode, config, env))
            print(
                f"  {name} run {index + 1}/{args.runs} done",
                file=sys.stderr,
                flush=True,
            )
        a, b = median_run(runs[0]), median_run(runs[1])
        table = comparison_table(
            name, a, b, (spread_percent(runs[0]), spread_percent(runs[1]))
        )
        print(table.replace("{runs}", str(args.runs)))
    print(
        f"\nA = {shas[0][:10]} ({args.commit_a}), B = {shas[1][:10]} ({args.commit_b})"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (BenchCompareError, pending.PendingBaselinesError) as exc:
        sys.exit(f"bench_compare: {exc}")
