#!/usr/bin/env python3
"""List, and with --apply delete, stale per-task build caches.

    uv run python scripts/gc_caches.py [--days 3] [--apply]

Covers ~/.cache/*-target (ad-hoc cargo target dirs left by finished tasks),
the pending_baselines.py cache (worktrees, stacks, logs, reviews) and
abandoned /tmp/bench-ab-* worktree directories. Anything not touched for
--days days is stale, unless a git worktree of any repo lives in it or under
it, or a running process has its cwd or an open file there. The reused caches
~/.cache/pending-baselines/{target,samples,audit} are always kept.
Dry run unless --apply.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import time
from pathlib import Path

CACHE = Path("~/.cache").expanduser()
PENDING = CACHE / "pending-baselines"
KEPT = (
    PENDING / "target",
    PENDING / "samples",
    PENDING / "audit",
    CACHE / "serval-target",
)
SECONDS_PER_DAY = 86400.0


def newest_mtime(path: Path) -> float:
    newest = path.stat().st_mtime
    if path.is_dir():
        for child in path.iterdir():
            newest = max(newest, child.stat().st_mtime)
    return newest


def size_bytes(path: Path) -> int:
    result = subprocess.run(
        ["du", "-sk", str(path)], capture_output=True, text=True, check=True
    )
    return int(result.stdout.split()[0]) * 1024


def candidates() -> list[Path]:
    found = sorted(CACHE.glob("*-target"))
    for name in ("wt", "stack", "stack-logs", "review"):
        directory = PENDING / name
        if directory.is_dir():
            found.extend(sorted(directory.iterdir()))
    found.extend(sorted(Path("/tmp").glob("bench-ab-*")))
    return [p for p in found if p not in KEPT]


def worktree_paths() -> list[Path]:
    paths = []
    repos = [Path("~/Developer/serval").expanduser()]
    for repo in repos:
        listing = subprocess.run(
            ["git", "worktree", "list", "--porcelain"],
            cwd=repo,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        paths += [
            Path(line.split(" ", 1)[1])
            for line in listing.splitlines()
            if line.startswith("worktree ")
        ]
    return paths


def process_paths() -> list[Path]:
    found = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            found.append(Path(os.readlink(proc / "cwd")))
            for fd in (proc / "fd").iterdir():
                found.append(Path(os.readlink(fd)))
        except OSError:
            continue
    return found


def holder(path: Path, worktrees: list[Path], busy: list[Path]) -> str | None:
    for other in worktrees:
        if other == path or path in other.parents or other in path.parents:
            return f"git worktree {other}"
    for other in busy:
        if other == path or path in other.parents:
            return f"open by a process: {other}"
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--days", type=float, default=3.0)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--keep",
        action="append",
        default=[],
        help="path (or ~/.cache name) never removed; repeatable",
    )
    args = parser.parse_args()
    cutoff = time.time() - args.days * SECONDS_PER_DAY
    total = 0
    worktrees, busy = worktree_paths(), process_paths()
    kept = [Path(k).expanduser() if "/" in k else CACHE / k for k in args.keep]
    for path in candidates():
        if path in kept:
            print(f"   kept  {path}  (--keep)")
            continue
        if newest_mtime(path) >= cutoff:
            continue
        reason = holder(path, worktrees, busy)
        if reason:
            print(f"   kept  {path}  ({reason})")
            continue
        size = size_bytes(path)
        total += size
        age = (time.time() - newest_mtime(path)) / SECONDS_PER_DAY
        print(f"{size / 2**30:7.2f} GiB  {age:5.1f} d  {path}")
        if args.apply:
            shutil.rmtree(path) if path.is_dir() else path.unlink()
    subprocess.run(["git", "worktree", "prune"], check=False)
    verb = "removed" if args.apply else "would remove (use --apply)"
    print(f"{verb}: {total / 2**30:.2f} GiB")


if __name__ == "__main__":
    main()
