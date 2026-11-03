#!/usr/bin/env python3
"""List, and with --apply delete, stale per-task build caches.

    uv run python scripts/gc_caches.py [--days 3] [--apply]

Covers ~/.cache/*-target (ad-hoc cargo target dirs left by finished tasks),
the pending_baselines.py cache (worktrees, stacks, logs, per-commit samples)
and abandoned /tmp/bench-ab-* worktree directories. Anything not touched for
--days days is stale. The shared ~/.cache/pending-baselines/target dir is
kept: it is the one target dir the tools reuse. Dry run unless --apply.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import time
from pathlib import Path

CACHE = Path("~/.cache").expanduser()
PENDING = CACHE / "pending-baselines"
SHARED_TARGET = PENDING / "target"
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
    for name in ("wt", "stack", "stack-logs", "samples", "review"):
        directory = PENDING / name
        if directory.is_dir():
            found.extend(sorted(directory.iterdir()))
    found.extend(sorted(Path("/tmp").glob("bench-ab-*")))
    return [p for p in found if p != SHARED_TARGET]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--days", type=float, default=3.0)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    cutoff = time.time() - args.days * SECONDS_PER_DAY
    total = 0
    for path in candidates():
        if newest_mtime(path) >= cutoff:
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
