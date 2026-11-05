#!/usr/bin/env python3
"""List, and with --apply delete, stale per-task build caches.

    uv run python scripts/gc_caches.py [--days 3] [--apply]

Covers ~/.cache/*-target (ad-hoc cargo target dirs left by finished tasks),
the pending_baselines.py cache (worktrees, stacks, logs, reviews) and
abandoned /tmp/bench-ab-* worktree directories. Anything not touched for
--days days is stale, unless a git worktree of any repo lives in it or under
it, or a running process has its cwd or an open file there. The reused caches
~/.cache/pending-baselines/{samples,audit} are always kept; the per-worktree
cargo target dirs in pending-baselines/targets age out like the rest.
Also:
- docker images tagged klipper-build-* or kalico-sim-* (3.6 GB / 0.65 GB per
  worktree slot) older than --image-hours (default 12) that no container uses
  are removed, and docker build cache older than a week is pruned;
- pending-baselines/targets keeps at most MAX_SCRATCH_TARGETS directories, the
  newest ones, whatever their age (a scratch worktree per commit used to leave
  one 0.5-1 GB target each: 495 dirs, 277 GB);
- --worktree-targets removes rust/target of worktrees that are idle (detached
  and clean) or whose branch is merged into origin/main, when no process uses
  them; the worktree source is never touched;
- --if-free-below GB does nothing while the home filesystem has at least GB
  free, and turns --worktree-targets on when it runs.
Dry run unless --apply.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

CACHE = Path("~/.cache").expanduser()
PENDING = CACHE / "pending-baselines"
KEPT = (PENDING / "samples", PENDING / "audit")
SECONDS_PER_DAY = 86400.0
MAX_SCRATCH_TARGETS = 24
DOCKER_IMAGE_PREFIXES = ("klipper-build-", "kalico-sim-")
DOCKER_BUILD_CACHE_HOURS = 168
SECONDS_PER_HOUR = 3600.0
GIB = 2**30


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
    for name in (
        "wt",
        "stack",
        "stack-logs",
        "review",
        "targets",
        "fuzz-triage",
    ):
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


def free_gib(path: Path = CACHE) -> float:
    return shutil.disk_usage(path).free / GIB


def over_scratch_cap(targets: list[Path]) -> list[Path]:
    ordered = sorted(targets, key=newest_mtime, reverse=True)
    return ordered[MAX_SCRATCH_TARGETS:]


def docker_lines(*args: str) -> list[str]:
    result = subprocess.run(
        ["docker", *args], capture_output=True, text=True, check=False
    )
    return result.stdout.splitlines() if result.returncode == 0 else []


def stale_docker_images(hours: float) -> list[str]:
    in_use = set(docker_lines("ps", "-a", "--format", "{{.Image}}"))
    stale = []
    for line in docker_lines(
        "images", "--format", "{{.Repository}}:{{.Tag}}\t{{.CreatedAt}}"
    ):
        name, _, created = line.partition("\t")
        if not name.startswith(DOCKER_IMAGE_PREFIXES) or name in in_use:
            continue
        stamp = " ".join(created.split()[:2])
        try:
            age = time.time() - time.mktime(
                time.strptime(stamp, "%Y-%m-%d %H:%M:%S")
            )
        except ValueError:
            continue
        if age > hours * SECONDS_PER_HOUR:
            stale.append(name)
    return stale


def worktree_entries() -> list[dict]:
    listing = subprocess.run(
        ["git", "worktree", "list", "--porcelain"],
        cwd=Path("~/Developer/serval").expanduser(),
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    entries, current = [], {}
    for line in listing.splitlines() + [""]:
        if not line:
            if current:
                entries.append(current)
            current = {}
        else:
            key, _, value = line.partition(" ")
            current[key] = value
    return entries


def is_clean(path: Path) -> bool:
    status = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=path,
        capture_output=True,
        text=True,
    )
    return status.returncode == 0 and not status.stdout.strip()


def branch_merged(path: Path, branch: str) -> bool:
    return (
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", branch, "origin/main"],
            cwd=path,
        ).returncode
        == 0
    )


def reclaimable_worktree_targets() -> list[Path]:
    found = []
    for entry in worktree_entries()[1:]:
        tree = Path(entry["worktree"])
        target = tree / "rust" / "target"
        if not target.is_dir() or not is_clean(tree):
            continue
        branch = entry.get("branch", "").removeprefix("refs/heads/")
        if "detached" in entry or (branch and branch_merged(tree, branch)):
            found.append(target)
    return found


def remove(path: Path) -> None:
    shutil.rmtree(path) if path.is_dir() else path.unlink()


def process_holder(path: Path, busy: list[Path]) -> str | None:
    return holder(path, [], busy)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--days", type=float, default=3.0)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--image-hours", type=float, default=12.0)
    parser.add_argument("--worktree-targets", action="store_true")
    parser.add_argument("--if-free-below", type=float, metavar="GB")
    parser.add_argument(
        "--keep",
        action="append",
        default=[],
        help="path (or ~/.cache name) never removed; repeatable",
    )
    args = parser.parse_args()
    if args.if_free_below is not None:
        if free_gib() >= args.if_free_below:
            return
        print(
            f"gc_caches: {free_gib():.0f} GiB free, below {args.if_free_below:.0f}: cleaning",
            file=sys.stderr,
        )
        args.worktree_targets = True
    cutoff = time.time() - args.days * SECONDS_PER_DAY
    total = 0
    worktrees, busy = worktree_paths(), process_paths()
    kept = [Path(k).expanduser() if "/" in k else CACHE / k for k in args.keep]
    scratch = (
        sorted((PENDING / "targets").glob("*"))
        if (PENDING / "targets").is_dir()
        else []
    )
    capped = set(over_scratch_cap(scratch))
    victims = []
    for path in candidates():
        if path in kept:
            print(f"   kept  {path}  (--keep)")
        elif newest_mtime(path) >= cutoff and path not in capped:
            continue
        else:
            victims.append((path, holder(path, worktrees, busy)))
    if args.worktree_targets:
        victims += [
            (target, process_holder(target, busy))
            for target in reclaimable_worktree_targets()
        ]
    for path, reason in victims:
        if reason:
            print(f"   kept  {path}  ({reason})")
            continue
        size = size_bytes(path)
        total += size
        age = (time.time() - newest_mtime(path)) / SECONDS_PER_DAY
        print(f"{size / GIB:7.2f} GiB  {age:5.1f} d  {path}")
        if args.apply:
            remove(path)
    for image in stale_docker_images(args.image_hours):
        print(f"   docker image  {image}")
        if args.apply:
            subprocess.run(["docker", "rmi", image], capture_output=True)
    if args.apply:
        subprocess.run(
            [
                "docker",
                "builder",
                "prune",
                "-f",
                "--filter",
                f"until={DOCKER_BUILD_CACHE_HOURS}h",
            ],
            capture_output=True,
        )
    subprocess.run(["git", "worktree", "prune"], check=False)
    verb = "removed" if args.apply else "would remove (use --apply)"
    print(f"{verb}: {total / GIB:.2f} GiB (+ docker images and build cache)")


if __name__ == "__main__":
    main()
