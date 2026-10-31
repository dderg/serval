#!/usr/bin/env python3
"""Bump mtimes of files whose CONTENT changed since the last run.

BuildKit's local context scan trusts mtimes; on macOS file sharing it has
been observed to serve stale file content for files whose mtimes it thought
it knew. Touching every source file forces a re-read but also destroys the
host's cargo/make incrementality (every sim run used to trigger a full
workspace recompile). Hashing the tree and touching only content-changed
files keeps both caches honest: BuildKit re-reads exactly the files that
changed, and cargo rebuilds only what it must.

Usage: touch_changed.py <repo_root> <manifest_path> <path>...
"""

import hashlib
import json
import os
import sys

PRUNE_DIRS = {"target", "target-linux", "__pycache__", "third_party_repos"}


def iter_files(root: str, paths: list) -> list:
    out = []
    for path in paths:
        top = os.path.join(root, path)
        if os.path.isfile(top):
            out.append(top)
            continue
        for dirpath, dirnames, filenames in os.walk(top):
            dirnames[:] = [d for d in dirnames if d not in PRUNE_DIRS]
            for name in filenames:
                out.append(os.path.join(dirpath, name))
    return out


def file_hash(path: str) -> str:
    h = hashlib.blake2b(digest_size=16)
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    root, manifest_path = sys.argv[1], sys.argv[2]
    paths = sys.argv[3:]
    try:
        with open(manifest_path) as f:
            old = json.load(f)
    except (OSError, ValueError):
        old = None

    new = {}
    changed = 0
    for path in iter_files(root, paths):
        try:
            digest = file_hash(path)
        except OSError:
            continue
        rel = os.path.relpath(path, root)
        new[rel] = digest
        if old is not None and old.get(rel) != digest:
            os.utime(path)
            changed += 1
    if old is None:
        for rel in new:
            os.utime(os.path.join(root, rel))
        changed = len(new)

    os.makedirs(os.path.dirname(manifest_path), exist_ok=True)
    tmp = manifest_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(new, f)
    os.replace(tmp, manifest_path)
    print(f"touch_changed: {changed} of {len(new)} files bumped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
