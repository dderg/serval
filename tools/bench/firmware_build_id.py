"""Which sources built a bench firmware.

A firmware build id is the first ID_HEX_CHARS of the sha256 over every file
git tracks or would add under the firmware's sources (path, length, content,
sorted by path), read from the working tree, so dirty and new files count,
the way the motion engine's build id covers rust/. The F401 printer firmware
reads src/, lib/, rust/ (the mcu crates), scripts/ (buildcommands.py writes
its dictionary) and the Makefile, built under one kconfig; the F407 capture
firmware reads tools/bench/capture-fw. deploy.py bakes the id into the build
and refuses a flashed board that reports any other.
"""

from __future__ import annotations

import hashlib
import os
import subprocess
from pathlib import Path

ID_HEX_CHARS = 40
TARGET_SOURCES = ("src", "lib", "rust", "scripts", "Makefile")
CAPTURE_SOURCES = ("tools/bench/capture-fw",)


def source_names(tree: Path, pathspecs: tuple[str, ...]) -> list[str]:
    listing = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "-co",
            "--exclude-standard",
            "--",
            *pathspecs,
        ],
        cwd=tree,
        check=True,
        capture_output=True,
    ).stdout
    names = {n for n in listing.decode().split("\0") if n}
    return sorted(n for n in names if os.path.lexists(tree / n))


def digest(tree: Path, names: list[str]) -> str:
    sha = hashlib.sha256()
    for name in names:
        content = (tree / name).read_bytes()
        sha.update(name.encode() + b"\0")
        sha.update(str(len(content)).encode() + b"\0")
        sha.update(content)
    return sha.hexdigest()[:ID_HEX_CHARS]


def target_build_id(tree: Path, kconfig: Path) -> str:
    names = source_names(tree, TARGET_SOURCES)
    config = kconfig.resolve().relative_to(tree.resolve()).as_posix()
    return digest(tree, sorted({*names, config}))


def capture_build_id(tree: Path) -> str:
    return digest(tree, source_names(tree, CAPTURE_SOURCES))
