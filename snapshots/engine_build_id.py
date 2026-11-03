"""Which Rust sources built `_motion_engine`, and whether they are the ones in
the tree the engine sits in.

The engine carries a build id: the first ID_HEX_CHARS of the sha256 over every
file under rust/ that git tracks or would add (path, length, content, sorted by
path), read from the working tree at build time, so dirty and new files count.
motion-engine's build.rs computes it; tree_build_id() computes the same value
from a tree. A snapshot, analyzer or review that loads an engine built from
other sources than the tree it sits in reports a reference that is not the
code in front of the reader, so verify() refuses it. A git-less tree (the CI
image's copy) holds an engine built there, also `unknown`: nothing to compare.
"""

import functools
import hashlib
import os
import subprocess
from pathlib import Path

ID_HEX_CHARS = 40
UNKNOWN = "unknown"
REBUILD_HINT = "make -f Makefile.rust motion-engine-fast"


class EngineMismatch(ImportError):
    pass


def is_git_tree(repo_root: Path) -> bool:
    return (
        subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            cwd=repo_root,
            capture_output=True,
        ).returncode
        == 0
    )


def source_names(repo_root: Path) -> list[str]:
    listing = subprocess.run(
        ["git", "ls-files", "-z", "-co", "--exclude-standard", "--", "rust"],
        cwd=repo_root,
        check=True,
        capture_output=True,
    ).stdout
    names = {n for n in listing.decode().split("\0") if n}
    return sorted(n for n in names if os.path.lexists(repo_root / n))


@functools.lru_cache(maxsize=None)
def tree_build_id(repo_root: Path) -> str:
    digest = hashlib.sha256()
    for name in source_names(repo_root):
        content = (repo_root / name).read_bytes()
        digest.update(name.encode() + b"\0")
        digest.update(str(len(content)).encode() + b"\0")
        digest.update(content)
    return digest.hexdigest()[:ID_HEX_CHARS]


def verify(engine) -> None:
    if not hasattr(engine, "build_id"):
        raise EngineMismatch(
            f"{engine.__file__} predates build ids: rebuild with {REBUILD_HINT}"
        )
    built = engine.build_id()
    root = Path(engine.__file__).resolve().parents[1]
    in_git_tree = is_git_tree(root)
    if built == UNKNOWN and not in_git_tree:
        return
    if built == UNKNOWN:
        raise EngineMismatch(
            f"{engine.__file__} was built outside a git tree but sits in one, "
            f"so its sources cannot be checked: rebuild with {REBUILD_HINT}"
        )
    if not in_git_tree:
        raise EngineMismatch(
            f"{engine.__file__} carries build id {built} but {root} is not a "
            "git tree, so it cannot be compared"
        )
    current = tree_build_id(root)
    if built != current:
        raise EngineMismatch(
            f"{engine.__file__} was built from rust/ id {built}, but the tree "
            f"at {root} is {current}: rebuild with {REBUILD_HINT}"
        )
