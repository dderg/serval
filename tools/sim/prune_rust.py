#!/usr/bin/env python3
"""Copy the cargo workspace keeping only the crates that build some packages.

    prune_rust.py SRC DEST [PACKAGE ...]

Workspace members outside the packages' normal and build dependency closure
keep their manifest over empty target files, so the workspace and its
Cargo.lock still resolve unchanged. A Docker stage building from DEST then
stays cached when a crate outside its closure changes. Closure crates drop
their tests/, benches/ and examples/, which `cargo build -p` never compiles.
With no packages every member is stubbed: the tree `cargo fetch` needs.
"""

from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys

import tomllib

ROOT_FILES = ("Cargo.toml", "Cargo.lock")
ROOT_DIRS = (".cargo",)
NON_BUILD_DIRS = frozenset(("tests", "benches", "examples"))
BUILD_DEP_KINDS = (None, "build")
TARGET_DIRS = {
    "bin": "src/bin",
    "test": "tests",
    "bench": "benches",
    "example": "examples",
}


def workspace_members(src: pathlib.Path) -> dict[str, dict]:
    out = subprocess.run(
        ["cargo", "metadata", "--no-deps", "--offline", "--format-version=1"],
        cwd=src,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return {pkg["name"]: pkg for pkg in json.loads(out)["packages"]}


def build_closure(members: dict[str, dict], roots: list[str]) -> set[str]:
    closure: set[str] = set()
    pending = list(roots)
    while pending:
        name = pending.pop()
        if name in closure:
            continue
        if name not in members:
            raise SystemExit(f"prune_rust: {name} is not a workspace member")
        closure.add(name)
        pending += [
            dep["name"]
            for dep in members[name]["dependencies"]
            if dep.get("path") and dep["kind"] in BUILD_DEP_KINDS
        ]
    return closure


def declared_targets(manifest: dict) -> list[str]:
    paths = []
    if "path" in manifest.get("lib", {}):
        paths.append(manifest["lib"]["path"])
    for table, directory in TARGET_DIRS.items():
        paths += [
            t.get("path", f"{directory}/{t['name']}.rs")
            for t in manifest.get(table, [])
        ]
    build = manifest.get("package", {}).get("build")
    if isinstance(build, str):
        paths.append(build)
    return paths


def touch_files(dest: pathlib.Path, rels: list[str]) -> None:
    for rel in rels:
        stub = dest / rel
        stub.parent.mkdir(parents=True, exist_ok=True)
        stub.touch()


def write_stub(crate: pathlib.Path, dest: pathlib.Path) -> None:
    manifest_text = (crate / "Cargo.toml").read_text()
    dest.mkdir(parents=True)
    (dest / "Cargo.toml").write_text(manifest_text)
    targets = declared_targets(tomllib.loads(manifest_text))
    touch_files(dest, ["src/lib.rs", *targets])


def copy_crate(crate: pathlib.Path, dest: pathlib.Path) -> None:
    def skip_non_build(directory: str, names: list[str]) -> list[str]:
        if pathlib.Path(directory) != crate:
            return []
        return [name for name in names if name in NON_BUILD_DIRS]

    shutil.copytree(crate, dest, ignore=skip_non_build)
    manifest = tomllib.loads((crate / "Cargo.toml").read_text())
    dropped = [
        rel
        for rel in declared_targets(manifest)
        if pathlib.PurePosixPath(rel).parts[0] in NON_BUILD_DIRS
    ]
    touch_files(dest, dropped)


def prune(src: pathlib.Path, dest: pathlib.Path, roots: list[str]) -> None:
    members = workspace_members(src)
    closure = build_closure(members, roots)
    dest.mkdir(parents=True)
    for name in ROOT_FILES:
        shutil.copy2(src / name, dest / name)
    for name in ROOT_DIRS:
        shutil.copytree(src / name, dest / name)
    for name, pkg in members.items():
        crate = pathlib.Path(pkg["manifest_path"]).parent
        crate_dest = dest / crate.relative_to(src)
        if name in closure:
            copy_crate(crate, crate_dest)
        else:
            write_stub(crate, crate_dest)


def main() -> None:
    src = pathlib.Path(sys.argv[1]).resolve()
    dest = pathlib.Path(sys.argv[2])
    prune(src, dest, sys.argv[3:])


if __name__ == "__main__":
    main()
