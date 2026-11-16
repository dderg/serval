from __future__ import annotations

import pathlib
import subprocess

import pytest

from tools.sim.prune_rust import prune

pytestmark = pytest.mark.sim_unit

ROOT_MANIFEST = """\
[workspace]
members = ["app", "base", "devtool", "tool"]
resolver = "2"
"""

LOCK = "version = 4\n"


def _crate(root: pathlib.Path, name: str, manifest_tail: str, files) -> None:
    crate = root / name
    (crate / "src").mkdir(parents=True)
    (crate / "Cargo.toml").write_text(
        f'[package]\nname = "{name}"\nversion = "0.1.0"\nedition = "2021"\n'
        + manifest_tail
    )
    for rel, text in files.items():
        path = crate / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


@pytest.fixture
def workspace(tmp_path):
    root = tmp_path / "src"
    root.mkdir()
    (root / "Cargo.toml").write_text(ROOT_MANIFEST)
    (root / "Cargo.lock").write_text(LOCK)
    (root / ".cargo").mkdir()
    (root / ".cargo" / "config.toml").write_text("[build]\n")
    _crate(
        root,
        "app",
        '[dependencies]\nbase = { path = "../base" }\n'
        '[dev-dependencies]\ndevtool = { path = "../devtool" }\n'
        '[[bench]]\nname = "speed"\nharness = false\n',
        {
            "src/lib.rs": "pub use base::VALUE;\n",
            "benches/speed.rs": "fn main() {}\n",
            "tests/app.rs": "#[test]\nfn t() {}\n",
        },
    )
    _crate(root, "base", "", {"src/lib.rs": "pub const VALUE: u8 = 7;\n"})
    _crate(root, "devtool", "", {"src/lib.rs": "pub fn helper() {}\n"})
    _crate(
        root,
        "tool",
        '[[bin]]\nname = "gen"\npath = "src/bin/gen.rs"\n'
        '[[test]]\nname = "golden"\n',
        {"src/bin/gen.rs": "fn main() {}\n", "tests/golden.rs": "\n"},
    )
    return root


def _load(tree: pathlib.Path) -> None:
    subprocess.run(
        ["cargo", "metadata", "--offline", "--format-version=1"],
        cwd=tree,
        check=True,
        capture_output=True,
    )


def test_closure_crates_keep_their_build_sources(workspace, tmp_path):
    dest = tmp_path / "pruned"

    prune(workspace, dest, ["app"])

    assert (
        dest / "base/src/lib.rs"
    ).read_text() == "pub const VALUE: u8 = 7;\n"
    assert (dest / "app/src/lib.rs").read_text() == "pub use base::VALUE;\n"
    assert (dest / ".cargo/config.toml").exists()
    assert not (dest / "app/tests").exists()
    assert (dest / "app/benches/speed.rs").read_text() == ""
    _load(dest)


def test_other_members_become_stubs_that_still_load(workspace, tmp_path):
    dest = tmp_path / "pruned"

    prune(workspace, dest, ["app"])

    assert (dest / "devtool/src/lib.rs").read_text() == ""
    assert (dest / "tool/src/bin/gen.rs").read_text() == ""
    assert (dest / "tool/tests/golden.rs").read_text() == ""
    assert (dest / "Cargo.lock").read_text() == LOCK
    _load(dest)


def test_without_packages_every_member_is_a_stub(workspace, tmp_path):
    dest = tmp_path / "skeleton"

    prune(workspace, dest, [])

    assert (dest / "base/src/lib.rs").read_text() == ""
    assert (dest / "app/Cargo.toml").read_text() == (
        workspace / "app/Cargo.toml"
    ).read_text()
    _load(dest)


def test_unknown_package_fails_loudly(workspace, tmp_path):
    with pytest.raises(SystemExit, match="not a workspace member"):
        prune(workspace, tmp_path / "pruned", ["missing"])
