from __future__ import annotations

import subprocess
from pathlib import Path

import firmware_build_id
import pytest


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    files = {
        "src/sched.c": "int x;\n",
        "Makefile": "all:\n",
        "rust/mcu/lib.rs": "fn f() {}\n",
        "docs/notes.md": "notes\n",
        "tools/bench/f401.config": "CONFIG_MACH_STM32=y\n",
        "tools/bench/capture-fw/src/main.rs": "fn main() {}\n",
        "tools/bench/capture-fw/target/junk": "built\n",
        ".gitignore": "target/\n",
    }
    for name, text in files.items():
        (tmp_path / name).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / name).write_text(text)
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    return tmp_path


def _target(tree: Path) -> str:
    return firmware_build_id.target_build_id(
        tree, tree / "tools/bench/f401.config"
    )


def test_a_firmware_source_edit_changes_the_target_id(tree):
    before = _target(tree)
    (tree / "src/sched.c").write_text("int y;\n")
    assert _target(tree) != before


def test_the_kconfig_the_build_runs_under_is_part_of_the_target_id(tree):
    before = _target(tree)
    (tree / "tools/bench/f401.config").write_text("CONFIG_MACH_STM32=n\n")
    assert _target(tree) != before


def test_files_no_firmware_reads_leave_both_ids_alone(tree):
    target, capture = _target(tree), firmware_build_id.capture_build_id(tree)
    (tree / "docs/notes.md").write_text("more notes\n")
    (tree / "tools/bench/capture-fw/target/junk").write_text("rebuilt\n")
    assert _target(tree) == target
    assert firmware_build_id.capture_build_id(tree) == capture


def test_the_capture_id_covers_the_capture_firmware_only(tree):
    capture = firmware_build_id.capture_build_id(tree)
    (tree / "src/sched.c").write_text("int y;\n")
    assert firmware_build_id.capture_build_id(tree) == capture
    (tree / "tools/bench/capture-fw/src/main.rs").write_text("fn main() { }\n")
    assert firmware_build_id.capture_build_id(tree) != capture
