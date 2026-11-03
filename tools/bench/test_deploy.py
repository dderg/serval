from __future__ import annotations

import deploy


def test_a_remote_file_missing_from_the_tree_is_stale():
    remote = ["rust/motion-engine/build.rs", "rust/a.rs", "klippy/x.py"]
    local = ["rust/a.rs", "klippy/x.py"]
    assert deploy.stale_remote_files(remote, local) == [
        "rust/motion-engine/build.rs"
    ]


def test_build_outputs_are_never_stale():
    remote = [
        ".bench-deploy.json",
        "klippy/_motion_engine.so",
        "klippy/chelper/c_helper.Linux-aarch64.so",
        "klippy/__pycache__/gcode.cpython-313.pyc",
        "rust/target/release/x",
        ".git/HEAD",
    ]
    assert deploy.stale_remote_files(remote, []) == []


def test_files_the_tree_carries_are_kept():
    names = ["a.py", "rust/b.rs"]
    assert deploy.stale_remote_files(names, names) == []
