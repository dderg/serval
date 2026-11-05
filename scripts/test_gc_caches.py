import os
import subprocess
import sys
import time
from pathlib import Path

import gc_caches as gc


def touch_dir(path: Path, age_seconds: float) -> Path:
    path.mkdir(parents=True)
    stamp = time.time() - age_seconds
    os.utime(path, (stamp, stamp))
    return path


def test_the_scratch_target_cap_drops_only_the_oldest(tmp_path):
    dirs = [
        touch_dir(tmp_path / f"t{i}", i * 60)
        for i in range(gc.MAX_SCRATCH_TARGETS + 3)
    ]
    dropped = gc.over_scratch_cap(dirs)
    assert set(dropped) == set(dirs[gc.MAX_SCRATCH_TARGETS :])


def test_under_the_cap_nothing_is_dropped(tmp_path):
    dirs = [touch_dir(tmp_path / f"t{i}", i) for i in range(3)]
    assert gc.over_scratch_cap(dirs) == []


def image_listing(*rows):
    return [f"{name}\t{created}" for name, created in rows]


def hours_ago(hours):
    stamp = time.time() - hours * 3600
    return time.strftime("%Y-%m-%d %H:%M:%S +0100 CET", time.localtime(stamp))


def test_only_old_unused_repo_images_are_stale(monkeypatch):
    def fake(*args):
        if args[0] == "ps":
            return ["kalico-sim-busy:latest"]
        return image_listing(
            ("klipper-build-old:latest", hours_ago(30)),
            ("klipper-build-new:latest", hours_ago(1)),
            ("kalico-sim-busy:latest", hours_ago(30)),
            ("ubuntu:24.04", hours_ago(500)),
        )

    monkeypatch.setattr(gc, "docker_lines", fake)
    assert gc.stale_docker_images(12) == ["klipper-build-old:latest"]


def test_no_docker_means_no_stale_images(monkeypatch):
    monkeypatch.setattr(gc, "docker_lines", lambda *args: [])
    assert gc.stale_docker_images(1) == []


def run_main(monkeypatch, capsys, free, *argv):
    monkeypatch.setattr(gc, "free_gib", lambda path=gc.CACHE: free)
    monkeypatch.setattr(gc, "candidates", lambda: pytest_fail())
    monkeypatch.setattr(sys, "argv", ["gc_caches.py", *argv])


def pytest_fail():
    raise AssertionError("cleaning ran")


def test_if_free_below_is_a_no_op_when_there_is_enough_space(
    monkeypatch, capsys
):
    run_main(monkeypatch, capsys, 500.0, "--if-free-below", "100")
    gc.main()
    assert capsys.readouterr().err == ""


def test_if_free_below_cleans_when_space_is_short(monkeypatch, capsys):
    monkeypatch.setattr(gc, "free_gib", lambda path=gc.CACHE: 20.0)
    monkeypatch.setattr(gc, "candidates", lambda: [])
    monkeypatch.setattr(gc, "worktree_paths", lambda: [])
    monkeypatch.setattr(gc, "process_paths", lambda: [])
    monkeypatch.setattr(gc, "reclaimable_worktree_targets", lambda: [])
    monkeypatch.setattr(gc, "stale_docker_images", lambda hours: [])
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, 0, "", ""),
    )
    monkeypatch.setattr(sys, "argv", ["gc_caches.py", "--if-free-below", "100"])
    gc.main()
    assert "below 100" in capsys.readouterr().err


def test_a_worktree_target_held_by_a_process_is_kept():
    busy = [Path("/wt/rust/target/debug")]
    assert gc.process_holder(Path("/wt/rust/target"), []) is None
    assert gc.process_holder(Path("/wt/rust"), busy) is not None
