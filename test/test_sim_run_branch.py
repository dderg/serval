"""tools/sim/run.sh --branch X tests X: its image is built from X's own
tools/sim. Overlaying this worktree's harness on X takes
--overlay-sim-tools, and then the banner, the run's closing line and the
image tag all say so (#952: a "main fails" report ran a branch's libvtime
on main)."""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

RUN_SH = Path(__file__).resolve().parents[1] / "tools" / "sim" / "run.sh"
HARNESS = "tools/sim/harness.py"

FAKE_DOCKER = """#!/bin/bash
if [[ "$1" == build ]]; then
    for ((i = 1; i <= $#; i++)); do
        [[ "${!i}" == -t ]] && { j=$((i + 1)); echo "${!j}" > "$OUT/tag"; }
    done
    cp "${!#}/tools/sim/harness.py" "$OUT/built_harness"
fi
"""


def git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    root = tmp_path / "wt"
    (root / "tools" / "sim").mkdir(parents=True)
    shutil.copy(RUN_SH, root / "tools" / "sim" / "run.sh")
    (root / "tools" / "sim" / "Dockerfile").write_text("FROM scratch\n")
    git(root, "init", "-q", "-b", "main")
    git(root, "config", "user.email", "t@t")
    git(root, "config", "user.name", "t")
    (root / HARNESS).write_text("branch harness\n")
    git(root, "add", "-A")
    git(root, "commit", "-q", "-m", "x")
    git(root, "branch", "other")
    (root / HARNESS).write_text("worktree harness\n")
    git(root, "commit", "-q", "-am", "worktree")
    (root / HARNESS).write_text("uncommitted harness\n")
    return root


def run_sim(repo: Path, *args: str) -> tuple[int, str, Path]:
    out = repo.parent / "out"
    out.mkdir(exist_ok=True)
    fakes = repo.parent / "bin"
    fakes.mkdir(exist_ok=True)
    (fakes / "docker").write_text(FAKE_DOCKER)
    (fakes / "docker").chmod(0o755)
    result = subprocess.run(
        ["bash", str(repo / "tools" / "sim" / "run.sh"), "test", *args],
        cwd=repo,
        env={**os.environ, "PATH": f"{fakes}:{os.environ['PATH']}", "OUT": str(out)},
        capture_output=True,
        text=True,
    )
    return result.returncode, result.stdout + result.stderr, out


def test_branch_builds_its_own_tools_sim(repo):
    rc, output, out = run_sim(repo, "--branch", "other")
    assert rc == 0, output
    assert (out / "built_harness").read_text() == "branch harness\n"
    assert (out / "tag").read_text().strip() == "kalico-sim-other"
    sha = git(repo, "rev-parse", "--short", "other")
    assert f"tools/sim: other @ {sha}" in output
    assert "OVERLAID" not in output


def test_the_overlay_is_opt_in_and_named_everywhere(repo):
    rc, output, out = run_sim(repo, "--branch", "other", "--overlay-sim-tools")
    assert rc == 0, output
    assert (out / "built_harness").read_text() == "uncommitted harness\n"
    assert (out / "tag").read_text().strip() == (
        "kalico-sim-other.sim-tools-wt"
    )
    head = git(repo, "rev-parse", "--short", "HEAD")
    line = (
        f"tools/sim: OVERLAID from {repo} @ {head} +dirty,"
        f" 1 file differs from other"
    )
    assert output.count(line) == 2, output
    assert HARNESS in output


def test_an_overlay_without_a_branch_is_refused(repo):
    rc, output, out = run_sim(repo, "--overlay-sim-tools")
    assert rc == 2
    assert "--overlay-sim-tools" in output
    assert not (out / "built_harness").exists()
