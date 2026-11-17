"""ship.py formats Python with `ci.sh ruff-format` and the quick gate checks
it with `ci.sh ruff`: both run the one ruff ci.sh pins, and every other place
that names a ruff version (CI's lint workflow, the uv dev group and lock, the
pre-commit hook) pins that same one."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

SCRIPTS = Path(__file__).parent
ROOT = SCRIPTS.parent
PINNED = re.search(
    r'^RUFF_VERSION="([^"]+)"$', (SCRIPTS / "ci.sh").read_text(), re.M
).group(1)


def run_ci(tmp_path: Path, job: str, *, uvx: bool) -> tuple[int, list[str]]:
    root = tmp_path.resolve()
    (root / "scripts").mkdir(parents=True)
    shutil.copy(SCRIPTS / "ci.sh", root / "scripts" / "ci.sh")
    fakes = root / "bin"
    fakes.mkdir()
    calls = root / "calls"
    for tool in ["ruff", "uvx"] if uvx else ["ruff"]:
        fake = fakes / tool
        fake.write_text(f'#!/bin/sh\necho "{tool} $*" >> "{calls}"\n')
        fake.chmod(0o755)
    path = [str(fakes)] + [
        d
        for d in os.environ["PATH"].split(os.pathsep)
        if not (Path(d) / "uvx").exists()
    ]
    result = subprocess.run(
        ["bash", str(root / "scripts" / "ci.sh"), job],
        env={
            "PATH": os.pathsep.join(path),
            "HOME": str(root),
            "CI_NO_SLOT": "1",
        },
        capture_output=True,
        text=True,
    )
    logged = calls.read_text().splitlines() if calls.exists() else []
    return result.returncode, [line.replace(str(root), "ROOT") for line in logged]


def test_the_formatter_and_the_gate_run_the_pinned_ruff(tmp_path):
    assert run_ci(tmp_path / "format", "ruff-format", uvx=True) == (
        0,
        [f"uvx ruff@{PINNED} format ROOT"],
    )
    assert run_ci(tmp_path / "check", "ruff", uvx=True) == (
        0,
        [
            f"uvx ruff@{PINNED} check ROOT",
            f"uvx ruff@{PINNED} format --check ROOT",
        ],
    )


def test_without_uvx_neither_job_falls_back_to_an_unpinned_ruff(tmp_path):
    for job in ("ruff", "ruff-format"):
        rc, calls = run_ci(tmp_path / job, job, uvx=False)
        assert rc != 0 and calls == [], (job, calls)


def test_every_other_ruff_pin_is_the_gates():
    workflow = (ROOT / ".github/workflows/ci-lintformat.yaml").read_text()
    assert set(re.findall(r'version: "([^"]+)"', workflow)) == {PINNED}

    pyproject = (ROOT / "pyproject.toml").read_text()
    assert set(re.findall(r'"ruff([^"]*)"', pyproject)) == {f"=={PINNED}"}

    lock = (ROOT / "uv.lock").read_text()
    locked = re.findall(r'name = "ruff"\nversion = "([^"]+)"', lock)
    assert locked == [PINNED]

    pre_commit = (ROOT / ".pre-commit-config.yaml").read_text()
    revs = re.findall(r"ruff-pre-commit\n\s+rev: v(\S+)", pre_commit)
    assert revs == [PINNED]
