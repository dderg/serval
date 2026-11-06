from __future__ import annotations

import shutil
import subprocess

import pytest

from tools.motion_eval import run, sweep

UPSTREAM_COMMIT = "1ee39ff2c"


@pytest.fixture(scope="module")
def upstream_root(tmp_path_factory):
    root = tmp_path_factory.mktemp("upstream_klippy")
    shutil.copytree(
        run.REPO_ROOT / "klippy",
        root / "klippy",
        ignore=shutil.ignore_patterns("__pycache__"),
    )
    source = subprocess.check_output(
        ["git", "show", UPSTREAM_COMMIT + ":klippy/toolhead.py"],
        cwd=run.REPO_ROOT,
    )
    (root / "klippy" / "toolhead.py").write_bytes(source)
    return root


@pytest.mark.parametrize("config", sorted(sweep.CONFIGS))
@pytest.mark.parametrize("case", sweep.corpus_cases())
def test_no_limiter_plans_exactly_like_upstream(upstream_root, case, config):
    text = (sweep.CORPUS / (case + ".gcode")).read_text()
    cfg = sweep.CONFIGS[config]["cfg"]
    start = sweep.case_start(text)
    ours = run.run_klippy(cfg, text, start).capture["moves"]
    upstream = run.run_klippy(cfg, text, start, klippy_root=upstream_root)
    assert ours == upstream.capture["moves"]
