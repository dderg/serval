import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _thread_count_after(code, env):
    status = subprocess.run(
        [
            sys.executable,
            "-c",
            code + "; print(open('/proc/self/status').read())",
        ],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return int(re.search(r"^Threads:\s+(\d+)$", status, re.MULTILINE).group(1))


def test_importing_klippy_keeps_numpy_blas_on_the_importing_thread():
    env = {
        key: value
        for key, value in os.environ.items()
        if key != "OPENBLAS_NUM_THREADS"
    }
    assert _thread_count_after("import klippy, numpy", env) == 1


def test_an_explicit_blas_thread_count_wins(monkeypatch):
    from klippy.blas import limit_blas_to_one_thread

    monkeypatch.setenv("OPENBLAS_NUM_THREADS", "2")
    limit_blas_to_one_thread()
    assert os.environ["OPENBLAS_NUM_THREADS"] == "2"
