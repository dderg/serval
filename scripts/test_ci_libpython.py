import os
import stat
import subprocess
from pathlib import Path

CI = Path(__file__).with_name("ci.sh")


def function_source():
    text = CI.read_text()
    start = text.index('LIBPYTHON_CHECKED=""')
    end = text.index("host_cargo() {")
    return text[start:end]


def run(tmp_path, python_says, github=True):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "sudo.log"
    python = bin_dir / "python3"
    python.write_text(
        f"#!/usr/bin/env bash\ncat >/dev/null\necho {python_says}\n"
    )
    sudo = bin_dir / "sudo"
    sudo.write_text(f'#!/usr/bin/env bash\necho "$@" >> {log}\n')
    for tool in (python, sudo):
        tool.chmod(tool.stat().st_mode | stat.S_IEXEC)
    env = {"PATH": f"{bin_dir}:{os.environ['PATH']}"}
    if github:
        env["GITHUB_ACTIONS"] = "true"
    script = function_source() + "\nensure_libpython\nensure_libpython\n"
    subprocess.run(
        ["bash", "-c", script], env=env, check=True, capture_output=True
    )
    return log.read_text() if log.exists() else ""


def test_a_missing_libpython_on_a_github_runner_installs_the_dev_package_once(
    tmp_path,
):
    calls = run(tmp_path, "False")
    assert calls.count("install -y libpython3-dev") == 1
    assert "update" in calls


def test_an_interpreter_with_its_library_installs_nothing(tmp_path):
    assert run(tmp_path, "True") == ""


def test_outside_github_a_missing_library_is_left_alone(tmp_path):
    assert run(tmp_path, "False", github=False) == ""
