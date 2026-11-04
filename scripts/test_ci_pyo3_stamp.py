import os
import stat
import subprocess
from pathlib import Path

CI = Path(__file__).with_name("ci.sh")


def function_source():
    text = CI.read_text()
    start = text.index('PYO3_STAMP_CHECKED=""')
    end = text.index("host_cargo() {")
    return text[start:end]


def run(
    tmp_path, python_says, existing_stamp=None, build_dir=True, github=True
):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    log = tmp_path / "cargo.log"
    python = bin_dir / "python3"
    python.write_text(f"#!/usr/bin/env bash\necho '{python_says}'\n")
    cargo = bin_dir / "cargo"
    cargo.write_text(f'#!/usr/bin/env bash\necho "$@" >> {log}\n')
    for tool in (python, cargo):
        tool.chmod(tool.stat().st_mode | stat.S_IEXEC)
    rust = tmp_path / "rust"
    target = tmp_path / "target"
    rust.mkdir(exist_ok=True)
    if build_dir:
        (target / "debug" / "build").mkdir(parents=True, exist_ok=True)
    if existing_stamp is not None:
        target.mkdir(exist_ok=True)
        (target / ".pyo3-interpreter").write_text(existing_stamp + "\n")
    env = {
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "RUST": str(rust),
        "CARGO_TARGET_DIR": str(target),
    }
    if github:
        env["GITHUB_ACTIONS"] = "true"
    script = (
        function_source()
        + "\nrefresh_pyo3_for_interpreter\nrefresh_pyo3_for_interpreter\n"
    )
    subprocess.run(
        ["bash", "-c", script], env=env, check=True, capture_output=True
    )
    stamp = (target / ".pyo3-interpreter").read_text().strip()
    return (log.read_text() if log.exists() else ""), stamp


def test_a_restored_target_with_no_stamp_has_its_pyo3_configuration_rebuilt(
    tmp_path,
):
    calls, stamp = run(tmp_path, "3.14 /usr/bin/python3 libpython3.14.so")
    assert calls.count("clean -p pyo3-build-config -p pyo3-ffi") == 1
    assert stamp == "3.14 /usr/bin/python3 libpython3.14.so"


def test_a_stamp_from_another_interpreter_triggers_the_rebuild(tmp_path):
    calls, stamp = run(tmp_path, "3.14 x", existing_stamp="3.12 y")
    assert "clean" in calls and stamp == "3.14 x"


def test_a_matching_stamp_leaves_the_cache_alone(tmp_path):
    calls, _ = run(tmp_path, "3.12 same", existing_stamp="3.12 same")
    assert calls == ""


def test_a_fresh_target_is_only_stamped(tmp_path):
    calls, stamp = run(tmp_path, "3.14 x", build_dir=False)
    assert calls == "" and stamp == "3.14 x"


def test_outside_github_nothing_is_touched(tmp_path):
    calls, _ = run(tmp_path, "3.14 x", github=False, existing_stamp="old")
    assert calls == ""
