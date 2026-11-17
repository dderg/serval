import os
import resource
import subprocess
import sys
from pathlib import Path

import job_scope
import pytest
from job_scope import Caps, CoreDumps

SLOTS = Path(__file__).with_name("slots.py")
TINY = Caps(high_gib=1, max_gib=1, swap_gib=0, cores=CoreDumps.DROP)
OVER_TINY = "x = b'\\x01' * (2 * 1024**3)"


def user_manager() -> bool:
    probe = subprocess.run(
        ["systemctl", "--user", "is-system-running"],
        capture_output=True,
        text=True,
    )
    return probe.stdout.strip() in ("running", "degraded")


needs_manager = pytest.mark.skipif(
    not user_manager(), reason="no systemd user manager (container or CI)"
)


@pytest.fixture(autouse=True)
def capped_env(monkeypatch):
    monkeypatch.delenv(job_scope.ENV_UNCAPPED, raising=False)


def properties(call: list[str]) -> dict[str, str]:
    flat = call[call.index("fail") + 2 : -1]
    found, index = {}, 0
    while index < len(flat):
        name, kind = flat[index], flat[index + 1]
        width = 2 if kind == "au" else 1
        found[name] = " ".join(flat[index + 2 : index + 2 + width])
        index += 2 + width
    return found


def test_the_scope_call_caps_memory_and_keeps_the_caller_alive_on_a_kill():
    call = job_scope.start_call("serval-slot-7-1.scope", 7, TINY, "job")
    assert call[:3] == ["busctl", "--user", "call"]
    assert "serval-slot-7-1.scope" in call
    assert properties(call) == {
        "PIDs": "1 7",
        "MemoryHigh": str(2**30),
        "MemoryMax": str(2**30),
        "MemorySwapMax": "0",
        "OOMPolicy": "continue",
        "CollectMode": "inactive-or-failed",
        "Description": "job",
    }


def test_no_user_manager_fails_loudly(monkeypatch):
    monkeypatch.setattr(job_scope, "start_call", lambda *_: ["false"])
    with pytest.raises(job_scope.NoUserManager, match="SERVAL_SLOT_UNCAPPED"):
        with job_scope.capped("job", TINY):
            pass


def test_the_opt_out_runs_uncapped(monkeypatch):
    monkeypatch.setenv(job_scope.ENV_UNCAPPED, "1")
    monkeypatch.setattr(job_scope, "start_call", lambda *_: ["false"])
    with job_scope.capped("job", TINY):
        pass


HOLDER = (
    "import subprocess, sys\n"
    "import job_scope\n"
    "from job_scope import Caps, CoreDumps\n"
    "caps = Caps(high_gib=1, max_gib=1, swap_gib=0, cores=CoreDumps.DROP)\n"
    "try:\n"
    "    with job_scope.capped('tiny job', caps):\n"
    f"        subprocess.run([sys.executable, '-c', {OVER_TINY!r}])\n"
    "except job_scope.MemoryCapKill as kill:\n"
    "    print(kill)\n"
    "    print('holder alive in', job_scope.own_cgroup().name)\n"
)


@needs_manager
def test_a_job_over_its_cap_is_killed_alone_and_named():
    bystander = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"]
    )
    try:
        done = subprocess.run(
            [sys.executable, "-c", HOLDER],
            env={**os.environ, "PYTHONPATH": str(SLOTS.parent)},
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert done.returncode == 0, done.stderr
        assert "tiny job was killed by its memory cap (MemoryMax=1 GiB" in (
            done.stdout
        )
        assert "holder alive in serval-slot-" in done.stdout
        assert bystander.poll() is None, "the cap killed outside its scope"
    finally:
        bystander.kill()
        bystander.wait()


@needs_manager
def test_a_sweep_scope_drops_core_dumps_for_its_children_only():
    limit = resource.getrlimit(resource.RLIMIT_CORE)
    with job_scope.capped("sweep", TINY):
        child = subprocess.run(
            [
                sys.executable,
                "-c",
                "import resource; print(resource.getrlimit(resource.RLIMIT_CORE)[0])",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
    assert child.stdout.strip() == "0"
    assert resource.getrlimit(resource.RLIMIT_CORE) == limit


SLOT_CLI = (
    "import sys\n"
    "import slots\n"
    "from job_scope import Caps, CoreDumps\n"
    "slots.CAPS[slots.Kind.SWEEP] = Caps(1, 1, 0, CoreDumps.DROP)\n"
    "sys.argv = ['slots.py', 'run', '--kind', 'sweep', '--',\n"
    f"            sys.executable, '-c', {OVER_TINY!r}]\n"
    "sys.exit(slots.cli())\n"
)


@needs_manager
def test_a_slot_job_killed_by_its_cap_exits_with_the_named_code(tmp_path):
    import slots

    done = subprocess.run(
        [sys.executable, "-c", SLOT_CLI],
        env={
            **os.environ,
            "PYTHONPATH": str(SLOTS.parent),
            "SERVAL_SLOTS_DIR": str(tmp_path / "slots"),
            "SERVAL_SLOT_HELD": "",
        },
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert done.returncode == slots.MEMORY_CAP_EXIT, done.stderr
    assert "was killed by its memory cap (MemoryMax=1 GiB" in done.stderr


BOX_RAM_GIB = 62


def test_the_caps_of_every_slot_fit_in_ram_with_room_for_the_session():
    import slots

    slots.check_caps_fit(slots.DEFAULT_SLOTS, BOX_RAM_GIB)
    with pytest.raises(ValueError, match="lower SERVAL_SLOTS"):
        slots.check_caps_fit(2 * slots.DEFAULT_SLOTS, BOX_RAM_GIB)
