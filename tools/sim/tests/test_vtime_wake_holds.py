"""Forced repro of virtual time leaping while a woken participant waits for
a CPU: libvtime's VTIME_TEST_WOKEN_SPIN_US keeps every host thread that a
wait returned early to runnable-but-not-running for that long, the state a
starved CI runner leaves it in. The clock must not move until it runs."""

import os
import subprocess
import sys
import textwrap

import pytest

from tools.sim.world import vtime_create, vtime_destroy

pytestmark = pytest.mark.sim_unit

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
LIBVTIME = os.path.join(REPO_ROOT, "tools/sim/preload/libvtime.so")
STARVED_US = 100_000
CLOCK_PUSHES = 100
PUSH_S = 0.001
HELD_TOLERANCE_S = 0.0002


@pytest.fixture
def shm_name():
    assert os.path.exists(LIBVTIME), (
        "build shim first: make -C tools/sim/preload"
    )
    name = f"/vtime-wake-holds-{os.getpid()}"
    vtime_create(name)
    yield name
    vtime_destroy(name)


def _run_host(shm_name, script):
    env = os.environ.copy()
    env["LD_PRELOAD"] = LIBVTIME
    env["VTIME_SHM_NAME"] = shm_name
    env["VTIME_SPEED"] = "1000"
    env["VTIME_HOST"] = "1"
    env["VTIME_TEST_WOKEN_SPIN_US"] = str(STARVED_US)
    out = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert out.returncode == 0, out.stderr
    return float(out.stdout.strip())


def test_woken_host_thread_holds_the_clock_until_it_runs(shm_name):
    lag = _run_host(
        shm_name,
        f"""
        import threading, time
        woken = threading.Event()
        woke_at = []
        def sleeper():
            woken.wait()
            woke_at.append(time.monotonic())
        thread = threading.Thread(target=sleeper)
        thread.start()
        time.sleep(0.01)
        set_at = time.monotonic()
        woken.set()
        for _ in range({CLOCK_PUSHES}):
            time.sleep({PUSH_S})
        thread.join()
        print(woke_at[0] - set_at)
        """,
    )
    assert lag < HELD_TOLERANCE_S, (
        f"virtual time moved {lag * 1e3:.3f} ms while the woken thread "
        "waited for a CPU"
    )
