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


def _env(shm_name):
    env = os.environ.copy()
    env["LD_PRELOAD"] = LIBVTIME
    env["VTIME_SHM_NAME"] = shm_name
    env["VTIME_SPEED"] = "1000"
    return env


def _run_host(shm_name, script, *args):
    env = _env(shm_name)
    env["VTIME_HOST"] = "1"
    env["VTIME_TEST_WOKEN_SPIN_US"] = str(STARVED_US)
    out = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script), *args],
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


def test_a_forked_host_child_leaves_the_host_gate_alone(shm_name):
    host_present = _run_host(
        shm_name,
        f"""
        import os, struct, sys, time
        child = os.fork()
        if child == 0:
            time.sleep({PUSH_S})
            sys.exit(0)
        while os.waitpid(child, os.WNOHANG) == (0, 0):
            time.sleep({PUSH_S})
        with open("/dev/shm" + {shm_name!r}, "rb") as shm:
            shm.seek(232)
            print(struct.unpack("<I", shm.read(4))[0])
        """,
    )
    assert host_present == 1, (
        "a forked klippy child's exit cleared host_present for the parent"
    )


DEVICE = """
import ctypes, os, select, time, tty
libc = ctypes.CDLL(None)
def fortified_read(fd, n):
    buf = ctypes.create_string_buffer(n)
    got = libc.__read_chk(fd, buf, n, n)
    return buf.raw[:got]
read = fortified_read if {fortified} else os.read
master, slave = os.openpty()
tty.setraw(slave)
print(os.ttyname(slave), flush=True)
select.select([master], [], [])
read(master, 64)
starved_until = time.time() + {starved_s}
while time.time() < starved_until:
    pass
os.write(master, repr(time.monotonic()).encode() + b"\\n")
select.select([master], [], [])
"""


@pytest.mark.parametrize("fortified", [False, True])
def test_a_device_answers_at_the_instant_the_host_asked(shm_name, fortified):
    device = subprocess.Popen(
        [
            sys.executable,
            "-c",
            DEVICE.format(fortified=fortified, starved_s=STARVED_US / 1e6),
        ],
        env=_env(shm_name),
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        lag = _run_host(
            shm_name,
            f"""
            import os, sys, time, tty
            fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)
            tty.setraw(fd)
            sent_at = time.monotonic()
            os.write(fd, b"clock?")
            for _ in range({CLOCK_PUSHES}):
                time.sleep({PUSH_S})
            reply = b""
            while not reply.endswith(b"\\n"):
                reply += os.read(fd, 64)
            print(float(reply) - sent_at)
            """,
            device.stdout.readline().strip(),
        )
    finally:
        device.kill()
        device.wait()
    assert lag < HELD_TOLERANCE_S, (
        f"the device answered {lag * 1e3:.3f} ms of virtual time after the "
        "host asked"
    )
