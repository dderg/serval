"""VTIME_CRYSTAL_PPM gives one process its own oscillator: its clock runs
that many ppm off the shared virtual timeline, and the deadlines it sets on
its own clock land at the matching shared instant."""

import os
import subprocess
import sys
import textwrap

import pytest

from tools.sim.world import vtime_create, vtime_destroy

pytestmark = pytest.mark.sim_unit

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
LIBVTIME = os.path.join(REPO_ROOT, "tools/sim/preload/libvtime.so")
CRYSTAL_PPM = 250.0


@pytest.fixture
def shm_name():
    assert os.path.exists(LIBVTIME), (
        "build shim first: make -C tools/sim/preload"
    )
    name = f"/vtime-crystal-{os.getpid()}"
    vtime_create(name)
    yield name
    vtime_destroy(name)


def test_a_fast_crystal_reads_and_sleeps_on_its_own_clock(shm_name):
    env = os.environ.copy()
    env["LD_PRELOAD"] = LIBVTIME
    env["VTIME_SHM_NAME"] = shm_name
    env["VTIME_SPEED"] = "1000"
    env["VTIME_CRYSTAL_PPM"] = str(CRYSTAL_PPM)
    script = f"""
        import mmap, os, struct, time
        fd = os.open("/dev/shm" + {shm_name!r}, os.O_RDONLY)
        shm = mmap.mmap(fd, 8, prot=mmap.PROT_READ)
        def shared():
            return struct.unpack_from("<Q", shm, 0)[0]
        shared_start, local_start = shared(), time.monotonic_ns()
        time.sleep(2.0)
        local_end, shared_end = time.monotonic_ns(), shared()
        print(local_start / shared_start,
              (local_end - local_start) / (shared_end - shared_start),
              (local_end - local_start) / 1e9)
    """
    out = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert out.returncode == 0, out.stderr
    reading_ratio, rate_ratio, slept_local_s = map(float, out.stdout.split())
    scale = 1.0 + CRYSTAL_PPM * 1e-6
    assert reading_ratio == pytest.approx(scale, abs=1e-5)
    assert rate_ratio == pytest.approx(scale, abs=1e-5)
    assert slept_local_s >= 2.0, (
        f"a 2 s sleep on the process's own clock woke after {slept_local_s} s"
    )
