"""An xdist worker can load its conftest on an execnet thread: execnet 2.0
runs a task on a new thread when it arrives while the gateway's main thread
is still finishing the previous one (xdist's `-v` rinfo probe). Nothing the
harness imports may install a signal handler, which raises off the main
thread; importing the klippy package does (klippy/util.py fix_sigint).
"""

from __future__ import annotations

import os
import subprocess
import sys

import pytest

pytestmark = pytest.mark.sim_unit

IMPORT_HARNESS_ON_A_THREAD = """
import importlib
import signal
import threading

sigint_handler = signal.getsignal(signal.SIGINT)
failures = []


def import_harness():
    for module in ("tools.sim.conftest", "tools.sim.emulators.cartographer_mcu"):
        try:
            importlib.import_module(module)
        except BaseException as e:
            failures.append(f"{module}: {e!r}")


thread = threading.Thread(target=import_harness)
thread.start()
thread.join()
assert not failures, failures
assert signal.getsignal(signal.SIGINT) is sigint_handler
"""


def test_harness_imports_on_a_thread_without_touching_signals():
    subprocess.run(
        [sys.executable, "-c", IMPORT_HARNESS_ON_A_THREAD],
        env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        check=True,
    )
