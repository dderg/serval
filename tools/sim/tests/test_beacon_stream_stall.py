"""The beacon stub streams samples from a real-time thread while virtual
time free-runs, so a few hundred real milliseconds of lag in that thread (a
starved CI runner) once became seconds of printer time without a sample,
past the plugin's one-second stream timeout: "Beacon sensor not receiving
data" shut klippy down mid-probe. The stub's stream thread now holds the
virtual clock within a stall budget of the next due batch."""

import time

import pytest

from tools.sim.tests.test_beacon import _cfg, _home

pytestmark = pytest.mark.needs_elf

STALLS = 4
STALL_REAL_S = 0.6
BATCHES_BETWEEN_STALLS = 25


def test_a_stalled_sample_thread_cannot_starve_the_plugin_of_samples(sim_world):
    world = sim_world(_cfg(), beacon=True)
    _home(world)
    world.gcode_ok("G1 Z3 F600", timeout=30)

    send = world.beacon._send_msg
    state = {"batches": 0, "stalls": 0}

    def stalling_send(msgformat, **kwargs):
        if msgformat.startswith("beacon_data"):
            state["batches"] += 1
            if (
                state["batches"] % BATCHES_BETWEEN_STALLS == 0
                and state["stalls"] < STALLS
            ):
                state["stalls"] += 1
                time.sleep(STALL_REAL_S)
        send(msgformat, **kwargs)

    world.beacon._send_msg = stalling_send
    world.gcode_ok("PROBE PROBE_METHOD=proximity SAMPLES=2", timeout=120)
    world.gcode_ok("PROBE_ACCURACY SAMPLES=3", timeout=180)

    assert world.shutdown_line() is None
    assert state["stalls"] == STALLS, state
