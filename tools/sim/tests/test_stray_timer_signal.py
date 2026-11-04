"""A SIGALRM the MCU did not arm for its current head timer (a superseded
alarm whose signal lands after the re-arm, which libvtime's backstop thread
and the kernel can both produce) must not dispatch that head early. The linux
MCU's timer_dispatch used to run the timer-list head on any signal, so a
future step timer fired before its waketime and the step-edge guard shut the
MCU down: "Step timer dispatched before its waketime" (#633). This test sends
SIGALRM to the MCU every few real milliseconds through a slow move."""

import ctypes
import os
import signal
import threading
import time

import pytest

from tools.sim.tests.test_issue_420_z_tilt import _config as follower_config

pytestmark = pytest.mark.needs_elf

SIGNAL_PERIOD_REAL_S = 0.003
SYS_TGKILL = 234
libc = ctypes.CDLL(None, use_errno=True)
MOVE = "G1 X175 F300"
START = "SET_KINEMATIC_POSITION X=125 Y=125 Z=5"


def _config(world):
    return follower_config(world)


def test_a_stray_alarm_signal_does_not_fire_a_future_step_timer_early(
    sim_world,
):
    world = sim_world(_config, dual_mcu=False)
    world.gcode_ok(START)
    pid = world.mcus[0].process.pid
    stop = threading.Event()
    sent = [0]

    def storm():
        while not stop.wait(SIGNAL_PERIOD_REAL_S):
            libc.syscall(SYS_TGKILL, pid, pid, signal.SIGALRM)
            sent[0] += 1

    thread = threading.Thread(target=storm, daemon=True)
    thread.start()
    try:
        world.gcode_ok(MOVE, timeout=120)
        world.gcode_ok("M400", timeout=120)
    finally:
        stop.set()
        thread.join()
        time.sleep(0.05)
    assert sent[0] > 50, sent
    assert world.shutdown_line() is None, world.log_tail()
    assert world.toolhead_position()[0] == pytest.approx(175.0, abs=0.01)
