import threading

import pytest

from tools.sim.emulators.restartable_thread import RestartableThread

pytestmark = pytest.mark.sim_unit

WAIT_S = 5.0


class Worker:
    """A loop that runs while `wanted`, then spends time in cleanup that the
    test holds open, the way the beacon stub's thread closes its step socket
    and releases its pacer."""

    def __init__(self):
        self.wanted = True
        self.runs_started = 0
        self.runs_streaming = threading.Semaphore(0)
        self.left_loop = threading.Event()
        self.finish_cleanup = threading.Event()
        self.runner = RestartableThread(self.body, "worker")

    def body(self):
        self.runs_started += 1
        first = self.runs_started == 1
        while self.runner.keep_running(self.wanted):
            self.runs_streaming.release()
            threading.Event().wait(0.001)
        self.left_loop.set()
        if first:
            self.finish_cleanup.wait(WAIT_S)


def test_a_restart_during_the_old_threads_cleanup_starts_a_new_thread():
    worker = Worker()
    worker.runner.ensure_running()
    assert worker.runs_streaming.acquire(timeout=WAIT_S)

    worker.wanted = False
    assert worker.left_loop.wait(WAIT_S), "the loop never noticed the stop"
    worker.wanted = True
    worker.runner.ensure_running()

    for _ in range(3):
        assert worker.runs_streaming.acquire(timeout=WAIT_S), (
            "nothing streams: the restart found the dying thread and "
            "started no new one"
        )
    assert worker.runs_started == 2
    worker.finish_cleanup.set()


def test_ensure_running_while_running_does_not_start_a_second_thread():
    worker = Worker()
    worker.runner.ensure_running()
    assert worker.runs_streaming.acquire(timeout=WAIT_S)
    worker.runner.ensure_running()
    worker.runner.ensure_running()
    assert worker.runs_started == 1
    worker.wanted = False
    worker.finish_cleanup.set()
    worker.runner.join(WAIT_S)


def test_a_stopped_runner_can_be_started_again_after_it_left_its_loop():
    worker = Worker()
    worker.finish_cleanup.set()
    worker.runner.ensure_running()
    assert worker.runs_streaming.acquire(timeout=WAIT_S)
    worker.wanted = False
    assert worker.left_loop.wait(WAIT_S)
    worker.wanted = True
    worker.runner.ensure_running()
    assert worker.runs_streaming.acquire(timeout=WAIT_S)
    assert worker.runs_started == 2
