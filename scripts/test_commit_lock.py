import threading
import time

import pending_baselines as pending

HOLD_SECONDS = 0.4


def test_two_users_of_the_same_name_run_one_after_the_other(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(pending, "LOCKS", tmp_path / "locks")
    spans = []

    def user():
        with pending.commit_lock("wt-abc"):
            start = time.monotonic()
            time.sleep(HOLD_SECONDS)
            spans.append((start, time.monotonic()))

    threads = [threading.Thread(target=user) for _ in range(3)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    spans.sort()
    for earlier, later in zip(spans, spans[1:]):
        assert later[0] >= earlier[1] - 0.01, "two holders overlapped"


def test_different_names_do_not_block_each_other(tmp_path, monkeypatch):
    monkeypatch.setattr(pending, "LOCKS", tmp_path / "locks")
    inside = threading.Event()
    release = threading.Event()

    def holder():
        with pending.commit_lock("wt-one"):
            inside.set()
            release.wait(5)

    thread = threading.Thread(target=holder)
    thread.start()
    assert inside.wait(5)
    with pending.commit_lock("wt-two"):
        pass
    release.set()
    thread.join()
