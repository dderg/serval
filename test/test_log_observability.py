import logging

import pytest

from klippy import structured_log
from klippy.extras import log_observability as lo


class CaptureHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture(autouse=True)
def _reset():
    structured_log.clear_print()
    structured_log.bind_session("k-test-1")
    yield
    structured_log.clear_session()
    structured_log.clear_print()


def test_heartbeat_emits_observability_event():
    cap = CaptureHandler()
    evlog = logging.getLogger("kalico.event")
    # In a bare test env the root logger defaults to WARNING, which would
    # filter the INFO heartbeat before it reaches the handler. klippy sets the
    # level at startup; here we lower it explicitly to observe the record.
    prev_level = evlog.level
    evlog.setLevel(logging.DEBUG)
    evlog.addHandler(cap)
    try:
        lo.emit_heartbeat()
    finally:
        evlog.removeHandler(cap)
        evlog.setLevel(prev_level)
    rec = next(
        r for r in cap.records if getattr(r, "event", None) == "heartbeat"
    )
    assert rec.subsystem == "observability"


def test_lag_within_threshold_is_ok():
    assert lo.check_lag(bytes_behind=1024, threshold=1_048_576) is False


def test_lag_over_threshold_is_flagged():
    assert lo.check_lag(bytes_behind=5_000_000, threshold=1_048_576) is True


def test_lag_at_threshold_is_not_stale():
    # boundary: exactly at threshold is not yet stale (strictly greater)
    assert lo.check_lag(bytes_behind=1_048_576, threshold=1_048_576) is False


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))


def _fake_proc(tmp_path, psi=True):
    (tmp_path / "meminfo").write_text(
        "MemTotal:        1998848 kB\n"
        "MemFree:           28672 kB\n"
        "MemAvailable:    1331200 kB\n"
        "SwapTotal:       4194304 kB\n"
        "SwapFree:        3738368 kB\n"
    )
    selfdir = tmp_path / "self"
    selfdir.mkdir()
    (selfdir / "status").write_text(
        "Name:\tklippy\nVmRSS:\t  78932 kB\nVmSwap:\t  1024 kB\n"
    )
    if psi:
        pressure = tmp_path / "pressure"
        pressure.mkdir()
        (pressure / "memory").write_text(
            "some avg10=1.50 avg60=0.80 avg300=0.20 total=1234\n"
            "full avg10=0.30 avg60=0.10 avg300=0.00 total=567\n"
        )
    return str(tmp_path)


def test_memory_snapshot_reads_fake_proc(tmp_path):
    fields = lo.host_memory_snapshot(_fake_proc(tmp_path))
    assert fields["mem_available_kb"] == 1331200
    assert fields["swap_used_kb"] == 4194304 - 3738368
    assert fields["own_rss_kb"] == 78932
    assert fields["own_swap_kb"] == 1024
    assert fields["psi_mem_some_avg10"] == 1.50
    assert fields["psi_mem_full_avg10"] == 0.30


def test_memory_snapshot_omits_missing_psi(tmp_path):
    fields = lo.host_memory_snapshot(_fake_proc(tmp_path, psi=False))
    assert "psi_mem_some_avg10" not in fields
    assert fields["mem_available_kb"] == 1331200


def test_memory_snapshot_of_absent_proc_is_empty(tmp_path):
    assert lo.host_memory_snapshot(str(tmp_path / "nope")) == {}


def test_swap_growth_detection():
    assert lo.swapped_out_since(None, {"own_swap_kb": 5}) is None
    assert lo.swapped_out_since(5, {"own_swap_kb": 5}) is None
    assert lo.swapped_out_since(5, {}) is None
    assert lo.swapped_out_since(5, {"own_swap_kb": 3}) is None
    assert lo.swapped_out_since(5, {"own_swap_kb": 40}) == 35


def test_heartbeat_carries_memory_fields():
    cap = CaptureHandler()
    evlog = logging.getLogger("kalico.event")
    prev_level = evlog.level
    evlog.setLevel(logging.DEBUG)
    evlog.addHandler(cap)
    try:
        lo.emit_heartbeat({"mem_available_kb": 12345, "own_swap_kb": 7})
    finally:
        evlog.removeHandler(cap)
        evlog.setLevel(prev_level)
    rec = next(
        r for r in cap.records if getattr(r, "event", None) == "heartbeat"
    )
    assert rec.mem_available_kb == 12345
    assert rec.own_swap_kb == 7
