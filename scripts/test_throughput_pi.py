from __future__ import annotations

import pytest
import throughput_pi
from throughput_gate import GateError

REPLAY_OUT = """\
moves=6767 segments=19421 stream_secs=26.918 wall_secs=6.5 overall_x=4.15
worst stream windows (t_lo..t_hi realtime_x):
      14.03..    19.02    3.65x
       0.01..     4.01    4.19x
"""


def test_a_replay_records_its_window_stream_and_wall(monkeypatch):
    monkeypatch.setattr(throughput_pi.recheck, "ssh", lambda _cmd: REPLAY_OUT)
    record = throughput_pi.replay("a.cfg", "0,1", 1)
    assert record["config"] == "a.cfg"
    assert record["cores"] == "0,1"
    assert record["run"] == 1
    assert record["worst_x"] == 3.65
    assert record["stream_secs"] == 26.918
    assert record["wall_secs"] == 6.5
    assert record["call_s"] >= 0.0


def test_a_replay_without_its_numbers_fails(monkeypatch):
    monkeypatch.setattr(throughput_pi.recheck, "ssh", lambda _cmd: "panicked")
    with pytest.raises(GateError, match="a.cfg on cores 0,1"):
        throughput_pi.replay("a.cfg", "0,1", 0)


def test_windows_are_each_configs_runs_on_its_cores():
    replays = [
        {"config": "a", "cores": "0,1", "worst_x": 2.1},
        {"config": "a", "cores": "0,1,2", "worst_x": 2.6},
        {"config": "a", "cores": "0,1", "worst_x": 2.0},
    ]
    assert throughput_pi.windows(replays, ["a"], "0,1") == {"a": [2.1, 2.0]}
    assert throughput_pi.windows(replays, ["a"], "0,1,2") == {"a": [2.6]}
