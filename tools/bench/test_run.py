from __future__ import annotations

import json
from types import SimpleNamespace

import pytest
import run


class _Capture:
    def __init__(self, run_dir, both_edges_mask, events):
        pass

    def start(self):
        pass

    def stop(self):
        return SimpleNamespace(unit="samples", units=0)


@pytest.fixture
def bench(monkeypatch):
    fetched = []
    monkeypatch.setattr(run, "Capture", _Capture)
    monkeypatch.setattr(run.time, "sleep", lambda _s: None)
    monkeypatch.setattr(run, "http", lambda *_a: None)
    monkeypatch.setattr(
        run, "scp_from_pi", lambda src, *_a, **_k: fetched.append(src)
    )
    return fetched


def test_a_failing_cancel_does_not_hide_why_the_print_failed(
    tmp_path, monkeypatch, bench
):
    def print_fails(_capture, _timeout):
        raise run.BenchError("capture stalled after 10 s")

    def cancel_fails():
        raise run.BenchError("webhooks: No registered callback")

    monkeypatch.setattr(run, "poll_print", print_fails)
    monkeypatch.setattr(run, "cancel_active_print", cancel_fails)
    with pytest.raises(run.BenchError, match="capture stalled") as raised:
        run.execute(tmp_path, "x.gcode", {}, 1.0, False, False)
    assert any("No registered callback" in n for n in raised.value.__notes__)
    assert any(src.endswith("klippy.log") for src in bench)
    assert (
        json.loads((tmp_path / "run.json").read_text())["print_state"] is None
    )


F401_STEP_BENCHMARK = {
    "mcu": "mcu",
    "clock_freq": 84000000.0,
    "pulsed_ticks_per_step": 301,
    "both_edge_ticks_per_step": 160,
    "simulated": False,
    "motors": [{"name": "a", "step_distance": 0.0125, "mode": "pulsed"}],
}


def test_every_mcus_step_benchmark_is_recorded_as_klippy_reports_it(
    monkeypatch,
):
    requests = []

    def http(method, path):
        requests.append(path)
        if path == "/printer/objects/list":
            return {"objects": ["mcu", "mcu toolboard", "print_stats"]}
        return {
            "status": {
                "mcu": {"step_capacity": F401_STEP_BENCHMARK},
                "mcu toolboard": {"mcu_version": "v1"},
            }
        }

    monkeypatch.setattr(run, "http", http)
    assert run.mcu_step_benchmarks() == [F401_STEP_BENCHMARK]
    assert requests[-1] == "/printer/objects/query?mcu&mcu%20toolboard"


def test_a_klippy_without_mcus_is_refused(monkeypatch):
    monkeypatch.setattr(run, "http", lambda *_a: {"objects": ["print_stats"]})
    with pytest.raises(run.BenchError, match="no mcu objects"):
        run.mcu_step_benchmarks()
