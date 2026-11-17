from __future__ import annotations

import json

import pytest
import throughput_gate as gate
import throughput_pi
from throughput_gate import GateError
from throughput_recheck import HoldKind

recheck = throughput_pi.recheck

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


TREES = {"rust": "r1", "tools/bench/configs": "c1", "tools/sim/gcode": "g1"}
SHA = "5" * 64
UNCAPPED = "policy0 ondemand 2400000 2400000\n"
SETTLED = ["uncapped policy0 from 1500000 to 2400000 kHz"]


class FakePi:
    def __init__(self, identity=None, cpufreq=UNCAPPED, throttled="0x0"):
        self.identity = identity
        self.sha = SHA
        self.cpufreq = cpufreq
        self.throttled = throttled
        self.events: list[str] = []

    def settle(self) -> list[str]:
        self.events.append("settle")
        return SETTLED

    def sleep(self, _secs: float) -> None:
        self.events.append("cooldown")

    def ssh(self, command: str) -> str:
        if command.startswith(f"cat {throughput_pi.IDENTITY}"):
            self.events.append("identity")
            return "" if self.identity is None else json.dumps(self.identity)
        if command.startswith(f"sha256sum {throughput_pi.BINARY}"):
            self.events.append("sha256")
            return f"{self.sha}  {throughput_pi.BINARY}\n"
        if command == "vcgencmd get_throttled":
            self.events.append("throttle")
            return f"throttled={self.throttled}\n"
        if command == recheck.CPUFREQ_STATE:
            self.events.append("governors")
            return self.cpufreq
        if "taskset" in command:
            self.events.append("replay")
            return REPLAY_OUT
        self.events.append(command)
        return ""


@pytest.fixture
def fake_pi(monkeypatch):
    def install(**state) -> FakePi:
        pi = FakePi(**state)
        monkeypatch.setattr(recheck, "ssh", pi.ssh)
        monkeypatch.setattr(recheck, "settle_bench", pi.settle)
        monkeypatch.setattr(throughput_pi.time, "sleep", pi.sleep)
        monkeypatch.setattr(gate, "measured_trees", lambda: TREES)
        monkeypatch.setattr(gate, "CONFIGS", ("a.cfg",))
        monkeypatch.setattr(gate, "FLOOR_SHORTFALLS", {})
        return pi

    return install


BUILT = {"trees": TREES, "sha256": SHA}


def test_the_pi_holds_a_binary_of_these_trees_only_with_its_sha256(fake_pi):
    pi = fake_pi()
    assert throughput_pi.built_on_pi(TREES) is False
    pi.identity = BUILT
    assert throughput_pi.built_on_pi(TREES) is True
    assert throughput_pi.built_on_pi({**TREES, "rust": "r2"}) is False
    pi.sha = "6" * 64
    assert throughput_pi.built_on_pi(TREES) is False


def test_a_measure_hold_settles_first_then_checks_and_brackets_the_replays(
    fake_pi, tmp_path
):
    pi = fake_pi(identity=BUILT)
    result = tmp_path / "hold.json"
    throughput_pi.measure_held([], result)
    assert pi.events == [
        "settle",
        "identity",
        "sha256",
        "cooldown",
        "throttle",
        "governors",
        "replay",
        "replay",
        "throttle",
    ]
    held = json.loads(result.read_text())
    assert held["restored"] == SETTLED
    assert held["governors"]["policy0"]["scaling_max_freq"] == 2400000
    assert held["cooldown_s"] == recheck.PI_COOLDOWN_SECS
    assert [r["worst_x"] for r in held["replays"]] == [3.65, 3.65]


def test_a_measure_hold_refuses_a_capped_policy(fake_pi, tmp_path):
    pi = fake_pi(identity=BUILT, cpufreq="policy0 ondemand 1500000 2400000\n")
    with pytest.raises(GateError, match="capped"):
        throughput_pi.measure_held([], tmp_path / "hold.json")
    assert "replay" not in pi.events


def test_a_measure_hold_refuses_a_throttled_pi(fake_pi, tmp_path):
    pi = fake_pi(identity=BUILT, throttled="0x2")
    with pytest.raises(GateError, match="throttling before the runs"):
        throughput_pi.measure_held([], tmp_path / "hold.json")
    assert "replay" not in pi.events


def test_a_measure_hold_on_another_runs_binary_reports_it_stale(
    fake_pi, tmp_path
):
    pi = fake_pi(identity={**BUILT, "sha256": "7" * 64})
    result = tmp_path / "hold.json"
    throughput_pi.measure_held([], result)
    assert pi.events == ["settle", "identity", "sha256"]
    assert json.loads(result.read_text())["stale"] is True


def test_a_build_hold_drops_the_identity_first_and_writes_it_last(
    fake_pi, monkeypatch, tmp_path
):
    pi = fake_pi()
    monkeypatch.setattr(
        gate, "run", lambda command, cwd=None: pi.events.append(command[0])
    )
    result = tmp_path / "hold.json"
    throughput_pi.build_held(result)
    assert pi.events[:2] == ["settle", f"rm -f {throughput_pi.IDENTITY}"]
    assert "cargo build" in pi.events[-3]
    assert pi.events[-2] == "sha256"
    assert pi.events[-1].endswith(f"> {throughput_pi.IDENTITY}")
    assert json.dumps(BUILT) in pi.events[-1]
    held = json.loads(result.read_text())
    assert held["sha256"] == SHA
    assert held["restored"] == SETTLED


def hold_record(kind: HoldKind, **facts) -> dict:
    record = {
        "kind": kind.value,
        "wait_s": 12.0,
        "held_s": 30.0,
        "exit": 0,
        "started": 1.0,
        "trees": TREES,
        "restored": [],
    }
    if kind is HoldKind.BUILD:
        return {**record, "sync_s": 1.5, "build_s": 1.2, **facts}
    replays = [
        {"config": "a.cfg", "cores": gate.LIVE_CORES, "worst_x": x, "call_s": 9}
        for x in (2.5, 2.4)
    ]
    return {
        **record,
        "cooldown_s": 60,
        "measured_at": "T",
        "replays": replays,
        **facts,
    }


@pytest.fixture
def holds(monkeypatch, tmp_path):
    def install(built: list[bool], measures: list[dict]) -> list[str]:
        taken: list[str] = []

        def take_hold(kind: HoldKind, command: list[str]) -> dict:
            taken.append(kind.value)
            if kind is HoldKind.BUILD:
                return hold_record(kind)
            return measures.pop(0)

        monkeypatch.delenv(recheck.lock.TOKEN_ENV, raising=False)
        monkeypatch.setattr(gate, "measured_trees", lambda: TREES)
        monkeypatch.setattr(gate, "CONFIGS", ("a.cfg",))
        monkeypatch.setattr(gate, "FLOOR_SHORTFALLS", {})
        monkeypatch.setattr(gate, "ROOT", tmp_path)
        monkeypatch.setattr(gate, "PI_WINDOW", tmp_path / "pi_window.json")
        monkeypatch.setattr(
            throughput_pi, "built_on_pi", lambda _: built.pop(0)
        )
        monkeypatch.setattr(recheck, "take_hold", take_hold)
        return taken

    return install


def test_the_build_hold_is_skipped_when_the_pi_holds_these_trees(holds):
    taken = holds([True], [hold_record(HoldKind.MEASURE)])
    throughput_pi.measure([])
    assert taken == ["measure"]
    window = json.loads(gate.PI_WINDOW.read_text())
    assert window["worst_x"] == {"a.cfg": [2.5, 2.4]}
    assert window["trees"] == TREES
    [measured] = window["timing"]["holds"]
    assert measured["index"] == 1
    assert (measured["wait_s"], measured["held_s"]) == (12.0, 30.0)
    assert "measured_at" not in measured
    assert "trees" not in measured


def test_a_binary_of_other_trees_is_built_in_its_own_hold_first(holds):
    taken = holds([False], [hold_record(HoldKind.MEASURE)])
    throughput_pi.measure([])
    assert taken == ["build", "measure"]
    timing = json.loads(gate.PI_WINDOW.read_text())["timing"]
    assert [(h["index"], h["kind"]) for h in timing["holds"]] == [
        (1, "build"),
        (2, "measure"),
    ]


def test_a_binary_changed_under_the_measure_hold_is_rebuilt_and_remeasured(
    holds,
):
    stale = hold_record(HoldKind.MEASURE, stale=True)
    taken = holds([True, False], [stale, hold_record(HoldKind.MEASURE)])
    throughput_pi.measure([])
    assert taken == ["measure", "build", "measure"]


def test_a_binary_rebuilt_under_every_measure_hold_fails(holds):
    stale = [hold_record(HoldKind.MEASURE, stale=True) for _ in range(2)]
    holds([True, True], stale)
    with pytest.raises(GateError, match="another throughput_pi.py"):
        throughput_pi.measure([])


def test_throughput_pi_refuses_to_run_inside_a_hold(monkeypatch):
    monkeypatch.setenv(recheck.lock.TOKEN_ENV, "1")
    with pytest.raises(GateError, match="outside lock.py hold"):
        throughput_pi.measure([])
