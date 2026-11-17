from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path

import pytest
import throughput_gate as gate
import throughput_recheck as recheck
from throughput_recheck import HoldKind, lock


def test_a_fit_through_the_anchor_recovers_the_factor_and_its_window():
    xs = {"a": -0.1, "b": 0.05, "c": 0.1, "d": 0.2}
    ys = {k: math.log(2.0) - 3.0 * x for k, x in xs.items()}
    fitted = recheck.fit(xs, ys, anchor="b")
    assert fitted["factor"] == pytest.approx(3.0)
    assert fitted["intercept"] == pytest.approx(math.log(2.0))
    assert set(fitted["residual_pct"]) == {"a", "c", "d"}
    assert fitted["max_residual_pct"] == pytest.approx(0.0, abs=1e-9)
    assert fitted["max_loo_residual_pct"] == pytest.approx(0.0, abs=1e-9)


def test_residuals_are_the_gates_error_predicting_each_point_from_the_anchor():
    xs = {"ref": 0.0, "b": 1.0, "c": 2.0}
    ys = {"ref": 0.0, "b": -1.0, "c": -1.0}
    fitted = recheck.fit(xs, ys, anchor="ref")
    assert fitted["factor"] == pytest.approx(0.6)
    assert fitted["residual_pct"]["b"] == pytest.approx(
        100.0 * (math.exp(-0.6 + 1.0) - 1.0), abs=0.01
    )
    assert fitted["loo_residual_pct"]["b"] == pytest.approx(
        100.0 * (math.exp(-0.5 + 1.0) - 1.0), abs=0.01
    )
    assert fitted["loo_residual_pct"]["c"] == pytest.approx(
        100.0 * (math.exp(-2.0 + 1.0) - 1.0), abs=0.01
    )


def test_a_fit_wants_two_points_besides_the_anchor():
    with pytest.raises(gate.GateError):
        recheck.fit(
            {"ref": 0.0, "b": 1.0}, {"ref": 0.0, "b": 1.0}, anchor="ref"
        )


CAL = recheck.CALIBRATION_LABEL


def rows(job: str, cpu: str, costs: dict[str, list[float]]) -> list[dict]:
    return [
        {
            "job": job,
            "cpu": cpu,
            "label": label,
            "config": "c.cfg",
            "cost": cost,
        }
        for label, samples in costs.items()
        for cost in samples
    ]


def test_live_costs_divide_by_the_jobs_calibration_and_average_one_cpu():
    measured = [
        *rows("1/x", "Slow", {CAL: [2.0, 2.0, 9.0], "b": [4.0, 4.0]}),
        *rows("1/y", "Slow", {CAL: [1.0, 1.1], "b": [2.0, 2.2]}),
        *rows("2/x", "Fast", {CAL: [1.0, 1.0], "b": [1.0, 1.0]}),
    ]
    live = recheck.live_costs(measured, [CAL, "b"], ["c.cfg"])["c.cfg"]
    assert live["cpus"]["Slow"]["jobs"] == 2
    assert live["cpus"]["Slow"]["x"]["b"] == pytest.approx(math.log(2.0))
    assert live["cpus"]["Slow"]["x"][CAL] == 0.0
    assert live["cpus"]["Fast"]["x"] == {CAL: 0.0, "b": 0.0}
    assert sorted(live["calibration_costs"]) == pytest.approx([1.0, 1.05, 2.0])
    assert max(live["spreads_pct"]) == pytest.approx(10.0)


def test_a_job_counts_for_the_variants_it_ran_beside_the_calibration():
    measured = [
        *rows("1/x", "Slow", {CAL: [2.0, 2.0], "b": [4.0], "c": [3.0, 3.0]}),
        *rows("1/y", "Slow", {CAL: [1.0, 1.0], "b": [3.0, 3.0]}),
        *rows("1/z", "Slow", {CAL: [1.0], "b": [3.0, 3.0]}),
    ]
    live = recheck.live_costs(measured, [CAL, "b", "c"], ["c.cfg"])["c.cfg"]
    assert live["cpus"]["Slow"]["jobs"] == 2
    assert live["cpus"]["Slow"]["x"]["b"] == pytest.approx(math.log(3.0))
    assert live["cpus"]["Slow"]["x"]["c"] == pytest.approx(math.log(1.5))


def test_one_live_factor_fits_every_cpu_model():
    ys = {
        CAL: math.log(2.0),
        "a": math.log(2.0) - 0.2,
        "b": math.log(2.0) + 0.1,
    }
    xs = {
        "Slow": {CAL: 0.0, "a": 0.1, "b": -0.05},
        "Fast": {CAL: 0.0, "a": 0.2, "b": -0.2},
    }
    fitted = recheck.pooled_fit(xs, ys, {"Slow": 3, "Fast": 1})
    factor = 0.085 / 0.0925
    without_slow = 0.06 / 0.08
    without_a = 0.025 / 0.0425
    assert fitted["factor"] == pytest.approx(factor)
    assert fitted["intercept"] == pytest.approx(math.log(2.0))
    slow = fitted["by_cpu"]["Slow"]
    assert slow["jobs"] == 3
    for name, slope in (
        ("residual_pct", factor),
        ("cpu_out_residual_pct", without_slow),
        ("loo_residual_pct", without_a),
    ):
        assert slow[name]["a"] == pytest.approx(
            100.0 * (math.exp(-slope * 0.1 + 0.2) - 1.0), abs=0.01
        )
    assert fitted["max_cpu_out_residual_pct"] == max(
        cpu["max_cpu_out_residual_pct"] for cpu in fitted["by_cpu"].values()
    )


def test_the_live_fit_wants_two_cpu_models_and_two_variants():
    ys = {CAL: 0.0, "a": -0.1, "b": 0.1}
    with pytest.raises(gate.GateError, match="CPU models"):
        recheck.pooled_fit({"Only": {"a": 0.1, "b": -0.1}}, ys, {"Only": 1})
    with pytest.raises(gate.GateError, match="variants"):
        recheck.pooled_fit(
            {"A": {"a": 0.1}, "B": {"a": 0.2}}, ys, {"A": 1, "B": 1}
        )


def test_the_probe_budget_is_each_ci_jobs_timeout():
    workflow = (
        "jobs:\n"
        "  rust-test:\n"
        "    timeout-minutes: 20\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh rust-test\n"
        "  no-timeout:\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh other\n"
        "  rust-fmt:\n"
        "    timeout-minutes: 5\n"
        "    steps:\n"
        "      - run: ./scripts/ci.sh rust-fmt\n"
    )
    assert recheck.job_timeouts(workflow) == {"rust-test": 20, "rust-fmt": 5}


def test_probe_rows_come_from_the_run_log_keyed_by_run_job_and_variant():
    row = {
        "cpu": "X",
        "label": "a",
        "config": "c.cfg",
        "cost": 1.5,
        "job": "rust-test",
    }
    log = (
        "rust-test\tProbe\t2026-11-04T10:00:00Z noise\n"
        f"rust-test\tProbe\t2026-11-04T10:00:01Z RECHECK {json.dumps(row)}\n"
    )
    assert recheck.probe_rows(log, 7, {"a": "sha-a"}) == [
        {**row, "job": "7/rust-test", "key": "sha-a"}
    ]


def test_collected_rows_follow_their_variant_key_and_drop_unlisted_ones(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(recheck, "WORK", tmp_path)
    recheck.save(
        "collect",
        {
            "rows": [
                {"job": "1/x", "label": "reference", "key": "sha-cal"},
                {"job": "1/x", "label": "old", "key": "sha-old"},
                {"job": "1/x", "label": "a", "key": "sha-a"},
            ]
        },
    )
    assert recheck.collected_rows({"a": "sha-a", CAL: "sha-cal"}) == [
        {"job": "1/x", "label": CAL, "key": "sha-cal"},
        {"job": "1/x", "label": "a", "key": "sha-a"},
    ]


def test_variants_add_the_calibration_commit_and_refuse_clashing_labels():
    baseline = {
        "pi_calibration": {"variants": [{"label": "a", "commit": "x"}]},
    }
    assert recheck.variants(baseline, "cal") == [
        {"label": "a", "commit": "x"},
        {"label": CAL, "commit": "cal"},
    ]
    clash = {"pi_calibration": {"variants": [{"label": CAL, "commit": "x"}]}}
    with pytest.raises(gate.GateError):
        recheck.variants(clash, "cal")
    with pytest.raises(gate.GateError):
        recheck.variants({}, "cal")
    itself = {"pi_calibration": {"variants": [{"label": "a", "commit": "cal"}]}}
    with pytest.raises(gate.GateError, match="calibration commit"):
        recheck.variants(itself, "cal")
    patched = {
        "pi_calibration": {
            "variants": [{"label": "a", "commit": "cal", "patch": "p.patch"}]
        }
    }
    assert recheck.variants(patched, "cal")[0]["label"] == "a"


def test_the_calibration_commit_is_the_reference_unless_re_pinned(monkeypatch):
    resolved = []

    def rev_parse(command: list[str], cwd=None) -> str:
        resolved.append(command[-1])
        return "sha\n"

    monkeypatch.setattr(gate, "run", rev_parse)
    assert recheck.calibration_commit({"commit": "ref"}, None) == "sha"
    assert recheck.calibration_commit({"commit": "ref"}, "pin") == "sha"
    assert resolved == ["ref^{commit}", "pin^{commit}"]
    with pytest.raises(gate.GateError, match="--calibration-commit"):
        recheck.calibration_commit({}, None)


def test_the_pi_wall_comes_from_the_stamps_around_the_replay():
    out = "pi_started_ns=1000000000\nmoves=1\npi_ended_ns=3500000000\n"
    assert recheck.pi_wall_secs(out) == pytest.approx(2.5)
    with pytest.raises(gate.GateError):
        recheck.pi_wall_secs("pi_started_ns=1\n")


def test_the_ci_probe_hook_replaces_the_rust_line_once():
    assert recheck.CI_PROBE_HOOK.startswith('RUST="$ROOT/rust"\n')
    assert '--job "${1:-}"' in recheck.CI_PROBE_HOOK


REPLAY_OUT = """\
pi_started_ns=1000000000
moves=6767 segments=19421 stream_secs=20.0 wall_secs=6.5 overall_x=4.15
worst stream windows (t_lo..t_hi realtime_x):
      14.03..    19.02    3.65x
pi_ended_ns=6000000000
"""
UNCAPPED = "policy0 ondemand 2400000 2400000\n"
CAPPED = "policy0 ondemand 1500000 2400000\n"
UNTHROTTLED = "throttled=0x0\n"
SETTLED = ["removed drop-in /etc/systemd/system/klipper.service.d/x.conf"]


class FakePi:
    def __init__(self, keys, cpufreq=UNCAPPED, throttled=UNTHROTTLED):
        self.keys = dict(keys)
        self.cpufreq = cpufreq
        self.throttled = throttled
        self.events: list = []

    def settle(self) -> list[str]:
        self.events.append("settle")
        return SETTLED

    def sleep(self, _secs: float) -> None:
        self.events.append("cooldown")

    def ssh(self, command: str) -> str:
        if command.startswith(f"cd {recheck.PI_DIR}/bins"):
            self.events.append("keys")
            return "".join(
                f"{label}.key:{key}\n" for label, key in self.keys.items()
            )
        if command == "vcgencmd get_throttled":
            self.events.append("throttle")
            return self.throttled
        if command == recheck.CPUFREQ_STATE:
            self.events.append("governors")
            return self.cpufreq
        words = command.split()
        if "taskset" in words:
            label = next(w for w in words if w.startswith("bins/"))[5:]
            config = words[words.index("--config") + 1]
            config = config.removeprefix(f"trees/{label}/")
            self.events.append(("replay", label, config))
            return REPLAY_OUT
        self.events.append(command)
        return ""


@pytest.fixture
def fake_pi(monkeypatch):
    def install(keys, **state) -> FakePi:
        pi = FakePi(keys, **state)
        monkeypatch.setattr(recheck, "ssh", pi.ssh)
        monkeypatch.setattr(recheck, "settle_bench", pi.settle)
        monkeypatch.setattr(recheck.time, "sleep", pi.sleep)
        monkeypatch.setattr(gate, "CONFIGS", ("a.cfg", "b.cfg"))
        return pi

    return install


KEYS = {"a": "sha-a", CAL: "sha-cal"}


def test_a_measure_hold_settles_verifies_cools_then_alternates(
    fake_pi, tmp_path
):
    pi = fake_pi(KEYS)
    result = tmp_path / "hold.json"
    recheck.pi_measure_held(KEYS, 1, result)
    assert pi.events == [
        "settle",
        "keys",
        "cooldown",
        "throttle",
        "governors",
        ("replay", CAL, "a.cfg"),
        ("replay", "a", "a.cfg"),
        ("replay", CAL, "b.cfg"),
        ("replay", "a", "b.cfg"),
        "throttle",
    ]
    held = json.loads(result.read_text())
    assert held["restored"] == SETTLED
    assert held["governors"] == {
        "policy0": {
            "scaling_governor": "ondemand",
            "scaling_max_freq": 2400000,
            "cpuinfo_max_freq": 2400000,
        }
    }
    assert held["measured"]["worst_x"]["a"] == {"a.cfg": 3.65, "b.cfg": 3.65}
    assert held["measured"]["cost"][CAL]["b.cfg"] == pytest.approx(0.25)


def test_a_measure_hold_refuses_a_capped_policy_before_any_replay(
    fake_pi, tmp_path
):
    pi = fake_pi(KEYS, cpufreq=CAPPED)
    with pytest.raises(gate.GateError, match="policy0.*capped"):
        recheck.pi_measure_held(KEYS, 0, tmp_path / "hold.json")
    assert not [event for event in pi.events if event[0] == "replay"]


def test_a_measure_hold_refuses_a_throttled_pi_before_any_replay(
    fake_pi, tmp_path
):
    pi = fake_pi(KEYS, throttled="throttled=0x4\n")
    with pytest.raises(gate.GateError, match="throttling before the runs"):
        recheck.pi_measure_held(KEYS, 0, tmp_path / "hold.json")
    assert pi.events == ["settle", "keys", "cooldown", "throttle"]


def test_a_measure_hold_reports_a_variant_rebuilt_under_it_unmeasured(
    fake_pi, tmp_path
):
    pi = fake_pi({**KEYS, "a": "someone-elses"})
    result = tmp_path / "hold.json"
    recheck.pi_measure_held(KEYS, 0, result)
    assert pi.events == ["settle", "keys"]
    assert json.loads(result.read_text())["stale"] == ["a"]


def test_a_build_hold_drops_the_key_first_and_writes_it_last(
    fake_pi, monkeypatch, tmp_path
):
    pi = fake_pi({})
    monkeypatch.setattr(
        gate, "run", lambda command, cwd=None: pi.events.append(command[0])
    )
    result = tmp_path / "hold.json"
    recheck.pi_build_held("a", "sha-a", result)
    settle, drop, sync_rust, sync_tools, build, write = pi.events
    assert settle == "settle"
    assert drop.endswith(f"rm -f {recheck.PI_DIR}/bins/a.key")
    assert (sync_rust, sync_tools) == ("rsync", "rsync")
    assert "cargo build" in build and build.endswith("../../bins/a")
    assert write == f"printf '%s\\n' sha-a > {recheck.PI_DIR}/bins/a.key"
    held = json.loads(result.read_text())
    assert held["label"] == "a"
    assert held["restored"] == SETTLED


def test_a_hold_records_its_lock_wait_and_held_seconds(monkeypatch):
    commands = []

    def lock_hold(command, cwd, check):
        commands.append(command)
        result = Path(command[command.index("--result") + 1])
        result.write_text(json.dumps({"started": time.time(), "label": "a"}))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(recheck.subprocess, "run", lock_hold)
    record = recheck.take_hold(HoldKind.BUILD, ["held", "--label", "a"])
    assert commands[0][3:5] == [str(recheck.BENCH / "lock.py"), "hold"]
    assert commands[0][8:12] == ["held", "--label", "a", "--result"]
    assert record["kind"] == "build"
    assert record["exit"] == 0
    assert record["label"] == "a"
    assert record["wait_s"] >= 0.0
    assert record["held_s"] >= 0.0


def test_a_hold_killed_at_the_cap_fails_naming_the_resume(monkeypatch):
    monkeypatch.setattr(
        recheck.subprocess,
        "run",
        lambda command, cwd, check: subprocess.CompletedProcess(
            command, lock.HOLD_EXPIRED_EXIT
        ),
    )
    with pytest.raises(gate.GateError, match="resumes with that hold"):
        recheck.take_hold(HoldKind.MEASURE, ["held"])


def test_every_pi_hold_fits_the_cap_and_an_oversized_one_is_refused():
    replays = 13 * len(gate.CONFIGS)
    recheck.require_fits(HoldKind.BUILD, recheck.build_plan_s())
    recheck.require_fits(HoldKind.MEASURE, recheck.measure_plan_s(replays))
    with pytest.raises(gate.GateError, match="one hold may plan"):
        recheck.require_fits(HoldKind.MEASURE, recheck.measure_plan_s(100))


def test_the_pi_stage_refuses_to_run_inside_a_hold(monkeypatch):
    monkeypatch.setenv(lock.TOKEN_ENV, "1")
    with pytest.raises(gate.GateError, match="outside lock.py hold"):
        recheck.pi_stage(KEYS)


def measured(worst: float, started: float) -> dict:
    return {
        "kind": "measure",
        "wait_s": 0.5,
        "held_s": 60.0,
        "exit": 0,
        "started": started,
        "measured": {
            "measured_at": f"T{started:.0f}",
            "worst_x": {"a": {"a.cfg": worst}, CAL: {"a.cfg": worst + 1}},
            "cost": {"a": {"a.cfg": 0.3}, CAL: {"a.cfg": 0.2}},
        },
    }


class FakeHolds:
    def __init__(self, on_pi: dict[str, str]):
        self.on_pi = on_pi
        self.taken: list[tuple[str, str]] = []
        self.replies: list[dict] = []

    def take(self, kind: HoldKind, command: list[str]) -> dict:
        if kind is HoldKind.BUILD:
            label = command[command.index("--label") + 1]
            self.taken.append(("build", label))
            self.on_pi[label] = KEYS[label]
            return {"kind": "build", "started": 5.0, "label": label}
        self.taken.append(("measure", command[command.index("--run") + 1]))
        return self.replies.pop(0)


@pytest.fixture
def holds(monkeypatch, tmp_path):
    def install(on_pi: dict[str, str]) -> FakeHolds:
        fake = FakeHolds(dict(on_pi))
        monkeypatch.setattr(recheck, "WORK", tmp_path)
        monkeypatch.setattr(gate, "CONFIGS", ("a.cfg",))
        monkeypatch.delenv(lock.TOKEN_ENV, raising=False)
        monkeypatch.setattr(recheck, "pi_keys", lambda: dict(fake.on_pi))
        monkeypatch.setattr(recheck, "take_hold", fake.take)
        return fake

    return install


def test_a_rerun_measures_only_missing_runs_and_builds_only_stale_variants(
    holds,
):
    fake = holds({"a": "sha-a", CAL: "older"})
    run0 = measured(2.0, 1.0)
    recheck.save(
        "pi-run-0",
        {"variants": KEYS, **run0.pop("measured"), "holds": [run0]},
    )
    fake.replies = [measured(2.1, 9.0)]
    recheck.pi_stage(KEYS)
    assert fake.taken == [("build", CAL), ("measure", "1")]
    pi = recheck.load("pi", KEYS)
    assert pi["measured_at"] == "T1"
    assert pi["worst_x"] == {
        "a": {"a.cfg": [2.0, 2.1]},
        CAL: {"a.cfg": [3.0, 3.1]},
    }
    assert [(h["index"], h["kind"]) for h in pi["holds"]] == [
        (1, "measure"),
        (2, "build"),
        (3, "measure"),
    ]
    assert not recheck.data_path("pi-run-0").exists()
    assert not recheck.data_path("pi-builds").exists()


def test_a_run_saved_for_other_variants_is_measured_again(holds):
    fake = holds(KEYS)
    old = measured(1.0, 1.0)
    recheck.save(
        "pi-run-0",
        {"variants": {**KEYS, "a": "x"}, **old.pop("measured"), "holds": []},
    )
    fake.replies = [measured(2.0, 2.0), measured(2.1, 3.0)]
    recheck.pi_stage(KEYS)
    assert fake.taken == [("measure", "0"), ("measure", "1")]


def test_a_variant_rebuilt_under_the_measure_hold_is_rebuilt_and_remeasured(
    holds, monkeypatch
):
    monkeypatch.setattr(recheck, "PI_RUNS", 1)
    fake = holds(KEYS)
    stale = {"kind": "measure", "started": 2.0, "stale": ["a"]}

    def rebuilt_by_another_run(kind, command):
        reply = FakeHolds.take(fake, kind, command)
        if reply is stale:
            fake.on_pi["a"] = "another"
        return reply

    monkeypatch.setattr(recheck, "take_hold", rebuilt_by_another_run)
    fake.replies = [stale, measured(2.0, 3.0)]
    recheck.pi_stage(KEYS)
    assert fake.taken == [("measure", "0"), ("build", "a"), ("measure", "0")]
    assert len(recheck.load("pi", KEYS)["holds"]) == 3
