from __future__ import annotations

import argparse
import contextlib
import datetime
import json
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import qualify

REV = "0123456789abcdef0123456789abcdef01234567"
TARGET = "e2fa311458d0b25df462bb9c9807540466538ced"
IDENTITY = {
    "serial": "10000000abcdef01",
    "kernel": "6.12.25+rpt-rpi-2712",
    "capture_build_id": "a0e9c9d157567fa14a4ca70159dc332a62e7c683",
}
T0 = datetime.datetime(2026, 11, 16, 20, 0, tzinfo=datetime.timezone.utc)
UNIT_S = 600
WAIT_S = 30.0
SUMMARY = {
    "rev": REV,
    "expect_fail": [{"row": "pause_sparse", "issue": 860}],
    "units": [{"unit": "deploy", "seconds": 61, "passed": True, "holds": []}],
}


def git(cwd: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def commit(root: Path, files: dict[str, str], message: str) -> None:
    for name, text in files.items():
        (root / name).parent.mkdir(parents=True, exist_ok=True)
        (root / name).write_text(text)
    git(root, "add", *files)
    git(root, "commit", "-qm", message)


def merge(root: Path, branch: str, files: dict[str, str], pr: int) -> None:
    git(root, "switch", "-qc", branch)
    commit(root, files, f"Work on {branch}")
    git(root, "switch", "-q", "main")
    message = f"Merge pull request #{pr} from me/{branch}"
    git(root, "merge", "-q", "--no-ff", "-m", message, branch)


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    git(tmp_path, "init", "-q", "-b", "main", str(root))
    git(root, "config", "user.email", "t@t")
    git(root, "config", "user.name", "t")
    commit(root, {"README": "base"}, "Base")
    return root


class Clock:
    def __init__(self, now: datetime.datetime):
        self.now = now

    def __call__(self) -> datetime.datetime:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += datetime.timedelta(seconds=seconds)


@pytest.fixture
def clock(monkeypatch) -> Clock:
    synthetic = Clock(T0)
    monkeypatch.setattr(qualify, "utc_now", synthetic)
    return synthetic


class Bench:
    def __init__(self, clock: Clock):
        self.clock = clock
        self.ran: list[str] = []
        self.failing: set[str] = set()

    def unit(self, name: str, rows: tuple[str, ...] = ()) -> qualify.Unit:
        return qualify.Unit(name, self.run, rows)

    def run(self, _context, unit: qualify.Unit) -> qualify.Outcome:
        self.ran.append(unit.name)
        self.clock.advance(UNIT_S)
        passed = unit.name not in self.failing
        taken = {
            "index": 1,
            "rows": list(unit.rows),
            "wait_s": WAIT_S,
            "held_s": float(UNIT_S),
        }
        return qualify.Outcome(
            passed,
            {"failed_rows": [] if passed else list(unit.rows)},
            () if passed else ("broke",),
            (taken,),
        )

    def units(self) -> list[qualify.Unit]:
        return [
            self.unit("deploy"),
            self.unit("suite-1", ("first_layers", "pause_sparse")),
            self.unit("suite-2", ("estop",)),
            self.unit("pi"),
        ]


@pytest.fixture
def bench(clock: Clock) -> Bench:
    return Bench(clock)


def context(tmp_path: Path, **changes) -> qualify.Context:
    fields = {
        "rev": REV,
        "tree": tmp_path,
        "record": tmp_path / "record",
        "expected": {"pause_sparse": 860},
        "target_id": TARGET,
        "identity": IDENTITY,
    }
    return qualify.Context(**{**fields, **changes})


def test_the_units_are_deploy_planned_suite_holds_soak_rows_pi_calibration():
    rows = [SimpleNamespace(name=name) for name in ("a", "b", "c")]
    soak = [SimpleNamespace(name=n) for n in ("soak_mesh", "soak_throttled")]
    suite = SimpleNamespace(ROWS=rows, SOAK_ROWS=soak)
    asked = []

    def plan(names: list[str]) -> list[list[str]]:
        asked.append(names)
        return [["a", "b"], ["c"]]

    units = qualify.units_of(suite, plan)
    assert asked == [["a", "b", "c"]]
    assert [(unit.name, unit.rows, unit.detail) for unit in units] == [
        ("deploy", (), qualify.Detail.TALLIES),
        ("suite-1", ("a", "b"), qualify.Detail.TALLIES),
        ("suite-2", ("c",), qualify.Detail.TALLIES),
        ("soak-soak_mesh", ("soak_mesh",), qualify.Detail.ROWS),
        ("soak-soak_throttled", ("soak_throttled",), qualify.Detail.ROWS),
        ("pi", (), qualify.Detail.TALLIES),
        ("calibration", (), qualify.Detail.TALLIES),
    ]


def test_a_plan_that_loses_a_suite_row_is_refused():
    rows = [SimpleNamespace(name=name) for name in ("a", "b")]
    suite = SimpleNamespace(ROWS=rows, SOAK_ROWS=[])
    with pytest.raises(qualify.QualifyError, match="exactly once"):
        qualify.units_of(suite, lambda _names: [["a"]])


def test_a_passed_unit_with_the_same_key_within_the_window_is_skipped(
    tmp_path: Path, clock: Clock, bench: Bench, capsys
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    clock.now = T0 + datetime.timedelta(seconds=qualify.QUALIFY_WINDOW_S)
    done = qualify.run_units(bench.units(), context(tmp_path), {})
    assert bench.ran == ["deploy", "suite-1", "suite-2", "pi"]
    assert [entry["passed"] for entry in done] == [True] * 4
    out = capsys.readouterr().out
    assert "qualify: skip deploy: passed at 2026-11-16T20:10:00+00:00" in out
    assert "with the same rev, F401 build" in out


@pytest.mark.parametrize(
    ("changes", "rerun"),
    [
        ({"rev": "f" * 40}, ["deploy", "suite-1", "suite-2", "pi"]),
        ({"target_id": "9" * 40}, ["deploy", "suite-1", "suite-2", "pi"]),
        ({"expected": {"pause_sparse": 861}}, ["suite-1"]),
        ({"expected": {}}, ["suite-1"]),
        ({"expected": {"pause_sparse": 860, "estop": 855}}, ["suite-2"]),
        (
            {"identity": {**IDENTITY, "kernel": "6.12.30+rpt-rpi-2712"}},
            ["deploy", "suite-1", "suite-2", "pi"],
        ),
    ],
)
def test_a_unit_reruns_when_its_key_changes(
    tmp_path: Path, bench: Bench, changes: dict, rerun: list[str]
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    bench.ran.clear()
    qualify.run_units(bench.units(), context(tmp_path, **changes), {})
    assert bench.ran == rerun


def test_a_changed_pi_names_what_changed(tmp_path: Path, bench: Bench, capsys):
    qualify.run_units(bench.units(), context(tmp_path), {})
    newer = {**IDENTITY, "kernel": "6.12.30+rpt-rpi-2712"}
    qualify.run_units(bench.units(), context(tmp_path, identity=newer), {})
    out = capsys.readouterr().out
    assert "qualify: unit deploy (pi_identity_sha256, pi kernel changed" in out


def test_a_unit_reruns_past_the_window(
    tmp_path: Path, clock: Clock, bench: Bench, capsys
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    bench.ran.clear()
    clock.advance(qualify.QUALIFY_WINDOW_S + 1)
    qualify.run_units(bench.units(), context(tmp_path), {})
    assert bench.ran == ["deploy", "suite-1", "suite-2", "pi"]
    assert "past the 24.0 h window" in capsys.readouterr().out


def test_a_failed_unit_stops_the_run_and_reruns_next_time(
    tmp_path: Path, bench: Bench
):
    bench.failing = {"suite-1"}
    done = qualify.run_units(bench.units(), context(tmp_path), {})
    assert bench.ran == ["deploy", "suite-1"]
    assert [(e["unit"], e["passed"]) for e in done] == [
        ("deploy", True),
        ("suite-1", False),
    ]
    assert qualify.recorded(tmp_path / "record", "suite-2") is None
    bench.failing = set()
    bench.ran.clear()
    qualify.run_units(bench.units(), context(tmp_path), {})
    assert bench.ran == ["suite-1", "suite-2", "pi"]


def test_deploy_reruns_when_the_board_was_flashed_since(
    tmp_path: Path, bench: Bench
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    bench.ran.clear()
    deployed = {"firmware": {"target": {"build_id": "9" * 40}}}
    drift = qualify.deploy_drift(deployed, TARGET)
    assert "names F401 build 999" in drift["deploy"]
    qualify.run_units(bench.units(), context(tmp_path), drift)
    assert bench.ran == ["deploy"]
    on_target = {"firmware": {"target": {"build_id": TARGET}}}
    assert qualify.deploy_drift(on_target, TARGET) == {}


def test_a_unit_that_raises_fails_with_its_reason(tmp_path: Path, clock):
    def broken(_context, _unit):
        raise qualify.QualifyError("the Pi is unreachable")

    entry = qualify.executed(qualify.Unit("deploy", broken), context(tmp_path))
    assert entry["passed"] is False
    assert entry["failures"] == ["the Pi is unreachable"]
    assert entry["seconds"] == 0
    assert entry["holds"] == []


@pytest.fixture
def tagged(monkeypatch) -> list[dict]:
    summaries = []

    def publish(record: Path, summary: dict) -> str:
        summaries.append(summary)
        return "qualified-2026-11-16"

    monkeypatch.setattr(qualify, "publish", publish)
    return summaries


def test_the_tag_is_published_when_every_unit_passed_within_the_window(
    tmp_path: Path, bench: Bench, tagged: list[dict]
):
    ctx = context(tmp_path)
    assert qualify.qualified(bench.units(), ctx, "Merge #877", {}) == 0
    [summary] = tagged
    assert summary["record"] == str(tmp_path / "record")
    assert [unit["unit"] for unit in summary["units"]] == [
        "deploy",
        "suite-1",
        "suite-2",
        "pi",
    ]
    assert summary["units"][1]["holds"] == [
        {
            "index": 1,
            "rows": ["first_layers", "pause_sparse"],
            "wait_s": WAIT_S,
            "held_s": float(UNIT_S),
        }
    ]
    assert summary["bench_s"] == 4 * UNIT_S
    assert summary["lock_wait_s"] == 4 * WAIT_S
    assert summary["pi_identity"] == IDENTITY
    assert summary["pi_identity_sha256"] == qualify.identity_sha256(IDENTITY)
    assert all("key" not in unit for unit in summary["units"])
    assert qualify.parse_message(qualify.tag_message(summary)) == summary


def test_no_tag_while_a_unit_failed(
    tmp_path: Path, bench: Bench, tagged: list[dict], capsys
):
    bench.failing = {"suite-2"}
    assert qualify.qualified(bench.units(), context(tmp_path), "s", {}) == 1
    assert tagged == []
    out = capsys.readouterr().out
    assert "qualify: FAIL at unit suite-2 on 0123456789ab; no tag" in out
    written = tmp_path / "record" / qualify.RECORD_FILE
    record = json.loads(written.read_text())
    assert record["units"][3] == {"unit": "pi", "missing": True, "holds": []}


def test_no_tag_when_a_skipped_unit_left_the_window_during_the_run(
    tmp_path: Path, clock: Clock, bench: Bench, tagged: list[dict], capsys
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    deploy_finished = T0 + datetime.timedelta(seconds=UNIT_S)
    window = datetime.timedelta(seconds=qualify.QUALIFY_WINDOW_S)
    clock.now = deploy_finished + window - datetime.timedelta(seconds=300)
    retolerated = context(tmp_path, expected={"pause_sparse": 861})
    assert qualify.qualified(bench.units(), retolerated, "s", {}) == 1
    assert bench.ran[-1] == "suite-1"
    assert tagged == []
    out = capsys.readouterr().out
    assert "qualify: no tag on 0123456789ab: deploy: passed 24.1 h ago" in out


def test_a_unit_without_a_record_blocks_the_tag(
    tmp_path: Path, clock: Clock, bench: Bench
):
    qualify.run_units(bench.units(), context(tmp_path), {})
    units = [*bench.units(), bench.unit("calibration")]
    assert qualify.unpublishable(units, context(tmp_path), clock()) == [
        "calibration: no record"
    ]


def test_a_hold_that_never_got_the_lock_adds_no_wait(tmp_path: Path, clock):
    ctx = context(tmp_path)
    never = {"index": 1, "rows": ["a"], "wait_s": None, "held_s": 0.0}
    taken = {"index": 2, "rows": ["a"], "wait_s": 4.0, "held_s": 100.0}
    entry = {"unit": "suite-1", "holds": [never, taken]}
    qualify.write_json(qualify.unit_path(ctx.record, "suite-1"), entry)
    summary = qualify.summarized([qualify.Unit("suite-1", None)], ctx, "s")
    assert (summary["bench_s"], summary["lock_wait_s"]) == (100.0, 4.0)


def test_a_tag_already_on_the_rev_is_reused(
    repo: Path, tmp_path: Path, monkeypatch
):
    qualify.create_tag("qualified-2026-11-16", "HEAD", SUMMARY, cwd=repo)
    monkeypatch.chdir(repo)
    monkeypatch.setattr(qualify, "pushed_tag", pytest.fail)
    head = git(repo, "rev-parse", "HEAD")
    record = tmp_path / "record"
    name = qualify.publish(record, {**SUMMARY, "rev": head})
    assert name == "qualified-2026-11-16"
    written = json.loads((record / "qualification.json").read_text())
    assert written["tag"] == name


def test_the_pi_identity_is_read_over_ssh_without_the_boot_id():
    sums = (
        "1111  /var/lib/dpkg/status\n"
        "2222  /boot/firmware/config.txt\n"
        "3333  /boot/firmware/cmdline.txt\n"
    )
    asked = []

    def ssh(command: str) -> str:
        asked.append(command)
        if command.startswith("sha256sum"):
            return sums
        return f"{command}-out\n"

    deployed = {"firmware": {"capture": {"build_id": "a0e9"}}}
    identity = qualify.pi_identity(ssh, deployed)
    assert identity["kernel"] == "uname -r-out"
    assert identity["dpkg_status_sha256"] == "1111"
    assert identity["config_txt_sha256"] == "2222"
    assert identity["cmdline_txt_sha256"] == "3333"
    assert identity["capture_build_id"] == "a0e9"
    assert set(identity) == {
        *qualify.PI_FACTS,
        *(f"{name}_sha256" for name in qualify.PI_FILES),
        "capture_build_id",
    }
    assert not any("boot_id" in command for command in asked)
    with pytest.raises(qualify.QualifyError, match="flash-capture"):
        qualify.pi_identity(ssh, {"firmware": {}})


def test_a_deploy_hold_cut_at_max_hold_records_the_hold(tmp_path: Path):
    handoff = tmp_path / "deploy-hold.json"
    lost = qualify.handed_back(handoff, 1, 124)
    assert lost.failures == (
        "the deploy hold exited 1 before taking the lock: deploy.log",
    )
    taken = {"index": 1, "rows": [], "wait_s": 12.0, "acquired": 0.0}
    handoff.write_text(json.dumps({"hold": taken}))
    cut = qualify.handed_back(handoff, 124, 124)
    assert not cut.passed
    assert "exited 124 at MAX_HOLD_S" in cut.failures[0]
    assert cut.holds[0]["exit"] == 124
    assert cut.holds[0]["held_s"] > 0
    finished = {**taken, "held_s": 41.5, "restored": ["klippy was shutdown"]}
    verdict = {"passed": True, "facts": {"x": 1}, "failures": []}
    handoff.write_text(json.dumps({"hold": finished, "outcome": verdict}))
    done = qualify.handed_back(handoff, 0, 124)
    assert done.passed
    assert done.facts == {"x": 1}
    assert done.holds == ({**finished, "exit": 0},)


def test_bisect_runs_one_suite_per_candidate_without_an_outer_hold(
    tmp_path: Path, monkeypatch, capsys
):
    candidates = [f"m{n}" for n in range(6)]

    def history(*args: str, cwd=None) -> str:
        if args[0] == "rev-list":
            return "\n".join(candidates)
        return "Merge pull request #12 from me/x"

    commands = []

    def stream(command, cwd, log, target):
        commands.append(command)
        sha = Path(command[command.index("--tree") + 1]).name
        summary = tmp_path / f"{sha}-summary.json"
        summary.write_text(json.dumps({"rows": [{"passed": sha < "m3"}]}))
        return 0, f"summary: {summary}\n"

    lock = SimpleNamespace(TOKEN_ENV="BENCH_LOCK_TOKEN")
    order = _lock_order(monkeypatch)
    monkeypatch.delenv("BENCH_LOCK_TOKEN", raising=False)
    monkeypatch.setattr(qualify, "checked_out", lambda rev: rev)
    monkeypatch.setattr(qualify, "resolve", lambda rev: rev)
    monkeypatch.setattr(qualify, "bench", lambda _tree, _name: lock)
    monkeypatch.setattr(qualify, "hold", pytest.fail)
    monkeypatch.setattr(qualify, "git", history)
    monkeypatch.setattr(qualify, "checkout", lambda _tree, sha: tmp_path / sha)
    monkeypatch.setattr(qualify, "unmoved", lambda _tree, _sha: None)
    monkeypatch.setattr(
        qualify, "build_engine", lambda tree: order.append(("engine", ()))
    )

    def held_stream(command, cwd, log, target):
        order.append(("suite", tuple(order.held)))
        return stream(command, cwd, log, target)

    monkeypatch.setattr(qualify, "stream", held_stream)
    monkeypatch.setattr(qualify, "QUALIFICATIONS", tmp_path)
    args = argparse.Namespace(row="pause_sparse", good="m", bad="m5")
    assert qualify.bisect_command(args) == 0
    trees = [Path(c[c.index("--tree") + 1]).name for c in commands]
    assert len(trees) == len(set(trees))
    for command in commands:
        assert command[3:5] == ("tools/bench/suite.py", "--row")
        assert command[5] == "pause_sparse"
        assert not any("lock.py" in part for part in command)
    assert "bisect: first bad merge m3 (PR #12)" in capsys.readouterr().out
    monkeypatch.setenv("BENCH_LOCK_TOKEN", "4242")
    with pytest.raises(qualify.QualifyError, match="outside lock.py hold"):
        qualify.bisect_command(args)
    assert ("suite", ("lock",)) in order
    assert all(held == ("lock",) for name, held in order if name == "suite")
    assert all(not held for name, held in order if name == "slot")


class Order(list):
    held: list[str]


def _lock_order(monkeypatch) -> Order:
    order = Order()
    order.held = []

    def recorded(name: str):
        @contextlib.contextmanager
        def manager(*_args, **_kwargs):
            order.append((name, tuple(order.held)))
            order.held.append(name)
            try:
                yield
            finally:
                order.held.remove(name)

        return manager

    monkeypatch.setattr(qualify.slots, "slot", recorded("slot"))
    monkeypatch.setattr(qualify.pending, "commit_lock", recorded("lock"))
    return order


def test_builds_take_the_slot_before_the_lock_and_units_no_slot(monkeypatch):
    order = _lock_order(monkeypatch)

    def step(name: str):
        return lambda *_args: order.append((name, tuple(order.held)))

    lock = SimpleNamespace(TOKEN_ENV="BENCH_LOCK_TOKEN")
    monkeypatch.delenv("BENCH_LOCK_TOKEN", raising=False)
    monkeypatch.setattr(qualify, "checked_out", lambda rev: "abc")
    monkeypatch.setattr(qualify, "bench", lambda _tree, _name: lock)
    monkeypatch.setattr(qualify, "unmoved", step("unmoved"))
    monkeypatch.setattr(qualify, "build_engine", step("engine"))
    monkeypatch.setattr(qualify, "build_target", step("f401"))
    monkeypatch.setattr(qualify, "run", lambda *a: step("units")() or 0)
    args = argparse.Namespace(rev="origin/main", expect_fail=[])
    assert qualify.run_command(args) == 0
    assert order == [
        ("lock", ()),
        ("slot", ()),
        ("lock", ("slot",)),
        ("unmoved", ("slot", "lock")),
        ("engine", ("slot", "lock")),
        ("f401", ("slot", "lock")),
        ("lock", ()),
        ("unmoved", ("lock",)),
        ("units", ("lock",)),
    ]


def test_a_run_tree_another_qualify_moved_is_refused(monkeypatch, tmp_path):
    monkeypatch.setattr(qualify, "git", lambda *_a, cwd=None: "f" * 40)
    with pytest.raises(qualify.QualifyError, match="another qualify.py"):
        qualify.unmoved(tmp_path, "a" * 40)
    qualify.unmoved(tmp_path, "f" * 40)


def test_the_tag_message_round_trips_the_summary():
    message = qualify.tag_message(SUMMARY)
    assert message.startswith("Qualify 0123456789ab\n\n")
    assert qualify.parse_message(message) == SUMMARY


def test_a_tag_carries_the_summary_through_git(repo: Path):
    qualify.create_tag("qualified-2026-11-16", "HEAD", SUMMARY, cwd=repo)
    assert qualify.read_tag("qualified-2026-11-16", cwd=repo) == SUMMARY


def test_a_second_qualification_on_one_day_gets_the_next_suffix():
    day = datetime.date(2026, 11, 16)
    assert qualify.next_tag(set(), day) == "qualified-2026-11-16"
    taken = {"qualified-2026-11-16", "qualified-2026-11-16-2"}
    assert qualify.next_tag(taken, day) == "qualified-2026-11-16-3"


def test_the_debt_counts_first_parent_merges_since_the_tag(repo: Path):
    qualify.create_tag("qualified-2026-11-01", "HEAD", SUMMARY, cwd=repo)
    merge(repo, "a", {"rust/x.rs": "1", "klippy/y.py": "1"}, 10)
    merge(repo, "b", {"tools/bench/z.py": "1"}, 11)
    merge(repo, "c", {"docs/d.md": "1", "src/e.c": "1"}, 12)
    merges = qualify.first_parents("qualified-2026-11-01", "main", repo)
    assert [merge.pr for merge in merges] == [12, 11, 10]
    assert qualify.debt(merges) == {
        "merges": 3,
        "rust/": 1,
        "src/": 1,
        "klippy/": 1,
        "tools/bench/": 1,
    }
    line = qualify.status(qualify.View.LINE, "main", repo)
    assert line == (
        f"3 merges since qualified-2026-11-01; run {qualify.RUN_COMMAND}"
    )
    table = qualify.status(qualify.View.TABLE, "main", repo).splitlines()
    assert table[0].startswith("last qualified: qualified-2026-11-01 at ")
    assert "touching rust/ 1, src/ 1, klippy/ 1, tools/bench/ 1" in table[1]
    assert table[2].endswith("#12 Merge pull request #12 from me/c [src/]")


def test_without_a_tag_nothing_is_qualified(repo: Path):
    assert qualify.status(qualify.View.LINE, "main", repo) == (
        f"no qualified-* tag on main yet; run {qualify.RUN_COMMAND}"
    )
    assert "none of its 1 first-parent commits" in qualify.status(
        qualify.View.TABLE, "main", repo
    )


def test_bisect_finds_the_first_merge_the_row_fails_on():
    candidates = [f"m{n}" for n in range(9)]
    asked = []

    def passes(sha: str) -> bool:
        asked.append(sha)
        return int(sha[1:]) < 6

    assert qualify.first_bad(candidates, passes) == "m6"
    assert asked[0] == "m8"
    assert len(asked) <= 5


def test_bisect_refuses_a_bad_end_that_passes():
    with pytest.raises(qualify.QualifyError, match="flaky or already fixed"):
        qualify.first_bad(["m0", "m1"], lambda _sha: True)


def test_an_only_candidate_that_fails_is_the_culprit():
    assert qualify.first_bad(["m0"], lambda _sha: False) == "m0"


def test_a_tolerated_failure_needs_its_issue_number():
    assert qualify.expect_fail(["pause_sparse=#860", "estop=855"]) == {
        "pause_sparse": 860,
        "estop": 855,
    }
    for value in ("pause_sparse", "pause_sparse=", "pause_sparse=#x"):
        with pytest.raises(qualify.QualifyError, match="ROW=ISSUE"):
            qualify.expect_fail([value])


def test_a_failed_row_prints_its_bisect_command():
    done = [
        {"unit": "deploy", "passed": True, "failures": [], "facts": {}},
        {
            "unit": "soak-soak_throttled",
            "passed": False,
            "failures": ["soak_throttled: FAIL pacer_brakes"],
            "facts": {"failed_rows": ["soak_throttled"]},
        },
    ]
    lines = qualify.failure_report(done, REV)
    assert lines[0] == (
        "qualify: FAIL at unit soak-soak_throttled on 0123456789ab; no tag"
    )
    assert lines[-2] == (
        "  bisect: uv run python scripts/qualify.py bisect --row "
        f"soak_throttled --bad {REV}"
    )
    assert lines[-1].startswith(f"  resume: {qualify.RUN_COMMAND} {REV}")


def deployed(head: str = REV, target: str = TARGET) -> dict:
    firmware = {"target": {"build_id": target}}
    return {"head": head, "dirty_paths": [], "firmware": firmware}


def suite_summary(path: Path, rows: list[dict]) -> Path:
    holds = [
        {
            "index": 1,
            "rows": [row["name"] for row in rows],
            "wait_s": 4.0,
            "held_s": 1200.0,
            "restored": [],
            "exit": 0,
        }
    ]
    path.write_text(
        json.dumps(
            {
                "rows": rows,
                "aborted": None,
                "step_benchmark": {"within_band": True},
                "holds": holds,
            }
        )
    )
    return path


def test_a_suite_summary_tallies_rows_and_names_unexpected_failures(
    tmp_path: Path,
):
    rows = [
        {"name": "a", "passed": True, "expected_fail": False},
        {"name": "b", "passed": False, "expected_fail": True},
        {"name": "c", "passed": False, "expected_fail": False},
    ]
    summary = suite_summary(tmp_path / "summary.json", rows)
    outcome = qualify.judged_suite(
        summary, 1, qualify.Detail.TALLIES, context(tmp_path)
    )
    assert not outcome.passed
    assert outcome.facts["tallies"] == {
        "rows": 3,
        "passed": 1,
        "failed": 1,
        "xfail": 1,
        "xpass": 0,
    }
    assert outcome.facts["failed_rows"] == ["c"]
    assert "rows" not in outcome.facts
    assert outcome.holds[0]["rows"] == ["a", "b", "c"]
    assert outcome.holds[0]["held_s"] == 1200.0


def test_a_row_run_on_another_firmware_fails_its_unit(tmp_path: Path):
    rows = [
        {"name": "a", "passed": True, "expected_fail": False},
        {
            "name": "b",
            "passed": True,
            "expected_fail": False,
            "deployed": deployed(target="9" * 40),
        },
        {
            "name": "c",
            "passed": True,
            "expected_fail": False,
            "deployed": deployed(),
        },
    ]
    summary = suite_summary(tmp_path / "summary.json", rows)
    outcome = qualify.judged_suite(
        summary, 0, qualify.Detail.ROWS, context(tmp_path)
    )
    assert not outcome.passed
    assert outcome.failures == (
        f"b ran on 0123456789ab (dirty: []) with F401 build {'9' * 40}, "
        f"not clean 0123456789ab with {TARGET}",
    )
    assert [row["name"] for row in outcome.facts["rows"]] == ["a", "b", "c"]
