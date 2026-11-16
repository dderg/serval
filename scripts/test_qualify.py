from __future__ import annotations

import datetime
import json
import subprocess
from pathlib import Path

import pytest
import qualify

SUMMARY = {
    "rev": "0123456789abcdef0123456789abcdef01234567",
    "expect_fail": [{"row": "pause_sparse", "issue": 860}],
    "stages": [{"stage": "deploy", "seconds": 61, "passed": True}],
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


def test_stages_run_in_order_and_stop_at_the_first_failure():
    ran = []

    def stage(name: str, passed: bool) -> qualify.Stage:
        def run(_context):
            ran.append(name)
            return qualify.Outcome(
                passed, {"note": name}, () if passed else ("x",)
            )

        return qualify.Stage(name, run)

    stages = [stage("deploy", True), stage("suite", False), stage("soak", True)]
    done = qualify.run_stages(stages, context=None)
    assert ran == ["deploy", "suite"]
    assert [(d["stage"], d["passed"]) for d in done] == [
        ("deploy", True),
        ("suite", False),
    ]
    assert done[1]["failures"] == ["x"]
    assert done[1]["note"] == "suite"


def test_a_stage_that_raises_fails_with_its_reason():
    def broken(_context):
        raise qualify.QualifyError("the Pi is unreachable")

    done = qualify.run_stages([qualify.Stage("deploy", broken)], context=None)
    assert done == [
        {
            "stage": "deploy",
            "seconds": 0,
            "passed": False,
            "failures": ["the Pi is unreachable"],
        }
    ]


def test_the_stages_are_deploy_suite_soak_pi_calibration():
    assert [stage.name for stage in qualify.STAGES] == [
        "deploy",
        "suite",
        "soak",
        "pi",
        "calibration",
    ]


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
    stages = [
        {"stage": "deploy", "passed": True, "failures": []},
        {
            "stage": "soak",
            "passed": False,
            "failures": ["soak_throttled: FAIL pacer_brakes"],
            "failed_rows": ["soak_throttled"],
        },
    ]
    lines = qualify.failure_report(stages, SUMMARY["rev"])
    assert lines[0] == "qualify: FAIL at stage soak on 0123456789ab; no tag"
    assert lines[-1] == (
        "  bisect: uv run python scripts/qualify.py bisect --row "
        f"soak_throttled --bad {SUMMARY['rev']}"
    )


def test_a_suite_summary_tallies_rows_and_names_unexpected_failures(
    tmp_path: Path,
):
    rows = [
        {"name": "a", "passed": True, "expected_fail": False},
        {"name": "b", "passed": False, "expected_fail": True},
        {"name": "c", "passed": False, "expected_fail": False},
    ]
    summary = tmp_path / "summary.json"
    summary.write_text(
        json.dumps(
            {
                "rows": rows,
                "aborted": None,
                "step_benchmark": {"within_band": True},
            }
        )
    )
    outcome = qualify.judged_suite(summary, 1, qualify.Detail.TALLIES)
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
