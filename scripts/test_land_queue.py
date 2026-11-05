import land_queue as lq
import pytest


def review(cases=(), new=(), removed=()):
    return {
        "base": "b" * 40,
        "head": "h" * 40,
        "expectations": [],
        "summary": {"new": list(new), "unchanged": 3, "removed": list(removed)},
        "cases": [
            {"name": n, "verdict": v, "regressions": ["r"], "suspicions": ["s"]}
            for n, v in cases
        ],
    }


def test_queue_lines_parse_with_and_without_expectation():
    entries = lq.parse_queue(
        "# comment\n\na|Title A|axes=e;dp=1e-9\nb|Title B\nc|Title C|\n"
    )
    assert entries == [
        lq.Entry("a", "Title A", "axes=e;dp=1e-9"),
        lq.Entry("b", "Title B", ""),
        lq.Entry("c", "Title C", ""),
    ]


@pytest.mark.parametrize("text", ["lonely", "a||x", "|t|x", "a|t|x\na|u|y"])
def test_malformed_or_duplicated_queue_lines_are_refused(text):
    with pytest.raises(lq.QueueError):
        lq.parse_queue(text)


def test_an_entry_without_expectation_stops_on_any_moved_case():
    cases, errors = lq.snapshot_gate(review([("a/p/c", "EXPECTED")]), "")
    assert cases == [] and errors


def test_an_entry_without_expectation_stops_on_a_new_case():
    assert lq.snapshot_gate(review(new=["n/p/c"]), "")[1]


def test_an_entry_without_expectation_passes_when_nothing_moved():
    assert lq.snapshot_gate(review(), "") == ([], [])


def test_expected_and_new_cases_are_accepted_under_an_expectation():
    cases, errors = lq.snapshot_gate(
        review([("a/p/c", "EXPECTED")], new=["n/p/c"]), "axes=e"
    )
    assert cases == ["a/p/c", "n/p/c"] and errors == []


@pytest.mark.parametrize("verdict", ["SUSPICIOUS", "REGRESSION"])
def test_a_flagged_case_stops_the_queue_even_with_an_expectation(verdict):
    assert lq.snapshot_gate(review([("a/p/c", verdict)]), "axes=e")[1]


def test_removed_cases_stop_the_queue():
    assert lq.snapshot_gate(review(removed=["gone/p/c"]), "axes=e")[1]


def run(name, conclusion, number):
    return {"name": name, "conclusion": conclusion, "databaseId": number}


def test_a_lone_red_sim_e2e_run_is_rerunnable():
    runs = [run("Ruff", "success", 1), run("Sim e2e", "failure", 2)]
    assert lq.failed_flaky_run(runs) == 2


def test_any_other_red_workflow_is_not_rerun():
    runs = [run("Sim e2e", "failure", 2), run("Snapshot tests", "failure", 3)]
    assert lq.failed_flaky_run(runs) is None


def test_no_red_run_means_nothing_to_rerun():
    assert lq.failed_flaky_run([run("Ruff", "success", 1)]) is None


def test_landed_branches_are_read_back(tmp_path):
    path = tmp_path / "q.landed"
    assert lq.read_landed(path) == set()
    path.write_text("a\nb\n")
    assert lq.read_landed(path) == {"a", "b"}


def test_reexec_is_skipped_when_already_running_the_current_copy(monkeypatch):
    monkeypatch.setenv(lq.REEXEC_VAR, "1")
    monkeypatch.setattr(
        lq.pending, "run", lambda *a, **k: pytest.fail("fetched")
    )
    lq.reexec_current_copy()


CHECK = """\
  EXACT    a/p/c
  CHANGED  b/p/c
  PENDING  n/p/c
"""


def test_a_case_already_exact_on_the_branch_is_not_accepted_again():
    assert lq.not_yet_accepted(["a/p/c", "b/p/c", "n/p/c"], CHECK) == [
        "b/p/c",
        "n/p/c",
    ]


def test_everything_already_exact_leaves_nothing_to_accept():
    assert lq.not_yet_accepted(["a/p/c"], CHECK) == []


def test_a_case_missing_from_the_check_output_is_still_accepted():
    assert lq.not_yet_accepted(["z/p/c"], CHECK) == ["z/p/c"]


def test_a_dirty_pr_is_detected_from_the_merge_state(monkeypatch, tmp_path):
    monkeypatch.setattr(
        lq.pending, "run", lambda args, cwd=None, env=None: "DIRTY\n"
    )
    assert lq.pr_is_dirty(tmp_path, "b")
    monkeypatch.setattr(
        lq.pending, "run", lambda args, cwd=None, env=None: "BLOCKED\n"
    )
    assert not lq.pr_is_dirty(tmp_path, "b")


def _repo_with_conflicting_main(tmp_path):
    import subprocess as sp

    def git(cwd, *a):
        return sp.run(
            ["git", *a], cwd=cwd, check=True, capture_output=True, text=True
        ).stdout.strip()

    origin = tmp_path / "origin.git"
    git(tmp_path, "init", "-q", "--bare", "-b", "main", str(origin))
    repo = tmp_path / "repo"
    git(tmp_path, "clone", "-q", str(origin), str(repo))
    for k, v in (("user.email", "t@t"), ("user.name", "t")):
        git(repo, "config", k, v)
    (repo / "f").write_text("1\n")
    git(repo, "add", "f")
    git(repo, "commit", "-qm", "base")
    git(repo, "push", "-q", "origin", "main")
    git(repo, "checkout", "-q", "-b", "topic")
    return repo, git


def test_remerge_main_brings_in_a_clean_main_and_reports_true(tmp_path):
    repo, git = _repo_with_conflicting_main(tmp_path)
    (repo / "g").write_text("topic\n")
    git(repo, "add", "g")
    git(repo, "commit", "-qm", "topic")
    git(repo, "checkout", "-q", "main")
    (repo / "h").write_text("main\n")
    git(repo, "add", "h")
    git(repo, "commit", "-qm", "main moves")
    git(repo, "push", "-q", "origin", "main")
    git(repo, "checkout", "-q", "topic")
    assert lq.remerge_main(repo, lq.Entry("topic", "t", "")) is True
    assert (repo / "h").exists()


def test_remerge_main_aborts_a_real_conflict_and_names_the_files(tmp_path):
    repo, git = _repo_with_conflicting_main(tmp_path)
    (repo / "f").write_text("topic\n")
    git(repo, "commit", "-qam", "topic edit")
    git(repo, "checkout", "-q", "main")
    (repo / "f").write_text("main\n")
    git(repo, "commit", "-qam", "main edit")
    git(repo, "push", "-q", "origin", "main")
    git(repo, "checkout", "-q", "topic")
    with pytest.raises(lq.QueueError, match="Conflicted files:\nf"):
        lq.remerge_main(repo, lq.Entry("topic", "t", ""))
    assert git(repo, "status", "--porcelain") == ""
