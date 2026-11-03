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
