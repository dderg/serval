import json

import fuzz_triage as ft

SEED = "5c0ad8087ca5beb11ee58d94b016b830335211ac6eab95eb9f2970ca593d9db3"
OTHER_SEED = "2f9a69f78f9c4ef9adc92c9be879fb454a69344a523ab8edcb4006c1f5f84fc0"

LOG = f"""\
^[[32;1m    Starting^[[0m ^[[1m10^[[0m tests across ^[[1m3^[[0m binaries
^[[31;1m        FAIL^[[0m [   1.103s] ( 8/10) ^[[35;1mpipeline-snapshot::planner_conservation_fuzz^[[0m ^[[34;1mthe_extruder_lays_its_ratio^[[0m
    stdout

    running 1 test
    test the_extruder_lays_its_ratio ... FAILED
    stderr
    thread 'the_extruder_lays_its_ratio' panicked at motion-pipeline/src/fit_stage.rs:428:29:
    fit_stage: junction plan failed: LineSliver {{ line_no: 10 }}

    thread 'the_extruder_lays_its_ratio' panicked at motion-pipeline/src/fit_stage.rs:428:29:
    fit_stage: junction plan failed: LineSliver {{ line_no: 10 }}

    thread 'the_extruder_lays_its_ratio' panicked at pipeline-snapshot/tests/planner_conservation_fuzz.rs:339:1:
    Test failed: junction plan failed.
    minimal failing input: limits = Limits {{ x: 1 }}
    	successes: 0
    cc {SEED}
^[[31;1m     Summary^[[0m [   1.487s] ^[[1m10^[[0m tests run: ^[[1m9^[[0m passed, 1 failed
^[[31;1m        FAIL^[[0m [   1.103s] ( 8/10) ^[[35;1mpipeline-snapshot::planner_conservation_fuzz^[[0m ^[[34;1mthe_extruder_lays_its_ratio^[[0m
^[[31;1merror^[[0m: test run failed
"""

TIMEOUT_LOG = """\
     TIMEOUT [ 180.004s] (4/110) gcode::property_lex lexer_terminates_on_long_input
"""


def test_a_nextest_log_yields_one_failure_with_seed_and_inner_panic_location():
    [failure] = ft.parse_log(LOG)
    assert failure.binary == "pipeline-snapshot::planner_conservation_fuzz"
    assert failure.test == "the_extruder_lays_its_ratio"
    assert failure.seed == f"cc:{SEED}"
    assert failure.location == "motion-pipeline/src/fit_stage.rs:428:29"
    assert "LineSliver" in failure.message


def test_the_summary_repeat_of_the_fail_line_does_not_duplicate_the_failure():
    assert len(ft.parse_log(LOG)) == 1


def test_a_timeout_has_no_seed_and_no_location():
    [failure] = ft.parse_log(TIMEOUT_LOG)
    assert failure.seed is None and failure.location is None


def test_the_machine_readable_line_round_trips_through_the_parser():
    [failure] = ft.parse_log(LOG)
    line = ft.format_line(failure)
    assert line.startswith("FUZZ_FAILURE {")
    assert json.loads(line.split(" ", 1)[1])["seed"] == f"cc:{SEED}"
    assert ft.parse_log("some prefix " + line + "\n") == [failure]


def test_the_key_names_test_and_location_not_the_seed():
    [failure] = ft.parse_log(LOG)
    assert failure.key == (
        "pipeline-snapshot::planner_conservation_fuzz::the_extruder_lays_its_ratio"
        "@motion-pipeline/src/fit_stage.rs:428:29"
    )
    assert SEED not in failure.key


def test_an_existing_issue_with_the_key_is_found_and_others_are_not():
    [failure] = ft.parse_log(LOG)
    issues = [
        {"number": 1, "body": "unrelated"},
        {"number": 2, "body": ft.issue_body(failure, "http://run", "b")},
    ]
    assert ft.find_issue(issues, failure.key)["number"] == 2
    assert ft.find_issue(issues[:1], failure.key) is None


def test_the_repro_command_selects_the_binary_and_test():
    [failure] = ft.parse_log(LOG)
    command = ft.repro_command(failure)
    assert "-p pipeline-snapshot" in command
    assert "--test planner_conservation_fuzz" in command
    assert "test(=the_extruder_lays_its_ratio)" in command


class FakeWorld:
    def __init__(self, monkeypatch, jobs, log, on_main, contains_main=True):
        self.calls = []
        self.issues = []
        monkeypatch.setattr(ft, "failed_jobs", lambda run_id: jobs)
        monkeypatch.setattr(ft, "fails_on_main", lambda failure: on_main)
        monkeypatch.setattr(ft, "contains_main", lambda branch: contains_main)
        monkeypatch.setattr(ft, "repo_slug", lambda: "o/r")

        def gh(*args):
            self.calls.append(args)
            if args[:2] == ("run", "view") and "--log-failed" in args:
                return log
            if args[:2] == ("issue", "list"):
                return json.dumps(self.issues)
            if args[:2] == ("issue", "view"):
                number = int(args[2])
                body = next(
                    i["body"] for i in self.issues if i["number"] == number
                )
                return json.dumps({"body": body, "comments": []})
            return "https://github.com/o/r/issues/9\n"

        monkeypatch.setattr(ft, "gh", gh)

    def ran(self, *prefix):
        return any(call[: len(prefix)] == prefix for call in self.calls)


def test_a_failure_main_also_has_opens_an_issue_and_reruns_the_job(monkeypatch):
    world = FakeWorld(monkeypatch, ["rust-fuzz (2/3)"], LOG, on_main=True)
    assert ft.triage("b", "77") == ft.RERUN_EXIT
    assert world.ran("issue", "create")
    assert world.ran("run", "rerun", "77", "--failed")


def test_a_known_issue_gets_the_seed_once_not_a_duplicate(monkeypatch):
    world = FakeWorld(monkeypatch, ["rust-fuzz (2/3)"], LOG, on_main=True)
    [failure] = ft.parse_log(LOG)
    world.issues = [
        {"number": 5, "body": ft.issue_body(failure, "http://x", "b")}
    ]
    assert ft.triage("b", "77") == ft.RERUN_EXIT
    assert not world.ran("issue", "create")
    assert not world.ran("issue", "comment"), (
        "the body already carries this seed"
    )


def test_a_known_issue_gets_a_comment_for_a_new_seed(monkeypatch):
    world = FakeWorld(
        monkeypatch,
        ["rust-fuzz (2/3)"],
        LOG.replace(SEED, OTHER_SEED),
        on_main=True,
    )
    [failure] = ft.parse_log(LOG)
    world.issues = [
        {"number": 5, "body": ft.issue_body(failure, "http://x", "b")}
    ]
    assert ft.triage("b", "77") == ft.RERUN_EXIT
    assert world.ran("issue", "comment", "5")


def test_a_failure_that_passes_on_main_is_this_prs_regression(monkeypatch):
    world = FakeWorld(monkeypatch, ["rust-fuzz (1/3)"], LOG, on_main=False)
    assert ft.triage("b", "77") == ft.REGRESSION_EXIT
    assert not world.ran("run", "rerun")


def test_a_branch_behind_main_is_told_to_merge_main(monkeypatch, capsys):
    FakeWorld(
        monkeypatch,
        ["rust-fuzz (1/3)"],
        LOG,
        on_main=False,
        contains_main=False,
    )
    assert ft.triage("b", "77") == ft.REGRESSION_EXIT
    assert "merge main" in capsys.readouterr().err


def test_another_red_job_makes_the_run_untriageable(monkeypatch):
    world = FakeWorld(
        monkeypatch, ["rust-fuzz (1/3)", "rust-test"], LOG, on_main=True
    )
    assert ft.triage("b", "77") == ft.UNTRIAGEABLE_EXIT
    assert not world.ran("run", "rerun")


def test_a_failure_without_a_seed_is_untriageable(monkeypatch):
    FakeWorld(monkeypatch, ["rust-fuzz (1/3)"], TIMEOUT_LOG, on_main=True)
    assert ft.triage("b", "77") == ft.UNTRIAGEABLE_EXIT


def test_a_test_binary_that_cannot_be_placed_is_untriageable(monkeypatch):
    FakeWorld(monkeypatch, ["rust-fuzz (1/3)"], LOG, on_main=None)
    assert ft.triage("b", "77") == ft.UNTRIAGEABLE_EXIT


RNG_LOG = """\
        FAIL [   2.0s] ( 1/3) pipeline-snapshot::planner_fuzz hard_invariants_hold
    stderr
    planner_fuzz seed: reproduce with PROPTEST_RNG_SEED=1234567890123
    thread 'hard_invariants_hold' panicked at motion-pipeline/src/lower_stage.rs:77:9:
    boom
"""


def test_a_fixed_rng_seed_suite_reports_its_env_seed():
    [failure] = ft.parse_log(RNG_LOG)
    assert failure.seed == "rng:1234567890123"
    command = ft.repro_command(failure)
    assert command.startswith(
        "PROPTEST_RNG_SEED=1234567890123 PROPTEST_CASES=312 "
    )


def test_a_file_seed_repro_names_the_regressions_file_step():
    [failure] = ft.parse_log(LOG)
    assert f"add `cc {SEED}`" in ft.seed_instructions(failure)
    assert "PROPTEST_RNG_SEED" not in ft.repro_command(failure)
