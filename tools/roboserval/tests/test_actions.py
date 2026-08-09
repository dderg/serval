import json
import sqlite3
import threading
import time
from pathlib import Path

import pytest

from serval_bot.actions import ActionDenied, ActionGateway, parse_sim_directive
from serval_bot.database import ActionConflict, Database, Event
from serval_bot.policy import Mode, RepositoryPolicy


class FakeProxy:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.runs: dict[int, dict] = {}
        self.ref_runs: dict[tuple, int] = {}
        self.sim_status = "completed"

    def add_labels(self, repo: str, issue_number: int, labels: list[str]) -> dict:
        self.calls.append(("labels", repo, issue_number, labels))
        return {"labels": labels}

    def post_comment(self, repo: str, issue_number: int, body: str) -> dict:
        self.calls.append(("comment", repo, issue_number, body))
        return {"id": 1, "url": "https://example.test/comment"}

    def search_issues(self, repo: str, query: str) -> dict:
        self.calls.append(("search", repo, query))
        return {"items": []}

    def dispatch_sim(self, repo: str, issue_number: int, workflow: str, ref: str, head_sha: str | None) -> dict:
        self.calls.append(("dispatch", repo, issue_number, workflow, ref, head_sha))
        key = (ref, head_sha)
        run_id = self.ref_runs.get(key)
        if run_id is None:
            run_id = 99 + len(self.runs)
            self.ref_runs[key] = run_id
            self.runs[run_id] = {
                "run_id": run_id,
                "url": f"https://example.test/run/{run_id}",
                "status": "queued",
                "conclusion": None,
                "head_sha": head_sha,
                "ref": f"serval-dispatch-{run_id}",
                "workflow": f".github/workflows/{workflow}",
                "requested_ref": ref,
            }
        return dict(self.runs[run_id])

    def sim_result(self, repo: str, run_id: int) -> dict:
        self.calls.append(("sim_result", repo, run_id))
        run = self.runs[run_id]
        return {
            **run,
            "status": self.sim_status,
            "conclusion": "success" if self.sim_status == "completed" else None,
            "jobs": [],
        }


def _event(
    actor: str = "reporter",
    event_type: str = "issues.opened",
    comment: str | None = None,
    delivery_id: str = "delivery",
    issue_number: int = 7,
) -> Event:
    payload = {}
    if comment is not None:
        payload["comment"] = {"body": comment}
    return Event(delivery_id, event_type, "dderg/serval", issue_number, actor, payload, "running", 1, None)


def _policy(mode: Mode) -> RepositoryPolicy:
    return RepositoryPolicy(
        repo="dderg/serval",
        mode=mode,
        bot_login="roboserval",
        maintainers=frozenset({"dderg"}),
        sim_workflow="ci-sim-e2e.yaml",
    )


def _claimed(database: Database, event: Event) -> Event:
    database.record_event(
        event.delivery_id, event.event_type, event.repo, event.issue_number, event.actor, event.payload
    )
    claimed = database.claim()
    assert claimed is not None
    return claimed


def _gateway(
    database: Database,
    event: Event,
    mode: Mode,
    proxy: FakeProxy | None = None,
) -> ActionGateway:
    return ActionGateway(database, event, _policy(mode), "trunk", proxy)


_SIM_SHA = "a" * 40
_SIM_FARM_REF = "farm/7-restart"


def test_shadow_mode_records_without_github_side_effect(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        result = json.loads(_gateway(database, event, Mode.SHADOW, proxy).classify("bug", "p2", ["host"], "reason"))
        assert result["state"] == "proposed"
        assert proxy.calls == []
        assert database.actions_for_issue(event.repo, event.issue_number)[0].state == "proposed"
    finally:
        database.close()


def test_triage_mode_applies_comment(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.classify("bug", "p2", ["host"], "reproducible host failure")
        result = json.loads(gateway.post_comment("Attach logs"))
        assert result["state"] == "applied"
        assert proxy.calls == [
            ("labels", "dderg/serval", 7, ["bug", "area:host", "priority:p2"]),
            ("comment", "dderg/serval", 7, "Attach logs"),
        ]
        actions = database.actions_for_issue(event.repo, event.issue_number)
        assert [action.kind for action in actions] == ["classify", "comment"]
        assert all(action.state == "applied" for action in actions)
    finally:
        database.close()


def test_opened_issue_exactly_one_classify_then_one_comment(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.classify("bug", None, ["host"], "reason")
        gateway.post_comment("first")
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.classify("bug", None, ["host"], "again")
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.post_comment("second")
        assert [call for call in proxy.calls if call[0] in {"labels", "comment"}] == [
            ("labels", "dderg/serval", 7, ["bug", "area:host"]),
            ("comment", "dderg/serval", 7, "first"),
        ]
    finally:
        database.close()


def test_comment_before_classify_denied_on_opened_issue(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        with pytest.raises(ActionDenied, match="successfully classified"):
            _gateway(database, event, Mode.TRIAGE, proxy).post_comment("jump the queue")
        assert proxy.calls == []
        assert database.actions_for_issue(event.repo, event.issue_number) == []
    finally:
        database.close()


def test_comment_after_failed_classification_is_denied(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        database.add_action(event, "classify", {"primary": "bug"}, "failed")
        with pytest.raises(ActionDenied, match="successfully classified"):
            _gateway(database, event, Mode.TRIAGE, proxy).post_comment("classification failed")
        assert proxy.calls == []
    finally:
        database.close()


def test_followup_classify_denied_and_exactly_one_comment(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event(event_type="issue_comment.created", comment="@roboserval triage this"))
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        with pytest.raises(ActionDenied, match="newly opened issues"):
            gateway.classify("bug", None, ["host"], "reason")
        gateway.post_comment("response")
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.post_comment("duplicate")
        assert [call for call in proxy.calls if call[0] == "comment"] == [("comment", "dderg/serval", 7, "response")]
    finally:
        database.close()


def test_pr_review_classify_and_labels_impossible(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event(event_type="pull_request_review.requested", comment="@roboserval review"))
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        with pytest.raises(ActionDenied, match="newly opened issues"):
            gateway.classify("bug", None, ["host"], "reason")
        gateway.post_comment("no findings")
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.post_comment("second")
        assert [call for call in proxy.calls if call[0] in {"labels", "comment"}] == [
            ("comment", "dderg/serval", 7, "no findings")
        ]
    finally:
        database.close()


def test_search_issues_is_read_only_and_allowed_for_all_events(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        for issue_number, event_type in enumerate(
            ("issues.opened", "issue_comment.created", "pull_request_review.requested"), start=7
        ):
            event = _claimed(
                database,
                _event(
                    delivery_id=f"delivery-{event_type}",
                    event_type=event_type,
                    comment="@roboserval hi",
                    issue_number=issue_number,
                ),
            )
            result = json.loads(_gateway(database, event, Mode.SHADOW, proxy).search_issues("restart"))
            assert result == {"items": []}
        assert [call for call in proxy.calls if call[0] == "search"] == [
            ("search", "dderg/serval", "restart"),
            ("search", "dderg/serval", "restart"),
            ("search", "dderg/serval", "restart"),
        ]
    finally:
        database.close()


def test_issue370_wording_dispatches_in_triage(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        result = json.loads(_gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None))
        assert result["state"] == "applied"
        assert result["result"]["run_id"] == 99
        assert result["result"]["requested_ref"] == "trunk"
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None)]
        runs = database.workflow_runs_for_issue(event.repo, event.issue_number)
        assert [(run["run_id"], run["ref"], run["workflow"]) for run in runs] == [
            (99, proxy.runs[99]["ref"], "ci-sim-e2e.yaml")
        ]
    finally:
        database.close()


def test_sim_dispatch_in_maintainer_mode_applies_and_persists(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        result = json.loads(_gateway(database, event, Mode.MAINTAINER, proxy).dispatch_sim(_SIM_FARM_REF, _SIM_SHA))
        assert result["state"] == "applied"
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", _SIM_FARM_REF, _SIM_SHA)]
        runs = database.workflow_runs_for_issue(event.repo, event.issue_number)
        assert [(run["run_id"], run["ref"]) for run in runs] == [(99, proxy.runs[99]["ref"])]
    finally:
        database.close()


def test_repeated_dispatch_denied_before_proxy(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.dispatch_sim("trunk", None)
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None)]
    finally:
        database.close()


def test_same_ref_dispatch_never_reuses_claimed_run(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        result = json.loads(_gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None))
        assert result["state"] == "applied"
        first_ref = result["result"]["ref"]
        database.finish(event.delivery_id, "done")
        later = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="issue_comment.created",
                comment="@roboserval Reproduce in the simulator",
                delivery_id="delivery-later",
            ),
        )
        with pytest.raises(ActionDenied, match="already claimed"):
            _gateway(database, later, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        runs = database.workflow_runs_for_issue(event.repo, event.issue_number)
        assert [(run["run_id"], run["ref"]) for run in runs] == [(99, first_ref)]
    finally:
        database.close()


def test_concurrent_same_ref_dispatch_serializes_and_claims_once(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    entered: list[str] = []
    entered_guard = threading.Lock()
    released = threading.Event()
    original_dispatch = proxy.dispatch_sim

    def blocking_dispatch(repo: str, issue_number: int, workflow: str, ref: str, head_sha: str | None) -> dict:
        with entered_guard:
            entered.append(ref)
        released.wait(10)
        return original_dispatch(repo, issue_number, workflow, ref, head_sha)

    proxy.dispatch_sim = blocking_dispatch
    first = _claimed(
        database,
        _event(
            actor="dderg",
            event_type="issue_comment.created",
            comment="@roboserval Reproduce in the simulator",
            delivery_id="delivery-first",
        ),
    )
    second = _claimed(
        database,
        _event(
            actor="dderg",
            event_type="issue_comment.created",
            comment="@roboserval Reproduce in the simulator",
            delivery_id="delivery-second",
            issue_number=8,
        ),
    )
    gateway_first = _gateway(database, first, Mode.TRIAGE, proxy)
    gateway_second = _gateway(database, second, Mode.TRIAGE, proxy)
    outcomes: list[dict] = []
    errors: list[Exception] = []

    def dispatch(gateway: ActionGateway) -> None:
        try:
            outcomes.append(json.loads(gateway.dispatch_sim("trunk", None)))
        except Exception as exc:
            errors.append(exc)

    thread_first = threading.Thread(target=dispatch, args=(gateway_first,))
    thread_second = threading.Thread(target=dispatch, args=(gateway_second,))
    thread_first.start()
    deadline = time.monotonic() + 10
    while not entered and time.monotonic() < deadline:
        time.sleep(0.01)
    assert entered == ["trunk"]
    thread_second.start()
    time.sleep(0.2)
    with entered_guard:
        assert entered == ["trunk"], "second dispatch entered the proxy before the first finished"
    released.set()
    thread_first.join(10)
    thread_second.join(10)
    assert not thread_first.is_alive() and not thread_second.is_alive()
    assert len(outcomes) == 1 and outcomes[0]["state"] == "applied"
    first_ref = outcomes[0]["result"]["ref"]
    assert len(errors) == 1
    assert isinstance(errors[0], ActionDenied)
    assert "already claimed" in str(errors[0])
    runs = database.workflow_runs_for_issue(first.repo, first.issue_number)
    assert [(run["run_id"], run["ref"]) for run in runs] == [(99, first_ref)]
    database.close()


def test_sim_result_applies_in_triage_and_persists(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.dispatch_sim("trunk", None)
        result = json.loads(gateway.sim_result(99))
        assert result["state"] == "applied"
        assert result["result"]["conclusion"] == "success"
        assert proxy.calls == [
            ("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None),
            ("sim_result", "dderg/serval", 99),
        ]
        runs = database.workflow_runs_for_issue(event.repo, event.issue_number)
        # The persisted association holds the actual dispatch ref, and the
        # sim_result identity check validated the run against it.
        assert [(run["run_id"], run["ref"], run["status"], run["conclusion"]) for run in runs] == [
            (99, proxy.runs[99]["ref"], "completed", "success")
        ]
    finally:
        database.close()


def test_repeated_sim_result_polls_refresh_the_run_scoped_record(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.dispatch_sim("trunk", None)
        first = json.loads(gateway.sim_result(99))
        assert first["state"] == "applied"
        second = json.loads(gateway.sim_result(99))
        assert second["state"] == "applied"
        assert [call for call in proxy.calls if call[0] == "sim_result"] == [
            ("sim_result", "dderg/serval", 99),
            ("sim_result", "dderg/serval", 99),
        ]
        reads = [
            action
            for action in database.actions_for_delivery(event.delivery_id)
            if action.kind.startswith("sim_result")
        ]
        assert [action.kind for action in reads] == ["sim_result:99"]
        assert all(action.state == "applied" for action in reads)
    finally:
        database.close()


def test_sim_result_requires_recorded_dispatch_before_proxy(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        with pytest.raises(ActionDenied, match="no recorded dispatch"):
            gateway.sim_result(99)
        assert proxy.calls == []
        assert database.workflow_runs_for_issue(event.repo, event.issue_number) == []
        assert database.actions_for_issue(event.repo, event.issue_number) == []
    finally:
        database.close()


def test_sim_result_denied_for_run_of_another_issue(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="issue_comment.created",
                comment="@roboserval Reproduce in the simulator",
                issue_number=7,
            ),
        )
        other = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="issue_comment.created",
                comment="@roboserval Reproduce in the simulator",
                delivery_id="delivery-other",
                issue_number=8,
            ),
        )
        _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="no recorded dispatch"):
            _gateway(database, other, Mode.TRIAGE, proxy).sim_result(99)
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None)]
    finally:
        database.close()


def test_sim_result_rejects_identity_mismatch(tmp_path: Path) -> None:
    for field, value, message in (
        ("workflow", ".github/workflows/other.yaml", "configured simulator workflow"),
        ("ref", "farm/other", "recorded ref"),
        ("run_id", 77, "different run"),
    ):
        database = Database(tmp_path / f"{field}.sqlite")
        proxy = FakeProxy()
        try:
            event = _claimed(
                database,
                _event(
                    actor="dderg",
                    event_type="issue_comment.created",
                    comment="@roboserval Reproduce in the simulator",
                    delivery_id=f"delivery-{field}",
                ),
            )
            gateway = _gateway(database, event, Mode.TRIAGE, proxy)
            gateway.dispatch_sim("trunk", None)
            proxy.runs[99][field] = value
            with pytest.raises(ActionDenied, match=message):
                gateway.sim_result(99)
            actions = database.actions_for_issue(event.repo, event.issue_number)
            assert actions[-1].kind == "sim_result:99"
            assert actions[-1].state == "failed"
            runs = database.workflow_runs_for_issue(event.repo, event.issue_number)
            assert [(run["run_id"], run["status"]) for run in runs] == [(99, "queued")]
        finally:
            database.close()


def test_sim_dispatch_shadow_records_proposal_only(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        result = json.loads(_gateway(database, event, Mode.SHADOW, proxy).dispatch_sim("trunk", None))
        assert result["state"] == "proposed"
        assert proxy.calls == []
        assert database.workflow_runs_for_issue(event.repo, event.issue_number) == []
        assert database.actions_for_issue(event.repo, event.issue_number)[0].state == "proposed"
    finally:
        database.close()


def test_sim_result_shadow_records_proposal_only(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.SHADOW, proxy)
        gateway.dispatch_sim("trunk", None)
        result = json.loads(gateway.sim_result(99))
        assert result["state"] == "proposed"
        assert proxy.calls == []
        assert database.workflow_runs_for_issue(event.repo, event.issue_number) == []
        kinds = [action.kind for action in database.actions_for_issue(event.repo, event.issue_number)]
        assert kinds == ["dispatch_sim:trunk:default", "sim_result:99"]
    finally:
        database.close()


def test_nonmaintainer_cannot_dispatch_simulator(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        with pytest.raises(ActionDenied, match="not authorized"):
            _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="not authorized"):
            _gateway(database, event, Mode.TRIAGE, proxy).sim_result(99)
        assert proxy.calls == []
    finally:
        database.close()


def test_maintainer_dispatch_is_recorded(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        result = json.loads(_gateway(database, event, Mode.MAINTAINER, proxy).dispatch_sim("trunk", _SIM_SHA))
        assert result["state"] == "applied"
        assert result["result"]["run_id"] == 99
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", _SIM_SHA)]
    finally:
        database.close()


def test_autonomous_opened_event_cannot_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event(actor="dderg"))
        with pytest.raises(ActionDenied, match="explicit mention comment"):
            _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="explicit mention comment"):
            _gateway(database, event, Mode.TRIAGE, proxy).sim_result(99)
        assert proxy.calls == []
    finally:
        database.close()


def test_maintainer_cannot_dispatch_unrelated_ref(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.MAINTAINER, proxy)
        for ref in ("sota-motion", "farm/calib-1", "farm/8-restart", "farm/7-", "farm/7", "farm/7-restart/extra"):
            with pytest.raises(ActionDenied, match="must be the default branch"):
                gateway.dispatch_sim(ref, _SIM_SHA)
        with pytest.raises(ActionDenied, match="farm/7-<slug>"):
            gateway.dispatch_sim("farm/8-restart", _SIM_SHA)
        assert proxy.calls == []
        assert database.actions_for_issue(event.repo, event.issue_number) == []
    finally:
        database.close()


def test_farm_ref_requires_exact_head_sha(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.MAINTAINER, proxy)
        with pytest.raises(ActionDenied, match="40-character HEAD SHA"):
            gateway.dispatch_sim(_SIM_FARM_REF, None)
        for sha in ("", "a" * 39, "A" * 40, "a" * 41, 123):
            with pytest.raises(ActionDenied, match="invalid simulator HEAD SHA"):
                gateway.dispatch_sim(_SIM_FARM_REF, sha)
        with pytest.raises(ActionDenied, match="invalid simulator HEAD SHA"):
            gateway.dispatch_sim("trunk", "short")
        assert proxy.calls == []
        assert database.actions_for_issue(event.repo, event.issue_number) == []
    finally:
        database.close()


def test_simulator_directive_allows_distinct_dispatches_and_denies_duplicates(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        control = json.loads(gateway.dispatch_sim("trunk", None))
        repro = json.loads(gateway.dispatch_sim(_SIM_FARM_REF, _SIM_SHA))
        assert control["state"] == "applied" and control["result"]["run_id"] == 99
        assert repro["state"] == "applied" and repro["result"]["run_id"] == 100
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="already recorded"):
            gateway.dispatch_sim(_SIM_FARM_REF, _SIM_SHA)
        assert proxy.calls == [
            ("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None),
            ("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", _SIM_FARM_REF, _SIM_SHA),
        ]
    finally:
        database.close()


def test_simulator_directive_comment_requires_dispatch_result_and_completed_run(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        with pytest.raises(ActionDenied, match="dispatch_sim then sim_result"):
            gateway.post_comment("done")
        gateway.dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="dispatch_sim then sim_result"):
            gateway.post_comment("done")
        proxy.sim_status = "in_progress"
        gateway.sim_result(99)
        with pytest.raises(ActionDenied, match="has not completed"):
            gateway.post_comment("done")
        proxy.sim_status = "completed"
        gateway.sim_result(99)
        result = json.loads(gateway.post_comment("done"))
        assert result["state"] == "applied"
        assert [call[0] for call in proxy.calls] == ["dispatch", "sim_result", "sim_result", "comment"]
    finally:
        database.close()


def test_pr_review_sim_directive_requires_dispatch_result_and_completed_run(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="pull_request_review.requested",
                comment="@roboserval Reproduce in the simulator",
            ),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        with pytest.raises(ActionDenied, match="dispatch_sim then sim_result"):
            gateway.post_comment("done")
        gateway.dispatch_sim("trunk", None)
        proxy.sim_status = "in_progress"
        gateway.sim_result(99)
        with pytest.raises(ActionDenied, match="has not completed"):
            gateway.post_comment("done")
        proxy.sim_status = "completed"
        gateway.sim_result(99)
        result = json.loads(gateway.post_comment("done"))
        assert result["state"] == "applied"
        assert [call[0] for call in proxy.calls] == ["dispatch", "sim_result", "sim_result", "comment"]
    finally:
        database.close()


def test_completed_dispatch_flow_survives_later_failed_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.dispatch_sim("trunk", None)
        gateway.sim_result(99)

        def exploding_dispatch(repo: str, issue_number: int, workflow: str, ref: str, head_sha: str | None) -> dict:
            proxy.calls.append(("dispatch", repo, issue_number, workflow, ref, head_sha))
            raise RuntimeError("dispatch exploded")

        proxy.dispatch_sim = exploding_dispatch
        with pytest.raises(RuntimeError, match="dispatch exploded"):
            gateway.dispatch_sim(_SIM_FARM_REF, _SIM_SHA)
        failed = [
            action
            for action in database.actions_for_delivery(event.delivery_id)
            if action.kind.startswith("dispatch_sim")
        ][-1]
        assert failed.state == "failed"
        result = json.loads(gateway.post_comment("run summary"))
        assert result["state"] == "applied"
        assert [call[0] for call in proxy.calls] == ["dispatch", "sim_result", "dispatch", "comment"]
    finally:
        database.close()


def test_failed_only_dispatch_satisfies_no_simulator_delivery(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval Reproduce in the simulator"),
        )
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)

        def exploding_dispatch(repo: str, issue_number: int, workflow: str, ref: str, head_sha: str | None) -> dict:
            proxy.calls.append(("dispatch", repo, issue_number, workflow, ref, head_sha))
            raise RuntimeError("dispatch exploded")

        proxy.dispatch_sim = exploding_dispatch
        with pytest.raises(RuntimeError, match="dispatch exploded"):
            gateway.dispatch_sim("trunk", None)
        with pytest.raises(ActionDenied, match="no recorded dispatch"):
            gateway.sim_result(99)
        with pytest.raises(ActionDenied, match="dispatch_sim then sim_result"):
            gateway.post_comment("done")
        assert proxy.calls == [("dispatch", "dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", None)]
        actions = database.actions_for_delivery(event.delivery_id)
        assert [action.kind for action in actions] == ["dispatch_sim:trunk:default"]
        assert actions[0].state == "failed"
    finally:
        database.close()


def test_ordinary_followup_comment_does_not_require_simulator_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval triage this"),
        )
        result = json.loads(_gateway(database, event, Mode.TRIAGE, proxy).post_comment("ok"))
        assert result["state"] == "applied"
        assert proxy.calls == [("comment", "dderg/serval", 7, "ok")]
    finally:
        database.close()


def test_unrelated_maintainer_comment_cannot_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval please triage this issue"),
        )
        with pytest.raises(ActionDenied, match="does not unambiguously"):
            _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        assert proxy.calls == []
    finally:
        database.close()


def test_negated_sim_request_cannot_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="issue_comment.created",
                comment="@roboserval do not reproduce in the simulator",
            ),
        )
        with pytest.raises(ActionDenied, match="does not unambiguously"):
            _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        assert proxy.calls == []
    finally:
        database.close()


def test_ambiguous_sim_question_cannot_dispatch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(
                actor="dderg",
                event_type="issue_comment.created",
                comment="@roboserval can you reproduce in the simulator?",
            ),
        )
        with pytest.raises(ActionDenied, match="does not unambiguously"):
            _gateway(database, event, Mode.TRIAGE, proxy).dispatch_sim("trunk", None)
        assert proxy.calls == []
    finally:
        database.close()


def test_sim_result_requires_directive_even_for_maintainer(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(
            database,
            _event(actor="dderg", event_type="issue_comment.created", comment="@roboserval what is the status?"),
        )
        with pytest.raises(ActionDenied, match="does not unambiguously"):
            _gateway(database, event, Mode.TRIAGE, proxy).sim_result(99)
        assert proxy.calls == []
    finally:
        database.close()


def test_parse_sim_directive_accepts_unambiguous_requests() -> None:
    for body in (
        "@roboserval Reproduce in the simulator",
        "@roboserval please run this in the simulator",
        "@roboserval dispatch the simulator",
        "@roboserval please dispatch the simulator",
        "@roboserval simulate this crash",
        "@roboserval start a simulator run of the attached model",
        "@roboserval reproduce in the simulator.",
        "@roboserval dispatch the simulator!",
    ):
        assert parse_sim_directive("roboserval", body), body


def test_parse_sim_directive_rejects_ambiguous_and_negated() -> None:
    for body in (
        "",
        "Reproduce in the simulator",
        "@serval-bot Reproduce in the simulator",
        "@roboserval do not reproduce in the simulator",
        "@roboserval don't run the simulator",
        "@roboserval never dispatch the simulator",
        "@roboserval skip the simulator this time",
        "@roboserval can you reproduce in the simulator?",
        "@roboserval what is the simulator status?",
        "@roboserval please triage this issue",
        "@roboserval thanks for the simulator run",
        "@roboserval maybe run the simulator",
        "@roboserval run in the simulator, not on hardware",
        "@roboserval reproduce it on the farm",
        "@roboserval check whether the simulator is up",
        "@roboserval only if the hardware is unavailable, run the simulator",
        "@roboserval run no simulator",
        "@roboserval run neither the simulator nor the farm",
        "@roboserval run anything but the simulator",
        "@roboserval run everything other than the simulator",
        "@roboserval ran the simulator",
        "@roboserval dispatched the simulator yesterday",
        "@roboserval acknowledged the simulator run",
        "@roboserval triage this.\nRun the simulator.",
        "@roboserval run the simulator.\nActually, never mind.",
        "@roboserval reproduce in the simulator.\nCancel that.",
        "@roboserval run the simulator.\nStop the simulator run.",
        "@roboserval run the simulator.\nDon't, run the farm instead.",
        "@roboserval run everything except the simulator",
        "@roboserval run the simulator except on the farm",
        "@roboserval rerun the simulator for the failing G-code",
        "@roboserval reproduce in the simulator.\nAlso attach the events.",
        "Reproduce in the simulator, @roboserval",
        "@roboserval: run the simulator",
        "@roboserval run the simulator",
        "@roboserval run the simulator and the farm",
        "@roboserval simulate this crash please",
        "@roboserval reproduce in the simulator now",
        "@roboserval start a simulator run",
        "@roboserval reproduce in the simulator..",
    ):
        assert not parse_sim_directive("roboserval", body), body


def test_database_action_conflict_is_atomic(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _claimed(database, _event())
        gateway = _gateway(database, event, Mode.TRIAGE, proxy)
        gateway.classify("bug", None, ["host"], "reason")
        gateway.post_comment("first")
        with pytest.raises(ActionConflict, match="already recorded"):
            database.add_action(event, "comment", {"body": "second"}, "proposed")
        assert [call for call in proxy.calls if call[0] in {"labels", "comment"}] == [
            ("labels", "dderg/serval", 7, ["bug", "area:host"]),
            ("comment", "dderg/serval", 7, "first"),
        ]
    finally:
        database.close()


def test_migration_accepts_existing_valid_database(tmp_path: Path) -> None:
    path = tmp_path / "bot.sqlite"
    database = Database(path)
    try:
        event = _claimed(database, _event())
        database.add_action(event, "classify", {"primary": "bug"}, "applied")
        database.add_action(event, "comment", {"body": "hi"}, "applied")
    finally:
        database.close()
    reopened = Database(path)
    try:
        kinds = [action.kind for action in reopened.actions_for_delivery("delivery")]
        assert kinds == ["classify", "comment"]
    finally:
        reopened.close()


def test_migration_fails_loudly_on_conflicting_duplicates(tmp_path: Path) -> None:
    path = tmp_path / "bot.sqlite"
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE events (
            delivery_id TEXT PRIMARY KEY,
            event_type TEXT NOT NULL,
            repo TEXT NOT NULL,
            issue_number INTEGER NOT NULL,
            actor TEXT NOT NULL,
            payload TEXT NOT NULL,
            state TEXT NOT NULL DEFAULT 'queued',
            attempts INTEGER NOT NULL DEFAULT 0,
            error TEXT,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        );
        CREATE TABLE actions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            delivery_id TEXT NOT NULL REFERENCES events(delivery_id),
            repo TEXT NOT NULL,
            issue_number INTEGER NOT NULL,
            kind TEXT NOT NULL,
            arguments TEXT NOT NULL,
            state TEXT NOT NULL,
            result TEXT,
            created_at TEXT NOT NULL
        );
        """
    )
    connection.execute(
        "INSERT INTO events (delivery_id, event_type, repo, issue_number, actor, payload, created_at, updated_at) "
        "VALUES ('delivery', 'issues.opened', 'dderg/serval', 7, 'reporter', '{}', 't', 't')"
    )
    connection.execute(
        "INSERT INTO actions (delivery_id, repo, issue_number, kind, arguments, state, created_at) "
        "VALUES ('delivery', 'dderg/serval', 7, 'comment', '{}', 'applied', 't')"
    )
    connection.execute(
        "INSERT INTO actions (delivery_id, repo, issue_number, kind, arguments, state, created_at) "
        "VALUES ('delivery', 'dderg/serval', 7, 'comment', '{}', 'applied', 't')"
    )
    connection.commit()
    connection.close()
    with pytest.raises(RuntimeError, match="duplicate actions"):
        Database(path)
