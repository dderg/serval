from pathlib import Path

from serval_bot.database import Database


def _payload() -> dict:
    return {"issue": {"number": 7, "title": "failure"}}


def test_event_delivery_is_deduplicated_and_claimed(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        assert database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        assert not database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        event = database.claim()
        assert event is not None
        assert event.state == "running"
        assert event.attempts == 1
        database.finish("delivery", "done")
        assert database.claim() is None
    finally:
        database.close()


def test_running_event_resumes_after_restart(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        assert database.claim() is not None
        assert database.reset_running() == 1
        resumed = database.claim()
        assert resumed is not None
        assert resumed.attempts == 2
    finally:
        database.close()


def test_delayed_retry_waits_and_preserves_attempts(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        event = database.claim()
        assert event is not None
        assert database.schedule_retry("delivery", 60, "temporary failure")
        assert database.claim() is None
        row = database.recent_events(1)[0]
        assert row["state"] == "queued"
        assert row["attempts"] == 1
        assert row["error"] == "temporary failure"
        assert row["available_at"] is not None
    finally:
        database.close()


def test_failed_event_can_be_replayed_once(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        assert database.claim() is not None
        database.finish("delivery", "failed", "credential expired")
        assert database.replay("delivery")
        assert not database.replay("delivery")
        replayed = database.claim()
        assert replayed is not None
        assert replayed.attempts == 2
        assert replayed.error is None
    finally:
        database.close()


def test_action_records_proposed_arguments(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        event = database.claim()
        assert event is not None
        action_id = database.add_action(event, "comment", {"body": "need logs"}, "proposed")
        actions = database.actions_for_issue("dderg/serval", 7)
        assert [action.id for action in actions] == [action_id]
        assert actions[0].arguments == {"body": "need logs"}
        assert actions[0].state == "proposed"
    finally:
        database.close()


def test_workflow_run_claim_is_atomic_and_never_reused(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        assert database.claim_workflow_run("dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", 99, "u", "queued", None)
        assert not database.claim_workflow_run("dderg/serval", 8, "ci-sim-e2e.yaml", "trunk", 99, "u", "queued", None)
        assert not database.claim_workflow_run("dderg/serval", 7, "ci-sim-e2e.yaml", "trunk", 99, "u", "queued", None)
        recorded = database.workflow_run("dderg/serval", 7, 99)
        assert recorded is not None
        assert (recorded["run_id"], recorded["workflow"], recorded["ref"]) == (99, "ci-sim-e2e.yaml", "trunk")
        assert database.workflow_run("dderg/serval", 8, 99) is None
        database.update_workflow_run_status(99, "completed", "success")
        assert database.workflow_run("dderg/serval", 7, 99)["status"] == "completed"
    finally:
        database.close()
