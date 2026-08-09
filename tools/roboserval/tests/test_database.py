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
