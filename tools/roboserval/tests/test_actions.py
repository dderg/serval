import json
from pathlib import Path

import pytest

from serval_bot.actions import ActionDenied, ActionGateway
from serval_bot.database import Database, Event
from serval_bot.policy import Mode, RepositoryPolicy


class FakeProxy:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def add_labels(self, repo: str, issue_number: int, labels: list[str]) -> dict:
        self.calls.append(("labels", repo, issue_number, labels))
        return {"labels": labels}

    def post_comment(self, repo: str, issue_number: int, body: str) -> dict:
        self.calls.append(("comment", repo, issue_number, body))
        return {"id": 1, "url": "https://example.test/comment"}

    def dispatch_sim(self, repo: str, workflow: str, ref: str, head_sha: str | None) -> dict:
        self.calls.append(("dispatch", repo, workflow, ref, head_sha))
        return {
            "run_id": 99,
            "url": "https://example.test/run/99",
            "status": "queued",
            "conclusion": None,
        }


def _event(actor: str = "reporter") -> Event:
    return Event("delivery", "issues.opened", "dderg/serval", 7, actor, {}, "running", 1, None)


def _policy(mode: Mode) -> RepositoryPolicy:
    return RepositoryPolicy(
        repo="dderg/serval",
        mode=mode,
        bot_login="serval-bot",
        maintainers=frozenset({"dderg"}),
        sim_workflow="ci-sim-e2e.yaml",
    )


def test_shadow_mode_records_without_github_side_effect(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _event()
        database.record_event(event.delivery_id, event.event_type, event.repo, event.issue_number, event.actor, {})
        claimed = database.claim()
        assert claimed is not None
        result = json.loads(
            ActionGateway(database, claimed, _policy(Mode.SHADOW), "trunk", proxy).classify(
                "bug", "p2", ["host"], "reproducible host failure"
            )
        )
        assert result["state"] == "proposed"
        assert proxy.calls == []
        assert database.actions_for_issue(event.repo, event.issue_number)[0].state == "proposed"
    finally:
        database.close()


def test_triage_mode_applies_comment(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _event()
        database.record_event(event.delivery_id, event.event_type, event.repo, event.issue_number, event.actor, {})
        claimed = database.claim()
        assert claimed is not None
        result = json.loads(
            ActionGateway(database, claimed, _policy(Mode.TRIAGE), "trunk", proxy).post_comment("Attach logs")
        )
        assert result["state"] == "applied"
        assert proxy.calls == [("comment", "dderg/serval", 7, "Attach logs")]
        assert database.actions_for_issue(event.repo, event.issue_number)[0].state == "applied"
    finally:
        database.close()


def test_nonmaintainer_cannot_dispatch_simulator(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        with pytest.raises(ActionDenied, match="not authorized"):
            ActionGateway(database, _event(), _policy(Mode.MAINTAINER), "trunk", FakeProxy()).dispatch_sim(
                "trunk", None
            )
    finally:
        database.close()


def test_maintainer_dispatch_is_recorded(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    proxy = FakeProxy()
    try:
        event = _event("dderg")
        database.record_event(event.delivery_id, event.event_type, event.repo, event.issue_number, event.actor, {})
        claimed = database.claim()
        assert claimed is not None
        result = json.loads(
            ActionGateway(database, claimed, _policy(Mode.MAINTAINER), "trunk", proxy).dispatch_sim("trunk", None)
        )
        assert result["state"] == "applied"
        assert result["result"]["run_id"] == 99
    finally:
        database.close()


def test_maintainer_cannot_dispatch_obsolete_default_branch(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        with pytest.raises(ActionDenied, match="outside the bot namespace"):
            ActionGateway(database, _event("dderg"), _policy(Mode.MAINTAINER), "trunk", FakeProxy()).dispatch_sim(
                "sota-motion", None
            )
    finally:
        database.close()
