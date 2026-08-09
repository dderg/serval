from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import httpx
import pytest

from serval_bot.config import BotSettings
from serval_bot.database import Database, Event
from serval_bot.policy import Mode, PolicySet, RepositoryPolicy
from serval_bot.server import Poller, Worker, create_app


class FakeAgent:
    def run(self, event: Event) -> str:
        return event.delivery_id


class FakePollSource:
    def __init__(self, events: list[dict[str, Any]]):
        self.events = events
        self.calls: list[tuple[str, str, str]] = []

    def poll_events(self, repo: str, since: str, bot_login: str) -> dict[str, Any]:
        self.calls.append((repo, since, bot_login))
        return {"events": self.events}


class FailingPollSource:
    def poll_events(self, repo: str, since: str, bot_login: str) -> dict[str, Any]:
        raise RuntimeError(f"poll failed for {repo} since {since} as {bot_login}")


def _settings(tmp_path: Path) -> BotSettings:
    return BotSettings(
        proxy_url=None,
        proxy_hmac_key=None,
        policy_path=tmp_path / "policy.toml",
        data_dir=tmp_path,
        model="test/model",
        provider=None,
        thinking="off",
        omp_command=("omp",),
        bind_host="127.0.0.1",
        bind_port=8080,
        task_timeout_seconds=10,
        poll_interval_seconds=30,
        poll_overlap_seconds=300,
    )


def _policies() -> PolicySet:
    return PolicySet(
        {
            "dderg/serval": RepositoryPolicy(
                "dderg/serval",
                Mode.SHADOW,
                "roboserval",
                frozenset({"dderg"}),
                "ci-sim-e2e.yaml",
            )
        }
    )


def _event() -> dict[str, Any]:
    return {
        "delivery_id": "poll:comment:41:created",
        "event_type": "issue_comment.created",
        "issue_number": 7,
        "actor": "reporter",
        "occurred_at": "2026-08-09T12:02:00Z",
        "payload": {
            "action": "created",
            "repository": {"full_name": "dderg/serval"},
            "sender": {"login": "reporter"},
            "issue": {"number": 7, "title": "failure", "body": "details"},
            "comment": {"body": "@roboserval investigate"},
        },
    }


@pytest.mark.asyncio
async def test_poller_initializes_cursor_without_replaying_history(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    source = FakePollSource([])
    worker = Worker(database, FakeAgent(), 10)
    poller = Poller(database, _policies(), source, worker, 30, 300)
    now = datetime(2026, 8, 9, 12, 0, tzinfo=UTC)
    try:
        assert await poller.poll_once(now) == 0
        assert database.poll_cursor("dderg/serval") == now.isoformat()
        assert source.calls == []
    finally:
        database.close()


@pytest.mark.asyncio
async def test_poller_persists_before_advancing_and_deduplicates_overlap(tmp_path: Path) -> None:
    path = tmp_path / "bot.sqlite"
    database = Database(path)
    source = FakePollSource([_event()])
    worker = Worker(database, FakeAgent(), 10)
    poller = Poller(database, _policies(), source, worker, 30, 300)
    previous = datetime(2026, 8, 9, 12, 0, tzinfo=UTC)
    first = datetime(2026, 8, 9, 12, 5, tzinfo=UTC)
    second = datetime(2026, 8, 9, 12, 10, tzinfo=UTC)
    database.record_poll_batch("dderg/serval", previous.isoformat(), [])
    try:
        assert await poller.poll_once(first) == 1
        assert await poller.poll_once(second) == 0
        event = database.claim()
        assert event is not None
        assert event.delivery_id == "poll:comment:41:created"
        assert database.claim() is None
    finally:
        database.close()

    reopened = Database(path)
    try:
        assert reopened.poll_cursor("dderg/serval") == second.isoformat()
        assert reopened.recent_events()[0]["delivery_id"] == "poll:comment:41:created"
    finally:
        reopened.close()


@pytest.mark.asyncio
async def test_poller_does_not_advance_cursor_after_failure(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    previous = datetime(2026, 8, 9, 12, 0, tzinfo=UTC)
    database.record_poll_batch("dderg/serval", previous.isoformat(), [])
    poller = Poller(database, _policies(), FailingPollSource(), Worker(database, FakeAgent(), 10), 30, 300)
    try:
        with pytest.raises(RuntimeError, match="poll failed"):
            await poller.poll_once(datetime(2026, 8, 9, 12, 5, tzinfo=UTC))
        assert database.poll_cursor("dderg/serval") == previous.isoformat()
    finally:
        database.close()


@pytest.mark.asyncio
async def test_webhook_ingress_is_removed(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    app = create_app(
        _settings(tmp_path),
        _policies(),
        database,
        FakeAgent(),
        start_worker=False,
        start_poller=False,
    )
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post("/webhook/github", json={})
        assert response.status_code == 404
    finally:
        database.close()
