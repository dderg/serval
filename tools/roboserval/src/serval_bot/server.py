from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from datetime import UTC, datetime, timedelta
from typing import Any, Protocol

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

from serval_bot.config import BotSettings
from serval_bot.database import Database, PolledEvent
from serval_bot.policy import PolicySet
from serval_bot.proxy_client import ProxyClient
from serval_bot.runtime import Agent, WorkerPool

log = logging.getLogger(__name__)


class PollSource(Protocol):
    def poll_events(self, repo: str, since: str, bot_login: str) -> dict[str, Any]: ...


class Poller:
    def __init__(
        self,
        database: Database,
        policies: PolicySet,
        source: PollSource,
        worker: WorkerPool,
        interval_seconds: int,
        overlap_seconds: int,
    ):
        self._database = database
        self._policies = policies
        self._source = source
        self._worker = worker
        self._interval_seconds = interval_seconds
        self._overlap = timedelta(seconds=overlap_seconds)
        self._stop = asyncio.Event()

    def stop(self) -> None:
        self._stop.set()

    async def run(self) -> None:
        while not self._stop.is_set():
            await self.poll_once()
            with suppress(TimeoutError):
                await asyncio.wait_for(self._stop.wait(), timeout=self._interval_seconds)

    async def poll_once(self, now: datetime | None = None) -> int:
        started = now or datetime.now(UTC)
        inserted = 0
        for policy in self._policies.repositories.values():
            cursor = self._database.poll_cursor(policy.repo)
            if cursor is None:
                self._database.record_poll_batch(policy.repo, started.isoformat(), [])
                continue
            since = datetime.fromisoformat(cursor) - self._overlap
            result = await asyncio.to_thread(
                self._source.poll_events,
                policy.repo,
                since.isoformat(),
                policy.bot_login,
            )
            events = _polled_events(result)
            inserted += self._database.record_poll_batch(policy.repo, started.isoformat(), events)
        if inserted:
            self._worker.wake()
        return inserted


def _polled_events(result: dict[str, Any]) -> list[PolledEvent]:
    raw_events = result.get("events")
    if not isinstance(raw_events, list):
        raise TypeError("poll response has no events list")
    events: list[PolledEvent] = []
    for item in raw_events:
        if not isinstance(item, dict):
            raise TypeError("poll response event is not an object")
        delivery_id = item.get("delivery_id")
        event_type = item.get("event_type")
        issue_number = item.get("issue_number")
        actor = item.get("actor")
        occurred_at = item.get("occurred_at")
        payload = item.get("payload")
        if (
            not isinstance(delivery_id, str)
            or event_type not in {"issues.opened", "issue_comment.created", "pull_request_review.requested"}
            or not isinstance(issue_number, int)
            or issue_number <= 0
            or not isinstance(actor, str)
            or not actor
            or not isinstance(occurred_at, str)
            or not isinstance(payload, dict)
        ):
            raise RuntimeError(f"invalid poll response event: {item!r}")
        parsed_time = datetime.fromisoformat(occurred_at)
        if parsed_time.tzinfo is None:
            raise RuntimeError(f"poll response event time has no timezone: {occurred_at}")
        events.append(PolledEvent(delivery_id, event_type, issue_number, actor, occurred_at, payload))
    return events


def create_app(
    settings: BotSettings,
    policies: PolicySet,
    database: Database,
    agent: Agent,
    proxy: ProxyClient | None = None,
    *,
    start_worker: bool = True,
    start_poller: bool = True,
) -> FastAPI:
    worker = WorkerPool(
        database,
        agent,
        timeout_seconds=settings.task_timeout_seconds,
        hard_grace_seconds=settings.task_hard_grace_seconds,
        max_concurrency=settings.max_concurrency,
    )
    if start_poller and proxy is None:
        raise RuntimeError("GitHub proxy is required when polling is enabled")
    poller = (
        Poller(
            database,
            policies,
            proxy,
            worker,
            settings.poll_interval_seconds,
            settings.poll_overlap_seconds,
        )
        if proxy is not None
        else None
    )

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        async with asyncio.TaskGroup() as tasks:
            if start_worker:
                tasks.create_task(worker.run())
            if start_poller and poller is not None:
                tasks.create_task(poller.run())
            try:
                yield
            finally:
                if poller is not None:
                    poller.stop()
                worker.stop()

    app = FastAPI(lifespan=lifespan)

    @app.get("/healthz")
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/readyz")
    async def ready() -> dict[str, str]:
        return {"status": "ready"}

    @app.get("/events")
    async def events(limit: int = 100) -> list[dict[str, Any]]:
        return database.recent_events(max(1, min(limit, 500)))

    @app.get("/actions/{owner}/{repository}/{issue_number}")
    async def actions(owner: str, repository: str, issue_number: int) -> list[dict[str, Any]]:
        return [
            {
                "id": action.id,
                "kind": action.kind,
                "arguments": action.arguments,
                "state": action.state,
                "result": action.result,
            }
            for action in database.actions_for_issue(f"{owner}/{repository}", issue_number)
        ]

    @app.get("/", response_class=HTMLResponse)
    async def dashboard() -> str:
        return _DASHBOARD

    return app


_DASHBOARD = """<!doctype html>
<html><head><meta charset="utf-8"><title>Serval Bot</title>
<style>
body{font:14px system-ui;margin:2rem;max-width:1100px}
pre{white-space:pre-wrap;background:#f4f4f4;padding:1rem}
</style>
</head><body><h1>Serval Bot</h1><p>Recent durable GitHub events.</p><pre id="events">loading</pre>
<script>
fetch('/events').then(r=>r.json()).then(v=>events.textContent=JSON.stringify(v,null,2))
</script></body></html>"""
