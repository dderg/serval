from __future__ import annotations

import asyncio
import os
import shutil
import signal
import stat
import threading
from collections.abc import Callable
from contextlib import suppress
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import httpx
import pytest

import serval_bot.runtime as runtime_module
from serval_bot.config import BotSettings
from serval_bot.database import Database, Event
from serval_bot.policy import Mode, PolicySet, RepositoryPolicy
from serval_bot.runtime import (
    FIRST_SLOT_UID,
    HardGraceExceeded,
    SlotReapError,
    WorkerPool,
    reap_slot,
    slot_pids,
    slot_uids,
)
from serval_bot.server import Poller, create_app


class FakeAgent:
    def __init__(
        self,
        *,
        result: str | None = None,
        error: Exception | None = None,
        blocking: set[str] | None = None,
    ) -> None:
        self.result = result
        self.error = error
        self.blocking = blocking or set()
        self.runs: list[tuple[str, int | None]] = []
        self.stops: list[str] = []
        self.run_ended = threading.Event()
        self._release: dict[str, threading.Event] = {}

    def run(self, event: Event, slot_uid: int | None = None) -> str:
        self.runs.append((event.delivery_id, slot_uid))
        if self.error is not None:
            raise self.error
        if event.delivery_id in self.blocking and not self._release_for(event.delivery_id).wait(10):
            raise AssertionError(f"run for {event.delivery_id} was not released")
        self.run_ended.set()
        return self.result or event.delivery_id

    def stop(self, delivery_id: str) -> None:
        self.stops.append(delivery_id)
        self._release_for(delivery_id).set()

    def release(self, delivery_id: str) -> None:
        self._release_for(delivery_id).set()

    def _release_for(self, delivery_id: str) -> threading.Event:
        return self._release.setdefault(delivery_id, threading.Event())


class BarrierAgent:
    def __init__(self, parties: int) -> None:
        self.barrier = threading.Barrier(parties)
        self.started: list[str] = []
        self.slots: dict[str, int | None] = {}

    def run(self, event: Event, slot_uid: int | None = None) -> str:
        self.started.append(event.delivery_id)
        self.slots[event.delivery_id] = slot_uid
        self.barrier.wait(timeout=5)
        return event.delivery_id

    def stop(self, delivery_id: str) -> None:
        return None


class DrainingAgent:
    """First event blocks until stop; the second must not start before the first drained."""

    def __init__(self) -> None:
        self.first_started = threading.Event()
        self.first_ended = threading.Event()
        self.second_started = threading.Event()
        self.stop_calls: list[str] = []
        self.violated = False

    def run(self, event: Event, slot_uid: int | None = None) -> str:
        if event.delivery_id == "first":
            self.first_started.set()
            self.first_ended.wait(10)
            return "first"
        if not self.first_ended.is_set():
            self.violated = True
        self.second_started.set()
        return "second"

    def stop(self, delivery_id: str) -> None:
        self.stop_calls.append(delivery_id)
        self.first_ended.set()


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
        policy_toml='[repositories."dderg/serval"]',
        data_dir=tmp_path,
        model="test/model",
        provider=None,
        thinking="off",
        omp_command=("omp",),
        bind_host="127.0.0.1",
        bind_port=8080,
        task_timeout_seconds=10,
        task_hard_grace_seconds=60,
        max_concurrency=1,
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


def _pool(database: Database, agent: Any, **kwargs: Any) -> WorkerPool:
    return WorkerPool(
        database,
        agent,
        timeout_seconds=kwargs.pop("timeout_seconds", 10),
        hard_grace_seconds=kwargs.pop("hard_grace_seconds", 5),
        max_concurrency=kwargs.pop("max_concurrency", 1),
        max_retries=kwargs.pop("max_retries", 0),
        retry_delay_seconds=kwargs.pop("retry_delay_seconds", None),
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


def _payload() -> dict[str, Any]:
    return {"issue": {"number": 7, "title": "failure"}}


def _event_row(database: Database, delivery_id: str) -> dict[str, Any]:
    rows = {row["delivery_id"]: row for row in database.recent_events(500)}
    assert delivery_id in rows
    return rows[delivery_id]


async def _wait_until(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.01)
    raise AssertionError("condition was not met in time")


@pytest.mark.asyncio
async def test_poller_initializes_cursor_without_replaying_history(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    source = FakePollSource([])
    poller = Poller(database, _policies(), source, _pool(database, FakeAgent()), 30, 300)
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
    poller = Poller(database, _policies(), source, _pool(database, FakeAgent()), 30, 300)
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
    poller = Poller(database, _policies(), FailingPollSource(), _pool(database, FakeAgent()), 30, 300)
    try:
        with pytest.raises(RuntimeError, match="poll failed"):
            await poller.poll_once(datetime(2026, 8, 9, 12, 5, tzinfo=UTC))
        assert database.poll_cursor("dderg/serval") == previous.isoformat()
    finally:
        database.close()


def test_claim_excludes_queued_events_for_running_issue(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
        database.record_event("second", "issue_comment.created", "dderg/serval", 7, "reporter", _payload())
        database.record_event("other", "issues.opened", "dderg/serval", 8, "reporter", _payload())
        first = database.claim()
        assert first is not None
        assert first.delivery_id == "first"
        other = database.claim()
        assert other is not None
        assert other.delivery_id == "other"
        assert database.claim() is None
        database.finish("first", "done")
        second = database.claim()
        assert second is not None
        assert second.delivery_id == "second"
    finally:
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_success_marks_done_with_slot(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent()
    pool = _pool(database, agent)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: _event_row(database, "first")["state"] == "done")
        assert agent.runs == [("first", FIRST_SLOT_UID)]
        assert agent.stops == []
        assert agent.run_ended.is_set()
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_failure_marks_failed_before_next_claim(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent(error=RuntimeError("boom"))
    pool = _pool(database, agent)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: _event_row(database, "first")["state"] == "failed")
        row = _event_row(database, "first")
        assert "RuntimeError: boom" in row["error"]
        assert agent.stops == []
        assert len(agent.runs) == 1
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_retries_with_bounded_schedule(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent(error=RuntimeError("temporary"))
    delays: list[int] = []

    def retry_delay(attempt: int) -> float:
        delays.append(attempt)
        return 0

    pool = _pool(database, agent, max_retries=3, retry_delay_seconds=retry_delay)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: _event_row(database, "first")["state"] == "failed")
        row = _event_row(database, "first")
        assert row["attempts"] == 4
        assert "RuntimeError: temporary" in row["error"]
        assert delays == [1, 2, 3]
        assert len(agent.runs) == 4
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_timeout_stops_drains_before_next_claim(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = DrainingAgent()
    pool = _pool(database, agent, timeout_seconds=1)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    database.record_event("second", "issues.opened", "dderg/serval", 8, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: agent.first_started.is_set())
        await _wait_until(lambda: _event_row(database, "first")["state"] == "failed")
        assert agent.stop_calls == ["first"]
        assert "deadline" in _event_row(database, "first")["error"]
        assert agent.first_ended.is_set()
        await _wait_until(lambda: agent.second_started.is_set())
        await _wait_until(lambda: _event_row(database, "second")["state"] == "done")
        assert not agent.violated
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_shutdown_stops_drains_and_leaves_event_running(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent(blocking={"first"})
    pool = _pool(database, agent, timeout_seconds=30)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: len(agent.runs) == 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, timeout=10)
        assert agent.stops == ["first"]
        assert agent.run_ended.is_set()
        assert _event_row(database, "first")["state"] == "running"
        assert database.reset_running() == 1
    finally:
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_serializes_same_issue_events(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent(blocking={"first"})
    pool = _pool(database, agent, max_concurrency=2)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    database.record_event("second", "issue_comment.created", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: len(agent.runs) == 1)
        await asyncio.sleep(0.2)
        assert [delivery for delivery, _ in agent.runs] == ["first"]
        agent.release("first")
        await _wait_until(lambda: _event_row(database, "second")["state"] == "done")
        assert [delivery for delivery, _ in agent.runs] == ["first", "second"]
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


@pytest.mark.asyncio
async def test_private_replay_endpoint_requeues_failed_event(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    assert database.claim() is not None
    database.finish("delivery", "failed", "temporary")
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
            response = await client.post("/replay/delivery")
            duplicate = await client.post("/replay/delivery")
        assert response.status_code == 200
        assert response.json() == {"delivery_id": "delivery", "state": "queued"}
        assert duplicate.status_code == 409
    finally:
        database.close()


@pytest.mark.asyncio
async def test_replay_endpoint_rejects_non_loopback_client(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    database.record_event("delivery", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    assert database.claim() is not None
    database.finish("delivery", "failed", "temporary")
    app = create_app(
        _settings(tmp_path),
        _policies(),
        database,
        FakeAgent(),
        start_worker=False,
        start_poller=False,
    )
    try:
        transport = httpx.ASGITransport(app=app, client=("203.0.113.5", 4123))
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            response = await client.post("/replay/delivery")
        assert response.status_code == 403
        assert _event_row(database, "delivery")["state"] == "failed"
    finally:
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_runs_independent_issues_concurrently(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = BarrierAgent(2)
    pool = _pool(database, agent, max_concurrency=2)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    database.record_event("second", "issues.opened", "dderg/serval", 8, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(
            lambda: (
                _event_row(database, "first")["state"] == "done" and _event_row(database, "second")["state"] == "done"
            )
        )
        assert sorted(agent.started) == ["first", "second"]
        assert set(agent.slots.values()) == set(slot_uids(2))
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
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


@pytest.mark.asyncio
async def test_poller_failure_signals_termination_and_unreadies(tmp_path: Path, monkeypatch: Any) -> None:
    database = Database(tmp_path / "bot.sqlite")
    database.record_poll_batch("dderg/serval", "2026-08-09T12:00:00Z", [])
    app = create_app(
        _settings(tmp_path),
        _policies(),
        database,
        FakeAgent(),
        proxy=FailingPollSource(),
        start_worker=True,
        start_poller=True,
    )
    killed: list[tuple[int, int]] = []
    monkeypatch.setattr("serval_bot.server.os.kill", lambda pid, sig: killed.append((pid, sig)))
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            async with app.router.lifespan_context(app):
                await asyncio.sleep(3600)
        assert caught.value.subgroup(lambda exc: isinstance(exc, RuntimeError) and "poll failed" in str(exc))
        await _wait_until(lambda: bool(killed))
        assert killed == [(os.getpid(), signal.SIGTERM)]
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            response = await client.get("/readyz")
        assert response.status_code == 503
    finally:
        database.close()


@pytest.mark.asyncio
async def test_worker_failure_signals_termination_and_unreadies(tmp_path: Path, monkeypatch: Any) -> None:
    database = Database(tmp_path / "bot.sqlite")
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    monkeypatch.setattr(
        database,
        "claim",
        lambda: (_ for _ in ()).throw(RuntimeError("claim failed")),
    )
    app = create_app(
        _settings(tmp_path),
        _policies(),
        database,
        FakeAgent(),
        start_worker=True,
        start_poller=False,
    )
    killed: list[tuple[int, int]] = []
    monkeypatch.setattr("serval_bot.server.os.kill", lambda pid, sig: killed.append((pid, sig)))
    try:
        with pytest.raises(BaseExceptionGroup) as caught:
            async with app.router.lifespan_context(app):
                await asyncio.sleep(3600)
        assert caught.value.subgroup(lambda exc: isinstance(exc, RuntimeError) and "claim failed" in str(exc))
        await _wait_until(lambda: bool(killed))
        assert killed == [(os.getpid(), signal.SIGTERM)]
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            response = await client.get("/readyz")
        assert response.status_code == 503
    finally:
        database.close()


def _proc_status(proc_root: Path, pid: int, *, uid: int = FIRST_SLOT_UID, state: str = "R") -> None:
    status = proc_root / str(pid) / "status"
    status.parent.mkdir(parents=True, exist_ok=True)
    status.write_text(f"State:\t{state} (running)\nUid:\t{uid} {uid} {uid} {uid}\n")


def _active_slot_permissions(monkeypatch: Any) -> None:
    monkeypatch.setattr(runtime_module, "slot_permissions_active", lambda slot_uid: True)


def test_slot_pids_returns_slot_owned_non_zombie_pids(tmp_path: Path) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    _proc_status(proc_root, 1002, state="Z")
    _proc_status(proc_root, 1003, uid=0)
    assert slot_pids(FIRST_SLOT_UID, proc_root) == (1001,)


def test_slot_pids_unparseable_uid_is_fatal(tmp_path: Path) -> None:
    proc_root = tmp_path / "proc"
    status = proc_root / "1001" / "status"
    status.parent.mkdir(parents=True)
    status.write_text("State:\tR (running)\nUid:\tgarbage\n")
    with pytest.raises(SlotReapError, match="unparseable ownership"):
        slot_pids(FIRST_SLOT_UID, proc_root)


def test_reap_slot_kills_slot_processes_and_verifies_empty(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    _proc_status(proc_root, 1002)
    _proc_status(proc_root, 1003, uid=0)
    _proc_status(proc_root, 1004, state="Z")
    _active_slot_permissions(monkeypatch)
    killed: list[int] = []

    def fake_kill(pid: int, sig: int) -> None:
        killed.append(pid)
        shutil.rmtree(proc_root / str(pid))

    monkeypatch.setattr(runtime_module.os, "kill", fake_kill)
    assert reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root) == 2
    assert sorted(killed) == [1001, 1002]


def test_reap_slot_rescans_forked_descendants_until_empty(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    _active_slot_permissions(monkeypatch)
    killed: list[int] = []

    def fake_kill(pid: int, sig: int) -> None:
        killed.append(pid)
        shutil.rmtree(proc_root / str(pid))
        if pid == 1001:
            _proc_status(proc_root, 1002)

    monkeypatch.setattr(runtime_module.os, "kill", fake_kill)
    assert reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root) == 2
    assert killed == [1001, 1002]


def test_reap_slot_residual_process_at_deadline_is_fatal(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    _active_slot_permissions(monkeypatch)
    monkeypatch.setattr(runtime_module.os, "kill", lambda pid, sig: None)
    with pytest.raises(SlotReapError, match="1001"):
        reap_slot(FIRST_SLOT_UID, deadline_seconds=0.05, proc_root=proc_root)


def test_reap_slot_kill_error_is_fatal(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    _active_slot_permissions(monkeypatch)

    def deny_kill(pid: int, sig: int) -> None:
        raise PermissionError("denied")

    monkeypatch.setattr(runtime_module.os, "kill", deny_kill)
    with pytest.raises(SlotReapError, match="failed to kill"):
        reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root)


def test_reap_slot_scan_error_is_fatal(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    proc_root.write_text("not a directory")
    _active_slot_permissions(monkeypatch)
    with pytest.raises(SlotReapError, match="failed to scan"):
        reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root)


def test_reap_slot_unreadable_status_is_fatal(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    (proc_root / "1001").mkdir(parents=True)
    (proc_root / "1001" / "status").mkdir()
    _active_slot_permissions(monkeypatch)
    with pytest.raises(SlotReapError, match="failed to read"):
        reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root)


def test_reap_slot_is_noop_without_slot_permissions(tmp_path: Path, monkeypatch: Any) -> None:
    proc_root = tmp_path / "proc"
    _proc_status(proc_root, 1001)
    monkeypatch.setattr(runtime_module, "slot_permissions_active", lambda slot_uid: False)
    monkeypatch.setattr(runtime_module.os, "kill", lambda pid, sig: pytest.fail("kill called without slot permissions"))
    assert reap_slot(FIRST_SLOT_UID, deadline_seconds=1.0, proc_root=proc_root) == 0


class UndrainedToolAgent:
    """run() blocks like an in-flight host tool that never completes."""

    def __init__(self) -> None:
        self.runs: list[str] = []
        self.started = threading.Event()

    def run(self, event: Event, slot_uid: int | None = None) -> str:
        self.runs.append(event.delivery_id)
        self.started.set()
        threading.Event().wait(60)
        return event.delivery_id

    def stop(self, delivery_id: str) -> None:
        return None


@pytest.mark.asyncio
async def test_worker_pool_undrained_run_withholds_slot_and_fails_loudly(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = UndrainedToolAgent()
    pool = _pool(database, agent, timeout_seconds=1, hard_grace_seconds=1)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    database.record_event("second", "issues.opened", "dderg/serval", 8, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: agent.started.is_set())
        with pytest.raises(HardGraceExceeded):
            await asyncio.wait_for(task, timeout=10)
        assert _event_row(database, "first")["state"] == "running"
        assert _event_row(database, "second")["state"] == "queued"
        assert database.reset_running() == 1
    finally:
        if not task.done():
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_reap_failure_withholds_slot_and_fails_loudly(tmp_path: Path, monkeypatch: Any) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent()
    calls = {"count": 0}

    def flaky_reap(slot_uid: int | None) -> int:
        calls["count"] += 1
        if calls["count"] > 1:
            raise SlotReapError(f"slot user {slot_uid} still owns processes")
        return 0

    monkeypatch.setattr(runtime_module, "reap_slot", flaky_reap)
    pool = _pool(database, agent)
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    database.record_event("second", "issues.opened", "dderg/serval", 8, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: _event_row(database, "first")["state"] == "done")
        with pytest.raises(SlotReapError):
            await asyncio.wait_for(task, timeout=10)
        assert _event_row(database, "second")["state"] == "queued"
    finally:
        if not task.done():
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        database.close()


@pytest.mark.asyncio
async def test_worker_pool_releases_slot_exactly_once(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    agent = FakeAgent(error=RuntimeError("boom"))
    pool = _pool(database, agent)
    releases: list[int] = []
    original_release = pool._pool.release

    def recording_release(slot_uid: int) -> None:
        releases.append(slot_uid)
        original_release(slot_uid)

    pool._pool.release = recording_release
    database.record_event("first", "issues.opened", "dderg/serval", 7, "reporter", _payload())
    task = asyncio.create_task(pool.run())
    try:
        await _wait_until(lambda: len(releases) == 1)
        await asyncio.sleep(0.1)
        assert releases == [FIRST_SLOT_UID]
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        database.close()


def test_chown_event_paths_hands_trees_to_slot_alone(tmp_path: Path, monkeypatch: Any) -> None:
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"
    workspace.mkdir(parents=True)
    (workspace / "run.sh").write_text("#!/bin/sh\n")
    (workspace / "run.sh").chmod(0o755)
    (workspace / "note.txt").write_text("x\n")
    session = tmp_path / "sessions" / "owner--repo" / "7"

    chowned: list[tuple[str, int, int]] = []
    monkeypatch.setattr(runtime_module, "slot_permissions_active", lambda slot_uid: slot_uid is not None)
    monkeypatch.setattr(runtime_module.os, "chown", lambda path, uid, gid: chowned.append((str(path), uid, gid)))

    runtime_module.chown_event_paths(workspace, session, 2003)

    assert stat.S_IMODE(workspace.stat().st_mode) == 0o700
    assert stat.S_IMODE((workspace / "run.sh").stat().st_mode) == 0o700
    assert stat.S_IMODE((workspace / "note.txt").stat().st_mode) == 0o600
    assert stat.S_IMODE(session.stat().st_mode) == 0o700
    assert stat.S_IMODE((session / ".tmp").stat().st_mode) == 0o700
    assert stat.S_IMODE(session.parent.stat().st_mode) == 0o755
    assert stat.S_IMODE(session.parent.parent.stat().st_mode) == 0o755
    assert chowned
    assert all(uid == 2003 and gid == 2003 for _, uid, gid in chowned)
    assert not any(path.startswith(str(tmp_path / "workspaces" / "owner--repo" / "pool.git")) for path, _, _ in chowned)
