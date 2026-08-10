from __future__ import annotations

import json
import sqlite3
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Literal

EventState = Literal["queued", "running", "done", "failed", "skipped"]
ActionState = Literal["proposed", "applied", "rejected", "failed"]


class ActionConflict(RuntimeError):
    """The delivery already recorded an action of this kind."""

    def __init__(self, delivery_id: str, kind: str):
        super().__init__(f"delivery {delivery_id} already recorded action kind {kind}")
        self.delivery_id = delivery_id
        self.kind = kind


_SCHEMA = """
PRAGMA journal_mode=WAL;
PRAGMA foreign_keys=ON;
CREATE TABLE IF NOT EXISTS events (
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
CREATE INDEX IF NOT EXISTS events_state_idx ON events(state, created_at);
CREATE TABLE IF NOT EXISTS actions (
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
CREATE INDEX IF NOT EXISTS actions_issue_idx ON actions(repo, issue_number, id);
CREATE TABLE IF NOT EXISTS workflow_runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    repo TEXT NOT NULL,
    issue_number INTEGER NOT NULL,
    workflow TEXT NOT NULL,
    ref TEXT NOT NULL,
    run_id INTEGER NOT NULL UNIQUE,
    url TEXT NOT NULL,
    status TEXT NOT NULL,
    conclusion TEXT,
    updated_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS poll_cursors (
    repo TEXT PRIMARY KEY,
    cursor TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
"""
_UNIQUE_ACTION_KIND_INDEX = "CREATE UNIQUE INDEX IF NOT EXISTS actions_delivery_kind_idx ON actions(delivery_id, kind);"


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="microseconds")


@dataclass(slots=True, frozen=True)
class Event:
    delivery_id: str
    event_type: str
    repo: str
    issue_number: int
    actor: str
    payload: dict[str, Any]
    state: EventState
    attempts: int
    error: str | None


@dataclass(slots=True, frozen=True)
class PolledEvent:
    delivery_id: str
    event_type: str
    issue_number: int
    actor: str
    occurred_at: str
    payload: dict[str, Any]


@dataclass(slots=True, frozen=True)
class Action:
    id: int
    delivery_id: str
    repo: str
    issue_number: int
    kind: str
    arguments: dict[str, Any]
    state: ActionState
    result: dict[str, Any] | None


class Database:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self._connection.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        with self._lock:
            self._connection.executescript(_SCHEMA)
            self._migrate_action_uniqueness()

    def _migrate_action_uniqueness(self) -> None:
        """Enforce one action kind per delivery; refuse ambiguous history loudly."""
        rows = self._connection.execute(
            """
            SELECT delivery_id, kind, COUNT(*) AS count
            FROM actions
            GROUP BY delivery_id, kind
            HAVING COUNT(*) > 1
            ORDER BY delivery_id, kind
            LIMIT 20
            """
        ).fetchall()
        if rows:
            details = ", ".join(f"{row['delivery_id']}:{row['kind']}x{row['count']}" for row in rows)
            raise RuntimeError(
                f"database has conflicting duplicate actions (delivery:kind x count): {details}; "
                "refusing to start with ambiguous action history"
            )
        self._connection.execute(_UNIQUE_ACTION_KIND_INDEX)

    @contextmanager
    def _transaction(self) -> Iterator[sqlite3.Connection]:
        with self._lock:
            self._connection.execute("BEGIN IMMEDIATE")
            try:
                yield self._connection
            except BaseException:
                self._connection.rollback()
                raise
            else:
                self._connection.commit()

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def reset_running(self) -> int:
        with self._transaction() as connection:
            cursor = connection.execute(
                "UPDATE events SET state='queued', updated_at=? WHERE state='running'", (_now(),)
            )
            return cursor.rowcount

    def record_event(
        self,
        delivery_id: str,
        event_type: str,
        repo: str,
        issue_number: int,
        actor: str,
        payload: dict[str, Any],
    ) -> bool:
        timestamp = _now()
        with self._transaction() as connection:
            cursor = connection.execute(
                """
                INSERT OR IGNORE INTO events
                (delivery_id, event_type, repo, issue_number, actor, payload, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (delivery_id, event_type, repo, issue_number, actor, json.dumps(payload), timestamp, timestamp),
            )
            return cursor.rowcount == 1

    def poll_cursor(self, repo: str) -> str | None:
        with self._lock:
            row = self._connection.execute("SELECT cursor FROM poll_cursors WHERE repo=?", (repo,)).fetchone()
        return str(row["cursor"]) if row is not None else None

    def record_poll_batch(self, repo: str, cursor: str, events: list[PolledEvent]) -> int:
        inserted = 0
        timestamp = _now()
        with self._transaction() as connection:
            for event in events:
                result = connection.execute(
                    """
                    INSERT OR IGNORE INTO events
                    (delivery_id, event_type, repo, issue_number, actor, payload, created_at, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event.delivery_id,
                        event.event_type,
                        repo,
                        event.issue_number,
                        event.actor,
                        json.dumps(event.payload),
                        event.occurred_at,
                        timestamp,
                    ),
                )
                inserted += result.rowcount
            connection.execute(
                """
                INSERT INTO poll_cursors (repo, cursor, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(repo) DO UPDATE SET
                    cursor=excluded.cursor,
                    updated_at=excluded.updated_at
                """,
                (repo, cursor, timestamp),
            )
        return inserted

    def claim(self) -> Event | None:
        with self._transaction() as connection:
            row = connection.execute(
                """
                SELECT * FROM events AS candidate
                WHERE candidate.state='queued'
                  AND NOT EXISTS (
                      SELECT 1 FROM events AS active
                      WHERE active.state='running'
                        AND active.repo=candidate.repo
                        AND active.issue_number=candidate.issue_number
                  )
                ORDER BY candidate.created_at LIMIT 1
                """
            ).fetchone()
            if row is None:
                return None
            connection.execute(
                "UPDATE events SET state='running', attempts=attempts+1, updated_at=? WHERE delivery_id=?",
                (_now(), row["delivery_id"]),
            )
            return _event_from_row(row, state="running", attempts=int(row["attempts"]) + 1)

    def finish(self, delivery_id: str, state: EventState, error: str | None = None) -> None:
        if state not in {"done", "failed", "skipped"}:
            raise ValueError(f"terminal event state required: {state}")
        with self._transaction() as connection:
            cursor = connection.execute(
                "UPDATE events SET state=?, error=?, updated_at=? WHERE delivery_id=? AND state='running'",
                (state, error, _now(), delivery_id),
            )
            if cursor.rowcount != 1:
                raise RuntimeError(f"event is not running: {delivery_id}")

    def add_action(
        self,
        event: Event,
        kind: str,
        arguments: dict[str, Any],
        state: ActionState,
        result: dict[str, Any] | None = None,
    ) -> int:
        with self._transaction() as connection:
            try:
                cursor = connection.execute(
                    """
                    INSERT INTO actions
                    (delivery_id, repo, issue_number, kind, arguments, state, result, created_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event.delivery_id,
                        event.repo,
                        event.issue_number,
                        kind,
                        json.dumps(arguments),
                        state,
                        json.dumps(result) if result is not None else None,
                        _now(),
                    ),
                )
            except sqlite3.IntegrityError as exc:
                if "UNIQUE constraint failed: actions.delivery_id, actions.kind" in str(exc):
                    raise ActionConflict(event.delivery_id, kind) from exc
                raise
            return int(cursor.lastrowid)

    def find_action(self, delivery_id: str, kind: str) -> Action | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM actions WHERE delivery_id=? AND kind=? ORDER BY id LIMIT 1",
                (delivery_id, kind),
            ).fetchone()
        return _action_from_row(row) if row is not None else None

    def actions_for_delivery(self, delivery_id: str) -> list[Action]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM actions WHERE delivery_id=? ORDER BY id",
                (delivery_id,),
            ).fetchall()
        return [_action_from_row(row) for row in rows]

    def update_action(self, action_id: int, state: ActionState, result: dict[str, Any]) -> None:
        with self._transaction() as connection:
            cursor = connection.execute(
                "UPDATE actions SET state=?, result=? WHERE id=?",
                (state, json.dumps(result), action_id),
            )
            if cursor.rowcount != 1:
                raise RuntimeError(f"unknown action: {action_id}")

    def actions_for_issue(self, repo: str, issue_number: int) -> list[Action]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM actions WHERE repo=? AND issue_number=? ORDER BY id",
                (repo, issue_number),
            ).fetchall()
        return [_action_from_row(row) for row in rows]

    def recent_events(self, limit: int = 100) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM events ORDER BY created_at DESC LIMIT ?", (limit,)
            ).fetchall()
        return [dict(row) for row in rows]

    def claim_workflow_run(
        self,
        repo: str,
        issue_number: int,
        workflow: str,
        ref: str,
        run_id: int,
        url: str,
        status: str,
        conclusion: str | None,
    ) -> bool:
        """Atomically associate a workflow run with an issue.

        Returns False when the run is already associated with any issue, so a
        dispatch never silently reuses a previously claimed run.
        """
        with self._transaction() as connection:
            try:
                connection.execute(
                    """
                    INSERT INTO workflow_runs
                    (repo, issue_number, workflow, ref, run_id, url, status, conclusion, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (repo, issue_number, workflow, ref, run_id, url, status, conclusion, _now()),
                )
            except sqlite3.IntegrityError as exc:
                if "UNIQUE constraint failed: workflow_runs.run_id" in str(exc):
                    return False
                raise
            return True

    def workflow_run(self, repo: str, issue_number: int, run_id: int) -> dict[str, Any] | None:
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM workflow_runs WHERE repo=? AND issue_number=? AND run_id=?",
                (repo, issue_number, run_id),
            ).fetchone()
        return dict(row) if row is not None else None

    def update_workflow_run_status(self, run_id: int, status: str, conclusion: str | None) -> None:
        with self._transaction() as connection:
            cursor = connection.execute(
                "UPDATE workflow_runs SET status=?, conclusion=?, updated_at=? WHERE run_id=?",
                (status, conclusion, _now(), run_id),
            )
            if cursor.rowcount != 1:
                raise RuntimeError(f"unknown workflow run: {run_id}")

    def workflow_runs_for_issue(self, repo: str, issue_number: int) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT * FROM workflow_runs WHERE repo=? AND issue_number=? ORDER BY id",
                (repo, issue_number),
            ).fetchall()
        return [dict(row) for row in rows]


def _event_from_row(row: sqlite3.Row, *, state: EventState | None = None, attempts: int | None = None) -> Event:
    return Event(
        delivery_id=str(row["delivery_id"]),
        event_type=str(row["event_type"]),
        repo=str(row["repo"]),
        issue_number=int(row["issue_number"]),
        actor=str(row["actor"]),
        payload=json.loads(row["payload"]),
        state=state or row["state"],
        attempts=attempts if attempts is not None else int(row["attempts"]),
        error=row["error"],
    )


def _action_from_row(row: sqlite3.Row) -> Action:
    return Action(
        id=int(row["id"]),
        delivery_id=str(row["delivery_id"]),
        repo=str(row["repo"]),
        issue_number=int(row["issue_number"]),
        kind=str(row["kind"]),
        arguments=json.loads(row["arguments"]),
        state=row["state"],
        result=json.loads(row["result"]) if row["result"] else None,
    )
