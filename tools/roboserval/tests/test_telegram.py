from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from urllib.request import Request

from serval_bot.database import Database, Event
from serval_bot.telegram import TelegramNotifier


def _claimed_event(database: Database) -> Event:
    database.record_event(
        "poll:review:42:head:requested",
        "pull_request_review.requested",
        "dderg/serval",
        390,
        "dderg",
        {"pull_request": {"number": 390}},
    )
    event = database.claim()
    assert event is not None
    return event


class FakeResponse:
    status = 200

    def __enter__(self) -> FakeResponse:
        return self

    def __exit__(self, *_: Any) -> None:
        return None

    @staticmethod
    def read() -> bytes:
        return b'{"ok":true,"result":{"message_id":73}}'


def test_telegram_notifier_sends_private_generic_failure_once(tmp_path: Path, monkeypatch: Any) -> None:
    requests: list[Request] = []

    def send(request: Request, timeout: float) -> FakeResponse:
        assert timeout == 10.0
        requests.append(request)
        return FakeResponse()

    monkeypatch.setattr("serval_bot.telegram.urlopen", send)
    database = Database(tmp_path / "bot.sqlite")
    try:
        event = _claimed_event(database)
        notifier = TelegramNotifier(database, "secret-token", "123456", api_base="https://telegram.test")
        notifier.notify_event_failure(event)
        notifier.notify_event_failure(event)

        assert len(requests) == 1
        assert requests[0].full_url == "https://telegram.test/botsecret-token/sendMessage"
        payload = json.loads(requests[0].data or b"")
        assert payload["chat_id"] == "123456"
        assert "dderg/serval" in payload["text"]
        assert "#390" in payload["text"]
        assert "provider" not in payload["text"].casefold()
        assert "token" not in payload["text"].casefold()
        action = database.find_action(event.delivery_id, "telegram_failure_alert")
        assert action is not None
        assert action.state == "applied"
        assert action.result == {"message_id": 73}
    finally:
        database.close()
