from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from serval_bot.database import ActionConflict, Database, Event


class TelegramFailure(RuntimeError):
    pass


@dataclass(slots=True)
class TelegramNotifier:
    database: Database
    token: str
    chat_id: str
    timeout: float = 10.0
    api_base: str = "https://api.telegram.org"

    def notify_event_failure(self, event: Event) -> None:
        kind = "telegram_failure_alert"
        if self.database.find_action(event.delivery_id, kind) is not None:
            return
        try:
            action_id = self.database.add_action(
                event,
                kind,
                {"chat_id": self.chat_id},
                "proposed",
            )
        except ActionConflict:
            return
        text = (
            "RoboServal needs attention\n\n"
            f"Repository: {event.repo}\n"
            f"Issue or PR: #{event.issue_number}\n"
            f"Task: {event.event_type}\n"
            f"Failure ID: {event.delivery_id}\n\n"
            "The agent task failed. Check the private runtime logs and credentials, then request the action again."
        )
        try:
            payload = self._send(text)
        except Exception as exc:
            self.database.update_action(
                action_id,
                "failed",
                {"error": f"{type(exc).__name__}: {exc}"},
            )
            raise
        self.database.update_action(action_id, "applied", {"message_id": payload.get("result", {}).get("message_id")})

    def _send(self, text: str) -> dict[str, Any]:
        body = json.dumps({"chat_id": self.chat_id, "text": text}).encode()
        request = Request(
            f"{self.api_base}/bot{self.token}/sendMessage",
            data=body,
            headers={"content-type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                status = response.status
                content = response.read()
        except HTTPError as exc:
            raise TelegramFailure(f"Telegram sendMessage failed with HTTP {exc.code}") from None
        except URLError:
            raise TelegramFailure("Telegram sendMessage transport failed") from None
        if status >= 400:
            raise TelegramFailure(f"Telegram sendMessage failed with HTTP {status}")
        payload: Any = json.loads(content)
        if not isinstance(payload, dict) or payload.get("ok") is not True:
            raise TelegramFailure("Telegram sendMessage returned an unsuccessful response")
        return payload
