from __future__ import annotations

import hashlib
import hmac
import time
from dataclasses import dataclass

TIMESTAMP_HEADER = "X-Serval-Bot-Timestamp"
SIGNATURE_HEADER = "X-Serval-Bot-Signature"


def _message(method: str, path: str, timestamp: str, body: bytes) -> bytes:
    digest = hashlib.sha256(body).hexdigest()
    return f"{method.upper()}\n{path}\n{timestamp}\n{digest}".encode()


def sign(method: str, path: str, body: bytes, key: str, timestamp: str | None = None) -> tuple[str, str]:
    current = timestamp or str(int(time.time()))
    signature = hmac.new(key.encode(), _message(method, path, current, body), hashlib.sha256).hexdigest()
    return current, signature


@dataclass(slots=True, frozen=True)
class Verification:
    valid: bool
    reason: str


def verify(
    method: str,
    path: str,
    body: bytes,
    key: str,
    timestamp: str | None,
    signature: str | None,
    now: float | None = None,
    skew_seconds: int = 30,
) -> Verification:
    if not timestamp or not signature:
        return Verification(False, "missing signature")
    try:
        request_time = int(timestamp)
    except ValueError:
        return Verification(False, "invalid timestamp")
    if abs(int(now if now is not None else time.time()) - request_time) > skew_seconds:
        return Verification(False, "expired timestamp")
    expected = hmac.new(key.encode(), _message(method, path, timestamp, body), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected, signature):
        return Verification(False, "signature mismatch")
    return Verification(True, "")
