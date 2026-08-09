from __future__ import annotations

from pathlib import Path


class TokenFailure(RuntimeError):
    pass


class StaticTokenProvider:
    def __init__(self, token_path: Path):
        try:
            token = token_path.read_text().strip()
        except OSError as exc:
            raise TokenFailure(f"cannot read GitHub token from {token_path}: {exc}") from exc
        if not token:
            raise TokenFailure(f"GitHub token is empty: {token_path}")
        self._token = token

    async def token(self) -> str:
        return self._token

    async def close(self) -> None:
        return None
