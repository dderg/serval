from pathlib import Path

import pytest

from serval_bot.token_auth import StaticTokenProvider, TokenFailure


@pytest.mark.asyncio
async def test_static_token_provider_reads_secret_once(tmp_path: Path) -> None:
    path = tmp_path / "token"
    path.write_text("ghp_original\n")
    provider = StaticTokenProvider(path)
    path.write_text("ghp_replacement\n")
    assert await provider.token() == "ghp_original"


@pytest.mark.parametrize("contents", [None, "", "\n"])
def test_static_token_provider_rejects_missing_or_empty_secret(tmp_path: Path, contents: str | None) -> None:
    path = tmp_path / "token"
    if contents is not None:
        path.write_text(contents)
    with pytest.raises(TokenFailure):
        StaticTokenProvider(path)
