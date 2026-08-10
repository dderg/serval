from __future__ import annotations

import os
import shlex
import tomllib
from dataclasses import dataclass
from pathlib import Path

from serval_bot.policy import PolicyError, PolicySet
from serval_bot.runtime import MAX_SLOTS


class ConfigurationError(RuntimeError):
    pass


def _required(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise ConfigurationError(f"required environment variable is empty: {name}")
    return value


def _positive_int(name: str, default: int) -> int:
    raw = os.environ.get(name, str(default))
    try:
        value = int(raw)
    except ValueError as exc:
        raise ConfigurationError(f"{name} must be an integer") from exc
    if value <= 0:
        raise ConfigurationError(f"{name} must be positive")
    return value


def _bounded_int(name: str, default: int, minimum: int, maximum: int) -> int:
    raw = os.environ.get(name, str(default))
    try:
        value = int(raw)
    except ValueError as exc:
        raise ConfigurationError(f"{name} must be an integer") from exc
    if not minimum <= value <= maximum:
        raise ConfigurationError(f"{name} must be between {minimum} and {maximum}")
    return value


@dataclass(slots=True, frozen=True)
class BotSettings:
    proxy_url: str | None
    proxy_hmac_key: str | None
    policy_toml: str
    data_dir: Path
    model: str
    provider: str | None
    thinking: str
    omp_command: tuple[str, ...]
    bind_host: str
    bind_port: int
    task_timeout_seconds: int
    poll_interval_seconds: int
    poll_overlap_seconds: int
    task_hard_grace_seconds: int = 60
    max_concurrency: int = 1

    @classmethod
    def from_env(cls) -> BotSettings:
        proxy_url = os.environ.get("SERVAL_BOT_PROXY_URL", "").strip() or None
        proxy_key = os.environ.get("SERVAL_BOT_PROXY_HMAC_KEY", "").strip() or None
        if (proxy_url is None) != (proxy_key is None):
            raise ConfigurationError("SERVAL_BOT_PROXY_URL and SERVAL_BOT_PROXY_HMAC_KEY must be set together")
        command = tuple(shlex.split(os.environ.get("SERVAL_BOT_OMP_COMMAND", "omp")))
        if not command:
            raise ConfigurationError("SERVAL_BOT_OMP_COMMAND must not be empty")
        return cls(
            proxy_url=proxy_url,
            proxy_hmac_key=proxy_key,
            policy_toml=_required("SERVAL_BOT_REPOSITORY_POLICY"),
            data_dir=Path(os.environ.get("SERVAL_BOT_DATA_DIR", "/data")),
            model=_required("SERVAL_BOT_MODEL"),
            provider=os.environ.get("SERVAL_BOT_PROVIDER", "").strip() or None,
            thinking=os.environ.get("SERVAL_BOT_THINKING", "medium").strip(),
            omp_command=command,
            bind_host=os.environ.get("SERVAL_BOT_BIND_HOST", "0.0.0.0"),
            bind_port=_positive_int("SERVAL_BOT_BIND_PORT", 8080),
            task_timeout_seconds=_positive_int("SERVAL_BOT_TASK_TIMEOUT_SECONDS", 3600),
            task_hard_grace_seconds=_bounded_int("SERVAL_BOT_TASK_TIMEOUT_HARD_GRACE_SECONDS", 60, 1, 3600),
            max_concurrency=_bounded_int("SERVAL_BOT_MAX_CONCURRENCY", 1, 1, MAX_SLOTS),
            poll_interval_seconds=_positive_int("SERVAL_BOT_POLL_INTERVAL_SECONDS", 30),
            poll_overlap_seconds=_positive_int("SERVAL_BOT_POLL_OVERLAP_SECONDS", 300),
        )

    def ensure_paths(self) -> None:
        for path in (self.data_dir, self.data_dir / "workspaces", self.data_dir / "sessions"):
            path.mkdir(parents=True, exist_ok=True)


@dataclass(slots=True, frozen=True)
class ProxySettings:
    github_token_path: Path
    hmac_key: str
    bind_host: str
    bind_port: int
    max_log_bytes: int
    workspace_root: Path
    policy: PolicySet

    @classmethod
    def from_env(cls) -> ProxySettings:
        try:
            policy = PolicySet.parse(_required("SERVAL_BOT_REPOSITORY_POLICY"))
        except (PolicyError, tomllib.TOMLDecodeError) as exc:
            raise ConfigurationError(f"invalid SERVAL_BOT_REPOSITORY_POLICY: {exc}") from exc
        return cls(
            github_token_path=Path(_required("SERVAL_BOT_GITHUB_TOKEN_PATH")),
            hmac_key=_required("SERVAL_BOT_PROXY_HMAC_KEY"),
            bind_host=os.environ.get("SERVAL_BOT_PROXY_BIND_HOST", "0.0.0.0"),
            bind_port=_positive_int("SERVAL_BOT_PROXY_BIND_PORT", 8081),
            max_log_bytes=_positive_int("SERVAL_BOT_PROXY_MAX_LOG_BYTES", 20_000),
            workspace_root=Path(os.environ.get("SERVAL_BOT_WORKSPACE_ROOT", "/data/workspaces")),
            policy=policy,
        )
