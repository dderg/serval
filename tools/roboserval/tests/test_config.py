import pytest

from serval_bot.config import BotSettings, ConfigurationError, ProxySettings
from serval_bot.policy import Mode, PolicySet


def test_bot_settings_reads_inline_repository_policy(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.delenv("SERVAL_BOT_EVENT_MAX_RETRIES", raising=False)
    monkeypatch.delenv("SERVAL_BOT_EVENT_RETRY_DELAYS_SECONDS", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv(
        "SERVAL_BOT_REPOSITORY_POLICY",
        '[repositories."dderg/serval"]\nmode = "triage"\n',
    )

    settings = BotSettings.from_env()

    policy = PolicySet.parse(settings.policy_toml).require("dderg/serval")
    assert policy.mode is Mode.TRIAGE
    assert settings.task_timeout_seconds == 3600
    assert settings.event_max_retries == 3
    assert settings.event_retry_delays_seconds == (30, 120, 600)


def test_bot_settings_parses_retry_schedule(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv("SERVAL_BOT_REPOSITORY_POLICY", '[repositories."dderg/serval"]')
    monkeypatch.setenv("SERVAL_BOT_EVENT_MAX_RETRIES", "4")
    monkeypatch.setenv("SERVAL_BOT_EVENT_RETRY_DELAYS_SECONDS", "5,20")
    monkeypatch.setattr("serval_bot.config.random.random", lambda: 0.5)

    settings = BotSettings.from_env()

    assert settings.event_max_retries == 4
    assert settings.retry_delay_seconds(1) == 5
    assert settings.retry_delay_seconds(3) == 20


def _proxy_env(monkeypatch) -> None:
    monkeypatch.setenv("SERVAL_BOT_GITHUB_TOKEN_PATH", "/run/secrets/github_token")
    monkeypatch.setenv("SERVAL_BOT_PROXY_HMAC_KEY", "proxy-secret")


def test_proxy_settings_parses_repository_policy_from_same_env_var(monkeypatch) -> None:
    _proxy_env(monkeypatch)
    policy_toml = '[repositories."dderg/serval"]\nmode = "triage"\nmaintainers = ["dderg"]\n'
    monkeypatch.setenv("SERVAL_BOT_REPOSITORY_POLICY", policy_toml)

    settings = ProxySettings.from_env()

    policy = settings.policy.require("dderg/serval")
    assert policy.mode is Mode.TRIAGE
    assert policy.is_maintainer("dderg")


def test_proxy_settings_requires_repository_policy(monkeypatch) -> None:
    _proxy_env(monkeypatch)
    monkeypatch.delenv("SERVAL_BOT_REPOSITORY_POLICY", raising=False)

    with pytest.raises(ConfigurationError, match="SERVAL_BOT_REPOSITORY_POLICY"):
        ProxySettings.from_env()


def test_proxy_settings_rejects_invalid_repository_policy(monkeypatch) -> None:
    _proxy_env(monkeypatch)
    monkeypatch.setenv("SERVAL_BOT_REPOSITORY_POLICY", "not = [valid toml")

    with pytest.raises(ConfigurationError, match="SERVAL_BOT_REPOSITORY_POLICY"):
        ProxySettings.from_env()


def test_proxy_settings_rejects_unknown_policy_fields(monkeypatch) -> None:
    _proxy_env(monkeypatch)
    monkeypatch.setenv(
        "SERVAL_BOT_REPOSITORY_POLICY",
        '[repositories."dderg/serval"]\nbase_branch = "sota-motion"\n',
    )

    with pytest.raises(ConfigurationError, match="unknown policy fields"):
        ProxySettings.from_env()
