import pytest

from serval_bot.config import BotSettings, ConfigurationError, ProxySettings
from serval_bot.policy import Mode, PolicySet


def test_bot_settings_reads_inline_repository_policy(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.delenv("SERVAL_BOT_TELEGRAM_TOKEN", raising=False)
    monkeypatch.delenv("SERVAL_BOT_TELEGRAM_CHAT_ID", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv(
        "SERVAL_BOT_REPOSITORY_POLICY",
        '[repositories."dderg/serval"]\nmode = "triage"\n',
    )

    settings = BotSettings.from_env()

    policy = PolicySet.parse(settings.policy_toml).require("dderg/serval")
    assert policy.mode is Mode.TRIAGE
    assert settings.task_timeout_seconds == 3600


def test_bot_settings_reads_telegram_credentials_as_a_pair(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv("SERVAL_BOT_REPOSITORY_POLICY", '[repositories."dderg/serval"]')
    monkeypatch.setenv("SERVAL_BOT_TELEGRAM_TOKEN", "telegram-secret")
    monkeypatch.setenv("SERVAL_BOT_TELEGRAM_CHAT_ID", "123456")

    settings = BotSettings.from_env()

    assert settings.telegram_token == "telegram-secret"
    assert settings.telegram_chat_id == "123456"


def test_bot_settings_rejects_partial_telegram_configuration(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv("SERVAL_BOT_REPOSITORY_POLICY", '[repositories."dderg/serval"]')
    monkeypatch.setenv("SERVAL_BOT_TELEGRAM_TOKEN", "telegram-secret")
    monkeypatch.delenv("SERVAL_BOT_TELEGRAM_CHAT_ID", raising=False)

    with pytest.raises(ConfigurationError, match="must be set together"):
        BotSettings.from_env()


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
