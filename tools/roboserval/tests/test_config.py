from serval_bot.config import BotSettings
from serval_bot.policy import Mode, PolicySet


def test_bot_settings_reads_inline_repository_policy(monkeypatch) -> None:
    monkeypatch.delenv("SERVAL_BOT_PROXY_URL", raising=False)
    monkeypatch.delenv("SERVAL_BOT_PROXY_HMAC_KEY", raising=False)
    monkeypatch.setenv("SERVAL_BOT_MODEL", "test/model")
    monkeypatch.setenv(
        "SERVAL_BOT_REPOSITORY_POLICY",
        '[repositories."dderg/serval"]\nmode = "triage"\n',
    )

    settings = BotSettings.from_env()

    policy = PolicySet.parse(settings.policy_toml).require("dderg/serval")
    assert policy.mode is Mode.TRIAGE
