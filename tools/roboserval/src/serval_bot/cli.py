from __future__ import annotations

import logging

import uvicorn

from serval_bot.agent import TriageAgent
from serval_bot.config import BotSettings
from serval_bot.database import Database
from serval_bot.policy import PolicySet
from serval_bot.proxy_client import ProxyClient
from serval_bot.server import create_app


def build_app():
    settings = BotSettings.from_env()
    settings.ensure_paths()
    policies = PolicySet.parse(settings.policy_toml)
    database = Database(settings.data_dir / "serval-bot.sqlite")
    proxy = (
        ProxyClient(settings.proxy_url, settings.proxy_hmac_key)
        if settings.proxy_url is not None and settings.proxy_hmac_key is not None
        else None
    )
    agent = TriageAgent(settings, policies, database, proxy)
    return create_app(settings, policies, database, agent, proxy)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    settings = BotSettings.from_env()
    uvicorn.run(build_app(), host=settings.bind_host, port=settings.bind_port)
