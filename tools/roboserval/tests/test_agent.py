from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import serval_bot.agent as agent_module
from serval_bot.agent import PreparedWorkspace, TriageAgent
from serval_bot.config import BotSettings
from serval_bot.database import Event
from serval_bot.policy import Mode, PolicySet, RepositoryPolicy


class FakeDatabase:
    def actions_for_issue(self, repo: str, issue_number: int) -> list[Any]:
        return []


class FakeRpcClient:
    request_timeout: float | None = None
    prompt_timeout: float | None = None
    prompt: str | None = None

    def __init__(self, *, request_timeout: float, **_: Any):
        type(self).request_timeout = request_timeout

    def __enter__(self) -> FakeRpcClient:
        return self

    def __exit__(self, *_: object) -> None:
        return None

    def install_headless_ui(self) -> None:
        return None

    def prompt_and_wait(self, prompt: str, *, timeout: float) -> SimpleNamespace:
        type(self).prompt_timeout = timeout
        type(self).prompt = prompt
        return SimpleNamespace(assistant_text="done")


def test_task_timeout_covers_full_agent_turn(tmp_path: Path, monkeypatch: Any) -> None:
    workspace = tmp_path / "workspaces" / "dderg--serval"
    workspace.mkdir(parents=True)
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    monkeypatch.setattr(
        agent_module.WorkspaceManager,
        "prepare",
        lambda self, policy: PreparedWorkspace(workspace, "trunk"),
    )
    settings = BotSettings(
        proxy_url=None,
        proxy_hmac_key=None,
        policy_toml='[repositories."dderg/serval"]',
        data_dir=tmp_path,
        model="test/model",
        provider=None,
        thinking="xhigh",
        omp_command=("omp",),
        bind_host="127.0.0.1",
        bind_port=8080,
        task_timeout_seconds=1200,
        poll_interval_seconds=30,
        poll_overlap_seconds=300,
    )
    policies = PolicySet(
        {
            "dderg/serval": RepositoryPolicy(
                repo="dderg/serval",
                mode=Mode.SHADOW,
                bot_login="roboserval",
                maintainers=frozenset({"dderg"}),
                sim_workflow="ci-sim-e2e.yaml",
            )
        }
    )
    event = Event(
        delivery_id="delivery",
        event_type="issue_comment.created",
        repo="dderg/serval",
        issue_number=370,
        actor="dderg",
        payload={"issue": {"title": "restart", "body": "details"}, "comment": {"body": "triage"}},
        state="running",
        attempts=1,
        error=None,
    )

    answer = TriageAgent(settings, policies, FakeDatabase(), None).run(event)

    assert answer == "done"
    assert FakeRpcClient.request_timeout == 1200.0
    assert FakeRpcClient.prompt_timeout == 1200.0
    assert FakeRpcClient.prompt is not None
    assert "Default branch: trunk" in FakeRpcClient.prompt
