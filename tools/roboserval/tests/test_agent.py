from __future__ import annotations

import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, ClassVar

import pytest
from omp_rpc import RpcProcessExitError

import serval_bot.agent as agent_module
from serval_bot.actions import ActionGateway
from serval_bot.agent import (
    AgentFailure,
    AgentInterrupted,
    HostToolTracker,
    PreparedWorkspace,
    PullRequestContext,
    TriageAgent,
)
from serval_bot.config import BotSettings
from serval_bot.database import Database, Event
from serval_bot.policy import Mode, PolicySet, RepositoryPolicy


def _comment_action() -> SimpleNamespace:
    return SimpleNamespace(kind="comment", state="applied")


def _classify_action() -> SimpleNamespace:
    return SimpleNamespace(kind="classify", state="applied")


class FakeDatabase:
    def __init__(self, actions: list[Any] | None = None):
        self.actions = list(actions or [])

    def actions_for_delivery(self, delivery_id: str) -> list[Any]:
        return list(self.actions)


class FakeRpcClient:
    request_timeout: float | None = None
    prompt_timeout: float | None = None
    prompt: str | None = None
    prompts: ClassVar[list[str]] = []
    command: tuple[str, ...] | None = None
    started = False
    stopped = False

    def __init__(self, *, command: tuple[str, ...], request_timeout: float, **_: Any):
        type(self).command = command
        type(self).request_timeout = request_timeout

    def __enter__(self) -> FakeRpcClient:
        return self.start()

    def __exit__(self, *_: object) -> None:
        self.stop()

    def start(self) -> FakeRpcClient:
        type(self).started = True
        return self

    def stop(self) -> None:
        type(self).stopped = True

    def install_headless_ui(self) -> None:
        return None

    def prompt_and_wait(self, prompt: str, *, timeout: float) -> SimpleNamespace:
        type(self).prompt_timeout = timeout
        type(self).prompt = prompt
        type(self).prompts.append(prompt)
        return SimpleNamespace(assistant_text="done")


def test_task_timeout_covers_full_agent_turn(tmp_path: Path, monkeypatch: Any) -> None:
    workspace = tmp_path / "workspaces" / "dderg--serval"
    workspace.mkdir(parents=True)
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    monkeypatch.setattr(
        agent_module.WorkspaceManager,
        "prepare",
        lambda self, policy, pull_request=None: PreparedWorkspace(workspace, "trunk"),
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

    answer = TriageAgent(settings, policies, FakeDatabase(actions=[_comment_action()]), None).run(event)

    assert answer == "done"
    assert FakeRpcClient.request_timeout == 1200.0
    assert FakeRpcClient.prompt_timeout == 1200.0
    assert FakeRpcClient.prompt is not None
    assert "Default branch: trunk" in FakeRpcClient.prompt


def test_pull_request_review_uses_exact_revision_and_posts_comment(tmp_path: Path, monkeypatch: Any) -> None:
    workspace = tmp_path / "workspaces" / "dderg--serval"
    workspace.mkdir(parents=True)
    prepared: list[PullRequestContext | None] = []
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    monkeypatch.setattr(
        agent_module.WorkspaceManager,
        "prepare",
        lambda self, policy, pull_request=None: (
            prepared.append(pull_request) or PreparedWorkspace(workspace, "sota-motion")
        ),
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
                mode=Mode.TRIAGE,
                bot_login="roboserval",
                maintainers=frozenset({"dderg"}),
                sim_workflow="ci-sim-e2e.yaml",
            )
        }
    )
    event = Event(
        delivery_id="poll:review:45:requested",
        event_type="pull_request_review.requested",
        repo="dderg/serval",
        issue_number=373,
        actor="dderg",
        payload={
            "issue": {"title": "Fix LEDs", "body": "Change priority dispatch"},
            "review_request": {
                "id": 45,
                "event": "review_requested",
                "requested_reviewer": {"login": "roboserval"},
            },
            "pull_request": {
                "number": 373,
                "title": "Fix LEDs",
                "body": "Change priority dispatch",
                "html_url": "https://github.com/dderg/serval/pull/373",
                "base": {"ref": "sota-motion", "sha": "a" * 40},
                "head": {"ref": "fix-leds", "sha": "b" * 40},
            },
        },
        state="running",
        attempts=1,
        error=None,
    )

    class ReviewDatabase:
        def actions_for_delivery(self, delivery_id: str) -> list[Any]:
            return [_comment_action()]

    answer = TriageAgent(settings, policies, ReviewDatabase(), None).run(event)

    assert answer == "done"
    assert prepared[0] is not None
    assert prepared[0].head_sha == "b" * 40
    assert FakeRpcClient.command is not None
    assert "read,grep,glob,lsp,bash" in FakeRpcClient.command
    assert "write" not in FakeRpcClient.command
    assert "Pull request: #373 Fix LEDs" in (FakeRpcClient.prompt or "")
    assert f"Review the exact diff {'a' * 40}...{'b' * 40}" in (FakeRpcClient.prompt or "")
    assert "Do not classify or label the pull request" in (FakeRpcClient.prompt or "")
    assert "@dderg requested @roboserval as a reviewer through GitHub reviewer assignment" in (
        FakeRpcClient.prompt or ""
    )


def _prepared_settings(tmp_path: Path, monkeypatch: Any) -> BotSettings:
    workspace = tmp_path / "workspaces" / "dderg--serval"
    workspace.mkdir(parents=True)
    monkeypatch.setattr(
        agent_module.WorkspaceManager,
        "prepare",
        lambda self, policy, pull_request=None: PreparedWorkspace(workspace, "trunk"),
    )
    return BotSettings(
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


def _shadow_policies() -> PolicySet:
    return PolicySet(
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


def _comment_event(delivery_id: str, issue_number: int) -> Event:
    return Event(
        delivery_id=delivery_id,
        event_type="issue_comment.created",
        repo="dderg/serval",
        issue_number=issue_number,
        actor="dderg",
        payload={"issue": {"title": "restart", "body": "details"}, "comment": {"body": "triage"}},
        state="running",
        attempts=1,
        error=None,
    )


def _sim_directive_event(delivery_id: str, issue_number: int) -> Event:
    return Event(
        delivery_id=delivery_id,
        event_type="issue_comment.created",
        repo="dderg/serval",
        issue_number=issue_number,
        actor="dderg",
        payload={
            "issue": {"title": "restart", "body": "details"},
            "comment": {"body": "@roboserval Reproduce in the simulator"},
        },
        state="running",
        attempts=1,
        error=None,
    )


def _pr_sim_directive_event(delivery_id: str, issue_number: int) -> Event:
    return Event(
        delivery_id=delivery_id,
        event_type="pull_request_review.requested",
        repo="dderg/serval",
        issue_number=issue_number,
        actor="dderg",
        payload={
            "issue": {"title": "Fix LEDs", "body": ""},
            "comment": {"body": "@roboserval Reproduce in the simulator"},
            "pull_request": {
                "number": issue_number,
                "title": "Fix LEDs",
                "body": "",
                "html_url": f"https://github.com/dderg/serval/pull/{issue_number}",
                "base": {"ref": "sota-motion", "sha": "a" * 40},
                "head": {"ref": "fix-leds", "sha": "b" * 40},
            },
        },
        state="running",
        attempts=1,
        error=None,
    )


def test_stop_before_registration_prevents_start(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.started = False
    FakeRpcClient.stopped = False
    agent = TriageAgent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), FakeDatabase(), None)
    agent.stop("delivery-early")

    with pytest.raises(AgentInterrupted):
        agent.run(_comment_event("delivery-early", 370))

    assert not FakeRpcClient.started
    assert not FakeRpcClient.stopped


def test_stop_during_start_sees_live_client_and_kills(tmp_path: Path, monkeypatch: Any) -> None:
    class RacingRpcClient(FakeRpcClient):
        start_entered = threading.Event()
        proceed = threading.Event()
        stop_called = threading.Event()

        def start(self) -> RacingRpcClient:
            self.start_entered.set()
            assert self.proceed.wait(5)
            return self

        def stop(self) -> None:
            self.stop_called.set()
            super().stop()

        def prompt_and_wait(self, prompt: str, *, timeout: float) -> SimpleNamespace:
            self.stop_called.wait(5)
            raise RpcProcessExitError("RPC process stopped")

    monkeypatch.setattr(agent_module, "RpcClient", RacingRpcClient)
    agent = TriageAgent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), FakeDatabase(), None)
    event = _comment_event("delivery-race", 371)
    outcome: dict[str, str] = {}

    def turn() -> None:
        try:
            agent.run(event)
            outcome["result"] = "completed"
        except AgentInterrupted:
            outcome["result"] = "interrupted"
        except BaseException as exc:
            outcome["result"] = f"{type(exc).__name__}"

    runner = threading.Thread(target=turn)
    runner.start()
    assert RacingRpcClient.start_entered.wait(5)
    stopper = threading.Thread(target=lambda: agent.stop(event.delivery_id))
    stopper.start()
    time.sleep(0.05)
    RacingRpcClient.proceed.set()
    runner.join(10)
    stopper.join(10)

    assert not runner.is_alive()
    assert not stopper.is_alive()
    assert RacingRpcClient.stop_called.is_set()
    assert outcome["result"] == "interrupted"


def _opened_event(delivery_id: str, issue_number: int) -> Event:
    return Event(
        delivery_id=delivery_id,
        event_type="issues.opened",
        repo="dderg/serval",
        issue_number=issue_number,
        actor="reporter",
        payload={"issue": {"title": "crash", "body": "details"}},
        state="running",
        attempts=1,
        error=None,
    )


def test_opened_issue_without_classification_gets_one_reminder_then_fails(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.prompts = []
    agent = TriageAgent(
        _prepared_settings(tmp_path, monkeypatch),
        _shadow_policies(),
        FakeDatabase(actions=[]),
        None,
    )

    with pytest.raises(AgentFailure, match="without classification and comment"):
        agent.run(_opened_event("delivery-reminder", 374))

    assert len(FakeRpcClient.prompts) == 2
    assert "final turn" in FakeRpcClient.prompts[1]


def test_reminder_turn_can_complete_required_delivery(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.prompts = []

    class ReminderDatabase:
        calls = 0

        def actions_for_delivery(self, delivery_id: str) -> list[Any]:
            type(self).calls += 1
            if type(self).calls == 1:
                return []
            return [_classify_action(), _comment_action()]

    answer = TriageAgent(
        _prepared_settings(tmp_path, monkeypatch),
        _shadow_policies(),
        ReminderDatabase(),
        None,
    ).run(_opened_event("delivery-reminder-ok", 375))

    assert answer == "done\n\ndone"
    assert len(FakeRpcClient.prompts) == 2


def test_followup_without_comment_gets_one_reminder_then_fails(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.prompts = []
    agent = TriageAgent(
        _prepared_settings(tmp_path, monkeypatch),
        _shadow_policies(),
        FakeDatabase(actions=[]),
        None,
    )

    with pytest.raises(AgentFailure, match="without a response comment"):
        agent.run(_comment_event("delivery-reminder-followup", 376))

    assert len(FakeRpcClient.prompts) == 2


def _gateway(database: Database, event: Event) -> ActionGateway:
    return ActionGateway(
        database,
        event,
        _shadow_policies().require("dderg/serval"),
        "trunk",
        None,
    )


def test_opened_issue_tools_allow_classify_and_exclude_simulator(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        event = _opened_event("delivery-tools-opened", 377)
        names = [
            tool.name
            for tool in TriageAgent._tools(_gateway(database, event), _shadow_policies().require("dderg/serval"), event)
        ]
        assert "classify_issue" in names
        assert "post_issue_comment" in names
        assert "search_issues" in names
        assert "dispatch_simulator" not in names
        assert "get_simulator_result" not in names
    finally:
        database.close()


def test_followup_tools_exclude_classify_and_include_simulator(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        event = _comment_event("delivery-tools-followup", 378)
        tools = [
            tool
            for tool in TriageAgent._tools(_gateway(database, event), _shadow_policies().require("dderg/serval"), event)
        ]
        names = [tool.name for tool in tools]
        assert "classify_issue" not in names
        assert "post_issue_comment" in names
        assert "search_issues" in names
        assert "dispatch_simulator" in names
        assert "get_simulator_result" in names
        dispatch = next(tool for tool in tools if tool.name == "dispatch_simulator")
        assert "farm/378-<slug>" in dispatch.description
        assert "default branch" in dispatch.description
        assert "[skip ci]" in dispatch.description
        assert ".github/workflows/**" in dispatch.description
        assert dispatch.parameters["required"] == ["ref"]
        assert dispatch.parameters["properties"]["head_sha"]["pattern"] == "^[0-9a-f]{40}$"
        result = next(tool for tool in tools if tool.name == "get_simulator_result")
        assert "Poll" in result.description
        assert "terminal" in result.description
    finally:
        database.close()


def test_simulator_directive_prompt_requires_dispatch_result_comment(tmp_path: Path) -> None:
    policy = _shadow_policies().require("dderg/serval")
    event = _sim_directive_event("delivery-prompt-sim", 381)
    prompt = TriageAgent._prompt(event, policy, "trunk", None)
    assert "farm/381-<slug>" in prompt
    assert "tools/sim/tests/test_*.py" in prompt
    assert "dispatch_simulator" in prompt
    assert "get_simulator_result" in prompt
    assert "[skip ci]" in prompt
    assert ".github/workflows/**" in prompt
    assert "Never push with git" in prompt or "never push with git" in prompt
    assert "control" in prompt and "reproduced or not-reproduced evidence" in prompt
    assert "cites the exact run" in prompt


def test_ordinary_followup_prompt_stays_comment_only(tmp_path: Path) -> None:
    event = _comment_event("delivery-prompt-ordinary", 382)
    prompt = TriageAgent._prompt(event, _shadow_policies().require("dderg/serval"), "trunk", None)
    assert "dispatch_simulator" not in prompt
    assert "exactly one comment completes this delivery" in prompt


def test_pr_review_tools_exclude_classify(tmp_path: Path) -> None:
    database = Database(tmp_path / "bot.sqlite")
    try:
        event = Event(
            delivery_id="delivery-tools-pr",
            event_type="pull_request_review.requested",
            repo="dderg/serval",
            issue_number=379,
            actor="dderg",
            payload={"issue": {"title": "Fix LEDs", "body": ""}, "comment": {"body": "@roboserval review"}},
            state="running",
            attempts=1,
            error=None,
        )
        names = [
            tool.name
            for tool in TriageAgent._tools(_gateway(database, event), _shadow_policies().require("dderg/serval"), event)
        ]
        assert "classify_issue" not in names
        assert "post_issue_comment" in names
        assert "search_issues" in names
    finally:
        database.close()


def _agent(settings: Any, policies: Any, actions: list[Any]) -> TriageAgent:
    return TriageAgent(settings, policies, FakeDatabase(actions=actions), None)


def test_simulator_directive_completion_requires_dispatch_result_comment(tmp_path: Path, monkeypatch: Any) -> None:
    settings = _prepared_settings(tmp_path, monkeypatch)
    policies = _shadow_policies()
    event = _sim_directive_event("delivery-complete-sim", 383)
    cases = [
        ([_comment_action()], False),
        (
            [SimpleNamespace(kind="dispatch_sim:trunk:default", state="applied"), _comment_action()],
            False,
        ),
        (
            [
                SimpleNamespace(kind="dispatch_sim:trunk:default", state="applied"),
                SimpleNamespace(kind="sim_result:99", state="applied"),
                _comment_action(),
            ],
            True,
        ),
        (
            [
                SimpleNamespace(kind="dispatch_sim:farm/383-restart:" + "a" * 40, state="proposed"),
                SimpleNamespace(kind="sim_result:99", state="proposed"),
                _comment_action(),
            ],
            True,
        ),
    ]
    for actions, completed in cases:
        agent = _agent(settings, policies, actions)
        assert agent._completed(event, None) is completed


def test_ordinary_followup_completion_requires_comment_only(tmp_path: Path, monkeypatch: Any) -> None:
    settings = _prepared_settings(tmp_path, monkeypatch)
    policies = _shadow_policies()
    event = _comment_event("delivery-complete-ordinary", 384)
    assert _agent(settings, policies, [_comment_action()])._completed(event, None) is True
    assert (
        _agent(
            settings,
            policies,
            [SimpleNamespace(kind="dispatch_sim:trunk:default", state="applied")],
        )._completed(event, None)
        is False
    )
    opened = _opened_event("delivery-complete-opened", 385)
    assert _agent(settings, policies, [_classify_action(), _comment_action()])._completed(opened, None) is True
    assert _agent(settings, policies, [_comment_action()])._completed(opened, None) is False


def test_simulator_directive_reminder_demands_dispatch_result_comment(tmp_path: Path, monkeypatch: Any) -> None:
    agent = _agent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), [])
    event = _sim_directive_event("delivery-reminder-sim", 386)
    reminder = agent._reminder_prompt(event, None)
    assert "dispatch_simulator" in reminder
    assert "get_simulator_result" in reminder
    assert "farm/386-<slug>" in reminder
    assert "[skip ci]" in reminder
    assert "post_issue_comment" in reminder
    incomplete = agent._incomplete_message(event, None)
    assert "dispatch" in incomplete and "result" in incomplete and "comment" in incomplete


def test_simulator_directive_command_enables_write_and_edit(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.command = None
    actions = [
        SimpleNamespace(kind="dispatch_sim:trunk:default", state="applied"),
        SimpleNamespace(kind="sim_result:99", state="applied"),
        _comment_action(),
    ]
    agent = _agent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), actions)
    answer = agent.run(_sim_directive_event("delivery-command-sim", 387))
    assert answer == "done"
    assert FakeRpcClient.command is not None
    assert "read,grep,glob,lsp,bash,write,edit" in FakeRpcClient.command


def test_ordinary_followup_command_stays_read_only(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FakeRpcClient)
    FakeRpcClient.command = None
    agent = _agent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), [_comment_action()])
    answer = agent.run(_comment_event("delivery-command-ordinary", 388))
    assert answer == "done"
    assert FakeRpcClient.command is not None
    assert "read,grep,glob,lsp" in FakeRpcClient.command
    assert "bash" not in FakeRpcClient.command
    assert "write" not in FakeRpcClient.command


def test_pr_review_sim_directive_prompt_is_reproduction_not_review(tmp_path: Path) -> None:
    policy = _shadow_policies().require("dderg/serval")
    event = _pr_sim_directive_event("delivery-pr-prompt-sim", 389)
    pull_request = PullRequestContext.from_event(event)
    assert pull_request is not None
    prompt = TriageAgent._prompt(event, policy, "sota-motion", pull_request)
    assert "A maintainer simulator directive" in prompt
    assert "farm/389-<slug>" in prompt
    assert "dispatch_simulator" in prompt
    assert "Review the exact diff" not in prompt
    assert "post exactly one PR conversation comment" not in prompt


def test_pr_review_sim_directive_reminder_prioritizes_reproduction(tmp_path: Path, monkeypatch: Any) -> None:
    agent = _agent(_prepared_settings(tmp_path, monkeypatch), _shadow_policies(), [])
    event = _pr_sim_directive_event("delivery-pr-reminder-sim", 390)
    pull_request = PullRequestContext.from_event(event)
    assert pull_request is not None
    reminder = agent._reminder_prompt(event, pull_request)
    assert "dispatch_simulator" in reminder
    assert "farm/390-<slug>" in reminder
    incomplete = agent._incomplete_message(event, pull_request)
    assert "simulator directive" in incomplete


def test_host_tool_tracker_waits_for_inflight_executions() -> None:
    tracker = HostToolTracker()
    entered = threading.Event()
    release = threading.Event()

    def slow_execute(args: Any, context: Any) -> str:
        entered.set()
        assert release.wait(5)
        return "ok"

    wrapped = tracker.wrap(slow_execute)
    results: dict[str, str] = {}
    tool_thread = threading.Thread(target=lambda: results.update(value=wrapped({}, None)))
    tool_thread.start()
    assert entered.wait(5)
    drained: list[bool] = []
    waiter = threading.Thread(target=lambda: (tracker.wait_drained(), drained.append(True)))
    waiter.start()
    waiter.join(0.3)
    assert waiter.is_alive(), "wait_drained returned while a tool was still executing"
    release.set()
    tool_thread.join(5)
    waiter.join(5)
    assert not waiter.is_alive()
    assert drained == [True]
    assert results == {"value": "ok"}


def test_host_tool_tracker_is_drained_when_idle() -> None:
    tracker = HostToolTracker()
    tracker.wait_drained()
    wrapped = tracker.wrap(lambda args, context: "done")
    assert wrapped({}, None) == "done"
    tracker.wait_drained()


def test_host_tool_tracker_refuses_admission_after_close() -> None:
    tracker = HostToolTracker()
    executed: list[Any] = []
    wrapped = tracker.wrap(lambda args, context: executed.append(args) or "ok")
    assert wrapped({"query": "first"}, None) == "ok"
    assert executed == [{"query": "first"}]
    tracker.close_and_drain()
    with pytest.raises(RuntimeError, match="admission is closed"):
        wrapped({"query": "late"}, None)
    assert executed == [{"query": "first"}]


def test_host_tool_tracker_close_waits_for_admitted_callbacks() -> None:
    tracker = HostToolTracker()
    entered = threading.Event()
    release = threading.Event()

    def slow_execute(args: Any, context: Any) -> str:
        entered.set()
        assert release.wait(5)
        return "ok"

    wrapped = tracker.wrap(slow_execute)
    results: dict[str, str] = {}
    tool_thread = threading.Thread(target=lambda: results.update(value=wrapped({}, None)))
    tool_thread.start()
    assert entered.wait(5)
    closed: list[bool] = []
    closer = threading.Thread(target=lambda: (tracker.close_and_drain(), closed.append(True)))
    closer.start()
    closer.join(0.3)
    assert closer.is_alive(), "close_and_drain returned while an admitted tool was still executing"
    release.set()
    tool_thread.join(5)
    closer.join(5)
    assert not closer.is_alive()
    assert closed == [True]
    assert results == {"value": "ok"}


class FiringRpcClient(FakeRpcClient):
    """Fires a host tool call on a daemon thread, exactly like omp-rpc does."""

    tool_finished: ClassVar[threading.Event] = threading.Event()

    def __init__(
        self,
        *,
        command: tuple[str, ...],
        request_timeout: float,
        custom_tools: tuple[Any, ...] = (),
        **_: Any,
    ):
        super().__init__(command=command, request_timeout=request_timeout)
        self.custom_tools = custom_tools

    def prompt_and_wait(self, prompt: str, *, timeout: float) -> SimpleNamespace:
        tool = next(tool for tool in self.custom_tools if tool.name == "search_issues")

        def run_tool() -> None:
            try:
                tool.execute({"query": "duplicate"}, object())
            except Exception:
                pass
            finally:
                type(self).tool_finished.set()

        threading.Thread(target=run_tool, daemon=True).start()
        return SimpleNamespace(assistant_text="done")


def test_run_waits_for_inflight_host_tool_before_returning(tmp_path: Path, monkeypatch: Any) -> None:
    monkeypatch.setattr(agent_module, "RpcClient", FiringRpcClient)
    entered = threading.Event()
    release = threading.Event()

    def blocked_search_issues(self: Any, query: str) -> str:
        entered.set()
        assert release.wait(10)
        return "[]"

    monkeypatch.setattr(ActionGateway, "search_issues", blocked_search_issues)
    FiringRpcClient.tool_finished = threading.Event()
    agent = TriageAgent(
        _prepared_settings(tmp_path, monkeypatch),
        _shadow_policies(),
        FakeDatabase(actions=[_comment_action()]),
        None,
    )
    outcome: dict[str, str] = {}
    runner = threading.Thread(
        target=lambda: outcome.update(result=agent.run(_comment_event("delivery-tool-drain", 372)))
    )
    runner.start()
    try:
        assert entered.wait(5)
        runner.join(0.3)
        assert runner.is_alive(), "run returned while a host tool was still executing"
    finally:
        release.set()
    runner.join(10)
    assert not runner.is_alive()
    assert outcome.get("result") == "done"
    assert FiringRpcClient.tool_finished.is_set()
