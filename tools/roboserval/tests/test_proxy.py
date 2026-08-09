import json
import re
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import httpx
import pytest

from serval_bot.auth import SIGNATURE_HEADER, TIMESTAMP_HEADER, sign
from serval_bot.config import ProxySettings
from serval_bot.policy import PolicySet
from serval_bot.proxy import (
    DispatchRequest,
    GitHubApi,
    GitHubFailure,
    PollRequest,
    SimResultRequest,
    WorkspaceRequest,
    create_proxy_app,
)


class StaticTokenProvider:
    async def token(self) -> str:
        return "installation-token"

    async def close(self) -> None:
        return None


class FakeGitHubApi:
    def __init__(self) -> None:
        self.labels: list[str] = []
        self.polls: list[Any] = []
        self.calls: list[str] = []

    async def close(self) -> None:
        return None

    async def sync_workspace(self, request) -> dict[str, Any]:
        self.calls.append("sync_workspace")
        return {"workspace": request.repo.replace("/", "--"), "default_branch": "trunk"}

    async def add_labels(self, request) -> dict[str, Any]:
        self.calls.append("add_labels")
        self.labels = request.labels
        return {"labels": request.labels}

    async def post_comment(self, request) -> dict[str, Any]:
        self.calls.append("post_comment")
        return {"id": 1, "url": "https://example.test/comment"}

    async def search_issues(self, request) -> dict[str, Any]:
        self.calls.append("search_issues")
        return {"items": []}

    async def poll_events(self, request) -> dict[str, Any]:
        self.calls.append("poll_events")
        self.polls.append(request)
        return {"events": []}

    async def dispatch_sim(self, request) -> dict[str, Any]:
        self.calls.append("dispatch_sim")
        return {"run_id": 1, "url": "https://example.test/run", "status": "queued", "conclusion": None}

    async def sim_result(self, request) -> dict[str, Any]:
        self.calls.append("sim_result")
        return {"run_id": request.run_id, "status": "completed", "conclusion": "success", "jobs": []}


_POLICY_TOML = """
[repositories."dderg/serval"]
mode = "triage"
bot_login = "roboserval"
maintainers = ["dderg"]
sim_workflow = "ci-sim-e2e.yaml"
"""

_SHADOW_POLICY_TOML = """
[repositories."dderg/serval"]
mode = "shadow"
"""


def _settings(policy_toml: str = _POLICY_TOML) -> ProxySettings:
    return ProxySettings(
        github_token_path=Path("/tmp/token"),
        hmac_key="proxy-secret",
        bind_host="127.0.0.1",
        bind_port=8081,
        max_log_bytes=20_000,
        workspace_root=Path("/tmp/workspaces"),
        policy=PolicySet.parse(policy_toml),
    )


def _signed(path: str, payload: dict) -> tuple[bytes, dict[str, str]]:
    body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode()
    timestamp, signature = sign("POST", path, body, "proxy-secret")
    return body, {TIMESTAMP_HEADER: timestamp, SIGNATURE_HEADER: signature, "content-type": "application/json"}


@pytest.mark.asyncio
async def test_proxy_requires_valid_signature() -> None:
    app = create_proxy_app(_settings(), FakeGitHubApi())
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(
            "/github/add-labels",
            json={"repo": "dderg/serval", "issue_number": 7, "labels": ["bug"]},
        )
    assert response.status_code == 401, response.text


_ENDPOINT_PAYLOADS = {
    "/github/sync-workspace": {"repo": "dderg/serval"},
    "/github/add-labels": {"repo": "dderg/serval", "issue_number": 7, "labels": ["bug"]},
    "/github/comment": {"repo": "dderg/serval", "issue_number": 7, "body": "hello"},
    "/github/search-issues": {"repo": "dderg/serval", "query": "is:open"},
    "/github/poll-events": {"repo": "dderg/serval", "since": "2026-08-09T12:00:00Z", "bot_login": "roboserval"},
    "/github/dispatch-sim": {"repo": "dderg/serval", "workflow": "ci-sim-e2e.yaml", "ref": "sota-motion"},
    "/github/sim-result": {"repo": "dderg/serval", "run_id": 42},
}


@pytest.mark.asyncio
@pytest.mark.parametrize(("path", "payload"), _ENDPOINT_PAYLOADS.items())
async def test_proxy_triage_allows_all_endpoint_classes(path: str, payload: dict) -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(), api)
    body, headers = _signed(path, payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(path, content=body, headers=headers)
    assert response.status_code == 200, response.text


@pytest.mark.asyncio
@pytest.mark.parametrize(("path", "payload"), _ENDPOINT_PAYLOADS.items())
async def test_proxy_rejects_unknown_repo_with_valid_hmac(path: str, payload: dict) -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(), api)
    body, headers = _signed(path, {**payload, "repo": "other/repo"})
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(path, content=body, headers=headers)
    assert response.status_code == 403, response.text
    assert api.calls == []


@pytest.mark.asyncio
async def test_proxy_verifies_hmac_before_repo_allowlist() -> None:
    app = create_proxy_app(_settings(), FakeGitHubApi())
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(
            "/github/add-labels",
            json={"repo": "other/repo", "issue_number": 7, "labels": ["bug"]},
        )
    assert response.status_code == 401, response.text


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("path", "payload"),
    {
        "/github/add-labels": _ENDPOINT_PAYLOADS["/github/add-labels"],
        "/github/comment": _ENDPOINT_PAYLOADS["/github/comment"],
        "/github/dispatch-sim": _ENDPOINT_PAYLOADS["/github/dispatch-sim"],
        "/github/sim-result": _ENDPOINT_PAYLOADS["/github/sim-result"],
    }.items(),
)
async def test_proxy_shadow_mode_blocks_mutations_and_simulator(path: str, payload: dict) -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(_SHADOW_POLICY_TOML), api)
    body, headers = _signed(path, payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(path, content=body, headers=headers)
    assert response.status_code == 403, response.text
    assert api.calls == []


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("path", "payload"),
    {
        "/github/sync-workspace": _ENDPOINT_PAYLOADS["/github/sync-workspace"],
        "/github/search-issues": _ENDPOINT_PAYLOADS["/github/search-issues"],
        "/github/poll-events": _ENDPOINT_PAYLOADS["/github/poll-events"],
    }.items(),
)
async def test_proxy_shadow_mode_allows_read_sync_search(path: str, payload: dict) -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(_SHADOW_POLICY_TOML), api)
    body, headers = _signed(path, payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(path, content=body, headers=headers)
    assert response.status_code == 200, response.text


@pytest.mark.asyncio
async def test_proxy_rejects_malformed_repo_syntax_before_allowlist() -> None:
    app = create_proxy_app(_settings(), FakeGitHubApi())
    body, headers = _signed("/github/sync-workspace", {"repo": "not-a-repo"})
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/github/sync-workspace", content=body, headers=headers)
    assert response.status_code == 422, response.text


@pytest.mark.asyncio
async def test_proxy_applies_signed_label_request() -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(), api)
    payload = {"repo": "dderg/serval", "issue_number": 7, "labels": ["bug"]}
    body, headers = _signed("/github/add-labels", payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/github/add-labels", content=body, headers=headers)
    assert response.status_code == 200, response.text
    assert api.labels == ["bug"]


@pytest.mark.asyncio
async def test_proxy_sync_request_has_no_configured_branch() -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(), api)
    payload = {"repo": "dderg/serval"}
    body, headers = _signed("/github/sync-workspace", payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/github/sync-workspace", content=body, headers=headers)
    assert response.status_code == 200, response.text
    assert response.json()["default_branch"] == "trunk"


@pytest.mark.asyncio
async def test_proxy_applies_signed_poll_request() -> None:
    api = FakeGitHubApi()
    app = create_proxy_app(_settings(), api)
    payload = {"repo": "dderg/serval", "since": "2026-08-09T12:00:00Z", "bot_login": "roboserval"}
    body, headers = _signed("/github/poll-events", payload)
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post("/github/poll-events", content=body, headers=headers)
    assert response.status_code == 200, response.text
    assert api.polls[0].bot_login == "roboserval"


@pytest.mark.asyncio
async def test_github_sync_resolves_repository_default_branch(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str, str]] = []

    def sync(_, repo: str, branch: str, token: str, **kwargs) -> Path:
        calls.append((repo, branch, token))
        assert kwargs == {"fetch_ref": None, "expected_sha": None}
        return tmp_path / "dderg--serval"

    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.path == "/repos/dderg/serval"
        return httpx.Response(200, json={"default_branch": "trunk"})

    monkeypatch.setattr("serval_bot.proxy.CredentialedWorkspace.sync", sync)
    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler), tmp_path)
    try:
        result = await api.sync_workspace(WorkspaceRequest(repo="dderg/serval"))
    finally:
        await api.close()

    assert result == {"workspace": "dderg--serval", "default_branch": "trunk", "head_sha": None}
    assert calls == [("dderg/serval", "trunk", "installation-token")]


@pytest.mark.asyncio
async def test_github_sync_rejects_missing_default_branch(tmp_path: Path) -> None:
    api = GitHubApi(
        StaticTokenProvider(),
        20_000,
        httpx.MockTransport(lambda _: httpx.Response(200, json={})),
        tmp_path,
    )
    try:
        with pytest.raises(GitHubFailure, match="no default branch"):
            await api.sync_workspace(WorkspaceRequest(repo="dderg/serval"))
    finally:
        await api.close()


@pytest.mark.asyncio
async def test_sim_result_includes_failed_job_log() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/actions/runs/42"):
            return httpx.Response(
                200,
                json={
                    "id": 42,
                    "status": "completed",
                    "conclusion": "failure",
                    "html_url": "https://example.test/run/42",
                    "head_sha": "a" * 40,
                    "head_branch": "trunk",
                    "path": ".github/workflows/ci-sim-e2e.yaml",
                },
            )
        if request.url.path.endswith("/actions/runs/42/jobs"):
            return httpx.Response(
                200,
                json={
                    "jobs": [
                        {
                            "id": 9,
                            "name": "sim-e2e (probe)",
                            "status": "completed",
                            "conclusion": "failure",
                            "html_url": "https://example.test/job/9",
                        }
                    ]
                },
            )
        if request.url.path.endswith("/actions/jobs/9/logs"):
            return httpx.Response(200, content=b"assertion failed\n")

    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler))

    try:
        result = await api.sim_result(SimResultRequest(repo="dderg/serval", run_id=42))
    finally:
        await api.close()
    assert result["conclusion"] == "failure"
    assert result["workflow"] == ".github/workflows/ci-sim-e2e.yaml"
    assert result["ref"] == "trunk"
    assert result["jobs"][0]["failure_log"] == "assertion failed\n"


@pytest.mark.asyncio
async def test_dispatch_sim_correlates_exactly_one_new_run() -> None:
    sequence: list[str] = []
    runs_calls = {"count": 0}
    token = {"value": None}

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/dispatches"):
            sequence.append("post")
            assert request.method == "POST"
            body = json.loads(request.content)
            assert body["ref"] == "trunk"
            assert re.fullmatch(r"[0-9a-f]{32}", body["inputs"]["serval_dispatch_id"])
            token["value"] = body["inputs"]["serval_dispatch_id"]
            return httpx.Response(204)
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/runs"):
            sequence.append("runs")
            assert request.method == "GET"
            assert request.url.params["branch"] == "trunk"
            assert request.url.params["event"] == "workflow_dispatch"
            assert request.url.params["per_page"] == "100"
            runs_calls["count"] += 1
            if runs_calls["count"] <= 2:  # baseline snapshot, then first poll
                return httpx.Response(200, json={"workflow_runs": []})
            # Run 78 appears concurrently but lacks the dispatch token and is
            # on a different head sha, so it never qualifies; only run 77
            # carries the echoed token and matches the requested head sha.
            return httpx.Response(
                200,
                json={
                    "workflow_runs": [
                        {
                            "id": 78,
                            "status": "queued",
                            "conclusion": None,
                            "html_url": "https://example.test/run/78",
                            "display_title": "Manual dispatch",
                            "head_sha": "c" * 40,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                        },
                        {
                            "id": 77,
                            "status": "queued",
                            "conclusion": None,
                            "html_url": "https://example.test/run/77",
                            "display_title": f"serval-{token['value']}",
                            "head_sha": "b" * 40,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                        },
                    ]
                },
            )
        raise AssertionError(f"unexpected GitHub request: {request.url}")

    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler))
    try:
        result = await api.dispatch_sim(
            DispatchRequest(
                repo="dderg/serval",
                workflow="ci-sim-e2e.yaml",
                ref="trunk",
                head_sha="b" * 40,
            )
        )
    finally:
        await api.close()
    # The baseline snapshot must be taken before the dispatch POST, and the
    # dispatch POST must carry the fresh random token as the input.
    assert sequence == ["runs", "post", "runs", "runs"]
    assert token["value"] is not None
    assert result == {
        "run_id": 77,
        "url": "https://example.test/run/77",
        "status": "queued",
        "conclusion": None,
        "head_sha": "b" * 40,
        "ref": "trunk",
        "workflow": ".github/workflows/ci-sim-e2e.yaml",
    }


@pytest.mark.asyncio
async def test_dispatch_sim_fails_on_ambiguous_new_runs() -> None:
    runs_calls = {"count": 0}
    token = {"value": None}

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/dispatches"):
            token["value"] = json.loads(request.content)["inputs"]["serval_dispatch_id"]
            return httpx.Response(204)
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/runs"):
            runs_calls["count"] += 1
            if runs_calls["count"] == 1:  # baseline: nothing preexisting
                return httpx.Response(200, json={"workflow_runs": []})
            run_name = f"serval-{token['value']}"
            return httpx.Response(
                200,
                json={
                    "workflow_runs": [
                        {
                            "id": 77,
                            "status": "queued",
                            "html_url": "https://example.test/run/77",
                            "display_title": run_name,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                        },
                        {
                            "id": 78,
                            "status": "queued",
                            "html_url": "https://example.test/run/78",
                            "display_title": run_name,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                        },
                    ]
                },
            )
        raise AssertionError(f"unexpected GitHub request: {request.url}")

    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler))
    try:
        with pytest.raises(GitHubFailure, match="ambiguous"):
            await api.dispatch_sim(DispatchRequest(repo="dderg/serval", workflow="ci-sim-e2e.yaml", ref="trunk"))
    finally:
        await api.close()


@pytest.mark.asyncio
async def test_dispatch_sim_rejects_external_run_without_token() -> None:
    runs_calls = {"count": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/dispatches"):
            return httpx.Response(204)
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/runs"):
            runs_calls["count"] += 1
            if runs_calls["count"] == 1:  # baseline: nothing preexisting
                return httpx.Response(200, json={"workflow_runs": []})
            # A manual dispatch lands on the same branch and head sha after the
            # baseline snapshot: it is the only new run, yet it does not carry
            # the random token, so it must never be correlated to this dispatch.
            return httpx.Response(
                200,
                json={
                    "workflow_runs": [
                        {
                            "id": 9,
                            "status": "queued",
                            "conclusion": None,
                            "html_url": "https://example.test/run/9",
                            "display_title": "Manual dispatch",
                            "head_sha": "b" * 40,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                        }
                    ]
                },
            )
        raise AssertionError(f"unexpected GitHub request: {request.url}")

    api = GitHubApi(
        StaticTokenProvider(),
        20_000,
        httpx.MockTransport(handler),
        dispatch_poll_attempts=2,
        dispatch_poll_interval=0,
    )
    try:
        with pytest.raises(GitHubFailure, match="did not appear"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", workflow="ci-sim-e2e.yaml", ref="trunk", head_sha="b" * 40)
            )
    finally:
        await api.close()


@pytest.mark.asyncio
async def test_dispatch_sim_excludes_preexisting_run_despite_fresh_timestamps() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/dispatches"):
            return httpx.Response(204)
        if request.url.path.endswith("/workflows/ci-sim-e2e.yaml/runs"):
            # The preexisting run is served with a freshly generated created_at
            # on every poll, so a wall-clock comparison would misclassify it as
            # new. Baseline id membership excludes it regardless of timestamps,
            # clock skew, or even a token-shaped title.
            return httpx.Response(
                200,
                json={
                    "workflow_runs": [
                        {
                            "id": 5,
                            "status": "queued",
                            "conclusion": None,
                            "html_url": "https://example.test/run/5",
                            "display_title": "serval-deadbeef",
                            "head_sha": "b" * 40,
                            "head_branch": "trunk",
                            "path": ".github/workflows/ci-sim-e2e.yaml",
                            "created_at": datetime.now(UTC).isoformat(),
                        }
                    ]
                },
            )
        raise AssertionError(f"unexpected GitHub request: {request.url}")

    api = GitHubApi(
        StaticTokenProvider(),
        20_000,
        httpx.MockTransport(handler),
        dispatch_poll_attempts=2,
        dispatch_poll_interval=0,
    )
    try:
        with pytest.raises(GitHubFailure, match="did not appear"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", workflow="ci-sim-e2e.yaml", ref="trunk", head_sha="b" * 40)
            )
    finally:
        await api.close()


@pytest.mark.asyncio
async def test_github_poll_returns_only_new_issues_and_mentioned_comments() -> None:
    issue = {
        "id": 10,
        "number": 5,
        "title": "new failure",
        "body": "details",
        "created_at": "2026-08-09T12:01:00Z",
        "updated_at": "2026-08-09T12:01:00Z",
        "user": {"login": "reporter"},
    }
    old_issue = {
        **issue,
        "id": 11,
        "number": 6,
        "created_at": "2026-08-08T12:00:00Z",
        "updated_at": "2026-08-09T12:02:00Z",
    }
    pull_parent = {**issue, "id": 14, "number": 8, "pull_request": {}}
    pull_request = {
        "number": 8,
        "title": "Fix LEDs",
        "body": "Change priority dispatch",
        "html_url": "https://github.com/dderg/serval/pull/8",
        "base": {"ref": "sota-motion", "sha": "a" * 40},
        "head": {"ref": "fix-leds", "sha": "b" * 40},
    }
    parent = {**issue, "id": 12, "number": 7}
    mentioned = {
        "id": 41,
        "body": "@roboserval investigate",
        "created_at": "2026-08-09T12:02:00Z",
        "updated_at": "2026-08-09T12:02:00Z",
        "issue_url": "https://api.github.com/repos/dderg/serval/issues/7",
        "user": {"login": "maintainer"},
    }
    pull_mentioned = {
        **mentioned,
        "id": 45,
        "created_at": "2026-08-09T12:03:00Z",
        "updated_at": "2026-08-09T12:03:00Z",
        "issue_url": "https://api.github.com/repos/dderg/serval/issues/8",
    }
    requested_pages: list[str | None] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/repos/dderg/serval/issues":
            return httpx.Response(200, json=[issue, old_issue, pull_parent])
        if request.url.path == "/repos/dderg/serval/issues/comments":
            page = request.url.params.get("page")
            requested_pages.append(page)
            if page == "2":
                return httpx.Response(200, json=[])
            ignored = {**mentioned, "id": 42, "body": "no mention"}
            bot_authored = {**mentioned, "id": 43, "user": {"login": "roboserval"}}
            edited_old = {**mentioned, "id": 44, "created_at": "2026-08-08T12:00:00Z"}
            return httpx.Response(
                200,
                json=[mentioned, pull_mentioned, ignored, bot_authored, edited_old],
                headers={"link": '<https://api.github.com/repos/dderg/serval/issues/comments?page=2>; rel="next"'},
            )
        if request.url.path == "/repos/dderg/serval/issues/7":
            return httpx.Response(200, json=parent)
        if request.url.path == "/repos/dderg/serval/issues/8":
            return httpx.Response(200, json=pull_parent)
        if request.url.path == "/repos/dderg/serval/pulls/8":
            return httpx.Response(200, json=pull_request)
        raise AssertionError(f"unexpected GitHub request: {request.url}")

    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler))
    request = PollRequest(
        repo="dderg/serval",
        since=datetime(2026, 8, 9, 12, 0, tzinfo=UTC),
        bot_login="roboserval",
    )
    try:
        result = await api.poll_events(request)
    finally:
        await api.close()
    assert [event["delivery_id"] for event in result["events"]] == [
        "poll:issue:10:opened",
        "poll:comment:41:created",
        "poll:comment:45:created",
    ]
    review_event = result["events"][2]
    assert review_event["event_type"] == "pull_request_review.requested"
    assert review_event["payload"]["pull_request"]["head"]["sha"] == "b" * 40
    assert requested_pages == [None, "2"]
