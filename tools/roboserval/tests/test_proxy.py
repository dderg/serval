import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import httpx
import pytest

from serval_bot.auth import SIGNATURE_HEADER, TIMESTAMP_HEADER, sign
from serval_bot.config import ProxySettings
from serval_bot.proxy import (
    GitHubApi,
    GitHubFailure,
    PollRequest,
    RepositoryRequest,
    SimResultRequest,
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

    async def close(self) -> None:
        return None

    async def sync_workspace(self, request) -> dict[str, Any]:
        return {"workspace": request.repo.replace("/", "--"), "default_branch": "trunk"}

    async def add_labels(self, request) -> dict[str, Any]:
        self.labels = request.labels
        return {"labels": request.labels}

    async def post_comment(self, request) -> dict[str, Any]:
        return {"id": 1, "url": "https://example.test/comment"}

    async def search_issues(self, request) -> dict[str, Any]:
        return {"items": []}

    async def poll_events(self, request) -> dict[str, Any]:
        self.polls.append(request)
        return {"events": []}

    async def dispatch_sim(self, request) -> dict[str, Any]:
        return {"run_id": 1, "url": "https://example.test/run", "status": "queued", "conclusion": None}

    async def sim_result(self, request) -> dict[str, Any]:
        return {"run_id": request.run_id, "status": "completed", "conclusion": "success", "jobs": []}


def _settings() -> ProxySettings:
    return ProxySettings(Path("/tmp/token"), "proxy-secret", "127.0.0.1", 8081, 20_000, Path("/tmp/workspaces"))


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

    def sync(_, repo: str, branch: str, token: str) -> Path:
        calls.append((repo, branch, token))
        return tmp_path / "dderg--serval"

    def handler(request: httpx.Request) -> httpx.Response:
        assert request.url.path == "/repos/dderg/serval"
        return httpx.Response(200, json={"default_branch": "trunk"})

    monkeypatch.setattr("serval_bot.proxy.CredentialedWorkspace.sync", sync)
    api = GitHubApi(StaticTokenProvider(), 20_000, httpx.MockTransport(handler), tmp_path)
    try:
        result = await api.sync_workspace(RepositoryRequest(repo="dderg/serval"))
    finally:
        await api.close()

    assert result == {"workspace": "dderg--serval", "default_branch": "trunk"}
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
            await api.sync_workspace(RepositoryRequest(repo="dderg/serval"))
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
    assert result["jobs"][0]["failure_log"] == "assertion failed\n"


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
    parent = {**issue, "id": 12, "number": 7}
    mentioned = {
        "id": 41,
        "body": "@roboserval investigate",
        "created_at": "2026-08-09T12:02:00Z",
        "updated_at": "2026-08-09T12:02:00Z",
        "issue_url": "https://api.github.com/repos/dderg/serval/issues/7",
        "user": {"login": "maintainer"},
    }
    requested_pages: list[str | None] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/repos/dderg/serval/issues":
            return httpx.Response(200, json=[issue, old_issue, {**issue, "id": 13, "pull_request": {}}])
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
                json=[mentioned, ignored, bot_authored, edited_old],
                headers={"link": '<https://api.github.com/repos/dderg/serval/issues/comments?page=2>; rel="next"'},
            )
        if request.url.path == "/repos/dderg/serval/issues/7":
            return httpx.Response(200, json=parent)
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
    ]
    assert requested_pages == [None, "2"]
