import json
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
from serval_bot.workspace import WorkspaceFailure


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
    "/github/dispatch-sim": {
        "repo": "dderg/serval",
        "issue_number": 7,
        "workflow": "ci-sim-e2e.yaml",
        "ref": "sota-motion",
    },
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


class _DispatchHarness:
    """Mock GitHub API around an ephemeral-ref workflow dispatch."""

    def __init__(
        self,
        *,
        ref: str = "trunk",
        head_sha: str = "b" * 40,
        poll_attempts: int = 30,
        poll_interval: float = 1.0,
    ) -> None:
        self.ref = ref
        self.head_sha = head_sha
        self.poll_attempts = poll_attempts
        self.poll_interval = poll_interval
        self.sequence: list[str] = []
        self.temp_ref: str | None = None
        self.dispatched_on: str | None = None
        self.dispatch_error: int | None = None
        self.delete_error: int | None = None
        self.runs_pages: list[list[dict[str, Any]]] = []
        self.runs_calls = 0

    def api(self) -> GitHubApi:
        return GitHubApi(
            StaticTokenProvider(),
            20_000,
            httpx.MockTransport(self.handler),
            dispatch_poll_attempts=self.poll_attempts,
            dispatch_poll_interval=self.poll_interval,
        )

    def run(self, run_id: int) -> dict[str, Any]:
        return {
            "id": run_id,
            "status": "queued",
            "conclusion": None,
            "html_url": f"https://example.test/run/{run_id}",
            "head_sha": self.head_sha,
            "head_branch": self.temp_ref,
            "path": ".github/workflows/ci-sim-e2e.yaml",
        }

    def handler(self, request: httpx.Request) -> httpx.Response:
        path = request.url.path
        if path.endswith(f"/git/ref/heads/{self.ref}"):
            self.sequence.append("resolve")
            return httpx.Response(
                200,
                json={"ref": "refs/heads/trunk", "object": {"sha": self.head_sha, "type": "commit"}},
            )
        if path.endswith("/git/refs") and request.method == "POST":
            self.sequence.append("create")
            body = json.loads(request.content)
            assert body["ref"].startswith("refs/heads/serval-")
            assert body["sha"] == self.head_sha
            self.temp_ref = body["ref"].removeprefix("refs/heads/")
            return httpx.Response(201, json={"ref": body["ref"]})
        if request.method == "DELETE" and path.endswith(f"/git/refs/heads/{self.temp_ref}"):
            self.sequence.append("delete")
            if self.delete_error is not None:
                return httpx.Response(self.delete_error, json={"message": "Reference does not exist"})
            return httpx.Response(204)
        if path.endswith("/workflows/ci-sim-e2e.yaml/dispatches"):
            self.sequence.append("dispatch")
            body = json.loads(request.content)
            assert body == {"ref": self.temp_ref}
            self.dispatched_on = body["ref"]
            if self.dispatch_error is not None:
                return httpx.Response(self.dispatch_error, json={"message": "dispatch failed"})
            return httpx.Response(204)
        if path.endswith("/workflows/ci-sim-e2e.yaml/runs"):
            self.sequence.append("runs")
            assert request.url.params["branch"] == self.temp_ref
            assert request.url.params["event"] == "workflow_dispatch"
            assert request.url.params["per_page"] == "100"
            page = self.runs_pages[min(self.runs_calls, len(self.runs_pages) - 1)] if self.runs_pages else []
            self.runs_calls += 1
            return httpx.Response(200, json={"workflow_runs": page})
        raise AssertionError(f"unexpected GitHub request: {request.url} {request.method}")


@pytest.mark.asyncio
async def test_dispatch_sim_create_dispatch_poll_delete_ordering() -> None:
    harness = _DispatchHarness()
    harness.runs_pages = [[], [], [harness.run(77)]]
    api = harness.api()
    try:
        result = await api.dispatch_sim(
            DispatchRequest(
                repo="dderg/serval",
                issue_number=7,
                workflow="ci-sim-e2e.yaml",
                ref="trunk",
                head_sha="b" * 40,
            )
        )
    finally:
        await api.close()
    # Resolve the ref, create the temporary branch at its exact sha, dispatch
    # on that unique branch with no inputs, poll it, then delete the branch.
    assert harness.sequence == ["resolve", "create", "runs", "dispatch", "runs", "runs", "delete"]
    assert harness.dispatched_on == harness.temp_ref
    assert result["run_id"] == 77
    assert result["ref"] == harness.temp_ref
    assert result["requested_ref"] == "trunk"
    assert result["head_sha"] == "b" * 40
    assert result["workflow"] == ".github/workflows/ci-sim-e2e.yaml"


@pytest.mark.asyncio
async def test_dispatch_sim_fails_on_ambiguous_new_runs() -> None:
    harness = _DispatchHarness()
    harness.runs_pages = [[], [harness.run(77), harness.run(78)]]
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="ambiguous"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="trunk")
            )
    finally:
        await api.close()
    assert "delete" in harness.sequence


@pytest.mark.asyncio
async def test_dispatch_sim_polls_only_the_temp_ref() -> None:
    harness = _DispatchHarness(poll_attempts=2, poll_interval=0)
    harness.runs_pages = [[]]
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="did not appear"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="trunk")
            )
    finally:
        await api.close()
    # Every runs request was scoped to the temporary branch (asserted inside
    # the harness), so a run on the original ref can never qualify, and the
    # temporary branch is deleted even though no run appeared.
    assert harness.sequence == ["resolve", "create", "runs", "dispatch", "runs", "runs", "delete"]


@pytest.mark.asyncio
async def test_dispatch_sim_verifies_supplied_head_sha() -> None:
    harness = _DispatchHarness()
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="moved"):
            await api.dispatch_sim(
                DispatchRequest(
                    repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="trunk", head_sha="c" * 40
                )
            )
    finally:
        await api.close()
    assert harness.sequence == ["resolve"]


@pytest.mark.asyncio
async def test_dispatch_sim_deletes_temp_ref_on_dispatch_error() -> None:
    harness = _DispatchHarness()
    harness.dispatch_error = 500
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="dispatch failed"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="trunk")
            )
    finally:
        await api.close()
    assert harness.sequence == ["resolve", "create", "runs", "dispatch", "delete"]


@pytest.mark.asyncio
async def test_dispatch_sim_delete_failure_is_loud() -> None:
    harness = _DispatchHarness()
    harness.runs_pages = [[], [harness.run(77)]]
    harness.delete_error = 422
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="DELETE"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="trunk")
            )
    finally:
        await api.close()


def _fake_publish(harness: _DispatchHarness, *, sha: str = "b" * 40, error: str | None = None):
    def publish(self, repo, issue_number, token, *, ref, expected_sha) -> None:
        harness.sequence.append("publish")
        assert repo == "dderg/serval"
        assert issue_number == 7
        assert token == "installation-token"
        assert ref == "farm/7-calib"
        assert expected_sha == sha
        if error is not None:
            raise WorkspaceFailure(error)

    return publish


@pytest.mark.asyncio
async def test_dispatch_sim_farm_publishes_before_correlating(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = _DispatchHarness(ref="farm/7-calib")
    harness.runs_pages = [[], [], [harness.run(77)]]
    monkeypatch.setattr("serval_bot.proxy.CredentialedWorkspace.publish_issue", _fake_publish(harness))
    api = harness.api()
    try:
        result = await api.dispatch_sim(
            DispatchRequest(
                repo="dderg/serval",
                issue_number=7,
                workflow="ci-sim-e2e.yaml",
                ref="farm/7-calib",
                head_sha="b" * 40,
            )
        )
    finally:
        await api.close()
    # Publish the exact issue-workspace commit, resolve it, create the
    # temporary branch at that commit, dispatch on it, poll it, then delete it.
    assert harness.sequence == ["publish", "resolve", "create", "runs", "dispatch", "runs", "runs", "delete"]
    assert harness.dispatched_on == harness.temp_ref
    assert result["run_id"] == 77
    assert result["requested_ref"] == "farm/7-calib"
    assert result["head_sha"] == "b" * 40
    assert result["ref"] == harness.temp_ref


@pytest.mark.asyncio
async def test_dispatch_sim_farm_requires_head_sha() -> None:
    harness = _DispatchHarness(ref="farm/7-calib")
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="head_sha"):
            await api.dispatch_sim(
                DispatchRequest(repo="dderg/serval", issue_number=7, workflow="ci-sim-e2e.yaml", ref="farm/7-calib")
            )
    finally:
        await api.close()
    assert harness.sequence == []


@pytest.mark.asyncio
async def test_dispatch_sim_rejects_farm_ref_scoped_to_another_issue() -> None:
    harness = _DispatchHarness(ref="farm/5-calib")
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="scoped to issue 7"):
            await api.dispatch_sim(
                DispatchRequest(
                    repo="dderg/serval",
                    issue_number=7,
                    workflow="ci-sim-e2e.yaml",
                    ref="farm/5-calib",
                    head_sha="b" * 40,
                )
            )
    finally:
        await api.close()
    assert harness.sequence == []


@pytest.mark.asyncio
@pytest.mark.parametrize("ref", ["farm/7-", "farm/7-bad ref"])
async def test_dispatch_sim_rejects_malformed_farm_refs(ref: str) -> None:
    harness = _DispatchHarness(ref=ref)
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure):
            await api.dispatch_sim(
                DispatchRequest(
                    repo="dderg/serval",
                    issue_number=7,
                    workflow="ci-sim-e2e.yaml",
                    ref=ref,
                    head_sha="b" * 40,
                )
            )
    finally:
        await api.close()
    assert harness.sequence == []


@pytest.mark.asyncio
async def test_dispatch_sim_farm_verifies_published_head_sha(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = _DispatchHarness(ref="farm/7-calib")
    monkeypatch.setattr("serval_bot.proxy.CredentialedWorkspace.publish_issue", _fake_publish(harness, sha="c" * 40))
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="moved"):
            await api.dispatch_sim(
                DispatchRequest(
                    repo="dderg/serval",
                    issue_number=7,
                    workflow="ci-sim-e2e.yaml",
                    ref="farm/7-calib",
                    head_sha="c" * 40,
                )
            )
    finally:
        await api.close()
    assert harness.sequence == ["publish", "resolve"]


@pytest.mark.asyncio
async def test_dispatch_sim_publication_failure_is_loud(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = _DispatchHarness(ref="farm/7-calib")
    monkeypatch.setattr(
        "serval_bot.proxy.CredentialedWorkspace.publish_issue",
        _fake_publish(harness, error="issue workspace is not clean:\n?? scratch.txt"),
    )
    api = harness.api()
    try:
        with pytest.raises(GitHubFailure, match="publication failed"):
            await api.dispatch_sim(
                DispatchRequest(
                    repo="dderg/serval",
                    issue_number=7,
                    workflow="ci-sim-e2e.yaml",
                    ref="farm/7-calib",
                    head_sha="b" * 40,
                )
            )
    finally:
        await api.close()
    assert harness.sequence == ["publish"]


@pytest.mark.asyncio
async def test_github_poll_returns_new_issues_mentions_and_reviewer_assignments() -> None:
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
    review_requested = {
        "id": 46,
        "event": "review_requested",
        "created_at": "2026-08-09T12:04:00Z",
        "actor": {"login": "maintainer"},
        "review_requester": {"login": "maintainer"},
        "requested_reviewer": {"login": "roboserval"},
    }
    requested_pages: list[str | None] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/repos/dderg/serval/issues":
            return httpx.Response(200, json=[issue, old_issue, pull_parent])
        if request.url.path == "/repos/dderg/serval/issues/8/events":
            ignored_reviewer = {
                **review_requested,
                "id": 47,
                "requested_reviewer": {"login": "someone-else"},
            }
            return httpx.Response(200, json=[review_requested, ignored_reviewer])
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
        "poll:review:46:requested",
    ]
    mentioned_review = result["events"][2]
    assigned_review = result["events"][3]
    assert mentioned_review["event_type"] == "pull_request_review.requested"
    assert assigned_review["event_type"] == "pull_request_review.requested"
    assert assigned_review["actor"] == "maintainer"
    assert assigned_review["payload"]["pull_request"]["head"]["sha"] == "b" * 40
    assert assigned_review["payload"]["review_request"] == review_requested
    assert requested_pages == [None, "2"]
