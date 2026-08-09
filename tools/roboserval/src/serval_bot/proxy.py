from __future__ import annotations

import asyncio
import io
import re
import zipfile
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol

import httpx
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field

from serval_bot.auth import SIGNATURE_HEADER, TIMESTAMP_HEADER, verify
from serval_bot.config import ProxySettings
from serval_bot.token_auth import StaticTokenProvider
from serval_bot.workspace import CredentialedWorkspace

_REPO_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
_WORKFLOW_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+\.ya?ml$")


class GitHubFailure(RuntimeError):
    pass


class RepositoryRequest(BaseModel):
    repo: str


class WorkspaceRequest(RepositoryRequest):
    pull_number: int | None = Field(default=None, gt=0)
    head_sha: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")


class IssueRequest(RepositoryRequest):
    issue_number: int = Field(gt=0)


class AddLabelsRequest(IssueRequest):
    labels: list[str] = Field(min_length=1, max_length=20)


class CommentRequest(IssueRequest):
    body: str = Field(min_length=1, max_length=65_000)


class SearchRequest(RepositoryRequest):
    query: str = Field(min_length=1, max_length=256)


class DispatchRequest(RepositoryRequest):
    workflow: str
    ref: str = Field(min_length=1, max_length=256)
    head_sha: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")


class SimResultRequest(RepositoryRequest):
    run_id: int = Field(gt=0)


class PollRequest(RepositoryRequest):
    since: datetime
    bot_login: str = Field(min_length=1, max_length=39, pattern=r"^[A-Za-z0-9-]+$")


class TokenProvider(Protocol):
    async def token(self) -> str: ...

    async def close(self) -> None: ...


class GitHubApi:
    def __init__(
        self,
        tokens: TokenProvider,
        max_log_bytes: int,
        transport: httpx.AsyncBaseTransport | None = None,
        workspace_root: Path | None = None,
    ):
        self._tokens = tokens
        self._client = httpx.AsyncClient(
            base_url="https://api.github.com",
            headers={
                "accept": "application/vnd.github+json",
                "x-github-api-version": "2022-11-28",
                "user-agent": "serval-bot-proxy",
            },
            follow_redirects=True,
            timeout=120.0,
            transport=transport,
        )
        self._max_log_bytes = max_log_bytes
        self._workspace = CredentialedWorkspace(workspace_root or Path("/data/workspaces"))

    async def close(self) -> None:
        await asyncio.gather(self._client.aclose(), self._tokens.close())

    async def sync_workspace(self, request: WorkspaceRequest) -> dict[str, Any]:
        repository = (await self.request("GET", f"/repos/{request.repo}")).json()
        default_branch = repository.get("default_branch")
        if not isinstance(default_branch, str) or not default_branch:
            raise GitHubFailure(f"repository metadata has no default branch: {request.repo}")
        if (request.pull_number is None) != (request.head_sha is None):
            raise GitHubFailure("pull_number and head_sha must be set together")
        fetch_ref = f"refs/pull/{request.pull_number}/head" if request.pull_number is not None else None
        token = await self._tokens.token()
        path = await asyncio.to_thread(
            self._workspace.sync,
            request.repo,
            default_branch,
            token,
            fetch_ref=fetch_ref,
            expected_sha=request.head_sha,
        )
        return {
            "workspace": path.name,
            "default_branch": default_branch,
            "head_sha": request.head_sha,
        }

    async def request(self, method: str, path: str, **kwargs: Any) -> httpx.Response:
        supplied_headers = kwargs.pop("headers", {})
        headers = {"authorization": f"Bearer {await self._tokens.token()}", **supplied_headers}
        response = await self._client.request(method, path, headers=headers, **kwargs)
        if response.is_error:
            retry = response.headers.get("retry-after")
            detail = response.text[:4000]
            raise GitHubFailure(f"GitHub {method} {path} failed: {response.status_code}, retry={retry}, {detail}")
        return response

    async def add_labels(self, request: AddLabelsRequest) -> dict[str, Any]:
        response = await self.request(
            "POST",
            f"/repos/{request.repo}/issues/{request.issue_number}/labels",
            json={"labels": request.labels},
        )
        return {"labels": [item["name"] for item in response.json()]}

    async def post_comment(self, request: CommentRequest) -> dict[str, Any]:
        response = await self.request(
            "POST",
            f"/repos/{request.repo}/issues/{request.issue_number}/comments",
            json={"body": request.body},
        )
        data = response.json()
        return {"id": data["id"], "url": data["html_url"]}

    async def search_issues(self, request: SearchRequest) -> dict[str, Any]:
        response = await self.request(
            "GET",
            "/search/issues",
            params={"q": f"repo:{request.repo} is:issue {request.query}", "per_page": 10},
        )
        items = response.json().get("items", [])
        return {
            "items": [
                {
                    "number": item["number"],
                    "title": item["title"],
                    "state": item["state"],
                    "url": item["html_url"],
                }
                for item in items
            ]
        }

    async def poll_events(self, request: PollRequest) -> dict[str, Any]:
        since = request.since.astimezone(UTC).isoformat().replace("+00:00", "Z")
        issues = await self._pages(
            f"/repos/{request.repo}/issues",
            {"state": "all", "since": since, "sort": "updated", "direction": "asc", "per_page": 100},
        )
        events = [
            {
                "delivery_id": f"poll:issue:{issue['id']}:opened",
                "event_type": "issues.opened",
                "issue_number": issue["number"],
                "actor": issue["user"]["login"],
                "occurred_at": issue["created_at"],
                "payload": {
                    "action": "opened",
                    "repository": {"full_name": request.repo},
                    "sender": issue["user"],
                    "issue": issue,
                },
            }
            for issue in issues
            if "pull_request" not in issue
            and issue["created_at"] >= since
            and _normalize_login(issue["user"]["login"]) != _normalize_login(request.bot_login)
        ]
        comments = await self._pages(
            f"/repos/{request.repo}/issues/comments",
            {"since": since, "sort": "updated", "direction": "asc", "per_page": 100},
        )
        for comment in comments:
            actor = comment["user"]["login"]
            if (
                comment["created_at"] < since
                or _normalize_login(actor) == _normalize_login(request.bot_login)
                or not _mention(comment["body"], request.bot_login)
            ):
                continue
            issue_number = _issue_number(comment["issue_url"])
            issue = (await self.request("GET", f"/repos/{request.repo}/issues/{issue_number}")).json()
            pull_request = None
            event_type = "issue_comment.created"
            if "pull_request" in issue:
                pull_request = (await self.request("GET", f"/repos/{request.repo}/pulls/{issue_number}")).json()
                event_type = "pull_request_review.requested"
            payload = {
                "action": "created",
                "repository": {"full_name": request.repo},
                "sender": comment["user"],
                "issue": issue,
                "comment": comment,
            }
            if pull_request is not None:
                payload["pull_request"] = pull_request
            events.append(
                {
                    "delivery_id": f"poll:comment:{comment['id']}:created",
                    "event_type": event_type,
                    "issue_number": issue_number,
                    "actor": actor,
                    "occurred_at": comment["created_at"],
                    "payload": payload,
                }
            )
        events.sort(key=lambda event: (event["occurred_at"], event["delivery_id"]))
        return {"events": events}

    async def _pages(self, path: str, params: dict[str, Any]) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        next_path: str | None = path
        next_params: dict[str, Any] | None = params
        while next_path is not None:
            response = await self.request("GET", next_path, params=next_params)
            page = response.json()
            if not isinstance(page, list):
                raise GitHubFailure(f"GitHub GET {next_path} returned a non-list response")
            items.extend(page)
            next_path = response.links.get("next", {}).get("url")
            next_params = None
        return items

    async def dispatch_sim(self, request: DispatchRequest) -> dict[str, Any]:
        started = datetime.now(UTC)
        await self.request(
            "POST",
            f"/repos/{request.repo}/actions/workflows/{request.workflow}/dispatches",
            json={"ref": request.ref},
        )
        for _ in range(30):
            response = await self.request(
                "GET",
                f"/repos/{request.repo}/actions/workflows/{request.workflow}/runs",
                params={"branch": request.ref, "event": "workflow_dispatch", "per_page": 10},
            )
            for run in response.json().get("workflow_runs", []):
                created = datetime.fromisoformat(run["created_at"])
                if created < started:
                    continue
                if request.head_sha is not None and run.get("head_sha") != request.head_sha:
                    continue
                return {
                    "run_id": run["id"],
                    "url": run["html_url"],
                    "status": run["status"],
                    "conclusion": run.get("conclusion"),
                    "head_sha": run.get("head_sha"),
                }
            await asyncio.sleep(1)
        raise GitHubFailure("dispatched workflow run did not appear within 30 seconds")

    async def sim_result(self, request: SimResultRequest) -> dict[str, Any]:
        run_response, jobs_response = await asyncio.gather(
            self.request("GET", f"/repos/{request.repo}/actions/runs/{request.run_id}"),
            self.request(
                "GET",
                f"/repos/{request.repo}/actions/runs/{request.run_id}/jobs",
                params={"per_page": 100},
            ),
        )
        run = run_response.json()
        jobs = jobs_response.json().get("jobs", [])
        rendered_jobs = []
        remaining = self._max_log_bytes
        for job in jobs:
            failure_log = ""
            if job.get("conclusion") not in {None, "success", "skipped"} and remaining > 0:
                failure_log = await self._job_log(request.repo, int(job["id"]), remaining)
                remaining -= len(failure_log.encode())
            rendered_jobs.append(
                {
                    "id": job["id"],
                    "name": job["name"],
                    "status": job["status"],
                    "conclusion": job.get("conclusion"),
                    "url": job["html_url"],
                    "failure_log": failure_log,
                }
            )
        return {
            "run_id": run["id"],
            "status": run["status"],
            "conclusion": run.get("conclusion"),
            "url": run["html_url"],
            "head_sha": run.get("head_sha"),
            "jobs": rendered_jobs,
        }

    async def _job_log(self, repo: str, job_id: int, limit: int) -> str:
        response = await self.request("GET", f"/repos/{repo}/actions/jobs/{job_id}/logs")
        content = response.content
        if zipfile.is_zipfile(io.BytesIO(content)):
            with zipfile.ZipFile(io.BytesIO(content)) as archive:
                content = b"\n".join(archive.read(name) for name in archive.namelist())
        return content[-limit:].decode("utf-8", errors="replace")


def _normalize_login(login: str) -> str:
    return login.strip().removesuffix("[bot]").casefold()


def _mention(body: str, login: str) -> bool:
    return re.search(rf"(?<![\w-])@{re.escape(login)}(?![\w-])", body, re.IGNORECASE) is not None


def _issue_number(issue_url: str) -> int:
    match = re.search(r"/issues/([1-9][0-9]*)$", issue_url)
    if match is None:
        raise GitHubFailure(f"invalid issue URL in comment: {issue_url}")
    return int(match.group(1))


def _validate_repo(repo: str) -> None:
    if not _REPO_PATTERN.fullmatch(repo):
        raise HTTPException(422, "invalid repository")


def _validated_repo(request: RepositoryRequest) -> RepositoryRequest:
    _validate_repo(request.repo)
    return request


def create_proxy_app(settings: ProxySettings, api: GitHubApi | None = None) -> FastAPI:
    if api is None:
        tokens = StaticTokenProvider(settings.github_token_path)
        github = GitHubApi(tokens, settings.max_log_bytes, workspace_root=settings.workspace_root)
    else:
        github = api

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        yield
        await github.close()

    app = FastAPI(lifespan=lifespan)

    async def authenticate(request: Request) -> None:
        body = await request.body()
        result = verify(
            request.method,
            request.url.path,
            body,
            settings.hmac_key,
            request.headers.get(TIMESTAMP_HEADER),
            request.headers.get(SIGNATURE_HEADER),
        )
        if not result.valid:
            raise HTTPException(401, "invalid proxy signature")

    @app.get("/healthz")
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.post("/github/sync-workspace", dependencies=[Depends(authenticate)])
    async def sync_workspace(request: WorkspaceRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.sync_workspace(request)

    @app.post("/github/add-labels", dependencies=[Depends(authenticate)])
    async def add_labels(request: AddLabelsRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.add_labels(request)

    @app.post("/github/comment", dependencies=[Depends(authenticate)])
    async def comment(request: CommentRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.post_comment(request)

    @app.post("/github/search-issues", dependencies=[Depends(authenticate)])
    async def search(request: SearchRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.search_issues(request)

    @app.post("/github/poll-events", dependencies=[Depends(authenticate)])
    async def poll_events(request: PollRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.poll_events(request)

    @app.post("/github/dispatch-sim", dependencies=[Depends(authenticate)])
    async def dispatch(request: DispatchRequest) -> dict[str, Any]:
        _validated_repo(request)
        if not _WORKFLOW_PATTERN.fullmatch(request.workflow):
            raise HTTPException(422, "invalid workflow")
        return await github.dispatch_sim(request)

    @app.post("/github/sim-result", dependencies=[Depends(authenticate)])
    async def result(request: SimResultRequest) -> dict[str, Any]:
        _validated_repo(request)
        return await github.sim_result(request)

    @app.exception_handler(GitHubFailure)
    async def github_failure(_: Request, exc: GitHubFailure) -> Response:
        return JSONResponse(status_code=502, content={"detail": str(exc)})

    return app


def main() -> None:
    settings = ProxySettings.from_env()
    uvicorn.run(create_proxy_app(settings), host=settings.bind_host, port=settings.bind_port)
