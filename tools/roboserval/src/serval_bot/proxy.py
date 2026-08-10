from __future__ import annotations

import asyncio
import io
import re
import secrets
import zipfile
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Literal, Protocol

import httpx
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field

from serval_bot.auth import SIGNATURE_HEADER, TIMESTAMP_HEADER, verify
from serval_bot.config import ProxySettings
from serval_bot.policy import Mode, PolicyError
from serval_bot.token_auth import StaticTokenProvider
from serval_bot.workspace import CredentialedWorkspace, WorkspaceFailure

_REPO_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
_WORKFLOW_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+\.ya?ml$")
_FARM_SLUG_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


class GitHubFailure(RuntimeError):
    def __init__(self, message: str, status_code: int | None = None):
        super().__init__(message)
        self.status_code = status_code


class RepositoryRequest(BaseModel):
    repo: str


class WorkspaceRequest(RepositoryRequest):
    issue_number: int | None = Field(default=None, gt=0)
    pull_number: int | None = Field(default=None, gt=0)
    head_sha: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")


class IssueRequest(RepositoryRequest):
    issue_number: int = Field(gt=0)


class AddLabelsRequest(IssueRequest):
    labels: list[str] = Field(min_length=1, max_length=20)


class CommentRequest(IssueRequest):
    body: str = Field(min_length=1, max_length=65_000)


class ReviewComment(BaseModel):
    path: str = Field(min_length=1, max_length=4096)
    line: int = Field(gt=0)
    side: Literal["LEFT", "RIGHT"]
    body: str = Field(min_length=1, max_length=65_000)


class PullReviewRequest(RepositoryRequest):
    pull_number: int = Field(gt=0)
    commit_id: str = Field(pattern=r"^[0-9a-f]{40}$")
    event: Literal["APPROVE", "REQUEST_CHANGES"]
    body: str = Field(min_length=1, max_length=65_000)
    comments: list[ReviewComment] = Field(default_factory=list, max_length=100)


class SearchRequest(RepositoryRequest):
    query: str = Field(min_length=1, max_length=256)


class DispatchRequest(RepositoryRequest):
    issue_number: int = Field(gt=0)
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
        *,
        dispatch_poll_attempts: int = 30,
        dispatch_poll_interval: float = 1.0,
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
        self._dispatch_locks: dict[tuple[str, str, str], asyncio.Lock] = {}
        self._dispatch_poll_attempts = dispatch_poll_attempts
        self._dispatch_poll_interval = dispatch_poll_interval

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
        sync_kwargs: dict[str, Any] = {}
        if request.issue_number is not None:
            sync_kwargs["issue_number"] = request.issue_number
        path = await asyncio.to_thread(
            self._workspace.sync,
            request.repo,
            default_branch,
            token,
            fetch_ref=fetch_ref,
            expected_sha=request.head_sha,
            **sync_kwargs,
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
            raise GitHubFailure(
                f"GitHub {method} {path} failed: {response.status_code}, retry={retry}, {detail}",
                response.status_code,
            )
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

    async def submit_review(self, request: PullReviewRequest) -> dict[str, Any]:
        response = await self.request(
            "POST",
            f"/repos/{request.repo}/pulls/{request.pull_number}/reviews",
            json={
                "commit_id": request.commit_id,
                "event": request.event,
                "body": request.body,
                "comments": [comment.model_dump() for comment in request.comments],
            },
        )
        data = response.json()
        return {"id": data["id"], "url": data["html_url"], "state": data["state"]}

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
        issues_by_number = {issue["number"]: issue for issue in issues}
        pull_requests: dict[int, dict[str, Any]] = {}

        async def pull_request(issue_number: int) -> dict[str, Any]:
            if issue_number not in pull_requests:
                response = await self.request("GET", f"/repos/{request.repo}/pulls/{issue_number}")
                value = response.json()
                if not isinstance(value, dict):
                    raise GitHubFailure(f"GitHub pull request {request.repo}#{issue_number} is not an object")
                pull_requests[issue_number] = value
            return pull_requests[issue_number]

        open_pull_requests = await self._pages(
            f"/repos/{request.repo}/pulls",
            {"state": "open", "per_page": 100},
        )
        review_heads: list[dict[str, Any]] = []
        for current_pull_request in open_pull_requests:
            requested_reviewers = current_pull_request.get("requested_reviewers")
            issue_number = current_pull_request.get("number")
            if not isinstance(requested_reviewers, list) or not isinstance(issue_number, int):
                raise GitHubFailure(f"GitHub open pull request is malformed: {current_pull_request!r}")
            if not any(
                isinstance(reviewer, dict)
                and _normalize_login(reviewer.get("login", "")) == _normalize_login(request.bot_login)
                for reviewer in requested_reviewers
            ):
                continue
            pull_requests[issue_number] = current_pull_request
            head = current_pull_request.get("head")
            if not isinstance(head, dict) or not isinstance(head.get("sha"), str):
                raise GitHubFailure(f"GitHub pull request {request.repo}#{issue_number} has no head SHA")
            head_sha = head["sha"]
            review_heads.append({"issue_number": issue_number, "head_sha": head_sha})
            issue = issues_by_number.get(issue_number)
            if issue is None:
                value = (await self.request("GET", f"/repos/{request.repo}/issues/{issue_number}")).json()
                if not isinstance(value, dict):
                    raise GitHubFailure(f"GitHub issue {request.repo}#{issue_number} is not an object")
                issue = value
                issues_by_number[issue_number] = issue
            issue_events = await self._pages(
                f"/repos/{request.repo}/issues/{issue_number}/events",
                {"per_page": 100},
            )
            matching_requests: list[tuple[str, int, dict[str, Any], dict[str, Any]]] = []
            for issue_event in issue_events:
                requested_reviewer = issue_event.get("requested_reviewer")
                if (
                    issue_event.get("event") != "review_requested"
                    or not isinstance(requested_reviewer, dict)
                    or _normalize_login(requested_reviewer.get("login", "")) != _normalize_login(request.bot_login)
                ):
                    continue
                review_requester = issue_event.get("review_requester")
                event_id = issue_event.get("id")
                occurred_at = issue_event.get("created_at")
                if (
                    not isinstance(review_requester, dict)
                    or not isinstance(review_requester.get("login"), str)
                    or not isinstance(event_id, int)
                    or not isinstance(occurred_at, str)
                ):
                    raise GitHubFailure(f"GitHub review request event is malformed: {issue_event!r}")
                matching_requests.append((occurred_at, event_id, issue_event, review_requester))
            if not matching_requests:
                raise GitHubFailure(
                    f"GitHub pull request {request.repo}#{issue_number} requests {request.bot_login} "
                    "but has no matching review_requested event"
                )
            occurred_at, event_id, issue_event, review_requester = max(matching_requests)
            actor = review_requester["login"]
            if _normalize_login(actor) == _normalize_login(request.bot_login):
                continue
            if not await self._head_checks_passed(request.repo, head_sha):
                continue
            events.append(
                {
                    "delivery_id": f"poll:review:{event_id}:{head_sha}:requested",
                    "event_type": "pull_request_review.requested",
                    "issue_number": issue_number,
                    "actor": actor,
                    "occurred_at": occurred_at,
                    "payload": {
                        "action": "review_requested",
                        "repository": {"full_name": request.repo},
                        "sender": review_requester,
                        "issue": issue,
                        "pull_request": current_pull_request,
                        "review_request": issue_event,
                    },
                }
            )

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
            issue = issues_by_number.get(issue_number)
            if issue is None:
                issue = (await self.request("GET", f"/repos/{request.repo}/issues/{issue_number}")).json()
            event_type = "issue_comment.created"
            payload = {
                "action": "created",
                "repository": {"full_name": request.repo},
                "sender": comment["user"],
                "issue": issue,
                "comment": comment,
            }
            if "pull_request" in issue:
                payload["pull_request"] = await pull_request(issue_number)
                event_type = "pull_request_review.requested"
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
        return {"events": events, "review_heads": review_heads}

    async def _head_checks_passed(self, repo: str, head_sha: str) -> bool:
        response = await self.request(
            "GET",
            f"/repos/{repo}/commits/{head_sha}/check-runs",
            params={"filter": "latest", "per_page": 100},
        )
        payload = response.json()
        if not isinstance(payload, dict):
            raise GitHubFailure(f"GitHub check runs for {repo}@{head_sha} are not an object")
        total_count = payload.get("total_count")
        check_runs = payload.get("check_runs")
        if not isinstance(total_count, int) or not isinstance(check_runs, list):
            raise GitHubFailure(f"GitHub check runs for {repo}@{head_sha} are malformed")
        if total_count > len(check_runs):
            raise GitHubFailure(f"GitHub check runs for {repo}@{head_sha} exceed one page")
        if not check_runs:
            return False
        for check_run in check_runs:
            if not isinstance(check_run, dict):
                raise GitHubFailure(f"GitHub check run for {repo}@{head_sha} is malformed")
            status = check_run.get("status")
            conclusion = check_run.get("conclusion")
            if not isinstance(status, str) or (conclusion is not None and not isinstance(conclusion, str)):
                raise GitHubFailure(f"GitHub check run for {repo}@{head_sha} is malformed")
            if status != "completed" or conclusion not in {"success", "neutral", "skipped"}:
                return False
        return True

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

    async def _resolve_ref_sha(self, repo: str, ref: str) -> str:
        response = await self.request("GET", f"/repos/{repo}/git/ref/heads/{ref}")
        data = response.json()
        sha = data.get("object", {}).get("sha")
        if not isinstance(sha, str) or len(sha) != 40:
            raise GitHubFailure(f"could not resolve ref heads/{ref} to a commit sha")
        return sha

    async def _workflow_dispatch_runs(self, request: DispatchRequest, ref: str) -> list[dict[str, Any]]:
        response = await self.request(
            "GET",
            f"/repos/{request.repo}/actions/workflows/{request.workflow}/runs",
            params={"branch": ref, "event": "workflow_dispatch", "per_page": 100},
        )
        runs = response.json().get("workflow_runs", [])
        if request.head_sha is not None:
            runs = [run for run in runs if run.get("head_sha") == request.head_sha]
        return runs

    async def _delete_temp_ref(self, repo: str, ref: str, *, allow_missing: bool = False) -> None:
        try:
            await self.request("DELETE", f"/repos/{repo}/git/refs/heads/{ref}")
        except GitHubFailure as exc:
            if not allow_missing or exc.status_code not in {404, 422}:
                raise

    async def _publish_farm_ref(self, request: DispatchRequest) -> None:
        """Publish the exact issue-workspace commit to the requested farm ref.

        Farm refs are scoped to the issue the request derives from: the agent
        never supplies a workspace path or authority, only the branch name and
        the commit it claims to have made. The workspace is located and
        validated by the credentialed root, and its exact HEAD is pushed before
        any dispatch happens; any validation or publication failure aborts the
        dispatch loudly.
        """
        if request.head_sha is None:
            raise GitHubFailure("farm ref dispatch requires the workspace head_sha")
        prefix = f"farm/{request.issue_number}-"
        if not request.ref.startswith(prefix) or len(request.ref) == len(prefix):
            raise GitHubFailure(f"farm ref must be scoped to issue {request.issue_number}: {request.ref}")
        if not _FARM_SLUG_PATTERN.fullmatch(request.ref.removeprefix(prefix)):
            raise GitHubFailure(f"invalid farm ref: {request.ref}")
        token = await self._tokens.token()
        try:
            await asyncio.to_thread(
                self._workspace.publish_issue,
                request.repo,
                request.issue_number,
                token,
                ref=request.ref,
                expected_sha=request.head_sha,
            )
        except WorkspaceFailure as exc:
            raise GitHubFailure(f"issue workspace publication failed: {exc}") from exc

    async def dispatch_sim(self, request: DispatchRequest) -> dict[str, Any]:
        key = (request.repo, request.workflow, request.ref)
        lock = self._dispatch_locks.setdefault(key, asyncio.Lock())
        async with lock:
            # Authoritative correlation: farm refs are first published from the
            # exact validated issue-workspace commit; the requested ref is then
            # resolved to its exact sha (verifying a caller-supplied head_sha),
            # the existing workflow is dispatched on a fresh random temporary
            # branch at that sha, and only that unique branch is polled. An
            # external or manual run on any other ref can never qualify. The
            # temporary ref is deleted in a strict finally path, and a failed
            # delete fails loudly.
            if request.ref.startswith("farm/"):
                await self._publish_farm_ref(request)
            sha = await self._resolve_ref_sha(request.repo, request.ref)
            if request.head_sha is not None and sha != request.head_sha:
                raise GitHubFailure(f"ref heads/{request.ref} moved: expected {request.head_sha}, found {sha}")
            temp_ref = f"serval-{secrets.token_hex(16)}"
            creation_confirmed = False
            try:
                await self.request(
                    "POST",
                    f"/repos/{request.repo}/git/refs",
                    json={"ref": f"refs/heads/{temp_ref}", "sha": sha},
                )
                creation_confirmed = True
                baseline = {run["id"] for run in await self._workflow_dispatch_runs(request, temp_ref)}
                await self.request(
                    "POST",
                    f"/repos/{request.repo}/actions/workflows/{request.workflow}/dispatches",
                    json={"ref": temp_ref},
                )
                for _ in range(self._dispatch_poll_attempts):
                    matches = [
                        run
                        for run in await self._workflow_dispatch_runs(request, temp_ref)
                        if run["id"] not in baseline
                    ]
                    if len(matches) == 1:
                        run = matches[0]
                        return {
                            "run_id": run["id"],
                            "url": run["html_url"],
                            "status": run["status"],
                            "conclusion": run.get("conclusion"),
                            "head_sha": run.get("head_sha"),
                            "ref": temp_ref,
                            "workflow": run.get("path"),
                            "requested_ref": request.ref,
                        }
                    if len(matches) > 1:
                        raise GitHubFailure(
                            f"ambiguous workflow dispatch: multiple new runs appeared for {request.workflow}@{temp_ref}"
                        )
                    await asyncio.sleep(self._dispatch_poll_interval)
                raise GitHubFailure(
                    f"dispatched workflow run did not appear within "
                    f"{self._dispatch_poll_attempts * self._dispatch_poll_interval:.0f} seconds"
                )
            finally:
                await self._delete_temp_ref(request.repo, temp_ref, allow_missing=not creation_confirmed)

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
            "ref": run.get("head_branch"),
            "workflow": run.get("path"),
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


_ALL_MODES = frozenset(Mode)
_ACTIVE_MODES = frozenset({Mode.TRIAGE, Mode.MAINTAINER})


def _authorize_repo(settings: ProxySettings, request: RepositoryRequest, modes: frozenset[Mode]) -> None:
    try:
        policy = settings.policy.require(request.repo)
    except PolicyError as exc:
        raise HTTPException(403, "repository not allowlisted") from exc
    if policy.mode not in modes:
        raise HTTPException(403, f"endpoint not permitted for {policy.mode.value} repositories")


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
        _authorize_repo(settings, request, _ALL_MODES)
        return await github.sync_workspace(request)

    @app.post("/github/add-labels", dependencies=[Depends(authenticate)])
    async def add_labels(request: AddLabelsRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ACTIVE_MODES)
        return await github.add_labels(request)

    @app.post("/github/comment", dependencies=[Depends(authenticate)])
    async def comment(request: CommentRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ACTIVE_MODES)
        return await github.post_comment(request)

    @app.post("/github/review", dependencies=[Depends(authenticate)])
    async def review(request: PullReviewRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ACTIVE_MODES)
        return await github.submit_review(request)

    @app.post("/github/search-issues", dependencies=[Depends(authenticate)])
    async def search(request: SearchRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ALL_MODES)
        return await github.search_issues(request)

    @app.post("/github/poll-events", dependencies=[Depends(authenticate)])
    async def poll_events(request: PollRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ALL_MODES)
        return await github.poll_events(request)

    @app.post("/github/dispatch-sim", dependencies=[Depends(authenticate)])
    async def dispatch(request: DispatchRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ACTIVE_MODES)
        if not _WORKFLOW_PATTERN.fullmatch(request.workflow):
            raise HTTPException(422, "invalid workflow")
        return await github.dispatch_sim(request)

    @app.post("/github/sim-result", dependencies=[Depends(authenticate)])
    async def result(request: SimResultRequest) -> dict[str, Any]:
        _validated_repo(request)
        _authorize_repo(settings, request, _ACTIVE_MODES)
        return await github.sim_result(request)

    @app.exception_handler(GitHubFailure)
    async def github_failure(_: Request, exc: GitHubFailure) -> Response:
        return JSONResponse(status_code=502, content={"detail": str(exc)})

    return app


def main() -> None:
    settings = ProxySettings.from_env()
    uvicorn.run(create_proxy_app(settings), host=settings.bind_host, port=settings.bind_port)
