from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import httpx

from serval_bot.auth import SIGNATURE_HEADER, TIMESTAMP_HEADER, sign


class ProxyError(RuntimeError):
    pass


@dataclass(slots=True)
class ProxyClient:
    base_url: str
    hmac_key: str
    timeout: float = 120.0
    transport: httpx.BaseTransport | None = None

    def _request(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode()
        timestamp, signature = sign("POST", path, body, self.hmac_key)
        headers = {
            "content-type": "application/json",
            TIMESTAMP_HEADER: timestamp,
            SIGNATURE_HEADER: signature,
        }
        with httpx.Client(
            base_url=self.base_url,
            timeout=self.timeout,
            transport=self.transport,
        ) as client:
            response = client.post(path, content=body, headers=headers)
        if response.is_error:
            raise ProxyError(f"proxy {path} failed with {response.status_code}: {response.text[:2000]}")
        data = response.json()
        if not isinstance(data, dict):
            raise ProxyError(f"proxy {path} returned a non-object response")
        return data

    def sync_workspace(
        self,
        repo: str,
        issue_number: int,
        pull_number: int | None = None,
        head_sha: str | None = None,
    ) -> dict[str, Any]:
        return self._request(
            "/github/sync-workspace",
            {
                "repo": repo,
                "issue_number": issue_number,
                "pull_number": pull_number,
                "head_sha": head_sha,
            },
        )

    def add_labels(self, repo: str, issue_number: int, labels: list[str]) -> dict[str, Any]:
        return self._request("/github/add-labels", {"repo": repo, "issue_number": issue_number, "labels": labels})

    def post_comment(self, repo: str, issue_number: int, body: str) -> dict[str, Any]:
        return self._request("/github/comment", {"repo": repo, "issue_number": issue_number, "body": body})

    def submit_review(
        self,
        repo: str,
        pull_number: int,
        commit_id: str,
        event: str,
        body: str,
        comments: list[dict[str, Any]],
    ) -> dict[str, Any]:
        return self._request(
            "/github/review",
            {
                "repo": repo,
                "pull_number": pull_number,
                "commit_id": commit_id,
                "event": event,
                "body": body,
                "comments": comments,
            },
        )

    def search_issues(self, repo: str, query: str) -> dict[str, Any]:
        return self._request("/github/search-issues", {"repo": repo, "query": query})

    def poll_events(self, repo: str, since: str, bot_login: str) -> dict[str, Any]:
        return self._request(
            "/github/poll-events",
            {"repo": repo, "since": since, "bot_login": bot_login},
        )

    def dispatch_sim(
        self,
        repo: str,
        issue_number: int,
        workflow: str,
        ref: str,
        head_sha: str | None,
    ) -> dict[str, Any]:
        return self._request(
            "/github/dispatch-sim",
            {
                "repo": repo,
                "issue_number": issue_number,
                "workflow": workflow,
                "ref": ref,
                "head_sha": head_sha,
            },
        )

    def sim_result(self, repo: str, run_id: int) -> dict[str, Any]:
        return self._request("/github/sim-result", {"repo": repo, "run_id": run_id})
