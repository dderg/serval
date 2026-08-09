from __future__ import annotations

import base64
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

_REPO_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
_BRANCH_PATTERN = re.compile(r"^[A-Za-z0-9._/-]+$")
_PULL_REF_PATTERN = re.compile(r"^refs/pull/[1-9][0-9]*/head$")
_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")


class WorkspaceFailure(RuntimeError):
    pass


@dataclass(slots=True)
class CredentialedWorkspace:
    root: Path
    remote_template: str = "https://github.com/{repo}.git"

    def sync(
        self,
        repo: str,
        branch: str,
        token: str,
        *,
        fetch_ref: str | None = None,
        expected_sha: str | None = None,
    ) -> Path:
        if not _REPO_PATTERN.fullmatch(repo):
            raise WorkspaceFailure(f"invalid repository: {repo}")
        if not _BRANCH_PATTERN.fullmatch(branch) or ".." in branch or branch.startswith("/"):
            raise WorkspaceFailure(f"invalid branch: {branch}")
        if (fetch_ref is None) != (expected_sha is None):
            raise WorkspaceFailure("fetch_ref and expected_sha must be set together")
        if fetch_ref is not None and (
            not _PULL_REF_PATTERN.fullmatch(fetch_ref) or not _SHA_PATTERN.fullmatch(expected_sha or "")
        ):
            raise WorkspaceFailure("invalid pull request revision")
        self.root.mkdir(parents=True, exist_ok=True)
        destination = self.root / repo.replace("/", "--")
        environment = self._environment(token)
        if not destination.exists():
            self._run(
                "git",
                "clone",
                "--branch",
                branch,
                "--single-branch",
                self.remote_template.format(repo=repo),
                str(destination),
                cwd=self.root,
                environment=environment,
            )
        self._run("git", "fetch", "--prune", "origin", branch, cwd=destination, environment=environment)
        if fetch_ref is None:
            revision = f"origin/{branch}"
            self._run("git", "checkout", "-B", branch, revision, cwd=destination, environment=environment)
        else:
            review_ref = "refs/remotes/origin/roboserval-review"
            self._run(
                "git",
                "fetch",
                "--force",
                "origin",
                f"{fetch_ref}:{review_ref}",
                cwd=destination,
                environment=environment,
            )
            revision = self._run("git", "rev-parse", review_ref, cwd=destination, environment=environment)
            if revision != expected_sha:
                raise WorkspaceFailure(f"pull request head changed: expected {expected_sha}, got {revision}")
            self._run("git", "checkout", "--detach", revision, cwd=destination, environment=environment)
        self._run("git", "reset", "--hard", revision, cwd=destination, environment=environment)
        self._run("git", "clean", "-fd", cwd=destination, environment=environment)
        return destination

    @staticmethod
    def _environment(token: str) -> dict[str, str]:
        credentials = base64.b64encode(f"x-access-token:{token}".encode()).decode()
        return {
            **os.environ,
            "GIT_CONFIG_COUNT": "1",
            "GIT_CONFIG_KEY_0": "http.extraHeader",
            "GIT_CONFIG_VALUE_0": f"Authorization: Basic {credentials}",
            "GIT_TERMINAL_PROMPT": "0",
        }

    @staticmethod
    def _run(*command: str, cwd: Path, environment: dict[str, str]) -> str:
        result = subprocess.run(
            command,
            cwd=cwd,
            env=environment,
            text=True,
            capture_output=True,
            timeout=300,
            check=False,
        )
        if result.returncode != 0:
            output = (result.stdout + result.stderr)[-4000:]
            raise WorkspaceFailure(f"git command failed ({result.returncode}): {' '.join(command)}\n{output}")
        return result.stdout.strip()
