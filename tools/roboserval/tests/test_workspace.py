import subprocess
from pathlib import Path

import pytest

from serval_bot.workspace import CredentialedWorkspace, WorkspaceFailure


def _git(cwd: Path, *args: str) -> None:
    subprocess.run(("git", *args), cwd=cwd, check=True, capture_output=True, text=True)


def test_workspace_sync_keeps_token_out_of_git_config(tmp_path: Path) -> None:
    remote = tmp_path / "remotes" / "owner" / "repo.git"
    remote.parent.mkdir(parents=True)
    remote.mkdir()
    _git(remote, "init", "--bare")

    seed = tmp_path / "seed"
    seed.mkdir()
    _git(seed, "init", "-b", "main")
    _git(seed, "config", "user.name", "Test")
    _git(seed, "config", "user.email", "test@example.com")
    (seed / "state.txt").write_text("one\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "initial")
    _git(seed, "remote", "add", "origin", str(remote))
    _git(seed, "push", "origin", "main")

    workspace = CredentialedWorkspace(
        tmp_path / "workspaces",
        str(tmp_path / "remotes" / "{repo}.git"),
    ).sync("owner/repo", "main", "secret-token")

    assert (workspace / "state.txt").read_text() == "one\n"
    assert "secret-token" not in (workspace / ".git" / "config").read_text()


def test_workspace_sync_rejects_parent_branch(tmp_path: Path) -> None:
    manager = CredentialedWorkspace(tmp_path / "workspaces")
    with pytest.raises(WorkspaceFailure, match="invalid branch"):
        manager.sync("owner/repo", "../main", "secret-token")
