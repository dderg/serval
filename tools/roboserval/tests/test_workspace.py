import os
import shutil
import subprocess
from pathlib import Path

import pytest

from serval_bot.workspace import CredentialedWorkspace, WorkspaceFailure


def _git(cwd: Path, *args: str) -> None:
    subprocess.run(("git", *args), cwd=cwd, check=True, capture_output=True, text=True)


def _git_out(cwd: Path, *args: str) -> str:
    result = subprocess.run(("git", *args), cwd=cwd, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def _make_remote(tmp_path: Path) -> tuple[Path, Path]:
    remote = tmp_path / "remotes" / "owner" / "repo.git"
    remote.parent.mkdir(parents=True)
    remote.mkdir()
    _git(remote, "init", "--bare")
    _git(remote, "symbolic-ref", "HEAD", "refs/heads/main")

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
    return remote, seed


def _manager(tmp_path: Path, remote: Path) -> CredentialedWorkspace:
    return CredentialedWorkspace(tmp_path / "workspaces", str(remote))


def _pool_for(tmp_path: Path) -> Path:
    return tmp_path / "workspaces" / "owner--repo" / "pool.git"


def test_workspace_sync_keeps_token_out_of_git_config(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    workspace = _manager(tmp_path, remote).sync("owner/repo", "main", "secret-token", issue_number=7)

    assert (workspace / "state.txt").read_text() == "one\n"
    pool_config = (_pool_for(tmp_path) / "config").read_text()
    assert "secret-token" not in pool_config
    assert (workspace / ".git").is_dir()
    assert (workspace / ".git" / "objects" / "info" / "alternates").read_text() == (
        f"{_pool_for(tmp_path) / 'objects'}\n"
    )


def test_workspace_pool_reuses_workspace_per_issue(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)

    first = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    again = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    assert first == again
    assert (first / ".git").is_dir()
    assert _git_out(first, "rev-parse", "--absolute-git-dir") == str(first / ".git")
    pool = _pool_for(tmp_path)
    assert _git_out(first, "rev-parse", "HEAD") == _git_out(pool, "rev-parse", "refs/heads/main")

    (seed / "state.txt").write_text("updated\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "update")
    _git(seed, "push", "origin", "main")
    refreshed = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    assert refreshed == first
    assert (refreshed / "state.txt").read_text() == "updated\n"


def test_workspace_issues_do_not_rewrite_each_other(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)

    seven = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    (seven / "junk.txt").write_text("agent scratch\n")
    nine = manager.sync("owner/repo", "main", "secret-token", issue_number=9)

    assert nine != seven
    assert _git_out(seven, "rev-parse", "--absolute-git-dir") == str(seven / ".git")
    assert _git_out(nine, "rev-parse", "--absolute-git-dir") == str(nine / ".git")
    assert (nine / "state.txt").read_text() == "one\n"
    assert not (nine / "junk.txt").exists()

    (seed / "state.txt").write_text("updated\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "update")
    _git(seed, "push", "origin", "main")

    seven_again = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    assert seven_again == seven
    assert (seven / "state.txt").read_text() == "updated\n"
    assert not (seven / "junk.txt").exists()
    assert (nine / "state.txt").read_text() == "one\n"
    assert not (nine / "junk.txt").exists()


def test_workspace_pull_request_sha_mismatch_fails(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)

    (seed / "state.txt").write_text("pull request\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "pull request")
    _git(seed, "push", "origin", "HEAD:refs/pull/7/head")
    head_sha = _git_out(seed, "rev-parse", "HEAD")

    workspace = manager.sync(
        "owner/repo",
        "main",
        "secret-token",
        issue_number=7,
        fetch_ref="refs/pull/7/head",
        expected_sha=head_sha,
    )
    assert (workspace / "state.txt").read_text() == "pull request\n"

    with pytest.raises(WorkspaceFailure, match="pull request head changed"):
        manager.sync(
            "owner/repo",
            "main",
            "secret-token",
            issue_number=7,
            fetch_ref="refs/pull/7/head",
            expected_sha="0" * 40,
        )
    assert (workspace / "state.txt").read_text() == "pull request\n"


def test_workspace_legacy_sync_without_issue_number(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    checkout = _manager(tmp_path, remote).sync("owner/repo", "main", "secret-token")

    assert checkout == tmp_path / "workspaces" / "legacy" / "owner--repo"
    assert (checkout / ".git").is_dir()
    assert _git_out(checkout, "rev-parse", "--abbrev-ref", "HEAD") == "main"
    assert (checkout / "state.txt").read_text() == "one\n"


def test_workspace_sync_rejects_parent_branch(tmp_path: Path) -> None:
    manager = CredentialedWorkspace(tmp_path / "workspaces")
    with pytest.raises(WorkspaceFailure, match="invalid branch"):
        manager.sync("owner/repo", "../main", "secret-token", issue_number=7)


def test_workspace_rejects_legacy_shared_git_layout(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    old_pool = tmp_path / "workspaces" / "owner--repo" / "clone.git"
    old_pool.mkdir(parents=True)
    _git(old_pool, "init", "--bare")
    worktree = tmp_path / "workspaces" / "owner--repo" / "7"
    worktree.mkdir()
    (worktree / ".git").write_text(f"gitdir: {old_pool}/worktrees/7\n")

    with pytest.raises(WorkspaceFailure, match="legacy shared-Git pool"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    shutil.rmtree(old_pool)
    with pytest.raises(WorkspaceFailure, match="legacy linked worktree"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)


def test_workspace_legacy_checkout_migrates_to_pool_backed_standalone(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    legacy = tmp_path / "workspaces" / "legacy" / "owner--repo"
    legacy.parent.mkdir(parents=True)
    _git(tmp_path, "clone", str(remote), str(legacy))

    (seed / "state.txt").write_text("updated\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "update")
    _git(seed, "push", "origin", "main")

    checkout = manager.sync("owner/repo", "main", "secret-token")

    assert checkout == legacy
    assert (checkout / "state.txt").read_text() == "updated\n"
    config = (legacy / ".git" / "config").read_text()
    assert str(_pool_for(tmp_path)) in config
    assert "[user]" in config


def test_workspace_rewrites_agent_controlled_metadata_before_privileged_ops(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    (seed / "state.txt").write_text("updated\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "update")
    _git(seed, "push", "origin", "main")

    hook_marker = tmp_path / "hook-ran"
    alias_marker = tmp_path / "alias-ran"
    hooks = workspace / ".git" / "hooks"
    hooks.mkdir()
    (hooks / "post-checkout").write_text(f"#!/bin/sh\ntouch {hook_marker}\n")
    (hooks / "post-checkout").chmod(0o755)
    config = workspace / ".git" / "config"
    config.write_text(
        config.read_text()
        + f"[alias]\n\tfetch = !touch {alias_marker}\n"
        + f"[core]\n\thooksPath = {hooks}\n"
        + "[user]\n\tname = Attacker\n"
        + f'[remote "origin"]\n\turl = {tmp_path / "nowhere.git"}\n'
    )

    refreshed = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    assert (refreshed / "state.txt").read_text() == "updated\n"
    assert not hook_marker.exists()
    assert not alias_marker.exists()
    rewritten = config.read_text()
    assert "[alias]" not in rewritten
    assert str(hooks) not in rewritten
    assert str(_pool_for(tmp_path)) in rewritten
    assert "Attacker" not in rewritten
    assert "Attacker" not in (_pool_for(tmp_path) / "config").read_text()


def test_workspace_fails_loudly_on_group_writable_pool(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    (_pool_for(tmp_path) / "config").chmod(0o666)

    with pytest.raises(WorkspaceFailure, match="unsafe repository control data"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)


def test_workspace_fails_loudly_on_group_writable_or_symlinked_metadata(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    (workspace / ".git").chmod(0o777)
    with pytest.raises(WorkspaceFailure, match="unsafe repository control data"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    (workspace / ".git").chmod(0o700)
    os.unlink(workspace / ".git" / "config")
    os.symlink(_pool_for(tmp_path) / "config", workspace / ".git" / "config")
    with pytest.raises(WorkspaceFailure, match="symlink inside git metadata"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)


def test_workspace_slot_can_commit_into_own_object_store(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    (workspace / "agent-note.txt").write_text("local scratch\n")
    _git(workspace, "add", "agent-note.txt")
    _git(workspace, "commit", "-m", "agent commit")
    head = _git_out(workspace, "rev-parse", "HEAD")
    assert _git_out(workspace, "log", "-1", "--format=%an") == "RoboServal"

    # the commit lives in the workspace's own object store; the read-only pool
    # neither gained it nor sees the scratch file
    pool = _pool_for(tmp_path)
    result = subprocess.run(("git", "-C", str(pool), "cat-file", "-e", head), capture_output=True, text=True)
    assert result.returncode != 0
    assert "agent-note.txt" not in _git_out(pool, "ls-tree", "-r", "--name-only", "refs/heads/main")
    assert "agent-note.txt" not in _git_out(seed, "ls-tree", "-r", "--name-only", "main")


def test_workspace_git_environment_pins_fixed_overrides(tmp_path: Path) -> None:
    from serval_bot.workspace import _git_environment, _issue_overrides

    base = {
        "PATH": "/usr/bin",
        "GIT_CONFIG_COUNT": "1",
        "GIT_CONFIG_KEY_0": "http.extraHeader",
        "GIT_CONFIG_VALUE_0": "Authorization: Basic c2VjcmV0",
    }
    workspace = tmp_path / "workspace"
    pool = tmp_path / "pool.git"
    env = _git_environment(base, workspace, pool, overrides=_issue_overrides(pool))

    assert env["GIT_CONFIG_NOSYSTEM"] == "1"
    assert env["GIT_CONFIG_GLOBAL"] == os.devnull
    assert env["GIT_TERMINAL_PROMPT"] == "0"
    entries: dict[str, str] = {}
    for index in range(int(env["GIT_CONFIG_COUNT"])):
        entries[env[f"GIT_CONFIG_KEY_{index}"]] = env[f"GIT_CONFIG_VALUE_{index}"]
    assert entries["http.extraHeader"] == "Authorization: Basic c2VjcmV0"
    assert entries["remote.origin.url"] == str(pool)
    assert entries["remote.origin.fetch"] == "+refs/heads/*:refs/remotes/origin/*"
    assert entries["core.hooksPath"] == ""
    assert entries["core.fsmonitor"] == ""
    assert entries["protocol.file.allow"] == "always"
    safe = [
        env[f"GIT_CONFIG_VALUE_{index}"]
        for index in range(int(env["GIT_CONFIG_COUNT"]))
        if env[f"GIT_CONFIG_KEY_{index}"] == "safe.directory"
    ]
    assert safe == [str(workspace), str(pool)]
