import os
import shutil
import stat
import subprocess
from pathlib import Path
from typing import Any

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
    (seed / ".github/workflows").mkdir(parents=True)
    (seed / ".github/workflows" / "ci-sim-e2e.yaml").write_text("name: sim-e2e\non: workflow_dispatch\n")
    _git(seed, "add", ".")
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


def test_workspace_upgrade_adopts_legacy_namespace_with_existing_pool(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    namespace = tmp_path / "workspaces" / "owner--repo"
    (workspace / "agent-note.txt").write_text("scratch\n")

    # legacy deployment state: namespace group-writable (old setgid omp layout)
    namespace.chmod(0o775)
    refreshed = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    assert refreshed == workspace
    assert stat.S_IMODE(namespace.stat().st_mode) == 0o755
    assert not (refreshed / "agent-note.txt").exists()

    # pool control data tampering is still rejected loudly after adoption
    (_pool_for(tmp_path) / "config").chmod(0o666)
    with pytest.raises(WorkspaceFailure, match="unsafe repository control data"):
        manager.sync("owner/repo", "main", "secret-token", issue_number=7)


def test_workspace_upgrade_adopts_legacy_namespace_with_new_pool(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    namespace = tmp_path / "workspaces" / "owner--repo"
    namespace.mkdir(parents=True)
    namespace.chmod(0o775)  # legacy state, no pool yet

    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)

    assert (workspace / "state.txt").read_text() == "one\n"
    assert stat.S_IMODE(namespace.stat().st_mode) == 0o755
    pool = _pool_for(tmp_path)
    pool.chmod(0o750)
    manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    assert stat.S_IMODE(pool.stat().st_mode) == 0o755
    assert (pool / "config").is_file()
    assert stat.S_IMODE((pool / "config").stat().st_mode) & 0o022 == 0


def test_workspace_adopt_namespace_dir_adopts_foreign_owner_and_rejects_symlink(
    tmp_path: Path, monkeypatch: Any
) -> None:
    from serval_bot import workspace as workspace_module

    namespace = tmp_path / "workspaces" / "owner--repo"
    namespace.mkdir(parents=True)
    namespace.chmod(0o770)
    chowned: list[tuple[str, int, int]] = []
    monkeypatch.setattr(workspace_module.os, "geteuid", lambda: 10001)
    monkeypatch.setattr(workspace_module.os, "chown", lambda path, uid, gid: chowned.append((str(path), uid, gid)))

    workspace_module._adopt_namespace_dir(namespace)

    # only the namespace directory itself is adopted, never its contents
    assert chowned == [(str(namespace), 10001, 10001)]
    assert stat.S_IMODE(namespace.stat().st_mode) == 0o755

    link = tmp_path / "evil-namespace"
    os.symlink(namespace, link)
    with pytest.raises(WorkspaceFailure, match="not a directory"):
        workspace_module._adopt_namespace_dir(link)


def _farm_branch(workspace: Path, message: str = "reproduce issue 7 [skip ci]") -> str:
    _git(workspace, "checkout", "-b", "farm/7-calib")
    (workspace / "repro.py").write_text("print('repro')\n")
    _git(workspace, "add", "repro.py")
    _git(workspace, "commit", "-m", message)
    return _git_out(workspace, "rev-parse", "HEAD")


def _remote_has(remote: Path, ref: str) -> bool:
    result = subprocess.run(("git", "-C", str(remote), "show-ref", "--verify", ref), capture_output=True, text=True)
    return result.returncode == 0


def test_workspace_publish_pushes_exact_commit_to_farm_ref(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _farm_branch(workspace)

    manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    # idempotent: the same exact commit is published again without error
    manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)

    assert _git_out(remote, "rev-parse", "refs/heads/farm/7-calib") == head
    assert _git_out(remote, "rev-parse", "refs/heads/main") != head
    assert "secret-token" not in (workspace / ".git" / "config").read_text()
    assert "secret-token" not in (_pool_for(tmp_path) / "config").read_text()


def test_workspace_publish_rejects_dirty_workspace(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _farm_branch(workspace)
    (workspace / "scratch.txt").write_text("uncommitted\n")

    with pytest.raises(WorkspaceFailure, match="not clean"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_rejects_head_sha_mismatch(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    _farm_branch(workspace)

    with pytest.raises(WorkspaceFailure, match="head mismatch"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha="0" * 40)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")
    assert _git_out(remote, "rev-parse", "refs/heads/main") == _git_out(
        workspace, "rev-parse", "refs/remotes/origin/main"
    )


def test_workspace_publish_rejects_wrong_branch(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _git_out(workspace, "rev-parse", "HEAD")

    # a freshly synced workspace is detached, so it can never satisfy the branch check
    with pytest.raises(WorkspaceFailure, match="not on branch farm/7-calib"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)

    _git(workspace, "checkout", "-b", "farm/7-other")
    with pytest.raises(WorkspaceFailure, match="not on branch farm/7-calib"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_requires_skip_ci_commit_message(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _farm_branch(workspace, message="reproduce issue 7")

    with pytest.raises(WorkspaceFailure, match="skip ci"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_rejects_workflow_changes(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    _git(workspace, "checkout", "-b", "farm/7-calib")
    workflow = workspace / ".github/workflows/ci-sim-e2e.yaml"
    workflow.write_text(workflow.read_text() + "permissions: write-all\n")
    _git(workspace, "add", ".github/workflows/ci-sim-e2e.yaml")
    _git(workspace, "commit", "-m", "tweak workflow [skip ci]")
    head = _git_out(workspace, "rev-parse", "HEAD")

    with pytest.raises(WorkspaceFailure, match="workflows"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_rejects_orphan_history(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    # a parentless commit whose tree only differs from main by the repro file
    _git(workspace, "checkout", "--orphan", "farm/7-calib")
    (workspace / "repro.py").write_text("print('repro')\n")
    _git(workspace, "add", "repro.py")
    _git(workspace, "commit", "-m", "reproduce issue 7 [skip ci]")
    head = _git_out(workspace, "rev-parse", "HEAD")

    with pytest.raises(WorkspaceFailure, match="does not descend"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_rejects_stale_base_history(tmp_path: Path) -> None:
    remote, seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    _git(workspace, "checkout", "-b", "farm/7-calib")
    (workspace / "repro.py").write_text("print('repro')\n")
    _git(workspace, "add", "repro.py")
    _git(workspace, "commit", "-m", "reproduce issue 7 [skip ci]")
    head = _git_out(workspace, "rev-parse", "HEAD")

    # upstream advances while the agent still sits on the old base
    (seed / "state.txt").write_text("updated upstream\n")
    _git(seed, "add", "state.txt")
    _git(seed, "commit", "-m", "upstream update")
    _git(seed, "push", "origin", "main")

    with pytest.raises(WorkspaceFailure, match="does not descend"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)
    assert not _remote_has(remote, "refs/heads/farm/7-calib")


def test_workspace_publish_allows_non_workflow_reproduction_files(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    _git(workspace, "checkout", "-b", "farm/7-calib")
    (workspace / "tools/sim/tests").mkdir(parents=True)
    (workspace / "tools/sim/tests" / "test_repro.py").write_text("def test_repro():\n    assert True\n")
    (workspace / "notes").mkdir()
    (workspace / "notes" / "repro.md").write_text("steps\n")
    (workspace / "state.txt").write_text("repro state\n")
    _git(workspace, "add", ".")
    _git(workspace, "commit", "-m", "reproduce issue 7 [skip ci]")
    head = _git_out(workspace, "rev-parse", "HEAD")

    manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)

    assert _git_out(remote, "rev-parse", "refs/heads/farm/7-calib") == head


def test_workspace_publish_rejects_missing_workspace(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    with pytest.raises(WorkspaceFailure, match="workspace not found"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha="a" * 40)


def test_workspace_publish_rejects_invalid_inputs(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _git_out(workspace, "rev-parse", "HEAD")
    with pytest.raises(WorkspaceFailure, match="invalid publication ref"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="../escape", expected_sha=head)
    with pytest.raises(WorkspaceFailure, match="invalid expected sha"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha="not-a-sha")
    with pytest.raises(WorkspaceFailure, match="invalid repository"):
        manager.publish_issue("not-a-repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)


def test_workspace_publish_fails_loudly_on_unsafe_metadata(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    (workspace / ".git").chmod(0o777)
    with pytest.raises(WorkspaceFailure, match="unsafe repository control data"):
        manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha="a" * 40)


def test_workspace_publish_rewrites_agent_controlled_metadata_before_push(tmp_path: Path) -> None:
    remote, _seed = _make_remote(tmp_path)
    manager = _manager(tmp_path, remote)
    workspace = manager.sync("owner/repo", "main", "secret-token", issue_number=7)
    head = _farm_branch(workspace)

    hook_marker = tmp_path / "hook-ran"
    alias_marker = tmp_path / "alias-ran"
    hooks = workspace / ".git" / "hooks"
    hooks.mkdir()
    (hooks / "pre-push").write_text(f"#!/bin/sh\ntouch {hook_marker}\n")
    (hooks / "pre-push").chmod(0o755)
    config = workspace / ".git" / "config"
    config.write_text(
        config.read_text()
        + f"[alias]\n\tstatus = !touch {alias_marker}\n"
        + f'[remote "origin"]\n\turl = {tmp_path / "nowhere.git"}\n'
    )

    manager.publish_issue("owner/repo", 7, "secret-token", ref="farm/7-calib", expected_sha=head)

    assert _git_out(remote, "rev-parse", "refs/heads/farm/7-calib") == head
    assert not hook_marker.exists()
    assert not alias_marker.exists()
    assert "nowhere.git" not in (workspace / ".git" / "config").read_text()
