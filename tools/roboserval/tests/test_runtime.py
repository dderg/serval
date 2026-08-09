from __future__ import annotations

import os
from pathlib import Path

import pytest

from serval_bot.runtime import AgentDirFailure, slot_env


def _session_dir(tmp_path: Path, issue: int = 7) -> Path:
    workspace = tmp_path / "workspaces" / "owner--repo" / str(issue)
    workspace.mkdir(parents=True)
    session_dir = tmp_path / "sessions" / "owner--repo" / str(issue)
    session_dir.mkdir(parents=True)
    return session_dir


def _shared_agent_dir(tmp_path: Path) -> Path:
    shared = tmp_path / "omp-agent"
    (shared / "config").mkdir(parents=True)
    (shared / "auth").mkdir()
    (shared / "config.yml").write_text("model: test\n")
    (shared / "config" / "overlay.yml").write_text("thinking: low\n")
    (shared / "auth" / "credentials.json").write_text('{"key": "value"}\n')
    (shared / "agent.db").write_bytes(b"legacy agent db")
    (shared / "history.db").write_bytes(b"legacy history db")
    (shared / "models").mkdir()
    (shared / "models" / "model.bin").write_bytes(b"model weights")
    (shared / "sessions").mkdir()
    (shared / "sessions" / "old.jsonl").write_text("old session\n")
    return shared


def _snapshot(root: Path) -> dict[str, bytes]:
    return {str(path.relative_to(root)): path.read_bytes() for path in root.rglob("*") if path.is_file()}


def test_slot_env_provisions_private_agent_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = _shared_agent_dir(tmp_path)
    monkeypatch.setenv("PI_CODING_AGENT_DIR", str(shared))
    before = _snapshot(shared)
    session_dir = _session_dir(tmp_path)

    env = slot_env(None, tmp_path / "workspaces" / "owner--repo" / "7", session_dir)

    private = session_dir / ".omp-agent"
    assert env["PI_CODING_AGENT_DIR"] == str(private)
    assert private.is_dir()
    assert os.access(private, os.W_OK)
    assert (private / "config.yml").read_text() == "model: test\n"
    assert (private / "config" / "overlay.yml").read_text() == "thinking: low\n"
    assert (private / "auth" / "credentials.json").read_text() == '{"key": "value"}\n'
    assert not (private / "agent.db").exists()
    assert not (private / "history.db").exists()
    assert not (private / "models").exists()
    assert not (private / "sessions").exists()
    assert _snapshot(shared) == before


def test_slot_env_reprovisions_agent_dir_per_event(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = _shared_agent_dir(tmp_path)
    monkeypatch.setenv("PI_CODING_AGENT_DIR", str(shared))
    session_dir = _session_dir(tmp_path)
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"

    slot_env(None, workspace, session_dir)
    private = session_dir / ".omp-agent"
    (private / "stale.txt").write_text("junk\n")

    env = slot_env(None, workspace, session_dir)

    assert env["PI_CODING_AGENT_DIR"] == str(private)
    assert not (private / "stale.txt").exists()
    assert (private / "config.yml").exists()


def test_slot_env_missing_shared_source_seeds_empty_private(tmp_path: Path) -> None:
    session_dir = _session_dir(tmp_path)
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"

    env = slot_env(None, workspace, session_dir)

    private = session_dir / ".omp-agent"
    assert env["PI_CODING_AGENT_DIR"] == str(private)
    assert private.is_dir()
    assert list(private.iterdir()) == []


def test_slot_env_rejects_symlink_seed_sources(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = tmp_path / "omp-agent"
    (shared / "config").mkdir(parents=True)
    (shared / "config.yml").symlink_to("/etc/hosts")
    (shared / "config" / "evil.yml").symlink_to("/etc/passwd")
    monkeypatch.setenv("PI_CODING_AGENT_DIR", str(shared))
    session_dir = _session_dir(tmp_path)
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"

    with pytest.raises(AgentDirFailure, match="symlink"):
        slot_env(None, workspace, session_dir)

    (shared / "config.yml").unlink()
    with pytest.raises(AgentDirFailure, match="symlink"):
        slot_env(None, workspace, session_dir)


def test_slot_env_rejects_unexpected_seed_source_type(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = tmp_path / "omp-agent"
    shared.mkdir()
    os.mkfifo(shared / "config")
    monkeypatch.setenv("PI_CODING_AGENT_DIR", str(shared))
    session_dir = _session_dir(tmp_path)
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"

    with pytest.raises(AgentDirFailure, match="unexpected type"):
        slot_env(None, workspace, session_dir)


def test_slot_env_seeds_agent_dir_before_chown(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    shared = _shared_agent_dir(tmp_path)
    monkeypatch.setenv("PI_CODING_AGENT_DIR", str(shared))
    seen: dict[str, bool] = {}

    def spy_chown(workspace: Path, session_dir: Path, slot_uid: int | None) -> None:
        seen["seeded"] = (session_dir / ".omp-agent" / "config.yml").exists()
        seen["agent_dir_exists"] = (session_dir / ".omp-agent").is_dir()

    monkeypatch.setattr("serval_bot.runtime.chown_event_paths", spy_chown)
    session_dir = _session_dir(tmp_path)
    workspace = tmp_path / "workspaces" / "owner--repo" / "7"

    slot_env(2003, workspace, session_dir)

    assert seen["agent_dir_exists"] is True
    assert seen["seeded"] is True
