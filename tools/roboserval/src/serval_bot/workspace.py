from __future__ import annotations

import base64
import os
import re
import shutil
import stat
import subprocess
import threading
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path

from serval_bot.runtime import MAX_SLOTS, slot_uids

_REPO_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
_BRANCH_PATTERN = re.compile(r"^[A-Za-z0-9._/-]+$")
_PULL_REF_PATTERN = re.compile(r"^refs/pull/[1-9][0-9]*/head$")
_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")

_POOL_DIRNAME = "pool.git"
_LEGACY_POOL_DIRNAME = "clone.git"
_LEGACY_REVIEW_REF = "refs/roboserval/legacy/head"

_lock_guard = threading.Lock()
_repo_locks: dict[tuple[str, str], threading.Lock] = {}

# Root writes this exact config into every issue workspace before running any
# Git command there, so agent-controlled settings (aliases, filters, hooks,
# transports) can never survive into a privileged invocation. Identity is
# fixed so the slot can commit without setup; the slot owns the file after
# handoff and may change it, and the next sync rewrites it again.
_TRUSTED_CONFIG = """\
[core]
\trepositoryformatversion = 0
\tfilemode = true
\tbare = false
\tlogallrefupdates = true
\thooksPath =
[protocol "file"]
\tallow = always
[user]
\tname = RoboServal
\temail = roboserval@localhost
[remote "origin"]
\turl = {pool}
\tfetch = +refs/heads/*:refs/remotes/origin/*
"""

# Fixed overrides for every root Git invocation. Environment-level
# GIT_CONFIG_* entries take precedence over repository config, so these pin
# the values an agent could otherwise poison: no hooks, no fsmonitor, and a
# transport policy that never needs a special case.
_GIT_SAFE_OVERRIDES = (
    ("core.hooksPath", ""),
    ("core.fsmonitor", ""),
    ("protocol.file.allow", "always"),
)


class WorkspaceFailure(RuntimeError):
    pass


def _repo_lock(root: Path, repo_key: str) -> threading.Lock:
    key = (str(root.resolve()), repo_key)
    with _lock_guard:
        lock = _repo_locks.get(key)
        if lock is None:
            lock = _repo_locks[key] = threading.Lock()
        return lock


def _validate_revision(repo: str, branch: str | None, fetch_ref: str | None, expected_sha: str | None) -> None:
    if not _REPO_PATTERN.fullmatch(repo):
        raise WorkspaceFailure(f"invalid repository: {repo}")
    if branch is not None and (not _BRANCH_PATTERN.fullmatch(branch) or ".." in branch or branch.startswith("/")):
        raise WorkspaceFailure(f"invalid branch: {branch}")
    if (fetch_ref is None) != (expected_sha is None):
        raise WorkspaceFailure("fetch_ref and expected_sha must be set together")
    if fetch_ref is not None and (
        not _PULL_REF_PATTERN.fullmatch(fetch_ref) or not _SHA_PATTERN.fullmatch(expected_sha or "")
    ):
        raise WorkspaceFailure("invalid pull request revision")


def _git_environment(
    environment: dict[str, str],
    *repos: Path,
    overrides: tuple[tuple[str, str], ...] = (),
) -> dict[str, str]:
    """Sanitized environment for root Git commands.

    System and global config are ignored; the caller's GIT_CONFIG_* entries
    (e.g. the credential header) are preserved; each repo is marked safe; and
    the fixed overrides are appended last so they win over anything an agent
    could have written into repository config.
    """
    env = {
        "PATH": environment.get("PATH", os.defpath),
        "HOME": environment.get("HOME", str(Path.home())),
        "LC_ALL": "C",
        "GIT_TERMINAL_PROMPT": "0",
        "GIT_PAGER": "cat",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": os.devnull,
    }
    entries: list[tuple[str, str]] = []
    for index in range(int(environment.get("GIT_CONFIG_COUNT", "0"))):
        key = environment.get(f"GIT_CONFIG_KEY_{index}")
        value = environment.get(f"GIT_CONFIG_VALUE_{index}")
        if key is not None:
            entries.append((key, value))
    for repo in repos:
        entries.append(("safe.directory", str(repo)))
    entries.extend(overrides)
    for index, (key, value) in enumerate(entries):
        env[f"GIT_CONFIG_KEY_{index}"] = key
        env[f"GIT_CONFIG_VALUE_{index}"] = value
    env["GIT_CONFIG_COUNT"] = str(len(entries))
    return env


def _issue_overrides(pool: Path) -> tuple[tuple[str, str], ...]:
    return (
        *_GIT_SAFE_OVERRIDES,
        ("remote.origin.url", str(pool)),
        ("remote.origin.fetch", "+refs/heads/*:refs/remotes/origin/*"),
    )


def _trusted_owners(*, allow_slots: bool) -> frozenset[int]:
    owners = {os.geteuid()}
    if allow_slots:
        owners.update(slot_uids(MAX_SLOTS))
    return frozenset(owners)


def _validate_repo_control(repo: Path, *, allow_slots: bool, bare: bool = False) -> None:
    """Fail loudly when repository control data is writable by an untrusted principal."""
    trusted = _trusted_owners(allow_slots=allow_slots)
    gitdir = repo if bare else repo / ".git"

    def fail(reason: str) -> None:
        raise WorkspaceFailure(f"unsafe repository control data at {repo}: {reason}")

    for ancestor in (repo.parent, repo.parent.parent):
        try:
            st = ancestor.lstat()
        except FileNotFoundError:
            fail(f"missing parent {ancestor}")
        if stat.S_ISLNK(st.st_mode) or not stat.S_ISDIR(st.st_mode):
            fail(f"parent {ancestor} is not a directory")
        if st.st_uid != os.geteuid():
            fail(f"parent {ancestor} owned by uid {st.st_uid}")
        if st.st_mode & 0o022:
            fail(f"parent {ancestor} is group or world writable (mode {stat.S_IMODE(st.st_mode):o})")

    for path, label in ((repo, "repository directory"), (gitdir, "git metadata directory")):
        try:
            st = path.lstat()
        except FileNotFoundError:
            fail(f"missing {label}")
        if stat.S_ISLNK(st.st_mode):
            fail(f"{label} is a symlink")
        if not stat.S_ISDIR(st.st_mode):
            fail(f"{label} is not a directory")
        if st.st_uid not in trusted:
            fail(f"{label} owned by uid {st.st_uid}")
        if st.st_mode & 0o022:
            fail(f"{label} is group or world writable (mode {stat.S_IMODE(st.st_mode):o})")

    for dirpath, dirnames, filenames in os.walk(gitdir, followlinks=False):
        for name in dirnames + filenames:
            path = Path(dirpath) / name
            try:
                st = path.lstat()
            except FileNotFoundError:
                continue
            if stat.S_ISLNK(st.st_mode):
                fail(f"symlink inside git metadata: {path}")

    config = gitdir / "config"
    try:
        st = config.lstat()
    except FileNotFoundError:
        pass
    else:
        if st.st_uid not in trusted:
            fail(f"git config owned by uid {st.st_uid}")
        if st.st_mode & 0o022:
            fail(f"git config is group or world writable (mode {stat.S_IMODE(st.st_mode):o})")


def _secure_pool(pool: Path) -> None:
    """Normalize the shared pool to root-owned, world-readable, non-group-writable."""
    hooks = pool / "hooks"
    if hooks.is_symlink() or hooks.is_file():
        hooks.unlink()
    elif hooks.is_dir():
        shutil.rmtree(hooks)
    for dirpath, dirnames, filenames in os.walk(pool, followlinks=False):
        for name in dirnames:
            (Path(dirpath) / name).chmod(0o755)
        for name in filenames:
            (Path(dirpath) / name).chmod(0o644)


def _ensure_pool(root: Path, repo_key: str, remote: str, environment: dict[str, str]) -> Path:
    """Root-owned bare mirror: the read-only object/reference cache for issues."""
    pool = root / repo_key / _POOL_DIRNAME
    if not (pool / "config").exists():
        _run("git", "init", "--bare", str(pool), cwd=root, environment=environment)
        _run("git", "remote", "add", "origin", remote, cwd=pool, environment=environment)
        _run("git", "config", "remote.origin.fetch", "+refs/heads/*:refs/heads/*", cwd=pool, environment=environment)
        _run("git", "config", "protocol.file.allow", "always", cwd=pool, environment=environment)
        _secure_pool(pool)
    _validate_repo_control(pool, allow_slots=False, bare=True)
    env = _git_environment(environment, pool, overrides=_GIT_SAFE_OVERRIDES)
    _run("git", "fetch", "--prune", "origin", cwd=pool, environment=env)
    default_branch = _pool_default_branch(pool, env)
    _run("git", "symbolic-ref", "HEAD", f"refs/heads/{default_branch}", cwd=pool, environment=env)
    _secure_pool(pool)
    return pool


def _pool_default_branch(pool: Path, environment: dict[str, str]) -> str:
    head = _run("git", "ls-remote", "--symref", "origin", "HEAD", cwd=pool, environment=environment)
    for line in head.splitlines():
        if not line.startswith("ref: "):
            continue
        target = line.removeprefix("ref: ").split("\t", 1)[0]
        prefix = "refs/heads/"
        if not target.startswith(prefix) or len(target) == len(prefix):
            raise WorkspaceFailure(f"invalid origin HEAD: {target}")
        return target.removeprefix(prefix)
    raise WorkspaceFailure(f"origin HEAD not found: {head}")


def _revision(
    pool: Path,
    branch: str,
    pool_pull_ref: str,
    fetch_ref: str | None,
    expected_sha: str | None,
    environment: dict[str, str],
) -> str:
    if fetch_ref is None:
        return _run("git", "rev-parse", f"refs/heads/{branch}", cwd=pool, environment=environment)
    _run("git", "fetch", "--force", "origin", f"{fetch_ref}:{pool_pull_ref}", cwd=pool, environment=environment)
    revision = _run("git", "rev-parse", pool_pull_ref, cwd=pool, environment=environment)
    if revision != expected_sha:
        raise WorkspaceFailure(f"pull request head changed: expected {expected_sha}, got {revision}")
    return revision


def _reject_legacy_layout(repo_dir: Path, destination: Path) -> None:
    """Fail loudly on the old writable shared-Git layout (clone.git + linked worktrees)."""
    if (repo_dir / _LEGACY_POOL_DIRNAME).exists():
        raise WorkspaceFailure(
            f"legacy shared-Git pool {repo_dir / _LEGACY_POOL_DIRNAME} is writable by slots; "
            f"remove {repo_dir} (and any issue directories under it) and re-sync to migrate"
        )
    git_file = destination / ".git"
    if git_file.exists() and not git_file.is_dir():
        raise WorkspaceFailure(f"legacy linked worktree at {destination}; remove {destination.parent} to migrate")


def _reset_metadata(repo: Path, pool: Path) -> None:
    """Recreate untrusted per-issue Git metadata from the trusted pool."""
    gitdir = repo / ".git"
    hooks = gitdir / "hooks"
    if hooks.is_symlink() or hooks.is_file():
        hooks.unlink()
    elif hooks.is_dir():
        shutil.rmtree(hooks)
    for dirpath, _, filenames in os.walk(gitdir, followlinks=False):
        for name in filenames:
            if name.endswith(".lock"):
                with suppress(FileNotFoundError):
                    (Path(dirpath) / name).unlink()
    info = gitdir / "objects" / "info"
    info.mkdir(parents=True, exist_ok=True)
    alternates = info / "alternates"
    if alternates.is_symlink():
        alternates.unlink()
    alternates.write_text(f"{pool}/objects\n")
    config = gitdir / "config"
    if config.is_symlink():
        config.unlink()
    config.write_text(_TRUSTED_CONFIG.format(pool=pool))


def _materialize_standalone(
    pool: Path,
    destination: Path,
    revision: str,
    *,
    environment: dict[str, str],
    pull_ref: str | None = None,
    checkout_branch: str | None = None,
    clean_ignored: bool = True,
) -> None:
    """Materialize one issue workspace as a standalone repository.

    The workspace owns its full writable Git metadata; pool objects are
    reachable only through a read-only alternates file. On re-sync the
    metadata is reset from the trusted pool before any Git command runs, so
    agent-controlled config and hooks are never consumed by root.
    """
    if destination.exists() and not (destination / ".git").is_dir():
        shutil.rmtree(destination)
    if not destination.exists():
        _run(
            "git",
            "clone",
            "--shared",
            "--no-checkout",
            str(pool),
            str(destination),
            cwd=destination.parent,
            environment=environment,
        )
    _validate_repo_control(destination, allow_slots=True)
    _reset_metadata(destination, pool)
    env = _git_environment(environment, destination, pool, overrides=_issue_overrides(pool))
    _run("git", "fetch", "origin", cwd=destination, environment=env)
    _run("git", "remote", "set-head", "origin", "--auto", cwd=destination, environment=env)
    if pull_ref is not None:
        _run("git", "fetch", "origin", f"+{pull_ref}:{pull_ref}", cwd=destination, environment=env)
    if checkout_branch is None:
        _run("git", "checkout", "--detach", "--force", revision, cwd=destination, environment=env)
    else:
        _run("git", "checkout", "-B", checkout_branch, revision, cwd=destination, environment=env)
    _run("git", "reset", "--hard", revision, cwd=destination, environment=env)
    _run("git", "clean", "-fdx" if clean_ignored else "-fd", cwd=destination, environment=env)


def prepare_workspace(
    root: Path,
    repo: str,
    branch: str | None,
    remote: str,
    issue_number: int,
    *,
    environment: dict[str, str],
    fetch_ref: str | None = None,
    expected_sha: str | None = None,
) -> tuple[Path, str]:
    """Standalone per-issue repository backed by a read-only shared object cache.

    Returns (workspace, default_branch). The workspace is hard-reset and
    cleaned to the requested revision, detached at that revision; other
    issues' workspaces are never touched, and the shared pool is never
    writable by slot identities.
    """
    if type(issue_number) is not int or issue_number <= 0:
        raise WorkspaceFailure(f"invalid issue number: {issue_number!r}")
    _validate_revision(repo, branch, fetch_ref, expected_sha)
    root.mkdir(parents=True, exist_ok=True)
    root.chmod(0o755)
    repo_key = repo.replace("/", "--")
    repo_dir = root / repo_key
    repo_dir.mkdir(parents=True, exist_ok=True)
    repo_dir.chmod(0o755)
    workspace = repo_dir / str(issue_number)
    _reject_legacy_layout(repo_dir, workspace)
    pool_pull_ref = f"refs/roboserval/pull/{issue_number}/head"
    with _repo_lock(root, repo_key):
        pool = _ensure_pool(root, repo_key, remote, environment)
        env = _git_environment(environment, pool, overrides=_GIT_SAFE_OVERRIDES)
        default_branch = _pool_default_branch(pool, env)
        revision = _revision(pool, branch or default_branch, pool_pull_ref, fetch_ref, expected_sha, env)
        _materialize_standalone(
            pool,
            workspace,
            revision,
            environment=environment,
            pull_ref=pool_pull_ref if fetch_ref is not None else None,
        )
        return workspace, default_branch


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
        issue_number: int | None = None,
        fetch_ref: str | None = None,
        expected_sha: str | None = None,
    ) -> Path:
        """Prepare a workspace for one issue; without an issue number, fall back
        to a single standalone checkout under root/legacy (pre-worktree callers)."""
        _validate_revision(repo, branch, fetch_ref, expected_sha)
        if issue_number is not None and (type(issue_number) is not int or issue_number <= 0):
            raise WorkspaceFailure(f"invalid issue number: {issue_number!r}")
        self.root.mkdir(parents=True, exist_ok=True)
        self.root.chmod(0o755)
        environment = self._environment(token)
        remote = self.remote_template.format(repo=repo)
        if issue_number is None:
            return self._sync_legacy(repo, branch, remote, fetch_ref, expected_sha, environment)
        workspace, _default_branch = prepare_workspace(
            self.root,
            repo,
            branch,
            remote,
            issue_number,
            environment=environment,
            fetch_ref=fetch_ref,
            expected_sha=expected_sha,
        )
        return workspace

    def _sync_legacy(
        self,
        repo: str,
        branch: str,
        remote: str,
        fetch_ref: str | None,
        expected_sha: str | None,
        environment: dict[str, str],
    ) -> Path:
        repo_key = repo.replace("/", "--")
        legacy_dir = self.root / "legacy"
        legacy_dir.mkdir(parents=True, exist_ok=True)
        legacy_dir.chmod(0o755)
        destination = legacy_dir / repo_key
        _reject_legacy_layout(self.root / repo_key, destination)
        with _repo_lock(self.root, repo_key):
            pool = _ensure_pool(self.root, repo_key, remote, environment)
            env = _git_environment(environment, pool, overrides=_GIT_SAFE_OVERRIDES)
            default_branch = _pool_default_branch(pool, env)
            revision = _revision(pool, branch or default_branch, _LEGACY_REVIEW_REF, fetch_ref, expected_sha, env)
            _materialize_standalone(
                pool,
                destination,
                revision,
                environment=environment,
                pull_ref=_LEGACY_REVIEW_REF if fetch_ref is not None else None,
                checkout_branch=None if fetch_ref is not None else branch,
                clean_ignored=False,
            )
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
