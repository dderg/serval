import subprocess
from pathlib import Path

INSTALL = Path(__file__).with_name("install-git-guards.sh")


def git(cwd, *args, check=True):
    return subprocess.run(
        ["git", *args], cwd=cwd, capture_output=True, text=True, check=check
    )


def make_repo(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init", "-q", "-b", "main")
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(repo, "config", key, value)
    (repo / ".githooks").mkdir()
    for name in ("reference-transaction", "post-checkout"):
        hook = INSTALL.parent.parent / ".githooks" / name
        (repo / ".githooks" / name).write_text(hook.read_text())
        (repo / ".githooks" / name).chmod(0o755)
    (repo / "scripts").mkdir()
    (repo / "scripts" / "install-git-guards.sh").write_text(INSTALL.read_text())
    (repo / "f").write_text("a")
    git(repo, "add", "-A")
    git(repo, "commit", "-qm", "init")
    subprocess.run(
        ["bash", "scripts/install-git-guards.sh"], cwd=repo, check=True
    )
    return repo


def test_stash_is_refused_and_the_work_is_left_in_place(tmp_path):
    repo = make_repo(tmp_path)
    (repo / "f").write_text("work in progress")
    result = git(repo, "stash", check=False)
    assert result.returncode != 0 and "forbidden" in result.stderr
    assert (repo / "f").read_text() == "work in progress"
    assert git(repo, "stash", "list").stdout == ""


def test_stash_is_refused_in_every_worktree_of_the_clone(tmp_path):
    repo = make_repo(tmp_path)
    other = tmp_path / "other"
    git(repo, "worktree", "add", "-q", str(other), "-b", "topic")
    (other / "f").write_text("other work")
    result = git(other, "stash", check=False)
    assert result.returncode != 0 and "forbidden" in result.stderr
    assert (other / "f").read_text() == "other work"


def test_autostash_is_refused_too(tmp_path):
    repo = make_repo(tmp_path)
    git(repo, "branch", "side")
    (repo / "f").write_text("dirty")
    result = git(repo, "merge", "--autostash", "side", check=False)
    assert git(repo, "stash", "list").stdout == ""
    assert (repo / "f").read_text() == "dirty"
    assert result.returncode in (0, 128)


def test_an_existing_stash_entry_can_still_be_dropped(tmp_path):
    repo = make_repo(tmp_path)
    hooks = repo / ".git" / "hooks" / "reference-transaction"
    saved = hooks.read_text()
    hooks.unlink()
    (repo / "f").write_text("old stash")
    git(repo, "stash")
    hooks.write_text(saved)
    hooks.chmod(0o755)
    assert git(repo, "stash", "drop", check=False).returncode == 0
    assert git(repo, "stash", "list").stdout == ""


def test_the_installer_is_idempotent(tmp_path):
    repo = make_repo(tmp_path)
    again = subprocess.run(
        ["bash", "scripts/install-git-guards.sh"],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    assert (
        again.returncode == 0 and "installed" not in again.stderr + again.stdout
    )


def test_a_plain_worktree_add_prints_the_wt_sh_nudge_but_succeeds(tmp_path):
    repo = make_repo(tmp_path)
    other = tmp_path / "serval-topic"
    result = git(
        repo, "worktree", "add", str(other), "-b", "topic", check=False
    )
    assert result.returncode == 0
    assert "wt.sh new" in result.stderr


def test_a_pool_or_scratch_worktree_is_not_nagged(tmp_path):
    repo = make_repo(tmp_path)
    for name in ("serval-pool-9", "scratch"):
        result = git(
            repo,
            "worktree",
            "add",
            "--detach",
            str(tmp_path / name),
            check=False,
        )
        assert result.returncode == 0 and "wt.sh" not in result.stderr
