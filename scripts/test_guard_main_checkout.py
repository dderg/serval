import os
import subprocess
from pathlib import Path

GUARD = Path(__file__).with_name("guard-main-checkout.sh")


def git(cwd, *args):
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def guard(mode, tree, any_name=True):
    env = {**os.environ}
    if any_name:
        env["GUARD_ANY"] = "1"
    return subprocess.run(
        ["bash", str(GUARD), mode, str(tree)],
        env=env,
        capture_output=True,
        text=True,
    )


def make_main(tmp_path, name="serval"):
    repo = tmp_path / name
    repo.mkdir()
    git(repo, "init", "-q", "-b", "main")
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(repo, "config", key, value)
    (repo / "sub").mkdir()
    (repo / "sub" / "a.txt").write_text("1")
    (repo / "b.txt").write_text("1")
    git(repo, "add", "-A")
    git(repo, "commit", "-qm", "one")
    return repo


def can_write(path: Path) -> bool:
    return os.access(path, os.W_OK)


def test_apply_makes_tracked_files_and_their_directories_unwritable(tmp_path):
    repo = make_main(tmp_path)
    assert guard("apply", repo).returncode == 0
    assert not can_write(repo / "b.txt") and not can_write(
        repo / "sub" / "a.txt"
    )
    assert not can_write(repo / "sub") and not can_write(repo)
    try:
        (repo / "b.txt").write_text("edit")
        wrote = True
    except PermissionError:
        wrote = False
    assert not wrote
    guard("lift", repo)


def test_git_itself_still_works_on_a_guarded_checkout(tmp_path):
    repo = make_main(tmp_path)
    guard("apply", repo)
    git(repo, "branch", "other")
    assert "main" in git(repo, "status", "-sb")
    guard("lift", repo)


def test_lift_restores_write_and_a_fast_forward_then_works(tmp_path):
    repo = make_main(tmp_path)
    git(repo, "checkout", "-q", "-b", "next")
    (repo / "b.txt").write_text("2")
    git(repo, "commit", "-qam", "two")
    git(repo, "checkout", "-q", "main")
    guard("apply", repo)
    guard("lift", repo)
    git(repo, "merge", "-q", "--ff-only", "next")
    assert (repo / "b.txt").read_text() == "2"
    guard("apply", repo)
    assert not can_write(repo / "b.txt")
    guard("lift", repo)


def test_untracked_files_and_git_dir_are_left_alone(tmp_path):
    repo = make_main(tmp_path)
    (repo / "scratch.txt").write_text("x")
    guard("apply", repo)
    assert can_write(repo / "scratch.txt") and can_write(repo / ".git")
    guard("lift", repo)


def test_a_checkout_not_named_serval_is_ignored_without_the_override(tmp_path):
    repo = make_main(tmp_path, name="other-clone")
    result = guard("apply", repo, any_name=False)
    assert result.returncode == 0 and can_write(repo / "b.txt")


def test_apply_is_idempotent(tmp_path):
    repo = make_main(tmp_path)
    assert (
        guard("apply", repo).returncode == 0
        and guard("apply", repo).returncode == 0
    )
    guard("lift", repo)
