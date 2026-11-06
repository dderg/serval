import os
import subprocess
from pathlib import Path

WT = Path(__file__).with_name("wt.sh")
LAND = Path(__file__).with_name("land-pr.sh")
USERS = Path(__file__).with_name("worktree_users.sh")


def git(cwd, *args):
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def make_repo(tmp_path):
    origin = tmp_path / "origin.git"
    git(tmp_path, "init", "-q", "--bare", "-b", "main", str(origin))
    repo = tmp_path / "repo"
    git(tmp_path, "clone", "-q", str(origin), str(repo))
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(repo, "config", key, value)
    (repo / "a").write_text("1")
    git(repo, "add", "a")
    git(repo, "commit", "-qm", "one")
    git(repo, "push", "-q", "origin", "main")
    return repo


def release(path):
    (Path(git(path, "rev-parse", "--absolute-git-dir")) / "wt-lease").unlink(
        missing_ok=True
    )


def wt(repo, tmp_path, *args):
    env = {**os.environ, "WT_POOL_DIR": str(tmp_path)}
    return subprocess.run(
        ["bash", str(WT), *args],
        cwd=repo,
        env=env,
        capture_output=True,
        text=True,
    )


def test_new_creates_the_first_pool_worktree_on_a_topic_branch(tmp_path):
    repo = make_repo(tmp_path)
    result = wt(repo, tmp_path, "new", "topic-a")
    assert result.returncode == 0, result.stderr
    path = Path(result.stdout.strip())
    assert path == tmp_path / "serval-pool-1"
    assert git(path, "rev-parse", "--abbrev-ref", "HEAD") == "topic-a"


def test_a_busy_pool_worktree_is_not_reused_and_a_second_is_made(tmp_path):
    repo = make_repo(tmp_path)
    wt(repo, tmp_path, "new", "topic-a")
    result = wt(repo, tmp_path, "new", "topic-b")
    assert Path(result.stdout.strip()) == tmp_path / "serval-pool-2"


def test_an_idle_pool_worktree_is_reused_with_its_files_kept(tmp_path):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    with (repo / ".git" / "info" / "exclude").open("a") as exclude:
        exclude.write("target-marker\n")
    (first / "target-marker").write_text("warm")
    git(first, "switch", "-q", "--detach", "origin/main")
    release(first)
    again = Path(wt(repo, tmp_path, "new", "topic-b").stdout.strip())
    assert again == first
    assert (again / "target-marker").read_text() == "warm"
    assert git(again, "rev-parse", "--abbrev-ref", "HEAD") == "topic-b"


def test_a_dirty_idle_worktree_is_not_reused(tmp_path):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    git(first, "switch", "-q", "--detach", "origin/main")
    release(first)
    (first / "a").write_text("edited")
    second = Path(wt(repo, tmp_path, "new", "topic-b").stdout.strip())
    assert second != first


def test_an_existing_branch_name_is_refused(tmp_path):
    repo = make_repo(tmp_path)
    git(repo, "branch", "taken")
    result = wt(repo, tmp_path, "new", "taken")
    assert result.returncode != 0 and "exists" in result.stderr


def test_list_reports_idle_and_busy(tmp_path):
    repo = make_repo(tmp_path)
    wt(repo, tmp_path, "new", "topic-a")
    assert "busy" in wt(repo, tmp_path, "list").stdout


def test_concurrent_allocations_never_share_an_idle_worktree(tmp_path):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "seed").stdout.strip())
    git(first, "switch", "-q", "--detach", "origin/main")
    git(first, "branch", "-D", "seed")
    release(first)
    env = {**os.environ, "WT_POOL_DIR": str(tmp_path)}
    procs = [
        subprocess.Popen(
            ["bash", str(WT), "new", f"race-{i}"],
            cwd=repo,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        for i in range(6)
    ]
    outputs = [p.communicate() for p in procs]
    assert all(p.returncode == 0 for p in procs), outputs
    paths = [out.strip() for out, _ in outputs]
    assert len(set(paths)) == 6, paths


def test_a_lease_whose_topic_branch_still_exists_keeps_a_detached_clean_worktree_busy(
    tmp_path,
):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    git(first, "switch", "-q", "--detach", "origin/main")
    lease = Path(git(first, "rev-parse", "--absolute-git-dir")) / "wt-lease"
    assert "topic=topic-a" in lease.read_text()
    second = Path(wt(repo, tmp_path, "new", "topic-b").stdout.strip())
    assert second == tmp_path / "serval-pool-2"
    git(first, "branch", "-D", "topic-a")
    third = Path(wt(repo, tmp_path, "new", "topic-c").stdout.strip())
    assert third == first


def test_a_process_with_its_cwd_inside_makes_an_idle_looking_worktree_busy(
    tmp_path,
):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    git(first, "switch", "-q", "--detach", "origin/main")
    git(first, "branch", "-D", "topic-a")
    (first / "sub").mkdir()
    sitter = subprocess.Popen(["sleep", "30"], cwd=first / "sub")
    try:
        second = Path(wt(repo, tmp_path, "new", "topic-b").stdout.strip())
        assert second == tmp_path / "serval-pool-2"
        users = subprocess.run(
            ["bash", str(USERS), str(first)], capture_output=True, text=True
        ).stdout
        assert users.startswith(str(sitter.pid))
    finally:
        sitter.kill()
        sitter.wait()
    third = Path(wt(repo, tmp_path, "new", "topic-c").stdout.strip())
    assert third == first


def test_the_callers_own_cwd_does_not_make_a_worktree_busy(tmp_path):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    users = subprocess.run(
        ["bash", str(USERS), str(first)],
        cwd=first,
        capture_output=True,
        text=True,
    ).stdout
    assert users == ""


def test_an_untracked_file_makes_a_worktree_busy(tmp_path):
    repo = make_repo(tmp_path)
    first = Path(wt(repo, tmp_path, "new", "topic-a").stdout.strip())
    git(first, "switch", "-q", "--detach", "origin/main")
    git(first, "branch", "-D", "topic-a")
    (first / "stray.txt").write_text("wip")
    release(first)
    second = Path(wt(repo, tmp_path, "new", "topic-b").stdout.strip())
    assert second == tmp_path / "serval-pool-2"
