import os
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).with_name("land-pr.sh")


def git(cwd, *args):
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def make_clone(tmp_path):
    origin = tmp_path / "origin.git"
    git(tmp_path, "init", "-q", "--bare", "-b", "main", str(origin))
    seed = tmp_path / "seed"
    git(tmp_path, "clone", "-q", str(origin), str(seed))
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(seed, "config", key, value)
    (seed / "a").write_text("1")
    git(seed, "add", "a")
    git(seed, "commit", "-qm", "one")
    git(seed, "push", "-q", "origin", "main")
    checkout = tmp_path / "checkout"
    git(tmp_path, "clone", "-q", str(origin), str(checkout))
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(checkout, "config", key, value)
    (seed / "a").write_text("2")
    git(seed, "commit", "-qam", "two")
    git(seed, "push", "-q", "origin", "main")
    git(checkout, "fetch", "-q", "origin")
    return checkout


def advance_main(checkout):
    script = SCRIPT.read_text().rsplit('main "$@"', 1)[0]
    return subprocess.run(
        ["bash", "-c", script + "\ndry_run=0\nadvance_main"],
        cwd=checkout,
        env={**os.environ, "GH_UTIL": str(SCRIPT.with_name("gh_util.py"))},
        capture_output=True,
        text=True,
    )


def test_a_clean_checked_out_main_is_fast_forwarded(tmp_path):
    checkout = make_clone(tmp_path)
    result = advance_main(checkout)
    assert result.returncode == 0, result.stderr
    assert git(checkout, "rev-parse", "main") == git(
        checkout, "rev-parse", "origin/main"
    )


def test_a_checked_out_main_with_local_changes_is_left_alone(tmp_path):
    checkout = make_clone(tmp_path)
    (checkout / "a").write_text("local edit")
    before = git(checkout, "rev-parse", "main")
    result = advance_main(checkout)
    assert git(checkout, "rev-parse", "main") == before
    assert (checkout / "a").read_text() == "local edit"
    assert "uncommitted changes" in result.stderr


def test_a_main_that_is_not_checked_out_is_moved_to_origin_main(tmp_path):
    checkout = make_clone(tmp_path)
    git(checkout, "checkout", "-q", "-b", "other")
    result = advance_main(checkout)
    assert result.returncode == 0, result.stderr
    assert git(checkout, "rev-parse", "main") == git(
        checkout, "rev-parse", "origin/main"
    )
