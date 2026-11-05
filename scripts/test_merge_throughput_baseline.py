import copy
import json
import subprocess
from pathlib import Path

import merge_throughput_baseline as mtb
import pytest

DRIVER = Path(__file__).with_name("merge_throughput_baseline.py")

BASE = {
    "slice_lines": 1000,
    "commit": "base",
    "reason": "start",
    "configs": {
        "a.cfg": {"ir": 100, "stream_secs": 10.0},
        "b.cfg": {"ir": 200, "stream_secs": 20.0},
        "c.cfg": {"ir": 300, "stream_secs": 30.0},
    },
    "pi_calibration": {"commit": "cal", "variants": [1, 2]},
}


def variant(**changes):
    value = copy.deepcopy(BASE)
    for key, new in changes.items():
        value[key] = new
    return value


def with_config(name, ir, secs=None):
    value = copy.deepcopy(BASE)
    value["configs"][name] = {
        "ir": ir,
        "stream_secs": secs or value["configs"][name]["stream_secs"],
    }
    return value


def test_a_change_on_one_side_only_is_taken():
    merged = mtb.merge(BASE, with_config("a.cfg", 90), BASE)
    assert merged["configs"]["a.cfg"]["ir"] == 90
    assert merged == mtb.merge(BASE, BASE, with_config("a.cfg", 90))


def test_two_accepts_of_different_configs_both_survive():
    merged = mtb.merge(
        BASE, with_config("a.cfg", 90), with_config("b.cfg", 190)
    )
    assert merged["configs"]["a.cfg"]["ir"] == 90
    assert merged["configs"]["b.cfg"]["ir"] == 190


def test_two_accepts_of_the_same_config_keep_the_lower_ir_with_its_own_seconds():
    ours = with_config("a.cfg", 95, 9.5)
    theirs = with_config("a.cfg", 90, 9.0)
    merged = mtb.merge(BASE, ours, theirs)
    assert merged["configs"]["a.cfg"] == {"ir": 90, "stream_secs": 9.0}
    assert mtb.merge(BASE, theirs, ours)["configs"]["a.cfg"] == {
        "ir": 90,
        "stream_secs": 9.0,
    }


def test_commit_is_ours_and_differing_reasons_are_both_kept():
    ours = variant(commit="mine", reason="saved A")
    theirs = variant(commit="theirs", reason="saved B")
    merged = mtb.merge(BASE, ours, theirs)
    assert merged["commit"] == "mine"
    assert merged["reason"] == "saved A; saved B"


def test_a_calibration_changed_differently_on_both_sides_is_a_real_conflict():
    ours = variant(pi_calibration={"commit": "x"})
    theirs = variant(pi_calibration={"commit": "y"})
    with pytest.raises(mtb.Unresolvable, match="pi_calibration"):
        mtb.merge(BASE, ours, theirs)


def test_a_calibration_changed_on_one_side_merges_with_an_accept_on_the_other():
    ours = with_config("a.cfg", 90)
    theirs = variant(pi_calibration={"commit": "new", "variants": []})
    merged = mtb.merge(BASE, ours, theirs)
    assert (
        merged["configs"]["a.cfg"]["ir"] == 90
        and merged["pi_calibration"]["commit"] == "new"
    )


def git(cwd, *args):
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


def test_git_runs_the_driver_for_a_real_merge_and_leaves_no_conflict(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init", "-q", "-b", "main")
    for key, value in (("user.email", "t@t"), ("user.name", "t")):
        git(repo, "config", key, value)
    git(
        repo,
        "config",
        "merge.throughput-baseline.driver",
        f"python3 {DRIVER} %O %A %B %P",
    )
    (repo / ".gitattributes").write_text(
        "baseline.json merge=throughput-baseline\n"
    )
    path = repo / "baseline.json"
    path.write_text(json.dumps(BASE, indent=2) + "\n")
    git(repo, "add", "-A")
    git(repo, "commit", "-qm", "base")
    git(repo, "checkout", "-q", "-b", "other")
    path.write_text(
        json.dumps(
            mtb.merge(BASE, with_config("b.cfg", 150), BASE)
            | {"commit": "other"},
            indent=2,
        )
        + "\n"
    )
    git(repo, "commit", "-qam", "accept b")
    git(repo, "checkout", "-q", "main")
    path.write_text(
        json.dumps(with_config("a.cfg", 80) | {"commit": "main"}, indent=2)
        + "\n"
    )
    git(repo, "commit", "-qam", "accept a")
    git(repo, "merge", "-q", "--no-edit", "other")
    merged = json.loads(path.read_text())
    assert (
        merged["configs"]["a.cfg"]["ir"] == 80
        and merged["configs"]["b.cfg"]["ir"] == 150
    )
    assert "<<<<" not in path.read_text()
