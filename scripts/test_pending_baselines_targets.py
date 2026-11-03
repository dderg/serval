from pathlib import Path

import pending_baselines as pending


def test_each_worktree_gets_its_own_target_dir(tmp_path):
    a, b = tmp_path / "a" / "wt", tmp_path / "b" / "wt"
    a.mkdir(parents=True)
    b.mkdir(parents=True)
    assert pending.target_dir_for(a) != pending.target_dir_for(b)


def test_a_worktrees_target_dir_is_stable_and_purpose_keyed(tmp_path):
    tree = tmp_path / "wt"
    tree.mkdir()
    assert pending.target_dir_for(tree) == pending.target_dir_for(tree)
    assert pending.target_dir_for(tree) != pending.target_dir_for(tree, "audit")
    assert pending.target_dir_for(tree).parent == pending.TARGETS


def test_the_target_dir_never_lies_inside_a_worktree(tmp_path):
    tree = tmp_path / "wt"
    tree.mkdir()
    assert tree not in pending.target_dir_for(tree).parents
