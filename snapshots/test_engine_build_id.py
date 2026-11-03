import subprocess
from types import SimpleNamespace

import engine_build_id as ebi
import pytest


def git(cwd, *args):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True)


@pytest.fixture
def repo(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "rust").mkdir()
    (tmp_path / "rust" / "lib.rs").write_text("fn a() {}\n")
    (tmp_path / "notes.txt").write_text("not rust\n")
    git(tmp_path, "add", "-A")
    ebi.tree_build_id.cache_clear()
    yield tmp_path
    ebi.tree_build_id.cache_clear()


def build_id(repo):
    ebi.tree_build_id.cache_clear()
    return ebi.tree_build_id(repo)


def test_the_id_is_stable_and_has_the_documented_length(repo):
    assert build_id(repo) == build_id(repo)
    assert len(build_id(repo)) == ebi.ID_HEX_CHARS


def test_editing_a_rust_file_changes_the_id_without_committing(repo):
    before = build_id(repo)
    (repo / "rust" / "lib.rs").write_text("fn a() { 1; }\n")
    assert build_id(repo) != before


def test_a_new_untracked_rust_file_changes_the_id(repo):
    before = build_id(repo)
    (repo / "rust" / "new.rs").write_text("fn b() {}\n")
    assert build_id(repo) != before


def test_a_change_outside_rust_does_not(repo):
    before = build_id(repo)
    (repo / "notes.txt").write_text("changed\n")
    assert build_id(repo) == before


def engine(repo, built):
    return SimpleNamespace(
        __file__=str(repo / "klippy" / "_motion_engine.so"),
        build_id=lambda: built,
    )


def test_an_engine_built_from_this_tree_is_accepted(repo):
    ebi.verify(engine(repo, build_id(repo)))


def test_an_engine_built_from_other_sources_is_refused_with_the_rebuild_hint(
    repo,
):
    with pytest.raises(ebi.EngineMismatch, match="make -f Makefile.rust"):
        ebi.verify(engine(repo, "0" * ebi.ID_HEX_CHARS))


def test_an_engine_built_outside_git_but_sitting_in_a_git_tree_is_refused(repo):
    with pytest.raises(ebi.EngineMismatch, match="outside a git tree"):
        ebi.verify(engine(repo, ebi.UNKNOWN))


def test_an_engine_without_a_build_id_is_refused(repo):
    old = SimpleNamespace(__file__=str(repo / "klippy" / "_motion_engine.so"))
    with pytest.raises(ebi.EngineMismatch, match="predates build ids"):
        ebi.verify(old)


def test_a_git_less_tree_with_an_unknown_engine_has_nothing_to_compare(
    tmp_path,
):
    ebi.verify(engine(tmp_path, ebi.UNKNOWN))


def test_a_git_less_tree_with_an_identified_engine_is_refused(tmp_path):
    with pytest.raises(ebi.EngineMismatch, match="not a git tree"):
        ebi.verify(engine(tmp_path, "a" * ebi.ID_HEX_CHARS))
