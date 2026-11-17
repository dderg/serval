"""The committed scenario-fuzzer fixtures: every repro and regression
directory is named for its issue and loads, and the repros README lists
exactly the repros there are."""

import pathlib
import re

import pytest

from tools.sim.fuzzing import scenario

pytestmark = pytest.mark.sim_unit

FUZZING_DIR = pathlib.Path(__file__).resolve().parents[1] / "fuzzing"
SAVED_KINDS = ("repros", "regressions")
ISSUE_DIR = re.compile(r"issue-\d+")
README_ROW = re.compile(r"^\| #(\d+) \|", re.MULTILINE)


def _saved(kind: str) -> list[pathlib.Path]:
    return sorted(p for p in (FUZZING_DIR / kind).iterdir() if p.is_dir())


@pytest.mark.parametrize("kind", SAVED_KINDS)
def test_every_saved_scenario_is_named_for_its_issue_and_loads(kind):
    for directory in _saved(kind):
        assert ISSUE_DIR.fullmatch(directory.name), directory
        assert scenario.load(directory).programs, directory


def test_the_repros_readme_lists_exactly_the_repros():
    readme = (FUZZING_DIR / "repros" / "README.md").read_text()
    listed = {f"issue-{number}" for number in README_ROW.findall(readme)}
    present = {directory.name for directory in _saved("repros")}
    assert listed == present


def test_a_bug_is_a_repro_or_a_regression_not_both():
    repros = {d.name for d in _saved("repros")}
    regressions = {d.name for d in _saved("regressions")}
    assert not repros & regressions
