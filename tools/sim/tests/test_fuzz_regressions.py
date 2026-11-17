"""Saved scenarios of fixed product bugs (tools/sim/fuzzing/regressions/
issue-N/), each moved there from tools/sim/fuzzing/repros/ by its fix:
replayed, every one must pass."""

import pathlib

import pytest

from tools.sim.fuzzing import execute, scenario

pytestmark = pytest.mark.needs_elf

REGRESSIONS_DIR = (
    pathlib.Path(__file__).resolve().parents[1] / "fuzzing" / "regressions"
)
REGRESSIONS = sorted(
    path for path in REGRESSIONS_DIR.iterdir() if path.is_dir()
)


@pytest.mark.parametrize("directory", REGRESSIONS, ids=lambda path: path.name)
def test_a_fixed_bug_stays_fixed(directory, tmp_path):
    chosen = scenario.load(directory)
    result = execute.run_scenario(
        chosen, tmp_path / "world", tmp_path / "failure"
    )
    assert result["status"] == execute.PASS, (
        f"{directory.name}: {scenario.describe(chosen)}\n"
        f"{result['signature']}\n{result['evidence']}"
    )
