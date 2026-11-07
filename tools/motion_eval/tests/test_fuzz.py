from __future__ import annotations

import pathlib

import pytest

from tools.motion_eval import fuzz

HERE = pathlib.Path(__file__).parent
SEEDS_FILE = HERE / "fuzz_seeds.txt"
CASES_DIR = HERE / "fuzz_cases"


def seeds():
    return [
        int(line.split("#")[0])
        for line in SEEDS_FILE.read_text().splitlines()
        if line.split("#")[0].strip()
    ]


@pytest.mark.parametrize("seed", seeds())
def test_random_gcode_holds_the_invariants(seed):
    result = fuzz.check(fuzz.generate(seed))
    assert not result["violations"], result["violations"][:10]


@pytest.mark.parametrize(
    "path", sorted(CASES_DIR.glob("*.json")), ids=lambda p: p.stem
)
def test_fuzz_counterexample_stays_fixed(path):
    result = fuzz.check(fuzz.load_case(path))
    assert not result["violations"], result["violations"][:10]
