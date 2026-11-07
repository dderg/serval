from __future__ import annotations

import pathlib

import pytest

from tools.motion_eval import fuzz

SEEDS_FILE = pathlib.Path(__file__).with_name("fuzz_seeds.txt")


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
