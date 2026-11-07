from __future__ import annotations

import pathlib
import random
import tempfile

import pytest

from tools.motion_eval import fuzz, sweep

SEEDS_FILE = pathlib.Path(__file__).with_name("flush_seeds.txt")
CASES = ("circles", "voron_top_layers", "zigzag_infill", "arcs")


def seeds():
    return [
        int(line.split("#")[0])
        for line in SEEDS_FILE.read_text().splitlines()
        if line.split("#")[0].strip()
    ]


def corpus_case(seed):
    """A corpus case on owner.cfg with lazy lookahead flushes after random
    moves and M400 after random gcode lines, both drawn from seed."""
    rng = random.Random(seed)
    case = rng.choice(CASES)
    text = (sweep.CORPUS / (case + ".gcode")).read_text()
    lines = text.splitlines()
    move_lines = [
        i for i, line in enumerate(lines) if line.startswith(("G1", "G2", "G3"))
    ]
    return fuzz.Case(
        seed,
        "owner",
        text,
        tuple(sweep.case_start(text)),
        tuple(sorted(rng.sample(range(len(move_lines)), rng.randint(1, 30)))),
        tuple(sorted(rng.sample(move_lines, rng.randint(1, 8)))),
    )


@pytest.mark.parametrize("seed", seeds())
def test_junction_limits_do_not_depend_on_flushes(seed):
    case = corpus_case(seed)
    v = fuzz.Violations()
    with tempfile.TemporaryDirectory() as tmp:
        fuzz.check_flushes(v, case, tmp)
    assert not v, v[:10]
