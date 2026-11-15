from __future__ import annotations

import math
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


def _limits_before(raw, end):
    return {i: lim for i, lim in fuzz._junctions(raw).items() if i < end}


@pytest.mark.parametrize("setting", ["axes", "steppers", "both"])
@pytest.mark.parametrize("seed", seeds())
def test_bounded_junction_limits_do_not_depend_on_lazy_flushes(seed, setting):
    """Up to the first runout stop: a lazy flush can leave so little
    planned that Klipper stops the toolhead (its own full flush; the
    default planner's trapq changes with the same flushes), and the move
    after that stop starts from rest, a different plan."""
    case = corpus_case(seed)
    lazy_env = {
        "MOTION_EVAL_LAZY_FLUSH_AT": ",".join(map(str, case.lazy_flush_at))
    }
    with tempfile.TemporaryDirectory() as tmp:
        section = fuzz.bound_section(fuzz.bound_caps(case, "corexy")[setting])
        batch = fuzz._run(case, tmp, section)
        lazy = fuzz._run(case, tmp, section, env=lazy_env)
    runout = fuzz._full_flushes(lazy) - fuzz._full_flushes(batch)
    end = min(runout, default=math.inf)
    limits, batch_limits = _limits_before(lazy, end), _limits_before(batch, end)
    assert limits.keys() == batch_limits.keys()
    differ = [
        (i, limits[i], batch_limits[i])
        for i in sorted(limits)
        if not fuzz._same_own_limit(limits[i], batch_limits[i])
    ]
    assert not differ, differ[:10]
