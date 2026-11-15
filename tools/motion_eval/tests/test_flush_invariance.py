from __future__ import annotations

import math
import pathlib
import random
import tempfile

import pytest

from klippy.extras import corner_deviation as cd
from tools.motion_eval import fuzz, sweep

SEEDS_FILE = pathlib.Path(__file__).with_name("flush_seeds.txt")
CASES = ("circles", "voron_top_layers", "zigzag_infill", "arcs")
MIN_COMPARED_SHARE = 0.5


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


def _traverse_time(length, va, vb, accel, vmax):
    peak = min(vmax, math.sqrt(0.5 * (va * va + vb * vb) + accel * length))
    return length / peak if peak > 0.0 else 0.0


def _window_span(raw):
    (state,) = raw.capture["shapers"]
    kx, ky = (cd.kernel_from_spec(fuzz._shaper_spec(state[a])) for a in "xy")
    return cd.BOUND_SEGMENTS * cd.bound_shape(kx, ky)["h"]


def limiter_inputs(raw, span):
    """add_move index -> (result, inputs) of every limiter call: the
    toolhead cap, the move before's max_start_v2 and the history window
    of earlier steps (index, tau, v2), newest first, as cd_junction_v2
    notes them; the window counts elapsed time from 0 instead of from the
    move before's duration, so it holds at least the C window's steps."""
    steps, out = [], {}
    for index, cap, v2, start_v2, length, accel, cruise_v2 in raw.capture[
        "junctions"
    ]:
        window, elapsed = [], 0.0
        for step in reversed(steps):
            if elapsed > span:
                break
            window.append(step)
            elapsed += step[1]
        out[int(index)] = (v2, (cap, start_v2, tuple(window)))
        tau = _traverse_time(
            length,
            math.sqrt(start_v2),
            math.sqrt(v2),
            accel,
            math.sqrt(cruise_v2),
        )
        steps.append((int(index), tau, v2))
    return out


@pytest.mark.parametrize("setting", ["axes", "steppers", "both"])
@pytest.mark.parametrize("seed", seeds())
def test_bounded_junction_limits_do_not_depend_on_lazy_flushes(seed, setting):
    """Every junction whose limiter inputs agree in the batch and the lazy
    run gets the same limit, and they agree at most junctions. A runout
    stop (a lazy flush left so little planned that Klipper stopped the
    toolhead: its own full flush, which changes the default planner's
    trapq too) changes the inputs after it: the next move starts from
    rest, max_start_v2 chains forward until a junction limit independent
    of it binds, and the window lacks the stop's step until it ages out.
    A lazy plan with a runout stop still holds the guarantee."""
    case = corpus_case(seed)
    caps = fuzz.bound_caps(case, "corexy")[setting]
    lazy_env = {
        "MOTION_EVAL_LAZY_FLUSH_AT": ",".join(map(str, case.lazy_flush_at))
    }
    with tempfile.TemporaryDirectory() as tmp:
        section = fuzz.bound_section(caps)
        batch = fuzz._run(case, tmp, section)
        lazy = fuzz._run(case, tmp, section, env=lazy_env)
    span = _window_span(batch)
    inputs, batch_inputs = (
        limiter_inputs(lazy, span),
        limiter_inputs(batch, span),
    )
    stops, batch_stops = fuzz._full_flushes(lazy), fuzz._full_flushes(batch)
    assert set(inputs) - set(batch_inputs) <= batch_stops
    assert set(batch_inputs) - set(inputs) <= stops
    common = sorted(set(inputs) & set(batch_inputs))
    compared = [i for i in common if inputs[i][1] == batch_inputs[i][1]]
    differ = [
        (i, inputs[i][0], batch_inputs[i][0])
        for i in compared
        if inputs[i][0] != batch_inputs[i][0]
    ]
    report = "compared %d of %d junctions (%.1f%%), runout stops %r" % (
        len(compared),
        len(common),
        100.0 * len(compared) / len(common),
        sorted(stops - batch_stops),
    )
    assert not differ, (report, differ[:10])
    assert len(compared) >= MIN_COMPARED_SHARE * len(common), report
    if stops - batch_stops:
        v = fuzz.Violations()
        fuzz._check_shaped(v, "bounded %s, lazy" % (setting,), lazy, caps)
        assert not v, v[:10]
