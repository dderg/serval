import math
import re

import crawl_gcode
import pytest
import run
from crawl_gcode import CRAWL_PRINTS, CRAWL_S

BED_MM = (0.0, 300.0)
WORD = re.compile(r"([XYEF])(-?\d+\.?\d*)")
SPECS = {spec.name: spec for spec in CRAWL_PRINTS}


def _slow_section(spec) -> tuple[list[str], list[str]]:
    lines = crawl_gcode.render(spec).splitlines()
    start = lines.index(f"M220 S{spec.factor_pct}")
    end = lines.index(f"M220 S{crawl_gcode.FULL_SPEED_PCT}")
    return lines[:start], lines[start + 1 : end]


def _words(line: str) -> dict[str, float]:
    return {axis: float(value) for axis, value in WORD.findall(line)}


@pytest.mark.parametrize("spec", CRAWL_PRINTS, ids=lambda spec: spec.name)
def test_the_slow_section_is_one_extruding_run_without_a_rest(spec):
    _, slow = _slow_section(spec)
    assert all(line.startswith("G1 X") for line in slow)
    moves = [_words(line) for line in slow]
    assert all(move["E"] > 0 and move["F"] == spec.crawl_feed for move in moves)
    headings = []
    for before, after in zip(moves, moves[1:]):
        headings.append((after["X"] - before["X"], after["Y"] - before["Y"]))
    for first, second in zip(headings, headings[1:]):
        dot = first[0] * second[0] + first[1] * second[1]
        assert dot >= -1e-9, "a reversal would bring the toolhead to rest"


@pytest.mark.parametrize("spec", CRAWL_PRINTS, ids=lambda spec: spec.name)
def test_the_slow_section_lasts_crawl_s_at_its_speed(spec):
    head, slow = _slow_section(spec)
    x, y = (
        _words(
            next(line for line in reversed(head) if line.startswith("G1 X"))
        )[k]
        for k in "XY"
    )
    path_mm = 0.0
    for move in map(_words, slow):
        path_mm += math.hypot(move["X"] - x, move["Y"] - y)
        x, y = move["X"], move["Y"]
    assert path_mm / spec.crawl_speed_mm_s == pytest.approx(CRAWL_S)


def test_only_the_rest_print_rests_before_its_slow_section():
    for spec in CRAWL_PRINTS:
        head, _ = _slow_section(spec)
        rested = head[-1] == f"G4 P{spec.rest_ms}"
        assert rested == (spec.name == "crawl_s5_rest")


@pytest.mark.parametrize("spec", CRAWL_PRINTS, ids=lambda spec: spec.name)
def test_every_print_stays_on_the_bed_and_runs_on_the_bench(spec):
    text = crawl_gcode.render(spec)
    for move in map(_words, text.splitlines()):
        for axis in "XY":
            if axis in move:
                assert BED_MM[0] <= move[axis] <= BED_MM[1]
    run.sanitize(text)


def test_the_slow_speeds_span_the_crawl_speed():
    speeds = sorted(spec.crawl_speed_mm_s for spec in CRAWL_PRINTS)
    assert speeds[0] == 1.0
    assert speeds[-1] == 20.0
    assert SPECS["crawl_s5"].crawl_speed_mm_s == 2.5
