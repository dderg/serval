"""Crawl prints: a lead-in, then a long slow extruding section.

uv run python tools/bench/crawl_gcode.py    # writes every crawl print

Each print travels to the start, prints `lead_strokes` lead-in strokes at
100 mm/s (one stroke group is 2.3 s and ends in a reversal), then
`M220 S<factor>` and `crawl_s` of extruding square laps cut into PIECE_S
pieces that never come to rest, `M220 S100` and `lead_out_strokes` strokes.
On the Pi the slow section's planning outruns the motion committed ahead of
it at the stream start (#983); a rest before it (`rest_ms`) does not. The
mid-print prints put the slow section between 20 s of printing on each
side. A slowed print takes an ordinary sliced file and applies
`M220 S<factor>` to `slow_s` of it, from line `from_line`. The files are
generated, never committed.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

BENCH = Path(__file__).resolve().parent
GENERATED = BENCH / "gcode" / "generated"
FIRST_LAYERS = BENCH.parents[1] / "tools/sim/gcode/voron_first_layers.gcode"
CRAWL_S = 120.0
PIECE_S = 2.0
SIDES = 6
PRINT_FEED = 6000
TRAVEL_FEED = 12000
E_PER_MM = 0.0374
LAYER_Z = 0.2
START = (40.0, 50.0)
STROKE_MM = 100.0
STROKE_GAP_MM = 10.0
FULL_SPEED_PCT = 100
SQUARE_HEADINGS = ((1, 0), (0, 1), (-1, 0), (0, -1))
MID_PRINT_STROKES = 9
MID_PRINT_CRAWL_S = 300.0
AXIS_WORD = re.compile(r"([XYZEF])(-?\d*\.?\d+)")


@dataclass(frozen=True)
class CrawlPrint:
    name: str
    factor_pct: int
    crawl_feed: int
    sides: int = SIDES
    rest_ms: int = 0
    crawl_s: float = CRAWL_S
    lead_strokes: int = 1
    lead_out_strokes: int = 1

    @property
    def crawl_speed_mm_s(self) -> float:
        return self.crawl_feed / 60.0 * self.factor_pct / FULL_SPEED_PCT

    @property
    def side_mm(self) -> float:
        return self.crawl_speed_mm_s * self.crawl_s / self.sides

    @property
    def path(self) -> Path:
        return GENERATED / f"{self.name}.gcode"

    def text(self) -> str:
        return render(self)


@dataclass(frozen=True)
class SlowedPrint:
    name: str
    source: Path
    factor_pct: int
    from_line: int
    slow_s: float

    @property
    def path(self) -> Path:
        return GENERATED / f"{self.name}.gcode"

    def text(self) -> str:
        return render_slowed(self)


CRAWL_PRINTS = (
    CrawlPrint("crawl_s1", 1, 6000),
    CrawlPrint("crawl_s5", 5, 3000),
    CrawlPrint("crawl_s5_fast", 5, 6000),
    CrawlPrint("crawl_s10", 10, 6000),
    CrawlPrint("crawl_s20", 20, 6000, sides=12),
    CrawlPrint("crawl_s5_20s", 5, 3000, crawl_s=20.0),
    CrawlPrint("crawl_s5_lead", 5, 3000, lead_strokes=MID_PRINT_STROKES),
    CrawlPrint("crawl_s5_rest", 5, 3000, rest_ms=500),
)
MID_PRINTS = (
    CrawlPrint(
        "mid_crawl_s5_300s",
        5,
        3000,
        sides=15,
        crawl_s=MID_PRINT_CRAWL_S,
        lead_strokes=MID_PRINT_STROKES,
        lead_out_strokes=MID_PRINT_STROKES,
    ),
    CrawlPrint(
        "mid_crawl_s1_300s",
        1,
        6000,
        crawl_s=MID_PRINT_CRAWL_S,
        lead_strokes=MID_PRINT_STROKES,
        lead_out_strokes=MID_PRINT_STROKES,
    ),
    SlowedPrint("mid_first_layers_m220_s5", FIRST_LAYERS, 5, 1000, CRAWL_S),
)


def fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".")


def extrude(x: float, y: float, mm: float, feed: int) -> str:
    return f"G1 X{fmt(x)} Y{fmt(y)} E{fmt(mm * E_PER_MM)} F{feed}"


def strokes(x0: float, y0: float) -> tuple[list[str], tuple[float, float]]:
    x1 = x0 + STROKE_MM
    y1 = y0 + STROKE_GAP_MM
    lines = [
        extrude(x1, y0, STROKE_MM, PRINT_FEED),
        extrude(x1, y1, STROKE_GAP_MM, PRINT_FEED),
        extrude(x0, y1, STROKE_MM, PRINT_FEED),
    ]
    return lines, (x0, y1)


def stroke_run(
    count: int, x: float, y: float
) -> tuple[list[str], tuple[float, float]]:
    lines = []
    for _ in range(count):
        run, (x, y) = strokes(x, y)
        lines += run
    return lines, (x, y)


def slow_section(
    spec: CrawlPrint, x: float, y: float
) -> tuple[list[str], tuple[float, float]]:
    pieces = round(spec.side_mm / (spec.crawl_speed_mm_s * PIECE_S))
    piece_mm = spec.side_mm / pieces
    lines = [f"G4 P{spec.rest_ms}"] if spec.rest_ms else []
    lines.append(f"M220 S{spec.factor_pct}")
    for side in range(spec.sides):
        dx, dy = SQUARE_HEADINGS[side % len(SQUARE_HEADINGS)]
        for _ in range(pieces):
            x += dx * piece_mm
            y += dy * piece_mm
            lines.append(extrude(x, y, piece_mm, spec.crawl_feed))
    lines.append(f"M220 S{FULL_SPEED_PCT}")
    return lines, (x, y)


def render(spec: CrawlPrint) -> str:
    lines = [
        f"; {spec.name}: {spec.crawl_s:.0f} s at {fmt(spec.crawl_speed_mm_s)} mm/s",
        "G90",
        "M83",
        f"G1 Z{LAYER_Z} F600",
        f"G1 X{fmt(START[0])} Y{fmt(START[1])} F{TRAVEL_FEED}",
    ]
    lead_in, (x, y) = stroke_run(spec.lead_strokes, *START)
    lines += lead_in
    slow, (x, y) = slow_section(spec, x, y)
    lines += slow
    lines.append(
        f"G1 X{fmt(START[0])} Y{fmt(y + STROKE_GAP_MM)} F{TRAVEL_FEED}"
    )
    lead_out, _ = stroke_run(spec.lead_out_strokes, START[0], y + STROKE_GAP_MM)
    lines += lead_out
    lines += ["G1 E-0.5 F2400", "G1 Z5 F600"]
    return "\n".join(lines) + "\n"


def slowed_move_s(
    words: dict[str, float], at: dict[str, float], factor_pct: int
) -> float:
    travel_mm = math.dist(
        [words.get(axis, at[axis]) for axis in "XYZ"],
        [at[axis] for axis in "XYZ"],
    )
    mm = travel_mm or abs(words.get("E", 0.0))
    return mm / (at["F"] / 60.0 * factor_pct / FULL_SPEED_PCT)


def render_slowed(spec: SlowedPrint) -> str:
    source = spec.source.read_text().splitlines()
    at = {"X": 0.0, "Y": 0.0, "Z": 0.0, "F": float(TRAVEL_FEED)}
    lines = [f"; {spec.name}: {spec.source.name} at {spec.factor_pct} %"]
    slowed_s = 0.0
    for number, line in enumerate(source):
        if number == spec.from_line:
            lines.append(f"M220 S{spec.factor_pct}")
        lines.append(line)
        if not line.startswith("G1"):
            continue
        words = {axis: float(v) for axis, v in AXIS_WORD.findall(line)}
        at["F"] = words.get("F", at["F"])
        if spec.from_line <= number and slowed_s < spec.slow_s:
            slowed_s += slowed_move_s(words, at, spec.factor_pct)
            if slowed_s >= spec.slow_s:
                lines.append(f"M220 S{FULL_SPEED_PCT}")
        at.update({axis: words[axis] for axis in "XYZ" if axis in words})
    if slowed_s < spec.slow_s:
        raise ValueError(
            f"{spec.source} after line {spec.from_line} holds "
            f"{slowed_s:.1f} s at {spec.factor_pct} %, not {spec.slow_s} s"
        )
    return "\n".join(lines) + "\n"


def ensure(spec: CrawlPrint | SlowedPrint) -> Path:
    text = spec.text()
    if not spec.path.exists() or spec.path.read_text() != text:
        spec.path.parent.mkdir(parents=True, exist_ok=True)
        spec.path.write_text(text)
    return spec.path


def ensure_all() -> None:
    for spec in (*CRAWL_PRINTS, *MID_PRINTS):
        ensure(spec)


def main() -> None:
    for spec in (*CRAWL_PRINTS, *MID_PRINTS):
        print(ensure(spec))


if __name__ == "__main__":
    main()
