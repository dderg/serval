"""Crawl prints: a short lead-in, then 120 s of slow extruding motion.

uv run python tools/bench/crawl_gcode.py    # writes every crawl print

Each print travels to the start, prints a 2.3 s lead-in at 100 mm/s that
ends in a reversal, then `M220 S<factor>` and CRAWL_S of extruding square
laps cut into PIECE_S pieces that never come to rest, `M220 S100` and a
lead-out. On the Pi the slow section's planning outruns the motion committed
ahead of it at the stream start (#983); a rest before it (`rest_ms`) does
not. The files are generated, never committed.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

BENCH = Path(__file__).resolve().parent
GENERATED = BENCH / "gcode" / "generated"
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


@dataclass(frozen=True)
class CrawlPrint:
    name: str
    factor_pct: int
    crawl_feed: int
    sides: int = SIDES
    rest_ms: int = 0

    @property
    def crawl_speed_mm_s(self) -> float:
        return self.crawl_feed / 60.0 * self.factor_pct / FULL_SPEED_PCT

    @property
    def side_mm(self) -> float:
        return self.crawl_speed_mm_s * CRAWL_S / self.sides

    @property
    def path(self) -> Path:
        return GENERATED / f"{self.name}.gcode"


CRAWL_PRINTS = (
    CrawlPrint("crawl_s1", 1, 6000),
    CrawlPrint("crawl_s5", 5, 3000),
    CrawlPrint("crawl_s10", 10, 6000),
    CrawlPrint("crawl_s20", 20, 6000, sides=12),
    CrawlPrint("crawl_s5_rest", 5, 3000, rest_ms=500),
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
        f"; {spec.name}: {CRAWL_S:.0f} s at {fmt(spec.crawl_speed_mm_s)} mm/s",
        "G90",
        "M83",
        f"G1 Z{LAYER_Z} F600",
        f"G1 X{fmt(START[0])} Y{fmt(START[1])} F{TRAVEL_FEED}",
    ]
    lead_in, (x, y) = strokes(*START)
    lines += lead_in
    slow, (x, y) = slow_section(spec, x, y)
    lines += slow
    lines.append(
        f"G1 X{fmt(START[0])} Y{fmt(y + STROKE_GAP_MM)} F{TRAVEL_FEED}"
    )
    lead_out, _ = strokes(START[0], y + STROKE_GAP_MM)
    lines += lead_out
    lines += ["G1 E-0.5 F2400", "G1 Z5 F600"]
    return "\n".join(lines) + "\n"


def ensure(spec: CrawlPrint) -> Path:
    text = render(spec)
    if not spec.path.exists() or spec.path.read_text() != text:
        spec.path.parent.mkdir(parents=True, exist_ok=True)
        spec.path.write_text(text)
    return spec.path


def ensure_all() -> None:
    for spec in CRAWL_PRINTS:
        ensure(spec)


def main() -> None:
    for spec in CRAWL_PRINTS:
        print(ensure(spec))


if __name__ == "__main__":
    main()
