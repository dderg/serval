"""Soak prints: long slicer-like prints stacked from the sim prints.

uv run python tools/bench/soak_gcode.py    # writes every soak print, prints them

voron_first_layers.gcode whole, then `body_repeats` times its layers above
the first followed by voron_dense_top_layers.gcode, each block's Z shifted
to start one LAYER_HEIGHT_MM above the block before: perimeters and dense
top infill alternating for `planned_print_s` at the printer's pace (what
gcode_replay_bench plans for the file on printer.cfg), from gcode the repo
already has. Each print is sized so its soak row fits one bench hold
(hold_plan.py): SESSION_PRINT runs twice in one klippy session, LONG_PRINT
once, the longest continuous print a hold leaves room for. The files are
generated, never committed; the same sources give the same bytes.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

BENCH = Path(__file__).resolve().parent
SIM_GCODE = BENCH.parents[1] / "tools" / "sim" / "gcode"
FIRST_LAYERS = SIM_GCODE / "voron_first_layers.gcode"
DENSE_TOP = SIM_GCODE / "voron_dense_top_layers.gcode"
GENERATED = BENCH / "gcode" / "generated"
LAYER_CHANGE = ";LAYER_CHANGE"
LAYER_HEIGHT_MM = 0.2
Z_WORD = re.compile(r"(?<=\s)Z(-?\d*\.?\d+)")


@dataclass(frozen=True)
class SoakPrint:
    name: str
    body_repeats: int
    planned_print_s: float

    @property
    def path(self) -> Path:
        return GENERATED / f"{self.name}.gcode"


SESSION_PRINT = SoakPrint("soak_session", 2, 427.8)
LONG_PRINT = SoakPrint("soak_long", 6, 997.8)
SOAK_PRINTS = (SESSION_PRINT, LONG_PRINT)


def code_of(line: str) -> str:
    return line.split(";", 1)[0]


def z_values(lines: list[str]) -> list[float]:
    return [
        float(match.group(1))
        for line in lines
        for match in Z_WORD.finditer(code_of(line))
    ]


def shifted(line: str, offset_mm: float) -> str:
    code, separator, comment = line.partition(";")
    moved = Z_WORD.sub(
        lambda match: f"Z{float(match.group(1)) + offset_mm:.3f}", code
    )
    return moved + separator + comment


def stacked(block: list[str], top_mm: float) -> tuple[list[str], float]:
    heights = z_values(block)
    offset_mm = top_mm + LAYER_HEIGHT_MM - min(heights)
    moved = [shifted(line, offset_mm) for line in block]
    return moved, max(heights) + offset_mm


def upper_layers(lines: list[str]) -> list[str]:
    changes = [
        i for i, line in enumerate(lines) if line.startswith(LAYER_CHANGE)
    ]
    return lines[changes[1] :]


def soak_lines(body_repeats: int) -> list[str]:
    first = FIRST_LAYERS.read_text().splitlines()
    blocks = (upper_layers(first), DENSE_TOP.read_text().splitlines())
    lines, top_mm = list(first), max(z_values(first))
    for _ in range(body_repeats):
        for block in blocks:
            moved, top_mm = stacked(block, top_mm)
            lines.extend(moved)
    return lines


def ensure(soak: SoakPrint) -> Path:
    text = "\n".join(soak_lines(soak.body_repeats)) + "\n"
    if not soak.path.exists() or soak.path.read_text() != text:
        soak.path.parent.mkdir(parents=True, exist_ok=True)
        soak.path.write_text(text)
    return soak.path


def ensure_all() -> list[Path]:
    return [ensure(soak) for soak in SOAK_PRINTS]


if __name__ == "__main__":
    print("\n".join(str(path) for path in ensure_all()))
