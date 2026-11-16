"""The soak print: a long slicer-like print stacked from the sim prints.

uv run python tools/bench/soak_gcode.py    # writes PATH and prints it

voron_first_layers.gcode whole, then BODY_REPEATS times its layers above
the first followed by voron_dense_top_layers.gcode, each block's Z shifted
to start one LAYER_HEIGHT_MM above the block before: perimeters and dense
top infill alternating for PLANNED_PRINT_S at the printer's pace (what
gcode_replay_bench plans for the file on printer.cfg), from gcode the repo
already has. The file is generated, never committed (2.5 MB); the same
sources give the same bytes.
"""

from __future__ import annotations

import re
from pathlib import Path

BENCH = Path(__file__).resolve().parent
SIM_GCODE = BENCH.parents[1] / "tools" / "sim" / "gcode"
FIRST_LAYERS = SIM_GCODE / "voron_first_layers.gcode"
DENSE_TOP = SIM_GCODE / "voron_dense_top_layers.gcode"
PATH = BENCH / "gcode" / "generated" / "soak.gcode"
LAYER_CHANGE = ";LAYER_CHANGE"
LAYER_HEIGHT_MM = 0.2
BODY_REPEATS = 8
PLANNED_PRINT_S = 1283.0
Z_WORD = re.compile(r"(?<=\s)Z(-?\d*\.?\d+)")


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


def soak_lines() -> list[str]:
    first = FIRST_LAYERS.read_text().splitlines()
    blocks = (upper_layers(first), DENSE_TOP.read_text().splitlines())
    lines, top_mm = list(first), max(z_values(first))
    for _ in range(BODY_REPEATS):
        for block in blocks:
            moved, top_mm = stacked(block, top_mm)
            lines.extend(moved)
    return lines


def ensure() -> Path:
    text = "\n".join(soak_lines()) + "\n"
    if not PATH.exists() or PATH.read_text() != text:
        PATH.parent.mkdir(parents=True, exist_ok=True)
        PATH.write_text(text)
    return PATH


if __name__ == "__main__":
    print(ensure())
