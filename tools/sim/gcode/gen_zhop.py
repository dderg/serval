#!/usr/bin/env python3
"""Write the z-hop variant of a sliced print, as a slicer's "lift Z on
retract" would emit it.

    python3 tools/sim/gcode/gen_zhop.py SOURCE.gcode DEST.gcode [--hop MM]

After every retract (a `G1` moving only E, backward) at a known height, Z
lifts by HOP: `G1 Z<z+hop>`. The travels that follow run at the lifted
height, a layer change's own Z becoming the height Z returns to. Before the
unretract (a `G1` moving only E, forward) Z comes back down: `G1 Z<z>`. A
pure-Z move inside the hop moves the height Z returns to instead. Anything
else inside a hop, an extruding move, lowers Z before it. A retract before
the first Z the print sets has no height to lift from and stays unlifted.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

WORD = re.compile(r"([A-Z])(-?[0-9]*\.?[0-9]+)")


def words(line: str) -> dict[str, float]:
    command = line.split(";", 1)[0].strip()
    if not command.startswith(("G0 ", "G1 ")):
        return {}
    return {letter: float(value) for letter, value in WORD.findall(command[2:])}


def number(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


def hop(lines: list[str], lift: float) -> tuple[list[str], int]:
    out: list[str] = []
    height: float | None = None
    returning: float | None = None
    hops = 0
    for line in lines:
        moved = words(line)
        axes = set(moved) - {"F"}
        if returning is None:
            out.append(line)
            if axes == {"E"} and moved["E"] < 0 and height is not None:
                out.append(f"G1 Z{number(height + lift)}\n")
                returning = height
                hops += 1
            if "Z" in moved:
                height = moved["Z"]
            continue
        if axes == {"Z"}:
            returning = moved["Z"]
            continue
        if axes and axes <= {"X", "Y", "Z"}:
            if "Z" in moved:
                returning = moved["Z"]
                line = re.sub(r"\s*Z-?[0-9]*\.?[0-9]+", "", line, count=1)
            out.append(line)
            continue
        if axes:
            out.append(f"G1 Z{number(returning)}\n")
            height, returning = returning, None
        out.append(line)
    return out, hops


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("dest", type=Path)
    parser.add_argument("--hop", type=float, default=0.4)
    args = parser.parse_args()
    lines, hops = hop(
        args.source.read_text().splitlines(keepends=True), args.hop
    )
    args.dest.write_text("".join(lines))
    print(f"{args.dest}: {hops} hops of {args.hop} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
