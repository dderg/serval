"""Regenerate the motion_eval corpus.

    uv run python -m tools.motion_eval.corpus.build_corpus [--voron-dir DIR]

Synthetic stress cases are generated here. Slicer slices are cut from the
voron_*.gcode sim prints (serval tools/sim/gcode) and reduced to motion:
only G0-G3, G90-G92, M82/M83 survive, so the configured accel, velocity and
corner settings are the ones under test (slicer M204/SET_VELOCITY_LIMIT are
dropped). Every file starts with a `; motion_eval start=X,Y,Z` header that
the sweep passes to the harness as the initial toolhead position.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import re

CORPUS_DIR = pathlib.Path(__file__).resolve().parent
KEEP_COMMANDS = {"G0", "G1", "G2", "G3", "G90", "G91", "G92", "M82", "M83"}
E_PER_MM = 0.0333
VORON_SLICES = (
    ("voron_first_layers.gcode", "voron_first_layer_perimeters", 0, 1400),
    ("voron_dense_top_layers.gcode", "voron_top_layers", 0, 1400),
)
CORNER_TURNS_DEG = (45, 90, 135, 170)
CORNER_SPEEDS_MM_S = (50, 150, 300)
CORNER_LEG_MM = 20.0
CORNER_LEGS = 8
CIRCLE_RADII_MM = (1.5, 3.0, 6.0, 12.0)
CIRCLE_CHORD_ERROR_MM = 0.0125
CIRCLE_SPEED_MM_S = 200
ZIGZAG_LINE_MM = (2.0, 4.0, 8.0)
ZIGZAG_SPACING_MM = 0.45
ZIGZAG_LINES = 30
ZIGZAG_SPEED_MM_S = 300
ARC_RADII_MM = (2.0, 5.0)
ARC_SPEED_MM_S = 150
SYNTHETIC_Z = 0.2
WORD = re.compile(r"([A-Z])([-+]?[0-9]*\.?[0-9]*)")


def header(start):
    return "; motion_eval start=%.4f,%.4f,%.4f\nG90\nM83\n" % start


def g1(x, y, feed, extrude=True, prev=None):
    e = ""
    if extrude and prev is not None:
        e = " E%.5f" % (E_PER_MM * math.dist(prev, (x, y)))
    return "G1 X%.4f Y%.4f%s F%d\n" % (x, y, e, feed * 60)


def corners(turn_deg):
    start = (60.0, 100.0)
    out = []
    for speed in CORNER_SPEEDS_MM_S:
        out.append(g1(*start, speed, extrude=False))
        pos, heading = start, 0.0
        for leg in range(CORNER_LEGS):
            nxt = (
                pos[0] + CORNER_LEG_MM * math.cos(heading),
                pos[1] + CORNER_LEG_MM * math.sin(heading),
            )
            out.append(g1(*nxt, speed, prev=pos))
            pos = nxt
            heading += math.radians(turn_deg) * (1 if leg % 2 == 0 else -1)
    return header(start + (SYNTHETIC_Z,)) + "".join(out)


def circles():
    out, start = [], (150.0, 150.0)
    pos = start
    for radius in CIRCLE_RADII_MM:
        center = (start[0] + 40.0 * CIRCLE_RADII_MM.index(radius), start[1])
        step = 2.0 * math.acos(1.0 - CIRCLE_CHORD_ERROR_MM / radius)
        n = max(8, math.ceil(2.0 * math.pi / step))
        first = (center[0] + radius, center[1])
        out.append(g1(*first, CIRCLE_SPEED_MM_S, extrude=False))
        pos = first
        for k in range(1, 3 * n + 1):
            a = 2.0 * math.pi * k / n
            nxt = (
                center[0] + radius * math.cos(a),
                center[1] + radius * math.sin(a),
            )
            out.append(g1(*nxt, CIRCLE_SPEED_MM_S, prev=pos))
            pos = nxt
    return header(start + (SYNTHETIC_Z,)) + "".join(out)


def zigzag():
    out, start = [], (100.0, 60.0)
    pos = start
    for block, line in enumerate(ZIGZAG_LINE_MM):
        x0 = start[0] + 30.0 * block
        y = start[1]
        first = (x0, y)
        out.append(g1(*first, ZIGZAG_SPEED_MM_S, extrude=False))
        pos = first
        for k in range(ZIGZAG_LINES):
            x_end = x0 + (line if k % 2 == 0 else 0.0)
            nxt = (x_end, y)
            out.append(g1(*nxt, ZIGZAG_SPEED_MM_S, prev=pos))
            pos = nxt
            y += ZIGZAG_SPACING_MM
            nxt = (x_end, y)
            out.append(g1(*nxt, ZIGZAG_SPEED_MM_S, prev=pos))
            pos = nxt
    return header(start + (SYNTHETIC_Z,)) + "".join(out)


def arcs():
    out, start = [], (80.0, 200.0)
    x, y = start
    for radius in ARC_RADII_MM:
        for k in range(6):
            cmd = "G2" if k % 2 == 0 else "G3"
            out.append(
                "%s X%.4f Y%.4f I%.4f J0 E%.5f F%d\n"
                % (
                    cmd,
                    x + 2 * radius,
                    y,
                    radius,
                    E_PER_MM * math.pi * radius,
                    ARC_SPEED_MM_S * 60,
                )
            )
            x += 2 * radius
    return header(start + (SYNTHETIC_Z,)) + "".join(out)


def sanitize(lines, first, count):
    kept = []
    for raw in lines:
        line = raw.split(";")[0].strip()
        if not line:
            continue
        if line.split()[0] in KEEP_COMMANDS:
            kept.append(line)
    kept = kept[first : first + count]
    start = {}
    for line in kept:
        if line.split()[0] not in ("G0", "G1"):
            continue
        for axis, value in WORD.findall(line):
            if axis in "XYZ" and axis not in start and value:
                start[axis] = float(value)
    missing = set("XYZ") - set(start)
    if missing:
        raise ValueError("slice never sets %s" % sorted(missing))
    return header((start["X"], start["Y"], start["Z"])) + "\n".join(kept) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--voron-dir",
        type=pathlib.Path,
        default=pathlib.Path.home() / "Developer/serval/tools/sim/gcode",
    )
    args = ap.parse_args()
    cases = {"circles": circles(), "zigzag_infill": zigzag(), "arcs": arcs()}
    for turn in CORNER_TURNS_DEG:
        cases["corners_turn%d" % turn] = corners(turn)
    for source, name, first, count in VORON_SLICES:
        lines = (args.voron_dir / source).read_text().splitlines()
        cases[name] = sanitize(lines, first, count)
    for name, text in cases.items():
        (CORPUS_DIR / (name + ".gcode")).write_text(text)
        print(name, text.count("\n"), "lines")


if __name__ == "__main__":
    main()
