"""The gcode's kinematic moves with their slicer tags, matched one to one
to the toolhead moves klippy planned (motion_eval capture)."""

from __future__ import annotations

import dataclasses
import math
import re

WORD = re.compile(r"([A-Z])([-+]?[0-9]*\.?[0-9]+)")
UNSUPPORTED = ("G2", "G3", "G91", "G10", "G11", "G20")


class GcodeError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class Move:
    start: tuple
    end: tuple
    extrusion: float
    kind: str | None
    layer: int | None
    line: int

    @property
    def length_xy(self):
        return math.hypot(
            self.end[0] - self.start[0], self.end[1] - self.start[1]
        )


def start_position(text):
    """X, Y, Z of the first G0/G1 that gives all three: the gcode must
    start there (SET_KINEMATIC_POSITION before it)."""
    for line in text.splitlines():
        code = line.split(";", 1)[0].strip().upper()
        if code.startswith(("G0 ", "G1 ")):
            words = dict(WORD.findall(code[2:]))
            if all(a in words for a in "XYZ"):
                return tuple(float(words[a]) for a in "XYZ")
            raise GcodeError(
                "the first G0/G1 must give X, Y and Z (the start position): %r"
                % (line,)
            )
    raise GcodeError("no G0/G1 move")


def parse(text):
    """Kinematic moves (any of X, Y, Z changes) in order. Absolute XYZ;
    E absolute (M82) or relative (M83)."""
    pos = None
    e = 0.0
    relative_e = False
    kind = None
    layer = None
    moves = []
    for number, raw in enumerate(text.splitlines(), start=1):
        if raw.startswith(";TYPE:"):
            kind = raw[len(";TYPE:") :].strip()
            continue
        if raw.startswith(";LAYER:"):
            layer = int(raw[len(";LAYER:") :])
            continue
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        head = code.split()[0]
        if head in UNSUPPORTED:
            raise GcodeError("line %d: %s is not supported" % (number, head))
        if head == "M82":
            relative_e = False
        elif head == "M83":
            relative_e = True
        elif head == "G92":
            words = dict(WORD.findall(code[3:]))
            if set(words) - {"E"}:
                raise GcodeError("line %d: G92 may only reset E" % (number,))
            e = float(words.get("E", 0.0))
        elif head in ("G0", "G1"):
            words = dict(WORD.findall(code[2:]))
            if pos is None:
                pos = tuple(float(words[a]) for a in "XYZ")
                if "E" in words:
                    e = (
                        float(words["E"])
                        if not relative_e
                        else e + float(words["E"])
                    )
                continue
            end = tuple(float(words.get(a, p)) for a, p in zip("XYZ", pos))
            de = 0.0
            if "E" in words:
                value = float(words["E"])
                de = value if relative_e else value - e
                e = e + value if relative_e else value
            if end != pos:
                moves.append(
                    Move(
                        pos, end, de, kind if de > 0.0 else None, layer, number
                    )
                )
            pos = end
    return moves


def match(moves, captured):
    """Pair each parsed move with klippy's toolhead move, checking start
    and end agree."""
    if len(moves) != len(captured):
        raise GcodeError(
            "%d kinematic moves in the gcode, %d planned by klippy"
            % (len(moves), len(captured))
        )
    tol = 1e-6
    for m, c in zip(moves, captured):
        start = (c["start_x"], c["start_y"], c["start_z"])
        if max(abs(a - b) for a, b in zip(start, m.start)) > tol:
            raise GcodeError(
                "line %d: planned move starts at %r, gcode at %r"
                % (m.line, start, m.start)
            )
    return list(zip(moves, captured))


def move_end_time(c):
    return c["print_time"] + c["accel_t"] + c["cruise_t"] + c["decel_t"]
