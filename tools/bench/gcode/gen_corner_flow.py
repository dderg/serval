"""Generate corner_flow.gcode: the extruding corners and short-segment
curves analyze.py --corner-flow measures nozzle flow through.

uv run python tools/bench/gcode/gen_corner_flow.py

Every case starts and ends at rest, so it is an epoch of its own: it
travels dry to its start, rests, extrudes its legs at one speed, rests.
The extrusion ratio is a 0.4 x 0.2 mm bead from 1.75 mm filament, as
tools/corner_flow.py lays.
"""

from __future__ import annotations

import math
from pathlib import Path

OUT = Path(__file__).resolve().with_name("corner_flow.gcode")
TRAVEL_FEED = 12000
REST_MS = 200
BEAD_WIDTH_MM = 0.4
BEAD_HEIGHT_MM = 0.2
FILAMENT_DIAMETER_MM = 1.75
RATIO = (
    BEAD_WIDTH_MM * BEAD_HEIGHT_MM / (math.pi * (FILAMENT_DIAMETER_MM / 2) ** 2)
)
LEG_MM = 40.0
CORNER_ANGLES_DEG = (45, 90, 135)
CORNER_SPEEDS_MM_S = (50, 100, 200, 300)
ARC_RADIUS_MM = 10.0
ARC_SEGMENT_MM = 0.5
ARC_SPEEDS_MM_S = (100, 200)
ZIGZAG_LEG_MM = 2.0
ZIGZAG_TURNS = 20
ZIGZAG_SPEEDS_MM_S = (50, 150)
CENTRE = (150.0, 150.0)


def extrude(
    x: float, y: float, from_xy: tuple[float, float], speed: float
) -> str:
    length = math.hypot(x - from_xy[0], y - from_xy[1])
    return f"G1 X{x:.4f} Y{y:.4f} E{RATIO * length:.6f} F{speed * 60:.0f}"


def case(path: list[tuple[float, float]], speed: float) -> list[str]:
    lines = [
        f"G1 X{path[0][0]:.4f} Y{path[0][1]:.4f} F{TRAVEL_FEED}",
        f"G4 P{REST_MS}",
    ]
    for previous, point in zip(path, path[1:]):
        lines.append(extrude(*point, previous, speed))
    lines.append(f"G4 P{REST_MS}")
    return lines


def corner(interior_deg: float) -> list[tuple[float, float]]:
    turn = math.radians(180.0 - interior_deg)
    start = (CENTRE[0] - LEG_MM, CENTRE[1] - LEG_MM / 2)
    apex = (start[0] + LEG_MM, start[1])
    end = (apex[0] + LEG_MM * math.cos(turn), apex[1] + LEG_MM * math.sin(turn))
    return [start, apex, end]


def arc() -> list[tuple[float, float]]:
    lead = (CENTRE[0] - LEG_MM, CENTRE[1] - ARC_RADIUS_MM)
    entry = (CENTRE[0], CENTRE[1] - ARC_RADIUS_MM)
    segments = round(math.pi * ARC_RADIUS_MM / ARC_SEGMENT_MM)
    points = [lead, entry]
    for index in range(1, segments + 1):
        angle = -math.pi / 2 + math.pi * index / segments
        points.append(
            (
                CENTRE[0] + ARC_RADIUS_MM * math.cos(angle),
                CENTRE[1] + ARC_RADIUS_MM * math.sin(angle),
            )
        )
    points.append((CENTRE[0] - LEG_MM, CENTRE[1] + ARC_RADIUS_MM))
    return points


def zigzag() -> list[tuple[float, float]]:
    rise = ZIGZAG_LEG_MM / math.sqrt(2.0)
    x, y = CENTRE[0] - LEG_MM, CENTRE[1]
    points = [(x, y), (x + LEG_MM / 2, y)]
    x += LEG_MM / 2
    for index in range(ZIGZAG_TURNS):
        x += rise
        y = CENTRE[1] + (rise if index % 2 == 0 else 0.0)
        points.append((x, y))
    points.append((x + LEG_MM / 2, y))
    return points


def corner_flow() -> list[str]:
    lines = ["G90", "M83"]
    for interior in CORNER_ANGLES_DEG:
        for speed in CORNER_SPEEDS_MM_S:
            lines += case(corner(interior), speed)
    for speed in ARC_SPEEDS_MM_S:
        lines += case(arc(), speed)
    for speed in ZIGZAG_SPEEDS_MM_S:
        lines += case(zigzag(), speed)
    lines.append("M400")
    return lines


if __name__ == "__main__":
    OUT.write_text("\n".join(corner_flow()) + "\n")
