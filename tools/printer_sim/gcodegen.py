"""Deterministic test prints: a calibration-cube-like part and a
ringing-tower-like hollow square. Rectangular beads: E per mm =
width * layer_height / filament_area, so the nominal bead the deposition
model lays (width = flow / (speed * layer height)) is exactly `width`.

Every extruding move carries a `;TYPE:` region (WALL-OUTER, WALL-INNER,
FILL) and every layer a `;LAYER:` line, which deposition.py reads back.
Outer walls run counter-clockwise, so the outward normal of a segment
(dx, dy) is (dy, -dx).
"""

from __future__ import annotations

import dataclasses
import math

LAYER_HEIGHT = 0.2
WIDTH = 0.45
FILAMENT_DIAMETER = 1.75
TRAVEL_SPEED = 600.0
CENTRE = (175.0, 175.0)


@dataclasses.dataclass(frozen=True)
class Print:
    name: str
    size: float
    layers: int
    perimeters: int
    outer_speeds: tuple
    inner_speed: float
    fill_speed: float
    solid_bottom: int
    solid_top: int
    sparse_density: float

    def outer_speed(self, layer):
        band = layer * len(self.outer_speeds) // self.layers
        return self.outer_speeds[band]


CUBE = Print(
    name="cube",
    size=20.0,
    layers=30,
    perimeters=3,
    outer_speeds=(300.0,),
    inner_speed=450.0,
    fill_speed=450.0,
    solid_bottom=3,
    solid_top=4,
    sparse_density=0.25,
)
TOWER = Print(
    name="tower",
    size=40.0,
    layers=45,
    perimeters=2,
    outer_speeds=(300.0, 450.0, 600.0),
    inner_speed=300.0,
    fill_speed=0.0,
    solid_bottom=0,
    solid_top=0,
    sparse_density=0.0,
)
PRINTS = {p.name: p for p in (CUBE, TOWER)}


def e_per_mm():
    area = math.pi * (FILAMENT_DIAMETER / 2.0) ** 2
    return WIDTH * LAYER_HEIGHT / area


class Writer:
    def __init__(self):
        self.lines = []
        self.pos = None
        self.e = 0.0
        self.kind = None

    def travel(self, x, y, z=None):
        z_part = "" if z is None else " Z%.4f" % z
        self.lines.append(
            "G1 X%.4f Y%.4f%s F%.0f" % (x, y, z_part, TRAVEL_SPEED * 60.0)
        )
        self.pos = (x, y)

    def extrude(self, x, y, speed, kind):
        if kind != self.kind:
            self.lines.append(";TYPE:%s" % kind)
            self.kind = kind
        length = math.hypot(x - self.pos[0], y - self.pos[1])
        self.e += length * e_per_mm()
        self.lines.append(
            "G1 X%.4f Y%.4f E%.6f F%.0f" % (x, y, self.e, speed * 60.0)
        )
        self.pos = (x, y)

    def loop(self, half, speed, kind):
        cx, cy = CENTRE
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
        ]
        self.travel(*corners[0])
        for corner in corners[1:] + corners[:1]:
            self.extrude(*corner, speed, kind)


def outer_half(p: Print):
    """Half side of the outer wall's centre line."""
    return p.size / 2.0 - WIDTH / 2.0


def fill_lines(p: Print, layer, spacing):
    """Rectilinear fill inside the innermost perimeter: lines y = +-x + c,
    `spacing` apart, alternating direction (boustrophedon) and angle per
    layer. Lines shorter than two widths are left out."""
    cx, cy = CENTRE
    inner = outer_half(p) - p.perimeters * WIDTH + WIDTH / 2.0
    slope = 1.0 if layer % 2 == 0 else -1.0
    step = spacing * math.sqrt(2.0)
    count = int(4.0 * inner / step)
    lines = []
    for k in range(count + 1):
        c = -2.0 * inner + 0.5 * (4.0 * inner - count * step) + k * step
        if slope > 0.0:
            x0, x1 = max(-inner, -inner - c), min(inner, inner - c)
        else:
            x0, x1 = max(-inner, c - inner), min(inner, c + inner)
        if x1 - x0 < 2.0 * WIDTH / math.sqrt(2.0):
            continue
        a = (cx + x0, cy + slope * x0 + c)
        b = (cx + x1, cy + slope * x1 + c)
        lines.append((b, a) if len(lines) % 2 else (a, b))
    return lines


def generate(p: Print) -> str:
    w = Writer()
    w.lines += [
        "; printer_sim test print %s" % (p.name,),
        "M82",
        "G92 E0",
    ]
    cx, cy = CENTRE
    half = outer_half(p)
    w.travel(cx - half, cy - half, LAYER_HEIGHT)
    for layer in range(p.layers):
        z = LAYER_HEIGHT * (layer + 1)
        w.lines.append(";LAYER:%d" % (layer,))
        w.kind = None
        w.lines.append("G1 Z%.4f F%.0f" % (z, 15.0 * 60.0))
        for k in reversed(range(1, p.perimeters)):
            w.loop(half - k * WIDTH, p.inner_speed, "WALL-INNER")
        w.loop(half, p.outer_speed(layer), "WALL-OUTER")
        if p.fill_speed:
            solid = layer < p.solid_bottom or layer >= p.layers - p.solid_top
            spacing = WIDTH if solid else WIDTH / p.sparse_density
            for a, b in fill_lines(p, layer, spacing):
                w.travel(*a)
                w.extrude(*b, p.fill_speed, "FILL")
    return "\n".join(w.lines) + "\n"
