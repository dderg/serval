"""Print programs composed from segment generators, and the model that
says where each line leaves the toolhead.

Every motion segment opens with an absolute travel to its own start, so
any subset of segments is a valid program: the minimizer drops segments
without fixing up the ones after them."""

from __future__ import annotations

import dataclasses
import enum
import functools
import math
import pathlib
import random
import re
from typing import Optional

from tools.sim.fuzzing.printer import (
    DONE_PIN,
    MARK_PIN,
    MESH_PROFILE,
    MeshKind,
    PrinterSpec,
)

GCODE_DIR = pathlib.Path(__file__).resolve().parents[1] / "gcode"
SLICE_SOURCES = ("voron_first_layers.gcode", "voron_dense_top_layers.gcode")
PROBE = ";fuzz_probe"
PROBE_HOLD_MS = 1500
EXTRUDE_PER_MM = 0.0374
LAYER_Z = 0.2
REGION_MARGIN_MM = 40.0
MAX_TRAVEL_FEED = 60000
Z_FEED = 600
RETRACT_MM = 0.8
RETRACT_FEED = 2400
SLICE_LINES = (80, 400)
SLICE_DROPPED = ("M73",)
PRELUDE = ("G90", "M83")
LIMIT_RESET = "RESET_VELOCITY_LIMIT"


class Kind(enum.Enum):
    SLICE = "slice"
    PERIMETERS = "perimeters"
    INFILL = "infill"
    STROKES = "strokes"
    TINY = "tiny"
    ZIGZAG = "zigzag"
    TRAVEL = "travel"
    Z_MOVES = "z_moves"
    LIMITS = "limits"
    EXTRUDE_ONLY = "extrude_only"
    RETRACT = "retract"
    REST = "rest"
    G92 = "g92"
    RELATIVE = "relative"
    ABSOLUTE_E = "absolute_e"
    LATTICE = "lattice"
    MISC = "misc"
    MESH = "mesh"


STATE_CHANGING = (Kind.LIMITS, Kind.MESH)


@dataclasses.dataclass(frozen=True)
class Segment:
    kind: Kind
    lines: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class Program:
    name: str
    segments: tuple[Segment, ...]
    end_print: bool

    def kinds(self) -> set:
        return {segment.kind for segment in self.segments}


@dataclasses.dataclass(frozen=True)
class Region:
    low: float
    high: float

    def point(
        self, rng: random.Random, width=0.0, height=0.0
    ) -> tuple[float, float]:
        """A corner from which a `width` x `height` box stays inside."""
        return (
            round(rng.uniform(self.low, self.high - width), 3),
            round(rng.uniform(self.low, self.high - height), 3),
        )


def _fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text not in ("-0", "") else "0"


def _move(x=None, y=None, z=None, e=None, f=None, verb="G1") -> str:
    words = [verb]
    for letter, value in (("X", x), ("Y", y), ("Z", z), ("E", e), ("F", f)):
        if value is not None:
            words.append(f"{letter}{_fmt(value)}")
    return " ".join(words)


@dataclasses.dataclass(frozen=True)
class Context:
    rng: random.Random
    region: Region
    max_velocity: float

    def print_feed(self) -> float:
        return 60 * round(self.rng.uniform(20.0, min(400.0, self.max_velocity)))

    def travel_feed(self) -> float:
        return 60 * round(self.rng.uniform(100.0, self.max_velocity))

    def layer_z(self) -> float:
        return round(LAYER_Z * self.rng.randint(1, 10), 2)


def _travel_to(ctx: Context, x: float, y: float) -> list[str]:
    return [
        "G90",
        _move(z=ctx.layer_z(), f=Z_FEED),
        _move(x, y, f=ctx.travel_feed()),
    ]


def _extruding_path(ctx: Context, points, feed) -> list[str]:
    lines = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        e = round(math.hypot(x1 - x0, y1 - y0) * EXTRUDE_PER_MM, 5)
        lines.append(_move(x1, y1, e=e, f=feed))
    return lines


def _clamp(ctx: Context, value: float) -> float:
    return round(min(max(value, ctx.region.low), ctx.region.high), 3)


def _perimeters(ctx: Context) -> list[str]:
    half = ctx.rng.uniform(3.0, 30.0)
    cx, cy = (v + half for v in ctx.region.point(ctx.rng, 2 * half, 2 * half))
    chamfer = ctx.rng.uniform(0.0, half / 3)
    loops = ctx.rng.randint(1, 4)
    feed = ctx.print_feed()
    lines = []
    for loop in range(loops):
        h = half - 0.45 * loop
        corners = [
            (cx - h + chamfer, cy - h),
            (cx + h - chamfer, cy - h),
            (cx + h, cy - h + chamfer),
            (cx + h, cy + h - chamfer),
            (cx + h - chamfer, cy + h),
            (cx - h + chamfer, cy + h),
            (cx - h, cy + h - chamfer),
            (cx - h, cy - h + chamfer),
        ]
        points = [(_clamp(ctx, x), _clamp(ctx, y)) for x, y in corners]
        points.append(points[0])
        if not lines:
            lines += _travel_to(ctx, *points[0])
        else:
            lines.append(_move(*points[0], f=ctx.travel_feed()))
        lines += _extruding_path(ctx, points, feed)
    return lines


def _infill(ctx: Context) -> list[str]:
    width = ctx.rng.uniform(5.0, 60.0)
    rows = ctx.rng.randint(4, 40)
    spacing = ctx.rng.choice((0.45, 0.5, 1.0, 2.0))
    x0, y0 = ctx.region.point(ctx.rng, width, rows * spacing)
    feed = ctx.print_feed()
    points = []
    for row in range(rows):
        y = _clamp(ctx, y0 + row * spacing)
        xs = (x0, x0 + width) if row % 2 == 0 else (x0 + width, x0)
        points += [(_clamp(ctx, xs[0]), y), (_clamp(ctx, xs[1]), y)]
    return _travel_to(ctx, *points[0]) + _extruding_path(ctx, points, feed)


def _strokes(ctx: Context) -> list[str]:
    count = ctx.rng.randint(2, 8)
    points = [ctx.region.point(ctx.rng) for _ in range(count + 1)]
    feed = 60 * round(ctx.rng.uniform(100.0, ctx.max_velocity))
    extrude = ctx.rng.random() < 0.5
    tail = (
        _extruding_path(ctx, points, feed)
        if extrude
        else [_move(x, y, f=feed) for x, y in points[1:]]
    )
    return _travel_to(ctx, *points[0]) + tail


def _tiny(ctx: Context) -> list[str]:
    radius = ctx.rng.uniform(0.5, 15.0)
    cx, cy = (
        v + radius for v in ctx.region.point(ctx.rng, 2 * radius, 2 * radius)
    )
    step = ctx.rng.uniform(0.02, 0.3)
    count = min(400, max(8, int(2 * math.pi * radius / step)))
    sweep = ctx.rng.uniform(0.5, 2.0) * 2 * math.pi
    points = [
        (
            _clamp(ctx, cx + radius * math.cos(sweep * i / count)),
            _clamp(ctx, cy + radius * math.sin(sweep * i / count)),
        )
        for i in range(count + 1)
    ]
    return _travel_to(ctx, *points[0]) + _extruding_path(
        ctx, points, ctx.print_feed()
    )


def _zigzag(ctx: Context) -> list[str]:
    length = ctx.rng.uniform(5.0, 80.0)
    moves = ctx.rng.randint(4, 60)
    x0, y0 = ctx.region.point(ctx.rng, length, moves * 0.2)
    feed = ctx.print_feed()
    points = [
        (
            _clamp(ctx, x0 + (length if i % 2 else 0.0)),
            _clamp(ctx, y0 + (i // 2) * 0.4),
        )
        for i in range(moves + 1)
    ]
    return _travel_to(ctx, *points[0]) + _extruding_path(ctx, points, feed)


def _travel(ctx: Context) -> list[str]:
    points = [ctx.region.point(ctx.rng) for _ in range(ctx.rng.randint(2, 10))]
    lines = ["G90"]
    for x, y in points:
        lines.append(_move(x, y, f=min(MAX_TRAVEL_FEED, ctx.travel_feed())))
    return lines


def _z_moves(ctx: Context) -> list[str]:
    lines = ["G90"]
    for _ in range(ctx.rng.randint(1, 4)):
        z = round(ctx.rng.uniform(LAYER_Z, 15.0), 3)
        lines.append(_move(z=z, f=60 * round(ctx.rng.uniform(2.0, 25.0))))
    if ctx.rng.random() < 0.5:
        x, y = ctx.region.point(ctx.rng)
        lines.append(_move(x, y, z=ctx.layer_z(), f=ctx.travel_feed()))
    return lines


def _limits(ctx: Context) -> list[str]:
    choices = (
        lambda: f"M204 S{round(ctx.rng.uniform(500, 40000))}",
        lambda: (
            f"SET_VELOCITY_LIMIT VELOCITY={round(ctx.rng.uniform(50, ctx.max_velocity))}"
            f" ACCEL={round(ctx.rng.uniform(500, 40000))}"
        ),
        lambda: (
            f"SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={round(ctx.rng.uniform(1, 20), 1)}"
        ),
        lambda: (
            f"SET_VELOCITY_LIMIT CORNER_DEVIATION={round(ctx.rng.uniform(0.01, 0.5), 3)}"
        ),
        lambda: LIMIT_RESET,
    )
    return [ctx.rng.choice(choices)() for _ in range(ctx.rng.randint(1, 2))]


def _extrude_only(ctx: Context) -> list[str]:
    lines = []
    for _ in range(ctx.rng.randint(1, 3)):
        e = round(ctx.rng.uniform(-3.0, 5.0), 3)
        lines.append(_move(e=e, f=60 * round(ctx.rng.uniform(1.0, 40.0))))
    return lines


def _retract(ctx: Context) -> list[str]:
    x, y = ctx.region.point(ctx.rng)
    return [
        _move(e=-RETRACT_MM, f=RETRACT_FEED),
        "G90",
        _move(x, y, f=ctx.travel_feed()),
        _move(e=RETRACT_MM, f=RETRACT_FEED),
    ]


def _rest(ctx: Context) -> list[str]:
    if ctx.rng.random() < 0.5:
        return [f"G4 P{ctx.rng.choice((0, 1, 10, 100, 500, 1500))}"]
    return ["M400"]


def _g92(ctx: Context) -> list[str]:
    return ["G92 E0"]


def _relative(ctx: Context) -> list[str]:
    x, y = ctx.region.point(ctx.rng)
    lines = _travel_to(ctx, x, y) + ["G91"]
    feed = ctx.print_feed()
    for _ in range(ctx.rng.randint(2, 12)):
        dx = round(_clamp(ctx, x + ctx.rng.uniform(-10.0, 10.0)) - x, 3)
        dy = round(_clamp(ctx, y + ctx.rng.uniform(-10.0, 10.0)) - y, 3)
        x, y = x + dx, y + dy
        e = round(math.hypot(dx, dy) * EXTRUDE_PER_MM, 5)
        lines.append(_move(dx, dy, e=e, f=feed))
    return lines + ["G90"]


def _absolute_e(ctx: Context) -> list[str]:
    x, y = ctx.region.point(ctx.rng)
    lines = _travel_to(ctx, x, y) + ["G92 E0", "M82"]
    e = 0.0
    feed = ctx.print_feed()
    for _ in range(ctx.rng.randint(2, 10)):
        nx, ny = ctx.region.point(ctx.rng)
        e = round(e + math.hypot(nx - x, ny - y) * EXTRUDE_PER_MM, 5)
        lines.append(_move(nx, ny, e=e, f=feed))
        x, y = nx, ny
    return lines + ["M83"]


LATTICE_STEP_MM = 0.0125


def _lattice_coordinate(ctx: Context, on_lattice: bool) -> float:
    value = ctx.rng.uniform(ctx.region.low, ctx.region.high)
    if on_lattice:
        return round(round(value / LATTICE_STEP_MM) * LATTICE_STEP_MM, 4)
    return round(value, ctx.rng.choice((3, 4, 6)))


def _lattice(ctx: Context) -> list[str]:
    lines = ["G90"]
    for _ in range(ctx.rng.randint(1, 3)):
        on_lattice = ctx.rng.random() < 0.6
        x = _lattice_coordinate(ctx, on_lattice)
        y = _lattice_coordinate(ctx, on_lattice)
        lines += [_move(x, y, f=ctx.travel_feed()), PROBE]
    return lines


def _misc(ctx: Context) -> list[str]:
    choices = (
        lambda: f"M106 S{ctx.rng.randint(0, 255)}",
        lambda: "M107",
        lambda: f"M104 S{ctx.rng.randint(140, 200)}",
        lambda: "M117 fuzz",
    )
    return [ctx.rng.choice(choices)()]


def _mesh(ctx: Context) -> list[str]:
    if ctx.rng.random() < 0.5:
        return ["BED_MESH_CLEAR"]
    return [f"BED_MESH_PROFILE LOAD={MESH_PROFILE}"]


@functools.lru_cache(maxsize=None)
def _read_source(name: str) -> tuple[str, ...]:
    kept = []
    for raw in (GCODE_DIR / name).read_text().splitlines():
        line = raw.split(";", 1)[0].strip()
        if line and not line.startswith(SLICE_DROPPED):
            kept.append(line)
    return tuple(kept)


WORD = re.compile(r"([A-Z])(-?[\d.]+)")


def gcode_words(line: str) -> dict[str, float]:
    return {
        letter: float(value) for letter, value in WORD.findall(line.upper()[2:])
    }


@functools.lru_cache(maxsize=None)
def _z_floor(name: str) -> float:
    return min(
        gcode_words(line)["Z"]
        for line in _read_source(name)
        if line.startswith("G1") and "Z" in gcode_words(line)
    )


def _shift_z(line: str, shift: float) -> str:
    if not line.startswith("G1") or "Z" not in gcode_words(line):
        return line
    return re.sub(
        r"Z(-?[\d.]+)", lambda m: "Z" + _fmt(float(m.group(1)) - shift), line
    )


def _last_z(lines, start: int) -> Optional[float]:
    for line in reversed(lines[: start + 1]):
        if line.startswith("G1") and "Z" in gcode_words(line):
            return gcode_words(line)["Z"]
    return None


def _first_xy(lines, start: int) -> Optional[tuple[float, float]]:
    for line in lines[start:]:
        words = gcode_words(line) if line.startswith("G1") else {}
        if "X" in words and "Y" in words:
            return words["X"], words["Y"]
    return None


def _slice(ctx: Context) -> list[str]:
    name = ctx.rng.choice(SLICE_SOURCES)
    source = _read_source(name)
    count = ctx.rng.randint(*SLICE_LINES)
    start = ctx.rng.randint(0, len(source) - count)
    first = _first_xy(source, start)
    if first is None:
        return _travel(ctx)
    floor = _z_floor(name)
    shift = floor - LAYER_Z
    z = _last_z(source, start) or floor
    lead = [
        "G90",
        "M83",
        _move(z=z - shift, f=Z_FEED),
        _move(*first, f=ctx.travel_feed()),
    ]
    return lead + [
        _shift_z(line, shift) for line in source[start : start + count]
    ]


GENERATORS = {
    Kind.SLICE: _slice,
    Kind.PERIMETERS: _perimeters,
    Kind.INFILL: _infill,
    Kind.STROKES: _strokes,
    Kind.TINY: _tiny,
    Kind.ZIGZAG: _zigzag,
    Kind.TRAVEL: _travel,
    Kind.Z_MOVES: _z_moves,
    Kind.LIMITS: _limits,
    Kind.EXTRUDE_ONLY: _extrude_only,
    Kind.RETRACT: _retract,
    Kind.REST: _rest,
    Kind.G92: _g92,
    Kind.RELATIVE: _relative,
    Kind.ABSOLUTE_E: _absolute_e,
    Kind.LATTICE: _lattice,
    Kind.MISC: _misc,
    Kind.MESH: _mesh,
}

WEIGHTS = {
    Kind.SLICE: 4,
    Kind.PERIMETERS: 4,
    Kind.INFILL: 3,
    Kind.STROKES: 3,
    Kind.TINY: 3,
    Kind.ZIGZAG: 3,
    Kind.TRAVEL: 2,
    Kind.Z_MOVES: 2,
    Kind.LIMITS: 2,
    Kind.EXTRUDE_ONLY: 1,
    Kind.RETRACT: 2,
    Kind.REST: 2,
    Kind.G92: 1,
    Kind.RELATIVE: 2,
    Kind.ABSOLUTE_E: 1,
    Kind.LATTICE: 3,
    Kind.MISC: 1,
    Kind.MESH: 1,
}

PROBE_CHANCE = 0.35
SEGMENTS_PER_PROGRAM = (3, 9)


def _kinds(spec: PrinterSpec) -> list[Kind]:
    kinds = list(GENERATORS)
    if spec.mesh is MeshKind.NONE:
        kinds.remove(Kind.MESH)
    return kinds


def draw_program(rng: random.Random, spec: PrinterSpec, name: str) -> Program:
    region = Region(REGION_MARGIN_MM, spec.shape.bed_mm - REGION_MARGIN_MM)
    ctx = Context(rng, region, spec.max_velocity)
    kinds = _kinds(spec)
    weights = [WEIGHTS[kind] for kind in kinds]
    segments = []
    for _ in range(rng.randint(*SEGMENTS_PER_PROGRAM)):
        kind = rng.choices(kinds, weights)[0]
        lines = GENERATORS[kind](ctx)
        if kind is not Kind.LATTICE and rng.random() < PROBE_CHANCE:
            lines = lines + [PROBE]
        segments.append(Segment(kind, tuple(lines)))
    return Program(name, tuple(segments), end_print=rng.random() < 0.5)


def probe_lines(value: int) -> list[str]:
    return [
        "M400",
        f"SET_PIN PIN={MARK_PIN} VALUE={value}",
        f"G4 P{PROBE_HOLD_MS}",
    ]


DONE_LINES = (
    "M400",
    f"SET_PIN PIN={DONE_PIN} VALUE=1",
    f"G4 P{PROBE_HOLD_MS}",
    "M400",
)


def render(program: Program) -> list[str]:
    """The print file: probe placeholders become an M400, a mark pin
    toggle and a dwell the harness reads the step counters in. The done
    pin closes the body and flips (its schedule lead is shorter than the
    dwell) before END_PRINT starts, so no timeline action lands in the
    end macro (a PAUSE after its M84 cannot park)."""
    lines = list(PRELUDE)
    mark = 0
    for segment in program.segments:
        for line in segment.lines:
            if line != PROBE:
                lines.append(line)
                continue
            mark ^= 1
            lines += probe_lines(mark)
    if Kind.LIMITS in program.kinds():
        lines.append(LIMIT_RESET)
    lines += DONE_LINES
    if program.end_print:
        lines.append("END_PRINT")
    return lines


@dataclasses.dataclass(frozen=True)
class Probe:
    value: int
    position: tuple[float, float, float, float]


class Model:
    """Where the commanded toolhead stands after each line: G90/G91,
    M82/M83, G92, G0/G1 and the config's macros."""

    def __init__(self, position, macros: Optional[dict] = None):
        self.position = list(position)
        self.base = [0.0, 0.0, 0.0, 0.0]
        self.relative_xyz = False
        self.relative_e = False
        self.macros = macros or {}
        self.probes: list[Probe] = []

    def run(self, lines) -> None:
        for line in lines:
            self.step(line)

    def step(self, line: str) -> None:
        text = line.split(";", 1)[0].strip()
        if not text:
            return
        command = text.split()[0].upper()
        if command in self.macros:
            self.run(self.macros[command])
            return
        handler = self._HANDLERS.get(command)
        if handler is not None:
            handler(self, text)

    def _set_mode(self, text: str) -> None:
        command = text.split()[0].upper()
        if command in ("G90", "G91"):
            self.relative_xyz = command == "G91"
        else:
            self.relative_e = command == "M83"

    def _g92(self, text: str) -> None:
        for index, letter in enumerate("XYZE"):
            value = gcode_words(text).get(letter)
            if value is not None:
                self.base[index] = self.position[index] - value

    def _g1(self, text: str) -> None:
        words = gcode_words(text)
        for index, letter in enumerate("XYZE"):
            if letter not in words:
                continue
            relative = self.relative_xyz or (letter == "E" and self.relative_e)
            if relative:
                self.position[index] += words[letter]
            else:
                self.position[index] = self.base[index] + words[letter]

    def _set_pin(self, text: str) -> None:
        params = dict(
            word.upper().split("=", 1)
            for word in text.split()[1:]
            if "=" in word
        )
        if params.get("PIN") != MARK_PIN.upper():
            return
        self.probes.append(
            Probe(int(float(params["VALUE"])), tuple(self.position))
        )

    _HANDLERS = {
        "G90": _set_mode,
        "G91": _set_mode,
        "M82": _set_mode,
        "M83": _set_mode,
        "G92": _g92,
        "G0": _g1,
        "G1": _g1,
        "SET_PIN": _set_pin,
    }
