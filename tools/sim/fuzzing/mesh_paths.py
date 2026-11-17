"""Moves that drive a loaded bed mesh's seams, drawn into every program of
a printer with a mesh: the probed area's edges on each side and through
its corners, its grid lines (along one, and off one backwards), its nodes,
an edge grazed near-tangent and the fade band, each at low and high speed;
and a profile switch at the print's boundaries.

The planner reads another surface piece at each of them (a mesh patch, the
flat clamp past the probed area, a fade side). A move crossing an edge at
speed panicked it (#883, #884, fixed by PR #917), and that fix named two
seams met at once (a node, a grazing touch) and the fade band as
untested: a program's Focus biases its seams toward them."""

from __future__ import annotations

import dataclasses
import enum
import math
import random

from tools.sim.fuzzing.printer import (
    MESH_ALT_PROFILE,
    MESH_PROFILE,
    MeshGrid,
    MeshKind,
    PrinterSpec,
    mesh_grid,
)
from tools.sim.fuzzing.program import (
    EXTRUDE_PER_MM,
    MAX_TRAVEL_FEED,
    PROBE,
    PROBE_CHANCE,
    REGION_MARGIN_MM,
    Z_FEED,
    Kind,
    Program,
    Region,
    Segment,
    move,
)
from tools.sim.fuzzing.timeline import MAX_JOG_MM

XY = tuple[float, float]

LIMIT_CLEARANCE_MM = MAX_JOG_MM + 1.0
EDGE_GAP_MM = 0.5
MIN_RUN_MM = 10.0
INSIDE_RUN_MM = (5.0, 150.0)
RUN_UP_MM = (20.0, 150.0)
RUN_UP_CHANCE = 0.4
MAX_CROSSING_ANGLE_DEG = 60.0
CORNER_ANGLE_DEG = (15.0, 75.0)
CORNER_OFFSETS_MM = (0.0, 0.0, 0.0, 0.002, -0.002, 0.05, -0.05, 0.5)
EDGE_NODE_CHANCE = 0.5
DIAGONAL_CHANCE = 0.5
BACKWARDS_RUN_MM = (2.0, 40.0)
BACKWARDS_DRIFT_MM = 20.0
SHALLOW_SLOPES = (0.001, 0.01, 0.05)
SHALLOW_RUN_MM = (30.0, 200.0)
GRAZE_OFFSETS_MM = (-0.05, -0.002, 0.0, 0.002, 0.05)
ARC_RADIUS_MM = (10.0, 200.0)
ARC_HALF_ANGLE_DEG = (5.0, 30.0)
ARC_CHORD_MM = (1.0, 5.0)
FADE_OVERSHOOT_MM = 4.0
FADE_MARGIN_MM = 0.1
LOWEST_Z_MM = 0.2
LOW_SPEED_MM_S = (5.0, 30.0)
HIGH_SPEED_SHARE = 0.7
EXTRUDING_MAX_MM_S = 400.0
TRAVEL_MIN_MM_S = 100.0
EXTRUDE_CHANCE = 0.4
LAYER_Z_MM = (0.2, 2.0)
PROFILE_SWITCH_CHANCE = 0.4


class Figure(enum.Enum):
    SIDE_CROSSING = "side_crossing"
    CORNER_CROSSING = "corner_crossing"
    ALONG_GRID_LINE = "along_grid_line"
    OFF_GRID_LINE_BACKWARDS = "off_grid_line_backwards"
    THROUGH_NODE = "through_node"
    NEAR_TANGENT = "near_tangent"
    FADE_CROSSING = "fade_crossing"


class Focus(enum.Enum):
    SPREAD = "spread"
    MERGING_SEAMS = "merging_seams"
    FADE_BAND = "fade_band"


class Speed(enum.Enum):
    LOW = "low"
    HIGH = "high"


class Band(enum.Enum):
    BEFORE = "before"
    INSIDE = "inside"
    PAST = "past"


class FadeLevel(enum.Enum):
    BELOW = "below"
    START = "start"
    INSIDE = "inside"
    END = "end"
    ABOVE = "above"


FOCUS_WEIGHTS = {Focus.SPREAD: 5, Focus.MERGING_SEAMS: 3, Focus.FADE_BAND: 2}
FOCUS_FIGURE_WEIGHTS = {
    Focus.SPREAD: {},
    Focus.MERGING_SEAMS: {
        Figure.THROUGH_NODE: 4,
        Figure.NEAR_TANGENT: 4,
        Figure.CORNER_CROSSING: 3,
    },
    Focus.FADE_BAND: {Figure.FADE_CROSSING: 6},
}
SEAMS_PER_PROGRAM = {
    Focus.SPREAD: (1, 3),
    Focus.MERGING_SEAMS: (3, 6),
    Focus.FADE_BAND: (3, 6),
}


@dataclasses.dataclass(frozen=True)
class Side:
    """One edge of the probed area: the axis it bounds (0 X, 1 Y), its
    coordinate, and the sign of the direction out of the area."""

    axis: int
    edge: float
    outward: int

    def point(self, across: float, along: float) -> XY:
        """`across` mm past the edge (outward positive), at `along` on the
        other axis."""
        return _xy(self.axis, self.edge + self.outward * across, along)

    def inward(self, angle: float) -> XY:
        """Unit direction into the area, `angle` radians off the normal."""
        return _xy(self.axis, -self.outward * math.cos(angle), math.sin(angle))


@dataclasses.dataclass(frozen=True)
class Seam:
    figure: Figure
    xy: tuple[XY, ...]
    z: tuple[float, ...]
    speed: Speed
    extrude: bool


def _xy(axis: int, value: float, other: float) -> XY:
    """The point whose coordinate on `axis` is `value`."""
    return (value, other) if axis == 0 else (other, value)


def _sides(grid: MeshGrid) -> tuple[Side, ...]:
    return tuple(
        Side(axis, edge, outward)
        for axis in (0, 1)
        for edge, outward in ((grid.low, -1), (grid.high, 1))
    )


def _box(grid: MeshGrid) -> tuple[float, float]:
    """Where a seam may go: a PAUSE jog from any point of it stays inside
    the travel limits [0, bed]."""
    return LIMIT_CLEARANCE_MM, grid.bed - LIMIT_CLEARANCE_MM


def _reach(grid: MeshGrid, point: XY, direction: XY) -> float:
    """How far from `point` along the unit `direction` stays in the box."""
    low, high = _box(grid)
    reach = math.inf
    for coordinate, component in zip(point, direction):
        if component > 0:
            reach = min(reach, (high - coordinate) / component)
        elif component < 0:
            reach = min(reach, (low - coordinate) / component)
    return reach


def _ray(point: XY, direction: XY, distance: float) -> XY:
    return (
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
    )


def _coordinate(rng: random.Random, grid: MeshGrid) -> float:
    """A coordinate before, inside or past the probed area, alike."""
    low, high = _box(grid)
    band = rng.choice(tuple(Band))
    if band is Band.BEFORE:
        return rng.uniform(low, grid.low - EDGE_GAP_MM)
    if band is Band.PAST:
        return rng.uniform(grid.high + EDGE_GAP_MM, high)
    return rng.uniform(grid.low, grid.high)


def _run(rng: random.Random, grid: MeshGrid) -> tuple[float, float]:
    start = _coordinate(rng, grid)
    end = _coordinate(rng, grid)
    while abs(end - start) < MIN_RUN_MM:
        end = _coordinate(rng, grid)
    return start, end


def _side_crossing(rng: random.Random, grid: MeshGrid) -> list[XY]:
    """Into (or, reversed, out of) the area across one side, at an angle
    off its normal, optionally after a run-up along the outside band."""
    side = rng.choice(_sides(grid))
    angle = math.radians(
        rng.uniform(-MAX_CROSSING_ANGLE_DEG, MAX_CROSSING_ANGLE_DEG)
    )
    crossing = side.point(0.0, rng.uniform(grid.low + 1.0, grid.high - 1.0))
    inward = side.inward(angle)
    outward = (-inward[0], -inward[1])
    gap = EDGE_GAP_MM / math.cos(angle)
    outside = _ray(
        crossing, outward, rng.uniform(gap, _reach(grid, crossing, outward))
    )
    inside = _ray(
        crossing,
        inward,
        rng.uniform(
            INSIDE_RUN_MM[0],
            min(INSIDE_RUN_MM[1], _reach(grid, crossing, inward)),
        ),
    )
    path = [outside, inside]
    if rng.random() < RUN_UP_CHANCE:
        along = _xy(side.axis, 0.0, rng.choice((-1.0, 1.0)))
        run_up = min(rng.uniform(*RUN_UP_MM), _reach(grid, outside, along))
        path.insert(0, _ray(outside, along, run_up))
    return path if rng.random() < 0.5 else path[::-1]


def _corner_crossing(rng: random.Random, grid: MeshGrid) -> list[XY]:
    """Through a corner of the area, where both edges cross at once, or
    `offset` mm beside it so the two crossings come apart slightly."""
    corner_x, sign_x = rng.choice(((grid.low, -1), (grid.high, 1)))
    corner_y, sign_y = rng.choice(((grid.low, -1), (grid.high, 1)))
    angle = math.radians(rng.uniform(*CORNER_ANGLE_DEG))
    outward = (sign_x * math.cos(angle), sign_y * math.sin(angle))
    inward = (-outward[0], -outward[1])
    offset = rng.choice(CORNER_OFFSETS_MM)
    through = (corner_x - offset * outward[1], corner_y + offset * outward[0])
    gap = EDGE_GAP_MM / min(abs(outward[0]), abs(outward[1]))
    outside = _ray(
        through, outward, rng.uniform(gap, _reach(grid, through, outward))
    )
    inside = _ray(
        through,
        inward,
        rng.uniform(
            INSIDE_RUN_MM[0],
            min(INSIDE_RUN_MM[1], _reach(grid, through, inward)),
        ),
    )
    path = [outside, inside]
    return path if rng.random() < 0.5 else path[::-1]


def _along_grid_line(rng: random.Random, grid: MeshGrid) -> list[XY]:
    """Exactly along one grid line (an edge among them), often from or to
    past the area, across the perpendicular edges."""
    axis = rng.choice((0, 1))
    line = rng.choice(grid.lines)
    start, end = _run(rng, grid)
    return [_xy(axis, line, start), _xy(axis, line, end)]


def _off_grid_line_backwards(rng: random.Random, grid: MeshGrid) -> list[XY]:
    """Arrives on a grid line and leaves it back the way it came (the
    path starting on a mesh line that an intermediate #917 fix mistook for
    a crossing)."""
    axis = rng.choice((0, 1))
    line = rng.choice(grid.lines)
    arriving = rng.choice((-1.0, 1.0))
    low, high = _box(grid)
    other = rng.uniform(low, high)

    def back(distance: float, drift: float) -> XY:
        across = min(max(line - arriving * distance, low), high)
        return _xy(axis, across, min(max(other + drift, low), high))

    drift = rng.choice(
        (0.0, rng.uniform(-BACKWARDS_DRIFT_MM, BACKWARDS_DRIFT_MM))
    )
    return [
        back(rng.uniform(*BACKWARDS_RUN_MM), -drift),
        _xy(axis, line, other),
        back(rng.uniform(*BACKWARDS_RUN_MM), drift),
    ]


def _node(rng: random.Random, grid: MeshGrid) -> XY:
    edges = (grid.low, grid.high)
    x, y = rng.choice(grid.lines), rng.choice(grid.lines)
    if rng.random() < EDGE_NODE_CHANCE:
        x = rng.choice(edges)
    if rng.random() < EDGE_NODE_CHANCE:
        y = rng.choice(edges)
    return x, y


def _through_node(rng: random.Random, grid: MeshGrid) -> list[XY]:
    """A straight line through a grid node, so two grid lines (two edges
    at a corner node) cross at one instant. Diagonals step whole
    millimetres, so the rendered endpoints keep the node exactly on the
    line."""
    node = _node(rng, grid)
    if rng.random() < DIAGONAL_CHANCE:
        direction = (rng.choice((-1.0, 1.0)), rng.choice((-1.0, 1.0)))
        backward = (-direction[0], -direction[1])
        before = rng.randint(1, math.floor(_reach(grid, node, backward)))
        after = rng.randint(1, math.floor(_reach(grid, node, direction)))
        return [
            (node[0] - before * direction[0], node[1] - before * direction[1]),
            (node[0] + after * direction[0], node[1] + after * direction[1]),
        ]
    angle = rng.uniform(0.0, math.pi)
    direction = (math.cos(angle), math.sin(angle))
    backward = (-direction[0], -direction[1])
    return [
        _ray(node, backward, rng.uniform(1.0, _reach(grid, node, backward))),
        _ray(node, direction, rng.uniform(1.0, _reach(grid, node, direction))),
    ]


def _shallow_crossing(
    rng: random.Random, grid: MeshGrid, side: Side
) -> list[XY]:
    """Crosses `side` at a slope of a few thousandths: a long, near-tangent
    approach to the seam."""
    slope = rng.choice(SHALLOW_SLOPES)
    run = min(rng.uniform(*SHALLOW_RUN_MM), grid.high - grid.low - 2.0)
    start = rng.uniform(grid.low + 1.0, grid.high - 1.0 - run)
    half = slope * run / 2
    path = [side.point(half, start), side.point(-half, start + run)]
    return path if rng.random() < 0.5 else path[::-1]


def _grazing_arc(rng: random.Random, grid: MeshGrid, side: Side) -> list[XY]:
    """Chords of an arc whose apex reaches `offset` mm past `side`: a
    positive offset crosses out and back within a few microns, zero
    touches, a negative one misses. The arc bulges out of the area from
    inside or into it from outside."""
    offset = rng.choice(GRAZE_OFFSETS_MM)
    bulge = rng.choice((-1, 1))
    half = math.radians(rng.uniform(*ARC_HALF_ANGLE_DEG))
    outside_room = grid.low - LIMIT_CLEARANCE_MM
    widest = (grid.high - grid.low - 2.0) / (2 * math.sin(half))
    tallest = (outside_room - abs(offset)) / (1 - math.cos(half))
    longest = ARC_RADIUS_MM[1] if bulge > 0 else min(ARC_RADIUS_MM[1], tallest)
    radius = rng.uniform(ARC_RADIUS_MM[0], min(longest, widest))
    chords = 2 * math.ceil(half * radius / rng.uniform(*ARC_CHORD_MM))
    reach = radius * math.sin(half)
    middle = rng.uniform(grid.low + 1.0 + reach, grid.high - 1.0 - reach)
    apex = bulge * offset
    centre = apex - bulge * radius
    points = []
    for index in range(chords + 1):
        phi = -half + 2 * half * index / chords
        across = centre + bulge * radius * math.cos(phi)
        points.append(side.point(across, middle + radius * math.sin(phi)))
    return points


def _near_tangent(rng: random.Random, grid: MeshGrid) -> list[XY]:
    side = rng.choice(_sides(grid))
    if rng.random() < 0.5:
        return _shallow_crossing(rng, grid, side)
    return _grazing_arc(rng, grid, side)


def _fade_height(
    rng: random.Random, fade: tuple[float, float], level: FadeLevel
) -> float:
    start, end = fade
    if level is FadeLevel.BELOW:
        return round(rng.uniform(LOWEST_Z_MM, start - FADE_MARGIN_MM), 3)
    if level is FadeLevel.START:
        return start
    if level is FadeLevel.INSIDE:
        return round(
            rng.uniform(start + FADE_MARGIN_MM, end - FADE_MARGIN_MM), 3
        )
    if level is FadeLevel.END:
        return end
    return round(rng.uniform(end + FADE_MARGIN_MM, end + FADE_OVERSHOOT_MM), 3)


def _fade_heights(
    rng: random.Random, fade: tuple[float, float], count: int
) -> tuple[float, ...]:
    """Z from one fade level to another (a different one, so the move
    crosses or leaves a fade side) spread over the path's points."""
    first, last = rng.sample(list(FadeLevel), 2)
    low, high = _fade_height(rng, fade, first), _fade_height(rng, fade, last)
    return tuple(
        round(low + (high - low) * index / (count - 1), 3)
        for index in range(count)
    )


FLAT_FIGURES = {
    Figure.SIDE_CROSSING: _side_crossing,
    Figure.CORNER_CROSSING: _corner_crossing,
    Figure.ALONG_GRID_LINE: _along_grid_line,
    Figure.OFF_GRID_LINE_BACKWARDS: _off_grid_line_backwards,
    Figure.THROUGH_NODE: _through_node,
    Figure.NEAR_TANGENT: _near_tangent,
}
FADE_PATHS = (Figure.SIDE_CROSSING, Figure.THROUGH_NODE, Figure.ALONG_GRID_LINE)


def draw_seam(rng: random.Random, grid: MeshGrid, figure: Figure) -> Seam:
    if figure is Figure.FADE_CROSSING:
        if grid.fade is None:
            raise ValueError("a fade crossing needs a mesh with a fade band")
        xy = FLAT_FIGURES[rng.choice(FADE_PATHS)](rng, grid)
        z = _fade_heights(rng, grid.fade, len(xy))
    else:
        xy = FLAT_FIGURES[figure](rng, grid)
        z = (round(rng.uniform(*LAYER_Z_MM), 2),) * len(xy)
    return Seam(
        figure,
        tuple(xy),
        z,
        rng.choice(tuple(Speed)),
        rng.random() < EXTRUDE_CHANCE,
    )


def _feed(rng: random.Random, seam: Seam, max_velocity: float) -> float:
    ceiling = min(
        max_velocity,
        EXTRUDING_MAX_MM_S if seam.extrude else MAX_TRAVEL_FEED / 60,
    )
    if seam.speed is Speed.LOW:
        return 60 * round(rng.uniform(*LOW_SPEED_MM_S))
    return 60 * round(rng.uniform(HIGH_SPEED_SHARE, 1.0) * ceiling)


def seam_lines(
    rng: random.Random, seam: Seam, grid: MeshGrid, max_velocity: float
) -> list[str]:
    """Opens with an absolute travel to the seam's start, like every
    motion segment, and ends back inside the print region."""
    travel = 60 * round(
        rng.uniform(TRAVEL_MIN_MM_S, min(max_velocity, MAX_TRAVEL_FEED / 60))
    )
    feed = _feed(rng, seam, max_velocity)
    lines = [
        "G90",
        move(z=seam.z[0], f=Z_FEED),
        move(*seam.xy[0], f=travel),
    ]
    steps = zip(seam.xy, seam.xy[1:], seam.z, seam.z[1:])
    for start, end, z_start, z_end in steps:
        e = (
            round(math.dist(start, end) * EXTRUDE_PER_MM, 5)
            if seam.extrude
            else None
        )
        lines.append(
            move(*end, z=None if z_end == z_start else z_end, e=e, f=feed)
        )
    region = Region(REGION_MARGIN_MM, grid.bed - REGION_MARGIN_MM)
    lines.append(move(*region.point(rng), f=travel))
    return lines


def draw_focus(rng: random.Random, grid: MeshGrid) -> Focus:
    focuses = [
        f for f in Focus if grid.fade is not None or f is not Focus.FADE_BAND
    ]
    return rng.choices(focuses, [FOCUS_WEIGHTS[f] for f in focuses])[0]


def draw_figure(rng: random.Random, grid: MeshGrid, focus: Focus) -> Figure:
    figures = [
        f
        for f in Figure
        if grid.fade is not None or f is not Figure.FADE_CROSSING
    ]
    weights = [FOCUS_FIGURE_WEIGHTS[focus].get(f, 1) for f in figures]
    return rng.choices(figures, weights)[0]


def _profile_load(name: str) -> Segment:
    return Segment(Kind.MESH, (f"BED_MESH_PROFILE LOAD={name}",))


def with_seams(
    rng: random.Random, spec: PrinterSpec, program: Program
) -> Program:
    """`program` with seam segments at drawn places, and sometimes a
    profile load at each end (different profiles, so every print boundary
    of a session switches). A printer without a mesh keeps it as is."""
    if spec.mesh is MeshKind.NONE:
        return program
    grid = mesh_grid(spec)
    focus = draw_focus(rng, grid)
    segments = list(program.segments)
    for _ in range(rng.randint(*SEAMS_PER_PROGRAM[focus])):
        seam = draw_seam(rng, grid, draw_figure(rng, grid, focus))
        lines = seam_lines(rng, seam, grid, spec.max_velocity)
        if rng.random() < PROBE_CHANCE:
            lines.append(PROBE)
        segment = Segment(Kind.MESH_SEAM, tuple(lines))
        segments.insert(rng.randint(0, len(segments)), segment)
    if rng.random() < PROFILE_SWITCH_CHANCE:
        first, last = rng.sample((MESH_PROFILE, MESH_ALT_PROFILE), 2)
        segments = [_profile_load(first), *segments, _profile_load(last)]
    return dataclasses.replace(program, segments=tuple(segments))
