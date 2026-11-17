"""The mesh seam figures drive what each one is named for, inside the
travel limits a PAUSE jog still fits in, and generated scenarios carry
them: every side crossed both ways, a corner, grid lines, nodes, grazing
touches, fade-band crossings and profile switches."""

import collections
import dataclasses
import math
import random

import pytest

from tools.sim.fuzzing import mesh_paths, printer, program, scenario
from tools.sim.fuzzing.mesh_paths import Figure, Focus, Speed
from tools.sim.fuzzing.printer import MeshKind

pytestmark = pytest.mark.sim_unit

DRAWS = 400
SCENARIO_SEEDS = range(300)
ON_LINE_MM = 1e-6
FADE = (printer.MESH_FADE_START_MM, printer.MESH_FADE_END_MM)


def _grid(bed: float, probes: int, fade=FADE) -> printer.MeshGrid:
    spec = scenario.generate(0).printer
    shaped = dataclasses.replace(
        spec,
        base=next(b for b, s in printer.BASES.items() if s.bed_mm == bed),
        mesh=MeshKind.WAVY,
        mesh_points=((0.0,) * probes,) * probes,
        mesh_fade=fade is not None,
    )
    return printer.mesh_grid(shaped)


GRIDS = [_grid(200.0, 3), _grid(250.0, 5), _grid(300.0, 5, fade=None)]


def _seams(grid, figure, seed=1):
    rng = random.Random(seed)
    return [mesh_paths.draw_seam(rng, grid, figure) for _ in range(DRAWS)]


def _flat_figures(grid):
    return [f for f in Figure if grid.fade or f is not Figure.FADE_CROSSING]


def _crossings(xy, axis, edge):
    """+1 per step from below the edge to above it, -1 the other way."""
    for start, end in zip(xy, xy[1:]):
        if start[axis] < edge < end[axis]:
            yield 1
        elif end[axis] < edge < start[axis]:
            yield -1


def _distance_to_segment(point, start, end) -> float:
    span = (end[0] - start[0], end[1] - start[1])
    length = math.hypot(*span)
    along = (
        (point[0] - start[0]) * span[0] + (point[1] - start[1]) * span[1]
    ) / length
    along = min(max(along, 0.0), length)
    nearest = (
        start[0] + span[0] * along / length,
        start[1] + span[1] * along / length,
    )
    return math.dist(point, nearest)


@pytest.mark.parametrize("grid", GRIDS)
def test_every_seam_leaves_room_for_a_pause_jog(grid):
    low, high = (
        mesh_paths.LIMIT_CLEARANCE_MM,
        grid.bed - mesh_paths.LIMIT_CLEARANCE_MM,
    )
    for figure in _flat_figures(grid):
        for seam in _seams(grid, figure):
            for point in seam.xy:
                assert all(low - 1e-9 <= c <= high + 1e-9 for c in point), (
                    figure,
                    seam.xy,
                )


@pytest.mark.parametrize("grid", GRIDS)
def test_side_crossings_enter_and_leave_through_every_side(grid):
    seen = collections.Counter()
    for seam in _seams(grid, Figure.SIDE_CROSSING):
        for axis in (0, 1):
            for edge in (grid.low, grid.high):
                for direction in _crossings(seam.xy, axis, edge):
                    seen[axis, edge, direction] += 1
    assert len(seen) == 8, seen


@pytest.mark.parametrize("grid", GRIDS)
def test_corner_crossings_pass_through_a_corner(grid):
    corners = [
        (x, y) for x in (grid.low, grid.high) for y in (grid.low, grid.high)
    ]
    exact = collections.Counter()
    for seam in _seams(grid, Figure.CORNER_CROSSING):
        start, end = seam.xy
        nearest = min(
            corners, key=lambda c: _distance_to_segment(c, start, end)
        )
        distance = _distance_to_segment(nearest, start, end)
        assert distance <= max(mesh_paths.CORNER_OFFSETS_MM) + 1e-9
        if distance < ON_LINE_MM:
            exact[nearest] += 1
    assert set(exact) == set(corners)


@pytest.mark.parametrize("grid", GRIDS)
def test_a_grid_line_run_holds_one_grid_coordinate(grid):
    past_an_edge = 0
    for seam in _seams(grid, Figure.ALONG_GRID_LINE):
        (x0, y0), (x1, y1) = seam.xy
        axis = 0 if x0 == x1 and x0 in grid.lines else 1
        assert seam.xy[0][axis] == seam.xy[1][axis] in grid.lines
        other = [p[1 - axis] for p in seam.xy]
        past_an_edge += min(other) < grid.low or max(other) > grid.high
    assert past_an_edge > DRAWS / 2


@pytest.mark.parametrize("grid", GRIDS)
def test_a_path_arrives_on_a_grid_line_and_leaves_it_backwards(grid):
    for seam in _seams(grid, Figure.OFF_GRID_LINE_BACKWARDS):
        before, on, after = seam.xy
        axis = 0 if on[0] in grid.lines else 1
        line = on[axis]
        assert line in grid.lines
        assert (before[axis] - line) * (after[axis] - line) > 0, seam.xy


@pytest.mark.parametrize("grid", GRIDS)
def test_node_paths_pass_through_nodes_corners_included(grid):
    corners = {
        (x, y) for x in (grid.low, grid.high) for y in (grid.low, grid.high)
    }
    passed = set()
    for seam in _seams(grid, Figure.THROUGH_NODE):
        start, end = seam.xy
        nodes = [
            (x, y)
            for x in grid.lines
            for y in grid.lines
            if _distance_to_segment((x, y), start, end) < ON_LINE_MM
        ]
        assert nodes, seam.xy
        passed.update(nodes)
    assert corners <= passed


@pytest.mark.parametrize("grid", GRIDS)
def test_near_tangent_paths_graze_an_edge(grid):
    apexes = set()
    for seam in _seams(grid, Figure.NEAR_TANGENT):
        if len(seam.xy) == 2:
            (x0, y0), (x1, y1) = seam.xy
            slope = min(abs(x1 - x0), abs(y1 - y0)) / max(
                abs(x1 - x0), abs(y1 - y0)
            )
            assert slope <= max(mesh_paths.SHALLOW_SLOPES) + 1e-9
            continue
        apex = seam.xy[len(seam.xy) // 2]
        for axis in (0, 1):
            for edge in (grid.low, grid.high):
                offset = round(abs(apex[axis] - edge), 9)
                if offset <= max(mesh_paths.GRAZE_OFFSETS_MM):
                    apexes.add(offset)
    assert apexes == {abs(o) for o in mesh_paths.GRAZE_OFFSETS_MM}


def _fade_side_crossed(heights) -> bool:
    start, end = FADE
    low, high = min(heights), max(heights)
    return any(low <= side <= high for side in (start, end))


def test_fade_crossings_move_xy_while_crossing_a_fade_side():
    grid = GRIDS[0]
    for seam in _seams(grid, Figure.FADE_CROSSING):
        assert _fade_side_crossed(seam.z), seam.z
        assert seam.z[0] != seam.z[-1]
        assert seam.xy[0] != seam.xy[-1]


def test_a_fade_crossing_needs_a_fade_band():
    with pytest.raises(ValueError):
        mesh_paths.draw_seam(random.Random(0), GRIDS[2], Figure.FADE_CROSSING)
    rng = random.Random(0)
    for _ in range(DRAWS):
        focus = mesh_paths.draw_focus(rng, GRIDS[2])
        assert focus is not Focus.FADE_BAND
        assert (
            mesh_paths.draw_figure(rng, GRIDS[2], focus)
            is not Figure.FADE_CROSSING
        )


def test_a_focus_biases_its_figures():
    rng = random.Random(0)
    grid = GRIDS[0]

    def share(focus, figures):
        drawn = [mesh_paths.draw_figure(rng, grid, focus) for _ in range(4000)]
        return sum(f in figures for f in drawn) / len(drawn)

    merging = {Figure.THROUGH_NODE, Figure.NEAR_TANGENT}
    assert share(Focus.MERGING_SEAMS, merging) > 1.5 * share(
        Focus.SPREAD, merging
    )
    fade = {Figure.FADE_CROSSING}
    assert share(Focus.FADE_BAND, fade) > 3 * share(Focus.SPREAD, fade)


def _feeds_mm_s(lines) -> list[float]:
    return [
        program.gcode_words(line)["F"] / 60
        for line in lines[3:-1]
        if line.startswith("G1")
    ]


def test_seams_run_at_low_and_high_speed():
    rng = random.Random(0)
    grid = GRIDS[1]
    max_velocity = 700.0
    slow = fast = 0
    for _ in range(DRAWS):
        seam = mesh_paths.draw_seam(rng, grid, Figure.SIDE_CROSSING)
        feeds = _feeds_mm_s(
            mesh_paths.seam_lines(rng, seam, grid, max_velocity)
        )
        if seam.speed is Speed.LOW:
            slow += all(f <= mesh_paths.LOW_SPEED_MM_S[1] for f in feeds)
        else:
            ceiling = (
                mesh_paths.EXTRUDING_MAX_MM_S if seam.extrude else max_velocity
            )
            fast += all(
                f >= mesh_paths.HIGH_SPEED_SHARE * ceiling - 1 for f in feeds
            )
    assert slow + fast == DRAWS and slow and fast


def test_a_printer_without_a_mesh_draws_the_program_it_always_did():
    for seed in SCENARIO_SEEDS:
        chosen = scenario.generate(seed)
        if chosen.printer.mesh is MeshKind.NONE:
            kinds = {k for p in chosen.programs for k in p.kinds()}
            assert program.Kind.MESH_SEAM not in kinds


def _moves(chosen, name):
    model = program.Model(scenario.START_POSITION + (0.0,))
    for line in program.render(chosen.program(name)):
        before = tuple(model.position)
        model.step(line)
        if line.startswith("G1"):
            yield line, before, tuple(model.position)


def test_mesh_scenarios_cross_every_side_and_the_fade_band():
    crossed = set()
    fade_crossings = 0
    for seed in SCENARIO_SEEDS:
        chosen = scenario.generate(seed)
        if chosen.printer.mesh is MeshKind.NONE:
            continue
        grid = printer.mesh_grid(chosen.printer)
        for name in chosen.prints:
            for line, before, after in _moves(chosen, name):
                for axis in (0, 1):
                    for edge in (grid.low, grid.high):
                        for direction in _crossings(
                            (before, after), axis, edge
                        ):
                            crossed.add((axis, edge == grid.low, direction))
                moved_xy = before[:2] != after[:2]
                if grid.fade and moved_xy and before[2] != after[2]:
                    fade_crossings += _fade_side_crossed((before[2], after[2]))
    assert len(crossed) == 8
    assert fade_crossings > 0


def test_profile_switches_load_different_profiles_at_each_end():
    switched = 0
    for seed in SCENARIO_SEEDS:
        for chosen_program in scenario.generate(seed).programs:
            first, last = (
                chosen_program.segments[0],
                chosen_program.segments[-1],
            )
            if first.kind is last.kind is program.Kind.MESH:
                profiles = {first.lines[0], last.lines[0]}
                switched += profiles == {
                    f"BED_MESH_PROFILE LOAD={printer.MESH_PROFILE}",
                    f"BED_MESH_PROFILE LOAD={printer.MESH_ALT_PROFILE}",
                }
    assert switched > 0


def test_the_saved_config_holds_both_profiles():
    spec = next(
        s.printer
        for s in map(scenario.generate, SCENARIO_SEEDS)
        if s.printer.mesh is not MeshKind.NONE
    )
    block = printer.saved_mesh_block(spec)
    assert f"[bed_mesh {printer.MESH_PROFILE}]" in block
    assert f"[bed_mesh {printer.MESH_ALT_PROFILE}]" in block
    alt = printer.alt_mesh_points(spec.mesh_points)
    for i, row in enumerate(spec.mesh_points):
        for j, value in enumerate(row):
            assert alt[j][i] == -value


def test_the_grid_has_mesh_pps_lines_between_probes():
    for grid, probes in zip(GRIDS, (3, 5, 5)):
        assert len(grid.lines) == (probes - 1) * (printer.MESH_PPS + 1) + 1
        assert grid.lines[0] == grid.low and grid.lines[-1] == grid.high
        assert len(grid.probe_lines) == probes
