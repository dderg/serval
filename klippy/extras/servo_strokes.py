"""Stroke plans for servo calibration: kinematics-derived move sequences.

Consults the active kinematics (`coupled_xy()`, `rails`) so no command in
`servo_calibration` re-derives which drives move for a given axis or
CoreXY diagonal. A `StrokePlan` is plain data - a coordinate callback plus
the servo/motor list a caller needs to capture and prep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence, overload

from . import servo_axis

Bounds = dict[str, tuple[float, float]]


@dataclass
class StrokePlan:
    coord: Callable[[float], str]
    start: float
    end: float
    th_per_unit: float
    servos: list[str]
    motors: list[servo_axis.ServoMotor]
    prep: tuple[str, ...]
    diagonal: bool
    rails: list[servo_axis.ServoRail] = field(default_factory=list)


@overload
def parse_floats(text: str) -> list[float]: ...
@overload
def parse_floats(text: None) -> None: ...
def parse_floats(text: str | None) -> list[float] | None:
    if text is None:
        return None
    return [float(p.strip()) for p in text.split(",") if p.strip()]


def grid(
    gcmd: Any,
    accels_default: Sequence[float],
    speeds_default: Sequence[float],
    iterations_default: int,
    dwell_default: int,
) -> tuple[list[float], list[float], int, int]:
    accels = parse_floats(gcmd.get("ACCELS", None)) or list(accels_default)
    speeds = parse_floats(gcmd.get("SPEEDS", None)) or list(speeds_default)
    iterations = gcmd.get_int("ITERATIONS", iterations_default, minval=1)
    dwell = gcmd.get_int("DWELL_MS", dwell_default, minval=0)
    return accels, speeds, iterations, dwell


def axis_bounds(gcmd: Any, bounds: Bounds, axis: str) -> tuple[float, float]:
    lo, hi = bounds.get(axis, (None, None))
    start = gcmd.get_float("START", lo)
    end = gcmd.get_float("END", hi)
    if start is None or end is None:
        raise gcmd.error(
            "START/END required for axis %s - no bounds configured" % (axis,)
        )
    return start, end


def xy_bounds(gcmd: Any, bounds: Bounds) -> tuple[float, float, float, float]:
    return (
        gcmd.get_float("X_START", bounds["X"][0]),
        gcmd.get_float("X_END", bounds["X"][1]),
        gcmd.get_float("Y_START", bounds["Y"][0]),
        gcmd.get_float("Y_END", bounds["Y"][1]),
    )


def axis_rails(gcmd: Any, kin: Any, axis: str) -> list[servo_axis.ServoRail]:
    if axis not in ("X", "Y", "Z"):
        raise gcmd.error("AXIS must be X, Y or Z (got %r)" % (axis,))
    lane = "XYZ".index(axis)
    lanes = [0, 1] if kin.coupled_xy() and lane in (0, 1) else [lane]
    rails = []
    for i in lanes:
        rail = kin.rails[i]
        if not isinstance(rail, servo_axis.ServoRail):
            raise gcmd.error(
                "axis %s is driven by non-servo rail %r"
                % (axis, rail.get_name())
            )
        rails.append(rail)
    return rails


def axis_servos(gcmd: Any, kin: Any, axis: str) -> list[str]:
    return [
        m.get_motor_name()
        for r in axis_rails(gcmd, kin, axis)
        for m in r.get_motors()
    ]


def rail_motors_in_slot_order(
    rail: servo_axis.ServoRail,
) -> list[servo_axis.ServoMotor]:
    return sorted(rail.get_motors(), key=lambda m: m.get_chain_index())


def _rail_columns(
    rail: servo_axis.ServoRail,
) -> list[tuple[str, float, int]]:
    motors = rail_motors_in_slot_order(rail)
    return [
        (
            m.get_motor_name(),
            -1.0 if m.get_invert_direction() else 1.0,
            len(motors),
        )
        for m in motors
    ]


def spatial_frame(kin: Any) -> dict[str, Any] | None:
    """Motor-space -> cartesian position map for the dashboard's spatial
    view: mode_pos[k] = sum(frame[k][s] * drive_frame_pos_mm[s]) over the
    motors listed in `axes`. Each motor's invert sign is folded into its
    column (the SERVO_FIT_DYNAMICS profile convention), so raw drive-frame
    positions go in. None when no servo rail drives X or Y."""
    if kin.coupled_xy():
        belts = list(kin.rails[:2])
        if not all(isinstance(r, servo_axis.ServoRail) for r in belts):
            return None
        columns = [
            (belt, name, sign, drives)
            for belt, rail in enumerate(belts)
            for name, sign, drives in _rail_columns(rail)
        ]
        return {
            "modes": ["x", "y"],
            "axes": [name for _b, name, _s, _d in columns],
            "frame": [
                [sign / (2.0 * drives) for _b, _n, sign, drives in columns],
                [
                    (sign if belt == 0 else -sign) / (2.0 * drives)
                    for belt, _n, sign, drives in columns
                ],
            ],
        }
    lanes = [
        (mode, kin.rails[lane])
        for lane, mode in ((0, "x"), (1, "y"))
        if lane < len(kin.rails)
        and isinstance(kin.rails[lane], servo_axis.ServoRail)
    ]
    if not lanes:
        return None
    axes: list[str] = []
    columns = []
    for mode, rail in lanes:
        for name, sign, drives in _rail_columns(rail):
            axes.append(name)
            columns.append((mode, sign, drives))
    return {
        "modes": [mode for mode, _rail in lanes],
        "axes": axes,
        "frame": [
            [
                sign / drives if col_mode == mode else 0.0
                for col_mode, sign, drives in columns
            ]
            for mode, _rail in lanes
        ],
    }


def diagonal_rail(gcmd: Any, kin: Any, axis: str) -> servo_axis.ServoRail:
    if not kin.coupled_xy():
        raise gcmd.error(
            "AXIS=%s runs a CoreXY diagonal - the active kinematics is "
            "not coupled_xy" % (axis,)
        )
    lane = 0 if axis == "A" else 1
    rail = kin.rails[lane]
    if not isinstance(rail, servo_axis.ServoRail):
        raise gcmd.error(
            "CoreXY lane %d is driven by non-servo rail %r"
            % (lane, rail.get_name())
        )
    return rail


def build_plan(gcmd: Any, kin: Any, bounds: Bounds, axis: str) -> StrokePlan:
    if axis in ("A", "B"):
        rail = diagonal_rail(gcmd, kin, axis)
        x_start, x_end, y_start, y_end = xy_bounds(gcmd, bounds)
        xc = (x_start + x_end) / 2.0
        yc = (y_start + y_end) / 2.0
        half = min(abs(x_end - x_start), abs(y_end - y_start)) / 2.0
        start = gcmd.get_float("START", -half)
        end = gcmd.get_float("END", half)
        sign = 1.0 if axis == "A" else -1.0

        def coord(u: float) -> str:
            return "X%.3f Y%.3f" % (xc + u, yc + sign * u)

        motors = list(rail.get_motors())
        return StrokePlan(
            coord=coord,
            start=start,
            end=end,
            th_per_unit=math.sqrt(2.0),
            servos=[m.get_motor_name() for m in motors],
            motors=motors,
            prep=("X", "Y"),
            diagonal=True,
        )

    start, end = axis_bounds(gcmd, bounds, axis)
    rails = axis_rails(gcmd, kin, axis)
    motors = [m for r in rails for m in r.get_motors()]

    def coord(u: float) -> str:
        return "%s%.3f" % (axis, u)

    return StrokePlan(
        coord=coord,
        start=start,
        end=end,
        th_per_unit=1.0,
        servos=[m.get_motor_name() for m in motors],
        motors=motors,
        prep=(axis,),
        diagonal=False,
        rails=rails,
    )


def check_reachable(
    gcode: Any, length: float, speed: float, accel: float
) -> None:
    reach = speed * speed / accel
    if reach > length:
        raise gcode.error(
            "stroke %.1fmm (toolhead frame) too short to reach %.0fmm/s "
            "at %.0fmm/s^2 (needs %.1fmm)" % (length, speed, accel, reach)
        )


def emit_strokes(
    gcode: Any,
    coord: Callable[[float], str],
    start: float,
    end: float,
    th_per_unit: float,
    speed: float,
    accel: float,
    iterations: int,
    dwell: int,
) -> None:
    if end <= start:
        raise gcode.error("END=%.1f must exceed START=%.1f" % (end, start))
    check_reachable(gcode, (end - start) * th_per_unit, speed, accel)
    feed = int(speed * 60)
    lines = ["SET_VELOCITY_LIMIT ACCEL=%.0f" % (accel,), "G90"]
    for _ in range(iterations):
        lines += [
            "G1 %s F%d" % (coord(end), feed),
            "M400",
            "G4 P%d" % (dwell,),
            "M400",
            "G1 %s F%d" % (coord(start), feed),
            "M400",
            "G4 P%d" % (dwell,),
            "M400",
        ]
    gcode.run_script_from_command("\n".join(lines))


def emit_strokes_with_stop_times(
    printer: Any,
    gcode: Any,
    coord: Callable[[float], str],
    start: float,
    end: float,
    th_per_unit: float,
    speed: float,
    accel: float,
    iterations: int,
    dwell: int,
) -> list[float]:
    """`emit_strokes`, but each stroke is submitted alone and its
    commanded-stop print-time is read off the motion fence before the dwell —
    the ring-down analyzer windows accelerometer tails from these."""
    if end <= start:
        raise gcode.error("END=%.1f must exceed START=%.1f" % (end, start))
    check_reachable(gcode, (end - start) * th_per_unit, speed, accel)
    toolhead = printer.lookup_object("toolhead")
    feed = int(speed * 60)
    gcode.run_script_from_command(
        "SET_VELOCITY_LIMIT ACCEL=%.0f\nG90" % (accel,)
    )
    stops: list[float] = []
    for _ in range(iterations):
        for target in (end, start):
            gcode.run_script_from_command("G1 %s F%d" % (coord(target), feed))
            stops.append(toolhead.get_last_move_time())
            gcode.run_script_from_command("M400\nG4 P%d\nM400" % (dwell,))
    return stops


@dataclass
class PatternMove:
    x: float
    y: float
    length: float
    peak_velocity: float


def pattern_points(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    small_size: float,
) -> list[tuple[float, float]]:
    """TEST_SPEED-style corner sequence: diagonals then box over the large
    bounds, then the same over a small box centered in them. Starts and
    ends at (x_min, y_min) so iterations chain with identical segment
    lengths; the caller travels there first."""

    def diagonals_then_box(
        lo_x: float, hi_x: float, lo_y: float, hi_y: float
    ) -> list[tuple[float, float]]:
        return [
            (hi_x, hi_y),
            (lo_x, lo_y),
            (hi_x, lo_y),
            (lo_x, hi_y),
            (hi_x, lo_y),
            (lo_x, lo_y),
            (lo_x, hi_y),
            (hi_x, hi_y),
            (hi_x, lo_y),
            (lo_x, lo_y),
        ]

    xc = (x_min + x_max) / 2.0
    yc = (y_min + y_max) / 2.0
    half = small_size / 2.0
    return (
        diagonals_then_box(x_min, x_max, y_min, y_max)
        + diagonals_then_box(xc - half, xc + half, yc - half, yc + half)
        + [(x_min, y_min)]
    )


def pattern_geometry(
    gcode: Any,
    x_lo: float,
    x_hi: float,
    y_lo: float,
    y_hi: float,
    inset: float,
    small_size: float,
) -> tuple[list[tuple[float, float]], float, float]:
    x_min, x_max = x_lo + inset, x_hi - inset
    y_min, y_max = y_lo + inset, y_hi - inset
    if x_max <= x_min or y_max <= y_min:
        raise gcode.error(
            "pattern bounds collapse after inset %.1f: X %.1f..%.1f "
            "Y %.1f..%.1f" % (inset, x_min, x_max, y_min, y_max)
        )
    if small_size <= 0.0:
        raise gcode.error("SMALL_SIZE=%.1f must be positive" % (small_size,))
    if small_size > min(x_max - x_min, y_max - y_min):
        raise gcode.error(
            "SMALL_SIZE=%.1f exceeds the inset pattern bounds "
            "(X span %.1f, Y span %.1f)"
            % (small_size, x_max - x_min, y_max - y_min)
        )
    points = pattern_points(x_min, x_max, y_min, y_max, small_size)
    return points, x_min, y_min


def pattern_moves(
    gcode: Any,
    points: Sequence[tuple[float, float]],
    start_x: float,
    start_y: float,
    speed: float,
    accel: float,
) -> list[PatternMove]:
    """Per-segment achieved-peak velocity assuming a rest-to-rest profile:
    min(speed, sqrt(accel*length)). Segments below `speed` run a triangular
    profile - intended behavior, not an error, so results can be labeled by
    what the toolhead actually reached instead of the requested feed."""
    moves: list[PatternMove] = []
    px, py = start_x, start_y
    for x, y in points:
        length = math.hypot(x - px, y - py)
        if length <= 0.0:
            raise gcode.error(
                "degenerate pattern segment at X%.3f Y%.3f (zero length)"
                % (x, y)
            )
        moves.append(
            PatternMove(x, y, length, min(speed, math.sqrt(accel * length)))
        )
        px, py = x, y
    return moves


def emit_pattern(
    gcode: Any,
    points: Sequence[tuple[float, float]],
    start_x: float,
    start_y: float,
    speed: float,
    accel: float,
    iterations: int,
    dwell: int,
) -> list[PatternMove]:
    moves = pattern_moves(gcode, points, start_x, start_y, speed, accel)
    feed = int(speed * 60)
    lines = [
        "SET_VELOCITY_LIMIT VELOCITY=%.0f ACCEL=%.0f" % (speed, accel),
        "G90",
    ]
    for _ in range(iterations):
        lines += ["G0 X%.3f Y%.3f F%d" % (m.x, m.y, feed) for m in moves]
        lines += ["M400", "G4 P%d" % (dwell,), "M400"]
    gcode.run_script_from_command("\n".join(lines))
    return moves


def pattern_reach_summary(moves: Sequence[PatternMove], speed: float) -> str:
    triangular = [m for m in moves if m.peak_velocity < speed]
    if not triangular:
        return "all %d pattern segments reach %.0fmm/s" % (len(moves), speed)
    slowest = min(m.peak_velocity for m in triangular)
    return (
        "%d of %d pattern segments run triangular profiles (peaks "
        "%.0f-%.0fmm/s of the requested %.0fmm/s) - kept on purpose to "
        "excite the low-velocity range"
        % (
            len(triangular),
            len(moves),
            slowest,
            max(m.peak_velocity for m in triangular),
            speed,
        )
    )


def goto(gcode: Any, travel_speed: float, coord: str, dwell: int) -> None:
    gcode.run_script_from_command(
        "\n".join(
            [
                "G90",
                "G1 %s F%d" % (coord, int(travel_speed * 60)),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
            ]
        )
    )


def goto_xy(
    gcode: Any, travel_speed: float, x: float, y: float, dwell: int
) -> None:
    goto(gcode, travel_speed, "X%.3f Y%.3f" % (x, y), dwell)


def corexy_fit_layout(gcmd: Any, kin: Any) -> dict[str, Any]:
    if not kin.coupled_xy():
        raise gcmd.error(
            "corexy fit layout requires coupled_xy kinematics; the active "
            "kinematics is cartesian"
        )
    rails = axis_rails(gcmd, kin, "X")
    pairs = [
        [m.get_motor_name() for m in rail_motors_in_slot_order(r)]
        for r in rails
    ]
    sizes = {len(p) for p in pairs}
    servos = [name for pair in pairs for name in pair]
    if sizes == {1}:
        return {"servos": servos, "pairs": None}
    if sizes == {2}:
        nodes = {m.get_node_name() for r in rails for m in r.get_motors()}
        if len(nodes) != 1:
            raise gcmd.error(
                "AWD corexy fit needs all four drives on one ethercat node "
                "(a coupled dynamics profile is per node); got nodes: %s"
                % (", ".join(sorted(nodes)),)
            )
        return {
            "servos": servos,
            "pairs": ";".join(",".join(pair) for pair in pairs),
        }
    raise gcmd.error(
        "corexy fit needs one or two drives per belt on both belts, got %s"
        % (
            "; ".join(
                "%s: %s" % (r.get_name(short=True), ", ".join(p))
                for r, p in zip(rails, pairs)
            ),
        )
    )


def belt_pair(
    printer: Any, gcmd: Any, kin: Any, belt: str, cmd_name: str
) -> tuple[list[str], list[Any], Any, list[int]]:
    layout = corexy_fit_layout(gcmd, kin)
    if layout["pairs"] is None:
        raise gcmd.error(
            "%s needs two drives per belt "
            "(AWD); this printer has one drive per belt" % (cmd_name,)
        )
    pair_names = layout["pairs"].split(";")["AB".index(belt)].split(",")
    motors = [
        servo_axis.resolve_servo_motor(printer, name, cmd_name)[1]
        for name in pair_names
    ]
    node = printer.lookup_object("ethercat_node " + motors[0].get_node_name())
    handle = node.get_engine_handle()
    if handle is None:
        raise gcmd.error(
            "belt %s drives have no live EtherCAT engine handle "
            "(node not claimed)" % (belt,)
        )
    slots = [node.get_slot_for_motor(m.get_motor_name()) for m in motors]
    return pair_names, motors, handle, slots


def check_servos_override(gcmd: Any, layout: dict[str, Any]) -> None:
    override = gcmd.get("SERVOS", None)
    if override is None:
        return
    given = sorted(s.strip() for s in override.split(",") if s.strip())
    if given != sorted(layout["servos"]):
        raise gcmd.error(
            "SERVOS=%s does not match the drives the kinematics says power "
            "the belts (%s); the fit pairing is derived from the "
            "kinematics, so drop SERVOS= or fix the config"
            % (override, ", ".join(layout["servos"]))
        )


def scalar_fit_drive(gcmd: Any, kin: Any) -> str | None:
    axis = gcmd.get("AXIS", "X").upper()
    servos = axis_servos(gcmd, kin, axis)
    drive = gcmd.get("DRIVE", None)
    if drive is None:
        if len(servos) > 1:
            raise gcmd.error(
                "AXIS=%s records %d drives (%s); pass DRIVE= to pick which "
                "one the scalar fit describes"
                % (axis, len(servos), ", ".join(servos))
            )
        return None
    if drive not in servos:
        raise gcmd.error(
            "DRIVE=%s is not among the drives of AXIS=%s (%s)"
            % (drive, axis, ", ".join(servos))
        )
    return drive


def prep(printer: Any, gcode: Any, axis: str, dwell: int) -> None:
    curtime = printer.get_reactor().monotonic()
    toolhead = printer.lookup_object("toolhead")
    homed = toolhead.get_kinematics().get_status(curtime)["homed_axes"]
    lines = []
    if axis.lower() not in homed:
        lines.append("G28 %s" % (axis,))
    lines += ["M400", "G4 P%d" % (dwell,), "M400"]
    gcode.run_script_from_command("\n".join(lines))
