"""Stroke plans for servo calibration: kinematics-derived move sequences.

Consults the active kinematics (`coupled_xy()`, `rails`) so no command in
`servo_calibration.py` re-derives which drives move for a given axis or
CoreXY diagonal. A `StrokePlan` is plain data - a coordinate callback plus
the servo/motor list a caller needs to capture and prep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

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


def goto_xy(
    gcode: Any, travel_speed: float, x: float, y: float, dwell: int
) -> None:
    gcode.run_script_from_command(
        "\n".join(
            [
                "G90",
                "G1 X%.3f Y%.3f F%d" % (x, y, int(travel_speed * 60)),
                "M400",
                "G4 P%d" % (dwell,),
                "M400",
            ]
        )
    )


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
