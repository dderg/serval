"""Printer test gcodes for [corner_deviation] at the owner operating point.

    uv run python -m tools.motion_eval.printer_test [--out DIR]

Writes, into tools/motion_eval/printer_test/ by default:
- corners_<setting>.gcode: the corner/ringing pattern, one file per
  setting (scv 5, 20, 50; [corner_deviation] floor, cap 0.07, and bounded
  at max_corner_accel 1.03 x max_accel).
- stress_<setting>.gcode: dense zigzags, polyline circles and reversals
  at scv 50, floor and bounded, between two printed reference marks.

Every file is replanned through run_klippy on configs/owner.cfg under its
own setting. The generator asserts that:
- the motion stays inside the bed margin;
- every corner of the pattern is followed by a long enough
  constant-velocity window;
- the plan aligns unambiguously against itself (accel_residual.align);
- the stress blocks reach the executed peak acceleration they are there
  for.
It prints those peaks. PROCEDURE: PRINTER_TEST.md.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
import sys

import numpy as np

from klippy.extras.corner_deviation import ACCEL_MARGIN

from . import accel_residual, evaluate

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "printer_test"
CONFIG = accel_residual.DEFAULT_CONFIG
BED_MM = 350.0
BED_MARGIN_MM = 50.0
CENTER = (175.0, 175.0)
TRAVEL_Z = 10.0
POSITION_SPEED = 100.0
QUIET_LEAD_MS = 250
QUIET_TAIL_MS = 100
SELF_CHECK_RATE_HZ = 3200.0
SELF_CHECK_OFFSET_S = 1000.0031416
PRIME_GAP_MM = 5.0
BOUNDED_RATIO = 1.03
BOUNDED_CORNER_ACCEL = round(
    BOUNDED_RATIO
    * float(accel_residual.config_option(CONFIG, "printer", "max_accel"))
)
CORNER_SETTINGS = {
    "scv5": (5.0, "none"),
    "scv20": (20.0, "none"),
    "scv50": (50.0, "none"),
    "floor": (None, "floor"),
    "cap0.07": (None, "0.07"),
    "bounded": (None, "bound%d" % BOUNDED_CORNER_ACCEL),
}
STRESS_SETTINGS = ("scv50", "floor", "bounded")
BOUNDED_FILES = ("corners_bounded", "stress_bounded")
LOWEST_MODE_RATIO = 0.8
RING_PERIODS = 5.0
CRUISE_MARGIN = 1.5
SYNC_SPEED = 200.0
SYNC_TAPS = (
    ("x", 3.0, 40),
    ("y", 2.0, 90),
    ("x", -1.5, 30),
    ("y", -3.0, 60),
    ("x", 2.0, 120),
    ("y", 1.0, 50),
    ("x", -2.5, 70),
)
SYNC_SETTLE_MS = 100
CORNER_BLOCKS = (
    ("square", 0.0, 90.0, 4, 3, (50.0, 200.0, 300.0)),
    ("diamond", 45.0, 90.0, 4, 3, (50.0, 200.0)),
    ("octagon", 0.0, 45.0, 8, 2, (120.0, 300.0)),
    ("star", 0.0, 135.0, 8, 2, (200.0,)),
    ("reverse_x", 0.0, 180.0, 2, 3, (200.0,)),
    ("reverse_y", 90.0, 180.0, 2, 3, (200.0,)),
    ("reverse_diagonal", 45.0, 180.0, 2, 3, (200.0,)),
)
STRESS_CENTER = (215.0, 175.0)
STRESS_REPEATS = 8
ZIGZAG_LINES = 20
ZIGZAG_X = (6.0, 0.1, 200.0)
ZIGZAG_Y = (12.0, 1.0, 400.0)
ZIGZAG_DIAGONAL = (6.0, 0.1, 400.0)
CIRCLE = (2.5, 0.7, 400.0, 3)
REVERSAL = (12.0, 400.0, 4)
MARK_CENTER = (115.0, 175.0)
MARK_SPEED = 20.0
MARK_FIRST_Z = 0.2
MARK_SECOND_Z = 0.4
E_PER_MM = 0.0333
PRIME_MM = 30.0
CROSS_ARM_MM = 10.0
VERNIER_TICKS = 5
VERNIER_MAIN_PITCH = 1.0
VERNIER_PITCH = 0.9
VERNIER_TICK_MM = 4.0
VERNIER_INDEX_MM = 2.0
VERNIER_GAP_MM = 2.0
STRESS_PEAK_EXPECTED = 1.6
STRESS_BURST_FILE = "stress_scv50"


@dataclasses.dataclass
class Program:
    """Gcode lines plus the number of trapq moves they plan: kinematic G1s
    that change the toolhead position."""

    pos: tuple
    lines: list = dataclasses.field(default_factory=list)
    moves: int = 0

    def add(self, *lines):
        self.lines.extend(lines)

    def move(self, x, y, speed, z=None, e=None):
        z = self.pos[2] if z is None else z
        words = "G1 X%s Y%s" % (_num(x), _num(y))
        if z != self.pos[2]:
            words += " Z%s" % _num(z)
        if e is not None:
            words += " E%.5f" % e
        self.add(words + " F%d" % round(speed * 60.0))
        target = (round(x, 3), round(y, 3), round(z, 3))
        if target != self.pos:
            self.moves += 1
        self.pos = target

    def relative(self, steps, speed):
        """G91 moves by (dx, dy) each; the block must close on itself."""
        self.add("G91", "G1 F%d" % round(speed * 60.0))
        closure = [0.0, 0.0]
        for step in steps:
            words = ["G1"]
            for i, (axis, value) in enumerate(zip("XY", step)):
                text = _num(value)
                if text != "0":
                    words.append(axis + text)
                    closure[i] += float(text)
            self.add(" ".join(words))
            self.moves += 1
        self.add("G90")
        if any(abs(c) > 1e-9 for c in closure):
            raise AssertionError("relative block drifts by %s" % (closure,))


def _num(value):
    text = ("%.3f" % value).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def _accel():
    return float(accel_residual.config_option(CONFIG, "printer", "max_accel"))


def _lowest_mode():
    return LOWEST_MODE_RATIO * min(accel_residual.shaper_freqs(CONFIG).values())


def _min_window():
    return RING_PERIODS / _lowest_mode()


def leg_length(speed):
    """Cruise CRUISE_MARGIN x the ring window at `speed`, plus room to
    accelerate and decelerate from rest at half max_accel (the
    minimum_cruise_ratio 0.5 accel_to_decel)."""
    return speed * CRUISE_MARGIN * _min_window() + 2.0 * speed**2 / _accel()


def polygon(start, heading_deg, turn_deg, sides, side):
    points, (x, y), heading = [], start, math.radians(heading_deg)
    for _ in range(sides):
        x += side * math.cos(heading)
        y += side * math.sin(heading)
        points.append((x, y))
        heading += math.radians(turn_deg)
    if math.dist(points[-1], start) > 1e-9:
        raise AssertionError("polygon %r does not close" % (turn_deg,))
    points[-1] = start
    return points


def sync(prog):
    x0, y0 = prog.pos[:2]
    for axis, dist, dwell in SYNC_TAPS:
        dx, dy = (dist, 0.0) if axis == "x" else (0.0, dist)
        prog.move(x0 + dx, y0 + dy, SYNC_SPEED)
        prog.move(x0, y0, SYNC_SPEED)
        prog.add("G4 P%d" % dwell)
    prog.add("G4 P%d" % SYNC_SETTLE_MS)


def corner_pattern(prog):
    for name, heading, turn, sides, laps, speeds in CORNER_BLOCKS:
        for speed in speeds:
            prog.add(
                "; %s: %g deg turns at %g mm/s, %d laps"
                % (name, turn, speed, laps)
            )
            points = polygon(
                prog.pos[:2], heading, turn, sides, leg_length(speed)
            )
            for _ in range(laps):
                for x, y in points:
                    prog.move(x, y, speed)


def _unit(deg):
    return math.cos(math.radians(deg)), math.sin(math.radians(deg))


def zigzag_steps(line_deg, length, jog, lines):
    """`lines` passes of `length` along line_deg, jogging `jog` sideways
    (+90 deg) between them, then the same back so the block closes."""
    lx, ly = _unit(line_deg)
    jx, jy = _unit(line_deg + 90.0)
    steps = []
    for direction in (1.0, -1.0):
        for i in range(lines):
            sign = 1.0 if i % 2 == 0 else -1.0
            steps.append((sign * length * lx, sign * length * ly))
            steps.append((direction * jog * jx, direction * jog * jy))
    return steps


def circle_steps(radius, chord, turns):
    sides = max(3, round(2.0 * math.pi * radius / chord))
    points = [
        (
            radius * math.cos(2.0 * math.pi * k / sides) - radius,
            radius * math.sin(2.0 * math.pi * k / sides),
        )
        for k in range(sides + 1)
    ]
    steps = np.diff(np.array(points), axis=0).tolist() * turns
    rounded = [[float(_num(v)) for v in step] for step in steps]
    for i in range(2):
        rounded[-1][i] -= round(sum(s[i] for s in rounded), 3)
    return [tuple(s) for s in rounded]


def reversal_steps(line_deg, length, count):
    lx, ly = _unit(line_deg)
    return [(length * lx, length * ly), (-length * lx, -length * ly)] * count


def stress_blocks():
    """(name, steps, speed) of one stress repetition."""
    xl, xj, xv = ZIGZAG_X
    yl, yj, yv = ZIGZAG_Y
    dl, dj, dv = ZIGZAG_DIAGONAL
    radius, chord, cv, turns = CIRCLE
    rl, rv, rc = REVERSAL
    return (
        ("zigzag_x", zigzag_steps(0.0, xl, xj, ZIGZAG_LINES), xv),
        ("zigzag_y", zigzag_steps(90.0, yl, yj, ZIGZAG_LINES), yv),
        ("zigzag_diag_a", zigzag_steps(45.0, dl, dj, ZIGZAG_LINES), dv),
        ("zigzag_diag_b", zigzag_steps(-45.0, dl, dj, ZIGZAG_LINES), dv),
        ("circle", circle_steps(radius, chord, turns), cv),
        ("reverse_x", reversal_steps(0.0, rl, rc), rv),
        ("reverse_y", reversal_steps(90.0, rl, rc), rv),
    )


def stress_pattern(prog):
    for repeat in range(STRESS_REPEATS):
        for name, steps, speed in stress_blocks():
            prog.add("; %s, repeat %d" % (name, repeat + 1))
            prog.relative(steps, speed)


def _extrude_line(prog, a, b):
    if prog.pos[:2] != (round(a[0], 3), round(a[1], 3)):
        prog.move(*a, POSITION_SPEED)
    prog.move(*b, MARK_SPEED, e=E_PER_MM * math.dist(a, b))


def cross(prog, z):
    cx, cy = MARK_CENTER
    prog.move(cx - CROSS_ARM_MM, cy, POSITION_SPEED, z=z)
    _extrude_line(prog, (cx - CROSS_ARM_MM, cy), (cx + CROSS_ARM_MM, cy))
    _extrude_line(prog, (cx, cy - CROSS_ARM_MM), (cx, cy + CROSS_ARM_MM))


def vernier(prog, pitch, outward):
    """Ticks across X (for X shifts) above the cross and across Y (for Y
    shifts) to its right. The main scale (pitch 1.0) ends at the scale
    boundary, the vernier (pitch 0.9) starts there: vernier tick i lining up
    with a main tick means a shift of i * 0.1 mm. Tick 0 of each is longer."""
    cx, cy = MARK_CENTER
    boundary = (
        CROSS_ARM_MM + VERNIER_INDEX_MM + VERNIER_GAP_MM + VERNIER_TICK_MM
    )
    for i in range(-VERNIER_TICKS, VERNIER_TICKS + 1):
        length = VERNIER_TICK_MM + (VERNIER_INDEX_MM if i == 0 else 0.0)
        near, far = (
            (boundary, boundary + length)
            if outward
            else (boundary - length, boundary)
        )
        x = cx + i * pitch
        _extrude_line(prog, (x, cy + near), (x, cy + far))
        y = cy + i * pitch
        _extrude_line(prog, (cx + near, y), (cx + far, y))


def prime(prog):
    cx, cy = MARK_CENTER
    y = cy - CROSS_ARM_MM - PRIME_GAP_MM
    a = (cx - 0.5 * PRIME_MM, y)
    _extrude_line(prog, a, (cx + 0.5 * PRIME_MM, y))


def _measured_block(prog, name, body):
    prog.add("M400", accel_residual.START_MACRO, "G4 P%d" % QUIET_LEAD_MS)
    first = prog.moves
    sync_end = prog.moves + 2 * len(SYNC_TAPS)
    sync(prog)
    body(prog)
    end = prog.moves
    prog.add(
        "G4 P%d" % QUIET_TAIL_MS,
        "M400",
        "%s NAME=%s" % (accel_residual.STOP_MACRO, name),
    )
    return first, end, sync_end


def _setup(prog, scv):
    prog.add(
        "G90",
        "M83",
        "G1 X%.3f Y%.3f Z%.3f F%d"
        % (prog.pos + (round(POSITION_SPEED * 60.0),)),
    )
    if scv is not None:
        prog.add("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=%g" % scv)


def corners_program(setting):
    scv, _ = CORNER_SETTINGS[setting]
    prog = Program(pos=(CENTER[0], CENTER[1], TRAVEL_Z))
    _setup(prog, scv)
    span = _measured_block(prog, "corners_" + setting, corner_pattern)
    return prog, span


def stress_program(setting):
    scv, _ = CORNER_SETTINGS[setting]
    prog = Program(pos=(CENTER[0], CENTER[1], TRAVEL_Z))
    _setup(prog, scv)
    prog.add("; reference marks before the stress block, layer 1")
    prog.move(*MARK_CENTER, POSITION_SPEED)
    prog.move(
        MARK_CENTER[0],
        MARK_CENTER[1] - CROSS_ARM_MM,
        POSITION_SPEED,
        z=MARK_FIRST_Z,
    )
    prime(prog)
    cross(prog, MARK_FIRST_Z)
    vernier(prog, VERNIER_MAIN_PITCH, outward=False)
    prog.move(prog.pos[0], prog.pos[1], POSITION_SPEED, z=TRAVEL_Z)
    prog.move(*STRESS_CENTER, POSITION_SPEED)
    span = _measured_block(prog, "stress_" + setting, stress_pattern)
    prog.add("; reference marks after: vernier on layer 1, cross on layer 2")
    prog.move(MARK_CENTER[0], MARK_CENTER[1], POSITION_SPEED)
    prog.move(prog.pos[0], prog.pos[1], POSITION_SPEED, z=MARK_FIRST_Z)
    vernier(prog, VERNIER_PITCH, outward=True)
    cross(prog, MARK_SECOND_Z)
    prog.move(prog.pos[0], prog.pos[1], POSITION_SPEED, z=TRAVEL_Z)
    return prog, span


def _headers(prog_start, setting, corner_deviation, measured, sync_s):
    return [
        "; motion_eval start=%.4f,%.4f,%.4f" % prog_start,
        "; motion_eval setting=%s" % setting,
        "; motion_eval corner_deviation=%s" % corner_deviation,
        "; motion_eval measured_moves=%d,%d" % measured,
        "; motion_eval sync_s=%.6f" % sync_s,
    ]


DESCRIPTIONS = {
    "corners": (
        "; Corner/ringing pattern, travel only, at Z %g. Squares, diamonds,"
        % TRAVEL_Z,
        "; octagons (45 deg), stars (135 deg) and reversals; every corner is",
        "; followed by a constant-velocity leg. Needs MOTION_EVAL_ACCEL_START/"
        "STOP",
        "; macros (PRINTER_TEST.md). Prints nothing.",
    ),
    "stress": (
        "; Skipped-step stress: zigzags, polyline circles and reversals at",
        "; the speeds of the largest executed corner bursts. EXTRUDES the",
        "; reference cross and vernier at X%g Y%g: preheat, home and level"
        % MARK_CENTER,
        "; first (PRINTER_TEST.md). Compare the two crosses and read the"
        " verniers.",
    ),
}


def render(kind, setting):
    """Gcode text of `<kind>_<setting>` and its replay. The setting header
    is the file stem, so accel_residual's rows name the file."""
    build = corners_program if kind == "corners" else stress_program
    prog, (first, end, sync_end) = build(setting)
    _, corner_deviation = CORNER_SETTINGS[setting]
    name = "%s_%s" % (kind, setting)
    start = (CENTER[0], CENTER[1], TRAVEL_Z)
    body = "\n".join(prog.lines) + "\n"
    draft = (
        "\n".join(_headers(start, name, corner_deviation, (first, end), 0.0))
        + "\n"
        + body
    )
    raw = accel_residual.replay(draft, CONFIG)
    moves = raw.capture["moves"]
    if len(moves) != prog.moves:
        raise AssertionError(
            "%s: planned %d trapq moves, the generator counted %d"
            % (name, len(moves), prog.moves)
        )
    sync_s = moves[sync_end]["print_time"] - moves[first]["print_time"]
    headers = _headers(start, name, corner_deviation, (first, end), sync_s)
    text = "\n".join(headers + list(DESCRIPTIONS[kind])) + "\n" + body
    return text, raw


def _check_bounds(name, cap):
    for m in cap["moves"]:
        for axis in "xy":
            pos = m["start_" + axis]
            if not BED_MARGIN_MM <= pos <= BED_MM - BED_MARGIN_MM:
                raise AssertionError(
                    "%s: %s %.1f outside the bed margin" % (name, axis, pos)
                )


def _check_windows(name, plan):
    """Each corner after the sync has a ring window of RING_PERIODS
    periods of the lowest mode before the next corner."""
    need = _min_window()
    starts = np.array([w[0] for w in plan.windows])
    lengths = np.array([w[1] - w[0] for w in plan.windows])
    corners = plan.vertices
    nxt = np.append(corners[1:], plan.span[1])
    for t, t_next in zip(corners, nxt):
        sel = (starts > t) & (starts < t_next)
        if not sel.any() or lengths[sel].max() < need:
            raise AssertionError(
                "%s: the corner at t=%.4f has no %.1f ms constant-velocity"
                " window before the next corner" % (name, t, need * 1e3)
            )


def _check_self_alignment(name, plan):
    """The plan, sampled like an ADXL345 at its default rate from the
    measurement start, aligns against itself at the offset it was given."""
    step = 1.0 / SELF_CHECK_RATE_HZ
    t = np.arange(
        plan.span[0] - QUIET_LEAD_MS / 1000.0,
        plan.span[1] + QUIET_TAIL_MS / 1000.0,
        step,
    )
    meas = accel_residual.Measurement(
        t + SELF_CHECK_OFFSET_S, {a: plan.acc_at(a, t) for a in "xy"}
    )
    found = accel_residual.align(meas, plan)
    if (
        abs(found.offset - SELF_CHECK_OFFSET_S)
        > accel_residual.CHUNK_TOLERANCE_SAMPLES * step
    ):
        raise AssertionError(
            "%s: aligned at %.7f against its own plan, expected %.7f"
            % (name, found.offset, SELF_CHECK_OFFSET_S)
        )
    return found


def executed_peaks(cap):
    """Executed peak |a| / max_accel on x, y and the corexy motors
    a = x + y, b = x - y."""
    _, _, _, _, _, motion = evaluate.executed_motion(cap, accel_residual.DT)
    ax, ay = motion["x"].acc, motion["y"].acc
    accel = cap["settings"]["max_accel"]
    return {
        "x": float(np.abs(ax).max() / accel),
        "y": float(np.abs(ay).max() / accel),
        "motor a": float(np.abs(ax + ay).max() / accel),
        "motor b": float(np.abs(ax - ay).max() / accel),
    }


def block_peaks(setting):
    """Executed peaks of each stress block alone, from rest at the
    stress center."""
    scv, corner_deviation = CORNER_SETTINGS[setting]
    out = {}
    for name, steps, speed in stress_blocks():
        prog = Program(pos=(STRESS_CENTER[0], STRESS_CENTER[1], TRAVEL_Z))
        if scv is not None:
            prog.add("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=%g" % scv)
        prog.relative(steps, speed)
        text = (
            "\n".join(
                _headers(
                    prog.pos, setting, corner_deviation, (0, prog.moves), 0.0
                )
                + prog.lines
            )
            + "\n"
        )
        out[name] = executed_peaks(accel_residual.replay(text, CONFIG).capture)
    return out


def generate(out):
    out.mkdir(parents=True, exist_ok=True)
    freqs = accel_residual.shaper_freqs(CONFIG)
    report = []
    jobs = [("corners", s) for s in CORNER_SETTINGS] + [
        ("stress", s) for s in STRESS_SETTINGS
    ]
    for kind, setting in jobs:
        name = "%s_%s" % (kind, setting)
        text, raw = render(kind, setting)
        _check_bounds(name, raw.capture)
        plan = accel_residual.plan_from_capture(
            raw.capture, accel_residual.parse_headers(text), freqs
        )
        if kind == "corners":
            _check_windows(name, plan)
        found = _check_self_alignment(name, plan)
        peaks = executed_peaks(raw.capture)
        if name == STRESS_BURST_FILE and (
            max(peaks["x"], peaks["y"]) < STRESS_PEAK_EXPECTED
        ):
            raise AssertionError(
                "%s: executed peak x %.3f y %.3f of max_accel, below the"
                " %.2f it is meant to reach"
                % (name, peaks["x"], peaks["y"], STRESS_PEAK_EXPECTED)
            )
        bounded_limit = (
            BOUNDED_CORNER_ACCEL
            * (1.0 + ACCEL_MARGIN)
            / raw.capture["settings"]["max_accel"]
        )
        if name in BOUNDED_FILES and (
            max(peaks["x"], peaks["y"]) > bounded_limit
        ):
            raise AssertionError(
                "%s: executed peak x %.3f y %.3f of max_accel, above the"
                " %.4f that max_corner_accel %d plus ACCEL_MARGIN allows"
                % (
                    name,
                    peaks["x"],
                    peaks["y"],
                    bounded_limit,
                    BOUNDED_CORNER_ACCEL,
                )
            )
        (out / (name + ".gcode")).write_text(text)
        report.append(
            "%-18s %6.2f s  corners %4d  windows %4d  sync %.2f/%.2f  peak"
            " x %.3f y %.3f motor a %.3f b %.3f"
            % (
                name,
                plan.span[1] - plan.span[0],
                len(plan.vertices),
                len(plan.windows),
                found.sync_peak,
                found.second_peak,
                peaks["x"],
                peaks["y"],
                peaks["motor a"],
                peaks["motor b"],
            )
        )
    for setting in STRESS_SETTINGS:
        for block, peaks in block_peaks(setting).items():
            report.append(
                "stress_%-11s %-14s peak x %.3f y %.3f motor a %.3f b %.3f"
                % (
                    setting,
                    block,
                    peaks["x"],
                    peaks["y"],
                    peaks["motor a"],
                    peaks["motor b"],
                )
            )
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m tools.motion_eval.printer_test"
    )
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    args = ap.parse_args(argv)
    sys.stdout.write("\n".join(generate(args.out)) + "\n")


if __name__ == "__main__":
    main()
