"""Property test of [corner_deviation] over random gcode.

    python3 ~/Developer/serval/scripts/slots.py run -- \
        uv run python -m tools.motion_eval.fuzz --seeds 0:10000 [--jobs 12]

Each seed generates a gcode program: segments from 1 um to 200 mm, exact
reversals, collinear runs, arcs, Z hops, extrude-only moves and retracts,
G92, SET_VELOCITY_LIMIT, M204 and SET_INPUT_SHAPER mid-print (smoothers,
pulse shapers and off), on owner.cfg, bench.cfg or fast.cfg. It runs the
default planner (no limiter), [corner_deviation] in floor mode, bounded
by max_corner_accel, and floor mode again with forced and with lazy
lookahead flushes, and checks:

- every run completes, every trapq move is finite with positive duration,
  and the executed motion matches the decoded steps (the harness check);
- each corner of the shaped path rounds at most the shaper floor
  max_accel * hypot(sigma_x^2, sigma_y^2) / 2 of the moves around it;
- the trapq accels and the final step positions equal the default run's;
- executed peak acceleration per axis <= what the limiter admits
  (CornerAccel.worst, cluster margin included, in floor mode; within
  ACCEL_MARGIN of max_corner_accel in the bounded run);
- lazy flushes at random moves leave the trapq and the junction limits
  identical; forced flushes (M400) leave every junction limit not at a
  flush identical and every move at most as fast.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import functools
import json
import math
import os
import pathlib
import random
import re
import sys
import tempfile

import numpy as np

from klippy.extras import corner_deviation

from . import analysis, evaluate, run

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE / ".build" / "fuzz"
CONFIGS = (
    ("owner", HERE / "configs" / "owner.cfg", 0.6),
    ("bench", HERE / "configs" / "bench.cfg", 0.2),
    ("fast", HERE / "configs" / "fast.cfg", 0.2),
)
BED = (5.0, 295.0)
Z_RANGE = (5.0, 40.0)
MIN_SEGMENT_MM = 1e-3
MAX_SEGMENT_MM = 200.0
SPEED_MM_S = (20.0, 1000.0)
MAX_SEGMENT_S = 0.15
ACCEL_MM_S2 = (500.0, 60000.0)
SHAPER_FREQ_HZ = (25.0, 35.0, 50.0, 70.0, 100.0, 130.0, 160.0, 200.0, 220.0)
SHAPERS = ("smooth_mzv", "smooth_ei", "smooth_zv", "mzv", "ei", "zv")
EXTRUDE_PER_MM = 0.033
DT = 10e-6
FLOAT_TOLERANCE = 1e-9
APPROACH_MM = (50.0, 200.0)
APPROACH_SPEED_MM_S = (300.0, 1000.0)
CHORD_MM = (0.02, 1.0)
CHORD_TURN_DEG = (2.0, 25.0)
MICRO_MM = (0.01, 0.5)
CLUSTER_MOVES = (3, 40)
CLUSTERS = (3, 12)
CORNER_ACCEL_RATIO = 1.03
TABLE_CACHE_DIR = os.path.expanduser(corner_deviation.TABLE_CACHE_DIR)


@dataclasses.dataclass
class Case:
    seed: int
    config: str
    gcode: str
    start: tuple
    lazy_flush_at: tuple
    forced_flush_lines: tuple


class Program:
    def __init__(self, rng, start):
        self.rng = rng
        self.lines = []
        self.pos = list(start)
        self.offset = [0.0, 0.0, 0.0]
        self.heading = rng.uniform(0.0, 2.0 * math.pi)
        self.feed = rng.uniform(*SPEED_MM_S)

    def emit(self, line):
        self.lines.append(line)

    def logical(self, axis):
        return self.pos[axis] - self.offset[axis]

    def speed(self):
        if self.rng.random() < 0.6:
            lo, hi = SPEED_MM_S
            self.feed = math.exp(self.rng.uniform(math.log(lo), math.log(hi)))
        return " F%.3f" % (60.0 * self.feed,)

    def length(self):
        lo, hi = MIN_SEGMENT_MM, min(MAX_SEGMENT_MM, MAX_SEGMENT_S * self.feed)
        return math.exp(self.rng.uniform(math.log(lo), math.log(max(lo, hi))))

    def turn(self):
        r = self.rng.random()
        if r < 0.25:
            return 0.0
        if r < 0.35:
            return math.pi
        if r < 0.55:
            return (
                self.rng.choice((1, -1))
                * self.rng.choice((45, 90, 135))
                * (math.pi / 180.0)
            )
        if r < 0.75:
            return self.rng.gauss(0.0, 0.2)
        return self.rng.uniform(-math.pi, math.pi)

    def target(self, heading, length):
        x = self.pos[0] + length * math.cos(heading)
        y = self.pos[1] + length * math.sin(heading)
        if not (BED[0] <= x <= BED[1] and BED[0] <= y <= BED[1]):
            heading += math.pi
            x = self.pos[0] + length * math.cos(heading)
            y = self.pos[1] + length * math.sin(heading)
        x = min(max(x, BED[0]), BED[1])
        y = min(max(y, BED[0]), BED[1])
        return heading, x, y

    def line(self, extrude):
        self.heading, x, y = self.target(
            self.heading + self.turn(), self.length()
        )
        d = math.hypot(x - self.pos[0], y - self.pos[1])
        self.pos[0], self.pos[1] = x, y
        e = " E%.5f" % (EXTRUDE_PER_MM * d,) if extrude else ""
        self.emit(
            "G1 X%.6f Y%.6f%s%s"
            % (self.logical(0), self.logical(1), e, self.speed())
        )

    def zigzag(self):
        length = (
            self.length()
            if self.rng.random() < 0.5
            else self.rng.uniform(MIN_SEGMENT_MM, 0.1)
        )
        for _ in range(self.rng.randint(2, 12)):
            self.heading, x, y = self.target(self.heading + math.pi, length)
            self.pos[0], self.pos[1] = x, y
            self.emit("G1 X%.6f Y%.6f" % (self.logical(0), self.logical(1)))

    def arc(self):
        radius = self.rng.uniform(0.5, 30.0)
        sweep_angle = self.rng.uniform(0.1, 1.9 * math.pi)
        clockwise = self.rng.random() < 0.5
        side = -1.0 if clockwise else 1.0
        center_heading = self.heading + side * 0.5 * math.pi
        cx = self.pos[0] + radius * math.cos(center_heading)
        cy = self.pos[1] + radius * math.sin(center_heading)
        start_angle = math.atan2(self.pos[1] - cy, self.pos[0] - cx)
        end_angle = start_angle + (-sweep_angle if clockwise else sweep_angle)
        lo, hi = BED[0] + radius, BED[1] - radius
        if not (lo <= cx <= hi and lo <= cy <= hi):
            return self.line(False)
        x = cx + radius * math.cos(end_angle)
        y = cy + radius * math.sin(end_angle)
        i, j = cx - self.pos[0], cy - self.pos[1]
        self.pos[0], self.pos[1] = x, y
        self.heading = end_angle + (
            -0.5 * math.pi if clockwise else 0.5 * math.pi
        )
        self.emit(
            "%s X%.6f Y%.6f I%.6f J%.6f%s"
            % (
                "G2" if clockwise else "G3",
                self.logical(0),
                self.logical(1),
                i,
                j,
                self.speed(),
            )
        )

    def z_hop(self):
        hop = self.rng.uniform(0.05, 2.0)
        z = min(self.pos[2] + hop, Z_RANGE[1])
        self.emit("G1 Z%.4f F%.1f" % (z - self.offset[2], 60.0 * 15.0))
        self.pos[2] = z
        self.line(False)
        z = max(z - hop, Z_RANGE[0])
        self.emit("G1 Z%.4f F%.1f" % (z - self.offset[2], 60.0 * 15.0))
        self.pos[2] = z

    def extruder(self):
        if self.rng.random() < 0.5:
            self.emit("G1 E-%.4f F2400" % (self.rng.uniform(0.1, 2.0),))
            self.line(False)
            self.emit("G1 E%.4f F2400" % (self.rng.uniform(0.1, 2.0),))
        else:
            self.emit("G1 E%.4f F1800" % (self.rng.uniform(0.01, 5.0),))

    def g92(self):
        if self.rng.random() < 0.5:
            self.emit("G92 E0")
            return
        shift = [self.rng.uniform(-50.0, 50.0) for _ in range(2)]
        for axis in (0, 1):
            self.offset[axis] = self.pos[axis] - (
                self.logical(axis) + shift[axis]
            )
        self.emit("G92 X%.6f Y%.6f" % (self.logical(0), self.logical(1)))

    def limits(self):
        lo, hi = ACCEL_MM_S2
        accel = math.exp(self.rng.uniform(math.log(lo), math.log(hi)))
        if self.rng.random() < 0.5:
            self.emit("M204 S%.1f" % (accel,))
            return
        self.emit(
            "SET_VELOCITY_LIMIT VELOCITY=%.1f ACCEL=%.1f MINIMUM_CRUISE_RATIO=%.2f"
            % (
                self.rng.uniform(50.0, 1200.0),
                accel,
                self.rng.choice((0.0, 0.25, 0.5, self.rng.uniform(0.0, 0.95))),
            )
        )

    def approach(self):
        """A long fast move: the cluster after it is entered decelerating."""
        self.heading, x, y = self.target(
            self.heading + self.turn(), self.rng.uniform(*APPROACH_MM)
        )
        self.pos[0], self.pos[1] = x, y
        self.feed = self.rng.uniform(*APPROACH_SPEED_MM_S)
        self.emit(
            "G1 X%.6f Y%.6f F%.3f"
            % (self.logical(0), self.logical(1), 60.0 * self.feed)
        )

    def polyline_arc(self):
        """Short chords turning the same way, as a slicer's small arcs."""
        chord = math.exp(self.rng.uniform(*map(math.log, CHORD_MM)))
        step = self.rng.choice((1, -1)) * math.radians(
            self.rng.uniform(*CHORD_TURN_DEG)
        )
        speed = self.speed() if self.rng.random() < 0.5 else ""
        for i in range(self.rng.randint(*CLUSTER_MOVES)):
            self.heading, x, y = self.target(self.heading + step, chord)
            self.pos[0], self.pos[1] = x, y
            self.emit(
                "G1 X%.6f Y%.6f%s"
                % (self.logical(0), self.logical(1), speed if i == 0 else "")
            )

    def micro_cluster(self):
        """Very short moves turning at random."""
        for _ in range(self.rng.randint(*CLUSTER_MOVES)):
            length = math.exp(self.rng.uniform(*map(math.log, MICRO_MM)))
            self.heading, x, y = self.target(
                self.heading + self.rng.uniform(-math.pi, math.pi), length
            )
            self.pos[0], self.pos[1] = x, y
            self.emit("G1 X%.6f Y%.6f" % (self.logical(0), self.logical(1)))

    def shaper(self):
        if self.rng.random() < 0.15:
            self.emit("SET_INPUT_SHAPER SHAPER_FREQ_X=0 SHAPER_FREQ_Y=0")
            return
        self.emit(
            "SET_INPUT_SHAPER SHAPER_TYPE=%s SHAPER_FREQ_X=%.2f SHAPER_FREQ_Y=%.2f"
            % (
                self.rng.choice(SHAPERS),
                self.rng.choice(SHAPER_FREQ_HZ),
                self.rng.choice(SHAPER_FREQ_HZ),
            )
        )


STEPS = (
    ("line", 0.35),
    ("extrude", 0.2),
    ("zigzag", 0.07),
    ("arc", 0.1),
    ("z_hop", 0.04),
    ("extruder", 0.05),
    ("g92", 0.03),
    ("limits", 0.06),
    ("shaper", 0.04),
    ("dwell", 0.02),
    ("relative", 0.04),
)


def generate(seed):
    rng = random.Random(seed)
    names, paths, weights = zip(*CONFIGS)
    config = rng.choices(names, weights)[0]
    start = (
        rng.uniform(*BED),
        rng.uniform(*BED),
        rng.uniform(*Z_RANGE),
    )
    prog = Program(rng, start)
    prog.emit("M83")
    for _ in range(rng.randint(5, 60)):
        step = rng.choices([s for s, _ in STEPS], [w for _, w in STEPS])[0]
        if step == "line":
            prog.line(False)
        elif step == "extrude":
            prog.line(True)
        elif step == "dwell":
            prog.emit("G4 P%d" % (rng.randint(0, 50),))
        elif step == "relative":
            prog.heading, x, y = prog.target(
                prog.heading + prog.turn(), prog.length()
            )
            prog.emit("G91")
            prog.emit(
                "G1 X%.6f Y%.6f%s"
                % (x - prog.pos[0], y - prog.pos[1], prog.speed())
            )
            prog.emit("G90")
            prog.pos[0], prog.pos[1] = x, y
        else:
            getattr(prog, step)()
    lines = prog.lines
    move_lines = [
        i for i, line in enumerate(lines) if line.startswith(("G1", "G2", "G3"))
    ]
    forced = tuple(
        sorted(rng.sample(move_lines, min(len(move_lines), rng.randint(1, 4))))
    )
    lazy = tuple(sorted(rng.sample(range(400), rng.randint(1, 12))))
    return Case(
        seed,
        config,
        "\n".join(lines) + "\n",
        start,
        lazy,
        forced,
    )


def generate_clusters(seed):
    """Corner clusters within one kernel reach (short-chord arcs and
    micro-move clusters), each entered decelerating from a long fast move,
    under a random shaper and accel."""
    rng = random.Random(seed)
    names, paths, weights = zip(*CONFIGS)
    config = rng.choices(names, weights)[0]
    start = tuple(rng.uniform(*BED) for _ in range(2)) + (
        rng.uniform(*Z_RANGE),
    )
    prog = Program(rng, start)
    if rng.random() < 0.7:
        prog.shaper()
    if rng.random() < 0.5:
        prog.limits()
    for _ in range(rng.randint(*CLUSTERS)):
        prog.approach()
        rng.choice((prog.polyline_arc, prog.micro_cluster))()
    lines = prog.lines
    move_lines = [i for i, line in enumerate(lines) if line.startswith("G1")]
    return Case(
        seed,
        config,
        "\n".join(lines) + "\n",
        start,
        tuple(sorted(rng.sample(range(400), rng.randint(1, 12)))),
        tuple(
            sorted(
                rng.sample(move_lines, min(len(move_lines), rng.randint(1, 4)))
            )
        ),
    )


GENERATORS = {"random": generate, "clusters": generate_clusters}


def _config(case, tmpdir, section):
    base = dict((n, p) for n, p, _ in CONFIGS)[case.config]
    if section is None:
        return base
    path = pathlib.Path(tempfile.mkstemp(suffix=".cfg", dir=tmpdir)[1])
    path.write_text("[include %s]\n\n%s" % (base, section))
    return path


def _config_max_accel(case):
    base = dict((n, p) for n, p, _ in CONFIGS)[case.config]
    (value,) = re.findall(r"^max_accel:\s*(\S+)", base.read_text(), re.M)
    return float(value)


def _corner_accel(case):
    """The bounded run bounds the corner burst, CORNER_ACCEL_RATIO above
    the config's max_accel; the floor run leaves it unbounded."""
    return CORNER_ACCEL_RATIO * _config_max_accel(case)


def _sections(case):
    return {
        "floor": "[corner_deviation]\n",
        "bounded": "[corner_deviation]\nmax_corner_accel: %r\n"
        % (_corner_accel(case),),
    }


def _with_forced_flushes(case):
    lines = case.gcode.splitlines()
    out = []
    for i, line in enumerate(lines):
        out.append(line)
        if i in case.forced_flush_lines:
            out.append("M400")
    return "\n".join(out) + "\n"


def _run(case, tmpdir, section, gcode=None, env=None):
    return run.run_klippy(
        _config(case, tmpdir, section),
        case.gcode if gcode is None else gcode,
        case.start,
        env=env,
    )


class Violations(list):
    def check(self, ok, fmt, *args):
        if not ok:
            self.append(fmt % args)


def _move_end(m):
    return m["print_time"] + m["accel_t"] + m["cruise_t"] + m["decel_t"]


def _check_moves(v, name, moves):
    for i, m in enumerate(moves):
        values = [m[k] for k in m if isinstance(m[k], float)]
        v.check(
            all(math.isfinite(x) for x in values),
            "%s: move %d has non-finite fields %r",
            name,
            i,
            m,
        )
        v.check(
            m["accel_t"] + m["cruise_t"] + m["decel_t"] > 0.0,
            "%s: move %d has zero duration",
            name,
            i,
        )


def _window_max(moves, values, lo, hi):
    """Per window [lo[k], hi[k]], the largest value of the moves
    overlapping it (0 where none does)."""
    starts = np.array([m["print_time"] for m in moves])
    ends = np.array([_move_end(m) for m in moves])
    out = np.zeros(len(lo))
    for k in range(len(lo)):
        near = (starts <= hi[k]) & (ends >= lo[k])
        out[k] = values[near].max(initial=0.0)
    return out


def _painted_max(moves, values, t0, n, reach):
    """Per sample t0 + DT*i, the largest value of the moves overlapping
    [t - reach, t + reach]."""
    out = np.zeros(n)
    for m, value in zip(moves, values):
        i0 = max(0, int(math.floor((m["print_time"] - reach - t0) / DT)))
        i1 = min(n, int(math.ceil((_move_end(m) + reach - t0) / DT)) + 1)
        np.maximum(out[i0:i1], value, out=out[i0:i1])
    return out


def _floor_per_unit_accel(epoch):
    return 0.5 * math.hypot(
        epoch.kernels["x"].second_moment(), epoch.kernels["y"].second_moment()
    )


def _shaper_spec(state):
    """corner_deviation.shaper_spec of a captured shaper state."""
    if state["kind"] == "pulses":
        if not state["A"]:
            return None
        return ("pulses", tuple(state["A"]), tuple(state["T"]))
    if not state["smooth_time"]:
        return None
    return ("smoother", tuple(state["coeffs"]), state["smooth_time"])


def _admitted_accel(capture, corner_accel):
    """Per move, the worst executed (x, y) acceleration the limiter admits
    at the move's accel under its shaper epoch (corner_deviation's
    CornerAccel; a stop at every corner without shaping)."""
    moves = capture["moves"]
    max_velocity = max(m["max_cruise_v"] for m in moves)
    models, worst = {}, {}
    out = np.zeros((len(moves), 2))
    for i, m in enumerate(moves):
        epoch = m["shaper_epoch"]
        if epoch not in models:
            state = capture["shapers"][epoch]
            specs = tuple(_shaper_spec(state[axis]) for axis in "xy")
            models[epoch] = None
            if None not in specs:
                kx, ky = (corner_deviation.kernel_from_spec(s) for s in specs)
                arrays = corner_deviation.table_arrays(
                    specs, kx, ky, TABLE_CACHE_DIR
                )
                models[epoch] = corner_deviation.CornerAccel(arrays, kx, ky)
        key = (epoch, m["accel"])
        if key not in worst:
            model = models[epoch]
            worst[key] = (
                (m["accel"], m["accel"])
                if model is None
                else model.worst(m["accel"], max_velocity, corner_accel)
            )
        out[i] = worst[key]
    return out


def _check_shaped(v, name, raw, corner_accel):
    """A vertex's deviation is measured over [t_arrive - W, t_leave + W]
    (W the kernel reach), and the shaped path there depends on the moves
    within another W: the floor and the stop exemption look that far."""
    result = evaluate.analyze(raw, dt=DT, sample_dt=None)
    moves = raw.capture["moves"]
    seg, epochs, t0, n, _, motion = evaluate.executed_motion(raw.capture, DT)
    vertices = result["executed"]["vertices"]
    t_arrive = np.array(vertices["t_arrive"])
    t_leave = np.array(vertices["t_leave"])
    deviation = np.array(vertices["deviation_mm"])
    reach = 2.0 * max(
        max(abs(s) for s in evaluate.kernel_support(e.kernels)) for e in epochs
    )
    lo, hi = t_arrive - reach, t_leave + reach
    floor_per_accel = np.zeros(len(moves))
    for e in epochs:
        floor_per_accel[e.first : e.last] = _floor_per_unit_accel(e)
    accel = np.array([m["accel"] for m in moves])
    max_accel = np.array([m["max_accel"] for m in moves])
    floor_near = _window_max(moves, floor_per_accel * accel, lo, hi)
    grid = 0.5 * _window_max(moves, max_accel, lo, hi) * DT**2 + FLOAT_TOLERANCE
    metrics = {
        "floor_ratio": float(
            (deviation / np.where(floor_near > 0.0, floor_near, np.inf)).max(
                initial=0.0
            )
        )
    }
    for k in np.flatnonzero(deviation > floor_near + grid):
        v.check(
            False,
            "%s: corner at t=%.6f rounds %.6f mm > shaper floor %.6f mm",
            name,
            t_arrive[k],
            deviation[k],
            floor_near[k],
        )
    t = t0 + DT * np.arange(n)
    near = _painted_max(moves, max_accel, t0, n, reach)
    admitted = _admitted_accel(raw.capture, corner_accel)
    admitted_near = {
        axis: _painted_max(moves, admitted[:, i], t0, n, reach)
        for i, axis in enumerate("xy")
    }
    admitted_near["z"] = near
    margin = (
        1.0 if corner_accel is None else 1.0 + corner_deviation.ACCEL_MARGIN
    )
    slack = FLOAT_TOLERANCE * near.max()
    metrics["accel_ratio"] = 0.0
    metrics["admitted_ratio"] = 0.0
    for axis in "xyz":
        acc = np.abs(motion[axis].acc)
        limit = margin * admitted_near[axis]
        metrics["accel_ratio"] = max(
            metrics["accel_ratio"],
            float((acc / np.where(near > 0.0, near, np.inf)).max(initial=0.0)),
        )
        positive = admitted_near[axis] > 0.0
        metrics["admitted_ratio"] = max(
            metrics["admitted_ratio"],
            float(
                (acc / np.where(positive, admitted_near[axis], np.inf)).max(
                    initial=0.0
                )
            ),
        )
        v.check(
            bool(np.isfinite(acc).all()), "%s: non-finite executed accel", name
        )
        over = np.flatnonzero(acc > limit + slack)
        if len(over):
            k = over[np.argmax(acc[over] - limit[over])]
            v.check(
                False,
                "%s: executed %s accel %.1f > %.1f at t=%.6f",
                name,
                axis,
                acc[k],
                limit[k],
                t[k],
            )
    return result, metrics


def _final_positions(raw):
    return {
        name: st["final_mcu_position"]
        for name, st in raw.capture["steppers"].items()
    }


def _junctions(raw):
    """add_move index -> (toolhead limit passed in, limiter result)."""
    return {int(i): (cap, v2) for i, cap, v2 in raw.capture["junctions"]}


def _same_own_limit(a, b):
    """Both runs' results equal min(cap, L) for one limiter limit L."""
    (cap_a, v_a), (cap_b, v_b) = a, b
    if v_a < cap_a and v_b < cap_b:
        return v_a == v_b
    if v_a < cap_a:
        return v_b == min(cap_b, v_a)
    if v_b < cap_b:
        return v_a == min(cap_a, v_b)
    return True


def check(case):
    v = Violations()
    out = {"seed": case.seed, "config": case.config}
    with tempfile.TemporaryDirectory(prefix="motion_eval_fuzz_") as tmp:
        runs = {}
        for name, section in (("default", None),) + tuple(
            _sections(case).items()
        ):
            try:
                runs[name] = _run(case, tmp, section)
            except run.KlippyRunError as e:
                v.check(False, "%s: klippy failed: %s", name, str(e)[-3000:])
        if "default" not in runs:
            out["violations"] = v
            return out
        default_moves = runs["default"].capture["moves"]
        for name, raw in runs.items():
            moves = raw.capture["moves"]
            _check_moves(v, name, moves)
            if name == "default":
                continue
            try:
                result, metrics = _check_shaped(
                    v,
                    name,
                    raw,
                    _corner_accel(case) if name == "bounded" else None,
                )
            except analysis.ModelMismatch as e:
                v.check(False, "%s: harness model mismatch: %s", name, e)
                continue
            out[name] = {
                "print_time_s": result["print_time_s"],
                "corner_max_mm": result["executed"]["corners"]["max_mm"],
                **metrics,
            }
            v.check(
                [m["accel"] for m in moves]
                == [m["accel"] for m in default_moves],
                "%s: trapq accels differ from the default run",
                name,
            )
            v.check(
                _final_positions(raw) == _final_positions(runs["default"]),
                "%s: final step positions %r != default %r",
                name,
                _final_positions(raw),
                _final_positions(runs["default"]),
            )
        if "floor" in runs:
            out["flushes"] = check_flushes(
                v, case, tmp, runs["floor"], runs["default"]
            )
    out["violations"] = v
    return out


MCR_SETTING = re.compile(r" MINIMUM_CRUISE_RATIO=[0-9.e+-]+")


def without_cruise_ratio(case):
    """The case with minimum_cruise_ratio 0 throughout. Klipper's own
    lookahead (no limiter) lets moves after a forced stop cruise faster
    with a ratio above 0 (fuzz seed 64: 140 moves), so the forced-flush
    speeds are compared at 0, where a stop can only slow the plan."""
    gcode = "SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=0\n" + MCR_SETTING.sub(
        "", case.gcode
    )
    return dataclasses.replace(
        case,
        gcode=gcode,
        forced_flush_lines=tuple(i + 1 for i in case.forced_flush_lines),
    )


def _full_flushes(raw):
    """add_move counts at which the lookahead was flushed to a stop before
    the end of the program (M400, G4, input shaper changes, or Klipper's
    own runout flush when step generation catches up with the plan)."""
    end = raw.capture["added_moves"]
    return {
        added
        for added, lazy, count in raw.capture["flushes"]
        if not lazy and count and added < end
    }


def _profiles(raw):
    """Every trapq move without its start time: a flush that moves the
    plan in time but not in shape (print_time resynced at a different
    moment) leaves this unchanged."""
    return [
        {k: x for k, x in m.items() if k != "print_time"}
        for m in raw.capture["moves"]
    ]


def _check_junctions(v, label, raw, base):
    """Every junction both runs computed has the same limiter limit; a
    junction only one of them computed sits at a stop of the other."""
    junctions, base_junctions = _junctions(raw), _junctions(base)
    stops, base_stops = _full_flushes(raw), _full_flushes(base)
    for index, limits in junctions.items():
        if index not in base_junctions:
            v.check(
                index in base_stops,
                "%s: junction %d computed only here, not at a batch stop",
                label,
                index,
            )
            continue
        v.check(
            _same_own_limit(limits, base_junctions[index]),
            "%s: junction %d (toolhead cap, result) %r vs batch %r",
            label,
            index,
            limits,
            base_junctions[index],
        )
    for index in set(base_junctions) - set(junctions):
        v.check(
            index in stops,
            "%s: junction %d skipped without a stop",
            label,
            index,
        )


def check_flushes(v, case, tmp, floor=None, default=None):
    """Floor mode with lazy flushes after case.lazy_flush_at moves, and
    with M400 after case.forced_flush_lines, against the plain batch run
    (planned here unless given). Junction limits: identical wherever both
    runs compute the junction. Lazy flushes: identical trapq unless a lazy
    flush left so little planned that Klipper's runout check stopped the
    toolhead, or the default planner's own trapq changes with the same
    flushes (minimum_cruise_ratio smoothing sees a different window; the
    junction check still holds the limiter to identical limits). Forced
    flushes, at minimum_cruise_ratio 0: no move faster."""
    if floor is None:
        floor = _run(case, tmp, _sections(case)["floor"])
    if default is None:
        default = _run(case, tmp, None)
    lazy_env = {
        "MOTION_EVAL_LAZY_FLUSH_AT": ",".join(map(str, case.lazy_flush_at))
    }
    lazy = _run(case, tmp, _sections(case)["floor"], env=lazy_env)
    default_lazy = _run(case, tmp, None, env=lazy_env)
    label = "lazy flush at %r" % (case.lazy_flush_at,)
    _check_junctions(v, label, lazy, floor)
    extra_stops = _full_flushes(lazy) - _full_flushes(floor)
    upstream_invariant = _profiles(default_lazy) == _profiles(default)
    info = {
        "lazy_runout_stops": len(extra_stops),
        "upstream_lazy_differs": not upstream_invariant,
    }
    if not extra_stops and upstream_invariant:
        v.check(
            _profiles(lazy) == _profiles(floor),
            "%s: trapq differs from the batch run with no extra stop, while"
            " the default planner's does not",
            label,
        )
    flat = without_cruise_ratio(case)
    flat_base = _run(flat, tmp, _sections(case)["floor"])
    base_moves = flat_base.capture["moves"]
    forced = _run(
        flat, tmp, _sections(case)["floor"], gcode=_with_forced_flushes(flat)
    )
    moves = forced.capture["moves"]
    v.check(
        len(moves) == len(base_moves),
        "forced flush: %d moves != %d",
        len(moves),
        len(base_moves),
    )
    if len(moves) != len(base_moves):
        return info
    _check_junctions(v, "forced flush", forced, flat_base)
    for i, (m, b) in enumerate(zip(moves, base_moves)):
        for key in (
            "start_x",
            "start_y",
            "start_z",
            "axes_r_x",
            "axes_r_y",
            "accel",
        ):
            v.check(
                m[key] == b[key], "forced flush: move %d %s differs", i, key
            )
        for key in ("start_v", "cruise_v"):
            v.check(
                m[key] <= b[key] * (1.0 + FLOAT_TOLERANCE) + FLOAT_TOLERANCE,
                "forced flush: move %d %s %r faster than batch %r",
                i,
                key,
                m[key],
                b[key],
            )
    return info


def load_case(path):
    """A Case frozen as JSON (tests/fuzz_cases), so a counterexample stays
    reproducible when the generator changes."""
    data = json.loads(pathlib.Path(path).read_text())
    return Case(
        data["seed"],
        data["config"],
        data["gcode"],
        tuple(data["start"]),
        tuple(data["lazy_flush_at"]),
        tuple(data["forced_flush_lines"]),
    )


def parse_seeds(text):
    lo, hi = text.split(":")
    return range(int(lo), int(hi))


def summary(results):
    """Largest floor/accel ratio per mode and flush counts."""
    lines = []
    for mode in ("floor", "bounded"):
        for metric in (
            "floor_ratio",
            "accel_ratio",
            "admitted_ratio",
        ):
            values = [
                (r[mode][metric], r["seed"])
                for r in results
                if metric in r.get(mode, {})
            ]
            if values:
                value, seed = max(values)
                lines.append(
                    "%s %s max %.4f (seed %d)" % (mode, metric, value, seed)
                )
    flushes = [r["flushes"] for r in results if "flushes" in r]
    lines.append(
        "lazy flush cases %d: with runout stops %d, default planner"
        " not invariant %d"
        % (
            len(flushes),
            sum(1 for f in flushes if f["lazy_runout_stops"]),
            sum(1 for f in flushes if f["upstream_lazy_differs"]),
        )
    )
    return "\n".join(lines)


def _check_seed(generator, seed):
    return check(GENERATORS[generator](seed))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.motion_eval.fuzz")
    ap.add_argument("--seeds", type=parse_seeds, default=parse_seeds("0:100"))
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    ap.add_argument("--generator", choices=sorted(GENERATORS), default="random")
    args = ap.parse_args(argv)
    generate_case = GENERATORS[args.generator]
    args.out.mkdir(parents=True, exist_ok=True)
    results = []
    with concurrent.futures.ProcessPoolExecutor(args.jobs) as pool:
        for i, res in enumerate(
            pool.map(
                functools.partial(_check_seed, args.generator),
                args.seeds,
                chunksize=4,
            ),
            1,
        ):
            results.append(res)
            if res["violations"]:
                case = dataclasses.asdict(generate_case(res["seed"]))
                (args.out / ("seed%d.json" % res["seed"])).write_text(
                    json.dumps(case, indent=1) + "\n"
                )
                print(
                    "seed %d (%s): %s"
                    % (res["seed"], res["config"], res["violations"][0]),
                    file=sys.stderr,
                )
            if i % 100 == 0:
                print("%d/%d cases" % (i, len(args.seeds)), file=sys.stderr)
    (args.out / "results.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in results)
    )
    failed = [r for r in results if r["violations"]]
    print("%d cases, %d with violations" % (len(results), len(failed)))
    print(summary(results))
    for r in failed:
        print(
            "  seed %d (%s): %d violations, first: %s"
            % (
                r["seed"],
                r["config"],
                len(r["violations"]),
                r["violations"][0][:500],
            )
        )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
