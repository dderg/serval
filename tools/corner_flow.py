#!/usr/bin/env python3
"""Bead-width quality of an extruder chain at corners, from the real planner.

Plans a fixed set of moves (45/90/135 degree corners, a 2 mm zigzag, a
perimeter with a 1.5x flow side, and one entered and left by travel at
speed) under a printer config with the snapshot engine, then reads the
exact planned x, y and e tracks. The commanded e track is passed through a first-order melt
model (``e_cmd = e_out + K * d(e_out)/dt``, K defaulting to the configured
linear pressure advance), and the material leaving the nozzle is measured per
millimetre of the *shaped* nozzle path. Ideal flow lays the commanded bead
everywhere: deviation 0. The commanded bead at each point is the one the
G-code asks for where the unshaped plan passes the same place: the two
plans are paired at every vertex of the program, since the lag between
them drifts along the path. A corner where the bead ratio steps (travel to
extrusion, a flow change, a retract's rest) is read as the material the
switch moves across the vertex and the path it takes, not as over-fill.

    make -f Makefile.rust motion-engine-fast
    python3 tools/corner_flow.py tools/bench/printer.cfg
    python3 tools/corner_flow.py printer.cfg --set pa.k=0.04 --json
    python3 tools/corner_flow.py kernel.cfg --kernel-free
    python3 tools/corner_flow.py kernel.cfg --sweep smooth_time=0,0.005,0.02

``--kernel-free`` evaluates a config that carries an extruder kernel override
with its extruder smoothing kernels taken out, leaving the recommended chain:
every follower ramps its extrusion-ratio steps by the ramp time its limits
derive, kernel or not. An advance with no kernel
needs extrude-only caps, and the retract case is only comparable between
chains under the same ones: ``--extrude-only V,A`` fills in whichever the
config lacks, for every chain.

Each case plans in a process of its own: a planner panic fails that case
alone, as an ERROR row carrying the panic's message.

``--write-gcode FILE`` writes the cases' G-code instead, one marked
section per case and speed; the fit audit plans
``tools/sim/gcode/corner_flow_cases.gcode``, written so, on the bench
configs and holds the extruder to its limits there
(``rust/pipeline-snapshot/tests/fit_tolerance.rs``).

Columns, per case and speed (worst over the case's corners; ``-`` where it
has none of that kind):

- ``peak`` / ``dip``: largest bead over- / under-fill, % of the nominal
  bead, along the path within the window of a corner the ratio does not
  step at.
- ``w>5%``: path length (mm) around one such corner where the bead is off
  by more than 5%.
- ``blob``: over-filled volume per such corner, nL (1 nL = 0.001 mm^3).
- ``moved`` / ``net`` / ``sw``: at a corner where the ratio steps, the
  material laid across the switch against the G-code's sharp switch at the
  vertex (nL: the mean of the excess before and the shortfall after), the
  material the step gains or loses (nL, signed), and the path the laid bead
  takes from 5% to 95% of the step (mm).
- ``ve+`` / ``ve-`` / ``ae``: peak extruder velocity (mm/s) and
  acceleration (mm/s^2) while the nozzle moves.
- ``|ve|*`` / ``|ae|*``: the same peaks over the whole run, the travel,
  retract and prime transitions at rest included.
- Material the extruder moves while the unshaped toolhead rests (a retract
  and its prime) lays no bead and is left out of the bead columns.
- ``dE_gc``: final e minus the G-code's total E (mm). Negative by design
  where the shaped path cuts a corner: the bead follows the shorter path.
- ``dE_adv``: what the run lays beyond the bead ratio times its own shaped
  path, minus the same for the run without the advance stage(s), mm; only
  for cases that extrude one ratio throughout (``n/a`` otherwise). The two
  runs need not travel the same path: the extruder caps budget the
  toolhead's acceleration for the advance, so its corners are planned and
  shaped differently. An advance vanishes at rest; the remainder is the
  chord rule at the spills (docs/rewrite/shaper.md, "Followers"), up to
  about 1e-4 mm where the two plans' spills curve differently, so anything
  well above that is lost or invented material.
- ``t``: the case's planned print time (s), what the extruder limits and
  any kernel cost in time.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
for _path in (REPO / "klippy", REPO / "snapshots", REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from tools import flow_metric  # noqa: E402

CASE_JOB_FLAG = "--case-job"
CASE_MARKER = "; corner_flow case: "
ERROR_CHARS = 200
SPEEDS_MM_S = (50.0, 150.0, 300.0)
START = (0.0, 0.0)
ADVANCE_TYPES = {
    "linear_pressure_advance": "k",
    "tanh_pressure_advance": "linear_advance",
    "recipr_pressure_advance": "linear_advance",
}


@dataclass
class Move:
    x: float
    y: float
    e: float


@dataclass
class Geometry:
    name: str
    moves: list[Move]
    corners: list[tuple[float, float]]
    window_mm: float = flow_metric.CORNER_WINDOW_MM

    def vertices(self) -> list[tuple[float, float]]:
        return flow_metric.path_vertices(
            [START, *((move.x, move.y) for move in self.moves)]
        )

    def corner_ratios(self) -> list[tuple[float, float]]:
        """The bead (over the nominal) of the path moves into and out of
        each corner."""
        ends, scales = [], []
        x, y = START
        for move in self.moves:
            if (move.x, move.y) != (x, y):
                ends.append((move.x, move.y))
                scales.append(move.e)
            x, y = move.x, move.y
        found = []
        index = 0
        for corner_xy in self.corners:
            index = ends.index(corner_xy, index)
            found.append((scales[index], scales[index + 1]))
            index += 1
        return found


@dataclass
class Row:
    case: str
    speed: float
    sweep: str = ""
    peak: float = math.nan
    dip: float = math.nan
    width_mm: float = math.nan
    blob_nl: float = math.nan
    moved_nl: float = math.nan
    net_nl: float = math.nan
    switch_mm: float = math.nan
    ve_max: float = math.nan
    ve_min: float = math.nan
    ae_max: float = math.nan
    ve_all: float = math.nan
    ae_all: float = math.nan
    de_gcode: float = math.nan
    de_advance: float = math.nan
    time_s: float = math.nan
    error: str = ""
    corners: list[dict] = field(default_factory=list)
    steps: list[dict] = field(default_factory=list)


def corner(interior_deg: float, leg: float = 60.0) -> Geometry:
    turn = math.radians(180.0 - interior_deg)
    apex = (leg, 0.0)
    end = (leg + leg * math.cos(turn), leg * math.sin(turn))
    return Geometry(
        f"c{interior_deg:g}",
        [Move(*apex, 1.0), Move(*end, 1.0)],
        [apex],
    )


def zigzag(leg: float = 2.0, turns: int = 20, lead: float = 30.0) -> Geometry:
    rise = leg / math.sqrt(2.0)
    moves = [Move(lead, 0.0, 1.0)]
    x, y = lead, 0.0
    apexes = []
    for index in range(turns + 1):
        apexes.append((x, y))
        x += rise
        y = rise if index % 2 == 0 else 0.0
        moves.append(Move(x, y, 1.0))
    apexes.append((x, y))
    moves.append(Move(x + lead, y, 1.0))
    return Geometry("zigzag", moves, apexes[3:-3], window_mm=leg / 2.0)


def perimeter(name: str, flow: list[float], approach: float) -> Geometry:
    """A 20 mm square loop entered and left along lines extruding at
    `approach` times the nominal bead: 0 is travel at speed."""
    loop = [(40.0, 20.0), (40.0, 40.0), (20.0, 40.0), (20.0, 20.0)]
    moves = [Move(10.0, 10.0, approach), Move(20.0, 20.0, approach)]
    moves += [Move(x, y, scale) for (x, y), scale in zip(loop, flow)]
    moves += [Move(20.0, 5.0, approach), Move(5.0, 5.0, approach)]
    corners = [(20.0, 20.0)] + loop
    return Geometry(name, moves, corners, window_mm=10.0)


def retract(length: float = 0.8) -> Geometry:
    """A line that stops, retracts, travels at speed, primes and extrudes
    again; its corners are the two rest points."""
    moves = [
        Move(30.0, 0.0, 1.0),
        Move(30.0, 0.0, -length),
        Move(30.0, 20.0, 0.0),
        Move(30.0, 20.0, length),
        Move(60.0, 20.0, 1.0),
    ]
    return Geometry(
        "retract", moves, [(30.0, 0.0), (30.0, 20.0)], window_mm=10.0
    )


GEOMETRIES = (
    corner(45),
    corner(90),
    corner(135),
    zigzag(),
    perimeter("flow 1.5x", [1.0, 1.5, 1.0, 1.0], approach=1.0),
    perimeter("travel", [1.0, 1.0, 1.0, 1.0], approach=0.0),
    retract(),
)
KERNEL_TYPES = ("smooth_triangle", "smooth_bell", "smooth_mzv", "smooth_zv")


def gcode(geometry: Geometry, speed: float, ratio: float) -> str:
    lines = [
        "G21",
        "G90",
        "M83",
        f"G1 X{START[0]:g} Y{START[1]:g} F{speed * 60:.0f}",
    ]
    x, y = START
    for move in geometry.moves:
        length = math.hypot(move.x - x, move.y - y)
        e = move.e * ratio * length if length > 0 else move.e
        if length > 0 and move.e == 0:
            lines.append(f"G1 X{move.x:.5f} Y{move.y:.5f}")
        elif length > 0:
            lines.append(f"G1 X{move.x:.5f} Y{move.y:.5f} E{e:.6f}")
        else:
            lines.append(f"G1 E{e:.6f}")
        x, y = move.x, move.y
    return "\n".join(lines) + "\n"


def gcode_total_e(geometry: Geometry, ratio: float) -> float:
    total = 0.0
    x, y = START
    for move in geometry.moves:
        length = math.hypot(move.x - x, move.y - y)
        total += move.e * ratio * length if length > 0 else move.e
        x, y = move.x, move.y
    return total


def bead_ratio(bead: tuple[float, float, float]) -> float:
    """Filament per path millimetre of a `width, height, filament` bead."""
    width, height, filament = bead
    return width * height / (math.pi * (filament / 2.0) ** 2)


def cases_gcode(bead: tuple[float, float, float], speeds: list[float]) -> str:
    """Every case at every speed, each opened by a marker line naming it."""
    ratio = bead_ratio(bead)
    return "".join(
        f"{CASE_MARKER}{geometry.name} {speed:g}\n"
        + gcode(geometry, speed, ratio)
        for geometry in GEOMETRIES
        for speed in speeds
    )


class PrinterConfig:
    """The printer config as klippy reads it, editable by section."""

    def __init__(self, fileconfig):
        self.fileconfig = fileconfig

    @classmethod
    def load(cls, path: Path) -> PrinterConfig:
        from klippy import configfile

        loader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
        loader.printer = None
        return cls(loader.read_config(str(path)).fileconfig)

    @classmethod
    def parse(cls, text: str) -> PrinterConfig:
        from klippy import configfile

        loader = configfile.PrinterConfig.__new__(configfile.PrinterConfig)
        loader.printer = None
        return cls(loader._build_config_wrapper(text, "corner_flow").fileconfig)

    def text(self) -> str:
        return self.fileconfig.write_string()

    def copy(self) -> PrinterConfig:
        return PrinterConfig.parse(self.text())

    def getfloat(self, section: str, option: str, default: float) -> float:
        if self.fileconfig.has_option(section, option):
            return float(self.fileconfig.get(section, option))
        return default

    def follower_axis(self) -> str:
        followers = [
            section
            for section in self.fileconfig.sections()
            if section.startswith("axis ")
            and self.fileconfig.has_option(section, "follows")
            and self.fileconfig.get(section, "follows").strip()
        ]
        if len(followers) != 1:
            raise SystemExit(
                f"expected exactly one following axis, found {followers}"
            )
        return followers[0]

    def chain(self, axis: str) -> list[str]:
        if not self.fileconfig.has_option(axis, "post_processors"):
            return []
        names = self.fileconfig.get(axis, "post_processors").split(",")
        return [name.strip() for name in names if name.strip()]

    def post_processor_type(self, name: str) -> str:
        return self.fileconfig.get(f"post_processor {name}", "type").strip()

    def set_param(self, target: str, value: str) -> None:
        name, _, option = target.rpartition(".")
        names = [name] if name else self.chain(self.follower_axis())
        sections = [
            f"post_processor {pp}"
            for pp in names
            if self.fileconfig.has_option(f"post_processor {pp}", option)
        ]
        if not sections:
            raise SystemExit(f"no post-processor has '{target}'")
        for section in sections:
            self.fileconfig.set(section, option, value)

    def unshaped(self) -> PrinterConfig:
        clone = self.copy()
        for section in clone.fileconfig.sections():
            if not section.startswith("axis "):
                continue
            if clone.fileconfig.has_option(section, "post_processors"):
                clone.fileconfig.set(section, "post_processors", "")
        return clone

    def with_extrude_only_caps(self, caps: tuple[str, str]) -> PrinterConfig:
        clone = self.copy()
        for option, value in zip(
            ("max_extrude_only_velocity", "max_extrude_only_accel"), caps
        ):
            if not clone.fileconfig.has_option("extruder", option):
                clone.fileconfig.set("extruder", option, value)
        return clone

    def kernel_free(self) -> PrinterConfig:
        """The chain with its follower smoothing kernels taken out."""
        clone = self.copy()
        axis = clone.follower_axis()
        kept = [
            name
            for name in clone.chain(axis)
            if clone.post_processor_type(name) not in KERNEL_TYPES
        ]
        clone.fileconfig.set(axis, "post_processors", ", ".join(kept))
        return clone

    def describe(self) -> str:
        axis = self.follower_axis()
        stages = []
        for name in self.chain(axis):
            section = f"post_processor {name}"
            params = ",".join(
                f"{option}={self.fileconfig.get(section, option)}"
                for option in self.fileconfig.options(section)
                if option != "type"
            )
            stages.append(f"{self.post_processor_type(name)}({params})")
        return f"[{axis}] " + " -> ".join(stages or ["(empty)"])

    def without_advance(self) -> PrinterConfig:
        clone = self.copy()
        axis = clone.follower_axis()
        kept = [
            name
            for name in clone.chain(axis)
            if clone.post_processor_type(name) not in ADVANCE_TYPES
        ]
        clone.fileconfig.set(axis, "post_processors", ", ".join(kept))
        return clone

    def advance_k(self) -> float:
        for name in self.chain(self.follower_axis()):
            option = ADVANCE_TYPES.get(self.post_processor_type(name))
            if option:
                return self.getfloat(f"post_processor {name}", option, 0.0)
        return 0.0


def import_engine():
    import harness

    return harness._import_engine()


def sample(config_text: str, program: str):
    engine = import_engine()
    waypoints = engine.parse_gcode(program, config_text)
    probe, _ = engine.pipeline_snapshot_axis_samples(
        waypoints, config_text, [0], 2
    )
    t_end = json.loads(probe)["trajectory"]["t_end"]
    count = int(t_end / flow_metric.SAMPLE_DT_S) + 2
    _, rows = engine.pipeline_snapshot_axis_samples(
        waypoints, config_text, [0, 1, 3], count
    )
    return np.linspace(0.0, t_end, count), np.array(rows)


def measure(
    geometry: Geometry,
    speed: float,
    config: PrinterConfig,
    melt_k: float,
    bead: tuple[float, float, float],
) -> Row:
    width, height, _ = bead
    ratio = bead_ratio(bead)
    row = Row(geometry.name, speed)
    program = gcode(geometry, speed, ratio)
    try:
        t, axes = sample(config.text(), program)
        _, plain = sample(config.without_advance().text(), program)
        plain_xy = np.column_stack([plain[0][:, 0], plain[1][:, 0]])
        t_raw, raw = sample(config.unshaped().text(), program)
    except Exception as error:  # noqa: BLE001
        row.error = str(error).splitlines()[0][:ERROR_CHARS]
        return row
    x, vx = axes[0][:, 0], axes[0][:, 1]
    y, vy = axes[1][:, 0], axes[1][:, 1]
    e_cmd, ve, ae = axes[2][:, 0], axes[2][:, 1], axes[2][:, 2]
    speed_xy = np.hypot(vx, vy)
    xy = np.column_stack([x, y])
    trace = flow_metric.flow_trace(
        t,
        xy,
        e_cmd,
        flow_metric.commanded_bead(t, xy, t_raw, raw, geometry.vertices()),
        melt_k,
        ratio,
    )
    bead_area = width * height
    for apex, ratios in zip(
        flow_metric.apex_indices(xy, geometry.corners), geometry.corner_ratios()
    ):
        if flow_metric.is_ratio_step(*ratios):
            row.steps.append(
                flow_metric.step_report(
                    trace, apex, ratios, geometry.window_mm, bead_area
                )
            )
        else:
            row.corners.append(
                flow_metric.corner_report(
                    trace, apex, geometry.window_mm, bead_area
                )
            )
    if row.corners:
        row.peak = max(c["peak"] for c in row.corners) * 100.0
        row.dip = min(c["dip"] for c in row.corners) * 100.0
        row.width_mm = max(c["width_mm"] for c in row.corners)
        row.blob_nl = max(c["blob_nl"] for c in row.corners)
    if row.steps:
        row.moved_nl = max(s["moved_nl"] for s in row.steps)
        row.net_nl = max((s["net_nl"] for s in row.steps), key=abs)
        row.switch_mm = max(s["switch_mm"] for s in row.steps)
    moving = speed_xy > flow_metric.MOVING_SPEED_MM_S
    row.ve_max = float(ve[moving].max())
    row.ve_min = float(ve[moving].min())
    row.ae_max = float(np.abs(ae[moving]).max())
    row.ve_all = float(np.abs(ve).max())
    row.ae_all = float(np.abs(ae).max())
    row.time_s = float(t[-1])
    row.de_gcode = float(e_cmd[-1] - gcode_total_e(geometry, ratio))
    if all(move.e == 1.0 for move in geometry.moves):
        row.de_advance = beyond_the_path(xy, e_cmd, ratio) - beyond_the_path(
            plain_xy, plain[2][:, 0], ratio
        )
    return row


def _signed(value: float) -> str:
    return "n/a" if math.isnan(value) else f"{value:+.1e}"


def _cell(value: float, spec: str, width: int) -> str:
    text = "-" if math.isnan(value) else format(value, spec)
    return f"{text:>{width}}"


def beyond_the_path(xy, e, ratio: float) -> float:
    """Material laid beyond `ratio` times the length of the shaped path
    `xy`, sampled densely enough that its chords sum to the path."""
    path = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])).sum()
    return float(e[-1] - e[0] - ratio * path)


def run_case(job) -> Row:
    sweep_label, config_text, geometry_index, speed, melt_k, bead = job
    config = PrinterConfig.parse(config_text)
    row = measure(GEOMETRIES[geometry_index], speed, config, melt_k, bead)
    row.sweep = sweep_label
    return row


def case_command() -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), CASE_JOB_FLAG]


def run_isolated(job, command: list[str] | None = None) -> Row:
    """Runs one case in a process of its own, `command` reading the job as
    JSON on stdin and printing its row as JSON last on stdout. A planner
    panic aborts that process, so it fails this case alone, reported with
    the panic's message, and every other case still runs."""
    sweep_label, _, geometry_index, speed, _, _ = job
    done = subprocess.run(
        command or case_command(),
        input=json.dumps(job),
        capture_output=True,
        text=True,
        check=False,
    )
    if done.returncode == 0:
        return Row(**json.loads(done.stdout.splitlines()[-1]))
    return Row(
        GEOMETRIES[geometry_index].name,
        speed,
        sweep=sweep_label,
        error=crash_message(done.stderr, done.returncode)[:ERROR_CHARS],
    )


def crash_message(stderr: str, returncode: int) -> str:
    lines = [line for line in stderr.splitlines() if line.strip()]
    for panic, message in zip(lines, lines[1:]):
        if " panicked at " in panic:
            return f"engine panic: {message}"
    return f"engine exited {returncode}"


def run_case_job() -> None:
    print(json.dumps(asdict(run_case(json.loads(sys.stdin.read())))))


def table(rows: list[Row], sweep_name: str) -> str:
    head = (
        f"{'case':<10} {'mm/s':>4} | {'peak':>6} {'dip':>5} {'w>5%':>5} "
        f"{'blob':>6} | {'moved':>6} {'net':>6} {'sw':>5} | "
        f"{'ve+':>6} {'ve-':>6} {'ae':>7} | {'|ve|*':>6} "
        f"{'|ae|*':>7} | {'dE_gc':>8} {'dE_adv':>8} | {'t':>6}"
    )
    if sweep_name:
        head = f"{sweep_name:>11} " + head
    lines = [head, "-" * len(head)]
    for row in rows:
        prefix = f"{row.sweep:>11} " if sweep_name else ""
        if row.error:
            lines.append(
                f"{prefix}{row.case:<10} {row.speed:>4.0f} | ERROR "
                f"{row.error[:90]}"
            )
            continue
        lines.append(
            f"{prefix}{row.case:<10} {row.speed:>4.0f} | "
            f"{_cell(row.peak, '+.0f', 5)}% {_cell(row.dip, '+.0f', 4)}% "
            f"{_cell(row.width_mm, '.1f', 5)} {_cell(row.blob_nl, '.1f', 6)} | "
            f"{_cell(row.moved_nl, '.1f', 6)} {_cell(row.net_nl, '+.1f', 6)} "
            f"{_cell(row.switch_mm, '.2f', 5)} | "
            f"{row.ve_max:>6.1f} {row.ve_min:>6.1f} {row.ae_max:>7.0f} | "
            f"{row.ve_all:>6.1f} {row.ae_all:>7.0f} | "
            f"{row.de_gcode:>+8.4f} {_signed(row.de_advance):>8} | "
            f"{row.time_s:>6.3f}"
        )
    return "\n".join(lines)


def main() -> None:
    if sys.argv[1:] == [CASE_JOB_FLAG]:
        run_case_job()
        return
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("\n\n", 1)[1],
    )
    parser.add_argument(
        "config", type=Path, nargs="?", help="printer.cfg to evaluate"
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="[PP.]PARAM=VALUE",
        help="override a post-processor parameter; a bare PARAM applies to "
        "every post-processor of the following axis that has it",
    )
    parser.add_argument(
        "--sweep",
        metavar="[PP.]PARAM=V1,V2,...",
        help="repeat the table for each value, as with --set",
    )
    parser.add_argument(
        "--melt-k",
        type=float,
        help="melt time constant K in s (default: the configured advance)",
    )
    parser.add_argument(
        "--bead",
        default="0.4,0.2,1.75",
        metavar="WIDTH,HEIGHT,FILAMENT",
        help="nominal bead width and layer height, filament diameter (mm)",
    )
    parser.add_argument(
        "--speeds",
        default=",".join(f"{s:g}" for s in SPEEDS_MM_S),
        help="comma-separated feed rates, mm/s",
    )
    parser.add_argument(
        "--kernel-free",
        action="store_true",
        help="take the follower's smoothing kernels out of its chain",
    )
    parser.add_argument(
        "--extrude-only",
        default="50,5000",
        metavar="V,A",
        help="extrude-only caps (mm/s, mm/s^2) filled in where the config "
        "lacks them",
    )
    parser.add_argument("--json", action="store_true", help="emit JSON rows")
    parser.add_argument(
        "--write-gcode",
        type=Path,
        metavar="FILE",
        help="write every case's G-code at --speeds and --bead to FILE, "
        f"each opened by a '{CASE_MARKER}<case> <speed>' line, and exit; "
        "no config is read",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=os.cpu_count() or 1,
        help="parallel planner runs",
    )
    args = parser.parse_args()
    bead = tuple(float(v) for v in args.bead.split(","))
    speeds = [float(v) for v in args.speeds.split(",")]
    if args.write_gcode:
        args.write_gcode.write_text(cases_gcode(bead, speeds))
        return
    if args.config is None:
        parser.error("a config is required unless --write-gcode is given")

    velocity, _, accel = args.extrude_only.partition(",")
    base = PrinterConfig.load(args.config).with_extrude_only_caps(
        (velocity, accel)
    )
    if args.kernel_free:
        base = base.kernel_free()
    for assignment in args.set:
        target, _, value = assignment.partition("=")
        base.set_param(target, value)
    variants = [("", base)]
    sweep_name = ""
    if args.sweep:
        sweep_name, _, values = args.sweep.partition("=")
        variants = []
        for value in values.split(","):
            variant = base.copy()
            variant.set_param(sweep_name, value)
            variants.append((value, variant))

    jobs = []
    for label, variant in variants:
        melt_k = args.melt_k if args.melt_k is not None else variant.advance_k()
        for index in range(len(GEOMETRIES)):
            for speed in speeds:
                jobs.append((label, variant.text(), index, speed, melt_k, bead))
    with concurrent.futures.ThreadPoolExecutor(max(args.jobs, 1)) as pool:
        rows = list(pool.map(run_isolated, jobs))

    if args.json:
        print(json.dumps([asdict(row) for row in rows], indent=1))
        return
    melt = (
        f"{args.melt_k:g} s"
        if args.melt_k is not None
        else "configured advance per variant"
    )
    print(f"{args.config}  melt K: {melt}  bead w,h,filament: {args.bead}")
    for label, variant in variants:
        print(f"  {label or 'chain'}: {variant.describe()}")
    print(table(rows, sweep_name))


if __name__ == "__main__":
    main()
