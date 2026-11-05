"""Judge a bench capture against the offline planner's reference trajectory.

Usage: uv run python tools/bench/analyze.py <run_dir>
    [--step-benchmark PULSED,BOTH_EDGE,CLOCK_HZ,FRACTION]

run_dir holds capture.bin, capture.json, print.gcode and printer.cfg. The
verdict goes to stdout and run_dir/analysis.json; plots to run_dir/*.png
when matplotlib is importable.

The reference is capped by the MCU step capacity exactly as the live run
was: run.py records every MCU's step benchmark, with the capacity fraction
the live planner applied, in run.json and the reference plans under it. A
run.json from before that recording plans uncapped unless --step-benchmark
supplies the bench MCU's benchmark and the fraction its run was capped at
(e.g. the F401's 301,160,84000000,0.75 for runs before the fraction became
0.625), charged to every captured motor stepped pulsed; the verdict's
reference_step_capacity says which ("recorded", "supplied", "absent").

A run recorded with run.py's dispatch record also holds
dispatch_record_*.bin: what the live host dispatched, segment by segment,
with its anchoring and every drain to rest. When the pacer braked the
stream to rest, the plan no longer describes the motion, so the record is
the reference (reference_source "dispatch_record", reference time = host
seconds since its first segment) and the brakes fail no_pacer_brakes. Any
record is also compared with the plan segment for segment: outside the
windows a pacer brake replanned, every dispatched segment must be the
planned one (dispatch_matches_plan_outside_brakes). The extruder is held
to the plan only until the first brake: a brake stops on a corner the plan
cuts, so the extrusion the follower spreads over what follows differs, and
dispatch_record.max_extruder_deviation_after_brakes_steps reports how far.

Alignment model per motion epoch:
    t_capture = offset_s + (1 + ppm * 1e-6) * t_reference
Timing errors are capture minus aligned reference: positive = step late.
"""

from __future__ import annotations

import argparse
import configparser
import difflib
import hashlib
import json
import math
import re
import shlex
import sys
from collections.abc import Callable
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
for _path in (REPO_ROOT / "snapshots", REPO_ROOT / "klippy", REPO_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import capture_events  # noqa: E402
import corner_flow_bench  # noqa: E402
import dispatch_record  # noqa: E402
import harness  # noqa: E402
import mcu_execution  # noqa: E402

REFERENCE_PLANNED = "plan"
REFERENCE_DISPATCHED = "dispatch_record"
DISPATCH_RECORD_GLOB = "dispatch_record_*.bin"
DISPATCH_SEAM_SLACK_S = 1e-9
DISPATCH_DURATION_TOLERANCE_S = 1e-9
REPORTED_DIVERGENCES_LIMIT = 50
BRAKE_SETTLE_S = 0.1
EXTRUDER_AXIS = 3

CAPTURE_ROLES = ("a", "b", "z", "e")
ROLE_AXIS_WEIGHTS = {
    "a": (1.0, 1.0, 0.0, 0.0),
    "b": (1.0, -1.0, 0.0, 0.0),
    "z": (0.0, 0.0, 1.0, 0.0),
    "e": (0.0, 0.0, 0.0, 1.0),
}
FOLLOWER_ROLES = frozenset({"e"})
REFERENCE_SAMPLE_PERIOD_S = 0.5e-3
REFERENCE_FINE_FACTOR = 50
REFERENCE_CHUNK_INTERVALS = 20_000
REFERENCE_BLOCK_SEPARATION_S = 100.0
BLOCK_EDGE_SLACK_S = 1e-9
EMPTY_DIFF_SHA256 = hashlib.sha256(b"").hexdigest()
LATTICE_SNAP_STEPS = 1e-6
DEFAULT_EPOCH_GAP_S = 0.15
OFFSET_SEARCH_S = 1.0
OFFSET_SEARCH_BIN_S = 1e-3
FIT_WINDOWS_S = (10e-3, 2e-3, 2e-3, 2e-3)
REPORT_MATCH_WINDOW_S = 100e-3
PAIR_WINDOW_S = 2e-3
NS_PER_S = 1e9
EPOCH_DURATION_TOLERANCE_S = 1.0
EPOCH_DURATION_TOLERANCE_FRACTION = 0.5
TIMING_TOLERANCE_US = 50.0
PACE_TOLERANCE_S = 20e-3
PACE_TOLERANCE_PPM = 500.0
POSITION_EQUIVALENT_TOLERANCE_STEPS = 0.05
POSITION_BOUND_RELAXES_BELOW_HZ = POSITION_EQUIVALENT_TOLERANCE_STEPS / (
    TIMING_TOLERANCE_US * 1e-6
)
TIMING_CHECK = (
    f"timing_within_{TIMING_TOLERANCE_US:g}us"
    f"_or_{POSITION_EQUIVALENT_TOLERANCE_STEPS:g}_step"
)
CONDITIONED_RATE_HZ = 200.0
COUNT_ERROR_MARGIN_S = 0.05
INLIER_FLOOR_S = 200e-6
MIN_SCALE_FIT_DURATION_S = 1.0
MIN_FIT_PAIRS = 20
WANDER_WINDOW_S = 1.0
STALL_BIN_S = 5e-3
STALL_MIN_STEPS = 3
PEAK_RATE_WINDOW_TICKS = 160
REPORTED_STALLS_LIMIT = 50
EDGE_TIE_STEPS = 0.5
EPOCH_SPLIT_COUNT_SLACK = 200
EPOCH_SPLIT_MIN_GAP_S = 1e-3
STEP_BENCHMARKS_KEY = "step_benchmarks"
BENCH_MCU = "mcu"
STEP_CAPACITY_RECORDED = "recorded"
STEP_CAPACITY_SUPPLIED = "supplied"
STEP_CAPACITY_ABSENT = "absent"


class AnalysisError(RuntimeError):
    pass


@dataclass(frozen=True)
class SuppliedBenchmark:
    pulsed_ticks_per_step: int
    both_edge_ticks_per_step: int
    clock_hz: float
    capacity_fraction: float

    @classmethod
    def parse(cls, text: str) -> SuppliedBenchmark:
        parts = text.split(",")
        if len(parts) != 4:
            raise argparse.ArgumentTypeError(
                f"{text!r} is not PULSED,BOTH_EDGE,CLOCK_HZ,FRACTION"
            )
        return cls(
            int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3])
        )


@dataclass(frozen=True)
class Motor:
    role: str
    name: str
    channel: int
    step_mm: float
    dir_sign: int

    @property
    def axis_weights(self) -> tuple[float, float, float, float]:
        return ROLE_AXIS_WEIGHTS[self.role]

    @property
    def track_restarts_per_block(self) -> bool:
        return self.role in FOLLOWER_ROLES


@dataclass
class StepEvents:
    time: np.ndarray
    level: np.ndarray
    direction: np.ndarray
    rate_hz: np.ndarray

    @classmethod
    def empty(cls) -> StepEvents:
        return cls(
            np.empty(0, np.float64),
            np.empty(0, np.int64),
            np.empty(0, np.int8),
            np.empty(0, np.float64),
        )

    @classmethod
    def concat(cls, parts: list[StepEvents]) -> StepEvents:
        if not parts:
            return cls.empty()
        return cls(
            *(
                np.concatenate([getattr(p, f.name) for p in parts])
                for f in fields(cls)
            )
        )

    def __len__(self) -> int:
        return len(self.time)

    def between(self, start_s: float, end_s: float) -> StepEvents:
        lo = np.searchsorted(self.time, start_s, "left")
        hi = np.searchsorted(self.time, end_s, "right")
        return StepEvents(*(getattr(self, f.name)[lo:hi] for f in fields(self)))

    def start_count(self) -> int:
        return int(self.level[0] - self.direction[0])


@dataclass
class Capture:
    tick_hz: float
    deltas: np.ndarray
    exact_steps: list[tuple[np.ndarray, np.ndarray]] | None = None
    events: capture_events.EventCapture | None = None

    @property
    def duration_s(self) -> float:
        return len(self.deltas) / self.tick_hz


@dataclass
class ReferenceBlock:
    stream_offset_s: float
    duration_s: float
    sample_times: np.ndarray
    motor_pva_steps: dict[str, np.ndarray]


@dataclass
class Reference:
    source: str
    start_xyz: tuple[float, float, float]
    final_xyze: tuple[float, float, float, float]
    blocks: list[ReferenceBlock]
    events: dict[str, StepEvents]
    start_counts: dict[str, int]
    final_counts: dict[str, int]
    level_offsets: dict[str, list[float]]

    def trajectory_span_s(self, t_stream: float) -> tuple[float, float] | None:
        for block in self.blocks:
            end_s = block.stream_offset_s + block.duration_s
            if block.stream_offset_s <= t_stream <= end_s:
                return block.stream_offset_s, end_s
        return None

    def positions(
        self, role: str, start_s: float, end_s: float
    ) -> np.ndarray | None:
        for block, offset in zip(self.blocks, self.level_offsets[role]):
            local_start = start_s - block.stream_offset_s
            local_end = end_s - block.stream_offset_s
            if (
                local_start < -BLOCK_EDGE_SLACK_S
                or local_end > block.duration_s + BLOCK_EDGE_SLACK_S
            ):
                continue
            times = block.sample_times
            lo = min(
                len(times) - 2,
                max(0, int(np.searchsorted(times, local_start, "right")) - 1),
            )
            hi = max(lo + 1, int(np.searchsorted(times, local_end, "left")))
            fine_t, fine_p = upsample(
                times[lo : hi + 1],
                block.motor_pva_steps[role][lo : hi + 1],
                REFERENCE_FINE_FACTOR,
            )
            inside = (fine_t >= local_start) & (fine_t <= local_end)
            ends = np.interp([local_start, local_end], fine_t, fine_p)
            return np.concatenate((fine_p[inside], ends)) - offset
        return None


@dataclass
class Job:
    run_dir: Path
    motors: list[Motor]
    config: harness.PrinterConfigData
    gcode_text: str
    step_benchmarks: list[dict] | None
    step_benchmarks_source: str
    dispatch_record: dispatch_record.DispatchRecord | None


@dataclass
class RunInputs(Job):
    capture: Capture


@dataclass
class Epoch:
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass
class Alignment:
    offset_s: float
    ppm: float
    scale_fitted: bool

    def to_capture(self, t_stream: np.ndarray | float):
        return self.offset_s + (1.0 + self.ppm * 1e-6) * t_stream

    def to_stream(self, t_capture: np.ndarray | float):
        return (t_capture - self.offset_s) / (1.0 + self.ppm * 1e-6)


@dataclass
class MatchedSteps:
    late_s: np.ndarray
    capture_time_s: np.ndarray
    reference_rate_hz: np.ndarray
    unmatched_capture: int
    unmatched_reference: int
    edge_ties: int
    rest_ties: int = 0


@dataclass
class EpochResult:
    capture: Epoch
    reference: Epoch
    alignment: Alignment
    steps: EpochSteps
    matches: dict[str, MatchedSteps]


def load_capture(run_dir: Path) -> Capture:
    meta = json.loads((run_dir / "capture.json").read_text())
    channels = meta["channel_count"]
    if channels != len(CAPTURE_ROLES):
        raise AnalysisError(
            f"capture.json channel_count={channels}, analyzer expects "
            f"{len(CAPTURE_ROLES)} ({', '.join(CAPTURE_ROLES)})"
        )
    if meta.get("mode", "counts") == "events":
        events = capture_events.load(run_dir)
        return Capture(
            float(capture_events.TICK_HZ),
            events.binned_deltas(),
            [
                (c.step_ticks / float(meta["tick_hz"]), c.step_signs())
                for c in events.channels
            ],
            events,
        )
    raw = np.fromfile(run_dir / "capture.bin", dtype=np.int8)
    if raw.size % channels:
        raise AnalysisError(
            f"capture.bin size {raw.size} is not a multiple of {channels}"
        )
    return Capture(float(meta["tick_hz"]), raw.reshape(-1, channels))


def _config_sections(config_text: str) -> configparser.RawConfigParser:
    parser = configparser.RawConfigParser(strict=False)
    parser.read_string(config_text)
    return parser


def _single_motor_name(value: str, owner: str) -> str:
    names = [n.strip() for n in value.split(",") if n.strip()]
    if len(names) != 1:
        raise AnalysisError(
            f"{owner} lists motors {names}; the capture wiring has exactly "
            "one channel per role"
        )
    return names[0]


def read_motors(sections: configparser.RawConfigParser) -> list[Motor]:
    kin_type = sections.get("kinematics", "type")
    if kin_type != "corexy":
        raise AnalysisError(
            f"kinematics type {kin_type!r} unsupported: analyzer inverts corexy"
        )
    names = {
        "a": _single_motor_name(sections.get("kinematics", "a_motors"), "a"),
        "b": _single_motor_name(sections.get("kinematics", "b_motors"), "b"),
        "z": _single_motor_name(sections.get("kinematics", "z_motors"), "z"),
        "e": _single_motor_name(sections.get("axis e", "motors"), "[axis e]"),
    }
    motors = []
    for channel, role in enumerate(CAPTURE_ROLES):
        section = f"motor {names[role]}"
        if sections.has_option(section, "gear_ratio"):
            raise AnalysisError(f"[{section}] gear_ratio is not supported")
        full_steps = sections.getint(
            section, "full_steps_per_rotation", fallback=200
        )
        step_mm = sections.getfloat(section, "rotation_distance") / (
            full_steps * sections.getint(section, "microsteps")
        )
        dir_pin = sections.get(section, "dir_pin").strip()
        motors.append(
            Motor(
                role=role,
                name=names[role],
                channel=channel,
                step_mm=step_mm,
                dir_sign=-1 if dir_pin.startswith("!") else 1,
            )
        )
    return motors


def recorded_step_benchmarks(run_dir: Path) -> list[dict] | None:
    run_json = run_dir / "run.json"
    if not run_json.exists():
        return None
    return json.loads(run_json.read_text()).get(STEP_BENCHMARKS_KEY)


def supplied_step_benchmarks(
    motors: list[Motor], benchmark: SuppliedBenchmark
) -> list[dict]:
    """The bench MCU stepping every captured motor pulsed, at a benchmark
    supplied for a run whose run.json predates recording it."""
    return [
        {
            "mcu": BENCH_MCU,
            "clock_freq": benchmark.clock_hz,
            "capacity_fraction": benchmark.capacity_fraction,
            "pulsed_ticks_per_step": benchmark.pulsed_ticks_per_step,
            "both_edge_ticks_per_step": benchmark.both_edge_ticks_per_step,
            "simulated": False,
            "motors": [
                {"name": m.name, "step_distance": m.step_mm, "mode": "pulsed"}
                for m in motors
            ],
        }
    ]


def load_job(run_dir: Path, supplied: SuppliedBenchmark | None = None) -> Job:
    config = harness.read_printer_config(run_dir / "printer.cfg")
    motors = read_motors(_config_sections(config.config_text))
    recorded = recorded_step_benchmarks(run_dir)
    if recorded is not None and supplied is not None:
        raise AnalysisError(
            f"{run_dir / 'run.json'} records the step benchmarks the run was "
            "capped by; a supplied benchmark would replace them"
        )
    if recorded is not None:
        step_benchmarks, source = recorded, STEP_CAPACITY_RECORDED
    elif supplied is not None:
        step_benchmarks = supplied_step_benchmarks(motors, supplied)
        source = STEP_CAPACITY_SUPPLIED
    else:
        step_benchmarks, source = None, STEP_CAPACITY_ABSENT
    return Job(
        run_dir=run_dir,
        motors=motors,
        config=config,
        gcode_text=(run_dir / "print.gcode").read_text(),
        step_benchmarks=step_benchmarks,
        step_benchmarks_source=source,
        dispatch_record=load_dispatch_record(run_dir),
    )


def load_dispatch_record(
    run_dir: Path,
) -> dispatch_record.DispatchRecord | None:
    """The live host's record of what it dispatched for the print: the one
    file of the run that holds segments. A pipeline set up and torn down
    before the print leaves an empty one."""
    paths = sorted(run_dir.glob(DISPATCH_RECORD_GLOB))
    if not paths:
        return None
    try:
        records = {p.name: dispatch_record.load(p) for p in paths}
    except dispatch_record.RecordError as exc:
        raise AnalysisError(str(exc)) from exc
    moving = {name: r for name, r in records.items() if len(r)}
    if len(moving) != 1:
        raise AnalysisError(
            f"{run_dir}: expected one dispatch record with segments, found "
            f"{sorted(moving) or 'none'} among {sorted(records)}"
        )
    ((name, record),) = moving.items()
    if record.nudges:
        raise AnalysisError(
            f"{name}: {record.nudges} nudges bypassed the planner; neither "
            "the plan nor the record's segments describe that motion"
        )
    if record.axes != len(ROLE_AXIS_WEIGHTS["a"]):
        raise AnalysisError(
            f"{name}: records {record.axes} axes, the analyzer reads "
            f"{len(ROLE_AXIS_WEIGHTS['a'])}"
        )
    return record


def load_run(
    run_dir: Path, supplied: SuppliedBenchmark | None = None
) -> RunInputs:
    return RunInputs(
        **vars(load_job(run_dir, supplied)), capture=load_capture(run_dir)
    )


def _command_word(line: str) -> str:
    code = line.split(";", 1)[0].strip()
    return code.split(None, 1)[0].upper() if code else ""


def _kinematic_start(line: str) -> tuple[float, float, float]:
    params = dict(re.findall(r"([XYZ])=(\S+)", line.upper()))
    if set(params) != {"X", "Y", "Z"}:
        raise AnalysisError(
            f"first command {line.strip()!r} must set X, Y and Z explicitly"
        )
    return float(params["X"]), float(params["Y"]), float(params["Z"])


def _start_position(text: str) -> tuple[float, float, float]:
    lines = [ln for ln in text.splitlines() if _command_word(ln)]
    if not lines or _command_word(lines[0]) != "SET_KINEMATIC_POSITION":
        raise AnalysisError(
            "print.gcode must start with SET_KINEMATIC_POSITION X= Y= Z="
        )
    return _kinematic_start(lines[0])


def split_at_rests(waypoints: list[dict]) -> list[list[dict]]:
    """The print's moves as motion epochs: the runs of moves between two
    dwells at rest, each starting on the waypoint the previous run ended
    on."""
    blocks = [[waypoints[0]]]
    for waypoint in waypoints[1:]:
        if waypoint["dwell_before_s"] is not None:
            blocks.append([blocks[-1][-1]])
            waypoint = {**waypoint, "dwell_before_s": None}
        blocks[-1].append(waypoint)
    return [block for block in blocks if len(block) > 1]


@dataclass
class PlannedPrint:
    start_xyz: tuple[float, float, float]
    final_xyze: tuple
    waypoints: list[dict]
    blocks: list[list[dict]]


def plan_waypoints(job: Job) -> PlannedPrint:
    start = _start_position(job.gcode_text)
    engine = harness._import_engine()
    try:
        waypoints = engine.parse_gcode(job.gcode_text, job.config.config_text)
    except ValueError as exc:
        raise AnalysisError(f"print.gcode: {exc}") from exc
    if tuple(waypoints[0]["position"][:3]) != start:
        raise AnalysisError(
            f"parsed start {waypoints[0]['position']} differs from "
            f"SET_KINEMATIC_POSITION {start}"
        )
    blocks = split_at_rests(waypoints)
    if not blocks:
        raise AnalysisError("print.gcode contains no motion")
    return PlannedPrint(
        start, tuple(waypoints[-1]["position"]), waypoints, blocks
    )


def stream_epochs(
    job: Job,
    planned: PlannedPrint,
    config_text: str,
    axes: list[int],
    sample_period_s: float,
) -> list[tuple[float, float, np.ndarray]]:
    """The print planned as one stream, as the live host plans it, and
    sampled across each epoch it moves between two dwells: `(t_start,
    t_end, axes x samples x (p, v, a))`. Planning an epoch on its own from
    rest is not the same plan: the planner's fits may cut a stream deep in
    history differently, within their tolerance."""
    engine = harness._import_engine()
    epochs = engine.pipeline_snapshot_epoch_samples(
        planned.waypoints,
        config_text,
        axes,
        sample_period_s,
        job.step_benchmarks,
    )
    if len(epochs) != len(planned.blocks):
        raise AnalysisError(
            f"the stream plan moves in {len(epochs)} epochs between rests, "
            f"the print's dwells split it into {len(planned.blocks)}"
        )
    return [
        (t_start, t_end, np.asarray(rows, dtype=np.float64))
        for t_start, t_end, rows in epochs
    ]


def _motor_pva_steps(motor: Motor, axes_pva: np.ndarray) -> np.ndarray:
    weights = np.asarray(motor.axis_weights)
    return np.einsum("a,asc->sc", weights, axes_pva) / motor.step_mm


def reference_block(
    job: Job, epoch: tuple[float, float, np.ndarray], stream_offset_s: float
) -> ReferenceBlock:
    t_start, t_end, axes_pva = epoch
    duration = t_end - t_start
    return ReferenceBlock(
        stream_offset_s=stream_offset_s,
        duration_s=duration,
        sample_times=np.linspace(0.0, duration, axes_pva.shape[1]),
        motor_pva_steps={
            m.role: _motor_pva_steps(m, axes_pva) for m in job.motors
        },
    )


def _quintic_hermite_basis(fine_factor: int) -> np.ndarray:
    s = np.arange(fine_factor) / fine_factor
    s2, s3, s4, s5 = s**2, s**3, s**4, s**5
    return np.stack(
        [
            1 - 10 * s3 + 15 * s4 - 6 * s5,
            s - 6 * s3 + 8 * s4 - 3 * s5,
            0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5,
            10 * s3 - 15 * s4 + 6 * s5,
            -4 * s3 + 7 * s4 - 3 * s5,
            0.5 * s3 - s4 + 0.5 * s5,
        ],
        axis=1,
    )


def upsample(
    times: np.ndarray, pva: np.ndarray, fine_factor: int
) -> tuple[np.ndarray, np.ndarray]:
    h = np.diff(times)
    coefs = np.stack(
        [
            pva[:-1, 0],
            h * pva[:-1, 1],
            h * h * pva[:-1, 2],
            pva[1:, 0],
            h * pva[1:, 1],
            h * h * pva[1:, 2],
        ]
    )
    basis = _quintic_hermite_basis(fine_factor)
    fine_p = (basis @ coefs).T.reshape(-1)
    fine_t = (
        times[:-1, None] + h[:, None] * (np.arange(fine_factor) / fine_factor)
    ).reshape(-1)
    return (
        np.append(fine_t, times[-1]),
        np.append(fine_p, pva[-1, 0]),
    )


@dataclass
class PlayState:
    count: int
    last_direction: int
    last_off_lattice: float = math.nan

    @classmethod
    def resting_at(cls, position_steps: float) -> PlayState:
        count = round(position_steps)
        on_lattice = abs(position_steps - count) < LATTICE_SNAP_STEPS
        below = on_lattice or count == math.floor(position_steps)
        return cls(count, 1 if below else -1)

    def settle_on_lattice(self, p: np.ndarray) -> np.ndarray:
        rounded = np.round(p)
        on_lattice = np.abs(p - rounded) < LATTICE_SNAP_STEPS
        off_index = np.where(on_lattice, -1, np.arange(p.size))
        last_off = np.maximum.accumulate(off_index)
        previous_off = np.where(
            last_off >= 0, p[np.maximum(last_off, 0)], self.last_off_lattice
        )
        if last_off[-1] >= 0:
            self.last_off_lattice = float(p[last_off[-1]])
        reached_from_above = previous_off > rounded
        return np.where(
            on_lattice,
            rounded - np.where(reached_from_above, LATTICE_SNAP_STEPS, 0.0),
            p,
        )


def lattice_steps(t: np.ndarray, p: np.ndarray, state: PlayState) -> StepEvents:
    p = state.settle_on_lattice(p)
    floors = np.floor(p).astype(np.int64)
    jumps = np.diff(floors)
    at = np.flatnonzero(jumps)
    if at.size == 0:
        return StepEvents.empty()
    sizes = np.abs(jumps[at])
    rep = np.repeat(at, sizes)
    within = np.arange(rep.size) - np.repeat(np.cumsum(sizes) - sizes, sizes)
    direction = np.sign(jumps[rep]).astype(np.int8)
    level = np.where(
        direction > 0, floors[rep] + 1 + within, floors[rep] - within
    )
    p0, p1 = p[rep], p[rep + 1]
    frac = (level - p0) / (p1 - p0)
    times = t[rep] + frac * (t[rep + 1] - t[rep])
    previous = np.concatenate(([state.last_direction], direction[:-1]))
    fires = direction == previous
    state.last_direction = int(direction[-1])
    rate_hz = np.abs(p1 - p0) / (t[rep + 1] - t[rep])
    events = StepEvents(
        times[fires], level[fires], direction[fires], rate_hz[fires]
    )
    if len(events):
        state.count = int(events.level[-1])
    return events


def block_steps(
    block: ReferenceBlock, role: str, state: PlayState, lattice_origin: float
) -> StepEvents:
    pva = block.motor_pva_steps[role]
    times = block.sample_times
    parts = []
    for lo in range(0, len(times) - 1, REFERENCE_CHUNK_INTERVALS):
        hi = min(lo + REFERENCE_CHUNK_INTERVALS, len(times) - 1)
        fine_t, fine_p = upsample(
            times[lo : hi + 1], pva[lo : hi + 1], REFERENCE_FINE_FACTOR
        )
        part = lattice_steps(fine_t, fine_p - lattice_origin, state)
        part.time += block.stream_offset_s
        parts.append(part)
    return StepEvents.concat(parts)


def _lattice_origins(motor: Motor, blocks: list[ReferenceBlock]) -> list[float]:
    if not motor.track_restarts_per_block:
        return [0.0] * len(blocks)
    origins = []
    live_position = blocks[0].motor_pva_steps[motor.role][0, 0]
    for block in blocks:
        track = block.motor_pva_steps[motor.role][:, 0]
        origins.append(track[0] - live_position)
        live_position += track[-1] - track[0]
    return origins


def _motor_steps(
    motor: Motor, blocks: list[ReferenceBlock]
) -> tuple[StepEvents, int, list[float]]:
    origins = _lattice_origins(motor, blocks)
    state = PlayState.resting_at(blocks[0].motor_pva_steps[motor.role][0, 0])
    parts = [
        block_steps(block, motor.role, state, origin)
        for block, origin in zip(blocks, origins)
    ]
    return StepEvents.concat(parts), state.count, origins


def planned_blocks(job: Job, planned: PlannedPrint) -> list[ReferenceBlock]:
    blocks = []
    offset = 0.0
    for epoch in stream_epochs(
        job,
        planned,
        job.config.config_text,
        [0, 1, 2, 3],
        REFERENCE_SAMPLE_PERIOD_S,
    ):
        block = reference_block(job, epoch, offset)
        blocks.append(block)
        offset += block.duration_s + REFERENCE_BLOCK_SEPARATION_S
    return blocks


def dispatched_runs(
    record: dispatch_record.DispatchRecord,
) -> list[tuple[int, int]]:
    """The record's segments as runs of continuous motion: a run ends where
    the stream drained to rest, re-anchored, or left a gap in host time."""
    count = len(record)
    starts = np.zeros(count, bool)
    starts[0] = True
    starts[1:] |= (record.flags[1:] & dispatch_record.FLAG_FRESH_EPOCH) != 0
    starts[1:] |= (
        np.abs(record.host_start()[1:] - record.host_end()[:-1])
        > DISPATCH_SEAM_SLACK_S
    )
    drained = [
        d.after_segments for d in record.drains if 0 < d.after_segments < count
    ]
    starts[drained] = True
    bounds = [*np.flatnonzero(starts).tolist(), count]
    return list(zip(bounds[:-1], bounds[1:]))


def dispatched_blocks(
    job: Job, record: dispatch_record.DispatchRecord
) -> list[ReferenceBlock]:
    origin = float(record.host_start()[0])
    blocks = []
    for lo, hi in dispatched_runs(record):
        times, rows = [], []
        for index in range(lo, hi):
            samples = record.segment_samples(index)
            n = len(samples)
            local = record.t_start[index] + (
                record.t_end[index] - record.t_start[index]
            ) * np.arange(n) / (n - 1)
            local[-1] = record.t_end[index]
            keep = n if index == hi - 1 else n - 1
            times.append(record.t0[index] + local[:keep])
            rows.append(samples[:keep])
        host = np.concatenate(times)
        axes_pva = np.concatenate(rows).transpose(1, 0, 2)
        blocks.append(
            ReferenceBlock(
                stream_offset_s=float(host[0]) - origin,
                duration_s=float(host[-1] - host[0]),
                sample_times=host - host[0],
                motor_pva_steps={
                    m.role: _motor_pva_steps(m, axes_pva) for m in job.motors
                },
            )
        )
    return blocks


def build_reference(job: Job) -> Reference:
    planned = plan_waypoints(job)
    record = job.dispatch_record
    if record is not None and record.pacer_brakes():
        source, blocks = REFERENCE_DISPATCHED, dispatched_blocks(job, record)
    else:
        source, blocks = REFERENCE_PLANNED, planned_blocks(job, planned)
    events: dict[str, StepEvents] = {}
    start_counts: dict[str, int] = {}
    final_counts: dict[str, int] = {}
    level_offsets: dict[str, list[float]] = {}
    for motor in job.motors:
        first = blocks[0].motor_pva_steps[motor.role][0, 0]
        start_counts[motor.role] = PlayState.resting_at(first).count
        (
            events[motor.role],
            final_counts[motor.role],
            level_offsets[motor.role],
        ) = _motor_steps(motor, blocks)
    return Reference(
        source=source,
        start_xyz=planned.start_xyz,
        final_xyze=planned.final_xyze,
        blocks=blocks,
        events=events,
        start_counts=start_counts,
        final_counts=final_counts,
        level_offsets=level_offsets,
    )


def planned_dispatch_record(job: Job) -> dispatch_record.DispatchRecord:
    engine = harness._import_engine()
    try:
        data = engine.pipeline_snapshot_dispatch_record(
            plan_waypoints(job).waypoints,
            job.config.config_text,
            job.step_benchmarks,
        )
    except ValueError as exc:
        raise AnalysisError(f"print.gcode: {exc}") from exc
    return dispatch_record.parse(bytes(data), "plan")


def _motor_positions_steps(
    job: Job, record: dispatch_record.DispatchRecord
) -> np.ndarray:
    per_step = np.array(
        [np.asarray(m.axis_weights) / m.step_mm for m in job.motors]
    ).T
    return record.samples[:, :, 0] @ per_step


@dataclass(frozen=True)
class SegmentDeviation:
    leaders_steps: float
    followers_steps: float


@dataclass
class DispatchComparison:
    job: Job
    live: dispatch_record.DispatchRecord
    plan: dispatch_record.DispatchRecord
    live_steps: np.ndarray
    plan_steps: np.ndarray

    def deviation(self, i: int, j: int) -> SegmentDeviation:
        """How far live segment `i` strays from planned segment `j` in motor
        steps, wherever both stand at the same instant of their own. A
        follower is compared from where the segment starts it."""
        live, plan = self.live, self.plan
        live_span = live.t_end[i] - live.t_start[i]
        plan_span = plan.t_end[j] - plan.t_start[j]
        a = self.live_steps[live.sample_start[i] : live.sample_start[i + 1]]
        b = self.plan_steps[plan.sample_start[j] : plan.sample_start[j + 1]]
        if (
            abs(live_span - plan_span) > DISPATCH_DURATION_TOLERANCE_S
            or a.shape != b.shape
        ):
            return SegmentDeviation(math.inf, math.inf)
        diff = np.abs(a - b)
        followers = [m.track_restarts_per_block for m in self.job.motors]
        leaders = [not f for f in followers]
        rebased = np.abs((a - a[0]) - (b - b[0]))[:, followers]
        return SegmentDeviation(
            float(diff[:, leaders].max(initial=0.0)),
            float(rebased.max(initial=0.0)),
        )

    def extruder_offset_mm(self, i: int, j: int) -> float:
        return float(
            self.live.samples[self.live.sample_start[i], EXTRUDER_AXIS, 0]
            - self.plan.samples[self.plan.sample_start[j], EXTRUDER_AXIS, 0]
        )


def _merge_runs(
    runs: list[tuple[int, int]], spans_s: np.ndarray
) -> list[tuple[int, int]]:
    """Unpaired or straying runs of segments joined across less than
    BRAKE_SETTLE_S of agreeing motion: the shaper and the extruder smooth a
    brake's stop and restart into a few neighbours, some of which still
    happen to land within tolerance of the plan."""
    merged: list[list[int]] = []
    for lo, hi in sorted(runs):
        if merged and spans_s[merged[-1][1] : lo].sum() <= BRAKE_SETTLE_S:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    return [(lo, hi) for lo, hi in merged]


def compare_dispatch_with_plan(
    job: Job, live: dispatch_record.DispatchRecord
) -> dict:
    """Pair the live record's segments with the plan's by source line and
    hold each pair to the position-equivalent tolerance. A run of segments
    that pairs with nothing, or strays, is a brake window when a pacer brake
    drained the stream inside it and a divergence otherwise.

    The extruder is held to the plan only until the first brake. A brake
    stops on a corner the plan cuts, so the path it lays, and with it the
    extrusion budget the follower spreads over what follows, differs from
    the plan's for the rest of the print; how far is reported, not judged."""
    plan = planned_dispatch_record(job)
    comparison = DispatchComparison(
        job,
        live,
        plan,
        _motor_positions_steps(job, live),
        _motor_positions_steps(job, plan),
    )
    matcher = difflib.SequenceMatcher(
        None,
        live.source_line.tolist(),
        plan.source_line.tolist(),
        autojunk=False,
    )
    brakes = [d.after_segments for d in live.pacer_brakes()]
    first_brake = min(brakes, default=len(live))
    runs: list[tuple[int, int]] = []
    unpaired: set[int] = set()
    strays: dict[int, float] = {}
    matched = 0
    extruder_offset_mm = 0.0
    replanned_extruder_steps = 0.0
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag != "equal":
            runs.append((i1, i2))
            unpaired.update(range(i1, i2))
            continue
        for i, j in zip(range(i1, i2), range(j1, j2)):
            deviation = comparison.deviation(i, j)
            followers_held = i < first_brake
            stray = max(
                deviation.leaders_steps,
                deviation.followers_steps if followers_held else 0.0,
            )
            if stray > POSITION_EQUIVALENT_TOLERANCE_STEPS:
                runs.append((i, i + 1))
                strays[i] = stray
                continue
            matched += 1
            if not followers_held:
                replanned_extruder_steps = max(
                    replanned_extruder_steps, deviation.followers_steps
                )
            extruder_offset_mm = max(
                extruder_offset_mm, abs(comparison.extruder_offset_mm(i, j))
            )
    origin = float(live.host_start()[0])
    spans = live.t_end - live.t_start
    windows, divergences = [], []
    for lo, hi in _merge_runs(runs, spans):
        braked = [b for b in brakes if lo <= b <= hi]
        at = min(lo, len(live) - 1)
        window = {
            "segments": [lo, hi],
            "source_lines": [
                int(live.source_line[at]),
                int(live.source_line[max(at, hi - 1)]),
            ],
            "host_s": float(live.host_start()[at]) - origin,
            "motion_s": float(spans[lo:hi].sum()),
        }
        if braked:
            windows.append({**window, "pacer_brakes": len(braked)})
            continue
        stray = [strays[i] for i in range(lo, hi) if i in strays]
        divergences.append(
            {
                **window,
                "max_deviation_steps": max(stray, default=None),
                "straying_segments": len(stray),
                "unpaired_segments": len(unpaired.intersection(range(lo, hi))),
            }
        )
    return {
        "segments": len(live),
        "planned_segments": len(plan),
        "matched_segments": matched,
        "pacer_brakes": len(brakes),
        "requested_drains": len(live.drains) - len(brakes),
        "brake_windows": windows,
        "max_brake_window_motion_s": max(
            (w["motion_s"] for w in windows), default=0.0
        ),
        "max_extruder_offset_mm": extruder_offset_mm,
        "max_extruder_deviation_after_brakes_steps": replanned_extruder_steps,
        "divergences": len(divergences),
        "first_divergences": divergences[:REPORTED_DIVERGENCES_LIMIT],
        "matches_plan_outside_brakes": not divergences,
    }


def capture_steps(
    capture: Capture, motor: Motor, start_count: int
) -> StepEvents:
    if capture.exact_steps is not None:
        times, signs = capture.exact_steps[motor.channel]
        direction = (signs * motor.dir_sign).astype(np.int8)
        return StepEvents(
            time=times,
            level=start_count + np.cumsum(direction, dtype=np.int64),
            direction=direction,
            rate_hz=np.full(times.size, np.nan),
        )
    deltas = capture.deltas[:, motor.channel].astype(np.int64) * motor.dir_sign
    ticks = np.flatnonzero(deltas)
    if ticks.size == 0:
        return StepEvents.empty()
    sizes = np.abs(deltas[ticks])
    rep = np.repeat(ticks, sizes)
    per_tick = np.repeat(sizes, sizes)
    within = np.arange(rep.size) - np.repeat(np.cumsum(sizes) - sizes, sizes)
    direction = np.sign(deltas[rep]).astype(np.int8)
    count_before = start_count + np.cumsum(deltas)[rep] - deltas[rep]
    return StepEvents(
        time=(rep + (within + 0.5) / per_tick) / capture.tick_hz,
        level=count_before + direction * (within + 1),
        direction=direction,
        rate_hz=np.full(rep.size, np.nan),
    )


def capture_counts(capture: Capture, motor: Motor, start_count: int):
    deltas = capture.deltas[:, motor.channel].astype(np.int64) * motor.dir_sign
    return start_count + np.cumsum(deltas)


def find_epochs(events: dict[str, StepEvents], gap_s: float) -> list[Epoch]:
    times = np.sort(np.concatenate([ev.time for ev in events.values()]))
    if times.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(times) > gap_s)
    starts = np.concatenate(([times[0]], times[breaks + 1]))
    ends = np.concatenate((times[breaks], [times[-1]]))
    return [Epoch(float(s), float(e)) for s, e in zip(starts, ends)]


def split_epochs_by_reference_counts(
    capture_events: dict[str, StepEvents],
    capture_epochs: list[Epoch],
    reference_events: dict[str, StepEvents],
    reference_epochs: list[Epoch],
) -> list[Epoch]:
    if len(capture_epochs) >= len(reference_epochs):
        return capture_epochs
    capture_times = np.sort(
        np.concatenate([e.time for e in capture_events.values()])
    )
    reference_times = np.concatenate(
        [e.time for e in reference_events.values()]
    )
    counts = [
        int(
            np.count_nonzero(
                (reference_times >= ref.start_s)
                & (reference_times <= ref.end_s)
            )
        )
        for ref in reference_epochs
    ]
    slack = EPOCH_SPLIT_COUNT_SLACK * len(reference_epochs)
    if abs(sum(counts) - capture_times.size) > slack:
        return capture_epochs
    gaps = np.diff(capture_times)
    cuts = []
    for target in np.cumsum(counts)[:-1]:
        lo = max(1, int(target) - EPOCH_SPLIT_COUNT_SLACK)
        hi = min(capture_times.size - 1, int(target) + EPOCH_SPLIT_COUNT_SLACK)
        cut = lo + int(np.argmax(gaps[lo - 1 : hi]))
        if gaps[cut - 1] < EPOCH_SPLIT_MIN_GAP_S:
            raise AnalysisError(
                f"no rest of at least {EPOCH_SPLIT_MIN_GAP_S * 1e3:g} ms near "
                f"capture step {int(target)} (t={capture_times[cut]:.3f}s) "
                "separates the reference blocks"
            )
        cuts.append(cut)
    bounds = [0, *cuts, capture_times.size]
    return [
        Epoch(float(capture_times[lo]), float(capture_times[hi - 1]))
        for lo, hi in zip(bounds[:-1], bounds[1:])
    ]


def _describe_epochs(epochs: list[Epoch]) -> str:
    return ", ".join(f"[{e.start_s:.3f}s +{e.duration_s:.3f}s]" for e in epochs)


def pair_epochs(
    capture_epochs: list[Epoch], reference_epochs: list[Epoch]
) -> list[tuple[Epoch, Epoch]]:
    if len(capture_epochs) != len(reference_epochs):
        raise AnalysisError(
            f"{len(capture_epochs)} capture epochs vs {len(reference_epochs)} "
            f"reference epochs — capture: {_describe_epochs(capture_epochs)}; "
            f"reference: {_describe_epochs(reference_epochs)}"
        )
    for index, (cap, ref) in enumerate(zip(capture_epochs, reference_epochs)):
        tolerance = max(
            EPOCH_DURATION_TOLERANCE_S,
            EPOCH_DURATION_TOLERANCE_FRACTION * ref.duration_s,
        )
        if abs(cap.duration_s - ref.duration_s) > tolerance:
            raise AnalysisError(
                f"epoch {index}: capture lasts {cap.duration_s:.3f}s, "
                f"reference {ref.duration_s:.3f}s (tolerance {tolerance:.3f}s)"
            )
    return list(zip(capture_epochs, reference_epochs))


def _step_keys(events: StepEvents) -> np.ndarray:
    return events.level * 2 + (events.direction > 0)


def _key_time_composite(
    query_key: np.ndarray,
    query_t: np.ndarray,
    ref_key: np.ndarray,
    ref_t: np.ndarray,
    margin_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    keys, rank = np.unique(
        np.concatenate([ref_key, query_key]), return_inverse=True
    )
    ref_rank, query_rank = rank[: ref_key.size], rank[ref_key.size :]
    t_floor = min(ref_t.min(), query_t.min()) - margin_s
    span_ns = (
        int(
            np.ceil(
                (max(ref_t.max(), query_t.max()) - t_floor + margin_s)
                * NS_PER_S
            )
        )
        + 1
    )
    if keys.size * span_ns >= 1 << 62:
        raise AnalysisError(
            f"{keys.size} step keys over {span_ns} ns overflow the key/time index"
        )
    ref_ns = np.rint((ref_t - t_floor) * NS_PER_S).astype(np.int64)
    query_ns = np.rint((query_t - t_floor) * NS_PER_S).astype(np.int64)
    order = np.argsort(ref_rank * span_ns + ref_ns, kind="stable")
    ref_composite = (ref_rank * span_ns + ref_ns)[order]
    query_composite = query_rank * span_ns + query_ns
    return order, ref_composite, query_composite, span_ns


def nearest_same_key(
    query_key: np.ndarray,
    query_t: np.ndarray,
    ref_key: np.ndarray,
    ref_t: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    nearest = np.full(query_t.size, -1, np.int64)
    delta = np.full(query_t.size, np.inf)
    if query_t.size == 0 or ref_t.size == 0:
        return nearest, delta
    order, ref_composite, query_composite, _ = _key_time_composite(
        query_key, query_t, ref_key, ref_t, 0.0
    )
    sorted_key, sorted_t = ref_key[order], ref_t[order]
    position = np.searchsorted(ref_composite, query_composite)
    for candidate in (position - 1, position):
        valid = (candidate >= 0) & (candidate < sorted_t.size)
        safe = np.where(valid, candidate, 0)
        valid &= sorted_key[safe] == query_key
        candidate_delta = np.where(valid, query_t - sorted_t[safe], np.inf)
        better = np.abs(candidate_delta) < np.abs(delta)
        nearest = np.where(better, order[safe], nearest)
        delta = np.where(better, candidate_delta, delta)
    return nearest, delta


@dataclass
class EpochSteps:
    capture: dict[str, StepEvents]
    reference: dict[str, StepEvents]

    def roles(self) -> list[str]:
        return [
            role
            for role, ev in self.capture.items()
            if len(ev) and len(self.reference[role])
        ]


def match_capture_to_reference(
    steps: EpochSteps, role: str, alignment: Alignment
) -> tuple[np.ndarray, np.ndarray]:
    cap, ref = steps.capture[role], steps.reference[role]
    return nearest_same_key(
        _step_keys(cap),
        cap.time,
        _step_keys(ref),
        alignment.to_capture(ref.time),
    )


def fit_alignment(
    t_ref: np.ndarray, t_cap: np.ndarray, fixed_ppm: float | None
) -> Alignment:
    if fixed_ppm is None:
        center = float(np.mean(t_ref))
        slope, intercept = np.polyfit(t_ref - center, t_cap, 1)
        return Alignment(
            offset_s=float(intercept - slope * center),
            ppm=float((slope - 1.0) * 1e6),
            scale_fitted=True,
        )
    slope = 1.0 + fixed_ppm * 1e-6
    return Alignment(
        offset_s=float(np.mean(t_cap - slope * t_ref)),
        ppm=fixed_ppm,
        scale_fitted=False,
    )


def _inliers(residual: np.ndarray) -> np.ndarray:
    median = np.median(residual)
    spread = 1.4826 * np.median(np.abs(residual - median))
    return np.abs(residual - median) < max(INLIER_FLOOR_S, 5.0 * spread)


def _matched_pairs(
    steps: EpochSteps, alignment: Alignment, window_s: float
) -> tuple[np.ndarray, np.ndarray]:
    t_ref_parts, t_cap_parts = [np.empty(0)], [np.empty(0)]
    for role in steps.roles():
        nearest, delta = match_capture_to_reference(steps, role, alignment)
        close = np.abs(delta) < window_s
        t_cap_parts.append(steps.capture[role].time[close])
        t_ref_parts.append(steps.reference[role].time[nearest[close]])
    return np.concatenate(t_ref_parts), np.concatenate(t_cap_parts)


def _modal_alignment(
    steps: EpochSteps,
    capture_epoch: Epoch,
    reference_epoch: Epoch,
    ppm: float,
) -> Alignment:
    guess = Alignment(
        capture_epoch.start_s - (1.0 + ppm * 1e-6) * reference_epoch.start_s,
        ppm,
        False,
    )
    t_ref, t_cap = _matched_pairs(steps, guess, OFFSET_SEARCH_S)
    if t_ref.size < MIN_FIT_PAIRS:
        raise AnalysisError(
            f"epoch at capture {capture_epoch.start_s:.3f}s: only "
            f"{t_ref.size} steps have a reference counterpart within "
            f"{OFFSET_SEARCH_S:g}s; cannot align"
        )
    edges = np.arange(
        -OFFSET_SEARCH_S,
        OFFSET_SEARCH_S + OFFSET_SEARCH_BIN_S,
        OFFSET_SEARCH_BIN_S,
    )
    histogram = np.histogram(t_cap - guess.to_capture(t_ref), edges)[0]
    mode = int(np.argmax(histogram))
    return Alignment(
        guess.offset_s + 0.5 * (edges[mode] + edges[mode + 1]), ppm, False
    )


def align_epoch(
    steps: EpochSteps,
    capture_epoch: Epoch,
    reference_epoch: Epoch,
    fixed_ppm: float | None,
) -> Alignment:
    alignment = _modal_alignment(
        steps, capture_epoch, reference_epoch, fixed_ppm or 0.0
    )
    for window in FIT_WINDOWS_S:
        t_ref, t_cap = _matched_pairs(steps, alignment, window)
        if t_ref.size < MIN_FIT_PAIRS:
            raise AnalysisError(
                f"epoch at capture {capture_epoch.start_s:.3f}s: only "
                f"{t_ref.size} steps matched the reference within "
                f"{window * 1e3:.1f} ms; cannot align"
            )
        keep = _inliers(t_cap - alignment.to_capture(t_ref))
        alignment = fit_alignment(t_ref[keep], t_cap[keep], fixed_ppm)
    return alignment


def epoch_steps(
    capture_events: dict[str, StepEvents],
    reference_events: dict[str, StepEvents],
    capture_epoch: Epoch,
    reference_epoch: Epoch,
) -> EpochSteps:
    return EpochSteps(
        capture={
            r: ev.between(capture_epoch.start_s, capture_epoch.end_s)
            for r, ev in capture_events.items()
        },
        reference={
            r: ev.between(reference_epoch.start_s, reference_epoch.end_s)
            for r, ev in reference_events.items()
        },
    )


PositionWindow = Callable[[float, float], "np.ndarray | None"]


def _counts_after(
    events: StepEvents, event_times: np.ndarray, at: np.ndarray
) -> np.ndarray:
    index = np.searchsorted(event_times, at, "right") - 1
    return np.where(
        index >= 0, events.level[np.maximum(index, 0)], events.start_count()
    )


def _stays_in_count_band(
    positions: PositionWindow,
    start_s: float,
    end_s: float,
    counts: tuple[int, int],
) -> bool:
    path = positions(start_s, end_s)
    return path is not None and bool(
        np.all(path > min(counts) - EDGE_TIE_STEPS)
        and np.all(path < max(counts) + EDGE_TIE_STEPS)
    )


def _steps_near_their_lines(
    positions: PositionWindow, times: np.ndarray, lines: np.ndarray
) -> bool:
    for t, line in zip(times, lines):
        path = positions(t, t)
        if path is None or np.any(np.abs(path - line) >= EDGE_TIE_STEPS):
            return False
    return True


def edge_ties(
    capture: StepEvents,
    capture_stream_s: np.ndarray,
    capture_unmatched: np.ndarray,
    reference: StepEvents,
    reference_unmatched: np.ndarray,
    positions: PositionWindow,
    trajectory_s: tuple[float, float] | None,
) -> tuple[np.ndarray, np.ndarray, int]:
    capture_tie = np.zeros(len(capture), bool)
    reference_tie = np.zeros(len(reference), bool)
    rest_ties = 0
    event_times = np.union1d(capture_stream_s, reference.time)
    times = np.concatenate(([-math.inf], event_times))
    capture_counts = np.concatenate(
        (
            [capture.start_count()],
            _counts_after(capture, capture_stream_s, event_times),
        )
    )
    reference_counts = np.concatenate(
        (
            [reference.start_count()],
            _counts_after(reference, reference.time, event_times),
        )
    )
    disagreement = capture_counts - reference_counts
    capture_open = np.flatnonzero(capture_unmatched)
    reference_open = np.flatnonzero(reference_unmatched)
    capture_open_s = capture_stream_s[capture_open]
    reference_open_s = reference.time[reference_open]
    for lo, hi in _disagreement_spans(times, disagreement):
        start_s = times[lo]
        end_s = times[hi] if hi < times.size else math.inf
        in_capture = capture_open[
            np.searchsorted(capture_open_s, start_s, "left") : np.searchsorted(
                capture_open_s, end_s, "right"
            )
        ]
        in_reference = reference_open[
            np.searchsorted(
                reference_open_s, start_s, "left"
            ) : np.searchsorted(reference_open_s, end_s, "right")
        ]
        if not (in_capture.size or in_reference.size):
            continue
        if np.any(np.abs(disagreement[lo:hi]) > 1):
            continue
        if lo == 0 or hi == times.size:
            if (
                trajectory_s is not None
                and in_capture.size + in_reference.size == 1
                and _rests_on_lattice_line(
                    positions, trajectory_s[0 if lo == 0 else 1]
                )
            ):
                capture_tie[in_capture] = True
                reference_tie[in_reference] = True
                rest_ties += 1
            continue
        band_holds = all(
            _stays_in_count_band(
                positions,
                times[k],
                times[k + 1],
                (int(capture_counts[k]), int(reference_counts[k])),
            )
            for k in range(lo, hi)
        )
        if band_holds and _steps_near_their_lines(
            positions,
            capture_stream_s[in_capture],
            capture.level[in_capture],
        ):
            capture_tie[in_capture] = True
            reference_tie[in_reference] = True
    return capture_tie, reference_tie, rest_ties


def _rests_on_lattice_line(positions: PositionWindow, rest_s: float) -> bool:
    path = positions(rest_s, rest_s)
    return path is not None and bool(
        abs(path[-1] - round(path[-1])) < LATTICE_SNAP_STEPS
    )


def _disagreement_spans(
    times: np.ndarray, disagreement: np.ndarray
) -> list[tuple[int, int]]:
    spans: list[tuple[int, int]] = []
    for lo, hi in _runs(disagreement != 0):
        agreed_s = times[lo] - times[spans[-1][1]] if spans else math.inf
        if agreed_s < TIMING_TOLERANCE_US * 1e-6:
            spans[-1] = (spans[-1][0], hi)
        else:
            spans.append((lo, hi))
    return spans


def same_key_count(
    query_key: np.ndarray,
    query_t: np.ndarray,
    ref_key: np.ndarray,
    ref_t: np.ndarray,
) -> np.ndarray:
    if query_t.size == 0 or ref_t.size == 0:
        return np.zeros(query_t.size, np.int64)
    _, ref_composite, query_composite, _ = _key_time_composite(
        query_key, query_t, ref_key, ref_t, REPORT_MATCH_WINDOW_S
    )
    window_ns = round(REPORT_MATCH_WINDOW_S * NS_PER_S)
    return np.searchsorted(
        ref_composite, query_composite + window_ns, "right"
    ) - np.searchsorted(ref_composite, query_composite - window_ns, "left")


def _is_pair(
    offset_s: np.ndarray, own_rivals: np.ndarray, partner_rivals: np.ndarray
) -> np.ndarray:
    unambiguous = (own_rivals == 1) & (partner_rivals == 1)
    window = np.where(unambiguous, REPORT_MATCH_WINDOW_S, PAIR_WINDOW_S)
    return np.abs(offset_s) < window


def measure_epoch(
    steps: EpochSteps, alignment: Alignment, reference: Reference
) -> dict[str, MatchedSteps]:
    matches = {}
    for role, cap in steps.capture.items():
        ref = steps.reference[role]
        if not len(cap) or not len(ref):
            matches[role] = MatchedSteps(
                np.empty(0), np.empty(0), np.empty(0), len(cap), len(ref), 0
            )
            continue
        nearest, late = match_capture_to_reference(steps, role, alignment)
        ref_capture_t = alignment.to_capture(ref.time)
        reverse_nearest, reverse = nearest_same_key(
            _step_keys(ref), ref_capture_t, _step_keys(cap), cap.time
        )
        capture_rivals = same_key_count(
            _step_keys(cap), cap.time, _step_keys(ref), ref_capture_t
        )
        reference_rivals = same_key_count(
            _step_keys(ref), ref_capture_t, _step_keys(cap), cap.time
        )
        matched = _is_pair(
            late, capture_rivals, reference_rivals[np.maximum(nearest, 0)]
        )
        reference_unmatched = ~_is_pair(
            reverse,
            reference_rivals,
            capture_rivals[np.maximum(reverse_nearest, 0)],
        )
        capture_tie, reference_tie, rest_ties = edge_ties(
            cap,
            alignment.to_stream(cap.time),
            ~matched,
            ref,
            reference_unmatched,
            lambda lo, hi, role=role: reference.positions(role, lo, hi),
            reference.trajectory_span_s(float(ref.time[0])),
        )
        matches[role] = MatchedSteps(
            late_s=late[matched],
            capture_time_s=cap.time[matched],
            reference_rate_hz=ref.rate_hz[nearest[matched]],
            unmatched_capture=int(np.count_nonzero(~matched & ~capture_tie)),
            unmatched_reference=int(
                np.count_nonzero(reference_unmatched & ~reference_tie)
            ),
            edge_ties=int(
                np.count_nonzero(capture_tie) + np.count_nonzero(reference_tie)
            ),
            rest_ties=rest_ties,
        )
    return matches


def conditioned_span(epoch: EpochResult) -> dict:
    matches = epoch.matches.values()
    rate = np.concatenate([m.reference_rate_hz for m in matches])
    keep = rate >= CONDITIONED_RATE_HZ
    if not keep.any():
        return {}
    capture_t = np.concatenate([m.capture_time_s for m in matches])[keep]
    late = np.concatenate([m.late_s for m in matches])[keep]
    stream_t = epoch.alignment.to_stream(capture_t - late)
    first, last = int(np.argmin(capture_t)), int(np.argmax(capture_t))
    capture_span = float(capture_t[last] - capture_t[first])
    stream_span = float(stream_t[last] - stream_t[first])
    return {
        "conditioned_span_capture_s": capture_span,
        "conditioned_span_stream_s": stream_span,
        "conditioned_span_capture_minus_stream_ms": (capture_span - stream_span)
        * 1e3,
    }


def paired_steps_s(epoch: EpochResult) -> tuple[np.ndarray, np.ndarray]:
    """Every capture step matched to its reference step where the reference
    moves at least CONDITIONED_RATE_HZ, as (capture, reference) times in
    capture order. Pairing by match, not by index, keeps a tie step at an
    edge (withheld at a rest on one side, taken on the other) from shifting
    every later step against the wrong partner; a crawling step's time says
    little about pace."""
    matches = epoch.matches.values()
    conditioned = np.concatenate([m.reference_rate_hz for m in matches]) >= (
        CONDITIONED_RATE_HZ
    )
    capture = np.concatenate([m.capture_time_s for m in matches])[conditioned]
    late = np.concatenate([m.late_s for m in matches])[conditioned]
    if capture.size == 0:
        raise AnalysisError(
            f"epoch at capture {epoch.capture.start_s:.3f}s matched no step "
            f"moving at {CONDITIONED_RATE_HZ:g} Hz or faster"
        )
    reference = epoch.alignment.to_stream(capture - late)
    order = np.argsort(capture, kind="stable")
    return capture[order], reference[order]


def epoch_pace(capture_s: np.ndarray, reference_s: np.ndarray) -> dict:
    capture = capture_s - capture_s[0]
    stream = reference_s - reference_s[0]
    lag = capture - stream
    allowed = PACE_TOLERANCE_S + PACE_TOLERANCE_PPM * 1e-6 * stream
    worst = int(np.argmax(np.abs(lag) - allowed))
    return {
        "pace_capture_over_stream": float(capture[-1] / stream[-1])
        if stream[-1] > 0.0
        else 1.0,
        "pace_max_abs_lag_ms": float(np.abs(lag).max() * 1e3),
        "pace_worst_lag_ms": float(lag[worst] * 1e3),
        "pace_worst_at_stream_s": float(stream[worst]),
        "pace_within_tolerance": bool(np.all(np.abs(lag) <= allowed)),
    }


def residual_wander_us(matches: dict[str, MatchedSteps]) -> float:
    late = np.concatenate([m.late_s for m in matches.values()])
    times = np.concatenate([m.capture_time_s for m in matches.values()])
    if late.size == 0:
        return 0.0
    keep = _inliers(late)
    late, times = late[keep], times[keep]
    bins = np.floor((times - times.min()) / WANDER_WINDOW_S).astype(np.int64)
    medians = [
        np.median(late[bins == b])
        for b in np.unique(bins)
        if np.count_nonzero(bins == b) >= MIN_FIT_PAIRS
    ]
    return float((max(medians) - min(medians)) * 1e6) if medians else 0.0


def nearest_fitted_ppm(fitted_ppm: list[float | None], index: int) -> float:
    """The rate an epoch too short to fit its own runs at. The live host
    re-syncs its host→mcu map at every rest, so the stream's rate against
    the mcu clock moves from epoch to epoch as the clock estimate
    converges; the latest fitted epoch holds the estimate nearest the one
    in force, and a median over the whole run can sit tens of ppm off it."""
    for ppm in reversed(fitted_ppm[:index]):
        if ppm is not None:
            return ppm
    for ppm in fitted_ppm[index + 1 :]:
        if ppm is not None:
            return ppm
    return 0.0


def align_all(
    capture_events: dict[str, StepEvents],
    reference: Reference,
    gap_s: float,
) -> list[EpochResult]:
    reference_epochs = find_epochs(reference.events, gap_s)
    pairs = pair_epochs(
        split_epochs_by_reference_counts(
            capture_events,
            find_epochs(capture_events, gap_s),
            reference.events,
            reference_epochs,
        ),
        reference_epochs,
    )
    steps = [
        epoch_steps(capture_events, reference.events, cap, ref)
        for cap, ref in pairs
    ]
    alignments: list[Alignment | None] = [None] * len(pairs)
    for index, (cap, ref) in enumerate(pairs):
        if cap.duration_s >= MIN_SCALE_FIT_DURATION_S:
            alignments[index] = align_epoch(steps[index], cap, ref, None)
    fitted_ppm = [None if a is None else a.ppm for a in alignments]
    results = []
    for index, (cap, ref) in enumerate(pairs):
        alignment = alignments[index] or align_epoch(
            steps[index], cap, ref, nearest_fitted_ppm(fitted_ppm, index)
        )
        results.append(
            EpochResult(
                capture=cap,
                reference=ref,
                alignment=alignment,
                steps=steps[index],
                matches=measure_epoch(steps[index], alignment, reference),
            )
        )
    return results


def reference_counts_at(
    events: StepEvents, start_count: int, t_stream: np.ndarray
) -> np.ndarray:
    if events.time.size == 0:
        return np.full(t_stream.shape, start_count)
    index = np.searchsorted(events.time, t_stream, "right") - 1
    return np.where(index >= 0, events.level[np.maximum(index, 0)], start_count)


@dataclass
class CountError:
    capture_time_s: np.ndarray
    error_steps: np.ndarray


def count_errors(
    capture: Capture,
    motor: Motor,
    reference: Reference,
    epochs: list[EpochResult],
) -> CountError:
    start = reference.start_counts[motor.role]
    counts = capture_counts(capture, motor, start)
    times_parts, error_parts = [], []
    for epoch in epochs:
        lo = max(
            0,
            int(
                (epoch.capture.start_s - COUNT_ERROR_MARGIN_S) * capture.tick_hz
            ),
        )
        hi = min(
            len(counts),
            int((epoch.capture.end_s + COUNT_ERROR_MARGIN_S) * capture.tick_hz)
            + 1,
        )
        tick_end_s = (np.arange(lo, hi) + 1) / capture.tick_hz
        expected = reference_counts_at(
            reference.events[motor.role],
            start,
            epoch.alignment.to_stream(tick_end_s),
        )
        times_parts.append(tick_end_s)
        error_parts.append(counts[lo:hi] - expected)
    return CountError(np.concatenate(times_parts), np.concatenate(error_parts))


@dataclass
class Stall:
    role: str
    kind: str
    capture_start_s: float
    stream_start_s: float
    duration_ms: float
    reference_steps: int
    capture_steps: int


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    padded = np.concatenate(([False], mask, [False])).astype(np.int8)
    edges = np.flatnonzero(np.diff(padded))
    return list(zip(edges[::2], edges[1::2]))


def find_stalls(epoch: EpochResult) -> list[Stall]:
    steps = epoch.steps
    edges = np.arange(
        epoch.capture.start_s - STALL_BIN_S,
        epoch.capture.end_s + 2 * STALL_BIN_S,
        STALL_BIN_S,
    )
    stalls = []
    for role, cap in steps.capture.items():
        ref_times = epoch.alignment.to_capture(steps.reference[role].time)
        cap_hist = np.histogram(cap.time, edges)[0]
        ref_hist = np.histogram(ref_times, edges)[0]
        for kind, mask in (
            (
                "capture_idle_reference_moving",
                (ref_hist >= STALL_MIN_STEPS) & (cap_hist == 0),
            ),
            (
                "capture_moving_reference_idle",
                (cap_hist >= STALL_MIN_STEPS) & (ref_hist == 0),
            ),
        ):
            for lo, hi in _runs(mask):
                stalls.append(
                    Stall(
                        role=role,
                        kind=kind,
                        capture_start_s=float(edges[lo]),
                        stream_start_s=float(
                            epoch.alignment.to_stream(edges[lo])
                        ),
                        duration_ms=float((hi - lo) * STALL_BIN_S * 1e3),
                        reference_steps=int(ref_hist[lo:hi].sum()),
                        capture_steps=int(cap_hist[lo:hi].sum()),
                    )
                )
    return stalls


def _timing_stats(late_s: np.ndarray, capture_time_s: np.ndarray) -> dict:
    if late_s.size == 0:
        return {}
    late_us = late_s * 1e6
    magnitude = np.abs(late_us)
    worst = int(np.argmax(magnitude))
    return {
        "p50": float(np.percentile(late_us, 50)),
        "p99": float(np.percentile(late_us, 99)),
        "abs_p50": float(np.percentile(magnitude, 50)),
        "abs_p99": float(np.percentile(magnitude, 99)),
        "abs_max": float(magnitude[worst]),
        "abs_max_at_capture_s": float(capture_time_s[worst]),
        "max_late": float(late_us.max()),
        "max_early": float(late_us.min()),
    }


def _step_at(
    late_s: np.ndarray,
    capture_time_s: np.ndarray,
    rate_hz: np.ndarray,
    index: int,
) -> dict:
    return {
        "capture_s": float(capture_time_s[index]),
        "timing_error_us": float(late_s[index] * 1e6),
        "reference_rate_hz": float(rate_hz[index]),
        "position_equivalent_steps": float(abs(late_s[index]) * rate_hz[index]),
    }


def timing_report(
    late_s: np.ndarray, capture_time_s: np.ndarray, rate_hz: np.ndarray
) -> dict:
    position_steps = np.abs(late_s) * rate_hz
    beyond_time_bound = np.flatnonzero(
        np.abs(late_s) * 1e6 > TIMING_TOLERANCE_US
    )
    beyond_position_bound = (
        position_steps[beyond_time_bound] > POSITION_EQUIVALENT_TOLERANCE_STEPS
    )
    return {
        "timing_error_capture_minus_reference_us": _timing_stats(
            late_s, capture_time_s
        ),
        "position_equivalent_error_abs_max": _step_at(
            late_s, capture_time_s, rate_hz, int(np.argmax(position_steps))
        )
        if late_s.size
        else {},
        "timing_beyond_time_bound_steps": int(beyond_time_bound.size),
        "timing_beyond_time_bound_worst_position": _step_at(
            late_s,
            capture_time_s,
            rate_hz,
            int(
                beyond_time_bound[np.argmax(position_steps[beyond_time_bound])]
            ),
        )
        if beyond_time_bound.size
        else {},
        "timing_violation_steps": int(np.count_nonzero(beyond_position_bound)),
    }


def _gcode_net_steps(motor: Motor, reference: Reference) -> int:
    weights = np.asarray(motor.axis_weights)
    start = float(weights @ np.array([*reference.start_xyz, 0.0]))
    final = float(weights @ np.asarray(reference.final_xyze))
    return round(final / motor.step_mm) - round(start / motor.step_mm)


def _peak_capture_rate_hz(capture: Capture, motor: Motor) -> float:
    activity = np.concatenate(
        (
            [0],
            np.cumsum(
                np.abs(capture.deltas[:, motor.channel].astype(np.int64))
            ),
        )
    )
    window = PEAK_RATE_WINDOW_TICKS
    if activity.size <= window:
        return 0.0
    busiest = (activity[window:] - activity[:-window]).max()
    return float(busiest * capture.tick_hz / window)


def _peak_reference_rate_hz(reference: Reference, role: str) -> float:
    return float(
        max(
            np.abs(b.motor_pva_steps[role][:, 1]).max()
            for b in reference.blocks
        )
    )


def corexy_report(
    inputs: RunInputs,
    reference: Reference,
    finals: dict[str, int],
    errors: dict[str, CountError],
) -> dict:
    motors = {m.role: m for m in inputs.motors}
    a_mm, b_mm = motors["a"].step_mm, motors["b"].step_mm
    if a_mm != b_mm:
        raise AnalysisError(
            f"corexy a/b step distances differ: {a_mm} vs {b_mm}"
        )
    x_err = (errors["a"].error_steps + errors["b"].error_steps) * a_mm / 2
    y_err = (errors["a"].error_steps - errors["b"].error_steps) * a_mm / 2
    captured = (
        (finals["a"] + finals["b"]) * a_mm / 2,
        (finals["a"] - finals["b"]) * a_mm / 2,
        finals["z"] * motors["z"].step_mm,
        finals["e"] * motors["e"].step_mm,
    )
    return {
        "final_xyze_captured_mm": [round(v, 6) for v in captured],
        "final_xyze_gcode_mm": list(reference.final_xyze),
        "final_xyz_vs_gcode_mm": [
            round(c - g, 6)
            for c, g in zip(captured[:3], reference.final_xyze[:3])
        ],
        "e_net_vs_gcode_corner_cut_mm": round(
            captured[3] - reference.final_xyze[3], 6
        ),
        "max_abs_x_error_mm": float(np.abs(x_err).max()) if x_err.size else 0.0,
        "max_abs_y_error_mm": float(np.abs(y_err).max()) if y_err.size else 0.0,
    }


def analyze(
    inputs: RunInputs, gap_s: float = DEFAULT_EPOCH_GAP_S
) -> tuple[dict, dict]:
    reference = build_reference(inputs)
    capture_events = {
        m.role: capture_steps(inputs.capture, m, reference.start_counts[m.role])
        for m in inputs.motors
    }
    epochs = align_all(capture_events, reference, gap_s)
    stalls = [s for e in epochs for s in find_stalls(e)]
    errors = {
        m.role: count_errors(inputs.capture, m, reference, epochs)
        for m in inputs.motors
    }
    finals = {
        m.role: int(
            reference.start_counts[m.role]
            + inputs.capture.deltas[:, m.channel].astype(np.int64).sum()
            * m.dir_sign
        )
        for m in inputs.motors
    }
    motors_report = {}
    for motor in inputs.motors:
        role = motor.role
        late = np.concatenate([e.matches[role].late_s for e in epochs])
        late_at = np.concatenate(
            [e.matches[role].capture_time_s for e in epochs]
        )
        rate = np.concatenate(
            [e.matches[role].reference_rate_hz for e in epochs]
        )
        conditioned = rate >= CONDITIONED_RATE_HZ
        gcode_net = _gcode_net_steps(motor, reference)
        error = errors[role]
        worst = (
            int(np.argmax(np.abs(error.error_steps)))
            if error.error_steps.size
            else None
        )
        captured_net = finals[role] - reference.start_counts[role]
        expected_net = (
            reference.final_counts[role] - reference.start_counts[role]
        )
        motors_report[role] = {
            "motor": motor.name,
            "channel": motor.channel,
            "step_mm": motor.step_mm,
            "net_steps_captured": captured_net,
            "net_steps_expected": expected_net,
            "net_steps_capture_minus_expected": captured_net - expected_net,
            "net_steps_gcode": gcode_net,
            "matched_steps": int(late.size),
            "unmatched_capture_steps": sum(
                e.matches[role].unmatched_capture for e in epochs
            ),
            "unmatched_reference_steps": sum(
                e.matches[role].unmatched_reference for e in epochs
            ),
            "edge_tie_steps": sum(e.matches[role].edge_ties for e in epochs),
            "rest_tie_steps": sum(e.matches[role].rest_ties for e in epochs),
            **timing_report(
                late[conditioned], late_at[conditioned], rate[conditioned]
            ),
            "timing_reference_rate_floor_hz": CONDITIONED_RATE_HZ,
            "timing_position_bound_steps": POSITION_EQUIVALENT_TOLERANCE_STEPS,
            "timing_position_bound_relaxes_below_hz": (
                POSITION_BOUND_RELAXES_BELOW_HZ
            ),
            "timing_steps_below_rate_floor": int(
                np.count_nonzero(~conditioned)
            ),
            "max_abs_count_error_steps": int(abs(error.error_steps[worst]))
            if worst is not None
            else 0,
            "max_abs_count_error_at_capture_s": float(
                error.capture_time_s[worst]
            )
            if worst is not None
            else None,
            "peak_step_rate_captured_hz": _peak_capture_rate_hz(
                inputs.capture, motor
            ),
            "peak_step_rate_reference_hz": _peak_reference_rate_hz(
                reference, role
            ),
            "max_steps_in_one_tick": int(
                np.abs(inputs.capture.deltas[:, motor.channel]).max()
            ),
        }
    paces = [epoch_pace(*paired_steps_s(e)) for e in epochs]
    checks = {
        "net_steps_match": all(
            abs(r["net_steps_capture_minus_expected"]) <= r["rest_tie_steps"]
            for r in motors_report.values()
        ),
        "no_stalls": not stalls,
        "all_steps_matched": all(
            r["unmatched_capture_steps"] == 0
            and r["unmatched_reference_steps"] == 0
            for r in motors_report.values()
        ),
        TIMING_CHECK: all(
            r["timing_violation_steps"] == 0 for r in motors_report.values()
        ),
        "pace_matches_reference": all(
            p["pace_within_tolerance"] for p in paces
        ),
    }
    record = inputs.dispatch_record
    dispatched = (
        None if record is None else compare_dispatch_with_plan(inputs, record)
    )
    if dispatched is not None:
        checks["no_pacer_brakes"] = dispatched["pacer_brakes"] == 0
        checks["dispatch_matches_plan_outside_brakes"] = dispatched[
            "matches_plan_outside_brakes"
        ]
    verdict = {
        "ok": all(checks.values()),
        "checks": checks,
        "reference_source": reference.source,
        "reference_step_capacity": inputs.step_benchmarks_source,
        "dispatch_record": dispatched,
        "alignment_model": "t_capture = offset_s + (1 + ppm*1e-6) * t_reference",
        "capture": {
            "ticks": len(inputs.capture.deltas),
            "tick_hz": inputs.capture.tick_hz,
            "duration_s": inputs.capture.duration_s,
        },
        "reference": {
            "blocks": len(reference.blocks),
            "block_durations_s": [b.duration_s for b in reference.blocks],
            "start_xyz_mm": list(reference.start_xyz),
            "final_xyze_mm": list(reference.final_xyze),
        },
        "epochs": [
            {
                "capture_start_s": e.capture.start_s,
                "capture_duration_s": e.capture.duration_s,
                "stream_start_s": e.reference.start_s,
                "stream_duration_s": e.reference.duration_s,
                **conditioned_span(e),
                **pace,
                "offset_s": e.alignment.offset_s,
                "ppm": e.alignment.ppm,
                "scale_fitted": e.alignment.scale_fitted,
                "matched_steps": int(
                    sum(m.late_s.size for m in e.matches.values())
                ),
                "residual_wander_us": residual_wander_us(e.matches),
            }
            for e, pace in zip(epochs, paces)
        ],
        "motors": motors_report,
        "corexy": corexy_report(inputs, reference, finals, errors),
        "mcu_execution": mcu_execution.report(
            inputs.run_dir, inputs.capture.events, inputs.motors
        ),
        "stalls": {
            "count": len(stalls),
            "items": [vars(s) for s in stalls[:REPORTED_STALLS_LIMIT]],
        },
    }
    detail = {
        "reference": reference,
        "capture_events": capture_events,
        "epochs": epochs,
        "errors": errors,
    }
    return verdict, detail


def save_plots(inputs: RunInputs, detail: dict) -> list[str]:
    try:
        import matplotlib
    except ModuleNotFoundError:
        return []
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    epochs: list[EpochResult] = detail["epochs"]
    reference: Reference = detail["reference"]
    errors: dict[str, CountError] = detail["errors"]
    roles = [m.role for m in inputs.motors]
    written = []

    fig, axes = plt.subplots(len(roles), 1, sharex=True, figsize=(14, 10))
    for ax, role in zip(axes, roles):
        err = errors[role]
        stride = max(1, err.error_steps.size // 200_000)
        ax.plot(err.capture_time_s[::stride], err.error_steps[::stride], lw=0.5)
        ax.set_ylabel(f"{role} count err [steps]")
    axes[-1].set_xlabel("capture time [s]")
    fig.suptitle("captured count minus reference count (aligned)")
    written.append(_save(fig, inputs.run_dir / "count_error.png", plt))

    fig, axes = plt.subplots(len(roles), 1, sharex=True, figsize=(14, 10))
    for ax, role in zip(axes, roles):
        for epoch in epochs:
            m = epoch.matches[role]
            ax.plot(m.capture_time_s, m.late_s * 1e6, ".", ms=1)
        ax.set_ylabel(f"{role} late [us]")
    axes[-1].set_xlabel("capture time [s]")
    fig.suptitle("step timing error: capture minus aligned reference")
    written.append(_save(fig, inputs.run_dir / "timing_error.png", plt))

    bin_ticks = 80
    fig, axes = plt.subplots(len(roles), 1, sharex=True, figsize=(14, 10))
    for ax, motor in zip(axes, inputs.motors):
        deltas = inputs.capture.deltas[:, motor.channel].astype(np.int64)
        usable = deltas.size // bin_ticks * bin_ticks
        binned = deltas[:usable].reshape(-1, bin_ticks).sum(1) * motor.dir_sign
        times = (
            (np.arange(binned.size) + 0.5) * bin_ticks / inputs.capture.tick_hz
        )
        ax.plot(
            times,
            binned * inputs.capture.tick_hz / bin_ticks,
            lw=0.5,
            label="capture",
        )
        for epoch in epochs:
            for block in reference.blocks:
                stream_t = block.sample_times + block.stream_offset_s
                inside = (
                    stream_t >= epoch.reference.start_s - COUNT_ERROR_MARGIN_S
                ) & (stream_t <= epoch.reference.end_s + COUNT_ERROR_MARGIN_S)
                ax.plot(
                    epoch.alignment.to_capture(stream_t[inside]),
                    block.motor_pva_steps[motor.role][inside, 1],
                    lw=0.5,
                    label="reference",
                )
        ax.set_ylabel(f"{motor.role} [steps/s]")
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("capture time [s]")
    fig.suptitle("motor velocity: capture (2 ms bins) vs aligned reference")
    written.append(_save(fig, inputs.run_dir / "velocity.png", plt))
    return written


def _save(fig, path: Path, plt) -> str:
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path.name


def deployed_engine_mismatch(
    run: dict, engine_build_id: str, analyze_argv: list[str]
) -> str | None:
    """Why the reference this engine plans is not what the run executed: the
    Pi planned the print with the rust/ sources `deploy.py sync` recorded,
    and a reference from any other planner holds the capture against motion
    the printer never commanded."""
    identity = run["deployed"]
    deployed = identity.get("rust_build_id")
    if deployed == engine_build_id:
        return None
    head = identity["head"]
    reason = (
        f"the run executed rust/ {deployed} ({identity.get('branch')} @ "
        f"{head}, {identity.get('worktree')}) but the local engine was built "
        f"from {engine_build_id}. Reanalyze it from a worktree at the "
        "deployed head:\n"
        f'  cd "$(scripts/wt.sh new reanalyze-{head[:12]} --base {head})"\n'
        "  make -f Makefile.rust motion-engine-fast\n"
        f"  uv run python tools/bench/analyze.py {shlex.join(analyze_argv)}"
    )
    dirty_paths = identity.get("dirty_paths")
    diff_sha256 = identity.get("diff_sha256")
    if dirty_paths or diff_sha256 != EMPTY_DIFF_SHA256:
        reason += (
            f"\nThe deployed tree was dirty (dirty paths {dirty_paths}, "
            f"git diff HEAD sha256 {diff_sha256}), so {head} alone may not "
            f"reproduce rust/ {deployed}."
        )
    return reason


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--gap",
        type=float,
        default=DEFAULT_EPOCH_GAP_S,
        help="idle seconds that split motion epochs",
    )
    parser.add_argument(
        "--corner-flow",
        action="store_true",
        help="also measure nozzle flow through every extruding corner",
    )
    parser.add_argument(
        "--step-benchmark",
        type=SuppliedBenchmark.parse,
        metavar="PULSED,BOTH_EDGE,CLOCK_HZ",
        help="cap the reference of a run whose run.json predates recording "
        "step benchmarks: the bench MCU's ticks per step pulsed and both-edge "
        "and its clock, every captured motor stepped pulsed",
    )
    args = parser.parse_args(argv)
    options = [
        token
        for token in (sys.argv[1:] if argv is None else argv)
        if Path(token) != args.run_dir
    ]
    mismatch = deployed_engine_mismatch(
        json.loads((args.run_dir / "run.json").read_text()),
        harness._import_engine().build_id(),
        [str(args.run_dir.resolve()), *options],
    )
    if mismatch is not None:
        raise AnalysisError(mismatch)
    inputs = load_run(args.run_dir, args.step_benchmark)
    verdict, detail = analyze(inputs, args.gap)
    if args.corner_flow:
        planned = plan_waypoints(inputs)
        verdict["corner_flow"] = corner_flow_bench.report(
            inputs,
            detail,
            planned.blocks,
            ROLE_AXIS_WEIGHTS,
            lambda config_text, axes, period: stream_epochs(
                inputs, planned, config_text, axes, period
            ),
        )
    verdict["plots"] = save_plots(inputs, detail)
    text = json.dumps(verdict, indent=2)
    (args.run_dir / "analysis.json").write_text(text + "\n")
    print(text)
    return 0 if verdict["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
