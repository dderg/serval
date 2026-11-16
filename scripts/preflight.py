#!/usr/bin/env python3
"""Plan a print offline through the production pipeline before printing it.

    uv run python scripts/preflight.py PRINT.gcode --config PRINTER.cfg \
        [--mesh PROFILE] [--no-pi]

klippy's own config reader and G-code dispatch run the print with the
printer's [gcode_macro]s and [bed_mesh] (scripts/preflight_gcode.py); what
reaches the motion planner replays through `gcode_replay_bench` under the
config's limits, kinematics and post-processor chains, over the mesh the
print loads: once synchronously with every emitted segment checked, once
staged on two cores for the realtime factor, scaled to a Pi 5 with the
throughput gate's calibration (scripts/throughput_gate.py).

Exit 0 PASS, 1 FAIL (the planner panicked, errored or emitted a non-finite
segment; klippy would raise or drop motion), 2 UNVERIFIED (preflight cannot
plan the file). docs/rewrite/preflight.md: how each command is treated.
"""

from __future__ import annotations

import argparse
import dataclasses
import enum
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "snapshots")]

import engine_build_id  # noqa: E402
import slots  # noqa: E402
import throughput_gate as gate  # noqa: E402
from preflight_gcode import (  # noqa: E402
    DEFAULT_MESH_PROFILE,
    FrontEnd,
    HostError,
    MeshChange,
    Replay,
    Treatment,
)

from klippy import configfile  # noqa: E402

RUST = ROOT / "rust"
BENCH_EXAMPLE = "gcode_replay_bench"
BENCH = RUST / "target" / "release" / "examples" / BENCH_EXAMPLE
FAILURE_PREFIX = "REPLAY_FAILURE "
CACHE = Path.home() / ".cache" / "serval-preflight"
PI_SPEED_CACHE = CACHE / "pi_speed.json"
LINE_PREFIX = re.compile(r"^line (\d+): ")
SUMMARY = re.compile(
    r"stream_secs=(?P<stream>[0-9.]+) wall_secs=(?P<wall>[0-9.]+) "
    r"overall_x=(?P<overall>[0-9.]+)"
)
WINDOW = re.compile(
    r"^\s+(?P<lo>[0-9.]+)\.\.\s*(?P<hi>[0-9.]+)\s+(?P<x>[0-9.]+)x$"
)
WINDOWS_HEADER = "worst stream windows"
PLANNER_FAILURE_KINDS = {
    "config": "klippy would not start: the planner refuses the config",
    "mesh": "klippy would raise loading the mesh",
    "gcode_stops_print": "klippy would raise on this line and stop the print",
    "move_refused": "the planner refuses a move",
    "panic": "the planner panicked",
    "non_finite": "the planner emitted a segment that does not evaluate finite",
    "pipeline_closed": "the planner closed its input",
}
UNVERIFIED_KINDS = {
    "gcode_unplannable": "preflight cannot plan this G-code",
    "nothing_to_plan": "the print holds no motion preflight can plan",
}
NOT_COVERED = (
    "heaters, fans and thermal behaviour (heater waits plan as stops at rest)",
    "homing and probing moves; the print starts where its first move names",
    "MCU dispatch: step generation and compression, step-rate ceilings, "
    "queues and the host-MCU link",
    "host runtime: the pacer, drains when the host falls behind, "
    "pause/resume/cancel, clock sync, timing on the printer's own host",
)


class Verdict(enum.IntEnum):
    PASS = 0
    FAIL = 1
    UNVERIFIED = 2


class PreflightError(RuntimeError):
    pass


@dataclass(frozen=True)
class BenchRun:
    returncode: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class ReplayFailure:
    kind: str
    message: str
    location: str | None
    fed_line: int | None
    emitted_line: int | None
    stream_t: float | None


@dataclass(frozen=True)
class Throughput:
    stream_s: float
    wall_s: float
    overall_x: float
    worst_x: float | None
    worst_window: tuple[float, float] | None


@dataclass(frozen=True)
class MeshChoice:
    change: MeshChange | None
    why: str


@dataclass(frozen=True)
class PiEstimate:
    low_x: float
    high_x: float
    note: str


@dataclass
class Outcome:
    verdict: Verdict = Verdict.PASS
    problems: list[str] = dataclasses.field(default_factory=list)

    def worsen(self, verdict: Verdict, problem: str) -> None:
        self.verdict = max(self.verdict, verdict)
        self.problems.append(problem)


def bench_env() -> dict[str, str]:
    """A CARGO_TARGET_DIR leaked from the caller's shell would build into
    someone else's target (CLAUDE.md)."""
    env = dict(os.environ)
    env.pop("CARGO_TARGET_DIR", None)
    return env


class Bench:
    """`gcode_replay_bench` built from this tree's rust/: built and checked
    in a machine-wide slot, timed in all of them (scripts/slots.py)."""

    def build(self) -> None:
        with slots.slot(label="preflight build"):
            built = subprocess.run(
                [
                    "cargo",
                    "build",
                    "--release",
                    "-p",
                    "motion-core",
                    "--example",
                    BENCH_EXAMPLE,
                ],
                cwd=RUST,
                env=bench_env(),
                capture_output=True,
                text=True,
                check=False,
            )
        if built.returncode != 0:
            raise PreflightError(
                f"building {BENCH_EXAMPLE} failed:\n{built.stderr[-3000:]}"
            )

    def check(self, arguments: list[str]) -> BenchRun:
        with slots.slot(label="preflight check"):
            return bench_run([*arguments, "--synchronous", "--check"])

    def time(self, arguments: list[str]) -> BenchRun:
        with slots.slot(exclusive=True, label="preflight timing"):
            return bench_run(arguments, ["taskset", "-c", gate.LIVE_CORES])


def bench_run(
    arguments: list[str], pinning: list[str] | None = None
) -> BenchRun:
    done = subprocess.run(
        [*(pinning or []), str(BENCH), *arguments],
        capture_output=True,
        text=True,
        check=False,
    )
    return BenchRun(done.returncode, done.stdout, done.stderr)


def replay_failure(run: BenchRun) -> ReplayFailure | None:
    for line in run.stdout.splitlines():
        if line.startswith(FAILURE_PREFIX):
            record = json.loads(line[len(FAILURE_PREFIX) :])
            return ReplayFailure(**record)
    if run.returncode == 0:
        return None
    return ReplayFailure(
        "crash", run.stderr[-2000:].strip(), None, None, None, None
    )


def throughput(run: BenchRun) -> Throughput:
    summary = SUMMARY.search(run.stdout)
    if summary is None:
        raise PreflightError(
            f"no replay summary in the bench output:\n{run.stdout[-2000:]}"
        )
    worst = first_window(run.stdout.splitlines())
    return Throughput(
        stream_s=float(summary["stream"]),
        wall_s=float(summary["wall"]),
        overall_x=float(summary["overall"]),
        worst_x=None if worst is None else worst[2],
        worst_window=None if worst is None else worst[:2],
    )


def first_window(lines: list[str]) -> tuple[float, float, float] | None:
    header = next(
        (i for i, line in enumerate(lines) if line.startswith(WINDOWS_HEADER)),
        None,
    )
    if header is None or header + 1 >= len(lines):
        return None
    window = WINDOW.match(lines[header + 1])
    if window is None:
        return None
    return float(window["lo"]), float(window["hi"]), float(window["x"])


def at_origin(message: str, replay: Replay) -> str:
    """`line N: ...` of the replay file, said of the print line it came from."""
    match = LINE_PREFIX.match(message)
    origin = replay.origin_of(int(match[1])) if match else None
    if origin is None:
        return message
    return f"{origin.describe()}: {message[match.end() :]}"


def describe_failure(failure: ReplayFailure, replay: Replay) -> str:
    lines = [f"{failure.kind}: {at_origin(failure.message, replay)}"]
    if failure.location:
        lines.append(f"  at {failure.location}")
    fed = replay.origin_of(failure.fed_line)
    if fed is not None:
        lines.append(f"  last move fed to the planner: {fed.describe()}")
    emitted = replay.origin_of(failure.emitted_line)
    if emitted is not None:
        lines.append(
            f"  last segment emitted: {emitted.describe()}, ending at stream "
            f"{failure.stream_t:.3f} s"
        )
    return "\n".join(lines)


def judge_failure(
    failure: ReplayFailure, replay: Replay, outcome: Outcome
) -> None:
    if failure.kind in UNVERIFIED_KINDS:
        outcome.worsen(
            Verdict.UNVERIFIED,
            f"{UNVERIFIED_KINDS[failure.kind]}\n{describe_failure(failure, replay)}",
        )
        return
    reason = PLANNER_FAILURE_KINDS.get(failure.kind, "the replay crashed")
    outcome.worsen(
        Verdict.FAIL, f"{reason}\n{describe_failure(failure, replay)}"
    )


def choose_mesh(front: FrontEnd, requested: str | None) -> MeshChoice:
    """The mesh the replay plans over: `requested`, else the first the print
    loads (or `bed_mesh_default` loads at startup), else 'default'."""
    if requested is not None:
        if requested not in front.mesh_profiles():
            raise PreflightError(
                f"--mesh {requested}: the config saves {front.mesh_profiles()}"
            )
        return MeshChoice(front.load_mesh_profile(requested), "--mesh")
    loads = [
        change for change in front.mesh_changes if change.profile is not None
    ]
    if loads:
        return MeshChoice(loads[0], f"loaded {loads[0].origin.describe()}")
    if DEFAULT_MESH_PROFILE in front.mesh_profiles():
        return MeshChoice(
            front.load_mesh_profile(DEFAULT_MESH_PROFILE),
            "the print loads no mesh: the saved 'default' profile",
        )
    return MeshChoice(None, "no saved bed mesh profile: planned without a mesh")


def unreplayed_mesh_changes(
    front: FrontEnd, choice: MeshChoice
) -> list[MeshChange]:
    return [
        change
        for change in front.mesh_changes
        if change is not choice.change and change.origin.line > 0
    ]


def lost_motion(replay: Replay, outcome: Outcome) -> None:
    for name, tally in replay.by_treatment(Treatment.LOST_ON_PRINTER).items():
        outcome.worsen(
            Verdict.FAIL,
            f"{name} x{tally.count} (first {tally.first.describe()}): "
            f"{Treatment.LOST_ON_PRINTER.value}",
        )


def bench_arguments(
    scratch: Path, front: FrontEnd, replay: Replay, mesh: MeshChoice
) -> list[str]:
    gcode = scratch / "replay.gcode"
    config = scratch / "printer.cfg"
    gcode.write_text(replay.text())
    config.write_text(front.config_text())
    arguments = [str(gcode), "--config", str(config)]
    if mesh.change is not None:
        request = scratch / "bed_mesh.json"
        request.write_text(json.dumps(mesh.change.request))
        arguments += ["--mesh", str(request)]
    return arguments


def replay_print(
    bench: Bench, arguments: list[str], replay: Replay, outcome: Outcome
) -> Throughput | None:
    checked = replay_failure(bench.check(arguments))
    if checked is not None:
        judge_failure(checked, replay, outcome)
        return None
    live = bench.time(arguments)
    staged = replay_failure(live)
    if staged is not None:
        judge_failure(staged, replay, outcome)
        return None
    return throughput(live)


def load_average() -> str:
    return " ".join(Path("/proc/loadavg").read_text().split()[:3])


def calibration_costs(commit: str) -> dict[str, float]:
    """This box's live 2-core cost (wall per stream second) of the gate's
    calibration replay, per gated config, measured once per CPU model."""
    cpu = gate.cpu_model()
    cached = (
        json.loads(PI_SPEED_CACHE.read_text())
        if PI_SPEED_CACHE.exists()
        else {}
    )
    if commit in cached.get(cpu, {}):
        return cached[cpu][commit]
    gate.gate_cores()
    target = RUST / "target"
    with slots.slot(label="preflight calibration build"):
        gate.extract_commit(commit, target)
        tree = gate.commit_tree(target, commit)
        bench = gate.build_in_lab(tree / "rust", "calibration")
    with slots.slot(exclusive=True, label="preflight calibration replay"):
        costs = {
            config: gate.fastest_mean(
                [
                    gate.live_secs_per_stream_sec(bench, tree, config)[0]
                    for _ in range(gate.LIVE_FASTEST)
                ]
            )
            for config in gate.CONFIGS
        }
    cached.setdefault(cpu, {})[commit] = costs
    PI_SPEED_CACHE.parent.mkdir(parents=True, exist_ok=True)
    PI_SPEED_CACHE.write_text(json.dumps(cached, indent=2) + "\n")
    return costs


def pi_estimate(box_worst_x: float, config: Path) -> PiEstimate:
    """The Pi 5's worst window at this box's speed over the Pi's on the
    gate's calibration replay of `config` when the gate replays it, else
    on each gated config."""
    calibration = gate.load_baseline()["pi_calibration"]
    costs = calibration_costs(calibration["commit"])
    speeds = {
        gated: calibration["configs"][gated]["live"]["pi_cost"] / cost
        for gated, cost in costs.items()
    }
    gated = repo_relative(config)
    if gated in speeds:
        speeds = {gated: speeds[gated]}
    estimates = [box_worst_x / speed for speed in speeds.values()]
    speed_range = span(min(speeds.values()), max(speeds.values()))
    note = (
        f"estimate: this box's 2-core worst window over its speed against the "
        f"Pi 5 ({speed_range}x) on the throughput gate's calibration replay of "
        f"{', '.join(Path(name).stem for name in speeds)} (commit "
        f"{calibration['commit'][:10]}, Pi measured "
        f"{calibration['measured_at'][:10]}); the gate's own live model misses "
        f"the Pi by up to ~{max_live_residual(calibration):.0f} %"
    )
    return PiEstimate(min(estimates), max(estimates), note)


def repo_relative(path: Path) -> str | None:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return None


def max_live_residual(calibration: dict) -> float:
    return max(
        model["live"]["max_cpu_out_residual_pct"]
        for model in calibration["configs"].values()
    )


def measured_on_pi(gcode: Path, config: Path) -> float | None:
    """The Pi's own worst window, when it measured exactly this tree's
    replay of this print and config (tools/bench/pi_window.json)."""
    if gcode.resolve() != (ROOT / gate.GCODE).resolve():
        return None
    window = gate.pi_window(gate.measured_trees())
    return window.worst_x.get(repo_relative(config))


def tally_lines(replay: Replay, kind: Treatment) -> list[str]:
    return [
        f"{name} x{tally.count} (first {tally.first.describe()})"
        for name, tally in sorted(replay.by_treatment(kind).items())
    ]


def coverage_lines(
    front: FrontEnd, replay: Replay, mesh: MeshChoice
) -> list[str]:
    lines = [
        f"skipped macro {s.origin.describe()}: {s.reason}"
        for s in front.skipped
    ]
    for kind in Treatment:
        if kind in (Treatment.PLANNED, Treatment.LOST_ON_PRINTER):
            continue
        tallies = tally_lines(replay, kind)
        if tallies:
            lines.append(f"{kind.value}: {', '.join(tallies)}")
    for change in unreplayed_mesh_changes(front, mesh):
        action = f"load {change.profile}" if change.profile else "clear"
        lines.append(
            f"mesh {action} {change.origin.describe()}: not replayed, the "
            "replay keeps the mesh above for the whole print"
        )
    return [*lines, *NOT_COVERED]


def mesh_line(mesh: MeshChoice) -> str:
    if mesh.change is None:
        return mesh.why
    request = mesh.change.request
    grid = f"{request['nx']}x{request['ny']}"
    return f"{mesh.change.profile} ({grid}; {mesh.why})"


def report(
    args: argparse.Namespace,
    front: FrontEnd,
    replay: Replay,
    mesh: MeshChoice,
    outcome: Outcome,
    timing: Throughput | None,
    pi: PiEstimate | None,
) -> None:
    print(f"preflight {args.gcode} on {args.config}")
    print(f"  rust build id  {engine_build_id.tree_build_id(ROOT)} ({ROOT})")
    print(f"  bed mesh       {mesh_line(mesh)}")
    print(f"VERDICT {outcome.verdict.name}")
    for problem in outcome.problems:
        print("  " + problem.replace("\n", "\n  "))
    if timing is not None:
        print_timing(timing, pi, measured_on_pi(args.gcode, args.config))
    print("not covered:")
    for line in coverage_lines(front, replay, mesh):
        print(f"  - {line}")


def print_timing(
    timing: Throughput, pi: PiEstimate | None, pi_measured: float | None
) -> None:
    print(
        f"  stream {timing.stream_s:.1f} s planned in {timing.wall_s:.1f} s wall "
        f"({timing.overall_x:.2f}x realtime) on this box, 2 cores "
        f"(taskset -c {gate.LIVE_CORES}), load {load_average()}"
    )
    if timing.worst_x is None:
        print("  worst 5 s window: none (the print is shorter than one window)")
        return
    lo, hi = timing.worst_window
    print(
        f"  worst 5 s window {timing.worst_x:.2f}x at stream {lo:.1f}-{hi:.1f} s"
    )
    if pi_measured is not None:
        verdict = "ok" if pi_measured >= gate.PI_FLOOR_X else "BELOW"
        print(
            f"  Pi 5 2-core worst window {pi_measured:.2f}x (measured on the Pi "
            f"for this tree), floor {gate.PI_FLOOR_X}x: {verdict}"
        )
    if pi is None:
        return
    verdict = (
        "ok" if pi.low_x >= gate.PI_FLOOR_X else "WARNING: below the floor"
    )
    print(
        f"  Pi 5 2-core worst window ~{span(pi.low_x, pi.high_x)}x, floor "
        f"{gate.PI_FLOOR_X}x: {verdict}\n    ({pi.note})"
    )


def span(low: float, high: float) -> str:
    if f"{low:.2f}" == f"{high:.2f}":
        return f"{low:.2f}"
    return f"{low:.2f}-{high:.2f}"


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("gcode", type=Path, help="the file about to be printed")
    parser.add_argument(
        "--config", type=Path, required=True, help="the printer's printer.cfg"
    )
    parser.add_argument(
        "--mesh",
        help="saved [bed_mesh] profile (default: the one the print loads, else 'default')",
    )
    parser.add_argument(
        "--no-pi", action="store_true", help="skip the Pi 5 estimate"
    )
    return parser.parse_args(argv)


def expand_print(front: FrontEnd, gcode: Path, outcome: Outcome) -> None:
    try:
        front.expand(gcode.read_text())
    except HostError as error:
        outcome.worsen(
            Verdict.FAIL,
            f"klippy would raise and stop the print: {error}\n"
            "  (the replay plans the print up to that line)",
        )


def mesh_for(
    front: FrontEnd, requested: str | None, outcome: Outcome
) -> MeshChoice:
    try:
        return choose_mesh(front, requested)
    except HostError as error:
        outcome.worsen(
            Verdict.FAIL, f"klippy would raise loading the mesh: {error}"
        )
        return MeshChoice(
            None, "the mesh does not load: planned without a mesh"
        )


def main(
    argv: list[str] | None = None, bench_factory: Callable[[], Bench] = Bench
) -> int:
    args = parse_args(argv)
    front = FrontEnd(args.config)
    outcome = Outcome()
    expand_print(front, args.gcode, outcome)
    mesh = mesh_for(front, args.mesh, outcome)
    replay = Replay.of(front.commands)
    lost_motion(replay, outcome)
    bench = bench_factory()
    bench.build()
    with tempfile.TemporaryDirectory(prefix="preflight-") as scratch:
        arguments = bench_arguments(Path(scratch), front, replay, mesh)
        timing = replay_print(bench, arguments, replay, outcome)
    pi = None
    if timing is not None and timing.worst_x is not None and not args.no_pi:
        pi = pi_estimate(timing.worst_x, args.config)
    report(args, front, replay, mesh, outcome, timing, pi)
    return int(outcome.verdict)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (PreflightError, configfile.error) as error:
        print(f"VERDICT UNVERIFIED\n  {error}", file=sys.stderr)
        sys.exit(Verdict.UNVERIFIED)
