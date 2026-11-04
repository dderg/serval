#!/usr/bin/env python3
"""Replay-throughput gate: every user-space instruction, plus a live 2-core A/B.

    python3 scripts/throughput_gate.py [--accept --reason R]
        [--calibrate-pi-worst CFG=X[,X...] ...] [--accept-base REV]
    ./scripts/ci.sh throughput-gate [same arguments]

tools/bench/throughput_baseline.json records an accepted commit (the
reference) and, per config, what the gate measured there. Each run measures
the checkout two ways.

Instructions (deterministic). The first SLICE_LINES lines of the dense print
replay through `gcode_replay_bench --synchronous --digest` under valgrind
cachegrind (`--cache-sim=no`), pinned to one core: one thread (no shaper
thread, and a one-core affinity leaves the worker pool without helpers).
Every user-space instruction of the process counts (`Ir`): the repo's code,
the std and registry code inlined into it, libc's memcpy and malloc, libm.
Counting only instructions whose debug-info file lay under rust/ dropped
about half the work, and a different half whenever inlining moved. valgrind
shows the guest one fixed CPUID per instruction-set level, so glibc picks
the same memcpy and libm variants on every AVX2 host.

Live 2-core A/B (measured). Instructions cannot see what costs the Pi most.
PR #663 handed the shaper thread 3.3x more items on trident_kernel: its
instructions grew 1.8 % over the full print, its Pi 2-core worst window fell
17 %, paid in time per instruction and in thread handoffs that only the live
pipeline (planning, shaper and pool threads sharing two cores) has. So the
whole dense print also replays live (no --synchronous) under `taskset -c
0,1`, built from this checkout and from the reference commit (extracted with
`git archive`, built in its own target dir), LIVE_REPS times each, the two
alternating. A config's live ratio is this checkout's median wall seconds
per stream second over the reference's: the same machine runs both, so the
ratio does not depend on the machine's speed, which a fixed synthetic
workload could not normalise away (its ratio to the replay differed 5-11 %
between a Ryzen 7800X3D and a GitHub EPYC runner). CPU seconds are no
substitute for wall: on four kinds of runner #663 moved trident_kernel's by
-0.2 to +2.3 % where its wall moved +4.8 to +7.6 %. Replaying one binary as
both sides on three GitHub runners (2x EPYC 7763, Xeon 8370C) put the
median ratio within 0.45 % of 1; #663 moved it by +2.6 % (Xeon) to +5.4 %
(EPYC 9V74) on trident_kernel and by +4.4 to +7.8 % on
p3_voron_direct_drive_kernel, so LIVE_TOLERANCE_PCT sits between the two.

The gate fails when, for any config:
- Ir moved more than IR_TOLERANCE_PCT from its baseline;
- the live ratio is more than LIVE_TOLERANCE_PCT away from 1;
- the Pi 5 2-core worst 5 s window either measurement predicts is below
  PI_FLOOR_X, once PI_FLOOR_FAILS is set; until then it is reported as a
  warning, because main itself sits below the floor.
Growth is a regression. Shrinkage must be accepted, so the baseline ratchets
down and the gain stays locked in.

The Ir prediction is K x stream_secs / Ir: the replay's realtime factor
scaled by the instruction rate the Pi sustains in the dense print's worst
window. The live prediction is the reference's Pi worst window divided by
the live ratio. Both come from a calibration, the Pi 2-core worst window of
the full print (`taskset -c 0,1 gcode_replay_bench <dense.gcode> --config
CFG`, the lowest of two runs) measured at the checked-out commit:

    python3 scripts/throughput_gate.py --calibrate-pi-worst \\
        p3_voron_direct_drive_ramp=2.12,2.13 trident_kernel=2.29,2.30 ...

Accepting (`--accept --reason "..."`) makes HEAD the reference and rewrites
every config's Ir with this run's; an accept without a calibration carries
the reference's Pi worst window over by the live ratio it measured.
Accepting and calibrating both refuse a dirty tree, so the recorded commit
is the measured code. A PR whose tools/bench/throughput_baseline.json
differs from its base (--accept-base, which CI sets to the merge commit's
base parent) must carry the reason in a fenced block in its body, read from
$PR_BODY:

    ```throughput-accept
    the shaper's follower projection now fits twice per window (+4 % Ir)
    ```

scripts/ship.py refuses to land such a branch whose --body lacks the block.

Without valgrind on PATH the measurement runs in the scripts/Dockerfile-build
image named by $THROUGHPUT_GATE_IMAGE (ci.sh builds and sets it), with the
checkout and cargo home mounted at the GitHub runner's paths, so a local run
compiles exactly what CI compiles.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import io
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import tarfile
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUST = ROOT / "rust"
BASELINE = ROOT / "tools" / "bench" / "throughput_baseline.json"
GCODE = Path("tools") / "sim" / "gcode" / "voron_dense_top_layers.gcode"
CONFIGS = (
    "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg",
    "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg",
    "tools/bench/configs/trident_kernel.cfg",
)
SLICE_LINES = 1000
IR_TOLERANCE_PCT = 2.0
LIVE_REPS = 5
LIVE_TOLERANCE_PCT = 2.0
LIVE_CORES = "0,1"
PI_FLOOR_X = 2.0
# TODO(trident-panic): floor checks hard once main ≥2.0x
PI_FLOOR_FAILS = False
MEASURED_TREES = ("rust", "tools/bench/configs", "tools/sim/gcode")
REFERENCE_DIR = "throughput-reference"
EXTRACTED_MARKER = ".extracted"
CI_ROOT = Path("/home/runner/work/serval/serval")
CI_CARGO_HOME = "/home/runner/.cargo"
DOCKER_CACHE = RUST / "target" / "throughput-gate"
ACCEPT_BLOCK = re.compile(r"```throughput-accept[ \t]*\n(.*?)```", re.S)
STREAM_SECS = re.compile(r"\bstream_secs=([0-9.]+)")


class GateError(RuntimeError):
    pass


@dataclass(frozen=True)
class Measurement:
    ir: int
    stream_secs: float
    live_ratio: float
    live_wall_secs: float
    reference_wall_secs: float


@dataclass(frozen=True)
class Verdict:
    config: str
    measurement: Measurement
    ir_delta_pct: float
    predicted_x: tuple[float, float] | None
    problems: tuple[str, ...]
    warnings: tuple[str, ...]


def run(command: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(
        command, cwd=cwd, capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise GateError(
            f"failed ({result.returncode}): {' '.join(command)}\n"
            f"{result.stdout[-4000:]}{result.stderr[-4000:]}"
        )
    return result.stdout


def instruction_count(cachegrind_out: str) -> int:
    total = 0
    summary = None
    for line in cachegrind_out.splitlines():
        if line[:1].isdigit():
            fields = line.split()
            if len(fields) != 2:
                raise GateError(
                    f"cachegrind line is not '<line> <Ir>': {line!r}"
                )
            total += int(fields[1])
        elif line.startswith("summary:"):
            summary = int(line.split()[1])
    if summary is None:
        raise GateError("cachegrind output has no summary line")
    if summary != total:
        raise GateError(
            f"cachegrind records sum to {total}, its summary says {summary}"
        )
    return total


def stream_secs(bench_stdout: str) -> float:
    match = STREAM_SECS.search(bench_stdout)
    if not match:
        raise GateError(f"no stream_secs in the bench output:\n{bench_stdout}")
    return float(match.group(1))


def live_ratio(
    head_secs_per_stream_sec: list[float],
    reference_secs_per_stream_sec: list[float],
) -> float:
    if not head_secs_per_stream_sec or not reference_secs_per_stream_sec:
        raise GateError("the live A/B needs at least one run of each side")
    return statistics.median(head_secs_per_stream_sec) / statistics.median(
        reference_secs_per_stream_sec
    )


def accept_reason(body: str) -> str:
    match = ACCEPT_BLOCK.search(body or "")
    return match.group(1).strip() if match else ""


def accept_block_problem(base: str, body: str) -> str | None:
    relative = str(BASELINE.relative_to(ROOT))
    changed = subprocess.run(
        ["git", "diff", "--quiet", base, "--", relative], cwd=ROOT, check=False
    ).returncode
    if changed not in (0, 1):
        raise GateError(f"git diff against {base} failed")
    if changed == 0 or accept_reason(body):
        return None
    return (
        f"{relative} differs from {base}, so the PR body needs a "
        "```throughput-accept block saying why the baselines moved"
    )


def toolchain_channel() -> str:
    text = (RUST / "rust-toolchain.toml").read_text()
    match = re.search(r'^channel\s*=\s*"([^"]+)"', text, re.M)
    if not match:
        raise GateError("rust/rust-toolchain.toml has no channel")
    return match.group(1)


def reference_tree(target_dir: Path, commit: str) -> Path:
    return target_dir / REFERENCE_DIR / commit


def prepare_reference(commit: str, target_dir: Path) -> None:
    tree = reference_tree(target_dir, commit)
    if (tree / EXTRACTED_MARKER).exists():
        return
    present = subprocess.run(
        ["git", "cat-file", "-e", f"{commit}^{{commit}}"], cwd=ROOT, check=False
    ).returncode
    if present != 0:
        run(
            ["git", "fetch", "--quiet", "--depth=1", "origin", commit], cwd=ROOT
        )
    archive = subprocess.run(
        ["git", "archive", "--format=tar", commit, "--", *MEASURED_TREES],
        cwd=ROOT,
        capture_output=True,
        check=False,
    )
    if archive.returncode != 0:
        raise GateError(
            f"git archive of the reference {commit} failed:\n"
            f"{archive.stderr.decode(errors='replace')}"
        )
    if tree.exists():
        shutil.rmtree(tree)
    tree.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(archive.stdout)) as tar:
        tar.extractall(tree, filter="data")
    (tree / EXTRACTED_MARKER).write_text(commit + "\n")


def build_bench(rust_dir: Path) -> Path:
    run(
        [
            "cargo",
            "build",
            "--release",
            "--locked",
            "-p",
            "motion-core",
            "--example",
            "gcode_replay_bench",
        ],
        cwd=rust_dir,
    )
    return rust_dir / "target" / "release" / "examples" / "gcode_replay_bench"


def gate_cores() -> list[int]:
    cores = sorted(os.sched_getaffinity(0))
    if cores[:2] != [0, 1]:
        raise GateError(
            f"the live replay pins cores {LIVE_CORES}, this process may run on {cores}"
        )
    for core in cores:
        if run(["taskset", "-c", str(core), "nproc"]).strip() != "1":
            raise GateError(
                f"taskset -c {core} leaves more than one core visible"
            )
    if run(["taskset", "-c", LIVE_CORES, "nproc"]).strip() != "2":
        raise GateError(
            f"taskset -c {LIVE_CORES} does not leave two cores visible"
        )
    return cores


def count_instructions(
    bench: Path, sliced: Path, config: str, core: int, scratch: Path
) -> tuple[int, float]:
    out_file = scratch / f"cachegrind-{Path(config).stem}.out"
    stdout = run(
        [
            "taskset",
            "-c",
            str(core),
            "valgrind",
            "--tool=cachegrind",
            "--cache-sim=no",
            f"--cachegrind-out-file={out_file}",
            str(bench),
            str(sliced),
            "--config",
            str(ROOT / config),
            "--synchronous",
            "--digest",
        ]
    )
    return instruction_count(out_file.read_text()), stream_secs(stdout)


def measure_instructions(
    bench: Path, configs: tuple[str, ...], cores: list[int]
) -> dict[str, tuple[int, float]]:
    lines = (ROOT / GCODE).read_text().splitlines(keepends=True)
    if len(lines) < SLICE_LINES:
        raise GateError(
            f"{GCODE} has {len(lines)} lines, the slice wants {SLICE_LINES}"
        )
    started = time.monotonic()
    with tempfile.TemporaryDirectory() as scratch_dir:
        scratch = Path(scratch_dir)
        sliced = scratch / "slice.gcode"
        sliced.write_text("".join(lines[:SLICE_LINES]))
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=len(cores)
        ) as pool:
            futures = {
                config: pool.submit(
                    count_instructions,
                    bench,
                    sliced,
                    config,
                    cores[index % len(cores)],
                    scratch,
                )
                for index, config in enumerate(configs)
            }
            counted = {
                config: future.result() for config, future in futures.items()
            }
    print(
        f"counted instructions in {time.monotonic() - started:.0f} s",
        file=sys.stderr,
        flush=True,
    )
    return counted


def live_secs_per_stream_sec(
    bench: Path, tree: Path, config: str
) -> tuple[float, float]:
    for needed in (tree / GCODE, tree / config):
        if not needed.exists():
            raise GateError(
                f"{needed} is missing, so the live A/B cannot replay {config} "
                "there: accept a new baseline"
            )
    started = time.monotonic()
    stdout = run(
        [
            "taskset",
            "-c",
            LIVE_CORES,
            str(bench),
            str(tree / GCODE),
            "--config",
            str(tree / config),
        ]
    )
    wall = time.monotonic() - started
    return wall / stream_secs(stdout), wall


def measure_live(
    head_bench: Path,
    reference_bench: Path,
    reference: Path,
    configs: tuple[str, ...],
) -> dict[str, tuple[float, float, float]]:
    sides = {
        "head": (head_bench, ROOT),
        "reference": (reference_bench, reference),
    }
    costs = {(config, side): [] for config in configs for side in sides}
    walls = {(config, side): [] for config in configs for side in sides}
    for rep in range(LIVE_REPS):
        order = ("head", "reference") if rep % 2 == 0 else ("reference", "head")
        for config in configs:
            for side in order:
                cost, wall = live_secs_per_stream_sec(*sides[side], config)
                costs[config, side].append(cost)
                walls[config, side].append(wall)
            print(
                f"live rep {rep + 1}/{LIVE_REPS} {config}: head "
                f"{walls[config, 'head'][-1]:.2f} s, reference "
                f"{walls[config, 'reference'][-1]:.2f} s",
                file=sys.stderr,
                flush=True,
            )
    return {
        config: (
            live_ratio(costs[config, "head"], costs[config, "reference"]),
            statistics.median(walls[config, "head"]),
            statistics.median(walls[config, "reference"]),
        )
        for config in configs
    }


def measure_here(
    configs: tuple[str, ...], reference_commit: str
) -> dict[str, Measurement]:
    reference = reference_tree(RUST / "target", reference_commit)
    if not (reference / EXTRACTED_MARKER).exists():
        raise GateError(
            f"the reference {reference_commit} is not extracted at {reference}"
        )
    cores = gate_cores()
    head_bench = build_bench(RUST)
    reference_bench = build_bench(reference / "rust")
    counted = measure_instructions(head_bench, configs, cores)
    live = measure_live(head_bench, reference_bench, reference, configs)
    return {
        config: Measurement(*counted[config], *live[config])
        for config in configs
    }


def measure_in_docker(
    image: str, configs: tuple[str, ...], reference_commit: str
) -> dict[str, Measurement]:
    target = DOCKER_CACHE / "target"
    cargo_home = DOCKER_CACHE / "cargo-home"
    target.mkdir(parents=True, exist_ok=True)
    cargo_home.mkdir(parents=True, exist_ok=True)
    prepare_reference(reference_commit, target)
    result_name = "throughput-measure.json"
    command = [
        "docker",
        "run",
        "--rm",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--tmpfs",
        "/tmp:exec,mode=1777,size=4g",
        "-e",
        "HOME=/tmp/gate-home",
        "-e",
        f"CARGO_HOME={CI_CARGO_HOME}",
        "-e",
        f"RUSTUP_TOOLCHAIN={toolchain_channel()}",
        "-e",
        "PYTHONDONTWRITEBYTECODE=1",
        "-v",
        f"{ROOT}:{CI_ROOT}",
        "-v",
        f"{target}:{CI_ROOT / 'rust' / 'target'}",
        "-v",
        f"{cargo_home}:{CI_CARGO_HOME}",
        "-w",
        str(CI_ROOT),
        "--entrypoint",
        "python3",
        image,
        "scripts/throughput_gate.py",
        "--measure-json",
        f"rust/target/{result_name}",
        "--reference",
        reference_commit,
        "--configs",
        *configs,
    ]
    print(f"measuring in {image}", file=sys.stderr, flush=True)
    if subprocess.run(command, check=False).returncode != 0:
        raise GateError(f"the measurement in {image} failed")
    raw = json.loads((target / result_name).read_text())
    return {config: Measurement(**raw[config]) for config in configs}


def measure(
    configs: tuple[str, ...], reference_commit: str
) -> dict[str, Measurement]:
    if shutil.which("valgrind"):
        prepare_reference(reference_commit, RUST / "target")
        return measure_here(configs, reference_commit)
    image = os.environ.get("THROUGHPUT_GATE_IMAGE")
    if not image:
        raise GateError(
            "valgrind is not installed and THROUGHPUT_GATE_IMAGE is unset: "
            "run ./scripts/ci.sh throughput-gate"
        )
    return measure_in_docker(image, configs, reference_commit)


def load_baseline() -> dict:
    return json.loads(BASELINE.read_text()) if BASELINE.exists() else {}


def write_baseline(baseline: dict) -> None:
    BASELINE.write_text(json.dumps(baseline, indent=2) + "\n")


def pi_calibration(
    measurement: Measurement, worst: list[float], commit: str, accepted: bool
) -> dict:
    lowest = min(worst)
    return {
        "k_ir": lowest * measurement.ir / measurement.stream_secs,
        "worst_x_reference": lowest
        if accepted
        else lowest * measurement.live_ratio,
        "worst_x": worst,
        "commit": commit,
    }


def carried_pi(pi: dict, measurement: Measurement) -> dict:
    return {
        **pi,
        "worst_x_reference": pi["worst_x_reference"] / measurement.live_ratio,
    }


def predicted_worst_x(
    entry: dict, measurement: Measurement
) -> tuple[float, float] | None:
    pi = entry.get("pi_2core")
    if pi is None:
        return None
    return (
        pi["k_ir"] * measurement.stream_secs / measurement.ir,
        pi["worst_x_reference"] / measurement.live_ratio,
    )


def drift_problems(
    name: str, delta_pct: float, tolerance_pct: float
) -> list[str]:
    if delta_pct > tolerance_pct:
        return [
            f"{name} grew {delta_pct:+.2f} % (limit {tolerance_pct} %): a "
            "throughput regression; if it is the price of a wanted change, "
            "accept it with a reason"
        ]
    if delta_pct < -tolerance_pct:
        return [
            f"{name} shrank {delta_pct:+.2f} % (beyond {tolerance_pct} %): "
            "accept the improvement so the baseline ratchets down"
        ]
    return []


def judge(config: str, entry: dict, measurement: Measurement) -> Verdict:
    ir_delta_pct = 100.0 * (measurement.ir - entry["ir"]) / entry["ir"]
    predicted = predicted_worst_x(entry, measurement)
    problems = [
        *drift_problems("Ir", ir_delta_pct, IR_TOLERANCE_PCT),
        *drift_problems(
            "live 2-core wall per stream second",
            100.0 * (measurement.live_ratio - 1.0),
            LIVE_TOLERANCE_PCT,
        ),
    ]
    floor_misses = []
    if predicted is None:
        problems.append("no Pi calibration: run --calibrate-pi-worst")
    else:
        for source, value in zip(("Ir", "the live ratio"), predicted):
            if value < PI_FLOOR_X:
                floor_misses.append(
                    f"the Pi 2-core worst window {source} predicts, "
                    f"{value:.2f}x, is below the {PI_FLOOR_X:.1f}x floor"
                )
    if PI_FLOOR_FAILS:
        problems.extend(floor_misses)
        floor_misses = []
    return Verdict(
        config,
        measurement,
        ir_delta_pct,
        predicted,
        tuple(problems),
        tuple(floor_misses),
    )


def report(verdicts: list[Verdict], reference_commit: str) -> None:
    print(f"reference {reference_commit}")
    print(
        f"{'config':<62} {'Ir':>15} {'delta':>8} {'live':>8} "
        f"{'Pi pred Ir':>10} {'Pi pred live':>12}"
    )
    for verdict in verdicts:
        m = verdict.measurement
        predicted = (
            ("uncalibrated", "uncalibrated")
            if verdict.predicted_x is None
            else tuple(f"{value:.2f}x" for value in verdict.predicted_x)
        )
        print(
            f"{verdict.config:<62} {m.ir:>15} {verdict.ir_delta_pct:>+7.2f}% "
            f"{100.0 * (m.live_ratio - 1.0):>+7.2f}% "
            f"{predicted[0]:>10} {predicted[1]:>12}"
        )
        print(
            f"  live full print, median of {LIVE_REPS}: {m.live_wall_secs:.2f} s, "
            f"reference {m.reference_wall_secs:.2f} s"
        )
        for problem in verdict.problems:
            print(f"  FAIL: {problem}")
        for warning in verdict.warnings:
            print(f"  WARN (PI_FLOOR_FAILS is off): {warning}")


def parse_calibrations(
    items: list[str], configs: tuple[str, ...]
) -> dict[str, list[float]]:
    calibrations = {}
    for item in items:
        name, sep, values = item.partition("=")
        if not sep:
            raise GateError(
                f"--calibrate-pi-worst wants CFG=X[,X...], got {item!r}"
            )
        matches = [c for c in configs if name in (c, Path(c).stem)]
        if len(matches) != 1:
            raise GateError(
                f"{name!r} names none of the gated configs {list(configs)}"
            )
        worst = [float(value) for value in values.split(",") if value]
        if not worst:
            raise GateError(f"{item!r} has no worst-window measurement")
        calibrations[matches[0]] = worst
    return calibrations


def head_commit() -> str:
    dirty = run(
        ["git", "status", "--porcelain", "--", *MEASURED_TREES], cwd=ROOT
    )
    if dirty.strip():
        raise GateError(
            "accepting or calibrating records HEAD as the measured code, but "
            f"the measured trees have uncommitted changes:\n{dirty}"
        )
    return run(["git", "rev-parse", "HEAD"], cwd=ROOT).strip()


def baseline_problem(baseline: dict, configs: tuple[str, ...]) -> str | None:
    if baseline.get("slice_lines") != SLICE_LINES:
        return (
            f"the baselines were taken over {baseline.get('slice_lines')} lines, "
            f"the gate replays {SLICE_LINES}: accept new baselines"
        )
    if not baseline.get("commit"):
        return "the baseline names no reference commit: accept new baselines"
    missing = [c for c in configs if c not in baseline.get("configs", {})]
    if missing:
        return f"no baseline for {missing}: accept one"
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--accept", action="store_true")
    parser.add_argument("--reason", default="")
    parser.add_argument(
        "--calibrate-pi-worst", nargs="+", default=[], metavar="CFG=X"
    )
    parser.add_argument("--accept-base")
    parser.add_argument("--measure-json", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--reference", help=argparse.SUPPRESS)
    parser.add_argument(
        "--configs", nargs="+", default=list(CONFIGS), help=argparse.SUPPRESS
    )
    args = parser.parse_args()
    configs = tuple(args.configs)

    if args.measure_json:
        measurements = measure_here(configs, args.reference)
        args.measure_json.write_text(
            json.dumps(
                {c: dataclasses.asdict(m) for c, m in measurements.items()},
                indent=2,
            )
        )
        return 0
    if args.accept and not args.reason.strip():
        raise GateError("--accept needs --reason saying why the baselines move")
    if args.accept_base:
        problem = accept_block_problem(
            args.accept_base, os.environ.get("PR_BODY", "")
        )
        if problem:
            raise GateError(problem)
    calibrations = parse_calibrations(args.calibrate_pi_worst, configs)
    commit = head_commit() if args.accept or calibrations else ""

    baseline = load_baseline()
    problem = baseline_problem(baseline, configs)
    if problem and not args.accept:
        raise GateError(problem)
    reference_commit = baseline.get("commit") or commit
    entries = baseline.get("configs", {})

    measurements = measure(configs, reference_commit)
    if args.accept:
        entries = {
            config: {
                "ir": measurements[config].ir,
                "stream_secs": measurements[config].stream_secs,
                **(
                    {
                        "pi_2core": carried_pi(
                            entries[config]["pi_2core"], measurements[config]
                        )
                    }
                    if "pi_2core" in entries.get(config, {})
                    else {}
                ),
            }
            for config in configs
        }
    for config, worst in calibrations.items():
        entries[config]["pi_2core"] = pi_calibration(
            measurements[config], worst, commit, args.accept
        )
    if args.accept:
        reference_commit = commit
        measurements = {
            config: dataclasses.replace(measurement, live_ratio=1.0)
            for config, measurement in measurements.items()
        }
    if args.accept or calibrations:
        write_baseline(
            {
                "slice_lines": SLICE_LINES,
                "commit": reference_commit,
                "reason": args.reason.strip()
                if args.accept
                else baseline["reason"],
                "configs": entries,
            }
        )
        print(f"wrote {BASELINE.relative_to(ROOT)}")

    verdicts = [judge(c, entries[c], measurements[c]) for c in configs]
    report(verdicts, reference_commit)
    failed = [v.config for v in verdicts if v.problems]
    if failed:
        print(
            f"throughput gate: FAIL ({len(failed)} of {len(verdicts)} configs)"
        )
        return 1
    print(
        f"throughput gate: PASS ({len(verdicts)} configs: Ir within "
        f"{IR_TOLERANCE_PCT} %, live within {LIVE_TOLERANCE_PCT} % of the reference)"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput gate: {exc}")
