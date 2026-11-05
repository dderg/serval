#!/usr/bin/env python3
"""Replay-throughput gate: every user-space instruction, plus a live 2-core A/B.

    python3 scripts/throughput_gate.py [--accept --reason R] [--accept-base REV]
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
`git archive`), LIVE_REPS times each, the two alternating. Both sides build
from one directory, one after the other, so their binaries differ only where
their code does: built from different paths, identical code gets different
symbol hashes and code layout. The instruction count replays the same
checkout build. A config's live ratio is this checkout's wall seconds per
stream second over the reference's, each the mean of its LIVE_FASTEST
fastest runs: a busy machine only ever slows a run down, and averaging two
blunts one lucky run. The same machine runs both sides, so the
ratio does not depend on the machine's speed, which a fixed synthetic
workload could not normalise away (its ratio to the replay differed 5-11 %
between a Ryzen 7800X3D and a GitHub EPYC runner). CPU seconds are no
substitute for wall: on four kinds of runner #663 moved trident_kernel's by
-0.2 to +2.3 % where its wall moved +4.8 to +7.6 %. Replaying one binary as
both sides on three GitHub runners (2x EPYC 7763, Xeon 8370C) put the
ratio within 0.45 % of 1; #663 moved it by +2.6 % (Xeon) to +5.4 %
(EPYC 9V74) on trident_kernel and by +4.4 to +7.8 % on
p3_voron_direct_drive_kernel, so LIVE_TOLERANCE_PCT sits between the two.

The gate fails when, for any config:
- Ir moved more than IR_TOLERANCE_PCT from its baseline;
- the live ratio is more than LIVE_TOLERANCE_PCT away from 1;
- the Pi 5 2-core worst 5 s window either measurement predicts is below
  PI_FLOOR_X (PI_FLOOR_FAILS off turns this into a warning). A config
  FLOOR_SHORTFALLS pins below the floor, to the issue that owns the
  shortfall, misses it instead when a prediction leaves the pinned one by
  more than SHORTFALL_TOLERANCE_PCT (grown: a regression; shrunk: re-pin)
  or reaches the floor (unpin);
- the calibration replay below is inconsistent.
Growth is a regression. Shrinkage must be accepted, so the baseline ratchets
down and the gain stays locked in.

Both Pi predictions come from fits over paired points that
scripts/throughput_recheck.py measures: commits and patched variants of
them, and the calibration commit pi_calibration.commit, each with its Pi
2-core worst window (the full print under `taskset -c 0,1`, the lowest of
two runs), its slice's Ir and its live cost on the runners. Neither
measurement moves the Pi one for one. For #663, the Ir ratio predicted a
worst window of 2.05x and the Pi measured 1.89x. So each config gets its own
exponents, fitted through the calibration commit, whose Pi window W_cal the
re-check measured:

    Ir:   W = W_cal x (Ir/stream_secs / the calibration's)^-beta
    live: W = W_cal x (live cost / the calibration's)^-alpha

The live cost needs a yardstick that holds still on every runner: the live
replay of the calibration commit, a third side of the live A/B, built from
that commit's tree (its code, config and print never change with the repo)
and replayed alongside the head. The head's live cost over the calibration's
is the head's speed in units of a workload whose Pi window is known. One
alpha per config serves every runner CPU: fitted over every CPU model the
re-check saw, each model counted once, it misses the points by about as much
as one alpha per CPU model did, and a CPU model left out of the fit is
predicted as well as the ones in it. The baseline's pi_calibration block
keeps the points, the fits, and each fit's error and leave-one-out error, in
% of the worst window, overall and per CPU model. The gate prints them next
to its predictions. #663's thread handoffs cost the Pi more per unit of x86
cost than the other changes do, so the models under-predict that kind of
regression: treat a prediction within ~10 % of PI_FLOOR_X as uncertain, and
measure the Pi.

The normalisation is only as good as the calibration replay, so the gate
fails when it is inconsistent:
- its fastest LIVE_FASTEST runs differ by more than CALIBRATION_SPREAD_PCT.
  Over the 113 runner jobs and 3 configs of the first fit they differed by
  at most 4.3 % (99th percentile 3.2 %). A 5 % spread moves the prediction
  by about 2.5 %, a quarter of the fit's own error;
- this runner's speed on it, as a multiple of the Pi's, lies more than
  CALIBRATION_SPEED_MARGIN_PCT outside the speeds of the jobs the fit
  measured. Runners of one CPU model ran it up to 30 % apart (EPYC 9V74), so
  a runner within that margin is one like those fitted; beyond it the fit
  extrapolates.
An unknown CPU model is not a failure.

The calibration goes stale. The head drifts away from the code the points
were measured on, and a toolchain bump or a Pi OS update moves the Pi without
moving the runner. So the gate fails once the Pi measurement is more than
PI_CALIBRATION_STALE_DAYS old, and names the re-check that re-measures every
point and refits. Main took 11 accepts in the 3 days before the first
calibration; 11 changes of ~2 % each move the head by about as much as the
fit's own error, so 3 days is the limit.

Accepting (`--accept --reason "..."`) makes HEAD the reference and rewrites
every config's Ir with this run's. It refuses a dirty tree, so the recorded
commit is the measured code. A PR whose tools/bench/throughput_baseline.json
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
import datetime
import io
import json
import math
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
from typing import TypeVar

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
LIVE_FASTEST = 2
LIVE_TOLERANCE_PCT = 2.0
LIVE_CORES = "0,1"
PI_FLOOR_X = 2.0
SHORTFALL_TOLERANCE_PCT = 2.0
PI_FLOOR_FAILS = True
PI_CALIBRATION_STALE_DAYS = 3
CALIBRATION_SPREAD_PCT = 5.0
CALIBRATION_SPEED_MARGIN_PCT = 30.0
RECHECK_COMMAND = "python3 scripts/throughput_recheck.py"
MEASURED_TREES = ("rust", "tools/bench/configs", "tools/sim/gcode")
REFERENCE_DIR = "throughput-reference"
AB_LAB_DIR = "throughput-ab"
EXTRACTED_MARKER = ".extracted"
CI_ROOT = Path("/home/runner/work/serval/serval")
CI_CARGO_HOME = "/home/runner/.cargo"
DOCKER_CACHE = RUST / "target" / "throughput-gate"
ACCEPT_BLOCK = re.compile(r"```throughput-accept[ \t]*\n(.*?)```", re.S)
STREAM_SECS = re.compile(r"\bstream_secs=([0-9.]+)")
Replay = TypeVar("Replay")


class GateError(RuntimeError):
    pass


@dataclass(frozen=True)
class Measurement:
    """One config's measurements. Costs are live wall seconds per stream
    second, each side's the mean of its LIVE_FASTEST fastest runs; the
    calibration side replays pi_calibration.commit."""

    ir: int
    stream_secs: float
    live_ratio: float
    calibration_ratio: float
    calibration_cost: float
    calibration_spread_pct: float
    live_wall_secs: float
    reference_wall_secs: float
    calibration_wall_secs: float
    cpu: str


@dataclass(frozen=True)
class Verdict:
    config: str
    measurement: Measurement
    ir_delta_pct: float
    predicted_x: tuple[float, float]
    problems: tuple[str, ...]
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class FloorShortfall:
    """A config's Pi 2-core worst windows below PI_FLOOR_X, as the gate
    predicts them from (Ir, the live ratio), owned by GitHub issue `issue`."""

    issue: int
    predicted_x: tuple[float, float]

    def __post_init__(self) -> None:
        if not all(value < PI_FLOOR_X for value in self.predicted_x):
            raise ValueError(
                f"a floor shortfall pins predictions below {PI_FLOOR_X}x, "
                f"not {self.predicted_x}"
            )


FLOOR_SHORTFALLS: dict[str, FloorShortfall] = {
    "tools/bench/configs/trident_kernel.cfg": FloorShortfall(
        issue=693, predicted_x=(1.63, 1.37)
    ),
}
assert set(FLOOR_SHORTFALLS) <= set(CONFIGS), (
    "a floor pin names an ungated config"
)


def run(
    command: list[str],
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
) -> str:
    result = subprocess.run(
        command,
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
        env=None if env is None else {**os.environ, **env},
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


def fastest(samples: list[float]) -> list[float]:
    if len(samples) < LIVE_FASTEST:
        raise GateError(
            f"the live A/B needs {LIVE_FASTEST} runs of each side, got {len(samples)}"
        )
    return sorted(samples)[:LIVE_FASTEST]


def fastest_mean(samples: list[float]) -> float:
    return statistics.fmean(fastest(samples))


def fastest_spread_pct(samples: list[float]) -> float:
    runs = fastest(samples)
    return 100.0 * (runs[-1] / runs[0] - 1.0)


def live_ratio(
    head_secs_per_stream_sec: list[float],
    reference_secs_per_stream_sec: list[float],
) -> float:
    return fastest_mean(head_secs_per_stream_sec) / fastest_mean(
        reference_secs_per_stream_sec
    )


LiveRuns = list[tuple[float, float]]


def summarise(
    ir: int,
    stream: float,
    head: LiveRuns,
    reference: LiveRuns,
    calibration: LiveRuns,
    cpu: str,
) -> Measurement:
    """A config's measurement from its slice's Ir and each side's live
    (cost, wall) runs."""

    def costs(runs: LiveRuns) -> list[float]:
        return [cost for cost, _ in runs]

    def wall(runs: LiveRuns) -> float:
        return fastest_mean([secs for _, secs in runs])

    return Measurement(
        ir=ir,
        stream_secs=stream,
        live_ratio=live_ratio(costs(head), costs(reference)),
        calibration_ratio=live_ratio(costs(head), costs(calibration)),
        calibration_cost=fastest_mean(costs(calibration)),
        calibration_spread_pct=fastest_spread_pct(costs(calibration)),
        live_wall_secs=wall(head),
        reference_wall_secs=wall(reference),
        calibration_wall_secs=wall(calibration),
        cpu=cpu,
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


def sync_tree(source: Path, dest: Path) -> None:
    wanted = set()
    for directory, subdirs, files in os.walk(source):
        subdirs[:] = [name for name in subdirs if name != "target"]
        for name in files:
            path = Path(directory) / name
            if path.is_symlink():
                raise GateError(
                    f"{path} is a symlink, which the A/B lab cannot copy"
                )
            relative = path.relative_to(source)
            wanted.add(relative)
            target = dest / relative
            content = path.read_bytes()
            if target.is_file() and target.read_bytes() == content:
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
            shutil.copymode(path, target)
    for directory, _, files in os.walk(dest):
        for name in files:
            path = Path(directory) / name
            if path.relative_to(dest) not in wanted:
                path.unlink()


def build_in_lab(source_rust: Path, side: str) -> Path:
    lab = RUST / "target" / AB_LAB_DIR
    sync_tree(source_rust, lab / "rust")
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
        cwd=lab / "rust",
        env={"CARGO_TARGET_DIR": str(lab / "target")},
    )
    bench = lab / f"{side}-gcode_replay_bench"
    shutil.copy2(
        lab / "target" / "release" / "examples" / "gcode_replay_bench", bench
    )
    return bench


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
    bench: Path, sliced: Path, config: Path, core: int, out_file: Path
) -> tuple[int, float]:
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
            str(config),
            "--synchronous",
            "--digest",
        ]
    )
    return instruction_count(out_file.read_text()), stream_secs(stdout)


def write_slice(tree: Path, sliced: Path) -> None:
    lines = (tree / GCODE).read_text().splitlines(keepends=True)
    if len(lines) < SLICE_LINES:
        raise GateError(
            f"{tree / GCODE} has {len(lines)} lines, the slice wants {SLICE_LINES}"
        )
    sliced.write_text("".join(lines[:SLICE_LINES]))


def measure_instructions(
    replays: dict[Replay, tuple[Path, Path, str]], cores: list[int]
) -> dict[Replay, tuple[int, float]]:
    """Ir and stream seconds of each (bench, tree, config) replay of the slice."""
    started = time.monotonic()
    with tempfile.TemporaryDirectory() as scratch_dir:
        scratch = Path(scratch_dir)
        slices = {}
        for tree in {tree for _, tree, _ in replays.values()}:
            slices[tree] = scratch / f"slice-{len(slices)}.gcode"
            write_slice(tree, slices[tree])
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=len(cores)
        ) as pool:
            futures = {
                key: pool.submit(
                    count_instructions,
                    bench,
                    slices[tree],
                    tree / config,
                    cores[index % len(cores)],
                    scratch / f"cachegrind-{index}.out",
                )
                for index, (key, (bench, tree, config)) in enumerate(
                    replays.items()
                )
            }
            counted = {key: future.result() for key, future in futures.items()}
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
                "there: accept a new baseline, or, if the calibration commit "
                f"lacks it, re-pin that with {RECHECK_COMMAND} "
                "--calibration-commit REV"
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
    sides: dict[str, tuple[Path, Path]], configs: tuple[str, ...]
) -> dict[tuple[str, str], LiveRuns]:
    """Each (config, side)'s live (cost, wall) runs of the full print, the
    sides replaying (bench, tree) in turn, the order reversed every rep."""
    names = list(sides)
    runs = {(config, side): [] for config in configs for side in names}
    for rep in range(LIVE_REPS):
        order = names if rep % 2 == 0 else names[::-1]
        for config in configs:
            for side in order:
                runs[config, side].append(
                    live_secs_per_stream_sec(*sides[side], config)
                )
            walls = ", ".join(
                f"{side} {runs[config, side][-1][1]:.2f} s" for side in names
            )
            print(
                f"live rep {rep + 1}/{LIVE_REPS} {config}: {walls}",
                file=sys.stderr,
                flush=True,
            )
    return runs


def extracted(commit: str, role: str) -> Path:
    tree = reference_tree(RUST / "target", commit)
    if not (tree / EXTRACTED_MARKER).exists():
        raise GateError(f"the {role} {commit} is not extracted at {tree}")
    return tree


def measure_here(
    configs: tuple[str, ...], reference_commit: str, calibration_commit: str
) -> dict[str, Measurement]:
    """The reference and the calibration replay from their extracted trees;
    one commit in both roles replays once."""
    cores = gate_cores()
    trees = {
        "reference": extracted(reference_commit, "reference"),
        "calibration": extracted(calibration_commit, "calibration commit"),
    }
    if calibration_commit == reference_commit:
        del trees["calibration"]
    sides = {
        side: (build_in_lab(tree / "rust", side), tree)
        for side, tree in trees.items()
    }
    sides["head"] = (build_in_lab(RUST, "head"), ROOT)
    counted = measure_instructions(
        {config: (sides["head"][0], ROOT, config) for config in configs}, cores
    )
    runs = measure_live(sides, configs)
    calibration_side = "calibration" if "calibration" in sides else "reference"
    cpu = cpu_model()
    return {
        config: summarise(
            *counted[config],
            runs[config, "head"],
            runs[config, "reference"],
            runs[config, calibration_side],
            cpu,
        )
        for config in configs
    }


def cpu_model() -> str:
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        key, _, value = line.partition(":")
        if key.strip() == "model name":
            return " ".join(value.split())
    raise GateError("/proc/cpuinfo names no CPU model")


def run_in_image(image: str, script: str, arguments: list[str]) -> None:
    """Run `python3 script arguments` in the image, the checkout and cargo
    home mounted at the GitHub runner's paths and DOCKER_CACHE/target as its
    rust/target."""
    target = DOCKER_CACHE / "target"
    cargo_home = DOCKER_CACHE / "cargo-home"
    target.mkdir(parents=True, exist_ok=True)
    cargo_home.mkdir(parents=True, exist_ok=True)
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
        script,
        *arguments,
    ]
    print(f"running {script} in {image}", file=sys.stderr, flush=True)
    if subprocess.run(command, check=False).returncode != 0:
        raise GateError(f"{script} {' '.join(arguments)} failed in {image}")


def measure_in_docker(
    image: str,
    configs: tuple[str, ...],
    reference_commit: str,
    calibration_commit: str,
) -> dict[str, Measurement]:
    target = DOCKER_CACHE / "target"
    target.mkdir(parents=True, exist_ok=True)
    prepare_reference(reference_commit, target)
    prepare_reference(calibration_commit, target)
    result_name = "throughput-measure.json"
    run_in_image(
        image,
        "scripts/throughput_gate.py",
        [
            "--measure-json",
            f"rust/target/{result_name}",
            "--reference",
            reference_commit,
            "--calibration",
            calibration_commit,
            "--configs",
            *configs,
        ],
    )
    raw = json.loads((target / result_name).read_text())
    return {config: Measurement(**raw[config]) for config in configs}


def measure(
    configs: tuple[str, ...], reference_commit: str, calibration_commit: str
) -> dict[str, Measurement]:
    if shutil.which("valgrind"):
        prepare_reference(reference_commit, RUST / "target")
        prepare_reference(calibration_commit, RUST / "target")
        return measure_here(configs, reference_commit, calibration_commit)
    image = os.environ.get("THROUGHPUT_GATE_IMAGE")
    if not image:
        raise GateError(
            "valgrind is not installed and THROUGHPUT_GATE_IMAGE is unset: "
            "run ./scripts/ci.sh throughput-gate"
        )
    return measure_in_docker(
        image, configs, reference_commit, calibration_commit
    )


def load_baseline() -> dict:
    return json.loads(BASELINE.read_text()) if BASELINE.exists() else {}


def write_baseline(baseline: dict) -> None:
    BASELINE.write_text(json.dumps(baseline, indent=2) + "\n")


def predicted_worst_x(
    model: dict, measurement: Measurement
) -> tuple[float, float]:
    ir, live = model["ir"], model["live"]
    ir_per_stream_sec = measurement.ir / measurement.stream_secs
    return (
        math.exp(ir["intercept"] - ir["factor"] * math.log(ir_per_stream_sec)),
        math.exp(
            live["intercept"]
            - live["factor"] * math.log(measurement.calibration_ratio)
        ),
    )


def speed_vs_pi(live: dict, measurement: Measurement) -> float:
    return live["pi_cost"] / measurement.calibration_cost


def speed_envelope(live: dict) -> tuple[float, float]:
    margin = 1.0 + CALIBRATION_SPEED_MARGIN_PCT / 100.0
    return live["speed_x"]["min"] / margin, live["speed_x"]["max"] * margin


def calibration_replay_problems(
    live: dict, measurement: Measurement
) -> list[str]:
    problems = []
    if measurement.calibration_spread_pct > CALIBRATION_SPREAD_PCT:
        problems.append(
            f"the calibration replay's fastest {LIVE_FASTEST} runs differ by "
            f"{measurement.calibration_spread_pct:.2f} % (limit "
            f"{CALIBRATION_SPREAD_PCT} %): this machine's timing is too "
            "noisy to normalise by"
        )
    speed = speed_vs_pi(live, measurement)
    low, high = speed_envelope(live)
    if not low <= speed <= high:
        problems.append(
            f"this machine replays the calibration commit {speed:.2f}x as "
            f"fast as the Pi, outside the {low:.2f}-{high:.2f}x the fit's jobs "
            f"span (widened {CALIBRATION_SPEED_MARGIN_PCT} %): the live "
            f"prediction would extrapolate; extend the fit with {RECHECK_COMMAND}"
        )
    return problems


def calibration_problem(
    baseline: dict, configs: tuple[str, ...], now: datetime.datetime
) -> str | None:
    calibration = baseline.get("pi_calibration", {})
    measured_at = calibration.get("measured_at")
    if not measured_at:
        return f"the baseline records no Pi calibration: run {RECHECK_COMMAND}"
    missing = [c for c in configs if c not in calibration.get("configs", {})]
    if missing:
        return f"the Pi calibration has no model for {missing}: run {RECHECK_COMMAND}"
    age = now - datetime.datetime.fromisoformat(measured_at)
    if age > datetime.timedelta(days=PI_CALIBRATION_STALE_DAYS):
        return (
            f"the Pi was last measured {age.days} days ago ({measured_at}, at "
            f"{calibration['commit']}), past the {PI_CALIBRATION_STALE_DAYS}-day "
            f"bound: re-measure it and refit with {RECHECK_COMMAND}"
        )
    return None


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


def floor_miss(source: str, predicted: float) -> str | None:
    if predicted >= PI_FLOOR_X:
        return None
    return (
        f"the Pi 2-core worst window {source} predicts, {predicted:.2f}x, "
        f"is below the {PI_FLOOR_X:.1f}x floor"
    )


def shortfall_miss(
    source: str, predicted: float, pinned_x: float, issue: int
) -> str | None:
    window = f"the Pi 2-core worst window {source} predicts, {predicted:.2f}x,"
    if predicted >= PI_FLOOR_X:
        return (
            f"{window} reaches the {PI_FLOOR_X:.1f}x floor: unpin the "
            f"shortfall FLOOR_SHORTFALLS holds for #{issue}"
        )
    drift_pct = 100.0 * (predicted - pinned_x) / pinned_x
    if drift_pct < -SHORTFALL_TOLERANCE_PCT:
        return (
            f"{window} grew the floor shortfall pinned at "
            f"{pinned_x:.2f}x for #{issue} ({drift_pct:+.2f} %)"
        )
    if drift_pct > SHORTFALL_TOLERANCE_PCT:
        return (
            f"{window} shrank the floor shortfall pinned at "
            f"{pinned_x:.2f}x for #{issue} ({drift_pct:+.2f} %): re-pin it"
        )
    return None


def judge(
    config: str,
    entry: dict,
    measurement: Measurement,
    model: dict,
    shortfalls: dict[str, FloorShortfall] = FLOOR_SHORTFALLS,
) -> Verdict:
    ir_delta_pct = 100.0 * (measurement.ir - entry["ir"]) / entry["ir"]
    predicted = predicted_worst_x(model, measurement)
    problems = [
        *drift_problems("Ir", ir_delta_pct, IR_TOLERANCE_PCT),
        *drift_problems(
            "live 2-core wall per stream second",
            100.0 * (measurement.live_ratio - 1.0),
            LIVE_TOLERANCE_PCT,
        ),
        *calibration_replay_problems(model["live"], measurement),
    ]
    pin = shortfalls.get(config)
    floor_misses = []
    for index, (source, value) in enumerate(
        zip(("Ir", "the live cost"), predicted)
    ):
        miss = (
            floor_miss(source, value)
            if pin is None
            else shortfall_miss(
                source, value, pin.predicted_x[index], pin.issue
            )
        )
        if miss:
            floor_misses.append(miss)
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


def model_summary(model: dict, measurement: Measurement) -> list[str]:
    ir, live = model["ir"], model["live"]
    low, high = speed_envelope(live)
    fitted_on = live["by_cpu"].get(measurement.cpu)
    this_cpu = (
        "a CPU model the fit did not see"
        if fitted_on is None
        else f"max {fitted_on['max_residual_pct']:.1f} %, leave-one-out "
        f"{fitted_on['max_loo_residual_pct']:.1f} % on this CPU model"
    )
    return [
        f"  Pi models: Ir factor {ir['factor']:.2f} (calibration error max "
        f"{ir['max_residual_pct']:.1f} %, leave-one-out "
        f"{ir['max_loo_residual_pct']:.1f} %), live factor "
        f"{live['factor']:.2f} (max {live['max_residual_pct']:.1f} %, "
        f"leave-one-out {live['max_loo_residual_pct']:.1f} % over "
        f"{len(live['by_cpu'])} CPU models; {this_cpu})",
        f"  calibration replay: {speed_vs_pi(live, measurement):.2f}x the "
        f"Pi's speed (fit {low:.2f}-{high:.2f}x), fastest {LIVE_FASTEST} "
        f"{measurement.calibration_spread_pct:.2f} % apart (limit "
        f"{CALIBRATION_SPREAD_PCT} %)",
    ]


def report(
    verdicts: list[Verdict], reference_commit: str, calibration: dict
) -> None:
    print(f"reference {reference_commit}")
    stale_at = datetime.datetime.fromisoformat(
        calibration["measured_at"]
    ) + datetime.timedelta(days=PI_CALIBRATION_STALE_DAYS)
    print(
        f"Pi calibration at {calibration['commit']}, measured "
        f"{calibration['measured_at']}, stale from {stale_at.isoformat()} "
        f"(re-check: {RECHECK_COMMAND}); this CPU: {verdicts[0].measurement.cpu}"
    )
    print(
        f"{'config':<62} {'Ir':>15} {'delta':>8} {'live':>8} {'vs cal':>8} "
        f"{'Pi pred Ir':>10} {'Pi pred live':>12}"
    )
    for verdict in verdicts:
        m = verdict.measurement
        predicted = tuple(f"{value:.2f}x" for value in verdict.predicted_x)
        print(
            f"{verdict.config:<62} {m.ir:>15} {verdict.ir_delta_pct:>+7.2f}% "
            f"{100.0 * (m.live_ratio - 1.0):>+7.2f}% "
            f"{100.0 * (m.calibration_ratio - 1.0):>+7.2f}% "
            f"{predicted[0]:>10} {predicted[1]:>12}"
        )
        print(
            f"  live full print, fastest {LIVE_FASTEST} of {LIVE_REPS}: "
            f"{m.live_wall_secs:.2f} s, reference {m.reference_wall_secs:.2f} s, "
            f"calibration {m.calibration_wall_secs:.2f} s"
        )
        for line in model_summary(calibration["configs"][verdict.config], m):
            print(line)
        for problem in verdict.problems:
            print(f"  FAIL: {problem}")
        for warning in verdict.warnings:
            print(f"  WARN (PI_FLOOR_FAILS is off): {warning}")


def head_commit() -> str:
    dirty = run(
        ["git", "status", "--porcelain", "--", *MEASURED_TREES], cwd=ROOT
    )
    if dirty.strip():
        raise GateError(
            "accepting records HEAD as the measured code, but "
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
    parser.add_argument("--accept-base")
    parser.add_argument("--measure-json", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--reference", help=argparse.SUPPRESS)
    parser.add_argument("--calibration", help=argparse.SUPPRESS)
    parser.add_argument(
        "--configs", nargs="+", default=list(CONFIGS), help=argparse.SUPPRESS
    )
    args = parser.parse_args()
    configs = tuple(args.configs)

    if args.measure_json:
        measurements = measure_here(configs, args.reference, args.calibration)
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
    commit = head_commit() if args.accept else ""

    baseline = load_baseline()
    problem = baseline_problem(baseline, configs)
    if problem and not args.accept:
        raise GateError(problem)
    problem = calibration_problem(
        baseline, configs, datetime.datetime.now(datetime.timezone.utc)
    )
    if problem:
        raise GateError(problem)
    calibration = baseline["pi_calibration"]
    reference_commit = baseline.get("commit") or commit
    entries = baseline.get("configs", {})

    measurements = measure(configs, reference_commit, calibration["commit"])
    if args.accept:
        entries = {
            config: {
                "ir": measurements[config].ir,
                "stream_secs": measurements[config].stream_secs,
            }
            for config in configs
        }
        reference_commit = commit
        measurements = {
            config: dataclasses.replace(measurement, live_ratio=1.0)
            for config, measurement in measurements.items()
        }
        write_baseline(
            {
                "slice_lines": SLICE_LINES,
                "commit": reference_commit,
                "reason": args.reason.strip(),
                "configs": entries,
                "pi_calibration": calibration,
            }
        )
        print(f"wrote {BASELINE.relative_to(ROOT)}")

    verdicts = [
        judge(c, entries[c], measurements[c], calibration["configs"][c])
        for c in configs
    ]
    report(verdicts, reference_commit, calibration)
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
