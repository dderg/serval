#!/usr/bin/env python3
"""Deterministic replay-throughput gate: instructions of this repo's code.

    python3 scripts/throughput_gate.py [--accept --reason R]
        [--calibrate-pi-worst CFG=X[,X...] ...] [--accept-base REV]
    ./scripts/ci.sh throughput-gate [same arguments]

Wall-clock throughput on a shared CI runner is noise; instruction counts are
not. Each config in tools/bench/throughput_baseline.json replays the first
SLICE_LINES lines of the dense print through `gcode_replay_bench
--synchronous --digest` under `taskset -c 0` and valgrind cachegrind
(`--cache-sim=no`): one thread (no shaper thread, and a one-core affinity
leaves the worker pool without helpers), so the count is a pure function of
the code. Only instructions whose debug-info source file lies in this repo's
rust/ tree count (`repo Ir`): glibc picks memcpy/malloc variants per CPU and
the std/crossbeam sync paths spin by schedule, so libc, ld.so, std and every
registry crate are left out.

The gate fails when, for any config:
- repo Ir grew more than THRESHOLD_PCT over its baseline (a regression);
- repo Ir shrank more than THRESHOLD_PCT (an improvement: accept it so the
  baseline ratchets down and the gain stays locked in);
- the predicted Pi 5 2-core worst 5 s window falls below PI_FLOOR_X, bar a
  config FLOOR_SHORTFALLS pins below it to the issue that owns the
  shortfall: that config fails instead when its prediction leaves the pinned
  one by more than THRESHOLD_PCT (grown: a regression; shrunk: re-pin) or
  reaches the floor (unpin).

The prediction is K x stream_secs / repo_Ir over the slice, the realtime
factor scaled by the instruction rate the Pi sustains in the dense print's
worst window. K comes from a calibration: the Pi 2-core worst window of the
full print (`taskset -c 0,1 gcode_replay_bench <dense.gcode> --config CFG`,
the lowest of two runs), measured at the commit the gate runs at:

    python3 scripts/throughput_gate.py --calibrate-pi-worst \\
        p3_voron_direct_drive_ramp=2.12,2.13 trident_kernel=2.29,2.30 ...

Accepting (`--accept --reason "..."`) rewrites every config's baseline with
this run's repo Ir and stream seconds, the reason and HEAD; it and the
calibration refuse a dirty rust/ tree, so the recorded commit is the measured
code. A PR whose tools/bench/throughput_baseline.json differs from its base
(--accept-base, which CI sets to the merge commit's base parent) must carry
the reason in a fenced block in its body, read from $PR_BODY:

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
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUST = ROOT / "rust"
BASELINE = ROOT / "tools" / "bench" / "throughput_baseline.json"
GCODE = ROOT / "tools" / "sim" / "gcode" / "voron_dense_top_layers.gcode"
CONFIGS = (
    "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg",
    "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg",
    "tools/bench/configs/trident_kernel.cfg",
)
SLICE_LINES = 1000
THRESHOLD_PCT = 2.0
PI_FLOOR_X = 2.0
MEASURED_TREES = ("rust", "tools/bench/configs", "tools/sim/gcode")
CI_ROOT = Path("/home/runner/work/serval/serval")
CI_CARGO_HOME = "/home/runner/.cargo"
DOCKER_CACHE = RUST / "target" / "throughput-gate"
ACCEPT_BLOCK = re.compile(r"```throughput-accept[ \t]*\n(.*?)```", re.S)
STREAM_SECS = re.compile(r"\bstream_secs=([0-9.]+)")


class GateError(RuntimeError):
    pass


@dataclass(frozen=True)
class Measurement:
    repo_ir: int
    total_ir: int
    stream_secs: float


@dataclass(frozen=True)
class Verdict:
    config: str
    repo_ir: int
    baseline_ir: int
    stream_secs: float
    predicted_x: float | None
    problems: tuple[str, ...]

    @property
    def delta_pct(self) -> float:
        return 100.0 * (self.repo_ir - self.baseline_ir) / self.baseline_ir


@dataclass(frozen=True)
class FloorShortfall:
    issue: int
    predicted_x: float

    def __post_init__(self) -> None:
        if not self.predicted_x < PI_FLOOR_X:
            raise ValueError(
                f"a floor shortfall pins a prediction below {PI_FLOOR_X}x, "
                f"not {self.predicted_x}x"
            )


FLOOR_SHORTFALLS: dict[str, FloorShortfall] = {}
assert set(FLOOR_SHORTFALLS) <= set(CONFIGS), (
    "a floor pin names an ungated config"
)


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


def is_repo_source(path: str, rust_root: Path) -> bool:
    if not path.startswith("/"):
        return False
    try:
        relative = Path(os.path.normpath(path)).relative_to(rust_root)
    except ValueError:
        return False
    return relative.parts[:1] != ("target",)


def instruction_counts(cachegrind_out: str, rust_root: Path) -> tuple[int, int]:
    repo = total = 0
    summary = None
    in_repo = False
    verdicts: dict[str, bool] = {}
    for line in cachegrind_out.splitlines():
        if line.startswith("fl="):
            path = line[3:]
            if path not in verdicts:
                verdicts[path] = is_repo_source(path, rust_root)
            in_repo = verdicts[path]
        elif line[:1].isdigit():
            fields = line.split()
            if len(fields) != 2:
                raise GateError(
                    f"cachegrind line is not '<line> <Ir>': {line!r}"
                )
            count = int(fields[1])
            total += count
            if in_repo:
                repo += count
        elif line.startswith("summary:"):
            summary = int(line.split()[1])
    if summary is None:
        raise GateError("cachegrind output has no summary line")
    if summary != total:
        raise GateError(
            f"cachegrind records sum to {total}, its summary says {summary}"
        )
    if repo == 0:
        raise GateError(
            f"no instructions attributed to sources under {rust_root}"
        )
    return repo, total


def stream_secs(bench_stdout: str) -> float:
    match = STREAM_SECS.search(bench_stdout)
    if not match:
        raise GateError(f"no stream_secs in the bench output:\n{bench_stdout}")
    return float(match.group(1))


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


def measure_here(configs: tuple[str, ...]) -> dict[str, Measurement]:
    if run(["taskset", "-c", "0", "nproc"]).strip() != "1":
        raise GateError("taskset -c 0 leaves more than one core visible")
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
        cwd=RUST,
    )
    bench = RUST / "target" / "release" / "examples" / "gcode_replay_bench"
    lines = GCODE.read_text().splitlines(keepends=True)
    if len(lines) < SLICE_LINES:
        raise GateError(
            f"{GCODE} has {len(lines)} lines, the slice wants {SLICE_LINES}"
        )
    results = {}
    with tempfile.TemporaryDirectory() as scratch:
        sliced = Path(scratch) / "slice.gcode"
        sliced.write_text("".join(lines[:SLICE_LINES]))
        for config in configs:
            started = time.monotonic()
            out_file = Path(scratch) / "cachegrind.out"
            stdout = run(
                [
                    "taskset",
                    "-c",
                    "0",
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
            repo_ir, total_ir = instruction_counts(out_file.read_text(), RUST)
            results[config] = Measurement(
                repo_ir, total_ir, stream_secs(stdout)
            )
            print(
                f"measured {config}: repo Ir {repo_ir} of {total_ir} "
                f"in {time.monotonic() - started:.0f} s",
                file=sys.stderr,
                flush=True,
            )
    return results


def measure_in_docker(
    image: str, configs: tuple[str, ...]
) -> dict[str, Measurement]:
    target = DOCKER_CACHE / "target"
    cargo_home = DOCKER_CACHE / "cargo-home"
    target.mkdir(parents=True, exist_ok=True)
    cargo_home.mkdir(parents=True, exist_ok=True)
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
        "--configs",
        *configs,
    ]
    print(f"measuring in {image}", file=sys.stderr, flush=True)
    if subprocess.run(command, check=False).returncode != 0:
        raise GateError(f"the measurement in {image} failed")
    raw = json.loads((target / result_name).read_text())
    return {config: Measurement(**raw[config]) for config in configs}


def measure(configs: tuple[str, ...]) -> dict[str, Measurement]:
    if shutil.which("valgrind"):
        return measure_here(configs)
    image = os.environ.get("THROUGHPUT_GATE_IMAGE")
    if not image:
        raise GateError(
            "valgrind is not installed and THROUGHPUT_GATE_IMAGE is unset: "
            "run ./scripts/ci.sh throughput-gate"
        )
    return measure_in_docker(image, configs)


def load_baseline() -> dict:
    return json.loads(BASELINE.read_text()) if BASELINE.exists() else {}


def write_baseline(baseline: dict) -> None:
    BASELINE.write_text(json.dumps(baseline, indent=2) + "\n")


def predicted_worst_x(entry: dict, measurement: Measurement) -> float | None:
    pi = entry.get("pi_2core")
    if pi is None:
        return None
    return pi["k"] * measurement.stream_secs / measurement.repo_ir


def floor_problem(predicted: float, pin: FloorShortfall | None) -> str | None:
    if pin is None:
        if predicted >= PI_FLOOR_X:
            return None
        return (
            f"predicted Pi 2-core worst window {predicted:.2f}x is below "
            f"the {PI_FLOOR_X:.1f}x floor"
        )
    if predicted >= PI_FLOOR_X:
        return (
            f"predicted Pi 2-core worst window {predicted:.2f}x reaches the "
            f"{PI_FLOOR_X:.1f}x floor: unpin the shortfall FLOOR_SHORTFALLS "
            f"holds for #{pin.issue}"
        )
    drift_pct = 100.0 * (predicted - pin.predicted_x) / pin.predicted_x
    if drift_pct < -THRESHOLD_PCT:
        return (
            f"predicted Pi 2-core worst window {predicted:.2f}x grew the "
            f"floor shortfall pinned at {pin.predicted_x:.2f}x for "
            f"#{pin.issue} ({drift_pct:+.2f} %)"
        )
    if drift_pct > THRESHOLD_PCT:
        return (
            f"predicted Pi 2-core worst window {predicted:.2f}x shrank the "
            f"floor shortfall pinned at {pin.predicted_x:.2f}x for "
            f"#{pin.issue} ({drift_pct:+.2f} %): re-pin it"
        )
    return None


def judge(
    config: str,
    entry: dict,
    measurement: Measurement,
    shortfalls: dict[str, FloorShortfall] = FLOOR_SHORTFALLS,
) -> Verdict:
    baseline_ir = entry["ir"]
    delta_pct = 100.0 * (measurement.repo_ir - baseline_ir) / baseline_ir
    predicted = predicted_worst_x(entry, measurement)
    problems = []
    if delta_pct > THRESHOLD_PCT:
        problems.append(
            f"repo Ir grew {delta_pct:+.2f} % (limit {THRESHOLD_PCT} %): "
            "a throughput regression; if it is the price of a wanted change, "
            "accept it with a reason"
        )
    elif delta_pct < -THRESHOLD_PCT:
        problems.append(
            f"repo Ir shrank {delta_pct:+.2f} % (beyond {THRESHOLD_PCT} %): "
            "accept the improvement so the baseline ratchets down"
        )
    if predicted is None:
        problems.append("no Pi calibration: run --calibrate-pi-worst")
    elif problem := floor_problem(predicted, shortfalls.get(config)):
        problems.append(problem)
    return Verdict(
        config,
        measurement.repo_ir,
        baseline_ir,
        measurement.stream_secs,
        predicted,
        tuple(problems),
    )


def report(verdicts: list[Verdict]) -> None:
    print(
        f"{'config':<62} {'repo Ir':>15} {'delta':>8} {'stream s':>9} "
        f"{'Pi 2-core pred':>14}"
    )
    for verdict in verdicts:
        predicted = (
            "uncalibrated"
            if verdict.predicted_x is None
            else f"{verdict.predicted_x:.2f}x"
        )
        print(
            f"{verdict.config:<62} {verdict.repo_ir:>15} "
            f"{verdict.delta_pct:>+7.2f}% {verdict.stream_secs:>9.3f} "
            f"{predicted:>14}"
        )
        for problem in verdict.problems:
            print(f"  FAIL: {problem}")


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


def measurement_json(measurements: dict[str, Measurement]) -> str:
    return json.dumps(
        {c: m.__dict__ for c, m in measurements.items()}, indent=2
    )


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
    parser.add_argument(
        "--configs", nargs="+", default=list(CONFIGS), help=argparse.SUPPRESS
    )
    args = parser.parse_args()
    configs = tuple(args.configs)

    if args.measure_json:
        measurements = measure_here(configs)
        args.measure_json.write_text(measurement_json(measurements))
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
    if not args.accept and baseline.get("slice_lines") != SLICE_LINES:
        raise GateError(
            f"the baselines were taken over {baseline.get('slice_lines')} lines, "
            f"the gate replays {SLICE_LINES}: accept new baselines"
        )
    entries = baseline.get("configs", {})
    missing = [c for c in configs if c not in entries]
    if missing and not args.accept:
        raise GateError(f"no baseline for {missing}: accept one")

    measurements = measure(configs)
    if args.accept:
        entries = {
            config: {
                "ir": measurements[config].repo_ir,
                "stream_secs": measurements[config].stream_secs,
                "reason": args.reason.strip(),
                "commit": commit,
                **(
                    {"pi_2core": entries[config]["pi_2core"]}
                    if "pi_2core" in entries.get(config, {})
                    else {}
                ),
            }
            for config in configs
        }
    for config, worst in calibrations.items():
        measurement = measurements[config]
        entries[config]["pi_2core"] = {
            "k": min(worst) * measurement.repo_ir / measurement.stream_secs,
            "worst_x": worst,
            "commit": commit,
        }
    if args.accept or calibrations:
        write_baseline({"slice_lines": SLICE_LINES, "configs": entries})
        print(f"wrote {BASELINE.relative_to(ROOT)}")

    verdicts = [judge(c, entries[c], measurements[c]) for c in configs]
    report(verdicts)
    failed = [v.config for v in verdicts if v.problems]
    if failed:
        print(
            f"throughput gate: FAIL ({len(failed)} of {len(verdicts)} configs)"
        )
        return 1
    print(
        f"throughput gate: PASS ({len(verdicts)} configs within {THRESHOLD_PCT} %)"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput gate: {exc}")
