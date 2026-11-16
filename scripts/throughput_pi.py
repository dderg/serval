#!/usr/bin/env python3
"""Measure this checkout's Pi 5 worst windows for the throughput gate.

    python3 scripts/throughput_pi.py [--pin CONFIG ...]

The gate (scripts/throughput_gate.py) fails a config either of whose Pi
predictions falls below its floor, unless the Pi itself measured the tree,
and holds a config FLOOR_SHORTFALLS pins to the Pi's window on PINNED_CORES:
a PR may measure it, qualification (scripts/qualify.py) always does. This
script takes both measurements.
Under tools/bench/lock.py it syncs the committed measured trees to the bench
Pi, builds the replay there, and runs the full dense print of every gated
config PI_RUNS times under `taskset -c LIVE_CORES`, and of every pinned
config, plus every config `--pin` names (one about to be pinned), PI_RUNS
times under `taskset -c PINNED_CORES`, the way
scripts/throughput_recheck.py measures its points. It writes each config's
worst windows (`worst_x`, and `pinned_cores_worst_x` for the pinned ones),
with the git trees they were measured on, to tools/bench/pi_window.json;
the gate credits them only while the checkout's measured trees are exactly
those. Commit the file with the change it measured. `timing` in the file
says where the bench time went: wall seconds of the sync, the build on the
Pi, the cooldown, and of every replay (its stream and wall seconds as the
replay reports them, and the ssh call around it).
"""

from __future__ import annotations

import argparse
import datetime
import json
import re
import sys
import time
from collections.abc import Callable
from pathlib import Path

import throughput_gate as gate
import throughput_recheck as recheck
from throughput_gate import GateError

PI_DIR = "throughput-pi"
WALL_SECS = re.compile(r"\bwall_secs=([0-9.]+)")


def held(pinning: list[str]) -> None:
    trees = gate.measured_trees()
    if trees is None:
        raise GateError(
            "the measured trees have uncommitted changes: commit them, so the "
            "measurement names the code it measured"
        )
    sync_s = timed(sync_to_pi)
    print("building on the Pi", file=sys.stderr, flush=True)
    build_s = timed(build_on_pi)
    time.sleep(recheck.PI_COOLDOWN_SECS)
    if recheck.pi_throttled_now():
        raise GateError("the Pi is throttling before the runs")
    measured_at = datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds"
    )
    pinned = sorted({*gate.FLOOR_SHORTFALLS, *pinning})
    measured = [
        *((config, gate.LIVE_CORES) for config in gate.CONFIGS),
        *((config, gate.PINNED_CORES) for config in pinned),
    ]
    replays = [
        replay(config, cores, run)
        for run in range(recheck.PI_RUNS)
        for config, cores in measured
    ]
    if recheck.pi_throttled_now():
        raise GateError(
            "the Pi throttled during the runs: their windows are void"
        )
    timing = {
        "sync_s": sync_s,
        "build_s": build_s,
        "cooldown_s": recheck.PI_COOLDOWN_SECS,
        "replays": replays,
    }
    gate.PI_WINDOW.write_text(
        json.dumps(
            {
                "trees": trees,
                "measured_at": measured_at,
                "worst_x": windows(replays, gate.CONFIGS, gate.LIVE_CORES),
                "pinned_cores": gate.PINNED_CORES,
                "pinned_cores_worst_x": windows(
                    replays, pinned, gate.PINNED_CORES
                ),
                "timing": timing,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {gate.PI_WINDOW.relative_to(gate.ROOT)}")
    for config, runs in windows(replays, pinned, gate.PINNED_CORES).items():
        print(f"{config} on cores {gate.PINNED_CORES}: {min(runs):.2f}x")
    replays_s = sum(record["call_s"] for record in replays)
    print(
        f"timing: sync {sync_s:.1f} s, build {build_s:.1f} s, cooldown "
        f"{recheck.PI_COOLDOWN_SECS} s, {len(replays)} replays {replays_s:.1f} s"
    )


def timed(step: Callable[[], None]) -> float:
    started = time.monotonic()
    step()
    return round(time.monotonic() - started, 1)


def sync_to_pi() -> None:
    recheck.ssh(f"mkdir -p {PI_DIR}/lab {PI_DIR}/tree")
    gate.run(
        [
            "rsync",
            "-rlp",
            "--checksum",
            "--delete",
            "--exclude",
            "target",
            f"{gate.RUST}/",
            f"{recheck.PI_HOST}:{PI_DIR}/lab/rust/",
        ]
    )
    for tree in ("tools/bench/configs", "tools/sim/gcode"):
        recheck.ssh(f"mkdir -p {PI_DIR}/tree/{tree}")
        gate.run(
            [
                "rsync",
                "-a",
                "--delete",
                f"{gate.ROOT / tree}/",
                f"{recheck.PI_HOST}:{PI_DIR}/tree/{tree}/",
            ]
        )


def build_on_pi() -> None:
    recheck.ssh(
        f"cd {PI_DIR}/lab/rust && . ~/.cargo/env && "
        "CARGO_TARGET_DIR=../target cargo build --release --locked -q "
        "-p motion-core --example gcode_replay_bench"
    )


def windows(
    replays: list[dict], configs: list[str] | tuple[str, ...], cores: str
) -> dict[str, list[float]]:
    return {
        config: [
            record["worst_x"]
            for record in replays
            if record["config"] == config and record["cores"] == cores
        ]
        for config in configs
    }


def replay(config: str, cores: str, run: int) -> dict:
    started = time.monotonic()
    out = recheck.ssh(
        f"cd {PI_DIR} && taskset -c {cores} "
        "lab/target/release/examples/gcode_replay_bench "
        f"tree/{gate.GCODE} --config tree/{config}"
    )
    call_s = round(time.monotonic() - started, 2)
    window = recheck.WORST_WINDOW.search(out)
    wall = WALL_SECS.search(out)
    if not window or not wall:
        raise GateError(
            f"no worst window or wall_secs in the Pi replay of {config} on "
            f"cores {cores}:\n{out}"
        )
    print(
        f"pi run {run + 1}/{recheck.PI_RUNS} {Path(config).stem} cores {cores}: "
        f"{window.group(1)}x",
        file=sys.stderr,
        flush=True,
    )
    return {
        "config": config,
        "cores": cores,
        "run": run,
        "worst_x": float(window.group(1)),
        "stream_secs": gate.stream_secs(out),
        "wall_secs": float(wall.group(1)),
        "call_s": call_s,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--held", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--pin",
        action="append",
        default=[],
        metavar="CONFIG",
        help=f"also measure CONFIG on cores {gate.PINNED_CORES} for a floor pin",
    )
    args = parser.parse_args()
    unknown = set(args.pin) - set(gate.CONFIGS)
    if unknown:
        raise GateError(
            f"--pin names ungated configs {sorted(unknown)}: name one of "
            f"{list(gate.CONFIGS)}"
        )
    if args.held:
        held(args.pin)
        return 0
    recheck.stream(
        [
            "uv",
            "run",
            "python",
            str(gate.ROOT / "tools" / "bench" / "lock.py"),
            "hold",
            "--lock-timeout",
            str(recheck.PI_LOCK_TIMEOUT_SECS),
            "--",
            sys.executable,
            str(Path(__file__).resolve()),
            "--held",
            *[arg for config in args.pin for arg in ("--pin", config)],
        ]
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput pi: {exc}")
