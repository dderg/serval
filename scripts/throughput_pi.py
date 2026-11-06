#!/usr/bin/env python3
"""Measure this checkout's Pi 5 2-core worst windows for the throughput gate.

    python3 scripts/throughput_pi.py

The gate (scripts/throughput_gate.py) fails a config either of whose Pi
predictions falls below its floor, unless the Pi itself measured the tree:
this script takes that measurement. Under tools/bench/lock.py it syncs the
committed measured trees to the bench Pi, builds the replay there, and runs
the full dense print of every gated config PI_RUNS times under `taskset -c
0,1`, the way scripts/throughput_recheck.py measures its points. It writes
each config's worst windows, with the git trees they were measured on, to
tools/bench/pi_window.json; the gate credits them only while the checkout's
measured trees are exactly those. Commit the file with the change it
measured. Every config FLOOR_SHORTFALLS pins, and every config named by
`--pin` (one about to be pinned), is also run PI_RUNS times on PINNED_CORES,
and the lowest of those windows, which a pin records as its
`pinned_cores_x`, is printed.
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import time
from pathlib import Path

import throughput_gate as gate
import throughput_recheck as recheck
from throughput_gate import GateError

PI_DIR = "throughput-pi"


def held(pinning: list[str]) -> None:
    unknown = set(pinning) - set(gate.CONFIGS)
    if unknown:
        raise GateError(f"--pin names ungated configs: {sorted(unknown)}")
    trees = gate.measured_trees()
    if trees is None:
        raise GateError(
            "the measured trees have uncommitted changes: commit them, so the "
            "measurement names the code it measured"
        )
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
    print("building on the Pi", file=sys.stderr, flush=True)
    recheck.ssh(
        f"cd {PI_DIR}/lab/rust && . ~/.cargo/env && "
        "CARGO_TARGET_DIR=../target cargo build --release --locked -q "
        "-p motion-core --example gcode_replay_bench"
    )
    time.sleep(recheck.PI_COOLDOWN_SECS)
    if recheck.pi_throttled_now():
        raise GateError("the Pi is throttling before the runs")
    measured_at = datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds"
    )
    worst = {config: [] for config in gate.CONFIGS}
    pinned_worst = {
        config: [] for config in sorted({*gate.FLOOR_SHORTFALLS, *pinning})
    }
    for run in range(recheck.PI_RUNS):
        for config in gate.CONFIGS:
            worst[config].append(pi_worst_x(config, gate.LIVE_CORES, run))
        for config in pinned_worst:
            pinned_worst[config].append(
                pi_worst_x(config, gate.PINNED_CORES, run)
            )
    if recheck.pi_throttled_now():
        raise GateError(
            "the Pi throttled during the runs: their windows are void"
        )
    gate.PI_WINDOW.write_text(
        json.dumps(
            {"trees": trees, "measured_at": measured_at, "worst_x": worst},
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {gate.PI_WINDOW.relative_to(gate.ROOT)}")
    for config, runs in pinned_worst.items():
        print(
            f"{config} on cores {gate.PINNED_CORES}: pinned_cores_x={min(runs)}"
        )


def pi_worst_x(config: str, cores: str, run: int) -> float:
    out = recheck.ssh(
        f"cd {PI_DIR} && taskset -c {cores} "
        "lab/target/release/examples/gcode_replay_bench "
        f"tree/{gate.GCODE} --config tree/{config}"
    )
    match = recheck.WORST_WINDOW.search(out)
    if not match:
        raise GateError(
            f"no worst window in the Pi replay of {config} on cores {cores}"
        )
    print(
        f"pi run {run + 1}/{recheck.PI_RUNS} {Path(config).stem} cores {cores}: "
        f"{match.group(1)}x",
        file=sys.stderr,
        flush=True,
    )
    return float(match.group(1))


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
    pin_args = [arg for config in args.pin for arg in ("--pin", config)]
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
            *pin_args,
        ]
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput pi: {exc}")
