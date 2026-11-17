#!/usr/bin/env python3
"""Measure this checkout's Pi 5 worst windows for the throughput gate.

    python3 scripts/throughput_pi.py [--pin CONFIG ...]

The gate (scripts/throughput_gate.py) fails a config either of whose Pi
predictions falls below its floor, unless the Pi itself measured the tree,
and holds a config FLOOR_SHORTFALLS pins to the Pi's window on PINNED_CORES:
a PR may measure it, qualification (scripts/qualify.py) always does. This
script takes both measurements.
It syncs the committed measured trees to the bench
Pi, builds the replay there, and runs the full dense print of every gated
config PI_RUNS times under `taskset -c LIVE_CORES`, and of every pinned
config, plus every config `--pin` names (one about to be pinned), PI_RUNS
times under `taskset -c PINNED_CORES`, the way
scripts/throughput_recheck.py measures its points. It writes each config's
worst windows (`worst_x`, and `pinned_cores_worst_x` for the pinned ones),
with the git trees they were measured on, to tools/bench/pi_window.json;
the gate credits them only while the checkout's measured trees are exactly
those. Commit the file with the change it measured.

It takes the bench lock (tools/bench/lock.py) once per hold and releases it
in between, so it refuses to run inside a hold; each hold is planned the way
throughput_recheck.py plans its own. A build hold: deploy.py settle, sync,
cargo build, then IDENTITY next to the binary (the trees and the binary's
sha256); skipped when the Pi's IDENTITY already names these trees and the
binary beside it. Then a measure hold: deploy.py settle, the identity
verified (a binary another run rebuilt is built again in a new build hold
and the measure hold retried), the cooldown, no throttling and no capped
cpufreq policy, the replays, no throttling again. A rerun after a lost hold
starts again from the hold it lost. `timing.holds` in the file says where
the bench time went, per hold: its lock wait and held seconds, what settle
undid, and for a build the sync and build seconds, for a measure the
cpufreq policies, the cooldown and every replay (its stream and wall
seconds as the replay reports them, and the ssh call around it).
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import sys
import time
from pathlib import Path

import throughput_gate as gate
import throughput_recheck as recheck
from throughput_gate import GateError
from throughput_recheck import HoldKind

PI_DIR = "throughput-pi"
RELEASE_BENCH = "lab/target/release/examples/gcode_replay_bench"
BINARY = f"{PI_DIR}/{RELEASE_BENCH}"
IDENTITY = f"{BINARY}.identity.json"
WALL_SECS = re.compile(r"\bwall_secs=([0-9.]+)")


def measure(pinning: list[str]) -> None:
    recheck.refuse_nested_hold()
    trees = committed_trees()
    replays = recheck.PI_RUNS * (len(gate.CONFIGS) + len(pinned(pinning)))
    recheck.require_fits(HoldKind.BUILD, recheck.build_plan_s())
    recheck.require_fits(HoldKind.MEASURE, recheck.measure_plan_s(replays))
    holds = []
    for _ in range(recheck.STALE_ATTEMPTS):
        if not built_on_pi(trees):
            holds.append(take(HoldKind.BUILD, pinning, trees))
        hold = take(HoldKind.MEASURE, pinning, trees)
        holds.append(hold)
        if "stale" not in hold:
            write_window(trees, pinning, holds)
            return
        print(
            f"{BINARY} changed under the measure hold: rebuilding",
            file=sys.stderr,
            flush=True,
        )
    raise GateError(
        f"{BINARY} changed on the Pi under {recheck.STALE_ATTEMPTS} measure "
        "holds: is another throughput_pi.py building there?"
    )


def committed_trees() -> dict[str, str]:
    trees = gate.measured_trees()
    if trees is None:
        raise GateError(
            "the measured trees have uncommitted changes: commit them, so the "
            "measurement names the code it measured"
        )
    return trees


def pinned(pinning: list[str]) -> list[str]:
    return sorted({*gate.FLOOR_SHORTFALLS, *pinning})


def take(kind: HoldKind, pinning: list[str], trees: dict[str, str]) -> dict:
    hold = recheck.take_hold(
        kind,
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--hold",
            kind.value,
            *[arg for config in pinning for arg in ("--pin", config)],
        ],
    )
    if hold.pop("trees") != trees:
        raise GateError(
            f"the measured trees changed during the {kind.value} hold: rerun "
            "on a checkout nobody commits to meanwhile"
        )
    return hold


def built_on_pi(trees: dict[str, str]) -> bool:
    recorded = recheck.ssh(f"cat {IDENTITY} 2>/dev/null || true").strip()
    if not recorded:
        return False
    return json.loads(recorded) == {"trees": trees, "sha256": binary_sha256()}


def binary_sha256() -> str:
    listing = recheck.ssh(f"sha256sum {BINARY} 2>/dev/null || true").split()
    return listing[0] if listing else ""


def build_held(result: Path) -> None:
    started = time.time()
    restored = recheck.settle_bench()
    trees = committed_trees()
    recheck.ssh(f"rm -f {IDENTITY}")
    sync_s = recheck.timed(sync_to_pi)
    print("building on the Pi", file=sys.stderr, flush=True)
    build_s = recheck.timed(build_on_pi)
    sha256 = binary_sha256()
    if not sha256:
        raise GateError(f"the Pi build left no {BINARY}")
    identity = json.dumps({"trees": trees, "sha256": sha256})
    recheck.ssh(f"printf '%s\\n' {shlex.quote(identity)} > {IDENTITY}")
    recheck.write_result(
        result,
        started,
        {
            "trees": trees,
            "restored": restored,
            "sync_s": sync_s,
            "build_s": build_s,
            "sha256": sha256,
        },
    )


def measure_held(pinning: list[str], result: Path) -> None:
    started = time.time()
    restored = recheck.settle_bench()
    trees = committed_trees()
    if not built_on_pi(trees):
        recheck.write_result(
            result,
            started,
            {"trees": trees, "restored": restored, "stale": True},
        )
        return
    time.sleep(recheck.PI_COOLDOWN_SECS)
    if recheck.pi_throttled_now():
        raise GateError("the Pi is throttling before the runs")
    governors = recheck.governors()
    measured_at = recheck.utc_now()
    measured = [
        *((config, gate.LIVE_CORES) for config in gate.CONFIGS),
        *((config, gate.PINNED_CORES) for config in pinned(pinning)),
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
    recheck.write_result(
        result,
        started,
        {
            "trees": trees,
            "restored": restored,
            "governors": governors,
            "cooldown_s": recheck.PI_COOLDOWN_SECS,
            "measured_at": measured_at,
            "replays": replays,
        },
    )


def write_window(
    trees: dict[str, str], pinning: list[str], holds: list[dict]
) -> None:
    measured_at = holds[-1].pop("measured_at")
    replays = holds[-1]["replays"]
    timing = {
        "holds": [
            {"index": index, **hold} for index, hold in enumerate(holds, 1)
        ]
    }
    gate.PI_WINDOW.write_text(
        json.dumps(
            {
                "trees": trees,
                "measured_at": measured_at,
                "worst_x": windows(replays, gate.CONFIGS, gate.LIVE_CORES),
                "pinned_cores": gate.PINNED_CORES,
                "pinned_cores_worst_x": windows(
                    replays, pinned(pinning), gate.PINNED_CORES
                ),
                "timing": timing,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {gate.PI_WINDOW.relative_to(gate.ROOT)}")
    pinned_windows = windows(replays, pinned(pinning), gate.PINNED_CORES)
    for config, runs in pinned_windows.items():
        print(f"{config} on cores {gate.PINNED_CORES}: {min(runs):.2f}x")
    for hold in timing["holds"]:
        print(timing_line(hold))


def timing_line(hold: dict) -> str:
    spent = (
        f"timing: {hold['kind']} hold {hold['index']}: lock wait "
        f"{hold['wait_s']:.1f} s, held {hold['held_s']:.1f} s"
    )
    if "stale" in hold:
        return f"{spent}, binary stale"
    if hold["kind"] == HoldKind.BUILD.value:
        return (
            f"{spent}: sync {hold['sync_s']:.1f} s, build "
            f"{hold['build_s']:.1f} s"
        )
    replays_s = sum(record["call_s"] for record in hold["replays"])
    return (
        f"{spent}: cooldown {hold['cooldown_s']} s, "
        f"{len(hold['replays'])} replays {replays_s:.1f} s"
    )


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
        f"cd {PI_DIR} && taskset -c {cores} {RELEASE_BENCH} "
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
    parser.add_argument(
        "--hold",
        choices=[kind.value for kind in HoldKind],
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--result", type=Path, help=argparse.SUPPRESS)
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
    if args.hold == HoldKind.BUILD.value:
        build_held(args.result)
    elif args.hold == HoldKind.MEASURE.value:
        measure_held(args.pin, args.result)
    else:
        measure(args.pin)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except GateError as exc:
        sys.exit(f"throughput pi: {exc}")
