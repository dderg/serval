"""Run one sanitized gcode print on the bench while capturing STEP/DIR.

uv run python tools/bench/run.py <gcode> [--runs-dir ~/bench/runs]
    [--config tools/bench/configs/fast.cfg]
    [--host-cpus N] [--host-max-mhz M]
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path

import lock
import throttle
from capture import Capture
from pi import (
    PI_LOGS,
    REPO_CFG,
    BenchError,
    http,
    prepare_printer,
    read_deployed,
    scp_from_pi,
    upload,
)

TERMINAL_STATES = {"complete", "error", "cancelled"}
ALLOWED = {
    "G0",
    "G1",
    "G2",
    "G3",
    "G4",
    "G90",
    "G91",
    "G92",
    "G21",
    "M82",
    "M83",
    "M400",
}
DROPPED = {"M104", "M109", "M140", "M190", "M106", "M107", "M73", "G28"}
PREAMBLE = ["SET_KINEMATIC_POSITION X=0 Y=0 Z=0", "G92 E0"]
POSTAMBLE = ["M400"]
POLL_S = 0.5


def sanitize(source: str) -> tuple[list[str], dict[str, int]]:
    out = list(PREAMBLE)
    stats = {"source_lines": 0, "kept": 0, "dropped": 0, "blank_or_comment": 0}
    for number, raw in enumerate(source.splitlines(), 1):
        stats["source_lines"] += 1
        line = raw.split(";", 1)[0].strip()
        if not line:
            stats["blank_or_comment"] += 1
            continue
        command = line.split()[0].upper()
        if command in DROPPED:
            stats["dropped"] += 1
        elif command in ALLOWED:
            out.append(line)
            stats["kept"] += 1
        else:
            raise BenchError(f"line {number}: unsupported command {raw!r}")
    out.extend(POSTAMBLE)
    return out, stats


def print_stats() -> dict:
    return http("GET", "/printer/objects/query?print_stats")["status"][
        "print_stats"
    ]


def poll_print(capture: Capture, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        capture.check()
        stats = print_stats()
        if stats["state"] in TERMINAL_STATES:
            return stats
        time.sleep(POLL_S)
    raise BenchError(f"print did not finish within {timeout_s}s")


def run(
    gcode: Path,
    runs_dir: Path,
    timeout_s: float,
    config: Path = REPO_CFG,
    host_throttle: throttle.Throttle | None = None,
) -> Path:
    lines, stats = sanitize(gcode.read_text())
    run_dir = runs_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True)
    sanitized = ("\n".join(lines) + "\n").encode()
    (run_dir / "print.gcode").write_bytes(sanitized)
    (run_dir / "printer.cfg").write_text(config.read_text())

    deployed = read_deployed()
    prepare_printer(config)
    remote_name = f"bench_{run_dir.name}.gcode"
    upload(remote_name, sanitized)
    summary: dict = {
        "source_gcode": str(gcode.resolve()),
        "config": str(config.resolve()),
        "deployed": deployed,
        "sanitized_lines": len(lines),
        **stats,
    }
    with throttle.throttled(host_throttle) as applied_throttle:
        summary["host_throttle"] = applied_throttle
        execute(run_dir, remote_name, summary, timeout_s)
    return run_dir


def execute(
    run_dir: Path, remote_name: str, summary: dict, timeout_s: float
) -> None:
    capture = Capture(run_dir, both_edges_mask=0)
    final: dict = {}
    failure: BaseException | None = None
    started = time.time()
    capture.start()
    try:
        time.sleep(1)
        http(
            "POST",
            f"/printer/print/start?filename={urllib.parse.quote(remote_name)}",
        )
        final = poll_print(capture, timeout_s)
        time.sleep(1)
    except BaseException as exc:
        failure = exc
    try:
        result = capture.stop()
        summary["capture_ticks"] = result.ticks
    except BaseException as exc:
        failure = failure or exc
    ended = time.time()

    scp_from_pi(f"{PI_LOGS}/klippy.log", run_dir / "klippy.log")
    scp_from_pi(f"{PI_LOGS}/events", run_dir / "events", recursive=True)
    summary.update(
        host_start=datetime.fromtimestamp(started).isoformat(),
        host_end=datetime.fromtimestamp(ended).isoformat(),
        duration_s=ended - started,
        print_state=final.get("state"),
        print_message=final.get("message"),
    )
    (run_dir / "run.json").write_text(json.dumps(summary, indent=2))
    if failure is not None:
        raise failure
    if final["state"] != "complete":
        raise BenchError(
            f"print ended in state {final['state']!r}: {final.get('message')}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gcode", type=Path)
    parser.add_argument(
        "--runs-dir", type=Path, default=Path("~/bench/runs").expanduser()
    )
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--config", type=Path, default=REPO_CFG)
    parser.add_argument("--host-cpus", type=int)
    parser.add_argument("--host-max-mhz", type=int)
    lock.add_timeout_argument(parser)
    args = parser.parse_args()
    with lock.bench_lock(args.lock_timeout):
        print(
            run(
                args.gcode,
                args.runs_dir.expanduser(),
                args.timeout,
                args.config,
                throttle.Throttle(args.host_cpus, args.host_max_mhz),
            )
        )


if __name__ == "__main__":
    main()
