"""Run one sanitized gcode print on the bench while capturing STEP/DIR.

uv run python tools/bench/run.py <gcode> [--runs-dir ~/bench/runs]
    [--config tools/bench/configs/fast.cfg]
    [--host-cpus N] [--host-max-mhz M] [--events [--step-tap]]
    [--no-dispatch-record]
"""

from __future__ import annotations

import argparse
import contextlib
import json
import time
import urllib.parse
import urllib.request
from collections.abc import Iterator
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
    printer_state,
    read_deployed,
    restart_klipper,
    scp_from_pi,
    ssh,
    upload,
)

STEP_TAP_ENV = "SERVAL_STEP_TAP_DIR"
PI_STEP_TAP_DIR = "/tmp/serval-step-tap"
DISPATCH_RECORD_ENV = "SERVAL_DISPATCH_RECORD_DIR"
PI_DISPATCH_RECORD_DIR = "/tmp/serval-dispatch-record"
RECORDING_DROPIN = f"{throttle.DROPIN_DIR}/bench-recording.conf"

TERMINAL_STATES = {"complete", "error", "cancelled"}
ACTIVE_STATES = {"printing", "paused"}
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
    "M18",
    "M84",
    "FORCE_MOVE",
    "SET_STEPPER_ENABLE",
    "SET_KINEMATIC_POSITION",
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


def mcu_step_benchmarks() -> list[dict]:
    """Every MCU's step benchmark the live planner caps moves by, as klippy
    reports it in the MCU's status: the analyzer plans its reference under
    the same cap."""
    objects = http("GET", "/printer/objects/list")["objects"]
    mcus = [
        name for name in objects if name == "mcu" or name.startswith("mcu ")
    ]
    if not mcus:
        raise BenchError(f"klippy reports no mcu objects: {objects}")
    query = "&".join(urllib.parse.quote(name) for name in mcus)
    status = http("GET", f"/printer/objects/query?{query}")["status"]
    return [
        status[name]["step_capacity"]
        for name in mcus
        if "step_capacity" in status[name]
    ]


def poll_print(capture: Capture, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        capture.check()
        state = printer_state()
        if state != "ready":
            raise BenchError(f"klippy entered state {state!r} during the print")
        stats = print_stats()
        if stats["state"] in TERMINAL_STATES:
            return stats
        time.sleep(POLL_S)
    raise BenchError(f"print did not finish within {timeout_s}s")


def cancel_active_print() -> None:
    if printer_state() == "ready" and print_stats()["state"] in ACTIVE_STATES:
        http("POST", "/printer/print/cancel")


def recordings(tap: bool, dispatch_record: bool) -> dict[str, str]:
    """The host recordings a run asks for: each env var the live host reads
    and the empty Pi directory it writes into."""
    wanted = {
        STEP_TAP_ENV: (tap, PI_STEP_TAP_DIR),
        DISPATCH_RECORD_ENV: (dispatch_record, PI_DISPATCH_RECORD_DIR),
    }
    return {env: path for env, (on, path) in wanted.items() if on}


@contextlib.contextmanager
def recording_session(env: dict[str, str]) -> Iterator[None]:
    if not env:
        yield
        return
    dirs = " ".join(env.values())
    lines = "".join(f"Environment={k}={v}\\n" for k, v in env.items())
    ssh(
        f"rm -rf {dirs} && mkdir -p {dirs} && "
        f"sudo mkdir -p {throttle.DROPIN_DIR} && printf "
        f"'[Service]\\n{lines}' "
        f"| sudo tee {RECORDING_DROPIN} >/dev/null && sudo systemctl daemon-reload"
    )
    restart_klipper()
    try:
        yield
    finally:
        ssh(f"sudo rm -f {RECORDING_DROPIN} && sudo systemctl daemon-reload")
        restart_klipper()


def run(
    gcode: Path,
    runs_dir: Path,
    timeout_s: float,
    config: Path = REPO_CFG,
    host_throttle: throttle.Throttle | None = None,
    events: bool = False,
    tap: bool = False,
    dispatch_record: bool = True,
) -> Path:
    if tap and not events:
        raise BenchError("--step-tap is only analyzable with --events")
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
        "step_benchmarks": mcu_step_benchmarks(),
    }
    with (
        throttle.throttled(host_throttle) as applied_throttle,
        recording_session(recordings(tap, dispatch_record)),
    ):
        summary["host_throttle"] = applied_throttle
        summary["step_tap"] = tap
        summary["dispatch_record"] = dispatch_record
        execute(
            run_dir,
            remote_name,
            summary,
            timeout_s,
            events,
            tap,
            dispatch_record,
        )
    return run_dir


def execute(
    run_dir: Path,
    remote_name: str,
    summary: dict,
    timeout_s: float,
    events: bool,
    tap: bool,
    dispatch_record: bool,
) -> None:
    capture = Capture(run_dir, both_edges_mask=0, events=events)
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
        summary[f"capture_{result.unit}"] = result.units
    except BaseException as exc:
        failure = failure or exc
    if failure is not None:
        try:
            cancel_active_print()
        except Exception as exc:
            failure.add_note(f"cancelling the print also failed: {exc!r}")
    ended = time.time()

    scp_from_pi(f"{PI_LOGS}/klippy.log", run_dir / "klippy.log")
    scp_from_pi(f"{PI_LOGS}/events", run_dir / "events", recursive=True)
    if tap:
        scp_from_pi(f"{PI_STEP_TAP_DIR}/step_tap_mcu*.bin", run_dir)
    if dispatch_record:
        scp_from_pi(f"{PI_DISPATCH_RECORD_DIR}/dispatch_record_*.bin", run_dir)
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
    parser.add_argument(
        "--events",
        action="store_true",
        help="capture in event mode (timestamp every edge)",
    )
    parser.add_argument(
        "--step-tap",
        action="store_true",
        help="record the host's commanded step frames (needs --events)",
    )
    parser.add_argument(
        "--dispatch-record",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="record every segment the host dispatched, the reference "
        "analyze.py judges a run the pacer braked against",
    )
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
                args.events,
                args.step_tap,
                args.dispatch_record,
            )
        )


if __name__ == "__main__":
    main()
