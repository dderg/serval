"""One live bench run: print a program on the Pi while the F407 captures.

The klipper service must already run the tree under test (bench.activate).
Per run: restart klippy with a fresh log (a fresh process also has no step
history from earlier runs for the tap to replay), start the Pi recorder
(CPU samples, optional queue_step tap), start the event-mode capture,
print, then fetch everything into the run dir.
"""

from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

from . import bench
from .program import Program
from .serval_tools import capture as serval_capture

STEPPERS = ("stepper_x", "stepper_y", "stepper_z", "extruder")
TERMINAL_STATES = {"complete", "error", "cancelled"}
POLL_S = 0.5
AGENT_READY_TIMEOUT_S = 20.0


def start_agent(remote_dir: str, tap: bool) -> None:
    pid = bench.pi.klipper_pid()
    steppers = f"--steppers {','.join(STEPPERS)}" if tap else ""
    bench.ssh(
        f"rm -rf {remote_dir} && mkdir -p {remote_dir} && "
        f"nohup python3 {bench.PI_AGENT} {remote_dir} --pid {pid} {steppers} "
        f"> {remote_dir}/agent.log 2>&1 < /dev/null & echo $! > {remote_dir}/agent.pid"
    )
    deadline = time.monotonic() + AGENT_READY_TIMEOUT_S
    while (
        bench.ssh(f"cat {remote_dir}/ready 2>/dev/null || true").strip() != "ok"
    ):
        if time.monotonic() > deadline:
            log = bench.ssh(f"cat {remote_dir}/agent.log || true")
            raise bench.BenchError(f"pi agent not ready: {log}")
        time.sleep(0.2)


def stop_agent(remote_dir: str) -> None:
    bench.ssh(
        f"pid=$(cat {remote_dir}/agent.pid); kill -TERM $pid; "
        f"while kill -0 $pid 2>/dev/null; do sleep 0.1; done"
    )


def pi_monotonic() -> float:
    """klippy's reactor clock (CLOCK_MONOTONIC_RAW, chelper get_monotonic)."""
    return float(
        bench.ssh(
            "python3 -c 'import time; "
            "print(time.clock_gettime(time.CLOCK_MONOTONIC_RAW))'"
        )
    )


def run(
    program: Program,
    config: Path,
    runs_dir: Path,
    label: str,
    tap: bool,
    timeout_s: float = 1800.0,
    extra: dict | None = None,
) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = runs_dir / f"{stamp}-{label}"
    run_dir.mkdir(parents=True)
    (run_dir / "program.gcode").write_text(program.text())
    remote_name = f"kv2bench_{stamp}.gcode"
    remote_dir = f"{bench.PI_KV2}/runs/{stamp}"
    summary: dict = {
        "label": label,
        "program": program.name,
        "tap": tap,
        "klipper_cmdline": None,
        **(extra or {}),
    }
    bench.push_config(config)
    (run_dir / "printer.cfg").write_text(config.read_text())
    bench.ssh(f": > {bench.PI_LOG}")
    summary["settled_after_restart"] = bench.restart_ready()
    summary["klipper_cmdline"] = bench.klipper_cmdline()
    summary["mcu_version"] = bench.mcu_version()
    bench.upload(remote_name, program.text())
    start_agent(remote_dir, tap)
    capture = serval_capture.Capture(run_dir, both_edges_mask=0, events=True)
    failure: BaseException | None = None
    final: dict = {}
    capture.start()
    try:
        time.sleep(1.0)
        summary["print_start_pi_monotonic"] = pi_monotonic()
        bench.start_print(remote_name)
        deadline = time.monotonic() + timeout_s
        while True:
            capture.check()
            state = bench.pi.printer_state()
            if state != "ready":
                raise bench.BenchError(f"klippy entered state {state!r}")
            final = bench.print_stats()
            if final["state"] in TERMINAL_STATES:
                break
            if time.monotonic() > deadline:
                raise bench.BenchError(f"print exceeded {timeout_s} s")
            time.sleep(POLL_S)
        summary["print_end_pi_monotonic"] = pi_monotonic()
        time.sleep(1.0)
    except BaseException as exc:
        failure = exc
    try:
        result = capture.stop()
        summary["capture_bytes"] = result.units
    except BaseException as exc:
        failure = failure or exc
    try:
        stop_agent(remote_dir)
    except BaseException as exc:
        failure = failure or exc
    if failure is not None:
        try:
            if bench.pi.printer_state() == "ready":
                bench.pi.http("POST", "/printer/print/cancel")
        except Exception as exc:
            failure.add_note(f"cancelling the print also failed: {exc!r}")
    bench.pi.scp_from_pi(bench.PI_LOG, run_dir / "klippy.log")
    for name in ("cpu.jsonl", "tap.jsonl", "agent.json", "agent.log"):
        if bench.ssh(f"test -e {remote_dir}/{name} && echo y || true").strip():
            bench.pi.scp_from_pi(f"{remote_dir}/{name}", run_dir / name)
    bench.remove_upload(remote_name)
    bench.ssh(f"rm -rf {remote_dir}")
    summary.update(
        print_state=final.get("state"),
        print_message=final.get("message"),
        print_duration_s=final.get("print_duration"),
        failure=None if failure is None else repr(failure),
    )
    (run_dir / "run.json").write_text(json.dumps(summary, indent=2))
    agent = run_dir / "agent.json"
    if agent.exists() and json.loads(agent.read_text())["failures"]:
        failure = failure or bench.BenchError(
            f"pi agent failed: {agent.read_text()}"
        )
    if failure is not None:
        failure.add_note(f"run dir {run_dir}")
        raise failure
    if final.get("state") != "complete":
        raise bench.BenchError(
            f"print ended {final.get('state')!r}: {final.get('message')} ({run_dir})"
        )
    return run_dir
