"""Real-time headroom of a live run: klippy CPU, buffer_time, MCU errors.

CPU comes from the Pi recorder's /proc samples (whole process and per
thread) over the print window; buffer_time and print_stall from klippy's
1 Hz `Stats` lines. buffer_time is print_time minus the MCU's estimated
print time: how far ahead the host has planned. It is judged only while the
host is still appending moves (print_time grew since the previous line),
since it drains to zero by design once the program's last move is planned.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

FATAL_PATTERNS = (
    "Timer too close",
    "Stepper too far in past",
    "Rescheduled timer in the past",
    "Lost communication with MCU",
    "MCU 'mcu' shutdown",
    "Transition to shutdown state",
)
STATS_LINE = re.compile(r"^Stats (\d+\.\d+): (.*)$")
PEAK_WINDOW_S = 1.0


def parse_stats(log: str) -> list[dict]:
    rows = []
    for line in log.splitlines():
        match = STATS_LINE.match(line)
        if match is None:
            continue
        row: dict = {"eventtime": float(match.group(1))}
        section = ""
        for token in match.group(2).split():
            if token.endswith(":"):
                section = token[:-1] + "."
                continue
            key, _, value = token.partition("=")
            try:
                parsed: float | str = float(value)
            except ValueError:
                parsed = value
            row.setdefault(section + key, parsed)
            row.setdefault(key, parsed)
        rows.append(row)
    return rows


def fatal_lines(log: str) -> list[str]:
    return [
        line
        for line in log.splitlines()
        if any(pattern in line for pattern in FATAL_PATTERNS)
    ]


def cpu_usage(path: Path, start: float, end: float) -> dict:
    lines = path.read_text().splitlines()
    clk_tck = json.loads(lines[0])["clk_tck"]
    samples = [json.loads(line) for line in lines[1:]]
    inside = [s for s in samples if start <= s["mono"] <= end]
    if len(inside) < 2:
        raise ValueError(f"{len(inside)} CPU samples inside the print window")
    t = np.array([s["mono"] for s in inside])
    busy = np.array([s["utime"] + s["stime"] for s in inside]) / clk_tck
    mean = (busy[-1] - busy[0]) / (t[-1] - t[0])
    window = max(1, int(round(PEAK_WINDOW_S / np.median(np.diff(t)))))
    peak = (
        float(
            np.max(
                (busy[window:] - busy[:-window]) / (t[window:] - t[:-window])
            )
        )
        if len(t) > window
        else mean
    )
    first, last = inside[0]["tasks"], inside[-1]["tasks"]
    threads: dict[str, float] = {}
    for tid, (comm, utime, stime) in last.items():
        before = first.get(tid, (comm, 0, 0))
        spent = (utime + stime - before[1] - before[2]) / clk_tck
        threads[comm] = threads.get(comm, 0.0) + spent / (t[-1] - t[0])
    return {
        "window_s": float(t[-1] - t[0]),
        "process_mean_pct": 100.0 * mean,
        "process_peak_1s_pct": 100.0 * peak,
        "threads_mean_pct": {
            name: round(100.0 * share, 2)
            for name, share in sorted(threads.items(), key=lambda kv: -kv[1])
        },
        "loadavg_max": max(s["loadavg"] for s in inside),
    }


def buffer_while_feeding(stats: list[dict], start: float, end: float) -> dict:
    before = [r for r in stats if r["eventtime"] < start][-1:]
    rows = before + [r for r in stats if start <= r["eventtime"] <= end]
    feeding = [
        cur
        for prev, cur in zip(rows, rows[1:])
        if cur["print_time"] > prev["print_time"]
    ]
    if not feeding:
        raise ValueError("no Stats line shows the host appending moves")
    first_stall = rows[0]["print_stall"] if rows else 0
    return {
        "stats_lines": len(rows),
        "feeding_lines": len(feeding),
        "buffer_time_min_s": min(r["buffer_time"] for r in feeding),
        "buffer_time_median_s": float(
            np.median([r["buffer_time"] for r in feeding])
        ),
        "print_stall_delta": int(rows[-1]["print_stall"] - first_stall),
        "mcu_task_avg_max_s": max(r.get("mcu.mcu_task_avg", 0.0) for r in rows),
        "bytes_retransmit_max": max(
            r.get("mcu.bytes_retransmit", 0.0) for r in rows
        ),
    }


def analyze(run_dir: Path) -> dict:
    run = json.loads((run_dir / "run.json").read_text())
    log = (run_dir / "klippy.log").read_text()
    start = run["print_start_pi_monotonic"]
    end = run["print_end_pi_monotonic"]
    result = {
        "fatal_lines": fatal_lines(log),
        "cpu": cpu_usage(run_dir / "cpu.jsonl", start, end),
        "buffer": buffer_while_feeding(parse_stats(log), start, end),
    }
    (run_dir / "headroom.json").write_text(json.dumps(result, indent=2))
    return result
