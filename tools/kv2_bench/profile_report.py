"""Summarize the probe (klippy_probe.py) and py-spy output of probed runs.

uv run python -m tools.kv2_bench.profile_report RUN_DIR [...]
"""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

from . import headroom

TOP = 8


def pyspy_top(path: Path) -> dict:
    self_samples: collections.Counter = collections.Counter()
    total = 0
    for line in path.read_text().splitlines():
        stack, _, count = line.rpartition(" ")
        if not stack:
            continue
        samples = int(count)
        total += samples
        self_samples[stack.split(";")[-1]] += samples
    return {
        "samples": total,
        "self": [
            {"frame": frame, "share": n / total}
            for frame, n in self_samples.most_common(TOP)
        ],
    }


def summarize(run_dir: Path) -> dict:
    run = json.loads((run_dir / "run.json").read_text())
    probe = json.loads((run_dir / "probe.json").read_text())
    events = probe["events"]
    start = run.get("print_start_pi_monotonic")
    ready = events.get("klippy:ready")
    out = {
        "run": run_dir.name,
        "host": run["record"]["host"],
        "config": run["record"]["config"],
        "print_state": run.get("print_state"),
        "failure": run.get("failure"),
        "fatal_lines": headroom.fatal_lines(
            (run_dir / "klippy.log").read_text()
        )[:4],
        "start_to_ready_s": None
        if ready is None
        else ready - probe["process_start"],
        "table_builds_s": [
            {
                "duration": b["end"] - b["start"],
                "ended_before_ready": ready is not None and b["end"] <= ready,
                "ended_before_print_start_s": None
                if start is None
                else start - b["end"],
            }
            for b in probe["table_builds"]
        ],
        "max_timer_late_after_ready_s": max(
            (t["late_s"] for t in probe["late_timers_ready"]), default=None
        ),
        "late_timers_after_ready": [
            {
                "callback": t["callback"],
                "late_s": round(t["late_s"], 4),
                "since_print_start_s": None
                if start is None
                else round(t["at"] - start, 2),
            }
            for t in probe["late_timers_ready"][:5]
        ],
        "long_callbacks_after_ready": [
            {
                "callback": c["callback"],
                "wall_s": round(c["wall_s"], 4),
                "since_print_start_s": None
                if start is None
                else round(c["at"] - start, 2),
            }
            for c in probe["long_callbacks_ready"][:5]
        ],
        "shutdown_since_print_start_s": None
        if start is None or "klippy:shutdown" not in events
        else events["klippy:shutdown"] - start,
    }
    if (run_dir / "pyspy.txt").exists():
        out["pyspy"] = pyspy_top(run_dir / "pyspy.txt")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", type=Path, nargs="+")
    args = parser.parse_args()
    for run_dir in args.run_dirs:
        print(json.dumps(summarize(run_dir), indent=1))


if __name__ == "__main__":
    main()
