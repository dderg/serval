"""Summarize a `--gates trials` session: failure rate, lateness, stall stacks.

uv run python -m tools.kv2_bench.trials_report ~/bench/kv2/<stamp>
"""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

import numpy as np

STALL_BEFORE_SHUTDOWN_S = 2.0
STACK_LINES = 8
PYSPY_LINES = 40


def run_window(run_dir: Path, probe: dict) -> tuple[float | None, float | None]:
    run = json.loads((run_dir / "run.json").read_text())
    start = run.get("print_start_pi_monotonic")
    end = run.get("print_end_pi_monotonic") or probe["events"].get(
        "klippy:shutdown"
    )
    return start, end


def table_build(build: dict, probe: dict) -> dict:
    """A corner table build: duration, publish time against klippy:ready,
    and the reactor's worst timer lateness while the build thread ran."""
    published = build.get("published")
    ready = probe["events"].get("klippy:ready")
    during = [
        late
        for at, late in probe["late_samples"]
        if published is not None and build["start"] <= at <= published
    ]
    return {
        "cache_files_before": build.get("cache_files_before"),
        "build_s": None if published is None else published - build["start"],
        "published_after_ready_s": None
        if published is None or ready is None
        else published - ready,
        "max_late_during_build_ms": 1e3 * max(during, default=0.0),
        "error": build.get("error"),
    }


def summarize_run(record: dict) -> dict:
    run_dir = Path(record["run_dir"])
    probe = json.loads((run_dir / "probe.json").read_text())
    start, end = run_window(run_dir, probe)
    inside = [
        late
        for at, late in probe["late_samples"]
        if start is not None and start <= at <= (end or float("inf"))
    ]
    out = {
        "run_dir": str(run_dir),
        "failed": "error" in record,
        "error": record.get("error"),
        "fatal_lines": record.get("fatal_lines", [])[:3],
        "max_late_ms": 1e3 * max(inside, default=0.0),
        "late_ms": [1e3 * v for v in inside],
        "late_events_states": collections.Counter(
            (e["toolhead"] or {}).get("queuing_state", "?")
            for e in probe["late_events"]
            if start is not None and e["at"] >= start
        ),
        "pyspy_native": probe.get("pyspy_native"),
        "table_builds": [
            table_build(build, probe) for build in probe.get("table_builds", [])
        ],
    }
    shutdown = probe["events"].get("klippy:shutdown")
    if shutdown is not None and start is not None:
        out["shutdown_since_print_start_s"] = shutdown - start
        stalls = [
            s
            for s in probe["stalls"]
            if shutdown - STALL_BEFORE_SHUTDOWN_S <= s["at"] <= shutdown
        ]
        out["stalls_before_shutdown"] = [
            {
                "since_print_start_s": round(s["at"] - start, 3),
                "max_overdue_ms": round(1e3 * s["max_overdue_s"], 1),
                "overdue_timer": s["overdue_timer"],
                "toolhead": s["toolhead"],
                "main_stack": [
                    line.strip() for line in s["main_stack"][-STACK_LINES:]
                ],
                "pyspy": (s.get("pyspy") or "").splitlines()[:PYSPY_LINES],
            }
            for s in stalls
        ]
    return out


def summarize(session: Path) -> dict:
    records = [
        r
        for r in json.loads((session / "session.json").read_text())["runs"]
        if r["gate"] == "trials" and r.get("run_dir")
    ]
    by_config: dict[str, list[dict]] = collections.defaultdict(list)
    for record in records:
        cache = record["host"].get("table_cache")
        key = record["config"] + (f" ({cache} table cache)" if cache else "")
        by_config[key].append(summarize_run(record))
    result = {}
    for config, runs in by_config.items():
        pooled = np.array([v for r in runs for v in r["late_ms"]] or [0.0])
        maxima = np.array([r["max_late_ms"] for r in runs])
        states = sum(
            (r["late_events_states"] for r in runs), collections.Counter()
        )
        result[config] = {
            "runs": len(runs),
            "failures": sum(r["failed"] for r in runs),
            "failure_rate": sum(r["failed"] for r in runs) / len(runs),
            "per_run_max_late_ms": {
                "p50": float(np.percentile(maxima, 50)),
                "p90": float(np.percentile(maxima, 90)),
                "max": float(maxima.max()),
            },
            "pooled_late_ge_2ms": {
                "count": int(len(pooled)),
                "p50_ms": float(np.percentile(pooled, 50)),
                "p99_ms": float(np.percentile(pooled, 99)),
                "max_ms": float(pooled.max()),
            },
            "late_ge_20ms_by_toolhead_state": dict(states),
            "failed_runs": [
                {k: v for k, v in r.items() if k not in ("late_ms",)}
                for r in runs
                if r["failed"]
            ],
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    result = summarize(args.session.expanduser())
    (args.session.expanduser() / "trials.json").write_text(
        json.dumps(result, indent=2, default=dict)
    )
    print(json.dumps(result, indent=2, default=dict))


if __name__ == "__main__":
    main()
