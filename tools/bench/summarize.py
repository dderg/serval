"""One line per bench run: verdict, timing, epochs, deployed identity.

uv run python tools/bench/summarize.py <run_dir|glob>... [--json]
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

MOTORS = ("a", "b", "z", "e")


def expand(patterns: list[str]) -> list[Path]:
    paths = []
    for pattern in patterns:
        matches = sorted(glob.glob(str(Path(pattern).expanduser())))
        if not matches:
            sys.exit(f"no run dir matches {pattern!r}")
        paths.extend(Path(m) for m in matches)
    return paths


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def motor_timing(analysis: dict) -> dict:
    out = {}
    for role in MOTORS:
        stats = analysis["motors"][role][
            "timing_error_capture_minus_reference_us"
        ]
        out[role] = (
            {"abs_max": stats["abs_max"], "abs_p99": stats["abs_p99"]}
            if stats
            else None
        )
    return out


def summarize(run_dir: Path) -> dict:
    info = read_json(run_dir / "run.json")
    analysis = read_json(run_dir / "analysis.json")
    deployed = info.get("deployed") or {}
    row = {
        "run": run_dir.name,
        "print_state": info.get("print_state"),
        "duration_s": info.get("duration_s"),
        "config": Path(info["config"]).name if info.get("config") else None,
        "throttle": info.get("host_throttle") or None,
        "deployed": {
            "head": (deployed.get("head") or "")[:9],
            "branch": deployed.get("branch"),
            "dirty": len(deployed.get("dirty_paths") or []),
        },
        "analyzed": bool(analysis),
    }
    if not analysis:
        return row
    row["ok"] = analysis["ok"]
    row["failed_checks"] = [k for k, v in analysis["checks"].items() if not v]
    row["stalls"] = analysis["stalls"]["count"]
    row["timing_us"] = motor_timing(analysis)
    row["epochs"] = [
        {
            "span_delta_ms": e.get("conditioned_span_capture_minus_stream_ms"),
            "ppm": e["ppm"],
        }
        for e in analysis["epochs"]
    ]
    execution = analysis.get("mcu_execution")
    if execution:
        row["mcu_lateness_ns"] = execution["lateness_ns"]
    return row


def fmt_timing(row: dict) -> str:
    cells = []
    for role in MOTORS:
        stats = row["timing_us"][role]
        cells.append(
            f"{role}:{stats['abs_max']:.0f}/{stats['abs_p99']:.0f}"
            if stats
            else f"{role}:-"
        )
    return " ".join(cells)


def fmt_epochs(row: dict) -> str:
    epochs = row["epochs"]
    if not epochs:
        return "-"
    deltas = [
        e["span_delta_ms"] for e in epochs if e["span_delta_ms"] is not None
    ]
    ppms = [e["ppm"] for e in epochs]
    span = f"{min(deltas):+.1f}..{max(deltas):+.1f}ms" if deltas else "-"
    return f"{len(epochs)}ep span {span} ppm {min(ppms):.0f}..{max(ppms):.0f}"


def format_line(row: dict) -> str:
    deployed = row["deployed"]
    identity = (
        f"{deployed['head']}{'+' + str(deployed['dirty']) if deployed['dirty'] else ''}"
        if deployed["head"]
        else "-"
    )
    head = (
        f"{row['run']:<18} {str(row['print_state']):<9} "
        f"{(row['duration_s'] or 0):6.1f}s {identity:<12}"
    )
    if not row["analyzed"]:
        return f"{head} not analyzed"
    verdict = "ok" if row["ok"] else "FAIL " + ",".join(row["failed_checks"])
    late = row.get("mcu_lateness_ns")
    lateness = f" mcu_late_p99={late['p99']:.0f}ns" if late else ""
    return (
        f"{head} {verdict:<8} stalls={row['stalls']} "
        f"abs_max/p99us {fmt_timing(row)} {fmt_epochs(row)}{lateness}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    rows = [summarize(path) for path in expand(args.runs)]
    if args.json:
        print(json.dumps(rows, indent=2))
    else:
        print("\n".join(format_line(row) for row in rows))


if __name__ == "__main__":
    main()
