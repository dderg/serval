"""Triage a bench run's structured events and klippy Stats trend.

uv run python tools/bench/events.py <run_dir> [--around T] [--window S]
    [--names a,b] [--tail N]

events/*.jsonl hold the Pi's whole history, so records are cut to the run's
host_start..host_end window. T is seconds since the run started.
"""

from __future__ import annotations

import argparse
import collections
import json
import re
from datetime import datetime
from pathlib import Path

MARGIN_S = 5.0
ERROR_LEVELS = ("error", "fatal")
STATS = re.compile(
    r"^Stats [\d.]+:.*?sysload=(?P<sysload>[\d.]+).*?"
    r"print_time=(?P<print_time>[\d.]+) buffer_time=(?P<buffer_time>-?[\d.]+)"
)
RESPONSE_NAME = re.compile(r"'name': '(\w+)'")


def run_window(run_dir: Path) -> tuple[float, float]:
    info = json.loads((run_dir / "run.json").read_text())
    start = datetime.fromisoformat(info["host_start"]).timestamp()
    end = datetime.fromisoformat(info["host_end"]).timestamp()
    return start, end


def record_time(record: dict) -> float:
    return datetime.fromisoformat(
        record["_time"].replace("Z", "+00:00")
    ).timestamp()


def load_records(run_dir: Path, start: float, end: float) -> list[dict]:
    records = []
    for path in sorted((run_dir / "events").glob("*.jsonl")):
        for line in path.read_text(errors="replace").splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue
            when = record_time(record)
            if start - MARGIN_S <= when <= end + MARGIN_S:
                record["_t"] = when - start
                records.append(record)
    return sorted(records, key=lambda r: r["_t"])


def event_name(record: dict) -> str:
    named = record.get("event")
    if named:
        return named
    response = RESPONSE_NAME.search(record["_msg"])
    if response:
        return f"response:{response.group(1)}"
    return re.sub(r"\d+", "N", record["_msg"].splitlines()[0])[:70]


def count_table(records: list[dict]) -> list[tuple]:
    counts = collections.Counter(
        (r["source"], r["level"], event_name(r)) for r in records
    )
    return sorted(counts.items(), key=lambda item: (-item[1], item[0]))


def stats_trend(run_dir: Path) -> list[dict]:
    rows = []
    for line in (
        (run_dir / "klippy.log").read_text(errors="replace").splitlines()
    ):
        match = STATS.match(line)
        if match:
            rows.append({k: float(v) for k, v in match.groupdict().items()})
    return rows


def print_counts(records: list[dict], names: set[str]) -> None:
    print("count  source     level  event")
    for (source, level, name), count in count_table(records):
        if not names or name in names:
            print(f"{count:<6} {source:<10} {level:<6} {name}")


def print_errors(records: list[dict]) -> None:
    errors = [r for r in records if r["level"] in ERROR_LEVELS]
    print(f"\n{len(errors)} error/fatal record(s)")
    for record in errors:
        print(f"  t={record['_t']:+.3f}s [{record['source']}] {record['_msg']}")
        extra = record.get("error")
        if extra:
            print(f"    error: {extra}")


def print_around(records: list[dict], around: float, window: float) -> None:
    print(f"\nrecords within {window:g}s of t={around:g}s")
    for record in records:
        if abs(record["_t"] - around) <= window:
            print(
                f"  t={record['_t']:+.3f}s {record['level']:<5} "
                f"[{record['source']}] {event_name(record)}: "
                f"{record['_msg'][:140]}"
            )


def print_trend(stats: list[dict], tail: int) -> None:
    print(f"\nklippy Stats trend ({len(stats)} lines); last {tail}:")
    print("print_time buffer_time sysload")
    for row in stats[-tail:]:
        print(
            f"{row['print_time']:>10.3f} {row['buffer_time']:>11.3f} "
            f"{row['sysload']:>7.2f}"
        )
    if stats:
        worst = min(stats, key=lambda r: r["buffer_time"])
        print(
            f"min buffer_time {worst['buffer_time']:.3f} at print_time "
            f"{worst['print_time']:.3f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--around", type=float)
    parser.add_argument("--window", type=float, default=2.0)
    parser.add_argument("--names", default="")
    parser.add_argument("--tail", type=int, default=10)
    args = parser.parse_args()
    start, end = run_window(args.run_dir)
    records = load_records(args.run_dir, start, end)
    names = {n for n in args.names.split(",") if n}
    print(f"{args.run_dir.name}: {len(records)} records in window")
    print_counts(records, names)
    print_errors(records)
    if args.around is not None:
        print_around(records, args.around, args.window)
    print_trend(stats_trend(args.run_dir), args.tail)


if __name__ == "__main__":
    main()
